local KEY, PREFIX = 93, "QYAW_"
assert(param:add_table(KEY, PREFIX, 11), "QYAW: parameter table unavailable")

local function parameter(index, name, default)
    assert(param:add_param(KEY, index, name, default), "QYAW: parameter " .. name)
    return Parameter(PREFIX .. name)
end

local ENABLE = parameter(1, "ENABLE", 0)
local SYSID = parameter(2, "SYSID", 255)
local COMPID = parameter(3, "COMPID", 191)
local MIN_HGT = parameter(4, "MIN_HGT", 4)
local DIST_MAX = parameter(5, "DIST_MAX", 2)
local MAX_AGE = parameter(6, "MAX_AGE", 0.4)
local STABLE = parameter(7, "STABLE", 0.5)
local TOL = parameter(8, "TOL", 3)
local RATE_MAX = parameter(9, "RATE_MAX", 20)
local GAIN = parameter(10, "GAIN", 1)
local FORWARD = parameter(11, "FORWARD", 0)

local supported, api = pcall(function()
    return quadplane.set_landing_yaw_rate and quadplane.clear_landing_yaw_rate
end)
assert(supported and api, "QYAW: firmware lacks landing yaw API")
mavlink:init(10, 1)
mavlink:register_rx_msgid(149)

local target, stable_since, last_capture, last_received
local transport_delay_ms = 0
local correcting = false
local state

local function finite(x)
    return x == x and math.abs(x) < math.huge
end

local function set_state(value)
    if state ~= value then
        state = value
        gcs:send_text(6, "QYAW: " .. value)
    end
end

local function release(reason)
    quadplane:clear_landing_yaw_rate()
    stable_since = nil
    correcting = false
    set_state(reason)
end

local function valid_parameters()
    return ENABLE:get() == 1 and SYSID:get() >= 1 and SYSID:get() <= 255
        and COMPID:get() >= 1 and COMPID:get() <= 255
        and MIN_HGT:get() >= 1 and finite(DIST_MAX:get()) and DIST_MAX:get() > 0
        and MAX_AGE:get() >= 0.05 and MAX_AGE:get() <= 0.5
        and STABLE:get() >= 0.1 and STABLE:get() <= 5
        and TOL:get() >= 1 and TOL:get() <= 20
        and RATE_MAX:get() > 0 and RATE_MAX:get() <= 45
        and GAIN:get() > 0 and GAIN:get() <= 5 and finite(FORWARD:get())
end

local function decode(raw)
    -- receive_chan supplies mavlink_message_t, with the payload at byte 13.
    -- ArduPilot has already checked the MAVLink checksum/signature on reception.
    if #raw < 72 or string.byte(raw, 3) ~= 0xFD or string.byte(raw, 4) < 60 then
        return nil
    end
    -- ArduPilot Lua has 32-bit integers/floats: keep the uint64 capture time in four exact words.
    local a, b, c, d, _, _, distance, _, _, number, frame, x, y, z, w, qx, qy, qz, kind, valid =
        string.unpack("<I2I2I2I2fffffBBfffffffBB", raw, 13)
    local norm = w*w + qx*qx + qy*qy + qz*qz
    if (a == 0 and b == 0 and c == 0 and d == 0) or number ~= 0 or frame ~= 12 or kind ~= 2 or valid ~= 1
        or not finite(x) or not finite(y) or not finite(z) or z <= 0
        or not finite(distance) or distance <= 0 or not finite(norm)
        or norm < 0.9 or norm > 1.1 then
        return nil
    end
    norm = math.sqrt(norm)
    return {stamp = {a,b,c,d}, x = x, y = y, z = z, w = w/norm, qx = qx/norm, qy = qy/norm, qz = qz/norm}
end

local function newer(a, b)
    if not b then return true end
    for i = 4, 1, -1 do
        if a[i] ~= b[i] then return a[i] > b[i] end
    end
    return false
end

local function capture_delta_ms(a, b)
    local delta, borrow = {}, 0
    for i = 1, 4 do
        local value = a[i] - b[i] - borrow
        borrow = value < 0 and 1 or 0
        delta[i] = value + borrow*65536
    end
    if delta[3] ~= 0 or delta[4] ~= 0 then return math.huge end
    return (delta[2]*65536.0 + delta[1])/1000.0
end

local function receive(now)
    for _ = 1, 10 do
        local raw, _, received_ms = mavlink:receive_chan()
        if not raw then break end
        if #raw >= 12 and string.byte(raw, 8) == SYSID:get() and string.byte(raw, 9) == COMPID:get()
            and string.unpack("<I3", raw, 10) == 149 then
            local sample = decode(raw)
            local age = (now - received_ms):toint()
            if not sample or age < 0 or age > MAX_AGE:get()*1000 then
                target, stable_since = nil, nil
            elseif newer(sample.stamp, last_capture) then
                local gap = last_capture and capture_delta_ms(sample.stamp, last_capture) or math.huge
                if last_received then
                    transport_delay_ms = math.max(0, transport_delay_ms + (received_ms-last_received):toint() - gap)
                end
                -- Bound added transport delay without converting either clock to floating point.
                if transport_delay_ms > MAX_AGE:get()*1000 then
                    target, stable_since = nil, nil
                else
                    if not target or gap > MAX_AGE:get()*1000 then
                        stable_since = nil
                    end
                    sample.received = received_ms
                    target = sample
                end
                last_capture = sample.stamp
                last_received = received_ms
            end
        end
    end
end

local function heading_error(sample)
    local roll, pitch = ahrs:get_roll_rad(), ahrs:get_pitch_rad()
    if not finite(roll) or not finite(pitch) then return nil end
    local cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    local north = cp*sample.x + sp*sr*sample.y + sp*cr*sample.z
    local east = cr*sample.y - sr*sample.z
    local down = -sp*sample.x + cp*sr*sample.y + cp*cr*sample.z
    if down < MIN_HGT:get() or north*north + east*east > DIST_MAX:get()^2 then return nil end

    local w, x, y, z = sample.w, sample.qx, sample.qy, sample.qz
    local offset = math.rad(FORWARD:get())
    local c, s = math.cos(offset), math.sin(offset)
    local vx = c*(1 - 2*(y*y + z*z)) + s*2*(x*y - w*z)
    local vy = c*2*(x*y + w*z) + s*(1 - 2*(x*x + z*z))
    local vz = c*2*(x*z - w*y) + s*2*(y*z + w*x)
    local hx, hy = cp*vx + sp*sr*vy + sp*cr*vz, cr*vy - sr*vz
    if hx*hx + hy*hy < 0.04 then return nil end
    return math.deg(math.atan(hy, hx))
end

local function update()
    local now = millis()
    receive(now)

    if not valid_parameters() or vehicle:get_mode() ~= 20 or not arming:is_armed() then
        target, last_capture, last_received = nil, nil, nil
        transport_delay_ms = 0
        release("inactive")
        return
    end
    if not ahrs:healthy() or vehicle:has_ekf_failsafed() or battery:has_failsafed()
        or not rc:has_valid_input() or param:get("Q_WVANE_ENABLE") ~= 0 then
        release("vehicle unavailable")
        return
    end
    if not target or (now - target.received):toint() > MAX_AGE:get()*1000 then
        target = nil
        release("waiting for target")
        return
    end
    local error_deg = heading_error(target)
    if not error_deg then
        release("outside alignment envelope")
        return
    end
    stable_since = stable_since or now
    if (now - stable_since):toint() < STABLE:get()*1000 then
        quadplane:clear_landing_yaw_rate()
        set_state("acquiring target")
        return
    end
    if math.abs(error_deg) <= TOL:get() then
        correcting = false
    elseif math.abs(error_deg) > TOL:get() + 2 then
        correcting = true
    end
    local rate = correcting and math.max(-RATE_MAX:get(), math.min(RATE_MAX:get(), GAIN:get()*error_deg)) or 0
    if not quadplane:set_landing_yaw_rate(rate) then
        release("pilot or flight controller priority")
        return
    end
    set_state(correcting and "aligning during descent" or "aligned")
end

local function run()
    local ok, err = pcall(update)
    if not ok then
        quadplane:clear_landing_yaw_rate()
        target, stable_since = nil, nil
        correcting = false
        gcs:send_text(3, "QYAW: " .. tostring(err))
    end
    return run, ok and 50 or 1000
end

gcs:send_text(6, "QYAW: loaded; QLAND yaw controller")
return run, 50
