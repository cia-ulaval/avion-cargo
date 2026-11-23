local script = assert(arg[1])
local checks = 0

local function uint32(value)
    value = value & 0xffffffff
    return setmetatable({value = value, toint = function() return value end}, {
        __sub = function(a, b) return uint32(a.value - b.value) end,
    })
end

local function fixture(options)
    options = options or {}
    local f = {now = 1000, mode = 20, armed = true, healthy = true, allowed = true,
        queue = {}, rates = {}, messages = {}, clears = 0, values = {QYAW_ENABLE = 1, Q_WVANE_ENABLE = 0}}
    for key, value in pairs(options) do f.values[key] = value end
    param = {
        add_table = function() return true end,
        add_param = function(_, _, _, name, default)
            local key = "QYAW_" .. name
            if f.values[key] == nil then f.values[key] = default end
            return true
        end,
        get = function(_, name) return f.values[name] end,
    }
    Parameter = function(name) return {get = function() return f.values[name] end} end
    millis = function() return uint32(f.now) end
    mavlink = {
        init = function() end,
        register_rx_msgid = function(_, id) assert(id == 149) end,
        receive_chan = function()
            local packet = table.remove(f.queue, 1)
            if packet then return packet.raw, 0, uint32(packet.received) end
        end,
    }
    quadplane = {
        set_landing_yaw_rate = function(_, rate)
            if not f.allowed then return false end
            table.insert(f.rates, rate)
            return true
        end,
        clear_landing_yaw_rate = function() f.clears = f.clears + 1 end,
    }
    ahrs = {
        healthy = function() return f.healthy end,
        get_roll_rad = function() if f.crash then error("injected fault") end return f.roll or 0 end,
        get_pitch_rad = function() return f.pitch or 0 end,
    }
    vehicle = {get_mode = function() return f.mode end, has_ekf_failsafed = function() return f.ekf_failed end}
    arming = {is_armed = function() return f.armed end}
    battery = {has_failsafed = function() return f.battery_failed end}
    rc = {has_valid_input = function() return not f.rc_failed end}
    gcs = {send_text = function(_, _, message) table.insert(f.messages, message) end}
    f.run = assert(loadfile(script))()
    function f:send(degrees, fields)
        fields = fields or {}
        local angle = math.rad(degrees or 30)/2
        local q = fields.q or {math.cos(angle), 0, 0, math.sin(angle)}
        local stamp = fields.stamp or (self.now + 100000)*1000
        local payload = string.pack("<I8fffffBBfffffffBB", stamp, 0, 0, 6, 0, 0, 0,
            fields.frame or 12, fields.x or 0, fields.y or 0, fields.z or 6,
            q[1], q[2], q[3], q[4], 2, fields.valid or 1)
        local raw = string.pack("<HBBBBBBBI3", 0, fields.version or 0xFD, 60, 0, 0, 1,
            fields.sysid or 255, fields.compid or 191, 149) .. payload
        table.insert(self.queue, {raw = raw, received = fields.received or (self.now & 0xffffffff)})
    end
    function f:tick(degrees, fields)
        self.now = self.now + 50
        if degrees then self:send(degrees, fields) end
        local next_run = self.run()
        self.run = next_run
    end
    function f:acquire(degrees, fields)
        for _ = 1, 14 do self:tick(degrees or 30, fields) end
    end
    return f
end

local function check(name, test)
    test()
    checks = checks + 1
    print("PASS " .. name)
end

check("bounded clockwise and counterclockwise rates", function()
    for _, angle in ipairs({30, -30, 179, -179}) do
        local f = fixture()
        f:acquire(angle)
        assert(#f.rates > 0)
        assert(f.rates[#f.rates] == (angle > 0 and 20 or -20))
        f:tick(1)
        assert(f.rates[#f.rates] == 0)
    end
end)

check("target must be stable", function()
    local f = fixture()
    for _ = 1, 8 do f:tick(30) end
    assert(#f.rates == 0)
    f:acquire(30)
    assert(#f.rates > 0)
end)

check("loss and duplicate capture expire", function()
    for _, repeated in ipairs({false, true}) do
        local f = fixture()
        f:acquire()
        local count = #f.rates
        local stamp = (f.now + 100000)*1000
        for _ = 1, 10 do f:tick(repeated and 30 or nil, {stamp = stamp}) end
        local after = #f.rates
        f:tick(repeated and 30 or nil, {stamp = stamp})
        assert(#f.rates == after and after >= count)
        assert(f.messages[#f.messages] == "QYAW: waiting for target")
    end
end)

check("invalid geometry, orientation, frame and source cannot command yaw", function()
    for _, fields in ipairs({{z=3}, {x=3}, {q={0,0,0,0}}, {q={0/0,0,0,0}},
        {q={math.sqrt(0.5),0,math.sqrt(0.5),0}}, {frame=8}, {version=0xFE},
        {compid=1}, {sysid=1}, {valid=0}, {z=0/0}, {received=1}}) do
        local f = fixture()
        f:acquire(30, fields)
        assert(#f.rates == 0)
    end
end)

check("roll compensation and marker forward offset", function()
    local f = fixture({QYAW_FORWARD=90})
    f:acquire(-60)
    assert(f.rates[#f.rates] == 20)
    f = fixture()
    f.roll = math.rad(30)
    local c, s = math.cos(math.rad(15)), math.sin(math.rad(15))
    f:acquire(30, {y=3, z=6*math.cos(f.roll), q={c*c,-s*c,s*s,c*s}})
    assert(f.rates[#f.rates] == 20)
end)

check("mode, arm, failsafe, pilot and runtime fault release immediately", function()
    for _, condition in ipairs({"mode", "armed", "healthy", "allowed", "rc_failed", "ekf_failed", "battery_failed", "crash"}) do
        local f = fixture()
        f:acquire()
        local count, clears = #f.rates, f.clears
        if condition == "mode" then f.mode = 19
        elseif condition == "armed" or condition == "healthy" or condition == "allowed" then f[condition] = false
        else f[condition] = true end
        f:tick(30)
        assert(#f.rates == count and f.clears > clears, condition)
    end
end)

check("disabled and invalid parameters cannot command yaw", function()
    for _, options in ipairs({{QYAW_ENABLE=0}, {QYAW_RATE_MAX=90}, {QYAW_GAIN=0},
        {QYAW_MIN_HGT=0}, {QYAW_MAX_AGE=2}, {Q_WVANE_ENABLE=1}}) do
        local f = fixture(options)
        f:acquire()
        assert(#f.rates == 0)
    end
end)

check("backlog and timestamp rollback cannot refresh target", function()
    for _, backwards in ipairs({false, true}) do
        local f = fixture()
        f:acquire()
        local old_stamp = (f.now + 100000)*1000
        for _ = 1, 20 do f:tick() end
        local count = #f.rates
        f:acquire(30, {stamp=old_stamp + (backwards and -1 or 1)})
        assert(#f.rates == count)
    end
end)

check("millisecond clock rollover", function()
    local f = fixture()
    f.now = 0xffffffff - 300
    f:acquire()
    assert(f.rates[#f.rates] == 20)
end)

check("64-bit capture timestamps keep consecutive frames distinct", function()
    local f = fixture()
    for i = 1, 14 do
        f:tick(30, {stamp=1234567890000 + i*50000})
    end
    assert(f.rates[#f.rates] == 20)
end)

print(string.format("%d QLAND Lua scenarios passed", checks))
