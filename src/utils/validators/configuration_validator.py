import json
import re
from pathlib import Path
from typing import Any, Optional

from utils.validators.validation_error import ValidationError


class ConfigurationValidator:
    """
    Validate a JSON configuration file for Autolander.
    Expected schema:

    {
      "camera": {
        "id": int>=0,
        "use_picamera": bool,
        "fps": int in [1..240],
        "calibration_filepath": str ("" allowed, else must exist)
      },
      "vision": {
        "targeted_marker": {
          "length": float>0,
          "id": int>=0,
          "dictionary": int>=0
        }
      },
      "streaming": {
        "port": int in [1..65535],
        "data": { "dps": int in [1..240] },
        "video": { "fps": int in [1..240] }
      },
      "drone_connection": {
        "use_serial": bool,
        "address": str non-empty,
        "port": int in [1..65535] OR null,
        "baud_rate": int>0
      }
    }
    """

    def __init__(self, config_path: Path) -> None:
        self.config_path = config_path

    def validate(self) -> None:
        p = self.config_path

        # ---- file checks
        if not isinstance(p, Path):
            raise ValidationError("config_path must be a pathlib.Path", "root")

        if p.suffix.lower() != ".json":
            raise ValidationError(f"Configuration must be a .json file, got '{p.name}'", "root")

        if not p.exists():
            raise ValidationError(f"Configuration file not found: {p}", "root")

        if not p.is_file():
            raise ValidationError(f"Configuration path is not a file: {p}", "root")

        # ---- parse JSON
        try:
            raw = json.loads(p.read_text(encoding="utf-8"))
        except json.JSONDecodeError as e:
            raise ValidationError(f"Invalid JSON: {e.msg} (line {e.lineno}, col {e.colno})", "root") from e

        if not isinstance(raw, dict):
            raise ValidationError("Root JSON must be an object/dict", "root")

        # ---- schema validation
        self._validate_camera(raw)
        self._validate_vision(raw)
        self._validate_streaming(raw)
        self._validate_drone_connection(raw)

        # Optionnel: refuser les clés inconnues au root (strict)
        allowed_root = {"camera", "vision", "streaming", "drone_connection"}
        extra = set(raw.keys()) - allowed_root
        if extra:
            raise ValidationError(f"Unknown root keys: {sorted(extra)}", "root")

    # -------------------------
    # Section validators
    # -------------------------

    def _validate_camera(self, root: dict[str, Any]) -> None:
        camera = self._req_obj(root, "camera", "root")

        cid = self._req_int(camera, "id", "camera", min_=0)
        _ = cid  # for clarity

        self._req_bool(camera, "use_picamera", "camera")
        self._req_int(camera, "fps", "camera", min_=1, max_=240)

        calib = self._req_str(camera, "calibration_filepath", "camera", allow_empty=True)
        if calib != "":
            calib_path = (self.config_path.parent / calib).resolve() if not Path(calib).is_absolute() else Path(calib)
            if not calib_path.exists():
                raise ValidationError(
                    f"calibration_filepath does not exist: {calib_path}",
                    "camera.calibration_filepath",
                )
            if not calib_path.is_file():
                raise ValidationError(
                    f"calibration_filepath is not a file: {calib_path}",
                    "camera.calibration_filepath",
                )

        self._no_extra_keys(camera, {"id", "use_picamera", "fps", "calibration_filepath"}, "camera")

    def _validate_vision(self, root: dict[str, Any]) -> None:
        vision = self._req_obj(root, "vision", "root")
        tm = self._req_obj(vision, "targeted_marker", "vision")

        self._req_number(tm, "length", "vision.targeted_marker", min_exclusive=0.0)
        self._req_int(tm, "id", "vision.targeted_marker", min_=0)
        self._req_int(tm, "dictionary", "vision.targeted_marker", min_=0)

        self._no_extra_keys(tm, {"length", "id", "dictionary"}, "vision.targeted_marker")
        self._no_extra_keys(vision, {"targeted_marker"}, "vision")

    def _validate_streaming(self, root: dict[str, Any]) -> None:
        streaming = self._req_obj(root, "streaming", "root")
        self._req_int(streaming, "port", "streaming", min_=1, max_=65535)

        data = self._req_obj(streaming, "data", "streaming")
        self._req_int(data, "dps", "streaming.data", min_=1, max_=240)

        video = self._req_obj(streaming, "video", "streaming")
        self._req_int(video, "fps", "streaming.video", min_=1, max_=240)

        self._no_extra_keys(data, {"dps"}, "streaming.data")
        self._no_extra_keys(video, {"fps"}, "streaming.video")
        self._no_extra_keys(streaming, {"port", "data", "video"}, "streaming")

    def _validate_drone_connection(self, root: dict[str, Any]) -> None:
        dc = self._req_obj(root, "drone_connection", "root")

        use_serial = self._req_bool(dc, "use_serial", "drone_connection")
        address = self._req_str(dc, "address", "drone_connection", allow_empty=False)
        self._req_int(dc, "baud_rate", "drone_connection", min_=1)

        # port can be null or int
        port_val = self._req_key(dc, "port", "drone_connection")
        if port_val is None:
            port: Optional[int] = None
        else:
            if not isinstance(port_val, int) or isinstance(port_val, bool):
                raise ValidationError("port must be an int or null", "drone_connection.port")
            if not (1 <= port_val <= 65535):
                raise ValidationError("port must be in [1..65535]", "drone_connection.port")
            port = port_val

        # Coherence rules
        if use_serial:
            # pour Serial, port n’est généralement pas utilisé
            if port is not None:
                raise ValidationError("When use_serial=true, port must be null", "drone_connection.port")
            # adresse attendue: device path non vide
            if not address.startswith("/"):
                # On reste tolérant, mais on protège contre un IP typo
                raise ValidationError(
                    "When use_serial=true, address should be a device path like '/dev/serial0'",
                    "drone_connection.address",
                )
        else:
            # pour UDP/TCP, port requis
            if port is None:
                raise ValidationError("When use_serial=false, port is required (not null)", "drone_connection.port")
            # adresse attendue: ip/host (simple check)
            if address.startswith("/"):
                raise ValidationError(
                    "When use_serial=false, address should be an IP/hostname, not a device path",
                    "drone_connection.address",
                )
            self._validate_host_like(address, "drone_connection.address")

        self._no_extra_keys(dc, {"use_serial", "address", "port", "baud_rate"}, "drone_connection")

    # -------------------------
    # Helpers
    # -------------------------

    def _req_key(self, d: dict[str, Any], key: str, parent_path: str) -> Any:
        if key not in d:
            raise ValidationError(f"Missing required key '{key}'", f"{parent_path}.{key}")
        return d[key]

    def _req_obj(self, d: dict[str, Any], key: str, parent_path: str) -> dict[str, Any]:
        v = self._req_key(d, key, parent_path)
        if not isinstance(v, dict):
            raise ValidationError("Expected an object/dict", f"{parent_path}.{key}")
        return v

    def _req_bool(self, d: dict[str, Any], key: str, parent_path: str) -> bool:
        v = self._req_key(d, key, parent_path)
        if not isinstance(v, bool):
            raise ValidationError("Expected a boolean", f"{parent_path}.{key}")
        return v

    def _req_int(
        self,
        d: dict[str, Any],
        key: str,
        parent_path: str,
        *,
        min_: Optional[int] = None,
        max_: Optional[int] = None,
    ) -> int:
        v = self._req_key(d, key, parent_path)
        # bool est un int en Python -> on l’exclut
        if not isinstance(v, int) or isinstance(v, bool):
            raise ValidationError("Expected an integer", f"{parent_path}.{key}")
        if min_ is not None and v < min_:
            raise ValidationError(f"Must be >= {min_}", f"{parent_path}.{key}")
        if max_ is not None and v > max_:
            raise ValidationError(f"Must be <= {max_}", f"{parent_path}.{key}")
        return v

    def _req_number(
        self,
        d: dict[str, Any],
        key: str,
        parent_path: str,
        *,
        min_inclusive: Optional[float] = None,
        min_exclusive: Optional[float] = None,
        max_inclusive: Optional[float] = None,
    ) -> float:
        v = self._req_key(d, key, parent_path)
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            raise ValidationError("Expected a number", f"{parent_path}.{key}")
        fv = float(v)
        if min_inclusive is not None and fv < min_inclusive:
            raise ValidationError(f"Must be >= {min_inclusive}", f"{parent_path}.{key}")
        if min_exclusive is not None and fv <= min_exclusive:
            raise ValidationError(f"Must be > {min_exclusive}", f"{parent_path}.{key}")
        if max_inclusive is not None and fv > max_inclusive:
            raise ValidationError(f"Must be <= {max_inclusive}", f"{parent_path}.{key}")
        return fv

    def _req_str(self, d: dict[str, Any], key: str, parent_path: str, *, allow_empty: bool) -> str:
        v = self._req_key(d, key, parent_path)
        if not isinstance(v, str):
            raise ValidationError("Expected a string", f"{parent_path}.{key}")
        if not allow_empty and v.strip() == "":
            raise ValidationError("Must be a non-empty string", f"{parent_path}.{key}")
        return v

    def _no_extra_keys(self, obj: dict[str, Any], allowed: set[str], path: str) -> None:
        extra = set(obj.keys()) - allowed
        if extra:
            raise ValidationError(f"Unknown keys: {sorted(extra)}", path)

    def _validate_host_like(self, host: str, path: str) -> None:
        # Tolérant: hostname ou IPv4
        ipv4 = re.compile(r"^\d{1,3}(\.\d{1,3}){3}$")
        hostname = re.compile(r"^(?=.{1,253}$)([a-zA-Z0-9-]{1,63}\.)*[a-zA-Z0-9-]{1,63}$")

        if ipv4.match(host):
            parts = host.split(".")
            if any(int(p) > 255 for p in parts):
                raise ValidationError("Invalid IPv4 address", path)
            return

        if hostname.match(host):
            return

        raise ValidationError("Invalid host/hostname format", path)
