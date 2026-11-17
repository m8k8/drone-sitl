"""Confirm that each observed emergency mode was selected by ArduCopter."""

from pymavlink import mavutil

# ArduCopter Mode::Number and ModeReason in the pinned firmware.
MODE_NAMES = {6: "RTL", 9: "LAND"}
BATTERY_FAILSAFE_REASON = 4


def verify(bin_logs, summary, *, critical):
    observed = []
    for path in bin_logs:
        log = mavutil.mavlink_connection(str(path))
        try:
            while (message := log.recv_match(type="MODE")) is not None:
                mode = MODE_NAMES.get(message.ModeNum)
                if mode is not None:
                    observed.append({"time_boot_s": message.TimeUS / 1e6, "mode": mode, "reason": int(message.Rsn)})
        finally:
            log.close()
    summary["metrics"]["battery_failsafe"]["controller_mode_log"] = observed
    expected = ["RTL", "LAND"] if critical else ["RTL"]
    if ([entry["mode"] for entry in observed] != expected
            or any(entry["reason"] != BATTERY_FAILSAFE_REASON for entry in observed)):
        raise RuntimeError(f"Expected battery failsafe mode transitions {expected}, got {observed}")
