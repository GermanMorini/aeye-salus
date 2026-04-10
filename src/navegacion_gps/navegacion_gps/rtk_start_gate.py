import sys
import time
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, String

NAVSAT_STATUS_GBAS_FIX = 2
FIX_TYPE_NAME_TO_VALUE = {
    "NO_GPS": 0,
    "NO_FIX": 1,
    "2D_FIX": 2,
    "2D": 2,
    "3D_FIX": 3,
    "3D": 3,
    "DGPS": 4,
    "RTK_FLOAT": 5,
    "RTK": 5,
    "RTK_FIX": 5,
    "RTK_FIXED": 6,
}


def parse_required_fix_type(value: object) -> int:
    text = str(value).strip()
    if not text:
        return 5
    try:
        numeric = int(text)
    except (TypeError, ValueError):
        normalized = text.upper()
        if normalized not in FIX_TYPE_NAME_TO_VALUE:
            raise ValueError(
                "required_fix_type must be an integer or one of "
                f"{sorted(FIX_TYPE_NAME_TO_VALUE)}; got {value!r}"
            )
        numeric = FIX_TYPE_NAME_TO_VALUE[normalized]
    return max(0, min(6, int(numeric)))


def status_text_fix_type(status_text: str) -> Optional[int]:
    text = str(status_text).strip().lower()
    if not text:
        return None
    if "rtk_fixed" in text:
        return 6
    if ("rtk_float" in text) or ("rtk_fix" in text) or ("rtk" in text):
        return 5
    if "dgps" in text:
        return 4
    if "3d_fix" in text or "3d fix" in text:
        return 3
    if "2d_fix" in text or "2d fix" in text:
        return 2
    if "no_fix" in text or "no fix" in text:
        return 1
    if "no_gps" in text or "no gps" in text:
        return 0
    return None


def status_text_matches_required(status_text: str, required_fix_type: int) -> bool:
    numeric = status_text_fix_type(status_text)
    if numeric is None:
        return False
    return numeric >= int(required_fix_type)


def fix_type_matches_required(
    fix_type: Optional[int], required_fix_type: int
) -> bool:
    if fix_type is None:
        return False
    try:
        numeric = int(fix_type)
    except (TypeError, ValueError):
        return False
    return numeric >= int(required_fix_type)


def navsat_status_matches_required(
    status: Optional[int], required_fix_type: int
) -> bool:
    if status is None:
        return False
    try:
        numeric = int(status)
    except (TypeError, ValueError):
        return False
    required = int(required_fix_type)
    if required >= 4:
        return numeric >= NAVSAT_STATUS_GBAS_FIX
    return numeric >= 0


def any_fix_signal(
    *,
    status_text: Optional[str] = None,
    fix_type: Optional[int] = None,
    navsat_status: Optional[int] = None,
    required_fix_type: int = 5,
) -> bool:
    return (
        status_text_matches_required(status_text or "", required_fix_type)
        or fix_type_matches_required(fix_type, required_fix_type)
        or navsat_status_matches_required(navsat_status, required_fix_type)
    )


class RtkStartGateNode(Node):
    def __init__(self) -> None:
        super().__init__("rtk_start_gate")

        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("rtk_status_topic", "/gps/rtk_status")
        self.declare_parameter("fix_type_topic", "/gps/fix_type")
        self.declare_parameter("required_fix_type", "RTK_FLOAT")
        self.declare_parameter("timeout_s", 60.0)

        self.gps_topic = str(self.get_parameter("gps_topic").value)
        self.rtk_status_topic = str(self.get_parameter("rtk_status_topic").value)
        self.fix_type_topic = str(self.get_parameter("fix_type_topic").value)
        self.required_fix_type = parse_required_fix_type(
            self.get_parameter("required_fix_type").value
        )
        self.timeout_s = max(1.0, float(self.get_parameter("timeout_s").value))

        self._ready = False
        self._ready_source = ""
        self._last_status_text = ""
        self._last_fix_type: Optional[int] = None
        self._last_navsat_status: Optional[int] = None

        self.create_subscription(
            String, self.rtk_status_topic, self._on_rtk_status, qos_profile_sensor_data
        )
        self.create_subscription(
            Int32, self.fix_type_topic, self._on_fix_type, qos_profile_sensor_data
        )
        self.create_subscription(
            NavSatFix, self.gps_topic, self._on_gps_fix, qos_profile_sensor_data
        )

        self.get_logger().info(
            "Startup fix gate waiting for fix_type-or-better "
            f"(required_fix_type={self.required_fix_type}, "
            f"timeout_s={self.timeout_s:.1f}, gps_topic={self.gps_topic}, "
            f"rtk_status_topic={self.rtk_status_topic}, fix_type_topic={self.fix_type_topic})"
        )

    def _mark_ready(self, source: str, detail: str) -> None:
        if self._ready:
            return
        self._ready = True
        self._ready_source = str(source)
        self.get_logger().info(
            f"Startup fix gate satisfied via {source}: {detail}"
        )

    def _on_rtk_status(self, msg: String) -> None:
        status_text = str(msg.data or "")
        self._last_status_text = status_text
        if any_fix_signal(
            status_text=status_text,
            required_fix_type=self.required_fix_type,
        ):
            self._mark_ready("rtk_status", status_text)

    def _on_fix_type(self, msg: Int32) -> None:
        fix_type = int(msg.data)
        self._last_fix_type = fix_type
        if any_fix_signal(
            fix_type=fix_type,
            required_fix_type=self.required_fix_type,
        ):
            self._mark_ready("fix_type", str(fix_type))

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        navsat_status = int(msg.status.status)
        self._last_navsat_status = navsat_status
        if any_fix_signal(
            navsat_status=navsat_status,
            required_fix_type=self.required_fix_type,
        ):
            self._mark_ready("gps_fix", str(navsat_status))

    def wait_until_ready_or_timeout(self) -> int:
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        started = time.monotonic()
        try:
            while rclpy.ok():
                if self._ready:
                    return 0
                elapsed = time.monotonic() - started
                if elapsed >= self.timeout_s:
                    self.get_logger().error(
                        "Startup fix gate timeout "
                        f"(required_fix_type={self.required_fix_type}, "
                        f"timeout_s={self.timeout_s:.1f}, "
                        f"last_status_text='{self._last_status_text}', "
                        f"last_fix_type={self._last_fix_type}, "
                        f"last_navsat_status={self._last_navsat_status})"
                    )
                    return 1
                executor.spin_once(timeout_sec=0.2)
        finally:
            executor.remove_node(self)
            executor.shutdown()
        return 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtkStartGateNode()
    exit_code = 1
    try:
        exit_code = node.wait_until_ready_or_timeout()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main(sys.argv)
