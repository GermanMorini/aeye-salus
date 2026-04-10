from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_rtk_start_gate_contains_embedded_rtk_semantics() -> None:
    gate_contents = (
        PACKAGE_ROOT / "navegacion_gps" / "rtk_start_gate.py"
    ).read_text(encoding="utf-8")

    assert "NAVSAT_STATUS_GBAS_FIX = 2" in gate_contents
    assert "FIX_TYPE_NAME_TO_VALUE = {" in gate_contents
    assert 'self.declare_parameter("required_fix_type", "RTK_FLOAT")' in gate_contents
    assert "def parse_required_fix_type(value: object) -> int:" in gate_contents
    assert '"DGPS": 4,' in gate_contents
    assert '"RTK_FLOAT": 5,' in gate_contents
    assert '"RTK_FIXED": 6,' in gate_contents
    assert "def status_text_fix_type(status_text: str) -> Optional[int]:" in gate_contents
    assert 'if "dgps" in text:' in gate_contents
    assert "def fix_type_matches_required(" in gate_contents
    assert "return numeric >= int(required_fix_type)" in gate_contents
    assert "def navsat_status_matches_required(" in gate_contents
    assert "if required >= 4:" in gate_contents
    assert "def any_fix_signal(" in gate_contents
