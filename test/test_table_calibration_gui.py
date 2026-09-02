import os
from pathlib import Path
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5 import QtWidgets
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from table_calibration_gui import CalibrationWindow, POINT_IDS, SECTIONS  # noqa: E402


def test_gui_builds_session_with_offsets_and_all_five_points():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = CalibrationWindow()
    window.unit.setCurrentText("mm")
    window.left_offset.setValue(75.0)
    window.right_offset.setValue(-12.5)
    for point_index, point_id in enumerate(POINT_IDS):
        for section_index, section in enumerate(SECTIONS):
            for axis in range(3):
                value = 100.0 * section_index + 10.0 * point_index + axis
                window.cells[(section, point_id, axis)].setText(str(value))

    session = window._session_from_form(allow_partial=False)

    assert "table_points_m" not in session
    assert len(session["left_arm_tcp_points_m"]) == 5
    assert session["eef_z_offset_m"]["left_arm"] == 0.075
    assert session["eef_z_offset_m"]["right_arm"] == -0.0125
    assert session["right_arm_tcp_points_m"]["X_MINUS_Y_MINUS"][2] == pytest.approx(0.142)
    window.deleteLater()
    app.processEvents()
