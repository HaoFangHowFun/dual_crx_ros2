#!/usr/bin/env python3
"""Deliberately simple GUI for the manual five-point calibration workflow."""

from __future__ import annotations

import math
from pathlib import Path
import sys

from PyQt5 import QtCore, QtGui, QtWidgets

from table_calibration_tool import (
    ARMS,
    DEFAULT_BASE_TO_FANUC_WORLD_Z_M,
    POINT_IDS,
    CalibrationError,
    activate_candidate_file,
    atomic_yaml_write,
    default_candidate_path,
    default_session_path,
    load_yaml,
    new_session,
    solve_session,
    utc_now,
)


WORKSPACE = Path(__file__).resolve().parents[1]
ACTIVE_PLACEMENT = (
    WORKSPACE
    / "src"
    / "dual_crx_description"
    / "config"
    / "robot_placement_physical.yaml"
)
SECTIONS = (
    "left_arm_tcp_points_m",
    "right_arm_tcp_points_m",
)


class CalibrationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dual CRX five-point table calibration")
        self.resize(1260, 620)
        self.loaded_metadata = {}
        self.last_session_path: Path | None = None
        self.last_candidate_path: Path | None = None
        self.cells: dict[tuple[str, str, int], QtWidgets.QLineEdit] = {}

        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)

        warning = QtWidgets.QLabel(
            "Manual input only — this GUI never connects to or moves a robot. "
            "Use the same calibrated UTOOL and UFRAME 0 for all readings."
        )
        warning.setWordWrap(True)
        warning.setStyleSheet("font-weight: bold; color: #8a3b00")
        layout.addWidget(warning)

        settings = QtWidgets.QGridLayout()
        self.unit = QtWidgets.QComboBox()
        self.unit.addItems(["mm", "m"])
        self.measurement_frame = QtWidgets.QComboBox()
        self.measurement_frame.addItems(["fanuc_world", "base_link"])
        self.left_offset = self._number_box()
        self.right_offset = self._number_box()
        self.left_offset.setRange(-2000.0, 2000.0)
        self.right_offset.setRange(-2000.0, 2000.0)
        self.offset_unit_labels = [QtWidgets.QLabel("mm"), QtWidgets.QLabel("mm")]
        self.unit.currentTextChanged.connect(self._unit_label_changed)
        self.tcp_note = QtWidgets.QLineEdit()
        self.tcp_note.setPlaceholderText("例如：UTOOL 3，尖端已校正")

        settings.addWidget(QtWidgets.QLabel("Input unit"), 0, 0)
        settings.addWidget(self.unit, 0, 1)
        settings.addWidget(QtWidgets.QLabel("Measurement frame"), 0, 2)
        settings.addWidget(self.measurement_frame, 0, 3)
        settings.addWidget(QtWidgets.QLabel("Left EEF Z offset"), 1, 0)
        settings.addWidget(self.left_offset, 1, 1)
        settings.addWidget(self.offset_unit_labels[0], 1, 2)
        settings.addWidget(QtWidgets.QLabel("Right EEF Z offset"), 1, 3)
        settings.addWidget(self.right_offset, 1, 4)
        settings.addWidget(self.offset_unit_labels[1], 1, 5)
        settings.addWidget(QtWidgets.QLabel("TCP / setup note"), 2, 0)
        settings.addWidget(self.tcp_note, 2, 1, 1, 5)
        layout.addLayout(settings)

        offset_help = QtWidgets.QLabel(
            "Offset convention: contact Z = entered Z + offset. If the offset is along "
            "the rotating tool-local Z axis, keep tool Z parallel to UFRAME 0 Z at every "
            "point; otherwise XYZ-only input cannot rotate the offset correctly."
        )
        offset_help.setWordWrap(True)
        offset_help.setStyleSheet("color: #555")
        layout.addWidget(offset_help)

        point_map = QtWidgets.QLabel(
            "+Y ↑       X_MINUS_Y_PLUS       X_PLUS_Y_PLUS\n"
            "   |                CENTER\n"
            "   +----→ +X  X_MINUS_Y_MINUS      X_PLUS_Y_MINUS"
        )
        point_map.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        point_map.setStyleSheet("background: #f3f3f3; padding: 6px")
        layout.addWidget(point_map)

        grid = QtWidgets.QGridLayout()
        headers = (
            "Point",
            "Left X",
            "Left Y",
            "Left Z",
            "Right X",
            "Right Y",
            "Right Z",
        )
        for column, title in enumerate(headers):
            label = QtWidgets.QLabel(title)
            label.setStyleSheet("font-weight: bold")
            grid.addWidget(label, 0, column)
        for row, point_id in enumerate(POINT_IDS, start=1):
            grid.addWidget(QtWidgets.QLabel(point_id), row, 0)
            column = 1
            for section in SECTIONS:
                for axis in range(3):
                    edit = QtWidgets.QLineEdit()
                    edit.setPlaceholderText("0.000")
                    edit.setValidator(QtGui.QDoubleValidator(-1e9, 1e9, 6, edit))
                    edit.setMinimumWidth(92)
                    self.cells[(section, point_id, axis)] = edit
                    grid.addWidget(edit, row, column)
                    column += 1
        layout.addLayout(grid)

        buttons = QtWidgets.QHBoxLayout()
        for title, callback in (
            ("Load session", self.load_session),
            ("Save session", self.save_session),
            ("Solve + save candidate", self.solve_candidate),
            ("Activate candidate", self.activate_candidate),
            ("Clear", self.clear_form),
        ):
            button = QtWidgets.QPushButton(title)
            button.clicked.connect(callback)
            buttons.addWidget(button)
        layout.addLayout(buttons)

        self.result = QtWidgets.QPlainTextEdit()
        self.result.setReadOnly(True)
        self.result.setMaximumHeight(155)
        self.result.setPlainText("Fill the left/right XYZ for all five labelled points, then solve.")
        layout.addWidget(self.result)

    @staticmethod
    def _number_box() -> QtWidgets.QDoubleSpinBox:
        box = QtWidgets.QDoubleSpinBox()
        box.setDecimals(4)
        box.setRange(-1e6, 1e6)
        return box

    def _unit_label_changed(self, unit: str) -> None:
        for label in self.offset_unit_labels:
            label.setText(unit)

    def _scale_to_m(self) -> float:
        return 0.001 if self.unit.currentText() == "mm" else 1.0

    def _session_from_form(self, allow_partial: bool) -> dict:
        scale = self._scale_to_m()
        session = new_session(
            self.unit.currentText(),
            self.measurement_frame.currentText(),
            self.tcp_note.text().strip(),
            self.left_offset.value() * scale,
            self.right_offset.value() * scale,
        )
        if self.loaded_metadata.get("created_at"):
            session["created_at"] = self.loaded_metadata["created_at"]
        session["updated_at"] = utc_now()
        for section in SECTIONS:
            values = session[section]
            for point_id in POINT_IDS:
                texts = [
                    self.cells[(section, point_id, axis)].text().strip()
                    for axis in range(3)
                ]
                if not any(texts) and allow_partial:
                    continue
                if not all(texts):
                    raise CalibrationError(f"{section} {point_id} has an incomplete XYZ")
                try:
                    xyz = [float(text) * scale for text in texts]
                except ValueError as exc:
                    raise CalibrationError(f"{section} {point_id} contains invalid text") from exc
                if not all(math.isfinite(value) for value in xyz):
                    raise CalibrationError(f"{section} {point_id} contains NaN or infinity")
                values[point_id] = xyz
        session["base_to_fanuc_world_z_m"] = DEFAULT_BASE_TO_FANUC_WORLD_Z_M
        return session

    def _fill_form(self, session: dict) -> None:
        unit = session.get("input_unit", "mm")
        if unit not in ("mm", "m"):
            unit = "mm"
        self.unit.setCurrentText(unit)
        scale_from_m = 1000.0 if unit == "mm" else 1.0
        frame = session.get("measurement_frame", "fanuc_world")
        self.measurement_frame.setCurrentText(frame)
        offsets = session.get("eef_z_offset_m", {})
        self.left_offset.setValue(float(offsets.get("left_arm", 0.0)) * scale_from_m)
        self.right_offset.setValue(float(offsets.get("right_arm", 0.0)) * scale_from_m)
        self.tcp_note.setText(session.get("tcp_setup_note", ""))
        for section in SECTIONS:
            values = session.get(section, {})
            for point_id in POINT_IDS:
                xyz = values.get(point_id)
                for axis in range(3):
                    edit = self.cells[(section, point_id, axis)]
                    edit.clear()
                    if isinstance(xyz, list) and len(xyz) == 3:
                        edit.setText(f"{float(xyz[axis]) * scale_from_m:.6f}")
        self.loaded_metadata = {"created_at": session.get("created_at")}

    def _show_error(self, exc: Exception) -> None:
        self.result.setPlainText(f"ERROR: {exc}")
        QtWidgets.QMessageBox.critical(self, "Calibration error", str(exc))

    def load_session(self) -> None:
        path_text, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load calibration session",
            str(WORKSPACE / "calibration_sessions"),
            "YAML (*.yaml *.yml)",
        )
        if not path_text:
            return
        try:
            path = Path(path_text)
            session = load_yaml(path)
            self._fill_form(session)
            self.last_session_path = path
            self.result.setPlainText(f"Loaded session: {path}")
        except Exception as exc:
            self._show_error(exc)

    def save_session(self) -> Path | None:
        try:
            session = self._session_from_form(allow_partial=True)
            suggested = self.last_session_path or (WORKSPACE / default_session_path())
            path_text, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "Save calibration session", str(suggested), "YAML (*.yaml)"
            )
            if not path_text:
                return None
            path = Path(path_text)
            atomic_yaml_write(path, session)
            self.last_session_path = path
            self.loaded_metadata = {"created_at": session.get("created_at")}
            self.result.setPlainText(f"Saved session: {path}")
            return path
        except Exception as exc:
            self._show_error(exc)
            return None

    def solve_candidate(self) -> None:
        try:
            session = self._session_from_form(allow_partial=False)
            candidate = solve_session(session)
            base_session = self.last_session_path or (WORKSPACE / default_session_path())
            suggested = WORKSPACE / default_candidate_path(Path(base_session))
            path_text, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "Save calibration candidate", str(suggested), "YAML (*.yaml)"
            )
            if not path_text:
                return
            path = Path(path_text)
            atomic_yaml_write(path, candidate)
            self.last_candidate_path = path
            lines = ["VALID" if candidate["valid"] else "INVALID"]
            for arm in ARMS:
                quality = candidate["quality"]["arms"][arm]
                placement = candidate[arm]
                lines.append(
                    f"{arm}: RMS {quality['rms_m'] * 1000:.2f} mm, "
                    f"max {quality['max_m'] * 1000:.2f} mm"
                )
                lines.append(f"  xyz={placement['xyz']}")
                lines.append(f"  rpy={placement['rpy']}")
            lines.append(
                f"cross-arm max: {candidate['quality']['cross_arm_max_m'] * 1000:.2f} mm"
            )
            lines.append(f"candidate: {path}")
            if not candidate["valid"]:
                lines.append("Not activatable: redo the largest-error point(s).")
            self.result.setPlainText("\n".join(lines))
        except Exception as exc:
            self._show_error(exc)

    def activate_candidate(self) -> None:
        initial = self.last_candidate_path or (WORKSPACE / "calibration_candidates")
        path_text, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select valid candidate", str(initial), "YAML (*.yaml *.yml)"
        )
        if not path_text:
            return
        candidate_path = Path(path_text)
        answer = QtWidgets.QMessageBox.question(
            self,
            "Activate physical placement?",
            f"Activate:\n{candidate_path}\n\nas:\n{ACTIVE_PLACEMENT}?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        try:
            backup = activate_candidate_file(candidate_path, ACTIVE_PLACEMENT)
            message = f"Activated: {ACTIVE_PLACEMENT}"
            if backup:
                message += f"\nBackup: {backup}"
            self.result.setPlainText(message)
        except Exception as exc:
            self._show_error(exc)

    def clear_form(self) -> None:
        if QtWidgets.QMessageBox.question(
            self,
            "Clear form?",
            "Clear every coordinate field?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        ) != QtWidgets.QMessageBox.Yes:
            return
        for edit in self.cells.values():
            edit.clear()
        self.loaded_metadata = {}
        self.last_session_path = None
        self.last_candidate_path = None
        self.result.setPlainText("Form cleared.")


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    window = CalibrationWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
