from pathlib import Path

import pytest

from dual_crx_control.workcell import (
    MIN_CLEARANCE_M,
    WorkcellError,
    load_workcell,
    verification_targets,
)


PROFILE = (
    Path(__file__).parents[3]
    / "calibration_profiles"
    / "dual_crx_lab_table_2026-09-02.yaml"
)


def test_profile_metadata_identifies_exact_loaded_file():
    data, metadata = load_workcell(PROFILE)

    assert metadata["valid"] is True
    assert metadata["profile_name"] == "dual_crx_lab_table_2026-09-02"
    assert metadata["placement_file"] == str(PROFILE.resolve())
    assert len(metadata["sha256"]) == 64
    assert metadata["left_eef_z_offset_m"] == -0.035
    assert data["coordinate_contract"]["world_frame"] == "table_frame"


def test_verification_targets_apply_eef_offset_and_safe_clearance():
    data, _ = load_workcell(PROFILE)
    targets = verification_targets(data, "right_arm", 0.020, 0.080)

    assert len(targets) == 5
    assert targets[0]["point"] == "CENTER"
    assert targets[0]["transit"][2] > targets[0]["checkpoint"][2]
    assert targets[0]["checkpoint"][2] == pytest.approx(0.055, abs=0.001)


def test_three_millimetres_is_rejected_for_current_coarse_calibration():
    data, _ = load_workcell(PROFILE)

    with pytest.raises(WorkcellError, match="at least 15 mm"):
        verification_targets(data, "left_arm", 0.003, 0.080)

    assert MIN_CLEARANCE_M == 0.015
