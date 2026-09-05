# Reviewed calibration profiles

Files in this directory are reviewed, Git-tracked physical-placement profiles. They
are safe to transfer between computers only when those computers control the same
robots, table, TCP setup, and unchanged physical installation.

After pulling the repository, activate the required profile from the repository root:

```bash
./scripts/table_calibration_tool.py activate \
  calibration_profiles/dual_crx_lab_table_2026-09-02.yaml
```

Type `ACTIVATE` when prompted. This copies the selected profile into the local,
Git-ignored runtime file:

```text
src/dual_crx_description/config/robot_placement_physical.yaml
```

Activation does not move either robot. Restart the physical launch after activation,
then verify TF/RViz and make a conservative single-arm physical check before enabling
normal motion.

Do not put partial sessions or unreviewed candidates in this directory. Create a new
dated profile when the robot bases, table, TCP, or calibration method changes.

## Manual table Z adjustment on the lab profile

The lab profile `dual_crx_lab_table_2026-09-02.yaml` retains an operator-confirmed
-185 mm adjustment along table-frame Z for the **left arm only**. Both arms were
initially adjusted by -185 mm, but the operator reported improved left alignment
and a remaining right-arm discrepancy. The right-arm adjustment was therefore
reverted by adding 185 mm back to its saved Z translation.

| Arm | Original base Z (m) | Current base Z (m) | Net adjustment (m) |
| --- | ---: | ---: | ---: |
| Left | 0.25506070218176946 | 0.07006070218176946 | -0.185 |
| Right | 0.06217679014864952 | 0.06217679014864952 | 0.0 |

X/Y translations and rotations are unchanged. Relative to the initial calibration,
a stationary left flange reads 185 mm lower in Table / World Z; the right flange
uses its original calibration. This changes the calibrated relative placement of
the arms, so check their physical alignment before using dual-arm motion.

This is an empirical placement adjustment, not a change to the solver's 185 mm
ROS base-link/FANUC-world conversion. The loader uses the saved translations
as-is; do not apply another correction during import or activation. Reactivating
this profile copies the current values and does not accumulate the offset.
A new calibration solve does not automatically include this manual adjustment.

`manual_table_z_adjustment` records the original heights and per-arm deltas.
The retained quality statistics belong to the original five-point fit and do not
validate the manual adjustment. Local runtime backups are named
`robot_placement_physical.yaml.backup_before_table_z_*` (before the initial change)
and `robot_placement_physical.yaml.backup_before_right_z_restore_*` (before restoring
right Z). Restart the physical launch in connection-only mode to load the changed
file, and verify table/tool alignment before enabling motion.
