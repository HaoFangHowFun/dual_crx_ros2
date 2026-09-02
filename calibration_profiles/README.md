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
