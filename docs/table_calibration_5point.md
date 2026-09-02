# Manual five-point table calibration

This workflow calibrates both CRX-5iA base frames to one shared `table_frame` without
commanding either robot. The operator moves each arm with the teach pendant, touches the
same centre and four square-corner marks, and types the displayed TCP XYZ coordinates.
No pre-measured table XYZ values are required.

## What the five points define

Use one centre mark and four marks that form a square around it. Label them from a
top-down view exactly as follows:

```text
                         +Y
                          ↑
       X_MINUS_Y_PLUS          X_PLUS_Y_PLUS

                   CENTER                → +X

       X_MINUS_Y_MINUS         X_PLUS_Y_MINUS
```

Both arms must touch the same physical mark for the same label. `CENTER` becomes
`[0, 0, 0]`. The averaged negative-to-positive corner directions define `+X` and `+Y`;
the solver orthogonalizes them, then defines `+Z = +X × +Y`. Applied as shown on a
top-down label sheet, `+Z` points above the tabletop.

The square side length is inferred in metres from the robot readings and averaged between
the arms. The solver reports non-square, non-planar, asymmetric, and cross-arm residuals.
The physical square should still be reasonably large and accurately marked; no numeric
dimension needs to be typed into the tool.

## Before recording

- Calibrate and select the intended UTOOL/TCP on both controllers. The physical tip that
  contacts the mark must be the selected TCP.
- Display Cartesian position in `UFRAME 0` (FANUC world) on both teach pendants.
- Use millimetres unless explicitly starting the tool with `--unit m`.
- Record only XYZ. W/P/R are not used. Tool orientation may vary only when the selected
  TCP is already at the contact tip or the configured correction is truly along the
  measurement-frame Z; see the EEF-offset limitation below.
- Stop automatic ROS motion control. Move one arm at a time using the approved teach or
  hand-guiding procedure, with the other arm parked.

The CRX-5iA model places ROS `base_link` 185 mm below FANUC `wbase`. The tool applies this
known fixed offset when the default `fanuc_world` measurement frame is selected.

## Collect and solve

### Simple GUI

From the repository root, start the basic fill-in form:

```bash
./scripts/table_calibration_gui.py
```

Each row is one named centre/corner point. Fill only `Left XYZ` and `Right XYZ`, then use
**Save session** or **Solve + save candidate**. The GUI can reload a partial session and
explicitly activate a valid candidate.

The GUI has separate left/right **EEF Z offset** fields. Their exact convention is:

```text
contact Z in the selected measurement frame = entered Z + EEF Z offset
```

For example, enter `-50 mm` when the physical contact point is 50 mm below the entered
EEF position along UFRAME 0 Z. This is a signed correction; verify the sign with one
known pose before collecting all five points.

An XYZ-only constant offset is correct only when the displacement is along the selected
measurement frame's Z. If the offset is along the tool-local Z and the tool rotates,
keep tool-local Z parallel to UFRAME 0 Z for every sample. Otherwise each point also
needs orientation (W/P/R) to rotate the offset, which this deliberately simple GUI does
not collect. The better controller-side alternative is a calibrated UTOOL whose TCP is
already at the physical contact tip; then leave the GUI offset at zero.

### CLI

The same solver is available from the terminal:

```bash
./scripts/table_calibration_tool.py collect
```

CLI offsets can be supplied when creating a new session:

```bash
./scripts/table_calibration_tool.py collect \
  --left-eef-z-offset-mm -50 \
  --right-eef-z-offset-mm -50
```

The prompts request left-arm readings for the five named marks, followed by right-arm
readings for the same marks. There is no table-coordinate entry stage.

Each entry is saved immediately under `calibration_sessions/`. Enter `q` to stop and
rerun with the displayed session path to resume:

```bash
./scripts/table_calibration_tool.py collect \
  --session calibration_sessions/table_5point_YYYYMMDD_HHMMSS.yaml
```

To replace a mistaken reading and solve again:

```bash
./scripts/table_calibration_tool.py record SESSION.yaml \
  --arm left --point P3 --xyz X Y Z --unit mm
./scripts/table_calibration_tool.py solve SESSION.yaml
```

The default acceptance limits are 3 mm RMS, 6 mm worst point, and 6 mm maximum
left-versus-right disagreement. An invalid candidate is saved for review but cannot be
activated.

## Activate in the virtual workcell

Review the candidate's per-arm and per-point errors, then explicitly activate it:

```bash
./scripts/table_calibration_tool.py activate \
  calibration_candidates/NAME_candidate.yaml
```

Type `ACTIVATE` at the confirmation prompt. This creates the local, git-ignored file:

```text
src/dual_crx_description/config/robot_placement_physical.yaml
```

The resulting `xyz/rpy` values mean `table_frame -> left_base_link` and
`table_frame -> right_base_link`. The combined robot model publishes an identity
`world -> table_frame`, so MoveIt and RViz retain `world` compatibility while clients can
use the explicit table frame. Physical combined-arm launch requires this file to contain
`valid: true`; it never falls back to the mock placement.

After activation, start connection-only and inspect TF/RViz before enabling motion:

```bash
./scripts/build.sh
./scripts/launch_real_gui.sh --launch-rviz
```

Do not accept a calibration solely because the numerical fit is good. Independently move
each TCP to a safe height above several marks and verify physical alignment before using
the transform for collision-aware dual-arm motion.
