# Working notes, phase 1 — from the professor's message to a working manifold planner

**Chronologically this precedes `session_notes.md`.** That file opens with "the
5-DOF manifold planner already existed and worked"; this file is how it came to
exist. Read this one first.

Same convention as the other file: roughly chronological, includes the dead ends
because several are more informative than the fixes, and every number is measured
on the running system rather than estimated.

---

## 0. The starting point

The professor's message (2026-08-03), verbatim:

> Δεν προτείνω να μην χρησιμοποιήσεις τους planners που υπάρχουν. Απλά προτείνω να
> μειώσεις το χώρο αναζητησης αντι για τις 7 αρθρώσεις στις 4 ή 3. Για το collision
> avoidance μπορεις να θεωρήσεις το κομμάτι του σφαιρικού καρπού σαν ένα στερεό
> (λαμβάνοντας όλες τις δυνατές θέσεις και προσανατολισμού του σωλήνα) και να κάνεις
> augment το βραχίονα των 3-4 DoFs ώστε να λύνεις το πρόβλημα με OMPL. Αυτό θα είναι
> και καλό για τη διπλωματική σου σαν δευτερη προσέγγιση.

Three separate instructions, worth separating because only two of them survived:

1. **Keep the existing planners.** Not a request to write a new planner — a
   request to change what they search over.
2. **Shrink the search space from 7 joints to 4 or 3.**
3. **For collision avoidance, treat the spherical wrist as a rigid body** — the
   volume swept over all positions and orientations the tube can take — and
   augment the reduced arm with it.

Plus the framing: a *second approach* for the thesis.

Instructions 1 and 2 are in the shipped system. Instruction 3 was implemented in
full, measured, and found not to work on this robot — see §4. That negative
result, and the modification that fixed it, is the actual contribution.

A note on interpretation that mattered later: the user clarified partway through
that the professor did not want two pipelines maintained for comparison — the new
approach should work, with the old one as a fallback. The benchmark in §5 was
still worth building, but as evidence rather than as a deliverable.

---

## 1. Environment and setup

Recording this because two full sessions were lost to environment problems that
look like code bugs.

### 1.1 The stack

| component | location | notes |
|---|---|---|
| workspace | `~/elephant_robots_ws` | `colcon build` must run from here |
| MoveIt | `~/ws_moveit2` — **source build** | 2.5.9, includes `trac_ik` |
| ROS | `/opt/ros/humble` | also has MoveIt 2.5.9, but *without* trac_ik |
| fake driver | `~/my_robot/fake_robot.py` | action servers + `/fake_joint_states` |
| GUI | `~/my_robot/GUI.py` | |

**Source order matters and is not optional:**

```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash
source ~/elephant_robots_ws/install/setup.bash
```

Skipping the middle line makes `move_group` fail at startup with

```
trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin ... does not exist
ompl_interface/OMPLPlanner ... does not exist
```

which looks like a broken config and is not. Both trees ship 2.5.9, so there is
no ABI problem — only a search-path one.

### 1.2 Running it

```bash
cd ~/my_robot && python3 fake_robot.py                       # terminal 1
ros2 launch robot_config fake.launch.py                      # terminal 2
ros2 launch my_robot_control run_simple_move.launch.py       # terminal 3
```

`fake.launch.py` is the correct launch file for the fake driver — its
`joint_state_publisher` has `source_list: ['fake_joint_states']`, matching what
`fake_robot.py` publishes, and its controller manager declares the two action
servers `fake_robot.py` creates. `demo.launch.py` has neither and cannot drive
the fake robot.

Markers 1–6 have no ArUco TF in this setup, so `simple_move` falls back to
hardcoded tube positions. That is by design and is what all testing ran against.

### 1.3 Build gotcha that cost real time

**After editing anything in `include/my_robot_control/`, rebuild clean.**

```bash
rm -rf build/my_robot_control install/my_robot_control
colcon build --packages-select my_robot_control --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Two fields were added to `ManifoldPlanner::Options`. The incremental build
recompiled `manifold_planner.cpp` but *not* `simple_move.cpp`, so one side
constructed the old, smaller struct while the other read the two new fields from
memory past its end — getting zeros. No compile error, no warning, no link
error. The symptom was a change that appeared to do nothing at all: identical
diagnostic counts (`32 arm IK solutions, 1 rejected by wrist limits, 31 in
collision`) before and after a fix that should have changed them by 20×.

The tell was that the numbers were *identical*, not merely similar. Byte-for-byte
identical output after a real change means the change is not running.

### 1.4 Testing discipline

**Restart the whole stack between tests.** Restarting only `simple_move` leaves
attached collision objects in `move_group`'s planning scene. A fresh
`simple_move` then believes the gripper is empty while the scene still has a tube
welded to `link7`, so every goal reads as a collision.

This produced a completely bogus diagnosis — "650 arm IK solutions, 629 in
collision" — that sent me down a wrong path until the user pointed it out. The
`session_notes.md` file records the same rule being learned again from the DDS
side (duplicate node names breaking discovery). It is the *partial* restart that
bites, in at least two independent ways.

Also: **never `pkill -f <pattern>` where the pattern matches the shell running
it.** `pkill -f benchmark_reduced` matched its own wrapper's command line and
killed the launcher while leaving three worker processes alive, all competing for
one `move_group` and writing to the same CSV. Match on process name instead:
`ps -C <name> -o pid=`.

### 1.5 Pre-existing issues found but deliberately not fixed

None of these were touched, because they were outside what was asked. All three
are real.

| where | issue |
|---|---|
| `robot_config/config/*.srdf` | stray `2` after `</group_state>` on the `open` state. XML parsers treat it as a text node and ignore it, so nothing breaks. |
| `robot_config/launch/fake.launch.py:78` | Pilz `request_adapters` names `default_plan**ning**_request_adapters/*`, but this MoveIt provides `default_plan**ner**_request_adapters/*`, and three of the four class names are from a newer MoveIt. The Pilz pipeline fails to load at every startup — four red ERROR blocks that look serious and are not. OMPL is unaffected. |
| `my_robot_control/launch/run_simple_move.launch.py:22` | sets `use_sim_time: True` while `fake.launch.py` sets it `False` for everything else and nothing publishes `/clock`. |

(The later session did modify `fake.launch.py` and `run_simple_move.launch.py`
for other reasons — see `session_notes.md` §8.)

---

## 2. Verifying the robot's structure

Everything downstream depends on one property, so it was verified numerically
against the URDF rather than assumed.

### 2.1 The result

**The arm is S-R-S** — spherical shoulder, revolute elbow, spherical wrist.

- Joints 1, 2, 3 intersect at a single point: the shoulder, at `(0, 0, 0.1695)`.
  Pairwise common-normal distance between all three axes: **0**.
- Joints 5, 6, 7 intersect at a single point: the wrist centre.

Upper arm `L1 = 0.1155 m`, forearm `L2 = 0.1278 m`, so the wrist centre lies
between **0.1347 m and 0.2433 m** from the shoulder.

### 2.2 Why the geometry is deceptive

Both intersections are hidden by offsets that look like they break them:

- joint3's origin is `xyz="0 -0.1155 0"` — but that offset lies **along joint3's
  own axis direction**, so the axis line still passes through the shoulder.
- joint7's origin is `xyz="0 0.066 0"` — same trick, so the axis still passes
  through the wrist centre.

A naive "are the offsets zero?" check gives the wrong answer for both. The
correct test is the common-normal distance between axis *lines*, which is what
was computed.

### 2.3 The two consequences that carry everything

1. **The wrist centre is a function of q1–q4 alone.** Joints 5–7 change
   orientation and nothing else. Position planning is therefore a 4-DOF problem —
   which is exactly what the professor asked for, and it is exact rather than an
   approximation.
2. **|W − S| depends on q4 alone**, because a spherical shoulder can rotate the
   upper-arm/forearm triangle but never change its shape. The elbow angle falls
   out of one law of cosines.

Folding in the URDF's fixed rpy offsets, both joint triples turn out to be **ZYZ
Euler sets**:

```
R_shoulder = Rz(q1) Ry(q2) Rz(q3)
R_wrist    = Rz(q5) Ry(-q6) Rz(q7)
```

so both invert in closed form. `Rx(-π/2) Rz(q2) Rx(π/2) == Ry(q2)` is the
identity that makes the shoulder one; the wrist one is the same shape.

### 2.4 Validation

| check | result |
|---|---|
| analytic FK vs MoveIt FK (wrist centre) | max error **1.3 µm** |
| position IK round-trip, 5000 random states | **5000 / 5000**, residual **0 m** |
| wrist ZYZ IK round-trip, 3000 random states | **3000 / 3000**, residual 1e-5 |

The 1.3 µm residual is **not** slack in the derivation. The URDF writes every
right angle as `1.5708`, which is 3.67×10⁻⁶ rad short of π/2. Replaying the same
FK with exact π/2 brings the disagreement to **1×10⁻¹⁶ m**. The closed form is
exact; the URDF is rounded. (`session_notes.md` §3 reaches the same conclusion
independently from the axis-intersection side, predicting 2.424×10⁻⁷ m over the
66 mm lever and matching to four significant figures.)

`SrsKinematics`'s constructor runs this self-test at startup and **throws** if it
fails, with tolerances set well above the rounding artifact and well below
anything a genuine convention change could produce. If the URDF is ever
regenerated with different frame conventions, the system fails loudly instead of
moving quietly wrong.

### 2.5 Joint 3 is the redundancy

Voxelising the reachable wrist-centre set at 15 mm over 3 M samples:

| | voxels |
|---|---|
| q3 free (4-DOF) | 16312 |
| q3 locked at 0 (3-DOF) | 15655 |

A **5 % loss**, entirely at the workspace boundary. So `{q1, q2, q4}` is a
genuine 3-DOF positioner, and the professor's lower number is viable. This
measurement is what justified offering a 3-DOF variant at all.

---

## 3. Approach A — the swept wrist volume

The professor's instruction 3, implemented literally.

### 3.1 The idea and the immediate problem

Replace everything distal of the wrist centre — link5, link6, link7, flange,
gripper, and the carried tube — with one rigid body enclosing every pose those
parts can take. Plan over q1–q4 against that body, and any accepted path is
collision-free for the real arm whatever the wrist does.

Sweeping *all* wrist orientations gives a sphere of radius **~0.27 m** around the
wrist centre, on an arm whose entire reach is **0.2433 m**. Dead on arrival — it
collides with the table everywhere.

The task's own constraint is what rescues it. While carrying, the tool axis must
stay vertical (`uprightConstraint()`), which leaves exactly **one** free degree of
freedom: roll about that axis. Sweeping only that gives a flat **annulus** rather
than a sphere.

### 3.2 The frame problem, and why the reduced URDF has two extra joints

The annulus is a solid of revolution about the **world** vertical, because the
constraint is stated in the world frame — the tube stays upright wherever the arm
is.

That means it cannot be welded to link4. A link4-fixed body would rotate as link4
rotates, into orientations the tube never actually takes, and to stay correct you
would have to fall back to the useless sphere.

So in the generated reduced model the blob hangs off `wrist_center` through two
**continuous "align" joints**:

```
wrist_center --[blob_align_x]-- blob_align_link --[blob_align_y]-- wrist_blob
```

They are marked `passive_joint`, are in no planning group, and the sampler never
touches them. The state validity checker sets them at every state so the blob's
own z axis points along world z. Two joints suffice because the blob is a solid
of revolution about that axis — the third rotation is the symmetry axis and is
free. Reading it off directly: with `v = R_wc^T · e_z`, then `b = asin(v_x)` and
`a = atan2(−v_y, v_z)`.

Verified: the blob's z axis stays on world z to **1e-16** across 2000 random arm
poses.

### 3.3 The generator, and four bugs found building it

`tools/wrist_blob_gen.py` sweeps the real collision geometry out of the URDF
(binary STL meshes for the links, boxes for the gripper) and emits a revolved
STL, a xacro cylinder stack, and a JSON profile.

Bugs found while building it, all of which inflated the blob:

1. **The six gripper joints all `<mimic>` `endeffector_gripper`** — the hand is
   one degree of freedom, not six. Sweeping them independently let the fingers
   reach places the mechanism physically cannot put them.
2. **The tube cylinder was built z-aligned**, but the tube's long axis is the
   TCP's **x** axis. That put its 130 mm length along the wrong axis and added
   ~35 mm of phantom radius.
3. **XML comments containing `--`** are invalid. `<!-- GENERATED ... -- do not
   edit -->` produced malformed output that `minidom` rejected.
4. A Python scoping bug (`tris +=` inside a nested function makes it local).

Worth recording the *derivation* fixed by bug 2, because the same offset is used
in three places. `simple_move.cpp` expresses the tube's position in the **world**
frame as `(0, −0.135, −0.030)` from the TCP, which is only correct at the one
specific upright yaw it always uses. Rotating that into the TCP frame through
`q_upright = RPY(0, −π/2, π/2)`:

```
R = Rz(π/2) Ry(−π/2)  =>  R·e_x = +Z_world,  R·e_y = −X_world,  R·e_z = −Y_world
world (0, −0.135, −0.030)  =>  tcp (−0.030, 0, +0.135)
```

The `+0.135` along the TCP's z is the **same direction** as the wrist-centre →
flange offset (0.066), so the two **add**: the tube sits **0.201 m** from the
wrist centre. That number is the whole story of why Approach A fails, and it
matches `flange_offset_ (0.066) + gripper (0.135)` computed independently by the
manifold planner.

### 3.4 Measured geometry

| | constraint-aware (tilt 0.30) | naive full SO(3) |
|---|---|---|
| max radius from wrist centre | 236.6 mm | 238.2 mm |
| height span | −153.7 … +96.4 mm (250 mm) | −176.1 … +231.3 mm (407 mm) |
| volume | 26.4 L | 49.3 L |

The constraint buys **1.9× less volume**, essentially all of it vertical. The
radius is unchanged, because the ring's width is set by the gripper-plus-tube
reach, which no constraint can shrink.

Sensitivity to `TILT_TOLERANCE`:

| tolerance | max radius | volume |
|---|---|---|
| 0.05 rad | 222.5 mm | 16.3 L |
| 0.10 rad | 226.4 mm | 20.0 L |
| 0.20 rad | 232.6 mm | 23.0 L |
| 0.30 rad | 236.6 mm | 26.3 L |

Keeping the ring's central hole matters. Emitting the blob as a revolved mesh
rather than a solid cylinder stack recovers about a third of the lost workspace
in the band that matters: at wrist height 0.28 m, 61.6 % → 75.0 %; at 0.30 m,
81.5 % → 91.0 %. (MoveIt/FCL handles concave BVH meshes correctly, so the hole
survives; a cylinder stack necessarily fills it.)

### 3.5 The reduced model generator

`tools/make_reduced_model.py` reads the full URDF and emits
`generated/reduced_arm.urdf` + `.srdf`. Generated rather than hand-written so
joints 1–4 stay bit-identical to the real robot — hand-copying once is fine,
keeping them in sync after a URDF change is not.

Verified: the generated `wrist_center` frame matches the full model's wrist
centre to **0.0 m** over 2000 random configurations.

It also emits `generated/preview_arm.urdf` — the *whole* real arm plus the blob —
for the RViz preview (§3.7).

Two SRDF groups: `arm_positioning` (joints 1–4) and `arm_positioning_3dof`
(joints 1, 2, 4). All `disable_collisions` pairs among the kept links are carried
over from the real SRDF, plus the blob is disabled against every arm link — on
the real robot the wrist assembly cannot reach any arm link either (every
link7/gripper pair in the full SRDF is already marked `Never`), and leaving them
enabled would make the blob, which always engulfs link4, report a permanent
self-collision.

### 3.6 The planner and its offline results

`ReducedPlanner` drives OMPL RRTConnect directly over a `RealVectorStateSpace` of
3 or 4 dimensions, with the same `longest_valid_segment_fraction` as
`ompl_planning.yaml` so the two pipelines check motions at the same resolution.

One design point worth recording: `CollisionRequest::group_name` is left **empty
on purpose**. The blob is not a member of the planning group, and a group-scoped
request would skip exactly the link the whole model exists for.

Goal generation is where most of the speed comes from: one Cartesian target maps
to a whole family of joint configurations (two elbow branches × two shoulder
branches × a continuum of arm angles), and all of them are handed to OMPL as an
`ob::GoalStates`. In the offline test that is 48 goal states for a single target.

Offline, in isolation, it is fast:

| | plan time | collision checks | goal states |
|---|---|---|---|
| 4-DOF (`arm_positioning`) | 2.6 ms | 173 | 48 |
| 3-DOF (`arm_positioning_3dof`) | 1.5 ms | 87 | 1 |

Workspace admitted by the blob, by wrist height (mesh geometry):

| wrist z | 4-DOF | 3-DOF |
|---|---|---|
| 0.18 m | 0 % | 0 % |
| 0.20 m | 53.6 % | 55.3 % |
| 0.22 m | 54.7 % | 55.9 % |
| 0.24 m | 57.8 % | 57.7 % |
| 0.26 m | 66.3 % | 65.6 % |
| 0.28 m | 75.0 % | 74.0 % |
| 0.30 m | 91.0 % | 90.5 % |
| ≥ 0.32 m | 100 % | 100 % |

**The 3-DOF variant costs essentially nothing** relative to 4-DOF — same admitted
percentage within 1 %, and 1.7× faster. That answers the professor's "3 or 4?"
directly: 3 is viable here.

### 3.7 A scoping finding that turned out to be the whole story

The blob reaches **154 mm below** the wrist centre, so it clears the table only
when the wrist is above ~144 mm. At grasp height (wrist ≈ 110 mm) it always
reports a table collision — correctly, because it contains the swept tube and the
real tube genuinely is at table level.

At the time this was read as a scoping result: the reduced planner covers
**transit**, not approach/grasp/retreat, which happens to match the existing code
structure (`moveFreeSpaceUpright` for transit, Cartesian interpolation for the
short segments, which never used OMPL).

The RViz preview (`preview_reduced.cpp` + `preview_reduced.launch.py`) makes this
visceral: it shows the *real* 7-DOF arm with the carried tube inside the
translucent swept ring, animating along actual reduced-planner paths. The point
of showing both together is that the gripper and tube must never leave the ring —
that containment is the assumption the whole approach rests on.

Building the preview produced its own finding. Hardcoding the task's tube
stations made the node refuse to start: at 0.24 m no configuration above the rack
is admissible. The preview now sweeps the workspace and auto-picks admissible
targets — at z = 0.30 there are 295 of them, at 0.24 far fewer, below ~0.20 none.

### 3.8 The verdict: 0 / 30

On the actual task transits — the arm's own stations, at its own working height —
Approach A scored **0 / 30**.

The cause is geometric and specific to this robot. The gripper plus carried tube
reaches **201 mm** from the wrist centre, on an arm with **243 mm** of total
reach. The swept ring is therefore nearly as wide as the robot. Parked above the
tube rack it always contains a neighbouring tube, and every configuration is
rejected.

This is a quantified negative result, not a failure of execution. The rigid-body
surrogate is sound in principle and not viable at this robot's scale — a larger
arm, or a shorter end-effector, would change the conclusion. Presenting it that
way is more useful than presenting it as a rejection of the suggestion.

---

## 4. The benchmark, and four ways it was nearly wrong

`benchmark_reduced.cpp` compares methods on identical queries against a live
`move_group`. Getting it *honest* took more work than getting it to run, and each
correction is worth recording because each would have produced a flattering but
false result.

### 4.1 The four corrections

**Infeasible queries.** The first query set used transit heights up to 0.32 m,
where the tube stations are past the arm's 0.2433 m reach. Both methods fail
there — but the reduced planner rejects in microseconds during goal IK while
`move_group` burns its entire budget first. That manufactures a large fake
speedup. Both endpoints must now be provably reachable *and* collision-free on
the full model, or the pair is dropped.

**An empty world.** `move_group`'s planning scene starts empty; it is
`simple_move` that publishes the table, tubes and mixer, and it was not running.
So the 7-DOF side was planning through open air while the reduced side planned
against real obstacles. The benchmark now publishes the scene via
`PlanningSceneInterface` and attaches the carried tube before snapshotting —
because the benchmark is about *carrying*, and the blob already contains the tube,
so the 7-DOF side must be holding one too.

**A handicapped baseline.** `simple_move.cpp` configures
`setGoalPositionTolerance(0.005)`, `setGoalOrientationTolerance(0.02)` and
`setNumPlanningAttempts(5)`. I had left MoveIt's defaults — 1e-4 m and a single
attempt, far tighter and less persistent than what actually runs. That makes the
baseline look much worse than it is. Those are now the benchmark defaults.

**A control column.** `7dof-nocon` runs the same queries with no orientation
constraint. It is not a candidate method — it is free to tip the tube, and does.
It exists to separate "the constraint is what makes this hard" from "the
benchmark is misconfigured".

One further check, because a 60 %-failing baseline is exactly the shape of a
misconfiguration: MoveIt's state-space selector was instrumented at DEBUG level
to confirm `enforce_constrained_state_space: true` actually takes effect. It logs
`ConstrainedPlanningJointModel` on every constrained request and `JointModel` on
the control. The baseline is genuinely running the constrained state space, not
silently falling back to rejection sampling. Note the parameters live under the
*pipeline namespace* (`move_group.arm_group.*` with `demo.launch.py`'s nesting,
`ompl.arm_group.*` with `fake.launch.py`'s) — a mismatch there would silently
disable the setting.

### 4.2 Results

15 station-to-station transits at z = 0.24, 2 repetitions, 10 s budget, 5
attempts, identical scene and start states.

| method | success | wall (med) | plan (med) | path len | max tilt |
|---|---|---|---|---|---|
| `7dof-strict` — the current pipeline | **60.0 %** | 10.56 s | 10.03 s | 13.82 | **0.418 rad** |
| `7dof-free` — multi-roll goal fan | 16.7 % | 18.32 s | 10.05 s | 13.69 | 0.312 rad |
| `reduced` — swept volume | **0.0 %** | — | — | — | — |
| `7dof-nocon` — control | 96.7 % | 8.26 s | **0.66 s** | 11.99 | 3.057 rad |

Three readings:

- **The constraint alone costs a 15× slowdown and 37 points of success rate**
  (96.7 % / 0.66 s → 60.0 % / 10.03 s). The control did its job.
- **`7dof-free` was my own idea and it backfired.** I added it so I could not be
  accused of handicapping the baseline by pinning the goal yaw — but handing
  MoveIt twelve pose goals appears to split the constrained goal sampler's effort
  across all of them rather than concentrating it, and success fell to 16.7 %.
  That column says something about MoveIt's goal handling, not about the
  pipeline. **The honest baseline is the 60 %.**
- **`7dof-strict`'s max tilt is 0.418 rad — above the nominal 0.30 rad
  tolerance.** Measured the same way `maxTiltAlongTrajectory()` measures it. With
  `ROTATION_VECTOR` parameterization MoveIt interprets the per-axis tolerances
  differently from a direct tool-axis angle, so this may be within spec as MoveIt
  defines it while still being a 24° tip in reality.

### 4.3 Process failures during the benchmark

Recording these because they wasted more wall-clock time than the benchmark
itself, and all three are generic.

- **Orphaned processes.** `pkill -f` self-matching (§1.4) left three benchmark
  processes running concurrently against one `move_group`, at ~2 % CPU each —
  blocked, not computing — and all writing to the same CSV. Every timing from
  those runs was meaningless and discarded.
- **Seven accumulated polling loops.** I had used `until grep "finished"; do
  sleep; done` waiters to detect completion. When the runs they watched were
  killed, the condition could never become true and the waiters polled forever —
  seven of them, the oldest at 1h34m, doing nothing. The visible "the benchmark
  takes 1h30m" was mostly this.
- **CSV buffered to nothing.** `std::ofstream` only flushes at clean exit, so
  every interrupted run left a zero-byte file. Now flushed per row, with a
  progress line and a watchdog that aborts loudly if six consecutive planning
  calls fail in under 0.5 s (a real failure burns the full budget; instant
  failures mean the action server is gone).

There is a genuine, unavoidable cost underneath all that: **a failing constrained
plan costs the entire time budget while a succeeding one costs milliseconds.**
RRTConnect returns the instant it finds a path but searches until the clock runs
out when it cannot. With a baseline failing 40 % of the time at a 10 s budget,
most calls cost 10 s. That is the measurement, not overhead.

---

## 5. Approach B — the 5-DOF exact manifold planner

### 5.1 The insight

The swept volume pays for every value the roll *might* take. But the roll was
never genuinely unknown — it was only unknown **to a 4-DOF planner**.

Promote it from an uncertainty the collision checker must absorb to a decision
the planner makes. Add it as a fifth search dimension and the whole robot becomes
determined:

```
q1..q4     ->  wrist centre                     (S-R-S decomposition)
phi        ->  tool orientation = upright(phi)
q5,q6,q7   ->  closed-form ZYZ inversion of that orientation
```

Every point of the 5-D space maps to a complete 7-DOF configuration. Three things
follow at once:

- Collision checking sees the **real** gripper and tube. No surrogate, no
  conservatism, nothing engulfing the rack.
- The upright constraint holds **exactly**, by construction, at every sample —
  not approximately by rejection sampling or numerical projection.
- The search is still 5-dimensional rather than 7; 4 with the arm angle locked.

This is planning directly on the constraint manifold using an exact analytic
chart. MoveIt's `ConstrainedPlanningStateSpace` attempts the same thing
numerically by projection, and on this arm that path succeeds 60 % of the time.

`session_notes.md` §4 has the cleaner framing for the write-up: both approaches
reduce the *arm* search to 4 joints, and the difference is entirely what happens
to φ — swept (0/30) versus searched (100 %).

### 5.2 Four implementation bugs, in the order they were found

**(a) Unstable wrist branch index.** `wristIk()` drops out-of-limit ZYZ branches,
so `sols[0]` can be the positive branch at one state and the negative branch at a
neighbour. With the roll as a *state dimension* that makes the state →
configuration map discontinuous, and the solution path jumps between wrist
postures mid-edge. Added `wristIkBranch()` with a stable index (0 = `+acos(M22)`,
1 = `−acos`), and the planner fixes the branch **once per plan, from the start
state** — which also guarantees the plan begins in the configuration the arm is
actually in.

**(b) A quarter-turn convention error — mine.** The wrist-centre → flange
direction is `(sin φ, −cos φ, 0)`, **not** `(cos φ, sin φ, 0)`: `q_upright`
maps the tool's z axis onto world −y, so the circle is a quarter turn out of
phase. `rollFor()` was therefore returning `φ − π/2`.

This is silent when φ is swept over a full circle, because the *set* of wrist
centres is identical either way — which is why it never affected `ReducedPlanner`.
It only bites when a specific φ must correspond to a specific wrist centre, which
is exactly what a planner carrying φ as a state dimension needs. The symptom was
`RRTConnect: Motion planning start tree could not be initialized!` — the start
state failing its own validity check. There is now a single `flangeDirection()`
helper used everywhere so the convention cannot drift again.

Verified numerically before changing anything, rather than reasoned about:
`atan2(z.x, −z.y)` recovers φ exactly; `atan2(z.y, z.x)` gives φ − π/2.

**(c) The 2π wrist wrap.** `q5` and `q7` come from `atan2`, so they live in
(−π, π] and wrap. Across that wrap the **orientation** is perfectly continuous —
`Rz(q)` and `Rz(q − 2π)` are the same rotation — but the **joint value** jumps by
2π, and a controller replaying those waypoints executes a full 360° wrist spin,
sweeping the tube through every orientation on the way.

The symptom was the planner honestly reporting 0.0005° of tilt while the arm
visibly tumbled through **173.7°**. It was right about every waypoint and wrong
about the motion *between* two of them. Fixed by unwrapping the wrist joints
against the previous waypoint with a joint-limit check: **173.7° → 8.9°**.

**(d) The joint-7 seam.** Joint 7's limit is exactly ±π, so a path crossing that
seam asks it to rotate through the one point it cannot reach. My first fix caught
this only at *extraction* time, after the search — which throws away a finished
plan and falls back, at up to 19.2° of tilt from the fallback.

The seam is a property of the **edge**, not of a state or of the finished path, so
it belongs in OMPL's `MotionValidator`. A custom validator now walks each
candidate edge, unwraps the wrist against the previous state, and rejects the edge
if that would leave the joint limits. RRTConnect routes around the seam during
search. Cost ~2–3× in planning time (89 ms vs 31 ms offline); it recovers the
cases that previously fell back. **19.2° → 0.001°.**

Bugs (c) and (d) are the same underlying subtlety — *the orientation is continuous
while the joint path is not* — and both are generic to planning on an analytic
chart, not specific to this robot. Worth a paragraph in the report.

### 5.3 Results

Same 30 task transits, same scene, same carried tube:

| | success | plan time | tool tilt |
|---|---|---|---|
| swept volume (4-DOF) | 0 % | — | — |
| 7-DOF constrained | 60 % | ~10 000 ms | up to 0.418 rad |
| **5-DOF manifold** | **100 %** | **~31 ms** | **0.000 rad** |

Stable across heights, and the 4-D variant is faster again:

| transit z | 5-D | 4-D (arm angle locked) |
|---|---|---|
| 0.20 m | 100 %, 33 ms | 100 %, 26 ms |
| 0.24 m | 100 %, 29 ms | 100 %, 17 ms |
| 0.28 m | 100 %, 32 ms | 100 %, 18 ms |

The 0.20 m row matters: that is *below* the height at which Approach A can operate
at all.

It also fails fast. When a goal is genuinely impossible the manifold planner
reports it in ~20 ms, where the 7-DOF ladder spends ~3 s working through its
tight, loose and verified rungs to reach the same conclusion. Measured live on
`m1`/`m5` while carrying tube 3 — targets that would put the carried tube where
another tube already sits. **Both** planners correctly refuse; only the cost
differs.

---

## 6. Integration into `simple_move.cpp`

Deliberately small — the reduced planner replaces the internals of one function.

| added | what |
|---|---|
| `moveFreeSpaceManifold()` | sibling to `moveFreeSpaceUpright()`, same signature; plans, lifts, time-parameterises (TOTG, matching the `AddTimeOptimalParameterization` adapter), executes |
| `transitUpright()` | dispatcher: manifold first, `moveFreeSpaceUpright()` unchanged as fallback, with counters |
| `IkValidator::psm()` | accessor so the planner reuses the existing scene monitor rather than starting a second one |
| planner init in `main()` | inside try/catch, so a bad model disables the feature instead of killing the task |

Four call sites changed to `transitUpright`. **`moveFreeSpaceUpright()` itself is
untouched.** Architecturally the result cannot be worse than the original: a
manifold failure costs ~20 ms before falling back to exactly the previous
behaviour.

Runtime switches: `use_manifold:=false` restores the original behaviour exactly;
`manifold_lock_arm_angle:=true` selects the 4-D variant; `manifold_time:=<s>` sets
the budget (default 2.0).

Live results at the end of this phase, on the fake-driver stack: carrying
transits plan in **20–140 ms** with **0.0002–0.0005°** planned tilt and
**0.001–0.94°** measured executed tilt. Descents 100 %.

### 6.1 The attempt that failed, and became the next session's problem

The user asked, reasonably, why orientation-keeping only applied while carrying —
empty-gripper moves tumbled the wrist through **32.5°** (measured; the gating is
pre-existing, at `simple_move.cpp:1462` and `:2011`). The original justification
was that constrained planning cost ~10 s and failed 40 % of the time, so you only
paid for it when you had to. At 30 ms that justification is gone.

Making it unconditional was implemented, and it **broke the descent**: the transit
succeeded and the arm arrived correctly above the tube, but the following
straight-line descent stalled between 10 % and 80 % of the way down.

The correct diagnosis is in `session_notes.md` §1 — the transit was landing on the
mirror wrist branch with joint5 saturated against its limit, and nothing was ever
in collision. What I did instead is worth recording as an anti-pattern:

- I hypothesised that the *configuration at the same pose* was wrong (correct in
  outline) and that **goal-set size** was the mechanism (wrong).
- I saw `1 goal state → descent 100 %` and `627 goal states → descent fails`,
  and built **three** fixes on it: exact goal tolerances, a tight hint radius
  (0.6 → 0.12 rad), and a single-goal `planToConfiguration()`.
- The next run showed `1 goal state → 10.6 %`. **The correlation was noise.**
- I never dumped the contact pairs, which would have shown zero collisions in one
  run and killed the hypothesis immediately.

I reverted to manifold-while-carrying-only, which is the state `session_notes.md`
opens from. The reverted code paths (`planToConfiguration`, `goal_hint_radius`)
survived and were repurposed there — the hint became a *ranking* rather than a
filter, which was itself one of the five defects.

**The lesson, stated plainly for the report:** three fixes were built on an
unmeasured hypothesis, and a fourth measurement invalidated all three. The
instrumentation that settled it (`CollisionRequest::contacts = true`) cost one
run.

---

## 7. Numbers reference

Everything measured in this phase, in one place.

### Kinematics
| quantity | value |
|---|---|
| shoulder | `(0, 0, 0.1695)` |
| upper arm `L1` / forearm `L2` | 0.1155 m / 0.1278 m |
| wrist-centre reach | 0.1347 – 0.2433 m |
| wrist centre → flange | 0.066 m |
| flange → fingertips | 0.135 m |
| wrist centre → tube | **0.201 m** |
| analytic vs MoveIt FK | 1.3 µm (URDF rounding); 1e-16 with exact π/2 |
| position IK | 5000/5000, residual 0 m |
| wrist IK | 3000/3000, residual 1e-5 |
| cost of locking q3 | 5 % of wrist-centre workspace |

### Approach A
| quantity | value |
|---|---|
| blob radius / height / volume (tilt 0.30) | 236.6 mm / 250 mm / 26.4 L |
| naive SO(3) equivalent | 238.2 mm / 407 mm / 49.3 L |
| planning time 4-DOF / 3-DOF | 2.6 ms / 1.5 ms |
| task transits | **0 / 30** |

### Approach B
| quantity | value |
|---|---|
| task transits | **30 / 30** |
| plan time 5-D / 4-D | ~29–33 ms / ~17–26 ms |
| after motion validator | 100 %, 89 ms |
| planned tilt | 0.000 rad |
| live measured tilt (carrying) | 0.001 – 0.94° |
| rejection of an impossible goal | ~20 ms (vs ~3 s) |

### Baseline
| quantity | value |
|---|---|
| constrained 7-DOF | 60 %, 10.03 s, tilt to 0.418 rad |
| unconstrained control | 96.7 %, 0.66 s, tilt to 3.057 rad |

---

## 8. Files and tools produced in this phase

| path | role |
|---|---|
| `tools/wrist_blob_gen.py` | sweeps the wrist assembly, emits STL + xacro + JSON |
| `tools/make_reduced_model.py` | generates reduced URDF/SRDF and the RViz preview model |
| `include/.../srs_kinematics.hpp` `src/srs_kinematics.cpp` | closed-form S-R-S kinematics, self-testing at construction |
| `include/.../reduced_planner.hpp` `src/reduced_planner.cpp` | Approach A: 3/4-DOF swept-volume planner |
| `include/.../manifold_planner.hpp` `src/manifold_planner.cpp` | Approach B: 5-DOF exact manifold planner |
| `include/.../path_lifter.hpp` `src/path_lifter.cpp` | wrist-centre path → 7-DOF, plus full-model validation |
| `src/test_reduced.cpp` | offline validation of Approach A + coverage sweep |
| `src/test_manifold.cpp` | offline validation of Approach B |
| `src/benchmark_reduced.cpp` | head-to-head benchmark (needs `move_group`) |
| `src/preview_reduced.cpp` `launch/preview_reduced.launch.py` `config/preview_reduced.rviz` | RViz preview: real arm inside the swept volume |
| `launch/moveit_headless.launch.py` | headless stack for benchmarking |
| `doc/reduced_space_planning.md` | formal technical record of this phase |

### Reproducing the numbers

```bash
# Approach A geometry, then the reduced model
python3 tools/wrist_blob_gen.py --tag carry --tilt 0.30
python3 tools/wrist_blob_gen.py --tag naive --full-so3      # comparison
python3 tools/make_reduced_model.py --geometry mesh

# Approach A: coverage + timing (no move_group needed)
ros2 run my_robot_control test_reduced --gen <share>/generated \
  --full-urdf <urdf> --full-srdf <srdf>
ros2 run my_robot_control test_reduced --gen <share>/generated \
  --group arm_positioning_3dof

# Approach B (no move_group needed)
ros2 run my_robot_control test_manifold --urdf <urdf> --srdf <srdf> --transit-z 0.24
ros2 run my_robot_control test_manifold --urdf <urdf> --srdf <srdf> --lock-arm-angle

# Head-to-head (needs move_group)
ros2 launch my_robot_control moveit_headless.launch.py
ros2 run my_robot_control benchmark_reduced --ros-args \
  -p gen_dir:=<share>/generated -p max_queries:=15 -p reps:=2

# RViz preview
ros2 launch my_robot_control preview_reduced.launch.py transit_z:=0.30
```

`test_reduced` and `test_manifold` build the scene in-process and need no robot,
no `move_group` and no RViz — they are the fastest way to re-derive most of the
numbers above.

---

## 9. Methodological points from this phase

Complementing `session_notes.md` §7, which reaches several of the same
conclusions independently.

- **Every unmeasured hypothesis in this phase was wrong at least once.** The
  goal-set-size theory (three fixes built on noise); "the fallback succeeds where
  the manifold fails" (it did not — both failed, I had not checked); the tube
  offset direction (reasoned, wrong, then verified numerically and corrected).
  The pattern is consistent enough to be worth stating as a finding.
- **Verify the instrument before trusting the measurement.** The tool-tilt
  monitor was checked against TF (`axis_diff = 0.000000`) only after I had
  already reported numbers from it. One earlier figure (32.5°) came from an
  unvalidated instrument on a polluted scene and should not be cited.
- **Identical output after a real change means the change is not running.** The
  ABI mismatch (§1.3) presented as byte-identical diagnostic counts.
- **A negative result needs the same rigour as a positive one.** Approach A's
  0/30 is only worth reporting because the benchmark's four fairness corrections
  were made first — otherwise it would be indistinguishable from a bad
  implementation.
- **Conservative surrogates scale badly with end-effector length.** The general
  lesson from Approach A: the swept volume grows with the *payload* reach, not
  the arm's, so the approach gets worse exactly where the end-effector is long
  relative to the arm. 201 mm on 243 mm is fatal; the same method on an
  industrial arm with a short tool would be fine.

---

## 10. Open items carried out of this phase

Superseded items are marked; the rest were still open when `session_notes.md`
began.

- ~~Empty-gripper orientation keeping breaks the descent~~ — **diagnosed and
  fixed** in the later session (`session_notes.md` §1–2). Cause was wrist-branch
  saturation at joint5's limit, never a collision.
- The manifold planner fixes the wrist branch **per plan by design**, so a
  transit whose start and goal lie on opposite branches cannot be represented and
  falls back. A branch-changing chart through the `q6 = 0` singularity is
  possible future work. (Also recorded in `session_notes.md` §2.7 with measured
  numbers at tube 3's standoff.)
- The three pre-existing environment issues in §1.5 — SRDF stray character, Pilz
  adapter names, `use_sim_time` mismatch — were never fixed here. The later
  session modified both launch files for other reasons; worth re-checking whether
  these remain.
- Approach A's blob is retained and buildable but unused. Keeping it is
  deliberate: it is the measured negative result, and the tooling regenerates
  from the URDF, so it stays valid if the robot changes.
