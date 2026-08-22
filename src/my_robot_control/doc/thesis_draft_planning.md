# Reduced-dimension constrained motion planning for a redundant arm with a spherical wrist

*Draft chapter text. Written to be adapted, not quoted verbatim.*

---

## 1. Problem

The manipulator is a 7-DOF arm carrying an open test tube between stations. The
liquid imposes a hard requirement on the whole motion, not merely on its
endpoints: the tube's axis must remain vertical throughout. In planning terms
this is a **path constraint** on the end-effector orientation, and it changes the
character of the problem completely.

Sampling-based planners such as RRTConnect explore the configuration space by
drawing random samples and connecting them. A path constraint makes almost every
such sample invalid, because the constraint surface is a measure-zero subset of
the space being sampled. Planners handle this either by rejection sampling —
discarding samples that violate the constraint — or by numerically projecting
samples onto the constraint surface. Both degrade badly as the tolerance
tightens.

The effect is easy to quantify by planning the same set of transfers with and
without the constraint. Unconstrained, the queries succeed 96.7 % of the time in
a median of 0.66 s. With the upright constraint enforced in the full 7-DOF joint
space, success falls to 60.0 % and the median planning time rises to 10.03 s — a
15-fold slowdown for a problem that is otherwise easy. Worse, the constraint is
only satisfied approximately: the resulting trajectories tilt the tube by up to
0.418 rad (24°), against a nominal tolerance of 0.30 rad.

The constraint, in other words, is the entire difficulty. The work described here
removes it from the search rather than enforcing it during the search.

## 2. Kinematic decomposition

The arm has the classical **S-R-S** structure — a spherical shoulder, a single
revolute elbow, and a spherical wrist. This is not an approximation adopted for
convenience; it is a property of the mechanism, and it was verified directly from
the kinematic model by computing the mutual distances between the joint axis
lines. Joints 1, 2 and 3 intersect at a common point, the shoulder; joints 5, 6
and 7 intersect at a second common point, the wrist centre. In both cases the
pairwise common-normal distance is zero.

The geometry is easy to misread. Both triples contain link offsets that appear to
break the intersection — the wrist, for instance, carries a 66 mm offset between
the last two joints. That offset lies *along* the axis direction of the joint it
belongs to, so the axis line still passes through the common point. Testing
whether the offsets vanish gives the wrong answer; testing the distance between
the axis lines gives the right one.

Two consequences follow, and together they carry the whole method.

**The wrist centre depends only on the first four joints.** Joints 5–7 rotate the
tool about a fixed point and change nothing else. Positioning is therefore a
four-dimensional problem embedded in a seven-dimensional one.

**The distance from shoulder to wrist centre depends only on the elbow.** A
spherical shoulder can reorient the upper-arm/forearm triangle but cannot change
its shape, so the elbow angle follows from a single application of the law of
cosines. With link lengths 0.1155 m and 0.1278 m, the wrist centre is confined to
a spherical shell of inner radius 0.1347 m and outer radius 0.2433 m.

Once the model's fixed frame rotations are folded in, both joint triples reduce to
**ZYZ Euler sets**,

    R_shoulder = Rz(q1) Ry(q2) Rz(q3)
    R_wrist    = Rz(q5) Ry(−q6) Rz(q7)

so both admit closed-form inversion. This is the standard Pieper decoupling:
position is solved with the arm, orientation with the wrist. The arm joints do
influence the final tool orientation, since they rotate the forearm frame, but a
spherical wrist can always cancel that contribution — which is precisely why the
decoupling holds.

Among the first four joints only three are needed to place a point in space, so
one degree of freedom is redundant. It corresponds to the familiar *arm angle*:
the elbow's rotation about the shoulder–wrist axis, which sweeps the elbow through
a circle while the wrist centre stays fixed. Locking it reduces the reachable
wrist-centre volume by roughly 5 %, all of it at the workspace boundary.

## 3. First approach: bounding the wrist conservatively

The natural way to exploit the decomposition is to plan only the wrist centre,
over the first four joints, and to make the collision check independent of the
last three. Since those joints are unknown to such a planner, everything distal of
the wrist centre — the wrist links, the gripper and the carried tube — is replaced
by a single rigid body that encloses every pose those parts can assume. Any path
accepted against that body is collision-free for the real arm, whatever the wrist
does. The guarantee is one-sided but sound: the surrogate is conservative, so it
may reject feasible paths but can never accept an infeasible one.

Enclosing *all* wrist orientations, however, produces a sphere whose radius
approaches the arm's own reach, which collides with the environment almost
everywhere. The task constraint is what makes the idea tractable. If the tool axis
must stay vertical, the wrist retains exactly one degree of freedom — rotation
about that vertical axis — and the swept set collapses from a ball to a flat
annulus. Measured on this manipulator, imposing the constraint reduces the swept
volume from 49.3 L to 26.4 L, and the reduction is almost entirely in height: the
body becomes 250 mm tall instead of 407 mm, while its radius is essentially
unchanged at 237 mm.

That the swept set is a solid of revolution about the *world* vertical, rather
than about any axis fixed in the forearm, is an important structural detail. The
constraint is expressed in the world frame, so the surrogate must retain its
orientation as the arm moves; rigidly attaching it to the forearm would be
incorrect and would force a return to the useless spherical bound.

The approach is principled, and on this manipulator it does not work. The reason
is a ratio rather than a flaw in the construction: the gripper and tube extend
**201 mm** beyond the wrist centre on an arm whose total reach is **243 mm**. The
swept body is therefore nearly as wide as the robot itself. Positioned above the
tube rack it invariably encloses a neighbouring tube, and every candidate
configuration is rejected. Across the task's own transfers, the method solved none.

This is a useful negative result, and it generalises. The conservative bound grows
with the *end-effector's* reach, not the arm's, so the technique degrades exactly
where the payload is long relative to the manipulator. On an industrial arm with a
compact tool it would be entirely serviceable; here it is not.

## 4. Second approach: exact reduction by parameterising the residual freedom

The failure above has a precise cause. The surrogate pays, at every configuration,
for every value the wrist's remaining freedom *might* take. But that freedom is
not genuinely unknown — it is unknown only to a planner that does not represent
it.

The remedy is to promote it from an uncertainty the collision checker must absorb
into a decision the planner makes. Let φ denote the tool's rotation about the
vertical, the single orientational freedom the constraint leaves. Then a search
over

    (q1, q2, q3, q4, φ)

determines the entire manipulator: the first four coordinates fix the wrist
centre, φ fixes the tool orientation, and the remaining three joints follow from
the closed-form ZYZ inversion of that orientation. Every point of this
five-dimensional space maps to a complete seven-joint configuration.

Three properties follow immediately, and they are the reason this is the right
construction rather than merely a working one.

**The constraint is satisfied identically.** Every sample is drawn from the set of
configurations that already satisfy it, so no rejection, projection or tolerance
is involved. The tube is exactly vertical by construction, not vertical to within
some bound. The residual tilt measured on planned trajectories is zero.

**The collision check becomes exact.** Because the mapping yields the full
configuration, the checker tests the real gripper and the real tube. The
conservatism that defeated the first approach disappears entirely — there is no
surrogate left to be conservative about.

**The dimension is forced, not chosen.** Two independent arguments agree. The
constraint fixes two of the three orientational degrees of freedom, leaving
7 − 2 = 5. Equivalently, three coordinates locate the wrist centre, one is the
redundant arm angle, and one is φ. Locking the arm angle yields a genuine
four-dimensional search at the 5 % workspace cost noted earlier.

This is planning on the constraint manifold using an exact analytic chart. General
constrained planners construct such a chart numerically, by projection; the
kinematic structure of this arm supplies one in closed form, which is why the
method is both cheaper and exact where the general technique is expensive and
approximate.

## 5. Continuity of the chart

A chart is only useful if the mapping from chart coordinates to joint values is
continuous, and here it is not automatically so. Two failures arise, both of the
same character: the *orientation* varies smoothly while the *joint trajectory*
does not.

The wrist angles are recovered through inverse trigonometric functions and
therefore lie in a half-open interval. A path that crosses the branch cut produces
an orientation that is perfectly continuous — a rotation and the same rotation
less a full turn are identical — while the joint value jumps by 2π. Executed
literally, such a trajectory spins the wrist through a complete revolution,
sweeping the tube through every orientation on the way. The planner reports zero
tilt and is correct about every waypoint; the defect lies strictly between them.
Unwrapping the recovered angles against the preceding waypoint removes it.

The second failure is subtler. The ZYZ inversion admits two solution branches, and
the joint whose limit spans exactly one turn cannot pass through the point where
the branches exchange. A path requiring that passage is not executable at all.
Detecting this after planning merely discards a finished solution; the correct
place for the test is the *edge* between samples, since traversability is a
property of the connection rather than of either endpoint. Enforcing it in the
motion validator lets the planner route around the obstruction during search
instead of failing afterwards.

Both effects are generic to planning on an analytic chart of a constraint manifold
and are not specific to this manipulator.

## 6. Results

Planning the task's transfers under the upright constraint:

| method | success | planning time | tool tilt |
|---|---|---|---|
| conservative swept volume, 4-D | 0 % | — | — |
| constrained search in full joint space, 7-D | 60 % | ≈ 10 s | up to 0.418 rad |
| exact manifold chart, 5-D | **100 %** | **≈ 31 ms** | **0** |

The improvement is roughly three orders of magnitude in planning time, with a
higher success rate and exact rather than approximate constraint satisfaction. The
four-dimensional variant, obtained by locking the arm angle, is faster again.

A secondary benefit is the cost of failure. When a target is genuinely
unreachable, the manifold formulation establishes this in tens of milliseconds
during goal generation, whereas a constrained search in the full space must
exhaust its time budget before concluding the same thing.

## 7. Limitations

Holding the wrist branch fixed for the duration of a plan is what makes the chart
continuous, and it is also its principal limitation: a motion whose start and goal
lie on opposite branches cannot be represented within a single chart, and must be
delegated to a general constrained planner. The two branches meet at the wrist
singularity, so an atlas of two charts joined there would remove the restriction —
a natural extension.

The method also assumes the decoupling holds exactly. It does so here by
construction, but a wrist whose axes only approximately intersect would introduce
a position error in the recovered configuration that grows with the offset.

Finally, the reduction is specific to constraints that leave a *parameterisable*
residual freedom. An upright axis leaves a single rotation and is therefore ideal;
a constraint leaving a two-dimensional residual set would enlarge the chart
accordingly, and one leaving an irregular set would not admit this treatment at
all.
