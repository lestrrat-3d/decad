# Linkage Check Design

How `Document.VerifyLinkage` answers "does any link of this mechanism hit a fixture, or another link, while
the joints move?" without changing the document: where the capability sits (§1), the linkage vocabulary
(§2), the entry point (§3), the report (§4), what a pose and an interval prove for a chain of joints (§5),
the procedure (§6), coverage (§7), errors (§8), what is deferred and why (§9), cost (§10), required tests
(§11), increments (§12), settled points (§13), the joint-box check `Document.VerifyJointBox`, which
proves a whole box of joint values clear cell by cell (§14), and closed loops — a four-bar, a
slider-crank — whose dependent joints are read from `sketch`'s certified enclosure (§15), and the joint
box over such a loop (§16). Companion to
`docs/motion-check-design.md` ("motion §N"),
which owns the one-mover check this design generalises and every proof piece it reuses; to
`docs/api-design.md` ("core §N"); to `docs/verification-design.md` ("verification §N"), which owns `Status`,
`Diagnostic` and the result vocabulary; and to `docs/clearance-design.md` ("clearance §N") and
`docs/interference-design.md` ("interference §N"), which own the pair kernels every pose runs. Nothing here
changes those contracts.

## 1. Scope: the layer above `VerifyMotion`, inside the root package

Motion §1 lists "kinematic chains, joints, linkages, more than one independent motion" as out of scope for
`VerifyMotion` and sends joints here. **`VerifyLinkage` is that layer.** It takes a tree of
links joined by revolute and prismatic joints, a one-parameter drive that moves every joint through a
stated range, and proves, pair by pair and interval by interval, that no link meets a static body or another
link anywhere along the drive — or finds where one does.

**It lives in the root package**, in `linkage.go`, `linkage_verify.go` and `linkage_bound.go`, beside
`motion.go`, and not in a subpackage like `dynamics/`. The reason is what a pose evaluation needs: the
transient placement of a payload under a composed transform (`payload.placed` with the reserved
`transientProducer`, motion §5.1), the clearance kernel with its per-call carrier cache
(`clearancePairCached`), the overlap transfer (`transferredOverlap`) and the exact ideal-pose enclosure
`η` is charged against (`internal/motionbound.IdealPose`). `dynamics/` sits above the root because the
public queries it consumes — `SweepPair`, `ContactPair`, `SweptBox` — already prove what its payload
families need. A linkage check needs `VerifyMotion`'s whole kernel reach, and the only way to give a
subpackage that reach is a public primitive that evaluates several bodies under stated transforms and
takes a rational-interval ideal pose per body, which core §5 forbids as a public shape. Should such a
primitive ever be designed, the vocabulary of §2 moves with it unchanged. No new `go.mod` module is added.

The layering rule holds: `decad -> sketch -> r3 -> units`. Every joint pose is `r3.RotationAround` or
`r3.Translation`, composed with `Transform.Then`; decad hand-rolls no rotation. v1 imports nothing new from
`sketch` for a tree, which drives every joint explicitly; a looped linkage imports `sketch.Enclose` (§15).

| Out of scope for v1 | Where it goes |
|---|---|
| Closed loops (a four-bar, a slider-crank) | §15: a tree plus a closure joint, the dependent joints read from `sketch.Enclose` |
| A box of joint values (a multi-DOF workspace sweep) | §14: `VerifyJointBox`, a second entry point over the same travel bound |
| Screw, cylindrical, spherical and planar joints; a joint driven by a caller-supplied transform | a later addition to the sealed `Joint` set |
| Dynamics, forces, time | the `dynamics` subpackage |
| Drawing the mechanism | a module above decad that calls `Linkage.PoseAt` (§2.4) |

## 2. The vocabulary

The words follow mechanism engineering, not animation: a **linkage** is a tree of **links** joined by
**joints**; `kinetograph`'s `Rig` and `Node` are the animation-side names for the same tree and are not
reused.

### 2.1 Links and joints

```go
// Linkage is a tree of links joined by joints, rooted at the ground link. A
// link is created only through its parent, so the tree has no cycles. Every
// body of the Document that belongs to no link is static.
type Linkage struct { /* private */ }

func NewLinkage() *Linkage
func (l *Linkage) Ground() *Link
func (l *Linkage) Links() []*Link // every link but the ground, in creation order

// Link is one rigid set of bodies and the joint that attaches it to its parent.
type Link struct { /* private */ }

func (p *Link) Revolute(center, axis r3.Vec, bodies []*Body, opts ...JointOption) (*Link, error)
func (p *Link) Prismatic(dir r3.Vec, bodies []*Body, opts ...JointOption) (*Link, error)
func (k *Link) Parent() *Link // nil for the ground
func (k *Link) Joint() Joint  // nil for the ground
func (k *Link) Bodies() []*Body

// Joint is sealed: RevoluteJoint and PrismaticJoint are its only members.
type Joint interface{ joint() }

// RevoluteJoint rotates its link about the axis through Center along Axis,
// right-handed, by the joint's value, an Angle. Center and Axis are world
// coordinates at the zero pose.
type RevoluteJoint struct {
    Center, Axis r3.Vec
    Limits       *JointLimits // nil when none were declared
}

// PrismaticJoint slides its link along Dir by the joint's value, a Length.
type PrismaticJoint struct {
    Dir    r3.Vec
    Limits *JointLimits
}

type JointLimits struct{ Min, Max units.Value } // the joint's Kind; Min < Max
func WithJointLimits(minimum, maximum units.Value) JointOption
```

**The zero pose is the document as it stands.** Every joint value is `0` when the linkage is built, and at
that pose every body sits exactly where `Document.Bodies()` has it. A joint's `Center`, `Axis` and `Dir` are
therefore stated in world coordinates at the zero pose, which is the same convention `Revolute` and
`Prismatic` use (motion §2): a caller who can state a `Revolute` for one body can state a joint. A body
that is rigidly attached to a link — a gripper bolted to a wrist — is listed among that link's bodies;
there is no `Fixed` joint, because at the zero pose a constant transform would move nothing the document
does not already show.

A `Revolute` or `Prismatic` call refuses, in this order: a nil parent or a parent of another linkage
(`ErrDegenerate`); an empty `bodies`, a nil body, or a body already listed in a link of this linkage
(`ErrDegenerate`); a non-finite `center`, `axis` or `dir` component (`ErrNotFinite`); a zero `axis` or
`dir` (`ErrDegenerate`); a nil option (`ErrDegenerate`), limits of the wrong `Kind` (`ErrUnitKind`),
non-finite (`ErrNotFinite`) or with `Min >= Max` (`ErrDegenerate`). Limits and joint values compare as the
exact quantities they denote (`motionbound.MotionParam`): a degree limit against a radian sweep end is
signed through `π`'s enclosure, and a difference that enclosure cannot sign is refused. Liveness and document membership are not checked here; `VerifyLinkage`
checks them against the document it is called on (§3). Limits that exclude `0` are legal: the zero pose
then lies outside the joint's working range, and only a drive that visits it is refused (§2.4).

### 2.2 Declared joint contacts

Adjacent links meet at their joint by construction: a pin sits in its bore, a slider rides in its rail,
a stacked arm rests on its pivot's cap. Those pairs touch or sit at a clearance fit at every pose, so the
interval certificate of §5.2 can never close over them, and a report that evaluated them would read
`Suspect` for every real mechanism. The caller says which pairs those are:

```go
// DeclareJointContact names a pair of bodies that meet at a joint by
// construction. The pair is checked for proven overlap at every evaluated pose
// and contributes to no interval certificate (§5.4). a and b MUST belong to
// different links, or one to a link and the other to no link.
func (l *Linkage) DeclareJointContact(a, b *Body) error
```

The declaration is explicit and visible: `LinkageReport.JointContacts` lists every declared pair, and §5.4
states exactly what is and is not claimed about one. Nothing is inferred from geometry — two bodies that
touch at the zero pose and were not declared are evaluated like any other pair, and read `Suspect` or
`Interfering` as the kernel finds them. A declared pair is never exempt from the overlap proof: a pin whose
head reaches the bore's flange at some pose is a `Collision` there.

`DeclareJointContact` refuses a nil body, two bodies of one link, two bodies that both belong to no link,
a pair declared twice in either order, and `a == b`, each with `ErrDegenerate`; `Linkage.JointContacts`
returns the declared pairs in declaration order. Liveness and document membership
are checked at `VerifyLinkage`.

### 2.3 The drive

```go
// Drive is a one-parameter motion of the linkage, parameterised by the
// Dimensionless fraction s ∈ [0, 1]: each listed joint passes through its
// waypoints From, Via…, To in order, linearly between consecutive ones, and
// an unlisted joint holds 0.
type Drive []JointSweep

type JointSweep struct {
    Link     *Link         // the link whose joint this moves
    From, To units.Value   // the joint's Kind: Angle for a revolute, Length for a prismatic
    Via      []units.Value // the joint's value at each interior waypoint, in order; the same Kind
}
```

**Waypoints and segments.** A drive whose sweeps carry `n − 1` `Via` values each has `n + 1` waypoints and
`n` segments; every listed sweep of one drive MUST carry the same number. Write a joint's waypoints
`w_0 = From`, `w_j = Via[j − 1]`, `w_n = To`. Segment `j` covers `s ∈ [j/n, (j+1)/n]`, an equal share each,
and in it the joint's value is

```text
q(s) = w_j + t·(w_{j+1} − w_j),    t = n·s − j
```

The two segments that meet at a waypoint give it the same value, so `q` is continuous; the fraction `j/n`
takes segment `j`, and `s = 1` the last. A drive with no `Via` is one segment, and `q(s) = From + s·(To −
From)`, exactly as a `Between`'s parameter is a fraction of its path (motion §2). A lift, then a swing, then
a lowering is one drive of three segments: the lifting joint's sweep is `0 → 20 → 20 → 0` mm and the
swinging joint's `0° → 0° → 90° → 90°`. A sweep whose `Via` values lie, in order and in `From`'s unit,
exactly on the line from `From` to `To` passes the same path as the plain sweep: every label, exact value
and bound of §2.4 and §5 is the same, and so is the report but for its `Drive` (§11 scene 5).

The share is equal, not weighted, because `s` orders poses and sets where the resolution floor samples, and
no claim depends on it: the report speaks for the path, not for time. A weight per segment would only move
the floor's sample budget between segments. A caller who wants one stretch of the motion sampled finer
splits it with more waypoints on the same line.

The parameter every report record carries is `units.Scalar(s)`, a dyadic fraction, exact in float. Each
joint's value is a label, computed as `motionSpec.label` computes a `Between` pose's parameter on the segment
from `w_j` to `w_{j+1}` at `t`, and carried in `w_j`'s unit. Every bound reads the exact rational `w_j + t·(w_{j+1} −
w_j)` through `motionbound.MotionParam.Lerp`. A waypoint fraction `j/n` is in general not dyadic (`n = 3`
puts the waypoints at `1/3` and `2/3`), so §6's grid need not land on one, and §5.2 does not need it to.
A report locates a pose by `s` and the pose's `Values`: its segment is `⌊n·s⌋`, `n − 1` at `s = 1`, and its
local fraction `n·s − j`. No report field carries the segment.

A sweep whose waypoints are all equal holds its joint at that value for the whole drive; a drive in which
every listed sweep holds is `ErrDegenerate`, as `From == To` is for a `Motion`. `From > To` is legal and
runs the joint the other way, and `From == To` with a different `Via` value takes the joint out and back.

This is option (a) of the two the design weighed: one scalar drives every joint through a stated schedule.
It is the natural extension of `VerifyMotion`, whose `Between` already proves a path over a dimensionless
fraction, and it covers the question a mechanism designer asks first — "does this coordinated motion
clear?" — with the interval certificate unchanged in form. Option (b), proving a whole box of joint values
clear, is the second entry point of §14.

### 2.4 The pose

```go
// LinkagePose is every link's pose at one parameter value.
type LinkagePose struct {
    At     units.Value    // Dimensionless s
    Values []units.Value  // each link's joint value at s, in Linkage.Links() order
    Bounds []units.Value  // each value's proven half-width, in its Kind; zero for a stated joint (§15.4)
    Poses  []r3.Transform // each link's world pose at s, in the same order
}

func (l *Linkage) PoseAt(d Drive, at units.Value) (LinkagePose, error)
```

A link's pose is **its own joint's motion, then its parent's pose**:

```text
Pose_k(s) = J_k(q_k(s)).Then(Pose_parent(s)),   Pose_ground = r3.Identity()
```

where `J_k` is `Revolute{Center, Axis}.PoseAt(q)` for a revolute joint and `Prismatic{Dir}.PoseAt(q)` for a
prismatic one (motion §2), so a joint's refusals are a `Motion`'s. Applied to a point of link `k` at the
zero pose, this rotates it about its own joint first, where that joint's axis still sits at its zero-pose
place, and then carries the result through every ancestor joint up to the ground; `kinetograph`'s
`Node.World` composes the same way, `Local(t).Then(parent.World(t))`. The pose composes onto each body's
own placement as `Body.Placed` composes, `placement.Then(Pose_k)`. `PoseAt` is the one place a pose is
built, so a renderer above and the verifier below evaluate the same transform from the same inputs; a
`TransformTrack` for `kinetograph` is one `PoseAt` call per frame: `kinetograph.LinkageTrack`, which
`Scene.AddLinkage` puts on one driven node per link under the rig's root, reading the drive fraction from a
`Dimensionless` channel. The `_gallery` module's `linkage` subcommand films §11's scene 1 that way, and its
test asserts each node's transform equals `PoseAt`'s bit for bit.

`PoseAt` refuses a drive naming a link of another linkage or a link twice (`ErrDegenerate`), a sweep whose
`From`, `To` or `Via` value has the wrong `Kind` for its joint (`ErrUnitKind`) or is non-finite
(`ErrNotFinite`), sweeps carrying different numbers of `Via` values (`ErrDegenerate`), a value outside
declared limits — `q(s)` is linear within each segment, so every waypoint inside the limits puts every
`q(s)` inside them, and any waypoint outside, or an unlisted joint whose limits exclude the `0` it holds,
is `ErrDegenerate` with a message naming the link — and an `at` that is not a finite `Dimensionless` value
(`ErrUnitKind`, `ErrNotFinite`). An `at` outside `[0, 1]` is legal for `PoseAt`, which takes no range: the
first segment's line extends below `0` and the last's above `1`. `VerifyLinkage` never evaluates one.

## 3. The entry point

```go
func (d *Document) VerifyLinkage(ctx context.Context, l *Linkage, drive Drive, opts ...MotionOption) (*LinkageReport, error)
```

The options are `VerifyMotion`'s own, with the meanings motion §3 gives them: `WithMotionTolerance` is the
relative tolerance on every bounded reading; `WithResolution` is the finest fraction the check refines to,
a `Dimensionless` value as for a `Between`, default `units.Scalar(1.0/1024)`; `WithMinClearance` is a spec
over every evaluated pair and decides `Assessment`. One `Go` option type serves both entry points because
the three settings mean the same thing on either path.

**Two floors.** `VerifyLinkage` refines to two floors. The **verdict floor** bounds §6 step 5's bisection
for the verdict and motion §6 step 6's refinement for a `WithMinClearance` margin. The **reading floor**
bounds step 6's refinement for the whole-drive `Clearance` reading's tolerance gate, which goes only to the
interval holding the smallest certified bound.

- `WithResolution` stated: both floors are the stated value, and every step refines to it exactly as
  motion §3 states for `VerifyMotion`. `WithResolution(Scalar(1))` evaluates the endpoints alone.
- `WithResolution` not stated: the verdict floor is `units.Scalar(1.0/1024)` and the reading floor
  `units.Scalar(1.0/16384)`, so a clear drive's reading meets the default gate at logarithmic cost around
  an isolated minimum (§10).

No option sets the reading floor alone: one stated resolution fixes both. `Request.Resolution` reports the
verdict floor and `LinkageReport.ReadingResolution` the reading floor. `VerifyMotion` keeps one floor for
the verdict and the reading alike (motion §3); nothing here changes it.

How far the reading refines is decided by the certificate that bounds a pair's gap over an interval. The
travel bound of §5.2 sits up to `τ/2` below the gap whatever the gap does, so it meets the gate only at a
step of about `4·rel·gap/τ_rate`; the projection certificate of §5.8 charges only the first-order change
each joint alone can make and a second-order remainder, so around a flat minimum it meets the gate within
a few halvings of the width at which the verdict settled — at the verdict floor itself for §10's
three-joint arm at the defaults. The reading floor is reached where that certificate is loose (§5.8) or
a stated tolerance asks for it.

Every body of every link MUST be a live body of `d`: a retired body is `ErrRetiredBody`, another
document's `ErrForeignBody`, a body this evaluator did not build `ErrUnsupported`. A linkage with no link
is `ErrDegenerate`. The static set is every other live body of `d`, with no option to narrow it (motion
§3): a report is `Sound` about the whole document or not at all. A nil `ctx`, nil `l` or nil `d` is
`ErrDegenerate` before anything else; validation precedes the cancellation check, as in verification §1.2.

## 4. The report

```go
type LinkageReport struct {
    Request       MotionRequest        // the validated effective settings, including defaults
    ReadingResolution units.Value      // the reading floor (§3): Request.Resolution, or Scalar(1.0/16384) by default
    Linkage       *Linkage
    Drive         Drive                // as stated
    Links         []*Link              // Linkage.Links() order
    Against       []*Body              // every static body, in Document.Bodies() order
    JointContacts []DiagnosticPair     // every declared joint contact, in declaration order
    Poses         []LinkagePoseResult  // every pose evaluated, from s = 0 to s = 1; a loop may leave a parameter with no pose (§15.6)
    Intervals     []MotionInterval     // tiling [0, 1] in order; an interval's end has a pose unless a loop refused it (§15.6)
    Collisions    []LinkCollision      // every proven collision, in traversal order then pair order
    Clearance     *ScalarReading       // the minimum gap over the whole drive; nil unless every interval is IntervalClear
    Assessment    Assessment           // against WithMinClearance
    Diagnostics   []Diagnostic         // interval findings, then pose findings, then the path reading's
    Status        Status
}

func (r *LinkageReport) Passed() bool // Status == Sound

type LinkagePoseResult struct {
    Pose          LinkagePose
    Interferences []Interference // A moves; B is static, or belongs to a later link
    Clearances    []Clearance    // the same A/B rule
    Diagnostics   []Diagnostic   // this pose's undecided or unsupported pairs, At set
}

// LinkCollision is a proven overlap at an evaluated pose, about the ideal pose
// (motion §5.1). A belongs to a link; B is a static body or a body of a later
// link. Volume.Value − Volume.Bound is a proven lower bound on the ideal overlap.
type LinkCollision struct {
    At     units.Value // Dimensionless s
    A, B   *Body
    Volume Measurement
}
```

`MotionInterval`, `IntervalOutcome`, `Interference`, `Clearance`, `Diagnostic`, `DiagnosticPair`,
`ScalarReading`, `Assessment` and `MotionRequest` are reused as they are. The four `DiagMotion*` codes
carry the same findings here, with `At` the fraction `s` — a collision is a collision whether one joint or
three moved the body — so no new `DiagnosticCode` is added. `Status` is verification §6's worst-wins
aggregate over the findings of motion §4.1's table, read with "mover" meaning "link body" and "static"
meaning "the other body of the pair".

**Pair order** is fixed: for each link in `Links()` order, for each of its bodies in `Bodies()` order,
first every static body in `Against` order, then every body of every later link in `Links()` then
`Bodies()` order. `A` is always the earlier body in that order. Two bodies of one link form no pair: a
shared rigid motion preserves their relation, which `Verify` reports for the document as it stands
(motion §3).

**No fabricated rows**, as motion §4 states: an interval the certificate did not close is
`IntervalUndecided` and reads `Suspect`; `Clearance` is present only when every interval is
`IntervalClear`.

## 5. The proof

### 5.1 What one pose proves

A pose is evaluated exactly as motion §5.1 evaluates one: each link body's payload is re-evaluated under
`placement.Then(Pose_k(s))` as a transient body that is never committed (`Document.Bodies()`, liveness and
the next producer identity are unchanged by a call), and the pair kernel of clearance §1–§7 and the overlap
proof of interference §3 run over every evaluated pair. The three rules of motion §5.1 hold verbatim, and
only the ideal pose changes shape:

**The ideal pose of link `k` is the exact composition of its joints' ideal poses.** Each joint's ideal
transform at its exact parameter is `motionbound.MotionFrame.At(q)` — the Rodrigues rotation of
`ParamSinCos`'s sine and cosine enclosures about the exact axis through the exact centre, or the exact
translation along the enclosed unit direction (motion §5.1) — and the link's ideal pose is

```text
T*_k(s) = J*_1(q_1(s)) ∘ J*_2(q_2(s)) ∘ … ∘ J*_k(q_k(s))    (applied right to left, J*_k first)
```

composed over rational intervals: `IdealPose.Then` writes `x ↦ R₂·(R₁·x + t₁) + t₂` as one affine map
with interval matrix `R₂·R₁` (`motionbound.IvMat.Mul`) and interval shift `R₂·t₁ + t₂`. A joint with value
exactly `0`, or a revolute at a whole number of quarter turns about an axis-aligned axis, contributes a
point enclosure, so a link whose path joints all hold `0` has `η` exactly zero, as the identity pose does
today. `η_k(s)` is `motionbound.PoseDeviation(composed, placement, T*_k(s), R0)` with `R0` the body's
record radius (`moverRecordRadius`); it covers every rounding the float pose committed — `math.Sincos`,
Rodrigues, the pivot offset, and one `Then` per joint on the path — and grows with the number of joints
only through the enclosure widths, which are of order `1e-16` per joint. Both bodies of a link-link pair
move, so that pair's gap interval is widened by `η_A + η_B` (contact-sweep §3), and a collision on it
transfers only when the measured volume clears `sweptVolumeAllow(η_A, Area_A) + sweptVolumeAllow(η_B, Area_B)`,
each term exactly as motion §5.1 forms it with stretch base `1` (every ideal linear part is a product of
exactly orthogonal rotations). A (link body, static) pair takes `η_A` alone.

### 5.2 What an interval proves: the chain travel bound

Motion §5.2's fact is unchanged: the distance between two closed sets is 1-Lipschitz in a displacement of
either, so the interval `[s_a, s_b]` is clear for a pair when `lo_a + lo_b > τ(s_b − s_a)`, where `τ`
bounds how far any point of either body travels across the interval, and the interval's `Clearance` lower
bound is `(lo_a + lo_b − τ)/2` rounded down. What changes is `τ`: a link's pose is a product of joint
motions, not one screw, so its travel is bounded joint by joint.

**The telescoping bound.** Write the pose of link `k` at the two ends as products `T_k(a) = J_1(a)…J_k(a)`
and `T_k(b) = J_1(b)…J_k(b)`, and move one joint at a time: let `S_i = J_1(b)…J_i(b)·J_{i+1}(a)…J_k(a)`, so
`S_0 = T_k(a)` and `S_k = T_k(b)`. For a point `p` of link `k`,

```text
S_i·p − S_{i−1}·p = J_1(b)…J_{i−1}(b) · [J_i(b) − J_i(a)] · y,    y = J_{i+1}(a)…J_k(a)·p
```

The prefix is an isometry and changes no length, so the step moves `p` by exactly `|J_i(b)·y − J_i(a)·y|`,
the displacement of ONE point under ONE joint: for a prismatic joint it is `|Δq_i|`; for a revolute joint
it is the chord of an arc of radius `dist(y, axis_i)` and angle `|Δq_i|`, at most `dist(y, axis_i)·|Δq_i|`.
Summing the `k` steps,

```text
τ_k(Δ) = Σ_{i ≤ k, revolute} ρ_{ik} · |Δq_i|  +  Σ_{i ≤ k, prismatic} |Δq_i|
```

where `ρ_{ik}` bounds `dist(y, axis_i)` over every point `y` of link `k` after the joints strictly below
`i` on its path have moved to ANY value in the drive's range. The `y` above sits at the `s_a`
configuration, but `ρ_{ik}` is taken over every configuration so that one reading serves every interval.
`|Δq_i|` is `MotionParam.SpanUpper` of the two exact joint values — `2π·|Δturn| + |Δbase|` with `π` at its
upper enclosure for an angle, exact for a length — when no waypoint lies strictly inside the interval.

**A waypoint inside the interval.** The certificate below needs, for every `s` in `[s_a, s_b]`, the travel
from `s_a` to `s` plus the travel from `s` to `s_b` to stay within `τ`. Within one segment `q_i` is
monotone, the two travels sum to the span of the ends, and the span above serves. Across a waypoint it
does not: a joint that turns `0° → 60° → 10°` with the corner inside the interval moves `110°` for ends
`10°` apart. The interval is therefore cut at every waypoint `j/n` strictly inside it, and `|Δq_i|` is the
sum of `SpanUpper` over the pieces `[s_a, j/n], …, [j'/n, s_b]`, the exact values at each cut being the
waypoints themselves. The sum bounds the joint's total variation over the interval, which is additive over
any split at `s`, and the telescoping step above holds between any two configurations, so each one-sided
travel is bounded by its share and the two shares by the sum. §6's grid is not forced onto a waypoint: an
interval holding one is bounded on both sides of it and certifies when its gaps cover that sum (§11
scene 5).

**Reading `ρ_{ik}`.** Each link's rest box is the per-axis union of its bodies' `Bounds()` boxes, each
inflated outward by its own `Bound`, read as exact rationals (`boxCornersExact`). The reading walks DOWN the
path from link `k` toward joint `i`, carrying a ball that encloses link `k` under the joints walked so far:

| step | enclosure carried |
|---|---|
| start, `i = k` | `ρ_{kk}` is `moverAxisRadius`'s reading: the largest exact distance of a rest-box corner from axis `k`, rooted upward. Distance from a line is convex, so the maximum over the box sits at a corner. No ball is needed. |
| ball under joint `k` | revolute: centre `c_k` (the joint's `Center`), radius the largest corner distance from `c_k`, rooted upward — a rotation about any axis through `c_k` preserves distance to `c_k`. Prismatic: centre the box centre, radius the box's half-diagonal plus `m_k`, the largest `|w|` over the joint's waypoints, the farthest the joint slides over the drive (`0` for an unlisted joint); `q_k` is linear within each segment, so its extremes sit at waypoints. |
| ball under joint `m`, given the ball `(c, R)` under joints `m+1..k` | revolute: `(c_m, |c − c_m| + R)` — every point within `R` of `c` stays within `|c − c_m| + R` of `c_m` under any rotation about an axis through `c_m`. Prismatic: `(c, R + m_m)`. |
| `ρ_{ik}` for `i < k` | `dist(c, axis_i) + R`, where `(c, R)` is the ball under joints `i+1..k` and the distance from the exact centre to the exact axis line is `|(c − c_i) × a_i| / |a_i|`, squared exactly and rooted upward. |

Every square root is `proofbound.RatSqrtUp`, an up-rounded float read back as an exact rational; every sum
and product is `big.Rat` arithmetic. No float operation touches a bound after its root, so each `ρ_{ik}`
and each `τ_k` is a proven upper bound, not an estimate (motion §5.2, and the rule that a proven bound stops
covering a value once raw float ops touch it). The ball form is loose where a link is long along a joint's
axis — §11's first scene reads `ρ_{12} ≈ 102.6` mm for a true `98` — and refinement absorbs the slack; a
cylinder enclosure for parallel axes is a later tightening, not a soundness question.

**Two moving bodies.** For a pair of bodies in links `j` and `m`, `τ_{jm}(Δ) = τ^{(L)}_j(Δ) + τ^{(L)}_m(Δ)`,
where `L` is the two links' lowest common ancestor and `τ^{(L)}` sums over the joints strictly below `L`
on each branch only. The joints at and above `L` move both bodies by one rigid motion and change their
distance by nothing, which is the argument motion §3 makes for two movers under one `Motion`; dropping them
is what keeps an elbow pair's certificate as cheap as a one-joint `VerifyMotion`. For a (link body, static)
pair, `L` is the ground and the sum runs over every joint on the path. A joint whose sweep holds
(every waypoint equal) has `|Δq_i| = 0` and contributes nothing to any `τ`; its `m_i = |From_i|` still
enters the balls, because the link sits displaced by it.

**The certificate.** With `τ` so formed, `(lo_a + lo_b − τ(s_b − s_a))/2` is a proven lower bound on
the pair's gap at every parameter of the interval — the **travel bound** — with `lo` the proven lower ends
of the two endpoint gap intervals after `η` (§5.1); it is positive exactly when `lo_a + lo_b > τ(s_b −
s_a)` over exact rationals. The interval is `IntervalClear` for the pair when the larger of the travel
bound and the **projection bound** of §5.8 is positive, and that larger value, rounded down, is the pair's
lower bound over the interval; every undeclared pair must certify for the interval to be `IntervalClear`.
Motion §5.2's three consequences carry over unchanged: a touching endpoint never certifies its interval, a
margin is proven by the lower envelope and disproven by an upper bound only, and a proven collision at an
endpoint ends the argument.

### 5.3 What is exact, what is bounded, what refuses

| Quantity | Standing |
|---|---|
| `s`, each joint's exact value `q_i(s)`, each `|Δq_i|` for a length, every sum and product in `τ` and the balls | exact rationals |
| `|Δq_i|` for an angle, each sine and cosine in an ideal pose | enclosed: `π` and `TurnSinCosInterval` at their upper enclosures |
| each square root in a `ρ` | up-rounded (`RatSqrtUp`) |
| a projection direction `n`, a partner box's extreme `β_n` along it, the second-derivative bounds `B_ij` and the remainder `Rem` (§5.8) | exact rationals |
| a rest-box corner's position `f_c` and velocities `v_{i,c}` at a pose, projected onto `n` (§5.8) | enclosed by the ideal poses at that pose, rounded outward to floats; the bound takes the end that weakens it |
| the float pose the kernel measured | charged by `η_k` (§5.1); never trusted |
| a link body with no record radius (a stitched payload) under a path with any revolute joint | `η` unbounded: the pair reads `DiagUndecidedClearance` at every pose and no collision transfers, exactly as motion §7's stitched mover under a `Revolute`; a path of prismatic joints alone leaves the linear part the identity and `η` is the translation term |
| a `Bounds()` box or an axis the exact reader cannot form (non-finite) | `ErrNotFinite` at the call |

### 5.4 Declared joint contacts

A declared pair (§2.2) runs the pair procedure at every evaluated pose like any other pair, and its
outcome is published asymmetrically:

- a proven overlap that transfers through `η` is a `LinkCollision`, an `Interference` row and a
  `DiagMotionCollision` (with the volume reading's own tolerance finding when its bound fails the gate),
  and drives §6's onset bisection — a jammed or over-rotated joint is found;
- a measured gap publishes its `Clearance` row and nothing else: no margin finding, no tolerance finding,
  and no part in the whole-drive reading or the `Assessment`;
- a touching, undecided, unsupported or sheet outcome, and an overlap that does not transfer, publishes
  nothing — no row, no diagnostic and no `Suspect`.

A declared pair contributes to no interval certificate. `IntervalClear` therefore claims, for a report
with declared pairs: every undeclared pair has a proven positive gap at every parameter of the interval,
AND every declared pair is proven free of transferred overlap at both endpoints. Nothing continuous is
claimed about a declared pair — a pin that leaves its bore and re-enters it between two samples is not
found — and §5.6 says so. This is the reason the declaration is explicit, listed in the report, and
per pair: the caller names the scope of the sampled claim, and the report shows it.

### 5.5 What the whole drive proves

Motion §5.3 holds: when every interval is `IntervalClear`, the minimum gap over every undeclared pair lies
between the smallest interval lower bound and the smallest evaluated upper bound, and `Clearance` reports
that interval's midpoint and half-width, judged against the pair diameter of the pair that attained the
upper end. Declared pairs do not enter the reading.

A margin is decided on motion §5.2's terms, and an excluded pair — by swept box or by layer (§5.7) —
contributes its proven lower bound and nothing else. A `WithMinClearance` above that bound therefore reads
`AssessmentUndecided`, with a `DiagMotionUndecidedClearance` on each interval it holds back, and the report
reads `Suspect`: the bound cannot meet the margin, and no pose measures the pair to disprove it. It never
reads `AssessmentViolated`. This is motion §6 step 3's outcome for a swept-box-excluded pair.

### 5.6 What is never claimed

- Nothing about `s` outside `[0, 1]`, nothing about a configuration the drive does not visit (§14 is the
  check over a box of them).
- Nothing between two bodies of one link.
- Nothing continuous about a declared joint contact (§5.4).
- Nothing about closure the caller did not state: a caller who drives two joints to keep an undeclared
  loop closed has stated two independent sweeps, which the report speaks for as stated, not a closure. A
  loop stated with `Close` is spoken for as §15 says: about the document's own loop, every bar at its exact
  length.
- An `IntervalUndecided` claims nothing; a `LinkCollision` claims overlap at that `s` and nothing about the
  interval around it; no tolerance decides admission (motion §5.4).

### 5.7 The layer exclusion

A pair whose relative motion never changes either body's height along one direction cannot close a gap
that height separates. That is the common shape of a planar mechanism — arms stacked along their parallel
joint axes, an arm swinging over a table — and its pairs keep a constant gap the interval certificate
pays for at every step (§10).

**The rule.** A pair's relative path is the joints strictly below the two links' lowest common ancestor on
each branch, and every joint on the body's path for a (link body, static) pair (§5.2). A joint that holds
`0` — unlisted, or every waypoint `0` — moves nothing and is passed over. The pair is **layer-separated
along `a`** when every other joint on its relative path is a revolute whose `Axis` is parallel to `a` or a
prismatic whose `Dir` is perpendicular to `a`, and the two bodies' `a`-extents are separated by `w > 0`.
It is then settled for the whole drive as a swept-box exclusion is (motion §6 step 3): never evaluated at
any pose, and contributing the lower bound `w/|a|`, rounded down, to every interval.

**Why it holds.** A rotation about any line parallel to `a` maps `x` to `R·(x − c) + c` with `Rᵀa = a`,
so `a·x` is unchanged; a slide along `d ⊥ a` adds nothing along `a`. Every joint of the relative path,
at any value, therefore preserves `a·x` for every point of either body, and so does their composition:
relative to the common ancestor, each body's `a`-extent at every `s` is its extent at the zero pose. The
common ancestor's own motion carries both bodies rigidly and changes their distance by nothing (§5.2). For
`x` in one body and `y` in the other, `|a·(x − y)| ≥ w`, so `|x − y| ≥ w/|a|` at every `s`. The claim is
about the ideal poses, as every interval claim is (§5.1).

**Exactness.** Every test is an exact rational comparison, with no tolerance:

- `a` is the `Axis` of the first joint on the relative path that is a revolute and does not hold `0`, read
  exactly; parallel means the exact cross product `a × Axis_i` is the zero vector, and perpendicular means
  the exact dot product `a·Dir_i` is zero. A path with no such revolute tries, in order, the coordinate
  axes `X`, `Y`, `Z` and the exact cross product of its first two non-parallel slide directions, each
  admitted only when every slide is perpendicular to it.
- A body's `a`-extent is the least and greatest exact `a·x` over the eight corners of its zero-pose
  `Bounds()` box inflated by its own `Bound` (`boxCornersExact`); `a·x` is linear, so the extremes over the
  box sit at corners.
- `w` is compared strictly with zero: touching extents (`w = 0`) never exclude, so a link resting on its
  pivot's cap or on a table is evaluated, and a declared contact still reads as §5.4 says.
- `|a|` is rooted upward (`proofbound.RatSqrtUp`), so `w/up(|a|)` is a lower bound on `w/|a|`, and it is
  rounded down to the float the interval publishes.

A joint whose axis leans off `a` by any amount, or a slide with any component along `a`, fails the rule
and leaves the pair to the interval certificate. A pair the rule settles carries no gap row at any pose and
no upper bound, so a drive whose every pair is settled reads `Sound` with no whole-drive `Clearance`,
exactly as motion §5.3 states for swept-box exclusion.

### 5.8 The projection certificate

The travel bound of §5.2 and the centre certificate of §14.3 charge an interval or a cell up to `τ/2`
below the gaps measured at its ends or its centre, whatever the gap does across it. Where the closest
approach is a **flat minimum** — the gap grows with the square of the joint change while `τ` shrinks only
with its first power — the reading's defect per interval is about `τ_rate·Δ/4` and per cell `τ_half`, and
meeting the gate `rel·gap` needs a step of about `4·rel·gap/τ_rate` over the whole region where the gap
sits within `rel·gap` of its minimum. For a gap curving as `κ·Δq²` that region is about
`2·√(rel·gap/κ)` wide along each joint, so the cell count grows as `(τ_rate/√(rel·gap·κ))^n`: about a
hundred intervals for the three-joint arm's drive (§10: `251` poses under the travel bound alone), and about `10^8` cells for the same
arm's box (§14.7), which no budget reaches. This certificate bounds a pair's gap over an interval or a
cell to second order in the step instead, from the bodies' `Bounds()` boxes and the ideal poses alone.

**The fact.** For any unit vector `n` and any configuration, the distance between two bodies is at least
the separation of their projections onto `n`,

```text
dist(A, B) ≥ sep_n(A, B) = inf_{b ∈ B} n·b − sup_{a ∈ A} n·a
```

because `|a − b| ≥ n·(b − a)` for every pair of points. Each side is bounded from a `Bounds()` box. A
static body `B` lies inside its box inflated by its `Bound` (core §5.3), so `inf_B n·b ≥ β_n`, the least
`n·b` over that box's eight corners. A body `A` of link `k` at configuration `q` lies inside the image
under the link's ideal pose `T_k(q)` of its own inflated rest box, a parallelepiped whose extreme along
`n` sits at a corner, so `sup_{A(q)} n·a ≤ max_c f_c(q)` over the eight rest-box corners `c`, with
`f_c(q) = n·x_c(q)` and `x_c(q) = T_k(q)·c`. Both hold for a body of any shape; they are tight where the
body reaches its box along `n` — an axis-aligned prism's face, a block's corner — and loose where it does
not (below). §5.7 is the case where every joint on the relative path keeps `n·x`, so `f_c` is constant.

**The corner expansion.** `x_c(q)` is a smooth function of the joint values on the link's path. For a
configuration `m` and any `q` with `|q_i − m_i| ≤ h_i` on every joint `i` of the path,

```text
f_c(q) ≤ f_c(m) + Σ_i |n·v_{i,c}(m)|·h_i + Rem(h),      Rem(h) = ½ · Σ_{i,j} B_ij · h_i · h_j
```

`v_{i,c}(m)` is the corner's velocity under joint `i` alone at `m`: `ω_i × (x_c(m) − o_i)` for a revolute
whose axis at `m` runs through `o_i = Pose_parent(i)(m)·Center_i` along the unit vector `ω_i =
R_parent(i)(m)·Axis_i/|Axis_i|`, where `Pose_parent(i)` is the parent link's ideal pose (the identity for
a child of the ground) and `R_parent(i)` its rotation; and the unit vector `R_parent(i)(m)·Dir_i/|Dir_i|`
for a prismatic. `B_ij` bounds every second derivative `∂²x_c/∂q_i∂q_j` at every configuration: with
`w_j = ρ_jk` for a revolute joint `j` and `1` for a prismatic one (§5.2), `B_ij = B_ji = w_j` when `i` is a
revolute joint at or above `j` on the path, and `0` when the shallower of the two joints is a prismatic.

*Why.* Taylor's theorem along the straight segment from `m` to `q` in joint space gives `x_c(q) = x_c(m)
+ Σ_i v_{i,c}(m)·(q_i − m_i) + R` with `|R| ≤ ½·max_t |δᵀ·H(m + t·δ)·δ| ≤ ½·Σ_{i,j} B_ij·|δ_i|·|δ_j|`,
`H` the matrix of second derivatives and `δ = q − m`; projecting onto `n` and bounding each first-order
term by its absolute value gives the inequality. The derivatives are §5.2's telescoping step read
infinitesimally. Write `x_c = P_{i−1}·J_i(q_i)·y_i`, with `P_{i−1}` the isometry of the joints above `i`
and `y_i` the corner under the joints below `i`. `∂x_c/∂q_i` moves `J_i` alone: for a revolute,
`v_{i,c} = P_{i−1}·[â_i × (J_i·y_i − c_i)]`, whose length is `dist(y_i, axis_i)`, exactly what `ρ_ik`
bounds; for a prismatic, `P_{i−1}·d̂_i`, of length `1`. Differentiating `v_{i,c}` once more: by a revolute
joint `j` above `i`, only `P_{i−1}` changes, and it turns the whole vector, so `∂v_{i,c}/∂q_j = ω_j ×
v_{i,c}`, of length at most `|v_{i,c}| ≤ w_i`; by `i` itself, `∂v_{i,c}/∂q_i = ω_i × v_{i,c}`, the same
bound; by a prismatic joint above `i`, nothing turns and the derivative is zero; and a joint below `i` is
the symmetric case with the roles exchanged. Each bound holds at every configuration because `ρ_ik` is
read over the whole drive or box (§5.2, §14.3).

**The bound.** For a cell with half-spans `h_i` about its centre `m` (§14.3), or an interval read from one
of its ends `e` with `h_i` the joint's travel bound from that end (below),

```text
L_n = β_n − max_c [ f_c(m) + Σ_i |n·v_{i,c}(m)|·h_i ] − Rem(h)
```

is a proven lower bound on the pair's gap at every configuration of the cell, or every parameter of the
interval, and the pair is **clear over it when `L_n > 0`**. The certificate takes the largest `L_n` over
the six coordinate directions `±X`, `±Y`, `±Z`, each an exact unit vector, and then **the larger of that
and the travel bound** — `(lo_a + lo_b − τ)/2` on an interval (§5.2), `lo_m − τ_half` on a cell (§14.3).
Two proven lower bounds on one quantity are both lower bounds, so their maximum is one. For a pair of two
link bodies, the joints at and above their lowest common ancestor move both rigidly and are dropped
(§5.2): each body's corners are mapped through the ideal poses of the joints strictly below the ancestor
on its own branch, in zero-pose world coordinates, and the partner's side is expanded downward the same
way, `inf_{B(q)} n·b ≥ min_{c'} [ f'_{c'}(m) − Σ_i |n·v'_{i,c'}(m)|·h_i ] − Rem_B(h)`, each body with its
own `ρ`. A held joint has `h_i = 0` and contributes nothing to either sum; a held link's expansion is
its constant placement.

**Why it closes a flat minimum.** Where one corner `c` attains the maximum over the cell and the partner
reaches its box along `n`, the true minimum of the gap over the cell is `β_n − max_q f_c(q)`, and the
expansion is sharp to second order on both sides: `max_q f_c(q) ≥ f_c(m) + Σ_i |n·v_{i,c}(m)|·h_i −
Rem(h)`, approached toward the corner of the cell the gradient points at. `L_n` therefore sits within
`2·Rem(h)` of the true minimum over the cell, second order in the half-spans, while the travel bound sits
up to `τ_half` below it. The reading closes once `Rem(h)` falls under about `rel·gap/2`, at a half-span of
about `√(rel·gap/Σ_{ij} B_ij)` — a fixed number of halvings per joint whatever the tolerance's effect on
the first-order step. Nothing about flatness is assumed: at a cell on the box's face, or an interval far
from the minimum, the first-order term is the exact first-order drop toward the cell's far side and the
bound is sharp to second order all the same. A coarse interval or cell is still the travel bound's: the
two cross near `h ≈ (τ_rate − 2·Σ_i |n·v_i|)/Σ_{ij} B_ij`, and above it `Rem` exceeds `τ`.

**On a drive: the segment term.** The box form bounds every configuration within the half-spans `h_i`
of the end, but an interval of a drive visits only the drive's path. On an interval inside which no
waypoint bends a joint's schedule (below), every stated joint is affine in `s` (§2.3), so the path is the
straight joint-space
segment `q(s) = q(s_a) + t·Δq`, with `t = (s − s_a)/(s_b − s_a)` running over `[0, 1]` and
`Δq = q(s_b) − q(s_a)`. Taylor's theorem along that segment, read from end `a`, gives

```text
f_c(q(s)) = f_c(m) + t·Σ_i n·v_{i,c}(m)·Δq_i + R,      |R| ≤ ½ · Σ_{i,j} B_ij · t²·|Δq_i|·|Δq_j| ≤ Rem(|Δq|)
```

with the same `B_ij` and the same remainder as the box form, since `|δ_i| = t·|Δq_i| ≤ |Δq_i|` and every
configuration on the segment lies in the drive's range. The first-order term is linear in `t`, so its
largest value over `[0, 1]` sits at an end of `[0, 1]`:

```text
f_c(q(s)) ≤ f_c(m) + max(0, Σ_i n·v_{i,c}(m)·Δq_i) + Rem(|Δq|)
```

Read from end `b`, the segment runs backward, `δ = −t·Δq`, and the term is `max(0, −Σ_i n·v_{i,c}(m)·Δq_i)`
at `m = q(s_b)`. The partner's side is the mirror image, `f'_{c'}(q(s)) ≥ f'_{c'}(m) + min(0, ±Σ_i
n·v'_{i,c'}(m)·Δq_i) − Rem_B`, its sign the end's, each body on its own relative path. The term is at most
the box form's `Σ_i |n·v_{i,c}|·|Δq_i|`, so on such an interval it replaces the box form. A waypoint
inside the interval at which some joint's schedule bends — its two neighbouring segments change that
joint by different turns or bases — keeps the box form with `h_i` the total variation, since the
configurations leave one segment; a waypoint on the straight line through its neighbours at equal shares
bends nothing, so a drive with collinear `Via` values reads the bounds the plain drive reads (§2.3).
The sum vanishes to first order where the drive's gap is flat, even when its terms do not: at the
three-joint arm's minimum the wrist's corner moves along `X` at about `−2.8`, `−0.7` and `+3.6` mm per
radian of its three joints, which the box form charges as `7.1` and the segment term as their sum, near
`0`. The interval form is then second order on a drive, as the cell form is in a box, and the reading
closes once `Rem(|Δq|)` falls under about `rel·gap/2`.

**Where it is loose, and what still holds.** The bound is as tight as the two boxes are along `n`. A body
whose extreme along `n` lies inside its box corner's image is charged the corner: a disc or a cylinder
turning about its own axis keeps every point within its radius `r` of the axis while its box corner
sweeps a circle of radius `r·√2`, so between the quarter turns `sep_n` falls short of the gap by up to
`(√2 − 1)·r`, the travel bound is the larger, and the pair pays §10's linear cost as before — §11's disc
beside a wall evaluates `513` poses either way. A partner met along a direction off the coordinate axes,
a tilted wall, is charged its box corner the same way. In both the certificate is still the larger of
two proven bounds, so no interval or cell reads worse than under the travel bound alone. A tighter support
of a body along `n`, read from the clearance kernel's own carriers, and the kernel's closest-point
direction as a seventh candidate `n`, are later tightenings that change no soundness argument; so is a
rule that a full-turn revolve on its own joint axis is invariant under that joint. A pair whose relative
path holds a dependent joint (§15) takes the travel bound alone: its value at a pose is an enclosure and
its travel over an interval the hull bound of §15.5, both of which the expansion could consume through
`MotionFrame.AtRange`, and that extension is a later increment.

**The interval form.** On `[s_a, s_b]` the bound is read from each end `e ∈ {a, b}`, the larger of the two
ends' `L_n` serving. The remainder takes `h_i = D_i`, `jointSpan`'s total variation of joint `i` over the
interval (§5.2), which bounds `|q_i(s) − q_i(s_e)|` for every `s` of the interval, waypoints inside it
included. The first-order term is the segment term when no waypoint inside the interval bends a joint's
schedule, and the box form's `Σ_i |n·v_{i,c}|·h_i` otherwise, so no interval is cut. A touching end
still never certifies: `sep_n(e) ≤ gap(e) = 0`, and the expansion from the far end covers the touching
one, so every `L_n ≤ 0`. The two-sided form of the travel bound is not reproduced here: the two one-sided
bounds each cover the whole interval, and their maximum is sound.

**What is exact, what is bounded.** `n` is an exact unit coordinate vector and `β_n` a rational read off
the partner's inflated box. `f_c(m)` and each `v_{i,c}(m)` are rational intervals built from the ideal
poses at `m` (`idealPosesOf`, each unit axis through `UnitScaleInterval`, the cross product over
intervals), and the bound takes `f_c`'s upper end and the larger absolute end of each `n·v`, or, for the
segment term, the end of the interval sum `Σ_i n·v_{i,c}·Δq_i` that weakens it. `h_i` is half of
`MotionParam.SpanUpper` across the cell — `π` at its upper enclosure for an angle, exact for a length — or
`jointSpan` over the interval, and `Δq_i` is the exact difference of the joint's two values, `2π·Δturn +
Δbase` with `π` over its enclosure. Once per pose each coordinate's enclosure and each velocity component's
enclosure is widened to the floats around it, and each `h_i` and `Δq_i` is widened the same way, so the
sums over an interval run on short dyadics; rounding outward only lowers `L_n`. `w_j` is the rational
`ρ_jk` of §5.2, `B_ij` and `Rem` are `big.Rat` arithmetic, and `L_n` is compared with zero exactly and
rounded down to the float the interval or cell publishes. No float pose enters: the claim is about the
ideal poses, and the box inflated by its `Bound` encloses the ideal body, so no `η` is charged.

**Scope.** The projection bound is tried for exactly the pairs the travel bound is tried for: an
undeclared evaluated pair with a measured gap at both ends of the interval, or at the cell's centre. A
pair the kernel leaves undecided, a sheet, an invalid operand, a declared pair and an excluded pair stand
as §5.4, §5.7, §6 and §7 say. `VerifyMotion` is unchanged: its driver supplies no projection bound, every
one of its intervals is the travel bound's, and its reports are the same byte for byte.

**Cost of the certificate.** Per pose or centre and per moving body it reads eight corner positions and
their velocities under each joint on the path once, and projects them onto six directions: a few hundred
rational-interval operations, and a few hundred sums over short dyadics per pair and interval. The
readings are kept on the pose and serve both intervals it ends. Measured on §10's three-joint arm: at
`WithResolution(Scalar(1.0/64))` its `10` poses take about `50` ms against `30` under the travel bound
alone, and at the defaults the reading closes at a step of `1/256` in `16` poses and about `80` ms, where
the travel bound alone refines to the reading floor in `251` poses and about `0.8` s. Estimated, and
measured by §14.9's P2: the arm's box reads `Sound` in about a thousand centres in place of exhausting the
budget (§14.7). The disc is unchanged. A box whose minimum sits at a corner of the box with a nonzero gradient
(§14.8's clear crane box) gains little: its reading's upper end is a centre's gap, and no centre reaches
the corner, so that end closes only linearly in the cell whatever bounds the lower end.

**The split axis.** Where a pair's bound over a cell or an interval is the projection bound, the joint
whose halving lowers it most is the one with the largest share of its defect, `|n·v_{i,c}(m)|·h_i +
Σ_j B_ij·h_i·h_j` over the direction and corner that attained the bound; where it is the travel bound,
the share is `w_i·span_i` as §14.4 states. A joint the gap does not depend on — the slide under scene 13's
pendulum — has a zero share and is never split for that pair, which is what keeps a gap constant along one
joint from costing cells along it. The rule decides cost alone, never soundness.

**Why this form.** A mean-value interval form, `f_c(m) + ∇f_c(C)·(C − m)` with the gradient enclosed over
the whole cell, converges at the same order and needs an interval pose over the cell per corner and joint,
with the widening every interval composition adds; the remainder here is proven from `ρ` once and costs a
quadratic form. The kernel's gap at the centre plus a directional derivative is not sound without a
convexity proof — `gap(m)` can exceed `sep_n(m)` for a non-convex body — while `sep_n` assumes no shape. A
larger budget does not meet the gate: `65536` centres and six minutes do not (§14.7).

**Required tests for the drive.** §11's standard holds. Each scene below states its minimum in closed
form, and every `IntervalClear` interval's `Clearance` is asserted at or below the closed-form gap at
each of its ends, which is at least the minimum over it.

- **The three-joint arm (§10) at the defaults.** The wrist's leading corner `(150, −10)` sits at
  `X(θ) = 50·(cos θ + cos 2θ + cos 3θ) + 10·sin 3θ`, `Y(θ) = 50·(sin θ + sin 2θ + sin 3θ) − 10·cos 3θ`
  for `θ = 90°·s`; `X` peaks where `X'(θ) = 0`, at `θ* ≈ 2.44°` (`s* ≈ 0.0271`, bracketed to `1e-12` by
  bisection of `X'`), with `Y ≈ 2.8` inside the post's face, so the drive's minimum gap is `160 − X(θ*)
  ≈ 9.360` mm. Assert `Sound`, `Clearance` enclosing it with `ToleranceSatisfied`, no interval narrower
  than `1/1024`, and the pose count recorded: `16`, against the estimate of about `90` that assumed a step
  of `1/512`; the segment term closes the reading at `1/256`. Red when the projection bound is dropped:
  the reading then refines to `1/16384` (§10). The reading floor's leg (§11) runs the same arm at
  `WithMotionTolerance(Scalar(1e-5))`: `Sound` with some interval narrower than `1/1024`, measured `26`
  poses down to `1/4096`; red when the segment term is dropped, since the box form's first-order defect
  then leaves the reading beyond tolerance at the reading floor.
- **Scene 13 — the pendulum (the remainder's leg).** A block `x ∈ [−5, 5], y ∈ [−50, −40], z ∈ [0, 10]`
  hangs from a revolute joint about `Z` through the origin and swings `0° → 90°` toward a wall
  `x ∈ [−100, 100], y ∈ [20, 40], z ∈ [−10, 20]` above the pivot. Its highest corner is `(5, −40)`, at
  `y = 5·sin θ − 40·cos θ`, so the gap is `g(θ) = 20 − 5·sin θ + 40·cos θ`, decreasing from `60` to `15`
  and concave up to `θ = atan 8 ≈ 82.9°`: on a concave stretch the minimum over an interval is the gap at
  its far end, and the expansion from the near end without its remainder, `g(a) − |g'(a)|·Δθ`, exceeds it
  by about `½·|g''|·Δθ²`. `ρ_11 = √(50² + 5²) ≈ 50.25` and `|g''| ≤ 40`, so `Rem = ½·ρ_11·Δθ²`
  covers the remainder and half of it does not near `0°`. Assert, at `WithResolution(Scalar(1.0/16))`,
  every `IntervalClear` interval's `Clearance` at or below `g` at its far end; red when `Rem` is dropped
  and when it is halved. At `WithMotionTolerance(Scalar(1e-5))` assert `Sound`: the minimum, `15` mm at
  the drive's end, is approached at `40` mm per radian and the travel bound sits about `5.1` mm per radian
  of the last step below it, so it needs a step of about `3.7e-5` there, under `1/16384`, and reads
  `Suspect` alone, which is the leg that goes red when the projection bound is dropped. Measured: `11`
  poses, against `16` that end at the reading floor under the travel bound alone. The same scene stated along each of the six directions — the wall on each
  side of the pivot, the block hanging opposite — reads `Sound` at that tolerance; red when that direction
  is dropped. **The pendulum under a slide:** the pivot carried by a prismatic joint along `X` under the
  ground on a carriage `x ∈ [−5, 5], y ∈ [−8, −2], z ∈ [30, 40]`, the box `d ∈ [0, 30]` mm × `θ ∈ [0°,
  90°]` (§14.8); the layer exclusion settles the carriage against the wall along `Y` at `22` and against
  the block along `Z` at `20`, both above the pendulum's `15`.
- **Scene 14 — the two arms (both bodies moving).** Arm `A`, `x ∈ [0, 50], y ∈ [−5, 5], z ∈ [0, 10]`, on
  a revolute about `Z` through the origin sweeping `−30° → 30°`; arm `B`, `x ∈ [60, 110]`, the same
  section, on a revolute about `Z` through `(110, 0, 0)` under the ground sweeping `30° → −30°`. The
  corners `(50, −5)` of `A` and `(60, −5)` of `B` share their height `50·sin θ − 5·cos θ` at every `θ`
  and their edges lean away from each other, so the gap is the corners' distance `g(θ) = 110 − 100·cos θ
  − 10·|sin θ|`, flat at `tan θ = 1/10`: `θ* = 5.7106°`, `g* = 110 − 10·√101 ≈ 9.501` mm, at `s = 0.4048`
  and `0.5952`. The relative path is both joints, and their `z`-extents coincide, so the layer exclusion
  does not settle the pair. Assert `Sound` at the defaults, `Clearance` enclosing `g*`, no interval
  narrower than `1/1024`; red when the partner's expansion is dropped from a link-link pair (the bound
  then exceeds `g` at an interval end near a minimum). Measured: `23` poses, against `445` down to
  `1/4096` under the travel bound alone.
- **Internal tests** in `linkage_internal_test.go`: `B_ij` against finite differences of `n·x_c` on §11's
  four-joint chain — revolute, prismatic, revolute, prismatic — at a grid of configurations, every
  difference quotient at or below its `B_ij`, red when the prismatic-ancestor zero and the revolute's `w_j`
  are exchanged; `v_{i,c}` against the closed-form `ω × r` of scene 1's forearm corner under each of its
  two joints, red when the parent's pose is dropped from the elbow's axis; `L_n` for scene 13's first
  interval against the hand sum, red when the remainder or the first-order term is dropped; and the
  certificate taking the larger bound — on the disc of §11 every interval's bound equals the travel
  bound's bit for bit, red when the projection bound is taken alone.
- **Agreement with `VerifyMotion`** (§11) is restated. At the endpoints alone, on motion §9's fixtures 1,
  2 and 4, neither bound certifies the one interval and the two reports agree in every reading.
  Bisected, `VerifyMotion` keeps the travel bound alone and the linkage certifies some intervals sooner
  on all three: the leg asserts the same collisions, no more poses than `VerifyMotion`'s, every linkage
  interval's `Clearance` at or below the closed-form gap at each of its ends, equal `Status` on fixtures
  1 and 4, and on fixture 2, where the linkage's reading meets the gate at the floor and `VerifyMotion`'s
  does not, `Sound` against `Suspect` with both readings enclosing `10` within `0.1` mm. Measured:
  fixture 1 evaluates `11` poses against `13`, fixture 2 `16` against `102`, fixture 4 `21` against
  `114`.

## 6. The procedure

Motion §6's deterministic dyadic bisection is reused as it stands — endpoints first, then bisection for
the verdict with onset bracketing of a colliding interval, then bisection for the reading, each bounded by
the resolution floor — and `linkage_verify.go` runs it over `motion_verify.go`'s engine generalised to
several moving groups (§12 PR 1). The steps that differ:

1. **Validate** (§2, §3, §8) before reading `ctx`.
2. **Resolve each link's standing.** A link whose path joints all hold `0` — every sweep on its path
   absent or with every waypoint `0` — moves nothing: its bodies are evaluated as static bodies, with no
   transient placement and no `η`, and its pairs against the ground's statics are not formed (they are
   `Verify`'s); its pairs against a moving link are evaluated with its bodies as they stand. A link whose
   path joints all hold, at least one at a nonzero value, is a constant placement: each body's transient
   placement is built once, at `s = 0`, and every pose reuses it, with `τ = 0` on every pair it forms. A
   held link stays in `Links()` and its poses stay in each `LinkagePose`; pair order (§4) is unchanged.
3. **Read the bounds** (§5.2): per link, `R0`, area and `σ` per body as motion §6 step 2 reads them, and
   `ρ_{ik}` for every joint `i` on its path.
4. **Swept-box exclusion.** Each body of link `k` has its rest box, inflated by its own `Bound` plus its
   link's travel from the zero pose to the farthest the drive takes it, `Σ_{i ≤ k} ρ_{ik}·m_i` (prismatic
   terms `m_i`), compared with each static body's inflated box as exact rational extremes (motion §6 step
   3); a strictly positive axis gap settles the pair for the whole drive. The body's own box serves, not
   the link's: `ρ_{ik}` is read over the link's whole rest box, so it bounds the travel of every one of its
   bodies. A link-link pair is settled the same way when the two bodies' swept boxes, each grown by its own
   link's travel from the zero pose, separate. The travel is measured from the zero pose, not from `s = 0`,
   for the reason motion §6 step 3 gives: a joint whose `From` is `80°` has moved before the drive begins.
   A pair the swept boxes leave is then tried by the layer exclusion (§5.7). A declared pair is excluded
   on the same terms: a pair proven apart over the whole drive is proven free of overlap too.
5. **Evaluate, bisect and publish** as motion §6 steps 4–7, with `τ` per pair from §5.2, the projection
   bound per pair from §5.8 read off each pose once and serving both intervals it ends, and the declared
   pairs handled as §5.4 says. Step 6's refinement for the reading's tolerance gate stops at the reading
   floor and its refinement for a margin at the verdict floor (§3). The onset bisection halves a colliding interval that still has a
   collision-free end whether the collision sits on a declared pair or an undeclared one.

`ctx` is checked before every pose and inside every kernel call; a cancelled call returns `ctx.Err()` and
no report. Two calls on the same inputs return reports equal in every field.

## 7. Coverage

Per body, the reach is motion §7's table, applied to every link body as a mover and every static body as
a static. A link-link pair is reachable exactly when both bodies are reachable as movers. A link body that
is not a proven solid leaves every pair it forms undecided and carries its validity diagnostic into each
pose, as motion §7 states for a mover.

## 8. Errors

`VerifyLinkage` returns `(*LinkageReport, error)` by core §12: an error is a call that could not be made.

| Condition | Error |
|---|---|
| nil `ctx`, nil document, nil linkage; a linkage with no link; a drive in which every sweep holds; a drive naming a link twice or a link of another linkage; sweeps carrying different numbers of `Via` values | `ErrDegenerate` |
| a link body retired, foreign, or not built by this evaluator | `ErrRetiredBody`, `ErrForeignBody`, `ErrUnsupported` |
| a declared joint contact naming a retired or foreign body | `ErrRetiredBody`, `ErrForeignBody` |
| a sweep's `From`, `To` or `Via` value of the wrong `Kind` for its joint; a wrong-`Kind` resolution, tolerance or minimum | `ErrUnitKind` |
| a non-finite sweep value, option, box or axis read | `ErrNotFinite` |
| a sweep waypoint outside the joint's declared limits | `ErrDegenerate`, message naming the link |
| a negative or zero resolution, a negative tolerance or minimum; a zero minimum | `ErrNegativeMagnitude`; `ErrDegenerate` |
| `ctx` cancelled after validation | `ctx.Err()` |
| an invariant failure inside a kernel | that error, no report |

The constructor refusals of §2.1 and §2.2 apply at construction. An undecided pair, an unsupported payload,
a sheet, a resolution floor reached — none is an error; each is a finding that reads `Suspect`.

## 9. Deferred, and the condition for each

### 9.1 Loops with two sliders, and loops driven below the common link

A planar loop is §15, about any axis. What §15 refuses with `ErrUnsupported` waits on named conditions: a
loop with two prismatic joints, or a prismatic joint below the common link, waits on a scene dimension that
measures a displacement along a second direction; a loop driven at a joint whose parent is not the common
link waits on a reference for the driving angle that moves with the parent. Until then a caller who knows a
dependent value in closed form states it as its own sweep of a tree, and the report speaks for that stated
motion, not for closure (§5.6).

### 9.2 A box of joint values

Proving every configuration in `[Min_1, Max_1] × … × [Min_n, Max_n]` clear is §5.2's travel bound over
cells of joint space instead of intervals of one parameter. It is a second entry point,
`VerifyJointBox`, designed in §14 and landing as §14.9's increments; it changes nothing in §1–§8.

### 9.3 A tighter `ρ` for parallel axes

§5.2's balls are loose where a link is long along a joint's axis; a cylinder enclosure about parallel
axes would tighten `ρ_{ik}`. It changes no soundness argument, only how early an interval certifies, and it
waits on a measured drive whose pose count the ball's slack decides.

## 10. Cost

Each pose runs the pair procedure once per evaluated pair; a declared pair costs the same as an evaluated
one. The per-call `bodyGeomCache` holds every static body's clearance carriers for the whole run, and each
transient link placement is built, used for its pairs, and dropped per pose, as `VerifyMotion` does. The
one-mesh-per-body cache of interference §5.3 is `Verify`'s and does not apply: a pose measures transient
bodies, and each pair keeps its own tolerance, as that section records for the motion check. A static
body's mesh, when a pair reaches the mesh path, comes from the body's own one-entry tessellation cache.

The pose count is governed by the travel rate. A link under `k` sweeping revolute joints has
`τ ≈ Δs·Σ_i ρ_{ik}·|To_i − From_i|`; a wrist link with `ρ` of `50`, `100` and `150` mm under three quarter-
turn sweeps has `τ ≈ 471·Δs` mm, so a `10` mm gap certifies at `Δs ≲ 1/32` and a clear sweep evaluates
about `33` poses plus the reading refinement's `log₂(1/Resolution)` extra poses around the minimum. For a
three-joint arm of one body per link, two fixtures and two declared elbow contacts, that is `9` pairs per
pose — `6` against the fixtures, one undeclared link-link pair, two declared — about `400` pair evaluations
for a clear sweep, and `10`–`20` onset poses per contact found.

Measured on `BenchmarkVerifyLinkageThreeJointArm` (three stacked `50` mm links each turning `0° → 90°`,
the two elbows declared, a post `10` mm past the wrist and a far post): the declared elbows touch and
publish nothing, the shoulder-wrist pair is settled by the layer exclusion (§5.7), and four pairs are
evaluated per pose. At `WithResolution(Scalar(1.0/64))` the verdict settles in `10` poses, about `50` ms,
every interval `IntervalClear` and the reading beyond tolerance. At the defaults the projection bound
(§5.8) closes the reading around its one minimum at a step of `1/256`: `16` poses, about `80` ms,
`Sound`. The travel bound alone settles the verdict in `50` poses and refines the reading to the reading
floor in `251`, about `0.8` s.

**The whole-drive reading at the default floor.** The verdict is cheap; the reading need not be. A certified
interval's lower bound sits up to `τ/2` below the true gap, so the reading's half-width near the minimum
is about `τ_rate·Δs/4`, and the gate (verification §2) admits `rel·gap`. A chain's `τ_rate` is the sum of
its `ρ_{ik}·|To_i − From_i|` — hundreds of millimetres per unit `s` for an arm of a few links — so at
`rel = 1e-3` and a `10` mm gap the reading needs `Δs ≈ 7e-5`, under the verdict floor `1/1024`. That is
why the reading has its own floor (§3): refinement past the verdict floor goes only to the interval holding
the smallest bound, so around an isolated minimum it costs about `log₂(16)` halvings per tie broken (the
three-joint arm above under the travel bound alone: `50` poses for the verdict, `251` with the reading).
The projection bound of §5.8 sits second order in the step below the gap, and closes such a minimum's
reading a few halvings past the width at which the verdict settles (`16` poses for the same arm); the reading
floor then serves a tighter tolerance and the pairs that certificate leaves loose. A caller who states
`WithResolution` stops the reading there too, and a clear drive whose reading the stated floor leaves
coarse reads `Suspect` with a `DiagMeasurementBeyondTolerance` on it. A minimum that holds along the drive —
a pair whose gap does not change — makes every interval tie for the smallest bound, step 6 refines all of
them, and the cost is linear in `1/Δs`: at most `16385` poses at the default reading floor, and fewer where
the gate is met sooner (a disc of radius `5` spinning a quarter turn `7` mm from a wall: `513` poses). The layer
exclusion (§5.7) settles the common case of that shape, a stacked planar mechanism, before any pose:
scene 1 without the wall, and the same arms over a table, each evaluate the two endpoints and read
`Sound` at every resolution. A constant gap the rule cannot settle — a link turning about an axis that
leans off its partner's layer, or a pair whose extents along the axis overlap — still pays the linear cost.

## 11. Required tests

Every test asserts on computed geometry through the production path (CLAUDE.md "Correctness must be
observable"); bounds are asserted `InDelta` at a stated slack, never pinned (memory: float bounds are
arch-specific); each bound leg is deleted once and seen to fail, and the test file records which legs.

**Scene 1 — the folding arm (the first acceptance target).** Link 1, the upper arm `A`: motion §9's arm,
a prism over `x ∈ [0, 48], y ∈ [−14, 14]`, `z ∈ [0, 10]`, on a revolute joint about `Z` through the origin,
sweeping `0° → 90°`. Link 2, the forearm `B`: a prism over `x ∈ [48, 96], y ∈ [−14, 14]`, `z ∈ [12, 22]`,
under link 1 on a revolute joint about `Z` through `(48, 0, 0)`, sweeping `0° → −90°`. The two sweeps
cancel in orientation: at `s`, with `θ = 90°·s`, `B` keeps its zero-pose orientation and translates on the
circle of radius `48` about the origin, so its top face is the plane `y = 48·sin θ + 14`. Static: a wall
`x ∈ [−100, 150], y ∈ [38, 58], z ∈ [−10, 40]`, reaching past every cap so no pair shares a face plane.

- `B`'s top face reaches `y = 38` when `sin θ = 1/2`: `θ* = 30°`, `s* = 1/3`, in closed form. `A`'s far
  corner `(48, 14)`, at radius `50`, reaches `y = 38` at `θ = asin(38/50) − atan(7/24) ≈ 33.2°`,
  `s ≈ 0.369`, later. Past `s*`, `B`'s overlap with the wall is the slab `48 × (48·sin θ − 24) × 10` mm³.
- `A` and `B` never meet: their relative motion is joint 2 alone, about `Z`, so their `z`-extents
  `[0, 10]` and `[12, 22]` hold at every `s`, and the layer exclusion (§5.7) settles the pair with the
  lower bound `2` mm before any pose.
- At `WithResolution(units.Scalar(1.0/256))` assert: `Status` is `Interfering`; the first `LinkCollision`
  has `A` the forearm and `B` the wall, `At` strictly above `1/3` and within `2/256` of it — the grid
  point `86/256`, where the overlap is `480·(48·sin(30.234°) − 24) ≈ 81.5` mm³, asserted within `1e-3`
  mm³ with `Bound` below `Value`; every `IntervalClear` interval ends at or below `1/3`; the interval
  containing `1/3` is not `IntervalClear`; every `(A, wall)` collision has `At > 0.369`; no pose carries an
  `(A, B)` row. The check evaluates `11` poses.
- The same linkage with the wall removed, at the defaults: `Sound`, the two endpoints alone, one
  `IntervalClear` interval whose `Clearance` is exactly `2`, and no whole-drive `Clearance`. A `1` mm
  `WithMinClearance` is `AssessmentMet`; a `3` mm one is `AssessmentUndecided` with
  `DiagMotionUndecidedClearance`, since the layer's proven `2` mm cannot meet it and no pose measures the
  arms to disprove it.
- A post rigidly on `A`, `x ∈ [20, 30], y ∈ [26, 36], z ∈ [0, 30]`, shares `B`'s layer, so its pair is
  evaluated; its relative motion is joint 2 alone and its gap, `21`–`31` mm, certifies every interval at
  `WithResolution(Scalar(1.0/4))`. This is the leg that goes red when the common-ancestor joint is left in
  the pair's `τ`: every interval is then undecided at that floor. The travel terms themselves are pinned by
  the internal travel test below and by scene 4.
- `examples/` gains `Example_decad_linkageCheck` on this scene, printing `Status`, the first collision's
  bodies and its fraction to three decimals — `0.336` on every platform, since the grid is dyadic.

**Scene 2 — the stacked elbow and the declared contact.** Link 2 lowered to `z ∈ [10, 20]`, so its
underside shares `A`'s cap plane `z = 10`, with `DeclareJointContact(A, B)`; joint 1 unlisted (held at
`0`), joint 2 sweeping `0° → 180°`, swinging `B` back over `A`; a fence rigidly in link 1, a prism
`x ∈ [0, 10], y ∈ [−40, 40], z ∈ [10.5, 30]`, standing at `A`'s root and sharing no face plane with `B`.
`B`'s far corner `(96, 14)` sits at radius `50` and polar angle `atan(7/24)` from the elbow, and reaches
the fence's face `x = 10`, which is `38` mm behind the elbow, when `cos(φ + atan(7/24)) = −38/50`:
`φ* = acos(−19/25) − atan(7/24) ≈ 123.20°`, `s* = φ*/180° ≈ 0.6844`, at `y = 50·sin(φ* + atan(7/24))
≈ 32.5`, inside the fence. Every other point of `B`'s leading side reaches `x = 10` later (its radius is
smaller and its polar angle larger), and the trailing corner `(96, −14)` reaches it at `≈ 155.7°`.
Assert: `Interfering`, the first `LinkCollision` on `(fence, B)` — the fence's link comes first in pair
order — with `At` strictly above `s*` and within `2/256` of it at `WithResolution(Scalar(1.0/256))`: the
grid point `176/256`, where `B`'s leading corner pokes depth `d` past `x = 10` and the overlap is the
triangular prism `9.5·d²/(2·(−cos φ)·sin φ)` mm³, asserted within `1e-6`; some interval `IntervalClear`;
`JointContacts` lists `(A, B)`; no diagnostic names `(A, B)`. The same scene without the declaration holds
no interval clear and raises a finding naming `(A, B)`. A declared pair's other outcomes are pinned on
their own fixtures: the motion arm swinging away from a block `12` mm past its tip in its own layer,
declared, publishes its gap row at every pose and nothing else, so a `15` mm `WithMinClearance` raises no
finding and reads `AssessmentMet`; a block resting on a slab, and a
block sunk `1` mm into one across a shared face plane, each sliding along that plane, publish no row and
no finding; a block sliding inside a declared sheet raises no `DiagUnsupportedPairSheet`, which the
undeclared pair raises at every pose. The leg that a declared pair is still proven for overlap is pinned
separately: a
`10` mm cube on a prismatic joint sliding `0 → 5` mm along `X`, sunk `1` mm into a static slab
`x ∈ [−20, 40], y ∈ [−20, 20], z ∈ [−10, 0]` (cube `z ∈ [−1, 9]`, sharing no plane with the slab), the
pair declared: assert a `LinkCollision` on it at `s = 0` and `s = 1` with `Volume` within `1e-6` of `100`
mm³, and `Status` `Interfering`.

**Scene 3 — the crane.** Link 1: a mast `x, y ∈ [−5, 5], z ∈ [0, 38]` on a revolute joint about `Z`
through the origin, sweeping `0° → 90°`; link 2: a boom `x ∈ [10, 60], y ∈ [−5, 5], z ∈ [40, 50]`, `2` mm
above the mast, on a prismatic joint along `+X`, extending `0 → 30` mm; a wall `x ∈ [−100, 150],
y ∈ [60, 80], z ∈ [30, 100]`. The boom's tip corners sit at `(60 + 30s, ±5)` in the boom's own frame,
turned by `θ = 90°·s`, so the `+5` corner's height is `y = (60 + 30s)·sin θ + 5·cos θ`, increasing in
`s`, and it reaches the wall's face `y = 60` at `s* ≈ 0.535`, the one root of that equation in `[0, 1]`,
which the test brackets to `1e-12` by bisection of the closed form (`s* ≈ 0.535155`); the `−5` corner
follows later. Assert `Interfering` with the first collision on `(boom, wall)` within `2/256` above `s*` —
the grid point `137/256` — with the overlap the triangular prism `10·d²/(2·sin θ·cos θ)` mm³ for the
corner's depth `d` past `y = 60`, within `1e-6`. The boom slides along `X`, perpendicular to `Z`, so the
mast's `z`-extent `[0, 38]` and the boom's `[40, 50]` hold at every `s` and the layer exclusion settles
the `(mast, boom)` pair: no pose carries its row. The prismatic term is pinned with the mast unlisted and a `10` mm block `x ∈ [10, 20]` in the
boom's place (the `50` mm boom covers any pin it passes at the end): a `2 × 2 × 2` mm pin at
`x ∈ [24, 26]`, `4` mm ahead of the block at rest and `14` mm behind it at the end, with the endpoints
alone (`WithResolution(Scalar(1))`): no `Collision`, `IntervalUndecided`, `Suspect` — exactly motion §9
test 5's pin, which goes red when the `|Δq|` term of a prismatic joint is dropped.

**Scene 4 — a near miss between samples.** Motion §9 test 3's blade (`x ∈ [0, 50], y ∈ [−0.5, 0.5]`,
`z ∈ [0, 10]`) and pin (a `0.8` mm cube at radius `49`, polar angle `α = 90·31/64°`, `z ∈ [4.6, 5.4]`),
in two chains:

- **4a, the whole swing on joint 1.** Link 1: a hub `x, y ∈ [−5, 5], z ∈ [−10, −5]` on a revolute joint
  about `Z` through the origin, sweeping `0° → 90°`; link 2: the blade on a prismatic joint along `+X`,
  sliding `0 → 0.1` mm. `τ_blade = ρ_{12}·(π/2)·Δs + 0.1·Δs` with `ρ_{12} = 0 + √(50² + 0.5² + 10²) ≈ 51`,
  so test 3's arithmetic holds: at `WithResolution(Scalar(1.0/30))` assert no `Collision`, `Suspect`, the
  `IntervalUndecided` interval containing `31/64`; at `WithResolution(Scalar(1.0/900))` assert
  `Interfering` and the first `Collision.At` within `[31/64 − 0.013856, 31/64 + 0.013856]`. The
  `(hub, blade)` pair has a constant `5` mm gap and certifies at `Δs = 1/4` through joint 2 alone. This is
  the fixture that goes red when an ancestor joint's term is dropped from a link's `τ`: the blade's travel
  is then `0.1·Δs` and the coarse interval around the pin certifies falsely.
- **4b, the ball reading.** Link 1: the hub on the same joint 1; link 2: the blade at `x ∈ [50, 100]`
  pointing from its elbow at `(100, 0, 0)` back toward the base, on a revolute joint about `Z` through the
  elbow, held at `180°` (`From == To`), so the blade points outward and its tip is `150` mm from the base
  axis; the pin at radius `149`. The rest box about axis 1 reads `100`; the ball reads
  `100 + √(50² + 0.5² + 10²) ≈ 151`. Motion §9 test 15's arithmetic holds: at `WithResolution(Scalar(1.0/30))`
  the width-`1/32` interval around `31/64` has travel `≈ 7.4` mm against a gap sum of `5.18`, undecided, while
  `ρ = 100` gives `4.9` and certifies it falsely. Assert the undecided outcome, and `Interfering` at
  `WithResolution(Scalar(1.0/900))` with the first `Collision.At` within `[31/64 − 0.41/90, 31/64 + 0.41/90]`.
  This is the fixture that goes red when the revolute ball's radius is dropped from `ρ_{ik}`.

Measured: 4a's first collision lands at `s = 0.4707`, 4b's at `0.4805`, against `31/64 = 0.4844`, each
inside its window.

**Scene 5 — waypoints.** A hub `x, y ∈ [−5, 5], z ∈ [−20, −12]` on a revolute joint about `Z` through the
origin carries an arm `x ∈ [0, 50], y ∈ [−5, 5], z ∈ [0, 10]` on a prismatic joint along `+Z`. The drive has
three segments: the lift runs `0 → 20 → 20 → 0` mm and the swing `0° → 0° → 90° → 90°`, so the arm lifts
over `s ∈ [0, 1/3]`, swings over `[1/3, 2/3]` and lowers over `[2/3, 1]`. A post `x, y ∈ [17, 25],
z ∈ [−10, 15]` stands in the swing's path `5` mm below the lifted arm; a landing block `x ∈ [−20, 20],
y ∈ [30, 40], z ∈ [−10, 5]` sits under the arm's final place.

- The lowering arm's underside stands at `z = 60·(1 − s)` and reaches the block's top `z = 5` at
  `s* = 11/12`, in closed form; the overlap is then `100·(5 − 60·(1 − s))` mm³. At
  `WithResolution(Scalar(1.0/256))` assert `Interfering`, the first `LinkCollision` on `(arm, landing
  block)` at the grid point `235/256` with the overlap `7.8125` mm³ within `1e-6`, every collision past
  `s*` and every `IntervalClear` interval ending at or before it, no collision on the post, every pose's `Values` the closed-form schedule, and the intervals
  holding `1/3` and `2/3` each `IntervalClear`. The same swing with the lift unlisted strikes the post.
- The same folding arm as scene 1, its sweeps given collinear `Via` values (`45°` and `−45°`; then `30°, 60°`
  and `−30°, −60°`), reports exactly what the one-segment drive reports but for `Drive`.
- **A joint turning back.** A blade `x ∈ [0, 50], y ∈ [−0.5, 0.5], z ∈ [0, 10]` turns about `Z` through
  `0° → 60° → 10° → 20°`, and a `0.8` mm pin at radius `49` and polar angle `59.5°`, `z ∈ [4.6, 5.4]`,
  sits in its path only near the `60°` corner at the non-dyadic `s = 1/3`. Contact needs
  `|q − 59.5°| < 1.25°`, so every collision lies in `[58.25/180, 1/3 + 1.75/150]`. At
  `WithResolution(Scalar(1.0/128))` assert `Interfering`, every collision on the pin inside that window,
  and no `IntervalClear` interval holding `1/3`. At `WithResolution(Scalar(1.0/2))` assert `Suspect`,
  three poses, `[0, 1/2]` undecided and `[1/2, 1]` clear. This is the leg that goes red when `|Δq_i|` is
  the span of the interval's ends: the ends are `20°` apart, `[0, 1]` certifies from them, and the report
  reads `Sound` with two poses.
- The motion arm on a revolute joint about `Z`, a wall `x ∈ [−20, 20], y ∈ [30, 40], z ∈ [−10, 20]` where
  it points at `90°`, at `WithResolution(Scalar(1.0/2))`: `0° → 90° → 10°` strikes the wall at `s = 1/2`
  with the overlap `2800` mm³, red when `m_i` ignores `Via` (the swept box grows by `10°`'s reach and
  excludes the wall); `0° → 90° → 0°` strikes it, red when a link is read as held at `0` from its ends,
  and when the drive is read as a hold from them; `20° → 90° → 20°` strikes it, red when the link is read
  as one constant placement from its ends.
- Limits: a `Via` value past `Max` or below `Min` with both ends inside is `ErrDegenerate` naming the link
  and the value, red when only the ends are checked; a drive whose every waypoint is inside poses, and its
  value at the waypoint's fraction is the waypoint as stated. `PoseAt` on a four-segment drive, whose
  waypoints sit at dyadic fractions, returns each waypoint exactly at its fraction, the segment's line
  inside it, and the first and last segments' lines outside `[0, 1]`. An internal test pins `|Δq_i|`
  against the hand sum across one and two waypoints, exactly for a prismatic joint whose ends agree.
- `examples/` gains `Example_decad_linkageWaypoints` on the lift-swing-lower scene, printing `Status`, the
  first collision at `s = 0.918`, and its segment and local fraction: the lowering, `75%` through it.

**Agreement with `VerifyMotion`.** A one-link linkage on a revolute joint and the same body under the
equivalent `Revolute`, on motion §9's fixtures 1, 2 and 4 at the endpoints alone: equal `Status`, equal
interval outcomes in order, each `Collision.At` equal under `s ↦ From + s·(To − From)` within `1e-9°`,
path `Clearance.Value` within `1e-9` mm. The one-link `τ` is `ρ_{11}·|Δq_1|`, which is `MoverTravel`'s
value, and the ideal pose is one `MotionFrame.At`, so the two paths read the same bounds. Bisected, the
linkage also takes the projection bound and the reports part as §5.8 states.

**Exclusions and held links.** Each exclusion admits a pair without evaluating it, so each is pinned by a
fixture whose answer goes wrong when the rule lets through a pair it must not. The layer exclusion: the
motion arm turning `0° → 30°` about `Z` over a table `10` mm below is settled with the exact lower bound
`10`; a block under the arm turning `0° → 180°` about the `X` axis through `(0, 0, 10)`, and the same block
sliding `0 → 40` mm along `(1, 0, −1)`, each come down into a table and their collisions are found, red
when the axis or the perpendicularity test is skipped; the arm resting on a table's top face reads
`Suspect` with an `Exact` zero row, red when touching extents exclude; and a block on two slides along
`(1, 0, 1)` and `(0, 1, 1)` is settled against a box `140` mm away along their cross product, red when that
candidate is dropped. The link-link swept box: a far sibling block is settled, and a sibling block sliding
`30` mm toward the arm's swing, its rest box `46` mm from the arm's, collides — red when the boxes are not
grown by each link's travel. Held links: a base on a joint the drive never moves, resting on a table and
beside a wall, forms neither pair and reads `Sound`, red when its pairs are formed; held at `30°` its
wall row is the same at every pose, and an internal test shows its one transient placement reused and kept
cached across poses.

**The reading floor.** At `WithMotionTolerance(Scalar(1e-5))` the three-joint arm of §10 reads `Sound`, its
reading inside the gate, with `ReadingResolution` `1/16384` and some interval narrower than `1/1024`; red
when the reading floor is dropped. At the defaults the projection bound closes its reading before the
verdict floor (§5.8). At `WithResolution(Scalar(1.0/64))` it reads `Suspect` with no interval narrower than
`1/64`; red when a stated resolution leaves the reading floor at its default. Scene 1's arms against a
`3` mm margin read `AssessmentUndecided` with no interval narrower than `1/1024`; red when the margin
refines to the reading floor. The quarter-turn disc beside a wall evaluates exactly `513` poses and reads
`Sound`, its reading enclosing the true `7` mm.

**Standing tests.** Errors, one subtest per row of §8 and per constructor refusal of §2; non-mutation and
determinism as motion §9 test 7; cancellation; pose deviation charged (a joint centre at `(1e6, 0, 0)`
widens a row's `Bound` as motion §9 test 8 shows, carried through a child link's composition); swept-box
exclusion of a far body, and a swept box grown by every ancestor joint's travel from the zero pose (scene
1's shoulder swinging `80° → 90°` with the elbow held carries the forearm into a wall `46` mm beyond its
zero-pose box); `PoseAt` composition pinned on three joints with closed-form world points (`(96, 0, 0)` on
link 2 under scene 1 at `s = 1/3` is `(48·cos 30°, 48·sin 30°, 0) + (48, 0, 0)`). Internal tests in
`linkage_internal_test.go` pin the ball propagation on a four-joint chain — revolute, prismatic, revolute,
prismatic, so every row of §5.2's table is walked — against hand-computed radii and show each ball step go
red when its term is dropped; pin the telescoping `τ` against the hand sum for a static partner and below
two common ancestors, each joint's term red when dropped; pin the ideal pose of scene 1's forearm against
its closed-form images, red when the parent's pose or the composition order is dropped; and pin a link-link
pair's gap as the kernel's widened by `η_A + η_B`, red when the partner's `η` is dropped.

`.github/test-shards.txt` and `.github/test-shards-apitest.txt` are updated for every test and example,
and `go test . ./apitest/ -run '^TestCI'` is run before the push.

## 12. Increments

| PR | lands | still `Suspect` after it |
|---|---|---|
| 1 (`linkage.go`, `linkage_verify.go`, `linkage_bound.go`; `motion_verify.go` generalised to several moving groups with per-group poses, ideal frames and group-group pairs, `VerifyMotion` bit-identical as the one-group case, every motion test unchanged) | §2's vocabulary, `PoseAt`, §5's bounds and certificate, `VerifyLinkage` with every option, both bisection steps, swept-box exclusion against statics, scene 1 and its example, the agreement tests, the errors, non-mutation and cancellation tests | every pair that touches at a joint: declarations are PR 2 |
| 2 | `DeclareJointContact`, `JointContacts`, §5.4's asymmetric outcome; `WithJointLimits` with `JointOption`, `JointLimits` and the joints' `Limits` fields; scenes 2, 3 and 4; the three-joint benchmark of §10 | a clear drive's whole-drive reading at the default floor (§10) |
| 3 | §6 step 2's held links — a link holding `0` stands where it is and forms no pair with a static body, a link held elsewhere is one constant placement; link-link swept-box exclusion; the layer exclusion (§5.7); their tests | a clear drive's whole-drive reading at the default floor where its minimum is reached at one parameter (§10) |
| 4 | the reading floor (§3): `ReadingResolution`, the reading's refinement past the verdict floor at the defaults, the margin held to the verdict floor; its tests | a stated `WithResolution` too coarse for the reading, and a constant gap the layer exclusion cannot settle whose gate needs a step under `1/16384` |
| 5 | waypoints (§2.3): `JointSweep.Via`, equal-share segments, `PoseAt` per segment, limits at every waypoint, `m_i`, holds and standings over every waypoint, `|Δq_i|` cut at every waypoint inside an interval (§5.2); scene 5 and its example | as after PR 4 |

PR 1 is the end-to-end instance: two real links, a real fixture, the real kernel, the chain certificate,
one report, with the closed-form answer of scene 1 as its acceptance. This design document ships in PR 1.

## 13. Settled points

The linkage check lives in the root package, not a subpackage, because the per-pose kernel is private
and a public multi-body primitive carrying rational-interval ideal poses is not a shape core §5 admits
(§1). Links are sets of bodies and joints attach a link to its parent; the zero pose is the document as
it stands; joint frames are world coordinates at the zero pose (§2.1). v1's joints are revolute and
prismatic; there is no fixed joint, because a rigidly attached part is listed in its link (§2.1). The
drive is one dimensionless fraction with a piecewise-linear schedule per joint; a box of joint values is the
second entry point `VerifyJointBox` (§2.3, §14). A closed loop is a tree plus a revolute closure, its
dependent joints read from `sketch.Enclose`'s certified enclosure and never from a float solve (§15).
Joint contacts are declared by the caller per pair, listed in the report, and still checked for
overlap at every pose (§2.2, §5.4). The travel bound is the telescoping joint-by-joint sum with ball
enclosures read down the chain, over exact rationals (§5.2); a pair's bound sums only the joints below the
two links' lowest common ancestor (§5.2). A pair whose relative path turns about axes parallel to one
direction and slides perpendicular to it, with the two bodies' extents along that direction strictly
separated, is settled before any pose by an exact layer exclusion (§5.7). A drive passes
through waypoints stated per joint as `Via` values, each segment an equal share of `s`, and an interval
holding a waypoint takes each joint's travel on both sides of it (§2.3, §5.2). The options and the report
vocabulary are `VerifyMotion`'s (§3, §4).

## 14. The joint box

`Document.VerifyJointBox` answers "is every configuration in this box of joint values clear?" — the
question §9.2 defers — with the same per-pose kernel, the same travel bound and the same report
vocabulary as `VerifyLinkage`, over cells of joint space instead of intervals of one parameter. A drive
is one path through joint space; a box is every configuration in `[Min_1, Max_1] × … × [Min_n, Max_n]`
of the joints the caller varies, with the other joints held. Everything §1–§8 states holds here unless
this section says otherwise, with "the drive's range" read as "the box": a joint's reach `m_i` is
`max(|Min_i|, |Max_i|)`, and a held joint's reach its held value.

### 14.1 The entry point

```go
// JointBox is a box of joint values. A listed joint ranges over [Min, Max]
// when Min < Max and holds at Min when Min == Max; an unlisted joint holds 0.
type JointBox []JointRange

type JointRange struct {
    Link     *Link
    Min, Max units.Value // the joint's Kind: Angle for a revolute, Length for a prismatic
}

func (d *Document) VerifyJointBox(ctx context.Context, l *Linkage, box JointBox, opts ...JointBoxOption) (*JointBoxReport, error)

// JointBoxOption configures VerifyJointBox. Every MotionOption is one.
type JointBoxOption interface{ /* sealed, option.Interface */ }

// MotionOption configures VerifyMotion and VerifyLinkage; it embeds
// JointBoxOption, so the three shared options pass to VerifyJointBox unchanged.
type MotionOption interface { JointBoxOption /* sealed */ }

// WithCellBudget caps the number of cell centres the check evaluates
// (§14.4); default 16384. cells < 1 is ErrDegenerate.
func WithCellBudget(cells int) JointBoxOption

// JointConfiguration is one point of joint space: every link's joint value,
// its proven half-width (zero for a stated joint; §16.3) and world pose, in
// Linkage.Links() order.
type JointConfiguration struct {
    Values []units.Value
    Bounds []units.Value
    Poses  []r3.Transform
}

// Configuration builds every link's pose at the stated joint values, one per
// link in Links() order; it composes exactly as PoseAt composes (§2.4).
func (l *Linkage) Configuration(values []units.Value) (JointConfiguration, error)
```

A **varying** joint is a listed joint with `Min < Max`; `n` is their count. A box with no varying joint
names one configuration, which `PlacedCopy` then `Verify` already answers, and is `ErrDegenerate`, as a
drive in which every sweep holds is. `Min > Max` is `ErrDegenerate`: a box has no sense of traversal, so
nothing is run the other way. A range's `Kind`, finiteness and limits are checked as a sweep's are (§2.4):
both ends inside the joint's declared limits put the whole range inside them, and an unlisted joint whose
limits exclude `0` is refused with a message naming the link. A link named twice is `ErrDegenerate`.

`WithMotionTolerance`, `WithResolution` and `WithMinClearance` keep §3's meanings. The resolution is a
`Dimensionless` fraction **of each varying joint's own range**: `WithResolution(Scalar(1.0/64))` lets no
cell be narrower than `(Max_i − Min_i)/64` along joint `i`, whatever its `Kind`. Unstated, the verdict
floor is `Scalar(1.0/1024)` and the reading floor `Scalar(1.0/16384)` per axis, as §3 fixes them for a
drive; stated, one value is both floors. `WithCellBudget` is the box's own: the sealed set is widened by
making every `MotionOption` a `JointBoxOption`, so passing the budget to `VerifyMotion` or `VerifyLinkage`
does not compile, and no runtime refusal is needed. `Configuration` refuses a value count other than
`len(Links())` (`ErrDegenerate`), a wrong `Kind` (`ErrUnitKind`), a non-finite value (`ErrNotFinite`), a
value outside its joint's limits (`ErrDegenerate`, naming the link) and a pose `r3` cannot represent
(`ErrNotFinite`). It is the one place a tree linkage's configuration is built, so a renderer drawing a
cell's corner and the verifier evaluating its centre read the same transform; `PoseAt` is `Configuration`
of the drive's values at `s`. A looped linkage's configuration is a held drive (§16.1).

One joint varying is legal and is not what the box is for: a drive over the same range costs about half
the poses — an interval's endpoint serves two intervals, a cell's centre serves one cell (§14.3) — and
brackets the onset of a collision, which §14.4 does not. `VerifyLinkage` stays the entry point for one
parameter.

### 14.2 The report

```go
type JointBoxReport struct {
    Request           JointBoxRequest   // the validated effective settings, including defaults
    ReadingResolution units.Value       // the reading floor (§14.1)
    Linkage           *Linkage
    Box               JointBox          // as stated
    Links             []*Link           // Linkage.Links() order
    Against           []*Body           // every static body, in Document.Bodies() order
    JointContacts     []DiagnosticPair  // every declared joint contact, in declaration order
    Cells             []JointCellResult // the leaves of the subdivision, in cell order (§14.4); they tile the box
    CellsEvaluated    int               // every centre evaluated, split cells included
    Collisions        []JointBoxCollision // every proven collision at every evaluated centre, in evaluation order
    Clearance         *ScalarReading    // the minimum gap over the whole box; nil unless every cell is CellClear
    Assessment        Assessment        // against WithMinClearance
    Diagnostics       []Diagnostic      // per leaf its centre's findings then its own, in cell order; then split cells' collisions and violations; then the budget's; then the reading's
    Status            Status
}

func (r *JointBoxReport) Passed() bool // Status == Sound

type JointBoxRequest struct {
    RelativeTolerance units.Value
    Resolution        units.Value  // Dimensionless, per axis
    MinClearance      *units.Value // non-nil exactly when requested
    CellBudget        int
}

// JointCell is a box of joint values inside the stated box: per link, in
// Links() order, the least and greatest value the cell holds. A held or
// unlisted joint has Min == Max.
type JointCell struct {
    Min, Max []units.Value
}

type JointCellResult struct {
    Cell          JointCell
    Outcome       CellOutcome
    Center        JointConfiguration // the evaluated centre
    Interferences []Interference     // at the centre; A moves, B is static or belongs to a later link
    Clearances    []Clearance        // at the centre, the same A/B rule
    Diagnostics   []Diagnostic       // the centre's undecided or unsupported pairs, and the cell's own finding
    Clearance     *Measurement       // a proven lower bound on the gap over the whole cell; nil unless CellClear
}

type CellOutcome int
// CellNotEvaluated | CellClear | CellBlocked | CellColliding | CellUndecided

// JointBoxCollision is a proven overlap at an evaluated centre, about the ideal
// poses (§5.1), with the configuration as its witness.
type JointBoxCollision struct {
    Configuration JointConfiguration
    A, B          *Body
    Volume        Measurement
}
```

| Outcome | Claim |
|---|---|
| `CellClear` | at EVERY configuration of the cell, every undeclared evaluated pair has disjoint interiors, and every declared pair is free of transferred overlap at the centre; `Clearance` is a proven lower bound on the gap over the cell (§14.3) |
| `CellBlocked` | at EVERY configuration of the cell, some pair overlaps (§14.3); the centre's collisions are its witnesses |
| `CellColliding` | a proven collision sits at the centre; nothing is claimed about the rest of the cell. The floor or the budget stopped the split that would have told more |
| `CellUndecided` | none of the above: the centre certifies nothing and the floor or the budget stopped the split |

`CellNotEvaluated` is the zero value and never appears in a returned report: every published cell's centre
was evaluated (§14.4). `Interference`, `Clearance`, `Diagnostic`, `ScalarReading`, `Assessment` and the
pair order of §4 are reused as they are; `A` is the earlier body in that order. The four `DiagMotion*`
codes carry the same findings: a `CellUndecided` cell raises `DiagMotionUndecidedInterval`, a `CellClear`
cell whose bound does not reach a requested margin `DiagMotionUndecidedClearance`, a centre's transferred
overlap `DiagMotionCollision`, and a centre's gap below the margin `DiagMotionClearanceViolated`. One code
is added: `DiagJointBoxBudgetExhausted` (`joint_box_budget_exhausted`, `Suspect`, raised at most once, with
`Pair`, `Body` and `Cell` nil) says that `WithCellBudget` stopped a split the floor would have allowed, and
its `Message` names the budget and the cells it held.

**`Diagnostic` gains one field.** `Cell *JointCell` is the cell a joint-box finding concerns: for a cell
finding the cell itself, for a centre finding the cell whose centre was evaluated — which may since have
been split, so it need not be in `Cells`. It is nil on every diagnostic `Verify`, `VerifyMotion` and
`VerifyLinkage` emit, and on the budget and reading findings. It is additive, as `At` was (motion §4.1),
and `report.go` owns it; verification §1.1 gains the one-line statement that it exists. `At` is nil in a
joint-box report: a cell has no scalar parameter.

`Cells` lists the leaves of the subdivision and nothing else, so they tile the box exactly, as
`Intervals` tile `[0, 1]`. A split cell's centre is gone from `Cells`, but a collision proven there is a
fact the caller asked for, so `Collisions` keeps every transferred collision at every evaluated centre, and
`Diagnostics` keeps their `DiagMotionCollision` findings. A margin disproven at a split cell's centre is
kept the same way, as its `DiagMotionClearanceViolated`, so a `Violated` assessment always carries its
finding. A split cell's other centre findings are not published, because its leaves carry their own.

### 14.3 What a cell proves

**Travel over a cell.** For a cell `C = Π [a_i, b_i]` with centre `m`, and a configuration `q` in `C`,
§5.2's telescoping bound moves one joint at a time from `m` to `q`: a point of link `k` travels at most
`Σ_i w_i·|q_i − m_i|`, with `w_i = ρ_{ik}` for a revolute joint `i` on the link's path and `1` for a
prismatic one. `ρ_{ik}` is read over the whole box (`readLinkBounds` with `m_i = max(|Min_i|, |Max_i|)`),
so one reading serves every cell. Since `|q_i − m_i| ≤ (b_i − a_i)/2`,

```text
τ_half(C) = ½ · Σ_i w_i · span_i(C),    span_i = MotionParam.SpanUpper of the cell's two ends along joint i
```

bounds the travel of every point of the link from the centre to any configuration of the cell. For a pair
the sum runs, as in §5.2, over the joints strictly below the two links' lowest common ancestor on each
branch, each body with its own `ρ`; a (link body, static) pair sums every joint on the body's path. A held
joint has `span_i = 0` and contributes nothing; its value still enters the balls, as a held sweep's does.

**The centre certificate.** With `lo_m` the proven lower end of the pair's gap at the centre after `η`
(§5.1), the 1-Lipschitz fact of motion §5.2 gives `gap(q) ≥ lo_m − τ_half` for every `q` in `C`: the
**travel bound** over the cell, positive exactly when

```text
lo_m > τ_half(C)      (compared over exact rationals; lo_m the float the pose proved)
```

The cell's **projection bound** for the pair is §5.8's `L_n(C)`, read from the centre's ideal poses with
`h_i = span_i(C)/2`, the largest over the six coordinate directions. The pair's proven lower bound over
the cell is the larger of the two, rounded down, and the cell is clear for the pair exactly when that
larger bound is positive. The cell is `CellClear` when every undeclared evaluated pair certifies and no
declared pair collides at the centre; an excluded pair (§5.7, §6 step 4) contributes its proven lower
bound and nothing else. A touching or undecided centre certifies nothing for that pair, as a touching
endpoint does (motion §5.2, §5.8).

**Why the centre and not the corners.** §9.2's sketch evaluated a cell's `2^n` corners. Both forms rest on
the same `τ_half`: from any configuration of the cell, the nearest corner is at most `τ_half` away in the
weighted travel metric, and so is the centre. The corner form certifies on `min_c lo_c > τ_half`; the
centre form on `lo_m > τ_half`. For a gap that is linear across the cell the two sides agree to first
order, and for one that bows the centre reading is the one that is not pulled down by the farthest
corner. The costs differ: a centre is one pose per cell, while bisecting one axis of a cell adds
`2^(n−1)` new corners, so corners cost `2^(n−1)` poses per split even when shared between neighbours — equal
at `n = 2`, double at `n = 3`. The centre is taken. What corners would buy is the mixed-cell rule of §6
step 5 (some corners colliding, some clear ⇒ the boundary passes through); the blocked certificate below
replaces it on the colliding side, and the floor brackets the boundary on the clear side.

**The blocked certificate.** The overlap volume of two bodies changes, along any path in joint space, at a
rate bounded by each body's surface area times its speed: the derivative of `vol(M_a ∩ M_b)` is the
integral over the part of `∂M_a` inside `M_b` of the normal velocity, plus the same for `M_b`, so
`|Δvol| ≤ A_a·(travel of a) + A_b·(travel of b)`, with `A` a proven upper bound on the body's area at
rest, which a rigid motion preserves: `motionMover.area`, the body's own certified area enclosure
`Area.Value + Area.Bound` rounded up. An area below the true one is unsound here, since it lets a cell block
that a configuration can clear. On the straight joint-space segment from `m` to
`q`, a point of link `k` moves at speed at most `Σ_i w_i·|q̇_i|` — the instantaneous form of the
telescoping bound, the velocity under joint `i` alone being `|q̇_i|·dist(y, axis_i) ≤ |q̇_i|·ρ_{ik}` with the
prefix an isometry — so its travel is at most `τ_half`. A pair whose transferred collision at the centre
has `V_lo = Volume.Value − Volume.Bound` (the published bound, `η`'s allowance already in it) therefore
overlaps at every configuration of the cell when

```text
V_lo > SweptVolumeAllow(τ_half(a), A_a) + SweptVolumeAllow(τ_half(b), A_b)
```

with each term `proofbound.SweptVolumeAllow`'s up-rounded product, the right side read back as exact
rationals and the comparison exact; `τ_half(b)` is `0` for a static partner. A cell with such a pair,
declared or not, is `CellBlocked`, and is not split: the question inside a proven collision is answered.
A colliding centre whose volume does not clear the allowance leaves the cell `CellColliding` until a split
produces children whose centres decide more, or the floor stops it.

**Declared pairs, exclusions, held links and limits** are §5.4, §5.7, §6 steps 2 and 4, and §2.4 verbatim
over the box: a declared pair runs at every centre, publishes a transferred collision and a measured gap
row, enters no certificate, and claims nothing continuous; a link whose path joints all hold `0` stands as
static and forms no pair against a static body; a link whose path joints all hold is one constant
placement; the swept-box exclusion grows each body's rest box by its link's reach `Σ ρ_{ik}·m_i` over the
box; the layer exclusion admits a direction every varying joint on the relative path keeps. The settled
pairs are settled before any cell, and `settlePairs` runs unchanged.

**What is exact, what is bounded.** §5.3's table holds. `span_i` for an angle carries `π` at its upper
enclosure; every product and sum in `τ_half` and the allowance is `big.Rat` arithmetic; a cell's ends and
centre are dyadic fractions of each range, exact in float, and each joint's value at the centre is the
label `motionDomain.label` gives a pose at that fraction, while every bound reads the exact
`Min_i + f·(Max_i − Min_i)`.

### 14.4 The procedure

1. **Validate** (§14.1, §14.6) before reading `ctx`.
2. **Resolve each link's standing, read the bounds, settle pairs** as §6 steps 2–4 over the box.
3. **Evaluate the root.** The whole box is one cell; its centre is evaluated as §5.1 evaluates a pose,
   under the link poses `Configuration` builds and the ideal poses `MotionFrame.At` composes at the exact
   centre values. The root's centre is always evaluated, so declared pairs are checked at least once.
4. **Classify** the cell from its centre: `CellBlocked` when the blocked certificate holds for some pair;
   else `CellColliding` when some pair collides there; else `CellClear` when every pair certifies — the
   larger of its travel bound and its projection bound positive (§14.3) — else `CellUndecided` for now.
5. **Split for the verdict.** A cell is **splittable** when it is `CellUndecided` or `CellColliding` and
   some varying joint's span is wider than the verdict floor. Its split axis is the varying joint
   maximising `w_i·span_i` over the pairs that held the cell back — the uncertified pairs of an undecided
   cell, the colliding pairs of a colliding one — ties to the earliest link, among the axes wider than the
   floor; halving that axis lowers `τ_half` the most for the pair that needs it. Among the splittable
   cells, the next to split is the **shallowest first, then earliest in cell order**: level by level, so
   that a budget that runs out leaves the whole box examined coarsely rather than one corner finely. The
   cell is replaced in place by its two halves, lower half first, each with its centre evaluated and
   classified; **cell order** is that left-to-right order of the subdivision tree, and `Cells` lists the
   leaves in it. A cell no centre can certify — one held back by a pair that is never evaluated, an
   invalid operand or a sheet (§7) — is not split: it is published `CellUndecided` with that pair's own
   finding, since no split changes the pair's standing.
6. **Split for the reading and the margin**, once no splittable cell remains: while every leaf is
   `CellClear` and the whole-box reading would fail the tolerance gate, split the clear cell holding the
   smallest lower bound (ties in cell order) along the axis with the largest share of that bound's defect
   for the pair that attained it — `w_i·span_i` for a travel bound, the projection bound's own share
   (§5.8) otherwise — while that axis is wider than the reading floor; and while a requested margin is neither
   proven by every cell's bound nor disproven by some centre's `hi`, split the same cell on the same terms
   while the axis is wider than the verdict floor. A child evaluated here is an ordinary cell: it can
   collide, be blocked, or be undecided, and step 5 then resumes on it.
7. **The budget.** Every split evaluates two centres. When fewer than two evaluations remain of
   `WithCellBudget`, no cell is split: each stands as classified, and the report raises
   `DiagJointBoxBudgetExhausted` once. A budget of `1` evaluates the root alone.
8. **Publish** (§14.2). `Status` is verification §6's worst-wins aggregate over the findings; `Assessment`
   is `AssessmentViolated` when some centre proves `hi < minimum`, else `AssessmentMet` when every leaf is
   `CellClear` with a bound at or above the minimum, else `AssessmentUndecided`; `Clearance` is present
   only when every leaf is `CellClear`, and is motion §5.3's reading with the leaves' bounds as the lower
   end and the smallest evaluated `hi` as the upper end.

`ctx` is checked before every centre and inside every kernel call. The subdivision is a deterministic
function of the inputs — dyadic cells, a fixed split rule, a fixed order — so two calls on the same inputs
return reports equal in every field. The pose count is `CellsEvaluated ≤ CellBudget`.

### 14.5 What is never claimed

§5.6 holds, and in addition:

- Nothing about a configuration outside the box.
- Nothing about a `CellColliding` cell beyond its centre, and nothing about a `CellUndecided` cell. The
  boundary of the colliding region is bracketed by the undecided and colliding leaves around it, to the
  floor or the budget, and never reported as a curve.
- A `CellBlocked` cell claims overlap of SOME pair at every configuration; it does not claim that the pair
  found at the centre is the one overlapping everywhere when several collide there — the certificate is
  per pair, and the report names the pairs that held it.
- No tolerance decides admission: every comparison above is a strict inequality over bounds rounded
  against the claim.

### 14.6 Errors

`VerifyJointBox` returns `(*JointBoxReport, error)` by core §12. §8's table holds with "drive" read as
"box" and "sweep" as "range", and these rows are added or changed:

| Condition | Error |
|---|---|
| a box with no varying joint; a range with `Min > Max`; a link named twice | `ErrDegenerate` |
| `WithCellBudget` below `1` | `ErrDegenerate` |
| `Configuration` with a value count other than `len(Links())`, or a value outside its joint's limits | `ErrDegenerate` |
| a sheet, an invalid operand, an undecided pair, a floor or a budget reached | a finding, never an error |

### 14.7 Cost

Each centre costs one pose, as §10 prices it: about `3` ms for the three-joint arm's four pairs. Cells
certify when `lo_m > τ_half`, so a clear region at gap `g` resolves into cells of weighted side about
`2g/Σ_i w_i·range_i` per axis and the leaf count grows as the product over axes.

- **Two joints.** Scene 6 (§14.8): the crane's boom under the mast's quarter turn (`ρ ≈ 91`) and a `30`
  mm slide, so `τ_half` at the root is `½·(91·(4π/9) + 30) ≈ 79` mm. At `WithResolution(Scalar(1.0/64))`
  the leaves number at most `4096`; the estimate is a few hundred clear leaves, about `90` undecided leaves
  along the boundary curve, and the colliding side split to the floor. The blocked certificate does not
  shorten that: the boom's overlap never exceeds about `2100` mm³ against an area of `2200` mm², so a
  colliding cell blocks only once its `τ_half` falls under about `0.9` mm, near `1/128` of the `θ` range,
  below this floor. Measured: `2953` centres into `1477` leaves (`88` clear, `98` undecided, `1291`
  colliding), about `15` s and `1` GB, and `70` s under the race detector, the overlap-volume proof at each
  colliding centre taking nine tenths of it; at `Scalar(1.0/16)`, `239` centres in about `1` s. The test
  runs at `Scalar(1.0/16)` for that cost.
- **Three joints.** The three-joint arm of §10 with each joint over `[0°, 90°]`. Measured: the verdict
  is cheap, every leaf `CellClear` within `127` centres, about `0.4` s, since the elbows are declared, the
  shoulder-wrist pair is settled by its layer and only the wrist's pairs with the posts are evaluated. The
  whole-box reading is not under the travel bound alone. The wrist's leading corner `(150, −10)` sits at
  `X = 50·(cos θ₁ + cos(θ₁ + θ₂) + cos(θ₁ + θ₂ + θ₃)) + 10·sin(θ₁ + θ₂ + θ₃)`, whose largest value over
  the box, `100 + √2600`, is reached at `(0°, 0°, atan(1/5) ≈ 11.31°)` with the corner at the post's
  mid-height, so the box's minimum gap is `60 − 10·√26 ≈ 9.0098` mm, on the box's edge `θ₁ = θ₂ = 0`,
  and every first derivative of the gap vanishes there: a **flat minimum**. The gap grows with the square
  of each joint's turn while `τ_half` shrinks only linearly with the cell, so step 6 crowds cells around
  that edge. Under the travel bound alone the default budget evaluates `16383` centres in about `43` s and
  `1` GB and reads `Suspect`, the reading beyond tolerance and the budget finding raised, and
  `WithCellBudget(65536)` takes about `6` minutes and `2.1` GB and still does not meet the gate. The
  projection certificate (§5.8) bounds the gap over a cell to second order: with `Σ B_ij ≈ 700` mm/rad²
  for this pair, the reading closes at a half-span of about `4e-3` rad, depth `8` along each joint, in an
  estimated thousand centres; §14.9's P2 measures and records the count here. A caller who wants the
  verdict alone states `WithResolution`, which stops the reading at the same floor. The layer exclusion is
  what makes a planar stack cheap: a pair it settles costs no cell, and scene 1's arms without the wall
  read `Sound` from the root alone.
- **The reading.** Around an isolated minimum, step 6 costs about `n·log₂(1/ReadingResolution)` splits;
  a gap constant along one axis makes every cell along it tie, and the cost is linear in that axis's cell
  count, capped by the budget.

### 14.8 Required tests

§11's standard holds: every assertion is on computed geometry through the production path, every bound
is `InDelta` at a stated slack, and each guarding leg is deleted once and seen to fail.

**Scene 6 — the crane's box (the acceptance target).** Scene 3's mast and boom, the mast turning
`θ ∈ [0°, 80°]` and the boom sliding `d ∈ [0, 30]` mm, against a wall `x ∈ [−100, 150], y ∈ [62, 82],
z ∈ [30, 100]`. The boom's `+5` tip corner sits at `y(θ, d) = (60 + d)·sin θ + 5·cos θ`, which increases
in `d` and, since `∂y/∂θ = (60 + d)·cos θ − 5·sin θ > 0` for `θ < 85.2°`, in `θ` over the whole box. The
colliding region is therefore `{y(θ, d) > 62}`, closed toward larger `θ` and `d`, and its boundary is the
one curve `d*(θ) = (62 − 5·cos θ)/sin θ − 60`, from `θ* = asin(62/√(90² + 5²)) − atan(5/90) ≈ 40.29°` at
`d = 30` to `d* ≈ 2.075` at `θ = 80°`. The `(mast, boom)` pair is settled by the layer exclusion along `Z`
and the `(mast, wall)` pair by its swept box, so each centre evaluates `(boom, wall)` alone. At
`WithResolution(Scalar(1.0/16))` — §14.7 gives the cost at `1/64` — assert:

- `Status` is `Interfering`; the leaves tile the box: their fraction extents sum to `1` and no two overlap;
  every leaf's `Center` is the midpoint of its `Cell`.
- Every `CellClear` leaf has `y < 62` at its `(Max θ, Max d)` corner — monotonicity makes that corner the
  whole cell's worst — and its `Clearance.Value` at or below `62 − y` there, the true minimum gap over the
  cell. This is the leg that goes red when `τ_half` is halved again or a joint's term is dropped from it.
- Every `CellBlocked` leaf has `y > 62` at its `(Min θ, Min d)` corner — none blocks at this floor, and
  the leg that charges the certificate too small an area goes red here, a cell straddling the curve
  reading blocked; every `CellColliding` leaf has
  `y > 62` at its centre; every `CellUndecided` and `CellColliding` leaf is at the floor along both
  joints; and an undecided leaf's centre lies within its own `τ_half` of the boundary,
  `|62 − y| ≤ ½·(ρ·Δθ + Δd)` with `ρ = 35 + √675 + 30` the ball reading. Within one cell's width of the
  curve is not the claim: near `θ = 80°` the tip rises only about `5` mm per radian of `θ`, while `ρ`
  charges the whole boom's travel, so a floor cell a full width clear of the curve can stay undecided.
- Every `JointBoxCollision` has `y > 62` at its configuration; where the corner's depth `δ = y − 62` is
  below `10·cos θ`, `50·sin θ` and `20`, so that only the one corner has crossed, its `Volume` is the
  triangular prism `10·δ²/(2·sin θ·cos θ)` within `1e-6`, `Bound` below `Value`; at least one collision is
  that shallow.
- `CellsEvaluated` is below the default budget and no `DiagJointBoxBudgetExhausted` is raised. The test
  file records the measured count.
- `examples/` gains `Example_decad_jointBox` on this scene at `WithResolution(Scalar(1.0/16))`, printing
  `Status`, the first collision's bodies and its `θ` and `d` to two decimals; the centres are dyadic, so
  the printed values hold on every platform.

**The clear box and its reading.** The same box with the wall moved to `y ∈ [100, 120]` and the mast cut
to `z ∈ [0, 20]`: at scene 6's `38` mm mast the layer exclusion's `2` mm between mast and boom would be the
box's minimum, a bound no pose measures, and the reading could never close. Now the layer bound is `20` mm
and the minimum gap over the box is `100 − y(80°, 30) = 100 − 90·sin 80° − 5·cos 80° ≈ 10.499` mm, at the
box's corner. Assert `Sound`, every leaf `CellClear` with its bound below the true gap at its worst corner,
`Clearance` enclosing `10.499` with `ToleranceSatisfied` at the defaults, `ReadingResolution` `1/16384`,
some leaf narrower than `1/1024` along `θ`, fewer than `1024` centres (measured: `137`; the minimum sits
at the box's corner with a nonzero gradient, so the reading's upper end, a centre's gap, closes only
linearly in the cell and the projection certificate changes this count little); `WithMinClearance`
`10` mm `AssessmentMet`, at the default tolerance and at a relative tolerance of `0.5`, where only the
margin's own refinement proves it; `11` mm `AssessmentViolated` with a `DiagMotionClearanceViolated` whose
`Cell` is set. At `WithResolution(Scalar(1.0/64))` the reading is beyond tolerance and the report `Suspect`
with no leaf narrower than `1/64`; red when a stated resolution leaves the reading floor at its default.

**Scene 12 — the three-joint box's flat minimum (the projection certificate's acceptance target).** §10's
three-joint arm with each joint over `[0°, 90°]`, at the defaults. Its minimum is §14.7's `60 − 10·√26
≈ 9.0098` mm at `(0°, 0°, atan(1/5))`, where the wrist's corner `(150, −10)` faces the post at
mid-height. Assert `Sound`, no `DiagJointBoxBudgetExhausted`, `Clearance` enclosing the minimum with
`ToleranceSatisfied`, `CellsEvaluated` below `4096` (the count recorded), and for every `CellClear` leaf
whose centre puts that corner within the post's face (`|Y| ≤ 10` at the centre) a bound at or below the
closed-form gap `160 − X` at its centre, which is at least the cell's minimum; for every other leaf a bound
at or below the centre row's upper end. Red when the projection bound is dropped (the budget is exhausted
and the report reads `Suspect`, as §14.7 records) and when the linear term is dropped (a leaf far from the
minimum reads a bound above the gap at its own centre). The leg that the remainder is load-bearing lives on
the pendulum below, whose gap is concave: here the gap is convex around its minimum and a dropped
remainder does not overshoot.

**Scene 13's box — the pendulum under a slide (the prismatic-ancestor rule).** §5.8's pendulum hung from a
carriage on a prismatic joint along `X`, the box `d ∈ [0, 30]` mm × `θ ∈ [0°, 90°]`. The wall is wide, so
the gap `g(θ) = 20 − 5·sin θ + 40·cos θ` is constant along `d`, and the shallower joint is a prismatic,
so `B_11 = B_12 = 0` and `Rem = ½·ρ_22·h_θ²` carries no slide term. Assert `Sound` at the defaults,
`Clearance` enclosing `15`, every `CellClear` leaf's bound at or below `g` at its greater `θ` end — the
cell's true minimum, since `g` decreases — and no leaf narrower than the box along `d`: the reading splits
along `θ` alone. Red when a prismatic ancestor is charged `w_j` in `B` (the remainder then carries
`h_θ·h_d` terms and the reading splits along `d`), and when `Rem` is dropped or halved (a leaf's bound
exceeds `g` at its far end). At `WithCellBudget(1)` the root's bound is below `15`.

**Scene 14's box — the two arms.** §5.8's two arms with each joint over `[−30°, 30°]`: the gap `110 −
50·(cos θ_A + cos θ_B) − 5·(|sin θ_A| + |sin θ_B|)` has four flat minima of `110 − 10·√101 ≈ 9.501` mm, at
`(±θ*, ±θ*)` with `tan θ* = 1/10`, each with some corner pair at a shared height. Assert `Sound`,
`Clearance` enclosing `9.501`, every `CellClear` leaf's bound at or below the closed-form gap at its centre,
and `CellsEvaluated` below the budget with the count recorded; red when the partner's expansion is dropped
from a link-link pair.

**The blocked box.** The near wall again, over `θ ∈ [70°, 80°]`, `d ∈ [25, 30]`: `y(70°, 25) ≈ 81.6 > 62`,
so every configuration collides, and at `θ = 75°, d = 27.5` the boom passes through the whole wall with
overlap `100·20/sin θ ≈ 2071` mm³ against a boom area of `2200` mm², so the blocked certificate closes
once `τ_half` falls under about `0.9` mm — cells of `1/32` along `θ` and `1/8` along `d`; measured, `255`
centres into `128` leaves. Assert every leaf `CellBlocked`, `CellsEvaluated` below `1024`, no undecided or
merely colliding leaf. Red when the blocked
certificate is dropped: the run then splits to the floor, exhausts the budget, and reads `CellColliding`
with `DiagJointBoxBudgetExhausted`.

**The budget.** Scene 6 at `WithCellBudget(16)`: `CellsEvaluated ≤ 16`, `DiagJointBoxBudgetExhausted`
raised once, some leaf wider than the floor that is `CellUndecided` or `CellColliding`, every leaf's centre
evaluated. At `WithCellBudget(1)`: one leaf, the whole box, its centre `(40°, 15)` clear
(`y = 75·sin 40° + 5·cos 40° ≈ 52.0`), outcome `CellUndecided`.

**One varying joint.** Scene 2's elbow as the box `[0°, 180°]` with the shoulder unlisted: `Interfering`;
every `CellClear` leaf ends at or below `s* ≈ 0.6844` of the range, every `CellBlocked` leaf begins above
it, and the `(A, B)` declared pair raises no finding. A held joint contributes nothing to `τ_half`, while
the link it holds is still read where it holds it: scene 4b's blade held at `180°` on its own joint, the
hub's joint the one varying joint over `[0°, 90°]`, at `WithResolution(Scalar(1.0/900))` reads
`Interfering` with a witness within `0.41/90` of `31/64` of the range, and goes red, the cell around the pin
certified clear, when the revolute ball's radius is dropped from `ρ_{ik}`.

**Standing tests.** Errors, one subtest per row of §14.6 and per shared row of §8; non-mutation and
determinism; cancellation; pose deviation charged at a centre; `Configuration` pinned against scene 1's
closed-form point and against `PoseAt` at `s = 1/3`. Internal tests in `linkage_box_internal_test.go` pin
`τ_half` for scene 6's root against `½·(ρ·(4π/9) + 30)` with `ρ` the ball reading, each joint's term red
when dropped; the split-axis choice (the `θ` axis first at the root, since `91·(4π/9) > 30`); the
shallowest-first order on a hand-built tree; and the blocked allowance as `A·τ_half` per moving body, red
when a static partner is charged an area or a moving one is not.

`.github/test-shards.txt` and `.github/test-shards-apitest.txt` are updated for every test and example,
and `go test . ./apitest/ -run '^TestCI'` is run before the push.

### 14.9 Increments

| PR | lands | still `Suspect` after it |
|---|---|---|
| 1 (`linkage_box.go`; `motion_verify.go`'s `evaluatePose` split into building a pose's groups and running its pairs, `intervalOutcome`'s pair walk and `conclude`'s status fold shared, `VerifyMotion` and `VerifyLinkage` bit-identical) | `JointBox`, `JointRange`, `JointBoxOption` with `MotionOption` embedding it, `WithCellBudget`, `JointConfiguration` and `Linkage.Configuration`; `VerifyJointBox` with the centre certificate, `CellClear`/`CellColliding`/`CellUndecided`, step 5's split rule and order, the floor and the budget, `Diagnostic.Cell`, `DiagJointBoxBudgetExhausted`, the settled pairs, held links and declared contacts over the box, the whole-box reading over the leaves as they stand; scene 6's verdict and tiling assertions at `WithResolution(Scalar(1.0/16))`, since every colliding cell splits to the floor, its example, the budget, one-joint and standing tests | a colliding region's interior: every colliding cell splits to the floor or the budget |
| 2 | the blocked certificate and `CellBlocked`; scene 6's blocked assertions, still at `Scalar(1.0/16)` (§14.7), the blocked box, and the blocked allowance pinned per moving body | a clear box's whole-box reading at the default floor |
| 3 | step 6: the whole-box reading's refinement, the reading floor and `ReadingResolution`, the margin; the clear box and its reading; the three-joint cost of §14.7 measured and recorded | a stated `WithResolution` too coarse for the reading; a gap constant along an axis, or a flat minimum, whose gate needs more than the budget |
| P1 (`linkage_bound.go`: the corner velocities, `B_ij`, `Rem` and `L_n` over the six directions, read once per pose; `motion_verify.go`: `motionDriver` gains an optional projection bound per pair and interval, `intervalOutcome` takes the larger of the two bounds, `singleMotion` supplies none so `VerifyMotion` is bit-identical; `linkage_verify.go`: the driver's bound for a tree, nil on a path with a dependent joint) | §5.8's interval form for `VerifyLinkage`; the three-joint drive, scene 13, scene 14 and the internal tests of §5.8; the agreement test restated; §10's measured counts, `TestVerifyLinkageReadingFloor`'s default leg and the benchmark's reported poses re-measured and recorded | the box's flat minimum (§14.7); a disc or a tilted contact still pays the travel bound's linear cost |
| P1b (`linkage_bound.go`: `jointStep` and the segment term in `projectionSide`; `linkage_verify.go`: the driver's steps per interval, negated from the far end) | §5.8's segment term on an interval no waypoint bends; the three-joint drive's `rel = 1e-5` leg and the reading floor's leg there; the out-and-back pin; the segment term's internal test; the measured counts re-recorded | as after P1 |
| P2 (`linkage_box.go`: `classify` takes the larger of `lo_m − τ_half` and `L_n(C)`; `readingAxis` and `splitAxis` read the attained bound's own shares, §5.8) | §5.8's cell form for `VerifyJointBox`; scene 12, scene 13's and scene 14's boxes; §14.7's three-joint count and the clear box's `137` re-measured and recorded | a disc or a tilted contact in a box |

PR 1 is the end-to-end instance: the real crane, the real kernel, the cell certificate over real cells,
one report, with scene 6's closed-form region as its acceptance. This section ships in PR 1. P1 is the
projection certificate's end-to-end instance: the real three-joint drive, the real kernel, the bound read
off real poses, one report, with its closed-form minimum as acceptance; P2 adds the cell form on it. Both
leave `VerifyMotion`'s reports, every collision and every outcome the travel bound certified unchanged —
a pose the larger bound spares lies inside an interval or cell proven clear, where no collision sits.
Where the projection bound is the larger they raise interval and cell lower bounds, turn undecided
intervals and cells clear, and evaluate fewer poses or centres. The whole-drive reading is then read over
fewer poses and closes at the first step that meets the gate, so its interval can sit slightly higher or
wider while inside the gate; each test that pins a measured count is re-measured in the PR that moves it.
Later increments, each a tightening with no soundness change (§5.8): the kernel's closest-point direction
as a seventh `n`;
a body's own support along `n` from the kernel's carriers, for discs and tilted contacts; the expansion
over a dependent joint's enclosure; the same certificate for `VerifyMotion`, which changes motion §9 test
2's default-resolution leg and its reports.

### 14.10 Settled points

A box is stated per joint as a range, held joints as `Min == Max`, unlisted joints at `0`, with no
traversal sense (§14.1). The options are `VerifyLinkage`'s plus a cell budget, and the sealed set is
widened by embedding so the budget cannot reach the one-parameter checks (§14.1). The cell certificate
evaluates the centre and charges half the cell's weighted span, not the `2^n` corners: the bound is the
same and the cost is one pose per cell (§14.3). A colliding cell is proven colliding throughout by the
volume's Lipschitz bound in the bodies' areas, with the same `τ_half` (§14.3). Bisection halves one axis,
the one whose halving lowers `τ_half` most for the pair that held the cell back, shallowest cell first
(§14.4). The resolution is a fraction of each joint's own range, and the budget counts evaluated centres
(§14.1, §14.4). Leaves tile the box; a split cell's collisions stay in the report (§14.2). One diagnostic
code and one `Diagnostic` field are added (§14.2). A pair's bound over a cell, and over a drive's
interval, is the larger of the travel bound and the projection bound: the gap bounded below by the
bodies' separation along a coordinate direction, each body's rest-box corners expanded to second order in
the joint values from the centre or an end, with the remainder proven from `ρ` (§5.8, §14.3). The six
coordinate directions are the only candidates in v1, and `VerifyMotion` keeps the travel bound alone
(§5.8).

## 15. Closed loops

A **loop** is a tree plus one **closure joint**: a revolute pin that joins two links which both already
have parents. In a four-bar the crank, the coupler and the follower hang off the ground as a tree, and
the pin at the coupler's far end closes the loop onto the follower; in a slider-crank the rod's far pin
closes onto the slider. The caller states one joint of the loop in the `Drive`; every other joint of the
loop is **dependent**, and its value at a parameter is whatever closes the loop there. That is a 2D
constraint problem on a planar loop, and CLAUDE.md sends it to `sketch`: decad builds a private scene from
the loop's own pins and bar lengths, asks `Sketch.Enclose` for a certified enclosure of the exact solution
over a range of the driver's value, and consumes what it returns — the enclosed value of each dependent
joint, charged into `η` at a pose and into `τ` over an interval. decad computes no 2D answer itself and
admits no pose on a float solve; every check it runs on what `sketch` returns can only refuse. Everything
in §1–§8 holds for a looped linkage unless this section says otherwise; the kernel, the certificate, the
bisection and the report vocabulary are unchanged. `linkage_loop.go` owns this section's code.

### 15.1 Vocabulary

```go
// Close joins a and b, two distinct links of l that already have parents, with
// a revolute pin at center about axis (world coordinates at the zero pose), so
// the tree gains one loop. Which joint drives the loop is stated per Drive.
func (l *Linkage) Close(a, b *Link, center, axis r3.Vec) (*LinkageLoop, error)
func (l *Linkage) Loops() []*LinkageLoop // in Close order

// LinkageLoop is one closure and the links it ties together.
type LinkageLoop struct { /* private */ }

func (lp *LinkageLoop) Common() *Link          // the two links' lowest common ancestor: the ground for a four-bar
func (lp *LinkageLoop) Links() []*Link         // every link on the loop but Common, in Linkage.Links() order
func (lp *LinkageLoop) Closure() RevoluteJoint // Center and Axis as stated; Limits nil
func (lp *LinkageLoop) Bars() []LoopBar        // Common's bar first, then Links() order; none for a slide (§15.2)

// LoopBar is the length the private scene holds a link's two loop pins at:
// the exact distance between them in the loop's plane. Length.Value is the
// up-rounded root; the scene states the bar as the interval
// [Value − Bound, Value], which holds the exact length. Exactness is Exact
// when the length is a float, with Bound zero.
type LoopBar struct {
    Link   *Link
    Length Measurement
}
```

The type is `LinkageLoop`, not `Loop`: `Loop` is the topology's face boundary (`topology.go`).

**The loop's joints.** The loop runs from `a` up to `Common` and down to `b`; its joints are the tree
joints of every link in `Links()` and the closure. `Common` may be `a` or `b` itself — `Close(L4, L1)`
on a chain `ground → L1 → L2 → L3 → L4` has `Common = L1` — or the ground, as in §15.10's four-bar, two
branches off the ground. Each link of `Links()` has exactly two **loop pins**: its own joint and the joint
of the next link along the loop (the closure for the two links `a` and `b`). A revolute link's **bar** is
the segment between them, and `Common`'s bar joins its two pins: the first joint on each side, or the
closure on a side with no link. A loop holds at most one prismatic joint, the **slide**, a child of
`Common`: its loop pins are its **rail**, the line through its next pin's zero-pose position along its
`Dir`, and that next pin, which rides the rail. A slide has no bar, and neither has `Common` on a loop with
a slide, since one of `Common`'s loop pins is then the rail.

**A drive names the driver.** A `Drive` lists at most one joint of a loop. That joint is the loop's
**driver** for the drive, and every other joint of the loop is dependent; a drive that lists two joints of
one loop is `ErrDegenerate`, since the second value cannot be stated. A drive that lists none, or whose
listed joint holds `0`, holds the loop at the zero pose, where every joint value is `0`: its links are the
tree's held links (§6 step 2) and no scene is built. The driver's sweep may carry `Via` values, hold a
nonzero value, or cross `0` (§15.8). v1 takes a driver only when its parent is `Common`,
so that its reference line (§15.2) is fixed in the scene: a four-bar is driven at the crank or at the
follower, a slider-crank at the crank or at the slide, and a drive listing the coupler's joint is
`ErrUnsupported`.

**Admission.** `Close` refuses, in this order, and every test is an exact rational comparison with no
tolerance (`linkage_bound.go`'s `ratCross`, `ratDot`):

| Condition | Error |
|---|---|
| nil `l`, nil or parentless `a` or `b`, `a == b`, a link of another linkage | `ErrDegenerate` |
| a non-finite `center` or `axis` component | `ErrNotFinite` |
| a zero `axis` | `ErrDegenerate` |
| a loop revolute's `Axis` not exactly parallel to `axis`; a loop prismatic's `Dir` not exactly perpendicular to `axis` | `ErrUnsupported`, naming the joint |
| a second prismatic joint on the loop; a loop prismatic whose parent is not `Common` | `ErrUnsupported` |
| a tree joint already on another loop (two loops may share `Common` and nothing else) | `ErrUnsupported` |
| two loop pins of one link coincident in the loop's plane, `Common`'s two included | `ErrDegenerate` |

`axis` may point anywhere: a loop about a tilted axis states each pin's plane position to `sketch` as an
exact enclosure (§15.2). Every other joint of the linkage, off the loop, is unrestricted. The
closure takes no `JointOption`: its relative angle is not a value the drive states or the report
publishes, so limits on it have nothing to compare against. The closing pin's bodies go through
`DeclareJointContact` like every other joint's (§2.2): nothing is inferred from the closure.

### 15.2 The scene

decad builds one private `sketch.Sketch` per loop and per drive, from the loop's recorded pins and nothing
else, as `prism_boolean.go` builds a scene from recorded sections. The scene is never published and no
`sketch` type crosses decad's API.

**Frame.** The plane's axes are `u* = u/|u|` and `v* = v/|v|`, with `u` and `v` exact rational directions,
`v = n × u` for the closure axis `n` as stated, so `u* × v* = n/|n|` exactly. A loop with a slide takes `u`
the slide's own `Dir`, so the slide's displacement is its pin's `u*`-coordinate change. An all-revolute
loop about `±e_i` takes `u = e_{i+1}`, so `v*` is `±e_{i+2}` — `(X, Y)` for `+Z`, `(X, −Y)` for `−Z`,
`(Y, Z)` for `+X`, `(Z, X)` for `+Y` — and any other all-revolute loop takes `u = n × e_m`, `e_m` the
coordinate axis along which `n` has its smallest component. Each of these `u` is a float vector read
exactly, and `v` is formed from it in exact rational arithmetic. A pin `p`'s plane position is
`(p·u/|u|, p·v/|v|)`: `p·u` and `p·v` are exact rationals, and `|u|` and `|v|` lie between the two floats
around their exact roots (`proofbound.RatSqrtDown`, `RatSqrtUp`), so each coordinate is enclosed in a
rational interval about `1e-16` of its size wide. The third coordinate, the pin's height along `n`, is
dropped, because a rotation about an axis parallel to `n` moves a point within its own height. On a
coordinate-axis loop whose slide, if any, runs along a coordinate axis, `|u|` and `|v|` are exact and
each enclosure is one float, two of the pin's own coordinates. The float positions the scene's points are
created at come from `r3.NewFrame(origin, u, v)`'s `ToLocal`; they seed the solves and claim nothing.

**Points and lines.** Each loop pin is one `*sketch.Point` at its zero-pose plane position, shared by the
two links it joins. Every fixed point is stated to `Enclose` at its exact plane position: where either
coordinate's enclosure is not one float, as on a tilted loop, with `WithFixedBox(p, x, y)` (sketch #155)
over the outward-rounded float box `x × y` of the enclosure, so every claim holds for every position in
the box and so for the exact one. `Common`'s two pins are fixed (`Sketch.Fix`), and `Common`'s line joins
them. Each revolute link of `Links()` is a `Line` from its own pin to its next pin and a `NewDistance`
between them.
A slide's rail is the fixed line from a fixed point `P₀` at its next pin's zero-pose position to a fixed
point one millimetre along `u`, and its next pin `P` is free with `NewPointOnLine(P, rail)`; the rail is
the slide's line and `Common`'s, and `Common`'s other pin is fixed.

**The bars are the document's.** A bar's exact length is the root of the exact rational squared distance
`L² = |d|² − (d·n)²/|n|²` between its two pins in the plane, `d` their difference, which is in general
irrational. Each `NewDistance` carries the target `RatSqrtUp(L²)` and is stated to `Enclose` with
`WithTargetRange(d, RatSqrtDown(L²), RatSqrtUp(L²))` (sketch #155): every claim of every enclosure holds for every length in that interval, so for the exact
one, and the scene describes the document's own loop. `LoopBar.Length` publishes the interval — `Value` the
up-rounded root, `Bound` the one-float gap down to the lower root, `Exact` with a zero `Bound` when `L²` is a
float's square. Nothing about the bars is left uncharged.

**Dimensions.** The driver, with parent `Common`, is stated against a fixed reference. A revolute driver
takes the fixed line from its `Center` to a fixed point at its next pin's zero-pose position, and the
driving `NewAngle(ref, bar, 0)` from that line to the driver's bar; a slide takes the driving
`NewHorizontalDistance(P₀, P, 0)`. At the zero pose either reads exactly `0`, so the driver's target IS
the joint value in the scene's sense. Every dependent joint carries a driven dimension
(`SetDriven(true)`): `NewAngle(parentLine, bar)` from its parent's line — `Common`'s line for a child of
`Common`, the rail for a child of the slide — to its own, or for a dependent slide
`NewHorizontalDistance(P₀, P)`. The closure has no dimension. Only kinds `Enclose` certifies are used:
coincident points (by sharing), fixed points, point-on-line, distance, horizontal distance, angle.
`Sketch.Solve` runs once after construction so each driven dimension's target is its zero-pose reading,
the whole-turn reference `Enclose` shifts a driven angle toward; a horizontal distance is never read
modulo anything, and its whole-turn count (§15.3) stays `0`.

**Signs and the scene's side.** A loop revolute's `Axis` is exactly parallel to `n` with sense `s_k = ±1`
(the sign of the exact dot product). A right-handed rotation by `q` about `+n` is a counterclockwise turn
by `q` in `(u, v)`, so a joint's value `q_k` appears in the scene as `s_k·q_k`, and a driven angle's
reading maps back as `s_k·(reading − reading₀)` (§15.4); a slide's `s_k` is the sense of its `Dir` against
`u`. `Enclose` solves at the range's lower end and
continues upward only (`WithContinuation` requires `lo == prev.Range().Hi`), and the branch decad states is
the zero pose, so every chain of enclosures starts at the driver value `0` and grows toward larger values.
The scene is therefore built on the side of the plane that makes the driver's scene value `|q|`: for a
drive that takes a revolute driver's value negative in the scene, `v` is negated (the mirror image, normal
`−n`, every revolute `s_k` flipped), and for a slide driven backward `u` and `v` are both negated (a half
turn in the plane, the same normal, the slide's `s_k` flipped); readings map back through that side's
signs. Every side describes one mechanism and seeds at the same zero pose. A drive builds the scene for
every side its sub-segments read (§15.8): one for a driver that keeps one side of `0`, both for one that
crosses it.

**The zero pose, `E0`, and the one falsifier.** The first call on the scene is `Enclose(ctx, driver, 0, 0)`
with the bars' target ranges and the fixed boxes: one piece, a box around the seed in which the exact
solution at driver value `0` is the only one for every bar length in its interval and every fixed position
in its box, and for each driven dimension the interval `reading₀` of its zero-pose value. `reading₀` is the
offset every dependent value is measured from. The document's pins satisfy every equation of the scene
exactly at driver value `0` — the bars are their exact distances, held in the stated intervals, the fixed
pins sit at their exact positions, held in their boxes, and the driver's reference point is its own pin —
so they are one of the solutions `E0` claims to hold uniquely. Each pin's exact plane position is known as
its enclosure (§15.2's frame), and a pin whose enclosure is not proven inside its `E0` point box is refused:
outside it, the pin holds some other solution, of another branch, which disproves the enclosure, and where
the two only overlap nothing places the pin inside. decad refuses the loop with `ErrUnsupported`. A fixed
pin's point box is its own fixed box, which holds its enclosure. A pin inside its box admits nothing;
admission is `E0`'s own certificate. `E0` refusing is an error from the call (§15.6).

### 15.3 The enclosure chain

**The driver's value is enclosed, not stated.** `q(s) = From + s·(To − From)` is an exact rational turn and
radian pair (§2.3), and a turn is irrational in radians, so decad asks about the float range
`[q_lo(s), q_hi(s)]` that bounds `|q(s)|` from outside: `proofbound.RatFloatDown` and `RatFloatUp` of the
rational interval `2π·|turn| + |base|` with `π` at both enclosures, in radians, or the exact millimetre value
read as the float it is when it is one. A pose at `s` is a **point ask** `Enclose(driver, q_lo, q_hi)`, one
piece about `1e-13` wide; an interval `[s_a, s_b]` is a **cell ask** `Enclose(driver, q_hi(s_a), q_lo(s_b))`,
its range ordered by the scene's `q`. A cell whose range would be empty is refused as `sketch` refuses one.

**Every ask is continued from a canonical predecessor**, so that each enclosure certifies the zero-pose
branch and two calls at the same parameter return the same floats. Each sub-segment of the drive
(§15.8) has its own chain, on its own side's scene, running from its **near end**, the end whose driver
value is nearer `0`, toward its far end:

| Ask | Continued from |
|---|---|
| `E0` | nothing; the seed is the scene's zero pose |
| the **approach**, when the near end has `q_near > 0`: `Enclose(driver, 0, q_lo(s_near))` | `E0` |
| the point at the near end | the approach, or `E0` itself when `q_near = 0` |
| the cell of a dyadic interval `[s_a, s_b]` of §6's grid, read in chain direction | the point at its chain-start |
| the point at a grid parameter `s` of depth `d ≤ 30` (the least `d` with `s` a multiple of `2^−d`), other than the near end | the depth-`d` cell that ends at `s` in chain direction |
| the point at a deeper parameter (a renderer's frame at `i/300`, as a float) | the cell from the nearest depth-`14` grid parameter on its near side (`linkageReadingFloor`) to it, itself continued from that grid parameter's point |
| any point or cell of a sub-segment that holds its driver | the point at its near end: the sub-segment has one driver value, so one point |

A cell's chain-start never passes the sub-segment's near end: where the grid rule would start a cell
beyond it — a sub-segment that starts at a waypoint `1/3` or a zero crossing — the cell starts at the near
end. A point at `s` of depth `d` thus reaches back to the near end through the cells of the binary
expansion of `s`, each shared with every other parameter that passes through it. The schedule (§15.7)
caches every enclosure by its sub-segment, its kind and its ends. A refused ask makes every ask continued
from it refused with the same
cause.

**Why two chains agree.** The cell `[s_a, s_b]` is continued from the point at `s_a`; the point at `s_b`
is continued from a cell of its own depth, in general a different one. Each enclosure claims, per piece,
exactly one solution in its box for every `q` of its range, continuous in `q`, and that its first piece
holds the solution its predecessor ended on. Two enclosures over overlapping ranges that both descend from
`E0` describe the same solution: at the first `q` where two such continuous solution paths could part,
both lie in the box of a piece that holds exactly one solution there, so they coincide, and by continuity
they coincide on the whole overlap. decad consumes this consequence of `sketch`'s claims and proves
nothing about the geometry itself; the hull of the point at `s_a`, the cell and the point at `s_b` is
therefore an enclosure of the dependent's whole value set over `[s_a, s_b]` (§15.5).

**Whole turns.** `Enclose` reads a driven angle modulo `2π`, its first piece within half a turn of the
target at call time and each later piece within half a turn of the one before. Every call's target is the
zero-pose reading, so a dependent that turns more than half a turn from the zero pose reads, in a fresh
call, a representative a whole turn away from its predecessor's. Each enclosure therefore carries a whole
turn count per dependent, its predecessor's plus the integer `m` that brings its first piece's reading
next to its predecessor's last; a reading shifted by `m` turns that is proven disjoint from the
predecessor's, for every `π` in its enclosure, disproves the continuation and refuses the ask. A
dependent's value is then `2π·turns + reading − reading₀`, a turn and a radian interval, as a
`MotionParam` carries an angle.

### 15.4 What a pose proves

A dependent revolute's **value** at `s` is `θ_k(s) = s_k·(2π·turns + reading(s) − reading₀)`, the
interval difference of the point ask's `Driven` hull and `E0`'s over exact rationals
(`[lo − hi₀, hi − lo₀]`, the sign applied by swapping ends), so `θ_k(0)` is an interval around `0` of
twice the box width and the value at every other `s` is the joint's turn from the zero pose. The joint's
`MotionParam` pair is the turn and the two radian endpoints. A dependent slide's value is
`s_k·(reading(s) − reading₀)` in millimetres, its `MotionParam` pair the two millimetre endpoints. The
driver's value is the stated `q(s)`, exact.

**The float pose is built at the label**, `units.New(mid, units.Radian)` — `units.Millimeter` for a slide —
with `mid` the float nearest the interval's midpoint; `LinkagePose.Values[k]` carries it, as every stated
joint carries its own label
(§2.3). The labels of every link — stated or dependent — go through `linkageSpec.posesOf`, the one pose
builder `PoseAt`, `Configuration`, `VerifyLinkage` and `VerifyJointBox` share, so a loop adds no second
place a pose is composed. **The ideal pose is the joint's rotation by the whole interval**:
`motionbound.MotionFrame.AtRange(lo, hi)` builds the Rodrigues matrix from the sine and cosine enclosures
of `ParamSinCos(lo)` widened on both sides by `lo.SpanUpper(hi)`, since sine and cosine are 1-Lipschitz
in the angle, and the exact translation by `[lo, hi]` along the unit direction for a prismatic. `AtRange(p,
p)` is `At(p)`. The ideal poses compose as §5.1 composes them, each stated joint at its exact value; `η_k`
is `PoseDeviation` against that enclosure, so the distance between the float pose at the label and any
rotation in the interval is charged: about `ρ_{kk}` times the half-width, `1e-12` mm for a point ask on a
`100` mm link. `η` grows through the composition for every link below a dependent joint. A collision
transfers through that `η` or it is not a collision, and a gap row is widened by it, as §5.1 states.

`LinkagePose` carries each value's proven half-width (§2.4):

```go
type LinkagePose struct {
    At     units.Value
    Values []units.Value  // each link's joint value at s: stated, or the enclosure's midpoint label
    Bounds []units.Value  // each link's proven half-width about Values[k], in its Kind; zero for a stated joint
    Poses  []r3.Transform
}
```

`Bounds[k]` is the larger distance from the label to an end of the interval, rounded up, in radians, so
`Values[k] ± Bounds[k]` encloses the exact value as a `Measurement`'s `Value ± Bound` does. It is zero, in
the value's own unit, for every joint whose value the drive states, so a tree linkage's poses are
unchanged but for the field.

### 15.5 What an interval proves

§5.2's certificate needs, for every `s` in `[s_a, s_b]`, the travel from `s_a` to `s` plus the travel from
`s` to `s_b` within `τ`, joint by joint. A stated joint is monotone within a segment and its two travels
sum to the span of its ends. A dependent joint is not: the crank-rocker's follower turns back inside
`[0°, 90°]`. Its `|Δq_k|` over an interval is read from the hull `H = [h_lo, h_hi]` of its value over the
interval (§15.3) and the enclosures `A = [a_lo, a_hi]`, `B = [b_lo, b_hi]` of its values at the two ends,
each as rational radians with `π` at the enclosure end that widens it: the two one-sided travels from any
`x` in `H` are `|x − q_a| + |q_b − x|`, a convex function of `x` whose maximum over `H` sits at an end of
`H`, so

```text
|Δq_k| = max( a_hi + b_hi − 2·h_lo,  2·h_hi − a_lo − b_lo )     (exact rationals)
```

It equals the hull's width where the joint is monotone across the interval and twice it where the joint
returns to its starting value, which is the joint's total variation in each case. An interval holding a
sub-segment boundary — a waypoint or a zero crossing of the driver — is cut there, each piece read from
its own sub-segment's point asks and cell, and the pieces' terms summed: total variation is additive over
the cut, and the two one-sided travels from any `x` in one piece are bounded by that piece's term plus
every other piece's, each of which bounds the joint's change across it. The term enters
`chainTravel` as every other joint's does, multiplied by `ρ_{ik}` for a revolute. The engine reads the
interval's three asks before any pair (`linkageDriver.intervalGate`); an interval whose point or cell ask
`sketch` refused is `IntervalUndecided` whatever its pairs prove (§15.6).

**`m_i` and the balls.** A dependent joint's reach `m_i` (§5.2) is the largest `|value|` over the
certified drive: the hull of every certified cell of the schedule's decomposition (§15.7) and of the
points at its ends. The ball reading, the swept-box reach `Σ ρ_{ik}·m_i` and the layer exclusion are then
§5.2, §6 step 4 and §5.7 verbatim. Every later point or interval whose dependent values reach past `m_i`
is refused — unbuildable or undecided — so every value a claim is made about lies within the reach every
swept box was grown by. Every loop joint keeps `a·x` for `a = n`, so a stacked planar loop's link-link
pairs are layer-separated as a stacked arm's are.

**Limits.** A dependent joint with `WithJointLimits` is held to that same whole-drive hull once the
decomposition has read it: a hull not proven inside `[Min, Max]` — its lower end at or above `Min`'s upper
enclosure and its upper end at or below `Max`'s lower — is `ErrDegenerate` with a message naming the link
and the hull. The hull is wider than the exact value set by the pieces' slack, so a drive that reaches a
limit exactly is refused; the refusal is conservative and admits nothing. A dependent is never held to the
`0` an unlisted joint holds, so limits that exclude the zero pose admit a drive that stays inside them.

### 15.6 Refusals

`Enclose` returns a nil enclosure and a wrapped sentinel. decad maps each as follows; every mapping either
refuses the call or claims less, never more:

| `sketch` refusal | At `E0` | At any later ask |
|---|---|---|
| `ErrUnderconstrained`, `ErrRedundant` (DOF with the driver held — a loop whose Jacobian is singular at the zero pose, a loop of two links) | `ErrUnsupported`, wrapping the sketch error so `errors.Is` finds both | the ask is refused |
| `ErrNotConverged`, `ErrNotCertified` (no configuration, the Krawczyk test did not close, a fold inside the range, the piece budget) | `ErrUnsupported` | the ask is refused |
| `ErrUncertifiedConstraint`, `ErrForeignHandle`, `ErrNonFiniteGeometry` | an invariant failure: that error, no report | the same |
| `ctx.Err()` | `ctx.Err()`, no report | the same; never cached |

A refused point ask leaves its pose **unbuildable**, and a refused cell ask its interval undecided.
`Enclose` never returns `ErrUncertifiedConstraint` on a scene decad built, because §15.2 uses certified
kinds only; the row is the invariant's statement. `sketch` does not prove folds: the non-Grashof four-bar
driven into its fold refuses the cell holding the fold, and every point past it is continued from that
cell and refused with its cause, `ErrNotCertified`; decad reports both as the rows say, never as a proven
fold.

**An unbuildable pose is not evaluated, and an interval that ends at it is undecided.** §6's bisection is
unchanged except where a pose it asks for is unbuildable: the interval is `IntervalUndecided` and is
bisected while it is wider than the verdict floor, and an interval both of whose ends are unbuildable is
never bisected. At publication, the two intervals on either side of an unbuildable pose inside the drive
merge into one `IntervalUndecided`. `Poses` therefore lists every pose the loop could be enclosed at,
`Intervals` still tile `[0, 1]`, and an interval may end at a parameter that has no pose — exactly one such
interval ends at `1` when the drive leaves the certifiable range and does not return. Each undecided
interval raises `DiagMotionUndecidedInterval` with `At` its `From`, and the `Message` of one the loop
refused names the loop by its two closed links and carries `sketch`'s cause; no new `DiagnosticCode` is
added. A refusal is never an error from `VerifyLinkage` after validation.

The errors added to §8, all before `ctx` is read:

| Condition | Error |
|---|---|
| a drive listing two joints of one loop | `ErrDegenerate` |
| a drive listing a loop joint whose parent is not `Common` | `ErrUnsupported` |
| `E0` refused as the table above says, or a document pin outside `E0`'s box (§15.2) | `ErrUnsupported` |
| a dependent joint whose whole-drive hull is not proven inside its limits (§15.5) | `ErrDegenerate`, after the decomposition reads the hull |
| a looped linkage given to `Linkage.Configuration` (§16.1) | `ErrUnsupported` |

`E0` is asked under a context that is never canceled (`context.WithoutCancel`), so its refusal is a
validation error even under a canceled context; the decomposition of §15.7 follows the cancellation check.

### 15.7 `PoseAt`, and the schedule a renderer calls per frame

```go
// Schedule is a drive prepared for repeated PoseAt calls on one linkage. For
// every loop the drive moves it holds the private scene of §15.2 and the
// enclosure chain of §15.3, built as far as the drive could be certified.
type Schedule struct { /* private */ }

func (l *Linkage) Schedule(ctx context.Context, d Drive) (*Schedule, error)
func (s *Schedule) PoseAt(ctx context.Context, at units.Value) (LinkagePose, error)
func (s *Schedule) Drive() Drive
```

`Schedule` validates the drive as `PoseAt` does (§2.4) and §15.6's rows, then for each loop the drive moves
builds the scene, asks `E0` and the approach, and **decomposes** the drive: it asks `[0, 1]` as one cell,
replaces a refused cell by its two halves, and so on down to a floor — `1/1024` for `Schedule`, the
verdict floor for `VerifyLinkage` — so the dependents' reach (§15.5) is known before any pose is built. A
tree linkage's schedule holds the validated drive and nothing else. `Schedule.PoseAt(ctx, at)` builds the
pose of §15.4 at `at`: a point ask and the cells its chain needs, each taken from the cache or asked and
cached. Two calls at the same `at` on one schedule, or on two schedules of the same inputs, return equal
poses bit for bit, because every ask is a deterministic function of the parameter under §15.3's canonical
rule and `Enclose` is deterministic on the same state. `PoseAt` is safe for concurrent callers: the
schedule serialises the asks on each scene behind a mutex (`Enclose` must not run concurrently on one
sketch), and the cache is read under it. An `at` outside `[0, 1]` is legal for a tree linkage as before and
`ErrUnsupported` for a drive that moves a loop, since the chain is built over the drive; an `at` whose
point ask is refused is `ErrUnsupported` wrapping `sketch`'s error.

`Linkage.PoseAt(d, at)` is unchanged for a tree linkage and for a drive that moves no loop. For a drive
that moves a loop it builds a one-shot schedule under `context.Background()`, calls `PoseAt` on it and
discards it, so a one-off pose needs no new object, and a renderer that wants one pose per frame calls
`Schedule` once. `VerifyLinkage` builds its own schedule and reads every pose and every cell through it,
so the verifier's pose at `s` and `Schedule.PoseAt(s)` are the same transform from the same enclosures,
which is §2.4's one-place rule for loops; a tree linkage's poses are built as before.

### 15.8 Waypoints, held loops and the joint box

**Sub-segments.** A loop's driver may carry `Via` values like any joint (§2.3). Each segment of its
schedule is one **sub-segment**, or two where the driver's value crosses `0` inside it, at the fraction
`s₀ = j/n + t₀/n` with `t₀ = q_j / (q_j − q_{j+1})`, exact when both waypoints are whole turns or both
radians or lengths. On each sub-segment the driver's value keeps one sign, so it reads on one side of the
plane (§15.2) with its own chain from its own near end (§15.3): the end with the smaller `|q|`, the crossing
itself where there is one. A sub-segment whose two waypoints are equal **holds** the driver; a drive whose
every sub-segment holds is a held loop, its links `linkConstant` (§6 step 2), each dependent one point ask
after its approach. A sub-segment holding the driver at `0` reads on a side some moving sub-segment uses.

**A crossing between waypoints stated in mixed terms.** Where one waypoint is in whole turns and the other
in radians, `t₀ = 2π·T/(2π·T − B)` depends on `π` and `s₀` is irrational, so the segment is cut at two
rationals around it instead (`crossingCuts`): `t₀` is read at both ends of `π`'s enclosure, the two
readings are widened outward by the gap between them, and the driver's value at each cut is then proven,
for every `π` in the enclosure, to carry the sign of the waypoint on its side. On §15.10's `−30° → 1 rad`
drive the cuts lie `2e-76` apart. The segment becomes three sub-segments: up to the lower cut, on its waypoint's side, and
on from the upper cut, on the other's, each with its near end at its cut — a nonzero driver value, reached
by an approach from `E0` as for a sub-segment that starts off `0` — and between them the **straddle**,
which has no chain. Anywhere on the straddle the driver's value lies between its values at the two cuts,
on one side of `0` or the other, so each dependent's value lies in the hull of the two neighbours'
approaches and of their points at the cuts: the branch from the zero pose to each cut, certified on that
cut's side. A pose inside the straddle takes that hull as its enclosure, and an interval holding the
straddle is cut at both its ends, the straddle's piece read as that hull, with the neighbours' points at
the cuts as its ends. A pair of cuts whose signs cannot be proven is `ErrUnsupported`.

**A parameter on two sub-segments.** A waypoint or a crossing lies on two sub-segments. A pose there is
read on the one whose near end it is, and otherwise on the first; either reads one exact configuration,
since on one side of `0` the dependents along the branch certified from the zero pose are one continuous
function of the driver's value, and at `0` they are the zero pose. An interval holding the boundary is cut
there for the travel sum of §15.5, each piece read in its own sub-segment's chain.

A loop whose joints the drive does not list, or whose listed joint holds `0`, stands at the zero pose
(§15.1).

`VerifyJointBox` (§14) varies a loop at its driver alone and reads each cell's dependents from this
chain — the centre's point ask and the hull over the cell's driver range — on §16's terms.
`Linkage.Configuration` takes a value per link and names no driver, so it refuses a looped linkage with
`ErrUnsupported`; a looped configuration is a held drive posed by `PoseAt` (§16.1).

### 15.9 Cost

An `Enclose` point ask costs about a millisecond on the crank-rocker; a cell costs its pieces, about
`0.3` ms each, and `sketch`'s tightness rule splits a full turn of the crank-rocker into about `1400`
pieces (`0.4` s). The decomposition asks a certifiable drive as one cell. A pose at depth `d` asks at most
`d` new cells and one point, and the cells of one depth tile the drive once, so a verification that
reaches depth `d` everywhere asks about `d` times the full-range work. Measured: scene 7 at
`WithResolution(Scalar(1.0/256))` evaluates `37` poses in about `0.4` s; scene 9 at the defaults evaluates
`8` poses in about `0.7` s, most of it the decomposition walking the refused cells down to the fold; scene
8 at `WithResolution(Scalar(1.0/256))` evaluates `10` poses in about `0.1` s. The
kernel cost per pose is §10's. A tilted loop costs the scene nothing measurable, but on a tilted
mechanism the links' axis-aligned `Bounds()` boxes overlap along `n`, so the layer exclusion (§5.7)
settles no pair and the pair kernel evaluates every one: scene 10's tilted scene 7 evaluates `113` poses
in about `1.5` s, and its tilted scene 8 `254` poses in about `2` s. `sketch` refuses an angle target beyond `±64` rad, so a crank driven more
than ten turns reads undecided past that; §15.10 does not test it.

### 15.10 Required tests

§11's standard holds: every assertion is on computed geometry through the production path, every bound
is `InDelta` at a stated slack, and each guarding leg is deleted once and seen to fail. Links sit in
separate layers along `Z` so the layer exclusion settles every link-link pair, and the one static body in
each scene reaches past the caps of the link it meets so no pair shares a face plane. Angles below are the
closed forms of the four-bar's two-circle construction, each bracketed to `1e-12` by bisection of the
closed form where it is an inverse.

**Scene 7 — the crank-rocker and a wall (the acceptance target).** Ground pivots `O2 = (0, 0, 0)` and
`O4 = (100, 0, 0)`, axes `+Z`. Link 1, the crank: a prism `x ∈ [0, 30], y ∈ [−4, 4], z ∈ [0, 8]` on a
revolute about `Z` through `O2`. Link 2, the coupler, under the crank: a bar of half-width `4` from
`A = (30, 0)` to `B = (75.714285…, 65.652118…)` — the closed-form `B` at `θ2 = 0` — `z ∈ [10, 18]`, on a
revolute about `Z` through `A`. Link 3, the follower, under the ground: a bar of half-width `4` from `O4`
to `B`, `z ∈ [20, 28]`, on a revolute about `Z` through `O4`. `Close(coupler, follower, B, +Z)`. The
follower's top corner is `B − 4·n4`, `n4 = (−sin θ4, cos θ4)`, at height `y = 70·sin θ4 − 4·cos θ4`,
highest at the follower's minimum `θ4 = 101.5370°` (`θ2 = 38.5727°`): `69.3857`. A wall
`x ∈ [−50, 150], y ∈ [68.5, 78.5], z ∈ [19, 29]` sits in the follower's layer alone. The crank turns
`0° → 90°`.

- `Bars()` lists the ground, crank, coupler and follower bars with `Value` within `1e-9` of `100`, `30`,
  `80` and `70` and `Bound` below `1e-12`; the crank's and the ground's are `Exact` with `Bound` `0`.
- The corner reaches `y = 68.5` at `θ2 = 12.625006°`, `s₁ = 0.140278`, and leaves it at `θ2 =
  66.792358°`, `s₂ = 0.742137`. At `WithResolution(Scalar(1.0/256))` assert: `Status` `Interfering`; the
  first `LinkCollision` is `(follower, wall)` at the grid point `36/256`, the first above `s₁`; every
  collision lies in `(s₁, s₂)` with `Bound` below `Value`; every `IntervalClear` interval ends at or below
  `s₁` or starts at or above `s₂`; the last interval is `IntervalClear`. Measured: `37` poses.
- At `θ2 = 38.671875°`, the grid point `110/256` nearest the follower's minimum, as the end of a drive
  `0° → 38.671875°` evaluated at its endpoints alone: the corner's depth `δ = y − 68.5` is below
  `8·|cos θ4|`, so the overlap is the triangular prism `8·δ²/(2·sin θ4·(−cos θ4))` mm³, asserted within
  `1e-6` with `Bound` below `Value`.
- Every pose's `Values[2]` (the follower, `Links()` index `2`) is within `1e-9` rad of the closed-form
  `θ4(θ2) − θ4(0)`, `θ4(0) = 110.3002°`, and `Values[1]` (the coupler, its turn relative to the crank) of
  `(θ3 − θ2)(θ2) − θ3(0)` with `θ3 = atan2(B_y − A_y, B_x − A_x)`; each dependent's `Bounds` is positive
  and below `1e-9` and the crank's is zero; at `s = 0` both dependents are within `1e-9` of `0`; at `s = 1`
  the follower reads `3.0248°`. This is the leg that goes red when `reading₀` is not subtracted (the
  follower then reads its angle from the ground line).
- The same linkage driven `0° → −90°`, endpoints alone: the follower at `s = 1` reads `146.7235° −
  110.3002° = 36.4233°` within `1e-9` rad. Red when the mirrored scene's signs are dropped (the follower then
  reads `3.0248°`).
- Every joint about `−Z`, the crank driven `0° → −90°`, which is the `+90°` turn: each dependent reads the
  negation of its `+Z` value. Red when the mirrored scene's signs are dropped.
- No pose carries a `(coupler, follower)`, `(crank, coupler)` or `(crank, follower)` row.
- `Schedule(ctx, drive).PoseAt(ctx, s)` equals the report's pose at every evaluated `s`, bit for bit, and
  two schedules agree; `Linkage.PoseAt(drive, Scalar(1.0/3))` equals the schedule's. Eight goroutines
  calling `PoseAt` at interleaved parameters get the same poses as one.
- `examples/` gains `Example_decad_linkageLoop` on this scene, printing `Status`, the first collision's
  bodies and its fraction to three decimals — `0.141` on every platform.

**The pin between samples (the dependent travel term).** Scene 7 without the wall and a `0.8` mm pin in
the follower's layer, `z ∈ [23.6, 24.4]`. The follower's end face sweeps every radius from `70` to
`70.114` mm about `O4`, so a pin on the corner's own path would sit inside the bar for a wide stretch of the
drive; the pin is centred `70.55` mm from `O4`, `0.2°` past the corner's polar angle at `θ2 = 16.875°`,
where only the corner grazes it, from `θ2 = 16.551652°` to `19.24°` (the onset bracketed by bisection of
the separating-axis gap between the follower's outline and the pin's), and again near `60°`. Between the
grid points `θ2 = 11.25°` and `22.5°` of `WithResolution(Scalar(1.0/8))` the corner moves `3.6` mm and
passes the pin. Assert `Suspect`, no `Collision`, the interval `[1/8, 1/4]` `IntervalUndecided`; at
`WithResolution(Scalar(1.0/1024))` `Interfering` with the first `Collision.At` above `16.551652/90` and
within `2/1024` of it. This is the leg that goes red when a dependent joint contributes nothing to `τ`:
the coarse interval then certifies from its ends.

**The turn-back (the two-sided hull).** Scene 7 without the wall, the crank driven `0° → 81.857366°`,
where the follower returns to its starting angle `110.3002°` after dipping to `101.5370°`; the pin centred
on the corner at the follower's minimum, `(89.919184, 69.385713)`. At `WithResolution(Scalar(1))` assert
`Suspect` and the one interval `IntervalUndecided`. Red when `|Δq_k|` is the span of the interval's ends:
the follower's ends agree, the interval certifies, and the report reads `Sound` with two poses.

**Scene 8 — the slider-crank and an end stop.** Crank `x ∈ [0, 30], y ∈ [−3, 3], z ∈ [0, 8]` about `Z`
through the origin; rod `x ∈ [30, 110], y ∈ [−3, 3], z ∈ [10, 18]` under the crank on a revolute through
`A = (30, 0, 0)`; slider block `x ∈ [105, 115], y ∈ [−5, 5], z ∈ [20, 28]` under the ground on a prismatic
along `+X`; `Close(rod, slider, (110, 0, 0), +Z)`. The slider pin is `x(θ) = 30·cos θ + √(6400 − 900·sin² θ)`
and the rod's angle from `+X` is `−asin(30·sin θ / 80)`. A stop `x ∈ [60, 70], y ∈ [−20, 20], z ∈ [19, 29]`
sits in the slider's layer. The crank turns `0° → 90°`.

- `Bars()` lists the crank's and the rod's bars, `30` and `80`, both `Exact`: no bar for `Common` or the
  slide.
- The block's left face `x − 5` reaches `70` at `cos θ = 1/36`, `θ* = 88.408246°`, `s* = 0.982314`. At
  `WithResolution(Scalar(1.0/256))` assert `Interfering`, the first `LinkCollision` on `(slider, stop)` at
  the grid point `252/256`, where `x = 74.901876` and the overlap is `80·(75 − x) = 7.849912` mm³, within
  `1e-6`, with `Bound` below `Value`; every collision past `s*`; every `IntervalClear` interval ends at or
  below `s*`.
- Every pose's `Values[2]` is in millimetres and within `1e-9` of `x(θ) − 110` with `Bounds[2]` below
  `1e-9`; `Values[1]` (the rod's turn from the crank) is within `1e-9` of `−θ − asin(30·sin θ / 80)`. The
  crank driven `0° → −90°` reads the same closed forms at `θ = −90°` on the mirrored side.
- Red when the rail is dropped (the slider pin is free in the plane and `E0` refuses), and when a slide's
  value is read in radians.
- The pin leg for a prismatic dependent: the stop replaced by a `2 × 2 × 2` mm pin at `x ∈ [85, 87]`,
  `y ∈ [−1, 1]`, `z ∈ [23, 25]`, the crank driven `0° → 180°`; the block's left face passes `x = 86` at
  `cos θ = 2781/5460`, `θ = 59.380079°`, `s = 0.329889`, between the grid points `1/4` and `1/2` of
  `WithResolution(Scalar(1.0/4))`, across which the slider moves `24.19` mm. Assert `Suspect` with
  `[1/4, 1/2]` `IntervalUndecided` and no collision, and `Interfering` at `WithResolution(Scalar(1.0/1024))`
  with the first `Collision.At` above the contact onset — the face reaching the pin's far face `x = 87`,
  `cos θ = 2964/5520`, `s = 0.319574` — and within `2/1024` of it. Red when the prismatic dependent's term
  is dropped: the slider's travel is then nothing and `[1/4, 1/2]` certifies.
- The slide as the driver: a slider-crank whose crank stands along `+Y` at the zero pose, `A = (0, 30)`,
  so the slider pin `P = (√5500, 0)` sits off the dead centre and the rod's bar is reached from an
  irrational pin (`Approximate`, within `1e-12` of `80`). The slide driven `0 → 5` mm and `0 → −5` mm: at
  `s = 0`, `1/2` and `1` the crank reads `φ − 90°` within `1e-9`, `φ` the root near `90°` of
  `30·cos φ + √(6400 − 900·sin² φ) = √5500 + d`, and the rod its closed-form turn from the crank; the slide
  carries its stated value with a zero `Bound`. Red when the half-turned side is dropped: the backward slide
  is then asked as a forward one.

**Scene 9 — the fold.** The non-Grashof four-bar: ground `100`, crank `50`, coupler `60`, follower `50`,
in scene 7's layout with `A = (50, 0)` and the closed-form `B` at `θ2 = 0` (`θ4 = 106.2602°`), no static
body. The loop folds at `cos θ2 = 0.04`, `θ2 = 87.707557°`.

- Driven `0° → 90°` at the defaults: `Status` `Suspect`; no `Collision`; every pose's `At` below
  `s_fold = 0.974528`; the last pose's `At` within one verdict floor below it — measured `997/1024 =
  0.973633`; the last interval is `IntervalUndecided` from the last pose to `1`, with no pose at `1`; the
  report's one diagnostic is that interval's `DiagMotionUndecidedInterval`, with `At` its `From` and a
  `Message` naming the loop and `sketch`'s refusal; every earlier interval is `IntervalClear` (the layer
  exclusion settles every pair, so the certificate is about the loop alone). Red when a pose past the fold
  is built from certified values (a pose at `998/1024` is then published).
- `Schedule.PoseAt(ctx, Scalar(1))` on that drive is `ErrUnsupported` and `errors.Is` finds
  `sketch.ErrNotCertified`, the cause of the cell holding the fold; `Linkage.PoseAt` agrees.
- The flat four-bar — ground `100`, crank `30`, coupler `40`, follower `30`, `A = (30, 0)` and
  `B = (70, 0)`, so coupler and follower lie along the ground line at the zero pose and the loop's
  Jacobian is singular there: `E0` refuses, and `VerifyLinkage`, `Schedule` and `PoseAt` return
  `ErrUnsupported` wrapping `sketch.ErrUnderconstrained` (one degree of freedom remains at the singular
  configuration).
- Driven `0° → 100° → 0°` at the defaults: the crank enters the fold at `s = 87.707557/200 = 0.438538` and
  leaves it at `0.561462`. Assert `Suspect`; poses at `0` and `1` both read every dependent `Value` within
  `1e-9` of `0`; no pose lies in `[0.438538, 0.561462]`; exactly one interval is not `IntervalClear`, an
  `IntervalUndecided` from at or below `0.438538` to at or above `0.561462`, merged across the unbuildable
  poses between.

**Scene 10 — the tilted loop.** Scenes 7 and 8 carried by the rotation that takes `X` to `(1, −1, 0)/√2`
and `Z` to `(1, 1, 1)/√3` (`r3.FromFrame` of the frame `u = (1, −1, 0)`, `v = (1, 1, −2)`): each body built
as its scene builds it and placed by the rotation (`Body.Placed`), each pin the rotation of the scene's,
every revolute and the closure about `(1, 1, 1)`, scene 8's slide along `(1, −1, 0)`, exactly
perpendicular to it. No pin's plane position is a float, so every fixed point is a box.

- Scene 7 tilted, its wall carried with it, at `WithResolution(Scalar(1.0/256))`: the bars within `1e-9`
  of `100`, `30`, `80` and `70` with `Bound` below `1e-12`; `Interfering`; the first `LinkCollision` is
  `(follower, wall)` at the untilted report's own first `At`, `36/256`; every collision lies in
  `(s₁, s₂)` with `Bound` below `Value`, every `IntervalClear` interval outside it; every pose's dependent
  values within `1e-9` of scene 7's closed forms, each `Bounds` positive and below `1e-9`. Red when the
  fixed boxes are dropped: each of `Common`'s pins is then fixed at its float plane position, which `E0`
  reports as its box and which does not hold the pin's enclosure, so the falsifier refuses the loop.
- Scene 8 tilted, its stop carried with it, at `WithResolution(Scalar(1.0/256))`: `Interfering`, the first
  `LinkCollision` `(slider, stop)` at the untilted report's own first `At`, `252/256`, its volume within
  `1e-6` of `80·(75 − x) = 7.849912` mm³ with `Bound` below `Value`; every collision past `s*`; every
  pose's slide value within `1e-9` of `x(θ) − 110` mm and the rod's turn of its closed form. Dropping the
  fixed boxes here is not a leg the fixture fails: the crank's ground pin is the origin, a float, and the
  rail's two points are not pins, so the falsifier has no pin to refuse. The boxes are what makes the
  scene the document's; the falsifier only refuses.

**Drives over a loop.** Scene 7's crank-rocker without a wall; every assertion on a dependent value is the
closed form at the crank angle the drive states, within `1e-9`, at every pose the check evaluates and at
every `k/16` of a schedule, with each non-dyadic boundary added.

- **Waypoints.** The crank `0° → 60° → 20°`: the second segment's value falls toward `0`, so its chain
  starts at `s = 1`; the waypoint `1/2` and `1/3`, `2/3` read the closed form; the report is `Sound`. Red
  when a sub-segment's chain starts at its first waypoint instead of its near end: the falling segment's
  cells are asked downward, refused, and its poses are unbuildable.
- **Out and back.** The crank `0° → 30° → 0°` and a `0.8` mm pin in the follower's layer centred on the
  follower's top corner at `θ2 = 30°`. Endpoints alone: both read the zero pose, and the one interval,
  holding the waypoint, is `IntervalUndecided` — the follower's travel is summed over its two pieces, about
  twice the corner's `10.2` mm arc, against endpoint gaps summing to about `19.2` mm. At
  `WithResolution(Scalar(1.0/2))` the pin is struck at `s = 1/2`. Red when one piece's term stands for the
  sum: the interval certifies and the report reads `Sound` past a collision.
- **Crossing zero.** The crank `−30° → 60°`, across `0` at the non-dyadic `s = 1/3`: the stretch below
  reads on the mirrored scene, the one above on the scene's own side, each chain starting at the crossing;
  `s = 0` reads the follower at `14.583238°` and `s = 1/3` exactly `0`. The slide-driven slider-crank of
  scene 8's last leg driven `−5 → 5` mm crosses at `s = 1/2`, the backward stretch on the half-turned side.
  Red, for each, when every stretch reads on the scene's own side.
- **Crossing zero in mixed terms.** The crank `−30° → 1 rad`, across `0` at the irrational
  `s₀ = (π/6)/(1 + π/6)`, at `WithResolution(Scalar(1.0/16))`: `Sound`, every pose and every `k/16` of a
  schedule and the float nearest `s₀` read the closed form, `s = 0` the follower at `14.583238°`, the float
  nearest `s₀` within `1e-12` of the zero pose. Red when the stretch below `s₀` reads on the scene's own
  side: `s = 0` then reads the follower's turn at `+30°`, `−8.33°`.
- **A held driver.** The crank held at `30°` while a separate arm off the loop turns: every pose reads the
  closed form at `30°` and the loop's links stand at one placement. The crank `0° → 30° → 30° → 0°` reads
  `30°` over its middle segment. Red when a held stretch's cells are asked: their range is empty, `sketch`
  refuses them, and the stretch's poses are unbuildable.
- **Limits on a dependent.** Under the crank `0° → 90°` the follower turns over `[−8.763°, 3.025°]`:
  limits `[−5°, 5°]` refuse the drive with `ErrDegenerate` naming link `2`, `[−10°, 5°]` admit it. Limits
  `[−10°, −1°]`, which exclude the zero pose, admit the crank `20° → 50°`. Red when the hull check is
  deleted (the first drive is admitted), and when a dependent is held to the `0` of an unlisted joint (the
  last is refused).

**Standing tests.** Errors, one subtest per row of §15.1's and §15.6's tables: a nil link, the ground, one
link twice, a link of another linkage, a non-finite center, a zero axis, a closure axis tilted by `1e-9`
from the loop's revolutes about `Z`, a loop revolute about `(0, 1e-12, 1)`, a slide along the closure
axis, a slide along `(1, 0, 1e-12)`, a slide under a link that is not `Common`, two slides on one loop, a
second closure on the coupler, a closure coincident with a pin in the plane, a drive listing crank and
follower, a drive listing the coupler, a schedule pose outside `[0, 1]`, `Configuration` on scene 7's
linkage; and a drive listing no loop
joint standing every link at the zero pose with zero `Bounds`. Non-mutation of the document and
determinism of two reports as motion §9 test 7; cancellation at every depth of the check, inside an
`Enclose` call among them, returns `ctx.Err()` and no report. Internal tests in
`linkage_loop_internal_test.go` pin `MotionFrame.AtRange` on a synthetic interval `[θ − w, θ + w]`: every
entry of its rotation contains the rotation at both ends, and its sine entry's width is at least
`2·w·cos θ` minus the point enclosure's own width — red when `AtRange` drops the widening and reads one
end alone; pin the dependent `|Δq_k|` of §15.5 on hand-made `A`, `B`, `H` for the monotone and the
turn-back case — red when it is the span of the ends; pin the canonical chain on a four-level grid: for a
drive from `10°`, the point at `5/8` is continued from the cell `[1/2, 5/8]`, that from the point at `1/2`,
that from the cell `[0, 1/2]`, that from the point at `0`, that from the approach, by asserting the cache
holds exactly those keys after one point ask; pin the zero-pose falsifier, on scene 7's loop and on its
tilt about `(1, 1, 1)`: each of the four pins' recorded positions moved by `1e-6` mm after the scene is
built is refused, the record as built admitted — red, on both, when the pin-in-box check is deleted; pin
the straddle on the crank `−30° → 1 rad`: three sub-segments that tile the segment, the mirrored side
below and the scene's own above, each neighbour's chain starting at its cut, the cuts under `1e-60`
apart; a pose inside the straddle holds both neighbours' values at their cuts and lies within `1e-12` of
`0`; the pieces of `[1/4, 1/2]` tile it with the straddle in the middle — red when the straddle is not a
sub-segment, which leaves a gap no fixture's pose or travel term is wide enough to show; pin the interval gate: a refused cell put in place of a certified one
leaves the interval refused with `sketch`'s cause — red when the gate's refusal is deleted, which no
public fixture shows, since a real chain refuses the point at a refused cell's end too; and pin the scene's
frame: for each of the six axis senses and both scene sides, `U` and `V` are unit coordinate axes bit for
bit, `U × V` is the closure axis times the side, and a pin's plane position is the caller's coordinates bit
for bit, its exact enclosure that one value; for a loop with a slide, for each of the four slide
directions perpendicular to the axis and on the half-turned side too, `U` is the slide's sense, negated on
the half-turned side; and for the tilted axes `(1, 1, 1)` and `(−0.3, 2, 0.7)`, with and without a slide,
on every side, `u ⟂ v`, `u × v` along the side's normal, `u` along the slide's own sense, and a pin's two
enclosures each below `1e-12` wide with `x² + y²` holding the pin's exact squared distance from the axis
— red when the coordinates are not divided by `|u|` and `|v|`. A slide's reading
sense `s_k` against `u` is not a leg any fixture can fail: `u` is the slide's own sense on every side a
dependent slide is read on, since a loop holds one slide and only a slide driver half-turns the scene; nor
is the zero whole-turn count of a horizontal distance, whose continued readings agree to the boxes' width.
The reach guard of §15.5 is not a leg any fixture can fail: the decomposition reads the reach from
the same canonical asks every later pose and interval reads.

`.github/test-shards.txt` and `.github/test-shards-apitest.txt` are updated for every test and example,
and `go test . ./apitest/ -run '^TestCI'` is run before the push.

### 15.11 Increments

| PR | lands | still refused or `Suspect` after it |
|---|---|---|
| L1 (`linkage_loop.go`: `Close`, `LinkageLoop`, the scene, the chain and `Schedule`; `motionbound.MotionFrame.AtRange`; the engine's unbuildable poses and interval gate; `go.mod` and `_gallery/go.mod` pinned to sketch `821a4460` (`add interval targets and fixed boxes to Enclose (#155)`); this section) | §15.1's vocabulary and admission with every revolute loop about a coordinate axis, §15.2's scene on either side with the bars as target ranges and the zero-pose falsifier, §15.3's chain with whole-turn counts, §15.4's pose with `LinkagePose.Bounds`, §15.5's travel term, reach and reach guard, §15.6's refusals with unbuildable poses and merged undecided intervals, §15.7's `Schedule` and `PoseAt`, `Configuration` refusing a loop; scene 7 with its pin and turn-back legs, scene 9's fold and flat four-bar, the standing tests, the example; a `docs/layout.md` row for `linkage_loop.go` | a prismatic joint on a loop; a loop driver with `Via`, held off `0` or crossing `0`; limits on a dependent; a loop about a tilted axis |
| L2 (`linkage_loop.go`) | the prismatic loop joint: the rail as a fixed line along `u` (`u` the slide's sense of its coordinate axis, `v = n × u`) and `NewPointOnLine(P, rail)`, the driving or driven `NewHorizontalDistance(P₀, P)` from the slide's fixed zero-pose point, the half-turned scene side (`u` and `v` both negated) for a slide driven backward, no bar for the slide or for `Common`; `Close`'s prismatic rows (a second prismatic on the loop, one whose parent is not `Common`, a slide not exactly along a coordinate axis or not exactly perpendicular to the closure axis); scene 8 with its pin leg, the slide as the driver | a loop driver with `Via`, held off `0` or crossing `0`; limits on a dependent; a tilted loop |
| L3 (`linkage_loop.go`) | `Via` on a loop's driver: sub-segments at waypoints and zero crossings, each with its own chain and scene side, the near-end clamp of §15.3, the pose's sub-segment at a boundary, the cuts of §15.5, held drivers and held stretches; limits on a dependent, checked against its whole-drive hull after the decomposition; scene 9's out-and-back drive and the drives over a loop of §15.10 | a loop driver crossing `0` between waypoints stated in mixed terms; a tilted loop |
| L4 (`linkage_loop.go`) | a loop about any axis and a slide along any direction perpendicular to it: §15.2's exact frame with each pin's plane position enclosed, each fixed point stated with `WithFixedBox` where its enclosure is not one float, each free point seeded at `r3.Frame.ToLocal`'s float, the bars' squared lengths off `n`, the falsifier refusing a pin whose enclosure is not proven inside its box; §15.1's coordinate-axis rows gone; a driver crossing `0` between waypoints stated in mixed terms, cut at two rationals around the crossing with the straddle between them (§15.8); scene 10 and the mixed-terms crossing drive | — |

L1 is the end-to-end instance: the real four-bar, the real `Enclose`, the real kernel, one report, with
scene 7's closed-form onset as its acceptance. The `_gallery` linkage clip of a looped scene is a separate
`_gallery` change after L1.

### 15.12 Settled points, and what sketch #155 supplies

A loop is a tree plus a revolute closure about any axis, holding at most one prismatic joint, a child of
the common link that rides its next pin along a fixed rail perpendicular to the axis; the drive names the
driver, whose parent is the loop's common link; every other loop joint is dependent and its value is read
from `sketch`'s certified enclosure, never from a float solve (§15.1, §15.4). The public type is
`LinkageLoop`, since `Loop` is the topology's. The scene is built in an exact orthonormal frame from the
document's own pins, every pin's plane position enclosed exactly, every fixed point stated as the box
around its enclosure and every bar as the interval around its exact length, so every claim is about the
document's loop; the document's pins falsify `E0` and never admit it (§15.2). Every enclosure is
continued from a canonical predecessor rooted at the zero pose, carries a whole-turn count per dependent,
and poses and intervals share one cached chain (§15.3, §15.7). A dependent's travel is the two-sided hull
bound, which doubles where the joint turns back, and its reach is the decomposition's hull, held against
every later read (§15.5). A refusal is a finding, except at the zero pose, where it is an error; an
unbuildable pose is not evaluated and its interval is undecided (§15.6). A drive over a loop is cut into
sub-segments at its waypoints and its zero crossings, each with its own chain and side, an irrational
crossing at two rational cuts around a straddle read from both sides' branches, and a dependent with
limits is held to its whole-drive hull (§15.5, §15.8). The joint box over a loop reads the same chain per
cell (§16).

**What sketch #155 supplies.** `.tmp/decad-handoff-interval-targets.md` in the `sketch` repository asked
for two additions to `Enclose`, and both landed in `821a4460`: `WithTargetRange`, a dimension's target as
an interval with every claim holding for every target in it, which §15.2 states every bar with and which
makes the zero-pose falsifier valid; and `WithFixedBox`, a fixed point as a box, which §15.2 states every
fixed point of a tilted loop with, since its plane position is then irrational. Nothing in this section
waits on `sketch`.

## 16. The joint box over a closed loop

`Document.VerifyJointBox` takes a looped linkage (§15) on this section's terms: the box varies a loop at
its **driver** alone, every other joint of the loop follows it, and each cell reads its dependents' values
from `sketch`'s certified enclosures exactly as a drive's interval does (§15.3–§15.5) — the enclosure of
each dependent at the cell's centre enters `η`, and its hull over the cell's driver range enters `τ_half`.
Everything in §14 and §15 holds unless this section says otherwise. `linkage_box.go` runs the subdivision
and `linkage_loop.go` answers its asks; no new file is added.

### 16.1 Which joints a box varies

A `JointBox` lists at most one joint of each loop, as a `Drive` does (§15.1): that joint is the loop's
driver for the box, its parent MUST be `Common` (`ErrUnsupported` otherwise), and every other joint of the
loop is dependent and is not listed. A box listing two joints of one loop is `ErrDegenerate`, since the
second value cannot be stated. The driver's range stands for the loop:

| The driver's range | The loop |
|---|---|
| unlisted, or `Min == Max == 0` | stands at the zero pose: its links are the tree's held links (§6 step 2) and no scene is built |
| `Min == Max ≠ 0` | is **held**: one scene, `E0`, the approach and one point ask (§15.8); its links are constant placements, and its dependents contribute nothing to any `τ_half` |
| `Min < Max` | is the box's **loop axis**, one varying axis |

A dependent is never a box axis, so `n` counts a loop once. A cell publishes its dependents' values
(§16.4); the caller never states them.

`Linkage.Configuration` refuses a looped linkage with `ErrUnsupported`: it takes one value per link and
names no driver, and a four-bar is driven at its crank or at its follower. A configuration of a looped
linkage is stated as a drive whose every sweep holds — the driver and each moved tree joint listed with
`From == To` — and posed by `Linkage.PoseAt` or `Schedule.PoseAt` (§15.7, §15.8), which read the dependents
at that one driver value. A cell's `Center` is built by the same reading (§16.3), so `PoseAt` of the held
drive at a centre's stated values and that `Center` agree to the enclosures' widths, each enclosing the one
exact configuration; they are not bit-identical, because the held drive's point ask is continued from its
approach and the centre's from its cell (§16.2).

### 16.2 The loop axis is a drive

The loop axis is the one-segment drive `Min → Max` in the fraction `f ∈ [0, 1]` of the driver's range, and
§15.2, §15.3 and §15.8 apply to it verbatim, about any axis the loop closes about: `driverSubs` cuts it at
a crossing of `0` into two sub-segments, each read on its own scene side with its own chain from the
crossing, and at a crossing between ends stated in mixed terms into three, the straddle between two
rational cuts read from both neighbours' branches; the scene is built per side; `E0` and the zero-pose
falsifier run; and `prepareLoops` decomposes `[0, 1]` down to the verdict floor before any cell, so each
dependent's reach `m_j` and its limits (§15.5) are read first.

A cell's extent along the loop axis is a dyadic interval `[a, b]` of `f` and its centre `m = (a + b)/2` a
dyadic fraction, because §14.4 halves one axis at a time from the root. The cell tree's loop-axis intervals
are therefore §6's grid, and §15.3's canonical rule names every ask's predecessor — a cell `[a, b]` is
continued from the point at its chain-start, a point at depth `d` from the depth-`d` cell ending at it —
whatever order the subdivision asks them in. A cell asks for:

- its **centre**: the point ask at `m`, on the sub-segment `subAt(m)` picks (§15.8), giving each
  dependent's enclosure `M_j = [m_lo, m_hi]` — inside a straddle, the hull of its asks (§15.8);
- its **hull**: `intervalSpans(a, b)` — the point asks at `a` and `b` and the cell ask between them, per
  sub-segment piece the interval is cut into (§15.5) — and over the pieces the hull `H_j = [h_lo, h_hi]` of
  each dependent's value.

Cells that share a loop-axis interval share every ask: §15.7's cache is keyed by the sub-segment, the
kind and the ends of the fraction, not by the cell, so two cells that differ along a tree axis alone cost
one chain between them. The number of distinct asks is the number of distinct loop-axis intervals and
points the subdivision reaches — about what a `VerifyLinkage` over `Min → Max` asks at the same floor. The
subdivision is deterministic as §14.4 states and each ask as §15.7 states, so two calls on the same inputs
return reports equal in every field.

### 16.3 What a cell proves over a loop

**The centre.** The centre's pose is §15.4's: each dependent at the float midpoint label of `M_j`, posed
by `posesOf`; its ideal pose composed with `MotionFrame.AtRange(m_lo, m_hi)`, so `η` charges the whole
enclosure; every stated joint at its exact centre value. `JointConfiguration` carries the half-widths:

```go
type JointConfiguration struct {
    Values []units.Value
    Bounds []units.Value  // each value's proven half-width, in its Kind; zero for a stated joint (§15.4)
    Poses  []r3.Transform
}
```

`Bounds` is zero for every joint of a tree linkage, so `Configuration` and a tree box's `Center` are
unchanged but for the field.

**The dependent's term in `τ_half`.** §14.3 moves one joint at a time from the centre `m` to a
configuration `q` of the cell. A stated joint `i` changes by at most half its span, which is `τ_half`'s
`½·w_i·span_i`. A dependent `j` changes from its exact centre value `c_j` to its exact value at `q`: both
lie in `H_j`, and `c_j` lies in `M_j` too, because the centre's point ask and the cell's asks enclose the
one solution continued from the zero pose (§15.3). So

```text
δ_j(C) = max( h_hi − m_lo,  m_hi − h_lo )      (exact rationals)
```

bounds `|θ_j(q) − c_j|` for every `q` in `C`, and the dependent's term is `w_j·δ_j(C)` — the full one-sided
distance, not a half. The centre's value sits wherever the loop puts it in `H_j`: at an end of the hull
where the dependent is monotone over the cell, inside it where the dependent turns back, and `δ_j` is a
bound in both cases where half the hull's width is not. Hence

```text
τ_half(C) = ½·Σ_{stated i} w_i·span_i(C)  +  Σ_{dependent j} w_j·δ_j(C)
```

summed as §14.3 sums, over the joints below the pair's lowest common ancestor on each branch, with
`w_j = ρ_{jk}` for a revolute dependent and `1` for a dependent slide. A held loop's dependents have
`δ_j = 0` (§16.1). The same `τ_half` enters the blocked certificate's allowance (§14.3): along the straight
joint-space segment from `m` to `q` each dependent changes by at most `δ_j`, so the volume's Lipschitz bound
holds with the same per-body travel. `τ_half` is the one quantity that changes; the centre certificate,
the blocked certificate and the cell's `Clearance` read as §14.3 writes them.

**The loop gate.** A cell whose hull ask `sketch` refused — a fold inside its loop-axis range, the piece
budget, a reading past a dependent's reach (§15.5) — has no `δ_j`, so no pair of it has a `τ_half`, and it
is neither `CellClear` nor `CellBlocked` whatever its pairs prove, even when every pair was settled before
any cell: the claim that the cell's configurations lie on the zero-pose branch is `sketch`'s, and it was
refused. Such a cell is `CellColliding` when its centre collides and `CellUndecided` otherwise, with the
loop's finding (§16.4). This is §15.5's interval gate over a cell.

**An unbuildable centre.** A cell whose centre's point ask is refused has no pose: it is `CellUndecided`,
its `Center` is the zero `JointConfiguration`, it carries no row, and its finding names the loop and
`sketch`'s cause. It counts in `CellsEvaluated` and against the budget, as the split that produced it
spent them.

**Reach, exclusions, limits.** A dependent's reach `m_j` is the decomposition's hull (§15.5) and enters
the balls, the swept-box reach and the layer exclusion as §14.3 states them for a held or varying joint;
every loop joint keeps `a·x` for `a = n`, so a stacked planar loop's link-link pairs are layer-separated. A
dependent with `WithJointLimits` is held to the decomposition's hull over the box's loop axis
(`ErrDegenerate`, after the decomposition). A cell whose hull reaches past `m_j` is refused by the reach
guard, and the gate makes it undecided.

### 16.4 The procedure over a loop

§14.4 runs with these additions:

- **Step 2** builds each loop's scenes, asks `E0` and decomposes the loop axis to the verdict floor
  (`prepareLoops`, §15.7) before the bounds are read, as `VerifyLinkage` does.
- **Step 4** classifies through the gate: a refused hull or an unbuildable centre is never clear or
  blocked.
- **Step 5's axis.** Wherever §14.4 ranks the varying axes by a joint's term — the split axis of an
  undecided or colliding cell, step 6's reading axis — a dependent's term is charged to its loop's driver
  axis as `2·w_j·δ_j`, twice its share of `τ_half`, which is how a stated joint's `w_i·span_i` counts;
  halving the driver's range is what shrinks the hull. A pair's share of an axis is the sum of its terms
  charged there, so the driver of a crank on a pair's path carries its own term and its dependents'. A
  gated cell — its hull refused or its centre unbuildable, colliding at its centre or not — has no term
  to rank and splits along the refusing loop's axis, the one axis a split changes, while that axis is
  wider than the verdict floor; it is **stuck**, published as it stands, when its loop-axis range overlaps
  no stretch the decomposition certified by more than a point, because every centre a split could place
  inside it was refused at the floor already. A cell whose range overlaps a certified stretch is split, and
  a child whose range lies inside the certified set has a buildable centre.
- **Step 8** publishes, per dependent `j` of a cell, `Cell.Min[j]` and `Cell.Max[j]` as the floats of
  `H_j`'s ends rounded outward, in the joint's `Kind` — the proven enclosure of the dependent over the cell,
  `M_j` for a held loop — or the zero `units.Value` when the hull was refused, and for every dependent of
  every loop when the centre is unbuildable, since no hull is read then; `Center.Values[j]` and `Center.Bounds[j]` as §15.4
  labels them; and an undecided cell's `DiagMotionUndecidedInterval` with `Cell` set and, where the gate
  held the cell, a `Message` naming the loop by its two closed links and carrying `sketch`'s cause, as
  §15.6's interval finding does. No new `DiagnosticCode` is added.

### 16.5 What is never claimed

§14.5 and §15.6 hold, and in addition: nothing about a cell whose hull or centre `sketch` refused, beyond
a collision proven at its centre; nothing about a dependent's value outside the hull its cell publishes;
nothing about a branch of the loop other than the zero pose's (§15.3).

### 16.6 Errors

§14.6 holds, with these rows added, each before `ctx` is read and each as §15.6 states it for a drive:

| Condition | Error |
|---|---|
| a box listing two joints of one loop | `ErrDegenerate` |
| a box listing a loop joint whose parent is not `Common` | `ErrUnsupported` |
| `E0` refused, or a document pin outside `E0`'s box (§15.2) | `ErrUnsupported` |
| a dependent's decomposition hull not proven inside its limits | `ErrDegenerate`, after the decomposition |
| `Linkage.Configuration` on a looped linkage (§16.1) | `ErrUnsupported` |

A refused cell or centre is a finding, never an error, as §15.6 says of every ask after validation.

### 16.7 Cost

Each centre costs §14.7's kernel pose plus one point ask (about a millisecond on the crank-rocker, §15.9),
and each distinct loop-axis interval one cell ask whose pieces serve every cell over it. At a floor of
`1/2^d` along the loop axis the asks number about `2^(d+1)` cells and points — the chain a `VerifyLinkage`
over `Min → Max` builds — so the enclosure work is a few hundred milliseconds where the kernel work is
seconds; the root's cell ask, over the whole range, costs the most pieces (a quarter turn of the
crank-rocker: a few hundred). A stuck cell costs its one refused point ask, which the cache answers from
its predecessor's refusal. Measured: scene 11 at `WithResolution(Scalar(1.0/16))` evaluates `445` centres
in about `2.3` s, `13` s under the race detector; the one-axis loop box `107` centres in about `2` s, `6` s
under the race detector; the fold box `21` centres in about `2` s, most of it the decomposition walking the
refused cells down to the fold. Under the race detector the clear box's reading and its two margins take
about `15` s together, the blocked box `3` s, the held loop `2` s and the turn-back cell `1` s. The test file
records the counts.

### 16.8 Required tests

§11's standard holds: every assertion is on computed geometry through the production path, every bound is
`InDelta` at a stated slack, and each guarding leg is deleted once and seen to fail. The four-bar is
scene 7's (ground `100`, crank `30`, coupler `80`, follower `70`, pivots `O2 = (0, 0)` and `O4 = (100, 0)`,
links in layers along `Z`), with `θ4(θ2)` the two-circle closed form and the follower's top corner at

```text
y_c(θ2) = 70·sin θ4 − 4·cos θ4,    θ4 = θ4(θ2)
```

which rises from `67.0399` at `θ2 = 0` to `69.3857` at the follower's extreme `θ2* = 38.5727°`
(`θ4 = 101.5370°`) and falls to `65.8629` at `90°`. The follower's rest box is `x ∈ [71.9627, 103.7516]`,
`y ∈ [−1.3878, 67.0399]`, and its ball reading about `O4`'s axis — the farthest corner, `(71.9627,
67.0399)` — is `ρ = 72.6666`; its area is `2368` mm². Every non-dyadic value below is bracketed to `1e-12`
by bisection of the closed form.

**Scene 11 — the crank-rocker and the gate (the acceptance target).** Scene 7's four-bar with no static
body, and link 4, the **gate**: a prism `x ∈ [−50, 150], y ∈ [72, 82], z ∈ [19, 29]` on a prismatic joint
under the ground along `(0, −1, 0)`, so its value `d` lowers it. The box is the crank over `[0°, 90°]` and
the gate over `[0, 10]` mm. The gate's underside is `y = 72 − d`, so the colliding region is
`{d > d*(θ2)}` with `d*(θ2) = 72 − y_c(θ2)`: `4.9601` at `0°`, `2.6143` at `θ2*`, `6.1371` at `90°` — a
boundary curve that is monotone in `d` and turns back in the crank. The `(crank, gate)` and `(coupler,
gate)` pairs are settled by the layer exclusion along `Z` (`z`-gaps `11` and `1`), the loop's own pairs
as in scene 7, so each centre evaluates `(follower, gate)` alone; the follower's reach `ρ·0.1529` — its
dependent turns over `[−8.7632°, 3.0248°]` — leaves its swept box meeting the gate's. At
`WithResolution(Scalar(1.0/16))` assert:

- `Status` is `Interfering`; the leaves tile the box; every leaf's `Center` is its cell's midpoint along
  the crank and the gate; `Center.Values[2]` (the follower) is within `1e-9` rad of `θ4(θ2_c) − θ4(0)`
  with `Bounds[2]` positive and below `1e-9`, and `Bounds[0]`, `Bounds[3]` zero; `Cell.Min[2]` and
  `Cell.Max[2]` enclose the closed form's extremes over the cell's crank range (at its ends, or at `θ2*`
  where the cell holds it), and their width is at most `1.05` times the true variation plus `1e-6` rad —
  `sketch`'s piece slack is `5%` of the hull plus `1e-7` of the coordinate scale. Measured, the widest is
  `1.0052` times the true variation, `1.2e-4` rad over it.
- Every `CellClear` leaf has `y_c(θ̂) < 72 − d_hi`, with `θ̂` the crank value of the cell nearest `θ2*` —
  `y_c`'s maximum over the cell — and `Clearance.Value` at or below `72 − d_hi − y_c(θ̂)`, the true minimum
  gap over the cell. This is the leg that goes red when the dependent's term is dropped from `τ_half`.
- Every `CellBlocked` leaf has `y_c(θ̌) > 72 − d_lo`, with `θ̌` the end of its crank range farther from
  `θ2*`; none blocks at this floor, since the gate's own allowance `½·Δd·8200 ≈ 2560` mm³ exceeds every
  overlap. Every `CellColliding` leaf has `y_c(θ_c) > 72 − d_c` at its centre; every undecided and colliding
  leaf is at the floor along both axes; and an undecided leaf's centre gap `72 − d_c − y_c(θ_c)` is at
  most `ρ·(1.1·v + 1e-6) + Δd/2`, with `v` the largest `|θ4(θ) − θ4(θ_c)|` over the cell's crank range:
  `δ` is at most `v` plus the slack of a hull `5%` wider than the variation, which is at most `2·v`.
- Every `JointBoxCollision` has `y_c > 72 − d` at its configuration; where the corner's depth
  `δ = y_c − (72 − d)` is below `8·|cos θ4|`, so that only the corner has crossed, its `Volume` is the
  triangular prism `8·δ²/(2·sin θ4·(−cos θ4))` mm³ within `1e-6`, `Bound` below `Value`; at least one
  collision is that shallow.
- `CellsEvaluated` is below the default budget and no `DiagJointBoxBudgetExhausted` is raised; the test
  file records the count: `445` centres into `223` leaves, `24` clear, `168` colliding, `31` undecided.
- `examples/` gains `Example_decad_jointBoxLoop` on this scene at `WithResolution(Scalar(1.0/16))`,
  printing `Status`, the first collision's bodies and its crank angle and gate value to two decimals; the
  centres are dyadic, so the printed values hold on every platform.

**The clear box and its reading.** Scene 11 with the gate over `[0, 2]` mm: `d*` is at least `2.6143`, so
no configuration collides, and the minimum gap over the box is `72 − 2 − 69.3857 = 0.6143` mm at
`(θ2*, 2)` — interior along the crank, at the gate's end. The minimum is flat along the crank, but the gap
falls linearly along the gate, so the reading meets the default tolerance by refining the gate axis. At the
defaults assert `Sound`; every leaf `CellClear` with `Clearance.Value` at or below
`72 − d_hi − y_c(θ̂)`; `Clearance` enclosing `0.6143` with `ToleranceSatisfied`; `ReadingResolution`
`1/16384`; some leaf narrower than `1/1024` along one axis; `CellsEvaluated` below the budget (measured:
`305` centres into `153` leaves, the narrowest `1/2048` of a range). At `WithMotionTolerance(Scalar(0.5))`,
where only a margin's own refinement decides it, `WithMinClearance` `0.5` mm reads `AssessmentMet` (`83`
centres) and `0.7` mm `AssessmentViolated` (`103` centres) with a `DiagMotionClearanceViolated` whose
`Cell` is set. Red when the dependent's term is dropped from `τ_half` (a leaf's bound exceeds the true gap
at its worst configuration), when the reading stops at the verdict floor (no leaf is narrower than
`1/1024`; the gate is met there at `357` centres), and when the margin's refinement is dropped (both
margins read `AssessmentUndecided`).

**The one-axis loop box.** Scene 7's linkage and wall (`y ∈ [68.5, 78.5]`), the crank alone over
`[0°, 90°]`, at `WithResolution(Scalar(1.0/64))`. The corner is inside the wall for `θ2 ∈ (12.6250°,
66.7924°)`, the fractions `s₁ = 0.140278` and `s₂ = 0.742137`. Assert `Interfering`; every `CellClear`
leaf's range lies inside `[0, s₁]` or `[s₂, 1]`; every `CellColliding` leaf's centre lies inside
`(s₁, s₂)`; every `CellBlocked` leaf's range lies inside `(s₁, s₂)` — none blocks at this floor, since the
corner's overlap is at most about `16` mm³ against the follower's area, so blocking needs `τ_half` under
about `7e-3` mm, cells near `1/5000` of the range; the leaves holding `s₁` and `s₂` are undecided or
colliding; every undecided leaf is at the floor; every collision's `Volume` is the triangular prism above
within `1e-6` where the corner's depth is below `8·|cos θ4|`; `Center.Values[2]` is within `1e-9` rad of
the closed form with `Bounds[2]` positive and below `1e-9`. This is the leg that goes red when the
dependent's term is dropped from the blocked allowance: `τ_half` of `(follower, wall)` is then `0`, every
colliding centre blocks, and the leaf holding `s₁` reads `CellBlocked` though its lower part is clear. Measured: `107` centres into `54` leaves.

**The turn-back cell.** Scene 7 without the wall, the crank over `[0°, 81.857366°]` — the follower returns
to `θ4(0)` at the range's end (§15.10) — and a `0.8` mm pin in the follower's layer (`z ∈ [23.6, 24.4]`)
inside the follower's zero-pose bar, centred `60` mm along its axis from `O4` and `3.5` mm off it along
`n4 = (−sin θ4, cos θ4)`, so the pin is struck at both ends of the range. At `WithResolution(Scalar(1))`,
the root alone: its centre `40.9287°` turns the follower `−8.7314°` away from the pin, and the bar's near
long edge stands `8.0955` mm from the pin's nearest corner, the gap row's value within `1e-9`; the
follower's hull over the range is `[−8.7632°, 0]` from `θ4(0)`, so `δ = 8.7314° = 0.15240` rad and the
pair's `τ_half` is about `11.1` mm. Assert `CellUndecided`. Red when `δ` is half the hull's width — `5.6`
mm, and the root reads `CellClear` across a struck pin — and when the dependent's term is dropped. A pin on
the corner itself is no fixture for the leg: the bar's trailing edge passes within `2.2` mm of it at the
centre, under either `τ_half`.

**The held loop.** Scene 11 with the crank over `[30°, 30°]` and the gate over `[0, 10]`, at
`WithResolution(Scalar(1.0/64))`: the follower stands at `θ4(30°) = 101.9717°`, `y_c = 69.3072`, and the
boundary is `d_30 = 2.6928`. Assert every `CellClear` leaf has `d_hi ≤ d_30` and every colliding or blocked
leaf's centre `d_c > d_30`; every leaf's `Center.Values[2]` is within `1e-9` rad of `θ4(30°) − θ4(0) =
−8.3285°` with `Bounds[2]` positive and below `1e-9`, and `Cell.Min[2]`, `Cell.Max[2]` within `1e-9` of it
(measured: `97` centres into `49` leaves). Red when a held driver's loop stands at the zero pose: the
boundary is then `4.9601`, and a clear leaf's bound exceeds its true gap.

**The fold box.** Scene 9's four-bar (ground `100`, crank `50`, coupler `60`, follower `50`), the crank
over `[0°, 90°]`, no static body, at the defaults. The loop folds at `s_fold = 0.974528`. Assert `Suspect`;
no collision; every `CellClear` leaf's range ends at or below `s_fold`; the leaf holding `s_fold` is
undecided and at the floor; every leaf past it is undecided with a zero `Center`, no row, and a finding
whose `Cell` is set and whose `Message` names the loop and `sketch`'s refusal; some such leaf is wider
than the verdict floor; `CellsEvaluated` is below `64` (measured: `21` centres into `11` leaves). Red when
the hull gate is dropped — the root's hull spans the fold, every pair is settled, and the root reads
`CellClear` and the report `Sound` — and when a stuck cell splits to the floor: every leaf past the fold is
then floor-sized, `67` centres.

**The blocked box.** Scene 11 with the gate shortened to `x ∈ [60, 120]` (area `2600` mm²), the crank over
`[35°, 42°]` and the gate over `[8, 10]`, at `WithResolution(Scalar(1.0/64))`. Over that crank range `y_c`
stays within `[69.3725, 69.3857]`, so the corner's depth below the underside `y = 72 − d ∈ [62, 64]` is at
least `5.37` mm, past the bar's full width `8·|cos θ4| ≈ 1.61`, and the overlap is the trapezoidal prism
`8·8·(δ − 4·|cos θ4|)/sin θ4 ≥ 298` mm³; at the floor the allowance is about
`½·(2/64)·2600 + ρ·δ_f·2368 ≈ 57` mm³. Assert every leaf `CellBlocked`, no undecided or colliding leaf,
`CellsEvaluated` below `512` (measured: `83` centres into `42` leaves), and every collision's `Volume` the
trapezoidal prism within `1e-6`. Red when the blocked certificate is dropped: every cell then splits to the
floor along both axes, `8191` centres, each leaf `CellColliding`.

**Standing tests.** Errors, one subtest per row of §16.6: a box listing the crank and the follower, a
box listing the coupler, the flat four-bar of scene 9 (`E0` refuses,
`ErrUnsupported` wrapping `sketch.ErrUnderconstrained`), follower limits `[−5°, 5°]` under the crank over
`[0°, 90°]` (`ErrDegenerate` naming link `2`) and `[−10°, 5°]` admitted, `Configuration` on scene 7's
linkage. The crank over `[−30°, 60°]`, crossing `0` at the non-dyadic `1/3`, and over `[−30°, 1 rad]`,
crossing it at an irrational fraction cut around a straddle (§15.8), each with the gate over `[0, 10]` at
`WithResolution(Scalar(1.0/4))`: every centre reads the closed form within `1e-9`, the stretch below `0` on
the mirrored scene; red when every cell reads on the scene's own side. `PoseAt` of the held drive at a leaf's stated centre values
agrees with its `Center.Values` within `1e-9` and with the closed form. Non-mutation and determinism as
motion §9 test 7; cancellation at every depth, inside an `Enclose` call among them. Internal tests in
`linkage_box_internal_test.go` pin `δ_j` on hand-made `M` and `H` — `M` at an end of `H` gives the hull's
whole width, `M` in its middle half of it — red when it is half the hull's width or §15.5's two-sided term;
pin the split axis at the root of scene 11 built from boxes, the follower a box `x ∈ [70, 100]` with
`ρ = √(30² + 4²)` and the gate over `[0, 5]` mm — the crank's share is exactly `2·ρ·δ ≈ 12` mm against the
gate's `5`, so the crank splits first although it is not on the pair's path — red when a dependent's term
is charged to no axis or dropped; and pin the stuck rule on a hand-built decomposition, a cell inside the
refused set left unsplit, one straddling it split along the loop axis, one meeting it at a point only
left unsplit.

`.github/test-shards.txt` and `.github/test-shards-apitest.txt` are updated for every test and example,
and `go test . ./apitest/ -run '^TestCI'` is run before the push.

### 16.9 Increments

| PR | lands | still `Suspect` after it |
|---|---|---|
| B1 (`linkage_box.go`, `linkage_loop.go`) | §16.1's admission and errors; the loop axis as a drive with `prepareLoops` before the bounds (§16.2); the centre over `AtRange`, `JointConfiguration.Bounds`, `δ_j` in `τ_half` and in the blocked allowance, the loop gate, unbuildable centres and the stuck rule, the driver-axis attribution, the dependents' `Cell` and `Center` entries (§16.3, §16.4); scene 11 and its example, the one-axis loop box, the fold box, the standing and internal tests; this section, and §14.1, §15.6, §15.8 and §15.10 read as this section states | nothing this section refuses: the remaining fixtures pin terms B1 already carries |
| B2 (tests only) | the clear box and its reading, the turn-back cell, the held loop, the blocked box; the measured counts and times of §16.7 recorded | — |

B1 is the end-to-end instance: the real four-bar, the real `Enclose`, the real kernel, cells over a loop,
one report, with scene 11's closed-form boundary as its acceptance. Its code is one change — `τ_half`
and the blocked allowance share one term list, so a dependent's term reaches both or neither — which is
why B1 is not split by certificate.

### 16.10 Settled points

A box varies a loop at its driver alone, listed as a drive lists it; a dependent is never an axis and
is never stated; a held driver holds the loop (§16.1). `Configuration` refuses a loop, and a looped
configuration is a held drive posed by `PoseAt` (§16.1). The loop axis is the drive `Min → Max` in the
fraction, so the cell tree's dyadic intervals are §6's grid and §15.3's chain is unchanged; cells sharing
a loop-axis interval share every ask (§16.2). A dependent's term in `τ_half` is the full one-sided
distance `δ_j` from the centre's enclosure to the far end of the cell's hull, never half the hull, and the
same term enters the blocked allowance (§16.3). A cell whose hull or centre `sketch` refused is never clear
or blocked, splits along the loop axis alone, and is stuck once its range meets no certified cell of the
decomposition (§16.3, §16.4). A dependent's term is charged to its driver's axis when an axis is ranked,
and a cell publishes its dependents' hull and the centre's bounded value (§16.4).
