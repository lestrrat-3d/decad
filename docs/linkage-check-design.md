# Linkage Check Design

How `Document.VerifyLinkage` answers "does any link of this mechanism hit a fixture, or another link, while
the joints move?" without changing the document: where the capability sits (§1), the linkage vocabulary
(§2), the entry point (§3), the report (§4), what a pose and an interval prove for a chain of joints (§5),
the procedure (§6), coverage (§7), errors (§8), what is deferred and why (§9), cost (§10), required tests
(§11), increments (§12), settled points (§13), and the joint-box check `Document.VerifyJointBox`, which
proves a whole box of joint values clear cell by cell (§14). Companion to `docs/motion-check-design.md` ("motion §N"),
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
`sketch`, because v1 drives every joint explicitly and solves no closure (§9).

| Out of scope for v1 | Where it goes |
|---|---|
| Closed loops (a four-bar, a slider-crank) | §9.1: deferred until `sketch` certifies a solved configuration |
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
    Poses         []LinkagePoseResult  // every pose evaluated, from s = 0 to s = 1
    Intervals     []MotionInterval     // between adjacent Poses, in the same order
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

**The certificate.** With `τ` so formed, the interval is `IntervalClear` for a pair exactly when
`lo_a + lo_b > τ(s_b − s_a)` over exact rationals, with `lo` the proven lower ends of the two endpoint gap
intervals after `η` (§5.1), and every undeclared pair must certify for the interval to be `IntervalClear`.
Motion §5.2's three consequences carry over unchanged: a touching endpoint never certifies its interval, a
margin is proven by the lower envelope and disproven by an upper bound only, and a proven collision at an
endpoint ends the argument.

### 5.3 What is exact, what is bounded, what refuses

| Quantity | Standing |
|---|---|
| `s`, each joint's exact value `q_i(s)`, each `|Δq_i|` for a length, every sum and product in `τ` and the balls | exact rationals |
| `|Δq_i|` for an angle, each sine and cosine in an ideal pose | enclosed: `π` and `TurnSinCosInterval` at their upper enclosures |
| each square root in a `ρ` | up-rounded (`RatSqrtUp`) |
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
- Nothing about a closed loop: v1 builds trees only, and a caller who drives two joints to keep an
  undeclared loop closed has stated two independent sweeps, which the report speaks for as stated, not a
  closure (§9.1).
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
5. **Evaluate, bisect and publish** as motion §6 steps 4–7, with `τ` per pair from §5.2 and the declared
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

### 9.1 Closed loops

A four-bar's follower angle is not stated by the caller; it is whatever closes the loop at each crank
angle. That is a 2D constraint problem for a planar loop, and CLAUDE.md's rule sends it to `sketch`. v1
does not take that step, for a reason the soundness rule decides: `sketch.Solve` returns float coordinates
with a residual norm and `Converged`, and `Sketch.Verify` reports `Solvable` against a tolerance, but
neither publishes an enclosure of the solved configuration — a bound on the distance from the returned
coordinates to the exact solution. A pose built on an uncertified solve is a pose the report cannot speak
for: decad may falsify it (a proven gap at a pin-bore pair the loop claims closed disproves closure at that
pose) but never admit it, and a Newton-type bound computed on decad's side would be an admission gate on a
residual, which the hard rule forbids. A follower angle is also not linear in the crank angle, so §2.3's
schedules could not carry it even if it were certified.

Loops therefore wait on one upstream capability: `sketch` publishing a certified enclosure of a solved
configuration — the analogue of `BoundaryEdge.TExact` for a solve. With it, a loop becomes a tree plus a
closure joint whose dependent schedule is enclosed per `s`, the chain bound of §5.2 takes the enclosure's
width as an extra travel term, and decad still only selects among what `sketch` returns. Until then, a
caller who knows a dependent angle in closed form states it as its own sweep of a tree, and the report
speaks for that stated motion, not for closure (§5.6).

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
evaluated per pose. At `WithResolution(Scalar(1.0/64))` the verdict settles in `10` poses, about `30` ms,
every interval `IntervalClear` and the reading beyond tolerance. At the defaults the verdict settles by the
verdict floor in `50` poses, and the reading refines around its one minimum to the reading floor: `251`
poses, about `0.7` s, `Sound`.

**The whole-drive reading at the default floor.** The verdict is cheap; the reading need not be. A certified
interval's lower bound sits up to `τ/2` below the true gap, so the reading's half-width near the minimum
is about `τ_rate·Δs/4`, and the gate (verification §2) admits `rel·gap`. A chain's `τ_rate` is the sum of
its `ρ_{ik}·|To_i − From_i|` — hundreds of millimetres per unit `s` for an arm of a few links — so at
`rel = 1e-3` and a `10` mm gap the reading needs `Δs ≈ 7e-5`, under the verdict floor `1/1024`. That is
why the reading has its own floor (§3): refinement past the verdict floor goes only to the interval holding
the smallest bound, so around an isolated minimum it costs about `log₂(16)` halvings per tie broken (the
three-joint arm above: `50` poses for the verdict, `251` with the reading). A caller who states
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
  `(A, B)` row. The check evaluates `16` poses.
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
equivalent `Revolute`: equal `Status`, equal interval outcomes in order, each `Collision.At` equal under
`s ↦ From + s·(To − From)` within `1e-9°`, path `Clearance.Value` within `1e-9` mm, on motion §9's
fixtures 1, 2 and 4. The one-link `τ` is `ρ_{11}·|Δq_1|`, which is `MoverTravel`'s value, and the ideal
pose is one `MotionFrame.At`, so the two paths read the same bounds.

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

**The reading floor.** At the defaults the three-joint arm of §10 reads `Sound`, its reading inside the
gate, with `ReadingResolution` `1/16384` and some interval narrower than `1/1024`; red when the reading
floor is dropped. At `WithResolution(Scalar(1.0/64))` it reads `Suspect` with no interval narrower than
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
second entry point `VerifyJointBox` (§2.3, §14). Closed loops wait on a certified solve from `sketch`; decad
never admits an uncertified one
(§9.1). Joint contacts are declared by the caller per pair, listed in the report, and still checked for
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

// JointConfiguration is one point of joint space: every link's joint value
// and world pose, in Linkage.Links() order.
type JointConfiguration struct {
    Values []units.Value
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
(`ErrNotFinite`). It is the one place a configuration's poses are built, so a renderer drawing a cell's
corner and the verifier evaluating its centre read the same transform; `PoseAt` is `Configuration` of the
drive's values at `s`.

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
(§5.1), the 1-Lipschitz fact of motion §5.2 gives `gap(q) ≥ lo_m − τ_half` for every `q` in `C`, so the
cell is clear for the pair exactly when

```text
lo_m > τ_half(C)      (compared over exact rationals; lo_m the float the pose proved)
```

and its proven lower bound over the cell is `lo_m − τ_half`, rounded down. The cell is `CellClear` when
every undeclared evaluated pair certifies and no declared pair collides at the centre; an excluded pair
(§5.7, §6 step 4) contributes its proven lower bound and nothing else. A touching or undecided centre
certifies nothing for that pair, as a touching endpoint does (motion §5.2).

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
   else `CellColliding` when some pair collides there; else `CellClear` when every pair certifies; else
   `CellUndecided` for now.
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
   smallest lower bound (ties in cell order) along the axis maximising `w_i·span_i` for the pair that
   attained it, while that axis is wider than the reading floor; and while a requested margin is neither
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
- **Three joints.** The three-joint arm of §10 over three quarter turns has `τ_half ≈ 236` mm at the root;
  a `10` mm gap everywhere needs cells of about `1/24` per axis, `24³ ≈ 14000` leaves and twice that many
  centres, at the edge of the default budget: the default run reaches depth `14`, about `1/20` per axis,
  and raises the budget finding. A caller who wants that box proven states `WithCellBudget(65536)` and
  waits about three minutes, or holds one joint. The layer exclusion is what makes a planar stack cheap: a
  pair it settles costs no cell, and scene 1's arms without the wall read `Sound` from the root alone.
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

**The clear box and its reading.** The same box with the wall moved to `y ∈ [100, 120]`: the minimum gap
over the box is `100 − y(80°, 30) = 100 − 90·sin 80° − 5·cos 80° ≈ 10.499` mm, at the box's corner.
Assert `Sound`, every leaf `CellClear`, `Clearance` enclosing `10.499` with `ToleranceSatisfied` at the
defaults, `ReadingResolution` `1/16384`, some leaf narrower than `1/1024` along `θ`; `WithMinClearance`
`10` mm `AssessmentMet`, `11` mm `AssessmentViolated` with a `DiagMotionClearanceViolated` whose `Cell` is
set. At `WithResolution(Scalar(1.0/64))` the reading is beyond tolerance and the report `Suspect` with no
leaf narrower than `1/64`; red when a stated resolution leaves the reading floor at its default.

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
| 3 | step 6: the whole-box reading's refinement, the reading floor and `ReadingResolution`, the margin; the clear box and its reading; the three-joint cost of §14.7 measured and recorded | a stated `WithResolution` too coarse for the reading; a gap constant along an axis whose gate needs the budget |

PR 1 is the end-to-end instance: the real crane, the real kernel, the cell certificate over real cells,
one report, with scene 6's closed-form region as its acceptance. This section ships in PR 1.

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
code and one `Diagnostic` field are added (§14.2).
