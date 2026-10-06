# Linkage Check Design

How `Document.VerifyLinkage` answers "does any link of this mechanism hit a fixture, or another link, while
the joints move?" without changing the document: where the capability sits (§1), the linkage vocabulary
(§2), the entry point (§3), the report (§4), what a pose and an interval prove for a chain of joints (§5),
the procedure (§6), coverage (§7), errors (§8), what is deferred and why (§9), cost (§10), required tests
(§11), increments (§12) and settled points (§13). Companion to `docs/motion-check-design.md` ("motion §N"),
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
| A box of joint values (a multi-DOF workspace sweep) | §9.2: a later increment over the same travel bound |
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
`dir` (`ErrDegenerate`); limits of the wrong `Kind` (`ErrUnitKind`), non-finite (`ErrNotFinite`) or with
`Min >= Max` (`ErrDegenerate`). Liveness and document membership are not checked here; `VerifyLinkage`
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

`DeclareJointContact` refuses two bodies of one link, two bodies that both belong to no link, a pair
declared twice in either order, and `a == b`, each with `ErrDegenerate`. Liveness and document membership
are checked at `VerifyLinkage`.

### 2.3 The drive

```go
// Drive is a one-parameter motion of the linkage, parameterised by the
// Dimensionless fraction s ∈ [0, 1]: each listed joint runs linearly from
// From to To as s runs from 0 to 1, and an unlisted joint holds 0.
type Drive []JointSweep

type JointSweep struct {
    Link     *Link       // the link whose joint this moves
    From, To units.Value // the joint's Kind: Angle for a revolute, Length for a prismatic
}
```

A joint's value at `s` is `q(s) = From + s·(To − From)`, exactly as a `Between`'s parameter is a fraction
of its path (motion §2): the parameter every report record carries is `units.Scalar(s)`, a dyadic
fraction, exact in float, and each joint's value is a label computed as `motionSpec.label` computes a
`Between` pose's parameter, while every bound reads the exact rational `From + s·(To − From)` through
`motionbound.MotionParam.Lerp`. `From == To` is legal for one sweep and holds the joint at that value for
the whole drive; a drive in which every listed sweep holds is `ErrDegenerate`, as `From == To` is for a
`Motion`. `From > To` is legal and runs the joint the other way.

This is option (a) of the two the design weighed: one scalar drives every joint through a stated schedule.
It is the natural extension of `VerifyMotion`, whose `Between` already proves a path over a dimensionless
fraction, and it covers the question a mechanism designer asks first — "does this coordinated motion
clear?" — with the interval certificate unchanged in form. Option (b), proving a whole box of joint values
clear, is §9.2.

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
`TransformTrack` for `kinetograph` is one `PoseAt` call per frame.

`PoseAt` refuses a drive naming a link of another linkage or a link twice (`ErrDegenerate`), a sweep whose
`From` or `To` has the wrong `Kind` for its joint (`ErrUnitKind`) or is non-finite (`ErrNotFinite`), a
value outside declared limits — `q(s)` is monotone in `s`, so `From` and `To` inside the limits put every
`q(s)` inside them, and either endpoint outside is `ErrDegenerate` with a message naming the link — and an
`at` that is not a finite `Dimensionless` value (`ErrUnitKind`, `ErrNotFinite`). An `at` outside `[0, 1]`
is legal for `PoseAt`, which takes no range, and is never evaluated by `VerifyLinkage`.

## 3. The entry point

```go
func (d *Document) VerifyLinkage(ctx context.Context, l *Linkage, drive Drive, opts ...MotionOption) (*LinkageReport, error)
```

The options are `VerifyMotion`'s own, with the meanings motion §3 gives them: `WithMotionTolerance` is the
relative tolerance on every bounded reading; `WithResolution` is the finest fraction the check refines to,
a `Dimensionless` value as for a `Between`, default `units.Scalar(1.0/1024)`; `WithMinClearance` is a spec
over every evaluated pair and decides `Assessment`. One `Go` option type serves both entry points because
the three settings mean the same thing on either path.

Every body of every link MUST be a live body of `d`: a retired body is `ErrRetiredBody`, another
document's `ErrForeignBody`, a body this evaluator did not build `ErrUnsupported`. A linkage with no link
is `ErrDegenerate`. The static set is every other live body of `d`, with no option to narrow it (motion
§3): a report is `Sound` about the whole document or not at all. A nil `ctx`, nil `l` or nil `d` is
`ErrDegenerate` before anything else; validation precedes the cancellation check, as in verification §1.2.

## 4. The report

```go
type LinkageReport struct {
    Request       MotionRequest        // the validated effective settings, including defaults
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
upper enclosure for an angle, exact for a length.

**Reading `ρ_{ik}`.** Each link's rest box is the per-axis union of its bodies' `Bounds()` boxes, each
inflated outward by its own `Bound`, read as exact rationals (`boxCornersExact`). The reading walks DOWN the
path from link `k` toward joint `i`, carrying a ball that encloses link `k` under the joints walked so far:

| step | enclosure carried |
|---|---|
| start, `i = k` | `ρ_{kk}` is `moverAxisRadius`'s reading: the largest exact distance of a rest-box corner from axis `k`, rooted upward. Distance from a line is convex, so the maximum over the box sits at a corner. No ball is needed. |
| ball under joint `k` | revolute: centre `c_k` (the joint's `Center`), radius the largest corner distance from `c_k`, rooted upward — a rotation about any axis through `c_k` preserves distance to `c_k`. Prismatic: centre the box centre, radius the box's half-diagonal plus `m_k = max(|From_k|, |To_k|)`, the farthest the joint slides over the drive (`0` for an unlisted joint). |
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
(`From == To`) has `|Δq_i| = 0` and contributes nothing to any `τ`; its `m_i = |From_i|` still enters the
balls, because the link sits displaced by it.

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
  `DiagMotionCollision`, and drives §6's onset bisection — a jammed or over-rotated joint is found;
- a measured gap publishes its `Clearance` row and nothing else;
- a touching, undecided or unsupported outcome publishes nothing — no diagnostic and no `Suspect`.

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

### 5.6 What is never claimed

- Nothing about `s` outside `[0, 1]`, nothing about a configuration the drive does not visit (§9.2).
- Nothing between two bodies of one link.
- Nothing continuous about a declared joint contact (§5.4).
- Nothing about a closed loop: v1 builds trees only, and a caller who drives two joints to keep an
  undeclared loop closed has stated two independent sweeps, which the report speaks for as stated, not a
  closure (§9.1).
- An `IntervalUndecided` claims nothing; a `LinkCollision` claims overlap at that `s` and nothing about the
  interval around it; no tolerance decides admission (motion §5.4).

## 6. The procedure

Motion §6's deterministic dyadic bisection is reused as it stands — endpoints first, then bisection for
the verdict with onset bracketing of a colliding interval, then bisection for the reading, each bounded by
the resolution floor — and `linkage_verify.go` runs it over `motion_verify.go`'s engine generalised to
several moving groups (§12 PR 1). The steps that differ:

1. **Validate** (§2, §3, §8) before reading `ctx`.
2. **Resolve each link's standing.** A link whose path joints all hold `0` — every sweep on its path
   absent or `From == To == 0` — moves nothing: its bodies are evaluated as static bodies, with no
   transient placement and no `η`, and its pairs against the ground's statics are not formed (they are
   `Verify`'s). A link whose path joints all hold a nonzero value is a constant placement: it is evaluated
   once, at `s = 0`, and every pose reuses that evaluation, with `τ = 0` on every pair it forms.
3. **Read the bounds** (§5.2): per link, `R0`, area and `σ` per body as motion §6 step 2 reads them, and
   `ρ_{ik}` for every joint `i` on its path.
4. **Swept-box exclusion.** Each body of link `k` has its rest box, inflated by its own `Bound` plus its
   link's travel from the zero pose to the farthest the drive takes it, `Σ_{i ≤ k} ρ_{ik}·m_i` (prismatic
   terms `m_i`), compared with each static body's inflated box as exact rational extremes (motion §6 step
   3); a strictly positive axis gap settles the pair for the whole drive. The body's own box serves, not
   the link's: `ρ_{ik}` is read over the link's whole rest box, so it bounds the travel of every one of its
   bodies. A link-link pair is settled the same way when the two
   swept boxes separate. The travel is measured from the zero pose, not from `s = 0`, for the reason motion
   §6 step 3 gives: a joint whose `From` is `80°` has moved before the drive begins.
5. **Evaluate, bisect and publish** as motion §6 steps 4–7, with `τ` per pair from §5.2 and the declared
   pairs handled as §5.4 says. The onset bisection halves a colliding interval that still has a
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
| nil `ctx`, nil document, nil linkage; a linkage with no link; a drive in which every sweep holds; a drive naming a link twice or a link of another linkage | `ErrDegenerate` |
| a link body retired, foreign, or not built by this evaluator | `ErrRetiredBody`, `ErrForeignBody`, `ErrUnsupported` |
| a declared joint contact naming a retired or foreign body | `ErrRetiredBody`, `ErrForeignBody` |
| a sweep's `From` or `To` of the wrong `Kind` for its joint; a wrong-`Kind` resolution, tolerance or minimum | `ErrUnitKind` |
| a non-finite sweep value, option, box or axis read | `ErrNotFinite` |
| a sweep endpoint outside the joint's declared limits | `ErrDegenerate`, message naming the link |
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

Proving every configuration in `[Min_1, Max_1] × … × [Min_n, Max_n]` clear is the same certificate over a
grid in `n` dimensions: §5.2's `τ` already bounds travel for any change of several joints at once, so a
box bisected per axis to a resolution floor certifies cell by cell with the same `lo + lo > τ` test at its
`2^n` corners. The cost is exponential in `n` and the report needs a cell vocabulary; it is a later
increment that adds a second entry point and changes nothing here.

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
for a clear sweep, and `10`–`20` onset poses per contact found. §12 PR 2 records the measured figure.

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
- `A` and `B` never meet: their caps are the parallel planes `z = 10` and `z = 12`, `2` mm apart at every
  `s`. The pair's relative motion is joint 2 alone (§5.2's lowest-common-ancestor rule), `ρ_{22} = 50`, so
  `τ = 50·(π/2)·Δs` and `2 + 2 > τ` holds at `Δs = 1/32`.
- At `WithResolution(units.Scalar(1.0/256))` assert: `Status` is `Interfering`; the first `LinkCollision`
  has `A` the forearm and `B` the wall, `At` strictly above `1/3` and within `2/256` of it — the grid
  point `86/256`, where the overlap is `480·(48·sin(30.234°) − 24) ≈ 81.5` mm³, asserted within `1e-3`
  mm³ with `Bound` below `Value`; every `IntervalClear` interval ends at or below `1/3`; the interval
  containing `1/3` is not `IntervalClear`; every `(A, wall)` collision has `At > 0.369`; every `(A, B)`
  row is a `Clearance` within `1e-6` of `2` mm; fewer than `32` poses were evaluated. The check evaluates
  `21`: the `(A, B)` pair certifies at `Δs = 1/32` over `[0, 1/3]`, and past the first collision every
  interval collides at both ends and is not split.
- The same linkage with the wall removed, at the default resolution and `WithMotionTolerance(Scalar(0.05))`:
  `Sound`, every interval `IntervalClear`, `Clearance.Value` within `0.1` mm of `2` with
  `ToleranceSatisfied`. The `(A, B)` interval lower bound is `2 − 25·(π/2)·Δs`, so the whole-drive
  reading's half-width is about `19.6·Δs` mm: the default `rel = 1e-3` admits `0.002` mm on a `2` mm gap,
  which takes `Δs ≈ 1e-4`, while `0.05` admits `0.1` mm, which the default floor reaches at `257` poses.
- The pose-count assertion is the leg that goes red when the common-ancestor joint is left in `τ_{AB}`:
  the `(A, B)` certificate then needs `Δs ≤ 1/128` over `[0, 1/3]` and the count rises to `50`. The
  travel terms themselves are not detectable here — `B` translates on a circle at `75` mm per unit `s`,
  under every partial sum of its bound — and are pinned by the internal travel test below and by scene 4.
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
Assert: `Interfering`, the first `LinkCollision` on `(B, fence)` with `At` strictly above `s*` and within
`2/256` of it at `WithResolution(Scalar(1.0/256))`; `JointContacts` lists `(A, B)`; no diagnostic names
`(A, B)`; the same scene without the declaration reads `Suspect` with every interval touching `(A, B)`
`IntervalUndecided`. The leg that a declared pair is still proven for overlap is pinned separately: a
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
which the test brackets to `1e-9` by bisection of the closed form; the `−5` corner follows later. Assert
`Interfering` with the first collision on `(boom, wall)` within `2/256` above `s*`, and that the
`(mast, boom)` pair — relative motion joint 2 alone, gaps `2 + 2 > 30·Δs` — reads `Clearance` rows within
`1e-6` of `2` mm. The prismatic term is pinned with the mast unlisted: a `2 × 2 × 2` mm pin `4` mm ahead
of the boom's tip, `14` mm behind it at the end, with the endpoints alone (`WithResolution(Scalar(1))`):
no `Collision`, `IntervalUndecided`, `Suspect` — exactly motion §9 test 5's pin, which goes red when the
`|Δq|` term of a prismatic joint is dropped.

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

**Agreement with `VerifyMotion`.** A one-link linkage on a revolute joint and the same body under the
equivalent `Revolute`: equal `Status`, equal interval outcomes in order, each `Collision.At` equal under
`s ↦ From + s·(To − From)` within `1e-9°`, path `Clearance.Value` within `1e-9` mm, on motion §9's
fixtures 1, 2 and 4. The one-link `τ` is `ρ_{11}·|Δq_1|`, which is `MoverTravel`'s value, and the ideal
pose is one `MotionFrame.At`, so the two paths read the same bounds.

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
| 2 | `DeclareJointContact`, `JointContacts`, §5.4's asymmetric outcome; `WithJointLimits` with `JointOption`, `JointLimits` and the joints' `Limits` fields; scenes 2, 3 and 4; the three-joint benchmark of §10 | — |
| 3 | §6 step 2's held-link promotion to static and constant placement; link-link swept-box exclusion; the cylinder enclosure for parallel axes where measured cost justifies it | — |

PR 1 is the end-to-end instance: two real links, a real fixture, the real kernel, the chain certificate,
one report, with the closed-form answer of scene 1 as its acceptance. This design document ships in PR 1.

## 13. Settled points

The linkage check lives in the root package, not a subpackage, because the per-pose kernel is private
and a public multi-body primitive carrying rational-interval ideal poses is not a shape core §5 admits
(§1). Links are sets of bodies and joints attach a link to its parent; the zero pose is the document as
it stands; joint frames are world coordinates at the zero pose (§2.1). v1's joints are revolute and
prismatic; there is no fixed joint, because a rigidly attached part is listed in its link (§2.1). The
drive is one dimensionless fraction with a linear schedule per joint; a box sweep is a later entry point
(§2.3, §9.2). Closed loops wait on a certified solve from `sketch`; decad never admits an uncertified one
(§9.1). Joint contacts are declared by the caller per pair, listed in the report, and still checked for
overlap at every pose (§2.2, §5.4). The travel bound is the telescoping joint-by-joint sum with ball
enclosures read down the chain, over exact rationals (§5.2); a pair's bound sums only the joints below the
two links' lowest common ancestor (§5.2). The options and the report vocabulary are `VerifyMotion`'s (§3,
§4).
