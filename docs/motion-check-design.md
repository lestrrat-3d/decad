# Motion Check Design

How `Document.VerifyMotion` answers "does this body hit anything while it moves along this path?" without
changing the document: where the capability sits (§1), how the caller states the motion (§2), the entry point
and its options (§3), the report (§4), what a pose proves and what an interval between two poses proves (§5),
the refinement procedure (§6), evaluator coverage (§7), errors (§8), required tests (§9), increments (§10),
and the dependencies and settled points (§11). Companion to `docs/api-design.md` (the core contract,
referenced as "core §N"), `docs/verification-design.md` ("verification §N", which owns `Status`,
`Diagnostic`, the tolerance gate and the result vocabulary this report reuses), `docs/clearance-design.md`
("clearance §N", which owns the pair kernel every pose runs), `docs/interference-design.md` ("interference
§N", which owns the pair relation and the read-only overlap proof) and `docs/evaluator-design.md`
("evaluator §N", whose §8 owns placement re-evaluation). Nothing here changes those contracts.

## 1. Scope: what decad owns, and what the layer above owns

**`VerifyMotion` owns the verification half of one prescribed motion: a proof that one rigidly moving set of
bodies does or does not meet the rest of the document anywhere along a stated one-parameter path.** Core §1
lists "do these bodies interfere, and what is the clearance?" among the questions decad is answerable for,
and core §13 settles how
such a pair is positioned: interference and clearance run between explicitly placed bodies, through
`Body.Placed(ctx, t)`, with no assembly machinery. A motion check is that same pair question asked over a
family of placements `t(s)`, `s ∈ [From, To]`, instead of at one placement. It consumes exactly what core
§13 already grants — a rigid motion stated by the caller as an `r3.Transform` — and adds no `Component`, no
`Occurrence`, no joint graph, no timeline and no view state. Every v1 non-goal of core §13 stays a non-goal.

Three things do not belong in `VerifyMotion`, and this design keeps them out:

| Out of scope | Why | Where it goes |
|---|---|---|
| Animation, frame generation, rendering | Core §4 rejects GUI and view state on the geometry model; nothing here produces an image or a frame sequence | A separate module that imports decad and calls `Motion.PoseAt` (§2) for the poses it wants to draw |
| Kinematic chains, joints, linkages, more than one independent motion | One rigid moving set has one path | `Document.SweepPair` checks two source-box paths; a layer above owns joints |
| Dynamics, contact forces, time | `VerifyMotion` measures geometry; its parameter `s` is an angle, a length or a dimensionless fraction of the path, never a time | The `dynamics` subpackage; see `docs/collision-dynamics-design.md` |

The layering rule holds unchanged: `decad -> sketch -> r3 -> units`. Every pose is an `r3.Transform` built by
`r3.RotationAround`, `r3.Translation` or, for a `Between`, `r3.Transform.Screw` read once and `r3.Screw.At`
per pose; decad composes it onto a body's placement with `Transform.Then` exactly as `Body.Placed` does, and
hand-rolls no rotation. §11 records what each dependency supplies and the one bound `r3` does not publish.

## 2. The motion vocabulary

`Motion` is a sealed interface, like `Axis` and `Extent` (core §6.2, §8.1): a motion is one of a fixed set of
shapes, and an illegal one is unrepresentable rather than rejected at runtime.

```go
// Motion is a one-parameter family of rigid motions, parameterised by a typed
// scalar whose Kind the variant fixes: an Angle for Revolute, a Length for
// Prismatic, a Dimensionless fraction of the path for Between. PoseAt returns
// the rigid motion at one parameter value; it is what a layer above calls to
// draw the moving set, and what VerifyMotion calls to evaluate a pose.
type Motion interface {
    PoseAt(at units.Value) (r3.Transform, error)
    motion()
}

// Revolute rotates about the axis through Center along Axis, right-handed,
// from angle From to angle To. The angle 0 is the moving body as it currently
// sits; the motion composes onto each moving body's own placement.
type Revolute struct {
    Center r3.Vec      // a position, millimetres (core §5.2)
    Axis   r3.Vec      // a direction; only its direction is used
    From   units.Value // Kind Angle, signed
    To     units.Value // Kind Angle, signed
}

// Prismatic translates along Dir from displacement From to displacement To.
// The displacement 0 is the moving body as it currently sits.
type Prismatic struct {
    Dir  r3.Vec      // a direction; only its direction is used
    From units.Value // Kind Length, signed
    To   units.Value // Kind Length, signed
}

// Between is the screw motion joining two rigid poses: the parameter s runs
// from 0, where each moving body sits under From composed onto its own
// placement, to 1, where it sits under To. The path rotates through the
// shorter arc between the two orientations and slides uniformly along the
// rotation axis (Chasles; r3 docs/screw-motion-design.md).
type Between struct {
    From r3.Transform // the pose at s = 0; r3.Identity() is the moving set as it currently sits
    To   r3.Transform // the pose at s = 1
}
```

A `Revolute`'s and a `Prismatic`'s `From` and `To` are **signed displacements, not magnitudes**, on the same
terms as `ToFace.Offset` and `ExtrudeOpts.Taper` (core §12): the sign says which way the body moves, so a
negative value is a legal intent and never `ErrNegativeMagnitude`. `From == To` names no path and is
`ErrDegenerate`. `From > To` is legal and traverses the path in the other sense; the report (§4) lists poses
in traversal order.

A `Between`'s parameter is the **dimensionless fraction `s ∈ [0, 1]` of the path**, carried as
`units.Scalar(s)` (`Kind` `Dimensionless`) wherever a report carries a parameter: `PoseResult.At`,
`MotionInterval.From`/`To`, `Collision.At` and `Diagnostic.At`. Its path always runs from `0` to `1`, so it
has no reversed sense, and every parameter the check labels a pose with is a dyadic fraction, exact in
float. A `From` equal to `To` names no path and is `ErrDegenerate` (§8).

`PoseAt(at)` is `r3.RotationAround(Center, Axis, at)` for a `Revolute` and `r3.Translation(d·at)` for a
`Prismatic`, where `d` is `Dir` normalised by `r3.Vec.Normalize`. For a `Between`, `PoseAt(s)` reads the
relative motion `rel = From⁻¹ ∘ To` (`From.Inverse().Then(To)`), decomposes it with `rel.Screw()` into the
axis direction `Axis`, the axis point `Point`, the angle `θ ∈ [0, π]` radians and the slide `d` (r3
`docs/screw-motion-design.md` §3–§4), and returns `From.Then(screw.At(s))`: the rotation by `s·θ` about the
line through `Point` along `Axis`, slid by `s·d` along it, applied after `From`. At `s == 0` it returns
`From` and at `s == 1` it returns `To`, both as stated: `screw.At(0)` is the identity bit for bit, so
`From.Then(screw.At(0))` IS `From`, while `screw.At(1)` rebuilds `rel` only to rounding (r3 §7), so the
stated `To` is returned in its place and §5.1 charges the difference. `Transform.Screw` is a deterministic
function of its input bits (r3 §3), so every `PoseAt` call and §5's exact frame read the same parameters.
`Transform.Interpolate` is not called: the certificate needs the parameters, not only the pose. A `Between`
traces the SHORTER of the two arcs between its orientations, because `θ ≤ π`; a swing past a half-turn is
stated as a `Revolute`, or as two `Between` legs through an intermediate pose. At exactly `π` both arcs are
equally short and r3 picks one deterministically (r3 §3); the report's `Poses` show which. `PoseAt` is the
ONLY place a pose transform is built, so the renderer above and the verifier below evaluate the same pose
from the same inputs. Its refusals follow the variant's own fields and the `r3` constructor's:

| Input | Error |
|---|---|
| `From` or `To` of the wrong `Kind` (a length for a `Revolute`, an angle for a `Prismatic`, a bare scalar for either) | `ErrUnitKind` |
| `at` of the wrong `Kind` (a `Between` takes a `Dimensionless` fraction and refuses an angle or a length) | `ErrUnitKind` |
| a non-finite `From`, `To`, `at`, `Center`, `Axis` or `Dir` component | `ErrNotFinite` |
| `Axis` or `Dir` the zero vector (`r3.ErrDegenerateAxis`, or `Normalize` reporting no direction) | `ErrDegenerate` |
| `From == To` (checked by `VerifyMotion`, not by `PoseAt`, which takes no range) | `ErrDegenerate` |
| the `r3` constructor refusing the result (`r3.ErrNonFinite`, an overflowing pivot offset) | `ErrNotFinite` |
| a `Between` `From` or `To` with a non-finite basis or translation component | `ErrNotFinite` |
| a `Between` `From` or `To` that is not a rigid motion (`Transform.IsValid` false: the zero `r3.Transform{}`, a skewed basis) | `ErrDegenerate` |
| a `Between` joining a reflection to a proper motion (`r3.ErrImproper`: no rigid path joins the two handedness classes); two reflections are a legal pair, as a reflecting placement is legal for `Placed` | `ErrDegenerate` |
| a `Between` whose `From` equals `To` (`==` on the two values), or whose relative motion decomposes to the zero screw (`Angle` and `Slide` both zero); checked by `VerifyMotion`, since `PoseAt` of such a motion is `From` at every `s` | `ErrDegenerate` |
| `rel.Screw()` or `screw.At` refusing the result (`r3.ErrNonFinite`: an axis point or slide past `MaxFloat64`, a `1e-300` rad rotation with a `1e10` mm translation) | `ErrNotFinite` |

A `Between` checks its fields in that order: finiteness, then rigidity, then handedness, then degeneracy;
a `Revolute` and a `Prismatic` check `Kind`, then finiteness, then the direction.

A `Revolute` names its axis with two vectors, a position and a direction, because that form needs no
selector resolution at the call. A second form taking core §6.2's sealed `Axis` — an `EdgeAxis` naming a
hinge pin's own edge — is a later addition on the same `Motion` set; it is not refused today, it does not
exist.

**One shape is deliberately absent.** A list of discrete poses adds nothing the API lacks: `PlacedCopy`
then `Verify` already checks any finite set of placements the caller can name, and a pose list carries no
path between its entries, so nothing continuous could be proven over it (§5.2). A caller with a sequence of
poses states one `Between` per consecutive pair.

## 3. The entry point

```go
func (d *Document) VerifyMotion(ctx context.Context, moving []*Body, m Motion, opts ...MotionOption) (*MotionReport, error)

type MotionOption interface{ /* sealed, option.Interface */ }

func WithMotionTolerance(rel units.Value) MotionOption     // Dimensionless; default units.Scalar(1e-3)
func WithResolution(step units.Value) MotionOption         // the motion's own Kind; default (To−From)/1024 in magnitude
func WithMinClearance(minimum units.Value) MotionOption    // Kind Length; a spec, so Assessment is decided
```

`moving` is the rigid set that moves together under `m`; every other live body of `d` is static and is
checked against every mover. There is no option to narrow the static set: a report is `Sound` about the
whole document or not at all, exactly as `Verify`'s is, and §6's swept-box exclusion is what keeps a far
body cheap rather than a caller's omission of it. A pair of two movers is never evaluated: a shared rigid motion preserves their
relative position, so their relation is whatever `Verify` reports for the document as it stands, and
re-deriving it per pose would prove nothing new. `moving` MUST be non-empty, and each entry MUST be a live
body of `d`: an empty set is `ErrDegenerate`, a retired body `ErrRetiredBody`, another document's body
`ErrForeignBody`, a body listed twice `ErrDegenerate`.

`WithMotionTolerance` is verification §2's relative tolerance under a name of its own, because
`WithTolerance` returns a `VerifyOption` and Go gives one constructor one return type. It governs every
bounded reading the motion report carries (§4) by verification §2's rule, with the pair diameter of
clearance §7 as the reference. Its refusals are `WithTolerance`'s: `ErrUnitKind`, `ErrNegativeMagnitude`,
`ErrNotFinite`.

`WithResolution` states the finest parameter step the check will refine to (§6), for the verdict and for
the reading alike. An interval narrower than the resolution that the certificate still cannot settle reads
undecided and is not split further, and a path reading or margin the floor leaves coarse is published
coarse; it is the caller's statement that a feature narrower than this does not need to be found and that
figures finer than it buys are not asked for. It is a magnitude of the
motion's own `Kind` — an angle for a `Revolute`, a length for a `Prismatic`, a dimensionless fraction of the
path for a `Between` — so a value of the wrong `Kind` is `ErrUnitKind`, a negative or zero one
`ErrNegativeMagnitude`, a non-finite one `ErrNotFinite`. A resolution larger than `|To − From|` (larger than
`1` for a `Between`) is legal and means the endpoints alone are evaluated. The default, `(To − From)/1024`
in magnitude and `units.Scalar(1.0/1024)` for a `Between`, caps a worst-case run at 1025 poses per pair; it
is a constant the implementation owns and re-sizes from measured per-pose cost, not a value the API
promises.
If that default rounds to zero in `From`'s unit or its base unit, the check uses and reports the smallest
positive resolution in `From`'s unit that `WithResolution` accepts. This can exceed the path span for a
subnormal displacement.

`WithMinClearance` states a spec: the moving set MUST stay at least `minimum` from every static body over the
whole path. Like `WithMinWallThickness` it turns a measurement into an `Assessment` (verification §1.0):
`AssessmentMet` when every interval certifies the margin (§5.2), `AssessmentViolated` when some evaluated
pose proves a gap below it, `AssessmentUndecided` otherwise. Its magnitude rules are `WithMinWallThickness`'s.

Duplicate options keep verification §1.0's last-occurrence-wins rule. A nil `ctx` returns `ErrDegenerate`
before `VerifyMotion` checks the document, moving set or options. With a non-nil context, it validates those
inputs before checking cancellation, so a validation error wins over an already-cancelled context exactly
as in verification §1.2; after validation, a cancelled context returns `ctx.Err()` unchanged and no report.
The context is the public work control: there is no separate budget or progress API
(interference §7.2), and the resolution floor is a spec about the answer, not a work limit.

## 4. The report

The motion report is written in the vocabulary `Verify`'s report already uses — `Status`, `Diagnostic`,
`Interference`, `Clearance`, `ScalarReading`, `Assessment` — and adds only the records that carry the path.

```go
type MotionReport struct {
    Request    MotionRequest    // the validated effective settings this call used, including defaults
    Motion     Motion           // the motion as stated
    Moving     []*Body          // the rigid set, in the order given
    Against    []*Body          // every static body considered, in Document.Bodies() order
    Poses      []PoseResult     // every pose evaluated, in traversal order from From to To
    Intervals  []MotionInterval // the consecutive intervals between adjacent Poses, in the same order
    Collisions []Collision      // every proven collision, in traversal order then pair order
    Clearance  *ScalarReading   // the minimum gap over the WHOLE path; nil unless every interval is certified (§5.3)
    Assessment Assessment       // against WithMinClearance; AssessmentNotEvaluated when not requested
    Diagnostics []Diagnostic    // interval findings in interval order, then pose findings in pose order (§4.1)
    Status     Status           // Unverified on a zero value; VerifyMotion always returns a decided status
}

func (r *MotionReport) Passed() bool // Status == Sound

type MotionRequest struct {
    RelativeTolerance units.Value  // always present
    Resolution        units.Value  // always present, in the motion's Kind
    MinClearance      *units.Value // non-nil exactly when WithMinClearance was requested
}

// PoseResult is one evaluated pose: the parameter, the rigid motion applied to
// the moving set there, and the pair results at that pose in Verify's own shape.
type PoseResult struct {
    At            units.Value    // the parameter; Kind Angle, Length or Dimensionless as the Motion fixes
    Pose          r3.Transform   // Motion.PoseAt(At): what composes onto each mover's own placement
    Interferences []Interference // A is the mover, B the static body; proven overlap, bounded volume
    Clearances    []Clearance    // A is the mover, B the static body; every pair proven disjoint or touching
    Diagnostics   []Diagnostic   // this pose's own undecided or unsupported pairs, pair codes unchanged
}

// MotionInterval is the stretch of the path between two adjacent evaluated poses.
type MotionInterval struct {
    From, To  units.Value     // the two adjacent PoseResult.At values, in traversal order
    Outcome   IntervalOutcome
    Clearance *Measurement    // a proven lower bound on the gap over the interval; nil unless Outcome is IntervalClear (§5.2)
}

type IntervalOutcome int
// IntervalNotEvaluated | IntervalClear | IntervalColliding | IntervalUndecided

// Collision is a proven overlap at an evaluated pose, about the IDEAL pose the
// parameter names (§5.1): the measured overlap survives the pose's own
// deviation from the ideal motion, and Volume's bound carries that allowance.
type Collision struct {
    At     units.Value   // the parameter of the pose
    Pose   r3.Transform  // Motion.PoseAt(At)
    Moving *Body
    Static *Body
    Volume Measurement   // the overlap volume, Value − Bound a proven lower bound on the ideal overlap; always present
}
```

**Every pair result at a pose is one of `Verify`'s own rows or one of its own pair diagnostics.** A pose's
`Interferences` and `Clearances` are the same two lists verification §1 gives a document, computed for the
(mover, static) pairs alone and by the same kernels; an undecided or unsupported pair at a pose emits the
same `DiagUndecidedPair`, `DiagUndecidedClearance`, `DiagUnsupportedPair*` or `DiagUnsupportedPairSheet` it
would under `Verify`. A `Clearance` row at a pose is a measurement at that pose only; the continuous claim
lives on the interval.

`IntervalOutcome` is a closed set with `String()`, like every enumeration in `report.go`:

| Outcome | Claim |
|---|---|
| `IntervalClear` | for EVERY parameter in the closed interval, every (mover, static) pair has disjoint interiors; `Clearance` is a proven lower bound on the gap over the interval (§5.2) |
| `IntervalColliding` | a proven collision sits at one of its endpoints; nothing is claimed about the interior. Where one endpoint is clear, §6 step 5 has bisected it down to the resolution, so contact onset is bracketed to it |
| `IntervalUndecided` | neither of the above: the certificate did not close and the resolution floor stopped refinement, or a pose it needs was undecided |

`Collisions` is the flat list of every `(PoseResult, Interference)` pair, so a caller that asks only "where
does it hit?" reads it without walking `Poses`. An `Interference` row at a pose and a `Collision` entry are
the same finding, listed in two places on purpose, exactly as `Report.Diagnostics` flattens body findings
(verification §1.1).

### 4.1 Status, assessment and diagnostics

`Status` is verification §6's worst-wins aggregate over the findings below, and `Passed()` is true exactly
when it is `Sound`:

| Finding | Rung | Diagnostic |
|---|---|---|
| a pose proves overlap for some pair and the overlap transfers to the ideal pose (§5.1) | `Interfering` | `DiagMotionCollision` — `Pair` set, `At` set, `Reading` `ReadingOverlapVolume`, `Observed` the transferred volume |
| a pose proves overlap at the float pose that does not transfer: unmeasured, or measured below the `η_k` allowance (§5.1) | `Suspect` | `DiagUndecidedInterference` — `Pair` set, `At` set, `Reading` `ReadingNone` |
| `WithMinClearance` asked and some pose's gap interval lies wholly below the minimum (`hi < minimum`) | `Violating` | `DiagMotionClearanceViolated` — `Pair`, `At`, `Reading` `ReadingGap`, `Observed` the gap, `Required` the minimum |
| an interval is `IntervalUndecided` | `Suspect` | `DiagMotionUndecidedInterval` — `Pair` nil, `At` the interval's `From`, `Reading` `ReadingNone`; `Message` names both ends |
| `WithMinClearance` asked and an interval is `IntervalClear` but its lower bound does not reach the minimum, while no pose falsifies it | `Suspect` | `DiagMotionUndecidedClearance` — `At` the interval's `From`, `Reading` `ReadingGap`, `Observed` the interval's `Clearance`, `Required` the minimum |
| a pose's pair is undecided or unsupported | `Suspect` | that pair's own `Verify` code, with `At` set |
| `MotionReport.Clearance` or a pose's `Clearance.Gap` or `Interference.Volume` is beyond tolerance | `Suspect` | `DiagMeasurementBeyondTolerance`, `At` set for a pose reading and nil for the path reading |

`Assessment` is decided before the numeric gate, by verification §6's order: `AssessmentViolated` when any
pose proves `hi < minimum`, else `AssessmentMet` when every interval is `IntervalClear` with a lower bound at
or above the minimum, else `AssessmentUndecided`. A violated spec is `Violating` even when the gap's bound is
coarse, and a met spec measured beyond tolerance is `Suspect`, as for the wall rule.

**`Diagnostic` gains one field.** `At *units.Value` is the motion parameter a finding concerns; it is nil on
every diagnostic `Verify` emits and on every motion finding about the whole path. It is additive — no
existing field changes meaning — and it is what lets a caller branch on WHERE a finding sits without parsing
`Message`, which verification §1.1 forbids. The four `DiagMotion*` codes join `DiagnosticCode` with stable
lower-snake tokens (`motion_collision`, `motion_clearance_violated`, `motion_undecided_interval`,
`motion_undecided_clearance`). `report.go` owns both additions; verification §1.1 gains the one-line
statement that `At` exists and is nil outside motion reports.

**No fabricated rows.** Verification §6's standard governs here unchanged: an interval the certificate did
not close is `IntervalUndecided` and reads `Suspect`; it is never `IntervalClear` because its endpoints
happened to be clear. `MotionReport.Clearance` is present only when every interval is `IntervalClear`,
because a minimum over the path is a claim about the whole path and an undecided stretch leaves it unknown.

## 5. The proof

### 5.1 What one pose proves

A pose is evaluated by re-evaluating each mover's payload under the composed motion, exactly as
`Body.Placed` does (evaluator §8), and then running the pair kernel of clearance §1–§7 and the overlap proof
of interference §3 over every (placed mover, static) pair. Three rules make it non-mutating and sound:

- **`VerifyMotion` MUST NOT call public `Placed`, `PlacedCopy` or `Duplicate`.** Those register a body and
  advance the producer identity (core §8). The check calls the payload's private `placed` with the composed
  transform and a reserved transient producer identity that never enters the document, and never calls
  `commit`. Interference §2's invariants hold verbatim: `Document.Bodies()` membership and order, every
  body's live/retired state, and the next producer identity are unchanged by a call, successful or not. The
  transient body's provenance values are never published; `PoseResult` carries the `Pose` transform, not the
  transient body.
- **A placed pose carries its placement rounding as `bodyGeom.delta` already.** Evaluator §8 charges
  `frameAndPlacementRoundAllow` into every placed coordinate, and clearance §5 subtracts both bodies' deltas
  from the kernel's `lo` before anything is proven. The transient body is a placed body like any other, so
  the pose's gap interval `[lo_k, hi_k]` already covers the float evaluation of the composed transform.
- **The float pose is not the ideal pose, and the difference is charged.** `Motion.PoseAt(s_k)` returns a
  float `r3.Transform`: `Rotation` evaluates `math.Sincos` and Rodrigues' formula in float, `RotationAround`
  rounds `Center − R·Center`, `Then` rounds the composition, and `units.Value.In(units.Radian)` rounds a
  degree-stated angle through a float `π/180`. The kernel measured the body under the float transform `C_k`
  the payload holds; the claim is about the ideal transform `T*(s_k)∘P0`, where `P0` is the mover's own
  placement and `T*(s_k)` the exact rotation or translation by the stated parameter. For a record point `p`
  with `|p| ≤ R0`, the two images differ by at most `η_k = ‖B(C_k) − B(T*(s_k))·B(P0)‖_F · R0 + |t(C_k) −
  (B(T*(s_k))·t(P0) + t*(s_k))|`, every term a rational interval: the float matrices are read exactly off
  `Basis()`/`Translation()`, the ideal rotation's sine and cosine are enclosed by `turnSinCosInterval`
  (`moments_trig.go`) for a degree-stated angle, which is an exact rational turn, and through
  `rat_interval.go`'s `π` enclosures for a radian-stated one; `R0` is the mover's record-coordinate radius
  read off its payload envelope (`prism_payload.go`'s profile envelopes and the axial extent for a prism, the
  revolve's generator envelope and radius for a revolve). `η_k` is then subtracted from `lo_k` exactly as
  `clearanceDeltaWiden` subtracts a delta, by `downRound`, and `exact` collapses to false whenever it is
  nonzero. It is of order `1e-15 · R0` and never matters to a verdict in practice, which is precisely why it
  must be charged rather than argued away: a bound stops covering a value the moment unproven float ops touch
  it, and the pose is such an op.

**A `Between`'s ideal path is the exact screw of the READ parameters, applied after `From`.** `rel.Screw()`
returns float values for `Axis`, `Point`, `Angle` and `Slide`; the ideal path takes each as the exact
rational the float denotes and composes the exact screw they name onto `From`, itself taken as the exact
rationals of its float entries: `T*(s) = S*(s) ∘ From`, where `S*(s)` rotates by `s·θ` radians about the
line through `c = Point` along `n = Axis/|Axis|` and slides `s·d` along `n`. In `idealPose`'s form
`x ↦ rot·x + shift` that is `rot = R(s·θ, n)·B(From)` and `shift = R(s·θ, n)·(t(From) − c) + c + s·d·n`,
every entry a rational interval: `R`'s sine and cosine come from `radianSinCos` — `s·θ` is a radian value,
never a turn fraction, so it is enclosed through `rat_interval.go`'s π enclosures exactly as a radian-stated
`Revolute` angle is — and `n` carries `unitScaleInterval`'s enclosure of `1/|Axis|`, a point for an
axis-aligned `Axis`. `S*(0)` is the identity exactly (the enclosure of `sin 0, cos 0` is the point pair
`(0, 1)`), so `T*(0)` is `From` itself and the `s = 0` pose of an unplaced mover under `From = Identity()`
has `η_0` exactly zero. `T*(1)` is NOT `To`: the exact screw of the read parameters rebuilds `rel` only to
within r3's rebuild error, about `ε·|t(rel)|` (r3 §7). **The gap between the ideal end and the stated `To`
is CHARGED, not reported and not refused.** `PoseAt(1)` returns `To`, so the kernel measures the mover under
`C_1 = P0 then To`, and the pose charges `η_1 = max(η_ideal, η_To)`: `η_ideal` is `poseDeviation` of `C_1`
against `T*(1)∘P0`, and `η_To` is `poseDeviation` of `C_1` against the exact product `To∘P0` (an `idealPose`
with `rot = B(To)` and `shift = t(To)`, both point intervals). Every gap row and every transferred collision
at `s = 1` therefore holds for the ideal end, which the interval certificate consumes, AND for the stated
`To`, which the caller named. `η_ideal(1)` is of order `ε·|t(rel)|` — about `1e-10` mm for a pivot `1e6` mm
away, `1e-14` mm for coordinates of order ten — and is strictly positive wherever `θ ≠ 0`, because the
enclosures it is built from have nonzero width; `η_To` is the rounding of one `Then`, exactly zero for an
unplaced mover. The path the report speaks for is `T*(s)`, `s ∈ [0, 1]`: a rigid screw motion throughout,
starting at the stated `From` and ending within `η_ideal(1)` of the stated `To`. Nothing is claimed about
any other screw joining the two poses (§5.4).

With those three in place, pose `k` proves, for each pair, one of interference §1's four relations about the
FLOAT pose, and when disjoint or touching a gap interval `[lo_k, hi_k]` — widened by `η_k` on both sides,
which is what `recordGap` does — with `lo_k` a proven lower bound and `hi_k` a proven upper bound on the true
distance between the IDEAL placed mover and the static body.

**A collision is transferred to the ideal pose through the same `η_k`, or it is not a collision.** The
overlap proof runs on the float pose, so a proven overlap there is, by itself, a claim about a transform that
differs from the ideal one by up to `η_k`; publishing it as a collision on the stated path would be a false
falsification whenever the true overlap is thinner than `η_k`. The transfer uses the volume: moving every
boundary point of the mover by at most `η_k` changes the overlap volume by at most the volume its boundary
sweeps, `sweptVolumeAllow(η_k, A)` (`bounds.go`), where `A` bounds the mover's surface area at EVERY point of
the straight path between the float pose's image and the ideal pose's, which is what that helper's contract
asks for. Relative to the mover at rest, a point of that path is `M_t·x + c` with
`M_t = R + (1 − t)·(B(C_k) − R·B(P0))·B(P0)⁻¹` and `R` exactly orthogonal, so `‖M_t‖₂ ≤ 1 + linear/σ` and an
area scales by at most its square: `A = (Area.Value + Area.Bound)·(1 + linear/σ)²`, up-rounded, where
`linear` is `η_k`'s own linear factor `‖B(C_k) − R·B(P0)‖_F` and `σ` a proven lower bound on the smallest
singular value of `B(P0)`, read exactly off its columns as `√(1 − ‖B(P0)ᵀB(P0) − I‖_F)` rounded down. A pose
whose linear part matches the ideal one exactly takes `A = Area.Value + Area.Bound` unscaled. For a
`Between` the ideal linear part is `R(s·θ, n)·B(From)`, which is orthogonal only as far as `B(From)` is:
`r3` admits a basis skewed up to `1e-9`, so `‖R·B(From)‖₂ ≤ σ_max`, where `σ_max` is a proven upper bound on
the largest singular value of `B(From)` — `√(1 + ‖B(From)ᵀB(From) − I‖_F)` rounded up, `basisSigmaLower`'s
mirror — and at `s = 1`, where `η_To` also enters, the larger of `B(From)`'s and `B(To)`'s. The stretch is
then `σ_max + linear/σ`, and the unscaled shortcut becomes `A·σ_max²`. `pathAreaUpper` takes that stretch
base as an argument; a `Revolute` and a `Prismatic` pass exactly `1`, so their allowance is unchanged. A
pose therefore publishes a `Collision` for a pair exactly when the
read-only proof bounded the overlap volume `V` and

```text
V.Value − V.Bound > sweptVolumeAllow(η_k, A)      (the right side up-rounded, the left down-rounded)
```

and the published `Collision.Volume` and `Interference.Volume` carry `V.Bound` widened by that allowance,
so `Value − Bound` stays a proven lower bound on the IDEAL overlap volume. An overlap proven at the float
pose that fails this — an overlap the read-only path could not measure (interference §7.1), or one whose
measured volume does not clear the allowance — is NOT a `Collision`: the pose records
`DiagUndecidedInterference` with `Pair` and `At` set (the pair is proven to overlap at the float pose and its
ideal-pose overlap is unbounded), the pose has no gap for that pair, every interval touching the pose is
`IntervalUndecided`, and the report reads `Suspect`. This is the falsification CLAUDE.md's hard rule
permits, stated about the pose the caller named rather than the one float arithmetic produced, and it is
sound on any payload the read-only overlap proof reaches (§7), whether or not the clearance kernel can model
the pair. Where `η_k` is exactly zero — the parameter `0` under an identity placement, a whole number of
quarter turns about an axis-aligned exact axis — the allowance is zero and every measured positive overlap
transfers unchanged.

### 5.2 What an interval proves

Sampling proves nothing between samples; a thin pin the arm passes between two evaluated poses leaves every
pose clear. The continuous claim rests on one fact and one bound:

**The fact.** The distance between two closed sets is 1-Lipschitz in a displacement of either: if every point
of `M` moves by at most `τ`, then `d(M', S) ≥ d(M, S) − τ`. So on `[s_k, s_{k+1}]`, for every `s`,

```text
d(M(s), S) ≥ max( lo_k − τ_k(s − s_k),  lo_{k+1} − τ_k(s_{k+1} − s) )
```

where `τ_k(Δ)` bounds how far any point of the mover travels over a parameter change `Δ`.

**The bound.** Every term of it is an exact rational (`motion_bound.go`), so no float rounding sits between
a bound and the comparison it feeds. A motion parameter denotes `θ = 2π·turn + base`: a degree-stated angle
is the exact rational turn `deg/360` (the degree count is what the caller stated; `units.Degree`'s factor is
a rounded `π/180` and is never used), any other angle unit is `magnitude × factor` radians read exactly, and
a length is `base` millimetres, and a `Between` fraction is `base`, dimensionless. The span of an interval is
`2π·|Δturn| + |Δbase|` with `π` at its upper enclosure (`rat_interval.go`), exact for a length and for a
fraction. For a `Prismatic`, every point travels exactly the span along the unit direction, so `τ(Δ)` is the
span. For a `Revolute`, a point at distance `ρ` from the axis travels an arc of length `ρ·|Δθ|`, which
bounds its chord, so `τ(Δθ) = ρ_max × span`, where `ρ_max` bounds the distance from the axis of every point
of the mover. For a `Between`, a point at distance `ρ` from the screw axis rotates through an arc of length
`ρ·|Δs|·θ` and slides `|Δs|·|d|` along the axis; the two displacements are perpendicular, and their sum
bounds their resultant, so

```text
τ(Δs) = |Δs| × (ρ_max × θ + |d|)      (θ and d the exact rationals of the floats rel.Screw() returned; no π enters)
```

The Pythagorean form `|Δs|·√((ρ_max·θ)² + d²)` is tighter by at most a factor `√2` and needs a root;
refinement absorbs the difference, so the sum is used. `ρ_max` is read ONCE, from the mover's `Bounds()` at
its current placement, the parameter `0`: the box is inflated outward by its own `Bound` and read as exact
rational extremes, each of its eight corners' squared distance from the axis line, `|(x − c) × a|² / |a|²`,
is taken exactly, the largest is rooted upward by `ratSqrtUp`; distance from a line is convex, so the maximum
over the box sits at a corner. For a `Between` the path starts at `From`, not at rest, and the screw axis —
the line through `Point` along `Axis` — in general passes nowhere near the origin, so the eight inflated
corners are first mapped through `From` exactly, `x' = B(From)·x + t(From)` over rationals (the ideal
`T*(0)`), and each image's squared distance from that line is what is taken; the moved body lies inside the
convex hull of the eight images, and distance from a line is convex, so the maximum sits at an image. A
rotation about the axis and a slide along it both preserve every point's distance from it, so one reading
covers every pose.

**The certificate.** The interval is `IntervalClear` for a pair when the two one-sided bounds together cover
it with strictly positive distance everywhere, which holds exactly when

```text
lo_k + lo_{k+1} > τ_k(s_{k+1} − s_k)      (compared over exact rationals; lo_k, lo_{k+1} are the floats the poses proved)
```

The two sides meet where `lo_k − τ(s − s_k) = lo_{k+1} − τ(s_{k+1} − s)`, and the lower envelope's minimum
over the interval is `(lo_k + lo_{k+1} − τ_k)/2`, down-rounded — that is the interval's `Clearance` lower
bound. The envelope is strictly positive on the whole closed interval, so the boundaries never meet, so no
boundary crossing occurs and the pair's relation cannot change from the disjoint one proven at an endpoint:
a nesting cannot begin without the boundaries crossing, and a touching cannot occur with positive distance.
The interval is `IntervalClear` only when EVERY (mover, static) pair certifies.

Three consequences are deliberate:

- **A touching endpoint can never certify its interval.** A swing that begins resting on a stop has
  `lo_0 = 0`, an `Exact` zero (clearance §6). The far pose's gap is at most `τ_0`: the point of contact at
  `s_0` travels at most `τ_0` to reach `s_1`, so the mover there is within `τ_0` of the static body, so
  `lo_1 ≤ τ_0` and `lo_0 + lo_1 ≤ τ_0` — the strict inequality can never hold, from either side. The
  interval is `IntervalUndecided` and the report `Suspect`. This is not a weakness of the two-sided form; it
  is what the claim means: `IntervalClear` asserts a positive gap at EVERY parameter of the closed interval,
  and the interval contains a pose at zero distance. Proving that a body lifts cleanly off a resting contact
  needs a direction-of-motion argument — that the contact point's velocity points out of the static body and
  stays so — and this design makes no such argument. A caller who wants the swing certified states a `From`
  strictly past the contact, at which point the certificate applies unchanged. The two-sided form is kept
  because it is the tight form of the Lipschitz bound and halves the refinement an asymmetric gap needs; it
  is not kept for the touching case.
- **`WithMinClearance` is the same certificate at a higher level.** The margin `m` is met over the interval
  when `(lo_k + lo_{k+1} − τ_k)/2 ≥ m`, down-rounded; it is falsified at a pose when `hi_k < m`, up-rounded.
  Only those two directions exist: a lower bound can prove the margin, an upper bound can disprove it, and
  neither does the other's job.
- **A proven overlap at an endpoint ends the argument.** The interval is `IntervalColliding`, and no bound
  is computed over it: a collision has been found, which is what the caller asked.

Everything the certificate consumes is a proven bound already in hand or a closed-form up-rounded term; it
adds no new geometric proof. The one new proof piece is `η_k` (§5.1), and the one new reading is `ρ_max`.

### 5.3 What the whole path proves

When every interval is `IntervalClear`, the minimum gap over the path lies in `[min_k lower envelope,
min_k hi_k]`: the lower end is the smallest interval `Clearance`, the upper end the smallest evaluated
`hi_k`, because a pose's upper bound bounds the path's minimum from above. `MotionReport.Clearance` reports
the midpoint and half-width of that interval as a `ScalarReading`, judged by verification §2 against the
pair diameter of the pair that attained `min_k hi_k`. It is `Approximate` whenever either end came from a
rounded term, which is always, so it is never `Exact`. A pair settled by swept-box exclusion alone (§6)
contributes a lower bound and no upper bound; a path on which every pair was so excluded carries no
`Clearance` reading and reads `Sound` without one, exactly as a document of zero pairs answers the empty
list (clearance §7).

### 5.4 What is never claimed

- Nothing is claimed about a parameter outside `[From, To]`.
- Nothing is claimed between two movers (§3).
- An `IntervalUndecided` claims nothing: not clear, not colliding. A report with one is `Suspect`, never
  `Sound`, and the pin the arm passed between samples is reported as "not proven clear" rather than missed.
- A `Collision` claims overlap at that parameter and nothing about the interval around it; the first
  parameter at which contact begins is bracketed by the adjacent `IntervalUndecided` or `IntervalClear`
  records, to the resolution, never reported as a number.
- No tolerance decides admission. Every comparison above is a strict inequality on bounds rounded against
  the claim; a bound that fails by one ulp fails.
- Nothing is claimed about any path but the ideal one of §5.1. For a `Between` that is the exact screw of
  the read parameters composed onto `From`; the exact screw joining `From` to `To` differs from it by
  amounts this design does not bound, and only the stated `To` itself, at `s = 1`, is covered beyond it.

## 6. The refinement procedure

Evaluation is deterministic bisection on a dyadic grid, so a replay reproduces the same poses, the same
verdicts and the same report (evaluator §8):

1. **Validate** (§3, §8) before reading `ctx`.
2. **Read every mover's `ρ_max`** (§5.2) for a `Revolute` or a `Between`; a `Prismatic` needs none.
3. **Swept-box exclusion.** The mover's `Bounds()` box is read at REST, the parameter `0`, not at `From`, so
   the travel that inflates it runs from rest to the farther endpoint: `max(τ(0 → From), τ(0 → To))`.
   Inflating by `τ(To − From)` alone is unsound whenever `From ≠ 0` — an arm swinging from 80° to 90° has
   moved 80° before the path begins, and a box grown by 10° of travel excludes a wall it is already inside.
   For each (mover, static) pair, the mover's box inflated outward by its `Bound` plus that travel, and the
   static body's by its `Bound`, are compared as exact rational extremes per axis; a strictly positive gap
   along some axis proves the pair apart at every parameter, and the largest such gap, rounded down, is a
   proven lower bound on the pair's distance over the whole path. The pair is never evaluated at any pose and
   contributes to every interval as `IntervalClear` (§5.3). This is the same Lipschitz fact as §5.2 applied
   once, and it is what keeps a large document cheap when the mover is far from most of it. A `Between`'s
   path starts at `From`, which the rest box has not undergone, so its box is the **From-placed box**: the
   axis-aligned hull, per axis the exact rational minimum and maximum, of the eight inflated rest-box
   corners mapped through `From` exactly — the same corner images `ρ_max` reads (§5.2) — inflated by the
   whole path's travel `τ(0 → 1) = ρ_max·θ + |d|` and by nothing else, since the corners already carry the
   box's `Bound`. Inflating the rest box by the path's travel alone is unsound whenever `From` is not the
   identity: the mover has moved by `From` before the path begins, and a wall beside its START is excluded
   by a box that never left its resting place.
4. **Evaluate the endpoints** `From` and `To` (§5.1) for every remaining pair.
5. **Bisect for the verdict.** Take the first interval, in traversal order, whose width exceeds the
   resolution and that is either undecided — neither `IntervalClear` nor `IntervalColliding` — or
   `IntervalColliding` with one endpoint that holds no collision; evaluate its midpoint `(s_a + s_b)/2` —
   exact in float because the grid is dyadic in `(To − From)` — and replace the interval by its two halves.
   Repeat until no such interval remains. An interval whose width is at or below the resolution and that
   still fails the certificate becomes `IntervalUndecided`. The colliding case is what brackets contact
   onset: a path whose far endpoint collides is otherwise one `IntervalColliding` interval that says nothing
   about where the overlap began, and halving the colliding interval that still has a collision-free end
   walks the first collision down to within one resolution step of the onset. An interval colliding at both
   ends is not split: the overlap's interior is not the question.
6. **Bisect for the reading.** A certified interval's `Clearance` sits below the true minimum by up to
   `τ_k/2` — for the arm of §9, `25 mm × Δθ` — so an interval that certifies at a coarse width leaves the
   path reading's half-width (§5.3) at that scale, and a margin that the true gap meets by less than
   `τ_k/2` reads undecided. Refinement therefore continues past certification, on certified intervals, with
   two stopping rules, each bounded by the resolution floor: while `MotionReport.Clearance` would fail the
   tolerance gate, bisect the interval holding the smallest `Clearance` (ties in traversal order) unless its
   width is at or below the resolution; and while `WithMinClearance` is neither proven (every interval's
   `Clearance ≥ minimum`) nor disproven (some pose's `hi < minimum`), bisect the interval holding the
   smallest `Clearance` on the same terms. A midpoint evaluated here is an ordinary pose: it can disprove the
   margin, find a collision, or leave the interval undecided, and each of those stands. When the floor stops
   the loop, the reading is published beyond tolerance and reads `Suspect`, or the margin reads
   `AssessmentUndecided` — the honest coarse answer of clearance §7, never a tightened number.
7. **Publish** (§4), with `Poses` in parameter order including every midpoint inserted.

The pose count is bounded by the floor: at most `|To − From| / Resolution + 1` poses on the path, `1025` at
the default, each running every non-excluded pair through the kernel once. Step 5 spends poses wherever
the certificate fails, which near a collision or a close approach is down to the floor; bracketing a
contact's onset costs about `log₂(|To − From| / Resolution)` further poses per contact, one per halving of
the colliding interval that still has a collision-free end; step 6 spends them
only around the current minimum, so its cost grows with the logarithm of `1 / Resolution` rather than with
the path length. A caller who wants only the verdict and not the figures states a coarse `WithResolution`;
a caller who wants a path reading at the gate states one fine enough that `ρ_max × Resolution / 4` — for a
`Between`, `(ρ_max·θ + |d|) × Resolution / 4` — is below `rel × gap` (§9 tests 2 and 13 work the
arithmetic).

A pose whose pair is undecided or unsupported (a payload the kernel cannot model, an uncertified contact)
offers no `lo`, so no interval touching it can certify through that endpoint; the far endpoint may still
close it. A pose at which some pair is undecided and no interval around it closes leaves those intervals
`IntervalUndecided` after the resolution is reached, with that pose's own pair diagnostic beside them.

`ctx` is checked before every pose evaluation and inside the kernel as clearance §3 and interference §7.2
already check it; the shared `workBudget` is passed through. A cancelled call returns `ctx.Err()` and no
report. Each pose runs the full pair kernel, so the cost is the number of poses evaluated times the
document's per-pair cost (verification §1.2); the resolution bounds the pose count at
`|To − From| / Resolution + 1` in the worst case, and the swept-box step removes most pairs from every pose.

## 7. Coverage

The proof path is capability-based, as interference §9 states it, and the motion check inherits each
payload's reach without adding a weaker one:

| Mover or static payload | Pose relation and gap | Collision proof | Over the path |
|---|---|---|---|
| mover: unplaced or placed `prismPayload` with zero section displacement, or `revolvePayload`; static: the same, or a zero-vertex-bound closed `stitchPayload` | analytic kernel (clearance §2); the mover's record radius `R0` (§5.1) is read off its prism or revolve envelope, so `η_k` is bounded under every motion | kernel overlap, or read-only intersection, transferred through `η_k` | `IntervalClear` reachable |
| mover: zero-vertex-bound closed `stitchPayload` | analytic kernel, but the payload states no `R0`: where the pose's linear part matches the ideal one exactly — under a `Prismatic`, and under a `Between` whose `From` and `To` are both translations, so the read `θ` is `0` and `B(From)` the identity — `η_k` is the translation term alone and the gap is measured; under a `Revolute`, and under any other `Between`, `η_k` is unbounded, the pair reads `DiagUndecidedClearance` at every pose, and no collision transfers | read-only intersection, where the linear parts match only | `IntervalClear` reachable where the linear parts match; every interval `IntervalUndecided` otherwise unless swept-box exclusion settles the pair |
| `prismPayload` with nonzero section displacement, `cupPayload`, `facetedPayload`, `loftPayload`, `capBlendPayload` | box separation only; gap `Suspect` (clearance §8) | read-only intersection where the payload tessellates (interference §9) | collisions found; intervals `IntervalUndecided` unless swept-box exclusion settles the pair |
| a `BodySheet` operand on either side | none | none | the pair reads `DiagUnsupportedPairSheet` at every pose, intervals `IntervalUndecided`, unless swept-box exclusion settles it |
| a mover this evaluator did not build (`payload == nil`) | — | — | `ErrUnsupported` at the call, as for `Placed` |

A mover or static body that is not a proven solid is treated as `Verify` treats it (interference §2): its
pairs are not evaluated, every interval is `IntervalUndecided`, and the body's own validity diagnostic is
carried in each pose's `Diagnostics`. The motion check never widens a payload's reach; a pair the clearance
kernel leaves `Suspect` under `Verify` leaves every interval it touches `IntervalUndecided` here.

## 8. Errors

`VerifyMotion` returns `(*MotionReport, error)` by core §12: the error is for a call that could not be made,
and a report is returned only when the check ran.

| Condition | Error |
|---|---|
| `moving` empty, or a body listed twice | `ErrDegenerate` |
| a mover retired | `ErrRetiredBody` |
| a mover of another document | `ErrForeignBody` |
| a mover this evaluator did not build | `ErrUnsupported` |
| `From == To` (a `Between` included, compared with `==`), or a `Between` whose relative motion is the zero screw | `ErrDegenerate` |
| `Axis`/`Dir` with no direction | `ErrDegenerate` |
| a `Between` `From` or `To` with a non-finite component (kept as a guard; `r3`'s public API cannot produce one, so no test reaches it) | `ErrNotFinite` |
| a `Between` `From` or `To` that is not a rigid motion (`Transform.IsValid` false, the zero `r3.Transform{}` included) | `ErrDegenerate` |
| a `Between` joining a reflection to a proper motion (`r3.ErrImproper`) | `ErrDegenerate` |
| a `Between` whose screw point or slide, or whose pose at some `s`, `r3` cannot represent (`r3.ErrNonFinite`) | `ErrNotFinite` |
| a wrong-`Kind` `From`, `To`, resolution, tolerance or minimum | `ErrUnitKind` |
| a non-finite parameter, vector component, resolution, tolerance or minimum | `ErrNotFinite` |
| a negative or zero resolution, a negative tolerance or minimum | `ErrNegativeMagnitude` |
| a zero minimum, which no gap can fall below (`WithMinWallThickness`'s rule for a zero tool) | `ErrDegenerate` |
| `ctx` cancelled after validation | `ctx.Err()` unchanged |
| an invariant failure inside a pose's kernel or overlap proof (interference §7.1) | that error, no report |

An undecided pair, an unsupported payload, a sheet, a resolution floor reached — none of these is an error.
Each is a finding in the report and reads `Suspect`.

## 9. Required tests

Every test asserts on computed geometry and on the production path, never on a local model of it
(CLAUDE.md "Correctness must be observable"). Fixtures are dyadic where a value is exact, and every bound is
asserted negligible or `InDelta` at a stated slack, never pinned to a literal, because FMA contraction moves
the last ulp between amd64 and arm64. Each test that guards a bound term records in a comment which legs
were deleted and seen to fail, and which are proven redundant with the argument written out.

**The arm.** Three fixtures share one mover: a prism extruded 10 mm from the XY-plane rectangle
`x ∈ [0, 48], y ∈ [−14, 14]`, revolving about the Z axis through the origin from `0°` to `90°`, so its
farthest corner `(48, 14)` sits at exactly `50` mm from the axis and at polar angle `atan(7/24)`.

1. **Known collision angle.** Static wall: a prism occupying `x ∈ [−100, 100], y ∈ [40, 60], z ∈ [−10, 30]`,
   reaching past the arm's caps at `z = 0` and `10` on both sides so the two bodies share no face plane — the
   read-only overlap proof refuses a contact across a shared plane, and an overlap it cannot measure is not
   a collision (§5.1). The corner `(48, 14)` reaches `y = 40` when `sin(θ + atan(7/24)) = 4/5`, that is at
   `θ* = atan(3/4) ≈ 36.87°`, and every other point of the arm reaches it later. The `90°` endpoint
   collides, so the first collision near `θ*` is found by §6 step 5's onset bisection of the colliding
   interval; deleting that bisection leaves the first collision at `90°` and turns this test red. Assert: `Status` is
   `Interfering`; `Collisions` is non-empty; every `Collision.At` is strictly greater than `θ*`; every
   `IntervalClear` interval ends at or below `θ*`; the interval containing `θ*` is not `IntervalClear`; with
   `WithResolution(0.25°)` the first `Collision.At` is within `0.5°` above `θ*`; every `Collision.Volume` is
   positive with its `Bound` below its `Value`. Assert the same under a `Revolute` stated in radians.
2. **Clear swing with a stated margin.** Static wall at `y ∈ [60, 80]`. The minimum gap over the path is
   exactly `10` mm, at `θ = 90° − atan(7/24)`. The two endpoint gaps are `46` mm at `0°` and `12` mm at
   `90°`; their sum, `58` mm, is under the whole swing's travel `50 × π/2 ≈ 78.54` mm, so the endpoints
   alone certify nothing, and the test has two parts. **Endpoints only** (no bisection): assert `Status` is
   `Suspect`, the one interval is `IntervalUndecided`, `Collisions` is empty, `Clearance` is nil, the `0°`
   row is an `Exact` `46` and the `90°` row encloses `12` with an ulp-scale bound. **Bisected**, with
   `WithResolution(0.01°)`: a certified interval's `Clearance` dips below the true gap by up to
   `τ_k/2 = 25 mm × Δθ`, so the path reading's half-width is about `12.5 mm × Δθ`, and the gate at
   `rel = 1e-3` on a `10` mm gap admits `0.01` mm; `Δθ = 0.01° ≈ 1.75e-4 rad` gives `2.2e-3` mm, inside it,
   while the default floor `90°/1024` gives `0.019` mm, outside it. Assert: `Status` is `Sound`; every
   interval is `IntervalClear`; `MotionReport.Clearance.Value` is within `0.1` mm of `10` with
   `Tolerance.State` `ToleranceSatisfied`; with `WithMinClearance(9 mm)` the `Assessment` is
   `AssessmentMet`; with `WithMinClearance(11 mm)` it is `AssessmentViolated` and `Status` is `Violating`
   with a `DiagMotionClearanceViolated` whose `At` is the pose of the proving gap; with
   `WithMinClearance(10 mm)` exactly, the `Assessment` is `AssessmentUndecided` and the report `Suspect` —
   never `Met`, because no proven lower bound can reach an exact minimum. Assert also that at the default
   resolution the same fixture is `Sound` in verdict but its `Clearance` reading is beyond tolerance, so the
   report reads `Suspect` with a `DiagMeasurementBeyondTolerance` on `ReadingGap` and a nil `At`.
3. **Near miss between samples.** Mover: a blade `x ∈ [0, 50], y ∈ [−0.5, 0.5]`, same motion. Static: a pin,
   an axis-aligned `0.8` mm cube, `z ∈ [4.6, 5.4]`, centred at polar angle `α = 90·31/64 = 43.59375°`,
   radius `49`. The window is worked out as follows. With the blade at angle `θ`, the pin's centre sits
   `49·|sin(θ − α)|` from the blade's mid-plane, and the pin reaches toward that plane by its support
   `0.4·(|sin θ| + |cos θ|) ≤ 0.4·√2 ≈ 0.566` mm. The blade's half-thickness is `0.5` mm, so contact needs
   `49·|sin(θ − α)| ≤ 1.066`, that is `|θ − α| ≤ 1.247°`: the contact window lies inside
   `[42.34°, 44.85°]`, under `2.5°` wide. At `WithResolution(3°)` refinement stops at width
   `90°/32 = 2.8125°`, so the only poses it can reach are multiples of `2.8125°`, and the two nearest `α`,
   `42.1875°` and `45°`, each sit `1.40625°` from it — outside the window by a gap of at least
   `49·sin(1.40625°) − 1.066 ≈ 0.137` mm. A `1` mm pin would not do: its support at `45°` is `0.707` mm,
   the window reaches `±1.411°`, and the `45°` grid point lands inside it. With `WithResolution(3°)`
   assert: no `Collision`; `Status` is `Suspect`, never `Sound`; the `IntervalUndecided` interval contains
   `α`. With `WithResolution(0.1°)` — a floor of `90°/1024 ≈ 0.088°`, whose grid includes `α` itself —
   assert: `Status` is `Interfering` and the first `Collision.At` lies within `[α − 1.247°, α + 1.247°]`.
   This fixture is the one that goes red when the `ρ_max·Δθ` term is deleted or when `ρ_max` is read from
   the centroid instead of the box: either lets a coarse interval spanning the pin certify. Making the
   certificate one-sided — `lo_k > τ_k` or `lo_{k+1} > τ_k` alone — is not a leg any fixture can fail:
   either one-sided form implies the two-sided `lo_k + lo_{k+1} > τ_k`, so it certifies fewer intervals,
   never more, and costs refinement rather than soundness; the test file records that argument.
4. **Touching start.** The arm with a stop prism sharing its `y = −14` face plane at `θ = 0` (a stop-built
   extrude, clearance §6's coplanar certificate), swinging away. Assert: the first `PoseResult.Clearances`
   row is an `Exact` zero with a zero bound (the `0°` pose is the identity motion, so the transient arm
   carries no placement rounding and `η_0` is exactly zero); `Collisions` is empty, because touching is not
   overlap; the first interval is `IntervalUndecided`; `Status` is `Suspect`. The pinned outcome is §5.2's:
   an interval that contains a zero-distance pose is never `IntervalClear`. This fixture goes red when the
   exact sine and cosine of a whole number of quarter turns is deleted (the identity pose then carries an
   enclosure-width `η` and its row is no longer `Exact`).
5. **Prismatic.** A 10 mm cube translating along `+X` from `0` to `30` mm. **Hit**: a wall whose face is at
   `x = 25`; assert a collision with every `Collision.At` strictly greater than `15` mm, every `IntervalClear`
   ending at or below `15` mm, the last interval `IntervalColliding`, and the last collision's volume within
   `1e-6` of `500` mm³ (the cube at `[30, 40]` against the wall's `[25, 35]`). **Clear, evaluated**: an
   L-shaped static prism whose bar face stands at `x = 45` and whose arm along `y ∈ [50, 60]` reaches back
   over the cube's resting place, so the wall's box meets the cube's swept box and the pair is evaluated
   rather than excluded while the arm's own `40` mm gap never becomes the minimum. The cube's gap to the
   bar shrinks at exactly the travel rate, so the certificate's lower envelope meets the far endpoint's gap
   and the endpoints alone certify; assert a `Sound` report, one `IntervalClear` interval whose `Clearance`
   is at most `5` and within `0.01` mm of it, and a path `Clearance` within `0.01` mm of `5` with
   `ToleranceSatisfied`. **Clear, excluded**: a plain slab at `x ∈ [45, 55]`; its box lies outside the
   cube's resting box grown by the whole travel, so the pair is settled by swept-box exclusion alone. Assert
   a `Sound` report, no pose row for the slab, one `IntervalClear` interval whose `Clearance` is positive
   and at most `5`, and a nil path `Clearance` (§5.3: no pose measured an upper bound). **Pin between
   samples**: a `2 × 2 × 2` mm pin `4` mm ahead of the resting cube, `14` mm behind it at the end; with the
   endpoints alone, assert no `Collision`, `IntervalUndecided`, `Suspect`.
6. **Swept-box exclusion.** Fixture 2 plus a far body at `x ∈ [500, 510]`. Assert the far pair appears in
   `Against`, in no `PoseResult` row, and that the report's `Status` and `Clearance` are unchanged from
   fixture 2.
7. **Non-mutation and determinism** (interference §10.1). Before and after each call: `Document.Bodies()`
   identical in membership and order; every body's live state unchanged; a `Duplicate` after the call
   receives the producer identity it would have received before it. Two calls on the same inputs return
   reports equal in every field.
8. **Pose deviation is charged.** For a `Revolute` stated in degrees with `Center` at `(1e6, 0, 0)`, assert
   that the pose's `Clearance.Gap.Bound` exceeds the same pose's bound with `Center` at the origin, and that
   both are below the tolerance gate for a 50 mm body. The leg is small by construction; the test shows it is
   nonzero and sized to the pivot, which is what deleting it would zero.
9. **Errors.** One subtest per row of §8's table, each asserting `errors.Is` and that the document is
   unchanged.
10. **Cancellation.** A context cancelled during bisection returns `ctx.Err()` and a nil report, with the
    document unchanged.
11. **Example.** `examples/` gains `Example_decad_motionCheck`, fixture 1 at `WithResolution(0.25°)`, whose
    `// Output:` prints the first collision parameter to two decimals; the grid is dyadic in `90°`, so the
    printed value is the same on every platform.

**The screw arm.** Tests 12, 13, 17, 20 and 21 share one `Between`: the arm of tests 1–2 under
`Between{From: r3.Identity(), To: RotationAround(origin, Z, 90°).Then(Translation(0, 0, 20))}`, a quarter
turn about Z with a `20` mm rise. Its read screw is `Axis (0, 0, 1)`, `Point (0, 0, 0)`, `Angle` the float
nearest `π/2` and `Slide 20`, and `From.Then(screw.At(1))` reproduces `To` bit for bit, so `η_1` is of
enclosure-width scale. The rise leaves every `y` unchanged, so every angle test 1 and test 2 work out holds
at the fraction `s = θ/(π/2)`; the walls are raised to `z ∈ [−10, 40]` so they reach past the arm's caps at
every height the rise visits. Every `Between` resolution below is a `units.Scalar`; a `Scalar(1)` evaluates
the endpoints alone.

12. **Known collision on a screw.** **Rotation-led**: the screw arm against test 1's wall raised. The corner
    `(48, 14)` reaches `y = 40` at `θ* = atan(3/4)`, so at `s* = (2/π)·atan(3/4) ≈ 0.40967`. Assert: `Status`
    is `Interfering`; every `Collision.At` has `Kind` `Dimensionless` and is strictly greater than `s*`;
    every `IntervalClear` interval ends at or below `s*`; the interval containing `s*` is not
    `IntervalClear`; with `WithResolution(Scalar(1.0/256))` the first `Collision.At` is within `2/256` above
    `s*`; every `Collision.Volume` is positive with its `Bound` below its `Value`; the `s = 1` pose's `Pose`
    equals `To` with `==`, and its collision volume is within `1e-6` of `2240` mm³ (at `s = 1` the arm spans
    `x ∈ [−14, 14]`, `y ∈ [0, 48]`, `z ∈ [20, 30]`, and the overlap with the wall is `28 × 8 × 10`).
    **Slide-led**: a floor `x, y ∈ [−100, 100]`, `z ∈ [−40, −15]`, and
    `To = RotationAround(origin, Z, 90°).Then(Translation(0, 0, −20))`, descending. The arm's underside
    `z = 0` reaches the floor at slide `−15`, `s = 3/4` exactly, a grid point where the two share a face plane
    and touch without overlapping. Assert: `Interfering`; every `Collision.At` strictly greater than `3/4`;
    with `WithResolution(Scalar(1.0/256))` the first within `2/256` above it; no `IntervalClear` interval
    ends above `3/4`. The slide-led
    fixture goes red when the slide is dropped from the ideal pose: `η` then grows to `|s·d|`, `15` mm at the
    floor, no overlap clears the allowance, and the report reads `Suspect`.
13. **Clear screw path with a stated margin.** The screw arm against test 2's wall raised. The minimum gap is
    `10` mm at `s₁ = 1 − (2/π)·atan(7/24) ≈ 0.81933`; the endpoint gaps are `46` at `s = 0` and `12` at
    `s = 1`, their sum `58` under the whole path's travel `τ(0 → 1) = 50·π/2 + 20 ≈ 98.54` mm, so the
    endpoints alone certify nothing. **Endpoints only** (`WithResolution(Scalar(1))`): assert `Suspect`, one
    `IntervalUndecided`, no `Collisions`, nil `Clearance`, the `s = 0` row an `Exact` `46` (`From` is the
    identity and the mover unplaced, so `η_0` is exactly zero), the `s = 1` row enclosing `12` with an
    ulp-scale bound. **Bisected**: a certified interval's `Clearance` dips below the true gap by up to
    `τ_k/2 = (50·θ + 20)/2 × Δs ≈ 49.27 × Δs` mm, so the path reading's half-width is about `24.6 × Δs`; the
    gate at `rel = 1e-3` on a `10` mm gap admits `0.01` mm; `Δs = 1/4096` gives `6.0e-3` mm, inside it,
    while the default `1/1024` gives `0.024` mm, outside. With `WithResolution(Scalar(1.0/4096))` assert:
    `Sound`; every interval `IntervalClear`; `Clearance.Value` within `0.1` mm of `10` with
    `ToleranceSatisfied`; `WithMinClearance(9 mm)` gives `AssessmentMet`; `11 mm` gives
    `AssessmentViolated`, `Violating` and a `DiagMotionClearanceViolated` whose `At` is the proving pose;
    `10 mm` exactly gives `AssessmentUndecided` and `Suspect`. At the default resolution assert the verdict
    `Sound` but the reading beyond tolerance: `Suspect`, with a `DiagMeasurementBeyondTolerance` on
    `ReadingGap` and a nil `At`. This fixture goes red when the ideal pose omits the slide or reads `θ` in
    anything but radians: `η` then exceeds the gaps and no interval certifies.
14. **Near miss between samples under a screw.** Test 3's blade and pin under
    `Between{Identity(), RotationAround(origin, Z, 90°)}`. The pin's angle `α = 90·31/64°` is the fraction
    `s_α = 31/64`, a
    depth-6 grid point, and test 3's contact window `±1.247°` is `±0.013856` in `s`. With
    `WithResolution(Scalar(1.0/30))` (3° of the quarter turn) refinement stops at width `1/32`, whose grid
    points `15/32` and `16/32` each sit `1/64` from `s_α` — `1.40625°`, outside the window by at least
    `0.137` mm as test 3 works out. Assert: no `Collision`; `Status` is `Suspect`, never `Sound`; the
    `IntervalUndecided` interval contains `31/64`. With `WithResolution(Scalar(1.0/900))` (a floor of
    `1/1024`, whose grid includes `31/64`) assert `Interfering` and the first `Collision.At` within
    `[31/64 − 0.013856, 31/64 + 0.013856]`. This is the fixture that goes red when the `ρ_max·θ` term is
    deleted from a `Between`'s `τ`.
15. **Offset screw axis.** Test 3's blade under
    `Between{From: Translation(50, 0, 0), To: Translation(50, 0, 0).Then(RotationAround((−50, 0, 0), Z, 90°))}`:
    the blade starts at `x ∈ [50, 100]` and swings about the Z axis through `(−50, 0, 0)`, so its tip is
    `150` mm from the screw axis; the read screw has `Point` within `1e-9` of `(−50, 0, 0)`, `Angle` the
    float nearest `π/2` and `Slide 0`. The pin is a `0.8` mm cube centred at
    `(−50 + 149·cos α, 149·sin α, 5)`, `α = 90·31/64°`, `149` mm from the axis. Its centre sits
    `149·|sin(θ − α)|` from the blade's mid-plane, so the contact window is
    `|θ − α| ≤ asin(1.066/149) ≈ 0.41°`; the depth-5 grid points `15/32` and `16/32` are `1.40625°` from
    `α` and clear of the pin by
    `149·sin(1.40625°) − 1.066 ≈ 2.59` mm each. Over a width-`1/32` interval the travel with the true
    `ρ_max ≈ 150` is `150·(π/2)/32 ≈ 7.36` mm, above the two gaps' sum `5.18`, so the interval stays
    undecided; a `ρ_max` of `100` — what the REST box about the true axis reads (`x ∈ [0, 50]` against
    `x = −50`), and what the From-placed box about an axis through the ORIGIN reads (`x ∈ [50, 100]`) —
    gives `4.91` mm and falsely certifies it. With `WithResolution(Scalar(1.0/30))` assert: no `Collision`;
    `Suspect`; the `IntervalUndecided` interval contains `31/64`. With `WithResolution(Scalar(1.0/900))`
    assert `Interfering` and the first `Collision.At` within `[31/64 − 0.41/90, 31/64 + 0.41/90]`. This
    fixture goes red when `ρ_max` is read from the rest box instead of the From-placed corners, or about the
    axis through the origin instead of through `Point`.
16. **A pure translation agrees with `Prismatic`.** Test 5's cube under
    `Between{Identity(), Translation(30, 0, 0)}` and under `Prismatic{Dir: X, From: 0 mm, To: 30 mm}`, on
    test 5's three static fixtures: the
    wall at `x = 25`, the L-shaped clear body, and the pin between samples with the endpoints alone
    (`WithResolution(Scalar(1))` against `WithResolution(30 mm)`). The read screw is `Axis (1, 0, 0)`,
    `Angle 0`, `Slide 30`, `Point 0`: the ideal rotation is the identity exactly, the ideal shift
    `s·30·(1, 0, 0)` is exact, and `screw.At(s)` builds the translation `(30·s, 0, 0)` exactly for every
    dyadic `s`, so the float poses, the kernel results and `η` (zero on both sides) coincide. Assert, pair
    by pair: equal `Status`; equal pose counts; interval outcomes equal in order and each interval
    `Clearance.Value` equal within `1e-9` mm; each `Collision.At` equal under `s ↦ 30·s` mm within `1e-9` mm
    and each `Collision.Volume.Value` within `1e-9` mm³; the path `Clearance.Value` within `1e-9` mm, or nil
    on both. The pin subtest goes red when the slide term is deleted from a `Between`'s `τ`: the read `θ` is
    `0`, `τ` collapses to zero, and the endpoints falsely certify the interval.
17. **A pure rotation agrees with `Revolute`.** The arm under
    `Between{Identity(), RotationAround(origin, Z, 90°)}` and under test 1's `Revolute`, on test 1's wall
    (`WithResolution(Scalar(1.0/256))` against
    `WithResolution(90°/256)`), on test 2's wall bisected (`Scalar(0.01/90)` against `0.01°`: both floors
    stop the dyadic grid at depth 14) and on test 4's stop. The read screw's `Angle` is the float nearest
    `π/2`, not the exact quarter turn the `Revolute` denotes, so the two ideal paths differ by about `6e-17`
    rad and their bounds by about `3e-15` mm, twelve orders below the fixtures' margins. Assert: equal
    `Status`; interval outcomes equal in order; each `Collision.At` equal under `s ↦ 90·s` degrees within
    `1e-9°`; the path `Clearance.Value` within `1e-9` mm, or nil on both; on the stop fixture the `s = 0` row
    an `Exact` zero with a zero bound, as test 4's `0°` row is.
18. **The swept box is the From-placed box.** Test 5's cube, at rest at `x ∈ [0, 10]`, under
    `Between{From: Translation(100, 0, 0), To: Translation(130, 0, 0)}`, so it travels `x ∈ [100, 140]`;
    static bodies: a
    wall `x ∈ [125, 135]` spanning the cube in `y` and `z`, a slab `x ∈ [20, 30]` beside the cube's RESTING
    place, and a far slab `x ∈ [500, 510]`. The cube's leading face reaches the wall at `s = 1/2` exactly, a
    grid point where the two touch across a shared plane. Assert: `Interfering`; every `Collision.At`
    strictly greater than `1/2` and, at `WithResolution(Scalar(1.0/256))`, the first within `2/256` above
    it; the last collision's volume within `1e-6` of `500` mm³; all three static bodies in `Against`; the
    near slab and the far slab in no `PoseResult` row, both excluded by the From-placed box `[100, 110]`
    grown by the travel `30` to `[70, 140]`. This fixture goes red when the swept box is read at rest: the
    rest box `[0, 10]` grown by `30` excludes the wall, and the report reads `Sound`.
19. **The stated `To` is covered.** The arm against test 2's wall (`y ∈ [60, 80]`) under
    `Between{From: RotationAround((1e6, 3e5, 0), Z, 90°).Inverse(), To: r3.Identity()}`, endpoints only:
    the arm starts about `1.5e6` mm away and arrives at `s = 1` exactly where it sits, `46` mm from the
    wall. The relative
    motion is the far rotation itself, whose read screw rebuilds it only to about `ε·|t| ≈ 1e-10` mm (r3
    §7), so the ideal end misses the identity by that much, while the kernel's own `s = 1` reading is the
    exact axis-aligned box gap. Assert: the `s = 1` row's `Pose` equals `r3.Identity()`; its `Gap.Value` is
    within `1e-6` of `46`; its `Gap.Bound` is strictly positive and below `1e-6`, with `Exactness`
    `Approximate`; the same fixture with the pivot at the origin
    (`From = RotationAround(origin, Z, 90°).Inverse()`) has a strictly smaller, still positive, `s = 1`
    bound; both bounds pass the default gate. This test goes red when `η_1` is charged against `To` alone: `C_1` is then `To` bit for bit for the
    unplaced arm, `η_To` is exactly zero, and the row comes back `Exact` with a zero bound — a claim about an
    ideal end the check never bounded. The `η_To` leg of the maximum is not a leg any fixture can fail:
    `max(η_ideal, η_To) ≥ η_To` by definition, and its size is one `Then` rounding; the test file records
    that argument.
20. **Errors, non-mutation and cancellation.** Test 9 gains one subtest per `Between` row of §8's table
    that the public API can reach: the zero `r3.Transform{}` as `From` and as `To`
    (`ErrDegenerate`); a reflection against the identity (`ErrDegenerate`); `From` equal to `To`
    (`ErrDegenerate`); `Rotation(Z, 1e-300 rad).Then(Translation(1e10, 0, 0))` as `To` against the identity,
    whose screw point is not a float64 (`ErrNotFinite`); a resolution stated as a length or an angle
    (`ErrUnitKind`); each asserting `errors.Is`, no report and an unchanged document. `PoseAt` with an angle
    or a length is `ErrUnitKind`, with `NaN` `ErrNotFinite`. Test 7 runs its before-and-after and replay
    comparison on the screw arm as well. Test 10 needs no change: cancellation is decided before the motion
    kind. The non-finite-component row (`ErrNotFinite`) is kept in code and has no test: every `r3`
    producer validates what it builds and `Transform`'s fields are unexported, so `r3`'s public API cannot
    produce a `Transform` with a non-finite component.
21. **Example.** `examples/` gains `Example_decad_motionBetween`: the screw arm of test 12 against the raised
    wall at `WithResolution(Scalar(1.0/256))`, printing `Status` and the first collision's fraction to three
    decimals. The grid is dyadic, so it prints `0.410` — `105/256`, the first depth-8 grid point above
    `s* ≈ 0.40967`, where the corner penetrates the wall by `0.0235` mm and the overlap is a `5.8e-3` mm³
    wedge, far above the transfer allowance — on every platform.

Three legs of the `Between` bound sit below any public fixture's observability and are pinned by internal
tests in `motion_internal_test.go`, each on the production function: `pathAreaUpper`'s stretch base — a
base above `1` scales both the unscaled shortcut and the stretched allowance, and the base `1` reproduces
the `Revolute` value exactly — which goes red when the base is dropped; `basisSigmaUpper`, exactly `1` for
the identity and strictly above `1` for a basis whose columns are not exactly orthonormal; and the
`Between` frame's ideal end, which maps the rest-box corners of the screw arm, of test 19's far pivot and
of test 15's offset axis to within `1e-9·(1 + |t|)` of their images under `To` — a test, not an admission
gate, and it admits nothing. Test 15's offset axis is the fixture that shows a frame composed in the wrong
order: its `From` translates the blade off the screw axis, so the screw before `From` and the screw after
it land the corners tens of millimetres apart. The screw arm and test 19 cannot show it, because their
`From` commutes with the screw — the identity in one, a rotation about the screw's own axis in the other —
so both orders give the same end.

`.github/test-shards.txt` is updated for every root-package test, fuzz target and example above, and
`go test . -run '^TestCIWorkflowRaceShardsCoverEveryPackage$'` is run before the push.

## 10. Increments

| PR | lands | still `Suspect` after it |
|---|---|---|
| 1 (`motion.go`, `motion_verify.go`, `motion_bound.go`) | `Motion`, `Revolute`, `Prismatic`, `PoseAt` and their refusals; `WithMotionTolerance`; `Diagnostic.At` and the four `DiagMotion*` codes; `VerifyMotion` over the two endpoints only (§6 steps 1–4 and 7, no bisection), the swept-box exclusion with travel from rest, `η_k` charged to gaps, `ρ_max` and `τ` over exact rationals; `IntervalClear`/`IntervalColliding`/`IntervalUndecided`; `MotionRequest.RelativeTolerance`; tests 2 (endpoints only), 4, 5, 6, 7, 8, 9, 10 and the swept-box-from-rest test | every interval the endpoints alone cannot certify (test 2's swing among them); a collision at a pose whose `η_k` is nonzero is published without the §5.1 transfer |
| 2 | §6 steps 5 and 6 — bisection for the verdict, including the onset bisection of a colliding interval with a collision-free end, and for the reading — `WithResolution`, `WithMinClearance`, `Assessment`, `MotionRequest.Resolution`/`MinClearance`; the §5.1 collision transfer through `sweptVolumeAllow(η_k, A)`, with `DiagUndecidedInterference` at a pose for an overlap that does not transfer, and `Collision`'s doc comment restated as a claim about the ideal pose; tests 1, 2 (bisected), 3, 11, and a transfer test: a pose with a nonzero `η_k` whose measured overlap is published with its bound widened by the allowance, and a fixture whose overlap volume is below the allowance that reads `DiagUndecidedInterference` rather than `Collision` | pairs the clearance kernel leaves undecided (§7); a stitched mover under a `Revolute` |
| 3 (`motion.go`, `motion_verify.go`, `motion_bound.go`; `go.mod` pins `r3` at `b624f6d`) | `Between`, its `PoseAt` and refusals (§2, §8); the `Dimensionless` parameter, its resolution and labels (§2, §3); the screw frame — `θ`, `d`, `Axis`, `Point` and `From` read exactly, `radianSinCos` for `s·θ` — and `η_1 = max(η_ideal, η_To)` at the stated `To` (§5.1); `ρ_max` and the swept box from the From-placed corners (§5.2, §6); `τ` with the slide term (§5.2); `pathAreaUpper`'s stretch base and `basisSigmaUpper` (§5.1); tests 12–21, the internal tests of §9, and test 9's and test 7's `Between` rows | pairs the clearance kernel leaves undecided (§7); a stitched mover under any `Between` that is not a translation between translations |

PR 1 was the end-to-end instance: a real mover, a real static body, the real kernel, one certificate, one
report, at endpoint scope. PR 2 builds on it and changes no PR 1 result except the two the transfer rule
names: a collision whose measured volume does not clear `sweptVolumeAllow(η_k, A)` stops being a
`Collision`, and every published collision volume carries the widened bound. PR 3 changes no `Revolute` or
`Prismatic` result: both pass the stretch base `1`, read their swept box at rest and their `τ` without a
slide term, exactly as before.

## 11. Dependencies and settled points

**What each dependency supplies.**

- `r3` at `b624f6d` provides `Transform.Screw`, `Screw.At` and `Transform.Interpolate` (r3
  `docs/screw-motion-design.md`), and `go.mod` pins that revision. `Between` reads the axis, point, angle and
  slide with `Screw` — once per `PoseAt` call, and once per `VerifyMotion` for the exact frame — builds every
  pose with `At`, and never calls `Interpolate`, whose pose alone would not give the travel bound its
  parameters. `Screw` is a deterministic function of its input bits, which is what lets the frame and
  `PoseAt` read the same parameters; the soundness of `η` does not depend on it, since `η` charges whatever
  separates the pose actually built from the ideal one.
- `r3` publishes no bound on how far `Rotation`, `RotationAround`, `Translation`, `Then`, `Screw` and `At`
  deviate from the ideal isometry they denote; it validates only that each result IS an isometry within
  `1e-9` (r3 §10 leaves this out of scope). decad closes the gap itself with `η_k`'s exact enclosure (§5.1)
  — for a `Between`, including the rebuild error between the read screw's end and the stated `To` — so this
  is not blocking. An `r3`-published bound would let the enclosure be replaced by a read, and would be the
  first `r3` API to carry a `units.Value` bound.
- `units` and `sketch` need nothing. `units.Degree`'s factor is a rounded `π/180`, which is exactly why a
  degree-stated angle is enclosed as an exact turn fraction rather than through `In(units.Radian)`; a
  `Between`'s `Scalar` fraction and `Screw`'s radian `Angle` both carry the factor `1` and are read exactly.

**Settled design points.** Each of these is stated in full where it applies and summarised here so a
reader finds them in one place: `Diagnostic` carries `At *units.Value`, nil on every `Verify` diagnostic
(§4.1); the motion tolerance is its own `WithMotionTolerance` and `WithTolerance` is unchanged (§3); the
default resolution is `(To − From)/1024`, `1/1024` for a `Between` (§3); the static set is every other live
body, with no option to narrow it (§3); `Revolute` names its axis with two vectors, and a form taking core
§6.2's sealed `Axis` is a later addition (§2); the interval certificate is two-sided (§5.2); a `Between`'s
parameter is the dimensionless fraction `s ∈ [0, 1]` (§2); its ideal path is the exact screw of the read
parameters composed onto `From`, and the stated `To` is charged at `s = 1` through `η_1 = max(η_ideal,
η_To)`, not reported and not refused (§5.1); its travel bound is the sum `|Δs|·(ρ_max·θ + |d|)`, not the
Pythagorean form (§5.2); its `ρ_max` and swept box are read from the From-placed corners (§5.2, §6); it
traces the shorter arc, accepts two reflections and refuses a reflection on one side only (§2).
