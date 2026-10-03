# Motion Check Design

How `Document.VerifyMotion` answers "does this body hit anything while it moves along this path?" without
changing the document: where the capability sits (§1), how the caller states the motion (§2), the entry point
and its options (§3), the report (§4), what a pose proves and what an interval between two poses proves (§5),
the refinement procedure (§6), evaluator coverage (§7), errors (§8), required tests (§9), increments (§10),
and the dependency gaps and settled points (§11). Companion to `docs/api-design.md` (the core contract,
referenced as "core §N"), `docs/verification-design.md` ("verification §N", which owns `Status`,
`Diagnostic`, the tolerance gate and the result vocabulary this report reuses), `docs/clearance-design.md`
("clearance §N", which owns the pair kernel every pose runs), `docs/interference-design.md` ("interference
§N", which owns the pair relation and the read-only overlap proof) and `docs/evaluator-design.md`
("evaluator §N", whose §8 owns placement re-evaluation). Nothing here changes those contracts.

## 1. Scope: what decad owns, and what the layer above owns

**decad owns the verification half of motion: a proof that one rigidly moving set of bodies does or does not
meet the rest of the document anywhere along a stated one-parameter path.** Core §1 lists "do these bodies
interfere, and what is the clearance?" among the questions decad is answerable for, and core §13 settles how
such a pair is positioned: interference and clearance run between explicitly placed bodies, through
`Body.Placed(ctx, t)`, with no assembly machinery. A motion check is that same pair question asked over a
family of placements `t(s)`, `s ∈ [From, To]`, instead of at one placement. It consumes exactly what core
§13 already grants — a rigid motion stated by the caller as an `r3.Transform` — and adds no `Component`, no
`Occurrence`, no joint graph, no timeline and no view state. Every v1 non-goal of core §13 stays a non-goal.

Three things do NOT belong in decad, and this design keeps them out:

| Out of scope | Why | Where it goes |
|---|---|---|
| Animation, frame generation, rendering | Core §4 rejects GUI and view state on the geometry model; nothing here produces an image or a frame sequence | A separate module that imports decad and calls `Motion.PoseAt` (§2) for the poses it wants to draw |
| Kinematic chains, joints, linkages, more than one independent motion | A second independent motion makes the relative motion of two movers a composition decad would have to derive; one rigid moving set has one path | The same layer above, which can call `VerifyMotion` once per relative motion it has already resolved |
| Dynamics, contact forces, time | decad measures geometry; the parameter `s` is an angle or a length, never a time | Out of scope |

The layering rule holds unchanged: `decad -> sketch -> r3 -> units`. Every pose is an `r3.Transform` built by
`r3.RotationAround` or `r3.Translation`; decad composes it onto a body's placement with `Transform.Then`
exactly as `Body.Placed` does, and hand-rolls no rotation. §11 records the one `r3` gap this exposes.

## 2. The motion vocabulary

`Motion` is a sealed interface, like `Axis` and `Extent` (core §6.2, §8.1): a motion is one of a fixed set of
shapes, and an illegal one is unrepresentable rather than rejected at runtime.

```go
// Motion is a one-parameter family of rigid motions, parameterised by a typed
// scalar whose Kind the variant fixes: an Angle for Revolute, a Length for
// Prismatic. PoseAt returns the rigid motion at one parameter value; it is
// what a layer above calls to draw the moving set, and what VerifyMotion
// calls to evaluate a pose.
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
```

`From` and `To` are **signed displacements, not magnitudes**, on the same terms as `ToFace.Offset` and
`ExtrudeOpts.Taper` (core §12): the sign says which way the body moves, so a negative value is a legal intent
and never `ErrNegativeMagnitude`. `From == To` names no path and is `ErrDegenerate`. `From > To` is legal and
traverses the path in the other sense; the report (§4) lists poses in traversal order.

`PoseAt(at)` is `r3.RotationAround(Center, Axis, at)` for a `Revolute` and `r3.Translation(d·at)` for a
`Prismatic`, where `d` is `Dir` normalised by `r3.Vec.Normalize`. It is the ONLY place a pose transform is
built, so the renderer above and the verifier below evaluate the same pose from the same inputs. Its
refusals follow the variant's own fields and the `r3` constructor's:

| Input | Error |
|---|---|
| `From` or `To` of the wrong `Kind` (a length for a `Revolute`, an angle for a `Prismatic`, a bare scalar for either) | `ErrUnitKind` |
| `at` of the wrong `Kind` | `ErrUnitKind` |
| a non-finite `From`, `To`, `at`, `Center`, `Axis` or `Dir` component | `ErrNotFinite` |
| `Axis` or `Dir` the zero vector (`r3.ErrDegenerateAxis`, or `Normalize` reporting no direction) | `ErrDegenerate` |
| `From == To` (checked by `VerifyMotion`, not by `PoseAt`, which takes no range) | `ErrDegenerate` |
| the `r3` constructor refusing the result (`r3.ErrNonFinite`, an overflowing pivot offset) | `ErrNotFinite` |

A `Revolute` names its axis with two vectors, a position and a direction, because that form needs no
selector resolution at the call. A second form taking core §6.2's sealed `Axis` — an `EdgeAxis` naming a
hinge pin's own edge — is a later addition on the same `Motion` set; it is not refused today, it does not
exist.

**Two shapes are deliberately absent.** A list of discrete poses adds nothing the API lacks: `PlacedCopy`
then `Verify` already checks any finite set of placements the caller can name, and a pose list carries no
path between its entries, so nothing continuous could be proven over it (§5.2). An interpolation between two
`r3.Transform`s — `Between{From, To r3.Transform}`, the screw motion joining them — needs the axis, angle and
pitch of a rigid motion read back out of its matrix, and `r3` has no such reader (§11). It is **staged, not
rejected**: when `r3` gains it, `Between` joins the sealed set with the §5.2 travel bound of a screw motion
(a rotation term and a translation term summed), and until then no `Between` type exists, so no caller can
be handed `ErrUnsupported` for it.

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

`WithResolution` states the finest parameter step the check will refine to (§6). An interval narrower than
the resolution that the certificate still cannot settle reads undecided and is not split further; it is the
caller's statement that a feature narrower than this does not need to be found. It is a magnitude of the
motion's own `Kind` — an angle for a `Revolute`, a length for a `Prismatic` — so a value of the wrong
`Kind` is `ErrUnitKind`, a negative or zero one `ErrNegativeMagnitude`, a non-finite one `ErrNotFinite`. A
resolution larger than `|To − From|` is legal and means the endpoints alone are evaluated. The default,
`(To − From)/1024` in magnitude, caps a worst-case run at 1025 poses per pair; it is a constant the
implementation owns and re-sizes from measured per-pose cost, not a value the API promises.

`WithMinClearance` states a spec: the moving set MUST stay at least `minimum` from every static body over the
whole path. Like `WithMinWallThickness` it turns a measurement into an `Assessment` (verification §1.0):
`AssessmentMet` when every interval certifies the margin (§5.2), `AssessmentViolated` when some evaluated
pose proves a gap below it, `AssessmentUndecided` otherwise. Its magnitude rules are `WithMinWallThickness`'s.

Duplicate options keep verification §1.0's last-occurrence-wins rule. `VerifyMotion` validates the document,
the moving set and every option before it reads `ctx`, so a validation error wins over an already-cancelled
context exactly as in verification §1.2; after validation, a cancelled context returns `ctx.Err()` unchanged
and no report. The context is the public work control: there is no separate budget or progress API
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
    At            units.Value    // the parameter; Kind Angle or Length as the Motion fixes
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

// Collision is a proven overlap at an evaluated pose.
type Collision struct {
    At     units.Value   // the parameter of the pose
    Pose   r3.Transform  // Motion.PoseAt(At)
    Moving *Body
    Static *Body
    Volume *Measurement  // the overlap volume when the read-only proof bounded it; nil when overlap is proven but unmeasured
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
| `IntervalColliding` | a proven collision sits at one of its endpoints; nothing is claimed about the interior |
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
| a pose proves overlap for some pair | `Interfering` | `DiagMotionCollision` — `Pair` set, `At` set, `Reading` `ReadingOverlapVolume` with `Observed` the volume when bounded, else `ReadingNone` |
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

With those three in place, pose `k` proves, for each pair, one of interference §1's four relations about the
IDEAL pose, and when disjoint or touching a gap interval `[lo_k, hi_k]` with `lo_k` a proven lower bound and
`hi_k` a proven upper bound on the true distance between the ideal placed mover and the static body. A proven
overlap is a `Collision`: this is the falsification CLAUDE.md's hard rule permits, and it is sound on any
payload the read-only overlap proof reaches (§7), whether or not the clearance kernel can model the pair.

### 5.2 What an interval proves

Sampling proves nothing between samples; a thin pin the arm passes between two evaluated poses leaves every
pose clear. The continuous claim rests on one fact and one bound:

**The fact.** The distance between two closed sets is 1-Lipschitz in a displacement of either: if every point
of `M` moves by at most `τ`, then `d(M', S) ≥ d(M, S) − τ`. So on `[s_k, s_{k+1}]`, for every `s`,

```text
d(M(s), S) ≥ max( lo_k − τ_k(s − s_k),  lo_{k+1} − τ_k(s_{k+1} − s) )
```

where `τ_k(Δ)` bounds how far any point of the mover travels over a parameter change `Δ`.

**The bound.** For a `Prismatic`, every point travels exactly `|Δ|` along the unit direction, so `τ(Δ) =
|Δ|`, up-rounded. For a `Revolute`, a point at distance `ρ` from the axis travels an arc of length `ρ·|Δθ|`,
which bounds its chord, so `τ(Δθ) = ρ_max · |Δθ|` in radians, up-rounded, where `ρ_max` bounds the distance
from the axis of every point of the mover. `ρ_max` is read ONCE, from the mover's `Bounds()` at its current
placement: the box inflated outward by its own `Bound`, the eight corners' distances from the axis line
computed with `r3.Vec.Sub`, `Cross` and `Len`, each up-rounded, the largest taken, then the rounding of the
`r3` operations themselves charged by `rigidRoundAllow`'s discipline at the corner's magnitude. A rotation
about the axis preserves every point's distance from it, so one reading covers every pose. The angle `Δθ` is
the exact difference of two dyadic grid parameters (§6) converted to radians through an upward-rounded `π`
enclosure for a degree-stated motion, and taken as stated for a radian one.

**The certificate.** The interval is `IntervalClear` for a pair when the two one-sided bounds together cover
it with strictly positive distance everywhere, which holds exactly when

```text
lo_k + lo_{k+1} > τ_k(s_{k+1} − s_k)      (every term rounded against the claim)
```

The two sides meet where `lo_k − τ(s − s_k) = lo_{k+1} − τ(s_{k+1} − s)`, and the lower envelope's minimum
over the interval is `(lo_k + lo_{k+1} − τ_k)/2`, down-rounded — that is the interval's `Clearance` lower
bound. The envelope is strictly positive on the whole closed interval, so the boundaries never meet, so no
boundary crossing occurs and the pair's relation cannot change from the disjoint one proven at an endpoint:
a nesting cannot begin without the boundaries crossing, and a touching cannot occur with positive distance.
The interval is `IntervalClear` only when EVERY (mover, static) pair certifies.

Three consequences are deliberate:

- **A touching endpoint does not block the interval.** A swing that begins resting on a stop has `lo_0 = 0`,
  an `Exact` zero (clearance §6); the certificate still closes from the far end when `lo_1 > τ_0`. The
  one-sided form `lo_k > τ_k` alone could never certify the first interval of such a swing, which is why the
  certificate is two-sided.
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

## 6. The refinement procedure

Evaluation is deterministic bisection on a dyadic grid, so a replay reproduces the same poses, the same
verdicts and the same report (evaluator §8):

1. **Validate** (§3, §8) before reading `ctx`.
2. **Read every mover's `ρ_max`** (§5.2) for a `Revolute`; a `Prismatic` needs none.
3. **Swept-box exclusion.** For each (mover, static) pair, inflate the mover's `Bounds()` box outward by its
   `Bound` plus the whole path's travel bound `τ(To − From)`, and the static body's by its `Bound`. Boxes
   `boxesDisjoint` prove the pair clear over the whole path, with a lower bound of the box gap minus the
   travel bound; the pair is never evaluated at any pose and contributes to every interval as `IntervalClear`
   (§5.3). This is the same Lipschitz fact as §5.2 applied once, and it is what keeps a large document cheap
   when the mover is far from most of it.
4. **Evaluate the endpoints** `From` and `To` (§5.1) for every remaining pair.
5. **Bisect.** Take the first interval, in traversal order, that is neither `IntervalClear` nor
   `IntervalColliding` and whose width exceeds the resolution; evaluate its midpoint `(s_a + s_b)/2` — exact
   in float because the grid is dyadic in `(To − From)` — and replace the interval by its two halves.
   Repeat until no such interval remains. An interval whose width is at or below the resolution and that
   still fails the certificate becomes `IntervalUndecided`.
6. **Publish** (§4), with `Poses` in parameter order including every midpoint inserted.

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
| unplaced or placed `prismPayload` with zero section displacement, `revolvePayload`, zero-vertex-bound closed `stitchPayload` | analytic kernel (clearance §2) | kernel overlap, or read-only intersection | `IntervalClear` reachable |
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
| `From == To` | `ErrDegenerate` |
| `Axis`/`Dir` with no direction | `ErrDegenerate` |
| a wrong-`Kind` `From`, `To`, resolution, tolerance or minimum | `ErrUnitKind` |
| a non-finite parameter, vector component, resolution, tolerance or minimum | `ErrNotFinite` |
| a negative or zero resolution, a negative tolerance or minimum | `ErrNegativeMagnitude` |
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

1. **Known collision angle.** Static wall: a prism occupying `x ∈ [−100, 100], y ∈ [40, 60], z ∈ [0, 10]`.
   The corner `(48, 14)` reaches `y = 40` when `sin(θ + atan(7/24)) = 4/5`, that is at
   `θ* = atan(3/4) ≈ 36.87°`, and every other point of the arm reaches it later. Assert: `Status` is
   `Interfering`; `Collisions` is non-empty; every `Collision.At` is strictly greater than `θ*`; every
   `IntervalClear` interval ends at or below `θ*`; the interval containing `θ*` is not `IntervalClear`; with
   `WithResolution(0.25°)` the first `Collision.At` is within `0.5°` above `θ*`; every `Collision.Volume` is
   positive with its `Bound` below its `Value`. Assert the same under a `Revolute` stated in radians.
2. **Clear swing with a stated margin.** Static wall at `y ∈ [60, 80]`. The minimum gap over the path is
   exactly `10` mm, at `θ = 90° − atan(7/24)`. Assert: `Status` is `Sound`; every interval is
   `IntervalClear`; `MotionReport.Clearance.Value` is within `0.1` mm of `10` with its bound below the
   tolerance gate; with `WithMinClearance(9 mm)` the `Assessment` is `AssessmentMet`; with
   `WithMinClearance(11 mm)` it is `AssessmentViolated` and `Status` is `Violating` with a
   `DiagMotionClearanceViolated` whose `At` is the pose of the proving gap; with `WithMinClearance(10 mm)`
   exactly, the `Assessment` is `AssessmentUndecided` and the report `Suspect` — never `Met`, because no
   proven lower bound can reach an exact minimum.
3. **Near miss between samples.** Mover: a blade `x ∈ [0, 50], y ∈ [−0.5, 0.5]`, same motion. Static: a pin,
   a 1 mm cube centred at polar angle `90·31/64 ≈ 43.59°`, radius `49`, so the blade sweeps through it over a
   window about `2.3°` wide that contains no dyadic grid point of depth 5. With `WithResolution(3°)` assert:
   no `Collision`; `Status` is `Suspect`, never `Sound`; the `IntervalUndecided` interval contains `43.59°`.
   With `WithResolution(0.1°)` assert: `Status` is `Interfering` and the first `Collision.At` lies within the
   window. This fixture is the one that goes red when the `ρ_max·Δθ` term is deleted, when `ρ_max` is read
   from the centroid instead of the box, or when the certificate is made one-sided and the pin is moved to
   the first interval; the test file records each.
4. **Touching start.** The arm of fixture 2 with a stop prism sharing its `y = −14` face plane at `θ = 0`
   (a stop-built extrude, clearance §6's coplanar certificate), swinging away. Assert: the first
   `PoseResult.Clearances` row is an `Exact` zero; the first interval is `IntervalClear`; `Status` is `Sound`.
   This fixture goes red when the certificate loses its `lo_{k+1}` side.
5. **Prismatic.** A 10 mm cube translating along `+X` from `0` to `30` mm toward a wall whose face is at
   `x = 25`. Assert a collision with every `Collision.At` strictly greater than `15` mm, every `IntervalClear`
   ending at or below `15` mm, and with the wall at `x = 45` a `Sound` report whose `Clearance` is within
   `0.01` mm of `5`.
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

`.github/test-shards.txt` is updated for every root-package test, fuzz target and example above, and
`go test . -run '^TestCIWorkflowRaceShardsCoverEveryPackage$'` is run before the push.

## 10. Increments

| PR | lands | still `Suspect` after it |
|---|---|---|
| 1 | `Motion`, `Revolute`, `Prismatic`, `PoseAt` and their refusals; `Diagnostic.At` and the four codes; `VerifyMotion` over the analytic kernel with endpoints only (no bisection), swept-box exclusion, `η_k`, `ρ_max`; `IntervalClear`/`IntervalColliding`/`IntervalUndecided`; tests 2, 4, 5, 6, 7, 8, 9, 10 | every interval the endpoints alone cannot certify |
| 2 | bisection to the resolution floor, `WithResolution`, `WithMinClearance` and `Assessment`; tests 1, 3, 11 | pairs the clearance kernel leaves undecided (§7) |
| 3 | `Between` over a screw motion, once `r3` reads axis, angle and pitch out of a `Transform` (§11) | — |

PR 1 is the end-to-end instance: a real mover, a real static body, the real kernel, one certificate, one
report. If `η_k`'s rational enclosure or the transient-placement path blocks it, that is the blocker to
report, not a reason to ship a sampled check under the continuous name.

## 11. Dependency gaps and settled points

**Dependency gaps.**

- `r3` has no interpolation between two `Transform`s and no reader for a transform's screw parameters —
  rotation axis, angle and translation along the axis (`Rotation` constructs from axis and angle; nothing
  inverts it). `Between` (§2) is staged on it. It belongs in `r3`, which owns coordinate math, never in decad.
- `r3` publishes no bound on how far `Rotation`, `RotationAround`, `Translation` and `Then` deviate from the
  ideal isometry they denote; it validates only that the result IS an isometry within `1e-9`. decad closes
  the gap itself with `η_k`'s exact enclosure (§5.1), so this is not blocking. An `r3`-published bound would
  let the enclosure be replaced by a read, and would be the first `r3` API to carry a `units.Value` bound.
- `units` and `sketch` need nothing. `units.Degree`'s factor is a rounded `π/180`, which is exactly why a
  degree-stated angle is enclosed as an exact turn fraction rather than through `In(units.Radian)`.

**Settled design points.** Each of these is stated in full where it applies and summarised here so a
reader finds them in one place: `Diagnostic` carries `At *units.Value`, nil on every `Verify` diagnostic
(§4.1); the motion tolerance is its own `WithMotionTolerance` and `WithTolerance` is unchanged (§3); the
default resolution is `(To − From)/1024` (§3); the static set is every other live body, with no option to
narrow it (§3); `Revolute` names its axis with two vectors, and a form taking core §6.2's sealed `Axis` is a
later addition (§2); the interval certificate is two-sided (§5.2).
