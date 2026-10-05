# Multibody Dynamics Design

This document owns the program that takes decad's pair contact proofs and the `dynamics` subpackage from
narrowly gated two- and three-body slices to N-body certified replay: the N-body world, state and trace; pair
enumeration and the certified broad phase; island formation and the certification of the general projected
solver; the multi-step `Timeline`; the order in which mass properties, contact manifolds and sweeps extend to
every solid payload; the interface the `_gallery` module needs from `kinetograph` to film a trace; and the
phases, filmable exit scenes and PR order that deliver all of it.

It owns the N-body ALGORITHMS and their certificates. The response law, step configuration and conservation
readings stay with `docs/rigid-dynamics-design.md`; the claims `ContactPair` and `SweepPair` make stay with
`docs/contact-geometry-design.md` and `docs/contact-sweep-design.md`; the mass integrals stay with
`docs/dynamic-mass-design.md`; the system map stays with `docs/collision-dynamics-design.md`. Where this
document adds an outcome, a relation value, a reason or a field to one of those contracts, that document
carries the addition and points here for the algorithm.

Current state: §13 PRs 1, 2, 3, 4, 6, 7, 8, 10, 11, 12, 14, 16, 17 and 19 have shipped, and so have the
root parts of PRs 13 and 18. `dynamics.World` holds any number of bodies, the canonical pair table and
per-pair material of §3.1, and the slice-backed `State` of §3.2. Its step resolves two bodies, or three bodies with one, two or three dynamic bodies and
every other body fixed, through the closed-form responses `docs/rigid-dynamics-design.md` lists. A world of four or more
bodies takes the scheduled step of §4.3 and §5: one kick, then slices of drift from event to event. Initial
contacts, impact brackets and transition brackets cut a slice at their exact fraction; every body advances
there on its certified path; the touching and impacting pairs, with the contact-set pairs their bodies rest
on, form §6.1's islands, which §6.2 proposes, §6.3's rows certify and §6.6 corrects, publishing
one `IslandReport` per island and one event per pair; the next slice starts from the post-event state. A
graze publishes its event without cutting the slice. Kinematic bodies with translating drivers join
islands, and a positive-friction pair takes the Coulomb rows of §6.2 and §6.3. The published state
carries the step's contact set and reuse cache (§3.2), so a later step continues a resting pair with no
solve, reuses every certificate whose inputs repeat (§5.3), and restarts a repeated island at its fixed
point (§6.2); `MaxPairSweeps` bounds the rest. The multi-event
`Trace` of §3.4 and §7.1, the `Timeline` of §7.2 and §12's typed diagnostics ship. `Document.SweptBox`
(§4.2) is public; the cylinder and bounded-faceted clear sweeps certify with it, and the scheduled step's
broad phase reads it.
`docs/collision-v1-support.md` is the inventory of the shape pairs, responses and refusals that ship, and
this document does not restate it. The two- and three-body steps query their pairs one by one with no broad
phase, and their `Trace` holds fixed two- and three-body slots. The exact arithmetic every certificate below
is stated in already exists as one package: `internal/proof` (`Dyadic`, `DyV3`, `RatInterval` and the float
rounding bounds), which the root package imports today and which `dynamics` can import as well, since an
`internal/` package is visible to every package of this module. §3.1, §3.2, §4 and §8.1–§8.8 describe
shipped code. For worlds of four or more bodies, §5, §6.1–§6.4, §6.6, §7, §3.3, §3.4 and §12 ship as
well. §9.1–§9.4 ship for prisms over whole `LineSeg` sections and for zero-bound
Booleans, directly or through a translation-only placement; stitched solids and lofts are not admitted yet.
§10.1, §10.2 and §10.3's sweep certificate ship for the same bodies; `dynamics` does not consume band
tracks yet. §10.4's `ContactBand`, its sweep and its replay ship for positive-bound faceted Booleans and
all-planar cap-loop chamfers; `dynamics` does not consume a band yet. Everything else is design-only
until the PR table in §13 says otherwise.

Navigation only; the named sections own the rules:

| Question | Section |
|---|---|
| What is in scope, and what is deliberately not? | §1 Goals and non-goals |
| Which scene proves each phase done? | §2 Phases and exit scenes |
| What do an N-body `World`, `State`, `StepReport` and `Trace` hold? | §3 N-body data model |
| How are pairs enumerated and which are swept? | §4 Pair schedule and certified broad phase |
| How does a step advance through many events? | §5 Event schedule |
| How are simultaneous contacts solved and certified? | §6 Islands and the certified projected solver |
| How is a many-event, many-body trace replayed? | §7 Trace and `Timeline` |
| Which mass properties extend, in what order? | §8 Mass-property extensions |
| How does an arbitrary planar solid get a contact manifold? | §9 Exact planar faceted contact |
| How does an arbitrary solid sweep while rotating, and rest while rotating? | §10 General rotating sweep and band tracks |
| What does the viewer need, and how does the gallery film a trace? | §11 Kinetograph interface and gallery |
| What stops a step, and how is that reported? | §12 Work budgets, cancellation and `Undecided` |
| Which PR lands what, in what order, proven by which test? | §13 Delivery order |
| How is each capability tested? | §14 Test and fixture strategy |

## 1. Goals and non-goals

The goal is a certified replay: a scene of N decad solids, stepped by `dynamics`, whose every published
pose at every sampled time is backed by a geometry certificate, filmed through `kinetograph` in the
`_gallery` module. Four decisions fix the shape of the program:

1. **Phased by shape family.** N-body support lands first for the source primitives the contact kernel
   already certifies (source boxes at signed-permutation poses, source spheres, source cylinders on an
   axial disk or a vertical sidewall, faceted floors with an exact source-box lower face), then arbitrary
   exact planar solids under any proper rotation, then the remaining payloads. §2 names the phases.
2. **Certified replay only.** Every step is ideal-plus-rounded certified exactly as the shipped two-body
   step is. A step the proofs cannot settle returns `Undecided` with a typed reason (§12) and the
   `Timeline` stops there; no "best effort" integrator exists anywhere, the gallery included.
3. **The viewer bridge is a new kinetograph node kind** that takes a time-varying transform (§11).
   kinetograph's own implementation of that node is outside this document.
4. **One general island solver.** The fixed-order projected iterative solve that
   `docs/rigid-dynamics-design.md` "Response" specifies replaces the closed-form responders. Every shipped
   closed-form fixture becomes a regression fixture of the general solver and keeps producing the same
   certified numbers (§6.5).

Non-goals, each with the reason it is out:

| Not in this program | Why |
|---|---|
| Joints, deformation, fracture | `docs/collision-dynamics-design.md` §2 limits the first stage to rigid solids; nothing here needs more. |
| Sleeping or deactivation | A sleeping body publishes poses no certificate backs; a resting island costs one persistent track per pair and stays certified. |
| A continuous-force integrator between events | The step's force law is the one semi-implicit kick `docs/rigid-dynamics-design.md` "Step and event schedule" defines; the caller picks `dt`. A tumbling body therefore moves at constant `ω` between kicks, and §10.3's band track is what lets it rest. |
| Non-convex dynamic bodies with a manifold | §9 proves a manifold for a convex body against any planar face. A non-convex DYNAMIC body gets a relation and no manifold (`ContactNonConvex`); a non-convex FIXED or kinematic body is fine, since only its faces enter. |
| kinetograph interpolating between certified poses | §11: the viewer asks for a pose at a frame time and shows that pose; it never blends two. |
| Uncertified mass from a render mesh | `docs/dynamic-mass-design.md` §2.2: a `VerifyNone`/`VerifyBoundary` mesh never supplies inertia. |

## 2. Phases and exit scenes

Each phase ends with one filmable scene that runs through the real producers (`Body.MassProperties`,
`ContactPair`, `SweepPair`, `World.Step`, `Timeline`, the gallery bridge) with no hand-written manifold,
event or pose anywhere. The scene is a `_gallery` test (§11.3) AND a `dynamics` integration test; the
gallery renders it with `go run . dynamics -scene <name>`.

| Phase | Shape families | Exit scene | What the scene proves |
|---|---|---|---|
| 1 | Source boxes at signed-permutation poses, source spheres, source cylinders (axial disk or vertical sidewall), faceted floors with an exact source-box lower face (contact-geometry §2) | `stack-and-drop` | N-body world, broad phase, islands, friction and resting stacks, multi-event trace, viewer bridge. |
| 2 | Any exact planar solid (zero boundary displacement) under any proper rotation; convex dynamic bodies, any planar fixed body | `tumble` | Face, edge and vertex impacts under rotation; a tumbling body comes to rest through band tracks. |
| 3 | Every remaining solid payload: curved source families, positive-displacement bodies, lofts, sweeps, cups, cap blends, stitched solids | `parts-bin` | Mass for every payload; banded contact for positive-displacement bodies; curved rolling contact. |

**Phase 1 — `stack-and-drop`.** A fixed source-box floor `200×200×10 mm`, top at `z = 0`. Six `20 mm`
source boxes in a `3-2-1` pyramid: three on the floor at `x = 0, 25, 50`, two bridging the gaps on top,
one on the top row. Three radius-`8 mm` source spheres with centers released at `z = 60, 90, 120 mm`
over `x = 120`, offset in `y` so the second lands on the first and the third on both. One source cylinder (`Ø20 × 30 mm`, axis along `z`)
released axially at `z = 80 mm` over `x = 160`. Gravity `-9810 mm/s²`, box restitution `0.3`, sphere
restitution `0.6`, friction `0.4` everywhere, density `0.001 kg/mm³`, `dt = 1/240 s`, `2 s` of motion,
rendered at `60 fps`. Exit criterion: `Timeline.End()` equals `2 s` (no `Undecided`); the six pyramid
boxes end within `PenetrationResidual` of their start poses; every step's conservation gate passes; the
bridging boxes' two half-face patches each carry their share of the support impulse; the first sphere's
first floor impact occurs at the free-fall time `sqrt(2·(60−8)/9810) s` within `TimeResolution`; the clip
renders `120` frames.

**Phase 2 — `tumble`.** The floor plus a fixed tray built as a zero-bound Boolean `Cut`: an outer source
box minus an inner source box that opens its top, leaving a floor and four walls inside `160×160 mm`. A
mesh `Union` of overlapping boxes is not used, because its triangle diagonals generally cross the other
operand's planes off the dyadic grid, which leaves a positive mesh bound that §9 does not admit. Dynamic: four `20 mm` source boxes released with
proper rotations about `(1, 1, 0)` by `30°`, `45°`, `60°`, `75°` and spin `(2, 1, 0) rad/s`; one
hexagonal prism (`20 mm` across flats, `12 mm` tall) released on a vertex; one triangular wedge; one
stitched tetrahedron. Material as above, `3 s`. Exit criterion: the timeline reaches `3 s`; each body's
final pose is a face-down rest (every dynamic body's velocity within `VelocityResidual` of zero and at
least one band or persistent track per body in the last step); the trace carries at least one
`ContactImpact` whose manifold has a single point (vertex impact), one with two points (edge impact) and
one with four or more (face impact); each box's first impact time matches the exact drift of its lowest
corner within `TimeResolution`.

**Phase 3 — `parts-bin`.** The tray plus: a shelled box (`cupPayload`), a `12 mm` cap-loop chamfered
block (positive displacement), a square-to-octagon loft, a straight sweep of a hexagon, a revolved
bottle (full revolve of a line-and-arc half-profile), and a source cylinder released on its side so it
rolls. `4 s`. Exit criterion: `Body.MassProperties` publishes for every body; the timeline reaches `4 s`;
the rolling cylinder's trace carries a rotating band track with its contact-point speed within
`VelocityResidual` of zero (rolling without slip under friction `0.4`); the chamfered block rests on a
`ContactBand` track whose published depth is at most its boundary displacement plus `PenetrationResidual`.

## 3. N-body data model

### 3.1 World and pairs

`WorldConfig` is unchanged. `NewWorld` admits `len(cfg.Bodies) >= 2`. Until §13 PR 5 it keeps refusing the
two- and three-body role mixes the shipped step cannot take, and a three-body world keeps its
`threeBodyWorld` container, whose per-pair response worlds are built from this table's entries; both go away
in PR 5. Until then the two- and three-body worlds keep their shipped steps, so their published results
stay unchanged, and only a world of four or more bodies takes the scheduled step of §4.3 and §5; PR 5 moves
every world onto it. Internally:

```go
type worldBody struct {
    definition RigidBody
    mass       decad.MassProperties // valid only for Dynamic
}

// worldPair is one unordered body pair in canonical world order (a < b).
type worldPair struct {
    a, b        int
    excluded    bool
    moving      bool               // at least one body is not Fixed
    restitution units.Value        // effective pair value, rigid-dynamics "World and State"
    friction    frictionCoefficient
}

type World struct {
    doc    *decad.Document
    bodies []worldBody   // insertion order
    pairs  []worldPair   // canonical order: (0,1), (0,2), …, (1,2), …
    index  map[*decad.Body]int
    step   StepConfig
}

func (w *World) Bodies() []RigidBody   // copies, world order
func (w *World) Pairs() []BodyPair     // every pair in canonical order, excluded ones included
```

Pair material mixes per pair exactly as `docs/rigid-dynamics-design.md` "World and State" states; the
`frictionCoefficient` interval lives on the pair, never on the world. An excluded pair is not mixed. A
Fixed/Fixed pair enters no response, so a friction mean outside the finite nonzero range is refused only for a
pair with a moving body. `NewWorld` returns `ErrUnsupported` for a positive-friction pair with a moving body
whose family §6.4 cannot certify in the current phase. A non-excluded Fixed/Fixed pair is queried once per
`Step` at its constant poses by `ContactPair`; an `Overlapping` or `Undecided` relation makes the step
`Undecided` with `StepFixedPairRelation` (§12). Exclude such pairs, or model a tray as one body, as the exit
scenes do.

### 3.2 State

```go
type State struct {
    world    *World
    entries  []BodyState   // world order
    contacts []int         // canonical keys of the pairs resting on a persistent track here
    cache    *contactCache // immutable reuse cache, nil without one
}
```

`Entries()` and `Body()` keep their contracts. `NewState` requires exactly one entry per world body and
sets neither of the last two fields. The scheduled step sets both on the state it publishes as `Next`,
which is also its trace's end; every state derived from another by changing an entry carries neither,
since both describe one set of entries.

The CONTACT SET is part of the physical state, like a velocity: it names the pairs whose last certified
relation is a persistent track through the end of the step (§5 step 8), and §5 step 2 reads it. It lives
on `State`, not in the cache and not in the `Trace`: a step reads only its input state, the next step
needs it, and it decides a pair's start policy rather than repeating a computation. `World` stays
immutable configuration and every per-step datum stays in the run, the published state or the report,
so concurrent steps from one state share nothing mutable.

One rule keeps a resting or continuing contact silent: it publishes an event only when a solve changes
something. A pair the contact set carries (§5 step 2) continues on its track from the step start with no
solve. A touch met afresh, a pair the input state's contact set does not hold, stops the first slice at
zero, and its island is silent when it changes nothing (§6.1); it then joins the contact set the same
way. The two paths publish
the same events; the carried one saves the stop, its `StopAtInitialContact` sweep and the solve. A kicked
pair is never carried, so a stack resting under gravity is solved every step and publishes its support
impulses, as the two- and three-body steps do.

The cache is pure reuse: dropping it changes no published value except `Solver.Iterations`. It holds
every `SweptBox` and `SweepPair` result the step used, keyed by their exact inputs (§5.3), and every
island the step certified, with its exact problem and final proposal state (§6.2). A
step reads it only when its entries equal the input state's entries.

### 3.3 Step configuration and report

`StepConfig` gains one field:

```go
MaxPairSweeps uint64 // positive; counts SweptBox and SweepPair calls per step across every slice
```

A call the step answers from its own record or the input state's cache (§5.3) is not charged. `NewWorld`
refuses a zero `MaxPairSweeps` for every world; only the scheduled step charges it.

`StepReport` gains `Islands []IslandReport`; `ContactEvent` gains `Island int`, the index of the island
that published it, or `-1` for a graze or transition, which enters no solve; `StepDiagnostic` gains typed
fields (§12).

```go
// IslandReport is one simultaneous solve: the bodies and pairs that shared
// constraints at one event time, and the certified residuals of that solve.
type IslandReport struct {
    Time   units.Value     // from the start of the step
    Bodies []*decad.Body   // world order; Fixed and Kinematic participants included
    Pairs  []BodyPair      // world order
    Events []int           // indices into StepReport.Events
    Solver ContactSolverReport
}
```

### 3.4 Trace

```go
type Trace struct {
    start, end State
    duration   units.Value
    slices     []traceSlice
    events     []traceEvent
}

// traceSlice is one event-free interval of the step. Every body drifts on
// paths[i]; every scheduled pair carries the rounded certificate that proves
// those paths, or the swept-box exclusion that made no sweep necessary. The
// paths and certificates run over [start, span], span being the step's end;
// an event at end cuts the slice, and only its prefix is replayed.
type traceSlice struct {
    start, end units.Value       // held times from the step start, compared exactly
    span       units.Value       // held end of the paths and certificates
    from, to   State
    paths      []decad.PairPath  // world order: Fixed or Kinematic PoseSegment, Dynamic RigidDriftSegment
    proofs     []pairProof       // scheduled pairs, canonical order
}

type pairProof struct {
    pair     int                  // index into World.pairs
    sweep    *decad.SweepReport   // rounded certificate over [start, end]
    boxClear bool                 // §4.2: both swept boxes strictly disjoint; sweep is nil
}

type traceEvent struct {
    at        *big.Rat
    pre, post State
    islands   []int // indices into StepReport.Islands
}
```

§7 owns how `Sample` reads these.

## 4. Pair schedule and certified broad phase

### 4.1 The schedule

A step's pair schedule is the list of `worldPair` entries that are not excluded and whose `moving` flag
is set, in canonical order. Fixed/Fixed pairs are outside the schedule (§3.1). The schedule is fixed for
the step; what changes per slice is which scheduled pairs are CANDIDATES, decided by §4.2, and which
candidates need a fresh sweep, decided by §5.3.

### 4.2 Swept boxes in decad

The whole-path swept box of `docs/contact-sweep-design.md` §4.2 becomes a public read-only query so one
implementation owns the certificate and `dynamics` consumes it:

```go
// SweptBox encloses every point a body occupies at any time of its path: the
// body's bounded Bounds() at its placement, mapped through the path's From,
// expanded on every axis by the path's whole-duration travel bound
// (contact-sweep §4.1), or for a translating path the exact hull of its start
// and end boxes. Its extremes are exact dyadic rationals.
type SweptBox struct { /* lo, hi [3]proof.Dyadic (internal/proof); body *Body */ }

func (d *Document) SweptBox(ctx context.Context, b *Body, path PairPath) (SweptBox, error)

// StrictlyDisjoint reports a positive gap on at least one axis, compared exactly.
// Boxes that merely meet are not disjoint.
func (s SweptBox) StrictlyDisjoint(o SweptBox) bool
func (s SweptBox) Box() Box // outward float conversion of the exact extremes
```

`SweptBox` validates the body and path as `SweepPair` does and returns `ErrUnsupported` when the body's
bounds or the travel bound are not finite (contact-sweep §4.1: an unbounded radius or velocity). A rotating
`PoseSegment` takes the §4.1 screw formula over its read screw, whether or not `SweepPair` admits that
screw, and a rotating `RigidDriftSegment` takes the drift formula; both measure `ρ` from the rotation line.
A path that only translates — a `PoseSegment` whose endpoints share a basis, or a drift with zero `ω` —
takes the exact hull of its start box and that box moved by the exact displacement instead: every
intermediate translate lies in the hull, and the hull lies inside the travel expansion, so a stationary
path has zero travel. The zero `SweptBox` is never `StrictlyDisjoint` from any box. `sourceCylinderFaceSweep`
proves its clear outcome from the face-axis gap between the two `SweptBox` values, and
`sweepBoundedFacetedFloorClear` from their strict disjointness; both read the private `sweptBoxOf` that
`Document.SweptBox` wraps.

### 4.3 Sort-and-sweep in dynamics

Per slice, `dynamics` computes one `SweptBox` per body on its slice path (a Fixed body's box is computed
once per step and reused). It sorts bodies by `Box().Min.X`, ties by world index, and walks the sorted
list keeping an active set: a pair whose `x` intervals overlap is then tested with `StrictlyDisjoint` on
all three axes. A scheduled pair is a candidate when it is not strictly disjoint. The walk reads the
outward float `Box()`, the only extent `SweptBox` publishes: an active body leaves the set only when its
`Box().Max.X` lies strictly below the next body's `Box().Min.X`, and since the float extents enclose the
exact ones, that body's exact `x` interval is then strictly apart from the next body's and from every
later one. That is `StrictlyDisjoint`'s own certificate on the `x` axis, so the float walk discards no pair
the exact test would keep; boxes that merely meet stay in the set and reach `StrictlyDisjoint`. The candidate list is
sorted into canonical pair order before use, so no sort tie or walk order reaches a published result;
`docs/contact-sweep-design.md` §4.2 already permits a sweep-and-prune that discards only by the strict
box certificate and changes no order. A pair the boxes exclude is recorded in the slice as
`pairProof{boxClear: true}` and is never swept.

Cost per slice is `O(N log N)` box work plus one `SweepPair` per candidate that §5.3 cannot reuse. The
step charges each `SweptBox` call and each `SweepPair` call against `MaxPairSweeps` (§12, §13 PR 8).

The step sweeps every candidate and reports one diagnostic per candidate that does not prove clear, in
canonical pair order, so the set of diagnostics does not depend on world insertion order. A swept body's published end pose is the pose its every pair
certificate replays at the slice end through `CertifiedPosesAtInterval`; a refused or different pose
makes the step `Undecided` with `StepPairUndecided`.

## 5. Event schedule

The step keeps the structure `docs/rigid-dynamics-design.md` "Step and event schedule" specifies — one
full-step kick, then event-driven drift — and generalizes the body count. In order:

1. Validate inputs, drivers and loads; reject before reading `ctx`. Kick every dynamic body once.
2. Set `t = 0`. Set the CONTACT SET to the input state's contact set (§3.2), less every pair with a
   kinematic body and every pair the kick changed: a pair stays only when the kick left both bodies'
   entries exactly as the previous step published them, so the track that ended that step continues
   under the same velocities. A kicked pair starts under `StopAtInitialContact` and reaches its island as
   an initial contact, as a resting stack under gravity does every step. A state from `NewState` has an
   empty contact set.
3. **Slice.** Build each body's path for `[t, dt]`: a Fixed body a constant `PoseSegment`, a Kinematic
   body its driver sliced to the remaining time (from its current pose to the driver's end pose), a
   Dynamic body a `RigidDriftSegment` from its current pose, world mass center and velocities. Run §4.3 to
   get the candidates. For each candidate that §5.3 does not reuse, call `SweepPair` with the policy §5.2
   assigns. The paths' duration is `dt − t`, at the nearest float when that difference is not one: a slice
   and its certificates replay over the held span `[t, dt]` through `CertifiedPosesAtInterval`, so the
   duration parameterizes the spatial path and the held times label it.
4. **Classify.** A candidate whose report is `SweepUndecided` makes the step `Undecided` with
   `StepPairUndecided`, wherever its unresolved interval starts: an undecided report replays no prefix, so
   it cannot certify the slice up to a later event either. A persistent track that does not span the slice
   within `PenetrationResidual`, or a pair continued in contact whose sweep starts touching or overlapping,
   is `StepTrackUnproved`. The CUTTING events are an initial contact (only under `StopAtInitialContact`)
   at fraction zero, and an `ImpactBracket` or `ContactTransitionBracket` at its bracket's exact right
   fraction; a touch that ends inside a slice reaches the step as a transition bracket. Choose the
   earliest cutting fraction `f_e`, and gather the cutting events at exactly `f_e`. A bracket that only
   overlaps `f_e` is not gathered: its pair is swept again from `t_e`, as an initial contact when it
   touches there. A `GrazingTouch` cuts nothing (step 6).
5. **No event.** Advance every body to `dt` on its certified path, run `ContactPair` on every pair in the
   contact set at the completed poses, reject a relation other than separated, touching, or overlapping
   with a bounded manifold whose penetration lies within `PenetrationResidual` (two co-moving bodies drift
   on separately rounded translations and may overlap by an ulp), record the final slice, and publish.
   The published state's contact set is every pair continued under `ContinueCertifiedTouch` whose final
   sweep is a persistent track through `dt`.
6. **Advance** every body to `f_e` on its certified path: a swept body takes the pose its pair reports
   replay at `f_e` through `CertifiedPosesAtInterval`, and every report covering it must replay the same
   pose; a body no sweep covers takes the pose `pathPoseAt` evaluates on its own path (§5.4), as
   `Trace.Sample` does. Every candidate records its full report, re-sliced to `[t, t_e]` through `CertifiedPosesAtInterval`; an
   event pair's own report covers its prefix through the bracket's right fraction, so it needs no separate
   prefix sweep. A rotating pair's impact bracket replays only through its left end, so its impact is
   `StepPairUndecided`. Every box-excluded pair is checked at the rounded poses (§7.1). The event's held time
   `t_e` is the exact `t + f_e·(dt − t)` when that is a float, else the float just below it, so every time
   the prefix replays maps to a fraction at or below `f_e`; a label that does not pass `t` is
   `StepUnsupported`. Each graze before `f_e` publishes a zero-impulse `ContactGraze` at its instant, after
   its pair's report replays its poses there and its enclosed relative normal speed at them lies within
   `VelocityResidual` of zero; its pair's report replays the whole slice, so the graze needs no cut. A
   graze at exactly `f_e` is `StepUnsupported`.
7. **Islands.** A transition publishes a zero-impulse `ContactTransition` and its pair leaves the contact
   set without entering a solve. The impact and initial-contact pairs, with every contact-set pair whose
   persistent track covers `f_e` (its manifold read through `ManifoldAt(f_e)`), form islands over the
   active constraints at `t_e` (§6.1); only the islands an impact or initial contact reaches are solved,
   so a resting stack elsewhere keeps drifting and publishes nothing. Correct positions per island
   (§6.6), solve and certify each island in world order (§6.2–§6.4), publish one `IslandReport` and the
   island's events.
8. Update the contact set: pairs the solve left touching with a nonpositive relative normal speed stay
   in; pairs every solved normal speed exceeds `VelocityResidual` leave it; a departing pair leaves it
   once its departure completes at or before `t_e`, a clear pair and a pair the broad phase drops leave it
   at once. Set `t = t_e`. When the published events reach `MaxEvents` and `t_e < dt`, the step stops
   with `StepEventBudget` after this event, which stays in the certified prefix (§12); a graze that
   reaches `MaxEvents` stops it the same way. Otherwise go to step 3; an event at `dt` itself completes
   the step from the post-event state, whose contact set is every pair the event left under
   `ContinueCertifiedTouch`.

### 5.1 Zero-time repeats and Zeno sequences

A repeated event at the same `t_e` is legal only as a resting solve: a pair that is still touching and
closing after an island solve re-enters §5.2's `ContinueCertifiedTouch` policy on the next slice, which
either certifies a persistent or band track (no event) or brackets a transition. A third event time at one
clock reading has made no progress and is `Undecided` with `StepUnsupported`. A pair that bounces with
restitution produces one `ContactImpact` per bounce; `ImpactSpeed` is the guard that ends such a sequence,
because an incoming normal speed at or above `-ImpactSpeed` targets zero post-impact speed
(rigid-dynamics "Response") and the pair then rests. A scene whose bounces do not fall under `ImpactSpeed`
before `MaxEvents` is `Undecided` with `StepEventBudget`; that is the stated limit, not a defect.

Each impact lies at its bracket's right sample, at most one `TimeResolution` after the slice's exact
contact time, and §6.6 puts the pair back in exact touch there, so the `k`-th impact of a bounce lies at
most `k·TimeResolution` after its closed-form time.

### 5.2 Start policies

| Pair state at the slice start | Policy |
|---|---|
| Not in the contact set | `StopAtInitialContact` |
| In the contact set, every solved normal speed `> VelocityResidual` at the last solve | `ContinueSeparatingTouch` |
| In the contact set otherwise | `ContinueCertifiedTouch` |

These are the three policies `docs/contact-sweep-design.md` §5.1 defines; nothing here adds one.

### 5.3 Reusing a certificate across slices and steps

`SweepPair` and `SweptBox` are pure functions of their bodies, paths and request, so a call whose inputs
repeat exactly returns the report it returned before. The step reuses such a report with no new call:
a `SweepPair` whose pair, both paths and request (duration-derived `TimeResolution` and start policy
included) equal an earlier call's, and a `SweptBox` whose body and path do; a stationary `PoseSegment`'s
box depends on its pose alone, so a Fixed body's box is computed once. The earlier call may lie in an
earlier slice of the same step or in the previous step, through the input state's cache (§3.2). Inputs
repeat when an event at a slice's start leaves a body's state unchanged (a zero-time event, or an
initial contact elsewhere) and when a resting body starts a step where it started the last one: the
resting pyramid's every step after the first calls neither for its pairs. A reused report is the
report a new call would return, so reuse changes no certified outcome, no published pose and no trace
sample, and costs nothing against `MaxPairSweeps`.

Every other candidate is swept again from the new slice start. Mapping the remaining interval onto an
earlier slice's report through `CertifiedPosesAtInterval`, keeping the bodies outside the event on their
earlier paths, would save more calls but change the published poses: a body moved along its earlier path
lands on different rounded poses than one moved along a path rebuilt at the event, and an event fraction
mapped onto the earlier report is in general not a float, which §5 step 6 refuses. The step therefore
reuses only exact repeats. A kinematic body's path is sliced, so a pair containing one repeats only when
an event at the slice start leaves its driver slice unchanged.

### 5.4 Bodies outside every candidate pair

A body none of whose scheduled pairs is a candidate in a slice drifts on its own path; the swept-box
exclusions of its pairs are its certificate for the slice, and `Trace.Sample` evaluates its path directly
(§7.1). It still receives its kick and appears in the conservation readings.

## 6. Islands and the certified projected solver

`docs/rigid-dynamics-design.md` "Response" owns the law: the relative velocity, the restitution target
with `ImpactSpeed`, nonnegative normal impulses, complementarity, the Coulomb disk, the deterministic
tangent basis, fixed processing order, no hidden tolerance, and residual gating at `MaxIterations`. This
section owns how an island is formed, how the solve proposes impulses, and how every published number is
certified in exact arithmetic over the admitted intervals.

### 6.1 Island formation

An ACTIVE CONSTRAINT at `t_e` is one manifold point of: an `ImpactBracket` event's `SweepEvent.Manifold`;
an initially touching pair's `InitialEvent.Manifold` whose relative normal speed is closing or zero within
`VelocityResidual`; a persistent or band track's `ContactTrack.ManifoldAt(fraction)` at `t_e`; a
kinematic/dynamic contact of any of those forms, with the driver's contact-point velocity as
rigid-dynamics "Step input and configuration" defines it. A kinematic participant's driver slice must
translate, and its exact velocity is the slice's displacement over its duration; a rotating driver in an
island is `Undecided` with `StepUnsupported`. A pair gathered for the event with no
consumable manifold (`Manifold == nil`, or point or normal bounds wider than `StepConfig.Contact`) makes
the step `Undecided` with `StepManifoldMissing`.

A pair enters the solve when any of its points is active: the lower end of that point's relative normal
speed, enclosed over the mass-center, witness and normal balls, is at most `VelocityResidual`. All of an
entering pair's points then join the solve; a pair whose every point certainly separates faster than that
keeps no constraint and continues under `ContinueSeparatingTouch`.

A solved island that changes nothing publishes nothing: when every point's enclosed pre-solve normal speed
lies within `VelocityResidual` of zero, every certified normal and tangent impulse is exactly zero, every
dynamic body keeps its exact pre-solve velocities and §6.6 moves no body, the island publishes no event and
no `IslandReport`, and its pairs join the contact set under `ContinueCertifiedTouch`. A stationary or
sliding touch at the step start therefore continues on its track without an event, as in the two- and
three-body steps; in a later step the carried contact set continues it without this solve (§3.2).

Islands are the connected components of the graph whose vertices are DYNAMIC bodies and whose edges are
active constraints between two dynamic bodies; a constraint against a Fixed or Kinematic body attaches
that body to the dynamic body's island without joining islands through it (a floor under two separate
stacks does not couple the stacks). Components are numbered by the smallest world index of their dynamic
bodies and solved in that order. An island with no dynamic body cannot occur; a closing constraint with no dynamic
participant is `Undecided` with `StepIslandDegenerate`, as rigid-dynamics "Response" requires.

### 6.2 Unknowns and proposal

Per active constraint `k` the unknowns are `λn_k >= 0` and a tangent pair `λt_k ∈ R²` in the
deterministic basis of rigid-dynamics "Response". Let `u` be each body's pre-solve velocity (kicked plus
earlier events this step). The proposal runs projected Gauss–Seidel in `float64`, in the fixed order
islands → pairs (canonical) → manifold points (manifold order), for at most `MaxIterations` sweeps:

```text
w_k        = relative contact velocity at k from the current velocities
Δλn        = max(0, λn_k + (target_k − w_k·n_k) / K_nn,k) − λn_k;   apply ±Δλn·n_k and r×(Δλn·n_k)
Δλt        = −w_k,t / L_k;   λt_k += Δλt;   project λt_k onto the disk of radius μ_k·λn_k;  apply
```

`K_nn,k` and the 2×2 `K_tt,k` are the usual effective-mass terms from the nominal inverse mass and
inverse world inertia, and `L_k = max(K_t1t1, K_t2t2) + |K_t1t2|` bounds the largest eigenvalue of
`K_tt,k`. The tangent row steps by that scalar rather than by `K_tt,k⁻¹`: at a fixed point on the disk's
rim the impulse is then opposite the slip `w_k,t` itself, which the slip gate of §6.3 requires, where a
`K_tt,k⁻¹` step would leave it opposite `K_tt,k⁻¹·w_k,t`. The proposal is a nominal solution and proves
nothing; `μ_k` here is the nominal mean the pair's `frictionCoefficient` carries. A nominal proposal that
is not finite, or a closing constraint whose nominal `K_nn,k <= 0` or `L_k <= 0`, is `Undecided` with
`StepIslandDegenerate`.

An island of at most eight manifold points starts the sweeps from a DIRECT solution of its contact
problem at the pre-solve velocities. Active point sets are tried from the largest to the smallest, ties
by bit order, and each solves `K_AA·λ_A = target_A − w_A` for its rows by the minimum-norm pseudo-inverse
of the symmetric `K_AA` (a cyclic Jacobi eigen-decomposition dropping eigenvalues at or below `2⁻⁴⁰` of
the largest, so an indeterminate patch such as a box face's four corners takes the even split). A set is
accepted when its normal impulses are nonnegative and every row meets its target within
`VelocityResidual/16`, an inactive normal row at or above it. When a point has friction, the STICKING
solution is tried first: each active friction point adds its two tangent rows with target zero, and each
point's tangent impulse must lie in its nominal disk within `ImpulseResidual/16`. When the minimum-norm
split leaves a corner outside its disk, the friction of each patch (points sharing their bodies and
normal) is split again in proportion to the points' normal impulses, and the rows are rechecked. A
sticking start that no set admits falls back to the frictionless one. The direct start proves nothing: it
only lets a small island reach a solution the certificate accepts within a few sweeps, as the closed
forms do in one.

Without a direct start, the tangent rows of positive-friction pairs join the sweeps once a sweep of the
normal rows alone changes no impulse in `float64`; with one, they run from the first sweep. Friction then
starts from the frictionless or sticking contact state: a resting patch has stopped spinning there, so
its sticking corners do not turn the transient spin of the first normal sweeps into self-cancelling corner
friction. The sweeps run until one changes no impulse in `float64`, or `MaxIterations` have run; an
island whose normal rows reach no fixed point within the budget is certified without friction and refused
by its stick rows if any point slides. Each point's world tangent
impulse `λt_k` is published as `t1·λt1 + t2·λt2`, rounded once, with a component within `1/16` of
`ImpulseResidual` of zero published as exactly zero. The published post velocities are then recomputed
once from the pre-solve velocities and the final normal and published tangent impulses in the fixed
order, and a component within `1/16` of its residual (`VelocityResidual` or `AngularVelocityResidual`) of
zero is published as exactly zero; a spin component snaps only while it is also within `VelocityResidual/16`
divided by the body's longest island lever, so a body rolling at a few rad/s under a coarse
`AngularVelocityResidual` keeps the spin its sticking contacts need. Co-moving dynamic bodies then publish one common velocity: two dynamic
bodies of an island pair are co-moving when their published spins are equal and every component of their
linear velocities differs by at most `VelocityResidual/8`; each connected group of them whose every member
lies within `VelocityResidual/16` of the group's mass-weighted mean, per component, publishes that mean for
every member. Float rounding leaves a bouncing or resting stack's bodies about `1e-14` to `1e-7 mm/s`
apart, and only exactly equal velocities let `SweepPair` prove the touch that continues the stack, as the
two-body sphere path's common zero-restitution velocity does for one pair. These choices serve the
continuation: a resting island must publish exactly equal normal velocities on its touching pairs, and
exactly zero spin, before `SweepPair` can prove a persistent touch from it. None of them is a claim; the
certificate judges the published values, so a common velocity whose change from a member's own fails the
linear law within `ImpulseResidual + m_hi·VelocityResidual` is refused like any other proposal.

WARM START. The input state's cache (§3.2) holds every island the previous step certified: its problem
(bodies, their pre-solve entries and driver velocities, and each pair's manifold points, whose
`ContactFeature` pairs are part of the match, so nothing keys by a topology slice index), the final
state of the sweeps (each point's normal and tangent impulses and each body's nominal velocities), the
sweep count, and whether the last sweep changed no impulse. An island whose problem equals a cached one
exactly restarts from that state. When the cold solve ended at a fixed point, the restart's first sweep
is the cold solve's last sweep, which changed no impulse, so the solve publishes the cold solve's
proposal after one sweep; `Solver.Iterations` is the only published difference. When the cold solve
ran to `MaxIterations` without one, further sweeps would move the proposal, so the restart publishes
that proposal as it stands, with its sweep count. Whether a solve reaches an exact fixed point before
`MaxIterations` depends on the host's float contraction: the pyramid's does on amd64 and need not on
arm64. Either way the certificate judges the republished proposal afresh. An island whose problem
differs in any bit starts cold. Restarting such an island from the cached impulses of matching
pairs and features would move a statically indeterminate island, such as a four-corner patch, to a
different split of the same total, and the published impulses would depend on the cache; the cache never
changes a published impulse.

### 6.3 Certification

After the last sweep the solver certifies the published proposal in exact rational interval arithmetic,
forward only:
it never inverts an interval tensor. The interval vocabulary is `internal/proof/interval.go`'s
`RatInterval` with `AddInterval`, `SubInterval`, `NegInterval`, `MulInterval`, `ScaleInterval` and
`PointInterval`; `dynamics` imports that package directly. The two vector forms this section needs and
that file lacks, a three-component interval dot product and cross product, are added to the same file
(§13 PR 4) so the root package and `dynamics` share one implementation; no interval code lives in
`dynamics`.

Inputs read as intervals: each dynamic body's mass `[m_lo, m_hi]` and six inertia components, rotated into
world axes with the pose basis read as exact rationals and widened by the basis's orthonormality defect
(`docs/dynamic-mass-design.md` §4); each manifold point's `OnA`, `OnB` and `Normal` with their balls; the
pair's `frictionCoefficient` interval `[μ_lo, μ_hi]`; the published pre-solve velocities as exact rationals;
and each kinematic participant's exact driver velocity, which the event does not change. The proposal's
published outputs are the per-point impulses `(λn_k, λt_k)` and each body's post velocities `(v', ω')`,
rounded to `float64` and read back as exact rationals.

The certificate then checks, every comparison over the full interval:

| Gate | Exact statement | Limit |
|---|---|---|
| Linear law | `m·(v' − v) − ΣJ` per component, over both mass endpoints | `ImpulseResidual + m_hi·VelocityResidual` |
| Angular law | `I_world·(ω' − ω) − Σ r×J` per component, over the inertia and point intervals | `ImpulseResidual·ρ + λ_lo(I)·AngularVelocityResidual`, with `ρ` the island's largest lever bound and `λ_lo(I)` the certified lower eigenvalue (rigid-dynamics "World and State") |
| Normal sign | `λn_k >= 0` exactly | none |
| Non-penetration | `w'_k·n_k − target_k` lower end | `>= −VelocityResidual` |
| Complementarity | if `λn_k > 0`: `|w'_k·n_k − target_k|` upper end | `<= VelocityResidual` |
| Cone | `‖λt_k‖ − μ_lo·λn_k` | `<= ImpulseResidual` |
| Stick | if `‖λt_k‖ < μ_lo·λn_k − ImpulseResidual`: `‖w'_k,t‖` upper end | `<= VelocityResidual` |
| Slip | otherwise: `λt_k·w'_k,t + ‖λt_k‖·‖w'_k,t‖` upper end | `<= ImpulseResidual·‖w'_k,t‖ + VelocityResidual·‖λt_k‖` |
| Energy | island kinetic energy after minus before, over the mass and inertia intervals, minus kinematic work: at each point a driver on side `A` delivers `λn·(n·V)` and one on side `B` `−λn·(n·V)`, taken at the upper end over the normal ball | the allowance rigid-dynamics "Completion, conservation, and trace" states, summed over the island's bodies |
| Momentum | the per-event linear and angular momentum checks of the same section, applied to the island as one event set | as stated there |

The friction rows read `λt_k` as the published world tangent impulse, so `‖λt_k‖²` is exact: the cone
and the stick condition compare squares exactly, `w'_k,t = w'_k − (w'_k·n_k)·n_k` is enclosed over the
normal ball and the levers, and the slip row takes `‖λt_k‖` and `‖w'_k,t‖` at their upper ends on its
left side and at their lower ends on its right. `ContactSolverReport` publishes the largest attained
value of each gate as its residual, plus the sweep count: `NormalResidual` for non-penetration and
complementarity, `TangentResidual` for the stick row's `‖w'_k,t‖`, `ConeResidual` for the cone's excess
`‖λt_k‖ − μ_lo·λn_k`, `LinearResidual`, `AngularResidual`, `MomentumResidual` and
`AngularMomentumResidual` for the laws and the island momentum, and `EnergyResidual` as the signed upper
end of the island's kinetic-energy change; the slip row passes or refuses and publishes no residual.
`AngularUpper` is an upper bound on the largest published post-solve angular speed of the island's
dynamic bodies. A pre-solve normal speed
whose enclosure straddles `−ImpactSpeed` selects neither target; with a positive restitution that is the
`restitution target` refusal. The two momentum rows are implied by the per-body laws: the island sum of
`m·Δv − ΣJ` cancels each dynamic pair's impulses, and the sum of `I·Δω + c×m·Δv` is the sum of the
angular-law residuals plus each `c×` its linear-law residual, which the angular-momentum limit covers term by
term. A gate that fails at `MaxIterations` is `Undecided` with `StepIslandResidual`, naming the island,
the gate and the limit. Residuals are decided over intervals, so a proposal whose nominal value passes
but whose interval does not is refused; narrow source intervals keep the arithmetic residual the binding
one, exactly as rigid-dynamics "Response" states.

These gates are the solver's CLAIM, not an admission of a geometric fact: the published state is defined
as "velocities that satisfy the discrete law within the stated residuals", and the certificate proves that
definition holds. CLAUDE.md's reject-only rule governs geometric claims decad is handed; every geometric
input here (manifold, mass, normal) arrives already certified by its producer, and nothing in this table
upgrades one.

### 6.4 Friction families per phase

The cone, stick and slip gates make friction generic, so no per-shape friction solver remains after
§13 PR 5. What limits friction per phase is the manifold producer: a pair needs a bounded manifold with
point and normal balls that keep the slip and cone intervals inside the limits. Phase 1 families all
publish such manifolds; a positive-friction pair whose family publishes relation only (no manifold) is
refused at `NewWorld` with `ErrUnsupported`, as today.

### 6.5 Parity with the closed-form responders

Every shipped response fixture in `dynamics/*_test.go` — the `150 kg·mm/s` box rebound, the
`sqrt(5000)` tilted support, the four-corner Coulomb slide, the two-sphere `37.5 kg·mm/s` impact, the
coupled stack `JU = −mU·vU`, the sphere island active-set cases, the off-center `Iyy` tip, the sphere-floor
`Y` spin, the graze, the planar off-axis sphere-pair Coulomb impact and its interior-time form
(`sphere_pair_offaxis_friction_test.go`), the sphere sticking to two fixed orthogonal box faces
(`three_body_friction_island_test.go`), the isolated three-dynamic `75 kg·mm/s` sphere impact
(`three_body_all_dynamic_test.go`), the symmetric three-dynamic friction island with `2N+T=m(1+e)v` and
`N+(2+mr²/I)T=mv` and its later all-clear spinning step (`three_body_dynamic_friction_test.go`), the
cylinder sidewall rebound (`cylinder_sidewall_impact_test.go`), the rotated sphere's tangential impact
(`oblique_sphere_tangent_test.go`), and the positive-bound union's floor impact and rest
(`faceted_floor_step_test.go`) — runs unchanged through the general island solver and asserts the same
impulses, velocities and poses within its existing `InDelta` slack. A single-point island converges in
one sweep to the isolated formula exactly in float; the four-point and two-pair islands converge to the
same solutions the closed forms publish because those solutions satisfy the same complementarity
system. A fixture that does not pass through the general solver blocks §13 PR 5, which deletes the
closed-form responders: `three_body.go` and every `three_body_*.go` (`all_dynamic`, `dynamic_friction`,
`friction_island`, `island`, `sequential`, `sphere_island`, `stack`, `two_dynamic`), `friction_patch.go`,
`friction_pair.go`, `friction_pair_certificate.go`, `friction_pair_step.go`, `friction_step.go`,
`friction_impact.go`, `sphere_floor_friction.go`, `sphere_pair_friction.go`,
`sphere_pair_offaxis_friction.go`, `sphere_pair_response.go`, `oblique_response.go`,
`oblique_sphere_step.go`, `fixed_offcenter.go`, `cylinder_impact.go`. `grazing_step.go` stays, since a
graze is a schedule outcome, not a solve; `resting.go`, `kinematic*.go`, `load.go`, `material_mix.go`
and the conservation files are inputs and readings, not responders, and stay.

Where the general path and a closed form answer differently, the parity run follows these rules:

- A fixture whose closed form refuses an event the general path certifies keeps the general answer, and
  its assertion asserts the certified computed values instead of the refusal: the falling frictional
  box face of `TestFixedFloorFrictionRejectsUnsupportedMaterialsAndPatch`, the rotating driver of
  `TestKinematicDriverRejectsInvalidPaths`, the tangent approach of
  `TestObliqueSupportRefusesUnresolvedMotion` and the overlapping brackets of
  `TestThreeBodyTwoDynamicOverlappingPairEventsRemainUndecided`.
- A zero-speed touch publishes no event (§6.1), and `MaxEvents` stops a step when the published events
  reach it with time remaining (§5 step 8).
- Refusal wording and report shape follow §12: an `Undecided` report carries the events of its certified
  prefix and names the general path's reason.
- A test that calls a deleted responder directly is replaced by a test of the same fixture through the
  general path that asserts the same computed quantities.
- An initially touching pair takes §6.2's restitution target like any other contact. The two- and
  three-body steps rest an initially touching cardinal box face with zero restitution while an initially
  touching sphere pair or tilted face takes its restitution, and no rule on the approach speed or
  `ImpactSpeed` separates the two, so the assertions of `TestObliqueInitialTouchContinuesAsPersistentContact`,
  `TestRestingBoxUsesPersistentContactTrack` and `TestThreeBodySimultaneousCornerImpact` that encode the
  cardinal-face rest are rewritten to the restitution answer.

### 6.6 Position correction across an island

Rigid-dynamics "Response" bounds one pair's correction by certified geometry displacement plus bracket
travel plus `ContactSlop`, and requires every correction to be swept against every other pair. Across an
island: each dynamic body receives one translation, the sum over its island pairs of its inverse-mass share
of the pair's deepest point penetration along the pair normal (a Fixed or Kinematic body takes no share; a
penetrating pair whose points disagree on the normal is refused). The penetration is the one the rounded
event poses show: `ContactPair` at those poses for an interior impact, whose solve still uses the manifold
the sweep certified at the bracket's right sample, and the gathered manifold itself for an initial contact
or a track. Dynamic bodies joined by contact-set pairs on persistent tracks move as one: their touch is
exact and their velocities equal (§6.2), so they take one translation computed with their summed mass,
which keeps that touch. A group resting on a Fixed or Kinematic body through a persistent track is
anchored and takes no share either, so a body landing on a resting one is corrected alone and the
resting body keeps its exact touch with its support; a penetrating pair with no free body is refused. A
pair's penetration may not exceed its allowance above, its bracket travel bounded
by the bracket's elapsed width times an upper bound on the pair's contact-point speed (the L1 norm of the
linear velocity difference plus each body's spin times its lever, both L1 norms), and a body's
translation length may not exceed the summed allowances of the pairs that moved it, plus the band depth
`ε` of §10.3 when the slice ended on a band track. Island pairs with a moved
body must still be `Touching` at the corrected poses; every other scheduled pair with a moved body is swept
over the correction.

A translation along a float normal rarely lands two curved bodies in exact touch, so an island pair whose
solve separates it (every point leaves faster than `VelocityResidual`, §5.2) is PUSHED just apart when the
corrected poses leave it either way:

- still overlapping: its dynamic bodies move along the deepest point's normal by that point's depth plus
  its separation bound, split by inverse mass, the share doubled until it moves the rounded pose;
- apart by less than `ContactPair` can prove (`Undecided` with `ContactNoGapProof`): its dynamic bodies
  move apart along the event manifold's normal, split by inverse mass, by one ulp of their largest
  coordinate and then twice as far each time, until `ContactPair` proves the pair separated or touching.

The pair is then checked again, for at most four passes over the event's islands. It must end `Touching`
or `Separated`, and each pushed body's whole translation from its pre-event pose, measured as the
correction's, must stay within its correction allowance; a separated corrected pose within that allowance
is what rigid-dynamics "Response" admits at a separating impact. A pair that ends touching continues under
`ContinueSeparatingTouch`; one that ends separated leaves the contact set, so its next slice starts under
`StopAtInitialContact`, since the rotating sphere-pair sweep needs a touching start for a departure. A
pair the solve does not separate is never pushed and may not end separated, since its continuation needs
exact touch: a resting curved pair whose correction leaves an ulp of overlap is `Undecided` with
`StepCorrectionFailed`.

The root package's rotating sphere-pair sweep proves only clear and departing paths: two spinning spheres
whose clear start leads to another impact inside the slice return `SweepUndecided` with
`SweepContactUnsupported`, so two spheres a glancing frictional impact sets spinning stop the step at
their next impact (`StepPairUndecided`).

A sphere pair that continues in persistent touch after an interior impact is refused at its next replay:
the root package's sphere-pair persistent replay requires the two rounded centers to stay exactly one
radius sum apart at every replayed fraction, and centers corrected at the bracket's right sample are not
dyadic enough for both translations to round alike. No correction makes that hold in general (two centers
in different binades round the same displacement differently), and the certificate is not weakened to
admit it, so `TestThreeBodyTwoDynamicSphereZeroRestitutionImpact` does not pass through the general path. The corrections of one island are applied together, then every
candidate pair touching a corrected body is swept over the correction as a `PoseSegment` of zero
duration-independent travel (the usual §4.2 swept-box exclusion applies first). A new contact, a lost
relation or an undecided interval is `Undecided` with `StepCorrectionFailed`. Corrections are recorded in
the event and the trace and never claim to conserve energy.

## 7. Trace and `Timeline`

### 7.1 Sampling one step

`Trace.Sample(t)` keeps its signature. It compares the exact held `t` against the slice boundaries
(`big.Rat`), returns an event's `post` state at an exact event time, `start` at `0` and `end` at
`duration`. Inside a slice it produces every body's pose at the fraction `(t − start)/(span − start)` of
the slice's paths and certificates:

- a body in at least one `pairProof` with a sweep: `sweep.CertifiedPosesAtInterval(t, start, span)`;
  when two proofs cover the same body, their rounded poses must be identical (`ErrUnsupported`
  otherwise), which holds when both evaluate the same float path at the same exact fraction;
- a body whose candidate pairs are all `boxClear`, or that is in no candidate pair: its own `bodyPath`
  evaluated at the mapped fraction (`RigidDriftSegment` through `r3.RotationAround` and
  `r3.Translation` exactly as the sweep evaluates it);
- a Fixed body: its constant pose.

The swept boxes that excluded a pair enclose its ideal path, but a rounded pose may leave that path by an
ulp, so a pair whose exact gap is below an ulp could land in contact no certificate covers. Every
box-excluded pair is therefore checked at the sampled poses: the two bodies' bounds, mapped exactly
through those poses by `Document.SweptBox` along a stationary `PoseSegment`, must be strictly disjoint, or
`Sample` returns `ErrUnsupported`. The step runs the same check at every pose it publishes, a slice end or
an event's advance, and refuses with `StepPairUndecided`. The check reads the document, so the document
must not change while a trace is sampled.

Velocities inside a slice are the slice's `from` velocities. A sample a rounded certificate refuses
(`CertifiedPosesAtInterval` returns `ErrUnsupported` when the rounded pose leaves the certified relation)
makes `Sample` return `ErrUnsupported`, as today.

### 7.2 `Timeline`

```go
// Timeline chains the steps of one world from one start state. Its certified
// end is the completed time of the last Advanced step; it stops at the first
// Undecided step and never advances past it.
type Timeline struct { /* world; start State; steps []timelineStep; stop *StepReport */ }

var ErrTimelineStopped = errors.New("dynamics: timeline stopped at an undecided step")

func NewTimeline(w *World, start State) (*Timeline, error)
func (tl *Timeline) Advance(ctx context.Context, input StepInput, dt units.Value) (*StepReport, error)
func (tl *Timeline) End() units.Value           // exact sum of advanced durations
func (tl *Timeline) Steps() []*StepReport        // copies, in order
func (tl *Timeline) Stopped() *StepReport        // the Undecided report, or nil
func (tl *Timeline) Sample(t units.Value) (State, error)
```

`Advance` runs `World.Step` from the last `Next`; on `Advanced` it appends; on `Undecided` it records the
report as `stop` and returns it with a nil error (the step is not an error); a later `Advance` returns
`ErrTimelineStopped`; a step error leaves the timeline unchanged. `Sample(t)` locates the step by exact
cumulative time and delegates to that step's `Trace.Sample` with the local time, `t` less the step's exact
start; when that difference is not a float, the nearest float inside the step labels it, and the pose
returned is the certified pose at that label. `End()` is the exact sum at the nearest float when the sum
is not one. `t` equal to `End()` returns the last advanced step's `end` state,
which that step's certificate covers; `t` below zero or beyond `End()` is `ErrUnsupported`, and the
same test, "beyond `End()`", is the one §11.2's track applies. A step boundary belongs to the later
step's `start`, which is the earlier step's `end` by construction. The gallery reads only `Timeline`; it
never calls `Step` itself.

`Sample` is safe for concurrent calls and returns the same `State` for the same `t`: it reads `steps`
and `stop` and writes nothing, and every `Trace`, `pairProof` and sweep certificate it consults is
read-only once `Advance` has returned. `Advance` is the only writer; it must not run concurrently with
`Sample` or with another `Advance`. The gallery advances the timeline to the clip length before it
builds the scene (§11.2), so no render worker ever overlaps an `Advance`.

## 8. Mass-property extensions

`docs/dynamic-mass-design.md` owns the integrals and admission gates; this section fixes which payload
lands when and through which path, in the order below. Each item ships with the computed test of
dynamic-mass §6 for that shape.

### 8.1 Prism with a non-cardinal frame or placement basis

Analytic, in `mass_properties_rotated.go`: frame-local `V, P, Q` about `(0, 0, zm)`, `zm` the recorded mid
level, from `momentSecondOrder` section moments; then `M I Mᵀ` with `M` the product of the placement and
frame bases read as exact rationals. The reading is about the rigid rotation `Q` nearest `M` (its polar
factor). With `d` the entrywise absolute sum of `MᵀM − I`, an upper bound on its Frobenius norm,
`‖M − Q‖_F ≤ d`, so each world component widens outward by `3·d·(2+d)·m`, `m` the local tensor's
largest magnitude. Tensor positivity is proved before rotation: the local Gershgorin lower bound must
exceed the summed full widths of all nine published world entries, since a rotation keeps eigenvalues.
Cardinal, undisplaced prisms keep the signed-permutation path in `mass_properties.go`.

### 8.2 Prism with positive `z0Delta`, `z1Delta` or `sectionDelta`

Analytic plus an occupied-volume error
`E = SDA·(h + z0Delta + z1Delta) + A_upper·(z0Delta + z1Delta)`, `SDA` the section's
`sectionDisplacementArea` over its walk count and proven perimeter, `h` the recorded height and
`A_upper` the recorded area's upper end. It is charged as `E`, `R·E`, `R²·E` on `V`, each `P_i` and each
`Q_ij` per dynamic-mass §2.2, `R` the larger of the section envelope plus `sectionDelta` and
`h/2 + max(z0Delta, z1Delta)`. Phase 3's cap-blend and chamfer bodies need it; Phase 1 bodies have zero
deltas.

### 8.3 Loft

A solid loft's mesh is the exact restatement of its held triangles
(`docs/tessellation-design.md` §2's `loftPayload` row), so its mass takes §8.5's path and settles at
the ladder's first step: `mass_properties_mesh.go` integrates the held triangles through the
`tetraMoments` sums (the `/120` form) it shares with `mass_properties_faceted.go`, widened by the
mesh proof's `volSymDiff` as dynamic-mass §2.2 states. `loft_moments.go`'s accumulator is not
reused: its Volume and Centroid apply the exact twist corrections of chorded cells, which have no
second-moment counterpart, while the restated triangles are uncorrected and their four-leg
`volSymDiff` covers that gap.

### 8.4 Stitched planar solid, zero-bound Boolean, translation-placed Boolean

The Boolean path is `mass_properties_faceted.go`'s, over the payload's own held triangle set and
bound. A stitched solid takes §8.5's path over its restated triangles. Its mesh carries an
occupied-volume proof only when it is closed, all-planar and zero-bound at every vertex
(tessellation §2's `stitchPayload` row), so a placed or certificate-welded stitched solid returns
`ErrUnsupported`.

### 8.5 Generic `VerifyAll` fallback

For any payload whose `Tessellate(ctx, tol, VerifyAll)` reports `VolumeVerified()`: the dynamic-mass
§2.2 curved path over the mesh with `E = volSymDiff`, after the shell closure, orientation,
vertex-link and exact facet-crossing audits rerun on its triangles (the crossing audit is skipped
only where the tessellation ran its own facet-contact audit). The widening is per axis: with
`|x_i − O_i| ≤ R_i` over both the mesh and the denoted solid, `V` widens by `E`, `P_i` by `R_i·E`
and `Q_ij` by `R_i·R_j·E`. The tolerance ladder is deterministic, `tol_k = diameter · 2^−k` for
`k = 8 … 14`, `diameter` the body box's diagonal; integration stops at the first `k` whose volume
and tensor intervals pass the positivity proof. A mesh without an occupied-volume proof, a failed
audit or a tessellation refusal ends the ladder with that refusal, and an exhausted ladder is
`ErrUnsupported`. A revolve mesh reaches `k = 10` at most: its facet-contact audit refuses a finer
mesh at the fixed facet-pair ceiling. This covers lofts, exact stitched solids, cups, cap blends
whose band publishes tessellation-reach §7's proof, and any revolve the analytic path refuses; chain
payloads and curved stitched solids publish no occupied-volume proof and return `ErrUnsupported`.

### 8.6 General revolve, full or partial, any admitted section

Analytic, in `mass_properties_revolve.go`. `moments.go`'s `momentThirdOrder` adds `∫u³`, `∫u²v`, `∫uv²`,
`∫v³` as rational intervals about the plane origin, every segment kind in the one boundary form
`∮u^(p+1)·v^q dv/(p+1)`: lines there, arcs and circles in `moments_circular.go` through the trig-power
reduction `circularMonomials`, Tier A spans in `spline_moments.go`. The plane moments are re-expressed as
`∫z^a·ρ^b` in the exact axis frame and multiplied by the angular factors `∫dφ`, `∫cos φ`, `∫sin φ`,
`∫cos² φ`, `∫sin φ cos φ`, `∫sin² φ` over `[φ0, φ1]`, built from the payload's own sweep denotation
(`sweepDenotation.widthInterval`, `angleDenotation.sinCosFor`); a full turn's endpoints have exact sine
and cosine, so its odd factors vanish exactly and `π` enters through the width alone. Dynamic-mass §2.1
names exactly these terms; partial turns keep their mixed components. The local tensor reaches world axes
through §8.1's rotation and defect widening, and positivity is proved by the leading principal minors of
the published tensor. The path refuses an inexact axis, a nonzero axis-snap or admitted-band allowance, an
undenoted sweep end and a section displacement, none of which it charges.

### 8.7 Sweep

Analytic, in `mass_properties_sweep.go`. A single straight span takes 8.1/8.2; a single arc span takes
8.6 over the arc's partial revolve. A composite sweep integrates each `sweepSpanPayload` in its own local
coordinates and sums `V`, `P` and `Q` about one shared anchor; no parallel-axis shortcut per span, since
`P` and `Q` already refer to the shared anchor. Each span reaches the composite's unplaced coordinates
through its own rigid motion: the exact image of its local origin and the rotation nearest its held frame
matrix `F`, with `d` its orthonormality defect, widening each `P_i` by `d·‖P‖₁` and each `Q_ij` by
`3·d·(2+d)·m` as 8.1 does. The sum reaches world axes through the one placement every span shares, by
8.1's rotation, and positivity is proved as in 8.6. A span placement that differs from the first span's,
or a span either path refuses (an arc span whose axis is not exact in its transported plane), leaves the
sweep to 8.5.

### 8.8 Cup

Analytic, in `mass_properties_cup.go`. Outer prism minus cavity prism (both 8.1/8.2), subtracted at the
`V, P, Q` level with each contribution's own outward interval (dynamic-mass §2, §3), the cavity re-anchored
exactly onto the outer prism's mid level first. `cupPayload` holds both sections and the three levels with
their deltas; each level delta is charged on its prism, and the offset section's displacement
(`offsetDelta`, `docs/modify-design.md` §9: the thickness conversion and the offset solve's rounding together)
is charged as that region's `sectionDelta`, on the cavity for an inward cup and on the outer region for an
outward one. The difference reaches world axes by 8.1's rotation and positivity is proved as in 8.6. A
cup either prism refuses leaves the cup to 8.5.

A dynamic body whose payload matches no item returns `ErrUnsupported` from `Body.MassProperties`, and
`NewWorld` rejects it with that error; a Fixed or Kinematic body never needs mass.

## 9. Exact planar faceted contact

This is `docs/contact-geometry-design.md` §7's stage C3 made concrete for the solids whose held boundary
is exact: every vertex an exact dyadic rational at the query pose (the recorded coordinates times the
placement and query transforms with exact products and sums, as `sourceOrientedBoxAtPose` does for a
box) and every face planar with a source normal read exactly off its vertices. That is: a prism whose
section is all `LineSeg` with zero deltas, at any proper pose; a stitched all-planar solid with zero
`delta`; a zero-bound Boolean or its translation-only placement; a loft whose stations are exact
(`delta == 0`). A body with a positive boundary displacement is Phase 3 (§10.4).

### 9.1 Relation for two exact planar bodies

The relation of two such bodies at exact poses is decided by exact rational tests over their triangle
sets, prefiltered by per-triangle boxes as `meshBoolean` does and charged to the shared `workBudget`
with `ctx` polled every `workPollInterval` operations. The split follows the one `contact_box.go` and
`internal/pair/axis_box.go` already make: `internal/pair/planar.go` takes the two exact vertex and
triangle snapshots and returns the relation, gap and feature pairs over `proof.Dyadic`; the root
`contact_faceted_pair.go` admits the bodies, builds the snapshots at the query poses, maps feature
indices back to live `*Face` values and publishes the typed report. A prism's caps reuse the
tessellator's cap triangulation of its recorded section, accepted only after an exact check that every
cap triangle is counterclockwise; with the closed-mesh audit, that proves the triangles tile the section
exactly. The tests are:

- a certified transversal crossing of two facets (`boolean_exact.go`'s predicates) proves `Overlapping`;
- with no crossing, one nesting cast per shell (`docs/clearance-design.md` §2's ray ladder, closed-form
  for planes) proves containment, hence `Overlapping`, or mutual outsideness;
- with no crossing and no nesting, the exact minimum over facet pairs of the squared distance
  (vertex-face and edge-edge candidates, rational) is either positive, proving `Separated` with the gap
  enclosed between two floats whose squares are compared exactly against it, or zero, proving `Touching`
  when every zero-distance feature pair has opposed material sides.

Two coplanar facets with matching outward normals and a positive-area overlap also prove `Overlapping`.
A shell whose every vertex lies on the other body's boundary needs no cast: it meets the contact set.
"Opposed material sides" is a local proof at each zero-distance site (a vertex on a facet, two crossing
edges, a collinear edge overlap, a coplanar edge chord through a facet), taken over every triangle of
each body that holds the site. It holds when either a plane through the site has one body's triangles
on or behind it and the other's on or in front, with a triangle of each body whose own plane bounds that
body's triangles there, or one body's triangles lie in front of every plane of the other's there (a box
in a tray corner). A site where neither holds leaves the relation `Undecided` with
`ContactAmbiguousFeature`. Known limit: two flush bodies meeting at a saddle point (side by side on one
base plane, one filling the other's notch) are such a site, so their touch stays `Undecided`; proving it
needs a complete local-cone decision, which this design does not include.

A convex body's relation against a convex body may shortcut through separating axes (face normals and
edge cross products) exactly as `classifyOrientedSourceBoxes` does; the triangle path is the general one.

### 9.2 Convexity certificate

A body is CONVEX when every held vertex lies on or behind every facet plane, tested as exact rational
signed volumes, and its mesh passes the solid audits of `docs/tessellation-design.md` §1. The certificate
is computed once per body at its placement and cached on the body (it is invariant under any affine
pose with a positive determinant, which is every pose §9 admits). A dynamic body without it publishes
relations only, with the appended reason `ContactNonConvex` on an absent manifold. A Fixed or Kinematic
body needs no certificate. `ContactPair` does not know motion types: it names `ContactNonConvex` on a
touching or overlapping pair when neither body carries the certificate, since §9.3 needs one convex side.

### 9.3 Manifold

At a certified touch of convex `A` against planar-faced `B` (either order), the contact set is a union of
feature pairs at zero distance. Publish a manifold only when the set is one of:

| Contact set | Points published | Normal |
|---|---|---|
| Face of `A` coplanar and opposed to a face of `B`, positive area | every extremal vertex of the planar patch, §9.4 | the `B` face normal (exact), oriented `A` toward `B` |
| Edge of `A` in a face of `B` (or a face of `A` on an edge of `B`) | the clipped segment's two endpoints | the face normal |
| Vertex of `A` in a face of `B` (or the reverse) | one point | the face normal |
| Edge of `A` crossing an edge of `B`, non-parallel | one point | the normalized cross product of the edge directions, oriented by material side; its ball from the exact cross product's `proof.DySqrtDown`/`proof.DySqrtUp` length |
| Several of the above on distinct faces of `B` (a box in a tray corner) | the union, each entry carrying its own face and normal | per entry |

The normal rule is contact-geometry §3's cone rule applied: at an edge or vertex of `A` the admissible
normals form a cone, and the touching certificate proves the `B` face plane supports `A` there, so the
face normal is the one direction the solver may use and is published as a unique normal. Two edges
crossing have one normal up to sign. Vertex-on-vertex, vertex-on-edge and parallel edge-on-edge contacts
leave the manifold absent with `ContactAmbiguousFeature`. Every published point carries its two original
`Face`s (or the edge's faces through `ContactFeature`), a point ball from the exact-to-float conversion,
a normal ball from normalizing the exact direction (zero when the unit normal is a float, so
`NormalAngle` is zero there), and a `Separation` interval containing zero.

The kernel (`internal/pair/planar_manifold.go`) builds the manifold from the zero-distance feature pairs
§9.1 records, which cover the whole contact set. With `X` a convex body and `Y` the other, each pair must
be covered by one accepted piece holding one of its faces, tried in this order; any uncovered pair
withholds the manifold with `ContactAmbiguousFeature`:

- a face of `X` coplanar with and opposed to a face of `Y`: their clip (§9.4), or, when the clip has zero
  area, the ends and isolated points of the degenerate intersection, published with the face normal;
- a face of `Y` whose plane `X` lies wholly in front of and touches along one edge or at one vertex: that
  edge's clipped pieces or that vertex, each piece reaching the face's interior; a piece that only meets
  the face's rim is a vertex-on-edge or parallel edge-on-edge contact and covers nothing;
- the same with the roles swapped, only when `Y` is convex too: a face of `A` on an edge of a
  non-convex `B` is not admitted and withholds the manifold;
- a crease edge of `X` crossing a crease edge of `Y` at an interior point.

Duplicate points are merged only under exact equality of point and features. The normal is normalized
from the exact direction after scaling it by its largest component, so the two query orders publish
exactly opposed normals.

Shallow penetration, which a bracket's right sample may show: for convex `A` against convex `B`, the six
directed translations of the box path generalize to the minimum-translation axis among `A`'s and `B`'s
face normals and the edge cross products; a unique strictly smallest positive translation whose selected
faces cross publishes the patch at depth, exactly as contact-geometry §4's box penetration path does.
The selected faces are `A`'s face with outward normal along the translation and `B`'s face against it;
`B`'s face moves by the translation onto `A`'s plane and is clipped there (§9.4), each corner pairing its
point on `A`'s face with its translate on `B`'s face, and `Separation` is minus the translation length.
A tied minimum, an edge-cross axis, faces that do not cross (one body holding the other along that
axis), or a non-convex body keeps `Overlapping` and withholds the manifold.

### 9.4 The planar patch by exact rational clipping

The patch of a coplanar face pair is computed inside decad, in exact rational arithmetic: the clip
itself in `internal/pair/planar_patch.go` over the two exact loops, the face map and witness publication
in the root `contact_faceted_manifold.go` and `contact_faceted_patch.go`, the same split §9.1 makes.
This is a decad-side 2D answer under CLAUDE.md's "Ask `sketch` for 2D answers by default" rule,
which admits one where it clearly wins on performance or correctness and asks the owning design to
state the reason. Both reasons apply here. Performance: a manifold is asked at every
bracket sample, track end and replay check of every touching pair, and building, arranging and reading
back a private scene per request costs far more than clipping two polygons. Correctness: both loops are
polygons over exact rational vertices, so the clip samples no cut parameter, curve crossing or region
membership and the only rounding is the final witness conversion; the shipped rectangle clips already
work this way.

**Inputs.** The face of convex `A` and the face of `B` that §9.1's touch certificate put on one plane with
opposed normals: every vertex of both loops is an exact rational at the query pose (§9), `B`'s face has
an exact outward normal `n` read off its vertices, and the two plane equations agree exactly (the touch
certificate proves every `A` vertex of the contact set has zero signed height). The shared plane frame
is the coordinate plane obtained by dropping the axis `k` with the largest `|n_k|`: the map from the
plane to `(x_i, x_j)` is an exact rational bijection whose inverse `x_k = (n·q − n_i·x_i − n_j·x_j) / n_k`
(`q` any vertex of the `B` face) lifts a clipped vertex back to 3D without rounding. The dropped axis
is chosen, not fixed to `z`, so a vertical wall pair clips in a nondegenerate frame. Both projected
loops are oriented counterclockwise by the sign of their exact double area before clipping.

**Algorithm.** Sutherland–Hodgman: `B`'s face loop is the subject, and each edge of `A`'s face in turn
is a closed half-plane that the subject is clipped against, with every crossing vertex an exact rational
intersection of the two edge lines. Sutherland–Hodgman is chosen over half-plane intersection because
it needs only the CLIP polygon convex, and that is the one side §9.2's certificate already proves (every
face of a convex body is a convex polygon); `B`'s face may be any simple polygon. The output is passed
through consecutive-duplicate removal and the exact shoelace double area; a result with fewer than three
vertices or zero double area is not a face patch and falls to §9.3's edge and vertex rows. A hole loop
of the `B` face is clipped the same way: a hole clip with positive double area means material is
missing inside the patch, and the manifold is withheld with `ContactAmbiguousFeature`. A non-convex `B`
face may give a patch of several components joined in the output by zero-width edges; the published
points are the union of the components' vertices. The clip takes a poll function, as
`integrateMomentRecordWithPoll` does, that the root adapter builds from the shared `workBudget`; every
rational operation is charged through it, and the loop polls `ctx` every `workPollInterval` operations.

**Published points.** Each output vertex that is a corner of the patch (a vertex the polygon passes
straight through is not) is lifted to 3D exactly, then converted once to a float witness with an
outward ball through `orientedBoxPoint`, exactly as the shipped clips do. A witness whose ball exceeds
`PointResolution` withholds the manifold with `ContactPointTooCoarse`. The normal is `B`'s exact face
normal oriented `A` toward `B`, normalized with the ball §9.3 describes; an angle above
`NormalResolution` withholds the manifold with `ContactNoNormalProof`, and an axis-aligned face has a
zero ball and a zero `NormalAngle`. `Separation` is an exact zero interval.

**The shipped clips as special cases.** `contact_clipped_patch.go`'s `clipHorizontalPolygon` is this
algorithm with the clip polygon an axis-aligned rectangle (four axis-parallel half-planes) in the world
`XY` plane, and `contact_oriented_patch.go` is the case of two rectangles with parallel edges, clipped
in `A`'s rational dual basis. Both stay as they are and keep their dispatch for source boxes; PR 11's
parity test runs their fixtures through the general clip and requires point-for-point identical output.

**Positive-displacement bodies.** A body whose held boundary carries `δ > 0` never reaches this path in
Phase 2 (§9's admission). In Phase 3 it reaches it only under §10.4's `ContactBand`: the clip runs on
the held vertices, which are exact rationals at the pose, and the band is charged afterwards exactly as
§10.4 states — every point ball widened by `δ` and `Separation` carrying the band instead of an exact
zero. The clip itself is unchanged; only the published bounds differ.

**Tests** (`contact_faceted_manifold_test.go`, PR 11), every one asserting computed coordinates:

- A hexagonal prism face overhanging one corner of a rotated floor face publishes seven points, each
  compared against its hand-computed rational coordinate within the ball, with the patch's double area
  equal to the hand-computed rational.
- Parity: the rotated-box-on-floor fixture that publishes eight clipped points today produces the same
  eight points, bit for bit, through the general clip.
- A plank spanning the notch of an L-shaped zero-bound Boolean floor publishes the union of both
  components' vertices; a box covering a through-hole of a plate is withheld with
  `ContactAmbiguousFeature`, and a box beside the hole publishes its four corners.
- Two vertical wall faces touching (normal along `x`) publish the clipped patch; fixing the dropped axis
  to `z` is shown to fail (zero area).
- A `B` face recorded with reversed loop order publishes the same patch; skipping the orientation step
  is shown to fail (empty output).
- A crossing vertex at the non-dyadic coordinate `1/3 mm` publishes at a `PointResolution` above its
  conversion ball and withholds with `ContactPointTooCoarse` below it.
- A hexagon vertex on a floor corner yields zero clipped area and publishes §9.3's single vertex point.

### 9.5 Reversal, order and refusal

Reversing `A` and `B` swaps point and face fields and negates normals. Points sort by `A` face order,
then `B` face order, then exact coordinate (contact-geometry §5). A requested `PointResolution` below a
point ball keeps the relation and withholds the manifold with `ContactPointTooCoarse`. Every refusal is
typed; an inconsistent face map is an evaluator error.

## 10. General rotating sweep and band tracks

### 10.1 Rotating drift over exact planar bodies

`contact_sweep_rotation.go`'s `rotationalPairSweep` already carries the general machinery for a rigid
drift: `prepareRotationalSweepPath` builds the ideal path and travel bound, `refine` is contact-sweep
§5's left-first dyadic search, `intervalClear` is §4.3's `g_l + g_r > T` certificate, and `roundedAt`
bounds the float-to-ideal pose deviation from the body's exact corners. It is limited to source boxes by
its eight-corner inputs. `contact_sweep_faceted.go` generalizes it to any §9 body: the exact vertex set
replaces the corners, the per-sample relation and gap come from §9.1 at the rounded pose, and the
deviation is the largest outward corner distance over all vertices. `SweepPair` dispatches to it when
either path rotates and both bodies are §9 bodies; affine paths of §9 bodies take the same run with a
zero angular term. The `Clear`, `ImpactBracket`, `InitiallyTouching`, `InitiallyOverlapping` and
`Undecided` outcomes follow unchanged.

Each sample transfers the rounded-pose relation to the ideal path by contact-sweep §3. With zero deviation
on both bodies the rounded pose is the ideal pose, and the report transfers whole. Otherwise a gap
transfers with both deviations added to its bound. An overlap transfers through a vertex of one body that
lies inside the other farther than the summed deviations from its boundary (`internal/pair`'s
`PlanarDeepVertex`): the ball of that radius stays inside the moved body, so the moved vertex does too. A
touch cannot survive a nonzero deviation and stays `Undecided`, so a first impact brackets onto an
overlapping right sample. The clear certificate is §4.3's `g_l + g_r > T` or the strict separation of the
coordinate hulls of every vertex's ideal path over the interval. An initial touch continues by §10.2 or
§10.3; two source boxes whose box proofs (contact-sweep §5.1) leave a touch `Undecided` rerun as §9
bodies on this path.

The report keeps a replay proof that never reruns §9.1's relation, because replay has no context to
poll and a relation costs work quadratic in the triangle counts. It holds both prepared paths with their
vertex snapshots, and per certified clear interval its end samples' lower gaps and its vertex-hull gap;
a departure adds its §10.2 vertex heights and rates, and a track its §10.3 plane. A replayed fraction
costs one pass over both vertex sets: the deviation of every staged vertex from its ideal position, which
must fit `PointResolution`, and then one check. In a clear interval the larger of the hull gap and each
end's lower gap less the §4.3 travel to the fraction must exceed the summed deviation; inside a departure
its lower-gap function must; the initial touch replays only at zero deviation; a track's rounded vertex
heights must stay above minus its depth widened by that deviation. Beyond an impact bracket's left edge
or a track's end, replay refuses.

### 10.2 Departure from touch under rotation

Contact-sweep §5.1 asks a departure proof for a lower-gap function `L(u) >= c·u − K·u²` with proven
`c > 0`, `K >= 0`. For a §9 body `A` touching a planar face of `B` with outward normal `n`, under drifts
`(v_A, ω_A, c_A)` and `(v_B, ω_B, c_B)`, each vertex `p` of `A` has signed height `h_p(u)` above the
moving face plane with

```text
h_p(u)   = n(u) · (p(u) − q(u))                        q: a point of B's face, n: its outward normal
h_p'(0)  = n · ((v_A + ω_A×(p − c_A)) − (v_B + ω_B×(q − c_B)))
|h_p''|  <= |ω_A|²·ρ_A + |ω_B|²·ρ_B                                      from p'' and q''
          + 2·|ω_B|·(|v_A − v_B| + |ω_A|·ρ_A + |ω_B|·ρ_B)                 from 2·n'·(p' − q')
          + |ω_B|²·D                                                      from n''·(p − q)
```

with `ρ` each body's largest vertex distance from its pivot and `D` an outward bound on `|p − q|` over
the horizon: `ρ_A + ρ_B + |c_A − c_B| + |v_A − v_B|·h`. Here `q` is the foot of `B`'s pivot on the
plane: it moves rigidly with `B`, lies within `ρ_B` of the pivot because the plane holds a vertex of `B`,
and `h_p(u) = n(u)·(p(u) − q(u))` for every `p`. The `h_p'(0)` form, `n·(v_A − v_B + ω_A×(p − c_A) −
ω_B×(p − c_B))`, is exact for every vertex. For a stationary or translating `B` (every floor, tray and
fixed body) only the first line remains, `|ω_A|²·ρ_A`. Take `c` as the exact minimum of `h_p'(0)` over
the vertices of the contact set and `K` as half the bound above; a non-contact vertex contributes its
positive `h_p(0)` and the same derivative bound. This is the proof `tangentAxisSpinDepartureFraction`
already runs for a box spinning about `Y` with `K = ω_y²·(|Δx| + |Δz|)`, stated for any vertex set and
both bodies moving.

The plane is a SUPPORT PLANE: a face plane of `B` with every vertex of `B` on or behind it and every
vertex of `A` on or in front of it, the contact set being `A`'s vertices on it. Each body lies in its
vertices' hull, so the least vertex height bounds the pair's separation below, whatever the shapes. The
run tries `B` then `A` as the plane's owner, each over its triangles in order, and takes the first plane
whose every contact rate is positive. The horizon is the largest fraction on the sweep's dyadic grid
(`TimeResolution`, capped at 52 levels so it stays a float) at which `c − K·h > 0` and every non-contact
bound `h_p(0) + h_p'(0)·h − K·h²` is positive; both are monotone in `h`, so a binary search over the
grid finds it. A touch no support plane covers — two crossing edges, or a box in a tray corner, whose
floor plane does not hold the walls — stays `Undecided`. After the horizon, §5's search continues on the
remainder as today.

### 10.3 Band tracks: resting and rolling at constant `ω`

A body resting on an edge while rotating, or a cylinder rolling, is a persistent contact whose contact
points do not stay exactly on the support plane under a constant-`ω` drift: their height is second
order in `u`. `SweepPersistentTouch` cannot certify it, because touch is not exact at interior times.
The new outcome states what IS exact:

```go
// SweepPersistentBand: the pair stays within a certified band of one normal.
// At every instant of the track the signed separation along Normal() of every
// contact-set point lies in [−Depth, Depth], interiors overlap by at most
// Depth along that normal, every other vertex of the touching body stays
// strictly in front of the support plane, and the source features are
// stable. Depth is the track's Band(), a Length Measurement. The track may
// end before the duration.
```

`SweepContactTrack` gains `Band() *Measurement` (nil for an exact touch track). The certificate reads a
§10.2 support plane whose owner `B` does not rotate, so its normal is constant. With `r` the largest
`|h_p'(0)|` over the contact set (the solver leaves those speeds within `VelocityResidual` of zero, so
`r` is small) and `K` as in §10.2, every contact-set vertex satisfies `|h_p(u)| <= r·u + K·u²` for
`0 <= u <= h`, and `Depth` is that bound at `h`, its exact value enclosed by the published Value and
Bound. Every non-contact vertex keeps `h_p(u) > 0` by §10.2's bound from its positive `h_p(0)`, and
`A` in front of the plane with `B` behind it bounds the overlap. Every contact vertex's foot must stay
inside `B`'s face: the vertex's ideal path box over `[0, h]`, less `B`'s translation and grown by `Depth`,
is projected along the axis of the normal's largest component, and must meet no bounding edge of the face
with a corner inside one of its triangles. The face's triangles must belong to one `Face`. The track ends
at the largest grid fraction where both checks hold. A track with zero depth that reaches the duration is
an exact `SweepPersistentTouch`. `ManifoldAt(fraction)` stages each contact vertex through the rounded
pose and publishes it with its exact foot on `B`'s rounded plane; both balls carry the pose deviations
(`B` only translates, so its plane keeps its normal), `Separation` is `[−Depth, Depth]`, and the normal
is the face normal.

`dynamics` consumes a band track as a persistent contact when `Depth <= PenetrationResidual`: the pair
stays in the contact set, its points enter the next island at the track end or the next event, and the
next correction (§6.6) removes the accumulated depth within the allowance widened by `Depth`. A track
whose `Depth` exceeds the residual ends the slice at the time the bound reaches the residual (an exact
rational root of the quadratic, bracketed on the dyadic grid as a `ContactTransitionBracket`), so a slow
tip proceeds through several short band tracks and corrections, each certified. Replay inside a band
slice checks the rounded pose's exact vertex heights against `[−Depth − deviation, …]`.

### 10.4 Positive-displacement bodies (Phase 3)

A body whose held boundary carries a positive two-sided displacement `δ` (a tessellated curved face, a
chamfer with non-dyadic feet, a placed loft) cannot certify an exact touch: contact-geometry §2 makes
that relation `Undecided`. §10.3's band is the honest replacement, with `δ` charged:

```go
// ContactBand (appended after ContactOverlapping): interiors are disjoint
// except possibly within a band of width Gap.Bound around the published
// Gap.Value, which is zero; the manifold's Separation intervals carry the
// same band.
```

**Admission.** Two held meshes are admitted, each exact (§9's dyadic vertices) and standing for its true
boundary within `δ`: a positive-bound faceted Boolean that the exact-source path of §9 does not cover,
read off its payload with `δ` its mesh bound, and a cap-loop chamfer whose every face is planar, read off
its own tessellation with `δ` that mesh's `Bound` (zero when nothing rounds, which admits it to §9 as an
exact body). A held mesh moves through the exact float query pose, whose linear part is orthonormal only
to rounding, so `δ` at a pose is the body's figure times `s = max(1, (1 + g)/2)`, `g` the largest
absolute row sum of the basis's exact Gram matrix, an upper bound on the pose's stretch. Prisms with a
positive section or level displacement and lofts are not admitted yet.

**Relation.** With `δ` the two bodies' displacements summed at the query poses, every true boundary
point lies within `δ` of the held pair's. `ContactPair` reads §9.1's exact held relation:

| Held relation | Published |
|---|---|
| `Separated`, gap lower end above `δ` | `Separated`, the held gap with `δ` added to its bound |
| `Touching`, or `Separated` with gap upper end at most `δ` | `ContactBand`, `Gap = [−2δ, 2δ]` |
| `Overlapping` with a vertex deeper than `δ` inside the other body (`pair.PlanarDeepVertex`) | `Overlapping`, no manifold |
| `Overlapping` of two convex bodies whose §9.3 shallow patch has depth at most `δ` | `ContactBand`, `Gap = [−2δ, 2δ]` |
| anything else | `Undecided` |

A band from a held touch or shallow patch carries that held manifold charged afterwards (§9.4): each
witness ball grows by its own body's `δ`, and every `Separation` is the band. The normal is published only
when, at every point, it is the exact face normal of a body with zero `δ` read at a face that holds the
point; a held face of a displaced body only approximates its true face's direction, so otherwise the
manifold is withheld with `ContactNoNormalProof`, as it is for a band from a held gap.

**Sweep.** §10.1's samples transfer as there, over the held vertices: a band widens by both pose
deviations and drops its manifold, as a touch would; an overlap needs a vertex deeper than both
deviations plus both displacements at the rounded poses; a vertex-hull gap counts only past `δ`. The
ideal pose is a rigid motion, so it moves each true body within its `δ` of its held one. A first impact
brackets onto a band sample like onto a touch. A pair that starts in its band is an initial contact: no
departure is published, since the true pair may stay in the band however fast the held pair separates,
and `ContinueCertifiedTouch` takes §10.3's track over the held touch with three charges. The support
plane's owner must carry zero `δ`, since the track publishes its face normal; each contact foot's box grows
by the touching body's `δ` before the face-containment check; and `SweepPersistentBand`'s `Depth` is the
held depth widened by `2δ`, so a band pair never publishes an exact touch track. `ManifoldAt` grows both
balls of each point by that `δ`. Replay of a clear span needs the proven lower gap to exceed the summed
pose deviation plus `(s + 1)·δ` for each body, `s` the rounded pose's stretch: a true point lies within `δ`
of its held body, and the rounded and ideal linear parts move that offset by at most `s` and one. A track
replay keeps reading the held depth against the held vertices.

**Dynamics.** `dynamics` treats `ContactBand` as a touching relation whose penetration bound is
`Gap.Bound`, admitted when `Gap.Bound + Depth <= PenetrationResidual`. A positive-`δ` body therefore needs a
`PenetrationResidual` above `2δ`, which the caller sets; a tighter residual leaves the pair `Undecided`,
never silently touching.

Curved source families with their own exact occupied sets (sphere, axial cylinder) keep their exact
paths; curved families without one (a cylinder rolling on its side, cone, torus) enter contact-geometry
§7 stage C2 through the clearance kernel's face-pair table for the relation, with manifolds from the
certified ruling feet of `docs/clearance-design.md` §6 (contact-geometry §4.5) and normals from
`Face.NormalAt`; a rolling
cylinder's band track takes §10.3 with the ruling's two endpoints as the contact set and the cylinder's
`Face.NormalAt` ball charged into the band. Their delivery is §13's last three PRs.

## 11. Kinetograph interface and gallery

### 11.1 What decad needs from kinetograph

kinetograph's `Node` kinds are `Fixed`, which takes one constant transform, and `Revolute` and
`Prismatic`, each driven by a scalar keyframe `Channel`; no kind takes a transform that varies with time,
and its design (D1) interpolates no orientation. A certified trace must not be re-expressed as keyframes:
a keyframed channel blends between two poses, and the blend is a pose no certificate backs. The interface
this program needs upstream, which kinetograph records as its design decision D12 (a driven node shows a
caller-supplied transform at each time and is never interpolated, held or cached across times):

```go
// kinetograph root package: a node whose local transform is supplied per time.
type TransformTrack interface {
    // At returns the node's local transform at t. kinetograph calls it any
    // number of times for one t, concurrently from several goroutines, and
    // uses each result as Local(t); it never blends two results. At must
    // return the same transform for the same t and be safe for concurrent
    // calls.
    At(t time.Duration) (r3.Transform, error)
}

// Driven adds a child whose Local(t) is track.At(t). Driven(nil) returns
// ErrNilTrack. An error from At fails every evaluation of that frame with
// the track's error wrapped.
func (n *Node) Driven(track TransformTrack) (*Node, error)
```

The contract the gallery relies on:

- `At` returns the LOCAL transform. `Node.World` composes it as `Local(t).Then(parent.World(t))`, so a
  driven node may sit under any parent and any node kind may sit under it. The gallery puts each body's
  driven node directly under the root, so the local transform is the body's world pose.
- `At` is called once per part, light and camera on the node for every evaluation of a frame, again in
  the gallery's probe pass, and from `Sequence`'s concurrent render workers. kinetograph keeps no cache
  per `(node, t)`; a track with a costly `At` caches on its own side.
- Each result is validated as `Fixed` validates its argument: an invalid transform is
  `ErrInvalidTransform` and a reflection is `ErrReflection`, both wrapped with `t`.
- An error from `At` fails the frame on every path with `errors.Is` and `errors.As` intact:
  `Node.Local`/`World` return it wrapped; `Scene.At`/`AtCached` wrap it with the part, camera or light
  name and the time, or return `ctx.Err()` when `ctx` is done; `Clip.Frame`/`FrameCached` return it
  unchanged; `render.Renderer.Frame` and `Sequence` return a `*render.FrameError` for that frame index,
  and `Sequence` writes no file for that frame and none after it. kinetograph never substitutes the last
  good pose for a failed one; the gallery depends on that failure to stop the render when the timeline
  stops.

That is the whole dependency: one interface, one node constructor, one sentinel and the error path.
Everything else kinetograph does (parts on nodes, tessellation at `VerifyNone` for rendering only,
per-frame vertex transform by the renderer, integer-nanosecond frame times passed to `At` unchanged)
already fits. kinetograph imports nothing from `dynamics`; `TransformTrack` lives in its root package and
the gallery adapts a `dynamics.Timeline` to it.

### 11.2 The gallery bridge

`_gallery/dynamics_track.go` adapts a `dynamics.Timeline` to one `TransformTrack` per body:

```go
type timelineTrack struct {
    timeline *dynamics.Timeline
    body     *decad.Body
    base     r3.Transform // the body's committed placement; the part frame is the body as modeled
}

func (t timelineTrack) At(d time.Duration) (r3.Transform, error)
```

`At` converts the frame time to `units.Seconds(float64(d) / 1e9)` — this rounds the TIME LABEL by at
most one ulp of a second; the pose returned is the certified pose at the rounded time, which is what the
frame shows, so no geometric claim moves — then calls `Timeline.Sample` and returns the sampled
`BodyState.Pose` composed onto the body's placement. A sample beyond `Timeline.End()` (§7.2), or one the
replay refuses, returns the error; kinetograph fails the frame, and the gallery command fails with the
`StepDiagnostic` of the stopped step printed. The gallery never freezes the last certified pose and never
extrapolates. kinetograph's half-open frame times keep every frame strictly before the clip length, so a
timeline advanced to the clip length is never sampled at `End()` itself. `timelineTrack` holds no
mutable state and `Timeline.Sample` is safe for concurrent calls (§7.2), so `At` meets §11.1's contract
under `Sequence`'s workers.

`_gallery/dynamics_clip.go` holds the scene builders (`stackAndDropScene`, `tumbleScene`,
`partsBinScene`), each returning the document, the `WorldConfig`, the start `State`, the per-step
`StepInput`, `dt` and the clip length. `main.go` gains `go run . dynamics -scene <name> [-out -fps -width
-height -workers -smoke]`, which advances the timeline to the clip length, builds one kinetograph scene
with a `Driven` node per body and the still camera and lights of `scene.go`, and renders through
`render.New` as `clip.go` does. The gallery module stays the only module that imports kinetograph.

### 11.3 Scene tests

`_gallery/dynamics_clip_test.go` runs each exit scene's timeline (without rendering) and asserts the
§2 exit criteria on the trace: certified end time, final poses, event kinds and times, conservation.
`dynamics/scene_test.go` asserts the same scene from inside the module so CI runs it without the
gallery's toolchain. Both use the real producers end to end.

## 12. Work budgets, cancellation and `Undecided`

Validation runs before `ctx` is read; afterwards every loop that calls into decad — the box pass,
each `SweepPair`, each `ContactPair`, each island sweep, each correction sweep — checks `ctx` before the
call, and cancellation returns `ctx.Err()` with a nil report, as the two-body step does. Within decad,
the new kernels (§9.1, §9.4, §10.1) charge a `workBudget` from `newWorkBudget(ctx)` and poll it per
`docs/interference-design.md` §7.2.

Three budgets bound a step's work without changing any certified outcome: `MaxPairSweeps` (§3.3) over
`SweptBox` and `SweepPair` calls, `MaxEvents` over published events, `MaxIterations` per island sweep.
`MaxPairSweeps` counts every call the step makes — the broad phase, the pair sweeps, the box-exclusion
checks of §7.1 and the correction sweeps of §6.6 — and not a call §5.3 reuses; `Trace.Sample`'s own
checks are not charged. Each `SweepPair` call carries `MaxPoseEvaluations` as today. Exhausting any
budget with time remaining is `Undecided`.

```go
type StepReason int

const (
    StepNoReason           StepReason = iota
    StepPairUndecided                 // a candidate sweep is Undecided before the next event
    StepManifoldMissing               // an event pair has no manifold within StepConfig.Contact
    StepIslandDegenerate              // a closing constraint with no dynamic body, or K <= 0, or a non-finite proposal
    StepIslandResidual                // a §6.3 gate exceeds its limit at MaxIterations
    StepCorrectionFailed              // §6.6 correction exceeds its allowance or loses a relation
    StepTrackUnproved                 // a contact-set pair has neither a persistent nor a band track
    StepKickUnbounded                 // the force kick cannot be bounded within the velocity residuals
    StepConservationFailed            // an island or step conservation gate fails
    StepEventBudget                   // MaxEvents reached with time remaining
    StepPairBudget                    // MaxPairSweeps reached
    StepTravelUnbounded               // SweptBox returned ErrUnsupported for a body
    StepFixedPairRelation             // a non-excluded Fixed/Fixed pair is Overlapping or Undecided
    StepUnsupported                   // the current phase has no solver for this event family
)

type StepDiagnostic struct {
    Code     StepReason
    Pair     BodyPair        // the pair, when one is responsible
    Bodies   []*decad.Body   // the island's bodies, when an island is responsible
    From, To units.Value     // the time interval, from the step start
    Limit    units.Value     // the configured limit a residual exceeded, when one did
    Reason   string          // human-readable message; callers branch on Code
}
```

A report with `Undecided` carries every diagnostic that applied at the stopping time, in canonical pair
then island order, and `Next == nil`; its `Trace` holds the certified prefix for inspection, its `Events`
and `Islands` those the prefix published. The prefix ends at the last certified time: the start of the
step, an event's post state, or the pre-event state when the event's solve refused, and its `duration` is
that time, so `Trace.Sample` replays exactly the certified part.

The scheduled step of a world of four or more bodies fills every field: `From` and `To` bound the slice a
pair diagnostic covers (an undecided sweep's unresolved interval, mapped onto the step clock), or name the
event time of a refusal raised there; `StepEventBudget` runs from that event to the end of the step.
`Limit` holds the refused gate's limit for `StepIslandResidual` (in the gate's own kind), the correction
allowance for `StepCorrectionFailed`, `PenetrationResidual` for `StepTrackUnproved`, `MaxEvents` as a
dimensionless scalar for `StepEventBudget`, and `MaxPairSweeps` likewise for `StepPairBudget`, whose
interval runs from the last certified time to the end of the step. The two- and three-body steps leave `Code` `StepNoReason` and
set only `Pair` and `Reason`.

## 13. Delivery order

Each PR is one branch, one review, one merge, and carries the test that proves it. "Root tests" means
`.github/test-shards.txt` must list every new root-package test name; `dynamics` tests are not sharded.
Dependencies are listed; PRs with no edge between them may land in either order. Phase 1 is PRs 1–9,
Phase 2 is PRs 10–15, Phase 3 is PRs 16–21. PR 9 waits on the upstream node of §11.1; PRs 1–8 do not.
A PR that changes what ships — a body count `NewWorld` admits, a shape pair, a response, a refusal —
also updates `docs/collision-v1-support.md`, the user-facing inventory, in the same PR; the "Files"
lines below do not repeat it.

### PR 1 (Phase 1) — N-body `World` and `State`

- Delivers §3.1 and §3.2: the pair table, per-pair material, `Bodies()`/`Pairs()`, `NewState` over a
  slice; the shipped two- and three-body steps read through the table.
- Files: `dynamics/world.go`, new `dynamics/pairs.go`, `dynamics/three_body.go`.
- Test: `dynamics/world_test.go` constructs a five-body world, asserts canonical pair order and every
  exclusion and override rule; every existing fixture is unchanged.
- Depends on: nothing.

### PR 2 (Phase 1) — `Document.SweptBox`

- Delivers §4.2: `SweptBox`, `StrictlyDisjoint`, `Box()`; the cylinder and bounded-faceted sweeps consume it.
- Files: new `swept_box.go`, `contact_cylinder_sweep.go`, `contact_faceted_sweep.go`.
- Test (root): `swept_box_test.go`: a rotating box's swept box contains its exact corners at fractions
  `0`, `1/3` and `1`; dropping the travel term lets a corner escape (shown to fail); meeting boxes are
  not disjoint.
- Depends on: nothing.

### PR 3 (Phase 1) — N-body drift step with the broad phase

- Delivers §4.3 and §5 without islands: every candidate pair must be `Clear` or `DepartedClear`; an
  event is `Undecided` with `StepUnsupported` until PR 4.
- Files: new `dynamics/broadphase.go`, `dynamics/schedule.go`; `dynamics/step.go`.
- Test: `dynamics/schedule_test.go`: six separated boxes, two rotating, drift `0.1 s`; only the three
  near pairs are swept (`PoseEvaluations` is zero on box-excluded pairs); `Trace.Sample` at interior
  times matches the exact drift.
- Depends on: PRs 1, 2.

### PR 4 (Phase 1) — islands and the frictionless certified solver

- Delivers §6.1–§6.3 without the cone, stick and slip rows, and §6.6 per-island correction.
- Files: new `dynamics/island.go`, `dynamics/island_solve.go`, `dynamics/island_certify.go`;
  `internal/proof/interval.go` gains the three-component interval dot and cross products of §6.3, with
  their tests in `internal/proof/interval_test.go`.
- Test: `dynamics/island_test.go`: the `3-2-1` pyramid rests under gravity through one island of nine
  pairs and 36 manifold-point constraints with every normal impulse computed and the bridging patches'
  impulses summing to the supported weight; the two-sphere `37.5 kg·mm/s` fixture runs through the general
  path as a four-body world, the touching pair plus two spheres the broad phase excludes, so the public
  `World.Step` reaches the island solver with the real producers and asserts the two-body fixture's numbers.
- Depends on: PR 3.
- Shipped. The pyramid uses `dt = 1/256 s`, so the kick is exact, and `ImpactSpeed = 64 mm/s`, so its
  restitution `0.3` targets zero and the stack rests.

### PR 5 (Phase 1) — Coulomb friction and parity

- Delivers the cone, stick and slip rows of §6.3, the parity run of §6.5, and the deletion of the
  closed-form responders §6.5 lists.
- Files: `dynamics/island_solve.go`, `dynamics/island_certify.go`; the deleted files.
- Test: every existing `dynamics` response test passes through the general solver with its original
  assertions, `friction_patch_test.go`'s slide and the stack fixture included.
- Depends on: PR 4.
- The cone, stick and slip rows ship for worlds of four or more bodies, with
  `dynamics/island_friction_test.go`: the four-corner slide of `friction_step_test.go` in both body
  orders, a box slipping across a dynamic box the floor holds by sticking, and the three rows' tamper
  fixtures. So do §6.2's direct start and lever-bounded spin snap, §6.1's silent zero-speed island,
  §5's certificate-replayed slice poses and `MaxEvents` rule, and §6.3's `AngularUpper`, with
  `dynamics/island_direct.go` and `dynamics/island_direct_test.go`: a two-box stack in eight sweeps, a
  frictional face impact that friction stops exactly, and a sphere rolling in a corner. §6.6's
  separating push ships with `dynamics/island_push_test.go`: a sphere bouncing off a tilted face, its
  allowance, and a resting sphere the push leaves alone; its sub-ulp-gap push and the anchored correction
  with `dynamics/island_sphere_landing_test.go`: the stack-and-drop sphere column landing on itself, a
  glancing landing, and a gap no float can prove. Routing the two- and three-body worlds through
  the general step, the assertion rewrites of §6.5 and the deletions remain.

### PR 6 (Phase 1) — multi-event `Trace`, `Timeline`, typed diagnostics

- Delivers §3.4, §7.1, §7.2 and §12.
- Files: `dynamics/step.go`, new `dynamics/trace.go`, `dynamics/timeline.go`.
- Test: `dynamics/timeline_test.go`: a sphere bouncing on a floor with `e = 0.5` over three steps; the
  bounce times match the closed-form sequence; `Timeline.Sample` across a step boundary; a forced
  `Undecided` stops the timeline with `StepEventBudget` and `Advance` then returns `ErrTimelineStopped`.
- Depends on: PR 4.
- Shipped, with §5's interior events (impacts, transitions, grazes, zero-time repeats), kinematic island
  participants, §6.2's common velocity, §6.6's group correction and §7.1's per-sample check. Its tests add
  `dynamics/schedule_event_test.go`: five bounces between a floor and a ceiling in one step ending in rest
  under `ImpactSpeed`, a box stack bouncing as one, a kinematic platform's impact and work, a graze, an
  edge transition, the event budget's certified prefix, and the box-exclusion check at a slice end and at
  a sample.

### PR 7 (Phase 1) — rotated and displaced prism mass

- Delivers §8.1 and §8.2.
- Files: `mass_properties.go`, new `mass_properties_rotated.go`.
- Test (root): `mass_properties_rotated_test.go`: a box rotated `30°` about `(1,1,1)` encloses
  `R I Rᵀ` in every component; the orthonormality-defect leg is shown to fail.
- Depends on: nothing.

### PR 8 (Phase 1) — budgets, certificate reuse, warm start

- Delivers `MaxPairSweeps` (§3.3), reuse (§5.3) and the warm-start `contactCache` keyed by pair and
  `ContactFeature` (rigid-dynamics "Response"), with no change to any certified outcome.
- Files: `dynamics/world.go`, `dynamics/schedule.go`, new `dynamics/contact_cache.go`.
- Test: `dynamics/schedule_test.go`: the pyramid's second step performs no `SweepPair` on pairs whose
  bodies did not move and converges in one sweep from the cache; outcomes are bit-identical with the
  cache disabled.
- Depends on: PR 6.
- Shipped, with the contact set carried on `State` (§3.2, §5 step 2), so a later step continues a
  resting pair with no zero-time stop or solve, and the solver's restart in `dynamics/island_solve.go`. Its tests in
  `dynamics/schedule_test.go`: `TestScheduledStepReusesCertificates` (the pyramid beside two drifting
  boxes: the second step reuses every pyramid report and box, sweeps only the drifting pair, restarts
  the island in one sweep where its cold solve settles, and matches the cache-free step bit for bit),
  `TestScheduledStepRepublishesUnsettledIsland` (the same island capped at 64 sweeps),
  `TestScheduledStepPairBudget`, and the meeting boxes' second step, which continues the carried pair
  with one `SweepPair` call fewer than a state rebuilt without the contact set.

### PR 9 (Phase 1) — gallery bridge and `stack-and-drop`

- Delivers §11 and the Phase 1 exit scene.
- Files: `_gallery/dynamics_track.go`, `_gallery/dynamics_clip.go`, `_gallery/main.go`,
  `_gallery/go.mod` (kinetograph bump), new `dynamics/scene_test.go`.
- Test: `_gallery/dynamics_clip_test.go` and `dynamics/scene_test.go` assert the Phase 1 exit criteria.
- Depends on: PRs 6, 7, 8 and the upstream `Driven` node.

### PR 10 (Phase 2) — exact planar pair relation and convexity

- Delivers §9.1 and §9.2.
- Files: new `internal/pair/planar.go` and `contact_faceted_pair.go`; `contact_pair.go`.
- Test (root): `contact_faceted_pair_test.go`, with the snapshot-level cases in
  `internal/pair/planar_test.go`: a hexagonal prism at a `37°` pose against a tray reads a
  `3 mm` gap enclosed, a vertex touch and a shallow crossing; a small box nested in a hollow Boolean's
  wall is `Overlapping` and one in its cavity is `Separated`; the non-convex Booleans report
  `ContactNonConvex`.
- Depends on: nothing.

### PR 11 (Phase 2) — faceted manifolds and the exact-clipped patch

- Delivers §9.3, §9.4 and the shallow-penetration patch.
- Files: new `internal/pair/planar_patch.go`, `internal/pair/planar_manifold.go`,
  `contact_faceted_manifold.go`, `contact_faceted_patch.go`; `contact_faceted_pair.go`.
- Test (root): `contact_faceted_manifold_test.go`: a rotated box on a face publishes one point at a
  vertex touch, two at an edge touch, and the clipped hexagon's seven extremal vertices at a face
  touch, each at its exact coordinate; §9.4's parity, wall, reversed-loop, hole and
  `ContactPointTooCoarse` fixtures, with the wall and reversed-loop legs shown to fail.
- Depends on: PR 10.

### PR 12 (Phase 2) — general rotating sweep

- Delivers §10.1.
- Files: new `contact_sweep_faceted.go`; `contact_sweep.go`, `contact_sweep_rotation.go`.
- Test (root): `contact_sweep_faceted_test.go`: a wedge tumbling toward a floor brackets its first
  vertex impact at the exact drift time within `TimeResolution`; the deviation leg is shown to fail.
- Depends on: PRs 10, 11.
- Shipped, with the deep-vertex overlap witness in new `internal/pair/planar_depth.go`.

### PR 13 (Phase 2) — generalized departure and band tracks

- Delivers §10.2, `SweepPersistentBand` and `Band()` (§10.3); `dynamics` consumes band tracks.
- Files: `contact_sweep_faceted.go`, `contact_sweep.go`, `dynamics/schedule.go`, `dynamics/island.go`.
- Test (root): `contact_sweep_band_test.go`: a box resting on an edge with `ω = (0,1,0) rad/s`
  publishes a band track whose `Depth` equals `K·h²` for the computed `K`. `dynamics/tip_test.go`: the
  box tips flat over `0.5 s` in short band slices and rests on four points.
- Depends on: PR 12.
- Ships in two parts, split around PR 6's rewrite of `dynamics/schedule.go`, `island.go` and `step.go`.
  The root part has shipped: §10.2 and §10.3 in new `contact_sweep_band.go`, and §10.1's replay proof in
  `contact_sweep_faceted.go` and `contact_sweep_replay.go`, with `contact_sweep_band_test.go`. The
  `dynamics` part, band-track consumption and `dynamics/tip_test.go`, has not shipped.

### PR 14 (Phase 2) — loft, stitched and fallback mass

- Delivers §8.3, §8.4 and §8.5.
- Files: `mass_properties.go`, `mass_properties_faceted.go`, new `mass_properties_mesh.go`.
- Test (root): `mass_properties_mesh_test.go`: the stitched tetrahedron's tensor against the closed
  form; a loft between two exact octagons, one a square with its edge midpoints pushed out (a loft
  pairs equal segment counts and its cap triangulator refuses collinear corners); a cup against its
  closed form. `mass_properties_mesh_internal_test.go`: the ladder narrows a sphere's tensor interval
  at each `k` a revolve mesh reaches, and a revolve the analytic path refuses is read off its mesh.
- Depends on: nothing.

### PR 15 (Phase 2) — `tumble`

- Delivers the Phase 2 exit scene of §2.
- Files: `_gallery/dynamics_clip.go`, `dynamics/scene_test.go`.
- Test: the Phase 2 exit criteria.
- Depends on: PRs 9, 13, 14.

### PR 16 (Phase 3) — third-order section moments and the general revolve

- Delivers §8.6.
- Files: `moments.go`, `moments_circular.go`, `spline_moments.go`, new `mass_properties_revolve.go`.
- Test (root): `mass_properties_revolve_test.go`: a quarter revolve of an off-axis rectangle encloses
  the independently integrated `r³`, `r²z` and `rz²` terms in its mixed components, and an off-axis
  triangle makes the `r²z` products nonzero; a full torus against the closed form; a rotated placement
  encloses `Q I Qᵀ`, with the orthonormality-defect leg shown to fail.
- Depends on: nothing.

### PR 17 (Phase 3) — sweep and cup mass

- Delivers §8.7 and §8.8.
- Files: new `mass_properties_sweep.go`, `mass_properties_cup.go`.
- Test (root): a composite sweep against the sum of its spans; the cup against outer minus cavity.
- Depends on: PRs 7, 16.

### PR 18 (Phase 3) — `ContactBand` for positive-displacement bodies

- Delivers §10.4.
- Files: `contact_faceted_pair.go`, `contact_sweep_faceted.go`, `dynamics/island.go`.
- Test (root): `contact_band_test.go`: a chamfered block with `δ = 1e-9 mm` publishes `ContactBand`
  with `Gap.Bound = 2δ` on the floor, rests in `dynamics` at `PenetrationResidual = 1e-6 mm`, and is
  `Undecided` at `1e-10 mm`.
- Depends on: PR 13.
- Ships in two parts, so the root part does not touch `dynamics/schedule.go` and `island*.go` while PRs 5
  and 8 rewrite them. The root part has shipped: §10.4's admission, relation and manifold charge in
  `contact_faceted_pair.go`, its sweep transfer, band track and replay in `contact_sweep_faceted.go`,
  `contact_sweep_rotation.go`, `contact_sweep_band.go` and `contact_sweep_replay.go`, with
  `contact_band_test.go` and `contact_band_internal_test.go`. No real producer publishes `δ = 1e-9 mm`:
  the chamfered block is a cap-loop chamfer whose `δ`, read through `Tessellate`, is its contour's
  rounding (about `1e-15 mm`), and a box `Union` a chorded disc (`δ` about `4e-4 mm`) places held gaps
  inside and outside the band. The `dynamics` part, band consumption in `dynamics/island.go` with the
  rest and `Undecided` residuals, lands with PR 13's `dynamics` part and picks its two residuals around
  its fixture's real `2δ`.

### PR 19 (Phase 3) — curved analytic manifolds (C2)

- Delivers plane/cylinder ruling and cylinder/cylinder manifolds from clearance's certified feet with
  `Face.NormalAt` balls.
- Files: new `contact_analytic_manifold.go`; `clearance_tiers.go`.
- Test (root): `contact_analytic_manifold_test.go`: a cylinder on its side against a floor publishes
  the ruling's two endpoints with the computed normal ball.
- Depends on: nothing.
- Shipped, at identity query poses. The gate the ruling certificates read is
  `docs/clearance-design.md` §6's carrier displacement, so a full revolve's end-angle term does not refuse
  them.

### PR 20 (Phase 3) — rolling band tracks

- Delivers the last paragraph of §10.4.
- Files: `contact_sweep_faceted.go`, new `contact_sweep_rolling.go`.
- Test: `dynamics/rolling_test.go`: the cylinder rolls `π·20 mm` in one turn at `ω = 2π rad/s` with
  contact-point speed within `VelocityResidual` of zero.
- Depends on: PRs 13, 19.

### PR 21 (Phase 3) — `parts-bin`

- Delivers the Phase 3 exit scene of §2.
- Files: `_gallery/dynamics_clip.go`, `dynamics/scene_test.go`.
- Test: the Phase 3 exit criteria.
- Depends on: PRs 15, 17, 18, 20.

## 14. Test and fixture strategy

Every PR's test asserts computed geometry and dynamics — impulses, velocities, poses, event times, gaps,
tensor components — through the real producer and the real consumer, never an enum alone and never a
hand-written manifold, event or pose (CLAUDE.md "Correctness must be observable";
`docs/contact-sweep-design.md` §8; `docs/rigid-dynamics-design.md` "Verification"). Beyond that:

- **Every bound leg is shown to fail.** For each new certificate (travel bound, pose deviation, band
  depth, orthonormality defect, cone and slip gates, `E·R²` widening) the test file records that each
  term was deleted or zeroed and the fixture went red, or states the argument for why a leg is provably
  redundant. A fixture whose offset, rotation or displacement is zero cannot exercise the term, so each
  fixture is built with the term nonzero.
- **No pinned bound literals.** Bounds are asserted negligible against a slack figure with a comment
  saying why; values are `InDelta` at a stated slack. FMA contraction differs between hosts.
- **Dyadic inputs.** Fixture coordinates, velocities and times are dyadic so exact comparisons (event
  fractions, touch equalities) are platform-independent.
- **Reversal and order.** Every pair kernel test runs both body orders; every schedule test permutes
  world insertion order and asserts identical events up to pair naming.
- **Budgets and cancellation.** Each new loop has a test that exhausts its budget (`Undecided` with the
  named reason, document unchanged) and one that cancels mid-loop (`ctx.Err()`, nil report).
- **Parity.** PR 5 keeps every shipped response test byte-for-byte in its assertions, except for the
  rewrites §6.5 lists; a loosened slack there is a review refusal.
- **Scene tests** run in both modules (§11.3); the gallery test is the one that also renders a smoke
  frame.
