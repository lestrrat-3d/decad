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

Current state: every PR of §13's delivery order has shipped, PRs 1 to 21 with 14a–14f and 20a–20g.
`dynamics.World` holds any number of bodies, the canonical pair table and
per-pair material of §3.1, and the slice-backed `State` of §3.2. Every world, whatever its body count,
takes the scheduled step of §4.3 and §5: one kick, then slices of drift from event to event. Initial
contacts, impact brackets and transition brackets cut a slice at their exact fraction; every body advances
there on its certified path; the touching and impacting pairs, with the contact-set pairs their bodies rest
on, form §6.1's islands, which §6.2 proposes, §6.3's rows certify and §6.6 corrects, publishing
one `IslandReport` per island and one event per pair; the next slice starts from the post-event state. A
graze publishes its event without cutting the slice. A rotating pair's impact advances to its
bracket's right end, and a contact-set pair on a band track continues while its band stays within
`PenetrationResidual` (§10.3); a `ContactBand` is a touch within that residual (§10.4). Kinematic bodies join
islands with their driver's exact velocity field, translating or rotating (§6.1), and a positive-friction
pair takes the Coulomb rows of §6.2 and §6.3. The published state
carries the step's contact set and reuse cache (§3.2), so a later step continues a resting pair with no
solve, reuses every certificate whose inputs repeat (§5.3), and restarts a repeated island at its fixed
point (§6.2); `MaxPairSweeps` bounds the rest. The multi-event
`Trace` of §3.4 and §7.1, the `Timeline` of §7.2 and §12's typed diagnostics ship. `Document.SweptBox`
(§4.2) is public; the cylinder and bounded-faceted clear sweeps certify with it, and the scheduled step's
broad phase reads it. A source cylinder that lands on its end disk rests on contact-sweep §4.6's persistent
disk track. The gallery bridge of §11 films a `Timeline` through kinetograph's driven node, and §2's three
exit scenes run their full length in `_gallery` and in `dynamics/scene_test.go`, the last two there only
under `DECAD_TUMBLE_FULL` and `DECAD_PARTSBIN_FULL` (§13 PRs 15 and 21): the Phase 1 scene,
`stack-and-drop`, its `2 s`; the Phase 2 scene, `tumble`, its `3 s` with every body resting face down;
and the Phase 3 scene, `parts-bin`, its `4 s` with the five dropped bodies resting on a `ContactBand` and
the cylinder rolling without slip.
`docs/collision-v1-support.md` is the inventory of the shape pairs, responses and refusals that ship, and
this document does not restate it. No closed-form responder remains in `dynamics` (§6.5). The exact
arithmetic every certificate below is stated in is one package, `internal/proof` (`Dyadic`, `DyV3`,
`RatInterval` and the float rounding bounds), which the root package and `dynamics` both import.
§3 to §12 describe shipped code, for every world, with the limits each section states. §9's exact
planar path admits prisms over whole `LineSeg` sections, zero-bound Booleans, directly or through a
translation-only placement, closed all-planar stitched solids, and every other solid payload whose §10.4
held mesh has zero `δ`, such as §2's cup, loft and sweep. §10.4's held mesh admits every remaining solid
payload without an exact contact family of its own at its positive `δ`, a body with a curved face at the
request's `HeldChord`. §10.4's rolling band track carries a full source cylinder on an exact planar body
from any start pose, on a plain floor or a tray's floor (§10.6), and the scheduled step rolls such a
cylinder.

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
| How does an arbitrary solid sweep while rotating, rest while rotating, and come to rest flat? | §10 General rotating sweep and band tracks |
| How does a convex body poke through one face of a tray, and rest on or depart from that face? | §9.6 Shallow penetration through a face, §10.6 Face-local support planes |
| How does a pair inside the `SupportBand` continue after an event? | §10.7 Continuation inside the band |
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
| A manifold for every contact of two non-convex bodies | §9 proves a manifold when one body of the pair is convex, and §10.5's guest rule when every contact of two non-convex bodies lies on one face of one of them (§2's cup on the tray's floor). Any other touch or overlap of two non-convex bodies gets a relation and no manifold (`ContactNonConvex`); a non-convex FIXED or kinematic body against a convex dynamic one is fine, since only its faces enter. |
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

**Phase 1 — `stack-and-drop`.** A fixed source-box floor with its top at `z = 0`. Six `20 mm`
source boxes in a `3-2-1` pyramid: three on the floor at `x = 0, 25, 50`, two bridging the gaps on top,
one on the top row. Three radius-`8 mm` source spheres with centers released at `z = 60, 90, 120 mm`
over `x = 120`, offset in `y` so the second lands on the first and the third on them (`y = 10, 14, 6`).
One source cylinder (`Ø20 × 30 mm`, axis along `z`) released axially with its lower disk at `z = 80 mm`
over `(160, 10)`. The floor is `360×2000×10 mm`, spanning `x = −60…300`, `y = −990…1010`: the glancing
sphere impacts leave the spheres rolling along `y` at up to about `410 mm/s` (no rolling resistance is
modeled), up to `766 mm` from their drop line by `2 s`, and the sphere-box contact needs each sphere's
contact inside the floor's top face. Gravity `-9810 mm/s²`, box and floor
restitution `0.3`, sphere restitution `0.6`, so a pair takes the smaller (rigid-dynamics "World and State"):
`0.3` against the floor and the boxes, `0.6` between spheres. The cylinder takes the boxes' material.
Friction `0.4` everywhere, density `0.001 kg/mm³`, `dt = 1/256 s` so every kick `−9810/256 mm/s` is exact,
`ImpactSpeed = 64 mm/s`, above that kick, so a body resting under gravity targets zero speed each step and a
bounce sequence ends, `2 s` of motion (512 steps), rendered at `60 fps`. Exit criterion: `Timeline.End()`
equals `2 s` (no `Undecided`); the six pyramid boxes end within `PenetrationResidual` of their start poses;
every step's conservation readings balance (the linear momentum change equals the gravity and contact
impulses within the readings' bounds and the islands' linear-law limits); the bridging boxes' two half-face
patches each carry a positive share of the support impulse, summing to the top box's weight; the first
sphere's first floor impact occurs at the free-fall time of the step's discrete law (one exact kick per
step, then drift) and at most `TimeResolution` after it; the cylinder ends at rest on its disk; each sphere
ends rolling on the floor with its contact-point speed within `VelocityResidual` of zero; the clip
renders `120` frames, each showing the certified poses at its time.

**Phase 2 — `tumble`.** A fixed tray built as a zero-bound Boolean `Cut`: an outer source box minus an
inner source box that opens its top, leaving a floor and four walls inside `160×160 mm`. The tray's own
floor is the support and the tray is the scene's one fixed body, so no second fixed body touches it. A
mesh `Union` of overlapping boxes is not used, because its triangle diagonals generally cross the other
operand's planes off the dyadic grid, which leaves a positive mesh bound that §9 does not admit. Dynamic: four `20 mm` source boxes released with
proper rotations about `(1, 1, 0)` by `30°`, `45°`, `60°`, `75°` and spin `(2, 1, 0) rad/s`; one
hexagonal prism (`20 mm` across flats, `12 mm` tall) released on a vertex; one triangular wedge; one
stitched tetrahedron. Material as above, with `PenetrationResidual = 10 µm` and `SupportBand = 5 µm`: a box
that lands on a face with a slide ends that step with one edge up to `5 µm` above the floor, and only a kick
that lands on all four corners leaves it at rest (§10.8). `3 s`. Exit criterion: the timeline reaches `3 s`; each body's
final pose is a face-down rest (every dynamic body's velocity within `VelocityResidual` of zero, and in
the last step each body's floor pair either rides a band or persistent track or is excluded by swept
boxes strictly apart while the body hovers inside the `SupportBand`, §10.5); the trace carries at least one
`ContactImpact` whose manifold has a single point (vertex impact), one with two points (edge impact) and
one with four or more (face impact); each box's first impact time matches, within `TimeResolution`, the time
at which the exact drift of its lowest corner enters the `SupportBand`, which is where a falling pair's
first `ContactBand` sample lies (§10.5).

**Phase 3 — `parts-bin`.** The tray plus: a `24×16×16 mm` box shelled `2 mm` through its top
(`cupPayload`, non-convex, exact); a `12 mm` cube whose top loop is chamfered `2.3 mm`, so its feet are
not dyadic and its displacement is its contour's rounding (about `1e-15 mm`, §10.4), and whose held mesh
carries §9.2's convexity certificate (a `2.1 mm` chamfer rounds one foot an ulp in front of a facet plane,
which §9.6's overlap path refuses, §13 PR 20c); a loft from a pushed-out
square to an octagon (exact); a straight `14 mm` sweep of a hexagon (exact, read as its prism); a revolved
bottle, the full revolve of a line-and-arc half-profile (`Ø16 mm` base, a quarter-circle shoulder to a
`Ø8 mm` neck), read as a held mesh at the request's `HeldChord` (§10.4); and a `Ø20 × 30 mm` source
cylinder released on its side on the tray floor with `ω = (−1.5, 0, 0) rad/s` and `v = (0, 15, 0) mm/s`, so
it rolls without slip from its first step. Every other body is released `8 mm` above the floor at a
translation pose, at least `20 mm` inside the walls, as PR 15 places its bodies. Material as above, with
`PenetrationResidual = 0.125 mm`, `SupportBand = 0.05 mm`, `HeldChord = 0.03 mm` (the bottle's `δ` is then
about `0.03 mm`; the band track that carries its rest holds its lifted base at up to `SupportBand + 2δ`,
about `0.109 mm`, inside the residual, §10.4) and `PointResolution = 0.1 mm`, since a displaced body's
witness balls carry its `δ` (§10.4). `4 s`. Exit criterion: `Body.MassProperties` publishes for every body;
the timeline reaches `4 s`; the rolling cylinder's trace carries a rotating band track on the tray's floor
with its contact-point speed within `VelocityResidual` of zero (rolling without slip under friction `0.4`);
the chamfered block and the bottle each rest on a `ContactBand` whose published `Gap.Bound` is at most the
body's boundary displacement plus `SupportBand`; the bottle's landing island publishes a `WitnessSpin`
(§6.3) below `1 rad/s` and each of its resting islands one below `0.05 rad/s`, the spin its `δ` leaves
uncertain in its published zero; the cup rests on four lifted points; every body but the rolling
cylinder ends with both velocities within `VelocityResidual` of zero.

## 3. N-body data model

### 3.1 World and pairs

`WorldConfig` is unchanged. `NewWorld` admits `len(cfg.Bodies) >= 2` in any role mix, and every world
takes the scheduled step of §4.3 and §5. Internally:

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
pair with a moving body. `NewWorld` returns `ErrUnsupported` for a positive-friction pair with a kinematic
body (`dynamics/pairs.go`); §6.4 states where else friction stops. A non-excluded Fixed/Fixed pair is
queried once per `Step` at its constant poses by `ContactPair`; an `Overlapping` or `Undecided` relation, or a `ContactBand`
whose band exceeds `PenetrationResidual` (§10.4), makes the step `Undecided` with `StepFixedPairRelation`
(§12). Exclude such pairs, or model a tray as one body, as the exit
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
impulses.

The cache is pure reuse: dropping it changes no published value except `Solver.Iterations`. It holds
every `SweptBox` and `SweepPair` result the step used, keyed by their exact inputs (§5.3), and every
island the step certified, with its exact problem and final proposal state (§6.2), and the step's
`Completion` conservation reading of those entries, which the next step publishes as its `Input`. A
step reads it only when its entries equal the input state's entries. Within one step, a conservation
reading of entries equal to ones the step already read is reused too: the kicked state starts the first
drift slice and the completed state ends the last. Each reading is a pure function of the world and the
entries, so a reused one is the one a new reading returns.

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
   is `StepTrackUnproved`. A `SweepPersistentBand` of a pair continued under `ContinueCertifiedTouch`
   continues it while its band stays within `PenetrationResidual` (§10.3): a track from the slice start
   to its end whose `Band()` lies within the residual cuts nothing; otherwise the BAND END is the last
   fraction of the sweep's own dyadic grid, no later than the track's end, through which `BandAt` lies
   within the residual, and a track with no positive such fraction is `StepTrackUnproved`. A track over a
   support set (§10.5) ends before any vertex outside its contact set reaches the plane, so that vertex's
   arrival is a band end.
   The CUTTING events are an initial contact (only under `StopAtInitialContact`) at fraction zero, an `ImpactBracket`
   or `ContactTransitionBracket` at its bracket's exact right fraction, and a band end at its fraction; a
   touch that ends inside a slice reaches the step as a transition bracket. Choose the
   earliest cutting fraction `f_e`, and gather the cutting events at exactly `f_e`. A bracket that only
   overlaps `f_e` is not gathered: its pair is swept again from `t_e`, as an initial contact when it
   touches there. A `GrazingTouch` cuts nothing (step 6).
5. **No event.** Advance every body to `dt` on its certified path, run `ContactPair` on every pair in the
   contact set at the completed poses, reject a relation other than separated, touching, or overlapping
   with a bounded manifold whose penetration lies within `PenetrationResidual` (two co-moving bodies drift
   on separately rounded translations and may overlap by an ulp), record the final slice, and publish.
   A `ContactBand` there must lie within `PenetrationResidual` (§10.4). The published state's contact set
   is every pair continued under `ContinueCertifiedTouch` whose final sweep is a persistent touch track
   through `dt`, or a band track that continues the pair through `dt` (step 4).
6. **Advance** every body to `f_e` on its certified path: a swept body takes the pose its pair reports
   replay at `f_e` through `CertifiedPosesAtInterval`, and every report covering it must replay the same
   pose; a body no sweep covers takes the pose `pathPoseAt` evaluates on its own path (§5.4), as
   `Trace.Sample` does. Every candidate records its full report, re-sliced to `[t, t_e]` through `CertifiedPosesAtInterval`; an
   event pair's own report covers its prefix through the bracket's right fraction, so it needs no separate
   prefix sweep; a rotating pair's impact replays its bracket through the right end by the travel from its
   left end's proven gap (§10.1). Every box-excluded pair is checked at the rounded poses (§7.1). The event's held time
   `t_e` is the exact `t + f_e·(dt − t)` when that is a float, else the float just below it, so every time
   the prefix replays maps to a fraction at or below `f_e`; a label that does not pass `t` is
   `StepUnsupported`. Each graze before `f_e` publishes a zero-impulse `ContactGraze` at its instant, after
   its pair's report replays its poses there and its enclosed relative normal speed at them lies within
   `VelocityResidual` of zero; its pair's report replays the whole slice, so the graze needs no cut. A
   graze at exactly `f_e` is `StepUnsupported`.
7. **Islands.** A transition publishes a zero-impulse `ContactTransition` and its pair leaves the contact
   set without entering a solve. The impact, initial-contact and band-end pairs, with every contact-set
   pair whose persistent or band track covers `f_e` (its manifold read through `ManifoldAt(f_e)`), form
   islands over the active constraints at `t_e` (§6.1); only the islands an impact, initial contact or
   band end reaches are solved, so a resting stack elsewhere keeps drifting and publishes nothing. An
   impact solves on the manifold its sweep certified at the bracket's right sample; a rotating pair's
   right sample deviates from its ideal pose and so carries none, and its solve takes the manifold
   `ContactPair` publishes at the rounded event poses, which are the poses the step publishes. A band end
   solves on its track's manifold at `f_e`. Each impact and band end reads `ContactPair` at the rounded
   event poses for §6.6; a `ContactBand` there must lie within `PenetrationResidual`, else
   `StepPairUndecided`, as must an initial contact's band. An initial contact whose `InitialEvent` reads
   `Overlapping` with no manifold within the request solves on the manifold `ContactPair` publishes at the
   slice-start poses, an initial contact that the previous step's contact set held at the same poses
   corrects the penetration step 5 admitted there, and a contact-set pair on a band track with positive `Band()` covering `f_e` whose
   rounded poses there read `Overlapping` is gathered as a band end at `f_e` (§10.8); a gathered pair whose
   rounded poses read `Overlapping` enters the solve whether or not a point closes. Correct positions per island
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
| In the contact set, every solved normal speed `> VelocityResidual` at the last solve, and the event's post poses read exactly `Touching` | `ContinueSeparatingTouch` |
| In the contact set otherwise: a resting solve, or any event whose post poses read a `ContactBand` within `PenetrationResidual`, whatever the solved speeds (§10.7) | `ContinueCertifiedTouch` |

These are the three policies `docs/contact-sweep-design.md` §5.1 defines; nothing here adds one. A pair
gathered at a band end or from a track that enters no solve, because every point separates faster than
`VelocityResidual` (§6.1), keeps `ContinueCertifiedTouch` when its rounded event poses read `Touching` or
a `ContactBand` within the residual, and leaves the contact set when they read `Separated` (§10.7).

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
rigid-dynamics "Step input and configuration" defines it. A kinematic participant moves with its driver
slice's exact velocity field. A translating `PoseSegment` moves every point by its displacement over its
duration. A rotating one follows the screw its endpoints define, read as the exact rationals of its float
axis `a`, axis point `p`, angle `θ` and slide `s` over the duration `T`: a point `x` moves at
`ω × (x − p) + a·s/T` with `ω = a·θ/T`. The field gives each contact point's driver velocity in the
proposal and the certificate, the driver work `J·V(x)` the energy gate charges at each point, and the
closing-speed bound of §6.6; an event reports the field at the driver's pose origin and its `ω`. A driver
whose field cannot be read exactly is `Undecided` with `StepUnsupported` when its pair reaches an island,
and a graze with a rotating driver is `Undecided` with `StepUnsupported`. A pair gathered for the event with no
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
sliding touch at the step start therefore continues on its track without an event; in a later step the
carried contact set continues it without this solve (§3.2).

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
`dynamics`. The certificate runs every row in `internal/proof/shared_interval.go`'s `SInterval`, which
writes every input over one odd denominator `D` shared by the island, as a `Dyadic` numerator over a power
of `D`: each operation chooses the endpoints its `RatInterval` twin chooses, sums and products need no GCD,
and a value becomes a `big.Rat` only where `ContactSolverReport` or a refusal publishes it. It reads its
inputs straight from the held float readings into that form (`dynamics/island_certify_input.go`), with no
`big.Rat` between: a float, and every sum and product of floats the reading forms (a mass-center ball, an
orthonormality defect, a witness ball), is a `Dyadic` over `D^0`, the same rational under every `D`. Only
the inputs `dynamics` holds as `big.Rat` are lifted over `D`: a kinematic driver's exact velocity, a body's
certified inertia floor and row ceiling, and a pair's `μ_lo`. `D` is `1` unless one of
them has an odd factor in its denominator; one whose odd factor `D` lacks widens `D`, and those inputs are
lifted again before any row runs. The pre-solve classification of a gathered pair, which encloses its
relative normal speed at every manifold point, reads its two participants and its points the same way.

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
| Angular law | `I_world·(ω' − ω) − Σ r×J` per component, over the inertia and point intervals | `ImpulseResidual·ρ + λ_lo(I)·AngularVelocityResidual + T_β`, with `ρ` the island's largest lever bound, `λ_lo(I)` the certified lower eigenvalue (rigid-dynamics "World and State") and `T_β` the body's witness torque (below) |
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

WITNESS TORQUE. A contact point is known to its witness ball, whose radius the caller's `PointResolution`
admits (§6.1; contact-geometry §3), so the torque of an impulse about the mass center is known only to that
radius times the impulse: over the lever interval the angular residual spreads by up to `b_k·|J_k|_1` per
component at each point, `b_k` the body's own witness ball (`OnA.Bound` on side `A`, `OnB.Bound` on side
`B`) and `|J_k|_1` the L1 norm of the point's impulse interval at its upper end. A `0.03 mm` ball under the
§2 bottle's `1877 kg·mm/s` landing impulse is a torque of `55 kg·mm²/s`, and no gate can certify the spin
closer than the admitted geometry fixes it. The angular law's limit therefore carries that spread as the
body's WITNESS TORQUE `T_β = Σ_k b_k·|J_k|_1` over the body's points, and nothing else widens it: the
mass-center ball, the normal ball and the inertia intervals stay in the residual alone, so a mass center or
normal too uncertain for the published spin is refused as before
(`TestFacetedFloorImpactRefusesUncertifiedResponse`). The angular-momentum row sums each body's limit and so
carries `T_β` with it. The term is published, never silent: `ContactSolverReport.WitnessTorque` is the
largest `T_β` over the island's dynamic bodies and `WitnessSpin` the largest `T_β / λ_lo(I_β)`, both rounded
up, and a refusal's `Limit` carries it. What the gate certifies, for every admissible inertia, mass center,
normal and contact point at once, is `|I·Δω_pub − Σ r×J|_∞ <= L_β := ImpulseResidual·ρ +
λ_lo·AngularVelocityResidual + T_β`; with `Δω*` the exact law's spin change at the true inertia and the true
contact points, `I*·(Δω_pub − Δω*)` is that residual at one admissible instantiation, so
`|Δω_pub − Δω*|_2 <= √3·L_β / λ_lo`: the published spin lies within
`√3·(AngularVelocityResidual + (ImpulseResidual·ρ + T_β) / λ_lo)` of the law at the true contact points.
That claim is the one every exact fixture already makes, since an exact planar body's `T_β` is the rounding
of its vertices times the impulse and no published number of theirs changes; for the §2 bottle it is the
record of §13 PR 20g: `WitnessSpin` about `0.5 rad/s` at the landing and `0.04 rad/s` at each resting kick.
Raising `AngularVelocityResidual` to the landing's `0.5 rad/s` instead would state one flat tolerance for
every body and every event, widen every body's energy allowance with it (rigid-dynamics "Completion,
conservation, and trace"), and leave the resting kicks' tenfold smaller uncertainty unstated; shrinking `HeldChord` until the
spread fits the old limit needs `δ` below `1e-7 mm`, tens of thousands of base vertices at a classification
cost of seconds per step already at `0.03 mm`.

The velocity rows carry no such term: non-penetration, complementarity, stick and slip enclose each point's
velocity over the lever interval, so a spinning displaced body's point speed spreads by `|ω'|·b_k` (and the
restitution target's by `e·|ω|·b_k`), which those rows refuse beyond `VelocityResidual`. A displaced body
therefore certifies an event only while its spin times its witness ball lies within `VelocityResidual`; the
§2 bottle lands and rests with exactly zero spin.

These gates are the solver's CLAIM, not an admission of a geometric fact: the published state is defined
as "velocities that satisfy the discrete law within the stated residuals", and the certificate proves that
definition holds. CLAUDE.md's reject-only rule governs geometric claims decad is handed; every geometric
input here (manifold, mass, normal) arrives already certified by its producer, and nothing in this table
upgrades one.

### 6.4 Friction families per phase

The cone, stick and slip gates make friction generic, so no per-shape friction solver remains. What limits
friction is the manifold producer: a pair needs a bounded manifold with point and normal balls that keep the
slip and cone intervals inside the limits. A pair whose family publishes a relation only (no manifold)
stops the step with `StepManifoldMissing` when it reaches an island (§6.1), with or without friction;
`NewWorld` refuses positive friction only on a pair with a kinematic body (§3.1).

The model applies every impulse of an island at once, with Newton restitution on each normal and Coulomb
friction read at the post-event velocity. Friction at one contact can therefore drive a body into another
contact: a sphere striking a floor and a wall together with restitution 0.5 and friction 0.5 takes about
twice the frictionless normal impulse at each (`TestThreeBodyFrictionIslandRealPath`, "restitution"). This
is a known property of the model, not a solver error.

### 6.5 Parity with the closed-form fixtures

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
impulses, velocities and poses within its existing `InDelta` slack, except for the rewrites listed
below. A single-point island converges in
one sweep to the isolated formula exactly in float; the four-point and two-pair islands converge to the
same solutions the closed forms publish because those solutions satisfy the same complementarity
system. Every world takes the general step, and no closed-form responder remains in `dynamics`.

Where the general path and a closed form answer differently, the parity run follows these rules:

- A fixture whose closed form refuses an event the general path certifies keeps the general answer, and
  its assertion asserts the certified computed values instead of the refusal, each hand-checked against
  closed-form physics: the falling frictional box face of
  `TestFixedFloorFrictionRejectsUnsupportedMaterialsAndPatch`; the rotating driver, the inexact
  derivative and the torque-driven spin of `TestKinematicDriverRejectsInvalidPaths` and `load_test.go`;
  the tangent approach, off-center face and spinning face of `TestObliqueSupportRefusesUnresolvedMotion`;
  the fixed-A and reversed off-center pairs of `offcenter_pair_test.go`; the lateral slide and bouncing
  stack of `three_body_stack_test.go`; the unequal-mass and asymmetric-speed islands of
  `TestThreeDynamicSimultaneousSphereFriction`; the anisotropic inertia and incoming spin of
  `TestSpherePairInitialFrictionRefusesUnsupportedResponse`; the friction-cone and restitution corners of
  `TestThreeBodyFrictionIslandRealPath` (§6.4); and the overlapping brackets of
  `TestThreeBodyTwoDynamicOverlappingPairEventsRemainUndecided`.
- A closed-form value that contradicts its own impulses follows the impulses: the equal-mass-scale case of
  `TestThreeDynamicSimultaneousSphereFriction` asserts 2.1875, which its certified impulses give, not
  the closed form's 2.8125.
- An impact read at its bracket's right sample (§5) agrees with the exact-time closed form within the
  step's residuals: the interior sphere-pair friction impacts of `sphere_pair_friction_test.go` and
  `sphere_pair_offaxis_friction_test.go` compare impulses within `ImpulseResidual` and velocities within
  `VelocityResidual` and `AngularVelocityResidual` (§14), and their event-time pose reads separated by
  at most `ContactSlop` after §6.6's push.
- A pair §6.6 corrects into exact touch reads `Touching`, where the closed form left a sub-`TimeResolution`
  gap (`TestSpherePairInteriorZeroRestitutionRest`).
- Two world insertion orders sum an island's rows in different float orders, so the reversed world of
  `TestThreeBodyFrictionIslandRealPath` compares its final spin, translation and basis within an explicit
  slack of `1e-12`.
- A driver path whose derivative no float can represent is `Undecided` with `StepIslandDegenerate`
  (`TestKinematicDriverRejectsInvalidPaths`).
- A rotating driver solves inside an island (§6.1), so `TestKinematicRotatingDriverInteriorImpact…` keep
  their assertions, and the turntable of `TestScheduledStepTurntableTouchRefuses`, which a block rests on
  at zero normal speed, now stops at the root package's missing touch track under a turning platform
  (`StepPairUndecided`, `SweepContactTrackUnproved`) with its start as the certified prefix.
- `TestThreeBodyTwoDynamicSphereZeroRestitutionImpact` is a known regression against its closed form
  (§6.6): it asserts the certified impact at 0.3 s and the `StepPairUndecided` that follows it.
- A zero-speed touch publishes no event (§6.1), and `MaxEvents` stops a step when the published events
  reach it with time remaining (§5 step 8).
- Refusal wording and report shape follow §12: an `Undecided` report carries the events of its certified
  prefix and names the general path's reason.
- A test of a closed-form responder's internals is a test of the same fixture through the general step,
  asserting the same computed impulses, velocities and spins. A responder's own bound or residual
  helper (its cross-product error, its correction and impulse residual gates, its rotating-driver
  slice, its combined sphere spin) has no counterpart in the exact island certificate, whose legs
  `island_test.go` and `island_friction_test.go` record, so its fixture keeps only those computed
  quantities, and the event-conservation tampers become island-certificate tampers
  (`TestSpinEventConservationUsesRealPointImpulses`).
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
the sweep certified at the bracket's right sample when it has one, and for a band end, and the gathered
manifold itself for an initial contact or a track. Dynamic bodies joined by contact-set pairs on persistent tracks move as one: their touch is
exact and their velocities equal (§6.2), so they take one translation computed with their summed mass,
which keeps that touch. A group resting on a Fixed or Kinematic body through a persistent track is
anchored and takes no share either, so a body landing on a resting one is corrected alone and the
resting body keeps its exact touch with its support; a penetrating pair with no free body is refused. A
pair's penetration may not exceed its allowance above, its bracket travel bounded
by the bracket's elapsed width times an upper bound on the pair's contact-point speed (the L1 norm of the
linear velocity difference plus each body's spin times its lever, both L1 norms), and a body's
translation length may not exceed the summed allowances of the pairs that moved it, plus the band depth
`ε` of §10.3 when the slice ended on a band track, or `PenetrationResidual` for an initial contact that
the previous step's contact set held at the same poses (§10.8). Island pairs with a moved
body must still be `Touching` at the corrected poses; every other scheduled pair with a moved body is swept
over the correction.

A translation along a float normal rarely lands two curved bodies in exact touch. Two spheres whose center
line is off every axis never do: their center difference is dyadic, and three dyadic coordinates whose
squares sum to the square of a dyadic radius sum have at most one nonzero (a sum of three squares equal to
`4^k` has all three even). An island pair the correction leaves overlapping or apart is therefore settled
in one of three ways. A PUSHABLE body is a dynamic one with a correction allowance; a body the correction
held in place (an anchored group, or a body no penetration moved) has none and moves in none of them, so
a body landing on a resting one is pushed alone and the resting one keeps its exact touch with its support.

- A pair the solve leaves RESTING (some point leaves no faster than `VelocityResidual`, §5.2) is first
  placed back in touch: its pushable bodies move along the event manifold's normal, split by inverse mass,
  apart when the pair overlaps and together when it stands apart. The search steps by the overlap's depth
  plus its bound (or the gap plus its bound, or one ulp of the larger body coordinate when the gap cannot
  be proved), doubling until the relation changes, then halves the interval between the last overlapping
  and the last non-overlapping amount until the two are adjacent floats. The first pose `ContactPair`
  proves `Touching`, or in a `ContactBand` within `PenetrationResidual` (§10.4), is kept, and the pair
  stays in the contact set. A disk resting on a face has that pose (its height is a float), so a disk a
  correction rounds an ulp into the floor rests again.
- A resting pair the search cannot place in touch, unless it continues on a persistent track, and a pair
  the solve separates (every point leaves faster than `VelocityResidual`) are PUSHED apart:
  - still overlapping: the pushable bodies move along the deepest point's normal by that point's depth
    plus its separation bound plus the pair's margin, split by inverse mass, the share doubled until it
    moves the rounded pose;
  - apart by less than `ContactPair` can prove (`Undecided` with `ContactNoGapProof`): they move apart along
    the event manifold's normal, split by inverse mass, by the margin or one ulp of their largest
    coordinate, whichever is larger, and then twice as far each time, until `ContactPair` proves the pair
    separated or touching;
  - a resting pair apart by less than its margin is pushed along the event manifold's normal by the
    shortfall of its proved gap.

  A separating pair has no margin and ends just apart. A resting pair's margin is half of `ContactSlop`,
  which every pair's allowance carries: at an ulp apart, the smallest correction of a neighbor later in
  the same step would reach it, as when a sphere rests between two others and each of its pairs lands
  separately.
- A resting pair on a persistent track the search cannot place in touch is refused: its track needs exact
  touch.

The pair is then checked again, for at most four passes over the event's islands. It must end `Touching`
(or in the band) or `Separated`, and each moved body's whole translation from its pre-event pose, measured
as the correction's, must stay within its correction allowance; a separated corrected pose within that
allowance is what rigid-dynamics "Response" admits. A pushed pair that ends touching continues under
`ContinueSeparatingTouch` when the solve separates it; one that ends separated leaves the contact set, so
its next slice starts under `StopAtInitialContact`, since the rotating sphere-pair sweep needs a touching
start for a departure and a resting pair needs exact touch to continue. A resting pair placed in touch
continues under `ContinueCertifiedTouch`. A resting pair apart takes the next step's kick as a new impact,
as a step that starts in touch does. Under a positive `SupportBand` (§10.5) the band a resting planar pair
may end in includes an exact pair apart by at most the band, whose `ContactBand` publishes its support set.

Two spheres a glancing frictional impact sets spinning meet again through the root package's rotating
sphere-pair sweep: from that clear start it brackets their next impact from the exact affine paths of
their centers, which their spin about those centers does not move (contact-sweep §4.5).

A sphere pair that continues in persistent touch after an interior impact is refused at its next replay:
the root package's sphere-pair persistent replay requires the two rounded centers to stay exactly one
radius sum apart at every replayed fraction, and centers corrected at the bracket's right sample are not
dyadic enough for both translations to round alike. No correction makes that hold in general (two centers
in different binades round the same displacement differently), and the certificate is not weakened to
admit it. `TestThreeBodyTwoDynamicSphereZeroRestitutionImpact`, which the closed form advanced, is
therefore a known regression: its step publishes the certified 0.3 s impact, both spheres leaving at
50 mm/s, and stops `Undecided` with `StepPairUndecided` from that impact. The corrections of one island are applied together, then every
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

`docs/dynamic-mass-design.md` owns the integrals and admission gates; this section fixes which path each
payload's mass takes. Each item carries the computed test of dynamic-mass §6 for that shape.

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
`delta`; a zero-bound Boolean or its translation-only placement; and any other solid payload whose
§10.4 held mesh has zero `δ`, an exact loft, cup or one-span straight sweep among them. A body with a
positive boundary displacement takes §10.4's held mesh at that `δ`.

A stitched solid is read off its own audited triangle set (`stitchPayload.tris` over `verts`, with
`triFaces` naming each triangle's live face), which `Stitch` assembles only for a closed all-planar weld
whose crossing audit passed (`auditClean`); a curved or mixed stitch holds no triangle set and is not
admitted, nor is an open sheet. Its vertices are the welded table's floats, exact dyadics, and its
displacement is the largest per-vertex weld bound (`vertBound`, each the class bound with the placement
rounding `delta` already added), and at least `delta`: zero for a weld built at the identity whose every
class is zero-bound, which is then a §9
body, and positive for a placed or certificate-welded stitch, which is a §10.4 held mesh with that δ. The
scene's tetrahedron is the former: four patches on the `XY`, `XZ`, slanted and diagonal planes, welded at
the identity, every vertex bound zero, every face planar (`contact_faceted_pair.go`, §13 PR 14b).

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

- a certified transversal crossing of two facets (`internal/meshbool/boolean_exact.go`'s predicates) proves `Overlapping`;
  the kernel records every crossing it finds, an edge of one body through a facet of the other or two
  coplanar facets of positive overlap, as `PlanarResult.Crossings`, each naming the facet and the edge's
  two facets, so §9.6 can tell which faces a shallow overlap passes through;
- with no crossing, one nesting cast per shell (`docs/clearance-design.md` §2's ray ladder, closed-form
  for planes) proves containment, hence `Overlapping`, or mutual outsideness;
- with no crossing and no nesting, the exact minimum over facet pairs of the squared distance
  (vertex-face and edge-edge candidates, rational) is either positive, proving `Separated` with the gap
  enclosed between two floats whose squares are compared exactly against it, or zero, proving `Touching`
  when every zero-distance feature pair has opposed material sides.

The distance scan visits its candidates in a fixed index order and records the first one that reaches
the minimum. Each body caches, per partner body, the candidate pair that set the last minimum of their
relation. The next relation of the pair reads that pair's exact distance first and uses it only to
prune box pairs farther than it, never as the minimum, so the recorded candidate and every outcome are
the same with or without the cache (`internal/pair/planar_prune.go`).

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
pose with a positive determinant, which is every pose §9 admits). A dynamic body without it gets a
manifold only against a body that carries one (§9.3, §9.6) or through §10.5's non-convex guest rule, and
otherwise relations only, with the appended reason `ContactNonConvex` on an absent manifold. A Fixed or
Kinematic body needs no certificate. `ContactPair` does not know motion types: it names
`ContactNonConvex` on an overlapping pair when neither body carries the certificate, since §9.3 and §9.6 need one convex side, and on a
touching pair of two such bodies that §10.5's non-convex guest rule does not cover.

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
| Under a positive `SupportBand`: the lifted set of each support plane (§10.5) | each lifted vertex with its exact foot, after the rows above | the plane's face normal |

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
  non-convex `B` is not admitted and withholds the manifold, since §10.5's non-convex guest rule runs only
  when neither body is convex;
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
When only one body has a face across the translation, the other pokes through that face with an edge or
a vertex: its deepest vertices along the translation, which must be one vertex or the two ends of one
edge, are published as §9.3's support row publishes a touching edge or vertex, each paired with its foot
on the face's plane, the edge clipped to the face and every piece reaching the face's interior, at the
same depth. This is the shape a rotating pair's impact shows at its bracket's right sample: a turned box
lands an edge or a corner first, never a face. A tied minimum, an edge-cross axis, bodies that do not
cross along that axis (one holding the other), or a non-convex body keeps `Overlapping` on this path;
§9.6 then publishes the patch of a convex body poking through one face of any planar body, convex or
not, and only a pair neither path covers withholds the manifold.

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
A touch or overlap of two oriented source boxes that their patches leave without a manifold takes this
section's manifold when the exact planar relation agrees and every published point is an edge or a
vertex in, on or crossing the other box (§9.3's support and crossing rows and the shallow edge or
vertex): a turned box on its edge. Face pairs stay with the box patches, which withhold the degenerate
ones.

**Positive-displacement bodies.** A body whose held boundary carries `δ > 0` reaches this path only
through §10.4's held mesh, under a `ContactBand` or a held overlap deeper than `δ`: the clip runs on the
held vertices, which are exact rationals at the pose, and §10.4's charges follow — every point ball
widened by its body's `δ`, and `Separation` carrying the band instead of an exact zero, or the held depth
widened by the summed `δ`. The clip itself is unchanged; only the published bounds differ.

**Tests** (`apitest/contact_faceted_manifold_test.go`, PR 11), every one asserting computed coordinates:

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

### 9.6 Shallow penetration through a face of any planar body

§9.3's shallow-penetration patch separates two convex bodies along a global separating axis. A convex
body `M` whose corner or edge pokes through one face of a non-convex body `S` — a turned box landing on
the floor of a `Cut` tray, which every rotating impact's right sample shows (§10.1) — has no such axis:
the separating-axis argument needs both bodies convex, and `S`'s walls rise past any floor-normal axis.
The FACE-LOCAL patch publishes that overlap from the one face it passes through. With `h` a flat face of
`S` (every triangle coplanar, one outward normal `n`, exact), the kernel
(`internal/pair/planar_face_penetration.go`, `PlanarFacePenetration`) publishes a manifold when all of
the following hold, each an exact rational test:

1. **One crossed face.** Every crossing §9.1 recorded between `M` and `S` lies in a triangle of `h`, or
   in an edge of `S` both of whose triangles belong to `h`; a crossing in any other face of `S`, or no
   crossing at all (one body nested in the other), publishes nothing. Two faces crossed — a corner in a
   tray's inner corner — withhold the manifold with `ContactAmbiguousFeature`, as §9.3's tied minimum
   does.
2. **One deepest feature.** The vertices of `M` at the least height `n·(p − q)` below `h`'s plane
   (`q` a vertex of `h`), at depth `d = −min/|n| > 0`, are one vertex or the two ends of one edge; the
   deepest set with a triangle of `M` in it is a face, which this path does not publish.
3. **The sunk part lies over the face.** Every vertex of `M` on or behind the plane, and the exact point
   where each edge of `M` crosses the plane, projects along `n` strictly inside `h`'s region (§9.3's
   `locate` test, holes included); no edge of those projections' convex hull meets a loop of `h`; and
   no loop vertex of `h` lies in that hull. The first two put the hull's boundary in the region's
   interior, so a loop can reach the hull only by lying wholly inside it, which the third refuses (a
   hole under the sunk part). `M` is convex, so its sunk part is the hull of those points, and this
   places the whole sunk part over `h`'s material. A sunk part that leaves `h` in general position
   also crosses `h`'s rim, which condition 1 refuses first; this test covers the crossings through a
   vertex or along an edge that §9.1 does not record.
4. **The column is clear.** Every triangle of `S` with a vertex strictly in front of `h`'s plane
   projects along `n` strictly apart from the projection of `M`'s vertex hull (§10.6's column test with
   a zero margin). Material of `S` in front of the plane therefore meets no part of `M`, so `M ∩ S` is
   exactly the sunk part within `h`'s slab, of depth `d` along `n`; a triangle of `S` that `M` swallows
   whole, which §9.1's crossings cannot see, is refused here.

The published points are §9.3's shallow row: each deepest vertex paired with its exact foot on `h`'s
plane (the feet of an edge clipped to `h`'s region, each piece reaching the interior), `Separation` an
exact enclosure of `−d`, the normal `h`'s exact outward normal oriented `A` toward `B`, and the features
the vertex's or edge's faces and `h`. Under a positive `SupportBand` the lifted set of `h` follows
(§10.5's `Overlapping` row, `PlanarSupportSets` with `overlap` set). Reversal swaps sides as §9.5 states.
`publishPlanarManifold` runs this path whenever §9.3's convex-convex path publishes nothing, including
for a pair §9.3 never reads because only one body carries the convexity certificate. Each certified
body in turn is tried as `M` against the other as `S`; the patch is published when exactly one order
publishes, and two publishing orders withhold it with `ContactAmbiguousFeature`, so the two query orders
agree. A pair with no certified body keeps `ContactNonConvex`, and a patch the four tests refuse for any
reason but condition 1's keeps the reason `ContactPair` already carries.

The claim is local and §6.6 consumes it locally: the correction translates the pair apart by `d` along
`n`, and `ContactPair` at the corrected poses, which must read `Touching` or a `ContactBand`, is what
admits the result (reject-only; rigid-dynamics "Response"). Condition 4 is what makes the published
contact set complete: without it a bump of `S` inside `M` would leave the event's manifold claiming
one vertex while the bodies also meet elsewhere.

Rejected: reading the deepest vertex against every face plane of `S` and publishing the shallowest.
A non-convex `S` has face planes that cut through its own material (a wall's plane crosses the floor
slab), so "shallowest over all planes" is not a penetration depth of `S`; only a face the overlap
passes through has one.

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
heights must stay above minus its depth widened by that deviation. Inside an impact bracket, after its
left edge `lo` and through its right edge, every ideal vertex lies within `(f − lo)·T` of its place at
`lo`, `T` the two travel bounds per unit fraction, where the pair was the left end's lower gap `g` apart;
the fraction replays when `(f − lo)·T − g` plus the summed deviation fits `PointResolution`, so the
rounded pair lies within that resolution of a separated one, the claim the affine source-box replay makes
inside its bracket. The rotating source-box replay does the same with its left sample's lower gap, so a
rotating pair's impact advances to the bracket's right end (§5 step 6). Beyond a bracket's right edge or
a track's end, replay refuses. `T` charges every point's full travel, so a bracket `TimeResolution` wide
can exceed `g + PointResolution`: the `tumble` tetrahedron's second impact on the tray floor, with `T`
about `2200 mm/s` and `TimeResolution = 1 ns`, carries `2.0·10⁻⁶ mm` against `g ≈ 1.0·10⁻⁶ mm`. The
search narrows such a bracket below `TimeResolution`, while its right edge would refuse, by halving: a
meeting midpoint becomes the right edge, a separated midpoint the clear certificate joins to the left
edge becomes the left edge (contact-sweep §6). The float floor, the pose budget and a midpoint that
settles neither leave the bracket found, so narrowing never turns a bracket into `SweepUndecided`.

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

The plane is a SUPPORT PLANE: the plane of a flat face of `B` with every vertex of `A` on or in front of
it, the contact set being `A`'s vertices on it, and with `B`'s material in front of the plane kept
laterally clear of `A` by §10.6's column test; a convex `B`, every vertex of which lies on or behind the
plane, passes that test with nothing to check. Each body lies in its vertices' hull, so the least vertex
height bounds the pair's separation below, whatever the shapes, until it exceeds the column's lateral
clearance (§10.6). The
run tries `B` then `A` as the plane's owner, each over its triangles in order, and takes the first plane
whose every contact rate is positive. The horizon is the largest fraction on the sweep's dyadic grid
(`TimeResolution`, capped at 52 levels so it stays a float) at which `c − K·h > 0` and every non-contact
bound `h_p(0) + h_p'(0)·h − K·h²` is positive; both are monotone in `h`, so a binary search over the
grid finds it. A touch no support plane covers — two crossing edges, or a box in a tray corner, which
touches two faces whose planes each hold part of the box behind them — stays `Undecided`; a box on a
tray's floor, away from its walls, has the floor's face-local plane (§10.6). After the horizon, §5's
search continues on the remainder as today.

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
Bound. Every non-contact vertex keeps `h_p(u) > 0` by §10.2's bound from its positive `h_p(0)` (a rested
lifted vertex, §10.8, is held two-sided instead), and
`A` in front of the plane with `B` behind it bounds the overlap. Every contact vertex's foot must stay
inside `B`'s face: the vertex's ideal path box over `[0, h]`, less `B`'s translation and grown by `Depth`,
is projected along the axis of the normal's largest component, and must meet no bounding edge of the face
with a corner inside one of its triangles. The face's triangles must belong to one `Face`. The track ends
at the largest grid fraction where both checks hold. A track with zero depth that reaches the duration is
an exact `SweepPersistentTouch`. `ManifoldAt(fraction)` stages each contact vertex through the rounded
pose and publishes it with its exact foot on `B`'s rounded plane; both balls carry the pose deviations
(`B` only translates, so its plane keeps its normal), `Separation` is `[−Depth, Depth]`, and the normal
is the face normal.

`BandAt(fraction)` publishes the same bound over the track's prefix through `fraction`: `r·t + K(t)·t²`
at `t` the fraction's elapsed time, over `|n|`'s lower bound, widened by §10.4's `2δ`. `K(t)` covers
`[0, t]` and the track's clearance and face checks hold through its end, so the prefix bound holds at
every earlier instant; at the track's end it is `Band()`, and it grows with the fraction.

`dynamics` consumes a band track as a persistent contact while its depth stays within
`PenetrationResidual` (§5 step 4): a track that spans the slice within the residual continues the pair,
which stays in the contact set. Otherwise the slice ends at the BAND END, the last fraction of the
sweep's own dyadic grid, no later than the track's end, through which `BandAt` lies within the residual;
`BandAt` grows with the fraction, so a binary search over the grid finds it. The pair then enters an
island at the band end with its track's manifold there, the correction (§6.6) removes the depth the
rounded poses show within the allowance widened by the band, and the next slice continues it afresh, so a
slow tip proceeds through several short band tracks, each certified. A band end is a grid fraction rather
than the exact root of the band's quadratic: the root is in general not a float, and §5 step 6 refuses
an event fraction that is not one. Replay inside a band slice checks the rounded pose's exact vertex
heights against `[−Depth − deviation, …]`. With a positive `SupportBand` the track runs over §10.5's support
set, the contact set plus the vertices within the band, and ends before any of those reaches the plane.

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
read off its payload with `δ` its mesh bound, and any other solid payload without an exact contact
family of its own (a cap-loop chamfer, a cup, a loft, a sweep, a general revolve; a source sphere or
cylinder keeps its exact path), read off its own `VerifyAll` tessellation with `δ` that mesh's `Bound`,
which is the two-sided displacement between the mesh and the true boundary (`tessellate.go`), zero when
nothing rounds, which admits the body to §9 as an exact body. A body whose every face is planar is read at
a chord of `1 mm`, which chords nothing; a body with a curved face is read at the request's `HeldChord`:

```go
HeldChord units.Value // nonnegative Length; zero admits no body with a curved face
```

`SweepRequest` reads it through `ContactRequest`; a negative, non-finite or non-`Length` value is an
input error at both entry points. The chord is the caller's, as `SupportBand` is (§10.5): it fixes `δ`,
and with it which residual admits the body, and a zero chord leaves a curved body `Undecided` rather than
chording it at a width the caller never stated. The held snapshot is computed once per body and chord
and cached on the body beside its §9.2 certificate; the cache changes no outcome. A held mesh moves
through the exact float query pose, whose linear part is orthonormal only to rounding, so `δ` at a pose is
the body's figure times `s = max(1, (1 + g)/2)`, `g` the largest absolute row sum of the basis's exact
Gram matrix, an upper bound on the pose's stretch. A held vertex is a point of the held mesh and nothing
more: `Bound` does not place it on the true surface, so every witness read off a held vertex carries its
body's `δ` in its ball, below and in §10.5.

**Relation.** With `δ` the two bodies' displacements summed at the query poses, every true boundary
point lies within `δ` of the held pair's. `ContactPair` reads §9.1's exact held relation:

| Held relation | Published |
|---|---|
| `Separated`, gap lower end above `δ`, and no support plane with a lifted set | `Separated`, the held gap with `δ` added to its bound |
| `Separated`, gap upper end at most `b = max(SupportBand, δ)`, and some support plane of the zero-`δ` body has a nonempty lifted set within `b` | `ContactBand`, `Gap = [0 ± (g + δ)]`, `g` the held gap's upper float, with the lifted sets as the manifold (§10.5) |
| `Touching`, or `Separated` with gap upper end at most `δ` and no lifted set | `ContactBand`, `Gap = [−2δ, 2δ]` |
| `Overlapping` with a vertex deeper than `δ` inside the other body (`pair.PlanarDeepVertex`) | `Overlapping`, with the held penetration manifold charged, when one publishes |
| `Overlapping` of two convex bodies whose §9.3 shallow patch has depth at most `δ` | `ContactBand`, `Gap = [−2δ, 2δ]` |
| anything else | `Undecided` |

The lifted band `b` reads `δ` from below: a resting displaced pair is placed with its held gap at most `δ`
(§6.6), and a held vertex within `δ` of the plane is as near to touching as the body's own precision can
tell, so the lifted set is never empty at a rest the band admits, however small `SupportBand` is. The
width is a selection of which vertices are published, not a claim: each lifted point's claims are the
exact ones of §10.5 charged with `δ`, so the published set is sound at any `b`. The true gap lies in
`[g_lo − δ, g_hi + δ]`, inside `[−(g + δ), g + δ]`.

A band from a held touch or shallow patch carries that held manifold charged afterwards (§9.4): each
witness ball grows by its own body's `δ`, and every `Separation` is the band. A lifted point of a displaced
pair carries the same charges: its `M` witness ball grows by `M`'s `δ`, its `S` witness ball by `S`'s, and
its `Separation` is the vertex's exact height widened by the summed `δ`, since the true `M` boundary lies
within `M`'s `δ` of the held vertex and the true `S` face within `S`'s `δ` of the held plane. A held overlap
deeper than `δ` is a true overlap, and its manifold is the held one §9.3's convex-convex path or §9.6's
face-local path publishes, charged the same way, with §9.6's conditions read over `M`'s held vertices
grown by `M`'s `δ`, so that a true `M` poking a second face within `δ` is refused as the held one would
be; a landing bracket places a body of `δ` near `1e-15 mm` deeper than `δ` at every rounded event pose, so
without that manifold no such body could land. The normal is published only
when, at every point, it is the exact face normal of a body with zero `δ` read at a face that holds the
point; a held face of a displaced body only approximates its true face's direction, so otherwise the
manifold is withheld with `ContactNoNormalProof`, as it is for a band from a held gap, and a support plane
hosted by a displaced body publishes no lifted set.

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
balls of each point by that `δ`. Replay of a clear span needs the proven lower gap, which every clear
sample and hull gap already states for the true bodies with `δ` subtracted (`intervalAxisGap`,
`separatedIdealGap`), to exceed the summed pose deviation plus the TRANSFER CHARGE `‖R_r − R_i‖_F·δ` for
each body, `R_r` the rounded pose's float basis and `R_i` the ideal rotation's interval enclosure at the
fraction: a true point is `x + e` with `|e| <= δ` in the body's frame, its rounded image differs from its
ideal one by the held deviation plus `(R_r − R_i)·e`, and `|(R_r − R_i)·e| <= ‖R_r − R_i‖_F·δ`. A path that
only translates has `R_r = R_i` exactly and charges nothing; a rotating path charges a few ulps of `δ`.
The triangle-inequality bound `(s + 1)·δ` would refuse every rest of a displaced body at a held gap near
`δ`, so the charge is the tight form. A departure's lower gap reads held heights and needs no `δ`, since only a
zero-`δ` pair departs (`ContinueSeparatingTouch` is refused to a displaced pair). A track
replay keeps reading the held depth against the held vertices.

**Dynamics.** `dynamics` treats `ContactBand` as a touching relation whose penetration bound is
`Gap.Bound`, admitted when `Gap.Bound <= PenetrationResidual`, and a band track by §10.3's
`BandAt`, whose depth already carries the `2δ`. An initial contact or rounded event poses in a band
beyond the residual are `StepPairUndecided`, a completed contact-set pair is `StepTrackUnproved`, a
corrected island pair must touch or lie in an admitted band, and a non-excluded Fixed/Fixed pair in a
band beyond the residual is `StepFixedPairRelation` (§3.1), each with `PenetrationResidual` as its
`Limit`. A positive-`δ` body therefore needs a `PenetrationResidual` above `2δ`, and one that rests on a
lifted set (§10.5) above `max(SupportBand, δ) + 2δ`, since the band track that carries its rest holds each
lifted height, at most that band, widened by `2δ`; the caller sets it, and a tighter residual leaves the
pair `Undecided` at the first band track after its landing, never silently touching. The landing solve
itself certifies through §6.3's witness torque: every lifted point's witness ball carries `δ`, so the
landing impulse's torque about the mass center is known only to `δ` times the impulse, which the angular
law's limit carries and the island publishes as `WitnessTorque` and `WitnessSpin`.

Curved source families with their own exact occupied sets (sphere, axial cylinder) keep their exact
paths; curved families without one (a cylinder rolling on its side, cone, torus) enter contact-geometry
§7 stage C2 through the clearance kernel's face-pair table for the relation, with manifolds from the
certified ruling feet of `docs/clearance-design.md` §6 (contact-geometry §4.5) and normals from
`Face.NormalAt`. A full source cylinder's ruling is the one such manifold that ships: at identity query
poses (§13 PR 19), and against a signed-axis face of an exact planar body at any pose, rolling included
(§13 PRs 20 and 20e).

**Rolling.** A cylinder rolling on a floor takes §10.3's track with the ruling's two ends as the
contact set (`contact_sweep_rolling.go`). `SweepPair` admits a full source cylinder `M` at any start
pose with a positive determinant whose path rotates, against an exact planar body `S` with zero `δ`
whose path only translates; `ContactPair` must prove the start a ruling touch or the placed band of
contact-geometry §4.5. The support plane is the plane that section picks: a face plane of `S` whose
normal `n̂` is a signed axis, with every vertex of `S` on or behind it and the cylinder's axis nearly
across it. A cylinder has no vertices, so the proof reads the centers `c±` of its end disks, material
points on its axis, staged exactly through the start pose's float basis `B`. That basis is orthonormal
only to rounding, so the start body's section is the disk's image under `B`, with `gram` and
`α = n̂·Bâ` as in contact-geometry §4.5, `â` the identity axis. Over an end disk the least height is
`n̂·c − r·|P·Bᵀ(u)n̂|`, `B(u) = R(u)·B`, and the cylinder's is the lesser of its two disks', since
height is affine along the axis. With `H = n̂·c − d − r` for the plane at offset `d`, each rim's least
height is `g = H + r·(1 − |P·Bᵀ(u)n̂|)`, and with `ã = Bâ`

```text
H'(0)  = n̂·(v_M − v_S + ω×(c − c_M))                      exact
|H''| <= |ω|·|ω×(c − c_M)|                                 c'' = R(u)·(ω×(ω×(c − c_M)))
|r − r·|P·Bᵀ(u)n̂|| <= r·(gram + α(u)²)                     |P·Bᵀ(u)n̂|² ∈ [1 − gram − α(u)², 1 + gram]
|α(u)| <= |α| + β·u                                        β = |ω×ã|, |R(u)ã − ã| <= β·u
```

so `|g(u)| <= c₀ + (|H'(0)| + 2·r·β·|α|)·u + (K + r·β²)·u²` with `c₀ = |H(0)| + r·(gram + α²)`, `K`
half the curvature bound, each term the larger over both ends, and `Depth` is that bound at the track
end; `BandAt` reads it at a prefix's end. A signed-axis start with `α = 0` has `gram = c₀ = 0`, and
the bound is `|H'(0)|·u + (K + r·|ω×â|²)·u²`. Each rim's lowest point lies within
`r·(3·gram + (3/2)·|α(u)|)` of `c − r·n̂` while `|α(u)| <= 1/4` (contact-geometry §4.5), so a start
other than an exact one ends its track where `|α| + β·t` reaches `1/4`; an exact start's drift
`r·√(2·(1 − s)) <= (3/2)·r·β·u` holds with no gate. One foot box spanning both ends' ideal path boxes,
grown by the depth and that drift, must stay inside `S`'s face, so the whole ruling does, and
`ManifoldAt` grows both balls of each end by it beside the pose deviations. `M`'s pose deviation reads
the eight corners of its identity disk-by-interval box, whose hull holds the cylinder. The track
publishes `S`'s exact face normal, as §10.3 does; the axis tilt that would turn the cylinder's own
normal is charged into the band instead. A cylinder rolling without slip about its own axis from a
signed-axis start has zero depth and is an exact `SweepPersistentTouch` over a whole turn; from any
other start its depth is `c₀`, the rounding of the start basis, about `1e-15 mm` after a few turns.
Replay checks each rounded end center's height less `r` against `Depth` widened by both pose
deviations. A start `ContactPair` does not prove touching or banded stays `Undecided`: a separated
start with `SweepMissingBound`, since no clear search covers a rotating cylinder.

`dynamics` rolls such a cylinder with no rolling-specific step code. Under gravity each kick takes the
pair out of the contact set, so every step opens with the initial contact of the ruling's two ends at
time zero, whose island solve stops the kick and whose friction rows leave the contact point at rest,
and the pair then continues on the track under `ContinueCertifiedTouch`. Without a kick the contact set
carries the pair into the next step on its band track. The first step from a signed-axis pose rolls on
an exact touch track; every later step starts at a turned pose and rolls on a band track whose depth
`PenetrationResidual` admits.

### 10.5 The support set and flat rest

A body that lands on a face never lands exactly flat: its pose is a float rotation, so §9.1's exact relation
sees one edge or vertex on the support plane and the rest of the face a little above it. Under the one-kick
step three things then follow for a §9 body. A support on one edge turns into a rotation onto the other: the
`8 mm` cube of `dynamics/tip_test.go` lands its far edge at `34 rad/s`, and the frictionless two-point impact
turns it back at about `6.5 rad/s`, which brings its near edge down at about `52 mm/s`. §10.3's band track over
the exact contact set cannot carry that edge's arrival: the track ends where the arriving vertex's lower height
bound reaches zero, which lies inside the first grid step once the edge is within `52 mm/s × TimeResolution`
of the plane, and a sweep that starts in touch has no bracket for a second feature's impact. And a correction cannot in general land a second feature
in exact touch: a vertex's height at a pose is a sum of products of float entries, and a float translation
cancels it exactly only when that sum is itself a float, which a rotation about `Y` with dyadic levers gives
and a rotation about `(1, 1, 0)` does not. A face whose unit normal is irrational in its body's own frame (a
side of a regular hexagonal prism, the slanted face of a wedge, every face of a tetrahedron) has no float pose
at all that makes it coplanar with the floor, because the pose's third row would have to be an exact multiple
of the face's normal.

The design therefore does not seek exact flatness. It publishes, carries and solves a SUPPORT SET: every vertex
of the touching body that lies within a caller-stated band above the support plane, whether its exact height
is zero or not. The band is the one `ContactBand` already states for displaced bodies (§10.4), read from above
for exact bodies, and `dynamics` admits it as it admits that one, within `PenetrationResidual`.

**Admission.** `ContactRequest` gains one field:

```go
SupportBand units.Value // nonnegative Length; zero publishes the exact contact set alone
```

`SweepRequest` embeds it through `ContactRequest`, so `ContactPair` and `SweepPair` read one value. A negative,
non-finite or non-`Length` value is an input error at both entry points. `validateStepConfig` requires
`Contact.SupportBand <= PenetrationResidual` (`ErrInvalidInput`), since every published band must pass the
residual. The field is not derived from the residual: a positive band changes which relation and manifold a
pair publishes, and the caller chooses that. Every shipped fixture keeps a zero band and its published numbers.

**The support set.** For a face plane of `S` with every `M` vertex on or in front of it, read with its exact
unnormalized normal `n` through an `S` vertex `q`, the support set of `M`, convex or not, is every `M` vertex `p`
whose exact height `h_p = n·(p − q)` lies within the band, `h_p² <= SupportBand²·(n·n)`, and whose exact foot on
the plane lies strictly inside `S`'s face there (the `locate` test of §9.3's support row, holes included). A
foot on the rim or outside is not published, so a face overhanging `S` publishes only the vertices over `S`.
Its CONTACT SET is the subset at zero height, which is §10.2's contact set; its LIFTED SET is the rest. The
comparison is exact, through squares, and admits a vertex only when its true height is proven at most the
band. `ContactPair` reads any such face plane, so a tray's floor and wall each publish theirs even though
neither holds the whole tray behind it. The sweep reads only a support plane of §10.2, which also has every
`S` vertex on or behind it: `M` lies in its vertices' hull, so the lowest vertex height then bounds the pair's
separation below, whatever the support set holds. Convexity of `M` is not needed anywhere in the set: every
point of `M` is a convex combination of `M`'s vertices, so a point of `M` within the band over the plane has
a vertex of `M` within the band, and each lifted point's claims (an exact vertex of `M`, its exact foot
inside the host face, its exact height) hold for any `M`; `planarLiftedSet` reads every guest. The §2 cup
is non-convex and rests on its lifted set. A non-convex guest at a `Touching` relation, which §9.3 withholds
with `ContactNonConvex`, publishes a manifold when every zero-distance feature pair §9.1 recorded lies on one
support plane of `S`, every `M` vertex on or in front of it and each contact's foot inside the host face:
that plane's contact set (its `M` vertices at zero height, each strictly inside the face) followed by its
lifted set, with the plane's face normal, each contact vertex an exact touching vertex with an exact foot,
which needs no convexity either. A contact lies on the face when its `S` feature is a facet of the face, or
an edge or vertex one of whose faces it is; `PlanarGuestTouch` (`internal/pair/planar_manifold.go`) tries
every such face of each body in turn as the host and publishes every one whose contact set is nonempty.
§9.4's clipped patch, which would add the crossing points of a face overhanging `S`, needs §9.3's
certificate and is not attempted; a contact feature off that plane keeps `ContactNonConvex` with no manifold.

**`ContactPair`.** With a positive `SupportBand` the exact planar path (§9.1, `contact_faceted_pair.go`)
publishes:

| Exact relation | Published relation | Manifold |
|---|---|---|
| `Touching` | `Touching`, `Gap` exact zero | §9.3's manifold, then, for every support plane whose contact set §9.3 covered by a face-face or support piece, that plane's lifted set |
| `Separated`, gap upper end at most `SupportBand`, and some support plane has a nonempty lifted set | `ContactBand`, `Gap = [0 ± g]`, `g` the exact gap's upper float | the union over those planes of their lifted sets |
| `Separated` otherwise | `Separated` as today | none |
| `Overlapping`, shallow (§9.3) along a face normal of `S`, `M`'s vertex or edge poking through at depth `d` | `Overlapping` as today | §9.3's poking points, then every other `M` vertex with signed height in `[−d, SupportBand]` over that plane whose foot lies inside the face, at its signed height |

A lifted point publishes the vertex as its `M` witness (exact, converted once with its ball), the exact foot
as its `S` witness, the plane's exact face normal oriented `A` toward `B` with §9.3's ball, the vertex's
`ContactFeature` and the face's, and `Separation` as its exact signed height rounded once with that rounding
as the bound. A lifted point's interval therefore does not contain zero; contact-geometry §3 states the
exception. Lifted points follow the exact points, in support-plane order and then `M`'s vertex order, so the two
query orders publish the same set reversed (§9.5). Several planes (a box against a tray floor and a wall)
publish per plane with their own normals, as §9.3's last row does. Two oriented source boxes take this path
for a touch, a shallow overlap or a gap within the band, as §9.4's last paragraph routes their edge and vertex
touches today: the box patches certify exact touches and know no band. `internal/pair/planar_manifold.go`
gains `PlanarSupportSets`, which takes the two snapshots, their support planes and the band and returns the lifted
points with their exact heights; `contact_faceted_manifold.go` maps and publishes them.

**`SweepPair`.** A §9 pair whose first sample is `Touching`, or `ContactBand` from this path, continues under
both policies over the support set (`contact_sweep_band.go`; `planarContinuation` in
`contact_sweep_faceted.go` admits a `ContactBand` initial event of two source boxes as it admits a touch):

- `planarSupports` admits a plane whose support set is nonempty and records each lifted vertex's exact start
  height `h0` and start rate `h'(0)`.
- §10.2's departure needs every CONTACT rate positive; a lifted vertex is a clear vertex there, positive at the
  start and bounded by §10.2's quadratic. A `ContactBand` start of an exact pair can therefore depart under
  `ContinueSeparatingTouch` (its contact set is empty and every vertex is positive), unlike a displaced pair's
  band start, which never does (§10.4); the two are told apart by the pair's zero `δ`. The departure's last
  sample may itself read `ContactBand`, the pair still apart by at most the band; its `GapAtUntil` is then
  `ContactPair`'s gap at the same rounded poses under a zero band, transferred to the ideal path as a
  separated sample's is.
- §10.3's band track runs over the support set. Its horizon is the largest grid fraction through which the
  curvature is bounded, every clear vertex keeps a positive lower height bound, every LIFTED vertex keeps a
  positive lower height bound `h0 + h'(0)·t − K·t²` as well, unless §10.8 rests it, and every support-set foot
  stays inside `S`'s face
  (the `contains` box test, grown by the depth). A lifted vertex that would reach the plane inside the slice
  therefore ends the track before it does: an arrival is a band end, at most one grid step before the arriving
  vertex's exact height reaches zero. Nothing penetrates inside a track that a contact vertex's own band does
  not already allow.
- `Depth` at elapsed time `t` is `max(r·t + K·t², max over the lifted set of h0 + max(0, h'(0))·t + K·t²)`
  over `|n|_lo`, widened by §10.4's `2δ`. A contact vertex lies in `[−(r·t + K·t²), r·t + K·t²]` as today. A
  lifted vertex lies in `(0, h0 + max(0, h'(0))·t + K·t²]`: it never crosses the plane inside the track (a
  rested one may, within §10.8's two-sided bound), and
  its height rises by at most its start rate when that rate is positive. `Band()` and `BandAt` publish the same
  bound; a track with zero depth through the duration is still an exact `SweepPersistentTouch`, which a lifted
  vertex rules out. `SweepPersistentBand`'s claim covers every published point.
- `ManifoldAt(f)` stages the whole support set through the rounded pose, contact vertices first, each with its
  exact foot on `S`'s rounded plane and `Separation = [0 ± Depth]`. `replayHeights` and §10.1's replay rules
  are unchanged: they read every `M` vertex against the held depth.

**Dynamics.**

- A band end at an arrival is §5 step 4's band end: the track's end lies within the residual, so the slice cuts
  there. The arriving vertex is in general a CLEAR vertex of the track, not a lifted one: it stood far above
  the band at the slice's start and enters it only in the track's last grid step, so `ManifoldAt(end)` does
  not hold it. Under a positive `SupportBand` the island therefore solves on the manifold `roundedDepth` reads
  at the rounded event poses, the poses the step publishes: `ContactPair`'s support set there, which holds the
  arriving vertex beside the track's own set (`dynamics/schedule_event.go`). For the cube that island is the
  two edges' four corners with the near edge closing. The solve returns a positive impulse on each edge and
  zero linear and angular velocity afterwards, which §6.2 publishes as exact zeros. No rotation remains to
  lift an edge, and the rounded event poses show no penetration: `roundedDepth` reads `Separated` within the
  band, hence `ContactBand` with the support-set manifold, and the correction moves nothing.
- A resting pair continues from a `ContactBand` start. When its swept boxes stay strictly apart, as the
  cube's do while it hovers inside the band with zero velocity, §4.3's broad phase excludes it and no sweep
  runs. Otherwise its support set carries the next slice's band track with `r = 0` and `K = 0`, so `Depth` is
  the largest lifted height, within the band and the residual, and the track reaches the duration. The next
  step's kick brings the pair in as an initial contact whose `InitialEvent` is the `ContactBand` with four
  points; the solve absorbs the kick on the whole face with zero rotation, as the pyramid's faces do.
- §6.6: an island pair the solve leaves resting may end `Touching` or in a `ContactBand` within the band, as
  §10.4 already admits. §6.6's settle search places a resting pair whose corrected poses still overlap back in
  touch or in the band; under a positive `SupportBand` that band includes an exact pair apart by at most it.
- §5 step 5 and the contact-set rules read a `ContactBand` as a touch within the residual, as §10.4 states.

The numbers of `dynamics/tip_test.go`'s run, stated as a record of that run rather than a bound: the cube
lands its far edge in its eighth step, `1.28 ms` into it, about `3.8e-7 mm` above the floor, inside the
`5e-7 mm` band, with its near edge `3.8e-4 mm` up. The two-point impact turns it back at `6.5 rad/s`, and the
far edge's band track ends `7.4 µs` later where the near edge, arriving at about `52 mm/s`, would reach the
floor within a grid step: the remaining `2.6 ms` slice at `TimeResolution = 1e-9 s` has a `2⁻²²` grid of about
`6.3e-10 s`, so the near edge is caught `3.2e-8 mm` above the floor, while the far edge has risen to
`3.83e-7 mm`. The four-point solve delivers `5.57 kg·mm/s` at each near corner and `1.11 kg·mm/s` at each far
one and leaves both velocities exactly zero; every later step absorbs its `19.62 kg·mm/s` kick on the four
corners. A vertex that arrives
faster than the band per grid step enters no support set in time and leaves the pair `StepPairUndecided` as
today; the band and the time resolution are the caller's.

**Rest of a tumbling body.** A `tumble` body comes to rest through successive band ends, each adding a feature
to its support set. The hexagonal prism released on a vertex rides that vertex's band track until a
neighbouring vertex arrives (a band end whose two-point island leaves rotation about the two vertices' line),
then the edge's track until a third vertex arrives (a three-point island, which stops the body when its mass
center lies over the triangle and otherwise leaves it tipping about one of the triangle's edges). The remaining
vertices of that face are coplanar with the three in the body's frame, so they lie within a few bands of the
plane and enter the support set at the next slice. The wedge and the tetrahedron take the same sequence; the
boxes rest on four points. Every such rest is a `ContactBand` rest: no correction lands a feature in exact
touch under a rotation about `(1, 1, 0)`, and no float pose makes a side face of the hexagonal prism, the
wedge's slanted face or any face of the tetrahedron coplanar with the floor. A bounce under the scene's
restitution `0.3` separates the pair at its solve; §6.6 pushes it, and when the push leaves the exact pair
inside the band it continues on a band track under `ContinueCertifiedTouch` (§10.7), which ends where
the band leaves the residual or where the next vertex arrives. A body touching the tray floor and a wall
is a touch no single support plane covers, which §10.3
leaves `SweepUndecided`; PR 15's releases place every body so that none reaches a wall within its `3 s`, and a
scene that needs a wall rest needs a multi-plane track, which this design does not include. The floor of
the tray itself is a face-local support plane (§10.6).

**Rejected alternatives.**

- Snap to exact flatness by a rotation correction. A face whose unit normal is irrational in its body's frame
  has no float pose that makes it coplanar with the floor, so the hexagonal prism's sides, the wedge's slant and
  the tetrahedron cannot be snapped; and a translation lands a feature in exact touch only when the feature's
  exact height is itself a float.
- A sweep outcome that brackets a second feature's impact while a band carries the first. It lands the second
  edge, but the solve at the bracket's right end sees the arriving edge's two points alone whenever the rounded
  poses show the pivot edge a little above the plane, so the cube reverses onto the pivot edge within a grid step
  and the step stops as today; and even with both edges solved, each later step's kick on one exact edge turns
  into rotation, so the cube rocks with two events per step and never rests on its face. The support set makes
  the arrival a band end and the rest a face solve with no new outcome.
- A `dynamics`-side rule that treats a face with tilt below a bound as a face patch. `dynamics` builds no
  manifold and holds no exact vertices; the heights are exact only in the planar kernel, and a manifold the
  producer did not certify carries no `ContactFeature` or ball.
- Admitting lifted vertices within `PointResolution` instead of a new field. `PointResolution` bounds the ball
  of a witness on the true boundary; a lifted vertex is no contact point, and admitting it is the slop decision
  of the dynamics, stated by its caller beside `PenetrationResidual`.
- Letting an arriving lifted vertex penetrate to the residual before the band ends, as a contact vertex may.
  The correction then lifts the pivot edge to the band's rim, the next step kicks the cube on one edge again, and
  the rest alternates between the edges at the residual's scale.

### 10.6 Face-local support planes and the column clearance

§10.2's support plane requires every vertex of its owner `S` on or behind it, which no face of a `Cut`
tray satisfies: its walls rise above its floor and its rim faces lie above its walls' feet. The
scene's bodies all rest on, depart from and tip over the tray's floor, so `planarSupportOf`
(`contact_sweep_band.go`) admits a FACE-LOCAL support plane instead: the plane of a flat face `h` of
`S` (one `Face`, every triangle coplanar with one outward normal `n`, as `planarSupportFace` already
requires) whose owner only translates, so the column test reads `S`'s start triangles less that
translation, such that

- every vertex of `M` lies on or in front of the plane, the contact and lifted sets being §10.2's and
  §10.5's as today;
- the COLUMN TEST holds over the horizon: with `B(f)` the coordinate box of every `M` vertex's ideal path
  over `[0, f]` (`cornerSpan`), less `S`'s own translation, and `P` the projection along `n` onto the
  plane's frame (the axis of `n`'s largest component dropped, §9.4's frame), every triangle of `S` with
  a vertex strictly in front of the plane has `P(triangle)` strictly apart from `P(B(f))`, decided by
  the separating axes of the projected box and the triangle's three edge normals in exact rational
  arithmetic (`planarSegmentMeetsBox`'s form);
- the LATERAL CLEARANCE `m(f)` is the least, over those triangles, of the largest separating-axis gap
  between `P(triangle)` and `P(B(f))`, each gap divided by an upper bound on its axis's length
  (`ratSqrtUp`), so that `m(f)` is a lower bound on the in-plane distance, and the projection shortens
  no distance, on the true one; a convex `S`, every vertex of which lies on or behind the plane, has
  no such triangle and `m = +∞`.

`B(f)` grows with `f`, so the column test and `m(f)` are monotone and the grid search of §10.2 and
§10.3 reads them as it reads `clearAt` and `contains`: the departure's horizon and the band track's end
are the largest grid fractions through which every test holds. Soundness: every point of `M` lies in
its vertices' hull, hence in the column over `P(B(f))`; material of `S` behind the plane is at least
the least vertex height away along `n`; material of `S` in front of the plane projects at least `m(f)`
away, hence is at least `m(f)` away. The pair's separation on `(0, f]` is therefore at least
`min(least vertex height bound / |n|_hi, m(f))`, and that minimum is what `planarDepartureProof.lowerGap`
publishes to the replay (§10.1); `departureGap` keeps reading `ContactPair`'s exact gap at the
departure's end sample. A band track's overlap claim (§10.3) needs
nothing about what lies behind the plane: material of `S` in front of the plane meets no part of `M`,
so every point of `M ∩ S` lies behind the plane, and every point of `M` lies at height at least
`−Depth`, so the overlap along `n` is at most `Depth` whatever `S` holds behind the face, a slab thinner
than `Depth` included. The column test is shared with §9.6's condition 4 (`internal/pair`,
`PlanarColumnClear`), which calls it with `f = 0` and no margin.

Limits, stated rather than solved: a body that overhangs a wall's top rim while resting on the floor
fails the column test, because the rim's triangles lie in front of the floor's plane and project under
the body; a box in a tray corner touches two faces and has no single plane (§10.2). Both stay
`SweepUndecided`. PR 15 keeps every release at least `20 mm` inside the walls.

The placed ruling of contact-geometry §4.5 (`rulingSupport`, `contact_analytic_manifold.go`) reads a
face-local plane the same way: a signed-axis face plane of `S` with an `S` vertex strictly in front is
admitted when the column test holds at `f = 0` over the coordinate box of the eight staged corners of the
cylinder's identity box, whose hull holds the cylinder, and it records that box's lateral clearance `m`. A
separated ruling publishes its gap with lower end `min(σ_lo, m)` and upper end `σ_hi` unchanged: the
least height bounds the true gap from above, because the ruling's feet lie inside the face (so `S` has
material under the cylinder at that height), while the clearance only bounds it from below (`m` is a
lower bound on the in-plane distance to the material in front, not that distance). A touch or band on a
face-local plane is published only when `m` exceeds the band's half-width, so the ruling is the pair's
only contact within the band; a cylinder nearer than that to a wall is `Undecided`, since it has no ruling
against the wall. The rolling track (§10.4) runs the column test at every grid fraction of its band
search, as the planar band track does, over the corners' path box over `[0, f]` intersected with the
hull of the end centers' path boxes grown by `r·√(1 + gram)`, less `S`'s translation. Both boxes hold
the cylinder at every instant: every cylinder point lies within `r·√(1 + gram)` of the axis point in its
section, a convex combination of the end centers. The corners' interval enclosure loosens past a
quarter turn, while the end centers of a cylinder turning about its own axis only translate, so the
second box carries a whole turn. The track publishes no lower gap, so `m(f)` is not read there.

### 10.7 Continuation inside the band

Three ways a `tumble` body stops on a plain convex floor share one cause, which the policy table of §5.2
closes. Each is a record of the exploration run in `dynamics/tumble_explore_test.go` (§13 PR 15's
fixture) with the §2 material (`e = 0.3`, `μ = 0.4`), `dt = 1/256 s`, `TimeResolution = 1 ns`,
`PenetrationResidual = 1 nm` and `SupportBand = 0.5 nm`:

- The spinning `20 mm` box lands on an edge, the solve leaves both edge corners rising at about
  `3·10⁻³ mm/s` (the edge is the instantaneous axis of the tip that follows), and the pair rides a band
  track that ends `2.8 µs` later where the corners' quadratic lower height bound reaches zero. Nothing
  closes at that band end, so §6.1 enters no solve and the pair takes `ContinueSeparatingTouch`. The next
  sweep departs for `6.7 µs` and then cannot certify the pair clear: inside the band its samples read
  `ContactBand`, whose published `Gap` is `[0 ± g]` with no lower end, and §4.3's travel certificate needs
  `g_l + g_r` above the travel per grid step, `270 mm/s × 1 ns ≈ 2.7·10⁻⁷ mm` for this box against
  corners `10⁻⁸` to `6·10⁻⁸ mm` up. The search bisects to the time floor, `SweepTimeFloor`, and the
  oriented-box report that the planar rerun fell back from says `SweepDepartureUnproved`.
- The hexagonal prism bounces off three vertices with `+21`, `+5.5` and `+5.5 mm/s` while its pivot vertex,
  `6·10⁻⁷ mm` up, descends at `41 mm/s`. The solve separates the pair, the push leaves it a `ContactBand`,
  and under `ContinueSeparatingTouch` the departure ends `14.6 ns` later at the pivot's arrival with the pair
  still inside the band. An impact bracket needs a `Separated` left end, so the bracket of the pivot's
  touch, `[ContactBand, Overlapping]`, is `SweepTimeFloor`.
- The wedge bounces off one vertex at `39 mm/s` while another, `1.2·10⁻³ mm` up, descends at `72 mm/s`;
  the same bracket shape stops it `17 µs` later.

The cause is the policy, not the certificates: a pair that stands inside the `SupportBand` after an event
is a contact for the step (§10.5 admits the band as a touch within the residual), and the band track over
the support set is the certificate that carries such a pair whatever its vertices do — a lifted vertex
may rise, since `Depth` grows with `max(0, h'(0))·t`, and a clear vertex may arrive, since the track
ends before it reaches the plane — while `ContinueSeparatingTouch` asks for a departure and a clear
search that no certificate can finish while the pair is apart by less than the travel per grid step.
`ContinueSeparatingTouch` is therefore assigned only to a pair the solve separates whose post-event poses
read exactly `Touching`, which §6.6's push can produce for curved pairs and for exact planar touches,
and whose §10.2 departure then has a positive gap to open. Every pair an event leaves in a `ContactBand`
within the residual continues under `ContinueCertifiedTouch`, and a pair gathered at a band end or from a
track that enters no solve keeps `ContinueCertifiedTouch` while its rounded event poses read `Touching` or
such a band, and leaves the contact set when they read `Separated`; its next slice then starts
`Separated` by more than the band, which exceeds the travel per grid step whenever
`SupportBand >= (|v| + ρ·|ω|)·TimeResolution` for the pair's faster body, a relation the scene's
`0.5 nm` band and `1 ns` resolution satisfy up to about `500 mm/s`. A pair that leaves the band slower
than that stops at the time floor, as a pair starting apart by less than its travel per grid step does
today; the band and the resolution are the caller's.

Under this rule each run above passes the stop recorded for it. The prism's band track ends at the
arriving vertex, one grid step before it reaches the floor, where it stands inside the band and the
support set read at the rounded event poses holds it beside the bounced vertex, so the island solves
three vertices of its hexagonal cap together; every later step absorbs its kick on all six, and the
prism rests with exactly zero velocities through `1 s`. A zero `SupportBand` never publishes a
`ContactBand` for an exact pair, so every shipped zero-band fixture keeps its policy and its numbers; a
§10.4 band pair that a solve separates, which `ContinueSeparatingTouch` could never depart (§10.4),
continues on its band track as well. Dropping a `Separated` pair from the contact set changes no
certified outcome of an exact planar pair: a planar sweep that starts `Separated` runs the same clear
search under every start policy, so the drop only keeps the §5.3 reuse key and the contact set honest.

The box and the wedge rest is a limit of the band track that §10.8 closes. Each reaches a state in which the solve
leaves it resting on one or two lifted vertices, so the pair reads `ContactBand` and continues on a band
track that ends where a lifted vertex's lower height bound `h0 − K·t²` reaches zero. Under the body's
rotation the true descent of that vertex is only `7`–`14 %` of what `K` allows, so at each band end the
vertex stands at about `0.86`–`0.93` of its previous height; the solve removes a closing speed of a few
`µm/s` or less, and the vertex arrives again sooner. The band ends shrink geometrically and never reach
the step's end, and the step stops with `StepEventBudget` (MaxEvents `64`): the `30°` box in its step
`39`, the wedge in its step `29`, the `45°` and `60°` boxes in steps `80` and `48`, while the `75°` box runs
256 steps still sliding at about `5 mm/s` with seven events per step. No `dynamics` rule alone closes the
cycle, since no correction lands a rotated vertex in exact touch and a sweep that starts `Overlapping` is
`SweepInitiallyOverlapping`; §10.8 changes the band track itself (`contact_sweep_band.go`) and adds the two
step rules the changed track needs.

Rejected alternatives:

- Letting an exact pair's `ContactBand` sample carry the exact relation's lower gap, so the clear search
  and the impact bracket could read a band sample as a separated one. It closes the prism and the wedge,
  whose bracket then reads `[ContactBand, Overlapping]`, but not the box: no lower gap below the travel
  per grid step certifies an interval clear, so the slow lift out of the band still ends at the time
  floor. It also changes the first impact of every falling body from the band's entry to the exact
  touch, and with it the landing fixtures of §13 PR 14a.
- Chaining §10.2's departure proof inside one sweep, restarting the quadratic bound at each horizon from
  the enclosed ideal heights and rates there. It certifies the box's lift, but it is the band-end chain
  the step already runs, done inside the sweep with interval starts instead of exact ones, and it leaves
  the prism and the wedge to the bracket change above.
- A band track returned under `ContinueSeparatingTouch` when the departure's search fails. It moves a
  policy decision into the sweep, which would then publish a track under a policy whose contract
  (`docs/contact-sweep-design.md` §5.1) asks for a departure.

### 10.8 Rest inside the band

§10.7 records the band-end chain that stops the boxes and the wedge. Two bounds of the band track make the
chain, and one law of the step limits what a rest can be; this section closes the chain and states the
limit, each from the exploration runs of `dynamics/tumble_explore_test.go` (`TUMBLE_NOTRAY`, a plain `240 mm`
floor, 256 steps) with the §2 material and `dt`.

**The chain.** After a vertex impact the solve leaves the body on one or two LIFTED vertices (§10.5): inside
the band, above the plane, their normal speeds within `VelocityResidual` of zero, the body still turning. The
band track holds a lifted vertex one-sided and ends where its lower bound `h0 + h'(0)·t − K·t²` reaches zero
(§10.5). `K` is §10.2's global bound `½·|n|·|ω|²·ρ`, `ρ` the farthest vertex from the pivot. The true second
derivative of a height is `n·R(ωu)·(ω×(ω×(p0 − c)))`, whose magnitude `|ω|·|ω×(p0 − c)|` is constant in `u`
and is zero for a vertex on the spin axis through the mass center. The `60°` box spinning on a corner at
`|ω| ≈ 5.5 rad/s` about a nearly vertical axis therefore descends by about `3 %` of what `K·t²` allows, and
the `30°` box by `7`–`14 %`. At each band end the vertex stands at a fraction of its previous height, the
solve removes the closing speed the true curvature built (impulses of `10⁻³` to `5·10⁻⁶ kg·mm/s`), and the
next track ends at the smaller root. No correction lands a rotated vertex in exact touch (§10.5), so the chain has no last
event. Its two causes are the one-sided hold and the global `K`.

**Rested vertices.** `SweepRequest` gains one field:

```go
RestSpeed units.Value // nonnegative Velocity; the zero Value rests no vertex
```

A lifted vertex is RESTED when `RestSpeed` is positive and its exact start rate satisfies
`h'(0) >= −RestSpeed·|n|_lo`: it closes no faster than `RestSpeed`, or rises. The band track holds a rested
vertex as it holds a contact vertex, on both sides of the plane: `|h(t)| <= h0 + |h'(0)|·t + K_p·t²` by
Taylor's theorem with the per-vertex curvature `K_p` below, and `Depth` is the largest of that bound over the
rested set, `r·t + K_p·t²` over the contact set and §10.5's one-sided bound over the remaining lifted set. The
horizon test (`clearAt`) skips a rested vertex, so the track no longer ends where its lower bound reaches
zero; it ends where `Depth` leaves the residual (§5 step 4), where a clear or unrested lifted vertex would
reach the plane, or where a foot leaves the face. §10.2's departure is unchanged and keeps every vertex
outside the contact set positive, rested or not: a departure claims strict separation. `ManifoldAt` publishes
a rested vertex as before, with `Separation = [0 ± Depth]`, and `replayHeights` reads the held depth as
before. A `RestSpeed` that is not a `Velocity`, is negative or is not finite is an input error at
`SweepPair`, as a bad `SupportBand` is (`ErrUnitKind`, `ErrDegenerate`, `ErrNotFinite`); the field is part of the request, so §5.3's reuse key carries it. `dynamics` fills `RestSpeed`
with `VelocityResidual` in `sweepRequest` (`dynamics/step.go`): §6.3 leaves a resting point's normal speed
within that residual of zero, so a vertex the solve just rested is rested on the next slice, a vertex that
arrives faster is not, and an arriving lifted vertex still ends its track one grid step before the plane
(§10.5's last rejected alternative stays rejected). A rested vertex that rises out of the band leaves the
support set at the next band end or event, where `ContactPair` reads the rounded poses, as a lifted one
does today.

**Per-vertex curvature.** For every `M` vertex `p` the `p''` term of §10.2's bound, `|ω_M|²·ρ_M`, is replaced
by `|ω_M|·|ω_M×(p − c_M)|`, each factor rounded up by `ratSqrtUp`; the terms of a rotating owner `S` stay
global. `|p''(u)| = |R(ωu)·(ω×(ω×(p0 − c)))| = |ω|·|ω×(p0 − c)|` for every `u`, so
`K_p = ½·|n|_hi·(|ω_M|·|ω_M×(p − c_M)| + the S terms)` bounds `|h_p''|` on the whole horizon as `K` did, and
`K_p <= K` because `|ω×(p − c)| <= |ω|·ρ`. `clearAt`, `depthAt`, the departure's horizon and
`planarDepartureProof.lowerGap` read `K_p` in place of `K`; the replay still reads the held depth. A body
whose contact vertices lie on its spin axis with `r = 0` now publishes an exact `SweepPersistentTouch` where
the global bound published a band of depth `K·h²`: the edge box of `apitest/contact_sweep_band_test.go`, turning
about its resting edge, is one. The `60°` box, which the global bound ends every `46`–`62 µs` under the rested
hold (`StepEventBudget` in its step `48`), spins on its corner through 256 steps under `K_p`.

**An overlapping start and a cut track.** A rested vertex may stand below the plane, within `Depth`, when a
step ends or when another pair's event cuts the slice, and a sweep from those poses starts `Overlapping`,
which neither policy continues. Two rules of §5 step 7 cover both: an initial contact whose `InitialEvent`
reads `Overlapping` with no manifold within the request solves on the manifold `ContactPair` publishes at the
slice-start poses, which are those poses themselves (fraction zero has zero deviation) and whose penetration
the correction removes; the rotating source-box path transfers no manifold for an overlapping start
(`orientedIdealEvent`), while the planar path transfers it whole, and the rule reads the same poses either
way. And a contact-set pair on a band track with positive `Band()` covering `f_e` whose rounded poses at `f_e`
read `Overlapping` is gathered as a band end there, not as a track pair, which §6.1 would leave to drift: it
enters the solve whether or not a point closes, its island corrects the penetration (§6.6), and when nothing
closes the impulses are zero and the island publishes only the correction. Without the first rule the `60°`
box at the shipped residuals stops in step `49`, `StepManifoldMissing`, "initial contact has no manifold
within the contact request"; without the second the wedge, resting beside the prism, stops in step `29`,
`701.7 µs` in, `StepTrackUnproved`, "a pair continued in contact starts SweepInitiallyOverlapping", at the
prism's impact.

**What rests, and at which band.** Under the three changes, at the shipped residuals (`PenetrationResidual =
1 nm`, `SupportBand = 0.5 nm`) the prism (six points, from step `32`) and the wedge (its triangular cap, three
points, from step `30`) rest with exactly zero velocities through 256 steps in one world. Every box runs 256
steps `Advanced` without resting: each settles into a step-periodic rocking cycle of five or six events per
step (vertex impacts, then two edge impacts) and ends every step with its lowest corner `2.3 µm` up, its
mass center rising at `1.26 mm/s` and yawing at `0.49 rad/s`. The cycle's mechanism, read at a `1 µm`
residual with a `0.5 µm` band, where it has two events per step: the kick lands the `30°` box on one edge
inside the band (two points, `J = 161 kg·mm/s`, friction at the Coulomb limit `0.4·J`, the box turning at
`1.73 rad/s` about the edge and sliding at `7.4 mm/s`); the far edge arrives `131 µs` later, and the
four-point solve puts `72.6 kg·mm/s` on each far corner and nothing on the near ones, because the far
corners' friction impulses (`27 kg·mm/s` each) stop the slide and their torque about the mass center lifts
the near edge at `1.2 mm/s`, the mass center at `0.61 mm/s`. §5's step has one kick, so the box drifts up for
the remaining `3.77 ms` and ends the step with its near edge `4.5 µm` up; the next kick lands it on the far
edge alone, and the end state repeats to the published digits: the kick's `306 kg·mm/s` is what the two
solves absorb. Under continuous gravity the lifted edge would return in `0.24 ms` and its re-landing's
four-point solve would zero both velocities, as the tip fixture's does (§10.5); the one-kick law does not
model that return, and no correction may stand in for it (§6.6 moves a body only to remove penetration or to
place a resting pair in touch).

A box therefore rests only when a step's kick lands on all four corners, that is when the band covers the
hover. The `tumble` scene runs at `PenetrationResidual = 10 µm` and `SupportBand = 5 µm` (§2), and there
every body rests with exactly zero velocities (the wedge within `10⁻⁷ mm/s`): the boxes from steps `45`
(`30°`), `77` (`45°`), `54` (`60°`) and `72` (`75°`), the prism from `32`, the wedge from `30`, the
tetrahedron from `29`, each later step one face event, the `30°` box's four lower corners ending between
`0.002` and `3.8 µm` above the floor. The first impact of a falling box is then the band's entry, `SupportBand`
before the exact touch; §2 states its criterion against that entry. At this band the rested hold and `K_p`
change no outcome of the box and wedge runs (each rests without them); they stay for the chain, which the
wedge at the shipped residuals shows and §13 PR 14f pins. The `1 nm` residual of §13 PR 14e's prism fixture
and of every earlier fixture is unchanged.

A rested vertex may also stand below the plane when a step ends, within the residual §5 step 5 admits,
and the next step's kick takes the pair out of the contact set, so it starts that step as an initial
contact whose penetration exceeds `ContactSlop`. Its correction allowance (§6.6) is therefore widened by
`PenetrationResidual` when the pair ended the previous step in the contact set and both bodies still stand
at the poses that step published (`carriedPenetration`, `dynamics/schedule_event.go`): the penetration is
the one step 5 admitted there. Any other initial contact keeps the plain allowance. Without it the tray
scene below stops in step `42`, `StepCorrectionFailed`, the `30°` box `1.25 nm` into the tray's floor
against a `1 nm` allowance.

With the tray (the §2 scene itself, §9.6's face-local shallow penetration included) the `10 µm` run of all
eight bodies goes 256 steps `Advanced`, every body resting on one face from its step on, seven events a step.

Rejected alternatives:

- Ending the chain with an inelastic-collapse rule on the response, analogous to `ImpactSpeed`: a closing
  speed below a threshold absorbed without an event. The chain is in the certificate, not the response: the
  track still ends at the bound's root, the slice still cuts there, and a chain of slices without events
  reaches `MaxPairSweeps` instead of `MaxEvents`.
- Holding every lifted vertex two-sided. An arriving lifted vertex then penetrates to the residual before
  the band ends, and §10.5's last rejected alternative follows: the correction lifts the pivot out of the
  band and the rest alternates between the edges. `RestSpeed = VelocityResidual` tells the vertex the solve
  rested from the one that arrives.
- Admitting a vertex below the plane into the support set, so a sweep could continue from an `Overlapping`
  start. It changes the contract of both continuation policies (`docs/contact-sweep-design.md` §5.1) for a
  case two rules of the step cover at the poses the step publishes.
- Transferring the overlapping manifold on the rotating source-box path at zero deviation instead of
  reading the rounded poses in `dynamics`. Sound, and it may land later; the step's rule covers every
  family, including the planar path, through one site that already reads rounded poses for impacts and band
  ends.
- A shorter `dt` for the hover. At `dt = 1/1024 s` the lifted edge ends its step about `1.1 µm` up, still
  thousands of `0.5 nm` bands; the hover is the one-kick law's, and the band is the caller's.

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
}

func (t timelineTrack) At(d time.Duration) (r3.Transform, error)
```

`At` converts the frame time to `units.Seconds(float64(d) / 1e9)` — this rounds the TIME LABEL by at
most one ulp of a second; the pose returned is the certified pose at the rounded time, which is what the
frame shows, so no geometric claim moves — then calls `Timeline.Sample` and returns the sampled
`BodyState.Pose` as it is: a dynamics pose maps the body as modeled, its placement included, to world, and
that is the frame of the part kinetograph tessellates, so a driven node directly under the root needs no
other transform. A sample beyond `Timeline.End()` (§7.2), or one the
replay refuses, returns the error; kinetograph fails the frame, and the gallery command fails with the
`StepDiagnostic` of the stopped step printed. The gallery never freezes the last certified pose and never
extrapolates. kinetograph's half-open frame times keep every frame strictly before the clip length, so a
timeline advanced to the clip length is never sampled at `End()` itself. `timelineTrack` holds no
mutable state and `Timeline.Sample` is safe for concurrent calls (§7.2), so `At` meets §11.1's contract
under `Sequence`'s workers.

`_gallery/dynamics_clip.go` holds the scene builders (`stackAndDropScene`, `tumbleScene`,
`partsBinScene`), each returning the document, the `WorldConfig`, the world and its start `State`, the
per-step `StepInput`, `dt`, the clip length, the parts' colors and a still camera that frames the scene.
`main.go` gains `go run . dynamics -scene <name> [-out -fps -width -height -workers -smoke]`, which advances
the timeline to the clip length, builds one kinetograph scene with a `Driven` node per body, the scene's
camera and the directional lights of `scene.go`, and renders through `render.New` as `clip.go` does. When the
timeline stops early, the frames before its certified end render and the command fails at the first frame
past it, naming the stopped step's diagnostics. `-smoke` advances the whole timeline and renders the first
frame alone at `160×90`. The gallery module stays the only module that imports kinetograph.

### 11.3 Scene tests

`_gallery/dynamics_clip_test.go` runs each exit scene's timeline, built through the `dynamics` subcommand's
scene map, and asserts the §2 exit criteria on the trace: certified end time, final poses, event kinds and
times, conservation. It films each timeline and renders its first frame at the `-smoke` size. The scenes'
tests run in parallel, so CI's gallery job waits on the slowest scene alone, and the job smoke-renders only
stack-and-drop through the subcommand, whose path is the same for every scene.
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

The scheduled step fills every field: `From` and `To` bound the slice a
pair diagnostic covers (an undecided sweep's unresolved interval, mapped onto the step clock), or name the
event time of a refusal raised there; `StepEventBudget` runs from that event to the end of the step.
`Limit` holds the refused gate's limit for `StepIslandResidual` (in the gate's own kind), the correction
allowance for `StepCorrectionFailed`, `PenetrationResidual` for `StepTrackUnproved`, `MaxEvents` as a
dimensionless scalar for `StepEventBudget`, and `MaxPairSweeps` likewise for `StepPairBudget`, whose
interval runs from the last certified time to the end of the step.

## 13. Delivery order

Each PR is one branch, one review, one merge, and carries the test that proves it. "Root tests" means
`.github/test-shards.txt` must list every new root-package test name and `.github/test-shards-apitest.txt` every new `apitest`
test name; `dynamics` tests are not sharded.
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
- Test (`apitest`): `apitest/swept_box_test.go`: a rotating box's swept box contains its exact corners at fractions
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
  closed-form responders.
- Files: `dynamics/island_solve.go`, `dynamics/island_certify.go`; the deleted files.
- Test: every existing `dynamics` response test passes through the general solver with its original
  assertions, `friction_patch_test.go`'s slide and the stack fixture included, except for the rewrites
  §6.5 lists.
- Depends on: PR 4.
- The cone, stick and slip rows ship, with
  `dynamics/island_friction_test.go`: the four-corner slide of `friction_step_test.go` in both body
  orders, a box slipping across a dynamic box the floor holds by sticking, and the three rows' tamper
  fixtures. So do §6.2's direct start and lever-bounded spin snap, §6.1's silent zero-speed island,
  §5's certificate-replayed slice poses and `MaxEvents` rule, and §6.3's `AngularUpper`, with
  `dynamics/island_direct.go` and `dynamics/island_direct_test.go`: a two-box stack in eight sweeps, a
  frictional face impact that friction stops exactly, and a sphere rolling in a corner. §6.6's
  separating push ships with `dynamics/island_push_test.go`: a sphere bouncing off a tilted face, its
  allowance, and a resting sphere pushed apart by its margin; the resting search, margin and anchored push
  with `dynamics/island_rest_test.go`: a disk an ulp into a floor placed back in touch, a tilted face with
  no touching pose, and the sphere column resting on itself off center; its sub-ulp-gap push and the
  anchored correction
  with `dynamics/island_sphere_landing_test.go`: the stack-and-drop sphere column landing on itself, a
  glancing landing whose spinning spheres meet again, and a gap no float can prove. Every world routes
  through the general step, with §6.5's assertion rewrites and §6.1's rotating drivers in islands
  (`TestKinematicHingedPaddleStrikesWithItsField`, whose legs are each shown to fail). The closed-form
  responders (`three_body*.go`, `friction_patch.go`, `friction_pair*.go`, `friction_step.go`,
  `friction_impact.go`, the `sphere_floor_friction.go` and `sphere_pair_*.go` responders, `oblique_*.go`,
  `fixed_offcenter.go`, `cylinder_impact.go`), the two-body steps of `step.go`, `resting.go`,
  `grazing_step.go`, `kinematic.go` and `kinematic_impact.go` with their `Trace` slots, and `NewWorld`'s two-
  and three-body role-mix refusals are deleted, and `.golangci.yml` holds no `unused` exclusion.
- Shipped.

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
- Test (`apitest`): `apitest/mass_properties_rotated_test.go`: a box rotated `30°` about `(1,1,1)` encloses
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
- Shipped, against kinetograph's `Driven` node (its decision D12). The scene's steps are `1/256 s` and its
  `ImpactSpeed` `64 mm/s`, as the pyramid fixtures use (§2). Both scene tests replay all 512 steps; the
  gallery's also checks every frame's poses against `Timeline.Sample`, evaluates the clip from concurrent
  goroutines, renders the first frame, and fails a frame past a shortened timeline's certified end.

### PR 10 (Phase 2) — exact planar pair relation and convexity

- Delivers §9.1 and §9.2.
- Files: new `internal/pair/planar.go` and `contact_faceted_pair.go`; `contact_pair.go`.
- Test (`apitest`): `apitest/contact_faceted_pair_test.go`, with the snapshot-level cases in
  `internal/pair/planar_test.go`: a hexagonal prism at a `37°` pose against a tray reads a
  `3 mm` gap enclosed, a vertex touch and a shallow crossing; a small box nested in a hollow Boolean's
  wall is `Overlapping` and one in its cavity is `Separated`; the non-convex Booleans report
  `ContactNonConvex`.
- Depends on: nothing.

### PR 11 (Phase 2) — faceted manifolds and the exact-clipped patch

- Delivers §9.3, §9.4 and the shallow-penetration patch.
- Files: new `internal/pair/planar_patch.go`, `internal/pair/planar_manifold.go`,
  `contact_faceted_manifold.go`, `contact_faceted_patch.go`; `contact_faceted_pair.go`.
- Test (`apitest`): `apitest/contact_faceted_manifold_test.go`: a rotated box on a face publishes one point at a
  vertex touch, two at an edge touch, and the clipped hexagon's seven extremal vertices at a face
  touch, each at its exact coordinate; §9.4's parity, wall, reversed-loop, hole and
  `ContactPointTooCoarse` fixtures, with the wall and reversed-loop legs shown to fail.
- Depends on: PR 10.

### PR 12 (Phase 2) — general rotating sweep

- Delivers §10.1.
- Files: new `contact_sweep_faceted.go`; `contact_sweep.go`, `contact_sweep_rotation.go`.
- Test (`apitest`): `apitest/contact_sweep_faceted_test.go`: a wedge tumbling toward a floor brackets its first
  vertex impact at the exact drift time within `TimeResolution`; the deviation leg is shown to fail.
- Depends on: PRs 10, 11.
- Shipped, with the deep-vertex overlap witness in new `internal/pair/planar_depth.go`.

### PR 13 (Phase 2) — generalized departure and band tracks

- Delivers §10.2, `SweepPersistentBand`, `Band()` and `BandAt()` (§10.3); `dynamics` consumes band
  tracks and advances through a rotating pair's impact (§5 steps 4, 6 and 7).
- Files: `contact_sweep_faceted.go`, `contact_sweep.go`, new `contact_sweep_band.go`,
  `contact_sweep_replay.go`, `contact_sweep_rotation.go`, `contact_pair.go`,
  `internal/pair/planar_manifold.go`, `dynamics/schedule.go`, new `dynamics/schedule_band.go`,
  `dynamics/schedule_event.go`, `dynamics/island.go`.
- Test (`apitest`): `apitest/contact_sweep_band_test.go`: a box resting on an edge with `ω = (0,1,0) rad/s`
  publishes a band track whose `Depth` equals `K·h²` for the computed `K`, and `BandAt` the same closed
  form over a prefix; `contact_sweep_band_internal_test.go` shows the rotating bracket replay's travel
  and deviation charges; `apitest/contact_sweep_rotation_test.go` publishes a turned box's edge poking through a
  face, and `internal/pair/planar_patch_test.go` a wedge's, clipped to the face. `dynamics/tip_test.go`: a
  cube on its edge, turned `30°` with its center of mass beyond the edge, tips over in a few steps; each
  step lands the edge as a rotating impact, rides a band track to the band end, and lifts clear; every
  impulse meets the discrete linear and angular laws, every band end is the last grid fraction within
  `PenetrationResidual`, and the trace replays every rotating bracket. The cube lands on its far edge.
- Depends on: PR 12.
- Shipped, without the flat rest. A cube landing on its far edge is turned by a float rotation that is
  never exactly flat, so the exact relation sees one edge in touch and the other a little above, and with
  restitution zero the cube rocks between its two bottom edges with a shrinking tilt. Once the far edge
  closes within one grid step of the near edge's touch, no certificate covers the pair: the band track
  ends before the first grid fraction, and a sweep that starts in touch cannot bracket another vertex's
  impact. `dynamics/tip_test.go` asserts that stop, `StepPairUndecided` in the landing step, with its
  certified prefix through the landing; under a positive `SupportBand` the cube rests flat (§10.5, PR 14a).
  Tip times also differ from the design's `0.5 s`: the one-kick step with the edge's support solved at the
  start of each step tips a near-balanced cube within about `16` steps of `1/256 s`.

### PR 14 (Phase 2) — loft, stitched and fallback mass

- Delivers §8.3, §8.4 and §8.5.
- Files: `mass_properties.go`, `mass_properties_faceted.go`, new `mass_properties_mesh.go`.
- Test (`apitest`): `apitest/mass_properties_mesh_test.go`: the stitched tetrahedron's tensor against the closed
  form; a loft between two exact octagons, one a square with its edge midpoints pushed out (a loft
  pairs equal segment counts and its cap triangulator refuses collinear corners); a cup against its
  closed form. `mass_properties_mesh_internal_test.go`: the ladder narrows a sphere's tensor interval
  at each `k` a revolve mesh reaches, and a revolve the analytic path refuses is read off its mesh.
- Depends on: nothing.

### PR 14a (Phase 2) — support set and flat rest

- Delivers §10.5: `ContactRequest.SupportBand`, the support-set manifold of `ContactPair`, the band track over
  the support set, the exact pair's band start and departure, and the flat rest of a tipped box.
- Files: `contact_pair.go`, `contact_faceted_pair.go`, `contact_faceted_manifold.go`,
  `internal/pair/planar_manifold.go`, `contact_sweep.go`, `contact_sweep_band.go`, `contact_sweep_faceted.go`,
  `contact_sweep_rotation.go`, `dynamics/world.go`, `dynamics/schedule_event.go`.
- Test (`apitest`): new `apitest/contact_support_band_test.go`, over the `8 mm` cube on a floor turned about `Y` by
  `sin θ = 2⁻²⁴`, so its far edge stands exactly `2⁻²¹ mm` up and every height below is a float: at
  `SupportBand = 2⁻²⁰ mm` the touch publishes four points, the far edge's two with `Separation` exactly `2⁻²¹`
  and zero bound, in both body orders, and two points at a zero band; the cube lifted by `2⁻³⁰ mm` is
  `ContactBand` with `Gap.Bound` at least `2⁻³⁰` and four points at heights `2⁻³⁰` and `2⁻³⁰ + 2⁻²¹`, and
  `Separated` at a band below `2⁻³⁰`; the cube sunk by `2⁻³⁰ mm` is `Overlapping` with four points, two at
  `−2⁻³⁰` and two at `2⁻²¹ − 2⁻³⁰`; the cube with its far edge past the floor's rim publishes two points (the
  foot test deleted, four: shown to fail); the cube against an L-shaped tray's floor and wall publishes the
  floor's two points and the wall's two exact and two lifted points, each with its own normal.
  `apitest/contact_sweep_band_test.go` gains three tests over the tilted cube turning at `1 rad/s` about its near edge,
  which lowers the far edge, in a `2⁻²² s` sweep at a `2⁻³² s` resolution: `TestSweepPairSupportSetBand`,
  whose band track ends at the last grid fraction before the far edge's lower bound `2⁻²¹ − |h'(0)|·t − K·t²`
  reaches zero, within one grid step of that root, and whose `BandAt` encloses the far edge's exact height at
  each sampled time (the lifted term deleted from `Depth`: shown to fail); `TestSweepPairSupportSetDeparts`,
  the lifted cube moving up, `DepartedClear` from a `ContactBand` start; and
  `TestSweepPairSupportSetArrivalEndsTrack`, whose track ends before the arrival and whose replay refuses
  every instant past its end (the lifted lower-bound leg deleted, the track runs to the duration: shown to
  fail). `.github/test-shards-apitest.txt` lists them.
- Test (`dynamics`): `dynamics/tip_test.go`'s `TestBoxTipsOverAndRestsFlat` with
  `SupportBand = PenetrationResidual/2`: the cube tips as under a zero band and lands its far edge; the
  landing step's last event carries four points with a positive impulse on each edge and `PostVelocityB` and
  `PostAngularVelocityB` exactly zero; every later step publishes one initial-contact event with four points
  whose impulses sum to the kick's momentum `m·g·dt` within the certificate's linear slack, and ends with the
  pair excluded by swept boxes strictly apart or on a `SweepPersistentBand` that reaches the duration; after
  `32` steps every corner's exact height, staged through the published pose, lies in
  `[0, PenetrationResidual]` and both velocities are zero. Legs shown to fail: a zero band
  stops the landing step `StepPairUndecided` (`TestBoxTipsOverOnBandTracks`); the support set read at a band
  end's rounded event poses deleted, the solve at the landing's band end reads the far edge alone and the next
  slice's departure is unproved, `StepPairUndecided`; the `SupportBand` validation deleted, `dynamics/world_test.go`'s band above the residual passes `NewWorld`,
  and the tip scene at twice the residual stops its second step `StepPairUndecided` on a rounded `ContactBand`
  beyond the residual. The lifted lower-bound leg changes nothing in the tip run, whose arriving edge is a
  clear vertex at its track's start; the root fixtures show it.
- Depends on: PRs 13, 18.
- Shipped. The band end solves on the support set read at the rounded event poses, since the arriving vertex
  enters the band only in the track's last grid step, and the resting cube hovers inside the band with its
  swept boxes strictly apart, so no band track runs while it rests.

### PR 14b (Phase 2) — stitched planar solids in the exact pair

- Delivers §9's admission of a closed all-planar stitched solid: `planarSolidAtPose` reads
  `stitchPayload.tris`, `verts` and `triFaces` (the face map through `Body.Faces()`), refuses a nil
  triangle set, an open sheet or a failed crossing audit, and returns the largest `vertBound`, at least
  `delta`, as the displacement, zero for the scene's tetrahedron.
- Files: `contact_faceted_pair.go`; `docs/collision-v1-support.md`.
- Test (`apitest`): `apitest/contact_faceted_pair_test.go` gains `TestExactPlanarPairAdmitsStitchedSolid`: the §2
  tetrahedron (four patches welded at the identity) `5 mm` above a source-box floor reads
  `Separated` with `Gap` exactly `5 mm`; resting on its `XY` face it reads `Touching`, and the pair's
  reverse order the same; the tetrahedron carries the convexity certificate. `apitest/contact_faceted_manifold_test.go`
  gains `TestPlanarManifoldStitchedVertexTouch`: the tetrahedron turned to stand on its apex publishes one
  point at the apex's exact staged coordinate with `Normal` `(0, 0, 1)` and `FeatureB.Vertex` the live
  apex, and on its `XY` face the three face corners at their exact coordinates. Legs shown to fail: the
  face map deleted (`Faces` nil), the apex fixture keeps `Touching` with no manifold; the `delta` reading
  deleted, a placed copy of the tetrahedron (positive `delta`) reads an exact `Touching` where §10.4
  requires a `ContactBand`. The `vertBound` widening has no identity-placed fixture: only a
  certificate-welded stitch (`docs/surface-design.md` §6.2) carries a positive class bound at the
  identity, and the test file records that the term is the same per-vertex bound `tessellate_stitch.go`
  publishes per face, read back rather than proved twice. `.github/test-shards-apitest.txt` lists both tests.
- Depends on: PRs 10, 11. PR 14 for the scene's mass only.
- Shipped. The tests also read the tetrahedron on the `Cut` tray's floor, whose manifold needs the
  tetrahedron's certificate since the tray has none, and assert the placed copy's band against twice its
  largest live vertex bound.

### PR 14c (Phase 2) — face-local support planes

- Delivers §10.6: `planarSupportOf` admits a plane whose owner has material in front of it, under the
  column test and lateral clearance; `planarDepartureProof.lowerGap` and the replay read the clearance;
  `PlanarColumnClear` in `internal/pair` owns the projected box-triangle test and the gap.
- Files: `contact_sweep_band.go`, `contact_sweep_faceted.go`, `contact_sweep_replay.go`, new
  `internal/pair/planar_column.go`.
- Test (`apitest`): `apitest/contact_sweep_band_test.go` gains three tests over the §2 `Cut` tray and the `8 mm` cube
  of `apitest/contact_support_band_test.go`, each in both body orders: `TestSweepPairDepartsFromTrayFloor`, the
  cube rising from the tray's floor at `100 mm/s` with `ω = (0, 1, 0) rad/s`, `DepartedClear`, its
  `Departure.GapAtUntil` within its ball of the cube's lowest staged corner height at the horizon;
  `TestSweepPairBandTrackOnTrayFloor`, the cube on its edge on the tray's floor turning about that edge,
  a `SweepPersistentBand` whose `Band()` equals the `K·h²` closed form `apitest/contact_sweep_band_test.go`
  already checks on a plain floor; and `TestSweepPairColumnEndsDepartureAtWall`, the cube rising while
  sliding at `200 mm/s` toward a wall `1 mm` away, whose departure horizon is the last grid fraction before
  the cube's path box reaches the wall's projection and whose later search brackets the wall impact within
  `TimeResolution` of the exact drift. Legs shown to fail: the column test deleted, the wall fixture's
  horizon runs past the wall and the sweep is `SweepUndecided` with `SweepDepartureUnproved`; the
  lateral clearance deleted from `lowerGap`, `contact_sweep_band_internal_test.go`'s
  `TestPlanarDepartureLowerGapIsBoundedByLateralClearance`, the cube rising `1 µm` from a wall, publishes a
  lower gap above `1 µm`. `internal/pair/planar_column_test.go` checks the projected gap of a triangle
  diagonal to a box against its closed form and that a triangle behind the plane is not read.
  `.github/test-shards.txt` lists the root test and `.github/test-shards-apitest.txt` the `apitest` tests.
- Depends on: PR 14a.
- Shipped. The wall fixture spins the cube at `2⁻¹⁶ rad/s` about `Z`, since an axis-aligned source box
  that only translates against a faceted body takes the faceted-floor sweep; the band track's column
  leg is shown by `TestSweepPairColumnRefusesTrackOverRim`, a box on its edge leaning over a `2 mm`
  wall's rim, whose track without the leg runs into the rim.

### PR 14d (Phase 2) — face-local shallow penetration

- Delivers §9.6 and §9.1's crossing set: `ClassifyPlanar` records `PlanarResult.Crossings`,
  `PlanarFacePenetration` publishes the patch through one face, and `publishPlanarManifold` runs it
  when §9.3's convex-convex path publishes nothing.
- Files: `internal/pair/planar.go`, new `internal/pair/planar_face_penetration.go`,
  `contact_faceted_manifold.go`; `docs/collision-v1-support.md`.
- Test (`apitest`): `apitest/contact_faceted_manifold_test.go` gains, each in both body orders, over the §2 tray:
  `TestPlanarManifoldCornerThroughTrayFloor`, the `8 mm` cube turned `30°` about `(1, −1, 0)` and sunk so
  one corner stands exactly `2⁻²⁰ mm` below the floor, publishing one point whose `OnB` is the corner's
  exact staged coordinate, whose `OnA` is its foot on `z = 0`, whose `Separation` is exactly `−2⁻²⁰` with
  zero bound and whose `Normal` is `(0, 0, 1)`, the same under `SupportBand = 2⁻¹⁹ mm`, and the cube
  barely turned so its other bottom corners stand within that band publishing them after the sunk
  corner at their exact heights; `TestPlanarManifoldEdgeThroughTrayFloor`, the cube turned `30°` about
  `−Y` onto an edge sunk by `2⁻²⁰ mm`, two points at the edge's ends;
  `TestPlanarManifoldWithholdsCornerThroughTwoFaces`, the corner sunk into the floor and a wall at once,
  `Overlapping` with `ContactAmbiguousFeature` and no manifold. Legs shown to fail: condition 1 deleted,
  the two-face fixture reads `ContactNoNormalProof` (conditions 3 and 4 still refuse it), and
  `internal/pair/planar_face_penetration_test.go`'s corner sunk through a floor's thin skin into a cavity
  below publishes the corner at its full depth; condition 4 deleted, that file's snapshot of a tray with a
  second shell, a `1 mm` cube standing on the floor wholly inside the sunk box (no crossing, so §9.1's
  nesting cast never runs and only the column test sees it), publishes one corner; each part of
  condition 3 deleted, its region helper accepts a point set over a hole, across an L's notch or around a
  hole (`planar_face_penetration_internal_test.go`); the lifted set deleted, the barely turned cube
  publishes its sunk corner alone. `internal/pair/planar_test.go` gains
  `TestClassifyPlanarRecordsCrossings`: the sunk corner's crossings name the floor's two triangles, its
  three edges with its faces' diagonals, and the floor's diagonal, and a box crossing floor and wall
  names both faces; stopping the scan at the first crossing is shown to fail. `.github/test-shards-apitest.txt`
  lists the `apitest` tests.
- Depends on: PRs 11, 14a, 14c.
- Shipped. A pair of two certified bodies tries both orders as `M` (§9.6), and the hexagonal prism of
  `apitest/contact_faceted_pair_test.go` sunk `0.5 mm` into the `Cut` tray's floor now publishes its corner.

### PR 14e (Phase 2) — continuation inside the band

- Delivers §10.7 and the §5.2 table: `solveIslands` assigns `ContinueSeparatingTouch` only to a solved
  pair whose post-event poses read exactly `Touching`, keeps a band-end or track pair that enters no solve
  under `ContinueCertifiedTouch` while its rounded poses read `Touching` or a band within the residual,
  and drops it from the contact set when they read `Separated`; `solveEvent` passes the rounded relation
  of each gathered pair to the solve.
- Files: `dynamics/island.go`, `dynamics/schedule_event.go`.
- Test (`dynamics`): new `dynamics/tumble_rest_test.go`, `TestHexagonalPrismRestsFromVertex`: a four-body
  world (a `240 mm` source-box floor, the §2 prism at its §2 release, two far boxes) under the §2 material,
  `dt`, residuals and `SupportBand = PenetrationResidual/2`, run through `Timeline` for `1 s`, every step
  `Advanced`; every impact's impulses meet the discrete linear law; the trace carries a one-point and a
  three-or-more-point `ContactImpact` (the prism lands on a vertex and the band end at the next arrival
  solves three cap vertices at once, so no two-point impact occurs); the last step's one event holds six
  points; the final velocities are exactly zero, every vertex's exact staged height is at or above the
  floor and at least three lie within `PenetrationResidual`, and the last slice certifies the pair by swept
  boxes strictly apart or a band track through the step's end. Legs shown to fail: the band rule deleted,
  and its separated-solve rule alone deleted, each stops the prism in its step `30`, `65.8 µs` in,
  `StepPairUndecided` with `SweepTimeFloor`. The `Separated` drop deleted changes nothing in the run, since
  a planar sweep that starts `Separated` runs the same clear search under every start policy (§10.7).
  Every zero-band fixture of the package is unchanged.
- Depends on: PR 14a.
- Shipped without the box and wedge rests, which stop `StepEventBudget` on the band-end cycle §10.7
  records as an open limit.

PRs 14b, 14c and 14e touch disjoint files and may land in any order; PR 14d follows 14c, whose
`PlanarColumnClear` it calls; PR 14f follows 14e.

### PR 14f (Phase 2) — rest inside the band

- Delivers §10.8: `SweepRequest.RestSpeed` and the two-sided hold of a rested vertex, the per-vertex
  curvature `K_p`, the overlapping initial contact, the carried correction allowance and the cut band track
  of §5 step 7 and §6.6, and the §2 residuals of the `tumble` scene.
- Files: `contact_sweep.go` (the field and its validation), `contact_sweep_band.go`, `dynamics/step.go`
  (`sweepRequest`), `dynamics/schedule_event.go` (`solveEvent`, `trackPairs`, `carriedPenetration`), `dynamics/island.go`
  (`solveIslands`), `.github/test-shards-apitest.txt`.
- Test (`apitest`): `apitest/contact_sweep_test.go` rejects a negative, non-finite or non-`Velocity` `RestSpeed` with
  `SupportBand`'s errors. `apitest/contact_sweep_band_test.go` gains `TestSweepPairRestedVertexHoldsTwoSided`: the §13
  PR 14a tilted cube, its far edge `2⁻²¹ mm` up, turning about its near edge so the far edge descends at
  `8·cos θ mm/s`, swept under `ContinueCertifiedTouch` with `RestSpeed = 10 mm/s`: the track reaches the
  duration, the far corners' exact staged heights at its end are negative, `BandAt` encloses each far
  corner's exact height at every sampled fraction, and replay accepts every fraction through the end; with
  `RestSpeed = 1 mm/s` or the zero Value the track ends at the arrival as `TestSweepPairSupportSetArrivalEndsTrack`'s
  does; under `ContinueSeparatingTouch` with `RestSpeed = 10 mm/s` the departure's horizon is the one-sided
  root (the departure rests nothing): over `2⁻⁹ s` the cube rises at `8 + 2⁻⁸ mm/s`, so its far edge rises
  slowly, is rested, and still ends the departure where `2⁻²¹ − K_p·u²` and its rate reach zero; a slow spin
  about `Z` routes the pair past the source-box departure, and the far edge, out of the band by then, lets
  the clear search finish the sweep. Legs shown to fail: the rested skip deleted from `clearAt`, the track
  ends at the arrival; the rested skip copied into the departure, the horizon reaches the duration with a
  corner below the plane. `TestSweepPairPerVertexCurvature`: the edge box of `edgeBoxScene` turning about
  its resting edge publishes `SweepPersistentTouch` with a nil `Band()` in both orders, and the same box
  with its pivot moved `(0, 0, 4) mm` off the edge publishes a band whose `Band()` is
  `½·|ω|·|ω×(p − c)|·h²` for its contact vertices within the published bound (the per-vertex term replaced
  by `|ω|²·ρ`: shown to fail on the exact touch). Fixtures whose closed form pins the global `K` and go red
  under `K_p`, re-pinned to the per-vertex form: `TestSweepPairPlanarBandTrack` and
  `TestSweepPairBandTrackOnTrayFloor` (both now an exact touch), `TestSweepPairPlanarBandLeavesFace`,
  `TestSweepPairPlanarDepartureFromEdge` and `TestSweepPairPlanarDepartureRotatingSupport` (horizons). Every
  other root fixture is unchanged.
- Test (`dynamics`, `dynamics/tumble_rest_test.go`, the PR 14e fixture's world and material):
  `TestWedgeRestsFromVertex`, the §2 wedge at its release beside the §2 prism, two dynamic bodies, at the
  shipped residuals (`1 nm`, band `0.5 nm`), 256 steps every one `Advanced`; the trace carries a one-point and
  a three-point `ContactImpact` for the wedge; the last step's events are the prism's six points and the
  wedge's three; both bodies end with exactly zero velocities, the wedge's three cap vertices within
  `PenetrationResidual` and every other vertex above the floor; every impact meets the discrete linear law.
  Legs shown to fail: `RestSpeed` left the zero Value in `sweepRequest`, the wedge stops `StepEventBudget`
  in its step `29` on the chain of §10.7; the cut rule deleted (`trackPairs` gathers every covering band
  track as a track pair), the wedge stops `StepTrackUnproved` in step `29`, `701.7 µs` in, at the prism's
  impact. `TestBoxSpinsOnCornerInsideBand`, the `60°` box at its §2 release and spin, shipped residuals, 50
  steps every one `Advanced` (the run then rocks on the corner with six events a step at about `0.8 s` a
  step, which 256 steps would carry past the `dynamics` package's race budget), with some step's published
  pose holding a lower corner at a negative exact height within `PenetrationResidual` and none lower, at a
  `ContactSlop` of `1e-12 mm`. Legs shown to fail: the carried allowance deleted, step `49`
  `StepCorrectionFailed`, the corner `4e-11 mm` below the floor at the step's start; the overlapping initial
  contact deleted, step `49` `StepManifoldMissing`; `K_p` replaced by the global `K`, step `48` `StepEventBudget` with band ends `46` to
  `62 µs` apart. `TestBoxBouncesOnEdgeAndRestsFlat`, the `30°` box at its §2 release and spin with
  `PenetrationResidual = 10 µm` and `SupportBand = 5 µm`, 256 steps every one `Advanced`; the trace carries a
  one-point, a two-point and a four-point `ContactImpact`; at least one slice follows a band end with
  `Request.StartPolicy == ContinueCertifiedTouch` and a `ContactBand` start sample; every step from the
  `65th` publishes one four-point event (the run records the `46th`; the margin covers another
  architecture's rounding); the final velocities are exactly zero and each lower corner's exact staged
  height lies in `[0, PenetrationResidual]`. Leg shown to fail: `SupportBand = 0.5 nm` with
  `PenetrationResidual = 1 nm`, the box still rocking with six events in its step `64` (§10.8). The island
  rule that sends an `Overlapping` gathered pair into the solve when nothing closes changes none of these
  runs when deleted: every overlapping pair they gather has a closing point. Every shipped `dynamics` fixture, the prism's included, is unchanged.
- Depends on: PRs 14a, 14e.

### PR 15 (Phase 2) — `tumble`

- Delivers the Phase 2 exit scene of §2.
- Files: `_gallery/dynamics_clip.go`, `dynamics/scene_test.go`.
- Test: the Phase 2 exit criteria at §2's residuals; every release is placed so that no body reaches a tray
  wall within `3 s` and no body's swept box reaches a wall's projection (§10.5, §10.6). The exploration run
  with the tray at those residuals goes 256 steps `Advanced` with every body resting (§10.8).
- Depends on: PRs 9, 13, 14, 14a, 14b, 14c, 14d, 14e, 14f.
- Shipped. The tray, `[−90, 90]²×[−10, 40]` minus `[−80, 80]²×[0, 50]`, is the only fixed body. The boxes
  start with their centers `40 mm` up at `(±45, ±45)`, the prism tipped `37°` about `−Y` on a vertex `20 mm`
  up at `(−55, 0)`, the wedge (a `16/12 mm` triangle, `8 mm` thick) turned `50°` about `(1, 2, 3)` at
  `(40, 0, 25)`, and the tetrahedron (PR 14's, twice the size) turned `40°` about `(3, −1, 2)` at
  `(0, 0, 25)`. Every step of the `3 s` is `Advanced`, every event is a body on the tray's floor, and from
  step `77` every body rests: seven face events a step, the boxes on four points, the prism on six, the
  wedge and the tetrahedron on three. `dynamics/scene_test.go`'s `requireTumbleExit` asserts §2's criteria:
  the linear momentum balance of every step, a one-, a two- and a four-or-more-point impact, each box's first
  impact within `TimeResolution` after the band entry of its lowest corner's drift (a zero band in its place
  is red, the exact touch lying `7 µs` later), and every body at rest face down inside the walls with a
  track or a box exclusion in its last slice. The whole scene spends nearly all of its time in its first
  `80` steps, in the rotating sweeps, and runs beyond the `dynamics` package's ten-minute budget on the CI
  runners: `TestTumbleScene` runs it under
  `DECAD_TUMBLE_FULL`, `TestTumbleSceneSubset` runs the `30°` box, the prism, the wedge and the tetrahedron
  for `0.5 s` through the same assertions on the legs without the race detector, and the `_gallery` job
  runs the whole scene in its tests, which also render its first frame.

### PR 16 (Phase 3) — third-order section moments and the general revolve

- Delivers §8.6.
- Files: `moments.go`, `moments_circular.go`, `spline_moments.go`, new `mass_properties_revolve.go`.
- Test (`apitest`): `apitest/mass_properties_revolve_test.go`: a quarter revolve of an off-axis rectangle encloses
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
- Files: `contact_faceted_pair.go`, `contact_sweep_faceted.go`, `contact_sweep_rotation.go`,
  `contact_sweep_band.go`, `contact_sweep_replay.go`, `dynamics/schedule.go`, `dynamics/schedule_band.go`,
  `dynamics/schedule_event.go`, `dynamics/island.go`.
- Test (`apitest`): `apitest/contact_band_test.go` and `contact_band_internal_test.go`: the band, its charges and
  each refusal over a cap-loop chamfered block, whose `δ` read through `Tessellate` is its contour's
  rounding (about `1e-15 mm`), and a box `Union` a chorded disc, whose `δ` is the chord sagitta (about
  `4e-4 mm`) and places held gaps inside and outside the band. `dynamics/contact_band_test.go`: an
  octagonal prism rests on that knob's disc in its band (`2δ` about `7.3e-4 mm`) at
  `PenetrationResidual = 1e-3 mm`, the disc delivering each kick's momentum and a band track carrying
  each step, and the contact set carries the band pair into a step without gravity; at `1e-4 mm` the
  landing is `StepPairUndecided`, and the same pair fixed is `StepFixedPairRelation`.
- Depends on: PR 13.
- Shipped.

### PR 19 (Phase 3) — curved analytic manifolds (C2)

- Delivers plane/cylinder ruling and cylinder/cylinder manifolds from clearance's certified feet with
  `Face.NormalAt` balls.
- Files: new `contact_analytic_manifold.go`; `clearance_tiers.go`.
- Test (`apitest`): `apitest/contact_analytic_manifold_test.go`: a cylinder on its side against a floor publishes
  the ruling's two endpoints with the computed normal ball.
- Depends on: nothing.
- Shipped, at identity query poses; PR 20 adds the plane/cylinder ruling at placed poses
  (contact-geometry §4.5). The gate the ruling certificates read is
  `docs/clearance-design.md` §6's carrier displacement, so a full revolve's end-angle term does not refuse
  them.

### PR 20 (Phase 3) — rolling band tracks

- Delivers §10.4's rolling track.
- Files: `contact_sweep_faceted.go`, new `contact_sweep_rolling.go`, `contact_analytic_manifold.go`,
  `contact_pair.go`.
- Test: `dynamics/rolling_test.go`: the cylinder rolls `π·20 mm` in one turn at `ω = 2π rad/s` with
  contact-point speed within `VelocityResidual` of zero.
- Depends on: PRs 13, 19.
- Shipped. The rolling track, its manifold, `BandAt` and replay live in `contact_sweep_rolling.go`,
  dispatched from `contact_sweep_faceted.go` and read through `contact_sweep.go` and
  `contact_sweep_replay.go`; the placed-pose ruling that lets a turned cylinder start a track lives in
  `contact_analytic_manifold.go`, dispatched from `contact_pair.go`. Root tests:
  `apitest/contact_sweep_rolling_test.go` rolls a `Ø20` cylinder `π·20 mm` in one turn at `2π rad/s` as an
  exact touch track whose contact point is at rest within its published ball, publishes the computed
  depths of sinking, orbiting and tilting drifts, and starts tracks from translated, long-rolled and
  tipped poses; `apitest/contact_analytic_manifold_test.go` checks each placed band, ball and gap against the
  float pose's true occupied set in 512-bit arithmetic. `dynamics/rolling_test.go` rolls the cylinder
  sixteen steps of `1/16 s` in a four-body world, with and without gravity, in both body orders.

### PR 20a (Phase 3) — one-span straight sweep tessellation

- Delivers `Body.Tessellate` of a one-span straight sweep (a `sweepPayload` with no spans, no arc and no
  surface result) as the prism it reduced to (`sweepPayload.prism`), its wall roles read through the span
  prefix `prefixSweepSpanZeroRole` minted (`side(0,i,j)`); `docs/sweep-design.md` D2 gains that exception.
  The reduction builds through `evalPrismContext`, so the prism's mesh and proofs are the sweep's.
- Files: `tessellate.go`, `docs/sweep-design.md`.
- Test (`apitest`): `apitest/sweep_tessellate_test.go`: the §2 hexagon sweep's mesh has the vertex set, triangle count,
  zero `Bound()`, `BoundaryVerified` and `VolumeVerified` of the same profile's `Extrude`, and every wall
  triangle's source face is the sweep's live wall `Face` under its prefixed role; an arc sweep and a
  two-span sweep stay refused with `ErrUnsupported`. Leg shown to fail: the role prefix deleted, the
  sweep refuses with `ErrDegenerate` naming the missing role. `.github/test-shards-apitest.txt` lists it.
- Depends on: nothing.

### PR 20b (Phase 3) — `HeldChord` and the held-mesh admission of every solid payload

- Delivers §10.4's admission: `ContactRequest.HeldChord` with `SupportBand`'s validation at both entry
  points and in `validateStepConfig`; `planarSolidAtPose`'s default arm reads any solid payload without
  an exact contact family off its `VerifyAll` tessellation (all-planar at `1 mm`, a curved face at
  `HeldChord`, a zero chord refusing it), `δ` the mesh's `Bound`, the face map from the mesh's source
  faces; the snapshot cached per body and chord beside the §9.2 certificate.
- Files: `contact_faceted_pair.go`, `contact_pair.go`, `contact_sweep.go`, `dynamics/world.go`.
- Test (`apitest`): `apitest/contact_band_test.go` gains `TestHeldMeshAdmitsEveryPayload`: the §2 bottle `5 mm` above a
  source-box floor at `HeldChord = 0.03 mm` reads `Separated` with `5 mm` inside its gap interval and a
  bound at least the `Bound()` of `Tessellate` at that chord and at most `0.05 mm`; at a zero chord it is
  `Undecided`; the §2 cup, loft and sweep each `5 mm` up read `Separated` with `Gap` exactly `5 mm`, and the
  cup on the floor at a translation pose reads `Touching`; the `2.1 mm` chamfered block `5 mm` up reads a
  positive bound below `1e-12 mm`; a second call at the same chord returns a bit-identical report; the
  request rejects a negative, non-finite or non-`Length` chord at `ContactPair`, `SweepPair` and `NewWorld`.
  Leg shown to fail: the `δ` reading deleted, the bottle with its held bottom `δ/2` above the floor reads
  an exact `Separated` whose gap interval excludes zero while the revolve's true lowest point, evaluated
  in 512-bit arithmetic as `apitest/contact_analytic_manifold_test.go` evaluates a cylinder, lies below the held
  bottom by more than that gap. `.github/test-shards-apitest.txt` lists it.
- Depends on: PR 18; PR 20a for the scene's sweep only.
- Shipped. Every reader of the chord goes through `heldChordOf` (`contact_faceted_pair.go`). The
  snapshot at the identity pose is cached on the body (`planarSnapshotOf`, `topology.go`), keyed by the
  chord only for a curved held mesh, and the §9.2 certificate shares that key; each query still maps the
  snapshot through its pose and audits it. `TestHeldMeshAdmitsEveryPayload` also reads the bottle at a
  coarser chord between two fine ones (leg shown to fail: the chord key deleted, the coarse query
  publishes the fine bound). `TestHeldChordValidation` and `dynamics/world_test.go`'s
  `TestNewWorldRejectsHeldChord` hold the refusals.

### PR 20c (Phase 3) — the displaced pair's support set and penetration manifold

- Delivers §10.4's relation rows for a lifted set and a deep overlap: `planarBandPair.classify` routes a
  held `Separated` pair within `b = max(SupportBand, δ)` through §10.5's support band over the zero-`δ`
  host's planes alone, charges `g + δ` on the gap, `δ` on each lifted point's `Separation` and each body's
  `δ` on its own witness ball; a held overlap deeper than `δ` publishes §9.3's convex-convex manifold or
  §9.6's face-local one, read over `M`'s held vertices grown by `M`'s `δ`, charged the same way.
- Files: `contact_faceted_pair.go`, `contact_faceted_manifold.go`, `internal/pair/planar_face_penetration.go`.
- Test (`apitest`): `apitest/contact_band_test.go` gains, each in both body orders over the §2 tray: the `2.1 mm`
  chamfered block turned about `Y` by `sin θ = 2⁻²⁴` and lifted `2⁻³⁰ mm` at `SupportBand = 2⁻²⁰ mm`,
  `ContactBand` with `Gap.Bound` the lowest corner's height plus `δ`, four points whose `M` balls and
  `Separation` bounds are each at least `δ` and whose `Normal` is `(0, 0, 1)`; the same block sunk
  `2⁻²⁰ mm`, `Overlapping` with the sunk corners at `−2⁻²⁰ ± δ` and the lifted corners after them; the
  bottle at `HeldChord = 0.03 mm` with its held bottom `0.8·δ` up at `SupportBand = 0.01 mm`, `ContactBand`
  with every bottom vertex (`b = δ`); the bottle `0.8·δ` above PR 18's chorded knob, `ContactBand` with no
  manifold and `ContactNoNormalProof`. Legs shown to fail: the gap charge deleted, the bottle on its side,
  turned about its axis by half a chord step so a chord of its base is lowest at a held height `δ/10`,
  publishes a `Gap` interval that excludes the true base's lowest point in 512-bit arithmetic, which hangs
  a sagitta below the chord; the host gate deleted, the knob fixture publishes a normal read from the
  chorded disc; the §9.6 margin deleted, a block whose held corner clears a wall by `δ/2` while sunk in
  the floor publishes one face. The ball and `Separation` charges have no red fixture: every shipped
  tessellation samples its vertices on the true surface within rounding, and the test file records that
  the charges follow `Bound`'s contract rather than any producer's behavior, as PR 14b records its
  `vertBound` term.
- Test (`dynamics`): `dynamics/contact_band_test.go` gains the `2.1 mm` block dropped `8 mm` onto the §2 tray
  at §2's Phase 3 residuals, landing through an `Overlapping` initial contact with a charged manifold and
  resting within `32` steps on a four-point `ContactBand` whose `Gap.Bound` is at most `δ + SupportBand`,
  both velocities exactly zero. Leg shown to fail: the overlap manifold deleted, the block's landing is
  `StepManifoldMissing`. The bottle's drop is PR 20g's fixture: its landing solve needs §6.3's witness
  torque.
- Depends on: PRs 14d, 18, 20b.
- Shipped. `planarBandPair.liftedBand` publishes the lifted set and `overlapManifold` the deep overlap's
  patch, both charged by `chargedManifold`; `pair.PlanarFacePenetrationGrown` reads §9.6's conditions 3
  and 4 over M grown by its δ. `planarLiftedSet` reads every guest, as PR 20d states. The §9.6 path needs
  M's §9.2 certificate, which the `2.1 mm` block's held mesh does not carry (a rounded foot leaves a vertex
  an ulp in front of a facet plane), so that block sunk in the floor reads `Overlapping` with
  `ContactNonConvex` and the overlap fixtures use a `2.3 mm` block, whose held mesh is certified; §2's scene
  drops the `2.3 mm` block for that reason, and this PR's `2.1 mm` drop takes the same lifted-band landing,
  never the overlap path. The §9.6 margin fixture moves the
  tray so its wall is the plane `y = 0`, where `δ/2` is a float, and both growths must be deleted for it
  to go red, since either alone refuses it. The host-gate fixture sets the bottle on the `2.1 mm` block
  instead of the knob, whose held pair with the bottle costs `14 s` of exact classification; its red reads
  `Gap.Bound` below `2δ`, since `chargedManifold`'s exact-face check withholds a displaced host's normal
  on its own. `TestContactPairBandChargesHeldGap`'s `2⁻¹⁴ mm` gap now publishes the knob's four lower
  corners under the octagon's face. In `dynamics`, the block's held gap enters the lifted band at about
  `SupportBand` above the floor, so it lands on a four-point `ContactBand` rather than an overlap and
  rests at step `17` (leg shown to fail: the lifted band deleted, the landing's right sample is an
  overlap of the block's flat face and the step stops with `StepManifoldMissing`). Without PR 20g the
  bottle's landing solve stops with `StepIslandResidual` at §6.3's angular law, its residual the witness
  torque of its `49` lifted base vertices, about `55 kg·mm²/s`, against a limit of `1.2e-4`; its exact
  classification against the tray costs about `2 s` a step.

### PR 20d (Phase 3) — the support set of a non-convex guest

- Delivers §10.5's hull rule: `planarLiftedSet` reads every guest, and a `Touching` non-convex guest whose
  §9.1 contacts all lie on one support plane publishes that plane's contact set then its lifted set.
- Files: `contact_faceted_manifold.go`, `internal/pair/planar_manifold.go`.
- Test (`apitest`): `apitest/contact_faceted_manifold_test.go` gains, in both orders over the §2 tray: the §2 cup on the
  floor at a translation pose, `Touching` with its four bottom corners at their exact coordinates,
  `Normal` `(0, 0, 1)` and the cup's live vertices as features; the cup turned about `Y` by
  `sin θ = 2⁻²⁴` and lifted `2⁻³⁰ mm` at `SupportBand = 2⁻¹⁹ mm`, `ContactBand` with four points at their
  exact heights; the cup on the tray's rim with two corners past its outer edge publishing the two over
  it; the cup standing on the floor with its rim against a wall, `ContactNonConvex` with no manifold (a
  contact off the floor's plane). Leg shown to fail: the convexity gate restored, the first three fixtures
  publish no manifold.
- Depends on: PR 20b.
- Shipped as `TestPlanarManifoldNonConvexGuest`. The tilted cup's band is `2⁻¹⁹ mm` because its far
  corners stand `24·sin θ = 1.5·2⁻²⁰ mm` above its near ones. The rim fixture rests the cup on the tray's
  wall top at `z = 40`. `contact_faceted_pair.go` sends a touch of two non-convex bodies to the rule, and
  `contact_pair.go`'s reason and entry-point comments follow it. Further legs shown to fail: the
  host-face test on every contact deleted, the wall fixture publishes corners under two normals; the foot
  test deleted, the rim fixture withholds with `ContactAmbiguousFeature`; the guest-in-front test deleted,
  `TestPlanarManifoldNonConvexGuestBehind`'s inverted L, whose leg hangs below the step face its arm lies
  on, publishes the arm's corners. PR 10's hollow shell on the tray's floor publishes its four corners,
  and that test's `ContactNonConvex` fixture is the shell in the tray's corner.

### PR 20e (Phase 3) — the face-local ruling plane

- Delivers §10.6 for the placed ruling and the rolling track: `rulingSupport` admits a face-local plane
  under the column test at `f = 0` over the cylinder's staged corner box and records `m`;
  `classifyPlacedRuling` publishes a separated gap with lower end `min(σ_lo, m)` and a touch or band only
  when `m` exceeds its half-width; the rolling track's band search runs the column test over the corners'
  path box less `S`'s translation.
- Files: `contact_analytic_manifold.go`, `contact_sweep_rolling.go`.
- Test (`apitest`): `apitest/contact_analytic_manifold_test.go` gains, over the §2 tray: the `Ø20 × 30 mm` cylinder on
  its side on the floor at a signed-axis pose, `Touching` with the ruling's two ends and `Normal`
  `(0, 0, 1)`; turned `2⁻²⁰ rad` about `Z`, the band the plain-floor fixture publishes; `1 mm` above the
  floor with an end `0.5 mm` from a wall, `Separated` with lower end at most `0.5 mm`, upper end `1 mm`
  and the 512-bit true gap inside; an end over a wall's rim, `Undecided`; within `2⁻²⁰ mm` of the floor
  and `2⁻³⁰ mm` from a wall, `Undecided`. Legs shown to fail: the upper end clamped to `m`, the near-wall
  fixture's true gap lies above the published interval; the column test deleted, the rim fixture reads a
  touch. `apitest/contact_sweep_rolling_test.go` gains one turn on the tray's floor publishing the plain floor's
  depths, and a roll toward a wall `1 mm` away whose track ends at the last grid fraction before the
  corners' path box reaches the wall's projection; leg shown to fail: the column test deleted, the track
  reaches the duration through the wall. `dynamics/rolling_test.go` gains sixteen steps on the tray under
  gravity in both orders with the contact-point speed within `VelocityResidual` of zero.
- Depends on: PRs 14c, 20.
- Shipped. The root tests are `TestContactPairPlacedRulingOnTray`, `TestSweepPairRollingOnTrayFloor` and
  `TestSweepPairRollingTowardTrayWall`. An end disk faces a wall flat, so its true gap equals `m` and
  cannot show the clamp leg; the near-wall fixture instead rolls the cylinder `2⁻⁶ rad` about its axis
  with its corner box `0.5 mm` from a wall, where the corners bulge past the round side and the true gap,
  about `0.655 mm`, lies strictly between `m` and the `1 mm` floor gap, so deleting the lower end's `m`
  turns it red as well. The §2 tray's walls stand above the cylinder, so the rim fixture lays the ruling
  `1 mm` inside the floor with the side through the wall. The band fixture near a wall is a `2⁻²⁵ rad`
  tilt, whose band lies within `2⁻²⁰ mm`; deleting the half-width gate publishes its band. The roll
  toward a wall ends where the end centers' grown box, not the corners', reaches the wall, and with the
  axis box deleted the whole turn on the tray's floor ends a third of the way round.

### PR 20f (Phase 3) — the transfer charge

- Delivers §10.4's replay transfer charge `‖R_r − R_i‖_F·δ` in `replayDeviation`.
- Files: `contact_sweep_rotation.go`, `contact_sweep_replay.go`.
- Test (root): `contact_band_internal_test.go` gains `TestReplayTransferChargeIsTheBasisDifference`: PR 18's
  knob pair (`δ` about `4e-4 mm`) resting at a held gap `1.5·δ` on a translating path replays every
  fraction, where the `(s + 1)·δ` form refuses each (recorded in the test); on a rotating path the charge
  at every sampled fraction equals the Frobenius norm of the rounded basis less the ideal enclosure, each
  entry read at its farther endpoint so the norm bounds every member, times `δ`, computed independently
  in the test, and lies below `2⁻⁴⁰·δ`. Leg shown to fail: the charge zeroed, the rotating fixture's
  charge reads zero.
- Depends on: PR 18.
- Shipped. `rotationalSweepPath.transferCharge` in `contact_sweep_rotation.go` computes the charge and
  `replayDeviation` returns it, so replay (`contact_sweep_replay.go`) and the rotating bracket's
  narrowing read the same figure. `apitest/contact_band_test.go`'s knob falling onto the floor replays its
  bracket's left edge, whose proven gap is far under `δ`.

### PR 20g (Phase 3) — the witness torque

- Delivers §6.3's witness torque: each dynamic body's angular-law limit gains `T_β = Σ_k b_k·|J_k|_1` over
  the body's points, `b_k` its own witness ball and `|J_k|_1` the impulse interval's L1 norm at its upper
  end; the angular-momentum limit inherits it through the per-body sum; `ContactSolverReport` gains
  `WitnessTorque` (the island's largest `T_β`, `AngularMomentum` kind) and `WitnessSpin` (the largest
  `T_β / λ_lo(I_β)`, `AngularVelocity` kind), both rounded up. The mass-center ball, the normal ball and the
  velocity rows are unchanged. Rigid-dynamics "Response" names the term and the two fields and points here.
- Files: `dynamics/island_certify.go`, `dynamics/step.go`, `docs/rigid-dynamics-design.md`.
- Test (`dynamics`): `dynamics/contact_band_test.go` gains `TestDisplacedBottleRestsOnTray`: the §2 bottle
  dropped `8 mm` onto the §2 tray at §2's Phase 3 residuals, landing through a `ContactBand` initial contact
  on every vertex of its held base, bouncing at restitution `0.3`, bouncing again below `ImpactSpeed` and
  resting within `32` steps with both velocities exactly zero, its `ContactPair` at rest a `ContactBand` whose
  `Gap.Bound` is at most `δ + SupportBand`; the landing island's `WitnessTorque` equals, within one rounding,
  `Σ_k b_k·|J_k|_1` recomputed in the test from the event's manifold balls and point impulses, its
  `WitnessSpin` equals that over the bottle's certified lower eigenvalue (read through `apitest/export_test.go`) and
  lies below `1 rad/s`, and each resting island's `WitnessSpin` lies below `0.05 rad/s`; at
  `PointResolution = 1e-6 mm` the landing step is `StepManifoldMissing` (`ContactPointTooCoarse` withholds the
  band's manifold at the rounded event poses); at `PenetrationResidual = 0.1 mm` the landing step is `StepTrackUnproved`, the band
  track after the bounce holding the lifted base at `SupportBand + 2δ` (§10.4). `dynamics/island_test.go`
  gains the tamper leg of the bottle's landing proposal (`IslandProposalGates`): its spin about `Z` raised
  by four times `WitnessSpin + AngularVelocityResidual` is refused at the angular law and the angular
  momentum, and the untampered proposal passes every gate; its leg comment records that `T_β` is the one
  limit term that is not a rounding residual, with the legs below.
- Legs shown to fail: `T_β` zeroed, the bottle's landing is `StepIslandResidual` at the angular law with a
  residual about `55 kg·mm²/s` against `1.2e-4` (§13 PR 20c's record); `T_β` left out of the
  angular-momentum sum alone, the same landing is refused at the angular momentum row; the mass-center
  ball added to `b_k`, `TestFacetedFloorImpactRefusesUncertifiedResponse`'s "mass center uncertainty"
  advances; `WitnessTorque` published from `PointResolution` in place of the attained balls, the
  recomputation leg reads a published torque about `3.4` times the recomputed one. Every shipped `dynamics`
  fixture keeps its numbers: an exact body's `T_β` is the rounding of its witnesses times the impulse, and
  every refusal fixture at the angular law stands by more than that.
- Depends on: PR 20c.
- Rejected alternatives, each recorded in §6.3: a scene-wide `AngularVelocityResidual` of `0.5 rad/s`, and a
  `HeldChord` small enough for the spread to fit the old limit.
- Shipped. `bodyWitnessTorque` (`dynamics/island_certify.go`) sums `T_β` per body. The bottle's held base
  lifts `49` vertices; it lands at step `9` (`WitnessSpin` about `0.5 rad/s`), rebounds at `0.3` again at
  step `15`, meets the floor below `ImpactSpeed` and rests at step `17` (`WitnessSpin` about `0.04 rad/s`).
  `IslandProposalGates` solves a step's initial contacts and the drop's landing is an interior impact, so
  the tamper fixture (`TestIslandWitnessTorqueGates`) starts the bottle `1/32 mm` above the floor, inside
  the lifted band, closing at the landing's `383.203125 mm/s`; with `T_β` zeroed its landing is refused at the drop's
  angular residual.
  The drop costs about `33 s` locally: each of its three impact steps about `5.5 s`, the
  `PointResolution = 1e-6 mm` landing step about `4 s` and the `0.1 mm` one about `5 s`.

PRs 20a, 20e and 20f touch disjoint files and may land in any order; PR 20b follows 20a only for the
scene's sweep; PRs 20c and 20d follow 20b; PR 20g follows 20c.

### PR 21 (Phase 3) — `parts-bin`

- Delivers the Phase 3 exit scene of §2: the `2.3 mm` block, the `0.125 mm` residual and the bottle's
  `WitnessSpin` criteria are §2's.
- Files: `_gallery/dynamics_clip.go`, `dynamics/scene_test.go`.
- Test: the Phase 3 exit criteria. The exploration run (`dynamics/partsbin_explore_test.go`, kept out of
  the tree) of 48 steps with the six bodies took `63 s` with every held snapshot rebuilt at each call; the
  scene test records its cost after PR 20b's cache, and a scene that exceeds the `dynamics` package's race
  budget shortens by a change to §2.
- Depends on: PRs 15, 17, 18, 20, 20a–20g.
- Shipped at §2's settings. The tray is PR 15's; the cup, the block, the loft, the sweep and the bottle drop
  from `8 mm` at `(−45, −45)`, `(0, −45)`, `(−45, 0)`, `(−45, 45)` and `(0, 45)`, and the cylinder's axis
  starts at `(45, −40, 10)`, rolling `15 mm/s` along `+y` to `y = 20` in the `4 s`. Every step is `Advanced`.
  The five dropped bodies land in step `9`, bounce, land again and rest from step `17`, every one on a
  `ContactBand`: the cup on four lifted corners, the block on four with `δ` about `1.8e-15 mm`, the loft on
  eight, the sweep on six and the bottle on its `49` lifted base vertices with `δ` about `0.0295 mm` and
  `Gap.Bound` about `0.079 mm`. The bottle's landing `WitnessSpin` is about `0.50 rad/s` and its resting
  islands' at most about `0.042 rad/s`. The cylinder rides a rolling band track in every step after its first,
  its ruling ends at rest within `VelocityResidual`. `dynamics/scene_test.go`'s `requirePartsBinExit` asserts
  §2's criteria plus every step's linear momentum balance and every event on the floor's `+Z`. The whole scene
  takes about four minutes on an amd64 workstation, nearly all of it in the bottle (each of its three impact
  steps about `5 s`, each resting step about `0.2 s`): `TestPartsBinScene` runs it under
  `DECAD_PARTSBIN_FULL`, `TestPartsBinSceneSubset` runs every body but the bottle for `0.125 s` (about `3 s`,
  `10 s` under the race detector), and the `_gallery` job runs the whole scene in its tests, which also
  render its first frame.

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
- **Deterministic rounding.** The Go spec lets a compiler fuse `x*y + z` into one fused multiply-add,
  and arm64 does where amd64 does not, so an unrounded product can move a published float by an ulp and
  turn a touching outcome into a separating one. Every float product `dynamics` writes that feeds an add
  or a subtract on a published or certified path is rounded explicitly with `float64(...)`, which the
  spec says forbids the fusion. `r3` follows the same rule in every vector, tensor, frame and transform
  product, so the general step publishes the same bits on amd64, `GOAMD64=v3` builds and arm64.
  `GOARCH=arm64 go build -gcflags='github.com/lestrrat-3d/decad/dynamics=-d=fmahash=vy'` lists every
  fused site in `dynamics`, and it lists none.
- **No pinned bound literals.** Bounds are asserted negligible against a slack figure with a comment
  saying why; values are `InDelta` at a stated slack. FMA contraction differs between hosts.
- **Dyadic inputs.** Fixture coordinates, velocities and times are dyadic so exact comparisons (event
  fractions, touch equalities) are platform-independent.
- **Reversal and order.** Every pair kernel test runs both body orders; every schedule test permutes
  world insertion order and asserts identical events up to pair naming.
- **Budgets and cancellation.** Each new loop has a test that exhausts its budget (`Undecided` with the
  named reason, document unchanged) and one that cancels mid-loop (`ctx.Err()`, nil report).
- **Parity.** PR 5 keeps every shipped response test byte-for-byte in its assertions, except for the
  rewrites §6.5 lists; any other loosened slack is a review refusal. Two loosenings are signed off: the
  interior sphere-pair friction impacts compare within `ImpulseResidual`, `VelocityResidual` and
  `AngularVelocityResidual`, since their event is read up to one `TimeResolution` after the exact
  contact, and the reversed world of `TestThreeBodyFrictionIslandRealPath` compares within `1e-12` for
  its float summation order. `TestThreeBodyTwoDynamicSphereZeroRestitutionImpact` asserts its certified
  prefix and refusal as a known regression (§6.6).
- **Scene tests** run in both modules (§11.3); the gallery test is the one that also renders a smoke
  frame.
