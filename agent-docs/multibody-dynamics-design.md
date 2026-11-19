# Multibody Dynamics Design

This document owns the program that takes decad's pair contact proofs and the `dynamics` subpackage from
narrowly gated two- and three-body slices to N-body certified replay: the N-body world, state and trace; pair
enumeration and the certified broad phase; island formation and the certification of the general projected
solver; the multi-step `Timeline`; the order in which mass properties, contact manifolds and sweeps extend to
every solid payload; the interface the `_gallery` module needs from `kinetograph` to film a trace; and the
phases, filmable exit scenes and PR order that deliver all of it.

It owns the N-body ALGORITHMS and their certificates. The response law, step configuration and conservation
readings stay with Part R; the claims `ContactPair` and `SweepPair` make stay with
Part G and Part S; the mass integrals stay with
Part M; the system map stays with Part C. Where this
document adds an outcome, a relation value, a reason or a field to one of those contracts, that part
carries the addition and points here for the algorithm.

## Parts

This file holds 6 parts. A section number with no part prefix is a section of Part N.
`Part X §N` is section N of part X. Short names such as `collision-dynamics §3` in comments and design docs
mean the same part. The parts keep their own section numbers.

| Part | Former document | Short name | Owns |
|---|---|---|---|
| Part N | the unprefixed sections of this file | `multibody-dynamics` | N-body world, pair schedule, islands, projected-solver certificates, `Timeline`, delivery order |
| Part C | Collision and rigid dynamics system map | `collision-dynamics` | System map, package boundary, delivery order |
| Part R | Rigid dynamics | `rigid-dynamics` | `dynamics` world, state, step, response and trace contracts |
| Part M | Dynamic mass | `dynamic-mass` | Mass, center-of-mass and inertia readings |
| Part G | Contact geometry | `contact-geometry` | Pair pose query, manifolds, source-box proofs |
| Part S | Two-body contact sweep | `contact-sweep` | `Document.SweepPair`: first impact, departure, persistent contact |

Current state: every PR of §13's delivery order has shipped.
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
exit scenes run their full length in `_gallery`. `dynamics/scene_test.go` runs focused steps for Phase 3
and runs Phase 2's full scene only under `DECAD_TUMBLE_FULL` (§13 PRs 15 and 21): the Phase 1 scene,
`stack-and-drop`, its `2 s`; the Phase 2 scene, `tumble`, its `3 s` with every body resting face down;
and the Phase 3 scene, `parts-bin`, its `4 s` with the five dropped bodies resting on a `ContactBand` and
the cylinder rolling without slip.
`agent-docs/collision-v1-support.md` is the inventory of the shape pairs, responses and refusals that ship, and
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
   Part R "Response" specifies replaces the closed-form responders. Every shipped
   closed-form fixture becomes a regression fixture of the general solver and keeps producing the same
   certified numbers (§6.5).

Non-goals, each with the reason it is out:

| Not in this program | Why |
|---|---|
| Joints, deformation, fracture | Part C §2 limits the first stage to rigid solids; nothing here needs more. |
| Sleeping or deactivation | A sleeping body publishes poses no certificate backs; a resting island costs one persistent track per pair and stays certified. |
| A continuous-force integrator between events | The step's force law is the one semi-implicit kick Part R "Step and event schedule" defines; the caller picks `dt`. A tumbling body therefore moves at constant `ω` between kicks, and §10.3's band track is what lets it rest. |
| A manifold for every contact of two non-convex bodies | §9 proves a manifold when one body of the pair is convex, and §10.5's guest rule when every contact of two non-convex bodies lies on one face of one of them (§2's cup on the tray's floor). Any other touch or overlap of two non-convex bodies gets a relation and no manifold (`ContactNonConvex`); a non-convex FIXED or kinematic body against a convex dynamic one is fine, since only its faces enter. |
| kinetograph interpolating between certified poses | §11: the viewer asks for a pose at a frame time and shows that pose; it never blends two. |
| Uncertified mass from a render mesh | Part M §2.2: a `VerifyNone`/`VerifyBoundary` mesh never supplies inertia. |

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
sphere impacts leave the spheres rolling along `y` (no rolling resistance is modeled), and the
sphere-box contact needs each sphere's contact inside the floor's top face. Gravity `-9810 mm/s²`, box and floor
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

`NewWorld` admits `len(cfg.Bodies) >= 2` in any role mix, and every world
takes the scheduled step of §4.3 and §5. Internally:

```go
type worldBody struct {
    definition RigidBody
    mass       decad.MassProperties // valid only for Dynamic
    exact      *exactMass           // mass as exact rationals; non-nil only for Dynamic
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

Pair material mixes per pair exactly as Part R "World and State" states; the
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

`NewState` requires exactly one entry per world body and
sets neither of the last two fields. The scheduled step sets both on the state it publishes as `Next`,
which is also its trace's end; every state derived from another by changing an entry carries neither,
since both describe one set of entries.

The CONTACT SET is part of the physical state, like a velocity: it names the pairs whose last certified
relation is a persistent track through the end of the step (§5 step 8), and §5 step 2 reads it. It lives
on `State`, not in the cache and not in the `Trace`: a step reads only its input state, the next step
needs it, and it decides a pair's start policy rather than repeating a computation. `World` stays
immutable configuration and every per-step datum stays in the run, the published state or the report,
so concurrent steps from one state share nothing mutable that can change an outcome. The one exception
is a memo (`dynamics/world_inertia.go`): each `World` keeps its last 1024 world-axis inertia tensors
and their inverses, keyed by body index and the bits of the pose basis. A body's admitted mass reading
never changes and `r3`'s rotation reads only the basis, so a repeat returns the bits `r3` computes. Only
a successful rotation and inversion enters it, and a mutex guards it for concurrent steps.

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
island the step certified, with its exact problem and final proposal state (§6.2), the band reading
(§10.3: the slice cut, or that the track spans the slice) of every band-track report it read, keyed by
the report, and the step's `Completion` conservation reading of those entries, which the next step
publishes as its `Input`. A step reads it only when its entries equal the input state's entries. A
resting band pair reuses its report step after step, and the step reads its band twice (§5 steps 4 and
8), so its band is read once while the report is reused. Within one step, a conservation
reading of entries equal to ones the step already read is reused too: the kicked state starts the first
drift slice and the completed state ends the last. Each reading is a pure function of the world and the
entries, so a reused one is the one a new reading returns.

### 3.3 Step configuration and report

`StepConfig` carries:

```go
MaxPairSweeps uint64 // positive; counts SweptBox and SweepPair calls per step across every slice
```

A call the step answers from its own record or the input state's cache (§5.3) is not charged. `NewWorld`
refuses a zero `MaxPairSweeps` for every world; only the scheduled step charges it.

`StepReport` carries `Islands []IslandReport`; `ContactEvent` carries `Island int`, the index of the island
that published it, or `-1` for a graze or transition, which enters no solve; `StepDiagnostic` carries typed
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

The whole-path swept box of Part S §4.2 is a public read-only query, so one
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
Part S §4.2 already permits a sweep-and-prune that discards only by the strict
box certificate and changes no order. A pair the boxes exclude is recorded in the slice as
`pairProof{boxClear: true}` and is never swept.

Cost per slice is `O(N log N)` box work plus one `SweepPair` per candidate that §5.3 cannot reuse. The
step charges each `SweptBox` call and each `SweepPair` call against `MaxPairSweeps` (§12, §13 PR 8).

The step sweeps every candidate and reports one diagnostic per candidate that does not prove clear, in
canonical pair order, so the set of diagnostics does not depend on world insertion order. A swept body's published end pose is the pose its every pair
certificate replays at the slice end through `CertifiedPosesAtInterval`; a refused or different pose
makes the step `Undecided` with `StepPairUndecided`.

## 5. Event schedule

The step keeps the structure Part R "Step and event schedule" specifies — one
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

These are the three policies Part S §5.1 defines; nothing here adds one. A pair
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

Part R "Response" owns the law: the relative velocity, the restitution target
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
every member. Float rounding leaves a bouncing or resting stack's bodies slightly apart in velocity, and only exactly equal velocities let `SweepPair` prove the touch that continues the stack, as the
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
`MaxIterations` is not guaranteed, and `TestScheduledStepReusesCertificates` takes either branch for the resting pyramid.
Either way the certificate judges the republished proposal afresh. An island whose problem
differs in any bit starts cold. Restarting such an island from the cached impulses of matching
pairs and features would move a statically indeterminate island, such as a four-corner patch, to a
different split of the same total, and the published impulses would depend on the cache; the cache never
changes a published impulse.

### 6.3 Certification

After the last sweep the solver certifies the published proposal in exact rational interval arithmetic,
forward only:
it never inverts an interval tensor. The interval vocabulary is `internal/proof/interval.go`'s
`RatInterval` with `AddInterval`, `SubInterval`, `NegInterval`, `MulInterval`, `ScaleInterval` and
`PointInterval`; `dynamics` imports that package directly. The same file holds the three-component forms this section needs, `DotInterval3` and `CrossInterval3`, so
the root package and `dynamics` share one implementation; no interval code lives in `dynamics`. The certificate runs every row in `internal/proof/shared_interval.go`'s `SInterval`, which
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
(Part M §4); each manifold point's `OnA`, `OnB` and `Normal` with their balls; the
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
§2 bottle's landing impulse spreads the torque by `0.03 mm` times that impulse, and no gate can certify the
spin closer than the admitted geometry fixes it. The angular law's limit therefore carries that spread as the
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
of its vertices times the impulse and no published number of theirs changes; for the §2 bottle the landing
island publishes a `WitnessSpin` below `1 rad/s` (`dynamics/island_test.go`, `dynamics/scene_test.go`).
Raising `AngularVelocityResidual` to the landing's `WitnessSpin` instead would state one flat tolerance for
every body and every event, widen every body's energy allowance with it (rigid-dynamics "Completion,
conservation, and trace"), and leave the smaller uncertainty of other events unstated; shrinking `HeldChord`
until the spread fits the old limit multiplies the base vertices the classification must visit.

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
  at zero normal speed, stops at the root package's missing touch track under a turning platform
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

`Trace.Sample(t)` compares the exact held `t` against the slice boundaries
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
makes `Sample` return `ErrUnsupported`.

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

Part M owns the integrals and admission gates; this section fixes which path each
payload's mass takes. Each item carries the computed test of dynamic-mass §6 for that shape.

### 8.1 Prism with a non-cardinal frame or placement basis

Analytic, in `internal/massmoment/prism_properties.go`: frame-local `V, P, Q` about `(0, 0, zm)`, `zm` the recorded mid
level, from the section's second-order moments. `M`, the product of the placement and frame bases read
as exact rationals, is the map the prism's volume, areas and vertices denote through
(`agent-docs/evaluator-design.md` §5.1), and it is orthonormal only to rounding. The reading is that map's exact
image (`massmoment.AffineInertia`, as 8.6): mass `ρ·|det M|·V`, and the inertia of the second moment
`|det M|·M·S·Mᵀ`, so no defect widening enters. Positivity is proved by the leading principal minors of
the published tensor. Cardinal, undisplaced prisms keep the signed-permutation path in
`internal/massdispatch/dispatch.go`: an exact signed permutation has `|det M| = 1` and `MᵀM = I` exactly, so its image is
the rigid one.

### 8.2 Prism with positive `z0Delta`, `z1Delta` or `sectionDelta`

Analytic plus an occupied-volume error
`E = SDA·(h + z0Delta + z1Delta) + A_upper·(z0Delta + z1Delta)`, `SDA` the section's
`proofbound.SectionDisplacementArea` over its walk count and proven perimeter, `h` the recorded height and
`A_upper` the recorded area's upper end. It is charged as `E`, `R·E`, `R²·E` on `V`, each `P_i` and each
`Q_ij` per dynamic-mass §2.2, `R` the larger of the section envelope plus `sectionDelta` and
`h/2 + max(z0Delta, z1Delta)`. Phase 3's cap-blend and chamfer bodies need it; Phase 1 bodies have zero
deltas.

### 8.3 Loft

A solid loft's mesh is the exact restatement of its held triangles
(`agent-docs/tessellation-design.md` §2's `loftPayload` row), so its mass takes §8.5's path and settles at
the ladder's first step: `internal/massdispatch/dispatch.go` passes the held triangles to
`massmoment.HeldMeshMassProperties` (`internal/massmoment/mesh.go`), which integrates them through the
`tetraMoments` sums (the `/120` form) and widens by the mesh proof's `volSymDiff` as dynamic-mass §2.2 states. `loftmesh.MassAccumulator` is not
reused: its Volume and Centroid apply the exact twist corrections of chorded cells, which have no
second-moment counterpart, while the restated triangles are uncorrected and their four-leg
`volSymDiff` covers that gap.

### 8.4 Stitched planar solid, zero-bound Boolean, translation-placed Boolean

The Boolean path is `facetedMassProperties` in `internal/massdispatch/dispatch.go`, over the payload's own held triangle set and
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

Analytic, in `internal/massmoment/revolve_properties.go`. `internal/momentregion` at order `freeform.MomentThirdOrder`
adds `∫u³`, `∫u²v`, `∫uv²`, `∫v³` as rational intervals about the plane origin, every segment kind in the one
boundary form `∮u^(p+1)·v^q dv/(p+1)`: lines in `internal/momentline`, arcs and circles in
`internal/circularbounds/moments.go` through the trig-power reduction `circularMonomials`, Tier A spans in
`internal/freeform/spline_moments.go`. The plane moments are re-expressed as
`∫z^a·ρ^b` in the exact axis frame and multiplied by the angular factors `∫dφ`, `∫cos φ`, `∫sin φ`,
`∫cos² φ`, `∫sin φ cos φ`, `∫sin² φ` over `[φ0, φ1]`, built from the payload's own sweep denotation
(`revolveangle.Sweep.WidthInterval`, `revolveangle.Angle.SinCosFor`); a full turn's endpoints have exact sine
and cosine, so its odd factors vanish exactly and `π` enters through the width alone. Dynamic-mass §2.1
names exactly these terms; partial turns keep their mixed components. The local moments reach world axes
through `L`, the exact product of the placement basis and the local basis: the map the revolve's volume,
area and vertices denote through (`agent-docs/evaluator-design.md` §6), which is orthonormal only to rounding.
`massmoment.AffineInertia` takes that map's exact image, not the nearest rigid rotation: the mass is
`ρ·|det L|·V`, the centroidal second moment `|det L|·L·S·Lᵀ`, and the inertia `ρ·(trace(S′)·1 − S′)` of
that moment `S′`, so no defect widening enters. Positivity is proved by the leading principal minors of
the published tensor. The path refuses an inexact axis, a nonzero axis-snap or admitted-band allowance, an
undenoted sweep end and a section displacement, none of which it charges.

### 8.7 Sweep

Analytic, in `internal/massmoment/sweep_properties.go`. A single straight span takes 8.1/8.2; a single arc span takes
8.6 over the arc's partial revolve. A composite sweep integrates each `massmoment.SweepSpanRecord` in its own local
coordinates and sums `V`, `P` and `Q` about one shared anchor; no parallel-axis shortcut per span, since
`P` and `Q` already refer to the shared anchor. Each span reaches the composite's unplaced coordinates
as its exact image under its held frame matrix `F` (`massmoment.Transform`: `|det F|·V`, `|det F|·F·P`,
`|det F|·F·Q·Fᵀ`) beside the exact image of its local origin. The sum reaches world axes as its exact
image under the one placement basis every span shares (`massmoment.AffineInertia`), and positivity is
proved as in 8.6. A span placement that differs from the first span's,
or a span either path refuses (an arc span whose axis is not exact in its transported plane), leaves the
sweep to 8.5.

### 8.8 Cup

Analytic, in `internal/massmoment/cup_properties.go`. Outer prism minus cavity prism (both 8.1/8.2), subtracted at the
`V, P, Q` level with each contribution's own outward interval (dynamic-mass §2, §3), the cavity re-anchored
exactly onto the outer prism's mid level first. `cupPayload` holds both sections and the three levels with
their deltas; each level delta is charged on its prism, and the offset section's displacement
(`offsetDelta`, `agent-docs/modify-design.md` §9: the thickness conversion and the offset solve's rounding together)
is charged as that region's `sectionDelta`, on the cavity for an inward cup and on the outer region for an
outward one. The difference reaches world axes as its exact image under 8.1's `M` and positivity is proved
as in 8.6. A
cup either prism refuses leaves the cup to 8.5.

A dynamic body whose payload matches no item returns `ErrUnsupported` from `Body.MassProperties`, and
`NewWorld` rejects it with that error; a Fixed or Kinematic body never needs mass.

## 9. Exact planar faceted contact

This is Part G §7's stage C3 made concrete for the solids whose held boundary
is exact: every vertex an exact dyadic rational at the query pose (the recorded coordinates times the
placement and query transforms with exact products and sums, as `contactbox.SourceOrientedBoxAtPose` does for a
box) and every face planar with a source normal read exactly off its vertices. That is: a prism whose
section is all `LineSeg` with zero deltas, at any proper pose; a stitched all-planar solid with zero
`delta`; a zero-bound Boolean or its translation-only placement; and any other solid payload whose
§10.4 held mesh has zero `δ`, an exact loft, cup or one-span straight sweep among them. A body with a
positive boundary displacement takes §10.4's held mesh at that `δ`.

A stitched solid is read off its own audited triangle set (`stitchPayload.Tris` over `Verts`, with
`TriFaces` naming each triangle's live face), which `Stitch` assembles only for a closed all-planar weld
whose crossing audit passed (`AuditClean`); a curved or mixed stitch holds no triangle set and is not
admitted, nor is an open sheet. Its vertices are the welded table's floats, exact dyadics, and its
displacement is the largest per-vertex weld bound (`VertBound`, each the class bound with the placement
rounding `delta` already added), and at least `delta`: zero for a weld built at the identity whose every
class is zero-bound, which is then a §9
body, and positive for a placed or certificate-welded stitch, which is a §10.4 held mesh with that δ. The
scene's tetrahedron is the former: four patches on the `XY`, `XZ`, slanted and diagonal planes, welded at
the identity, every vertex bound zero, every face planar (`internal/planarpose/pose.go`, §13 PR 14b).

### 9.1 Relation for two exact planar bodies

The relation of two such bodies at exact poses is decided by exact rational tests over their triangle
sets, prefiltered by per-triangle boxes as `meshBoolean` does and charged to the shared `workBudget`
with `ctx` polled every `proofbound.WorkPollInterval` operations. The split follows the one `internal/contactbox/box.go` and
`internal/pair/box/axis_box.go` already make: `internal/pair/planar/planar.go` takes the two exact vertex and
triangle snapshots and returns the relation, gap and feature pairs over `proof.Dyadic`; the root
`internal/planarpose/pose.go` admits the bodies and builds the snapshots at the query poses, and
`internal/planarband` maps feature indices back to the bodies' faces and publishes the report. A prism's caps reuse the
tessellator's cap triangulation of its recorded section, accepted only after an exact check that every
cap triangle is counterclockwise; with the closed-mesh audit, that proves the triangles tile the section
exactly. The tests are:

- a certified transversal crossing of two facets (`internal/proof/orientation.go`'s predicates) proves `Overlapping`;
  the kernel records every crossing it finds, an edge of one body through a facet of the other or two
  coplanar facets of positive overlap, as `PlanarResult.Crossings`, each naming the facet and the edge's
  two facets, so §9.6 can tell which faces a shallow overlap passes through;
- with no crossing, one nesting cast per shell (`agent-docs/clearance-design.md` §2's ray ladder, closed-form
  for planes) proves containment, hence `Overlapping`, or mutual outsideness;
- with no crossing and no nesting, the exact minimum over facet pairs of the squared distance
  (vertex-face and edge-edge candidates, rational) is either positive, proving `Separated` with the gap
  enclosed between two floats whose squares are compared exactly against it, or zero, proving `Touching`
  when every zero-distance feature pair has opposed material sides.

The distance scan visits its candidates in a fixed index order and records the first one that reaches
the minimum. Each body caches, per partner body, the candidate pair that set the last minimum of their
relation. The next relation of the pair reads that pair's exact distance first and uses it only to
prune box pairs farther than it, never as the minimum, so the recorded candidate and every outcome are
the same with or without the cache (`internal/pair/planar/planar_prune.go`).

Neither the crossing scan nor the distance scan tests every pair of boxes. Each asks a bounding-volume
tree over the other body's facet (or edge) float boxes for the boxes the float pre-test keeps, in index
order, so the pairs visited and the order of the sites are those of the full scan
(`internal/pair/planar/planar_boxtree.go`). The scans charge one poll per query and one per box kept;
a call that charges more than `stepLimit` polls (2^29, `internal/pair/planar/planar.go`) fails with `planar.ErrWorkLimit`. `planarpose.ClassifyExactPair` reports that as a body
pair the exact kernel does not admit, so `ContactPair` goes on to its other routes and ends `Undecided`
or unsupported instead of running unbounded.

Two coplanar facets with matching outward normals and a positive-area overlap also prove `Overlapping`.
A shell whose every vertex lies on the other body's boundary needs no cast: it meets the contact set.
"Opposed material sides" is a local proof at each zero-distance site (a vertex on a facet, two crossing
edges, a collinear edge overlap, a coplanar edge chord through a facet), taken over every triangle of
each body that holds the site. The triangles that may hold a site come from the same facet box tree,
queried with the site's outward float box, plus every triangle with a zero normal, which the point test
reads without a box; the point test then decides each exactly, so the fan is the one a scan over every
triangle finds (`internal/pair/planar/planar_local.go`). It holds when either a plane through the site has one body's triangles
on or behind it and the other's on or in front, with a triangle of each body whose own plane bounds that
body's triangles there, or one body's triangles lie in front of every plane of the other's there (a box
in a tray corner). A site where neither holds leaves the relation `Undecided` with
`ContactAmbiguousFeature`. Known limit: two flush bodies meeting at a saddle point (side by side on one
base plane, one filling the other's notch) are such a site, so their touch stays `Undecided`; proving it
needs a complete local-cone decision, which this design does not include.

A convex body's relation against a convex body may shortcut through separating axes (face normals and
edge cross products) exactly as `contactbox.ClassifyOriented` does; the triangle path is the general one.

### 9.2 Convexity certificate

A body is CONVEX when every held vertex lies on or behind every facet plane, tested as exact rational
signed volumes, and its mesh passes the solid audits of `agent-docs/tessellation-design.md` §1. The certificate
is computed once per body at its placement and cached on the body (it is invariant under any affine
pose with a positive determinant, which is every pose §9 admits). A dynamic body without it gets a
manifold only against a body that carries one (§9.3, §9.6) or through §10.5's non-convex guest rule, and
otherwise relations only, with the appended reason `ContactNonConvex` on an absent manifold. A Fixed or
Kinematic body needs no certificate. `ContactPair` does not know motion types: it names
`ContactNonConvex` on an overlapping pair when neither body carries the certificate, since §9.3 and §9.6 need one convex side, and on a
touching pair of two such bodies that §10.5's non-convex guest rule does not cover.
A float pre-test skips an exact signed volume only where it proves the vertex on or behind the plane, so
the certificate and its work charge equal the all-exact loop's (`agent-docs/planar-convex-filter-design.md`).

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

The kernel (`internal/pair/planar/planar_manifold.go`) builds the manifold from the zero-distance feature pairs
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
itself in `internal/pair/planar/planar_patch.go` over the two exact loops, the face map and witness publication
in `internal/planarband`, the same split §9.1 makes.
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
rational operation is charged through it, and the loop polls `ctx` every `proofbound.WorkPollInterval` operations.

**Published points.** Each output vertex that is a corner of the patch (a vertex the polygon passes
straight through is not) is lifted to 3D exactly, then converted once to a float witness with an
outward ball through `internal/pair/box`'s `PointMeasurement`, exactly as the shipped clips do. A witness whose ball exceeds
`PointResolution` withholds the manifold with `ContactPointTooCoarse`. The normal is `B`'s exact face
normal oriented `A` toward `B`, normalized with the ball §9.3 describes; an angle above
`NormalResolution` withholds the manifold with `ContactNoNormalProof`, and an axis-aligned face has a
zero ball and a zero `NormalAngle`. `Separation` is an exact zero interval.

**The shipped clips as special cases.** `internal/contactbox/clipped.go`'s `PublishClippedHorizontalPatch` (clipping through `pair/box.ClipHorizontalPolygon`) is this
algorithm with the clip polygon an axis-aligned rectangle (four axis-parallel half-planes) in the world
`XY` plane, and `internal/contactbox/oriented.go` is the case of two rectangles with parallel edges, clipped
in `A`'s rational dual basis. Both keep their dispatch for source boxes.
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

**Tests** (`apitest/contact_faceted_manifold_test.go`), every one asserting computed coordinates:

- `TestContactPairPlanarManifoldHexagonOverhang`: a hexagonal prism face overhanging one corner of a rotated
  floor face publishes its clipped points against hand-computed rational coordinates.
- `TestContactPairPlanarManifoldNotch`: a plank spanning the notch of an L-shaped zero-bound Boolean floor
  publishes the union of both components' vertices.
- `TestContactPairPlanarManifoldPlateHole`: a box covering a through-hole of a plate is withheld with
  `ContactAmbiguousFeature`, and a box beside the hole publishes its corners.
- `TestContactPairPlanarManifoldWalls`: two vertical wall faces touching (normal along `x`) publish the
  clipped patch; fixing the dropped axis to `z` reads zero area.
- `TestContactPairPlanarManifoldThirdCrossing`: a crossing vertex at the non-dyadic coordinate `1/3 mm`
  publishes at a `PointResolution` above its conversion ball and withholds with `ContactPointTooCoarse`
  below it.
- `TestContactPairPlanarManifoldRotatedBox`: a box corner on a face's rim yields zero clipped area and
  publishes §9.3's single vertex point.
- Skipping the loop-orientation step loses the patch of `TestContactPairPlanarManifoldPenetration` (the test
  file's header lists every leg shown to fail).

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
(`internal/pair/planar/planar_face_penetration.go`, `PlanarFacePenetration`) publishes a manifold when all of
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
`internal/planarband`'s `publishManifold` runs this path whenever §9.3's convex-convex path publishes nothing, including
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

`internal/sweeprun/rotation.go`'s `rotationalPairSweep` already carries the general machinery for a rigid
drift: `prepareRotationalSweepPath` builds the ideal path and travel bound, `refine`
(`internal/sweeprun/depart.go`) is contact-sweep §5's left-first dyadic search, `intervalClear` is
§4.3's `g_l + g_r > T` certificate, and `planarsweep.BodyPath.RoundedBox` (`internal/planarsweep/`)
bounds the float-to-ideal pose deviation from the body's exact corners. It is limited to source boxes by
its eight-corner inputs. `internal/sweeprun/planar.go` generalizes it to any §9 body: the exact vertex set
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
can exceed `g + PointResolution`, as the `tumble` tetrahedron's second impact on the tray floor does. The
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
The outcome states what IS exact:

```go
// SweepPersistentBand: the pair stays within a certified band of one normal.
// At every instant of the track the signed separation along Normal() of every
// contact-set point lies in [−Depth, Depth], interiors overlap by at most
// Depth along that normal, every other vertex of the touching body stays
// strictly in front of the support plane, and the source features are
// stable. Depth is the track's Band(), a Length Measurement. The track may
// end before the duration.
```

`SweepContactTrack` has `Band() *Measurement` (nil for an exact touch track). The certificate reads a
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
| `Overlapping` that a translation of the first body no longer than `δ` leaves touching or separated (`planar.PlanarShallowOverlap`) | `ContactBand`, `Gap = [−2δ, 2δ]`, no manifold |
| anything else | `Undecided` |

The clearing translation `v` bounds a held overlap's depth for any pair, convex or not: a point `x` inside
both held interiors has `x + v` outside the second body's interior, since the moved first body shares no
interior point with it, so `x` lies within `|v|` of the second body's boundary, and by the same argument with
`x − v` within `|v|` of the first's. That is the bound a convex pair's shallow patch gives. The tried
directions are the crossing facets' outward normals, the second body's as they are and the first's
reversed, the three most frequent first; floats pick a direction, and the offset is the exact lift of its
float components, admitted when its exact length is at most `δ`. A trial that reaches the planar kernel's work
limit ends the search, and no clearing translation leaves the pair `Undecided`. This decides a body seated
flush in a pocket whose rounded placement sinks it a few ulps into the floor; a body pinched by two
opposite walls has no single clearing direction and stays `Undecided`.

The lifted band `b` reads `δ` from below: a resting displaced pair is placed with its held gap at most `δ`
(§6.6), and a held vertex within `δ` of the plane is as near to touching as the body's own precision can
tell, so the lifted set is never empty at a rest the band admits, however small `SupportBand` is. The
width is a selection of which vertices are published, not a claim: each lifted point's claims are the
exact ones of §10.5 charged with `δ`, so the published set is sound at any `b`. The true gap lies in
`[g_lo − δ, g_hi + δ]`, inside `[−(g + δ), g + δ]`.

A band from a clearing translation publishes no manifold, and its reason is `ContactNonConvex` when neither
body is convex. A band from a held touch or shallow patch carries that held manifold charged afterwards (§9.4): each
witness ball grows by its own body's `δ`, and every `Separation` is the band. A lifted point of a displaced
pair carries the same charges: its `M` witness ball grows by `M`'s `δ`, its `S` witness ball by `S`'s, and
its `Separation` is the vertex's exact height widened by the summed `δ`, since the true `M` boundary lies
within `M`'s `δ` of the held vertex and the true `S` face within `S`'s `δ` of the held plane. A held overlap
deeper than `δ` is a true overlap, and its manifold is the held one §9.3's convex-convex path or §9.6's
face-local path publishes, charged the same way, with §9.6's conditions read over `M`'s held vertices
grown by `M`'s `δ`, so that a true `M` poking a second face within `δ` is refused as the held one would
be; a landing bracket places a body whose `δ` is only rounding deeper than `δ` at every rounded event pose, so
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
certified ruling feet of `agent-docs/clearance-design.md` §6 (contact-geometry §4.5) and normals from
`Face.NormalAt`. A full source cylinder's ruling is the one such manifold that ships: at identity query
poses (§13 PR 19), and against a signed-axis face of an exact planar body at any pose, rolling included
(§13 PRs 20 and 20e).

**Rolling.** A cylinder rolling on a floor takes §10.3's track with the ruling's two ends as the
contact set. `internal/sweeprun/rolling.go` picks the sides, reads the start through `ContactPair` and
publishes the track; `internal/planarsweep/rolling.go` proves the band (`planarsweep.ProveRollingBand`)
and certifies it (`planarsweep.RollingTrack`: depth, manifold and replay check). `SweepPair` admits a
full source cylinder `M` at any start pose with a positive determinant whose path rotates, against an
exact planar body `S` with zero `δ` whose path only translates; `ContactPair` must prove the start a
ruling touch or the placed band of contact-geometry §4.5. The support plane is the plane that section picks: a face plane of `S` whose
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
other start its depth is `c₀`, the rounding of the start basis.
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
`8 mm` cube of `dynamics/tip_test.go` lands its far edge, and the frictionless two-point impact turns it
back, which brings its near edge down. §10.3's band track over
the exact contact set cannot carry that edge's arrival: the track ends where the arriving vertex's lower height
bound reaches zero, which lies inside the first grid step once the edge is within its approach speed × `TimeResolution`
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

**Admission.** `ContactRequest` has one field for it:

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
inside the host face, its exact height) hold for any `M`; `PlanarSupportSets` reads every guest. The §2 cup
is non-convex and rests on its lifted set. A non-convex guest at a `Touching` relation, which §9.3 withholds
with `ContactNonConvex`, publishes a manifold when every zero-distance feature pair §9.1 recorded lies on one
support plane of `S`, every `M` vertex on or in front of it and each contact's foot inside the host face:
that plane's contact set (its `M` vertices at zero height, each strictly inside the face) followed by its
lifted set, with the plane's face normal, each contact vertex an exact touching vertex with an exact foot,
which needs no convexity either. A contact lies on the face when its `S` feature is a facet of the face, or
an edge or vertex one of whose faces it is; `PlanarGuestTouch` (`internal/pair/planar/planar_manifold.go`) tries
every such face of each body in turn as the host and publishes every one whose contact set is nonempty.
§9.4's clipped patch, which would add the crossing points of a face overhanging `S`, needs §9.3's
certificate and is not attempted; a contact feature off that plane keeps `ContactNonConvex` with no manifold.

**`ContactPair`.** With a positive `SupportBand` the exact planar path (§9.1, `internal/planarband`)
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
touches today: the box patches certify exact touches and know no band. `internal/pair/planar/planar_support_sets.go`
defines `PlanarSupportSets`, which takes the two snapshots, their support planes and the band and returns the lifted
points with their exact heights; `internal/planarband` maps and publishes them.

**`SweepPair`.** A §9 pair whose first sample is `Touching`, or `ContactBand` from this path, continues under
both policies over the support set (`planarContinuation` in
`internal/sweeprun/planar.go` admits a `ContactBand` initial event of two source boxes as it admits a touch):

- `planarsweep.FindPathSupports` admits a plane whose support set is nonempty and records each lifted
  vertex's exact start height `h0` and start rate `h'(0)`.
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
  exact foot on `S`'s rounded plane and `Separation = [0 ± Depth]`. `planarsweep.TrackProof.ReplayHeights`
  and §10.1's replay rules are unchanged: they read every `M` vertex against the held depth.

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

`TestBoxTipsOverAndRestsFlat` (`dynamics/tip_test.go`) pins the sequence. The cube lands its far edge as a
two-point impact. The far edge's band track ends where the near edge would reach the floor within a grid step,
and the near edge is caught inside the band. The four-point solve at that band end puts a positive impulse on
each edge and leaves both velocities exactly zero. Every later step absorbs its kick on the four corners. A vertex that arrives
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
scene's bodies all rest on, depart from and tip over the tray's floor, so `planarsweep.ReadSupportCandidate`
(`internal/planarsweep/`) admits a FACE-LOCAL support plane instead: the plane of a flat face `h` of
`S` (one `Face`, every triangle coplanar with one outward normal `n`, as `planar.BuildSupportFace` already
requires) whose owner only translates, so the column test reads `S`'s start triangles less that
translation, such that

- every vertex of `M` lies on or in front of the plane, the contact and lifted sets being §10.2's and
  §10.5's as today;
- the COLUMN TEST holds over the horizon: with `B(f)` the coordinate box of every `M` vertex's ideal path
  over `[0, f]` (`BodyPath.CornerSpan`), less `S`'s own translation, and `P` the projection along `n` onto the
  plane's frame (the axis of `n`'s largest component dropped, §9.4's frame), every triangle of `S` with
  a vertex strictly in front of the plane has `P(triangle)` strictly apart from `P(B(f))`, decided by
  the separating axes of the projected box and the triangle's three edge normals in exact rational
  arithmetic (`segmentMeetsBox`'s form in `internal/pair/planar/support_face.go`);
- the LATERAL CLEARANCE `m(f)` is the least, over those triangles, of the largest separating-axis gap
  between `P(triangle)` and `P(B(f))`, each gap divided by an upper bound on its axis's length
  (`ratSqrtUp`), so that `m(f)` is a lower bound on the in-plane distance, and the projection shortens
  no distance, on the true one; a convex `S`, every vertex of which lies on or behind the plane, has
  no such triangle and `m = +∞`.

`B(f)` grows with `f`, so the column test and `m(f)` are monotone and the grid search of §10.2 and
§10.3 reads them as it reads `planarsweep.ClearAt` and `contains`: the departure's horizon and the band track's end
are the largest grid fractions through which every test holds. Soundness: every point of `M` lies in
its vertices' hull, hence in the column over `P(B(f))`; material of `S` behind the plane is at least
the least vertex height away along `n`; material of `S` in front of the plane projects at least `m(f)`
away, hence is at least `m(f)` away. The pair's separation on `(0, f]` is therefore at least
`min(least vertex height bound / |n|_hi, m(f))`, and that minimum is what `planarsweep.DepartureProof.LowerGap`
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

The placed ruling of contact-geometry §4.5 (`placedruling.Support`) reads a
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
closes. Each shows in a `tumble` run with the §2 material (`e = 0.3`, `μ = 0.4`), `dt = 1/256 s`,
`TimeResolution = 1 ns`, `PenetrationResidual = 1 nm` and `SupportBand = 0.5 nm` (the fixtures of
`dynamics/tumble_rest_test.go`, §13 PR 14e):

- The spinning `20 mm` box lands on an edge, and the solve leaves both edge corners rising slowly (the edge is
  the instantaneous axis of the tip that follows). The pair rides a band track that ends where the corners'
  quadratic lower height bound reaches zero. Nothing closes at that band end, so §6.1 enters no solve and the
  pair takes `ContinueSeparatingTouch`. The next sweep departs briefly and then cannot certify the pair clear:
  inside the band its samples read `ContactBand`, whose published `Gap` is `[0 ± g]` with no lower end, and
  §4.3's travel certificate needs `g_l + g_r` above the travel per grid step, which is larger than the
  corners' height. The search bisects to the time floor, `SweepTimeFloor`, and the oriented-box report that
  the planar rerun fell back from says `SweepDepartureUnproved`.
- The hexagonal prism bounces off three vertices while its pivot vertex, inside the band, descends. The solve
  separates the pair, the push leaves it a `ContactBand`, and under `ContinueSeparatingTouch` the departure
  ends at the pivot's arrival with the pair still inside the band. An impact bracket needs a `Separated` left
  end, so the bracket of the pivot's touch, `[ContactBand, Overlapping]`, is `SweepTimeFloor`.
- The wedge bounces off one vertex while another, inside the band, descends; the same bracket shape stops it.

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
`SupportBand >= (|v| + ρ·|ω|)·TimeResolution` for the pair's faster body (`0.5 nm / 1 ns` is `500 mm/s`). A pair that leaves the band slower
than that stops at the time floor, as a pair starting apart by less than its travel per grid step does
today; the band and the resolution are the caller's.

Under this rule each run above passes its stop. The prism's band track ends at the arriving vertex, one grid
step before it reaches the floor, where it stands inside the band and the support set read at the rounded
event poses holds it beside the bounced vertex, so the island solves three vertices of its hexagonal cap
together; every later step absorbs its kick on all six, and the prism rests with exactly zero velocities. A zero `SupportBand` never publishes a
`ContactBand` for an exact pair, so every shipped zero-band fixture keeps its policy and its numbers; a
§10.4 band pair that a solve separates, which `ContinueSeparatingTouch` could never depart (§10.4),
continues on its band track as well. Dropping a `Separated` pair from the contact set changes no
certified outcome of an exact planar pair: a planar sweep that starts `Separated` runs the same clear
search under every start policy, so the drop only keeps the §5.3 reuse key and the contact set honest.

The box and the wedge rest is a limit of the band track that §10.8 closes. Each reaches a state in which the solve
leaves it resting on one or two lifted vertices, so the pair reads `ContactBand` and continues on a band
track that ends where a lifted vertex's lower height bound `h0 − K·t²` reaches zero. Under the body's
rotation the true descent of that vertex is only a small fraction of what `K` allows, so at each band end the
vertex stands at a fraction of its previous height; the solve removes a small closing speed, and the vertex
arrives again sooner. The band ends shrink geometrically and never reach the step's end, and the step stops
with `StepEventBudget`. No `dynamics` rule alone closes the
cycle, since no correction lands a rotated vertex in exact touch and a sweep that starts `Overlapping` is
`SweepInitiallyOverlapping`; §10.8 changes the band track itself (`internal/sweeprun/planar.go`) and adds the two
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
  (Part S §5.1) asks for a departure.

### 10.8 Rest inside the band

§10.7 records the band-end chain that stops the boxes and the wedge. Two bounds of the band track make the
chain, and one law of the step limits what a rest can be; this section closes the chain and states the
limit, each from the exploration runs of the `tumble` fixtures of `dynamics/tumble_rest_test.go` (a plain `240 mm` floor, 256 steps) with the §2
material and `dt`.

**The chain.** After a vertex impact the solve leaves the body on one or two LIFTED vertices (§10.5): inside
the band, above the plane, their normal speeds within `VelocityResidual` of zero, the body still turning. The
band track holds a lifted vertex one-sided and ends where its lower bound `h0 + h'(0)·t − K·t²` reaches zero
(§10.5). `K` is §10.2's global bound `½·|n|·|ω|²·ρ`, `ρ` the farthest vertex from the pivot. The true second
derivative of a height is `n·R(ωu)·(ω×(ω×(p0 − c)))`, whose magnitude `|ω|·|ω×(p0 − c)|` is constant in `u`
and is zero for a vertex on the spin axis through the mass center. A box spinning on a corner about a nearly vertical axis therefore descends by a small fraction of what
`K·t²` allows. At each band end the vertex stands at a fraction of its previous height, the
solve removes the closing speed the true curvature built, and the
next track ends at the smaller root. No correction lands a rotated vertex in exact touch (§10.5), so the chain has no last
event. Its two causes are the one-sided hold and the global `K`.

**Rested vertices.** `SweepRequest` has one field for it:

```go
RestSpeed units.Value // nonnegative Velocity; the zero Value rests no vertex
```

A lifted vertex is RESTED when `RestSpeed` is positive and its exact start rate satisfies
`h'(0) >= −RestSpeed·|n|_lo`: it closes no faster than `RestSpeed`, or rises. The band track holds a rested
vertex as it holds a contact vertex, on both sides of the plane: `|h(t)| <= h0 + |h'(0)|·t + K_p·t²` by
Taylor's theorem with the per-vertex curvature `K_p` below, and `Depth` is the largest of that bound over the
rested set, `r·t + K_p·t²` over the contact set and §10.5's one-sided bound over the remaining lifted set. The
horizon test (`planarsweep.ClearAt`) skips a rested vertex, so the track no longer ends where its lower bound reaches
zero; it ends where `Depth` leaves the residual (§5 step 4), where a clear or unrested lifted vertex would
reach the plane, or where a foot leaves the face. §10.2's departure is unchanged and keeps every vertex
outside the contact set positive, rested or not: a departure claims strict separation. `ManifoldAt` publishes
a rested vertex as before, with `Separation = [0 ± Depth]`, and `ReplayHeights` reads the held depth as
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
`K_p <= K` because `|ω×(p − c)| <= |ω|·ρ`. `ClearAt`, `DepthAt`, the departure's horizon and
`planarsweep.DepartureProof.LowerGap` read `K_p` in place of `K`; the replay still reads the held depth. A body
whose contact vertices lie on its spin axis with `r = 0` now publishes an exact `SweepPersistentTouch` where
the global bound published a band of depth `K·h²`: the edge box of `apitest/contact_sweep_band_test.go`, turning
about its resting edge, is one. A box spinning on its corner, which the global bound stops under the rested hold with `StepEventBudget`,
runs on under `K_p` (`TestBoxSpinsOnCornerInsideBand`).

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
closes the impulses are zero and the island publishes only the correction. Without the first rule the `60°` box at the shipped residuals stops in step `49` with `StepManifoldMissing`
("initial contact has no manifold within the contact request"); without the second the wedge, resting beside
the prism, stops in step `29` with `StepTrackUnproved` ("a pair continued in contact starts
SweepInitiallyOverlapping") at the prism's impact.

**What rests, and at which band.** Under the three changes, at the shipped residuals (`PenetrationResidual =
1 nm`, `SupportBand = 0.5 nm`) the prism (six points) and the wedge (its triangular cap, three points) rest with exactly zero velocities
through 256 steps in one world. Every box runs 256 steps `Advanced` without resting: each settles into a
step-periodic rocking cycle of several events per step (vertex impacts, then two edge impacts) and ends
every step with its lowest corner above the band, its mass center rising and yawing slowly. The cycle's
mechanism, read at a larger residual and band, where it has fewer events per step: the kick lands
the `30°` box on one edge inside the band (two points, friction at the Coulomb limit, the box turning about
the edge and sliding); the far edge arrives later, and the four-point solve puts its impulse on the far
corners and nothing on the near ones, because the far corners' friction impulses stop the slide and their
torque about the mass center lifts the near edge. §5's step has one kick, so the box drifts up for the
remaining slice and ends the step with its near edge up; the next kick lands it on the far edge alone, and
the end state repeats. Under continuous gravity the lifted edge would return and its re-landing's
four-point solve would zero both velocities, as the tip fixture's does (§10.5); the one-kick law does not
model that return, and no correction may stand in for it (§6.6 moves a body only to remove penetration or to
place a resting pair in touch).

A box therefore rests only when a step's kick lands on all four corners, that is when the band covers the
hover. The `tumble` scene runs at `PenetrationResidual = 10 µm` and `SupportBand = 5 µm` (§2), and there
every body rests with exactly zero velocities (the wedge within the velocity residual), each later step one face
event. The first impact of a falling box is then the band's entry, `SupportBand`
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
scene below stops with `StepCorrectionFailed`: the `30°` box stands into the tray's floor by more than
the plain allowance.

With the tray (the §2 scene itself, §9.6's face-local shallow penetration included) the `10 µm` run of all
eight bodies goes 256 steps `Advanced`, every body resting on one face from its step on.

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
  start. It changes the contract of both continuation policies (Part S §5.1) for a
  case two rules of the step cover at the poses the step publishes.
- Transferring the overlapping manifold on the rotating source-box path at zero deviation instead of
  reading the rounded poses in `dynamics`. Sound, and it may land later; the step's rule covers every
  family, including the planar path, through one site that already reads rounded poses for impacts and band
  ends.
- A shorter `dt` for the hover. The lifted edge still ends its step many bands up; the hover is the one-kick law's, and the band is the caller's.

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
`main.go` provides `go run . dynamics -scene <name> [-out -fps -width -height -workers -smoke]`, which advances
the timeline to the clip length, builds one kinetograph scene with a `Driven` node per body, the scene's
camera and the directional lights of `scene.go`, and renders through `render.New` as `clip.go` does. When the
timeline stops early, the frames before its certified end render and the command fails at the first frame
past it, naming the stopped step's diagnostics. `-smoke` advances the whole timeline and renders the first
frame alone at `160×90`. The gallery module is the only module that imports kinetograph.

### 11.3 Scene tests

`_gallery/dynamics_clip_test.go` runs each exit scene's timeline, built through the `dynamics` subcommand's
scene map, and asserts the §2 exit criteria on the trace: certified end time, final poses, event kinds and
times, conservation. It films each timeline and renders its first frame at the `-smoke` size. The scenes'
tests run in parallel, and the CI gallery job smoke-renders only stack-and-drop through the subcommand,
whose path is the same for every scene.
`dynamics/scene_test.go` asserts the same scene from inside the module so CI runs it without the
gallery's toolchain. Both use the real producers end to end.

## 12. Work budgets, cancellation and `Undecided`

Validation runs before `ctx` is read; afterwards every loop that calls into decad — the box pass,
each `SweepPair`, each `ContactPair`, each island sweep, each correction sweep — checks `ctx` before the
call, and cancellation returns `ctx.Err()` with a nil report, as the two-body step does. Within decad,
the kernels (§9.1, §9.4, §10.1) charge a `workBudget` from `newWorkBudget(ctx)` and poll it per
`agent-docs/interference-design.md` §7.2.

Three budgets bound a step's work without changing any certified outcome: `MaxPairSweeps` (§3.3) over
`SweptBox` and `SweepPair` calls, `MaxEvents` over published events, `MaxIterations` per island sweep.
`MaxPairSweeps` counts every call the step makes — the broad phase, the pair sweeps, the box-exclusion
checks of §7.1 and the correction sweeps of §6.6 — and not a call §5.3 reuses; `Trace.Sample`'s own
checks are not charged. Each `SweepPair` call carries `MaxPoseEvaluations`. Exhausting any
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

Every PR below has shipped. Each entry names what the PR delivers, the files it owns and the tests that pin
it; "Depends on" lists the PRs whose code it builds on. Phase 1 is PRs 1–9, Phase 2 is PRs 10–15, Phase 3 is
PRs 16–21. A root-package test is listed in `.github/test-shards.txt` and an `apitest` test in
`.github/test-shards-apitest.txt`; `dynamics` tests are not sharded. A change to what ships — a body count
`NewWorld` admits, a shape pair, a response, a refusal — also updates `agent-docs/collision-v1-support.md`,
the user-facing inventory; the "Files" lines below do not repeat it.

### PR 1 (Phase 1) — N-body `World` and `State`

- Delivers §3.1 and §3.2: the pair table, per-pair material, `Bodies()`/`Pairs()`, `NewState` over a
  slice; the two- and three-body steps of that time read through the table (PR 5 deleted them).
- Files: `dynamics/world.go`, new `dynamics/pairs.go`, `dynamics/three_body.go`.
- Test: `dynamics/world_test.go` constructs a five-body world, asserts canonical pair order and every
  exclusion and override rule; every existing fixture is unchanged.
- Depends on: nothing.

### PR 2 (Phase 1) — `Document.SweptBox`

- Delivers §4.2: `SweptBox`, `StrictlyDisjoint`, `Box()`; the cylinder and bounded-faceted sweeps consume it.
- Files: new `swept_box.go`, `internal/sweeprun/cylinder.go`, `internal/sweeprun/faceted.go`.
- Test (`apitest`): `apitest/swept_box_test.go`: a rotating box's swept box contains its exact corners at fractions
  `0`, `1/3` and `1`; dropping the travel term lets a corner escape (shown to fail); meeting boxes are
  not disjoint.
- Depends on: nothing.

### PR 3 (Phase 1) — N-body drift step with the broad phase

- Delivers §4.3 and §5 without islands: every candidate pair must be `Clear` or `DepartedClear`; an
  event is `Undecided` with `StepUnsupported` before PR 4's solver.
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
- The pyramid uses `dt = 1/256 s`, so the kick is exact, and `ImpactSpeed = 64 mm/s`, so its
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

### PR 6 (Phase 1) — multi-event `Trace`, `Timeline`, typed diagnostics

- Delivers §3.4, §7.1, §7.2 and §12.
- Files: `dynamics/step.go`, new `dynamics/trace.go`, `dynamics/timeline.go`.
- Test: `dynamics/timeline_test.go`: a sphere bouncing on a floor with `e = 0.5` over three steps; the
  bounce times match the closed-form sequence; `Timeline.Sample` across a step boundary; a forced
  `Undecided` stops the timeline with `StepEventBudget` and `Advance` then returns `ErrTimelineStopped`.
- Depends on: PR 4.
- Includes §5's interior events (impacts, transitions, grazes, zero-time repeats), kinematic island
  participants, §6.2's common velocity, §6.6's group correction and §7.1's per-sample check. Its tests add
  `dynamics/schedule_event_test.go`: five bounces between a floor and a ceiling in one step ending in rest
  under `ImpactSpeed`, a box stack bouncing as one, a kinematic platform's impact and work, a graze, an
  edge transition, the event budget's certified prefix, and the box-exclusion check at a slice end and at
  a sample.

### PR 7 (Phase 1) — rotated and displaced prism mass

- Delivers §8.1 and §8.2.
- Files: `internal/massdispatch/dispatch.go`, `internal/massmoment/prism_properties.go`.
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
- Includes the contact set carried on `State` (§3.2, §5 step 2), so a later step continues a
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
- Built against kinetograph's `Driven` node (its decision D12). The scene's steps are `1/256 s` and its
  `ImpactSpeed` `64 mm/s`, as the pyramid fixtures use (§2). Both scene tests replay all 512 steps; the
  gallery's also checks every frame's poses against `Timeline.Sample`, evaluates the clip from concurrent
  goroutines, renders the first frame, and fails a frame past a shortened timeline's certified end.

### PR 10 (Phase 2) — exact planar pair relation and convexity

- Delivers §9.1 and §9.2.
- Files: new `internal/pair/planar/planar.go` and `internal/planarpose/pose.go`; `contact_pair.go`.
- Test (`apitest`): `apitest/contact_faceted_pair_test.go`, with the snapshot-level cases in
  `internal/pair/planar/planar_test.go`: a hexagonal prism at a `37°` pose against a tray reads a
  `3 mm` gap enclosed, a vertex touch and a shallow crossing; a small box nested in a hollow Boolean's
  wall is `Overlapping` and one in its cavity is `Separated`; the non-convex Booleans report
  `ContactNonConvex`.
- Depends on: nothing.

### PR 11 (Phase 2) — faceted manifolds and the exact-clipped patch

- Delivers §9.3, §9.4 and the shallow-penetration patch.
- Files: new `internal/pair/planar/planar_patch.go`, `internal/pair/planar/planar_manifold.go`,
  `internal/planarpose/pose.go`.
- Test (`apitest`): `apitest/contact_faceted_manifold_test.go`: a rotated box on a face publishes one point at a
  vertex touch, two at an edge touch, and the clipped hexagon's seven extremal vertices at a face
  touch, each at its exact coordinate; §9.4's parity, wall, reversed-loop, hole and
  `ContactPointTooCoarse` fixtures, with the wall and reversed-loop legs shown to fail.
- Depends on: PR 10.

### PR 12 (Phase 2) — general rotating sweep

- Delivers §10.1.
- Files: new `internal/sweeprun/planar.go`; `contact_sweep.go`, `internal/sweeprun/rotation.go`.
- Test (`apitest`): `apitest/contact_sweep_faceted_test.go`: a wedge tumbling toward a floor brackets its first
  vertex impact at the exact drift time within `TimeResolution`; the deviation leg is shown to fail.
- Depends on: PRs 10, 11.
- Includes the deep-vertex overlap witness in new `internal/pair/planar/planar_depth.go`.

### PR 13 (Phase 2) — generalized departure and band tracks

- Delivers §10.2, `SweepPersistentBand`, `Band()` and `BandAt()` (§10.3); `dynamics` consumes band
  tracks and advances through a rotating pair's impact (§5 steps 4, 6 and 7).
- Files: `internal/sweeprun/planar.go`, `contact_sweep.go`,
  `internal/sweepproof/replay.go`, `internal/sweeprun/rotation.go`, `contact_pair.go`,
  `internal/pair/planar/planar_manifold.go`, `dynamics/schedule.go`, new `dynamics/schedule_band.go`,
  `dynamics/schedule_event.go`, `dynamics/island.go`.
- Test (`apitest`): `apitest/contact_sweep_band_test.go`: a box resting on an edge with `ω = (0,1,0) rad/s`
  publishes a band track whose `Depth` equals `K·h²` for the computed `K`, and `BandAt` the same closed
  form over a prefix; `TestSweepPairPlanarDepartureFromEdge` replays a rotating bracket
  whose gap lies below its own pose rounding, which shows the deviation charge; `apitest/contact_sweep_rotation_test.go` publishes a turned box's edge poking through a
  face, and `internal/pair/planar/planar_patch_test.go` a wedge's, clipped to the face. `dynamics/tip_test.go`: a
  cube on its edge, turned `30°` with its center of mass beyond the edge, tips over in a few steps; each
  step lands the edge as a rotating impact, rides a band track to the band end, and lifts clear; every
  impulse meets the discrete linear and angular laws, every band end is the last grid fraction within
  `PenetrationResidual`, and the trace replays every rotating bracket. The cube lands on its far edge.
- Depends on: PR 12.
- Ships without the flat rest. A cube landing on its far edge is turned by a float rotation that is
  never exactly flat, so the exact relation sees one edge in touch and the other a little above, and with
  restitution zero the cube rocks between its two bottom edges with a shrinking tilt. Once the far edge
  closes within one grid step of the near edge's touch, no certificate covers the pair: the band track
  ends before the first grid fraction, and a sweep that starts in touch cannot bracket another vertex's
  impact. `dynamics/tip_test.go` asserts that stop, `StepPairUndecided` in the landing step, with its
  certified prefix through the landing; under a positive `SupportBand` the cube rests flat (§10.5, PR 14a).
  The one-kick step with the edge's support solved at the start of each step tips a near-balanced cube in
  a few steps of `1/256 s`.

### PR 14 (Phase 2) — loft, stitched and fallback mass

- Delivers §8.3, §8.4 and §8.5.
- Files: `internal/massdispatch/dispatch.go`.
- Test (`apitest`): `apitest/mass_properties_mesh_test.go`: the stitched tetrahedron's tensor against the closed
  form; a loft between two exact octagons, one a square with its edge midpoints pushed out (a loft
  pairs equal segment counts and its cap triangulator refuses collinear corners); a cap-chamfered box, which
  has no analytic mass path and reaches the tolerance ladder, against its closed form
  (`TestMassPropertiesCapChamferFallback`); `TestMassPropertiesRevolvedSphere` reads a revolved sphere.
- Depends on: nothing.

### PR 14a (Phase 2) — support set and flat rest

- Delivers §10.5: `ContactRequest.SupportBand`, the support-set manifold of `ContactPair`, the band track over
  the support set, the exact pair's band start and departure, and the flat rest of a tipped box.
- Files: `contact_pair.go`, `internal/planarpose/pose.go`,
  `internal/pair/planar/planar_manifold.go`, `contact_sweep.go`, `internal/sweeprun/planar.go`,
  `internal/sweeprun/rotation.go`, `dynamics/world.go`, `dynamics/schedule_event.go`.
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
- The band end solves on the support set read at the rounded event poses, since the arriving vertex
  enters the band only in the track's last grid step, and the resting cube hovers inside the band with its
  swept boxes strictly apart, so no band track runs while it rests.

### PR 14b (Phase 2) — stitched planar solids in the exact pair

- Delivers §9's admission of a closed all-planar stitched solid: `planarpose.SolidAtPose` reads
  `stitchPayload.Tris`, `Verts` and `TriFaces` (the face map through `Body.Faces()`), refuses a nil
  triangle set, an open sheet or a failed crossing audit, and returns the largest `VertBound`, at least
  `delta`, as the displacement, zero for the scene's tetrahedron.
- Files: `internal/planarpose/pose.go`; `agent-docs/collision-v1-support.md`.
- Test (`apitest`): `apitest/contact_faceted_pair_test.go` gains `TestExactPlanarPairAdmitsStitchedSolid`: the §2
  tetrahedron (four patches welded at the identity) `5 mm` above a source-box floor reads
  `Separated` with `Gap` exactly `5 mm`; resting on its `XY` face it reads `Touching`, and the pair's
  reverse order the same; the tetrahedron carries the convexity certificate. `apitest/contact_faceted_manifold_test.go`
  gains `TestPlanarManifoldStitchedVertexTouch`: the tetrahedron turned to stand on its apex publishes one
  point at the apex's exact staged coordinate with `Normal` `(0, 0, 1)` and `FeatureB.Vertex` the live
  apex, and on its `XY` face the three face corners at their exact coordinates. Legs shown to fail: the
  face map deleted (`Faces` nil), the apex fixture keeps `Touching` with no manifold; the `delta` reading
  deleted, a placed copy of the tetrahedron (positive `delta`) reads an exact `Touching` where §10.4
  requires a `ContactBand`. The `VertBound` widening has no identity-placed fixture: only a
  certificate-welded stitch (`agent-docs/surface-design.md` §6.2) carries a positive class bound at the
  identity, and the test file records that the term is the same per-vertex bound `internal/stitchtess/stitch.go`
  publishes per face, read back rather than proved twice. `.github/test-shards-apitest.txt` lists both tests.
- Depends on: PRs 10, 11. PR 14 for the scene's mass only.
- The tests also read the tetrahedron on the `Cut` tray's floor, whose manifold needs the
  tetrahedron's certificate since the tray has none, and assert the placed copy's band against twice its
  largest live vertex bound.

### PR 14c (Phase 2) — face-local support planes

- Delivers §10.6: `planarsweep.ReadSupportCandidate` admits a plane whose owner has material in front of
  it, under the column test and lateral clearance; `planarsweep.DepartureProof.LowerGap` and the replay
  read the clearance;
  `PlanarColumnClear` in `internal/pair` owns the projected box-triangle test and the gap.
- Files: `internal/sweeprun/planar.go`, `internal/sweepproof/replay.go`, new
  `internal/pair/planar/planar_column.go`.
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
  horizon runs past the wall and the sweep is `SweepUndecided` with `SweepDepartureUnproved`. `internal/pair/planar/planar_column_test.go` checks the projected gap of a triangle
  diagonal to a box against its closed form and that a triangle behind the plane is not read.
  `.github/test-shards.txt` lists the root test and `.github/test-shards-apitest.txt` the `apitest` tests.
- Depends on: PR 14a.
- The wall fixture spins the cube at `2⁻¹⁶ rad/s` about `Z`, since an axis-aligned source box
  that only translates against a faceted body takes the faceted-floor sweep; the band track's column
  leg is shown by `TestSweepPairColumnRefusesTrackOverRim`, a box on its edge leaning over a `2 mm`
  wall's rim, whose track without the leg runs into the rim.

### PR 14d (Phase 2) — face-local shallow penetration

- Delivers §9.6 and §9.1's crossing set: `ClassifyPlanar` records `PlanarResult.Crossings`,
  `PlanarFacePenetration` publishes the patch through one face, and `internal/planarband`'s `publishManifold` runs it
  when §9.3's convex-convex path publishes nothing.
- Files: `internal/pair/planar/planar.go`, new `internal/pair/planar/planar_face_penetration.go`,
  `internal/planarpose/pose.go`; `agent-docs/collision-v1-support.md`.
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
  `internal/pair/planar/planar_face_penetration_test.go`'s corner sunk through a floor's thin skin into a cavity
  below publishes the corner at its full depth; condition 4 deleted, that file's snapshot of a tray with a
  second shell, a `1 mm` cube standing on the floor wholly inside the sunk box (no crossing, so §9.1's
  nesting cast never runs and only the column test sees it), publishes one corner; each part of
  condition 3 deleted, its region helper accepts a point set over a hole, across an L's notch or around a
  hole (`TestHullInsideRegion` in that same file); the lifted set deleted, the barely turned cube
  publishes its sunk corner alone. `.github/test-shards-apitest.txt`
  lists the `apitest` tests.
- Depends on: PRs 11, 14a, 14c.
- A pair of two certified bodies tries both orders as `M` (§9.6), and the hexagonal prism of
  `apitest/contact_faceted_pair_test.go` sunk `0.5 mm` into the `Cut` tray's floor publishes its corner.

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
- Ships without the box and wedge rests, which stop `StepEventBudget` on the band-end cycle §10.7
  records as an open limit.

PRs 14b, 14c and 14e touch disjoint files and may land in any order; PR 14d follows 14c, whose
`PlanarColumnClear` it calls; PR 14f follows 14e.

### PR 14f (Phase 2) — rest inside the band

- Delivers §10.8: `SweepRequest.RestSpeed` and the two-sided hold of a rested vertex, the per-vertex
  curvature `K_p`, the overlapping initial contact, the carried correction allowance and the cut band track
  of §5 step 7 and §6.6, and the §2 residuals of the `tumble` scene.
- Files: `contact_sweep.go` (the field and its validation), `internal/sweeprun/planar.go`, `dynamics/step.go`
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
  steps every one `Advanced` (the run then rocks on the corner with six events a step, and 256 steps would exceed the
  `dynamics` package's race budget), with some step's published
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
- The tray, `[−90, 90]²×[−10, 40]` minus `[−80, 80]²×[0, 50]`, is the only fixed body. The boxes
  start with their centers `40 mm` up at `(±45, ±45)`, the prism tipped `37°` about `−Y` on a vertex `20 mm`
  up at `(−55, 0)`, the wedge (a `16/12 mm` triangle, `8 mm` thick) turned `50°` about `(1, 2, 3)` at
  `(40, 0, 25)`, and the tetrahedron (PR 14's, twice the size) turned `40°` about `(3, −1, 2)` at
  `(0, 0, 25)`. Every step of the `3 s` is `Advanced`, every event is a body on the tray's floor, and every
  body ends resting on one face: the boxes on four points, the prism on six, the wedge and the tetrahedron
  on three. `dynamics/scene_test.go`'s `requireTumbleExit` asserts §2's criteria:
  the linear momentum balance of every step, a one-, a two- and a four-or-more-point impact, each box's first
  impact within `TimeResolution` after the band entry of its lowest corner's drift (a zero band in its place
  is red, the exact touch lying later), and every body at rest face down inside the walls with a
  track or a box exclusion in its last slice. The whole scene spends nearly all of its time in its first
  `80` steps, in the rotating sweeps, and exceeds the `dynamics` package's CI time budget: `TestTumbleScene` runs it under
  `DECAD_TUMBLE_FULL`, `TestTumbleSceneSubset` runs the `30°` box, the prism, the wedge and the tetrahedron
  for `0.5 s` through the same assertions on the legs without the race detector, and the `_gallery` job
  runs the whole scene in its tests, which also render its first frame.

### PR 16 (Phase 3) — third-order section moments and the general revolve

- Delivers §8.6.
- Files: `internal/momentregion`, `moments_circular.go`, `spline_moments.go`, `internal/massmoment/revolve_properties.go`.
- Test (`apitest`): `apitest/mass_properties_revolve_test.go`: a quarter revolve of an off-axis rectangle encloses
  the independently integrated `r³`, `r²z` and `rz²` terms in its mixed components, and an off-axis
  triangle makes the `r²z` products nonzero; a full torus against the closed form; a rotated placement
  encloses `Q I Qᵀ`, with the orthonormality-defect leg shown to fail.
- Depends on: nothing.

### PR 17 (Phase 3) — sweep and cup mass

- Delivers §8.7 and §8.8.
- Files: `internal/massmoment/sweep_properties.go`, `internal/massmoment/cup_properties.go`.
- Test (root): a composite sweep against the sum of its spans; the cup against outer minus cavity.
- Depends on: PRs 7, 16.

### PR 18 (Phase 3) — `ContactBand` for positive-displacement bodies

- Delivers §10.4.
- Files: `internal/planarpose/pose.go`, `internal/sweeprun/planar.go`, `internal/sweeprun/rotation.go`,
  `internal/sweepproof/replay.go`, `dynamics/schedule.go`, `dynamics/schedule_band.go`,
  `dynamics/schedule_event.go`, `dynamics/island.go`.
- Test (`apitest`): `apitest/contact_band_test.go`: the band, its charges and
  each refusal over a cap-loop chamfered block, whose `δ` read through `Tessellate` is its contour's
  rounding (about `1e-15 mm`), and a box `Union` a chorded disc, whose `δ` is the chord sagitta (about
  `4e-4 mm`) and places held gaps inside and outside the band. `dynamics/contact_band_test.go`: an
  octagonal prism rests on that knob's disc in its band (`2δ` about `7.3e-4 mm`) at
  `PenetrationResidual = 1e-3 mm`, the disc delivering each kick's momentum and a band track carrying
  each step, and the contact set carries the band pair into a step without gravity; at `1e-4 mm` the
  landing is `StepPairUndecided`, and the same pair fixed is `StepFixedPairRelation`.
- Depends on: PR 13.

### PR 19 (Phase 3) — curved analytic manifolds (C2)

- Delivers plane/cylinder ruling and cylinder/cylinder manifolds from clearance's certified feet with
  `Face.NormalAt` balls.
- Files: new `internal/contactruling/placed/ruling.go`; `internal/clearance/pairkernel/cells.go`.
- Test (`apitest`): `apitest/contact_analytic_manifold_test.go`: a cylinder on its side against a floor publishes
  the ruling's two endpoints with the computed normal ball.
- Depends on: nothing.
- Ships at identity query poses; PR 20 adds the plane/cylinder ruling at placed poses
  (contact-geometry §4.5). The gate the ruling certificates read is
  `agent-docs/clearance-design.md` §6's carrier displacement, so a full revolve's end-angle term does not refuse
  them.

### PR 20 (Phase 3) — rolling band tracks

- Delivers §10.4's rolling track.
- Files: `internal/sweeprun/planar.go`, new `internal/sweeprun/rolling.go`, `internal/contactruling/placed/ruling.go`,
  `contact_pair.go`.
- Test: `dynamics/rolling_test.go`: the cylinder rolls `π·20 mm` in one turn at `ω = 2π rad/s` with
  contact-point speed within `VelocityResidual` of zero.
- Depends on: PRs 13, 19.
- The rolling band proof, its certificate, manifold, `BandAt` depth and replay check live in
  `internal/planarsweep/rolling.go`; `internal/sweeprun/rolling.go` runs the sweep and publishes the track,
  dispatched from `internal/sweeprun/planar.go` and read through `contact_sweep.go` and
  `internal/sweepproof/replay.go`; the placed-pose ruling that lets a turned cylinder start a track lives in
  `internal/contactruling/placed/ruling.go`, dispatched from `contact_pair.go`. Root tests:
  `apitest/contact_sweep_rolling_test.go` rolls a `Ø20` cylinder `π·20 mm` in one turn at `2π rad/s` as an
  exact touch track whose contact point is at rest within its published ball, publishes the computed
  depths of sinking, orbiting and tilting drifts, and starts tracks from translated, long-rolled and
  tipped poses; `apitest/contact_analytic_manifold_test.go` checks each placed band, ball and gap against the
  float pose's true occupied set in 512-bit arithmetic. `dynamics/rolling_test.go` rolls the cylinder
  sixteen steps of `1/16 s` in a four-body world, with and without gravity, in both body orders.

### PR 20a (Phase 3) — one-span straight sweep tessellation

- Delivers `Body.Tessellate` of a one-span straight sweep (a `sweepPayload` with no spans, no arc and no
  surface result) as the prism it reduced to (`sweepPayload.Prism`), its wall roles read through the span
  prefix `sweeprecord`'s `prefixSpanZeroRole` minted (`side(0,i,j)`); `agent-docs/sweep-design.md` D2 gains that exception.
  The reduction builds through `prismrecord.Build`, so the prism's mesh and proofs are the sweep's.
- Files: `tessellate.go`, `agent-docs/sweep-design.md`.
- Test (`apitest`): `apitest/sweep_tessellate_test.go`: the §2 hexagon sweep's mesh has the vertex set, triangle count,
  zero `Bound()`, `BoundaryVerified` and `VolumeVerified` of the same profile's `Extrude`, and every wall
  triangle's source face is the sweep's live wall `Face` under its prefixed role; an arc sweep and a
  two-span sweep stay refused with `ErrUnsupported`. Leg shown to fail: the role prefix deleted, the
  sweep refuses with `ErrDegenerate` naming the missing role. `.github/test-shards-apitest.txt` lists it.
- Depends on: nothing.

### PR 20b (Phase 3) — `HeldChord` and the held-mesh admission of every solid payload

- Delivers §10.4's admission: `ContactRequest.HeldChord` with `SupportBand`'s validation at both entry
  points and in `validateStepConfig`; `planarpose.SolidAtPose`'s default arm reads any solid payload without
  an exact contact family off its `VerifyAll` tessellation (all-planar at `1 mm`, a curved face at
  `HeldChord`, a zero chord refusing it), `δ` the mesh's `Bound`, the face map from the mesh's source
  faces; the snapshot cached per body and chord beside the §9.2 certificate.
- Files: `internal/planarpose/pose.go`, `contact_pair.go`, `contact_sweep.go`, `dynamics/world.go`.
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
- Every reader of the chord goes through `ContactRequest.HeldChord` (`internal/planarpose/pose.go`). The
  snapshot at the identity pose is cached on the body (`planarpose.snapshotOf`, `topology.go`), keyed by the
  chord only for a curved held mesh, and the §9.2 certificate shares that key; each query still maps the
  snapshot through its pose and audits it. `TestHeldMeshAdmitsEveryPayload` also reads the bottle at a
  coarser chord between two fine ones (leg shown to fail: the chord key deleted, the coarse query
  publishes the fine bound). `TestHeldChordValidation` and `dynamics/world_test.go`'s
  `TestNewWorldRejectsHeldChord` hold the refusals.

### PR 20c (Phase 3) — the displaced pair's support set and penetration manifold

- Delivers §10.4's relation rows for a lifted set and a deep overlap: `internal/planarband.Publish` routes a
  held `Separated` pair within `b = max(SupportBand, δ)` through §10.5's support band over the zero-`δ`
  host's planes alone, charges `g + δ` on the gap, `δ` on each lifted point's `Separation` and each body's
  `δ` on its own witness ball; a held overlap deeper than `δ` publishes §9.3's convex-convex manifold or
  §9.6's face-local one, read over `M`'s held vertices grown by `M`'s `δ`, charged the same way.
- Files: `internal/planarpose/pose.go`, `internal/pair/planar/planar_face_penetration.go`.
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
  `VertBound` term.
- Test (`dynamics`): `dynamics/contact_band_test.go` gains the `2.1 mm` block dropped `8 mm` onto the §2 tray
  at §2's Phase 3 residuals, landing through an `Overlapping` initial contact with a charged manifold and
  resting within `32` steps on a four-point `ContactBand` whose `Gap.Bound` is at most `δ + SupportBand`,
  both velocities exactly zero. Leg shown to fail: the overlap manifold deleted, the block's landing is
  `StepManifoldMissing`. The bottle's drop is PR 20g's fixture: its landing solve needs §6.3's witness
  torque.
- Depends on: PRs 14d, 18, 20b.
- `internal/planarband`'s `liftedBand` publishes the lifted set and `overlapManifold` the deep overlap's
  patch, both charged by `chargedManifold`; `pair.PlanarFacePenetrationGrown` reads §9.6's conditions 3
  and 4 over M grown by its δ. `PlanarSupportSets` reads every guest, as PR 20d states. The §9.6 path needs
  M's §9.2 certificate, which the `2.1 mm` block's held mesh does not carry (a rounded foot leaves a vertex
  an ulp in front of a facet plane), so that block sunk in the floor reads `Overlapping` with
  `ContactNonConvex` and the overlap fixtures use a `2.3 mm` block, whose held mesh is certified; §2's scene
  drops the `2.3 mm` block for that reason, and this PR's `2.1 mm` drop takes the same lifted-band landing,
  never the overlap path. The §9.6 margin fixture moves the
  tray so its wall is the plane `y = 0`, where `δ/2` is a float, and both growths must be deleted for it
  to go red, since either alone refuses it. The host-gate fixture sets the bottle on the `2.1 mm` block
  instead of the knob, whose held pair with the bottle classifies far faster; its red reads
  `Gap.Bound` below `2δ`, since `chargedManifold`'s exact-face check withholds a displaced host's normal
  on its own. `TestContactPairBandChargesHeldGap`'s `2⁻¹⁴ mm` gap now publishes the knob's four lower
  corners under the octagon's face. In `dynamics`, the block's held gap enters the lifted band at about
  `SupportBand` above the floor, so it lands on a four-point `ContactBand` rather than an overlap and
  rests within the test's `32` steps (leg shown to fail: the lifted band deleted, the landing's right sample is an
  overlap of the block's flat face and the step stops with `StepManifoldMissing`). Without PR 20g the
  bottle's landing solve stops with `StepIslandResidual` at §6.3's angular law, its residual the witness
  torque of its lifted base vertices, far above the limit.

### PR 20d (Phase 3) — the support set of a non-convex guest

- Delivers §10.5's hull rule: `PlanarSupportSets` reads every guest, and a `Touching` non-convex guest whose
  §9.1 contacts all lie on one support plane publishes that plane's contact set then its lifted set.
- Files: `internal/planarpose/pose.go`, `internal/pair/planar/planar_manifold.go`.
- Test (`apitest`): `apitest/contact_faceted_manifold_test.go` gains, in both orders over the §2 tray: the §2 cup on the
  floor at a translation pose, `Touching` with its four bottom corners at their exact coordinates,
  `Normal` `(0, 0, 1)` and the cup's live vertices as features; the cup turned about `Y` by
  `sin θ = 2⁻²⁴` and lifted `2⁻³⁰ mm` at `SupportBand = 2⁻¹⁹ mm`, `ContactBand` with four points at their
  exact heights; the cup on the tray's rim with two corners past its outer edge publishing the two over
  it; the cup standing on the floor with its rim against a wall, `ContactNonConvex` with no manifold (a
  contact off the floor's plane). Leg shown to fail: the convexity gate restored, the first three fixtures
  publish no manifold.
- Depends on: PR 20b.
- Pinned by `TestPlanarManifoldNonConvexGuest`. The tilted cup's band is `2⁻¹⁹ mm` because its far
  corners stand `24·sin θ = 1.5·2⁻²⁰ mm` above its near ones. The rim fixture rests the cup on the tray's
  wall top at `z = 40`. `internal/planarband` sends a touch of two non-convex bodies to the rule, and
  `contact_pair.go`'s reason and entry-point comments follow it. Further legs shown to fail: the
  host-face test on every contact deleted, the wall fixture publishes corners under two normals; the foot
  test deleted, the rim fixture withholds with `ContactAmbiguousFeature`; the guest-in-front test deleted,
  `TestPlanarManifoldNonConvexGuestBehind`'s inverted L, whose leg hangs below the step face its arm lies
  on, publishes the arm's corners. PR 10's hollow shell on the tray's floor publishes its four corners,
  and that test's `ContactNonConvex` fixture is the shell in the tray's corner.

### PR 20e (Phase 3) — the face-local ruling plane

- Delivers §10.6 for the placed ruling and the rolling track: `placedruling.Support` admits a face-local plane
  under the column test at `f = 0` over the cylinder's staged corner box and records `m`;
  `contactruling/placed.ClassifyPlacedRuling` publishes a separated gap with lower end `min(σ_lo, m)` and a touch or band only
  when `m` exceeds its half-width; the rolling track's band search runs the column test over the corners'
  path box less `S`'s translation.
- Files: `internal/contactruling/placed/ruling.go`, `internal/sweeprun/rolling.go`.
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
- The root tests are `TestContactPairPlacedRulingOnTray`, `TestSweepPairRollingOnTrayFloor` and
  `TestSweepPairRollingTowardTrayWall`. An end disk faces a wall flat, so its true gap equals `m` and
  cannot show the clamp leg; the near-wall fixture instead rolls the cylinder `2⁻⁶ rad` about its axis
  with its corner box `0.5 mm` from a wall, where the corners bulge past the round side and the true gap,
  `0.5 mm + 10·(cos 2⁻⁶ + sin 2⁻⁶ − 1) mm`, lies strictly between `m` and the `1 mm` floor gap, so deleting the lower end's `m`
  turns it red as well. The §2 tray's walls stand above the cylinder, so the rim fixture lays the ruling
  `1 mm` inside the floor with the side through the wall. The band fixture near a wall is a `2⁻²⁵ rad`
  tilt, whose band lies within `2⁻²⁰ mm`; deleting the half-width gate publishes its band. The roll
  toward a wall ends where the end centers' grown box, not the corners', reaches the wall, and with the
  axis box deleted the whole turn on the tray's floor ends before the full turn (a capability leg, not a
  soundness leg).

### PR 20f (Phase 3) — the transfer charge

- Delivers §10.4's replay transfer charge `‖R_r − R_i‖_F·δ` in `planarsweep.BodyPath.ReplayDeviation`.
- Files: `internal/sweeprun/rotation.go`, `internal/sweepproof/replay.go`.
- Test: `internal/planarsweep/body_path_test.go`'s `TestReplayTransferChargeIsTheBasisDifference`: a
  unit box holding `δ = 2⁻¹² mm` charges zero on a translating path. Turning about Z at `1 rad/s`, its
  charge at each sampled fraction equals the Frobenius norm of the rounded basis less the ideal
  enclosure, each entry read at its farther endpoint so the norm bounds every member, times `δ`,
  computed independently in the test, and lies below `2⁻⁴⁰·δ`.
  `internal/motionbound/sweep_pose_test.go`'s `TestSweepTransferChargeReadsTheFartherEndpoint` holds a
  rounded entry inside an enclosure entry `[3/4, 3/2]` and reads exactly `δ/2`. Legs shown to fail: the
  charge zeroed, the turning path's charge reads zero; the nearer endpoint read, the wide enclosure's
  charge reads `δ/4`.
- Depends on: PR 18.
- `planarsweep.BodyPath.ReplayDeviation` calls `motionbound.SweepTransferCharge` and returns
  its charge, so replay (`internal/sweepproof/replay.go`) and the rotating bracket's narrowing read the same
  figure. `apitest/contact_band_test.go`'s knob falling onto the floor replays its
  bracket's left edge, whose proven gap is far under `δ`.

### PR 20g (Phase 3) — the witness torque

- Delivers §6.3's witness torque: each dynamic body's angular-law limit gains `T_β = Σ_k b_k·|J_k|_1` over
  the body's points, `b_k` its own witness ball and `|J_k|_1` the impulse interval's L1 norm at its upper
  end; the angular-momentum limit inherits it through the per-body sum; `ContactSolverReport` gains
  `WitnessTorque` (the island's largest `T_β`, `AngularMomentum` kind) and `WitnessSpin` (the largest
  `T_β / λ_lo(I_β)`, `AngularVelocity` kind), both rounded up. The mass-center ball, the normal ball and the
  velocity rows are unchanged. Rigid-dynamics "Response" names the term and the two fields and points here.
- Files: `dynamics/island_certify.go`, `dynamics/step.go`, Part R.
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
  residual far above its limit (§13 PR 20c's record); `T_β` left out of the
  angular-momentum sum alone, the same landing is refused at the angular momentum row; the mass-center
  ball added to `b_k`, `TestFacetedFloorImpactRefusesUncertifiedResponse`'s "mass center uncertainty"
  advances; `WitnessTorque` published from `PointResolution` in place of the attained balls, the
  recomputation leg reads a published torque larger than the recomputed one. Every shipped `dynamics`
  fixture keeps its numbers: an exact body's `T_β` is the rounding of its witnesses times the impulse, and
  every refusal fixture at the angular law stands by more than that.
- Depends on: PR 20c.
- Rejected alternatives, each recorded in §6.3: a scene-wide `AngularVelocityResidual` raised to the landing's `WitnessSpin`, and a
  `HeldChord` small enough for the spread to fit the old limit.
- `bodyWitnessTorque` (`dynamics/island_certify.go`) sums `T_β` per body. The bottle lands on
  every vertex of its held base, rebounds at `0.3`, rebounds again, meets the floor below `ImpactSpeed` and rests
  within `32` steps.
  `IslandProposalGates` solves a step's initial contacts and the drop's landing is an interior impact, so
  the tamper fixture (`TestIslandWitnessTorqueGates`) starts the bottle `1/32 mm` above the floor, inside
  the lifted band, closing at the landing's `383.203125 mm/s`; with `T_β` zeroed its landing is refused at the drop's
  angular residual.

PRs 20a, 20e and 20f touch disjoint files and may land in any order; PR 20b follows 20a only for the
scene's sweep; PRs 20c and 20d follow 20b; PR 20g follows 20c.

### PR 21 (Phase 3) — `parts-bin`

- Delivers the Phase 3 exit scene of §2: the `2.3 mm` block, the `0.125 mm` residual and the bottle's
  `WitnessSpin` criteria are §2's.
- Files: `_gallery/dynamics_clip.go`, `dynamics/scene_test.go`.
- Test: the Phase 3 exit criteria. The held-snapshot cache of PR 20b keeps the scene within the `dynamics`
  package's race budget; a scene that exceeds it is shortened by a change to §2.
- Depends on: PRs 15, 17, 18, 20, 20a–20g.
- The scene uses §2's settings. The tray is PR 15's; the cup, the block, the loft, the sweep and the bottle drop
  from `8 mm` at `(−45, −45)`, `(0, −45)`, `(−45, 0)`, `(−45, 45)` and `(0, 45)`, and the cylinder's axis
  starts at `(45, −40, 10)`, rolling `15 mm/s` along `+y` to `y = 20` in the `4 s`. Every step is `Advanced`.
  The five dropped bodies land, bounce, land again and rest, every one on a
  `ContactBand`: the cup on four lifted corners and the bottle on every vertex of its held base, with each
  body's `Gap.Bound` at most its `δ` plus `SupportBand`. The bottle's landing `WitnessSpin` is below `1 rad/s` and
  its resting islands' below `0.05 rad/s`. The cylinder rides a rolling band track in every step after its first,
  its ruling ends at rest within `VelocityResidual`. `dynamics/scene_test.go`'s `requirePartsBinExit` asserts
  §2's criteria plus every step's linear momentum balance and every event on the floor's `+Z`. Nearly all of the scene's time is in the bottle. `TestPartsBinDropProofs` advances each dropped body through 24 steps to landing and rest;
  `TestPartsBinRollingProofs` checks the cylinder's contact track through 24 steps. Both use
  `requirePartsBinExit` for per-step momentum balance, floor events and final contact proofs. The `_gallery`
  tests run the complete 1024-step scene and render its first frame.

## 14. Test and fixture strategy

Every PR's test asserts computed geometry and dynamics — impulses, velocities, poses, event times, gaps,
tensor components — through the real producer and the real consumer, never an enum alone and never a
hand-written manifold, event or pose (CLAUDE.md "Correctness must be observable";
Part S §8; Part R "Verification"). Beyond that:

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
- **Parity.** PR 5 kept every shipped response test byte-for-byte in its assertions, except for the
  rewrites §6.5 lists; any other loosened slack is a review refusal. Two loosenings are signed off: the
  interior sphere-pair friction impacts compare within `ImpulseResidual`, `VelocityResidual` and
  `AngularVelocityResidual`, since their event is read up to one `TimeResolution` after the exact
  contact, and the reversed world of `TestThreeBodyFrictionIslandRealPath` compares within `1e-12` for
  its float summation order. `TestThreeBodyTwoDynamicSphereZeroRestitutionImpact` asserts its certified
  prefix and refusal as a known regression (§6.6).
- **Scene tests** run in both modules (§11.3); the gallery test is the one that also renders a smoke
  frame.

## Part C — Collision and rigid dynamics system map

This part is the system map for collision-aware rigid motion. `Document.ContactPair` certifies pair contact,
`Document.SweepPair` certifies two-body paths, `Body.MassProperties` bounds mass, center and inertia, and
`dynamics.World.Step` consumes those certificates and publishes a state, a trace and conservation readings.
Part N's "Current state" paragraph describes the N-body step that every world takes. Part R's opening
paragraphs list the contact paths the step resolves, with the conditions each one checks.
`agent-docs/collision-v1-support.md` is the inventory of the shape pairs, responses and refusals that ship.
Each companion document owns its detail.

| Design | Ownership |
|---|---|
| [Contact geometry](Part G) | `Document.ContactPair`, pair relation, and certified contact manifold. |
| [Contact sweep](Part S) | `Document.SweepPair`, two-body paths, continuous proof, and first-contact bracket. |
| [Dynamic mass](Part M) | `Body.MassProperties`, bounded center and inertia, and mass admission. |
| [Rigid dynamics](Part R) | `dynamics.World`, state, stepping, response, and replay trace. |
| [Multibody dynamics](Part N) | The N-body program: pair schedule and certified broad phase, islands and solver certification, `Timeline`, the extension order for mass, manifolds and sweeps, band tracks, the kinetograph bridge, and the phased PR order. |

### 1. System boundary

`decad` owns immutable solid geometry and the proof that a pair is separated,
touching, or overlapping. It also owns a read-only sweep that finds the first
certifiable contact interval for two moving bodies. The `dynamics` subpackage
owns body roles, material parameters, loads, velocity changes, and the state
produced by each step. It consumes `decad` pair reports and sweep certificates.
`Document.VerifyMotion` (`motion_verify.go`; `agent-docs/linkage-check-design.md` Part M) is a
prescribed-motion verification API with no response calculation. A viewer such as `kinetograph` may replay a
dynamics trace; it does not decide contact or change the solver result. The
`_gallery` module replays a `dynamics.Timeline` through the kinetograph node
kind Part N §11 specifies; the viewer shows the
certified pose at each frame time and blends none.

The detailed designs define the report fields, input gates, units, refusal
reasons, and numerical algorithms. If this system map omits or conflicts with a
detailed rule, the owning design controls that rule.

### 2. Shared rules

- Queries and steps do not mutate a `Document`, `Body`, or evaluator payload.
  A step returns a new state and a trace; the caller decides whether to retain
  them.
- Geometry certifies its claims. A numerical response may use a certified
  manifold, but an approximate impulse cannot establish that contact occurred.
  An unresolved pair stops a step with an explicit result and interval.
- Clear, touching, and overlapping are distinct relations. Touching alone
  does not prove an incoming impact. A resting or sliding pair needs its own
  certified continuation across time.
- Every dynamic body has a positive mass and invertible inertia with proved
  bounds. `units.Value` carries physical scalar kinds; `decad.QuantityVec`
  carries vectors whose components share a kind.
- Time resolution, pose evaluation limits, and response iteration limits are
  explicit. Running out of proof or work budget never silently means clear.
- Physics admits proper rigid transforms. The first stage handles rigid
  solids; joints, deformation, and fracture are outside this design.

### 3. Required dependencies

The pinned `units` dependency provides Time, Velocity, Acceleration, AngularVelocity,
Force, Torque, Impulse, and AngularMomentum for the public sweep and step APIs. The pinned `r3`
dependency provides a symmetric tensor with rotation and inversion support, which the angular response uses.
The dynamics API is a subpackage of this module, so it needs no separate module and no geometry dependency
pointing back to it.

### 4. First real integration slice

The first slice passes the real source-box contact and two-body sweep reports into one dynamics
step. `TestVerticalBoxReboundUsesProductionGeometry` (`dynamics/step_test.go`) uses a fixed box with its
top at `z=0` and a dynamic `10×10×10 mm` box starting at `z=[10,20] mm`.
The dynamic box moves down at `100 mm/s`; restitution is `0.5`, gravity is
zero, and the step lasts `0.2 s`. Contact occurs at `0.1 s`. The box leaves
at `50 mm/s` upward and finishes with its bottom at `z=5 mm`. A fixture that
constructs a manifold by hand cannot validate the geometry-to-solver boundary.

The source-box proof covers posed face contact, the immediate departure
after rebound, and persistent planar contact for resting and sliding
fixtures. It reports when a feature transition ends its proof. The
dynamic path uses linear center-of-mass travel and angular drift; endpoint
screw interpolation is reserved for prescribed kinematic motion.

### 5. Delivery order

The stack is layered in this order, and each layer's tests run the real producer and consumer:

1. The pinned physical units and the tensor operations the angular response needs, with focused tests.
2. Exact source-box contact and sweep, with the real rebound fixture of §4 run through `dynamics.World.Step`
   before any wider shape coverage.
3. Independent movers, kinematic paths, friction, resting contact, and contact transitions. Every state
   transition stays behind a sweep or contact certificate.
4. Mass and contact proofs for supported curved and mesh payloads, using each evaluator's existing bounds and
   refusal rules.
5. Candidate filtering and work limits that leave certified outcomes unchanged, and replay checks against the
   same returned trace a viewer consumes.
6. The generalization from pairs to N bodies in three phases — source primitives, exact planar solids under
   rotation, every remaining payload — each ending in one filmable scene. Part N §2 owns the phases and §13
   the PR order; layers 4 and 5 land inside those PRs.

Each layer has focused local tests for the contracts it owns.

## Part R — Rigid dynamics

This part owns the `dynamics` subpackage's world, state, step, response,
and trace contracts. Part C owns the package
boundary; Part G and Part S own
geometry results. Part N owns the N-body world,
pair schedule, island formation, the certification of the projected solver
this part's "Response" section specifies, the multi-event trace and
`Timeline`. Every world steps through Part N's §5 and §6; the slices
below are fixtures that step reproduces, not separate code paths. A two-body world steps one pair with at least one dynamic
body using density-derived or supplied mass. An axis-aligned
certified contact normal determines the response component; tangent velocity
continues through an oblique impact. Centered impacts of two dynamic bodies
apply equal and opposite impulses. An affine source-sphere pair with one
exact interior tangent advances with unchanged velocity, one zero-impulse
contact event, and full-span trace replay.
An initial frictionless, half-width offset face impact between two dynamic
source boxes can produce equal Y spin. Four equal normal point impulses pass
the bounded velocity, angular, energy, and momentum checks before a rotating
departure sweep supplies both endpoint poses.
A fixed box and a dynamic source box at matching 45-degree Y poses can resolve
an initial centered frictionless face impact. The solver checks the bounded
normal and four point impulses against its momentum and angular residuals.
Zero restitution stops the box and requires ideal and rounded persistent-contact
tracks. Positive restitution reverses and scales its normal velocity; both
paths must certify one-sided departure and a separated endpoint. A second step
from that endpoint must certify a clear path. Tangential incoming motion and
off-center contact return `Undecided` in this tilted-pose path.
At a horizontal partly overhanging source-box face, a centered frictionless
fixed/dynamic pair with zero restitution can stop against the exact clipped
polygon. The response checks omitted spin and requires an ideal and rounded
persistent-contact track. An off-center patch requiring spin returns
`Undecided`.
`Trace.Sample` also returns interior states on a persistent-contact path. The
rounded source-box poses must stay within the cached oblique sweep's point
resolution and keep its bounded face track; otherwise replay returns
`ErrUnsupported` (§7.1).
A source semicircle sphere with supplied or density-derived mass can rebound
from a fixed source box on an isolated face-point contact when its affine sweep
stays within that face corridor.
A centered sphere can also hit a fixed source box rotated 45 degrees about Y.
Its one-point manifold, exact affine first-impact bracket, mass and inertia
bounds, and frictionless positive-restitution response must all pass. The
solver bounds incoming normal speed on both sides of `ImpactSpeed` before
choosing restitution. An interval crossing that threshold returns
`Undecided`. The source-sphere response also bounds omitted angular speed,
contact-point speed, rotational energy, and full-step displacement. Its
angular-energy allowance is the dynamic mass upper bound times half the
squared `VelocityResidual`; full-step displacement must fit
`PointResolution`, `ContactSlop`, and `PenetrationResidual`. A supplied mass
center 2 mm off the source center with inertia 5e8 kg·mm² returns `Undecided`
(`TestRotatedSphereBoxOffCenterMassNeedsPointMotionProof`): the omitted point speed exceeds `VelocityResidual`,
although the omitted angular speed fits `AngularVelocityResidual`. With zero friction, the impulse
changes only the bounded contact-normal component of velocity. The incoming
tangent component continues through the impact. The response bounds normal
restitution error, tangent change from normal uncertainty, mass-interval
momentum error, and omitted spin against the configured residuals. It records
zero tangent impulse and corrects the shallow overlap to a bounded positive
gap. Ideal and rounded clear sweeps must certify the full rebound remainder.
`Trace.Sample` replays both sides from those cached rounded sweeps. Zero
restitution, edge/corner approach, skew read rotation, incoming spin, and
an unproved tangent corridor return `Undecided` in this path.
A full circular source prism or a full rectangular-profile source revolve can
rebound from a fixed source-box face with supplied or density-derived mass when
its affine sweep proves the first axial face impact and separating remainder.
The zero-spin response bounds omitted angular speed,
point speed over the complete source body, rotational energy, and travel over
the full step. A supplied center 5 mm off the source axis with inertia
1e8 kg·mm² fails the point-motion bound even when its angular speed meets
`AngularVelocityResidual` (`TestCylinderSlowOffCenterSpinUndecided`).
A vertical full circular source prism can also strike the side of a broad
fixed source box along its cylindrical wall. The line-contact proof supplies
one centered witness and an exact horizontal normal. A frictionless positive
restitution response uses density-derived mass, bounds omitted whole-body
spin travel, and requires ideal and rounded separating sweeps before its
trace replays the approach and rebound. A line reaching the box-face edge or
relative axial translation returns `Undecided` on this path.
A verified faceted Boolean union with one complete rectangular lower support
face can hit a fixed source-box floor vertically with supplied mass and zero
spin. Its four-point source-face manifold and exact affine first-impact
bracket enter the same centered frictionless response. Positive restitution
requires ideal and rounded separating sweeps; zero restitution requires ideal
and rounded persistent-contact tracks. The trace replays both impact sides
from those certificates. A mass-center bound that permits an omitted spin
above `AngularVelocityResidual` stops the step.
At zero restitution against one fixed body, the dynamic body's published
normal velocity equals the fixed body's exactly after the bounded response
check, so floating-point cancellation cannot turn rest into departure.
A translation-only placed copy of that zero-bound union may carry positive
held-mesh and occupied-volume bounds while preserving the exact lower face.
With density-derived bounded mass and center readings, zero spin, zero friction,
and positive restitution, the same fixed-floor response consumes its exact
axial impact bracket. Ideal and rounded departure sweeps certify the rebound;
the trace replays the approach, touch, and separating remainder. A placed
Boolean without the exact source-face proof returns `Undecided` at impact.
A mesh `Union` with positive boundary and occupied-volume bounds can also
rebound or rest on the fixed floor when its exact source-box lower face survives
an operand certified strictly above that face. Its four-point live faceted
manifold and vertical sweep enter the same response with density-derived mass.
The configured impulse and angular residuals must contain the mass and center
bounds; otherwise the step returns `Undecided`.
Two source semicircle spheres with supplied or density-derived mass can
rebound as a centered dynamic pair when the impact has a cardinal manifold.
Their approach may
include transverse motion when the sweep certifies first impact.
At an initial cardinal touch with zero restitution, publish one shared
mass-weighted normal velocity only when the bounded pair impulse and both
response intervals admit it. The ideal and rounded persistent-contact sweeps
must still certify the full step.
An isolated frictionless off-axis sphere-pair impact also admits a bounded
center-line normal with density-derived or supplied bounded mass and center
readings when neither body starts with spin. The solver checks both mass
interval endpoints against the impulse law and bounds torque from the contact
witness, center, and normal intervals against `AngularVelocityResidual`.
The center restitution error plus both omitted contact-point speeds must fit
one `VelocityResidual` budget. Both omitted rotational energies share the
impulse-times-remaining-residual allowance, and their full-step point travel
shares one `PointResolution` budget.
The impulse changes both velocity vectors along that normal. A small
separated position correction within the certified
bracket and contact slop is allowed when a full clear remainder sweep proves
the pair cannot meet again. Unresolved tangency or a noncentral mass stops the
step when its angular response exceeds that residual.
At zero restitution, an interior first-impact bracket may leave zero
normal departure speed. The response admits that speed within
`VelocityResidual`, including one exactly shared velocity when the bounded
impulse, momentum, and energy checks admit it. The bounded correction must
produce a separated pair, and both ideal and rounded remainder sweeps must
certify `SweepClear`. The trace replays the rounded impact prefix and clear
remainder; a failed correction or sweep returns `Undecided` without a state.
When two dynamic source spheres start at one certified off-axis point touch,
have zero spin and friction, and have bounded closing normal speed, the
bounded two-body impulse admits positive or zero restitution. The event has
time zero and a zero-width bracket at the initial `SweepInstant`. No position
correction is made. A positive-restitution response needs ideal and rounded
`SweepDepartedClear` paths and a separated endpoint. At zero restitution,
the response may certify the same departure through tangential relative
motion or a full-span `SweepPersistentTouch` with a touching endpoint. A
resting response may set both post velocities to one exactly equal value only
when their bounded impulse, momentum, energy, and contact-point residuals
admit that value. Both paths use the real ideal drift and rounded endpoint
sweep. A missing initial float witness, a failed response bound, or an
unproved continuation returns `Undecided` with no event or `Next`.
An initial cardinal X point touch between two dynamic source spheres can
also receive a positive-restitution one-point impulse with zero or positive
friction.
Both source centers, exact supplied mass centers, and query translations
must coincide; both exact inertia tensors must be positive and isotropic.
The pair starts without spin or Z velocity, and the effective friction
coefficient must have equal exact bounds. The rational response first solves
the normal restitution impulse, then applies the smaller of the tangent
sticking impulse and the Coulomb limit. It rounds and checks both linear
velocities, both Z spins, restitution, tangent slip or sliding direction,
the friction cone, and each point impulse against configured residuals.
Zero friction leaves the tangent velocities and spin unchanged. A zero-friction,
zero-restitution pair uses the existing response. The event records equal
and opposite linear impulses and both spin responses. The
rotating `SweepDepartedClear` report must prove ideal and rounded separation
at its endpoint and certify trace replay.
A planar off-axis source-sphere pair can use the same positive-restitution
Coulomb law when its exact center offset has nonzero X and Y components, Z is
zero, and its squared length equals the squared sum of its source radii.
The exact offset divided by that radius sum selects the normal; the bounded
ideal and rounded witnesses must enclose the exact point and normal. Both
source sphere centers and supplied mass centers coincide, both exact inertia
tensors are isotropic, and both incoming spins and Z velocities are zero.
The tangent is the in-plane perpendicular to the normal. Solve rational normal
and tangent impulses, update both linear velocities and Z spins, then check
restitution, Coulomb cone, stick or slide, and rounding residuals before
publishing the rotating separating sweep, event, and trace.
A separated source-sphere pair can enter either response at an interior impact. The event lies at the
real `SweepImpactBracket`'s right fraction, at most one `TimeResolution` after the exact contact (Part N §5), and
the pair's own report covers the prefix up to it. The solve reads the contact at that fraction, then the
position correction removes the shallow overlap the rounded poses show (Part N §6.6). Both bodies must have exact
centered isotropic supplied mass, zero initial spin, positive restitution, and an exact effective friction
coefficient. The contact response uses the remaining duration and its rotating
departure sweep. The event retains the original bracket; the trace replays the prefix and remainder from
separate cached proofs. A failed correction or unproved continuation returns `Undecided` without an event or
state.
An initially touching off-axis pair with exactly shared linear velocity and
zero spin keeps a full-span certified persistent track without an impulse,
including when effective friction is positive. Other positive-friction
sphere-pair inputs outside these paths return `Undecided`.
A two-body world's pair may be excluded; its bodies drift independently even
through overlap, and no pair material is mixed.
Three-body fixtures cover one dynamic body and two fixed bodies, two dynamic
bodies and one fixed body, and three dynamic bodies: sequential impacts, a
centered stack, an isolated zero-restitution sphere impact and simultaneous
sphere islands, each resolved by the general step. An excluded pair is not
queried and does not mix material.
This first two-dynamic path returns `Undecided` for other initial contacts and
nonzero angular velocity.
Three dynamic source spheres also resolve one symmetric simultaneous
positive-friction island at the initial time. Body zero touches body one on
its positive X side and body two on its positive Y side; the outer pair has
a certified clear full-step path. All three use equal exact centered isotropic
supplied masses and equal source radii. They start without spin or Z motion.
After the full-step force kick, the outer spheres are still, and body zero
has equal positive X and Y speeds. Both exact one-point manifolds have zero
normal and witness bounds. The effective materials have equal exact positive
friction, equal restitution strictly between zero and one, and a sticking
solution inside both Coulomb cones.
For mass `m`, inertia `I`, radius `r`, closing speed `v`, and restitution
`e`, the two equal normal impulses `N` and tangent impulses `T` satisfy
`2N+T=m(1+e)v` and `N+(2+mr²/I)T=mv`. The step checks rounded per-body
linear and angular impulse equations, both restitution speeds, both zero-slip
point speeds, and whole-island kinetic energy. It records two ordered events
for the simultaneous solve; `MaxEvents` must exceed two. The two active
rotating sphere-pair sweeps must certify departure, and the rotating outer
pair sweep must certify clear motion. Their replay certificates must agree
exactly on each shared body pose. The trace uses those three certificates.
On a later step, three exact centered source spheres with admitted supplied
mass can continue spinning without another contact when all three real pair
sweeps certify clear motion and their replay endpoints agree on every body.
The clear step records no event and keeps each kicked linear and angular
velocity while the three rounded certificates replay the complete drift.
Asymmetric speeds, unequal masses or radii, sliding impulses, and unproved
pair paths return `Undecided` without a next state.
Two dynamic source boxes can start in a centered vertical stack on a fixed
source-box floor. Both face pairs must start at certified four-point contact,
and the floor-to-upper pair must be clear. After the single full-step force
kick, both boxes must move strictly downward without lateral velocity or spin.
Both contact pairs have zero effective friction and restitution. The
floor-to-lower and lower-to-upper normals must point upward in physical stack
order.
Let `mL`, `mU` be the two mass readings and `vL`, `vU` their kicked vertical
velocities. The simultaneous normal impulses are `JU = −mU vU` from lower to
upper and `JF = −mL vL + JU` from floor to lower. The published zero final
velocities and both impulses must pass the mass-interval and impulse residual
checks. Four equal point impulses per face must pass the bounded angular
residual and whole-step omitted-spin travel checks. Two initial-time events
share one coupled response; their intermediate velocity records account for
each impulse, while the energy check applies to the full coupled result.
With time remaining, `MaxEvents` must exceed two. Both ideal and rounded
face paths must report full-span persistent contact with bounded manifolds;
the floor-to-upper path must remain clear. Endpoint `ContactPair` calls must
confirm both touches and the gap. `Trace.Sample` replays all three rounded
pair sweeps. A second gravity step repeats the same response from the resting
state. Other source-box stacks, lateral motion, spin, positive friction or
restitution remain unsupported.
With one dynamic body, it sweeps each non-excluded dynamic/fixed pair. One active pair advances only
when every other pair has a certified clear path through the response. Two
strictly ordered, frictionless positive-restitution source-box face impacts can advance within one
step. The full-duration force kick occurs once. Each impact restarts both pair
sweeps, and every drift slice stores both rounded pair certificates for replay.
An overlapping later impact bracket remains `Undecided`. Two
simultaneous initial face contacts also advance when their certified normals
lie on distinct coordinate axes, both effective pair friction coefficients
are zero, and the fixed/fixed pair is separated or excluded. It applies each
zero-restitution normal impulse to the shared dynamic body, bounds their
combined omitted spin and whole-step point travel, and certifies both ideal
and rounded persistent tracks through the endpoint. Resting constraints may
receive zero impulse. Each impulse produces an ordered event; the step reports
their summed contact impulse and the dynamic body's conservation readings.
An initial source sphere touching a fixed source-box face and a fixed source
sphere also advances when the face normal is cardinal, the sphere-pair normal
is oblique, and both pairs have one bounded point. Both effective pair
coefficients are zero. The two-normal solve checks all four active sets; a
positive impulse requires zero post-contact normal speed within the velocity
and normal bounds, while a zero impulse requires nonclosing speed. The shared
mass interval, both normal bounds, and both omitted-spin bounds must fit the
response residuals. Both torque bounds act on one body: add their angular
speed bounds before squaring against its common inertia ceiling. Each active
pair needs ideal and rounded full-span
persistent-contact tracks. A separating pair needs ideal and rounded
one-sided departure, and each endpoint must retain its certified relation.
The trace checks both rounded pair certificates at every interior sample and
retains the after-kick velocity when no contact impulse occurs.
The two ordered events describe arithmetic stages of one simultaneous solve.
The combined momentum and kinetic-energy checks cover both impulses together;
an intermediate stage can gain energy while the completed island loses it.
An initial source sphere touching two fixed, orthogonal source-box faces can also
resolve two positive-friction point impulses together. Both fixed poses are
identity, the sphere has exact centered isotropic mass, and the two exact
normals point along positive X and Z toward its center. It starts without spin,
closes along both normals, and slips along Y. Both pairs have zero restitution
and exact positive friction. The joint sticking solution uses the same linear
mass and spherical inertia for both contacts. Each normal impulse stops its
closing component, while both Y impulses share the Y speed and create X and Z
spin. It accepts the solution only when each impulse fits its Coulomb disk,
both rounded contact-point speeds fit `VelocityResidual`, and the combined
linear, angular, and energy readings fit their configured residuals. Each
rotating sweep certifies its ideal persistent track and rounded pose replay.
Both certificates must yield the same exact endpoint sphere pose, which the
step publishes. The trace replays both rounded paths. A sliding joint
solution, later simultaneous impacts, and other coupled normals return
`Undecided`. An excluded pair may overlap or cross without a contact
event or material mixing.
At an initial face touch, a fixed floor and dynamic source box can receive a
full-step kick from gravity and any center force, then a zero-restitution
support impulse. A zero-restitution impact can continue as certified
persistent contact, and a stationary touching pair can advance without an
impulse. These paths require full-span ideal and rounded contact
tracks. Each dynamic body accepts at most one center force and world-frame
torque; mass and inertia intervals must fit the kicked velocities within their
respective velocity residuals.
An initially touching pair with exactly zero relative normal speed can slide
without an impulse when both full-span paths certify persistent contact. The
first edge-exit step accepts a transition whose right bracket sample is
separated, then certifies a clear remainder. A transition with a touching
right sample or insufficient `MaxEvents` remains `Undecided`.
An affine translating kinematic driver can move through a certified clear
step, push an initially touching dynamic box with a zero-restitution response,
depart from an initially touching box without an impulse, or cause a centered
interior impact. Departure requires ideal and rounded full-path certificates;
a zero-restitution response requires certified persistent touch.
An axis-screw rotating driver can cross a centered source-box face and
rebound the dynamic box when its oriented contact witness and separating
remainder certify the step. Its event records the contact-point derivative,
while the prescribed body's stored velocity stays zero.
`World.Step` admits a positive effective pair friction coefficient
for a fixed floor and a dynamic source box in either world order. An initial
four-corner face touch
with zero incoming spin, positive X slip, and closing Z speed can use the joint
Coulomb solver. The response publishes zero Y/Z velocity, bounded corner and
aggregate impulses, a residual report, and a persistent-touch trace after
both ideal and rounded sweeps certify the full remainder. Zero X slip uses
the centered normal support path with zero tangent impulse.
The floor stays at identity placement. The box can start from any pure
translation whose real four-corner track passes the same bounds, including
the endpoint of a previous step. Its mass center is translated with exact
rational coordinate sums for the lever and torque certificates.
A centered source sphere can instead receive one point impulse on an
identity-placed fixed box floor when the source sphere center is its query
origin, mass and isotropic inertia have exact zero bounds, restitution is zero,
and the positive pair friction coefficient has exact equal bounds. The
initial contact may have incoming Y spin. A full-step gravity kick must give
negative Z speed, with Y linear speed and X/Z spin zero. The normal impulse
stops Z motion. The tangent impulse opposes X contact-point slip and is the
smaller of the exact sticking impulse and the Coulomb limit. Check its rounded
linear and angular results against their configured residuals and the cone.
Publish the Y spin and one point impulse in world order. A rotating source
sphere sweep must prove the full persistent contact track and replayable
endpoint before the step advances. A later step can reuse the returned spin.
The same point response admits an interior first-impact bracket for a
translated, nonspinning source sphere with X velocity, negative Z velocity,
and zero Y velocity. The sweep and a separate rounded prefix must identify
the same sphere and floor features. Correct the bracket-right pose only within
the certified normal travel and `ContactSlop`, then query `ContactPair` for a
touching point with those features. The exact mass, isotropic inertia,
restitution, friction, and point gates above still apply. The one-point
normal and Coulomb impulses stop Z velocity and change X velocity and Y spin.
The rotating sphere sweep must certify persistent contact and its replayable
endpoint for the remaining time. `Trace.Sample` uses the rounded prefix before
the event and that rotating certificate afterward. A bracket that cannot be
corrected to touch, a changed contact feature, insufficient event budget, or
a failed remainder returns `Undecided` without a next state.
An interior first-impact bracket admits the same centered fixed-floor box with
zero restitution, positive X and negative Z velocity, zero initial spin, and
zero Y velocity. Correct the bracket's touching pose within the certified
penetration allowance, then use its real four-corner manifold. A symmetric
pressure distribution gives positive point normal impulses and tangential
impulses within each Coulomb cone. It shifts the normal pressure toward the
direction of motion so the friction torque and normal torque cancel. The
rational mass, point, impulse, and torque certificate must bound zero outgoing
spin and the outgoing X velocity; the response can stick or keep positive X
slip. Both ideal and rounded full-span persistent tracks must certify the
remainder, and the event's rounded prefix must certify interior trace replay.
Positive restitution, a nonrectangular or off-center patch, insufficient
event budget, or a failed response or path bound returns `Undecided`.
An initial horizontal fixed/dynamic source-box patch can resolve a frictionless
off-center impact when supplied mass puts the dynamic center beyond one patch
edge. Two symmetric vertices on that edge receive the normal impulse; the
other vertices must separate at or above the restitution target. The bounded
linear and angular response, event conservation, rotating departure sweep,
and interior trace replay must pass. A centered mass at the patch edge, a
mass or inertia interval exceeding the configured residual, and a remainder
without certified departure return `Undecided`.
An initial two-dynamic face impact can publish bounded spin when its rotational
departure sweep certifies the full remainder. The step reports bounded
kinetic energy, linear momentum, and angular momentum for dynamic bodies
at input, after the full-step force kick, and at completion. It also reports
gravity, center-force, and fixed/kinematic contact impulses separately, plus
the drift-only energy, linear momentum, and angular momentum changes.
Torque-driven rotation advances when `SweepPair` certifies a clear full-span
drift and its rounded endpoint. Its trace replays clear interior poses against
the cached rotating source-box proof. A source box spinning about world Z can also
rebound from a wide fixed horizontal source-box face with positive
restitution and zero friction. Its normal impulse leaves the admitted spin
unchanged within `AngularVelocityResidual`; the returned pose uses the
certified rotating departure sweep. A rotating contact that lacks these
proofs returns `Undecided`. Contacts outside the paths above go to Part N's islands (§6) or return
`Undecided`; `agent-docs/collision-v1-support.md` lists what ships.

Navigation only; the named sections own the rules:

| Question | Section |
|---|---|
| Which bodies and state values can enter a world? | World and State |
| Which loads, drivers, and limits does a step take? | Step input and configuration |
| How does time advance to the next event? | Step and event schedule |
| How are impact, friction, and resting contact solved? | Response |
| What does failure return, and how is motion replayed? | Completion, conservation, and trace |
| Which computed fixtures gate implementation? | Verification |

### World and State

`World` copies one document's ordered rigid-body definitions, pair exclusions,
material overrides, and `StepConfig`. It stores no changing pose or velocity.
Construction rejects a nil/duplicate/foreign/retired/non-solid body. It checks
each body's liveness again at every step. A fixed or kinematic body needs no
mass properties; a dynamic body needs one of these complete sources:

- A positive exact density → call `Body.MassProperties(ctx, density)` once and
  cache its bounded mass, center, and six inertia components.
- A supplied `decad.MassProperties` → use the same bounded shapes. Its center
  and tensor refer to the body's committed world placement, with the tensor
  about that center in committed world axes. Reject a partial supplied record.

The density and supplied modes are exclusive. World construction copies the
selected input, including a supplied record, so later caller edits do not
change a world. It rejects an unknown exactness value and an `Exact` reading
with a nonzero bound. World construction requires
mass's lower bound to be positive and a certified positive lower eigenvalue of
the complete inertia interval tensor. A nominal positive determinant is
insufficient. Each of the six components enters as the exact rational
interval `value ± bound`. Either of two sound proofs certifies the tensor:
row dominance, whose floor is the smallest diagonal lower end less that row's
largest product magnitudes; or Sylvester's criterion, where each leading
principal minor evaluated in rational interval arithmetic has a strictly
positive lower end and the floor is `4·det_lo/trace_hi²`, since
`λ_min = det/(λ_mid·λ_max)` and `λ_mid·λ_max ≤ (trace/2)²`. The larger floor
is the certified eigenvalue. The minors admit an elongated body at an oblique
placement, whose products of inertia defeat row dominance. Reject
inconsistent bounds, a tensor neither proof establishes, or a floor whose
reciprocal is not a finite `float64` as `ErrInvalidMassProperties`. The mass design owns the
integrals and their bounds; `r3` owns symmetric-tensor rotation and inversion.
At a state pose, transform the certified center and tensor from the committed
placement into world coordinates. Recompute world inverse inertia after every
change of orientation.

```go
type BodyRole int // Fixed, Kinematic, Dynamic

type QuantityVec = decad.QuantityVec
// Each use validates all components against its required Kind and finiteness.

type Material struct {
    Restitution units.Value // Dimensionless, [0, 1]
    Friction    units.Value // Dimensionless, >= 0
}

type RigidBody struct {
    Body       *decad.Body
    Role       BodyRole
    Material   Material
    Density    *units.Value         // exactly one mass source for Dynamic
    Supplied   *decad.MassProperties
}

type BodyPair struct { A, B *decad.Body }
type PairMaterial struct { Pair BodyPair; Restitution, Friction units.Value }

type WorldConfig struct {
    Bodies    []RigidBody
    Excluded  []BodyPair
    Overrides []PairMaterial
    Step      StepConfig
}

func NewWorld(ctx context.Context, doc *decad.Document, cfg WorldConfig) (*World, error)
func (w *World) NewState(entries []BodyState) (State, error)

type BodyState struct {
    Body            *decad.Body
    Pose            r3.Transform // composes onto committed placement
    LinearVelocity  QuantityVec  // world Velocity
    AngularVelocity QuantityVec  // world Angle / Time
}
```

`NewState` requires exactly one entry for each world body, in any caller order;
it stores them in world insertion order and binds the state to that world.
`State` provides read-only, copy-returning accessors. `Step` rejects a state
from another world. Every pose must be valid and orientation-preserving:
`r3.Transform.IsValid()` true and `IsReflection()` false. `NewState` rejects
fixed and kinematic entries with nonzero velocity. A fixed pose is constant;
a kinematic pose comes from its step driver, and its stored velocity stays
zero. A dynamic pose and both velocity vectors must be finite. World order
fixes pair enumeration, event order, and trace order. The complete excluded
pair list appears in `World` inspection and each step report; an override for
an excluded pair is an input error.

The effective pair restitution is the smaller body's coefficient; pair
friction is the geometric mean. An explicit `PairMaterial` replaces both
values for that pair. Pair names are canonicalized by world order. Duplicate
or unknown exclusions/overrides fail construction. A pair with neither body
dynamic can still be checked when a kinematic driver moves; it cannot be
resolved by an impulse if closing contact occurs.

`NewWorld` accepts two or more bodies and holds every body pair, with its
exclusion and effective material, in a canonical pair table
(Part N §3.1), in any role mix. Every world takes
the scheduled step of Part N §5, which drifts from
event to event and solves the contacts at each event time as certified Coulomb
islands (§6).
The typed `StepDiagnostic` fields are Part N §12.
Every pair of a world can be excluded or given one material override. `NewWorld`
rejects a pair naming a body outside the world, including nil or repeated
bodies, with `ErrInvalidInput`. It also rejects a second override or exclusion
for the same pair, including reverse order. An override for an excluded pair
returns `ErrInvalidInput`. An excluded pair skips effective material mixing;
each body's material still passes input validation. Held exclusions and
reported event pairs use world order. `World.Excluded()` and
`StepReport.Excluded` return separate copies.

The positive-friction paths above cover a fixed floor and dynamic box or
source sphere in either world order, two dynamic source boxes at an initial
opposed face touch, and the initial and interior sphere pairs. Without an
override, the pair coefficient is the geometric
mean of the held body coefficients. The solver proposes impulses with a nominal
rounded mean, then checks the friction cone and slip law against rational
bounds that enclose the exact mean. A zero body coefficient selects the
frictionless response. A positive mean below the smallest positive float64 or
above the largest finite float64 returns `ErrUnsupported` at `NewWorld` for a
pair with a non-fixed body; a fixed/fixed pair enters no response, so its mean
is not refused.
With an override, its exact held coefficient replaces
the body values; a positive override goes to the patch solver even when both
body coefficients are zero. A zero override selects the frictionless response
even when the body coefficients differ. `NewWorld` returns `ErrUnsupported` for a
positive-friction pair that is not excluded and has a kinematic body. Any other positive-friction pair is
decided when the step reaches its contact: the generic Coulomb gates certify it from the pair's manifold, or the
step stops with a typed diagnostic (Part N §6.4).
The fixed-body patch solver reads a floor-to-box witness in both orders. The
two-dynamic solver applies equal and opposite impulses through both masses
and inertias, then bounds each body's angular response. A nonzero outgoing
spin requires a certified rotational remainder before the step publishes it.
Each event keeps its original world-order manifold. Its normal impulse is nonnegative,
and its tangent and point impulses describe the impulse on world-order B.

### Step input and configuration

```go
type StepConfig struct {
    Contact             decad.ContactRequest
    TimeResolution      units.Value // positive Time
    ContactSlop         units.Value // nonnegative Length
    VelocityResidual    units.Value // positive Velocity
    AngularVelocityResidual units.Value // positive AngularVelocity
    ImpulseResidual     units.Value // positive Impulse
    PenetrationResidual units.Value // positive Length
    ImpactSpeed         units.Value // nonnegative Velocity
    MaxPoseEvaluations  uint64      // >= 2, per pair sweep
    MaxIterations       int         // positive, per event
    MaxEvents           int         // positive, per step
    MaxPairSweeps       uint64      // positive, SweptBox and SweepPair calls per step
}

type BodyLoad struct {
    Body   *decad.Body
    Force  QuantityVec // world Force, at center of mass
    Torque QuantityVec // world Torque, about center of mass
}

type KinematicDriver struct {
    Body *decad.Body
    Path decad.PairPath // defined over the entire dt
}

type StepInput struct {
    Gravity QuantityVec // world Acceleration
    Loads   []BodyLoad
    Drivers []KinematicDriver
}

func (w *World) Step(ctx context.Context, from State,
    input StepInput, dt units.Value) (*StepReport, error)
```

The pinned `units` dependency provides Time and the composed kinds used above,
including AngularVelocity, Force, Torque, and Impulse, with named units for
public input.
All scalar public values use `units.Value`; no configuration tolerance or
material coefficient is implicit. Reject wrong kinds, non-finite values,
negative slop or impact speed, nonpositive resolutions or residual limits,
and nonpositive limits. `MaxPoseEvaluations < 2` is invalid. `dt` is positive
Time. A pair sweep uses `min(TimeResolution, remaining span)` as its time
resolution and passes `MaxPoseEvaluations` to `decad.SweepRequest`.

`Loads` has at most one entry per dynamic body. Missing loads mean zero;
loads on fixed or kinematic bodies are invalid. Gravity applies to every
dynamic body; force and torque are held constant in world coordinates for
this step. A force acts at the current center of mass. The caller converts
an off-center force to force plus torque. `Drivers` has exactly one path per
kinematic body and no path for another role. Each driver's duration equals
`dt`, its start pose equals the state's pose, and its start/end poses are
proper rotations. The driver supplies contact-point velocity as the
path's time derivative, with its certified numerical bound; absent or
unbounded derivatives make a potential contact undecided. It receives no
impulse. A fixed body has zero contact-point velocity.

The step accepts one typed, finite center force and torque per dynamic body.
A nil, duplicate, foreign, or nondynamic load body or a wrong-kind/non-finite
vector returns `ErrInvalidInput`. It bounds the linear kick over the admitted
mass interval and the angular kick over every admitted inertia component. A
published kick outside `VelocityResidual` or `AngularVelocityResidual` returns
`Undecided`.

The kinematic slice admits one `PoseSegment` with either constant orientation
and an exactly representable affine derivative or a cardinal read screw axis
with exactly representable full-step linear and angular rates. Its start and
duration must exactly match the state and step. A missing, duplicate, or
nonkinematic driver is `ErrInvalidInput`; a rotating path outside this proof or
a derivative that cannot meet the full-step speed proof is `ErrUnsupported`. An initially
touching, closing kinematic/dynamic pair can receive a zero-restitution
support impulse. An initially touching pair with strictly separating relative
normal speed can advance without an event only when the initial manifold is
bounded, both full-step sweeps certify departure, and the endpoint is separated.
This path does not consume `MaxEvents`. A centered interior impact can use
the bounded bracket-right sample and correct only the dynamic pose. A
separating response needs ideal and rounded sliced paths that certify
departure. A zero-restitution response
needs full-span bounded persistent-contact tracks on both sliced paths and a
bounded touching endpoint. A sliced affine driver derivative must equal the
admitted full-step derivative exactly.
Other contact schedules remain `Undecided`. The driver's effective contact
speed appears in the event, while `State` and `Trace` store zero kinematic
velocity. A separated pair can complete a clear driver path without an event.
For a cardinal rotating screw, the collision uses the full path's axial
translation rate at an axis-normal face. A bounded one-point oriented manifold
can supply an interior impact. The event's driver velocity is the screw's
linear derivative plus `ω×(point−axisPoint)` at that witness; the dynamic
body receives the impulse and the kinematic body receives none. A sliced
remainder may differ from the full-step read screw only within the configured
linear and angular velocity residuals. Its axis line must lie within
`PointResolution`, and its contact-point velocity must agree within
`VelocityResidual`. The original screw's support-plane gap must increase
strictly after the impact. Both ideal and rounded sliced sweeps must certify
departure. A rotated persistent contact, off-axis normal, or
unbounded contact-point derivative returns `Undecided`.

`units.Angle` is dimensionally distinct from `units.Dimensionless`. At the
physics boundary, validate every AngularVelocity component as Angle/Time,
convert its base-unit radians per second to an internal scalar, then use
`r3` vector operations for `ω×r`, torque, and `Iω`. Convert the result back
to Angle/Time before publishing state. The same explicit radian-to-scalar
rule applies to angular acceleration and angular impulse. Do not use
`Kind.Mul` to treat Angle/Time × Length as Velocity. Internal numeric vectors
carry a declared physical kind; conversion cannot silently change one.

### Step and event schedule

`Step` uses one semi-implicit force kick for the complete `dt`, then an
event-driven drift. It evaluates each dynamic body's world acceleration at
the initial state: `a = g + F/m` and `α = I_world^-1(τ − ω×I_world ω)`.
The gyroscopic term uses the initial orientation and radian scalar angular
speed. Set the kicked velocities to `v + a dt` and `ω + α dt` once. The
step's discrete force law is this kick; contact-event restarts do not repeat
it. Report the applied force and torque impulses. An overflow, an invalid
updated tensor, or a non-finite kicked velocity is undecided with no state.
Every event's pre-impact velocity is the kicked velocity plus impulses from
earlier events in this step. It is not a claim about velocity under a
continuous force law at that physical time.

For each remaining interval, construct a `decad.RigidDriftSegment` for each
dynamic body from its current pose, world center of mass, kicked/current
linear and angular velocity, and remaining duration. Its ideal center follows
`c(t)=c0+v t`; its orientation follows a world-axis rotation at constant
`ω`. This path is not generally the read screw between endpoint poses when
`v` has a component perpendicular to `ω`. `SweepPair` must consume the real
drift segment and certify that trajectory. Kinematic bodies retain their
original whole-step paths and are evaluated on the same elapsed-time slice;
fixed bodies use constant paths. All path poses compose onto each body's
committed placement. The geometry query's pair order is world order.

Sweep all non-excluded pairs that have a moving body. Do not discard a pair
from endpoint samples. New pairs use `StopAtInitialContact`. A pair whose
contact was just solved uses `ContinueSeparatingTouch` when every solved
normal speed exceeds `VelocityResidual`; otherwise it uses
`ContinueCertifiedTouch`. These policies require the one-sided departure
or persistent-touch proof owned by contact-sweep §5.1. They never skip an
initial touch by sampling at a positive time. `SweepDepartedClear` permits
the rest of the drift. `SweepPersistentTouch` permits the rest only when
the solver's normal/friction residuals stay within limits throughout its
`ContactTrack`. The first source-box implementation checks the track's
affine patch and constant normal/velocity; unsupported varying tracks are
undecided. `SweepContactTransitionBracket` stops the drift at the first
feature or patch change. It is a solver restart, not necessarily an impulse.

When the one world pair is excluded, validate input, driver, and force kick as
usual. Drift each dynamic body independently; put a kinematic body at its
driver endpoint. Do not call `SweepPair` or `ContactPair` for that pair, even
when its source solids overlap or cross. Publish no contact event and typed
zero contact impulse and kinematic work. The trace and conservation readings
still describe the completed step.

Every world sweeps its candidate pairs through the broad phase of
Part N §4.3 and counts every event against one
`MaxEvents` limit. A dynamic body outside a responding pair drifts and replays
through every slice, and its mass, loads, energy and momentum enter the world
report. A dynamic/dynamic impulse contributes zero to total external contact
impulse; a fixed/dynamic impulse contributes its signed value.

A `SweepPair` `Undecided` that could precede the next event makes the step
`Undecided`; a later undecided interval can be revisited after an earlier
event changes the drift. `Clear` permits advancement across its whole span.
`InitiallyTouching` enters the contact set if closing or resting; separating
touch receives no impulse and needs a continuation proof before advancement.
`InitiallyOverlapping` is accepted only within the correction allowance
below. `ImpactBracket` provides a proven-clear lower time and a contact upper
time, not an exact impact time. Compare impact and contact-transition brackets
in world order, choose the earliest upper time, and gather every event whose
uncertainty interval overlaps that time.
The solver consumes each `SweepEvent.Manifold`, whose bounds include the
path-pose transfer.
`ContactPair` at the chosen float poses can refine a missing manifold only
if the sweep transfers its bounds to the ideal path. If any gathered pair
still has no adequate manifold, stop undecided. Recheck any pair whose bracket
starts before the chosen time but whose upper time lies later; do not advance
past a possible earlier contact. `MaxEvents` counts impacts and contact
transitions and grazing touches. Reaching it with remaining time returns
`Undecided` and no `Next`.

#### Grazing touch

A pair whose sweep reports `SweepGrazingTouch` inside a slice publishes a
`ContactGraze` event without cutting the slice
(Part N §5): its sweep replays the whole slice,
through the touch and the separation after it. The pair must be frictionless,
its manifold must lie within `StepConfig.Contact`, its poses at the graze are
the ones its sweep replays, and its relative contact-point normal speed,
enclosed over the bounded event normal, must lie within `VelocityResidual` of
zero; a nominal zero alone cannot pass. A kinematic participant must
translate. Any other graze is `Undecided` with `StepUnsupported`.

The event carries the bounded manifold, the exact event time, a zero-width
`Bracket` at `SweepReport.Event.At`, typed zero normal, tangent and point
impulses, and a nil `Solver`. Its pre and post velocities and poses are
identical and its position changes are zero. It applies no impulse or
correction, counts once against `MaxEvents`, and adds nothing to the
conservation report's contact impulse.

Advance poses along the same certified paths to the chosen upper time. That
time is the numerical event time; the report retains the original bracket.
Every bracket's travel uncertainty contributes to the allowed penetration
and correction. If another pair impacts before a persistent track ends,
advance every body and sample that track's manifold at the earlier time.
Include its active constraints in the simultaneous solve, then discard the
unused track suffix and sweep all pairs under the new velocities. At a
contact-transition bracket, advance to the chosen time, recheck contact
features, and solve only constraints now closing or resting; the transition
itself applies no impulse. After solving, rebuild each remaining dynamic
drift from the new pose and velocity, and sweep again. No minimum time
quantum may silently skip a persistent contact: a zero-time repeat enters
the resting contact solve or stops undecided at `MaxEvents`. On an event-free
remainder, advance to `dt`. Run `ContactPair` on every active contact at
the completed pose and reject penetration beyond the configured residual.

### Response

At a contact point let `n` point from A to B; let `rA` and `rB` run from
each world center of mass to the geometry-certified point. Use the bounded
manifold's representative point and normal only when its point and angular
bounds meet `StepConfig.Contact`. A source feature may supply several
required normal constraints; solve them together in stable feature order.
Never derive a normal from overlap volume or a render mesh.

The relative velocity is
`u = (vB + ωB×rB) − (vA + ωA×rA)`. Closing means `u·n < 0`.
For one isolated frictionless contact, let
`K = mA^-1 + mB^-1 + (rA×n)·I_A^-1(rA×n) +
(rB×n)·I_B^-1(rB×n)`. Fixed and kinematic inverse mass and inertia are
zero. If pre-impact `u·n < −ImpactSpeed`, the target post-impact normal
speed is `−e(u·n)`; otherwise its target is zero. The isolated impulse is
`Jn=max(0,(target−u·n)/K)`. Apply `−Jn n` to A and `+Jn n` to B, with angular
impulses `r×J`. Reject `K <= 0` for a closing pair. Separating contacts
with no resting constraint receive zero impulse.

For simultaneous points, solve normal complementarity and the Coulomb disk
jointly, rather than applying the isolated formula point by point. Each
normal impulse is nonnegative. Each post-solve normal speed is at least its
target; an impulse above zero requires speed equal to the target within
`VelocityResidual`. Two orthogonal tangential components satisfy
`||J_t|| <= μ J_n`; sticking requires near-zero tangential velocity, while
slipping requires friction opposite tangent motion within the same velocity
gate. Construct the tangent basis deterministically from the least-aligned
world axis and `n`. Use a fixed-order projected iterative solve over the
whole contact island. Process islands, pairs, and manifold features in world
and source-feature order. Warm-start only from the input state's immutable
contact cache, keyed by body pair and certified source features; do not
key by topology slice index. Restart only an island whose whole problem
matches a cached one exactly, from that island's final proposal state, so
the warm start never changes a published impulse
(Part N §6.2).

Publish the maximum normal-velocity, tangent-velocity, friction-cone
(Impulse kind), and penetration residuals, the witness torque and spin, plus
iteration count, for each island. `VelocityResidual`, `ImpulseResidual`, and
`PenetrationResidual` gate success. If a residual exceeds its limit when
`MaxIterations` is reached, return `Undecided`. Use no random ordering or
solver tolerance hidden from `StepConfig`. Island formation, the proposal
order, and the exact-interval statement of every gate above are
Part N §6. Resting contact uses zero
restitution, persists in the state's contact set, and passes the same gate every
step. The initial off-axis sphere-pair impact above also admits zero
restitution when its full continuation is certified.
A closing contact with no dynamic participant is undecided because
its prescribed trajectory cannot receive an impulse.
Evaluate each residual over the admitted mass/inertia, contact-point, and
normal bounds; a nominal solution whose uncertainty can exceed a limit is
`Undecided`. Exact source boxes and analytic mass can make these bounds
narrow; the arithmetic residual still applies. The angular law's limit also
carries each contact point's witness ball times its impulse, the witness
torque, which the island publishes as `WitnessTorque` with its spin
equivalent `WitnessSpin`; the mass-center and normal balls stay in the
residual (Part N §6.3).

The fixed/dynamic frictional step uses the identity-placed floor's real
four-corner manifold, exact-rational impulse and torque sums, and the
mass/inertia bounds. It checks that the entire body cannot expose a larger
contact-point lever than the solver audited at the contact corners. It also
bounds omitted-spin travel over the full step. A narrow floor patch, an
off-center interior frictional impact, a rotating state, and a contact transition
return `Undecided`.

The two-dynamic solver uses the same real four-point manifold but includes
both inverse inertias and both mass intervals in each impulse and residual.
For zero friction and zero incoming tangent speed, it proposes equal normal
impulses at all four points. It publishes only when every point reaches its
restitution target within `VelocityResidual`, the bounded angular laws pass,
and the rotational remainder proves departure.
It publishes neither a completed state nor an event when the solved outgoing
spin lacks a certified rotational remainder. The corresponding `World.Step`
returns `Undecided` with `Next == nil`.

The fixed/dynamic off-center solver accepts a horizontal four-point patch,
fixed body A, dynamic body B, zero incoming spin and tangent velocity, zero
friction, positive restitution, and identity poses. The dynamic mass center
must lie strictly beyond the patch's positive-X edge. Two vertices on that
edge must be symmetric about the mass center's Y coordinate. The solver splits
one normal impulse equally between them, bounds the complete supplied mass
and inertia response, checks the restitution target at both active vertices,
and checks nonclosing speed at the other two. The event publishes all four
point impulses, including zero at inactive vertices. The full
`RigidDriftSegment` and fixed `PoseSegment` must prove one-sided departure and
a separated fraction-one sample before the step publishes an endpoint.

For an admitted spinning response, the rotational `RigidDriftSegment` sweep
must certify departure and every later interval through the requested end.
`World.Step` takes each returned endpoint pose from that sweep's fraction-one
sample. The sample's relation must be certified separated after charging its
float-pose deviation against the ideal path. The trace records the same two
rigid drift paths and their sweep report. It returns stored event and endpoint
checkpoints; an interior sample requires a separate rotational replay proof.
An absent fraction-one sample, a nonpositive endpoint gap, or a trace path
that differs from the swept path returns `Undecided` with no `Next`.

The fixed-floor Z-spin impact uses the rotating sweep's four-point horizontal
manifold. Its spin axis is the certified contact normal, so contact-point
spin contributes no normal speed. The bounded four-corner lever check limits
unpublished angular impulse. The event records the pre-impact poses, pre/post
angular velocities, and equal normal point impulses so conservation checks
both angular momentum and rotational energy. If float position correction
leaves a positive exact gap within the original correction allowance, move the dynamic box
back by that gap and accept only a rechecked `Touching` relation. The
remaining `RigidDriftSegment` must prove immediate departure and a separated
fraction-one sample that equals the published endpoint. Zero-restitution
spinning support still needs a contact-track proof and returns `Undecided`.

The fixed/dynamic translating-box slice publishes zero spin. It bounds the
omitted angular speed from the real manifold's patch-center offset and point
bounds, the mass-center bound, an impulse upper bound, and a certified lower
inertia eigenvalue. It returns `Undecided` when that upper speed exceeds
`AngularVelocityResidual`. The impulse and velocity response residuals compare
the published floats with exact rational evaluation of the held input values;
one final arithmetic ULP does not cover all intermediate rounding.
For zero-slip support, four equal normal corner impulses use the same bounded
witnesses. The sum of their horizontal levers and each point and center bound
set an upper angular speed through the certified inertia lower bound. The step
also checks that this speed can move no box point beyond
`PenetrationResidual` over the full duration. A bound that exceeds either
limit returns `Undecided`.

Position correction may move a dynamic body by at most the sum of certified
geometry displacement, the sweep bracket's point-travel bound, and
`ContactSlop`. An initial overlap beyond that allowance is undecided.
Correct all simultaneous contacts together. Record every correction in the
event and trace; sweep the correction path against every other pair and
reject a new contact or undecided interval. Active contacts may remain
touching while their penetration decreases. A post-correction `ContactPair`
bounds residual penetration. At a separating impact at the final step time,
a separated corrected pose is accepted only when the contact report's upper gap is within
the same correction allowance. A nonzero remainder still requires a corrected
pose that touches, or lies within the request's `SupportBand`
(Part N §10.5), and a certified continuation. A correction does not alter
velocity and cannot claim to conserve mechanical energy. A failed correction
or new uncertain pair returns `Undecided`.

### Completion, conservation, and trace

`ContactEvent.Kind` distinguishes `ContactImpact`, `ContactTransition`, and
`ContactGraze`.
An impact carries a bounded manifold and a normal impulse. A separated
contact transition carries its original bracket, the chosen right time,
unchanged velocities, and zero impulse; its `Manifold` is empty because the
right state is separated. `Trace.Sample` at that time returns the right state.
The frictional patch event also carries a typed aggregate tangent impulse,
one typed impulse per cloned manifold point, and bounded solver residuals with
the iteration count. A frictionless event and the centered static support
event carry a typed zero tangent impulse; the static support has no joint
solver report.
An isolated graze carries its bounded touching point and exact event time,
with zero-width bracket, typed zero impulses, and unchanged pre/post state.

```go
type StepStatus int // Advanced, Undecided

type StepReport struct {
    Status      StepStatus
    Next        *State // non-nil exactly for Advanced
    Events      []ContactEvent
    Excluded    []BodyPair // canonical world order; copied per report
    Trace       Trace
    Diagnostics []StepDiagnostic
    Conservation *StepConservation // non-nil for Advanced
    // Effective input/config, elapsed time, residuals,
    // angular momentum, and other conservation diagnostics.
}
```

Input validation runs before reading `ctx`. Invalid input returns a typed
error and nil report; cancellation after validation returns `ctx.Err()` and
nil report. A valid call whose geometry, mass, arithmetic, event limit, or
solver cannot settle returns an `Undecided` report with `Next == nil`.
The diagnostic names the pair or island, time interval, failing certificate
or residual, and requested limit. `from`, `World`, and `Document` never
change. The report owns copies of all slices and state data. A report with
`Next == nil` may contain a diagnostic trace prefix for inspection, but it
cannot be passed as an advanced state.

`StepConservation.Input`, `AfterKick`, and `Completion` each contain a bounded
`KineticEnergy` scalar, `LinearMomentum` vector, and `AngularMomentum` vector
summed over dynamic bodies. `KineticEnergy` uses `units.Torque` because the
registered torque unit has the same `kg·mm²/s²` dimension as energy; the field
denotes energy, not a turning moment. Linear momentum components and bounds
have kind `units.Impulse`; angular momentum components and bounds have kind
`units.AngularMomentum`. Angular momentum sums the orbital term
`mass × (world mass center × linear velocity)` and the rotated inertia tensor
times world angular velocity. Source mass-center and inertia bounds widen
the reading.
`GravityImpulse`, `LoadImpulse`, and `ContactImpulse` are separate bounded
linear vectors. `TorqueImpulse` reports the exact held world torque times
step duration as an `AngularMomentum` vector. `KinematicWork` is a signed
bounded energy reading with kind
`units.Torque`. For each kinematic contact event, it sums the exact held
aggregate impulse delivered to the dynamic body dotted with the driver's
published effective event velocity. A step without a kinematic impact has
typed zero work. Contact impulse sums only events against fixed or kinematic
bodies;
the two impulses of a dynamic pair cancel in the world total. These readings
describe the discrete step and exclude fixed/kinematic bodies' own energy and
momentum. The contact reading encloses the published numerical event impulses;
it does not claim to enclose an uncomputed physical impulse. Mass uncertainty
and conversion rounding widen energy, momentum, and gravity readings. A
reading that cannot be enclosed by finite typed values makes the step
`Undecided` with no `Next`.

`StepConservation.DriftChange` contains bounded energy, linear momentum, and
angular momentum changes over recorded trace slices. Without an event,
it uses `AfterKick → Completion`. With an event, it sums `AfterKick → pre-event`
and `post-event → Completion`. It excludes the force kick, event impulse, and
event position correction. Each body's translation and velocity coefficients
sum before the one held mass interval is applied, so the same source-center
uncertainty cancels across each pure-translation slice. Energy and linear
momentum change by zero on pure-translation slices. A rotating slice subtracts
separately enclosed endpoint readings, including rotated inertia. A changed
velocity within a drift slice makes the reading `Undecided`.

Kinetic energy includes translation and spin. Each advanced contact event
checks every dynamic body's published velocity change
against its signed aggregate normal and tangent impulse. It checks both ends
of the body's held mass interval against `ImpulseResidual + massHigh ×
VelocityResidual` on each component. A zero-impulse transition must preserve
the published velocities exactly. A `ContactGraze` separately requires zero
aggregate and per-point impulses, identical pre/post velocities and poses,
and zero position change before its event conservation check succeeds.
Impacts also check kinetic energy at the event. For each body, the code first
subtracts squared pre-event speed from squared post-event speed, then
multiplies this one difference by
the mass endpoint that gives the largest energy change. The allowed numerical
gain is the sum, over each dynamic body and Cartesian component, of
`(massHigh × VelocityResidual + ImpulseResidual) ×
(|preVelocity| + |postVelocity| + VelocityResidual)`. A failed gate makes the
step `Undecided`. The full-step force kick remains outside these event checks.
For a kinematic impact, the energy gate subtracts the driver's work from the
dynamic energy gain. It adds `ImpulseResidual` times the sum of the absolute
driver velocity components to the numerical allowance. Events that publish
angular velocity and event poses also check spin-energy change against the
same held inertia interval and compare angular-momentum change with the
signed torque from every published point impulse. Angular-velocity and
contact-point residuals widen that comparison.

`Trace` contains the starting state, each certified drift slice, each
event's pre/post states, each position correction, and the ending state.
`Trace.Sample(t)` evaluates the recorded paths without running geometry or
the solver. The N-body slice record, how a sample reads a body covered by
several pair certificates or by none, and the multi-step `Timeline` are
Part N §3.4 and §7. At a numerical event time it returns the post-event velocity
and pose; an event exposes the pre-event values separately. Its event stores
the pair's original bodies, the sweep time bracket, chosen numerical time,
certified manifold and its bounds, impulses, residuals, and pre/post
velocities. `Trace.Sample` refuses a time outside `[0, dt]`. Feed `Next` to
the next `Step` for deterministic replay with the same inputs and config.
The trace never labels the chosen numerical time as an exact physical impact.
`ContactEvent.Bracket` retains the producing pair sweep's local fraction and
elapsed readings. `SliceStart` and `SliceDuration` state that sweep's global
start and requested duration; `ContactEvent.Time` is measured from the start
of the complete step. Compare bracket order using exact held values:
`SliceStart + Bracket.<end>.Fraction × SliceDuration`. A rounded global event
time must stay within that bracket and its rounded pose must pass the same
contact transfer checks before the event is published.
Contact transitions appear in the trace with zero impulse unless the
restarted solver finds a closing constraint at that transition.

The translating first slice re-sweeps every returned rounded drift endpoint
as a `PoseSegment`. A numerical path that lacks `Clear` or
`DepartedClear` proof returns `Undecided`, even if its ideal drift was clear.
The rotational response uses the certified `RigidDriftSegment` as its stored
path and returns that sweep's certified fraction-one float pose. It does not
claim that an independently interpolated `PoseSegment` is clear.
`Trace.Sample` evaluates clear, departed, persistent-contact, and transition
slices against each stored rounded sweep's cached source-box, source-sphere,
or certified faceted support-face certificate.
An isolated graze uses one full-span rounded sphere-pair certificate across
both open sides and checks exact touch at its event time.
For an off-axis sphere-pair impact, the rounded impact prefix must reach its
own certified bracket endpoint with the same source faces, bounded normal,
and separation within `PenetrationResidual`. Interior trace samples before
and after the event use the corresponding sphere-pair sweep certificates.
For an off-axis sphere-pair impact at time zero, no prefix is needed.
`Trace.Sample(0)` returns the post-impact state; each positive interior time
uses the certified rounded departure or persistent-contact path, and
`Trace.Sample(dt)` returns `Next`.
It compares the exact held time values when selecting an event or endpoint.
An event at a proved final sweep fraction retains the input `dt` value;
interior event times must lie strictly inside the exact held duration.
For each slice it maps the exact time between its recorded global endpoints
onto the certified rounded path fraction, so subtraction rounding leaves no
unsampled gap before the step endpoint.
It returns `ErrUnsupported` for a slice without a rounded certificate, or
when the requested float pose exceeds the sweep's resolution. Impact prefixes
have a separate rounded sweep ending at the published pre-event pose, so
samples on both sides of an impact consume their own certificates. The
spinning source-box impact uses the original full-step rotating sweep for its
clear prefix and maps pre-event trace time against the full step duration.
An event-free clear rotating source-box drift also replays its interior poses.
The sweep retains its exact held source corners and path inputs. Each requested
rounded pose must lie within `PointResolution` of the ideal path under the
same staged source-corner transform used by contact geometry. The
rounded pair's exact separating-axis gap must exceed that pose error.
For a co-moving oriented source-box face track, replay checks the rounded
four-point manifold against the producer's face identities and point and
normal bounds. A fixed-floor spinning impact replays clear times before its
unresolved bracket from the original full-step sweep. Its separating
remainder replays from the cached departure sweep. Samples inside the
unresolved bracket return `ErrUnsupported`.

Publish kinetic energy, linear momentum, and angular momentum at the input,
after the full-step force kick, and at completion as typed numeric readings;
angular momentum includes orbital and spin terms about the fixed world
origin. Publish gravity/load impulse and kinematic/fixed contact impulse
separately. The full-step kick's energy and momentum change is charged once,
before any event. For an isolated dynamic pair, compare momentum immediately
before and after each impulse: equal/opposite impulses must leave total
linear momentum within the published numerical residual. Check angular
momentum about one world origin using both lever arms at that event. With
restitution in `[0,1]` and no kinematic participant, compare kinetic energy
immediately before and after an impulse; frictional impact cannot increase
it beyond its computed numerical residual. A failed event gate is undecided.
Also publish drift-only changes separately, since this first-order rotation
update need not conserve an asymmetric body's angular momentum exactly.
Kinematic work, the discrete force kick, and position correction are reported
so an energy difference in those cases is not mislabeled as collision loss.
These numeric checks apply to this discrete integrator; they are not
certified bounds on real continuous-force motion.

### Verification

The isolated three-dynamic fixture uses three real radius-5 mm source
spheres with exact supplied mass `1 kg` each. A and B start at
`(-6,-8,0)` and `(6,8,0) mm`, moving at `(30,40,0)` and
`(-30,-40,0) mm/s`; C starts at `(0,0,100) mm` and moves at
`(1,0,0) mm/s`. With restitution `0.5`, zero friction, and `dt=0.2 s`,
the real A/B sweep brackets one impact near `0.1 s` while both C pair
sweeps stay clear. `Step` publishes one A/B event with normal impulse
`75 kg·mm/s`; C ends at `x=0.2 mm` and its trace samples at `0.05`,
`0.1`, and `0.15 s` have `x=0.05`, `0.1`, and `0.15 mm`. Total input
and completion kinetic energies are `2500.5` and `625.5 kg·mm²/s²`,
and total linear X momentum is `1 kg·mm/s` at both ends. A force of
`10 kg·mm/s²` applied only to C makes its full-step kick `2 mm/s`;
C ends at `x=0.6 mm` with `3 mm/s` velocity, while the A/B event
remains unchanged. With `MaxEvents=1`, the step returns `Undecided`
and no `Next` because time remains after the impact.

Two real radius-5 mm source spheres start with centers at `(0,0,0)` and
`(6,8,0)` mm. Both have supplied mass `1 kg` and zero spin. B approaches at
`(-30,-40,0) mm/s`, A starts still, restitution is `0.5`, friction is zero,
and `dt=0.1 s`. The real `ContactPair` must return a bounded point touch,
and `SweepPair` must return `SweepInitiallyTouching`. `World.Step` must publish
one time-zero impact with a zero-width bracket and normal impulse
`37.5 kg·mm/s`; A leaves at `(-22.5,-30,0) mm/s`, and B leaves at
`(-7.5,-10,0) mm/s` within the stated response residuals. The ideal and
rounded full-span sweeps must both return `SweepDepartedClear`. The final
centers are `(-2.25,-3,0)` and `(5.25,7,0)` mm, 12.5 mm apart. `Trace.Sample`
at zero, an interior time, and `dt` must return the certified post-impact
velocities and poses. Reverse world order and reject an off-center supplied
mass whose omitted spin exceeds `AngularVelocityResidual`.

The isolated-graze fixture uses two real radius-5 mm source spheres. A stays
at `(0,0,0)` and B starts at `(20,10,0)` with velocity `(-40,0,0) mm/s`
for `1 s`. Give B positive admitted mass and A either positive admitted mass
or the fixed role. The real `ContactPair` at `0.5 s` must report touch with
one bounded point; the real ideal and rounded `SweepPair` reports must both
return `SweepGrazingTouch` at exact fraction `1/2`. `World.Step` must advance
to B at `(-20,10,0)` with its velocity unchanged and one `ContactGraze`
event at `0.5 s`. All event impulses and contact conservation impulses are
typed zero. `Trace.Sample` before, at, and after `0.5 s` must return the
certified poses and the same velocities; the event sample is touching and
the other samples are separated. Repeat with A/B world order reversed and
with two dynamic bodies to check the normal and momentum signs.

A `y=11 mm` near miss must advance without an event after `SweepClear`; a
`y=9 mm` crossing must enter impact response or return `Undecided`, never
`ContactGraze`. Refuse a tangent at `t=0` or `t=dt` on this path, as well as
any rounded endpoint path whose exact minimum differs from the ideal one.
With a two-pose sweep budget or an event time that cannot be represented
exactly, `Step` returns `Undecided` and `Next == nil`. A trace replay query
whose rounded pose loses strict separation or exact event touch returns
`ErrUnsupported`. The tests use the real source-body producer, sweep, world,
and trace; they do not supply a hand-written event.
Start from that real event and change one conservation field at a time: the
aggregate normal or tangent impulse, the point normal or tangent impulse,
one pre/post linear or angular velocity, one event or trace pose, and one
position-change component. The `ContactGraze` conservation branch must reject
each change. It must also reject a missing point-impulse record or a non-nil
solver report. The unchanged real event must pass and publish zero event
energy, momentum, and contact impulse.

First use a real `decad` box with top `z=0 mm` and a dynamic `10×10×10 mm`
box initially spanning `z=[10,20] mm`. Give the dynamic box density
`0.001 kg/mm³` → mass `1 kg`; set gravity and friction to zero, initial
downward velocity `100 mm/s`, restitution `0.5`, and `dt=0.2 s`. The real
`SweepPair` bracket and `ContactPair` manifold must reach the solver. The
contact time lies within the requested time resolution of `0.1 s`; normal
impulse is `150 kg·mm/s`; the final upward velocity is `50 mm/s`; and the
bottom ends at `z=5 mm`, within the stated integration/geometry tolerance.
Initial/final kinetic energies are `5000` and `1250 kg·mm²/s²`. The document
body set and placements are unchanged. A hand-written manifold does not
exercise this boundary.
With `dt=0.1 s`, the same real path impacts at the final fraction. Its
`150 kg·mm/s` impulse leaves upward velocity `50 mm/s`; any separated rounded
endpoint gap must fit the certified correction allowance.

The supplied-mass fixture passes that box's real `Body.MassProperties` result
through `RigidBody.Supplied`, `NewWorld`, `SweepPair`, and `Step`. It has the same
`150 kg·mm/s` impulse, `50 mm/s` outgoing speed, and `z=5 mm` final bottom.
Changing the caller's record after world construction does not change the step.

The oblique fixture uses a wide fixed floor and the same `1 kg` box, starting
`10 mm` above the floor with velocity `(50, 0, −100) mm/s`. At `0.1 s`, the
real sweep brackets face contact and the step applies `150 kg·mm/s` along the
certified vertical normal. At `0.2 s`, the box has velocity `(50, 0, 50) mm/s`
and translation `(10, 0, 5) mm`. The tangent speed stays `50 mm/s`; the ideal
and rounded departure paths are both certified.

The tilted-pose fixture rotates two touching `10 mm` boxes by 45 degrees
around world Y. One box is fixed; the `1 kg` dynamic box enters at
`(-50, 0, 50) mm/s`. Its four-point bounded manifold and initial-touch sweep
produce a `sqrt(5000) kg·mm/s` support impulse. The dynamic box stops and
both the ideal and rounded sweeps prove persistent contact through `0.01 s`.
The reported contact-impulse bound is positive; another stationary step
advances without an event. With restitution `0.5`, the initial impulse is
`1.5 sqrt(5000) kg·mm/s`, the outgoing velocity is `(25, 0, −25) mm/s`, and
the box translates `(0.25, 0, −0.25) mm` over `0.01 s`. Both ideal and rounded
paths prove one-sided departure; a second step advances clear without an event.

The center-force fixture starts the same `1 kg` box `10 mm` above the floor at
rest. A `−500 kg·mm/s²` center force over `0.2 s` gives a full-step kick of
`−100 mm/s`. The real sweep brackets contact at `0.1 s`; the step reports a
`150 kg·mm/s` contact impulse and finishes at `+50 mm/s` with its bottom at
`z=5 mm`. A supplied mass interval of `1 ± 0.01 kg` returns `Undecided` at a
`1e−6 mm/s` velocity residual because its kick cannot be bounded that tightly.

The torque fixture starts a `1 kg`, `10×10×10 mm` source box `100 mm` above
the floor. A `100 kg·mm²/s²` world-Z torque over `0.1 s` produces `0.6 rad/s`
spin and a `0.06 rad` drift rotation. The real full-span rotating sweep proves
separation; the step reports `10 kg·mm²/s` torque impulse and bounded spin
energy and angular momentum. A second step from its rotated endpoint, with
zero load, advances another `0.06 rad`. An approaching torque-driven box
returns `Undecided` when the rotating sweep cannot certify the contact.

The resting-contact fixture starts a `1 kg` source box at rest on
a fixed floor and applies gravity `−1000 mm/s²` for `0.1 s`. The full-step
kick gives `v_z=−100 mm/s` before the zero-time contact event. The support
impulse is `100 kg·mm/s`, and the box finishes with zero velocity and its
bottom at `z=0 mm`. The second step repeats those results from the returned
state. Both steps consume the real face manifold and require a full-span
`SweepPersistentTouch` track for the ideal and rounded drift paths.

The zero-restitution impact fixture drops the same box from `10 mm` above
the floor at `100 mm/s` for `0.2 s`. It reaches contact at about `0.1 s`,
receives `100 kg·mm/s`, and finishes at rest with its bottom at `z=0 mm`.
The remaining drift passes ideal and rounded `SweepPersistentTouch` checks.
The stationary-touch fixture starts with the box already on the floor, uses
zero gravity for `0.1 s`, and returns the same pose and velocity with no
contact event or impulse. It also checks both full-span contact tracks.

The frictionless-slide fixture starts the same `1 kg` box touching a wide
fixed floor with `(50, 0, 0) mm/s` velocity and zero gravity. At `0.1 s`,
it has translated `(5, 0, 0) mm` with unchanged velocity and no contact
event or impulse. Real ideal and rounded sweeps each return a full-span
`SweepPersistentTouch` track, and `ContactPair` confirms both endpoints.
A narrower floor that changes the contact patch during the step returns
`Undecided` when the step cannot certify the transition and remainder. The
edge-exit case has a `10 mm` wide floor and touching box,
with the box moving at `5 mm/s` for `3 s`. The transition brackets `2 s`,
and a certified clear remainder ends at `x=15 mm` with unchanged velocity.
It reports one zero-impulse transition event. `MaxEvents=1` refuses the
remaining `1 s`; `MaxEvents=2` admits the step.
The solver also sweeps the rounded prefix directly to its published right
pose. The full-step rounded segment can sample a different pose by one ULP
at that time; the direct prefix must certify the same separated transition.

The kinematic-push fixture has two touching `10 mm` source boxes. Body A
follows a `PoseSegment` of `+1.25 mm` in `0.125 s`; body B has mass `1 kg`
and starts at rest. With zero restitution, friction, and gravity, the real
initial face manifold produces a `10 kg·mm/s` impulse. The ideal driver and
dynamic drift and their rounded paths certify persistent touch. Both bodies
finish `1.25 mm` to the right, B moves at `10 mm/s`, and A stores zero velocity.

The interior kinematic-impact fixture starts A at `x=[0,10] mm` and B at
`x=[20,30] mm`. A's driver moves `+20 mm` in `0.25 s` at `80 mm/s`; B has
mass `1 kg` and starts at rest. With restitution `0.5`, friction zero, and
gravity zero, the real sweep brackets impact near `0.125 s`. The step consumes
the bracket-right sample, applies `120 kg·mm/s`, and certifies departure on
both sliced remainder paths. B ends near `x=[35,45] mm` at `120 mm/s`; A ends
at `x=[20,30] mm` with zero stored velocity. The tiny difference from B's
ideal final coordinates is bounded by the impact bracket and correction.

With the same bodies and driver but restitution zero, the bracket-right
correction and `80 kg·mm/s` impulse give B `80 mm/s`. Both sliced remainder
paths certify persistent face touch. B ends near `x=[30,40] mm` and A ends at
`x=[20,30] mm`; the bracket and correction bound B's small offset from the
ideal coordinates. A still stores zero velocity.

For the no-impulse departure fixture, A starts at `x=[0,10] mm` touching B
at `x=[10,20] mm`. A's driver moves `−5 mm` in `0.125 s`, and B starts at
rest. Both full-step paths certify `SweepDepartedClear`. The step reports no
event, moves A to `x=[−5,5] mm`, leaves B at `x=[10,20] mm`, and stores zero
velocity for both. It advances with `MaxEvents=1` because no event occurs.

For the sliding-friction fixture, put the same `1 kg` box
on a fixed floor wide enough for a `5 mm` slide. With gravity
`−1000 mm/s²`, initial horizontal speed `100 mm/s`, friction `0.5`, and
`dt=0.1 s`, the full-step kick gives `v_z=−100 mm/s`. The support impulse
is `100 kg·mm/s`; the friction impulse is `50 kg·mm/s` opposite motion.
The box then slides at `50 mm/s` and moves `5 mm` horizontally while its
bottom stays at `z=0 mm`. `SweepPersistentTouch` must certify the full
face-contact track. A second `0.05 s` step from that endpoint receives a
`50 kg·mm/s` support impulse and `25 kg·mm/s` friction impulse. It ends at
`x=6.25 mm` with `25 mm/s` horizontal speed. A run starting with zero
horizontal velocity must end with zero velocity and the same bottom height.
Every run uses the real contact manifold and sweep continuation, not a
touching sample alone.

Then run focused checks for an off-center impact with spin, two independent
moving bodies, a kinematic push, three simultaneous bodies, Coulomb stick
and slip, a resting stack across two steps, and exact trace replay. For each,
assert computed position or velocity, event ordering, the stated residuals,
and the unchanged document. Exercise reflected/invalid poses, bad mass
bounds, mixed units, missing drivers, an unresolved sweep, an inadequate
manifold, event/iteration limits, initial overlap, and cancellation. Test
energy and momentum on isolated dynamic pairs and report external work in
forced/kinematic cases. Run the real geometry producer through the real
solver in every integration check; direct impulse-equation unit tests may
use constructed contact data but cannot claim pair-query integration.
The off-center spin check may end at the impact event. A longer remainder needs a real
pair-clearance certificate; co-oriented oblique source-box face contact has a bounded manifold,
while other rotated face arrangements may return `Undecided`.

## Part M — Dynamic mass

This part owns mass, center-of-mass, and inertia readings used by rigid
dynamics. It supplies the mass gate in Part C §2.
The source-box query, bounded section-moment integration for admitted
untapered prisms under any frame and rigid placement, including displaced
section and level records, full source-sphere integration, and full source-cylinder
integration for a revolved axis-incident rectangle are implemented, as is
third-order section-moment integration for a full or partial revolve about
an exact in-plane axis under any frame and rigid placement, and the sweeps and
cups those prism and revolve integrals compose. Faceted
Boolean solids with verified occupied-volume bounds publish density-derived
mass and inertia when their resulting volume and tensor intervals pass the
positive admission gates, and so does every other solid whose `VerifyAll`
mesh carries an occupied-volume proof, through §2.2's curved path.
A payload with neither path returns `ErrUnsupported`; Part N
§8 owns which of §2.1 or §2.2 each payload takes.
`agent-docs/evaluator-design.md` §4 owns
the existing planar area moments; this document owns the additional volume
moments and their use by dynamics.

### 1. Boundary and inputs

`decad` computes mass properties of its immutable `BodySolid` geometry for a
uniform density. `dynamics` chooses one of two explicit input modes per dynamic
body: density-derived properties or caller-supplied mass, center, and inertia.
The modes cannot be combined. Fixed and kinematic bodies need neither mode.
Multiple lumps and void shells contribute to one body's net mass properties;
they do not become separate dynamic bodies.

```go
func (b *Body) MassProperties(ctx context.Context, density units.Value) (MassProperties, error)

type MassProperties struct {
    Mass    Measurement        // units.Mass
    Center  VecMeasurement     // world position, millimetres
    Inertia InertiaReading      // about Center, in world axes
}

type InertiaReading struct {
    XX, YY, ZZ Measurement     // units.MomentOfInertia
    XY, XZ, YZ Measurement     // units.MomentOfInertia; signed products
}
```

Each `Measurement.Bound` is an absolute, nonnegative error of the same kind as
its value. `Center.Bound` is a length radius of a ball around `Center.Value`.
An `Exact` component has a zero bound; an inexact component is `Approximate`.
The six inertia components describe the symmetric tensor
`[[XX,XY,XZ],[XY,YY,YZ],[XZ,YZ,ZZ]]`. The products already include the
physical inertia sign: for example, `XY = -ρ∫(x-Cx)(y-Cy)dV`. Callers do not
negate them again. Each component has its own bound; one bound for all six
would hide which entries are certified tightly enough to invert.

`density` has kind `units.Density`, is finite, and is strictly positive. It is
the caller's stated uniform material parameter, not a measured interval.
Conversion from its registered unit to the base `kg/mm³` charges any numerical
rounding in the computed readings. A caller with uncertain physical density
must widen the returned readings itself or use a future bounded-density input;
the current signature cannot represent that uncertainty. The same density
applies to all material lumps and excludes all cavities.

`MassProperties` is a read-only query. It may read a retired body, as `Volume`
and `Centroid` do; the `dynamics.World` live-body gate is separate. A nil body,
a body without an evaluator payload, or a sheet is rejected. The method does
not call `Transform`, commit a body, consume a producer identity, or change the
document. An uncanceled repeated call on the same body and density produces
the same values, bounds, and error.

### 2. Geometric integrals

Choose a finite anchor `O` near the body's proven bounds, before integration.
For coordinates `q = x - O`, compute certified intervals for

```text
V       = ∫ dV
P_i     = ∫ q_i dV
Q_ij    = ∫ q_i q_j dV, i <= j
C       = O + P/V
S_ij    = Q_ij - P_i P_j/V
I_ij    = ρ (trace(S) δ_ij - S_ij)
M       = ρ V
```

`V` has kind Volume, `P` has Length × Volume, and `Q` has Length² × Volume.
`S` is the second volume moment about `C`; `I` has kind MomentOfInertia.
All interval operations round outward, including division and conversion to
the public `float64` readings. The body publishes no `MassProperties` unless
the certified volume interval has a strictly positive lower end. Signed
shell, cell, and lump contributions may cancel algebraically only after each
contribution has its own outward interval. An uncertainty term never cancels
because a neighboring term has the opposite sign.

The evaluator computes the integrals from its own admitted records. It does
not ask `sketch` to infer a missing cut or replace an uncertain curve with a
sampled one. The existing `Volume()` and `Centroid()` readings remain
independent public calls; the new query may reuse their certified internal
integrals when those carry every term needed for `P` and `Q`. It must not infer
`Q` from volume, centroid, `Bounds`, or a bounding box's inertia.

#### 2.1 Analytic records

An untapered prism integrates each section's area, first moments, and second
moments over its signed axial interval. For a section at `(u,v)` and axial
coordinate `z`, `∫u²dV = (z1-z0)∫u²dA`, `∫uzdV =
(z1²-z0²)∫u dA/2`, and `∫z²dV = (z1³-z0³)A/3` before rotation into world
axes. The corresponding `v` and `uv` terms follow the same rule. Holes use
their recorded winding. `internal/momentregion` already owns section moments through
second order; this path must reuse that validated record and its bounds.
Part N §8.1 owns the rotation of the local
tensor through a non-cardinal frame or placement basis, and §8.2 the
occupied-volume charge of a displaced section or level.

A revolve requires section integrals through third polynomial order because
the cylindrical Jacobian contributes one radius and a transverse second
volume moment contributes two more. Extend the recorded-section moment engine
for `r³`, `r²z`, and `rz²`, then integrate the stated angular interval with
certified trigonometric intervals. Full turns may simplify terms by symmetry;
partial turns must retain their nonzero mixed terms. Circular and free-form
segments retain their existing exactness tiers and admission rules. A sphere,
cylinder, cone, or torus may use an equivalent closed-form primitive integral
only when it represents that payload's exact denotation, including its cuts,
holes, and placement.

The implemented general revolve path (`internal/massmoment/revolve_properties.go`) keeps the
third-order sum as rational intervals about the plane origin, every segment
kind in the boundary form `∮u^(p+1)·v^q dv/(p+1)`. It re-expresses the plane
moments in the exact axis frame and multiplies them by the sweep's own
denoted angular factors. It forms `S` from one common `V`, `P`, `Q` enclosure
and proves the interval tensor positive definite by its leading principal
minors. It refuses a payload whose readings carry a term it does not charge:
an axis with a nonzero anchor or direction bound, a nonzero axis-snap or
admitted-band allowance, a sweep end without a denotation, or a section
displacement. The local tensor reaches world axes through the exact rational
product of the placement basis and the local axis, radial, and sweep
directions, widened by its orthonormality defect as the rotated prism is.

The full source-sphere path accepts a full revolution of one exact
semicircle and its on-axis diameter when its source and closed spherical face
prove the occupied ball. It integrates `V = 4πr³/3` and the isotropic tensor
`I = 2Mr²/5` with the in-tree rational π enclosure. It uses the evaluator's
bounded world centroid. Any rigid placement leaves its centroidal tensor
unchanged; a partial revolution takes the general revolve path.

The full source-cylinder path accepts a full revolution of an axis-incident
rectangular half-profile when its source, cardinal axis, and closed faces
prove the occupied cylinder. Its exact axial and radial extrema give `r` and
`h`; the in-tree π enclosure bounds `M = ρπr²h`, axial inertia `Mr²/2`, and
transverse inertia `M(3r²+h²)/12`. The evaluator's bounded centroid supplies
the world center. Cardinal rigid placements reorder the tensor entries.
Partial, annular, toroidal, and obliquely placed revolves take the general
revolve path above.

Other analytic payloads may provide certified `V`, `P`, and `Q` directly.
Otherwise they use §2.2 if they have a suitable occupied-volume proof. A
payload with neither path returns `ErrUnsupported`; it does not borrow a nearby
primitive's formula merely because the boundary looks similar.

#### 2.2 Faceted and curved records

For a truly faceted solid, audit its closed, consistently outward, embedded
boundary before integrating signed tetrahedra from `O` to its oriented
triangles. Exact arithmetic over held vertex coordinates yields each
tetrahedron's `V`, `P`, and `Q`, followed by outward conversion. A signed
tetrahedron sum alone is insufficient if the mesh self-intersects or fails
the solid topology audit. The faceted body may also carry a separate
displacement from its held triangles to its denoted solid; charge that
displacement through a certified occupied-volume error.

The implemented faceted arm requires a `VerifyAll` mesh with both boundary and
occupied-volume proofs. It reruns shell closure and orientation, vertex-link,
and exact facet-crossing audits on the held triangles. A failed or capped audit
returns `ErrUnsupported`. It integrates held `V`, `P`, and `Q` as exact rationals
anchored at one held vertex. The body's bounded box and all held vertices give
an exact rational extent `R_i` per axis with `|x_i - O_i| <= R_i` over both
occupied regions. The payload's certified `volSymDiff = E` widens held `V`,
each `P_i`, and each `Q_ij` by `E`, `R_i E`, and `R_i R_j E`, which never
exceeds the `R E` and `R² E` of any radius `R` enclosing both regions.
Interval division forms the center and centroidal tensor; final readings
round outward. A volume interval containing zero or a
tensor interval without a positive row-dominance proof returns
`ErrUnsupported`. A faceted payload without a matching verified mesh or finite
`E` and `R` also returns `ErrUnsupported`.

An admitted contained Cut retains source V/P/Q moments for the existing
audited-mesh comparison. A direct exact-rotation Copy of that original Cut
transports the frozen moments with its exact rational map before the same
comparison. See `contained-cut-moments-design.md` "Exact rotation copy"; later placements and
modifications do not inherit this evidence.

For a curved or approximated body, request a `VerifyAll` tessellation with
`BoundaryVerified()` and `VolumeVerified()`. Read its private certified
`volSymDiff = E`, where `E` bounds the occupied volume of
`TrueBody △ MeshSolid`. Both regions must lie within a proven radius `R`
about `O`, derived from the body's bounded `Bounds()`, the mesh vertices, and
their displacement bounds. Widen the mesh integrals by at least `E` for `V`,
`R E` for each `P_i`, and `R² E` for each `Q_ij`, plus outward arithmetic and
coordinate-construction terms. These are componentwise absolute bounds;
signed-volume closeness and a two-sided boundary distance alone do not imply
them. Refine the tessellation when permitted and useful. If the payload does
not publish an occupied-volume proof, or a finite `R` or finite moment bound
cannot be certified, return `ErrUnsupported`. The implemented curved path
anchors at the body box's midpoint, widens per axis as the faceted arm does,
and refines along Part N §8.5's tolerance ladder.

`VolumeVerified()` is a proof about the mesh and its source body, not a license
to use its triangles without the embedding and provenance audits. A render
mesh at `VerifyNone` or `VerifyBoundary` cannot provide density-derived
inertia. The same rule applies to placed faceted bodies and Boolean results:
their inherited occupied-volume allowances contribute to `E`.

An admitted faceted result of a strictly contained Cut also carries immutable
source `V`, `P`, and `Q` intervals for a rectangular-prism target minus a
whole circular-prism tool. `MassProperties` still computes the verified-mesh
tuple first. It publishes and selects the source tuple only when every mass
and inertia interval overlaps the mesh result, both center balls overlap, all
eight source bounds are no larger, and one is smaller. Other source classes
continue to use the verified-mesh path. The source evidence contains no
density or query cache.

### 3. Center, tensor, and numerical admission

Compute `S` with interval arithmetic from one common `V`, `P`, and `Q`
enclosure. Subtracting `P_iP_j/V` using point floats and attaching an error
afterward can lose the center-of-mass uncertainty. The near-body anchor limits
cancellation from a distant world origin; interval width still decides
whether a reading is usable. Center bounds include division by uncertain
volume and the anchor's coordinate or placement bound. Each inertia bound
includes density conversion, the `V/P/Q` intervals, center subtraction, and
the final tensor conversion.

Prove the entire admissible inertia interval positive definite before a
density-derived result is accepted for dynamics. A positive center tensor
alone is insufficient if its component bounds admit a singular tensor.
Use a certified lower eigenvalue bound or interval Cholesky test on the
six-component interval. Its lower bound must be strictly positive. A failed
proof may trigger tighter analytic arithmetic or a finer certified mesh;
exhausting a stated work limit returns `ErrUnsupported`. Inversion in
`dynamics` uses the certified interval, and the solver rejects a step if
resulting acceleration or impulse uncertainty cannot meet its residual gate.
All diagnostics identify whether volume positivity, a moment bound, or tensor
positivity blocked admission.

The parallel-axis rule is applied to each separate lump or primitive before
combining it about the body's common center: `I_at_C = I_at_c +
m[(d·d)1 - ddᵀ]`, where `d = c-C`. A cavity subtracts its `V`, `P`, and `Q`
before the body's final center and tensor are formed. A void is never a
negative-mass dynamic body. A disjoint-lump body with a common uniform density
returns one combined mass, center, and tensor.

### 4. Placement and supplied properties

The returned center and inertia refer to the body's **current world
placement**. For a later rigid pose `T`, transform the center with `T.Apply`
and rotate the centroidal tensor as `R I Rᵀ` using `T`'s orthogonal basis.
Translation leaves centroidal inertia unchanged. Reflection follows the same
tensor rule; it does not change mass. Carry interval bounds through each
basis product and its rounding. `Copy` or `Transform` must agree
with that transformed reading within both reported bounds. The dynamics state
applies its pose to the body's current placement exactly once.

The caller-supplied mode in `dynamics` contains a positive bounded `Mass`
reading, a bounded `Center` world position at the body's current placement,
and six bounded `Inertia` readings about that center in current world axes.
It is not an unlabelled diagonal or an inertia about the origin. All six
components and their bounds have kind `units.MomentOfInertia`; the mass value
and bound have kind `units.Mass`; the center bound has kind `units.Length`.
The lower mass endpoint and certified lower tensor eigenvalue are strictly
positive, and their reciprocals must fit in a finite `float64`. Every reading
has `Exact` or `Approximate` exactness; an `Exact` reading has a zero bound.
`dynamics.World` copies the supplied record during construction. The mode is
explicit, so a zero density value never means “use the
supplied tensor.” The caller may use measured values that differ from the
geometry-derived values; dynamics reports that input mode in its world
description and does not silently replace them with geometric readings.

`r3` needs a symmetric 3×3 numerical tensor with rotation by an orthogonal
transform, positive-definite validation, inversion, and vector action.
`decad` and `dynamics` use those operations rather than duplicating coordinate
algebra. The public six-component readings stay typed `units.Value`; `r3`'s
numerical tensor is an internal arithmetic representation, not a public
physical quantity.

### 5. Refusals and cancellation

Validation runs before reading `ctx`. A nil or payload-less receiver returns
`ErrDegenerate` or `ErrUnsupported`, respectively. A non-solid receiver
returns `ErrNotSolid`. Wrong density kind returns `ErrUnitKind`; non-finite
density returns `ErrNotFinite`; negative density returns
`ErrNegativeMagnitude`; zero density returns `ErrDegenerate`. A valid but
unsupported payload, absent occupied-volume certificate, interval volume
containing zero, or unproved positive tensor returns `ErrUnsupported` with
the specific cause. Arithmetic overflow that prevents a finite computed
reading returns `ErrNotFinite`. No failure returns partial properties.

After validation, every potentially long integration, tessellation,
refinement, and proof loop polls `ctx`. Cancellation returns `ctx.Err()` and
zero properties, including when cancellation arrives before publication.
The query leaves the document and body's geometric measurements unchanged on
success, refusal, and cancellation. `dynamics` validates caller-supplied
readings before a step; wrong kinds and non-finite values use its typed input
errors, and a mass or inertia interval that is not provably positive uses
`ErrInvalidMassProperties`.

### 6. Dependencies and computed checks

The pinned `units` version in `go.mod` defines `Mass`, `Density`, and
`MomentOfInertia`; the query itself needs no time kind. The pinned `r3` version has rigid
transforms and a symmetric-tensor type with rotation, positive-definite validation,
inversion, and vector action.

Computed tests must cover these results, with independent formulas or
quadrature and assertions against the published bounds:

| Body and input | Expected geometric result |
|---|---|
| `a × b × c` box with density `ρ` | `M = ρabc`; center at box midpoint; diagonal inertia `M(b²+c²)/12`, `M(a²+c²)/12`, `M(a²+b²)/12`; zero mixed terms in aligned axes. |
| Same box after a non-axis-aligned rigid rotation and translation | Mass unchanged; center transformed; all six components enclosed by `R I Rᵀ`; translation adds no centroidal inertia. |
| Full solid cylinder, radius `r`, height `h` | Axial inertia `Mr²/2`; transverse inertia `M(3r²+h²)/12`; bounds enclose the independent π-based values. |
| Partial revolve with an off-axis section | At least one nonzero mixed component agrees with independently integrated `r³`, `r²z`, and `rz²` terms. |
| Two separated equal boxes in one body | Common-center inertia includes the `m d²` parallel-axis contribution; no per-lump tensor is returned as the body tensor. |
| Hollow solid with a centered cavity | Mass and tensor equal outer solid minus cavity integrals; cavity material contributes no positive mass. |
| Faceted Boolean over boxes, with zero or certified positive error | Overlapping, disjoint, cavity, and placed results enclose independent box inclusion-exclusion mass, center, and all tensor components. |
| Verified faceted box and a curved body at coarse and fine tessellation | True mass and every true tensor component lie in both reported intervals; refined proof narrows at least one bound. |
| Curved body without occupied-volume proof | Query returns `ErrUnsupported`; a render mesh never supplies an inertia estimate. |
| Bad density, sheet, cancellation, and ill-conditioned tensor interval | Each returns the specified typed refusal and publishes no partial properties. |

The end-to-end dynamics check uses a real decad box, its density-derived
mass and inertia, a real pair sweep, and the real contact solver. Supplying a
hand-written tensor or contact to that check would not exercise this seam.

## Part G — Contact geometry

Navigation: pair verdict → §1–§2; manifold → §3; source-box proofs → §4;
contact cases → §5; refusal → §6; delivery tests → §7.

This part owns the pair pose query and the geometry passed to rigid-body
response. Part C maps the overall system, and
Part S owns the two-body sweep. `agent-docs/clearance-design.md`
owns boundary-distance and touching proofs; `agent-docs/interference-design.md`
owns positive-volume overlap;
`agent-docs/tessellation-design.md` owns source-face displacement and mesh audits.
This part adds witnesses and normal certificates without weakening any of
those admission gates.

Current code certifies relations and face manifolds for source boxes at
signed-permutation poses. An oriented source-box path also certifies relations
under arbitrary proper read poses by projecting the exact transformed corners
on all face and edge-cross axes. Two boxes whose three source edge directions
match can publish a four-point manifold at one isolated oblique face touch.
One horizontal rotated box face strictly inside an axis-aligned face publishes
a four-point manifold. At isolated touch, a partly overhanging horizontal
face publishes every vertex of its exact clipped polygon. Two opposed
axis-normal faces can publish
one bounded interior witness when one projected face center lies strictly
inside the other face. Other rotated box-box contacts publish no
manifold. A full source semicircle sphere against a source box also receives
an exact rational relation proof and a point manifold at one isolated face
support. When a query rotates the source box into an exactly orthogonal
parallelotope, the source sphere also gets an exact complete-box distance
proof and a bounded point manifold inside one rotated face. At identity query
poses, the analytic clearance kernel can certify
a relation for other admitted solids without a contact manifold. When it
proves a touch with a ruling certificate, a full source cylinder against a
planar face, or two parallel full source cylinders, publish the two ends of
the contact ruling (§4.5). Other curved
and broader faceted witness and normal proofs remain design contracts.
An internal reader can certify one complete rectangular axis support face of a
zero-bound faceted Boolean result. It requires zero boundary displacement,
zero occupied-volume difference, one outward-oriented planar source Face at
the extremum, and exact triangle area equal to the footprint rectangle. A
second support patch or nonrectangular footprint is refused. A zero-bound
Boolean may carry the same exact source mesh through translation-only `Transform`.
The rebuilt body has a positive held-mesh bound, but its saved source mesh and
exact accumulated translation still prove the true support plane and rectangle.
The reader checks that every rebuilt vertex lies within the published bound of
its exact source position and maps the support triangles to one placed Face.
An inherited positive bound, non-translation placement, or mismatched mesh
provenance cannot use this proof.
One mesh `Union` can instead retain an exact source-box lower face even when
its own boundary and occupied-volume bounds are positive. The other operand's
`Bounds` lower Z reading minus its position bound must lie strictly above the
box's exact lower plane. The result's lower-plane facets must all map to the
source box's lower-face group and one live faceted Face, with exact triangle
area and footprint equal to the source rectangle. The mesh bound widens the
rest of the result's held extents before the floor-footprint check. A later
`Transform` drops this source-box certificate; a tool reaching the lower plane
never receives it.
`ContactPair` uses the lower Z support against the upper face of a source-box
floor when the support rectangle lies strictly inside the floor face. Exact
support-plane equality proves `Touching` and publishes all four footprint
corners with the original faceted Face and an exact vertical normal. Positive
axial separation publishes the exact gap. A crossing support plane, floor
edge, lateral approach, or missing support proof remains `Undecided`.
When a faceted Boolean carries a positive finite boundary displacement without
an exact lower support proof, a signed-axis pose can still prove strict separation
above a source-box floor.
The held vertex extrema bound the whole held solid, and the certified two-sided
boundary displacement widens each extent. The widened X and Y extents must lie
strictly inside the floor face. The held lower Z extent minus the displacement
must stay strictly above the floor's upper face. The resulting gap reading
charges the boundary displacement and its own float conversion. This path
publishes no support face or manifold; a near touch or crossing it leaves
undecided falls to the band rule below.

A full circular source prism at a signed-axis pose can certify separation
from a source box across one axial face. Its complete projected disk must lie
strictly inside that box face. The source circle, prism extent, and placement
produce an exact outer box, and the axial support gap is the true pair gap.
The same proof admits a full revolve of a rectangular half-profile whose
inner radial edge lies exactly on the resolved axis. It requires exact
cardinal frame and axis readings, zero section displacement, and signed-axis
placements. The recorded rectangle and axis determine the exact axial limits
and radius; the resulting solid has two planar disks and one cylindrical wall.
Both source paths identify their original planar disk faces by exact signed
world normals. At axial touch, each publishes a bounded disk-center witness
and the opposed original face identities. A shallow crossing of the opposed
axial supports also proves overlap and publishes the same witnesses. A full
circular vertical source prism can also meet a broad box face along its
sidewall when the complete axial line and transverse diameter lie inside that
face. The
source cylindrical Face and box Face give one midpoint witness and an exact
transverse normal. Embedded cylinders, edge contact, revolved sidewalls, and
tilted axes remain `Undecided` on this source path.

Two full source semicircle spheres receive the exact relation proof from their
recorded centers and radii. A nonzero center offset with crossing sphere
surfaces publishes one bounded point manifold along the center line. Coincident
centers keep the relation and withhold the manifold.

Two exact planar solids receive an exact relation at any query pose whose
basis has a positive determinant. A solid is admitted as a prism over whole
`LineSeg` edges with zero deltas, or as a zero-bound faceted Boolean, directly
or through a translation-only placement. The relation is separation with an
enclosed gap, touch, or overlap, decided by the exact facet tests of
Part N §9.1. With neither body carrying §9.2's
convexity certificate, a touch or overlap has no manifold and the reason
`ContactNonConvex`. Otherwise a touch publishes §9.3's manifold: clipped
face patches, edge ends and vertices inside a face with that face's normal,
and crease crossings, or `ContactAmbiguousFeature` when a contact falls
outside that table. An overlap of two convex bodies publishes the patch at
depth through one unique shallowest pair of crossing faces, or, when only one
body has a face across that shallowest push, the other body's deepest edge
or vertex with its feet on that face; any other overlap keeps
`ContactNoNormalProof`. When the faceted floor-support proof leaves a
pair undecided, this relation decides it.

A positive-bound faceted Boolean that no exact source mesh covers, and a
cap-loop chamfer whose every face is planar, enter the same relation
through their held meshes. Each held mesh is exact, and its true boundary
lies within the mesh's two-sided displacement δ of it. The held relation
becomes the published one by Part N §10.4: a
held gap past the summed δ is a gap with δ charged, a held vertex deeper
than δ proves overlap, and a held touch, gap or shallow depth within δ is
`ContactBand` with `Gap` [−2δ, 2δ]. A band's manifold is the held one with
each witness ball widened by its body's δ, published only where an exact
face of a body with no displacement supplies the normal.

### 1. Claims and entry point

```go
func (d *Document) ContactPair(ctx context.Context, a, b *Body,
    poseA, poseB r3.Transform, req ContactRequest) (*ContactReport, error)

func (d *Document) ProvePairSeparated(ctx context.Context, a, b *Body,
    poseA, poseB r3.Transform, req ContactRequest) (bool, error)

type ContactRequest struct {
    PointResolution  units.Value // positive Length
    NormalResolution units.Value // positive Angle
    SupportBand      units.Value // nonnegative Length; zero publishes the exact contact set alone (multibody §10.5)
    HeldChord        units.Value // nonnegative Length; zero admits no curved held mesh
}

type ContactRelation int // Separated, Touching, Overlapping, Undecided,
                         // ContactBand (appended; multibody §10.4)

type ContactReport struct {
    A, B       *Body
    PoseA      r3.Transform
    PoseB      r3.Transform
    Request    ContactRequest
    Relation   ContactRelation
    Gap        *Measurement // Separated or Touching only
    Overlap    *Measurement // bounded Volume if proven; optional on Overlapping
    Manifold   *ContactManifold
    Reason     ContactReason // typed, only for Undecided or absent manifold
}
```

The snippets above show the shape only. The shipped types are the generic forms in `report/contact.go`
(`ContactReportOf`, `ContactManifoldOf`, `ContactPointOf`, `ContactFeatureOf`), instantiated with root handles:
`ContactReport` is `ContactReportOf[*Body, *Face, *Edge, *Vertex]`, and `ContactFeature` is
`ContactFeatureOf[*Face, *Edge, *Vertex]`. The fields and absence rules above hold. The report always names the original bodies and their caller-supplied
poses. Poses act **after** each body's recorded placement, as `Transform`
does. Two identical body pointers, a nil or foreign body, a retired body, a
sheet, an unsound solid, an invalid transform, or an invalid request is an
input error with no report. Use the existing typed errors for the relevant
cause. A valid solid payload that cannot be classified is `Undecided`, not an
input error. Validate bodies, poses, kinds, finiteness, and positive
resolutions before reading `ctx`; then propagate `ctx.Err()` unchanged.

`Separated` proves disjoint interiors and a strictly positive minimum
boundary gap. `Touching` proves disjoint interiors with the complete
zero-distance set certified by clearance's touching gate or §4's source-box
proof; `Gap` is `Exact` zero. `Overlapping` proves shared interior. An
optional `Overlap` volume is
present only after interference's independent volume gate passes; a boundary
crossing or containment witness alone may prove the relation. `Undecided`
asserts none of those three relations and has no gap or manifold. A known
relation may still have no usable manifold. `Reason` then names the missing
certificate, for example `ContactNoNormalProof`, `ContactAmbiguousFeature`,
`ContactPointTooCoarse`, `ContactPayloadUnsupported`, or `ContactNonConvex`
(appended; a dynamic body without Part N §9.2's
convexity certificate); callers branch on
that code, not its message. Do not reuse `Diagnostic` or `Verify.Status` as
the pair verdict. `ContactBand` is the one relation that is neither of the
three exact relations nor `Undecided`: it states a published band around zero
that the pair's separation lies in, for a body whose held boundary carries a
positive displacement, or for an exact pair apart by at most the request's
`SupportBand`. Part N §10.4 owns when a displaced
pair publishes it and §10.5 when an exact pair does, and what `Gap` and the
manifold carry in each case.

`ContactPair` always asks the analytic distance kernel for a gap on a
separated pair; box separation alone does not supply a minimum. If the
analytic kernel proves separation but cannot measure the gap, return
`Undecided` with a gap-specific reason. Keep the private four-way
`boundaryaudit.PairVerdict` separate from the public `ContactRelation`; map it only after
the transient pose's own displacement has been charged. A later overlap
volume proof may upgrade an undecided analytic relation, but may not weaken a
stronger earlier certificate.

`ProvePairSeparated` is a read-only proof query for callers that need only a
strict positive gap. It repeats `ContactPair`'s input validation and error
priority, reads both bodies at the supplied query poses and `HeldChord`, and
charges the same summed boundary displacement. It returns true only when an
exact planar relation proves the held gap greater than that displacement. At
zero `SupportBand` it uses `ClassifyPlanarWithin` with the displacement as its
distance limit, so it avoids measuring a positive gap. A nonzero `SupportBand`
or a non-planar snapshot returns false without claiming separation. Touching,
overlapping, `ContactBand`, ambiguous, and work-limited results also return
false; false means that the caller must use its existing `ContactPair` path.
The query does not read or write `ContactPair`'s completed-report memo.

### 2. Transient pose and proof transfer

Build both transient bodies through the same private payload placement path
that `VerifyMotion` uses. Reserve a transient producer identity; never call
public `Transform`, `Copy`, `Intersect`, or `commit`. The true
query pose is the exact composition of the read `r3.Transform` entries with
the body's read placement entries. Bound the difference between that exact
composition and the float transient payload with the existing
`PoseDeviation` (`internal/pair/box`) and the placement machinery. Charge each body's displacement once
into every separation, witness-point, and contact-set bound. A held positive
gap becomes a proven positive gap only if its lower endpoint remains above
zero after both charges.

Transfer a held overlap to the query pose by subtracting the two bodies'
conservative swept-volume allowances from its lower volume bound. The
allowances use each body's area upper bound and its own pose deviation, as in
`agent-docs/linkage-check-design.md` Part M §5.1. A positive remainder proves overlap;
otherwise return `Undecided` unless another exact set certificate applies.
Transfer a held exact touch only if the complete contact and material-side
certificate also applies to the exact query poses. A nonzero placement or
composition displacement does not turn an `Exact` zero into an approximate
touch: the relation becomes `Undecided` unless a new exact posed certificate
proves it. In particular, the first box-impact slice requires an exact
axis-aligned posed-box contact path rather than relying on the existing
zero-displacement coplanar certificate alone.

For that path, admit only a rectangular prism whose recorded closed section
and extent prove its **occupied set** is exactly one box, with zero source
boundary displacement. A body's `Bounds()` rectangle or a box around its
facets does not pass this gate. Admit its existing placement and both query
poses only when every read rotation is an exact signed permutation of the
coordinate axes. Interpret their finite float entries as exact dyadic
rationals. Apply the composed transforms to the source box corners with
exact rational products and sums, without taking the rounded transient
payload's AABB as the answer. This constructs the exact occupied axis-aligned
box at the caller's query pose. The rational box can certify a face touch
despite nonzero `bodyGeom.delta` on a separately built transient body. If a
pose comes from a `PoseSegment`, also enclose the difference between its read
float transform and that segment's ideal screw pose using the
`VerifyMotion` pose-deviation machinery. A relation is transferred to the
ideal path only if the charged enclosure proves it; an exact rational touch
of the float pose by itself does not certify an ideal-path touch. Endpoint
equality has a separate exact check against the endpoint the caller stated.

### 3. Manifold contract

```go
// Shape only; shipped as report.ContactManifoldOf and report.ContactPointOf (§1).
type ContactManifold struct {
    Points []ContactPoint // nonempty, stable feature order
}

type ContactPoint struct {
    OnA, OnB       VecMeasurement // world positions; Length ball bounds
    Normal         VecMeasurement // unit vector A toward B; Dimensionless bound
    NormalAngle    units.Value    // proven angular radius, Angle
    Separation     Measurement    // signed Length, B minus A along Normal
    FaceA, FaceB   *Face          // original live faces when certified
    FeatureA, FeatureB ContactFeature // stable face/edge/vertex source identity
}
```

The public representation must keep all scalar quantities typed. `OnA` and
`OnB` each enclose a point on the corresponding true trimmed boundary. The
normal's ball bound and `NormalAngle` both enclose every true normal assigned
to that entry; the angular radius is at most the requested
`NormalResolution`. The point bounds are each at most `PointResolution`.
`Separation` encloses the signed normal projection of the two true witness
points, including their point and normal errors; positive means apart,
negative means penetration. For a certified touch its interval contains
zero, except at a lifted vertex of a support set (multibody §10.5), whose
interval encloses that vertex's positive exact height. A report may omit `FaceA` or `FaceB` only when the contact feature has no
single certified owning face; it must still carry stable source identity.
Neither a mesh triangle index nor a transient face pointer crosses the API.

`Manifold != nil` means every published entry passes these bounds and the
set is sufficient for the response law over the whole certified contact set.
For a planar patch, derive the exact trimmed intersection region and publish
all extremal vertices, including holes and disconnected components, in
deterministic loop order. A line contact publishes its clipped endpoints.
A point contact publishes one point. A finite solver reduction for a curved
contact must bound the force and torque effect of the omitted continuum; until
that reduction exists, relation proof may be published with `Manifold == nil`.
Duplicate geometric points from neighboring faces are merged only under a
certified equality; near coordinates never define identity. The manifold is
read-only, deterministic, and never implies a particular impulse.

One outward material normal per side is needed to orient the contact normal
from A toward B. A face interior with a smooth analytic normal can supply
one. At an edge or vertex, enumerate every adjacent face normal and its
material side; the candidate set is a cone, not an average. Publish a normal
only if a unique direction or a bounded cone meets `NormalResolution` and
the solver can consume all its admissible directions. Otherwise omit the
manifold with `ContactAmbiguousFeature`. A concave rim, multiple simultaneous
features, and zero-area coplanar trims follow this rule. Normal orientation
comes from body material sides, not triangle winding alone. Where one side
of the contact is a face interior and the touching certificate proves that
face's plane supports the other body at the feature, the face normal is the
unique admissible direction and is published, whatever the other side's
cone; two non-parallel crossing edges have one direction up to sign.
Part N §9.3 owns the per-feature table.

An overlapping pair may have a usable manifold only when the kernel proves
the boundary contact features and a bounded penetration direction/depth
that the response correction can use. A positive `Interference.Volume`, a
nesting witness, or intersecting held facets cannot manufacture those data.
Full containment without a separating boundary contact has no manifold.
The solver must decline that event rather than choosing a closest face.

### 4. Geometry paths

| Path | Relation proof | Witness and normal proof |
|---|---|---|
| Source-certified boxes at admitted poses | Exact rational interval relation | Rational face patches and clipped lines/points; exact material-side axis normals. |
| Analytic prism/revolve carriers | Existing clearance tiers, casts, and certified contact families | Retain admitted stationary feet and complete contact-set geometry; evaluate analytic normals with trim and orientation bounds. |
| Verified faceted boundary | Existing bounded pair proofs | Map candidates to original faces; charge source displacement and pose error; require source normal enclosures. |
| Unsupported or coarse payload | Existing box separation may still prove clear | Return relation with no manifold, or `Undecided` if the relation itself lacks proof. |

The analytic path extends private candidate results; it does not infer a
manifold from a public `Clearance` row. A candidate on an unbounded carrier is
admitted only when its feet are proved inside the trimmed source faces.
Enumerate face-interior, face-edge, edge-edge, face-vertex, edge-vertex, and
vertex-vertex tiers, including cone apex and spindle-axis singularities, as
`agent-docs/clearance-design.md` §3 requires. Proof of a global minimum still needs
every rival's lower bound. Stationary feet supply upper witnesses, never a
global minimum or a contact by themselves.

The bounded-mesh path requires `BoundaryVerified()` and finite private
`sourceBound(face)` for each candidate. `VolumeVerified()` is additionally
required for an occupied-volume overlap proof, but alone proves neither a
point nor a normal. A source-face displacement is a two-sided position bound,
not a normal bound: obtain a source analytic derivative enclosure, a
certified local normal cone, or refuse. A triangle normal with no such source
proof cannot enter a manifold. The source-face mapping, oriented material
side, and trim audit must pass; an inconsistent mapping is an evaluator error
with no report. Refine tessellation when a chord bound covers possible
contact and point/normal bounds may yet meet the request. At the deterministic
work or resolution floor, return an explicit undecided relation or absent
manifold according to which proof is missing.

The exact faceted lower-support path reads the Boolean payload's zero boundary
and occupied-volume bounds and its complete source Face map. It checks every
held vertex against the support plane, every support triangle's oriented
normal, and exact area coverage of one rectangular footprint. The source-box
floor's complete occupied interval lies on the opposite side of that plane.
The strict footprint containment gate makes the shared support rectangle the
whole contact set and supplies both original Face identities. Witness rounding
must fit `PointResolution`; failure keeps the proven `Touching` relation but
omits the manifold with `ContactPointTooCoarse`.
For a translation-only placed copy of a zero-bound Boolean, run the same exact
face proof on its saved pre-placement mesh after applying its exact accumulated
translation. Confirm that each current held vertex lies within the payload's
published boundary bound, that the triangle indices still match the source,
and that one rebuilt Face owns the whole support patch. This admits an exact
touch despite positive placement rounding and occupied-volume bounds. It does
not admit an arbitrary positive-bound Boolean or any rotated placement.

For two admitted rational boxes, compare all three axis intervals exactly.
`internal/pair/box` owns that interval classification and the bounded face-patch
calculation. The root contact path admits source bodies, maps face slots back
to original `*Face` values, and publishes the typed report.
A positive gap on any axis excludes contact, but a global minimum distance
still needs the full box-distance calculation. Strictly positive overlap on
all axes proves `Overlapping`; equality on at least one axis with no positive
separation proves `Touching` because the complete boxes have disjoint
interiors and their boundaries meet. Two or three equal axes are valid
edge/vertex **relation** certificates, even when the normal cone makes the
manifold unavailable. A single equality axis with strict projected overlap
gives an opposed-face patch. Clip the two rectangles with exact rational
arithmetic and use that patch for the manifold. Convert each rational point
to `r3.Vec` with an outward Length ball bound; its axis normal is exact.

For shallow box penetration, calculate the six directed translations that
would move B clear of A. A manifold is available only when exactly one
strictly smallest positive translation is certified, the selected faces
cross rather than one box being fully contained in the other, and their
projected overlap is strictly two-dimensional. Use the two original true
face points at each clipped patch vertex and the chosen A-to-B axis normal;
their signed separation is the negative translation depth, bounded by the
point conversion. A tied minimum, strict containment, edge-only projection,
or requested point resolution below the conversion bound leaves the
`Overlapping` relation intact and the manifold absent. This is the C1 box
penetration path consumed by a sweep's later proven-overlap endpoint.

For a source box at a general read pose, transform all eight recorded corners
with exact dyadic operations. The resulting parallelotope is tested on both
sets of face-normal axes and all nonzero edge-cross axes. Positive separation
on one axis proves `Separated`; equality on at least one axis with no positive
separation proves `Touching`; strict projected overlap on every axis proves
`Overlapping`. A separated report bounds the minimum distance from below by
the largest normalized axis gap and from above by a certified vertex-to-face
or vertex-to-vertex witness. If those bounds cannot publish a positive gap,
the relation is `Undecided`. For one opposed face equality, two boxes with
parallel corresponding edges use the first box's rational dual basis to clip
their complete face rectangles. Each clipped corner records its original
face identities, a rational point with outward conversion bound, and a
face normal with outward vector and angular bounds. A second equality axis,
nonparallel source edges, or a request tighter than those bounds withholds
the manifold. Two oriented boxes are also exact planar solids, so a touch or
overlap their box patches leave without a manifold takes the planar one
(Part N §9.3) when that relation agrees and
every point is an edge or vertex in, on or crossing the other box: an edge
of a rotated box on a floor, or that edge poking slightly through it. Face
pairs stay with the box patches, which withhold the degenerate ones. A
convex body's corner or edge poking through one face of a non-convex planar
body, a tray's floor, takes the face-local patch of Part N's §9.6:
the one crossed face, its deepest feature, the sunk part over the face, and
a clear column above it, each proved exactly.

The horizontal rotated-face path requires one source box at a signed-axis
pose and another whose transformed source edges keep one face horizontal.
At isolated touch, clip the rotated face's exact dyadic corners against all
four signed-axis face half-planes with exact rational intersections. Publish
every distinct polygon vertex when its signed area is nonzero and every
rounded witness meets `PointResolution`. Preserve both original face
identities and the exact vertical material normal. A zero-area projected
intersection keeps the proven relation without a manifold.

For penetration, the existing contained-face path requires all four rotated
corners strictly inside the other projected face. Its depth must be strictly
smaller than every side margin, and the bodies must cross at opposed Z
supports rather than contain one another. It publishes the four rotated
corners. A partly overhanging penetration or tilted face keeps its proven
relation without a clipped manifold.

#### 4.1 Source-box contact set for sweeps

Keep a private contact-set proof with the exact occupied interval of
each box on **all three** world axes at the certified pose. Retain the source
box corners, exact read start placements, original face identities, every
axis where opposed supports are equal, and each remaining axis's exact
projected overlap interval. It is a proof about the complete occupied sets,
not just the manifold's sampled patch. At one face touch, its contact set is
the Cartesian product of the two projected overlap intervals on the common
support plane. If one projected interval shrinks to a point, retain the
edge-contact set; if both do, retain the vertex-contact set. The public
`ContactPair` report states only the relation at this pose. Its manifold
may be absent at an edge or vertex even when this private set is complete.

The sweep may consume this private proof only when it certifies that its
ideal time-zero poses are the same exact read placements. For a path proved
to be pure affine translation, the sweep can advance every interval endpoint
by the certified translation at time `u`. The contact kernel supplies exact
support-plane and projected-interval facts; the sweep owns all comparisons
over time, event times, and status. In particular, the sweep can use an
outward lower bound `w > 0` on the relative translation rate along an opposed
support axis. The whole-body gap along that axis is then at least `w*u` for
every `u > 0`. This is a one-sided departure proof even though the gap at
`u = 0` is exactly zero. Keep the product symbolic until a positive lower
gap at a chosen `h` is representable; a rounded zero proves nothing.

When the normal support gap stays zero, the exact projected intervals let
the sweep prove persistent face contact during tangential sliding. At an
exact projected-overlap endpoint equality, the complete contact set becomes
an edge or vertex. The sweep may resume clear-interval proof only after a
separate axis has a strictly positive gap. A positive normal velocity at a
single manifold point, sampled separated poses, a rounded near-equality,
or a rotating path cannot substitute for these whole-box proofs. If no
strict interval claim follows, the sweep returns its undecided outcome.

#### 4.2 Source semicircle sphere against a source box

Admit a full-revolution solid with one untrimmed spherical face only when its
recorded section is one complete semicircular arc and its on-axis diameter.
The arc's center and endpoints must lie exactly on an axis with exact cardinal
direction; the two axial radii must match as rationals. Require zero section
displacement and signed-permutation frame and placement. A query pose with an
arbitrary proper rotation is admitted when the placed sphere center is exactly
the query origin; the occupied ball then depends only on the query translation.
Other sphere query poses require a signed-permutation rotation. A sphere
surface tag without this occupied-set record cannot enter this path.
The exactly orthogonal rotated-box extension keeps the sphere pose gate and
allows the source box's query pose through its exact read-corner proof.

Map the recorded center and radius to exact rational world coordinates. Clamp
the center to the exact box intervals and compare squared distance to squared
radius. A strict excess proves separation, equality proves touch, and a
strict deficit proves overlap. A separated report encloses the true gap after
outward square-root and float conversion. A one-axis exterior center with its
whole projected radius strictly inside the other box intervals can publish
the opposed box face and spherical point. Shallow penetration also requires
the sphere not to reach the opposite box face. An edge, corner, internal
center, or wider patch keeps the proven relation but withholds the manifold.
Reversing body order reverses the exact axis normal and witness fields.

For an exactly orthogonal rotated source box, minimize squared distance from
the source sphere center to its exact read-pose parallelotope. Project the
center onto each pairwise orthogonal, nonzero edge with exact rational dot
products, clamp each coefficient to `[0, 1]`, and sum the squared distance to
the resulting point. Compare it with the exact squared radius to classify the
complete occupied sets. Bound a positive gap with outward square-root conversion.
An isolated face manifold requires the center outside one face, an orthogonal
projection strictly inside that face, and the projected sphere disk strictly
inside both face intervals. The opposite support must remain farther than one
radius during penetration. Convert the plane foot and spherical point with
outward position bounds; normalize the exact face cross product with a bounded
normal and angle. A skew exact read box returns `Undecided`; an edge/corner
projection, internal center, or coarse manifold request keeps any relation
proved by the exact closest-point calculation without inventing a point.

#### 4.3 Two source semicircle spheres

Admit each sphere through §4.2's source record and pose gates. Compare the
exact squared distance between their centers with the square of the radius
sum. A strict excess proves separation; equality proves touch; a deficit
proves overlap. Enclose a separated gap by outward square-root conversion.
Publish one point manifold when the center difference is nonzero and its
distance exceeds the absolute radius difference. The latter gate excludes a
sphere wholly inside the other. Enclose the center-line unit normal by a
bounded square root of the exact squared distance. Both witnesses lie on the
original spherical faces along that direction, with point bounds including
normal conversion. Their signed separation encloses center distance minus
radius sum and both witness errors. A request tighter than the normal or point
bounds keeps the relation but withholds the manifold. Reversing body order
reverses the normal and swaps witnesses.

#### 4.4 Source cylinder against a box face

Admit the full circular source prism or the full rectangular-profile source
revolve of the source-cylinder path only at signed-axis poses. Each occupied
set is the exact product of the recorded disk and axial interval. Require the
complete projected disk to lie strictly inside both projected intervals of
one source-box face. An axial support gap then
measures the true minimum distance. Equality proves the complete circular
face touching the box face. Strict crossing of the near supports proves
overlap only while the cylinder's far support remains beyond the same box
face and its near support remains outside the opposite box face.

Identify the original planar cylinder end face by its signed world material
normal. Reject a missing, duplicate, or non-axial end face.
The disk center is an exact interior point of both opposed faces and supplies
one bounded representative witness per body. The private disk and axial
interval certify the complete contact set. Reverse body order by swapping
the witnesses and reversing the normal. Withhold the manifold if a face
identity, point bound, or shallow-crossing proof fails.

For the full circular source prism's sidewall with vertical cylinder axis,
require its complete axial interval and other transverse diameter strictly
inside one box face. The
exact circle support on that face's normal axis identifies the complete
axial contact line and the source cylindrical Face. Publish one bounded
midpoint witness on the line, the opposed original Face identities, and the
exact transverse normal. A shallow crossing must leave the far support
outside the same box face and the near support outside the opposite face.
Lateral edge contact, tilted axes, revolved sidewalls, and an embedded
cylinder return `Undecided`; the identity-pose analytic fallback may prove a
relation without a manifold.

#### 4.5 Analytic ruling contact

At identity query poses, `ContactPair` reads the clearance kernel's verdict;
placed poses take the direct proof at the end of this section.
A touch the kernel proves with a ruling certificate (`agent-docs/clearance-design.md`
§6) keeps the two carriers, the exact separating plane and the exact ends of
the ruling segment on it: a plane face against a cylinder face along the
tangent ruling, or two parallel external cylinder faces along their common
ruling. The kernel admits the relation for any body pair it models. The
manifold needs the complete contact set as well.

Each cylinder side must be a full source cylinder, read through §4.4's source
record: the exact disk-by-interval box gives its axis, radius and axial
interval. Its occupied set then meets the separating plane in exactly its
tangent ruling, between the two cap rims. A plane side holds that whole ruling
inside its own trimmed face, which the certificate proved, so the contact set
of a plane and a cylinder is the cylinder's ruling. Two cylinders touch on the
overlap of their two rulings, which must have positive length. The exact ends
recomputed from the source records must equal the kernel's ends; any
difference withholds the manifold. A plane side maps to the one original
planar Face whose outward normal and plane offset equal the certificate's
exactly; a cylinder side maps to its one cylindrical Face.

The manifold publishes the two ends in exact coordinate order, each one
point on both bodies with an outward rational-to-float position bound. Each
entry's normal is the tighter of the two sides' `Face.NormalAt` balls at the
witness, oriented from A toward B. A cylinder reading taken at a rounded
witness also charges the radial tilt to the true end: radial vectors at most
`e` apart, the true one of length `r`, have unit directions at most `2e/r`
apart. Both balls must contain the exact separating normal, or the entry is
refused. The angular radius is four times the ball radius, as for the
oriented-box normal. `Separation` is an `Exact` zero, since both witnesses
are the same true point. A body that is not a full source cylinder keeps the
proven touch with `ContactPayloadUnsupported`; an unmatched face or end gives
`ContactAmbiguousFeature`; a point or normal bound beyond the request gives
`ContactPointTooCoarse` or `ContactNoNormalProof`. Reversing body order swaps
faces and reverses the normal; the ends keep their order.

**Transformed poses.** Elsewhere, a full source cylinder `M` at any pose with a
positive determinant against an exact planar body `S` with zero held
displacement (multibody §9, §10.4), at its own pose, reads the plane/cylinder
ruling directly (`internal/contactruling/placed.ClassifyPlacedRuling`). `M`'s identity record gives its
axis `â`, radius `r`, length `L` and end-disk centers; the query pose maps
the centers exactly to `c±`. The pose's float basis `B` is orthonormal only
to rounding, so the occupied set is the identity cylinder's image under the
exact float map, whose section is the disk's image under `B`. With `gram`
the largest absolute row sum of `BᵀB − I`, every unit `m` has `|Bᵀm|²` in
`[1 − gram, 1 + gram]`; `gram` must be at most `1/16`, which every valid
`r3.Transform` meets. The support plane is the face plane of `S` whose
outward normal `n̂` is a signed axis, with `|α| <= 1/4` for `α = n̂·Bâ` and
its triangles in one face, whose larger `|H±|` is least,
`H± = n̂·c± − d − r`; the first in triangle order wins a tie. A plane with a
vertex of `S` strictly in front of it, a tray's floor, is face-local and must
pass multibody §10.6's column test over the coordinate box of `M`'s eight
staged identity-box corners, which records its lateral clearance `m`.
Height is affine along the axis, so `M`'s least height above the plane is the
lesser of its end disks', each `n̂·c± − d − r·ρ` with `ρ = |P·Bᵀn̂|`, `P`
removing `â`. A signed-axis pose has `ρ = 1` and `α = 0` exactly, so the
least height is the lesser `H±` and proves `Touching` or `Separated` exactly.
At any other pose, `|ρ² − 1| <= gram + α²` gives
`|r − r·ρ| <= r·(gram + α²)`; with `δ = r·(gram + α²) + |α|·L`, a least
height above `δ` is `Separated` with its interval as the gap, and one within
`δ` of zero is `ContactBand` with `Gap = [−c₀, c₀]`,
`c₀ = max|H±| + r·(gram + α²)`. A depth beyond `δ` stays `Undecided`. Each
end's true lowest point lies within `r·(3·gram + (3/2)·|α|)` of `c± − r·n̂`
(with `AAᵀ = I + E`, `‖E‖ <= gram`, the lowest point is `c − r·Ay/|y|`,
`y = P·Aᵀn̂`, and `|Ay − |y|·n̂| <= 2·gram + α² + |α|·√(1 + gram)` over
`|y| >= √(7/8)`), and both points' feet, grown by that drift, must lie inside
`S`'s face, so the plane holds the whole contact ruling and the gap is the
least height. The manifold publishes the two points in exact coordinate
order, each with that drift as its ball and its foot on the plane as the `S`
witness; the normal is `S`'s exact face normal, which is `M`'s own normal at
its true lowest point, and `Separation` is the band, or an exact zero for a
touch. Reversing body order swaps witnesses and reverses the normal. On a
face-local plane a gap's lower end is the lesser of the least height's and
`m`, its upper end unchanged, and a touch or band publishes only when `m`
exceeds its half-width; a cylinder nearer a wall than that is `Undecided`.

### 5. Contact cases and ordering

| Case | Required result |
|---|---|
| Positive gap, including a body inside a cavity | `Separated` only after nesting or shell tests exclude shared interior; publish a bounded global minimum gap. |
| Strict full containment of a solid in material | `Overlapping`; no contact manifold from containment alone. |
| Coplanar positive-area patch with opposed material normals | `Touching` after all other crossings are excluded; publish the full trimmed patch reduction if its bounds meet the request. |
| Coplanar patch with matching material sides | `Overlapping` only when shared interior is proven; no touch manifold from the patch alone. |
| Edge-only or vertex-only meeting | `Touching` on the exact source-box and exact planar paths; `Undecided` under today's other clearance paths. A manifold requires a resolved normal cone. |
| Smooth tangency or ruling contact | `Touching` only for a complete certified family in clearance §6; the manifold may still be unavailable if point or normal bounds fail. |
| Transversal boundary crossing | `Overlapping` if the proof transfers to both exact query poses; the unique shallow source-box penetration path above may yield a manifold. Other crossings require a separate proof. |
| Near miss inside displacement bounds | `Undecided`; never promote a held-mesh gap or sample to `Separated`. |

Input order is preserved: reversing A and B swaps point and face fields,
reverses normals, and leaves the geometric relation and gap interval
unchanged within bounded arithmetic. No contact point order may depend on a
map iteration, BVH traversal, worker completion, or tessellation cache hit.
Sort source features by original body topology order, then exact parameter
order along each feature, then exact rational coordinate order for tied
generated points. Stable tie breaks use A before B. Repeated calls with the
same inputs publish identical reports, including reasons and bounds.

### 6. Refusal, cancellation, and non-mutation

Expected geometric limits return `Undecided` or a known relation with an
absent manifold and a typed `ContactReason`. These include a staged payload,
an uncertain trim, bounds spanning zero, a missing normal enclosure, an
ambiguous edge cone, and a requested resolution the current proof cannot
reach. An internal invariant failure, such as a missing source face or a
broken exact predicate, returns an error and no report. A point/normal bound
must be finite; an unbounded result is absence, never `+Inf` in a published
manifold. Context cancellation at any phase returns `ctx.Err()` and no
report. Poll within candidate, tessellation, containment, and refinement
loops under the shared work-counter discipline of clearance §3 and
interference §7.2.

The query changes no document membership, liveness, topology, payload,
measurement, or producer identity. Per-body tessellation caches may accept
only a complete successful mesh under their existing contract; partial or
canceled work never enters a cache. The report holds copies of result slices
and values. It may point only to original bodies and their original source
features, never transient ones. `ContactPair` is safe for concurrent reads
under the existing `Document.Verify` concurrency rule. Each body keeps the
last 1024 reports it was the first operand of (`contact_pair.go`),
keyed exactly on the second body, the bits of both poses and the request;
a body is immutable, so a repeat reads the stored report and changes no
outcome. Only a completed query is stored, never an error or a
cancellation, and every call returns its own copy of the report.

The proof paths fill the report's node form: `report.ContactReportOf`
instantiated with topology nodes, which each kernel package names
`contactReport`. The public `ContactReport` is the same generic type
instantiated with the root handles, so the two forms share one definition and
one set of readings. `ContactPair` lifts a completed report to the handle
instantiation once, with `handleLift.Report` (a `report.Lift` value), before the memo stores it: each node
maps to the handle that holds it, so every body, face, edge and vertex a
report names is the very pointer the body's own accessors return, and a nil
manifold, gap or overlap stays nil. A sweep that runs the same proof paths
lifts their reports in the same way where it publishes a `SweepEvent`, a
contact track's features or a track manifold.

### 7. Delivery and computed tests

The stage names C1-C4 group the contact paths. The table gives each stage's computed obligation and the
tests that pin it; a stage or clause with no pinning test says so.

| Stage | Delivered path | Computed obligation | Pinning tests |
|---|---|---|---|
| C1 | Exact posed source boxes and read-only pair report | Check a gap, face touch, shallow overlap, containment refusal, tied depth, and document state. | `TestContactPairSourceBoxes`, `TestContactPairShallowOverlapAndRefusals`, `TestContactPairSignedAxisPose` (`apitest/contact_pair_test.go`) |
| C2 | Analytic prism/revolve contact families | Check point, ruling, patch, and near-tangent refusal against calculated geometry. | `TestContactPairCylinderRulingOnFloor`, `TestContactPairCylinderPairRuling`, `TestContactPairRulingRefusals`, `TestContactPairPlacedRuling` (`apitest/contact_analytic_manifold_test.go`); `TestContactPairRevolvedCylinderAxialDiskFace` (`apitest/contact_cylinder_test.go`); `TestContactPairSourceSphereAndBox` |
| C3 | Verified faceted source witnesses and certified normal cones; the exact planar algorithm is Part N §9 | A curved source lies within each point ball; its analytic normal lies within the angular cone; a render-only mesh and a displacement bound without normal proof yield no manifold. | The `TestContactPairPlanarManifold*` tests (`apitest/contact_faceted_manifold_test.go`) and `TestContactPairExactPlanar*` (`apitest/contact_faceted_pair_test.go`) cover the exact planar half; no single test pins the curved-source point-ball clause. |
| C4 | Remaining shipped solid payloads and cost pass | Each payload computes certified contact or gives its typed missing-proof reason. | No test enumerates the payload kinds. `TestExactPlanarPairAdmitsStitchedSolid` covers the stitched solid. |

C1 uses two 10 mm boxes and reads a 3 mm gap (`TestContactPairSourceBoxes`). A face touch reads a zero gap
and four patch corners. A unique shallow overlap reads negative separation and the expected axis normal.
Strict containment and tied translation depths have no manifold. Each C1 test compares `doc.Bodies()` before
and after its queries. No test asserts that the next producer identity is unchanged. C2 evaluates a
plane/sphere point, a plane/cylinder ruling, and an opposed coplanar patch; the tests above pin the point
and ruling, and `TestCoplanarContactFaceToFace` (`apitest/coplanar_contact_api_test.go`) pins the patch. No
test named here pins void-wall orientation.

C1 also checks the private complete-set certificate with two opposed boxes. An outward relative translation
proves positive gap for every positive time. A tangential translation retains a face patch until its projected
overlap ends, reports an exact edge-contact set at the exit, and proves positive gap only afterward. A point's
positive normal velocity alone must not pass either test. The sweep tests `TestSweepPairSourceBoxPersistentSlide`
and `TestSweepPairSourceBoxSlideEdgeTransition` (`apitest/contact_sweep_test.go`) exercise this certificate
through `SweepPair`.

Every stage also checks pair reversal, deterministic ordering, wrong kinds, invalid/retired/foreign/sheet
bodies, cancellation during a real candidate walk, and non-mutation. `TestContactPairSourceBoxes` checks
reversal, `TestContactPairPlacedBodyAndInputGates` checks the input gates, and
`TestContactPairExactPlanarCancels` checks cancellation on a real walk. At least one test passes a real
`ContactPair` manifold into a dynamics response and checks the resulting position and velocity
(`examples/dynamics_box_collision_example_test.go` and the `dynamics/` tests); a hand-written manifold does not
cover that boundary. Tests at the point and normal resolution thresholds assert both sides of the gate: a finite bound just within
the request publishes an entry, and a wider bound omits it without changing a proven relation.
`TestContactPairRulingResolutionGates` pins the publishing side and the `coarseReq` case in
`TestContactPairSourceBoxes` pins the omitting side. No test named here pins the rule that a small positive
held gap whose placement allowance spans zero never reports `Separated`; the rule stands as a design contract.

## Part S — Two-body contact sweep

This part specifies the `Document.SweepPair` contract. Current affine
source-box sweeps certify first impact, immediate departure, persistent face
contact, and the first edge transition of a sliding patch. A source semicircle
sphere against a source box certifies affine or centered rotating first impact
and face-point departure or persistent touch while its projected radius remains strictly
inside one box face. A sphere against an exactly orthogonal rotated source
box also certifies a strict single-face affine clear span, first impact, or
separating departure. A full source cylinder certifies a clear path,
first face impact, and separating departure inside a source-box face, including
transverse translation that stays inside the face, and a persistent track for
an end disk resting or sliding on that face. A full circular vertical
source prism also certifies the same outcomes at its sidewall when its complete
axial contact line stays inside a broad source-box face.
One zero-bound faceted Boolean solid with a certified rectangular lower face
can certify an affine vertical clear path, first exact face touch, separating
departure, or persistent face contact against a containing source-box floor.
The same path accepts a translation-only `Transform` result when its saved exact
source mesh proves the complete lower face despite positive held-mesh and
occupied-volume bounds. The event and replay retain the placed body's Face.
A positive-bound mesh `Union` also takes this path when an exact source-box
lower face survives a second operand whose certified lower extent is strictly
above that face. Its lower patch must still map to one live faceted Face.
Both bodies must translate equally in X and Y, so the complete support patch
stays strictly inside the floor face. The lower support plane must start at or
above the floor's upper plane. A first-impact bracket ends at exact touch;
if its rational contact fraction is not a representable dyadic grid point,
the sweep returns `Undecided`. A bracket ending after support-plane crossing,
a lateral path, an unproved support face, and a footprint reaching the floor
edge also return `Undecided`. The contact track retains the original faceted
Face. Rounded replay checks the exact support-plane path and footprint proof.
A positive-bound faceted Boolean without an exact lower support proof can certify
only a strict clear span above a source-box floor. Both signed-axis paths must
have zero angular and transverse motion. The held vertex extrema, widened by
the certified boundary
displacement, stay inside the floor's projected face. The lower Z gap is affine;
both endpoints must exceed the displacement, including conversion error.
Both real endpoint contact queries must report separation. Replay charges the
rounded pose deviation and requires the widened actual extents to stay inside
the floor face with a positive gap. A path reaching the boundary allowance,
moving transversely, or rotating returns `SweepUndecided`, unless both bodies
are exact planar solids, which take the general sweep below.
Two source semicircle spheres certify affine first impact
through exact squared-distance motion, including transverse crossing, and
separating departure. Their exact isolated interior tangent publishes a
bounded point with strict separation on both sides. A touching source-sphere
pair with equal exact affine displacement certifies a full-span point track.
When one or both source spheres follow rigid drifts that spin about their own
centers, the same proofs certify a clear path, first impact, isolated graze,
and separating departure. Their sampled poses retain spin; the occupied balls
follow exact affine center paths.
Rotating source-box rigid drifts can also certify a clear path or bracket an
impact after exact oriented-box pose relations, a bounded float-to-ideal pose
difference, and whole-body travel bounds. An initial source-box face touch also certifies immediate
departure when both bodies have the same angular velocity and the bounded
normal separation rate is positive. Fixed oblique poses of co-oriented source
boxes also certify initial face touch and a persistent face patch when both
affine paths have the same translation. A partly overhanging horizontal
source-box face also certifies initial touch and a persistent clipped polygon
under the same translation. If their exact support-plane gap has
positive affine slope, the sweep certifies immediate departure and checks the
rounded endpoint against its ideal path. A box spinning about world Z can also
reach a contained horizontal face on a stationary box, then depart under
positive vertical velocity. A rotating `PoseSegment` whose read screw axis is
cardinal can certify a clear path or first impact. An axis-normal source face
can also certify departure when its support-plane gap increases throughout
the step. A source box spinning about Y can depart from a stationary horizontal
source-box face when every source corner has a positive bounded outward height
through the certified interval. Two exact planar solids
(Part N §9) that no path above admits run the
same search over every vertex under any rotating or affine path: a clear
path, a first impact bracketed onto an overlapping sample, or an initial
touch or overlap; the general rotating sweep, §10.1 there, owns it. From an
initial touch it proves §10.2's departure or, under `ContinueCertifiedTouch`,
§10.3's persistent touch or band track; two source boxes whose box proofs
cannot continue a touch rerun on that path. Its reports replay rounded poses
from recorded gap bounds and pose deviations. A body whose held mesh carries
a positive displacement takes the same run with §10.4's band: an impact
brackets onto a `ContactBand` sample, a band start never departs, and its
track's depth widens by twice the displacement. A full source cylinder whose
ruling touches a face of an exact planar body, or lies in the placed band of
contact-geometry §4.5 at a turned start, continues, under
`ContinueCertifiedTouch` and a spinning drift, with §10.4's rolling band
track over the ruling's two ends. Other unequal spins, rotating paths, and
payloads are not implemented.
An extruded circular source prism or a full revolve of an axis-incident
rectangular half-profile can certify a strictly separated axial affine path
against a containing source-box face. The sweep uses the source-derived outer
disk box at both endpoints and over the full translation, and replay charges
the rounded pose deviation before returning a clear sample. Both sources
can also certify a first face impact, separating departure, and an end-disk
persistent track. Near contact,
lateral motion outside the containing face, partial revolutions, and other
revolved profiles return `Undecided` on this path.
Part C owns the package
boundary and Part G owns relation and manifold
proofs at one pose. This document owns the paths, continuous clear certificate,
first-contact search, and sweep report. `agent-docs/linkage-check-design.md` Part M supplies
the existing one-mover proof pattern; its `VerifyMotion` API stays separate.

Navigation only: paths and API → §1–2; pose accounting → §3; continuous bounds
→ §4; search and report → §5–6; work and errors → §7; computed tests → §8.

### 1. Claim and ownership

`SweepPair` reads two live, sound solid bodies of one `Document`, each under its
own prescribed path over one shared positive duration. It changes neither body
nor document. It returns the earliest **certifiable** outcome:

| Outcome | Claim |
|---|---|
| `SweepClear` | The bodies have strictly positive separation at every time in the closed step. |
| `SweepDepartedClear` | The pair starts in certified touch, departs immediately, and has strictly positive separation at every later time in the step. |
| `SweepPersistentTouch` | The pair stays in certified touch throughout the step, with a stable source feature set and a bounded manifold track. |
| `SweepPersistentBand` | Appended after `SweepGrazingTouch`. The pair stays within a published depth of one certified normal throughout the track, with stable source features and a bounded manifold track; Part N §10.3 owns the certificate. |
| `SweepContactTransitionBracket` | The pair starts touching and a bracket encloses the first change in contact features, patch structure, or departure from touch. The preceding contact track is certified. |
| `SweepImpactBracket` | The clear prefix may follow a certified departure from initial touch. The left pose is separated and the right pose certifies touch or overlap. Width is at most `TimeResolution`. |
| `SweepGrazingTouch` | One exact interior instant has certified touch. Both open sides of the closed step are strictly separated; no overlap or other touch occurs. |
| `SweepInitiallyTouching` | Contact is certified at time zero. It says nothing about closing velocity or a later impact. |
| `SweepInitiallyOverlapping` | Interior overlap is certified at time zero. |
| `SweepUndecided` | The available proof cannot establish one of the above. The earliest unresolved interval and a structured reason are returned. |

The pair order is the caller's `(a, b)` order. Normal directions in any contact
report therefore point from A toward B. A certified relation does not imply
that the contact kernel supplied a manifold usable for dynamics. The solver
must inspect manifold availability separately; it never steps through an
undecided interval or treats a later overlap sample as the first impact.

### 2. Paths and entry point

```go
func (d *Document) SweepPair(ctx context.Context, a, b *Body,
    pathA, pathB PairPath, req SweepRequest) (*SweepReport, error)

// PairPath is sealed. The permitted values are PoseSegment and
// RigidDriftSegment. Each has one Duration of Kind Time.
type PairPath interface {
    pairPath()
}

type PoseSegment struct {
    From, To r3.Transform // relative to the body's current placement
    Duration units.Value // positive Time
}

// QuantityVec belongs to decad so geometry cannot import dynamics.
// Every component has the stated kind; all constructors validate it.
type QuantityVec struct { X, Y, Z units.Value }

type RigidDriftSegment struct {
    From            r3.Transform // relative start pose
    Center          r3.Vec       // world pivot at the start, millimetres
    LinearVelocity  QuantityVec  // Velocity, world coordinates
    AngularVelocity QuantityVec  // Angle / Time, world coordinates
    Duration        units.Value  // positive Time
}

type SweepRequest struct {
    ContactRequest
    TimeResolution    units.Value // positive Time, <= Duration
    MaxPoseEvaluations uint64     // >= 2; counts distinct pair times
    StartPolicy       SweepStartPolicy
}

type SweepStartPolicy int // StopAtInitialContact (zero),
                          // ContinueSeparatingTouch, ContinueCertifiedTouch
```

Both paths must name equal physical durations, compared as exact rational base
time values read from their units, without a tolerance. Their time fraction
`s = t / Duration` is shared; their transforms are evaluated independently at
that fraction. A stationary path is legal. `PoseSegment{From: p, To: p}` is
constant, and a drift with both velocity vectors zero is constant. A mixed
handedness screw segment is invalid; two proper endpoints or two reflected
endpoints are legal. A drift composes proper rotations onto its `From`, so a
reflected start stays reflected. Dynamics may impose a narrower no-reflection
gate on its own state.

For a `PoseSegment`, read `rel = From.Inverse().Then(To)` and `rel.Screw()` once.
At `s` the float pose is `From.Then(screw.At(s))`. At exactly `s = 0` and `1`,
return the stated `From` and `To`, as `Between.PoseAt` does. `From == To` skips
decomposition and returns `From` at every sample. A nonidentical pair whose
read screw has zero angle and slide is likewise constant; its endpoint
deviation still covers the stated `To`. The path takes r3's deterministic
shorter arc, including its half-turn axis choice. A caller needing a longer
rotation splits it into segments.

For `RigidDriftSegment`, let `u` be elapsed time. Its world-space step rotates
by `|ω|u` about the line through `Center` along `ω`, then translates by `v u`.
The resulting pose is `From.Then(step)`, where `r3.RotationAround`,
`r3.Translation`, and `Transform.Then` construct the float step. A zero `ω`
uses translation alone. `Center` is a path pivot, not an inferred mass property;
the dynamics layer ensures it matches its bounded center-of-mass state. This
path realizes `c(u) = Center + v u` and `R(u) = Rotation(ω, u) R(0)` for an
isolated rigid drift. Replacing it with endpoint screw interpolation would
move the center along a different path when `v` is perpendicular to `ω`.

Both variants compose onto each body's existing placement in the same order
as `Transform`. No path is constructed by subtracting one body's pose from
the other's at the endpoints. The relative transform can have nonlinear
motion even when both individual paths are simple.

### 3. Exact path and sampled pose

Treat every finite input float as the exact rational it denotes, and interpret
its unit through the exact or outward-enclosed unit factor. For a screw path,
the ideal transform is the exact screw of the **read** `Axis`, `Point`, `Angle`,
and `Slide`, composed after the exact `From`, exactly as motion-check §5.1
defines. For a drift, the ideal transform uses the stated world pivot and the
exact typed velocity components, with the axis normalization, angle, sine,
cosine, and any unit factor enclosed outward. The ideal body at each `s` is
that transform composed onto the exact read current placement.

The evaluator sees a float `r3.Transform` instead. For body `i` and evaluated
fraction `s`, calculate a point-displacement allowance `η_i(s)` between the
float placed body and its ideal body. Use motion-check §5.1's matrix-difference
times record-coordinate radius plus translation-difference form, with rational
intervals for every operation. Both bodies contribute: the ideal pair gap
interval is the float-pair gap interval widened by `η_A + η_B`. If either
radius or deviation is unbounded, the pose cannot prove a gap or contact.
For a rotating source box, compare all eight exact staged source corners under
the read query pose with their ideal-path intervals. The largest outward-rounded
corner distance bounds every point of the box. This uses the same staged
placement and pose operations as `ContactPair`; a float-composed transform can
round away an error that remains in those source corners. An exact planar
body compares every vertex of its snapshot the same way; the body lies in
their hull, where the affine difference of the two images is largest at a
vertex.
The stated screw `To` gets the motion-check §5.1 endpoint allowance against
both the ideal end and exact `To`; a drift has no separate `To` promise.

A float-pose touching result is not automatically ideal touching. The
contact kernel must supply a certificate that survives both pose deviations
or evaluates the exact source geometry under the stated pose. Otherwise the
ideal relation is `Undecided`. A float-pose overlap transfers when both pose
deviations are zero, or when its bounded volume exceeds both bodies'
boundary-sweep allowances, using motion-check §5.1's area scaling for each.
A separate exact source-set proof may establish overlap at the ideal pose.
An overlap known only by a witness cannot cross a nonzero pose deviation
without such a proof.
Contact geometry's exact-rational source-box path provides the first placed
box touching certificate; ordinary `bodyGeom.delta` clearance (contact geometry §2) alone does
not certify touch under a displaced transient placement.

`ContactPair` speaks about the exact query pose named by its two read float
transforms. `SweepPair` separately transfers its result to the ideal paths and
records that adapted relation in `SweepEvent`. A manifold's point and normal
bounds must include placement and path-pose deviation before the sweep can
publish it. The adapted relation may be certified while those bounds are too
wide for the requested `PointResolution` or `NormalResolution`. Never infer a
contact normal from an overlap volume or from the direction between centers.

### 4. Conservative continuous bounds

#### 4.1 Travel of each body

Read each body's `Bounds()` at its current placement, inflate its coordinates
by its outward `Bound`, and map all eight corners through the exact `From`.
Their coordinate hull contains the ideal start body. For a screw with axis line
through `Point` along `Axis`, let `ρ` be the maximum distance of those mapped
corners from that line, rooted upward. Distance from a line is convex, so no
interior point has a larger radius. Let `θ` and `d` be the read screw's angle
and slide. Over a fraction span `h`, every point travels at most

```text
τ_screw(h) = h (ρ θ + |d|).
```

For a drift, let `ρ` be the maximum distance of the same mapped corners from
`Center`; this is convex too. Let `V` and `Ω` be outward bounds on the norms
of the exact world linear and angular velocity vectors in mm/s and rad/s.
Over elapsed span `q`, every point travels at most

```text
τ_drift(q) = q (V + ρ Ω).
```

The same radius works throughout each path: screw motion preserves distance
to its axis; drift rotation preserves distance to its moving center, and the
center's translation is charged separately. Reflections in `From` do not alter
either argument. A stationary path has zero travel. Roots, products,
trigonometric factors, and final travel are rounded outward, never merely
evaluated in float and compared as though exact. If a finite bound cannot be
formed, the result is `SweepUndecided`, not a clear certificate.

The mapped corners, the pivot and the axis are all dyadic, so every corner's
squared cross product with the axis is exact dyadic arithmetic over the one
denominator `|axis|²`; the largest is chosen by that numerator and divided
once before the upward root, which is the rational the per-corner quotients'
maximum is. A step reads a path's radius for its swept box and again for its
sweep, so each body keeps a bounded set of recent radii (`radiusmemo.RadiusCap`), keyed by the `From` and pivot
bits and the exact axis (`internal/radiusmemo`, read by `internal/sweeppath/radius_reader.go`).

#### 4.2 Swept boxes

Build one conservative box per body from the exact `From`-mapped inflated
corner hull, expanded on every axis by its full-path travel bound. Compare
the two boxes using exact rational extremes. Strict separation on any axis
proves `SweepClear` without pair-pose evaluations. Box faces merely meeting
does not prove touch, clear, or overlap. Reading the original rest boxes and
expanding only by `To - From` is invalid when either `From` moves a body before
the path begins. A BVH or sweep-and-prune may discard only with this strict
box certificate; it cannot change pair or sample order. The box is a public
read-only query, `Document.SweptBox`, with `SweptBox.StrictlyDisjoint`;
Part N §4.2 owns its shape and §4.3 the
sort-and-sweep `dynamics` runs over it.

#### 4.3 Time interval certificate

At interval ends `l, r`, obtain proven positive lower gap bounds `g_l, g_r`
for the **ideal** placed pair. Let `T_A(l,r)` and `T_B(l,r)` be each body's
travel bound over the interval, and `T = T_A + T_B`, rounded upward. For every
time `u` inside, the true pair distance is at least each endpoint's lower gap
minus the travel from that endpoint. The interval has strictly positive gap
throughout when

```text
g_l + g_r > T.
```

The comparison uses exact rationals with outward bounds. A conservative lower
bound on the interval gap is `(g_l + g_r - T)/2`, rounded downward, and may be
loose. Neither two separated endpoint samples nor touching boxes prove this
inequality. No tolerance turns a failed strict comparison into `Clear`.
If an endpoint is touching, overlapping, or undecided, this positive-gap
certificate cannot use it. A different continuous geometric proof may settle
the interval only if the contact kernel explicitly supplies one.

For a Z-axis rigid drift against a stationary source box, compare the two
exact source-box Z supports after the stated vertical displacement at each
dyadic fraction. Rotation leaves these supports unchanged. A float-pose
horizontal manifold transfers to the ideal path only when every rotating
face corner stays strictly inside the stationary face after both pose
deviations and witness bounds are charged. A negative exact support gap
proves a shallow ideal overlap even if the rounded pose touches. A zero gap
proves ideal touch. Widen every transferred point by both pose deviations.
An exact initial support equality and positive vertical relative speed prove
a whole-body open-time gap; then the usual interval bounds certify the
remaining path. A partial patch or a tilted rotation has no such proof.

#### 4.4 Source sphere and box face corridor

For the source ball admitted by contact geometry §4.2, an affine sweep can
use one box face when the sphere center plus and minus its radius stays
strictly inside both projected box intervals at the start and end. The four
signed endpoint inequalities prove that containment for every intervening
time. The normal support gap is then one exact affine function. Its positive
sign throughout proves clear; a positive start gap and a decreasing gap give
the exact first-contact root. Choose a dyadic bracket with a strictly clear
left endpoint and a right endpoint beyond that root, no wider than the caller's
time resolution. The right margin keeps the float query on the overlapping
side when ideal and rounded poses differ by a small amount.

For a shortened impact prefix, the final fraction may replace the usual right
dyadic sample when the root lies before that endpoint and the shortened
bracket fits `TimeResolution`. The real endpoint sample must still report a
matching shallow sphere manifold. Replay caches the original sphere and box,
the face corridor, and the exact affine support gap. At an interior fraction,
check the rounded placements against the ideal path within `PointResolution`,
then require the same face corridor and the outcome's support-gap relation.
Refuse replay when either check fails.

At each sample, classify the ideal rational sphere and box independently of
the float `ContactPair` query. Transfer the float manifold only when relation,
source features, and normal agree; widen its two witness bounds and separation
bound by the exact pose difference. An initial touch with an increasing normal
gap proves immediate departure. A zero normal gap and the same face corridor
prove a full-span point track. A path leaving the corridor returns
`SweepUndecided`; a center sample alone cannot certify the missing span.

A `RigidDriftSegment` may rotate the sphere when its pivot equals the source
sphere center, that center equals the query pose translation, and the other
path is affine. Its angular velocity changes the
sampled pose, but the occupied ball follows the exact affine center path from
its linear velocity. Form the sampled proper rotation through `r3`, then set
the center translation from that affine path so rotation rounding cannot turn
an exact touch into a crossing. Use the same face-corridor, support-gap, and
point-track proof as the translating sphere. Replay recomputes the rotating
pose and checks its center against the cached affine path and point resolution.
Refuse a different pivot, an unrepresentable center displacement, a rotating
box, or a path outside the face corridor.

For an exactly orthogonal rotated box, require the same exterior source face,
strict projected disk margins, and opposite-face clearance at both affine
endpoints. Those inequalities are affine, so both endpoint proofs cover the
whole span. Compare the squared exact center-to-face support with squared
radius times squared face normal; this decides clear, touch, and overlap
without rounding the normal length. Binary search dyadic fractions for the
first sign change, then leave a right-side overlap margin within
`TimeResolution`. The real right sample must retain the same bounded point
manifold. Replay compares rounded source corners and sphere center against
the cached ideal affine path, then rechecks the same face corridor and support
relation. A skew box, edge/corner corridor, non-affine pose, or lost rounded
manifold returns `SweepUndecided` or `ErrUnsupported` as appropriate.

#### 4.5 Affine source-sphere pair

For two source balls admitted by contact geometry §4.3, require the relative
center displacement and initial center difference to use the same sole
cardinal axis. The signed center distance minus the radius sum is affine up
to first contact. A positive value throughout proves clear; an initial touch
with an increasing gap proves immediate departure. A decreasing positive
start gap has one exact first-contact root. Bracket it with dyadic fractions,
including a root at the final endpoint, with a clear left sample and a
touching or overlapping right sample. The endpoint can be ideal overlap
while the rounded pose is touch. Transfer the source manifold only when both
queries name the same source faces. For a cardinal center line, keep the exact
normal when both queries retain the same cardinal direction. For an off-axis
center line, bound the change in unit normal from the exact center-pose
difference and shorter center-line length. Charge that change to normal,
witness, angle, and separation bounds before publishing the event manifold.

For a transverse relative displacement, compare the exact quadratic squared
center distance minus squared radius sum over the held affine path. Its minimum
proves a full clear span or locates the earliest possible impact. Search the
decreasing side with dyadic fractions; require a separated left endpoint, a
touching or overlapping right endpoint, and width at most `TimeResolution`.
Find a hidden pass-through before reporting clear. An unrepresentable shallow
overlap returns `SweepUndecided`. An exact isolated tangent can use §4.5.1;
other tangent paths remain undecided. The bounded
center-line source manifold gate applies at the right sample. A zero-gap touch
with equal exact affine displacements and one bounded source point publishes a
full-span track. Its exact center-distance polynomial is identically zero.
Both endpoint samples must retain the same source faces. An interior track
query translates both cached spheres by their common exact displacement and
reduces them to a bounded point manifold. Replay requires exact touch of the
rounded pair. Unequal displacements remain undecided for persistent
continuation.

A centered rotating `RigidDriftSegment` may enter this sphere-pair proof under
`StopAtInitialContact`, or after initial point touch under
`ContinueSeparatingTouch`. Require each rotating sphere's exact source center
to equal both its query translation and stated pivot. Require each full linear
displacement to be representable as a dyadic rational. §2's drift rotates
about the line through its pivot and then translates, so a pivot at the ball's
center moves that center along exactly `Center + v u`; the rotation leaves the
occupied ball unchanged. An off-center pivot moves the center on a circle and
returns `SweepUndecided`. The exact squared center-distance polynomial of the
affine paths then decides every outcome above: a clear path, an initial touch
or overlap, a first-impact bracket, an isolated graze by §4.5.1, or a
departure. Each sample keeps the real rotation and rebuilds its center from
the exact affine path, rounding each coordinate once, so rotation rounding
cannot move it. The usual float-to-ideal manifold transfer charges the
remaining center difference. The real start pose must report the ideal
relation, and a clear path's real end pose separation. Replay checks rounded
poses against the same polynomial.
A separated or overlapping start under `ContinueSeparatingTouch`, and an
unresolved or closing departure, return `SweepUndecided`.

#### 4.5.1 Isolated sphere-pair graze

For a separated affine source-sphere pair, form the exact rational polynomial
`q(s) = |(centerB−centerA) + s(deltaB−deltaA)|² − (radiusA+radiusB)²`
from the held source centers, radii, and translations. Publish
`SweepGrazingTouch` only when its quadratic coefficient is positive, its
discriminant is exactly zero, and its sole minimum `s*` is strictly inside
`(0,1)`. Prove `q(0)>0` and `q(1)>0`. These facts prove `q(s)>0` on both open
sides of `s*` and `q(s*)=0`; separated endpoint samples alone do not prove it.
The same source-sphere pair proof must classify the exact minimum as touching
with one bounded point manifold. The manifold's normal, source faces, and
witness bounds pass §4.5's float-to-ideal transfer at the minimum.

Require `s*` to have an exact finite `units.Scalar` representation and an
exactly representable positive event time `Duration × s*` before publishing
the outcome. The report keeps the private rational fraction even though its
public `SweepInstant` uses `units.Value`. Evaluate the real `ContactPair` at
`0`, `s*`, and `1`; require separated endpoints and a touching float pose at
`s*` with the same source faces. A float overlap, missing manifold, or
insufficient point or normal resolution makes the sweep `Undecided` with its
earliest unresolved interval. `MaxPoseEvaluations` must fund all three poses;
the request's existing minimum of two remains valid, but two poses cannot
certify this outcome.
Use `SweepEventUnrepresentable` when the exact fraction or event time cannot
be published, `SweepPoseBudget` when a required pose cannot be evaluated,
`SweepPoseRelation` when the float relation disagrees, and
`SweepContactUnsupported` when the event manifold cannot be transferred.
The unresolved interval contains `s*` and no later event is published.

This outcome applies to affine `PoseSegment` and `RigidDriftSegment` paths
admitted by §4.5, including centered spinning drifts. It does not infer a
graze for another rotating path, source-box pair, sphere-box pair, nonrepresentable minimum,
initial touch, or final-instant touch. A positive quadratic minimum is
`SweepClear`; a negative minimum enters the existing earliest-impact search.
An initial touch follows §5.1, regardless of later motion. If a different
pair kernel cannot prove the full two-sided relation, it returns
`SweepUndecided` rather than this outcome.

#### 4.6 Source-cylinder box-face path

For a full circular source prism or axis-incident rectangular full revolve
and one source box at signed-axis poses, construct exact outer boxes from the
source disk, axial limits, and recorded placements. Admit an affine clear
sweep only when the cylinder's projected disk stays strictly inside the box
face at both path endpoints. Check that
one axial gap between the complete swept outer boxes exceeds
`PointResolution`. The two endpoint `ContactPair` queries must also prove
separation, and their rounded pose differences must fit `PointResolution`.
Both admitted source cylinders enter the axial impact path after an
outer-box intersection. The impact path admits transverse relative translation
when the cylinder's outer disk box stays strictly inside the same box face at
both endpoints. Each transverse edge difference is affine, so strict endpoint
containment proves containment throughout the interval. The initial axial
support gap must be nonnegative.
The signed axial support gap is affine. A decreasing positive gap supplies
its exact first-contact root, including equality at the final endpoint. Use a
dyadic bracket with a separated left
sample and a touching or shallow-overlapping right sample whose original
face identities and signed normal match the posed `ContactPair` query.
Transfer that query's bounded witness to the ideal path by charging the
rounded pose difference. Under either continuation policy, an initial touch
with increasing support gap proves immediate departure. Under
`ContinueCertifiedTouch`, an initial end-disk touch whose support gap has
zero slope (equal axial displacement, any transverse translation inside the
corridor) is a full-span persistent track: the gap is zero at every fraction,
and strict endpoint containment keeps the complete disk inside the same face,
so the same two original faces and exact normal hold throughout. Every
coordinate of the disk-center witness lies within the endpoint envelope of
both outer boxes. One outward ULP at its largest magnitude, as a point-ball
radius, must fit `PointResolution`, so `ManifoldAt` can read a bounded point
at any fraction. The final real sample must also touch with the same faces.
`ManifoldAt` translates both cached sources by the exact fraction, reclassifies
them, and returns the single disk-center point. A closing touch, a stationary
touch under `ContinueSeparatingTouch` (`SweepDepartureUnproved`), a sidewall
line touch, a near gap without a clear margin, or a path leaving the face
corridor returns `SweepUndecided`. Exhausting the pose budget at the bracket's
right sample returns `SweepUndecided` with `SweepPoseBudget` and the
unresolved bracket.

Replay translates the cached source outer boxes with the held affine paths.
At each requested fraction, the rounded cylinder projection must remain
inside the rounded box face and the pose difference must fit
`PointResolution`. A clear path keeps a support gap above that difference.
Departure requires exact touch at the start and positive ideal and rounded
gaps afterward. A persistent track's rounded gap must lie within the pose
difference of its zero ideal gap. Before an impact bracket, replay requires a
positive gap; inside the bracket, the ideal support gap must fit
`PointResolution`.

For one full circular source prism with a vertical cylinder axis, the same
affine proof may select a horizontal axis instead of the cylinder axis. Its
relative axial displacement must be zero. Its complete axial interval
and other transverse diameter must stay strictly inside the box face at
both endpoints. The contact query supplies the source cylindrical Face,
box Face, one bounded midpoint on the complete axial contact line, and the
exact transverse normal. The selected circle support is the exact outer-box
extremum, so its signed gap is affine and the impact, departure, and replay
checks above apply. A revolved sidewall, rotating cylinder, or line reaching
the face edge returns `SweepUndecided` on this path.

### 5. Earliest-event search

Evaluate time zero first unless swept-box exclusion has already proven the
whole path clear. Return `InitiallyOverlapping` on certified initial overlap.
At initial touch, the default `StopAtInitialContact` returns
`SweepInitiallyTouching`; the two continuation policies use §5.1. If the zero
pose is undecided, return `SweepUndecided` with its cause. Evaluate the far
endpoint next and maintain the samples in increasing dyadic fraction order.

Process intervals from left to right. A certified clear interval extends the
clear prefix. For the first interval not certified clear:

1. If it has a separated left endpoint, a certified touching or overlapping
   right endpoint, and exact duration no greater than `TimeResolution`, return
   `SweepImpactBracket`. Its left endpoint is the end of the clear prefix.
2. If its duration exceeds `TimeResolution`, evaluate its dyadic midpoint and
   process the left half before the right half. This order searches for an
   earlier hidden contact even when the old right endpoint overlaps.
3. If its duration is at the floor, and no right endpoint certifies contact
   or overlap, return `SweepUndecided` for that interval.

Continue only when an interval has become certified clear. If every interval
becomes clear, return `SweepClear`, or `SweepDepartedClear` after a certified
departure from initial touch. A tangential graze may have no overlap
sample. The exact sphere-pair proof in §4.5.1 can publish
`SweepGrazingTouch`; otherwise it leaves an undecided floor interval.
Separated samples alone cannot turn it into `Clear`.
If midpoint arithmetic cannot produce a distinct dyadic fraction, return
`SweepUndecided` with `SweepFractionFloor`. The search never skips an earlier
uncertified interval to publish a later collision as the first event.

The procedure also covers two moving bodies that pass through one another
and separate between initial and final samples: their combined travel keeps
the interval undecided until a contact sample is found or the floor reports
uncertainty. A contact's impulse decision remains with dynamics; the sweep
proves only the geometric continuation of the stated paths.

#### 5.1 Continuation from initial touch

`ContinueSeparatingTouch` and `ContinueCertifiedTouch` both require a new
one-sided certificate after an initial `Touching` relation. A private
departure proof supplies a duration `h > 0`, a proven positive gap lower bound
at `h`, and a global lower-gap function `L(u) > 0` for every `0 < u <= h`.
The proof must cover the **complete** initial contact set and all other body
features. Positive normal velocity at one manifold point is insufficient.
A general admissible form is `L(u) >= c*u - K*u*u` with proven `c > 0`,
`K >= 0`, and `c - K*h > 0`; a kernel may instead provide a stronger direct
bound. Select the largest representable dyadic fraction `h/Duration` within
the proved horizon, and evaluate its endpoint. If no positive fraction can be
represented, return `SweepUndecided` with `SweepFractionFloor`.

The initial source-box path in contact-geometry §4.1 proves departure for
certified pure affine translations. At time zero, an opposed support axis `n`
has equal A maximum and B minimum. An outward lower bound `w > 0` on
`(v_B - v_A)·n` proves the **whole-body** gap at least `w*u` for every `u > 0`.
Either body may move. The exact source boxes and placed poses are required;
transient AABBs with displacement bounds cannot establish the equality. A
rotation, tangential speed on that axis, an uncertain equality, or `w <= 0`
does not pass this source-box departure path.

For co-oriented source boxes at a fixed oblique pose, use the exact dyadic
cross product of two source edges as the common support-plane normal. A
four-point initial face manifold and exact equality of opposed support
projections identify the complete touching face. A positive exact dot product
of relative affine displacement with the outward normal proves a strictly
positive support gap for every positive path fraction. Check the chosen
rounded departure sample with its float-to-ideal pose bound. Every later
clear interval must keep this exact support gap positive at both endpoints;
affine motion then proves it positive throughout the interval.

For a stationary horizontal source-box face and a moving source box whose
only angular component is about world Y, read all eight exact staged moving
corners and the exact drift pivot. The initial four-point manifold must have
the same exact vertical normal at every point, and opposed full-box Z supports
must be equal. For each corner, bound its outward height derivative by the
exact `sign * (v_z - omega_y * (corner.x - center.x))`. Require the minimum
derivative `c` over all eight corners to be strictly positive. Bound the
absolute height acceleration of each corner by
`omega_y^2 * (abs(corner.x - center.x) + abs(corner.z - center.z))` and use the
maximum as `K`. The stationary support plane and the initially nonnegative
corner heights then give a full-body gap at least `c*u - K*u*u/2`. Admit a
dyadic horizon only when `K*h < c`; reduce the horizon when the full duration
fails. The rounded horizon sample must retain a positive gap after pose
deviation, and later intervals need their own continuous clear certificates.
The same `c`, `K` form over every vertex of any exact planar body, with both
bodies moving, is Part N §10.2; its band track
for a contact whose `c` is zero within the solver's residual is §10.3, and
a start within the request's `SupportBand` continues over the support set
of §10.5, holding a lifted vertex that closes no faster than the request's
`RestSpeed` on both sides of the plane (§10.8). The support plane may be one face of an owner with material in
front of it, a tray's floor, under the column test of Part N's §10.6,
which also caps the lower-gap function by the lateral clearance.

After departure, mark `(0,h]` as certified clear, put a separated sample at
`h`, and run §5's earliest-first search on `[h, Duration]`. If no later
contact appears, return `SweepDepartedClear`; if later contact appears, return
`SweepImpactBracket` with the initial touch retained separately. The clear
claim is open at zero: neither result calls the closed `[0, Duration]` path
strictly separated. If the departure proof fails under
`ContinueSeparatingTouch`, return `SweepUndecided` with
`SweepDepartureUnproved`; never discard the initial touching pose and start
searching at a positive time by sample choice alone.

`ContinueCertifiedTouch` first accepts the same departure proof. Otherwise
it asks for a complete persistent-contact track. A valid track proves that
the pair stays `Touching`, with opposed material sides and no overlap, at
every instant in its interval. It must provide a bounded manifold at every
instant that the dynamics solver will use. If the track reaches `Duration`
without changing source features or patch structure, return
`SweepPersistentTouch`. A stationary resting stack is this case. If no such
track can be proved, return `SweepUndecided` with
`SweepContactTrackUnproved`. Tangential velocity alone is no proof of
persistent contact.

For two source-certified boxes under pure affine translation, contact-geometry
§4.1 supplies exact axis intervals. Persistent face touch requires equality
on one opposed support axis for the whole tracked interval and strictly
positive projected overlaps on both other axes. Each projected overlap is
the minimum of two affine upper endpoints minus the maximum of two affine
lower endpoints. Enumerate exact rational roots where endpoint order changes
or projected overlap reaches zero. Between consecutive roots, the active
source features and clipped patch vertices follow fixed affine formulas;
check their inequalities at both ends with outward bounds. This certifies
the complete contact set and a manifold track for resting or planar sliding.
An endpoint-order tie at time zero is resolved by its right-sided derivative
when the patch and normal are continuous. It is recorded in the initial
track, rather than returning a zero-time transition that a solver would hit
again on every restart. If the right-sided contact set or normal cannot be
certified, return `SweepContactTrackUnproved`.
If an edge or vertex limit occurs, classify that exact limit as touching,
then test the next open interval for persistent contact or positive gap.
Do not infer separation merely because the face patch shrank to zero.

Return `SweepContactTransitionBracket` at the **first** root that changes a
source feature, clipped patch structure, or touching/separated relation.
Bracket the exact rational root by adjacent representable dyadic times with
width at most `TimeResolution`; a dyadic root may use a zero-width bracket.
The report certifies the contact track before the bracket and the relation
at both bracket samples. This is a solver restart point, not an impulse by
itself. On edge exit, an exact equality at the root and a strict projected
gap afterward prove the one-sided departure; a later recontact is searched
on the remaining interval after the restart. If a root cannot be bracketed
within the resolution or budget, return `SweepUndecided` with the earliest
unresolved interval. Curved, rotating, or non-box contact may remain
undecided until their own contact-track proof exists.

The dynamics solver invokes `ContinueSeparatingTouch` after resolving an
impact whose next drift begins at certified touch. It invokes
`ContinueCertifiedTouch` for supported resting or sliding contacts. It may
advance across a contact transition bracket only when its time-travel and
state residual gates cover the bracket; otherwise the step is undecided.

### 6. Report shape and ordering

```go
type SweepOutcome int // Clear, DepartedClear, PersistentTouch,
                      // ContactTransitionBracket, ImpactBracket,
                      // InitiallyTouching, InitiallyOverlapping, Undecided,
                      // GrazingTouch, PersistentBand; zero is invalid

type SweepCause int // None, PoseRelation, MissingBound, TimeFloor,
                    // FractionFloor, PoseBudget, ContactUnsupported,
                    // DepartureUnproved, ContactTrackUnproved,
                    // EventUnrepresentable

type SweepInstant struct {
    Fraction units.Value // Dimensionless dyadic fraction, canonical time
    Elapsed  Measurement // Time, outward bound on Fraction × Duration
}

type SweepInterval struct {
    From, To SweepInstant
}

type SweepDeparture struct {
    Until SweepInstant
    GapAtUntil Measurement // strictly positive lower gap at Until
}

// SweepContactTrack holds an immutable private continuous certificate.
// Accessors return copies; no exported field can change that certificate.
type SweepContactTrack struct { /* private source snapshot and proof */ } // report.SweepContactTrackOf, handle instantiation

func (t *SweepContactTrack) Start() SweepInstant
func (t *SweepContactTrack) End() SweepInstant
func (t *SweepContactTrack) Features() (ContactFeature, ContactFeature)
func (t *SweepContactTrack) Normal() VecMeasurement
func (t *SweepContactTrack) ManifoldAt(fraction units.Value) (*ContactManifold, error)
func (t *SweepContactTrack) Band() *Measurement // nil for an exact touch track; multibody §10.3
func (t *SweepContactTrack) BandAt(fraction units.Value) (*Measurement, error) // the band over [start, fraction]; nil for an exact touch track

type SweepSample struct {
    At           SweepInstant
    PoseA        r3.Transform
    PoseB        r3.Transform
    FloatContact *ContactReport // query at the two read float transforms
    Ideal        SweepEvent     // relation transferred to the ideal paths
}

type SweepEvent struct {
    At       SweepInstant
    Relation ContactRelation
    Gap      *Measurement // separated or certified touching only
    Overlap  *Measurement // bounded ideal overlap, when proven
    Manifold *ContactManifold
    Reason   ContactReason // when relation or manifold is unavailable
}

type SweepReport struct {
    A, B            *Body
    PathA, PathB    PairPath
    Request         SweepRequest
    Outcome         SweepOutcome
    Bracket         *SweepInterval // ImpactBracket or ContactTransitionBracket
    Unresolved      *SweepInterval // only for Undecided
    InitialEvent    *SweepEvent // certified touch/overlap at time zero
    Event           *SweepEvent // later impact, graze, initial event, or transition right sample
    Departure       *SweepDeparture // only after one-sided proof
    ContactTrack    *SweepContactTrack // complete certified touching prefix
    Cause           SweepCause     // only for Undecided
    Samples         []SweepSample  // increasing Fraction, no duplicate time
    BoxExcluded     bool           // true only when both swept boxes prove clear
    PoseEvaluations uint64
}

func (r *SweepReport) HasAffineReplayProof() bool
func (r *SweepReport) BracketEndsAtDuration() bool
func (r *SweepReport) CertifiedPosesAt(elapsed units.Value) (r3.Transform, r3.Transform, error)
func (r *SweepReport) CertifiedPosesAtInterval(time, start, end units.Value) (r3.Transform, r3.Transform, error)
```

`SweepReport` is `report.SweepReportOf` instantiated with the root handles,
and `SweepEvent` and `SweepSample` are its event and sample records; the
fields and methods above are defined there. The record also carries the
producer's replay proof and exact bracket right fraction, and a sample its
exact fraction, behind accessors that take an `internal/seal.Key`, so only
decad's own producers can read or set them.

`SweepPair` validates the bodies, paths and request, builds the report over
the bodies' topology nodes, and hands it to `internal/sweeprun`, which selects
the producer and fills it. The run takes two hooks from the document:
`ContactPair` in node form, with its recent-report memo, and the verified
mesher the held-body paths use. `report.LiftSweepReport` then maps the finished
node report to the root handles, keeping an event that is also the initial
event as one event.

`BracketEndsAtDuration` compares the producer's private exact bracket right
fraction with one. The public `Fraction` may be rounded.

For `SweepGrazingTouch`, `Event` is the exact interior touching sample.
`Bracket`, `Unresolved`, `InitialEvent`, `Departure`, and `ContactTrack` are
nil; `Cause` is `SweepNoCause`. `Samples` includes the separated start and end
and the touching event in increasing fraction order. The outcome is not an
impact bracket: it supplies no closing velocity and authorizes no impulse.
`SweepGrazingTouch` follows `SweepUndecided` in the enum, and
`SweepEventUnrepresentable` is the last cause value. Reversing A and B keeps the same event fraction
and reverses the bounded contact normal.

`CertifiedPosesAt` evaluates the same float path used by the sweep at
the requested elapsed time. For affine paths it checks the read float poses
against the cached exact source boxes, spheres, or cylinder outer box.
The sphere-pair path keeps
both source centers, radii, exact translations, and impact bracket. At an
arbitrary interior fraction it compares both rounded centers with their ideal
rational centers, then checks the exact squared center distance against the
radius sum. For a centered rotating drift, it evaluates the recorded rigid
rotation and exact affine center displacement before the same comparison. A
clear or departing sample must retain a positive gap after
charging the center displacement; an impact prefix stays clear before the
bracket and stays near contact at its right endpoint. A sample beyond the
bracket is unsupported. It refuses when total displacement exceeds
`PointResolution` or changes a clear or departing relation. For source-box
persistent contact and event brackets, it accepts only a rounded relation
within that resolution of the exact source-box relation. It does not call `ContactPair` or
read the document. It refuses times outside the sweep or outside a certified
event prefix. `CertifiedPosesAtInterval` maps exact held time values in a
specified interval onto the certified spatial path. A dynamics trace uses
its recorded event and step endpoints for that interval, avoiding a gap
when the rounded sweep duration differs from their exact difference.
For a grazing outcome, replay compares the exact squared center distance
with the squared radius sum on the whole interval. At `s*`, both ideal and
rounded squared center distances must equal the squared radius sum, and the
rounded source faces must match the event. At every other fraction, both
squared distances must be strictly greater than the squared radius sum, and
the rounded center deviation must fit `PointResolution`. The cached
quadratic establishes the ideal sign between queried times. If floating pose
construction moves the rounded pair across or away from touch at `s*`, replay
returns `ErrUnsupported`. The proof never turns a near graze into an exact
event.
For a clear rotating source-box drift, the report retains the sweep's exact
source corners and ideal-path bounds. Replay checks the rounded oriented boxes
with the exact separating-axis test. Their positive gap must exceed the total
bounded corner difference from the ideal poses, and that difference must fit
`PointResolution`. A rotating impact report replays its certified clear
prefix through the bracket's left edge `lo`, and inside the bracket through
its right edge it replays a fraction `f` when `(f − lo)·T − g + η` fits
`PointResolution`: `g` the left sample's proven lower gap, `T` both bodies'
travel bound per unit fraction, and `η` the rounded pose's deviation. Every
ideal point lies within `(f − lo)·T` of its place at `lo`, where the pair was
`g` apart, so the rounded pair lies within that bound of a separated pair,
the claim the affine source-box replay makes inside its bracket. A left
sample without a positive gap replays only through `lo`. `T` bounds every
point's travel in any direction, so for a fast or far-reaching body a bracket
`TimeResolution` wide can carry more travel than `g` plus `PointResolution`,
and its right edge would refuse. The rotating search therefore narrows a
found bracket below `TimeResolution` while its right edge fails that check,
read with the deviation replay will charge there: a meeting midpoint becomes
the right edge, and a separated one the clear certificate joins to the left
edge becomes the left edge. The bracket found stands when narrowing reaches
the float floor or the pose budget, or meets a midpoint that settles neither,
so narrowing never turns a bracket into `SweepUndecided`. The check that
stops narrowing admits nothing; replay's own check stays the gate.
Rotating departure reports replay their separated interior after the
producer's one-sided departure proof. Both paths check the exact oriented-box
gap against their staged corner deviation before returning a rounded pose.
For a co-moving oriented source-box persistent face track, replay uses the
same cached source corners and exact time fraction. The rounded poses must
fit `PointResolution` and yield a four-point patch on the producer's face
pair within its point and normal bounds. A changed face, missing patch, or
exceeded bound returns `ErrUnsupported`.
A general planar sweep (Part N §10.1) replays
without rerunning the pair relation, so its cost is one pass over both
vertex sets. Both rounded poses must fit `PointResolution` of their ideal
poses. On a clear interval or a departure the rounded pair must keep a gap:
the sweep's proven lower gap at that time must exceed the summed vertex
deviation, plus each positive-displacement body's displacement times its
rounded pose's stretch plus one. Inside an impact bracket the bracket bound
above applies with that deviation and displacement charge as `η`. A touch or
band track checks every rounded held vertex height against the held band
widened by that deviation.
Each report keeps its most recent replay answers, poses or refusal, keyed by the exact fraction, in a
bounded memo (`sweepreplay.ReplayPoseCap` answers; `internal/sweepreplay/`, `internal/ringmemo`). The replay
proof is complete before the report reaches a caller and reads none of its public fields, so a repeat returns
the same answer and changes no outcome. The memo is mutex-guarded, since `Trace.Sample` and `Step` may replay
one report concurrently.

`Fraction` and the input `Duration` define the exact search time; `Elapsed`
is a bounded convenience reading for callers. Bracket width is checked from
those exact fractions and the exact read duration, not by subtracting rounded
`Elapsed.Value` fields. A solver uses the fractions and duration to make its
own conservative time choice. `SweepSample.FloatContact` is non-nil for every
evaluated pair pose, including an undecided one, and names the original bodies.
Samples are sorted after refinement, independent of evaluation order. The
bracket's right sample and `Event` are the same ideal-path finding. `InitialEvent`
keeps the time-zero relation when continuation finds a later event. A
`Departure` certifies positive gap on `(0, Until]`, not at time zero. A
`ContactTrack` certifies touch from time zero through `End()`. It owns copies
of the source geometry and proof, so changing fields in another report or
retiring a body later cannot alter its result. `ManifoldAt` requires a
Dimensionless fraction in `[Start().Fraction, End().Fraction]` and returns a
fresh bounded manifold. A
`ContactTransitionBracket` carries both `ContactTrack` and `Bracket`, and
`Event` holds the right sample's new relation. `BoxExcluded`
reports the whole-path shortcut; its `Samples` is empty and
`PoseEvaluations` is zero.

For an undecided pose at time zero, `Unresolved` is the zero-width interval at
zero. For a missing continuous proof, it is the earliest floor interval. For
budget exhaustion, it is the earliest not-yet-certified interval. The cause
does not replace the contact kernel's structured diagnostic; `Event` is nil
when no contact is proven. Repeated calls with identical inputs produce the
same outcome, sample order, bracket, and diagnostic order. Any spatial index
or parallel work must preserve that published order.

### 7. Validation, cancellation, and cost

Validate both bodies, both path variants, durations, request units, finite
values, and positive limits before reading `ctx`. Reject identical body
pointers, foreign or retired bodies, non-solid bodies, a zero/invalid
transform, a nonpositive duration, unequal durations, mixed-handedness screw
endpoints, a nonfinite pivot or velocity component, wrong quantity kinds, an
invalid resolution, or `MaxPoseEvaluations < 2` with the matching existing
sentinel (`ErrDegenerate`, `ErrForeignBody`, `ErrRetiredBody`, `ErrUnitKind`,
`ErrNotFinite`, `ErrNegativeMagnitude`, or `ErrUnsupported`). A valid sound
solid whose pair kernel lacks a proof returns `SweepUndecided`, not an input
error. An internal invariant failure remains an error with no report.

`SweepPair` passes the validated request, a `report.SweepRequest`, to the sweep
path it selects. The public `SweepRequest` is an alias of that type, so the
request, its contact request and its start policy have one definition. When a
rotating source-box run ends undecided with `SweepDepartureUnproved` or
`SweepContactTrackUnproved` at a touching or `ContactBand` start,
`planarContinuation` reruns the pair through `sweepPlanarPair` with the
report's saved `Request`. The report's `Request` holds the caller's own value.
A `ContactPair` query made by a sweep receives the contact request unchanged;
the departure-gap query of a rotating run clears its `SupportBand` first.

The dyadic grid depth is `ceil(log2(Duration / TimeResolution))`. Fully
exploring it evaluates at most `2^depth + 1` distinct times, if that count is
representable. `MaxPoseEvaluations` caps this work without silently relaxing
`TimeResolution`. When the cap is reached, return `SweepUndecided` with
`SweepPoseBudget` and the earliest unresolved interval. Reuse one contact
result per pair time; use the existing bounded kernel work counter and
`ctx` checks within each pose. A caller controls expensive kernel work with
the context deadline, as interference §7.2 does. Do not claim that a pose
budget caps each pose's geometric cost.

A rotating run re-reads the same fractions and poses as it refines, samples
and tracks: each prepared path keeps a memo of its ideal pose by fraction,
its staged points' deviation by pose and fraction, and its point spans by
fraction span (`planarsweep.BodyPath`, `internal/planarsweep/body_path.go`).
A served deviation still charges the poll once per point. The run closes
both memos before it returns, so the path copies a proof keeps compute
afresh and hold no memory. The radian
sine and cosine enclosures behind them are a pure function of the exact
angle, read through one process-wide, mutex-guarded memo bounded by `radianSinCosMemoCap`
(`internal/motionbound/`). Every memo hands out copies, so no caller can
change what another reads, and none changes a value.

After validation, check `ctx` before each pose and at each subdivision;
pass it into the contact kernel. Cancellation returns `ctx.Err()` and a nil
report, even if an earlier prefix was clear. A contact-kernel invariant error
also returns no report. Never call public `Transform`, `Copy`, `Intersect`,
or a commit path. Snapshot tests must show that document membership/order,
body liveness, and next producer identity are unchanged after clear, impact,
undecided, error, and cancellation results.

### 8. Computed tests and first increment

Use real decad bodies, real pose evaluation, and the real contact kernel.
Assert computed locations, brackets, and bounds, not only enum values.

| Fixture | Required assertion |
|---|---|
| Two 10 mm boxes: A moves `+100 mm/s` from `x=[0,10]`; B is fixed at `x=[20,30]`; duration `0.2 s` | First contact is `0.1 s`. At `0.001 s` resolution, the bracket encloses it and names the original bodies. |
| Both boxes move: A as above, B starts at `x=[30,40]` and moves `-100 mm/s` | First contact is `0.1 s`. Dropping B's travel from the interval certificate must make the test fail. |
| A and B both drift at `+100 mm/s` with a 10 mm initial gap | `SweepClear` holds despite both bodies moving. |
| A box starts under `From=Translation(100,0,0)` and moves 30 mm toward a wall at `x=125` | Swept boxes cannot exclude the wall. A slab beside the original rest box at `x=[20,30]` is excluded. |
| A thin blade rotates while a small pin moves across its path | Coarse separated samples yield `SweepUndecided`; a finer resolution finds a bracket. Neither path may be replaced by endpoint relative interpolation. |
| A drift has `v=(10,0,0) mm/s`, `ω=(0,0,π) rad/s`, and center at the origin | At `0.5 s`, its center is `(5,0,0)` and an initial point `(1,0,0)` reaches `(5,1,0)`. Endpoint screw interpolation disagrees. |
| Initial cap touch, initial positive overlap, and an unsupported tangent graze | Touch and overlap return their distinct initial outcomes; the graze returns `Undecided`, never `Clear` from samples. |
| Two radius-5 mm source spheres: A stays at `(0,0,0)`, B starts at `(20,10,0)` and moves `(-40,0,0)` mm in 1 s | The exact polynomial is `1600(s−1/2)²`; `SweepGrazingTouch` reports a bounded point at `s=1/2`, separated endpoints, no bracket, and full-span interior replay. Reverse pair order and compare event time and reversed normal. |
| The same pair with B starting at `y=11`, `y=9`, or `y=10` and `x=20` moving to `x=0` at the final instant | The first path is `SweepClear`; the second enters impact search; the endpoint tangent is `Undecided`. |
| Two radius-5 source spheres spin about their own off-grid centers while their centers close transversely or along X | The bracket holds the closed-form root of the affine center paths, and the event's witnesses, normal and separation bound the exact ideal pair. Replay keeps spin and refuses past the bracket; an off-center pivot is `Undecided`. |
| A graze with an event fraction or elapsed time that `units.Value` cannot represent exactly, a rounded event pose that loses exact touch, or a two-pose budget | Return `SweepUndecided` with the earliest interval and specific cause; never publish `SweepClear` or a false graze. |
| A 10 mm box touches a fixed floor, then moves upward at `50 mm/s` for `0.1 s` | Default mode returns `InitiallyTouching`. `ContinueSeparatingTouch` returns `DepartedClear`, with a positive final gap enclosing `5 mm`. |
| The same touching boxes slide tangentially while their face patches overlap | `ContinueSeparatingTouch` returns `Undecided`; `ContinueCertifiedTouch` returns `PersistentTouch` until the first patch-feature change. |
| A radius-5 source cylinder rests on its end disk on a fixed floor, still or sliding `(8,-4,0) mm/s` for `1 s` | `ContinueCertifiedTouch` returns `PersistentTouch`; `ManifoldAt` puts the disk-center point at the slid center on the floor plane in both body orders. A closing path or a disk leaving the face is `Undecided`. |
| A top box starts at `x=[0,10]` on an equal fixed box, then slides `+5 mm/s` for `3 s` | The right-sided initial patch is admitted. `ContactTransitionBracket` encloses the edge exit at `2 s`; no earlier transition occurs. |
| A rotating body starts in touch without a one-sided contact-set proof | Both continuation policies return `Undecided`; positive velocity at one witness cannot certify the complete contact set. |
| Equal stationary segments and two same-handed reflected segments | Stationary paths are valid; reflected paths are accepted by geometry when the kernel can prove their relation. A mixed-handedness screw path returns `ErrDegenerate`. |
| Missing contact manifold at a proven overlap endpoint | The report keeps `ImpactBracket` and marks the event manifold unavailable. The four-body step takes the manifold `ContactPair` publishes at the rounded right-end poses, the poses it publishes (multibody §5 step 7), and refuses when that has none either. |
| Tight pose budget and cancellation during pair evaluation | The budget returns `Undecided` with earliest unresolved interval; cancellation returns `ctx.Err()` and nil report. Both preserve document state. |

The first end-to-end test uses the first box fixture. It must send the
production sweep's certified contact event and manifold to the production
dynamics solver, then assert the computed post-impact position and velocity
from Part R's Verification section. A hand-written
event fixture does not prove that
boundary. Root-package test names update `.github/test-shards.txt`, and `apitest/` test names
update `.github/test-shards-apitest.txt`.
