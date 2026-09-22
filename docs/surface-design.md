# Surface Design

The sheet body and the operations that build, fill and close one: decad's
answer to Fusion's Surface workspace. A sheet body is a `Body` that encloses
no material region — a boundary on its own — and this document owns what one
is, which operations produce one, what every reading and every `Verify`
question says about one, and which Fusion surface commands this design
deliberately does not admit.

Companion contracts stay authoritative for their own areas:

- `docs/api-design.md` owns the public model, the forward-compatibility
  invariants, and the sketch-profile seam;
- `docs/evaluator-design.md` owns topology construction, atomic commits, and
  payload dispatch;
- `docs/verification-design.md` owns what a report means and how every reading
  is gated;
- `docs/tessellation-design.md` owns the mesh contract and boolean admission;
- `docs/interference-design.md` owns the pair relation and its proof paths;
- `docs/loft-design.md` §6 owns the exact crossing audit §6.4 below reuses;
- `docs/sketch-seam-design.md` owns profile authentication and recording.

Nine tables are normative:

| Table | States | Section |
|---|---|---|
| **K** | the body kind, and what each `Kind()`/`IsSolid()` pairing means | §2.1 |
| **W** | what each surface-result feature builds, omits and orients | §4.2 |
| **J** | when two free edges join | §6.2 |
| **C** | what a stitch returns | §6.3 |
| **R** | refusals and their sentinels | §7 |
| **V** | what `Verify` says about a sheet body | §9.1 |
| **X** | which existing operations admit a sheet | §11 |
| **A** | the contract amendments this design forces | §12 |
| **D** | delivery, and what each increment lands | §14 |

## 1. Scope

### 1.1 What this design admits

- a **sheet body** — a `Body` whose boundary encloses no material region (§2);
- **`WithSurfaceResult()`** on `Extrude`, `Revolve`, `Sweep` and `Loft`: the
  feature builds its wall set and omits the faces that exist only to close the
  solid (§4);
- **`Patch`** — a planar face from a recorded profile, or a planar fill of a
  closed chain of a body's free edges (§5);
- **`Stitch` / `Unstitch`** — joining sheets along free edges that are proven
  coincident, and splitting a body back into one sheet per face (§6);
- what every reading, every `Verify` question and every export says about a
  sheet body (§8, §9, §10).

### 1.2 What this design names and stages

Each of these is a real Fusion command with a decad shape already decided, held
back because the proof it needs is not in hand. Table D says which increment
takes it up; §11 says what a caller gets until then.

| Command | Why it is not here yet |
|---|---|
| Thicken | The offset of a curved sheet self-intersects at radii under the thickness. The same open question blocks tapered extrude (`docs/evaluator-design.md` §12); neither lands without an offset formulation that **rejects** a self-intersecting offset rather than producing one. |
| Trim, Extend | Both need surface-surface intersection, which is where exactness dies (`docs/api-design.md` §2.1). Trimming a sheet with another sheet decides topology from a fitted curve, and decad does not fit. |
| Offset (surface) | Same offset gap as Thicken, without the closure question. |
| Ruled, Boundary Fill | Both need a fitted free-form patch through a boundary decad did not record. `docs/spline-design.md` owns what a recorded free-form curve may become; no rule there yet produces a surface from a boundary. |
| Reverse Normal | Not needed while orientation is decided at build (§2.3) and `Stitch` derives a consistent orientation combinatorially (§6.3). It becomes necessary only when a sheet enters an operation whose result depends on which side the caller meant. |
| A sheet operand in `Union` / `Cut` / `Intersect` (Fusion's Split Body) | Needs surface-solid intersection, the same fitted curve Trim needs. |

### 1.3 What this design refuses permanently

**A tolerant stitch.** Fusion stitches surfaces within a tolerance and produces
tolerant topology — `BRepEdge.isTolerant` is that kernel admitting in public
that its geometry does not meet (`docs/api-design.md` §2.1). decad joins two
free edges only where coincidence is **proven** (Table J), and leaves every
other edge free. A caller reads the residual free edges to see what did not
join. There is no gap parameter to widen, in this design or a later one.

## 2. The sheet body

### 2.1 `BodyKind` — what the body is, beside whether it is sound

```go
// BodyKind states what a body IS: whether its boundary encloses a material
// region. It is decided by the operation that built the body, never inferred
// from the boundary afterwards.
type BodyKind int

const (
    // BodySolid — the body encloses a material region. Its region quantities
    // are the question; Volume and Centroid answer once validity proves the
    // boundary closed.
    BodySolid BodyKind = iota
    // BodySheet — the body encloses no material region. It is a boundary on
    // its own, and Volume and Centroid are ErrNotSolid for every sheet,
    // proven sound or not.
    BodySheet
)

func (b *Body) Kind() BodyKind
```

`Kind()` is a **decided** answer, so it is a bare enum and carries no
`Exactness`: `docs/api-design.md` §6 exempts exactly the predicates the
evaluator decides — `IsSolid`, `IsConvex`, `IsOuter`, `IsVoid`, and §2.2's
`IsOpen` and `IsFree` — and this joins them. It is not a measurement of
anything.

**`Kind()` exists because `IsSolid() == false` already means two different
things, and conflating them is the confidently-wrong failure the whole project
is built against.** Table K is the whole of it:

**Table K — body kind against solidity**

| `Kind()` | `IsSolid()` | What the body is | What the caller does |
|---|---|---|---|
| `BodySolid` | `true` | a body built as a solid, whose boundary the evaluator proved closed | use its region quantities |
| `BodySolid` | `false` | a body built as a solid whose boundary did **not** prove closed | a defect: read `Verify`'s `Validity` diagnostics and change the model |
| `BodySheet` | `false` | a sheet, by construction — no defect is implied | read its boundary quantities; `Stitch` it to reach a solid (§6) |
| `BodySheet` | `true` | never produced | — |

The fourth row is structural, not a convention: a sheet encloses no region, so
no evaluator may prove one a solid.

**Closure is a property of the boundary; solidity is a claim about material,
and §6 is the one place that claim is made.** So a **closed sheet** is an
ordinary state, not a contradiction: a full revolution built as a surface
closes and stays `BodySheet` (§4.1), because nothing has yet claimed material
inside it. `Stitch` is what makes the claim — a welded all-planar boundary that
closes returns `BodySolid` (Table C) — and a closed sheet that `Stitch` has not
run on, or that it refused to promote (R8), keeps its kind.

### 2.2 Topology

A sheet body uses `docs/api-design.md` §6.1's chain unchanged —
`Body` → `Lump` → `Shell` → `Face` → `Loop` → `CoEdge` → `Edge` → `Vertex`.
Three things it holds differently, and nothing else changes:

- **A `Lump` is a connected piece of a body, not a connected *solid* piece.**
  A sheet lump holds one shell, one per CONNECTED piece of boundary — never
  the whole face set regardless of whether it is actually connected. A holed
  profile built as a surface loses the caps that would otherwise join its
  outer wall tube to its hole wall tube, so the two become genuinely separate
  pieces and the body reports one lump per piece, exactly as a disconnected
  solid does. `Body.Lumps()` returning more than one still means the body is
  disconnected, on a sheet exactly as on a solid. The alternative — hanging a
  sheet's shells off the body directly — would give `Body.Shells()` two
  traversal paths and break the 1:1 map onto Fusion's own `BRepBody.lumps`
  that `docs/api-design.md` §4 keeps deliberately.
- **A shell may be open**, and says so:

  ```go
  func (s *Shell) IsOpen() bool // true when the shell has at least one free edge
  ```

  `IsVoid()` is `false` on every sheet shell: a sheet bounds no cavity.
  `IsOpen()` and `IsVoid()` are independent questions and neither implies the
  other.
- **An edge may be free**, and says so:

  ```go
  func (e *Edge) IsFree() bool // true when exactly one face is adjacent
  ```

  `Edge.Faces()` reports the actual adjacent-face count on every body, as it
  already does. What `len(Faces())` MEANS is now stated per kind:
  `1` is a free edge on a sheet and **non-manifold** on a solid; `2` is an
  interior edge on either; `3` or more is non-manifold on either.

`Edge.IsConvex` needs no new rule. A surface-result feature's free edges are
exactly the rim edges `docs/evaluator-design.md` §3 already decides — a wall's
own copy in the plane the omitted cap would have occupied — and they keep the
sense that rule gives them. A patch's free edges are its own boundary walk,
read the same way.

### 2.3 Orientation — which side is positive

**A sheet shell carries one consistent orientation, decided at build, and
`Face.NormalAt` answers on it.** Every face of a shell agrees across every
interior edge: two faces sharing an edge traverse it in opposite senses
through their co-edges, exactly as two faces of a closed solid do.

Where the positive side comes from is per-operation, and Table W states it for
each surface-result feature. Two rules cover the rest:

- a surface-result feature's sheet takes the orientation the **solid** would
  have had — the outward normal on each wall. Nothing new is derived, and the
  sheet of a body and the body agree face for face.
- a patch takes the sense of the boundary walk it was built from (§5).

A sheet whose faces cannot be brought into agreement is **non-orientable** and
is refused wherever it would arise, which is `Stitch` alone (Table R, R7).
No feature builds one.

## 3. Public API

```go
// Body kind and the two new decided predicates.
func (b *Body) Kind() BodyKind
func (s *Shell) IsOpen() bool
func (e *Edge) IsFree() bool

// A surface result from a feature that would otherwise build a solid.
type SurfaceResultOption interface {
    ExtrudeOption
    RevolveOption
    SweepOption
    LoftOption
}

func WithSurfaceResult() SurfaceResultOption

// A planar face on its own.
func (d *Document) Patch(s *sketch.Sketch, p *sketch.Profile) (*Body, error)
func (d *Document) PatchContext(ctx context.Context, s *sketch.Sketch, p *sketch.Profile) (*Body, error)

// A planar fill of a closed chain of the receiver's free edges.
func (b *Body) Patch(sel EdgeSelector) (*Body, error)
func (b *Body) PatchContext(ctx context.Context, sel EdgeSelector) (*Body, error)

// Joining sheets, and taking a body apart.
func Stitch(bodies ...*Body) (*Body, error)
func StitchContext(ctx context.Context, bodies ...*Body) (*Body, error)
func (b *Body) Unstitch() ([]*Body, error)
func (b *Body) UnstitchContext(ctx context.Context) ([]*Body, error)

// The free-edge selector predicate.
func Free() EdgePredicate
```

Every `Context` form bounds cancellation in its own construction and audit
paths, returns `ctx.Err()` unchanged before commit, and leaves the document
and every operand unchanged. The plain form is the compatibility wrapper with
`context.Background()`, exactly as `docs/api-design.md` §8 states for every
other operation.

`Stitch` and `Unstitch` consume their operands and register their results, on
`docs/api-design.md` §6's uniform terms: `Stitch` retires every body handed
to it, `Unstitch` retires its receiver, and a retired body stays readable. No
`*Document` appears in `Stitch`'s signature because a `*Body` carries its
owning document, which is what lets `Union` take the same shape.

`Body.Patch` retires its receiver and registers the filled body.
`Document.Patch` consumes nothing — it builds from a sketch profile, as
`Extrude` does.

Worked, end to end — a closed box from three sheets:

```go
walls, err := doc.Extrude(s, prof,
    decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
    decad.WithSurfaceResult())                    // 4 walls, 8 free edges
bottom, err := doc.Patch(s, prof)                 // 1 face at z = 0
// topSketch draws the same profile on the z = 10 plane.
top, err := doc.Patch(topSketch, topProf)         // 1 face at z = 10

box, err := decad.Stitch(walls, bottom, top)      // Kind() == BodySolid
vol, err := box.Volume()                          // 60000 mm³, Exact
```

Each intermediate is a live body of the document until `Stitch` retires it, so
a caller can measure and verify the walls before closing them.

**That stitch closes because every coordinate in it is stated rather than
computed.** The walls' two rim levels come from the sketch plane and a stated
`Distance`, the patches' from their own sketch planes, and every plane-local
coordinate from the same recorded profile — so each rim edge and its matching
patch edge hold the same `float64` values with a zero bound, which is exactly
what Table J admits. A box built with a `ToFace` stop instead of a stated
distance does not close in this increment, because the stop's level is
computed and carries a bound (§6.2).

### 3.1 `Free()` — the free-edge predicate

`Free()` is an `EdgePredicate` and composes with every other clause:
`Edges(Free(), Circular())` picks the circular free edges,
`Edges(Free()).Exactly(8)` asserts the count a surface extrude of a rectangle
produces. It matches exactly the edges `Edge.IsFree()` reports.

It renders `free` in `docs/api-design.md` §9's stable query rendering, so
`Edges(Free()).Exactly(8)` reads `edges(free).exactly(8)`, and a
`SelectionError` or a verification `Diagnostic` naming that query prints it
the same way.

On a body with no free edge — every solid, and a closed sheet — the clause
matches nothing, which is an ordinary `ErrNoMatch` at resolve, or
`ErrCardinality` under an assertion. It is never an error in itself.

## 4. Surface-result features

### 4.1 What the option does

**`WithSurfaceResult()` builds the feature's wall set and omits every face
that exists only to close the solid.** It changes no wall, no measurement of a
wall, and no role: a wall face of the sheet is the same face, with the same
`side(i, j)` role and the same surface, that the solid would have carried.

That is the whole of the evaluator work, and it is deliberately small. Every
builder this option reaches already constructs its walls and its closing faces
separately — `prism_build.go`'s `buildLoopSidesAs` walk beside its two caps,
`revolve_build.go`'s swept walls beside its caps and seams — so the option
selects among faces the evaluator already has rather than asking it for new
geometry. No new surface is derived, so no new bound is derived either.

**A feature whose build mints no closing face returns a closed sheet, and that
is not a refusal.** A full revolution's wall set already closes — there is no
cap to omit — so `WithSurfaceResult()` there changes no face and yields
`Kind() == BodySheet` with no free edge. §2.1's closed-sheet rule already
separates the two questions: the boundary is closed, and no material claim has
been made about it. §6 is where that claim is made, and `Stitch` on that one
body is how a caller reaches it.

### 4.2 Table W — per feature

**Table W — what a surface-result feature builds**

| Feature | Sheet faces | Faces omitted | Positive side | Free edges |
|---|---|---|---|---|
| `Extrude` | every `side(i, j)` wall | `capStart`, `capEnd` | the solid's outward normal on each wall | both rims of every loop |
| `Revolve`, partial sweep, profile clear of the axis | every swept wall | `capStart`, `capEnd` | the solid's outward normal on each wall | both cap-plane rims |
| `Revolve`, partial sweep, profile meeting the axis | every swept wall | `capStart`, `capEnd` | the solid's outward normal on each wall | both cap-plane rims; the on-axis edge belonged to the two omitted caps and goes with them |
| `Revolve`, full revolution, profile clear of the axis | every swept wall | none — the walls already close | the solid's outward normal | none; the result is a closed sheet |
| `Revolve`, full revolution, profile meeting the axis | every swept wall | none | the solid's outward normal | none; the result is a closed sheet |
| `Sweep` | every transported wall | the start and end section caps | the solid's outward normal on each wall | both section-plane rims |
| `Loft` | every `side(i, j, k)` wall triangle | both section caps | the solid's outward normal on each triangle | both section-plane rims |

A surface-result body's payload is the feature's own payload plus the flag
that its closing faces are omitted. `Placed`, `Duplicate` and `PlacedCopy`
re-evaluate it exactly as they re-evaluate the solid's (`docs/api-design.md`
§8), and a copy carries the source's roles under its own producer identity by
the same copy-provenance rule.

`CapStart(b)` and `CapEnd(b)` on a surface-result body return well-formed
`FeatureRef` values that match nothing — the producer mints the roles, and the
faces that carried them are gone. That is the ordinary "the helper matches
nothing" outcome `docs/api-design.md` §9 already describes for a full
revolution, and it needs no new rule.

### 4.3 What the option does NOT change

**`Bounds` is the solid's box, unchanged.** Dropping the caps cannot move a
box face. Every cap is a compact region whose boundary lies in the wall set,
and a box extreme over a compact region is attained on its boundary, so no
cap interior point reaches past the walls in any axis direction. The
evaluator reuses the existing per-payload extent readings (`prism_extent.go`,
`revolve_extent.go`) verbatim, including their bounds.

**`Area` is the solid's area minus the omitted caps' areas.** Each cap area is
already computed and already carries its own bound
(`prism_build.go` stamps `capStart.areaBound`), so the sheet's area is a
subtraction of two quantities the builder holds, and its bound is their sum.
No area is re-integrated.

**Every wall's own readings are the solid's**, face for face: surface,
normal, area, edge lengths, vertex positions and every bound on them.

## 5. Patch

### 5.1 `Document.Patch` — a planar face from a profile

`Document.Patch(s, p)` records `p` through the seam exactly as `Extrude` does
— the same authentication, staleness, foreign-entity and `TExact` admission
gates of `docs/api-design.md` §7 and `docs/sketch-seam-design.md`, with no
relaxation — and builds a **single planar face** on `s.Plane().Frame()`,
carrying the profile's outer loop and every hole loop.

The result is a one-face sheet body: `Kind() == BodySheet`, one lump, one open
shell, and one free edge per coalesced boundary walk — `coalesceWalksContext`
merges adjacent collinear line segments, so a rectangle drawn as eight
collinear halves yields four edges, matching what `Extrude` already does for
its rims.

Its positive side is the sketch plane's normal, which is the sense
`Direction.Along` names for that plane (`docs/api-design.md` §7). The outer
loop walks counter-clockwise in the plane frame and every hole clockwise, so a
patch's walk is the profile's own, and `Edge.IsConvex` reads it by
`docs/evaluator-design.md` §3's rim rule: a hole's edges are concave.

**Its `Area` is the profile's own region area from `moments.go`**, net of
holes, with that engine's own exactness and bound — `Exact` for a line/arc
boundary, and the free-form tiers of `docs/spline-design.md` §5 where the
boundary carries them. Nothing is integrated twice and nothing new is proven.
`Bounds` is the recorded boundary's box, lifted through the plane frame.

### 5.2 `Body.Patch` — a planar fill of one or more free-edge chains

`b.Patch(sel)` resolves `sel` against `b`, requires the result to be a set of
**free** edges of `b` that partitions into one or more closed chains, proves
each chain planar and simple **independently**, and returns a new body
carrying `b`'s faces plus one new planar face per chain. `b` is retired.

**This is a one-time amendment to the original one-chain design: the
selection must partition into one or more closed chains, each proven planar
and simple on its own, rather than being exactly one closed chain.** A
one-chain rule makes the operation nearly unreachable on the sheets it was
written for: a surface-extruded plate's eight free edges are two congruent
rims, and no edge predicate separates them — `Free`, `Convex`, `Concave`,
`LongerThan` and `ParallelTo` match both identically, and `CreatedBy` keys on
the wall a rim bounds and so returns that wall's bottom and top edge together
— so `Edges(Free())` always resolves to both rims, and a one-chain rule
always refuses it, leaving capping a tube impossible even though that is
what the operation is for. There is no soundness cost to admitting more than
one chain: every chain is proven exactly as one would have been, on its own
edges alone, with no chain's proof reading another's.

Four gates, in this order, and each is reject-only:

1. **Every selected edge is free.** A shared or absent edge is `ErrDegenerate`
   (Table R, R4).
2. **The selection partitions into one or more closed chains.** Decided by
   vertex degree, counted over every selected edge's own use of it (an edge
   whose two ends coincide, a whole `Circle3`, uses its one vertex TWICE):
   every vertex the selection reaches must have degree exactly 2. A
   2-regular (multi)graph is a disjoint union of cycles, so this single
   condition is what proves the partition exists, not merely a necessary
   symptom of it. It corrects two things the original one-chain wording got
   wrong: a single CLOSED edge is a complete chain on its own and is the
   commonest thing anyone patches, which "shared with exactly two other
   selected edges" wrongly refused (a whole circle shares its vertex with no
   OTHER selected edge at all); and a two-edge chain of two arcs over one
   vertex pair gives each of the two vertices ONE other edge, which the
   degree-over-all-uses reading admits and the "two other edges" reading did
   not. Any vertex at a degree other than 2 is `ErrDegenerate` (R5).
3. **Each chain is planar, proven exactly.** Every chain vertex lies on one
   plane, decided over the exact rational lift of `dyadic.go` — a zero
   determinant, never a residual against a fitted plane. Three or more chain
   vertices that are not all collinear give the plane a determinant to take
   directly; under three independent vertices — a lone closed circular edge
   has exactly one, a two-edge chain of two arcs has exactly two — there is
   no such determinant, and the plane instead comes from a curved edge's own
   carrier plane (its `Center` and `Axis`), which is the only plane its whole
   curve, not just its two endpoints, can lie in. A curved edge is planar
   only when its own carrier plane is the chain's, which its recorded
   surface states. A vertex or a curve carrying a **nonzero bound** is not
   proven planar and is refused: the chain's true position is only known
   within that bound, and a plane fitted to it would be exactly the fitted
   geometry §1.3 refuses. `ErrUnsupported` (R6) — the chain may be planar, and
   a later increment with a shared-denotation certificate may admit it (§6.2).
   A chain proven **non-planar** is `ErrUnsupported` too: a non-planar patch
   needs a fitted free-form surface, which §1.2 stages.
4. **Each chain is simple in that plane.** The plane-local walk does not cross
   or touch itself, decided by `fillet_audit.go`'s existing §5 section audit —
   the same orientation, self-consuming-trim, crossing and nesting checks a
   modify op's rewritten section passes. That audit's own refusal is
   `ErrUnsupported` (an evaluator-reach reading, §5's own convention), which
   is remapped to `ErrDegenerate` at this gate's own boundary
   (`patchRemapCrossingError`, `patch_body.go`): a self-crossing chain is bad
   input this evaluator will never admit under a finer tolerance, not a
   capability this evaluator merely has not reached yet. `ErrDegenerate` (R5).

Each new face's orientation is the one that agrees with the face already
adjacent to its chain's edges: each selected edge is traversed by its one
adjacent face in some sense, and the patch traverses it in the opposite
sense. That is a combinatorial choice with one answer, and it needs no
geometry beyond deciding which of the two possible plane frames makes the
resulting loop read as its own outer (counter-clockwise) boundary
(`patchChainOrientedNormal`, `patch_body.go`) — never by trying a candidate
frame and correcting its sign from a computed area, which would silently
accept either sign and could never surface a dropped reversal. Where a
chain's own adjacent faces do not agree among themselves on that sense — an
assembly `Stitch`'s own orientation derivation refuses as non-orientable
before it is ever assembled (R7), and so a shape no builder in this package
can hand `Body.Patch` — that disagreement is `ErrDegenerate` (R18).

**Filling a HOLE loop is one of this operation's own cases, and it keeps the
holed face's normal rather than negating it.** A hole loop is walked
clockwise (moments.go's own "outer counter-clockwise, holes clockwise"
convention), so the opposite sense the patch takes reads counter-clockwise —
the same rotation an outer boundary's own fill reads — and the right-hand
rule sends that back to the SAME side the hole already faced. Filling a
face's own OUTER boundary instead (T15, §15 — doubling a flat sheet into a
two-sided plate) reverses an already-counter-clockwise loop into a clockwise
walk instead, landing on the opposite side. T19 (§15) is the hole-loop case.

`Body.Patch` admits a sheet or a solid receiver, and admits it only when this
evaluator built it (`b`'s own evaluator payload is not `nil`): a body reads
its topology regardless of payload, but this operation's own re-evaluation
contract — a placed copy re-derives each chain's orientation and re-runs its
simplicity audit rather than re-proving its planarity, on the same terms
every other rebuilding operation (`Placed`, `PlacedCopy`, `Duplicate`)
already states — needs the payload every one of those already requires. A
receiver with no payload is `ErrUnsupported` (R19). On a solid every edge is
shared, so gate 1 refuses every selection, which is the correct answer: a
solid has no hole to fill.

**The rebuild, never a mutation.** The result is built by rebuilding the
receiver from its own face set, never by attaching the new face to the
receiver's own topology in place: a retired body stays readable, and a
caller may still hold it, so mutating its edges would corrupt what they
read. This is `unstitch.go`'s own held-B-rep-under-a-rigid-motion mechanism
— record the receiver's own faces, never a built topology, and replay under
a composed motion — widened from one face to a whole face set, plus the
faces the patch mints on top of it; gate 3's own proof is never replayed,
for the same reason `Stitch`'s Table J admission never is (§6.4): a rigid
motion preserves planarity exactly, so re-proving it would only make a
placement refuse for no soundness gain. `Kind`, lumps and shells come from
the existing `sheetLumps`, run only after every face — the receiver's own
copies and every new one — is attached. A patch that closes a sheet's last
free edge leaves a **closed sheet** reporting `BodySheet` with `IsSolid()`
`false`: `Stitch` alone makes the material claim (§2.1). `Bounds` is the
rebuilt receiver's own box (`unstitchBounds`, reused verbatim): each new
face's boundary is already receiver geometry, and a bounded planar region
lies inside its own boundary's box, so the receiver's own box already
bounds the patched result too. `Area` is the receiver's own faces' area plus
each new face's, composed through `boundedAdd` — the same sum `Stitch`
already runs over its own constituent faces (§6.4/§8), reused here for one
operand's own face set plus the faces this call adds to it.

## 6. Stitch and Unstitch

### 6.1 What `Stitch` is for

**`Stitch` is the only operation that turns a boundary into a solid.** Every
other operation in decad either builds a solid outright from a swept profile
or transforms one. A caller who builds a part face by face — which is what
Fusion's Surface workspace is for — reaches a solid here and nowhere else.

`Stitch(bodies...)` welds every free-edge pair it can prove coincident across
the whole operand set, assembles the result, and returns one body. One operand
is meaningful: it re-audits that body's own boundary, which is how a closed
sheet (§2.1, §4.1) becomes a solid.

**Every operand must be a `BodySheet`.** A `BodySolid` operand is
`ErrUnsupported` (R17): it has no free edge, so stitching cannot change it, and
a caller combining two solids means `Union`. The refusal is permanent rather
than staged — there is no later proof that makes stitch the right operation
for two solids.

### 6.2 Table J — when two free edges join

**A free edge of one operand joins a free edge of another when, and only when,
both denote the same curve over the same range, proven from the records with
no tolerance anywhere.** Every row must hold. A pair failing any row does not
join, and both edges stay free — it is not an error (§6.3).

**Table J — free-edge join admission**

| # | Requirement | Decided by |
|---|---|---|
| J1 | Both edges are free (`Edge.IsFree()`). | topology |
| J2 | Both edges' `Curve` variants are the same type. | the sealed variant |
| J3 | The variants' parameters are **bit-identical** held `float64` values, up to the one sign freedom each variant allows: a `Line3`'s endpoint pair unordered; a `Circle3`/`Arc3`'s axis up to sign with its angular range reflected to match. | exact comparison of held coordinates |
| J4 | Both endpoint vertices' positions are bit-identical held `float64` coordinates, matched as an unordered pair. | exact comparison |
| J5 | Every one of those held values carries a **zero bound**. | `Vertex`'s own bound and the edge's `Curve` |

**Table J amendment — J5 is undecidable for every variant but `Line3`.**
`Circle3` and `Arc3` carry `Center`, `Axis` and `Radius` with no bound field
at all, so there is no held value on the variant itself to ask "does this
carry a zero bound" of — J5 cannot be answered either way for them, and an
undecidable row admits nothing (`CLAUDE.md`'s reject-only rule). `Line3`
alone answers it, and answers it **vacuously**: a straight edge's whole
geometry lives in its two vertices, which J4 already compares, so J3 states
nothing further about the variant itself to prove, and J5's vertex half is
exactly what `stitchVertexTable`'s own merge rule enforces (`stitch_weld.go`)
— two vertices merge into one table class only when both carry a zero bound,
so a shared class already proves J4 and J5's vertex half together. A free
edge whose `Curve` is `Circle3`, `Arc3`, `NURBSCurve` or `FacetedCurve`
therefore declines outright at J2/J5 and stays free — Table C's first row,
and never an error — until a later increment states a bound for the
variant's own parameters.

**A weld key claimed by three or more free edges joins none of them.** Table
J's admission is pairwise, but a caller can hand `Stitch` more than two
sheets whose free edges share one weld key (the same unordered pair of
vertex-table classes). Welding an arbitrary two of three would be a silent
choice among equally-claimed edges, and welding all three is non-manifold
(three faces on one edge, Table R row R7). So every edge in a group of three
or more stays free, and the result's own residual free edges name the
ambiguity exactly as they name any other unjoined pair (§6.3).

Orientation is **not** a join condition. J1–J5 decide the welding, and the
consistent orientation is derived afterwards over the welded graph (§6.3); a set
that admits no consistent choice is refused there as a whole (R7) rather than
leaving individual pairs unjoined.

**J5 is the load-bearing row, and it is what makes this sound rather than
merely strict.** Bit-identical held coordinates prove the two edges' *held*
data equal; they prove nothing about the two *true* curves when each is only
known within its own bound. Two edges each bounded by 1e-6 mm may hold the
same coordinate and lie 2e-6 mm apart, and welding them would claim a meeting
that was never proven — tolerant topology reached by a different road. A zero
bound closes the gap: the held value **is** the true value, so held equality
is true coincidence. This is `CLAUDE.md`'s reject-only rule applied to
coincidence: a small separation proves nothing, so only an exactly-zero one
admits.

J5 is what a later increment lifts, and §14's increment 3 names the mechanism:
two edges that derive from **one record** — the same profile segment swept to
the same computed level — denote the same curve exactly, whatever bound each
carries, because the bound is on the same underlying quantity. A
shared-denotation certificate carried in the payload states that, and admits a
bounded pair without ever comparing coordinates. Until it exists, a revolve's
computed-level rim and a `ToFace`-stopped prism's rim do not join, and the
free edges say so.

### 6.3 Table C — what a stitch returns

After welding every admitted pair, the evaluator assembles the surviving faces
into shells and lumps, derives one consistent orientation across every welded
edge, and decides the result.

**Table C — stitch outcome**

| Assembled boundary | Result | Reason |
|---|---|---|
| at least one free edge remains | a `BodySheet`, open | the boundary is not closed; the residual free edges name exactly what did not join |
| every edge welded, all faces planar, the crossing audit passes | a `BodySolid` | closure, manifoldness and non-self-intersection are proven, and the volume is exact (§6.4) |
| every edge welded, some face curved | `ErrUnsupported` (R8) | closure is proven but the region's volume is not yet computable, and narrowing the result to a closed sheet would silently give the caller something other than what they asked for (`docs/evaluator-design.md` §2) |
| the welded set cannot be consistently oriented | `ErrDegenerate` (R7) | a non-orientable assembly bounds nothing; no later proof makes it a solid |
| the crossing audit proves a self-contact or self-intersection | `ErrDegenerate` (R9) | the faces overlap, so the assembly is no solid's boundary |
| the crossing audit exhausts its facet-pair ceiling | `ErrUnsupported` (R10) | undecided, not disproven; `docs/loft-design.md` §6's own S8 outcome |

**A residual free edge is never an error.** A caller who expected a solid
reads `Kind()`, and a caller who wants to know what is missing resolves
`Edges(Free())` against the result. That is a better diagnosis than any error
message: the unjoined edges are handed back as selectable topology, with their
own curves and endpoints, so an agent can measure the gap it left and repair
the model. It is also why `Stitch` takes no tolerance and no
must-close option — there is nothing an option could say that the result does
not already say better.

### 6.4 The closure audit, and the volume it earns

**The reused audit is `docs/loft-design.md` §6's crossing audit alone, and it
does not by itself prove the assembled set manifold or watertight.** That
audit decides CONTACT between triangle pairs — the collapsed-triangle leg,
the fixed facet-pair ceiling, and the pairwise contact classification against
each pair's own expected shared entity — and nothing else. Three triangles
sharing one edge pass every pairwise test it runs: each pair sees the edge it
expects and nothing more, so the audit alone would admit a self-touching,
non-manifold assembly. For a loft, closure is a property of the construction
itself (`docs/loft-design.md` §5's own paired-station walk can never leave a
gap or a triple junction), so the loft audit's own doc comment never needed
to claim more than contact. `Stitch` builds its triangle set by welding
independently-authored faces, where closure is exactly the open question, so
it runs its own explicit **directed-edge parity leg** (`stitch.go`'s
`checkStitchClosure`) beside the reused audit: every edge must be adjacent to
one or two faces, and a two-face edge must be traversed by exactly one
forward and one backward coedge, with a coedge-use count that disagrees with
the edge's own adjacent-face count also a violation (`ErrDegenerate`, Table
R row R7). One adjacent face is an open boundary — Table C's first row, and
never an error. Together the two legs are what proves the assembled set
manifold, watertight (when closed) and free of self-intersection:

- every face is planar and every welded vertex carries a zero bound (J5), so
  triangulating each face by `triangulate.go`'s existing cap triangulator
  introduces no coordinate that is not already held;
- welding on proven-coincident vertices (J4, J5) gives two faces sharing a
  vertex the same **index** in the shared table — the exact, free fact the
  reused audit's contact classification is built on;
- the directed-edge parity leg is what proves adjacency itself is correct —
  never more than two faces per edge, and a consistent forward/backward use
  at every two-face edge — which the reused audit's own contact test does
  not ask.

**`Area` is the sum of the constituent faces' own `area`/`areaBound`, through
`boundedAdd` — never a triangle-sum reading.** A general planar triangle's
own area is a square root of a rational and is therefore never `Exact` the
way `spline_length.go`'s outward brackets state for `docs/loft-design.md`
§8's wall reading; each stitched face's own area, by contrast, already comes
from the closed-form region integral (`moments.go`) or the surface-result
subtraction (§4.3) that built it, and is `Exact` wherever that integral is.
Summing those already-proven readings is what §8 already prescribes for a
sheet, and `Stitch` reuses it unchanged for a solid: the box worked example
sums to 15200 mm² at a zero bound, the four walls' 3200 mm² (T1) plus the two
patches' 6000 mm² apiece (T2).

**The volume is `loft_moments.go`'s exact-rational tetrahedron sum over the
assembled triangle set**, and its bound is zero only when that exact
rational is itself representable in `units.Value`'s `float64` magnitude —
never unconditionally. Every held vertex is exact, so every tetrahedron term
and their sum are exact rationals, and the one rounding `Volume` ever
carries is that single publication step: `Exact` exactly when the published
number IS the sum (the box's 60000 mm³, a small integer, always is),
`Approximate` with a proven bound otherwise (a body a third of a millimetre
wide, whose exact volume is rarely representable in cubic millimetres to the
last bit). `Centroid` and `Bounds` follow the identical rule over their own
publication rounding.

That is why Table C admits closure only for an all-planar face set in this
design. A curved face's flux term is not a tetrahedron sum, and a per-surface
closed-form flux integral over an arbitrary trimmed analytic patch is its own
piece of work. §14's increment 3 takes it up.

**This audit governs the all-planar STITCH closure alone.** It is not §9.1's
sheet validity audit: a stitched solid's triangulated, exactly welded face set
is exactly this audit's input, but a surface-extruded sheet's curved walls
are not, and §9.1 reads their recorded topology directly rather than
chording them into one. §9.1 does gain one new admission, though: a
`stitchPayload` whose own crossing audit ran and passed — closed or open
alike — admits leg 4 the same way a proven-simple surface-result prism does,
which is the open-case decision two paragraphs below.

**The crossing audit runs on the OPEN case too, not on the closed case
alone.** A clean stitched sheet — every welded edge proven contact-clean —
can then read `ValidityValid` under §9.1's audit instead of being
permanently `Suspect`, the same way a surface-result prism's own leg 4 is
proven rather than assumed. The two outcomes stay asymmetric on purpose:
closed plus a refusing audit is `ErrDegenerate` (R9) — a caller asked for a
solid and the geometry cannot be one — while open plus a refusing audit
stays a sheet with leg 4 undecided and **no error**, preserving §6.3's rule
that a residual free edge is never an error. `Stitch` never re-wraps the
reused audit's own sentinel for the closed case: R9 and R10 are its
`ErrDegenerate`/`ErrUnsupported` surfaced unchanged.

**Placing a stitched body replays the recorded weld; it never re-derives
it.** `stitchPayload` records the operand faces and Table J's own admission
(`stitch_weld.go`'s `stitchWeldPlan`) — never a built topology — and
`Placed`/`Duplicate`/`PlacedCopy` re-evaluate from those same two values
under the composed transform. A rigid motion rounds every coordinate, so
re-running Table J against the PLACED, rounded values would admit nothing —
a placed box would come back a sheet with 24 free edges, since no two placed
corners are bit-identical any more. So the motion is applied ONCE, to the
single held shared vertex table, never per operand, and every downstream
step — the fresh topology rebuild, the derived orientation, the closure
leg — replays over that one placed table exactly as it does unplaced. The
rounding itself is charged as `bounds.go`'s `rigidRoundAllow`, folded into a
`delta` exactly as a placed loft's is (`docs/loft-design.md` §5/§12): zero
only when the transform is the identity, an exact struct comparison. Two
things ARE re-decided fresh on every placement, never replayed, because a
rigid motion can change either one: the crossing audit runs again on the
placed table, since rounding can bring two placed triangles into contact
that the unplaced ones were proven clear of; and the whole-assembly
orientation sign is re-decided from the placed triangle set's own signed
volume, since an improper motion (a reflection) flips it. A placed stitched
solid therefore publishes `Approximate` — `Volume`, `Area`, `Bounds` and
`Centroid` all carry `delta`, on the same terms `docs/loft-design.md` §8
states for a placed loft.

### 6.5 `Unstitch`

`b.Unstitch()` returns one single-face sheet body per face of `b`, in
`b.Faces()` order, retiring `b`. Each result carries that face's own surface,
loops and readings; every edge of every result is free.

It admits an analytic body, solid or sheet — the asymmetry with `Stitch`, which
takes sheets only (R17), is deliberate: `Unstitch` is how a caller gets sheets
to work with in the first place. A `Faceted` body — a mesh
boolean's result — is `ErrUnsupported` (R11): its faces are chord polygons
with no analytic identity, and returning thousands of facet sheets would be a
shape no caller asked for.

**`Unstitch` inverts `Stitch` exactly where `Stitch` reaches.** Every edge
`Unstitch` frees was welded from a proven-coincident pair, so Table J re-admits
every one of them and re-stitching the results reproduces the body. The round
trip therefore closes for an exact all-planar body and no further: unstitching a
revolve gives sheets whose rims carry the revolve's own bounds, which J5 does
not admit, and re-stitching them returns a sheet. That is the same reach limit
J5 and R8 state, not a separate one, and the closing case is a required test
(§15).

**`unstitchPayload` (`unstitch.go`) is the held-B-rep-under-a-rigid-motion
mechanism restricted to one face**, `stitchPayload`'s own idea narrowed from a
whole welded set to a single held `*Face`: it records that face and the
retiring receiver's own already-proven `Bounds`, never a built topology, and
`placed()` deep-copies fresh surface, loop, edge and vertex geometry under the
composed transform every time, exactly as `stitchPayload.placed` replays
`evalStitchContext`. Nothing in either result aliases the receiver's own
`*Face`, `*Edge` or `*Vertex`, so a retired receiver stays readable and
unmodified for any caller still holding it.

**Each result's own `Bounds` is a tight box exactly when its face is bounded
entirely by straight (`Line3`) edges on a `Plane` surface, and the sound
whole-receiver box otherwise.** A straight-edged planar face's enclosed
region is the polygon its own held vertices describe: the extreme along any
world axis is always attained at a vertex, so `faceBounds`
(`unstitch.go`) reads that box directly off the already-placed copy's own
vertex coordinates — no integration, no new proof, the same vertex-derived
reading every other planar measurement already publishes. A box's T3 worked
example is exactly this case: each of the six unstitched sheets reports its
own flat slab (the bottom `(0,0,0)`–`(100,60,0)`, the top
`(0,0,10)`–`(100,60,10)`, each wall its own thin rectangle) rather than the
whole box, and the six boxes' union still spans the original.

Every other face falls back to the receiver's own whole box
(`unstitchBounds`, `unstitch.go`) — sound, because a face's extent is always
a subset of the body it came from, but not proven tight. This covers a
CURVED surface (a cylinder wall bulges past the two seam vertices its own
loop holds) and, less obviously, a PLANAR surface with any non-`Line3` edge:
a disk's circular cap is flat, but its rim bulges past the single vertex a
full circle's own loop holds, exactly as a curved surface's rim does. `Box`
has no field that distinguishes "sound, not proven tight" from "proven
exactly representable" — `Exactness`/`Bound` state only whether further
NUMERICAL rounding remains, not whether the published extent is the
tightest one this evaluator could in principle prove. So an unplaced curved
face's box still reads `Exact` with a zero bound whenever the unplaced
receiver's own box did: that claims no further widening is needed to stay
sound, never that the box is this one face's tightest possible reading. A
future increment that wants to say "sound but not tight" for a `Box` needs a
new field to say it with. This is the one place in this design where `Exact`
does not imply tight, and it is a limit of `Box`'s own shape, not of this
evaluator: recorded here rather than left for a reader to discover by
surprise.

## 7. Table R — refusals and their sentinels

Every refusal is at the call, before any commit; the document and every
operand are unchanged. No new sentinel is introduced: each row lands on one
of `docs/api-design.md` §12's existing vocabulary, and the distinction the
caller acts on is the one that vocabulary already draws — `ErrDegenerate` is
input with no usable geometry, `ErrUnsupported` is this evaluator's reach.

**Table R — refusals**

| # | Condition | Sentinel |
|---|---|---|
| R1 | `WithSurfaceResult()` on a feature this evaluator cannot yet build as a surface (Table D) | `ErrUnsupported` |
| R2 | `Document.Patch` handed a profile that fails `docs/api-design.md` §7's seam gates | as the seam states: `ErrForeignProfile` / `ErrStaleProfile` / `ErrInvalidProfile` / `ErrUnrecordableProfile` |
| R3 | `Document.Patch` handed a profile whose recorded boundary carries a free-form segment this evaluator cannot integrate | `ErrUnsupported` |
| R4 | `Body.Patch` selection holds an edge that is not free | `ErrDegenerate` |
| R5 | `Body.Patch` selection does not partition into closed chains, or a chain's plane-local walk crosses or touches itself | `ErrDegenerate` |
| R6 | `Body.Patch` chain is proven non-planar, or carries a nonzero bound so planarity is not proven | `ErrUnsupported` |
| R7 | `Stitch`'s welded set cannot be consistently oriented | `ErrDegenerate` |
| R8 | `Stitch` closes a boundary holding a curved face | `ErrUnsupported` |
| R9 | `Stitch`'s crossing audit proves a self-contact or self-intersection | `ErrDegenerate` |
| R10 | `Stitch`'s crossing audit exhausts its facet-pair ceiling | `ErrUnsupported` |
| R11 | `Unstitch` on a `Faceted` body | `ErrUnsupported` |
| R12 | `Stitch` handed no body, or `Unstitch` on a body with no evaluator payload | `ErrDegenerate` |
| R13 | `Stitch` handed bodies owned by different documents | `ErrForeignBody` |
| R14 | `Stitch` or `Unstitch` handed a retired body | `ErrRetiredBody` |
| R15 | a sheet handed to an operation Table X refuses | `ErrUnsupported` |
| R16 | `Body.Patch`'s selector resolves to nothing, or fails its own cardinality assertion | `SelectionError` wrapping `ErrNoMatch` / `ErrCardinality`, unchanged |
| R17 | `Stitch` handed a `BodySolid` operand | `ErrUnsupported` |
| R18 | `Body.Patch` chain's adjacent faces cannot be brought into agreement on the new face's orientation | `ErrDegenerate` |
| R19 | `Body.Patch` handed a receiver with no evaluator payload | `ErrUnsupported` |

R6, R8 and R10 are `ErrUnsupported` rather than `ErrDegenerate` on
`docs/api-design.md` §8's own distinction: the input names real geometry and
the refusal is this evaluator's reach, not a zero or self-crossing region. R5,
R7 and R9 are `ErrDegenerate` because the geometry itself is the problem and
no later evaluator admits it. R11, R15 and R17 are `ErrUnsupported` for
permanent boundaries rather than staged ones, which is the same use
`docs/modify-reach-design.md` already makes of the sentinel: the caller's move
is a different operation, not a later version. R18 joins R5, R7 and R9 for the
same reason: an inconsistent assembly is the problem, not this evaluator's
reach. R19 joins R11, R15 and R17: a body this evaluator did not build is a
different operation's receiver, not a later version of this one.

## 8. Measurements

| Reading | Sheet body |
|---|---|
| `Area` | the sum of its faces' areas, each from the engine that already computes it, with the composed bound (§4.3, §5.1) |
| `Bounds` | the face set's box, from the existing per-payload extent readings |
| `Volume` | `ErrNotSolid`, for every sheet — closed or open, proven sound or not |
| `Centroid` | `ErrNotSolid`, on the same terms |

**`Volume` and `Centroid` refuse a sheet by kind, not by soundness.** On a
solid, `ErrNotSolid` reports a defect (`IsSolid()` false on a `BodySolid`
body); on a sheet it reports a category. `Kind()` is what distinguishes them,
which is §2.1's whole argument, and `docs/api-design.md` §6's rule that
`volume == 0` must never mean both "empty" and "not a solid" is what makes
the error the right shape in both cases.

**A sheet has no area centroid in this design.** The centroid of a region and
the centroid of a surface are different quantities, and adding a second
`Centroid` meaning on the same method would be exactly the overloaded
answer §4 of the core rejects. A later design may add a separate reading; this
one does not.

Every reading a sheet carries is judged by the tolerance gate exactly as a
solid's is (`docs/verification-design.md` §2, §5). That gate's reference
diameter is proven per payload by `verify_gate.go`, and every sheet payload
supplies one from geometry it already holds: a surface-result body's is its
solid's, from the same extent readings §4.3 reuses; a patch's is its recorded
boundary's own two farthest points; a stitched sheet's is its face set's. No
sheet payload withholds the gate.

## 9. Verification

### 9.1 Table V — what `Verify` says about a sheet

**Table V — a sheet body's report**

| `BodyReport` field | Sheet body |
|---|---|
| `Validity.Outcome` | `ValidityValid` when the sheet audit's three structural legs hold and non-self-intersection is admitted by construction; `ValidityInvalid` on a proven structural violation; `ValidityUndecided` otherwise |
| `Topology.Lumps` | the connected-piece count, as on a solid |
| `Topology.Voids` | `0` — a sheet bounds no cavity |
| `Area`, `Bounds` | present and gated, as on every body |
| `Region` | **nil, always** |
| `Wall` | `ScalarNotRequested` when omitted; `ScalarUnavailable` with a `DiagSurveyPrerequisite` when requested |
| `Undercut` | `CoverageNotRequested` when omitted; `CoverageUnavailable` with a `DiagSurveyPrerequisite` when requested |
| `ConcaveRadius` | `ScalarNotRequested` when omitted; `ScalarUnavailable` with a `DiagSurveyPrerequisite` when requested |
| pair rows | §9.3 |

**The sheet validity audit reads the recorded topology directly; it is not
§6.4's closure audit with closure dropped.** `loftCrossingAudit` proves its
three properties over a triangulated, all-planar face set sharing one exact
vertex table (§6.4) — an input a surface-extruded wall cannot supply. Its
`Plane`, `Cylinder` or `NURBSSurface` geometry, rimmed by `Arc3`, `Circle3` or
`NURBSCurve` segments, would first have to be chorded into triangles, and
admitting a sheet because its CHORD MESH audits clean is an admission gate
resting on an approximation — the very thing `CLAUDE.md`'s reject-only rule
forbids. No chording, and no reuse of `loftCrossingAudit`, reaches a curved
wall soundly.

The sheet audit instead runs three structural legs over the held
`Body`→`Lump`→`Shell`→`Face`→`Loop`→`CoEdge`→`Edge` chain — every face has at
least one loop of at least one coedge; every edge is adjacent to one or two
faces (three or more is non-manifold); and every two-face edge is traversed by
exactly one forward and one backward coedge, with a coedge-use count that
disagrees with the edge's own adjacent-face count also a violation — plus a
fourth leg, non-self-intersection, admitted by construction alone and never by
a residual or a chord test. A surface-extruded sheet earns the fourth leg
because `sketch` already proved the recorded section a simple closed planar
region, `evalPrismContext` already refuses a non-positive sweep height, a
simple planar region crossed with a positive interval cannot self-intersect,
and the rigid placement that follows preserves that. Any other construction —
or a section admitted only to within a nonzero displacement of what it
denotes, which breaks the transfer of simplicity — earns no such proof: the
fourth leg has nothing to stand on, which is undecided, not a violation. A
free edge, which on a closed body would be the watertightness failure, is the
expected shape here and is counted by the structural legs rather than
faulted.

**`Region` is nil for a sheet even when `Validity.Outcome` is
`ValidityValid`.** `docs/verification-design.md` §1 currently states the
biconditional over validity alone; §12 amends it to validity **and** kind. A
sound sheet is a sound *boundary*, which is the whole of what the caller
asked to be true about it.

**The three surveys need a solid**, and say so. Each asks a question about
material — how thin a wall is, whether a face opposes the pull, how tight a
concave feature is — and a sheet has none. `DiagSurveyPrerequisite` is the
existing code for "a requested survey needs a proven solid"; §12 widens its
stated cause to include a sheet, so a caller branching on it already handles
this. Draft analysis on an oriented sheet is a real question — Fusion answers
it — and §14's increment 3 takes up the undercut survey over a sheet's
positive side.

### 9.2 What a sound sheet costs

**A document holding only sheets verifies fully and reads `Sound`, provided no
survey was requested.** Nothing about a sheet makes a report `Suspect` on its
own: its boundary quantities are gated like any other, its validity is decided
in `Verify` — exactly where every other body kind's is, so a sheet's audit is
not a special build-time step — and an omitted survey reads its
`NotRequested` outcome as everywhere.

Two things do cost a sheet a `Suspect`, and both are the caller asking a
question a sheet cannot answer: a **requested** survey, which reads
`Unavailable` with a `DiagSurveyPrerequisite` (§9.1); and §9.3's pair rule,
where a sheet's box meets a solid's. Neither fires on a model that only holds
sheets and only asks the core questions.

### 9.3 Pairs

`docs/interference-design.md` §2 keeps **only proven solids** for its own four
relations (§1 there), so a pair with a sheet operand needs a different
question answered. Silence is the wrong answer: a caller whose sheet passes
straight through a solid would get no row and a `Sound` report, which is the
confidently-wrong outcome the project exists to prevent.

**The rule turns on box separation, which §3.1 of that design already runs:**

- **bounds-inflated boxes separated** → nothing is emitted under the default
  call. A sheet-sheet pair stays silent regardless of `WithClearances()`,
  since neither operand offers a closed boundary to measure a gap against. A
  sheet-against-solid pair honors `WithClearances()` exactly as a solid-solid
  pair does (`docs/interference-design.md` §3.1: box separation does not
  measure the true gap, and the analytic kernel still runs when a gap is
  asked for): the decision procedure below still runs, with the box
  separation already excluding crossing and containment, and reports the
  measured gap like any other pair.
- **boxes meet** → the decision procedure below always runs, whether or not a
  gap was asked for — a crossing must be checked either way.

That keeps the noise where the question is live. A model with a sheet parked
away from every solid verifies clean; a sheet that might be cutting through
one is decided.

**A sheet whose own validity is undecided or invalid does not take this box
rule.** It inherits the existing filter that only a proven-valid body enters
pair work at all (`docs/interference-design.md` §2), so it never reaches
either bullet above. No report reads falsely `Sound` for it: such a sheet
already contributes its own `Suspect` or `Unsound` through `Validity`, with no
help needed from the pair rule.

**A sheet-sheet pair takes none of the decision procedure below.** Neither
operand offers a closed boundary to cast the other against — the procedure's
last step needs a solid's boundary to cast a witness through — so a
sheet-sheet pair whose boxes meet goes straight to `DiagUnsupportedPairSheet`,
`Suspect`, exactly as increment 1 left it.

**The decision procedure, for a sheet operand against a proven solid whose
boxes meet OR whose separated boxes were asked for a gap
(`sheetSolidPair`, `clearance.go`):**

1. Build the clearance kernel's own carrier model (`docs/clearance-design.md`
   §2) for both operands. Either one missing — a revolve sheet is refused a
   model outright, or an operand is multi-lump or faceted — leaves the pair
   **undecided**.
2. Run the existing candidate enumeration (§3 of that design) over the two
   models UNCHANGED, with one omission: the §6 coplanar contact certificate is
   never run. That certificate proves each body's material lies wholly on its
   own side of a shared plane, which is a claim about material a sheet does
   not have, so it could never fire honestly here.
3. An admitted transversal crossing between a sheet face and a solid face
   proves the sheet **crosses** the solid's boundary. Box separation already
   excludes this outcome — a box that does not even meet the solid's own box
   cannot admit a crossing — so this step never fires when the boxes were
   separated.
4. Absent a crossing, an unsure candidate, an infinite proven upper bound, or
   a proven lower bound at or below the tolerance leaves the pair
   **undecided** — the same standard the solid-solid path already holds a
   small residual to.
5. A proven POSITIVE lower bound means the sheet's boundary misses the
   solid's boundary entirely. **When the boxes were already proven
   separated, that alone decides outside** — a box that does not meet the
   solid's own box cannot admit containment either, so the witness cast below
   would only confirm a settled answer, and is skipped, exactly as
   `clearancePair`'s own `nestingExcluded` parameter skips its
   two-directional cast for a box-proven solid pair. Otherwise, one
   deterministic sheet vertex — the same witness `docs/clearance-design.md`
   §2's per-shell casts already use — is cast through the solid's existing
   three-outcome containment cast (`bodyGeom.pointInBody`,
   `clearance_geom.go`): inside proves the sheet **contained**, outside
   proves it **outside** with the measured gap, and a failed cast leaves the
   pair undecided.

**Why one witness decides the whole sheet.** Once the boundary distance is
proven positive, the sheet's boundary misses the solid's boundary entirely.
Being one connected shell, the sheet then lies wholly within one connected
component of space minus that boundary, so one point's membership answers for
the whole shell. The premise is asserted, not assumed: §2.2 gives a sheet lump
exactly one shell, and the clearance kernel already refuses any body with more
than one lump a model at all, so a sheet that reaches this step always has
exactly one witness to cast. The two-directional cast the solid-solid path
runs (`docs/clearance-design.md` §2) exists for a different reason — an inner
body's several shells can be cut apart by the outer body's void shells, so the
outer's own witnesses must be checked too — and a one-shell sheet has no void
shells of its own for anything to cut apart, so the reverse cast is not merely
skipped as an optimization: it has nothing left to prove.

**The report each outcome produces:**

- **Crossing** → the pair reads `Interfering` and carries exactly one new
  diagnostic, `DiagSheetSolidCrossing` — stable token
  `"sheet_solid_crossing"`, `Reading` `ReadingNone`, every `Observed*`,
  `Required` and `Body` nil, `Pair` set. No `Interference` row is emitted: a
  sheet encloses no region, so there is no overlap volume to report, and that
  absent row is **not** a proven non-overlap here — it is the ordinary shape
  of a sheet pair report, crossing or not.
- **Contained**, or **outside** → the pair reads `Sound`, no diagnostic.
  Under `WithClearances()` it carries a `Clearance` row with the SAME proven
  gap interval either way: `docs/api-design.md` §6.2 states the consequence
  that follows.
- **Undecided** (missing model, unsure candidate, or a failed witness cast)
  while the boxes MEET → the pair keeps `DiagUnsupportedPairSheet`,
  `Suspect`. Its message now names the kernel's own failure to settle the
  pair, not an unsupported sheet operand — a sheet body is fully supported;
  some of its pairs are not yet decidable. The same undecided answer while
  the boxes are SEPARATED is `DiagUndecidedClearance` instead, `Suspect`: box
  separation already proves the sheet outside the solid, so only the
  requested gap itself is unmeasured — the same code and the same reasoning
  a box-proven solid-solid pair uses when its own kernel cannot measure the
  gap (`docs/verification-design.md` §1.1).

**`DiagUnsupportedPairSheet` is the existing code in the
`DiagUnsupportedPair*` family**, stable token `"unsupported_pair_sheet"`,
`Reading` `ReadingNone`, every `Observed*` and `Required` nil, `Pair` set,
contributing `Suspect`. `DiagSheetSolidCrossing` is the new sibling code the
decision procedure adds. §12 records both.

## 10. Tessellation and export

**A sheet body tessellates, with the closed-mesh audit replaced by the
manifold-with-boundary audit.** `docs/tessellation-design.md` §1.2 owns that
audit's three requirements, the fact that every other row of its §1 contract
binds a sheet mesh unchanged, why a sheet mesh reaches no boolean, and what
`STL` and `OBJ` write. §12 records the amendment that added it.

A surface-result **prism** (`Extrude`) sheet tessellates and exports: its
walls chord exactly as the solid the same record would build, both caps are
omitted from the mesh exactly as they are from the body, and the
manifold-with-boundary audit runs in the closed-mesh audit's place. A
surface-result **revolve** sheet does not yet: `Tessellate`/`STL`/`OBJ` refuse
it with `ErrUnsupported` at the dispatch that would otherwise look up a role a
surface result's own build never attached (§4.2) — this evaluator's own
reach, never the `ErrDegenerate` a missing face role would otherwise report,
since the body's geometry is not the problem. Increment 4 takes up the revolve
path, and it is where a CLOSED sheet's mesh is settled: a full revolution's
surface result has no free edge at all, and no earlier increment produces one
(§14).

Two consequences this design leans on, stated here as claims and derived there.
A sheet mesh is never a boolean operand, which is what Table X's boolean row
rests on. An open body's STL is not a solid file, and `Body.STL`'s doc comment
says so, which is how a caller learns it before handing the file to a slicer
rather than after.

## 11. Table X — which existing operations admit a sheet

**Table X — a sheet as receiver or operand**

| Operation | With a sheet | Why |
|---|---|---|
| `Union` / `Cut` / `Intersect` | `ErrUnsupported` | needs surface-solid intersection (§1.2), and the mesh path needs an occupied-volume proof a sheet has none of (§10) |
| `Fillet` / `Chamfer` | `ErrUnsupported` | `docs/modify-design.md`'s reduction rewrites a prism's **section**; a sheet's free boundary is not a section, and blending to a free edge is its own design |
| `Shell` | `ErrUnsupported` | offsets a section into a wall of thickness `t`; a sheet has no section, and the offset is Thicken's own open question (§1.2) |
| `Placed` / `PlacedCopy` / `Duplicate` | admitted, unchanged | a rigid motion of a payload; nothing in it reads solidity |
| `Tessellate` / `STL` / `OBJ` | a prism sheet tessellates and exports; a revolve sheet is `ErrUnsupported`, staged (§10) | the manifold-with-boundary audit §10 describes is built and runs on the prism path; the revolve path awaits increment 4 |
| `ToFace` / `ToFaceAngular` naming a **planar** face of a live sheet | admitted | the stop reads the face's plane and nothing about material, so `stops.go`'s resolution is unchanged |
| `ToFace` naming a curved face of a sheet | as for a solid | this design changes no curved-stop reach |
| `EdgeAxis` naming a linear edge of a live sheet | admitted | the axis reads the edge's line; `docs/api-design.md` §6.2's exactly-one and liveness rules apply unchanged |
| `ThroughAll` / `ThroughAllSide` resolving its stops over a document holding a sheet | the sheet is skipped, not refused | a sheet encloses no material to stop against, and refusing would stop any document holding one sheet from using `ThroughAll` at all |
| `Body.Faces` / `Edges` / `Vertices` / `Lumps` / `Shells` | admitted, unchanged | traversal and inspection, as on every body |
| every selector predicate | admitted, unchanged | a sheet's faces and edges answer `Planar()`, `Convex()`, `CreatedBy()` and the rest on the same terms |

A sheet is a first-class body everywhere it is admitted: it is registered in
the document, it appears in `Document.Bodies()`, `Verify` reports on it, and
it retires and copies on `docs/api-design.md` §6's uniform terms.

## 12. Table A — the contract amendments this design forces

Each row changes a decision an existing design document states, and changing a
decision means changing the doc (`CLAUDE.md`). Each is an extension, and none
reverses a decision already taken.

**Table A — amendments**

| Document | What changes |
|---|---|
| `docs/api-design.md` §6 | `Body` gains `Kind()`; `Volume`/`Centroid`'s `ErrNotSolid` gains the by-kind reading (§8); `Lump`'s gloss becomes "a connected piece of a body"; `Shell` gains `IsOpen`; `Edge` gains `IsFree` |
| `docs/api-design.md` §6.1 | `Edge.Faces()`'s `len != 2` gloss gains the per-kind reading (§2.2) |
| `docs/api-design.md` §8 | the v1 feature vocabulary gains `Patch`, `Stitch`, `Unstitch` and `WithSurfaceResult`; the signatures and the `Context` forms land beside the existing ones |
| `docs/api-design.md` §9 | the predicate list gains `Free()`, and the rendering table gains its `free` token |
| `docs/api-design.md` §13 | the v1 non-goal list keeps sheet-metal, and gains the §1.2 staged commands so the list stays the whole of what is out |
| `docs/evaluator-design.md` §3 | the `Lump` row states "connected piece"; the topology-model rules gain the free-edge reading |
| `docs/evaluator-design.md` §11 | the increment table gains Table D's rows |
| `docs/verification-design.md` §1 | `Region`'s biconditional becomes validity **and** `Kind() == BodySolid` (§9.1) |
| `docs/verification-design.md` §1.1 | `DiagUnsupportedPairSheet` is added, with its stable token, and its stated cause widens to the decision procedure's own undecided leg (§9.3); `DiagSheetSolidCrossing` is added, with its stable token; `DiagSurveyPrerequisite`'s stated cause widens to include a sheet; the `ScalarUnavailable` prose states that a sheet is a solid prerequisite failure by category |
| `docs/verification-design.md` §5 | the flat-body paragraph is scoped to a body built as a **solid**, so its `Unsound` verdict does not read as covering a sheet |
| `docs/verification-design.md` §6 | `aggregateStatus` gains the volume-less `Interfering` rung: a diagnostic whose own `Status` is `Interfering` (`DiagSheetSolidCrossing`) makes the report `Interfering` with no `Interference` row at all (§9.3) |
| `docs/interference-design.md` §1 | the four-relation table is scoped to a pair of proven solids; a sheet operand takes §9.3's decision procedure instead |
| `docs/interference-design.md` §2 | the report walk states §9.3's decision procedure for a pair holding a sheet, in place of the box rule alone |
| `docs/clearance-design.md` §2 | states that `sheetSolidPair` reuses the carrier model and candidate enumeration, skipping the §6 coplanar certificate, which a sheet's missing material could never satisfy |
| `docs/clearance-design.md` §3 | states that a sheet-against-solid pair settles a proven positive lower bound with ONE witness cast, never the two-directional nesting relation a solid pair needs, and why (§9.3) |
| `docs/api-design.md` §6.2 | `Clearance`'s doc comment states the one consequence: for a sheet-solid pair the row states the distance to the solid's boundary without asserting which side the sheet is on; a solid-solid row's meaning is unchanged |
| `docs/tessellation-design.md` §1 | the Geometry row and the mandatory closed-mesh audit become kind-conditional (§10) |
| `docs/layout.md` | a row for this document, and one per `.go` file each increment adds |
| `CLAUDE.md` | a "Read before you write" row pointing here for sheet-body, surface-feature, patch and stitch code |

## 13. The upstream ask — an open sketch chain

**Fusion's most common surface extrude starts from an open sketch curve, and
decad cannot express one.** `sketch.Profile` is a closed planar region by
definition, and `sketch` exposes no open boundary chain, so
`WithSurfaceResult()` builds only from a closed profile: a rectangle gives a
four-walled tube, never a single ribbon from one line.

What decad needs from `sketch`, stated so the seam's existing gates apply
unchanged:

- an **ordered open chain** of `BoundaryEdge` values over the sketch's own
  entities, with the same `TStart`/`TEnd`/`TExact` trim contract a `Profile`'s
  boundary carries, so `docs/sketch-seam-design.md`'s admission gate and
  reject-only falsifiers govern it with no new rule;
- the same **authentication and staleness** handles a `Profile` carries — a
  source-sketch back-reference and a revision — so
  `docs/api-design.md` §7's foreign, stale and snapshot-match gates apply
  verbatim;
- the chain **self-intersection** answer, from `sketch`'s own arrangement, so
  decad consumes it rather than re-deriving it.

This is the same shape as `docs/spline-design.md` §9's upstream ask, and it is
tracked the same way: named here, with the capability it unlocks stated, and
no decad-side workaround. decad will not assemble a chain from selected
entities itself — deciding which entities form a chain and in what order is a
2D arrangement question, and `CLAUDE.md`'s rule is that decad asks `sketch`
for the answer and never re-derives it.

## 14. Table D — delivery

Each increment is a PR series behind this contract. Staging surfaces exactly
as `docs/evaluator-design.md` §11 states it: an intent the evaluator cannot
BUILD is `ErrUnsupported` at the call, and a `Verify` question it cannot
ANSWER is accepted and reads `Suspect`.

**Table D — increments**

| # | Lands |
|---|---|
| 1 | `BodyKind` and `Kind()`, `Shell.IsOpen`, `Edge.IsFree`, `Free()`; `WithSurfaceResult()` on `Extrude` and `Revolve`; `Document.Patch`; the sheet validity audit; `DiagUnsupportedPairSheet` and §9.3's box rule; every Table A amendment; prism sheet tessellation and export with the manifold-with-boundary audit. The revolve sheet mesh is staged to increment 4 (§10) |
| 2 | `Stitch` over exact all-planar boundaries (Table J with J5, Table C's first two rows), including its own directed-edge parity leg, derived orientation, and the recorded-weld replay a placement reuses; `Body.Patch`; the sheet-against-solid containment cast and clearance gap of §9.3, narrowing when `DiagUnsupportedPairSheet` fires. `Unstitch` is a separate follow-up: it needs no new proof this increment does not already carry, but it is its own PR |
| 3 | `WithSurfaceResult()` on `Sweep` and `Loft`; the shared-denotation certificate, which lifts J5 for bounded edges and §5.2 gate 3's bounded-chain half of R6 together; the per-surface flux integral that lifts Table C's curved-closure refusal (R8); the undercut survey over a sheet's positive side |
| 4 | The revolve sheet mesh (§10): the meridian and angular chordings a surface result keeps, the caps and poles it omits, the cap terms its area slack drops, and the manifold-with-boundary audit in the closed-mesh audit's place. It also settles which audit a CLOSED sheet runs |
| 5 | A stitched solid's own mesh: the manifold-with-boundary/closed-mesh audit reads a `stitchPayload`'s already-triangulated face set directly rather than chording one. A stitched solid's own clearance-kernel carrier model: `newBodyGeomBudget` (`docs/clearance-design.md` §2) gains a `stitchPayload` arm, which is what lets a stitched solid reach a proven pair relation at all — until it lands, `Tessellate`/`STL`/`OBJ` and every pair question read `ErrUnsupported`/undecided exactly as they do for any other payload this evaluator has not wired an arm for |

**Increment 4 depends on neither 2 nor 3, and they do not depend on it.**
Increment 5 depends only on increment 2, which is what gives it a triangle
set and a topology to read; it is numbered after 3 and 4 only so that no
reference to them has to move. Any order among 3, 4 and 5 is admissible.

**A closed sheet is increment 4's own question, and no earlier increment meets
one.** `Extrude` never closes its wall set, so every prism sheet has free
edges, while a full revolution's does close (§4.1, Table W) and carries none.
The two mesh audits agree there — with no free edge, every directed edge has
its reverse, which is exactly what the closed-mesh audit counts — so
`docs/tessellation-design.md` §1.2's manifold-with-boundary audit is the one a
closed sheet runs, and it passes for
the same reason the closed-mesh audit would. Increment 4 states that in
`docs/tessellation-design.md` §1.2 rather than leaving it to coincidence.

Staged past increment 3, each with the gap §1.2 names: Thicken, Trim, Extend,
surface Offset, Ruled, Boundary Fill, Reverse Normal, and a sheet operand in
any boolean. Every one of them refuses at the call with `ErrUnsupported`
until its own design lands.

## 15. Test obligations

`CLAUDE.md` requires every capability to ship with a test asserting on
computed geometry, never merely that it ran. Each row below names a concrete
assertion, and each bound assertion must first be **shown to fail** — the
proof leg deleted, the test watched to go red — before it is trusted.

| # | Fixture | Asserted |
|---|---|---|
| T1 | a 100×60 mm rectangle, surface-extruded 10 mm `Along` | `Kind() == BodySheet`; `IsSolid() == false`; `Volume()` is `ErrNotSolid`; `Area` is exactly 3200 mm² and `Exact`; 4 faces; `Edges(Free()).Exactly(8)` resolves; `Bounds` equals the solid extrude's box, value and bound |
| T2 | the same profile, `Document.Patch` | one face; `Area` exactly 6000 mm² and `Exact`; `Edges(Free()).Exactly(4)` resolves; the positive side is the sketch plane normal |
| T3 | T1's walls plus a patch at each end, stitched | `Kind() == BodySolid`; `IsSolid()`; `Volume` exactly 60000 mm³ with a zero bound; `Area` exactly 15200 mm²; `Edges(Free())` matches nothing; `Verify` reads `Sound` |
| T4 | T3's operands with one patch displaced 1e-9 mm | the stitch returns a **sheet**, not an error; `Edges(Free()).Exactly(8)` resolves — the displaced patch's four edges and the four wall rims they failed to meet — while the other patch's four welded |
| T5 | T3's solid, unstitched then re-stitched | 6 sheets out; the re-stitched body's `Volume` equals T3's to the bit |
| T6 | a half-disc revolved a full turn about its diameter, as a surface | a closed sheet: `Kind() == BodySheet`, `Edges(Free())` matches nothing, `Volume()` is `ErrNotSolid`; stitching it alone is `ErrUnsupported` (R8) in increment 2 |
| T7 | a surface-extruded profile and a solid whose boxes meet, with an admitted transversal crossing between them | `Verify` reads `Interfering` with exactly one `DiagSheetSolidCrossing` naming the pair, and no `Interference` or `Clearance` row; `Passed()` is false |
| T8 | the same pair moved until the boxes separate | `Verify` reads `Sound`, with no diagnostic; under `WithClearances()`, a `Clearance` row carries the measured gap, `Exact` |
| T7a | a sheet frame around a smaller solid, boxes meeting but the frame's material never touching it | `Verify` reads `Sound`; under `WithClearances()` a `Clearance` row carries the closed-form gap, `Exact`, and no diagnostic |
| T7b | a sheet nested wholly inside a solid, boxes meeting | `Verify` reads `Sound`; under `WithClearances()` the SAME shape of `Clearance` row as T7a, `Exact`, and no diagnostic |
| T7c | a sheet whose one free-form wall the clearance kernel cannot model, against a solid whose boxes meet | `Verify` keeps `DiagUnsupportedPairSheet`, `Suspect`, message naming the kernel's failure to settle the pair rather than an unsupported sheet operand |
| T7d | two sheets whose boxes meet, in a fixture that would cross if either were a solid | `Verify` keeps `DiagUnsupportedPairSheet`, `Suspect`: neither operand offers a closed boundary to cast against |
| T7e | T7's fixture with the two bodies created in the opposite order | the `DiagSheetSolidCrossing` diagnostic's `Pair.A`/`Pair.B` follow `Document.Bodies()` order, not "sheet first" |
| T9 | a sheet handed to `Union`, `Fillet`, `Chamfer` and `Shell` | each is `ErrUnsupported`; the receiver and every operand stay live, and `Document.Bodies()` is unchanged |
| T10 | a sheet tessellated | a prism sheet tessellates and exports through the manifold-with-boundary audit (§10, Table D; `tessellate_sheet_test.go`); a revolve sheet still refuses with `ErrUnsupported`, deferred to increment 4 |
| T11 | a three-face assembly welded into a Möbius orientation | `Stitch` is `ErrDegenerate` (R7), and the document is unchanged |
| T12 | `Body.Patch` on a non-planar four-edge chain | `ErrUnsupported` (R6); and on a bounded-but-planar chain, `ErrUnsupported` on the same row |
| T13 | every Table R row | the stated sentinel, with `errors.Is` holding, and no document change |
| T14 | a wall extruded 5 inches `Along` (a nonzero-bound top rim, `document.go`'s unit-conversion rounding) against a patch built directly at the identical millimetre level (zero bound) | the stitch returns a **sheet**, not an error; `Edges(Free()).Exactly(12)` resolves — none of the four bit-identical rim/patch corner pairs join, because Table J's J5 refuses a nonzero-bound held value even where J4's coordinates match |
| T15 | `Body.Patch` filling a `Document.Patch` sheet's own sole 4-edge boundary | two faces; `Area` doubles to 12000 mm², `Exact`; `Edges(Free())` matches nothing; the new face's normal is the exact negation of the original face's |
| T16 | `Body.Patch` filling a single closed circular rim (a `Document.Patch` circle's own sole free edge) | which the original one-chain wording would have refused (§5.2); two faces, `Edges(Free())` matches nothing |
| T17 | `Body.Patch` capping BOTH rims of a surface-extruded tube's `Edges(Free()).Exactly(8)` in one call | this contract's own flagship case; two new 6000 mm² faces, `Exact`; `Edges(Free())` matches nothing |
| T18 | `Body.Patch` closing a stitched-but-still-open sheet's own last free edge | `Kind() == BodySheet` still, `IsSolid() == false`; `Edges(Free())` matches nothing |
| T19 | `Body.Patch` filling a `rectWithHoleSketch` `Document.Patch` sheet's own HOLE loop (`Edges(Free(), Concave())`) | two faces; the hole's own edge now bounds both; the four outer edges stay free; every face's normal equals the holed face's own, unlike T15 — a hole loop is walked clockwise, so the opposite sense the patch takes reads counter-clockwise and lands on the SAME side; `Area` returns to the rectangle's full 6000 mm², `Approximate` |

`.github/test-shards.txt` gains a row for every root-package test each
increment adds, and `go test . -run '^TestCIWorkflowRaceShardsCoverEveryPackage$'`
runs before any push that changes a root test name.
