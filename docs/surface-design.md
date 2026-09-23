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

A `Stitch`ed **solid** with more than one lump denotes the disjoint union of
the volume each lump encloses, never a shared or nested one: `Shell.void` is
hardcoded `false`, so a cavity — a void shell in the SAME lump as its outer
shell — is not a fact this evaluator records at all, and `Stitch` refuses a
boundary that would need one rather than publish a volume that double-counts
it (Table C, §6.4).

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
   proven planar by this arm and falls to the second arm below. A chain
   proven **non-planar** is `ErrUnsupported` (R6) outright: a non-planar
   patch needs a fitted free-form surface, which §1.2 stages, and no later
   arm admits it.

   **A second arm — the LEVEL half of the shared-denotation certificate — is
   tried only when this first, exact arm refuses on a nonzero bound**, and its
   ADMISSION decision never reads a coordinate, a residual or a bound
   magnitude. It requires every chain vertex, and every chain edge, to carry
   a `levelToken` (`denotation.go`) with the SAME non-zero id: a minted
   identity a straight prism build (`prism_build.go`) stamps once per swept
   end, only when its own record is drawn straight from the profile
   (`sectionDelta == 0`) and its frame axes are RECORDED rather than
   computed, so the only displacement in play is axial. A shared id proves
   the chain's true vertices lie on ONE plane — `frame.Origin + u·U + v·V +
   L·N` for one denoted level `L` — whatever bound each one's own held
   coordinate carries, because every one of them was stamped by the SAME
   evaluator call over the SAME recorded frame and level: a coplanarity
   proof by shared construction, never by comparison. Two independently
   built prisms whose bounded rims happen to hold bit-identical coordinates
   and bit-identical bounds still refuse — their level tokens were minted by
   two separate calls and are never equal, and nothing about admission ever
   compares the coordinates or the bounds themselves. A revolve's own seam
   is excluded on purpose: its half-plane's normal is itself bounded, not
   merely its offset, so `revolve_build.go` mints no level token, and a
   revolve seam chain stays on the first (exact) arm alone — still
   `ErrUnsupported` (R6). A chain admitted by neither arm is `ErrUnsupported`
   (R6): decad's reject-only rule treats "not proven planar by either arm"
   the same whether the failure was a proven non-planar chain or an
   undecided bounded one.

   **The published plane itself, once a chain is admitted this way, comes
   from the token — never from fitting one to the chain's own held vertex
   coordinates.** Held vertices at one recorded level are only
   APPROXIMATELY coplanar in float64: `frame.ToWorldUV(u, v)` rounds
   differently for each distinct `(u, v)`, so two vertices sharing one level
   generally do not land on bit-identical planes even though their true
   (unrounded) positions do — trivially zero for an axis-aligned sketch
   plane, where every cross term is an exact multiplication by 0 or 1, but
   not in general. A normal FITTED to that data (`patchChainOrientedNormal`'s
   Newell sum, which the exact arm's own admitted chains use safely, because
   gate 3 there already proved those SAME coordinates bit-for-bit coplanar)
   would tilt the published plane away from the true one by that same
   rounding, with no term in `axialDelta` to cover a TILT — `axialDelta` only
   ever states an OFFSET along an already-exact normal, on the same terms a
   prism cap's own `axialDelta` does. So a chain the level arm admits instead
   takes its plane's origin and normal from the token directly (`origin`,
   `normal`, transformed by this evaluation's own placement) — the same
   frame-derived construction `prism_build.go`'s own `capFrame` already uses
   for a solid prism's caps, needing no fitting argument at all.
   `patchChainOrientedNormal`'s fit still runs for such a chain, but only to
   settle which of the token's two normal directions matches the chain's own
   walk sense (`patchChainLevelNormal`): its magnitude, and any tilt fitting
   approximately-coplanar data would carry, are discarded, never published.
   `buildPatchFace` then sets the new face's `axialDelta`/`hasAxialDelta`
   from the token's own bound (folded with any placement rounding), the same
   fields `prism_build.go` already sets on a prism's own caps (no new field
   on `Face`). A chain the first, exact arm admits keeps its existing plane
   (fitted from its own proven-exact vertices) and a zero `axialDelta`,
   unchanged.
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

**J5 is lifted by the CURVE half of the shared-denotation certificate — a
separate proof from §5.2 gate 3's LEVEL half, over a separate code path.**
Coplanarity and coincidence do not follow from one another: a chain can be
provably coplanar with no two of its edges coincident, and two edges can be
provably coincident on a chain that is not planar. The LEVEL token proves the
first; the CURVE token below proves the second, and neither discharges the
other.

A `curveToken` (`denotation.go`) is minted once per denoted curve or point by
the evaluator that first builds it — today a straight prism's own rim edge
and rim vertex (`prism_build.go`, minted fresh per edge and per vertex, with
no `sectionDelta == 0` precondition: unlike coplanarity, an identity claims
nothing about the section itself) and a revolve's own internal junction edge
and its shared vertex (`revolve_build.go`; a revolve's SEAM — its boundary
copy at `phi0`/`phi1` — mints none, since it never has a partner to weld
against) — and propagated unchanged, composing the rigid motion it is applied
under, by every copier that reproduces the same geometry:
`rebuildStitchTopology` (`stitch.go`), `copyFaceUnderContext` (`unstitch.go`)
and `copyPatchFacesUnder` (`patch_body.go`). Two edges (or two vertices)
admit under the certificate when both carry a non-zero token, the ids are
equal, AND the motions they are stated under are equal
(`sameCurve`) — a placement breaks the certificate: two edges that shared a
denotation before one body moved no longer denote the same curve **at the
same place**, so the motion is part of the comparison, not merely the
identity. Two independently built bodies never share a token, whatever their
held coordinates or bounds say: two separate `Extrude` calls building the
identical profile at the identical extent each mint their own fresh ids from
the same document-local counter, which never repeats and never resets.

**The certificate admits; J3/J4's bit-identical comparison stays a
reject-only guard layered on top of it, and bit-identity alone never
admits.** `stitchVertexTable.classOf` (`stitch_weld.go`) gains a SECOND merge
route beside the existing zero-bound one: two vertices with the bit-identical
held key that also carry an equal non-zero token merge into one class too,
whatever bound either carries — narrowed by the same bit-identical key the
zero-bound route already requires, so a token match at a DIFFERENT held
coordinate never merges. `buildStitchWeldPlan`'s own edge gate widens from
"Line3 only" to "Line3, or a non-zero token": a `Line3` pair still joins
vacuously (J3 states nothing beyond what the shared vertex class already
proves), while any other pair — a revolve's own `Circle3`/`Arc3` junction,
which carries no bound field to decide J5 on at all — joins only through an
equal non-zero token AND `sameCurveVariant`'s own bit-identical guard
(`Center`/`Radius` bit-identical, `Axis` up to sign). Dropping this guard
would let a token match alone admit a pair whose HELD geometry disagrees,
which is exactly the tolerant admission this design refuses (`CLAUDE.md`'s
reject-only rule; `TestStitchNonzeroBoundRimStaysFreeAgainstExactPatch` is
the permanent proof that bit-identity alone never admits, and it stays green
unchanged by this certificate).

A rebuild from a record — `Placed`/`Duplicate`/`PlacedCopy` re-evaluating a
straight prism or a revolve from `evalPrismContext`/`evalRevolveContextWork`
— mints fresh ids every time, which is safe because a fresh id only ever
declines: two builds of the identical profile at the identical extent, one
placed and one not, never share a token regardless. A copy path — Unstitch
then Stitch, or a stitched/patched rebuild under a placement — instead
propagates the SAME id, composing the applied motion, which is what lets
that round trip re-admit exactly the pairs the original build already
proved coincident (§6.5, §15's T42).

### 6.3 Table C — what a stitch returns

After welding every admitted pair, the evaluator assembles the surviving faces
into shells and lumps, derives one consistent orientation across every welded
edge, proves every vertex's own link a manifold (§6.4), and decides the
result.

**Table C — stitch outcome**

| Assembled boundary | Result | Reason |
|---|---|---|
| some vertex's meeting faces do not form one connected fan (interior) or path (rim), open or closed alike | `ErrDegenerate` (R7) | the vertex-link audit runs before any row below is even considered; a pinch is proven at build time regardless of whether the boundary would otherwise close (§6.4) |
| at least one free edge remains | a `BodySheet`, open | the boundary is not closed; the residual free edges name exactly what did not join; an open multi-component assembly earns no lump-separation check either — it publishes no volume for a nested or interlocking pair of components to double-count |
| every edge welded, more than one connected component, and some pair's own axis-aligned bounding box (held vertex coordinates, each widened on both sides by that vertex's own proven bound) is not proven separate in any axis | `ErrUnsupported` (R20) | runs right after the vertex-link audit and the open check, before the two remaining rows below: a nested or interlocking pair of lumps would otherwise double-count, since neither the tetrahedron sum nor the per-surface flux integral carries any notion of which lump a triangle or face belongs to (§6.4) |
| every edge welded, all faces planar and straight-edged | a `BodySolid` | closure, manifoldness and non-self-intersection are proven, and the volume is exact (§6.4) |
| every edge welded, some face not planar-and-straight-edged, every face admits a landed flux arm (`Plane`/`Cylinder`/`Cone`/`Sphere`, zero `normalBound`) and the single source body proves the boundary simple by construction | a `BodySolid` | closure and the per-surface flux integral together prove volume and centroid; manifoldness rests on the vertex-link audit alone, since the reused crossing audit has no triangle set to run on (§6.4) |
| every edge welded, some face curved, and the set above does not admit | `ErrUnsupported` (R8) | closure is proven but this evaluator has no closed-form flux integral for the boundary as given — the surface kind, the face's own trim, or the construction proof is outside what has landed |
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
not already say better. **A pinched vertex link is not a residual free
edge, and this rule does not cover it.** An open sheet can still weld
cleanly at every edge it touches and yet pinch at one vertex — the same
shared-vertex-table mechanism that welds two coincident corners merges them
whether or not any edge joins the faces around them — so the vertex-link
audit above refuses it outright, `ErrDegenerate` (R7), even though it also
carries free edges of its own.

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
assembled triangle set**, and its bound is zero only when every held vertex
is exact AND that exact rational is itself representable in `units.Value`'s
`float64` magnitude — never unconditionally. Every held vertex is exact
whenever every welded vertex CLASS carries a zero bound, which is every case
this evaluator admitted before the CURVE certificate (§6.2's amendment): the
one rounding `Volume` then ever carries is the single publication step,
`Exact` exactly when the published number IS the sum (the box's 60000 mm³, a
small integer, always is), `Approximate` with a proven bound otherwise (a
body a third of a millimetre wide, whose exact volume is rarely
representable in cubic millimetres to the last bit).

**A weld the certificate admits changes what the triangle set's own vertices
are held to be, and `Volume`/`Centroid` charge exactly that.** A welded class
the certificate proves coincident by identity, rather than by a proven-zero
bound, is no longer zero-bound itself — the triangle set the tetrahedron sum
runs over is only within that class's own bound of the body's true vertices,
never `Exact` regardless of what the exact-rational sum over the HELD
vertices happens to round to. `evalStitchContext` charges this as
`massDelta`, `sweptVolumeAllow(massDelta, areaUpper)` widening the same term
a PLACEMENT's own rounding already widens (`delta`) rather than a second,
parallel proof: `massDelta` is `delta` further widened by the largest bound
any vertex class in the shared table carries
(`stitchVertexTable.boundByClass`), zero exactly when every class is, which
recovers the unwidened case unchanged. `Area` needs no equivalent charge — it
sums each already-built face's own `area`/`areaBound` through `boundedAdd`
(this section, above), and each face's own bound already covers a bounded
weld on its own boundary. `Centroid` and `Bounds` follow the identical rule
over their own publication rounding, `Bounds` needing nothing further since
`stitchBounds` already inflates each operand's own box by its own `Bound`.

**A curved face's flux term is not a tetrahedron sum**, and a per-surface
closed-form flux integral over an arbitrary trimmed analytic patch is its own
piece of work. §14's increment 3 lands `Plane`, `Cylinder`, `Cone`, `Sphere`
and `Torus`. Together they give a second admission rule beside the
tetrahedron sum's: **every face is either a `Plane` bounded entirely by
`Line3` edges (the tetrahedron path, unchanged), or a variant with a landed
flux arm, and every face carries a zero `normalBound`.** `stitch_flux.go`
owns the flux arms, `stitchRuleSAdmits` owns the second rule below.

**The `Cone` arm needs no general trimmed-boundary contour sum**, for the
identical reason `Plane` and `Cylinder` do not: this evaluator's own scope
restriction admits a `Cone` face only when it is bounded by exactly two full
`Circle3` rims at two distinct positions along the cone's own growth axis —
never a partial arc, never a generatrix edge — because that is the only
shape a reachable fixture (a frustum shell from a full-turn revolve of a
profile whose two non-radial sides both lean off the axis) exercises. A
`Cone`'s own vector-area integral, unlike a full-circumference `Cylinder`'s,
is NOT the zero vector — two full circles of DIFFERENT radii do not cancel
over a full turn the way two equal ones do — but it is still closed-form
without any contour-sum machinery: it reduces to the standard cone-shadow
identity `S_F = π(R_lo² − R_hi²)·Axis`, `R_lo`/`R_hi` the two rims' own
radii ordered by which sits nearer the apex, read through the identical
`boundedCircleRadius` this file's other arms already use. `flux_F = (apex −
anchor)·S_F` — `K_F = 0`, since the vector from the apex to any surface
point runs along a ruling and is therefore normal-orthogonal by the same
argument `NormalAt`'s own `Cone` case already encodes (`n = cosβ·radial −
sinβ·Axis`, orthogonal to any ruling direction). The apex itself is Origin
when `Radius` is exactly `0` — the ONLY case this evaluator admits, and
every reachable construction site (`revolve_build.go`, `capblend_geom.go`)
sets `Radius` to that literal constant (`Origin` already IS the apex), so
this costs no reachable face. The general formula, `Origin −
Axis·(Radius/tan(HalfAngle))`, is never evaluated: charging that division
would need a proven bound on `tan(HalfAngle)`'s own rounding, which this
evaluator has no sound way to produce — Go gives `Sin`/`Cos`/`Atan2`/`Hypot`
(and so `Tan`) no public ulp contract, a limit this codebase states
independently in at least four other places, and composing tan from
`boundedSin`/`boundedCos` through `boundedQuotient` — the natural-looking
fix — fails `boundedQuotient`'s own clearance check unconditionally, for
every angle, not only the degenerate ones, because `conservativeValueError`'s
structural bound on a `Sin`/`Cos` result is wider than the result itself.
So a nonzero `Radius` refuses (`ErrUnsupported`, R8) rather than publish an
apex this evaluator cannot bound, exercised at a hand-built internal
fixture. `coneApex` separately refuses a degenerate `HalfAngle` (`0`, or a
non-finite one caught one layer up by `units.Value.In`) regardless of
`Radius`, since no real `wallCone` ever carries one. The first-moment
sibling needs `tan²β` too, but reads it from the two rims' own radii and
axial positions (`(R_hi−R_lo)/(z_hi−z_lo)`) rather than from `HalfAngle`,
so no arm here ever needs `tan(HalfAngle)`'s VALUE, only its sign as a
validity check.

**The `Sphere` arm is scoped to a face with NO boundary loop at all** — a
complete, closed spherical shell — rather than to any trimmed shape: this
evaluator's only reachable `Sphere` fixture is a half-disc revolved a full
turn about its own diameter (§15's T50), and `fullRevLoops` mints a latitude
circle only for a junction OFF the revolve axis, while both of a
diameter-revolved semicircle's own junctions are poles ON it. So the face
carries zero loops and zero edges, and — since `Body.Vertices()` derives
from `Body.Edges()` — the whole stitched body carries zero vertices too. A
spherical zone or cap bounded by one or two rim circles would need the
general vector-area contour sum this file still does not build (this
section's own `Cone` paragraph), so any `Sphere` face carrying a boundary
loop refuses (`ErrUnsupported`, R8) rather than being guessed at.

Anchored at the sphere's own `Center`, `p − Center` is parallel to `n` at
every surface point, so `(p − Center)·n` is identically `σ·Radius` — the
same identity the `Cylinder` arm uses, with `Center` standing in for a point
on the cylinder's axis — giving `K_F = σ·Radius·f.area`, the identical shape
`Cylinder`'s own `K_F` takes. `Sphere` carries `Radius` as a bare
`units.Value` with no bound field, exactly like `Cylinder` and `Cone`, but
this arm cannot read `boundedCircleRadius`'s edge-length route the way they
do: a zero-loop face has no rim edge to read a circumference from at all.
Instead `boundedSphereRadius` inverts the face's own already-proven
`area`/`areaBound` (`Area = 4πR²`, so `R = √(Area/4π)`) through
`boundedQuotient` and `boundedSqrt` — a reuse of an already-published
reading, on the same terms `Cylinder`'s own `K_F` reuses `f.area` rather
than integrating anything fresh, never a fresh trust of the bare `Radius`
field. `boundedSqrt`'s own rational bracket (`ratSqrtDown`/`ratSqrtUp`) is
what makes the inversion itself sound, since Go's `math.Sqrt` carries no
accuracy contract this file would otherwise have to lean on either.

Because a zero-loop face has no boundary to sum `½∮p×dr` over, `S_F` is
exactly the zero vector by construction — no trig integral to collapse,
unlike `Cylinder`'s full-circumference argument — so the `(Center −
anchor)·S_F` cross term vanishes for EVERY anchor, and `flux_F` is exactly
`K_F` regardless of which anchor `stitchCurvedMass` is handed. That is what
makes this arm safe to reach with the zero anchor `evalStitchContext`
substitutes when the shared vertex table is empty (stitch.go's own doc
comment at that substitution, written against exactly this fixture before
this arm landed). The first moment follows the general shift-of-origin
identity for a region of volume `V` centred at `Ĉ`: `∫_Ω(x_i−a_i)dV =
V·(Ĉ_i−a_i)`, valid here because a zero-loop `Sphere` face is never one of
several faces sharing a boundary with others — it IS the whole closed
boundary of the ball it bounds, on its own — so `M_i = (flux_F/3)·(Center_i
− anchor_i)`, `flux_F/3` being this face's own signed volume by the same
normalization `stitchCurvedMass`'s own `vol := fluxSum/3` uses for the
total.

A zero-loop `Sphere` face also means `checkStitchClosure`'s directed-edge
parity leg and the vertex-link audit (`auditVertexLinksForStitchFaces`) both
walk an empty edge set and pass VACUOUSLY — neither proves anything about
this fixture's manifoldness, and closure and non-self-intersection rest
entirely on Rule S's own construction proof (the revolve axis argument,
above). A mathematical sphere built this way has no pinch point to catch in
the first place — both of the generating semicircle's own axis contacts are
its two poles, the ordinary way a sphere closes, never the two-isolated-
interior-point lens shape this section's vertex-link paragraph names — but
the vacuous pass is recorded here rather than left to read as a proof it is
not.

**The `Torus` arm is scoped to the ONE window shape this evaluator can
integrate without an unbounded trig call**, and admits nothing wider — this
evaluator's only reachable `Torus` fixture is `offAxisSemicircleSketch`'s
own full-revolution sheet (§15's T53), a straight chord at the tube's own
equatorial radius (revolving into the face's own `Cylinder` sibling)
alongside a semicircular arc bulging outward from that radius to
`Major+Minor` and back, sweeping exactly the tube's own OUTER quarter-to-
quarter window, `φ ∈ [−π/2, π/2]`, measuring `φ` from the plane through the
tube's own centre circle. A torus zone's `K_F = ∫∫ (Major·cosφ + Minor) dA`
integral is not constant the way `Cylinder`'s and `Sphere`'s are, so
integrating it needs the window's own endpoints as raw angles — and its
antiderivative carries a term LINEAR in that raw angle (`∫cos²φ dφ =
φ/2 + sin2φ/4`), not reducible to `sinφ`/`cosφ` alone, so recovering it from
a rim's own proven radius and axial position would need an inverse trig
function of computed data, which this evaluator has no sound bound for (the
`Cone` arm's own apex paragraph, above, states the identical limit at
length). Nor can `Major`, `Minor` and the window be recovered algebraically
without one: two rims of the SAME radius at axial offsets `±e` from
`Center` satisfy `(ρ−Major)²+e² = Minor²` for infinitely many `(Major,
Minor)` pairs — `Major=10, Minor=5` (window `±90°`) and `Major=8,
Minor=√29≈5.385` (window `≈±68.2°`) both put a full circle of radius 10 at
axial offset `±5` from the SAME centre — a genuine ambiguity, checked by
hand, not a derivation this evaluator merely has not found. So the ONLY
window this arm can integrate without an unbounded trig call is one whose
endpoints are KNOWN constants rather than recovered ones, and `±π/2` is the
sole such window any reachable fixture in this tree ever presents (a
complete, zero-loop torus is the other, `Δφ = 2π`, the `Sphere` arm's own
shape taken one variant over — no reachable fixture ever builds one).

Trusting `Major` and `Minor` off the tag at all needs its own gate first,
for a reason neither `Cylinder`, `Cone` nor `Sphere` shares: unlike a rim's
own circumference or a face's own area, a torus's two rims and its own
proven area do not determine `Major` and `Minor` independently even
together — the identical ambiguity above. So this arm reads
`Torus.Major`/`Torus.Minor` from the tag directly, gated on
`torusAxisIsCoordinateAligned` (the axis is exactly a signed coordinate
vector, and `Center` sits exactly on it through the world origin) — the one
condition under which that read carries the identical zero bound
`boundedCircleRadius`'s own doc comment names for the other radius fields'
axis-dependent rounding. Given that gate, the two rims' own axial offsets
from `Center` (`boundedDot`, EXACT under an axis-aligned `Axis`) are checked
against `±Minor` EXACTLY — never a tolerance, since a small residual proves
nothing (CLAUDE.md's own rule) — which is what proves `sinφ = ∓1` and so
`cosφ = 0`, `φ = ±π/2` EXACTLY, a pure algebraic consequence of the check
rather than a measurement of an angle. `torusFaceFluxAndMoment`'s own doc
comment carries the closed forms this gate makes reachable,

	K_F = 3·π²·Major·Minor² + 4·π·Minor·(Major² + Minor²)
	M_i = π·Minor·(Center_i − anchor_i)·(2·Major²·(1−Axis_i²) + π·Major·Minor + (4/3)·Minor²)

verified against independent numeric double integration before landing.
`K_F` needs no cross term: the window's two rims share the SAME radius
(`Major`, by the check above), so — exactly as `Cylinder`'s own
full-circumference argument shows for two equal-radius circles — the face's
own vector area is the zero vector, and `flux_F` is exactly `K_F` for every
anchor.

**`Face.normalBound` is nonzero exactly for a cap-blend band patch**
(topology.go's own field doc): the face is a ruled surface and the `Cone` or
`Plane` it publishes is that surface only to within a measured departure.
Integrating a closed form over the tag would be unsound for such a face, so
every admitting arm — the tetrahedron path included, not only the new flux
arms — requires a zero `normalBound`. An `Unstitch`ed fillet or chamfer face
re-stitched into a closed set refuses on this gate alone (§15's T38).

**The split is "`Plane` bounded entirely by `Line3`" versus everything
else, never "planar versus curved".** The old wording ("all faces planar")
was already wrong for a reachable shape: a `Plane` face bounded by a
`Circle3`/`Arc3` edge triangulates, under the tetrahedron path's own
polygon-from-coedge-start-vertices construction, to a boundary that drops
the arc's own bulge. No construction reaching `Stitch` today puts such a
face through that path without ALSO putting a genuinely curved surface
elsewhere in the same closed set (which already refused before
triangulation), so the shape was latent rather than live — closed here
regardless, by `faceIsTetrahedronEligible`'s own edge-kind check
(stitch.go), rather than left for a future fixture to discover it live.

**Rule S — the construction-proof gate a curved closed set needs, since the
reused crossing audit cannot run on one.** That audit (this section's own
opening paragraph) consumes a triangulated face set, which a curved
boundary has none of, and chording curved faces into one just to run it
would be an admission gate resting on an approximation — CLAUDE.md's
reject-only rule forbids exactly this, and `verify.go`'s own
`payloadProvesSimple` doc comment already states the two ways such a chord
mesh can be wrong in both directions. So a curved closed set's only
available proof of non-self-intersection is the one its own SOURCE
feature's construction already carries: `verify.go`'s `payloadProvesSimple`,
built for `Verify`'s own Table V leg 4 and reused here unchanged. Before the
flux arms ever run, `stitchRuleSAdmits` requires every operand face to
descend from exactly ONE source body (`stitchOperandBodies`), whose own
payload `payloadProvesSimple` admits. A full-turn `revolvePayload` clear of
the axis (an annular revolve sheet) admits; a `bodyPatchPayload` admits only
under Rule P below — on its own, an unqualified `Body.Patch` chain proves
nothing about the whole assembled boundary. A stitch of curved sheets from
two different features also refuses: Rule S's single-source restriction has
no way to compare two features' own proofs against each other.

**Rule P — the construction-proof gate a `Body.Patch`-capped boundary earns
from its own receiver**, closing §6.1's own motivating "walls, then cap,
then stitch" story for the one receiver shape it was written against. A
`bodyPatchPayload` proves its own boundary does not self-intersect, and so
admits Rule S, when all three hold, decided by `patch_body.go`'s
`bodyPatchPayloadProvesSimple`:

1. its receiver — the body `Body.Patch` was called on — itself admits under
   Rule S, decided by the identical `payloadProvesSimple` predicate, never a
   reimplementation of it;
2. every new face's chain is a COMPLETE free-edge chain of that receiver's
   own end, not a proper subset of one;
3. every new face's plane is exactly one of the receiver feature's own end
   planes.

Under those three the patched assembly IS the receiver feature's own solid
boundary — every chain vertex and edge `Body.Patch` selected is the SAME
object the receiver's own build stamped, never a copy or a fit — so the
receiver's own construction proof carries onto the patched body unchanged.
Any chain this cannot decide keeps the payload undecided, reject-only exactly
as every other Rule S arm.

Conditions 2 and 3 are both decided through the LEVEL half of the
shared-denotation certificate (`denotation.go`, §5.2), never by a coordinate
or a residual: a straight prism build (`prism_build.go`'s
`evalPrismContext`) stamps every rim vertex and edge at one end with the
SAME level token, minted fresh per build and per end whenever the build's
own section is drawn straight from its record (`sectionDelta == 0`) —
regardless of whether that end's own coordinate ends up zero-bound or not,
so the mechanism covers a plain `Distance` extrude exactly as it would a
computed one. Condition 3 asks whether a chain's own edges and vertices all
carry one shared, non-zero level id, proving the new face's plane IS that
recorded level's plane by identity — the same token §5.2's own gate 3 level
arm already reads for the identical reason, read here regardless of which of
gate 3's two arms actually admitted the chain's planarity. Condition 2 asks
whether that id's FULL set of the receiver's own free edges is exactly the
chain's own edge set, never a proper subset of it: a receiver whose end
holds more than one disjoint free-edge loop — an annular profile's inner and
outer rims at one level — proves nothing about the WHOLE end's
non-self-intersection from patching only one of them, so admitting on a
subset would be unsound. Only `prismPayload` mints level tokens today, so
Rule P admits a `Body.Patch`-capped surface-extruded tube's rims and nothing
wider yet: a loft, sweep or revolve receiver's own `Body.Patch`-capped sheet
stays undecided until a later increment mints a level token for it too, as
does a chain a SECOND `Body.Patch` call selects from an already-patched
body — `copyPatchFacesUnder` does not propagate a level token onto the
copies it mints, so nothing but the receiver's OWN first-hand rim ever
carries one.

**The vertex-link audit — manifoldness's own remaining gap, reject-only on
every arm.** Rule S proves non-self-intersection; it says nothing about
whether the faces and edges meeting at one VERTEX stay in one connected
piece, which `checkStitchClosure`'s own directed-edge parity leg does not
ask either (docs/surface-design.md's own record: a profile that touches its
revolve axis at more than one isolated point can pinch a revolved boundary
at those points, non-manifold there even though the parity leg and Rule S
both pass; two otherwise-unconnected stitched bodies that share exactly one
welded vertex-table entry, with no edge joining them, pinch the identical
way through the all-planar path). `sweep_composite.go`'s own
`auditVertexLinks` — hoisted out of `auditCompositeVertexLinks`, its `*Body`
parameter dropped along with a leg that turned out to prove nothing about
its own two inputs — is the existing reject-only mechanism for exactly
this, and `Stitch` runs it unconditionally, immediately after
`checkStitchClosure` proves the directed-edge parity leg and before Table
C's own closure/curvature branches decide anything at all — on every build
arm, all-planar or curved, open or closed alike, never the curved-closed
arm alone. The all-planar arm has a reachable public fixture that trips it:
two boxes sharing one corner vertex with no shared edge (§15's T76)
weld cleanly under Table J, and both `checkStitchClosure` and the reused
crossing audit admit the result, since neither one asks whether the faces
meeting at the shared vertex stay in one connected piece — the vertex-link
audit is the only leg standing between that shape and a falsely published
solid. The curved arm's own reachable fixtures still cannot trip it, for
two different reasons depending on the surface kind. Every admitted
`Plane`/`Cylinder`/`Cone`/`Torus` shape this increment's own scope reaches
stays strictly clear of the revolve axis — a `Cone` wall's own scope
restriction (the paragraph above) admits only two full-circle rims at two
distinct, provably positive radii, never a rim collapsed onto the axis, and
the `Torus` arm's own `±π/2` window (this section's own `Torus` paragraph
above) is bounded by two full circles at the tube's own equatorial radius,
`Major`, itself strictly positive whenever the arm admits at all. The
`Sphere` arm's own reachable fixture DOES touch the axis, at its
generating semicircle's own two poles — but a full-turn revolve's
`wallAxis` classification mints no face, edge or vertex at all for a
boundary stretch lying ON the axis, so a `Sphere` face's two poles carry no
topology for the audit to even examine (this section's own `Sphere`
paragraph above); the audit runs on an empty edge set and passes
vacuously, which is a different reason than "clear of the axis" but stays
just as safe, since a sphere closing at two ordinary poles has no pinch to
catch in the first place. A boundary pinch at an isolated INTERIOR point,
which needs a generatrix that touches the axis at a point other than its
own endpoint to produce, stays out of reach on the curved arm regardless
of surface kind. The leg stands as a proven-safe backstop for a curved
fixture whose generatrix touches the axis at an interior point, not a case
any of this increment's own curved tests can observe firing — the
all-planar T76 fixture above is what observes the leg firing at all.

**What is proven, and what is not, for a curved closed set.** Closure (the
directed-edge parity leg) and manifoldness at every vertex (the hoisted
audit) are proven directly. Non-self-intersection rests entirely on Rule
S's construction proof, never on a geometric test this evaluator runs
itself — a strictly different, and for a `Body.Patch`-capped flow strictly
narrower, standing than the tetrahedron path's own crossing audit gives an
all-planar solid. §9.1's own leg 4 for `Verify` is the identical proof,
reused rather than re-derived.

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

**The clearance kernel's own carrier model (`docs/clearance-design.md` §2)
admits a stitched body only when the recorded triangle set exists — every
face tetrahedron-eligible, this section's own closed, all-planar case — AND
every vertex of the body carries a proven bound of exactly zero**, on
`addPrismFaces`' own `sectionDelta != 0` reasoning (`clearance_geom.go`): the
kernel's certificates are exact statements about the carriers it reads, so a
body whose vertices are only within a bound of their true position is a body
it cannot model. The second condition subsumes a nonzero placement `delta`,
since a placement widens every vertex bound; it also subsumes a CURVE
certificate's own class bound (this section's own `massDelta` paragraph
above). A pair holding a body the gate refuses reads undecided rather than a
falsely precise gap — reject-only, never an approximation dressed as an
answer.

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
`Unstitch` frees was welded from a proven-coincident pair, so Table J
re-admits every one of them and re-stitching the results reproduces the body
— now true for a BOUNDED weld too, not only an exact one, because
`copyFaceUnderContext` propagates the CURVE half of the shared-denotation
certificate (composing the applied motion) rather than dropping it, so the
second `Stitch`'s own certificate route re-admits exactly the pair the
original build already proved coincident (§6.2's amendment, §15's T42).

**A revolve's own internal junction weld now round-trips too, closing back
to what the never-unstitched sheet itself already published — its own
free-edge shape, not the whole boundary.** `revolve_build.go` mints a curve
token for a junction edge and its shared vertex (the boundary an internal
`Circle3`/`Arc3` shares between two adjacent side faces before Unstitch ever
splits them), so `Unstitch` then `Stitch` re-admits that pair on the
certificate route (`sameCurveVariant`'s own bit-identical guard) exactly as
it always could for a straight prism's own `Line3` rim. A revolve's SEAM —
its boundary copy at `phi0`/`phi1`, a partial revolve's own free rim — mints
no token and never had a partner to weld against, before or after the round
trip, so it stays free either way: the round trip closes what was welded
before Unstitch, and leaves free what was already free, reproducing the
original sheet's own shape rather than losing it to every edge going free.

**`unstitchPayload` (`unstitch.go`) is the held-B-rep-under-a-rigid-motion
mechanism restricted to one face**, `stitchPayload`'s own idea narrowed from a
whole welded set to a single held `*Face`: it records that face and the
retiring receiver's own already-proven `Bounds`, never a built topology, and
`placed()` deep-copies fresh surface, loop, edge and vertex geometry under the
composed transform every time, exactly as `stitchPayload.placed` replays
`evalStitchContext`. Nothing in either result aliases the receiver's own
`*Face`, `*Edge` or `*Vertex`, so a retired receiver stays readable and
unmodified for any caller still holding it.

**Every result carries its source face's own `axialDelta`, `hasAxialDelta`
and `normalBound`, never their zero value.** The copy shares the source
face's identical surface and identical tag — `Unstitch` mints no new
geometry of its own — so `normalBound` (how far that surface departs from
that tag) and `axialDelta` (that same tag's own displacement along that same
normal) stay exactly as true of the copy as they were of the source.
`hasAxialDelta` and `normalBound` copy verbatim; `Body.Unstitch`'s public
call always places under the identity transform (`UnstitchContext`), so a
verbatim copy is correct for every call the public API makes today.
`unstitchPayload.placed`'s own replay under a composed, non-identity
transform additionally widens `axialDelta` by `absSumUpper` against the
placement's own proven displacement, the same treatment the vertex bound and
edge `lengthBound` already receive. `normalBound` is dimensionless while
that displacement is a length, so no such composition covers it, and this
package carries no separate term bounding how far a placement's own rounding
rotates the tag frame off the true rotation — so a placed copy of a face
whose `normalBound` is nonzero is `ErrUnsupported` rather than an invented
bound. `stitch.go`'s `rebuildStitchTopology` carries the same three fields
the same way, so a stitched body's own later placement never reopens this
gap.

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
| R7 | `Stitch`'s welded set cannot be consistently oriented, fails the directed-edge parity leg, or has a vertex whose meeting faces are not one connected fan or path | `ErrDegenerate` |
| R8 | `Stitch` closes a boundary holding a face this evaluator has no closed-form flux integral for | `ErrUnsupported` |
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
| R20 | `Stitch`'s assembled lumps are not proven mutually separate by axis-aligned bounding box | `ErrUnsupported` |

R6, R8, R10 and R20 are `ErrUnsupported` rather than `ErrDegenerate` on
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
| `Undercut` | `CoverageNotRequested` when omitted; when requested, **only a surface-extruded prism sheet answers**: `CoverageComplete` / `CoveragePartial` / `CoverageUndecided` exactly as a solid's own three-valued reading (§9.1 below); every other sheet family — loft, stitch, one-span sweep, revolve, arc/composite sweep, `Patch`, `Unstitch`, or a prism/loft sheet with a nonzero section displacement — reads `CoverageUnavailable`, with `DiagUnsupportedSurveyPayload` when its payload class is one the survey dispatch does not name, or `DiagSurveyPrerequisite` when its own validity is not `ValidityValid` |
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
and the rigid placement that follows preserves that. A surface-result loft
earns it on a STRONGER argument: the evaluator cannot return a loft body at
all unless `docs/loft-design.md` §6's crossing audit already passed over the
COMPLETE held triangle set — walls and both caps together — so
non-self-intersection of the walls alone, the sheet's own published faces
once the caps are omitted, follows from non-self-intersection of that
superset with no further proof needed. A positive section displacement (that
loft's own `sectionDelta`) means the body denotes a curved surface the held
chords are only within that displacement of, so simplicity of the chord mesh
does not transfer to the surface it stands for, and the fourth leg is
undecided rather than violated there too. A revolve sheet earns the fourth
leg on a different argument, since its own build runs no crossing audit over
a triangle set at all: it admits when the sweep is exactly one full turn
(`full == true`) AND the recorded profile's radial minimum about the resolved
axis is proven non-negative. Every boundary stretch lying on the axis is
classified `wallAxis` and emits no face, so the face set is the revolution of
the boundary's off-axis part alone; two distinct off-axis generating points
map to the same 3D point only by sharing the same radius and axial position,
which a simple closed planar region's boundary — `sketch`'s own proof — does
not repeat, so that face set is injective off the axis, and `full` makes the
angular fibre traverse it exactly once, so no off-axis point is covered
twice. The radial condition is deliberately stricter than `resolveAxisSide`'s
own build-time gate, which admits down to a `-tol` band rather than proving
the minimum clear of zero — an admission gate resting on a tolerance is what
CLAUDE.md's reject-only rule forbids, so this leg re-decides the question at
zero instead of inheriting the build's tolerance. A revolve profile can never
carry a free-form segment reaching this leg either: `resolveAxisSide` refuses
one before this leg is ever reached, so no separate free-form gate is needed
here. A partial-turn revolve sheet, or one whose radial minimum is not proven
clear of the axis, earns no such proof: the fourth leg has nothing to stand
on, which is undecided, not a violation. Any other construction earns no
proof either, for the same reason, EXCEPT a `bodyPatchPayload` that admits
under §6.4's Rule P: the identical `payloadProvesSimple` predicate serves
both `Stitch` and `Verify`, so a `Body.Patch`-capped sheet that closes its
receiver's own rim by identity earns the fourth leg here too, whether or not
it is ever stitched into a solid. A free edge, which on a closed body would
be the watertightness failure, is the expected shape here and is counted by
the structural legs rather than faulted.

**`Region` is nil for a sheet even when `Validity.Outcome` is
`ValidityValid`.** `docs/verification-design.md` §1 currently states the
biconditional over validity alone; §12 amends it to validity **and** kind. A
sound sheet is a sound *boundary*, which is the whole of what the caller
asked to be true about it.

**The wall and concave-radius surveys need a solid**, and say so. Each asks a
question about material — how thin a wall is, how tight a concave feature is
— and a sheet has none. `DiagSurveyPrerequisite` is the existing code for "a
requested survey needs a proven solid"; §12 widens its stated cause to
include a sheet asking either one, so a caller branching on it already
handles this. The undercut survey is different: draft analysis on an
oriented sheet is a real question — Fusion answers it — and §14's increment 3
takes it up over a surface-extruded prism sheet's positive side, the one
sheet family whose walls carry every input the existing reader needs
(§9.1's Table V row above). A sheet whose family the survey cannot yet
answer reads `CoverageUnavailable` through `DiagUnsupportedSurveyPayload`
rather than `DiagSurveyPrerequisite` — it is not blocked on having no
material, it is blocked on this evaluator not yet proving one over that
family's geometry.

### 9.2 What a sound sheet costs

**This is true only for a family whose construction proves §9.1's fourth
leg** — today the surface-extruded prism and the surface-result loft, each at
a zero displacement (sectionDelta for the loft). **A document holding only
sheets from such a family verifies fully and reads `Sound`, provided no survey
was requested.** Nothing about a PROVEN sheet makes a report `Suspect` on its
own: its boundary quantities are gated like any other, its validity is decided
in `Verify` — exactly where every other body kind's is, so a sheet's audit is
not a special build-time step — and an omitted survey reads its
`NotRequested` outcome as everywhere.

Two things do cost a PROVEN sheet a `Suspect`, and both are the caller asking a
question a sheet cannot answer: a **requested** wall or concave-radius
survey, which reads `Unavailable` with a `DiagSurveyPrerequisite` (§9.1); and
§9.3's pair rule, where a sheet's box meets a solid's. Neither fires on a
model that only holds proven sheets and only asks the core questions. A
**requested undercut survey is not one of these two** on a surface-extruded
prism sheet at a zero section displacement: it answers over the sheet's own
positive side (§9.1) exactly as it would on the equivalent solid, so a
document holding only such a sheet and asking `WithPullDirection` can still
read `Sound` when every wall provenly clears.

**A sheet whose family has no such proof reads `Suspect` regardless of what
the caller asks.** A PARTIAL-turn revolve sheet's fourth leg is undecided —
its own build runs no crossing audit over its triangle set at all, and §9.1's
construction argument needs a full turn — so a document holding one, even
alone and with no survey requested, already reads `Suspect` through
`Validity.Outcome == ValidityUndecided` and its `DiagUndecidedValidity`
diagnostic. A full-turn revolve sheet whose radial minimum is proven clear of
the axis instead earns the fourth leg by construction (§9.1) and can read
`ValidityValid`. The arc-reduced and composite sweep sheets a later increment
adds inherit the partial-turn sheet's undecided leg for the identical reason:
neither construction has a crossing-audit proof, nor §9.1's full-turn
argument, for `Verify`'s switch to read.

### 9.3 Pairs

**A stitched `BodySolid` takes the solid-solid path (`clearancePair`), never
this section's own `sheetSolidPair`** — §6.3's Table C already decided it is
a solid, not a sheet, so none of this section's box rule or its diagnostics
govern it; `docs/clearance-design.md` §2/§8 own its own carrier-model gate
and the diagnostics a pair holding one reads.

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
surface-result **revolve** sheet tessellates and exports the same way: its
walls chord exactly as the solid's, a partial sweep omits both caps from the
mesh — and, where the profile meets the axis, the on-axis edge only the caps
carried — and a full revolution mints no cap in either kind, so its sheet
mesh is bit-identical to the solid's and carries no free edge at all. With no
free edge to tell them apart, the manifold-with-boundary audit agrees with
the closed-mesh audit that a full-turn sheet would otherwise run, which is
where a CLOSED sheet's mesh is settled (§14). The signed-volume orientation
check (`docs/tessellation-design.md` §1.2) runs only when the mesh is
closed — a solid, or a full-turn sheet — and never on an open partial-sweep
sheet, where that sum is anchor-dependent and decides nothing.

A surface-result **loft** sheet does not yet: `Tessellate`/`STL`/`OBJ` refuse
it with `ErrUnsupported` at the dispatch that would otherwise look up a role a
surface result's own build never attached (§4.2) — this evaluator's own
reach, never the `ErrDegenerate` a missing face role would otherwise report,
since the body's geometry is not the problem. A loft sheet's own refusal sits
ahead of `tessellateLoft`'s exact restatement (`docs/loft-design.md` §9 Table
D row D1), which already holds the complete triangle set including both
omitted caps; the refusal is what keeps that set from being restated as a
solid mesh for a body with no material.

**A stitched `BodySolid` is a different case from every sheet above, and its
own mesh is a separate, later-landing capability (§14 Table D row 5).** A
stitched solid is not a sheet — Table X's whole subject — so nothing above
governs it. A CLOSED, all-planar stitched body (`stitchAllTetrahedronEligible`,
§6.4: every face a `Plane` bounded entirely by `Line3` edges, zero
`normalBound`) restates the triangle set `Stitch`'s own build already
assembled and audited (`tessellate_stitch.go`), the same shape
`docs/tessellation-design.md` §2's `loftPayload` row restates a loft's: no
chording, no retriangulation, no moved coordinate, and so no chord tolerance
to take. Its per-face bound is the largest `Vertex.Bound()` over the vertices
that face's own triangles touch, zero exactly when every one of them is —
the box worked example's own case.

**An OPEN, all-planar stitched body IS a sheet** (`Kind() == BodySheet`,
Table X's `Tessellate`/`STL`/`OBJ` row), and its mesh is the identical
restatement of the same triangle set `Stitch`'s own build already assembled —
the one it triangulated to check its own perturbed area sum (§6.4) and, until
this increment, discarded afterward. It runs
`docs/tessellation-design.md` §1.2's manifold-with-boundary audit in the
closed-mesh audit's place, exactly as a surface-result prism or revolve
sheet's own mesh does above: `requireSheetMesh` proves every free directed
edge attributes, face by face and chain count by chain count, to the body's
own recorded free `Edge`s, and `requireSheetVertexLinks` is its own
vertex-link safety net for an open boundary vertex's path-shaped link.
Attribution holds with no role lookup at all —
`docs/tessellation-design.md` §4's own point about a stitched body's
non-unique roles — because `stitchPayload.triFaces` and `Body.Edges()`'s own
`Faces()` both name the identical LIVE face pointer `rebuildStitchTopology`
built, so the mesh side and the body side of the audit agree by construction,
never by coincidence or by a second geometric test.

A stitched body holding a face that is not a `Plane` bounded entirely by
`Line3` edges stays `ErrUnsupported` whether the body is open or closed,
staged past this increment. The mesh publishes a zero occupied-volume proof
(`symDiffOK == true`) only for a CLOSED, all-planar body whose every vertex
carries a proven bound of exactly zero (`stitchZeroVertexBound`, the same
gate `docs/clearance-design.md` §2's stitch arm applies): every held vertex
is then the true boundary vertex, every triangulated polygon is that face's
own exact `Line3` boundary, and ear clipping tiles it exactly, so the held
triangle set occupies exactly the denoted volume, and `Union`, `Cut` and
`Intersect` admit it like any other zero-bound operand. Every other stitched
body — open, curved/mixed, placed, or certificate-welded — publishes
`symDiffOK == false`, so a boolean keeps refusing it: an open one on the
identical reasoning Table X's boolean row states for any sheet, a closed one
on `boolean.go`'s own `requireVolumeProvingPayload` arm — whatever the exact
tetrahedron sum itself already proves about signed volume.

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
| `Tessellate` / `STL` / `OBJ` | a prism, revolve or all-planar stitched sheet tessellates and exports; a loft sheet, or a stitched sheet holding a face that is not a `Plane` bounded entirely by `Line3` edges, is `ErrUnsupported`, staged (§10) | the manifold-with-boundary audit §10 describes runs on the prism, revolve and stitched paths; the loft path, and a curved or mixed stitched sheet, await a later increment |
| `ToFace` / `ToFaceAngular` naming a **planar** face of a live sheet | admitted | the stop reads the face's plane and nothing about material, so `stops.go`'s resolution is unchanged |
| `ToFace` naming a curved face of a sheet | as for a solid | this design changes no curved-stop reach |
| `EdgeAxis` naming a linear edge of a live sheet | admitted | the axis reads the edge's line; `docs/api-design.md` §6.2's exactly-one and liveness rules apply unchanged |
| `ThroughAll` / `ThroughAllSide` resolving its stops over a document holding a sheet | the sheet is skipped, not refused | a sheet encloses no material to stop against, and refusing would stop any document holding one sheet from using `ThroughAll` at all |
| `Body.Faces` / `Edges` / `Vertices` / `Lumps` / `Shells` | admitted, unchanged | traversal and inspection, as on every body |
| every selector predicate | admitted, unchanged | a sheet's faces and edges answer `Planar()`, `Convex()`, `CreatedBy()` and the rest on the same terms |

A sheet is a first-class body everywhere it is admitted: it is registered in
the document, it appears in `Document.Bodies()`, `Verify` reports on it, and
it retires and copies on `docs/api-design.md` §6's uniform terms.

A stitched body that closes to a `BodySolid` is not a sheet at all, so no row
above governs its mesh or its boolean reach: §10's own stitched-solid
paragraph and `docs/tessellation-design.md` §2's `stitchPayload` row state
its mesh, and the boolean row's own reasoning above — no occupied-volume
proof, so no boolean admits it — is exactly what a stitched solid's mesh
restates unchanged, until a later increment states one.

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
| 3 | `WithSurfaceResult()` on `Sweep` and `Loft`; the shared-denotation certificate — two distinct proofs sharing one name, never one lifted "together": a LEVEL token proving N chain vertices coplanar by shared construction, which lifts §5.2 gate 3's bounded-chain half of R6 for a straight prism's own rim; a separate CURVE token proving two edges (or two vertices) denote one curve or point, which lifts Table J's J5 for a straight prism's own rim and a revolve's own internal junction, and does not follow from the level token proving anything — coplanarity and coincidence are different proofs over different code paths; the per-surface flux integral that lifts Table C's curved-closure refusal (R8); the undercut survey over a surface-extruded prism sheet's positive side — the only sheet family this increment opens it on; a loft, stitch or one-span-sweep sheet moves from `DiagSurveyPrerequisite` to `DiagUnsupportedSurveyPayload` for it instead, and stays there until its own proof lands |
| 4 | The revolve sheet mesh (§10): the meridian and angular chordings a surface result keeps, the caps it omits — and, where the profile meets the axis, the on-axis edge between two poles that only the caps carried (Table W) — the cap terms its area slack drops, and the manifold-with-boundary audit in the closed-mesh audit's place. It also settles which audit a CLOSED sheet runs |
| 5 | An all-planar stitched body's own mesh, CLOSED or OPEN: `stitchPayload` records the final wound triangle set `Stitch`'s own build assembled and audited (§6.4), attributed by the live rebuilt face per triangle rather than by role (two welded operands can carry the same role string), and `tessellate_stitch.go` restates it with no chording. A CLOSED body runs the closed-mesh audit plus its own vertex-link safety net over that restated set; an OPEN body — a sheet — runs `docs/tessellation-design.md` §1.2's manifold-with-boundary audit instead, its free-boundary attribution agreeing with the body's own recorded free `Edge`s by the identical live face pointer on both sides, never a role lookup. A curved or mixed stitched body's mesh stays `ErrUnsupported`, staged to a later increment, whether open or closed. The mesh publishes a zero occupied-volume proof (`symDiffOK == true`) for a CLOSED body whose every vertex carries a proven bound of exactly zero, admitting it to a boolean like any other zero-bound operand; every other stitched body keeps `symDiffOK` false, so no boolean admits it — an open one refusing on Table X's own sheet-boolean rule, a bounded or placed closed one on `boolean.go`'s `requireVolumeProvingPayload` arm. `newBodyGeomBudget` (`docs/clearance-design.md` §2) carries the identical zero-bound `stitchPayload` arm already, which is what lets a stitched solid reach a proven pair relation at all; a bounded or placed stitched solid still reads undecided exactly as it does for any other payload this evaluator has not wired a carrier for |

**Increment 4 depends on neither 2 nor 3, and they do not depend on it.**
Increment 5 depends on increment 2 for the triangle set and topology its own
mesh restates, and on increment 3 for identifying — never yet covering — the
curved and flux-integrated stitched solids its own mesh does not reach; it is
numbered after 3 and 4 only so that no reference to them has to move. Any
order among 3, 4 and 5 is admissible.

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
| T6 | a half-disc revolved a full turn about its diameter, as a surface | a closed sheet: `Kind() == BodySheet`, `Edges(Free())` matches nothing, `Volume()` is `ErrNotSolid`; stitching it alone closes to a solid from T50 onward |
| T7 | a surface-extruded profile and a solid whose boxes meet, with an admitted transversal crossing between them | `Verify` reads `Interfering` with exactly one `DiagSheetSolidCrossing` naming the pair, and no `Interference` or `Clearance` row; `Passed()` is false |
| T8 | the same pair moved until the boxes separate | `Verify` reads `Sound`, with no diagnostic; under `WithClearances()`, a `Clearance` row carries the measured gap, `Exact` |
| T7a | a sheet frame around a smaller solid, boxes meeting but the frame's material never touching it | `Verify` reads `Sound`; under `WithClearances()` a `Clearance` row carries the closed-form gap, `Exact`, and no diagnostic |
| T7b | a sheet nested wholly inside a solid, boxes meeting | `Verify` reads `Sound`; under `WithClearances()` the SAME shape of `Clearance` row as T7a, `Exact`, and no diagnostic |
| T7c | a sheet whose one free-form wall the clearance kernel cannot model, against a solid whose boxes meet | `Verify` keeps `DiagUnsupportedPairSheet`, `Suspect`, message naming the kernel's failure to settle the pair rather than an unsupported sheet operand |
| T7d | two sheets whose boxes meet, in a fixture that would cross if either were a solid | `Verify` keeps `DiagUnsupportedPairSheet`, `Suspect`: neither operand offers a closed boundary to cast against |
| T7e | T7's fixture with the two bodies created in the opposite order | the `DiagSheetSolidCrossing` diagnostic's `Pair.A`/`Pair.B` follow `Document.Bodies()` order, not "sheet first" |
| T9 | a sheet handed to `Union`, `Fillet`, `Chamfer` and `Shell` | each is `ErrUnsupported`; the receiver and every operand stay live, and `Document.Bodies()` is unchanged |
| T10 | a sheet tessellated | a prism sheet tessellates and exports through the manifold-with-boundary audit (§10, Table D; `tessellate_sheet_test.go`); a revolve sheet does the same, over every Table W row and the design's T6 (`tessellate_revolve_sheet_test.go`); a loft sheet still refuses with `ErrUnsupported`, deferred to a later increment |
| T11 | a three-face assembly welded into a Möbius orientation | `Stitch` is `ErrDegenerate` (R7), and the document is unchanged |
| T12 | `Body.Patch` on a non-planar four-edge chain | `ErrUnsupported` (R6); and on a bounded chain that carries no shared level token — a revolve seam, or any other chain no builder stamped one onto — `ErrUnsupported` on the same row. A bounded chain that DOES share one level token is T26's own admission, gate 3's second arm |
| T13 | every Table R row | the stated sentinel, with `errors.Is` holding, and no document change |
| T14 | a wall extruded 5 inches `Along` (a nonzero-bound top rim, `document.go`'s unit-conversion rounding) against a patch built directly at the identical millimetre level (zero bound) | the stitch returns a **sheet**, not an error; `Edges(Free()).Exactly(12)` resolves — none of the four bit-identical rim/patch corner pairs join, because Table J's J5 refuses a nonzero-bound held value even where J4's coordinates match |
| T15 | `Body.Patch` filling a `Document.Patch` sheet's own sole 4-edge boundary | two faces; `Area` doubles to 12000 mm², `Exact`; `Edges(Free())` matches nothing; the new face's normal is the exact negation of the original face's |
| T16 | `Body.Patch` filling a single closed circular rim (a `Document.Patch` circle's own sole free edge) | which the original one-chain wording would have refused (§5.2); two faces, `Edges(Free())` matches nothing |
| T17 | `Body.Patch` capping BOTH rims of a surface-extruded tube's `Edges(Free()).Exactly(8)` in one call | this contract's own flagship case; two new 6000 mm² faces, `Exact`; `Edges(Free())` matches nothing |
| T18 | `Body.Patch` closing a stitched-but-still-open sheet's own last free edge | `Kind() == BodySheet` still, `IsSolid() == false`; `Edges(Free())` matches nothing |
| T19 | `Body.Patch` filling a `rectWithHoleSketch` `Document.Patch` sheet's own HOLE loop (`Edges(Free(), Concave())`) | two faces; the hole's own edge now bounds both; the four outer edges stay free; every face's normal equals the holed face's own, unlike T15 — a hole loop is walked clockwise, so the opposite sense the patch takes reads counter-clockwise and lands on the SAME side; `Area` returns to the rectangle's full 6000 mm², `Approximate` |
| T20 | a 100×60 rectangle surface-extruded 10 mm `Along`, verified with `WithPullDirection(r3.NewVec(1, 0, 1))` — the solid counterpart's own tilted-pull fixture | `Coverage == CoverageComplete`; exactly one face listed, and it is the −X wall (`f.NormalAt(p)` equals `r3.NewVec(-1, 0, 0)` exactly, zero `Bound`), never the bottom cap the solid also lists; `Assessment == AssessmentViolated`; `Status == Violating`; exactly one `DiagUndercut` with `Survey == SurveyUndercut`; `Passed() == false`; `Region == nil` and `Volume()` is still `ErrNotSolid` |
| T21 | the same sheet verified with `WithPullDirection(r3.NewVec(0, 0, 1))` — every wall exactly perpendicular | `Coverage == CoverageComplete`; `Faces` non-nil and empty, the proven all-clear; `Assessment == AssessmentMet`; `Status == Sound`; no diagnostics; the whole report `Sound` and `Passed()` |
| T22 | a surface-result `Loft` sheet (`ValidityValid`), any pull requested | `Coverage == CoverageUnavailable`; exactly one `DiagUnsupportedSurveyPayload` naming `SurveyUndercut` and naming `loftPayload` in its message; `Assessment == AssessmentUndecided`; `Status == Suspect` |
| T23 | the same surface-extruded plate as T20/T21, verified with `WithMinWallThickness`, `WithPullDirection(r3.NewVec(0, 0, 1))` and `WithConcaveRadius` together | `Wall.Outcome` and `ConcaveRadius.Outcome` both `ScalarUnavailable`, each with its own `DiagSurveyPrerequisite` naming its own survey and no mention of "pull" in either message; `Undercut` carries no `DiagSurveyPrerequisite` and reads a real `CoverageComplete` with empty `Faces`; `br.Diagnostics` has length 2, not 3; `Status == Suspect` on the wall and radius refusals alone |
| T24 | a surface-result `Revolve` sheet whose fourth leg is undecided (`ValidityUndecided`), any pull requested | `Coverage == CoverageUnavailable` with one `DiagSurveyPrerequisite` whose message names the undecided-validity cause, never a sheet-material cause |
| T25 | `publishUndercutResult` driven directly (internal) on a `BodySheet` body with `ValidityValid` and a populated `undercutOutcome` | the survey outcome is published as given, not replaced by the prerequisite refusal |
| T26 | `Body.Patch` capping BOTH rims of a `Symmetric` (both ends unit-converted, so both bounded) surface-extruded wall in ONE call — the LEVEL certificate's own flagship | no error, where the same chain carried no level token would be R6; `Kind() == BodySheet`; 6 faces; `Edges(Free())` matches nothing; `Area` is `Approximate` with `Bound.Base() > 0` |
| T27 | gate 3's level arm admits a chain whose vertices and edges all carry one shared, non-zero `levelID`, and refuses the same shape carrying no level token at all | direct unit coverage of `provePatchChainPlane`'s two arms (`patch_body_internal_test.go`), since a real body's own free-edge chain always carries a level token when its build minted one |
| T28 | two vertices whose held coordinates and held bounds are bit-identical, minted under two different `levelID`s | `Body.Patch`'s own public seam cannot construct one chain spanning two independently built bodies — a chain requires two edges to SHARE a vertex pointer, which only one evaluator's own build or a zero-bound weld creates, and Table J refuses a zero-bound weld of a bounded pair — so this is pinned directly against `provePatchChainPlane`: still `ErrUnsupported`, proving the certificate is an identity check, never a tolerance |
| T29 | `buildPatchFace` over a chain the level arm admitted | the new face's `axialDelta`/`hasAxialDelta` carry the chain's own proven axial bound, the same fields a prism cap already publishes |
| T30 | `Body.Patch` capping both rims of a `Distance` (one end recorded, one end unit-converted) surface-extruded wall in one call — the single-bounded-end case | no error: the recorded end's chain passes gate 3's first (exact) arm, the computed end's chain passes the second (level) arm, in the same call |
| T31 | `annularSketch` revolved a full turn as a surface, stitched alone | `Kind() == BodySolid`; `Volume` 2000π mm³, `Approximate`, enclosing the analytic value within its own `Bound`; `Area` 800π mm², bit-identical to the equivalent solid revolve's; `Centroid` (5, 0, 0), `Approximate`; `Edges(Free())` matches nothing |
| T32 | the same profile built as a solid `Revolve` with no option, and separately stitched from the surface-result build | the two bodies' `Volume` and `Centroid` agree within the two independently-composed bounds — the independent-producer cross-check, which proves the flux arms against the unrelated revolve engine rather than against its own arithmetic |
| T33 | T31's fixture swept across a family of radii and heights | `\|published − analytic\| <= Bound` on `Volume` and each `Centroid` coordinate for every member |
| T34 | a surface-extruded tube capped on both rims by `Body.Patch` in one call, stitched (`TestStitchPatchCappedTubeClosesToASolid`) | `Kind() == BodySolid`; `Volume` 1000π mm³, `Area` 400π mm², `Centroid` (0, 0, 5), all `Approximate`; `Edges(Free())` matches nothing — Rule P admits: the receiver's own `prismPayload` proves simple under Rule S, and each of the two chains is the receiver's own complete end rim under its own shared level id |
| T35 | a hand-built two-body face set, driven directly at `stitchRuleSAdmits` (internal — Table J's own J5 admits a free `Line3` edge alone, so no two curved rims from different features ever weld into a closed set through the public seam to reach this gate at all) | `stitchRuleSAdmits` reports `false` |
| T36 | `offAxisSemicircleSketch` revolved a full turn as a surface, stitched alone | closes to a solid from T53 onward |
| T37 | a hand-built face carrying a `NURBSSurface`, driven directly at `stitchFaceFluxAndMoment` (internal — `Body.Patch` itself refuses any chain carrying a `NURBSCurve` edge, so a free-form-walled sheet can never be closed through the public seam at all) | `ErrUnsupported` (R8), permanently: `NURBSSurface` exports no control net to integrate |
| T38 | a hand-built face carrying a nonzero `normalBound`, driven directly at `stitchFaceFluxAndMoment` (internal — a fillet or chamfer's `Unstitch`-then-`Stitch` round trip never re-closes, §6.5's own "no further" limit for any non-all-planar body, so no public fixture reaches this gate) | `ErrUnsupported` (R8) |
| T39 | `annularSketch` revolved a full turn as a surface | `Verify` reads `Validity.Outcome == ValidityValid` with no `Validity.Diagnostics`, admitted by construction: `payloadProvesSimple`'s `revolvePayload` arm holds because the sweep is exactly one full turn and the radial minimum is proven clear of the axis |
| T40 | `annularSketch` revolved a quarter turn as a surface | `Verify` stays `Validity.Outcome == ValidityUndecided` with one `DiagUndecidedValidity`: a partial turn earns no construction admission |
| T41 | two INDEPENDENT `Extrude` calls building the identical profile at the identical Symmetric extent, both `WithSurfaceResult()` | first, the premise: a rim vertex of each holds a bit-identical coordinate and an identical non-zero bound; then `Stitch(a, b)`: `Kind() == BodySheet`, and `Edges(Free())` resolves to the full 16 edges, none welded — the proof the CURVE certificate is an identity check, never a tolerance, since each independent build mints its own fresh `curveID` |
| T42 | a Symmetric surface-extruded wall (both rims bounded) on an off-axis, non-origin sketch plane, `Body.Patch`-capped on both rims in one call, then a single-operand `Stitch` | no error; `Kind() == BodySolid`; `Volume` `Approximate` with `Bound.Base() > 0`, and the denoted volume lies inside `[value−bound, value+bound]` — the enclosure assertion a missing `massDelta` charge breaks (§6.4's amendment). Unstitching that solid then re-stitching it closes to a `BodySolid` again, `Volume` matching bit for bit — the CURVE certificate's own copy-path proof, since Table J's J5 could never admit the bounded rim pair on bit-identity alone. Placing ONE of the six unstitched sheets by a motion neither axis-aligned nor centred on the origin, then stitching all six, leaves exactly that sheet's own boundary and its former neighbours' matching copies free (8 of 24 edges), while the remaining five unplaced siblings still weld among themselves — a placement breaks the certificate |
| T43 | a hand-built `bodyPatchPayload` whose receiver's own payload is a `prismPayload` with a nonzero `sectionDelta`, driven directly at `bodyPatchPayloadProvesSimple` (internal — Rule P condition 1: a receiver this evaluator can otherwise prove Rule S-admits, isolated from conditions 2 and 3, which the fixture's chain and receiver otherwise satisfy) | `bodyPatchPayloadProvesSimple` reports `false` |
| T44 | a hand-built `bodyPatchPayload` whose receiver carries two disjoint free-edge loops under the SAME level id and whose one new chain caps only one of them, driven directly at `bodyPatchPayloadProvesSimple` (internal — Rule P condition 2: the receiver's own construction proof does not transfer from a proper subset of its end's own free edges, the annular-rim hazard §6.4 names) | `bodyPatchPayloadProvesSimple` reports `false` |
| T45 | a hand-built `bodyPatchPayload` whose chain's edges all share one level id but one chain VERTEX carries a different one, driven directly at `bodyPatchPayloadProvesSimple` (internal — Rule P condition 3: every vertex, not only every edge, must be proven part of the same recorded plane) | `bodyPatchPayloadProvesSimple` reports `false` |
| T46 | a trapezoid profile clear of the axis (both non-radial sides leaning off it, so revolving sweeps two `Cone` walls of different half-angles, not two `Cylinder` walls), revolved a full turn as a surface, stitched alone | `Kind() == BodySolid`; `Volume` and `Area` match the analytic frustum-shell values, `Approximate`, enclosing the analytic value within `Bound`; `Centroid` matches the analytic value, `Approximate`; `Edges(Free())` matches nothing |
| T47 | the same profile built as a solid `Revolve` with no option, and separately stitched from the surface-result build | the two bodies' `Volume` and `Centroid` agree within the two independently-composed bounds — the independent-producer cross-check, proving the `Cone` arm against the unrelated revolve engine rather than against its own arithmetic |
| T48 | a hand-built `Cone` face whose `Origin` sits away from the apex (`Radius` nonzero), driven directly at the apex derivation (internal — every reachable construction site sets `Radius` literally to `0`, so a nonzero-`Radius` `Cone` never reaches `Stitch` through the public seam) | `ErrUnsupported` (R8): this evaluator has no sound bound for `tan(HalfAngle)`'s own rounding, so it refuses rather than publish an unbounded apex; a literal zero `Radius` still publishes `Origin` as an `Exact` apex |
| T49 | a hand-built `Cone` face whose `HalfAngle` is `0` (an exactly-zero tangent, a degenerate needle) or `NaN` (a non-finite tangent, a malformed value), driven directly at the apex derivation (internal — a reachable `wallCone` always carries a finite `HalfAngle` strictly between `0` and `π/2`, `revolve_axis.go`'s own analytic-walk requirement) | `ErrUnsupported` (R8): the half-angle's tangent is not a positive, finite float |
| T50 | `semicircleSketch` revolved a full turn as a surface (T6's own fixture), stitched alone | `Kind() == BodySolid`; `Volume` (4/3)π·125 mm³, `Area` 100π mm², `Centroid` (5, 0, 0), all `Approximate`, each enclosing its analytic value within `Bound`; `Edges(Free())` matches nothing — this replaces `TestStitchClosedCurvedSheetIsUnsupported`, whose whole subject (T6's second half) this row retires |
| T51 | the same profile built as a solid `Revolve` with no option, and separately stitched from the surface-result build | the two bodies' `Volume` and `Centroid` agree within the two independently-composed bounds — the independent-producer cross-check, proving the `Sphere` arm against the unrelated revolve engine rather than against its own arithmetic |
| T52 | T50's fixture swept across a family of radii and off-origin diameters (the generating semicircle's own centre moved along the axis) | `\|published − analytic\| <= Bound` on `Volume` and each `Centroid` coordinate for every member |
| T53 | `offAxisSemicircleSketch` revolved a full turn as a surface, stitched alone — one `Cylinder` wall and one `Torus` wall spanning exactly the tube's own outer half, `φ ∈ [−π/2, π/2]` | `Kind() == BodySolid`; `Volume` and `Area`, `Approximate`, each enclosing its own hand-derived (Pappus, over the 2D half-disc profile) analytic value within `Bound`; `Centroid` (5, 0, 0), `Approximate`; `Edges(Free())` matches nothing — this replaces `TestStitchTorusFaceStaysUnsupported`, whose whole subject (T36) this row retires |
| T54 | the same profile built as a solid `Revolve` with no option, and separately stitched from the surface-result build | the two bodies' `Volume` agree within the two independently-composed bounds — the independent-producer cross-check, proving the `Torus` arm against the unrelated revolve engine rather than against its own arithmetic. The matching `Centroid` comparison is real but WEAK for this fixture: the direct revolve engine's own centroid bound comes out about 80 mm wide for this 20 mm-wide solid — clear of the roughly 460 mm the `Sphere` arm's own cross-check hits for a profile touching the axis at both poles, since this fixture stays off the axis, but still far too wide to be decisive on its own; T53 and T55 carry the tight, decisive centroid proof |
| T55 | T53's fixture swept across a family of `Major`/`Minor` radii and axial centres | `\|published − analytic\| <= Bound` on `Volume` and `Centroid` for every member |
| T56 | a hand-built `Torus` face whose `Axis` is not exactly a signed coordinate vector, or whose `Center` sits off that axis line, driven directly at `stitchFaceFluxAndMoment` (internal — every public fixture revolves about `uAxis`, coordinate-aligned through the origin, so no public fixture reaches this gate) | `ErrUnsupported` (R8): this evaluator has no sound bound for `Major`/`Minor`'s own rounding off a coordinate-aligned axis |
| T57 | a hand-built `Torus` face whose two rims sit at axial offsets from `Center` other than exactly `±Minor` (a narrower window, a wider one, or both rims on the same side), driven directly at `stitchFaceFluxAndMoment` (internal — every reachable revolve wall junction in this tree happens to land on the exact half window, so no public fixture reaches this gate) | `ErrUnsupported` (R8): this evaluator has no sound way to recover a narrower angular window without an unbounded trig computation |
| T58 | T3's stitched box (`stitchBoxSheets`), `Tessellate(0.1 mm)` | exactly 12 triangles and 8 vertices; every vertex bit-equal to one of the box's 8 corners; `len(SourceFaces()) == 12` over exactly 6 distinct faces, 2 triangles each; `Bound()` exactly zero; the tetrahedron sum over the mesh equals 60000 mm³ to the bit; every directed edge occurs once with its reverse once |
| T59 | T58's box `Placed` by a translation far from the origin, then a rotation | `Bound()` is strictly positive and equals the body's own largest `Vertex.Bound()`; 12 triangles unchanged; the mesh's integrated volume encloses 60000 mm³ within the published bound — the placement-delta route to a nonzero per-face bound |
| T60 | T42's bounded-rim stitched solid (`TestStitchClosesABoundedPatchedWallWithChargedVolumeBound`'s fixture: a `Symmetric` surface-extruded wall, `Body.Patch`-capped, welded by the CURVE certificate) at identity, no placement in play | `Bound()` strictly positive and equal to the largest vertex bound; `areaSlack` strictly positive — the independent, class-bound route to a nonzero per-face bound, which T59 alone cannot distinguish from a placement-only charge |
| T61 | T4's displaced-patch stitched sheet (`TestStitchDisplacedPatchStaysASheet`'s fixture, open, all faces planar), `Tessellate(0.1 mm)` | the mesh's free directed edges attribute to exactly the 5 faces the body reports free `Edge`s on (the 4 wall faces and the displaced patch), one boundary chain per face on both sides; `requireSheetVertexLinks` passes; `Volume()` is still `ErrNotSolid`; `Union` with a plain solid block refuses |
| T62 | each of T46's frustum, T50's ball and T53's torus body, stitched | `Tessellate`, `STL` and `OBJ` each return `ErrUnsupported`; the message names the face this evaluator has no chording arm for, not the payload class; the body's analytic `Volume`/`Centroid` still read unchanged afterwards |
| T63 | T58's box tessellated twice at two different tolerances | equal vertex order, triangle order and source-face order; byte-identical STL and byte-identical OBJ; mutating the returned `Vertices()`/`Triangles()` slices changes nothing on a later call |
| T64 | `Union` of T58's box with a plain `Extrude` block | this fixture's own outcome changes from T71 onward, once the occupied-volume proof lifts for a CLOSED, all-planar, zero-vertex-bound stitched solid; a stitched operand this proof does not cover (placed, or certificate-welded) keeps this row's own `ErrUnsupported`, the volume-proof refusal (`operandSymDiff`'s wording, via `requireVolumeProvingPayload`'s own `stitchPayload` arm), not the payload-class one — T73, T74 |
| T65 | T58's box and a plain solid block 3 mm beyond its own +X wall, `Verify(WithClearances())` | `Sound`; exactly one `Clearance` row; its proven interval encloses 3 mm; `Exact`. Replaces `TestStitchSolidDoesNotYetReachAPairRelation`'s Sound-side premise |
| T66 | T58's box and a plain block straddling its own +X wall (a true, non-nesting overlap) | the clearance kernel itself proves `pairOverlapping` (`TestClearancePairProvesStitchedSolidOverlapDespiteTheBooleanRefusal`, internal) but the public report still reads `Suspect` with no `Interference` row: this exact fixture's block shares the box's own y-range and z = 0 base plane, so even once the occupied-volume proof lifts (T72), the mesh boolean's own separate, pre-existing coplanar-contact gate refuses it — `DiagUnsupportedPairContact` from T72 onward, no longer `DiagUnsupportedPairPayload` |
| T67 | a small stitched box wholly inside a large plain block, boxes meeting | the solid-solid path's own strict-containment certificate: `Interfering`, exactly one `Interference` row reusing the contained box's own `Volume`, no `Clearance` row — reached through the clearance kernel's own carrier model alone, with neither operand ever tessellated |
| T68 | T42's own certificate-welded stitched solid (nonzero vertex bound at identity) against a plain block with a real box-proven gap, `Verify(WithClearances())` | no carrier model; `Suspect` with `DiagUndecidedClearance`, never a `Clearance` row. Shown-to-fail: deleting the zero-bound gate lets this pair read `Sound` with a falsely `Exact` `Clearance` row that does not account for the certificate's own residual bound |
| T69 | T58's box `Placed` under a non-identity rigid motion, same pairing as T68 | identical outcome, reached by the other route: the placement's own `rigidRoundAllow` widens every vertex bound rather than a certificate weld's own class bound — neither T68 nor T69 alone would be trusted to test the gate itself rather than one particular cause of it |
| T70 | a hand-built stitched face set whose recorded triangle list repeats a directed edge, driven directly at `tessellateStitch` (internal — `checkStitchClosure` already catches this at build time, so no public fixture reaches this gate) | the audit refuses and no partial mesh is returned |
| T71 | `Union` of T58's box with a plain, disjoint `Extrude` block (`TestStitchSolidUnionComposesTheCorrectVolume`) | succeeds: `Exact` volume, the exact sum of both operands' own volumes, zero bound, two lumps — the mesh-boolean analog of `TestUnionDisjointCubes` over a stitched operand; `requireVolumeProvingPayload`'s `stitchPayload` arm no longer refuses a CLOSED, all-planar, zero-vertex-bound stitched operand. Shown-to-fail: reverting `tessellateStitch`'s `symDiffOK` publication to always `false` turns this `NoError` red with the volume-proof `ErrUnsupported` again |
| T72 | T58's box and a plain block crossing it cleanly — no operand face landing on the other operand's own face plane, unlike T66's own fixture (`TestStitchOverlappingSolidReportsRealInterference`) | `Interfering`; exactly one `Interference` row whose proven interval encloses the analytic overlap volume; no `Clearance` row — this replaces `TestStitchOverlappingSolidStaysUndecided`, whose whole subject (a stitched solid's overlap ever reaching a real `Interference` row) this row retires |
| T73 | T58's box `Placed` under a non-identity rigid motion, then `Union`ed with a plain block (`TestStitchPlacedSolidUnionStillRefusesOnTheVolumeProof`) | still `ErrUnsupported`, the volume-proof refusal: a placement's own `rigidRoundAllow` widens every vertex bound, so `stitchZeroVertexBound` no longer holds |
| T74 | T42's own certificate-welded stitched solid (nonzero vertex bound at identity), `Union`ed with a plain block (`TestStitchCertificateWeldedSolidUnionStillRefusesOnTheVolumeProof`) | still `ErrUnsupported`, the volume-proof refusal, reached by the CERTIFICATE-WELD route rather than T73's placement route — neither test alone would prove the gate reads the vertex bound itself rather than one particular cause of it |
| T75 | a hand-built planar face bounded by a 10x10 outer square and a 2x2 hole square, driven directly at `triangulateStitchFaces` (internal — `TestTriangulateStitchFacesTilesAHoledFaceExactly`; every public stitched fixture in this tree happens to be hole-free) | every returned triangle has strictly positive area and attributes to the one face handed in; the triangles' own summed area is bit-exactly the analytic 96 mm² outer-minus-hole region — a gap or an overlap would miss it in one direction or the other. Shown-to-fail: dropping hole-bridging (`triangulate2DContext`'s own `holes` slice) turns the summed area into the outer square's whole 100 mm², missing the assertion |
| T76 | two axis-aligned boxes (T1's box and a second one offset to x∈[100,200], y∈[60,120], z∈[10,20]), each three surface-extruded-wall-plus-two-`Body.Patch`-cap sheets, sharing exactly one corner vertex — (100,60,10) — with no edge joining them, all six sheets `Stitch`ed in one call (`TestStitchRefusesTwoBoxesPinchedAtOneVertex`) | `ErrDegenerate` (R7); the document and every operand are unchanged |
| T77 | the same two boxes as T76 translated apart so no vertex is shared, `Stitch`ed in one call (`TestStitchTwoDisjointBoxesFormATwoLumpSolid`) | `Kind() == BodySolid`; 16 vertices, 2 lumps, `Volume` `Exact` at 120000 mm³ — the narrowing T76 proves is a pinch refusal, not a blanket one |
| T78 | T76's two boxes with both top patches omitted, leaving an open, all-planar assembly that still shares the one pinched corner vertex (`TestStitchRefusesOpenAssemblyPinchedAtOneVertex`) | `ErrDegenerate` (R7) even though free edges remain — the vertex-link audit refuses the open arm exactly as it does the closed one (§6.3's amended Table C, §6.4) |
| T80 | a chamfered rectangular plate's own flat band patch (`normalBound` nonzero under a far placement), unstitched at identity | the unstitched face's `NormalAt` published bound at the patch's own held corner still covers the SOURCE face's own `normalBound` — asserted against that face's own value, never a literal, since the bound is architecture-dependent. Shown-to-fail: reverting `copyFaceUnderContext`'s carried `normalBound` to its old zero value turns this red, the published bound falling under the source's own `normalBound` |
| T81 | a pin extruded `ToFace` against a plate's own cap, where the plate's own extrude depth is stated in inches (a genuine unit-conversion rounding, so the cap's own `axialDelta` is nonzero) — once against the un-unstitched plate, once against that SAME plate unstitched into its six free sheets first | the two pins' own `Bounds` agree exactly, `Exactness` and `Bound` alike — the equality is the assertion, stronger than either number alone. Shown-to-fail: reverting `copyFaceUnderContext`'s carried `axialDelta`/`hasAxialDelta` to their old zero value turns the unstitched-cap pin's own `Bounds` `Exact` at a zero bound while the un-unstitched pin's stays `Approximate`, breaking the equality |
| T82 | the same far-placed chamfered plate's own flat band patch (T80's fixture), unstitched at identity then placed again under a second, non-identity motion | `ErrUnsupported`: `normalBound` is dimensionless and carries no term to absorb a placement's own length-scale rounding, so this evaluator refuses the placed copy rather than invent one. Shown-to-fail: removing `copyFaceUnderContext`'s placed-nonzero-`normalBound` refusal turns this `NoError`, publishing a placed copy whose `normalBound` no longer covers the tag's true departure |
| T90 | a hand-built revolve payload for a dipped shaft profile (axis exact, an admitted nonzero radial-minimum interval charged by hand), driven directly at `evalRevolveContext` (internal — `TestRevolveAxisBandVolumeContainsEnclosedVolume`) | `Volume` is `Approximate` with a strictly positive `Bound`, and its interval contains the volume the body's own (snapped-topology) cylindrical faces enclose. Shown-to-fail: dropping `revolveAxisAdmitVolumeCharge`'s own charge turns the containment assertion red |
| T91 | the same shape as T90, a quarter sweep instead of a full turn, driven directly at `evalRevolveContext` (internal — `TestRevolveAxisBandPartialSweepAreaContainsEnclosedArea`) | each of the two cap faces' `Area` is `Approximate` with a strictly positive `Bound`, and its interval contains the area the cap's own snapped loop encloses — the sharper of the two measured consequences, since a cap's loop and its published area come from two different profiles (snapped, unsnapped) whenever the charge is nonzero. Shown-to-fail: dropping the cap `areaBound`'s own charge turns the containment assertion red |
| T92 | `resolveAxisSide` against an axis-aligned, origin-anchored axis and a profile whose recorded near edge sits exactly on the axis (internal — `TestRevolveAxisBandAllowanceZeroForExactAxis`) | the returned `axisFrame.radialAdmitAllow` is exactly zero, and so are `revolveAxisAdmitBandCharge`/`revolveAxisAdmitVolumeCharge` off it — the charge never widens the common, exact-arithmetic case. Shown-to-fail: charging a spurious nonzero allowance regardless of admission turns this assertion red |
| T93 | `resolveAxisSide` against the SAME exact axis with the near edge dipped 1e-7 mm below it (internal — `TestRevolveAxisBandRefusesProvenNegativeRadialMinimumUnderExactAxis`; the fixture the investigation measured admitted under the pre-repair tolerance) | `ErrDegenerate`, never an admission with a widened bound — the strict half of the repair: an admission gate may not rest on a tolerance once the radial bound is proven zero. Shown-to-fail: removing the strict refusal turns this assertion red |
| T94 | `resolveAxisSide` against an axis anchored far from the origin (`aV = 1e10`) on a direction whose components are not exactly representable, mirroring `TestRevolvePayloadProvesSimpleChargesTheAxisOffsetShift`'s own fixture one level up (internal — `TestRevolveAxisBandChargesTheOffsetSubtraction`) | the axis-offset subtraction's own rounding is charged into the bound the "ambiguous side" refusal states, which is never zero. Shown-to-fail: reverting to a bare `rlo -= roff` drops the charge and the fixture wrongly reaches an unrefused, uncharged admission |
| T95 | a real sketch-solver-resolved shaft profile whose near edge sits AT (not below) `tiltedAxis` (`revolve_bounds_test.go`'s own anchor/direction), built end to end through `Document.Revolve` (internal — `TestRevolveAxisBandRealGeometryChargedPathVolumeContainment`) | `axisFrame.radialAdmitAllow` is confirmed positive (the charged path, not the strict one, fires on real geometry — `resolveAxisSide`'s own `planeDotDecompositionRoundAllow` charge on its scan arithmetic is what makes this reading agree across architectures instead of one proving the same true-zero radial minimum negative); `Volume`'s published interval contains the plain cylinder its snapped topology encloses; rebuilding the SAME payload with `radialAdmitAllow` zeroed publishes a bound smaller by less than one part in a million, establishing rather than assuming that this real fixture's own charge is swamped by the build's other analytic-rounding terms — T90/T91 are what make the charge itself load-bearing, at a magnitude no real sketch-resolved fixture in this tree reaches. Shown-to-fail: moving the near edge off the axis turns the `radialAdmitAllow` positivity assertion red; comparing the no-charge bound against an arbitrary literal instead of the real one turns the relative-magnitude assertion red |
| T110 | `semicircleSketch`'s half-disc (radius 5, centred at u=5) revolved a full turn about its own diameter, `Body.Centroid` (`TestRevolveSemicircleFullTurnCentroidBoundTightens`) | the published bound is a tiny fraction of the body's own 10 mm diameter (never a literal, since `circularSecondMomentInterval`'s bound is architecture-dependent) and still encloses the true (5, 0, 0) centroid — `moments_circular.go`'s new `circularSecondMomentInterval` replaces the old bound of exactly 460.625 mm, wide enough to admit any centroid inside the body. Shown-to-fail: forcing `circularSecondMomentInterval` to answer `ok == false` reproduces the old 460.625 mm envelope and turns the tightness assertion red |
| T111 | `ProfileRecord.SecondMoments()` on `semicircleSketch`'s own profile record (`TestSecondMomentsSemicircleBoundTightens`) | each of `UU`/`UV`/`VV`'s bound is a tiny fraction of its own value, never hundreds of times it (`UV`'s bound was 38385.42 mm⁴ against a 416.67 mm⁴ value before this fix). Shown-to-fail: forcing `circularSecondMomentInterval` to answer `ok == false` reproduces the old three magnitudes and turns every tightness assertion red |
| T112 | an all-straight-line profile's `SecondMoments` (`TestSecondMomentsRectangle`, unchanged by this fix — a `LineSeg`-only region never reaches `addCircular`) | `Exact` with a zero bound on all three components, exactly as before: the fix touches only the circular-segment arm, never the exact-rational line path |
| T113 | the 5 mm-radius ball (`semicircleSketch` revolved a full turn), `Body.Volume` (`TestRevolveSemicircleVolumeBoundUnaffected`) | the bound stays a tiny fraction of the volume, unmoved by this fix: `Volume` is Pappus's first theorem over the region's first moments alone (`axisMoments`'s `q`), which `circularSecondMomentInterval` never touches. Verified by hand: disabling `circularSecondMomentInterval` (forcing `ok == false`) leaves this reading's value and bound bit-for-bit unchanged while T110/T111's tightness assertions go red |
| T114 | `semicircleSketch`'s half-disc revolved a quarter turn (90°) about its own diameter, `Body.Centroid` (`TestRevolveSemicirclePartialSweepCentroidBoundTightens`) | the published bound is a tiny fraction of the body's own diameter and still encloses the centroid the half-disc's own analytic `q`/`mzr`/`mrr` (independently derived: `q = ⅔r³`, `mzr = ⅔r⁴`, `mrr = πr⁴/8`) predict through the same partial-sweep formula `TestRevolvePartialCentroidBoundTightens` uses — replacing the old 638.75 mm envelope, the quarter-turn case the investigation measured as the worse of the two. Shown-to-fail: forcing `circularSecondMomentInterval` to answer `ok == false` reproduces the old 638.75 mm envelope and turns the tightness assertion red |
| T120 | an inner box (`stitchBoxSheetsAtOffset`, x∈[20,40], y∈[20,40], z∈[2,3]) wholly inside T1's own outer box, all six sheets `Stitch`ed in one call (`TestStitchRefusesNestedBoxes`) | `ErrUnsupported` (R20); the document and every operand are unchanged — without the separation check this fixture publishes 60400 mm³ `Exact` against a true 60000 mm³, the point-set volume the nesting double-counts |
| T121 | the same shape as T120 with the inner box shrunk 1 mm off every wall of the outer box (x∈[1,99], y∈[1,59], z∈[1,9] inside T1's box), all six sheets `Stitch`ed in one call (`TestStitchRefusesNestedBoxesWithAGap`) | `ErrUnsupported` (R20); without the separation check this fixture publishes 105472 mm³ `Exact` against a true 60000 mm³ — wrong by more than three quarters, nowhere near a rounding miss |
| T122 | two hand-built, bit-identical (fully overlapping) zero-bound square faces, each its own single-face lump, driven directly at `tessellateStitch` on a hand-built `stitchPayload` — bypassing `Stitch`'s own public seam, since T120's own lump-separation gate refuses this shape before a body ever reaches tessellation (internal — `TestTessellateStitchClearsSymDiffForOverlappingLumps`) | `symDiffOK` is `false` even though `stitchZeroVertexBound` holds: `tessellateStitch`'s own lump-separation reading refuses independently of `evalStitchContext`'s, so the zero occupied-volume claim stays sound even if that earlier gate were ever loosened or bypassed |

`.github/test-shards.txt` gains a row for every root-package test each
increment adds, and `go test . -run '^TestCIWorkflowRaceShardsCoverEveryPackage$'`
runs before any push that changes a root test name.
