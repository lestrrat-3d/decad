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
- `docs/sketch-seam-design.md` owns profile authentication and recording;
- `docs/surface-intersection-design.md` owns `Trim`, `Extend` and `Split` over a
  pair whose two sweeps share one generator — the class §1.2's own rows name.

Ten tables are normative:

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
| **G** | what a chain-fed feature builds, mints and orients | §13.4 |
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
- **`Thicken`** — building a solid from an admitted sheet's recorded generator
  and a stated wall thickness (§16);
- **`Offset`** — a second sheet at a stated normal distance from an admitted
  sheet, the source left live (§17);
- **`ExtrudeChain` / `RevolveChain`** — a ribbon from one open sketch curve, and
  an uncapped shell from an open profile revolved (§13);
- what every reading, every `Verify` question and every export says about a
  sheet body (§8, §9, §10).

### 1.2 What this design names and stages

Each command below has an explicit reach boundary. Table D says which
increment takes it up; §11 says what a caller gets until then.

| Command | Why it is not here yet |
|---|---|
| Thicken | §16 admits a recorded planar patch and a prism sheet from a closed section. Other sheet families refuse until their own offset and closure proofs land. |
| Trim, Extend | `docs/surface-intersection-design.md` owns both. A pair whose two sweeps share one generator — two prisms along one direction, two revolves about one axis — meets along the sweep of a 2D crossing `sketch` certifies, so its topology is decided by a flag and nothing is fitted; that document's §2 states the exact predicate and §4 what it excludes. A pair sharing no generator meets along a space curve `CurveSegment` has no variant for, and stays refused on `docs/api-design.md` §2.1's own reasoning. Table D row 8. |
| A sheet operand in `Union` / `Cut` / `Intersect` (Fusion's Split Body) | The intent lands as `Document.Split` in `docs/surface-intersection-design.md` §8, one body in and several out, over the same shared-generator class. The three booleans keep refusing a sheet operand permanently (Table X): each owes its caller ONE body, and a sheet claims no material to union, cut or intersect with. Table D row 8. |
| `SweepChain` | `docs/sweep-design.md` §15 owns it and states its pairing rule: a composite sweep's join pairs by recorded-segment index, which a `ChainRecord` states exactly as a `LoopRecord` does, and a one-span path has no join to pair at all. That document's §15.6 stages the build. Table D rows 12 and 13. |
| `LoftChain` | `docs/loft-design.md` §16 owns it and states its pairing rule: Table P's segment-count and same-kind rows survive an open walk verbatim, the alignment offset is forced to `0`, and the walk direction `sketch` publishes replaces the winding P6 names. What a closed shell also supplied is the positive side, and §16.2's exactly-parallel plane gate replaces it. That document's §16.6 stages the build. Table D row 14. |

### 1.3 What this design refuses permanently

**A tolerant stitch.** Fusion stitches surfaces within a tolerance and produces
tolerant topology — `BRepEdge.isTolerant` is that kernel admitting in public
that its geometry does not meet (`docs/api-design.md` §2.1). decad joins two
free edges only where coincidence is **proven** (Table J), and leaves every
other edge free. A caller reads the residual free edges to see what did not
join. There is no gap parameter to widen, in this design or a later one.

**A fitted surface through a boundary — Fusion's Ruled and Boundary Fill.**
Both name a face whose interior no recorded entity generates. A boundary states
where a face ends and says nothing about what lies between, so the surface
through it is a choice of blending function — a fit — and
`docs/api-design.md` §2.1 is why that is refused rather than staged: every
topology decision over a fitted surface is a floating-point sign test on an
approximation, and a flipped sign gives a nonsensical answer rather than a
slightly wrong one.

**The rule decad builds by is not "mint no geometry".** It mints a section
offset's lines and arcs (§16.2, §17.2) and a cap blend's cone patches
(`docs/modify-reach-design.md` §8.3). The rule is a conjunction, and BOTH halves
must hold: every generated coordinate has a closed form it is proven equal to,
**and** every reading taken over it has a closed form too. A geometry meeting
one half and not the other is refused, so passing the first half is never the
admission.

A boundary-interpolating patch fails both. Its coordinates come from a blend
nobody stated, and its area integrand is the square root of a quartic even in
the simplest ruled case — the patch between two straight edges that are not
coplanar — so §8's rule that a sheet's `Area` is its faces' areas from the
engine that already computes each one has no engine to name. The tree already
shows this: `bounds_wall_area_internal_test.go`'s `convergedRuledArea` reaches a
ruled patch's area only by sweeping a numerical integral at increasing
resolutions until it settles, and it is used only to VALIDATE a bound, never to
publish a reading. A quantity the repository can reach no other way is a
quantity with no closed form here.
`docs/spline-design.md` cannot state the missing rule either: its §2 admits
whole recorded entities alone and mints no curve, and its §7 fixes both
free-form surface variants as the exact extrusion or revolution of a recorded
curve — a one-parameter rigid motion that GENERATES the surface. A boundary
names no such motion (`docs/spline-design.md` §7.1).

**What of the two intents a caller can already reach**, so that the refusal
costs no move: a closed PLANAR boundary is `Document.Patch` or `Body.Patch`
(§5); a ruled wall between two recorded closed sections is `Loft` under
`WithSurfaceResult` (Table W), whose wall is flat triangles between proven
stations rather than a fitted surface; the same between two open chains is
`LoftChain`, whose pairing rule `docs/loft-design.md` §16 states and whose
build Table D row 14 takes up; and extending a sheet from its own rim is `Extend`
(`docs/surface-intersection-design.md`). What remains under the two Fusion
names once those are subtracted is exactly the fitted non-planar patch, which
is what this paragraph refuses. `Body.Patch`'s own non-planar chain is the same
refusal reached from the other side, and R6 carries it.

**What would have to exist**, recorded so nobody re-derives it: a surface the
boundary DETERMINES rather than one a blend picks, with closed forms for its
coordinates, its normal, its area and its self-intersection test.

**This is NOT an upstream ask, and filing one would go to the wrong place.**
Every ask in `docs/spline-design.md` §9 is an ask to `sketch`, and `sketch` is a
2D constraint engine that publishes no surface at all — no change to it reaches
either command. The change that would unblock these two is a new exact surface
kernel inside decad, which is a different engine rather than an increment of
this one.

### 1.4 Reverse Normal — named, and staged for no increment

**No shipped operation's result depends on which side of a sheet the caller
meant while leaving the caller no way to say so.** Reversing a sheet's
published normals and co-edge senses therefore closes no gap, and no increment
takes it up. Every operation that could read a side either reads none, or takes
the side as an argument:

- `Thicken` takes `ThickenSide` and changes no source face (§16.1). §16.4 owns
  why the side option and the result skin's outward normal are different
  questions.
- `Trim` takes `TrimSide`, which names which pieces of the receiver survive the
  tool's cut rather than a side of the receiver's normal; `Extend` names the
  end to extend through the edge-selector vocabulary; `Split` returns every
  piece and reads no side at all (`docs/surface-intersection-design.md` §8).
- `Stitch` derives orientation combinatorially across welded edges and then
  fixes the global sign from the assembly's own signed volume or flux, so it
  accepts no side and needs none (§6.3, §6.4). `Unstitch` carries each face's
  orientation forward unchanged (§6.5).
- `Union`, `Cut` and `Intersect` refuse a sheet because it claims no material
  to combine, which is a statement about volume and not about orientation
  (Table X).
- The undercut survey and `Face.NormalAt` read the orientation §2.3 decides at
  build, and that orientation is a determined consequence of the profile
  winding and sweep direction the caller already chose (§4.2).
- A sheet-against-solid `Clearance` row states the distance to the solid's
  boundary without asserting which side the sheet is on (§9.3), so no side a
  caller might mean changes the published answer.
- `STL` and `OBJ` write the facet winding the build decided, which §10's audits
  read; neither format's options carry a side.

**What would make it necessary**: an operation whose result depends on the
sheet's side and that cannot take that side as an argument. `Cut(solid, sheet)`
keeping the piece on the sheet's negative side would be one, and
`docs/surface-intersection-design.md` §10 rejects it for exactly that reason,
giving the Split Body intent to `Document.Split` instead. Until such an
operation is proposed, this entry names an operation rather than a gap. §16.4
states what a Reverse Normal would have to do if one ever is.

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
| `BodySheet` | `false` | a sheet, by construction — no defect is implied | read its boundary quantities; `Stitch` or `Thicken` it to reach a solid (§6, §16) |
| `BodySheet` | `true` | never produced | — |

The fourth row is structural, not a convention: a sheet encloses no region, so
no evaluator may prove one a solid.

**Closure is a property of the boundary; solidity is a claim about material.**
For an existing closed sheet, §6's `Stitch` makes that claim. So a **closed
sheet** is an ordinary state, not a contradiction: a full revolution built
as a surface closes and stays `BodySheet` (§4.1), because nothing has yet claimed material
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
  This includes a closed sheet built by a full surface-result revolution or
  by patching its last free rim; closure alone makes no material claim.
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
func (d *Document) Patch(ctx context.Context, s *sketch.Sketch, p *sketch.Profile) (*Body, error)

// A planar fill of a closed chain of the receiver's free edges.
func (b *Body) Patch(ctx context.Context, sel EdgeSelector) (*Body, error)

// Joining sheets, and taking a body apart.
func Stitch(ctx context.Context, bodies ...*Body) (*Body, error)
func (b *Body) Unstitch(ctx context.Context) ([]*Body, error)

// Turn an admitted sheet into a solid (§16).
func (b *Body) Thicken(ctx context.Context, thickness units.Value, opts ...ThickenOption) (*Body, error)
func WithThickenSide(side ThickenSide) ThickenOption

// A second sheet at a stated normal distance from an admitted sheet (§17).
func (b *Body) Offset(ctx context.Context, distance units.Value, opts ...OffsetOption) (*Body, error)
func WithOffsetSide(side OffsetSide) OffsetOption

// A sheet from one open sketch curve. §13 owns all four.
type ChainExtrudeOption interface{ chainExtrudeOption() }
type ChainRevolveOption interface{ chainRevolveOption() }

func (d *Document) ExtrudeChain(s *sketch.Sketch, ch *sketch.Chain, e Extent, opts ...ChainExtrudeOption) (*Body, error)
func (d *Document) RevolveChain(s *sketch.Sketch, ch *sketch.Chain, axis Axis, a AngularExtent, opts ...ChainRevolveOption) (*Body, error)
func (d *Document) SweepChain(ctx context.Context, s *sketch.Sketch, ch *sketch.Chain, path *Path, opts ...SweepOption) (*Body, error)
func (d *Document) LoftChain(ctx context.Context, s0 *sketch.Sketch, c0 *sketch.Chain, s1 *sketch.Sketch, c1 *sketch.Chain, opts ...LoftOption) (*Body, error)

// The free-edge selector predicate.
func Free() EdgePredicate
```

Every one of these that takes a context bounds cancellation in its own
construction and audit paths, returns `ctx.Err()` unchanged before commit, and
leaves the document and every operand unchanged, exactly as
`docs/api-design.md` §8 states for every other operation. `ExtrudeChain` and
`RevolveChain` take none, for the same reason `Extrude` and `Revolve` take
none: each resolves an extent and walks a recorded boundary in bounded work,
with no audit to poll (§13.2).

`Stitch` and `Unstitch` consume their operands and register their results, on
`docs/api-design.md` §6's uniform terms: `Stitch` retires every body handed
to it, `Unstitch` retires its receiver, and a retired body stays readable. No
`*Document` appears in `Stitch`'s signature because a `*Body` carries its
owning document, which is what lets `Union` take the same shape.

`Body.Patch` retires its receiver and registers the filled body.
`Document.Patch` consumes nothing — it builds from a sketch profile, as
`Extrude` does. `Body.Offset` consumes nothing either: it registers its result
and leaves its receiver LIVE, on the non-consuming terms `PlacedCopy` and
`Duplicate` already take, because the offset sheet and the sheet it came from
are what a caller measures, stitches or thickens together (§17.1).

Worked, end to end — a closed box from three sheets:

```go
walls, err := doc.Extrude(s, prof,
    decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
    decad.WithSurfaceResult())                    // 4 walls, 8 free edges
bottom, err := doc.Patch(ctx, s, prof)            // 1 face at z = 0
// topSketch draws the same profile on the z = 10 plane.
top, err := doc.Patch(ctx, topSketch, topProf)    // 1 face at z = 10

box, err := decad.Stitch(ctx, walls, bottom, top) // Kind() == BodySolid
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

`Document.Patch(ctx, s, p)` records `p` through the seam exactly as `Extrude` does
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

`b.Patch(ctx, sel)` resolves `sel` against `b`, requires the result to be a set of
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
   patch needs a fitted free-form surface, which §1.3 refuses permanently, and
   no later arm admits it.

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

**`Stitch` turns an existing closed face assembly into a solid.** A caller who
builds a part face by face reaches a solid here. `Thicken` instead generates a
second skin and rim faces from one admitted sheet (§16).

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

`b.Unstitch(ctx)` returns one single-face sheet body per face of `b`, in
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
call always places under the identity transform (`Unstitch`), so a
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
| R21 | `ExtrudeChain` or `RevolveChain` handed a chain that fails one of §13.3's gates | as the seam states: `ErrForeignProfile` / `ErrStaleProfile` / `ErrInvalidProfile` / `ErrUnrecordableProfile` |
| R22 | `RevolveChain` handed a chain with both free ends on the resolved axis, or an on-axis free end whose incident walk lies along the axis | `ErrUnsupported` |
| R23 | `SweepChain` or `LoftChain`, in every increment before the one that BUILDS the case asked for — Table D rows 12 to 14. The pairing rule is stated: `docs/sweep-design.md` §15.1, `docs/loft-design.md` §16.1 | `ErrUnsupported` |
| R34 | `SweepChain` over a composite path, or over an arc span, before Table D row 13 (`docs/sweep-design.md` Table SC rows SC7 and SC9) | `ErrUnsupported` |
| R35 | `LoftChain` whose two recorded planes are not exactly parallel, or whose to-plane origin does not lie strictly on the from-plane's positive side (`docs/loft-design.md` §16.2, Table SL row SL5) | `ErrUnsupported` |
| R36 | `LoftChain` over a curved correspondence, before the increment that states each computed station's own side (`docs/loft-design.md` Table SL row SL7) | `ErrUnsupported` |
| R24 | `Thicken` on a live body other than §16.1's five admitted sheet families, including a solid or a sheet without an evaluator payload, and an admitted family's receiver whose recorded section is outside §16.2's own class | `ErrUnsupported` |
| R25 | `Thicken` with a wrong-kind, non-finite, negative or zero thickness | `ErrUnitKind` / `ErrNotFinite` / `ErrNegativeMagnitude` / `ErrDegenerate`, respectively |
| R26 | `Thicken`'s offset or assembled section drops a feature, cannot close a join, crosses or touches itself or the source boundary, fails strict nesting, or has an undecided offset audit | `ErrUnsupported` |
| R27 | `Thicken`'s offset or assembly cannot prove the generated plane-local geometry and millimetre thickness exact; a receiver's own interval cannot prove positive height | `ErrUnsupported` |
| R28 | `Thicken` on a retired receiver | `ErrRetiredBody` |
| R29 | `Trim`, `Extend` or `Split` handed a pair `docs/surface-intersection-design.md` §2's entry gate refuses, or a resolution its §6 cannot complete | as that table states: `ErrUnsupported` / `ErrUnrecordableProfile` |
| R30 | `Trim` whose tool separates no fragment of the receiver, or `Split` whose tool separates no part of the target | `ErrDegenerate` |
| R31 | `Trim`, `Extend` or `Split` in every increment before Table D row 8 | `ErrUnsupported` |
| R32 | a curved stitched mesh lacks §10.1's one-source route or §10.2's proven sibling-weld route, has an unsupported surface kind, or has a non-identity stitch placement | `ErrUnsupported`, naming a source surface kind |
| R33 | `RevolveChain` with an interior on-axis junction lacking one swept-wall end and one axis-line end, or with repeated on-axis junctions at one axial coordinate | `ErrDegenerate` |
| R37 | `Offset` on a live body other than §17.1's admitted `patchPayload` or profile-fed `prismPayload` sheet — a solid, a sheet with no evaluator payload, or a prism sheet whose section is outside §17.1's admitted shapes | `ErrUnsupported` |
| R38 | `Offset`'s prism offset drops a feature, cannot close a join, crosses or touches itself, or has an undecided offset audit or interval proof | `ErrUnsupported` |
| R39 | `Offset` cannot prove the prism arm's generated plane-local geometry and millimetre distance exact, or cannot carry the patch arm's translation | `ErrUnsupported` |
| R40 | `Offset` with a wrong-kind, non-finite, negative or zero distance | `ErrUnitKind` / `ErrNotFinite` / `ErrNegativeMagnitude` / `ErrDegenerate`, respectively |
| R41 | `Offset` on a retired receiver | `ErrRetiredBody` |
| R42 | `Offset` in every increment before Table D row 15 | `ErrUnsupported` |
| R43 | `Thicken` on a revolve sheet or a chain revolve shell whose axis is not stated exactly along a recorded plane axis, whose swept offset has no proven strictly positive radius from that axis over the whole interval, or whose assembled section leaves the axis side undecided (§16.5, §16.7) | `ErrUnsupported` |
| R44 | `Thicken` on a chain-fed sheet holding more than one recorded walk, carrying a nonzero section displacement, or holding a walk outside §16.6's axis-parallel right-angle class | `ErrUnsupported` |

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

R21 reuses the four seam sentinels rather than minting chain-specific ones.
Each already names a CAUSE — a foreign source, a stale snapshot, a failed
authentication, a range this seam cannot record exactly — and the caller's
repair for each is identical whether a profile or a chain carried it, so a
parallel set would double the branches a caller writes while every branch's
move stayed the same (§12 widens each sentinel's stated cause to name both).
R29 and R31 are `ErrUnsupported` and STAGED for the same reason, and R30 joins
R5, R7 and R9 as `ErrDegenerate`: a tool that separates nothing names no
trimmed body for any later evaluator to build.

R22, R23, R34 and R36 are `ErrUnsupported` and STAGED: each names real
geometry, and each waits on a topology build rather than on a different
operation. R43 and R44 are `ErrUnsupported` and NOT staged in that sense:
each names a receiver whose own geometry the proof cannot decide — an axis
whose re-expression rounds, a swept offset that reaches the axis, a section
displacement no exact-generation gate can hold — so the caller's move is to
change the model rather than to wait for a later increment (§16.5 to §16.7).
R35 is `ErrUnsupported` and NOT staged in the same sense: the ribbon
between two non-parallel open walks exists, and what this evaluator lacks is a
stated positive side for it (`docs/loft-design.md` §16.2), which is a reach
boundary its own §16.6 names rather than a build waiting in a queue. The one
refusal in §13 that is permanent has no Table R row at all, because the
compiler carries it: `Document.Patch` takes a `*sketch.Profile`, and
`WithSurfaceResult()` implements no chain option tier — `ChainExtrudeOption`,
`ChainRevolveOption`, `ChainSweepOption` or `ChainLoftOption` (§13.2, §13.5).

R32's own naming is not uniform across its causes, and the row's wording
states the weaker claim true of all of them. `tessellateStitchCurved`'s
surface-kind switch (`tessellate_stitch.go`) walks `sp.faces` in order and
returns at the first one its switch does not recognize, so that cause names
the actual offending face's surface kind. Every other cause the row
covers — an incomplete or duplicated face pairing, a non-identity stitch
placement, a broken sibling ancestry check — returns through one shared
`refuse` closure that always names `sp.faces[0]`'s surface kind instead, no
matter which face the cause itself points to, because most of those causes
(a mismatched weld group, an omitted original face) name no single face at
all. "Naming a source surface kind" is true of every cause for that reason;
only the unsupported-surface-kind cause additionally names the one at fault.

R37, R38, R39 and R42 take `ErrUnsupported` for the reasons R24, R26, R27 and
R31 take it: a receiver family this evaluator holds no recorded generator for,
a construction it cannot prove, a value it cannot generate exactly, and a build
that has not landed are each this evaluator's reach rather than geometry with
no usable shape. R40 and R41 restate R25 and R28 for the second entry point
rather than widening either row, so a caller reading one refusal reads both the
same way.

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

A surface-result **loft** sheet tessellates and exports its recorded wall
triangles. The payload holds walls before both cap ranges, so `tessellateLoft`
copies only `tris[:walls]` and attributes each triangle to its live
`side(i,j,k)` face (§4.2). `requireSheetMesh` matches the exposed section rims
to the body's recorded free edges, and `requireSheetVertexLinks` checks the
open boundary vertices. The complete held set still supplies the build-time
crossing and whole-shell orientation proofs; the open mesh does not run the
signed-volume audit, whose sum depends on the anchor when caps are absent.
The sheet mesh carries no occupied-volume proof.

**A stitched `BodySolid` is a different case from every sheet above, and its
own mesh follows §14 Table D row 5.** A
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
`Line3` edges follows the revolve-backed route below when its source qualifies.
The mesh publishes a zero occupied-volume proof
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

### 10.1 Curved stitched mesh from one revolve sheet

Table D row 9 admits a curved or mixed stitched body only when every source
face came from one `revolvePayload` sheet, the source face set is complete,
`Stitch` made no new edge weld or vertex-class merge, and the stitch
placement is the identity.
The source mesh is built through `tessellateRevolve` at the requested chord
tolerance and verification level. Its faces may be `Plane`, `Cylinder`,
`Cone`, `Sphere` or `Torus`, the same surface kinds the flux integral admits.
Any other surface kind, incomplete source face set, new weld or vertex merge, or
unsupported source payload returns `ErrUnsupported` naming a source
surface kind (Table R, R32). This restriction is on the source
construction proof, not on a surface tag by itself. Re-chording a tagged
surface independently was rejected: `Surface` does not record the generating
walk, the shared angular count, or the source mesher's certified rounding.

The source revolve mesh uses one global angular sequence and shared meridian
stations across every source face. `Stitch` copies each source face to one
live face without moving a coordinate under this gate. Reattribute each
triangle through that source-to-live pairing, and reverse its winding when
the live face's `reversed` bit differs from the source face's. The copied
vertex and triangle arrays are fresh; neither the source mesh nor the
stitched body aliases a returned array. A newly welded curved edge from
separate source meshes is refused because their chord stations can differ.
That refusal is necessary even when the two analytic edges share a CURVE
certificate: the certificate proves equal curves, not equal mesh samples.

Each live face inherits its source mesh face bound. That bound already
composes the meridian sagitta, angular sagitta, construction rounding and
source placement rounding through `docs/tessellation-design.md` §8. The
stitch gate proves zero extra placement and zero new weld displacement, so
the copied mesh adds no term. Its `areaSlack` is the source mesh's proved
non-cancelling area allowance. A chorded mesh never claims a zero bound
merely because the source has no recorded triangles. A closed chorded body
publishes no occupied-volume proof (`VolumeVerified() == false`); its signed
volume and its boundary displacement do not bound symmetric difference.
At `VerifyNone`, the source revolve contact audit is skipped and
`BoundaryVerified()` is false. It is true after `VerifyBoundary` or
`VerifyAll` passes that audit. An open sheet also keeps
`VolumeVerified() == false` at every level.

A closed result runs `requireClosedMesh` and `requireVertexLinks` on the
reattributed set. An open result runs `requireSheetMesh` and
`requireSheetVertexLinks` instead. A curved free `Edge` can contribute
several free directed mesh edges: `requireSheetMesh` compares connected
boundary chains per live face, not edge counts. Both sides use the same
live `*Face` pointer, and a source rim's consecutive chord segments stay
one chain. The rejected alternative was matching one mesh segment to one
recorded curved `Edge`; that would reject every sufficiently chorded rim.

### 10.2 Curved welds among one revolve sheet's unstitched faces

An OPEN stitched body also reuses the source revolve mesh when its inputs
are the complete set of single-face sheets made by `Unstitch` on one
revolve sheet, with no placement on the pieces or on the stitch. Every
`unstitchPayload.face` must belong to that same original sheet, and each
original face must appear once. The original may be retired: its payload
and faces remain readable, and `tessellateRevolve` works from that immutable
record. This route admits a new weld only when each welded pair of copied
`Edge`s maps back to the SAME original `*Edge` and every vertex class maps
back to ONE original `*Vertex`. It also requires every original two-face
edge to be re-welded in the stitched body. A failed ancestry check is R32.
The rejected alternative was accepting equal edge coordinates or a CURVE
certificate alone: either can weld analytic curves while two independently
chosen mesh chord sequences still differ.

`Unstitch` copies each face's loops, coedges and vertices in order. The
stitch gate pairs a copied edge and vertex with its original by those
recorded loop positions, then checks the weld plan against original pointer
identity. The source revolve mesh already gives the two original faces
identical chord stations at their shared edge through its global angular
sequence. Reattributing all its triangles through original face → live
stitched face therefore restores the same seam after the real weld. No
coordinate is snapped, interpolated or moved by this mesh path. Each face
inherits its original mesh bound, including meridian/angular chording and
construction/placement rounding; the identity copy and pointer-proven weld
add zero displacement. The source mesh's area allowance and the sheet
audits of §10.1 apply unchanged. This route publishes
`BoundaryVerified()` after the source contact audit and never publishes
`VolumeVerified()` for the open sheet.

## 11. Table X — which existing operations admit a sheet

**Table X — a sheet as receiver or operand**

| Operation | With a sheet | Why |
|---|---|---|
| `Union` / `Cut` / `Intersect` | `ErrUnsupported`, permanently | each owes its caller ONE body and a sheet claims no material to combine, while the mesh path needs an occupied-volume proof a sheet has none of (§10). The Split Body intent has its own entry point instead, `Document.Split` (§1.2, `docs/surface-intersection-design.md` §8) |
| `Fillet` / `Chamfer` | `ErrUnsupported` | `docs/modify-design.md`'s reduction rewrites a prism's **section**; a sheet's free boundary is not a section, and blending to a free edge is its own design |
| `Shell` | `ErrUnsupported` | removes faces from a solid and offsets its material section; a prism sheet does carry a section, but it has no material or caps to remove. `Thicken` uses that section under §16's separate contract |
| `Thicken` | §16.1's five admitted families build; the others are R24 | builds a new solid from a sheet's recorded generator and retires the sheet |
| `Offset` | §17's patch and profile-fed prism families build; the other families are R37 | builds a second sheet at a stated normal distance from the receiver's own faces, over the same recorded generator `Thicken` reads, and leaves the receiver live |
| `Placed` / `PlacedCopy` / `Duplicate` | admitted, unchanged | a rigid motion of a payload; nothing in it reads solidity |
| `Tessellate` / `STL` / `OBJ` | a prism, revolve, loft, all-planar stitched or §10.1–§10.2 revolve-backed stitched sheet tessellates and exports; other curved stitched sheets are R32 | each supported sheet path runs the manifold-with-boundary audit §10 describes |
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
| `docs/api-design.md` §8 | the v1 feature vocabulary gains `Patch`, `Stitch`, `Unstitch` and `WithSurfaceResult`; the signatures land beside the existing ones |
| `docs/api-design.md` §8 | `Body.Thicken`, `ThickenSide` and `WithThickenSide` add §16's sheet-to-solid path under §6's consuming-body rule |
| `docs/api-design.md` §8 | `Body.Offset`, `OffsetSide` and `WithOffsetSide` add §17's sheet-to-sheet path under §6's NON-consuming rule, the one `PlacedCopy` and `Duplicate` already take |
| `docs/api-design.md` §9 | the predicate list gains `Free()`, and the rendering table gains its `free` token |
| `docs/api-design.md` §13 | the v1 non-goal list keeps sheet-metal, and names §1.2's staged commands, §1.3's permanent refusals and §1.4's unstaged one, so the list stays the whole of what is out |
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
| `docs/api-design.md` §7 | the seam gains the open chain: `RecordChain` beside `RecordProfile`, under the identical foreign, stale, snapshot-match and `TExact` gates (§13.3) |
| `docs/api-design.md` §8 | the v1 feature vocabulary gains `ExtrudeChain`, `RevolveChain`, `SweepChain` and `LoftChain`; the four signatures land beside the existing surface block |
| `docs/api-design.md` §12 | `ErrForeignProfile`, `ErrStaleProfile`, `ErrInvalidProfile` and `ErrUnrecordableProfile` each state that their cause is a profile OR an open chain (§7 there, §13.3 here) |
| `docs/sketch-seam-design.md` §2 | the recording IR gains `ChainRecord` and `RecordChain`, over the same ten `CurveSegment` variants; §2.1's admission list gains the chain's own interior-junction reading (§13.3) |
| `docs/api-design.md` §2.1 | the admitted-class sentence names the shared-generator predicate and both owning documents, in place of the one class it named before |
| `docs/api-design.md` §8 | the v1 feature vocabulary gains `Body.Trim`, `Body.Extend` and `Document.Split`, with `TrimSide`; the signatures land beside the existing surface block (`docs/surface-intersection-design.md` §8) |
| `docs/prism-boolean-design.md` §3.4 | states that the same three displacement causes REFUSE rather than reroute wherever no mesh path exists, which is every sheet-involving pair (`docs/surface-intersection-design.md` §2.1 S7, §5) |
| `docs/layout.md` | a row for this document and one for `docs/surface-intersection-design.md`, and one per `.go` file each increment adds; the chain increment adds no file, so `record.go`'s and `seam.go`'s own rows name the chain instead |
| `CLAUDE.md` | a "Read before you write" row pointing here for sheet-body, surface-feature, patch and stitch code, and one pointing at `docs/surface-intersection-design.md` for trim, extend and split code |

## 13. The open sketch chain — a ribbon and an uncapped shell

**Fusion's commonest surface extrude starts from an open sketch curve, and this
section is where decad builds from one.** `sketch.Profile` is a closed planar
region, so `WithSurfaceResult()` only ever builds a wall set around a region: a
rectangle gives a four-walled tube, and a single line gives nothing at all.
`sketch.Chain` is that region's open counterpart — an ordered run of boundary
edges whose two ends are free — and `ExtrudeChain` sweeps one into a ribbon
while `RevolveChain` spins one into a shell with no cap.

### 13.1 What `sketch` publishes, and what decad reads

`Sketch.Chains()` returns the sketch's open connected runs: every edge of the
same single arrangement that no region boundary used, so no edge is reported by
both `Profiles()` and `Chains()`. Three things a chain carries are what let the
existing seam govern it with no new rule.

- **The ordered walk.** `Chain.Edges` is `[]BoundaryEdge` — the same struct a
  profile's `Outer` and `Holes` hold, filled by the same `mapBoundaryEdge`
  conversion — so `Partial`, `Reversed`, `TStart`, `TEnd` and `TExact` mean on a
  chain edge exactly what `docs/sketch-seam-design.md` §1 states they mean on a
  region boundary edge. `Chain.Entities` is the de-duplicated entity set in walk
  order, `Profile.Entities`'s counterpart.
- **Authentication and staleness.** `Chain.Sketch()` returns the source sketch,
  `Chain.Revision()` the `Sketch.Revision()` value the chain was built at, and
  `Chain.IsStale()` compares the two. One arrangement pass stamps every chain
  and every profile from one `Sketch.Revision()` read, so a chain's staleness
  answer is the answer a profile's is.
- **The arrangement's own self-intersection verdict.** `Chain.SelfIntersecting`
  marks a walk that crosses or touches itself; `Chain.Valid` is false for that,
  for an unresolvable degeneracy reaching the chain's own curves, and for a
  zero-length walk. Validity is scoped as `Profile.Valid` is: an attributable
  condition on curves this chain does not use leaves it valid, and an
  unattributable one invalidates every chain and every profile at once.

Every ask this section once held is answered by the published type, and decad
still assembles no chain of its own: which entities form a run, and in what
order, is a 2D arrangement question `sketch` answers (`CLAUDE.md`). One field
decad must not read as a measurement is `Chain.Length` — §13.3 states why, and
comparing it is the whole of what decad does with it.

**`sketch` cuts the walk wherever it cannot continue unambiguously, so one
curve a caller drew is not always one chain.** The cut points are a vertex where
three or more edges meet and the crossing point of two curves, so three lines
meeting at a point publish three chains, and a curve another curve crosses
publishes its pieces separately. Two consequences a caller acts on, stated here
rather than left to be found later:

- **One ribbon per chain.** An L drawn as two lines meeting at a plain corner is
  one chain, and `ExtrudeChain` builds one two-face ribbon from it. A T is three
  chains, so the caller extrudes each and may `Stitch` the three ribbons
  afterwards. No option asks `ExtrudeChain` to span a cut vertex: spanning one
  means choosing which two of three edges continue the walk, which is the
  arrangement decision `sketch` declined to make and decad never re-derives.
- **A chain's index is not a handle.** The published order consults entity and
  point names, and `Sketch.Revision()` hashes none of them, so renaming an
  entity re-ranks `Chains()` while every held chain stays fresh. A caller picks
  a chain by what it holds, and §13.3's authentication gate matches by content
  against the whole fresh set rather than at one index.

### 13.2 The entry points

**A chain gets its own two entry points rather than widening the profile
path's.**

```go
func (d *Document) ExtrudeChain(s *sketch.Sketch, ch *sketch.Chain, e Extent,
    opts ...ChainExtrudeOption) (*Body, error)
func (d *Document) RevolveChain(s *sketch.Sketch, ch *sketch.Chain, axis Axis,
    a AngularExtent, opts ...ChainRevolveOption) (*Body, error)
```

Each takes the sketch beside the chain, for the reason `Extrude` does: a chain's
geometry is plane-local and the plane is the sketch's (`docs/api-design.md` §7).
Each resolves its extent through the vocabulary its profile-fed sibling already
uses, and neither takes a context, exactly as `Extrude` and `Revolve` take none.

**The rejected alternative is a sealed `Section` interface that both
`*sketch.Profile` and `*sketch.Chain` satisfy**, handed to the existing
`Extrude` and `Revolve`. It would turn every existing call site's compile-time
guarantee — this argument is a closed region, so this call builds a solid — into
a runtime question, and it would make `WithSurfaceResult()`'s ABSENCE mean
"build a solid" for one variant and nothing at all for the other. Two entry
points keep each contract decidable by the type.

**`WithSurfaceResult()` does not compile against either call, which is this
design's answer to whether a chain result is always a sheet.** It always is: an
open walk encloses no region, so there is no closing face to omit and no solid
to ask for. `ChainExtrudeOption` and `ChainRevolveOption` are their own sealed
option tiers, and `SurfaceResultOption` — `ExtrudeOption` + `RevolveOption` +
`SweepOption` + `LoftOption` — implements neither, so the compiler refuses the
option and Table R carries no row for it. The rejected alternative is accepting
it as a no-op, which gives one option two meanings: omit the closing faces here,
state nothing there. Neither tier has a member in this increment; both exist so
a later chain-only option has a tier to land on.

### 13.3 `ChainRecord` — the record, and its gates

decad records a chain before building it, exactly as it records a profile:

```go
// ChainRecord is ProfileRecord's open counterpart: one directed OPEN walk,
// structural and plane-local, over the same sealed CurveSegment variants. The
// first segment's walk start and the last segment's walk end are FREE — they
// meet nothing, and nothing closes onto them.
type ChainRecord struct {
    Segments []CurveSegment
}

// RecordChain is RecordProfile's sibling: it admits, authenticates and records
// an open chain under the identical gates, and returns the structural values.
func RecordChain(s *sketch.Sketch, ch *sketch.Chain) (ChainRecord, PlaneRecord, error)
```

**What it shares with `ProfileRecord` is everything the seam already owns.** The
ten sealed `CurveSegment` variants record a chain edge exactly as they record a
region boundary edge — the entity's own defining fields verbatim, plus `sketch`'s
normalized range, with `Reversed` baked in as the order of that range
(`docs/sketch-seam-design.md` §2). The `PlaneRecord` beside it is the same value,
read through `s.Plane().Frame()`. Every admission gate is the same gate, run by
the same code: `record.go` owns the type beside `ProfileRecord`, and `seam.go`
owns `RecordChain` beside `RecordProfile`, because a file of its own would copy
the variant set and the gates both.

**What it cannot share is `LoopRecord`.** A `LoopRecord` states that its last
segment's walk closes onto its first, and every consumer downstream reads one as
the boundary of a closed region — `moments.go`'s region integrals,
`triangulate.go`'s cap tiling, `segment_walk.go`'s region walks. A chain states
the opposite at its two ends, so recording one as a `LoopRecord` carrying a flag
would hand each of those consumers a closure that is not there. Two further
things fall away with the loop: a `ChainRecord` holds no `Holes`, since an open
walk has no interior to put one in, and no walk-level winding, since "outer
counter-clockwise, holes clockwise" names a side an open walk does not have. A
closed-kind segment's own `CCW` survives, because it still records the walk
sense — but it survives only on a FRAGMENT, since a whole `*Circle`, `*Ellipse`
or `*ClosedSpline` edge is a closed run `sketch` publishes as a `Profile` or
publishes nowhere, and so never reaches a chain. Nor does a `ChainRecord` feed
any region reading: `moments.go` integrates `Area`, `Centroid` and
`SecondMoments` over a region, and a chain bounds none. Its only 2D reading is
per-segment arc length, through `segment_walk.go` and, for a free-form segment,
`spline_length.go`'s proven bracket.

**The gates are the existing ones, in the existing order, and every sentinel is
reused unchanged.**

| Condition | Handling | Sentinel |
|---|---|---|
| `ch.Sketch() != s`, or a `ch.Edges[i].Entity` that is nil or absent from `s.Entities()` | refused before anything is read off the chain, so no foreign plane-local coordinate is ever lifted through `s`'s frame | `ErrForeignProfile` |
| `ch.IsStale()` | refused: the walk describes the sketch's earlier geometry, and sweeping it builds the wrong shape silently | `ErrStaleProfile` |
| the snapshot matches no member of a fresh `s.Chains()` result | refused: every exported field — `Entities`, `Edges`, `Length`, `Valid`, `SelfIntersecting` — must match one fresh chain exactly, and the fresh match is what records | `ErrInvalidProfile` |
| `ch.Valid == false` | refused: `sketch`'s own verdict, never one decad recomputes | `ErrInvalidProfile` |
| `ch.SelfIntersecting == true` | refused on the same row; `Valid` is already false for it, so this is a reject-only redundancy rather than a second gate | `ErrInvalidProfile` |
| a `Partial` edge whose `TExact` is false | refused: an uncertified range is never recorded as an exact trim, never widened to the whole entity, and never repaired | `ErrUnrecordableProfile` |
| a certified range the reject-only falsifier disproves | refused, and the discrepancy reported upstream as a `sketch` bug | `ErrUnrecordableProfile` |
| an INTERIOR junction whose two coordinates contradict | refused by the same source-aware rule: bit-equal for two same-source coordinates, the range falsifier's relative threshold for a record endpoint against a certified cut node | `ErrUnrecordableProfile` |

Three readings of that table carry this increment's soundness argument, and each
follows from a rule already stated rather than adding one.

**The match is by content over the whole fresh set, never at one index.**
§13.1's renaming case is why: a held chain can be fresh and correct while
sitting at a different index, so an index comparison would reject a chain
nothing is wrong with. Matching every exported field against some fresh member
is what `docs/api-design.md` §7 already prescribes for a profile, which is
freshly allocated on every call for the same reason.

**Only the walk's INTERIOR junctions are checked.** A `LoopRecord`'s closure
check runs at every junction including the one that closes the loop; a chain has
no such junction, and asserting one would reject every chain there is. The two
free ends state no junction to check, exactly as a whole closed curve states
none (`docs/sketch-seam-design.md` §1).

**`Chain.Length` is compared and never consumed.** It is a bare `float64`,
published whatever `TExact` reports, and exact only for a `*Line`, `*Arc` or
`*Circle` fragment — for every other entity it is a sampling-convergent
UNDERESTIMATE, the chord sum of the entity's own curve over the reported range,
with no bound stated for the gap. A measurement decad published from it would be
confidently short by an unstated amount, which is the failure this engine exists
to prevent, and `docs/api-design.md` §5.1's rule that a scalar quantity is a
`units.Value` refuses the bare float besides. So the field enters the snapshot
comparison as an integrity check — equality against the fresh chain's own value,
the use `Profile.Area` already gets at the same seam — and nothing else reads
it. Every length decad publishes about a ribbon is computed from the recorded
segments, by the engines that already prove their own bounds.

`RevolveChain` runs `revolve_axis.go`'s existing axis resolution over the
recorded walk: the axis must be non-degenerate and coplanar with the sketch
plane, and the walk must lie in one closed half-plane of it. `axisFrame.walk`
snaps an endpoint within `snapTol` to exactly zero and charges the discarded
radial distance to `startVBound` or `endVBound`. The pole test reads that snapped
zero exactly; it does not admit a near-zero radius through a new tolerance.

**Exactly one free end may be a pole.** Its incident walk must leave the axis;
the other free end must be off-axis. The off-axis walk sweeps a wall ending at
one point rather than a latitude circle, so a full revolution has one free rim
and a partial sweep has two free meridian copies meeting at the pole plus the
other end's sweep arc. This admits a dome or hemisphere shell capped at its
pole. Both free ends on the axis remain R22: the resulting closed sheet has no
free rim and needs a separate no-boundary topology audit. An on-axis free end
whose incident walk lies along the axis also remains R22: that walk emits no
wall, so its free endpoint cannot be the proposed wall's pole.

The chain build reads its two free ends without wraparound. A free pole has one
incident off-axis walk end and needs no partner `LineSeg`; an interior on-axis
junction still needs the one off-axis/one on-axis-line pair of
`docs/evaluator-design.md` §6. The closed-profile revolve keeps its existing
axis-incidence audit and build unchanged. Reusing its wraparound rule at a
chain free end would reject the pole for a missing partner.

### 13.4 Table G — what a chain-fed feature builds

A chain-fed feature mints no face to close anything: the record states no loop,
so there is no cap, no seam face and nothing for a later option to omit. Table G
is therefore three columns rather than Table W's four.

**Table G — a chain-fed feature's faces, orientation and free edges**

| Feature | Sheet faces | Positive side | Free edges |
|---|---|---|---|
| `ExtrudeChain` | one wall per recorded segment, by `docs/evaluator-design.md` §5's own per-kind table: a `LineSeg` a `Plane`, a `CircleSeg`/`ArcSeg` fragment a `Cylinder` patch, a free-form segment a `NURBSSurface` | `T × N` at each wall point, `T` the walk tangent and `N` the plane normal — the walk's right-hand side, the identical construction a profile-fed wall's outward normal already takes | both rims of the walk, plus the one sweep edge at each of its two free ends |
| `RevolveChain`, partial sweep | one swept wall per off-axis recorded segment | the same sense, transported through the sweep by `revolve_build.go` unchanged | the walk's own copy at each of `phi0` and `phi1`, plus the arc each off-axis free end sweeps; a free pole mints no arc |
| `RevolveChain`, full revolution | one swept wall per off-axis recorded segment, the walls closing angularly | the same | the circle each off-axis free end sweeps; a free pole mints no circle |

Three readings of Table G are worth stating outright.

**Face count is segment count.** A single-line chain extruded is ONE face, the
ribbon this section opened on. `Extrude`'s own rim coalescing
(`coalesceWalksContext`) still merges adjacent collinear segments into one rim
edge, exactly as it does for a profile, so a line drawn as two collinear halves
gives one wall and one rim edge per end.

**A full revolution of an admitted open chain is an OPEN sheet.** Two off-axis
free ends sweep two free circles; one free pole and one off-axis end leave one
free circle. No face fills either circle. `Body.Patch` can fill one on §5.2's
terms. A pole is a shared `Vertex` at the meeting point of the two meridian
copies in a partial sweep. In a full revolution the pole is the face's singular
point, represented without a zero-length edge or a separately reachable
`Vertex`, exactly as `fullRevLoops` represents a closed-profile revolve's pole.
Creating a zero-length topological edge would give `Edges(Free())` a rim that
has no geometric extent. Both-ends-on-axis remains R22 rather than being
called a closed sheet by an empty free-edge set.

**The positive side derives nothing new.** `T × N` is the vector a profile-fed
wall already publishes as its outward normal — for a plane whose normal is `+Z`
and a walk running `+X` it is `−Y`, the side away from the material a closed
walk would carry on its left — so a chain-fed wall and a profile-fed wall over
the same recorded segment publish the same normal, face for face. The rejected
alternative is a caller-stated side option, which would add an input the profile
path does not have and give one built surface two admissible orientations.

**What the body publishes**, and where each answer's exactness comes from:

| Reading | A chain-fed body |
|---|---|
| `Kind()` | `BodySheet`, always and by construction — a decided enum carrying no bound (§2.1) |
| `IsSolid()` | `false`, always: Table K's fourth row is structural |
| `Volume()`, `Centroid()` | `ErrNotSolid`, by KIND rather than by soundness (§8) |
| faces | one per recorded segment, Table G |
| `Edges(Free())` | Table G's own count: a full-turn chain with one pole has one free rim; a partial turn with one pole and one segment has two free meridian edges plus one off-axis sweep arc |
| `Area` | the sum of each wall's own area through `boundedAdd`: `segment length · h` for an extrude, `boundedMul(walkAxisMoment, sweep)` for a revolve. A pole adds zero area. `walkAxisMoment` charges the snapped radial endpoint's bound and the segment-length bound, and that length bound itself carries what the snap discarded: moving an endpoint onto the axis moves the wall's two ends APART as well as inward, by at most the sum of the two discarded radial magnitudes, and the recorded length is the unsnapped one (`axisFrame.walk`). Charging only the radial endpoint would leave a nearly RADIAL wall short — a disk spends the whole discarded radius on its length where a steep cone spends a fraction of it. The sweep carries its angle-denotation bound; `boundedMul` charges the product and `boundedAdd` charges the wall sum. A circular wall also carries its proven integral enclosure, and a free-form wall carries `spline_length.go`'s proven bracket |
| `Bounds` | the recorded walk's per-segment analytic extremes swept over the signed interval, from `prism_extent.go` and `revolve_extent.go` verbatim, charging the frame and placement rounding those readings already charge |

No bound in that table is new, because no geometry is: every wall is built by
the identical per-segment construction — `buildLoopSidesAs` for a prism,
`revolve_build.go`'s wall walk for a revolve — so its surface, its area and its
bound are the ones that construction already proves. The tolerance gate's
reference diameter comes from the same extent readings a profile-fed payload
supplies it from, so §8's "no sheet payload withholds the gate" holds here
unchanged.

**§9.1's fourth validity leg is earned by construction, on the profile argument
with one word changed.** A chain-fed prism earns it because `sketch` already
proved the recorded walk a simple planar curve — `Valid` true means the walk
neither crosses nor touches itself, and a walk whose two ends met would be a
closed run `sketch` publishes as a `Profile` instead — `evalPrismContext`
already refuses a non-positive sweep height, and a simple planar curve crossed
with a positive interval cannot self-intersect. A chain-fed revolve earns it on
the full-turn argument §9.1 already carries: away from a free pole, two distinct
generating points map to one 3D point only by sharing both
radius and axial position, which inside one closed half-plane makes them the
same 2D point. At a free pole every angular fibre collapses to that one point;
the simple walk has no second incidence there. A partial-turn chain revolve
earns no such proof and reads `ValidityUndecided`, exactly as a partial-turn
profile revolve does (§9.2).

**Everything else a sheet already does, a ribbon does unchanged.** Table X
governs it row for row: a boolean, `Fillet`, `Chamfer` and `Shell` refuse it,
`Placed`/`PlacedCopy`/`Duplicate` admit it, a planar wall answers `ToFace`, and
every selector predicate reads its faces and edges. A chain-fed prism ribbon
tessellates and exports through §10's manifold-with-boundary audit, since its
walls chord exactly as the same record's profile-fed siblings do and it mints no
cap to leave out; a chain-fed revolve ribbon's mesh remains `ErrUnsupported`:
`tessellateBodyContext` has no `chainRevolvePayload` arm, and `planRevolve` plus
its wraparound axis-incidence audit consume closed loops. The existing pole-fan
chording does not prove that an open walk's free pole gets one manifold
boundary. Its mesh is a separate increment. A ribbon is an ordinary `Stitch` operand: a
straight walk's free edges are all `Line3`, so Table J's J5 answers vacuously
for them and two ribbons meeting at a proven-coincident end weld. `Body.Patch`
reads a ribbon's free edges as one closed chain and admits or refuses it on
§5.2's four gates with no new rule — planar for a straight walk, `ErrUnsupported`
(R6) for an L.

### 13.5 What refuses a chain

- **`Document.Patch` has no chain-fed form, permanently.** A planar fill needs a
  region to fill and an open walk encloses none; a walk whose ends met would be
  a `Profile`, so no chain a patch could take exists. The refusal is the type —
  `Document.Patch` takes a `*sketch.Profile` — and the caller's move is to close
  the curve in the sketch. This is §1.3's shape of permanent refusal: no later
  proof changes it.
- **`SweepChain` and `LoftChain` refuse with `ErrUnsupported` until the
  increment that builds the case asked for** (Table R, R23), on §14's own
  staging rule — the signature lands so a caller's intent has somewhere to go
  and a refusal to read, and the build lands with its own increment. Each
  pairing rule is stated in the document that owns the operation, never here:
  `docs/sweep-design.md` §15.1 for the composite join, and
  `docs/loft-design.md` §16.1 for Table P over two open walks, with its §16.2
  carrying the positive side a closed shell supplied and two open walks do not.
  Both entry points take a sealed chain-only option tier for §13.2's reason, so
  `WithSurfaceResult()` compiles against neither.
- **`Stitch` and `Unstitch` never see a chain.** Both take bodies, so there is
  nothing to refuse: a ribbon reaches them as the sheet it is, on §6's terms.
- **`WithSurfaceResult()` never reaches a chain-fed call**, by §13.2's option
  tiers — a compile error rather than a sentinel.

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
| 6 | The open sketch chain (§13): `ChainRecord` and `RecordChain` beside `ProfileRecord` and `RecordProfile`, under the same gates and the same four sentinels; `Document.ExtrudeChain` and `Document.RevolveChain` with their two sealed option tiers, building Table G's wall set with no cap and no closing face; the fourth validity leg for a chain-fed prism and for a full-turn chain revolve clear of the axis; prism ribbon tessellation and export on the identical manifold-with-boundary audit; Table A's four new amendment rows; §15's T130–T141. `Document.SweepChain` and `Document.LoftChain` land as signatures refusing with `ErrUnsupported` (R23); a chain free end ON the revolve axis stays R22 until row 10; the chain-fed revolve mesh remains separate |
| 7 | `Body.Thicken` for §16's patch and profile-fed prism cases, all three sides, the full offset-interval and cross-boundary audits, and T150–T157. The revolve and chain-fed families land in rows 15 to 17; the sweep, loft and stitched families stay R24 |
| 8 | `Trim`, `Extend` and `Split` over a pair whose two sweeps share one generator, in the four PRs `docs/surface-intersection-design.md` §11 states: its §2 entry gate, `buildPrismScene`'s `ChainRecord` arm, `classifyPrismCells`'s side reading consumed unchanged, the open-walk chaining of its §3.3, `chainPayload`'s and `chainRevolvePayload`'s walk set and section displacement, and the one displacement term its §7 derives from `bounds.go`'s existing `cutParamUlps`/`cutDisplacementAllow`. Table A's five new rows; Table R's R29–R31; §15's T170–T181. A pair sharing no generator, a chain ribbon as `Trim`'s receiver, and a second trim of an already-trimmed body each refuse with `ErrUnsupported`, and that document's §4 and §5 own why |
| 9 | §10.1's curved or mixed stitched mesh from one complete revolve sheet, closed or open. Other source constructions and new curved welds remain R32 until they can prove identical seam samples. |
| 10 | One free pole on `RevolveChain` (§13.3): Table G's pole topology, the wall `Area` and `Bounds` charges, one free rim after a full revolution, and T139 plus T142–T146. Both free ends on the axis stay R22; interior axis pinches are R33. The chain-fed revolve mesh remains a separate increment because its open-walk pole fan and boundary audit need their own proof |
| 11 | §10.2's open stitched mesh from all single-face sheets unstitched from one revolve sheet. Pointer-proven re-welds reuse the source mesh's identical curved seam samples; other curved welds remain R32. |
| 12 | `Document.SweepChain` over a ONE-SPAN straight path (`docs/sweep-design.md` §15, PR C1): the sealed `ChainSweepOption` tier in place of the staged signature's `SweepOption`, Table SC's gate set, the chain prism reduction over the path's own height and composed length bound, and §15's T190–T195. An arc span and every composite path stay R34 |
| 13 | `SweepChain`'s one-span ARC reduction and then §15.1's composite join (`docs/sweep-design.md` PRs C2 and C3): the wall-face rim lists a capless span supplies, the sew, the separation certificate over chain spans, and the assembled boundary audit. It retires R34 |
| 14 | `Document.LoftChain` over a `LineSeg`-only correspondence (`docs/loft-design.md` §16, PR L1): the sealed `ChainLoftOption` tier, Table SL's gate set, §16.2's exactly-parallel plane gate and per-wall positive side, the open-walk cell walk with its two free end rungs, and §15's T196–T200. A curved correspondence stays R36 and a non-parallel plane pair stays R35 |
| 15 | `Body.Offset` for §17's patch and profile-fed prism families, both sides: the patch arm's translation over the existing placement rebuild; the prism arm's `offsetProfile` section reused from §16.2 with its exact-generation gate, its offset audit and its whole-interval certification, and WITHOUT the source-and-offset nesting audit an annulus needs; Table R's R37–R42; Table X's own row; and §15's T200–T207. Other sheet families stay R37 |
| 16 | `Body.Thicken` on a profile-fed revolve sheet (§16.5): the exact axis class, the meridian annulus over §16.2's own offsets, the radial gate over the interval scan's own boxes, the re-resolved axis frame, and §15's T158–T161 |
| 17 | `Body.Thicken` on a chain ribbon (§16.6): the closed-form assembled section, its exact-generation gate and endpoint crossing audit, all three sides, and §15's T162–T164 |
| 18 | `Body.Thicken` on a chain revolve shell (§16.7): §16.6's assembled section under §16.5's sweep and radial gate, and §15's T165–T169. A free end on the axis stays R43 |

**Increment 7 depends on increment 1's patch and prism sheets and analytic
prism builder, plus `docs/modify-design.md` §5/§8's section offset and audit.**
It depends on none of increments 2–6.

**Increment 15 has increment 7's dependency set exactly, and neither increment
depends on the other**: 7 builds a solid from the offset section and 15 a
sheet, over the same `offsetProfile` construction and the same two gates. Either
may land first; 15 after 7 costs less review, because the gates 15 reuses are
then already in the tree.

**Increments 16, 17 and 18 each depend on increment 7 alone**, for the section
offset, the exact-generation gate and the interval scan every one of them
consumes unchanged. Increment 16 also depends on increment 1's revolve sheet and
17 and 18 on increment 6's chain-fed sheets, each for the receiver it thickens
and for nothing else. Increment 18 depends on 16 and 17 together, since it is
their two halves composed; 16 and 17 are independent of each other and either
order is admissible.

**Increment 8 depends on increments 1 and 6**, and on no other: increment 1 for
`BodyKind`, `Edge.IsFree` and §9.1's sheet validity audit, and increment 6 for
`ChainRecord` and the ribbon wall build every trimmed result is assembled by.
None of 2, 3, 4 or 5 is a prerequisite of it, and none of them depends on it. A
trimmed revolve sheet's own mesh waits on increment 4 exactly as a profile-fed
revolve sheet's does, which is that increment's reach rather than a dependency
of this one.

**Increment 6 depends on increment 1 alone**, for `BodyKind` and `Kind()`,
`Edge.IsFree` and `Free()`, §8's sheet measurement rows, §9.1's sheet validity
audit and the prism sheet mesh — every one of which it consumes unchanged, and
none of which it amends. It depends on none of 2, 3, 4 or 5, and none of them
depends on it: a ribbon reaches `Stitch` and `Body.Patch` as an ordinary sheet
operand once increment 2 lands, which is increment 2's own reach rather than a
prerequisite of this one. It is numbered after 5 only so that no reference to
an earlier row has to move, and any order among 2 through 6 that keeps 1 first
is admissible; 7 comes after 6 wherever 6 lands.

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
An admitted full-turn `RevolveChain` remains an open sheet: its off-axis free
ends sweep one or two circles that nothing fills (§13.4).

§16.8 stages the remaining `Thicken` receiver families at R24 and states what
each is missing; §17.1 refuses every family but its own two at R37, which is a
wider set than §16.1's since an offset publishes a surface where a thicken
publishes a solid. Trim and Extend are increment 8's, over the shared-generator
class alone, and a sheet operand in `Union`/`Cut`/`Intersect` refuses
permanently (Table X) with `Document.Split` carrying that intent instead. Ruled
and Boundary Fill take no increment at all — §1.3 refuses them permanently —
and Reverse Normal takes none either, on §1.4's own reading. Every unlanded
build refuses at the call with `ErrUnsupported`.

## 15. Test obligations

`CLAUDE.md` requires every capability to ship with a test asserting on
computed geometry, never merely that it ran. Each row below names a concrete
assertion, and each bound assertion must first be **shown to fail** — the
proof leg deleted, the test watched to go red — before it is trusted.

The row-9 curved-mesh obligations are:

- T83: Stitch T31's annular full-turn revolve sheet. Tessellate at two
  tolerances; assert every source face is live, all directed edges pair,
  the fine mesh's signed tetrahedron sum is closer to 2000π mm³ than the
  coarse mesh's, and each mesh copies its source sheet's vertices exactly.
- T84: Repeat with T46's two cones, T50's complete sphere and T53's
  cylinder/torus pair. Assert each mesh's surface-kind counts, positive
  facet areas and signed tetrahedron sum against the same-tolerance direct
  revolve mesh, whose independent solid proof encloses analytic volume.
- T85: Stitch one partial-revolution sheet whose two curved faces share an
  internal latitude and whose curved free rims each chord into several
  segments. Assert the mesh's per-face free-boundary chain counts equal
  `Body.Edges()`'s, and every shared seam segment has one reverse.
- T86: T83 at `VerifyNone`, `VerifyBoundary` and `VerifyAll` reports
  `BoundaryVerified()` false, true and true; all three report
  `VolumeVerified()` false. `STL` and `OBJ` export the same geometry at
  the same tolerance.
- T87: T34's patched prism tube and a hand-built `NURBSSurface` stitch
  each return `ErrUnsupported` from tessellation, naming `Cylinder` and
  `NURBSSurface` respectively. A placed copy of T83 also refuses R32.
- T88: Remove the inherited source-face bound from the T83 mesh, run the
  test, record the failure, and restore it. The asserted relation is that
  every stitched face bound covers its source mesh face bound; a missing
  charge publishes zero where the source bound is positive.
- T89: Remove the source-to-live face remap from T85, run the free-boundary
  attribution test, record its failure, and restore it. This isolates the
  face-identity proof from the chording proof.
- T190: Unstitch T85's partial cylinder/torus sheet into its two faces,
  stitch both pieces, then tessellate. Assert the original and re-stitched
  meshes have identical vertex coordinates and signed facet area, the
  curved shared `Edge` is no longer free, curved free arcs still chord into
  multiple segments, and the mesh/body free-chain counts agree per face.
- T191: Place one T190 piece before stitching. The result may be an open
  sheet, but its mesh is R32 because the original shared stations no longer
  describe the placed copy. A second fixture whose two copied edges map
  to different original `*Edge`s refuses through the ancestry gate.

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
| T10 | a sheet tessellated | prism, revolve and loft sheets tessellate and export through the manifold-with-boundary audit (§10; `tessellate_sheet_test.go`, `tessellate_revolve_sheet_test.go`, `surface_loft_test.go`); a loft sheet omits both recorded cap ranges and carries no occupied-volume proof |
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
| T62 | each of T46's frustum, T50's ball and T53's torus body, stitched | `Tessellate`, `STL` and `OBJ` succeed through §10.1; each mesh has a positive bound and no occupied-volume proof; the body's analytic `Volume`/`Centroid` read bit-identically before and after export |
| T63 | T58's box tessellated twice at two different tolerances | equal vertex order, triangle order and source-face order; byte-identical STL and byte-identical OBJ; mutating the returned `Vertices()`/`Triangles()` slices changes nothing on a later call |
| T64 | `Union` of T58's box with a plain `Extrude` block | this fixture's own outcome changes from T71 onward, once the occupied-volume proof lifts for a CLOSED, all-planar, zero-vertex-bound stitched solid; a stitched operand this proof does not cover (placed, or certificate-welded) keeps this row's own `ErrUnsupported`, the volume-proof refusal (`operandSymDiff`'s wording, via `requireVolumeProvingPayload`'s own `stitchPayload` arm), not the payload-class one — T73, T74 |
| T65 | T58's box and a plain solid block 3 mm beyond its own +X wall, `Verify(WithClearances())` | `Sound`; exactly one `Clearance` row; its proven interval encloses 3 mm; `Exact`. Replaces `TestStitchSolidDoesNotYetReachAPairRelation`'s Sound-side premise |
| T66 | T58's box and a plain block straddling its own +X wall (a true, non-nesting overlap) | the clearance kernel itself proves `pairOverlapping` (`TestClearancePairProvesStitchedSolidOverlapDespiteTheBooleanRefusal`, internal) but the public report still reads `Suspect` with no `Interference` row: this exact fixture's block shares the box's own y-range and z = 0 base plane, so even once the occupied-volume proof lifts (T72), the mesh boolean's own separate, pre-existing coplanar-contact gate refuses it — `DiagUnsupportedPairContact` from T72 onward, no longer `DiagUnsupportedPairPayload` |
| T67 | a small stitched box wholly inside a large plain block, boxes meeting | the solid-solid path's own strict-containment certificate: `Interfering`, exactly one `Interference` row reusing the contained box's own `Volume`, no `Clearance` row — reached through the clearance kernel's own carrier model alone, with neither operand ever tessellated |
| T68 | T42's own certificate-welded stitched solid (nonzero vertex bound at identity) against a plain block with a real box-proven gap, `Verify(WithClearances())` | a carrier model built from the body's own topology; `Sound` with exactly one `Clearance` row, honestly `Approximate` over a positive `Bound` — the body's own largest proven vertex bound (`stitchMaxVertexBound`) is charged into `bodyGeom.delta` and widens the interval once, rather than the bound refusing a model outright. Shown-to-fail: restoring the zero-vertex-bound gate on the `stitchPayload` arm turns this pair back to `Suspect` with `DiagUndecidedClearance` and no row |
| T69 | T58's box `Placed` under a non-identity rigid motion, same pairing as T68 | identical outcome, reached by the other route: the placement's own `rigidRoundAllow` widens every vertex bound rather than a certificate weld's own class bound — neither T68 nor T69 alone would be trusted to test that the charge reads the vertex bound itself rather than one particular cause of it |
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

| T130 | a single 40 mm line, the only non-construction entity in its sketch, `ExtrudeChain` 10 mm `Along` | `s.Profiles()` is empty and `s.Chains()` has length 1; `Kind() == BodySheet`, `IsSolid() == false`, `Volume()` is `ErrNotSolid`; exactly 1 face; `Area` exactly 400 mm² and `Exact`; `Edges(Free()).Exactly(4)` resolves; `Bounds` is the 40×0×10 slab, `Exact` |
| T131 | an open three-segment walk — line, arc, line, each meeting the next at a shared point | 3 faces; `Edges(Free()).Exactly(8)`, the two junction edges matching nothing under `Free()`; the arc wall's `Area` is `Approximate` over `rθ`'s own bound while both line walls read `Exact`; the body's own `Area` equals the three walls' sum through `boundedAdd`, value and bound |
| T132 | a rectangle with one side erased, `ExtrudeChain` 10 mm `Along` | 3 faces, not 4 — the erased side mints no wall, which is the ribbon §13 opens on; `Edges(Free()).Exactly(8)`; `Volume()` is `ErrNotSolid` |
| T133 | three lines meeting at one point, `ExtrudeChain` over each member of `s.Chains()` in turn | `s.Chains()` has length 3; each call returns its own one-face ribbon and the document holds three bodies, never one — the walk-cutting consequence §13.1 states, asserted rather than assumed |
| T134 | an open two-segment walk clear of the axis, `RevolveChain` a quarter turn | `Kind() == BodySheet`; 2 faces; `Edges(Free()).Exactly(6)` — the walk's two seam copies plus the arc each free end sweeps; `Area` `Approximate`, its interval enclosing the analytic Pappus value; `Verify` reads `Validity.Outcome == ValidityUndecided`, a partial turn earning no construction proof |
| T135 | the same walk revolved a FULL turn | still open: `Edges(Free()).Exactly(2)`, the two circles the free ends sweep; `Kind() == BodySheet`; `Volume()` is `ErrNotSolid`; `Verify` reads `ValidityValid` with no `Validity.Diagnostics`, the full-turn construction proof of §13.4 |
| T136 | a rectangle-minus-one-side chain in a sketch that also holds an untouched spline, so every edge reads `TExact == false` | `ErrUnrecordableProfile`, the identical seam refusal the equivalent profile earns, and the document is unchanged |
| T137 | a chain held across a `Params().SetValue` plus `Solve`, and separately a chain whose `Valid` is false | `ErrStaleProfile` for the first, `ErrInvalidProfile` for the second; neither call touches `Document.Bodies()` |
| T138 | a held chain re-ranked by RENAMING one of its entities, with no geometry change and no re-solve | `ch.IsStale()` is false and `ExtrudeChain` succeeds, publishing the same `Area` and `Bounds` as the pre-rename call: the authentication gate matches the held snapshot against the whole fresh `s.Chains()` set by content, never at one index. Shown-to-fail: matching at the held chain's original index turns this `ErrInvalidProfile` |
| T139 | `SweepChain` handed a valid chain and a composite path, and `RevolveChain` handed a chain with BOTH free ends on the resolved axis | `ErrUnsupported` at each call (R34, R22); `errors.Is` holds; the document and every operand are unchanged. From Table D row 12 onward the `SweepChain` leg is the composite path rather than any path, since a one-span straight path builds; from row 14 onward `LoftChain`'s own staged refusal is the curved pair (R36) and sits with T199 rather than here |
| T140 | two `ExtrudeChain` ribbons whose free end edges are a proven-coincident, zero-bound pair, `Stitch`ed | the pair welds under Table J's `Line3` row and the result is a `BodySheet` whose `Edges(Free())` resolves to the remaining 6 edges, not 8 — a ribbon is an ordinary stitch operand, admitted by the existing gates and not by a new one |
| T141 | a chain holding a free-form fragment, `ExtrudeChain` 10 mm `Along` | `Area` is `Approximate` with a strictly positive `Bound`, and its interval encloses the analytic wall area; `ch.Length · h` sits at or below that interval's lower end, never inside it — the underestimate §13.3 refuses to publish. Shown-to-fail: replacing the per-segment sum with `ch.Length · h` published as `Exact` turns the enclosure assertion red |
| T142 | a single line from (z = 0, r = 0) to (z = 4, r = 3), spun a full turn about r = 0 | `BodySheet`, one cone face, one free circle, `Bounds` exactly (0, -3, -3)–(4, 3, 3) mm, and `Area` enclosing 15π mm²; the pole adds no edge or area. Shown-to-fail: an otherwise identical start radius within the axis snap band must still enclose the snapped cone's 15π mm² area, and a snapped RADIAL wall — a meridian at constant z, which sweeps a disk — must enclose its own; dropping either of `walkAxisMoment`'s two snap terms, the start radius or the wall length, turns one of those enclosures red |
| T143 | T142's line spun a quarter turn | one cone face, one pole `Vertex` shared by the two meridian edges, three free edges, and `Area` enclosing 15π/4 mm². Shown-to-fail: zeroing `revolvePayload.sweep`'s width bound turns the area enclosure red |
| T144 | a valid half-circle chain from (z = -5, r = 0) to (z = 5, r = 0), spun a full turn | R22 `ErrUnsupported` names BOTH on-axis free ends; no body is committed |
| T145 | a one-pole cone from (z = 0, r = 0) to (z = 10⁶, r = 3), placed under a 30° rotation | its `Area` value and bound equal the unplaced cone's; one free rim remains; its `Bounds` interval encloses the exact dot products at the rotated rim's x and y extremes. Shown-to-fail: zeroing `extentBoundedAlong`'s final composed bound makes the x enclosure red |
| T146 | two off-axis lines meeting at one interior point on the revolve axis | R33 `ErrDegenerate` names the interior axis junction; no pinched shell is committed |
| T150 | 100×60 mm `Document.Patch`, positive 2 mm | `BodySolid`, 6 faces, `Volume` 12000 mm³ Exact, `Area` 12640 mm² Exact, centroid (50,30,1) mm, `Bounds` (0,0,0)–(100,60,2) mm; source retired and readable |
| T151 | T150's patch, negative and centered 2 mm | each volume is 12000 mm³; z bounds are [-2,0] and [-1,1] mm, centroids at z=-1 and z=0 mm; source normals are unchanged |
| T152 | 100×60 mm surface-extruded prism over 10 mm, negative 5 mm | 10 faces, `Volume` 15000 mm³ Exact, `Area` 9000 mm² Exact, centroid (50,30,5) mm; `Bounds` matches the sheet's |
| T153 | radius-10 mm whole-circle prism sheet over 10 mm: positive 2, negative 2, centered 4 mm | 4 faces each; volume intervals enclose 440π, 360π, 800π mm³; positive bounds reach ±12 mm |
| T154 | §16.2's 4 mm-neck axis-parallel prism sheet over 10 mm, negative 3 mm | R26 `ErrUnsupported` with the crossing-audit diagnostic; no result, source still live and document unchanged |
| T155 | 100×60 mm `Document.Patch` with a radius-10 mm hole, positive 2 mm | 7 faces; volume and area intervals enclose `12000-200π` mm³ and `12640-160π` mm²; box (0,0,0)–(100,60,2) mm |
| T156 | internal exact-generation check: U = 2^53 mm axis-parallel edge displaced by 1 mm | R27 `ErrUnsupported`; the held coordinate cannot equal the true derived coordinate |
| T157 | a straight one-span `Sweep` sheet and a `Loft` sheet | R24 `ErrUnsupported` for both, the message naming no admitted generator; both stay live and `Document.Bodies()` is unchanged |
| T158 | a 10 × 10 mm meridian rectangle at `r ∈ [10, 20]`, `z ∈ [0, 10]` spun a FULL turn as a surface, thickened inward 2 mm | `BodySolid`, 8 faces, one lump whose second shell is the toroidal void the eroded meridian sweeps; `Volume` and `Area` intervals each enclose `1920π` (mm³ and mm²); `Bounds` exactly (−20, 0, −20)–(20, 10, 20) mm; the source sheet is retired and still readable. Shown-to-fail: zeroing `revolvePayload.sweep`'s width bound publishes `Volume` `Exact` at a zero bound, and its interval then excludes `1920π` |
| T159 | a radius-5 mm circle centred at `r = 20` spun a full turn as a surface — a one-face torus sheet — thickened outward 2 mm, and separately centered 4 mm | 2 faces each; `Volume` intervals enclose `960π²` and `1600π²` mm³; the outward result's `Bounds` reaches ±27 mm radially and ±7 mm axially. Shown-to-fail: deleting the region integral's own first-moment bound publishes `Volume` `Exact` and its interval then excludes `960π²` |
| T160 | T158's meridian swept a QUARTER turn as a surface, thickened inward 2 mm | 10 faces — 8 walls and two annular caps of 64 mm² each; `Volume` encloses `480π mm³` and `Area` encloses `480π + 128 mm²`; the source sheet's own `Volume` was `ErrNotSolid` and the result's is a measurement |
| T161 | a radius-5 mm circle centred at `r = 6` thickened outward 2 mm, and separately T158's sheet about an axis tilted 30° in its own plane | R43 `ErrUnsupported` both ways: the first message names the least radius `−1 mm`, the second the axis that is not along a recorded plane axis. No body is committed and `Document.Bodies()` is unchanged |
| T162 | a 40 mm straight `ExtrudeChain` ribbon over 10 mm, thickened 2 mm positive, negative and centered | 6 faces each; `Volume` 800 mm³ `Exact` and `Area` 1000 mm² `Exact` in all three; the three boxes are `y ∈ [−2, 0]`, `[0, 2]` and `[−1, 1]` mm with `x ∈ [0, 40]` and `z ∈ [0, 10]` unchanged. Shown-to-fail: emitting the left copy forward rather than backward winds the section clockwise and the build refuses on its own non-positive area |
| T163 | the ribbon `(0,0)–(40,0)–(40,30)` over 10 mm, thickened 2 mm outward and 2 mm inward | outward: 9 faces and a `Volume` interval enclosing `1400 + 10π mm³`, the quarter disc the corner arc adds; inward: 8 faces and `Volume` exactly `1360 mm³`, the mitred quarter square. Shown-to-fail: deleting the region integral's arc bound turns the outward enclosure red and leaves the inward one green, which pins the π term to the arc rather than to the walk |
| T164 | the ribbon `(0,10)–(0,0)–(4,0)–(4,10)` over 10 mm — a 4 mm neck — thickened inward 3 mm and separately 1 mm | R26 `ErrUnsupported` at 3 mm, the message naming a nonadjacent contact inside the interval and no body committed; at 1 mm it builds with 10 faces and `Volume` exactly `220 mm³` |
| T165 | a 40 mm meridian at `r = 10` spun a full turn as a `RevolveChain` shell — a cylinder of area `800π mm²` — thickened 2 mm outward, 2 mm inward and 4 mm centered | 4 faces each; `Volume` intervals enclose `1760π`, `1440π` and `3200π mm³`; the boxes reach ±12, ±10 and ±12 mm radially with the axial interval unchanged |
| T166 | T165's shell thickened inward 11 mm | R43 `ErrUnsupported`, the message naming the least radius `−1 mm`; no body is committed and the shell stays live |
| T167 | the meridian `(10,0)–(10,20)–(20,20)` spun a full turn as a shell, thickened outward 2 mm | 6 faces; `Volume` encloses `1392π mm³`, the mitred outer corner's own first moment; `Bounds` reaches ±20 mm radially over `z ∈ [0, 20]` mm |
| T168 | a ribbon whose walk sits at `u = 2⁵³ mm`, thickened 1 mm | R27 `ErrUnsupported`; the message names the rounded generated coordinate rather than a contact, so the exact-generation gate is what fired |
| T169 | a chain revolve shell whose free end lies ON the resolved axis — a 10 mm radial meridian from `r = 0` — thickened 2 mm | R43 `ErrUnsupported` naming the least radius `0 mm`; no body is committed, and the shell stays live with its own `Area` |
| T200 | 100×60 mm `Document.Patch`, offset positive 2 mm | `Kind() == BodySheet`; one face; `Area` exactly 6000 mm² and `Exact`, equal to the source's value AND its bound; `Bounds` (0,0,2)–(100,60,2) mm; `Edges(Free()).Exactly(4)`; the SOURCE is still live and reports its own `Area` and `Bounds` unchanged |
| T201 | the same patch, offset negative 2 mm | the box is (0,0,-2)–(100,60,-2) mm and every other T200 assertion holds; source, positive result and negative result are three live bodies of one document, and `Document.Bodies()` reports all three |
| T202 | 100×60 mm surface-extruded prism sheet over 10 mm, offset positive 3 mm | 8 wall faces — four straight, four corner arcs; `Edges(Free()).Exactly(16)`; `Area` `Approximate` over an interval enclosing `3200 + 60π` mm²; `Volume()` is `ErrNotSolid`; `Bounds` is (-3,-3,0)–(103,63,10) mm |
| T203 | radius-10 mm whole-circle prism sheet over 10 mm, offset positive 2 mm and negative 2 mm | one wall face each; `Area` intervals enclose 240π and 160π mm²; the boxes reach ±12 and ±8 mm in x and y and keep z ∈ [0,10] mm. Shown-to-fail: remove the arc perimeter's rational π enclosure and the reading publishes a zero bound on a value no float represents, turning the `Approximate` assertion red |
| T204 | §17.2's 4 mm-wide neck axis-parallel prism sheet over 10 mm, offset negative 3 mm | R38 `ErrUnsupported` naming the crossing audit; no result is registered, the source stays live, and `Document.Bodies()` is unchanged |
| T205 | T202's result offset again by 1 mm, and T203's positive result offset again by 1 mm | R37 for the first — its section carries corner arcs and is no longer the axis-parallel line class — and a radius-13 mm sheet for the second, `Area` enclosing 260π mm² |
| T206 | a 40 mm `ExtrudeChain` ribbon, a full-turn `RevolveChain` shell, a `Body.Patch` result, an all-planar stitched sheet and a solid box | R37 `ErrUnsupported` for each; every receiver stays live and readable, and no document body set changes |
| T207 | internal exact-generation check: an axis-parallel edge at U = 2^53 mm offset by 1 mm | R39 `ErrUnsupported`; the held coordinate cannot equal the true generated one. Shown-to-fail: replace the `rationalFloatError` comparison with a small-residual test and the case is admitted, turning the refusal assertion red |

T152's section is the 100×60 outer rectangle less the 90×50 inner
rectangle. Omitting the inner loop makes its 15000 mm³ volume assertion fail
at 60000 mm³. T153's positive volume differs from the sheet-area product
`400π mm³`; deleting the region integral's area bound makes its independently
bracketed `440π mm³` enclosure fail. T155's volume enclosure fails under
the same deletion; its area enclosure fails when the composed area bound is
deleted separately.

T154's ordered polygon vertices are (0,0), (10,0), (10,8), (20,8),
(20,0), (30,0), (30,20), (20,20), (20,12), (10,12), (10,20), (0,20),
in millimetres. The current `Shell` call on the solid prism with both caps
removed and the same 3 mm inward thickness returns `the rewrite crosses
itself`, so the intended offset refusal is reachable. Deleting the crossing
audit changes T154's required crossing diagnostic. At 0.5 mm inward, the
same neck builds 30 faces and its volume interval encloses `640+2.5π` mm³.
At 1.5 mm inward, it still builds 30 faces and its volume interval encloses
`1800+22.5π` mm³; the contact-event proof admits this safe narrow section.
Deleting T156's exact-generation check changes its required rounded-coordinate
diagnostic to the contact-audit refusal. T157 also asserts that the source
bodies remain live and retain their original measurements.

T158's and T160's `1920π` and `480π` are Pappus's second theorem over the
annulus: `∫ρ dA` is `1500` for the outer rectangle and `540` for the eroded
one, and the sweep multiplies their difference. T159's `960π²` is the same
theorem over a circular meridian, `2π · 20 · π(7² − 5²)`, and the outward
result's `Area` carries the identical figure in mm² — the two agree because
this fixture's own radii make them agree, never by a rule. T164's 1 mm result
is the U-shaped band `[0,4] × [0,10]` less `[1,3] × [1,10]`, area 22 mm²,
which is also what pins the assembled section's counter-clockwise winding: the
same walk emitted the other way round would integrate to `−22`. T167's `696`
first moment is the band `[10,12] × [0,18]` plus `[10,20] × [18,20]`, the
shape the outer corner's miter produces; an arc there instead would change the
figure, so the fixture also pins which corner rule the negative-turn corner
took. T161's and T166's least-radius figures are exact rationals the gate
prints, so each assertion reads the number rather than only the sentinel.

| T170 | T1's 100×60 rectangle surface-extruded 10 mm `Along`, `Trim`med `KeepOutside` by a solid extruded from the square (40,−10)–(60,70) over z ∈ [−5, 15] — a tool spanning the sheet axially and cutting its bottom and top walls at x = 40 and x = 60 | `Kind() == BodySheet`; 2 lumps, 3 faces each; `Edges(Free()).Exactly(16)`; `Bounds` equal to the untrimmed sheet's, value and bound, since both surviving runs still reach every extreme; `Volume()` is `ErrNotSolid`; `Area` is `Approximate` with a strictly positive `Bound`, and its interval encloses the exact 2800 mm² taken over `math/big.Rat` from the two operands' own recorded floats, never a second float answer. Shown-to-fail: forcing the result's `sectionDelta` to zero publishes `Exact` at a zero bound and turns both the positive-`Bound` and the enclosure assertions red — the leg that proves `docs/surface-intersection-design.md` §7's `δ_cut` is charged rather than assumed away |
| T171 | T170's pair, `KeepInside` | 2 lumps, 1 face each; `Edges(Free()).Exactly(8)`; `Area` `Approximate` over an interval enclosing the exact 400 mm²; the two surviving walls' own `NormalAt` values equal the untrimmed sheet's at the same points, bit for bit — a trim moves no wall |
| T172 | T170's sheet against a tool whose section lies wholly outside it, and separately against one whose section wholly contains it | `ErrDegenerate` (R30) both ways: the first keeps every fragment under `KeepOutside`, the second none, and neither names a trimmed body. `Document.Bodies()` and both operands are unchanged |
| T173 | T170's trimmed result `Trim`med again by a second tool, and the same second tool applied to the untrimmed sheet | `ErrUnsupported` (R29) for the first, message naming the receiver's own section displacement rather than its payload class; `NoError` for the second — the refusal is the displacement's doing, not the tool's (`docs/surface-intersection-design.md` §2.1 S7, §5) |
| T174 | T170's sheet against a co-directional tool placed by `r3.RotationAround` at several counts from `docs/prism-boolean-design.md` §3.3's inexact set, and separately against a tool whose section sits on a plane parallel to the sheet's 5 mm away | `ErrUnsupported` (R29) in every case, never an admission: S4 reads the stored `r3.Vec` floats with Go `==` and a dot product against the literal `0.0`, so a pair one ulp off co-directional refuses and a parallel-plane pair refuses on S7's identity re-expression. The same model built through a hand-constructed `r3.FromBasis` placement on the sheet's own frame is admitted, which pins the refusal to the comparison rather than to the option |
| T175 | a 100×60 solid block over z ∈ [0, 10], `Split` by an `ExtrudeChain` ribbon from the single line (−10,30)–(110,30) over z ∈ [−5, 15] | 2 bodies, each `Kind() == BodySolid` and `IsSolid()`; each `Volume` `Approximate` with a strictly positive `Bound`; the two intervals' sum encloses the target's own exact 60000 mm³; the target and the tool are both retired and `Document.Bodies()` holds the two pieces in `sketch`'s own cell order, reproduced across a replay |
| T176 | the same block `Split` by a surface-extruded circle of radius 20 centred at (50,30) over z ∈ [−5, 15] — a closed-section tool | 2 bodies; the disk piece's `Volume` interval encloses 4000π mm³ and the remainder's encloses 60000 − 4000π mm³; the two intervals' sum encloses 60000 mm³ |
| T177 | the same block `Split` by a ribbon from the line (−10,30)–(40,30) — a tool that enters the block's footprint and stops inside it | `ErrDegenerate` (R30); the document and both operands are unchanged. This is the fixture that pins `sketch`'s own pruning: the inside stub bounds no published cell, the target's section is returned whole, and no split exists to return |
| T178 | a sketch holding the line (0,0)–(100,0) and a second line at x = 40 crossing it; the left fragment `ExtrudeChain`d 10 mm `Along` — a 400 mm² ribbon — then `Extend`ed on its x = 40 sweep edge against a solid extruded from the square (70,−10)–(90,10) over a spanning interval | no error; `Area` is `Approximate` with a strictly positive `Bound` over an interval enclosing 700 mm²; `Bounds` now reaches x = 70; the ribbon's other recorded bound is byte-identical to its own pre-extend record (`Point2` fields, not just area) — `docs/surface-intersection-design.md` §3.1's full-domain recreation introduces no coordinate of its own. The same call on a CLOSED-footprint sheet is `ErrUnsupported` (RS12), the receiver having no section end to lengthen |
| T179 | T178's fixture with a tool whose section crosses the carrier only beyond x = 100 — past the recorded entity's own natural domain | `ErrUnsupported` (R29, RS4), message naming the carrier's own domain; the document is unchanged. The same tool moved inside the domain succeeds, so the refusal is the domain's doing and not the tool's |
| T180 | a meridian segment from (r = 5, z = 0) to (r = 5, z = 20) revolved a full turn as a surface — a cylindrical sheet of area 200π mm² — `Trim`med `KeepOutside` by a solid revolve of the rectangle meridian r ∈ [0, 8], z ∈ [5, 10] about the identical axis on the identical frame | `Kind() == BodySheet`; 2 lumps, 1 face each; `Edges(Free()).Exactly(4)`, the two circles each surviving ribbon's own ends sweep (Table G's full-revolution row); `Area` `Approximate` over an interval enclosing 150π mm²; `Volume()` is `ErrNotSolid`; `Tessellate` returns `ErrUnsupported` until increment 4, as every revolve sheet's mesh does. Shown-to-fail: replacing S4's revolve arm with an angle comparison admits a pair whose axes differ in the last ulp and turns the enclosure assertion red |
| T181 | the revolve sheet of T180 against a co-directional prism solid sharing its axis direction, and separately a revolve-built cylinder of the same radius against that same prism | `ErrUnsupported` (R29) both ways: S1 reads the two payload families and refuses a mixed pair, and it reads the payload rather than the shape, so a cylinder authored as a revolve refuses where the same cylinder authored as an extruded circle is admitted. The message names the two generators, not the payload class |

| T190 | T130's single 40 mm line chain, `SweepChain` along a one-span `LineTo` path from the sketch plane's origin 10 mm along its positive normal | `Kind() == BodySheet`, `IsSolid() == false`, `Volume()` is `ErrNotSolid`; exactly 1 face; `Edges(Free()).Exactly(4)`; `Bounds` is the 40×0×10 slab, `Exact`; and `Area` is bit-identical to T130's own `ExtrudeChain` reading, value, `Exactness` and `Bound` alike — the equality is the assertion, since the two calls denote one ribbon and reach it by two different height derivations |
| T191 | T131's open line/arc/line walk, swept along the same one-span path | 3 faces; `Edges(Free()).Exactly(8)`, the two junction sweep edges matching nothing under `Free()`; the arc wall's `Area` is `Approximate` while both line walls read `Exact`; the body's `Area` equals the three walls' `boundedAdd` sum, value and bound |
| T192 | T190's chain on a sketch plane whose positive normal is not a signed coordinate vector, swept `7√2` mm along that normal | `Area` is `Approximate` with a strictly positive `Bound` whose interval encloses the analytic `280√2` mm²; each of the walk's two SWEEP edges reads `Length` `Approximate` with a strictly positive `Bound` whose interval encloses the analytic `7√2` mm. Shown-to-fail: dropping `validateStraightSweepPath`'s composed height bound — the square root's committed error against the exact rational squared length, and the per-component departure of the held sweep vector from the exact path tangent — turns the two sweep-edge assertions red. The edge reading is what ISOLATES that bound: a wall's area is a `boundedMul` of two scalars and carries the product's own rounding whatever the height bound says, while a sweep edge's length IS the height and carries nothing else |
| T193 | T190's chain against a path whose start lies off the sketch plane, and separately against one whose initial tangent opposes the plane's positive normal | `ErrDegenerate` both ways (`docs/sweep-design.md` Table SC row SC3); `Document.Bodies()` is unchanged |
| T194 | T190's chain against a two-span tangent line/arc path, and separately against a one-span `ArcThrough` path | `ErrUnsupported` both ways (R34), the message naming the composite join and the arc reduction respectively; the document is unchanged. Shown-to-fail: removing the span-count gate reaches the analytic reduction with a path it has no height for |
| T195 | `SweepChain` handed a chain held across a `Params().SetValue` plus `Solve`, and separately a chain whose `Valid` is false, each with a path that would otherwise refuse at SC3 | `ErrStaleProfile` for the first and `ErrInvalidProfile` for the second, never the path refusal — the seam gate runs first (`docs/sweep-design.md` Table SC); neither call touches `Document.Bodies()` |
| T196 | two 40 mm line chains on parallel planes 10 mm apart, the second directly above the first, `LoftChain` | `Kind() == BodySheet`; 2 faces, `side(0,0,0)` and `side(0,0,1)`; `Edges(Free()).Exactly(4)` — the two walk rims and the two end rungs; `Area` 400 mm², `Approximate` by `docs/loft-design.md` §8's constant rule; `Bounds` (0,0,0)–(40,0,10) `Exact` at a zero bound, every station being PINNED under `r3.Identity()`; `Volume()` is `ErrNotSolid`. Every wall's `NormalAt` equals the `ExtrudeChain` wall's over the SAME recorded segment, which is what pins §16.2's stated `T × N0` side to Table G rather than to this build's own winding |
| T197 | T196's pair with the second chain's plane tilted about the first walk's own direction, and separately with the second plane 10 mm BELOW the first; then the below pair with its two arguments swapped | `ErrUnsupported` (R35) for the first two, the message naming the non-parallel plane pair and the non-positive plane offset respectively; the swap succeeds, which pins the refusal to the argument ORDER rather than to the pair. Shown-to-fail: dropping §16.2's negative-side arm turns the below refusal red, and the ribbon it then builds publishes every wall normal as the NEGATION of the chain prism's over the same recorded segment — measured at (20, 0, −5): (0, 1, 0) against `ExtrudeChain`'s (0, −1, 0) |
| T198 | an open rectangle walk against its own mirrored walk on a parallel plane — a correspondence whose two long rungs meet at the middle of the ribbon — and separately two three-segment walks whose published directions run OPPOSITE ways | `ErrDegenerate` for the first, the crossing audit naming two triangles that share no recorded vertex yet make contact, which is the proof that audit transfers to an open ribbon with the cap triangles absent; the second BUILDS, twisted but not self-crossing, since an opposed pair is not necessarily a crossing one (`docs/loft-design.md` §16.1). The document is unchanged after the refusal |
| T199 | a three-segment walk pair against a two-segment one, a same-kind `ArcSeg` pair, and the same chain handed as both arguments | `ErrUnsupported` naming the segment-count mismatch (SL3), `ErrUnsupported` naming the `LineSeg`-pair-only reach (SL7, R36), and `ErrDegenerate` naming the shared geometric plane (S5); the document is unchanged. `WithLoftAlignment` needs no row: it does not implement `ChainLoftOption`, so the compiler refuses it (§16.3) |

T190's equality against T130 is the stronger assertion of the two available:
either reading alone could be right for the wrong reason, while the two agreeing
pins the chain sweep's own height derivation to the extent vocabulary's. T190
also places its ribbon by a translation and asserts the area unmoved and the box
translated, which is the only reading that exercises the chain sweep payload's
own replay. T192 is the only row in this group whose fixture needs a
non-axis-aligned frame, which is what makes its height bound positive at all —
on an axis-aligned frame every term of that bound is exactly zero and T190
asserts the `Exact` reading instead. Its plane is the one whose orthonormalized
`V` carries two bit-identical components, so the plane's own `U × V` is exactly
`(0, −c, c)` and the path direction `(0, −7, 7)` is exactly codirectional with
it over rationals; any other tilt fails the initial-tangent gate before the
bound can be read.

| T200 | a four-station `LoftChain` ribbon over a folded walk pair, then T196's ribbon `Placed` under a rotation about an axis neither coordinate-aligned nor origin-centred | the multi-cell build has `2n` faces, `Edges(Free()).Exactly(2n+2)`, and exactly `2n−1` interior edges — `n` diagonals plus `n−1` interior rungs — with its `Area` interval enclosing the walk length times the plane gap; the placed copy's `Area` agrees with the unplaced one inside the placed bound, while its `Bounds` turns `Approximate` over a strictly positive `Bound`. The two placed readings together are what show the payload's replay charges its own `delta` rather than reproducing the unplaced build |

T196's normal equality against `ExtrudeChain` is the stronger of the two
assertions available: either reading alone could be right for the wrong reason,
while the two agreeing pin a chain loft's stated side to the landed Table G
rule. T198's first leg is the only fixture in this group that reaches §6's
audit at all, and it is what establishes the audit's transfer to a capless
triangle set; its second leg is the counterexample that keeps §16.1 from
claiming more than the audit proves. No row in this group pins a bound to a
literal: T196 asserts an exact zero because every station is pinned under the
identity motion, and T200 asserts only that the placed bound is strictly
positive.

`.github/test-shards.txt` gains a row for every root-package test each
increment adds, and `go test . -run '^TestCIWorkflowRaceShardsCoverEveryPackage$'`
runs before any push that changes a root test name.

## 16. Thicken

### 16.1 Entry point, side and receivers

```go
func (b *Body) Thicken(ctx context.Context, thickness units.Value,
    opts ...ThickenOption) (*Body, error)

type ThickenOption interface{ thickenOption() }
type ThickenSide int
const (
    ThickenPositive ThickenSide = iota // default: along each source face's positive normal
    ThickenNegative                    // opposite the positive normal
    ThickenCentered                    // half the magnitude on each side
)
func WithThickenSide(side ThickenSide) ThickenOption
```

`thickness` is a strictly positive `units.Value` of kind Length. Its sign never
chooses a side. `WithThickenSide` is a sealed option, admits exactly one of the
three values above, and the last occurrence wins as `WithShellSense` does.
An unknown or nil option and a nil context are `ErrDegenerate`. A cancelled
context returns `ctx.Err()` unchanged before commit. The default positive
side makes a call without options deterministic; the caller can name the
negative or centered side without first reversing a sheet.

The source sheet's positive side is §2.3's built orientation. A
`Document.Patch` sheet's positive side is its sketch plane's normal. A
surface-extruded, profile-fed prism sheet's positive side is the profile region's
exterior on EVERY loop wall, and a profile-fed revolve sheet's is the meridian
region's exterior on every swept wall. A chain-fed sheet's is Table G's
`T × N` — the recorded walk's right-hand side — on the ribbon's wall and on
the chain shell's swept wall alike. The sheet remains readable but is retired on
success; a new `BodySolid` is registered atomically under a new producer
identity (§6). A refusal retires nothing. A `Document.Thicken` alternative
would leave the owning body's liveness and consuming semantics implicit, so
the entry point is on `*Body` as `Shell` is.

Five payload shapes build, and each one's own subsection states its class,
its construction and its readings:

| Sheet family and payload | Result at the call | Deciding record or missing proof |
|---|---|---|
| `Document.Patch`, `patchPayload` | admitted (§16.2) | one recorded profile accepted by `evalPrismContext`, including holes; extrude it over §16.2's signed normal interval |
| profile-fed prism sheet, `prismPayload` with `surfaceResult == true` | admitted (§16.2) | one outer loop, no holes; either all `LineSeg` walks parallel to recorded U or V, or one whole `CircleSeg`; `sectionDelta == 0` |
| profile-fed revolve sheet, `revolvePayload` with `surfaceResult == true` | admitted (§16.5) | §16.2's section class again, plus an axis stated exactly along a recorded plane axis and a proven strictly positive radius over the whole offset interval |
| chain ribbon, `chainPayload` | admitted (§16.6) | one recorded open walk of axis-parallel `LineSeg`s with right-angle interior corners; `sectionDelta == 0` |
| chain revolve shell, `chainRevolvePayload` | admitted (§16.7) | §16.6's walk class and §16.5's axis class together |
| sweep sheet, `sweepPayload`, including a straight span | R24 (§16.8) | the section is constant, but the role restoration and payload class its result must publish live in `sweep.go`, and a composite path needs an offset proof at each span join |
| loft sheet, `loftPayload` | R24 (§16.8) | paired wall stations have no single constant section, and a curved pairing's own `sectionDelta` is nonzero |
| stitched sheet, `stitchPayload` | R24 (§16.8) | welded faces have no one recorded generator |
| `Body.Patch` or `Unstitch` sheet | R24 (§16.8) | its held face set is not one recorded profile in `patchPayload` |

A `Body.Patch` result, an `Unstitch` result, a stitched sheet and a sweep or
loft sheet are R24, even when one face happens to be planar or one sweep
happens to be straight. Their payloads do not give this arm the same recorded
generator and proof, and §16.8 states what each one is still missing. These
cases refuse at the call, never after publishing a solid. The rejected
alternative is admitting every sheet with a generic surface offset: neither a
surface intersection nor a bounded general offset exists in this evaluator,
and every family this section does admit reaches its result through the
section offset and the closed-form section assembly instead.

### 16.2 Construction and refusal

For a patch, set the signed prism levels to `[0,t]` (positive), `[-t,0]`
(negative), or `[-t/2,t/2]` (centered) in the recorded patch frame. Keep
the patch's profile, frame and accumulated placement. Pass the millimetre
conversion's own bound through the computed level's `z0Delta`/`z1Delta`,
including the division and negation for centered or negative, then run
`evalPrismContext`. A planar face translated along its fixed normal cannot
self-intersect through the interval; the authenticated simple profile and a
proven positive interval supply that proof. A collapsed or undecidable held
interval is R27. `Body.Patch`'s extra face set is not this construction.

For a profile-fed prism sheet, write `P` for its recorded hole-free section and
`h = z1-z0 > 0` for its recorded sweep. Positive requests `Q+ = P ⊕ t`;
negative requests `Q- = P ⊖ t`; centered requests both `P ⊕ t/2` and
`P ⊖ t/2`. The result's section is the outer region less the inner region:
`Q+ ∖ P`, `P ∖ Q-`, or `Q+ ∖ Q-`, respectively, where the inner loop is
reversed into the hole walk. Its axial interval and both end displacements
are the source prism's unchanged ones. A convex outside corner grows a
radius-`t` arc and a concave inside corner miters, as `docs/modify-design.md`
§8 states. Centered thickness uses radius `t/2` on each side. A face is
never shifted by changing the source prism's axial levels.

`shell_offset.go`'s `offsetProfile` supplies each requested closed section;
`auditOffsetSectionBudget` checks each one. Its `reverseLoopRecordContext`
and `shell.go`'s `evalTubeContext` already form and build the one-sided,
hole-free annulus. A centered call forms the analogous annulus from TWO
offsets and uses `evalPrismContext`. `shell_cup.go`'s calls to
`prism_build.go`'s `buildLoopSidesAs`, its `renameCavityRoles` and bounded region-prism
measurements show how to assemble the skins, but `evalCupContext` itself
adds a kept cap and a floor at a second axial level: calling it would make a
cup, not this through-height wall. A holed prism sheet would produce one
outer band plus one band per source hole, `1+k` disconnected pieces with no
floor to join them (`docs/modify-design.md` B4). It stays R24 until a
multi-region payload holds those pieces.

The existing offset audit checks an OFFSET section internally. It does not
alone prove that the source and offset loops are disjoint and strictly
nested. Before building an annulus, run the same exact line/arc crossing and
containment primitives over the outer and reversed inner loops together.
Also certify the entire offset interval `0 < τ ≤ t` (or `t/2` on each side):
an endpoint that looks simple cannot prove an earlier offset did not pinch
and change which boundary the per-feature construction denotes. On the
admitted axis-parallel line class, moving line coordinates are affine in
`τ` and inserted corner-circle radii equal `τ`. Exclude the adjacent joins
the offset construction prescribes; isolate every OTHER positive line-line,
line-circle and circle-circle contact parameter and every zero-length
walk parameter with exact rational polynomial comparisons; refuse if one
lies in the requested interval or its order against the endpoint is
undecided. A whole circle has only its radius-zero event, settled by
its exact endpoint radius. Never sample intermediate offsets or infer their
validity from the final section alone.

**This interval scan is stated once and every offset arm runs it**, over its
own ring of moving pieces in loop order: §16.5's meridian annulus, §16.6's
assembled ribbon boundary, §16.7's meridian ribbon and §17.2's offset sheet
all present one closed ring whose pieces are affine in `τ`, so the adjacency
exclusion is "consecutive in the ring" for every one of them and no arm states
a second rule. What each arm adds beyond it, the arm's own subsection states.
Any crossing, tangency, shared boundary point, dropped walk, failed miter,
wrong orientation, lost nesting or undecided comparison is R26. In
particular, a 4 mm-wide neck eroded by 3 mm makes its two offset walls
cross, even though every source segment is valid. A scale-anchored contact
floor may refuse more inputs; a gap above that floor is never the proof of
separation. Only the closed-form segment-pair and nesting decisions on
decad's OWN synthesized section permit construction. No residual or
chorded mesh admits the annulus. A topology-changing offset can still
denote a solid after trimming, so R26 is `ErrUnsupported`, not a claim that
the caller's positive thickness is malformed.

Every arm but the patch has an exact-generation gate before either
offset is admitted. The thickness must convert to millimetres with zero
displacement, every source line endpoint must resolve with zero bound, and
every generated plane-local endpoint, center and radius
must equal its closed-form value as a binary rational. For the axis-parallel
line class, the U/V normal is selected by the nonzero coordinate directly;
the miter is solved over the exact dyadic input, and a convex arc takes the
stated corner and radius. A whole circle takes its stated center and the
exact sum or difference of its radius and thickness. Compare each held
result with that exact value using `dyadic`/`rationalFloatError`; refuse R27
when one is not representable or the required half-thickness vanishes.
Never infer exactness from a small residual or from `math.Hypot`, `Atan2`,
`Sin` or `Cos`: none has an accuracy contract here. This gate keeps
`prismPayload.sectionDelta == 0` truthful, so `evalPrismContext`'s existing
moment and extent proofs apply, and it is what lets §16.5's and §16.7's
results hold a revolve record whose own section carries no displacement
term at all. A future increment may carry a proven
nonzero generated-section displacement through EVERY downstream reading
instead of refusing it. The rejected alternative is feeding rounded offset
coordinates to `evalPrismContext` with a zero `sectionDelta` and publishing
measurements of a more precise wall than the record denotes.

Gate order is fixed: check context, liveness, owned options and magnitude;
select the receiver family; convert and, outside the patch arm, certify
thickness; construct the requested offsets, or assemble the requested closed
ring, and certify the generated values; run `auditOffsetSectionBudget` on each
offset; prove the whole offset interval, the radius where an axis is in play,
and the outer/inner boundary relation; build the prism or the revolve; recheck
liveness and context; commit. A construction refusal is returned unchanged
before any body is registered. This order lets T154 observe the shared
crossing diagnostic before the interval proof could report the same pinch less
specifically.

### 16.3 The patch and prism results, and their downstream readings

The patch and prism arms return a new one-lump, watertight `BodySolid`: `Kind()` is
`BodySolid`, `IsSolid()` is true, and every edge has two adjacent faces.
The patch arm has two planar caps plus one side face per coalesced recorded
boundary segment, across the outer loop and every hole. The prism arm has
one wall face per segment of EACH result loop, plus two planar annular rims:
`N_outer + N_inner + 2` faces. Inserted corner arcs count as segments. The
result uses the new producer's `side(i,j)`, `capStart` and `capEnd` roles,
all indexing the RESULT record; no source face role is inherited. Both arms
hold an ordinary solid `prismPayload`, so its existing placement, tessellation,
surveys, clearance, boolean and stop dispatches read the result by payload
class, under their existing proof gates.

The patch arm's `Volume = A_P·t`, `Area = 2A_P + L_P·t`, and centroid is the
profile centroid lifted to the signed interval midpoint. Its `Bounds` is
the prism's swept profile box. The prism arm's `Volume = (A_outer -
A_inner)·h`; `Area = 2(A_outer - A_inner) + (L_outer + L_inner)·h`;
and its centroid is the annular section's first moment divided by its area,
lifted to the source prism's interval midpoint. `Bounds` is the OUTER
section's prism box, since the inner region is strictly contained. A
circle of radius 10 mm thickened outward by 2 mm over 10 mm height has
volume `440π mm³`, whereas its sheet area times 2 mm is `400π mm³`.

`evalPrismContext`'s region integrals, segment lengths, `boundedAdd`/
`boundedSub`/`boundedMul`/`boundedQuotient`, point lift and extent readers
publish `Volume`, `Area`, `Centroid` and `Bounds` with their existing
outward bounds, including every source axial displacement and placement
rounding. A reading is `Exact` only when its complete value is exactly
representable with zero bound; an arc or whole circle's π contribution is
`Approximate` with the rational π enclosure, never an unbounded float
evaluation. No raw arithmetic may follow a proven bound without charging
its own rounding. A face's `Area`, `NormalAt`, edge lengths and vertex
positions follow the same prism builder's proof path. The rejected
`sheet Area × thickness` volume formula misses curvature, as the circle
fixture shows.

### 16.4 Reverse Normal and tapered extrude

`ThickenPositive` and `ThickenNegative` refer to the orientation §2.3
already publishes; `ThickenCentered` is independent of that orientation.
The side option names where material grows, not the result skin's outward
normal: the source wall can become the new solid's inner wall, wound in the
opposite sense.
Reverse Normal therefore does not gate this feature. When designed, Reverse
Normal should consume a sheet and rebuild it with every face normal and
loop/coedge traversal reversed, preserving positions, `Area` and `Bounds`
and their bounds; it must leave a solid receiver unsupported. It needs a
payload replay rule for each sheet family and a fresh proof that every
downstream orientation reader sees the reversal. §1.4 records why no increment
takes it up: no shipped operation's result turns on a side its caller cannot
already name.

This fixed-distance section construction does not land tapered extrude.
`WithTaper` needs an offset for EVERY axial level and a proof that no two
intermediate sections or their ruled walls cross, including a proof for
computed taper angles and their bounds. Certifying one endpoint section with
`offsetProfile` proves none of those statements. `docs/evaluator-design.md`
§12's tapered-extrude refusal therefore remains in force.

### 16.5 The revolve sheet

**A surface of revolution's own normal lies in its meridian half-plane, so
offsetting the meridian IS offsetting the surface.** There is no second,
three-dimensional offset to state: the revolve of `P ⊕ t` is the outward
offset of the revolve of `P`, point for point, and §16.2's section offset is
the whole of the construction. What the sweep adds is one proof the section
offset cannot see — that the offset family never reaches the axis — and §16.1
names it as the radial-axis half of this arm's missing proof.

**The admitted sub-class**, as an exact predicate. The receiver is a live
`BodySheet` holding a `revolvePayload` with `surfaceResult == true` and no
`profile.Holes`, and every one of the following holds:

- the payload's `axisFrame` carries `aUBound`, `aVBound`, `dUBound` and
  `dVBound` all exactly zero, and `(dU, dV)` is exactly `(1,0)`, `(-1,0)`,
  `(0,1)` or `(0,-1)`;
- the meridian is §16.2's own section class — every coalesced walk a `LineSeg`
  parallel to recorded U or V with right-angle corners and bit-equal adjacent
  joins, or one whole CCW `CircleSeg`;
- the thickness converts to millimetres with zero displacement, and a centered
  call's half-thickness is itself exactly representable;
- every offset the request needs passes §16.2's exact-generation gate, its own
  `auditOffsetSectionBudget`, the crossing and nesting audits over the
  assembled annulus, and §16.2's whole-interval scan;
- the whole swept family's least radius from the axis is proven strictly
  positive.

**The construction** takes §16.2's offsets unchanged: `Q+ = P ⊕ t` positive,
`Q- = P ⊖ t` negative, both at `t/2` centered, and the result's section is
`Q+ ∖ P`, `P ∖ Q-` or `Q+ ∖ Q-` with the inner loop reversed into the hole walk
by `reverseLoopRecordContext` — the identical annulus `evalTubeContext`
assembles for a prism. That section then goes to `evalRevolveContextWork` under
a `revolvePayload` whose `phi0`, `phi1`, `full`, `den`, `frame` and `xform` are
the sheet's own, unchanged, and whose `surfaceResult` is false. No level, no
angle and no placement moves, so a `Thicken` of a quarter-turn sheet spins
exactly the quarter turn the sheet spun.

**The axis is re-resolved against the annulus rather than inherited.** An
`axisFrame` carries four region integrals (`regionSnapAllow`), a radial
admission charge and an axial envelope, and every one of them is proven over
the region it was resolved for. The region changed, so `resolveAxisSide` runs
again over the annulus, handed the sheet's own anchor and direction back
unchanged — the axis itself is never re-derived, only its region-dependent
charges. A side it cannot decide is R43; the radial proof below means a
decided side is always the sheet's own.

**The radial proof.** Write `ρ(u, v) = (v − aV)·dU − (u − aU)·dV`, the
identical formula `axisFrame.toAxis` evaluates. On the admitted axis class
every product in it is by `0` or `±1`, so the coordinate the gate decides on is
the coordinate the built wall carries, with no rounding between them. `ρ` is
affine in `(u, v)`, so its minimum over an exact box sits at a corner, and
§16.2's interval scan already builds one exact box per moving piece enclosing
that piece over every `τ` in `[0, t]`. Four exact rational comparisons per box
decide the least radius the whole swept family reaches; refuse R43 unless every
one of them is strictly positive, naming the least radius found. At `τ = 0` the
boxes ARE the source meridian's own walks, so one scan certifies the source
section, the requested offset and every offset between them — the same
whole-interval statement §16.2 makes about contact, over the same pieces, with
no second enumeration.

**Why that settles the fold §16.1 names.** A swept offset folds in exactly two
directions and each has its own proof. In the MERIDIAN direction it folds where
a circular walk's offset radius reaches zero — `offsetRadius` refuses that as a
dropped feature (S11a) — and where two non-adjacent walks meet, which §16.2's
interval scan refuses. In the HOOP direction the principal radius of curvature
at a point is the distance along that point's own normal to the axis, so the
offset folds exactly where the offset reaches `ρ = 0`; the radial proof IS the
hoop curvature proof, stated as a positive-radius comparison rather than as a
curvature. No transcendental enters either half, which is what lets both be
decided rather than measured.

**Refusals, each typed and named.** A receiver outside the sub-class above —
`surfaceResult == false`, a holed meridian, or a meridian outside §16.2's
section class — is R24. A thickness or a generated coordinate that does not
land exactly is R27. A dropped feature, a failed join, a crossing, a contact,
lost nesting or an undecided audit is R26. An axis not stated exactly along a
recorded plane axis, a least radius not proven strictly positive, and an
undecided axis side for the annulus are R43.

**What the result publishes.** `Kind()` is `BodySolid` and `IsSolid()` is true.

| Reading | Value and where its bound comes from |
|---|---|
| faces | one wall per segment of EACH result loop, plus two planar cap faces when the source sweep is partial and none when it is full. An inserted corner arc counts as a segment and sweeps a `Torus`; a line parallel to the axis a `Cylinder` and one perpendicular a `Plane` |
| lumps and shells | one lump. A full revolution puts the hole loop's own closed wall set in a second, VOID shell of that lump — the toroidal cavity the annulus sweeps — while a partial sweep's two caps join both loops into one shell |
| `Volume` | `boundedMul(q, sweep)`, Pappus's second theorem over the annulus's own `∫ρ dA`, plus `revolveAxisAdmitVolumeCharge`. Its bound composes the region integral's own bound, `regionSnapAllow.first`, the sweep's own `sweepDenotation` width, and `boundedMul`'s product charge. The snap term is zero wherever the annulus's least radius clears the re-resolved frame's own `snapTol`, and positive where it does not — the radial gate proves that radius STRICTLY positive, which is a weaker statement than clearing the snap band, so the charge is carried rather than argued away |
| `Area` | `boundedAdd` over each wall's `boundedMul(walkAxisMoment, sweep)`, plus both cap faces' own area and bound for a partial sweep. A circular wall's moment carries `circularAxisMomentTotal`'s rational enclosure and a straight wall's its mean-radius composition |
| `Centroid` | `boundedDiv(mzr, q)` along the axis, plus the in-plane radial term for a partial sweep, `math.Min`-ed against `revolveCentroidGeometryBound` and then charged `revolveAxisAdmitBandCharge` |
| `Bounds` | `revolveBoundsContext` verbatim, charging the frame lift and the accumulated placement the sheet's own extent reading already charged |

No bound in that table is new, because no geometry is: the annulus is an
ordinary recorded section and `evalRevolveContext` builds it by the path it
already proves. The readings that change are `Volume` and `Centroid`, which
answered `ErrNotSolid` on the sheet and answer a measurement here. A 10 mm ×
10 mm meridian rectangle at `r ∈ [10, 20]` spun a full turn as a surface and
thickened inward 2 mm has `Volume` enclosing `1920π mm³`; the same sheet is
T158's fixture.

### 16.6 The chain ribbon

**An open walk has no interior to erode, so a ribbon needs no open-walk
offset.** §16.1 names `offsetProfile`'s closed-`ProfileRecord` contract as what
a ribbon lacks, and the reduction that lands is to stop asking for an offset at
all: what a thickened ribbon sweeps is ONE closed section, assembled in closed
form from pieces the same corner rule already decides. The section is the
walk's right-hand copy walked forward, one cap line at the walk's far end, the
left-hand copy walked backward, and one cap line at its near end.

**The admitted sub-class**, as an exact predicate. The receiver is a live
`BodySheet` holding a `chainPayload` with exactly one `ChainRecord`,
`sectionDelta == 0` and a proven positive sweep height, whose coalesced walks
are every one an axis-parallel `LineSeg` with zero endpoint bounds, whose
interior junctions agree bit for bit, and whose interior corners are all right
angles. The thickness gate is §16.2's, unchanged.

**Which copy is which side.** `ThickenPositive` offsets the RIGHT-hand copy and
leaves the left copy the recorded walk, `ThickenNegative` swaps them, and
`ThickenCentered` offsets both by `t/2`. The right-hand side is Table G's
`T × N`, which is the ribbon's own positive side, so the option means on a
ribbon what it means on every other receiver. That emission order also makes
the assembled loop counter-clockwise by construction rather than by a check:
for a walk running `+U` in a right-handed frame the right copy sits at `−V` and
the left at `+V`, and the four corners in emission order are `(start, −)`,
`(end, −)`, `(end, +)`, `(start, +)`.

**The corner rule is the closed one, per corner.** An interior corner inserts a
radius-`τ` arc about the corner point on the side the walk turns AWAY from and
miters on the side it turns toward — `sign(cross) == −s` of
`docs/modify-design.md` §7, with `s = −1` naming the right copy and `s = +1`
the left. That rule never consulted a loop's closure, so an open walk needs no
restatement of it; what the closed case reads at a wraparound junction, the
open case simply has no junction for.

**The whole-interval proof.** Every coordinate of every assembled piece is a
recorded walk coordinate plus an INTEGER multiple of `τ`: an offset endpoint
takes one axis normal, a miter the sum of two perpendicular ones, a cap's two
ends one normal each, and a corner arc's radius IS `τ`. The ring is therefore
affine in `τ` and §16.2's scan reads it with no change, "adjacent" meaning
consecutive in the ring. One reading of it is specific to a ribbon and is
stated here: at `τ = 0` the ring collapses onto the doubled walk and every
opposite pair coincides, which is a root of the pair's own contact polynomial
AT zero and nowhere else — the scan counts roots in `(0, t]` and tests each
line-line event root for `root > 0`, so the degenerate start is excluded by the
interval the proof is stated over, while any LATER coincidence is a genuine
pinch and refuses. The zero-length leg behaves the same way: a cap's own length
is `(k_R + k_L)·τ` and vanishes only at zero, while a walk the corner joins
consume vanishes inside the interval and refuses there.

**The exact-generation gate.** Each moving point is evaluated at `τ = t` over
`big.Rat` and the held `float64` is required to equal it exactly through
`rationalFloatError`, corner centres included; a coordinate that does not land
exactly is R27, and so is a generated piece of zero length. The gate reads the
construction's OWN output rather than comparing two constructions, because
there is no second construction to compare against — which is also why this arm
runs no `auditOffsetSectionBudget`: that audit compares an offset section to
its source section, and a ribbon has no source section.

**The endpoint audit.** `crossingAuditBudget` over the assembled loop's segment
entries refuses a crossing, a tangency or a shared boundary point at the
requested thickness — R26. Nesting is vacuous: the section is one loop.

**The build** is `evalPrismContext` over a `prismPayload` holding the assembled
section beside the ribbon's own `frame`, `z0`, `z1`, `z0Delta`, `z1Delta` and
`xform`. No level moves, so the solid occupies the ribbon's own interval.

**Refusals, each typed and named.** A `chainPayload` holding more than one
recorded walk, or a nonzero `sectionDelta`, and a walk outside the class above
are R44 — a trimmed ribbon carries a section displacement the exact-generation
gate cannot hold, and a trimmed ribbon is the one `chainPayload` that holds
more than one walk (`docs/surface-intersection-design.md` §3.4). A contact
inside the interval is R26, a rounded generated coordinate R27, a non-positive
sweep height R27.

**What the result publishes.** `Kind()` is `BodySolid`, `IsSolid()` is true,
one lump, and `N + 2` faces for an `N`-segment assembled section: one wall per
segment plus the two planar caps. `Volume = A·h`, `Area = 2A + L·h`, the
centroid is the section centroid lifted to the interval midpoint and `Bounds`
is the swept section box — every one `evalPrismContext`'s own reading under its
existing bound origins: the region integrals' own bounds, both axial
displacements, the frame lift and the accumulated placement rounding. A section
with no corner arc publishes `Exact` readings; one with a corner arc publishes
`Approximate` carrying the rational π enclosure. A 40 mm straight ribbon over
10 mm thickened 2 mm has `Volume` 800 mm³ `Exact` and `Area` 1000 mm² `Exact`
(T162); the same walk bent into an L and thickened 2 mm on its outside has
`Volume` enclosing `1400 + 10π mm³` and on its inside exactly `1360 mm³`
(T163), the quarter-disc and the mitred quarter-square the two corner rules
produce.

### 16.7 The chain revolve shell

**This arm is §16.6's section under §16.5's sweep**, and it states nothing of
its own beyond the two. The receiver is a live `BodySheet` holding a
`chainRevolvePayload` whose `chain` is §16.6's walk class and whose `ax` is
§16.5's axis class. The construction assembles §16.6's closed ring as the
meridian section, runs §16.5's radial gate over the same moving pieces and the
same boxes, re-resolves the axis against the assembled section, and builds
through `evalRevolveContextWork` over a `revolvePayload` carrying that section
beside the shell's own `phi0`, `phi1`, `full`, `den`, `frame` and `xform`, with
`surfaceResult` false.

**A free end ON the axis is R43, and that is the one thing this arm refuses
that §13.3 admits.** A pole's incident walk leaves the axis, so the assembled
ring touches the axis at that one point with two off-axis pieces meeting there
— a pinched meridian. `docs/evaluator-design.md` §6 admits an on-axis junction
only where one incident end is a swept wall and the other an axis line, and
that rule is stated for a RECORDED profile rather than for a section this arm
generated. The strict `ρ > 0` requirement therefore refuses the pole, naming
the least radius it found, instead of building a section no axis-incidence
audit has cleared. Admitting it needs that audit restated for a generated
section, which is its own increment.

**What the result publishes** is §16.5's table over ONE loop rather than two:
one wall per assembled section segment, two planar cap faces when the sweep is
partial and none when it is full, one lump with one shell, and the same
Pappus volume, swept-moment area, axial centroid and `revolveBoundsContext`
box, each under the same bound origins. A 40 mm meridian at `r = 10` spun a
full turn and thickened outward 2 mm has `Volume` enclosing `1760π mm³`
(T165).

### 16.8 What still refuses, and what each family is missing

**A sweep sheet is staged on a publication and one proof, not on the offset.**
A one-span straight or arc sweep DOES carry one constant recorded section, and
§16.2's offset with §16.5's sweep is the construction its result would need. Two
things stand between: `sweepPayload` wraps the reduced prism or revolve, and a
sweep's result must publish the sweep's own role names and payload class —
`finishStraightSweepBody`, `finishArcSweepBody` and `auditSheetBoundary`'s
`sweepPayload` arm each read it — so an arm that published the reduced payload
would hand back a body of a class the caller never asked for. A COMPOSITE path
needs a proof besides: adjacent spans meet at a join whose two offset sections
must be shown to meet without the offset wall folding there, and
`docs/sweep-design.md` §7's separation certificate is stated for the spans, not
for their offsets. `docs/sweep-design.md` owns the first; this section owns the
second.

**A loft sheet is staged behind a variable-section offset.** A loft holds two
recorded sections on two frames and rules a chorded wall between them, so there
is no single section to offset. Offsetting one means an offset at EVERY station
plus a proof that no two intermediate offset sections or their ruled walls
cross — the identical statement §16.4 refuses for `WithTaper`, and certifying
one endpoint section proves none of it. A curved pairing's own
`loftPayload.sectionDelta` is nonzero besides, so §16.2's exact-generation gate
cannot hold for it at all.

**A stitched sheet, a `Body.Patch` result and an `Unstitch` result are staged
behind the general surface offset.** Each holds a FACE SET, and a stitch a weld
plan; none holds a recorded generator to offset. Thickening one is the generic
surface offset §16.1 rejects, plus a join rule at every weld — a round on a
convex weld and a miter on a concave one, decided in 3D rather than in one
section plane. §1.2's surface Offset row names the first; no design in this
tree states the second.

None of the three is permanent. Each names real geometry and each waits on a
capability this tree could state rather than on a different operation, which is
why each is `ErrUnsupported` under Table R's own staged reading rather than
under §1.3's permanent one.

## 17. Offset

### 17.1 Entry point, side and receivers

```go
func (b *Body) Offset(ctx context.Context, distance units.Value,
    opts ...OffsetOption) (*Body, error)

type OffsetOption interface{ offsetOption() }
type OffsetSide int
const (
    OffsetPositive OffsetSide = iota // default: along each source face's positive normal
    OffsetNegative                   // opposite the positive normal
)
func WithOffsetSide(side OffsetSide) OffsetOption
```

**`Offset` returns a second sheet whose every face lies at the stated normal
distance from the source face it came from.** `distance` is a strictly positive
`units.Value` of kind Length, and its sign never chooses a side, exactly as
`Thicken`'s thickness does not (§16.1). `WithOffsetSide` is a sealed option,
admits exactly the two values above, and the last occurrence wins as
`WithThickenSide` does. There is no centered value: an offset publishes one
surface, and two surfaces are two calls. An unknown or nil option and a nil
context are `ErrDegenerate`; a cancelled context returns `ctx.Err()` unchanged
before commit.

**The receiver stays LIVE.** Offset depends on its source and consumes nothing:
the offset sheet and the sheet it came from are the pair a caller measures a
clearance between, stitches into a closed boundary, or thickens separately, and
retiring the source would put a `Duplicate` in front of each of those. The
result registers under a new producer identity on the non-consuming terms
`PlacedCopy` and `Duplicate` already take (`docs/api-design.md` §6). That is
the one contract difference from `Thicken`, which consumes.

The source sheet's positive side is §2.3's built orientation, read exactly as
§16.1 reads it: a `Document.Patch` sheet's is its sketch plane's normal, and a
surface-extruded profile-fed prism sheet's is the profile region's exterior on
EVERY loop wall. So `OffsetPositive` grows a prism sheet's section and
`OffsetNegative` erodes it, and the two never need a reversal of the source to
be named (§1.4).

The admitted receivers are §16.1's two, for §16.1's own reason: an offset needs
one recorded generator and a closed-form rule that carries it a stated distance.

| Sheet family and payload | Result at the call | Deciding record or missing proof |
|---|---|---|
| `Document.Patch`, `patchPayload` | admitted | the offset of a planar face along its own normal is that face translated, so §17.2's patch arm is a rigid motion over the unchanged recorded profile |
| profile-fed prism sheet, `prismPayload` with `surfaceResult == true` | admitted under §17.2's gates | one outer loop, no holes; either all `LineSeg` walks parallel to recorded U or V, or one whole `CircleSeg`; `sectionDelta == 0` |
| every other sheet family — revolve, sweep, loft, stitched, chain ribbon, chain revolve shell, `Body.Patch`, `Unstitch` | R37 | an offset publishes ONE surface at a stated normal distance, and this evaluator has that construction for a closed recorded section alone. §16.6's ribbon reaches its own result by assembling a CLOSED ring rather than by offsetting an open walk, so neither chain-fed family states an offset surface at all; a revolve or sweep meridian's own offset surface is a later increment's to state; and §16.8 states what the loft, stitched, `Body.Patch` and `Unstitch` families are missing for either operation |
| a `BodySolid` receiver | R37 | a solid encloses material, so offsetting its skin states nothing about the region; `Shell` is the operation that offsets a solid's section, and `docs/modify-design.md` §8 owns it |

### 17.2 Construction and refusal

**The patch arm is a translation, and it mints no geometry.** The result is the
receiver's `patchPayload` under the composed motion that translates it by
`±d·N` in the recorded patch frame, `N` the frame's own normal — the identical
rebuild `PlacedCopy` performs, charging the identical placement rounding. The
recorded profile, its frame axes and every plane-local coordinate are the
source's, unchanged: a planar region carried along its own normal is congruent
to itself, so no boundary is regenerated, no corner is minted and no offset
audit has anything to run on. The millimetre conversion's own displacement is
charged into the translation before the composition, as §16.2 charges it into a
patch's prism levels. A conversion or composition this evaluator cannot carry
is R39.

This arm therefore adds no proof the evaluator does not already run, and a
caller can spell it today as `PlacedCopy` under a translation built by hand.
The entry point exists so that the direction comes from the sheet's own
recorded normal rather than from the caller's arithmetic, and so that one
operation covers both admitted families under one refusal set.

**The prism arm is §16.2's offset without the annulus.** Write `P` for the
receiver's recorded hole-free section. `OffsetPositive` requests `Q = P ⊕ d`
and `OffsetNegative` requests `Q = P ⊖ d`. The result is a prism sheet over the
source's own sweep: the same `frame`, the same `z0`/`z1` with the same
`z0Delta`/`z1Delta`, the same accumulated placement, `surfaceResult` true, and
`Q` as the section. A convex outside corner grows a radius-`d` arc and a
concave inside corner miters, as `docs/modify-design.md` §8 states.
`shell_offset.go`'s `offsetProfile` supplies `Q` and `auditOffsetSectionBudget`
checks it, exactly as §16.2 has them supply and check each of Thicken's
requested sections, and the certification of the whole offset interval
`0 < τ ≤ d` runs unchanged for §16.2's own reason: an endpoint that looks
simple cannot prove an earlier offset did not pinch and change which boundary
the construction denotes.

**Two of Thicken's proofs do not apply, and naming which is the whole of the
difference.** The source-and-offset disjointness-and-nesting audit §16.2 runs
before building an annulus has nothing here to run on — the result holds ONE
region, bounded by `Q` alone, and denotes nothing whatever about `P`, so no
nesting relation is published and none is needed. The result is likewise not an
outer region less an inner one, so `reverseLoopRecordContext` and
`evalTubeContext` are not on this path. Everything else §16.2 states — the
exact-generation gate below, the offset audit, the interval certification and
the refusal set — is this arm's unchanged.

**The exact-generation gate is §16.2's, and it is what keeps
`sectionDelta == 0` truthful.** The distance must convert to millimetres with
zero displacement, every source line endpoint must resolve with zero bound, and
every generated plane-local endpoint, center and radius must equal its
closed-form value as a binary rational, compared with `dyadic`/
`rationalFloatError`. The U/V normal is selected by the nonzero coordinate
directly; a miter is solved over the exact dyadic input; a convex corner takes
the stated corner point and radius `d`; a whole circle takes its stated center
and the exact sum or difference of its radius and `d`. Never infer exactness
from a small residual or from `math.Hypot`, `Atan2`, `Sin` or `Cos`: none has
an accuracy contract here. A generated value that is not representable is R39.
Because the gate holds, the result carries `sectionDelta == 0`, so
`evalPrismContext`'s existing moment and extent proofs apply to the offset
section as they do to a recorded one, and no reading describes a more precise
wall than the record denotes.

**A dropped feature, a failed miter, a crossing, a tangency, a shared boundary
point, a lost orientation or an undecided comparison is R38**, on §16.2's own
wording: a 4 mm-wide neck eroded by 3 mm crosses its own offset walls although
every source segment is valid, and an erosion at or past the section's inradius
drops a loop outright. A scale-anchored contact floor may refuse more inputs; a
gap above that floor is never the proof of separation. Only the closed-form
segment-pair decisions on decad's OWN synthesized section permit construction.
`shell_offset.go`'s header owns that distinction, and it is why a closed-form
decision on the offset is admissible where a residual against a handed-in curve
would not be: `CLAUDE.md`'s reject-only rule governs what `sketch` hands over,
and this section is decad's own construction proven in closed form.

Gate order is fixed: check context, liveness, owned options and magnitude;
select the receiver family; for a prism, convert and certify the distance;
construct the requested offset and certify its generated values; run
`auditOffsetSectionBudget`; prove the whole offset interval; build the sheet;
recheck liveness and context; commit. A construction refusal is returned
unchanged before any body is registered, and the receiver stays live in a
refusal exactly as it does on success.

### 17.3 Result and downstream readings

**Both arms return a `BodySheet`.** The patch arm returns a one-face sheet with
one free edge per coalesced recorded boundary walk — the source's own face and
edge counts. The prism arm returns a prism sheet with one wall face per segment
of `Q` and both rims free, inserted corner arcs counted as segments: `N_Q`
faces and `2·N_Q` free edges for a single loop, and no cap. Roles are the new
producer's `side(i, j)` indexing the RESULT record; no source face role is
inherited (`docs/modify-design.md` §9), so a caller re-selects by geometric
predicate as it already must after a Fillet.

Both arms hold an ordinary payload of a class this evaluator already
dispatches — `patchPayload` and a `surfaceResult` `prismPayload` — so §8's
measurement rows, §9's `Verify` questions, §10's mesh and export, §11's Table X
rows and §16's own `Thicken` admission all read the result by payload class
under their existing gates. No row of any of them is amended.

Where each reading and each bound comes from:

| Reading | Value | Bound |
|---|---|---|
| `Area`, patch arm | the source's, unchanged | the source's, unchanged: the recorded profile and its region integral are the same ones, and a rigid motion moves no area |
| `Bounds`, patch arm | the recorded boundary's box under the composed motion | the frame and placement rounding `prismBoundsContext` already charges, as `PlacedCopy` charges it |
| `Area`, prism arm | `evalPrismContext`'s wall sum over `Q`, the omitted caps subtracted as §4.3 states | the perimeter and height terms that builder already composes; the section term is zero by §17.2's gate |
| `Bounds`, prism arm | `prism_extent.go`'s reading over `Q` | the source's `z0Delta`/`z1Delta` unchanged, plus the frame and placement rounding |
| `Volume`, `Centroid` | `ErrNotSolid`, both arms | — §8, by kind, not by soundness |

A zero bound is a positive claim of exactness and is published only where the
complete value is exactly representable. An offset whole circle's perimeter
carries the rational π enclosure and reads `Approximate`, never an unbounded
float evaluation, and no raw arithmetic follows a proven bound without charging
its own rounding.

**A second offset of an offset result is admitted for the whole circle alone.**
The result passes §17.1's family gate — it is a `surfaceResult` prism sheet
with `sectionDelta == 0` and no hole — and then meets the section-shape gate:
an offset circle is again one whole `CircleSeg` and offsets again, while an
offset rectilinear loop carries corner arcs and is no longer the axis-parallel
line class, so it is R37. The identical reading decides `Thicken` on an offset
result, through §16.1's own gate rather than a second rule.
