# Evaluator Design (v1)

How the v1 evaluator turns validated feature inputs into a `Body` — the
topology it builds, the measurements it computes and the bounds it proves, how
selectors resolve against it, how the tessellation-backed boolean works, and
how `Verify`'s checks are implemented. Companion to `agent-docs/api-design.md` (the
core contract this evaluator implements — references of the form "core §N"),
`agent-docs/sketch-seam-design.md` (the profile records it consumes),
`agent-docs/verification-design.md` (how its answers are judged). Nothing here
changes those contracts. `agent-docs/tessellation-design.md` owns the tessellation
contract and the private operand proofs the boolean consumes. The
payload-specific proofs and implementation order live in
`agent-docs/payload-verification-design.md`. `agent-docs/interference-design.md` owns the
read-only use of this evaluator's intersection volume inside `Verify`. Where
this document stages a capability, the staging is explicit and rejected loudly,
never silently approximated.

## 1. One-way feature evaluation

**The evaluator consumes structural profile records, never live sketch geometry.**
A feature call gates its live inputs (core §7), converts the profile and plane
to internal `momentinput.Profile` and `sectionrecord.PlaneRecord` values, and
evaluates from those values. The live `*sketch.Profile` is never read after
conversion.

The one thing the evaluator reads from the live profile, at feature-call time
only, is a **falsifier input**: `Profile.Area` (sketch's own area answer) is
compared against the area the evaluator computes from the records (§4). A
large mismatch rejects the call — the record and the profile disagree, which
is a bug somewhere — and a small one proves nothing and admits nothing, the
same one-sided shape as the seam's range falsifier.

## 2. What v1 is, and is not

Per core §2: **v1 uses analytic construction for features, and tessellation for
every boolean outside the analytic reduction's admitted class.** Feature bodies
carry analytic faces, as does a `Union`/`Cut`/`Intersect` result that reduction admits (§9).
Every measurement carries its own proof: exactly representable results are
`Exact`; float/transcendental closed forms and tessellated results are
`Approximate` with proven bounds.

`Exactness` on a boolean's measurement is decided by that measurement's own
proven bound, never by the fact that a boolean produced it (core §5.3: a zero
bound IS the claim that the value is exactly representable). A nonzero bound
reads `Approximate` and a **zero** bound reads `Exact`, on either path. Which
measurements can reach zero depends on the path that built the result.

**On the ordinary mesh path (§9), only VOLUME can reach it:** an all-planar pair
whose contact points round exactly is integrated in exact rational arithmetic
over the held mesh, with a genuinely zero volume bound. Other ordinary mesh
measurements, including its centroid, have nonzero bounds because their float
integrals charge rounding (§9, `internal/proofbound/bounds.go`). A contained
mesh Cut may also publish an Exact centroid when its strict-containment
certificate permits a source-derived first-moment calculation whose own bound
is zero. That centroid is Exact only from its own zero bound; the mesh proof
still supplies closure, topology and the ordinary fallback.

**On the analytic reduction (§9), the result is a swept payload and reads by §4
like any other one**, so a volume, an area, a centroid and a `Box` can all reach
a zero bound. `agent-docs/prism-boolean-design.md` §7 owns when each of them does; this
document does not restate its rule.

**Staging is explicit.** v1 lands in increments (§11), and an intent this
evaluator cannot yet build is **rejected at the call** with the sentinel
`ErrUnsupported` (core §12) — never built approximately or silently narrowed.

## 3. The topology model

Concrete structs with unexported fields; the accessors of core §6/§6.1 are the
whole public surface. Identity is the pointer, scoped to one body — never an
index (core §3 invariant #3).

Each topology type keeps its name, methods and godoc in the root package and
holds its data by value in an `internal/topo` node (`Vertex` holds a
`topo.Vertex`, `Edge` a `topo.Edge`, `CoEdge` a `topo.CoEdge`, `Face` a
`topo.Face`, `Loop` a `topo.Loop`, `FaceShell` a `topo.FaceShell`, `Lump` a
`topo.Lump`, `Document` a `topo.Document`,
`Mesh` a `topo.Mesh`, `Body` a `topo.Body`), so
internal packages can read and build it. The handle's pointer is still the
identity; no other rule of this section changes.

A node links to the nodes it shares by node pointer: an `Edge` node holds its
`Start` and `End` vertex nodes and its adjacent `Faces` face nodes, a `Loop`
node holds its `CoEdges` by value, each a `topo.CoEdge` with its `Edge` node and
walk sense, a `Face` node holds its `Loops` loop nodes (the outer loop first)
and its `Body` node, a `FaceShell` node holds its `Faces`, a `Lump` node its
`Shells` (the outer shell first), a `Body` node its `Lumps`, its `Doc`
document node and its cached tessellation (`Tess`, a `topo.TessEntry` holding
the `Mesh` node), a `Document` node its `LiveSet` of live body nodes (§8), and
a `Mesh` node the `Source` face node of each facet and its per-face
`FaceBound` record keyed by face node. A node another node links to carries
`Owner`, the handle that holds it, set once when the handle is born
(`bornVertex`, `bornBody`, `bornFace`, `bornEdge`, `bornLoop`,
`bornFaceShell`, `bornLump`, `bornDocument` and `bornMesh` in
`topology_handles.go`; every root `Vertex`, `Body`, `Face`, `Edge`, `Loop`,
`FaceShell`, `Lump`, `Document` and `Mesh` is born through them, and
`TestTopologyHandlesAreBorn` fails on any other construction). A `CoEdge` is a
value that nothing links to, so it has no `Owner`: the public `CoEdge` holds
its node by value and `Loop.CoEdges` wraps each stored use. Root maps a linked
node back to its handle through `Owner` only where a handle leaves the code
(`Edge.Start`, `Edge.End`, `Edge.Faces`, `CoEdge.Edge`, `CoEdge.Start`,
`CoEdge.End`, `Loop.Edges`, `Face.Loops`, `Face.Edges`, `FaceShell.Faces`,
`Lump.Shells`, `Body.Lumps`, `Body.Shells`, `Body.Faces`, `Body.Edges`,
`Body.Vertices`, `Body.Document`, `Document.Bodies`, `Mesh.SourceFaces`, the
cached mesh `Body.Tessellate` returns, and the body, face, edge, loop, shell,
lump or document a root function needs as a handle); reads of node data go node to node, and
a linked node compared with a handle compares nodes. Node and handle pointers pair one to one,
so identity, `==` and map keys are unchanged. A handle is never copied by
value: it holds a `noCopy` marker that `go vet`'s copylocks check enforces.

| Type | Holds |
|---|---|
| `Body` | owning `*Document` (the document node in the node, which `Body.Document()` maps to its handle), origin (a `provenance.Ref` in the node; `Body.Origin()` converts it to `FeatureRef`), `BodyKind` (a `topo.BodyKind` in the node; `Body.Kind()` converts it), `[]*Lump` (lump nodes in the node, which `Body.Lumps()` maps to their handles), cached measurements (computed at build, immutable after; `measurement.Measurement`, `VecMeasurement` and `Box` in the node, which `Volume`, `Area`, `Centroid` and `Bounds` convert), the payload it was built from (a `topo.Payload` in the node, §8), the last tessellation (`Tess` in the node, `agent-docs/tessellation-design.md` §1.1) |
| `Lump` | `[]*FaceShell` (shell nodes in its `topo.Lump`, which `Lump.Shells()` maps to their handles) — a connected piece of a body, solid or sheet; a sheet lump holds exactly one shell (`agent-docs/surface-design.md` §2.2) |
| `FaceShell` | `[]*Face` (face nodes in the node, which `FaceShell.Faces()` maps to their handles), `void bool`, `open bool` — `open` is true when the shell has a free edge (`agent-docs/surface-design.md` §2.2) |
| `Face` | tagged `Surface` (the `surfacegeom` variant in the node; `Face.Surface()` converts to the public one), `[]*Loop` (first outer; loop nodes in the node, which `Face.Loops()` maps to their handles), origin roles (≥1 `provenance.Ref`s in the node, exposed as `FeatureRef`s by `Face.Origins()` — a canonicalization merge UNIONS the merged faces' roles, and `FaceCreatedBy` matches on ANY of them, so provenance survives the merge) , back-ref to its body (the `Body` node in the `Face` node; root maps it to the handle through `Owner`) |
| `Loop` | ordered `[]CoEdge` (directed edge uses in boundary-walk order; `topo.CoEdge` values in the node, which `Loop.CoEdges()` wraps), `outer bool` |
| `CoEdge` | shared `*Edge` plus walk sense (the edge node and `Forward` in its `topo.CoEdge`); public `Edge`/`Start`/`End`/`IsForward` accessors map the edge and its vertices to their handles |
| `Edge` | tagged `Curve` (the `surfacegeom` variant in the node; `Edge.Curve()` converts to the public one), `Start`/`End` (vertex nodes in the node; `Edge.Start()`/`End()` map them to their handles), ALL adjacent faces (face nodes in the node; `Edge.Faces()` maps them to their handles; exactly 2 on a closed manifold body; `Edge.Faces()` reports the actual count, which is precisely how `len != 2` surfaces non-manifold topology — core §6.1; exactly 1 is a FREE edge on a sheet body, `agent-docs/surface-design.md` §2.2), `convex bool` |
| `Vertex` | position (mm), the proven bound on it |

Rules:

- **A `Body` is immutable after construction.** Every operation builds a new
  body; concurrency safety (core §12) falls out.
- **`BodyKind` is stamped by the operation that built the body, never inferred
  from the boundary afterwards.** A feature that closes its wall set into a
  solid stamps `BodySolid`; a surface-result feature, a patch, and a stitch
  that left a free edge stamp `BodySheet`. A closed boundary is not by itself a
  solid: a full revolution built as a surface closes and is still a sheet, and
  `Stitch` is the one operation that makes the material claim.
  `agent-docs/surface-design.md` §2.1 owns the kind and §6 owns that claim.
- **Provenance is structural.** `FeatureRef` identifies a private document-local
  producer plus a stable role within it; the topology nodes hold the same two
  values as `provenance.Ref`, and `FeatureRef` is its public form — `side(i, j)` (loop `i`, segment
  `j`) for a swept wall; `side(i, j, k)` for a Loft wall triangle;
  `capStart`, `capEnd`, and the revolve/boolean analogs. This bullet owns the
  role MECHANISM, not every grammar it carries: `agent-docs/loft-design.md` §7 owns
  what `i`, `j` and `k` index for a Loft, and the `side(i, j)` gloss above is
  the swept-wall grammar alone, which no Loft role reads. Roles derive from
  the evaluated feature, and the provenance predicates — `CreatedBy`
  for edges, `FaceCreatedBy` for faces (core §9) — select the same entities
  under every run.
- **Canonicalize at build.** Adjacent side faces merge where their two
  recorded segments lie on one exact line: parallel chords over the
  rationals that meet at one point, with no junction bound reaching off the
  line (`internal/boundarywalk`). A near-collinear kink keeps both faces, so
  no merge drops a vertex the record has. Ordinary Loft wall triangles do not
  merge with another face. A certified whole-circle through-Cut or direct Loft
  replaces only its hole-wall triangles with one curved face that retains their
  roles. Direct Loft admits a cylinder, cone or rational ruled surface under
  loft §18's exact axis and center proof, source and area gates. A pair whose
  frame lifts agree only after rounding keeps its separate faceted walls.
  This keeps
  `side(i,j,0)`/`side(i,j,1)` distinct within a cell and preserves every
  cross-cell rung between coplanar Loft triangles. A full cylinder is one
  face with two circular-edge loops and no seam edge. v1 counts already match
  the analytic answer, so vN does not churn them (core §3).
- **Every vertex carries its bound.** Exactness is decided by what the
  coordinate was READ FROM, never by which builder placed it. A coordinate the
  record states is exact (bound zero); every computed one carries its own
  computation's proven displacement. A swept vertex is read from two
  independent coordinates and carries both: its plane-local pair from the
  section — displaced where the payload carries a section displacement (§9,
  prism-boolean §7) — and its sweep level from the extent, displaced where the
  level was computed rather than stated (a `ToFace`/`ThroughAll` stop resolved
  in float, a magnitude rescaled from a non-base unit, a chamfered end's
  setback). A Boolean result that takes §9's mesh path carries the tessellation
  chord bound. A Boolean result that analytic reduction admits carries its
  payload's own bound instead, as `agent-docs/prism-boolean-design.md` §7 owns. A
  cap-loop chamfer's cap-level feet the offset solve's displacement
  (modify-reach §8.4). A loft's vertices carry the payload's own `delta`, the
  placement term `placeAllow` composed with the computed station's
  `stationRound`, and every consumer conditions on `delta > 0` rather than on
  the body having been placed. A loft is NOT swept, so its section
  displacement `sectionDelta` is not a vertex term at all.
  **`agent-docs/loft-design.md` §5.2's table owns every loft term, its condition
  and its refusal; this bullet names them and restates none.** The
  verification gate reads these (verification §4).
  A rim vertex where two recorded walks meet — a prism's, patch's or
  chain `Extrude`'s junction, and the point a revolve's junction vertex is
  measured against — is bounded against the points BOTH neighbours denote
  there (`boundarywalk.JunctionVertex`). It sits at the next walk's held
  start. That walk's own end bound reaches its denoted start, and the
  previous walk's end bound plus the distance between the two held ends
  reaches the previous walk's denoted end. The vertex takes the larger of
  the two in each component. An end bound is the walk's own: a trimmed
  line's lerp rounding, a circle's or trimmed arc's trig enclosure, a
  free-form chain's control-point rounding, and an arc's natural `t = 1`
  radial residual (§4). A chain's free end carries its one walk end's
  bound. Where both neighbours denote one point, at a circle's own seam,
  either distance alone reaches it. The vertex then takes the smaller and
  sits at whichever held end is the closer proven one, which places a
  clockwise hole's seam on its exact `t = 0` point. So a vertex where
  natural line ends meet at one recorded coordinate, or the seam of a circle
  whose centre plus radius is representable, is `Exact`. A cut junction, a
  trimmed line end, and a seam whose sum rounds carry a bound.
  A brep body's vertex (`agent-docs/general-boolean-design.md` §4.2) takes the
  largest bound over every face use meeting it. Each use charges the bound
  from its held walk end to the point its recorded segment denotes there
  (`brepgeom.Use`'s `StartBound` and `EndBound`, which call
  `boundarywalk.DenotedStartBound` and `DenotedEndBound`). Every such use
  holds the same reference coordinates, so the largest of them reaches both
  neighbours' denoted ends, an arc's natural `t = 1` end included.
  `PatchEdges` admits a rim with such a vertex only through its level-token
  arm, which a straight prism build stamps (`agent-docs/surface-design.md` §5.2,
  Table R row R6).
- **Every loop exposes its stored direction.** `Loop.CoEdges()` returns copied
  `CoEdge` values in boundary-walk order. Each use's `Start`/`End` follows that
  walk and `IsForward` states whether it matches the shared `Edge` orientation.
  `Loop.Edges()` returns the same edge identities and order without direction
  for compatibility.
- **`convex` is the walked boundary, decided at build** (core §6.1). The
  profile walk carries the material on its left — outer loop counter-clockwise,
  every hole clockwise (§4) — and every edge reads that walk, never a 3D
  dihedral measured after the fact. A junction edge — a prism's vertical edge,
  and a revolve's swept junction, which is a partial sweep's arc and a full
  revolution's latitude circle alike — takes the sign of the cross product of
  the incoming and outgoing walk tangents: a left turn is convex. A rim edge (a
  wall's own copy in a cap plane) takes the sense of the wall it runs along —
  a circular wall by its own turn, counter-clockwise convex and clockwise
  concave, which is the same test that decides whether the wall's material lies
  outside its cylinder; a straight wall, having no turn, by the role of its loop
  (outer convex, hole concave). A free-form wall's own curvature can turn sign
  inside one span, so it reads neither test — `agent-docs/spline-design.md` §6.5 owns
  its proof, its straight-walk case, and its refusal where the curve's own
  curvature sign is not proven. The on-axis edge shared by both caps of a
  partial revolve is convex when the sweep is under π. Consequence, and it is
  the intended one: a hole's rim edges are concave, and so are the rims of a
  concave round on the outer loop.

## 4. Mass properties — decad's own, on decad's own records

The faceted `MassProperties` query can consume immutable V/P/Q evidence from
the admitted strictly contained Cut of a rectangular prism by a whole circular
prism. It checks the source tuple against the independently audited mesh tuple
before selecting it. A direct exact-rotation Copy of that original Cut
transports its frozen moments through the exact map and repeats the same mesh
comparison. The transfer does not support chains or ordinary placements; see
`agent-docs/exact-copy-moments-design.md`.

The hard rule "Ask `sketch` for 2D answers by default" forbids re-deciding what
sketch has decided: closure, validity, cuts, intersections, projections. It does
**not** put the recorded region's *mass properties* out of reach — once
recorded, the region is decad's own geometry, and volumes, centroids and
areas of the bodies built from it are decad's 3D job. The boundary is:

- **sketch decides topology and admissibility** — what closes, what is valid,
  where curves cross, what a trim is. decad consumes.
- **decad integrates its own records** — area, first moments, the second and
  mixed moments (`∫u² dA`, `∫uv dA` — what a revolve centroid needs, §6),
  arc length, and extremes of the recorded loops, by closed form per segment
  kind: shoelace-family boundary integrals over segment chords plus the
  exact circular-segment corrections for `CircleSeg`/`ArcSeg` fragments (the
  same correction family geom uses; independent code on decad's own data,
  asserted against sketch's answer by the §1 falsifier at every feature
  call).

Internally built and decoded records carry no live sketch answer.
`momentinput.Profile` mass-property methods MUST validate before integration:
finite supported fields/ranges first, then region topology through a private
`sketch` reconstruction that exactly matches the recorded walks. Whole-circle
regions use direct disk containment/separation, preserving valid thin annuli
below sketch's general arrangement threshold. Reject malformed or overflowing
input; NEVER return a non-finite `Exact` result.

An `ArcSeg` denotes the circle of **Start's** radius about `Center`, swept from
Start's angle to End's angle — geom's own arc reading
(`agent-docs/sketch-seam-design.md` §2). The recorded `End` need not sit on that
circle exactly: a fillet or shell rewrite rounds each coordinate at its own
magnitude, and a solver-placed sketch arc carries the solver's residual, so the
two pinned radii generally differ by rounding of the *coordinates*, not of the
radius. The preflight refuses, as malformed input, an End whose radius differs
from Start's by more than 1024 ulps of the largest magnitude the record states
(its six coordinates and its two radii). That check is reject-only: passing it
certifies nothing. Every circular bracket encloses the denoted arc — Start's
radius, End's angle — by reading End's radial ratio `ρ = r / |End − Center|`
as the `ratSqrtDown`/`ratSqrtUp` bracket of the exact rational
`r² / |End − Center|²` and substituting `ρ·(End − Center)` for End's
coordinate differences. Equal radii give a point `ρ = 1` and lose nothing;
unequal radii have the difference charged into the published bound, never
trusted and never dropped to the magnitude envelope.

An `ArcSeg` recorded over a narrowed range — a cut fragment of a sketch arc,
a `Split` piece's meridian — keeps its parent's `Start`, `End` and `Center`
and walks `θ(t) = a0 + t·sweep` on Start's radius `r` from `t = TStart` to
`t = TEnd`, in the record's order (`agent-docs/sketch-seam-design.md` §2). `a0` and
the sweep are the `Atan2Interval` enclosures of Start's and End's angles,
under the same `+2π` branch correction a whole arc takes, and `t` is the
recorded float read as an exact rational. With `X = r·cos θ`, `Y = r·sin θ`
the walk's offsets from `Center`, `(cU, cV)` the centre about the walk
anchor, `Δθ = (TEnd − TStart)·sweep` signed and
`[g] = g(θ(TEnd)) − g(θ(TStart))`, Green's theorem gives every region
integral as a polynomial in the two ends' `(X, Y)`, the exact `r²` and `Δθ`:

- area `½∮(u dv − v du) = ½(r²·Δθ + cU·[Y] − cV·[X])`, from `u = cU + X`,
  `dv = X dθ`, `du = −Y dθ`;
- the first, second and third moments `(1/(p+1))∮u^(p+1)·v^q dv`, expanded
  binomially about the centre into `∫X^a·Y^b dθ`, which the trig-power
  reduction `J(a,b) = [X^(a−1)·Y^(b+1)]/(a+b) + (a−1)·r²·J(a−2,b)/(a+b)`
  (and its `b ≥ 2` mirror) brings down to `[·]` of end products and the one
  term `J(0,0) = Δθ`;
- the axial moment `∫ρ ds` a revolve's wall area reads, `ds = r·|dθ|`,
  walked over the ascending range from `θ(min(TStart, TEnd))`: about an axis
  anchored at `(aU, aV)`, with the recorded centre `(cU, cV)` and the ends
  `lo`/`hi` in ascending order, `∫(u − aU) ds = r·((cU − aU)·|Δθ| + Y_hi −
  Y_lo)` and `∫(v − aV) ds = r·((cV − aV)·|Δθ| + X_lo − X_hi)`, combined
  through the axis's outward normal.

Only the ends' offsets differ from a whole arc's. At `t = 0` the offset is
Start's recorded difference, at `t = 1` it is `ρ·(End − Center)`, and at a cut
`t` the record states no point, so the offset is `r·(cos θ(t), sin θ(t))`:
`r` the `ratSqrtDown`/`ratSqrtUp` bracket of the exact `r²`, the sine and
cosine `radSinCosSpan`'s enclosure over the enclosed angle. Every factor is an
exact rational or a proven enclosure, so the interval encloses the integral
the record denotes. A whole arc's region integrals read its two recorded ends,
so they carry no trig-bracket width there. A cut end adds about one ulp of `r`
of width, so a hemisphere or a sketch-cut cap reads its volume, area and
centroid within `1e-13` relative before a `Split` piece's cut-displacement band
(`agent-docs/surface-intersection-design.md` §7.2) and the junction charge below are
added. The interval encloses the DENOTED walk at the recorded `t`; whether that
`t` is the true crossing is `sketch`'s `TExact` claim, which this bracket
neither tests nor widens.

**A loop's region is closed at every junction.** Each segment
denotes its own entity at its own recorded parameter, so at a cut bound the
walk ending there and the walk starting there denote two different points:
neither recorded `t` is the exact crossing, and a float `t` places a point on
its entity no closer than about `ulp(t)·|entity|`. A 137000 mm line cut by a
7.3 mm circle leaves a 4e-12 mm gap, and a whole arc's natural end leaves the
radial residual above. The seam admits such a junction on `sketch`'s shared
node, never on the two denoted points (`agent-docs/sketch-seam-design.md` §1). The
region a loop denotes is the one its segments bound with each gap closed
between the two denoted ends. Where both neighbours are `LineSeg`s, the gap
closes through the corner: the exact crossing of the two supporting lines,
each through its segment's recorded `Start` and `End`, computed over exact
rationals (`lineCorner`). The closing path runs from one denoted end along
its own support to the corner and on along the other support to the other
end, so a polygon of crossing lines closes at exactly the crossings `sketch`
arranged. A T-junction, one line ending naturally on another, closes the same
way: its end need not lie on the other support, and the corner is the
supports' crossing, not that end. The corner closes the junction only when it
is provably near: the two legs together measure at most 1024 times the
straight chord between the two ends, each leg rounded up and the chord
rounded down. That ratio grows past 1024 only where the supports meet at well
under a degree, and there, as where the supports are parallel, the straight
chord closes the gap. Every other junction closes with the straight chord. A
sum of segment integrals alone omits the closing path, which moves a field by
up to `length·reach` about the walk anchor. `internal/momentinput/junctions.go` charges every junction whose
two ends are not provably one point. It bounds the gap from the two walk ends
and their own bounds (`survey2d.SegmentWalk`'s `StartBound`/`EndBound`, an
arc's `t = 1` end widened by the radial residual), and widens each held field
whose boundary form has degree `k` by `gap·reach^k`. Every field's form has an
integrand coefficient of at most 1, so the widening covers the chord in any
direction (`momentregion.State.ChargeJunction`); a corner path is charged leg
by leg, each with its own length. Where both ends are exact rationals — a
line's lerp, a free-form chain's end control point — the rational sum adds
the closing path's exact integral instead, so a line-only region stays
`Exact` and equals the crossing polygon: an oblique triangle of three
2e6 mm lines reads the crossing polygon's area rounded once. Two natural line
ends at one coordinate, and two fragments of one circle meeting at its seam,
are one point and charge nothing. A chord-closed junction differs from the
exact crossing geometry by a sliver of order `gap²`. A region with a circular
segment holds its fields widened by `gap·reach^k`, far above that sliver; a
region kept `Exact` through a chord — two nearly parallel lines, a free-form
chain's end — publishes the chord-closed region itself.

Increment 1 implements the closed forms for `LineSeg`/`CircleSeg`/`ArcSeg`.
`agent-docs/spline-design.md` owns the free-form kinds entirely: Table F there assigns
each an exactness tier and what a measurement over it may claim, §5 gives the
construction, Table R owns every refusal and its sentinel — the permanent and the
upstream-blocked ones among them — and §10 owns the landing order. This section
restates none of it; read it there.

## 5. Extrude

Input: the recorded profile + plane, a linear `Extent`, options. The sweep
direction is the plane normal `U × V` for `Along` (core §8.1).

A bounded `Body.Section` profile carries an opaque Sketch attachment.
`Extrude` admits it only while its marked reference boundary, profile, revision
and plane frame still match the source certificate. It uses the certified frame
directly and passes the source displacement into `prismPayload.SectionDelta`.
Only a straight solid sweep with an independent `Distance` extent consumes this
bound; other options and profile consumers refuse it. See section design.

Faces, per recorded loop segment (roles in parentheses):

| Segment | Side face |
|---|---|
| `LineSeg` | `Plane` (`side(i,j)`) |
| `CircleSeg` (whole) | full `Cylinder`, no seam edge |
| `CircleSeg` (fragment) / `ArcSeg` | `Cylinder` patch bounded by two line edges + two arc edges |
| free-form kinds | `NURBSSurface` — `agent-docs/spline-design.md` §7 owns the variant and its exactness, Table C the extrude reach, Table F the per-kind tier, Table R the refusals |

Caps: one planar face per end (`capStart`/`capEnd`), the outer loop plus one
loop per hole, holes wound opposite.

Measurements (untapered, increment-1 kinds): closed form with outward numerical
bounds; only proved exactly representable results are **Exact**. The
extent resolves to a signed sweep interval `[z0, z1]` along the plane normal
(`Along` positive; `Against`, `Symmetric` and `TwoSided` sides place `z0`/`z1`
on their own senses), and every formula reads the interval: `Volume = A·h`
with `h = |z1−z0|` (`A` the recorded region's area per §4 — measures are
magnitudes, so an `Against` sweep never reads negative), `Centroid` the
region centroid lifted `(z0+z1)/2` along the normal — the SIGNED midpoint,
`h/2` only in the one-sided `Along` case — `Bounds` from per-segment analytic
extremes swept over the signed `[z0, z1]`, beside the frame and placement's own
rounding: `xform.Apply`/`xform.ApplyDir` are isometries only in EXACT
arithmetic, so a frame that is not axis-aligned or a placement that is not the
identity rounds, and `Bounds` (a `capBlendPayload` result's included) charges
that rounding rather than reading the placed extreme as an exact leaf. Every
rim and side vertex `prismrecord.BuildLoopSidesAs` places charges its OWN frame/placement
lift rounding, measured exactly for that vertex (`prismPayload.LiftedVertex`,
`internal/proofbound/bounds.go`'s `ExactFrameLiftRound`: the held point
against the same origin + u·U + v·V + z·N chain and placement over dyadic
rationals) and folded into the vertex's own section/axial displacement rather
than into `Bounds` alone. The frame ORIGIN is a leaf of that chain, so an
axis-aligned sketch plane whose origin the plane-local coordinate does not add
to exactly (origin x = 10⁶ + 0.1, or 0.1) charges the rounding of that sum,
and a body built far away and placed back keeps the far lift's rounding. A
lift exact for the coordinates at hand — an integer origin and integer
coordinates under the identity placement, for one — charges zero. A cap-loop
chamfer's own cap-level vertices take the same per-vertex charge in
`capblend.BuildBand` (`capVertexAt`), beside their own contour
displacement, never inside the value an edge's own length bound still reads
unwidened; chain `Extrude`'s rim posts, a brep record's vertices and a patch's
rim vertices take it too. The cost is one exact dyadic evaluation per vertex,
about 0.3 µs, under 1% of a 1000-gon extrude's build.
`apitest/vertex_frame_origin_test.go` checks every payload kind at a
non-dyadic and a far origin against the exact rational lift, and
`clearance_lift_internal_test.go` the clearance carriers. Beside
it, the same reading charges the rounding its own FINAL SUMMATION of those
terms into one published coordinate commits. That is a separate mechanism, not
a consequence of the first: a placement can leave every coefficient exactly
right and still round when they are added, which is exactly what a pure
translation does, so a reading charging only the coefficients publishes a
translated box as `Exact` with a zero bound. Both terms are zero for an
unplaced, axis-aligned payload, which keeps the ordinary prism's box `Exact`
as before, and a placed one is `Exact` only where its own endpoint sum is
representable too. `Area` from cap areas + side areas
(`segment length · h`; arc length `rθ` carries its evaluation bound). Each END
of the interval carries its own proven **axial displacement** — how far the
level recorded there sits from the level the extent denotes. A level the caller
stated denotes itself and carries zero; a level the resolution computed carries
that computation's own rounding (a `ToFace`/`ThroughAll` stop resolved in float
against another body's face, a magnitude rescaled from a non-base unit, a
chamfered end's setback). It is the axial twin of the section displacement:
the two are tracked apart and neither stands in for the other — one moves a
coordinate IN the plane, the other moves a level ALONG the normal — while a
reading both displace, a side vertex or `Bounds`, sums them. The per-end pair
is spelled `prismPayload.Z0Delta`/`Z1Delta`; `cupPayload`'s `ZOpenDelta`,
`ZOuterDelta` and `ZCavDelta` are the cup's own spelling of it, and `capBlendPayload` preserves the pair and composes a
selected end's setback rounding into it. Every
level-derived reading takes it: `h` and the
volume, wall area and centroid built on it, `Bounds`, the mesh bound, the side
vertices and the vertical edge lengths. Extents: `Distance`,
`Symmetric`, and `TwoSided` of distance sides land in increment 1 — the three
whose interval the step's own quantities determine. `ThroughAll` and
`ThroughAllSide` have no finite stop geometry of their own (they stop at the
far side of every body the sweep meets), so they are body-relative exactly
like `ToFace`: all three land in increment 2 with selectors (§7), the stop an
intersection of the sweep direction with analytic target surfaces — closed
form — and `ErrUnsupported` until then. A through-all payload must provide a
directional extent BESIDE the proven displacement that reading's own ends
carry, and the stop consumes both: the level it resolves takes that
displacement as its own axial displacement, so a far side a bracket holds — a
revolved solid's partial-sweep extreme, or a boundary extreme riding a computed
arc radius or a walked endpoint the record does not state (§6) — never publishes
itself as the level it denotes. The in-path
test is decided OUTSIDE that displacement: material beyond the sketch plane by
more than it is met, material short of the plane by more than it is not, and an
interval the displacement straddles the plane with is `ErrUnsupported` rather
than a guessed dependency (`agent-docs/spline-design.md` §6.4). A prism with a nonzero
section displacement still returns `ErrUnsupported`: that term moves a
coordinate IN the plane and the extent reading does not carry it, so the stop
has no stated displacement to charge. A through-all
dependency is ambient at the CALL but never in the RECORD: core §6.2's depends-on rule covers this
case explicitly — the feature call resolves which live bodies actually bound
the stops from live bodies without consuming them. (`ToFaceAngular` is the
revolve analog and lands there, §6/§11.) `extent.ToFaceStop` admits the
selected face and computes the level; `extent.ToFaceAngularStop` admits an
angular stop face and computes its angle. A `ToFace` stop face must be an
analytic Plane whose held normal is exactly parallel to the sweep normal, by a
rational cross product of zero and not a tolerance, whose face states no
normal departure (`NormalBound` 0). A cap carries its producer's
`topo.CapRecord` and an axial displacement. The record is the plane
the cap denotes: the plane-local level `z` carried through the payload frame
`(O, U, V, N)` and the placement `(B, t)`, every held value read as an exact
rational. The plane it denotes is `m·(x − P) = 0` with `m = (B·U) × (B·V)` and
`P = B(O + z·N) + t`, not the held tag, whose origin is a rounded lift and whose
normal is a rounded cross product. It must be an exact level set of the sweep
frame `(O', U', V', N')`: `m·U' = m·V' = 0` in rationals. The stop is then the
single exact level `c = m·(P − O') / (m·N')` (`extent.PlaneStop`), and the cap
this evaluator builds, perpendicular to the sweep at one level, charges no
footprint-dependent departure. The published displacement is the sum of three
terms: the exact distance from the held stop to `c + travel·offset`
(`extent.StopLevelError`, so a rounded tag origin or a non-unit held normal is
charged and never trusted), the offset's conversion bound, and `k` times the
face's `AxialDelta`, with `k = max(|m·(B·N)|, |m|) / |m·N'|` rounded up, which is
exactly 1 when the frames and the placement are exactly orthonormal. A planar
Revolve wall can instead use its `Denoted` record. Every straight meridian end
must have the same exact axial coordinate, and both radial basis vectors must
span one exact level set of the sweep. `extent.RevolvedPlaneStop` computes that
level from the record. An angular Revolve cap can use its `Denoted` record when
the placed axis origin `A`, basis `(E0, E1, W)`, and angular direction
`d = (d0, d1)` are exact interval points. Its plane passes through `A` and is
spanned by `W` and `R = d0·E1 − d1·E0`; hence `m = W × R`. Require
`m·U' = m·V' = 0` and `m·N' ≠ 0`, then take
`c = m·(A − O') / (m·N')`. `StopLevelError` charges the held plane tag's
distance from either denoted level; neither record has a remaining axial
displacement. Any other face is
`ErrUnsupported`, however small its tilt, because nothing would charge
it. That includes a stop body whose placement rounds its normal, a revolve
wall with an uncertain axis or nonconstant meridian level, an angular revolve
cap with an uncertain axis or angle, a cap-loop chamfer band patch, a stop plane
that is not an exact level set
of the sweep, a face of a faceted boolean (a mirrored-loft union, a loft or
cylinder union), a copied cap placed by a non-identity transform
(`CapRecord.Under` keeps the record only under the identity placement), and a
sweep, mitred-sweep, twist or coil end cap and a `Body.Patch` chain patch. A
`Distance` extent reaches them. A nonzero
`WithTaper` builds a draft body over `Distance`, `ToFace`, `ThroughAll`,
`Symmetric`, or `TwoSided`. `agent-docs/draft-design.md` owns the tapered extrude:
its sharp offset family, its one- and two-slab payloads, its refusals and its
staging.

### 5.1 The denoted map

Every prism-family build integrates its readings in plane coordinates, while
the record denotes the plane-coordinate body carried through the linear map
L = B·[U V N]: the frame's held U, V and N and the placement's held basis B,
read as exact rationals (`massmoment.PrismRotation`), the leaves
`proofbound.ExactFrameLiftRound` compares every lifted vertex against. r3
keeps those vectors orthonormal only to a few ulps: `r3.NewFrame` and
`r3.FromBasis` normalize in float64, `IsValid` admits a departure up to 1e-9,
and N is the float cross product of U and V. The plane spanned by (1, 1, 0)
and (−1, 1, 1) reads det L − 1 ≈ 4.8e-17. `massmoment.MapCharge` reads two
numbers off L exactly, each rounded up: ||det L| − 1|, and the orthonormality
defect e, the entrywise absolute sum of LᵀL − I. Every eigenvalue of LᵀL lies
in [1 − e, 1 + e], so L scales an area, and for e ≤ 1 a length, by a factor in
that range, and a volume by exactly |det L|. `prismrecord.ChargeMap` keeps every held
value and widens the bounds: `Volume` by ||det L| − 1|·(|V| + bound), the
body's and every face's `Area` and every edge's `Length` by e·(|x| + bound)
(`proofbound.BoundedStretch`). The prism, the chain extrude, the planar patch,
the draft, the cap blend, the stacked prism (and the cup built on it) and the
class-B brep take it as their last step; a brep's face frames are signed
permutations of its first, so the first face's map stands for all. A face copy
(`Unstitch`, `PatchEdges`) carries its source's readings through its
placement alone and takes the placement's own charge; a `PatchEdges` face
fitted to a rim takes its fitted frame's, and one fitted to a straight-edged
rim instead bounds its area directly against the polygon its own vertices
denote (`patchchain.PolygonAreaBound`), and one with a circular edge edge by edge
through that edge's curve bound (`agent-docs/surface-design.md` §5.2), which also
charges the rim's float re-expression into the fitted frame. A prism-family
cap rim's circle or arc carries that curve bound (`circleCurveBound`): its
centre's exact lift rounding, `massmoment.CircleImageGap`'s tilt and stretch
terms, and the level and section displacements carried through L. `MassProperties` reads the exact image
under the same map (`agent-docs/multibody-dynamics-design.md` §8.1). The centroid takes no charge: an
affine map carries a body's centroid to its image's centroid. The box takes
none either: each reading is the extreme of the linear functional g·L over
the plane-coordinate body, which is exact for any linear map, and the
coefficients (L's columns)·g carry their own rounding charge. A frame and
placement whose bases are exactly orthonormal, every axis-aligned sketch plane
under the identity among them, read det L = 1 and e = 0, and every reading is
then unchanged bit for bit; `Transform` under a general rotation keeps each value
and may widen its bound. A defect at or above 1/2 refuses `ErrUnsupported`.
The loft, the mitred sweep, the faceted boolean and the stitch compute their
readings from world coordinates their own vertex bounds already cover, and
take no charge.

## 6. Revolve

Same recording shape; the axis must be validated non-degenerate and coplanar
with the profile plane, and the axis must not pass through the region's
INTERIOR: the region lies in one closed half-plane of the axis. Boundary
contact is allowed only through an exact axis-incidence audit. At each on-axis
point across all recorded loops, the incident walk ends MUST be exactly one
off-axis walk end and one `LineSeg` end lying ALONG the axis, from the same
loop. The off-axis walk sweeps to one pole/apex fan; the on-axis line sweeps
nothing. A second off-axis sector at that point, repeated loop incidence,
isolated endpoint tangency, missing on-axis continuation, interior curve
tangency, or crossing segment is `ErrDegenerate`, as is a region with interior
on both sides. Thus a circle kissing the axis remains a rejected self-touching
horn when it is segmented into arcs whose shared tangent point is a walk
endpoint; segmentation cannot turn the same geometric incidence into a pole.
This audit is what keeps §10's valid-by-construction claim true.
Faces: a `LineSeg` lying ON the axis emits no face at all — it sweeps a
zero-area set, and the neighboring segments' faces close the solid there;
parallel to the axis (off it) → `Cylinder`; inclined → `Cone` (an
endpoint on the axis is its apex); perpendicular → planar annulus (a disk
when it reaches the axis); `ArcSeg`/`CircleSeg` → `Torus`, or `Sphere` when
the arc's center lies on the axis — an endpoint ON the axis closes at a pole,
and an endpoint off it leaves a latitude-circle edge (a spherical band when
neither endpoint reaches the axis). The free-form segment kinds emit
`NURBSSurface` faces; `agent-docs/spline-design.md` §7 owns the variant and its
exactness, Table C the revolve reach, Table R the refusals, and §10 the revolve
increment.
Shells: a partial sweep's two caps join every wall into one shell. A full turn
has no caps, and an on-axis junction sweeps a point, never an edge, so each
RUN of a loop — its walks between two consecutive on-axis walks, or the whole
loop when it meets the axis along fewer than two — sweeps its own closed
surface and gets its own shell. A hole loop's run bounds a toroidal void. A
loop meeting the axis along k ≥ 2 walks gives k shells: the run whose two axis
ends bracket every other on-axis junction of the loop is the outer one, and
the others are voids (a hollow cylinder's meridian sweeps its skin and its
cavity wall). A surface result marks no shell void (`agent-docs/surface-design.md`
§2.2).
Each END of the sweep interval carries its own proven **angular
displacement** — how far the angle held there sits from the angle the
recorded `AngularExtent` denotes — the angular twin of the axial displacement
§5 states for extrude's swept levels. An angle the caller stated in radians
denotes itself exactly and carries zero; one stated in degrees denotes an
exact fraction of a turn, never the float `math.Pi/180` recovers, since
`units.Degree`'s own factor is a rounded approximation of that fraction; a
`FullRevolution` or a snapped-to-full `AngleExtent` denotes exactly what the
caller's own record states, whether or not that coincides with the held
`2π` the resolver snapped to; and a `ToFaceAngular` stop's resolved angle has
no better claim than itself, since the record names a target face rather
than an angle, so its displacement is unbounded and every reading it feeds
keeps the magnitude envelope it always has. The pair is spelled
`revolvePayload.Phi0`/`Phi1` beside `Den`, their `revolveangle.Angle` twin
(`internal/revolveangle/denotation.go`), and the per-end displacement is
`phi0Delta`/`phi1Delta` (`AngularDelta` for whichever a reading cannot
attribute to one end). Every reading that folds a held sweep angle into a
published measurement takes it: `Bounds`, the partial-sweep cap vertices and
cap-copy seam vertices, the cap faces' own `Plane` normal, the mesh's angular
sampling, the wall survey's cap wedge, and the tolerance gate's reference
diameter. The clearance kernel reads it as one term of its per-body
displacement (`bodyGeom.delta`, `agent-docs/clearance-design.md` §2), as it reads
the prism's axial displacement.
Every junction, seam and cap vertex `buildRevolveLoop` and the chain `Revolve`
build place is bounded by ONE exact comparison per sweep end
(`revolvePayload.sweptVertex`, `revolvemesh.RevolveLift.SweptPointGap`). The
vertex is PLACED from its walk's float axis coordinates (z, ρ), which
`axisFrame.walk` re-expresses from the plane-local point in float and snaps
onto the axis inside the contact tolerance. The comparison instead starts
from the recorded plane-local point (u, v) the vertex denotes, widened by the
bound that reaches both neighbouring walks' denoted ends there (§3's rim
vertex rule; a free end's own walk-end bound), re-expresses
it about the axis anchor and direction widened by `axisInPlane`'s four
bounds, rotates it by that end's certified sine and cosine
(`revolveangle.Angle.SinCosFor`), and lifts it through the frame and the
placement, all over rational intervals with the frame and the placement as
exact leaves. The held point's distance from that enclosure therefore covers
the re-expression's rounding, a snapped radius, the axis's own anchor and
direction error at any angle, the angular displacement and the
trigonometric evaluation, and the frame lift and placement rounding. The
on-axis vertex a partial sweep shares between its two caps denotes the
recorded point at both ends and takes the larger of the two comparisons. An
end with no denotation (a `ToFaceAngular` stop) bounds its vertices at +Inf.
Every interval is a single point for an exactly resolved axis, recorded
coordinates and an exactly stated end, so an integer profile about a
coordinate axis keeps its φ = 0 cap and seam vertices `Exact`. A trimmed
chain `Revolve`'s section displacement (`sectionDelta`) is not in the vertex
bound: `Area`, `Bounds` and the tolerance gate charge it, as they do for the
prism's chain. A whole-section displacement — an offset construction's, which
`sectionWhole` marks — is: the recorded point is widened by it per component
before the comparison (`agent-docs/surface-intersection-design.md` §7.2).
The swept edges' curves are held floats with no bound of their own: a
junction `Circle3`/`Arc3` takes its centre and radius from the walk's (z, ρ)
(`junctionCircle`) and a cap `Arc3` its centre from the walk's (cU, cV).
`internal/brepmodify/revolve_blend.go` matches a selected edge against `junctionCircle` by
float identity over the same payload, so it reads the same numbers and needs
no bound. The clearance kernel reads them through `newCEdge`, and its revolve
carriers read the same walk values, so the displacement it charges for the
carriers (`clearance.RevolveCarrierResult`'s `AxisGap`,
`agent-docs/clearance-design.md` §2) is the one those edges share; they add only
the float evaluation of the centre. STEP export,
`internal/extent/angular_stop.go`'s boundary probes and `internal/stitchrecord/flux.go`'s
circle rims read the held curves as they stand.
A wall's `Face.NormalAt` bound is proven against the surface its record
DENOTES, never against its `Surface` tag. The tag is a float re-expression:
its origin and centre are placed from the walk's (z, ρ), a centre within the
contact tolerance is snapped onto the axis (a `Sphere`), a segment within the
classifier's 1e-9 slope of parallel or perpendicular is tagged a `Cylinder` or
a `Plane`, and a `Cone`'s half angle is a float `atan2` of rounded runs. The
build stores the denoted surface beside the tag (`topo.Face.Denoted`,
`surfacenormal.Revolved`, built by
`revolveaxis.DenotationInput.WallDenotation` through
`revolvemesh.RevolveLift.StraightWallNormal`/`CircularWallNormal`): a
straight wall's recorded segments, each as its exact run (dz, dρ), and a
circular wall's recorded centre, re-expressed about the axis anchor and
direction widened by `axisInPlane`'s four bounds, with the axis frame lifted
through the frame and the placement as exact leaves — the swept-vertex
comparison's leaves. `NormalAt` keeps the arm's float direction and bounds its
distance from the enclosed outward unit normal of that surface at p. A
straight wall's normal is dρ·ẑ − dz·r̂ in the axis frame, the worst over the
recorded segments a coalesced wall covers. A circular wall's normal runs from
the rotated circle's centre at p's azimuth to p. Either is carried to the
world through the cofactor matrix of the placed basis, since that basis is not
exactly orthonormal. The bound therefore covers the re-expression, the snap,
the classification, the angle and every placement rounding at once, and the
tag's own arithmetic proof is not run. It is zero where every leaf is exact and
the unit normal is a coordinate axis, so an integer profile about a coordinate
axis keeps those wall normals `Exact`. Under a whole-section displacement each
recorded leaf is widened by it per component first, so the bound holds the
surface the displaced record denotes. A plain float `atan2` was not seen to
break the tag's own bound on its own; the snapped centre, a near-parallel or
near-perpendicular segment, and a far or tilted axis do
(`apitest/revolve_normal_bound_test.go`). A partial sweep's cap carries its
denoted plane the same way (`revolveaxis.DenotationInput.CapDenotation`,
`revolvemesh.RevolveLift.CapNormal`): the plane through the recorded axis at
the angle its end's record states, with outward normal (−sin φ, cos φ) on
(E0, E1) at the end cap and its negation at the start cap, sin φ and cos φ
the denotation's own certified enclosures. Its tag is spanned from the float
axis direction through the frame's float lift, at the float sine and cosine of
the held angle, and placed in float, and its `normalBound` charges only the
angle; a tilted axis on a tilted plane, a rotation and a reflection each left
a cap up to 1.33× that bound off its record's normal. `NormalAt` proves a cap
carrying the plane against it in place of the tag arm and `normalBound`,
which then stays the figure the gates that refuse a departed face read. An
end with no denotation (a `ToFaceAngular` stop) carries none and keeps the tag
arm composed with its `normalBound`. The start cap at the exact angle zero of
an integer profile about a coordinate axis stays `Exact`. The departure is NOT
a `normalBound`: no one dimensionless figure states it for every p. `Stitch`'s
flux arms integrate the tag, so they admit a revolve face only where its tag
is exactly its denoted surface, and charge a cone's float apex
(`agent-docs/surface-design.md` §6.4).

Partial sweeps get two planar cap faces. Volume by Pappus on the §4 first moments; the solid centroid from the §4
second and mixed moments (`∫u² dA`, `∫uv dA`) — a full revolution's centroid
lies on the axis with its axial position from the mixed moment, and a partial
sweep's is closed form in the sweep angle. The centroid is lifted as
A3 + W·axial (+ the partial sweep's in-plane term), and beside the axial
coordinate's own bound and the lift's rounding at the centroid's magnitude it
charges what that lift owes the axis basis (`revolveCentroidLift.charge`):
the anchor A3's exact frame-lift rounding, which rounds at the frame origin's
and the anchor's magnitudes rather than A3's own, and the axis's proven anchor
and direction displacement carried through A3, W, E0 and E1 at the axial and
in-plane magnitudes. Each is zero for an exactly lifted anchor and an axis
with no bound. `Area` by Pappus's first theorem
per side face — swept arc length × the sweep angle × the segment CURVE's
centroidal radius about the axis, a boundary first moment with closed forms
for `LineSeg`/`ArcSeg`/`CircleSeg` — plus the §4 region area for each partial-
sweep cap.
Every one of those readings integrates the RECORDED region while every face is
built from the SNAPPED one — `axisFrame.walk` assigns an endpoint within the
contact tolerance exactly 0 — so each carries the SNAP's own charge, and each
carries it where the reading is taken. A wall's own area takes it on the wall
LENGTH, because moving an endpoint onto the axis moves the wall's two ends apart
by at most the sum of the two discarded radial magnitudes. `Volume`, the cap
area and the centroid take it on the region integrals themselves
(`regionSnapAllow`): the snapped and recorded boundaries bound a ribbon of
radial width the discarded magnitude along each walk, so each integral moves by
at most that ribbon's area times the integrand's own proven envelope — nothing
for `∫dA`, the walk's radial envelope for `∫ρ dA`, and a radial beside an axial
one for `∫zρ dA`, all read in AXIS coordinates so a profile far down the axis
pays its large `|z|` only where `z` appears. Every one of those charges is
exactly zero for a profile whose on-axis endpoints already sit on the axis,
which is what leaves an ordinary axis-incident revolve as `Exact` as before.
A payload whose record sits within a proven `sectionDelta` of the meridian it
denotes charges that distance beside the snap, into the same three readings
and the region's own area (`agent-docs/surface-intersection-design.md` §7.2).
Every one of those readings is taken in plane coordinates and takes §5.1's
frame charge (`revolvePayload.frameCharge`), over the map L = B·[U V U×V]: the
third column is the exact cross product, because the sweep's own E1 = W × E0
is one and the swept-vertex comparison reads it so. Every wall's and cap's
area, in `Revolve` over a profile and over a chain alike, and every latitude, junction-arc
and cap-edge length widen by e, and `Volume` by ||det L| − 1|. The centroid
lift A3 + W·axial (+ the in-plane term) is L applied to the plane-coordinate
centroid, so it takes no charge. The tilted plane of §5.1 reads
det L − 1 ≈ 4.1e-16 here. `MassProperties` reads the exact image under the
same L (`agent-docs/multibody-dynamics-design.md` §8.6).
`Bounds` from per-face analytic extremes: each face's radial
extreme about the axis (a cylinder's radius, a cone's two end radii, a
torus/sphere's center distance ± minor/radius) and axial range, with a
partial sweep's angular interval deciding which cardinal directions are
reached — the same extreme analysis extrude uses, in cylindrical coordinates.
`Bounds` is `Exact` only where every one of those extremes is proven exactly
representable; a sweep amplitude no `float64` holds, a boundary extreme a
computed arc radius or a computed walk endpoint carries, the boundary scan's own
`gu·u + gv·v` arithmetic, the axis frame or
the placement's own rounding, or the reading's own summation of those terms
into a published endpoint, publishes the proven bound its own arithmetic
derives instead. The scan's arithmetic is charged apart from the extremes it
reads, at the SECTION's own coordinate envelope: a placement makes both
coefficients non-trivial, so multiplying them by a section coordinate rounds
even where every candidate position is a value the record states verbatim and
the extremes' own term is zero. No other term in the reading scales with that
magnitude, and it is zero wherever one coefficient is zero and the other is `0`
or `±1`, which is the axis-aligned unplaced case. The axis frame contributes
four terms. The first is its
resolved direction and anchor's own proven displacement (`axisInPlane`'s
`dUBound`/`dVBound`/`aUBound`/`aVBound`, already folded into the region's
moments by `revolvemass.AxisMoments`, and now into `Bounds` and the meridian
minimum-radius survey the same way), and each of those four is the ROUNDING
its own evaluation committed, proven over the rationals — never a magnitude
envelope over the value it bounds, which for an anchor would grow with the
axis's distance from the frame origin while the projection's own error stayed
zero. The second is, like the placement, the rounding `xform.Apply`/
`xform.ApplyDir` commit whenever the frame is not axis-aligned or the
placement is not the identity. The third is the extreme reading's OWN anchor
shift — the products and the subtraction that carry a plane-local extreme into
axis coordinates — which rounds at the anchor's magnitude rather than the
section's, so a far-offset axis rounds here even where the boundary scan
reports zero. The fourth is the float sweep basis itself: the coefficients
are read off the held A3, W, E0 and E1 (`RevolveLift.Basis`), while the record
denotes the basis built from the exact frame and axis leaves with E1 the exact
cross product, so `RevolveLift.BasisRound`'s four exact L1 gaps are charged at
the rates the coefficients move the extreme (`revolveaxis.FrameRoundAllow`).
The frame's departure from orthonormal enters the box through these
coefficients alone (§5.1). The endpoint summation is charged separately from all four,
since a pure translation leaves all four coefficients exactly right and rounds
only when they are added. Every one of these terms is zero for an axis-aligned,
unplaced revolve whose anchor projects and shifts exactly, which keeps the
ordinary revolve's box Exact as before.
A walk endpoint is a computed one
wherever the record does not state it: a trimmed line's or arc's own bound, and
every circle's, is evaluated rather than read, and the bound it carries is the
per-component gap from a certified enclosure of the point the record denotes.
That bound belongs to the directional extent reading itself,
not to the box, because a body-relative stop reads the same extent (§5) and
must see the same uncertainty rather than an exact coordinate. All mass results
carry the §4 numerical bounds. Increment 2.

## 7. Selectors

Resolution is a filter pipeline over the live body's topology: gather
(`Body.Edges()`/`Faces()`), apply each predicate as a pure function of the
analytic data (`Edge.IsConvex`, `Curve`/`Surface` variants, lengths, normals),
then enforce cardinality (`Exactly`/`AtLeast`; `ErrCardinality` at zero when
asserted, else `ErrNoMatch` — core §12 precedence). Predicates on a `Faceted`
body read the facet data the body holds; a predicate that needs analytic
identity a faceted face no longer has (e.g. `Cylindrical()`) simply does not
match it — matching is decided on what the face IS, never guessed from what
it approximates. Queries and predicates are recordable values (core §6.2);
resolution happens at feature-call time and at re-evaluation identically,
because provenance roles are stable (§3).

## 8. Document state and re-evaluation

`Document` owns the live body set, a private monotonic producer identity used for topology provenance, and successful public operation records. Its node (`topo.Document`) holds the live set and counters; root publishes records under the same membership lock. Builders mint level and curve identities through it (`MintLevel`, `MintCurve`). Every feature call validates live inputs, evaluates geometry without mutation, and commits only after success. Commit retires consumed bodies, registers the result, advances the producer identity and publishes its operation record atomically. Cancellation and every other failure leave the live body set, producer identity and operation history unchanged. A failed build may have advanced the level and curve counters; they only increase, so no identity is reused, and an identity a failed build minted appears on no committed body.

The mesh-built `Cut` may also carry a strict-containment Volume certificate. When
the source-derived subtraction agrees with the ordinary mesh interval and has a
strictly smaller bound, `Body.Volume` and `Verify` use the source reading. The
certificate retains bounded source readings and proof values, not topology or
document handles. It does not change the mesh, its bounds, closure, orientation,
or occupied-volume proof. See `agent-docs/operation-history-design.md` for its
admission and interval rules.

`Remove` is commit's retire step with no feature around it: it gates the body exactly as
`requireLive` does, retires it, and registers nothing, so the producer identity does not advance.
The live body set (the node's `LiveSet`) sits behind the document node's own lock (`LiveMu`).
`commitOperation*` helpers and `Remove` change it under that lock, and every reader —
`Bodies`, `Verify`, `VerifyMotion`, the `ThroughAll` stop scan and the liveness gates, all through
the node's `IsLive` and `Live` — reads it under the same lock, and a reader that walks the set walks
its own snapshot. Only root writes the set and the producer counter, and only root takes the write
lock: `TestTopoDocumentWriteGuard` fails on an internal package that writes `LiveSet`,
`NextProducer`, `NextLevel` or `NextCurve` or locks `LiveMu` for writing, and the counters' own
increments live in `MintLevel` and `MintCurve`. That is what lets a `Remove` run beside a `Verify`. The lock guards the set alone: the identity counters and a
feature's check-then-commit are still unguarded, which is why the document as a whole stays unsafe
for concurrent mutation (core §12).

Every payload implements `topo.Payload`, and the body node holds it
(`Payload`). `Placed` takes the builder environment `topo.Env` (the document
node, which mints level and curve identities and owns the result, the
producer the result's roles are minted under, and the node allocator `Alloc`)
and returns a body node; it never commits. `Transform`, `Copy`, a pattern
instance and a motion check reach it through `placePayload`
(`topology_handles.go`), which builds the environment (`envOf`) and maps the
node back to its handle (`bodyResult`); the caller commits. A builder in an
internal package links nodes only and allocates every node through
`env.Alloc`: root's `handleAlloc` returns the node of a freshly born handle,
so each node's `Owner` is set before the builder stores a pointer to it, and
internal unit tests allocate bare nodes (`topotest.Alloc`). `prismPayload` is
an alias of `prismrecord.Payload`: its `Placed` sets `Xform` on its own value
copy and runs `prismrecord.Build`, which every other root caller of the prism
builder calls through `envOf` too. `stackedPrismPayload` is an alias of
`stackedrecord.Payload` in the same way: its `Placed` runs `stackedrecord.Build`
under the payload's own plan. `cupPayload` is an alias of `cupwall.Payload`:
its `Placed` runs `cupwall.Build`, which builds through `stackedrecord.Build`
under the cup's own `stackedrecord.Plan`. `chainPayload` is an alias of `chainrecord.Payload`
likewise: its `Placed` runs `chainrecord.Build`. `patchPayload` and `bodyPatchPayload`
are aliases of `patchchain.Payload` and `patchchain.BodyPayload`: their `Placed` runs
`patchchain.Build` and `patchchain.BuildBody`. `revolvePayload` and
`chainRevolvePayload` are aliases of `revolveplan.Payload` and
`revolveplan.ChainPayload`: their `Placed` runs `revolveplan.Build` and
`revolveplan.BuildChain`. `mitredSweepPayload` is an alias of
`sweepmitre.Payload`: its `Placed` runs `sweepmitre.Build`. `coilPayload` is an alias of
`coilshell.Payload`: its `Placed` runs `coilshell.BuildBody`. `loftPayload` and
`chainLoftPayload` are aliases of `loftrecord.Payload` and `loftrecord.ChainPayload`:
their `Placed` runs `loftrecord.Build` and `loftrecord.BuildChain`. `capBlendPayload` is an alias of
`capblend.Payload`: its `Placed` runs `capblend.Build`. `allEdgeChamferPayload` is an alias of
`breprecord.AllEdgeChamferPayload`: its `Placed` runs `breprecord.BuildAllEdgeChamfer`. `brepPayload` is an
alias of `breprecord.Payload`: its `Placed` runs `breprecord.Build`. `twistedSweepPayload` and
`compositeTwistedSweepPayload` are aliases of `twistsweep.Payload` and `twistsweep.CompositePayload`:
their `Placed` runs `twistsweep.Eval`. `draftPayload` and `twoSidedDraftPayload` are aliases of
`draftrecord.Payload` and `draftrecord.TwoSidedPayload`: their `Placed` runs `draftrecord.Build` and
`draftrecord.BuildTwoSided`. `sweepPayload` and `chainSweepPayload` are aliases of `sweeprecord.Payload` and
`sweeprecord.ChainPayload`: their `Placed` rebuilds the one-span reduction and runs a `sweeprecord` finishing
step; a profile composite replays its spans and a chain composite refuses. `stitchPayload` and
`unstitchPayload` are aliases of `stitchrecord.Payload` and `stitchrecord.UnstitchPayload`: their `Placed`
runs `stitchrecord.Build` and `stitchrecord.BuildUnstitchFace`. `facetedPayload` is an alias of
`facetedrecord.Payload` (`facetGroup` of `facetedrecord.Group`, and each faceted source record of its
`facetedrecord` type): its `Placed` runs `PlacedWith`, which places the held mesh and runs
`facetedrecord.Build`. `facetedrecord.BuildWithProof` runs `facetedrecord.AssembleBody` over the held mesh
and its source groups, then stores the payload with the returned diameter and facet-to-face map on the
body. Every root payload name is such an alias; no payload type is declared in the root package.

`Transform` and `Copy` pass the caller's context through payload re-evaluation.
Faceted placement polls that context while transforming vertices, auditing and
rebuilding topology, and recomputing measurements.
For a faceted payload, the stored binary64 placement basis is read as an
exact rational map. Its certified length stretch multiplies every old vertex
bound; its determinant charge multiplies the old occupied-volume gap. The
placed held vertices' coordinate-box diagonal plus twice their new boundary
bound supplies the centroid's diameter after vertex rounding. Area
slack gains the old slack's stretch, twice the held mesh area's stretch to
cover different true and held tangent planes, and each rounded facet's area
allowance. Each held vertex starts at the nearest float to its exact rational
image under that map, before the embedding check can move it to another
corner of the image's float box. Vertex rounding and a swept-volume allowance
are added after the exact map. A non-finite bound refuses the placement.
A faceted copy made with `WithCopyRotation` instead denotes an exactly
orthogonal rational rotation `Q` with translation `t`, recorded at Copy time.
Its held vertices are placed by the same float basis, and each vertex pays
`|Q - D|` times its largest coordinate plus `|t - t_D|` in place of the length
stretch and determinant charge, because `Q` keeps length, area and volume. The
group centroids are re-derived as `Q*c + t` in rational arithmetic with no
stretch charge. Every record that describes the float placement instead of `Q`
is dropped: every group denoted patch, every non-retained group surface tag,
every `CapRecord` but a fixed plane's, `PointSection`,
`PointPlane`, `PointSimilarity`, the point-plane cap displacement,
`RevolveLoftSector` and `LoftMirrorCopy`. The cap-role provenance of a loft
mirror join is a separate record, so each exact copy still mints both outer-cap
roles under its own identity. A retained group tag stays as a float image with a
charged bound; a retained plane with a zero `NormalBound` that `Q` maps onto
itself keeps its tag, `CapRecord` and axial bound exactly (mirror-pattern §4.4). The zero-bound exact source mesh survives only an
exactly identity record. The copy lineage keeps the rotation only for a copy
made directly from the source's own payload; `Transform` and a copy of a copy
record none. A direct exact-rotation Copy of an admitted contained Cut also
attaches `ExactCopyMoments`, which the faceted mass reader transports and
compares against the audited mesh. Later placements and rewrites drop it.
Every exact placement sets the payload's `ExactPlaced` mark, and
every later placement, `Copy`, `Transform` and `ScaledAt` keeps it. The motion
checks (`VerifyMotion`, `VerifyLinkage`) refuse to re-place a marked mover with
`ErrUnsupported`, because their pose deviation model does not describe a
re-placed `Q*S + t`.
When a zero-bound faceted Boolean undergoes translation-only placement, its
payload also carries the original exact mesh beside the rebuilt held mesh.
The contact reader may use that source to prove an exact support plane after
checking the placed vertices against their published displacement bound.
A rotated placement and a Boolean with inherited positive error carry no such
source certificate.

`Transform` maps analytic geometry exactly in EXACT arithmetic — every
v1 surface variant maps to itself under an isometry (plane→plane,
cylinder→cylinder, …), with `IsReflection` flipping face orientation handling
— but the FLOAT evaluation of the frame lift and the isometry rounds wherever
the frame is not axis-aligned, the placement is not the identity, or the
frame origin and a coordinate do not add exactly. The held placement basis
is itself orthonormal only to rounding, so a placed body's volume, area and
length readings also take §5.1's map charge. Every reading built from
`xform.Apply`/`xform.ApplyDir` therefore carries that rounding as its own
proven displacement rather than reading the placed coordinate as an exact
leaf: `prismPayload`/`capBlendPayload`'s `Bounds` (§5) and
`revolvePayload`'s (§6) each charge it, composed outward with whatever other
displacement the same reading already carries — and beside it each charges the
rounding of its OWN recombination of the placed terms into a published
coordinate, which a pure translation commits even where the isometry's float
evaluation rounded nothing. `Vertex.Position` takes the identical charge for
every rim, junction and cap-level vertex `prismPayload`, `revolvePayload` and
`capBlendPayload` place, measured exactly per vertex (`ExactFrameLiftRound`,
`RevolveLift.SweptPointGap`; §5, §6): a vertex sits at a plane-local
coordinate lifted through the payload's own frame and then its placement, the
same two-step map `Bounds` reads, so a tilted frame or an origin the
coordinate does not add to exactly rounds it under the identity placement
exactly as it rounds a box extreme — the one difference from `Bounds` being
that a vertex takes no separate final-summation charge, since its exact
measurement already includes the placement's own sums. The clearance kernel's
carriers take the same per-point measurement for every recorded anchor they
lift (`clearance.CFace.LiftRound`, folded into `bodyGeom.delta`).
A DIRECTION carries no origin and no translation, so the directions every
`dir`-style construction builds keep `proofbound.DirRoundAllow`'s magnitude
charge, zero exactly for an axis-aligned frame under the identity placement.

A placement changes the MOTION and nothing else, so a prism's re-evaluation
reuses the plane-local walk resolution the original build published rather than
resolving the same section again. `prismPayload` carries a private
`*momentinput.ProfileWalks` (`Walks`) for its own record; `prismrecord.Build` publishes the set it
resolved onto the body it just built, and `Placed` — the one path that carries a
payload forward unchanged but for `Xform` — hands it back to the next build.
What makes it sound is that a walk holds nothing placement-dependent: `walkOf`
answers in the section's own (u, v), and the frame, the placement and the
reflection's orientation and winding corrections are all applied by consumers
DOWNSTREAM of it. So a reflected copy still runs every correction it always ran;
the arc-length bracketing and completed plane-local moment integration are reused.

`ProfileWalks` also owns the successful integration at its recorded moment order
and a deep copy of the source record as that result's identity key. Every read
runs the original field validation and charges before checking the key. A matching
record and order return owned copies of all rational fields; a changed record,
different order, or absent result integrates normally. Publishing a result creates
a new snapshot; existing walk caches are never mutated. This preserves concurrent
placement reads, work-limit failures and their metadata, and source-array edits
cannot silently change the cached result's key.

A tessellation reads the same published set. `tessellateContext`'s prism arm
chords the very loops the build resolved, so it replays the recorded charge onto
its own counter and hands the walks to `chordLoop`, which reads them exactly as
`prismrecord.BuildLoopSidesAs` does. It is where the resolution dominates: chording a
free-form section would otherwise spend almost all of its time in the
arc-length bracket under `walkOf`, while the chording itself — `chainStations`
over the Bézier chain the walk already holds — is cheap.

Reuse is decided by the record, never by the caller. `momentinput.ProfileWalks.Reusable`
admits a set only when it was resolved from a bit-identical `ProfileRecord`
(`Matches`, compared by float BITS) and measured its own work charge, and every
other payload — a plain extrude, a modify op's rewritten section, a boolean
result, a cup's derived region — carries none and resolves as before. A record
that changed in any way, including a rescale or a gained hole, is a different
record and resolves afresh. The reader is still CHARGED what the resolution
cost (`agent-docs/spline-design.md` §5.2), so the free-form work ceiling refuses the
same records it always refused, with the same error.

Feature tests cover every example model and operation. Repeated construction
preserves live-body order and provenance roles. Later evaluators may build a
different topology split but must preserve role/query meaning.

### Positive uniform scale of a prism, stack, solid Revolve, or faceted solid

`ScaledAt` admits a solid `prismPayload` whose outer and hole loops contain
only whole `LineSeg`, `CircleSeg`, and `ArcSeg` records and whose section and
sweep levels carry zero displacement. A `CircleSeg` fills its loop alone.
Rigid placement of such a prism is admitted. The source prism's
`Tessellate(VerifyAll)` result must report both boundary and occupied-volume
proofs. A sheet, other curve kind, trimmed segment, displaced section or level,
and an incomplete source proof return `ErrUnsupported`.

An all-line source uses a 0.1 mm tessellation chord. A source with a circle or
arc uses `nextafter(0.05/max(1, factor), 0)` mm. The smaller target leaves
room for source chording after scale and held-vertex rounding; an unrepresentable
target refuses. A later scale of the faceted result does not rechord it, so a
caller may need a tolerance above the accumulated `Mesh.Bound()`.

The verified source mesh already has shared-edge, closed and embedded topology.
A positive similarity preserves its exact topology. `facetedrecord.ScaleImage`
admits the source and builds the image payload; it calls `facetproof.ScaleMesh`,
which maps every held vertex through an exact rational similarity, audits the rounded
image for embedding, multiplies each source vertex displacement by the scale
(including each circular face's chord departure derived from its source bound),
and adds the image's rounding. It multiplies the source area allowance by the
scale squared and the occupied-volume difference by the scale cubed, then
charges the rounded boundary's area and swept volume. `facetedrecord.Build`
audits the image's topology and rebuilds bounded volume and area readings.
The centroid's diameter allowance takes the larger of the scaled source
diameter and the rounded image's held coordinate-box diagonal plus twice
its mesh displacement. This includes any vertex movement in the image.
The source-face groups keep their origin roles; each planar frame's origin is
scaled around the caller's anchor. A circular source wall becomes a `Faceted`
face with the inherited boundary bound; this path has no scaled curved-normal
certificate. The result is not a new `prismPayload`.

A solid `revolvePayload`, complete or partial, with zero section displacement
takes the same transfer after its own `Tessellate(VerifyAll)` proves boundary
and occupied volume. That tessellation's facet-pair contact audit supplies the
embedding evidence for the held mesh. The source mesh's occupied-volume bound
already covers the partial sweep, and the image multiplies it by the factor
cubed. Its initial chord is `nextafter(0.05/max(1, factor), 0)` mm. The mesh
keeps each source face's origin roles. Planar faces keep scaled plane tags;
curved faces publish `Faceted`, including a sphere's one face with no edge or
loop. The faceted topology builder admits that zero-boundary case only for a
closed, edge-connected, single-face mesh. That face has zero rim perimeter;
the source mesh's proved `areaSlack`, scaled by the factor squared, covers its
curved area difference without a rim term. A sheet Revolve, and any
source without complete mesh proofs, refuses
without retiring the source.

The faceted payload marks this source class so a later `Transform`, `Copy` or
`ScaledAt` can repeat the proof from its held mesh and bounds.
Booleans and modify rewrites do not inherit that source marker. A failed
embedding, non-finite bound or canceled context returns without retiring the
receiver.

An exact line-only `stackedPrismPayload`, or an A1 `brepPayload` that keeps its
stack, takes this transfer when it is a solid with zero section and level
displacements. A BRep also needs line-only planar and swept faces, no blend or
shell band, and zero side-line split displacement. The source's own
`Tessellate(VerifyAll)` must supply the complete boundary and occupied-volume
proof at 0.1 mm. An exact facet-pair audit checks the held mesh for contact
before `facetproof.ScaleMesh` maps its vertices. The image keeps each source
face's origin roles and planar tag. It becomes `facetedPayload`, so later
structural stack and BRep operations do not read its old, unscaled records.
The source remains live if any authentication, mesh proof or similarity fails.

The generic mesh `Union`, `Cut`, and `Intersect` also mark their faceted result
after the exact-predicate topology audit and occupied-volume reading succeed.
Their payload stores the stitched triangles, per-vertex boundary bounds,
occupied-volume difference, area slack, source-group ids and original face
roles. Before scale, `facetedrecord.CertifiedFacetedSimilaritySource` requires
a solid result, finite nonnegative proof terms, one bound per vertex, one valid
source group and built face id per triangle, and a mesh bound at least as large
as every facet's vertex bound. It refuses a source group without an origin
role. `facetproof.ScaleMesh` then applies the same rational similarity,
embedding audit and area/volume charges to the Boolean mesh. A retained
`Plane`, `Cylinder`, `Sphere` or `Torus` tag keeps its similar image
(`agent-docs/retained-modify-reach-design.md` §7). A retained `Cone`, every
denoted surface and every unretained curved tag read `Faceted`: none has a
similarity image here, and `Faceted` claims less than the source tag did. The scaled
result carries source-face roles and its plane tags, but drops
structural Boolean records whose original coordinates would no longer name
the scaled body. A rigid placement and another positive scale can repeat this
proof; a failed source gate, embedding audit or build leaves the receiver live.

Every other faceted solid has no similarity mark: a `LoftSections` or
`LoftFromPoint` result, `LoftFaces`, a structural join, a `Fillet` or `Chamfer`
of a mesh Boolean result that keeps its retained tags (`internal/retainedblend`),
any other modify rewrite that rebuilds a faceted record, and the images of
these under placement or scale. `ScaledAt` certifies such a
source when it is called. It runs the same `CertifiedFacetedSimilaritySource`
field checks, then `massmoment.AuditMesh` over the held mesh: shell closure and
orientation, vertex links, and the exact facet-pair crossing audit
(`loftmesh.LoftCrossingAudit`) unless the producer recorded `ContactAudited`.
These checks only refuse. The proof terms they read are the ones every
faceted reading already trusts: `Restate` publishes them as a complete
boundary and occupied-volume proof, and a Boolean takes them as an operand
proof. The audit supplies the embedded source mesh that `ScaleMesh` assumes.
The image then takes the transfer above. Plane tags and retained `Cylinder`,
`Sphere` and `Torus` tags scale to their similar images, so a retained
blend's cylinders and tori keep their kind with scaled radii. A retained
`Cone`, every denoted tag and every unretained curved tag read `Faceted`.
Every structural source record is dropped, including a point loft's
`PointSection` and any point-plane record that is not a Split slab's. A sheet
has no faceted record, so it does not reach this path. A failed shell,
orientation or vertex-link audit refuses. When the producer did not record
`ContactAudited`, the crossing audit also refuses a mesh above
`loftmesh.MaxLoftAuditTriangles` (65,528) triangles or above its pair-test
ceiling, so those ceilings bound its work. A failed field check or audit
leaves the receiver live.

## 9. The boolean — where exactness dies, and how

Increment 4, the deep end. Strategy:

`agent-docs/prism-boolean-design.md` is the approved analytic reduction
`performBoolean` dispatches, ahead of the tessellation path below, for
co-directional prism pairs on one plane or on shared-axis offset planes:
`Union`'s select-all/merge/chain path and
`Cut`/`Intersect`'s clean-nesting structural match. `Verify`'s own interference
evaluation (below) dispatches that same reduction for intersection through
`booleandispatch.AnalyticIntersect`, a read-only twin that builds the admitted payload
and never commits, so it consumes neither operand. `evaluateBoolean` remains
this section's mesh path: a pair neither dispatch admits falls back to it
unchanged, as does every other operation.

**Interference evaluation MUST be read-only; committing a public boolean stays
a wrapper.** Interference PR 1 (`agent-docs/interference-design.md` §11) factors one
internal `evaluateBoolean(ctx, op, a, b)` over tessellation, exact-predicate
classification, cutting, stitching, bound composition, and the rational volume
integral. It never advances provenance, retires an operand, or registers a body.
`Union` / `Cut` / `Intersect`
gate their operands, pass the caller context through evaluation and faceted-body
construction, then commit the step atomically. `Verify`
passes its own context to the read-only analytic intersection twin first, and
to this same mesh evaluator for a pair that twin does not admit; it consumes
only a bounded intersection volume from whichever path answers, never calls
public `Intersect`, and never builds a transient document body
(`agent-docs/interference-design.md` §5, §5.2). This split is implemented; public
consuming behavior is unchanged.

Expected geometric non-results — empty held intersection, an undecidable
contact arrangement, and `ErrUnsupported` staging — are private typed outcomes:
the public wrapper maps them to its existing errors, while `Verify` maps them to
an undecided pair. An invariant failure in source mapping, closure, orientation,
or exact classification remains an error from `Verify`; it is never hidden as
`Suspect`. The shared evaluator checks `ctx` at phase boundaries and at bounded
work intervals inside quadratic/refinement loops, as interference §7 specifies.

- **Tessellate both operands** with an evaluator-internal chord tolerance —
  a documented default derived from the pair's own diameter, raised past either
  operand's own section displacement, which no mesh of that operand can go
  below (`agent-docs/prism-boolean-design.md` §7, tessellation §5), and past a coil's,
  mitred sweep's, or twisted sweep's held `delta`, or a certified
  `LoftFaces` result's held `MeshBound`, which coarsens the partner's mesh but
  leaves its published bounds proven (`agent-docs/faceted-vertex-bounds-design.md`
  §5). An operand that
  restates a held mesh (a boolean result, coil, mitred sweep, or twisted sweep)
  is asked at that
  tolerance raised to its own held floor, which its restatement always meets,
  so the request never refuses; which of its facets the pair may cut is the
  rim bullet's gate below (`agent-docs/faceted-vertex-bounds-design.md` §5). A pair the gate
  refuses is retried from the source bodies of its chain when the chain regroup applies
  (`agent-docs/boolean-chain-regroup-design.md`). A route that moves accepted geometry
  MUST record an `opdiagnostic.Record` with a `Change` mechanism, because the regroup
  reads that field to tell a result that denotes its operation exactly from one that does not.
  `Verify`'s read-only intersection instead meshes each other body once per
  call, at the least tolerance any of its candidate pairs derives
  (`agent-docs/interference-design.md` §5.3). The booleans of core §8 expose no
  tolerance parameter, on purpose. What IS caller-visible
  is the proven bound the output carries: the tolerance's whole effect
  surfaces as `Bound`/`Exactness`, judged by the caller's `WithTolerance` at
  Verify. The machinery and its payload staging are
  `agent-docs/tessellation-design.md`'s: every mesh carries a two-sided boundary
  bound, source faces, and area slack. A boolean-admissible mesh additionally
  carries a proven occupied-volume symmetric-difference allowance. Export
  needs the boundary proof; a boolean rejects an operand that does not carry
  the occupied-volume proof. `internal/booloperand/` reads these per-operand
  answers from the operand's own record: its share of the pair tolerance, its
  held floor, and the payload-class precheck that refuses an operand with no
  occupied-volume proof before it is meshed.
- **Robust mesh boolean in-repo, stdlib-only.** The curated-dependency rule
  stands: no third-party mesh kernel. The algorithm is the exact-predicate
  route: triangle/triangle intersection and point classification decided by
  adaptive-precision sign tests that fall back to exact rational arithmetic
  exactly at the boundary cases — a sign decided exactly is a topology
  decision that cannot flip (core §2.1's whole fear), so the output is
  watertight **by construction** on the tessellated geometry. The general
  exact fallback is carried as homogeneous integer coordinates — an integer
  numerator triple over one shared positive denominator — rather than
  `math/big.Rat`: every predicate is a homogeneous form of fixed degree in the
  differences, so its sign is invariant under scaling by a positive
  denominator, and the exactness guarantee is unchanged. A point is reduced to
  its canonical form only at vertex emission, because welding is by exact
  identity and a homogeneous point has many spellings. Retriangulation along
  intersection curves, classification by exact winding tests, stitching by
  shared exact vertices.

  An uncertain binary64 orientation first tries an exact floating expansion
  when its coordinate differences are exactly representable and within
  `proof.OrientSignExpansion`'s bounds. Product remainders and error-free sums
  give the exact determinant sign. Rounded differences and out-of-range inputs
  use the general integer fallback. Rational source points use their integer
  planes directly; an expansion of rounded source coordinates cannot decide them.
- **Subdivide cut facets in bounded batches.** Each facet owns its cutter and
  cancellation budget. Independent subdivisions run concurrently; region
  classification, point-key caching and facet emission run on the caller in
  facet order. Keep each subdivision error in its facet's result until that
  result is consumed, so a later subdivision error cannot replace an earlier
  classification error. Join every worker before returning, including on
  cancellation. `internal/meshbool/boolean_cut_batch.go` owns worker selection;
  `keepSide` owns the batch bound. `cutProjectionScratch` defines reusable
  projection-buffer ownership.
- **One symmetric classifier per facet pair.** Two closed triangles are convex
  sets, so their intersection is empty, a point, a segment, or a 2-D region —
  and the pair pass computes exactly *which*, never "how many of A's vertices
  sit on B's plane, and whose geometry do I look on". A 2-D region is a
  coplanar face-on-face tangency: classifying material side over every
  positive-area coplanar patch is the designed route
  (`agent-docs/interference-design.md` §5.2), staged as that document's §11 PR4, and
  this evaluator refuses the case until that increment lands. A point is
  carried and refused unless some crossing chain owns it as an endpoint; a
  segment is a rim. When the operands name valid solids, an unlanded 2-D
  region and an unowned point are contacts this evaluator refuses, so the
  boolean surfaces them as
  `BooleanUnsupportedContact` (wrapping `ErrUnsupported`; `agent-docs/api-design.md`
  §8 / H2), never `ErrDegenerate` — which stays reserved for a genuinely
  malformed region, a zero-area or self-crossing one. The classifier still
  DETECTS the region-or-point shape; only the public error a valid such contact
  yields is decided against H2. Which triangle's boundary an endpoint lies on is
  decided by testing the point against *each* triangle, not by which list it was
  drawn from. The classification is direction-free by construction, which is what
  keeps it from disagreeing with itself.
- **Graze versus crossing is not a property of a facet pair.** When a contact
  segment runs ALONG a facet edge, the pair cannot tell whether the operand's
  boundary touches the other's plane and comes back (a graze — a tangency no
  side classification can be proven for, which surfaces on a valid model as
  `BooleanUnsupportedContact` / `ErrUnsupported`, `agent-docs/api-design.md` §8 / H2,
  never `ErrDegenerate`) or genuinely passes
  through it (an ordinary crossing, and a real rim). Only the edge's TWO
  adjacent facets can: their apex vertices strictly on one side is a graze,
  straddling is a crossing. So the pair pass only reports the in-plane edge,
  and the mesh pass makes the call once, with the adjacency in hand. A crossing
  edge subdivides the facet it passes through (never the facet it runs along —
  a segment on a facet's own boundary cuts nothing off it) and its regions
  classify by exact parity, because the other operand's boundary there is a
  DIHEDRAL and one plane of a dihedral decides nothing.
- **A tangency the held facets cannot see is refused, never assumed away.** A
  chord polygon can miss a touch on its analytic patch, and an inherited
  faceted boundary certificate can place the true patch away from its held
  polygon. A pre-pass therefore proves, per current operand FACE pair, that no
  touch can be hiding: if the true patches touch, that point is within δ_A of
  A's facets and δ_B of B's, so the facet sets come within δ_A + δ_B of each
  other. An analytic face's δ is its complete `sourceBound`, including curved
  trim displacement and every coordinate-construction/final-placement rounding
  allowance. A faceted face's δ is the largest facet bound over that face's
  own facets, each the largest of its corners' per-vertex bounds
  (`agent-docs/faceted-vertex-bounds-design.md` §4.4).
  Mesh facets carry this value as internal `sourceBound` (`topo.Mesh.FaceBound`)
  beside their current operand face node (`topo.Mesh.Source`); grouping facets
  for the pre-pass takes the maximum bound for that face. NEVER derive zero
  from `KindFaceted` alone. The pre-pass is `meshbool.RefuseUndecidableProximity`
  (`internal/meshbool/tangency_gate.go`).
  Upward-round `b = δ_A + δ_B`. When `b > 0`, a held-facet meet is decidable
  exactly when the facet sets INTERPENETRATE — cross with a signed penetration
  depth strictly GREATER than `b`. Then the true patches provably cross: A's
  true surface lies within δ_A of A's facets and B's within δ_B of B's, so even
  in the worst case — each true surface pulled back toward its own interior by
  its own bound — the two true surfaces still overlap. The chord error alone,
  bounded by `b`, cannot open a penetration deeper than `b`, so a depth past `b`
  is a proof of true crossing and the pair is ADMITTED. Every other close pair
  is undecidable and refused (`ErrUnsupported`): a pair whose held facets merely
  touch, a pair that overlaps by AT MOST `b`, and a pair whose facets stay apart
  but within `b`. A displacement within the bounds can pull such surfaces apart
  — chording a concave hole wall inward, for one, adds held material into the
  hole and makes truly disjoint surfaces meet — so a held meet at distance 0 or
  a shallow penetration ≤ `b` proves nothing when `b > 0`. The interpenetration
  depth is the implementer's to compute: it is the maximum signed penetration of
  the two facet sets, decided by the exact predicates, never assumed cheaply
  available. A depth past `b` is proven by a WITNESS: an exact point of one
  operand's held facets that exact ray parity places strictly inside the other
  operand, at a certified lower-bound distance greater than `b` from that
  operand's facets. The gate looks for one at each contacting facet's corners,
  edge midpoints and centroid, then at points walked from each contact
  segment's midpoint toward the corners of the segment's two facets, halving
  the step each time. The walk is needed: a long facet can cross the other
  operand deeply while all seven of its fixed points lie outside it. A pair
  with no witness is refused. Reject-only in the undecidable band: it may
  refuse a valid model whose operands genuinely pass that close, and that is
  the accepted price.
  A TANGENCY — facets that touch without crossing — has no positive-bound contact
  certificate in this evaluator increment and stays refused; deciding such a
  pair for real is the clearance kernel's job (`agent-docs/clearance-design.md`), not
  a held facet's. A planar face still carries
  every curved-trim, coordinate-construction, and placement displacement in its
  `sourceBound`; it has zero error only when its held trimmed polygon and stored
  coordinates are both proved exact. A faceted face has zero
  error only when its inherited boundary certificate proves that its true patch
  equals the held polygons (`agent-docs/tessellation-design.md` §2/§7, payload
  verification §5).
- **A strict held-triangle gap uses exact distance when projection gaps are too
  small.** `meshbool.DisjointTriDistanceAbove` first proves the two closed
  triangles disjoint with exact separating axes, including in-plane axes for
  coplanar pairs. It then compares the exact squared distances of all
  vertex-to-face and edge-to-edge candidates with the squared, exact dyadic
  input bound. It returns false for touching, intersecting, degenerate, or
  too-close triangles. A caller still supplies a certified source-surface
  departure bound; distance between held triangles alone does not establish
  clearance between their source surfaces.
- **An operand facet that collapsed is refused, never skipped.** A rigid
  placement's own rounding can flatten a facet of an already-faceted body to
  zero area. Such a facet has no plane and no interior, so every contact
  predicate here is blind to it — a point or tangent contact the other operand
  makes there would be classified by nothing at all — and it must not ride
  through the boolean on its component's verdict. The operand is refused
  (`ErrUnsupported`) rather than examined in part.
- **The final rounding to float64 welds, and what the weld takes it must
  answer for.** Two exact result vertices closer than an ulp round to one held
  vertex, and the facets they span collapse. A collapsed facet is a zero-area
  triangle: it moves neither the volume integral nor the area sum of the held
  mesh, and its two real directed edges cancel, so closure survives it. But the
  facet it stood for was not zero-area *before* the weld, and two things it
  carried have to be accounted for, in two different ways:
  - **The volume and the area it carried are BOUNDED.** The rounding's volume
    error is what its vertex displacement sweeps out over the surface the
    displacement acted ON — the stitched surface *before* any facet was
    dropped from it. Charging it against the mesh that survived would leave the
    dropped facets' own swept volume out of the bound entirely. So the swept
    volume is charged against the pre-round surface, and the area the weld
    dropped — which the held mesh can no longer report — joins the operands'
    chord deficit in the area bound (`internal/proofbound/bounds.go`: `sweptVolumeAllow`,
    `perturbedAreaUpper`).
  - **A component welded out of existence is REFUSED.** When *every* facet of a
    connected component collapses, that whole shell disappears — a lump gone
    from the body, with its volume, its place in `Lumps()` and its reach in the
    bounds box. No bound answers for that: the loss is topological, and the
    closure audit does not catch it, because the components that remain still
    close. It is refused (`ErrUnsupported`). Every other collapse is an edge
    contraction inside a component that survives, and the two bounds above
    cover it.
- **The final rounding to float64 keeps the held mesh EMBEDDED.** The exact
  stitched mesh is embedded: its operands are, and exact subdivision and
  stitching meet facets only along shared exact vertices and edges. Rounding
  can break that. A result vertex closer than an ulp to an edge or facet it does
  not touch — a new rim vertex beside a line of earlier rim vertices is the
  common case — can round to the far side of it, turning a sliver facet inside
  out or pushing its neighbours through another facet. The fold is ulp-sized,
  but a later boolean whose contact crosses it sees a self-crossing contact
  chain and cannot subdivide the facet, and a held mesh that is not embedded
  would publish `BoundaryVerified()` falsely (`agent-docs/tessellation-design.md` §1,
  the Embedding row). So the rounding runs in three steps
  (`internal/meshbool/held_embedding.go`):
  - **Round to nearest, then check.** Each exact vertex rounds to its nearest
    float, and coincident roundings weld as above. A held vertex is MOVED when
    it differs from an exact vertex it stands for, and only a facet with a
    moved vertex can meet anything differently from the exact mesh. Each such
    facet is checked against every held facet whose box overlaps it. For at
    least 4096 query facets, the read-only first scan uses a tree of unchanged
    facet boxes; repair uses a uniform grid whose padded cells still cover
    each permitted vertex move. Smaller checks scan that grid once on the calling
    goroutine without the pair memo, which only repair replays.
    Both scans use exact predicates over the held binary64 values: no shared
    vertex means no contact, one shared vertex means contact only there, one
    shared edge means contact only along it.
  - **Move a failing vertex within its float box.** Each movable vertex of a
    failing pair — one that stands for exactly one exact vertex — may take
    another corner of its exact point's float box: per coordinate the float at
    or below the exact value and the float at or above it, so every corner is
    within an ulp of the exact point per coordinate. The search visits the
    movable vertices of failing pairs in ascending held index; each vertex
    tries its corners in fixed order (x before y before z, the lower float
    before the upper) and moves to the one that leaves the fewest failing
    pairs among its own facets, when that is fewer than where it sits; ties go
    to the earlier corner. A corner another held vertex occupies is skipped,
    so the search never welds. Passes repeat, over the facets that failed or
    that a move touched, until a pass moves nothing. Every move strictly
    lowers the mesh's count of failing pairs, so the search cannot cycle, and
    its order is fixed, so equal inputs give equal held meshes. A welded
    vertex is never moved: no single float box belongs to it.
  - **Move a failing vertex within the gap tolerance.** When the document's
    gap tolerance is positive and the float-box search leaves a fold, the
    same vertices may also sit up to that far from their exact points, along
    the normals of their facets and the coordinate axes, nearest first
    (`agent-docs/gap-tolerance-design.md` §5.1). A mesh the float box clears
    never reaches this stage.
  - **Refuse what the bounded search does not clear.** A mesh still failing
    the exact held-facet checks is refused (`ErrUnsupported`,
    `BooleanExpectedUnsupported`). The evaluator cannot certify an embedded
    held mesh from the candidates this bounded search checked.

  `HeldFolds` runs the same exact check without the repair and returns the
  folded facet pairs, and `RepairHeldEmbedding` runs the repair and returns the
  facets still folded instead of refusing, for a caller that rewrites a held
  mesh and must know what still meets or folds.

  The bounds stay honest without new terms, because the rounding displacement
  is MEASURED: the per-coordinate distance from every exact vertex to the float
  that finally holds it is taken exactly after the search, and that maximum is
  the rounding bound every consumer composes (`StitchedMesh.Round`, then the
  swept-volume and perturbed-area terms above). A vertex the search moved is
  charged where it sits. The float-box stage moves no vertex farther than an
  ulp per coordinate from its exact point, and the gap-tolerance stage none
  farther than the document's gap tolerance.

  A rigid placement of a faceted body (`Transform`) rounds the same way. The exact
  image of an embedded mesh under the motion's float matrix and translation is
  embedded, and each held vertex starts at the nearest float to its exact
  image. The same check runs over the vertices that differ from their exact
  images, a failing vertex may take a corner of its exact image's float box,
  and the measured displacement of every vertex the check moved joins the
  motion's own rounding allowance. So every `Faceted` body is embedded when it
  is created, which is what its tessellation's `BoundaryVerified()` reports.
- **Ordinary mesh output**: faces are `Faceted`, one per CONNECTED PATCH of a current operand
  face. Each patch keeps that face's origins, so provenance (`FaceCreatedBy`)
  and face-level selection survive the boolean — but the operand face is not the
  face. A boolean can cut one operand face into pieces that no longer touch (a blind
  trench crosses a cap and leaves two separate strips of it standing), and each
  piece is its own face, bounded from outside by its own loop. Grouping by
  operand face alone would hand both strips to one face, which then has two outer
  boundaries and can call only one of them outer — reporting the other as a
  *hole* in a patch it is not part of, a wrong topology answer on the surface
  agents traverse. So the key is the patch: facets of one operand face reachable
  from each other across shared edges. Within a patch, which loop bounds it from
  outside is decided, not guessed: on a planar patch the boundary is walked with
  the material on its left, so about the patch's own outward normal the outer
  loop turns positive and every genuine hole turns negative, and the outer
  loop's area vector — the patch's area plus its holes' — is the largest. Never
  the longest perimeter: a serpentine slot can out-measure the boundary it is cut
  into. A curved patch has no such plane and no loop of it is a hole in another
  (a hole wall's two rims bound a tube), so there the longest boundary stands as
  the deterministic bookkeeping choice; validity never reads it. Measurements
  integrate the mesh exactly and report the verification-design
  bound shapes (volume composes the operands' own proven mesh
  symmetric-difference allowances by the set identity; the rim never enters
  that allowance) — `Approximate`, except the §2
  Exact-volume case: an all-planar pair whose contacts round exactly leaves the
  exact volume integral with a genuinely zero bound.
  A structural Loft/cylinder `Union` and its centered bore `Cut` instead tag
  their audited planar cap patches `Plane` and mint result cap roles; their
  other patches use the held facet groups.
  Each patch's group also carries `Group.Source`, an immutable `SourcePatch` (producer, operand, face ordinal,
  parent) that traces origin only and admits nothing; the patch's face holds the same pointer. A retained
  blend's new group carries its own record (modify producer, edge ordinal).
- **The rim is NOT bounded by δ_t.** A vertex an operand's tessellation
  contributed lies on that operand's surface; a vertex the BOOLEAN creates does
  not lie on either. It is the crossing of two chord PLANES, and the true
  intersection curve is anywhere within δ_A of the one and δ_B of the other —
  a tube of half-width **(δ_A + δ_B)/sin θ** about it, θ the crossing angle,
  with δ_A and δ_B the bounds of the two facets whose contact segment ends at
  the vertex and θ THAT pair's crossing angle
  (`agent-docs/faceted-vertex-bounds-design.md` §3.2). So a rim vertex's pre-weld
  bound is that trim-amplified displacement, computed from a proven lower
  bound on sin θ taken exactly from the two facet normals, and a surviving
  operand vertex keeps its operand's own bound. Each vertex then adds, with
  upward rounding, its own displacement from the exact stitched point to its
  stored welded binary64 coordinate (§3.4 there); the global boundary
  certificate takes the largest facet bound, each the largest of its corners'
  complete vertex bounds. Each boundary measurement reads the bounds of the
  geometry it measures (§4.2 there): `Vertex.Position` its own vertex bound,
  `Faceted.Bound` the largest facet bound over its face, `FacetedCurve.Bound`
  the largest vertex bound along its chain, `Box` each extreme's own reach,
  and every area bound the smaller of its face's bound times its perimeter and
  the sum of its facets' perturbations at their own bounds. The bound has no
  finite ceiling as the operands approach tangency: when the inflated bound
  reaches the pair's own diameter it has stopped bounding anything, and the
  operation is refused (`ErrUnsupported`) rather than reported with a number
  nobody can use — per rim vertex, at its own bound. Its ceiling in `δ` is
  certified per pair by the local chain-depth gate
  (`agent-docs/faceted-vertex-bounds-design.md` §5, core §8 "The chain depth"):
  after the contact classification and before any facet is cut, every facet
  of a restating operand that meets the other operand, or comes within the
  hidden-tangency pre-pass's slack of it, must carry `δ(t) ≤ tol`, the pair's
  chord tolerance; otherwise the boolean is refused with a plain
  `ErrUnsupported` (`meshbool.BooleanExpectedStaging`) naming the operand, the
  touched bound and `tol`. So a restating operand contributes at most `tol`
  to any rim, as a freshly chorded operand does, and a rim between two such
  facets carries at most `2·tol/sin θ` of its own facet pair before its weld.
  Held geometry the pair does not touch keeps whatever bound it already
  published and never causes a refusal. A chorded analytic operand is not
  gated: its `sourceBound` may exceed `tol` by tessellation §1's Tolerance row. The gate is reject-only: it compares two proven numbers
  and admits nothing. Every bound has exactly one owner
  (`internal/proofbound/bounds.go`); no measurement site computes one inline.
- Rejected alternatives: a third-party kernel (dependency rule; also the
  supply-chain surface); float-only BSP classification (the flipped-sign
  nonsense solid of core §2.1); snapping/welding heuristics that move geometry
  by distance and charge nothing for it (decad never repairs). The embedding
  search above is not one: it chooses only among the floats bracketing each
  exact vertex, and the displacement it causes is measured and charged like any
  other rounding.

## 10. Verify — how each answer is computed

- **Feature-built bodies** are valid **by construction, and the proof is the
  construction**: a prism/revolve over a sketch-proven-closed, decad-recorded
  region is watertight and manifold structurally (every edge bounds exactly
  two faces by the build tables of §5/§6), and cannot self-intersect when the
  revolve side condition holds. `Verify` still runs the structural audit (an
  invariant check, cheap) but its verdict is decided, not sampled. All
  quantities Exact → the tolerance gate passes them at any `rel`.
- **Faceted bodies** are judged per verification §6 and payload verification
  §5/§6: exact held-mesh facts plus an internal source certificate, with a BVH
  proving every remote triangle/triangle and nonadjacent trim-edge distance.
  A clean complete certificate whose remote feature scale exceeds twice its
  boundary displacement proves validity; a shallow defect remains `Suspect`.
  The tolerance gate is then applied to every bounded result the report
  carries; `Approximate` alone never assigns `Suspect`. The body's gate
  diameter is cached from the greatest distance between any two vertices in
  the complete held faceted payload, not from the smaller set exposed as B-rep
  boundary-loop vertices, and is published through verification §3's shared
  reader, which rounds that distance toward zero. A placement rebuild recomputes the diameter after
  transforming the payload vertices and polls its context through the
  quadratic scan. The area floor sums each unique topological edge's held
  length once, never its two coedge uses.
- **Pairs**: use the four-way relation and proof order of
  `agent-docs/interference-design.md`. Bounds separation or the analytic clearance
  kernel proves disjoint/touching; nesting and admitted crossings remain
  explicitly `boundaryaudit.PairOverlapping`, never folded into undecided. A strict
  full-containment or analytic equality certificate reuses an operand's
  volume. Every other candidate overlap runs the read-only `OpIntersect`
  evaluator and emits a row
  only when the volume interval proves positivity (`Value - Bound > 0`). Empty,
  contact, unsupported, or coarse results remain undecided and read `Suspect`;
  internal failures return from `Verify`. Box-disjointness proofs run from
  increment 1; increment 3 adds analytic clearance; increment 4 adds read-only
  transversal intersection volume, with later staged breadth listed by the
  interference design.
- **Wall thickness / undercuts / min radius**: the analytic surveys of
  verification §6, answered outright on this evaluator's own payloads
  (`internal/verifyrun/survey.go`, `internal/survey2d/`, `internal/revolvesurvey/`): prism/revolve wall reduces exactly to the 2D
  spanning-disk problem (a prism's profile with the height as the vertical
  fit; a revolve's meridian section, mirrored for a full turn). `Verify`
  passes one shared work counter through the prism/revolve survey; candidate
  generation streams each disk directly into validation, and generation,
  validation, containment, and boundary scans all poll that counter. A
  canceled context therefore aborts the survey and returns its error from
  `Verify`. Undercuts are per-face exact normal-range membership, and the min
  radius is the tightest concave principal radius. A cup uses its exact morphology theorem:
  exact zero for an allowance-qualified pinch, otherwise its shell thickness
  carrying that thickness's own millimetre-conversion displacement as the
  reading's bound (payload verification §4.1).
  A faceted body uses certified source-normal/curvature patches and bounded
  medial candidates. A payload or certificate the surveys cannot decide leaves
  the asked question undecided → `Suspect`, never a silent pass (payload
  verification §4/§8–§10).

`Verify` MUST run requested surveys before the total numeric gate, then apply
verification §3's field table to every present body reading. The gate compares
base-unit magnitudes and uses the inclusive `Bound <= rel × Ref` rule. One
shared scalar-gate path handles body scalar readings, `Clearance.Gap`, and
`Interference.Volume`; a gap-only shortcut MUST NOT become a second contract.
The wall interval verdict and undercut predicate verdict remain separate inputs
to status aggregation, as verification §6 specifies.

## 11. Increments

Each increment is a PR series behind the same public contract; nothing ships
half-silent. The staging surfaces differently by what is being staged, per
the sections above: an intent the evaluator cannot BUILD — a feature call, an
extent, a segment kind — is `ErrUnsupported` at the call (§2), while a
`Verify` question the evaluator cannot yet ANSWER is accepted and reads
`Suspect` (§10) — an asked-but-undecided answer, never an error and never a
silent pass.

| # | Lands |
|---|---|
| 1 | topology model, `Document` and private provenance wiring, `Extrude` for line/circle/arc profiles with `Distance`/`Symmetric`/`TwoSided`-of-distance-sides extents, mass properties (§4), `Transform`, structural `Verify` (validity by construction, quantities, tolerance gate; every pair undecided → `Suspect` unless box-proven disjoint) |
| 2 | `Revolve` (angular extents), selector vocabulary + resolution, the body-relative stops (`ToFace`/`ToFaceAngular`/`EdgeAxis`, `ThroughAll`/`ThroughAllSide`) |
| 3 | analytic clearance proofs and `WithClearances` (box-disjointness proofs already run from increment 1, §10/§11 row 1) |
| 4 | tessellation per `agent-docs/tessellation-design.md` + the exact-predicate mesh boolean, `Faceted` bodies, faceted `Verify`, `Tessellate`/`STL`/`OBJ`; supplies the geometry and bounds shared by public booleans and read-only interference evaluation |
| 5 | fillet/chamfer on analytic prism edges, shell |
| 6 | broader evaluator coverage, property tests, and fuzz tests for structural record validation |
| 7 | tapered extrude and `Draft`, per `agent-docs/draft-design.md` §14's count-free five-PR plan |

Free-form support is `agent-docs/spline-design.md`'s own increment plan (§10 there).
Its stages do not consume a global evaluator increment number.

Loft follows `agent-docs/loft-design.md` §12's count-free five-PR delivery plan.
PR 1 adds `Loft`, its four measurements, and structural/tolerance
`Verify`; PR 2 adds tessellation, mesh-boolean admission, and placement; PR 3
lands same-kind `CircleSeg`/`ArcSeg` correspondence; PR 4 lands same-kind
Tier A free-form correspondence; PR 5 reserves N-section and
guide-rail/centerline lofts, and stages the analytic clearance adapter and
non-constant-section wall survey.
Every unlanded Loft `Verify` question remains `Suspect`; a call this evaluator
cannot yet build returns `ErrUnsupported`.

Sweep follows `agent-docs/sweep-design.md` §12's count-free delivery plan. The current
build reduces line spans to Extrude and circular spans to Revolve, then joins
composite tangent paths when exact frame transport and the conservative contact
audit close. One-span sweeps and admitted composite paths with line or circular
profile walks tessellate and enter mesh booleans. Free-form profiles, twist,
clearance, and surveys remain staged as Table D states. An unlanded Sweep build returns
`ErrUnsupported`; an unlanded verification question reads `Suspect`.

Coil follows `agent-docs/helix-design.md` §11's count-free four-PR plan: PRs 1
and 2 build `Coil` over whole-line profiles with its four readings,
structural and tolerance `Verify`, placement, tessellation, boolean and
interference operands and mass properties; arc profiles and clearance land
with PRs 3 and 4.
An unlanded Coil build returns `ErrUnsupported`; an unlanded verification
question reads `Suspect`.

Sheet bodies and the surface operations follow `agent-docs/surface-design.md` §14's
count-free three-increment plan: the body kind with `WithSurfaceResult()` on
Extrude and Revolve and `PatchProfile`; then `Stitch`/`Unstitch` over exact
all-planar boundaries with `PatchEdges` and the sheet-against-solid pair
proofs; then the remaining surface-result features, the shared-denotation join
certificate and curved closure. An unlanded surface build returns
`ErrUnsupported`; an unlanded pair question reads `Suspect`. These stages do
not consume a global evaluator increment number.

Payload verification §13 gives count-free stages for the cup adapter and
faceted validity/clearance/survey work. Every later question stays `Suspect`
until its stage lands. These stages do not consume global evaluator increment
numbers.

## 12. Open questions

- **Free-form reach is decided.** `agent-docs/spline-design.md` owns it: the
  whole-entities-only scope, the exactness tiers, the refusals, and the upstream
  ask that retires `EllipticalArcSeg` (§9 there). `FitSplineSeg` is Tier A
  (Table F), and its own build-path refusal (R6) is retired (§10 P4b), not
  through any upstream ask.
- **Tapered extrude is decided.** `agent-docs/draft-design.md` owns it: the sharp
  offset family, the admitted profile class, the audit that refuses a
  self-intersecting or collapsed far section, and `Draft`.
- **Modify reach is decided.** `agent-docs/modify-reach-design.md` extends increment
  5's modify ops with staged sub-items (RX/SX/BX/DX), including exact admitted
  cases and permanent `ErrUnsupported` boundaries; these stages do not consume a
  global evaluator increment number.
