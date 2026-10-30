# Point containment and distance design

`Body.LocatePoint` classifies one world-coordinate point against a solid body.
A straight prism with an undisplaced section and exact denoted levels is
classified against its own record in exact arithmetic (§7), and so are an
undisplaced stacked prism, the BRep a stacked `Union` keeps its slabs on (§9),
a line-only A7 crossing blind `Cut` (§9.1), and a full solid `Revolve` (§10). A
mesh Boolean result first combines its operands' exact answers through the
operation history (§8). Every other body, and every point §7 to §10 leave
open, reads a verified mesh and its occupied-volume proof (§2). The query does
not change the body or the document.

## 1. Public result

```go
func (b *Body) LocatePoint(ctx context.Context, p r3.Vec, tol units.Value) (PointLocation, error)

type PointLocation int

const (
    PointUndecided PointLocation = iota
    PointOutside
    PointInside
    PointOnBoundary
)
```

`p` is a world coordinate in millimetres. `tol` is the positive chord tolerance
passed to `Body.Tessellate` at `VerifyAll`. A solid whose payload cannot produce
a mesh returns that tessellation's `ErrUnsupported` when no exact route
decides the point. A sheet or an unsound solid returns `ErrNotSolid`. A nil body or context
returns `ErrDegenerate`; non-finite point coordinates return `ErrNotFinite`. The
tolerance uses `Tessellate`'s kind, finiteness and zero gates, which the exact
routes apply themselves before they read a record, so an invalid tolerance fails the
same way on every body. Retired bodies remain readable.

`PointUndecided` means the proof cannot classify this point at the requested
tolerance. It is not an outside reading. The caller may request a finer mesh;
the query can still return `PointUndecided` if a payload publishes no occupied
volume proof or the point is too close to its boundary. `PointOnBoundary` is
reported only when the record places the point on its boundary exactly (§7,
§9, §10), the operands' records do so under §8's set algebra, or the mesh
boundary is exact at that point (§2).

## 2. Proof

`VerifyAll` must publish both `BoundaryVerified` and `VolumeVerified`. The
first proves the held mesh is embedded. The second gives `S`, an upper bound
on the volume of the symmetric difference between its occupied region `M` and
the body's true region `T`. `Mesh.Bound` gives `B`, a two-sided Hausdorff upper
bound between their boundaries. Both bounds are read as exact dyadic numbers
from their held `float64` values; neither is rounded down.

The exact mesh parity kernel classifies the query against `M`. A point on a
held facet is `PointOnBoundary` only if `B = 0`; otherwise it is undecided. A
parity cast that cannot avoid every facet edge also returns undecided.
Repeated queries reuse the held mesh's exact vertex projections and parity
index. The cache is built lazily and guarded while queries fill it, so
concurrent calls keep the same answer without changing the mesh.

For every held facet, compute the exact squared distance from `p` to its
closed triangle over rational coordinates. Let `D²` be the minimum. Round
`sqrt(D²)` downward to a rational `L`; this can only make the query harder to
decide. If `R = L - B` is positive, neither `M`'s nor `T`'s boundary meets the
open ball of radius `R` about `p`. If their membership at `p` differed, their
symmetric difference would contain that ball. Since π > 3, the ball's volume
is strictly greater than `4R³`. Therefore `4R³ > S` rules out different
membership and transfers the exact mesh parity answer to the body. Equality
is undecided. The calculation compares exact rationals and uses no sampled
distance or tolerance-based predicate.

The distance kernel tests the orthogonal projection onto each triangle's
plane. If that projection lies in the closed triangle, the squared distance is
the squared plane offset divided by the squared normal length. Otherwise the
kernel minimizes the exact quadratic over each closed edge. A degenerate
triangle is an evaluator failure; tessellation's Geometry row already excludes
it. A triangle's axis-aligned box gives an exact lower bound used only to skip
an exact distance when the current minimum is already no larger.

Parity and distance consider every triangle across every lump and shell. An
internal cavity therefore changes parity twice along a ray and classifies as
outside. No nearest-face selection or local normal decides membership.

## 3. Verification

- A box answers inside, outside and exact boundary, including a retired body.
- A plate with a through bore answers outside inside the bore and inside beside
  it, and on the boundary at a point of its exact circular wall (§7).
- A faceted boolean result answers inside or outside at points whose ball
  proof clears its symmetric-difference bound.
- A rotated body answers in world coordinates. A sheet returns `ErrNotSolid`.
- A canceled context, invalid point and invalid tolerance return their stated
  errors without changing the document.

## 4. Distance from a point to a solid

```go
func (b *Body) DistanceToPoint(ctx context.Context, p r3.Vec, tol units.Value) (Measurement, error)
```

The distance is zero for a point in the material or on its boundary. For a
point outside, it is the minimum distance to the body's boundary. The query
accepts the same body, point, tolerance and context inputs as `LocatePoint`,
including a retired solid. It returns the same input errors and the
`Tessellate(VerifyAll)` error when meshing fails. A returned mesh without a
verified boundary is `ErrUnsupported`.

Let `D²` be `MeshDistanceSquared`'s exact minimum over every held triangle.
`RatSqrtDown(D²)` and `RatSqrtUp(D²)` enclose the held boundary distance. Let
`B` be the verified mesh's two-sided boundary bound. The Hausdorff distance
inequality puts the true boundary distance in
`[max(0, sqrt(D²) − B), sqrt(D²) + B]`. The query computes these interval
ends over exact rationals from the directed square-root floats and `B`.

`MeshDistanceSquared` computes each triangle distance in exact dyadic
arithmetic and divides once. A downward-rounded float box bound only skips
a triangle whose bound is at least the upward-rounded exact minimum; it never
admits one or decides the value. Triangles are visited smallest bound first
once a float test with a proven error bound, or the exact normal, shows no
triangle is degenerate. A mesh with a degenerate triangle keeps the
index-order scan. Either way the query returns the value, budget steps and
errors of the index-order scan.

The distance query reads §2's mesh proof only; it does not consult §7 or §8.
When §2 proves the point outside, that interval encloses the distance to the
solid. When §2 proves inside or on the boundary, return exact zero. When the
occupied-volume proof is missing or membership is undecided, use
`[0, sqrt(D²) + B]`: zero covers an interior point, and the upper end covers
an exterior one. Round the interval's midpoint once, then round its farther
endpoint distance outward for `Measurement.Bound`. A non-finite value or bound
is `ErrUnsupported`; no unbounded `Measurement` is returned.

Verify a box at an interior point, a face point, an exterior face point and an
exterior corner point. Verify a through bore at its center: the interval must
contain the analytic bore radius. A point near an approximate wall may have
a zero lower bound while its upper bound remains finite.

## 5. Distance from a point to one trimmed face

```go
func (f *Face) DistanceToPoint(ctx context.Context, p r3.Vec, tol units.Value) (Measurement, error)
```

The distance is to the selected face's trimmed patch, including its boundary.
It may be positive for a point inside the owning solid. Solid and sheet faces,
including faces of retired bodies, use the same query. A nil face, nil context,
non-finite point, or invalid tolerance returns the matching input error.
An owning body whose payload cannot tessellate returns its tessellation error.

Tessellate the owning body at `VerifyBoundary`. Select only triangles whose
`SourceFaces` entry is the selected face. A face with no source triangles
returns `ErrUnsupported`; a missing source bound returns `ErrBooleanFailed`;
a non-finite bound returns `ErrUnsupported`. Let `D²` be the exact minimum squared
distance to those triangles and `B` the face's two-sided source bound. The
true distance lies in `[max(0, sqrt(D²) − B), sqrt(D²) + B]`. Use directed
square-root endpoints and `pointquery.RatIntervalMeasurement` to return a finite
`Measurement`. This query needs no occupied-volume proof or point-membership
classification.

Verify a box cap for points on the patch, inside the body, and beyond one
corner. Verify a curved wall using its analytic radius and a surface-extruded
sheet face.

## 6. Closest point

```go
type ClosestPoint struct {
    Point      r3.Vec
    Distance   Measurement
    OffSurface units.Value
}

func (b *Body) ClosestPoint(ctx context.Context, p r3.Vec, tol units.Value) (ClosestPoint, error)
func (f *Face) ClosestPoint(ctx context.Context, p r3.Vec, tol units.Value) (ClosestPoint, error)
```

Both methods take `DistanceToPoint`'s inputs and return its input and tessellation errors. `Distance` is the reading
`DistanceToPoint` gives for the same call, and `Point` is the mesh point that attains it.

| Decision | Rule |
|---|---|
| Q1 | `pointquery.MeshClosest` finds the exact minimum squared distance `D²` over the held triangles, as `MeshDistanceSquared` does, together with the triangle point that attains it. A point whose projection falls inside the triangle takes the foot of the perpendicular. Otherwise it takes the nearest point of the three edges. All of it is exact rational arithmetic. |
| Q2 | `Point` is that rational point rounded to the nearest floats. `OffSurface` is the verified mesh bound `B` (the face's source bound for a face) plus the distance from the rational point to `Point`, rounded up. |
| Q3 | `Body.ClosestPoint` returns `p` itself with `OffSurface` zero when §2 proves `p` inside or on the solid. Otherwise it returns the mesh point of Q2, also when membership is undecided. |
| Q4 | `Point` lies within `OffSurface` of the true boundary, because the mesh departs from it by at most `B`. Its distance from `p` is `sqrt(D²)` and so lies within `OffSurface` of the true distance. |

`Point` is one nearest mesh point. Where two places on the boundary are almost equally near, `Point` can sit at
either, and the other can be much farther along the boundary than `OffSurface`; only Q4's distance statement is
guaranteed. A face's `Point` lies on the face's own patch.

Verify a box for interior, boundary, face, edge and corner queries against the analytic nearest point. Verify a box
cap for a point beside it and above it.

## 7. Exact prism route

`pointquery.LocatePrism` classifies a point against a straight prism's own
record before `LocatePoint` builds a mesh. It returns a decided answer about
the set the record denotes, the same set §2 proves its answers for, or hands
the point to §2 unchanged.

### 7.1 Admission

The body's payload must be a `prismPayload` with zero `SectionDelta`, no
`SurfaceResult`, and finite, nonnegative endpoint bounds. `DenotedZ0` and
`DenotedZ1` must return exact levels in strictly increasing order. A zero bound
uses the held level; a displaced level needs a matching producer certificate.
Missing or stale certificates and malformed bounds go to §2. A faceted Boolean
result goes to §8, a stacked prism and a BRep to §9, and a revolve to §10.

### 7.2 Plane coordinates

The record denotes its plane-coordinate body under the affine map
`x ↦ T + B·O + L·x`, `L = B·[U V N]`: the frame's held origin and axes, the
placement's held basis `B` and translation `T`, all read as exact rationals
(`agent-docs/evaluator-design.md` §5.1, `massmoment.PrismRotation`). `L` is
orthonormal only to rounding, so its transpose is not its inverse. The route
solves `L·x = p − T − B·O` by Cramer's rule over exact rationals, as the
oblique sphere section solves its Gram system
(`agent-docs/section-design.md`, "Oblique plane of an authenticated source
sphere"). This is an exception to CLAUDE.md's "inverse = transpose, never a
solve" rule: the transpose inverts the held map only to rounding, so it could
never place a point exactly on the boundary the record denotes. A zero
determinant hands the point to §2. The solve gives the exact plane
coordinates `(u, v, z)` of `p`.

`z` is compared to the exact denoted levels, not their rounded held values.
A point beyond either level is `PointOutside` without reading the profile.

### 7.3 Plane-local region test

This is a decad-side 2D answer under CLAUDE.md's "Ask `sketch` for 2D answers
by default" rule, which admits one where it clearly wins on performance or
correctness and asks the owning design to state the reason. The precedent is
the coplanar contact patch (`agent-docs/multibody-dynamics-design.md` §9.4),
which clips two exact polygons inside decad for the same two reasons.
Correctness: the question is whether an exact rational point lies in the
region the record denotes, and every boundary piece this route reads is exact
rational data, so a fixed set of sign tests over exact rationals decides it
with no rounding, sampling or tolerance. `sketch` publishes no
point-in-profile query (`sketch@b8117fc` exposes `Profile.Containment(*Loop)`
only), and a private scene rebuilt from the record would rebuild its boundary
from floats and could not say on which side of an exact boundary an exact
point lies. Performance: one query reads each recorded segment once, where a
scene would be built and arranged per point.

The route decides only from exact recorded data. Every point it reads is a
recorded float read as an exact rational, or a line's lerp at its recorded
parameter (`momentinput.ExactLineEnds`). The recorded parameter of a `Partial`
fragment is its definition (`agent-docs/sketch-seam-design.md` §1). A record
holds no uncertified `Partial` fragment: the seam refuses `TExact` false at
recording time (`agent-docs/sketch-seam-design.md` §2.1), so a point is never
decided from an uncertified cut.

| Segment | Read as | Otherwise |
|---|---|---|
| `LineSeg`, whole or fragment | the segment between its exact lerps at `TStart` and `TEnd` | a non-finite field hands the point to §2 |
| whole `CircleSeg`, the loop's only segment | the closed disk of its exact centre and radius | a fragment, a radius not recorded in millimetres or a non-positive radius hands the point to §2 |
| whole `ArcSeg` with `\|End − Center\|² = \|Start − Center\|²` exactly | the circle points on the closed right of the chord `Start → End`, swept counter-clockwise | a fragment, an `End` off `Start`'s radius, or `Start = End` hands the point to §2 |
| any other kind | — | hands the point to §2 |

A whole arc denotes `Start`'s radius at `End`'s angle
(`agent-docs/evaluator-design.md` §4), so its `t = 1` end is the recorded
`End` exactly when the two squared radii are equal; otherwise that end carries
a square root and the route does not read the loop. A circle or arc fragment's
cut end carries trigonometry, so points near a cut curved edge stay with §2.

Each loop is walked into a ring of exact vertices in walk order: each line by
its two ends, each arc by its chord between its walk ends. Each junction
closes as the evaluator closes it (`agent-docs/evaluator-design.md` §4): two
equal ends close at that point, two line fragments close through the exact
crossing of their supports where `momentinput.JunctionCorner` admits it, and
every other junction closes with the straight chord between the two ends.

For one loop and a plane point `q`:

1. `q` on an arc (`|q − C|² = r²` and on the closed right of its chord), or on
   any ring edge other than an arc's chord, is on the loop's boundary.
2. `q` on an arc's chord and not on the boundary hands the point to §2.
3. Otherwise the loop's winding number about `q` is the ring's, from exact
   crossing and side tests, corrected for each arc whose open circular
   segment (`|q − C|² < r²` and strictly right of the chord) holds `q`: `+1`
   when the loop walks the arc from `Start` to `End`, `−1` when it walks it
   back. The loop and the ring differ by exactly those segments' boundary
   curves. Zero is outside, `±1` is inside, and any other value hands the
   point to §2.

The region is the outer loop's less every hole's. A point on more than one
loop, inside two holes, or inside or on a hole but outside the outer loop
hands the point to §2: a sketch-arranged record has none of these.

| Plane class | `Z0 < z < Z1` | `z = Z0` or `z = Z1` |
|---|---|---|
| inside | `PointInside` | `PointOnBoundary` |
| on the boundary | `PointOnBoundary` | `PointOnBoundary` |
| outside | `PointOutside` | `PointOutside` |

The route polls the context through a `proofbound.WorkBudget` once per
segment and ring edge. Its tolerance is unused, but §1's tolerance gates run
before it.

### 7.4 Verification

- A radius-5 cylinder at a 0.05 mm tolerance answers inside at r = 4 and
  r = 4.9, outside at r = 5.5, on the boundary on its wall and caps, and
  inside or outside 1e-9 mm either side of the wall and the top cap; §2 alone
  leaves all of these undecided.
- A plate copied with a 45° rotation answers on its `x = 0` face and 1e-9 mm
  either side of it through the exact placement solve.
- A square drawn as four overshooting lines, every segment a `Partial`
  fragment, answers on its corners and sides and 1e-9 mm either side. A unit
  fixture whose fragment ends fall a few ulps short of each corner answers on
  the corner through the junction corner; with the chord instead, it reads
  outside.
- A quarter pie whose arc `End` lies on `Start`'s radius answers on the arc,
  1e-9 mm either side of it, and outside on the circle beyond the arc; a
  point on its chord falls to §2. The same pie with `End` 500 ulps off the
  radius falls to §2 for every point, which leaves points near its boundary
  undecided.
- A square whose top side carries a half-circle notch walked back from `End`
  to `Start` answers outside inside the notch, where the ring alone reads
  inside and only the arc's `−1` correction decides.
- Unit fixtures hand the point to §2 for each admission gate, a singular
  placement, a circle or arc fragment, a circle in inches, a zero-radius
  circle, an arc whose `Start` equals its `End`, an ellipse, a one-line loop,
  two holes holding one point, a hole outside the outer loop, and a point on
  the outer loop inside a hole. A reversed record answers as its forward twin.

## 8. Boolean results through history

`pointquery.LocateBoolean` classifies a point against a mesh Boolean result
by combining its operands' answers through the operation history
(`agent-docs/operation-history-design.md`, "Point location through history").
It answers about the set §2 proves its answers for, or hands the point to §2
unchanged. It re-runs no geometry and builds no body.

### 8.1 What the history proves

A body built by a public `Cut`, `Union` or `Intersect` denotes that
regularized operation over its recorded consumed inputs, unless the build
moved accepted geometry. Its published bounds relate its held mesh to that set
(`agent-docs/faceted-vertex-bounds-design.md` §2), so that set is §2's true
region `T`. `agent-docs/boolean-chain-regroup-design.md`, "Eligibility of an
expandable body", states when the record is that set-equality proof, and
`boolchain.Expandable` decides it. The route reads the same function and
nothing else:

- The payload is a `facetedrecord.Payload`. An analytic result is classified
  by its own record (§7, §9, §10) or by §2.
- The document's producer record for the body is a `Cut`, `Union` or
  `Intersect` with the body as its output, read through `historyProducers`
  (`boolean_regroup.go`). A `Cut` must carry exactly a `target` and a `tool`.
- `PointCone` is nil. `decad.WithMatchingFaces` reaches only the point-cone
  route, and its result denotes the declared match with its effect charged
  into the bounds, not the operation over its inputs.
- Every `OperationWarnings` record is classified as moving no geometry: a
  `CodeChainAllowanceApplied` with no `Change` mechanism, which relaxes a
  gate and moves nothing. Any other record, including a code added later,
  makes the body a leaf. A gap-shifted retry or an embedding reach moved
  accepted geometry, so the result is not the operation over its inputs,
  however small the move.

Every other body is a leaf.

### 8.2 Leaves and placements

A leaf whose payload is a `prismrecord.Payload` is classified by §7, a
stacked prism or a BRep by §9, and a revolve by §10. A leaf those routes do
not decide, and every other leaf, reads undecided. The route reads no leaf's
mesh: that would tessellate each retired operand at `VerifyAll`.

The route does not walk `Transform` or `Copy`. A placed mesh result denotes
its source's set under its own placement record (an exact rotation or the
float delta, `facetedrecord.PlacedWith`), and §7 takes only a float query
point, so classifying the inverse-placed point would need a rational query
route §7 does not have. Such a body, as a query or as an operand, is a leaf
and reads undecided. A placed prism leaf, such as a rotated `Copy` operand,
is solved through its own held placement (§7.2).

### 8.3 Set algebra

Every operand is a regular closed set. Each operand gives `p` one of four
answers: inside (in its interior), on its boundary, outside (in the interior
of its complement) or undecided (any of the three). Write `A ∧ B` for the
regularized intersection, `¬A` for the closure of the complement, and
`A ∨ B = ¬(¬A ∧ ¬B)` for the union. Then `Intersect(A, B) = A ∧ B`,
`Cut(A, B) = A ∧ ¬B` and `Union(A, B) = A ∨ B`. Complement swaps inside and
outside and keeps the boundary, because a regular closed set and the closure
of its complement share one boundary.

| `A ∧ B` | B inside | B on boundary | B outside | B undecided |
|---|---|---|---|---|
| A inside | inside | on boundary | outside | undecided |
| A on boundary | on boundary | undecided | outside | undecided |
| A outside | outside | outside | outside | outside |
| A undecided | undecided | undecided | outside | undecided |

Union is the complement of the meet of the complements, so its table
follows from the one above with inside and outside swapped:

| `A ∨ B` | B inside | B on boundary | B outside | B undecided |
|---|---|---|---|---|
| A inside | inside | inside | inside | inside |
| A on boundary | inside | undecided | on boundary | undecided |
| A outside | inside | on boundary | outside | undecided |
| A undecided | inside | undecided | undecided | undecided |

| `¬A` | A inside | A on boundary | A outside | A undecided |
|---|---|---|---|---|
| answer | outside | on boundary | inside | undecided |

Proof of each row of the meet table:

- Outside either operand: a ball about `p` misses that operand, so it misses
  `A ∧ B`, which lies in both.
- Inside both: a ball about `p` lies in both interiors, so in the interior of
  `A ∧ B`.
- Inside `A`, on `B`'s boundary: take an open ball `N` about `p` inside
  `A`'s interior. Then `int A ∩ int B ∩ N = int B ∩ N`, so
  `(A ∧ B) ∩ N = cl(int B ∩ N) ∩ N = cl(int B) ∩ N = B ∩ N`, since `N` is
  open and `B` is regular. Near `p` the result is `B`, so `p` is on its
  boundary.
- On both boundaries: two boxes that share one face meet in no volume, so a
  point of that face is outside their meet, while a box met with itself keeps
  the point on its boundary. The answer is undecided.
- Undecided with any answer other than outside: the row's answers differ over
  the three classes the undecided operand may have.

`Intersect` and `Union` records with `k` inputs fold the table left to right;
each step is sound for every class consistent with its inputs, and `∧` and
`∨` are associative. The walk stops a `Union` at the first inside and an
`Intersect` or `Cut` at the first outside, and reads a `Cut`'s tool only when
its target is not outside. These answers hold whatever the remaining operands
are.

The route combines answers by these theorems over exact leaf answers. It
admits no geometry: eligibility is the record and payload facts of §8.1, and
no residual, sampled distance or tolerance takes part. It computes no 2D
answer of its own; the plane-local tests are §7's.

### 8.4 Walk

The walk is depth first over each producer's recorded consumed inputs, in
recorded order (a `Cut`'s target, then its tool). A body is consumed at most
once, so the walk is a tree over the document's finite history. The route
polls the context through a `proofbound.WorkBudget` once per body visited.
An inside, outside or on-boundary answer at the root is returned; undecided
hands the point to §2, whose decided answers agree with §8's because both
are about the same set.

### 8.5 Verification

- A 30 × 30 × 4 plate united with a 20 mm wide plate copied with a 45° turn
  about the x axis, a mesh result with no warning, answers inside 0.001 mm
  and 1e-9 mm under the flat top face, on that face, and outside 0.001 mm and
  1e-9 mm over it. An identity `Copy` of it reads §2 alone, which leaves the
  face and the 1e-9 mm points undecided. A point on the top face inside the
  turned plate answers inside; one on the turned plate's end face stays
  undecided.
- A 20 × 20 × 8 plate cut by a radius-2 pin through it answers on the pin's
  wall and the plate's top face, and inside or outside 1e-9 mm either side of
  the wall. A point on the bore's rim is on both operands' boundaries and
  stays undecided. The same pin intersected with the plate answers on the
  wall, on the top face inside the pin, and inside or outside 1e-9 mm either
  side of them.
- A `Cut` of the pin through the crossed plates answers through two levels of
  history: on the pin's wall inside both plates, inside or outside 1e-9 mm
  either side of it, and inside 0.001 mm under the top face.
- A quarter pie whose arc end lies 500 ulps off its radius (§7.4), united
  with a box, answers inside 1e-9 mm inside a box face because the box alone
  decides it. On the box face beyond the pie, and 1e-9 mm inside the pie's
  arc beyond the box, the pie's answer is undecided and so is the result.
- The 30 × 30 × 4 plate united with its full-width turned copy shares the
  plates' end faces, takes a gap-shifted retry, and stays undecided on its top
  face and 0.001 mm under it. A `Transform` of the warning-free crossed plates
  stays undecided on the image of the top face and 1e-9 mm under it.
- Unit fixtures check the meet, union and complement tables of §8.3 for
  every answer, each operation over two overlapping cubes, the
  fold over three inputs, a `Cut` over a `Union`, and each stop, which reads
  no later operand. They decline a point-cone record, a gap shift, an
  embedding reach, an analytic payload, a missing producer record, a
  one-input `Cut`, a mesh operand with no record and a section-displaced prism
  operand; a chain allowance still combines, and a canceled context returns
  its error. They also cut a stacked leaf by a cube and unite a revolve leaf
  with a cube (§9, §10).

## 9. Stacked prisms and stack BReps

`pointquery.LocateStacked` and `pointquery.LocateStackBrep` classify a point
against an undisplaced stack of slabs before `LocatePoint` builds a mesh. Like
§7, they return a decided answer about the set the record denotes or hand the
point to §2 unchanged.

### 9.1 Admission

Two payloads carry a slab stack:

- A `stackedPrismPayload` (`agent-docs/stacked-prism-design.md` §2.1) denotes
  the union of every slab's regions swept over that slab's interval, carried
  through §7.2's map with its own frame and placement (evaluator §5.1). Its
  interfaces are derived from the slabs, so the route does not read them.
  `SectionDelta` and every slab's `Z0Delta` and `Z1Delta` must be zero, so
  each level is its held float.
- A `brepPayload` that keeps a `Stack` (`agent-docs/general-boolean-design.md`
  §3 "A1 as a brep", §4.1) is built from exactly the stack's loops at the
  stack's levels, each face within the stack's section displacement of the
  set it denotes. The route requires that displacement and every stack level
  displacement to be zero, at least one face, every face's displacement and
  both level displacements zero, every side-line split's level displacement
  zero, no blend face, no loop band, and no boss or pocket shell band. Unlike
  the BRep's scale conversion, it does not require line-only faces: a curved
  loop reads through §7.3's own gates.
  Then the body is the union of the stack's slabs. The BRep joins each
  junction at one canonical vertex (general-boolean §4.1), so the route also
  requires every stack loop's junctions to close at two equal ends, where
  that vertex is the recorded end. The stack is stated in the reference
  frame: the first face's frame, which the stacked BRep build gives its first
  cap. The route requires that first face to be a planar cap (a region and no
  sweep) and reads §7.2's map with its frame and the payload's placement.

A crossing blind cut (general-boolean §3 A7), an opposed-pocket result (A8)
and a class-B result keep no A1 operand stack, a cup holds its record in its
own payload, and a displaced stack is not its held record; all of them go to
§2 unless the A7 result carries its query-only source prisms. That field is
attached only while the initial BRep body is built, when both inputs and every
result slab contain lines only and both input sections have zero displacement.
It cannot be read as a later Boolean operand. The query route classifies both
source prisms through §7 and combines their exact answers as a regularized Cut.
Displaced levels or any other §7 refusal leave the point to §2. Placement or
another generic BRep rebuild drops the field.

The route checks, reject-only, that the stack has a slab, every slab has a
region, every slab has `Z0 < Z1` exactly, and consecutive slabs share one held
level (stacked §2.2's I3 and I4). A record that fails goes to §2.

### 9.2 Classification

The route solves the plane coordinates `(u, v, z)` of `p` as §7.2 does. Let
`L_0 < … < L_n` be the levels and `c_i` the class of `q = (u, v)` in slab
`i`'s material: §7.3's class against each region, combined by §8.3's union
table. That table holds for regular closed sets in the plane as in space, so
a slab whose regions touch reads undecided where two of them meet.

- `z < L_0` or `z > L_n` is outside.
- `L_i < z < L_{i+1}` reads `c_i`. A ball about `p` short of both levels meets
  slab `i` alone, where the material is the region times an interval.
- `z = L_k` combines `a = c_{k−1}` below and `b = c_k` above, with `a` outside
  at the bottom level and `b` outside at the top level:

| level `z = L_k` | b inside | b on boundary | b outside | b undecided |
|---|---|---|---|---|
| a inside | inside | on boundary | on boundary | undecided |
| a on boundary | on boundary | on boundary | on boundary | on boundary |
| a outside | on boundary | on boundary | outside | undecided |
| a undecided | undecided | on boundary | undecided | undecided |

Proof. Near `p` only slabs `k − 1` and `k` meet a ball about `p`: every other
slab's interval lies a positive distance from `z`. So near `p` the material
is `R_{k−1} × (−∞, z]` together with `R_k × [z, ∞)`, an empty region standing
for a missing slab.

- Inside both: a disk about `q` lies in both regions, and the ball over it
  lies in the material.
- Outside both: a disk about `q` misses both regions, and so the ball over it
  misses the material.
- Inside one and outside the other: `p` is material, and the points straight
  through the level on the outside side are not.
- On `a`'s boundary: `q` lies in `R_{k−1}`, so `p` is material, and points
  `(q′, z − ε)` with `q′` outside `R_{k−1}` lie arbitrarily close to `p`
  outside the material, whatever `b` is. The same holds for `b` above.
- Undecided beside inside or outside: the row's answers differ over the
  classes the undecided side may have.

§7.3's height table is this table with one side outside.

The route polls the context through §7's budget per segment and ring edge
and once per slab region.

### 9.3 Verification

- A 10 mm cube with a radius-2.5 blind pocket answers on the pocket floor,
  inside and outside 1e-9 mm either side of it, inside at the interface level
  in the material, on the foot of the pocket wall, on the wall and 1e-9 mm
  either side of it, outside at the pocket mouth, and on the rim, the top,
  the outer wall and the bottom. The mesh alone leaves the floor and every
  1e-9 mm point undecided.
- A plate under a union-built round boss answers inside at the interface
  under the boss, on the boss foot and the plate top beside it, and on the
  boss wall and 1e-9 mm either side of it.
- Unit fixtures read a bored stack at every level and region class, and a
  prism group whose two squares share a side, which reads undecided on that
  side. The level table is checked for every pair.
- Unit fixtures hand the record to §2 for each stacked gate (a section or
  level displacement, a slab with no region, an empty interval, levels that
  do not meet, no slab, a singular placement) and each BRep gate (no stack,
  a stack or stack level displacement, no face, a loop band, a boss or
  pocket shell band, a face or face level displacement, a blend face, a
  displaced split, an open junction in an outer loop or a hole, a swept or
  restated first face).
- Through §8: a mesh `Cut` of the pocketed cube by a turned pin answers on
  the pocket floor and wall, 1e-9 mm either side of the floor, and inside at
  the interface level. A mesh `Cut` of the flush corner boss BRep by a turned
  pin answers on the shared wall, at the interface and 1e-9 mm either side
  of it, inside under the boss, and on the plate and boss tops and 1e-9 mm
  from them. The leaf reads no mesh, so without §9 these stay undecided. A
  stack BRep's own faces are planar here, so its own mesh is exact and §2
  alone decides it.

## 10. Full solid Revolve

`pointquery.LocateRevolve` classifies a point against a full solid
`Revolve` from its recorded meridian before `LocatePoint` builds a mesh. It
returns a decided answer about the set the record denotes or hands the point
to §2 unchanged.

### 10.1 Admission

The payload must be a `revolvePayload` with `Full` set, no `SurfaceResult`,
zero `SectionDelta`, and a sweep whose two ends denote exactly one turn apart
(`revolveangle.Sweep` with equal `Rad` parts and `Turn` parts exactly one
apart): a radian-stated sweep snapped to full denotes a slightly smaller
angle. The resolved axis must carry zero anchor and direction bounds, a zero
radial admission allowance and a zero snap allowance, the axis gates the
exact meridian section reads (`agent-docs/section-design.md`, "Plane through
or normal to the axis of a full Revolve"). A zero direction makes §10.2's
map singular.

Every meridian segment must be a whole line, a whole millimetre circle, or a
whole arc whose `End` lies exactly on `Start`'s radius (§7.3's table without
its line fragments), and every junction must close at two equal ends, so the
meridian is its recorded points. Reject-only, every line end, arc end and
centre, and circle centre must lie on the material side of the axis: its
exact radial coordinate `ρ = d × (P − A)` is not negative, and is either zero
or above the axis frame's `SnapTol`, since the build reads a point that close
as on the axis. Any other record goes to §2.

### 10.2 The denoted set

A meridian point `P` at angle `φ` denotes
`T + B·(A3 + W·z + (E0·cos φ + E1·sin φ)·ρ)`, with `z = (P − A)·d` and
`ρ = d × (P − A)` about the axis anchor `A` and direction `d`,
`A3 = O + U·a_u + V·a_v`, `W = U·d_u + V·d_v`, `E0 = V·d_u − U·d_v` and
`E1 = W × E0`, every leaf the held float read exactly. This is the
construction evaluator §6's swept-vertex comparison proves every vertex
against (`revolvemesh.RevolveLift.SweptPointGap`); it holds whether or not
`d` is a unit vector, and whether or not the frame is exactly orthonormal.
The body is therefore `G(S₀)` for the affine map
`G(z, x, y) = T + B·(A3 + W·z + E0·x + E1·y)` and the solid of revolution
`S₀ = {(z, x, y) : (z, √(x² + y²)) ∈ M̂ ∪ M̂′}`, where `M̂` is the meridian
region `M` mapped by `P − A ↦ (z, ρ)` and `M̂′` is its mirror `ρ ↦ −ρ`.

The route solves `G(z, x, y) = p` by Cramer's rule over exact rationals, as
§7.2 solves its map and for §7.2's reason: `W`, `E0` and `E1` are orthogonal
only to rounding. A zero determinant hands the point to §2. `G` is an
affine bijection, so `p`'s class in the body is `(z, x, y)`'s class in `S₀`.
Let `R = x² + y²`. The plane points with axis coordinates `(z, ±√R)` are
`Q± = A + (z·d ± √R·e)/|d|²`, `e = (−d_v, d_u)`, and the map
`P − A ↦ (z, ρ)` is linear and invertible, so classes in `M̂` are classes
of these points in `M`.

`S₀` is symmetric about its axis. A ball about `(z, x, y)` turns about the
axis onto a disk about `(z, √R)` in one meridian half-plane, and `S₀` meets
that plane in `M̂ ∪ M̂′`. So `(z, x, y)`'s class in `S₀` is `Q+`'s class in
`M ∪ M′`, `M′` the mirror of `M` across the axis, and this holds for a point
on the axis too.

The build orients the axis so the meridian lies on its non-negative side
(`revolveaxis.ResolveSide`), and a zero radial admission allowance states
the radial minimum was proven non-negative (evaluator §6). So `M` lies in the
closed half-plane `ρ ≥ 0`, and `M′` in `ρ ≤ 0`.

### 10.3 Classification

- `R > 0`: the answer is §8.3's union of `Q+`'s and `Q−`'s §7.3 classes in
  `M`, because `Q−`'s class in `M` is `Q+`'s class in `M′`. The union table
  holds for any regular closed sets; with `M` on `ρ ≥ 0`, `Q−` lies outside
  it and the answer is `Q+`'s class.
- `R = 0`: `Q = A + z·d/|d|²` is a rational point on the axis. Inside,
  outside and undecided carry over. On `M`'s boundary, the point is inside
  when every boundary piece through `Q` (a ring edge that is not a chord, an
  arc, a circle) is a line with both ends exactly on the axis line, and on
  the boundary otherwise.

Proof of the axis rule. If every piece through `Q` lies on the axis, near
`Q` the boundary of `M` lies on the axis. The open half-disks on either side
then hold no boundary point, and `M` is regular, so one of them lies in `M`'s
interior and `M ∪ M′` holds the whole disk: inside. If a piece through `Q`
leaves the axis, it holds boundary points of `M` with `ρ > 0` arbitrarily
close to `Q`: a line not along the axis leaves it on one side, and a circle
meets the axis line at most twice. Near such a point `M′` is absent, so it
is a boundary point of `M ∪ M′` too, and no disk about `Q` lies in
`M ∪ M′`: on the boundary.

`Q±` has an irrational coordinate. Each coordinate is `r₀ + r₁·s` with
rational `r₀`, `r₁` and `s = √R`. Every §7.3 test is a sign of a polynomial
of degree at most two in the query point's coordinates: a cross product, a
dot product of two offsets, a squared distance less a squared radius, and a
height comparison. With `s² = R` each is `a + b·s` for rationals `a` and
`b`. When `a` and `b·s` have one sign, that is the sign. Otherwise the term
of larger magnitude decides, found by comparing `a²` with `b²·R`, and equal
magnitudes give zero. No square root is formed and no comparison is rounded.

The route polls the context through §7's budget per segment and ring edge.

### 10.4 Verification

- The stepped shaft (r = 10 over z ∈ [0, 20], r = 6 over z ∈ [20, 40] about
  world z) answers on its wall at three angles, inside and outside 1e-9 mm
  either side of it, inside or outside at `(5√2, 5√2, 5)` by the sign of
  `x² + y² − 100` over its held coordinates, on the shoulder and 1e-9 mm
  either side of it, inside at the shoulder level within the neck, on both
  cap centres, and inside on the axis, at the shoulder level and 1e-9 mm
  inside the bottom cap. The mesh alone leaves the wall, shoulder, cap and
  1e-9 mm points undecided.
- A capsule (r = 5 over z ∈ [0, 10] under a hemisphere) answers on the pole,
  1e-9 mm either side of it and of the sphere, and on the equator. A tube
  (r ∈ [4, 10]) answers outside on its axis and on its bore wall. A
  translated shaft answers on its moved wall and cap centre, and a copy turned
  45° about x answers 0.001 mm inside and outside its wall.
- Through §8: the stepped shaft united with a box crossing its wall, a mesh
  result, answers on the shaft wall beside the box and 1e-9 mm either side of
  it, inside on the wall within the box, on the box's end face, on the
  bottom cap centre and inside on the axis.
- Unit fixtures revolve a rectangle and a quarter pie about the plane's v
  axis: walls, an irrational radius, axis points, cap centres, the pole and
  the pie's chord, which falls to §2. A frame turned about z is solved
  through its held axes. A circle touching the axis, which the build refuses,
  reads its axis point on the boundary.
- Unit fixtures hand the record to §2 for each admission gate: a partial
  sweep, a surface result, a section displacement, a radian-stated sweep, a
  turn with a radian part, a derived span, no denotation, each axis bound,
  the radial admission and snap allowances, a negative snap tolerance, a zero
  direction, a singular placement, a line fragment, an open junction, a point
  across the axis, a point inside the snap band and an arc centre across the
  axis.
- `surdSign` is checked against agreeing terms, vanishing terms, opposed
  terms of either larger magnitude and exact cancellation, and the surd
  point's tests against a rational point where `R` is a square.
