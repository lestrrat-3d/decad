# Point containment and distance design

`Body.LocatePoint` classifies one world-coordinate point against a solid body.
An undisplaced straight prism is first classified against its own record in
exact arithmetic (§7). A mesh Boolean result first combines its operands' exact
answers through the operation history (§8). Every other body, and every point
§7 or §8 leaves open, reads a verified mesh and its occupied-volume proof (§2). The query does not
change the body or the document.

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
a mesh returns that tessellation's `ErrUnsupported` when §7 or §8 does not
decide the point. A sheet or an unsound solid returns `ErrNotSolid`. A nil body or context
returns `ErrDegenerate`; non-finite point coordinates return `ErrNotFinite`. The
tolerance uses `Tessellate`'s kind, finiteness and zero gates, which §7 and §8
apply themselves before they read a record, so an invalid tolerance fails the
same way on every body. Retired bodies remain readable.

`PointUndecided` means the proof cannot classify this point at the requested
tolerance. It is not an outside reading. The caller may request a finer mesh;
the query can still return `PointUndecided` if a payload publishes no occupied
volume proof or the point is too close to its boundary. `PointOnBoundary` is
reported only when the record places the point on its boundary exactly (§7),
the operands' records do so under §8's set algebra, or the mesh boundary is
exact at that point (§2).

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
- A stacked blind pocket and a faceted boolean result answer inside or outside
  at points whose ball proof clears their symmetric-difference bounds.
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

The body's payload must be a `prismPayload` with `SectionDelta`, `Z0Delta` and
`Z1Delta` all zero, no `SurfaceResult`, and `Z0 < Z1` compared exactly. Only
then is the record the set it denotes (`prismrecord.Payload`'s doc,
`agent-docs/prism-boolean-design.md` §7). A faceted Boolean result goes to §8.
A displaced record, a stacked prism, a BRep and every other payload go to §2.

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

`z < Z0` or `z > Z1` is `PointOutside` without reading the profile.

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
  by its own record (§7) or by §2.
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

A leaf whose payload is a `prismrecord.Payload` is classified by §7. A leaf
§7 does not decide, and every other leaf, reads undecided. The route reads no
leaf's mesh: that would tessellate each retired operand at `VerifyAll`.

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
  one-input `Cut`, a mesh operand with no record and a displaced prism
  operand; a chain allowance still combines, and a canceled context returns
  its error.
