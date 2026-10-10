# Point containment and distance design

`Body.LocatePoint` classifies one world-coordinate point against a solid body.
The query reads a verified mesh and its occupied-volume proof; it does not
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
a mesh returns that tessellation's `ErrUnsupported`. A sheet or an unsound solid
returns `ErrNotSolid`. A nil body or context returns `ErrDegenerate`; non-finite
point coordinates return `ErrNotFinite`. The tolerance uses `Tessellate`'s
existing kind, finiteness and magnitude gates. Retired bodies remain readable.

`PointUndecided` means the proof cannot classify this point at the requested
tolerance. It is not an outside reading. The caller may request a finer mesh;
the query can still return `PointUndecided` if a payload publishes no occupied
volume proof or the point is too close to its boundary. `PointOnBoundary` is
reported only when the mesh boundary is exact at that point, as §2 states.

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
  it; its curved wall is undecided at a point within the requested mesh bound.
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
square-root endpoints and `ratIntervalMeasurement` to return a finite
`Measurement`. This query needs no occupied-volume proof or point-membership
classification.

Verify a box cap for points on the patch, inside the body, and beyond one
corner. Verify a curved wall using its analytic radius and a surface-extruded
sheet face.
