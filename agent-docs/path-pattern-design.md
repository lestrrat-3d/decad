# Path pattern design

`PathPatternCopies` and `PathPatternSpaced` repeat a body at points spaced along a `Path`, by count and by distance. It places each instance with `Copy` and a
translation and adds no geometry kernel. The points come from `internal/pathsample/`, which works on plain
millimetre vectors.

## 1. Contract

```go
func PathPatternCopies(ctx context.Context, b *Body, path *Path, count int, opts ...PathPatternOption) ([]*Body, error)
func PathPatternSpaced(ctx context.Context, b *Body, path *Path, spacing units.Value, opts ...PathPatternOption) ([]*Body, error)
func WithPathFollow() PathPatternOption
func WithPathPivot(pivot r3.Vec) PathPatternOption
func PathPatternJoin(ctx context.Context, target, feature *Body, path *Path, count int, opts ...PathPatternOption) (*Body, error)
func PathPatternJoinSpaced(ctx context.Context, target, feature *Body, path *Path, spacing units.Value, opts ...PathPatternOption) (*Body, error)
```

| Decision | Rule |
|---|---|
| P1 | Instance 0 is `b`, taken to sit at the path's start. The result holds the `count − 1` other instances, as `PatternCopies` does, and `b` stays live. |
| P2 | Instance `i` is `Copy(b, WithCopyPlacement(Translation(point(i) − point(0))))`, where `point(i)` lies `i/(count − 1)` of the way along the path's total length. The last instance sits at the path's end. |
| P3 | Instances keep the orientation of `b` unless `WithPathFollow` is given (§5). |
| P4 | Points are spaced by arc length over the whole path, not per segment. A short segment can hold no instance and a long one many. |
| P5 | `count` below 2, a nil path and a retired body are `ErrDegenerate`, `ErrDegenerate` and `ErrRetiredBody`. A nil context or a body of no document is `ErrDegenerate`. |
| P6 | A failed or canceled call retires every copy it already made and returns the error, so no copy outlives the call. |

## 2. Sampling

`pathsample.Sample` resolves each segment to a length and a position-at-length function.

- A `LineTo` segment is the straight line from the previous end, with the exact linear interpolation.
- An `ArcThrough` segment is the circle through the previous end, the through point and the end. Its centre is the
  point of the plane of the three that is equally far from them, and its sweep runs from the previous end, through the
  through point, to the end. The arc turns counter-clockwise about the normal of those three points in that order, so
  the sweep is the angle of the end in `[0, 2π)` and a clockwise arc in the world frame is a counter-clockwise one
  about the opposite normal.
- An `ArcAround` segment samples its explicit center and oriented axis using the path record's held directed angle.
- Points are the first at the start and the last at the end exactly. Those between come from arc length and so carry
  float rounding.

## 3. Exactness

A copy translated by a float vector is a `Copy` under a rigid motion, with the readings `Copy` gives for that motion.
The translation of an arc point is not exact, so such an instance reads as `Approximate`.

## 4. Not covered

- Turning instances about the path's own axis.

## 5. Following the path

`WithPathFollow` turns each copy to follow the path's direction of travel. `pathsample.SampleStations` returns each
point with the unit direction there, and `pathsample.Turn` gives the shortest rotation between two directions.

| Decision | Rule |
|---|---|
| F1 | The body is taken to sit at the path's start with its own forward along the path's direction there, which is the direction of instance 0. |
| F2 | Instance `i` is the turn about the path's start by the shortest rotation taking the start direction onto the direction at `point(i)`, followed by the move of P2. A body away from the start swings round it. |
| F3 | A line's direction is constant, and an arc's is the derivative of its circle at that length. A point where two segments meet takes the direction of the segment that holds it, and the last point takes the direction at the path's end. |
| F4 | The turn has no roll about the direction of travel: it is the shortest rotation, so a copy never twists about its own forward beyond what the path's bending carries. |
| F5 | A direction exactly opposite the start's has no shortest axis and is `ErrDegenerate`. A nil option is `ErrDegenerate`. |
| F7 | `WithPathPivot` replaces the path's start in F2: the turn is about the stated point, in millimetres, and the move of P2 follows it. A body centred on the pivot turns in place. The last pivot wins. A pivot with no `WithPathFollow` is `ErrDegenerate`, and a non-finite pivot is `ErrNotFinite`. |
| F6 | A direction equal to the start's leaves the copy unturned, so a straight path gives the same copies with and without the option. |

## 6. Spacing by distance

`PathPatternSpaced` places copies at a stated arc length instead of a count. `pathsample.SampleSpaced` returns the points.

| Decision | Rule |
|---|---|
| S1 | Instance `i` lies `i · spacing` of arc length along the path, for every `i` with `i · spacing` within the path's length. The first is at the path's start. |
| S2 | A spacing that divides the length, up to a relative 1e-12 of rounding, gives the last instance at the path's end exactly. Otherwise the last lies short of the end. |
| S3 | `spacing` is a positive length. A wrong unit kind is `ErrUnitKind`, a negative value is `ErrNegativeMagnitude`, and zero is `ErrDegenerate`. |
| S4 | A spacing that leaves no copy, because it is longer than the path, is `ErrDegenerate`. One that would place more than 100000 points is `ErrDegenerate`. |
| S5 | Orientation, `WithPathFollow`, the path's segments, failure and the copies' exactness are `PathPatternCopies`'s (P1 to P6, §5). |

## 7. Joining the copies

`PathPatternJoin` and `PathPatternJoinSpaced` are `PathPatternCopies` and `PathPatternSpaced` followed by one `Union`,
as `PatternJoin` is for `LinearPattern` and `CircularPattern`.

| Decision | Rule |
|---|---|
| J1 | The instances are the copies of `feature` as P1 to P6 and S1 to S5 say, `feature` itself being instance 0. The join is `Union(target, feature, copies...)`, so it takes the routes `Union` takes over those operands, and its result is `Union`'s. |
| J2 | `target` is live, distinct from `feature` and not a sheet. A nil, retired or equal target is `ErrDegenerate`, as is a sheet, which `Union` refuses. `feature`, `path` and the count or spacing carry the gates of `PathPatternCopies` and `PathPatternSpaced`. |
| J3 | A refused `Union` retires every copy, so the call leaves `target` and `feature` live and the document as it was. A success retires `target`, `feature` and every copy and returns the joined body. |
| J4 | The history holds the copies' `OperationCopy` entries and the `Union`'s own entry. |
