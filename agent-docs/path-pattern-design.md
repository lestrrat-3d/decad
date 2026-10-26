# Path pattern design

`PathPatternCopies` repeats a body at points spaced along a `Path`. It places each instance with `Copy` and a
translation and adds no geometry kernel. The points come from `internal/pathsample/`, which works on plain
millimetre vectors.

## 1. Contract

```go
func PathPatternCopies(ctx context.Context, b *Body, path *Path, count int) ([]*Body, error)
```

| Decision | Rule |
|---|---|
| P1 | Instance 0 is `b`, taken to sit at the path's start. The result holds the `count − 1` other instances, as `PatternCopies` does, and `b` stays live. |
| P2 | Instance `i` is `Copy(b, WithCopyPlacement(Translation(point(i) − point(0))))`, where `point(i)` lies `i/(count − 1)` of the way along the path's total length. The last instance sits at the path's end. |
| P3 | Instances keep the orientation of `b`. None turns to follow the path's tangent. |
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
- Points are the first at the start and the last at the end exactly. Those between come from arc length and so carry
  float rounding.

## 3. Exactness

A copy translated by a float vector is a `Copy` under a rigid motion, with the readings `Copy` gives for that motion.
The translation of an arc point is not exact, so such an instance reads as `Approximate`.

## 4. Not covered

- Turning instances to follow the path's tangent.
- A path of any segment kind beyond `LineTo` and `ArcThrough`, which is all `Path` holds.
- Spacing by a stated distance instead of a count.
- Joining the copies to a base. Pass them with the base to one `Union`.
