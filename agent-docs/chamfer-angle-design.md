# Chamfer by distance and angle design

`ChamferAngle` bevels an edge by a distance along a reference face and an angle between the bevel and that face. It
is a feature-level entry point over `Chamfer` and `WithAsymmetricChamfer`: it computes the second setback and adds no
geometry kernel. The setback model is `agent-docs/modify-reach-design.md` §6; this document owns the conversion.

## 1. Contract

```go
func ChamferAngle(ctx context.Context, b *Body, sel EdgeSelector, d units.Value, reference FaceSelector,
    angle units.Value) (*Body, error)
```

| Decision | Rule |
|---|---|
| A1 | The bevel leaves the reference face `d` from the edge and makes `angle` with that face. It meets the other face `d·tan(angle)` from the edge. |
| A2 | `ChamferAngle` calls `Chamfer(ctx, b, sel, d, WithAsymmetricChamfer(reference, d·tan(angle)))`. Receivers, routes, history entry (`OperationChamfer`) and refusals are `Chamfer`'s. |
| A3 | `angle` is strictly between 0 and a quarter turn. Zero or a quarter turn or more is `ErrDegenerate`, a negative value is `ErrNegativeMagnitude`, and a wrong unit kind is `ErrUnitKind`. |
| A4 | `d` is a positive length under `Chamfer`'s own gates. |
| A5 | Each selected edge has exactly two faces, both `Plane`, whose normals have a dot product of exactly zero. Any other edge is `ErrUnsupported`. |
| A6 | A nil context, body or reference is `ErrDegenerate`. A selector `Chamfer` would refuse is passed to `Chamfer` for its own error. |

## 2. The right-angle gate

The conversion `d·tan(angle)` is the other setback only where the two faces meet at a right angle. An edge at another
dihedral needs the sine rule, and decad would have to read that dihedral from the face tags. A decad-side test of a
near-right angle can only falsify (CLAUDE.md "Reject-only"), so A5 admits an edge only when the plane normals are
exactly perpendicular, as the edges of an axis-aligned box and of prisms on exact frames are. A merely close dihedral
is refused.

`WithTangentChain` is not offered: the chain would add edges A5 has not checked.

## 3. Exactness

The second setback is a float product, so a result is as exact as `Chamfer`'s asymmetric result for a distance of
that value. The tests compare volumes against `0.5·d·d·tan(angle)·height` within a relative `1e-9`.

## 4. Not covered

- An edge whose faces meet at an angle other than a right angle, or a curved face beside the edge.
- The angle measured from the other face. Name the other face as `reference`.
- `WithTangentChain`.
