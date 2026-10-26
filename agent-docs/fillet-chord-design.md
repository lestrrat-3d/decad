# Fillet by chord length design

`FilletChord` rounds an edge with an arc stated by its chord, the straight line between the arc's two tangent edges.
It is a feature-level entry point over `Fillet`: it computes the radius and adds no geometry kernel. The fillet
routes are `agent-docs/modify-design.md` §6 and `agent-docs/modify-reach-design.md`; this document owns the
conversion.

## 1. Contract

```go
func FilletChord(ctx context.Context, b *Body, sel EdgeSelector, chord units.Value) (*Body, error)
```

| Decision | Rule |
|---|---|
| C1 | On a right-angled edge the arc turns a quarter turn, so its radius is `chord/√2`. |
| C2 | `FilletChord` calls `Fillet(ctx, b, sel, chord/√2)`. Receivers, routes, history entry (`OperationFillet`) and refusals are `Fillet`'s. |
| C3 | `chord` is a positive length. Zero is `ErrDegenerate`, a negative value is `ErrNegativeMagnitude`, and a wrong unit kind is `ErrUnitKind`. |
| C4 | Each selected edge has exactly two faces, both `Plane`, whose normals have a dot product of exactly zero. Any other edge is `ErrUnsupported`. |
| C5 | A nil context or body is `ErrDegenerate`. A selector `Fillet` would refuse is passed to `Fillet` for its own error. |

## 2. The right-angle gate

The radius is `chord/√2` only where the two faces meet at a right angle. Another dihedral turns the arc through a
different angle `α`, and the radius is `chord/(2·sin(α/2))`, which needs the dihedral read from the face tags. A
decad-side test of a near-right angle can only falsify (CLAUDE.md "Reject-only"), so C4 admits an edge only when
the plane normals are exactly perpendicular. `ChamferAngle` applies the same gate through the shared
`Edge.requireRightAngle` (`agent-docs/chamfer-angle-design.md` §2).

`WithTangentChain` is not offered: the chain would add edges C4 has not checked.

## 3. Exactness

The radius is a float quotient, so a result is as exact as `Fillet`'s for a radius of that value. The tests compare
volumes against `(1 − π/4)·(chord²/2)·height` within a relative `1e-9`.

## 4. Not covered

- An edge whose faces meet at an angle other than a right angle, or a curved face beside the edge.
- `WithTangentChain`.
- Setback corners and curvature-continuous blends, which have no entry point.
