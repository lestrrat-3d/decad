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
| A1 | The bevel leaves the reference face `d` from the edge and makes `angle` with that face. It meets the other face `d·sin(angle)/sin(φ + angle)` from the edge, where φ is the interior angle between the two faces. That is `d·tan(angle)` on a right angle. |
| A2 | `ChamferAngle` calls `Chamfer(ctx, b, sel, d, WithAsymmetricChamfer(reference, other))`. Receivers, routes, history entry (`OperationChamfer`) and refusals are `Chamfer`'s. |
| A3 | `angle` is strictly between 0 and a half turn. Zero or a half turn or more is `ErrDegenerate`, a negative value is `ErrNegativeMagnitude`, and a wrong unit kind is `ErrUnitKind`. An edge whose φ makes `φ + angle` reach a half turn has a bevel that never meets the other face: `ErrDegenerate`. |
| A4 | `d` is a positive length under `Chamfer`'s own gates. |
| A5 | Each selected edge has exactly two faces, both `Plane`, and is convex or concave. A concave edge's φ is the open wedge the faces make in the air, which the bevel fills, so the same formula holds. Any other edge is `ErrUnsupported`. |
| A6 | A nil context, body or reference is `ErrDegenerate`. A selector `Chamfer` would refuse is passed to `Chamfer` for its own error. |
| A7 | `WithAsymmetricChamfer` takes one second distance, so every selected edge MUST give the same one within a relative 1e-12, which equal dihedrals do. A selection that mixes angles is `ErrUnsupported`: take the angles in separate calls. |

## 2. The dihedral

`Edge.planarDihedral` (`edge_dihedral.go`) reads φ from the unit outward normals of the edge's two planes, as
`agent-docs/fillet-chord-design.md` §2 states: the angle between them is `atan2(|n₁ × n₂|, n₁ · n₂)` and φ is π less
that. The second distance is a float function of those recorded planes and blesses no claim about them, so a result
is as exact as `Chamfer`'s asymmetric result for a distance of that value. The same-distance test of A7 only groups
edges, and the one distance used differs from another edge's by less than a relative 1e-12.

`WithTangentChain` is not offered: the chain would add edges A5 has not checked.

## 3. Exactness

The second setback is a float, so a result is as exact as `Chamfer`'s asymmetric result for a distance of that value.
The tests compare volumes against `0.5·d·other·sin(φ)·height` within a relative `1e-9`, and they recover the bevel's
angle from the volume independently of the formula.

## 4. Not covered

- A curved face beside the edge.
- The angle measured from the other face. Name the other face as `reference`.
- `WithTangentChain`.

## 5. Edges that are not right angled

The hexagonal prism's 120 degree edges and a right-isosceles prism's 45 degree edge take the sine rule. A 120 degree
edge with a bevel angle of 60 degrees or more never meets the other face and is `ErrDegenerate`, and a bevel that
reaches past the face's own width is `Chamfer`'s refusal.
