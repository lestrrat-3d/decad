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
| C1 | Where the two faces meet at the angle φ across the corner the arc turns π − φ, so its radius is `chord/(2·cos(φ/2))`, which is `chord/√2` on a right angle. |
| C2 | `FilletChord` calls `Fillet(ctx, b, sel, radius)`. Receivers, routes, history entry (`OperationFillet`) and refusals are `Fillet`'s. |
| C3 | `chord` is a positive length. Zero is `ErrDegenerate`, a negative value is `ErrNegativeMagnitude`, and a wrong unit kind is `ErrUnitKind`. |
| C4 | Each selected edge has exactly two faces, both `Plane`, and is convex or concave. φ is the interior angle of a convex edge and the open wedge between the faces of a concave one, which the round fills. Any other edge is `ErrUnsupported`. |
| C5 | A nil context or body is `ErrDegenerate`. A selector `Fillet` would refuse is passed to `Fillet` for its own error. |
| C6 | `Fillet` takes one radius, so every selected edge MUST give the same one within a relative 1e-12, which equal dihedrals do. A selection that mixes angles is `ErrUnsupported`: take the angles in separate calls. |

## 2. The dihedral

`Edge.planarDihedral` (`edge_dihedral.go`) reads φ from the unit outward normals of the edge's two planes: the angle
between them is `atan2(|n₁ × n₂|, n₁ · n₂)`, accurate near a flat edge and near a knife edge as well as at a right angle,
and φ is π less that. It is data of the recorded planes, not a test of a claim, so it blesses nothing: the radius is a
float function of those planes, and the result is `Fillet`'s result for a radius of that value. `ChamferAngle` reads its
dihedral the same way (`agent-docs/chamfer-angle-design.md` §2).

The same-radius test of C6 only groups edges. Two equal dihedrals read from different face pairs differ in rounding,
far below 1e-12, and the one radius used differs from the other edge's by less than that, far below the bound of the
result `Fillet` publishes.

`WithTangentChain` is not offered: the chain would add edges C4 has not checked.

## 3. Exactness

The radius is a float, so a result is as exact as `Fillet`'s for a radius of that value. The tests compare volumes
against `(1 − π/4)·(chord²/2)·height` on a right angle within a relative `1e-9`, and against `R²(cot(φ/2) − (π − φ)/2)·height`
on the 120 degree edges of a hexagonal prism.

## 4. Not covered

- A curved face beside a straight or free-form edge, and a torus: the angle there varies along the edge.
- `WithTangentChain`.
- Setback corners and curvature-continuous blends, which have no entry point.

## 5. Edges that are not right angled

The hexagonal prism's six vertical edges meet at 120 degrees. `FilletChord` of one gives the radius `chord`, and the
tests compare the volume with the area `R²(cot 60° − 30°)` a fillet of that radius removes from the corner. All six
together give one radius and are one call. A right-isosceles triangular prism's 45, 45 and 90 degree edges together are
`ErrUnsupported`, and the L-shaped prism's reflex edge is a concave edge: its faces turn 90 degrees, so a chord of `2√2` is a radius of 2 and the round adds `(1 − π/4)·r²·height` to the corner.

## 6. A circular edge between a plane and a surface of revolution

An edge that is a `Circle3` or an `Arc3` may lie between a plane and a `Cylinder`, `Cone` or `Sphere`, or between two of
those: such a circle lies on a surface of revolution about its own axis, so it meets the other face at one angle all round.
The rim of a rod, the mouth of a bore and the foot of a boss are such edges.

| Decision | Rule |
|---|---|
| C7 | φ is read from the two faces' outward normals at the edge's start, as `planarDihedral` reads it for planes. The radius is `chord/(2·cos(φ/2))` as C1 says, which is `chord/√2` for the right angle between a cap and a wall. |
| C8 | A curved face beside a straight or free-form edge, and a `Torus` anywhere, is `ErrUnsupported`: the angle there varies along the edge. Two faces that meet tangentially have φ of π and are refused. |
| C9 | The result is `Fillet`'s for that radius, so its refusals apply. The tests compare a rod's two rims with `∫ 2π(R−x)(r−√(r²−(r−x)²))dx` over the round. |
