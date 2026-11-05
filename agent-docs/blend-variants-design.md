# Edge blend variants design

One part per edge blend variant. Each part keeps its own decision IDs and `§` numbers; cite a `§` number as "Part name §N".

## Chord fillet

`FilletChord` rounds an edge with an arc stated by its chord, the straight line between the arc's two tangent edges.
It is a feature-level entry point over `Fillet`: it computes the radius and adds no geometry kernel. The fillet
routes are `agent-docs/modify-design.md` §6 and `agent-docs/modify-reach-design.md`; this document owns the
conversion.

### 1. Contract

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

### 2. The dihedral

`Edge.planarDihedral` (`edge_dihedral.go`) reads φ from the unit outward normals of the edge's two planes: the angle
between them is `atan2(|n₁ × n₂|, n₁ · n₂)`, accurate near a flat edge and near a knife edge as well as at a right angle,
and φ is π less that. It is data of the recorded planes, not a test of a claim, so it blesses nothing: the radius is a
float function of those planes, and the result is `Fillet`'s result for a radius of that value. `ChamferAngle` reads its
dihedral the same way (`agent-docs/blend-variants-design.md`, Angle chamfer §2).

The same-radius test of C6 only groups edges. Two equal dihedrals read from different face pairs differ in rounding,
far below 1e-12, and the one radius used differs from the other edge's by less than that, far below the bound of the
result `Fillet` publishes.

`WithTangentChain` is not offered: the chain would add edges C4 has not checked.

### 3. Exactness

The radius is a float, so a result is as exact as `Fillet`'s for a radius of that value. The tests compare volumes
against `(1 − π/4)·(chord²/2)·height` on a right angle within a relative `1e-9`, and against `R²(cot(φ/2) − (π − φ)/2)·height`
on the 120 degree edges of a hexagonal prism.

### 4. Not covered

- A curved face beside a straight or free-form edge, and a torus: the angle there varies along the edge.
- `WithTangentChain`.
- Setback corners and curvature-continuous blends, which have no entry point.

### 5. Edges that are not right angled

The hexagonal prism's six vertical edges meet at 120 degrees. `FilletChord` of one gives the radius `chord`, and the
tests compare the volume with the area `R²(cot 60° − 30°)` a fillet of that radius removes from the corner. All six
together give one radius and are one call. A right-isosceles triangular prism's 45, 45 and 90 degree edges together are
`ErrUnsupported`, and the L-shaped prism's reflex edge is a concave edge: its faces turn 90 degrees, so a chord of `2√2` is a radius of 2 and the round adds `(1 − π/4)·r²·height` to the corner.

### 6. A circular edge between a plane and a surface of revolution

An edge that is a `Circle3` or an `Arc3` may lie between a plane and a `Cylinder`, `Cone` or `Sphere`, or between two of
those: such a circle lies on a surface of revolution about its own axis, so it meets the other face at one angle all round.
The rim of a rod, the mouth of a bore and the foot of a boss are such edges.

| Decision | Rule |
|---|---|
| C7 | φ is read from the two faces' outward normals at the edge's start, as `planarDihedral` reads it for planes. The radius is `chord/(2·cos(φ/2))` as C1 says, which is `chord/√2` for the right angle between a cap and a wall. |
| C8 | A curved face beside a straight or free-form edge, and a `Torus` anywhere, is `ErrUnsupported`: the angle there varies along the edge. Two faces that meet tangentially have φ of π and are refused. |
| C9 | The result is `Fillet`'s for that radius, so its refusals apply. The tests compare a rod's two rims with `∫ 2π(R−x)(r−√(r²−(r−x)²))dx` over the round. |

## Angle chamfer

`ChamferAngle` bevels an edge by a distance along a reference face and an angle between the bevel and that face. It
is a feature-level entry point over `Chamfer` and `WithAsymmetricChamfer`: it computes the second setback and adds no
geometry kernel. The setback model is `agent-docs/modify-reach-design.md` §6; this document owns the conversion.

### 1. Contract

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

### 2. The dihedral

`Edge.planarDihedral` (`edge_dihedral.go`) reads φ from the unit outward normals of the edge's two planes, as
`agent-docs/blend-variants-design.md`, Chord fillet §2 states: the angle between them is `atan2(|n₁ × n₂|, n₁ · n₂)` and φ is π less
that. The second distance is a float function of those recorded planes and blesses no claim about them, so a result
is as exact as `Chamfer`'s asymmetric result for a distance of that value. The same-distance test of A7 only groups
edges, and the one distance used differs from another edge's by less than a relative 1e-12.

`WithTangentChain` is not offered: the chain would add edges A5 has not checked.

### 3. Exactness

The second setback is a float, so a result is as exact as `Chamfer`'s asymmetric result for a distance of that value.
The tests compare volumes against `0.5·d·other·sin(φ)·height` within a relative `1e-9`, and they recover the bevel's
angle from the volume independently of the formula.

### 4. Not covered

- A curved face beside a straight or free-form edge, and a torus: the angle there varies along the edge.
- The angle measured from the other face. Name the other face as `reference`.
- `WithTangentChain`.

### 5. Edges that are not right angled

The hexagonal prism's 120 degree edges and a right-isosceles prism's 45 degree edge take the sine rule. A 120 degree
edge with a bevel angle of 60 degrees or more never meets the other face and is `ErrDegenerate`, and a bevel that
reaches past the face's own width is `Chamfer`'s refusal.

### 5. A circular edge between a plane and a surface of revolution

`ChamferAngle` takes the edges `FilletChord` does in Chord fillet §6: a circular edge
between a plane and a `Cylinder`, `Cone` or `Sphere`, whose angle is one all round. φ is read as A1 says and the second
distance follows A1. A curved face beside a straight or free-form edge is `ErrUnsupported`. The tests compare a rod's rims
with the ring a triangle of legs `d` and `d·tan(angle)` sweeps about the axis.

## Oblique boss fillet

This design covers one complete junction between a rectangular plate's top
plane and a rooted circular cylinder whose axis rises obliquely. It extends
`agent-docs/general-boolean-design.md` and `agent-docs/modify-general-design.md` without
changing either operation's public API.

### Source and admission

The ordinary mesh `Union` remains responsible for finding contact and
proving the unmodified joined body. After a successful `Union`, it may retain
the two original prism records when one is an axis-aligned rectangular plate
and the other is a whole circular cylinder. The cylinder's near cap must lie
strictly below the plate's top, its far cap strictly above it, and its entire
projected intersection and requested fillet must lie inside the plate top.
The plate bottom and all four walls remain untouched. These inequalities
include the operands' recorded placement and section errors. A partial
junction, a side contact, a detached cap, or a tilted plate is not admitted.
The bounded implementation admits a 1–20° tilt, a cylinder length at least
its radius, radius and length at most 100 mm, and source coordinates within
10,000 mm. The recorded section and levels must have zero displacement.
The fillet radius is positive and below one quarter of the boss radius.

`Fillet` accepts exactly the one complete, closed junction edge between the
plate top and cylinder side. It rebuilds from the retained prisms; the mesh
edge is only a selection witness. Other edge selections keep their existing
refusal. The result is a faceted body with a dedicated blend face and source
provenance on the retained faces.

### Geometry

Let `H` be the plate top, `b` the cylinder's near-cap center, `a` its unit
axis, `R` its radius, and `r` the fillet radius. For radial unit vector
`d(u)` perpendicular to `a`, a rolling-ball center is

```
C(u) = b + (R+r)d(u) + a (H+r-b.z-(R+r)d.z(u))/a.z.
```

Every `C` lies in the plane `z=H+r`. The plane contact is `P=C-r ez` and the
cylinder contact is `S=C-r d`. The blend patch is

```
F(u,v) = C(u) - r normalize((1-v)ez + v d(u)),  0<=v<=1.
```

Both contact curves are shared vertex-for-vertex with their neighbors. The
top plate is the square-to-`P` annulus; the cylinder side is the `S`-to-far
cap ruled band. The mesh also contains the original plate walls, bottom cap,
and boss far cap.

### Proofs and refusal

The source placement and exact Sketch section bounds precede every mesh
claim. Rational intervals enclose every sampled sine, cosine, normal length,
and vertex position. The complete angular plan has gaps below 0.05 radians;
each blend interval is 1/16. For the admitted basis, `|q|>0.56` throughout
the blend. Second-derivative bounds on `F` charge angular interpolation,
blend interpolation, and the diagonal split into triangles. The near-unit
source radial basis adds at most `0.02r` where the blend meets the ruled
cylinder. The placed circle's two certified caps and linear ruling contribute
their source displacement, amplified by 16 through the plane crossing and
unit normal. The amplification covers at most `4ε/R` of radial-normal change
for a near-rigid affine image with cap displacement `ε`, `r/R<1/4`, plane
division by `a.z>0.94`, and normal division by `|q|>0.56`; the resulting
position change is below `16ε`. The resulting boundary bound must be below
0.1 mm.

The angularly ordered square and ellipse rings make a planar annulus; the
ruled cylinder side and rolling-ball band share their contact vertices.
An exact mesh audit checks closure, orientation and crossings. For occupied
volume, a full rectangular collar around the junction charges every possible
source-to-held change in the fillet zone. Outside that collar, a radial
annulus over the cylinder length and cap slabs enclose the remaining chord
and placement difference. The bound must be finite and leave a positive
lower volume. A clean mesh audit alone never supplies a source proof.

The output does not claim an analytic torus: its rolling-ball center follows
an ellipse, so a torus would be the wrong surface. Placement of the result
uses the existing faceted placement proof and drops the one-use source record.
