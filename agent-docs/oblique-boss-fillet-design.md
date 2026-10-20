# Oblique cylindrical boss fillet

This design covers one complete junction between a rectangular plate's top
plane and a rooted circular cylinder whose axis rises obliquely. It extends
`agent-docs/general-boolean-design.md` and `agent-docs/modify-general-design.md` without
changing either operation's public API.

## Source and admission

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

## Geometry

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

## Proofs and refusal

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
