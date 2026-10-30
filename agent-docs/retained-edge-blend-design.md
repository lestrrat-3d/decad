# Retained Edge Blend Design

`Fillet` and `Chamfer` of an edge of a mesh Boolean result whose two faces
carry retained analytic surfaces (`agent-docs/faceted-source-retention-design.md`,
"retention §N" below). Companion to `agent-docs/modify-design.md` ("modify §N"),
which owns the call contracts and base gates, and to
`agent-docs/modify-reach-design.md` ("reach §N"), whose SX9 row this document
narrows. Nothing here changes a prism, revolve or brep receiver.

## 1. Problem

Probe on `main` on 2026-10-20 (retention §1, row P4): a 40×20×20 box with four
filleted lateral edges, cross-drilled by a tool that a `Transform` placed, then
`Fillet` of the four x-parallel cap edges. The call returns `ErrUnsupported`
(reach SX9). The same box drilled by an unplaced tool is a `brepPayload`
(general-boolean class B) and takes the brep routes. A drilled plate whose
tool was placed, rotated or cut at an angle is the common machining shape, and
it is the shape the mesh path exists for.

Three things stop the call today.

1. **No analytic edge.** Before retention an edge of a mesh result was a chord
   chain (`FacetedCurve`) between two `Faceted` faces. Retention §10 now
   publishes `Line3`, `Circle3` and `Arc3` for edges between two retained
   faces, and §10.5 adds `Ellipse3`.
2. **No blend by Boolean.** A rolling-ball blend can be written as a Cut by a
   cutter solid, but the cutter's blend face is tangent to both faces it cuts.
   The mesh Boolean refuses that contact (`an operand edge grazes along the
   other operand's facet`, evaluator §9), and it should.
3. **No record the brep routes read.** `brepPayload` keeps every frame a signed
   permutation of one reference frame (general-boolean §4.1). A rotated or
   oblique result has no such record, which is why it is a mesh result.

This design builds the blend as a local rewrite of the held mesh, in the way
the prism route rewrites a section (modify §2) and reach §10.1 rewrites a
Union's source. The rewrite replaces the facets near one edge by facets that
lie on exact surfaces computed from the two retained tags.

## 2. What the design reads

| Input | Source | Used for |
|---|---|---|
| The two retained tags `T1`, `T2` | retention §3 | the exact surfaces the blend is tangent to |
| The edge's exact curve | retention §10 | the line or circle the blend is measured from |
| Held facets and vertex bounds | retention §3.3, faceted-vertex-bounds §2 | what the rewrite cuts and keeps |
| The slab bound of a retained plane facet | retention §11 | how far a cut point on a held facet sits from the tag plane (§6.3) |

Tags with a stated departure (`NormalBound != 0`, a placed float image) are not
read: their plane or axis may differ from the true one, so an exact setback
computed from them has no proof (SR1). A tag moved by an exact translation
states a zero departure and is read: a `Transform` whose moved origin is the
exact rational sum, whether it moved the Boolean result or an analytic operand
before the Boolean (retention §3.5), and the tags of a piece the shifted Boolean retry
translates (`agent-docs/gap-tolerance-design.md` §5.2), whose snapped shift makes that
sum exact for ordinary coordinates (retention §3.4). Such a blend is built on
the moved solid `A∘(B+s)`.

A face with a denoted patch (every Revolve wall and end face, `topo.Face.Denoted`)
is read only where its tag IS the surface its record denotes, decided by
`stitchflux.TagIsDenoted` on recorded data by exact equality: every leaf of the
patch is a point, the axis frame is exactly orthonormal in rationals, and the
tag equals the denoted cylinder or plane exactly. Nothing is admitted on a
residual. Only a `Plane` or `Cylinder` tag is considered here: R1–R5 refuse a
`Cone`, `Sphere` or `Torus` tag before they ask `TagIsDenoted`. A widened patch,
a partial-sweep cap, a rotated placement and an off-axis sketch line fail the
test and stay SR1. An admitted face is read as a retained `Plane` or `Cylinder`
tag, and every other gate of the route (the edge class, the end faces, the
setback and clearance gates) applies to it as to any retained face.

A `Sphere` face is read only by R6, through its own exact-record rule (§4.5):
`TagIsDenoted`'s sphere arm reads the centre and no radius, and R6 needs both.

**Interior vertices of an end face.** A revolve's end disc is triangulated as a
fan around a centre vertex on the axis, so the held end face has a vertex that
no boundary loop reaches. An end face `G` may hold such a vertex only where it
lies exactly on `G`'s plane tag (rational test of `n·(x − o) = 0` on the held
normal, origin and vertex). The same holds for the plane face of an R2 rim and
the base plane of a chain. Each of these faces is rewritten by discarding all
its facets and triangulating it again from its boundary loops alone (§5), so
the dropped vertex reaches neither the result nor its bounds, and the
exact-plane test shows it never carried geometry off the plane. A vertex off
the plane by any amount, and any stray vertex on a wall face, still refuses
(SR5).

## 3. Edge classes

Table RR lists the edges a blend can be built on. The others are refused.

| RR | Edge | Faces | Blend surface (fillet / chamfer) | Staged |
|---|---|---|---|---|
| **R1** | `Line3`, convex or concave | two retained `Plane`s, not parallel, ending at three-face corners whose third face is a retained `Plane` | `Cylinder` / `Plane` | increment B2; equal or two-distance chamfer |
| **R2** | `Circle3`, closed, centre on the axis | retained `Plane` and coaxial retained `Cylinder` | `Torus` / `Cone` | increment B1; equal or two-distance chamfer |
| **R3** | `Line3` of retention E4 | retained `Plane` and a `Cylinder` whose axis lies in the plane, ending on retained planes normal to the axis | `Cylinder` / `Plane` | increment B3; equal or two-distance chamfer |
| **R4** | `Arc3` | as R2, open, ending on planes parallel to the axis | as R2 | ends on two distinct retained planes; equal or two-distance chamfer; an arc whose ends meet one face is refused |
| **R5** | `Line3` of retention E10 | two retained `Cylinder`s with parallel axes, ending on retained planes normal to the axes | `Cylinder` / `Plane` | reach §8; equal or two-distance chamfer |
| **R6** | `Circle3` of retention E5, closed | retained `Plane` and a retained `Sphere` whose tag is exactly its record (§4.5) | `Torus` / `Cone` | §4.5; equal or two-distance chamfer |
| **R7** | closed saddle, any held curve | a retained `Cylinder` shaft (material inside) and a retained `Cylinder` bore of radius `r` (material outside), unit coordinate axes on two coordinates meeting exactly, `2r² < R²` | (next row) / ruled band held `Faceted` | §4.9; equal-setback `Chamfer`, by a mesh `Cut` |
| **R7 fillet** | a closed saddle, the ball outside the bore: inside the larger cylinder at a convex edge (σ = (−1, +1), the R7 mouth), or outside both at a convex edge (two crossing bores) or a concave one (a boss's root on a rod) (σ = (+1, +1)) | two retained `Cylinder`s of radii `R > r` with R7's axes and ratio: R7's faces, both `Reversed`, or neither | pipe held `Faceted` / (row above) | §4.10; `Fillet` by a rewrite of both walls; `2ρ < R − r` inside the larger cylinder |

Refused, with the reason a caller can act on:

- **An oblique plane and cylinder (`Ellipse3`).** The surface that rolls a ball
  between them is a pipe surface about an ellipse offset. It is neither a
  `Cylinder`, `Cone`, `Torus` nor `Sphere`, and the only other surface the
  vocabulary has is `NURBSSurface`, an opaque marker with no geometry to carry
  a proof (spline §7). The rim keeps its published ellipse and the blend
  refuses (SR1).
- **Any `Cylinder`/`Cylinder` rim with crossing axes outside R7 and its fillet
  row (a pin's root inside a tube's bore; a ball inside the bore; a bore off the
  larger cylinder's axis or off the coordinate axes; `2r² ≥ R²`; a chamfer of
  the fillet row's σ = (+1, +1) patterns), an open (`Arc3`) plane-sphere rim, a
  `Cone` or `Torus` face.** The ball rolled between crossing cylinders sweeps a
  canal surface about a quartic, which the vocabulary cannot carry as a tag, and
  only the R7 fillet row's sign patterns have a proven faceted pipe (§4.10). No
  Table RR row; SR1.
- **A rim with a face that is not retained,** or whose edge is a chord chain
  because retention §10 declined it. SR1.

Both the convex and the concave edge are built. A concave fillet adds
material in the corner, a convex one removes it. The two share the clip and
strip construction below; only the side of the faces the strip lies on and
the sign of the blend face's orientation change.

## 4. The exact blend

All values are computed in exact rational arithmetic from the tags' float
parameters and rounded once, upward where a bound is wanted.

### 4.1 R1, a straight edge between planes

Let `nA`, `nB` be the faces' outward unit normals (the frame normal, negated
for a `Reversed` face), `d̂ = (nA × nB) / |nA × nB|` the edge direction and
`φ ∈ (0, π)` the angle between the normals, `cos φ = nA · nB`. Let `e₀` be a
point of the edge line. The edge is convex when the face `A`'s inward
in-plane direction `uA` (the unit vector along `±(nA × d̂)` that points into
`A`'s held facets) satisfies `nB · uA < 0`; the held `Edge.Convex` flag must
agree or the call refuses (SR1). Let `s = −1` for a convex edge and `s = +1`
for a concave one.

| Quantity | Fillet radius `r` | Chamfer distances `dA`, `dB` |
|---|---|---|
| Setback on `A` and `B` | `t = r · tan(φ / 2)` | `dA`, `dB` |
| Setback line on `A` | the line at in-plane distance `t` from the edge, parallel to it, inside `A` | the same at `dA` |
| Setback line on `B` | the same at `t` | the same at `dB` |
| Blend surface | `Cylinder`, axis `d̂`, radius `r`, through `e₀ + s·b̂·r / cos(φ/2)` with `b̂ = (nA + nB) / |nA + nB|` | `Plane` through the two setback lines |
| Blend face orientation | `Reversed = false` convex, `true` concave | the plane's own normal, signed to point away from the material |

*Why.* A ball of radius `r` tangent to both planes has its centre on the line
parallel to the edge at distance `r / cos(φ/2)` along the bisector, and touches
each plane at distance `r · tan(φ/2)` from the edge. A 90° box edge has
`φ = π/2`, `t = r`, centre offset `r√2`, which is the familiar quarter circle.

The cylinder meets each face tangentially along its setback line, so the
construction needs the setback lines and nothing else of the cylinder. The
chamfer plane contains both setback lines, which are parallel.

### 4.2 R2, a closed circular rim

Take the meridian half-plane `(ρ, z)` about the common axis, with the plane at
`z = 0`, the cylinder at `ρ = R` and the rim at `(R, 0)`. The ball sits on the
material side for a convex rim and the void side for a concave one, and on the
outside of the cylinder (`ρ > R`) or the inside.

| Rim | Convex or concave | Ball centre `(ρ, z)` | `Torus.Major` |
|---|---|---|---|
| hole mouth: material at `ρ > R`, `z < 0` | convex | `(R + r, −r)` | `R + r` |
| boss root: boss at `ρ < R`, `z > 0`, plate at `z < 0` | concave | `(R + r, +r)` | `R + r` |
| shaft end: material at `ρ < R`, `z < 0` | convex | `(R − r, −r)` | `R − r` |
| blind-hole floor: void at `ρ < R`, `z > −h` | concave | `(R − r, −h + r)` | `R − r` |

The torus has `Center` on the axis at the centre's height, `Minor = r`, and the
blend face is the quarter of it between the two tangent circles: radius
`R ± r` on the plane (setback `r` from the rim along the plane) and height `r`
on the cylinder (setback `r` along the wall). A chamfer with distances `dA`
along the plane and `dB` along the wall is the `Cone` through the circle of
radius `R + σ·dA` on the plane and the circle at distance `dB` along the wall.
`σ = +1` when the plane face lies at `ρ > R`, else `−1`.

A fillet that would need `R − r ≤ 0` (the ball cannot sit inside the
cylinder) is SR3.

### 4.3 R3, a line between a plane and a parallel cylinder

Take the cross-section normal to the common direction `d̂` of the edge and the
cylinder's axis. The plane is a line `L` with outward normal `nA`; the
cylinder is a circle `C` of radius `R` about `o`, with outward normal `nB(x)`
radial (negated for a `Reversed` wall); the edge is their meeting `K`. The
corner is convex when the wall's material lies on the same side of `L` as the
plane's. The angle between the tangent of `C` at `K` and `L` must clear
`minSine`; a tangent meeting is not an edge.

**Fillet.** The ball touches `L` from the side `ball` (`−1` convex, `+1` concave)
along `nA`, so its centre lies on the line `L'` at distance `r` from `L`. It
touches `C` from the outside (`|c − o| = R + r`) when `κ = +1` and from the
inside (`|c − o| = R − r`) when `κ = −1`, where `κ = +1` exactly when the edge is
convex and the material lies outside `C`, or the edge is concave and the
material lies inside `C`. The centre is the point of `L' ∩ circle(o, R + κ·r)`
nearest `K`. The tangent points are `T_A = c − ball·r·nA` and the radial
projection `T_B` of `c` onto `C`. The unit vectors from the centre to them are
`a₀ = −ball·nA` and `a₁ = −κ·(c − o)/|c − o|`, `φ` is the angle between them, and
the blend arc runs between them through `φ`, as in R1. The surface is a
`Cylinder` of radius `r` and axis `d̂` through `c`.

**Chamfer.** The setback is `d` along `L` from `K` into the plane's face and arc
length `d` along `C` from `K` into the wall's face. The blend is the `Plane`
through the two generator lines.

The wall's setback line is the generator of the cylinder at `T_B`. The blend
removes the band of the wall whose angle about the axis, measured from `K` into
the wall, is below the angle of `T_B`, and, at an end where the wall extends
past the edge's corner (§5.0), which lies inside that end's plane `G`
(`internal/retainedblend/cyl_band.go`). Each held facet meets the band in a
convex polygon: the facet is clipped by `G`'s exact tag plane at each such
end, then by the half-plane that holds the axis and the generator. A vertex
is placed on a side only when it lies farther from the plane than its bound
plus the slack and `τ`; one within that distance refuses (SR5), except the
edge's corner and the wall's chain along `G`, which lie on `G`'s plane by
construction. A vertex placed outside an end plane is never read against the
generator: the band lies inside that plane, and the end plane's clip drops the
vertex before the generator's clip runs. A crossing on a held edge that reads
it still refuses an undecided side. So a feature of the wall past an end
plane, such as a cross hole, does not refuse, while one between the end planes
does. The facet keeps the rest, triangulated again with every point
where the band's boundary crosses a held facet edge and every corner of the
band inside it. A crossing point is read from the held edge's two vertices,
so the two facets beside the edge make the same point, and takes their larger
bound plus the slack and `τ`; a corner of the band, where the generator meets
`G`'s plane, takes the largest bound of its facet's vertices plus the same.
The crossings on the generator and the band's corners, in order along the
axis, are the strip's last row.

### 4.4 R4, an open arc

The surfaces are R2's: the same four rims, the same `κ`, torus or cone, with the
rim an arc of the circle from `V₀` to `V₁` and the faces ending on planes `G_i`
that are parallel to the axis. Each `G_i` meets the wall in a generator line and
the plane in a straight line, and the three meet at the end vertex.

The plane's setback curve is the arc of the circle of radius `ρ_A = R + κ·t`
between its points on the two end faces. A point `P_A,i` is where that circle
meets the line `A ∩ G_i`, taken on the side of the corner the chain leaves along
(a quadratic in the distance along the line). The wall's setback curve is the arc
at height `t_B` below the rim, and `P_B,i` is where it meets the generator
`B ∩ G_i`. The rows between them are the circles of the torus (or the straight
generators of the cone) at the tube angle `α_j`; row `j` runs from the azimuth
where its circle meets `G₀` to the one where it meets `G₁`, a solution of
`n_G · x = h_G` in the azimuth. The blend meets `G_i` in the curve those end
points trace, a section of a torus by a plane that is not a conic, so it stays a
chain of exact points.

The faces are rewritten as in R2 for the plane (a polygon on the setback arc)
and the wall (the cut at the setback height), and as in R1 for the ends.

### 4.5 R6, a plane-sphere rim

The edge is the closed `Circle3` of retention E5 between a retained `Plane`
(outward normal `n`, origin `o`) and a retained `Sphere` (centre `S`, radius
`R_s`). It is the rim a placed cut leaves on a revolved ball or dome.

**Exact record.** A `Sphere` face with a denoted patch (every Revolve sphere)
is read only where its tag is exactly the sphere its record states. Each test
compares recorded rationals by exact equality, and nothing is admitted on a
residual:

1. every leaf of the circular patch is a point and the axis frame is exactly
   orthonormal (`stitchflux.ExactRevolveOf`);
2. the recorded centre lies on the axis (`ρ = 0`), and the tag's centre is the
   axis point at the recorded `z` exactly;
3. the recorded radius is a point and equals the tag's radius exactly.

`stitchflux.TagIsDenoted` reads the first two and no radius, so the third is
this route's own. Each failure refuses SR1 with a message naming the leaf. A
rotated placement fails the first, a half-disk centre within the snap
tolerance of the axis fails the second, and a meridian whose recorded radius is
no float (an arc to `2.1` about `0.1`) fails the third. Retention §10.1 E12
applies the same three tests before it publishes the rim, so on a body built
by those three the rim already reads `FacetedCurve`, and `Unread` refuses it
SR1 as a tag not proven to be exactly its revolve record. A `Sphere` face with
no denoted patch is its own record, as for every other kind (§2).

**Rim.** Let `zs = n·(S − o)`. The rim's centre is `c = S − zs·n` and its radius
`ρ0 = √(R_s² − zs²)`. `ρ0 / R_s` is the sine of the angle at which the plane
crosses the sphere. A plane that misses or touches the sphere, or crosses it
below `minSine` (`10⁻³`), refuses SR5: a nearly tangent rim is no edge the
section can resolve. The published circle must be `(c, ρ0)` within slack (SR6).
The wall's side, the convexity and the side of the rim the plane face lies on
read as R2's: the held wall lies on one side of the plane, the edge's `Convex`
flag must agree, and the plane face lies outside the rim exactly when the rim's
convexity equals "the sphere's material is outside it" (`Reversed`).

**Section.** Take the meridian half-plane `(ρ, z)` about the axis through `S`
along `n`, with the plane at `z = 0`, the rim's point `K = (ρ0, 0)` and the
sphere the circle of radius `R_s` about `(0, zs)`. This is R3's line-circle
cross-section (§4.3): the plane's face runs from `K` along `±ρ`, toward the
plane face, and the wall along the circle's tangent at `K` toward the wall's
side. The same solve gives the fillet ball's centre `(ρc, zc)`, its tangent
points `T_A = (ρA, 0)` and `T_B = (ρB, zB)`, and the arc's rows. A chamfer's
setbacks are `dA` along the plane and the arc length `dB` along the meridian.
Revolving the section about the axis gives the blend: the `Torus` with
`Center = c + zc·n`, `Axis = n`, `Major = ρc` and `Minor = r` (`Reversed` for
a concave rim), or the `Cone` through the circles `(ρA, 0)` and `(ρB, zB)`,
whose `Origin` is the narrower circle.

**Size (SR3).** The rim refuses when no ball of radius `r` touches both faces,
when the plane's setback circle reaches the axis (`ρA ≤ τ + slack`), when
the sphere's setback runs past the pole (`ρB ≤ τ + slack`), and when the
tangent point or the arc is degenerate (R3's tests).

**Construction.** As R2 (§5). The plane's rim loop gives way to the polygon on
the circle `ρA`, and the wall is cut by the plane `z = zB`, which meets the
sphere in exactly the setback circle. A sphere's held facets carry vertices
inside the face. Such a vertex is the tessellation of a doubly curved surface,
not a feature of the face, so only the wall's boundary loops must clear the
band (§6.2). The cut removes an inner vertex with its facets or keeps it, by the
side of the cut plane its held position lies on. A kept inner vertex `v` whose
held height clears the cut plane by at most `β(v) + slack` may have its true
point in the removed zone, which is no longer boundary. Its bound becomes the
larger of `β(v)` and its distance to the blend face, plus slack: that distance
is the meridian distance from `v`'s image to the torus arc or the cone's
segment, and a point of the blend face is a true boundary point. The largest
such bound joins `ε` of §6.3. Where the band is thin against the wall's held
vertex bounds, the clipped facets can fold; the rebuilt mesh then fails its
audit (SR7).

**Exact areas.** The plane loses the annulus `π·|ρA² − ρ0²|` and the sphere the
zone `2π·R_s·|zB|` (Archimedes). The torus patch has area
`2π·r·(ρc·φ + r·∫a_ρ dα)` by Pappus, with `a(α)` the unit vector from the ball's
centre along the arc and `a_ρ` its radial part. The cone's frustum has
`π·(ρA + ρB)·|T_A T_B|`.

### 4.7 A mitre run of R1 fillets

Straight R1 fillet edges of one radius that meet end to end on one base face `A`, at corners where `A` and the two
walls are mutually perpendicular, are blended as one run (`internal/retainedblend/line_mitre.go`). The edges form one
open chain or one closed loop, no more than two at a corner, every edge convex, and the walls are distinct.

At a joint `C` between the edges with walls `B_k` and `B_{k+1}`, the two cylinders meet in the plane through `C` with
normal `n(B_{k+1}) − n(B_k)`, which bisects the walls. Each edge's rows end on that plane (`setbackPoint`), so the two
strips share the vertices of their rows there and the strips' end triangles coincide. No end face is trimmed at a
joint. The vertical edge under `C` loses its top part, and the two walls meet at the strips' feet.

The base face's boundary gives way to a polygon of the rows' base points at the joints and at any free end. A closed
run replaces the whole outer loop; an open run replaces the stretch it covers. A free end trims its end face as a
single edge's does (§5), and an end face is trimmed by one end only. Each strip is its own blend group. The removed
and added areas are the sums of the single-edge terms, each with its mitred ends' end planes replaced by the mitre
plane, and the volume change is bounded from those sums.

Refusals (SR2): three edges at a corner, edges not on one base face, a corner whose faces are not mutually
perpendicular, both free ends of an open run on one face, and a boundary of the base face or of a wall within the
strips' reach.

### 4.8 Corners of three perpendicular faces

Straight R1 fillet edges of one radius whose ends lie at corners of three mutually perpendicular retained planes are
blended together (`internal/retainedblend/corner_fillet.go`), however many edges the selection holds at each corner.
Every edge is convex and at least `2r` long.

Each corner is read from its three faces, with outward normals `n_0`, `n_1`, `n_2`; the corner's edge `k` runs along
`u_k = −n_k`. The strip of an edge ends on a plane that depends on how many selected edges reach the corner:

| Selected at the corner | End plane of each strip | Face `k` (which holds edges `i` and `j`) gets, in place of the corner |
|---|---|---|
| one, edge `k` | the plane of the end face, as a single edge's | the end row of edge `k`, an arc of centre `C + r·u_i + r·u_j` |
| two, `a` and `b` | the plane through the corner with normal `n_a − n_b` (§4.7) | `C + r·u_i + r·u_j` when `i` and `j` are both selected, else the point `r` along the unselected edge |
| three | the plane square to the edge `r` from the corner | `C + r·u_i + r·u_j` |

At a corner with three selected edges a sphere of radius `r` and centre `C + r·(u_0 + u_1 + u_2)` fills the gap. Its
patch is the octant between the three strips' end rows, gridded so that its sides are those rows' vertices. The
exact areas are those of the strips cut at their end planes, plus `3r²` removed from the three faces and `πr²/2` added
by the patch for every such corner. The chord tolerance is an eighth of a single edge's, so the many chord deficits stay
inside what `Verify` accepts for a box with all twelve edges rounded.

Refusals (SR2): a selected edge no longer than `2r`, a face edge at a corner no longer than `r`, a corner whose faces
are not mutually perpendicular retained planes, and a rebuilt face whose outline is not a simple polygon.

### 4.9 R7, the saddle of a cross hole (Chamfer)

R7 is the closed mouth a perpendicular bore leaves in a turned shaft. It is
built by `internal/retainedblend/saddle.go` (recognition) and `saddle_tool.go`
(tool and cut), not by §5's rewrite: the chamfer face is a ruled band between
two space curves, which is no `Plane`, `Cone` or `Cylinder`, so the result holds
it as an untagged `Faceted` face whose chords carry a proven bound, as
`agent-docs/cross-cylinder-bore-chamfer-design.md`'s band does. A coaxial cone
does not give equal setbacks: its setback along the bore wall varies by
`R − √(R² − r²)` around the mouth (0.46 mm for R = 10, r = 3).

**Recognition (tags only).** `looksLikeSaddle` admits a closed edge
(`Start == End`) between two faces with `RetainedTag`, `TagDeparture == 0` and
`tagIsRecord`, both `Cylinder` and of different radii: the larger is the shaft
(radius `R`), the smaller the bore (radius `r`). The edge's `Convex` flag must
be set and the ball's sides must be σ = (−1, +1) (§4.10), so the shaft is not
`Reversed` (material inside) and the bore is (material outside). Each axis is a unit
coordinate axis (one component ±1, two exactly 0), the two on different
coordinates, and the origins agree exactly on the third coordinate, so the axes
meet at right angles at a point `T` read from the tags. `2r² < R²` holds in
rationals. Any failure leaves the edge to `Unread`'s canal-surface reason (SR1).

**Pose.** A signed permutation and the translation `T` map the pair exactly
onto cross-cylinder-bore §"Source surface and band"'s pose with the bore axis
at `z = 0`: shaft axis local Z, bore axis local X, the third coordinate local
Y. The held seam loop is the boundary loop of the bore's facets that holds the
edge's start vertex. Every vertex of it must lie on the shaft's facets, within
its bound plus `64·ulp·scale` of both cylinders, and on one side of the shaft's
axis; that side becomes local `+X` (X and Y both flip for the far mouth). This
check only refuses.

**Size.** One setback `d` on both walls, `0 < d ≤ r/2`, converted exactly; an
asymmetric chamfer refuses. Several selected saddles are cut in one Boolean.

**Tool.** At each of `N = 1024` stations `θ_j = 2πj/N` the feet `O` (shaft)
and `B` (bore) are cross-cylinder-bore's, each `d` from the seam point `P` along
its unrolled cylinder. The tool section holds four vertices:

| Vertex | Local position |
|---|---|
| `O⁺` | `O + λ(O − B)`, `λ = 1/4`: the band run past the shaft wall |
| `A` | `O⁺` moved radially from the shaft's axis to `R + r`, same height |
| `K` | `(√(R² − r²cos²θ) + r, (r/2)cos θ, (r/2)sin θ)`: in the bore's extension outside the shaft |
| `B⁺` | `B − λ(O − B)`: the band run into the bore |

Quads join consecutive stations over the sides `O⁺A`, `AK`, `KB⁺` and the band
`B⁺O⁺`; the ring is wound outward by its signed volume. Its band is group
`chamfer(k)`, every other side one shared group.

**Proofs (each failure refuses `ErrUnsupported`).**

1. Band cells: `crossbore.ProveToolBand` runs `CellExactProof` on the `N/4 + 1`
   quarter-turn cells widened by `2π/N` (the outer foot outside the bore, the
   bore foot inside the shaft, polar progress, no fold), and mirrors them over
   the whole turn as cross-cylinder-bore does; the local pose is exactly
   symmetric by construction.
2. Chord bound `ε`: a point of a held band triangle pairs with the exact
   extended band `F(θ, t) = B⁺(θ) + t(O⁺ − B⁺)(θ)` at the same parameters; on
   either triangle of a quad the difference is `h·u(1 − u)` times the spread of
   the two end curves' θ-derivatives over the cell (`|[B⁺′] − [O⁺′]|` and the
   enclosures' widths), so at most `h/4` times it, `h = 2π/N`. Each held `O⁺`,
   `B⁺` is compared with a tight enclosure at its own exact angle, mapped to the
   world by the exact pose; the largest gap joins `ε`.
3. Band area: midpoint cells over exactly `[θ_k, θ_k+1]` enclose `|u + t(v − u)|`
   (`u = B⁺′ × D`, `v = O⁺′ × D`, `D = O⁺ − B⁺`) between `|u + v|/2` and
   `(|u| + |v|)/2`; four quarters give the band's exact area interval.
4. Outside: on the held vertices in exact rationals, every `O⁺A` and `AK` quad
   lies in one half-space `n·p ≥ (R + μ)|n|`, `n` horizontal, and every `K`
   and `B⁺` lies inside the bore shrunk to `r − μ'`, so the `KB⁺` quads do
   (the disc is convex). `μ` and `μ'` are the wall's largest held vertex bound
   plus `ε` plus slack.
5. Clearance: no facet of another face, and no segment of another boundary loop
   of either wall, grown by its held bounds plus slack, meets the tool's box.
   Inside the box the solid is then the region inside the shaft and outside the
   bore.
6. Each ring passes the exact crossing audit (`loftmesh.LoftCrossingAuditBounded`)
   and the rings' boxes are apart.
7. The Cut passes the mesh Boolean's own audits; a shifted retry refuses.

**Claim S.** Let `S` be the true solid and `W*` its exact chamfer wedge, the
part of `S` on the seam's side of the exact band. Let `T*` be the held tool
with its band replaced by the exact extended band. By proofs 4 and 5, `T*` meets
`S` only in `W*`: its sides lie outside `S`, the exact band runs past `O` outside
the shaft (`B` is strictly inside it) and past `B` inside the bore (`O` is
outside the bore and `B⁺` inside it). So `S − T* = S − W*`, the chamfered solid.
The held tool lies within `ε` of `T*`. The tool states vertex bound `ε` on `O⁺`
and `B⁺` and 0 on `A` and `K`, `VolSymDiff = ε·(A_held + A_exact)` over the
band (Claim K's argument), and `AreaSlack` the gap between the band's held area
and its exact interval plus a seal `2ε` times the length of both band ends. The
Boolean composes the result's bounds from these and the body's own.

**Chain allowance.** The Cut meets only the two walls and the bands, whose
facets carry the walls' held bounds and `ε`. The chain-depth gate
(`agent-docs/faceted-vertex-bounds-design.md` §5) would refuse them past the
pair's chord tolerance; it only compares two proven numbers, so the cut runs
with the chain tolerance raised to the largest of those bounds, as §5's rewrite
publishes the walls' bounds on its cut vertices.

**Result.** The walls keep their `Cylinder` tags; each band is a `Faceted` face
with role `chamfer(k)`. On the R = 10, r = 3 shaft the two-mouth result holds
about 20,000 facets and `Tessellate(…, VerifyAll)` verifies its boundary and
volume. `Verify` reads its area beyond the relative tolerance: the band rims hold
about `2N` segments, and `proofbound.ChainLengthBound` charges `2δ` per segment.

**Reach.** Built and checked: r/R = 0.3 (d = 0.5, 1.5) and 0.7 (d = 0.5) on a
Revolve shaft, and r/R = 0.3 (d = 1.5) on an Extrude shaft under each
assignment of the two axes to world coordinates and under a translation. The cell proofs also pass at r/R = 0.75;
recognition stops at `1/√2`, where the opening stops being one graph over the
shaft's angle.

### 4.10 R7 fillet, the rolling ball in a saddle

R7's saddle takes a `Fillet` where the ball lies outside the bore and either
inside the larger cylinder, the convex mouth a cross hole leaves in a shaft, or
outside it: the convex seam where two bores cross in a block, and the concave
root of a boss or pin on a rod. It is built by
`internal/retainedblend/saddle_fillet.go` (the rewrite), `internal/saddlecanal`
(the exact fillet and its cell proofs) and `saddle.go` (recognition), as a
rewrite of the held mesh near the mouth in the manner of §5. A Cut by a tool
would leave a crease where the true fillet meets each wall tangentially (§12).

**Recognition (tags only).** `saddleTags` reads the edge as §4.9 does, and
takes the larger cylinder (radius `R`) and the bore (radius `r`) by their radii.
Each wall gets the side σ of the rolling ball: −1 inside its cylinder, +1
outside. The ball lies in the material at a convex edge and in the void at a
concave one, and a face's material lies inside its cylinder unless the face is
`Reversed`. `saddleKey.filletBuilt` admits three patterns:

| σ | Edge | Faces | Example | The fillet |
|---|---|---|---|---|
| (−1, +1) | convex | the larger not `Reversed`, the bore `Reversed` | a cross hole's mouth in a shaft | removes material |
| (+1, +1) | convex | both `Reversed` | two bores crossing in a block | removes material |
| (+1, +1) | concave | neither `Reversed` | a boss on a rod, a pin joined through a shaft | adds material |

σ alone does not identify the first row: a concave pin root inside a tube's
bore has both `Reversed` flags and the `Convex` flag flipped from a mouth's, so
it reads σ = (−1, +1), and a blend there adds material. Its pipe is the
mouth's, but no fixture checks its walls, bands and winding, so it stays SR1
(B8), as does every pattern with the ball inside the bore. §4.9's chamfer reads only the first row
(`saddleKey.built`); a unit test checks that its rule admits exactly the edges
it admitted before σ existed, and another checks the fillet's three rows over
every pair of radii, `Reversed` flags and `Convex` flag. The pose and the seam
are read as in §4.9.

**Size (SR3).** With `R' = R + σ1·ρ` and `r' = r + ρ`, the ball's centre lies
on both offset cylinders, and they meet in one closed loop per mouth exactly
when `r' < R'`, decided in rationals. Inside the larger cylinder that is
`2ρ < R − r`; otherwise the call refuses `ErrDegenerate` "no ball of that radius
touches both walls of the saddle". For R = 10 and r = 3 the bound is ρ < 3.5.
Outside it `r' < R'` holds for every ball, since `r < R`; the reach ends at the
cell proofs, the window and the clearance.

**Fillet.** In the local pose (larger cylinder `x² + y² = R²` about z, bore
`y² + z² = r²` about x, mouth at x > 0) the centre curve is
`c(θ) = (√(R'² − r'²cos²θ), r'·cos θ, r'·sin θ)`. The ball touches the larger
wall along `a1 = −σ1·(c_x, c_y, 0)/R'` and the bore along
`a2 = −(0, cos θ, sin θ)`. Both are normal to `c′`, and the arc between them has
angle φ with `cos φ = σ1·r'·cos²θ / R'`. Inside the larger cylinder φ is
widest at θ = 0, `arccos(−r'/R')`; outside it at θ = ±π/2, π/2. The tangency
curves are `T1 = c + ρ·a1` on the larger wall, the curve
`(R'·y/|p_xy|)² + z² = r'²`, and `T2 = c + ρ·a2` on the bore, the curve
`x² + r'²y²/|p_yz|² = R'²`, which crosses each generator of the bore once at
the bore angle θ. T2 lies nearer the larger cylinder's axis than the seam
inside it and farther outside it. The fillet is the pipe
`F(θ, t) = c(θ) + ρ·w/|w|`. The direction `w` blends `a1` into the bisector
`b = (a1 + a2)/|a1 + a2|` over `t ∈ [0, 1/2]` and `b` into `a2` over
`[1/2, 1]`, each half as `(1 − τ)·from + τ·to`. Splitting at the bisector keeps
the parameter's angular speed within a factor 1.4 of uniform, where the single
blend from `a1` to `a2` varies by a factor 4.

**Stations and rows.** The strip has `N = 256` stations `θ_k = 2πk/N` and the
rows `t_j` that split each half of the widest arc φ* into equal angles no
larger than `2·arccos(1 − 2⁻¹⁰)`. The sagitta over that angle is `τ = 2⁻¹⁰·ρ`.
The rows are shared by every station: at R = 10, r = 3, ρ = 0.5 inside the
shaft there are 24 rows and the strip holds 12,288 facets per mouth; outside
the larger cylinder φ* = π/2 gives 18 rows.

**Walls.** The larger wall is cut out of a window `W`, a 64-gon in the (y, z)
projection. Its corners lie on `(R'/R)²y² + z² = s²` with `s·cos(π/64) =
r'·(1 + 1/32)`, so `W` holds T1 with a margin of `r'/32`. T1's ellipse holds the
seam's circle `y² + z² = r²` on either side: its semi-axes are `r' > r` and
`r'·R/R' > r`. Each side of `W` is a plane parallel to the bore's axis, so it
cuts every flat held facet of the larger wall along a straight segment: the clip
is exact. The part of a facet outside `W` runs along the facet's boundary
outside `W` and, from each point where that boundary enters `W`, clockwise along
`W`'s boundary through the corners inside the facet to the next crossing, which
must be an exit. Ear clipping in the projection triangulates each part. The cut
ring is the chain of those arcs around `W`, not a sort by angle. Two crossings
within `64·ulp·scale` of each other, across a held sliver, are one vertex. The
bore is cut by a plane `x = X_w` and keeps the part away from the seam: inside
the larger cylinder `X_w = √(R'² − r'²) − r'/32`, below T2's least x, keeping
`x < X_w`; outside it `X_w = R' + r'/32`, past T2's largest x, keeping
`x > X_w`. A held vertex within 10⁻⁶ mm of a cut, or a window corner as near a
held edge, turns the window (nine offsets of its corners) or moves the plane
(four places), else SR5. A cut along a curve of T1's own family would miss
facets: the larger wall's held facets are strips from the seam to the wall's
ends (168 strips 80 mm long on the R = 10 Revolve), T1 runs almost along the
generators at its two ends, and such a curve crosses a strip between the
strip's vertices.

**Bands.** On each wall a zipper of triangles joins the held cut ring to the
ring of exact tangency points at the stations: on the larger wall both rings
are ordered by the angle of `((R'/R)·y, z)`, on the bore by the angle about its
axis. T1 sits at angle θ_k in the first order and T2 in the second. Each band
facet is wound toward the solid's outward side of its wall, away from the
wall's axis when the face is not `Reversed` and toward it when it is, and then
tested exactly, in rationals on the held floats (`bandFacing`). On the larger
wall its normal's component along the bore's axis (+x) must have the outward
side's sign, + for a face that is not `Reversed`, so its (y, z) projection keeps
one orientation. On the bore its normal's component along `(0, y_i, z_i)` at
each vertex must have that sign, so its radial projection onto the bore keeps
one orientation over the facet's angular span. With both rings running once
around, either test embeds the band. A zero-area facet fails the test and
refuses. A band facet lies within `d + R·Δ²/8` of its wall, `d` the largest
distance of its vertices from the wall (a cut vertex's bound, or the strip's
sample error for a tangency point) and Δ the spread of its vertices' angles
about the wall's axis widened by `2d/(R − d)`; the band's bound is the largest
over its facets.

**Strip.** Consecutive stations and rows join in quads of two triangles
`(k, j), (k + 1, j), (k + 1, j + 1)` and `(k, j), (k + 1, j + 1), (k, j + 1)`.
Their local normal runs along `F_θ × F_t`, whose component along `u` has the
sign of proof 1's Jacobian: + inside the larger cylinder and − outside it. The
solid's outward side is `+u` at a convex edge (the ball in the material) and
`−u` at a concave one. The quads are swapped when the product of the
Jacobian's sign, the edge's (+ convex, − concave) and the pose's hand is
negative. Rows 0 and m are the T1 and T2 rings the bands share. The fillet is
an untagged `Faceted` face with role `fillet(k)`, as §4.9's band is.

**Proofs (each failure refuses `ErrUnsupported`).**

1. Cells (`saddlecanal.Prove`): over each station cell and row cell, split in
   two along t, second-order interval jets of `F` in outward-rounded float
   arithmetic give mean-value enclosures about the cell's midpoint, from sine
   and cosine enclosures of the exact turns k/N and (2k + 1)/(2N). Three tests
   must hold:
   - `s_J·det[F_θ, F_t, u] > 0` with `s_J = −σ1` (`Canal.JacobianSign`), so the
     pipe is regular and wound as for a small ball: its envelope is the surface
     the ball sweeps. As ρ → 0 the determinant tends to
     `ρ·det[c′, u_t, u] = −ρ·ω·(c′·ê)`, ê the unit axis about which `u` turns
     from `a1` to `a2` at speed ω > 0. φ stays inside (0, π) at every θ, so
     `c′·ê` keeps one sign around the loop, and θ = 0 shows it: `c′ = r'·ẑ`,
     and the arc turns about −ẑ inside the larger cylinder (`a1` in the first
     quadrant of the (x, y) plane, `a2 = −ŷ`) and about +ẑ outside it (`a1` in
     the third). A cell of the other sign lies past a fold.
   - `∂Φ/∂θ > 0`, with Φ the angle of `(F_y, F_z)` about the bore's axis, so
     every row runs once around the bore. Row 0 is the ellipse
     `(r'·cos θ·R/R', r'·sin θ)` in that projection and row m the circle of
     radius r, both run counter-clockwise for either σ1.
   - `∂(Φ, G)/∂(θ, t) > 0`, with `G = σ1·(|F_xy| − R) − (|F_yz| − r)` the
     distance from the larger wall on the ball's side less the distance from
     the bore. G is negative on T1 (on the larger wall, outside the bore) and
     positive on T2 (on the bore, off the larger wall on the ball's side) for
     either σ1. At a fixed Φ, `G` then grows with `t`, so `(Φ, G)` determines
     `(θ, t)` and the strip is embedded. The radius about the bore's axis
     cannot replace `G`: its t-derivative vanishes on T2, where the fillet
     touches the bore.

   A cell too wide to decide a test splits up to 8 × 16. At R = 10, r = 3,
   ρ = 3.4 and 3.2 inside the shaft, where `r' < R'` holds, the pipe folds near
   θ = 0 and the first test refuses: "the saddle fillet's cell at station 0
   folds: the pipe is not regular there". Dropping the sign `s_J` refuses every
   outside fillet with that text; dropping σ1 from `G` refuses them "is not
   embedded".
2. Chord ε: a point of a held strip triangle pairs with `F` at the same
   parameters. Linear interpolation on a triangle of a cell of widths h and k
   errs by at most `|F_θθ|·h²/8 + |F_θt|·h·k/4 + |F_tt|·k²/8` per coordinate,
   with the second partials enclosed over the cell. Each held vertex is compared
   with the point enclosure at its own station and row. ε is the largest bound
   plus that sample error, both read as 3D distances.
3. Areas: the fillet's, by the cells' mean-value integrals. The larger wall
   loses `A(r', R') − A(r, R)` with `A(a, b) = ∫₀^{2π} R·a²·sin²θ /
   (b·√(1 − (a/b)²cos²θ)) dθ`, the area of the ellipse `(b/R)²y² + z² ≤ a²` of
   the (y, z) projection lifted onto the wall. The bore loses
   `∫₀^{2π} r·|√(R² − r²cos²θ) − √(R'² − r'²cos²θ)| dθ`; outside the larger
   cylinder `R'² − r'²cos²θ − (R² − r²cos²θ) ≥ 2ρ·(R − r) > 0`. Each integral is
   enclosed station cell by station cell.
4. The band facet tests above (`bandFacing`), each exact.
5. Clearance: `checkMouthClear` refuses when a facet of a face other than the
   two walls, or a segment of another boundary loop of either wall, comes
   within its bound plus slack of the box of the window, the bore's cut-off
   part from the seam to `X_w`, and the held strip's vertices grown by ε:
   "another face of the body comes within the saddle fillet's reach". The box
   holds the region the fillet changes on either side of the seam, so a feature
   in the void beside a boss root refuses as one in the material beside a
   mouth does. Inside that box the solid's boundary is the two walls alone.
6. `checkStripClear` with both bands as taken ranges, and the strip's facets
   against each other.
7. The held audit of the rebuilt mesh.

**Claim F.** Let `H` be the held solid with `VolSymDiff = v`, `H'` the rewritten
one and `S'` the true filleted solid. Outside the box of proof 5, `H'` equals `H`
and `S'` equals `S`: proof 5 keeps every other feature out of it. Inside it `S'`
is bounded by the larger wall over `W` less T1, the fillet, and the bore from T2
to `X_w`, with the material on the side the faces' `Reversed` flags state. `H'`
is bounded there by the larger wall's band, the strip and the bore band, each
within ε of its exact surface, where ε is the largest of the strip's and the two
bands' bounds and every new vertex states it. So `VolSymDiff` grows by
`ε·(A_strip + A_fillet + A_bands + A_bands*)`, the held and exact areas, plus
rounding, whether the fillet removes material or adds it. `A_bands*` is bounded
on the larger wall by the cut ring's projected area less T1's ellipse
`π·r'²·R/R'`, times `R/√(R² − Y²)`, Y the ring's largest `|y|`, and on the bore
by `2πr` times the gap from the cut to T2's farthest x: `R' − X_w` inside the
larger cylinder, `X_w − √(R'² − r'²)` outside it. `AreaSlack` grows as §6.3
states, with the exact change the fillet's area less the two walls' (proof 3),
read at the end of its interval farther from the held change.

**Loose bounds at large ρ.** The fillet's area enclosure sums each cell's
mean-value remainder, which grows with the cell's second partials, and the
first split stops at the coarsest cell that passes the three tests. Every cell
splits in two along t first: a row cell is wider in t than a station cell in
θ, so the t term of the remainder dominates, and halving it quarters that term
for about a third more proof time. The enclosure reads [64.4, 66.9] mm² at
r = 3, ρ = 1.5 inside the R = 10 shaft and [180.6, 205.4] at ρ = 3.0, where
`AreaSlack` grows by about 43 mm² over two mouths, and [45.4, 47.1] for a boss
with R = 15, r = 5, ρ = 1. ε grows with ρ too, so the volume bound grows by about
135 mm³ at r = 7, ρ = 1.36. Each bound is proven; only its tightness suffers,
and splitting every cell finer would buy it back at a multiple of the proof's
time.

**Result.** The walls keep their `Cylinder` tags; each fillet is a `Faceted`
face. On the R = 10, r = 3 shaft the two-mouth result holds about 30,000 facets
at ρ = 0.5, and `Tessellate(…, VerifyAll)` verifies its boundary and volume.
The block's and the rod's results below read Sound under `Verify`.

**Reach.** Built and checked inside the larger cylinder: a Revolve shaft with
r/R = 0.3 (ρ = 0.5, 1.5) and 0.7 (ρ = 0.5), a `Hole` through the shaft's wall
(ρ = 1.5), and an Extrude shaft (ρ = 1.5) under each assignment of the two axes
to world coordinates and under a translation. Outside it: a block 80 × 60 × 20
with a radius-5 `Hole` down through it crossed by a radius-3 blind `Hole`
(ρ = 0.5, both mouths), a radius-5 `Boss` on a radius-15 rod (ρ = 1), and a
radius-3 pin joined through the R = 10 Revolve shaft (ρ = 1.5, both roots, a
left-handed pose). The cell proofs also pass at r/R = 0.8 and at r/R = 0.3 with
ρ = 3.0 inside, where ε grows to 0.022 mm, and outside at R = 5, r = 3 up to
ρ = 5 and at R = 10, r = 7, ρ = 10; at R = 5, r = 3, ρ = 20 a cell "is not
embedded". Near the size limit inside the shaft the route refuses before
`2ρ = R − r`: at r = 3 the pipe folds at ρ = 3.2; at r = 7, ρ = 1.4 the window
would reach past the side of the shaft (`s ≥ R'`, SR5). A boss whose top face
lies inside the bore's cut refuses at proof 5. The volume per mouth, from the
tube-coordinate integral of `apitest/retained_saddle_fillet_test.go`, is
1.635978 mm³ removed at ρ = 0.5, 7.317201 at ρ = 1.0 and 18.593228 at ρ = 1.5
for the R = 10, r = 3 mouth; 0.540755 removed for the block (R = 5, r = 3,
ρ = 0.5); 4.701377 added for the rod (R = 15, r = 5, ρ = 1); and 6.619077
added for the pin root (R = 10, r = 3, ρ = 1.5). A rolling-ball grid (a point is in the region
when it lies on the ball's side and farther than ρ from that side eroded by ρ)
agrees within 0.5 % for the mouth and 0.2 % for the outside patterns.

## 5. Construction

The rewrite touches the faces `A` and `B` of the edge and, for R1, the two end
faces `G₀` and `G₁`. Everything else in the held mesh is copied unchanged.

1. **Region.** Let `K` be the closed region the blend changes: for R1 the
   triangular prism of the corner between the two setback lines, cut by the
   planes of `G₀` and `G₁`; for R2 the annular wedge between the rim, the two
   setback circles and the blend torus or cone. §6's gates prove that nothing
   but `A`, `B`, and the end faces `G` meets `K`.
2. **Rewrite the faces.** The plane face's facets are replaced. Its boundary
   loops other than the rim are kept with their vertices, the rim loop gives way
   to a polygon of `N` vertices on the setback circle (R2) or a cut line (R1), and
   the region is triangulated again by ear clipping over those loops, with one
   bridge from each hole to the outer loop. The ear clipping uses every loop
   vertex, collinear ones included, so the face keeps the vertices its
   neighbours share. A face with a vertex off its boundary loops refuses
   (SR5), except a plane face's vertex exactly on its plane (§2), and the straight edge's own two faces' vertex exactly on that face's retained plane: the rewrite triangulates those faces again from their loops, so the point a mesh adds to fan a face is dropped. The wall's facets are cut by the setback plane: a facet wholly beyond
   it stays, one wholly before it goes, and a crossing facet is clipped, with one
   new vertex per cut mesh edge shared by the facets on either side. A new wall
   vertex lies on a held facet edge, so it takes that edge's larger vertex bound,
   plus `τ` and the rounding of the cut.
3. **Strip.** The blend face is a grid of rings. The first ring is the plane
   face's new polygon and the last is the wall's cut vertices, sorted by angle
   about the axis. A fillet has `m − 1` rings between them on the exact torus,
   with `m` the smallest count whose arc sagitta `r·(1 − cos(Δα/2))` is at most
   `τ = 2⁻¹⁰·r` (not below `64·2⁻⁵²·scale`). A chamfer has none. Neighbouring
   rings join by walking both in angle and advancing the ring whose next vertex
   comes first, which gives the band of triangles whatever the two rings' vertex
   counts. Each triangle is wound to face the blend's analytic outward normal.
4. **Ends (R1).** At each end vertex `V`, the end face `G` meets the blend
   cylinder in an arc: a circle arc when `G` is normal to `d̂`, an `Ellipse3`
   arc otherwise, running from `P_A` (the setback line of `A` on `A ∩ G`)
   to `P_B`. For a chamfer it is the segment `P_A P_B`. The region of `G`
   inside the polygon `V, P_A, arc, P_B` is removed from `G`'s facets by
   the same ear clipping; the arc enters as the chords of the strip's rows.
5. **Build.** The new vertex and triangle lists, the new groups and the new
   bounds go through `facetedrecord.Build`, which derives the topology, the
   measurements and the held audit the way `Placed` does.

### 5.0 R1, a straight edge

All four faces an R1 blend touches are planes, so each is rewritten the way the
plane face of R2 is: its boundary loops change, and the face is triangulated
again over the new loops. Let `e` run from `V₀` to `V₁`, with `d̂` its direction,
`G_i` the end face at `V_i`, `eA_i = A ∩ G_i` and `eB_i = B ∩ G_i`.

1. **Exact points.** `V_i` is the meeting of the planes `A`, `B` and `G_i`.
   The setback line `LA` on `A` is the line parallel to `d̂` at in-plane distance
   `tA` from `e`, and `LB` likewise. `P_A,i = LA ∩ G_i` and `P_B,i = LB ∩ G_i`.
   For a fillet, the arc rows `α_j = jφ/m` of the exact cylinder meet `G_i` at
   `q_j,i = p_j + s·d̂`, with `p_j` the point of the cross-section arc and `s`
   the offset along `d̂` that reaches `G_i`. Row 0 is `P_A` and row `m` is `P_B`;
   a chamfer has rows 0 and 1 only.
2. **Chains.** In `A`'s loop, `e` is the run of vertices from `V₀` to `V₁` that lie
   on the line of `e` within their bounds. `eA_i` is the run that leaves `V_i`
   on the line of `eA_i`. The setback point `P_A,i` falls between two vertices of
   that run, or between `V_i` and its first vertex. The run's vertices nearer
   `V_i` than `P_A,i` go; `P_A,i` takes their place.
3. **Loops.** `A`'s loop loses `V₀`, `V₁`, the run of `e` and the removed parts of
   `eA₀` and `eA₁`, and gains `P_A,₀` and `P_A,₁` joined by one straight edge.
   `B`'s loop changes the same way. `G_i`'s loop loses `V_i` and the removed parts
   of `eA_i` and `eB_i`, and gains `P_A,i`, the intermediate `q_j,i` and `P_B,i`,
   in the loop's own direction. Each face is then triangulated again.
4. **Strip.** Rows `j = 0 … m` each hold the two vertices `q_j,₀` and `q_j,₁`.
   Neighbouring rows join in one quadrilateral of two triangles, wound to face
   the blend's outward normal. The strip's boundary edges are the setback edges
   of `A` and `B` and the end curves of `G₀` and `G₁`, which are the vertices
   the four faces share with it.

**A face that is reflex at an end.** A concave edge can end where one of its two
faces wraps the corner, as a plate's top face does at the foot of a wall. That
face leaves the edge on the far side of its line: the setback line meets the
line of `A ∩ G` behind the corner, not along the chain. The face is not trimmed.
Its boundary runs on from the corner `V` to the setback point and along the
setback edge, so `V` stays in its loop. The end face `G` is convex at `V`, and the
blend's end cap lies in `G`'s plane beside it. `G` gains the cap: its loop keeps
`V`, takes the arc and the setback points next to it, and loses the removed part
of the other face's chain. A convex edge refuses a reflex face, and so does an end
where both faces are reflex. Which way a face leaves the edge comes from the
loop: a face lies to the left of its own boundary, so the way into `A` is
`n_A × δ`, with `δ` the direction the loop runs along the edge.

**An end face that is reflex at the corner.** A convex edge whose two faces both
trim at `V` can end on a `G` that is reflex there, as a boss's side edge ends on
the plate's top face, which wraps the boss's footprint. The corner region of the
cross-section, projected along `d̂` onto `G`'s plane, lies on the far side of
`G`'s two chains from `G`: before the blend it is inside the material, after it
is the floor of the removed region. `G` takes it as the blend's end cap. Its
loop is rebuilt as in step 3, which now encloses the corner region, and its
rebuilt facets are new surface: `checkStripClear` tests them, as it tests an
open chain's taken cap (retained-modify-reach §5.6), against every facet the
result keeps outside them, and refuses on any contact. The SR5 reach check
already keeps the rest of `G`'s boundary off the cap. Only this R1 route takes
a cap from a reflex `G`; the plane–cylinder, arc, cylinder–cylinder and mitre
routes refuse a reflex `G` under a convex edge (SR5).

**A wall that extends past the corner (R3).** A convex plane–cylinder edge can
end where the cylinder runs on past the end plane, as a keyway's top edge does
in a turned shaft: the shaft's surface continues beyond the keyway's end wall
`G`. The wall's chain leaves `V` along the end circle away from the wall's side
of the edge, `G` is convex at `V`, and the plane `A` trims there. Only the
band of the wall between the two end planes goes (§4.3), so the wall keeps `V`
and its whole chain. Beyond `G` the corner region stays material, so `G` takes
the corner region of the cross-section as the blend's end cap: its loop loses
the removed part of `A`'s chain and runs from `P_A` along the blend's rows to
the wall's setback point in `G`'s plane, then along the wall's cut in that
plane back to `V`. Its rebuilt facets are new surface and are cleared by
`checkStripClear` as the R1 cap is. The cap's loop of rows, the wall's run and
`A ∩ G` must be a simple polygon in `G`'s plane (`capSimple`), else SR5: no two
of its edges that share no vertex touch, no two that do run back along each
other, and it turns at `P_A` and at the band's corner the way it runs. A
fillet's cap closes in a cusp at `T_B`, where the ball touches the circle, and
the wall's facets lie inside the true circle by up to their bound. When the
ball's gap to the circle at the last row is under that, the row lies beyond
the facet the run is cut from and the loop crosses itself, as a fillet r = 0.3
on the R = 10 shaft can. The route refuses a wall that extends beside a
concave edge, or beside a convex edge whose plane is reflex at the
same end (SR5). Which way the chain leaves `V` comes from its first vertex,
which must lie farther along the circle's tangent at `V` than its bound plus
the slack, else SR5; `A`'s chain takes its sense from its last vertex the same
way.

**`G`'s turn at `V`.** On the R1 route the turn is read from the tag lines
`A ∩ G` and `B ∩ G`, not from held vertices: each line's direction is the cross
of two retained normals, and its sense comes from the chain's last vertex,
which must lie farther along the line than its bound plus the slack, else SR5.
The turn is the sine between the two directions about `n_G`; within `minSine`
of zero it is undecided, SR5. A reflex `G` is therefore never read as convex,
which would skip the cap's clearance. Nothing places the last vertex of a
chain whose face extends past `V` far from `V`; where it lies within its bound
the sense is undecided and the edge refuses. R3 reads the turn the same way at
an end where its wall extends past `V`, from the line `A ∩ G` and the end
circle's tangent at `V`. The other routes, and R3 at an end where its wall
ends with the edge, read the turn from `G`'s loop vertices beside `V`.

**Exact areas.** The removed region of `A` is the quadrilateral
`V₀ V₁ P_A,₁ P_A,₀`, and `B`'s likewise. The removed region of `G_i` is the
projection along `d̂` of the corner region of the cross-section, area
`r²(tan(φ/2) − φ/2)`, so its area is that over `|d̂ · n_G|`. A chamfer removes
the triangle `V_i P_A,i P_B,i` from `G_i`. A `G_i` reflex at `V_i` gains the
same area instead, as does an R3 end face where the wall extends past `V_i`; on
R3 the corner region is the plane's line from `K` to `T_A`, the blend, and the
wall's circle back to `K` (`cylLine.cornerArea`). The cylinder patch has area
`r·ℓ_c·φ + r²·tan(φ/2)·μ·(a₀ + a₁)`, with `ℓ(x) = ℓ_c + μ·(x − c)` the length of
the generator through `x` between the two end planes, `c` the axis point and
`a₀`, `a₁` the unit vectors from `c` to the two tangent lines. The chamfer plane
is a trapezoid of area `½(|P_A,₀P_A,₁| + |P_B,₀P_B,₁|)·|T_B − T_A|`.

### 5.1 Groups and tags

| Face | Group fields |
|---|---|
| `A`, `B`, `G` | the old group, with `Surface`, `Retained`, `Reversed`, `CapRecord` and `NormalBound` unchanged. The face's patch is smaller; its plane or cylinder is not |
| blend | a new group: `Surface` the exact `Cylinder`, `Plane`, `Torus` or `Cone` of §4, `Retained = true`, `Reversed` as §4 states, `NormalBound = 0`, `Origins` the role the step mints (`fillet(k)` or `chamfer(k)`, `k` the edge's ordinal in the call), `Source` a new `SourcePatch` naming the modify producer and `k` (`Blend` true, no parent). The old groups keep their `Source` pointers |

The blend tag satisfies retention Claim R: the true blend patch lies on the
surface by definition of the blend. The face's normal bound follows retention
§3.3 from its own `δ_f`, which is the strip's sagitta and rounding.
A face whose `2·δ_f / R` reaches 1 reads `Faceted` (retention A4); that is
not a refusal.

### 5.2 Edges

The new edges are constructed, not discovered, so they publish their curve
without retention's falsifier: the two setback edges (`Line3` for R1,
`Circle3` for R2), the end arcs (`Arc3`, or `Ellipse3` for an oblique `G`), and
for a chamfer the end segments (`Line3`). Each is marked `RetainedCurve`.
Their vertices are the exact intersections of the setback lines with the end
planes or the end faces' own edges, so a vertex position bound is the rounding
of that computation.

## 6. Gates and bounds

### 6.1 Table SR — refusals

| SR | Call | Existence | Sentinel |
|---|---|---|---|
| **SR1** | an edge outside Table RR: a curve that is not `Line3` or `Circle3`, a face that is not `RetainedTag`, a face with `NormalBound != 0` or a denoted patch whose tag is not exactly the surface it denotes (for R6's sphere, its record's exact centre and radius, §4.5), or a `Convex` flag that disagrees with §4.1 | body exists; no admitted source for the selected edge | `ErrUnsupported` (reach SX9's text) |
| **SR2** | two selected edges share a vertex and are neither a mitre run (§4.7) nor corners of perpendicular faces (§4.8), or a selected edge continues tangentially into a neighbour | no transition built | `ErrUnsupported` |
| **SR3** | no blend of that size exists: `R − r ≤ 0` for an inside ball (§4.2), `φ` within 10⁻⁹ of 0 or π, a chamfer distance of zero or below, an R6 setback that reaches the axis or runs past the sphere's pole (§4.5), or an R7 fillet inside the larger cylinder with `2ρ ≥ R − r`, where no ball touches both walls (§4.10) | no regular blend | `ErrDegenerate` |
| **SR4** | the setback does not fit (§6.2) and the held bounds prove it does not | the blend consumes the face | `ErrDegenerate` |
| **SR5** | the setback might not fit and the held bounds cannot decide it, or a plane that crosses an R6 sphere below `minSine` (§4.5), or an end vertex of an R1 edge is not a three-face corner with a retained planar `G`, or a chain's sense or `G`'s turn at the vertex is undecided (§5.0), or `G` turns the wrong way: a concave edge needs a reflex corner of `G`, or a convex one where `A` or `B` is reflex at the vertex, and a convex edge outside the R1 route (plane–cylinder, arc, cylinder–cylinder, mitre run) needs a convex one; or both `A` and `B` turn reflex at the vertex, or a convex edge meets a reflex face other than an R3 wall that extends past the vertex beside a plane that trims there (§5.0), or another part of `G` comes within the blend's reach of the vertex, or a vertex of an R3 wall lies within its bound of an end plane the band is clipped by, or, unless it lies outside such a plane, of the band's generator (§4.3), or the end cap an R3 end face takes is not a simple polygon (§5.0), or another feature comes within an R7 chamfer's or fillet's reach, or an R7 fillet's cell, window, cut ring or band test fails (§4.9, §4.10) | body exists; the evaluator cannot prove its construction | `ErrUnsupported` |
| **SR6** | the held mesh disagrees with the exact vertex of its three tags by more than the vertex bound plus slack (§6.3) | the record and the mesh do not describe one solid | `ErrUnsupported` |
| **SR7** | a facet the rewrite keeps outside the new strip meets a strip facet it shares no vertex with (the strip clearance of retained-modify-reach §5.6, run before the build), or the rewritten mesh fails the held audit (watertight, shells nested with consistent signs; the audit tests no facet pair for intersection) | the result is invalid | `ErrDegenerate` when the audit proves a face inside out (modify S8), else `ErrUnsupported` |

**Naming the unread edge.** When no route recognizes the selection, the generic
fillet or chamfer refusal of a mesh Boolean result keeps its text and its
sentinel and appends `; no retained blend class reads selected edge[N]: <reason>`
(`Unread`, `internal/retainedblend/unread.go`). It names the first selected edge
that is in no class of Table RR, from the faces' tags and the edge's face count
only, never from held geometry. The reasons are a face with no retained surface
(faceted), a free-form surface, a face without a retained tag, a tag with a
stated departure or one not proven to be exactly its revolve record, two retained cylinders
whose axes are not parallel (the canal surface of SR1), a retained cone or
torus, and otherwise the two retained kinds the edge lies between. `Unread`
skips the saddles §4.9 reads, the patterns both blends build; the canal-surface
reason names the crossing rims outside R7. The fillet's σ = (+1, +1) saddles
(§4.10) reach `Unread` only in a selection that mixes them with an edge no
route reads, and it names them with the canal-surface reason too.

**Displaced planes.** A retained `Plane` tag may state `AxialDelta` δ, the
proven distance of the true plane from the tag along its normal. A route never
reads the tag as exact past δ. Each reader adds `4δ/minSine` per plane it reads
(`planeCharge`) to its slack, which the vertex-agreement test of SR6 and the
bounds of §6.3 both carry. A tangent chain charges every plane of the held
body, summed in the order of the held faces. A negative or non-finite δ makes
the face no retained plane, so the edge is outside Table RR (SR1). The charge
widens the published bound or the route refuses; no residual admits a
displaced plane.

A blend face built from displaced planes may lie off its own tag by the same
charge. A chamfer plane states it as its `AxialDelta`. A fillet cylinder, a
rim torus or cone and a corner sphere have no field for a positional offset, so
with a nonzero charge they are not published as retained tags and the face
stays faceted. With no charge every tag is published as built. The tangent
chain (retained-modify-reach-design.md §5.3) reads its tags as exact, so a
plane with any nonzero `AxialDelta` is no chain wall, as
`facedeparture.TagDeparts` reads it.

Gate order: base gates of modify §4 stage 1, then SR1, SR2, SR3, SR6, SR5, SR4,
then the construction, then SR7. R7 runs its own order (§4.9): recognition,
the size, the seam loop, then per saddle the band proofs, the outside and
clearance proofs and the crossing audit, then the Cut. The R7 fillet's order
(§4.10) is recognition, the size, the seam loop, then per mouth the cell
proofs, the window and the bore cut, the clearance, the wall cuts, the bands,
the strip, `checkStripClear`, and the build's audit.

### 6.2 Clearance

The strip must lie inside `A` and `B`. Each held boundary vertex is known to
within its bound `b_v`, and the held bound of a retained plane facet is the slab
bound of retention §11. The gate uses held data and published bounds and
nothing fitted.

- **R1.** For `A`, let `s(v)` be the signed in-plane distance of a boundary
  vertex `v` from the edge line, positive into `A`. Every boundary vertex of
  `A` other than the edge's two end vertices must satisfy
  `s(v) − b_v ≥ t + slack`. A boundary segment between two such vertices stays
  at `s ≥ t + slack`, so no boundary edge of `A` other than the two end edges
  enters the strip. The two end edges are `Line3` edges to the end faces and
  reach `s ≥ t` by the same inequality at their far vertices. The same holds
  for `B`. If some vertex has `s(v) + b_v < t` the strip provably leaves the
  face (SR4); if neither inequality holds, SR5.
- **R2.** On the plane, every boundary segment of every loop other than the rim
  must be at distance at least `R + σ·t + b + slack` from the axis, the way
  `σ` of §4.2 points. On the cylinder, every boundary vertex other than the
  rim's must lie at axial distance at least `t + b + slack` from the rim's
  plane.
- **R6.** The plane as R2's, out to `ρA`. On the sphere, every vertex of a
  boundary loop other than the rim's must clear the cut plane at `|zB|` by more
  than `b + τ + slack` on the wall's side. A vertex inside the face is no
  feature and is not gated; a kept one near the cut takes the bound of §4.5.

### 6.3 Bounds of the result

A ring vertex on the plane face lies at the exact setback circle, so it is an
exact function of the tags; every strip vertex takes the bound `τ` plus the
rounding of its computation, because the chords between ring vertices leave the
exact surface by at most `τ` and a vertex bound is the only place a facet can
state that. A wall cut vertex takes the larger bound of its edge's two ends, plus
`τ` and the rounding (§5 step 2). An old vertex keeps its bound.

**Claim K.** Let `H` be the held solid with `VolSymDiff = v`, `H'` the rewritten
held solid, and `S'` the true blended solid. Then
`vol(H' △ S') ≤ v + τ·area(blend) + ρ`, with `ρ` the rounding of the new
vertices times the area of the faces they touch.

*Proof.* Outside `K`, `H'` equals `H` and `S'` equals the true solid `S`,
because the gates of §6.2 show that no other face enters `K`. So the difference
outside `K` lies inside `H △ S`. Inside `K`, `S'` is the exact blended region and
`H'` is the polyhedron on the strip's chords; they differ by at most the sagitta
`τ` times the blend face's area, plus the rounding of the grid's vertices. ∎

**Area.** Write `Δ_held` for the held area of the new facets less that of the
old facets the rewrite replaced, and `Δ_true` for the exact area of the blend
face less the exact area of the annulus and the wall band it removes. Both
bodies have the same area elsewhere, so the held total differs from the true one
by the old difference plus `Δ_held − Δ_true`. The old difference was proven
only with `G + AreaSlack`, where `G` sums every face's geometric term (the
smaller of its largest facet bound times its perimeter and its facets'
perturbation allowance, `facetedrecord.FaceAreaGeom`). The rebuilt body
charges its own sum `G'`, each term computed afresh from the face's new facets,
and a term can be smaller than before: a rewrite drops high-bound rim vertices
and whole facets, and the arc that replaces a corner shortens a face's
boundary. Write `G_g` and `G'_g` for the sums over the faces whose first facet
comes from source group `g`, in the old and the rebuilt body; group indices
carry over, since a rewrite only appends groups. The new `AreaSlack` is the old
one plus `|Δ_held − Δ_true|`, the rounding of the two area sums
(`proofbound.FacetAreaTermSlop` per facet), and the shortfall
`Σ_g max(0, G_g − G'_g)` over the old body's groups. Then
`G' + AreaSlack' ≥ G + AreaSlack + |Δ_held − Δ_true|`, which bounds the new
difference whatever any face's new term is. `G_g` is summed rounding up and
`G'_g` rounding down, and each positive difference is rounded up, so the
charged shortfall covers the exact one. `G'` exists only once the body is
built, so the shortfall joins `AreaSlack` and every area bound after the build
(`facetedrecord.ChargeAreaSlack`); `AreaSlack` enters no other reading.

The rebuilt body charges a rewritten face's new term itself, so `AreaSlack`
carries only what the face's term lost. Charging the face's whole old term
instead charges the same allowance a second time, beside its new term, and a
blend of `n` edges that each rebuild one large face then charges that face's
term `n` more times. A face that is the only face of its group in both bodies
keeps its own area bound at or above its old term plus the old `AreaSlack`.

A face's own area bound is not derived from its old one: its term is computed
afresh from its new facets (`agent-docs/faceted-vertex-bounds-design.md`
§4.2), and the shortfall keeps only the body total. A new boundary chord of
length `c` on a face, standing for an exact curve of the face whose sagitta
against it is `s`, leaves a segment of area at most `c·s` between them: the
segment lies in the chord's `c × s` rectangle, since the arc spans less than a
half circle. Both forms of the face's term charge at least `β·c` for that
chord, `β` the smaller bound of its two end vertices: the perimeter form's
`δ_f` is at least `β` and its perimeter holds `c`, and the facet beside the
chord charges `δ(t)·(|u| + |v|) ≥ β·c` by the triangle inequality. So the face's
own term covers its share of the chord error wherever `s ≤ β`. The rim ring
meets that (its step comes from `ringCount` at `τ`, and each ring vertex has
bound `τ + slack`), and so do a straight edge's rows on the blend's circle
(step `2·acos(1 − τ/r)`, bound `τ + slack`). A straight edge's rows projected
along the edge onto an end face oblique to it do not: in the end plane `G` a
row chord's sagitta reaches `τ/|d·n_G|` against the same bound `τ + slack`.

**Volume.** The new `VolSymDiff` is the old one plus `ε·(A_strip + A_true)`, with
`ε` the largest vertex bound of the strip and `A_strip`, `A_true` the held and
exact areas of the blend face. Two surfaces within `ε` of each other enclose at
most `ε` times the larger area between them.

The `Verify` and `VerifyAll` results come from the faceted audit of the rebuilt
body, not from these terms.

## 7. Receiver and dispatch

Reach RX6's row gains a class and SX9 narrows with it.

| Receiver | `Fillet` / `Chamfer` |
|---|---|
| `facetedPayload` whose selected edges are all Table RR edges | this document's route, applied one edge at a time to the evolving mesh. Each step reads the previous step's result as an ordinary retained body; the selection resolves up front to `(face pair, curve)` records, not mesh indices |
| any other `facetedPayload` call | SX9, unchanged |

A `Chamfer` whose selected edges are all R7 saddles takes §4.9 instead: one Cut
by a tool with a ring per saddle. A `Fillet` whose selected edges are all
saddles of the R7 fillet row takes §4.10, one mouth at a time on the evolving
mesh.

The result is a `facetedPayload` with retained tags. A further `Fillet`,
`Chamfer` or Boolean takes it like any retained body, and a placement carries
its tags by retention §3.4. `Shell` stays SX9. An asymmetric chamfer takes the
reference-face option of modify §7: SX3 resolves one reference face per edge,
and each step finds it again on the evolving mesh by its retained tag
(`agent-docs/retained-modify-reach-design.md` §4). A missing or ambiguous one
is reach SX3.

## 8. Readers

A blend result is a retained body, so every retention §4 and §10.3 reader row
applies unchanged. Two readers see something new: `FaceCreatedBy` finds the
blend face by its role, and the edges between the blend and `A` or `B` are
tangent junctions, the kind a tangent-chain selection expands (reach SX2).

## 9. Tests and the red leg each must show

Tests assert on computed geometry. Volumes are compared with the result's
published bound, never with a pinned literal.

| Test | Asserts | Red leg |
|---|---|---|
| Hole mouth in a placed-tool plate (R2 fillet, convex) | blend face is a `Torus` with `Major = R + r`, `Minor = r`; removed volume equals `π·r·(2Rr + r²) − 2π·((R + r) − 4r/(3π))·(π·r²/4)` within the bound; both setback edges are `Circle3` | build the cone for a fillet: the surface kind is wrong |
| Boss root fillet (R2 concave) | the volume grows by the same expression with the tangent circles at `R + r`; the blend face has `Reversed = true` | use the convex sign: the blend removes material |
| Shaft end fillet and chamfer (R2) | `Major = R − r`; `R − r ≤ 0` refuses with `ErrDegenerate` | drop SR3: the torus has a negative major radius |
| Straight 90° edge (R1 fillet and chamfer) | `Cylinder`, axis along the edge, removed volume `L·r²(1 − π/4)`; the chamfer removes `L·d²/2` | |
| 45° edge (R1 fillet) | setback `r·tan(φ/2)`; removed area per unit length `r²(tan(φ/2) − φ/2)` | use `r` for the setback at `φ ≠ π/2`: the 45° test fails |
| Plane to cylinder edge (R3): a groove's top edge, fillet and chamfer | the fillet's centre and tangent points are those of the cross-section; the volume change equals `L` times the area of the corner region, and the surface changes by the cylinder gained less the plane strip, the wall band and a corner of each end face | take `κ` the other way: the ball sits on the wrong side of the wall |
| Plane to cylinder edge at a boss's foot | the plate's top face is reflex at each end; the fill adds `L` times the corner region's area, and `VerifyAll` passes | |
| Keyway top edges in a turned shaft and a hub (`apitest/faceted_retained_blend_keyway_test.go`): an R = 10 shaft 80 mm long with a keyway `|y| ≤ 3` above z = 6.5 from x = 20 to 60, one from x = 20 running off the shaft's end, one from x = 20.37 to 60, and the 20–60 keyway with a Ø0.6 hole along z at x = 70 whose rim has a vertex on the r = 0.5 fillet's setback line; a hub with a bore r = 5 and a keyway `|y| ≤ 2` from x = 10 to 30 | the shaft runs on past each end wall, which is convex at the corner and takes the cap. Fillet r = 0.5 of both edges removes `2·40·A`, with `A` = 0.117968684 mm² the corner region (its closed form agrees with a sampled polygon), the run-off keyway `2·60·A`, Fillet r = 0.3 of the 20.37 keyway `2·39.63·A(0.3)`, the holed shaft `2·40·A` as without the hole, and Chamfer d = 0.5 of one edge `40·A_c`; the hub's bore holds its material outside the cylinder, its ball lies in the hub and its region (agreeing with a sampled polygon) is the quadrilateral less the sector and less the bore's segment, removed over `2·20`; the area changes by the blend face less the wall's strip and the cylinder's band, plus a cap at each end inside the cylinder and less a corner at the shaft's end disc; a point 0.125 mm inside the corner region lies on each end wall and more than 0.1 mm off the disc; `VerifyAll` passes. A cavity crossing the cap refuses "the cap an end face takes meets …", a pin hole into the end wall within the blend's reach refuses SR5, the cross hole moved to x = 40, inside the band, refuses SR5 on the setback line, Fillet r = 0.3 of a keyway from x = 20.37 running off the shaft's end refuses SR5 "the blend's end cap folds …", and the top edge of a blind groove in a plate, whose plane is reflex at its ends, refuses SR5. Unit tests (`internal/retainedblend/line_rewrite_internal_test.go`, `cyl_rewrite_internal_test.go`) splice the rows, the wall's run and `V` into an end face's loop, refuse tag directions whose turn is undecided or reflex while the held vertices show a convex corner, refuse a plane's or wall's chain whose sense vertex lies within its bound of the corner, charge `A` against the strip and band with the sign of each end's kind (`cylRemovedArea`), and refuse a cap loop that crosses itself or runs straight on at the band's corner | clip the wall by angle alone: "a wall facet reaches across half the cylinder"; read every wall vertex against the generator: the x = 70 hole refuses; drop the caps from the strip-clear check: the cavity builds; drop `capSimple`: the r = 0.3 run-off keyway refuses "closes on a degenerate triangle" instead; drop its crossing or its turn test: the matching unit case returns no error; drop the wall's run from the splice: the end face fails to triangulate; drop the reach check: the pin hole builds; drop the reflex-plane check: the groove builds; read the turn from held vertices: the unit cases return no error; drop either sense bound: its unit case returns no error; charge a cap where the wall extends as a corner given up: the area unit case fails |
| Convex side edges of a boss at its foot (`apitest/faceted_retained_blend_boss_foot_test.go`): a 20×20×15 box boss on an 80×60×10 plate with a placed through hole; a 45° corner, its slant sketched so its tag states no departure (retention §3.5), on a boss sunk 1 mm into the plate because the shifted retry re-normalises a slanted frame and states a departure; a pillar between two plates. The square bosses stand on the plate and take the shifted retry | the plate's top face is reflex at each foot and takes the cap. Fillet r = 3 of the four sides removes `4·15·r²(1 − π/4)`, Chamfer d = 2 removes `4·15·d²/2`, the 45° corner `15·r²(tan(φ/2) − φ/2)` with `φ = 3π/4`, the pillar `15·r²(1 − π/4)` with a cap at each end; the area changes by the blend face less the two strips, less the corner each convex top end gives up, plus each cap a reflex end takes (top and foot cancel on the boss; the pillar gains two caps), and the bound's growth stays under twice the caps' area; `VerifyAll` passes on the box fillet. A cavity crossing the cap refuses "the cap an end face takes meets …", a hole within the blend's reach on the plate's top refuses SR5, and a convex edge whose face is reflex at the foot refuses SR5. A unit test (`internal/retainedblend/line_rewrite_internal_test.go`) reads `G`'s turn from tag directions whose sine is under `minSine` while the held vertices show a square corner: it refuses as undecided | build no cap: the boss refuses SR5; drop the cap's facets from the strip-clear check: the cavity builds; charge the cap as a corner `G` gives up: the bound's growth passes twice the caps; read the turn from `G`'s loop vertices beside `V`: the undecided unit case returns no error; drop the reflex-face check: the step's wall edge builds |
| Concave edge in an L section | the fill adds `L·r²(1 − π/4)`, the blend's normal points into the void, `VerifyAll` passes | build the convex blend: the normal points into the material |
| Concave edge at a wall's foot | the plate face is reflex at each foot corner; the fill adds `L·r²(1 − π/4)`, the wall's end faces grow by the caps, `VerifyAll` passes; two foot edges together and a chamfer build as well | keep `V` out of the extended end face's loop: the mesh fails its audit |
| Oblique end face, sketched at 45° about z and, separately, about y (no exact tool leans both ways) | the end arc is an `Ellipse3` with `SemiMinor = r` and `SemiMajor = r / |d̂ · nG|`; removed volume equals the section area times the length between the end planes measured through the section's centroid | treat `G` as normal to the edge: the volume is off by the centroid offset |
| Clearance | a radius past the face width refuses `ErrDegenerate`; a radius within `b` of it refuses `ErrUnsupported` | drop the `− b_v` term of §6.2: the near case builds an invalid solid |
| Shared vertex / tangent chain | refuse SR2 | |
| Top loop of a mesh box, `Fillet` r = 2 | mitre run (§4.7): the volume is the box less the columns' removed height, the larger of the two walls' at each column, integrated on a 1000 × 1000 grid; the section 0.1 mm under the top has its corner on the plane x = y; `Verify` is `Sound` | delete the mitre and the corner volume is the sum of both strips |
| Two or three top edges of a mesh box in a row | a two-edge run matches the same integral; both free ends on one face, or three edges at a corner, refuse SR2 | |
| A pocket wall inside the strips' reach | refuse SR2 | |
| Oblique plane/cylinder rim | refuse SR1 and keep the `Ellipse3` of retention §10.5 | |
| Plane-sphere rim (R6): a Revolve ball cut by a placed box, fillet and chamfer | the removed volume equals the zone of the sphere down to `zB` less the solid under the torus (or the frustum), from the meridian's closed form; the area matches the cap's less the annulus and the zone plus the blend; the `Torus` has `Major = ρc` and `Minor = r`, the `Cone` starts at the narrower setback circle; `VerifyAll` passes | drop R6 from dispatch: SX9 |
| R6 two-distance chamfer, and both rims of a twice-cut ball | the plane takes `dA`, the sphere's meridian `dB`; the second rim is found again on the partly blended body | |
| R6 refusals | a snapped centre, a rounded radius and a rotated revolve keep the rim's chords (retention E12) and refuse SR1 as a tag not proven to be exactly its revolve record; a ball too large, a plane setback past the axis and a sphere setback past the pole refuse SR3; a grazing crossing refuses SR5 (unit test) | drop E12's denoted arm: each of the three publishes a `Circle3` rim; drop its sphere-radius test: the rounded radius does; delete an SR3 or SR5 check: its fixture builds |
| R6 inner wall vertices | a sphere zone that holds tessellation vertices builds | gate inner vertices as R2 gates the cylinder's: "the blend consumes the wall" |
| R6 kept inner vertex bound (unit test) | a vertex held `β` outside the sphere off a point inside the zone gets a bound that reaches the nearest point of the new boundary | keep its old bound: the bound falls short |
| R6 pocket and dome foot | `Cut(slab, ball)` (convex, plane outside the rim, material outside the sphere) and `Union(ball, slab)` (concave, sphere rising): fillet and chamfer change the volume and the area by the meridian's closed forms; the torus and cone parameters match and the blend's normal points away from the material | flip the chamfer normal's sphere term for an outside material: the cone's normal points into the material |
| Area slack carries the shortfall of the face terms | the fillet's area change lies within the bound's growth | drop the shortfall: the rebuilt bound grows by less than the change's error |
| Shortfall by group (`internal/retainedblend/route_internal_test.go`, `internal/facetedrecord/build_test.go`) | a group whose old terms exceed its new ones by an exact half charges at least that half to `AreaSlack` and the area bound, a group the new body lacks charges all its old terms, the lower sum stays at or below the exact sum, `ChargeAreaSlack` raises every face's and the body's bound by at least the charge, and a difference that is not a number, as of two infinite terms, charges `+Inf`, which the build refuses | drop the charge, the group the new body lacks, the lower sum's step down, a face's or the body's charge, or the `+Inf` charge: each test fails |
| Four boss side fillets, then a rim chamfer, on one plate (`apitest/faceted_retained_blend_area_charge_test.go`): the 20×20×15 box boss, a counterbored M6 hole and a placed round boss of radius 6 on an 80×60×10 plate, r = 2, then d = 1 | each volume and area equals the hand value within its bound; `Verify` reads `Sound` with the area within its tolerance | charge each rewritten face's whole old term: the plate's top charges its term once per side edge and the area bound passes its tolerance |
| Result is an ordinary retained body | a second blend on another edge, a `Cut` through the blended region and `Transform` all build; `NormalAt` on the blend face has a bound below 1 | store the blend with `Retained = false`: the chain loses the tag |
| New vertex bounds | every new vertex lies within its stated bound of its exact point | skip the rounding charge: the bound is zero and the check fails |
| Verify | `Verify` of a blended plate passes with no `Suspect` in watertightness | |
| R7 cross-hole chamfer: Revolve shaft R = 10, pin r = 3 through it, both mouths, d = 1.5 and 0.5; pin r = 7, d = 0.5 (`apitest/retained_saddle_chamfer_test.go`) | the volume falls by twice the wedge's divergence-theorem integral (24.474 mm³ per mouth at d = 1.5, 2.460 at d = 0.5) within both bodies' bounds, and at d = 1.5 that change exceeds the bounds; the walls stay `Cylinder`, the bands read `Faceted`; on the `VerifyAll` mesh the wedge's section centroid reads outside, farther than the mesh bound, and a point 0.3 mm past the band reads inside, at 8 stations of each mouth; an Extrude shaft at d = 1.5 under the three axis assignments and a translation does the same (`TestRetainedSaddleChamferExtrudeShaft`; under `-race` the translated case only, on one mouth) | cut with `d/2`: the d = 1.5 volume and the d = 0.5 centroids fail; drop the outer foot's rotation from the held stations: the sample comparison widens `ε` and proof 4 refuses; drop the mouth-side flip or permute the pose's axes: the race-mode Extrude case refuses |
| R7 fillet: Revolve shaft R = 10, pin r = 3 through it, both mouths, ρ = 1.5 and 0.5; pin r = 7, ρ = 0.5; a `Hole` r = 3 through the shaft's wall, ρ = 1.5 (`apitest/retained_saddle_fillet_test.go`) | the volume falls by twice the tube-coordinate integral of §4.10, computed by Gauss–Legendre over exact ray exits (18.593228 mm³ per mouth at ρ = 1.5, 1.635978 at ρ = 0.5), within both bodies' bounds, and at ρ = 1.5 that change exceeds the bounds; the walls stay `Cylinder`, the fillets read `Faceted`; `VerifyAll` verifies the result; on its mesh a point midway between the arc and the edge along the arc's bisector reads outside, farther than the mesh bound, and a point 0.3·ρ inside the ball reads inside, at 8 stations of each mouth; an Extrude shaft at ρ = 1.5 under the three axis assignments and a translation does the same (`TestRetainedSaddleFilletExtrudeShaft`; under `-race` the translated case only, on one mouth). Unit tests (`internal/saddlecanal/proof_internal_test.go`, both sides of the larger cylinder; under `-race` one canal of each): the cell proofs' area intervals hold a midpoint-rule area of the fillet and Green's-theorem areas of both walls, the chord bound covers sampled held triangles, the rows split the widest arc, a moved held vertex raises the sample error, and ρ = 3.4 folds; `internal/retainedblend/saddle_fillet_internal_test.go` chains a cut ring through a merged sliver, reads a pose's hand, checks that the chamfer's recognition admits exactly the edges its rule before σ admitted, and that the fillet's admits its three patterns, over every pair of radii, `Reversed` flags and `Convex` flag; a second fillet of a rebuilt r = 3 shaft at ρ = 0.5 tessellates to the same vertices and triangles | fillet with ρ/2: the ρ = 1.5 volume and the ρ = 0.5 points fail; flip σ1 in the route: every build refuses, "the bore's cut leaves fewer than three vertices"; drop the strip's winding swap for a left-handed pose: the Revolve result fails its audit; drop the mouth-side flip: the race-mode Extrude case puts the fillet on the far mouth and its points fail; drop the Jacobian's sign or σ1 in `G`: the outside unit canals refuse, "folds" and "is not embedded" |
| R7 fillet outside both cylinders (`TestRetainedSaddleFilletOutside`): a block 80 × 60 × 20 with a radius-5 `Hole` down through it crossed by a radius-3 blind `Hole`, both mouths, ρ = 0.5 (convex, removes); a radius-5 `Boss` 8 high on a radius-15 rod, ρ = 1 (concave, adds); a radius-3 pin joined through the R = 10 Revolve shaft, both roots, ρ = 1.5 (concave, a left-handed pose) | the volume falls (block) or grows (rod, pin) by the tube-coordinate integral per mouth (0.540755, 4.701377 and 6.619077 mm³) within both bodies' bounds; `VerifyAll` verifies the result; on its mesh the point midway between the arc and the edge reads outside for the block and inside for the rod and pin, farther than the mesh bound from every facet, and the point 0.3·ρ inside the ball keeps its side, at 8 stations of each mouth; under `-race` the rod only | expect ρ/2 or σ1 = −1 in the test: all three fail, at the ball's points or the volume; flip σ1 in the route: every build refuses, "the bore's cut leaves fewer than three vertices" |
| R7 fillet refusals | `ρ = 3.5` (`2ρ = R − r`) refuses `ErrDegenerate` "no ball of that radius touches both walls of the saddle"; ρ = 3.4 refuses "folds: the pipe is not regular there"; a pin moved 0.5 off the shaft's axis, the pin's part inside the shaft (`Intersect`, σ = (−1, −1)) and a concave pin root inside a tube's bore (`Chamfer` and `Fillet`) refuse with the canal-surface reason; a flat milled beside a mouth refuses "another face of the body comes within the saddle fillet's reach", as does a boss 1 high on the radius-15 rod at ρ = 1, whose top face lies inside the bore's cut (`TestRetainedSaddleFilletRefusals`) | drop the route's existence test: ρ = 3.5 refuses at `saddlecanal.Prove`'s own, "the saddle canal's offset cylinders do not meet in a loop" (a unit test pins that site's text); drop the Convex flag from `saddleKey.built`: the tube root's `Chamfer` builds and removes material where a concave blend adds it, and both recognition unit tests fail; drop the Jacobian test: ρ = 3.4 refuses at the progress test instead; drop the clearance check: the flat refuses later, "a corner of the saddle fillet's window lies on no facet of the shaft wall", and the short boss "the bore's cut leaves fewer than three vertices"; admit every σ: the tube root's `Fillet` builds and the clipped pin refuses "a corner of the saddle fillet's window lies on no facet of the shaft wall"; drop the exact axis meeting: the moved pin refuses with "a held seam vertex is off the two cylinders" |
| R7 refusals | `d > r/2` refuses; a pin moved 0.5 off the shaft's axis, a pin with r/R = 0.75 and a pin joined through the shaft refuse with the canal-surface reason; a flat milled beside a mouth refuses "another face of the body comes within the saddle chamfer's reach"; `Chamfer` of a pin's roots and of a boss's root on a rod keeps the canal-surface reason (`apitest/blend_refusal_names_test.go`) | drop the clearance check: the flat builds; admit a bore that is not `Reversed`: the joined pin builds; drop the exact axis meeting: the moved pin refuses with "a held seam vertex is off the two cylinders"; drop `2r² < R²`: r/R = 0.75 builds |

## 10. Files and fallout

| Area | Change |
|---|---|
| `internal/retainedblend/` | new: classification (Table RR), exact geometry (§4), clip, strip and end construction (§5), gates (§6); R6 in `sphere.go`, its exact-record rule in `denoted.go`; R3's wall band in `cyl_band.go` |
| `internal/facetedrecord/` | an entry that builds a payload from new vertices, triangles, groups and bounds; no change to `Build` |
| `internal/retainedblend/saddle.go`, `saddle_tool.go`, `internal/crossbore/saddle_tool.go` | R7 (§4.9): recognition, tool, band proof, outside and clearance proofs, the Cut |
| `internal/brepmodify/chamfer.go` | calls `retainedblend.SaddleChamfer` with the hooks' mesher before `retainedblend.Chamfer` |
| `internal/retainedblend/saddle_fillet.go`, `internal/saddlecanal/` | R7 fillet (§4.10): the window and bore cuts, the bands, the strip and the bounds; the exact fillet, its rows and its cell proofs |
| `internal/brepmodify/fillet.go` | calls `retainedblend.SaddleFillet` before `retainedblend.Fillet` |
| `fillet.go`, `chamfer.go` | call the route before SX9 for a `facetedPayload`; thin wrappers only |
| `agent-docs/modify-reach-design.md` | RX6 row and SX9 text; the sentence in §11 about what SX9 leaves a caller proving |
| `agent-docs/missing-features.md` | the row for faceted `Fillet`/`Chamfer` |
| `agent-docs/faceted-source-retention-design.md` | this document's row in §9, §13 |
| `agent-docs/layout/design-docs.md` | this document's row |
| `apitest/`, `examples/` | the tests of §9; one executable example, a filleted hole mouth on a placed-tool plate |

## 11. Increments

| # | Lands | Gate to start the next |
|---|---|---|
| B1 | R2 fillet and chamfer: classification, the four rims of §4.2, clip and strip, bounds, gates SR1–SR4 and SR6–SR7 | the hole-mouth, boss-root and shaft-end rows of §9 pass; `go test ./...` on CI |
| B2 | R1 fillet and equal-distance chamfer with planar end faces, SR5 | the 90° and 45° edge rows, the oblique-end row and the L-section concave row pass |
| B3 | R3, a line between a plane and a parallel cylinder, ending on planes normal to the axis | the groove, chamfer and boss-foot rows of §9 pass |
| B4 | R4, an open arc | as B1 for an arc with end faces |
| B5 | R6, a closed plane-sphere rim (§4.5) | the R6 rows of §9 pass |
| B6 | R7, the cross-hole saddle chamfer (§4.9) | the R7 rows of §9 pass |
| B7 | R7 fillet: the convex mouth (σ = (−1, +1)), and the ball outside both cylinders (σ = (+1, +1)) at a convex edge, two crossing bores, and at a concave one, a boss's root on a rod (§4.10) | the R7 fillet rows of §9 pass |
| B8 | the remaining sign patterns of crossing cylinders: a pin's root inside a tube's bore (σ = (−1, +1) at a concave edge) and every pattern with the ball inside the bore | a design for each pattern's walls and bands |

## 12. Rejected shortcuts

- **A Cut by a cutter solid.** The cutter's blend face is tangent to the faces
  it cuts, and the mesh Boolean refuses tangency. A slightly deeper cutter is
  not the fillet.
- **Promote the retained body to a `brepPayload`.** A brep keeps every frame a
  signed permutation of one reference frame. The bodies this route exists for
  are the ones that do not.
- **Fit a cylinder or torus to the held rim.** It blesses a surface from the
  polygons it should be checked against (CLAUDE.md). The tags come from the
  operands.
- **Admit a blend because the rebuilt volume matches.** A matching volume
  proves nothing about a blend that leaves its faces; the gates of §6.2 admit
  from published bounds, and the volume check can only reject.
- **Blend an oblique plane and cylinder as a `NURBSSurface`.** The surface
  carries no geometry a proof can read.
- **Chamfer a cross hole with a coaxial cone.** A countersink cone sets back
  unequally: along the bore wall its setback varies by `R − √(R² − r²)` around
  the mouth. R7 cuts the equal-setback ruled band instead (§4.9).
- **Fillet a cross hole by a Cut with a tool like §4.9's.** The tool's face
  would cut both walls at an angle. The true fillet meets each wall tangentially
  along T1 and T2, so the Cut leaves a crease the solid does not have.
- **Cut the shaft wall along a curve of T1's family.** The shaft's held facets
  are long strips, and such a curve crosses a strip between its vertices: a cut
  that reads the level at the vertices misses the strip, and a ring cut from the
  vertex levels lies up to a facet's width (0.4 mm on the R = 10 shaft) off the
  curve, far beyond the strip's first row. §4.10 cuts by planes instead.

## 13. Decisions the user may want to overturn

| Decision | Default | Alternative |
|---|---|---|
| Edge classes in the first increment | R2 (closed circular rims) first, because they have no end vertices | R1 first: more common, but needs the end-face construction and SR5 |
| A blend whose clearance is undecided within bounds | refuse (`ErrUnsupported`) | build with a bound that charges the undecided margin: smaller refusal set, larger proof |
| Chord tolerance `τ = 2⁻¹⁰·r` for the blend grid | fixed ratio of the radius | follow the body's `MeshBound`: finer blend, many more facets |
| Several selected edges | one at a time on the evolving mesh; each later rim is found again by its circle | require disjoint regions `K` and build them together |
| `Shell` of a retained body | stays SX9 | a separate design |
| R7 tool stations | `N = 1024`: two mouths cut in about 2 s, about 20,000 facets | `N = 4096`: `ε` four times smaller, about 70,000 facets and several times the time |
| R7 cut's chain allowance | raised to the walls' held bounds and `ε` (§4.9) | keep the document's: every R7 cut on a Boolean result refuses at the chain-depth gate |
| R7 ratio | `2r² < R²` | admit what the cell proofs pass (r/R = 0.75 builds with the gate dropped): wider reach, no checked fixture past 0.7 |
| R7 fillet stations and rows | `N = 256`, rows for `τ = 2⁻¹⁰·ρ`: two mouths in about 0.45 s and 30,000 facets at ρ = 0.5, ε from 0.0020 mm (ρ = 0.5) to 0.0059 mm (ρ = 1.5) | `N = 1024` as §4.9: ε about a quarter, four times the facets and time |
| R7 fillet cell split along t | every cell splits in two along t (`rowSplit`): the fillet's area enclosure narrows about fourfold (7.4 to 1.7 mm² on the R = 15, r = 5, ρ = 1 rod, whose `Verify` then reads Sound) for about a third more proof time | no split: the proof's time, and an area bound past `Verify`'s relative tolerance on that rod; or four: the enclosure narrows about twice more for twice the time |
| R7 fillet face | untagged `Faceted`, role `fillet(k)` | a sealed `Canal` surface variant (`agent-docs/trim-graph-design.md` increment 9): a tag a reader can query, a new public type |
| R7 fillet sign patterns | σ = (−1, +1) at a convex edge (the cross hole's mouth) and σ = (+1, +1) at either edge (crossing bores, a boss's root) | the patterns of B8 as well |
