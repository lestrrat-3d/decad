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
computed from them has no proof (SR1).

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

Refused, with the reason a caller can act on:

- **An oblique plane and cylinder (`Ellipse3`).** The surface that rolls a ball
  between them is a pipe surface about an ellipse offset. It is neither a
  `Cylinder`, `Cone`, `Torus` nor `Sphere`, and the only other surface the
  vocabulary has is `NURBSSurface`, an opaque marker with no geometry to carry
  a proof (spline §7). The rim keeps its published ellipse and the blend
  refuses (SR1).
- **A `Cylinder`/`Cylinder` rim with crossing axes, an open (`Arc3`) plane-sphere
  rim, a `Cone` or `Torus` face.** No Table RR row; SR1.
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

The wall's setback line is the generator of the cylinder at `T_B`. Cutting the
held wall there is a cut by the half-plane that holds the axis and that
generator, applied to the facets whose angle about the axis, measured from `K`
into the wall, is below the angle of `T_B`.

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
this route's own. Each failure refuses SR1 with a message naming the leaf: a
rotated placement fails the first, a half-disk centre within the snap
tolerance of the axis fails the second, and a meridian whose recorded radius is
no float (an arc to `2.1` about `0.1`) fails the third. A `Sphere` face with no
denoted patch is its own record, as for every other kind (§2).

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

**Exact areas.** The removed region of `A` is the quadrilateral
`V₀ V₁ P_A,₁ P_A,₀`, and `B`'s likewise. The removed region of `G_i` is the
projection along `d̂` of the corner region of the cross-section, area
`r²(tan(φ/2) − φ/2)`, so its area is that over `|d̂ · n_G|`. A chamfer removes
the triangle `V_i P_A,i P_B,i` from `G_i`. The cylinder patch has area
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
| **SR3** | no blend of that size exists: `R − r ≤ 0` for an inside ball (§4.2), `φ` within 10⁻⁹ of 0 or π, a chamfer distance of zero or below, or an R6 setback that reaches the axis or runs past the sphere's pole (§4.5) | no regular blend | `ErrDegenerate` |
| **SR4** | the setback does not fit (§6.2) and the held bounds prove it does not | the blend consumes the face | `ErrDegenerate` |
| **SR5** | the setback might not fit and the held bounds cannot decide it, or a plane that crosses an R6 sphere below `minSine` (§4.5), or an end vertex of an R1 edge is not a three-face corner with a retained planar `G`, or `G`'s corner at the vertex does not match the edge (a convex edge needs a convex corner of `G`, a concave edge a reflex one), or both `A` and `B` turn reflex at the vertex, or a convex edge meets a reflex face, or another part of `G` comes within the blend's reach of the vertex | body exists; the evaluator cannot prove its construction | `ErrUnsupported` |
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
torus, and otherwise the two retained kinds the edge lies between.

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
then the construction, then SR7.

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
only with every face's geometric term (the smaller of its largest facet bound
times its perimeter and its facets' perturbation allowance), and the rebuilt
body charges the rewritten faces' terms afresh from their new facets, which can
be smaller: a rewrite drops high-bound rim vertices and whole facets. The new
`AreaSlack` is therefore the old one plus `|Δ_held − Δ_true|`, the rounding of
the two area sums (`proofbound.FacetAreaTermSlop` per facet), and the old
geometric term of each face the rewrite took facets from
(`facetedrecord.FaceAreaGeom`).

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
| Concave edge in an L section | the fill adds `L·r²(1 − π/4)`, the blend's normal points into the void, `VerifyAll` passes | build the convex blend: the normal points into the material |
| Concave edge at a wall's foot | the plate face is reflex at each foot corner; the fill adds `L·r²(1 − π/4)`, the wall's end faces grow by the caps, `VerifyAll` passes; two foot edges together and a chamfer build as well | keep `V` out of the extended end face's loop: the mesh fails its audit |
| Oblique end face | the end arc is an `Ellipse3` with `SemiMinor = r` and `SemiMajor = r / |d̂ · nG|`; removed volume equals the section area times the length between the end planes measured through the section's centroid | treat `G` as normal to the edge: the volume is off by the centroid offset |
| Clearance | a radius past the face width refuses `ErrDegenerate`; a radius within `b` of it refuses `ErrUnsupported` | drop the `− b_v` term of §6.2: the near case builds an invalid solid |
| Shared vertex / tangent chain | refuse SR2 | |
| Top loop of a mesh box, `Fillet` r = 2 | mitre run (§4.7): the volume is the box less the columns' removed height, the larger of the two walls' at each column, integrated on a 1000 × 1000 grid; the section 0.1 mm under the top has its corner on the plane x = y; `Verify` is `Sound` | delete the mitre and the corner volume is the sum of both strips |
| Two or three top edges of a mesh box in a row | a two-edge run matches the same integral; both free ends on one face, or three edges at a corner, refuse SR2 | |
| A pocket wall inside the strips' reach | refuse SR2 | |
| Oblique plane/cylinder rim | refuse SR1 and keep the `Ellipse3` of retention §10.5 | |
| Plane-sphere rim (R6): a Revolve ball cut by a placed box, fillet and chamfer | the removed volume equals the zone of the sphere down to `zB` less the solid under the torus (or the frustum), from the meridian's closed form; the area matches the cap's less the annulus and the zone plus the blend; the `Torus` has `Major = ρc` and `Minor = r`, the `Cone` starts at the narrower setback circle; `VerifyAll` passes | drop R6 from dispatch: SX9 |
| R6 two-distance chamfer, and both rims of a twice-cut ball | the plane takes `dA`, the sphere's meridian `dB`; the second rim is found again on the partly blended body | |
| R6 refusals | a snapped centre, a rounded radius and a rotated revolve refuse SR1 with their own messages; a ball too large, a plane setback past the axis and a sphere setback past the pole refuse SR3; a grazing crossing refuses SR5 (unit test) | delete each check: its fixture builds, and the rotated one panics on the unread record |
| R6 inner wall vertices | a sphere zone that holds tessellation vertices builds | gate inner vertices as R2 gates the cylinder's: "the blend consumes the wall" |
| R6 kept inner vertex bound (unit test) | a vertex held `β` outside the sphere off a point inside the zone gets a bound that reaches the nearest point of the new boundary | keep its old bound: the bound falls short |
| R6 pocket and dome foot | `Cut(slab, ball)` (convex, plane outside the rim, material outside the sphere) and `Union(ball, slab)` (concave, sphere rising): fillet and chamfer change the volume and the area by the meridian's closed forms; the torus and cone parameters match and the blend's normal points away from the material | flip the chamfer normal's sphere term for an outside material: the cone's normal points into the material |
| Area slack carries the old face terms | the fillet's area change lies within the bound's growth | drop the old terms: the rebuilt bound grows by less than the change's error |
| Result is an ordinary retained body | a second blend on another edge, a `Cut` through the blended region and `Transform` all build; `NormalAt` on the blend face has a bound below 1 | store the blend with `Retained = false`: the chain loses the tag |
| New vertex bounds | every new vertex lies within its stated bound of its exact point | skip the rounding charge: the bound is zero and the check fails |
| Verify | `Verify` of a blended plate passes with no `Suspect` in watertightness | |

## 10. Files and fallout

| Area | Change |
|---|---|
| `internal/retainedblend/` | new: classification (Table RR), exact geometry (§4), clip, strip and end construction (§5), gates (§6); R6 in `sphere.go`, its exact-record rule in `denoted.go` |
| `internal/facetedrecord/` | an entry that builds a payload from new vertices, triangles, groups and bounds; no change to `Build` |
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

## 13. Decisions the user may want to overturn

| Decision | Default | Alternative |
|---|---|---|
| Edge classes in the first increment | R2 (closed circular rims) first, because they have no end vertices | R1 first: more common, but needs the end-face construction and SR5 |
| A blend whose clearance is undecided within bounds | refuse (`ErrUnsupported`) | build with a bound that charges the undecided margin: smaller refusal set, larger proof |
| Chord tolerance `τ = 2⁻¹⁰·r` for the blend grid | fixed ratio of the radius | follow the body's `MeshBound`: finer blend, many more facets |
| Several selected edges | one at a time on the evolving mesh; each later rim is found again by its circle | require disjoint regions `K` and build them together |
| `Shell` of a retained body | stays SX9 | a separate design |
