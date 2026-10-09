# Loop Fillet Design

`Body.Fillet` of a complete loop of a planar face: a prism's cap loop, a
brep or stacked result's planar-face loop, a hole rim or boss root, and the
honest form of a top edge ending on a fillet cylinder (brep-modify SB7). Companion to
`docs/modify-general-design.md` ("modify-general §N"), whose route L this
document extends with a fillet arm, and to `docs/modify-reach-design.md`
("reach §N"), whose §8.2 corner rule it replaces. `docs/modify-design.md`
("modify §N") keeps the call contract and the §5 audit;
`docs/brep-modify-design.md` ("brep-modify §N") keeps the brep receiver's
gate order.

Five tables are normative:

| Table | Owns | Section |
|---|---|---|
| **LF** | the result surface and edge kind per corner class of the loop | §2 |
| **RF** | receivers, and how each reaches route L | §3 |
| **SF** | every refusal the fillet arm adds, with its sentinel and gate order | §6 |
| **BF** / **DF** | result payload and what every consumer reads | §7 |
| **CF** | the strip coefficients every measurement reads | §5.1 |

## 1. The one idea: route L's band with a circular height map

Route L's chamfer band (modify-general §4.2) is the exact offset family of
the loop `ℓ` on face `F`: at axial fraction `s` from the side level toward
`F`, the body's section is `F`'s region with `ℓ` offset by `s·d` into `F`'s
material. The fillet of the same loop at radius `r` is the SAME family under
a different height map. A ball of radius `r` touching `F` and a wall beside
`ℓ` has its centre `r` below `F` (toward the walls) and `r` from the wall
into `F`'s material; the surface it sweeps, read at distance `h` from the
side level `sideZ = L_F ∓ r` toward `F`, is the loop offset by

```text
t(h) = r − √(r² − h²),   0 ≤ h ≤ r,   t(0) = 0 (the side contour, ℓ itself),  t(r) = r (the cap contour)
```

Every level curve of the fillet band is an exact 2D offset of `ℓ`: the lines
move along their inward normals, a circular wall's offset is the concentric
circle, a reflex corner of `F`'s region closes with the connector arc of
radius `t` about the corner, and a convex corner's two offsets cross at the
miter foot. The chamfer uses the linear map `h = t·ds/dc`; the fillet uses
the circular one. So the fillet arm reuses, unchanged:

- the cap contour at `t = r` (`capLoopBoundary`, modify-general §4.2 step 1),
  its displacement (`capband.ContourDisplacement`) charged into `F.delta`;
- the side contour, `ℓ` moved `r` along `F`'s normal (step 2), with
  `levelDelta`;
- the trims of the (sw) and (pl) faces beside the loop (step 3), the face
  audit (step 4), the record's closure (step 6);
- Table LB's admission (LB1–LB6) and the gates SX6, SX7 (as `requireBandReach`
  sums it per wall piece, so two bands on one wall from both ends refuse
  where `r₁ + r₂ ≥ h`), SX12, SX13 and SX14, each on the offset at `r`.
  SX12's argument holds under any monotone height map: a crossing anywhere
  in the family occurs no later than at the full offset.

What changes is the surface between the two directrices, the curve of a
convex corner's miter, the measurements, and the mesh.

## 2. Table LF — surfaces and edges per corner class

The loop's walks are resolved as route L resolves them
(`oneLoopCornerLoop`, `capOffsetJoins`). Each walk and each corner of `ℓ`
yields one patch or one edge:

| LF | Piece of `ℓ` | Patch (face) | Trimmed by |
|---|---|---|---|
| **LF1** | a straight walk of length `ℓᵢ` | `Cylinder{Origin: the wall's line offset r into F's material, at sideZ; Axis: the walk direction; Radius: r}`, the quarter from the contact line on `F` (the contour segment, at `L_F`) to the contact line on the wall (`ℓᵢ` at `sideZ`) | its two end curves (LF4–LF6) |
| **LF2** | a circular walk, radius `R > r`, centre `C`, material inside the circle | `Torus{Center: C at sideZ; Axis: F's normal; Major: R − r; Minor: r}`, the quarter of the tube from its top (radius `R − r` at `L_F`) to its outer equator (radius `R` at `sideZ`). `R − r < r` is a spindle torus whose outer quarter is the patch and never meets its axis | LF5/LF6 ends, or none on a whole turn |
| **LF3** | a circular walk, material outside the circle (a hole rim) | `Torus{Center: C; Axis: F's normal; Major: R + r; Minor: r}`, the quarter from the top (radius `R + r` at `L_F`) to the inner equator (radius `R` at `sideZ`) | the same |
| **LF4** | a convex corner where two straight walks meet at interior angle `θ` (material side) | no patch: the two cylinders meet along their intersection | the edge is a planar quarter ellipse in the corner's bisector plane, `Ellipse3{Center: the ball centre at the corner, c = corner + (r / sin(θ/2))·b̂ at sideZ; Axis: the bisector plane's normal; Major: b̂; SemiMajor: r / sin(θ/2); SemiMinor: r}`, from the corner vertex at `sideZ` to the contour corner at `L_F`; `b̂` is the in-plane bisector into the material |
| **LF5** | a G1 join (a line tangent to an arc, or two tangent arcs), route L's dead-zone rule | no patch: the two pipes share a meridian | `Arc3`, the quarter circle of radius `r` in the plane through the join normal to `ℓ`, from the join vertex at `sideZ` to the contour foot `v + r·n̂` at `L_F` |
| **LF6** | a reflex corner of `F`'s region with turn `ψ` (a hole or pocket mouth's corner, a square boss root's corner) | `Torus{Center: the corner at sideZ; Axis: F's normal; Major: r; Minor: r}`, the horn torus whose tube runs from the connector arc of radius `r` at `L_F` to the apex on its own axis at `sideZ`, over azimuth `ψ`; both joins to the neighbouring patches are G1 | two LF5 `Arc3` meridians, at the connector arc's two ends `pA`, `pB` |
| **LF7** | a whole circle | LF2 or LF3 over `2π`; two `Circle3` edges and no seam | — |
| **LF8** | a convex corner where a straight walk meets a circular one, or two circular walks meet, not tangent | refused, SF1 | — |
| **LF9** | inward open arc with both recorded endpoints exactly at `r` and straight G1 neighbours | sphere from side arc to cap pole, bounded by two LF5 meridians (`docs/vertex-blend-design.md` §1) | three arcs |

The sphere does not appear at a sharp convex corner. Reach §8.2 states a trimmed `Sphere` at a
zero-length miter; this document replaces that row (§9). At a convex corner
of one loop, the ball touching `F` and wall `A` reaches the centre `c` where
it also touches wall `B`, and from `c` it rolls along `B`. The ball never
rests at `c`, so no spherical patch is swept; the material the two blends
remove is the union of the two per-wall wedges, whose boundary inside the
body is cylinder `A` where it lies inside cylinder `B` and the converse,
meeting along the ellipse of LF4. A sphere octant at `c` would remove more:
its lower boundary arc lies in the plane `z = sideZ` on the circle of radius
`r` about `c`'s projection, and runs from `(r, 0)` to `(0, r)` in corner
coordinates, through `(r − r/√2, r − r/√2)`, a point strictly inside both
walls. Below that arc the body would keep its full corner square at
`z = sideZ` and above it only the quarter disk, so a flat shelf of area
`r²(1 − π/4)` would be exposed at the side level. No CAD kernel produces that
surface for two blended edges meeting at an unblended third: Fusion's fillet
of a box's top four edges shows the two blends meeting along a curved seam
and no third face. The sphere octant appears after the lateral edge is
filleted too; route V builds it through LF9 (`docs/vertex-blend-design.md`).

At height `h` the offset corner of two lines lies on the bisector at distance
`t(h)/sin(θ/2)` from the corner, so the miter curve is
`{corner + (t/sin(θ/2))·b̂ + h·n̂ : t = r − √(r² − h²)}`, which satisfies
`((u − r/sin(θ/2))·sin(θ/2))² + h² = r²` in bisector coordinates `(u, h)`:
the ellipse of LF4, which is also the section of either cylinder by the
bisector plane. A line–circle or circle–circle convex corner's foot locus is
a conic in the plane (reach §8.3), and lifted by `t(h)` it is a space curve
no `Curve` variant names and whose length and strip integrals (§5) are not
polynomial in `t`; SF1 refuses it rather than publish a `Line3` or `Arc3`
stand-in a first-order distance away.

`Ellipse3` is a new sealed `Curve` variant (api §4 extension, shipped in
PR F-0): `type Ellipse3 struct { Center, Axis, Major r3.Vec; SemiMajor,
SemiMinor units.Value }`, the arc swept counter-clockwise about `Axis` from
the start vertex to the end vertex, as `Arc3` is. `Major` is the unit
direction of the semi-major axis. Its parameters are exact for the curve they
name (api §4); the curve sits within `F.delta` of the one the fillet denotes,
since `c` is the offset corner's float solve. Every production switch on
`Curve` carries a default (`geometry.go`'s rule), so F-0 adds an arm only
where the default would be wrong: `surfacegeom/placement.go` (map `Center`,
`Axis`, `Major`), `selectedEdgeContext` (render centre and semi-axes),
`brepEdgeMatches` (match nothing: a band edge is in no record); it verifies
that `stops.go`, `verify_gate.go`, `clearance_geom.go` and `selector.go`
read an unknown curve through its two vertices or refuse it. `Edge.Length`
of an LF4 edge is the quarter-ellipse arc length
`∫₀^{π/2} √(b² + (a² − b²)·sin²φ) dφ`, `a = r/sin(θ/2)`, `b = r`, enclosed by
the monotone integrand's lower and upper Riemann sums over 64 equal
sub-ranges; the held value is the midpoint and the bound the half-width.

Edge convexity: the contact edges on `F` and on the walls, and every LF5
meridian, are tangent junctions and carry the band's sense (`IsConvex` true
on a band that removes material, false on one that fills); an LF4 ellipse
carries the loop corner's own 2D turn, the sign the receiver's vertical edge
there carried.

## 3. Table RF — receivers

| RF | Receiver | Path to route L |
|---|---|---|
| **RF1** | `brepPayload` | brep-modify §6 stages 2a–2b, then route L (modify-general §4.4 stage 2c); the fillet arm replaces SL3 |
| **RF2** | `stackedPrismPayload` | the same through `brepOfStacked` (SB2 first) |
| **RF3** | `prismPayload` whose selection holds a cap edge | `fillet.go`: where `matchCornerBudget` finds no lateral corner for a selected edge, the call takes `brepOfPrism(pp)` and dispatches route E, L or V by the selection (`docs/vertex-blend-design.md` §2). The result is a brep body; every-lateral selections keep the prism path and B1's roles |
| **RF4** | `revolvePayload` cap edge | SX5 unchanged: the cap's adjacent faces are revolve surfaces, not faces swept along the cap normal (LB3 fails), and the blend of a planar loop whose walls are revolved surfaces is a pipe about a general planar curve that no level set of an offset family states |
| **RF5** | `capBlendPayload`, draft, sweep, loft, faceted | SX10, modify S3, SX9 unchanged |

RF3's result is a `brepPayload`, not a `capBlendPayload`: the fillet patches
and their readers (§5, §7) live once, on the band record, and the prism cap
chamfer's payload is not given a second patch family. A prism cap-loop fillet
therefore returns `face(k)`/`wall(k)` roles with `filletLoop(f,l,p)` patches,
where its chamfer returns BX3's `capStart`/`side(i,j)`/`chamferCap`. §9.

Selection: Table LB unchanged. A `Fillet` whose matched edges are whole loops
of planar faces, of any segment count, takes the fillet arm; a single whole
circle no longer goes on to route E (SB4). Route E keeps every selection of
single straight edges along axes (`singleStraightEdges`).

## 4. Construction

For each admitted loop `ℓ` of face `F` (normal axis `n`, level `L_F`), radius
`r` with its conversion bound `rDelta` (`capSetback{dc: r, ds: r, dcDelta:
rDelta, dsDelta: rDelta}`), sense `σ` from LB6:

1. **Record rewrite.** Modify-general §4.2 steps 1–4 and 6 verbatim with
   `d = r`: the cap contour, `F`'s region rewritten, `sideZ = L_F + σ·r`
   (`brepLoopBand.sideLevel`), the (sw)/(pl) trims and audits.
2. **Corner classes.** Each corner of `ℓ`'s coalesced walk is classified from
   the record's own floats as `capband.OccupiedVolumeAdmission` classifies
   them: a reflex corner (`joins[i].arc`) is LF6; a line–line miter is LF4;
   a join `capband.JoinIsG1` proves tangent is LF5; a whole turn is LF7; any
   other corner is SF1. The classification is exact rational arithmetic on
   recorded coordinates and admits nothing on a residual.
3. **Band record.** `brepLoopBand` gains `kind` (`"chamfer"` | `"fillet"`);
   the record stores `(face, loop, orig, setback, sigma, kind)` as route L
   does. No face record holds a cylinder with an in-plane axis, a torus or an
   ellipse (modify-general §8).
4. **Build.** After `evalBrepContext` has built the record's faces and edges,
   `attachFilletBand` (`brep_loop_fillet.go`) runs in `F`'s frame over
   `brepLoopBand.view`, with the cap contour's coedges supplied as route L
   supplies them and the side contour's coedges the trimmed walls' rims at
   `sideZ`. Per walk `i` it mints the LF1–LF3 patch face whose loop is: the
   side contact edge (shared), the end curve at the corner after `i`, the cap
   contact edge reversed (shared), the end curve at the corner before `i`
   reversed. Per LF6 corner it mints the horn-torus face: the connector arc
   at `L_F` (shared with `F`), the `pB` meridian, the apex vertex at `sideZ`
   (the loop corner's own vertex, `sideVertexAt`), the `pA` meridian. Every
   end curve is minted once and shared by the two patches it bounds. Vertices
   are route L's: contour corners and connector ends at `L_F` with
   `F.delta`'s bound, loop corners at `sideZ` with `levelDelta`.
5. **Roles.** `filletLoop(f,l,p)` for face `f`, loop `l`, patch `p` in the
   band's own order: walk `i`'s patch, then the LF6 patch at the corner after
   walk `i`. A patch face's `Surface()` is its LF tag; `normalBound` is zero,
   because the tag's surface is the built surface and the quarter window is
   structural (§7 DF7); placement rounding of the tag's direction is
   `Face.NormalAt`'s own arithmetic term (`normal_bound.go`), as on a prism's
   cylinder wall.
6. **Closure.** `attachFaceLoopsContext` proves every edge bounds exactly two
   faces, as for a chamfer band.

Where the walls rise off `F` (`σ = +1`: a boss root, a pocket floor's outer
loop) the band fills the concave corner: the same rewrite with `sideZ` on
`F`'s outer side, and the patches' outward normals pointing into the void
between `F` and the wall. The ball's centre is then `r` above `F` and `r`
from the wall, so at height `h'` above `F` the fill extends
`r − √(2rh' − h'²)` from the wall, which is `t(r − h')`: read from `sideZ`
toward `F`, the offset is `t(h)` in both senses, and every formula below
measures `h` that way.

## 5. Measurements

### 5.1 Table CF — the strip coefficients

Let `S(t)` be the strip between `ℓ` and `ℓ` offset `t` into `F`'s material,
`0 ≤ t ≤ r`, in `F`'s plane-local `(u, v)`. For every admitted corner class
its area and first moment are polynomials in `t`:

```text
A(t) = a₁·t + a₂·t²
M(t) = ∫_{S(t)} (u, v) dA = m₁·t + m₂·t² + m₃·t³      (m₁, m₂, m₃ ∈ ℝ²)
```

because every admitted corner's foot locus is affine in `t` (a line–line
miter moves along the bisector at rate `1/sin(θ/2)`, a G1 foot along the
shared normal, a reflex foot along each wall's own normal; reach §8.3). The
coefficients sum over the loop's pieces. A straight walk from `P` to `Q`
has length `ℓᵢ`, unit tangent `τ`, inward unit normal `ν` (into `F`'s
material), and end rates `κ₀` at `P`, `κ₁` at `Q`, where `κ = cot(θ/2)` at
an LF4 corner of interior angle `θ` and `κ = 0` at an LF5 or LF6 corner; its
strip piece is the trapezoid `{P + s·τ + w·ν : 0 ≤ w ≤ t, κ₀·w ≤ s ≤ ℓᵢ −
κ₁·w}`. A circular walk has centre `C`, radius `R`, sweep `β > 0`, and
`E = (sin α₁ − sin α₀, cos α₀ − cos α₁)` over its angular interval
`[α₀, α₁]` taken increasing. A reflex corner at `V` with turn `ψ` spans the
angular interval `[γ₀, γ₁]` between the arriving walk's inward normal and the
leaving walk's, `E_V` likewise.

| CF | Piece | `a₁` | `a₂` | `m₁` | `m₂` | `m₃` |
|---|---|---|---|---|---|---|
| **CF1** | straight walk | `ℓᵢ` | `−(κ₀ + κ₁)/2` | `ℓᵢ·P + (ℓᵢ²/2)·τ` | `−(κ₀+κ₁)/2·P − (ℓᵢ·κ₁/2)·τ + (ℓᵢ/2)·ν` | `((κ₁² − κ₀²)/6)·τ − ((κ₀+κ₁)/3)·ν` |
| **CF2** | circular walk, material inside | `β·R` | `−β/2` | `β·R·C + R²·E` | `−(β/2)·C − R·E` | `E/3` |
| **CF3** | circular walk, material outside | `β·R` | `+β/2` | `β·R·C + R²·E` | `+(β/2)·C + R·E` | `E/3` |
| **CF4** | reflex corner (LF6) | 0 | `ψ/2` | 0 | `(ψ/2)·V` | `E_V/3` |
| **CF5** | LF4 corner, LF5 join | 0 | 0 (its share is in `κ`) | 0 | 0 | 0 |

Derivations. CF1: `M = ∫₀^t [P·Λ(w) + τ·((ℓᵢ − κ₁w)² − κ₀²w²)/2 + ν·w·Λ(w)] dw`
with `Λ(w) = ℓᵢ − (κ₀+κ₁)w`; the kite two walls overlap in at an LF4 corner
has area `t²·cot(θ/2)`, split as one triangle `κ·t²/2` per wall. CF2:
`∫_{R−t}^{R} ρ² dρ = R²t − Rt² + t³/3`; CF3: `∫_R^{R+t} ρ² dρ = R²t + Rt² +
t³/3`. CF4: the sector of radius `t` fills the gap the two wall strips leave.
The check on route L's own fixtures: P2's top loop has `a₁ = 160, a₂ = −4`
and its mouth `a₁ = 60, a₂ = π`, so the chamfer's `∫₀^d A(s) ds = a₁d²/2 +
a₂d³/3` reproduces `80d² − 4d³/3` and `30d² + πd³/3` (modify-general §9).

Every coefficient is enclosed in rational interval arithmetic
(`internal/proof`) over the recorded coordinates with outward-rounded square
roots: `ℓᵢ`, `τ`, `ν` from `Q − P`; `κ = cot(θ/2) = (1 + cos θ)/sin θ` from
the exact dot and cross products of the two walls' directions divided by the
enclosed lengths; `R` and `E` from the recorded centre and ends (`sin α =
(P_v − C_v)/R`); `β` and `ψ` through `Atan2Interval`. An axis-aligned loop of
lines at recorded coordinates has every coefficient an exact rational.

### 5.2 The height integrals

```text
J_k = ∫₀^r t(h)^k dh:   J₁ = r²·(1 − π/4)     J₂ = r³·(5/3 − π/2)     J₃ = r⁴·(3 − 15π/16)
H_k = ∫₀^r h·t(h)^k dh: H₁ = r³/6             H₂ = r⁴/12
```

By `h = r·sin a`, `t = r·(1 − cos a)`, `dh = r·cos a·da` over `[0, π/2]`:
`J₁` from `∫(cos a − cos²a) = 1 − π/4`, `J₂` from `∫(cos a − 2cos²a + cos³a)
= 1 − π/2 + 2/3`, `J₃` from `∫(cos a − 3cos²a + 3cos³a − cos⁴a) = 1 − 3π/4 +
2 − 3π/16`; `H₁ = r³/2 − r³/3`, `H₂ = 3r⁴/4 − 2r⁴/3`. `π` enters through
`proofbound.TwoPiInterval`'s rational bracket; `r` is the interval
`[r − rDelta, r + rDelta]`, so a radius stated in inches charges its
conversion here and nowhere else in the volume.

### 5.3 Volume, first moment, area

```text
V_strip  = a₁·J₁ + a₂·J₂                                        (the wedge the band removes or fills)
M_strip  = (m₁·J₁ + m₂·J₂ + m₃·J₃,  sideZ·V_strip − m·(a₁·H₁ + a₂·H₂))   (in-plane, axial; m = matSign)
V_body   = V_receiver + σ·V_strip
M_body   = M_receiver + σ·M_strip
```

`V_receiver` and `M_receiver` are general-boolean §4.3's face terms over the
record RESTORED: `F` read over its region with `orig` in the contour's place,
each face beside the loop read over its untrimmed level `L_F` instead of
`sideZ` (`brepPayload.filletRestored`, a view), every other face as it is.
`measureBrepContext` therefore sums the restored faces, which carry no
contour displacement (SB1 holds `delta = 0` on the receiver), then adds
`σ·V_strip` and `σ·M_strip` per fillet band. The patches' own flux is never
integrated: `Σ_patches flux = V_body − Σ_rewritten faces = (F's strip term)
+ Σ(wall strips over [sideZ, L_F]) + σ·V_strip`, and the two strip terms are
exactly the restored-minus-rewritten difference. The centroid divides
`M_body` by `V_body` and lifts through the reference frame and placement as a
chamfered brep's does, with `capband.CentroidGeometryBound`'s ceiling.

Patch areas, the perimeter of the level loop being `a₁ + 2a₂·t` (the
derivative of `A(F ⊖ t)`), integrate over the quarter arc:

```text
Σ_patches area = a₁·(π/2)·r + a₂·(π − 2)·r²
per LF1 patch:  (π/2)·r·ℓᵢ − (κ₀ + κ₁)·r²·(π/2 − 1)
per LF2/LF3:    β·r·(R·π/2 ∓ r·(π/2 − 1))
per LF6:        ψ·r²·(π/2 − 1)
```

`Body.Area()` sums the rewritten faces (`F` with `sectionDisplacementArea`
over `F.delta`, the walls trimmed) and the patches. Each patch publishes its
own area with the bound of its coefficients' enclosure.

Exactness: `π` appears in every `J_k` and every patch area, so a loop
fillet's volume, area and centroid are never `Exact`; their bounds are the
enclosures' reach, a few ulps on an axis-aligned loop at a millimetre
radius. `F.delta` does not enter the volume or the first moment: the strip
model reads the receiver's recorded loop, which the fillet denotes, and the
rewritten faces enter only through `Body.Area()` and the per-face readings.

### 5.4 Extents and `Bounds`

A fillet patch bulges past the hull of its two directrices (a quarter
cylinder's farthest point along an oblique direction is interior), so
`brepPayload.extentAlong` adds each band's patches. Along unit direction `g`
in `F`'s frame, with `A` the component of `g` along the direction from
`sideZ` toward `F` (`−m·ẑ`) and `B` its component along the patch's in-plane
outward normal:

- LF1: `f(φ) = g_τ·s_end(φ) + r·(A·cos φ + B·sin φ)` over `φ ∈ [0, π/2]` with
  `s_end(φ) = ℓᵢ − κ·r·(1 − sin φ)` at the end the sign of `g_τ` selects,
  a sinusoid in `φ` whose extreme is an endpoint or the interior stationary
  point; the result is `Exact` only at an endpoint that is a recorded
  vertex.
- LF2/LF3/LF6: `f(φ, a) = (R ∓ r·(1 − sin φ))·ĝ_⊥·ρ̂(a) + r·A·cos φ` with the
  azimuth extreme `|g_⊥|` where `g_⊥`'s direction lies in the patch's
  azimuth window and the window's end otherwise, then the same sinusoid in
  `φ`.

Every stationary point is isolated (a sinusoid has one per quarter), so no
`ErrUnsupported` arises here. `ThroughAll`/`ToFace` read the same extents.

## 6. Table SF — refusals and gate order

Modify §1's test picks every sentinel.

| SF | Call | Exists? | Sentinel |
|---|---|---|---|
| **SF1** | a convex corner of the loop where a straight walk meets a circular one, or two circular walks meet, not tangent (LF8): a plate with a semicircular bite, a D-shaped boss rim | yes; its miter is a space curve no `Curve` names and its strip integrals are not polynomial | `ErrUnsupported`, naming the corner |
| **SF2** | `Face.NormalAt` at an LF6 apex vertex, which lies on the horn torus's own axis | the point exists; the surface normal there does not | `ErrDegenerate` (the survey samples interior points only) |

Everything else is an existing row: SL1 (a partial loop outside the
selected-chain route in `docs/vertex-blend-design.md`),
SL2 (LB3/LB4/LB6), SB1 (`delta ≠ 0`), SX6
(`R − r < 0` on an LF2 wall, a dropped carrier), SX7 (band reach, two bands
on one wall), SX12/SX14 (the contour at `r`), SX13 (`R ∓ r == R` or
`L_F ∓ r == L_F`), SX5 (revolve), SX9/SX10, S3. SL3 is retired: a fillet of
complete loops of planar faces builds here or refuses with a row above.

Gate order for a `Fillet` on a brep, stacked, or (RF3) prism receiver, after
modify §4's stage 1 and reach SX10:

| Stage | Gates |
|---|---|
| 2a. record | RB dispatch; SB2; SB1; RF3's `brepOfPrism` |
| 2b. route P | brep-modify §6 unchanged |
| 2c. entry | single straight edges → route E; complete loops with independent straight edges → route V; otherwise LB1, LB2 (SL1) |
| 3. topology | LB3, LB4, LB6 (SL2); LB5 (SX7) |
| 4. existence | SX6, SX13 per band as the contour at `r` is built; SF1 per corner |
| 5. audit | per band SX14, SX7, SX12 on the contour; per (pl) face S8, S6, S7, S9; per `F` S8, S7, S9 |
| 6. build | the record's closure; `attachFilletBand` per band |

## 7. Tables BF and DF — results and downstream

| BF | Call | Payload | Topology | Roles |
|---|---|---|---|---|
| **BF1** | route L or V fillet, any RF1–RF3 receiver | `brepPayload` with `loopBands` of kind `"fillet"`, `stack` nil | the rewritten record's faces plus one LF1–LF3 or LF9 patch per walk and one LF6 patch per reflex corner; every edge on exactly two faces | `face(k)` / `wall(k)`; each patch `filletLoop(f,l,p)` |

| DF | Consumer | BF1 |
|---|---|---|
| **DF1** | mass properties, area | §5.3 |
| **DF2** | `Bounds`, `ThroughAll`/`ToFace` | §5.4 |
| **DF3** | `Verify` validity, tolerance gate | by construction; the bands' vertices are body vertices; `brepGateDiameter` reads every vertex |
| **DF4** | `Tessellate`, STL/OBJ/3MF | §7.1 |
| **DF5** | mesh boolean operand | admitted for every band: every ring of §7.1 lies on the denoted surface up to `F.delta` and interpolation rounding, so the slice-wise argument of tessellation-reach §7 holds with the per-vertex store as the vertex-motion allowance and §7.1's sagitta terms as the chord term. `capband.OccupiedVolumeAdmission`'s reflex refusal does not apply: the LF6 fan's stations are the connector arc's azimuths at every ring |
| **DF6** | clearance | no model: the pair reads `Suspect` unless its boxes decide it (DG6); a later increment may add the LF1 cylinder carrier, whose axis is in-plane |
| **DF7** | undercut survey | per patch through `Face.NormalAt`: the normal of an LF1 patch sweeps the quarter from `n̂_F` to the wall's outward normal, an LF2/LF3 patch the quarter at every azimuth of its window, an LF6 patch the quarter at every azimuth of `ψ` short of the apex; the component range along the pull is the closed-form range of `A·cos φ + B·sin φ` over the window, read with reach DX7's three-valued rule and its arithmetic allowance alone, since the stamped departure is zero |
| **DF8** | concave-radius survey | a band with `σ = +1` (a fill) contributes `r` exactly, the tag's `Minor`/`Radius`, which a placement leaves unchanged; a band with `σ = −1` is convex in its tube direction and contributes nothing; an LF6 patch's other principal radius is `ρ/sin φ`, which vanishes at the apex where the receiver's own sharp reflex edge already reads as `brepMinRadius` reads it today (a sharp corner enters no aggregate), so it adds nothing. `brepMinRadius` drops its `capPatchWindowSkew`/`normalBound` refusal for fillet bands, keeps it for chamfer bands |
| **DF9** | wall survey | staged `Suspect`, as for every brep |
| **DF10** | STEP | analytic: `supportsAnalyticSTEP` takes a whole-turn torus (LF7), a four-arc torus patch (LF2/LF3) and a straight-walk cylinder patch closed by `Ellipse3` edges (`docs/step-export-design.md`). A horn torus patch (LF6, a three-edge loop closing on the axis) keeps the faceted writer. `Body.Tessellate` admits a banded body (F-2), so `export.STEP` reaches the writer |
| **DF11** | a later `Fillet`/`Chamfer`/`Shell` | DG11: Tables RB/EB/SB/TC/LB over the rewritten record; a patch's edge is in no record and matches nothing (SL1); a trimmed wall's rim at `sideZ` ends on a patch, so route E's SB7 refuses an edge there; a record carrying bands reads as no prism (route P and Table TC refuse); a second loop fillet or chamfer on another loop appends to `loopBands`, provided the first left `delta = 0` |
| **DF12** | `Placed`, `Mirrored`, `PatternCopies` | re-lifts every face frame; `loopBands` re-attach on re-evaluation; a reflection flips `outward` and the ellipse's `Axis` sense with every winding (general-boolean A4) |

### 7.1 Tessellation

A fillet band is meshed as `n_φ` strips between `n_φ + 1` rings. Ring `0` is
the side contour (the trimmed wall's rim at `sideZ`, shared), ring `n_φ` the
cap contour (`F`'s new loop, shared with `F`'s triangulation). Ring `k` is
the loop offset by `t_k = r·(1 − cos φ_k)` at height `h_k = r·sin φ_k` from
`sideZ` toward `F`, `φ_k = k·(π/2)/n_φ`, and its vertices are the AFFINE
interpolation of the side ring's and cap ring's vertices at matched
azimuths:

```text
ring_k[j] = side[j] + (t_k / r)·(cap[j] − side[j])   in (u, v), at z = sideZ − m·h_k
```

This is exact for every admitted piece: a line's offset is affine in `t`, a
circle's foot moves along its radius, a line–line miter corner moves along
the bisector, a G1 foot along the shared normal, and a reflex connector's
ring is the arc of radius `t_k` about the apex (`side[j]` is the apex for
every `j` of the connector). So every ring vertex lies on the denoted
surface within `F.delta·t_k/r ≤ F.delta` plus the interpolation's rounding,
which the vertex store carries.

Counts: one count per walk shared by the trimmed wall, every ring and `F`'s
contour (`chordCapBlendLoop`, with `n(w) = max(chordCount(w),
chordCount(capArc(w)))` so the larger of the side and cap radii sets the
in-plane sagitta); `n_φ = ⌈(π/2) / (2·acos(1 − tol_φ/r))⌉` with `tol_φ` half
the chord budget, at least 2. An LF4 corner's ring vertices
are the ellipse's chord chain, shared by both cylinders; an LF5 join's are the
quarter circle's. Each strip between rings `k` and `k+1` is a quad per
azimuth interval with a fixed diagonal; along a straight walk the quad is a
planar trapezoid (two parallel chords).

Occupied-volume proof (general-boolean §4.4's structure, per band): the
true surface and the strip mesh correspond by `(azimuth, φ)`, and every
mesh point lies within `s_φ + s_ring + store` of its surface point, where
`s_φ = r·(1 − cos(Δφ/2))` is the quarter-arc chord sagitta, `s_ring` the
in-plane chord sagitta of the ring's arc at the larger radius (zero on a
straight walk), and `store` the ring vertex's bound. The band's symmetric
difference is at most `Σ_strips area(strip)·(s_φ + s_ring + store)`, summed
into `volSymDiff` beside the record faces' own terms. The strip's quads
are triangulated, so each cell's twist (`proofbound.CellTwistOffsetUpper`)
joins `s_φ + s_ring`; `store` enters through the per-vertex motion that
`SweptVolumeAllow` reads, where an interior ring vertex carries its two ends'
motion plus its own interpolation error (the interval error of `1 − cos φ_k`
times the end-to-end span, the exact rounding of the interpolation, and the
error of its level). A reflex connector's cap sample carries its station
bound, the radius's conversion bound and `F.delta`. The area slack a patch
adds is the measured gap between its held facets' area, enclosed in exact
rational arithmetic, and the patch's published area with its bound. The `Bound` a patch
face publishes on the mesh is the same sum per patch. Both terms are zero
at `n_φ → ∞` only, so a banded mesh is never `Exact`.

## 8. Reference fixtures

Every closed form below is `V_receiver + σ·(a₁·J₁ + a₂·J₂)` with Table CF's
coefficients, `J₁ = r²(1 − π/4)`, `J₂ = r³(5/3 − π/2)`. Each test asserts the
published volume's interval encloses it (`big.Rat` with a `π` bracket), the
patch count and kinds, the edge kinds, and the roles, through the public
API; a bound is asserted as a relation, never a literal.

| Part | Call | `a₁` | `a₂` | `σ` | Volume |
|---|---|---|---|---|---|
| **P1** 40×20×20, Ø6 along y | top loop, `r = 2` | 120 | −4 | −1 | `16000 − 180π − (1280/3 − 104π) = 46720/3 − 76π`; four `Cylinder` patches, four `Ellipse3` edges with `SemiMajor 2√2`, `SemiMinor 2`, centres `(2, 2, 18)` and its images; both x walls (sw) and both y walls (pl) trimmed to `z = 18` |
| | top and bottom loops, one call | 2 × above | | | `45440/3 + 28π` |
| | hole rim, `r = 1` (one y wall's hole loop, R = 3) | `6π` | `+π` | −1 | `16000 − 180π − ((23/3)π − 2π²) = 16000 − (563/3)π + 2π²`; one whole-turn `Torus{Axis: ŷ, Major: 4, Minor: 1}`, two `Circle3` edges: radius 4 on the wall face, radius 3 inside the hole at `y = 1` |
| **P2** 40×40×10, blind 20×10 pocket 5 deep (stacked) | pocket mouth, `r = 1.5` | 60 | `+π` (four LF6) | −1 | `15000 − (135 − 225π/8 − 27π²/16) = 14865 + 225π/8 + 27π²/16`; four `Cylinder` and four horn `Torus{Major: 1.5, Minor: 1.5}` patches, eight `Arc3` meridians, every pocket wall's level `z = 8.5` |
| | plate top loop, `r = 1.5` | 160 | −4 | −1 | `15000 − (337.5 − 333π/4) = 14662.5 + 333π/4` |
| | both in one call | | | | `14527.5 + 891π/8 + 27π²/16`; S7 passes (the contours sit 7 mm apart) |
| | pocket floor loop, `r = 1.5` (a fill) | 60 | −4 | +1 | `15000 + 112.5 − 27π`; `sideZ = 6.5`; four concave cylinders meeting along `Ellipse3` edges whose `IsConvex` is false; DF8 reads `1.5` exactly |
| **P3** 40×40×10 ∪ Ø10 boss 15 tall (A1) | boss root, `r = 1` (a fill) | `10π` | `+π` | +1 | `16000 + 375π + (35/3)π − 3π² = 16000 + (1160/3)π − 3π²`; Pappus check `2π[5(1 − π/4) + (5/3 − π/2)/2]`; `Torus{Center: (20,20,10), Major: 6, Minor: 1}`, cap circle radius 6 on the plate, side circle radius 5 at `z = 11` |
| | boss rim, `r = 1` | `10π` | `−π` | −1 | `16000 + 375π − ((25/3)π − 2π²) = 16000 + (1100/3)π + 2π²`; `Torus{Major: 4, Minor: 1}`, the boss top shrunk to radius 4 |
| | both | | | | `16000 + (1135/3)π − π²` |
| **P4** 60×40×8, four Ø5 holes (prism, RF3) | four hole mouths, `r = 1` | `5π` each | `+π` each | −1 | `19200 − 200π − 4((20/3)π − (7/4)π²) = 19200 − (680/3)π + 7π²`; four whole-turn tori `Major 3.5`; the result is a brep body (BF1) |
| | top loop, `r = 1` | 200 | −4 | −1 | `19200 − 200π − (580/3 − 48π) = 57020/3 − 152π` |
| **P6c** 60×40×30, blind 20×10 port 13 deep into one x wall | port mouth, `r = 1.5` | 60 | `+π` | −1 | `69400 − 135 + 225π/8 + 27π²/16` (P2's mouth on a wall face) |
| **P7** L bracket, Ø6 along x through the upright leg | top loop, `r = 1` | 160 | `−5 + π/4` | −1 | `17280 − 72π − (455/3 − (445/12)π − π²/8) = 51385/3 − (419/12)π + π²/8`; five `Ellipse3` edges and one horn torus |
| | one hole rim, `r = 1` (R = 3, on the upright leg's `x = 0` face) | `6π` | `+π` | −1 | `17280 − 72π − ((23/3)π − 2π²) = 17280 − (239/3)π + 2π²`; `Torus{Center: the hole centre on the face, Axis: x̂, Major: 4, Minor: 1}` |
| **P8** 40×20×20, four vertical edges filleted `r = 3`, Ø6 along y | top loop, `r = 2` | `96 + 6π` | `−π` (four LF2 at `β = π/2`) | −1 | `15280 − (384 − (256/3)π − 2π²) = 14896 + (256/3)π + 2π²`; four `Cylinder` and four `Torus{Major: 1, Minor: 2}` (spindle) patches joined by eight `Arc3` quarter meridians, no ellipse |

Bound fixture, shown to fail first: the trapezoid `(0, 0)`, `(100, 0)`,
`(72, 45)`, `(28, 45)` extruded 10 with a blind 20×20 pocket 5 deep (a
stacked receiver of 30400, route L's own fixture), top loop filleted `r = 1`. Its slanted sides have
length 53 and `cos θ = 28/53`, `sin θ = 45/53` at the base corners, so
`κ = cot(θ/2) = 9/5` there and `tan(θ/2) = 5/9` at the top corners:
`a₁ = 250`, `a₂ = −212/45`, and the volume `V_receiver − (250(1 − π/4) −
(212/45)(5/3 − π/2))` is an exact rational-`π` closed form the test
encloses. Legs recorded in the test: (1) every cap-level vertex's bound
encloses its exact rational contour corner, and deleting the `F.delta`
charge turns all four corners red (route L's mechanism); (2) the centroid of
the body placed `10⁶` mm along every axis encloses the placed exact centroid,
and deleting the frame-lift rounding term of the centroid turns it red
(reach §8.4's placed-plate mechanism). The centroid's x is exactly 50, which
lifts to `10⁶ + 50` with no rounding, so a placement along x alone leaves the
term unexercised. The volume's own bound
is the enclosures' reach alone and cannot be made to fail by deleting one
term, which the test records.

Refusals: a plate with a semicircular bite on its outer loop, top loop →
SF1 naming the two non-tangent corners; a partial loop outside the
selected-chain route → SL1;
P8's top loop at `r = 4` → SX6; the P2 mouth at `r = 5` → SX7;
a partial revolve's cap loop → SX5; a stacked
receiver whose section carries a displacement → SB1. Every refusal leaves the
receiver live. A full revolve's circles are meridian junctions, which the
revolve route rounds (reach §7), so no full-turn revolve fixture refuses here.

## 9. Decided questions

- **No sphere at a convex loop corner; the two pipes meet along an ellipse.**
  §2 states the derivation: the rolling ball never rests at the corner, and
  the octant would expose a shelf. Reach §8.2's miter row ("zero-length
  miter → trimmed `Sphere`") and modify-general §6's first two rows and SL3
  are rewritten in PR F-1 to say so; an LF9 sphere needs the third edge's
  fillet (`docs/vertex-blend-design.md`).
- **A new `Ellipse3` curve variant, rather than a `Line3`, `NURBSCurve` or
  `FacetedCurve` stand-in.** The miter is an exact conic; a `Line3` tag would
  be a first-order lie (sagitta up to `0.2·r`), a `NURBSCurve` would hold an
  irrational weight, and a `FacetedCurve` says the body holds chords where it
  holds a cylinder. api §4 is extended in F-0.
- **Refuse non-tangent convex corners at circular walks (SF1).** No surveyed
  part has one; the miter is a non-planar space curve and the strip integrals
  lose their polynomial form. A later increment can enclose both.
- **Measure through the receiver's restored faces plus the strip integral,
  not through patch flux.** §5.3: no new surface integral, no `π`-phase
  Fourier sums, no chord-versus-locus term, and no `F.delta` in the volume.
- **A prism receiver's loop fillet returns a brep body.** RF3: one band
  record, one reader set; its roles are BF1's, not B1's or BX3's. A caller
  reading `capStart` after a cap-loop fillet reads `face(k)` instead.
- **Rings interpolate affinely between the side and cap rings.** §7.1: every
  admitted piece's offset is affine in `t`, so no per-ring offset solve and
  one displacement per band.
- **STEP stays faceted until `step` ships `TOROIDAL_SURFACE` and `ELLIPSE`.**
  DF10, §12.
- **Restored faces keep the rewritten record's displacements.**
  `measureBrepContext` reads each face a fillet band rewrote restored to the
  receiver's level and region (`filletRestored`), but with the level and
  section displacements the rewritten record states, which are never less
  than the receiver's: sound, and zero for every fixture of §8.
- **SF1 also refuses a corner whose exact class disagrees with the
  contour's join.** The class is the exact turn of the recorded walks; a
  right turn the cap contour does not close with a connector arc, or a left
  or tangent turn it reads otherwise, is no corner of Table LF.
- **`Bounds` reads the band's extents too.** §5.4's extents are added to the
  box as to the through-all extent, since a patch bulges past its
  directrices along an oblique world axis under a placement.
- **The surveys read the patches in closed form.** The undercut survey
  decides each patch from the range of `cos φ·A + sin φ·B(θ)`, with `A` from
  `F`'s frame normal and `B` from the inward normal of `F`'s region along the
  patch's walk or through a reflex corner's turn (Table DF's DF7). Both are
  nondecreasing in `A` and `B`, so their ends bound the whole patch, and the
  three-valued rule of reach DX7 reads them with no departure term.
- **Hand-offs: `step` only.** No 2D answer beyond route L's own offset is
  asked; `sketch`, `r3` and `units` need nothing.

## 10. Do not do this

- **Put a sphere octant at a convex corner.** §2: a shelf of area
  `r²(1 − π/4)` at the side level. An LF9 sphere needs the third edge's
  fillet (`docs/vertex-blend-design.md`).
- **Tag the miter edge `Line3` with a length bound, as the chamfer's ruling
  is.** The chamfer's ruling departs from its conic locus at second order;
  the fillet's miter departs from a chord at first order.
- **Rule the patch between its two directrices and tag it `Cylinder`.** The
  chamfer's `Cone` stand-in needed reach §8.3's skew, departure and locus
  terms; the fillet builds the exact pipe and owes none of them.
- **Integrate patch flux with trigonometric endpoint terms.** §5.3's strip
  model is polynomial in `t`; the only transcendental is `π`.
- **Compute `V_body` from the rewritten record's faces plus a closed band
  region.** The restored record plus `σ·V_strip` is the same number with no
  displacement charge and no disk bookkeeping.
- **Solve each tessellation ring's offset contour separately.** §7.1's
  affine interpolation is exact for every admitted piece and carries one
  displacement.
- **Admit a line–circle convex corner by chording its miter.** SF1; a chord
  chain is a `FacetedCurve`, and the strip area is not a polynomial.
- **Read `F.delta` into the volume bound.** The strip model reads the
  recorded loop; `F.delta` belongs to `F`'s own area and the cap-level
  vertices.
- **Decide SF1 by comparing float tangents.** `capband.JoinIsG1` is exact
  rational arithmetic; a float-parallel pair that is not exactly tangent is
  SF1, as `OccupiedVolumeAdmission` already rules.

## 11. PR split

Each PR ships its code, tests and the documentation its lifted refusals
touch: this document's increment table, `doc.go`'s support map,
`docs/missing-features.md`'s Modify rows, the design-doc rows §9 names, the
functions' doc comments, a `docs/layout/` row per new root file, and
`.github/test-shards.txt`. This document ships with F-0.

| PR | Model | Lands | Files and functions | Proves | After |
|---|---|---|---|---|---|
| **F-0** | Sonnet, file-by-file | `Ellipse3` (api §4; `surfacegeom/geometry.go`, `placement.go`, `topology.go` alias, `selectedEdgeContext`, `brepEdgeMatches` default); `brepLoopBand.kind`; `brepLoopRoute`'s fillet arm as a stub keeping SL3's refusal; `fillet.go`'s cap-edge arm calling `brepOfPrism` into the stub | `internal/surfacegeom/`, `topology.go`, `fillet.go`, `brep_loop_band.go`, `brep_modify_loop.go`, `docs/api-design.md` | every existing fillet, chamfer, brep-modify and route L fixture bit for bit; an `Ellipse3` placed and mirrored | — |
| **F-1** | Opus, proof spec | the fillet arm: LF classification (SF1), `attachFilletBand` (LF1–LF7 faces, `Ellipse3`/`Arc3`/`Circle3` edges, shared vertices, roles), `internal/filletband/` (Table CF coefficients with interval enclosures, `J_k`/`H_k`, patch areas, extents), `measureBrepContext`'s restored-face sum plus `σ·V_strip`/`σ·M_strip`, `extentAlong`; tessellation, surveys and the boolean operand refuse a fillet-banded body (`ErrUnsupported`, naming F-2); reach §8.2, modify-general §6/SL3, brep-modify SB4/SB5/SB7 text, `missing-features.md`, `doc.go` | `brep_loop_fillet.go` (new), `brep_loop_band.go`, `brep_measure.go`, `brep_payload.go`, `brep_modify_loop.go`, `fillet.go`, `internal/filletband/` (new), `brep_loop_fillet_internal_test.go`, `apitest/brep_loop_fillet_test.go` | §8's volumes, areas, centroids, topology and roles; the bound fixture's two legs; every refusal | F-0 |
| **F-2** (landed) | Sonnet, file-by-file, copying `tessellate_brep_band.go` with §7.1 as its term table | DF4 rings and the proof terms, DF5 admission, DF7/DF8 surveys, DF12 re-attachment tests, DF10 export confirmed | `tessellate_brep_band.go`, `tessellate_brep_fillet.go` (new), `tessellate_brep.go`, `brep_loop_band.go` (admission), `brep_measure.go` and `internal/survey2d/fillet_pull.go` (surveys), tests | every §8 fixture's mesh closed and within its published bound; P2's floor loop reads `1.5` in the concave-radius survey; P3's root is listed by the undercut survey under a pull along `−z`; a `Cut` by a box over P1's filleted corner builds through the mesh path | F-1 |
| **F-3** (landed) | Sonnet, file-by-file | analytic STEP: `TOROIDAL_SURFACE` faces bounded by `Circle3`/`Arc3` edges, `ELLIPSE` edges on partial cylinders, `supportsAnalyticSTEP` arms | `export/step_analytic.go`, `export/step_analytic_torus.go`, `export/step_analytic_band_internal_test.go` | P3's root and rim and P8's top loop export analytic STEP; P1's top loop exports analytic STEP with four `ELLIPSE` edges | F-1, and the `step` module release of §12 |

F-2 and F-3 run in parallel after F-1: they share no file. F-0 runs alone;
F-1 runs alone.

Increment table — what still refuses after each PR:

| After | Still refused |
|---|---|
| F-0 (landed) | everything Table SB and SL3 refuse today |
| F-1 (landed) | every consumer of a fillet-banded body but mass properties, `Bounds`, `Verify`'s structural audit and gate, placement and later modify ops (F-2); SF1; SB5; SX5; SX4 |
| F-2 (landed) | SF1; SB5 (single edges sharing a vertex outside route V); SX5 (revolve cap edges); SX4 (partial loops); clearance model (DF6, `Suspect`) |
| F-3 (landed) | SF1; SB5; SX5; SX4; DF6; variable-radius fillets (no entry point) |

## 12. Hand-off

`github.com/lestrrat-3d/step` holds no `TOROIDAL_SURFACE`, `SPHERICAL_SURFACE`
or `ELLIPSE` constructor (`ap214/geometry.go` ends at `Line`). The hand-off
file `../step/.tmp/decad-handoff-loop-fillet.md` asks for
`ap214.ToroidalSurface(id, name, position, majorRadius, minorRadius)`,
`ap214.SphericalSurface(id, name, position, radius)` and
`ap214.Ellipse(id, name, position, semiAxis1, semiAxis2)`, with the AP214
attribute order and the decad faces that use them. PR F-3 consumes it.
