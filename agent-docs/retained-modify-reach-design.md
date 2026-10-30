# Retained Modify Reach Design

What the modify operations and placements do with a mesh Boolean result whose faces carry retained analytic
surfaces (`agent-docs/faceted-source-retention-design.md`, "retention §N"). Companion to
`agent-docs/retained-edge-blend-design.md` ("blend §N"), which owns Table RR, the blend construction and Table SR;
`agent-docs/modify-design.md` ("modify §N"), which owns the call contracts and base gates; and
`agent-docs/modify-reach-design.md` ("reach §N"), which owns tangent-chain expansion (§5), the asymmetric
reference (§6) and SX9. This document adds receiver classes and refusal rows to those tables. It changes no
prism, revolve or brep receiver.

Navigation only; the sections own the content.

| Question | Section |
|---|---|
| What each call does on a mesh Boolean result today | §1.1 |
| Which gaps build and which stay refused | §3 |
| Two-distance chamfer of a retained edge | §4 |
| Fillet or chamfer of a rounded outline (line, arc, line …) | §5 |
| `Copy` with `WithCopyRotation` | §6 |
| `ScaledAt` | §7 |
| Edge between two parallel cylinders | §8 |
| Why `Shell`, `Draft`, `Offset`, `PatternJoin`, cone and torus blends stay refused | §9 |
| Refusal rows and gate order | §10 |
| Tests and their red legs | §12 |
| PR order | §14 |

## 1. Probe

The fixture is retention §1's: a 40×20×20 box with its four z-parallel edges filleted at r=3, cut by a Ø8 tool
along y. The tool is unplaced (P1), translated by a `Transform` (P2), or translated and then rotated 20° about z
(P3). P1's Cut is a `brepPayload` (class B). P2 and P3 are mesh Boolean results whose faces read 6 `Plane` and
5 `Cylinder`, and whose edges publish `Line3`, `Arc3` and `Circle3` (retention §10). The **x-parallel edges** are
the four edges along x at `y ∈ {0, 20}`, `z ∈ {0, 20}`; the top front one runs from `(3,0,20)` to `(37,0,20)`.

### 1.1 Table P — calls on a mesh Boolean result

P2 and P3 give the same result on every row (probe on `main` at e36f26819, 2026-10-26; dated illustration).
Every refusal is `ErrUnsupported`; the quoted text follows `decad: not supported by the current evaluator:`.
"Kinds" counts the result's faces by `Face.Surface()` kind.

| Call on P2/P3 | Result | Kinds | Closed by, or why refused |
|---|---|---|---|
| `Hole` blind on the top face | builds | 7 `Plane`, 6 `Cylinder` | retention |
| `PatternCopies`, linear and circular | builds | 6 `Plane`, 5 `Cylinder` | retention |
| `Mirrored`, `MirroredCopy` across the +x face | builds | 6 `Plane`, 5 `Cylinder` | retention |
| `Transform`, translation or 20° about z | builds | 6 `Plane`, 5 `Cylinder` | retention |
| `Copy`, no option or `WithCopyPlacement` | builds | 6 `Plane`, 5 `Cylinder` | retention |
| `Copy` with `WithCopyRotation`, 20° about z | builds | 6 `Plane`, 5 `Cylinder` | GAP-A; M2 (§6) |
| `ScaledAt` 1.5 about `(20,10,0)` | builds | 6 `Plane`, 5 `Cylinder` | M5 (§7) |
| `Fillet` r=1 or `Chamfer` d=1 of one or all four x-parallel edges | refused (SM2) | — | GAP-E; §5.3, §15 |
| `Chamfer` of the four, `WithAsymmetricChamfer` 1 and 2 | refused (SM2) | — | as above |
| `Fillet` r=1 of the four, `WithTangentChain` | builds two chains | 6 `Plane`, 13 `Cylinder`, 8 `Torus` | M3 (§5) |
| `Chamfer` d=1 of the four, `WithTangentChain` | builds two chains | 14 `Plane`, 5 `Cylinder`, 8 `Cone` | M3 (§5) |
| `Chamfer` of the four, asymmetric, `WithTangentChain`, reference `Faces(Facing(+z)).Or(Facing(-z))` | builds two chains | 8 `Plane`, 8 `Cone` | M3 (§5.6) |
| `Chamfer` of the four, asymmetric, `WithTangentChain`, reference `Facing(+z)` | refused (text A) | — | §5.6 |
| `Shell` inward, top face removed | refused (text S) | — | GAP-B; §9.1 |
| `Draft` 2° of the walls about the bottom face | refused (SD23) | — | GAP-C; §9.2 |
| `Offset` | refused (text O) | — | §9.3 |
| `PatternJoin` of a linear pattern of a feature | refused (text J) | — | GAP-F; §9.4 |

The refusal texts are these.

- **SM2**: "the selected edge ends at the tangent junction 37,0,20 whose continuation is not selected;
  WithTangentChain selects the whole chain" (`internal/retainedblend/chain.go`).
- **Text A**: SX3, "names 0 of the faces adjacent to selected edge" (reach §6), for the chain on the bottom plane.
- **Text S**: "this evaluator shells a straight prism or its one-line Sweep reduction only"
  (`internal/brepmodify/shell.go`).
- **SD23**: "this evaluator drafts a straight prism only (an extrude, its filleted or chamfered result, a tube, or
  any of these placed) (draft SD23)".
- **Text O**: "Offset requires a sheet body" (`offset.go`).
- **Text J**: "Union of three or more bodies needs a certified prism, axial or copied-tooth set; join them two at
  a time" (`boolean_many.go`).

The asymmetric chain rows differ in the reference. `Faces(Facing(+z)).Or(Facing(-z))` holds both base planes, the
top and the bottom face. Reach SX3 admits it, because each expanded edge borders exactly one of the two. The four
seeds expand to two closed chains, one on each base plane, and each builds with its own base as reference (§5.6).
A bare `Facing(+z)` names no face adjacent to the edges of the bottom chain, so SX3 refuses with text A.

The probe calls neither the two-distance chamfer of a single edge (GAP-D, M1, §4) nor the parallel-cylinder blend
(M6, §8); §12's fixtures cover both.

On P1 the brep routes answer instead. A plain `Fillet` or `Chamfer` of an x-parallel edge refuses brep-modify SB7,
because the face across the edge's axis at `(3,0,20)` is the curved wall. All three `WithTangentChain` rows build,
the asymmetric one included. `Shell` builds (11 `Plane`, 10 `Cylinder`). `ScaledAt` refuses "scale needs an exact
line-section stack BRep".

### 1.2 The x-parallel edge

At each end the x-parallel top edge continues tangentially into an `Arc3` of radius 3 between the top face and a
lateral blend cylinder, because the front face is tangent to that cylinder along the generator `x = 3, y = 0`. The
edge has no end face: it is one edge of the top face's closed outline of 4 lines and 4 arcs. The same outline on
the box before the Cut takes reach §8's complete cap-loop blend.

## 2. What the design reads

| Input | Source | Used for |
|---|---|---|
| Retained tags with `NormalBound == 0` and no denoted patch, or a revolve patch whose tag is exactly its denoted surface (blend §2, and blend §4.5 for a sphere) | retention §3 | every exact surface in §4, §5, §8 |
| Retained edge curves (`Line3`, `Arc3`, `Circle3`) | retention §10 | edge classes and chain topology |
| Held vertices, facets and vertex bounds | retention §3.3, faceted-vertex-bounds §2 | rewrite and clearance gates |
| `brepmodify.Asymmetric.Refs` | reach §6, `modify_options.go` `resolveAsymmetricReference` | which face takes which distance (§4) |
| `exactrotation.Rotation` (`Linear`, `Translation`) | `document.go` `copyUnder` | the exact image of a tag (§6) |
| Anchor and factor of `ScaledAt` | `point_plane_scale.go` | the exact similar image of a tag (§7) |

Faces with a stated departure, or a denoted patch whose tag is not exactly the surface it denotes, are not read by §4, §5 or §8, for blend §2's reason. A Revolve `Plane` or `Cylinder` face whose tag is exactly its denoted surface is read as a retained tag (blend §2), and a Revolve `Sphere` whose tag is exactly its record's centre and radius is read by the plane-sphere rim route R6 (blend §4.5).

## 3. Verdicts

| Gap | Verdict | Reason in one clause | Section |
|---|---|---|---|
| GAP-D | build | the blend construction is already two-distance; only the reference mapping is missing | §4 |
| GAP-E | build | at an exact tangent junction the two neighbouring blends share one exact cross-section (Claim J) | §5 |
| GAP-A | build | the exact rotation is exactly orthogonal, so the tag's image costs only its rounding | §6 |
| `ScaledAt` | build | a uniform scale keeps every normal, so the scaled tag costs only its origin and radius rounding | §7 |
| Cylinder/Cylinder, parallel axes | build | the blend is a `Cylinder` from a circle–circle cross-section, as blend §4.3 | §8 |
| GAP-B (`Shell`) | refuse | the cavity needs the trims between offset faces, which a mesh result does not record | §9.1 |
| GAP-C (`Draft`) | refuse | each drafted face must be re-trimmed against its neighbours and the tool, with the same missing trims | §9.2 |
| `Offset` of a solid | refuse | surface-design R37 refuses every solid; not a retention matter | §9.3 |
| GAP-F (`PatternJoin`) | refuse here | the refusal is the many-body Union rule, which retention does not touch | §9.4 |
| Cylinder/Cylinder, crossing axes, `Fillet` outside blend R7 | refuse | the blend is a canal surface outside the surface vocabulary | §9.5 |
| Cylinder/Cylinder, a bore crossing a larger shaft at right angles, `Chamfer` | build (blend R7) | a proven band-shaped tool cuts the exact equal-setback wedge through the mesh Boolean | blend §4.9 |
| Cylinder/Cylinder, a bore crossing a larger shaft at right angles, `Fillet` of the convex mouth | build (blend R7 fillet) | both walls are cut by exact planes near the mouth and joined by a faceted pipe whose chord error is proven cell by cell | blend §4.10 |
| `Cone`, `Torus` faces | refuse | the admitted sources carry denoted interval surfaces, which this route does not read | §9.5 |
| Plane/`Sphere` rim | build (blend R6) | the meridian blend is R3's line-circle section revolved, and the sphere's record reads as exact points | blend §4.5 |

## 4. Two-distance chamfer of a retained edge (GAP-D)

`retainedblend` already builds every Table RR chamfer from two setbacks: `blendSpec.dA` along the plane face (R2,
R3, R4) or face `A` (R1), and `blendSpec.dB` along the wall or face `B`. The equal chamfer sets both to `d`. This
section passes reach §6's two distances through instead.

### 4.1 Mapping

SX3 has resolved, for each selected edge `e`, exactly one adjacent reference face `Refs[e]`. Let `F1`, `F2` be the
two faces in the order the class reader names them (`A`/`B` for R1, plane/wall for R2–R4).

| Reference | Setback on `F1` | Setback on `F2` |
|---|---|---|
| `Refs[e] == F1` | `D` (the positional distance) | `Other` |
| `Refs[e] == F2` | `Other` | `D` |

**Resolve each reference to a tag before the first step.** Edges after the first are found again on the evolving
mesh (blend §7), whose face pointers are new. Record, per target, the retained tag of its reference face. Each step
picks the face whose tag equals that tag bit for bit; the two faces of every RR edge carry distinct tags.

R3's chamfer reads `dA` for both setbacks (`internal/retainedblend/cyl_rewrite.go`, `solveSection`). It must read
`dA` along the plane line `L` and arc length `dB` along the wall circle, so `θ_T = dB / R`.

### 4.2 Geometry and proof

No new claim. R1's chamfer plane runs through two parallel setback lines at in-plane distances `dA` and `dB`; R2
and R4 take the `Cone` through the circle `R + σ·dA` on the plane and the circle at distance `dB` along the wall
(blend §4.2); R3 takes the plane through the generator at distance `dA` along `L` and the one at arc length `dB`
along `C`. Blend §5's exact areas and Claim K already state every term for `dA ≠ dB`. Each face's clearance
(blend §6.2) uses that face's own setback.

`DDelta` and `OtherDelta` must both be zero, for blend §6.1's existing "converts exactly to millimetres" rule.
A tangent chain (§5) takes one assignment for the whole chain; SX3 forces it (§5.6).

## 5. Tangent chains on one base plane (GAP-E)

### 5.1 Table RC — chain classes

A **chain** is a run of selected edges `e_1 … e_n` that all lie on one retained plane face `P` (the **base**),
where each `e_k` is an R1 edge between `P` and a retained plane wall `W_k`, or an R4 edge between `P` and a
retained cylinder wall `W_k` whose axis is normal to `P`, and consecutive edges meet at a **tangent junction**
(§5.2).

| RC | Chain | Ends | Staged |
|---|---|---|---|
| **C1** | closed: every edge of one loop of `P` | none | increment M3 |
| **C2** | open | each end at a three-face corner of a straight R1 edge, on an end plane `G` normal to the edge where the base and the wall are convex and `G` is convex or reflex (§5.5 step 5), or of a convex chain's R4 arc that blend §4.4 admits, on an end plane parallel to its axis and convex at the corner | increments M4, M4b, M8 |

At a junction both walls are perpendicular to `P`: a cylinder wall's axis is normal to `P`, and a plane wall
tangent to it along a generator contains that generator. So every edge of a chain has dihedral angle `π/2`, and
the whole chain is convex or the whole chain is concave, and `n_g = s·m` at every junction. Here `m = n_P × d`
points into `P` along the walk direction `d` (a face lies to the left of its loop), and the chain's **side** `s`
is `−1` for a convex chain, whose blends cut into the material, and `+1` for a concave one, whose blends fill the
void. A line edge reads `s` from its wall's outward normal: `n_W = −m` gives `−1` and `n_W = +m` gives `+1`; the
two candidates are a unit vector apart, so the float test only classifies, and the edge's own convexity flag must
agree. An arc edge takes `s` from the rim's side (blend §4.2), which `readSide` ties to the edge's convexity. A
chain whose edges read different sides is SM3; at a junction test 3 of §5.2 makes the walls' normals along `g`
equal, so public input does not reach it.

### 5.2 Tangent junction

Edges `e_k` and `e_{k+1}` meet at a tangent junction at vertex `v` when all of these hold.

1. Their faces are `{P, W_k}` and `{P, W_{k+1}}` with `W_k ≠ W_{k+1}`.
2. `W_k` and `W_{k+1}` are tangent along one line `g` normal to `P`, decided by an exact identity on the tags'
   floats read as rationals:
   - plane `(n, o)` and cylinder `(a, c, R)`: `a · n = 0` and `(n · (c − o))² = R²·(n · n)`;
   - cylinder `(a1, c1, R1)` and cylinder `(a2, c2, R2)`: `a1 × a2 = 0` and
     `|(c2 − c1)_⊥|² = (R1 ± R2)²`, with `_⊥` the part normal to `a1`, `+` for external and `−` for internal
     tangency;
   - two plane walls are never a junction;
   - every cylinder wall's axis is normal to `P` exactly: its cross product with `P`'s frame normal is zero.
3. The two walls' outward normals along `g` are equal: their exact dot product is positive (the sign test of
   reach §5.1).
4. `V = g ∩ P` is exact. `g` is the line parallel to the axis through the foot of the axis on the plane wall,
   `c − n·(n · (c − o)) / (n · n)`, or for two cylinders through `c1 + R1·(c2 − c1)_⊥ / (R1 ± R2)`. Both are
   rational once identity 2 holds, and `P` meets `g` in one rational point.
5. Falsifier: the held `v` lies within `bound(v) + slack` of `V` (blend §6.3's slack). A larger distance disproves
   the pairing and refuses (SR6). A smaller one proves nothing.

Identity 2 is the oracle of reach §5.1 applied to tag data: exact equality proves "tangent"; a difference above
`ClrAngTol` relative to the operands proves "not tangent"; anything between is undecided and is SX2. A tolerance
never proves "tangent".

### 5.3 Expansion

`expandTangentChain` (`brep_modify_partial.go`) expands a `facetedPayload` only when every seed is a retained line or
arc between two faces that §2 reads (`retainedblend.Chainable`); any other faceted seed set is SX9. The walk calls
a retained continuation test in place of `tangentchain.Continues`: §5.2 decides a candidate edge at `v`; an
incident edge whose faces fail §2, or whose curve is a chord chain, is undecided (SX2). At a junction the third incident edge is the seam `{W_k, W_{k+1}}`; its
face map sends `P` to a wall, test 4 of reach §5 fails, and it is "not a continuation". Seeds whose chains overlap
merge into one chain (reach §5's fixed point), so the four x-parallel seeds of §1 give two closed chains.

Without `WithTangentChain` the selection is what the caller named. A selection that names every edge of a chain
builds the chain. A selected edge that ends at a proven tangent junction whose continuation is not selected is
SM2 (§10).

### 5.4 Claim J

**Claim J.** Let `e_k` and `e_{k+1}` meet at a tangent junction `V` with base outward normal `n_P` and common wall
outward normal `n_g` along `g`. Let `Π` be the plane through `V` normal to `d_V = n_P × n_g`. Then the
cross-section of `e_k`'s blend by `Π` equals the cross-section of `e_{k+1}`'s blend by `Π`:

| Blend | Common section in `Π` |
|---|---|
| fillet of radius `r`, convex | the quarter circle of radius `r` about `c_J = V − r·n_P − r·n_g`, from `c_J + r·n_P` to `c_J + r·n_g` |
| fillet, concave | the quarter circle of radius `r` about `c_J = V + r·n_P + r·n_g`, from `c_J − r·n_P` to `c_J − r·n_g` |
| chamfer, setbacks `t_P` on `P`, `t_W` on the walls | the segment from `V − t_P·n_g` to `V − t_W·n_P` (convex; `+` for concave) |

With the chain's side `s` (§5.1), both fillet rows are `c_J = V + r·m + s·r·n_P`, because `n_g = s·m`; the
construction computes every junction row from that one form.

*Proof.* `Π` contains `n_P` and `n_g`, hence `g`. For an R1 edge, `d_V` is the edge direction, so `Π` is normal to
the blend cylinder's axis and cuts it in blend §4.1's circle: centre at `r/cos(π/4) = r√2` along the bisector, which
is `−r·n_P − r·n_W` with `n_W = n_g`. For an R4 edge, the meridian plane at `V`'s azimuth holds the axis (parallel
to `n_P`) and the radial direction at `V` (parallel to `n_g`), so it is `Π`. In it the torus is a circle of radius
`r` about the tube centre: radius `R − r` from the axis toward `V` when the material lies inside the cylinder
(`n_g` radial outward), `R + r` when it lies outside (`n_g` radial inward), at depth `r` below `P`. Both give
`V − r·n_g − r·n_P`. A concave edge puts the ball on the void side, which flips both signs in both readings. In
every case the two blends touch `P` and `g` at the feet of `c_J` on them, so both arcs share their end points. The
chamfer case reads the setback points the same way: blend §4.1's setback lines and blend §4.2's cone circles both
pass through `V ∓ t_P·n_g` and `V ∓ t_W·n_P`. For two cylinder walls, both meridian planes at `V` hold both axes
and `g`, so both are `Π`, and the argument repeats. ∎

Both blend surfaces contain the common section, and both tangent planes along it hold `d_V` and the section's
tangent, so the blends meet G1. Claim J needs identity 2 of §5.2 exactly; without it the two sections differ and no
shared row exists, which is why near tangency is SX2 and not a charge (§16 lists the alternative).

### 5.5 Construction

The rewrite touches `P`, every wall `W_k`, every seam between consecutive walls, and for C2 the two end faces.

1. **Exact data.** Per junction `J`: `V_J`, `c_J`, and one row of section points `q_j(J)`, `j = 0 … m`, at tube
   angles `α_j = j·(π/2)/m`. `m` follows blend §5 step 3 for `τ = 2⁻¹⁰·r`; every chain edge has angle `π/2` and
   the same `r`, so every edge takes the same `m`. A chamfer has `m = 1`. **Compute each junction row once** and
   give the same vertex indices to both strips that meet there; `q_0(J) = P_J` lies on `P` and
   `q_m(J) = S_J = g ∩ P'` on the seam, with `P'` the plane parallel to `P` at height `t_W` on the chain's side
   `s`: into the material for a convex chain, into the void for a concave one. A fillet row is
   `V + m·r(1 − sin α_j) + s·n_P·r(1 − cos α_j)` and a chamfer row `V + m·t_P(1 − σ) + s·n_P·t_W·σ`.
2. **Strips.** An R1 edge's strip is blend §5.0 step 4 with each end row taken from its junction (or, for a C2 end,
   from blend §5.0 step 1). An R4 edge's strip is blend §4.4's grid with its two end columns taken from its
   junction rows. Rows are wound to face the blend's outward normal. A line fillet's cylinder has its axis through
   `p_0 + r·m + s·r·n_P`, faces away from the axis when convex and toward it when concave (tag reversed); a
   chamfer's plane faces `n_P + s·m`. An arc's torus or cone reads the rim's side as blend §4.2 does.
3. **Base face.** The chain's run in `P`'s loop gives way to the setback contour: an R1 edge's setback edge between
   its two end setback points, an R4 edge's ring-0 polygon. `P_J` is shared by both neighbours. `P` is then
   triangulated again by ear clipping over its loops (blend §5 step 2), with its other loops kept.
4. **Walls.** Each `W_k` loses the band between `P` and `P'`, measured as the height `s·n_P·(p − o_P)` on the
   chain's side: a plane wall by blend §5.0 step 2's chain cut at the
   setback line, a cylinder wall by blend §5 step 2's cut at the setback height. Each seam's held run is cut at
   `S_J`: the run's vertices on `P`'s side of `S_J` go and `S_J` takes their place.
5. **Ends (C2).** Blend §5.0 (R1) at the first and last edge, which must be straight with a retained plane wall and
   an end face `G` that is a retained plane normal to the edge. The end column is the junction row's construction
   with `V` the corner `P ∩ W ∩ G`; its rows lie in `G`, and its last vertex is the cut of `W` on `G ∩ W`, shared
   with the wall. `G` gives up the corner and the chain vertices nearer than the setbacks, and takes the column. `P`'s
   loop gives up the chain's run and the removed part of `G ∩ P` at each end. A first or last arc (R4) takes blend
   §4.4's end: its end column holds, per row `j` of the junction rows' tube angles, the point where the row's circle of
   the torus (or cone) meets `G`, a root of `n_G · x = h_G` in the azimuth nearest the previous row's (blend §4.4).
   The plane's setback point is row 0, on `G ∩ P`; the wall's is the cut of the generator `G ∩ W` at depth `t_W`. The
   edge's interior columns fan out from the end curve: row `j` of column `i` lies at the fraction `i / n_a` of the
   way from its azimuth on `G` to the azimuth of the other end's meridian, so every vertex lies on the torus. The end
   curve stays a chain of exact points (it is no conic). A chamfer's arc end takes extra rows, as blend §4.4's cone
   does, so that its chords follow the section `G` cuts in the cone to within `τ`.

   At a straight end `P`, `W` and `G` meet pairwise orthogonally at `V` (`cornerOf` admits exactly three faces and
   `G` is normal to the edge). The end is classified by the turn of each face's loop at `V` about the face's
   outward normal. The turn is read from the corner's three lines, not from the held vertices next to `V`, which
   can lie within their bounds of both lines: the chain edge's direction comes from its exact end points, and the
   directions along `G ∩ P` and `G ∩ W` point from `V` to the last vertex of `G`'s chain on each line, which the
   end clearance places beyond the setback by more than its bound. The loops' senses at `V` are topological: the
   walk follows `P`'s loop, the held mesh is oriented, so `G`'s loop runs the mesh edge from `V` to `P`'s
   neighbour on `G ∩ P` the other way, and `W`'s loop runs the chain edge the other way from `P`'s. An end whose
   `G` loop does not run that edge the other way, so that `G` would leave the first corner from `P`'s side or
   enter the last one toward it, refuses ("an end face's loop runs its edge with the base the same way as the
   base's loop"); an oriented held mesh never reaches it. A turn whose lines are too near parallel is undecided
   and refuses; at a corner of three orthogonal planes it reads `±1`.

   | Chain `s` | `P` at `V` | `W` at `V` | `G` at `V` | Case | `G`'s cap |
   |---|---|---|---|---|---|
   | −1 | convex | convex | convex | ordinary convex end | loses |
   | −1 | convex | convex | reflex | reflex foot: a ledge standing out of a wall | gains |
   | +1 | convex | convex | reflex | run-out: a slot leaving through a face | gains |
   | +1 | convex | convex | convex | blind end: a pocket floor ending at a sharp pocket corner | loses |
   | any | reflex | any | any | the base wraps the corner | refuse (SM8) |
   | any | any | reflex | any | the wall wraps the corner | refuse (SM8) |

   The end column lies in `G`'s plane, in the quadrant between `G ∩ P` and `G ∩ W` that holds the blend's
   section. Where `G` is convex at `V` that quadrant is `G`'s own, so `G` gives the cap up; where `G` is reflex
   the quadrant lies outside `G`, so `G` takes it. The loop operation is one splice in both cases: `V` and the
   parts of `G`'s two chains inside the setbacks give way to the column, and because a face lies to the left of
   its loop the splice shrinks a convex corner and grows a reflex one. The exact area `G` loses is the cap,
   `A_f = r²(1 − π/4)` for a fillet or `t_P·t_W/2` for a chamfer, with sign `+` for a convex `G` and `−` for a
   reflex one. A cap `G` takes is new surface on the far side of `K_k`, and §5.6 tests it as it tests the strip.

   When both ends lie on one `G`, its loops are read once and both splices are applied in turn before `G` is
   triangulated once. Each end's run holds only the vertices nearer its corner than the setbacks, and the end
   clearance places the last vertex of each of its chains beyond them; where two runs would still share a vertex,
   the second splice finds its run broken and refuses ("a face boundary breaks a run into pieces").
   When the chain and the removed parts of `G ∩ P` are all of `P`'s loop (a tongue, a blind pocket floor), `P`'s
   new loop is the setback contour alone: its first and last points are the two setback points on `G ∩ P`, and
   its closing segment runs along `G ∩ P`; the contour is then tested as a closed polygon.

   An arc end is built for a convex chain on a `G` convex at the corner only. A concave chain that ends on an arc
   refuses ("a concave open tangent chain ends on an arc; only a straight end is built"), and so does an arc end
   on a reflex `G` ("an arc end of a tangent chain needs an end face convex at the corner"), both SM9.
6. **Build.** `facetedrecord.Build`, as blend §5 step 5. Each chain edge gets its own blend group with role
   `fillet(k)` or `chamfer(k)`, `k` the edge's ordinal in the expanded selection.

New edges: each fillet junction row is an `Arc3` (centre `c_J`, radius `r`, axis `±d_V`) between a `Cylinder` and a
`Torus` blend, published by retention §10.1 E11 and marked `RetainedCurve` (blend §5.2). A chamfer junction keeps its
chord chain: no edge rule publishes the `Plane`/`Cone` section. Setback edges and wall cut edges follow
blend §5.2.

### 5.6 Gates

All tests are exact rational predicates over held floats and published bounds, inflated by `b + slack` with `b`
the largest vertex bound of the tested chord. The inflation only refuses more.

- **Base band.** Edge `k`'s band in `P` is a rectangle (R1: between the edge line, its setback line and the planes
  `Π` or `G` at its ends) or an annular sector (R4: radii `R` and `R ∓ t_P` about the axis, between the two radial
  lines to its ends). Every held boundary chord of `P` that is not on the chain must miss every band. A rectangle
  test is affine. An annular test clips the chord to the sector's wedge (two half-planes, exact), then compares
  the clipped chord's exact minimum and maximum squared distance from the axis with the two squared radii. An R4
  edge whose span exceeds `π` has no convex wedge and is SM3.
- **Neighbours.** The vertices of edges `k − 1` and `k + 1` other than `V_J` are chords of `P` for edge `k`'s band
  test like any other. A G1 neighbour leaves `V_J` beyond `Π_J`, so it clears the band.
- **Walls.** Every held boundary vertex of every wall that lies on no chain edge and no seam has height at least
  `t_W + b + slack` from `P` on the chain's side `s`. Height above a plane is affine, so the chords between such
  vertices clear `P'` as well. A concave pocket shallower than `t_W` fails it (SR4).
- **Seams.** Each seam's far end lies at height at least `t_W + b + slack` from `P` on side `s`, so `S_J` cuts it.
- **Sizes.** Blend SR3 per edge: an R4 edge whose base lies inside its circle (`κ = −1`: a convex edge whose
  material lies inside its cylinder, or a concave pocket floor's arc) needs `R − r > 0` (fillet) or `R − t_P > 0`
  (chamfer).
- **Asymmetric.** `P` borders every edge of the chain, so a reference that names `P` names it for every edge. A
  reference naming walls names each edge's own wall. A mix gives some edge two reference faces, and SX3 refuses it
  first (reach §6's cap-loop argument); SM5 is the second check. Several chains of one selection (the top and
  bottom outline of a bored plate) each build with their own base as reference. A reference that names every base
  at once, such as `Facing(+z).Or(Facing(-z))`, resolves to one face per edge. A chain after the first is found
  again on the partly blended body, possibly directed opposite to its selected edges, and takes each edge's
  reference by the edge's two end points in either direction.

A chord inside a band deflated by `b + slack` proves the blend consumes another face (SR4). A chord that meets the
inflated band but not the deflated one is undecided (SR5). Every band of a chain lies in the prism over its base
band between `P` and `P'`, so disjoint base bands give disjoint regions `K_k`, adjacent ones sharing only `Π_J`.
A component that enters `K_k` without touching `P`, a wall or a seam either crosses `K_k`'s boundary or lies wholly
inside it. The held audit (SR7, `internal/facetproof/faceted.go`) does not see the first case: it checks that the
mesh closes and that its shells nest with consistent signs, reading one probe point per shell, and it tests no
facet for intersection. So the chain runs its own strip test before the build (`strip_clear.go`): every facet
the result keeps outside the strip, held or rebuilt, that shares no vertex with a strip facet must miss it, decided
by `meshbool.TriTriClassify`, the exact triangle/triangle classification, on the held floats. A contact refuses
with "the blend meets another facet of the body at" a point: an end of a point or segment contact, or a corner of
the other facet for a coplanar overlap, which carries no point; a facet whose corners are collinear refuses
with "the blend's clearance from a flat facet is undecided"; a classification that fails refuses with "the blend's
clearance from the rest of the body is undecided"; all three are `ErrUnsupported`. Float bounding boxes, whose
corners are exact, skip pairs that cannot meet, and a uniform grid over the strip facets' boxes finds the pairs
whose boxes meet without testing every pair. A coordinate's cell is clamped to the grid in floating point before
it becomes an integer, so the cell index never decreases as the coordinate grows, even for a query far outside.
A strip facet's vertices are all made by the construction (rows, setback points, wall cuts, end columns), so a
pair that shares a vertex with it is the construction's own seam: along the setback contour, along a wall cut, or
for an open chain along the end column that a rebuilt end face shares with the strip; the gates above and the end
checks already place those. A rebuilt end face that takes a cap also keeps `G`'s held boundary vertices, and ear
clipping can join one of them to the cap, so a held facet at such a vertex could share it and still cross the
cap. Its pairs are therefore read by kind of shared vertex. A pair that shares a vertex the construction made is
skipped: besides the strip, only the rebuilt base and walls carry such vertices, and their planes meet `G`'s along
`G ∩ P` and `G ∩ W`; the new `P` and walls lie within the held ones, which met the held `G` only on its boundary,
and the cap meets those lines only along its own edges. A pair that shares only held vertices is classified like
any other and passes only when the exact contact is a shared vertex or the edge between two shared vertices. A
held facet coplanar with `G` that shares an edge with a rebuilt `G` facet meets it in what the classification
reports as one coplanar region, so it is refused; the refusal can only be a false one, and no current fixture
reaches it. The rest of `K_k`'s boundary is the removed bands of `P` and the walls and, for an open chain, the
caps its end faces give up or take. No other facet of the held solid crosses a removed band or a cap a convex `G`
gives up, because each was part of the held surface. A cap a reflex `G` takes is new surface, and a held shell can
cross it: a solid floating in a run-out slot's void across `G`'s plane, or a cavity across `G`'s plane inside a
reflex foot's ledge. So the rebuilt facets of an end face that takes a cap are tested like the strip, against
every facet the result keeps outside them, and a contact refuses with "the cap an end face takes meets another
facet of the body at" a point (`ErrUnsupported`). A shell wholly inside `K_k` crosses nothing, so with no
crossings the audit's one-point parity reads its nesting correctly: a concave chain's `K_k` lies in the void, and a
positive shell inside it nests inside a positive shell and refuses; a convex chain's `K_k` lies in the material,
and a cavity inside it ends up contained by no shell, so the audit refuses its negative sign.

The routes that blend one edge (blend Table RR and R5 of §8) and the mitre run of blend §4.7 build through
`buildClear`, which runs the same strip test before the build, with the same texts and the same skipped pairs; for
these routes the shared vertices also include the trim of an end face along the strip's end row. For them the
refusal is blend SR7, where for a chain it is SM5. Their region `K` is bounded by the removed bands of the two faces
and the end faces' cut corners, which the blend gates (blend SR4, SR5) place, so the argument above carries over: a
cavity or solid crossing `K`'s boundary meets a strip facet and refuses, and one wholly inside `K` fails the audit's
parity. A cavity whose corner reaches the removed corner of a plate's top edge, a hole mouth, a broken-out arc, a
groove's edge or the ridge between two bores builds without the test and refuses with it.

### 5.7 Bounds

Claim K (blend §6.3) holds with `K = ∪ K_k`: outside it the rewritten mesh equals the held one; inside each `K_k`
the strip's chords leave the exact blend by at most `τ`. The shared junction rows add no term, because both
neighbouring strips use the same vertices, which lie on both exact blend surfaces (Claim J). A junction vertex
takes `τ` plus the rounding of its computation. `VolSymDiff` and `AreaSlack` add each edge's terms as blend §6.3
states them, and `AreaSlack` the old geometric term of each face the chain rewrites, once. The exact areas are
blend §5.0's for an R1 edge with `μ = 0` (its end planes are normal to it) and blend §4.4's for an R4 edge over its
azimuth span. An end face's cap enters the removed area with the sign of §5.5 step 5. A wrong sign moves the exact
change by four caps, which `AreaSlack = |Δ_held − Δ_true|` absorbs, so no published reading shows it; the sign has
its own unit test.

## 6. Exact-rotation `Copy` keeps its tags (GAP-A)

`WithCopyRotation` gives a faceted copy that denotes `Q·S + t_Q`, with `Q` an exactly orthogonal rational rotation
(`internal/exactrotation`). `ExactGroupPlaced` drops every group's `Denoted`, and every `Surface` and `CapRecord`
except those of a plane that `Q` maps onto itself (below), because they describe the float placement and not `Q`.

**Fixed planes.** A retained `Plane` group with no denoted patch and `NormalBound == 0` keeps its `Surface`,
`CapRecord`, `AxialDelta` and `HasAxialDelta` verbatim, at bound 0, when `Q` maps its plane onto itself: for the tag's
exact normal `m = U × V` (not the rounded `Frame.N()`), and for the `CapRecord` plane's normal `m = (B·U) × (B·V)` when there is a record, `Q·m = m`,
`Qᵀ·m = m` and `m·t_Q = 0`, all in rationals (`facetedrecord.PlaneFixedBy`). The image plane is then the source
plane, so the source's exact record and its axial bound denote the copy's plane. The check compares recorded
data and admits only when every equality holds. A rotation about z fixes the ±z caps. Every other plane takes the
construction below.

**Construction.** For any other group with `Retained == true`, no denoted patch, and `Surface` a `Plane`, `Cylinder`,
`Sphere` or `Torus`: set `Surface` to the float image of the tag under the float delta
(`surfaceplace.TransformSurface`) and `NormalBound` to `min(2, ε_F + add)`, where `ε_F` is the group's own
`NormalBound` and `add` is retention §3.4's charge computed against `Q`: the linear map is `Q.Linear()`, the
translation `Q.Translation()`, and `e = 0` because `Q` is exactly orthogonal. A plane takes `8a + P`; a curved tag
takes the held term of retention §3.3 at displacement `2·(P + a·D)`. `a` and `P` are the exact distances of the
moved frame vectors and origin from their images under `Q`, so they cover both the float rounding and the gap
between the float delta and `Q`. A retained `Cone` keeps the bound 2, as retention §3.4 states. Denoted patches,
non-retained structural tags and `CapRecord` are dropped.

**Proof.** The copy's true solid is `Q·S + t_Q`. By Claim R each true patch of the group lies on the source tag
`T`, so it lies on `Q·T + t_Q` exactly. The published tag is within `a` (directions) and `P` (origin) of that
surface, and `Q` maps normals to normals without stretch, so retention §3.4's derivation holds with `e = 0`. A
plane that `Q` fixes is handled by the fixed-plane rule above.

**Code.** `facetedrecord.PlacedTagDeparture` takes a float transform. Split out
`PlacedTagDepartureBy(tag, moved, linear [3][3]*big.Rat, translation [3]*big.Rat, e float64, verts, reach)`, keep
`PlacedTagDeparture` as its float-transform wrapper, and call it from `ExactGroupPlaced` with `fp.Verts` and the
`reach` that `PlacedWith` already computes.

## 7. `ScaledAt` keeps curved tags

`ScaleImage` (`internal/facetedrecord/point_plane_scale.go`) scales each retained `Plane`, `Cylinder`, `Sphere`
and `Torus` tag through `facetedrecord.ScaleTag` (`scale_tag.go`). A positive uniform scale `x ↦ a + s·(x − a)`
maps each surface to a similar one with the same normal at corresponding points, so the only charge is rounding.

| Tag | Scaled tag | Charge `add` |
|---|---|---|
| `Plane` | origin `round(a + s·(o − a))`, frame from `r3.NewFrame` on the unchanged `U`, `V` | `8·α + P` |
| `Cylinder` | axis unchanged, origin as above, `Radius = round(s·R)` | held term of retention §3.3 at `2·(P + ρ)` |
| `Sphere` | centre as above, `Radius = round(s·R)` | the same |
| `Torus` | axis unchanged, centre as above, `Major` and `Minor` each rounded from `s` times the source value | the same |

`P` is the exact distance of the rounded origin from `a + s·(o − a)`, `ρ` the exact gap of the rounded radius
from `s` times the source radius (a `Torus` takes the sum of its two radii's gaps), and `α` the exact gap of `NewFrame`'s vectors from `U`, `V`. Each group's
`NormalBound` becomes `min(2, ε_F + add)`; `ε_F` is a bound on an angle, which a uniform scale preserves. `Cone`
and denoted groups still read `Faceted`: retention §3.4 states no cone rule, and the interval denotation has no
similarity image here. A retained curved group passes through the same A4 check at assembly as any other. A plane group that is not
retained keeps the bound 2.

## 8. Parallel cylinders (R5)

Two cylinders with parallel axes meet in straight generators. Two rows publish and blend them.

**E10** (retention §10.1). `Cylinder` and `Cylinder` with `|a1 × a2| ≤ 10⁻¹²` and axis distance `s` with
`|R1 − R2| < s < R1 + R2` within slack: `Line3`. Two generators exist, at the cross-section's circle–circle
intersections; either passes if E6 accepts it, as in E4.

**R5** (Table RR). A `Line3` of E10 ending on retained planes normal to the axes. In the cross-section normal to
the edge, the walls are circles `C1 (o1, R1)` and `C2 (o2, R2)` meeting at `K`. The angle between their tangents at
`K` must clear `minSine`.

- Fillet: the centre is the point of `circle(o1, R1 + κ1·r) ∩ circle(o2, R2 + κ2·r)` nearest `K`, with `κ_i`
  chosen per circle exactly as blend §4.3 chooses `κ`. The tangent points are the radial projections of the centre
  onto `C1` and `C2`. The blend is a `Cylinder` of radius `r` along the edge.
- Chamfer: arc length `dA` along `C1` and `dB` along `C2` from `K`; the blend is the `Plane` through the two
  generators.
- Each wall is cut as blend §4.3 cuts R3's wall, by the half-plane through its axis and its setback generator.
- Ends: blend §4.3's end rule.

The offset circles not meeting is SR3. Both ends must be corners of the same type as the edge (blend SR5), so
a valley between bosses that stand on a plate, with one convex and one reflex end, refuses. The construction reuses `cylLine.solveSection` with the line `L` replaced
by `C1`.

## 9. Refusals

### 9.1 `Shell` (GAP-B)

An inward `Shell` removes the cavity `S ⊖_F t`: the erosion of `S` by `t`, open through the removed faces `F`
(modify §8's erosion rules). The tags give every offset surface exactly: a plane moved `t`, a cylinder at
`R ∓ t`, a sphere, a torus minor radius `∓ t`. The cavity also needs where those offsets meet: a miter at a convex
edge, a radius-`t` blend about a reflex edge, the faces that vanish when `t` exceeds them, and how a bore's offset
meets a wall's offset. A mesh result records none of it (retention §12, last bullet), so building the cavity from
tags is a general 3D erosion kernel.

Two subclasses were examined.

- **Every face a retained plane, and the body provably convex.** Convexity is proven when every held vertex off a
  face lies on the inner side of that face's plane by more than its bound. The cavity is then the intersection of
  the moved half-spaces, exact by rational vertex enumeration, and SX8's topology-change refusal applies when a
  face drops out. This is sound. It is not built: it is a new polyhedral kernel for bodies that mesh Booleans
  rarely produce.
- **Retained planes and cylinders** (the drilled rounded box of §1). The bore makes the body non-convex, and its
  reflex junctions need trims the record lacks. This design has no construction from tags alone.

`Shell` of a `facetedPayload` stays SX9 outside the source-backed routes (`agent-docs/shell-faceted-boss-design.md`).
§16 lists the operand-record route.

### 9.2 `Draft` (GAP-C)

Drafting turns each selected wall's tag about its line on the neutral plane. The turned plane is computable
(a rational rotation, as `internal/exactrotation`). Each drafted face then needs new trims against its neighbours,
and a bore through a drafted wall meets it in a new curve, which re-meshes the bore wall. These are the same
missing trims as §9.1. `Draft` of a brep receiver is also SD23 (the probe's unplaced-tool Cut, a `brepPayload`),
so a mesh `Draft` would come after a brep one. `Draft` of a `facetedPayload` stays SD23. §16 lists the
operand-record route.

### 9.3 `Offset` of a solid

`Offset` refuses every solid (surface-design R37, `agent-docs/missing-features.md` "Direct edits"). Retention
changes nothing about it.

### 9.4 `PatternJoin` (GAP-F)

Not a retention matter. `PatternJoin` joins the target and every instance in one many-body `Union`. Its pairwise
fallback, `tryPrismUnionChain` (`boolean_many.go`), admits prism and exact-Loft operands only; the contract is
`agent-docs/api-design.md`'s many-body `Union` paragraph. Each pair of a chain would be an ordinary mesh Boolean,
and retention §3.1 already carries tags through a second generation. Whether faceted operands may join that chain
belongs to that owner.

### 9.5 Cylinder/Cylinder, `Cone` and `Torus` blends

- **Crossing cylinders.** The rolling ball's centre runs along the intersection of two offset cylinders, a
  quartic, and the blend is a canal surface about it. It is no `Cylinder`, `Cone`, `Torus` or `Sphere`, and
  `NURBSSurface` carries no geometry a proof can read (spline §7). The convex saddle a bore leaves in a larger
  shaft, with exact perpendicular coordinate axes that meet and `2r² < R²`, builds an equal-setback `Chamfer` as
  blend R7 (§4.9) and a `Fillet` as an untagged faceted pipe (blend §4.10). Every other crossing pair (a pin
  joined through a shaft, two crossing bores, a bore off the shaft's axis or off the coordinate axes,
  `r/R ≥ 1/√2`, a two-distance chamfer) is SR1.
- **`Cone` and `Torus` faces.** In the meridian of a coaxial pair the blend is a `Torus`, so the geometry exists.
  The retained cones and tori that reach a mesh result come from revolves. Their tag is a float re-expression (the
  cone's half-angle is an `Atan2`, `internal/revolveplan/payload.go`) whose departure is stated only through the
  face's denoted interval surface (`topo.Face.Denoted`), which blend §2 reads only for a `Plane` or `Cylinder` tag
  equal to it, and blend §4.5 for a `Sphere` whose record's centre and radius are its tag's. A cone or torus that a blend
  built (blend §4.2) meets its neighbours tangentially, so it has no edge to blend, and a later cut through it
  gives a non-coaxial rim. SR1. §16 lists the denoted-surface route.

## 10. Table SM — added refusal rows

Rows narrow blend Table SR and reach SX2/SX9; sentinels follow those tables.

| SM | Call | Sentinel |
|---|---|---|
| **SM1** | a chain junction whose tangency §5.2 cannot decide | `ErrUnsupported` (reach SX2's text) |
| **SM2** | a selected edge ends at a proven tangent junction whose continuation is not selected and `WithTangentChain` is absent; the message names the junction point and `WithTangentChain` | `ErrUnsupported` |
| **SM3** | a chain whose edges share no base plane, a junction of two plane walls, a cylinder wall whose axis is not exactly normal to `P`, an R4 chain edge spanning more than `π`, or a chain that mixes convex and concave edges | `ErrUnsupported` |
| **SM4** | the chain gates of §5.6: proven overlap | `ErrDegenerate` (SR4) |
| **SM5** | the chain gates of §5.6: undecided; a strip facet, or a facet of an end face that takes a cap, that meets another facet of the body; or a chain whose resolved references mix `P` and walls | `ErrUnsupported` (SR5; SX3 first for the mix) |
| **SM6** | R5: the offset circles do not meet | `ErrDegenerate` (SR3) |
| **SM7** | R5: the walls meet at `K` below `minSine` | `ErrUnsupported` (SR5) |
| **SM8** | a straight end of an open chain whose base or wall is reflex at the corner: "a chain end's base or wall is reflex at the corner"; a turn there too near straight to read: "a chain end's base or wall turn at the corner is undecided"; an end face whose loop runs its edge with the base the way the base's loop does (a guard: an oriented held mesh never reaches it) | `ErrUnsupported` |
| **SM9** | an open concave chain that ends on an arc, or an arc end on an end face reflex at the corner | `ErrUnsupported` |

Kept refusals: `Shell` (the `shell.go` text, SX9), `Draft` (SD23), `Offset` (R37), `PatternJoin` (the many-body
`Union` text), crossing cylinders outside blend R7's chamfer and fillet, `Cone`, `Torus` and oblique plane/cylinder rims
(SR1).

Gate order: modify §4 stage 1, reach stage 2 (expansion, SM1), stage 3 (SX3), then SR1, SM3, SM9, SR2 and SM2,
SR3 and SM6, SR6, SM8, SR5, SM5 and SM7, SR4 and SM4, the construction, SR7.

## 11. Readers

- A chain result is a retained body; every retention §4 and §10.3 row applies. Its junction edges join two blend
  faces tangentially. `FaceCreatedBy` finds each blend face by its `fillet(k)` or `chamfer(k)` role.
- An exact-rotation copy answers `NormalAt`, `Facing` and `Face.Surface()` from its tags. `MirrorFace` admits a
  plane only at bound 0, which a plane that `Q` fixes keeps. `ToFace` also needs
  a `CapRecord`, which only such a fixed plane keeps; every other plane's record is dropped.
  A retained blend on the copy refuses at SR1 unless the bound is 0. `ExactCopyMoments` and the motion engine's
  `ExactPlaced` reading are unchanged.
- A scaled body answers the same readers; `MirrorFace` admits a plane whose charge is 0.

## 12. Tests and the red leg each must show

Tests assert on computed geometry. Volumes compare with the result's published bound and normals with
`NormalAt`'s bound, never with a pinned bound literal: float bounds differ between amd64 and arm64. Each test is
shown red first by the change named. `A_f = r²(1 − π/4)` is a fillet's corner area and
`x̄ = r·(10 − 3π) / (3·(4 − π))` its centroid's distance from the sharp corner along either face.

| Test | Asserts | Red leg |
|---|---|---|
| R1 two-distance chamfer (placed-tool plate, one reference face per edge) | the bevel plane runs through the setback `D` on the reference face and `Other` on the other; removed `L·D·Other/2` | swap the §4.1 mapping: the distances swap |
| R2 two-distance chamfer, reference the plane | removed `π·dA·dB·(R + dA/3)`; the `Cone` leans by `atan(dA / dB)`, starts `dB` below the plane and has radius `R` there | use `D` on both faces: the half-angle and volume are wrong |
| R3 two-distance chamfer of a groove's top edge | the wall's setback generator lies at angle `dB / R` from `K` | keep `solveSection`'s `dA` for the wall: the angle reads `dA / R` |
| R4 two-distance chamfer of an arc | as R2 over the arc's span | as R2 |
| Two R1 edges, two reference faces, one call | each edge's setbacks match its own reference | resolve references by face pointer: the second step finds no reference |
| Chain expansion on P2 | a top-front seed with `WithTangentChain` expands to 4 `Line3` and 4 `Arc3` in body order | keep the faceted gate in `expandTangentChain`: SX9 |
| Closed chain fillet on P2, r = 1 (C1) | 4 `Cylinder`, 4 `Torus` (`Major = R − r`); removed `A_f·(96 + 2π·(R − x̄))`; junctions `Arc3` of radius `r`; `VerifyAll` passes | build each junction row per edge: rows differ in the last bits, SR7 refuses |
| Same, torus side | as above | take `κ` the other way: `Major = R + r` and the volume is off |
| Closed chain chamfer, equal and with reference `P` | removed `(t_P·t_W / 2)·(ΣL + 2π·(R − t_P/3))` | let the arcs take `t_W` along `P`: the two-distance volume is off |
| Two closed chains (top and bottom outline), reference naming both bases | each chain removes `(t_P·t_W / 2)·(ΣL + 2π·(R − t_P/3))`; the 8 corner faces are `Cone` of radius `R − t_P` and half-angle `atan(t_P / t_W)`, the 8 straight bevels `Plane` | a later chain loses its reference: `the reference face is not a face of the selected edge` |
| Four x-parallel seeds | two closed chains; removed volume twice the C1 row's | skip merging overlapping chains: the corners are claimed twice and SR2 refuses |
| One straight edge, no option | SM2 naming `WithTangentChain` and the junction point | drop SM2: the message is blend SR5's "not a retained plane" |
| All 8 edges named, no option | builds the C1 result; volume equal within both bounds | treat shared junction corners as SR2: refused |
| Open chain (C2): one lateral fillet, seed its top edge | chain of line, arc, line; removed `A_f·(L1 + L2) + (π/2)·(R − x̄)·A_f` | stop the walk at the junction and use the R1 end rule: refused as grazing |
| Open chain ending on an arc (C2, R4): the corner-rounded box trimmed by a plane through its arc; seed the arc | line and arc ending on `G`; removed volume `A_f·L + ∫∫ ρ·Δφ(ρ)` over the section's corner, `Δφ` the angle from `G` to the tangent point | start every row at the corner's azimuth: the end rows leave `G` (unit test) |
| Concave closed chain: pocket floor (40×40×10 plate, 20×20 `r = 3` pocket, floor `z = 5`, `ΣL = 56`), fillet `r = 1` | fill `A_f·(ΣL + 2π·(R − x̄))`; 4 `Cylinder`, 4 `Torus` (`Major = R − r`); every blend's `NormalAt` points into the void; `VerifyAll` passes | restore the concave refusal: refused; convex rows: SR7; an unreversed cylinder tag: the normals point into the material |
| Concave closed chain chamfer, equal and with reference the floor | fill `(t_P·t_W / 2)·(ΣL + 2π·(R − t_P/3))` | the convex chamfer normal `n_P − m`: SR7; swap the setbacks: the two-distance fill is off |
| Concave closed chain: boss root (20×20 `r = 3` boss joined to the plate's top) | fill `A_f·(ΣL + 2π·(R + x̄))`; 4 `Torus` (`Major = R + r`) | take `κ` the other way in the arc tag: SR7 |
| Concave chain, all 8 edges named | builds without the option; the same fill | none |
| Chain strip clearance | a 2×1×1 box joined over the pocket floor with only its lower front edge in the fill, and a cavity under the cross-drilled box's top-front edge with only its upper front edge in the removed corner, both refuse SM5 "the blend meets another facet of the body" at a point with `x` in 15…17; unit tests check that the grid keeps every pair of meeting boxes (random, degenerate-axis, far, touching and boundary boxes) and that a crossing, a coplanar overlap and a flat facet refuse | drop the strip test: both build; report a strip vertex: the point lies outside the solid; visit one grid column, or convert the cell before clamping (amd64): the grid loses pairs; read a region contact's point: panic |
| Single-edge strip clearance | a cavity whose corner reaches the corner the blend removes refuses blend SR7 "the blend meets another facet of the body" at a point inside the cavity's `x` range, for a plate's top edge (R1, `r = 2`), a mitre run of two top edges, a hole mouth (R2), a broken-out arc, a groove's edge (R3) and the ridge between two bores (R5); a top-edge cavity at `y` 3.25…4.25, `z` 18.75…19.75 and one at `y` 1.99…4, `z` 16…19.995, whose corner lies inside the box of the strip's first facet but under its chord so the exact test runs on the cavity's facets, both build and remove `40·4·(1 − π/4)` | call `build` for one route in place of `buildClear`: that route's cavity builds; count every tested pair with a cavity vertex as a contact: the near cavity refuses and the far one still builds |
| Concave chain gates | a pocket 0.8 deep refuses SR4 "the blend consumes the wall"; `r = 3` refuses SR3; a cube floating inside the fill refuses SR7 by nesting parity while one clear of it builds; a chain of mixed sides refuses SM3 (unit test) | classify by `n_W = −m` only: every concave row refuses; measure the wall band on the convex side: the pocket refuses; drop the wall clearance: the shallow pocket refuses with another text; drop the mixed-side check: the unit test builds |
| Chain ends: reflex foot (wall `0…10 × 0…40 × 0…30` joined with a ledge `5…30 × 10…30 × 4…10` whose `x = 30` corners are `r = 3`), fillet `r = 1` | removed `A_f·(48 + π·(R − x̄))`; 3 `Cylinder`, 2 `Torus` (`Major = R − r`); the `x = 10` face's area grows by `2A_f` within its bounds and the face holds a point just inside each cap; `VerifyAll` passes | keep a reflex `G` refusing: "an end face's corner does not match the chain's convexity"; walk `G`'s loop the same way at both ends: "an end face leaves the chain's corner along neither of its lines" |
| Chain ends: run-out (the slot of `roundedPocket` leaving through `x = 0`), fillet and chamfer `1` | fill `A_f·(68 + π·(R − x̄))` and `(1/2)·(68 + π·(R − 1/3))`; the `x = 0` face grows by two caps and holds a point just inside each; blend normals point into the void | keep a reflex `G` refusing: refused; rebuild each end alone: the strip meets the duplicated `G` facets |
| Chain ends: blind end (pocket `10…30` with only its `x = 30` corners `r = 3`, floor `z = 5`), fillet `r = 1` | fill `A_f·(48 + π·(R − x̄))`; the `x = 10` wall shrinks by `2A_f` and lies more than 0.1 from a point just inside each cap | admit only a reflex `G` for a concave chain: refused; drop the whole-loop base branch: "a face boundary holds no run to replace" |
| Chain ends: tongue (40×20×20, `x = 40` corners `r = 3`, bored), fillet `r = 1`, both ends on `x = 0` | removed `A_f·(88 + π·(R − x̄))`; 3 `Cylinder`, 2 `Torus` (`Major = R − r`); the `x = 0` face shrinks by `2A_f` and gives up both caps | drop the whole-loop base branch, or rebuild each end alone |
| Chain end gates | the step (block 40×20×30 with `x = 40` corners `r = 3`, cut to `z = 10` for `x > 10`) and the blind pocket's mouth refuse SM8; the slot `−2…30` with four `r = 3` corners refuses SM9 for its concave arc ends, and a ledge `8…30` with four corners for its arc ends on the reflex wall; unit tests read convex, reflex and undecided turns, `G`'s turn at both ends of the tongue and the reflex foot, and the cap's sign | drop the wall's turn: the step refuses SR4; drop the base's turn: the mouth builds a flat-ended notch; drop either arc gate: the other arc text, or the ledge builds; read `G`'s turn one way at both ends: the unit test fails; drop the cap's sign: the unit test fails |
| Taken cap clearance | a 0.1 box joined across `x = 0` in the run-out slot's void, and a 0.1 cavity cut across `x = 10` in the reflex foot's ledge corner, each at either corner, refuse SM5 "the cap an end face takes meets another facet of the body"; the box moved clear of the cap builds with the run-out's fill; unit tests refuse a facet that crosses a taken cap but not the strip, one that shares a held vertex with the cap and crosses it beyond, and one coplanar with the cap beside a shared edge, while facets meeting the cap only at a shared vertex or along a shared edge pass, and `checkEndSense` refuses an end face's loop sense that matches the base's | skip the taken caps: the box builds, and the cavity refuses SR7 "a result shell has inconsistent orientation"; skip every pair that shares a vertex: the held-vertex unit case builds; refuse every contact of a sharing pair, or every point contact: the run-out, its chamfer and the reflex foot refuse; drop the loop-sense test from `checkEndSense`: its unit test builds; drop the call from `rewriteChain`: nothing goes red, because no oriented held mesh reaches it |
| Chain clearance | a pocket whose chord crosses a band while its vertices lie outside it refuses SM5; a hole deep inside a band refuses SM4 | test vertices only: the crossing chord builds and SR7 or the volume check fails |
| `internal/tangentchain` retained oracle | exactly tangent plane/cylinder tags prove "tangent"; tags one float step off are undecided; a clear miss is "not tangent" | replace the identity by a tolerance: the one-step case proves "tangent" |
| Exact-rotation copy of P3, 20° | 6 `Plane`, 5 `Cylinder`; `NormalAt` at an interior facet centroid within its bound plus 64 float steps of the rotated source normal; bounds below 1 | keep `ExactGroupPlaced`'s drop: `Faceted` |
| `facetedrecord` exact-copy charge | a plane whose image under a 20° `Q` rounds has `NormalBound ≥ 8a + P` recomputed in the test, and above 0 | charge 0: the relation fails |
| Exact-rotation copy of P3, 20°, ±z caps | bound 0; `ToFace` stops at exactly z = 20 and `MirrorFace` across the top cap mirrors to the exact z range; a tilted rotation still refuses | drop the fixed-plane arm in `ExactGroupPlaced`: both refuse |
| Cut of the 20° copy | untouched planes and cylinders keep their tags | clear `Retained` in `ExactGroupPlaced`: the Cut loses them |
| `ScaledAt` of P2 by 2 and by 1.1 | 6 `Plane`, 5 `Cylinder`; `Radius` is `2·R` at `s = 2` and within its rounding of `1.1·R`; `NormalAt` bounds below 1 | keep the drop: `Faceted`; keep the plane's 2: bound 2 |
| `facetedrecord` scale charge | a cylinder at `s = 1.1` has `NormalBound` at least the held term at `2·(P + ρ)` recomputed in the test | charge 0: the relation fails |
| R5: two overlapping parallel bores, placed-tool cuts | the bore–bore ridge is `Line3`; fillet and chamfer change the volume by `L` times the section's corner area; the fillet is a `Cylinder` of radius `r` through the ball's centre | flip `κ1`: the centre sits on the wrong side of `C1` |
| R5 refusals | a valley between bosses standing on a plate (ends of unlike corner type) and bores with crossing axes refuse with `ErrUnsupported` | drop the end-corner gate: the valley blend builds |
| Kept refusals on P2 | `Shell`, `Draft`, `Offset` and `PatternJoin` keep the texts of §10 | none; pins the refusals |

## 13. Files and fallout

| Area | Change |
|---|---|
| `internal/retainedblend/route.go` | `Chamfer` takes a two-distance spec with per-target reference tags (§4.1); group selected edges into chains; SM2, SM3 |
| `internal/retainedblend/line_rewrite.go`, `arc_rewrite.go`, `rewrite.go` | setbacks from the mapping; an end at a junction plane in place of an end face |
| `internal/retainedblend/cyl_rewrite.go` | the wall setback reads `dB` (§4.1) |
| `internal/retainedblend/chain.go`, `chain_rewrite.go` | new: Table RC, §5.2 junctions, Claim J rows, §5.5 construction, §5.6 gates |
| `internal/retainedblend/cyl2.go`, `cyl2_rewrite.go` | new: R5 |
| `internal/brepmodify/chamfer.go`, `asymmetric.go` | pass the resolved reference into `retainedblend.Chamfer` |
| `internal/tangentchain/` | the retained continuation test of §5.3 |
| `brep_modify_partial.go` | admit a retained `facetedPayload` and dispatch the retained test |
| `internal/facetedrecord/place.go`, `place_tag.go`, `placed.go` | §6 |
| `internal/facetedrecord/point_plane_scale.go`, `place_tag.go` | §7 |
| `internal/facetedrecord/edge_curve.go` | E10 |
| `document.go` | `WithCopyRotation`'s doc comment states the tags it keeps (§6) |
| `agent-docs/retained-edge-blend-design.md` | Table RR: the two-distance chamfer, R5, a pointer to this document's chains; §7 |
| `agent-docs/faceted-source-retention-design.md` | §3.4 (exact rotation), §6 (`ScaledAt`), §10.1 (E10) |
| `agent-docs/modify-reach-design.md` | RX6 row and SX9 text; §5's sentence that faceted curves never enter the oracle |
| `agent-docs/missing-features.md` | the rows for faceted `Fillet`/`Chamfer` and for analytic readings of a `WithCopyRotation` copy |
| `apitest/` | the tests of §12; `apitest/faceted_retained_blend_test.go` holds the blend rows |
| `examples/` | one executable example: the rounded box's top rim filleted after a placed-tool Cut |
| `docs/api.md`, `docs/data-structures.md` | none: no exported name or signature changes, and their rows name no receiver class |

## 14. Increments

Each increment is one PR. M1 ships this document.

| # | Lands | Gate to start the next |
|---|---|---|
| M1 | §4: two-distance chamfer on R1–R4 | the five two-distance rows of §12 pass; `go test ./...` on CI |
| M2 | §6: exact-rotation `Copy` keeps its tags (independent of M1) | the four exact-copy rows pass on CI |
| M3 | §5.2–§5.7 for C1: retained expansion, junctions, closed chains, SM1–SM5 | the expansion, C1, chamfer, seed-merge, SM2, all-8 and oracle rows pass, and the example's output is verified |
| M4 | C2: open chains with blend §5.0 straight ends; E11 for the fillet junction `Arc3` | the C2 row passes |
| M4b | C2: open chains whose first or last edge is a blend §4.4 arc | the arc-end rows pass |
| M5 | §7: `ScaledAt` keeps curved tags | the two scale rows pass |
| M6 | §8: E10 and R5 | the R5 row passes |
| M7 | C1 concave: the chain side `s` of §5.1 in the rows, wall bands and tags | the concave chain rows of §12 pass |
| M8 | C2 end corners: open concave chains, reflex feet and blind ends, the same-face end rebuild and the whole-loop base contour of §5.5 step 5; SM8, SM9 | the chain-end rows of §12 pass |

Chains are built for closed and open outlines of lines and arcs, convex or concave.
`retainedblend.Tangent` decides each junction (`chain_tangent.go`), `planChains` groups and orders the selection
(`chain.go`) and `rewriteChain` builds one chain (`chain_rewrite.go`); the ends of an open chain live in
`chain_open.go`, an arc end in `chain_arc_end.go`, and the strip's clearance from the rest of the body, for every
route, in `strip_clear.go`. A closed chain has at least three edges and an open chain at least two. A straight end
whose base or wall is reflex at the corner, a straight end on a face that is not normal to its edge, an arc end of
a concave chain, an arc end on a face reflex at the corner and an arc end on a face that is not parallel to the
axis are refused. The edge between a `Cylinder` and a `Torus` blend of a fillet chain is an `Arc3`
(retention §10.1 E11).

M3 starts with one real end-to-end run on the P2 fixture before any other fixture. If that run refuses, the
increment stops and reports the refusal.

## 15. Rejected shortcuts

- **Admit near tangency by a tolerance.** Reach §5 lets a tolerance prove "no" only. Claim J needs equal sections,
  which near tangency does not give.
- **Blend each chain edge on the evolving mesh.** Blending one edge alone leaves a flat end cap in `Π_J`; the
  next edge's blend then removes that cap to zero area, which no end rule builds.
- **Expand a tangent chain without `WithTangentChain`.** Cardinality applies to what the caller named (reach §2).
  SM2 names the option instead.
- **Re-derive the copy's or the scaled body's tag from its moved polygons.** It blesses a surface from the
  polygons it should be checked against (CLAUDE.md).
- **Scale or rotate a tag and set its bound to 0.** The origin and radius round; a zero bound would claim the
  rounded tag is the true surface.
- **Shell by a Cut with an inward-offset copy of the held mesh.** A vertex-normal offset of a mesh is not the
  erosion and carries no proof.
- **Rebuild the operands of a mesh Cut from tags and roles.** Payload §5.2 forbids rebuilding from roles, and tags
  carry no trims.

## 16. Decisions the user may want to overturn

| Decision | Default | Alternative |
|---|---|---|
| Chain junctions | exact tangency only (§5.2) | a bounded near-tangency rule |
| `Shell` and `Draft` of a mesh Cut | refuse (§9.1, §9.2) | a source-backed route on an operand record |
| `Cone` and `Torus` rims | refuse (§9.5) | a coaxial revolution class read from denoted surfaces |
| Increment order | M1 and M2 first (small), then the chains | chains first |
| R5 | build last (M6) | drop it |

- **Chain junctions.** The default proof is short and matches reach §5. It refuses every part whose wall and
  fillet tags miss tangency by a rounding, for example a 40.1 mm box. The bounded rule admits those parts. It costs
  a new proof that the seam lies within `√(2R·δ)` of the foot and that the crease angle is at most `√(2δ/R)`,
  with `δ` the exact tangency defect, both charged into the junction rows.
- **`Shell` and `Draft`.** The mesh Cut would keep an immutable record of its operands `T` and `U`. Then
  `Shell(T − U) = (T − U) − ((T ⊖_F t) − (U ⊕ t))` holds for modify §8's erosion when `F` are faces of `T` and `U`
  crosses `T` by more than `t`, and `Draft(T − U) = Draft(T) − U` holds when every drafted face descends from `T`.
  The route gains shells and drafts of drilled parts. It costs a new record on every mesh Cut, a dilation of the
  tool, two more mesh Booleans per call (which may hit the coarse-held-contact refusal on curved faces), and a
  separate design of about three PRs.
- **`Cone` and `Torus`.** The coaxial class's meridian blend is a `Torus`, built from the denoted interval
  surfaces. It gains blends on revolved shafts after a placed cut. It costs an interval construction and a proof
  that two denoted walls share one axis. A revolved shaft's plane and cylinder faces blend after a placed cut where
  the other gates of the route hold (blend §2), and a revolved ball's plane-sphere rim blends by R6 (blend §4.5); the
  cone and torus classes wait.
- **Increment order.** Chains first lands the highest-value case one PR sooner. M3's two-distance chain chamfer
  row then waits for M1.
- **R5.** Dropping it saves M6. Two parallel cylinders meeting in a transversal edge are rare in machined parts.
