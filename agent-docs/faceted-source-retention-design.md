# Faceted Source Retention Design

A mesh Boolean result stores, for each of its faces, the analytic surface of
the operand face it came from. Today every such face reads `Faceted`, so a
filleted cube that a tilted tool drilled reports its six planes and four
blend cylinders as polygons. This design makes each face that descends from
one operand face publish that face's surface, and lets the readers that need
an exact plane or a bounded normal use it. Companion to
`agent-docs/evaluator-design.md` §9 (the mesh Boolean),
`agent-docs/faceted-vertex-bounds-design.md` ("vertex bounds §N": per-vertex
displacement) and `agent-docs/payload-verification-design.md` §5 ("payload §N": the
faceted boundary certificate this design supplies the source identity for).
"core §N" is `agent-docs/api-design.md`.

No public type changes. `Face.Surface()`, `Face.NormalAt`, selectors and the
body-relative stops report a different, tighter answer on faces they read
`Faceted` or refuse on today (§6).

## 1. Problem

Probe on `main` on 2026-10-20: a 40×20×20 box, four lateral edges filleted
at r=3, then the next operation.

| Row | Next operation | Path | Result faces |
|---|---|---|---|
| P1 | Cross-drill Ø8 tool, same placement (offset-plane sketch, `Symmetric`) | class B (general-boolean §3) | 11 faces: 6 `Plane`, 5 `Cylinder` |
| P2 | Same cross-drill, tool moved by `Transform` | mesh | every face `Faceted` |
| P3 | Same tool rotated 20° about z | mesh | 12 faces, all `Faceted` |
| P4 | `Fillet` of the four x-parallel cap edges | `Fillet` on the filleted prism | `ErrUnsupported` (brep-modify SB7: the end face is the blend cylinder) |

P1 shows the reachable ideal: the part is still an analytic solid after a
drill, and a later hole or fillet is computed against the true cube. P3 shows
the loss. Of its 12 faces, ten come from the filleted box and two from the
tool's exact `Cylinder`. All 12 lose
their identity because `boolean_mesh.go` builds each source group with
`Origins` and `Planar` only, and `internal/facetedrecord/assemble.go` assigns
`surfacegeom.Faceted` to every face whose group has no `Surface`. A group's
`Surface` field exists, and structural builders (point-loft cone trims, the
smooth loft) already fill it; the generic Boolean never does.

What the loss costs a caller:

1. `Face.Surface()` is `Faceted`, `NormalAt` is `ErrUnsupported`, and
   `Facing`/`ParallelTo`-style selectors cannot read the plane normal of a
   flat face.
2. `ToFace`, `ToFaceAngular` and `MirrorFace` refuse a flat face that is
   `Faceted` (`internal/extent/to_face.go`, `mirror.go`).
3. Every later mesh Boolean treats an untouched plane or blend as the
   polygons the first Boolean left, with that Boolean's bounds.
4. Chains of modify operations cannot tell a flat untouched face from a
   curved one (P4 gives the refusal for a body that is one analytic fillet
   away from a clean route).

## 2. What a retained source claims

**Claim R.** Let group `g` be the facets that descend from operand face `F`
whose built surface is `T` (a `Plane`, `Cylinder`, `Cone`, `Sphere` or
`Torus`). Every point of the true result patch of `g` lies on `T`, and the
patch's outward normal at that point is `T`'s natural normal, negated when
`Reversed`.

Why it holds. The true result is the Boolean of the true operands, and its
boundary is made of pieces of the operands' boundaries. A piece of `F`
stays on `F`'s surface. The held mesh already relies on the same
correspondence: `MeshBound` is the distance from the true boundary to "the
held mesh's corresponding piece" (`facetedrecord.Payload.MeshBound`), and
payload §5.1 requires each held facet to map to one true patch. The
hidden-tangency refusals (payload §5.3) are what keep the correspondence
from flipping.

What Claim R does not say:

- The held facets of `g` lie on `T`. They lie within the face's `Faceted`
  displacement `δ_f` of the true patch, which lies on `T`.
- `T` is fitted to polygons. The surface comes from the operand face's
  recorded `Surface` at Boolean time. A `Faceted` operand face, a
  `NURBSSurface` face and a face with a `Denoted` patch contribute no surface
  (§3.2). Provenance roles are never used to rebuild one (payload §5.2).
- `T` is exact for the operand. The operand's own displacement of its built
  surface from its design intent is already inside `sourceBound`
  (payload §5.3) and enters the result through `δ_f`. The retained tag adds
  no position claim of its own.

Residuals. A distance from a held vertex to `T`, computed from the vertex
floats and the recorded surface, is a computation on two known objects. It
bounds how far that facet sits from `T`. It never replaces `δ_f` as the
statement of how far the facet is from the true patch, and it never admits an
input (CLAUDE.md, "may only FALSIFY"): `T` is trusted because the operand
recorded it, not because the polygons happen to sit near it.

## 3. The record

### 3.1 What a group carries

`boolean_mesh.go` fills each `facetGroup` from its operand face:

| Field | Value |
|---|---|
| `Surface` | the operand face's surface when §3.2 admits it, else nil |
| `Retained` | true when `Surface` came from an operand face; a structural builder's authenticated tag leaves it false |
| `Reversed` | the operand face's `Reversed`, flipped for the faces of the subtracted operand of a `Cut` |
| `NormalBound` | the tag's own departure `ε_F`: zero for a built operand face, the operand's `TagDeparture` for a retained one, the operand's own `NormalBound` for a denoted face (§3.3) |
| `Denoted` | the operand face's denoted patch, when it has one |
| `CapRecord`, `AxialDelta`, `HasAxialDelta` | for a `Plane` group, the operand face's. A Boolean never moves a plane, so the exact-plane record transfers verbatim; every placement and similarity drops it |

`Union` and `Intersect` keep both operands' orientations; `Cut` keeps the
target's and flips the tool's, because the tool's natural normal points into
the result. A second-generation Boolean reads the same fields from the
operand's face, so a tag survives a chain.

### 3.2 Admission

A group takes the tag only when all of these hold; a group that fails stays
`Faceted`, as now, and the Boolean does not refuse.

| # | Condition | Why |
|---|---|---|
| A1 | The operand face's surface is `Plane`, `Cylinder`, `Cone`, `Sphere` or `Torus` | Each has a normal whose sensitivity to a displaced point reads from its radii and, for a cone, its axial range (§3.3) |
| A2 | A built operand face with no denoted patch has `NormalBound == 0`; a retained face or a face with a denoted patch is admitted | A built face with a stated tilt (a ruled chamfer band, a tilted-frame cylinder) departs from its tag by more than the tilt, so its tag is not its geometry. A denoted patch proves the normal against the surface it denotes, which the result's patch still lies on, so the group carries the patch and `NormalAt` keeps judging against it |
| A3 | A `Sphere` group's patches all have a loop | A loop-less `Sphere` face is a closed analytic surface to `AuditSolidBoundary` (`internal/boundaryaudit/audit.go`) |
| A4 | After assembly, `2·δ_f / R < 1` for a curved group (§3.3) | Past that the normal bound is the trivial 2 and the tag says nothing a reader can use |
| A5 | The operand face is not already `Faceted` with no retained tag | Nothing to carry |

A4 is evaluated per built face, not per group: the group's patches may carry
different `δ_f`. A patch that fails it reads `Faceted`, and the group's other
patches keep their tag.

### 3.3 The face normal bound

`assemble.go` already computes `δ_f`, the largest facet bound over a built
face (`facetproof.FaceProofs`). For a retained face it sets

```
NormalBound_f = ε_F + L_T(δ_f)      (no denoted patch)
HeldNormalBound_f = L_T(δ_f)        (denoted patch; NormalAt adds it to the denoted proof)

L_plane    = 0
L_cylinder = L_sphere = 2·δ_f / R
L_cone     = cos(α) · 2·δ_f / ρ_min
L_torus    = 2·δ_f · (1 + 2M / (M − m)) / m
```

`ρ_min` is a lower bound on the distance from the cone's axis of every true
point of the patch. A cone's radius grows linearly along its axis, so it is
the radius at the lowest axial coordinate a true point can have: the lowest
held vertex coordinate less `δ_f`, less the rounding of the arithmetic. A
patch that may reach the apex has no bound and reads `Faceted`. The torus
uses `ρ ≥ M − m` for every true point, with `M` the major and `m` the minor
radius, and a torus with `M ≤ m` reads `Faceted`. The tube centre of a torus
point moves with its radial direction, which is where the factor
`1 + 2M / (M − m)` comes from. Every term is capped at 2 and rounds upward,
and A4 declines the face once the term reaches 1.

`ε_F` is zero for a built operand face and `TagDeparture` for a retained one,
so a second-generation Boolean recomputes the held term against its own `δ_f`
instead of adding the previous one. The cylinder and sphere term follows
from `|a/|a| − b/|b|| ≤ 2|a−b| / max(|a|,|b|)` applied to the radial parts
of a held point `p` and its nearest true point `q`: `|p−q| ≤ δ_f`, the
radial map is 1-Lipschitz, and `q`'s radial part has length `R`. Both terms
round upward through `proofbound`. `NormalAt` adds the tag's own arithmetic
allowance, as it does for every analytic variant
(`internal/topobuild/normal.go`).

### 3.4 Placement

`Transform` and `Copy` carry the tag to its float image
(`surfaceplace.TransformSurface`). A retained `Plane`, `Cylinder`, `Sphere` or
`Torus` group keeps its departure `ε_F` and adds the departure of the float
image from the exact image of the tag under the placement
(`facetedrecord.PlacedTagDeparture`). The held term of §3.3 is then recomputed
against the placed mesh's own `δ_f`, as for any Boolean result. A group that
is not retained, a retained `Cone`, a placement whose orthonormality defect
`e` exceeds `10⁻³`, and a displacement that reaches A4 keep the bound of 2.

Let the placement be the linear map `B` and the translation `t`, read as exact
rationals, with orthonormality defect `e` (`massmoment.MapCharge.Stretch`).

- **Plane.** `(B·U)×(B·V) = det(B)·B⁻ᵀ(U×V)`, so the exact image of the tag
  lies on the true normal's line. Only the rounding of the moved frame vectors
  remains. With `a` the largest distance of a moved vector from `B` applied to
  it, the crossed normal moves by under `3a`, and normalising by a length of at
  least 0.99 gives under `7a`. The charge is `8a + 4e + P`, with `P` the
  distance of the moved origin from the exact image of the origin. `4e` and `P`
  are not needed by the normal; they keep a zero bound meaning "the tag is the
  exact image plane", which `MirrorFace` reads (§4).
- **Cylinder, Sphere, Torus.** The true normal is `B⁻ᵀn` and the tag's is
  `B·n`. Their difference `B⁻ᵀ(I − BᵀB)n` has length at most `e/σ`, with
  `σ² ≥ 1 − e`, so the unit normals differ by at most `2e/σ² ≤ 4e`. The tag
  surface sits away from the true image by at most `P` for the origin, plus
  `(a + 2e)·D` for the moved axis and the stretch, where `D` bounds the
  distance from the tag's origin to any true point (the farthest held vertex
  plus the mesh bound and the largest vertex bound). The charge is `4e` plus
  the held term of §3.3 at twice that displacement.

A translation or a signed permutation has `e = 0` and `a = 0`, so the charge is
the origin's rounding alone, and a plane whose origin the translation leaves
exact adds nothing. The `CapRecord` follows the placement already, because the
placed plane is the record's image.

## 4. Readers

The existing readers fall into three groups. Each row says what the reader
does with a retained tag; the audit in §7 keeps the table complete.

| Reader | Reads | With a retained tag |
|---|---|---|
| `Face.Surface()`, `surfacegeom.SurfaceTypeName`, `decadtest` formatting | the tag | reports `Plane`/`Cylinder`/`Sphere` instead of `Faceted`; examples and tests that print `faceted` for these faces change (§8) |
| `Face.NormalAt` | tag + `NormalBound` | answers with the bound of §3.3 instead of `ErrUnsupported` |
| `internal/selectorquery/topo_views.go` planar and cylindrical views | the tag | a flat retained face matches `Facing`/`ParallelTo` by its plane normal, signed by `Reversed` |
| `internal/extent/to_face.go`, `angular_stop.go` | `Plane`, `NormalBound == 0`, `CapRecord`, `HasAxialDelta` | admits a retained `Plane` whose group carried `CapRecord` and `NormalBound == 0`; a placed or scaled body drops the record, so it refuses as before |
| `mirror.go` `resolveMirrorFace` | `Plane` | mirrors across a retained `Plane` with `NormalBound == 0`; a placed body's tag is a float image and keeps refusing |
| `Face.isPlanar` | `Plane` or `HeldPlanar` | unchanged; both stay true |
| `internal/contactfaceted/floor.go` `AxisSupportAtPose` | `HeldPlanar && Kind == Faceted` | **must change.** The predicate drops every retained plane from the support proof. It becomes `HeldPlanar && face is held geometry`, where a new `topo.Face.RetainedTag` flag marks a tag on a faceted payload (§5) |
| `internal/contactbox/`, `internal/contactround/`, `internal/contactruling/` | `Surface.(Plane/Cylinder/Sphere)` | gated by payload class (`prismPayload`, source cylinder) before they read a face; a faceted payload never reaches them. §7 pins this |
| `internal/clearance/stitch_carriers.go`, `internal/stackedrecord/mesh.go` | `Plane` | gated by `stitchPayload` / `stackedPrismPayload`; unreachable from a faceted payload |
| `export/step_analytic.go` `supportsAnalyticSTEP` | `Plane`/`Cylinder` plus each loop's edge curves | a retained face's edges are `FacetedCurve`, which the loop check refuses, so the whole body takes the faceted writer as now |
| `section.go` cardinal cone section | two faces, one `Cone`, one `Plane` | also requires the revolve payload record; a faceted payload never matches |
| `internal/boundaryaudit/audit.go` | `KindFaceted` for one-face loop-less images | A3 keeps a loop-less `Sphere` retained face out |

A reader that treats `Surface()` as the built geometry of a face's loops
(exact polygons in the plane, exact circle for a rim) is gated by payload
class and must stay so. `RetainedTag` is the one flag a reader may test when
it has to tell "tag that is the face's built geometry" from "tag on a face
whose geometry is held polygons".

## 5. Topology flag

`topo.Face` gains `RetainedTag bool`. `assemble.go` sets it on every face it
builds from a group that carries a `Surface`. `Reversed`, `NormalBound`,
`CapRecord` and `HasAxialDelta` already exist on `topo.Face`. `HeldPlanar`
stays as it is. Structural builders that already put a `Surface` on a
faceted group (point-loft cone trims, the smooth loft) set the flag through
the same path, and their `Denoted` patch continues to take precedence in
`NormalAt`.

## 6. What a caller sees

For row P3 of §1, with this design:

- `Face.Surface()` reports 6 `Plane` and 4 `Cylinder` faces for the
  untouched box faces and blends, and 2 `Cylinder` patches for the tool's
  wall; no face reads `Faceted`.
- `Face.NormalAt(p)` on a cut plane returns the plane normal with bound 0
  plus the tag's arithmetic allowance. On a blend cylinder it returns the
  radial direction with bound `2·δ_f / r` plus the allowance.
- From increment 2, an `Extrude` with a `ToFace` stop at an untouched planar
  face of the Boolean result builds, where today it refuses (§1 item 2).
- A selector `Faces(Facing(+z))` returns the top face by its plane normal.

Two readings change that are not improvements. A retained face no longer
publishes `Faceted.Bound`, its held displacement `δ_f`; a caller reads it from
the face's vertex bounds. `ScaledAt` keeps a retained `Plane`, `Cylinder`,
`Sphere` or `Torus` at its similar image, charged for rounding only
(`agent-docs/retained-modify-reach-design.md` §7). A retained `Cone` or
denoted face reads `Faceted` afterwards, because the scaled tag has no
similarity proof here.

What does not change: the held polygons, `Volume`, `Area`, `Centroid`,
`Box`, every vertex and edge bound, `Verify` results, and the tessellation.
An edge between two retained faces publishes the analytic curve of §10
(`Line3`, `Circle3` or `Arc3`) where an admission rule holds, and its length
reading is unchanged.

## 7. Tests and the red leg each must show

Each test asserts on computed geometry. Each is shown red first by deleting
the clause named.

| Test | Asserts | Red leg |
|---|---|---|
| P3 fixture | the 12 faces of §6 by kind, `Reversed` on the tool wall, plane normals to the bit | stop setting `Reversed` for the tool of a `Cut`: the tool wall's `NormalAt` points into the part |
| Cut / Union / Intersect orientation | `NormalAt` of every retained face points out of the material (sampled at the centroid of an interior facet) | invert the `Cut` flip: the hole wall's normal points into the void |
| Second generation | a Boolean of P3's result keeps its untouched planes and blends | read the operand's `Surface` through `Faceted` only: the chain loses the tags |
| A4 refusal of the tag | a patch with `δ_f ≥ R/2` reads `Faceted`; its siblings keep the tag | drop A4: `NormalAt` reports a bound of 2 on a face that claims an analytic cylinder |
| A3 | a Boolean that leaves one loop-less sphere face reads `Faceted` | drop A3: `AuditSolidBoundary` accepts a body whose only face is a retained loop-less sphere |
| `internal/contactfaceted/floor.go` predicate | the support proof still finds the floor face of a retained-plane body | keep the old `Kind == Faceted` test: the proof finds no valid face and the contact returns `ContactPayloadUnsupported` |
| `ToFace` on a retained plane | the stop level equals the plane's exact coordinate; `AxialDelta` is the operand's | clear `CapRecord` on the group: `ToFace` refuses |
| Placement | a translated and a 20°-rotated copy of P3 report `NormalAt` bounds within 1e-9 of the unplaced copy's, and the rotated normal lies within the two bounds of the rotated reading; a rotated plane never reads as an exact mirror plane | keep the old constant 2: the placed `NormalAt` bound is 2 |
| Reader audit | a test lists every non-test file under the module that matches `Surface\.\(surfacegeom\.` or `Surface\(\)\.\(`, and fails on a file this document's §4 does not name | add a reader without a §4 row: the audit fails |
| Existing assertions | the 23 `decad.Faceted` / `KindFaceted` assertions under `apitest/` and `examples/` (for example `apitest/mirror_test.go`, whose premise is a Boolean wall that carries no `Plane`) are rewritten to the retained kind where §3.2 admits the face, and kept where it declines | run the suite before the change; a test that still passes after it, on a face §3.2 admits, asserts the wrong thing |

Tests do not pin a bound or a volume by literal: float bounds are
architecture-specific (FMA differs between amd64 and arm64), so tests
assert relations (`NormalBound_f ≥ 2·δ_f / R`, equal to the recomputed
expression) and not a printed number.

## 8. Files and fallout

| Area | Change |
|---|---|
| `boolean_mesh.go` | fill `facetGroup.Surface`, `Retained`, `Reversed`, `NormalBound` from the operand face (§3.1, §3.2) |
| `internal/facetedrecord/assemble.go` | compute `NormalBound_f` per built face, apply A4, set `RetainedTag` |
| `internal/facetedrecord/place_tag.go`, `placed.go` | placement rule of §3.4 |
| `internal/topo/face.go` | `RetainedTag` |
| `internal/contactfaceted/floor.go` | predicate of §4 |
| `internal/facetedrecord/edge_curve.go`, `topology.go`, `internal/topo/edge.go` | edge curves and their witness flag (§10, increment 3) |
| `internal/meshbool/plane_slab.go`, `internal/booleaneval/retained_plane.go` | slab bound of a retained plane facet (§11, increment 5) |
| `mirror.go`, `internal/extent` | admit a retained `Plane` per §4 |
| `internal/selectorquery/topo_views.go` | no code change expected; the existing views read the tag |
| Tests | the `Faceted` assertions of §7's last row |
| Docs | `agent-docs/evaluator-design.md` §9 (group fields), `agent-docs/api-design.md` §6.1 (the tagged-stand-in paragraph), `agent-docs/faceted-vertex-bounds-design.md` §4.3 (placement), `agent-docs/payload-verification-design.md` §5.2 and §13 (the source identity half of the faceted certificate), `agent-docs/missing-features.md` (the row for `NormalAt` on a faceted face, if present), and this document's row in `agent-docs/layout/design-docs.md` |

## 9. Increments

Increments 1 to 6 are in. Nothing in 1–4 depends on 5 or 6.

| # | Lands | Gate to start the next |
|---|---|---|
| 1 | §3.1–§3.3 for `Plane`, `Cylinder`, `Sphere`; `RetainedTag`; the `internal/contactfaceted/floor.go` predicate; `NormalAt`, selector and `Surface()` readers; the reader audit; test updates | the P3 fixture and the generation-two fixture pass; `go test ./...` on CI |
| 2 | `Cone` and `Torus`, faces with a denoted patch (revolved walls and caps), and `ToFace` / `MirrorFace` on a retained plane | a revolved shaft keeps its cone, cylinder and caps through a cut |
| 3 | §10: `Line3`, `Circle3` and `Arc3` on an edge between two retained faces, the `Curve` reader audit and the Boolean-rim witness flag | the P3 fixture's edges publish their curves and the STEP test passes |
| 4 | Placement rule of §3.4 | a rotated copy of P3 reports a finite `NormalAt` bound below 2 |
| 5 | A later mesh Boolean charges a retained `Plane` facet's slab half-width by its measured distance from `T` instead of `β` (§11) | a second cut beside the first hole, previously refused by the held-mesh gate, builds with rim bounds below the first cut's |

| 6 | §10.5–§10.6: `Ellipse3` on an oblique `Plane`/`Cylinder` rim, with its sweep rule and the whole-turn convention | the oblique plate fixture publishes its two mouth ellipses |

Increments 1–4 only publish what Claim R and Claim E already prove (increment 4 adds the placement charge of §3.4).
Increment 5 uses the tag to tighten later arithmetic; its proof is Claim S in §11.

## 10. Edge curves

An edge of a Boolean result is a chain of held chords between two faces and
publishes `FacetedCurve` (`internal/facetedrecord/topology.go`). When both
faces carry a retained tag, the true edge lies on the intersection of the two
tags, and the tags are exact recorded surfaces. The edge then publishes that
curve instead.

**Claim E.** Let an edge's two faces be distinct retained faces with tags
`T1` and `T2`. The true edge is a connected piece of `T1 ∩ T2`, because a rim
between two faces lies on both of their true surfaces (Claim R) and a chain is
one connected rim between two face patches. When `T1 ∩ T2` has several
components (two generators of a cylinder cut by a plane), the chain follows
one of them.

### 10.1 Admission

`facetedrecord.RetainedEdgeCurve` decides it from the two faces and the
chain's held vertices with their bounds. An edge that no rule admits keeps its
chord chain; nothing refuses.

| # | Condition | Curve |
|---|---|---|
| E1 | The faces are distinct and both `RetainedTag` | |
| E2 | `Plane` and `Plane` with normals not parallel (`|n1 × n2| > 10⁻⁹`) | `Line3` |
| E3 | `Plane` and `Cylinder` with the axis normal to the plane (`|axis × n| ≤ 10⁻¹²`) | `Circle3` for a closed rim, `Arc3` for an open one: centre the axis-plane crossing, radius `R`, axis `±n` |
| E4 | `Plane` and `Cylinder` with the axis in the plane (`|axis · n| ≤ 10⁻¹²`) and the plane at distance `s` from the axis with `R² − s²` not below `−slack·R` | `Line3`. Two generators exist at `±√(R² − s²)` from the axis's foot, and a tangent plane gives one; either passes if the falsifier accepts it |
| E5 | `Plane` and `Sphere` with `R² > d²` for `d` the centre's distance from the plane | `Circle3` or `Arc3`: centre the foot of the sphere's centre, radius `√(R² − d²)` |
| E6 | The falsifier: every chain vertex lies within `bound(v) + slack` of the curve, `slack = 64·2⁻⁵²·max(1, |coordinate|)` | |
| E7 | For an `Arc3`, the chain has an interior vertex, every angular step about the axis clears `4(max bound + slack)/R`, stays below `π`, has one sign, and the steps sum below `2π` | the sign picks `Axis = ±n` so the run is counter-clockwise |
| E8 | `Plane` and `Cylinder` with the axis oblique to the plane: `|axis × n| > 10⁻¹²` and `|axis · n| ≥ 10⁻³` | `Ellipse3`, §10.5 |
| E11 | `Cylinder` of radius `r` and `Torus` of tube radius `r` (equal within slack) with the cylinder axis `d` normal to the torus axis `a` (`|d · a| ≤ 10⁻¹²`), where `c` is the point of the cylinder's axis nearest the torus centre `C`: `c − C` lies in the torus's equatorial plane (`|(c − C) · a| ≤ slack`) at distance `Major` from `C` (within slack) | `Arc3` about `c` of radius `r`, axis `±d`, or `Circle3` for a closed rim. This is the junction of two retained blends (reach §5.4); the falsifier E6 and the sweep rule E7 apply |

A `Line3` carries no parameters, so E2 and E4 assert only that the edge is
straight between its vertices. The other pairs (`Cylinder`/`Cylinder`, `Cone`,
any other `Torus` pairing, and a `Plane`/`Cylinder` pair E8 declines) keep `FacetedCurve`.

E6 can only reject. A vertex far from the curve disproves the pairing (a bug,
or a chain that does not follow the intersection), and the edge stays a chord
chain. A vertex near the curve proves nothing; the curve is trusted because
Claim R holds for both faces. This is the direction CLAUDE.md allows for a
residual.

### 10.2 What the curve says and does not say

The curve parameters (`Center`, `Axis`, `Radius`) are the exact intersection of
two recorded surfaces, floats rounded once. Vertex positions stay the held
positions with their `Bound`. An edge therefore runs between two vertices that
may sit up to their bound off the curve, and a reader that wants the curve
reads the curve, not the endpoints. `topo.Edge.CurveBounded` stays false: the
edge states no `CurveBound`, so a reader that needs one refuses as before.
`Length`, `LengthBound` and `LengthUnbounded` keep their chain values, and a
rim on a curved source still refuses `Length`.

`topo.Edge.RetainedCurve` marks the edge and `ChainBound` keeps the chord
bound the curve replaced. Three modify routes take a `FacetedCurve` as the
witness that an edge is a Boolean rim (`internal/brepmodify/prism_union.go`,
`oblique_boss_fillet.go`, `cross_cylinder_bore.go`); each accepts a retained
curve in its place.

### 10.3 Readers

The curve readers split the same way the surface readers did in §4.

| Reader | With a retained edge curve |
|---|---|
| `internal/selectorquery/topo_views.go` | `ParallelTo`, `Circular` and `EndpointAt` match a retained edge by its curve |
| `internal/modifyselect/edge_context.go` | a refusal message renders a retained closed rim as `closed circle through (p), centre (c), radius r mm` |
| `export/step_analytic.go` | `supportsAnalyticSTEP` requires the circle seams of a cylinder wall to align exactly. Held rim vertices do not, so a retained body still writes one planar face per triangle |
| `internal/brepmodify/prism_union.go`, `oblique_boss_fillet.go`, `cross_cylinder_bore.go` | accept `RetainedCurve` as the Boolean-rim witness (§10.2) |
| `internal/brepmodify`, `internal/modifyselect` | read the curve after a payload-class gate. A `facetedPayload` stays `ErrUnsupported` at modify S3/SX9, so the new curve opens no modify route by itself |
| `internal/extent/angular_stop.go`, `patch_body.go`, `internal/clearance/edge_topology.go`, `internal/gatediameter/body_gate.go` | each is gated by its own payload class |
| `Edge.Length`, `LengthBound` | unchanged (§10.2) |
| `internal/selectorquery/topo_views.go` (`Circular`), `internal/modifyselect/edge_context.go`, `export/step_analytic.go` for an `Ellipse3` rim | `Circular` does not match an ellipse; the refusal message keeps its existing ellipse text for an open arc and gains `closed ellipse through (p), centre (c), semi-axes a and b` for a whole turn; the STEP loop check refuses a retained body's edges as for any `FacetedCurve` |

Tests that must tell a mesh result from an analytic one cannot use a face or
edge kind any more. `decadtest.PayloadClass` names the payload class through a
hook (`internal/payloadclass`) that the root package installs; it adds no
public API.

### 10.4 Tests

| Test | Asserts | Red leg |
|---|---|---|
| Drilled filleted box | 16 `Line3`, 8 `Arc3` of radius 3 about z, 2 `Circle3` of radius 4 about y through (20, ·, 10) | return the chord chain unconditionally: no edge is a line, arc or circle |
| `RetainedEdgeCurve` unit tests | plane-plane and plane-cylinder (tangent generator, bore circle) accept; a mispaired chain, a face meeting itself, an untagged face and parallel planes decline; a counter-clockwise run sweeps about +z and a clockwise one about −z; two vertices or a doubled-back chain decline | drop E6: the 0.5 mm-off bore chain publishes a circle |
| STEP | a retained drilled plate still writes planar triangles and no analytic entity | none; the test pins that the analytic writer stays off |

### 10.5 The oblique plane and cylinder

A plane with unit normal `n` through `o` meets a cylinder with unit axis `a`,
origin `c` and radius `R` in one closed ellipse when `a · n ≠ 0`. Write
`k = |a · n|` and let `m` be the unit vector along `a − (a · n)·n`, the axis
projected into the plane. The ellipse is

| Field of `Ellipse3` | Value |
|---|---|
| `Center` | `c + t·a` with `t = n · (o − c) / (n · a)`, the axis-plane crossing |
| `Major` | `m` |
| `SemiMajor` | `R / k` |
| `SemiMinor` | `R` |
| `Axis` | `±n`, chosen as in E9 |

*Check.* Take `p = Center + x·m + y·(n × m)` in the plane. The part of `p − Center`
along `a` is `x·√(1 − k²)`, so the squared distance of `p` from the axis is
`x² + y² − x²(1 − k²) = x²k² + y²`. Setting it equal to `R²` gives
`(x / (R/k))² + (y / R)² = 1`. ∎

E8 stops at `k ≥ 10⁻³`. Below that the centre's `1/k` amplification of the
rounding in `o`, `c` and `a` exceeds what E6's slack covers, and the pair keeps its
chord chain. The pair has no second component, so there is no choice of branch as
in E4.

**E6 for an ellipse.** The distance from a point to an ellipse has no closed form.
The falsifier uses two necessary conditions instead: a vertex within
`bound + slack` of the ellipse is within that distance of the plane,
`|n · (v − o)| ≤ bound + slack`, and of the cylinder,
`| |radial(v)| − R | ≤ bound + slack`. Either one failing disproves the pairing. A
vertex that passes both may still be farther from the ellipse than its bound (the
distance to the curve is at most `bound / sin γ` for `γ` the angle between the two
surfaces there), so the test rejects less than the circle's. It admits nothing
(§10.1, E6): the curve is trusted because Claim R holds for both faces.

**E9 sweep sense.** Let `w = n × m`. A chain vertex `v` has the eccentric angle
`φ = atan2(w · (v − Center), k·(m · (v − Center)))`, the angle `Ellipse3` measures. A
vertex displaced by `b` moves `φ` by at most `b / R`, because the curve's speed per
unit of `φ` is at least `SemiMinor = R`. E7 then holds with these angles: every
step clears `4(max bound + slack) / R`, stays below `π`, has one sign, and the steps
sum below `2π`. The sign picks `Axis = n` for a positive run and `−n` for a negative
one. The edge runs from its start vertex to its end vertex counter-clockwise about
`Axis`, as `Arc3` does.

**Closed rims.** A closed rim publishes one whole ellipse, and the edge's start
vertex equals its end vertex. For `Ellipse3`, start equal to end means one full
turn from that vertex, the way a `Circle3` edge reads. `agent-docs/api-design.md` §4
states the convention in `Ellipse3`'s comment.

`supportsAnalyticSTEP` returns false for a face whose loop holds a whole-turn
`Ellipse3`: a planar loop admits `Line3`, `Arc3` and a closed `Circle3` only, and
a cylinder wall with two loops needs closed circles. The faceted writer then
takes the body.

### 10.6 Tests for E8 and E9

| Test | Asserts | Red leg |
|---|---|---|
| Plate drilled 30° off the normal by a placed tool | both mouths publish `Ellipse3` with `SemiMinor = R`, `SemiMajor = R / cos 30°`, `Center` on the tool axis, `Major` along the tilt, to the bit of the closed-form value; the closed rims are whole turns | return `FacetedCurve` for every oblique pair: no edge is an ellipse |
| `RetainedEdgeCurve` unit tests | an open chain sweeping counter-clockwise publishes `Axis = +n` and a clockwise one `−n`; a chain 0.5 mm off the cylinder declines; a chain 0.5 mm off the plane declines; `k = 5·10⁻⁴` declines; a doubled-back chain declines | drop the cylinder half of E6: the off-cylinder chain publishes an ellipse; negate the sweep sign: the clockwise chain reads counter-clockwise |

## 11. Why the hint helps a later Boolean, and its limit

A later Boolean refuses where its new contact lands on a facet of the earlier
result whose published bound `FacetBound` exceeds the pair's chord tolerance
(`agent-docs/faceted-vertex-bounds-design.md` §5), and composes each rim vertex it
creates from the two contacting facets' bounds (`meshbool.RimBound`). A planar
facet next to an earlier cut's rim has a corner on that rim, so it carries the
rim's bound, though its three corners lie in the plane. The probe of
2026-10-20 ran 374 second cuts after the P3 drill. The held-mesh gate refused
319 of them, and 287 of those refusals were on facets of a retained `Plane`.
The hidden-tangency pre-pass (`RefuseUndecidableProximity`) refused none.

**What RimBound reads.** Its proof (`agent-docs/faceted-vertex-bounds-design.md`
§3.2) needs only that the true piece of each facet lies within a slab about the
facet's own plane. `FacetBound` is a sufficient half-width. For a facet of a
retained `Plane` a thinner one holds.

**Claim S.** Let a facet have its true piece in the plane `T` (Claim R), within
`β = FacetBound` of the facet. Let `h` be the largest distance of the facet's
corners from `T`, and `φ` the angle between the facet's plane `P` and `T`.
Then the true piece lies within `h + β·sin φ` of `P`.

*Proof.* The distance to a plane is convex, so every point `y` of the facet
lies within `h` of `T`. Take `x` in the true piece and `y` in the facet with
`|x − y| ≤ β`, and let `y_T` be the projection of `y` onto `T`. The projection
does not lengthen distances, so `|x − y_T| ≤ β`. The distance to `P` is affine
on `T` with slope `sin φ`, and `dist(y_T, P) ≤ |y_T − y| ≤ h`. So
`dist(x, P) ≤ h + β·sin φ`. ∎

`h` and `sin φ` are computed in exact rational arithmetic from the held
floats and the tag's frame, and rounded up (`meshbool.SetSlabBounds`). The
slab bound is `min(β, h + β·sin φ)`. In-plane displacement of the facet's
boundary stays in `β`: the slope term charges it, so the claim needs no
statement about where the true patch ends.

**Where it applies.** A facet takes the slab bound only when its source face is
a `RetainedTag` `Plane` with `NormalBound == 0` and no denoted patch. A placed
tag with a stated departure (§3.4) is a float image whose plane may differ from
the true one by an offset, so it keeps `β`. The slab bound replaces `β` in the
held-mesh gate (`RefuseCoarseHeldContact`) and in `RimBound`. A vertex the
cutter places on a facet's interior keeps `β`, because its true boundary point
may sit anywhere in the plane within `β`. A curved retained group has no such
split, so a cylinder or sphere keeps `β`. In the probe, the 32 refusals whose
worst facet was a cylinder stay refused.

## 12. Rejected shortcuts

- **Fit a plane or cylinder to a group's polygons.** It blesses a surface
  from the polygons it should be checked against (CLAUDE.md). The tag comes
  from the operand's record only.
- **Reconstruct the tag from `Origins`.** Roles name the creating feature,
  not the surface; payload §5.2 forbids it.
- **Retain the tag and set `NormalBound` to 0 for curved faces.** `NormalAt`
  would claim the radial direction of the held point exactly while the point
  sits up to `δ_f` off the surface.
- **Retain an operand's `Denoted` patch.** The patch proof reads the
  operand's record, which the generic Boolean does not copy. The face stays
  `Faceted` instead of carrying a stale proof.
- **Refuse the Boolean when a group fails admission.** Admission affects what
  a face reports. The Boolean's own proof is unchanged, so refusing would
  remove a result that builds today.
- **Re-tessellate a retained surface to a finer chord in a later Boolean.**
  The trim region of a cut face is the held rim, which is not an analytic
  curve. A finer facet set would need the true trim, which this design does
  not record.

## 13. Decisions the user may want to overturn

| Decision | Default | Alternative |
|---|---|---|
| Declining a group that fails A4 instead of retaining it with a bound of 2 | decline (`Faceted`) | keep the tag with `NormalBound` 2: callers see the surface kind, `NormalAt` is useless |
| `Cone` and `Torus` | increment 2 | in increment 1; blocked only by the axis-distance lower bound |
| `ToFace` on a retained plane | increment 2, with the carried `CapRecord` | increment 1; needs a `CapRecord` for every operand plane first |
| `Line3`/`Circle3` on edges | increment 3, after 1 and 2 | never; keeps `FacetedCurve` everywhere |
| Fillet and chamfer of an edge between two retained planes | out of scope here; needs its own route in `agent-docs/modify-reach-design.md` (SX9) | include as increment 7 |
| Oblique `Plane`/`Cylinder` edges as `Ellipse3` | increment 6, with the whole-turn convention for a closed rim (§10.5) | keep `FacetedCurve`, or add a separate whole-ellipse curve type |
