# Faceted Source Retention Design

A mesh Boolean result stores, for each of its faces, the analytic surface of
the operand face it came from. Today every such face reads `Faceted`, so a
filleted cube that a tilted tool drilled reports its six planes and four
blend cylinders as polygons. This design makes each face that descends from
one operand face publish that face's surface, and lets the readers that need
an exact plane or a bounded normal use it. Companion to
`docs/evaluator-design.md` §9 (the mesh Boolean),
`docs/faceted-vertex-bounds-design.md` ("vertex bounds §N": per-vertex
displacement) and `docs/payload-verification-design.md` §5 ("payload §N": the
faceted boundary certificate this design supplies the source identity for).
"core §N" is `docs/api-design.md`.

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
| `NormalBound` | the tag's own departure `ε_F`: zero for a built operand face, the operand's `TagDeparture` for a retained one (§3.3) |

Increment 2 adds `CapRecord`, `AxialDelta` and `HasAxialDelta` for a `Plane`
group. A Boolean never moves a plane, so the exact-plane record transfers
verbatim.

`Union` and `Intersect` keep both operands' orientations; `Cut` keeps the
target's and flips the tool's, because the tool's natural normal points into
the result. A second-generation Boolean reads the same fields from the
operand's face, so a tag survives a chain.

### 3.2 Admission

A group takes the tag only when all of these hold; a group that fails stays
`Faceted`, as now, and the Boolean does not refuse.

| # | Condition | Why |
|---|---|---|
| A1 | The operand face's surface is `Plane`, `Cylinder` or `Sphere` (§9 increment 2 adds `Cone` and `Torus`) | These have a normal Lipschitz constant read from one radius (§3.3) |
| A2 | `Denoted == nil` on the operand face, and `NormalBound == 0` unless the face is itself a retained tag | A denoted patch proves its normal against a record the generic Boolean does not carry. A built face with a stated tilt (a ruled chamfer band, a tilted-frame cylinder) departs from its tag by more than the tilt, so its tag is not its geometry |
| A3 | A `Sphere` group's patches all have a loop | A loop-less `Sphere` face is a closed analytic surface to `AuditSolidBoundary` (`internal/reportvocab/boundary_audit.go`) |
| A4 | After assembly, `2·δ_f / R < 1` for a curved group (§3.3) | Past that the normal bound is the trivial 2 and the tag says nothing a reader can use |
| A5 | The operand face is not already `Faceted` with no retained tag | Nothing to carry |

A4 is evaluated per built face, not per group: the group's patches may carry
different `δ_f`. A patch that fails it reads `Faceted`, and the group's other
patches keep their tag.

### 3.3 The face normal bound

`assemble.go` already computes `δ_f`, the largest facet bound over a built
face (`facetproof.FaceProofs`). For a retained face it sets

```
NormalBound_f = ε_F + L_T(δ_f)
L_plane = 0          L_cylinder = L_sphere = min(2, 2·δ_f / R)
```

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
(`surfaceplace.TransformSurface`) and today charge the placed group a normal
bound of 2 unless the placement is the identity (vertex bounds §4.3). With
retention this makes every placed Boolean result report an unusable
`NormalAt`. Increment 4 replaces the 2 for a translation or a signed
permutation, whose image is exact, with `NormalBound_f` unchanged, and for a
general rotation with `NormalBound_f` plus the rotation's certified
orthonormality defect (`massmoment.MapCharge`). The `CapRecord` follows the
placement already, because the placed plane is the record's image.

## 4. Readers

The existing readers fall into three groups. Each row says what the reader
does with a retained tag; the audit in §7 keeps the table complete.

| Reader | Reads | With a retained tag |
|---|---|---|
| `Face.Surface()`, `reportvocab.SurfaceTypeName`, `decadtest` formatting | the tag | reports `Plane`/`Cylinder`/`Sphere` instead of `Faceted`; examples and tests that print `faceted` for these faces change (§8) |
| `Face.NormalAt` | tag + `NormalBound` | answers with the bound of §3.3 instead of `ErrUnsupported` |
| `internal/selectorquery/topo_views.go` planar and cylindrical views | the tag | a flat retained face matches `Facing`/`ParallelTo` by its plane normal, signed by `Reversed` |
| `internal/extent/to_face.go`, `angular_stop.go` | `Plane`, `NormalBound == 0`, `CapRecord`, `HasAxialDelta` | admits a retained `Plane` whose group carried `CapRecord` and `NormalBound == 0` |
| `mirror.go` `resolveMirrorFace` | `Plane` | keeps refusing a retained `Plane` until increment 2: the tag is a stand-in, not an exact mirror plane |
| `Face.isPlanar` | `Plane` or `HeldPlanar` | unchanged; both stay true |
| `contact_faceted_floor.go` `sourceFacetedAxisSupport` | `HeldPlanar && Kind == Faceted` | **must change.** The predicate drops every retained plane from the support proof. It becomes `HeldPlanar && face is held geometry`, where a new `topo.Face.RetainedTag` flag marks a tag on a faceted payload (§5) |
| `contact_box.go`, `contact_oriented_box.go`, `contact_analytic_manifold.go`, `contact_sphere.go` | `Surface.(Plane/Cylinder/Sphere)` | gated by payload class (`prismPayload`, source cylinder) before they read a face; a faceted payload never reaches them. §7 pins this |
| `internal/clearance/stitch_carriers.go`, `internal/stackedrecord/mesh.go` | `Plane` | gated by `stitchPayload` / `stackedPrismPayload`; unreachable from a faceted payload |
| `export/step_analytic.go` `supportsAnalyticSTEP` | `Plane`/`Cylinder` plus each loop's edge curves | a retained face's edges are `FacetedCurve`, which the loop check refuses, so the whole body takes the faceted writer as now |
| `section.go` cardinal cone section | two faces, one `Cone`, one `Plane` | also requires the revolve payload record; a faceted payload never matches |
| `internal/reportvocab/boundary_audit.go` | `KindFaceted` for one-face loop-less images | A3 keeps a loop-less `Sphere` retained face out |

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
the face's vertex bounds. `ScaledAt` keeps a retained `Plane` and reads a
retained `Cylinder` or `Sphere` as `Faceted` afterwards, because the scaled
tag has no similarity proof here.

What does not change: the held polygons, `Volume`, `Area`, `Centroid`,
`Box`, every vertex and edge bound, `Verify` results, and the tessellation.
An edge between two retained planes still reports `FacetedCurve` (its chord
length is already exact for planar sources, `topo.Face.HeldPlanar`);
publishing a `Line3` there is increment 3 (§10).

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
| `contact_faceted_floor.go` predicate | the support proof still finds the floor face of a retained-plane body | keep the old `Kind == Faceted` test: the proof finds no valid face and the contact returns `ContactPayloadUnsupported` |
| `ToFace` on a retained plane | the stop level equals the plane's exact coordinate; `AxialDelta` is the operand's | clear `CapRecord` on the group: `ToFace` refuses |
| Placement | translation and signed-permutation placements keep `NormalBound_f`; a 20° rotation adds the defect and no more | keep the old constant 2: the placed `NormalAt` bound is 2 |
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
| `internal/facetedrecord/place.go`, `placed.go` | placement rule of §3.4 |
| `internal/topo/face.go` | `RetainedTag` |
| `contact_faceted_floor.go` | predicate of §4 |
| `internal/facetedrecord/topology.go`, `export/step_analytic.go` | edge curves and the STEP bound gate of §10 (increment 3) |
| `mirror.go`, `internal/extent` | increment 1 keeps refusing a retained `Plane`; increment 2 admits it per §4 |
| `internal/selectorquery/topo_views.go` | no code change expected; the existing views read the tag |
| Tests | the `Faceted` assertions of §7's last row |
| Docs | `docs/evaluator-design.md` §9 (group fields), `docs/api-design.md` §6.1 (the tagged-stand-in paragraph), `docs/faceted-vertex-bounds-design.md` §4.3 (placement), `docs/payload-verification-design.md` §5.2 and §13 (the source identity half of the faceted certificate), `docs/missing-features.md` (the row for `NormalAt` on a faceted face, if present), and this document's row in `docs/layout/design-docs.md` |

## 9. Increments

Increments 1, 2 and 3 are the priority order. Increments 4 and 5 stay
planned; nothing in 1–3 depends on them.

| # | Lands | Gate to start the next |
|---|---|---|
| 1 | §3.1–§3.3 for `Plane`, `Cylinder`, `Sphere`; `RetainedTag`; the `contact_faceted_floor.go` predicate; `NormalAt`, selector and `Surface()` readers; the reader audit; test updates | the P3 fixture and the generation-two fixture pass; `go test ./...` on CI |
| 2 | `Cone` and `Torus` (needs a lower bound on a patch's distance from the axis for `L_T`) and `ToFace` / `MirrorFace` on a retained plane | a cone-capped part keeps its cone through a cut |
| 3 | §10: `Line3`, `Circle3` and `Arc3` on an edge between two retained faces, the `Curve` reader audit, and the STEP bound gate | the P3 fixture's edges publish their curves and the STEP test passes |
| 4 | Placement rule of §3.4 | a rotated copy of P3 reports a finite `NormalAt` bound below 2 |
| 5 | A later mesh Boolean charges a retained `Plane` group's out-of-plane displacement by its measured distance from `T` instead of `δ_f` (§11) | measured: a second cut near the first cut's rim, previously refused as hidden tangency, builds |

Increments 1–4 only publish what Claim R and Claim E already prove.
Increment 5 uses the tag to tighten later arithmetic and needs its own proof,
written in this document before it lands.

## 10. Edge curves

An edge of a Boolean result is a chain of held chords between two faces and
publishes `FacetedCurve` (`internal/facetedrecord/topology.go`). When both
faces carry a retained tag, the true edge lies on the intersection of the two
tags, and the tags are exact recorded surfaces. The edge can publish that
curve instead.

**Claim E.** Let an edge's two faces be distinct retained faces with tags
`T1` and `T2`, and let `C` be a connected component of `T1 ∩ T2`. If the
edge's chain follows `C`, the true edge is a connected piece of `C`, because
a rim between two faces lies on both of their true surfaces (Claim R) and a
chain is one connected rim between two face patches.

### 10.1 Admission

An edge publishes an analytic curve only when all of these hold; otherwise it
keeps `FacetedCurve` and nothing refuses.

| # | Condition | Curve |
|---|---|---|
| E1 | Both faces are distinct and `RetainedTag` | |
| E2 | `Plane` and `Plane`, normals not parallel (`n1 × n2` not zero in exact arithmetic over the stored floats) | `Line3`, direction `n1 × n2` |
| E3 | `Plane` and `Cylinder` with the axis normal to the plane (`axis × n` zero) | `Circle3` or `Arc3`: centre the axis-plane crossing, radius `R`, axis `±n` |
| E4 | `Plane` and `Cylinder` with the axis in the plane (`axis · n` zero) and the plane at distance `d < R` from the axis | `Line3` along the axis, at the generator nearer the chain's vertices; two generators exist, so the nearer must beat the other by more than `2·max β` over the chain, else decline |
| E5 | `Plane` and `Sphere`, distance `d` from the centre to the plane with `R − |d|` above `2·max β` over the chain | `Circle3` or `Arc3`: centre the foot of the sphere centre, radius `√(R² − d²)`, axis `±n` |
| E6 | A falsifier: every chain vertex lies within `β(v)` plus the arithmetic allowance of the curve | |
| E7 | For an `Arc3`, the sweep sense about the axis is the same at every interior chain vertex and decided beyond `β` | |

E6 can only reject. A vertex that sits far from the curve disproves the
pairing (a bug, or a chain that does not follow `C`), so the edge stays
`FacetedCurve`. A vertex that sits near the curve proves nothing; the curve
is trusted because Claim R holds for both faces. This is the direction
CLAUDE.md allows for a residual.

E3 and E4 handle the faces a filleted box and a drilled plate produce:
a blend cylinder meets an untouched flat face along a generator, and a
perpendicular hole meets a flat face along a circle. Other pairs
(`Cylinder`/`Cylinder`, oblique `Plane`/`Cylinder`, `Cone`, `Torus`) keep
`FacetedCurve`; an oblique `Plane`/`Cylinder` pair would publish `Ellipse3`
and is not in this increment.

### 10.2 What the curve says and does not say

The curve parameters (`Center`, `Axis`, `Radius`, the line's direction) are
the exact intersection of two recorded surfaces, floats rounded once.
Vertex positions stay the held positions with their `Bound`. A `Line3` edge
therefore runs between two vertices that may sit up to `β` off the exact line,
and a reader that wants the line reads the curve, not the endpoints. The
edge's `Length`, `LengthBound` and `LengthUnbounded` keep their chain values:
the chain length remains a proven bound for the true length, and tightening
it to `R·θ` for an arc is a separate step.

### 10.3 Readers

The curve readers (44 sites outside the payload builders) split the same way
the surface readers did in §4.

| Reader | With a retained edge curve |
|---|---|
| `internal/selectorquery/topo_views.go` | `ParallelTo`, `Circular` and `EndpointAt` match a retained edge by its curve; a bounded endpoint match still uses the vertex bound |
| `export/step_analytic.go`, `step_analytic_torus.go` | `supportsAnalyticSTEP` would admit a face set whose edges are all analytic. A vertex off its curve by `β` is a STEP vertex-on-curve mismatch, so the writer admits a retained body only when every vertex bound is below the writer's coordinate tolerance, and otherwise takes the faceted writer as now |
| `internal/modifyselect`, `brep_modify_*`, `chamfer.go`, `cap_edge_cutter.go`, `cross_cylinder_bore.go`, `oblique_boss_fillet.go`, `prism_union_fillet.go` | read the curve after a payload-class gate. A `facetedPayload` stays `ErrUnsupported` at modify S3/SX9, so the new curve opens no modify route by itself |
| `internal/extent/angular_stop.go`, `patch_body.go`, `internal/clearance/edge_topology.go`, `internal/gatediameter/body_gate.go` | each is gated by its own payload class; §7's reader audit extends to `Curve` readers and lists them |
| `Edge.Length`, `LengthBound` | unchanged (§10.2) |

### 10.4 Tests

| Test | Asserts | Red leg |
|---|---|---|
| P3 fixture edges | the box's untouched x-edges are `Line3` along the exact `n1 × n2`; the drill's rim on a flat face is `Circle3` with the tool's radius and centre | return `FacetedCurve` unconditionally: the type assertion fails |
| E4 two-generator case | a blend cylinder against the flat face resolves to the near generator | pick the first candidate: a fixture that lists the far generator first publishes the wrong line |
| E6 falsifier | a deliberately mispaired chain (a vertex 10·`β` off the curve) keeps `FacetedCurve` | drop E6: the mispaired edge publishes a line |
| E7 sense | an arc over 200° reports the right sweep sign | decide the sense from the endpoints alone: a 200° arc sweeps the wrong way |
| STEP | a retained body with tight vertex bounds writes analytic faces; one with `β` above the tolerance writes faceted | drop the bound gate: the second body writes analytic faces with vertices off their curves |

## 11. Why the hint helps a later Boolean, and its limit

For a retained `Plane` group, the true patch lies exactly in `T`. A held
vertex's distance from `T` is the part of its displacement that tangency
checks care about for a facet pair across that face; the in-plane part moves
the contact along the face and cannot make two surfaces touch that were not
touching. Increment 5 replaces `sourceBound` (payload §5.3) for such a facet
by its computed out-of-plane distance plus the rounding of the computation.
A curved retained group has no such split, so a cylinder or sphere keeps
`δ_f`.

The gain depends on how often the refusals come from rim vertices of planar
groups. That is unmeasured. Increment 5 starts with a probe that counts
hidden-tangency refusals on a chain of cuts, and it is dropped if the count
is zero.

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
| Fillet and chamfer of an edge between two retained planes | out of scope here; needs its own route in `docs/modify-reach-design.md` (SX9) | include as increment 6 |
| Oblique `Plane`/`Cylinder` edges as `Ellipse3` | not in increment 3 | add to increment 3; needs an ellipse sweep-sense proof like E7 |
