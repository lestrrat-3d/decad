# Mesh wall and undercut survey design

`Verify` with `WithMinWallThickness` surveys the wall of a faceted body, the result of a mesh Boolean, by casting rays
through its held mesh, and with `WithPullDirection` surveys its undercuts the same way (§6). The survey finds thin walls and cannot rule them out, so it proves a body too thin and never
proves one thick enough. The sampling is plain-data code in `internal/meshthick/`, and `internal/verifyrun/faceted_wall.go`
turns it into the wall reading the report publishes.

## 1. What it measures

| Decision | Rule |
|---|---|
| W1 | From the centre of each held triangle of nonzero area, a ray runs along the inward normal, the negated normal of a mesh wound counter-clockwise seen from outside. The ray's length to the nearest other triangle, at least `1e-9` of the mesh diameter away, is one thickness sample. The triangle the ray starts on is skipped. |
| W2 | The sampled thickness `s` is the smallest sample. The thinnest wall of the true body is no larger than `s`, because it is the minimum over every surface point and the samples are a subset of them. |
| W3 | The upper bound is `s + 2·MeshBound + 1e-9·max(diameter, s)`: `s` widened by the held mesh's proven displacement bound at each end of the wall, and by an allowance for the float rounding of the ray casts. |
| W4 | The reading published as the wall is the interval from zero to that upper bound: value and bound both half of it, `Approximate`. The lower end is zero because the rays prove no wall is thick enough. |
| W5 | The interval is below the tool exactly when the upper bound is, which is `DiagWallTooThin` and `Violating`, proven. An interval that reaches the tool straddles it, which is `DiagUndecidedWall` and `Suspect`. A body whose rays all miss, or whose mesh held no triangle, reads `DiagUnsupportedSurveyPayload` as before. |
| W6 | The reading's bound is half its upper end, so the tolerance gate always reports `DiagMeasurementBeyondTolerance` beside it. That finding is `Suspect` and never hides a `Violating` one. |

## 2. What it does not claim

The survey never reports a body `Sound` on the wall: a passing wall check needs a lower bound, and rays give only an
upper one. A thick body reads `Suspect` with an undecided wall, as it did when the survey was unsupported, and a thin
one now reads `Violating`.

A knife edge counts. A ray from a face near an acute edge, such as the rim where a bore meets a face at a shallow
angle, leaves the material a short way off, so such an edge reads as a wall as thin as the ray. The wall is thin there.

The mesh must be closed and wound counter-clockwise seen from outside, which a mesh Boolean result is. A mesh that is
not gives a reading without meaning.

## 3. Cost

The mesh is sorted into a bounding-volume tree of leaves of at most four triangles, and each ray prunes against the
smallest sample found so far. The work polls the verification budget once per ray, so a canceled context stops it.

## 4. Verification

The sampler tests measure a box, a cube, a cube with a cavity pushed to 1 mm from one face and a wedge. The API tests
union two 1 mm plates crossing at 45 degrees and require a proven `DiagWallTooThin` against a 2 mm tool, an undecided
wall against a 0.5 mm tool, and no thin finding for 8 mm plates against a 2 mm tool. A reading scaled below the upper
bound turns the second test red, and a survey that casts no rays turns the first red.

## 5. Not covered

- Proving a wall thick enough, and so a `Sound` wall verdict for a faceted body that is not an axis-square solid with no vertex bound, which `agent-docs/orthogonal-wall-design.md` reads exactly.
- A sheet body, which has no inward side.
- Proving a pull free of undercuts, and a concave-radius survey of a faceted body, which stays unsupported.

## 6. Undercuts against a pull

The same tree answers the pull question.

| Decision | Rule |
|---|---|
| U1 | A triangle whose outward normal has a component along the pull must be released along it, and one with a component against the pull along the reverse. A triangle within `1e-6` of square to the pull (a sine below that) is seen edge on and casts no ray. |
| U2 | From the centre of each such triangle a ray runs in its release direction. The triangle is undercut when the nearest triangle the ray crosses at least `clearance` out, and whose outward normal points against the ray by more than `1e-6`, is one the ray enters material through. `clearance` is twice the mesh's displacement bound plus `1e-9` of the diameter, so the hit is not the mesh's approximation of the triangle's own neighbours. |
| U3 | Blocked triangles name their faces, once each, in face order. Any listed face makes `DiagUndercut`, `Violating`, with `ok` and `undecided` both set. A body with no blocked triangle reads `DiagUndecidedUndercut`, `Suspect`, with nothing listed, because one ray per triangle shows no face free. |
| U4 | A ray that runs along a wall, as one along a tessellated cylinder's axis does, is within float rounding of every facet's plane. A hit counts only when its triangle is more than `1e-6` from parallel to the ray, in both the wall survey (W1) and this one, so a graze never reads as a crossing. |

A face blocked by the body is a proven undercut whatever parting surface the mould takes: a face looking along the pull
can only be formed by the half that moves along it, and material above the face stops that half.
