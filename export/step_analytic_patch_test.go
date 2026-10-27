package export

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// patchRoundedBoxChamfer is the 20x20x10 box at (off, off) with its four
// vertical edges filleted r = 3, then its top loop chamfered by 1: four cone
// patches, each two arcs about the corner's vertical axis and two generators,
// on cones of Radius 0 whose Origin is the apex.
func patchRoundedBoxChamfer(t *testing.T, off float64) *decad.Body {
	t.Helper()
	box := bandBox(t, decad.New(), off, off, 20+off, 20+off, 10)
	rounded, err := decad.Fillet(t.Context(), box, decad.Edges(decad.ParallelTo(bandUp)).Exactly(4), units.Millimeters(3))
	require.NoError(t, err)
	top := bandFacing(t, rounded, bandUp, 10)
	out, err := decad.Chamfer(t.Context(), rounded, bandLoopQuery(top.Loops()[0]), units.Millimeters(1))
	require.NoError(t, err)
	return out
}

// patchPocketMouthChamfer is the 40x40x10 plate with a 20x10 pocket 5 deep,
// its mouth chamfered by 1: four cone patches at the reflex corners, each one
// arc and two generators meeting at the apex vertex.
func patchPocketMouthChamfer(t *testing.T) *decad.Body {
	t.Helper()
	doc := decad.New()
	plate := bandBox(t, doc, 0, 0, 40, 40, 10)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 5)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	r := s.CreateRectangle(10, 15, 30, 25)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	out, err := decad.Chamfer(t.Context(), cut, bandLoopQuery(bandFacing(t, cut, bandUp, 10).Loops()[1]), units.Millimeters(1))
	require.NoError(t, err)
	return out
}

// patchFilletedCube is the 20 mm cube with all twelve edges filleted r = 2:
// eight sphere patches, each three great-circle quarter arcs.
func patchFilletedCube(t *testing.T) *decad.Body {
	t.Helper()
	out, err := decad.Fillet(t.Context(), bandBox(t, decad.New(), 0, 0, 20, 20, 20), decad.Edges(), units.Millimeters(2))
	require.NoError(t, err)
	return out
}

func isConePatch(f *decad.Face) bool {
	_, ok := f.Surface().(decad.Cone)
	return ok && len(f.Loops()) == 1
}

func isSpherePatch(f *decad.Face) bool {
	_, ok := f.Surface().(decad.Sphere)
	return ok && len(f.Loops()) == 1
}

// requirePatchFacesRunCounterClockwise reads every ADVANCED_FACE whose body
// face satisfies pick back from the file and requires its loop's chord
// polygon to have a vector area along the mean of the body face's outward
// normals at the polygon's vertices off its cone's apex, where no normal
// exists. The oracle reads only the file and decad's own face normal.
func requirePatchFacesRunCounterClockwise(t *testing.T, f step.File, body *decad.Body, pick func(*decad.Face) bool) int {
	t.Helper()
	checked := 0
	faces := bandEntities(f)["ADVANCED_FACE"]
	require.Len(t, faces, len(body.Faces()))
	for i, face := range faces {
		bodyFace := body.Faces()[i]
		if !pick(bodyFace) {
			continue
		}
		apex, isCone := bodyFace.Surface().(decad.Cone)
		pts := faceLoopPoints(t, f, face)
		area, mean := r3.Vec{}, r3.Vec{}
		for j, p := range pts {
			area = area.Add(p.Cross(pts[(j+1)%len(pts)]).Scale(0.5))
			if isCone && p == apex.Origin {
				continue
			}
			n, err := bodyFace.NormalAt(p)
			require.NoError(t, err)
			mean = mean.Add(n.Value)
		}
		require.Positive(t, area.Dot(mean), "face %d loop runs clockwise about its outward normal", i)
		checked++
	}
	return checked
}

// patchSenses reads the same_sense of every ADVANCED_FACE on a surface named
// surface.
func patchSenses(f step.File, surface string) []step.Enumeration {
	byName := bandEntities(f)
	ids := map[uint64]bool{}
	for _, e := range byName[surface] {
		ids[e.ID] = true
	}
	var out []step.Enumeration
	for _, face := range byName["ADVANCED_FACE"] {
		if ids[uint64(face.Parameters[2].(step.Reference))] {
			out = append(out, face.Parameters[3].(step.Enumeration))
		}
	}
	return out
}

// patchSurface is a CONICAL_SURFACE or SPHERICAL_SURFACE read back from a
// file: its placement's location and axis, its radius and, for a cone, its
// semi-angle.
type patchSurface struct {
	name              string
	location, axis    r3.Vec
	radius, semiAngle float64
}

// readPatchSurface reads the surface an ADVANCED_FACE names when it is a
// CONICAL_SURFACE or SPHERICAL_SURFACE, from the file's entities alone.
func readPatchSurface(t *testing.T, f step.File, face step.Entity) (patchSurface, bool) {
	t.Helper()
	byID := map[uint64]step.Entity{}
	for _, e := range f.Entities {
		byID[e.ID] = e
	}
	ref := func(v step.Value) step.Entity { return byID[uint64(v.(step.Reference))] }
	triple := func(e step.Entity) r3.Vec {
		list := e.Parameters[1].(step.List)
		return r3.NewVec(float64(list[0].(step.Real)), float64(list[1].(step.Real)), float64(list[2].(step.Real)))
	}
	surface := ref(face.Parameters[2])
	if surface.Name != "CONICAL_SURFACE" && surface.Name != "SPHERICAL_SURFACE" {
		return patchSurface{}, false
	}
	placement := ref(surface.Parameters[1])
	out := patchSurface{
		name:     surface.Name,
		location: triple(ref(placement.Parameters[1])),
		axis:     triple(ref(placement.Parameters[2])),
		radius:   float64(surface.Parameters[2].(step.Real)),
	}
	if surface.Name == "CONICAL_SURFACE" {
		out.semiAngle = float64(surface.Parameters[3].(step.Real))
	}
	return out, true
}

// requireFileVerticesOnPatchSurfaces reads every cone and sphere ADVANCED_FACE
// back from the file and requires each vertex its loop walks to lie on the
// surface the file states: at its radius from a sphere's location, or, for a
// cone, at R + h·tan(semi-angle) from the axis, h its height along the axis
// from the location (ISO 10303-42's conical_surface, h at or above −R/tan).
// The oracle reads only the file, so a writer that states a wrong centre,
// radius or half-angle fails it; 1e-9 mm is far above the rounding of these
// coordinates and far below any wrong placement. It returns how many faces it
// checked.
func requireFileVerticesOnPatchSurfaces(t *testing.T, f step.File) int {
	t.Helper()
	checked := 0
	for _, face := range bandEntities(f)["ADVANCED_FACE"] {
		surface, ok := readPatchSurface(t, f, face)
		if !ok {
			continue
		}
		for _, p := range faceLoopPoints(t, f, face) {
			d := p.Sub(surface.location)
			if surface.name == "SPHERICAL_SURFACE" {
				require.InDelta(t, surface.radius, d.Len(), 1e-9, "vertex %v off the sphere", p)
				continue
			}
			h := d.Dot(surface.axis)
			require.GreaterOrEqual(t, surface.radius+h*math.Tan(surface.semiAngle), -1e-9, "vertex %v on the far nappe", p)
			radial := d.Sub(surface.axis.Scale(h)).Len()
			require.InDelta(t, surface.radius+h*math.Tan(surface.semiAngle), radial, 1e-9, "vertex %v off the cone", p)
		}
		checked++
	}
	return checked
}

// conePatchArmFile writes a loop chamfer through the analytic arm's writer.
// Every corner cone of a loop chamfer is a cap-blend band patch whose tag
// departs from the ruled patch the build assembles by a stated bound
// (agent-docs/modify-reach-design.md §8.3), so tagsUndeparted refuses the body
// and NewSTEPFile writes it faceted. The face admission still takes it, and
// these bodies are the only ones that build cone patches, so the cone patch
// writer is read on them through armFile.
func conePatchArmFile(t *testing.T, body *decad.Body) step.File {
	t.Helper()
	require.False(t, tagsUndeparted(body), "a loop chamfer's corner cones state a departure from their tags")
	ok, err := supportsAnalyticSTEP(t.Context(), body)
	require.NoError(t, err)
	require.False(t, ok)
	ok, err = supportsAnalyticFaces(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok, "the face admission takes the chamfered body")
	return armFile(t, body)
}

// TestAnalyticConePatchesConvexCorners writes the rounded box's top chamfer
// through the cone patch writer (conePatchArmFile): four CONICAL_SURFACE
// patches at radius 0 (the apex), every vertex of each on the cone the file
// states, each of sense T, since the chamfer's outward normal leans away from
// the corner's axis as the cone's own does. Besides the four cones there are
// six planes, four corner cylinders and the four chamfer planes. Every edge
// is used once in each direction and every cone loop runs counter-clockwise
// about its outward normal, read back from the file. Shown to fail with
// supportsAnalyticConePatch refusing every loop (the face admission refused
// the body), with addConePatchFace's sense inverted (the senses read F), with
// patchWalks' decided walk inverted (every edge a patch shares was used twice
// in one direction), and with conicalSurface writing the half-angle 1% wide
// or its placement 0.5 mm along the axis (the vertices left the stated cone).
func TestAnalyticConePatchesConvexCorners(t *testing.T) {
	t.Parallel()
	body := patchRoundedBoxChamfer(t, 0)
	f := conePatchArmFile(t, body)
	byName := bandEntities(f)
	require.Len(t, byName["ADVANCED_FACE"], 18)
	require.Len(t, byName["PLANE"], 10)
	require.Len(t, byName["CYLINDRICAL_SURFACE"], 4)
	require.Equal(t, []float64{0, 0, 0, 0}, bandReals(byName["CONICAL_SURFACE"], 2))
	require.Equal(t, 4, requireFileVerticesOnPatchSurfaces(t, f))
	require.Equal(t, []step.Enumeration{"T", "T", "T", "T"}, patchSenses(f, "CONICAL_SURFACE"))
	requireEveryEdgeCurveInBothSenses(t, f)
	require.Equal(t, 4, requirePatchFacesRunCounterClockwise(t, f, body, isConePatch))
}

// TestAnalyticConePatchesReflexCorners writes the pocket mouth's chamfer
// through the cone patch writer (conePatchArmFile): four three-edge cone
// patches, one arc and two generators meeting at the apex vertex, every
// vertex on the cone the file states. Each is of sense F: the body's outward
// normal points into the pocket, toward the cone's axis, against the cone's
// own. Shown to fail with supportsAnalyticConePatch's minimum of three edges
// raised to four (the face admission refused the body) and with
// addConePatchFace's sense inverted. addConePatchFace skips a vertex at the
// apex when it reads the sense, since no normal exists there. That skip is
// not shown red: no fixture's loop starts at its apex vertex, so the first
// vertex already lies off the axis.
func TestAnalyticConePatchesReflexCorners(t *testing.T) {
	t.Parallel()
	body := patchPocketMouthChamfer(t)
	f := conePatchArmFile(t, body)
	byName := bandEntities(f)
	require.Len(t, byName["ADVANCED_FACE"], len(body.Faces()))
	require.Equal(t, []float64{0, 0, 0, 0}, bandReals(byName["CONICAL_SURFACE"], 2))
	require.Equal(t, 4, requireFileVerticesOnPatchSurfaces(t, f))
	require.Equal(t, []step.Enumeration{"F", "F", "F", "F"}, patchSenses(f, "CONICAL_SURFACE"))
	requireEveryEdgeCurveInBothSenses(t, f)
	require.Equal(t, 4, requirePatchFacesRunCounterClockwise(t, f, body, isConePatch))
}

// TestAnalyticSpherePatchesFilletedCube writes the cube with every edge
// filleted r = 2: six planes, twelve cylinder patches and eight
// SPHERICAL_SURFACE corners of radius 2 centred 2 in from each corner, read
// from the file's placements with every vertex on the stated sphere, each of
// sense T, bounded by three great-circle arcs. The only circles are the
// patches' 24 arcs, each of radius 2. Shown to fail with the Sphere arm of
// supportsAnalyticSTEP deleted (the body fell to the faceted writer), with
// addSphereFace's sense inverted, with patchWalks' decided walk inverted, and
// with addSphereFace placing the sphere 0.5 mm off its Center.
func TestAnalyticSpherePatchesFilletedCube(t *testing.T) {
	t.Parallel()
	body := patchFilletedCube(t)
	f := bandFile(t, body)
	byName := bandEntities(f)
	require.Len(t, byName["ADVANCED_FACE"], 26)
	require.Len(t, byName["PLANE"], 6)
	require.Len(t, byName["CYLINDRICAL_SURFACE"], 12)
	require.Equal(t, slices.Repeat([]float64{2}, 8), bandReals(byName["SPHERICAL_SURFACE"], 2))
	require.Equal(t, slices.Repeat([]float64{2}, 24), bandReals(byName["CIRCLE"], 2))
	var centres []r3.Vec
	for _, face := range byName["ADVANCED_FACE"] {
		if surface, ok := readPatchSurface(t, f, face); ok {
			centres = append(centres, surface.location)
		}
	}
	require.Equal(t, 8, requireFileVerticesOnPatchSurfaces(t, f))
	require.ElementsMatch(t, []r3.Vec{
		r3.NewVec(2, 2, 2), r3.NewVec(18, 2, 2), r3.NewVec(2, 18, 2), r3.NewVec(18, 18, 2),
		r3.NewVec(2, 2, 18), r3.NewVec(18, 2, 18), r3.NewVec(2, 18, 18), r3.NewVec(18, 18, 18),
	}, centres)
	require.Equal(t, slices.Repeat([]step.Enumeration{"T"}, 8), patchSenses(f, "SPHERICAL_SURFACE"))
	requireEveryEdgeCurveInBothSenses(t, f)
	require.Equal(t, 8, requirePatchFacesRunCounterClockwise(t, f, body, isSpherePatch))
}

// TestAnalyticConePatchWithBoundedVertexKeepsFaceted builds the rounded box's
// chamfer at (0.2, 0.2), whose patch vertices carry rounding bounds while
// every arc and generator still passes its exact test, and requires the face
// admission to refuse it, which the tag gate (tagsUndeparted) does first on
// its own. Shown to fail with supportsAnalyticConePatch's loopVerticesExact
// leg deleted (the face admission took the body).
func TestAnalyticConePatchWithBoundedVertexKeepsFaceted(t *testing.T) {
	t.Parallel()
	body := patchRoundedBoxChamfer(t, 0.2)
	bounded := 0
	for _, face := range body.Faces() {
		if isConePatch(face) && !loopVerticesExact(face.Loops()[0]) {
			bounded++
		}
	}
	require.Equal(t, 4, bounded, "every corner patch has a vertex with a bound")
	ok, err := supportsAnalyticFaces(t.Context(), body)
	require.NoError(t, err)
	require.False(t, ok, "a patch vertex with a bound sends the body to the faceted writer")
	f, err := NewSTEPFile(t.Context(), body, units.Millimeters(0.2), bandHeader())
	require.NoError(t, err)
	byName := bandEntities(f)
	require.Empty(t, byName["CONICAL_SURFACE"])
	require.Equal(t, len(byName["ADVANCED_FACE"]), len(byName["PLANE"]), "one planar face per mesh triangle")
}

// TestAnalyticPatchAdmissionRefusesForeignTags runs the patch admissions on
// the real patch loops against tags that do not hold them, each failing one
// leg: a cone whose Radius is not zero, one whose apex moved along its axis
// (the generators no longer pass through it), one whose axis tilts (the arcs
// are not about it), a sphere moved off the arcs' centre, and one of another
// radius. Each was shown to fail with its leg deleted, except the tilt, which
// the arc-centre leg also refuses.
func TestAnalyticPatchAdmissionRefusesForeignTags(t *testing.T) {
	t.Parallel()
	var cone *decad.Face
	for _, face := range patchRoundedBoxChamfer(t, 0).Faces() {
		if isConePatch(face) {
			cone = face
			break
		}
	}
	require.NotNil(t, cone)
	tag := cone.Surface().(decad.Cone)
	loop := cone.Loops()[0]
	require.True(t, supportsAnalyticConePatch(loop, tag))
	wide := tag
	wide.Radius = units.Millimeters(1)
	require.False(t, supportsAnalyticConePatch(loop, wide), "a cone patch needs its apex at Origin")
	moved := tag
	moved.Origin = tag.Origin.Add(tag.Axis)
	require.False(t, supportsAnalyticConePatch(loop, moved), "a generator must pass through the apex")
	tilted := tag
	tilted.Axis = r3.NewVec(1, 0, 0)
	require.False(t, supportsAnalyticConePatch(loop, tilted), "an arc must run about the cone's axis")

	var sphere *decad.Face
	for _, face := range patchFilletedCube(t).Faces() {
		if isSpherePatch(face) {
			sphere = face
			break
		}
	}
	require.NotNil(t, sphere)
	ball := sphere.Surface().(decad.Sphere)
	require.True(t, supportsAnalyticSpherePatch(sphere.Loops()[0], ball))
	shifted := ball
	shifted.Center = ball.Center.Add(r3.NewVec(0, 0, 1))
	require.False(t, supportsAnalyticSpherePatch(sphere.Loops()[0], shifted), "an arc must be a great circle")
	larger := ball
	larger.Radius = units.Millimeters(3)
	require.False(t, supportsAnalyticSpherePatch(sphere.Loops()[0], larger), "an arc must have the sphere's radius")
}

// TestPatchWalksDecideAndRefuse runs decideWalks on plain edge uses. Face 1
// is fixed and walks edges 1 and 2 forward. Patch 2 uses edge 1 forward, so
// it is reversed; patch 3 meets only patch 2, along edge 3, and is decided
// from it in a later round. A patch whose shared edges ask for both walks, and
// one no decided face reaches, are errPatchUnoriented. Shown to fail with the
// both-walks check deleted (the conflicting patch was decided) and with the
// no-progress check deleted (the loop never ended, caught by the test's
// timeout).
func TestPatchWalksDecideAndRefuse(t *testing.T) {
	t.Parallel()
	type face = walkFace[int, int]
	fixed := []face{{face: 1, steps: []walkStep[int]{{edge: 1, forward: true}, {edge: 2, forward: true}}}}
	t.Run("decided", func(t *testing.T) {
		t.Parallel()
		walks, err := decideWalks(fixed, []face{
			{face: 3, steps: []walkStep[int]{{edge: 3, forward: false}, {edge: 4, forward: true}}},
			{face: 2, steps: []walkStep[int]{{edge: 1, forward: true}, {edge: 3, forward: true}}},
		})
		require.NoError(t, err)
		// Patch 2 writes edge 1 backward and so edge 3 backward; patch 3,
		// whose source walks edge 3 backward too, is reversed to write it
		// forward.
		require.Equal(t, map[int]bool{2: true, 3: true}, walks)
	})
	t.Run("both walks", func(t *testing.T) {
		t.Parallel()
		_, err := decideWalks(fixed, []face{
			{face: 2, steps: []walkStep[int]{{edge: 1, forward: true}, {edge: 2, forward: false}}},
		})
		require.ErrorIs(t, err, errPatchUnoriented)
		require.ErrorContains(t, err, "ask for both walks")
	})
	t.Run("unreached", func(t *testing.T) {
		t.Parallel()
		_, err := decideWalks(fixed, []face{
			{face: 2, steps: []walkStep[int]{{edge: 7, forward: true}}},
		})
		require.ErrorIs(t, err, errPatchUnoriented)
		require.ErrorContains(t, err, "no decided face shares an edge")
	})
}

// TestArcInItsPlaneRefusesASwappedAxis runs arcInItsPlane on real patch arcs,
// a cone patch's and a sphere patch's, and on the same edges with each arc's
// Axis swapped for a perpendicular one, which no longer holds the edge's
// vertices. The admissions cannot be handed a foreign arc, since an edge's
// curve is the body's own, so this is the leg's red case. Shown to fail with
// arcInItsPlane always reporting true.
func TestArcInItsPlaneRefusesASwappedAxis(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		body *decad.Body
		pick func(*decad.Face) bool
	}{
		{"cone patch", patchRoundedBoxChamfer(t, 0), isConePatch},
		{"sphere patch", patchFilletedCube(t), isSpherePatch},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			checked := 0
			for _, face := range tc.body.Faces() {
				if !tc.pick(face) {
					continue
				}
				for _, ce := range face.Loops()[0].CoEdges() {
					arc, ok := ce.Edge().Curve().(decad.Arc3)
					if !ok {
						continue
					}
					require.True(t, arcInItsPlane(ce.Edge(), arc))
					// The arc's start radial is perpendicular to its Axis; the
					// swapped axis is that radial, which the start leaves.
					swapped := arc
					radial, ok := ce.Edge().Start().Position().Value.Sub(arc.Center).Normalize()
					require.True(t, ok)
					swapped.Axis = radial
					require.False(t, arcInItsPlane(ce.Edge(), swapped), "an axis that does not hold the arc's vertices")
					checked++
				}
			}
			require.Positive(t, checked)
		})
	}
}
