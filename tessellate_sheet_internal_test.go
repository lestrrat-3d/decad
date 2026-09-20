package decad

import (
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// internalSheetBoxBody surface-extrudes an axis-aligned rectangle, the sheet
// counterpart of interference_internal_test.go's internalBoxBody.
func internalSheetBoxBody(t *testing.T, doc *Document, x0, y0, x1, y1, h float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along}, WithSurfaceResult())
	require.NoError(t, err)
	return body
}

// internalHoledSheetBody is internalHoledPlateBody's (tessellate_proof_internal_test.go)
// own sheet counterpart: the same 100×60 plate with a 10 mm-radius hole at
// (70, 30), surface-extruded 8 mm — the fixture whose circular hole gives a
// non-trivial bound and area slack to compare against the solid's.
func internalHoledSheetBody(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	body, err := doc.Extrude(s, prof, Distance{D: units.Millimeters(8), Dir: Along}, WithSurfaceResult())
	require.NoError(t, err)
	return body
}

// TestSheetMeshPublishesNoOccupiedVolumeProof is docs/surface-design.md §10's
// own claim: a sheet mesh leaves symDiffOK false and volSymDiff zero, while
// the solid built from the same holed record — whose circular hole gives it
// a genuinely positive occupied-volume bound — publishes both.
func TestSheetMeshPublishesNoOccupiedVolumeProof(t *testing.T) {
	t.Parallel()
	solid := internalHoledPlateBody(t, New())
	solidMesh, err := tessellateContext(t.Context(), solid, units.Millimeters(0.5))
	require.NoError(t, err)
	require.True(t, solidMesh.symDiffOK)
	require.Positive(t, solidMesh.volSymDiff)

	sheet := internalHoledSheetBody(t, New())
	sheetMesh, err := tessellateContext(t.Context(), sheet, units.Millimeters(0.5))
	require.NoError(t, err)
	require.False(t, sheetMesh.symDiffOK)
	require.Zero(t, sheetMesh.volSymDiff)

	_, err = operandSymDiff(sheetMesh)
	require.ErrorIs(t, err, ErrUnsupported)
}

// TestSheetMeshFaceBoundCoversWallsOnly is docs/tessellation-design.md §2's
// per-face proof record, read for a sheet: one faceBound entry per wall and
// none for a cap role, while the solid built from the same record carries the
// wall entries plus the two caps'.
func TestSheetMeshFaceBoundCoversWallsOnly(t *testing.T) {
	t.Parallel()
	solid := internalHoledPlateBody(t, New())
	solidMesh, err := tessellateContext(t.Context(), solid, units.Millimeters(0.5))
	require.NoError(t, err)

	sheet := internalHoledSheetBody(t, New())
	sheetMesh, err := tessellateContext(t.Context(), sheet, units.Millimeters(0.5))
	require.NoError(t, err)

	require.Equal(t, len(sheet.Faces()), len(sheetMesh.faceBound), `one faceBound entry per wall, and a sheet carries no cap face`)
	for _, f := range sheet.Faces() {
		_, ok := sheetMesh.faceBound[f]
		require.True(t, ok, `every wall face has a published sourceBound`)
	}
	require.Equal(t, len(solid.Faces()), len(solidMesh.faceBound), `the solid's own faceBound additionally covers its two caps`)
}

// TestSheetAreaSlackIsBelowTheSolidsOnTheHoledFixture is the decision that a
// sheet's areaSlack drops the cap terms it no longer carries: on the holed
// fixture, whose circular hole gives every cap a genuinely positive
// chord-versus-arc deficit, the sheet's own areaSlack must be strictly below
// the solid's built from the same record.
func TestSheetAreaSlackIsBelowTheSolidsOnTheHoledFixture(t *testing.T) {
	t.Parallel()
	solid := internalHoledPlateBody(t, New())
	solidMesh, err := tessellateContext(t.Context(), solid, units.Millimeters(0.5))
	require.NoError(t, err)

	sheet := internalHoledSheetBody(t, New())
	sheetMesh, err := tessellateContext(t.Context(), sheet, units.Millimeters(0.5))
	require.NoError(t, err)

	require.Positive(t, solidMesh.areaSlack)
	// The gap is the two omitted caps' own circular-segment deficit, not mere
	// float noise from the smaller triangle count: require a gap wide enough
	// that only dropping the cap terms explains it.
	require.Greater(t, solidMesh.areaSlack-sheetMesh.areaSlack, 1.0)
}

// TestRequireSheetMeshRejectsBrokenFreeEdgeAttribution is the free-edge
// attribution's own falsifier. The public API can never construct a mesh
// whose free boundary is attributed to the wrong face — chordLoop always
// attributes a wall's own triangles to its own face — so this test hand-
// corrupts a real mesh's SourceFaces to prove requireSheetMesh actually
// refuses the mismatch it exists to catch, rather than passing vacuously.
func TestRequireSheetMeshRejectsBrokenFreeEdgeAttribution(t *testing.T) {
	t.Parallel()
	sheet := internalSheetBoxBody(t, New(), 0, 0, 10, 10, 5)
	mesh, err := tessellateContext(t.Context(), sheet, units.Millimeters(1))
	require.NoError(t, err)
	require.NoError(t, requireSheetMesh(t.Context(), sheet, mesh), `premise: the real mesh passes the audit as built`)

	faceA := mesh.source[0]
	var faceB *Face
	for _, f := range mesh.source {
		if f != faceA {
			faceB = f
			break
		}
	}
	require.NotNil(t, faceB, `premise: a box sheet's mesh names at least two distinct source faces`)

	broken := *mesh
	broken.source = append([]*Face(nil), mesh.source...)
	broken.source[0] = faceB
	err = requireSheetMesh(t.Context(), sheet, &broken)
	require.Error(t, err, `reassigning one triangle's source face unbalances the chain count on both faces`)
	require.ErrorIs(t, err, ErrDegenerate)
}

// TestRequireSheetMeshCatchesADuplicatedDirectedEdge is the manifold leg's
// own falsifier, for the same reason: the public API never builds a mesh with
// a repeated directed edge, so this hand-builds one triangle pair that shares
// a directed edge in the SAME direction — a fold rather than a fair
// back-to-back pairing — and confirms the audit refuses it.
func TestRequireSheetMeshCatchesADuplicatedDirectedEdge(t *testing.T) {
	t.Parallel()
	sheet := internalSheetBoxBody(t, New(), 0, 0, 10, 10, 5)
	mesh, err := tessellateContext(t.Context(), sheet, units.Millimeters(1))
	require.NoError(t, err)

	broken := *mesh
	broken.triangles = append([][3]int(nil), mesh.triangles...)
	// Force the first two triangles to share the directed edge (t0, t1) in
	// the same direction, which no valid mesh construction ever produces.
	broken.triangles[1] = [3]int{broken.triangles[0][0], broken.triangles[0][1], broken.triangles[0][2]}
	broken.source = append([]*Face(nil), mesh.source...)

	err = requireSheetMesh(t.Context(), sheet, &broken)
	require.Error(t, err)
	require.ErrorIs(t, err, ErrDegenerate)
}
