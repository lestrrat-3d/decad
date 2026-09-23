package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md §15's internal legs of T10's revolve
// row: the two claims tessellate_revolve_sheet_test.go cannot assert from
// outside the package because they read Mesh's own unexported proof fields
// directly — that a revolve sheet publishes no occupied-volume proof at
// all, and that its area slack is strictly below the same record's solid
// mesh once a circular meridian wall gives the dropped cap term something
// to drop. revolveAxisU is tessellate_revolve_volume_internal_test.go's own
// shared axis.

// internalOffAxisArcBody builds a half-disc entirely clear of the revolve
// axis (diameter at v=10, arc bulging to v=15): one straight wall and one
// CIRCULAR wall, so a partial sweep's cap segment area — the one term a
// sheet's area slack alone drops (docs/tessellation-design.md §10.2) — is
// provably nonzero. surfaceResult selects the sheet or the solid build from
// the identical record.
func internalOffAxisArcBody(t *testing.T, surfaceResult bool) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 10)
	s.Fix(o)
	end := s.CreatePoint(10, 10)
	c := s.CreatePoint(5, 10)
	s.CreateLine(o, end)
	s.CreateArc(c, end, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	quarter := AngleExtent{A: units.Degrees(90), Dir: Along}
	var opts []RevolveOption
	if surfaceResult {
		opts = append(opts, WithSurfaceResult())
	}
	body, err := New().Revolve(s, s.Profiles()[0], revolveAxisU, quarter, opts...)
	require.NoError(t, err)
	return body
}

// internalOffAxisArcFullTurnBody builds internalOffAxisArcBody's own
// profile revolved a FULL turn instead of a quarter: a full turn mints no
// cap in either kind, so its sheet carries no free edge at all — CLOSED,
// unlike the quarter-turn sheet above.
func internalOffAxisArcFullTurnBody(t *testing.T, surfaceResult bool) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 10)
	s.Fix(o)
	end := s.CreatePoint(10, 10)
	c := s.CreatePoint(5, 10)
	s.CreateLine(o, end)
	s.CreateArc(c, end, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var opts []RevolveOption
	if surfaceResult {
		opts = append(opts, WithSurfaceResult())
	}
	body, err := New().Revolve(s, s.Profiles()[0], revolveAxisU, FullRevolution{}, opts...)
	require.NoError(t, err)
	return body
}

// TestRevolveSheetPublishesNoOccupiedVolumeProof is CLAUDE.md's own
// permanent absence (docs/surface-design.md §10): a revolve sheet mesh
// leaves symDiffOK false and volSymDiff at its zero value, the same shape
// the prism sheet path documents.
func TestRevolveSheetPublishesNoOccupiedVolumeProof(t *testing.T) {
	t.Parallel()
	sheet := internalOffAxisArcBody(t, true)
	mesh, err := sheet.Tessellate(t.Context(), units.Millimeters(0.5))
	require.NoError(t, err)
	require.False(t, mesh.symDiffOK)
	require.Zero(t, mesh.volSymDiff)
}

// TestRevolveSheetAreaSlackBelowSolidOnCircularMeridian proves the sheet's
// area slack is strictly below the solid's on a fixture whose meridian is
// CIRCULAR: an all-straight fixture has a zero cap term and would prove
// nothing (the same reasoning that picks this fixture over annularSketch's
// rectangle).
func TestRevolveSheetAreaSlackBelowSolidOnCircularMeridian(t *testing.T) {
	t.Parallel()
	tol := units.Millimeters(0.5)

	solid := internalOffAxisArcBody(t, false)
	solidMesh, err := solid.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	require.Positive(t, solidMesh.areaSlack, `the fixture's circular wall must give the dropped cap term something to drop`)

	sheet := internalOffAxisArcBody(t, true)
	sheetMesh, err := sheet.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	// The gap must be the dropped CAP term itself (a few mm² on this
	// fixture), not merely the coordinate-rounding confound every extra
	// stored triangle carries regardless of this term (of order 1e-13 mm²
	// here) — so the threshold sits many orders of magnitude above the
	// latter and well below the former, and stays robust to the
	// architecture-specific FMA rounding that confound is made of.
	gap := solidMesh.areaSlack - sheetMesh.areaSlack
	require.Greater(t, gap, 1e-6, `the dropped cap term must dominate any coordinate-rounding confound`)
}

// TestRevolveSheetOrientationSignIsAnchorDependent is the falsifier for the
// signed-volume orientation guard (tessellate_revolve.go's `!sheet ||
// rp.full` arm around meshOrientationSign). No Table W fixture's own
// default anchor (rp.xform.Apply(p.basis.a3)) happens to flip sign when
// that guard is dropped, so a test built from that call alone would never
// catch its removal — the guard cannot be justified by any fixture's
// default anchor. What justifies it is that the sum is ANCHOR-DEPENDENT at
// all on an open mesh, which is what this test asserts directly: the same
// open (quarter-turn) sheet mesh reads a DIFFERENT sign at two anchors, so
// the sum is not a property of the geometry alone and the guard may never
// run it there. A CLOSED (full-turn) sheet mesh is the contrasting case the
// guard does not need to cover: with no free edge, the sum is the true
// enclosed volume regardless of anchor, so it reads the SAME sign at both.
func TestRevolveSheetOrientationSignIsAnchorDependent(t *testing.T) {
	t.Parallel()
	anchorA := r3.NewVec(0, 0, 0)
	anchorB := r3.NewVec(0, 1000, 0)

	open := internalOffAxisArcBody(t, true)
	openMesh, err := open.Tessellate(t.Context(), units.Millimeters(0.5))
	require.NoError(t, err)
	openSignA := meshOrientationSign(openMesh.vertices, openMesh.triangles, anchorA)
	openSignB := meshOrientationSign(openMesh.vertices, openMesh.triangles, anchorB)
	require.NotEqual(t, openSignA, openSignB,
		`an open sheet mesh's signed-volume sum must be anchor-dependent, or the orientation guard protects against nothing`)

	closed := internalOffAxisArcFullTurnBody(t, true)
	closedMesh, err := closed.Tessellate(t.Context(), units.Millimeters(0.5))
	require.NoError(t, err)
	closedSignA := meshOrientationSign(closedMesh.vertices, closedMesh.triangles, anchorA)
	closedSignB := meshOrientationSign(closedMesh.vertices, closedMesh.triangles, anchorB)
	require.Equal(t, closedSignA, closedSignB,
		`a closed sheet mesh's signed-volume sum must agree at every anchor, since it is the true enclosed volume`)
}
