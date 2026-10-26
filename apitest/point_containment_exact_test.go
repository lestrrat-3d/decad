package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers LocatePoint's exact prism route
// (agent-docs/point-containment-design.md §7). Each fixture is a real prism
// body, and each decided point lies where the verified mesh alone leaves the
// point undecided: on the boundary, 1e-9 mm from it, or inside the mesh's
// symmetric-difference band. Each refusal fixture shows a record the route
// does not read falling back to the mesh.

type locateCase struct {
	name string
	at   r3.Vec
	want decad.PointLocation
}

// requireLocations classifies each case at a 0.05 mm chord tolerance.
func requireLocations(t *testing.T, body *decad.Body, cases []locateCase) {
	t.Helper()
	for _, tc := range cases {
		got, err := body.LocatePoint(t.Context(), tc.at, units.Millimeters(0.05))
		require.NoError(t, err, tc.name)
		require.Equal(t, tc.want, got, tc.name)
	}
}

// TestPointLocationExactCylinder classifies points against a radius-5
// cylinder over z ∈ [0, 10] at a 0.05 mm chord tolerance, where the mesh
// alone reads every point from r = 4 to r = 5.5 undecided.
func TestPointLocationExactCylinder(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	cylinder := circleBodyAtZ(t, doc, 0, 5, 0, 10)
	require.Equal(t, "decad.prismPayload", decadtest.PayloadClass(cylinder))
	requireLocations(t, cylinder, []locateCase{
		{name: "r=4", at: r3.NewVec(4, 0, 5), want: decad.PointInside},
		{name: "r=4.9", at: r3.NewVec(4.9, 0, 5), want: decad.PointInside},
		{name: "r=5.5", at: r3.NewVec(5.5, 0, 5), want: decad.PointOutside},
		{name: "wall", at: r3.NewVec(5, 0, 5), want: decad.PointOnBoundary},
		{name: "wall diagonal", at: r3.NewVec(3, -4, 5), want: decad.PointOnBoundary},
		{name: "1e-9 inside the wall", at: r3.NewVec(5-1e-9, 0, 5), want: decad.PointInside},
		{name: "1e-9 outside the wall", at: r3.NewVec(5+1e-9, 0, 5), want: decad.PointOutside},
		{name: "top cap", at: r3.NewVec(0, 0, 10), want: decad.PointOnBoundary},
		{name: "bottom rim", at: r3.NewVec(0, 5, 0), want: decad.PointOnBoundary},
		{name: "1e-9 under the top cap", at: r3.NewVec(1, 1, 10-1e-9), want: decad.PointInside},
		{name: "1e-9 over the top cap", at: r3.NewVec(1, 1, 10+1e-9), want: decad.PointOutside},
		{name: "cap level beyond the wall", at: r3.NewVec(5.5, 0, 10), want: decad.PointOutside},
	})
}

// TestPointLocationExactPlacedPlate rotates a 30 × 30 × 4 plate 45° about the
// x axis through (15, 15, 2). The route solves the held placement exactly; a
// point on the x = 0 face stays there under the rotation.
func TestPointLocationExactPlacedPlate(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 30, 30, 4)
	turned, err := decad.Copy(t.Context(), plate,
		decad.WithCopyRotation(r3.NewVec(15, 15, 2), r3.NewVec(1, 0, 0), units.Degrees(45)))
	require.NoError(t, err)
	require.Equal(t, "decad.prismPayload", decadtest.PayloadClass(turned))
	rotation, err := r3.RotationAround(r3.NewVec(15, 15, 2), r3.NewVec(1, 0, 0), units.Degrees(45))
	require.NoError(t, err)
	world := func(x, y, z float64) r3.Vec { return rotation.Apply(r3.NewVec(x, y, z)) }
	requireLocations(t, turned, []locateCase{
		{name: "near the turned top face", at: world(1, 1, 3.999), want: decad.PointInside},
		{name: "beyond the turned top face", at: world(1, 1, 4.001), want: decad.PointOutside},
		{name: "x = 0 face", at: r3.NewVec(0, 15, 2), want: decad.PointOnBoundary},
		{name: "1e-9 inside the x = 0 face", at: r3.NewVec(1e-9, 15, 2), want: decad.PointInside},
		{name: "1e-9 outside the x = 0 face", at: r3.NewVec(-1e-9, 15, 2), want: decad.PointOutside},
	})
}

// TestPointLocationExactCutFragments reads a square drawn as four
// overshooting lines, so every recorded segment is a Partial fragment cut at
// a recorded parameter. Its fragments meet at equal ends, so no junction here
// needs a closing path; TestLocatePrismJunctionCorner
// (internal/pointquery) covers the closure through the exact corner.
func TestPointLocationExactCutFragments(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	square := overshootQuadBody(t, doc, [][2]float64{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, 0.13, 5)
	require.Equal(t, "decad.prismPayload", decadtest.PayloadClass(square))
	requireLocations(t, square, []locateCase{
		{name: "material", at: r3.NewVec(5, 5, 2), want: decad.PointInside},
		{name: "beyond the overshoot", at: r3.NewVec(-1, 5, 2), want: decad.PointOutside},
		{name: "corner", at: r3.NewVec(0, 0, 2), want: decad.PointOnBoundary},
		{name: "side", at: r3.NewVec(10, 3, 2), want: decad.PointOnBoundary},
		{name: "1e-9 inside the side", at: r3.NewVec(1e-9, 5, 2), want: decad.PointInside},
		{name: "1e-9 outside the side", at: r3.NewVec(-1e-9, 5, 2), want: decad.PointOutside},
	})
}

// pieBody extrudes the quarter disk of radius 20 about the origin over
// z ∈ [0, 10], its arc ending at (0, endV). The sketch is never solved, so
// the record keeps the drawn End.
func pieBody(t *testing.T, doc *decad.Document, endV float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	start := s.CreatePoint(20, 0)
	end := s.CreatePoint(0, endV)
	s.CreateLine(o, start)
	s.CreateArc(o, start, end)
	s.CreateLine(end, o)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	body, err := decad.Extrude(t.Context(), doc, s, profiles[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	require.Equal(t, "decad.prismPayload", decadtest.PayloadClass(body))
	return body
}

// TestPointLocationExactArc reads a quarter pie whose arc End lies exactly on
// Start's radius. A point on the arc's chord falls back to the mesh, which
// decides it 5.8 mm from the boundary.
func TestPointLocationExactArc(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	pie := pieBody(t, doc, 20)
	requireLocations(t, pie, []locateCase{
		{name: "material", at: r3.NewVec(12, 12, 5), want: decad.PointInside},
		{name: "beyond the arc", at: r3.NewVec(14.2, 14.2, 5), want: decad.PointOutside},
		{name: "on the arc", at: r3.NewVec(12, 16, 5), want: decad.PointOnBoundary},
		{name: "arc end", at: r3.NewVec(0, 20, 5), want: decad.PointOnBoundary},
		{name: "1e-9 inside the arc", at: r3.NewVec(12, 16-1e-9, 5), want: decad.PointInside},
		{name: "1e-9 outside the arc", at: r3.NewVec(12, 16+1e-9, 5), want: decad.PointOutside},
		{name: "circle beyond the arc", at: r3.NewVec(-12, 16, 5), want: decad.PointOutside},
		{name: "chord through the mesh", at: r3.NewVec(10, 10, 5), want: decad.PointInside},
	})
}

// TestPointLocationOffRadiusArcFallsBackToMesh reads the same pie with its
// arc End 500 ulps off Start's radius. Its denoted end carries a square root,
// so the route leaves the loop to the mesh: points near the boundary stay
// undecided and a point deep in the material is decided by the mesh proof.
func TestPointLocationOffRadiusArcFallsBackToMesh(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	pie := pieBody(t, doc, offRadiusEnd(20))
	requireLocations(t, pie, []locateCase{
		{name: "near the arc", at: r3.NewVec(12, 16-1e-9, 5), want: decad.PointUndecided},
		{name: "near a straight side", at: r3.NewVec(10, 1e-9, 5), want: decad.PointUndecided},
		{name: "deep material", at: r3.NewVec(6, 6, 5), want: decad.PointInside},
	})
}
