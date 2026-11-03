package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// lShapedTool is an L-shaped notch tool on the XY plane from z = −1 to 11:
// the union of the rectangles x ∈ [−1, 4] × y ∈ [−1, 2] and
// x ∈ [−1, 2] × y ∈ [−1, 6], drawn as one non-convex polygon.
func lShapedTool(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, profile := polygonSketch(t, [][2]float64{{-1, -1}, {4, -1}, {4, 2}, {2, 2}, {2, 6}, {-1, 6}})
	tool, err := decad.Extrude(t.Context(), doc, s, profile, decad.Distance{D: units.Millimeters(12), Dir: decad.Along})
	require.NoError(t, err)
	return placedAlongZ(t, tool, -1)
}

// TestRefinePolygonCutChain refines a block that a non-convex polygon tool
// notches and a Ø4 bore then drills: the block, the notch and the bore are
// all occupancy leaves, the notch split into convex pieces, so Refine reads
// the chain's record and certifies 0.01 mm where it used to refuse a polygon
// tool outright.
//
// Shown to fail: without polygon tool leaves Refine refuses the chain with
// "this body's history is not one".
func TestRefinePolygonCutChain(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	block := slab(t, doc, 0, 0, 20, 10, 0, 10, r3.Identity())
	notched, err := decad.Cut(t.Context(), block, lShapedTool(t, doc))
	require.NoError(t, err)
	bore := placedAlongZ(t, coaxialSection(t, doc, 12, 5, 12, 2), -1)
	drilled, err := decad.Cut(t.Context(), notched, bore)
	require.NoError(t, err)
	refined, err := decad.Refine(t.Context(), drilled, units.Millimeters(0.01))
	require.NoError(t, err)
	_, err = refined.Tessellate(t.Context(), units.Millimeters(0.01))
	require.NoError(t, err)
}

// crossingRod is a cylinder of radius r along x from x0, length long, about the
// line y = cy, z = cz.
func crossingRod(t *testing.T, doc *decad.Document, x0, cy, cz, r, length float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.YZ())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(length), Dir: decad.Along})
	require.NoError(t, err)
	box, err := body.Bounds()
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{X: x0 - box.Min.X, Y: cy, Z: cz})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, move)
	require.NoError(t, err)
	return placed
}

// TestCutChordsByTheCrossing drills a 300 mm bar laid along the diagonal of
// the XY plane with a Ø6 tool along x that is as long as the bar's box. The
// boxes overlap along most of the bar, but the tool crosses the bar in a
// span of about 20 mm, so the Cut chords its operands by that crossing and
// the held result's bound stays a small fraction of the pair's chord.
//
// Shown to fail: chording by the boxes' common part keeps the pair's chord,
// and the result's bound is several times larger.
func TestCutChordsByTheCrossing(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(45))
	require.NoError(t, err)
	bar := slab(t, doc, 0, -6, 300, 6, 0, 12, turn)
	drilled, err := decad.Cut(t.Context(), bar, crossingRod(t, doc, 0, 100, 6, 3, 220))
	require.NoError(t, err)
	mesh, err := drilled.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	t.Logf("held bound %g", mesh.Bound().Base())
	require.Less(t, mesh.Bound().Base(), 0.002)
}
