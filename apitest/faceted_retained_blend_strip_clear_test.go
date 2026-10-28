package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The strip clearance rows of agent-docs/retained-modify-reach-design.md §5.6
// for the routes that blend one edge, or a mitre run, rather than a tangent
// chain: a cavity whose corner reaches into the region the blend removes, the
// rest of it in the material behind the blend face, crosses the new strip
// without touching any face the blend rewrites. Each refuses, naming a contact
// point inside the cavity's x range.

// withCavity cuts from b the box (x0, y0)-(x1, y1) over z0..z1, placed so the
// Cut stays on the mesh path.
func withCavity(t *testing.T, b *decad.Body, x0, y0, x1, y1, z0, z1 float64) *decad.Body {
	t.Helper()
	box := rectBox(t, sketch.NewWorld(), b.Document(), x0, y0, x1, y1, z1-z0)
	out, err := decad.Cut(t.Context(), b, placed(t, box, z0))
	require.NoError(t, err)
	return out
}

// requireStripCrossed checks the strip clearance refusal and that its contact
// point has x in [lo, hi], where the cavity is.
func requireStripCrossed(t *testing.T, err error, lo, hi float64) {
	t.Helper()
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "the blend meets another facet of the body")
	requireContactX(t, err, lo, hi)
}

func TestFacetedRetainedBlendStripClear(t *testing.T) {
	t.Parallel()

	t.Run("a cavity crossing a straight edge's fillet", func(t *testing.T) {
		t.Parallel()
		// The cavity's corner (y, z) = (0.25, 19.75) lies 2.47 mm from the
		// fillet's axis at (2, 18), outside the 2 mm blend and inside the
		// corner it removes. The rest of the 4×4 section lies in the material
		// the fillet keeps, so without the strip test the held audit passes.
		_, bored := boredPlate(t)
		cav := withCavity(t, bored, 5, 0.25, 7, 4.25, 15.75, 19.75)
		_, err := decad.Fillet(t.Context(), cav, straightEdge(t, cav, r3.NewVec(0, 0, 20), r3.NewVec(1, 0, 0)),
			units.Millimeters(2))
		requireStripCrossed(t, err, 5, 7)
	})

	t.Run("a cavity clear of a straight edge's fillet builds", func(t *testing.T) {
		t.Parallel()
		// A 2×1×1 cavity 3.25 mm in from the front face lies wholly in the
		// material the fillet keeps.
		_, bored := boredPlate(t)
		cav := withCavity(t, bored, 5, 3.25, 7, 4.25, 18.75, 19.75)
		before, err := cav.Volume()
		require.NoError(t, err)
		out, err := decad.Fillet(t.Context(), cav, straightEdge(t, cav, r3.NewVec(0, 0, 20), r3.NewVec(1, 0, 0)),
			units.Millimeters(2))
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, 40*4*(1-math.Pi/4))
	})

	t.Run("a cavity close under a straight edge's fillet builds", func(t *testing.T) {
		t.Parallel()
		// The strip's rows are 5° apart, so its first facet runs from the
		// tangent line (y, z) = (2, 20) to (1.826, 19.992) and its box reaches
		// down to z = 19.992. The cavity's corner (1.99, 19.995) lies inside
		// that box but 1.995 mm from the fillet's axis at (2, 18), under the
		// facet's chord: the exact test runs on the cavity's facets and finds
		// no contact.
		_, bored := boredPlate(t)
		cav := withCavity(t, bored, 5, 1.99, 7, 4, 16, 19.995)
		before, err := cav.Volume()
		require.NoError(t, err)
		out, err := decad.Fillet(t.Context(), cav, straightEdge(t, cav, r3.NewVec(0, 0, 20), r3.NewVec(1, 0, 0)),
			units.Millimeters(2))
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, 40*4*(1-math.Pi/4))
	})

	t.Run("a cavity crossing a mitre run's fillet", func(t *testing.T) {
		t.Parallel()
		x, y := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
		pair := decad.Edges(decad.EndpointAt(r3.NewVec(0, 0, 20)), decad.ParallelTo(x)).
			Or(decad.EndpointAt(r3.NewVec(0, 0, 20)), decad.ParallelTo(y))
		_, bored := boredPlate(t)
		cav := withCavity(t, bored, 5, 0.25, 7, 4.25, 15.75, 19.75)
		_, err := decad.Fillet(t.Context(), cav, pair, units.Millimeters(2))
		requireStripCrossed(t, err, 5, 7)
	})

	t.Run("a cavity crossing a hole mouth's fillet", func(t *testing.T) {
		t.Parallel()
		// The cavity's corner at radius 4.25 from the bore's axis and height
		// 19.75 lies 1.06 mm from the torus's tube centre (radius 5, height
		// 19), outside the 1 mm blend and inside the corner it removes.
		_, bored := boredPlate(t)
		cav := withCavity(t, bored, 24.25, 9.5, 25.25, 10.5, 18.75, 19.75)
		_, err := decad.Fillet(t.Context(), cav, rimAt(t, cav, 20), units.Millimeters(1))
		requireStripCrossed(t, err, 24.25, 25.25)
	})

	t.Run("a cavity crossing an open arc's fillet", func(t *testing.T) {
		t.Parallel()
		// A Ø8 bore at x = 2 breaks out of the x = 0 face, so its top rim is
		// an arc; the cavity sits against it as the hole mouth's does.
		sc := newRetentionScene()
		bored, err := decad.Cut(t.Context(), blendPlate(t, sc), zCylinderAt(t, sc, 2, 10, 4, -30, 30))
		require.NoError(t, err)
		cav := withCavity(t, bored, 6.25, 9.5, 7.25, 10.5, 18.75, 19.75)
		var arc decad.EdgeSelector
		for _, e := range cav.Edges() {
			if a, ok := e.Curve().(decad.Arc3); ok && math.Abs(a.Center.Z-20) < 1e-6 {
				arc = decad.Edges(decad.Circular(), decad.EndpointAt(e.Start().Position().Value)).Exactly(1)
			}
		}
		require.NotNil(t, arc)
		_, err = decad.Fillet(t.Context(), cav, arc, units.Millimeters(1))
		requireStripCrossed(t, err, 6.25, 7.25)
	})

	t.Run("a cavity crossing a groove edge's fillet", func(t *testing.T) {
		t.Parallel()
		// The groove's near edge is at (y, z) = (10 − √12, 20) and the 1 mm
		// ball rolls at (6, 19). The cavity's corner (6.5, 19.95) lies 1.07 mm
		// from the ball's centre and 4.06 mm from the groove's axis at
		// (10, 22): in the corner the fillet removes.
		grooved := groovedPlate(t)
		cav := withCavity(t, grooved, 15, 5.5, 17, 6.5, 18.5, 19.95)
		k := 10 - math.Sqrt(12)
		_, err := decad.Fillet(t.Context(), cav, straightEdge(t, cav, r3.NewVec(0, k, 20), r3.NewVec(1, 0, 0)),
			units.Millimeters(1))
		requireStripCrossed(t, err, 15, 17)
	})

	t.Run("a cavity crossing the ridge fillet between two bores", func(t *testing.T) {
		t.Parallel()
		// The ridge is at (17.5, 8.06) and the 1 mm ball rolls at
		// (17.5, 6.43). The cavity's corner (17.6, 7.85) lies 1.42 mm from the
		// ball's centre and more than 4 mm from both bores' axes: in the
		// corner the fillet removes.
		pair := borePair(t)
		cav := withCavity(t, pair, 17.4, 7, 17.6, 7.85, 5, 7)
		_, err := decad.Fillet(t.Context(), cav, pairEdge(t, cav), units.Millimeters(1))
		requireStripCrossed(t, err, 17.4, 17.6)
	})
}
