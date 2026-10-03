package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/prism-boolean-design.md §15's offset-plane rows: G3's
// shared-axis arm end to end through the public Cut, and the parallel pairs
// that arm excludes, which stay on the mesh path.

// offsetPlateBody extrudes the 96×68 plate rectangle [-48, 48]×[-34, 34]
// drawn on plane by 16 mm along its normal. The plate shares w with the tools
// so a tool plane built by w.CreateOffsetPlane(w.XY(), …) is the plate's own
// datum offset.
func offsetPlateBody(t *testing.T, doc *decad.Document, w *sketch.World, plane *sketch.Plane) *decad.Body {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(-48, -34, 48, 34)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(16), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// offsetDiscBody extrudes a radius-r circle centred at (cx, 0) on plane by
// h mm along its normal.
func offsetDiscBody(t *testing.T, doc *decad.Document, w *sketch.World, plane *sketch.Plane, cx, r, h float64) *decad.Body {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(cx, 0)
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// offsetCutTools is the three hole tools (centre x, radius) every chain below
// cuts in turn.
var offsetCutTools = []struct{ cx, r float64 }{{0, 18}, {-36, 7}, {36, 7}}

// TestPrismCutOffsetPlaneToolsChainAnalytically drills three holes with tools
// sketched on CreateOffsetPlane(XY, -16): every step stays analytic and
// matches its closed-form volume, and every chain's result still fillets. Shown to
// fail: with admitPrismPairBudget's prismSharedAxisOf arm deleted, the first
// cut took the mesh path and the first anyFaceIsFaceted assertion went red.
func TestPrismCutOffsetPlaneToolsChainAnalytically(t *testing.T) {
	t.Parallel()

	// chain cuts the plate by the three tools, each sketched on plane and
	// extruded h, fillets the result, and returns the volume after each cut.
	chain := func(t *testing.T, plane func(w *sketch.World) *sketch.Plane, h float64) []float64 {
		t.Helper()
		doc := decad.New()
		w := sketch.NewWorld()
		got := offsetPlateBody(t, doc, w, w.XY())
		want := 96.0 * 68 * 16
		volumes := make([]float64, 0, len(offsetCutTools))
		for i, tool := range offsetCutTools {
			body := offsetDiscBody(t, doc, w, plane(w), tool.cx, tool.r, h)
			var err error
			got, err = decad.Cut(t.Context(), got, body)
			require.NoError(t, err, "cut %d", i+1)
			require.False(t, anyFaceIsFaceted(got), "cut %d must stay on the analytic path", i+1)
			vol, err := got.Volume()
			require.NoError(t, err)
			want -= math.Pi * tool.r * tool.r * 16
			require.InDelta(t, want, volumeMM(t, vol), 1e-6, "cut %d volume", i+1)
			require.Less(t, boundMM3(t, vol), 1e-9, "cut %d volume bound", i+1)
			volumes = append(volumes, volumeMM(t, vol))
		}

		// A fillet needs an analytic body with an exact section, so its
		// success is the public reading of the chained result's zero section
		// displacement.
		filleted, err := got.Fillet(t.Context(), verticalConvexEdge(), units.Millimeters(2))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		require.Less(t, volumeMM(t, after), volumes[len(volumes)-1], "the fillet removes material at the plate's corners")
		return volumes
	}

	offsetBelow := func(w *sketch.World) *sketch.Plane {
		plane, err := w.CreateOffsetPlane(w.XY(), -16)
		require.NoError(t, err)
		return plane
	}

	t.Run("premise: an offset of XY keeps U, V and N and moves the origin along N", func(t *testing.T) {
		t.Parallel()
		w := sketch.NewWorld()
		base, err := w.XY().Frame()
		require.NoError(t, err)
		below, err := offsetBelow(w).Frame()
		require.NoError(t, err)
		require.Equal(t, base.U(), below.U())
		require.Equal(t, base.V(), below.V())
		require.Equal(t, base.N(), below.N())
		require.Equal(t, r3.NewVec(0, 0, -16), below.Origin().Sub(base.Origin()))
	})

	offsetVolumes := chain(t, offsetBelow, 48)

	t.Run("an offset of zero and XY itself give the same volumes", func(t *testing.T) {
		t.Parallel()
		zero := chain(t, func(w *sketch.World) *sketch.Plane {
			plane, err := w.CreateOffsetPlane(w.XY(), 0)
			require.NoError(t, err)
			return plane
		}, 16)
		xy := chain(t, (*sketch.World).XY, 16)
		for i := range offsetVolumes {
			require.InDelta(t, offsetVolumes[i], zero[i], 1e-9, "offset 0, cut %d", i+1)
			require.InDelta(t, offsetVolumes[i], xy[i], 1e-9, "XY, cut %d", i+1)
		}
	})
}

// TestPrismCutOffsetPlaneExclusionsTakeMeshPath keeps every parallel pair
// outside G3's shared-axis arm on the mesh path (§4.4): each subtest's single
// Cut builds and its result carries a Faceted face. Shown to fail, one
// deletion at a time in prismSharedAxisOf: without the exact cross-product
// test "in-plane origin component" went red, without the placement
// comparison "placed along the normal" went red, and without the U/V
// comparison "tilted base" went red.
func TestPrismCutOffsetPlaneExclusionsTakeMeshPath(t *testing.T) {
	t.Parallel()

	requireMeshCut := func(t *testing.T, target, tool *decad.Body) {
		t.Helper()
		got, err := decad.Cut(t.Context(), target, tool)
		require.NoError(t, err)
		require.True(t, anyFaceIsFaceted(got), "the pair must take the mesh path")
	}

	t.Run("in-plane origin component", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		w := sketch.NewWorld()
		plate := offsetPlateBody(t, doc, w, w.XY())
		f, err := r3.NewFrame(r3.NewVec(3, 0, -16), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		plane, err := w.CreatePlaneFromFrame(f)
		require.NoError(t, err)
		requireMeshCut(t, plate, offsetDiscBody(t, doc, w, plane, 0, 18, 48))
	})

	t.Run("tilted base", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		w := sketch.NewWorld()
		tilted, err := r3.NewFrame(r3.NewVec(1, 2, 3), r3.NewVec(1, 1, 0), r3.NewVec(0, 1, 1))
		require.NoError(t, err)
		base, err := w.CreatePlaneFromFrame(tilted)
		require.NoError(t, err)
		below, err := w.CreateOffsetPlane(base, -16)
		require.NoError(t, err)
		baseFrame, err := base.Frame()
		require.NoError(t, err)
		belowFrame, err := below.Frame()
		require.NoError(t, err)
		// Extrude rebuilds each plane frame through r3.NewFrame, which brings
		// the two U back to one value and leaves V one ulp apart; either way
		// the stored bits differ and the shared-axis arm refuses the pair.
		require.NotEqual(t, baseFrame.U(), belowFrame.U(), "premise: the offset of a tilted base re-normalises U to other bits")
		plate := offsetPlateBody(t, doc, w, base)
		requireMeshCut(t, plate, offsetDiscBody(t, doc, w, below, 0, 18, 48))
	})

	t.Run("placed along the normal", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		w := sketch.NewWorld()
		plate := offsetPlateBody(t, doc, w, w.XY())
		tool := translated(t, offsetDiscBody(t, doc, w, w.XY(), 0, 18, 48), 0, 0, -16)
		requireMeshCut(t, plate, tool)
	})

	t.Run("opposite normal", func(t *testing.T) {
		// A plane 32 mm up whose V is reversed carries N = (0, 0, -1); the tool
		// swept 48 mm along it is the same z -16..32 solid.
		t.Parallel()
		doc := decad.New()
		w := sketch.NewWorld()
		plate := offsetPlateBody(t, doc, w, w.XY())
		flipped, err := r3.NewFrame(r3.NewVec(0, 0, 32), r3.NewVec(1, 0, 0), r3.NewVec(0, -1, 0))
		require.NoError(t, err)
		require.Equal(t, r3.NewVec(0, 0, -1), flipped.N(), "premise: the tool plane's normal opposes the plate's")
		above, err := w.CreatePlaneFromFrame(flipped)
		require.NoError(t, err)
		requireMeshCut(t, plate, offsetDiscBody(t, doc, w, above, 0, 18, 48))
	})
}
