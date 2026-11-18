package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

// These fixtures pin general-boolean §3 "A1 as a brep" for a bridge that
// rests on a tower by one part while another part of it touches the same
// tower's top at a point. The bridge and tower form one connected piece, so
// no check between separate pieces sees the point, yet the body meets itself
// there. Each touching case keeps the mesh path or refuses; its twin, moved
// 1 mm clear, builds the exact brep.

// pinchTower is a 60 × 40 × 10 plate with a 10 mm square tower over x, y
// 10..20 and z 10..25.
func pinchTower(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	body, err := decad.Union(t.Context(), boxBody(t, doc, 0, 0, 60, 40, 10), boxBodyAtZ(t, doc, 10, 10, 20, 20, 10, 15))
	require.NoError(t, err)
	require.False(t, tookMeshPath(body))
	return body
}

// requireNoExactPinch checks that the union of a and b is no exact brep: it
// keeps the mesh path or refuses.
func requireNoExactPinch(t *testing.T, a, b *decad.Body) {
	t.Helper()
	got, err := decad.Union(t.Context(), a, b)
	if err != nil {
		require.ErrorIs(t, err, decad.ErrUnsupported)
		return
	}
	require.True(t, tookMeshPath(got), "a body meeting itself at a point is not an exact brep")
}

// requireSoundExactUnion checks that got is an exact brep of volume want that
// Verify reads as sound.
func requireSoundExactUnion(t *testing.T, doc *decad.Document, got *decad.Body, want float64) {
	t.Helper()
	require.False(t, tookMeshPath(got))
	requireEveryEdgeOnTwoFaces(t, got)
	volume, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, want, volumeMM(t, volume))
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(got)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, reading.Status)
}

// TestStackedUnionSelfPinch puts bridges over z 25..30 on the tower. Each
// rests on the tower's top and also touches it at a point: a U's bar corner
// on the tower's corner (20, 10), a peak's apex on the tower's edge y = 10
// from outside, or a notch's apex on that edge from above, where the void in
// the notch meets the void under the bridge's overhang.
func TestStackedUnionSelfPinch(t *testing.T) {
	t.Parallel()
	// uBridge's inner arm rests on the tower and its bar lies at x > x0,
	// y < 10: with x0 = 20 the bar's corner meets the tower's top corner.
	uBridge := func(t *testing.T, doc *decad.Document, x0 float64) *decad.Body {
		t.Helper()
		return polygonPrismAtZ(t, doc, 25, 5,
			[2]float64{x0, 5}, [2]float64{30, 5}, [2]float64{30, 30}, [2]float64{12, 30}, [2]float64{12, 15},
			[2]float64{18, 15}, [2]float64{18, 25}, [2]float64{25, 25}, [2]float64{25, 10}, [2]float64{x0, 10})
	}
	// peakBridge is uBridge with a bar over y 0..5 whose triangular peak
	// reaches (15, apex) below the tower.
	peakBridge := func(t *testing.T, doc *decad.Document, apex float64) *decad.Body {
		t.Helper()
		return polygonPrismAtZ(t, doc, 25, 5,
			[2]float64{5, 0}, [2]float64{30, 0}, [2]float64{30, 30}, [2]float64{12, 30}, [2]float64{12, 15},
			[2]float64{18, 15}, [2]float64{18, 25}, [2]float64{25, 25}, [2]float64{25, 5}, [2]float64{18, 5},
			[2]float64{15, apex}, [2]float64{12, 5}, [2]float64{5, 5})
	}
	t.Run("corner on corner", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		requireNoExactPinch(t, pinchTower(t, doc), uBridge(t, doc, 20))
	})
	t.Run("corner on corner, bridge first", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		tower := pinchTower(t, doc)
		requireNoExactPinch(t, uBridge(t, doc, 20), tower)
	})
	t.Run("corner clear", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		got, err := decad.Union(t.Context(), pinchTower(t, doc), uBridge(t, doc, 21))
		require.NoError(t, err)
		// The plate, the tower, and the bridge's bar, right column, top and
		// arm over 5 mm.
		requireSoundExactUnion(t, doc, got, 24000+1500+(9*5+5*15+18*5+6*10)*5)
	})
	t.Run("apex on edge", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		requireNoExactPinch(t, pinchTower(t, doc), peakBridge(t, doc, 10))
	})
	t.Run("apex clear", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		got, err := decad.Union(t.Context(), pinchTower(t, doc), peakBridge(t, doc, 9))
		require.NoError(t, err)
		// The bar, the peak, the right column, the top and the arm.
		requireSoundExactUnion(t, doc, got, 24000+1500+(25*5+6*4/2+5*20+18*5+6*10)*5)
	})
	// notchBridge covers a 30 × 20 tower over x 10..40, y 10..30 but for a
	// notch whose apex (25, apex) points down at the tower's edge y = 10.
	notchBridge := func(t *testing.T, doc *decad.Document, apex float64) *decad.Body {
		t.Helper()
		return polygonPrismAtZ(t, doc, 25, 5,
			[2]float64{5, 5}, [2]float64{45, 5}, [2]float64{45, 35}, [2]float64{30, 35}, [2]float64{30, 20},
			[2]float64{25, apex}, [2]float64{20, 20}, [2]float64{20, 35}, [2]float64{5, 35})
	}
	wideTower := func(t *testing.T, doc *decad.Document) *decad.Body {
		t.Helper()
		body, err := decad.Union(t.Context(), boxBody(t, doc, 0, 0, 60, 40, 10), boxBodyAtZ(t, doc, 10, 10, 40, 30, 10, 15))
		require.NoError(t, err)
		return body
	}
	t.Run("notch apex on edge", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		requireNoExactPinch(t, wideTower(t, doc), notchBridge(t, doc, 10))
	})
	t.Run("notch apex inside", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		got, err := decad.Union(t.Context(), wideTower(t, doc), notchBridge(t, doc, 11))
		require.NoError(t, err)
		// The bridge's 40 × 30 outline less the notch's 10 × 15 slot and its
		// 10 × 9 triangle.
		requireSoundExactUnion(t, doc, got, 24000+9000+(1200-150-45)*5)
	})
}

// TestStackedUnionSelfTangent rests a U-shaped bridge's arm on a round boss
// (about (15, 0), r 5, over z 10..25) while the bar's edge y = −5 is tangent
// to the boss's top circle at (15, −5) from outside. The arrangement cuts no
// tangency, so the point is no vertex of the scene, and the union keeps the
// mesh path or refuses. With the bar 1 mm lower it builds the brep.
func TestStackedUnionSelfTangent(t *testing.T) {
	t.Parallel()
	bridge := func(t *testing.T, doc *decad.Document, y float64) *decad.Body {
		t.Helper()
		return polygonPrismAtZ(t, doc, 25, 5,
			[2]float64{5, -10}, [2]float64{30, -10}, [2]float64{30, 20}, [2]float64{12, 20}, [2]float64{12, -1},
			[2]float64{14, -1}, [2]float64{14, 15}, [2]float64{25, 15}, [2]float64{25, y}, [2]float64{5, y})
	}
	boss := func(t *testing.T, doc *decad.Document) *decad.Body {
		t.Helper()
		body, err := decad.Union(t.Context(), boxBody(t, doc, 0, -10, 60, 40, 10), circleBodyAtZ(t, doc, 15, 5, 10, 15))
		require.NoError(t, err)
		require.False(t, tookMeshPath(body))
		return body
	}
	t.Run("tangent", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		requireNoExactPinch(t, boss(t, doc), bridge(t, doc, -5))
	})
	t.Run("clear", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		got, err := decad.Union(t.Context(), boss(t, doc), bridge(t, doc, -6))
		require.NoError(t, err)
		require.False(t, tookMeshPath(got))
		requireEveryEdgeOnTwoFaces(t, got)
		volume, err := got.Volume()
		require.NoError(t, err)
		// The plate, the boss, and the bridge's bar (25 × 4), right column
		// (5 × 21), top (18 × 5) and arm (2 × 16) over 5 mm.
		want := 60*50*10 + 25*math.Pi*15 + (25*4+5*21+18*5+2*16)*5
		require.InDelta(t, want, volume.Value.Base(), volume.Bound.Base()+1e-9)
	})
}
