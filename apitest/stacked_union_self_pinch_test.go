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
	body, err := decad.Union(boxBody(t, doc, 0, 0, 60, 40, 10), boxBodyAtZ(t, doc, 10, 10, 20, 20, 10, 15)).Do(t.Context())
	require.NoError(t, err)
	require.False(t, tookMeshPath(body))
	return body
}

// requireNoExactPinch checks that the union of a and b is no exact brep: it
// keeps the mesh path or refuses.
func requireNoExactPinch(t *testing.T, a, b *decad.Body) {
	t.Helper()
	got, err := decad.Union(a, b).Do(t.Context())
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
		got, err := decad.Union(pinchTower(t, doc), uBridge(t, doc, 21)).Do(t.Context())
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
		got, err := decad.Union(pinchTower(t, doc), peakBridge(t, doc, 9)).Do(t.Context())
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
		body, err := decad.Union(boxBody(t, doc, 0, 0, 60, 40, 10), boxBodyAtZ(t, doc, 10, 10, 40, 30, 10, 15)).Do(t.Context())
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
		got, err := decad.Union(wideTower(t, doc), notchBridge(t, doc, 11)).Do(t.Context())
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
		body, err := decad.Union(boxBody(t, doc, 0, -10, 60, 40, 10), circleBodyAtZ(t, doc, 15, 5, 10, 15)).Do(t.Context())
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
		got, err := decad.Union(boss(t, doc), bridge(t, doc, -6)).Do(t.Context())
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

// TestStackedUnionPinchRecordCount meets two L-shaped prisms at the point
// p = (30, 20), each 10 thick: a lower L over z 0..10 missing the quadrant
// x > 30, y < 20 at p, and an upper L over z 10..20 missing x < 30, y > 20.
// Round p at z = 10 the lower L's floor and the upper L's ceiling sit in
// opposite quadrants between two quadrants both cover, so the empty spaces
// over the floor and under the ceiling meet at p and the body meets itself
// there.
//
// With a wedge added to each operand's other slab, every floor and ceiling
// corner at p turns from a lower record's edge to an upper record's, so only
// the count of records with edges at p (two of each side, not one) reads the
// pinch.
func TestStackedUnionPinchRecordCount(t *testing.T) {
	t.Parallel()
	lowerL := func(t *testing.T, doc *decad.Document) *decad.Body {
		t.Helper()
		return polygonPrismAtZ(t, doc, 0, 10,
			[2]float64{20, 10}, [2]float64{30, 10}, [2]float64{30, 20}, [2]float64{40, 20},
			[2]float64{40, 30}, [2]float64{20, 30})
	}
	upperL := func(t *testing.T, doc *decad.Document) *decad.Body {
		t.Helper()
		return polygonPrismAtZ(t, doc, 10, 10,
			[2]float64{22, 12}, [2]float64{38, 12}, [2]float64{38, 28}, [2]float64{30, 28},
			[2]float64{30, 20}, [2]float64{22, 20})
	}
	t.Run("two Ls", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		requireNoExactPinch(t, lowerL(t, doc), upperL(t, doc))
	})
	// operands builds A, the lower L with an upper wedge pointing at p from
	// x > 30, and B, a lower wedge pointing at p from x < 30 with the upper L.
	operands := func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
		t.Helper()
		a, err := decad.Union(lowerL(t, doc), polygonPrismAtZ(t, doc, 10, 10,
			[2]float64{30, 20}, [2]float64{35, 15}, [2]float64{35, 25})).Do(t.Context())
		require.NoError(t, err)
		b, err := decad.Union(polygonPrismAtZ(t, doc, 0, 10,
			[2]float64{30, 20}, [2]float64{25, 25}, [2]float64{25, 15}), upperL(t, doc)).Do(t.Context())
		require.NoError(t, err)
		return a, b
	}
	t.Run("wedged", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		a, b := operands(t, doc)
		requireNoExactPinch(t, a, b)
	})
	t.Run("wedged, swapped", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		a, b := operands(t, doc)
		requireNoExactPinch(t, b, a)
	})
}
