package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/general-boolean-design.md §3 A3 through the public
// booleans: two operands whose outlines share a span of one carrier, which
// sketch resolves once, under one operand's entity.

// requireAnalyticVolume asserts b is one analytic lump (no Faceted face)
// whose volume bound contains want.
func requireAnalyticVolume(t *testing.T, b *decad.Body, want float64) {
	t.Helper()
	for _, f := range b.Faces() {
		require.NotEqual(t, decad.KindFaceted, f.Surface().Kind())
	}
	require.Len(t, b.Lumps(), 1)
	vol, err := b.Volume()
	require.NoError(t, err)
	decadtest.Measures(t, "volume", vol, units.CubicMillimeters(want))
}

// TestPrismBooleanSharedWallBoxes is §2's W rows: a 10 mm box [0, 10]² and a
// second box sharing its wall x = 10 whole (W1), over an interior part of it
// (W2), along a longer wall reaching past both ends (W3), and overlapping it
// by 1 mm with collinear floor and roof (W4) union into one analytic prism
// of the closed-form volume, and Cut of the box by the second one (W6, and
// W4's overlap) leaves the box, or the box less the overlap.
//
// Shown to fail on sketch's pin before coincident line carriers (every row
// refused, the arrangement reporting an invalid region), and W2's Union with
// prismcells.SplitRuns returning the cells unsplit (the cell beside the
// partial span walks the wall whole, the merge finds no shared edge, and the
// pair takes the mesh path's coplanar refusal).
func TestPrismBooleanSharedWallBoxes(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name           string
		b              [4]float64
		union, cutRest float64
	}{
		{"W1 whole wall", [4]float64{10, 0, 20, 10}, 2000, 1000},
		{"W2 partial wall", [4]float64{10, 2, 20, 8}, 1600, 1000},
		{"W3 longer wall", [4]float64{10, -5, 20, 15}, 3000, 1000},
		{"W4 1 mm overlap, collinear floor and roof", [4]float64{9, 0, 19, 10}, 1900, 900},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			a := boxBody(t, doc, 0, 0, 10, 10, 10)
			b := boxBody(t, doc, tc.b[0], tc.b[1], tc.b[2], tc.b[3], 10)
			union, err := decad.Union(t.Context(), a, b)
			require.NoError(t, err)
			requireAnalyticVolume(t, union, tc.union)

			doc = decad.New()
			a = boxBody(t, doc, 0, 0, 10, 10, 10)
			b = boxBody(t, doc, tc.b[0], tc.b[1], tc.b[2], tc.b[3], 10)
			cut, err := decad.Cut(t.Context(), a, b)
			require.NoError(t, err)
			requireAnalyticVolume(t, cut, tc.cutRest)
		})
	}
}

// TestPrismBooleanTouchingCornerStaysOnMesh is §2's S12c: two boxes meeting
// at one vertical edge share no span, so A3 does not reach them. Their
// Union is two lumps joined along that edge, which no prism payload holds,
// and the pair keeps the mesh path's coplanar refusal.
func TestPrismBooleanTouchingCornerStaysOnMesh(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 10, 10, 20, 20, 10)
	_, err := decad.Union(t.Context(), a, b)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	var booleanErr *decad.BooleanError
	require.ErrorAs(t, err, &booleanErr)
	require.Equal(t, decad.BooleanUnsupportedContact, booleanErr.Code)
}

// TestPrismBooleanToothTouchingHubDoesNotOverlap is a tooth whose root arc
// lies on the hub circle, outside the hub. sketch names the shared arc under
// the tooth and withdraws it from the hub circle, so the tooth's cell has
// no hub edge of its own. Read through the shared arc, the tooth's cell is
// outside the hub: Cut of the tooth by the hub leaves the whole tooth
// analytically, Intersect builds no analytic body (its one candidate is
// empty, and the mesh path refuses the contact), and Verify reports no
// interference. Union stays analytic (TestPrismUnionGearToothOnHubSharedCarrier).
//
// Shown to fail with prismcells.Classify propagating the hub's membership
// across the shared arc, as it did before the coincident reading: Intersect
// then built the whole tooth, 220.84 mm³, and Verify reported the touching
// pair Interfering with one row of that volume.
func TestPrismBooleanToothTouchingHubDoesNotOverlap(t *testing.T) {
	t.Parallel()
	const r, r2, th1, th2, h = 20.0, 25.0, 0.0, 0.2, 10.0
	t.Run("Cut and Intersect", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		hub := hubBody(t, doc, r, h)
		tooth := toothBody(t, doc, r, r2, th1, th2, h)
		toothVol, err := tooth.Volume()
		require.NoError(t, err)

		_, err = decad.Intersect(t.Context(), hub, tooth)
		require.ErrorIs(t, err, decad.ErrUnsupported)

		rest, err := decad.Cut(t.Context(), tooth, hub)
		require.NoError(t, err)
		requireAnalyticVolume(t, rest, volumeMM(t, toothVol))
	})
	t.Run("Verify", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		hubBody(t, doc, r, h)
		toothBody(t, doc, r, r2, th1, th2, h)
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.NotEqual(t, decad.Interfering, report.Status)
		require.Empty(t, report.Interferences)
	})
}
