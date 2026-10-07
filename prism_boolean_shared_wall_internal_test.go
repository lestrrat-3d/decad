package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/stretchr/testify/require"
)

// This file is docs/general-boolean-design.md §3 A3's white-box suite: two
// operands whose outlines share a span of one carrier, which sketch resolves
// once under one operand's entity (internal/prismcells/coincident.go).
// apitest/prism_boolean_shared_wall_test.go covers the public booleans on
// the same shapes.

// prismMirroredL is mirror-pattern §2's L, the receiver of M3 and T2.
func prismMirroredL(t *testing.T, doc *Document) *Body {
	t.Helper()
	return internalPolyPrismBody(t, doc, [][2]float64{{0, 0}, {20, 0}, {20, 5}, {5, 5}, {5, 20}, {0, 20}}, prismFixtureHeight)
}

// TestPrismSharedWallMirroredL is M3 and T2: the L unioned with its image
// across x = 15 shares the collinear walls y = 0 and y = 5 over x ∈ [10, 20]
// and builds analytically, 2 × 175 − 50 mm² over the 10 mm sweep; Verify on
// the L beside its image measures the 50 mm² overlap as one interference
// row. The image is reflected by a placement, so it brings a displacement,
// and both spans are boundary of the union, which the displacement covers.
//
// Shown to fail with CrossingCharge reading the two coincident lines as a
// crossing (the zero sine at each span end has no charge, and both fall
// back to the mesh path).
func TestPrismSharedWallMirroredL(t *testing.T) {
	t.Parallel()
	t.Run("M3", func(t *testing.T) {
		t.Parallel()
		doc := New()
		l := prismMirroredL(t, doc)
		image, err := l.PlacedCopy(t.Context(), prismMirrorAcrossX(t, 15))
		require.NoError(t, err)
		got, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, l, image)
		require.NoError(t, err)
		require.True(t, ok)
		require.Positive(t, got.sectionDelta, "the reflected image brings a displacement")
		body, err := Union(t.Context(), l, image)
		require.NoError(t, err)
		requireSharedWallVolume(t, body, 300*prismFixtureHeight)
	})
	t.Run("T2", func(t *testing.T) {
		t.Parallel()
		doc := New()
		l := prismMirroredL(t, doc)
		_, err := l.PlacedCopy(t.Context(), prismMirrorAcrossX(t, 15))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, Interfering, report.Status)
		require.Len(t, report.Interferences, 1)
		requireSharedWallMeasure(t, report.Interferences[0].Volume, 50*prismFixtureHeight)
	})
}

// TestPrismSharedWallDisplacedInteriorSpanFallsBack pins
// prismSceneDelta.sharedSpansBounded: a box beside its own mirror image
// across x = 10 shares the wall x = 10, and the image brings a
// displacement. Union keeps that wall inside the result, where the two true
// walls can sit apart and leave a sliver no recorded edge bounds, so it
// takes the mesh path; Cut keeps it as the result's boundary, which the
// span's charge covers, and builds the box itself.
//
// Shown to fail with sharedSpansBounded always reporting true (Union then
// built analytically).
func TestPrismSharedWallDisplacedInteriorSpanFallsBack(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 10, 10, prismFixtureHeight)
	image, err := box.PlacedCopy(t.Context(), prismMirrorAcrossX(t, 10))
	require.NoError(t, err)

	_, ok, err := tryPrismBoolean(t.Context(), meshbool.OpUnion, box, image)
	require.NoError(t, err)
	require.False(t, ok)

	cut, ok, err := tryPrismBoolean(t.Context(), meshbool.OpCut, box, image)
	require.NoError(t, err)
	require.True(t, ok)
	require.Positive(t, cut.sectionDelta)
}

// TestPrismSharedWallChargesTheGap pins CoincidentReading.Gap: B's wall runs
// from (10, 0) to (10 − 1.07e-14, 10), inside sketch's identity band of A's
// wall x = 10, so sketch resolves the two as one carrier and names the span
// under A's line. Cut keeps A's wall as the result's boundary while B's true
// wall leans into A by up to the gap at y = 10, so the result's section
// displacement must cover that distance, computed here exactly.
//
// Shown to fail with the gap term deleted from chargeCrossings (the
// displacement read 0).
func TestPrismSharedWallChargesTheGap(t *testing.T) {
	t.Parallel()
	const lean = 9.99999999999999
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, prismFixtureHeight)
	b := internalPolyPrismBody(t, doc, [][2]float64{{10, 0}, {20, 0}, {20, 10}, {lean, 10}}, prismFixtureHeight)
	got, ok, err := tryPrismBoolean(t.Context(), meshbool.OpCut, a, b)
	require.NoError(t, err)
	require.True(t, ok)
	gap, _ := new(big.Rat).Sub(new(big.Rat).SetInt64(10), new(big.Rat).SetFloat64(lean)).Float64()
	require.Positive(t, gap)
	require.GreaterOrEqual(t, got.sectionDelta, gap)
}

// requireSharedWallVolume asserts b's volume bound contains want.
func requireSharedWallVolume(t *testing.T, b *Body, want float64) {
	t.Helper()
	vol, err := b.Volume()
	require.NoError(t, err)
	requireSharedWallMeasure(t, vol, want)
}

// requireSharedWallMeasure asserts m's bound contains want.
func requireSharedWallMeasure(t *testing.T, m Measurement, want float64) {
	t.Helper()
	require.LessOrEqual(t, math.Abs(m.Value.Base()-want), m.Bound.Base(), "%v ± %v against %v", m.Value.Base(), m.Bound.Base(), want)
}
