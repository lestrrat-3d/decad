package loftcontact

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestOrderedSeamBoundRefusesExtremeFiniteRectangle(t *testing.T) {
	f, delta := math.MaxFloat64, math.SmallestNonzeroFloat64
	carrier := [2]sectionrecord.Point2{{U: delta}, {U: f}}
	vertices := [4]sectionrecord.Point2{
		{U: f / 4, V: -delta}, {U: f / 2, V: -delta},
		{U: f / 2, V: f}, {U: f / 4, V: f},
	}
	left := [2]sectionrecord.Point2{vertices[3], vertices[0]}
	right := [2]sectionrecord.Point2{vertices[1], vertices[2]}
	budget := proofbound.NewWallWorkBudgetWithOperation(1000, proofbound.NewWorkBudget(t.Context()))
	var seams [2]LineSeam
	for i, line := range [][2]sectionrecord.Point2{left, right} {
		seam, ok, err := CertifyLineSeam(t.Context(), budget, line, line, carrier)
		require.NoError(t, err)
		require.True(t, ok)
		seams[i] = seam
		bounded, err := lineSeamOrderingBound(budget, seam)
		require.NoError(t, err)
		require.False(t, bounded)
	}
	// Each factor is already constructed. Their bit lengths prove the raw
	// cubic product exceeds the ceiling without allocating that product.
	productMinBits := seams[1].carrierNum[0].Num().BitLen() +
		seams[0].carrierDen[0].Num().BitLen() - 1
	require.Greater(t, productMinBits, 8192)
	_, ok, err := CertifyOrderedLineSeams(t.Context(), budget, left, left, right, right, carrier)
	require.NoError(t, err)
	require.False(t, ok)

	lines := make([]sectionrecord.LineSeg, len(vertices))
	for i := range lines {
		lines[i] = sectionrecord.LineSeg{
			Start: vertices[i], End: vertices[(i+1)%len(vertices)], TStart: 0, TEnd: 1,
		}
	}
	crossing, _, _, ok, err := CertifyCarrierOccupancy(t.Context(), budget, lines, lines, carrier)
	require.NoError(t, err)
	require.False(t, ok)
	require.Equal(t, [2]int{}, crossing)
}

func TestOrderedSeamBoundPollsCancellation(t *testing.T) {
	line := [2]sectionrecord.Point2{{U: 0, V: -1}, {U: 0, V: 1}}
	carrier := [2]sectionrecord.Point2{{U: -1}, {U: 1}}
	seam, ok, err := CertifyLineSeam(t.Context(), proofbound.NewWorkBudget(t.Context()),
		line, line, carrier)
	require.NoError(t, err)
	require.True(t, ok)
	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	steps := 0
	budget := &proofbound.WorkBudget{StepFn: func() error {
		steps++
		if steps == 2 {
			cancel()
		}
		return ctx.Err()
	}}
	bounded, err := lineSeamOrderingBound(budget, seam)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, bounded)
	require.Equal(t, 2, steps)
}
