package loftcontact_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// xyFrame is the world XY frame a prism sketched on world.XY() records.
func xyFrame(t *testing.T) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return frame
}

func TestLoftOutputWorldRefusesOversizedReplayArithmetic(t *testing.T) {
	frame := xyFrame(t)
	largeDenominator := new(big.Int).Lsh(big.NewInt(1), 8000)
	_, held, _, admitted := loftcontact.OutputWorld(frame, loftcontact.OutputPoint{
		U: new(big.Rat).SetFrac(big.NewInt(1), largeDenominator),
		V: big.NewRat(0, 1), Z: big.NewRat(0, 1),
	})
	require.False(t, admitted)
	require.Equal(t, r3.Vec{}, held)
	// Float64 aligns the numerator before dividing. This input fits the
	// rational operands but would allocate 8,195 bits during alignment.
	alignmentDenominator := new(big.Int).Lsh(big.NewInt(1), 8140)
	_, held, _, admitted = loftcontact.OutputWorld(frame, loftcontact.OutputPoint{
		U: new(big.Rat).SetFrac(big.NewInt(1), alignmentDenominator),
		V: big.NewRat(0, 1), Z: big.NewRat(0, 1),
	})
	require.False(t, admitted)
	require.Equal(t, r3.Vec{}, held)

	largeNumerator := new(big.Int).Lsh(big.NewInt(1), 8192)
	shell := loftcontact.NewOutputShell()
	_, admitted = shell.Vertex(frame, loftcontact.OutputPoint{
		U: new(big.Rat).SetInt(largeNumerator),
		V: big.NewRat(0, 1), Z: big.NewRat(0, 1),
	}, loftcontact.OutputFace{Kind: 0})
	require.False(t, admitted)
}

func TestLoftOutputCrossZeroRefusesOversizedReplayArithmetic(t *testing.T) {
	denominator := new(big.Int).Lsh(big.NewInt(1), 8000)
	tiny := new(big.Rat).SetFrac(big.NewInt(1), denominator)
	zero := big.NewRat(0, 1)
	points := [3]loftcontact.OutputPoint{
		{U: zero, V: zero, Z: zero},
		{U: tiny, V: zero, Z: zero},
		{U: zero, V: tiny, Z: zero},
	}
	// The exact triangle has nonzero area; the unsafe cross product refuses.
	require.True(t, loftcontact.OutputCrossZero(points))
}

func TestLoftOutputTileCapRefusesOversizedReturnedPoint(t *testing.T) {
	frame := xyFrame(t)
	zero, one := big.NewRat(0, 1), big.NewRat(1, 1)
	large := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 8192))
	polygon := []loftcontact.Point{
		{U: zero, V: zero}, {U: large, V: zero}, {U: zero, V: one},
	}
	shell := loftcontact.NewOutputShell()
	admitted, err := loftcontact.TileCap(t.Context(), proofbound.NewWorkBudget(t.Context()),
		frame, &shell, polygon, zero, loftcontact.OutputFace{Kind: 0}, false)
	require.NoError(t, err)
	require.False(t, admitted)
	require.Empty(t, shell.Triangles)
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 8140))
	polygon = []loftcontact.Point{
		{U: zero, V: zero}, {U: tiny, V: zero}, {U: zero, V: one},
	}
	admitted, err = loftcontact.TileCap(t.Context(), proofbound.NewWorkBudget(t.Context()),
		frame, &shell, polygon, zero, loftcontact.OutputFace{Kind: 0}, false)
	require.NoError(t, err)
	require.False(t, admitted)
	require.Empty(t, shell.Triangles)
}
