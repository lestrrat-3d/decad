package prismplacement_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCutTranslationExactShiftAndGates(t *testing.T) {
	u := r3.NewVec(0.024051868856736578, -0.31446392750578867, 0.94896467052368849)
	v := r3.NewVec(0.007967648549189436, 0.94926942345028398, 0.31436297218177989)
	fa, err := r3.NewFrame(r3.NewVec(229.99678960557387, 257.82946876781881, 108.21070606486229), u, v)
	require.NoError(t, err)
	fb, err := r3.NewFrame(r3.NewVec(241.39737544366702, 108.77356713007498, 558.01995989309057), u, v)
	require.NoError(t, err)
	xa, err := r3.FromFrame(fa)
	require.NoError(t, err)
	xb, err := r3.FromFrame(fb)
	require.NoError(t, err)
	a := levelOperand(t, 0, 0, 20)
	b := a
	a.Xform, b.Xform = xa, xb
	_, shift, ok := prismplacement.CutTranslation(a, b)
	require.True(t, ok)
	require.Negative(t, shift.Sign())
	_, top, ok := prismplacement.ShiftedIntervalBy(b, shift)
	require.True(t, ok)
	floor := new(big.Rat).Sub(big.NewRat(20, 1), top)
	thickness, _ := floor.Float64()
	require.InDelta(t, 1.3628423294448388e-14, thickness, 1e-25)
	_, _, low, high, ok := prismplacement.IntervalRangesShift(a, b, shift)
	require.True(t, ok)
	require.True(t, low.Hi.Cmp(big.NewRat(0, 1)) < 0)
	require.True(t, high.Hi.Cmp(big.NewRat(20, 1)) < 0)

	differentFrame := b
	differentFrame.Frame = levelOperand(t, 1, 0, 20).Frame
	_, _, ok = prismplacement.CutTranslation(a, differentFrame)
	require.False(t, ok)
	rotated, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Radians(.2))
	require.NoError(t, err)
	differentBasis := b
	differentBasis.Xform = rotated
	_, _, ok = prismplacement.CutTranslation(a, differentBasis)
	require.False(t, ok)
	zero := a
	zero.Xform = r3.Transform{}
	_, _, ok = prismplacement.CutTranslation(zero, zero)
	require.False(t, ok)
	_, err = r3.Translation(r3.NewVec(math.NaN(), 0, 0))
	require.Error(t, err)
	nonfinite := b
	nonfinite.Z0 = math.Inf(1)
	_, _, ok = prismplacement.ShiftedIntervalBy(nonfinite, shift)
	require.False(t, ok)
}
