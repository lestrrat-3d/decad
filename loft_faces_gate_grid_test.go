package decad

import (
	"context"
	"fmt"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

// lfBandGateRef is the corner-by-corner formulation lfBandGate replaced.
// lfBandGate proves injectivity over the entire source rectangle, not at
// sampled stations. Its exact rational interval boxes only tighten bounds;
// they do not change the one fixed matrix used for the whole band.
func lfBandGateRef(ctx context.Context, first, second loftFaceMap,
	box [4]*big.Rat) (float64, float64, int, error) {
	centerU := lfDiv(lfAdd(box[0], box[1]), big.NewRat(2, 1))
	centerV := lfDiv(lfAdd(box[2], box[3]), big.NewRat(2, 1))
	c0 := lfVScale(lfVAdd(first.deriv(centerU, centerV, 0), second.deriv(centerU, centerV, 0)),
		big.NewRat(1, 2))
	c1 := lfVScale(lfVAdd(first.deriv(centerU, centerV, 1), second.deriv(centerU, centerV, 1)),
		big.NewRat(1, 2))
	c2 := lfVSub(second.at(centerU, centerV), first.at(centerU, centerV))
	rows, det, err := lfInverseRows(c0, c1, c2)
	if err != nil {
		return 0, 0, 0, err
	}
	worst := lfZero()
	// A tile's positive-denominator linear-fractional extrema occur at its
	// corners. Sixteen subdivisions limit cancellation loss in the two cap
	// terms of the band velocity while retaining one global injectivity M.
	const tiles = 16
	widthU := lfDiv(lfSub(box[1], box[0]), big.NewRat(tiles, 1))
	widthV := lfDiv(lfSub(box[3], box[2]), big.NewRat(tiles, 1))
	for iu := range tiles {
		if err := ctx.Err(); err != nil {
			return 0, 0, 0, err
		}
		u0 := lfAdd(box[0], lfMul(big.NewRat(int64(iu), 1), widthU))
		u1 := lfAdd(u0, widthU)
		for iv := range tiles {
			v0 := lfAdd(box[2], lfMul(big.NewRat(int64(iv), 1), widthV))
			v1 := lfAdd(v0, widthV)
			corners := lfTileCorners(u0, u1, v0, v1)
			for row, inverse := range rows {
				deviation := lfZero()
				for axis := range 2 {
					firstRange, rangeErr := lfDerivRange(first, inverse, axis, corners)
					if rangeErr != nil {
						return 0, 0, 0, rangeErr
					}
					secondRange, rangeErr := lfDerivRange(second, inverse, axis, corners)
					if rangeErr != nil {
						return 0, 0, 0, rangeErr
					}
					hull := lfIntervalHull(firstRange, secondRange)
					deviation = lfAdd(deviation, lfIntervalAbs(hull, big.NewRat(int64(boolInt(row == axis)), 1)))
				}
				velocity := lfIntervalSub(lfScalarRange(second, inverse, corners),
					lfScalarRange(first, inverse, corners))
				deviation = lfAdd(deviation, lfIntervalAbs(velocity, big.NewRat(int64(boolInt(row == 2)), 1)))
				if deviation.Cmp(worst) > 0 {
					worst = deviation
				}
			}
		}
	}
	if worst.Cmp(lfOne()) >= 0 {
		return lfRatUp(worst), 0, det.Sign(),
			fmt.Errorf("%w: loft band has no global source-map injectivity bound (norm %.6g)",
				ErrUnsupported, lfRatUp(worst))
	}
	jac := lfMul(lfAbs(det), lfMul(lfAdd(lfOne(), worst),
		lfMul(lfAdd(lfOne(), worst), lfAdd(lfOne(), worst))))
	return lfRatUp(worst), lfRatUp(jac), det.Sign(), nil
}

// Draws a random loftFaceMap for the gate comparison.
func randomGateMap(rng *rand.Rand) loftFaceMap {
	f := func(scale float64) *big.Rat { return lfF((rng.Float64()*2 - 1) * scale) }
	vec := func(scale float64) loftFaceRatVec { return loftFaceRatVec{f(scale), f(scale), f(scale)} }
	return loftFaceMap{constant: vec(10), q0: vec(5), qu: vec(2), qv: vec(2),
		g0: lfF(3 + rng.Float64()), gu: f(0.3), gv: f(0.3), stretch: 1}
}

func TestLfBandGateGridMatchesReference(t *testing.T) {
	rng := rand.New(rand.NewPCG(3, 5))
	box := [4]*big.Rat{lfF(-1.25), lfF(2.5), lfF(-0.75), lfF(1.5)}
	matched := 0
	for range 3 {
		first, second := randomGateMap(rng), randomGateMap(rng)
		if lfPositiveDenominator(first, box) != nil || lfPositiveDenominator(second, box) != nil {
			continue
		}
		wantRho, wantJac, wantSign, wantErr := lfBandGateRef(t.Context(), first, second, box)
		gotRho, gotJac, gotSign, gotErr := lfBandGate(t.Context(), first, second, box)
		require.Equal(t, wantRho, gotRho)
		require.Equal(t, wantJac, gotJac)
		require.Equal(t, wantSign, gotSign)
		if wantErr == nil {
			require.NoError(t, gotErr)
		} else {
			require.EqualError(t, gotErr, wantErr.Error())
		}
		matched++
	}
	require.Positive(t, matched)
}
