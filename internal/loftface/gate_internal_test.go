package loftface

import (
	"context"
	"fmt"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/stretchr/testify/require"
)

// bandGateRef is the corner-by-corner formulation BandGate replaced.
// BandGate proves injectivity over the entire source rectangle, not at
// sampled stations. Its exact rational interval boxes only tighten bounds;
// they do not change the one fixed matrix used for the whole band.
func bandGateRef(ctx context.Context, first, second Map,
	box [4]*big.Rat) (float64, float64, int, error) {
	centerU := ratDiv(ratAdd(box[0], box[1]), big.NewRat(2, 1))
	centerV := ratDiv(ratAdd(box[2], box[3]), big.NewRat(2, 1))
	c0 := VScale(VAdd(first.Deriv(centerU, centerV, 0), second.Deriv(centerU, centerV, 0)),
		big.NewRat(1, 2))
	c1 := VScale(VAdd(first.Deriv(centerU, centerV, 1), second.Deriv(centerU, centerV, 1)),
		big.NewRat(1, 2))
	c2 := VSub(second.At(centerU, centerV), first.At(centerU, centerV))
	rows, det, err := InverseRows(c0, c1, c2)
	if err != nil {
		return 0, 0, 0, err
	}
	worst := ratZero()
	// A tile's positive-denominator linear-fractional extrema occur at its
	// corners. Sixteen subdivisions limit cancellation loss in the two cap
	// terms of the band velocity while retaining one global injectivity M.
	const tiles = 16
	widthU := ratDiv(ratSub(box[1], box[0]), big.NewRat(tiles, 1))
	widthV := ratDiv(ratSub(box[3], box[2]), big.NewRat(tiles, 1))
	for iu := range tiles {
		if err := ctx.Err(); err != nil {
			return 0, 0, 0, err
		}
		u0 := ratAdd(box[0], ratMul(big.NewRat(int64(iu), 1), widthU))
		u1 := ratAdd(u0, widthU)
		for iv := range tiles {
			v0 := ratAdd(box[2], ratMul(big.NewRat(int64(iv), 1), widthV))
			v1 := ratAdd(v0, widthV)
			corners := TileCorners(u0, u1, v0, v1)
			for row, inverse := range rows {
				deviation := ratZero()
				for axis := range 2 {
					firstRange, rangeErr := derivRangeRef(first, inverse, axis, corners)
					if rangeErr != nil {
						return 0, 0, 0, rangeErr
					}
					secondRange, rangeErr := derivRangeRef(second, inverse, axis, corners)
					if rangeErr != nil {
						return 0, 0, 0, rangeErr
					}
					hull := IntervalHull(firstRange, secondRange)
					deviation = ratAdd(deviation, IntervalAbs(hull, big.NewRat(int64(boolInt(row == axis)), 1)))
				}
				velocity := IntervalSub(ScalarRange(second, inverse, corners),
					ScalarRange(first, inverse, corners))
				deviation = ratAdd(deviation, IntervalAbs(velocity, big.NewRat(int64(boolInt(row == 2)), 1)))
				if deviation.Cmp(worst) > 0 {
					worst = deviation
				}
			}
		}
	}
	if worst.Cmp(ratOne()) >= 0 {
		return RatUp(worst), 0, det.Sign(),
			fmt.Errorf("%w: loft band has no global source-map injectivity bound (norm %.6g)",
				decaderr.ErrUnsupported, RatUp(worst))
	}
	jac := ratMul(ratAbs(det), ratMul(ratAdd(ratOne(), worst),
		ratMul(ratAdd(ratOne(), worst), ratAdd(ratOne(), worst))))
	return RatUp(worst), RatUp(jac), det.Sign(), nil
}

// Draws a random Map for the gate comparison.
func randomGateMap(rng *rand.Rand) Map {
	f := func(scale float64) *big.Rat { return ratF((rng.Float64()*2 - 1) * scale) }
	vec := func(scale float64) RatVec { return RatVec{f(scale), f(scale), f(scale)} }
	return Map{Constant: vec(10), Q0: vec(5), QU: vec(2), QV: vec(2),
		G0: ratF(3 + rng.Float64()), GU: f(0.3), GV: f(0.3), Stretch: 1}
}

func TestBandGateGridMatchesReference(t *testing.T) {
	rng := rand.New(rand.NewPCG(3, 5))
	box := [4]*big.Rat{ratF(-1.25), ratF(2.5), ratF(-0.75), ratF(1.5)}
	matched := 0
	for range 3 {
		first, second := randomGateMap(rng), randomGateMap(rng)
		if PositiveDenominator(first, box) != nil || PositiveDenominator(second, box) != nil {
			continue
		}
		wantRho, wantJac, wantSign, wantErr := bandGateRef(t.Context(), first, second, box)
		gotRho, gotJac, gotSign, gotErr := BandGate(t.Context(), first, second, box)
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

// derivRangeRef is the corner-by-corner derivative range that gateDerivRange
// is checked against.
func derivRangeRef(m Map, row RatVec, axis int,
	corners [4][2]*big.Rat) (Interval, error) {
	var num, den Interval
	for i, corner := range corners {
		g := m.Denominator(corner[0], corner[1])
		if g.Sign() <= 0 {
			return Interval{}, fmt.Errorf("%w: loft cap ray denominator is not positive", decaderr.ErrUnsupported)
		}
		q := m.Numerator(corner[0], corner[1])
		qAxis, gAxis := m.QU, m.GU
		if axis == 1 {
			qAxis, gAxis = m.QV, m.GV
		}
		n := VDot(row, VSub(VScale(qAxis, g), VScale(q, gAxis)))
		if i == 0 {
			num = Interval{n, n}
			den = Interval{g, g}
			continue
		}
		if n.Cmp(num.Lo) < 0 {
			num.Lo = n
		}
		if n.Cmp(num.Hi) > 0 {
			num.Hi = n
		}
		if g.Cmp(den.Lo) < 0 {
			den.Lo = g
		}
		if g.Cmp(den.Hi) > 0 {
			den.Hi = g
		}
	}
	lo2, hi2 := ratMul(den.Lo, den.Lo), ratMul(den.Hi, den.Hi)
	values := [4]*big.Rat{ratDiv(num.Lo, lo2), ratDiv(num.Lo, hi2),
		ratDiv(num.Hi, lo2), ratDiv(num.Hi, hi2)}
	out := Interval{values[0], values[0]}
	for _, v := range values[1:] {
		if v.Cmp(out.Lo) < 0 {
			out.Lo = v
		}
		if v.Cmp(out.Hi) > 0 {
			out.Hi = v
		}
	}
	return out, nil
}
