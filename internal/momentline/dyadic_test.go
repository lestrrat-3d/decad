package momentline_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentline"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

type Point = momentline.Point
type Line = momentline.Line

// These functions freeze the original endpoint, float and bound calculations.
func oracleEvaluate(seg Line, anchor Point, order freeform.MomentIntegralOrder) ([6]float64, [6]float64, freeform.ExactMoments) {
	exact := oracleExactLineMoments(seg, anchor, order)
	start := Point{U: seg.Start.U - anchor.U, V: seg.Start.V - anchor.V}
	end := Point{U: seg.End.U - anchor.U, V: seg.End.V - anchor.V}
	u0, v0 := oraclelerp(start, end, seg.TStart)
	u1, v1 := oraclelerp(start, end, seg.TEnd)

	area := 0.5 * (u0*v1 - u1*v0)
	mu := (v1 - v0) * (u0*u0 + u0*u1 + u1*u1) / 6
	mv := -(u1 - u0) * (v0*v0 + v0*v1 + v1*v1) / 6
	var values, bounds [6]float64
	values[0], values[1], values[2] = area, mu, mv
	bounds[0] = proofarith.RationalFloatError(exact.Area, area)
	bounds[1] = proofarith.RationalFloatError(exact.Mu, mu)
	bounds[2] = proofarith.RationalFloatError(exact.Mv, mv)
	if order < freeform.MomentSecondOrder {
		return values, bounds, exact
	}

	muu := (v1 - v0) * (u0*u0*u0 + u0*u0*u1 + u0*u1*u1 + u1*u1*u1) / 12
	mvv := -(u1 - u0) * (v0*v0*v0 + v0*v0*v1 + v0*v1*v1 + v1*v1*v1) / 12
	du, dv := u1-u0, v1-v0
	intU2V := v0*(u0*u0+u0*du+du*du/3) + dv*(u0*u0/2+2*u0*du/3+du*du/4)
	muv := 0.5 * dv * intU2V
	values[3], values[4], values[5] = muu, muv, mvv
	bounds[3] = proofarith.RationalFloatError(exact.Muu, muu)
	bounds[4] = proofarith.RationalFloatError(exact.Muv, muv)
	bounds[5] = proofarith.RationalFloatError(exact.Mvv, mvv)
	return values, bounds, exact
}

func oracleEvaluateArea(seg Line, anchor Point) (float64, float64, *big.Rat) {
	exact := oracleExactLineArea(seg, anchor)
	start := Point{U: seg.Start.U - anchor.U, V: seg.Start.V - anchor.V}
	end := Point{U: seg.End.U - anchor.U, V: seg.End.V - anchor.V}
	u0, v0 := oraclelerp(start, end, seg.TStart)
	u1, v1 := oraclelerp(start, end, seg.TEnd)
	area := 0.5 * (u0*v1 - u1*v0)
	return area, proofarith.RationalFloatError(exact, area), exact
}

func oraclelerp(start, end Point, t float64) (float64, float64) {
	switch t {
	case 0:
		return start.U, start.V
	case 1:
		return end.U, end.V
	}
	return start.U + t*(end.U-start.U), start.V + t*(end.V-start.V)
}

func oracleExactLineMoments(seg Line, anchor Point, order freeform.MomentIntegralOrder) freeform.ExactMoments {
	u0 := momentline.RatLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0 := momentline.RatLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1 := momentline.RatLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1 := momentline.RatLerp(seg.Start.V, seg.End.V, seg.TEnd)
	anchorU, anchorV := proofarith.FloatRat(anchor.U), proofarith.FloatRat(anchor.V)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil || anchorU == nil || anchorV == nil {
		return freeform.ExactMoments{}
	}
	u0.Sub(u0, anchorU)
	u1.Sub(u1, anchorU)
	v0.Sub(v0, anchorV)
	v1.Sub(v1, anchorV)
	return momentline.ExactChordMoments(u0, v0, u1, v1, order)
}

func oracleExactLineArea(seg Line, anchor Point) *big.Rat {
	u0 := momentline.RatLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0 := momentline.RatLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1 := momentline.RatLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1 := momentline.RatLerp(seg.Start.V, seg.End.V, seg.TEnd)
	anchorU, anchorV := proofarith.FloatRat(anchor.U), proofarith.FloatRat(anchor.V)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil || anchorU == nil || anchorV == nil {
		return nil
	}
	u0.Sub(u0, anchorU)
	u1.Sub(u1, anchorU)
	v0.Sub(v0, anchorV)
	v1.Sub(v1, anchorV)
	return momentline.RatScale(new(big.Rat).Sub(proofbound.RatMul(u0, v1), proofbound.RatMul(u1, v0)), 1, 2)
}

func requireSameExact(t *testing.T, want, got freeform.ExactMoments) {
	t.Helper()
	a := [6]*big.Rat{want.Area, want.Mu, want.Mv, want.Muu, want.Muv, want.Mvv}
	b := [6]*big.Rat{got.Area, got.Mu, got.Mv, got.Muu, got.Muv, got.Mvv}
	for i := range a {
		require.Equal(t, a[i] == nil, b[i] == nil, "nil field %d", i)
		if a[i] != nil {
			require.Zero(t, a[i].Cmp(b[i]), "exact field %d", i)
		}
	}
}

func TestDyadicLineMomentsMatchRationalOracle(t *testing.T) {
	cases := [][8]float64{
		{0, 0, 3, 2, 0, 1, 0, 0}, {2, 3, -4, 5, 1, 0, 7, -9},
		{1, 2, 3, 4, 0.125, 0.875, 1e16, -1e16}, {1, 2, 3, 4, -2, 3, 0, 0},
		{1, 2, 1, 2, 0, 1, 1, 2}, {math.Copysign(0, -1), 0, 0, math.Copysign(0, -1), 0, 1, 0, 0},
		{math.SmallestNonzeroFloat64, 0, 0, math.SmallestNonzeroFloat64, 0, 1, 0, 0},
		{math.MaxFloat64, -math.MaxFloat64, -math.MaxFloat64, math.MaxFloat64, 0.25, 0.75, 1, -1},
		{math.Ldexp(1, 700), math.Ldexp(1, -700), math.Ldexp(1, 701), math.Ldexp(1, -699), 0, 1, 0, 0},
	}
	rng := rand.New(rand.NewPCG(17, 41))
	for range 64 {
		var c [8]float64
		for i := range c {
			c[i] = math.Ldexp(2*rng.Float64()-1, rng.IntN(1201)-600)
		}
		c[4], c[5] = rng.Float64(), rng.Float64()
		cases = append(cases, c)
	}
	for i := range 8 {
		for _, bad := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
			for _, ends := range [][2]float64{{0, 1}, {1, 0}, {0.25, 0.75}} {
				c := [8]float64{1, 2, 3, 4, ends[0], ends[1], 5, 6}
				c[i] = bad
				cases = append(cases, c)
			}
		}
	}
	for index, c := range cases {
		t.Run(big.NewInt(int64(index)).String(), func(t *testing.T) {
			seg := Line{Start: Point{U: c[0], V: c[1]}, End: Point{U: c[2], V: c[3]}, TStart: c[4], TEnd: c[5]}
			anchor := Point{U: c[6], V: c[7]}
			for _, order := range []freeform.MomentIntegralOrder{freeform.MomentAreaOrder, freeform.MomentFirstOrder, freeform.MomentSecondOrder, freeform.MomentThirdOrder} {
				values, bounds, exact := momentline.Evaluate(seg, anchor, order)
				wantValues, wantBounds, wantExact := oracleEvaluate(seg, anchor, order)
				requireSameExact(t, wantExact, exact)
				for i := range values {
					require.Equal(t, math.Float64bits(wantValues[i]), math.Float64bits(values[i]), "value %d", i)
					require.Equal(t, math.Float64bits(wantBounds[i]), math.Float64bits(bounds[i]), "bound %d", i)
				}
			}
			value, bound, exact := momentline.EvaluateArea(seg, anchor)
			wantValue, wantBound, wantExact := oracleEvaluateArea(seg, anchor)
			require.Equal(t, math.Float64bits(wantValue), math.Float64bits(value))
			require.Equal(t, math.Float64bits(wantBound), math.Float64bits(bound))
			require.Equal(t, wantExact == nil, exact == nil)
			if exact != nil {
				require.Zero(t, wantExact.Cmp(exact))
			}
		})
	}
}

func TestDyadicLineMomentResultsOwnRationals(t *testing.T) {
	seg := Line{Start: Point{U: 1, V: 2}, End: Point{U: 4, V: 5}, TEnd: 1}
	for _, order := range []freeform.MomentIntegralOrder{freeform.MomentFirstOrder, freeform.MomentSecondOrder, freeform.MomentThirdOrder} {
		exact := momentline.ExactLineMoments(seg, Point{}, order)
		fields := [6]*big.Rat{exact.Area, exact.Mu, exact.Mv, exact.Muu, exact.Muv, exact.Mvv}
		saved := [6]*big.Rat{}
		for i := range fields {
			saved[i] = new(big.Rat).Set(fields[i])
		}
		for i := range fields {
			fields[i].SetInt64(12345)
			for j := range fields {
				if j != i {
					require.Zero(t, saved[j].Cmp(fields[j]), "mutation %d affected %d", i, j)
				}
			}
			fields[i].Set(saved[i])
		}
		requireSameExact(t, oracleExactLineMoments(seg, Point{}, order), momentline.ExactLineMoments(seg, Point{}, order))
	}
}
