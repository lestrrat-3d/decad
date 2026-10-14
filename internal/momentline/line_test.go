package momentline_test

import (
	"math"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentline"
	"github.com/stretchr/testify/require"
)

func TestEvaluateAreaMatchesThreeMomentReference(t *testing.T) {
	cases := []struct {
		line   momentline.Line
		anchor momentline.Point
	}{
		{momentline.Line{Start: momentline.Point{U: 1, V: 2}, End: momentline.Point{U: 3, V: 4},
			TStart: 0, TEnd: 1}, momentline.Point{U: 5, V: -7}},
		{momentline.Line{Start: momentline.Point{U: 1 + math.SmallestNonzeroFloat64, V: -0.5},
			End: momentline.Point{U: -3.25, V: 4.5}, TStart: 0.875, TEnd: 0.125},
			momentline.Point{U: -1.25, V: 2.75}},
		{momentline.Line{Start: momentline.Point{U: math.Ldexp(1, 500), V: math.Ldexp(1, -500)},
			End:    momentline.Point{U: math.Ldexp(1, 500) + math.Ldexp(1, 448), V: -math.Ldexp(1, -500)},
			TStart: 0, TEnd: 1}, momentline.Point{U: math.Ldexp(1, 500), V: 0}},
		{momentline.Line{Start: momentline.Point{U: 0, V: 0}, End: momentline.Point{U: 0, V: 0},
			TStart: 0, TEnd: 1}, momentline.Point{}},
		{momentline.Line{Start: momentline.Point{U: math.NaN(), V: 1},
			End: momentline.Point{U: 2, V: 3}, TStart: 0, TEnd: 1}, momentline.Point{}},
	}
	rng := rand.New(rand.NewSource(17))
	for range 200 {
		coord := func() float64 {
			mant := float64(rng.Intn(2047)-1023) / 1024
			return math.Ldexp(mant, rng.Intn(1001)-500)
		}
		cases = append(cases, struct {
			line   momentline.Line
			anchor momentline.Point
		}{momentline.Line{
			Start:  momentline.Point{U: coord(), V: coord()},
			End:    momentline.Point{U: coord(), V: coord()},
			TStart: 0.125, TEnd: 0.875,
		}, momentline.Point{U: coord(), V: coord()}})
	}
	for i, tc := range cases {
		values, bounds, exact := momentline.Evaluate(tc.line, tc.anchor, freeform.MomentAreaOrder)
		value, bound, area := momentline.EvaluateArea(tc.line, tc.anchor)
		require.Equal(t, math.Float64bits(values[0]), math.Float64bits(value), "case %d value", i)
		require.Equal(t, math.Float64bits(bounds[0]), math.Float64bits(bound), "case %d bound", i)
		if exact.Area == nil {
			require.Nil(t, area, "case %d exact", i)
			continue
		}
		require.NotNil(t, area, "case %d exact", i)
		require.Zero(t, area.Cmp(exact.Area), "case %d exact", i)
	}
}
