package meshbool_test

import (
	"context"
	"errors"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// rationalAreaSign uses an independent rational shoelace sum as the oracle.
func rationalAreaSign(poly []meshbool.Xp2) int {
	var sum big.Rat
	for i, a := range poly {
		b := poly[(i+1)%len(poly)]
		left := new(big.Rat).Mul(a.U, b.V)
		right := new(big.Rat).Mul(b.U, a.V)
		sum.Add(&sum, left.Sub(left, right))
	}
	return sum.Sign()
}

func areaPoint(u, v *big.Rat) meshbool.Xp2 { return meshbool.NewXP2(u, v) }

func TestPolyArea2SignMatchesRationalArea(t *testing.T) {
	t.Parallel()
	point := func(u, v int64) meshbool.Xp2 { return areaPoint(big.NewRat(u, 1), big.NewRat(v, 1)) }
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 1100))
	huge := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 1100))
	lowerLimit := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 300))
	upperLimit := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 250))
	shift := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 200))
	near := new(big.Rat).Add(shift, big.NewRat(1, 1))
	cases := map[string][]meshbool.Xp2{
		"empty":         {},
		"positive":      {point(0, 0), point(3, 0), point(3, 2), point(0, 2)},
		"negative":      {point(0, 2), point(3, 2), point(3, 0), point(0, 0)},
		"zero":          {point(0, 0), point(2, 0), point(0, 0), point(2, 0)},
		"repeated":      {point(0, 0), point(3, 0), point(3, 0), point(0, 2)},
		"near zero":     {areaPoint(shift, shift), areaPoint(near, shift), areaPoint(shift, near)},
		"near negative": {areaPoint(shift, near), areaPoint(near, shift), areaPoint(shift, shift)},
		"subnormal":     {point(0, 0), areaPoint(tiny, big.NewRat(0, 1)), point(0, 1)},
		"overflow":      {point(0, 0), areaPoint(huge, big.NewRat(0, 1)), point(0, 1)},
		"lower limit":   {point(0, 0), areaPoint(lowerLimit, big.NewRat(0, 1)), point(0, 1)},
		"upper limit":   {point(0, 0), areaPoint(upperLimit, big.NewRat(0, 1)), point(0, 1)},
		"cancellation": {point(0, 0), point(5, 0), point(5, 5), point(0, 5),
			point(0, 0), point(0, 5), point(5, 5), point(5, 0)},
	}
	unknown := meshbool.Xp2{U: big.NewRat(1, 1), V: big.NewRat(2, 1), Fu: 1e100, Fv: -1e100,
		FloatFinite: true}
	cases["unknown cache"] = []meshbool.Xp2{point(0, 0), unknown, point(3, 0)}
	cases["long fallback"] = make([]meshbool.Xp2, 1025)
	for i := range cases["long fallback"] {
		cases["long fallback"][i] = point(0, 0)
	}
	cases["1024-edge cancellation"] = make([]meshbool.Xp2, 0, 1024)
	for range 128 {
		cases["1024-edge cancellation"] = append(cases["1024-edge cancellation"],
			cases["cancellation"]...)
	}
	for name, poly := range cases {
		t.Run(name, func(t *testing.T) {
			before := make([][4]string, len(poly))
			for i, p := range poly {
				before[i] = [4]string{p.U.RatString(), p.V.RatString(),
					big.NewInt(0).SetUint64(math.Float64bits(p.Fu)).String(),
					big.NewInt(0).SetUint64(math.Float64bits(p.Fv)).String()}
			}
			got, err := meshbool.PolyArea2Sign(proofbound.NewWorkBudget(t.Context()), poly)
			require.NoError(t, err)
			require.Equal(t, rationalAreaSign(poly), got)
			for i, p := range poly {
				require.Equal(t, before[i][0], p.U.RatString())
				require.Equal(t, before[i][1], p.V.RatString())
				require.Equal(t, before[i][2], big.NewInt(0).SetUint64(math.Float64bits(p.Fu)).String())
				require.Equal(t, before[i][3], big.NewInt(0).SetUint64(math.Float64bits(p.Fv)).String())
			}
		})
	}
	rng := rand.New(rand.NewPCG(107, 211))
	for k := range 400 {
		poly := make([]meshbool.Xp2, 3+rng.IntN(6))
		for i := range poly {
			u := big.NewRat(int64(rng.IntN(2001)-1000), int64(1+rng.IntN(2000)))
			v := big.NewRat(int64(rng.IntN(2001)-1000), int64(1+rng.IntN(2000)))
			poly[i] = areaPoint(u, v)
		}
		got, err := meshbool.PolyArea2Sign(proofbound.NewWorkBudget(t.Context()), poly)
		require.NoError(t, err, "case %d", k)
		require.Equal(t, rationalAreaSign(poly), got, "case %d", k)
	}
}

func TestPolyArea2SignHomogeneousSpellingAndBudget(t *testing.T) {
	t.Parallel()
	xpt := func(x, y, w, scale int64) proof.Xpt {
		return proof.Xpt{X: big.NewInt(x * scale), Y: big.NewInt(y * scale),
			Z: big.NewInt(0), W: big.NewInt(w * scale)}
	}
	poly := []meshbool.Xp2{
		meshbool.NewXP2FromXpt(xpt(1, 2, 3, 7), 0, 1),
		meshbool.NewXP2FromXpt(xpt(8, 4, 5, 11), 0, 1),
		meshbool.NewXP2FromXpt(xpt(3, 9, 7, 13), 0, 1),
	}
	unscaled := []meshbool.Xp2{
		meshbool.NewXP2FromXpt(xpt(1, 2, 3, 1), 0, 1),
		meshbool.NewXP2FromXpt(xpt(8, 4, 5, 1), 0, 1),
		meshbool.NewXP2FromXpt(xpt(3, 9, 7, 1), 0, 1),
	}
	want := rationalAreaSign(poly)
	require.Equal(t, rationalAreaSign(unscaled), want)
	got, err := meshbool.PolyArea2Sign(proofbound.NewWorkBudget(t.Context()), poly)
	require.NoError(t, err)
	require.Equal(t, want, got)
	got, err = meshbool.PolyArea2Sign(proofbound.NewWorkBudget(t.Context()), unscaled)
	require.NoError(t, err)
	require.Equal(t, want, got)
	for stop := 1; stop <= len(poly)+1; stop++ {
		steps := 0
		stopErr := errors.New("stop")
		budget := &proofbound.WorkBudget{StepFn: func() error {
			steps++
			if steps == stop {
				return stopErr
			}
			return nil
		}, ErrFn: t.Context().Err}
		got, err := meshbool.PolyArea2Sign(budget, poly)
		if stop <= len(poly) {
			require.ErrorIs(t, err, stopErr)
			require.Equal(t, stop, steps)
			continue
		}
		require.NoError(t, err)
		require.Equal(t, want, got)
		require.Equal(t, len(poly), steps)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	budget := &proofbound.WorkBudget{StepFn: ctx.Err, ErrFn: ctx.Err}
	_, err = meshbool.PolyArea2Sign(budget, poly)
	require.ErrorIs(t, err, context.Canceled)
}

var areaSignSink int

func BenchmarkPolyArea2Sign(b *testing.B) {
	w := new(big.Int).Lsh(big.NewInt(1), 120)
	xpt := func(x, y int64) proof.Xpt {
		return proof.Xpt{X: big.NewInt(x), Y: big.NewInt(y), Z: big.NewInt(0), W: new(big.Int).Set(w)}
	}
	poly := []meshbool.Xp2{
		meshbool.NewXP2FromXpt(xpt(1, 2), 0, 1),
		meshbool.NewXP2FromXpt(xpt(19, 3), 0, 1),
		meshbool.NewXP2FromXpt(xpt(23, 17), 0, 1),
		meshbool.NewXP2FromXpt(xpt(2, 21), 0, 1),
	}
	b.ReportAllocs()
	for b.Loop() {
		v, err := meshbool.PolyArea2Sign(proofbound.NewWorkBudget(b.Context()), poly)
		if err != nil {
			b.Fatal(err)
		}
		areaSignSink = v
	}
}
