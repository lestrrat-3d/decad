package smoothloft

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestRationalPolarCircleToDiamond(t *testing.T) {
	w := polynomial.RatPoly{rint(1), rint(0), rint(1)}
	a := rationalCurve{x: polynomial.RatPoly{rint(1), rint(0), rint(-1)},
		y: polynomial.RatPoly{rint(0), rint(2)}, w: w}
	b := rationalCurve{x: polynomial.RpMul(w, polynomial.RatPoly{rint(1), rint(-1)}),
		y: polynomial.RpMul(w, polynomial.RatPoly{rint(0), rint(1)}), w: w}
	transition := rationalTransition{a: a, b: b}
	bounds, err := transition.bounds()
	require.NoError(t, err)
	require.Positive(t, bounds.bu.Sign())
	require.Positive(t, bounds.mut.Sign())
	start := transition.at(big.NewRat(1, 2), rint(0))
	end := transition.at(big.NewRat(1, 2), rint(1))
	require.Zero(t, start.U.Cmp(big.NewRat(3, 5)))
	require.Zero(t, start.V.Cmp(big.NewRat(4, 5)))
	require.Zero(t, end.U.Cmp(big.NewRat(1, 2)))
	require.Zero(t, end.V.Cmp(big.NewRat(1, 2)))
	interior := transition.at(big.NewRat(1, 2), big.NewRat(1, 4))
	require.Zero(t, interior.U.Cmp(big.NewRat(187, 320)))
	require.Zero(t, interior.V.Cmp(big.NewRat(241, 320)))
	linear := transition
	linear.linear = true
	linearInterior := linear.at(big.NewRat(1, 2), big.NewRat(1, 4))
	require.Zero(t, linearInterior.U.Cmp(big.NewRat(23, 40)))
	require.Zero(t, linearInterior.V.Cmp(big.NewRat(29, 40)))
	linearBounds, err := linear.bounds()
	require.NoError(t, err)
	require.Zero(t, linearBounds.mtt.Sign())
	rows := rationalPolarRows(a, b)
	require.NoError(t, rationalPolarPositive(t.Context(), rows, proofbound.NewWorkBudget(t.Context())))
	// The row evaluation is the exact cross product of the blended numerator
	// and its derivative, including a non-endpoint blend and station.
	u, s := big.NewRat(1, 3), big.NewRat(2, 5)
	x := polynomial.RpAdd(polynomial.RpScale(a.x, sub(rint(1), s)), polynomial.RpScale(b.x, s))
	y := polynomial.RpAdd(polynomial.RpScale(a.y, sub(rint(1), s)), polynomial.RpScale(b.y, s))
	want := polynomial.RpEval(rationalCross(x, y, polynomial.RpDeriv(x), polynomial.RpDeriv(y)), u)
	sRow := []*big.Rat{polynomial.BernsteinValue(rows[0], u), polynomial.BernsteinValue(rows[1], u),
		polynomial.BernsteinValue(rows[2], u)}
	require.Zero(t, want.Cmp(polynomial.BernsteinValue(sRow, s)))

	// Reversing the target walk makes its endpoint polar derivative negative.
	b.x, b.y = b.y, b.x
	err = rationalPolarPositive(t.Context(), rationalPolarRows(a, b), proofbound.NewWorkBudget(t.Context()))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.ErrorIs(t, rationalPolarPositive(ctx, rows, proofbound.NewWorkBudget(ctx)), context.Canceled)
}

func TestRationalCurveBoundsQuarterCircle(t *testing.T) {
	c := rationalCurve{x: polynomial.RatPoly{rint(1), rint(0), rint(-1)},
		y: polynomial.RatPoly{rint(0), rint(2)}, w: polynomial.RatPoly{rint(1), rint(0), rint(1)}}
	radius, speed, curvature := rationalCurveBounds(c, rint(1))
	// Derivatives of ((1-u²)/(1+u²), 2u/(1+u²)), evaluated exactly.
	for i := range int64(21) {
		u := big.NewRat(i, 20)
		u2 := mul(u, u)
		w := add(rint(1), u2)
		x, y := div(sub(rint(1), u2), w), div(mul(rint(2), u), w)
		dx := div(mul(rint(-4), u), mul(w, w))
		dy := div(mul(rint(2), sub(rint(1), u2)), mul(w, w))
		ddx := div(mul(rint(4), sub(mul(rint(3), u2), rint(1))), mul(mul(w, w), w))
		ddy := div(mul(mul(rint(4), u), sub(u2, rint(3))), mul(mul(w, w), w))
		require.LessOrEqual(t, add(abs(x), abs(y)).Cmp(radius), 0)
		require.LessOrEqual(t, add(abs(dx), abs(dy)).Cmp(speed), 0)
		require.LessOrEqual(t, add(abs(ddx), abs(ddy)).Cmp(curvature), 0)
	}
}
