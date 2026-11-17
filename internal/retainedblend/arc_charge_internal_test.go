package retainedblend

import (
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func bsq(x *big.Float) *big.Float { return new(big.Float).SetPrec(bigPrec).Mul(x, x) }

// bigCross is the crossing, with y > 0, of the circle of radius r1 about (x1, y1) and the
// circle of radius r2 about (x2, y2), in 200-bit arithmetic; ok is false when they do not
// cross.
func bigCross(x1, y1, r1, x2, y2, r2 *big.Float) (x, y *big.Float, ok bool) {
	dx := new(big.Float).SetPrec(bigPrec).Sub(x2, x1)
	dy := new(big.Float).SetPrec(bigPrec).Sub(y2, y1)
	d2 := new(big.Float).SetPrec(bigPrec).Add(bsq(dx), bsq(dy))
	d := bsqrt(d2)
	// a is the distance from circle 1's centre to the chord, h the half chord.
	a := new(big.Float).SetPrec(bigPrec).Sub(new(big.Float).Add(bsq(r1), d2), bsq(r2))
	a.Quo(a, new(big.Float).Mul(bf(2), d))
	h2 := new(big.Float).SetPrec(bigPrec).Sub(bsq(r1), bsq(a))
	if h2.Sign() < 0 {
		return nil, nil, false
	}
	h := bsqrt(h2)
	// The chord's foot, then the root on the side of positive y.
	ux, uy := new(big.Float).Quo(dx, d), new(big.Float).Quo(dy, d)
	fx := new(big.Float).SetPrec(bigPrec).Add(x1, new(big.Float).Mul(a, ux))
	fy := new(big.Float).SetPrec(bigPrec).Add(y1, new(big.Float).Mul(a, uy))
	px := new(big.Float).SetPrec(bigPrec).Add(fx, new(big.Float).Mul(h, new(big.Float).Neg(uy)))
	py := new(big.Float).SetPrec(bigPrec).Add(fy, new(big.Float).Mul(h, ux))
	qx := new(big.Float).SetPrec(bigPrec).Sub(fx, new(big.Float).Mul(h, new(big.Float).Neg(uy)))
	qy := new(big.Float).SetPrec(bigPrec).Sub(fy, new(big.Float).Mul(h, ux))
	if qy.Cmp(py) > 0 {
		px, py = qx, qy
	}
	return px, py, true
}

func TestAE(t *testing.T) {
	t.Parallel()
	// arg = (ρ² + d'² − r_G²)/(2ρd'); every sign of the moves of ρ (e), d' (2e) and r_G (e) must
	// move it by at most a_e.
	for _, c := range []struct{ rho, d, rG float64 }{
		{7.5, 9, 2}, {7.01, 9, 2}, {6.6, 5, 2}, {6.5, 7, 3}, {3, 2.5, 1.5},
	} {
		e := 1e-9
		a := aE(e, c.rho, c.rho, c.d, c.rG)
		require.Greater(t, a, 0.0)
		arg := func(rho, d, rG *big.Float) *big.Float {
			num := new(big.Float).SetPrec(bigPrec).Add(bsq(rho), bsq(d))
			num.Sub(num, bsq(rG))
			return num.Quo(num, new(big.Float).Mul(bf(2), new(big.Float).Mul(rho, d)))
		}
		base := arg(bf(c.rho), bf(c.d), bf(c.rG))
		worst := 0.0
		for _, sr := range []float64{-1, 0, 1} {
			for _, sd := range []float64{-1, 0, 1} {
				for _, sg := range []float64{-1, 0, 1} {
					moved := arg(bf(c.rho+sr*e), bf(c.d+sd*2*e), bf(c.rG+sg*e))
					diff, _ := new(big.Float).Sub(moved, base).Float64()
					worst = max(worst, math.Abs(diff))
				}
			}
		}
		require.LessOrEqual(t, worst, a, "%+v", c)
		require.Less(t, a, 40*worst+1e-30, "a_e should be near the true move, %+v", c)
	}
}

func TestSigmaCoversMovedCrossing(t *testing.T) {
	t.Parallel()
	const (
		R  = 7.0
		rG = 2.0
		d  = R + rG // external tangent: G* touches the wall's circle at T
	)
	e := 1e-9
	cv := &cylCurve{rG: rG, d: d}
	se, err := newSigmaEnd(e, d, rG, 7, 7.5)
	require.NoError(t, err)
	for _, rho := range []float64{7, 7.0005, 7.01, 7.1, 7.3, 7.5} {
		q := 0.0
		if rho > 7 {
			q = cv.rowQ(rho)
		}
		sigma := se.at(q)
		require.GreaterOrEqual(t, sigma, e)
		// The ideal crossing: the row's circle about O = (0, 0) with G*'s circle about (d, 0).
		x0, y0, ok := bigCross(bf(0), bf(0), bf(rho), bf(d), bf(0), bf(rG))
		if !ok {
			require.Equal(t, 7.0, rho, "only the wall's own row touches the boundary of G*'s reach")
			x0, y0 = bf(R), bf(0)
		}
		worst, crossed := 0.0, 0
		const dirs = 12
		// Each circle moves by e: the centre by t·e and the radius by (1 − t)·e, at every direction
		// and sign, and its true crossing stays within σ of the ideal one.
		for _, t1 := range []float64{0, 0.5, 1} {
			for k1 := range dirs {
				a1 := 2 * math.Pi * float64(k1) / dirs
				for _, t2 := range []float64{0, 0.5, 1} {
					for k2 := range dirs {
						a2 := 2 * math.Pi * float64(k2) / dirs
						for _, s1 := range []float64{-1, 1} {
							for _, s2 := range []float64{-1, 1} {
								ox, oy := t1*e*math.Cos(a1), t1*e*math.Sin(a1)
								cx, cy := d+t2*e*math.Cos(a2), t2*e*math.Sin(a2)
								x, y, ok := bigCross(bf(ox), bf(oy), bf(rho+s1*(1-t1)*e), bf(cx), bf(cy), bf(rG+s2*(1-t2)*e))
								if !ok {
									continue
								}
								crossed++
								dist := bsqrt(new(big.Float).SetPrec(bigPrec).Add(
									bsq(new(big.Float).Sub(x, x0)), bsq(new(big.Float).Sub(y, y0))))
								df, _ := dist.Float64()
								worst = max(worst, df)
							}
						}
					}
				}
			}
		}
		require.Positive(t, crossed, "rho %v", rho)
		require.LessOrEqual(t, worst, sigma, "rho %v: q %v, sigma %v", rho, q, sigma)
	}
}

func TestSigmaBranches(t *testing.T) {
	t.Parallel()
	e := 1e-12
	se, err := newSigmaEnd(e, 9, 2, 7, 7.5)
	require.NoError(t, err)
	wall := se.at(0)
	require.InDelta(t, se.sigma2+e, wall, 1e-18, "the wall's row takes σ₂")
	// A row well away from tangent takes the first-order σ₁, far below σ₂.
	first := se.at(0.5)
	require.Less(t, first, wall)
	require.InDelta(t, 2*e*(1+0x1p-10)/0.5+e, first, 1e-15)
	// A sine too small for σ₁'s remainder takes σ₂.
	require.InDelta(t, wall, se.at(1e-5), 1e-18)

	_, err = newSigmaEnd(1e-3, 9, 2, 7, 7.5)
	require.Error(t, err, "e above 2⁻²⁰·min(ρ_min, d', r_G) refuses")
}

func TestArcAreaCharge(t *testing.T) {
	t.Parallel()
	const w, h = 3.0, 2.0
	p := chargePiece{perimeter: 2 * (w + h), turning: 2 * math.Pi, area: w * h}

	t.Run("zero without σ and e", func(t *testing.T) {
		t.Parallel()
		require.Zero(t, arcAreaCharge([]chargePiece{p, {perimeter: 5, turning: 7, area: 3, rMin: 2}}, 0, 0))
	})

	t.Run("positive", func(t *testing.T) {
		t.Parallel()
		require.Positive(t, arcAreaCharge([]chargePiece{p}, 1e-6, 0))
		require.Positive(t, arcAreaCharge([]chargePiece{{area: 4, rMin: 2}}, 0, 1e-6))
	})

	t.Run("covers a rectangle whose sides all move by σ", func(t *testing.T) {
		t.Parallel()
		for _, sigma := range []float64{1e-9, 1e-4, 0.1} {
			moved := (w + 2*sigma) * (h + 2*sigma)
			require.GreaterOrEqual(t, arcAreaCharge([]chargePiece{p}, sigma, 0), moved-w*h)
		}
	})

	t.Run("covers a disc whose boundary moves by σ", func(t *testing.T) {
		t.Parallel()
		const r = 1.5
		disc := chargePiece{perimeter: 2 * math.Pi * r, turning: 2 * math.Pi, area: math.Pi * r * r}
		for _, sigma := range []float64{1e-9, 1e-4, 0.1} {
			for _, s := range []float64{-1, 1} {
				moved := math.Pi * (r + s*sigma) * (r + s*sigma)
				require.GreaterOrEqual(t, arcAreaCharge([]chargePiece{disc}, sigma, 0), math.Abs(moved-disc.area))
			}
		}
	})

	t.Run("covers a wall band on a cylinder whose radius and edges move", func(t *testing.T) {
		t.Parallel()
		const radius, span, tb = 7.0, 1.1, 0.5
		band := chargePiece{perimeter: 2*radius*span + 2*tb, turning: 2 * math.Pi, area: radius * span * tb, rMin: radius}
		e, sigma := 1e-6, 3e-6
		// The wall's radius grows by e and the band's height by σ.
		moved := (radius + e) * span * (tb + sigma)
		require.GreaterOrEqual(t, arcAreaCharge([]chargePiece{band}, sigma, e), moved-radius*span*tb)
		// Without the curvature term the radius change would not be covered.
		require.Less(t, arcAreaCharge([]chargePiece{{perimeter: band.perimeter, turning: band.turning}}, sigma, e)+0,
			arcAreaCharge([]chargePiece{band}, sigma, e))
	})
}
