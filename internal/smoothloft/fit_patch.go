package smoothloft

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// fitPatch is cubic in the fitted flank's shared span parameter and quadratic
// in section height. Every control coordinate is an exact rational over the
// three Sketch-exported natural-cubic interpolants.
type fitPatch [4][3]freeform.RatPoint

func fitPoint(u, v *big.Rat) freeform.RatPoint {
	return freeform.RatPoint{U: u, V: v}
}

func fitPointAdd(a, b freeform.RatPoint) freeform.RatPoint {
	return fitPoint(add(a.U, b.U), add(a.V, b.V))
}

func fitPointSub(a, b freeform.RatPoint) freeform.RatPoint {
	return fitPoint(sub(a.U, b.U), sub(a.V, b.V))
}

func fitPointScale(a freeform.RatPoint, q *big.Rat) freeform.RatPoint {
	return fitPoint(mul(a.U, q), mul(a.V, q))
}

func fitPointL1(a freeform.RatPoint) *big.Rat {
	return add(abs(a.U), abs(a.V))
}

func fitPatchOf(spans [3]freeform.BezierSpan) (fitPatch, error) {
	var out fitPatch
	for _, span := range spans {
		if len(span) != 4 {
			return out, fmt.Errorf("%w: a fitted flank needs cubic Bézier spans", decaderr.ErrUnsupported)
		}
	}
	for i := range out {
		out[i][0] = spans[0][i]
		out[i][2] = spans[2][i]
		mid := fitPointScale(spans[1][i], rint(2))
		endMean := fitPointScale(fitPointAdd(spans[0][i], spans[2][i]), big.NewRat(1, 2))
		out[i][1] = fitPointSub(mid, endMean)
	}
	return out, nil
}

func fitPatchSplitU(p fitPatch) (fitPatch, fitPatch) {
	var left, right fitPatch
	for j := range p[0] {
		u := make([]*big.Rat, 4)
		v := make([]*big.Rat, 4)
		for i := range p {
			u[i], v[i] = p[i][j].U, p[i][j].V
		}
		lu, ru := freeform.BernsteinSplit(u, big.NewRat(1, 2))
		lv, rv := freeform.BernsteinSplit(v, big.NewRat(1, 2))
		for i := range p {
			left[i][j] = fitPoint(lu[i], lv[i])
			right[i][j] = fitPoint(ru[i], rv[i])
		}
	}
	return left, right
}

func fitPatchSplitT(p fitPatch) (fitPatch, fitPatch) {
	var left, right fitPatch
	for i := range p {
		lu, ru := freeform.BernsteinSplit([]*big.Rat{p[i][0].U, p[i][1].U, p[i][2].U}, big.NewRat(1, 2))
		lv, rv := freeform.BernsteinSplit([]*big.Rat{p[i][0].V, p[i][1].V, p[i][2].V}, big.NewRat(1, 2))
		for j := range p[i] {
			left[i][j] = fitPoint(lu[j], lv[j])
			right[i][j] = fitPoint(ru[j], rv[j])
		}
	}
	return left, right
}

func fitPatchAtT(p fitPatch, t *big.Rat, end int) freeform.RatPoint {
	oneMinus := sub(rint(1), t)
	w0 := mul(oneMinus, oneMinus)
	w1 := mul(rint(2), mul(t, oneMinus))
	w2 := mul(t, t)
	return fitPointAdd(
		fitPointAdd(fitPointScale(p[end][0], w0), fitPointScale(p[end][1], w1)),
		fitPointScale(p[end][2], w2),
	)
}

// fitPatchPolarCoefficients gives cross(C, dC/du) in Bernstein form. Its
// coefficients are exact; positive coefficients prove positive polar-angle
// derivative throughout the parameter rectangle.
func fitPatchPolarCoefficients(p fitPatch) [6][5]*big.Rat {
	choose := [6][]int{
		{1}, {1, 1}, {1, 2, 1}, {1, 3, 3, 1},
		{1, 4, 6, 4, 1}, {1, 5, 10, 10, 5, 1},
	}
	var coeff [6][5]*big.Rat
	for k := range coeff {
		for l := range coeff[k] {
			coeff[k][l] = new(big.Rat)
		}
	}
	for i := range p {
		for m := range 3 {
			ku := i + m
			wu := big.NewRat(int64(choose[3][i]*choose[2][m]), int64(choose[5][ku]))
			for j := range p[i] {
				for n := range p[m] {
					kt := j + n
					wt := big.NewRat(int64(choose[2][j]*choose[2][n]), int64(choose[4][kt]))
					dx := fitPointScale(fitPointSub(p[m+1][n], p[m][n]), rint(3))
					cross := sub(mul(p[i][j].U, dx.V), mul(p[i][j].V, dx.U))
					coeff[ku][kt].Add(coeff[ku][kt], mul(cross, mul(wu, wt)))
				}
			}
		}
	}
	return coeff
}

func fitPatchPolarPositive(ctx context.Context, budget *proofbound.WorkBudget, p fitPatch) error {
	const maxDepth = 12
	var check func(fitPatch, int) error
	check = func(cell fitPatch, depth int) error {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := budget.Step(); err != nil {
			return err
		}
		coeff := fitPatchPolarCoefficients(cell)
		positive := true
		for _, row := range coeff {
			for _, c := range row {
				if c.Sign() <= 0 {
					positive = false
					break
				}
			}
		}
		if positive {
			return nil
		}
		if depth == maxDepth {
			return fmt.Errorf("%w: fitted loft has no positive polar-angle proof", decaderr.ErrUnsupported)
		}
		var left, right fitPatch
		if depth%2 == 0 {
			left, right = fitPatchSplitU(cell)
		} else {
			left, right = fitPatchSplitT(cell)
		}
		if err := check(left, depth+1); err != nil {
			return err
		}
		return check(right, depth+1)
	}
	return check(p, 0)
}

func fitPatchUErr(p fitPatch) *big.Rat {
	worst := new(big.Rat)
	for j := range p[0] {
		for i := 1; i < 3; i++ {
			linear := fitPointAdd(
				fitPointScale(p[0][j], big.NewRat(int64(3-i), 3)),
				fitPointScale(p[3][j], big.NewRat(int64(i), 3)),
			)
			worst = maxRat(worst, fitPointL1(fitPointSub(p[i][j], linear)))
		}
	}
	return worst
}

func fitPatchZErr(p fitPatch, bands int) *big.Rat {
	worst := new(big.Rat)
	for i := range p {
		a := fitPointAdd(fitPointSub(p[i][0], fitPointScale(p[i][1], rint(2))), p[i][2])
		worst = maxRat(worst, fitPointL1(a))
	}
	return div(worst, rint(int64(4*bands*bands)))
}
