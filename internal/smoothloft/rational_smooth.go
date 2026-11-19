package smoothloft

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// rationalSmoothPatches gives each smooth join common exact derivatives in
// every section before blending. A corner internal to one source face refuses:
// grouping it as one smooth wall would hide the edge.
func rationalSmoothPatches(ctx context.Context, patches [][]rationalTransition, edgeOf []int,
	firstFit int, preserveParameter bool) ([][]rationalTransition, error) {
	return rationalSmoothPatchesMode(ctx, patches, edgeOf, firstFit, preserveParameter, nil)
}

func rationalSmoothPatchesMode(ctx context.Context, patches [][]rationalTransition, edgeOf []int,
	firstFit int, preserveParameter bool, emerging []bool) ([][]rationalTransition, error) {
	n, levels := len(patches), len(patches[0])+1
	curves := make([][]rationalCurve, levels)
	start, end := make([][]*big.Rat, levels), make([][]*big.Rat, levels)
	for i := range curves {
		curves[i], start[i], end[i] = make([]rationalCurve, n), make([]*big.Rat, n), make([]*big.Rat, n)
		for k := range n {
			curve := patches[k][0].a
			if i > 0 {
				curve = patches[k][i-1].b
			}
			if i >= firstFit {
				if len(polynomial.RpRem(curve.x, curve.w)) != 0 || len(polynomial.RpRem(curve.y, curve.w)) != 0 {
					return nil, twistUnsupported("circular fitted loft spline denominator does not cancel exactly")
				}
				curve = rationalCurve{x: polynomial.RpQuo(curve.x, curve.w), y: polynomial.RpQuo(curve.y, curve.w),
					w: polynomial.RatPoly{rint(1)}}
			}
			curves[i][k], start[i][k], end[i][k] = curve, rint(1), rint(1)
		}
	}
	for k := range n {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		next := (k + 1) % n
		if len(emerging) != 0 && (emerging[k] || emerging[next]) {
			continue
		}
		ls, rs := make([]*big.Rat, levels), make([]*big.Rat, levels)
		smooth := true
		for i := range levels {
			var ok bool
			ls[i], rs[i], ok = rationalJoinSlopes(curves[i][k], curves[i][next])
			if !ok {
				smooth = false
				break
			}
		}
		if !smooth {
			if edgeOf[k] == edgeOf[next] {
				return nil, twistUnsupported("circular fitted loft has a corner inside source face %d", edgeOf[k])
			}
			continue
		}
		if preserveParameter {
			for i := 1; i < levels; i++ {
				if mul(ls[i], rs[0]).Cmp(mul(rs[i], ls[0])) != 0 {
					return nil, twistUnsupported("circular fitted loft smooth join %d has unequal parameter-speed ratios", k)
				}
			}
			continue
		}
		for i := range levels {
			end[i][k], start[i][next] = ls[i], rs[i]
		}
	}
	for i := range levels {
		for k := range n {
			curves[i][k] = rationalReparameterize(curves[i][k], start[i][k], end[i][k])
		}
	}
	for k := range n {
		for i := range levels - 1 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			a, b := curves[i][k], curves[i+1][k]
			w := polynomial.RpMul(a.w, b.w)
			commonA := rationalCurve{x: polynomial.RpMul(a.x, b.w), y: polynomial.RpMul(a.y, b.w), w: w}
			commonB := rationalCurve{x: polynomial.RpMul(b.x, a.w), y: polynomial.RpMul(b.y, a.w), w: w}
			patches[k][i] = rationalTransition{a: commonA, b: commonB, linear: patches[k][i].linear}
			if len(emerging) != 0 && emerging[k] && i < firstFit {
				if i == firstFit-1 {
					p := patches[k][i].at(rint(0), rint(0))
					f0 := patches[k][i].at(rint(0), rint(1))
					f1 := patches[k][i].at(rint(1), rint(1))
					if p.U.Cmp(f0.U) != 0 || p.U.Cmp(f1.U) != 0 ||
						mul(p.U, sub(f1.V, f0.V)).Sign() <= 0 {
						return nil, twistUnsupported("emerging flat lacks a positive exact polar determinant")
					}
				}
				continue
			}
			if err := rationalPolarPositive(ctx, rationalPolarRows(commonA, commonB), proofbound.NewWorkBudget(ctx)); err != nil {
				return nil, fmt.Errorf("circular fitted loft span %d band %d: %w", k, i, err)
			}
		}
	}
	return patches, nil
}
