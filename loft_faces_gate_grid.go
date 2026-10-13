package decad

import (
	"context"
	"fmt"
	"math/big"
)

// lfGateNode holds one map's exact values at one tile-grid node for the
// three inverse rows: the denominator g, the per-axis derivative numerators
// row·(q_axis g − q g_axis) and the scalar row·F(node).
type lfGateNode struct {
	g      *big.Rat
	deriv  [3][2]*big.Rat
	scalar [3]*big.Rat
}

// lfGateGrid evaluates a cap map at every (tiles+1)² node once. Every tile
// corner is one of these nodes, and the per-row quantities are the same
// exact rationals lfDerivRange and lfScalarRange form corner by corner:
// row·(q_axis g − q g_axis) = g (row·q_axis) − g_axis (row·q) and
// row·(constant + q/g) = row·constant + (row·q)/g by distributivity.
func lfGateGrid(m loftFaceMap, rows [3]loftFaceRatVec, us, vs []*big.Rat) ([][]lfGateNode, error) {
	var rowQu, rowQv, rowConstant [3]*big.Rat
	for r, row := range rows {
		rowQu[r], rowQv[r], rowConstant[r] = lfVDot(row, m.qu), lfVDot(row, m.qv), lfVDot(row, m.constant)
	}
	grid := make([][]lfGateNode, len(us))
	for i, u := range us {
		grid[i] = make([]lfGateNode, len(vs))
		for j, v := range vs {
			g := m.denominator(u, v)
			if g.Sign() <= 0 {
				return nil, fmt.Errorf("%w: loft cap ray denominator is not positive", ErrUnsupported)
			}
			q := m.numerator(u, v)
			inverseG := lfDiv(lfOne(), g)
			node := lfGateNode{g: g}
			for r, row := range rows {
				rowQ := lfVDot(row, q)
				node.deriv[r][0] = lfSub(lfMul(g, rowQu[r]), lfMul(m.gu, rowQ))
				node.deriv[r][1] = lfSub(lfMul(g, rowQv[r]), lfMul(m.gv, rowQ))
				node.scalar[r] = lfAdd(rowConstant[r], lfMul(rowQ, inverseG))
			}
			grid[i][j] = node
		}
	}
	return grid, nil
}

func lfGateCornerRange(values [4]*big.Rat) loftFaceInterval {
	out := loftFaceInterval{values[0], values[0]}
	for _, v := range values[1:] {
		if v.Cmp(out.lo) < 0 {
			out.lo = v
		}
		if v.Cmp(out.hi) > 0 {
			out.hi = v
		}
	}
	return out
}

// lfGateDerivRange is lfDerivRange over four precomputed grid nodes.
func lfGateDerivRange(corners [4]*lfGateNode, row, axis int) loftFaceInterval {
	var nums, dens [4]*big.Rat
	for i, c := range corners {
		nums[i], dens[i] = c.deriv[row][axis], c.g
	}
	num, den := lfGateCornerRange(nums), lfGateCornerRange(dens)
	lo2, hi2 := lfMul(den.lo, den.lo), lfMul(den.hi, den.hi)
	// Every g is positive, so 0 < lo2 <= hi2. A negative numerator is
	// smallest over the smaller denominator and a non-negative one over the
	// larger; the maximum is the mirror.
	loDen, hiDen := hi2, hi2
	if num.lo.Sign() < 0 {
		loDen = lo2
	}
	if num.hi.Sign() > 0 {
		hiDen = lo2
	}
	return loftFaceInterval{lo: lfDiv(num.lo, loDen), hi: lfDiv(num.hi, hiDen)}
}

func lfGateScalarRange(corners [4]*lfGateNode, row int) loftFaceInterval {
	var values [4]*big.Rat
	for i, c := range corners {
		values[i] = c.scalar[row]
	}
	return lfGateCornerRange(values)
}

// lfBandGate proves injectivity over the entire source rectangle, not at
// sampled stations. Its exact rational interval boxes only tighten bounds;
// they do not change the one fixed matrix used for the whole band.
func lfBandGate(ctx context.Context, first, second loftFaceMap,
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
	// A tile's positive-denominator linear-fractional extrema occur at its
	// corners. Sixteen subdivisions limit cancellation loss in the two cap
	// terms of the band velocity while retaining one global injectivity M.
	const tiles = 16
	widthU := lfDiv(lfSub(box[1], box[0]), big.NewRat(tiles, 1))
	widthV := lfDiv(lfSub(box[3], box[2]), big.NewRat(tiles, 1))
	us, vs := make([]*big.Rat, tiles+1), make([]*big.Rat, tiles+1)
	for i := range us {
		us[i] = lfAdd(box[0], lfMul(big.NewRat(int64(i), 1), widthU))
		vs[i] = lfAdd(box[2], lfMul(big.NewRat(int64(i), 1), widthV))
	}
	if err := ctx.Err(); err != nil {
		return 0, 0, 0, err
	}
	firstGrid, err := lfGateGrid(first, rows, us, vs)
	if err != nil {
		return 0, 0, 0, err
	}
	secondGrid, err := lfGateGrid(second, rows, us, vs)
	if err != nil {
		return 0, 0, 0, err
	}
	worst := lfZero()
	for iu := range tiles {
		if err := ctx.Err(); err != nil {
			return 0, 0, 0, err
		}
		for iv := range tiles {
			// Corner order matches lfTileCorners: (u0,v0),(u0,v1),(u1,v0),(u1,v1).
			firstCorners := [4]*lfGateNode{&firstGrid[iu][iv], &firstGrid[iu][iv+1],
				&firstGrid[iu+1][iv], &firstGrid[iu+1][iv+1]}
			secondCorners := [4]*lfGateNode{&secondGrid[iu][iv], &secondGrid[iu][iv+1],
				&secondGrid[iu+1][iv], &secondGrid[iu+1][iv+1]}
			for row := range rows {
				deviation := lfZero()
				for axis := range 2 {
					hull := lfIntervalHull(lfGateDerivRange(firstCorners, row, axis),
						lfGateDerivRange(secondCorners, row, axis))
					deviation = lfAdd(deviation, lfIntervalAbs(hull, big.NewRat(int64(boolInt(row == axis)), 1)))
				}
				velocity := lfIntervalSub(lfGateScalarRange(secondCorners, row),
					lfGateScalarRange(firstCorners, row))
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
