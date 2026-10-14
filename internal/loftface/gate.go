package loftface

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// gateNode holds one map's exact values at one tile-grid node for the
// three inverse rows: the denominator g, the per-axis derivative numerators
// row·(q_axis g − q g_axis) and the scalar row·F(node). Nodes on one grid
// line share their derivative rationals; every reader only reads them.
type gateNode struct {
	g      *big.Rat
	deriv  [3][2]*big.Rat
	scalar [3]*big.Rat
}

// gateGrid evaluates a cap map at every (tiles+1)² node once. Every tile
// corner is one of these nodes, and the per-row quantities are the same
// exact rationals that the corner-by-corner reference in the tests and
// ScalarRange form. With g = G0 + GU u + GV v and
// row·q = row·Q0 + (row·QU) u + (row·QV) v, the derivative numerators
// row·(q_axis g − q g_axis) = g (row·q_axis) − g_axis (row·q) reduce to
//
//	(G0 rowQU − GU rowQ0) + v (GV rowQU − GU rowQV)  for the u axis, and
//	(G0 rowQV − GV rowQ0) + u (GU rowQV − GV rowQU)  for the v axis,
//
// so a node's derivatives depend on one coordinate each, and each is
// computed once per grid line. The scalar row·(constant + q/g) is
// row·constant + (row·q)/g by distributivity.
func gateGrid(m Map, rows [3]RatVec, us, vs []*big.Rat) ([][]gateNode, error) {
	var rowQu, rowQv, rowConstant, rowQ0 [3]*big.Rat
	var byV, byU [3][]*big.Rat // deriv[r][0] by v index, deriv[r][1] by u index
	for r, row := range rows {
		rowQu[r], rowQv[r], rowConstant[r] = VDot(row, m.QU), VDot(row, m.QV), VDot(row, m.Constant)
		rowQ0[r] = VDot(row, m.Q0)
		a0 := ratSub(ratMul(m.G0, rowQu[r]), ratMul(m.GU, rowQ0[r]))
		b0 := ratSub(ratMul(m.GV, rowQu[r]), ratMul(m.GU, rowQv[r]))
		a1 := ratSub(ratMul(m.G0, rowQv[r]), ratMul(m.GV, rowQ0[r]))
		b1 := ratSub(ratMul(m.GU, rowQv[r]), ratMul(m.GV, rowQu[r]))
		byV[r] = make([]*big.Rat, len(vs))
		for j, v := range vs {
			byV[r][j] = ratAdd(a0, ratMul(b0, v))
		}
		byU[r] = make([]*big.Rat, len(us))
		for i, u := range us {
			byU[r][i] = ratAdd(a1, ratMul(b1, u))
		}
	}
	grid := make([][]gateNode, len(us))
	for i, u := range us {
		grid[i] = make([]gateNode, len(vs))
		for j, v := range vs {
			g := m.Denominator(u, v)
			if g.Sign() <= 0 {
				return nil, fmt.Errorf("%w: loft cap ray denominator is not positive", decaderr.ErrUnsupported)
			}
			inverseG := ratDiv(ratOne(), g)
			node := gateNode{g: g}
			for r := range rows {
				rowQ := ratAdd(ratAdd(rowQ0[r], ratMul(rowQu[r], u)), ratMul(rowQv[r], v))
				node.deriv[r][0] = byV[r][j]
				node.deriv[r][1] = byU[r][i]
				node.scalar[r] = ratAdd(rowConstant[r], ratMul(rowQ, inverseG))
			}
			grid[i][j] = node
		}
	}
	return grid, nil
}

func gateCornerRange(values [4]*big.Rat) Interval {
	out := Interval{values[0], values[0]}
	for _, v := range values[1:] {
		if v.Cmp(out.Lo) < 0 {
			out.Lo = v
		}
		if v.Cmp(out.Hi) > 0 {
			out.Hi = v
		}
	}
	return out
}

// gateDerivRange is the corner-by-corner reference derivative range over
// four precomputed grid nodes.
func gateDerivRange(corners [4]*gateNode, row, axis int) Interval {
	var nums, dens [4]*big.Rat
	for i, c := range corners {
		nums[i], dens[i] = c.deriv[row][axis], c.g
	}
	num, den := gateCornerRange(nums), gateCornerRange(dens)
	lo2, hi2 := ratMul(den.Lo, den.Lo), ratMul(den.Hi, den.Hi)
	// Every g is positive, so 0 < lo2 <= hi2. A negative numerator is
	// smallest over the smaller denominator and a non-negative one over the
	// larger; the maximum is the mirror.
	loDen, hiDen := hi2, hi2
	if num.Lo.Sign() < 0 {
		loDen = lo2
	}
	if num.Hi.Sign() > 0 {
		hiDen = lo2
	}
	return Interval{Lo: ratDiv(num.Lo, loDen), Hi: ratDiv(num.Hi, hiDen)}
}

func gateScalarRange(corners [4]*gateNode, row int) Interval {
	var values [4]*big.Rat
	for i, c := range corners {
		values[i] = c.scalar[row]
	}
	return gateCornerRange(values)
}

// BandGate proves injectivity over the entire source rectangle, not at
// sampled stations. Its exact rational interval boxes only tighten bounds;
// they do not change the one fixed matrix used for the whole band.
func BandGate(ctx context.Context, first, second Map,
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
	// A tile's positive-denominator linear-fractional extrema occur at its
	// corners. Sixteen subdivisions limit cancellation loss in the two cap
	// terms of the band velocity while retaining one global injectivity M.
	const tiles = 16
	widthU := ratDiv(ratSub(box[1], box[0]), big.NewRat(tiles, 1))
	widthV := ratDiv(ratSub(box[3], box[2]), big.NewRat(tiles, 1))
	us, vs := make([]*big.Rat, tiles+1), make([]*big.Rat, tiles+1)
	for i := range us {
		us[i] = ratAdd(box[0], ratMul(big.NewRat(int64(i), 1), widthU))
		vs[i] = ratAdd(box[2], ratMul(big.NewRat(int64(i), 1), widthV))
	}
	if err := ctx.Err(); err != nil {
		return 0, 0, 0, err
	}
	firstGrid, err := gateGrid(first, rows, us, vs)
	if err != nil {
		return 0, 0, 0, err
	}
	secondGrid, err := gateGrid(second, rows, us, vs)
	if err != nil {
		return 0, 0, 0, err
	}
	worst := ratZero()
	for iu := range tiles {
		if err := ctx.Err(); err != nil {
			return 0, 0, 0, err
		}
		for iv := range tiles {
			// Corner order matches TileCorners: (u0,v0),(u0,v1),(u1,v0),(u1,v1).
			firstCorners := [4]*gateNode{&firstGrid[iu][iv], &firstGrid[iu][iv+1],
				&firstGrid[iu+1][iv], &firstGrid[iu+1][iv+1]}
			secondCorners := [4]*gateNode{&secondGrid[iu][iv], &secondGrid[iu][iv+1],
				&secondGrid[iu+1][iv], &secondGrid[iu+1][iv+1]}
			for row := range rows {
				deviation := ratZero()
				for axis := range 2 {
					hull := IntervalHull(gateDerivRange(firstCorners, row, axis),
						gateDerivRange(secondCorners, row, axis))
					deviation = ratAdd(deviation, IntervalAbs(hull, big.NewRat(int64(boolInt(row == axis)), 1)))
				}
				velocity := IntervalSub(gateScalarRange(secondCorners, row),
					gateScalarRange(firstCorners, row))
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

func boolInt(ok bool) int {
	if ok {
		return 1
	}
	return 0
}
