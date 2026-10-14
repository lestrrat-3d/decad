package loftface

import (
	"cmp"
	"context"
	"math"
	"math/big"
	"slices"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// BandGate's norm is the exact maximum, over every tile and row, of
// gateTileDeviation. Most tile rows cannot reach that maximum, and the exact
// deviation of each costs several rational divisions. This file bounds each
// tile row's deviation from above in floats, evaluates the exact deviations in
// descending bound order, and stops when no remaining bound can exceed the
// exact maximum found so far.
//
// The float pass only ever skips a tile row. It never produces a norm, a
// refusal or a Jacobian bound, and it never admits anything: a bound that is
// too small would be the only way for it to change a result, so every step
// below is rounded outward, and any non-finite or out-of-range value makes the
// bound +Inf, which is never skipped.
//
// Rounding. A float operation on finite operands rounds to nearest, so the
// exact result lies within half an ulp of it, and one NextUp/NextDown step
// moves past the exact result. The wrappers gateUp and gateDown are that
// step. A product is always written float64(a*b) and is rounded and stepped
// before anything is added to it, so no multiply feeds an add directly and the
// compiler has no fused multiply-add (arm64) to form. Fused or not, every
// operation here is one correctly rounded operation followed by a step, which
// is the same enclosure.
//
// Range. Node enclosures hold only zero or magnitudes within [2^-256, 2^256]
// (gateEncOf), so every square, quotient and sum below stays normal and finite
// and no step is taken in the subnormal or overflow range. A value outside the
// range makes its node not ok and every tile bound that reads it +Inf.

const (
	gateEncMin = 0x1p-256
	gateEncMax = 0x1p256
)

func gateUp(x float64) float64   { return proofarith.NextUp(x) }
func gateDown(x float64) float64 { return proofarith.NextDown(x) }

// gateEnc encloses one exact rational in floats: lo <= x <= hi. A NaN end
// marks a value that could not be enclosed.
type gateEnc struct{ lo, hi float64 }

var gateEncBad = gateEnc{math.NaN(), math.NaN()}

func (e gateEnc) valid() bool { return !math.IsNaN(e.lo) && !math.IsNaN(e.hi) }

func gateEncInRange(x float64) bool {
	if x == 0 {
		return true
	}
	a := math.Abs(x)
	return a >= gateEncMin && a <= gateEncMax
}

// gateEncOf is x.Float64() widened by one step each way unless exact. Float64
// rounds to nearest, so an inexact x lies strictly between the result's
// neighbours. ok is false when either end is not zero or within
// [2^-256, 2^256]. Float64 reports a nonzero x that underflows to zero as
// exact, so a zero result for a nonzero x is refused rather than trusted; an
// overflow to an infinity and a subnormal result land outside the range.
func gateEncOf(x *big.Rat) (gateEnc, bool) {
	f, exact := x.Float64()
	if f == 0 && x.Sign() != 0 {
		return gateEncBad, false
	}
	e := gateEnc{f, f}
	if !exact {
		e = gateEnc{gateDown(f), gateUp(f)}
	}
	if !gateEncInRange(e.lo) || !gateEncInRange(e.hi) {
		return gateEncBad, false
	}
	return e, true
}

// gateNodeEnc holds the enclosures of one node's g, derivatives and scalars,
// and the exact signs of its derivatives. ok is true when every enclosure is
// valid and g's lower end is positive.
type gateNodeEnc struct {
	ok     bool
	g      gateEnc
	deriv  [3][2]gateEnc
	sign   [3][2]int
	scalar [3]gateEnc
}

// gateGridEnc encloses every node of grid. Nodes on one grid line share their
// derivative rationals (gateGrid), so a derivative equal by pointer to the
// previous node's on its line reuses that node's enclosure.
func gateGridEnc(grid [][]gateNode) [][]gateNodeEnc {
	out := make([][]gateNodeEnc, len(grid))
	for i := range grid {
		out[i] = make([]gateNodeEnc, len(grid[i]))
		for j := range grid[i] {
			n, e := &grid[i][j], &out[i][j]
			e.g, _ = gateEncOf(n.g)
			for r := range 3 {
				for a := range 2 {
					switch {
					case a == 0 && i > 0 && grid[i-1][j].deriv[r][0] == n.deriv[r][0]:
						e.deriv[r][0] = out[i-1][j].deriv[r][0]
					case a == 1 && j > 0 && grid[i][j-1].deriv[r][1] == n.deriv[r][1]:
						e.deriv[r][1] = out[i][j-1].deriv[r][1]
					default:
						e.deriv[r][a], _ = gateEncOf(n.deriv[r][a])
					}
					e.sign[r][a] = n.deriv[r][a].Sign()
				}
				e.scalar[r], _ = gateEncOf(n.scalar[r])
			}
			e.ok = e.g.valid() && e.g.lo > 0
			for r := range 3 {
				e.ok = e.ok && e.scalar[r].valid() && e.deriv[r][0].valid() && e.deriv[r][1].valid()
			}
		}
	}
	return out
}

// gateDerivBound returns floats lo <= Lo and Hi <= hi, where [Lo, Hi] is
// gateDerivRange(corners, row, axis) of the exact nodes the four enclosures
// belong to. ok is false when no bound is formed.
//
// Proof. Each corner's exact g is in its enclosure and its exact derivative
// numerator in its enclosure, so the exact minimum and maximum of each lie
// between the minimum and maximum of the corresponding ends. Every g is
// positive, so squaring is increasing and lo2 = (min g)^2, hi2 = (max g)^2
// lie in the squared, outward-rounded enclosures. gateDerivRange picks loDen
// and hiDen from the exact signs of the numerators, and so do these bounds
// (sign holds Sign() of the exact rationals). For a numerator n >= a and a
// denominator d in [dl, dh] with dl > 0, n/d >= a/d, which is at least a/dh
// when a >= 0 and a/dl when a < 0; for n <= b, n/d <= b/d, at most b/dl when
// b >= 0 and b/dh when b < 0. The quotients are rounded outward.
func gateDerivBound(c [4]*gateNodeEnc, row, axis int) (float64, float64, bool) {
	gmin, gmax := c[0].g, c[0].g
	nmin, nmax := c[0].deriv[row][axis], c[0].deriv[row][axis]
	neg, pos := c[0].sign[row][axis] < 0, c[0].sign[row][axis] > 0
	for _, k := range c[1:] {
		g, n := k.g, k.deriv[row][axis]
		gmin = gateEnc{min(gmin.lo, g.lo), min(gmin.hi, g.hi)}
		gmax = gateEnc{max(gmax.lo, g.lo), max(gmax.hi, g.hi)}
		nmin = gateEnc{min(nmin.lo, n.lo), min(nmin.hi, n.hi)}
		nmax = gateEnc{max(nmax.lo, n.lo), max(nmax.hi, n.hi)}
		neg = neg || k.sign[row][axis] < 0
		pos = pos || k.sign[row][axis] > 0
	}
	if !(gmin.lo > 0) {
		return 0, 0, false
	}
	lo2 := gateEnc{gateDown(float64(gmin.lo * gmin.lo)), gateUp(float64(gmin.hi * gmin.hi))}
	hi2 := gateEnc{gateDown(float64(gmax.lo * gmax.lo)), gateUp(float64(gmax.hi * gmax.hi))}
	loDen, hiDen := hi2, hi2
	if neg {
		loDen = lo2
	}
	if pos {
		hiDen = lo2
	}
	if !(loDen.lo > 0) || !(hiDen.lo > 0) {
		return 0, 0, false
	}
	var lo, hi float64
	if nmin.lo >= 0 {
		lo = gateDown(float64(nmin.lo / loDen.hi))
	} else {
		lo = gateDown(float64(nmin.lo / loDen.lo))
	}
	if nmax.hi >= 0 {
		hi = gateUp(float64(nmax.hi / hiDen.lo))
	} else {
		hi = gateUp(float64(nmax.hi / hiDen.hi))
	}
	if math.IsNaN(lo) || math.IsNaN(hi) {
		return 0, 0, false
	}
	return lo, hi, true
}

// gateAbsBound is a float upper bound of IntervalAbs over any exact interval
// inside [lo, hi] with center c (0 or 1): |x - c| <= max(hi - c, c - lo) for
// every x in the interval, and each difference is rounded up.
func gateAbsBound(lo, hi, c float64) float64 {
	return max(gateUp(hi-c), gateUp(c-lo), 0)
}

// gateVelocityBound returns floats lo <= Lo and Hi <= hi, where [Lo, Hi] is
// the exact interval IntervalSub(gateScalarRange(second), gateScalarRange(first))
// of a tile row. The scalar ranges' ends are enclosed by the minimum and
// maximum of the corner enclosures' ends; the difference's low end is
// s.Lo - f.Hi >= sLo - fHi and its high end s.Hi - f.Lo <= sHi - fLo, each
// rounded outward.
func gateVelocityBound(first, second [4]*gateNodeEnc, row int) (float64, float64) {
	sLo, sHi := second[0].scalar[row].lo, second[0].scalar[row].hi
	fLo, fHi := first[0].scalar[row].lo, first[0].scalar[row].hi
	for k := 1; k < 4; k++ {
		sLo, sHi = min(sLo, second[k].scalar[row].lo), max(sHi, second[k].scalar[row].hi)
		fLo, fHi = min(fLo, first[k].scalar[row].lo), max(fHi, first[k].scalar[row].hi)
	}
	return gateDown(sLo - fHi), gateUp(sHi - fLo)
}

// gateTileTerms returns float upper bounds of the three terms of
// gateTileDeviation(first, second, row): the two derivative axes' distances
// from the identity column and the velocity's distance from the third column.
// ok is false when a corner is not ok or a derivative bound is not formed.
//
// Proof. Each term is IntervalAbs of an exact interval about 0 or 1. For the
// axes the interval is the hull of the two maps' derivative ranges; the hull's
// ends are the minimum of the lower ends and the maximum of the upper ends, so
// the minimum of the two lower bounds and the maximum of the two upper bounds
// from gateDerivBound enclose them. The velocity interval comes from
// gateVelocityBound. gateAbsBound bounds each term from its interval's float
// bounds.
func gateTileTerms(first, second [4]*gateNodeEnc, row int) ([3]float64, bool) {
	var terms [3]float64
	for k := range first {
		if !first[k].ok || !second[k].ok {
			return terms, false
		}
	}
	for axis := range 2 {
		fLo, fHi, ok := gateDerivBound(first, row, axis)
		if !ok {
			return terms, false
		}
		sLo, sHi, ok := gateDerivBound(second, row, axis)
		if !ok {
			return terms, false
		}
		terms[axis] = gateAbsBound(min(fLo, sLo), max(fHi, sHi), float64(boolInt(row == axis)))
	}
	vLo, vHi := gateVelocityBound(first, second, row)
	terms[2] = gateAbsBound(vLo, vHi, float64(boolInt(row == 2)))
	return terms, true
}

// gateTileUpper is a float upper bound of gateTileDeviation(first, second,
// row), or +Inf when a corner is not ok or any step is not finite. The
// deviation is the exact sum of the three terms in gateTileTerms in the same
// order, so the two sums rounded up bound it: addition is increasing in each
// non-negative term and each rounded sum is stepped up.
func gateTileUpper(first, second [4]*gateNodeEnc, row int) float64 {
	terms, ok := gateTileTerms(first, second, row)
	if !ok {
		return math.Inf(1)
	}
	upper := gateUp(gateUp(terms[0]+terms[1]) + terms[2])
	if math.IsNaN(upper) {
		return math.Inf(1)
	}
	return upper
}

// gateTileDeviation is one tile row's exact deviation: the sum over the two
// source axes of the hull's distance from the identity column, plus the band
// velocity's distance from the third column.
func gateTileDeviation(first, second [4]*gateNode, row int) *big.Rat {
	deviation := ratZero()
	for axis := range 2 {
		hull := IntervalHull(gateDerivRange(first, row, axis), gateDerivRange(second, row, axis))
		deviation = ratAdd(deviation, IntervalAbs(hull, big.NewRat(int64(boolInt(row == axis)), 1)))
	}
	velocity := IntervalSub(gateScalarRange(second, row), gateScalarRange(first, row))
	return ratAdd(deviation, IntervalAbs(velocity, big.NewRat(int64(boolInt(row == 2)), 1)))
}

// gateCandidate is one tile row with its float upper bound.
type gateCandidate struct {
	upper  float64
	iu, iv int
	row    int
}

// gateWorst is the maximum of gateTileDeviation over every tile and row.
func gateWorst(ctx context.Context, firstGrid, secondGrid [][]gateNode, tiles, rows int) (*big.Rat, error) {
	worst, _, err := gateWorstCount(ctx, firstGrid, secondGrid, tiles, rows)
	return worst, err
}

// gateWorstCount is gateWorst and also returns how many exact deviations it
// evaluated.
//
// Skip rule. Candidates are taken in descending float bound. Once a candidate's
// bound is at most worstLo, a float no greater than the exact maximum found
// so far, so is every later bound, and each exact deviation is at most its
// bound. Today's loop replaces worst only when a deviation is strictly greater
// than it, so none of them could change worst; the pass stops. A tie with
// worstLo stops too: an equal deviation does not replace worst either. worstLo
// is the nearest float below or at worst (rounded down when inexact), so it
// never exceeds worst.
func gateWorstCount(ctx context.Context, firstGrid, secondGrid [][]gateNode,
	tiles, rows int) (*big.Rat, int, error) {
	fe, se := gateGridEnc(firstGrid), gateGridEnc(secondGrid)
	candidates := make([]gateCandidate, 0, tiles*tiles*rows)
	for iu := range tiles {
		if err := ctx.Err(); err != nil {
			return nil, 0, err
		}
		for iv := range tiles {
			// Corner order matches TileCorners: (u0,v0),(u0,v1),(u1,v0),(u1,v1).
			f := [4]*gateNodeEnc{&fe[iu][iv], &fe[iu][iv+1], &fe[iu+1][iv], &fe[iu+1][iv+1]}
			s := [4]*gateNodeEnc{&se[iu][iv], &se[iu][iv+1], &se[iu+1][iv], &se[iu+1][iv+1]}
			for row := range rows {
				candidates = append(candidates, gateCandidate{upper: gateTileUpper(f, s, row), iu: iu, iv: iv, row: row})
			}
		}
	}
	slices.SortStableFunc(candidates, func(a, b gateCandidate) int { return cmp.Compare(b.upper, a.upper) })
	worst, worstLo, exactCount := ratZero(), 0.0, 0
	for n, c := range candidates {
		if c.upper <= worstLo {
			break
		}
		if n%16 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, 0, err
			}
		}
		f := [4]*gateNode{&firstGrid[c.iu][c.iv], &firstGrid[c.iu][c.iv+1],
			&firstGrid[c.iu+1][c.iv], &firstGrid[c.iu+1][c.iv+1]}
		s := [4]*gateNode{&secondGrid[c.iu][c.iv], &secondGrid[c.iu][c.iv+1],
			&secondGrid[c.iu+1][c.iv], &secondGrid[c.iu+1][c.iv+1]}
		deviation := gateTileDeviation(f, s, c.row)
		exactCount++
		if deviation.Cmp(worst) <= 0 {
			continue
		}
		worst = deviation
		x, exact := worst.Float64()
		if !exact {
			x = gateDown(x)
		}
		worstLo = x
	}
	return worst, exactCount, nil
}
