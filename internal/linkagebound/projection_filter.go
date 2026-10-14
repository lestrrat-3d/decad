package linkagebound

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file is a float filter for LowerHullWithWitnessAbove. It only skips
// work: it proves a direction cannot matter and never supplies a value. Every
// float operation is followed by a step to the neighbouring float on the
// unsafe side, so a result is an enclosure of the real-number result however
// the compiler rounds or fuses the operation. Each product is wrapped in an
// explicit float64 conversion, which forbids fusing it into a later sum. A
// NaN or infinite value never proves anything, because every proving
// comparison is false for it.

func fDown(x float64) float64 { return proofarith.NextDown(x) }
func fUp(x float64) float64   { return proofarith.NextUp(x) }

// Each primitive below is an enclosure of the real-number result of its
// operation: down is at most it and up is at least it, whatever the operands'
// rounding. The operation rounds to nearest at worst, so its result is within
// one float of the real one. The explicit float64 conversion rounds the
// product at once, so no compiler may fuse it into a following sum.
func sumDown(x, y float64) float64      { return fDown(x + y) }
func sumUp(x, y float64) float64        { return fUp(x + y) }
func productDown(x, y float64) float64  { return fDown(float64(x * y)) }
func productUp(x, y float64) float64    { return fUp(float64(x * y)) }
func quotientUp(x, y float64) float64   { return fUp(x / y) }
func differenceUp(x, y float64) float64 { return fUp(x - y) }
func sqrtDown(x float64) float64        { return fDown(math.Sqrt(x)) }
func sqrtUp(x float64) float64          { return fUp(math.Sqrt(x)) }

// ratEnclose returns floats lo <= r <= hi.
func ratEnclose(r *big.Rat) (lo, hi float64) {
	f, exact := r.Float64()
	if exact {
		return f, f
	}
	return fDown(f), fUp(f)
}

// sideFloats encloses a side's corner coordinates in floats for
// hullDirectionBelow. ok is false when a side breaks the test's premises (a
// negative span H on the box-form path or a negative remainder, which would
// make a motion term negative) or a corner end is not finite.
type sideFloats struct {
	lo, hi [][3][2]float64 // per corner and axis, [lower, upper] float enclosure of Lo and Hi
	ok     bool
}

func floatSide(s Side) sideFloats {
	c := s.Corners
	if s.Seg == nil {
		for _, h := range s.H {
			if h.Sign() < 0 {
				return sideFloats{}
			}
		}
	}
	if s.Rem != nil && s.Rem.Sign() < 0 {
		return sideFloats{}
	}
	out := sideFloats{lo: make([][3][2]float64, len(c.Lo)), hi: make([][3][2]float64, len(c.Hi)), ok: true}
	if len(c.Lo) != len(c.Hi) {
		return sideFloats{}
	}
	exactFloats := len(c.LoF) == len(c.Lo) && len(c.HiF) == len(c.Hi)
	for k := range c.Lo {
		for d := range 3 {
			var l, h [2]float64
			if exactFloats {
				l, h = [2]float64{c.LoF[k][d], c.LoF[k][d]}, [2]float64{c.HiF[k][d], c.HiF[k][d]}
			} else {
				l[0], l[1] = ratEnclose(c.Lo[k][d])
				h[0], h[1] = ratEnclose(c.Hi[k][d])
			}
			for _, x := range [4]float64{l[0], l[1], h[0], h[1]} {
				if math.IsNaN(x) || math.IsInf(x, 0) {
					return sideFloats{}
				}
			}
			out.lo[k][d], out.hi[k][d] = l, h
		}
	}
	return out
}

// mulLo and mulHi bound x*y from below and above for x in [x0, x1] and a
// float y.
func mulLo(x0, x1, y float64) float64 {
	if y >= 0 {
		return productDown(x0, y)
	}
	return productDown(x1, y)
}

func mulHi(x0, x1, y float64) float64 {
	if y >= 0 {
		return productUp(x1, y)
	}
	return productUp(x0, y)
}

// staticLower returns lower bounds on the side's up and down extents along n
// with every motion term dropped: the largest over corners of the exact
// code's pos.Hi, and of its -pos.Lo.
func staticLower(f sideFloats, n [3]float64) (up, down float64) {
	up, down = math.Inf(-1), math.Inf(-1)
	for k := range f.lo {
		hiSum, loSum := 0.0, 0.0
		for d := range 3 {
			if n[d] == 0 {
				continue
			}
			p, q := f.hi[k][d], f.lo[k][d]
			if n[d] < 0 {
				p, q = q, p
			}
			hiSum = sumDown(hiSum, mulLo(p[0], p[1], n[d]))
			loSum = sumUp(loSum, mulHi(q[0], q[1], n[d]))
		}
		up = math.Max(up, hiSum)
		down = math.Max(down, -loSum)
	}
	return up, down
}

// hullDirectionBelow reports whether both candidates of the face normal v are
// proven below limit, a float at or below the bound to beat. A side that fails
// floatSide's premises must not be passed. Equal to limit proves nothing.
func hullDirectionBelow(a, b sideFloats, v r3.Vec, padsLo, limit float64) bool {
	return hullDirectionUpper(a, b, v, padsLo) < limit
}

// hullDirectionUpper is a float at least as large as both candidates of the
// face normal v, or +Inf or NaN when none is proven. It drops every motion
// term: each corner's first-order term and the remainder only raise up and
// down, so the static projections alone give lower bounds on them and an upper
// bound on each candidate. padsLo is at most the sum of both pads.
func hullDirectionUpper(a, b sideFloats, v r3.Vec, padsLo float64) float64 {
	nf := [3]float64{v.X, v.Y, v.Z}
	sqLo, sqHi := 0.0, 0.0
	for d := range 3 {
		sqLo = sumDown(sqLo, productDown(nf[d], nf[d]))
		sqHi = sumUp(sqHi, productUp(nf[d], nf[d]))
	}
	if !(sqLo > 0) || math.IsInf(sqHi, 0) {
		return math.Inf(1)
	}
	nLo, nHi := sqrtDown(sqLo), sqrtUp(sqHi)
	if !(nLo > 0) {
		return math.Inf(1)
	}
	aUp, aDown := staticLower(a, nf)
	bUp, bDown := staticLower(b, nf)
	upper := math.Inf(-1)
	for _, s := range [2]float64{sumDown(bDown, aUp), sumDown(bUp, aDown)} {
		m := -s
		var u float64
		if m > 0 {
			u = quotientUp(m, nLo)
		} else {
			u = quotientUp(m, nHi)
		}
		u = differenceUp(u, padsLo)
		if math.IsNaN(u) {
			return u
		}
		upper = math.Max(upper, u)
	}
	return upper
}
