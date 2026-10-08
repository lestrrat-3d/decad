// Package linkagebound computes exact projection bounds for linkage point readings.
package linkagebound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type Bounds struct {
	Lo, Hi [][3]*big.Rat
	Vel    [][][3]proofbound.RatInterval
	Pad    *big.Rat
	PrismK int
}

type Side struct {
	Corners Bounds
	H       []*big.Rat
	Seg     []proofbound.RatInterval
	Rem     *big.Rat
}

func ivAbsUpper(iv proofbound.RatInterval) *big.Rat {
	out := new(big.Rat).Abs(iv.Lo)
	if hi := new(big.Rat).Abs(iv.Hi); hi.Cmp(out) > 0 {
		out = hi
	}
	return out
}

func axisSq(a motionbound.RatVec) *big.Rat {
	return proofbound.RatAdd(proofbound.RatMul(a[0], a[0]), proofbound.RatMul(a[1], a[1]), proofbound.RatMul(a[2], a[2]))
}

func sqrtUpRat(q *big.Rat) *big.Rat {
	return proofarith.FloatRat(proofbound.RatSqrtUp(q))
}

// ZeroVec reports whether every exact component is zero.
func ZeroVec(v motionbound.RatVec) bool {
	return v[0].Sign() == 0 && v[1].Sign() == 0 && v[2].Sign() == 0
}

// firstOrder is the side's first-order terms at corner c along axis d: a
// proven upper bound on how far the expansion moves x[d] (up) and −x[d]
// (down) over the interval. The segment term's sum is linear in the
// fraction t ∈ [0, 1] of the step, so its largest value is at t = 0 or 1:
// max(0, Σ) along x[d] and max(0, −Σ) along −x[d], each over the enclosure
// of Σ. The box form charges |v[d]|·h along both.
func (s Side) FirstOrder(c, d int) (up, down *big.Rat) {
	if s.Seg != nil {
		sum := proofbound.PointInterval(new(big.Rat))
		for n, v := range s.Corners.Vel[c] {
			sum = proofbound.IntervalAdd(sum, proofbound.IntervalMul(v[d], s.Seg[n]))
		}
		up, down = new(big.Rat), new(big.Rat)
		if sum.Hi.Sign() > 0 {
			up.Set(sum.Hi)
		}
		if sum.Lo.Sign() < 0 {
			down.Neg(sum.Lo)
		}
		return up, down
	}
	lin := new(big.Rat)
	for n, v := range s.Corners.Vel[c] {
		if s.H[n].Sign() == 0 {
			continue
		}
		term := ivAbsUpper(v[d])
		proofarith.AddRat(lin, lin, proofarith.MulRat(term, term, s.H[n]))
	}
	return lin, new(big.Rat).Set(lin)
}

// extents are proven upper bounds on x[axis] (up) and on −x[axis] (down)
// over the body at every parameter of the interval, per axis: the largest
// over its corners of the corner's coordinate end plus its first-order term,
// then the remainder.
func (s Side) Extents() (up, down [3]*big.Rat) {
	for c := range s.Corners.Hi {
		for d := range 3 {
			linUp, linDown := s.FirstOrder(c, d)
			hi := proofarith.AddRat(linUp, linUp, s.Corners.Hi[c][d])
			lo := proofarith.SubRat(linDown, linDown, s.Corners.Lo[c][d])
			if up[d] == nil || hi.Cmp(up[d]) > 0 {
				up[d] = hi
			}
			if down[d] == nil || lo.Cmp(down[d]) > 0 {
				down[d] = lo
			}
		}
	}
	if s.Rem != nil {
		for d := range 3 {
			proofarith.AddRat(up[d], up[d], s.Rem)
			proofarith.AddRat(down[d], down[d], s.Rem)
		}
	}
	return up, down
}

// projectionLower is the largest L_n of docs/linkage-check-design.md §5.8
// over the six coordinate directions n = ±e_axis: the partner's least extent
// along n less the body's greatest. Along +e_axis that is −b.down − a.up,
// along −e_axis −b.up − a.down. The distance between two sets is at least
// the separation of their projections onto any unit vector, so each L_n is
// a proven lower bound on the pair's gap at every parameter of the interval.
func Lower(a, b Side) *big.Rat {
	aUp, aDown := a.Extents()
	bUp, bDown := b.Extents()
	var best *big.Rat
	for d := range 3 {
		for _, l := range []*big.Rat{
			new(big.Rat).Neg(proofarith.AddRat(new(big.Rat), bDown[d], aUp[d])),
			new(big.Rat).Neg(proofarith.AddRat(new(big.Rat), bUp[d], aDown[d])),
		} {
			if best == nil || l.Cmp(best) > 0 {
				best = l
			}
		}
	}
	return best
}

// faceNormals is docs/linkage-check-design.md §5.8's candidate directions
// read off a point reading: for a prism's hull points, its extrusion
// direction and each side face's normal, the cross product of the side's
// bottom edge with that direction; for box corners, the three edge
// directions at the first corner. Each is formed in float from the readings'
// lower ends and read exactly: any fixed nonzero vector serves as n, so none
// needs a proof, and a zero or non-finite one is skipped.
func FaceNormals(c Bounds) []motionbound.RatVec {
	at := func(n int) r3.Vec {
		f := func(q *big.Rat) float64 { v, _ := q.Float64(); return v }
		return r3.NewVec(f(c.Lo[n][0]), f(c.Lo[n][1]), f(c.Lo[n][2]))
	}
	var vecs []r3.Vec
	switch {
	case c.PrismK >= 3 && len(c.Lo) == 2*c.PrismK:
		k := c.PrismK
		axis := at(k).Sub(at(0))
		vecs = append(vecs, axis)
		for n := range k {
			edge := at((n + 1) % k).Sub(at(n))
			vecs = append(vecs, edge.Cross(axis))
		}
	case len(c.Lo) == 8:
		vecs = append(vecs, at(1).Sub(at(0)), at(2).Sub(at(0)), at(4).Sub(at(0)))
	}
	var out []motionbound.RatVec
	for _, v := range vecs {
		if r, ok := motionbound.RatVecOf(v); ok && !ZeroVec(r) {
			out = append(out, r)
		}
	}
	return out
}

// extentsAlong is Side.extents along any nonzero direction n:
// proven upper bounds on n·x (up) and on −n·x (down) over the body at every
// parameter of the interval, before its pad. The remainder bounds a vector,
// so it is charged Rem·norm, norm an upper bound on |n|.
func (s Side) ExtentsAlong(n motionbound.RatVec, norm *big.Rat) (up, down *big.Rat) {
	dot := func(p [3]proofbound.RatInterval) proofbound.RatInterval {
		var sum proofbound.RatInterval
		for d := range 3 {
			if n[d].Sign() == 0 {
				continue
			}
			term := proofbound.IntervalScale(p[d], n[d])
			if sum.Lo == nil {
				sum = term
			} else {
				sum = proofbound.IntervalAdd(sum, term)
			}
		}
		if sum.Lo == nil {
			return proofbound.PointInterval(new(big.Rat))
		}
		return sum
	}
	for c := range s.Corners.Hi {
		pos := dot([3]proofbound.RatInterval{
			proofbound.IntervalOwned(s.Corners.Lo[c][0], s.Corners.Hi[c][0]),
			proofbound.IntervalOwned(s.Corners.Lo[c][1], s.Corners.Hi[c][1]),
			proofbound.IntervalOwned(s.Corners.Lo[c][2], s.Corners.Hi[c][2]),
		})
		linUp, linDown := new(big.Rat), new(big.Rat)
		if s.Seg != nil {
			sum := proofbound.PointInterval(new(big.Rat))
			for j, v := range s.Corners.Vel[c] {
				sum = proofbound.IntervalAdd(sum, proofbound.IntervalMul(dot(v), s.Seg[j]))
			}
			if sum.Hi.Sign() > 0 {
				linUp.Set(sum.Hi)
			}
			if sum.Lo.Sign() < 0 {
				linDown.Neg(sum.Lo)
			}
		} else {
			for j, v := range s.Corners.Vel[c] {
				if s.H[j].Sign() == 0 {
					continue
				}
				term := ivAbsUpper(dot(v))
				proofarith.AddRat(linUp, linUp, proofarith.MulRat(term, term, s.H[j]))
			}
			linDown.Set(linUp)
		}
		hi := proofarith.AddRat(linUp, linUp, pos.Hi)
		lo := proofarith.SubRat(linDown, linDown, pos.Lo)
		if up == nil || hi.Cmp(up) > 0 {
			up = hi
		}
		if down == nil || lo.Cmp(down) > 0 {
			down = lo
		}
	}
	if s.Rem != nil {
		rem := proofarith.MulRat(new(big.Rat), s.Rem, norm)
		proofarith.AddRat(up, up, rem)
		proofarith.AddRat(down, down, rem)
	}
	return up, down
}

// projectionLowerHull is docs/linkage-check-design.md §5.8's hull bound for
// a pair whose two sides read hull points: the largest L_n over the six
// coordinate directions and every face normal of either side, each
// numerator divided by |n| rounded up when positive and down otherwise, less
// both pads; nil when no direction can be normed.
func LowerHull(a, b Side) *big.Rat {
	return LowerHullWithWitness(a, b).Bound
}

// HullWitness records the first direction attaining a lower hull bound.
type HullWitness struct {
	Bound *big.Rat
	Axis  motionbound.RatVec
	Norm  *big.Rat
	Sense int
}

// LowerHullWithWitness also returns the first direction attaining the bound.
// The upper norm is used to charge that direction's derivative shares.
func LowerHullWithWitness(a, b Side) HullWitness {
	one, zero := big.NewRat(1, 1), new(big.Rat)
	dirs := []motionbound.RatVec{{one, zero, zero}, {zero, one, zero}, {zero, zero, one}}
	dirs = append(dirs, FaceNormals(a.Corners)...)
	dirs = append(dirs, FaceNormals(b.Corners)...)
	pads := new(big.Rat)
	for _, p := range []*big.Rat{a.Corners.Pad, b.Corners.Pad} {
		if p != nil {
			pads.Add(pads, p)
		}
	}
	var best *big.Rat
	var winner motionbound.RatVec
	var winnerNorm *big.Rat
	winnerSense := 0
	for _, n := range dirs {
		sq := axisSq(n)
		normUp := sqrtUpRat(sq)
		normDown := proofarith.FloatRat(proofbound.RatSqrtDown(sq))
		if normUp == nil || normDown == nil || normDown.Sign() <= 0 {
			continue
		}
		aUp, aDown := a.ExtentsAlong(n, normUp)
		bUp, bDown := b.ExtentsAlong(n, normUp)
		for _, candidate := range []struct {
			num   *big.Rat
			sense int
		}{
			{new(big.Rat).Neg(proofarith.AddRat(new(big.Rat), bDown, aUp)), 1},
			{new(big.Rat).Neg(proofarith.AddRat(new(big.Rat), bUp, aDown)), -1},
		} {
			num := candidate.num
			norm := normDown
			if num.Sign() > 0 {
				norm = normUp
			}
			l := num.Quo(num, norm)
			l.Sub(l, pads)
			if best == nil || l.Cmp(best) > 0 {
				best = l
				winner, winnerNorm, winnerSense = n, normUp, candidate.sense
			}
		}
	}
	return HullWitness{Bound: best, Axis: winner, Norm: winnerNorm, Sense: winnerSense}
}
