// Package linkagebound computes exact projection bounds for linkage point readings.
package linkagebound

import (
	"math"
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
	// LoF and HiF are Lo and Hi as the floats RoundCorners read them back
	// from, entry for entry. They are nil for a Bounds built another way;
	// readers then convert Lo and Hi.
	LoF, HiF [][3]float64
}

type Side struct {
	Corners Bounds
	H       []*big.Rat
	Seg     []proofbound.RatInterval
	Rem     *big.Rat
}

// StepsFrom is the segment steps read from one end: as they are from the
// interval's first end, negated from its second, where the segment runs
// backward. nil stays nil.
func StepsFrom(steps []proofbound.RatInterval, backward bool) []proofbound.RatInterval {
	if steps == nil || !backward {
		return steps
	}
	out := make([]proofbound.RatInterval, len(steps))
	for n, step := range steps {
		out[n] = proofbound.IntervalNeg(step)
	}
	return out
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

// FirstOrder is the side's first-order terms at corner c along axis d: a
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
			if up[d] == nil || proofarith.CmpRat(hi, up[d]) > 0 {
				up[d] = hi
			}
			if down[d] == nil || proofarith.CmpRat(lo, down[d]) > 0 {
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

// Lower is the largest L_n of agent-docs/linkage-check-design.md §5.8
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

// LowerWithDirection returns Lower's bound and the first coordinate axis and
// sense that attains it, in Lower's axis and sense order.
func LowerWithDirection(a, b Side) (*big.Rat, int, int) {
	aUp, aDown := a.Extents()
	bUp, bDown := b.Extents()
	var best *big.Rat
	axis, sense := 0, 1
	for d := range 3 {
		for _, candidate := range []struct {
			bound *big.Rat
			sense int
		}{
			{new(big.Rat).Neg(proofarith.AddRat(new(big.Rat), bDown[d], aUp[d])), 1},
			{new(big.Rat).Neg(proofarith.AddRat(new(big.Rat), bUp[d], aDown[d])), -1},
		} {
			if best == nil || candidate.bound.Cmp(best) > 0 {
				best, axis, sense = candidate.bound, d, candidate.sense
			}
		}
	}
	return best, axis, sense
}

// FaceNormals is agent-docs/linkage-check-design.md §5.8's candidate directions
// read off a point reading: for a prism's hull points, its extrusion
// direction and each side face's normal, the cross product of the side's
// bottom edge with that direction; for box corners, the three edge
// directions at the first corner. Each is formed in float from the readings'
// lower ends and read exactly: any fixed nonzero vector serves as n, so none
// needs a proof, and a zero or non-finite one is skipped.
func FaceNormals(c Bounds) []motionbound.RatVec {
	var out []motionbound.RatVec
	for _, v := range faceNormalVecs(c) {
		r, _ := motionbound.RatVecOf(v)
		out = append(out, r)
	}
	return out
}

// faceNormalVecs is FaceNormals' directions as the floats they are formed in,
// in the same order, with the same zero and non-finite directions left out.
// Each component is a float, so motionbound.RatVecOf reads it exactly.
func faceNormalVecs(c Bounds) []r3.Vec {
	at := func(n int) r3.Vec {
		if len(c.LoF) == len(c.Lo) {
			return r3.NewVec(c.LoF[n][0], c.LoF[n][1], c.LoF[n][2])
		}
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
	out := vecs[:0]
	for _, v := range vecs {
		if !finiteVec(v) || (v.X == 0 && v.Y == 0 && v.Z == 0) {
			continue
		}
		out = append(out, v)
	}
	return out
}

func finiteVec(v r3.Vec) bool {
	for _, x := range [3]float64{v.X, v.Y, v.Z} {
		if math.IsNaN(x) || math.IsInf(x, 0) {
			return false
		}
	}
	return true
}

// ExtentsAlong is Side.extents along any nonzero direction n:
// proven upper bounds on n·x (up) and on −n·x (down) over the body at every
// parameter of the interval, before its pad. The remainder bounds a vector,
// so it is charged Rem·norm, norm an upper bound on |n|.
func (s Side) ExtentsAlong(n motionbound.RatVec, norm *big.Rat) (up, down *big.Rat) {
	dot := func(p [3]proofbound.RatInterval) proofbound.RatInterval {
		var lo, hi, term big.Rat
		for d := range 3 {
			sign := n[d].Sign()
			if sign == 0 {
				continue
			}
			low, high := p[d].Lo, p[d].Hi
			if sign < 0 {
				low, high = high, low
			}
			proofarith.AddRat(&lo, &lo, proofarith.MulRat(&term, low, n[d]))
			proofarith.AddRat(&hi, &hi, proofarith.MulRat(&term, high, n[d]))
		}
		return proofbound.IntervalOwned(&lo, &hi)
	}
	for c := range s.Corners.Hi {
		pos := dot([3]proofbound.RatInterval{
			proofbound.IntervalOwned(s.Corners.Lo[c][0], s.Corners.Hi[c][0]),
			proofbound.IntervalOwned(s.Corners.Lo[c][1], s.Corners.Hi[c][1]),
			proofbound.IntervalOwned(s.Corners.Lo[c][2], s.Corners.Hi[c][2]),
		})
		linUp, linDown := new(big.Rat), new(big.Rat)
		if s.Seg != nil {
			sum := proofbound.IntervalOwned(new(big.Rat), new(big.Rat))
			for j, v := range s.Corners.Vel[c] {
				term := proofbound.IntervalMul(dot(v), s.Seg[j])
				proofarith.AddRat(sum.Lo, sum.Lo, term.Lo)
				proofarith.AddRat(sum.Hi, sum.Hi, term.Hi)
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
	return LowerHullWithWitnessAbove(a, b, nil)
}

// LowerHullAbove is LowerHull with LowerHullWithWitnessAbove's floor.
func LowerHullAbove(a, b Side, floor *big.Rat) *big.Rat {
	return LowerHullWithWitnessAbove(a, b, floor).Bound
}

// LowerHullWithWitnessAbove is LowerHullWithWitness for a caller that keeps
// the bound only when it exceeds floor (nil: no floor). Whenever
// LowerHullWithWitness's bound exceeds floor it returns exactly that witness;
// otherwise its Bound is nil or at most floor.
//
// It skips a direction only when it proves the direction cannot be the first
// to attain a bound above floor, and never decides a value: every direction it
// evaluates runs the exact path. A face normal with one nonzero component is a
// scaled coordinate direction, whose candidates are at most the coordinate
// candidates already taken (the extents scale by the component's magnitude and
// the rounded norms only shrink the quotient; this needs a nonnegative
// remainder and span, which floatSide checks). Any other direction is skipped
// when hullDirectionBelow proves both candidates below the best bound so far
// or below floor.
func LowerHullWithWitnessAbove(a, b Side, floor *big.Rat) HullWitness {
	pads := new(big.Rat)
	for _, p := range []*big.Rat{a.Corners.Pad, b.Corners.Pad} {
		if p != nil {
			pads.Add(pads, p)
		}
	}
	padsLo, _ := ratEnclose(pads)
	limit := math.Inf(-1)
	if floor != nil {
		limit, _ = ratEnclose(floor)
	}
	fa, fb := floatSide(a), floatSide(b)
	filter := fa.ok && fb.ok
	var best *big.Rat
	var winner motionbound.RatVec
	var winnerNorm *big.Rat
	winnerSense := 0
	var aAxisUp, aAxisDown, bAxisUp, bAxisDown [3]*big.Rat
	one, zero := big.NewRat(1, 1), new(big.Rat)
	dirs := []motionbound.RatVec{{one, zero, zero}, {zero, one, zero}, {zero, zero, one}}
	vecs := faceNormalVecs(a.Corners)
	vecs = append(vecs, faceNormalVecs(b.Corners)...)
	for direction := range 3 + len(vecs) {
		var n motionbound.RatVec
		var normUp, normDown *big.Rat
		if direction < 3 {
			// RatSqrtUp and RatSqrtDown of 1 are both exactly 1.
			n = dirs[direction]
			normUp, normDown = big.NewRat(1, 1), big.NewRat(1, 1)
		} else {
			v := vecs[direction-3]
			if filter && (nonzeroComponents(v) == 1 || hullDirectionBelow(fa, fb, v, padsLo, limit)) {
				continue
			}
			n, _ = motionbound.RatVecOf(v)
			sq := axisSq(n)
			normUp = sqrtUpRat(sq)
			normDown = proofarith.FloatRat(proofbound.RatSqrtDown(sq))
			if normUp == nil || normDown == nil || normDown.Sign() <= 0 {
				continue
			}
			if !filter && exactAxial(n, normUp, normDown) {
				// Both senses of an exact axial direction were already checked.
				continue
			}
		}
		var aUp, aDown, bUp, bDown *big.Rat
		if direction < 3 {
			if direction == 0 {
				aAxisUp, aAxisDown = a.Extents()
				bAxisUp, bAxisDown = b.Extents()
			}
			aUp, aDown = aAxisUp[direction], aAxisDown[direction]
			bUp, bDown = bAxisUp[direction], bAxisDown[direction]
		} else {
			aUp, aDown = a.ExtentsAlong(n, normUp)
			bUp, bDown = b.ExtentsAlong(n, normUp)
		}
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
				if lo, _ := ratEnclose(best); lo > limit {
					limit = lo
				}
			}
		}
	}
	return HullWitness{Bound: best, Axis: winner, Norm: winnerNorm, Sense: winnerSense}
}

func nonzeroComponents(v r3.Vec) int {
	count := 0
	for _, x := range [3]float64{v.X, v.Y, v.Z} {
		if x != 0 {
			count++
		}
	}
	return count
}

// exactAxial reports whether n has one nonzero component whose magnitude both
// rounded norms equal.
func exactAxial(n motionbound.RatVec, normUp, normDown *big.Rat) bool {
	var component *big.Rat
	for _, v := range n {
		if v.Sign() == 0 {
			continue
		}
		if component != nil {
			return false
		}
		component = v
	}
	if component == nil {
		return false
	}
	magnitude := new(big.Rat).Abs(component)
	return normUp.Cmp(magnitude) == 0 && normDown.Cmp(magnitude) == 0
}
