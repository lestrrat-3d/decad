package filletband

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capband"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file is agent-docs/modify-design.md Part L §5.6–§5.7: an arc whose
// recorded End is off the circle its Start states. The record denotes Start's
// radius up to End's angle and a radial step to End
// (agent-docs/evaluator-design.md §4). The strip's volume and first moments
// read the loop restated on each such arc's bisector centre (§5.6,
// capband.OffRadiusShift); every other reading charges the step locally at
// the corner it sits at (§5.7). Every number here is rounded upward once.

// RestateTerms is one loop's restatement (§5.6): capband.OffRadiusLoop's
// terms at the fillet's reach, and how many LF6 corners sit beside a
// restated arc. It is zero for a loop with no off-radius arc.
type RestateTerms struct {
	capband.OffRadiusTerms
	ReflexBeside int
	// Walks marks the restated walks, nil for a loop that restates nothing.
	Walks []bool
}

// Charged reports whether corner k carries a near-tangent reach, a radial
// step, or a restated arc on either side: a reading the partial-fillet route
// does not charge (agent-docs/modify-design.md Part L §5.5).
func (l Loop) Charged(k int) bool {
	n := len(l.Walks)
	if l.Restate.Walks != nil && (l.Restate.Walks[k] || l.Restate.Walks[(k+n-1)%n]) {
		return true
	}
	return l.Gap != nil && (l.Rho[k] > 0 || l.RhoRec[k] > 0 || l.Gap[k] > 0)
}

// segmentOffRadius reports whether seg is an off-radius whole arc.
func segmentOffRadius(seg sectionrecord.CurveSegment) bool {
	arc, ok := seg.(sectionrecord.ArcSeg)
	return ok && capband.OffRadius(arc)
}

// walkTangents is a normalized segment's exact tangent at its walk start and
// end, an arc's read about centre c: a line runs Start → End (End → Start
// when TStart == 1); an arc runs counter-clockwise Start → End, or clockwise
// End → Start when TStart == 1, its tangent at P that sense times
// rot90(P − c).
func walkTangents(seg sectionrecord.CurveSegment, c ratVec) (tanStart, tanEnd ratVec, ok bool) {
	pt := func(p sectionrecord.Point2) (ratVec, bool) {
		q, err := ratPoint(Point{U: p.U, V: p.V})
		return q, err == nil
	}
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		a, okA := pt(s.Start)
		b, okB := pt(s.End)
		if s.TStart == 1 {
			a, b = b, a
		}
		d := b.sub(a)
		return d, d, okA && okB
	case sectionrecord.ArcSeg:
		a, okA := pt(s.Start)
		b, okB := pt(s.End)
		if !okA || !okB || c[0] == nil || c[1] == nil {
			return ratVec{}, ratVec{}, false
		}
		ta, tb := a.sub(c).perp(), b.sub(c).perp()
		if s.TStart == 1 {
			return tb.neg(), ta.neg(), true
		}
		return ta, tb, true
	}
	return ratVec{}, ratVec{}, false
}

// readCentre is seg's centre as the restated reading takes it: an off-radius
// arc's capband.RestatedCentre, an arc's recorded Centre otherwise, and none
// for a line.
func readCentre(seg sectionrecord.CurveSegment) (ratVec, bool) {
	arc, ok := seg.(sectionrecord.ArcSeg)
	if !ok {
		return ratVec{}, true
	}
	c, ok := capband.RestatedCentre(arc)
	return ratVec{c[0], c[1]}, ok
}

// restatedTurn is the exact sign of the turn from prev's walk end to cur's
// walk start with every off-radius arc read about its restated centre.
func restatedTurn(prev, cur sectionrecord.CurveSegment) (int, bool) {
	cp, okP := readCentre(prev)
	cc, okC := readCentre(cur)
	if !okP || !okC {
		return 0, false
	}
	_, a, okA := walkTangents(prev, cp)
	b, _, okB := walkTangents(cur, cc)
	if !okA || !okB {
		return 0, false
	}
	return a.cross(b).Sign(), true
}

// junctionGap bounds the radial step an arc's record holds at the recorded
// junction v: ||v − C| − |Start − C||, zero for a line and for an arc whose
// ends lie on one circle (§5.7). It reads |r_v² − r_S²| / (r_v + r_S) with
// both roots rounded down, and reports false where a number does not lift.
func junctionGap(seg sectionrecord.CurveSegment, v ratVec) (float64, bool) {
	arc, ok := seg.(sectionrecord.ArcSeg)
	if !ok || !capband.OffRadius(arc) {
		return 0, true
	}
	c, err := ratPoint(Point{U: arc.Center.U, V: arc.Center.V})
	if err != nil {
		return 0, false
	}
	s, err := ratPoint(Point{U: arc.Start.U, V: arc.Start.V})
	if err != nil {
		return 0, false
	}
	ds, dv := s.sub(c), v.sub(c)
	rs2, rv2 := ds.dot(ds), dv.dot(dv)
	diff := new(big.Rat).Abs(new(big.Rat).Sub(rv2, rs2))
	if diff.Sign() == 0 {
		return 0, true
	}
	a, b := proofarith.FloatRat(proofbound.RatSqrtDown(rv2)), proofarith.FloatRat(proofbound.RatSqrtDown(rs2))
	if a == nil || b == nil {
		return 0, false
	}
	den := new(big.Rat).Add(a, b)
	if den.Sign() <= 0 {
		return 0, false
	}
	up := proofbound.RatFloatUp(new(big.Rat).Quo(diff, den))
	return up, !proofbound.IsNonFinite(up)
}

// localRho is corner k's near-tangent reach the local terms read, the larger
// of its restated and recorded readings.
func (l Loop) localRho(k int) float64 {
	return math.Max(l.Rho[k], l.RhoRec[k])
}

// stepRoot is 2π·√(k·(T + δ + ρ)(δ + ρ)), the lateral reach of a radial step δ
// at offset up to T (§5.7), rounded upward; zero when δ is. k is
// stepCurvature's factor for the corner's two walks.
func stepRoot(T, delta, rho, k float64) float64 {
	if delta == 0 {
		return 0
	}
	prod := proofbound.ProductUpper(k, proofbound.ProductUpper(proofbound.AbsSumUpper(T, delta, rho), proofbound.AbsSumUpper(delta, rho)))
	root := proofbound.RatSqrtUp(proofarith.FloatRat(prod))
	// 6.2832 > 2π.
	return proofbound.ProductUpper(6.2832, root)
}

// stepCurvature is the factor a circular walk with its material outside
// (a hole rim's arc) puts on a step's lateral reach at corner k: its moved
// circle, of radius R + t, curves round v, so a chord of it reaching within
// t + δ + ρ of v is longer than a line's by √((R + T)/R) (§5.7). It is the
// larger of (R_hi + T)/R_lo over the corner's two walks, R read from the
// walk's recorded centre to its two recorded ends, and 1 for a line or a walk
// holding its material inside. ok is false where R_lo is not positive.
func (l Loop) stepCurvature(k int, T float64) (float64, bool) {
	n := len(l.Walks)
	out := 1.0
	for _, w := range []Walk{l.Walks[(k+n-1)%n], l.Walks[k]} {
		if !w.Circular || w.CCW {
			continue
		}
		c, err := ratPoint(w.Center)
		if err != nil {
			return 0, false
		}
		lo, hi := math.Inf(1), 0.0
		for _, p := range []Point{w.Start, w.End} {
			q, err := ratPoint(p)
			if err != nil {
				return 0, false
			}
			d := q.sub(c)
			sq := d.dot(d)
			lo = math.Min(lo, proofbound.RatSqrtDown(sq))
			hi = math.Max(hi, proofbound.RatSqrtUp(sq))
		}
		if !(lo > 0) {
			return 0, false
		}
		num := proofarith.FloatRat(proofbound.AbsSumUpper(hi, T))
		f := proofbound.RatFloatUp(new(big.Rat).Quo(num, proofarith.FloatRat(lo)))
		out = math.Max(out, f)
	}
	return out, true
}

// LocalReach is each corner's reach (§5.7): every point of the true band near
// the corner lies, in its own level, within it of the stated patches, and each
// stated point within it of a true one. It is 2ρ̃ + 3δ plus, at a radial step
// δ > 0, stepRoot, with ρ̃ the larger near-tangent reading and T the largest
// offset. A corner whose curvature factor does not lift reads +Inf, which
// every consumer refuses. It returns nil for a loop with no near-tangent
// reach, no step and no restated arc.
func (l Loop) LocalReach(radius proofbound.RatInterval) []float64 {
	if l.Gap == nil {
		return nil
	}
	T := proofbound.RatFloatUp(radius.Hi)
	out := make([]float64, len(l.Gap))
	for k, delta := range l.Gap {
		rho := l.localRho(k)
		curv, ok := l.stepCurvature(k, T)
		if !ok {
			out[k] = math.Inf(1)
			continue
		}
		out[k] = proofbound.AbsSumUpper(proofbound.ProductUpper(2, rho), proofbound.ProductUpper(3, delta), stepRoot(T, delta, rho, curv))
	}
	return out
}

// LevelLength is, per corner, the bound on how much the length of the true
// band's level curve near the corner differs from the stated patches' at any
// one offset (§5.7): 2ρ̃, plus 4·stepRoot + 10δ at a radial step. A patch's
// area changes by at most (π/2)·r_hi times its corners' sum.
func (l Loop) LevelLength(radius proofbound.RatInterval) []float64 {
	if l.Gap == nil {
		return nil
	}
	T := proofbound.RatFloatUp(radius.Hi)
	out := make([]float64, len(l.Gap))
	for k, delta := range l.Gap {
		rho := l.localRho(k)
		curv, ok := l.stepCurvature(k, T)
		if !ok {
			out[k] = math.Inf(1)
			continue
		}
		step := 0.0
		if delta > 0 {
			step = proofbound.AbsSumUpper(proofbound.ProductUpper(4, stepRoot(T, delta, rho, curv)), proofbound.ProductUpper(10, delta))
		}
		out[k] = proofbound.AbsSumUpper(proofbound.ProductUpper(2, rho), step)
	}
	return out
}

// LevelAreaAllowance is (π/2)·r_hi·length, the area change of one patch whose
// level curve changes by at most length at every offset: a pipe's area
// element is its level length times r·dφ over the quarter.
func LevelAreaAllowance(length float64, radius proofbound.RatInterval) (*big.Rat, bool) {
	d := proofarith.FloatRat(length)
	if d == nil || d.Sign() < 0 || radius.Hi.Sign() <= 0 {
		return nil, false
	}
	out := new(big.Rat).Mul(proofbound.HalfPiInterval().Hi, radius.Hi)
	return out.Mul(out, d), true
}

// MeridianExcess bounds, per corner, how far the length of a true seam there
// can exceed the quarter circle (π/2)·r a meridian states (§5.7). At an LF5
// or LF5′ corner: The seam lifts the in-plane path of the corner's true point, which
// moves along the bisector of the two carriers' inward normals at rate
// 1/cos(γ/2), γ <= (π/2)·ρ̃·(1/r_lo + Σ 1/R_lo) over the circular carriers;
// so the length is at most (π/2)·r·(1 + γ²/4). At any corner a radial step
// adds twice the local reach, the in-plane travel its monotone boundaries
// take. It reports
// false where a moved radius is not positive.
func (l Loop) MeridianExcess(walks []survey2d.SideWalk, radius proofbound.RatInterval) ([]float64, bool) {
	if l.Gap == nil {
		return nil, true
	}
	reach := l.LocalReach(radius)
	n := len(walks)
	out := make([]float64, n)
	for k := range l.Gap {
		if l.Gap[k] > 0 {
			out[k] = proofbound.ProductUpper(2, reach[k])
		}
		if l.Corners[k] != Tangent && l.Corners[k] != NearTangent {
			continue
		}
		rho := proofarith.FloatRat(l.localRho(k))
		if rho == nil || radius.Lo.Sign() <= 0 {
			return nil, false
		}
		inv := new(big.Rat).Inv(radius.Lo)
		for _, w := range []survey2d.SideWalk{walks[(k+n-1)%n], walks[k]} {
			if !w.IsCircular() {
				continue
			}
			lo, ok := movedRadiusLower(w, walks[k].StartU, walks[k].StartV, radius.Hi)
			if !ok {
				return nil, false
			}
			inv.Add(inv, new(big.Rat).Inv(lo))
		}
		gamma := new(big.Rat).Mul(proofbound.HalfPiInterval().Hi, new(big.Rat).Mul(rho, inv))
		excess := new(big.Rat).Mul(proofbound.HalfPiInterval().Hi, radius.Hi)
		excess.Mul(excess, new(big.Rat).Quo(new(big.Rat).Mul(gamma, gamma), big.NewRat(4, 1)))
		out[k] = proofbound.AbsSumUpper(out[k], proofbound.RatFloatUp(excess))
		if proofbound.IsNonFinite(out[k]) {
			return nil, false
		}
	}
	return out, true
}

// stepNormalTurn is the normal turn charged beside a radial step: 3/2 >
// √2, the unit-vector distance a quarter turn spans. Near the step the true
// band wraps round the step's own corner, whose normals run from the wall's
// to the step's (§5.7).
var stepNormalTurn = big.NewRat(3, 2)

// NormalTurn bounds, for corner k, how far the true band's in-plane outward
// normals near the corner stray from the normals of the two patches beside
// it: stepNormalTurn at a radial step, NearTangentNormalTurn at a corner with a
// near-tangent reach in either reading, and zero elsewhere. ok is false where that function refuses.
func (l Loop) NormalTurn(walks []survey2d.SideWalk, k int, radius proofbound.RatInterval) (*big.Rat, bool) {
	if l.Gap == nil {
		return new(big.Rat), true
	}
	if l.Gap[k] > 0 {
		return new(big.Rat).Set(stepNormalTurn), true
	}
	if l.localRho(k) == 0 {
		return new(big.Rat), true
	}
	return NearTangentNormalTurn(walks, k, l.localRho(k), radius)
}

// RestateMassAllowance bounds the change restating the loop's off-radius
// arcs makes to the strip's volume and to each in-plane first moment (§5.6).
// At every offset t <= T the strip areas of the recorded section X and the
// restated X′ differ by at most 4·Δ·Perimeter (each of the region and its
// offset by 2·Δ·Perimeter, the coarea bound of
// agent-docs/tessellation-reach-design.md §7.1); Table CF's piece of a
// restated arc moves by at most 2Δ·(4π(R_hi + T) + 2T) + πΔ² (its boundary's
// Δ-tube); and an LF6 sector beside one by at most T·Δ. Over the quarter the
// volume moves by r_hi times their sum, and each in-plane moment by that times
// coord + T + Δ, coord bounding the loop's plane-local coordinates.
func RestateMassAllowance(terms RestateTerms, coord float64, radius proofbound.RatInterval) (volume, moment *big.Rat, ok bool) {
	if terms.Arcs == 0 {
		return new(big.Rat), new(big.Rat), true
	}
	T := proofbound.RatFloatUp(radius.Hi)
	d := terms.Shift
	// 12.5664 > 4π, 3.1416 > π.
	body := proofbound.ProductUpper(4, proofbound.ProductUpper(d, terms.Perimeter))
	tube := proofbound.AbsSumUpper(
		proofbound.ProductUpper(2*d, proofbound.AbsSumUpper(proofbound.ProductUpper(12.5664, proofbound.AbsSumUpper(terms.RadiusHi, T)), 2*T)),
		proofbound.ProductUpper(3.1416, proofbound.ProductUpper(d, d)))
	pieces := proofbound.ProductUpper(float64(terms.Arcs), tube)
	sectors := proofbound.ProductUpper(float64(terms.ReflexBeside), proofbound.ProductUpper(T, d))
	level := proofbound.AbsSumUpper(body, pieces, sectors)
	v := proofarith.FloatRat(level)
	reach := proofarith.FloatRat(proofbound.AbsSumUpper(coord, T, d))
	if v == nil || reach == nil || proofbound.IsNonFinite(level) {
		return nil, nil, false
	}
	volume = new(big.Rat).Mul(radius.Hi, v)
	return volume, new(big.Rat).Mul(volume, reach), true
}
