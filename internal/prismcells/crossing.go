package prismcells

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// This file is docs/general-boolean-design.md §3 A6's crossing-sensitivity
// charge. An operand whose recorded boundary sits within δ of the boundary
// it denotes moves every crossing with the other operand by more than δ: two
// carriers that cross at an angle θ, displaced by δ1 and δ2, meet up to
// (δ1 + δ2)/sin θ from where the recorded carriers meet. The charge states
// that distance for every crossing the arrangement cut, with sin θ bounded
// below by exact rational arithmetic on the recorded carriers, and refuses a
// crossing whose bound falls below the dimensionless noise floor ε of
// docs/verification-design.md §4 (sectionaudit.ContactEps).
//
// The argument, for one crossing O of recorded carriers γ1 and γ2 and its
// denoted twin P* (within δ1 of γ1 and δ2 of γ2): take Q1 on γ1 and Q2 on
// γ2 nearest P*, so |Q1 − Q2| ≤ δ1 + δ2. Inside a ball around O where every
// tangent of γ1 makes an angle of at least θ_low (mod π) with every tangent
// of γ2, Q1 − O is a sum of γ1's tangents and Q2 − O of γ2's, so
// |Q1 − Q2| ≥ max(|Q1 − O|, |Q2 − O|)·sin θ_low. Hence
// |P* − O| ≤ (δ1 + δ2)/sin θ_low + min(δ1, δ2). sinLower bounds sin θ_low
// over a ball around the recorded junction point, and CrossingCharge widens
// that ball until it holds the whole distance it charges.

// CrossingCharge is the largest crossing displacement over every junction
// of every returned cell where the arrangement cut a carrier and two
// distinct entities meet. deltaA and deltaB are each operand's incoming
// coordinate displacement (§7's δ_A + δ_walkA and δ_B + δ_walkB +
// δ_reexpress). Both zero charges nothing and reads no junction.
//
// It returns decaderr.ErrUnsupported when a crossing's certified sin θ_low
// is not positive or falls below sectionaudit.ContactEps: a near-tangent
// crossing amplifies an input displacement by an amount this charge cannot
// bound. That check only refuses. A recorded edge sketch reports uncertified
// is sketchrecord.RecordEdge's own refusal.
func CrossingCharge(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, profiles []*sketch.Profile, deltaA, deltaB float64) (float64, error) {
	if deltaA == 0 && deltaB == 0 {
		return 0, nil
	}
	charge := 0.0
	for _, p := range profiles {
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for i, e1 := range loop {
				if err := budget.Step(); err != nil {
					return 0, err
				}
				e2 := loop[(i+1)%len(loop)]
				if e1.Entity == e2.Entity {
					continue
				}
				c, err := junctionCharge(tags, e1, e2, deltaA, deltaB)
				if err != nil {
					return 0, err
				}
				charge = math.Max(charge, c)
			}
		}
	}
	return charge, nil
}

// junctionCharge is CrossingCharge for the junction where e1's walk ends and
// e2's walk starts. A junction where neither side was cut is two recorded
// vertices meeting, which the arrangement computed nothing for, and charges
// nothing here.
func junctionCharge(tags map[sketch.Entity]Origin, e1, e2 sketch.BoundaryEdge, deltaA, deltaB float64) (float64, error) {
	seg1, err := sketchrecord.RecordEdge(e1)
	if err != nil {
		return 0, err
	}
	seg2, err := sketchrecord.RecordEdge(e2)
	if err != nil {
		return 0, err
	}
	end1, err := walkEndParam(seg1)
	if err != nil {
		return 0, err
	}
	start2, err := walkStartParam(seg2)
	if err != nil {
		return 0, err
	}
	if !cutAt(e1, seg1, end1) && !cutAt(e2, seg2, start2) {
		return 0, nil
	}
	o1, ok1 := tags[e1.Entity]
	o2, ok2 := tags[e2.Entity]
	if !ok1 || !ok2 {
		return 0, fmt.Errorf(`decad: a crossing junction traces to an entity the scene did not create`)
	}
	d1, d2 := deltaA, deltaA
	if o1.IsB {
		d1 = deltaB
	}
	if o2.IsB {
		d2 = deltaB
	}
	if d1 == 0 && d2 == 0 {
		return 0, nil
	}

	pt, pointErr, err := junctionPoint(e1, seg1, end1, e2, seg2, start2)
	if err != nil {
		return 0, err
	}
	reach := proofbound.AbsSumUpper(d1, d2)

	// sin θ_low over the ball that holds the recorded crossing O, then over a
	// ball twice the distance that first bound charges; the second bound is
	// accepted only when the distance it charges fits the ball it read.
	s0, err := sinLower(seg1, seg2, pt, pointErr)
	if err != nil {
		return 0, err
	}
	if err := refuseNearTangent(s0); err != nil {
		return 0, err
	}
	far0 := proofbound.DivUpper(reach, s0)
	rho := proofbound.AbsSumUpper(pointErr, far0, far0)
	s1, err := sinLower(seg1, seg2, pt, rho)
	if err != nil {
		return 0, err
	}
	if err := refuseNearTangent(s1); err != nil {
		return 0, err
	}
	far1 := proofbound.DivUpper(reach, s1)
	if far1 > 2*far0 || math.IsInf(far1, 1) {
		return 0, fmt.Errorf(`%w: a crossing's angle bound does not settle within the region it charges (sin θ ≥ %g, then %g)`,
			decaderr.ErrUnsupported, s0, s1)
	}
	return proofbound.AbsSumUpper(far1, math.Min(d1, d2)), nil
}

// refuseNearTangent is A6's noise-floor refusal: a crossing whose certified
// sin θ_low is not above ε = sectionaudit.ContactEps
// (docs/verification-design.md §4's floor for a dimensionless quantity).
func refuseNearTangent(s float64) error {
	if s > sectionaudit.ContactEps {
		return nil
	}
	return fmt.Errorf(`%w: two carriers cross at an angle whose proven sine (%g) is not above the noise floor %g, so an input displacement moves the crossing by an amount this evaluator cannot bound; the pair is too close to tangent`,
		decaderr.ErrUnsupported, s, sectionaudit.ContactEps)
}

// walkEndParam and walkStartParam are the recorded parameters a segment's
// walk ends and starts at (record.go: the walk runs TStart → TEnd).
func walkEndParam(seg CurveSegment) (float64, error) {
	_, t1, err := SegmentParamRange(seg)
	return t1, err
}

func walkStartParam(seg CurveSegment) (float64, error) {
	t0, _, err := SegmentParamRange(seg)
	return t0, err
}

// cutAt reports whether the arrangement computed the parameter t at which
// e's walk meets the junction. A whole edge computed nothing. A circle has
// no vertex, so every end of a Partial circle edge is a cut; a line or arc
// end is a cut unless it sits on its entity's own recorded bound.
func cutAt(e sketch.BoundaryEdge, seg CurveSegment, t float64) bool {
	if !e.Partial {
		return false
	}
	if _, circle := seg.(CircleSeg); circle {
		return true
	}
	return t != 0 && t != 1
}

// junctionPoint is a point P̂ held exactly with a proven bound on its
// distance from the crossing O the two recorded carriers state. A line side
// gives P̂ as the exact rational point at its recorded parameter, within its
// own cut allowance (CutDelta) of O; with no line side, the first circular
// walk's float endpoint stands in, adding the walk's own endpoint bound.
func junctionPoint(e1 sketch.BoundaryEdge, seg1 CurveSegment, end1 float64, e2 sketch.BoundaryEdge, seg2 CurveSegment, start2 float64) ([2]*big.Rat, float64, error) {
	if l, ok := seg1.(LineSeg); ok {
		return lineJunctionPoint(e1, l, end1)
	}
	if l, ok := seg2.(LineSeg); ok {
		return lineJunctionPoint(e2, l, start2)
	}
	w, err := boundarywalk.WalkOf(seg1, nil)
	if err != nil {
		return [2]*big.Rat{}, 0, err
	}
	u, v := proofarith.FloatRat(w.EndU), proofarith.FloatRat(w.EndV)
	if u == nil || v == nil {
		return [2]*big.Rat{}, 0, fmt.Errorf(`%w: a crossing junction's walked point is not finite`, decaderr.ErrUnsupported)
	}
	cut, err := CutDelta(e1, seg1)
	if err != nil {
		return [2]*big.Rat{}, 0, err
	}
	return [2]*big.Rat{u, v}, proofbound.AbsSumUpper(cut, proofbound.WalkEndBoundAllow(w.EndBound)), nil
}

func lineJunctionPoint(e sketch.BoundaryEdge, l LineSeg, t float64) ([2]*big.Rat, float64, error) {
	tr := proofarith.FloatRat(t)
	su, sv := proofarith.FloatRat(l.Start.U), proofarith.FloatRat(l.Start.V)
	eu, ev := proofarith.FloatRat(l.End.U), proofarith.FloatRat(l.End.V)
	if tr == nil || su == nil || sv == nil || eu == nil || ev == nil {
		return [2]*big.Rat{}, 0, fmt.Errorf(`%w: a crossing junction's line is not finite`, decaderr.ErrUnsupported)
	}
	lerp := func(a, b *big.Rat) *big.Rat {
		d := new(big.Rat).Sub(b, a)
		return d.Add(a, d.Mul(d, tr))
	}
	cut, err := CutDelta(e, l)
	if err != nil {
		return [2]*big.Rat{}, 0, err
	}
	return [2]*big.Rat{lerp(su, eu), lerp(sv, ev)}, cut, nil
}

// carrier is one recorded carrier as sinLower reads it: a line's exact
// direction, or a circle's exact centre and squared radius.
type carrier struct {
	line       bool
	du, dv     *big.Rat // line: End − Start
	cu, cv, r2 *big.Rat // circle or arc: centre and squared radius
}

func carrierOf(seg CurveSegment) (carrier, error) {
	rat := func(fs ...float64) ([]*big.Rat, bool) {
		out := make([]*big.Rat, len(fs))
		for i, f := range fs {
			out[i] = proofarith.FloatRat(f)
			if out[i] == nil {
				return nil, false
			}
		}
		return out, true
	}
	switch s := seg.(type) {
	case LineSeg:
		v, ok := rat(s.Start.U, s.Start.V, s.End.U, s.End.V)
		if !ok {
			break
		}
		return carrier{line: true, du: new(big.Rat).Sub(v[2], v[0]), dv: new(big.Rat).Sub(v[3], v[1])}, nil
	case CircleSeg:
		r, err := s.Radius.In(units.Millimeter)
		if err != nil {
			return carrier{}, fmt.Errorf(`decad: a crossing circle's radius is not a length: %w`, err)
		}
		v, ok := rat(s.Center.U, s.Center.V, r)
		if !ok {
			break
		}
		return carrier{cu: v[0], cv: v[1], r2: new(big.Rat).Mul(v[2], v[2])}, nil
	case ArcSeg:
		v, ok := rat(s.Center.U, s.Center.V, s.Start.U, s.Start.V)
		if !ok {
			break
		}
		du, dv := new(big.Rat).Sub(v[2], v[0]), new(big.Rat).Sub(v[3], v[1])
		return carrier{cu: v[0], cv: v[1], r2: dot(du, dv, du, dv)}, nil
	default:
		return carrier{}, fmt.Errorf(`%w: a %T carrier has no crossing angle this evaluator states`, decaderr.ErrUnsupported, seg)
	}
	return carrier{}, fmt.Errorf(`%w: a crossing carrier is not finite`, decaderr.ErrUnsupported)
}

func dot(au, av, bu, bv *big.Rat) *big.Rat {
	x := new(big.Rat).Mul(au, bu)
	return x.Add(x, new(big.Rat).Mul(av, bv))
}

func cross(au, av, bu, bv *big.Rat) *big.Rat {
	x := new(big.Rat).Mul(au, bv)
	return x.Sub(x, new(big.Rat).Mul(av, bu))
}

// sinLower is a proven lower bound on |sin| of the angle between any tangent
// of seg1's carrier and any tangent of seg2's carrier at points within rho of
// pt. Zero means no positive bound was proven.
//
//   - Two lines: |d1 × d2| / (|d1| |d2|), constant along both.
//   - A line d and a circle of centre C and radius R: a circle's tangent is
//     perpendicular to its radius P − C, so the sine is |d·(P − C)|/(|d| R),
//     and |d·(P − C)| ≥ |d·(pt − C)| − |d| rho.
//   - Two circles: |(P1 − C1) × (P2 − C2)|/(R1 R2), with each P − C within
//     rho of pt − C.
//
// A circle read over a ball wider than half its radius answers zero: its
// tangents there could turn through a half turn, past which the sum of
// tangents CrossingCharge's argument takes is no longer bounded by the
// extreme tangents alone.
//
// Every product and difference is exact over big.Rat; only the square roots
// round, each in the direction that lowers the bound.
func sinLower(seg1, seg2 CurveSegment, pt [2]*big.Rat, rho float64) (float64, error) {
	c1, err := carrierOf(seg1)
	if err != nil {
		return 0, err
	}
	c2, err := carrierOf(seg2)
	if err != nil {
		return 0, err
	}
	if math.IsInf(rho, 0) || math.IsNaN(rho) {
		return 0, nil
	}
	rhoR := proofarith.FloatRat(rho)
	for _, c := range []carrier{c1, c2} {
		if c.line {
			continue
		}
		// rho ≤ R/2  ⇔  4·rho² ≤ R²
		lim := new(big.Rat).Mul(rhoR, rhoR)
		lim.Mul(lim, big.NewRat(4, 1))
		if lim.Cmp(c.r2) > 0 {
			return 0, nil
		}
	}
	if !c1.line && c2.line {
		c1, c2 = c2, c1
	}
	var num *big.Rat // a proven lower bound on the numerator
	var den float64  // a proven upper bound on the denominator
	switch {
	case c1.line && c2.line:
		num = new(big.Rat).Abs(cross(c1.du, c1.dv, c2.du, c2.dv))
		den = proofbound.RatSqrtUp(new(big.Rat).Mul(dot(c1.du, c1.dv, c1.du, c1.dv), dot(c2.du, c2.dv, c2.du, c2.dv)))
	case c1.line:
		au, av := new(big.Rat).Sub(pt[0], c2.cu), new(big.Rat).Sub(pt[1], c2.cv)
		dLen := proofbound.RatSqrtUp(dot(c1.du, c1.dv, c1.du, c1.dv))
		dLenR := proofarith.FloatRat(dLen)
		if dLenR == nil || dLenR.Sign() == 0 {
			return 0, nil
		}
		// (|d·a| − |d| rho) with |d| taken at its upper bound, which only
		// lowers the difference.
		num = new(big.Rat).Abs(dot(c1.du, c1.dv, au, av))
		num.Sub(num, new(big.Rat).Mul(dLenR, rhoR))
		den = proofbound.ProductUpper(dLen, proofbound.RatSqrtUp(c2.r2))
	default:
		a1u, a1v := new(big.Rat).Sub(pt[0], c1.cu), new(big.Rat).Sub(pt[1], c1.cv)
		a2u, a2v := new(big.Rat).Sub(pt[0], c2.cu), new(big.Rat).Sub(pt[1], c2.cv)
		a1 := proofarith.FloatRat(proofbound.RatSqrtUp(dot(a1u, a1v, a1u, a1v)))
		a2 := proofarith.FloatRat(proofbound.RatSqrtUp(dot(a2u, a2v, a2u, a2v)))
		if a1 == nil || a2 == nil {
			return 0, nil
		}
		// |r1 × r2| ≥ |a1 × a2| − rho(|a1| + |a2|) − rho².
		num = new(big.Rat).Abs(cross(a1u, a1v, a2u, a2v))
		num.Sub(num, new(big.Rat).Mul(rhoR, new(big.Rat).Add(a1, a2)))
		num.Sub(num, new(big.Rat).Mul(rhoR, rhoR))
		den = proofbound.ProductUpper(proofbound.RatSqrtUp(c1.r2), proofbound.RatSqrtUp(c2.r2))
	}
	if num.Sign() <= 0 || math.IsInf(den, 0) || den <= 0 {
		return 0, nil
	}
	denR := proofarith.FloatRat(den)
	return proofbound.RatFloatDown(num.Quo(num, denR)), nil
}
