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
// below by exact rational arithmetic on the recorded carriers. A crossing
// whose bound is not above the dimensionless noise floor ε of
// docs/verification-design.md §4 (sectionaudit.ContactEps) has no charge;
// the caller then takes the mesh path.
//
// The argument, for one crossing O of recorded carriers γ1 and γ2 and its
// denoted twin P* (within δ1 of γ1 and δ2 of γ2): take Q1 on γ1 and Q2 on
// γ2 nearest P*, so |Q1 − Q2| ≤ δ1 + δ2. Inside a ball around O where every
// tangent of γ1 makes an angle of at least θ_low (mod π) with every tangent
// of γ2, Q1 − O is a sum of γ1's tangents and Q2 − O of γ2's, so
// |Q1 − Q2| ≥ max(|Q1 − O|, |Q2 − O|)·sin θ_low. Hence
// |P* − O| ≤ (δ1 + δ2)/sin θ_low + min(δ1, δ2). sinLower bounds sin θ_low
// over a ball around the recorded junction point, and junctionCharge widens
// that ball until it holds the whole distance it charges.
//
// The charge is read per arranged VERTEX, not per pair of consecutive edges
// in a returned cell. sketch returns only the bounded cells, so a pair of
// edges adjacent around a vertex only through the unbounded face would never
// be walked consecutively by any returned loop. Every pair of distinct
// entities incident at a cut vertex is charged instead, whichever face walks
// them.
//
// A touch from outside has no vertex at all. When B's apex touches A's wall
// from outside, or B's edge passes exactly through A's corner from outside,
// sketch arranges the two outlines as separate cells with no cut and no
// shared vertex (TestCrossingChargeOutsideTouchIsNoCut), so nothing is
// charged, and nothing needs to be: a point the displacement moves across
// either boundary there lies within δ_B of B's recorded boundary, which lies
// outside A, so the segment to it crosses A's recorded wall within δ_B. Every
// such point therefore lies within δ_B of A's recorded wall, which bounds
// every selected region the touch borders, inside the tube of half-width
// sectionDelta ≥ δ_B that prism-boolean §7 already charges around the
// recorded boundary.

// CrossingCharge is the largest crossing displacement over every arranged
// vertex where the arrangement cut a carrier, taken over every pair of
// distinct entities incident there. Vertices are the walk ends sketch
// reports in each edge's Polyline, matched by exact equality, the same
// vertex identity ChainClosedSurvivors reads. deltaA and deltaB are each
// operand's incoming coordinate displacement (§7's δ_A + δ_walkA and
// δ_B + δ_walkB + δ_reexpress). Both zero charges nothing and reads nothing.
//
// ok=false (err always nil then) means a crossing has no charge: its
// certified sin θ_low is not above sectionaudit.ContactEps, its bound does
// not settle within the region it charges, or one of its edges does not
// record (an uncertified fragment). The caller falls back to the mesh path.
// A non-nil error is the budget's own (cancellation).
func CrossingCharge(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, profiles []*sketch.Profile, deltaA, deltaB float64) (float64, bool, error) {
	if deltaA == 0 && deltaB == 0 {
		return 0, true, nil
	}
	// Two operands' coincident lines sharing a span (CoincidentEdges) do not
	// cross: sketch resolved them as one line at round-off. Where the span
	// ends, a third line meets them — the corner of the operand whose line
	// ends there — and its pair with the other operand's line is charged.
	coincident, ok, err := CoincidentEdges(budget, tags, profiles)
	if err != nil || !ok {
		return 0, false, err
	}
	type endKey struct {
		entity sketch.Entity
		t      float64
	}
	vertices := map[[2]float64][]vertexEnd{}
	seen := map[endKey]struct{}{}
	for _, p := range profiles {
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return 0, false, err
				}
				if len(e.Polyline) == 0 {
					return 0, false, nil
				}
				seg, t0, t1, ok := recordedRange(e)
				if !ok {
					return 0, false, nil
				}
				for _, end := range []vertexEnd{
					{edge: e, seg: seg, t: t0, atStart: true},
					{edge: e, seg: seg, t: t1},
				} {
					k := endKey{entity: e.Entity, t: end.t}
					if _, dup := seen[k]; dup {
						continue
					}
					seen[k] = struct{}{}
					at := e.Polyline[len(e.Polyline)-1]
					if end.atStart {
						at = e.Polyline[0]
					}
					vertices[at] = append(vertices[at], end)
				}
			}
		}
	}
	charge := 0.0
	for _, ends := range vertices {
		cut := false
		for _, end := range ends {
			cut = cut || cutAt(end.edge, end.seg, end.t)
		}
		if !cut {
			continue // recorded vertices meeting: nothing here was computed
		}
		for i := range ends {
			for j := i + 1; j < len(ends); j++ {
				if err := budget.Step(); err != nil {
					return 0, false, err
				}
				if ends[i].edge.Entity == ends[j].edge.Entity || coincident.Partners(ends[i].edge.Entity, ends[j].edge.Entity) {
					continue
				}
				c, ok := junctionCharge(tags, ends[i], ends[j], deltaA, deltaB)
				if !ok {
					return 0, false, nil
				}
				charge = math.Max(charge, c)
			}
		}
	}
	return charge, true, nil
}

// recordedRange records e and reads its recorded parameter range. ok=false
// means the edge does not record (an uncertified fragment, which
// sketchrecord.RecordEdge refuses) and so has no charge.
func recordedRange(e sketch.BoundaryEdge) (CurveSegment, float64, float64, bool) {
	seg, err := sketchrecord.RecordEdge(e)
	if err != nil {
		return nil, 0, 0, false
	}
	t0, t1, err := SegmentParamRange(seg)
	return seg, t0, t1, err == nil
}

// vertexEnd is one edge's walk end at an arranged vertex: the edge, its
// recorded segment, the recorded parameter at that end, and which end.
type vertexEnd struct {
	edge    sketch.BoundaryEdge
	seg     CurveSegment
	t       float64
	atStart bool
}

// junctionCharge is the crossing displacement for two distinct entities
// meeting at one arranged vertex. ok=false means no charge could be proven.
func junctionCharge(tags map[sketch.Entity]Origin, a, b vertexEnd, deltaA, deltaB float64) (float64, bool) {
	o1, ok1 := tags[a.edge.Entity]
	o2, ok2 := tags[b.edge.Entity]
	if !ok1 || !ok2 {
		return 0, false
	}
	d1, d2 := deltaA, deltaA
	if o1.IsB {
		d1 = deltaB
	}
	if o2.IsB {
		d2 = deltaB
	}
	if d1 == 0 && d2 == 0 {
		return 0, true
	}

	pt, pointErr, err := junctionPoint(a, b)
	if err != nil {
		return 0, false
	}
	reach := proofbound.AbsSumUpper(d1, d2)

	// sin θ_low over the ball that holds the recorded crossing O, then over a
	// ball twice the distance that first bound charges; the second bound is
	// accepted only when the distance it charges fits the ball it read.
	s0, err := sinLower(a.seg, b.seg, pt, pointErr)
	if err != nil || !aboveNoiseFloor(s0) {
		return 0, false
	}
	far0 := proofbound.DivUpper(reach, s0)
	rho := proofbound.AbsSumUpper(pointErr, far0, far0)
	s1, err := sinLower(a.seg, b.seg, pt, rho)
	if err != nil || !aboveNoiseFloor(s1) {
		return 0, false
	}
	far1 := proofbound.DivUpper(reach, s1)
	if far1 > 2*far0 || math.IsInf(far1, 1) {
		return 0, false
	}
	return proofbound.AbsSumUpper(far1, math.Min(d1, d2)), true
}

// aboveNoiseFloor is A6's noise floor: a crossing's certified sin θ_low must
// be above ε = sectionaudit.ContactEps (docs/verification-design.md §4's
// floor for a dimensionless quantity) for its displacement to be charged.
func aboveNoiseFloor(s float64) bool {
	return s > sectionaudit.ContactEps
}

// cutAt reports whether the arrangement computed the parameter t at which
// e's walk meets the vertex. A whole edge computed nothing. A circle has no
// vertex, so every end of a Partial circle edge is a cut; a line or arc end
// is a cut unless it sits on its entity's own recorded bound.
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
// walk's float end at the vertex stands in, adding the walk's own endpoint
// bound.
func junctionPoint(a, b vertexEnd) ([2]*big.Rat, float64, error) {
	if l, ok := a.seg.(LineSeg); ok {
		return lineJunctionPoint(a.edge, l, a.t)
	}
	if l, ok := b.seg.(LineSeg); ok {
		return lineJunctionPoint(b.edge, l, b.t)
	}
	w, err := boundarywalk.WalkOf(a.seg, nil)
	if err != nil {
		return [2]*big.Rat{}, 0, err
	}
	pu, pv, bound := w.EndU, w.EndV, w.EndBound
	if a.atStart {
		pu, pv, bound = w.StartU, w.StartV, w.StartBound
	}
	u, v := proofarith.FloatRat(pu), proofarith.FloatRat(pv)
	if u == nil || v == nil {
		return [2]*big.Rat{}, 0, fmt.Errorf(`%w: a crossing junction's walked point is not finite`, decaderr.ErrUnsupported)
	}
	cut, err := CutDelta(a.edge, a.seg)
	if err != nil {
		return [2]*big.Rat{}, 0, err
	}
	return [2]*big.Rat{u, v}, proofbound.AbsSumUpper(cut, proofbound.WalkEndBoundAllow(bound)), nil
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
