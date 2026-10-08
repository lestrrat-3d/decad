package tessellation

import (
	"math"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// CapBlendJoin is the sampled part of a resolved cap-contour corner.
type CapBlendJoin struct {
	Arc       bool
	VU, VV    float64
	M, PA, PB sectionrecord.Point2
}

// CapBlendSampleInput holds one loop's shared side and cap chord counts.
type CapBlendSampleInput struct {
	Walks                     []survey2d.SideWalk
	Segments                  []sectionrecord.CurveSegment
	Joins                     []CapBlendJoin
	Count                     []int
	CapRadius, CapTh0, CapTh1 []float64
	ArcCount                  []int
	ArcTh0, ArcTh1            []float64
	Whole, Chamfered          bool
	D                         float64
}

// CapBlendSamples records both rings and each walk or connector's first sample.
type CapBlendSamples struct {
	SidePts, CapPts                      []sectionrecord.Point2
	SideBound, CapBound                  []proofbound.WalkEndBound
	SideStart, CapWallStart, CapArcStart []int
}

// SampleCapBlend emits each piece's start and leaves its end to the next piece.
// The shared count on a circular wall places its side and cap rings at matching
// fractions of their windows. The caller supplies the recorded-curve enclosure
// of interior side stations. A cap wall's first station uses its corner foot
// verbatim so adjacent strips share one held vertex.
func SampleCapBlend(in CapBlendSampleInput, budget *proofbound.WorkBudget,
	stationBound func(sectionrecord.CurveSegment, int, int, float64, float64) proofbound.WalkEndBound,
) (CapBlendSamples, error) {
	var out CapBlendSamples
	n := len(in.Walks)
	out.SideStart = make([]int, n)
	for i, w := range in.Walks {
		out.SideStart[i] = len(out.SidePts)
		// The side ring's junction sample reaches the points both
		// neighbours denote there, as SampleLoop's does.
		start := sampleStartBound(in.Walks, in.Segments, i)
		if !w.IsCircular() {
			out.SidePts = append(out.SidePts, sectionrecord.Point2{U: w.StartU, V: w.StartV})
			out.SideBound = append(out.SideBound, start)
			continue
		}
		seg := in.Segments[w.Segs[0]]
		count := in.Count[i]
		dth := (w.Th1 - w.Th0) / float64(count)
		for k := range count {
			if err := budget.Step(); err != nil {
				return CapBlendSamples{}, err
			}
			p := sectionrecord.Point2{U: w.StartU, V: w.StartV}
			bound := start
			if k > 0 {
				th := w.Th0 + float64(k)*dth
				p = sectionrecord.Point2{U: w.CU + w.Radius*math.Cos(th), V: w.CV + w.Radius*math.Sin(th)}
				bound = stationBound(seg, k, count, p.U, p.V)
			}
			out.SidePts = append(out.SidePts, p)
			out.SideBound = append(out.SideBound, bound)
		}
	}
	if !in.Chamfered {
		return out, nil
	}

	out.CapWallStart = make([]int, n)
	out.CapArcStart = make([]int, n)
	for i := range n {
		out.CapArcStart[i] = -1
	}
	addCap := func(p sectionrecord.Point2, bound proofbound.WalkEndBound) {
		out.CapPts = append(out.CapPts, p)
		out.CapBound = append(out.CapBound, bound)
	}
	if in.Whole {
		w := in.Walks[0]
		out.CapWallStart[0] = 0
		count := in.Count[0]
		dth := (in.CapTh1[0] - in.CapTh0[0]) / float64(count)
		for k := range count {
			if err := budget.Step(); err != nil {
				return CapBlendSamples{}, err
			}
			th := in.CapTh0[0] + float64(k)*dth
			p := sectionrecord.Point2{U: w.CU + in.CapRadius[0]*math.Cos(th), V: w.CV + in.CapRadius[0]*math.Sin(th)}
			addCap(p, CapStationBound(w.CU, w.CV, in.CapRadius[0], th, p.U, p.V))
		}
		return out, nil
	}

	for i, w := range in.Walks {
		out.CapWallStart[i] = len(out.CapPts)
		start := in.Joins[i].M
		if in.Joins[i].Arc {
			start = in.Joins[i].PB
		}
		if !w.IsCircular() {
			addCap(start, proofbound.WalkEndBound{})
		} else {
			count := in.Count[i]
			dth := (in.CapTh1[i] - in.CapTh0[i]) / float64(count)
			for k := range count {
				if err := budget.Step(); err != nil {
					return CapBlendSamples{}, err
				}
				th := in.CapTh0[i] + float64(k)*dth
				p := sectionrecord.Point2{U: w.CU + in.CapRadius[i]*math.Cos(th), V: w.CV + in.CapRadius[i]*math.Sin(th)}
				if k == 0 {
					p = start
				}
				addCap(p, CapStationBound(w.CU, w.CV, in.CapRadius[i], th, p.U, p.V))
			}
		}
		ni := (i + 1) % n
		j := in.Joins[ni]
		if !j.Arc {
			continue
		}
		out.CapArcStart[ni] = len(out.CapPts)
		count := in.ArcCount[ni]
		dth := (in.ArcTh1[ni] - in.ArcTh0[ni]) / float64(count)
		for k := range count {
			if err := budget.Step(); err != nil {
				return CapBlendSamples{}, err
			}
			th := in.ArcTh0[ni] + float64(k)*dth
			p := sectionrecord.Point2{U: j.VU + in.D*math.Cos(th), V: j.VV + in.D*math.Sin(th)}
			if k == 0 {
				p = j.PA
			}
			addCap(p, CapStationBound(j.VU, j.VV, in.D, th, p.U, p.V))
		}
	}
	return out, nil
}

// CapStationBound encloses the gap from a held cap sample to its offset circle
// station. The offset contour itself has a separate displacement allowance;
// this bound charges only construction of a station on the held circle.
// Non-finite input or an unenclosable angle returns an underivable bound.
func CapStationBound(cU, cV, radius, theta, heldU, heldV float64) proofbound.WalkEndBound {
	underivable := proofbound.WalkEndBound{U: math.Inf(1), V: math.Inf(1)}
	rt, rr := proofarith.FloatRat(theta), proofarith.FloatRat(radius)
	ru, rv := proofarith.FloatRat(cU), proofarith.FloatRat(cV)
	if rt == nil || rr == nil || ru == nil || rv == nil {
		return underivable
	}
	sin, cos, ok := proofbound.RadSinCosInterval(rt)
	if !ok {
		return underivable
	}
	uIv := proofbound.IntervalAdd(proofbound.PointInterval(ru), proofbound.IntervalMul(proofbound.PointInterval(rr), cos))
	vIv := proofbound.IntervalAdd(proofbound.PointInterval(rv), proofbound.IntervalMul(proofbound.PointInterval(rr), sin))
	return proofbound.WalkEndBound{U: proofbound.IntervalFloatError(uIv, heldU), V: proofbound.IntervalFloatError(vIv, heldV)}
}
