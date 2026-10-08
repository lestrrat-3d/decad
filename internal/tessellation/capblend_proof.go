package tessellation

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// CapBlendLoopProof is the numeric part of one resolved cap-blend loop.
// Its slices share the caller's walk and sample order without copying them.
type CapBlendLoopProof struct {
	Walks                     []survey2d.SideWalk
	Count                     []int
	SideSag, CapSag           []float64
	CapRadius, CapTh0, CapTh1 []float64
	ArcCount                  []int
	ArcSag, ArcTh0, ArcTh1    []float64
	ZLo, ZHi                  proofbound.BoundedScalar
	Chamfered, OnStart, OnEnd bool
}

// CapBlendChordVolume bounds the slice-wise circular-segment volume between
// the denoted body and the ideal chord polyhedron. bandHeight holds a proven
// upper bound on the start and the end cap's own side setback ds, the axial
// extent of every band on that cap.
func CapBlendChordVolume(bandHeight [2]float64, loops []CapBlendLoopProof) float64 {
	total := 0.0
	for li := range loops {
		lm := &loops[li]
		trim := proofbound.BoundedSub(lm.ZHi, lm.ZLo)
		hTrimUpper := proofbound.AbsSumUpper(math.Abs(trim.Value), trim.Bound)
		sideSegs, bandSegs := 0.0, 0.0
		for i, w := range lm.Walks {
			if !w.IsCircular() {
				continue
			}
			sideSegs = proofbound.AbsSumUpper(sideSegs, WalkSegmentArea(w.SegmentWalk, lm.Count[i]))
			if !lm.Chamfered {
				continue
			}
			bandSegs = proofbound.AbsSumUpper(bandSegs, WalkSegmentArea(survey2d.SegmentWalk{
				Kind: survey2d.WalkCircular, Radius: math.Max(w.Radius, lm.CapRadius[i]),
				Th0: w.Th0, Th1: w.Th1, Closed: w.Closed,
			}, lm.Count[i]))
		}
		total = proofbound.AbsSumUpper(total, proofbound.ProductUpper(hTrimUpper, sideSegs))
		for c, chamfered := range []bool{lm.OnStart, lm.OnEnd} {
			if chamfered {
				total = proofbound.AbsSumUpper(total, proofbound.ProductUpper(bandHeight[c], bandSegs))
			}
		}
	}
	return total
}

// CapBlendRingSagitta is the largest side or contour sagitta on one loop.
func CapBlendRingSagitta(lm CapBlendLoopProof, contour bool) float64 {
	worst := 0.0
	for i := range lm.Walks {
		s := lm.SideSag[i]
		if contour {
			s = lm.CapSag[i]
		}
		worst = math.Max(worst, s)
	}
	if !contour {
		return worst
	}
	for i := range lm.ArcSag {
		worst = math.Max(worst, lm.ArcSag[i])
	}
	return worst
}

// CapBlendRingSegmentArea sums the circular segments omitted by one cap ring.
func CapBlendRingSegmentArea(lm CapBlendLoopProof, contour bool, d float64) float64 {
	total := 0.0
	for i, w := range lm.Walks {
		if !w.IsCircular() {
			continue
		}
		if !contour {
			total = proofbound.AbsSumUpper(total, WalkSegmentArea(w.SegmentWalk, lm.Count[i]))
			continue
		}
		total = proofbound.AbsSumUpper(total, WalkSegmentArea(survey2d.SegmentWalk{
			Kind: survey2d.WalkCircular, Radius: lm.CapRadius[i],
			Th0: lm.CapTh0[i], Th1: lm.CapTh1[i], Closed: w.Closed,
		}, lm.Count[i]))
	}
	if !contour {
		return total
	}
	for i, count := range lm.ArcCount {
		if count == 0 {
			continue
		}
		total = proofbound.AbsSumUpper(total, WalkSegmentArea(survey2d.SegmentWalk{
			Kind: survey2d.WalkCircular, Radius: d, Th0: lm.ArcTh0[i], Th1: lm.ArcTh1[i],
		}, count))
	}
	return total
}

// CapBlendMotionInput names the exact side-window station of each cap sample.
// D is the loop's in-plane setback and DDelta the rounding its unit
// conversion committed, zero for a setback stated in millimetres.
type CapBlendMotionInput struct {
	CapBlendLoopProof
	Loop         int
	Segments     []sectionrecord.CurveSegment
	CapPts       []sectionrecord.Point2
	CapWallStart []int
	Whole        bool
	D, DDelta    float64
	BandDelta    [2]float64
	HasBandDelta [2]bool
}

// CapBlendCapMotion bounds each cap sample against the ideal chord
// polyhedron's matching point. Reflex connectors retain an infinite bound.
func CapBlendCapMotion(budget *proofbound.WorkBudget, in CapBlendMotionInput,
	stationBound func(sectionrecord.CurveSegment, int, int, *big.Rat, float64, float64) proofbound.WalkEndBound,
) ([]float64, error) {
	capMotion := make([]float64, len(in.CapPts))
	for j := range capMotion {
		capMotion[j] = math.Inf(1)
	}
	if !in.Chamfered {
		return capMotion, nil
	}
	n := len(in.Walks)
	offset := func(w survey2d.SideWalk) *big.Rat { return capcontour.CapWallRadiusOffset(w, in.D) }
	// A circular wall's sample is bounded against the offset circle at D. The
	// denoted circle sits within DDelta of it radially, so a setback whose
	// unit conversion rounded charges that on top.
	circular := func(b proofbound.WalkEndBound) float64 {
		allow := proofbound.WalkEndBoundAllow(b)
		if in.DDelta > 0 {
			allow = proofbound.AbsSumUpper(allow, in.DDelta)
		}
		return allow
	}
	if in.Whole {
		w := in.Walks[0]
		seg := in.Segments[w.Segs[0]]
		off := offset(w)
		for k := range in.Count[0] {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			p := in.CapPts[k]
			capMotion[k] = circular(stationBound(seg, k, in.Count[0], off, p.U, p.V))
		}
		return capMotion, nil
	}
	// The contour displacement bounds each line-line miter at either cap.
	miter := 0.0
	for _, start := range []bool{true, false} {
		if (start && !in.OnStart) || (!start && !in.OnEnd) {
			continue
		}
		idx := 1
		if start {
			idx = 0
		}
		delta, ok := in.BandDelta[idx], in.HasBandDelta[idx]
		if !ok {
			return nil, fmt.Errorf(`%w: the payload states no contour displacement for the chamfer band on loop %d`, decaderr.ErrDegenerate, in.Loop)
		}
		miter = math.Max(miter, delta)
	}
	for i, w := range in.Walks {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		base := in.CapWallStart[i]
		if !w.IsCircular() {
			prev := in.Walks[(i+n-1)%n]
			if !prev.IsCircular() {
				capMotion[base] = miter
				continue
			}
			p := in.CapPts[base]
			prevIdx := (i + n - 1) % n
			prevSeg := in.Segments[prev.Segs[0]]
			cnt := in.Count[prevIdx]
			capMotion[base] = circular(stationBound(prevSeg, cnt, cnt, offset(prev), p.U, p.V))
			continue
		}
		seg := in.Segments[w.Segs[0]]
		off := offset(w)
		for k := range in.Count[i] {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			p := in.CapPts[base+k]
			capMotion[base+k] = circular(stationBound(seg, k, in.Count[i], off, p.U, p.V))
		}
	}
	return capMotion, nil
}

// CapBlendNextCapSample selects a connector's first cap station when present.
func CapBlendNextCapSample(arcStart, wallStart []int, i int) int {
	if arcStart != nil && arcStart[i] >= 0 {
		return arcStart[i]
	}
	return wallStart[i]
}

// CapBlendTwiceAreaSq is the exact squared length of (b-a)×(c-a).
func CapBlendTwiceAreaSq(a, b, c r3.Vec) *big.Rat {
	sub := func(p, q r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{
			new(big.Rat).Sub(proofarith.FloatRat(p.X), proofarith.FloatRat(q.X)),
			new(big.Rat).Sub(proofarith.FloatRat(p.Y), proofarith.FloatRat(q.Y)),
			new(big.Rat).Sub(proofarith.FloatRat(p.Z), proofarith.FloatRat(q.Z)),
		}
	}
	u, v := sub(b, a), sub(c, a)
	cross := [3]*big.Rat{
		new(big.Rat).Sub(new(big.Rat).Mul(u[1], v[2]), new(big.Rat).Mul(u[2], v[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(u[2], v[0]), new(big.Rat).Mul(u[0], v[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(u[0], v[1]), new(big.Rat).Mul(u[1], v[0])),
	}
	out := new(big.Rat)
	for _, k := range cross {
		out.Add(out, new(big.Rat).Mul(k, k))
	}
	return out
}
