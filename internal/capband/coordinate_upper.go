package capband

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// CoordUpper bounds every coordinate magnitude of a band's region: the
// larger of the original loop's and cap contour's in-plane coordinates
// (widening the latter by delta), and the two bounded axial levels. Each
// section offset between the loops includes its corner foot locus, so the
// same envelope covers every point in the band's region. The loop readings
// use each arc's swept extent rather than its whole circle.
func CoordUpper(loop, capBoundary sectionrecord.LoopRecord, delta float64,
	sideZB, capZB proofbound.BoundedScalar, work *freeform.FreeformWork) (float64, error) {
	coordUpper, err := loopLocalCoordinateUpper(loop, work)
	if err != nil {
		return 0, err
	}
	capCoordUpper, err := loopLocalCoordinateUpper(capBoundary, work)
	if err != nil {
		return 0, err
	}
	coordUpper = math.Max(coordUpper, proofbound.AbsSumUpper(capCoordUpper, delta))
	coordUpper = math.Max(coordUpper, proofbound.AbsSumUpper(math.Abs(sideZB.Value), sideZB.Bound))
	return math.Max(coordUpper, proofbound.AbsSumUpper(math.Abs(capZB.Value), capZB.Bound)), nil
}

// PointUpper bounds |P| = |(u, v, z)| over a band and its closure slivers.
// It reads the original and cap loops' L1 envelopes, widens the latter by
// contour displacement, then includes both levels and the closure's reach.
func PointUpper(loop, capBoundary sectionrecord.LoopRecord, delta float64, closure Closure,
	sideZB, capZB proofbound.BoundedScalar, work *freeform.FreeformWork) (float64, error) {
	coordUpper, err := momentinput.CoordinateUpper(momentinput.Profile{Outer: loop}, work, nil)
	if err != nil {
		return 0, err
	}
	capCoordUpper, err := momentinput.CoordinateUpper(momentinput.Profile{Outer: capBoundary}, work, nil)
	if err != nil {
		return 0, err
	}
	planeUpper := math.Max(coordUpper, proofbound.AbsSumUpper(capCoordUpper, delta))
	zUpper := math.Max(proofbound.AbsSumUpper(sideZB.Value, sideZB.Bound),
		proofbound.AbsSumUpper(capZB.Value, capZB.Bound))
	return proofbound.AbsSumUpper(planeUpper, zUpper, closure.Reach), nil
}

// loopLocalCoordinateUpper bounds max(|u|, |v|) over every point of a loop.
// For each segment it takes the smaller of the walk's L1 envelope
// (momentinput.WalkCoordinateUpper, which dominates max(|u|, |v|) and is the
// reading a whole circle gets) and SegmentCoordinateUpper's tighter recorded
// line or arc extent.
func loopLocalCoordinateUpper(loop sectionrecord.LoopRecord, work *freeform.FreeformWork) (float64, error) {
	upper := 0.0
	for _, seg := range loop.Segments {
		w, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return 0, err
		}
		segUpper := momentinput.WalkCoordinateUpper(w)
		if local, ok := SegmentCoordinateUpper(seg); ok {
			segUpper = math.Min(segUpper, local)
		}
		upper = math.Max(upper, segUpper)
	}
	return upper, nil
}

// SegmentCoordinateUpper bounds max(|u|, |v|) over a line or arc segment
// whose parameter range lies in its entity's [0, 1]. A line is bounded by its
// endpoints because each point is a convex combination of them. An arc's
// coordinate is monotone between axis directions, so its start, terminal
// point and each crossed axis direction bound it. Exact cross-product signs
// decide which axes it crosses. RatSqrtDown and RatSqrtUp bracket the radius
// and the terminal direction's length; interval operations round outward.
// A partial range lies inside the entity's full sweep. ok is false for other
// kinds, ranges outside [0, 1], and degenerate arcs.
func SegmentCoordinateUpper(seg sectionrecord.CurveSegment) (float64, bool) {
	inUnit := func(t0, t1 float64) bool { return t0 >= 0 && t0 <= 1 && t1 >= 0 && t1 <= 1 }
	switch sg := seg.(type) {
	case sectionrecord.LineSeg:
		if !inUnit(sg.TStart, sg.TEnd) {
			return 0, false
		}
		return math.Max(math.Max(math.Abs(sg.Start.U), math.Abs(sg.Start.V)), math.Max(math.Abs(sg.End.U), math.Abs(sg.End.V))), true
	case sectionrecord.ArcSeg:
		if !inUnit(sg.TStart, sg.TEnd) {
			return 0, false
		}
		return arcCoordinateUpper(sg)
	}
	return 0, false
}

// arcCoordinateUpper encloses the terminal point c + R·(End-c)/|End-c| and
// every axis direction the recorded counter-clockwise arc crosses.
func arcCoordinateUpper(sg sectionrecord.ArcSeg) (float64, bool) {
	cu, cv := proofarith.FloatRat(sg.Center.U), proofarith.FloatRat(sg.Center.V)
	su, sv := proofarith.FloatRat(sg.Start.U), proofarith.FloatRat(sg.Start.V)
	eu, ev := proofarith.FloatRat(sg.End.U), proofarith.FloatRat(sg.End.V)
	if cu == nil || cv == nil || su == nil || sv == nil || eu == nil || ev == nil {
		return 0, false
	}
	au, av := new(big.Rat).Sub(su, cu), new(big.Rat).Sub(sv, cv)
	bu, bv := new(big.Rat).Sub(eu, cu), new(big.Rat).Sub(ev, cv)
	sq := func(x, y *big.Rat) *big.Rat { return new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y)) }
	aa, bb := sq(au, av), sq(bu, bv)
	if aa.Sign() == 0 || bb.Sign() == 0 {
		return 0, false
	}
	rLo, rHi := proofarith.FloatRat(proofbound.RatSqrtDown(aa)), proofarith.FloatRat(proofbound.RatSqrtUp(aa))
	bLo, bHi := proofarith.FloatRat(proofbound.RatSqrtDown(bb)), proofarith.FloatRat(proofbound.RatSqrtUp(bb))
	if rLo == nil || rHi == nil || bLo == nil || bHi == nil || bLo.Sign() <= 0 {
		return 0, false
	}
	upper := math.Max(math.Abs(sg.Start.U), math.Abs(sg.Start.V))
	// The terminal point is enclosed component by component from the
	// brackets of R and |End-c|.
	radius := proofbound.Interval(rLo, rHi)
	for _, k := range [][2]*big.Rat{{cu, bu}, {cv, bv}} {
		comp, ok := proofbound.IntervalQuo(proofbound.IntervalScale(radius, k[1]), proofbound.Interval(bLo, bHi))
		if !ok {
			return 0, false
		}
		comp = proofbound.IntervalAdd(comp, proofbound.PointInterval(k[0]))
		upper = math.Max(upper, proofbound.RatFloatUp(proofbound.IntervalAbsUpper(comp)))
	}
	cross := func(xu, xv, yu, yv *big.Rat) int {
		return new(big.Rat).Sub(new(big.Rat).Mul(xu, yv), new(big.Rat).Mul(xv, yu)).Sign()
	}
	crossAB := cross(au, av, bu, bv)
	dotAB := new(big.Rat).Add(new(big.Rat).Mul(au, bu), new(big.Rat).Mul(av, bv)).Sign()
	one, zero, minus := big.NewRat(1, 1), new(big.Rat), big.NewRat(-1, 1)
	for _, d := range [][2]*big.Rat{{one, zero}, {zero, one}, {minus, zero}, {zero, minus}} {
		fromA, toB := cross(au, av, d[0], d[1]) >= 0, cross(d[0], d[1], bu, bv) >= 0
		var passes bool
		switch {
		case crossAB == 0 && dotAB > 0:
			passes = true // a whole turn
		case crossAB > 0:
			passes = fromA && toB
		default:
			passes = fromA || toB
		}
		if !passes {
			continue
		}
		// The axis point is c + R·d: its nonzero component is c_k ± R.
		c := cu
		sign := d[0]
		if d[0].Sign() == 0 {
			c, sign = cv, d[1]
		}
		for _, r := range []*big.Rat{rLo, rHi} {
			v := new(big.Rat).Add(c, new(big.Rat).Mul(sign, r))
			upper = math.Max(upper, proofbound.RatFloatUp(v.Abs(v)))
		}
		// The other component is c's own, which the ends need not reach.
		other := cv
		if d[0].Sign() == 0 {
			other = cu
		}
		upper = math.Max(upper, proofbound.RatFloatUp(new(big.Rat).Abs(other)))
	}
	return upper, true
}
