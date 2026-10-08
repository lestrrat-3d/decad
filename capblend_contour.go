package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file connects the CAP CONTOUR's displacement proof in internal/capcontour
// to the cap-band construction and its refusal classes (modify-reach §8.3/§8.4).
//
// The band has two directrices and they are not the same kind of number. The
// SIDE contour is the receiver's own recorded loop, held at its own (u, v) and
// moved axially, so every coordinate of it is a float the record already
// carried. The CAP contour is not: each of its corner points comes out of
// shell_offset.go's float offset — a line/line, line/circle or circle/circle
// solve over directions normalize2 divided by a hypot — so it is a COMPUTED
// coordinate that sits some distance from the point the construction denotes.
// Nothing about that distance is recorded anywhere in the offset, and without
// it a cap-level vertex has no honest bound to publish and a cap-level edge has
// no honest length: a zero bound asserts an exactness the solve never had, and
// an infinite one bounds nothing at all.
//
// capContourDelta is that distance, proven. It is derived ONCE per band, from
// the band's own walks and joins, and every cap-level vertex, every cap-level
// edge length and the payload's own directional extent read it.
//
// The proof is an ENCLOSURE, not an error model: the same closed forms the
// float build evaluates are re-evaluated over math/big.Rat intervals, with the
// recorded coordinates taken EXACTLY and the only widening at the square
// roots, which round outward through proofbound.RatSqrtDown/proofbound.RatSqrtUp. Interval arithmetic
// is inclusion-monotonic, so the resulting box holds the exact point the same
// closed form denotes, whatever the platform's sqrt and hypot did; the
// displacement is then the box's own greatest reach from the float point the
// build actually holds. Nothing here assumes an ulp contract, and nothing here
// is a residual test that could ADMIT a point — it only ever states how far
// the held point may be from the denoted one.

// errCapContourUnbounded is the refusal for a corner whose denoted contour
// point this evaluator cannot enclose: two offset carriers whose interval
// intersection is unbounded (a near-parallel miter whose determinant straddles
// zero) or whose exact carriers do not meet at all. The requested body exists —
// the corner has a real offset — and only this evaluator cannot state where it
// is, which is docs/modify-reach-design.md §4's ErrUnsupported side of the
// existence test.
//
// The float construction refuses first in every configuration reached so far:
// intersectOffsets rejects a determinant at or below filletTol (1e-9), while
// the interval widths here are the relative rounding of unit directions, so the
// determinant interval cannot straddle zero once the float one cleared that
// floor, and a G1 join (modify §7) never reaches the carrier intersection. It stands
// as the honest answer for a configuration that does reach it rather than as a
// case the tests can exhibit.
var errCapContourUnbounded = fmt.Errorf(`%w: this evaluator cannot prove a bound on the cap-loop chamfer's own offset contour at a corner, so no cap-level coordinate it emits there can be published with a proven displacement`, ErrUnsupported)

// capContourDelta is ONE chamfer band's contour displacement: a proven upper
// bound on how far any cap-level point the band emits sits from the point the
// construction denotes. Every cap-level vertex bound, every cap-level edge
// length bound and the payload's own contour-held directional extent are
// charged this one number, so no reader can be told a different story about the
// same contour.
//
// The point the construction denotes is the section offset by the setback the
// caller STATED, and d is only the float the unit conversion rounded that
// setback to. dDelta is that rounding (capSetback.dcDelta), so every enclosure
// below is taken over the whole span of offset amounts [d − dDelta, d + dDelta]
// rather than at d alone: a setback stated in inches moves every corner of the
// contour, and a contour displacement read at d would publish the held corner
// as the denoted one. A setback stated in millimetres converts exactly, its span
// is the single point d, and every enclosure is the one d alone gives.
//
// It is a MAXIMUM over the contour's own pieces rather than a per-point figure
// because the band's readings mix them: a wall's cap edge runs between two
// corner feet, a patch quad holds two, and a survey walking the band reads
// them in one sequence. Charging each of them the band's worst is honest and
// leaves no piece under-bounded.
//
// The length VALUE a cap-level edge carries must stay the actual held float,
// never a stand-in such as +Inf for "unknown": selector.go's LongerThan
// predicate compares the raw e.length field directly, with no reference to
// Exactness or lengthBound, so an edge whose length field were ever widened
// to signal uncertainty would silently match every LongerThan query instead.
// Uncertainty belongs in lengthBound alone.
func capContourDelta(walks []survey2d.SideWalk, joins []cornerJoin, d, dDelta float64) (float64, error) {
	if _, ok := capcontour.OffsetSpan(d, dDelta); !ok {
		return 0, errCapContourUnbounded
	}
	for _, w := range walks {
		if w.IsCircular() {
			if _, err := capBandRadius(w, d); err != nil {
				return 0, err
			}
		}
	}
	delta, ok := capcontour.Displacement(walks, capContourJoins(joins), d, dDelta)
	if !ok {
		return 0, errCapContourUnbounded
	}
	return delta, nil
}

func capContourJoins(joins []cornerJoin) []capcontour.Join {
	readings := make([]capcontour.Join, len(joins))
	for i, j := range joins {
		readings[i] = capcontour.Join{
			Arc: j.arc, G1: j.g1, VU: j.vU, VV: j.vV,
			M: j.m, PA: j.pA, PB: j.pB,
		}
	}
	return readings
}

// capWholeCircleDelta is the cornerless closed circle's own contour
// displacement — the one shape with no corner join at all, whose whole contour
// is the concentric circle at the offset radius. dDelta is the setback's own
// unit-conversion rounding, read as capContourDelta reads it.
func capWholeCircleDelta(w survey2d.SideWalk, d, dDelta float64) (float64, error) {
	held, err := capBandRadius(w, d)
	if err != nil {
		return 0, err
	}
	radius, ok := capOffsetRadiusSpan(w, d, dDelta)
	if !ok {
		return 0, errCapContourUnbounded
	}
	return proofbound.IntervalFloatError(radius, held), nil
}

// capOffsetRadiusSpan encloses every radius a circular wall's cap contour
// denotes: the wall radius offset by each amount the setback's span holds.
func capOffsetRadiusSpan(w survey2d.SideWalk, d, dDelta float64) (proofbound.RatInterval, bool) {
	span, ok := capcontour.OffsetSpan(d, dDelta)
	if !ok {
		return proofbound.RatInterval{}, false
	}
	return capcontour.ExactOffsetRadiusOver(w, span)
}

// loopContourDelta re-derives one loop's contour displacement from the loop
// record alone, for the readings that hold no built band — the payload's own
// extentAlong, which evaluates the same contour through capLoopBoundary. It
// walks and joins the loop exactly as buildCapBand does, so the two can never
// disagree about the same contour.
func loopContourDelta(ctx context.Context, loop LoopRecord, d, dDelta float64) (float64, error) {
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	cl, err := oneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return 0, err
	}
	if len(cl.walks) == 1 && cl.walks[0].Closed {
		return capWholeCircleDelta(cl.walks[0], d, dDelta)
	}
	joins, err := capOffsetJoins(budget, cl, d)
	if err != nil {
		return 0, err
	}
	return capContourDelta(cl.walks, joins, d, dDelta)
}

// errCapPatchHeldUnbounded is the refusal for a circular band patch whose held
// angles or radii this evaluator cannot enclose against the values the band's
// closed surface reads there: a window end on its own centre, a walk end whose
// displacement states no bound, or a coordinate that does not lift.
var errCapPatchHeldUnbounded = fmt.Errorf(`%w: a cap-loop chamfer's circular band patch holds an angle or radius this evaluator cannot bound against the point it names, so no measurement integrated over it can be published with a proven bound`, ErrUnsupported)

// capWallHeldAllow is a circular wall patch's capband.HeldAllow, before the
// patch normalizes its window (th0, th1 are the walk's own, start and end its
// cap feet in walk order, capTh0 and capTh1 capWallSweep's angles of them).
//
// The side window ends are float angles of the walk's two ends, and each end
// names the point the record denotes there to within its own proven end bound,
// so the side reference is the exact angle of that denoted point. The cap
// window ends are Atan2s of the held cap feet, which are the cap face's own
// recorded vertices, so the cap reference is their exact angle. The side
// radius is the walk's own RadiusBound — the gap between an ArcSeg's
// math.Hypot and the record's |Start − Center| — and the cap radius is
// allowed to reach either foot's exact distance from the centre, since the cap
// face records this arc through one foot (offset2d.ArcSegment) and states its
// radius from it.
func capWallHeldAllow(w survey2d.SideWalk, capRadius float64, start, end Point2, capTh0, capTh1 float64) (capband.HeldAllow, error) {
	a0, ok0 := capband.AngleAllow(w.CU, w.CV, capband.Point{U: w.StartU, V: w.StartV}, proofbound.WalkEndBoundAllow(w.StartBound), w.Th0)
	a1, ok1 := capband.AngleAllow(w.CU, w.CV, capband.Point{U: w.EndU, V: w.EndV}, proofbound.WalkEndBoundAllow(w.EndBound), w.Th1)
	c0, okC0 := capband.AngleAllow(w.CU, w.CV, start, 0, capTh0)
	c1, okC1 := capband.AngleAllow(w.CU, w.CV, end, 0, capTh1)
	r1, _, okR := capband.RadiusAllow(w.CU, w.CV, capRadius, start, end)
	if !ok0 || !ok1 || !okC0 || !okC1 || !okR || !(w.RadiusBound >= 0) || proofbound.IsNonFinite(w.RadiusBound) {
		return capband.HeldAllow{}, errCapPatchHeldUnbounded
	}
	return capband.HeldAllow{Th0: a0, Th1: a1, CapTh0: c0, CapTh1: c1, SideRadius: w.RadiusBound, CapRadius: r1}, nil
}

// capApexHeldAllow is a reflex corner's apex patch's capband.HeldAllow. Its
// window ends th0 and th1 are the float angles of the offset feet foot0 and
// foot1 about the recorded corner, and the cap face records the connector
// through one of them, so the cap reference is their exact angle and either
// foot's exact distance from the corner. The side directrix is the corner
// point itself: with a side radius of exactly zero no term of the closed forms
// reads a side angle, so the side window holds no allowance.
func capApexHeldAllow(j cornerJoin, dc float64, foot0, foot1 Point2, th0, th1 float64) (capband.HeldAllow, error) {
	c0, ok0 := capband.AngleAllow(j.vU, j.vV, foot0, 0, th0)
	c1, ok1 := capband.AngleAllow(j.vU, j.vV, foot1, 0, th1)
	r1, _, okR := capband.RadiusAllow(j.vU, j.vV, dc, foot0, foot1)
	if !ok0 || !ok1 || !okR {
		return capband.HeldAllow{}, errCapPatchHeldUnbounded
	}
	return capband.HeldAllow{CapTh0: c0, CapTh1: c1, CapRadius: r1}, nil
}

// capBandClosure bounds the slivers that separate a band's integrated patches
// from a closed surface (docs/modify-reach-design.md §8.4).
//
// The flux and moment integrals read each patch over its own held numbers,
// boxed by capband.HeldAllow so a circular patch's directrices are the ones the
// two faces record. The patches then meet the faces exactly along every
// directrix and meet each other along the corner rulings to within a gap: the
// held ends two walks meet at need not be one float, a walk end names its
// denoted point only to within its end bound, an ArcSeg's End need not lie at
// the radius its Start states, and a cap arc's recorded radius is read through
// one foot while the other sits at its own distance. Between two rulings a
// distance at most gap apart, each at most slant long, lies a ruled sliver of
// area at most (slant + gap)·gap; rim sums that over every ruling. A straight
// wall whose ends carry a bound integrates its held side edge while the side
// face records the denoted one, and sideLevel sums the strip between them,
// (L + bS + bE)·(bS + bE), with each corner's own gap² at both levels. reach is
// the largest gap, which widens the coordinate envelope a sliver's points sit
// in.
//
// Every term is zero where the walks' ends are recorded, every arc's End lies
// at its Start's radius and both cap feet of every circular patch lie at one
// exact distance, so an axis-aligned section's band charges nothing here.
type capBandClosure = capband.Closure

// capBandClosureOf converts the built band's corner and edge records into the
// held values used by the closure proof.
func capBandClosureOf(walks []survey2d.SideWalk, joins []cornerJoin, slantIn, slantOut []*Edge, slantInHeld, slantOutHeld []float64) (capBandClosure, error) {
	in := make([]float64, len(slantIn))
	out := make([]float64, len(slantOut))
	for i, edge := range slantIn {
		in[i] = edge.length
	}
	for i, edge := range slantOut {
		out[i] = edge.length
	}
	closure, ok := capband.ClosureOf(walks, capContourJoins(joins), in, out, slantInHeld, slantOutHeld)
	if !ok {
		return capBandClosure{}, errCapPatchHeldUnbounded
	}
	return closure, nil
}
