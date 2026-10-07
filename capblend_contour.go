package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/lengthbound"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file owns the CAP CONTOUR's displacement — the one term every cap-level
// reading of a cap-loop chamfer is bounded through (docs/modify-reach-design.md
// §8.3/§8.4).
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
// floor, and a G1 join (modify §7) never reaches ivIntersect at all. It stands
// as the honest answer for a configuration that does reach it rather than as a
// case the tests can exhibit.
var errCapContourUnbounded = fmt.Errorf(`%w: this evaluator cannot prove a bound on the cap-loop chamfer's own offset contour at a corner, so no cap-level coordinate it emits there can be published with a proven displacement`, ErrUnsupported)

type ivPoint = capcontour.Point
type ivCarrier = capcontour.Carrier

func ivAxisSpread(iv proofbound.RatInterval, c float64) (*big.Rat, bool) {
	return capcontour.AxisSpread(iv, c)
}

func ivUnion(a, b ivPoint) ivPoint { return capcontour.Union(a, b) }
func intervalHull(a, b proofbound.RatInterval) proofbound.RatInterval {
	return capcontour.IntervalHull(a, b)
}
func ivOffsetFoot(vU, vV, tu, tv, d float64) (ivPoint, bool) {
	return capcontour.OffsetFoot(vU, vV, tu, tv, d)
}
func ivCarrierOf(w survey2d.SideWalk, d float64) (ivCarrier, bool) { return capcontour.CarrierOf(w, d) }
func ivExactOffsetRadius(w survey2d.SideWalk, d float64) (*big.Rat, bool) {
	return capcontour.ExactOffsetRadius(w, d)
}
func ivIntersect(a, b ivCarrier) ([]ivPoint, bool)            { return capcontour.Intersect(a, b) }
func ivNearest(cands []ivPoint, u, v float64) (ivPoint, bool) { return capcontour.Nearest(cands, u, v) }
func ivNearestTo(cands []ivPoint, corner ivPoint) (ivPoint, bool) {
	return capcontour.NearestTo(cands, corner)
}
func miterLocusSpeedUpper(prev, cur survey2d.SideWalk, t0, t1, vU, vV float64) (float64, bool) {
	return capcontour.MiterLocusSpeedUpper(prev, cur, t0, t1, vU, vV)
}

// capContourDelta is ONE chamfer band's contour displacement: a proven upper
// bound on how far any cap-level point the band emits sits from the point the
// construction denotes. Every cap-level vertex bound, every cap-level edge
// length bound and the payload's own contour-held directional extent are
// charged this one number, so no reader can be told a different story about the
// same contour.
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
func capContourDelta(walks []survey2d.SideWalk, joins []cornerJoin, d float64) (float64, error) {
	delta := 0.0
	for _, w := range walks {
		if !w.IsCircular() {
			continue
		}
		held, err := capBandRadius(w, d)
		if err != nil {
			return 0, err
		}
		exact, ok := ivExactOffsetRadius(w, d)
		if !ok {
			return 0, errCapContourUnbounded
		}
		// The emitted arc sits at the float radius about the exact centre while
		// the denoted one sits at the exact radius about it, so the radial gap
		// between them IS the displacement of every point of that arc.
		delta = math.Max(delta, proofarith.RationalFloatError(exact, held))
	}
	n := len(walks)
	for i, j := range joins {
		if n == 0 {
			break
		}
		prev, cur := walks[(i+n-1)%n], walks[i]
		if j.arc {
			a, okA := ivOffsetFoot(j.vU, j.vV, prev.TanOutU, prev.TanOutV, d)
			b, okB := ivOffsetFoot(j.vU, j.vV, cur.TanInU, cur.TanInV, d)
			if !okA || !okB {
				return 0, errCapContourUnbounded
			}
			delta = math.Max(delta, math.Max(a.Reach(j.pA.U, j.pA.V), b.Reach(j.pB.U, j.pB.V)))
			continue
		}
		if j.g1 {
			// A G1 join intersects no carriers (modify §7; modify-reach §8.4): its
			// denoted corner is v + d·n̂ for the leaving wall's exact unit normal, and
			// the enclosure is the HULL of the two shared-normal feet, so a join the
			// dead zone classified G1 with a residual turn is charged the spread
			// between the two normals it could have taken.
			a, okA := ivOffsetFoot(j.vU, j.vV, prev.TanOutU, prev.TanOutV, d)
			b, okB := ivOffsetFoot(j.vU, j.vV, cur.TanInU, cur.TanInV, d)
			if !okA || !okB {
				return 0, errCapContourUnbounded
			}
			delta = math.Max(delta, ivUnion(a, b).Reach(j.m.U, j.m.V))
			continue
		}
		ca, okA := ivCarrierOf(prev, d)
		cb, okB := ivCarrierOf(cur, d)
		if !okA || !okB {
			return 0, errCapContourUnbounded
		}
		cands, okI := ivIntersect(ca, cb)
		if !okI {
			return 0, errCapContourUnbounded
		}
		enc, okN := ivNearest(cands, j.vU, j.vV)
		if !okN {
			return 0, errCapContourUnbounded
		}
		delta = math.Max(delta, enc.Reach(j.m.U, j.m.V))
	}
	if proofbound.IsNonFinite(delta) {
		return 0, errCapContourUnbounded
	}
	return delta, nil
}

// capWholeCircleDelta is the cornerless closed circle's own contour
// displacement — the one shape with no corner join at all, whose whole contour
// is the concentric circle at the offset radius.
func capWholeCircleDelta(w survey2d.SideWalk, d float64) (float64, error) {
	held, err := capBandRadius(w, d)
	if err != nil {
		return 0, err
	}
	exact, ok := ivExactOffsetRadius(w, d)
	if !ok {
		return 0, errCapContourUnbounded
	}
	return proofarith.RationalFloatError(exact, held), nil
}

// loopContourDelta re-derives one loop's contour displacement from the loop
// record alone, for the readings that hold no built band — the payload's own
// extentAlong, which evaluates the same contour through capLoopBoundary. It
// walks and joins the loop exactly as buildCapBand does, so the two can never
// disagree about the same contour.
func loopContourDelta(ctx context.Context, loop LoopRecord, d float64) (float64, error) {
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	cl, err := oneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return 0, err
	}
	if len(cl.walks) == 1 && cl.walks[0].Closed {
		return capWholeCircleDelta(cl.walks[0], d)
	}
	joins, err := capOffsetJoins(budget, cl, d)
	if err != nil {
		return 0, err
	}
	return capContourDelta(cl.walks, joins, d)
}

// The root proof and geometry callers keep their existing private names.
func dySquaredDistance3(a0, a1, a2, b0, b1, b2 float64) (proofarith.Dyadic, bool) {
	return lengthbound.DySquaredDistance3(a0, a1, a2, b0, b1, b2)
}

func ratSquaredDistance3(a0, a1, a2, b0, b1, b2 float64) *big.Rat {
	return lengthbound.RatSquaredDistance3(a0, a1, a2, b0, b1, b2)
}

func straightEdgeBound(held float64, squared proofarith.Dyadic, ok bool, endpointDeltas ...float64) float64 {
	return lengthbound.StraightEdgeBound(held, squared, ok, endpointDeltas...)
}

func capEdgeLengthBound(held float64, end, start Point2, delta float64) float64 {
	return lengthbound.CapEdgeLengthBound(held, end, start, delta)
}

func capApexArcBound(j cornerJoin, d, held float64, wraps int, delta float64) float64 {
	return lengthbound.CapApexArcBound(
		lengthbound.ApexJoin{VU: j.vU, VV: j.vV, PA: j.pA, PB: j.pB}, d, held, wraps, delta,
	)
}

func capCircleLengthBound(exactRadius *big.Rat, held float64) float64 {
	return lengthbound.CapCircleLengthBound(exactRadius, held)
}

func capWallArcBound(cU, cV float64, start, end Point2, capRadius, held float64, wraps int, delta float64) float64 {
	return lengthbound.CapWallArcBound(cU, cV, start, end, capRadius, held, wraps, delta)
}

func capSweepAllow(cU, cV, radius float64, start, end Point2, held float64, wraps int, delta float64) float64 {
	return lengthbound.CapSweepAllow(cU, cV, radius, start, end, held, wraps, delta)
}
