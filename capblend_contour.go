package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
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
// roots, which round outward through ratSqrtDown/ratSqrtUp. Interval arithmetic
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
// floor. It stands as the honest answer for a configuration that does reach it
// rather than as a case the tests can exhibit.
var errCapContourUnbounded = fmt.Errorf(`%w: this evaluator cannot prove a bound on the cap-loop chamfer's own offset contour at a corner, so no cap-level coordinate it emits there can be published with a proven displacement`, ErrUnsupported)

// ivPoint is a rational-interval enclosure of one plane-local (u, v) point.
type ivPoint struct{ u, v ratInterval }

// ivExactPoint lifts a pair of float64 coordinates, which are exact rationals.
func ivExactPoint(u, v float64) (ivPoint, bool) {
	ru, rv := floatRat(u), floatRat(v)
	if ru == nil || rv == nil {
		return ivPoint{}, false
	}
	return ivPoint{u: pointInterval(ru), v: pointInterval(rv)}, true
}

// reach is an upper bound on |p − q| over every q the enclosure holds, so a
// point known to lie in the enclosure sits at most this far from p.
func (e ivPoint) reach(p Point2) float64 {
	du, okU := ivAxisSpread(e.u, p.U)
	dv, okV := ivAxisSpread(e.v, p.V)
	if !okU || !okV {
		return math.Inf(1)
	}
	return ratSqrtUp(new(big.Rat).Add(
		new(big.Rat).Mul(du, du),
		new(big.Rat).Mul(dv, dv),
	))
}

// ivAxisSpread is max(|lo − c|, |hi − c|), the furthest the interval reaches
// from c along one axis.
func ivAxisSpread(iv ratInterval, c float64) (*big.Rat, bool) {
	rc := floatRat(c)
	if rc == nil || iv.lo == nil || iv.hi == nil {
		return nil, false
	}
	lo := new(big.Rat).Abs(new(big.Rat).Sub(iv.lo, rc))
	hi := new(big.Rat).Abs(new(big.Rat).Sub(iv.hi, rc))
	if lo.Cmp(hi) >= 0 {
		return lo, true
	}
	return hi, true
}

// ivUnion is the smallest box holding both enclosures — what an ambiguous root
// selection reports, so an unresolved choice widens the displacement rather
// than picking a branch the exact arithmetic has not decided.
func ivUnion(a, b ivPoint) ivPoint {
	return ivPoint{u: intervalHull(a.u, b.u), v: intervalHull(a.v, b.v)}
}

func intervalHull(a, b ratInterval) ratInterval {
	lo, hi := a.lo, a.hi
	if b.lo.Cmp(lo) < 0 {
		lo = b.lo
	}
	if b.hi.Cmp(hi) > 0 {
		hi = b.hi
	}
	return interval(lo, hi)
}

// intervalQuo divides two intervals. ok is false where the divisor straddles
// zero, since the quotient is then unbounded and no box encloses it.
func intervalQuo(a, b ratInterval) (ratInterval, bool) {
	if b.lo.Sign() <= 0 && b.hi.Sign() >= 0 {
		return ratInterval{}, false
	}
	corners := [4]*big.Rat{
		new(big.Rat).Quo(a.lo, b.lo),
		new(big.Rat).Quo(a.lo, b.hi),
		new(big.Rat).Quo(a.hi, b.lo),
		new(big.Rat).Quo(a.hi, b.hi),
	}
	lo, hi := corners[0], corners[0]
	for _, c := range corners[1:] {
		if c.Cmp(lo) < 0 {
			lo = c
		}
		if c.Cmp(hi) > 0 {
			hi = c
		}
	}
	return interval(lo, hi), true
}

// intervalSquare is x² over an interval. It is not intervalMul(a, a): the
// corner products of an interval straddling zero put a NEGATIVE value at the
// low end, and a square never takes one.
func intervalSquare(a ratInterval) ratInterval {
	lo2 := new(big.Rat).Mul(a.lo, a.lo)
	hi2 := new(big.Rat).Mul(a.hi, a.hi)
	if a.lo.Sign() >= 0 {
		return interval(lo2, hi2)
	}
	if a.hi.Sign() <= 0 {
		return interval(hi2, lo2)
	}
	hi := lo2
	if hi2.Cmp(hi) > 0 {
		hi = hi2
	}
	return interval(new(big.Rat), hi)
}

// intervalSqrt encloses the square root over a non-negative interval, each end
// rounded OUTWARD through spline_length.go's exact-comparison bracket, so no
// platform's sqrt can narrow it. A lower end below zero is clamped to zero:
// the enclosure then covers the tangency the float discriminant reached for.
func intervalSqrt(a ratInterval) (ratInterval, bool) {
	if a.hi.Sign() < 0 {
		return ratInterval{}, false
	}
	lo := 0.0
	if a.lo.Sign() > 0 {
		lo = ratSqrtDown(a.lo)
	}
	hi := ratSqrtUp(a.hi)
	rlo, rhi := floatRat(lo), floatRat(hi)
	if rlo == nil || rhi == nil {
		return ratInterval{}, false
	}
	return interval(rlo, rhi), true
}

// ivUnitVec encloses the EXACT unit vector of a float pair — the value
// normalize2 rounds. The pair itself is exact, so the only widening is the
// length's own outward-rounded square root.
func ivUnitVec(x, y float64) (ivPoint, bool) {
	rx, ry := floatRat(x), floatRat(y)
	if rx == nil || ry == nil {
		return ivPoint{}, false
	}
	n2 := new(big.Rat).Add(new(big.Rat).Mul(rx, rx), new(big.Rat).Mul(ry, ry))
	if n2.Sign() == 0 {
		return ivPoint{}, false
	}
	l, ok := intervalSqrt(pointInterval(n2))
	if !ok || l.lo.Sign() <= 0 {
		return ivPoint{}, false
	}
	u, okU := intervalQuo(pointInterval(rx), l)
	v, okV := intervalQuo(pointInterval(ry), l)
	if !okU || !okV {
		return ivPoint{}, false
	}
	return ivPoint{u: u, v: v}, true
}

// ivOffsetFoot encloses the exact material-side foot v + d·rot90(unit(t)) —
// the point shell_offset.go and capblend_geom.go both spell
// v + d·(−ty, tx) after normalize2.
func ivOffsetFoot(vU, vV, tu, tv, d float64) (ivPoint, bool) {
	n, ok := ivUnitVec(tu, tv)
	if !ok {
		return ivPoint{}, false
	}
	v, okV := ivExactPoint(vU, vV)
	rd := floatRat(d)
	if !okV || rd == nil {
		return ivPoint{}, false
	}
	return ivPoint{
		u: intervalAdd(v.u, intervalScale(intervalNeg(n.v), rd)),
		v: intervalAdd(v.v, intervalScale(n.u, rd)),
	}, true
}

// ivCarrier is one wall's offset carrier, enclosed: an offset LINE (a point on
// it plus its unit direction) or a concentric CIRCLE (the wall's own exact
// centre and the exact offset radius). It mirrors shell_offset.go's
// offsetCarrier field for field, so the enclosure is of the same object the
// float miter solve intersects.
type ivCarrier struct {
	isLine bool
	p, dir ivPoint
	c      ivPoint
	r      ratInterval
}

func ivCarrierOf(w sideWalk, d float64) (ivCarrier, bool) {
	if !w.isCircular() {
		p, okP := ivOffsetFoot(w.startU, w.startV, w.tanInU, w.tanInV, d)
		dir, okD := ivUnitVec(w.tanInU, w.tanInV)
		if !okP || !okD {
			return ivCarrier{}, false
		}
		return ivCarrier{isLine: true, p: p, dir: dir}, true
	}
	r, ok := ivExactOffsetRadius(w, d)
	if !ok {
		return ivCarrier{}, false
	}
	c, okC := ivExactPoint(w.cU, w.cV)
	if !okC {
		return ivCarrier{}, false
	}
	return ivCarrier{c: c, r: pointInterval(r)}, true
}

// ivExactOffsetRadius is offsetRadius's own R − insideSign·d taken EXACTLY:
// both operands are float64s, so their difference is a rational with no
// rounding at all, and the float the build holds is the rounding of THIS value.
func ivExactOffsetRadius(w sideWalk, d float64) (*big.Rat, bool) {
	inside := 1.0
	if w.th1 < w.th0 { // a clockwise walk has its material outside the circle
		inside = -1.0
	}
	rr, rd := floatRat(w.radius), floatRat(inside*d)
	if rr == nil || rd == nil {
		return nil, false
	}
	out := new(big.Rat).Sub(rr, rd)
	if out.Sign() <= 0 {
		return nil, false
	}
	return out, true
}

// ivIntersect encloses every root of the two offset carriers, dispatching
// exactly as fillet.go's intersectOffsets does over the same three cases.
func ivIntersect(a, b ivCarrier) ([]ivPoint, bool) {
	switch {
	case a.isLine && b.isLine:
		return ivLineLine(a, b)
	case a.isLine:
		return ivLineCircle(a, b)
	case b.isLine:
		return ivLineCircle(b, a)
	default:
		return ivCircleCircle(a, b)
	}
}

func ivLineLine(a, b ivCarrier) ([]ivPoint, bool) {
	den := intervalSub(intervalMul(a.dir.u, b.dir.v), intervalMul(a.dir.v, b.dir.u))
	num := intervalSub(
		intervalMul(intervalSub(b.p.u, a.p.u), b.dir.v),
		intervalMul(intervalSub(b.p.v, a.p.v), b.dir.u),
	)
	s, ok := intervalQuo(num, den)
	if !ok {
		return nil, false
	}
	return []ivPoint{{
		u: intervalAdd(a.p.u, intervalMul(s, a.dir.u)),
		v: intervalAdd(a.p.v, intervalMul(s, a.dir.v)),
	}}, true
}

func ivLineCircle(l, c ivCarrier) ([]ivPoint, bool) {
	fx := intervalSub(l.p.u, c.c.u)
	fy := intervalSub(l.p.v, c.c.v)
	bb := intervalAdd(intervalMul(fx, l.dir.u), intervalMul(fy, l.dir.v))
	cc := intervalSub(intervalAdd(intervalSquare(fx), intervalSquare(fy)), intervalSquare(c.r))
	disc := intervalSub(intervalSquare(bb), cc)
	if disc.hi.Sign() < 0 {
		// The exact carriers miss each other entirely: the float solve reached
		// a root of a system that has none, so there is no denoted point to
		// enclose.
		return nil, false
	}
	sq, ok := intervalSqrt(disc)
	if !ok {
		return nil, false
	}
	nb := intervalNeg(bb)
	out := make([]ivPoint, 0, 2)
	for _, s := range []ratInterval{intervalAdd(nb, sq), intervalSub(nb, sq)} {
		out = append(out, ivPoint{
			u: intervalAdd(l.p.u, intervalMul(s, l.dir.u)),
			v: intervalAdd(l.p.v, intervalMul(s, l.dir.v)),
		})
	}
	return out, true
}

func ivCircleCircle(a, b ivCarrier) ([]ivPoint, bool) {
	dx := intervalSub(b.c.u, a.c.u)
	dy := intervalSub(b.c.v, a.c.v)
	dsq := intervalAdd(intervalSquare(dx), intervalSquare(dy))
	dist, ok := intervalSqrt(dsq)
	if !ok || dist.lo.Sign() <= 0 {
		return nil, false
	}
	mid, okMid := intervalQuo(
		intervalSub(intervalAdd(dsq, intervalSquare(a.r)), intervalSquare(b.r)),
		intervalScale(dist, big.NewRat(2, 1)),
	)
	if !okMid {
		return nil, false
	}
	h2 := intervalSub(intervalSquare(a.r), intervalSquare(mid))
	if h2.hi.Sign() < 0 {
		return nil, false
	}
	h, okH := intervalSqrt(h2)
	if !okH {
		return nil, false
	}
	along, okA := intervalQuo(mid, dist)
	across, okC := intervalQuo(h, dist)
	if !okA || !okC {
		return nil, false
	}
	baseU := intervalAdd(a.c.u, intervalMul(along, dx))
	baseV := intervalAdd(a.c.v, intervalMul(along, dy))
	offU := intervalMul(across, dy)
	offV := intervalMul(across, dx)
	return []ivPoint{
		{u: intervalSub(baseU, offU), v: intervalAdd(baseV, offV)},
		{u: intervalAdd(baseU, offU), v: intervalSub(baseV, offV)},
	}, true
}

// ivNearest encloses intersectOffsets' own "root nearest the corner". A
// candidate whose squared-distance interval starts beyond another's end is
// PROVEN not to be the nearest and is dropped; every candidate the exact
// arithmetic leaves undecided joins the hull, so a near-tangency reports one
// wide honest displacement rather than a branch nothing decided.
func ivNearest(cands []ivPoint, vU, vV float64) (ivPoint, bool) {
	if len(cands) == 0 {
		return ivPoint{}, false
	}
	corner, ok := ivExactPoint(vU, vV)
	if !ok {
		return ivPoint{}, false
	}
	d2 := make([]ratInterval, len(cands))
	for i, c := range cands {
		d2[i] = intervalAdd(
			intervalSquare(intervalSub(c.u, corner.u)),
			intervalSquare(intervalSub(c.v, corner.v)),
		)
	}
	best := d2[0].hi
	for _, iv := range d2[1:] {
		if iv.hi.Cmp(best) < 0 {
			best = iv.hi
		}
	}
	var out ivPoint
	found := false
	for i, iv := range d2 {
		if iv.lo.Cmp(best) > 0 {
			continue
		}
		if !found {
			out, found = cands[i], true
			continue
		}
		out = ivUnion(out, cands[i])
	}
	return out, found
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
func capContourDelta(walks []sideWalk, joins []cornerJoin, d float64) (float64, error) {
	delta := 0.0
	for _, w := range walks {
		if !w.isCircular() {
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
		delta = math.Max(delta, rationalFloatError(exact, held))
	}
	n := len(walks)
	for i, j := range joins {
		if n == 0 {
			break
		}
		prev, cur := walks[(i+n-1)%n], walks[i]
		if j.arc {
			a, okA := ivOffsetFoot(j.vU, j.vV, prev.tanOutU, prev.tanOutV, d)
			b, okB := ivOffsetFoot(j.vU, j.vV, cur.tanInU, cur.tanInV, d)
			if !okA || !okB {
				return 0, errCapContourUnbounded
			}
			delta = math.Max(delta, math.Max(a.reach(j.pA), b.reach(j.pB)))
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
		delta = math.Max(delta, enc.reach(j.m))
	}
	if isNonFinite(delta) {
		return 0, errCapContourUnbounded
	}
	return delta, nil
}

// capWholeCircleDelta is the cornerless closed circle's own contour
// displacement — the one shape with no corner join at all, whose whole contour
// is the concentric circle at the offset radius.
func capWholeCircleDelta(w sideWalk, d float64) (float64, error) {
	held, err := capBandRadius(w, d)
	if err != nil {
		return 0, err
	}
	exact, ok := ivExactOffsetRadius(w, d)
	if !ok {
		return 0, errCapContourUnbounded
	}
	return rationalFloatError(exact, held), nil
}

// loopContourDelta re-derives one loop's contour displacement from the loop
// record alone, for the readings that hold no built band — the payload's own
// extentAlong, which evaluates the same contour through capLoopBoundary. It
// walks and joins the loop exactly as buildCapBand does, so the two can never
// disagree about the same contour.
func loopContourDelta(ctx context.Context, loop LoopRecord, d float64) (float64, error) {
	budget := newWorkBudget(ctx)
	work := newFreeformWork()
	cl, err := oneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return 0, err
	}
	if len(cl.walks) == 1 && cl.walks[0].closed {
		return capWholeCircleDelta(cl.walks[0], d)
	}
	joins, err := capOffsetJoins(budget, cl, d)
	if err != nil {
		return 0, err
	}
	return capContourDelta(cl.walks, joins, d)
}

// ratSquaredDistance3 is the exact squared distance between two plane-local
// points, every coordinate a float64 and hence an exact rational, so the
// returned value is the true square of the length the float evaluation
// approximated. sqrtIntervalError then reports what that evaluation committed.
func ratSquaredDistance3(a0, a1, a2, b0, b1, b2 float64) *big.Rat {
	sum := new(big.Rat)
	for _, pair := range [3][2]float64{{a0, b0}, {a1, b1}, {a2, b2}} {
		x, y := floatRat(pair[0]), floatRat(pair[1])
		if x == nil || y == nil {
			return nil
		}
		diff := new(big.Rat).Sub(x, y)
		sum.Add(sum, diff.Mul(diff, diff))
	}
	return sum
}

// straightEdgeBound is the proven bound on a straight cap-level edge's held
// length. It has three independent terms and each speaks for a different thing:
// the square root's own committed error, measured against the exact rational
// squared length rather than against a Hypot ulp contract Go does not give;
// and one displacement per endpoint, since moving an endpoint of a segment by
// e moves its length by at most e.
func straightEdgeBound(held float64, squared *big.Rat, endpointDeltas ...float64) float64 {
	if squared == nil {
		return math.Inf(1)
	}
	return absSumUpper(append([]float64{sqrtIntervalError(squared, held)}, endpointDeltas...)...)
}

// arcSweepAllow converts a contour displacement into the arc length it can move.
// A foot displaced by delta on a circle of radius r turns through at most
// arcsin(delta/r) ≤ (π/2)·delta/r, so the arc between two such feet changes
// length by at most r·2·(π/2)·delta/r = π·delta — independent of the radius.
// A displacement at or past the radius says nothing about the turn at all, and
// the caller then owes the whole-circumference envelope instead.
func arcSweepAllow(radius, delta float64) (float64, bool) {
	if radius <= 0 || delta >= radius || isNonFinite(radius) || isNonFinite(delta) {
		return 0, false
	}
	return productUpper(math.Nextafter(math.Pi, math.Inf(1)), delta), true
}

// capApexArcBound bounds the reflex connector arc's held length d·(th0 − th1).
// The arc's centre is the ORIGINAL corner and its radius is exactly the
// setback, both recorded, so the only error is in the sweep: the exact turn
// between the two feet the build actually holds (an atan2Interval bracket, so
// no libm accuracy is assumed) plus the turn those feet's own displacement can
// account for.
func capApexArcBound(j cornerJoin, d, held float64, wraps int, delta float64) float64 {
	fallback := conservativeValueError(held, productUpper(twoPiUpper(), math.Abs(d)))
	aU, aV := floatRat(j.pA.U-j.vU), floatRat(j.pA.V-j.vV)
	bU, bV := floatRat(j.pB.U-j.vU), floatRat(j.pB.V-j.vV)
	rd := floatRat(d)
	if aU == nil || aV == nil || bU == nil || bV == nil || rd == nil {
		return fallback
	}
	sweep := intervalSub(atan2Interval(aV, aU, false), atan2Interval(bV, bU, false))
	if wraps != 0 {
		sweep = intervalAdd(sweep, intervalScale(
			intervalScale(interval(piLower, piUpper), big.NewRat(2, 1)),
			big.NewRat(int64(wraps), 1),
		))
	}
	// The build's own float differences round, and that rounding displaces the
	// direction the angle is read from just as the contour itself does.
	shift := absSumUpper(
		delta,
		addRoundError(j.pA.U, -j.vU, j.pA.U-j.vU),
		addRoundError(j.pA.V, -j.vV, j.pA.V-j.vV),
		addRoundError(j.pB.U, -j.vU, j.pB.U-j.vU),
		addRoundError(j.pB.V, -j.vV, j.pB.V-j.vV),
	)
	turn, ok := arcSweepAllow(d, shift)
	if !ok {
		return fallback
	}
	bound := absSumUpper(intervalFloatError(intervalScale(sweep, rd), held), turn)
	return math.Min(bound, fallback)
}

// capCircleLengthBound bounds a whole cap-level circle's held 2πr against the
// EXACT offset radius: π is bracketed by moments.go's own rational constants,
// so the enclosure needs no float value of π and no libm accuracy.
func capCircleLengthBound(exactRadius *big.Rat, held float64) float64 {
	if exactRadius == nil {
		return math.Inf(1)
	}
	circumference := intervalScale(
		intervalScale(interval(piLower, piUpper), big.NewRat(2, 1)),
		exactRadius,
	)
	return intervalFloatError(circumference, held)
}

// capWallArcBound bounds a wall's own cap-level arc's held sweep
// capRadius·(capTh1 − capTh0) (signed, matching held's own sign convention —
// the caller passes capRadius*sweepSigned, never an absolute value, so the
// bracket below and held agree on which branch they are stating). The cap
// arc runs between the offset corner feet (start, end), whose angle about the
// wall's exact centre is generally DIFFERENT from the wall's own recorded
// th0/th1 wherever the corner is a genuine (non-tangent) miter
// (docs/modify-reach-design.md §8.3) — the cap directrix is TRIMMED there —
// so the sweep is bracketed straight from those feet, exactly the way
// capApexArcBound brackets a reflex corner's own connector: an atan2Interval
// enclosure of the two feet's own turn about the centre, so no libm accuracy
// is assumed of the sweep itself, plus wraps (capWallSweep's own unwrap count)
// to reproduce the same branch, plus the turn the two feet's own contour
// displacement can account for.
func capWallArcBound(cU, cV float64, start, end Point2, capRadius, held float64, wraps int, delta float64) float64 {
	fallback := conservativeValueError(held, productUpper(twoPiUpper(), math.Abs(capRadius)))
	aU, aV := floatRat(start.U-cU), floatRat(start.V-cV)
	bU, bV := floatRat(end.U-cU), floatRat(end.V-cV)
	rd := floatRat(capRadius)
	if aU == nil || aV == nil || bU == nil || bV == nil || rd == nil {
		return fallback
	}
	sweep := intervalSub(atan2Interval(bV, bU, false), atan2Interval(aV, aU, false))
	if wraps != 0 {
		sweep = intervalAdd(sweep, intervalScale(
			intervalScale(interval(piLower, piUpper), big.NewRat(2, 1)),
			big.NewRat(int64(wraps), 1),
		))
	}
	// The build's own float differences round, and that rounding displaces the
	// direction the angle is read from just as the contour itself does.
	shift := absSumUpper(
		delta,
		addRoundError(start.U, -cU, start.U-cU),
		addRoundError(start.V, -cV, start.V-cV),
		addRoundError(end.U, -cU, end.U-cU),
		addRoundError(end.V, -cV, end.V-cV),
	)
	turn, ok := arcSweepAllow(downRound(math.Abs(capRadius)), shift)
	if !ok {
		return fallback
	}
	bound := absSumUpper(intervalFloatError(intervalScale(sweep, rd), held), turn)
	return math.Min(bound, fallback)
}
