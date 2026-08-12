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
//
// The length VALUE a cap-level edge carries must stay the actual held float,
// never a stand-in such as +Inf for "unknown": selector.go's LongerThan
// predicate compares the raw e.length field directly, with no reference to
// Exactness or lengthBound, so an edge whose length field were ever widened
// to signal uncertainty would silently match every LongerThan query instead.
// Uncertainty belongs in lengthBound alone.
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

// ivOffsetFootRange generalises ivOffsetFoot to an OFFSET INTERVAL rather
// than one float: the enclosure of v + t·rot90(unit(t)) for every offset
// amount t in tRange, the point family a line carrier's own anchor sweeps as
// the offset amount varies. ivOffsetFoot is this at one degenerate point.
func ivOffsetFootRange(vU, vV, tu, tv float64, tRange ratInterval) (ivPoint, bool) {
	n, ok := ivUnitVec(tu, tv)
	if !ok {
		return ivPoint{}, false
	}
	v, okV := ivExactPoint(vU, vV)
	if !okV {
		return ivPoint{}, false
	}
	return ivPoint{
		u: intervalAdd(v.u, intervalMul(intervalNeg(n.v), tRange)),
		v: intervalAdd(v.v, intervalMul(n.u, tRange)),
	}, true
}

// ivCarrierOverRange generalises ivCarrierOf to an OFFSET INTERVAL [t0, t1]
// rather than one float, enclosing every carrier the wall's own offset
// construction occupies as the offset amount ranges over it — the same
// closed forms ivCarrierOf evaluates at one point, evaluated over the whole
// range instead. A line's carrier stays a single line: only its anchor point
// moves, along the line's own FIXED unit normal, so the direction needs no
// widening at all. A circle's carrier stays a single concentric circle whose
// radius now encloses the offset radius's own range rather than one value.
// At t0 == t1 == d it reduces to ivCarrierOf(w, d)'s own enclosure, since
// both build the offset amount from the identical closed form.
func ivCarrierOverRange(w sideWalk, t0, t1 float64) (ivCarrier, bool) {
	rt0, rt1 := floatRat(t0), floatRat(t1)
	if rt0 == nil || rt1 == nil {
		return ivCarrier{}, false
	}
	tRange := interval(rt0, rt1)
	if !w.isCircular() {
		p, okP := ivOffsetFootRange(w.startU, w.startV, w.tanInU, w.tanInV, tRange)
		dir, okD := ivUnitVec(w.tanInU, w.tanInV)
		if !okP || !okD {
			return ivCarrier{}, false
		}
		return ivCarrier{isLine: true, p: p, dir: dir}, true
	}
	rr := floatRat(w.radius)
	if rr == nil {
		return ivCarrier{}, false
	}
	inside := big.NewRat(1, 1)
	if w.th1 < w.th0 { // a clockwise walk has its material outside the circle
		inside = big.NewRat(-1, 1)
	}
	r := intervalSub(pointInterval(rr), intervalMul(tRange, pointInterval(inside)))
	if r.lo.Sign() <= 0 {
		return ivCarrier{}, false
	}
	c, okC := ivExactPoint(w.cU, w.cV)
	if !okC {
		return ivCarrier{}, false
	}
	return ivCarrier{c: c, r: r}, true
}

// miterConstraintRow is one carrier's own linear constraint on the miter
// locus's velocity P'(t), read at a foot enclosed within the box the
// intersection of the corner's two carriers over the offset range proves
// (ivCarrierOverRange, ivIntersect, ivNearest).
//
// A line carrier's offset satisfies n·X = n·v0 + t for its own fixed unit
// normal n (the construction's own p0(t) = v0 + t·n, and every point of the
// offset LINE shares n's component since the line runs perpendicular to it),
// so differentiating in t gives n·P'(t) = 1 — a row that needs no widening at
// all, since n does not depend on t.
//
// A circle carrier's offset satisfies |X-c| = R - inside·t (offsetRadius's
// own closed form), so differentiating gives û·P'(t) = -inside, with
// û = (X-c)/|X-c| the unit radial direction AT THE FOOT — which does depend
// on t, so it is enclosed from the box foot proves rather than read at one
// point.
func miterConstraintRow(w sideWalk, c ivCarrier, foot ivPoint) (ivPoint, *big.Rat, bool) {
	if c.isLine {
		n := ivPoint{u: intervalNeg(c.dir.v), v: c.dir.u}
		return n, big.NewRat(1, 1), true
	}
	dx := intervalSub(foot.u, c.c.u)
	dy := intervalSub(foot.v, c.c.v)
	distSq := intervalAdd(intervalSquare(dx), intervalSquare(dy))
	dist, ok := intervalSqrt(distSq)
	if !ok || dist.lo.Sign() <= 0 {
		return ivPoint{}, nil, false
	}
	ux, okU := intervalQuo(dx, dist)
	uy, okV := intervalQuo(dy, dist)
	if !okU || !okV {
		return ivPoint{}, nil, false
	}
	rhs := big.NewRat(-1, 1)
	if w.th1 < w.th0 { // a clockwise walk has its material outside the circle
		rhs = big.NewRat(1, 1)
	}
	return ivPoint{u: ux, v: uy}, rhs, true
}

// circleCircleLocusSpeedUpper bounds |dP/dt| for a corner where TWO circular
// walls' own offset carriers meet, by enclosing the corner foot over the
// whole offset range at once (ivCarrierOverRange, ivIntersect, ivNearest)
// and solving the 2x2 linear system miterConstraintRow builds from each
// carrier — Cramer's rule, carried out in interval arithmetic. ok is false
// whenever the determinant's own enclosure straddles zero (the two carriers'
// rows are nearly parallel there) or any enclosure along the way fails.
//
// This decorrelates the two carriers' own offset amount — each is enclosed
// over [t0, t1] independently, rather than tracked as the SAME shared
// parameter through both — which is sound (the true trajectory is always
// inside the resulting box) but loose exactly where a corner is TANGENT: two
// circles tangent at the corner stay tangent under a consistent offset (the
// same identity lineCircleLocusSpeedUpper's own doc comment derives for a
// line and a circle), so the true velocity is finite there, but this
// decorrelated enclosure cannot tell that persistent tangency from a
// momentary one and refuses on both. lineCircleLocusSpeedUpper below carries
// the exact closed form that tells them apart for a line meeting a circle —
// the common case, reached at every tangent-filleted corner in this
// codebase's own test fixtures — and this generic solve is kept for the
// circle-circle miter it does not cover, on the same reject-only footing
// every other refusal here stands on: never a wrong bound, only a
// conservative one at a tangent circle-circle corner.
func circleCircleLocusSpeedUpper(prev, cur sideWalk, t0, t1, vU, vV float64) (float64, bool) {
	ca, okA := ivCarrierOverRange(prev, t0, t1)
	cb, okB := ivCarrierOverRange(cur, t0, t1)
	if !okA || !okB {
		return 0, false
	}
	cands, okI := ivIntersect(ca, cb)
	if !okI {
		return 0, false
	}
	foot, okN := ivNearest(cands, vU, vV)
	if !okN {
		return 0, false
	}
	rowA, rhsA, okRA := miterConstraintRow(prev, ca, foot)
	rowB, rhsB, okRB := miterConstraintRow(cur, cb, foot)
	if !okRA || !okRB {
		return 0, false
	}
	det := intervalSub(intervalMul(rowA.u, rowB.v), intervalMul(rowB.u, rowA.v))
	if det.lo.Sign() <= 0 && det.hi.Sign() >= 0 {
		return 0, false
	}
	pu, okU := intervalQuo(intervalSub(intervalScale(rowB.v, rhsA), intervalScale(rowA.v, rhsB)), det)
	pv, okV := intervalQuo(intervalSub(intervalScale(rowA.u, rhsB), intervalScale(rowB.u, rhsA)), det)
	if !okU || !okV {
		return 0, false
	}
	mag, ok := intervalSqrt(intervalAdd(intervalSquare(pu), intervalSquare(pv)))
	if !ok {
		return 0, false
	}
	upper, exact := mag.hi.Float64()
	if !exact {
		upper = math.Nextafter(upper, math.Inf(1))
	}
	if isNonFinite(upper) || upper < 0 {
		return 0, false
	}
	return upper, true
}

// lineWallFrame is a straight wall's own EXACT local frame: n is the
// enclosed unit MATERIAL-SIDE normal (rot90 of the enclosed unit tangent)
// and e is the enclosed unit tangent itself, both read once, since a line's
// own direction never moves as its offset amount varies — only its anchor
// point does, along n.
type lineWallFrame struct {
	anchorU, anchorV float64
	n, e             ivPoint
}

// The interval e below deliberately EXCLUDES normalize2(w.tanInU, w.tanInV).
// That is not a hole in the enclosure, and widening e to cover normalize2's
// output would be wrong. ivUnitVec encloses the EXACT unit vector of the float
// pair — the value normalize2 rounds — and §8.4 requires this frame to hold the
// direction the construction DENOTES "whatever the platform's sqrt and hypot
// did", so anchoring it on a rounded float would replace the denoted direction
// with one particular platform's approximation of it. A tangent of (3, 4) is
// the clearest case: e is the exact [3/5, 3/5] × [4/5, 4/5], 3/5 is not a
// float, and normalize2 lands 2.22e-17 below the interval it is not supposed
// to be in. To falsify this claim, exhibit an admitted corner whose published
// Edge.Length interval fails to enclose the true locus length — not merely one
// where normalize2's output falls outside e, which is every rotated wall.
func lineWallFrameOf(w sideWalk) (lineWallFrame, bool) {
	e, ok := ivUnitVec(w.tanInU, w.tanInV)
	if !ok {
		return lineWallFrame{}, false
	}
	n := ivPoint{u: intervalNeg(e.v), v: e.u}
	return lineWallFrame{anchorU: w.startU, anchorV: w.startV, n: n, e: e}, true
}

// insideSignOf is the exact ±1 rational offsetRadius's own sign convention
// reads off a circular wall's walked sense: +1 when the wall's material
// lies inside the circle (th1 >= th0), −1 when it lies outside.
func insideSignOf(w sideWalk) *big.Rat {
	if w.th1 < w.th0 {
		return big.NewRat(-1, 1)
	}
	return big.NewRat(1, 1)
}

// lineCircleLocusSpeedUpper bounds |dP/dt| for the corner foot where a
// STRAIGHT wall's own offset carrier meets a CIRCULAR wall's, over the
// offset range [t0, t1], by an EXACT closed form rather than
// circleCircleLocusSpeedUpper's decorrelated enclosure — which matters
// because this is the common case, reached at every straight-to-arc
// tangent-filleted corner this codebase builds (a rounded rectangle's own
// corners among them), and that decorrelated method refuses on every one of
// them (see circleCircleLocusSpeedUpper's own doc comment).
//
// Parametrise a point on the offset line as anchor + t·n + s·e (n, e the
// line's own fixed unit normal and tangent — offsetCarrier's own
// construction), and substitute into the offset circle's own
// |X−c| = R − inside·t. Writing w(t) = (anchor + t·n) − c, the quadratic in
// s is s² + 2(w(t)·e)s + (w(t)·w(t) − (R−inside·t)²) = 0, whose
// discriminant is Δ(t) = (w(t)·e)² − w(t)·w(t) + (R−inside·t)². Because
// n·e = 0 and |n| = 1, w(t)·e is CONSTANT (call it β) and w(t)·w(t) is
// R² − α² + ... — expanding fully, Δ(t)'s own t² coefficient is exactly
// 1 − inside² = 0 (inside is ±1), so Δ is EXACTLY AFFINE in t:
// Δ(t) = (R² − α²) − 2(α + inside·R)·t, α = w0·n, w0 = anchor − c.
//
// That affine form is what tells a TANGENT join's own PERSISTENT tangency
// (Δ(t) ≡ 0, both coefficients zero) from a momentary one (Δ0 = 0 but
// Δ1 ≠ 0, a true fold where the speed is genuinely unbounded right at the
// corner): both of s's two roots collapse to the SAME value −β for every t
// when Δ1 = 0, so X(t) = anchor + t·n − β·e is itself affine in t and
// X'(t) = n exactly, a closed-form, finite velocity that never needed a
// square root at all. Where Δ1 ≠ 0, |s'(t)| = |Δ1| / (2·sqrt(Δ(t))), whose
// magnitude does not depend on which of the two roots s(t) actually is — so
// this never needs to pick the branch nearest the corner the way
// intersectOffsets' own POSITION solve does; both roots move at the same
// speed. Δ affine means its minimum over [t0, t1] is at one of the two
// ends, so bounding it needs no search.
//
// ok is false whenever Δ's own enclosure reaches or crosses zero somewhere
// in [t0, t1], WITHOUT Δ1's own enclosure being exactly zero (a momentary
// fold within this range), or any enclosure fails.
//
// A TANGENT join this evaluator itself built — Fillet's own corner rewrite
// (fillet.go), which is exactly what a rounded-rectangle wall's corner is —
// lands Δ1 at EXACTLY zero, bit for bit: the tangent condition
// α = −inside·R the construction holds by is stated in the SAME floats this
// derivation reads, with no residual from a numerical solve to round away.
// A corner recorded through sketch's own solver need not be so exact, and
// Δ1's enclosure straddling (rather than sitting AT) zero there still falls
// through to the general bound below, which is sound but can refuse on a
// near-tangent corner no public fixture reaches today — the PR body for
// this change names that as a known limitation.
func lineCircleLocusSpeedUpper(line, circle sideWalk, t0, t1 float64) (float64, bool) {
	frame, ok := lineWallFrameOf(line)
	if !ok {
		return 0, false
	}
	cx, cy := floatRat(circle.cU), floatRat(circle.cV)
	radius := floatRat(circle.radius)
	anchorU, anchorV := floatRat(frame.anchorU), floatRat(frame.anchorV)
	if cx == nil || cy == nil || radius == nil || anchorU == nil || anchorV == nil {
		return 0, false
	}
	w0u := new(big.Rat).Sub(anchorU, cx)
	w0v := new(big.Rat).Sub(anchorV, cy)
	alpha := intervalAdd(intervalMul(pointInterval(w0u), frame.n.u), intervalMul(pointInterval(w0v), frame.n.v))
	inside := insideSignOf(circle)

	// Delta(t) = (R^2 - alpha^2) - 2*(alpha + inside*R)*t = delta0 + delta1*t.
	delta0 := intervalSub(intervalSquare(pointInterval(radius)), intervalSquare(alpha))
	delta1 := intervalScale(intervalAdd(alpha, intervalScale(pointInterval(radius), inside)), big.NewRat(-2, 1))

	// Delta1 == 0 EXACTLY (both ends of its own enclosure) is the persistent-
	// tangency closed form this function's own doc comment derives: s(t) is
	// then constant, so X'(t) = n exactly and no square root — nor Delta0's
	// own sign, which this branch never even reads — enters the answer. The
	// bound is |n|'s own enclosed magnitude (n is unit by construction, so
	// this is 1 up to ivUnitVec's own tiny sqrt rounding) rather than
	// radius2D's √2-scaled one, since this specific case is common enough —
	// every tangent-filleted corner in this codebase's own test fixtures —
	// to be worth the tighter bound.
	if delta1.lo.Sign() == 0 && delta1.hi.Sign() == 0 {
		nMagUpper := ratSqrtUp(intervalAbsUpper(intervalAdd(intervalSquare(frame.n.u), intervalSquare(frame.n.v))))
		if isNonFinite(nMagUpper) {
			return 0, false
		}
		return nMagUpper, true
	}

	rt0, rt1 := floatRat(t0), floatRat(t1)
	if rt0 == nil || rt1 == nil {
		return 0, false
	}
	deltaAt := func(t *big.Rat) ratInterval {
		return intervalAdd(delta0, intervalMul(delta1, pointInterval(t)))
	}
	d0, d1 := deltaAt(rt0), deltaAt(rt1)
	// Delta is affine, so its minimum over [t0, t1] is at one of the two
	// ends — no interior point needs checking.
	deltaMinLo := d0.lo
	if d1.lo.Cmp(deltaMinLo) < 0 {
		deltaMinLo = d1.lo
	}
	if deltaMinLo.Sign() <= 0 {
		// The discriminant reaches or crosses zero somewhere this enclosure
		// cannot rule out: a genuine fold inside this range, where the
		// position solve's own branch can turn without bound.
		return 0, false
	}
	sqrtLower := ratSqrtDown(deltaMinLo)
	if sqrtLower <= 0 || isNonFinite(sqrtLower) {
		return 0, false
	}
	delta1Upper := intervalAbsUpper(delta1)
	delta1UpperF, exact := delta1Upper.Float64()
	if !exact {
		delta1UpperF = math.Nextafter(delta1UpperF, math.Inf(1))
	}
	sPrimeUpper := upRound(delta1UpperF / (2 * sqrtLower))
	if isNonFinite(sPrimeUpper) {
		return 0, false
	}
	// |dP/dt| = sqrt(1 + s'(t)^2), since n and e are orthonormal.
	upper := radius2D(1, sPrimeUpper)
	if isNonFinite(upper) {
		return 0, false
	}
	return upper, true
}

// miterLocusSpeedUpper bounds |dP/dt| — the in-plane speed of the corner
// foot's own denoted locus (docs/modify-reach-design.md §8.3's exact offset
// family) — over the offset range [t0, t1], for the corner where prev's own
// offset carrier meets cur's, dispatching to the exact closed form
// (lineCircleLocusSpeedUpper) where one carrier is straight and the other
// circular, or the generic interval solve (circleCircleLocusSpeedUpper)
// where both are circular. At least one of prev, cur must be circular — the
// caller (capSlantEdge) never reaches here for a line-line miter, whose
// locus is already exact.
func miterLocusSpeedUpper(prev, cur sideWalk, t0, t1, vU, vV float64) (float64, bool) {
	switch {
	case !prev.isCircular() && cur.isCircular():
		return lineCircleLocusSpeedUpper(prev, cur, t0, t1)
	case prev.isCircular() && !cur.isCircular():
		return lineCircleLocusSpeedUpper(cur, prev, t0, t1)
	default:
		return circleCircleLocusSpeedUpper(prev, cur, t0, t1, vU, vV)
	}
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
			twoPiInterval(),
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
	circumference := intervalScale(twoPiInterval(), exactRadius)
	return intervalFloatError(circumference, held)
}

// capSweepBracket is the atan2Interval enclosure of a cap-level directrix's
// swept angle — atan2(end−centre) − atan2(start−centre), unwrapped by
// wraps·2π to the same branch capWallSweep's own float computation picked —
// plus the coordinate shift (the contour's own displacement, folded in by
// the caller, plus each endpoint's own subtraction rounding) that a caller
// turns into an allowance for how far those feet may sit from the point the
// offset denotes. It is the ONE bracket capWallArcBound (a length bound,
// scaled by the wall's own radius) and capSweepAllow (an angle bound, read
// directly) both build from, so the two readers of one wall's cap-level
// sweep are never told two different enclosures of it.
func capSweepBracket(cU, cV float64, start, end Point2, wraps int, delta float64) (ratInterval, float64, bool) {
	aU, aV := floatRat(start.U-cU), floatRat(start.V-cV)
	bU, bV := floatRat(end.U-cU), floatRat(end.V-cV)
	if aU == nil || aV == nil || bU == nil || bV == nil {
		return ratInterval{}, 0, false
	}
	sweep := intervalSub(atan2Interval(bV, bU, false), atan2Interval(aV, aU, false))
	if wraps != 0 {
		sweep = intervalAdd(sweep, intervalScale(
			twoPiInterval(),
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
	return sweep, shift, true
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
	sweep, shift, ok := capSweepBracket(cU, cV, start, end, wraps, delta)
	if !ok {
		return fallback
	}
	rd := floatRat(capRadius)
	if rd == nil {
		return fallback
	}
	turn, ok := arcSweepAllow(downRound(math.Abs(capRadius)), shift)
	if !ok {
		return fallback
	}
	bound := absSumUpper(intervalFloatError(intervalScale(sweep, rd), held), turn)
	return math.Min(bound, fallback)
}

// capSweepAllow bounds |held − trueSweep| for a cap-level directrix's own RAW
// swept angle in radians — capSweepBracket's same enclosure, reported against
// the raw sweep rather than against radius·sweep, so patchAreaOf's Δθ factor
// (the frustum-sector area formula's own sweep) and capWallArcBound's length
// bound both read one proven enclosure of the same sweep
// (docs/modify-reach-design.md §8.4). radius is the circle the two feet lie
// on (capRadius for a regular wall's cap contour, d for a reflex corner's
// connector) and is used only to turn the feet's own contour displacement
// into the angular turn a foot at that radius can still account for:
// arcSweepAllow's own arc-LENGTH allowance (independent of which radius it is
// stated against, by its own derivation — see arcSweepAllow's doc comment)
// divided by that same radius restates it in radians, rounded down before
// dividing so the allowance can only widen, never tighten.
func capSweepAllow(cU, cV, radius float64, start, end Point2, held float64, wraps int, delta float64) float64 {
	fallback := conservativeValueError(held, twoPiUpper())
	sweep, shift, ok := capSweepBracket(cU, cV, start, end, wraps, delta)
	if !ok {
		return fallback
	}
	r := downRound(math.Abs(radius))
	lengthTurn, ok := arcSweepAllow(r, shift)
	if !ok {
		return fallback
	}
	angularTurn := upRound(lengthTurn / r)
	bound := absSumUpper(intervalFloatError(sweep, held), angularTurn)
	return math.Min(bound, fallback)
}
