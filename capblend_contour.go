package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

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

// ivPoint is a rational-interval enclosure of one plane-local (u, v) point.
type ivPoint struct{ u, v proofbound.RatInterval }

// ivExactPoint lifts a pair of float64 coordinates, which are exact rationals.
func ivExactPoint(u, v float64) (ivPoint, bool) {
	ru, rv := proofarith.FloatRat(u), proofarith.FloatRat(v)
	if ru == nil || rv == nil {
		return ivPoint{}, false
	}
	return ivPoint{u: proofbound.PointInterval(ru), v: proofbound.PointInterval(rv)}, true
}

// reach is an upper bound on |p − q| over every q the enclosure holds, so a
// point known to lie in the enclosure sits at most this far from p.
func (e ivPoint) reach(p Point2) float64 {
	du, okU := ivAxisSpread(e.u, p.U)
	dv, okV := ivAxisSpread(e.v, p.V)
	if !okU || !okV {
		return math.Inf(1)
	}
	return proofbound.RatSqrtUp(new(big.Rat).Add(
		new(big.Rat).Mul(du, du),
		new(big.Rat).Mul(dv, dv),
	))
}

// ivAxisSpread is max(|lo − c|, |hi − c|), the furthest the interval reaches
// from c along one axis.
func ivAxisSpread(iv proofbound.RatInterval, c float64) (*big.Rat, bool) {
	rc := proofarith.FloatRat(c)
	if rc == nil || iv.Lo == nil || iv.Hi == nil {
		return nil, false
	}
	lo := new(big.Rat).Abs(new(big.Rat).Sub(iv.Lo, rc))
	hi := new(big.Rat).Abs(new(big.Rat).Sub(iv.Hi, rc))
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

func intervalHull(a, b proofbound.RatInterval) proofbound.RatInterval {
	lo, hi := a.Lo, a.Hi
	if b.Lo.Cmp(lo) < 0 {
		lo = b.Lo
	}
	if b.Hi.Cmp(hi) > 0 {
		hi = b.Hi
	}
	return proofbound.Interval(lo, hi)
}

// intervalQuo divides two intervals. ok is false where the divisor straddles
// zero, since the quotient is then unbounded and no box encloses it.
func intervalQuo(a, b proofbound.RatInterval) (proofbound.RatInterval, bool) {
	if b.Lo.Sign() <= 0 && b.Hi.Sign() >= 0 {
		return proofbound.RatInterval{}, false
	}
	corners := [4]*big.Rat{
		new(big.Rat).Quo(a.Lo, b.Lo),
		new(big.Rat).Quo(a.Lo, b.Hi),
		new(big.Rat).Quo(a.Hi, b.Lo),
		new(big.Rat).Quo(a.Hi, b.Hi),
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
	return proofbound.Interval(lo, hi), true
}

// intervalSquare is x² over an interval. It is not proofbound.IntervalMul(a, a): the
// corner products of an interval straddling zero put a NEGATIVE value at the
// low end, and a square never takes one.
func intervalSquare(a proofbound.RatInterval) proofbound.RatInterval {
	lo2 := new(big.Rat).Mul(a.Lo, a.Lo)
	hi2 := new(big.Rat).Mul(a.Hi, a.Hi)
	if a.Lo.Sign() >= 0 {
		return proofbound.Interval(lo2, hi2)
	}
	if a.Hi.Sign() <= 0 {
		return proofbound.Interval(hi2, lo2)
	}
	hi := lo2
	if hi2.Cmp(hi) > 0 {
		hi = hi2
	}
	return proofbound.Interval(new(big.Rat), hi)
}

// intervalSqrt encloses the square root over a non-negative interval, each end
// rounded OUTWARD through spline_length.go's exact-comparison bracket, so no
// platform's sqrt can narrow it. A lower end below zero is clamped to zero:
// the enclosure then covers the tangency the float discriminant reached for.
func intervalSqrt(a proofbound.RatInterval) (proofbound.RatInterval, bool) {
	if a.Hi.Sign() < 0 {
		return proofbound.RatInterval{}, false
	}
	lo := 0.0
	if a.Lo.Sign() > 0 {
		lo = proofbound.RatSqrtDown(a.Lo)
	}
	hi := proofbound.RatSqrtUp(a.Hi)
	rlo, rhi := proofarith.FloatRat(lo), proofarith.FloatRat(hi)
	if rlo == nil || rhi == nil {
		return proofbound.RatInterval{}, false
	}
	return proofbound.Interval(rlo, rhi), true
}

// ivUnitVec encloses the EXACT unit vector of a float pair — the value
// normalize2 rounds. The pair itself is exact, so the only widening is the
// length's own outward-rounded square root.
func ivUnitVec(x, y float64) (ivPoint, bool) {
	rx, ry := proofarith.FloatRat(x), proofarith.FloatRat(y)
	if rx == nil || ry == nil {
		return ivPoint{}, false
	}
	n2 := new(big.Rat).Add(new(big.Rat).Mul(rx, rx), new(big.Rat).Mul(ry, ry))
	if n2.Sign() == 0 {
		return ivPoint{}, false
	}
	l, ok := intervalSqrt(proofbound.PointInterval(n2))
	if !ok || l.Lo.Sign() <= 0 {
		return ivPoint{}, false
	}
	u, okU := intervalQuo(proofbound.PointInterval(rx), l)
	v, okV := intervalQuo(proofbound.PointInterval(ry), l)
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
	rd := proofarith.FloatRat(d)
	if !okV || rd == nil {
		return ivPoint{}, false
	}
	return ivPoint{
		u: proofbound.IntervalAdd(v.u, proofbound.IntervalScale(proofbound.IntervalNeg(n.v), rd)),
		v: proofbound.IntervalAdd(v.v, proofbound.IntervalScale(n.u, rd)),
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
	r      proofbound.RatInterval
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
	return ivCarrier{c: c, r: proofbound.PointInterval(r)}, true
}

// ivExactOffsetRadius is offsetRadius's own R − insideSign·d taken EXACTLY:
// both operands are float64s, so their difference is a rational with no
// rounding at all, and the float the build holds is the rounding of THIS value.
func ivExactOffsetRadius(w sideWalk, d float64) (*big.Rat, bool) {
	inside := 1.0
	if w.th1 < w.th0 { // a clockwise walk has its material outside the circle
		inside = -1.0
	}
	rr, rd := proofarith.FloatRat(w.radius), proofarith.FloatRat(inside*d)
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
	den := proofbound.IntervalSub(proofbound.IntervalMul(a.dir.u, b.dir.v), proofbound.IntervalMul(a.dir.v, b.dir.u))
	num := proofbound.IntervalSub(
		proofbound.IntervalMul(proofbound.IntervalSub(b.p.u, a.p.u), b.dir.v),
		proofbound.IntervalMul(proofbound.IntervalSub(b.p.v, a.p.v), b.dir.u),
	)
	s, ok := intervalQuo(num, den)
	if !ok {
		return nil, false
	}
	return []ivPoint{{
		u: proofbound.IntervalAdd(a.p.u, proofbound.IntervalMul(s, a.dir.u)),
		v: proofbound.IntervalAdd(a.p.v, proofbound.IntervalMul(s, a.dir.v)),
	}}, true
}

func ivLineCircle(l, c ivCarrier) ([]ivPoint, bool) {
	fx := proofbound.IntervalSub(l.p.u, c.c.u)
	fy := proofbound.IntervalSub(l.p.v, c.c.v)
	bb := proofbound.IntervalAdd(proofbound.IntervalMul(fx, l.dir.u), proofbound.IntervalMul(fy, l.dir.v))
	cc := proofbound.IntervalSub(proofbound.IntervalAdd(intervalSquare(fx), intervalSquare(fy)), intervalSquare(c.r))
	disc := proofbound.IntervalSub(intervalSquare(bb), cc)
	if disc.Hi.Sign() < 0 {
		// The exact carriers miss each other entirely: the float solve reached
		// a root of a system that has none, so there is no denoted point to
		// enclose.
		return nil, false
	}
	sq, ok := intervalSqrt(disc)
	if !ok {
		return nil, false
	}
	nb := proofbound.IntervalNeg(bb)
	out := make([]ivPoint, 0, 2)
	for _, s := range []proofbound.RatInterval{proofbound.IntervalAdd(nb, sq), proofbound.IntervalSub(nb, sq)} {
		out = append(out, ivPoint{
			u: proofbound.IntervalAdd(l.p.u, proofbound.IntervalMul(s, l.dir.u)),
			v: proofbound.IntervalAdd(l.p.v, proofbound.IntervalMul(s, l.dir.v)),
		})
	}
	return out, true
}

func ivCircleCircle(a, b ivCarrier) ([]ivPoint, bool) {
	dx := proofbound.IntervalSub(b.c.u, a.c.u)
	dy := proofbound.IntervalSub(b.c.v, a.c.v)
	dsq := proofbound.IntervalAdd(intervalSquare(dx), intervalSquare(dy))
	dist, ok := intervalSqrt(dsq)
	if !ok || dist.Lo.Sign() <= 0 {
		return nil, false
	}
	mid, okMid := intervalQuo(
		proofbound.IntervalSub(proofbound.IntervalAdd(dsq, intervalSquare(a.r)), intervalSquare(b.r)),
		proofbound.IntervalScale(dist, big.NewRat(2, 1)),
	)
	if !okMid {
		return nil, false
	}
	h2 := proofbound.IntervalSub(intervalSquare(a.r), intervalSquare(mid))
	if h2.Hi.Sign() < 0 {
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
	baseU := proofbound.IntervalAdd(a.c.u, proofbound.IntervalMul(along, dx))
	baseV := proofbound.IntervalAdd(a.c.v, proofbound.IntervalMul(along, dy))
	offU := proofbound.IntervalMul(across, dy)
	offV := proofbound.IntervalMul(across, dx)
	return []ivPoint{
		{u: proofbound.IntervalSub(baseU, offU), v: proofbound.IntervalAdd(baseV, offV)},
		{u: proofbound.IntervalAdd(baseU, offU), v: proofbound.IntervalSub(baseV, offV)},
	}, true
}

// ivNearest encloses intersectOffsets' own "root nearest the corner". A
// candidate whose squared-distance interval starts beyond another's end is
// PROVEN not to be the nearest and is dropped; every candidate the exact
// arithmetic leaves undecided joins the hull, so a near-tangency reports one
// wide honest displacement rather than a branch nothing decided.
func ivNearest(cands []ivPoint, vU, vV float64) (ivPoint, bool) {
	corner, ok := ivExactPoint(vU, vV)
	if !ok {
		return ivPoint{}, false
	}
	return ivNearestTo(cands, corner)
}

// ivNearestTo is ivNearest about an enclosed corner: a candidate is dropped
// only when its squared distance to EVERY corner the box holds is proven
// beyond another candidate's farthest.
func ivNearestTo(cands []ivPoint, corner ivPoint) (ivPoint, bool) {
	if len(cands) == 0 {
		return ivPoint{}, false
	}
	d2 := make([]proofbound.RatInterval, len(cands))
	for i, c := range cands {
		d2[i] = proofbound.IntervalAdd(
			intervalSquare(proofbound.IntervalSub(c.u, corner.u)),
			intervalSquare(proofbound.IntervalSub(c.v, corner.v)),
		)
	}
	best := d2[0].Hi
	for _, iv := range d2[1:] {
		if iv.Hi.Cmp(best) < 0 {
			best = iv.Hi
		}
	}
	var out ivPoint
	found := false
	for i, iv := range d2 {
		if iv.Lo.Cmp(best) > 0 {
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
		delta = math.Max(delta, proofarith.RationalFloatError(exact, held))
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
		if j.g1 {
			// A G1 join intersects no carriers (modify §7; modify-reach §8.4): its
			// denoted corner is v + d·n̂ for the leaving wall's exact unit normal, and
			// the enclosure is the HULL of the two shared-normal feet, so a join the
			// dead zone classified G1 with a residual turn is charged the spread
			// between the two normals it could have taken.
			a, okA := ivOffsetFoot(j.vU, j.vV, prev.tanOutU, prev.tanOutV, d)
			b, okB := ivOffsetFoot(j.vU, j.vV, cur.tanInU, cur.tanInV, d)
			if !okA || !okB {
				return 0, errCapContourUnbounded
			}
			delta = math.Max(delta, ivUnion(a, b).reach(j.m))
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
	if proofbound.IsNonFinite(delta) {
		return 0, errCapContourUnbounded
	}
	return delta, nil
}

// ivOffsetFootRange generalises ivOffsetFoot to an OFFSET INTERVAL rather
// than one float: the enclosure of v + t·rot90(unit(t)) for every offset
// amount t in tRange, the point family a line carrier's own anchor sweeps as
// the offset amount varies. ivOffsetFoot is this at one degenerate point.
func ivOffsetFootRange(vU, vV, tu, tv float64, tRange proofbound.RatInterval) (ivPoint, bool) {
	n, ok := ivUnitVec(tu, tv)
	if !ok {
		return ivPoint{}, false
	}
	v, okV := ivExactPoint(vU, vV)
	if !okV {
		return ivPoint{}, false
	}
	return ivPoint{
		u: proofbound.IntervalAdd(v.u, proofbound.IntervalMul(proofbound.IntervalNeg(n.v), tRange)),
		v: proofbound.IntervalAdd(v.v, proofbound.IntervalMul(n.u, tRange)),
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
	rt0, rt1 := proofarith.FloatRat(t0), proofarith.FloatRat(t1)
	if rt0 == nil || rt1 == nil {
		return ivCarrier{}, false
	}
	tRange := proofbound.Interval(rt0, rt1)
	if !w.isCircular() {
		p, okP := ivOffsetFootRange(w.startU, w.startV, w.tanInU, w.tanInV, tRange)
		dir, okD := ivUnitVec(w.tanInU, w.tanInV)
		if !okP || !okD {
			return ivCarrier{}, false
		}
		return ivCarrier{isLine: true, p: p, dir: dir}, true
	}
	rr := proofarith.FloatRat(w.radius)
	if rr == nil {
		return ivCarrier{}, false
	}
	inside := big.NewRat(1, 1)
	if w.th1 < w.th0 { // a clockwise walk has its material outside the circle
		inside = big.NewRat(-1, 1)
	}
	r := proofbound.IntervalSub(proofbound.PointInterval(rr), proofbound.IntervalMul(tRange, proofbound.PointInterval(inside)))
	if r.Lo.Sign() <= 0 {
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
		n := ivPoint{u: proofbound.IntervalNeg(c.dir.v), v: c.dir.u}
		return n, big.NewRat(1, 1), true
	}
	dx := proofbound.IntervalSub(foot.u, c.c.u)
	dy := proofbound.IntervalSub(foot.v, c.c.v)
	distSq := proofbound.IntervalAdd(intervalSquare(dx), intervalSquare(dy))
	dist, ok := intervalSqrt(distSq)
	if !ok || dist.Lo.Sign() <= 0 {
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
	det := proofbound.IntervalSub(proofbound.IntervalMul(rowA.u, rowB.v), proofbound.IntervalMul(rowB.u, rowA.v))
	if det.Lo.Sign() <= 0 && det.Hi.Sign() >= 0 {
		return 0, false
	}
	pu, okU := intervalQuo(proofbound.IntervalSub(proofbound.IntervalScale(rowB.v, rhsA), proofbound.IntervalScale(rowA.v, rhsB)), det)
	pv, okV := intervalQuo(proofbound.IntervalSub(proofbound.IntervalScale(rowA.u, rhsB), proofbound.IntervalScale(rowB.u, rhsA)), det)
	if !okU || !okV {
		return 0, false
	}
	mag, ok := intervalSqrt(proofbound.IntervalAdd(intervalSquare(pu), intervalSquare(pv)))
	if !ok {
		return 0, false
	}
	upper, exact := mag.Hi.Float64()
	if !exact {
		upper = math.Nextafter(upper, math.Inf(1))
	}
	if proofbound.IsNonFinite(upper) || upper < 0 {
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
	n := ivPoint{u: proofbound.IntervalNeg(e.v), v: e.u}
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
	cx, cy := proofarith.FloatRat(circle.cU), proofarith.FloatRat(circle.cV)
	radius := proofarith.FloatRat(circle.radius)
	anchorU, anchorV := proofarith.FloatRat(frame.anchorU), proofarith.FloatRat(frame.anchorV)
	if cx == nil || cy == nil || radius == nil || anchorU == nil || anchorV == nil {
		return 0, false
	}
	w0u := new(big.Rat).Sub(anchorU, cx)
	w0v := new(big.Rat).Sub(anchorV, cy)
	alpha := proofbound.IntervalAdd(proofbound.IntervalMul(proofbound.PointInterval(w0u), frame.n.u), proofbound.IntervalMul(proofbound.PointInterval(w0v), frame.n.v))
	inside := insideSignOf(circle)

	// Delta(t) = (R^2 - alpha^2) - 2*(alpha + inside*R)*t = delta0 + delta1*t.
	delta0 := proofbound.IntervalSub(intervalSquare(proofbound.PointInterval(radius)), intervalSquare(alpha))
	delta1 := proofbound.IntervalScale(proofbound.IntervalAdd(alpha, proofbound.IntervalScale(proofbound.PointInterval(radius), inside)), big.NewRat(-2, 1))

	// Delta1 == 0 EXACTLY (both ends of its own enclosure) is the persistent-
	// tangency closed form this function's own doc comment derives: s(t) is
	// then constant, so X'(t) = n exactly and no square root — nor Delta0's
	// own sign, which this branch never even reads — enters the answer. The
	// bound is |n|'s own enclosed magnitude (n is unit by construction, so
	// this is 1 up to ivUnitVec's own tiny sqrt rounding) rather than
	// proofbound.Radius2D's √2-scaled one, since this specific case is common enough —
	// every tangent-filleted corner in this codebase's own test fixtures —
	// to be worth the tighter bound.
	if delta1.Lo.Sign() == 0 && delta1.Hi.Sign() == 0 {
		nMagUpper := proofbound.RatSqrtUp(intervalAbsUpper(proofbound.IntervalAdd(intervalSquare(frame.n.u), intervalSquare(frame.n.v))))
		if proofbound.IsNonFinite(nMagUpper) {
			return 0, false
		}
		return nMagUpper, true
	}

	rt0, rt1 := proofarith.FloatRat(t0), proofarith.FloatRat(t1)
	if rt0 == nil || rt1 == nil {
		return 0, false
	}
	deltaAt := func(t *big.Rat) proofbound.RatInterval {
		return proofbound.IntervalAdd(delta0, proofbound.IntervalMul(delta1, proofbound.PointInterval(t)))
	}
	d0, d1 := deltaAt(rt0), deltaAt(rt1)
	// Delta is affine, so its minimum over [t0, t1] is at one of the two
	// ends — no interior point needs checking.
	deltaMinLo := d0.Lo
	if d1.Lo.Cmp(deltaMinLo) < 0 {
		deltaMinLo = d1.Lo
	}
	if deltaMinLo.Sign() <= 0 {
		// The discriminant reaches or crosses zero somewhere this enclosure
		// cannot rule out: a genuine fold inside this range, where the
		// position solve's own branch can turn without bound.
		return 0, false
	}
	sqrtLower := proofbound.RatSqrtDown(deltaMinLo)
	if sqrtLower <= 0 || proofbound.IsNonFinite(sqrtLower) {
		return 0, false
	}
	delta1Upper := intervalAbsUpper(delta1)
	delta1UpperF, exact := delta1Upper.Float64()
	if !exact {
		delta1UpperF = math.Nextafter(delta1UpperF, math.Inf(1))
	}
	sPrimeUpper := proofbound.UpRound(delta1UpperF / (2 * sqrtLower))
	if proofbound.IsNonFinite(sPrimeUpper) {
		return 0, false
	}
	// |dP/dt| = sqrt(1 + s'(t)^2), since n and e are orthonormal.
	upper := proofbound.Radius2D(1, sPrimeUpper)
	if proofbound.IsNonFinite(upper) {
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
	return proofarith.RationalFloatError(exact, held), nil
}

// loopContourDelta re-derives one loop's contour displacement from the loop
// record alone, for the readings that hold no built band — the payload's own
// extentAlong, which evaluates the same contour through capLoopBoundary. It
// walks and joins the loop exactly as buildCapBand does, so the two can never
// disagree about the same contour.
func loopContourDelta(ctx context.Context, loop LoopRecord, d float64) (float64, error) {
	budget := proofbound.NewWorkBudget(ctx)
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

// dySquaredDistance3 is the exact squared distance between two points, every
// coordinate a float64 and hence an exact dyadic, so the returned value is the
// true square of the length the float evaluation approximated.
// dySqrtIntervalError then reports what that evaluation committed. ok is false
// where a coordinate is not finite, which states no distance at all.
func dySquaredDistance3(a0, a1, a2, b0, b1, b2 float64) (proofarith.Dyadic, bool) {
	sum := proofarith.DyZero()
	for _, pair := range [3][2]float64{{a0, b0}, {a1, b1}, {a2, b2}} {
		x, okX := proofarith.DyOf(pair[0])
		y, okY := proofarith.DyOf(pair[1])
		if !okX || !okY {
			return proofarith.Dyadic{}, false
		}
		diff := proofarith.DySubScalar(x, y)
		sum = proofarith.DyAdd(sum, proofarith.DyMul(diff, diff))
	}
	return sum, true
}

// ratSquaredDistance3 is dySquaredDistance3 as a big.Rat, for the callers that
// go on to divide by it or compare it against a general fraction. It answers
// nil where a coordinate is not finite.
func ratSquaredDistance3(a0, a1, a2, b0, b1, b2 float64) *big.Rat {
	d, ok := dySquaredDistance3(a0, a1, a2, b0, b1, b2)
	if !ok {
		return nil
	}
	return d.Rat()
}

// straightEdgeBound is the proven bound on a straight cap-level edge's held
// length. It has three independent terms and each speaks for a different thing:
// the square root's own committed error, measured against the exact squared
// length (dySquaredDistance3) rather than against a Hypot ulp contract Go does
// not give; and one displacement per endpoint, since moving an endpoint of a
// segment by e moves its length by at most e. ok false — a squared length the
// coordinates could not state — is an underivable bound, +Inf.
//
// A non-negative held length whose exact square IS the squared length
// (dySquareEquals, an exact dyadic comparison) is the true length, so its
// square-root term is zero and the bracket is not built. A negative held
// length never takes that shortcut: its square can match while the length
// itself is off by twice its magnitude, and the bracket measures that gap.
func straightEdgeBound(held float64, squared proofarith.Dyadic, ok bool, endpointDeltas ...float64) float64 {
	if !ok {
		return math.Inf(1)
	}
	sqrtErr := 0.0
	if held < 0 || !proofarith.DySquareEquals(held, squared) {
		sqrtErr = dySqrtIntervalError(squared, held)
	}
	return proofbound.AbsSumUpper(append([]float64{sqrtErr}, endpointDeltas...)...)
}

// capEdgeLengthBound is straightEdgeBound for a straight cap-level edge between
// two contour points, each displaced by the band's own delta.
func capEdgeLengthBound(held float64, end, start Point2, delta float64) float64 {
	squared, ok := dySquaredDistance3(end.U, end.V, 0, start.U, start.V, 0)
	return straightEdgeBound(held, squared, ok, delta, delta)
}

// arcSweepAllow converts a contour displacement into the arc length it can move.
// A foot displaced by delta on a circle of radius r turns through at most
// arcsin(delta/r) ≤ (π/2)·delta/r, so the arc between two such feet changes
// length by at most r·2·(π/2)·delta/r = π·delta — independent of the radius.
// A displacement at or past the radius says nothing about the turn at all, and
// the caller then owes the whole-circumference envelope instead.
func arcSweepAllow(radius, delta float64) (float64, bool) {
	if radius <= 0 || delta >= radius || proofbound.IsNonFinite(radius) || proofbound.IsNonFinite(delta) {
		return 0, false
	}
	return proofbound.ProductUpper(math.Nextafter(math.Pi, math.Inf(1)), delta), true
}

// capApexArcBound bounds the reflex connector arc's held length d·(th0 − th1).
// The arc's centre is the ORIGINAL corner and its radius is exactly the
// setback, both recorded, so the only error is in the sweep: the exact turn
// between the two feet the build actually holds (an proofbound.Atan2Interval bracket, so
// no libm accuracy is assumed) plus the turn those feet's own displacement can
// account for.
func capApexArcBound(j cornerJoin, d, held float64, wraps int, delta float64) float64 {
	fallback := proofbound.ConservativeValueError(held, proofbound.ProductUpper(proofbound.TwoPiUpper(), math.Abs(d)))
	aU, aV := proofarith.FloatRat(j.pA.U-j.vU), proofarith.FloatRat(j.pA.V-j.vV)
	bU, bV := proofarith.FloatRat(j.pB.U-j.vU), proofarith.FloatRat(j.pB.V-j.vV)
	rd := proofarith.FloatRat(d)
	if aU == nil || aV == nil || bU == nil || bV == nil || rd == nil {
		return fallback
	}
	sweep := proofbound.IntervalSub(proofbound.Atan2Interval(aV, aU, false), proofbound.Atan2Interval(bV, bU, false))
	if wraps != 0 {
		sweep = proofbound.IntervalAdd(sweep, proofbound.IntervalScale(
			proofbound.TwoPiInterval(),
			big.NewRat(int64(wraps), 1),
		))
	}
	// The build's own float differences round, and that rounding displaces the
	// direction the angle is read from just as the contour itself does.
	shift := proofbound.AbsSumUpper(
		delta,
		proofarith.AddRoundError(j.pA.U, -j.vU, j.pA.U-j.vU),
		proofarith.AddRoundError(j.pA.V, -j.vV, j.pA.V-j.vV),
		proofarith.AddRoundError(j.pB.U, -j.vU, j.pB.U-j.vU),
		proofarith.AddRoundError(j.pB.V, -j.vV, j.pB.V-j.vV),
	)
	turn, ok := arcSweepAllow(d, shift)
	if !ok {
		return fallback
	}
	bound := proofbound.AbsSumUpper(proofbound.IntervalFloatError(proofbound.IntervalScale(sweep, rd), held), turn)
	return math.Min(bound, fallback)
}

// capCircleLengthBound bounds a whole cap-level circle's held 2πr against the
// EXACT offset radius: π is bracketed by moments.go's own rational constants,
// so the enclosure needs no float value of π and no libm accuracy.
func capCircleLengthBound(exactRadius *big.Rat, held float64) float64 {
	if exactRadius == nil {
		return math.Inf(1)
	}
	circumference := proofbound.IntervalScale(proofbound.TwoPiInterval(), exactRadius)
	return proofbound.IntervalFloatError(circumference, held)
}

// capSweepBracket is the proofbound.Atan2Interval enclosure of a cap-level directrix's
// swept angle — atan2(end−centre) − atan2(start−centre), unwrapped by
// wraps·2π to the same branch capWallSweep's own float computation picked —
// plus the coordinate shift (the contour's own displacement, folded in by
// the caller, plus each endpoint's own subtraction rounding) that a caller
// turns into an allowance for how far those feet may sit from the point the
// offset denotes. It is the ONE bracket capWallArcBound (a length bound,
// scaled by the wall's own radius) and capSweepAllow (an angle bound, read
// directly) both build from, so the two readers of one wall's cap-level
// sweep are never told two different enclosures of it.
func capSweepBracket(cU, cV float64, start, end Point2, wraps int, delta float64) (proofbound.RatInterval, float64, bool) {
	aU, aV := proofarith.FloatRat(start.U-cU), proofarith.FloatRat(start.V-cV)
	bU, bV := proofarith.FloatRat(end.U-cU), proofarith.FloatRat(end.V-cV)
	if aU == nil || aV == nil || bU == nil || bV == nil {
		return proofbound.RatInterval{}, 0, false
	}
	sweep := proofbound.IntervalSub(proofbound.Atan2Interval(bV, bU, false), proofbound.Atan2Interval(aV, aU, false))
	if wraps != 0 {
		sweep = proofbound.IntervalAdd(sweep, proofbound.IntervalScale(
			proofbound.TwoPiInterval(),
			big.NewRat(int64(wraps), 1),
		))
	}
	// The build's own float differences round, and that rounding displaces the
	// direction the angle is read from just as the contour itself does.
	shift := proofbound.AbsSumUpper(
		delta,
		proofarith.AddRoundError(start.U, -cU, start.U-cU),
		proofarith.AddRoundError(start.V, -cV, start.V-cV),
		proofarith.AddRoundError(end.U, -cU, end.U-cU),
		proofarith.AddRoundError(end.V, -cV, end.V-cV),
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
// capApexArcBound brackets a reflex corner's own connector: an proofbound.Atan2Interval
// enclosure of the two feet's own turn about the centre, so no libm accuracy
// is assumed of the sweep itself, plus wraps (capWallSweep's own unwrap count)
// to reproduce the same branch, plus the turn the two feet's own contour
// displacement can account for.
func capWallArcBound(cU, cV float64, start, end Point2, capRadius, held float64, wraps int, delta float64) float64 {
	fallback := proofbound.ConservativeValueError(held, proofbound.ProductUpper(proofbound.TwoPiUpper(), math.Abs(capRadius)))
	sweep, shift, ok := capSweepBracket(cU, cV, start, end, wraps, delta)
	if !ok {
		return fallback
	}
	rd := proofarith.FloatRat(capRadius)
	if rd == nil {
		return fallback
	}
	turn, ok := arcSweepAllow(downRound(math.Abs(capRadius)), shift)
	if !ok {
		return fallback
	}
	bound := proofbound.AbsSumUpper(proofbound.IntervalFloatError(proofbound.IntervalScale(sweep, rd), held), turn)
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
	fallback := proofbound.ConservativeValueError(held, proofbound.TwoPiUpper())
	sweep, shift, ok := capSweepBracket(cU, cV, start, end, wraps, delta)
	if !ok {
		return fallback
	}
	r := downRound(math.Abs(radius))
	lengthTurn, ok := arcSweepAllow(r, shift)
	if !ok {
		return fallback
	}
	angularTurn := proofbound.UpRound(lengthTurn / r)
	bound := proofbound.AbsSumUpper(proofbound.IntervalFloatError(sweep, held), angularTurn)
	return math.Min(bound, fallback)
}
