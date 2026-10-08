package offset2d

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ErrOpeningCorner is docs/shell-opening-design.md's SO1: a side opening's end
// corner is smooth or cusped, or the removed walk's carrier, walked from the
// corner in the direction that enters the kept wall's band, never meets the
// kept walk's offset carrier before it leaves the band again.
var ErrOpeningCorner = fmt.Errorf(`%w: a side opening's end corner is smooth or cusped, or the removed face's carrier never reaches the kept wall's offset from that corner, so no rim closes the wall there (shell-opening SO1)`, decaderr.ErrUnsupported)

// ErrOpeningSpan is docs/shell-opening-design.md's SO2: a side opening's rim
// cut lies at or past the far end of the removed walk, where the cavity would
// continue on the next walk's carrier.
var ErrOpeningSpan = fmt.Errorf(`%w: a side opening's rim would run past the far end of the removed walk, where the cavity turns onto the next face; a trimmed-offset construction is not available (shell-opening SO2)`, decaderr.ErrUnsupported)

// opening is one end of a side opening's kept chain, read from the kept walk k
// and its removed neighbour r (docs/shell-opening-design.md §2.1).
type opening struct {
	vU, vV float64
	// kU, kV is k's unit tangent at v in k's own walk sense; nU, nV is s
	// times k's unit left normal there, pointing into the band.
	kU, kV, nU, nV float64
	// fU, fV is r's unit tangent at v pointing into r's span; forward says
	// that direction enters the band. dU, dV is the entering direction:
	// (fU, fV) when forward, its reverse otherwise.
	fU, fV  float64
	dU, dV  float64
	forward bool
}

// readOpening classifies the corner at one end of the kept chain. atEnd says
// v is k's end, where r starts; otherwise v is k's start, where r ends. A
// corner within the dead zone tol of a straight join or a cusp, or a removed
// tangent with no component across the band, is SO1.
func readOpening(k, r survey2d.SideWalk, atEnd bool, s, tol float64) (opening, error) {
	var o opening
	var aU, aV, bU, bV float64 // the loop's arriving and leaving tangents at v
	if atEnd {
		o.vU, o.vV = k.EndU, k.EndV
		aU, aV, bU, bV = k.TanOutU, k.TanOutV, r.TanInU, r.TanInV
	} else {
		o.vU, o.vV = k.StartU, k.StartV
		aU, aV, bU, bV = r.TanOutU, r.TanOutV, k.TanInU, k.TanInV
	}
	ax, ay, la := Normalize(aU, aV)
	bx, by, lb := Normalize(bU, bV)
	if la == 0 || lb == 0 {
		return opening{}, ErrNoDirection
	}
	if math.Abs(ax*by-ay*bx) <= tol {
		return opening{}, ErrOpeningCorner
	}
	if atEnd {
		o.kU, o.kV = ax, ay
		o.fU, o.fV = bx, by
	} else {
		o.kU, o.kV = bx, by
		o.fU, o.fV = -ax, -ay
	}
	o.nU, o.nV = s*(-o.kV), s*o.kU
	along := o.fU*o.nU + o.fV*o.nV
	if along == 0 {
		return opening{}, ErrOpeningCorner
	}
	o.forward = along > 0
	o.dU, o.dV = o.fU, o.fV
	if !o.forward {
		o.dU, o.dV = -o.fU, -o.fV
	}
	return o, nil
}

// OpeningForward reports whether the rim cut at one end of a side opening's
// kept chain runs forward, into the removed walk r's own span, or backward
// along r's carrier beyond the corner v (Table RO's reflex row inward, its
// convex rows outward). The arguments are OpeningJoin's, and so are the
// refusals.
func OpeningForward(k, r survey2d.SideWalk, atEnd bool, s, tol float64) (bool, error) {
	o, err := readOpening(k, r, atEnd, s, tol)
	if err != nil {
		return false, err
	}
	return o.forward, nil
}

// OpeningJoin is the rim cut at one end of a side opening's kept chain
// (docs/shell-opening-design.md §2.4): k is the kept walk and r the removed
// walk beside it; atEnd says the corner v is k's end, where r starts, and
// otherwise k's start, where r ends. The cut q is the first point at which
// r's own carrier, walked from v in the direction that enters the band between
// k's carrier and its offset by s*t, meets that offset carrier. It returns
// Join{VertU, VertV: v, M: q}, with Arc and G1 false; RimSegment writes the
// rim from it.
//
// A straight r whose raw tangent has a float dot product of exactly zero with
// k's takes k's offset foot v + s*t*n̂ along k's held unit left normal; on
// two axis-aligned lines that foot is the pair of levels the record holds. An
// axis-aligned line r through a circular k's centre takes the centre moved by
// k's offset radius along r's axis. Every other corner intersects the two
// carriers and keeps the first root reached in the entering direction: along
// a line by its signed parameter, around a circle by the sweep from v in the
// entering sense. k and r are lines or circular walks.
//
// The refusals are reject-only. A smooth or cusped corner, no root in the
// entering direction, or r's carrier re-crossing k's own carrier before the
// root, is ErrOpeningCorner (SO1). A cut in r's span direction at or past the
// far end of r is ErrOpeningSpan (SO2). A cut backward along r's carrier lies
// off r's span by construction and takes no span test. A walk with no
// direction is ErrNoDirection.
func OpeningJoin(k, r survey2d.SideWalk, atEnd bool, s, t, tol float64) (Join, error) {
	o, err := readOpening(k, r, atEnd, s, tol)
	if err != nil {
		return Join{}, err
	}
	q, ok := exactOpeningCut(k, r, atEnd, o, s, t, tol)
	if !ok {
		q, err = solveOpeningCut(k, r, o, s, t, tol)
		if err != nil {
			return Join{}, err
		}
	}
	if o.forward {
		if err := requireCutInSpan(r, o, q, tol); err != nil {
			return Join{}, err
		}
	}
	return Join{VertU: o.vU, VertV: o.vV, M: q}, nil
}

// exactOpeningCut is §2.4's exact-pair row. It reports false where the corner
// is not one of the pairs the row names.
//
// A straight r whose raw tangent's float dot product with k's at v is exactly
// zero takes k's offset foot, v + s*t along k's held unit left normal: the
// expression the right-angle open chain has always written, so every body it
// built keeps its record bit for bit. Two axis-aligned lines are such a pair,
// and there the foot is the pair of levels the record holds: r's own
// coordinate, which v carries, and k's offset level.
func exactOpeningCut(k, r survey2d.SideWalk, atEnd bool, o opening, s, t, tol float64) (Point, bool) {
	if r.IsCircular() {
		return Point{}, false
	}
	kU, kV, rU, rV := k.TanInU, k.TanInV, r.TanOutU, r.TanOutV
	if atEnd {
		kU, kV, rU, rV = k.TanOutU, k.TanOutV, r.TanInU, r.TanInV
	}
	if kU*rU+kV*rV == 0 {
		return Point{U: o.vU + s*t*(-o.kV), V: o.vV + s*t*o.kU}, true
	}
	if !k.IsCircular() {
		return Point{}, false
	}
	// An axis-aligned line through a circular k's own centre meets k's
	// offset circle at the centre moved by the offset radius along it.
	rr, ok := OffsetRadius(k, s, t, tol)
	if !ok {
		return Point{}, false
	}
	switch {
	case rU == 0 && o.vU == k.CU:
		return Point{U: k.CU, V: k.CV + math.Copysign(rr, o.vV-k.CV)}, true
	case rV == 0 && o.vV == k.CV:
		return Point{U: k.CU + math.Copysign(rr, o.vU-k.CU), V: k.CV}, true
	}
	return Point{}, false
}

// rimCarrier is r's own carrier: the line through v along r's held unit
// tangent, or r's circle.
func rimCarrier(r survey2d.SideWalk, o opening) Curve {
	if r.IsCircular() {
		return Curve{CX: r.CU, CY: r.CV, Radius: r.Radius}
	}
	return Curve{IsLine: true, PX: o.vU, PY: o.vV, DX: o.fU, DY: o.fV}
}

// solveOpeningCut is §2.4's float-solve row.
func solveOpeningCut(k, r survey2d.SideWalk, o opening, s, t, tol float64) (Point, error) {
	rc := rimCarrier(r, o)
	if k.IsCircular() {
		if _, ok := OffsetRadius(k, s, t, tol); !ok {
			return Point{}, ErrDrop
		}
	}
	scale := math.Max(1, math.Abs(o.vU)+math.Abs(o.vV)+t)
	// least is the smallest advance read as leaving v: a length on a line, a
	// sweep in radians on a circle.
	least := tol * scale
	if r.IsCircular() {
		scale = math.Max(scale, 2*r.Radius)
		least = tol
	}
	ahead := func(c [2]float64) float64 { return openingAdvance(r, o, c) }
	best, bestAt := math.Inf(1), [2]float64{}
	for _, c := range intersectAll(offsetCarrier(k, s, t, tol), rc) {
		a := ahead(c)
		if a > least && a < best {
			best, bestAt = a, c
		}
	}
	if math.IsInf(best, 1) {
		return Point{}, ErrOpeningCorner
	}
	// r's carrier leaves the band through k's own carrier before it reaches
	// the offset: the inner body does not close at v on r's carrier.
	for _, c := range intersectAll(offsetCarrier(k, s, 0, tol), rc) {
		if math.Hypot(c[0]-o.vU, c[1]-o.vV) <= tol*scale {
			continue
		}
		if a := ahead(c); a > least && a < best {
			return Point{}, ErrOpeningCorner
		}
	}
	q := Point{U: bestAt[0], V: bestAt[1]}
	// The cut lies on r's carrier. A straight r along a section axis holds
	// one coordinate exactly, v's, so the cut takes that coordinate rather
	// than the solve's rounding of it: the record then states the cut on r's
	// own plane, as the right-angle foot does. OpeningReach charges whatever
	// the other coordinate's rounding leaves.
	if !r.IsCircular() {
		switch {
		case o.fU == 0:
			q.U = o.vU
		case o.fV == 0:
			q.V = o.vV
		}
	}
	return q, nil
}

// openingAdvance is how far along r's carrier, in the entering direction from
// v, the carrier point c lies: a signed length along a line, and a sweep in
// [0, 2π) around a circle.
func openingAdvance(r survey2d.SideWalk, o opening, c [2]float64) float64 {
	if !r.IsCircular() {
		return (c[0]-o.vU)*o.dU + (c[1]-o.vV)*o.dV
	}
	a0 := math.Atan2(o.vV-r.CV, o.vU-r.CU)
	a1 := math.Atan2(c[1]-r.CV, c[0]-r.CU)
	sweep := a1 - a0
	if !enteringCCW(r, o) {
		sweep = -sweep
	}
	for sweep < 0 {
		sweep += 2 * math.Pi
	}
	for sweep >= 2*math.Pi {
		sweep -= 2 * math.Pi
	}
	return sweep
}

// enteringCCW reports whether the entering direction turns counterclockwise
// about a circular r's centre.
func enteringCCW(r survey2d.SideWalk, o opening) bool {
	return (o.vU-r.CU)*o.dV-(o.vV-r.CV)*o.dU > 0
}

// requireCutInSpan is §2.4's span row for a forward cut: it must lie strictly
// inside r's span, a line by length and an arc by sweep.
func requireCutInSpan(r survey2d.SideWalk, o opening, q Point, tol float64) error {
	if !r.IsCircular() {
		reach := (q.U-o.vU)*o.fU + (q.V-o.vV)*o.fV
		length := math.Hypot(r.EndU-r.StartU, r.EndV-r.StartV)
		if reach*(1+tol) >= length {
			return ErrOpeningSpan
		}
		return nil
	}
	sweep := openingAdvance(r, o, [2]float64{q.U, q.V})
	if sweep*(1+tol) >= math.Abs(r.Th1-r.Th0) {
		return ErrOpeningSpan
	}
	return nil
}

// OpeningReach is how far OpeningJoin's held cut j sits from the cut it
// denotes, over every signed offset in amount (docs/shell-opening-design.md
// §2.4): k's offset carrier and r's own carrier are enclosed exactly as
// LoopReach encloses a miter's, every root of the pair is enclosed, and the
// denoted cut is the root the entering direction reaches first. A root on a
// straight r is ahead of the corner v when its advance along the entering
// direction is certainly positive, and the first of two is the one whose
// advance is certainly smaller. Every root of a circular r is ahead, and the
// first of two is read from the orientation of v and the two roots, which on
// one circle is their order around it. A root the enclosures cannot place,
// or two they cannot order, is ErrUnbounded: the reach is never taken to the
// root nearest the held point. An exact pair whose carriers enclose to one
// point equal to the held float reaches zero.
func OpeningReach(k, r survey2d.SideWalk, atEnd bool, s float64, j Join, amount proofbound.RatInterval, tol float64) (float64, error) {
	o, err := readOpening(k, r, atEnd, s, tol)
	if err != nil {
		return 0, ErrUnbounded
	}
	cu, cv, cb := k.StartU, k.StartV, k.StartBound
	if atEnd {
		cu, cv, cb = k.EndU, k.EndV, k.EndBound
	}
	v, okV := capcontour.WalkPointEnclosure(cu, cv, cb)
	kc, okK := capcontour.OffsetCarrierEnclosure(k, amount)
	rc, okR := capcontour.OffsetCarrierEnclosure(r, proofbound.PointInterval(new(big.Rat)))
	if !okV || !okK || !okR {
		return 0, ErrUnbounded
	}
	cands, ok := capcontour.Intersect(kc, rc)
	if !ok {
		return 0, ErrUnbounded
	}
	du, dv := proofbound.PointInterval(new(big.Rat).SetFloat64(o.dU)), proofbound.PointInterval(new(big.Rat).SetFloat64(o.dV))
	advance := func(p capcontour.Point) proofbound.RatInterval {
		return proofbound.IntervalAdd(
			proofbound.IntervalMul(proofbound.IntervalSub(p.U, v.U), du),
			proofbound.IntervalMul(proofbound.IntervalSub(p.V, v.V), dv))
	}
	var ahead []capcontour.Point
	for _, p := range cands {
		if r.IsCircular() {
			ahead = append(ahead, p)
			continue
		}
		a := advance(p)
		switch {
		case a.Lo.Sign() > 0:
			ahead = append(ahead, p)
		case a.Hi.Sign() > 0:
			return 0, ErrUnbounded
		}
	}
	var first capcontour.Point
	switch len(ahead) {
	case 1:
		first = ahead[0]
	case 2:
		p1, p2 := ahead[0], ahead[1]
		var oneFirst bool
		if r.IsCircular() {
			cross := proofbound.IntervalSub(
				proofbound.IntervalMul(proofbound.IntervalSub(p1.U, v.U), proofbound.IntervalSub(p2.V, v.V)),
				proofbound.IntervalMul(proofbound.IntervalSub(p1.V, v.V), proofbound.IntervalSub(p2.U, v.U)))
			switch {
			case cross.Lo.Sign() > 0:
				oneFirst = enteringCCW(r, o)
			case cross.Hi.Sign() < 0:
				oneFirst = !enteringCCW(r, o)
			default:
				return 0, ErrUnbounded
			}
		} else {
			a1, a2 := advance(p1), advance(p2)
			switch {
			case a1.Hi.Cmp(a2.Lo) < 0:
				oneFirst = true
			case a2.Hi.Cmp(a1.Lo) < 0:
			default:
				return 0, ErrUnbounded
			}
		}
		first = p2
		if oneFirst {
			first = p1
		}
	default:
		return 0, ErrUnbounded
	}
	return first.Reach(j.M.U, j.M.V), nil
}

// RimSegment records the rim OpeningJoin's j cut at one end of the kept chain:
// the piece of r's own carrier between the corner v and the cut q, a LineSeg
// or an ArcSeg about r's centre. fromV says the segment runs from v to q;
// otherwise it runs from q to v. k, r, atEnd and s are the arguments j was
// cut with.
func RimSegment(k, r survey2d.SideWalk, atEnd bool, s float64, j Join, fromV bool) (sectionrecord.CurveSegment, error) {
	o, err := readOpening(k, r, atEnd, s, 0)
	if err != nil {
		return nil, err
	}
	v := sectionrecord.Point2{U: j.VertU, V: j.VertV}
	q := sectionrecord.Point2{U: j.M.U, V: j.M.V}
	from, to := v, q
	if !fromV {
		from, to = q, v
	}
	if !r.IsCircular() {
		return sectionrecord.LineSeg{Start: from, End: to, TStart: 0, TEnd: 1}, nil
	}
	// The entering direction runs v to q; the reverse runs q to v.
	ccw := enteringCCW(r, o) == fromV
	return ArcSegment(sectionrecord.Point2{U: r.CU, V: r.CV}, from, to, ccw), nil
}

// intersectAll returns every root of two carriers.
func intersectAll(a, b Curve) [][2]float64 {
	switch {
	case a.IsLine && b.IsLine:
		return lineLine(a, b)
	case a.IsLine && !b.IsLine:
		return lineCircle(a, b.CX, b.CY, b.Radius)
	case !a.IsLine && b.IsLine:
		return lineCircle(b, a.CX, a.CY, a.Radius)
	default:
		return sectionaudit.CircleCircle(a.CX, a.CY, a.Radius, b.CX, b.CY, b.Radius)
	}
}
