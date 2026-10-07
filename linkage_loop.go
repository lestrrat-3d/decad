package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"
	"strings"
	"sync"

	"github.com/lestrrat-3d/decad/internal/motionbound"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// This file is the closed loop of docs/linkage-check-design.md §15: Close and
// Loop, the private sketch scene a driven loop is read from, the canonical
// chain of certified enclosures every dependent joint value comes from, and
// the Schedule a renderer calls per frame. decad computes no 2D answer here:
// every dependent value is sketch.Enclose's certified enclosure, and the
// checks this file runs on what sketch returns only ever refuse.

// LinkageLoop is one closure joint of a linkage and the links it ties together
// (docs/linkage-check-design.md §15.1): a revolute pin joining two links that
// both already have parents, so the tree gains one loop. A drive lists at most
// one joint of a loop, the driver; every other joint of the loop is dependent,
// and its value at a parameter is whatever closes the loop there.
type LinkageLoop struct {
	linkage *Linkage
	a, b    *Link
	common  *Link
	closure RevoluteJoint
	// sideA and sideB are the links from Common's child down to a and to b;
	// either is empty when a or b is Common itself.
	sideA, sideB []*Link
	links        []*Link // every loop link but Common, in Linkage.Links() order
	// normal is the closure's axis n exactly as stated; coord is the
	// coordinate axis it lies along, or -1 for a tilted loop.
	normal motionbound.RatVec
	coord  int
	// slide is the loop's one prismatic link, nil for an all-revolute loop.
	// Its parent is Common; its loop pins are its rail and its next pin.
	slide *Link
	bars  []loopBar
}

// LoopBar is the length a loop's private scene holds one link's two loop
// pins apart at (docs/linkage-check-design.md §15.2): the exact distance, in
// the loop's plane, between the pins. The scene states the bar as the
// interval [Length.Value − Length.Bound, Length.Value], so its claims cover
// the exact length; Length is Exact with a zero Bound when the length is a
// float.
type LoopBar struct {
	Link   *Link
	Length Measurement
}

// loopBar is one bar of a loop: the link whose two loop pins it joins, and
// the exact squared length between them in the loop's plane, with the two
// floats around its root the scene states it as.
type loopBar struct {
	link     *Link
	own      r3.Vec // the link's own joint pin, or Common's first pin
	next     r3.Vec // the next pin along the loop
	down, up float64
}

// Common returns the two closed links' lowest common ancestor: the ground for
// a four-bar.
func (lp *LinkageLoop) Common() *Link { return lp.common }

// Links returns every link on the loop but Common, in Linkage.Links() order.
func (lp *LinkageLoop) Links() []*Link { return slices.Clone(lp.links) }

// Closure returns the closing pin as Close stated it, with nil Limits.
func (lp *LinkageLoop) Closure() RevoluteJoint { return lp.closure }

// Bars returns the length the loop's private scene holds each link's two loop
// pins apart at: Common's bar first, then Links() order. A loop with a
// prismatic joint lists no bar for Common or for the sliding link, whose loop
// pins include the slide's rail.
func (lp *LinkageLoop) Bars() []LoopBar {
	out := make([]LoopBar, len(lp.bars))
	for n, bar := range lp.bars {
		m := Measurement{Value: units.Millimeters(bar.up), Exactness: Exact, Bound: units.Millimeters(0)}
		if bar.down != bar.up {
			m.Exactness, m.Bound = Approximate, units.Millimeters(bar.up-bar.down)
		}
		out[n] = LoopBar{Link: bar.link, Length: m}
	}
	return out
}

// Loops returns every loop of l, in Close order.
func (l *Linkage) Loops() []*LinkageLoop {
	return slices.Clone(l.loops)
}

// Close joins a and b, two distinct links of l that already have parents,
// with a revolute pin at center about axis (world coordinates at the zero
// pose), so the tree gains one loop (docs/linkage-check-design.md §15.1).
// Which joint drives the loop is stated per Drive.
//
// It refuses, in this order: a nil linkage, a nil or parentless a or b,
// a == b, or a link of another linkage (ErrDegenerate); a non-finite center or
// axis component (ErrNotFinite); the zero axis (ErrDegenerate); a loop
// revolute whose axis is not exactly parallel to axis, or a loop prismatic
// whose direction is not exactly perpendicular to it (ErrUnsupported, naming
// the joint); a second prismatic joint on the loop, or one whose parent is not
// the loop's common link (ErrUnsupported); a loop joint already on another
// loop (ErrUnsupported); and two loop pins of one link coincident in the
// loop's plane (ErrDegenerate).
func (l *Linkage) Close(a, b *Link, center, axis r3.Vec) (*LinkageLoop, error) {
	if l == nil {
		return nil, fmt.Errorf(`%w: a nil linkage has no link to close`, ErrDegenerate)
	}
	if a == nil || b == nil || a.parent == nil || b.parent == nil {
		return nil, fmt.Errorf(`%w: a closure joins two links that already have parents`, ErrDegenerate)
	}
	if a == b {
		return nil, fmt.Errorf(`%w: a closure joins two distinct links`, ErrDegenerate)
	}
	if a.linkage != l || b.linkage != l {
		return nil, fmt.Errorf(`%w: a closure joins two links of this linkage`, ErrDegenerate)
	}
	if !proofbound.FiniteVec(center) || !proofbound.FiniteVec(axis) {
		return nil, fmt.Errorf(`%w: a closure's center and axis must be finite, got %v and %v`, ErrNotFinite, center, axis)
	}
	if zeroVec(axis) {
		return nil, fmt.Errorf(`%w: a zero closure axis names no direction`, ErrDegenerate)
	}
	lp := &LinkageLoop{linkage: l, a: a, b: b, closure: RevoluteJoint{Center: center, Axis: axis}, normal: ratVecExact(axis), coord: -1}
	if idx, _, ok := coordinateAxis(axis); ok {
		lp.coord = idx
	}
	lp.common = commonAncestor(a, b)
	lp.sideA, lp.sideB = linksBelow(lp.common, a), linksBelow(lp.common, b)
	on := make(map[*Link]struct{})
	for _, k := range append(slices.Clone(lp.sideA), lp.sideB...) {
		on[k] = struct{}{}
	}
	for _, k := range l.links {
		if _, ok := on[k]; ok {
			lp.links = append(lp.links, k)
		}
	}
	for _, k := range lp.links {
		switch j := k.joint.(type) {
		case RevoluteJoint:
			if c := ratCross(ratVecExact(j.Axis), lp.normal); !ratZero(c) {
				return nil, fmt.Errorf(`%w: link %d's revolute axis %v is not exactly parallel to the closure axis %v`, ErrUnsupported, k.index, j.Axis, axis)
			}
		case PrismaticJoint:
			if ratDot(ratVecExact(j.Dir), lp.normal).Sign() != 0 {
				return nil, fmt.Errorf(`%w: link %d's prismatic direction %v is not exactly perpendicular to the closure axis %v`, ErrUnsupported, k.index, j.Dir, axis)
			}
		}
	}
	for _, k := range lp.links {
		if _, ok := k.joint.(PrismaticJoint); !ok {
			continue
		}
		if lp.slide != nil {
			return nil, fmt.Errorf(`%w: links %d and %d both slide on the loop, and a loop holds one prismatic joint`, ErrUnsupported, lp.slide.index, k.index)
		}
		if k.parent != lp.common {
			return nil, fmt.Errorf(`%w: link %d slides on a link that is not the loop's common link`, ErrUnsupported, k.index)
		}
		lp.slide = k
	}
	for _, other := range l.loops {
		for _, k := range lp.links {
			if slices.Contains(other.links, k) {
				return nil, fmt.Errorf(`%w: link %d's joint is already on another loop`, ErrUnsupported, k.index)
			}
		}
	}
	bars, err := lp.readBars()
	if err != nil {
		return nil, err
	}
	lp.bars = bars
	l.loops = append(l.loops, lp)
	return lp, nil
}

// coordinateAxis reads an axis exactly parallel to a coordinate axis: its
// index and sense; ok is false for any other direction.
func coordinateAxis(v r3.Vec) (int, int, bool) {
	c := [3]float64{v.X, v.Y, v.Z}
	idx := -1
	for i, x := range c {
		if x == 0 {
			continue
		}
		if idx >= 0 {
			return 0, 0, false
		}
		idx = i
	}
	if idx < 0 {
		return 0, 0, false
	}
	if c[idx] < 0 {
		return idx, -1, true
	}
	return idx, 1, true
}

// unitAxis is sense·e_axis.
func unitAxis(axis, sense int) r3.Vec {
	c := [3]float64{}
	c[axis] = float64(sense)
	return r3.NewVec(c[0], c[1], c[2])
}

// ratVecExact reads a finite vector exactly; every caller has checked it is
// finite.
func ratVecExact(v r3.Vec) motionbound.RatVec {
	out, _ := motionbound.RatVecOf(v)
	return out
}

// commonAncestor is the lowest common ancestor of a and b in their tree.
func commonAncestor(a, b *Link) *Link {
	above := make(map[*Link]struct{})
	for k := a; k != nil; k = k.parent {
		above[k] = struct{}{}
	}
	for k := b; k != nil; k = k.parent {
		if _, ok := above[k]; ok {
			return k
		}
	}
	return nil
}

// linksBelow is the path from common's child down to k, in that order; empty
// when k is common.
func linksBelow(common, k *Link) []*Link {
	var out []*Link
	for at := k; at != common; at = at.parent {
		out = append([]*Link{at}, out...)
	}
	return out
}

// pin is a link's own joint pin: its revolute joint's center.
func (k *Link) pin() r3.Vec {
	if j, ok := k.joint.(RevoluteJoint); ok {
		return j.Center
	}
	return r3.Vec{}
}

// nextPin is the pin after link k along its side of the loop: the joint pin of
// its loop child, or the closure for the side's last link.
func (lp *LinkageLoop) nextPin(side []*Link, n int) r3.Vec {
	if n+1 < len(side) {
		return side[n+1].pin()
	}
	return lp.closure.Center
}

// commonPins are Common's two loop pins: the first link's joint pin on each
// side, or the closure on a side with no link.
func (lp *LinkageLoop) commonPins() (r3.Vec, r3.Vec) {
	first := func(side []*Link) r3.Vec {
		if len(side) == 0 {
			return lp.closure.Center
		}
		return side[0].pin()
	}
	return first(lp.sideA), first(lp.sideB)
}

// planeSq is the exact squared distance between two points in the loop's
// plane: their difference with the component along the closure axis n
// dropped, |d|² − (d·n)²/|n|².
func (lp *LinkageLoop) planeSq(p, q r3.Vec) *big.Rat {
	pd, qd := ratVecExact(p), ratVecExact(q)
	var d motionbound.RatVec
	for i := range 3 {
		d[i] = new(big.Rat).Sub(pd[i], qd[i])
	}
	along := ratDot(d, lp.normal)
	along.Mul(along, along)
	along.Quo(along, ratDot(lp.normal, lp.normal))
	return along.Sub(ratDot(d, d), along)
}

// linkNext is loop link k's next pin along its side of the loop.
func (lp *LinkageLoop) linkNext(k *Link) r3.Vec {
	side := lp.sideA
	if !slices.Contains(side, k) {
		side = lp.sideB
	}
	return lp.nextPin(side, slices.Index(side, k))
}

// readBars reads every bar of the loop: Common's, then each revolute loop
// link's in Links() order. A loop with a slide has no bar for Common, whose
// second loop pin is the slide's rail, nor for the slide, whose own is.
// Two pins of one link coincident in the plane are refused.
func (lp *LinkageLoop) readBars() ([]loopBar, error) {
	bar := func(link *Link, own, next r3.Vec) (loopBar, error) {
		sq := lp.planeSq(own, next)
		if sq.Sign() == 0 {
			return loopBar{}, fmt.Errorf(`%w: two loop pins of one link coincide in the loop's plane at %v`, ErrDegenerate, own)
		}
		down, up := proofbound.RatSqrtDown(sq), proofbound.RatSqrtUp(sq)
		if proofbound.IsNonFinite(up) || !(down > 0) {
			return loopBar{}, fmt.Errorf(`%w: a loop bar's length is not representable`, ErrNotFinite)
		}
		return loopBar{link: link, own: own, next: next, down: down, up: up}, nil
	}
	var out []loopBar
	if lp.slide == nil {
		pa, pb := lp.commonPins()
		common, err := bar(lp.common, pa, pb)
		if err != nil {
			return nil, err
		}
		out = append(out, common)
	}
	for _, k := range lp.links {
		if k == lp.slide {
			continue
		}
		b, err := bar(k, k.pin(), lp.linkNext(k))
		if err != nil {
			return nil, err
		}
		out = append(out, b)
	}
	return out, nil
}

// loopDrive is one loop moved by one drive: its driver and dependents, the
// private scenes, and the chains of certified enclosures every dependent value
// is read from (docs/linkage-check-design.md §15.2-§15.3, §15.8). The drive is
// cut into sub-segments, each a stretch of one segment on which the driver's
// value has one sign, with a chain of its own from its near end. The asks are
// serialised behind mu, since sketch.Enclose must not run concurrently on one
// sketch, and cached by their place in the canonical chains.
type loopDrive struct {
	loop     *LinkageLoop
	driver   int   // the driver's position in Linkage.Links()
	deps     []int // every dependent's position in Linkage.Links(), Loop.Links() order
	driverJt linkJoint
	// axisSense is a revolute driver's axis sense against the closure axis;
	// it picks each side's mirror. A slide driver's is unused.
	axisSense int
	subs      []loopSub
	// held reports that every sub-segment holds the driver: the loop's links
	// then stand at one placement for the whole drive.
	held bool

	mu sync.Mutex
	// scenes holds one scene per side of the plane: [0] where the driver's
	// scene value is q, [1] where it is −q; nil for a side no sub-segment
	// needs.
	scenes [2]*loopScene
	asks   map[string]*loopAsk
	reach  []*big.Rat // per dependent: a proven bound on |value| over the certified drive
	hulls  []proofbound.RatInterval
	// spans holds each verification interval's dependent readings, keyed by
	// its two ends, per piece the interval is cut into and per dependent,
	// from intervalGate until travel reads them.
	spans map[string][][]loopSpan
}

// loopSub is one sub-segment of a driven loop's drive
// (docs/linkage-check-design.md §15.8): the stretch [lo, hi] of the fraction
// on which the driver's value has one sign, its near end — where |q| is
// least, and the chain starts — and the side of the plane it is read on.
type loopSub struct {
	idx      int
	lo, hi   *big.Rat
	near     *big.Rat
	nearZero bool // the driver is exactly 0 at the near end
	held     bool // the driver holds one value over the sub-segment
	side     int  // 0 when q ≥ 0 over it, 1 when q ≤ 0
	// straddle marks the stretch between two rational cuts that holds an
	// irrational zero crossing. It has no chain: its two neighbours, the
	// sub-segments just before and after it, end at those cuts, and its
	// values are their branches' from the zero pose to each cut.
	straddle bool
}

// loopScene is the private sketch scene of one loop under one drive, on one
// side of the plane (docs/linkage-check-design.md §15.2).
type loopScene struct {
	plane      loopPlane
	sk         *sketch.Sketch
	driver     sketch.Dimension
	driven     []sketch.Dimension // per dependent: an angle, or a slide's horizontal distance
	angular    []bool             // per dependent: its reading is an angle, read modulo a turn
	signs      []int              // per dependent: the sense of its axis against the scene's normal, or of its slide against u
	opts       []sketch.EncloseOption
	pins       []scenePin
	e0         *loopAsk
	e0Readings []sketch.Interval // per dependent: its reading at the zero pose
}

// scenePin is one loop pin's sketch point, its world position, and the
// enclosure of its exact plane position in the document: one exact value per
// coordinate on a coordinate-axis loop.
type scenePin struct {
	p    *sketch.Point
	at   r3.Vec
	u, v proofbound.RatInterval
}

// loopAsk is one certified enclosure of a chain, on its scene, or the refusal
// that took its place. turns counts, per dependent, the whole turns its
// readings are shifted by to continue its predecessor's.
type loopAsk struct {
	scene *loopScene
	enc   *sketch.Enclosure
	turns []int64
	err   error
}

// loopSpan is one dependent's readings over a verification interval's piece:
// its value at the piece's two ends and the hull over it, as rational radians
// or millimetres.
type loopSpan struct {
	a, b, h proofbound.RatInterval
}

// unbuildableError marks a pose whose loop could not be enclosed
// (docs/linkage-check-design.md §15.6): the pose is not evaluated and the
// intervals that end at it are undecided.
type unbuildableError struct{ cause error }

func (e *unbuildableError) Error() string { return e.cause.Error() }
func (e *unbuildableError) Unwrap() error { return e.cause }
func (e *unbuildableError) unbuildable()  {}

// resolveLoops applies the loop rows of docs/linkage-check-design.md §15.6 to
// a resolved drive, marks every dependent joint, and cuts each driven loop's
// drive into sub-segments. A loop the drive does not list, or whose listed
// joint holds 0, stands at the zero pose and is left as the tree's held
// joints. The scenes are built later, by prepare.
func (l *Linkage) resolveLoops(spec *linkageSpec) error {
	for _, lp := range l.loops {
		var listed []int
		for _, k := range lp.links {
			if spec.joints[k.index].listed {
				listed = append(listed, k.index)
			}
		}
		if len(listed) > 1 {
			return fmt.Errorf(`%w: a drive states links %d and %d of one loop, but a loop's second value follows from its first`, ErrDegenerate, listed[0], listed[1])
		}
		if len(listed) == 0 {
			continue
		}
		k := listed[0]
		jt := spec.joints[k]
		if jt.link.parent != lp.common {
			return fmt.Errorf(`%w: a drive moves link %d of a loop, whose parent is not the loop's common link`, ErrUnsupported, k)
		}
		if heldAtZeroJoint(jt) {
			continue
		}
		ld := &loopDrive{loop: lp, driver: k, driverJt: jt, asks: make(map[string]*loopAsk), spans: make(map[string][][]loopSpan)}
		if jt.revolute {
			ld.axisSense = ratDot(ratVecExact(jt.axis), lp.normal).Sign()
		}
		subs, err := driverSubs(jt)
		if err != nil {
			return err
		}
		ld.subs = subs
		ld.held = !jt.moves()
		for _, link := range lp.links {
			if link.index != k {
				ld.deps = append(ld.deps, link.index)
			}
		}
		for _, d := range ld.deps {
			spec.joints[d].dep = ld
		}
		spec.loops = append(spec.loops, ld)
	}
	return nil
}

// driverSubs cuts a loop driver's schedule into sub-segments
// (docs/linkage-check-design.md §15.8): each segment once, or twice at the
// fraction where its driver value crosses 0. That fraction is exact when both
// of the segment's waypoints are whole turns, or both radians or lengths.
// Between waypoints stated in mixed terms it depends on π, and the segment is
// cut three times instead: up to a rational cut below the crossing, the
// straddle holding it, and on from a rational cut above it (crossingCuts).
// A sub-segment holding the driver at 0 is read on any side a moving one
// uses.
func driverSubs(jt linkJoint) ([]loopSub, error) {
	n := len(jt.points) - 1
	signs := make([]int, len(jt.values))
	for w, v := range jt.values {
		c, ok := paramCompare(v, units.New(0, v.Unit()))
		if !ok {
			return nil, fmt.Errorf(`%w: the sign of link %d's waypoint %s cannot be decided`, ErrUnsupported, jt.link.index, v)
		}
		signs[w] = c
	}
	side := func(sign int) int {
		if sign < 0 {
			return 1
		}
		return 0
	}
	var subs []loopSub
	add := func(sub loopSub) {
		sub.idx = len(subs)
		subs = append(subs, sub)
	}
	for j := range n {
		a, b := big.NewRat(int64(j), int64(n)), big.NewRat(int64(j+1), int64(n))
		pa, pb := jt.points[j], jt.points[j+1]
		sa, sb := signs[j], signs[j+1]
		switch {
		case pa.Turn.Cmp(pb.Turn) == 0 && pa.Base.Cmp(pb.Base) == 0:
			add(loopSub{lo: a, hi: b, near: a, nearZero: sa == 0, held: true, side: side(sa)})
		case sa*sb < 0:
			var num, den *big.Rat
			switch {
			case pa.Turn.Sign() == 0 && pb.Turn.Sign() == 0:
				num, den = pa.Base, new(big.Rat).Sub(pa.Base, pb.Base)
			case pa.Base.Sign() == 0 && pb.Base.Sign() == 0:
				num, den = pa.Turn, new(big.Rat).Sub(pa.Turn, pb.Turn)
			default:
				lo, hi, ok := crossingCuts(pa, pb, a, b, sa)
				if !ok {
					return nil, fmt.Errorf(`%w: link %d's driver crosses 0 between waypoints %s and %s stated in mixed terms, where the crossing cannot be bracketed`,
						ErrUnsupported, jt.link.index, jt.values[j], jt.values[j+1])
				}
				add(loopSub{lo: a, hi: lo, near: lo, side: side(sa)})
				add(loopSub{lo: new(big.Rat).Set(lo), hi: hi, near: new(big.Rat).Set(lo), side: side(sa), straddle: true})
				add(loopSub{lo: new(big.Rat).Set(hi), hi: b, near: new(big.Rat).Set(hi), side: side(sb)})
				continue
			}
			t := new(big.Rat).Quo(num, den)
			s0 := new(big.Rat).Sub(b, a)
			s0.Mul(s0, t).Add(s0, a)
			add(loopSub{lo: a, hi: s0, near: s0, nearZero: true, side: side(sa)})
			add(loopSub{lo: new(big.Rat).Set(s0), hi: b, near: new(big.Rat).Set(s0), nearZero: true, side: side(sb)})
		default:
			sign := sa + sb
			sub := loopSub{lo: a, hi: b, near: a, side: side(sign)}
			// The near end has the smaller |q|: the smaller value of a
			// positive stretch, the larger of a negative one.
			ends, ok := paramCompare(jt.values[j], jt.values[j+1])
			if !ok {
				return nil, fmt.Errorf(`%w: link %d's waypoints %s and %s cannot be ordered`, ErrUnsupported, jt.link.index, jt.values[j], jt.values[j+1])
			}
			if ends*sign > 0 {
				sub.near = b
			}
			sub.nearZero = (sub.near == a && sa == 0) || (sub.near == b && sb == 0)
			add(sub)
		}
	}
	// A stretch that holds 0 needs a zero pose, which every side's E0 is;
	// it takes a side some moving stretch uses.
	used := -1
	for _, sub := range subs {
		if !sub.held || !sub.nearZero {
			used = sub.side
			break
		}
	}
	for n := range subs {
		if subs[n].held && subs[n].nearZero && used >= 0 {
			subs[n].side = used
		}
	}
	return subs, nil
}

// crossingCuts brackets the irrational fraction where a driver crosses 0
// between waypoints pa at a and pb at b stated in mixed terms: two rationals
// lo < hi inside (a, b), the driver's value at lo proven to have pa's sign sa
// and at hi pb's, for every π in its enclosure. The crossing's local
// fraction q_a/(q_a − q_b) is read at both ends of π's enclosure, the two
// readings widened outward by their gap, and the signs then checked exactly;
// ok is false when a check fails.
func crossingCuts(pa, pb motionbound.MotionParam, a, b *big.Rat, sa int) (*big.Rat, *big.Rat, bool) {
	twoPi := proofbound.TwoPiInterval()
	var ts []*big.Rat
	for _, tp := range []*big.Rat{twoPi.Lo, twoPi.Hi} {
		qa := new(big.Rat).Mul(pa.Turn, tp)
		qa.Add(qa, pa.Base)
		qb := new(big.Rat).Mul(pb.Turn, tp)
		qb.Add(qb, pb.Base)
		den := new(big.Rat).Sub(qa, qb)
		if den.Sign() == 0 {
			return nil, nil, false
		}
		ts = append(ts, new(big.Rat).Quo(qa, den))
	}
	tlo, thi := ts[0], ts[1]
	if tlo.Cmp(thi) > 0 {
		tlo, thi = thi, tlo
	}
	gap := new(big.Rat).Sub(thi, tlo)
	tlo = new(big.Rat).Sub(tlo, gap)
	thi = new(big.Rat).Add(thi, gap)
	at := func(t *big.Rat) *big.Rat {
		out := new(big.Rat).Sub(b, a)
		return out.Add(out.Mul(out, t), a)
	}
	lo, hi := at(tlo), at(thi)
	if lo.Cmp(a) <= 0 || hi.Cmp(b) >= 0 || lo.Cmp(hi) >= 0 {
		return nil, nil, false
	}
	signed := func(t *big.Rat, want int) bool {
		q := pa.Lerp(pb, t)
		if want < 0 {
			return paramUpper(q).Sign() < 0
		}
		return paramLower(q).Sign() > 0
	}
	if !signed(tlo, sa) || !signed(thi, -sa) {
		return nil, nil, false
	}
	return lo, hi, true
}

// increasing reports that a sub-segment's chain runs toward larger fractions.
func (sub loopSub) increasing() bool { return sub.near.Cmp(sub.lo) == 0 }

// holds reports whether the sub-segment holds the exact fraction s.
func (sub loopSub) holds(s *big.Rat) bool { return sub.lo.Cmp(s) <= 0 && s.Cmp(sub.hi) <= 0 }

// subAt is the sub-segment a pose at s is read on: the one whose near end s is,
// when there is one, and otherwise the first that holds s. Every sub-segment
// holding a parameter reads one exact configuration there: on one side of 0
// the solution is one continuous function of the driver's value, certified
// from the zero pose, and at 0 it is the zero pose itself.
func (ld *loopDrive) subAt(s *big.Rat) loopSub {
	first := -1
	for n, sub := range ld.subs {
		if !sub.holds(s) {
			continue
		}
		if sub.near.Cmp(s) == 0 {
			return sub
		}
		if first < 0 {
			first = n
		}
	}
	if first < 0 {
		// s outside [0, 1]; the callers refuse it before asking.
		return ld.subs[0]
	}
	return ld.subs[first]
}

// pieces cuts [a, b] at every sub-segment boundary strictly inside it: each
// piece with the sub-segment it lies in, in order.
func (ld *loopDrive) pieces(a, b *big.Rat) []loopPiece {
	var out []loopPiece
	for _, sub := range ld.subs {
		lo, hi := a, b
		if sub.lo.Cmp(lo) > 0 {
			lo = sub.lo
		}
		if sub.hi.Cmp(hi) < 0 {
			hi = sub.hi
		}
		if lo.Cmp(hi) < 0 {
			out = append(out, loopPiece{sub: sub, lo: lo, hi: hi})
		}
	}
	return out
}

// loopPiece is a stretch of a verification interval inside one sub-segment.
type loopPiece struct {
	sub    loopSub
	lo, hi *big.Rat
}

// sides reports which scene sides the drive's sub-segments read.
func (ld *loopDrive) sides() [2]bool {
	var out [2]bool
	for _, sub := range ld.subs {
		out[sub.side] = true
	}
	return out
}

// sceneSide is the frame flip side reads on: for a revolute driver the side
// whose normal makes the driver's scene value |q|, the mirror image when its
// axis turns against that; for a slide the half-turned side for q ≤ 0.
func (ld *loopDrive) sceneSide(side int, slide bool) (mirror, halfTurn bool) {
	if slide {
		return false, side == 1
	}
	sign := 1
	if side == 1 {
		sign = -1
	}
	return sign*ld.axisSense < 0, false
}

// loopPlane is the loop's plane on one side (docs/linkage-check-design.md
// §15.2): the exact orthonormal axes u* = u/|u| and v* = v/|v|, with u and v
// exact rational directions and u* × v* the side's normal, and the float
// frame r3 builds along them, which seeds the scene's points.
type loopPlane struct {
	frame      r3.Frame
	u, v       motionbound.RatVec
	uLen, vLen proofbound.RatInterval // enclosures of |u| and |v|
}

// sceneFrame is the loop's plane on one side. u is the slide's own direction
// on a loop with a slide, e_{i+1} on an all-revolute loop about ±e_i, and
// otherwise n × e_m for the coordinate axis e_m along which n has its
// smallest component; v = n × u, with n the closure axis, negated by mirror.
// halfTurn negates both under the same normal. On a coordinate-axis loop
// whose slide, if any, runs along a coordinate axis, the float frame's U and
// V are unit coordinate axes, so a pin's float plane position is two of its
// own coordinates.
func (lp *LinkageLoop) sceneFrame(mirror, halfTurn bool) (loopPlane, error) {
	n := lp.normal
	if mirror {
		n = ratNeg(n)
	}
	var slideDir r3.Vec
	slideAxis := -1
	if lp.slide != nil {
		j, _ := lp.slide.joint.(PrismaticJoint)
		slideDir = j.Dir
		if idx, _, ok := coordinateAxis(j.Dir); ok {
			slideAxis = idx
		}
	}
	var uf, vf r3.Vec
	switch {
	case lp.coord >= 0 && lp.slide == nil:
		sense := ratVecSign(lp.normal, lp.coord)
		if mirror {
			sense = -sense
		}
		axes := [3]r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
		uf, vf = axes[(lp.coord+1)%3], axes[(lp.coord+2)%3]
		if sense < 0 {
			vf = vf.Scale(-1)
		}
	case lp.coord >= 0 && slideAxis >= 0:
		sense := ratVecSign(lp.normal, lp.coord)
		if mirror {
			sense = -sense
		}
		_, dir, _ := coordinateAxis(slideDir)
		uf = unitAxis(slideAxis, dir)
		vf = unitAxis(lp.coord, sense).Cross(uf)
	case lp.slide != nil:
		uf = slideDir
		vf = ratVecFloat(n).Cross(uf)
	default:
		m := 0
		for i := 1; i < 3; i++ {
			if new(big.Rat).Abs(lp.normal[i]).Cmp(new(big.Rat).Abs(lp.normal[m])) < 0 {
				m = i
			}
		}
		e := [3]float64{}
		e[m] = 1
		uf = ratVecFloat(lp.normal).Cross(r3.NewVec(e[0], e[1], e[2]))
		vf = ratVecFloat(n).Cross(uf)
	}
	if halfTurn {
		uf, vf = uf.Scale(-1), vf.Scale(-1)
	}
	frame, err := r3.NewFrame(r3.Vec{}, uf, vf)
	if err != nil {
		return loopPlane{}, err
	}
	// uf holds u exactly: a unit axis, the slide's direction, or n × e_m,
	// whose components are 0 and ± n's own. v is formed exactly from it.
	u := ratVecExact(uf)
	v := ratCross(n, u)
	return loopPlane{frame: frame, u: u, v: v, uLen: ratNorm(u), vLen: ratNorm(v)}, nil
}

// coords encloses p's exact coordinates (p·u*, p·v*) in the plane as two
// rational intervals; each is one exact value where the plane's axis has a
// float length and the quotient is exact, as on a coordinate-axis loop.
func (pl loopPlane) coords(p r3.Vec) (proofbound.RatInterval, proofbound.RatInterval) {
	pr := ratVecExact(p)
	return ratQuoInterval(ratDot(pr, pl.u), pl.uLen), ratQuoInterval(ratDot(pr, pl.v), pl.vLen)
}

// ratQuoInterval encloses num/d for d in the positive interval den.
func ratQuoInterval(num *big.Rat, den proofbound.RatInterval) proofbound.RatInterval {
	lo, hi := new(big.Rat).Quo(num, den.Hi), new(big.Rat).Quo(num, den.Lo)
	if num.Sign() < 0 {
		lo, hi = hi, lo
	}
	return proofbound.IntervalOwned(lo, hi)
}

// ratNorm encloses |w| between the two floats around its exact root.
func ratNorm(w motionbound.RatVec) proofbound.RatInterval {
	sq := ratDot(w, w)
	return proofbound.IntervalOwned(proofarith.FloatRat(proofbound.RatSqrtDown(sq)), proofarith.FloatRat(proofbound.RatSqrtUp(sq)))
}

// ratNeg is −w.
func ratNeg(w motionbound.RatVec) motionbound.RatVec {
	return motionbound.RatVec{new(big.Rat).Neg(w[0]), new(big.Rat).Neg(w[1]), new(big.Rat).Neg(w[2])}
}

// ratVecSign is the sign of w's component i.
func ratVecSign(w motionbound.RatVec, i int) int {
	if w[i].Sign() < 0 {
		return -1
	}
	return 1
}

// ratVecFloat is the float vector nearest w; every caller's w is a float
// vector read exactly, or its negation, so it is w itself.
func ratVecFloat(w motionbound.RatVec) r3.Vec {
	x, _ := w[0].Float64()
	y, _ := w[1].Float64()
	z, _ := w[2].Float64()
	return r3.NewVec(x, y, z)
}

// floatBox is the outward-rounded float interval around iv.
func floatBox(iv proofbound.RatInterval) sketch.Interval {
	return sketch.Interval{Lo: proofbound.RatFloatDown(iv.Lo), Hi: proofbound.RatFloatUp(iv.Hi)}
}

// buildScene builds the loop's private scene for the drive on one side
// (docs/linkage-check-design.md §15.2): one point per loop pin at its exact
// plane position, Common's pins fixed; a line and a distance per revolute
// loop link between its two pins, the distance's target the interval around
// the exact length; for a slide, a fixed rail along u through its next pin's
// zero-pose position, that pin held on it; the driver's angle from a fixed
// reference line along its zero-pose bar, or a slide driver's horizontal
// distance from its zero-pose point; and a driven angle per dependent from
// its parent's line to its own, or a dependent slide's horizontal distance.
func (ld *loopDrive) buildScene(ctx context.Context, spec *linkageSpec, side int) (*loopScene, error) {
	lp := ld.loop
	mirror, halfTurn := ld.sceneSide(side, spec.joints[ld.driver].link == lp.slide)
	plane, err := lp.sceneFrame(mirror, halfTurn)
	if err != nil {
		return nil, fmt.Errorf(`%w: a loop's plane frame: %w`, ErrNotFinite, err)
	}
	w := sketch.NewWorld()
	sk, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, fmt.Errorf(`%w: a loop's scene: %w`, ErrUnsupported, err)
	}
	sc := &loopScene{plane: plane, sk: sk}
	// fix grounds p at the exact plane position x × y: a fixed box around it
	// where either coordinate is not one float, as on a tilted loop.
	fix := func(p *sketch.Point, x, y proofbound.RatInterval) {
		sk.Fix(p)
		bx, by := floatBox(x), floatBox(y)
		if bx.Lo != bx.Hi || by.Lo != by.Hi {
			sc.opts = append(sc.opts, sketch.WithFixedBox(p, bx, by))
		}
	}
	points := make(map[r3.Vec]int) // each pin's index in sc.pins
	pinAt := func(at r3.Vec) scenePin {
		if n, ok := points[at]; ok {
			return sc.pins[n]
		}
		local := plane.frame.ToLocal(at)
		x, y := plane.coords(at)
		pin := scenePin{p: sk.CreatePoint(local.X, local.Y), at: at, u: x, v: y}
		points[at] = len(sc.pins)
		sc.pins = append(sc.pins, pin)
		return pin
	}
	point := func(at r3.Vec) *sketch.Point { return pinAt(at).p }
	fixPin := func(at r3.Vec) {
		pin := pinAt(at)
		fix(pin.p, pin.u, pin.v)
	}
	fixed := func(at r3.Vec, du float64) *sketch.Point {
		local := plane.frame.ToLocal(at)
		x, y := plane.coords(at)
		if du != 0 {
			local.X += du
			shift := proofarith.FloatRat(du)
			x = proofbound.IntervalOwned(new(big.Rat).Add(x.Lo, shift), new(big.Rat).Add(x.Hi, shift))
		}
		p := sk.CreatePoint(local.X, local.Y)
		fix(p, x, y)
		return p
	}
	lines := make(map[*Link]*sketch.Line)
	var cons []sketch.Constraint
	var commonLine *sketch.Line
	var railStart, slider *sketch.Point
	if lp.slide == nil {
		pa, pb := lp.commonPins()
		commonLine = sk.CreateLine(point(pa), point(pb))
		fixPin(pa)
		fixPin(pb)
	} else {
		// The slide's rail is the fixed line through its next pin's zero-pose
		// position along u; that pin rides it. The rail is the line Common's
		// other pin, and the slide's children, measure their angles from.
		pin := lp.linkNext(lp.slide)
		slider = point(pin)
		railStart = fixed(pin, 0)
		commonLine = sk.CreateLine(railStart, fixed(pin, 1))
		lines[lp.slide] = commonLine
		cons = append(cons, sketch.NewPointOnLine(slider, commonLine))
		pa, pb := lp.commonPins()
		other := pa
		if len(lp.sideA) > 0 && lp.sideA[0] == lp.slide {
			other = pb
		}
		fixPin(other)
	}
	for _, bar := range lp.bars {
		if bar.link == lp.common {
			continue
		}
		own, next := point(bar.own), point(bar.next)
		lines[bar.link] = sk.CreateLine(own, next)
		dist := sketch.NewDistance(own, next, bar.up)
		cons = append(cons, dist)
		sc.opts = append(sc.opts, sketch.WithTargetRange(dist, bar.down, bar.up))
	}
	parentLine := func(k *Link) *sketch.Line {
		if k.parent == lp.common {
			return commonLine
		}
		return lines[k.parent]
	}
	n := lp.normal
	if mirror {
		n = ratNeg(n)
	}
	senseOf := func(k int) int {
		if spec.joints[k].revolute {
			return ratDot(ratVecExact(spec.joints[k].axis), n).Sign()
		}
		return ratDot(ratVecExact(spec.joints[k].axis), plane.u).Sign()
	}
	driverLink := spec.joints[ld.driver].link
	if driverLink == lp.slide {
		sc.driver = sketch.NewHorizontalDistance(railStart, slider, 0)
	} else {
		next := lp.linkNext(driverLink)
		refLine := sk.CreateLine(point(driverLink.pin()), fixed(next, 0))
		sc.driver = sketch.NewAngle(refLine, lines[driverLink], 0)
	}
	cons = append(cons, sc.driver)
	for _, d := range ld.deps {
		link := spec.joints[d].link
		var dim sketch.Dimension
		if link == lp.slide {
			h := sketch.NewHorizontalDistance(railStart, slider, 0)
			h.SetDriven(true)
			dim = h
		} else {
			a := sketch.NewAngle(parentLine(link), lines[link], 0)
			a.SetDriven(true)
			dim = a
		}
		sc.driven = append(sc.driven, dim)
		sc.angular = append(sc.angular, link != lp.slide)
		sc.signs = append(sc.signs, senseOf(d))
		cons = append(cons, dim)
	}
	sk.AddConstraint(cons...)
	if _, err := sk.Solve(ctx); err != nil {
		return nil, fmt.Errorf(`%w: the loop's scene does not solve at the zero pose: %w`, ErrUnsupported, err)
	}
	return sc, nil
}

// prepare builds each driven loop's scene on every side its drive reads and
// asks each scene's zero pose, E0 (docs/linkage-check-design.md §15.2). E0
// refusing is ErrUnsupported wrapping sketch's error. The document's pins are
// exact solutions of the scene's equations at the driving value 0 for every
// target its intervals hold, so a pin outside E0's box disproves the
// enclosure and the loop is refused; a pin inside it admits nothing.
func (s *linkageSpec) prepare(ctx context.Context) error {
	for _, ld := range s.loops {
		for side, used := range ld.sides() {
			if !used || ld.scenes[side] != nil {
				continue
			}
			sc, err := ld.buildScene(ctx, s, side)
			if err != nil {
				return err
			}
			if err := sc.askZero(ctx); err != nil {
				return err
			}
			ld.scenes[side] = sc
		}
	}
	return nil
}

// askZero asks E0 and runs the zero-pose falsifier on it.
func (sc *loopScene) askZero(ctx context.Context) error {
	enc, err := sc.sk.Enclose(ctx, sc.driver, 0, 0, sc.opts...)
	if err != nil {
		if cerr := ctx.Err(); cerr != nil {
			return cerr
		}
		if loopInvariant(err) {
			return err
		}
		return fmt.Errorf(`%w: the loop cannot be enclosed at its zero pose: %w`, ErrUnsupported, err)
	}
	for _, pin := range sc.pins {
		x, y, ok := enc.PointBox(pin.p)
		if !ok || !intervalHolds(x, pin.u) || !intervalHolds(y, pin.v) {
			return fmt.Errorf(`%w: the loop's zero-pose enclosure does not hold the document's pin at %v, in the loop's plane (%v, %v), so it does not describe the document's mechanism`,
				ErrUnsupported, pin.at, pin.u.Lo.FloatString(6), pin.v.Lo.FloatString(6))
		}
	}
	sc.e0 = &loopAsk{scene: sc, enc: enc, turns: make([]int64, len(sc.driven))}
	for _, d := range sc.driven {
		r, ok := enc.Driven(d)
		if !ok {
			return fmt.Errorf(`%w: the loop's zero-pose enclosure reads no value for a dependent joint`, ErrUnsupported)
		}
		sc.e0Readings = append(sc.e0Readings, r)
	}
	return nil
}

// intervalHolds reports whether the outward-rounded iv holds every value of
// the exact enclosure x. A pin whose enclosure is not proven inside its box
// is refused, so the check can only refuse.
func intervalHolds(iv sketch.Interval, x proofbound.RatInterval) bool {
	lo, hi := proofarith.FloatRat(iv.Lo), proofarith.FloatRat(iv.Hi)
	return lo != nil && hi != nil && lo.Cmp(x.Lo) <= 0 && x.Hi.Cmp(hi) <= 0
}

// loopInvariant reports a sketch refusal that is an invariant failure on a
// scene decad built (docs/linkage-check-design.md §15.6).
func loopInvariant(err error) bool {
	return errors.Is(err, sketch.ErrUncertifiedConstraint) || errors.Is(err, sketch.ErrForeignHandle) || errors.Is(err, sketch.ErrNonFiniteGeometry)
}

// sceneValue is the driver's scene value at the exact fraction s, as the two
// floats around it: |q(s)| in radians or millimetres, each sub-segment read on
// the side where it is never negative (docs/linkage-check-design.md §15.3).
func (ld *loopDrive) sceneValue(s *big.Rat) (float64, float64) {
	q := jointParam(ld.driverJt, s)
	lo, hi := paramLower(q), paramUpper(q)
	if hi.Sign() <= 0 {
		lo, hi = hi.Neg(hi), lo.Neg(lo)
	}
	if lo.Sign() < 0 {
		lo = new(big.Rat)
	}
	return proofbound.RatFloatDown(lo), proofbound.RatFloatUp(hi)
}

// paramLower and paramUpper bound 2π·turn + base from below and above, π at
// its enclosure's ends.
func paramLower(p motionbound.MotionParam) *big.Rat {
	twoPi := proofbound.TwoPiInterval()
	f := twoPi.Lo
	if p.Turn.Sign() < 0 {
		f = twoPi.Hi
	}
	out := new(big.Rat).Mul(p.Turn, f)
	return out.Add(out, p.Base)
}

func paramUpper(p motionbound.MotionParam) *big.Rat {
	twoPi := proofbound.TwoPiInterval()
	f := twoPi.Hi
	if p.Turn.Sign() < 0 {
		f = twoPi.Lo
	}
	out := new(big.Rat).Mul(p.Turn, f)
	return out.Add(out, p.Base)
}

// enclose asks sketch for the enclosure of [lo, hi] on pred's scene,
// continued from pred, or records the refusal that takes its place; a refusal
// of pred carries over. Only a context error or an invariant failure is
// returned as an error, and neither is cached.
func (ld *loopDrive) enclose(ctx context.Context, key string, lo, hi float64, pred *loopAsk) (*loopAsk, error) {
	if ask, ok := ld.asks[key]; ok {
		return ask, nil
	}
	ask, err := encloseFresh(ctx, lo, hi, pred)
	if err != nil {
		return nil, err
	}
	ld.asks[key] = ask
	return ask, nil
}

func encloseFresh(ctx context.Context, lo, hi float64, pred *loopAsk) (*loopAsk, error) {
	if pred.err != nil {
		return &loopAsk{err: pred.err}, nil //nolint:nilerr // a predecessor's refusal is this ask's, recorded rather than returned
	}
	if lo > hi {
		return &loopAsk{err: fmt.Errorf(`%w: the driver range [%v, %v] is empty`, sketch.ErrNotCertified, lo, hi)}, nil
	}
	sc := pred.scene
	opts := append(slices.Clone(sc.opts), sketch.WithContinuation(pred.enc))
	enc, err := sc.sk.Enclose(ctx, sc.driver, lo, hi, opts...)
	if err != nil {
		if cerr := ctx.Err(); cerr != nil {
			return nil, cerr
		}
		if loopInvariant(err) {
			return nil, err
		}
		return &loopAsk{err: err}, nil
	}
	ask := &loopAsk{scene: sc, enc: enc, turns: make([]int64, len(sc.driven))}
	prevPieces, pieces := pred.enc.Pieces(), enc.Pieces()
	last, first := prevPieces[len(prevPieces)-1], pieces[0]
	for j, d := range sc.driven {
		l, okL := last.Driven(d)
		f, okF := first.Driven(d)
		if !okL || !okF {
			return &loopAsk{err: fmt.Errorf(`%w: an enclosure reads no value for a dependent joint`, sketch.ErrNotCertified)}, nil
		}
		m := 0.0
		if sc.angular[j] {
			// A slide's reading is a length, never read modulo a turn; its
			// continuation is still checked for overlap below.
			m = math.Round(((l.Lo+l.Hi)/2 - (f.Lo+f.Hi)/2) / (2 * math.Pi))
		}
		if math.IsNaN(m) || math.Abs(m) > 1<<40 {
			return &loopAsk{err: fmt.Errorf(`%w: a dependent reading cannot be continued`, sketch.ErrNotCertified)}, nil
		}
		shift := int64(m)
		if !turnsOverlap(f, shift, l) {
			return &loopAsk{err: fmt.Errorf(`%w: a dependent reading does not continue its predecessor's by any whole turn`, sketch.ErrNotCertified)}, nil
		}
		ask.turns[j] = pred.turns[j] + shift
	}
	return ask, nil
}

// turnsOverlap reports whether f shifted by m whole turns can meet l: false
// only when the two are proven disjoint for every π in its enclosure. A
// continued enclosure's first reading and its predecessor's last enclose one
// configuration, so a proven gap disproves the continuation.
func turnsOverlap(f sketch.Interval, m int64, l sketch.Interval) bool {
	turn := big.NewRat(m, 1)
	shiftLo := paramLower(motionbound.MotionParam{Turn: turn, Base: proofarith.FloatRat(f.Lo)})
	shiftHi := paramUpper(motionbound.MotionParam{Turn: turn, Base: proofarith.FloatRat(f.Hi)})
	return shiftLo.Cmp(proofarith.FloatRat(l.Hi)) <= 0 && proofarith.FloatRat(l.Lo).Cmp(shiftHi) <= 0
}

// linkageGridDepth is the deepest dyadic level the canonical chain reaches
// through binary cells; a deeper parameter is reached by one cell from the
// nearest grid point of linkageReadingFloor's depth on its near side.
const linkageGridDepth = 30

// chainStart is where the cell ending at s starts in sub's canonical chain
// (docs/linkage-check-design.md §15.3): the depth-d cell ending at s for a
// parameter of depth d, or the nearest grid parameter of the reading floor's
// depth on the near side for a parameter on no grid this file walks, never
// past the sub-segment's near end.
func chainStart(sub loopSub, s *big.Rat) *big.Rat {
	start := gridStart(sub, s)
	if sub.increasing() && start.Cmp(sub.near) < 0 || !sub.increasing() && start.Cmp(sub.near) > 0 {
		return new(big.Rat).Set(sub.near)
	}
	return start
}

func gridStart(sub loopSub, s *big.Rat) *big.Rat {
	den := s.Denom()
	depth := den.BitLen() - 1
	dyadic := new(big.Int).Lsh(big.NewInt(1), uint(depth)).Cmp(den) == 0
	if dyadic && depth <= linkageGridDepth {
		step := new(big.Rat).SetFrac(big.NewInt(1), den)
		if sub.increasing() {
			return step.Sub(s, step)
		}
		return step.Add(s, step)
	}
	scaled := new(big.Rat).Mul(s, big.NewRat(linkageReadingFloor, 1))
	q := new(big.Int).Div(scaled.Num(), scaled.Denom())
	if !sub.increasing() {
		q.Add(q, big.NewInt(1))
	}
	anchor := new(big.Rat).SetFrac(q, big.NewInt(linkageReadingFloor))
	if anchor.Cmp(s) == 0 {
		// Unreachable: a parameter on the reading floor's grid is dyadic of
		// depth 14 and takes the branch above. The near end keeps the chain
		// finite whatever reaches here.
		return new(big.Rat).Set(sub.near)
	}
	return anchor
}

// askKey names an ask of sub's chain.
func askKey(sub loopSub, kind string, ends ...*big.Rat) string {
	var b strings.Builder
	fmt.Fprintf(&b, "%d:%s", sub.idx, kind)
	for n, e := range ends {
		if n > 0 {
			b.WriteByte(',')
		}
		b.WriteString(e.RatString())
	}
	return b.String()
}

// point is the point ask at the exact fraction s in sub's chain, continued
// from its canonical predecessor. A sub-segment that holds its driver has one
// point, at its near end. Callers hold mu.
func (ld *loopDrive) point(ctx context.Context, sub loopSub, s *big.Rat) (*loopAsk, error) {
	if sub.held {
		s = sub.near
	}
	if s.Cmp(sub.near) == 0 {
		if sub.nearZero {
			return ld.scenes[sub.side].e0, nil
		}
		approach, err := ld.approach(ctx, sub)
		if err != nil {
			return nil, err
		}
		lo, hi := ld.sceneValue(s)
		return ld.enclose(ctx, askKey(sub, "p", s), lo, hi, approach)
	}
	key := askKey(sub, "p", s)
	if ask, ok := ld.asks[key]; ok {
		return ask, nil
	}
	cell, err := ld.cell(ctx, sub, chainStart(sub, s), s)
	if err != nil {
		return nil, err
	}
	lo, hi := ld.sceneValue(s)
	return ld.enclose(ctx, key, lo, hi, cell)
}

// straddleAsks are the asks a straddle's values are read from
// (docs/linkage-check-design.md §15.8): for each neighbour, its approach from
// the zero pose and its point at its cut. The driver's value anywhere on the
// straddle lies between its values at the two cuts, on one side of 0 or the
// other, so every dependent value there lies in the hull of these four.
// Callers hold mu.
func (ld *loopDrive) straddleAsks(ctx context.Context, sub loopSub) ([]*loopAsk, error) {
	var out []*loopAsk
	for _, nb := range []loopSub{ld.subs[sub.idx-1], ld.subs[sub.idx+1]} {
		approach, err := ld.approach(ctx, nb)
		if err != nil {
			return nil, err
		}
		at, err := ld.point(ctx, nb, nb.near)
		if err != nil {
			return nil, err
		}
		out = append(out, approach, at)
	}
	return out, nil
}

// askHull is dependent j's hull over every ask given.
func (ld *loopDrive) askHull(asks []*loopAsk, j int) proofbound.RatInterval {
	ivs := make([]proofbound.RatInterval, len(asks))
	for n, ask := range asks {
		ivs[n] = valueInterval(ld.value(ask, j))
	}
	return hull(ivs...)
}

// approach is the enclosure from the zero pose to sub's near-end driver value.
func (ld *loopDrive) approach(ctx context.Context, sub loopSub) (*loopAsk, error) {
	lo, _ := ld.sceneValue(sub.near)
	return ld.enclose(ctx, askKey(sub, "a"), 0, lo, ld.scenes[sub.side].e0)
}

// cell is the cell ask from start to end in sub's chain order, continued from
// the point at start; a sub-segment that holds its driver answers its one
// point. Callers hold mu.
func (ld *loopDrive) cell(ctx context.Context, sub loopSub, start, end *big.Rat) (*loopAsk, error) {
	if sub.held {
		return ld.point(ctx, sub, sub.near)
	}
	key := askKey(sub, "c", start, end)
	if ask, ok := ld.asks[key]; ok {
		return ask, nil
	}
	from, err := ld.point(ctx, sub, start)
	if err != nil {
		return nil, err
	}
	_, lo := ld.sceneValue(start)
	hi, _ := ld.sceneValue(end)
	return ld.enclose(ctx, key, lo, hi, from)
}

// chainCell is the cell over the piece [a, b], a < b, in its sub-segment's
// chain order.
func (ld *loopDrive) chainCell(ctx context.Context, sub loopSub, a, b *big.Rat) (*loopAsk, error) {
	if sub.increasing() {
		return ld.cell(ctx, sub, a, b)
	}
	return ld.cell(ctx, sub, b, a)
}

// value is dependent j's joint value over an enclosure: its whole hull, the
// reading shifted by the ask's whole turns, less its scene's zero-pose
// reading, in the joint's own sense on that scene. The two ends share one
// turn count.
func (ld *loopDrive) value(ask *loopAsk, j int) (motionbound.MotionParam, motionbound.MotionParam) {
	sc := ask.scene
	iv, _ := ask.enc.Driven(sc.driven[j])
	r0 := sc.e0Readings[j]
	lo := new(big.Rat).Sub(proofarith.FloatRat(iv.Lo), proofarith.FloatRat(r0.Hi))
	hi := new(big.Rat).Sub(proofarith.FloatRat(iv.Hi), proofarith.FloatRat(r0.Lo))
	turn := big.NewRat(ask.turns[j], 1)
	if sc.signs[j] < 0 {
		lo, hi = hi.Neg(hi), lo.Neg(lo)
		turn.Neg(turn)
	}
	return motionbound.MotionParam{Turn: turn, Base: lo}, motionbound.MotionParam{Turn: new(big.Rat).Set(turn), Base: hi}
}

// valueInterval is a value range as one rational radian interval.
func valueInterval(lo, hi motionbound.MotionParam) proofbound.RatInterval {
	return proofbound.IntervalOwned(paramLower(lo), paramUpper(hi))
}

// hull is the smallest interval holding every one given.
func hull(ivs ...proofbound.RatInterval) proofbound.RatInterval {
	lo, hi := ivs[0].Lo, ivs[0].Hi
	for _, iv := range ivs[1:] {
		if iv.Lo.Cmp(lo) < 0 {
			lo = iv.Lo
		}
		if iv.Hi.Cmp(hi) > 0 {
			hi = iv.Hi
		}
	}
	return proofbound.IntervalOwned(new(big.Rat).Set(lo), new(big.Rat).Set(hi))
}

// magnitude is the largest |x| over an interval.
func magnitude(iv proofbound.RatInterval) *big.Rat {
	m := new(big.Rat).Abs(iv.Lo)
	if hi := new(big.Rat).Abs(iv.Hi); hi.Cmp(m) > 0 {
		m = hi
	}
	return m
}

// decompose asks the drive as one cell, each piece of it in its own
// sub-segment's chain, and replaces each cell some piece of which is refused
// by its two halves down to floor (docs/linkage-check-design.md §15.7). The
// hull of every certified piece and of the points at its ends bounds each
// dependent's value over the certified drive: that is its reach m_i, every
// later enclosure is held to it, and a dependent with limits must hold it
// inside them.
func (ld *loopDrive) decompose(ctx context.Context, spec *linkageSpec, floor *big.Rat) error {
	ld.mu.Lock()
	defer ld.mu.Unlock()
	var hulls []proofbound.RatInterval
	add := func(ask *loopAsk) {
		if ask.err != nil {
			return
		}
		for j := range ld.deps {
			iv := valueInterval(ld.value(ask, j))
			if len(hulls) <= j {
				hulls = append(hulls, iv)
				continue
			}
			hulls[j] = hull(hulls[j], iv)
		}
	}
	for _, sub := range ld.subs {
		if sub.straddle {
			// Its neighbours' near ends are its values' sources.
			continue
		}
		near, err := ld.point(ctx, sub, sub.near)
		if err != nil {
			return err
		}
		add(near)
	}
	var walk func(a, b *big.Rat) error
	walk = func(a, b *big.Rat) error {
		var asks []*loopAsk
		refused := false
		for _, pc := range ld.pieces(a, b) {
			if pc.sub.straddle {
				st, err := ld.straddleAsks(ctx, pc.sub)
				if err != nil {
					return err
				}
				for _, ask := range st {
					refused = refused || ask.err != nil
				}
				asks = append(asks, st...)
				continue
			}
			c, err := ld.chainCell(ctx, pc.sub, pc.lo, pc.hi)
			if err != nil {
				return err
			}
			refused = refused || c.err != nil
			asks = append(asks, c)
			for _, end := range []*big.Rat{pc.lo, pc.hi} {
				p, err := ld.point(ctx, pc.sub, end)
				if err != nil {
					return err
				}
				asks = append(asks, p)
			}
		}
		if !refused {
			for _, ask := range asks {
				add(ask)
			}
			return nil
		}
		width := new(big.Rat).Sub(b, a)
		if width.Cmp(floor) <= 0 {
			return nil
		}
		mid := new(big.Rat).Add(a, b)
		mid.Quo(mid, big.NewRat(2, 1))
		if err := walk(a, mid); err != nil {
			return err
		}
		return walk(mid, b)
	}
	if err := walk(new(big.Rat), big.NewRat(1, 1)); err != nil {
		return err
	}
	ld.reach = make([]*big.Rat, len(ld.deps))
	ld.hulls = hulls
	for j := range ld.deps {
		ld.reach[j] = new(big.Rat)
		if j < len(hulls) {
			ld.reach[j] = magnitude(hulls[j])
		}
	}
	return ld.checkLimits(spec)
}

// checkLimits holds every dependent with declared limits to its whole-drive
// hull (docs/linkage-check-design.md §15.5): a hull not proven inside
// [Min, Max] — its lower end at or above Min's upper enclosure and its upper
// end at or below Max's lower — is ErrDegenerate naming the link. The hull is
// wider than the exact value set by the pieces' slack, so a drive that
// reaches a limit exactly is refused; the refusal admits nothing.
func (ld *loopDrive) checkLimits(spec *linkageSpec) error {
	for j, d := range ld.deps {
		jt := spec.joints[d]
		if jt.limits == nil {
			continue
		}
		lo, okLo := motionbound.ExactMotionParam(jt.limits.Min)
		hi, okHi := motionbound.ExactMotionParam(jt.limits.Max)
		inside := okLo && okHi && j < len(ld.hulls) &&
			ld.hulls[j].Lo.Cmp(paramUpper(lo)) >= 0 && ld.hulls[j].Hi.Cmp(paramLower(hi)) <= 0
		if inside {
			continue
		}
		unit := "rad"
		if !jt.revolute {
			unit = "mm"
		}
		reach := "nothing certified"
		if j < len(ld.hulls) {
			reach = fmt.Sprintf("[%s, %s] %s", ld.hulls[j].Lo.FloatString(6), ld.hulls[j].Hi.FloatString(6), unit)
		}
		return fmt.Errorf(`%w: the drive takes link %d's dependent joint over %s, not inside its limits [%s, %s]`,
			ErrDegenerate, d, reach, jt.limits.Min, jt.limits.Max)
	}
	return nil
}

// withinReach reports whether a value interval lies inside dependent j's
// reach; one outside it is refused, so the reach every swept box was grown by
// covers every value a claim is made about.
func (ld *loopDrive) withinReach(j int, iv proofbound.RatInterval) bool {
	return magnitude(iv).Cmp(ld.reach[j]) <= 0
}

// pointValues is every dependent's value range at the exact fraction s, read
// on the sub-segment subAt picks, or the refusal that makes the pose
// unbuildable.
func (ld *loopDrive) pointValues(ctx context.Context, s *big.Rat) ([][2]motionbound.MotionParam, error) {
	ld.mu.Lock()
	defer ld.mu.Unlock()
	sub := ld.subAt(s)
	if sub.straddle {
		return ld.straddleValues(ctx, sub)
	}
	ask, err := ld.point(ctx, sub, s)
	if err != nil {
		return nil, err
	}
	if ask.err != nil {
		return nil, &unbuildableError{cause: ld.describe(ask.err)}
	}
	out := make([][2]motionbound.MotionParam, len(ld.deps))
	for j := range ld.deps {
		lo, hi := ld.value(ask, j)
		if !ld.withinReach(j, valueInterval(lo, hi)) {
			return nil, &unbuildableError{cause: ld.describe(fmt.Errorf(`%w: a dependent value lies outside the certified drive's reach`, sketch.ErrNotCertified))}
		}
		out[j] = [2]motionbound.MotionParam{lo, hi}
	}
	return out, nil
}

// straddleValues is every dependent's value range at a fraction inside a
// straddle: its hull over the straddle's asks, as radians or millimetres.
// Callers hold mu.
func (ld *loopDrive) straddleValues(ctx context.Context, sub loopSub) ([][2]motionbound.MotionParam, error) {
	asks, err := ld.straddleAsks(ctx, sub)
	if err != nil {
		return nil, err
	}
	for _, ask := range asks {
		if ask.err != nil {
			return nil, &unbuildableError{cause: ld.describe(ask.err)}
		}
	}
	out := make([][2]motionbound.MotionParam, len(ld.deps))
	for j := range ld.deps {
		h := ld.askHull(asks, j)
		if !ld.withinReach(j, h) {
			return nil, &unbuildableError{cause: ld.describe(fmt.Errorf(`%w: a dependent value lies outside the certified drive's reach`, sketch.ErrNotCertified))}
		}
		out[j] = [2]motionbound.MotionParam{{Turn: new(big.Rat), Base: h.Lo}, {Turn: new(big.Rat), Base: h.Hi}}
	}
	return out, nil
}

// describe names the loop in a refusal.
func (ld *loopDrive) describe(err error) error {
	return fmt.Errorf(`the loop closing links %d and %d: %w`, ld.loop.a.index, ld.loop.b.index, err)
}

// intervalSpans reads every dependent's values over [a, b], cut at every
// sub-segment boundary inside it: per piece, the values at its two ends and
// the hull over it, from its sub-segment's point asks and the cell between
// them; err is the refusal that leaves the interval undecided.
func (ld *loopDrive) intervalSpans(ctx context.Context, a, b *big.Rat) ([][]loopSpan, error) {
	ld.mu.Lock()
	defer ld.mu.Unlock()
	var out [][]loopSpan
	for _, pc := range ld.pieces(a, b) {
		if pc.sub.straddle {
			piece, err := ld.straddleSpans(ctx, pc.sub)
			if err != nil {
				return nil, err
			}
			out = append(out, piece)
			continue
		}
		pa, err := ld.point(ctx, pc.sub, pc.lo)
		if err != nil {
			return nil, err
		}
		pb, err := ld.point(ctx, pc.sub, pc.hi)
		if err != nil {
			return nil, err
		}
		c, err := ld.chainCell(ctx, pc.sub, pc.lo, pc.hi)
		if err != nil {
			return nil, err
		}
		for _, ask := range []*loopAsk{pa, c, pb} {
			if ask.err != nil {
				return nil, &unbuildableError{cause: ld.describe(ask.err)}
			}
		}
		piece := make([]loopSpan, len(ld.deps))
		for j := range ld.deps {
			A, B, C := valueInterval(ld.value(pa, j)), valueInterval(ld.value(pb, j)), valueInterval(ld.value(c, j))
			H := hull(A, B, C)
			if !ld.withinReach(j, H) {
				return nil, &unbuildableError{cause: ld.describe(fmt.Errorf(`%w: a dependent value lies outside the certified drive's reach`, sketch.ErrNotCertified))}
			}
			piece[j] = loopSpan{a: A, b: B, h: H}
		}
		out = append(out, piece)
	}
	ld.spans[loopSpanKey(a, b)] = out
	return out, nil
}

// straddleSpans reads every dependent over a whole straddle: its values at
// the two cuts, the neighbours' points there, and the hull over its asks. An
// interval is cut at both of a straddle's ends, so its piece is the whole
// straddle. Callers hold mu.
func (ld *loopDrive) straddleSpans(ctx context.Context, sub loopSub) ([]loopSpan, error) {
	asks, err := ld.straddleAsks(ctx, sub)
	if err != nil {
		return nil, err
	}
	for _, ask := range asks {
		if ask.err != nil {
			return nil, &unbuildableError{cause: ld.describe(ask.err)}
		}
	}
	piece := make([]loopSpan, len(ld.deps))
	for j := range ld.deps {
		h := ld.askHull(asks, j)
		if !ld.withinReach(j, h) {
			return nil, &unbuildableError{cause: ld.describe(fmt.Errorf(`%w: a dependent value lies outside the certified drive's reach`, sketch.ErrNotCertified))}
		}
		piece[j] = loopSpan{a: valueInterval(ld.value(asks[1], j)), b: valueInterval(ld.value(asks[3], j)), h: h}
	}
	return piece, nil
}

// dependentSpan is |Δq_k| of docs/linkage-check-design.md §15.5 for one
// dependent over an interval's piece: the largest |x − q_a| + |q_b − x| over
// x in the hull H and q_a, q_b in the end enclosures A and B, which sits at an
// end of H: max(a_hi + b_hi − 2·h_lo, 2·h_hi − a_lo − b_lo).
func dependentSpan(sp loopSpan) *big.Rat {
	low := new(big.Rat).Add(sp.a.Hi, sp.b.Hi)
	low.Sub(low, new(big.Rat).Mul(big.NewRat(2, 1), sp.h.Lo))
	high := new(big.Rat).Mul(big.NewRat(2, 1), sp.h.Hi)
	high.Sub(high, sp.a.Lo)
	high.Sub(high, sp.b.Lo)
	if high.Cmp(low) > 0 {
		return high
	}
	return low
}

// span is a dependent joint's |Δq| over [sa, sb]: the sum of dependentSpan
// over the pieces intervalGate read the interval in, which bounds the joint's
// total variation since that is additive over the cuts; nil when there are no
// readings.
func (ld *loopDrive) span(joint int, sa, sb *big.Rat) *big.Rat {
	lo, hi := sa, sb
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	ld.mu.Lock()
	pieces, ok := ld.spans[loopSpanKey(lo, hi)]
	ld.mu.Unlock()
	j := slices.Index(ld.deps, joint)
	if !ok || j < 0 {
		return nil
	}
	sum := new(big.Rat)
	for _, piece := range pieces {
		sum.Add(sum, dependentSpan(piece[j]))
	}
	return sum
}

// loopSpanKey names a verification interval.
func loopSpanKey(a, b *big.Rat) string { return a.RatString() + "," + b.RatString() }

// loopPosesAt builds every link's pose at the exact fraction f for a drive
// that moves a loop (docs/linkage-check-design.md §15.4): each stated joint at
// its label, each dependent at the float midpoint of its enclosure, both
// posed by posesOf; every link's ideal pose composed over the dependent's
// whole enclosure by MotionFrame.AtRange; and each value's proven half-width.
func (s *linkageSpec) loopPosesAt(ctx context.Context, frames []motionbound.MotionFrame, f *big.Rat) ([]units.Value, []units.Value, []r3.Transform, []motionbound.IdealPose, error) {
	n := len(s.joints)
	values, bounds := make([]units.Value, n), make([]units.Value, n)
	lo, hi := make([]motionbound.MotionParam, n), make([]motionbound.MotionParam, n)
	for k, jt := range s.joints {
		values[k] = jt.label(f)
		bounds[k] = units.New(0, values[k].Unit())
		p := jointParam(jt, f)
		lo[k], hi[k] = p, p
	}
	for _, ld := range s.loops {
		ranges, err := ld.pointValues(ctx, f)
		if err != nil {
			return nil, nil, nil, nil, err
		}
		for j, d := range ld.deps {
			lo[d], hi[d] = ranges[j][0], ranges[j][1]
			iv := valueInterval(lo[d], hi[d])
			mid := new(big.Rat).Add(iv.Lo, iv.Hi)
			label, _ := mid.Quo(mid, big.NewRat(2, 1)).Float64()
			exact := proofarith.FloatRat(label)
			half := new(big.Rat).Sub(exact, iv.Lo)
			if up := new(big.Rat).Sub(iv.Hi, exact); up.Cmp(half) > 0 {
				half = up
			}
			unit := units.Radian
			if !s.joints[d].revolute {
				unit = units.Millimeter
			}
			values[d] = units.New(label, unit)
			bounds[d] = units.New(proofbound.RatFloatUp(half), unit)
		}
	}
	poses, err := s.posesOf(values)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	var ideals []motionbound.IdealPose
	if frames != nil {
		ideals = make([]motionbound.IdealPose, n)
		for k, jt := range s.joints {
			ideal := frames[k].AtRange(lo[k], hi[k])
			if jt.parent >= 0 {
				ideal = ideal.Then(ideals[jt.parent])
			}
			ideals[k] = ideal
		}
	}
	return values, bounds, poses, ideals, nil
}

// prepareLoops builds every driven loop's scene and E0 under a context that is
// never canceled, so a refusal there is a validation error, then asks the
// drive's certifiable set down to floor under ctx and records each
// dependent's reach.
func (s *linkageSpec) prepareLoops(ctx context.Context, floor *big.Rat) error {
	if err := s.prepare(context.WithoutCancel(ctx)); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	for _, ld := range s.loops {
		if err := ld.decompose(ctx, s, floor); err != nil {
			return err
		}
		for j, d := range ld.deps {
			s.joints[d].depReach = ld.reach[j]
		}
	}
	return nil
}

// Schedule is a drive prepared for repeated PoseAt calls on one linkage
// (docs/linkage-check-design.md §15.7). For every loop the drive moves it
// holds the private scene and the chain of certified enclosures, built as far
// as the drive could be certified. Its PoseAt is safe for concurrent use.
type Schedule struct {
	linkage *Linkage
	drive   Drive
	spec    *linkageSpec
}

// linkageScheduleFloor is the fraction a Schedule's certifiable set is asked
// down to, the default verdict floor.
const linkageScheduleFloor = 1024

// Schedule validates d as PoseAt does and prepares it for repeated PoseAt
// calls. For a drive that moves a loop it builds the loop's scene, asks its
// zero pose, and asks the drive as one cell, each refused cell replaced by its
// two halves down to 1/1024 of the drive. A nil context is ErrDegenerate; a
// loop the zero pose cannot be enclosed for is ErrUnsupported wrapping
// sketch's error; a canceled context returns ctx.Err().
func (l *Linkage) Schedule(ctx context.Context, d Drive) (*Schedule, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a schedule`, ErrDegenerate)
	}
	spec, err := l.resolveDrive(d)
	if err != nil {
		return nil, err
	}
	if len(spec.loops) > 0 {
		if err := spec.prepareLoops(ctx, big.NewRat(1, linkageScheduleFloor)); err != nil {
			return nil, err
		}
	}
	return &Schedule{linkage: l, drive: slices.Clone(d), spec: spec}, nil
}

// Drive returns the drive the schedule was built for.
func (s *Schedule) Drive() Drive {
	return slices.Clone(s.drive)
}

// PoseAt returns every link's joint value, its proven half-width and its
// world pose at the fraction at (docs/linkage-check-design.md §15.4, §15.7).
// For a tree linkage it is Linkage.PoseAt with zero Bounds. For a drive that
// moves a loop, each dependent joint's value is the float midpoint of its
// certified enclosure and Bounds its half-width; an at outside [0, 1], or one
// whose loop cannot be enclosed, is ErrUnsupported wrapping sketch's error.
// Two calls at the same at return equal poses bit for bit.
func (s *Schedule) PoseAt(ctx context.Context, at units.Value) (LinkagePose, error) {
	if ctx == nil {
		return LinkagePose{}, fmt.Errorf(`%w: a nil context cannot control a pose`, ErrDegenerate)
	}
	if err := motionValueValid(at, units.Dimensionless, "the pose fraction"); err != nil {
		return LinkagePose{}, err
	}
	p, ok := motionbound.ExactMotionParam(at)
	if !ok {
		return LinkagePose{}, fmt.Errorf(`%w: the pose fraction is not representable`, ErrNotFinite)
	}
	return s.spec.poseAt(ctx, at, p.Base)
}

// poseAt is one LinkagePose at the exact fraction f, published as at.
func (s *linkageSpec) poseAt(ctx context.Context, at units.Value, f *big.Rat) (LinkagePose, error) {
	if len(s.loops) == 0 {
		values, poses, err := s.posesAt(f)
		if err != nil {
			return LinkagePose{}, err
		}
		return LinkagePose{At: at, Values: values, Bounds: zeroBounds(values), Poses: poses}, nil
	}
	if f.Sign() < 0 || f.Cmp(big.NewRat(1, 1)) > 0 {
		return LinkagePose{}, fmt.Errorf(`%w: a drive that moves a loop is enclosed over [0, 1] only, not at %s`, ErrUnsupported, at)
	}
	values, bounds, poses, _, err := s.loopPosesAt(ctx, nil, f)
	if err != nil {
		var ub *unbuildableError
		if errors.As(err, &ub) {
			return LinkagePose{}, fmt.Errorf(`%w: the drive's loop cannot be enclosed at %s: %w`, ErrUnsupported, at, ub.cause)
		}
		return LinkagePose{}, err
	}
	return LinkagePose{At: at, Values: values, Bounds: bounds, Poses: poses}, nil
}

// zeroBounds is a zero half-width per stated value, in its own unit.
func zeroBounds(values []units.Value) []units.Value {
	out := make([]units.Value, len(values))
	for k, v := range values {
		out[k] = units.New(0, v.Unit())
	}
	return out
}
