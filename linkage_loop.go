package decad

import (
	"fmt"
	"math"
	"math/big"
	"slices"
	"sync"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/linkagebound/loopchain"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the closed loop of agent-docs/linkage-check-design.md §15: Close and
// Loop, the private sketch scene a driven loop is read from, the canonical
// chain of certified enclosures every dependent joint value comes from, the
// Schedule a renderer calls per frame, and the asks a joint box over a
// loop reads per cell (§16). decad computes no 2D answer here:
// every dependent value is sketch.Enclose's certified enclosure, and the
// checks this file runs on what sketch returns only ever refuse.

// LinkageLoop is one closure joint of a linkage and the links it ties together
// (agent-docs/linkage-check-design.md §15.1): a revolute pin joining two links that
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
	// slide is the loop's primary slide: the first prismatic link in Links()
	// order whose parent is Common, nil when there is none. Its rail is the
	// scene's u axis. Every other prismatic loop link is anchored
	// (agent-docs/linkage-check-design.md §15.2), and kappa holds each one's κ.
	slide *Link
	kappa map[*Link]*big.Rat
	reach float64 // four times the loop's perimeter at the zero pose
	bars  []loopBar
}

// LoopBar is the length a loop's private scene holds one link's two loop
// pins apart at (agent-docs/linkage-check-design.md §15.2): the exact distance, in
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
// pins apart at: Common's bar first, then Links() order. A slide has no bar,
// and neither has a link one of whose loop pins is a slide's rail: Common
// beside a slide, or a revolute link whose next link slides.
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
// pose), so the tree gains one loop (agent-docs/linkage-check-design.md §15.1).
// Which joint drives the loop is stated per Drive.
//
// It refuses, in this order: a nil linkage, a nil or parentless a or b,
// a == b, or a link of another linkage (ErrDegenerate); a non-finite center or
// axis component (ErrNotFinite); the zero axis (ErrDegenerate); a loop
// revolute whose axis is not exactly parallel to axis, or a loop prismatic
// whose direction is not exactly perpendicular to it (ErrUnsupported, naming
// the joint); two consecutive loop slides along exactly parallel directions
// (ErrDegenerate); a loop joint already on another loop (ErrUnsupported); and
// two loop pins of one link coincident in the
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
	if idx, _, ok := linkagebound.CoordinateAxis(axis); ok {
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
			if c := linkagebound.Cross(ratVecExact(j.Axis), lp.normal); !linkagebound.ZeroVec(c) {
				return nil, fmt.Errorf(`%w: link %d's revolute axis %v is not exactly parallel to the closure axis %v`, ErrUnsupported, k.index, j.Axis, axis)
			}
		case PrismaticJoint:
			if linkagebound.Dot(ratVecExact(j.Dir), lp.normal).Sign() != 0 {
				return nil, fmt.Errorf(`%w: link %d's prismatic direction %v is not exactly perpendicular to the closure axis %v`, ErrUnsupported, k.index, j.Dir, axis)
			}
		}
	}
	for _, k := range lp.links {
		if !isSlide(k) {
			continue
		}
		if k.parent == lp.common && lp.slide == nil {
			lp.slide = k
		}
		if k.parent != lp.common && isSlide(k.parent) &&
			linkagebound.ZeroVec(linkagebound.Cross(slideDir(k), slideDir(k.parent))) {
			return nil, fmt.Errorf(`%w: links %d and %d slide in a row along parallel directions, which slides the loop with no driver`, ErrDegenerate, k.parent.index, k.index)
		}
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
	lp.readKappa()
	l.loops = append(l.loops, lp)
	return lp, nil
}

// ratVecExact reads a finite held vector exactly.
func ratVecExact(v r3.Vec) motionbound.RatVec { return linkagebound.ExactVec(v) }

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
	return lp.planeSqRat(ratVecExact(p), ratVecExact(q))
}

// planeSqRat is planeSq between two exact points.
func (lp *LinkageLoop) planeSqRat(pd, qd motionbound.RatVec) *big.Rat {
	var d motionbound.RatVec
	for i := range 3 {
		d[i] = new(big.Rat).Sub(pd[i], qd[i])
	}
	along := linkagebound.Dot(d, lp.normal)
	along.Mul(along, along)
	along.Quo(along, linkagebound.Dot(lp.normal, lp.normal))
	return along.Sub(linkagebound.Dot(d, d), along)
}

// linkNext is loop link k's next pin along its side of the loop.
func (lp *LinkageLoop) linkNext(k *Link) r3.Vec {
	side := lp.sideOf(k)
	return lp.nextPin(side, slices.Index(side, k))
}

// sideOf is the side of the loop k lies on.
func (lp *LinkageLoop) sideOf(k *Link) []*Link {
	if slices.Contains(lp.sideA, k) {
		return lp.sideA
	}
	return lp.sideB
}

// nextLink is the link after k along its side of the loop, nil for the side's
// last link, whose next pin is the closure.
func (lp *LinkageLoop) nextLink(k *Link) *Link {
	side := lp.sideOf(k)
	if n := slices.Index(side, k); n+1 < len(side) {
		return side[n+1]
	}
	return nil
}

// isSlide reports whether k's joint is prismatic.
func isSlide(k *Link) bool {
	_, ok := k.joint.(PrismaticJoint)
	return ok
}

// slideDir is a slide's Dir, read exactly.
func slideDir(k *Link) motionbound.RatVec {
	j, _ := k.joint.(PrismaticJoint)
	return ratVecExact(j.Dir)
}

// anchored reports whether k is a loop slide other than the primary one,
// whose rail is held by an anchor (agent-docs/linkage-check-design.md §15.2).
func (lp *LinkageLoop) anchored(k *Link) bool {
	return isSlide(k) && k != lp.slide
}

// besideRail reports whether k's next link slides, so that k's second loop
// pin is that slide's rail.
func (lp *LinkageLoop) besideRail(k *Link) bool {
	next := lp.nextLink(k)
	return next != nil && isSlide(next)
}

// commonRail reports whether either side's first link slides, so that one of
// Common's loop pins is a rail.
func (lp *LinkageLoop) commonRail() bool {
	return len(lp.sideA) > 0 && isSlide(lp.sideA[0]) || len(lp.sideB) > 0 && isSlide(lp.sideB[0])
}

// riderAt is slide k's rider's zero-pose position P₀: the first revolute pin
// after k along its side of the loop, or the closure.
func (lp *LinkageLoop) riderAt(k *Link) r3.Vec {
	for next := lp.nextLink(k); next != nil; next = lp.nextLink(next) {
		if !isSlide(next) {
			return next.pin()
		}
	}
	return lp.closure.Center
}

// featureAt is loop link k's zero-pose position along the loop: its pin, or a
// slide's rider's P₀.
func (lp *LinkageLoop) featureAt(k *Link) r3.Vec {
	if isSlide(k) {
		return lp.riderAt(k)
	}
	return k.pin()
}

// readKappa reads the loop's reach — four times its perimeter, the sum of the
// plane distances between consecutive loop features at the zero pose — and
// picks κ for every anchored slide (agent-docs/linkage-check-design.md §15.2): the
// scale that carries its Dir to the reach, doubled until neither anchor
// meets a revolute parent's pin in the plane. κ only places the anchor; no
// claim depends on its value.
func (lp *LinkageLoop) readKappa() {
	var at []r3.Vec
	for _, k := range lp.sideA {
		at = append(at, lp.featureAt(k))
	}
	at = append(at, lp.closure.Center)
	for _, k := range slices.Backward(lp.sideB) {
		at = append(at, lp.featureAt(k))
	}
	perimeter := 1.0
	for n := range at {
		perimeter += proofbound.RatSqrtUp(lp.planeSq(at[n], at[(n+1)%len(at)]))
	}
	lp.reach = 4 * perimeter
	lp.kappa = make(map[*Link]*big.Rat)
	for _, k := range lp.links {
		if !lp.anchored(k) {
			continue
		}
		kappa := lp.scaleFor(slideDir(k))
		for k.parent != lp.common && !isSlide(k.parent) && (lp.anchorMeets(k, kappa, false) || lp.anchorMeets(k, kappa, true)) {
			kappa.Mul(kappa, big.NewRat(2, 1))
		}
		lp.kappa[k] = kappa
	}
}

// scaleFor is the least power of two λ with λ·|dir| at least the loop's
// reach.
func (lp *LinkageLoop) scaleFor(dir motionbound.RatVec) *big.Rat {
	_, exp := math.Frexp(lp.reach / proofbound.RatSqrtDown(linkagebound.Dot(dir, dir)))
	return new(big.Rat).SetFloat64(math.Ldexp(1, exp))
}

// anchorMeets reports whether slide k's anchor at κ meets its revolute
// parent's pin in the plane.
func (lp *LinkageLoop) anchorMeets(k *Link, kappa *big.Rat, ahead bool) bool {
	return lp.planeSqRat(ratVecExact(k.parent.pin()), lp.anchorAtKappa(k, kappa, ahead)).Sign() == 0
}

// anchorAt is anchored slide k's anchor A = P₀ − κ·Dir, or P₀ + κ·Dir ahead
// of its rider (agent-docs/linkage-check-design.md §15.2), exactly.
func (lp *LinkageLoop) anchorAt(k *Link, ahead bool) motionbound.RatVec {
	return lp.anchorAtKappa(k, lp.kappa[k], ahead)
}

func (lp *LinkageLoop) anchorAtKappa(k *Link, kappa *big.Rat, ahead bool) motionbound.RatVec {
	p0, dir := ratVecExact(lp.riderAt(k)), slideDir(k)
	var out motionbound.RatVec
	for i := range 3 {
		step := new(big.Rat).Mul(kappa, dir[i])
		if ahead {
			out[i] = step.Add(p0[i], step)
			continue
		}
		out[i] = step.Sub(p0[i], step)
	}
	return out
}

// anchorLength encloses κ·|Dir|, anchored slide k's reading from its anchor
// at the zero pose, between the two floats around its exact root.
func (lp *LinkageLoop) anchorLength(k *Link) proofbound.RatInterval {
	sq := lp.planeSqRat(lp.anchorAt(k, false), ratVecExact(lp.riderAt(k)))
	return proofbound.IntervalOwned(proofarith.FloatRat(proofbound.RatSqrtDown(sq)), proofarith.FloatRat(proofbound.RatSqrtUp(sq)))
}

// readBars reads every bar of the loop: Common's, then each revolute loop
// link's in Links() order. A slide has no bar, nor has a link one of whose
// loop pins is a slide's rail.
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
	if !lp.commonRail() {
		pa, pb := lp.commonPins()
		common, err := bar(lp.common, pa, pb)
		if err != nil {
			return nil, err
		}
		out = append(out, common)
	}
	for _, k := range lp.links {
		if isSlide(k) || lp.besideRail(k) {
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
// is read from (agent-docs/linkage-check-design.md §15.2-§15.3, §15.8). The drive is
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
	subs      linkagebound.DriverSegments
	// held reports that every sub-segment holds the driver: the loop's links
	// then stand at one placement for the whole drive.
	held bool

	mu sync.Mutex
	// scenes holds one scene per side of the plane: [0] where the driver's
	// scene value is q, [1] where it is −q; nil for a side no sub-segment
	// needs.
	scenes [2]*loopchain.Scene
	chain  *loopchain.Chain
	reach  []*big.Rat // per dependent: a proven bound on |value| over the certified drive
	hulls  []proofbound.RatInterval
	// certified lists the stretches [a, b] of the fraction the decomposition
	// asked whole and sketch certified; a joint-box cell whose loop-axis range
	// meets none of them is stuck (agent-docs/linkage-check-design.md §16.4).
	certified [][2]*big.Rat
}

type loopSpan = linkagebound.Span

// unbuildableError marks a pose whose loop could not be enclosed
// (agent-docs/linkage-check-design.md §15.6): the pose is not evaluated and the
// intervals that end at it are undecided. loop is the loop that refused.
type unbuildableError struct {
	cause error
	loop  *loopDrive
}

func (e *unbuildableError) Error() string { return e.cause.Error() }

func (e *unbuildableError) Unwrap() error { return e.cause }

func (e *unbuildableError) unbuildable() {}
