package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"
	"sync"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/linkagebound/loopchain"
	"github.com/lestrrat-3d/decad/internal/motionbound"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// This file is the closed loop of docs/linkage-check-design.md §15: Close and
// Loop, the private sketch scene a driven loop is read from, the canonical
// chain of certified enclosures every dependent joint value comes from, the
// Schedule a renderer calls per frame, and the asks a joint box over a
// loop reads per cell (§16). decad computes no 2D answer here:
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
	// slide is the loop's primary slide: the first prismatic link in Links()
	// order whose parent is Common, nil when there is none. Its rail is the
	// scene's u axis. Every other prismatic loop link is anchored
	// (docs/linkage-check-design.md §15.2), and kappa holds each one's κ.
	slide *Link
	kappa map[*Link]*big.Rat
	reach float64 // four times the loop's perimeter at the zero pose
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
// pose), so the tree gains one loop (docs/linkage-check-design.md §15.1).
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

// coordinateAxis reads an exactly coordinate-aligned axis and its sense.
func coordinateAxis(v r3.Vec) (int, int, bool) { return linkagebound.CoordinateAxis(v) }

// unitAxis is sense·e_axis.
func unitAxis(axis, sense int) r3.Vec { return linkagebound.UnitAxis(axis, sense) }

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
// whose rail is held by an anchor (docs/linkage-check-design.md §15.2).
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
// picks κ for every anchored slide (docs/linkage-check-design.md §15.2): the
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

// ratStep is p + λ·dir, exactly.
func ratStep(p, dir motionbound.RatVec, lambda *big.Rat) motionbound.RatVec {
	var out motionbound.RatVec
	for i := range 3 {
		out[i] = new(big.Rat).Mul(lambda, dir[i])
		out[i].Add(out[i], p[i])
	}
	return out
}

// anchorMeets reports whether slide k's anchor at κ meets its revolute
// parent's pin in the plane.
func (lp *LinkageLoop) anchorMeets(k *Link, kappa *big.Rat, ahead bool) bool {
	return lp.planeSqRat(ratVecExact(k.parent.pin()), lp.anchorAtKappa(k, kappa, ahead)).Sign() == 0
}

// anchorAt is anchored slide k's anchor A = P₀ − κ·Dir, or P₀ + κ·Dir ahead
// of its rider (docs/linkage-check-design.md §15.2), exactly.
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
	subs      linkagebound.DriverSegments
	// held reports that every sub-segment holds the driver: the loop's links
	// then stand at one placement for the whole drive.
	held bool

	mu sync.Mutex
	// scenes holds one scene per side of the plane: [0] where the driver's
	// scene value is q, [1] where it is −q; nil for a side no sub-segment
	// needs.
	scenes [2]*loopScene
	chain  *loopchain.Chain
	reach  []*big.Rat // per dependent: a proven bound on |value| over the certified drive
	hulls  []proofbound.RatInterval
	// certified lists the stretches [a, b] of the fraction the decomposition
	// asked whole and sketch certified; a joint-box cell whose loop-axis range
	// meets none of them is stuck (docs/linkage-check-design.md §16.4).
	certified [][2]*big.Rat
}

type loopSub = linkagebound.DriverSubsegment

// loopScene is the private sketch scene of one loop under one drive, on one
// side of the plane (docs/linkage-check-design.md §15.2).
type loopScene struct {
	plane      loopPlane
	sk         *sketch.Sketch
	driver     sketch.Dimension
	driven     []sketch.Dimension // per dependent: an angle, the primary slide's horizontal distance, or an anchored slide's distance
	angular    []bool             // per dependent: its reading is an angle, read modulo a turn
	anchored   []bool             // per dependent: an anchored slide, its reading a distance from its anchor
	signs      []int              // per dependent: its axis's sense against the scene's normal, the primary slide's against u, or +1
	opts       []sketch.EncloseOption
	pins       []scenePin
	e0         loopchain.Ask
	e0Readings []sketch.Interval // per dependent: its reading at the zero pose
	// offset is the driver's scene reading at the zero pose: exactly 0 for a
	// revolute driver whose parent is Common or the primary slide, the
	// enclosure of r₀ a probe scene read for a revolute driver below Common,
	// and κ·|Dir| for an anchored slide (docs/linkage-check-design.md §15.2).
	// The driver's target is offset + |q|.
	offset proofbound.RatInterval
	// side is the side of the plane the scene reads (1 where the driver's
	// value is −|q|), driverLink the driver's position in Linkage.Links(), and
	// slideDriver reports a prismatic driver, whose value is a length. A
	// proven fold is stated in the driver's own terms through them.
	side        int
	driverLink  int
	slideDriver bool
}

// scenePin is one loop pin's sketch point, its world position, and the
// enclosure of its exact plane position in the document: one exact value per
// coordinate on a coordinate-axis loop.
type scenePin struct {
	p    *sketch.Point
	at   r3.Vec
	u, v proofbound.RatInterval
}

// boundScene passes the private sketch handles to the enclosure-chain proof.
func (sc *loopScene) boundScene() loopchain.Scene {
	return loopchain.Scene{
		Sketch: sc.sk, Driver: sc.driver, Driven: sc.driven,
		Angular: sc.angular, Anchored: sc.anchored, Signs: sc.signs,
		Options: sc.opts, ZeroReadings: sc.e0Readings, Offset: sc.offset,
		Side: sc.side, DriverLink: sc.driverLink, SlideDriver: sc.slideDriver,
	}
}

type loopSpan = linkagebound.Span

// unbuildableError marks a pose whose loop could not be enclosed
// (docs/linkage-check-design.md §15.6): the pose is not evaluated and the
// intervals that end at it are undecided. loop is the loop that refused.
type unbuildableError struct {
	cause error
	loop  *loopDrive
}

func (e *unbuildableError) Error() string { return e.cause.Error() }
func (e *unbuildableError) Unwrap() error { return e.cause }
func (e *unbuildableError) unbuildable()  {}

// resolveLoops applies the loop rows of docs/linkage-check-design.md §15.6 to
// a resolved drive, marks every dependent joint, and cuts each driven loop's
// drive into sub-segments. A loop the drive does not list, or whose listed
// joint holds 0, stands at the zero pose and is left as the tree's held
// joints. The scenes are built later, by prepare. A joint box is read the
// same way, its loop driver's range the one-segment drive Min → Max (§16.2);
// noun names what states the values, "drive" or "box", in a refusal.
func (l *Linkage) resolveLoops(spec *linkageSpec, noun string) error {
	for _, lp := range l.loops {
		var listed []int
		for _, k := range lp.links {
			if spec.joints[k.index].listed {
				listed = append(listed, k.index)
			}
		}
		if len(listed) > 1 {
			return fmt.Errorf(`%w: a %s states links %d and %d of one loop, but a loop's second value follows from its first`, ErrDegenerate, noun, listed[0], listed[1])
		}
		if len(listed) == 0 {
			continue
		}
		k := listed[0]
		jt := spec.joints[k]
		if heldAtZeroJoint(jt) {
			continue
		}
		ld := &loopDrive{loop: lp, driver: k, driverJt: jt}
		if jt.revolute {
			ld.axisSense = linkagebound.Dot(ratVecExact(jt.axis), lp.normal).Sign()
		}
		subs, err := linkagebound.DriverSubsegments(jt.points, jt.values, jt.link.index)
		if err != nil {
			return err
		}
		ld.subs = linkagebound.DriverSegments(subs)
		ld.chain = loopchain.NewChain(ld.subs, linkageReadingFloor, ld.sceneValue)
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

// sceneSide is the frame flip side reads on: for a revolute driver the side
// whose normal makes the driver's scene value |q|, the mirror image when its
// axis turns against that; for the primary slide the half-turned side for
// q ≤ 0. An anchored slide driver keeps the loop's own frame on both sides.
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

// sceneFrame reads the loop's held slide and constructs its exact plane.
func (lp *LinkageLoop) sceneFrame(mirror, halfTurn bool) (loopPlane, error) {
	var slideDir *r3.Vec
	if lp.slide != nil {
		j, _ := lp.slide.joint.(PrismaticJoint)
		slideDir = &j.Dir
	}
	pl, err := linkagebound.SceneFrame(lp.normal, lp.coord, slideDir, mirror, halfTurn)
	if err != nil {
		return loopPlane{}, err
	}
	return loopPlane{frame: pl.Frame, u: pl.U, v: pl.V, uLen: pl.ULen, vLen: pl.VLen}, nil
}

// coords encloses p's exact coordinates in the loop plane.
func (pl loopPlane) coords(p r3.Vec) (proofbound.RatInterval, proofbound.RatInterval) {
	return pl.coordsRat(ratVecExact(p))
}

// coordsRat is coords of an exact point.
func (pl loopPlane) coordsRat(pr motionbound.RatVec) (proofbound.RatInterval, proofbound.RatInterval) {
	return linkagebound.PlaneCoordinates(pr, pl.u, pl.v, pl.uLen, pl.vLen)
}

// ratNeg is −w.
func ratNeg(w motionbound.RatVec) motionbound.RatVec { return linkagebound.NegVec(w) }

// ratVecFloat is the float vector nearest w.
func ratVecFloat(w motionbound.RatVec) r3.Vec { return linkagebound.VecFloat(w) }

// floatBox is the outward-rounded float interval around iv.
func floatBox(iv proofbound.RatInterval) sketch.Interval {
	return sketch.Interval{Lo: proofbound.RatFloatDown(iv.Lo), Hi: proofbound.RatFloatUp(iv.Hi)}
}

// buildScene builds the loop's private scene for the drive on one side
// (docs/linkage-check-design.md §15.2): one point per loop pin at its exact
// plane position, Common's pins fixed; a line and a distance per revolute
// loop link between its two pins, the distance's target the interval around
// the exact length; for the primary slide, a fixed rail along u through its
// rider's zero-pose position, the rider held on it; for every other slide,
// an anchored rail held rigid on its parent by exact distances, its rider on
// it; the driver's angle from a fixed reference line along its zero-pose
// line, from its parent's line offset by r₀ below Common, the primary slide's
// horizontal distance from its zero-pose point, or an anchored slide's
// distance from its anchor; and a driven reading per dependent: an angle
// from its parent's line to its own, the primary slide's horizontal
// distance, or an anchored slide's distance from its anchor.
func (ld *loopDrive) buildScene(ctx context.Context, spec *linkageSpec, side int) (*loopScene, error) {
	lp := ld.loop
	driverLink := spec.joints[ld.driver].link
	mirror, halfTurn := ld.sceneSide(side, driverLink == lp.slide)
	zero := proofbound.PointInterval(new(big.Rat))
	ahead := false
	switch {
	case lp.anchored(driverLink):
		// One frame on both sides; the anchor moves ahead for q ≤ 0.
		mirror, halfTurn, ahead = false, false, side == 1
		zero = lp.anchorLength(driverLink)
	case driverLink != lp.slide && driverLink.parent != lp.common:
		var err error
		if zero, err = ld.probeOffset(ctx, spec, mirror, halfTurn); err != nil {
			return nil, err
		}
	}
	sc, err := ld.buildSceneOn(ctx, spec, sceneFlip{mirror: mirror, halfTurn: halfTurn, ahead: ahead}, zero)
	if err != nil {
		return nil, err
	}
	sc.side, sc.driverLink, sc.slideDriver = side, driverLink.index, !spec.joints[ld.driver].revolute
	return sc, nil
}

// sceneFlip is how one side's scene differs from the loop's own: the
// frame's mirror image or half turn, and an anchored slide driver's anchor
// ahead of its rider.
type sceneFlip struct {
	mirror, halfTurn, ahead bool
}

// probeOffset reads r₀, the zero-pose angle a revolute driver below Common
// is measured by on the scene flipped by mirror and halfTurn
// (docs/linkage-check-design.md §15.2): the same scene driven by a loop joint
// whose parent is Common, at 0, with that angle driven, answers it as its E0
// reading after its own falsifier has run.
func (ld *loopDrive) probeOffset(ctx context.Context, spec *linkageSpec, mirror, halfTurn bool) (proofbound.RatInterval, error) {
	lp := ld.loop
	probe := &loopDrive{loop: lp, driver: -1}
	for _, k := range lp.links {
		if probe.driver < 0 && k.parent == lp.common {
			probe.driver = k.index
			continue
		}
		probe.deps = append(probe.deps, k.index)
	}
	j := slices.Index(probe.deps, ld.driver)
	if probe.driver < 0 || j < 0 {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: a loop has no joint on its common link to read its driver's reference by`, ErrUnsupported)
	}
	zero := proofbound.PointInterval(new(big.Rat))
	if lp.anchored(spec.joints[probe.driver].link) {
		// Unreachable: Common's first loop child, if it slides, is the
		// primary slide. An anchored probe would read from its anchor.
		zero = lp.anchorLength(spec.joints[probe.driver].link)
	}
	sc, err := probe.buildSceneOn(ctx, spec, sceneFlip{mirror: mirror, halfTurn: halfTurn}, zero)
	if err != nil {
		return proofbound.RatInterval{}, err
	}
	if err := sc.askZero(ctx); err != nil {
		return proofbound.RatInterval{}, err
	}
	r := sc.e0Readings[j]
	return proofbound.IntervalOwned(proofarith.FloatRat(r.Lo), proofarith.FloatRat(r.Hi)), nil
}

// buildSceneOn is buildScene on the side flip describes, the driver's
// zero-pose reading offset.
func (ld *loopDrive) buildSceneOn(ctx context.Context, spec *linkageSpec, flip sceneFlip, offset proofbound.RatInterval) (*loopScene, error) {
	lp := ld.loop
	plane, err := lp.sceneFrame(flip.mirror, flip.halfTurn)
	if err != nil {
		return nil, fmt.Errorf(`%w: a loop's plane frame: %w`, ErrNotFinite, err)
	}
	w := sketch.NewWorld()
	sk, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, fmt.Errorf(`%w: a loop's scene: %w`, ErrUnsupported, err)
	}
	sc := &loopScene{plane: plane, sk: sk, offset: offset}
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
	// newPin is a free point of the scene at the exact zero-pose position at
	// that no other feature shares — an anchored rail's points, a rider
	// between two slides — listed for the falsifier like every pin.
	newPin := func(at motionbound.RatVec) *sketch.Point {
		f := ratVecFloat(at)
		local := plane.frame.ToLocal(f)
		x, y := plane.coordsRat(at)
		pin := scenePin{p: sk.CreatePoint(local.X, local.Y), at: f, u: x, v: y}
		sc.pins = append(sc.pins, pin)
		return pin.p
	}
	fixedAt := func(at motionbound.RatVec) *sketch.Point {
		local := plane.frame.ToLocal(ratVecFloat(at))
		x, y := plane.coordsRat(at)
		p := sk.CreatePoint(local.X, local.Y)
		fix(p, x, y)
		return p
	}
	lines := make(map[*Link]*sketch.Line)
	var cons []sketch.Constraint
	// hold keeps p and q at the exact plane distance between pa and pb, stated
	// like a bar: its target the interval between the floats around the root.
	hold := func(p, q *sketch.Point, pa, pb motionbound.RatVec) error {
		sq := lp.planeSqRat(pa, pb)
		down, up := proofbound.RatSqrtDown(sq), proofbound.RatSqrtUp(sq)
		if proofbound.IsNonFinite(up) || !(down > 0) {
			return fmt.Errorf(`%w: a loop rail's distance is not representable`, ErrNotFinite)
		}
		dist := sketch.NewDistance(p, q, up)
		cons = append(cons, dist)
		sc.opts = append(sc.opts, sketch.WithTargetRange(dist, down, up))
		return nil
	}
	riders := make(map[*Link]*sketch.Point)
	rider := func(k *Link) *sketch.Point {
		if p, ok := riders[k]; ok {
			return p
		}
		p := point(lp.riderAt(k))
		if lp.besideRail(k) {
			p = newPin(ratVecExact(lp.riderAt(k)))
		}
		riders[k] = p
		return p
	}
	var commonLine *sketch.Line
	var railStart, slider *sketch.Point
	if lp.slide == nil {
		pa, pb := lp.commonPins()
		commonLine = sk.CreateLine(point(pa), point(pb))
		fixPin(pa)
		fixPin(pb)
	} else {
		// The primary slide's rail is the fixed line through its rider's
		// zero-pose position along u; the rider rides it. The rail is the
		// line Common's other pin, and the slide's children, measure their
		// angles from.
		pin := lp.riderAt(lp.slide)
		slider = rider(lp.slide)
		railStart = fixed(pin, 0)
		commonLine = sk.CreateLine(railStart, fixed(pin, 1))
		lines[lp.slide] = commonLine
		cons = append(cons, sketch.NewPointOnLine(slider, commonLine))
		other := lp.sideA
		if len(lp.sideA) > 0 && lp.sideA[0] == lp.slide {
			other = lp.sideB
		}
		switch {
		case len(other) == 0:
			fixPin(lp.closure.Center)
		case !isSlide(other[0]):
			fixPin(other[0].pin())
		}
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
	driverLink := spec.joints[ld.driver].link
	anchors := make(map[*Link]*sketch.Point)
	lineAt := make(map[*Link][2]motionbound.RatVec) // a rail-held link's line ends at the zero pose
	for _, k := range lp.links {
		if !lp.anchored(k) {
			continue
		}
		p0 := ratVecExact(lp.riderAt(k))
		at := lp.anchorAt(k, flip.ahead && k == driverLink)
		var a *sketch.Point
		var rail *sketch.Line
		par := k.parent
		switch {
		case par == lp.common:
			a = fixedAt(at)
			rail = sk.CreateLine(a, fixedAt(p0))
		case isSlide(par):
			// The parent's rider X lies on this rail at P₀; a second point Z
			// of the parent on its own rail, at the anchor's scale, fixes the
			// parent's turn.
			x := rider(par)
			zAt := ratStep(p0, slideDir(par), lp.scaleFor(slideDir(par)))
			z := newPin(zAt)
			a = newPin(at)
			cons = append(cons, sketch.NewPointOnLine(z, lines[par]))
			if err := errors.Join(hold(x, z, p0, zAt), hold(x, a, p0, at), hold(z, a, zAt, at)); err != nil {
				return nil, err
			}
			rail = sk.CreateLine(a, x)
		default:
			// The parent's frame is its pin O and a point C off the rail at
			// the anchor's scale. The rail's two points, A and B = 2P₀ − A
			// on the rider's other side, are each held to both, by
			// triangles that stay well shaped wherever O lies.
			o, oAt := point(par.pin()), ratVecExact(par.pin())
			perp := linkagebound.Cross(lp.normal, slideDir(k))
			cAt := ratStep(oAt, perp, lp.scaleFor(perp))
			var bAt motionbound.RatVec
			for i := range 3 {
				bAt[i] = new(big.Rat).Sub(new(big.Rat).Add(p0[i], p0[i]), at[i])
			}
			c := newPin(cAt)
			a = newPin(at)
			b := newPin(bAt)
			if err := errors.Join(hold(o, c, oAt, cAt), hold(o, a, oAt, at), hold(c, a, cAt, at), hold(o, b, oAt, bAt), hold(c, b, cAt, bAt)); err != nil {
				return nil, err
			}
			rail = sk.CreateLine(a, b)
			lines[par], lineAt[par] = rail, [2]motionbound.RatVec{at, bAt}
		}
		anchors[k] = a
		lines[k] = rail
		cons = append(cons, sketch.NewPointOnLine(rider(k), rail))
	}
	parentLine := func(k *Link) *sketch.Line {
		if k.parent == lp.common {
			return commonLine
		}
		return lines[k.parent]
	}
	n := lp.normal
	if flip.mirror {
		n = ratNeg(n)
	}
	senseOf := func(k int) int {
		if spec.joints[k].revolute {
			return linkagebound.Dot(ratVecExact(spec.joints[k].axis), n).Sign()
		}
		if lp.anchored(spec.joints[k].link) {
			return 1
		}
		return linkagebound.Dot(ratVecExact(spec.joints[k].axis), plane.u).Sign()
	}
	mid, _ := linkagebound.Midpoint(offset).Float64()
	switch {
	case driverLink == lp.slide:
		sc.driver = sketch.NewHorizontalDistance(railStart, slider, 0)
	case lp.anchored(driverLink):
		sc.driver = sketch.NewDistance(anchors[driverLink], rider(driverLink), mid)
	case driverLink.parent == lp.common && lp.besideRail(driverLink):
		ends := lineAt[driverLink]
		refLine := sk.CreateLine(fixedAt(ends[0]), fixedAt(ends[1]))
		sc.driver = sketch.NewAngle(refLine, lines[driverLink], 0)
	case driverLink.parent == lp.common:
		next := lp.linkNext(driverLink)
		refLine := sk.CreateLine(point(driverLink.pin()), fixed(next, 0))
		sc.driver = sketch.NewAngle(refLine, lines[driverLink], 0)
	default:
		// Below Common the driver turns from its parent's line, offset r₀.
		sc.driver = sketch.NewAngle(parentLine(driverLink), lines[driverLink], mid)
	}
	cons = append(cons, sc.driver)
	for _, d := range ld.deps {
		link := spec.joints[d].link
		var dim sketch.Dimension
		switch {
		case link == lp.slide:
			h := sketch.NewHorizontalDistance(railStart, slider, 0)
			h.SetDriven(true)
			dim = h
		case lp.anchored(link):
			r, _ := linkagebound.Midpoint(lp.anchorLength(link)).Float64()
			dist := sketch.NewDistance(anchors[link], rider(link), r)
			dist.SetDriven(true)
			dim = dist
		default:
			a := sketch.NewAngle(parentLine(link), lines[link], 0)
			a.SetDriven(true)
			dim = a
		}
		sc.driven = append(sc.driven, dim)
		sc.angular = append(sc.angular, !isSlide(link))
		sc.anchored = append(sc.anchored, lp.anchored(link))
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
		for side, used := range ld.subs.Sides() {
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
			ld.chain.SetScene(side, sc.boundScene(), sc.e0)
		}
	}
	return nil
}

// askZero asks E0 and runs the zero-pose falsifier on it.
func (sc *loopScene) askZero(ctx context.Context) error {
	bound := sc.boundScene()
	for _, pin := range sc.pins {
		bound.Pins = append(bound.Pins, loopchain.Pin{Point: pin.p, At: pin.at, U: pin.u, V: pin.v})
	}
	ask, readings, err := loopchain.Zero(ctx, bound, ErrUnsupported)
	if err != nil {
		return err
	}
	sc.e0Readings = readings
	sc.e0 = ask
	return nil
}

// sceneValue is the driver's scene value at the exact fraction s, as the two
// floats around it: |q(s)| in radians or millimetres, each sub-segment read on
// the side where it is never negative (docs/linkage-check-design.md §15.3).
func (ld *loopDrive) sceneValue(side int, s *big.Rat) (float64, float64) {
	q := jointParam(ld.driverJt, s)
	lo, hi := motionbound.ParamLower(q), motionbound.ParamUpper(q)
	if hi.Sign() <= 0 {
		lo, hi = hi.Neg(hi), lo.Neg(lo)
	}
	if lo.Sign() < 0 {
		lo = new(big.Rat)
	}
	off := ld.scenes[side].offset
	return proofbound.RatFloatDown(lo.Add(lo, off.Lo)), proofbound.RatFloatUp(hi.Add(hi, off.Hi))
}

// decompose prepares the internal chain and checks the dependent limits.
func (ld *loopDrive) decompose(ctx context.Context, spec *linkageSpec, floor *big.Rat) error {
	ld.mu.Lock()
	defer ld.mu.Unlock()
	result, err := ld.chain.Decompose(ctx, len(ld.deps), floor)
	if err != nil {
		return err
	}
	ld.certified = append(ld.certified, result.Certified...)
	ld.hulls, ld.reach = result.Hulls, result.Reach
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
			ld.hulls[j].Lo.Cmp(motionbound.ParamUpper(lo)) >= 0 && ld.hulls[j].Hi.Cmp(motionbound.ParamLower(hi)) <= 0
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

// pointValues is every dependent's value range at the exact fraction s, read
// on the sub-segment subAt picks, or the refusal that makes the pose
// unbuildable.
func (ld *loopDrive) pointValues(ctx context.Context, s *big.Rat) ([][2]motionbound.MotionParam, error) {
	ld.mu.Lock()
	defer ld.mu.Unlock()
	return ld.chain.PointValues(ctx, s, len(ld.deps), ld.reach,
		func(err error) error { return ld.unbuildable(err) })
}

// describe names the loop in a refusal.
func (ld *loopDrive) describe(err error) error {
	return fmt.Errorf(`the loop closing links %d and %d: %w`, ld.loop.a.index, ld.loop.b.index, err)
}

// unbuildable is sketch's refusal err, the loop named, as the error that
// leaves a pose unbuildable or an interval or cell undecided.
func (ld *loopDrive) unbuildable(err error) *unbuildableError {
	return &unbuildableError{cause: ld.describe(err), loop: ld}
}

// intervalSpans reads the internal chain and records its dependent spans.
func (ld *loopDrive) intervalSpans(ctx context.Context, a, b *big.Rat) ([][]loopSpan, error) {
	ld.mu.Lock()
	defer ld.mu.Unlock()
	return ld.chain.IntervalSpans(ctx, a, b, len(ld.deps), ld.reach,
		func(err error) error { return ld.unbuildable(err) })
}

// span is a dependent joint's |Δq| over [sa, sb]: the sum of dependentSpan
// over the pieces intervalGate read the interval in, which bounds the joint's
// total variation since that is additive over the cuts; nil when there are no
// readings.
func (ld *loopDrive) span(joint int, sa, sb *big.Rat) *big.Rat {
	j := slices.Index(ld.deps, joint)
	if j < 0 {
		return nil
	}
	ld.mu.Lock()
	defer ld.mu.Unlock()
	return ld.chain.Span(j, sa, sb)
}

// dependentHull is a dependent joint's hull over [sa, sb], from the readings
// intervalSpans took for the interval: the hull of every piece's hull, as
// rational radians or millimetres. ok is false when there are no readings.
func (ld *loopDrive) dependentHull(joint int, sa, sb *big.Rat) (proofbound.RatInterval, bool) {
	j := slices.Index(ld.deps, joint)
	if j < 0 {
		return proofbound.RatInterval{}, false
	}
	ld.mu.Lock()
	defer ld.mu.Unlock()
	return ld.chain.DependentHull(j, sa, sb)
}

// dependentAt is a dependent joint's enclosure at the exact fraction s, the
// one the pose there was built from, as rational radians or millimetres.
func (ld *loopDrive) dependentAt(ctx context.Context, joint int, s *big.Rat) (proofbound.RatInterval, bool) {
	j := slices.Index(ld.deps, joint)
	if j < 0 {
		return proofbound.RatInterval{}, false
	}
	values, err := ld.pointValues(ctx, s)
	if err != nil {
		return proofbound.RatInterval{}, false
	}
	return linkagebound.ValueInterval(values[j][0], values[j][1]), true
}

// cellHulls is every dependent's hull over the stretch [a, b] of the loop
// axis (docs/linkage-check-design.md §16.2): the hull, over every piece
// intervalSpans cuts the stretch into, of the values at the piece's ends and
// over its cell; err is the refusal that leaves a joint-box cell gated.
func (ld *loopDrive) cellHulls(ctx context.Context, a, b *big.Rat) ([]proofbound.RatInterval, error) {
	pieces, err := ld.intervalSpans(ctx, a, b)
	if err != nil {
		return nil, err
	}
	out := make([]proofbound.RatInterval, len(ld.deps))
	for j := range ld.deps {
		out[j] = linkagebound.HullOfPieces(pieces, j)
	}
	return out, nil
}

// meetsCertified reports whether the stretch [a, b] of the loop axis
// overlaps, over a positive length, some stretch the decomposition certified.
// A cell whose range meets none holds no buildable centre a split could place,
// since every such centre was refused at the floor already
// (docs/linkage-check-design.md §16.4).
func (ld *loopDrive) meetsCertified(a, b *big.Rat) bool {
	for _, c := range ld.certified {
		if a.Cmp(c[1]) < 0 && c[0].Cmp(b) < 0 {
			return true
		}
	}
	return false
}

// loopPose is every link's pose at one configuration of a looped linkage
// (docs/linkage-check-design.md §15.4): its joint value — stated, or the
// float midpoint label of a dependent's enclosure — and that value's proven
// half-width, its world pose, its ideal pose, and per joint the exact value
// range the ideal pose was composed over.
type loopPose struct {
	values, bounds []units.Value
	poses          []r3.Transform
	ideals         []motionbound.IdealPose
	lo, hi         []motionbound.MotionParam
}

// loopPosesAt builds every link's pose at the exact fraction f for a drive
// that moves a loop: loopPosesOf with every joint at f.
func (s *linkageSpec) loopPosesAt(ctx context.Context, frames []motionbound.MotionFrame, f *big.Rat) ([]units.Value, []units.Value, []r3.Transform, []motionbound.IdealPose, error) {
	fracs := make([]*big.Rat, len(s.joints))
	for k := range fracs {
		fracs[k] = f
	}
	lp, err := s.loopPosesOf(ctx, frames, fracs)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	return lp.values, lp.bounds, lp.poses, lp.ideals, nil
}

// loopPosesOf builds every link's pose with each stated joint k at the exact
// fraction fracs[k] of its own schedule — one fraction for every joint on a
// drive, each joint's own on a joint box — and each driven loop's dependents
// read at its driver's fraction (docs/linkage-check-design.md §15.4, §16.3):
// each stated joint at its label, each dependent at the float midpoint of its
// enclosure, both posed by posesOf; every link's ideal pose composed over the
// dependent's whole enclosure by MotionFrame.AtRange; and each value's proven
// half-width.
func (s *linkageSpec) loopPosesOf(ctx context.Context, frames []motionbound.MotionFrame, fracs []*big.Rat) (loopPose, error) {
	n := len(s.joints)
	values, bounds := make([]units.Value, n), make([]units.Value, n)
	lo, hi := make([]motionbound.MotionParam, n), make([]motionbound.MotionParam, n)
	for k, jt := range s.joints {
		values[k] = jt.label(fracs[k])
		bounds[k] = units.New(0, values[k].Unit())
		p := jointParam(jt, fracs[k])
		lo[k], hi[k] = p, p
	}
	for _, ld := range s.loops {
		ranges, err := ld.pointValues(ctx, fracs[ld.driver])
		if err != nil {
			return loopPose{}, err
		}
		for j, d := range ld.deps {
			lo[d], hi[d] = ranges[j][0], ranges[j][1]
			iv := linkagebound.ValueInterval(lo[d], hi[d])
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
		return loopPose{}, err
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
	return loopPose{values: values, bounds: bounds, poses: poses, ideals: ideals, lo: lo, hi: hi}, nil
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

// Linkage returns the linkage the schedule was built on. LinkagePose.Values
// and Poses from PoseAt follow its Links() order.
func (s *Schedule) Linkage() *Linkage {
	return s.linkage
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
	if err := motionbound.MotionValueValid(at, units.Dimensionless, "the pose fraction"); err != nil {
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
