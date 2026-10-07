package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"
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
	// axis and sense name the closure's axis n = sense·e_axis.
	axis, sense int
	bars        []loopBar
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
// pins apart at: Common's bar first, then Links() order.
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
// axis component (ErrNotFinite); the zero axis (ErrDegenerate); an axis not
// exactly parallel to a coordinate axis, a loop revolute whose axis is not
// exactly parallel to it, or a prismatic joint on the loop (ErrUnsupported,
// naming the joint); a loop joint already on another loop (ErrUnsupported);
// and two loop pins of one link coincident in the loop's plane
// (ErrDegenerate).
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
	lp := &LinkageLoop{linkage: l, a: a, b: b, closure: RevoluteJoint{Center: center, Axis: axis}}
	var ok bool
	if lp.axis, lp.sense, ok = coordinateAxis(axis); !ok {
		return nil, fmt.Errorf(`%w: a closure axis must be exactly parallel to a coordinate axis, got %v`, ErrUnsupported, axis)
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
	n := unitAxis(lp.axis, lp.sense)
	for _, k := range lp.links {
		switch j := k.joint.(type) {
		case RevoluteJoint:
			if c := ratCross(ratVecExact(j.Axis), ratVecExact(n)); !ratZero(c) {
				return nil, fmt.Errorf(`%w: link %d's revolute axis %v is not exactly parallel to the closure axis %v`, ErrUnsupported, k.index, j.Axis, axis)
			}
		case PrismaticJoint:
			return nil, fmt.Errorf(`%w: link %d's prismatic joint lies on the loop, and a prismatic loop joint is not checked yet`, ErrUnsupported, k.index)
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
// plane: their difference with the component along the closure axis dropped.
func (lp *LinkageLoop) planeSq(p, q r3.Vec) *big.Rat {
	pd, qd := ratVecExact(p), ratVecExact(q)
	sum := new(big.Rat)
	for i := range 3 {
		if i == lp.axis {
			continue
		}
		d := new(big.Rat).Sub(pd[i], qd[i])
		sum.Add(sum, d.Mul(d, d))
	}
	return sum
}

// readBars reads every bar of the loop: Common's, then each loop link's in
// Links() order. Two pins of one link coincident in the plane are refused.
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
	pa, pb := lp.commonPins()
	common, err := bar(lp.common, pa, pb)
	if err != nil {
		return nil, err
	}
	out := []loopBar{common}
	for _, k := range lp.links {
		side := lp.sideA
		if !slices.Contains(side, k) {
			side = lp.sideB
		}
		n := slices.Index(side, k)
		b, err := bar(k, k.pin(), lp.nextPin(side, n))
		if err != nil {
			return nil, err
		}
		out = append(out, b)
	}
	return out, nil
}

// loopDrive is one loop moved by one drive: its driver and dependents, the
// private scene, and the chain of certified enclosures every dependent value
// is read from (docs/linkage-check-design.md §15.2-§15.3). The asks are
// serialised behind mu, since sketch.Enclose must not run concurrently on one
// sketch, and cached by their place in the canonical chain.
type loopDrive struct {
	loop   *LinkageLoop
	driver int   // the driver's position in Linkage.Links()
	deps   []int // every dependent's position in Linkage.Links(), Loop.Links() order
	// sigma picks the scene: +1 for the closure axis's own sense, −1 for its
	// mirror image, so that the driver's scene value is never negative.
	sigma int
	// near is the drive end whose driver value is nearer 0, where the chain
	// starts; nearZero reports that the driver is exactly 0 there.
	near     *big.Rat
	nearZero bool
	driverJt linkJoint

	mu    sync.Mutex
	scene *loopScene
	asks  map[string]*loopAsk
	reach []*big.Rat // per dependent: a proven bound on |value| over the certified drive
	// spans holds each verification interval's dependent readings, keyed by
	// its two ends, from intervalGate until travel reads them.
	spans map[string][]loopSpan
}

// loopScene is the private sketch scene of one loop under one drive
// (docs/linkage-check-design.md §15.2).
type loopScene struct {
	frame      r3.Frame
	sk         *sketch.Sketch
	driver     *sketch.Angle
	driven     []*sketch.Angle // per dependent
	signs      []int           // per dependent: the sense of its axis against the scene's normal
	opts       []sketch.EncloseOption
	pins       []scenePin
	e0         *loopAsk
	e0Readings []sketch.Interval // per dependent: its reading at the zero pose
}

// scenePin is one loop pin's sketch point and its exact plane position in the
// document.
type scenePin struct {
	p    *sketch.Point
	u, v *big.Rat
}

// loopAsk is one certified enclosure of the chain, or the refusal that took
// its place. turns counts, per dependent, the whole turns its readings are
// shifted by to continue its predecessor's.
type loopAsk struct {
	enc   *sketch.Enclosure
	turns []int64
	err   error
}

// loopSpan is one dependent's readings over a verification interval: its
// value at the two ends and the hull over the interval, as rational radians.
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
// a resolved drive and marks every dependent joint. A loop the drive does not
// list, or whose listed joint holds 0, stands at the zero pose and is left as
// the tree's held joints. The scenes are built later, by prepare.
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
		if len(jt.values) > 2 {
			return fmt.Errorf(`%w: a loop's driver, link %d, passes Via waypoints, which are not checked yet`, ErrUnsupported, k)
		}
		if !jt.moves() {
			return fmt.Errorf(`%w: a loop's driver, link %d, holds at %s, which is not checked yet`, ErrUnsupported, k, jt.values[0])
		}
		from, okF := paramCompare(jt.values[0], units.New(0, jt.values[0].Unit()))
		to, okT := paramCompare(jt.values[1], units.New(0, jt.values[1].Unit()))
		ends, okE := paramCompare(jt.values[0], jt.values[1])
		if !okF || !okT || !okE {
			return fmt.Errorf(`%w: the sign of link %d's sweep cannot be decided`, ErrUnsupported, k)
		}
		if from*to < 0 {
			return fmt.Errorf(`%w: a loop's driver, link %d, crosses 0 inside the drive, which is not checked yet`, ErrUnsupported, k)
		}
		sign := from + to // both ends share a sign; one may be 0
		if sign > 0 {
			sign = 1
		} else {
			sign = -1
		}
		ld := &loopDrive{loop: lp, driver: k, driverJt: jt, asks: make(map[string]*loopAsk), spans: make(map[string][]loopSpan)}
		// The near end has the smaller |q|: the smaller value of a rising
		// positive sweep, the larger of a negative one.
		ld.near = new(big.Rat)
		if ends*sign > 0 {
			ld.near = big.NewRat(1, 1)
		}
		ld.nearZero = (ld.near.Sign() == 0 && from == 0) || (ld.near.Sign() != 0 && to == 0)
		axisSense := ratDot(ratVecExact(jt.axis), ratVecExact(unitAxis(lp.axis, lp.sense))).Sign()
		ld.sigma = sign * axisSense
		for _, link := range lp.links {
			if link.index == k {
				continue
			}
			if spec.joints[link.index].limits != nil {
				return fmt.Errorf(`%w: link %d is a dependent joint of a driven loop and carries limits, which are not checked yet`, ErrUnsupported, link.index)
			}
			ld.deps = append(ld.deps, link.index)
		}
		for _, d := range ld.deps {
			spec.joints[d].dep = ld
		}
		spec.loops = append(spec.loops, ld)
	}
	return nil
}

// sceneFrame is the loop's plane frame for scene sign sigma: two unit
// coordinate axes u, v with u × v the closure axis's sense times sigma.
func (lp *LinkageLoop) sceneFrame(sigma int) (r3.Frame, error) {
	axes := [3]r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	u, v := axes[(lp.axis+1)%3], axes[(lp.axis+2)%3]
	if lp.sense*sigma < 0 {
		v = v.Scale(-1)
	}
	return r3.NewFrame(r3.Vec{}, u, v)
}

// buildScene builds the loop's private scene for the drive
// (docs/linkage-check-design.md §15.2): one point per loop pin at its exact
// plane position, Common's two pins fixed; a line and a distance per loop link
// between its two pins, the distance's target the interval around the exact
// length; the driver's angle from a fixed reference line along its zero-pose
// bar; and a driven angle per dependent from its parent's line to its own.
func (ld *loopDrive) buildScene(ctx context.Context, spec *linkageSpec) error {
	lp := ld.loop
	frame, err := lp.sceneFrame(ld.sigma)
	if err != nil {
		return fmt.Errorf(`%w: a loop's plane frame: %w`, ErrNotFinite, err)
	}
	w := sketch.NewWorld()
	sk, err := w.CreateSketch(w.XY())
	if err != nil {
		return fmt.Errorf(`%w: a loop's scene: %w`, ErrUnsupported, err)
	}
	sc := &loopScene{frame: frame, sk: sk}
	points := make(map[r3.Vec]*sketch.Point)
	point := func(at r3.Vec) *sketch.Point {
		if p, ok := points[at]; ok {
			return p
		}
		local := frame.ToLocal(at)
		p := sk.CreatePoint(local.X, local.Y)
		points[at] = p
		sc.pins = append(sc.pins, scenePin{p: p, u: proofarith.FloatRat(local.X), v: proofarith.FloatRat(local.Y)})
		return p
	}
	pa, pb := lp.commonPins()
	commonLine := sk.CreateLine(point(pa), point(pb))
	sk.Fix(point(pa))
	sk.Fix(point(pb))
	lines := make(map[*Link]*sketch.Line)
	var cons []sketch.Constraint
	for _, bar := range lp.bars[1:] {
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
	n := ratVecExact(unitAxis(lp.axis, lp.sense*ld.sigma))
	senseOf := func(k int) int { return ratDot(ratVecExact(spec.joints[k].axis), n).Sign() }
	driverLink := spec.joints[ld.driver].link
	driverBar := lp.bars[slices.IndexFunc(lp.bars, func(b loopBar) bool { return b.link == driverLink })]
	ref := sk.CreatePoint(frame.ToLocal(driverBar.next).X, frame.ToLocal(driverBar.next).Y)
	sk.Fix(ref)
	refLine := sk.CreateLine(point(driverBar.own), ref)
	sc.driver = sketch.NewAngle(refLine, lines[driverLink], 0)
	cons = append(cons, sc.driver)
	for _, d := range ld.deps {
		link := spec.joints[d].link
		a := sketch.NewAngle(parentLine(link), lines[link], 0)
		a.SetDriven(true)
		sc.driven = append(sc.driven, a)
		sc.signs = append(sc.signs, senseOf(d))
		cons = append(cons, a)
	}
	sk.AddConstraint(cons...)
	if _, err := sk.Solve(ctx); err != nil {
		return fmt.Errorf(`%w: the loop's scene does not solve at the zero pose: %w`, ErrUnsupported, err)
	}
	ld.scene = sc
	return nil
}

// prepare builds each driven loop's scene and asks its zero pose, E0
// (docs/linkage-check-design.md §15.2). E0 refusing is ErrUnsupported wrapping
// sketch's error. The document's pins are exact solutions of the scene's
// equations at the driving value 0 for every target its intervals hold, so a
// pin outside E0's box disproves the enclosure and the loop is refused; a pin
// inside it admits nothing.
func (s *linkageSpec) prepare(ctx context.Context) error {
	for _, ld := range s.loops {
		if ld.scene != nil {
			continue
		}
		if err := ld.buildScene(ctx, s); err != nil {
			return err
		}
		if err := ld.askZero(ctx); err != nil {
			return err
		}
	}
	return nil
}

// askZero asks E0 and runs the zero-pose falsifier on it.
func (ld *loopDrive) askZero(ctx context.Context) error {
	sc := ld.scene
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
			return fmt.Errorf(`%w: the loop's zero-pose enclosure excludes the document's pin at (%v, %v) in the loop's plane, so it does not describe the document's mechanism`,
				ErrUnsupported, pin.u.FloatString(6), pin.v.FloatString(6))
		}
	}
	sc.e0 = &loopAsk{enc: enc, turns: make([]int64, len(sc.driven))}
	for _, d := range sc.driven {
		r, ok := enc.Driven(d)
		if !ok {
			return fmt.Errorf(`%w: the loop's zero-pose enclosure reads no value for a dependent joint`, ErrUnsupported)
		}
		sc.e0Readings = append(sc.e0Readings, r)
	}
	return nil
}

// intervalHolds reports whether the exact x lies in the outward-rounded iv.
func intervalHolds(iv sketch.Interval, x *big.Rat) bool {
	lo, hi := proofarith.FloatRat(iv.Lo), proofarith.FloatRat(iv.Hi)
	return lo != nil && hi != nil && lo.Cmp(x) <= 0 && x.Cmp(hi) <= 0
}

// loopInvariant reports a sketch refusal that is an invariant failure on a
// scene decad built (docs/linkage-check-design.md §15.6).
func loopInvariant(err error) bool {
	return errors.Is(err, sketch.ErrUncertifiedConstraint) || errors.Is(err, sketch.ErrForeignHandle) || errors.Is(err, sketch.ErrNonFiniteGeometry)
}

// sceneValue is the driver's scene value at the exact fraction s, as the two
// floats around it: |q(s)| in radians, the scene chosen so that it is never
// negative (docs/linkage-check-design.md §15.3).
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

// enclose asks sketch for the enclosure of [lo, hi] continued from pred, or
// records the refusal that takes its place; a refusal of pred carries over.
// Only a context error or an invariant failure is returned as an error, and
// neither is cached.
func (ld *loopDrive) enclose(ctx context.Context, key string, lo, hi float64, pred *loopAsk) (*loopAsk, error) {
	if ask, ok := ld.asks[key]; ok {
		return ask, nil
	}
	ask, err := ld.encloseFresh(ctx, lo, hi, pred)
	if err != nil {
		return nil, err
	}
	ld.asks[key] = ask
	return ask, nil
}

func (ld *loopDrive) encloseFresh(ctx context.Context, lo, hi float64, pred *loopAsk) (*loopAsk, error) {
	if pred.err != nil {
		return &loopAsk{err: pred.err}, nil //nolint:nilerr // a predecessor's refusal is this ask's, recorded rather than returned
	}
	if lo > hi {
		return &loopAsk{err: fmt.Errorf(`%w: the driver range [%v, %v] is empty`, sketch.ErrNotCertified, lo, hi)}, nil
	}
	sc := ld.scene
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
	ask := &loopAsk{enc: enc, turns: make([]int64, len(sc.driven))}
	prevPieces, pieces := pred.enc.Pieces(), enc.Pieces()
	last, first := prevPieces[len(prevPieces)-1], pieces[0]
	for j, d := range sc.driven {
		l, okL := last.Driven(d)
		f, okF := first.Driven(d)
		if !okL || !okF {
			return &loopAsk{err: fmt.Errorf(`%w: an enclosure reads no value for a dependent joint`, sketch.ErrNotCertified)}, nil
		}
		m := math.Round(((l.Lo+l.Hi)/2 - (f.Lo+f.Hi)/2) / (2 * math.Pi))
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

// increasing reports that the chain runs toward larger fractions.
func (ld *loopDrive) increasing() bool { return ld.near.Sign() == 0 }

// linkageGridDepth is the deepest dyadic level the canonical chain reaches
// through binary cells; a deeper parameter is reached by one cell from the
// nearest grid point of linkageReadingFloor's depth on its near side.
const linkageGridDepth = 30

// chainStart is where the cell ending at s starts in the canonical chain
// (docs/linkage-check-design.md §15.3): the depth-d cell ending at s for a
// parameter of depth d, or the nearest grid parameter of the reading floor's
// depth on the near side for a parameter on no grid this file walks.
func (ld *loopDrive) chainStart(s *big.Rat) *big.Rat {
	den := s.Denom()
	depth := den.BitLen() - 1
	dyadic := new(big.Int).Lsh(big.NewInt(1), uint(depth)).Cmp(den) == 0
	if dyadic && depth <= linkageGridDepth {
		step := new(big.Rat).SetFrac(big.NewInt(1), den)
		if ld.increasing() {
			return step.Sub(s, step)
		}
		return step.Add(s, step)
	}
	scaled := new(big.Rat).Mul(s, big.NewRat(linkageReadingFloor, 1))
	q := new(big.Int).Div(scaled.Num(), scaled.Denom())
	if !ld.increasing() {
		q.Add(q, big.NewInt(1))
	}
	anchor := new(big.Rat).SetFrac(q, big.NewInt(linkageReadingFloor))
	if anchor.Cmp(s) == 0 {
		// Unreachable: a parameter on the reading floor's grid is dyadic of
		// depth 14 and takes the branch above. The near end keeps the chain
		// finite whatever reaches here.
		return new(big.Rat).Set(ld.near)
	}
	return anchor
}

// point is the point ask at the exact fraction s, continued from its
// canonical predecessor. Callers hold mu.
func (ld *loopDrive) point(ctx context.Context, s *big.Rat) (*loopAsk, error) {
	if s.Cmp(ld.near) == 0 {
		if ld.nearZero {
			return ld.scene.e0, nil
		}
		approach, err := ld.approach(ctx)
		if err != nil {
			return nil, err
		}
		lo, hi := ld.sceneValue(s)
		return ld.enclose(ctx, "p"+s.RatString(), lo, hi, approach)
	}
	key := "p" + s.RatString()
	if ask, ok := ld.asks[key]; ok {
		return ask, nil
	}
	cell, err := ld.cell(ctx, ld.chainStart(s), s)
	if err != nil {
		return nil, err
	}
	lo, hi := ld.sceneValue(s)
	return ld.enclose(ctx, key, lo, hi, cell)
}

// approach is the enclosure from the zero pose to the near end's driver value.
func (ld *loopDrive) approach(ctx context.Context) (*loopAsk, error) {
	lo, _ := ld.sceneValue(ld.near)
	return ld.enclose(ctx, "a", 0, lo, ld.scene.e0)
}

// cell is the cell ask from start to end in chain order, continued from the
// point at start. Callers hold mu.
func (ld *loopDrive) cell(ctx context.Context, start, end *big.Rat) (*loopAsk, error) {
	key := "c" + start.RatString() + "," + end.RatString()
	if ask, ok := ld.asks[key]; ok {
		return ask, nil
	}
	from, err := ld.point(ctx, start)
	if err != nil {
		return nil, err
	}
	_, lo := ld.sceneValue(start)
	hi, _ := ld.sceneValue(end)
	return ld.enclose(ctx, key, lo, hi, from)
}

// chainCell is the cell over [a, b], a < b, in chain order.
func (ld *loopDrive) chainCell(ctx context.Context, a, b *big.Rat) (*loopAsk, error) {
	if ld.increasing() {
		return ld.cell(ctx, a, b)
	}
	return ld.cell(ctx, b, a)
}

// value is dependent j's joint value over an enclosure: its whole hull, the
// reading shifted by the ask's whole turns, less the zero-pose reading, in
// the joint's own sense. The two ends share one turn count.
func (ld *loopDrive) value(ask *loopAsk, j int) (motionbound.MotionParam, motionbound.MotionParam) {
	sc := ld.scene
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

// decompose asks the drive as one cell, and replaces each refused cell by its
// two halves down to floor (docs/linkage-check-design.md §15.7). The hull of
// every certified cell and of the points at its ends bounds each dependent's
// value over the certified drive: that is its reach m_i, and every later
// enclosure is held to it.
func (ld *loopDrive) decompose(ctx context.Context, floor *big.Rat) error {
	ld.mu.Lock()
	defer ld.mu.Unlock()
	var hulls []proofbound.RatInterval
	add := func(ask *loopAsk) {
		if ask.err != nil {
			return
		}
		for j := range ld.deps {
			iv := valueInterval(ld.value(ask, j))
			if hulls == nil || len(hulls) <= j {
				hulls = append(hulls, iv)
				continue
			}
			hulls[j] = hull(hulls[j], iv)
		}
	}
	near, err := ld.point(ctx, ld.near)
	if err != nil {
		return err
	}
	add(near)
	var walk func(a, b *big.Rat) error
	walk = func(a, b *big.Rat) error {
		c, err := ld.chainCell(ctx, a, b)
		if err != nil {
			return err
		}
		if c.err == nil {
			add(c)
			for _, end := range []*big.Rat{a, b} {
				p, err := ld.point(ctx, end)
				if err != nil {
					return err
				}
				add(p)
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
	for j := range ld.deps {
		ld.reach[j] = new(big.Rat)
		if j < len(hulls) {
			ld.reach[j] = magnitude(hulls[j])
		}
	}
	return nil
}

// withinReach reports whether a value interval lies inside dependent j's
// reach; one outside it is refused, so the reach every swept box was grown by
// covers every value a claim is made about.
func (ld *loopDrive) withinReach(j int, iv proofbound.RatInterval) bool {
	return magnitude(iv).Cmp(ld.reach[j]) <= 0
}

// pointValues is every dependent's value range at the exact fraction s, or
// the refusal that makes the pose unbuildable.
func (ld *loopDrive) pointValues(ctx context.Context, s *big.Rat) ([][2]motionbound.MotionParam, error) {
	ld.mu.Lock()
	defer ld.mu.Unlock()
	ask, err := ld.point(ctx, s)
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

// describe names the loop in a refusal.
func (ld *loopDrive) describe(err error) error {
	return fmt.Errorf(`the loop closing links %d and %d: %w`, ld.loop.a.index, ld.loop.b.index, err)
}

// intervalSpans reads every dependent's values over [a, b]: at the two ends
// and the hull over the interval, from the point asks and the cell between
// them; err is the refusal that leaves the interval undecided.
func (ld *loopDrive) intervalSpans(ctx context.Context, a, b *big.Rat) ([]loopSpan, error) {
	ld.mu.Lock()
	defer ld.mu.Unlock()
	pa, err := ld.point(ctx, a)
	if err != nil {
		return nil, err
	}
	pb, err := ld.point(ctx, b)
	if err != nil {
		return nil, err
	}
	c, err := ld.chainCell(ctx, a, b)
	if err != nil {
		return nil, err
	}
	for _, ask := range []*loopAsk{pa, c, pb} {
		if ask.err != nil {
			return nil, &unbuildableError{cause: ld.describe(ask.err)}
		}
	}
	out := make([]loopSpan, len(ld.deps))
	defer func() { ld.spans[loopSpanKey(a, b)] = out }()
	for j := range ld.deps {
		A, B, C := valueInterval(ld.value(pa, j)), valueInterval(ld.value(pb, j)), valueInterval(ld.value(c, j))
		H := hull(A, B, C)
		if !ld.withinReach(j, H) {
			return nil, &unbuildableError{cause: ld.describe(fmt.Errorf(`%w: a dependent value lies outside the certified drive's reach`, sketch.ErrNotCertified))}
		}
		out[j] = loopSpan{a: A, b: B, h: H}
	}
	return out, nil
}

// dependentSpan is |Δq_k| of docs/linkage-check-design.md §15.5 for one
// dependent over an interval: the largest |x − q_a| + |q_b − x| over x in the
// hull H and q_a, q_b in the end enclosures A and B, which sits at an end of
// H: max(a_hi + b_hi − 2·h_lo, 2·h_hi − a_lo − b_lo).
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

// span is dependent joint's |Δq| over [sa, sb] from the readings intervalGate
// took for the interval; nil when there are none.
func (ld *loopDrive) span(joint int, sa, sb *big.Rat) *big.Rat {
	lo, hi := sa, sb
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	ld.mu.Lock()
	spans, ok := ld.spans[loopSpanKey(lo, hi)]
	ld.mu.Unlock()
	j := slices.Index(ld.deps, joint)
	if !ok || j < 0 {
		return nil
	}
	return dependentSpan(spans[j])
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
			values[d] = units.New(label, units.Radian)
			bounds[d] = units.New(proofbound.RatFloatUp(half), units.Radian)
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
		if err := ld.decompose(ctx, floor); err != nil {
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
