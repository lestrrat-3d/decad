package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/linkagebound/loopchain"
	"github.com/lestrrat-3d/decad/internal/loopscene"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

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
func (ld *loopDrive) buildScene(ctx context.Context, spec *linkageSpec, side int) (*loopchain.Scene, error) {
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
	sc.Side, sc.DriverLink, sc.SlideDriver = side, driverLink.index, !spec.joints[ld.driver].revolute
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
	_, readings, err := loopchain.Zero(ctx, *sc, ErrUnsupported)
	if err != nil {
		return proofbound.RatInterval{}, err
	}
	r := readings[j]
	return proofbound.IntervalOwned(proofarith.FloatRat(r.Lo), proofarith.FloatRat(r.Hi)), nil
}

// buildSceneOn is buildScene on the side flip describes, the driver's
// zero-pose reading offset.
func (ld *loopDrive) buildSceneOn(ctx context.Context, spec *linkageSpec, flip sceneFlip, offset proofbound.RatInterval) (*loopchain.Scene, error) {
	lp := ld.loop
	links := make(map[*Link]*loopscene.Link, len(lp.links)+1)
	var linkOf func(*Link) *loopscene.Link
	linkOf = func(k *Link) *loopscene.Link {
		if k == nil {
			return nil
		}
		if existing, ok := links[k]; ok {
			return existing
		}
		out := &loopscene.Link{Index: k.index}
		links[k] = out
		out.Parent = linkOf(k.parent)
		if k == lp.common {
			return out
		}
		out.PinPos, out.NextPos, out.RiderPos = k.pin(), lp.linkNext(k), lp.riderAt(k)
		out.Slide, out.Anchored, out.BesideRail = isSlide(k), lp.anchored(k), lp.besideRail(k)
		if out.Anchored {
			out.Anchor[0], out.Anchor[1] = lp.anchorAt(k, false), lp.anchorAt(k, true)
			out.AnchorLength = lp.anchorLength(k)
		}
		if out.Slide {
			out.Dir = slideDir(k)
		}
		return out
	}
	convert := func(src []*Link) []*loopscene.Link {
		out := make([]*loopscene.Link, len(src))
		for i, k := range src {
			out[i] = linkOf(k)
		}
		return out
	}
	pa, pb := lp.commonPins()
	view := &loopscene.Loop{
		Links: convert(lp.links), SideA: convert(lp.sideA), SideB: convert(lp.sideB),
		Common: linkOf(lp.common), Slide: linkOf(lp.slide),
		CommonPins: [2]r3.Vec{pa, pb}, ClosureCenter: lp.closure.Center,
		Normal: lp.normal, Coord: lp.coord, Reach: lp.reach,
	}
	if lp.slide != nil {
		j, _ := lp.slide.joint.(PrismaticJoint)
		view.PrimaryDir = &j.Dir
	}
	for _, bar := range lp.bars {
		view.Bars = append(view.Bars, loopscene.Bar{
			Link: linkOf(bar.link), Own: bar.own, Next: bar.next, Down: bar.down, Up: bar.up,
		})
	}
	joints := make([]loopscene.Joint, len(spec.joints))
	for _, k := range lp.links {
		jt := spec.joints[k.index]
		joints[k.index] = loopscene.Joint{Link: linkOf(k), Revolute: jt.revolute, Axis: jt.axis}
	}
	built, err := loopscene.Build(ctx, loopscene.Input{
		Loop: view, Joints: joints, Driver: ld.driver, Deps: ld.deps,
		Mirror: flip.mirror, HalfTurn: flip.halfTurn, Ahead: flip.ahead, Offset: offset,
	})
	if err != nil {
		return nil, err
	}
	sc := &loopchain.Scene{
		Sketch: built.Sketch, Driver: built.Driver, Driven: built.Driven,
		Angular: built.Angular, Anchored: built.Anchored, Signs: built.Signs,
		Options: built.Options, Offset: built.Offset,
	}
	for _, pin := range built.Pins {
		sc.Pins = append(sc.Pins, loopchain.Pin{Point: pin.P, At: pin.At, U: pin.U, V: pin.V})
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
			zero, readings, err := loopchain.Zero(ctx, *sc, ErrUnsupported)
			if err != nil {
				return err
			}
			sc.ZeroReadings = readings
			ld.scenes[side] = sc
			ld.chain.SetScene(side, *sc, zero)
		}
	}
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
	off := ld.scenes[side].Offset
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
// on the sub-segment linkagebound.DriverSegments.At picks, or the refusal that makes the pose
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

// span is a dependent joint's |Δq| over [sa, sb]: the sum of linkagebound.SpanUpper
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
