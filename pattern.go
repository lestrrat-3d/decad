package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is PatternCopies (docs/mirror-pattern-design.md §4.3, §6.2):
// repeating a body along a line or about an axis, each instance a new live
// body. Where the receiver is a straight or stacked prism and the motion
// keeps it co-directional, an instance keeps the receiver's frame and
// placement and moves its RECORD in the plane, with the motion's rounding
// charged into its section displacement; every other instance is a
// PlacedCopy under the composed rigid motion.

// PatternSpec is how a pattern lays out its instances: a [LinearPattern] or
// a [CircularPattern]. The set is sealed.
type PatternSpec interface{ patternSpec() }

// LinearPattern places instance i at i·Step along Dir, i = 0..Count-1.
// Instance 0 is the receiver itself. Dir is a direction (dimensionless,
// non-zero, not normalised by the caller); Step is a length magnitude, and
// its sense is Dir's.
type LinearPattern struct {
	Dir   r3.Vec
	Step  units.Value
	Count int
}

// CircularPattern places instance i rotated by i/Count of a turn, right-handed
// about the axis through Center along Axis, i = 0..Count-1. The step angle is
// denoted by Count, never by a float, so a quarter turn is exactly a quarter
// turn. Center need not lie on the receiver's own sweep axis.
type CircularPattern struct {
	Center, Axis r3.Vec
	Count        int
}

// The sealed set.
func (LinearPattern) patternSpec()   {}
func (CircularPattern) patternSpec() {}

// PatternCopies returns Count−1 new live bodies, instances 1..Count−1 of
// spec in order, and leaves the receiver live (docs/mirror-pattern-design.md
// §4.3).
//
// The gates are PlacedCopy's — a nil context is ErrDegenerate, a body this
// evaluator did not build is ErrUnsupported, a retired or foreign receiver is
// refused — plus the spec's own: a nil spec or a Count below 2 is
// ErrDegenerate; a non-finite Dir, Axis or Center is ErrNotFinite and a zero
// Dir or Axis ErrDegenerate; a Step that is not a length is ErrUnitKind, a
// negative one ErrNegativeMagnitude and a zero one ErrDegenerate. Every
// instance is built before any is registered, so a refusal or a canceled
// context leaves the document unchanged.
//
// A straight or stacked prism patterned along a direction exactly
// perpendicular to its sweep, or about an axis bit-identical to its sweep
// normal, keeps one frame: each instance is the receiver's own record moved
// in its plane, so a pattern of integer-millimetre holes stays Exact. Its
// readings carry the motion's rounding as a section displacement.
func (b *Body) PatternCopies(ctx context.Context, spec PatternSpec) ([]*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a pattern`, ErrDegenerate)
	}
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	rp, err := resolvePattern(spec)
	if err != nil {
		return nil, err
	}
	if b.payload == nil {
		return nil, fmt.Errorf(`%w: this evaluator cannot copy a body it did not build`, ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	keeping, err := rp.keepsFrame(proofbound.NewWorkBudget(ctx), b.payload)
	if err != nil {
		return nil, err
	}
	base := d.nextProducerID()
	out := make([]*Body, 0, rp.count-1)
	for i := 1; i < rp.count; i++ {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		ref := base + producerID(i-1)
		var body *Body
		if keeping {
			body, err = rp.frameKeepingInstance(ctx, d, ref, b.payload, i)
		} else {
			body, err = rp.placedInstance(ctx, d, ref, b.payload, i)
		}
		if err != nil {
			return nil, err
		}
		out = append(out, body)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commitMany(out)
	return out, nil
}

// resolvedPattern is a validated spec.
type resolvedPattern struct {
	count    int
	circular bool
	// linear
	dir     r3.Vec
	stepMM  float64
	stepRat *big.Rat
	// circular
	center, axis r3.Vec
}

func finiteVec(v r3.Vec) bool {
	return !math.IsNaN(v.X) && !math.IsNaN(v.Y) && !math.IsNaN(v.Z) &&
		!math.IsInf(v.X, 0) && !math.IsInf(v.Y, 0) && !math.IsInf(v.Z, 0)
}

// resolvePattern runs the spec gates. The variants seal with value
// receivers, so a pointer to either is accepted and a nil one refused.
func resolvePattern(spec PatternSpec) (resolvedPattern, error) {
	errNil := fmt.Errorf(`%w: a nil pattern spec names no pattern`, ErrDegenerate)
	switch s := spec.(type) {
	case *LinearPattern:
		if s == nil {
			return resolvedPattern{}, errNil
		}
		return resolvePattern(*s)
	case *CircularPattern:
		if s == nil {
			return resolvedPattern{}, errNil
		}
		return resolvePattern(*s)
	case LinearPattern:
		if s.Count < 2 {
			return resolvedPattern{}, fmt.Errorf(`%w: a pattern needs a Count of at least 2, got %d (a pattern of one is the receiver)`, ErrDegenerate, s.Count)
		}
		if !finiteVec(s.Dir) {
			return resolvedPattern{}, fmt.Errorf(`%w: the pattern direction %v is not finite`, ErrNotFinite, s.Dir)
		}
		if zeroVec(s.Dir) {
			return resolvedPattern{}, fmt.Errorf(`%w: a zero pattern direction names no line`, ErrDegenerate)
		}
		step, err := magnitudeIn(s.Step, units.Length, units.Millimeter, "the pattern step")
		if err != nil {
			return resolvedPattern{}, err
		}
		if step == 0 {
			return resolvedPattern{}, fmt.Errorf(`%w: a zero pattern step stacks every instance on the receiver`, ErrDegenerate)
		}
		exact := exactConversion(s.Step, units.Millimeter)
		if exact == nil {
			return resolvedPattern{}, fmt.Errorf(`%w: the pattern step is not representable`, ErrNotFinite)
		}
		return resolvedPattern{count: s.Count, dir: s.Dir, stepMM: step, stepRat: exact}, nil
	case CircularPattern:
		if s.Count < 2 {
			return resolvedPattern{}, fmt.Errorf(`%w: a pattern needs a Count of at least 2, got %d (a pattern of one is the receiver)`, ErrDegenerate, s.Count)
		}
		if !finiteVec(s.Center) || !finiteVec(s.Axis) {
			return resolvedPattern{}, fmt.Errorf(`%w: the pattern axis (%v, %v) is not finite`, ErrNotFinite, s.Center, s.Axis)
		}
		if zeroVec(s.Axis) {
			return resolvedPattern{}, fmt.Errorf(`%w: a zero pattern axis names no rotation`, ErrDegenerate)
		}
		return resolvedPattern{count: s.Count, circular: true, center: s.Center, axis: s.Axis}, nil
	default:
		return resolvedPattern{}, errNil
	}
}

// motion is instance i's rigid motion in world space, as a caller's own loop
// would build it: a translation by i·step/|Dir| along Dir, or a rotation by
// i/Count of a turn about the axis.
func (rp resolvedPattern) motion(i int) (r3.Transform, error) {
	if rp.circular {
		return r3.RotationAround(rp.center, rp.axis, units.Degrees(360*float64(i)/float64(rp.count)))
	}
	k := float64(i) * rp.stepMM / rp.dir.Len()
	return r3.Translation(rp.dir.Scale(k))
}

// placedInstance is the PlacedCopy arm: the payload re-evaluated under its
// placement composed with instance i's motion.
func (rp resolvedPattern) placedInstance(ctx context.Context, d *Document, ref producerID, payload featurePayload, i int) (*Body, error) {
	m, err := rp.motion(i)
	if err != nil {
		return nil, fmt.Errorf(`%w: pattern instance %d has no rigid motion: %s`, ErrDegenerate, i, err)
	}
	composed, err := payload.transform().Then(m)
	if err != nil {
		return nil, fmt.Errorf(`decad: composing the placement failed: %w`, err)
	}
	return payload.placed(ctx, d, ref, composed)
}

// patternFrame is the frame and placement a frame-keeping instance shares
// with its receiver, and the regions it moves.
func patternFrameOf(payload featurePayload) (r3.Frame, r3.Transform, []ProfileRecord, bool) {
	switch p := payload.(type) {
	case prismPayload:
		return p.frame, p.xform, []ProfileRecord{p.profile}, true
	case stackedPrismPayload:
		regions := make([]ProfileRecord, 0, len(p.slabs))
		for _, slab := range p.slabs {
			regions = append(regions, slab.regions...)
		}
		return p.frame, p.xform, regions, true
	default:
		return r3.Frame{}, r3.Transform{}, nil, false
	}
}

// keepsFrame decides §4.3's frame-keeping arm: a prism or stacked receiver
// whose every segment is a line, arc or circle with no trimmed arc or circle,
// under a co-directional motion. Co-direction is read on the composed world
// normal xform.ApplyDir(frame.N()), as prism-boolean G3 reads it: a linear
// Dir whose exact rational dot product with that normal is zero, or a
// circular Axis bit-identical to ±that normal.
func (rp resolvedPattern) keepsFrame(budget *proofbound.WorkBudget, payload featurePayload) (bool, error) {
	frame, xform, regions, ok := patternFrameOf(payload)
	if !ok {
		return false, nil
	}
	for _, region := range regions {
		for _, loop := range append([]LoopRecord{region.Outer}, region.Holes...) {
			for _, seg := range loop.Segments {
				switch seg.(type) {
				case LineSeg, ArcSeg, CircleSeg:
				default:
					return false, nil
				}
			}
		}
		trimmed, err := prismProfileHasTrimmedCircularSource(budget, region)
		if err != nil {
			return false, err
		}
		if trimmed {
			return false, nil
		}
	}
	n := xform.ApplyDir(frame.N())
	if rp.circular {
		return rp.axis == n || rp.axis == n.Scale(-1), nil
	}
	return ratVecDot(ratVecOf(rp.dir), ratVecOf(n)).Sign() == 0, nil
}

// ratVec is an exact rational 3-vector.
type ratVec [3]*big.Rat

func ratVecOf(v r3.Vec) ratVec {
	return ratVec{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
}

func ratVecDot(a, b ratVec) *big.Rat {
	return proofbound.RatAdd(proofbound.RatMul(a[0], b[0]), proofbound.RatMul(a[1], b[1]), proofbound.RatMul(a[2], b[2]))
}

// placedAxes are the receiver's frame axes and origin as placed in world
// space, exact rationals of the held floats: B·U, B·V and B·O + t, with B the
// placement's basis and t its translation.
func placedAxes(frame r3.Frame, xform r3.Transform) (u, v, o ratVec) {
	basis := xform.Basis()
	ex, ey, ez := ratVecOf(basis.EX), ratVecOf(basis.EY), ratVecOf(basis.EZ)
	apply := func(w r3.Vec) ratVec {
		r := ratVecOf(w)
		var out ratVec
		for k := range out {
			out[k] = proofbound.RatAdd(proofbound.RatMul(ex[k], r[0]), proofbound.RatMul(ey[k], r[1]), proofbound.RatMul(ez[k], r[2]))
		}
		return out
	}
	u, v = apply(frame.U()), apply(frame.V())
	o = apply(frame.Origin())
	t := ratVecOf(xform.Translation())
	for k := range o {
		o[k] = new(big.Rat).Add(o[k], t[k])
	}
	return u, v, o
}

// pointMotion moves one plane-local point exactly and rounds it once,
// returning the held point beside the distance it can sit from the exact
// image (the two coordinates' errors summed, or the one that rounded).
type pointMotion func(Point2) (Point2, float64, error)

// heldOf rounds an exact coordinate interval to the float nearest its
// midpoint, beside the farthest the true coordinate can sit from it.
func heldOf(iv proofbound.RatInterval) (float64, float64, error) {
	mid := new(big.Rat).Add(iv.Lo, iv.Hi)
	mid.Quo(mid, big.NewRat(2, 1))
	held, _ := mid.Float64()
	if math.IsInf(held, 0) {
		return 0, 0, fmt.Errorf(`%w: a pattern instance coordinate overflows a float`, ErrNotFinite)
	}
	return held, proofbound.IntervalFloatError(iv, held), nil
}

func heldPoint(u, v proofbound.RatInterval) (Point2, float64, error) {
	hu, eu, err := heldOf(u)
	if err != nil {
		return Point2{}, 0, err
	}
	hv, ev, err := heldOf(v)
	if err != nil {
		return Point2{}, 0, err
	}
	p := Point2{U: hu, V: hv}
	switch {
	case eu == 0:
		return p, ev, nil
	case ev == 0:
		return p, eu, nil
	default:
		return p, proofbound.AbsSumUpper(eu, ev), nil
	}
}

// linearMotion is instance i's in-plane offset (§6.2): the world offset
// t = i·step·Dir/|Dir| read on the placed frame axes, (t·U, t·V). step is the
// exact rational the caller's Step denotes in millimetres, and 1/|Dir| is a
// certified enclosure from RatSqrtDown/RatSqrtUp over the exact Dir·Dir, of
// zero width when Dir·Dir is a float's square. Each moved coordinate is an
// exact interval rounded once at its midpoint, so the step's conversion, the
// enclosure's width and the sum's rounding are all inside the one charge.
func (rp resolvedPattern) linearMotion(frame r3.Frame, xform r3.Transform, i int) (pointMotion, error) {
	dir := ratVecOf(rp.dir)
	q := ratVecDot(dir, dir)
	lo, hi := proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q)
	if lo <= 0 || math.IsInf(hi, 0) {
		return nil, fmt.Errorf(`%w: the pattern direction's length has no certified enclosure`, ErrUnsupported)
	}
	scale := new(big.Rat).Mul(big.NewRat(int64(i), 1), rp.stepRat)
	k := proofbound.IntervalScale(proofbound.Interval(
		new(big.Rat).Inv(proofarith.FloatRat(hi)), new(big.Rat).Inv(proofarith.FloatRat(lo))), scale)
	u, v, _ := placedAxes(frame, xform)
	ou := proofbound.IntervalScale(k, ratVecDot(dir, u))
	ov := proofbound.IntervalScale(k, ratVecDot(dir, v))
	return func(p Point2) (Point2, float64, error) {
		return heldPoint(
			proofbound.IntervalAdd(proofbound.PointInterval(proofarith.FloatRat(p.U)), ou),
			proofbound.IntervalAdd(proofbound.PointInterval(proofarith.FloatRat(p.V)), ov))
	}, nil
}

// circularMotion is instance i's in-plane rotation (§6.2) by i/Count of a
// turn about the centre the pattern's axis passes through, read on the placed
// frame axes as an exact rational. The sense is the world rotation's, carried
// into the plane: reversed when Axis is −N, and reversed again when the
// placement is a reflection, which conjugates a rotation into its inverse.
//
// A half turn and a quarter turn are the exact maps (u, v) ↦ (−u, −v) and
// (−v, u) about the centre: the angle denotes exactly those, and no trig
// runs. Every other turn reads cos and sin from TurnSinCosInterval, the
// certified enclosure of sin(2πt) and cos(2πt) for a rational turn t, so each
// rotated coordinate is an exact interval rounded once at its midpoint.
func (rp resolvedPattern) circularMotion(frame r3.Frame, xform r3.Transform, i int) pointMotion {
	u, v, o := placedAxes(frame, xform)
	c := ratVecOf(rp.center)
	rel := ratVec{new(big.Rat).Sub(c[0], o[0]), new(big.Rat).Sub(c[1], o[1]), new(big.Rat).Sub(c[2], o[2])}
	cu, cv := ratVecDot(rel, u), ratVecDot(rel, v)
	sense := int64(1)
	if rp.axis != xform.ApplyDir(frame.N()) {
		sense = -sense
	}
	if xform.IsReflection() {
		sense = -sense
	}
	turn := big.NewRat(sense*int64(i), int64(rp.count))
	// Reduce into [0, 1) to name the exact quarter and half turns.
	whole := new(big.Int).Div(turn.Num(), turn.Denom())
	frac := new(big.Rat).Sub(turn, new(big.Rat).SetInt(whole))
	var cosIv, sinIv proofbound.RatInterval
	switch {
	case frac.Cmp(big.NewRat(1, 4)) == 0:
		cosIv, sinIv = proofbound.PointInterval(new(big.Rat)), proofbound.PointInterval(big.NewRat(1, 1))
	case frac.Cmp(big.NewRat(1, 2)) == 0:
		cosIv, sinIv = proofbound.PointInterval(big.NewRat(-1, 1)), proofbound.PointInterval(new(big.Rat))
	case frac.Cmp(big.NewRat(3, 4)) == 0:
		cosIv, sinIv = proofbound.PointInterval(new(big.Rat)), proofbound.PointInterval(big.NewRat(-1, 1))
	default:
		sinIv, cosIv = proofbound.TurnSinCosInterval(frac)
	}
	return func(p Point2) (Point2, float64, error) {
		du := proofbound.PointInterval(new(big.Rat).Sub(proofarith.FloatRat(p.U), cu))
		dv := proofbound.PointInterval(new(big.Rat).Sub(proofarith.FloatRat(p.V), cv))
		ru := proofbound.IntervalAdd(proofbound.PointInterval(cu),
			proofbound.IntervalSub(proofbound.IntervalMul(cosIv, du), proofbound.IntervalMul(sinIv, dv)))
		rv := proofbound.IntervalAdd(proofbound.PointInterval(cv),
			proofbound.IntervalAdd(proofbound.IntervalMul(sinIv, du), proofbound.IntervalMul(cosIv, dv)))
		return heldPoint(ru, rv)
	}
}

// moveRegion moves every segment of a region by mv, keeping each segment's
// kind, sense and range: a translation or a proper rotation changes neither.
// It returns the moved region beside the largest segment charge: a line's
// larger endpoint charge, a circle's centre charge, and an arc's largest
// point charge tripled, since its centre and radius both move with its three
// points' rounding (the argument offsetSectionDelta states for a recorded
// arc).
func moveRegion(budget *proofbound.WorkBudget, region ProfileRecord, mv pointMotion) (ProfileRecord, float64, error) {
	delta := 0.0
	moveLoop := func(loop LoopRecord) (LoopRecord, error) {
		out := make([]CurveSegment, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := budget.Step(); err != nil {
				return LoopRecord{}, err
			}
			pts := func(in ...Point2) ([]Point2, float64, error) {
				moved := make([]Point2, len(in))
				worst := 0.0
				for k, p := range in {
					m, e, err := mv(p)
					if err != nil {
						return nil, 0, err
					}
					moved[k], worst = m, math.Max(worst, e)
				}
				return moved, worst, nil
			}
			switch s := seg.(type) {
			case LineSeg:
				p, e, err := pts(s.Start, s.End)
				if err != nil {
					return LoopRecord{}, err
				}
				s.Start, s.End = p[0], p[1]
				out[i], delta = s, math.Max(delta, e)
			case ArcSeg:
				p, e, err := pts(s.Center, s.Start, s.End)
				if err != nil {
					return LoopRecord{}, err
				}
				s.Center, s.Start, s.End = p[0], p[1], p[2]
				out[i], delta = s, math.Max(delta, proofbound.ProductUpper(3, e))
			case CircleSeg:
				p, e, err := pts(s.Center)
				if err != nil {
					return LoopRecord{}, err
				}
				s.Center = p[0]
				out[i], delta = s, math.Max(delta, e)
			default:
				return LoopRecord{}, fmt.Errorf(`%w: a %T segment has no exact pattern motion`, ErrUnsupported, seg)
			}
		}
		return LoopRecord{Segments: out}, nil
	}
	outer, err := moveLoop(region.Outer)
	if err != nil {
		return ProfileRecord{}, 0, err
	}
	out := ProfileRecord{Outer: outer}
	for _, hole := range region.Holes {
		moved, err := moveLoop(hole)
		if err != nil {
			return ProfileRecord{}, 0, err
		}
		out.Holes = append(out.Holes, moved)
	}
	return out, delta, nil
}

// withPatternDelta adds an instance's motion charge to the receiver's own
// section displacement; a zero charge keeps it bit for bit.
func withPatternDelta(sectionDelta, delta float64) float64 {
	if delta == 0 {
		return sectionDelta
	}
	return proofbound.AbsSumUpper(sectionDelta, delta)
}

// frameKeepingInstance builds instance i of the frame-keeping arm: the
// receiver's own frame, placement, interval and axial displacements over its
// record moved in the plane, charged δ_pattern into the section displacement.
func (rp resolvedPattern) frameKeepingInstance(ctx context.Context, d *Document, ref producerID, payload featurePayload, i int) (*Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	frame, xform, _, _ := patternFrameOf(payload)
	var mv pointMotion
	if rp.circular {
		mv = rp.circularMotion(frame, xform, i)
	} else {
		var err error
		if mv, err = rp.linearMotion(frame, xform, i); err != nil {
			return nil, err
		}
	}
	switch p := payload.(type) {
	case prismPayload:
		moved, delta, err := moveRegion(budget, p.profile, mv)
		if err != nil {
			return nil, err
		}
		p.profile = moved
		p.sectionDelta = withPatternDelta(p.sectionDelta, delta)
		p.walks = nil
		return evalPrismContext(ctx, d, ref, p, freeform.NewFreeformWork())
	case stackedPrismPayload:
		slabs := make([]prismSlab, len(p.slabs))
		delta := 0.0
		for k, slab := range p.slabs {
			slabs[k] = slab
			slabs[k].regions = make([]ProfileRecord, len(slab.regions))
			for r, region := range slab.regions {
				moved, charge, err := moveRegion(budget, region, mv)
				if err != nil {
					return nil, err
				}
				slabs[k].regions[r] = moved
				delta = math.Max(delta, charge)
			}
		}
		if delta != 0 {
			runs, err := p.outerRuns()
			if err != nil {
				return nil, err
			}
			if len(runs) != 1 {
				// A union-built stack's narrower outer sits inside the wider
				// one by construction, which no audit re-proves. A motion that
				// rounds could move the two outers apart by its own rounding,
				// so only an exact motion keeps the frame; the rest copy.
				return rp.placedInstance(ctx, d, ref, payload, i)
			}
		}
		interfaces, err := stackedInterfaces(ctx, slabs, p.interfaces)
		if err != nil {
			return nil, err
		}
		p.slabs, p.interfaces = slabs, interfaces
		p.sectionDelta = withPatternDelta(p.sectionDelta, delta)
		return evalStackedContext(ctx, d, ref, p)
	default:
		return nil, fmt.Errorf(`%w: a %T receiver has no frame-keeping pattern instance`, ErrUnsupported, payload)
	}
}
