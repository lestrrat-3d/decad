package decad

import (
	"context"
	"fmt"
	"math"
	"strings"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file routes a zero-twist line or circular-arc path through the existing
// analytic prism and revolve evaluators. Composite paths are admitted when
// their transported frames and separation certificates close. The distinct
// sweepPayload preserves the operation's downstream staging and replays every
// reduction for placement.

// SweepOption configures Sweep.
type SweepOption interface {
	option.Interface
	sweepOption()
}

type sweepOption struct{ option.Interface }

func (sweepOption) sweepOption() {}

type identSweepTwist struct{}

// WithSweepTwist applies total signed rotation about the transported path
// tangent. The current evaluator accepts only zero twist; a nonzero angle is
// ErrUnsupported and leaves the document unchanged.
func WithSweepTwist(angle units.Value) SweepOption {
	return sweepOption{option.New(identSweepTwist{}, angle)}
}

// Sweep calls [Document.SweepContext] with [context.Background].
func (d *Document) Sweep(s *sketch.Sketch, p *sketch.Profile, path *Path, opts ...SweepOption) (*Body, error) {
	return d.SweepContext(context.Background(), s, p, path, opts...)
}

// SweepContext moves p along path, registers the resulting solid, and returns
// it. The path must start in the profile plane and its initial tangent must be
// exactly codirectional with the plane's positive normal. Composite paths also
// require tangent joins, exactly representable transported frames, and a
// certified absence of unintended span contact. Closed paths and nonzero twist
// remain staged as ErrUnsupported. Every failure and cancellation leaves the
// document unchanged.
func (d *Document) SweepContext(ctx context.Context, s *sketch.Sketch, p *sketch.Profile, path *Path, opts ...SweepOption) (*Body, error) {
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a sweep`, ErrDegenerate)
	}
	if s == nil || p == nil || path == nil {
		return nil, fmt.Errorf(`%w: Sweep requires a non-nil sketch, profile, and path`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	if err := validateSweepOptions(opts); err != nil {
		return nil, err
	}

	profile, plane, profileArea, err := recordProfile(s, p)
	if err != nil {
		return nil, err
	}
	work := newFreeformWork()
	if err := falsifyRecordedArea(profile, profileArea, work); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(path.records) == 0 || len(path.segments) == 0 {
		return nil, fmt.Errorf(`%w: a sweep path must contain at least one span`, ErrDegenerate)
	}

	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, ErrDegenerate, err)
	}
	if err := validateSweepPathGeometry(path, plane); err != nil {
		return nil, err
	}
	if err := validateAnalyticSweepProfile(profile); err != nil {
		return nil, err
	}

	ref := d.nextProducerID()
	if samePathPoint(path.Start(), path.End()) {
		return nil, fmt.Errorf(`%w: closed sweep paths are not implemented`, ErrUnsupported)
	}
	segments := path.Segments()

	var body *Body
	if len(segments) > 1 {
		body, err = evalCompositeSweepContext(ctx, d, ref, profile, plane, frame, path, work)
	} else {
		switch segment := segments[0].(type) {
		case LineTo:
			height, heightBound, lineErr := validateStraightSweepPath(path, frame)
			if lineErr != nil {
				return nil, lineErr
			}
			prism := prismPayload{
				profile: profile,
				frame:   frame,
				z1:      height,
				z1Delta: heightBound,
				xform:   r3.Identity(),
			}
			body, err = evalPrismContext(ctx, d, ref, prism, work)
			if err == nil {
				finishStraightSweepBody(body, sweepPayload{prism: prism, path: path})
			}
		case ArcThrough:
			body, err = evalArcSweepContext(ctx, d, ref, profile, plane, frame, path, path.records[0], work)
		default:
			err = fmt.Errorf(`%w: sweep path span %T is not supported`, ErrUnsupported, segment)
		}
	}
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

func validateSweepOptions(opts []SweepOption) error {
	haveTwist := false
	var twist sweepOption
	for _, raw := range opts {
		if raw == nil {
			return fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		// Checked before the sweepOption assertion below: a surfaceResultOption
		// is not a sweepOption, so falling through to that assertion would
		// answer ErrDegenerate and contradict Table R row R1's ErrUnsupported.
		if err := refuseSurfaceResult(raw, "Sweep"); err != nil {
			return err
		}
		o, ok := raw.(sweepOption)
		if !ok {
			return fmt.Errorf(`%w: the sweep option is not a decad sweep option (%T)`, ErrDegenerate, raw)
		}
		switch ident := o.Ident().(type) {
		case identSweepTwist:
			if haveTwist {
				return fmt.Errorf(`%w: WithSweepTwist was passed more than once`, ErrDegenerate)
			}
			twist = o
			haveTwist = true
		default:
			return fmt.Errorf(`%w: unknown sweep option identifier %T`, ErrDegenerate, ident)
		}
	}
	if !haveTwist {
		return nil
	}
	angle, ok := option.Get[units.Value](twist)
	if !ok {
		return fmt.Errorf(`%w: WithSweepTwist carries no angle`, ErrDegenerate)
	}
	if angle.Kind() != units.Angle {
		return fmt.Errorf(`%w: sweep twist must be an angle, got %s`, ErrUnitKind, angle.Kind())
	}
	if _, err := angle.In(units.Radian); err != nil {
		return fmt.Errorf(`%w: the sweep twist is not representable: %s`, ErrNotFinite, err)
	}
	if angle.Mag() != 0 {
		return fmt.Errorf(`%w: nonzero sweep twist is not implemented`, ErrUnsupported)
	}
	return nil
}

func validateSweepPathGeometry(path *Path, plane PlaneRecord) error {
	start := path.Start()
	normal := sweepRatFromDyadic(dvCross(dyVec(plane.U), dyVec(plane.V)))
	relStart := sweepRatSub(sweepRatVecOf(start), sweepRatVecOf(plane.Origin))
	if sweepRatDot(relStart, normal).Sign() != 0 {
		return fmt.Errorf(`%w: the sweep path must start in the profile plane`, ErrDegenerate)
	}

	var previousOut sweepRatVec
	for i, record := range path.records {
		tangentIn, tangentOut := record.tangentIn, record.tangentOut
		if i == 0 {
			if !sweepRatIsZero(sweepRatCross(tangentIn, normal)) || sweepRatDot(tangentIn, normal).Sign() <= 0 {
				return fmt.Errorf(`%w: the sweep path's initial tangent must follow the profile plane's positive normal`, ErrDegenerate)
			}
		} else if !sweepRatIsZero(sweepRatCross(previousOut, tangentIn)) || sweepRatDot(previousOut, tangentIn).Sign() <= 0 {
			return fmt.Errorf(`%w: sweep path join %d is not tangent`, ErrUnsupported, i)
		}
		previousOut = tangentOut
	}
	return nil
}

func validateStraightSweepPath(path *Path, frame r3.Frame) (float64, float64, error) {
	start := path.Start()
	end := path.End()
	if samePathPoint(start, end) {
		return 0, 0, fmt.Errorf(`%w: closed sweep paths are not implemented`, ErrUnsupported)
	}
	segments := path.Segments()
	if len(segments) != 1 {
		return 0, 0, fmt.Errorf(`%w: this evaluator sweeps one straight path span only`, ErrUnsupported)
	}
	line, ok := segments[0].(LineTo)
	if !ok {
		return 0, 0, fmt.Errorf(`%w: the sweep path span is not straight`, ErrUnsupported)
	}

	tangent := dvSub(dyVec(line.End), dyVec(start))

	delta := line.End.Sub(start)
	if !finiteVec(delta) {
		return 0, 0, fmt.Errorf(`%w: the sweep line's derived displacement is outside the representable range`, ErrUnsupported)
	}
	height := delta.Len()
	if math.IsInf(height, 0) || math.IsNaN(height) {
		return 0, 0, fmt.Errorf(`%w: the sweep line's length is outside the representable range`, ErrUnsupported)
	}
	lengthBound := straightEdgeBound(height, ratSquaredDistance3(
		start.X, start.Y, start.Z,
		line.End.X, line.End.Y, line.End.Z,
	))
	heldSweep := frame.N().Scale(height)
	bound := absSumUpper(
		lengthBound,
		dyadicFloatError(tangent[0], heldSweep.X),
		dyadicFloatError(tangent[1], heldSweep.Y),
		dyadicFloatError(tangent[2], heldSweep.Z),
	)
	if math.IsInf(bound, 0) || math.IsNaN(bound) {
		return 0, 0, fmt.Errorf(`%w: the sweep line's length has no finite error bound`, ErrUnsupported)
	}
	return height, bound, nil
}

func validateAnalyticSweepProfile(profile ProfileRecord) error {
	loops := append([]LoopRecord{profile.Outer}, profile.Holes...)
	for _, loop := range loops {
		for _, raw := range loop.Segments {
			segment, err := normalizeSegment(raw)
			if err != nil {
				return err
			}
			switch segment.(type) {
			case LineSeg, CircleSeg, ArcSeg:
			default:
				return fmt.Errorf(`%w: Sweep supports line, circle, and arc profile segments only`, ErrUnsupported)
			}
		}
	}
	return nil
}

func samePathPoint(a, b r3.Vec) bool {
	return a.X == b.X && a.Y == b.Y && a.Z == b.Z
}

// sweepPayload deliberately remains distinct from prismPayload and
// revolvePayload. Downstream consumers whose Sweep proof has not landed must
// dispatch on this payload and refuse instead of treating the analytic
// reduction as proof for the original operation.
type sweepPayload struct {
	prism          prismPayload
	revolve        revolvePayload
	arc            bool
	reverseArcCaps bool
	spans          []sweepSpanPayload
	path           *Path
}

func (sp sweepPayload) transform() r3.Transform {
	if len(sp.spans) != 0 {
		return sp.spans[0].transform()
	}
	if sp.arc {
		return sp.revolve.xform
	}
	return sp.prism.xform
}

func (sp sweepPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	if len(sp.spans) != 0 {
		sp.spans = append([]sweepSpanPayload(nil), sp.spans...)
		sp.prism.xform = composed
		for i := range sp.spans {
			sp.spans[i].prism.xform = composed
			sp.spans[i].revolve.xform = composed
		}
		return replayCompositeSweep(ctx, d, ref, sp)
	}
	if sp.arc {
		sp.revolve.xform = composed
		sp.prism.xform = composed
		body, err := evalRevolveContext(ctx, d, ref, sp.revolve)
		if err != nil {
			return nil, err
		}
		finishArcSweepBody(body, sp)
		return body, nil
	}
	sp.prism.xform = composed
	body, err := evalPrismContext(ctx, d, ref, sp.prism, newFreeformWork())
	if err != nil {
		return nil, err
	}
	finishStraightSweepBody(body, sp)
	return body, nil
}

func finishStraightSweepBody(body *Body, payload sweepPayload) {
	if built, ok := body.payload.(prismPayload); ok {
		payload.prism = built
	}
	for _, face := range body.Faces() {
		for i, origin := range face.origins {
			if suffix, ok := strings.CutPrefix(origin.Role, "side("); ok {
				origin.Role = "side(0," + suffix
				face.origins[i] = origin
			}
		}
	}
	body.payload = payload
}
