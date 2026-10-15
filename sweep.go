package decad

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file routes a zero-twist line or circular-arc path through the existing
// analytic prism and revolve evaluators. Composite paths are admitted when
// their transported frames and separation certificates close. The distinct
// sweepPayload preserves the operation's downstream staging and replays every
// reduction for placement. internal/twistsweep owns the bounded nonzero-twist case.
//
// WithSurfaceResult() (docs/surface-design.md §4, docs/sweep-design.md §14
// increment 3) reaches every one of Sweep's three build paths, but not through
// one shared mechanism: a one-span straight or arc path sets the flag on its
// reduced prismPayload/revolvePayload and gets the whole sheet behaviour for
// free from internal/prismrecord/build.go and internal/revolveplan/build.go, while a composite path's spans
// stay solid (sweepSpanPayload never carries the flag) and
// compositesweep.Assemble (internal/compositesweep/assemble.go) consumes sweepPayload's own
// flag to omit the two outer caps after every span has built solid.
//
// An open Sweep mints two section caps. The exact two-arc closed circle of
// §3.3 reduces to a full Revolve. The §§3.4–3.5 circular-section loops sew
// analytic spans cyclically. Neither closed build mints section caps.

const compositeFitSweepWorkLimit = 8 * freeform.FreeformWorkLimit

// SweepOption configures Sweep.
type SweepOption interface {
	option.Interface
	sweepOption()
}

// WithSweepTwist rotates the profile about the path tangent as it travels.
// A nonzero angle requires an origin XY sketch and a strictly convex whole-line
// profile centred on the axis. Section 17 admits a positive-Z straight path
// and a bounded line followed by a cardinal XZ quarter arc. Other cases
// return ErrUnsupported without changing the document.
func WithSweepTwist(angle units.Value) SweepOption {
	return sweepOptionValue{featureoption.WithSweepTwist(angle)}
}

// Sweep moves the section sec along path, registers the resulting body, and
// returns it. sec is a closed [*sketch.Profile], which sweeps to a solid
// (sheet under [WithSurfaceResult]), or an open [*sketch.Chain], which sweeps
// to a sheet. Both are recorded from s; the type argument S is inferred.
//
// A profile section's path must start in the profile plane and its initial
// tangent must be exactly codirectional with the plane's positive normal.
// Composite paths also require tangent joins, exactly representable transported
// frames, and a certified absence of unintended span contact. Section 17 of
// the Sweep design states the bounded nonzero-twist reach. Sections 3.3–3.5
// admit bounded closed-path classes.
//
// A chain section is gated exactly as a chain [Extrude] gates it
// (docs/surface-design.md §13.3); the seam's own sentinel wins before any path
// geometry is read. The path must start in the sketch plane with its initial
// tangent codirectional with that plane's positive normal (ErrDegenerate). The
// evaluator sweeps one straight span, one arc, or §15.8's cardinal line/arc pair;
// other composite paths are ErrUnsupported (Table SC). The result
// is always a sheet, one wall per recorded segment per span with no cap or
// closing face. A chain section therefore returns ErrUnsupported for
// [WithSurfaceResult] (there is no solid to toggle), [WithSweepTwist],
// [WithMitredJoins] and [WithSectionScale]. That refusal follows the chain's
// seam gate and precedes every path-geometry gate (docs/sweep-design.md §15.3).
//
// A nil context, document, sketch, section or path returns ErrDegenerate.
// Every failure and cancellation
// leaves the document unchanged.
func Sweep[S Section](ctx context.Context, d *Document, s *sketch.Sketch, sec S, path *Path,
	opts ...SweepOption) (*Body, error) {
	switch v := any(sec).(type) {
	case *sketch.Profile:
		return sweepProfile(ctx, d, s, v, pathNode(path), opts)
	case *sketch.Chain:
		return sweepChain(ctx, d, s, v, pathNode(path), opts)
	default:
		return nil, fmt.Errorf(`%w: Sweep section %T is not a profile or chain`, ErrDegenerate, sec)
	}
}

// sweepProfile is the profile-fed Sweep body (docs/sweep-design.md §3-§17).
func sweepProfile(ctx context.Context, d *Document, s *sketch.Sketch, p *sketch.Profile, path *sweepinput.Path,
	opts []SweepOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a sweep`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if s == nil || p == nil || path == nil {
		return nil, fmt.Errorf(`%w: Sweep requires a non-nil sketch, profile, and path`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	cfg, err := decodeSweepOptions(opts, len(path.Segments))
	if err != nil {
		return nil, err
	}
	surfaceResult := cfg.SurfaceResult

	profile, plane, profileArea, err := sectioncert.RecordProfile(s, p)
	if err != nil {
		return nil, err
	}
	work := freeform.NewFreeformWork()
	if len(path.Records) > 1 && !cfg.Mitred && !cfg.Scaled {
		work = newCompositeSweepWork(profile)
	}
	recordArea, err := falsifyRecordedArea(profile, profileArea, work)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(path.Records) == 0 || len(path.Segments) == 0 {
		return nil, fmt.Errorf(`%w: a sweep path must contain at least one span`, ErrDegenerate)
	}

	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, ErrDegenerate, err)
	}
	if cfg.Twisted {
		if sweepinput.SamePathPoint(path.Start, path.End) {
			return nil, fmt.Errorf(`%w: a closed sweep path cannot carry nonzero twist`, ErrUnsupported)
		}
		if err := sweepinput.ValidateSweepPathGeometry(path, plane); err != nil {
			return nil, err
		}
		body, err := bodyResult(twistsweep.Eval(ctx, envOf(documentNode(d), d.nextProducerID()), profile, plane,
			path, cfg.Twist, recordArea, r3.Identity()))
		if err != nil {
			return nil, err
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		d.commit(body)
		return body, nil
	}
	if cfg.Mitred || cfg.Scaled {
		// docs/sweep-design.md §16: either option selects the mitred
		// polyline builder, whose Table SM gates replace S6 and S10.
		body, err := sweepMitred(ctx, d, profile, plane, path, cfg)
		if err != nil {
			return nil, err
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		d.commit(body)
		return body, nil
	}
	if err := sweepinput.ValidateSweepPathGeometry(path, plane); err != nil {
		return nil, err
	}
	if err := sweepinput.ValidateAnalyticProfile(profile); err != nil {
		return nil, err
	}

	ref := d.nextProducerID()
	if sweepinput.SamePathPoint(path.Start, path.End) {
		if surfaceResult {
			return nil, fmt.Errorf(`%w: a closed sweep surface result is not implemented`, ErrUnsupported)
		}
		var body *Body
		if _, circle := sweepinput.ClosedCircle(path.Records); circle {
			body, err = evalClosedCircleSweep(ctx, d, ref, profile, plane, frame, path, work)
		} else {
			body, err = evalClosedCyclicSweep(ctx, d, ref, profile, plane, frame, path, work)
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
	segments := pathSegments(path)

	var body *Body
	if len(segments) > 1 {
		body, err = evalCompositeSweepContext(ctx, d, ref, profile, plane, frame, path, work, surfaceResult)
	} else {
		switch segment := segments[0].(type) {
		case LineTo:
			height, heightBound, lineErr := sweepinput.ValidateStraightSweepPath(path, frame)
			if lineErr != nil {
				return nil, lineErr
			}
			// The flag rides the REDUCED prismPayload, not just the finishing
			// sweepPayload literal below: internal/prismrecord/build.go's prismrecord.Build reads
			// pp.surfaceResult to decide Kind()/solid, cap omission, topobuild.SheetLumps and
			// the area subtraction, which is what gives the line reduction the
			// whole sheet behaviour for free (docs/surface-design.md §4).
			prism := prismrecord.Payload{
				Profile:       profile,
				Frame:         frame,
				Z1:            height,
				Z1Delta:       heightBound,
				Xform:         r3.Identity(),
				SurfaceResult: surfaceResult,
			}
			body, err = bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), prism, work))
			if err == nil {
				finishStraightSweepBody(body, sweepPayload{Prism: prism, Path: path, SurfaceResult: surfaceResult})
			}
		case ArcThrough:
			body, err = evalArcSweepContext(ctx, d, ref, profile, plane, frame, path, path.Records[0], work, surfaceResult)
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

// sweepPayload deliberately remains distinct from prismPayload and
// revolvePayload. Downstream consumers whose Sweep proof has not landed must
// dispatch on this payload and refuse instead of treating the analytic
// reduction as proof for the original operation.
//
// SurfaceResult is WithSurfaceResult's own flag, held here too rather than
// read only off prism/revolve: every finishing step below overwrites the
// built body's payload with this struct, so a flag that rode only the reduced
// prism/revolve payload would still reach Kind()/solid correctly on first
// build (prismrecord.Build/revolveplan.Build read it there) but would give
// a later consumer — auditSheetBoundary's own sweepPayload case
// (verify.go) — no uniform field to read across all three build paths. For
// the composite path it is the ONLY place the flag lives at all: every span
// stays solid (sweepSpanPayload carries no such field), and
// compositesweep.Assemble reads this field alone to decide which two caps
// to omit from the published face set.
type sweepPayload struct {
	Prism           prismPayload
	Revolve         revolvePayload
	Arc             bool
	ClosedCircle    bool
	ClosedComposite bool
	ReverseArcCaps  bool
	Spans           []sweepSpanPayload
	Path            *sweepinput.Path
	SurfaceResult   bool
}

// straightSweepPrismReduction returns the exact prism built for one solid,
// zero-twist, one-line profile Sweep. Modifiers still apply their own edge or
// face selection gates before using the reduction.
func straightSweepPrismReduction(payload topo.Payload) (prismPayload, bool) {
	sp, ok := payload.(sweepPayload)
	if !ok || sp.SurfaceResult || sp.Arc || sp.ClosedCircle || sp.ClosedComposite ||
		len(sp.Spans) != 0 || sp.Path == nil || len(sp.Path.Records) != 1 || len(sp.Path.Segments) != 1 {
		return prismPayload{}, false
	}
	if sp.Path.Segments[0].Arc {
		return prismPayload{}, false
	}
	return sp.Prism, true
}

func (sp sweepPayload) Transform() r3.Transform {
	if len(sp.Spans) != 0 {
		return sp.Spans[0].transform()
	}
	if sp.Arc {
		return sp.Revolve.Xform
	}
	return sp.Prism.Xform
}

func (sp sweepPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	d, ref := documentHandle(env.Doc), env.Ref
	if len(sp.Spans) != 0 {
		sp.Spans = append([]sweepSpanPayload(nil), sp.Spans...)
		sp.Prism.Xform = composed
		for i := range sp.Spans {
			sp.Spans[i].Prism.Xform = composed
			sp.Spans[i].Revolve.Xform = composed
		}
		return placedNode(replayCompositeSweep(ctx, d, ref, sp))
	}
	if sp.Arc {
		sp.Revolve.Xform = composed
		sp.Prism.Xform = composed
		body, err := bodyResult(revolveplan.Build(ctx, envOf(documentNode(d), ref), sp.Revolve, freeform.NewFreeformWork()))
		if err != nil {
			return nil, err
		}
		finishArcSweepBody(body, sp)
		return bodyNode(body), nil
	}
	sp.Prism.Xform = composed
	body, err := bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), sp.Prism, freeform.NewFreeformWork()))
	if err != nil {
		return nil, err
	}
	finishStraightSweepBody(body, sp)
	return bodyNode(body), nil
}

// This section is the chain-fed form of Sweep, docs/sweep-design.md §15: the open sketch
// chain swept along a Path. §15.1 states why the composite join's pairing rule
// survives an open walk — it pairs by recorded-segment index and reads no
// winding — and §15.2 states why a ONE-SPAN path needs no such rule at all: it
// has no join, so its whole build is the recorded walk transported over that
// one span. A one-span straight path reduces to the chain prism that chain
// Extrude already builds, over the path's own height rather than a resolved
// Extent. A one-span arc reduces to a chain Revolve. Composite paths outside
// §15.8's cardinal class stay ErrUnsupported (Table SC row SC7).

// chainSweepPayload is the chain Sweep's own record of a ribbon body: one chain
// prism or Revolve reduction, or two built chain spans, beside its source Path.
// It stays DISTINCT from chainPayload for the reason sweepPayload stays
// distinct from prismPayload — a downstream consumer whose Sweep proof has not
// landed dispatches on this type and refuses, rather than reading the analytic
// reduction as proof for the original operation (docs/sweep-design.md §10).
type chainSweepPayload struct {
	Chain   chainPayload
	Revolve chainRevolvePayload
	Arc     bool
	Path    *sweepinput.Path
	Spans   []chainSweepSpanPayload
}

// Transform is the accumulated rigid placement.
func (sp chainSweepPayload) Transform() r3.Transform {
	if len(sp.Spans) != 0 {
		return sp.Spans[0].transform()
	}
	if sp.Arc {
		return sp.Revolve.Transform()
	}
	return sp.Chain.Transform()
}

// Placed re-evaluates either one-span reduction under the composed motion
// (core §8). The cardinal composite refuses until its transformed join plane
// and mesh stations have their own bound.
func (sp chainSweepPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	d, ref := documentHandle(env.Doc), env.Ref
	if len(sp.Spans) != 0 {
		return nil, fmt.Errorf(`%w: composite chain Sweep placement has no transformed separation proof`, ErrUnsupported)
	}
	if sp.Arc {
		sp.Revolve.Xform = composed
		body, err := bodyResult(revolveplan.BuildChain(ctx, envOf(documentNode(d), ref), sp.Revolve, freeform.NewFreeformWork()))
		if err != nil {
			return nil, err
		}
		finishChainSweepBody(body, sp)
		return bodyNode(body), nil
	}
	sp.Chain.Xform = composed
	body, err := bodyResult(chainrecord.Build(ctx, envOf(documentNode(d), ref), sp.Chain, freeform.NewFreeformWork()))
	if err != nil {
		return nil, err
	}
	finishChainSweepBody(body, sp)
	return bodyNode(body), nil
}

// sweepChain is the chain-fed Sweep body (docs/sweep-design.md §15).
func sweepChain(ctx context.Context, d *Document, s *sketch.Sketch, ch *sketch.Chain, path *sweepinput.Path,
	opts []SweepOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a sweep`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if s == nil || ch == nil || path == nil {
		return nil, fmt.Errorf(`%w: Sweep requires a non-nil sketch, chain, and path`, ErrDegenerate)
	}
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// SC2 before SC3: a seam refusal names a repair the caller makes in the
	// sketch, and reporting a path refusal first would hide it behind geometry
	// the caller cannot act on (docs/sweep-design.md Table SC).
	chain, plane, err := recordChain(s, ch)
	if err != nil {
		return nil, err
	}
	// The option refusal follows the seam gate and precedes every path gate.
	if err := refuseChainSweepOptions(opts, len(path.Segments)); err != nil {
		return nil, err
	}
	if len(path.Records) == 0 || len(path.Segments) == 0 {
		return nil, fmt.Errorf(`%w: a sweep path must contain at least one span`, ErrDegenerate)
	}

	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, ErrDegenerate, err)
	}
	if err := sweepinput.ValidateSweepPathGeometry(path, plane); err != nil {
		return nil, err
	}
	if err := sweepinput.ValidateAnalyticSegments(chain.Segments, "chain"); err != nil {
		return nil, err
	}
	if sweepinput.SamePathPoint(path.Start, path.End) {
		return nil, fmt.Errorf(`%w: closed sweep paths are not implemented`, ErrUnsupported)
	}

	segments := pathSegments(path)
	if len(segments) > 1 {
		return d.sweepCompositeChain(ctx, chain, plane, path)
	}
	if _, ok := segments[0].(ArcThrough); ok {
		return d.sweepArcChain(ctx, chain, plane, frame, path)
	}
	if _, ok := segments[0].(LineTo); !ok {
		return nil, fmt.Errorf(`%w: this chain Sweep path span is not admitted`, ErrUnsupported)
	}
	height, heightBound, err := sweepinput.ValidateStraightSweepPath(path, frame)
	if err != nil {
		return nil, err
	}

	// ONE free-form work counter for the whole call, exactly as chain Extrude
	// opens for its own build (docs/spline-design.md §5.2).
	work := freeform.NewFreeformWork()
	ref := d.nextProducerID()
	// z0 is exactly zero and carries no bound: the recorded walk sits in the
	// sketch plane by construction and the path starts there (S5's own gate
	// above). Only the far level is derived, and z1Delta is where the path's
	// composed length bound lands — the term §15.2 names as the one thing a
	// chain sweep carries that the same walk's chain Extrude reading may not.
	reduction := chainPayload{
		Chains:  []chainRecord{chain},
		Frame:   frame,
		Z0:      0,
		Z1:      height,
		Z1Delta: heightBound,
		Xform:   r3.Identity(),
	}
	body, err := bodyResult(chainrecord.Build(ctx, envOf(documentNode(d), ref), reduction, work))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	finishChainSweepBody(body, chainSweepPayload{Chain: reduction, Path: path})
	d.commit(body)
	return body, nil
}

// refuseChainSweepOptions decodes opts like a profile Sweep and refuses each
// option a chain section cannot honour with ErrUnsupported. A chain sweep
// always returns a sheet, so WithSurfaceResult has no solid to toggle; the
// twist (Table S row S11), mitred-join and section-scale options build solids
// from a profile only.
func refuseChainSweepOptions(opts []SweepOption, segments int) error {
	cfg, err := decodeSweepOptions(opts, segments)
	if err != nil {
		return err
	}
	switch {
	case cfg.SurfaceResult:
		return fmt.Errorf(`%w: a chain sweep always returns a sheet; WithSurfaceResult has no solid to toggle`,
			ErrUnsupported)
	case cfg.Twisted || cfg.Twist.Kind() == units.Angle:
		return fmt.Errorf(`%w: WithSweepTwist is not implemented for a chain sweep (docs/sweep-design.md Table S row S11)`,
			ErrUnsupported)
	case cfg.Mitred || cfg.Scaled:
		return fmt.Errorf(`%w: WithMitredJoins and WithSectionScale build a solid from a profile only`, ErrUnsupported)
	}
	return nil
}

func finishStraightSweepBody(body *Body, payload sweepPayload) {
	if built, ok := body.n.Payload.(prismPayload); ok {
		payload.Prism = built
	}
	prefixSweepSpanZeroRole(body)
	body.n.Payload = payload
}

func finishChainSweepBody(body *Body, payload chainSweepPayload) {
	if built, ok := body.n.Payload.(chainPayload); ok {
		payload.Chain = built
	}
	if built, ok := body.n.Payload.(chainRevolvePayload); ok {
		payload.Revolve = built
	}
	prefixSweepSpanZeroRole(body)
	body.n.Payload = payload
}

// prefixSweepSpanZeroRole rewrites every wall role a one-span reduction minted
// so it carries Table B's own path-span index ahead of the section's loop and
// segment indices: side(i,j) becomes side(0,i,j). The reduction builds through
// the prism or chain-prism evaluator, which knows nothing of a path span, so
// the index is restored here rather than threaded through that builder.
func prefixSweepSpanZeroRole(body *Body) {
	for _, face := range body.Faces() {
		for i, origin := range face.n.Origins {
			if suffix, ok := strings.CutPrefix(origin.Role, "side("); ok {
				origin.Role = "side(0," + suffix
				face.n.Origins[i] = origin
			}
		}
	}
}
