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
//
// WithSurfaceResult() (docs/surface-design.md §4, docs/sweep-design.md §14
// increment 3) reaches every one of Sweep's three build paths, but not through
// one shared mechanism: a one-span straight or arc path sets the flag on its
// reduced prismPayload/revolvePayload and gets the whole sheet behaviour for
// free from prism_build.go/revolve_build.go, while a composite path's spans
// stay solid (sweepSpanPayload never carries the flag) and
// assembleCompositeSweepBody (sweep_composite.go) consumes sweepPayload's own
// flag to omit the two outer caps after every span has built solid.
//
// A Sweep never has a fully closed wall set the way a full Revolve does: a
// closed path is refused outright (S7, samePathPoint below) and ArcThrough's
// own three-point form cannot even STATE a full turn — its End coincides with
// Start, which the same closed-path refusal already catches before an angle is
// derived. So every admitted Sweep — one span or composite — mints exactly two
// section caps, and §4.1's closed-sheet carve-out (a build that mints no
// closing face at all) stays reachable through Revolve alone.

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

// Sweep moves p along path, registers the resulting solid, and returns
// it. The path must start in the profile plane and its initial tangent must be
// exactly codirectional with the plane's positive normal. Composite paths also
// require tangent joins, exactly representable transported frames, and a
// certified absence of unintended span contact. Closed paths and nonzero twist
// remain staged as ErrUnsupported. Every failure and cancellation leaves the
// document unchanged.
func (d *Document) Sweep(ctx context.Context, s *sketch.Sketch, p *sketch.Profile, path *Path, opts ...SweepOption) (*Body, error) {
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

	surfaceResult, err := validateSweepOptions(opts)
	if err != nil {
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
		body, err = evalCompositeSweepContext(ctx, d, ref, profile, plane, frame, path, work, surfaceResult)
	} else {
		switch segment := segments[0].(type) {
		case LineTo:
			height, heightBound, lineErr := validateStraightSweepPath(path, frame)
			if lineErr != nil {
				return nil, lineErr
			}
			// The flag rides the REDUCED prismPayload, not just the finishing
			// sweepPayload literal below: prism_build.go's evalPrismContext reads
			// pp.surfaceResult to decide Kind()/solid, cap omission, sheetLumps and
			// the area subtraction, which is what gives the line reduction the
			// whole sheet behaviour for free (docs/surface-design.md §4).
			prism := prismPayload{
				profile:       profile,
				frame:         frame,
				z1:            height,
				z1Delta:       heightBound,
				xform:         r3.Identity(),
				surfaceResult: surfaceResult,
			}
			body, err = evalPrismContext(ctx, d, ref, prism, work)
			if err == nil {
				finishStraightSweepBody(body, sweepPayload{prism: prism, path: path, surfaceResult: surfaceResult})
			}
		case ArcThrough:
			body, err = evalArcSweepContext(ctx, d, ref, profile, plane, frame, path, path.records[0], work, surfaceResult)
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

// validateSweepOptions resolves opts into the WithSweepTwist gate and the
// WithSurfaceResult flag. A surfaceResultOption is not a sweepOption, so it is
// matched and consumed before the sweepOption assertion below runs — falling
// through to that assertion would wrongly answer ErrDegenerate instead of
// setting the flag (docs/surface-design.md §4). A repeated WithSurfaceResult()
// is idempotent (surface.go's own doc comment), unlike WithSweepTwist's own
// repeat-is-ErrDegenerate rule below.
func validateSweepOptions(opts []SweepOption) (bool, error) {
	haveTwist := false
	surfaceResult := false
	var twist sweepOption
	for _, raw := range opts {
		if raw == nil {
			return false, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		if _, ok := raw.(surfaceResultOption); ok {
			surfaceResult = true
			continue
		}
		o, ok := raw.(sweepOption)
		if !ok {
			return false, fmt.Errorf(`%w: the sweep option is not a decad sweep option (%T)`, ErrDegenerate, raw)
		}
		switch ident := o.Ident().(type) {
		case identSweepTwist:
			if haveTwist {
				return false, fmt.Errorf(`%w: WithSweepTwist was passed more than once`, ErrDegenerate)
			}
			twist = o
			haveTwist = true
		default:
			return false, fmt.Errorf(`%w: unknown sweep option identifier %T`, ErrDegenerate, ident)
		}
	}
	if !haveTwist {
		return surfaceResult, nil
	}
	angle, ok := option.Get[units.Value](twist)
	if !ok {
		return false, fmt.Errorf(`%w: WithSweepTwist carries no angle`, ErrDegenerate)
	}
	if angle.Kind() != units.Angle {
		return false, fmt.Errorf(`%w: sweep twist must be an angle, got %s`, ErrUnitKind, angle.Kind())
	}
	if _, err := angle.In(units.Radian); err != nil {
		return false, fmt.Errorf(`%w: the sweep twist is not representable: %s`, ErrNotFinite, err)
	}
	if angle.Mag() != 0 {
		return false, fmt.Errorf(`%w: nonzero sweep twist is not implemented`, ErrUnsupported)
	}
	return surfaceResult, nil
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
		if err := validateAnalyticSweepSegments(loop.Segments, "profile"); err != nil {
			return err
		}
	}
	return nil
}

// validateAnalyticSweepSegments is Table S row S10 over one recorded walk,
// shared by the profile-fed gate above and SweepChain's own chain gate
// (docs/sweep-design.md Table SC row SC6). kind names the walk in the refusal
// so a caller reading it knows which argument to repair.
func validateAnalyticSweepSegments(segments []CurveSegment, kind string) error {
	for _, raw := range segments {
		segment, err := normalizeSegment(raw)
		if err != nil {
			return err
		}
		switch segment.(type) {
		case LineSeg, CircleSeg, ArcSeg:
		default:
			return fmt.Errorf(`%w: Sweep supports line, circle, and arc %s segments only`, ErrUnsupported, kind)
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
//
// surfaceResult is WithSurfaceResult's own flag, held here too rather than
// read only off prism/revolve: every finishing step below overwrites the
// built body's payload with this struct, so a flag that rode only the reduced
// prism/revolve payload would still reach Kind()/solid correctly on first
// build (evalPrismContext/evalRevolveContextWork read it there) but would give
// a later consumer — auditSheetBoundary's own sweepPayload case
// (verify.go) — no uniform field to read across all three build paths. For
// the composite path it is the ONLY place the flag lives at all: every span
// stays solid (sweepSpanPayload carries no such field), and
// assembleCompositeSweepBody reads this field alone to decide which two caps
// to omit from the published face set.
type sweepPayload struct {
	prism          prismPayload
	revolve        revolvePayload
	arc            bool
	reverseArcCaps bool
	spans          []sweepSpanPayload
	path           *Path
	surfaceResult  bool
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

// This section is SweepChain of docs/sweep-design.md §15: the open sketch
// chain swept along a Path. §15.1 states why the composite join's pairing rule
// survives an open walk — it pairs by recorded-segment index and reads no
// winding — and §15.2 states why a ONE-SPAN path needs no such rule at all: it
// has no join, so its whole build is the recorded walk transported over that
// one span. This increment builds the one-span STRAIGHT case, which reduces to
// the chain prism ExtrudeChain already builds, over the path's own height
// rather than a resolved Extent. The arc reduction and every composite path
// stay ErrUnsupported (Table SC rows SC7 and SC9, docs/surface-design.md R34).

// ChainSweepOption configures SweepChain. It is its own sealed tier rather
// than [SweepOption]: WithSurfaceResult() does not implement it, so the
// compiler refuses that option outright rather than accepting it as a no-op —
// a chain-fed sweep always returns a sheet, so there is no "build a solid
// instead" state for the option to toggle (docs/surface-design.md §13.2,
// docs/sweep-design.md §15.3). WithSweepTwist is not a member either: a
// nonzero twist is Table S row S11 for a profile-fed sweep, and a chain
// inherits that staging rather than a second spelling of it. No option is a
// member of this tier yet; it exists so a later chain-only option has a tier
// to land on.
type ChainSweepOption interface {
	option.Interface
	chainSweepOption()
}

// chainSweepPayload is SweepChain's own record of a ribbon body: the chain
// ribbon payload the reduction built, beside the Path that stated its height.
// It stays DISTINCT from chainPayload for the reason sweepPayload stays
// distinct from prismPayload — a downstream consumer whose Sweep proof has not
// landed dispatches on this type and refuses, rather than reading the analytic
// reduction as proof for the original operation (docs/sweep-design.md §10).
type chainSweepPayload struct {
	chain chainPayload
	path  *Path
}

// transform is the accumulated rigid placement.
func (sp chainSweepPayload) transform() r3.Transform { return sp.chain.transform() }

// placed re-evaluates the same records under the composed motion (core §8),
// through the identical reduction the first build took, and restates the
// path-span role prefix the reduction itself does not mint.
func (sp chainSweepPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	sp.chain.xform = composed
	body, err := evalChainExtrudeContext(ctx, d, ref, sp.chain, newFreeformWork())
	if err != nil {
		return nil, err
	}
	finishChainSweepBody(body, sp)
	return body, nil
}

// SweepChain sweeps the open chain ch of sketch s along path, and registers
// the resulting ribbon body. ch MUST be a chain of s and a current, unaltered
// snapshot, under the identical gates ExtrudeChain runs (RecordChain,
// docs/surface-design.md §13.3); the seam's own sentinel wins before any path
// geometry is read. The path MUST start in the sketch plane with its initial
// tangent codirectional with that plane's positive normal (ErrDegenerate), and
// this evaluator sweeps ONE straight span: a composite path and an arc span
// are ErrUnsupported (docs/sweep-design.md Table SC). The result is always a
// sheet — Kind() == BodySheet — one wall per recorded segment with no cap and
// no closing face, so WithSurfaceResult() does not compile against this call.
// A failed evaluation leaves the document unchanged.
func (d *Document) SweepChain(ctx context.Context, s *sketch.Sketch, ch *sketch.Chain, path *Path, opts ...ChainSweepOption) (*Body, error) {
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a sweep`, ErrDegenerate)
	}
	if s == nil || ch == nil || path == nil {
		return nil, fmt.Errorf(`%w: SweepChain requires a non-nil sketch, chain, and path`, ErrDegenerate)
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
	if err := validateAnalyticSweepSegments(chain.Segments, "chain"); err != nil {
		return nil, err
	}
	if samePathPoint(path.Start(), path.End()) {
		return nil, fmt.Errorf(`%w: closed sweep paths are not implemented`, ErrUnsupported)
	}

	segments := path.Segments()
	if len(segments) > 1 {
		return nil, fmt.Errorf(
			`%w: SweepChain has no composite path join yet: it sweeps one path span and this path has %d (docs/sweep-design.md §15.1)`,
			ErrUnsupported, len(segments))
	}
	if _, ok := segments[0].(LineTo); !ok {
		return nil, fmt.Errorf(
			`%w: SweepChain sweeps a straight path span only; the arc reduction is staged (docs/sweep-design.md §15.6)`,
			ErrUnsupported)
	}
	height, heightBound, err := validateStraightSweepPath(path, frame)
	if err != nil {
		return nil, err
	}

	// ONE free-form work counter for the whole call, exactly as ExtrudeChain
	// opens for its own build (docs/spline-design.md §5.2).
	work := newFreeformWork()
	ref := d.nextProducerID()
	// z0 is exactly zero and carries no bound: the recorded walk sits in the
	// sketch plane by construction and the path starts there (S5's own gate
	// above). Only the far level is derived, and z1Delta is where the path's
	// composed length bound lands — the term §15.2 names as the one thing a
	// chain sweep carries that the same walk's ExtrudeChain reading may not.
	reduction := chainPayload{
		chains:  []ChainRecord{chain},
		frame:   frame,
		z0:      0,
		z1:      height,
		z1Delta: heightBound,
		xform:   r3.Identity(),
	}
	body, err := evalChainExtrudeContext(ctx, d, ref, reduction, work)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	finishChainSweepBody(body, chainSweepPayload{chain: reduction, path: path})
	d.commit(body)
	return body, nil
}

func finishStraightSweepBody(body *Body, payload sweepPayload) {
	if built, ok := body.payload.(prismPayload); ok {
		payload.prism = built
	}
	prefixSweepSpanZeroRole(body)
	body.payload = payload
}

func finishChainSweepBody(body *Body, payload chainSweepPayload) {
	if built, ok := body.payload.(chainPayload); ok {
		payload.chain = built
	}
	prefixSweepSpanZeroRole(body)
	body.payload = payload
}

// prefixSweepSpanZeroRole rewrites every wall role a one-span reduction minted
// so it carries Table B's own path-span index ahead of the section's loop and
// segment indices: side(i,j) becomes side(0,i,j). The reduction builds through
// the prism or chain-prism evaluator, which knows nothing of a path span, so
// the index is restored here rather than threaded through that builder.
func prefixSweepSpanZeroRole(body *Body) {
	for _, face := range body.Faces() {
		for i, origin := range face.origins {
			if suffix, ok := strings.CutPrefix(origin.Role, "side("); ok {
				origin.Role = "side(0," + suffix
				face.origins[i] = origin
			}
		}
	}
}
