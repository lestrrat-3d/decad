package sweeprecord

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeptransport"
	"github.com/lestrrat-3d/decad/internal/topo"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

const CompositeFitWorkLimit = 8 * freeform.FreeformWorkLimit

// SpanPayload is one certified analytic reduction used by a composite
// Sweep. Its frame already names the section transported to this span's
// start. The common xform on every span is the composite body's accumulated
// rigid placement.
type SpanPayload struct {
	Prism          prismrecord.Payload
	Revolve        revolveplan.Payload
	Arc            bool
	ReverseArcCaps bool
}

// NewCompositeWork gives one composite fitted-spline profile a bounded
// counter shared by all of its analytic span reductions and replay steps.
func NewCompositeWork(profile momentinput.Profile) *freeform.FreeformWork {
	work := freeform.NewFreeformWork()
	if sweepinput.HasFitSplineProfile(profile) {
		work.RaiseLimit(CompositeFitWorkLimit)
	}
	return work
}

func replay(
	ctx context.Context,
	env topo.Env,
	payload Payload,
) (*topo.Body, error) {
	return replayWork(ctx, env, payload, NewCompositeWork(payload.Prism.Profile))
}

func replayWork(
	ctx context.Context,
	env topo.Env,
	payload Payload,
	work *freeform.FreeformWork,
) (*topo.Body, error) {
	if err := compositesweep.Preflight(payload.Prism.Profile, len(payload.Spans)); err != nil {
		return nil, err
	}
	parts := make([]compositesweep.SpanPart, len(payload.Spans))
	for i := range payload.Spans {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		span := &payload.Spans[i]
		body, err := span.Build(ctx, env, work)
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		parts[i], err = compositesweep.SpanPartOf(body)
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
	}
	var body *topo.Body
	var err error
	if payload.ClosedComposite {
		body, err = compositesweep.Assemble(ctx, env, parts, false,
			compositesweep.AuditClosedBodies, compositesweep.SewJoin, true)
	} else {
		body, err = compositesweep.Assemble(ctx, env, parts, payload.SurfaceResult,
			compositesweep.AuditBodies, compositesweep.SewJoin, false)
	}
	if err != nil {
		return nil, err
	}
	if err := compositesweep.AggregateBody(ctx, body, parts, payload.SurfaceResult); err != nil {
		return nil, err
	}
	body.Payload = payload
	return body, nil
}

func BuildComposite(
	ctx context.Context,
	env topo.Env,
	profile momentinput.Profile,
	plane sectionrecord.PlaneRecord,
	frame r3.Frame,
	path *sweepinput.Path,
	work *freeform.FreeformWork,
	surfaceResult bool,
) (*topo.Body, error) {
	return buildCompositeWithClosure(ctx, env, profile, plane, frame, path, work, surfaceResult, false)
}

func buildCompositeWithClosure(
	ctx context.Context,
	env topo.Env,
	profile momentinput.Profile,
	plane sectionrecord.PlaneRecord,
	frame r3.Frame,
	path *sweepinput.Path,
	work *freeform.FreeformWork,
	surfaceResult, closedComposite bool,
) (*topo.Body, error) {
	if err := compositesweep.Preflight(profile, len(path.Records)); err != nil {
		return nil, err
	}
	var frames []sweeptransport.Frame
	var err error
	if closedComposite {
		frames, err = sweeptransport.TransportClosedSweepFrames(ctx, path, plane)
	} else {
		frames, err = sweeptransport.TransportSweepFramesContext(ctx, path, plane)
	}
	if err != nil {
		return nil, err
	}
	for i, transported := range frames {
		if transported.OriginBound != 0 || transported.UBound != 0 || transported.VBound != 0 || transported.NBound != 0 {
			return nil, fmt.Errorf(`%w: transported frame %d carries displacement that analytic composite spans cannot represent`, decaderr.ErrUnsupported, i)
		}
	}

	// Every span below stays solid regardless of surfaceResult (LineSpan/
	// ArcSpan take no such flag): the span pairing below needs both cap
	// roles, and sewing pairs cap loops face to face. The flag is consumed by
	// compositesweep.Assemble alone, which omits only the outer two caps
	// from the published face set after every span has built solid
	// (agent-docs/surface-design.md §4).
	payload := Payload{
		Prism: prismrecord.Payload{
			Profile: profile,
			Frame:   frame,
			Xform:   r3.Identity(),
		},
		Spans:           make([]SpanPayload, len(path.Records)),
		Path:            path,
		SurfaceResult:   surfaceResult,
		ClosedComposite: closedComposite,
	}
	for i, record := range path.Records {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		current := frames[i].Frame
		var err error
		if record.Arc == nil {
			payload.Spans[i], err = LineSpan(profile, current, record)
		} else {
			spanPlane := sectionrecord.PlaneRecord{Origin: current.Origin(), U: current.U(), V: current.V()}
			payload.Spans[i], err = ArcSpan(ctx, profile, spanPlane, current, record, work)
		}
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
	}
	return replayWork(ctx, env, payload, work)
}

func LineSpan(
	profile momentinput.Profile,
	frame r3.Frame,
	record sweepinput.PathRecord,
) (SpanPayload, error) {
	tangent := sweeparc.Sub(sweeparc.VecOf(record.End), sweeparc.VecOf(record.Start))
	normal := sweeparc.VecOf(frame.N())
	if !sweeparc.IsZero(sweeparc.Cross(tangent, normal)) || sweeparc.Dot(tangent, normal).Sign() <= 0 {
		return SpanPayload{}, fmt.Errorf(`%w: a composite line does not follow its transported section normal`, decaderr.ErrUnsupported)
	}

	delta := record.End.Sub(record.Start)
	if !proofbound.FiniteVec(delta) {
		return SpanPayload{}, fmt.Errorf(`%w: the sweep line's displacement is outside the representable range`, decaderr.ErrUnsupported)
	}
	height := delta.Len()
	if math.IsInf(height, 0) || math.IsNaN(height) {
		return SpanPayload{}, fmt.Errorf(`%w: the sweep line's length is outside the representable range`, decaderr.ErrUnsupported)
	}
	heightSquared, heightSquaredOK := proofarith.DySquaredDistance3(
		record.Start.X, record.Start.Y, record.Start.Z,
		record.End.X, record.End.Y, record.End.Z,
	)
	heightBound := capcontour.StraightEdgeBound(height, heightSquared, heightSquaredOK)
	heldSweep := frame.N().Scale(height)
	heightBound = proofbound.AbsSumUpper(
		heightBound,
		proofarith.RationalFloatError(tangent[0], heldSweep.X),
		proofarith.RationalFloatError(tangent[1], heldSweep.Y),
		proofarith.RationalFloatError(tangent[2], heldSweep.Z),
	)
	if math.IsInf(heightBound, 0) || math.IsNaN(heightBound) {
		return SpanPayload{}, fmt.Errorf(`%w: the sweep line's length has no finite error bound`, decaderr.ErrUnsupported)
	}
	return SpanPayload{Prism: prismrecord.Payload{
		Profile: profile,
		Frame:   frame,
		Z1:      height,
		Z1Delta: heightBound,
		Xform:   r3.Identity(),
	}}, nil
}

func ArcSpan(
	ctx context.Context,
	profile momentinput.Profile,
	plane sectionrecord.PlaneRecord,
	frame r3.Frame,
	record sweepinput.PathRecord,
	work *freeform.FreeformWork,
) (SpanPayload, error) {
	geometry, err := sweeparc.Derive(record.Start, record.Arc,
		record.ArcPhi, record.ArcAngle, plane)
	if err != nil {
		return SpanPayload{}, err
	}
	ax, side, err := revolveplan.ResolveAxisSide(ctx, profile, geometry.Line, work)
	if err != nil {
		return SpanPayload{}, err
	}
	phi0, phi1 := 0.0, geometry.Phi
	den := revolveangle.Sweep{Phi0: revolveangle.Zero(), Phi1: geometry.Angle}
	reverseCaps := false
	if side < 0 {
		phi0, phi1 = -phi1, -phi0
		den.Phi0, den.Phi1 = den.Phi1.Neg(), den.Phi0.Neg()
		reverseCaps = true
	}
	return SpanPayload{
		Revolve: revolveplan.Payload{
			Profile: profile,
			Frame:   frame,
			Ax:      ax,
			Phi0:    phi0,
			Phi1:    phi1,
			Den:     den,
			Xform:   r3.Identity(),
		},
		Arc:            true,
		ReverseArcCaps: reverseCaps,
	}, nil
}

func (span SpanPayload) transform() r3.Transform {
	if span.Arc {
		return span.Revolve.Xform
	}
	return span.Prism.Xform
}

// Build evaluates one temporary closed span. compositesweep.Assemble
// removes the internal caps and sews adjacent section boundaries after every
// span has succeeded. No temporary body is committed to the document.
func (span *SpanPayload) Build(
	ctx context.Context,
	env topo.Env,
	work *freeform.FreeformWork,
) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if span.Arc {
		body, err := revolveplan.Build(ctx, env, span.Revolve, work)
		if err != nil {
			return nil, err
		}
		RestoreArcCapRoles(body, span.ReverseArcCaps)
		if built, ok := body.Payload.(revolveplan.Payload); ok {
			span.Revolve = built
		}
		return body, nil
	}
	body, err := prismrecord.Build(ctx, env, span.Prism, work)
	if err != nil {
		return nil, err
	}
	if built, ok := body.Payload.(prismrecord.Payload); ok {
		span.Prism = built
	}
	return body, nil
}

func BuildClosedCyclic(ctx context.Context, env topo.Env,
	profile momentinput.Profile, plane sectionrecord.PlaneRecord, frame r3.Frame, path *sweepinput.Path,
	work *freeform.FreeformWork,
) (*topo.Body, error) {
	if !sweepinput.ClosedRacetrack(profile, plane, path.Start, path.Records) &&
		!sweepinput.ClosedPlanarTangentLoop(profile, plane, path.Start, path.Records) {
		return nil, fmt.Errorf(`%w: the closed sweep path is outside the admitted cyclic classes`, decaderr.ErrUnsupported)
	}
	return buildCompositeWithClosure(ctx, env, profile, plane, frame, path, work, false, true)
}
