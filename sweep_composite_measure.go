package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeptransport"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// sweepSpanPayload is one certified analytic reduction used by a composite
// Sweep. Its frame already names the section transported to this span's
// start. The common xform on every span is the composite body's accumulated
// rigid placement.
type sweepSpanPayload struct {
	Prism          prismPayload
	Revolve        revolvePayload
	Arc            bool
	ReverseArcCaps bool
}

// newCompositeSweepWork gives one composite fitted-spline profile a bounded
// counter shared by all of its analytic span reductions and replay steps.
func newCompositeSweepWork(profile momentinput.Profile) *freeform.FreeformWork {
	work := freeform.NewFreeformWork()
	if sweepinput.HasFitSplineProfile(profile) {
		work.RaiseLimit(compositeFitSweepWorkLimit)
	}
	return work
}

func replayCompositeSweep(
	ctx context.Context,
	d *Document,
	ref producerID,
	payload sweepPayload,
) (*Body, error) {
	return replayCompositeSweepWork(ctx, d, ref, payload, newCompositeSweepWork(payload.Prism.Profile))
}

func replayCompositeSweepWork(
	ctx context.Context,
	d *Document,
	ref producerID,
	payload sweepPayload,
	work *freeform.FreeformWork,
) (*Body, error) {
	if err := compositesweep.Preflight(payload.Prism.Profile, len(payload.Spans)); err != nil {
		return nil, err
	}
	parts := make([]compositesweep.SpanPart, len(payload.Spans))
	for i := range payload.Spans {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		span := &payload.Spans[i]
		body, err := span.Build(ctx, d, ref, work)
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		parts[i], err = compositesweep.SpanPartOf(bodyNode(body))
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
	}
	var body *Body
	var err error
	if payload.ClosedComposite {
		body, err = bodyResult(compositesweep.Assemble(ctx, envOf(documentNode(d), ref), parts, false,
			compositesweep.AuditClosedBodies, compositesweep.SewJoin, true))
	} else {
		body, err = bodyResult(compositesweep.Assemble(ctx, envOf(documentNode(d), ref), parts, payload.SurfaceResult,
			compositesweep.AuditBodies, compositesweep.SewJoin, false))
	}
	if err != nil {
		return nil, err
	}
	if err := compositesweep.AggregateBody(ctx, bodyNode(body), parts, payload.SurfaceResult); err != nil {
		return nil, err
	}
	body.n.Payload = payload
	return body, nil
}

func evalCompositeSweepContext(
	ctx context.Context,
	d *Document,
	ref producerID,
	profile profileRecord,
	plane planeRecord,
	frame r3.Frame,
	path *sweepinput.Path,
	work *freeform.FreeformWork,
	surfaceResult bool,
) (*Body, error) {
	return evalCompositeSweepContextWithClosure(ctx, d, ref, profile, plane, frame, path, work, surfaceResult, false)
}

func evalCompositeSweepContextWithClosure(
	ctx context.Context,
	d *Document,
	ref producerID,
	profile profileRecord,
	plane planeRecord,
	frame r3.Frame,
	path *sweepinput.Path,
	work *freeform.FreeformWork,
	surfaceResult, closedComposite bool,
) (*Body, error) {
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
			return nil, fmt.Errorf(`%w: transported frame %d carries displacement that analytic composite spans cannot represent`, ErrUnsupported, i)
		}
	}

	// Every span below stays solid regardless of surfaceResult (compositeLine/
	// ArcSweepSpan take no such flag): the span pairing below needs both cap
	// roles, and sewing pairs cap loops face to face. The flag is consumed by
	// compositesweep.Assemble alone, which omits only the outer two caps
	// from the published face set after every span has built solid
	// (docs/surface-design.md §4).
	payload := sweepPayload{
		Prism: prismrecord.Payload{
			Profile: profile,
			Frame:   frame,
			Xform:   r3.Identity(),
		},
		Spans:           make([]sweepSpanPayload, len(path.Records)),
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
			payload.Spans[i], err = compositeLineSweepSpan(profile, current, record)
		} else {
			spanPlane := planeRecord{Origin: current.Origin(), U: current.U(), V: current.V()}
			payload.Spans[i], err = compositeArcSweepSpan(ctx, profile, spanPlane, current, record, work)
		}
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
	}
	return replayCompositeSweepWork(ctx, d, ref, payload, work)
}

func compositeLineSweepSpan(
	profile profileRecord,
	frame r3.Frame,
	record sweepinput.PathRecord,
) (sweepSpanPayload, error) {
	tangent := sweepRatSub(sweepRatVecOf(record.End), sweepRatVecOf(record.Start))
	normal := sweepRatVecOf(frame.N())
	if !sweepRatIsZero(sweepRatCross(tangent, normal)) || sweepRatDot(tangent, normal).Sign() <= 0 {
		return sweepSpanPayload{}, fmt.Errorf(`%w: a composite line does not follow its transported section normal`, ErrUnsupported)
	}

	delta := record.End.Sub(record.Start)
	if !proofbound.FiniteVec(delta) {
		return sweepSpanPayload{}, fmt.Errorf(`%w: the sweep line's displacement is outside the representable range`, ErrUnsupported)
	}
	height := delta.Len()
	if math.IsInf(height, 0) || math.IsNaN(height) {
		return sweepSpanPayload{}, fmt.Errorf(`%w: the sweep line's length is outside the representable range`, ErrUnsupported)
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
		return sweepSpanPayload{}, fmt.Errorf(`%w: the sweep line's length has no finite error bound`, ErrUnsupported)
	}
	return sweepSpanPayload{Prism: prismrecord.Payload{
		Profile: profile,
		Frame:   frame,
		Z1:      height,
		Z1Delta: heightBound,
		Xform:   r3.Identity(),
	}}, nil
}

func compositeArcSweepSpan(
	ctx context.Context,
	profile profileRecord,
	plane planeRecord,
	frame r3.Frame,
	record sweepinput.PathRecord,
	work *freeform.FreeformWork,
) (sweepSpanPayload, error) {
	geometry, err := sweeparc.Derive(record.Start, record.Arc,
		record.ArcPhi, record.ArcAngle, plane)
	if err != nil {
		return sweepSpanPayload{}, err
	}
	ax, side, err := revolveplan.ResolveAxisSide(ctx, profile, geometry.Line, work)
	if err != nil {
		return sweepSpanPayload{}, err
	}
	phi0, phi1 := 0.0, geometry.Phi
	den := revolveangle.Sweep{Phi0: revolveangle.Zero(), Phi1: geometry.Angle}
	reverseCaps := false
	if side < 0 {
		phi0, phi1 = -phi1, -phi0
		den.Phi0, den.Phi1 = den.Phi1.Neg(), den.Phi0.Neg()
		reverseCaps = true
	}
	return sweepSpanPayload{
		Revolve: revolvePayload{
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

func (span sweepSpanPayload) transform() r3.Transform {
	if span.Arc {
		return span.Revolve.Xform
	}
	return span.Prism.Xform
}

// Build evaluates one temporary closed span. compositesweep.Assemble
// removes the internal caps and sews adjacent section boundaries after every
// span has succeeded. No temporary body is committed to the document.
func (span *sweepSpanPayload) Build(
	ctx context.Context,
	d *Document,
	ref producerID,
	work *freeform.FreeformWork,
) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if span.Arc {
		body, err := bodyResult(revolveplan.Build(ctx, envOf(documentNode(d), ref), span.Revolve, work))
		if err != nil {
			return nil, err
		}
		restoreSweepArcCapRoles(body, span.ReverseArcCaps)
		if built, ok := body.n.Payload.(revolvePayload); ok {
			span.Revolve = built
		}
		return body, nil
	}
	body, err := bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), span.Prism, work))
	if err != nil {
		return nil, err
	}
	if built, ok := body.n.Payload.(prismPayload); ok {
		span.Prism = built
	}
	return body, nil
}

func restoreSweepArcCapRoles(body *Body, reverse bool) {
	if !reverse {
		return
	}
	for _, face := range body.Faces() {
		for i, origin := range face.n.Origins {
			origin.Role = compositesweep.SwapCapRole(origin.Role, provenance.RoleCapStart, provenance.RoleCapEnd)
			face.n.Origins[i] = origin
		}
	}
}
