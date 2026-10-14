package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeptransport"
	"github.com/lestrrat-3d/decad/internal/topobuild"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sweepSpanPayload is one certified analytic reduction used by a composite
// Sweep. Its frame already names the section transported to this span's
// start. The common xform on every span is the composite body's accumulated
// rigid placement.
type sweepSpanPayload struct {
	prism          prismPayload
	revolve        revolvePayload
	arc            bool
	reverseArcCaps bool
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
	return replayCompositeSweepWork(ctx, d, ref, payload, newCompositeSweepWork(payload.prism.Profile))
}

func replayCompositeSweepWork(
	ctx context.Context,
	d *Document,
	ref producerID,
	payload sweepPayload,
	work *freeform.FreeformWork,
) (*Body, error) {
	if err := compositesweep.Preflight(payload.prism.Profile, len(payload.spans)); err != nil {
		return nil, err
	}
	parts := make([]compositeSpanPart, len(payload.spans))
	for i := range payload.spans {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		span := &payload.spans[i]
		body, err := span.build(ctx, d, ref, work)
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		parts[i], err = compositeSpanPartOf(body)
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
	}
	var body *Body
	var err error
	if payload.closedComposite {
		body, err = assembleClosedCompositeSweepBody(ctx, d, ref, parts)
	} else {
		body, err = assembleCompositeSweepBody(ctx, d, ref, parts, payload.surfaceResult)
	}
	if err != nil {
		return nil, err
	}
	if err := aggregateCompositeSweepMeasurements(ctx, body, parts, payload.surfaceResult); err != nil {
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
	// assembleCompositeSweepBody alone, which omits only the outer two caps
	// from the published face set after every span has built solid
	// (docs/surface-design.md §4).
	payload := sweepPayload{
		prism: prismrecord.Payload{
			Profile: profile,
			Frame:   frame,
			Xform:   r3.Identity(),
		},
		spans:           make([]sweepSpanPayload, len(path.Records)),
		path:            path,
		surfaceResult:   surfaceResult,
		closedComposite: closedComposite,
	}
	for i, record := range path.Records {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		current := frames[i].Frame
		var err error
		if record.Arc == nil {
			payload.spans[i], err = compositeLineSweepSpan(profile, current, record)
		} else {
			spanPlane := planeRecord{Origin: current.Origin(), U: current.U(), V: current.V()}
			payload.spans[i], err = compositeArcSweepSpan(ctx, profile, spanPlane, current, record, work)
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
	return sweepSpanPayload{prism: prismrecord.Payload{
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
	ax, side, err := resolveAxisSide(ctx, profile, geometry.Line, work)
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
		revolve: revolvePayload{
			profile: profile,
			frame:   frame,
			ax:      ax,
			phi0:    phi0,
			phi1:    phi1,
			den:     den,
			xform:   r3.Identity(),
		},
		arc:            true,
		reverseArcCaps: reverseCaps,
	}, nil
}

func (span sweepSpanPayload) transform() r3.Transform {
	if span.arc {
		return span.revolve.xform
	}
	return span.prism.Xform
}

// build evaluates one temporary closed span. assembleCompositeSweepBody
// removes the internal caps and sews adjacent section boundaries after every
// span has succeeded. No temporary body is committed to the document.
func (span *sweepSpanPayload) build(
	ctx context.Context,
	d *Document,
	ref producerID,
	work *freeform.FreeformWork,
) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if span.arc {
		body, err := evalRevolveContextWork(ctx, d, ref, span.revolve, work)
		if err != nil {
			return nil, err
		}
		restoreSweepArcCapRoles(body, span.reverseArcCaps)
		if built, ok := body.n.Payload.(revolvePayload); ok {
			span.revolve = built
		}
		return body, nil
	}
	body, err := bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), span.prism, work))
	if err != nil {
		return nil, err
	}
	if built, ok := body.n.Payload.(prismPayload); ok {
		span.prism = built
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

// aggregateCompositeSweepMeasurements publishes the union readings after the
// caller has proved that span interiors are disjoint and has assembled their
// shared sections into body. Volume and first moments add over the spans,
// each of which stays a solid regardless of surfaceResult (sweep_composite.go
// keeps every span's own two cap roles for the pairing and sewing that
// already ran); bounds are the componentwise union of the span boxes.
//
// Area is a plain sum over body's own published faces, which already excludes
// every internal cap and, for a surface result, the two outer caps too. For a
// surface result, collapsing to that sum alone would hold the right VALUE but
// an uncomposed bound — §4.3 requires a sheet's area to be the full
// (cap-inclusive) sum minus the two omitted caps, each already-computed
// quantity contributing its own bound. So the two omitted caps are summed
// into the running total and then subtracted back out, exactly as
// internal/prismrecord/build.go and revolve_build.go compose theirs, rather than being left out
// of the sum from the start.
func aggregateCompositeSweepMeasurements(ctx context.Context, body *Body, parts []compositeSpanPart, surfaceResult bool) error {
	if body == nil || len(parts) == 0 {
		return fmt.Errorf(`%w: a composite sweep requires at least one built span`, ErrDegenerate)
	}
	budget := proofbound.NewWorkBudget(ctx)
	spans := make([]compositesweep.Span, len(parts))
	for i, part := range parts {
		span := part.body
		if span == nil {
			continue
		}
		spans[i] = compositesweep.Span{
			Solid: span.n.Solid, Volume: measurementScalar(measurementFromInternal(span.n.Volume)),
			Centroid: span.n.Centroid.Value, CentroidBound: span.n.Centroid.Bound.Base(),
			Min: span.n.Bounds.Min, Max: span.n.Bounds.Max, BoxBound: span.n.Bounds.Bound.Base(),
		}
	}
	faces := body.Faces()
	areas := make([]proofbound.BoundedScalar, len(faces))
	for i, face := range faces {
		areas[i] = proofbound.MeasuredScalar(face.n.Area, face.n.AreaBound)
	}
	var startCap, endCap proofbound.BoundedScalar
	if surfaceResult {
		startCap = proofbound.MeasuredScalar(parts[0].startCap.n.Area, parts[0].startCap.n.AreaBound)
		endCap = proofbound.MeasuredScalar(parts[len(parts)-1].endCap.n.Area, parts[len(parts)-1].endCap.n.AreaBound)
	}
	readings, err := compositesweep.Aggregate(budget, spans, areas, surfaceResult, startCap, endCap)
	if err != nil {
		return err
	}
	centroidBound := proofbound.Radius3D(max(readings.Centroid[0].Bound, readings.Centroid[1].Bound, readings.Centroid[2].Bound))
	body.n.Volume = measurementToInternal(scalarMeasurement(readings.Volume, units.CubicMillimeter))
	body.n.Area = measurementToInternal(scalarMeasurement(readings.Area, units.SquareMillimeter))
	body.n.Centroid = measurement.VecMeasurement{
		Value:     r3.NewVec(readings.Centroid[0].Value, readings.Centroid[1].Value, readings.Centroid[2].Value),
		Exactness: measurement.ExactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}
	body.n.Bounds = boxToInternal(compositeSweepBox(readings.Min[0], readings.Min[1], readings.Min[2],
		readings.Max[0], readings.Max[1], readings.Max[2]))
	if err := topobuild.ValidateAnalyticMeasurements(bodyNode(body)); err != nil {
		return fmt.Errorf(`%w: a composite sweep computed a non-finite measurement`, ErrUnsupported)
	}
	return budget.Err()
}

func measurementScalar(measurement Measurement) proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(measurement.Value.Base(), measurement.Bound.Base())
}

func scalarMeasurement(value proofbound.BoundedScalar, unit units.Unit) Measurement {
	return Measurement{
		Value:     units.New(value.Value, unit),
		Exactness: exactnessOf(value.Bound),
		Bound:     units.New(value.Bound, unit),
	}
}

func compositeSweepBox(minX, minY, minZ, maxX, maxY, maxZ proofbound.BoundedScalar) Box {
	bound := max(minX.Bound, minY.Bound, minZ.Bound, maxX.Bound, maxY.Bound, maxZ.Bound)
	if bound != 0 {
		bound = math.Nextafter(bound, math.Inf(1))
	}
	return Box{
		Min:       r3.NewVec(minX.Value, minY.Value, minZ.Value),
		Max:       r3.NewVec(maxX.Value, maxY.Value, maxZ.Value),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}
}
