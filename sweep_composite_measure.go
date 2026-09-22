package decad

import (
	"context"
	"fmt"
	"math"

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

const (
	// Composite evaluation builds temporary closed analytic bodies before it
	// sews their shared sections. These ceilings bound that pre-commit work and
	// allocation independently of the caller's path size.
	maxSweepSpansPerCall = 256
	maxSweepFacesPerCall = 262_144
)

func preflightCompositeSweep(profile ProfileRecord, spans int) error {
	if spans < 2 {
		return fmt.Errorf(`%w: a composite sweep requires at least two spans`, ErrDegenerate)
	}
	if spans > maxSweepSpansPerCall {
		return fmt.Errorf(`%w: a composite sweep exceeds the fixed span ceiling of %d`, ErrUnsupported, maxSweepSpansPerCall)
	}
	pairs, ok := wallChoose2(uint64(spans))
	if !ok || pairs > maxFacetPairTestsPerCall {
		return fmt.Errorf(`%w: the composite sweep audit exceeds its fixed pair budget`, ErrUnsupported)
	}
	segments := len(profile.Outer.Segments)
	for _, hole := range profile.Holes {
		var ok bool
		segments, ok = intCheckedAdd(segments, len(hole.Segments))
		if !ok {
			return fmt.Errorf(`%w: a composite sweep's profile topology count overflows`, ErrUnsupported)
		}
	}
	faces, ok := wallCheckedMul(uint64(spans), uint64(segments))
	if !ok {
		return fmt.Errorf(`%w: a composite sweep's face count overflows`, ErrUnsupported)
	}
	faces, ok = wallCheckedAdd(faces, 2)
	if !ok || faces > maxSweepFacesPerCall {
		return fmt.Errorf(`%w: a composite sweep exceeds the fixed face ceiling of %d`, ErrUnsupported, maxSweepFacesPerCall)
	}
	return nil
}

func intCheckedAdd(a, b int) (int, bool) {
	if b > int(^uint(0)>>1)-a {
		return 0, false
	}
	return a + b, true
}

func replayCompositeSweep(
	ctx context.Context,
	d *Document,
	ref producerID,
	payload sweepPayload,
) (*Body, error) {
	return replayCompositeSweepWork(ctx, d, ref, payload, newFreeformWork())
}

func replayCompositeSweepWork(
	ctx context.Context,
	d *Document,
	ref producerID,
	payload sweepPayload,
	work *freeformWork,
) (*Body, error) {
	if err := preflightCompositeSweep(payload.prism.profile, len(payload.spans)); err != nil {
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
	body, err := assembleCompositeSweepBody(ctx, d, ref, parts, payload.surfaceResult)
	if err != nil {
		return nil, err
	}
	if err := aggregateCompositeSweepMeasurements(ctx, body, parts, payload.surfaceResult); err != nil {
		return nil, err
	}
	body.payload = payload
	return body, nil
}

func evalCompositeSweepContext(
	ctx context.Context,
	d *Document,
	ref producerID,
	profile ProfileRecord,
	plane PlaneRecord,
	frame r3.Frame,
	path *Path,
	work *freeformWork,
	surfaceResult bool,
) (*Body, error) {
	if err := preflightCompositeSweep(profile, len(path.records)); err != nil {
		return nil, err
	}
	frames, err := transportSweepFramesContext(ctx, path, plane)
	if err != nil {
		return nil, err
	}
	for i, transported := range frames {
		if transported.originBound != 0 || transported.uBound != 0 || transported.vBound != 0 || transported.nBound != 0 {
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
		prism: prismPayload{
			profile: profile,
			frame:   frame,
			xform:   r3.Identity(),
		},
		spans:         make([]sweepSpanPayload, len(path.records)),
		path:          path,
		surfaceResult: surfaceResult,
	}
	for i, record := range path.records {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		current := frames[i].frame
		var err error
		if record.arc == nil {
			payload.spans[i], err = compositeLineSweepSpan(profile, current, record)
		} else {
			spanPlane := PlaneRecord{Origin: current.Origin(), U: current.U(), V: current.V()}
			payload.spans[i], err = compositeArcSweepSpan(ctx, profile, spanPlane, current, record, work)
		}
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
	}
	return replayCompositeSweepWork(ctx, d, ref, payload, work)
}

func compositeLineSweepSpan(
	profile ProfileRecord,
	frame r3.Frame,
	record pathSegmentRecord,
) (sweepSpanPayload, error) {
	tangent := sweepRatSub(sweepRatVecOf(record.end), sweepRatVecOf(record.start))
	normal := sweepRatVecOf(frame.N())
	if !sweepRatIsZero(sweepRatCross(tangent, normal)) || sweepRatDot(tangent, normal).Sign() <= 0 {
		return sweepSpanPayload{}, fmt.Errorf(`%w: a composite line does not follow its transported section normal`, ErrUnsupported)
	}

	delta := record.end.Sub(record.start)
	if !finiteVec(delta) {
		return sweepSpanPayload{}, fmt.Errorf(`%w: the sweep line's displacement is outside the representable range`, ErrUnsupported)
	}
	height := delta.Len()
	if math.IsInf(height, 0) || math.IsNaN(height) {
		return sweepSpanPayload{}, fmt.Errorf(`%w: the sweep line's length is outside the representable range`, ErrUnsupported)
	}
	heightBound := straightEdgeBound(height, ratSquaredDistance3(
		record.start.X, record.start.Y, record.start.Z,
		record.end.X, record.end.Y, record.end.Z,
	))
	heldSweep := frame.N().Scale(height)
	heightBound = absSumUpper(
		heightBound,
		rationalFloatError(tangent[0], heldSweep.X),
		rationalFloatError(tangent[1], heldSweep.Y),
		rationalFloatError(tangent[2], heldSweep.Z),
	)
	if math.IsInf(heightBound, 0) || math.IsNaN(heightBound) {
		return sweepSpanPayload{}, fmt.Errorf(`%w: the sweep line's length has no finite error bound`, ErrUnsupported)
	}
	return sweepSpanPayload{prism: prismPayload{
		profile: profile,
		frame:   frame,
		z1:      height,
		z1Delta: heightBound,
		xform:   r3.Identity(),
	}}, nil
}

func compositeArcSweepSpan(
	ctx context.Context,
	profile ProfileRecord,
	plane PlaneRecord,
	frame r3.Frame,
	record pathSegmentRecord,
	work *freeformWork,
) (sweepSpanPayload, error) {
	geometry, err := deriveSweepArc(record, plane)
	if err != nil {
		return sweepSpanPayload{}, err
	}
	ax, side, err := resolveAxisSide(ctx, profile, geometry.line, work)
	if err != nil {
		return sweepSpanPayload{}, err
	}
	phi0, phi1 := 0.0, geometry.phi
	den := sweepDenotation{phi0: zeroAngleDenotation(), phi1: geometry.den}
	reverseCaps := false
	if side < 0 {
		phi0, phi1 = -phi1, -phi0
		den.phi0, den.phi1 = den.phi1.neg(), den.phi0.neg()
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
	return span.prism.xform
}

// build evaluates one temporary closed span. assembleCompositeSweepBody
// removes the internal caps and sews adjacent section boundaries after every
// span has succeeded. No temporary body is committed to the document.
func (span *sweepSpanPayload) build(
	ctx context.Context,
	d *Document,
	ref producerID,
	work *freeformWork,
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
		if built, ok := body.payload.(revolvePayload); ok {
			span.revolve = built
		}
		return body, nil
	}
	body, err := evalPrismContext(ctx, d, ref, span.prism, work)
	if err != nil {
		return nil, err
	}
	if built, ok := body.payload.(prismPayload); ok {
		span.prism = built
	}
	return body, nil
}

func restoreSweepArcCapRoles(body *Body, reverse bool) {
	if !reverse {
		return
	}
	for _, face := range body.Faces() {
		for i, origin := range face.origins {
			switch origin.Role {
			case roleCapStart:
				origin.Role = roleCapEnd
			case roleCapEnd:
				origin.Role = roleCapStart
			}
			face.origins[i] = origin
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
// prism_build.go/revolve_build.go compose theirs, rather than being left out
// of the sum from the start.
func aggregateCompositeSweepMeasurements(ctx context.Context, body *Body, parts []compositeSpanPart, surfaceResult bool) error {
	if body == nil || len(parts) == 0 {
		return fmt.Errorf(`%w: a composite sweep requires at least one built span`, ErrDegenerate)
	}
	budget := newWorkBudget(ctx)

	volume := exactScalar(0)
	momentX, momentY, momentZ := exactScalar(0), exactScalar(0), exactScalar(0)
	var minX, minY, minZ, maxX, maxY, maxZ boundedScalar
	for i, part := range parts {
		if err := budget.step(); err != nil {
			return err
		}
		span := part.body
		if span == nil || !span.solid {
			return fmt.Errorf(`%w: composite sweep span %d is not a solid`, ErrDegenerate, i)
		}
		spanVolume := measurementScalar(span.volume)
		spanCentroidBound := span.centroid.Bound.Base()
		volume = boundedAdd(volume, spanVolume)
		momentX = boundedAdd(momentX, boundedMul(
			spanVolume,
			measuredScalar(span.centroid.Value.X, spanCentroidBound),
		))
		momentY = boundedAdd(momentY, boundedMul(
			spanVolume,
			measuredScalar(span.centroid.Value.Y, spanCentroidBound),
		))
		momentZ = boundedAdd(momentZ, boundedMul(
			spanVolume,
			measuredScalar(span.centroid.Value.Z, spanCentroidBound),
		))

		boxBound := span.bounds.Bound.Base()
		boxMinX := measuredScalar(span.bounds.Min.X, boxBound)
		boxMinY := measuredScalar(span.bounds.Min.Y, boxBound)
		boxMinZ := measuredScalar(span.bounds.Min.Z, boxBound)
		boxMaxX := measuredScalar(span.bounds.Max.X, boxBound)
		boxMaxY := measuredScalar(span.bounds.Max.Y, boxBound)
		boxMaxZ := measuredScalar(span.bounds.Max.Z, boxBound)
		if i == 0 {
			minX, minY, minZ = boxMinX, boxMinY, boxMinZ
			maxX, maxY, maxZ = boxMaxX, boxMaxY, boxMaxZ
			continue
		}
		minX, minY, minZ = boundedMin(minX, boxMinX), boundedMin(minY, boxMinY), boundedMin(minZ, boxMinZ)
		maxX, maxY, maxZ = boundedMax(maxX, boxMaxX), boundedMax(maxY, boxMaxY), boundedMax(maxZ, boxMaxZ)
	}

	if admitAbove(volume, 0) != survAdmit {
		return fmt.Errorf(`%w: a composite sweep's volume is not proven positive`, ErrUnsupported)
	}
	centroidX := boundedDiv(momentX, volume)
	centroidY := boundedDiv(momentY, volume)
	centroidZ := boundedDiv(momentZ, volume)
	centroidBound := radius3D(max(centroidX.bound, centroidY.bound, centroidZ.bound))

	area := exactScalar(0)
	for _, face := range body.Faces() {
		if err := budget.step(); err != nil {
			return err
		}
		area = boundedAdd(area, measuredScalar(face.area, face.areaBound))
	}
	if surfaceResult {
		startCap := measuredScalar(parts[0].startCap.area, parts[0].startCap.areaBound)
		endCap := measuredScalar(parts[len(parts)-1].endCap.area, parts[len(parts)-1].endCap.areaBound)
		area = boundedAdd(area, startCap)
		area = boundedAdd(area, endCap)
		area = boundedSub(area, startCap)
		area = boundedSub(area, endCap)
	}

	body.volume = scalarMeasurement(volume, units.CubicMillimeter)
	body.area = scalarMeasurement(area, units.SquareMillimeter)
	body.centroid = VecMeasurement{
		Value:     r3.NewVec(centroidX.value, centroidY.value, centroidZ.value),
		Exactness: exactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}
	body.bounds = compositeSweepBox(minX, minY, minZ, maxX, maxY, maxZ)
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return fmt.Errorf(`%w: a composite sweep computed a non-finite measurement`, ErrUnsupported)
	}
	return budget.err()
}

func measurementScalar(measurement Measurement) boundedScalar {
	return measuredScalar(measurement.Value.Base(), measurement.Bound.Base())
}

func scalarMeasurement(value boundedScalar, unit units.Unit) Measurement {
	return Measurement{
		Value:     units.New(value.value, unit),
		Exactness: exactnessOf(value.bound),
		Bound:     units.New(value.bound, unit),
	}
}

func boundedMax(a, b boundedScalar) boundedScalar {
	return boundedNeg(boundedMin(boundedNeg(a), boundedNeg(b)))
}

func compositeSweepBox(minX, minY, minZ, maxX, maxY, maxZ boundedScalar) Box {
	bound := max(minX.bound, minY.bound, minZ.bound, maxX.bound, maxY.bound, maxZ.bound)
	if bound != 0 {
		bound = math.Nextafter(bound, math.Inf(1))
	}
	return Box{
		Min:       r3.NewVec(minX.value, minY.value, minZ.value),
		Max:       r3.NewVec(maxX.value, maxY.value, maxZ.value),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}
}
