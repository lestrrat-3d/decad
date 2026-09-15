package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/units"
)

// This file is Verify's tolerance gate: given a body's readings and the
// caller's relative tolerance, it decides which of them are trustworthy and
// emits a Diagnostic for each that is not.
//
// Every comparison here is RELATIVE, against a reference the body itself
// supplies — its gate diameter, an edge length, an area or a volume — which
// is what keeps the gate scale-free. bodyToleranceInputs owns the choice of
// reference per reading kind; verify_gate.go owns how the diameter that
// anchors it is proven. See docs/verification-design.md §2-§3.

const toleranceEpsilon = 1e-9

// measurementReference forms one scalar result's reference magnitude from
// its non-negative base-unit value. The callback keeps the shared scalar gate
// usable for body readings, clearances, and the future interference volume.
type measurementReference func(value float64) (float64, bool)

// scalarToleranceRef is the one scalar tolerance gate (verification §2), and it
// hands back the reference it formed. Exactness does not enter: a zero Bound
// passes without asking for a diameter, while every nonzero Bound needs a
// finite, non-negative reference. The comparison is deliberately inclusive.
// haveRef reports whether a usable reference was built — false for a zero Bound
// (which passes without one) and for an unusable magnitude — so a caller may
// report rel*ref as a Required threshold only when it exists.
func scalarToleranceRef(m Measurement, rel float64, reference measurementReference) (bool, float64, bool) {
	bound := m.Bound.Base()
	if !usableMagnitude(bound) {
		return false, 0, false
	}
	if bound == 0 {
		return true, 0, false
	}
	value := math.Abs(m.Value.Base())
	if !usableMagnitude(value) {
		return false, 0, false
	}
	ref, ok := reference(value)
	if !ok {
		return false, 0, false
	}
	return withinTolerance(bound, ref, rel), ref, true
}

// boundedToleranceRef applies the same gate to a bounded non-scalar shape such
// as a Box or position VecMeasurement, handing back the reference it formed on
// the same terms as scalarToleranceRef.
func boundedToleranceRef(bound, rel float64, reference func() (float64, bool)) (bool, float64, bool) {
	if !usableMagnitude(bound) {
		return false, 0, false
	}
	if bound == 0 {
		return true, 0, false
	}
	ref, ok := reference()
	if !ok {
		return false, 0, false
	}
	return withinTolerance(bound, ref, rel), ref, true
}

// requiredThreshold builds the Required value a DiagMeasurementBeyondTolerance
// reports: rel*ref, carried in the reading's own unit so its Kind matches the
// reading's Bound (verification §1.1). sample supplies that unit.
func requiredThreshold(relRef float64, sample units.Value) *units.Value {
	v := units.FromBase(relRef, sample.Unit())
	return &v
}

// judgeTolerance turns the gate's pass/reference outcome into one reading's
// private tolerance verdict (proposal §8): Satisfied with a nil Limit for the
// zero-bound short circuit, Satisfied with the computed Limit when the gate
// accepted a nonzero bound against a usable reference, Exceeded with that
// Limit when the gate compared and rejected it, and Undecided with a nil
// Limit when a nonzero bound had no usable reference.
func judgeTolerance(pass, haveRef bool, rel, ref float64, sample units.Value) toleranceResult {
	if !haveRef {
		if pass {
			return toleranceResult{State: toleranceSatisfied}
		}
		return toleranceResult{State: toleranceUndecided}
	}
	limit := requiredThreshold(rel*ref, sample)
	if pass {
		return toleranceResult{State: toleranceSatisfied, Limit: limit}
	}
	return toleranceResult{State: toleranceExceeded, Limit: limit}
}

// scalarToleranceVerdict judges one scalar reading against the relative
// tolerance gate (proposal §8), returning the diagnostic to emit when its
// state is not Satisfied, nil otherwise. The diagnostic stays
// DiagMeasurementBeyondTolerance even for an Undecided verdict in this PR;
// PR 3 swaps that case for DiagToleranceReferenceUnavailable.
func scalarToleranceVerdict(reading ReadingKind, body *Body, m Measurement, rel float64, reference measurementReference) (toleranceResult, *Diagnostic) {
	pass, ref, haveRef := scalarToleranceRef(m, rel, reference)
	tr := judgeTolerance(pass, haveRef, rel, ref, m.Value)
	if pass {
		return tr, nil
	}
	obs := m
	return tr, &Diagnostic{
		Code:     DiagMeasurementBeyondTolerance,
		Status:   Suspect,
		Body:     body,
		Reading:  reading,
		Observed: &obs,
		Required: tr.Limit,
		Message:  fmt.Sprintf("the %s reading's bound %s is beyond the relative tolerance", reading, m.Bound),
	}
}

// boundsToleranceVerdict is scalarToleranceVerdict's ObservedBox counterpart,
// for the Bounds reading.
func boundsToleranceVerdict(body *Body, box Box, rel float64, reference func() (float64, bool)) (toleranceResult, *Diagnostic) {
	pass, ref, haveRef := boundedToleranceRef(box.Bound.Base(), rel, reference)
	tr := judgeTolerance(pass, haveRef, rel, ref, box.Bound)
	if pass {
		return tr, nil
	}
	observed := box
	return tr, &Diagnostic{
		Code:        DiagMeasurementBeyondTolerance,
		Status:      Suspect,
		Body:        body,
		Reading:     ReadingBounds,
		ObservedBox: &observed,
		Required:    tr.Limit,
		Message:     fmt.Sprintf("the bounds reading's bound %s is beyond the relative tolerance", box.Bound),
	}
}

// centroidToleranceVerdict is scalarToleranceVerdict's ObservedVec
// counterpart, for the Centroid reading.
func centroidToleranceVerdict(body *Body, cen VecMeasurement, rel float64, reference func() (float64, bool)) (toleranceResult, *Diagnostic) {
	pass, ref, haveRef := boundedToleranceRef(cen.Bound.Base(), rel, reference)
	tr := judgeTolerance(pass, haveRef, rel, ref, cen.Bound)
	if pass {
		return tr, nil
	}
	observed := cen
	return tr, &Diagnostic{
		Code:        DiagMeasurementBeyondTolerance,
		Status:      Suspect,
		Body:        body,
		Reading:     ReadingCentroid,
		ObservedVec: &observed,
		Required:    tr.Limit,
		Message:     fmt.Sprintf("the centroid reading's bound %s is beyond the relative tolerance", cen.Bound),
	}
}

func withinTolerance(bound, ref, rel float64) bool {
	if !usableMagnitude(ref) {
		return false
	}
	if ref == 0 || rel == 0 {
		return bound <= rel*ref
	}
	// Compare the represented ratio directly: multiplying that ratio back by
	// ref can round one ulp below the bound at the inclusive boundary.
	return bound/ref <= rel
}

func usableMagnitude(v float64) bool {
	return v >= 0 && !math.IsNaN(v) && !math.IsInf(v, 0)
}

// bodyToleranceInputs lazily reads one body's intrinsic reference data. The
// laziness is part of the contract: a zero Bound passes even when no usable D
// can be obtained. It carries the body and its certified area directly
// rather than a *BodyReport, so the gate no longer depends on the legacy
// report shape.
type bodyToleranceInputs struct {
	//nolint:containedctx // the reference callbacks have fixed signatures; the gate carries the caller's context through this per-call inputs struct so the diameter build stays cancellable.
	ctx  context.Context
	body *Body
	area Measurement
	err  error // a cancellation observed while lazily loading a reference

	diameterLoaded bool
	diameterValue  float64
	diameterOK     bool

	edgeLengthLoaded bool
	edgeLengthValue  float64
	edgeLengthOK     bool
}

func (in *bodyToleranceInputs) diameter() (float64, bool) {
	if !in.diameterLoaded {
		in.diameterValue, in.diameterOK, in.err = bodyGateDiameter(in.ctx, in.body)
		in.diameterLoaded = true
	}
	return in.diameterValue, in.diameterOK
}

func (in *bodyToleranceInputs) edgeLength() (float64, bool) {
	if in.edgeLengthLoaded {
		return in.edgeLengthValue, in.edgeLengthOK
	}
	in.edgeLengthLoaded = true
	in.edgeLengthOK = true
	for _, edge := range in.body.Edges() {
		if edge == nil || !usableMagnitude(edge.length) {
			in.edgeLengthOK = false
			break
		}
		in.edgeLengthValue += edge.length
		if !usableMagnitude(in.edgeLengthValue) {
			in.edgeLengthOK = false
			break
		}
	}
	return in.edgeLengthValue, in.edgeLengthOK
}

func (in *bodyToleranceInputs) areaReference(value float64) (float64, bool) {
	diameter, ok := in.diameter()
	if !ok {
		return 0, false
	}
	edgeLength, ok := in.edgeLength()
	if !ok {
		return 0, false
	}
	return math.Max(value, toleranceEpsilon*diameter*edgeLength), true
}

func (in *bodyToleranceInputs) volumeReference(value float64) (float64, bool) {
	diameter, ok := in.diameter()
	if !ok {
		return 0, false
	}
	area := math.Abs(in.area.Value.Base())
	if !usableMagnitude(area) {
		return 0, false
	}
	return math.Max(value, toleranceEpsilon*diameter*area), true
}

func (in *bodyToleranceInputs) lengthReference(value float64) (float64, bool) {
	diameter, ok := in.diameter()
	if !ok {
		return 0, false
	}
	return math.Max(value, toleranceEpsilon*diameter), true
}

func (in *bodyToleranceInputs) diameterReference() (float64, bool) {
	return in.diameter()
}

// bodyReadingSet carries the readings bodyReadingDiagnostics judges — one
// field per reading kind — so the gate depends on the body and its own
// readings directly rather than on the legacy BodyReport shape. Area and
// Bounds exist for every body; the rest are nil where the reading does not
// exist (an unproven region, or a survey not asked, unavailable, undecided,
// or proven absent).
type bodyReadingSet struct {
	Area     Measurement
	Bounds   Box
	Volume   *Measurement
	Centroid *VecMeasurement
	Wall     *Measurement
	Radius   *Measurement
}

// bodyReadingVerdicts is bodyReadingDiagnostics' per-reading tolerance
// verdict (proposal §8): the zero value's toleranceNotEvaluated stands for a
// reading bodyReadingSet did not carry.
type bodyReadingVerdicts struct {
	Area     toleranceResult
	Bounds   toleranceResult
	Volume   toleranceResult
	Centroid toleranceResult
	Wall     toleranceResult
	Radius   toleranceResult
}

// bodyReadingDiagnostics applies verification §3's complete body-field table,
// emitting one DiagMeasurementBeyondTolerance per present reading that fails —
// never short-circuiting, so a body beyond tolerance on two readings emits two
// (verification §1.1) — and returns every present reading's tolerance
// verdict beside them. Body.Edges already deduplicates topology edges, and
// edgeLength reads each held geometric chain directly even when public
// Edge.Length must refuse a curved boolean rim.
func bodyReadingDiagnostics(ctx context.Context, body *Body, readings bodyReadingSet, rel float64) (bodyReadingVerdicts, []Diagnostic, error) {
	in := &bodyToleranceInputs{ctx: ctx, body: body, area: readings.Area}
	verdicts, diags := in.readingDiagnostics(readings, rel)
	// A cancellation observed while a reference lazily built its geometry is
	// reported to the caller rather than folded into a Suspect verdict.
	if in.err != nil {
		return bodyReadingVerdicts{}, nil, in.err
	}
	return verdicts, diags, nil
}

func (in *bodyToleranceInputs) readingDiagnostics(r bodyReadingSet, rel float64) (bodyReadingVerdicts, []Diagnostic) {
	var diags []Diagnostic
	var verdicts bodyReadingVerdicts

	scalar := func(reading ReadingKind, m Measurement, reference measurementReference) toleranceResult {
		tr, diag := scalarToleranceVerdict(reading, in.body, m, rel, reference)
		if diag != nil {
			diags = append(diags, *diag)
		}
		return tr
	}

	verdicts.Area = scalar(ReadingArea, r.Area, in.areaReference)

	boundsTr, boundsDiag := boundsToleranceVerdict(in.body, r.Bounds, rel, in.diameterReference)
	verdicts.Bounds = boundsTr
	if boundsDiag != nil {
		diags = append(diags, *boundsDiag)
	}

	if r.Volume != nil {
		verdicts.Volume = scalar(ReadingVolume, *r.Volume, in.volumeReference)
	}

	if r.Centroid != nil {
		cenTr, cenDiag := centroidToleranceVerdict(in.body, *r.Centroid, rel, in.diameterReference)
		verdicts.Centroid = cenTr
		if cenDiag != nil {
			diags = append(diags, *cenDiag)
		}
	}

	if r.Wall != nil {
		verdicts.Wall = scalar(ReadingWall, *r.Wall, in.lengthReference)
	}
	if r.Radius != nil {
		verdicts.Radius = scalar(ReadingMinRadius, *r.Radius, in.lengthReference)
	}
	return verdicts, diags
}

// pairToleranceInputs owns pair-relative references. Clearance uses the
// length reference now; scalarToleranceRef accepts the interference volume
// reference through the same callback path once interference rows land.
type pairToleranceInputs struct {
	diameter float64
}

func (in pairToleranceInputs) lengthReference(value float64) (float64, bool) {
	if !usableMagnitude(in.diameter) {
		return 0, false
	}
	return math.Max(value, toleranceEpsilon*in.diameter), true
}
