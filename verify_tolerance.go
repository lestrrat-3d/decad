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
// can be obtained.
type bodyToleranceInputs struct {
	//nolint:containedctx // the reference callbacks have fixed signatures; the gate carries the caller's context through this per-call inputs struct so the diameter build stays cancellable.
	ctx    context.Context
	report *BodyReport
	err    error // a cancellation observed while lazily loading a reference

	diameterLoaded bool
	diameterValue  float64
	diameterOK     bool

	edgeLengthLoaded bool
	edgeLengthValue  float64
	edgeLengthOK     bool
}

func (in *bodyToleranceInputs) diameter() (float64, bool) {
	if !in.diameterLoaded {
		in.diameterValue, in.diameterOK, in.err = bodyGateDiameter(in.ctx, in.report.Body)
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
	for _, edge := range in.report.Body.Edges() {
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
	area := math.Abs(in.report.Area.Value.Base())
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

// bodyReadingDiagnostics applies verification §3's complete body-field table,
// emitting one DiagMeasurementBeyondTolerance per present reading that fails —
// never short-circuiting, so a body beyond tolerance on two readings emits two
// (verification §1.1). Body.Edges already deduplicates topology edges, and
// edgeLength reads each held geometric chain directly even when public
// Edge.Length must refuse a curved boolean rim.
func bodyReadingDiagnostics(ctx context.Context, br *BodyReport, rel float64) ([]Diagnostic, error) {
	in := &bodyToleranceInputs{ctx: ctx, report: br}
	diags := in.readingDiagnostics(rel)
	// A cancellation observed while a reference lazily built its geometry is
	// reported to the caller rather than folded into a Suspect verdict.
	if in.err != nil {
		return nil, in.err
	}
	return diags, nil
}

func (in *bodyToleranceInputs) readingDiagnostics(rel float64) []Diagnostic {
	br := in.report
	var diags []Diagnostic

	scalar := func(reading ReadingKind, m Measurement, reference measurementReference) {
		pass, ref, haveRef := scalarToleranceRef(m, rel, reference)
		if pass {
			return
		}
		obs := m
		d := Diagnostic{
			Code:     DiagMeasurementBeyondTolerance,
			Status:   Suspect,
			Body:     br.Body,
			Reading:  reading,
			Observed: &obs,
			Message:  fmt.Sprintf("the %s reading's bound %s is beyond the relative tolerance", reading, m.Bound),
		}
		if haveRef {
			d.Required = requiredThreshold(rel*ref, m.Value)
		}
		diags = append(diags, d)
	}

	scalar(ReadingArea, br.Area, in.areaReference)

	if pass, ref, haveRef := boundedToleranceRef(br.Bounds.Bound.Base(), rel, in.diameterReference); !pass {
		box := br.Bounds
		d := Diagnostic{
			Code:        DiagMeasurementBeyondTolerance,
			Status:      Suspect,
			Body:        br.Body,
			Reading:     ReadingBounds,
			ObservedBox: &box,
			Message:     fmt.Sprintf("the bounds reading's bound %s is beyond the relative tolerance", br.Bounds.Bound),
		}
		if haveRef {
			d.Required = requiredThreshold(rel*ref, br.Bounds.Bound)
		}
		diags = append(diags, d)
	}

	if br.Volume != nil {
		scalar(ReadingVolume, *br.Volume, in.volumeReference)
	}

	if br.Centroid != nil {
		if pass, ref, haveRef := boundedToleranceRef(br.Centroid.Bound.Base(), rel, in.diameterReference); !pass {
			cen := *br.Centroid
			d := Diagnostic{
				Code:        DiagMeasurementBeyondTolerance,
				Status:      Suspect,
				Body:        br.Body,
				Reading:     ReadingCentroid,
				ObservedVec: &cen,
				Message:     fmt.Sprintf("the centroid reading's bound %s is beyond the relative tolerance", br.Centroid.Bound),
			}
			if haveRef {
				d.Required = requiredThreshold(rel*ref, br.Centroid.Bound)
			}
			diags = append(diags, d)
		}
	}

	if br.MinWallThickness != nil {
		scalar(ReadingWall, *br.MinWallThickness, in.lengthReference)
	}
	if br.MinRadius != nil {
		scalar(ReadingMinRadius, *br.MinRadius, in.lengthReference)
	}
	return diags
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
