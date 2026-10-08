package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/tolerance"
)

// This file adapts Verify's body and its readings to internal/tolerance's
// relative gate and diagnostic records.
//
// Every comparison here is RELATIVE, against a reference the body itself
// supplies — its gate diameter, an edge length, an area or a volume — which
// is what keeps the gate scale-free. bodyToleranceInputs owns the choice of
// reference per reading kind; verify_gate.go owns how the diameter that
// anchors it is proven. See docs/verification-design.md §2-§3.

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
		if edge == nil || !tolerance.UsableMagnitude(edge.length) {
			in.edgeLengthOK = false
			break
		}
		in.edgeLengthValue += edge.length
		if !tolerance.UsableMagnitude(in.edgeLengthValue) {
			in.edgeLengthOK = false
			break
		}
	}
	return in.edgeLengthValue, in.edgeLengthOK
}

func (in *bodyToleranceInputs) areaReference(value float64) (float64, bool) {
	return tolerance.AreaReference(value, in.diameter, in.edgeLength)
}

func (in *bodyToleranceInputs) volumeReference(value float64) (float64, bool) {
	return tolerance.VolumeReference(value, in.area.Value, in.diameter)
}

func (in *bodyToleranceInputs) lengthReference(value float64) (float64, bool) {
	return tolerance.LengthReference(value, in.diameter)
}

func (in *bodyToleranceInputs) diameterReference() (float64, bool) {
	return in.diameter()
}

// The root names keep Verify's existing call sites tied to internal
// tolerance's reading and diagnostic records.
type bodyReadingSet = tolerance.BodyReadings
type bodyReadingVerdicts = tolerance.BodyVerdicts
type bodyReadingDiagSet = tolerance.BodyDiagSet[*Body, JointCell]

// bodyReadingDiagnostics applies verification §3's complete body-field table,
// emitting one diagnostic per present reading that fails — never
// short-circuiting, so a body beyond tolerance on two readings emits two
// (verification §1.1) — and returns every present reading's tolerance
// verdict beside them. Body.Edges already deduplicates topology edges, and
// edgeLength reads each held geometric chain directly even when public
// Edge.Length must refuse a curved boolean rim.
func bodyReadingDiagnostics(ctx context.Context, body *Body, readings bodyReadingSet, rel float64) (bodyReadingVerdicts, bodyReadingDiagSet, error) {
	in := &bodyToleranceInputs{ctx: ctx, body: body, area: readings.Area}
	verdicts, diagSet := tolerance.BodyDiagnostics[*Body, JointCell](body, readings, rel,
		tolerance.BodyReferences{
			Area: in.areaReference, Volume: in.volumeReference,
			Length: in.lengthReference, Diameter: in.diameterReference,
		})
	// A cancellation observed while a reference lazily built its geometry is
	// reported to the caller rather than folded into a Suspect verdict.
	if in.err != nil {
		return bodyReadingVerdicts{}, bodyReadingDiagSet{}, in.err
	}
	return verdicts, diagSet, nil
}

// pairToleranceInputs owns pair-relative references. Clearance uses the
// length reference now; tolerance.Scalar accepts the interference volume
// reference through the same callback path once interference rows land.
type pairToleranceInputs struct {
	diameter float64
}

func (in pairToleranceInputs) lengthReference(value float64) (float64, bool) {
	return tolerance.PairLengthReference(value, in.diameter)
}
