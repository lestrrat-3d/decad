package wallsurvey

import (
	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/r3"
)

// RevolveClearanceCarriers contains one solid revolve's analytic carriers and
// displacement bounds. The caller still owns body admission and topology.
type RevolveClearanceCarriers struct {
	Faces        []*clearance.CFace
	Vertices     []r3.Vec
	Delta        float64
	CarrierDelta float64
}

// BuildRevolveClearanceCarriers builds a revolved body's faces from its own
// payload. Its cap planes, clearance.AngWindow and witnesses read rp.Phi0/rp.Phi1
// as the held sweep angle rather than the denoted one. The angular displacement
// is scaled by the radial envelope, beside frame/placement rounding, per-face
// tilt and distance from the recorded meridian swept about the recorded axis.
// budget must be non-nil.
func BuildRevolveClearanceCarriers(budget *proofbound.WorkBudget, rp revolveplan.Payload) (RevolveClearanceCarriers, bool, error) {
	var no RevolveClearanceCarriers
	if rp.SectionDelta != 0 {
		// The carriers would state exact facts about a section the payload
		// holds only within a displacement (surface-intersection §7.2).
		return no, false, nil
	}
	in, ok, err := RevolveCarrierInput(budget, rp.Profile, rp.Ax, rp.Lift(), rp.AxisBound(),
		rp.Xform, rp.Full, rp.Phi0, rp.Phi1)
	if err != nil || !ok {
		return no, false, err
	}
	built := clearance.BuildRevolveCarriers(in)

	// Delta mirrors the prism carrier builder's three terms: the exact
	// frame/placement point rounding, angular displacement times the radial
	// envelope, and worst tilt across the newly built face carriers.
	pointTerm := clearance.CarrierLiftRound(built.Faces)
	angularTerm := proofbound.ProductUpper(built.AxisRadiusUpper, rp.AngularDelta())
	tiltTerm := clearance.BodyFaceTiltDelta(built.Faces, rp.Frame, rp.Xform)
	delta := proofbound.AbsSumUpper(pointTerm, angularTerm, tiltTerm)
	carrierDelta := delta
	if rp.Full {
		// Full-turn end-angle displacement moves seams and witnesses but no
		// carrier on the complete surface of revolution.
		carrierDelta = proofbound.AbsSumUpper(pointTerm, tiltTerm)
	}
	// The float axis re-expression, axis snap and axis bound can separate a
	// carrier from its record. A zero gap must not add a rounding charge.
	if built.AxisGap > 0 {
		delta = proofbound.AbsSumUpper(delta, built.AxisGap)
		carrierDelta = proofbound.AbsSumUpper(carrierDelta, built.AxisGap)
	}
	return RevolveClearanceCarriers{
		Faces: built.Faces, Vertices: built.Vertices,
		Delta: delta, CarrierDelta: carrierDelta,
	}, true, nil
}
