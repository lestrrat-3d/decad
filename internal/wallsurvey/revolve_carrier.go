package wallsurvey

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
)

// RevolveCarrierInput resolves the revolve's meridian walks, each beside the recorded
// plane-local walks it was re-expressed from, into the carrier builder's
// input. ok is false for a meridian this kernel cannot decompose, which
// leaves the body with no model.
func RevolveCarrierInput(budget *proofbound.WorkBudget, profile momentinput.Profile, ax revolveaxis.Frame,
	lift revolvemesh.RevolveLift, axis revolvemesh.AxisBound, xform r3.Transform, full bool,
	phi0, phi1 float64) (clearance.RevolveCarrierInput, bool, error) {
	loops, planes, err := RevolveLoopsPlane(budget, profile, ax)
	if err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return clearance.RevolveCarrierInput{}, false, err
		}
		if errors.Is(err, decaderr.ErrUnsupported) {
			// An undecomposable meridian is an unsupported payload, not a
			// failure: newBodyGeom answers with no model rather than a partial
			// one. Other errors remain visible to the caller.
			return clearance.RevolveCarrierInput{}, false, nil
		}
		return clearance.RevolveCarrierInput{}, false, err
	}
	var walls []clearance.RevolveWall
	for li, loop := range loops {
		for _, w := range loop {
			if err := budget.Step(); err != nil {
				return clearance.RevolveCarrierInput{}, false, err
			}
			wall := clearance.RevolveWall{
				Walk: w, Kind: revolveaxis.Classify(w.SegmentWalk, ax.SnapTol),
				First: planes[li][w.Segs[0]], Last: planes[li][w.Segs[len(w.Segs)-1]],
			}
			for _, seg := range w.Segs[1:] {
				rec := planes[li][seg]
				at := ax.Walk(rec)
				wall.Joints = append(wall.Joints, clearance.RevolveJoint{
					Z: at.StartU, Rho: at.StartV,
					Rec: revolvemesh.RecordedMeridian{U: rec.StartU, V: rec.StartV, UV: rec.StartBound},
				})
			}
			walls = append(walls, wall)
		}
	}
	return clearance.RevolveCarrierInput{
		Lift: lift, Axis: axis, Transform: xform, Full: full,
		Phi0: phi0, Phi1: phi1, Walls: walls,
	}, true, nil
}
