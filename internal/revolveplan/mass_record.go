package revolveplan

import (
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
)

// MassRecord presents the revolve record to the mass integrator.
func (rp Payload) MassRecord() massmoment.RevolveRecord {
	return massmoment.RevolveRecord{
		Profile: rp.Profile, Frame: rp.Frame, Transform: rp.Xform,
		Den: rp.Den, Phi0: rp.Phi0, Phi1: rp.Phi1,
		AU: rp.Ax.AU, AV: rp.Ax.AV, DU: rp.Ax.DU, DV: rp.Ax.DV,
		AUBound: rp.Ax.AUBound, AVBound: rp.Ax.AVBound,
		DUBound: rp.Ax.DUBound, DVBound: rp.Ax.DVBound,
		SectionDelta: rp.SectionDelta, RadialAdmitAllow: rp.Ax.RadialAdmitAllow,
		AxisSnap: rp.Ax.Snap != (revolveaxis.SnapAllow{}),
	}
}
