package stitchflux

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// MassState reports which bound prevented a stitched curved mass reading.
type MassState uint8

const (
	MassReady MassState = iota
	MassZeroVolume
	MassUnboundedCentroid
)

// MassFromFlux charges placement once to the flux and first moments, then
// divides each moment by the same bounded volume.
func MassFromFlux(flux proofbound.BoundedScalar, moments [3]proofbound.BoundedScalar,
	areaUpper, coordUpper, delta float64) (proofbound.BoundedScalar, [3]proofbound.BoundedScalar, MassState) {
	vol := proofbound.BoundedQuotient(flux.Value, flux.Bound, 3, 0)
	if delta > 0 {
		epsV := proofbound.SweptVolumeAllow(delta, areaUpper)
		epsM := proofbound.SweptMomentAllow(delta, areaUpper, coordUpper+delta)
		vol.Bound = proofbound.AbsSumUpper(vol.Bound, epsV)
		for i := range moments {
			moments[i].Bound = proofbound.AbsSumUpper(moments[i].Bound, epsM)
		}
	}
	if vol.Value == 0 {
		return proofbound.BoundedScalar{}, [3]proofbound.BoundedScalar{}, MassZeroVolume
	}
	var centroid [3]proofbound.BoundedScalar
	for i, moment := range moments {
		centroid[i] = proofbound.BoundedQuotient(moment.Value, moment.Bound, vol.Value, vol.Bound)
		if proofbound.IsNonFinite(centroid[i].Bound) {
			return proofbound.BoundedScalar{}, [3]proofbound.BoundedScalar{}, MassUnboundedCentroid
		}
	}
	return vol, centroid, MassReady
}
