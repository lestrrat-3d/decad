package revolvemass

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
)

// AxisMoments re-references plane-origin integrals into the revolve axis
// frame. It returns the radial first moment, axial-radial mixed moment, and
// squared-radial moment with their source and arithmetic bounds.
func AxisMoments(ig momentinput.Integrals, ax revolveaxis.Frame) (
	proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar,
) {
	aU, aV := proofbound.MeasuredScalar(ax.AU, ax.AUBound), proofbound.MeasuredScalar(ax.AV, ax.AVBound)
	dU, dV := proofbound.MeasuredScalar(ax.DU, ax.DUBound), proofbound.MeasuredScalar(ax.DV, ax.DVBound)
	nU, nV := proofbound.MeasuredScalar(-ax.DV, ax.DVBound), proofbound.MeasuredScalar(ax.DU, ax.DUBound)
	area := proofbound.MeasuredScalar(ig.Area, ig.AreaBound)
	mu := proofbound.MeasuredScalar(ig.Mu, ig.MuBound)
	mv := proofbound.MeasuredScalar(ig.Mv, ig.MvBound)
	muu := proofbound.MeasuredScalar(ig.Muu, ig.MuuBound)
	muv := proofbound.MeasuredScalar(ig.Muv, ig.MuvBound)
	mvv := proofbound.MeasuredScalar(ig.Mvv, ig.MvvBound)

	iuu := proofbound.BoundedAdd(
		proofbound.BoundedSub(muu, proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(2), aU), mu)),
		proofbound.BoundedMul(proofbound.BoundedMul(aU, aU), area),
	)
	iuv := proofbound.BoundedAdd(
		proofbound.BoundedSub(proofbound.BoundedSub(muv, proofbound.BoundedMul(aU, mv)), proofbound.BoundedMul(aV, mu)),
		proofbound.BoundedMul(proofbound.BoundedMul(aU, aV), area),
	)
	ivv := proofbound.BoundedAdd(
		proofbound.BoundedSub(mvv, proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(2), aV), mv)),
		proofbound.BoundedMul(proofbound.BoundedMul(aV, aV), area),
	)
	q := proofbound.BoundedAdd(
		proofbound.BoundedMul(nU, proofbound.BoundedSub(mu, proofbound.BoundedMul(aU, area))),
		proofbound.BoundedMul(nV, proofbound.BoundedSub(mv, proofbound.BoundedMul(aV, area))),
	)
	mzr := proofbound.BoundedAdd(
		proofbound.BoundedAdd(
			proofbound.BoundedMul(proofbound.BoundedMul(dU, nU), iuu),
			proofbound.BoundedMul(proofbound.BoundedAdd(proofbound.BoundedMul(dU, nV), proofbound.BoundedMul(dV, nU)), iuv),
		),
		proofbound.BoundedMul(proofbound.BoundedMul(dV, nV), ivv),
	)
	mrr := proofbound.BoundedAdd(
		proofbound.BoundedAdd(
			proofbound.BoundedMul(proofbound.BoundedMul(nU, nU), iuu),
			proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.BoundedMul(nU, nV)), iuv),
		),
		proofbound.BoundedMul(proofbound.BoundedMul(nV, nV), ivv),
	)
	return q, mzr, mrr
}

// AdmitBandCharge bounds the area of a radial band admitted by the axis gate.
// It is zero when the gate proved the radial minimum non-negative.
func AdmitBandCharge(radialAllow, axialExtentUpper float64) float64 {
	return proofbound.ProductUpper(radialAllow, axialExtentUpper)
}

// AdmitVolumeCharge bounds the same band after a full turn. Its width and
// radius are both at most radialAllow, so it scales with radialAllow squared.
func AdmitVolumeCharge(radialAllow, axialExtentUpper float64) float64 {
	return proofbound.ProductUpper(proofbound.TwoPiUpper(),
		proofbound.ProductUpper(proofbound.ProductUpper(radialAllow, radialAllow), axialExtentUpper))
}
