package revolvemass

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
)

// CentroidInput is the resolved axis, section envelope and bounded moments
// from which the revolve's placed centroid is read.
type CentroidInput struct {
	Basis                         revolvemesh.RevolveBasis
	Frame                         r3.Frame
	Placement                     r3.Transform
	Axis                          revolveaxis.Frame
	Full                          bool
	Phi0, Phi1                    float64
	DenPhi0, DenPhi1              revolveangle.Angle
	Q, MZR, MRR, Sweep            proofbound.BoundedScalar
	CoordUpper, SectionDelta      float64
	RadialAdmit, AxialExtentUpper float64
}

// Centroid places the bounded plane-coordinate centroid through the revolve
// basis. The geometry envelope bounds it independently of the Pappus quotient;
// the axis gate's admitted-band charge is added after that independent bound.
func Centroid(in CentroidInput) (r3.Vec, float64) {
	b := in.Basis
	axial := proofbound.BoundedDiv(in.MZR, in.Q)
	cen := b.A3.Add(b.W.Scale(axial.Value))
	centroidScale := proofbound.AbsSumUpper(proofbound.VecMaxAbs(b.A3), axial.Value)
	centroidBound := proofbound.AbsSumUpper(axial.Bound, proofbound.Radius3D(proofbound.AnalyticRoundBound(centroidScale)))
	lift := centroidLift{axialUpper: proofbound.AbsSumUpper(axial.Value, axial.Bound)}
	if !in.Full {
		// The in-plane term integrates the swept radial direction over the
		// interval. It vanishes identically for a full turn.
		sin1, cos1 := revolveangle.EndSinCos(in.DenPhi1, in.Phi1)
		sin0, cos0 := revolveangle.EndSinCos(in.DenPhi0, in.Phi0)
		rx := proofbound.BoundedSub(sin1, sin0)
		ry := proofbound.BoundedSub(cos0, cos1)
		radial := b.E0.Scale(rx.Value).Add(b.E1.Scale(ry.Value))
		radialBound := proofbound.Radius2D(rx.Bound, ry.Bound)
		radialScale := proofbound.BoundedDiv(in.MRR, proofbound.BoundedMul(in.Sweep, in.Q))
		cen = cen.Add(radial.Scale(radialScale.Value))
		radialUpper := centroidVecL1(radial)
		centroidBound = proofbound.AbsSumUpper(
			centroidBound,
			proofbound.ProductUpper(radialScale.Value, radialBound),
			proofbound.ProductUpper(radialUpper, radialScale.Bound),
			proofbound.Radius3D(proofbound.AnalyticRoundBound(proofbound.ProductUpper(radialScale.Value, radialUpper))),
		)
		lift.rxUpper = proofbound.AbsSumUpper(rx.Value, rx.Bound)
		lift.ryUpper = proofbound.AbsSumUpper(ry.Value, ry.Bound)
		lift.scaleUpper = proofbound.AbsSumUpper(radialScale.Value, radialScale.Bound)
	}
	// An absent charge must fold nothing: AbsSumUpper up-rounds even a zero.
	if axisLift := lift.charge(in.Frame, in.Axis, b); axisLift > 0 {
		centroidBound = proofbound.AbsSumUpper(centroidBound, axisLift)
	}
	centroidBound = proofbound.AbsSumUpper(
		centroidBound,
		proofbound.RigidRoundAllow(proofbound.VecMaxAbs(cen), proofbound.VecMaxAbs(in.Placement.Translation())),
	)
	value := in.Placement.Apply(cen)
	centroidBound = math.Min(centroidBound, centroidGeometryBound(in, value))
	return value, proofbound.AbsSumUpper(centroidBound, AdmitBandCharge(in.RadialAdmit, in.AxialExtentUpper))
}

// centroidGeometryBound bounds the centroid independently of the Pappus
// quotient. Every material point starts in the recorded profile plane,
// rotates about the resolved axis, then passes through a rigid placement.
// The L1 envelopes use three times an input L1 norm for any orthogonal map.
// SectionCoordUpper includes the denoted section's displacement from the
// recorded boundary (docs/surface-intersection-design.md §7.2).
func centroidGeometryBound(in CentroidInput, held r3.Vec) float64 {
	coordUpper := revolveaxis.SectionCoordUpper(in.CoordUpper, in.SectionDelta)
	originUpper := centroidVecL1(in.Frame.Origin())
	profileUpper := proofbound.AbsSumUpper(
		originUpper,
		proofbound.ProductUpper(centroidVecL1(in.Frame.U()), coordUpper),
		proofbound.ProductUpper(centroidVecL1(in.Frame.V()), coordUpper),
	)
	aUUpper := proofbound.AbsSumUpper(in.Axis.AU, in.Axis.AUBound)
	aVUpper := proofbound.AbsSumUpper(in.Axis.AV, in.Axis.AVBound)
	axisUpper := proofbound.AbsSumUpper(
		originUpper,
		proofbound.ProductUpper(centroidVecL1(in.Frame.U()), aUUpper),
		proofbound.ProductUpper(centroidVecL1(in.Frame.V()), aVUpper),
	)
	rotatedUpper := proofbound.AbsSumUpper(proofbound.ProductUpper(3, profileUpper), proofbound.ProductUpper(4, axisUpper))
	placedUpper := proofbound.AbsSumUpper(proofbound.ProductUpper(3, rotatedUpper), centroidVecL1(in.Placement.Translation()))
	return proofbound.AbsSumUpper(centroidVecL1(held), placedUpper)
}

// centroidLift carries the magnitudes the centroid's own lift
// A3 + W·axial + (E0·rx + E1·ry)·scale multiplies the axis basis by: the
// axial coordinate's, and for a partial sweep the in-plane term's three
// factors, each its value plus its own proven bound. A full turn leaves the
// partial-sweep three at zero.
type centroidLift struct {
	axialUpper, rxUpper, ryUpper, scaleUpper float64
}

// charge is what that lift owes the axis basis itself, beside the float
// rounding AnalyticRoundBound charges at the centroid's own magnitude.
//
// The anchor A3 is the frame lift O + U·aU + V·aV, whose products and sums
// round at the frame origin's and the anchor's magnitudes rather than at
// A3's: a far sketch plane whose anchor lifts back near the world origin
// rounds at ulp(10⁶) while A3 itself is small. Its exact rounding is
// RevolveLift.ExactPointRound's, read at z = ρ = 0 under the identity.
//
// The axis's own proven displacement moves the basis the true centroid is
// lifted through, and AxisMoments charges it only into the axial coordinate.
// Read as L1 norms of the basis's own change: A3 moves by |U|·aUBound +
// |V|·aVBound, W = U·dU + V·dV by dW = |U|·dUBound + |V|·dVBound,
// E0 = −U·dV + V·dU by dE0 = |U|·dVBound + |V|·dUBound, and E1 = W × E0 by
// at most dW·|E0*| + |W|·dE0. The latter follows from
// W×E0 − W*×E0* = (W − W*)×E0* + W×(E0 − E0*). An L1 cross product is at
// most the product of its operands' L1 norms. Each change moves the
// centroid by its own factor's magnitude. Every term is zero for a frame
// whose anchor lifts exactly and an axis whose anchor and direction have
// no bound.
func (l centroidLift) charge(frame r3.Frame, ax revolveaxis.Frame, b revolvemesh.RevolveBasis) float64 {
	lift := revolvemesh.RevolveLift{Frame: frame, AU: ax.AU, AV: ax.AV, DU: ax.DU, DV: ax.DV}
	a3Round := lift.ExactPointRound(r3.Identity(), 0, 0, 1, 0, b.A3)
	lu, lv := centroidVecL1(frame.U()), centroidVecL1(frame.V())
	anchor := proofbound.AbsSumUpper(proofbound.ProductUpper(lu, ax.AUBound), proofbound.ProductUpper(lv, ax.AVBound))
	dW := proofbound.AbsSumUpper(proofbound.ProductUpper(lu, ax.DUBound), proofbound.ProductUpper(lv, ax.DVBound))
	terms := []float64{a3Round, anchor, proofbound.ProductUpper(l.axialUpper, dW)}
	if l.scaleUpper > 0 {
		dE0 := proofbound.AbsSumUpper(proofbound.ProductUpper(lu, ax.DVBound), proofbound.ProductUpper(lv, ax.DUBound))
		wHeld := proofbound.AbsSumUpper(proofbound.ProductUpper(lu, math.Abs(ax.DU)), proofbound.ProductUpper(lv, math.Abs(ax.DV)))
		e0True := proofbound.AbsSumUpper(
			proofbound.ProductUpper(lu, proofbound.AbsSumUpper(ax.DV, ax.DVBound)),
			proofbound.ProductUpper(lv, proofbound.AbsSumUpper(ax.DU, ax.DUBound)),
		)
		dE1 := proofbound.AbsSumUpper(proofbound.ProductUpper(dW, e0True), proofbound.ProductUpper(wHeld, dE0))
		radial := proofbound.AbsSumUpper(proofbound.ProductUpper(l.rxUpper, dE0), proofbound.ProductUpper(l.ryUpper, dE1))
		terms = append(terms, proofbound.ProductUpper(l.scaleUpper, radial))
	}
	return proofbound.AbsSumUpper(terms...)
}

func centroidVecL1(v r3.Vec) float64 {
	return proofbound.AbsSumUpper(v.X, v.Y, v.Z)
}
