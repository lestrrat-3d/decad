package box

import (
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// SourceAxisBox maps all eight corners of a recorded rectangular prism
// through its frame, placement and query pose using exact held coordinates.
// The caller admits the section and transforms, then checks source topology.
func SourceAxisBox(frame r3.Frame, placement, pose r3.Transform,
	u, v, z [2]float64) (AxisBox, bool) {
	ud := [2]proof.Dyadic{proof.MustDyOf(u[0]), proof.MustDyOf(u[1])}
	vd := [2]proof.Dyadic{proof.MustDyOf(v[0]), proof.MustDyOf(v[1])}
	zd := [2]proof.Dyadic{proof.MustDyOf(z[0]), proof.MustDyOf(z[1])}
	origin := proof.DyVec(frame.Origin())
	fu, fv, fn := proof.DyVec(frame.U()), proof.DyVec(frame.V()), proof.DyVec(frame.N())
	var out AxisBox
	first := true
	for iu := range ud {
		for iv := range vd {
			for iz := range zd {
				p := proof.DvAdd(origin, proof.DvAdd(proof.DvScale(fu, ud[iu]),
					proof.DvAdd(proof.DvScale(fv, vd[iv]), proof.DvScale(fn, zd[iz]))))
				p = proof.DvTransform(placement, p)
				p = proof.DvTransform(pose, p)
				for axis := range 3 {
					if first || proof.DyCmp(p[axis], out.Lo[axis]) < 0 {
						out.Lo[axis] = p[axis]
					}
					if first || proof.DyCmp(p[axis], out.Hi[axis]) > 0 {
						out.Hi[axis] = p[axis]
					}
				}
				first = false
			}
		}
	}
	for axis := range 3 {
		if proof.DyCmp(out.Lo[axis], out.Hi[axis]) >= 0 {
			return AxisBox{}, false
		}
	}
	return out, true
}

// SourcePrismCylinderBox maps a full circular prism's disk center and two
// axial ends, then expands its exact outer box by the recorded radius.
func SourcePrismCylinderBox(frame r3.Frame, placement, pose r3.Transform,
	center sectionrecord.Point2, z0, z1, radius float64, axis int) AxisBox {
	base := proof.DvAdd(proof.DyVec(frame.Origin()), proof.DvAdd(
		proof.DvScale(proof.DyVec(frame.U()), proof.MustDyOf(center.U)),
		proof.DvScale(proof.DyVec(frame.V()), proof.MustDyOf(center.V))))
	low := proof.DvAdd(base, proof.DvScale(proof.DyVec(frame.N()), proof.MustDyOf(z0)))
	high := proof.DvAdd(base, proof.DvScale(proof.DyVec(frame.N()), proof.MustDyOf(z1)))
	low = proof.DvTransform(pose, proof.DvTransform(placement, low))
	high = proof.DvTransform(pose, proof.DvTransform(placement, high))
	return cylinderOuterBox(low, high, proof.MustDyOf(radius), axis)
}

// SourceRevolvedCylinderBox reads every corner of an axis-incident rectangular
// meridian and maps its axial interval and farthest radius into an exact box.
// The caller has admitted the profile shape, axis and held transforms.
func SourceRevolvedCylinderBox(frame r3.Frame, placement, pose r3.Transform,
	anchor, direction sectionrecord.Point2, segments []sectionrecord.CurveSegment, axis int) (AxisBox, bool) {
	var zlo, zhi, rhoLo, rhoHi proof.Dyadic
	for i, seg := range segments {
		line, ok := seg.(sectionrecord.LineSeg)
		if !ok || proofbound.IsNonFinite(line.Start.U) || proofbound.IsNonFinite(line.Start.V) {
			return AxisBox{}, false
		}
		du := proof.DySubScalar(proof.MustDyOf(line.Start.U), proof.MustDyOf(anchor.U))
		dv := proof.DySubScalar(proof.MustDyOf(line.Start.V), proof.MustDyOf(anchor.V))
		z := proof.DyAdd(proof.DyMul(du, proof.MustDyOf(direction.U)), proof.DyMul(dv, proof.MustDyOf(direction.V)))
		rho := proof.DySubScalar(proof.DyMul(dv, proof.MustDyOf(direction.U)),
			proof.DyMul(du, proof.MustDyOf(direction.V)))
		if i == 0 {
			zlo, zhi, rhoLo, rhoHi = z, z, rho, rho
		} else {
			zlo, zhi = dyMin(zlo, z), dyMax(zhi, z)
			rhoLo, rhoHi = dyMin(rhoLo, rho), dyMax(rhoHi, rho)
		}
	}
	if !rhoLo.IsZero() || rhoHi.Sign() <= 0 || proof.DyCmp(zlo, zhi) >= 0 {
		return AxisBox{}, false
	}
	base := proof.DvAdd(proof.DyVec(frame.Origin()), proof.DvAdd(
		proof.DvScale(proof.DyVec(frame.U()), proof.MustDyOf(anchor.U)),
		proof.DvScale(proof.DyVec(frame.V()), proof.MustDyOf(anchor.V))))
	w := proof.DvAdd(proof.DvScale(proof.DyVec(frame.U()), proof.MustDyOf(direction.U)),
		proof.DvScale(proof.DyVec(frame.V()), proof.MustDyOf(direction.V)))
	low := proof.DvTransform(pose, proof.DvTransform(placement, proof.DvAdd(base, proof.DvScale(w, zlo))))
	high := proof.DvTransform(pose, proof.DvTransform(placement, proof.DvAdd(base, proof.DvScale(w, zhi))))
	return cylinderOuterBox(low, high, rhoHi, axis), true
}

func cylinderOuterBox(low, high proof.DyV3, radius proof.Dyadic, axis int) AxisBox {
	var out AxisBox
	for i := range 3 {
		out.Lo[i], out.Hi[i] = dyMin(low[i], high[i]), dyMax(low[i], high[i])
		if i != axis {
			out.Lo[i] = proof.DySubScalar(out.Lo[i], radius)
			out.Hi[i] = proof.DyAdd(out.Hi[i], radius)
		}
	}
	return out
}
