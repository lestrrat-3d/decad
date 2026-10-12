package decad

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// obliqueBossRecord is a one-use witness for a whole plate-to-cylinder
// junction. The generic Union still proves its own mesh and contact.
type obliqueBossRecord struct {
	plate, boss  *Body
	rect         axisRect
	bottom, top  float64
	base         r3.Vec
	axis, ex, ey r3.Vec
	radius       float64
	length       float64
	sourceBound  float64
}

// recognizeObliqueBoss is reject-only. It never changes the mesh Union or
// asserts that a mesh intersection alone authenticates a source surface.
func recognizeObliqueBoss(a, b *Body) *obliqueBossRecord {
	plate, boss := a, b
	pp, plateOK := plate.payload.(prismPayload)
	cp, circleOK := boss.payload.(prismPayload)
	if !plateOK || !circleOK || len(cp.profile.Outer.Segments) != 1 {
		plate, boss = b, a
		pp, plateOK = plate.payload.(prismPayload)
		cp, circleOK = boss.payload.(prismPayload)
	}
	if !plateOK || !circleOK || pp.surfaceResult || cp.surfaceResult ||
		pp.sectionDelta != 0 || cp.sectionDelta != 0 ||
		pp.z0Delta != 0 || pp.z1Delta != 0 || cp.z0Delta != 0 || cp.z1Delta != 0 ||
		pp.xform != r3.Identity() || pp.frame.Origin() != (r3.Vec{}) ||
		pp.frame.U() != (r3.Vec{X: 1}) || pp.frame.V() != (r3.Vec{Y: 1}) ||
		cp.frame.Origin() != (r3.Vec{}) || cp.frame.U() != (r3.Vec{X: 1}) ||
		cp.frame.V() != (r3.Vec{Y: 1}) || len(cp.profile.Holes) != 0 ||
		len(cp.profile.Outer.Segments) != 1 || pp.z1 <= pp.z0 || cp.z0 != 0 || cp.z1 <= 0 {
		return nil
	}
	rect, ok := axisRectangle(pp.profile)
	if !ok {
		return nil
	}
	circle, ok := cp.profile.Outer.Segments[0].(circleSeg)
	if !ok || circle.Center != (Point2{}) || !circle.CCW ||
		!(circle.TStart == 0 && circle.TEnd == 1) {
		return nil
	}
	radius := circle.Radius.Base()
	length := cp.z1
	if !proofbound.FiniteVec(cp.point(0, 0, cp.z0)) || radius <= 0 ||
		proofbound.IsNonFinite(radius) || proofbound.IsNonFinite(length) || length < radius ||
		radius > 100 || length > 100 || math.Abs(pp.z0) > 10_000 ||
		math.Abs(pp.z1) > 10_000 || math.Abs(rect.u0) > 10_000 ||
		math.Abs(rect.u1) > 10_000 || math.Abs(rect.v0) > 10_000 ||
		math.Abs(rect.v1) > 10_000 {
		return nil
	}
	base := cp.point(0, 0, cp.z0)
	axisHeld := cp.dir(0, 0, 1)
	axisLength := axisHeld.Len()
	if !(axisLength > 0) || proofbound.IsNonFinite(axisLength) {
		return nil
	}
	axis := axisHeld.Scale(1 / axisLength)
	sinTilt := math.Hypot(axis.X, axis.Y)
	if !(sinTilt >= math.Sin(math.Pi/180)) || sinTilt > math.Sin(math.Pi/9) ||
		axis.Z <= 0 || proofbound.IsNonFinite(sinTilt) {
		return nil
	}
	hx, hy := axis.X/sinTilt, axis.Y/sinTilt
	ex := r3.Vec{X: axis.Z * hx, Y: axis.Z * hy, Z: -sinTilt}
	ey := r3.Vec{X: -hy, Y: hx}
	// The two circular cap edges carry the prism builder's radial-projection
	// certificate for the placed Sketch circle.
	sourceBound := 0.0
	capCount := 0
	for _, edge := range boss.Edges() {
		if _, ok := edge.curve.(Circle3); !ok {
			continue
		}
		if !edge.curveBounded {
			return nil
		}
		sourceBound = max(sourceBound, edge.curveBound)
		capCount++
	}
	if capCount != 2 || !(sourceBound < radius/1000) {
		return nil
	}
	farHeld := cp.point(0, 0, cp.z1)
	farIdeal := base.Add(axis.Scale(length))
	poseStretch := prismLiftFactor(cp) - 1
	if poseStretch < 0 || proofbound.IsNonFinite(poseStretch) {
		return nil
	}
	sourceBound = proofbound.AbsSumUpper(sourceBound,
		prismPointBound(cp, proofbound.ExactScalar(0), proofbound.ExactScalar(0), proofbound.ExactScalar(cp.z0)),
		prismPointBound(cp, proofbound.ExactScalar(0), proofbound.ExactScalar(0), proofbound.ExactScalar(cp.z1)),
		proofbound.DvLenUpper(proofbound.HeldDelta(farHeld, farIdeal)),
		proofbound.ProductUpper(poseStretch, proofbound.AbsSumUpper(radius, length)))
	if !(sourceBound < radius/1000) {
		return nil
	}
	// Strict margins include both the ideal cylinder's entire radial reach
	// and the source-to-ideal circle displacement. Fillet checks its own
	// radius against the remaining margin before rebuilding.
	margin := proofbound.AbsSumUpper(proofbound.DivUpper(radius, 0.93), sourceBound, 0.01)
	end := base.Add(axis.Scale(length))
	if !(base.Z-radius*sinTilt-sourceBound > pp.z0 &&
		base.Z+radius*sinTilt+sourceBound < pp.z1 &&
		end.Z-radius*sinTilt-sourceBound > pp.z1+0.01) {
		return nil
	}
	for _, center := range []r3.Vec{base, end} {
		if !(center.X-margin > rect.u0 && center.X+margin < rect.u1 &&
			center.Y-margin > rect.v0 && center.Y+margin < rect.v1) {
			return nil
		}
	}
	return &obliqueBossRecord{
		plate: plate, boss: boss, rect: rect, bottom: pp.z0, top: pp.z1,
		base: base, axis: axis, ex: ex, ey: ey, radius: radius,
		length: length, sourceBound: sourceBound,
	}
}
