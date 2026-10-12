package decad

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// crossCylinderBoreRecord keeps the two full circular prisms whose Cut made
// one perpendicular through-bore. The mesh Cut owns its original contact
// proof; this record permits a later mouth chamfer to rebuild from the exact
// source circles instead of interpreting a sampled intersection as a curve.
type crossCylinderBoreRecord struct {
	target, tool    *Body
	radius, bore    float64
	height, centerZ float64
	sourceBound     float64
}

func crossCylinderCircleSource(body *Body, u, v r3.Vec) (prismPayload, circleSeg, bool) {
	pp, ok := body.payload.(prismPayload)
	if !ok || !body.solid || body.kind != BodySolid || pp.surfaceResult ||
		pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 ||
		pp.xform != r3.Identity() || pp.frame.Origin() != (r3.Vec{}) ||
		pp.frame.U() != u || pp.frame.V() != v ||
		len(pp.profile.Holes) != 0 || len(pp.profile.Outer.Segments) != 1 ||
		!(pp.z1 > pp.z0) {
		return prismPayload{}, circleSeg{}, false
	}
	circle, ok := pp.profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 ||
		!proofbound.FiniteVec(r3.Vec{X: circle.Center.U, Y: circle.Center.V, Z: circle.Radius.Base()}) ||
		!(circle.Radius.Base() > 0) {
		return prismPayload{}, circleSeg{}, false
	}
	return pp, circle, true
}

func crossCylinderCircleBound(body *Body, radius float64) (float64, bool) {
	bound, count := 0.0, 0
	for _, edge := range body.Edges() {
		if _, ok := edge.curve.(Circle3); !ok {
			continue
		}
		if !edge.curveBounded || proofbound.IsNonFinite(edge.curveBound) {
			return 0, false
		}
		bound = max(bound, edge.curveBound)
		count++
	}
	return bound, count == 2 && bound < radius/1000
}

func recognizeCrossCylinderBore(target, tool *Body) *crossCylinderBoreRecord {
	// The canonical cardinal pose is intentionally narrow. It gives the
	// source intersection two explicit positive/negative X branches.
	tp, outer, ok := crossCylinderCircleSource(target, r3.Vec{X: 1}, r3.Vec{Y: 1})
	if !ok || outer.Center != (Point2{}) || tp.z0 != 0 ||
		proofbound.IsNonFinite(tp.z1) || tp.z1 <= 0 {
		return nil
	}
	bp, inner, ok := crossCylinderCircleSource(tool, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	if !ok || inner.Center.U != 0 {
		return nil
	}
	R, r, H, c := outer.Radius.Base(), inner.Radius.Base(), tp.z1, inner.Center.V
	if !proofbound.FiniteVec(r3.Vec{X: R, Y: r, Z: H}) ||
		proofbound.IsNonFinite(c) ||
		R < 2 || R > 20 || r < 0.25 || r > R/4 ||
		H < 2 || H > 40 || c-r <= 0 || c+r >= H ||
		bp.z0 >= -R-1 || bp.z1 <= R+1 ||
		math.Abs(bp.z0) > 100 || math.Abs(bp.z1) > 100 {
		return nil
	}
	targetBound, ok := crossCylinderCircleBound(target, R)
	if !ok {
		return nil
	}
	toolBound, ok := crossCylinderCircleBound(tool, r)
	if !ok {
		return nil
	}
	bound := proofbound.AbsSumUpper(targetBound, toolBound)
	if !(bound < min(R, r)/1000) {
		return nil
	}
	return &crossCylinderBoreRecord{
		target: target, tool: tool, radius: R, bore: r,
		height: H, centerZ: c, sourceBound: bound,
	}
}
