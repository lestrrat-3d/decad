package decad

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// crossCylinderBoreRecord keeps the two full circular prisms whose Cut made
// one perpendicular through-bore. The mesh Cut owns its original contact
// proof; this record permits a later mouth chamfer to rebuild from the exact
// source circles instead of interpreting a sampled intersection as a curve.
type crossCylinderBoreRecord struct {
	Target, Tool    *topo.Body
	Radius, Bore    float64
	Height, CenterZ float64
	SourceBound     float64
}

func crossCylinderCircleSource(body *Body, u, v r3.Vec) (prismPayload, circleSeg, bool) {
	pp, ok := body.n.Payload.(prismPayload)
	if !ok || !body.n.Solid || body.n.Kind != topo.BodySolid || pp.SurfaceResult ||
		pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		pp.Xform != r3.Identity() || pp.Frame.Origin() != (r3.Vec{}) ||
		pp.Frame.U() != u || pp.Frame.V() != v ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		!(pp.Z1 > pp.Z0) {
		return prismPayload{}, circleSeg{}, false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
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
		if _, ok := edge.n.Curve.(surfacegeom.Circle3); !ok {
			continue
		}
		if !edge.n.CurveBounded || proofbound.IsNonFinite(edge.n.CurveBound) {
			return 0, false
		}
		bound = max(bound, edge.n.CurveBound)
		count++
	}
	return bound, count == 2 && bound < radius/1000
}

func recognizeCrossCylinderBore(target, tool *Body) *crossCylinderBoreRecord {
	// The canonical cardinal pose is intentionally narrow. It gives the
	// source intersection two explicit positive/negative X branches.
	tp, outer, ok := crossCylinderCircleSource(target, r3.Vec{X: 1}, r3.Vec{Y: 1})
	if !ok || outer.Center != (planePoint{}) || tp.Z0 != 0 ||
		proofbound.IsNonFinite(tp.Z1) || tp.Z1 <= 0 {
		return nil
	}
	bp, inner, ok := crossCylinderCircleSource(tool, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	if !ok || inner.Center.U != 0 {
		return nil
	}
	R, r, H, c := outer.Radius.Base(), inner.Radius.Base(), tp.Z1, inner.Center.V
	if !proofbound.FiniteVec(r3.Vec{X: R, Y: r, Z: H}) ||
		proofbound.IsNonFinite(c) ||
		R < 2 || R > 20 || r < 0.25 || r > R/4 ||
		H < 2 || H > 40 || c-r <= 0 || c+r >= H ||
		bp.Z0 >= -R-1 || bp.Z1 <= R+1 ||
		math.Abs(bp.Z0) > 100 || math.Abs(bp.Z1) > 100 {
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
		Target: bodyNode(target), Tool: bodyNode(tool), Radius: R, Bore: r,
		Height: H, CenterZ: c, SourceBound: bound,
	}
}
