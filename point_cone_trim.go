package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/r3"
)

// tryPointConeBoolean is an accept-only structural arm. It does not change
// the mesh Boolean fallback for any pair outside this exact source class.
func tryPointConeBoolean(ctx context.Context, op meshbool.OperationKind, d *Document,
	ref producerID, target, tool *Body) (*Body, bool, error) {
	if op != meshbool.OpCut && op != meshbool.OpIntersect {
		return nil, false, nil
	}
	cone, ok := revolveplan.PointConeSource(&tool.n)
	if !ok {
		return nil, false, nil
	}
	fp, ok := target.n.Payload.(facetedPayload)
	if !ok || fp.Xform != r3.Identity() {
		return nil, false, nil
	}
	record := pointConeTrimRecord{Base: fp}
	if fp.PointCone != nil {
		record = *fp.PointCone
	} else if fp.PointSection == nil || fp.PointSection.Apex != (r3.Vec{}) {
		return nil, false, nil
	}
	if !pointcone.AboveFarCap(record.Base.Verts, record.Base.MeshBound, cone.Far) {
		return nil, false, nil
	}
	if op == meshbool.OpCut {
		if record.Lower != nil && !facetedrecord.PointConeLimitLater(record.Lower, cone, record.Base) {
			return nil, false, nil
		}
		record.Lower = cone
	} else {
		if record.Upper != nil && !facetedrecord.PointConeLimitEarlier(record.Upper, cone, record.Base) {
			return nil, false, nil
		}
		record.Upper = cone
	}
	if record.Lower != nil && record.Upper != nil &&
		!facetedrecord.PointConeLimitOrdered(record.Lower, record.Upper, record.Base) {
		return nil, false, nil
	}
	body, err := bodyResult(facetedrecord.BuildPointConeTrim(ctx,
		envOf(documentNode(d), ref), record))
	return body, true, err
}
