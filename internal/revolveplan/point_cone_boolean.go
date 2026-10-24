package revolveplan

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// PointConeBoolean is an accept-only structural arm. It does not change
// the mesh Boolean fallback for any pair outside this exact source class.
//
// matchingFaces marks the recorded cone limit as one whose cap meets the cone
// within rounding by design (decad.WithMatchingFaces).
func PointConeBoolean(ctx context.Context, op meshbool.OperationKind, env topo.Env,
	target, tool *topo.Body, matchingFaces bool) (*topo.Body, bool, error) {
	if op != meshbool.OpCut && op != meshbool.OpIntersect {
		return nil, false, nil
	}
	cone, ok := PointConeSource(tool)
	if !ok {
		return nil, false, nil
	}
	cone.MatchingFaces = matchingFaces
	fp, ok := target.Payload.(facetedrecord.Payload)
	if !ok || fp.Xform != r3.Identity() {
		return nil, false, nil
	}
	record := facetedrecord.PointConeTrimRecord{Base: fp}
	if fp.PointCone != nil {
		record = *fp.PointCone
	} else if fp.PointSection == nil || fp.PointSection.Apex != (r3.Vec{}) {
		return nil, false, nil
	}
	// A new limit joins the record only about the axis of every limit the
	// record already holds. A canonical axis never equals a general one, and
	// two parallel axes of different lengths differ too
	// (agent-docs/revolve-axis-design.md §3.3, §4.4).
	for _, held := range []*facetedrecord.PointConeLimit{record.Lower, record.Upper} {
		if held != nil && held.Axis != cone.Axis {
			return nil, false, nil
		}
	}
	if !pointcone.AboveFarCap(record.Base.Verts, record.Base.MeshBound, cone.Far, cone.Axis) {
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
	body, err := facetedrecord.BuildPointConeTrim(ctx, env, record)
	return body, true, err
}
