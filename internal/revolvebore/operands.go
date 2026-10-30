// Package revolvebore prepares the narrow axial through-bore of a linear Revolve.
package revolvebore

import (
	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Operands reads the evaluator records only for undisplaced solid operands.
// Further geometric and placement proofs are required before constructing a cut.
func Operands(target, tool *topo.Body) (revolveplan.Payload, prismrecord.Payload, bool) {
	if target == nil || tool == nil || !target.Solid || !tool.Solid ||
		target.Kind != topo.BodySolid || tool.Kind != topo.BodySolid {
		return revolveplan.Payload{}, prismrecord.Payload{}, false
	}
	rp, ok := target.Payload.(revolveplan.Payload)
	if !ok || !rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 || rp.SectionWhole || len(rp.Profile.Holes) != 0 ||
		len(rp.BlendSegs) != 0 || rp.BlendKind != "" || rp.DraftCircleCaps || rp.DraftCircleShellRoles ||
		rp.Ax.AUBound != 0 || rp.Ax.AVBound != 0 || rp.Ax.DUBound != 0 || rp.Ax.DVBound != 0 ||
		rp.Ax.Snap != (revolveaxis.SnapAllow{}) || rp.Ax.RadialAdmitAllow != 0 {
		return revolveplan.Payload{}, prismrecord.Payload{}, false
	}
	pp, ok := tool.Payload.(prismrecord.Payload)
	if !ok || pp.SurfaceResult || pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		!(pp.Z0 < pp.Z1) || len(pp.Profile.Holes) != 0 || len(pp.BlendSegs) != 0 || pp.BlendKind != "" ||
		len(pp.Profile.Outer.Segments) == 0 || !classbgeom.NaturalRecord([]sectionrecord.LoopRecord{pp.Profile.Outer}) {
		return revolveplan.Payload{}, prismrecord.Payload{}, false
	}
	den0, den1 := pp.DenotedZ0(), pp.DenotedZ1()
	z0, z1 := proofarith.FloatRat(pp.Z0), proofarith.FloatRat(pp.Z1)
	if den0 == nil || den1 == nil || z0 == nil || z1 == nil || den0.Cmp(z0) != 0 || den1.Cmp(z1) != 0 {
		return revolveplan.Payload{}, prismrecord.Payload{}, false
	}
	for _, segment := range pp.Profile.Outer.Segments {
		switch segment.(type) {
		case sectionrecord.LineSeg, sectionrecord.ArcSeg, sectionrecord.CircleSeg:
		default:
			return revolveplan.Payload{}, prismrecord.Payload{}, false
		}
	}
	return rp, pp, true
}
