package booleaneval

import (
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// capSides returns the cap sides of owner (agent-docs/coplanar-contact-design.md §19.2): for each planar face, the
// closed half-space of its plane the true solid lies in, when the payload proves the solid meets that plane only in
// the face. It reads three payloads and returns nil for every other:
//
//   - a straight prism, whose walls run along its frame's normal between its two caps;
//   - a plain loft, whose walls are ruled between its two sections, with neither a fillet, a bore, a join gap nor a
//     construction displacement on its record, which meshbool.SlabCapSides then requires to lie on parallel planes;
//   - a revolve whose recorded meridian is the one it denotes, which meshbool.RevolveCapSides then requires to be
//     straight.
//
// A sheet result has no solid to bound.
func capSides(owner *topo.Body) []meshbool.CapSide {
	switch p := owner.Payload.(type) {
	case prismrecord.Payload:
		if p.SurfaceResult {
			return nil
		}
		return meshbool.SlabCapSides(recordCaps(owner))
	case loftrecord.Payload:
		if p.SurfaceResult || len(p.BlendSegs) > 0 || p.ThroughBoreCut || p.AnalyticBore || p.ConstructionDelta != 0 ||
			p.JoinDelta != 0 || p.JoinPlaneGap != 0 {
			return nil
		}
		return meshbool.SlabCapSides(recordCaps(owner))
	case revolveplan.Payload:
		if p.SurfaceResult || p.SectionDelta != 0 {
			return nil
		}
		return meshbool.RevolveCapSides(topobuild.Faces(owner))
	}
	return nil
}

// recordCaps returns owner's faces that carry a CapRecord.
func recordCaps(owner *topo.Body) []*topo.Face {
	var out []*topo.Face
	for _, f := range topobuild.Faces(owner) {
		if f.CapRecord != nil {
			out = append(out, f)
		}
	}
	return out
}
