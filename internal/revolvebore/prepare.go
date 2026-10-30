package revolvebore

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// Candidate holds the source records and certified cap regions. It holds no
// topology handles and is not a built result.
type Candidate struct {
	Revolve       revolveplan.Payload
	Tool          prismrecord.Payload
	OriginalTool  prismrecord.Payload
	ToolReversed  bool
	Meridian      Meridian
	Caps          [2]momentinput.Profile
	TargetOrigins map[string][]provenance.Ref
	ToolOrigins   map[string][]provenance.Ref
}

// Prepare applies the current admission gates without building a result.
func Prepare(ctx context.Context, target, tool *topo.Body) (Candidate, bool, error) {
	if err := ctx.Err(); err != nil {
		return Candidate{}, false, err
	}
	rp, pp, ok := Operands(target, tool)
	if !ok {
		return Candidate{}, false, nil
	}
	if !rp.ProvesSimple(ctx) {
		return Candidate{}, false, ctx.Err()
	}
	mer, ok := LinearMeridian(rp)
	if !ok || !mer.HasSlopedWall {
		return Candidate{}, false, nil
	}
	original := pp
	pp, ok, err := MapTool(ctx, rp, mer, pp)
	if err != nil || !ok {
		return Candidate{}, false, err
	}
	if !InsideRadius(pp.Profile.Outer, mer.MinRadius) {
		return Candidate{}, false, nil
	}
	caps, ok, err := CapProfiles(ctx, mer, pp)
	if err != nil || !ok {
		return Candidate{}, false, err
	}
	pa, pb := pp.PlacementOperand(), original.PlacementOperand()
	if pa.Xform == pb.Xform {
		pa.Xform, pb.Xform = r3.Identity(), r3.Identity()
	}
	relative, err := prismplacement.Compose(pa, pb)
	if err != nil {
		return Candidate{}, false, err
	}
	reversed := !relative.Identity && relative.Map.Basis().EZ.Z < 0
	return Candidate{Revolve: rp, Tool: pp, OriginalTool: original, ToolReversed: reversed,
		Meridian: mer, Caps: caps, TargetOrigins: sourceOrigins(target), ToolOrigins: sourceOrigins(tool)}, true, nil
}

func sourceOrigins(body *topo.Body) map[string][]provenance.Ref {
	result := make(map[string][]provenance.Ref)
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			result[origin.Role] = slices.Clone(face.Origins)
		}
	}
	return result
}
