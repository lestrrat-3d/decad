package twistsweep

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func ShellFacetedBody(ctx context.Context, env topo.Env,
	held TwistShellHeld, outerArea, innerArea proofbound.RatInterval,
	height, thickness float64) (*topo.Body, error) {
	if len(held.Verts) == 0 || len(held.Tris) == 0 || len(held.Src) != len(held.Tris) {
		return nil, fmt.Errorf(`%w: twisted shell has no complete held boundary`, decaderr.ErrUnsupported)
	}
	trueVolume := proofbound.IntervalSub(
		proofbound.IntervalScale(outerArea, TwistRat(height)),
		proofbound.IntervalScale(innerArea,
			new(big.Rat).Sub(TwistRat(height), TwistRat(thickness))))
	if trueVolume.Lo.Sign() <= 0 {
		return nil, fmt.Errorf(`%w: twisted shell has no positive volume proof`, decaderr.ErrUnsupported)
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, held.Verts, held.Tris)
	if err != nil {
		return nil, err
	}
	heldVolume := new(big.Rat)
	for _, volume := range audit.CompVol {
		heldVolume.Add(heldVolume, volume)
	}
	if heldVolume.Sign() <= 0 || proofbound.IntervalAbsUpper(proofbound.IntervalSub(
		trueVolume, proofbound.PointInterval(heldVolume))).Cmp(TwistRat(held.VolumeBound)) > 0 {
		return nil, fmt.Errorf(`%w: twisted shell occupied-volume allowance misses the source reading`, decaderr.ErrUnsupported)
	}
	bound := max(held.OuterBound, held.InnerBound, held.CapBound)
	vertexBound := make([]float64, len(held.Verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	groups := []facetedrecord.Group{
		{Origins: []provenance.Ref{{Producer: env.Ref, Role: prismrecord.OuterFirstSideRole}}},
		{Origins: []provenance.Ref{{Producer: env.Ref, Role: "shellSide(0,0)"}}},
		{Origins: []provenance.Ref{{Producer: env.Ref, Role: provenance.RoleCapStart}}, Planar: true},
		{Origins: []provenance.Ref{{Producer: env.Ref, Role: cupwall.RoleShellCap}}, Planar: true},
		{Origins: []provenance.Ref{{Producer: env.Ref, Role: "rim(0)"}}, Planar: true},
	}
	payload := facetedrecord.Payload{
		Verts: held.Verts, VertexBound: vertexBound, Tris: held.Tris,
		Src: held.Src, Groups: groups, MeshBound: bound,
		VolSymDiff: held.VolumeBound, AreaSlack: held.AreaSlack,
		DPair: facetedrecord.SourceDiameterUpper(held.Verts, bound), Xform: r3.Identity(),
		ContactAudited: true,
	}
	return facetedrecord.BuildWithProof(ctx, env, payload, audit, measurement.Measurement{}, nil)
}
