package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/lestrrat-3d/r3"
)

func twistShellFacetedBody(ctx context.Context, d *Document, ref producerID,
	held twistsweep.TwistShellHeld, outerArea, innerArea proofbound.RatInterval,
	height, thickness float64) (*Body, error) {
	if len(held.Verts) == 0 || len(held.Tris) == 0 || len(held.Src) != len(held.Tris) {
		return nil, fmt.Errorf(`%w: twisted shell has no complete held boundary`, ErrUnsupported)
	}
	trueVolume := proofbound.IntervalSub(
		proofbound.IntervalScale(outerArea, twistsweep.TwistRat(height)),
		proofbound.IntervalScale(innerArea,
			new(big.Rat).Sub(twistsweep.TwistRat(height), twistsweep.TwistRat(thickness))))
	if trueVolume.Lo.Sign() <= 0 {
		return nil, fmt.Errorf(`%w: twisted shell has no positive volume proof`, ErrUnsupported)
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
		trueVolume, proofbound.PointInterval(heldVolume))).Cmp(twistsweep.TwistRat(held.VolumeBound)) > 0 {
		return nil, fmt.Errorf(`%w: twisted shell occupied-volume allowance misses the source reading`, ErrUnsupported)
	}
	bound := max(held.OuterBound, held.InnerBound, held.CapBound)
	vertexBound := make([]float64, len(held.Verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	groups := []facetGroup{
		{Origins: []provenance.Ref{{Producer: ref, Role: prismrecord.OuterFirstSideRole}}},
		{Origins: []provenance.Ref{{Producer: ref, Role: "shellSide(0,0)"}}},
		{Origins: []provenance.Ref{{Producer: ref, Role: provenance.RoleCapStart}}, Planar: true},
		{Origins: []provenance.Ref{{Producer: ref, Role: roleShellCap}}, Planar: true},
		{Origins: []provenance.Ref{{Producer: ref, Role: "rim(0)"}}, Planar: true},
	}
	payload := facetedPayload{
		Verts: held.Verts, VertexBound: vertexBound, Tris: held.Tris,
		Src: held.Src, Groups: groups, MeshBound: bound,
		VolSymDiff: held.VolumeBound, AreaSlack: held.AreaSlack,
		DPair: facetedrecord.SourceDiameterUpper(held.Verts, bound), Xform: r3.Identity(),
		ContactAudited: true,
	}
	return buildFacetedBodyWithProof(ctx, d, ref, payload, audit, Measurement{}, nil)
}
