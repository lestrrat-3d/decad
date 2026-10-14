package decad

import (
	"context"
	"fmt"
	"math/big"

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
		{origins: []provenance.Ref{{Producer: ref, Role: prismrecord.OuterFirstSideRole}}},
		{origins: []provenance.Ref{{Producer: ref, Role: "shellSide(0,0)"}}},
		{origins: []provenance.Ref{{Producer: ref, Role: provenance.RoleCapStart}}, planar: true},
		{origins: []provenance.Ref{{Producer: ref, Role: roleShellCap}}, planar: true},
		{origins: []provenance.Ref{{Producer: ref, Role: "rim(0)"}}, planar: true},
	}
	payload := facetedPayload{
		verts: held.Verts, vertexBound: vertexBound, tris: held.Tris,
		src: held.Src, groups: groups, meshBound: bound,
		volSymDiff: held.VolumeBound, areaSlack: held.AreaSlack,
		dPair: facetedSourceDiameterUpper(held.Verts, bound), xform: r3.Identity(),
		contactAudited: true,
	}
	return buildFacetedBodyWithProof(ctx, d, ref, payload, audit, Measurement{}, nil)
}
