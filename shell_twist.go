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

// shellTwistedSweep admits a top opening on a smooth, centrally symmetric,
// strictly convex one-line twisted section. The true cavity is the inward
// normal offset of the swept wall; its held faceted boundary is separately
// certified against that denoted source.
func (b *Body) shellTwistedSweep(ctx context.Context, p twistedSweepPayload,
	removed []*Face, sense ShellSense, thickness, thicknessDelta float64) (*Body, error) {
	if sense != Inward || len(removed) != 1 || removed[0] != prismCapsOf(b).end ||
		p.Path == nil || len(p.Path.Records) != 1 || p.Path.Records[0].Arc != nil ||
		p.Path.Start != (r3.Vec{}) || p.Path.End.X != 0 || p.Path.End.Y != 0 ||
		p.Path.End.Z <= 0 || p.Plane.Origin != (r3.Vec{}) ||
		p.Plane.U != r3.NewVec(1, 0, 0) || p.Plane.V != r3.NewVec(0, 1, 0) ||
		thicknessDelta != 0 {
		return nil, fmt.Errorf(`%w: twisted shell needs one straight axial sweep, a top opening, and exact inward thickness`, ErrUnsupported)
	}
	height := p.Path.End.Z
	source, err := twistsweep.TwistOffsetSource(p.Profile, height, p.Angle, thickness, thicknessDelta)
	if err != nil {
		return nil, err
	}
	stations, err := twistsweep.TwistOffsetSampleStations(p.Profile, height, p.Angle,
		thickness, thicknessDelta, 8, 16)
	if err != nil {
		return nil, err
	}
	poly, err := twistsweep.TwistPolygonOf(p.Profile)
	if err != nil {
		return nil, err
	}
	ref := b.n.Doc.NextProducer
	held, err := twistsweep.TwistShellSampledMesh(ctx, stations, height, thickness,
		p.Angle, poly.AreaInterval)
	if err != nil {
		return nil, err
	}
	out, err := twistShellFacetedBody(ctx, documentHandle(b.n.Doc), ref, held,
		poly.AreaInterval, source.Area, height, thickness)
	if err != nil {
		return nil, err
	}
	if p.Xform != r3.Identity() {
		gap, proofErr := twistsweep.TwistShellPlacedProof(p.Profile, height, thickness,
			p.Angle, source, p.Xform)
		if proofErr != nil {
			return nil, proofErr
		}
		facets, ok := out.n.Payload.(facetedPayload)
		if !ok {
			return nil, fmt.Errorf(`%w: twisted shell lost its held cup`, ErrUnsupported)
		}
		facets.VertexBound = append([]float64(nil), facets.VertexBound...)
		for i := range facets.VertexBound {
			facets.VertexBound[i] = proofbound.AbsSumUpper(facets.VertexBound[i], gap.Boundary)
		}
		facets.MeshBound = proofbound.AbsSumUpper(facets.MeshBound, gap.Boundary)
		facets.VolSymDiff = proofbound.AbsSumUpper(facets.VolSymDiff, gap.Volume)
		facets.AreaSlack = proofbound.AbsSumUpper(facets.AreaSlack, gap.Area)
		out, err = placePayload(ctx, facets, documentHandle(b.n.Doc), ref, p.Xform)
		if err != nil {
			return nil, err
		}
	}
	if err := documentHandle(b.n.Doc).requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	documentHandle(b.n.Doc).commit(out, b)
	return out, nil
}

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
	return bodyResult(facetedrecord.BuildWithProof(ctx, envOf(documentNode(d), ref), payload, audit, measurementToInternal(Measurement{}), nil))
}
