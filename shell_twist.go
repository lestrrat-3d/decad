package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/proofbound"
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
	node, err := twistsweep.ShellFacetedBody(ctx, envOf(b.n.Doc, ref), held,
		poly.AreaInterval, source.Area, height, thickness)
	if err != nil {
		return nil, err
	}
	out := bodyHandle(node)
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
