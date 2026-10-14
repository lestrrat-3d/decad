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
		p.path == nil || len(p.path.Records) != 1 || p.path.Records[0].Arc != nil ||
		p.path.Start != (r3.Vec{}) || p.path.End.X != 0 || p.path.End.Y != 0 ||
		p.path.End.Z <= 0 || p.plane.Origin != (r3.Vec{}) ||
		p.plane.U != r3.NewVec(1, 0, 0) || p.plane.V != r3.NewVec(0, 1, 0) ||
		thicknessDelta != 0 {
		return nil, fmt.Errorf(`%w: twisted shell needs one straight axial sweep, a top opening, and exact inward thickness`, ErrUnsupported)
	}
	height := p.path.End.Z
	source, err := twistsweep.TwistOffsetSource(p.profile, height, p.angle, thickness, thicknessDelta)
	if err != nil {
		return nil, err
	}
	stations, err := twistsweep.TwistOffsetSampleStations(p.profile, height, p.angle,
		thickness, thicknessDelta, 8, 16)
	if err != nil {
		return nil, err
	}
	poly, err := twistsweep.TwistPolygonOf(p.profile)
	if err != nil {
		return nil, err
	}
	ref := b.n.Doc.NextProducer
	held, err := twistsweep.TwistShellSampledMesh(ctx, stations, height, thickness,
		p.angle, poly.AreaInterval)
	if err != nil {
		return nil, err
	}
	out, err := twistShellFacetedBody(ctx, documentHandle(b.n.Doc), ref, held,
		poly.AreaInterval, source.Area, height, thickness)
	if err != nil {
		return nil, err
	}
	if p.xform != r3.Identity() {
		gap, proofErr := twistsweep.TwistShellPlacedProof(p.profile, height, thickness,
			p.angle, source, p.xform)
		if proofErr != nil {
			return nil, proofErr
		}
		facets, ok := out.payload.(facetedPayload)
		if !ok {
			return nil, fmt.Errorf(`%w: twisted shell lost its held cup`, ErrUnsupported)
		}
		facets.vertexBound = append([]float64(nil), facets.vertexBound...)
		for i := range facets.vertexBound {
			facets.vertexBound[i] = proofbound.AbsSumUpper(facets.vertexBound[i], gap.Boundary)
		}
		facets.meshBound = proofbound.AbsSumUpper(facets.meshBound, gap.Boundary)
		facets.volSymDiff = proofbound.AbsSumUpper(facets.volSymDiff, gap.Volume)
		facets.areaSlack = proofbound.AbsSumUpper(facets.areaSlack, gap.Area)
		out, err = facets.placed(ctx, documentHandle(b.n.Doc), ref, p.xform)
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
