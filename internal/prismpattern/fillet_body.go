package prismpattern

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
	"github.com/lestrrat-3d/r3"
)

// The joined-prism Fillet changes only the authenticated connector/circle
// junctions. Their true and held old-to-new symmetric differences each fit
// inside 3r disks, enlarged by the source mesh bound for held disks. The
// disks may overlap; summing their complete volumes remains an upper bound.
// The new arc's line/circle contacts and sampling bound apply inside them;
// every unchanged source facet keeps the Union's existing displacement bound.
func FilletBody(ctx context.Context, env topo.Env, mesher verifiedmesh.Mesher, body *topo.Body, edges []*topo.Edge,
	radius, radiusDelta float64) (*topo.Body, error) {
	if radiusDelta != 0 {
		return nil, fmt.Errorf("%w: joined-prism fillet radius carries a displacement", decaderr.ErrUnsupported)
	}
	old, ok := body.Payload.(facetedrecord.Payload)
	if !ok || old.PrismPatternJoin == nil {
		return nil, fmt.Errorf("%w: fillet has no certified joined-prism source", decaderr.ErrUnsupported)
	}
	contacts, err := FilletContacts(ctx, old, edges, radius)
	if err != nil {
		return nil, err
	}
	if len(contacts) == 0 {
		return nil, fmt.Errorf("%w: joined-prism fillet has no selected roots", decaderr.ErrUnsupported)
	}
	rec := old.PrismPatternJoin
	mesh, _, arcBound, err := FilletShell(ctx, rec, contacts, radius, env.Ref)
	if err != nil {
		return nil, err
	}
	bound := proofbound.AbsSumUpper(old.MeshBound, arcBound)
	if proofbound.IsNonFinite(bound) || bound > 0.1 {
		return nil, fmt.Errorf("%w: joined-prism fillet exceeds its 0.1 mm boundary proof", decaderr.ErrUnsupported)
	}
	// Each analytic line/circle cutback and its fillet arc is inside a 3r
	// disk about its original source root. The held cap outline differs only
	// inside the same disk enlarged by the original source-to-held bound and
	// the certified arc sampling error. Triangle inequality with the old
	// Union's occupied-volume certificate gives the following sum.
	height := rec.Source.Blank.Z1 - rec.Source.Blank.Z0
	volumeGap, areaSlack, err := FilletProofBounds(old.MeshBound,
		old.VolSymDiff, old.AreaSlack, arcBound, radius, height, len(contacts))
	if err != nil {
		return nil, err
	}
	vertexBound := make([]float64, len(mesh.Verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	result := facetedrecord.Payload{Verts: mesh.Verts, Tris: mesh.Tris, Src: mesh.Src,
		Groups: mesh.Groups, VertexBound: vertexBound, AxialSectionJoin: true,
		ContactAudited: true, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: facetedrecord.SourceDiameterUpper(mesh.Verts, bound),
		Xform: r3.Identity()}
	if proofbound.IsNonFinite(result.DPair) {
		return nil, fmt.Errorf("%w: joined-prism fillet has no finite diameter proof", decaderr.ErrUnsupported)
	}
	if err := loftcylinder.TagCaps(&result, rec.Blank,
		[2]int{rec.Groups[1], rec.Groups[2]},
		[2]float64{rec.Source.Blank.Z0, rec.Source.Blank.Z1}, env.Ref); err != nil {
		return nil, err
	}
	out, err := facetedrecord.Build(ctx, env, result)
	if err != nil {
		return nil, err
	}
	// The operation is a real consumer of the newly certified body, not only
	// a local shell probe. Keep the build detached until VerifyAll succeeds.
	verified, err := mesher.VerifyAll(ctx, out, 0.1)
	if err != nil {
		return nil, err
	}
	if !verified.BoundaryOK || !verified.SymDiffOK {
		return nil, fmt.Errorf("%w: joined-prism fillet lacks complete verification", decaderr.ErrUnsupported)
	}
	return out, nil
}
