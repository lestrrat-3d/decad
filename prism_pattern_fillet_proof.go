package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismpattern"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// The joined-prism Fillet changes only the authenticated connector/circle
// junctions. Their true and held old-to-new symmetric differences each fit
// inside 3r disks, enlarged by the source mesh bound for held disks. The
// disks may overlap; summing their complete volumes remains an upper bound.
// The new arc's line/circle contacts and sampling bound apply inside them;
// every unchanged source facet keeps the Union's existing displacement bound.
func prismPatternFilletBody(ctx context.Context, body *Body, edges []*Edge,
	radius, radiusDelta float64) (*Body, error) {
	if radiusDelta != 0 {
		return nil, fmt.Errorf("%w: joined-prism fillet radius carries a displacement", ErrUnsupported)
	}
	old, ok := body.n.Payload.(facetedPayload)
	if !ok || old.PrismPatternJoin == nil {
		return nil, fmt.Errorf("%w: fillet has no certified joined-prism source", ErrUnsupported)
	}
	contacts, err := prismPatternFilletContactsOf(ctx, body, edges, radius)
	if err != nil {
		return nil, err
	}
	if len(contacts) == 0 {
		return nil, fmt.Errorf("%w: joined-prism fillet has no selected roots", ErrUnsupported)
	}
	rec := old.PrismPatternJoin
	ref := body.n.Doc.NextProducer
	mesh, _, arcBound, err := prismpattern.FilletShell(ctx, rec, contacts, radius, ref)
	if err != nil {
		return nil, err
	}
	bound := proofbound.AbsSumUpper(old.MeshBound, arcBound)
	if proofbound.IsNonFinite(bound) || bound > 0.1 {
		return nil, fmt.Errorf("%w: joined-prism fillet exceeds its 0.1 mm boundary proof", ErrUnsupported)
	}
	// Each analytic line/circle cutback and its fillet arc is inside a 3r
	// disk about its original source root. The held cap outline differs only
	// inside the same disk enlarged by the original source-to-held bound and
	// the certified arc sampling error. Triangle inequality with the old
	// Union's occupied-volume certificate gives the following sum.
	piUpper := proofbound.RatFloatUp(proofbound.PiUpper)
	trueDisk := proofbound.ProductUpper(3, radius)
	heldDisk := proofbound.AbsSumUpper(trueDisk, old.MeshBound, arcBound)
	height := rec.Source.Blank.Z1 - rec.Source.Blank.Z0
	if height <= 0 || proofbound.IsNonFinite(heldDisk) {
		return nil, fmt.Errorf("%w: joined-prism fillet has no finite local disk", ErrUnsupported)
	}
	n := float64(len(contacts))
	trueDiskArea := proofbound.ProductUpper(piUpper, proofbound.ProductUpper(trueDisk, trueDisk))
	heldDiskArea := proofbound.ProductUpper(piUpper, proofbound.ProductUpper(heldDisk, heldDisk))
	volumeBand := proofbound.ProductUpper(n, proofbound.ProductUpper(height,
		proofbound.AbsSumUpper(trueDiskArea, heldDiskArea)))
	volumeGap := proofbound.AbsSumUpper(old.VolSymDiff, volumeBand)
	// Old and new walls inside each disk have total area below four full
	// cylinder walls; both changed caps fit in four complete disk faces.
	areaBand := proofbound.ProductUpper(n, proofbound.ProductUpper(4*piUpper,
		proofbound.AbsSumUpper(proofbound.ProductUpper(heldDisk, height),
			proofbound.ProductUpper(heldDisk, heldDisk))))
	areaSlack := proofbound.AbsSumUpper(old.AreaSlack, areaBand)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: joined-prism fillet has no finite volume or area proof", ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.Verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	result := facetedPayload{Verts: mesh.Verts, Tris: mesh.Tris, Src: mesh.Src,
		Groups: mesh.Groups, VertexBound: vertexBound, AxialSectionJoin: true,
		ContactAudited: true, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: facetedrecord.SourceDiameterUpper(mesh.Verts, bound),
		Xform: r3.Identity()}
	if proofbound.IsNonFinite(result.DPair) {
		return nil, fmt.Errorf("%w: joined-prism fillet has no finite diameter proof", ErrUnsupported)
	}
	if err := loftCylinderTagCaps(&result, bodyHandle(rec.Blank),
		[2]int{rec.Groups[1], rec.Groups[2]},
		[2]float64{rec.Source.Blank.Z0, rec.Source.Blank.Z1}, ref); err != nil {
		return nil, err
	}
	out, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(documentHandle(body.n.Doc)), ref), result))
	if err != nil {
		return nil, err
	}
	// The operation is a real consumer of the newly certified body, not only
	// a local shell probe. Keep the build detached until VerifyAll succeeds.
	verified, err := out.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
	if err != nil {
		return nil, err
	}
	if !verified.BoundaryVerified() || !verified.VolumeVerified() {
		return nil, fmt.Errorf("%w: joined-prism fillet lacks complete verification", ErrUnsupported)
	}
	return out, nil
}
