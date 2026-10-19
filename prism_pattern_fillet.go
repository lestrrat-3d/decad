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

// Selected chains must be authenticated source-root generators. A faceted
// edge with the same held endpoints is insufficient without that source map.
func prismPatternFilletContactsOf(ctx context.Context, body *Body, edges []*Edge,
	radius float64) ([]prismpattern.FilletContact, error) {
	fp, ok := body.n.Payload.(facetedPayload)
	if !ok || fp.PrismPatternJoin == nil || radius <= 0 || proofbound.IsNonFinite(radius) {
		return nil, fmt.Errorf("%w: fillet has no certified joined-prism source or radius", ErrUnsupported)
	}
	rec := fp.PrismPatternJoin
	seen := make(map[[2]int]struct{}, len(edges))
	contacts := make([]prismpattern.FilletContact, 0, len(edges))
	for _, edge := range edges {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if edge == nil || !edge.n.SelectorDirectionProven ||
			edge.n.SelectorDirection != (r3.Vec{Z: 1}) && edge.n.SelectorDirection != (r3.Vec{Z: -1}) {
			return nil, fmt.Errorf("%w: joined-prism fillet selection has an unproved axial edge", ErrUnsupported)
		}
		matched := false
		for i, part := range rec.Parts {
			for _, station := range []int{0, part.RootStart} {
				bottom := fp.Verts[part.Vertex[0][station]]
				top := fp.Verts[part.Vertex[1][station]]
				if (edge.n.Start.Position != bottom || edge.n.End.Position != top) &&
					(edge.n.Start.Position != top || edge.n.End.Position != bottom) {
					continue
				}
				key := [2]int{i, station}
				if _, duplicate := seen[key]; duplicate {
					return nil, fmt.Errorf("%w: joined-prism fillet selects one root twice", ErrUnsupported)
				}
				contact, err := prismpattern.FilletContactOf(part, i, station,
					rec.Source.RootArc, rec.Source.RootIndex, rec.Source.BlankRadius, radius)
				if err != nil {
					return nil, err
				}
				contact.RingCut, err = prismpattern.FilletRingCutOf(rec, contact)
				if err != nil {
					return nil, err
				}
				seen[key] = struct{}{}
				contacts = append(contacts, contact)
				matched = true
				break
			}
			if matched {
				break
			}
		}
		if !matched {
			return nil, fmt.Errorf("%w: joined-prism fillet selects no authenticated root", ErrUnsupported)
		}
	}
	if err := prismpattern.FilletClearance(ctx, rec, contacts, radius); err != nil {
		return nil, err
	}
	return contacts, nil
}

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
	height := rec.Source.Blank.Z1 - rec.Source.Blank.Z0
	volumeGap, areaSlack, err := prismpattern.FilletProofBounds(old.MeshBound,
		old.VolSymDiff, old.AreaSlack, arcBound, radius, height, len(contacts))
	if err != nil {
		return nil, err
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
