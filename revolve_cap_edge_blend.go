package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/tessdispatch"
	"github.com/lestrrat-3d/r3"
)

// This file builds one angular-cap radial-rim blend of a source-authenticated
// quarter cylinder. Its x-bound is either the fillet circle or a chamfer line;
// the other curved bound is the source cylinder. Shared z stations give an
// inscribed polyhedron, so no mesh Boolean contact decision is involved.

// tryRevolveCapRimFillet admits only the unplaced XY rectangular meridian
// swept by an exact denoted quarter turn about +X. The selected edge is the
// radial line at the high-X cap and the sweep's zero-angle plane.
func tryRevolveCapRimFillet(ctx context.Context, body *Body, rp revolvePayload,
	edges []*Edge, radius, radiusDelta float64) (*Body, bool, error) {
	source, ok := rp.RimSource(edgeNodes(edges), radius, radiusDelta)
	if !ok {
		return nil, false, nil
	}
	out, err := buildRevolveCapRimBlend(ctx, body, source)
	return out, true, err
}

// tryRevolveCapRimChamfer uses the same source class and one-edge selector as
// the fillet. An asymmetric setback changes the trimming plane and is staged.
func tryRevolveCapRimChamfer(ctx context.Context, body *Body, rp revolvePayload,
	edges []*Edge, setback, setbackDelta float64, asym *asymmetricChamfer) (*Body, bool, error) {
	if asym != nil {
		return nil, false, nil
	}
	source, ok := rp.RimSource(edgeNodes(edges), setback, setbackDelta)
	if !ok {
		return nil, false, nil
	}
	source.Kind = revolvemesh.RimChamfer
	out, err := buildRevolveCapRimBlend(ctx, body, source)
	return out, true, err
}

func buildRevolveCapRimBlend(ctx context.Context, body *Body, source revolvemesh.RimSource) (*Body, error) {
	stations, bound, missing, verts, err := revolvemesh.RimVertices(source)
	if err != nil {
		return nil, err
	}
	roles := facesByRole(body)
	roleOrder := [5]string{tessdispatch.PrismWallRole(0, 1), tessdispatch.PrismWallRole(0, 2), tessdispatch.PrismWallRole(0, 3),
		provenance.RoleCapStart, provenance.RoleCapEnd}
	for _, role := range roleOrder {
		if roles[role] == nil {
			return nil, fmt.Errorf("%w: the quarter-turn rim has no source %s face", ErrUnsupported, role)
		}
	}
	groups := make([]facetGroup, 6)
	for i, role := range roleOrder {
		f := roles[role]
		groups[i] = facetGroup{Origins: f.originRefs(), Planar: f.isPlanar()}
	}
	ref := body.n.Doc.NextProducer
	blendRole := "fillet(0,1)"
	if source.Kind == revolvemesh.RimChamfer {
		blendRole = "chamfer(0,1)"
	}
	groups[5] = facetGroup{Origins: []provenance.Ref{{Producer: ref, Role: blendRole}}}
	tris, src, vertexBound, areaSlack, dPair, err := revolvemesh.RimFacetsAndBounds(
		ctx, source, stations, verts, bound)
	if err != nil {
		return nil, err
	}
	fp := facetedPayload{Verts: verts, Tris: tris, Src: src, Groups: groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: missing,
		AreaSlack: areaSlack, DPair: dPair, Xform: r3.Identity()}
	out, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(documentHandle(body.n.Doc)), ref), fp))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	documentHandle(body.n.Doc).commit(out, body)
	return out, nil
}
