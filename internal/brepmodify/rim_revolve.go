package brepmodify

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessdispatch"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// This file builds one angular-cap radial-rim blend of a source-authenticated
// quarter cylinder. Its x-bound is either the fillet circle or a chamfer line;
// the other curved bound is the source cylinder. Shared z stations give an
// inscribed polyhedron, so no mesh Boolean contact decision is involved.

// RevolveCapRimFillet admits only the unplaced XY rectangular meridian
// swept by an exact denoted quarter turn about +X. The selected edge is the
// radial line at the high-X cap and the sweep's zero-angle plane.
func RevolveCapRimFillet(ctx context.Context, env topo.Env, body *topo.Body, rp revolveplan.Payload,
	edges []*topo.Edge, radius, radiusDelta float64) (*topo.Body, bool, error) {
	source, ok := rp.RimSource(edges, radius, radiusDelta)
	if !ok {
		return nil, false, nil
	}
	out, err := buildRevolveCapRimBlend(ctx, env, body, source)
	return out, true, err
}

// RevolveCapRimChamfer uses the same source class and one-edge selector as
// the fillet. An asymmetric setback changes the trimming plane and is staged.
func RevolveCapRimChamfer(ctx context.Context, env topo.Env, body *topo.Body, rp revolveplan.Payload,
	edges []*topo.Edge, setback, setbackDelta float64, asym *Asymmetric) (*topo.Body, bool, error) {
	if asym != nil {
		return nil, false, nil
	}
	source, ok := rp.RimSource(edges, setback, setbackDelta)
	if !ok {
		return nil, false, nil
	}
	source.Kind = revolvemesh.RimChamfer
	out, err := buildRevolveCapRimBlend(ctx, env, body, source)
	return out, true, err
}

func buildRevolveCapRimBlend(ctx context.Context, env topo.Env, body *topo.Body, source revolvemesh.RimSource) (*topo.Body, error) {
	stations, bound, missing, verts, err := revolvemesh.RimVertices(source)
	if err != nil {
		return nil, err
	}
	roles := topobuild.FacesByRole(body)
	roleOrder := [5]string{tessdispatch.PrismWallRole(0, 1), tessdispatch.PrismWallRole(0, 2), tessdispatch.PrismWallRole(0, 3),
		provenance.RoleCapStart, provenance.RoleCapEnd}
	for _, role := range roleOrder {
		if roles[role] == nil {
			return nil, fmt.Errorf("%w: the quarter-turn rim has no source %s face", decaderr.ErrUnsupported, role)
		}
	}
	groups := make([]facetedrecord.Group, 6)
	for i, role := range roleOrder {
		f := roles[role]
		groups[i] = facetedrecord.Group{Origins: slices.Clone(f.Origins), Planar: f.Surface.Kind() == surfacegeom.KindPlane || f.HeldPlanar}
	}
	ref := env.Ref
	blendRole := "fillet(0,1)"
	if source.Kind == revolvemesh.RimChamfer {
		blendRole = "chamfer(0,1)"
	}
	groups[5] = facetedrecord.Group{Origins: []provenance.Ref{{Producer: ref, Role: blendRole}}}
	tris, src, vertexBound, areaSlack, dPair, err := revolvemesh.RimFacetsAndBounds(
		ctx, source, stations, verts, bound)
	if err != nil {
		return nil, err
	}
	fp := facetedrecord.Payload{Verts: verts, Tris: tris, Src: src, Groups: groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: missing,
		AreaSlack: areaSlack, DPair: dPair, Xform: r3.Identity()}
	return facetedrecord.Build(ctx, env, fp)
}
