package sweepmitre

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file builds agent-docs/sweep-design.md §16's mitred polyline sweep: the
// §16.3 construction over exact rationals, Table SM rows SM5 through SM8, the
// single rounding into the held vertex table, Table BM's topology, and
// §16.6's four readings, all taken over the rational vertices.
//
// Every section is the image of the one before it under its span's wall
// lines, and every wall line of one span passes through that span's apex (or
// is parallel to the span when its ratio is one). So every wall quad is
// exactly planar, and the map from one section to the next is a projective
// map on a convex region holding the whole section (SM6 states each of its
// conditions as a linear inequality over the vertex, so it holds on the hull
// once it holds at every vertex). That map carries a simple polygon with
// holes to a simple polygon with holes and carries a triangulation of one to
// a triangulation of the other, which is why capEnd reuses capStart's
// triangulation index for index rather than triangulating a second time.

// ConstructPayload triangulates the recorded profile once, then adapts its
// spans to §16.3's exact section construction.
func ConstructPayload(ctx context.Context, mp Payload) (Construction, error) {
	pts2, loopIdx, err := Loops(mp.Profile.Outer, mp.Profile.Holes)
	if err != nil {
		return Construction{}, err
	}
	capTris, err := triangulation.Triangulate(ctx, pts2, loopIdx)
	if err != nil {
		return Construction{}, triangulation.WrapLoftError(err)
	}
	spans := make([]Span, len(mp.Path.Records))
	for k, record := range mp.Path.Records {
		spans[k] = Span{Start: record.Start, End: record.End}
	}
	built, err := Construct(ctx, mp.Plane, pts2, loopIdx, spans, mp.Factors)
	if err != nil {
		return Construction{}, err
	}
	built.LoopIdx, built.CapTris = loopIdx, capTris
	return built, nil
}

// Build builds the body for one payload record: §16.3's exact construction
// and placement, the single rounding, the triangle set and orientation, SM8's
// crossing audit, Table BM's topology and §16.6's readings. The first build
// and every placement run it.
func Build(ctx context.Context, env topo.Env, mp Payload) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	c, err := ConstructPayload(ctx, mp)
	if err != nil {
		return nil, err
	}

	placed, err := PlaceAndRound(c, mp.Xform, mp.LocalVol6 == nil && mp.Xform != r3.Identity())
	if err != nil {
		return nil, err
	}
	exact, localExact, anchor := placed.Exact, placed.LocalExact, placed.Anchor
	verts, vertexBound, delta := placed.Rounded, placed.VertexBound, placed.Delta

	a := Assemble(c, provenance.RoleCapStart, provenance.RoleCapEnd)
	localVol6, localMoments := mp.LocalVol6, mp.LocalMoments
	if localVol6 == nil {
		if localExact == nil {
			localExact = exact
		}
		localVol6, localMoments = VolumeMoments(localExact, a.Tris, c.Anchor)
	}
	vol6, moments := PlacedVolumeMoments(localVol6, localMoments, mp.Xform)
	switch vol6.Sign() {
	case 0:
		return nil, fmt.Errorf(`%w: the mitred sweep encloses no volume`, decaderr.ErrDegenerate)
	case -1:
		// §16.3's orientation rule: one exact sign for the whole shell.
		a.Reversed = true
		for t, tri := range a.Tris {
			a.Tris[t] = [3]int{tri[0], tri[2], tri[1]}
		}
		vol6.Neg(vol6)
		for axis := range moments {
			moments[axis].Neg(moments[axis])
		}
	}
	for t, tri := range a.Tris {
		if loftmesh.TriangleCollapsed(verts, tri) {
			return nil, fmt.Errorf(`%w: rounding the mitred sweep's vertices collapsed its triangle %d (%s)`, decaderr.ErrUnsupported, t, a.Roles[a.TriFace[t]])
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), verts, a.Tris); err != nil {
		if contact, ok := errors.AsType[*loftmesh.LoftContactError](err); ok {
			return nil, fmt.Errorf(`%w: mitred sweep faces %s and %s %s`, decaderr.ErrDegenerate,
				a.Roles[a.TriFace[contact.I]], a.Roles[a.TriFace[contact.J]], contact.Reason)
		}
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	areas, err := TriangleAreas(ctx, exact, a.Tris)
	if err != nil {
		return nil, err
	}
	body := env.Alloc.NewBody()
	body.Solid = true
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Kind = topo.BodySolid
	body.Doc = env.Doc
	faces, err := buildTopology(ctx, env, body, a, exact, verts, areas, delta)
	if err != nil {
		return nil, err
	}
	if err := topobuild.AttachFaceLoops(ctx, faces); err != nil {
		return nil, err
	}
	shell := env.Alloc.NewFaceShell()
	shell.Faces = faces
	lump := env.Alloc.NewLump()
	lump.Shells = []*topo.FaceShell{shell}
	body.Lumps = []*topo.Lump{lump}

	body.Volume = volumeReading(vol6)
	centroid, err := centroidReading(anchor, vol6, moments)
	if err != nil {
		return nil, err
	}
	body.Centroid = centroid
	body.Bounds = boundsReading(exact)
	areaLo, areaHi := new(big.Rat), new(big.Rat)
	for _, enclosure := range areas {
		areaLo.Add(areaLo, enclosure[0])
		areaHi.Add(areaHi, enclosure[1])
	}
	value, bound := Enclosure(areaLo, areaHi)
	body.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(value),
		Exactness: measurement.ExactnessOf(bound),
		Bound:     units.SquareMillimeters(bound),
	}
	if err := topobuild.ValidateLoftBodyMeasurements(body); err != nil {
		return nil, err
	}
	if !measurement.FiniteValues(bound) {
		return nil, fmt.Errorf(`%w: the mitred sweep's area has no finite bound`, decaderr.ErrUnsupported)
	}

	mp.Exact, mp.Verts, mp.Tris, mp.TriFace, mp.FaceRoles, mp.Delta = exact, verts, a.Tris, a.TriFace, a.Roles, delta
	mp.VertexBound = vertexBound
	mp.LocalVol6, mp.LocalMoments = localVol6, localMoments
	body.Payload = mp
	return body, nil
}
