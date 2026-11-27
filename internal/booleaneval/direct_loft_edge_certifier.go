package booleaneval

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// directLoftEdgeCertifier vouches for a mesh edge that an unshifted ordinary
// Cut's target and an authenticated direct loft share along the loft's first
// section (agent-docs/coplanar-contact-design.md §22).
type directLoftEdgeCertifier struct {
	target     *topo.Mesh
	targetBool *meshbool.BoolMesh
	loftBody   *topo.Body
	loftMesh   *topo.Mesh
	loftBool   *meshbool.BoolMesh
}

// newDirectLoftEdgeCertifier returns nil unless the loft is an authenticated
// direct loft with sealed original faces.
func newDirectLoftEdgeCertifier(target *topo.Mesh, targetBool *meshbool.BoolMesh, loftBody *topo.Body,
	loftMesh *topo.Mesh, loftBool *meshbool.BoolMesh) meshbool.SharedEdgeCertifier {
	if target == nil || targetBool == nil || loftBody == nil || loftMesh == nil || loftBool == nil {
		return nil
	}
	lp, ok := loftBody.Payload.(loftrecord.Payload)
	if !ok || !lp.MatchesDirectSections(loftBody) {
		return nil
	}
	return &directLoftEdgeCertifier{target: target, targetBool: targetBool, loftBody: loftBody,
		loftMesh: loftMesh, loftBool: loftBool}
}

// CertifySharedEdge implements meshbool.SharedEdgeCertifier. Edge k of target
// facet i and edge l of loft facet j qualify when
//
//   - the loft facet is a wall facet of the original first section, or an
//     original start-cap facet whose edge twin is such a wall facet;
//   - the wall edge lies within the retained straight target edge exactly, and
//     the held edges overlap by a positive length (directLoftSharedEdge);
//   - each of the two target faces beside the edge lies in an exact plane, and
//     the held facets at the edge lie in it;
//   - the held start-cap facets and the authenticated source boundary lie in
//     the cap plane and on one side of the edge's source line;
//   - the original wall triangle matches its paired source lines exactly.
//
// The four planes then pass through one true line, and their half-planes at it
// match the held facets' planes.
func (c *directLoftEdgeCertifier) CertifySharedEdge(ctx context.Context, i, k, j, l int) (bool, error) {
	if i < 0 || j < 0 || k < 0 || k > 2 || l < 0 || l > 2 || i >= len(c.targetBool.Tris) || j >= len(c.loftBool.Tris) {
		return false, nil
	}
	lp, ok := c.loftBody.Payload.(loftrecord.Payload)
	if !ok {
		return false, nil
	}
	wall, wallEdge := j, l
	if j < len(c.loftMesh.Source) && lp.DirectCapFaceMatches(c.loftBody, c.loftMesh.Source[j], 0) {
		var ok bool
		if wall, ok = c.loftBool.TwinFacet(j, l); !ok {
			return false, nil
		}
		wallEdge = -1
		for e := range 3 {
			if t, ok := c.loftBool.TwinFacet(wall, e); ok && t == j {
				wallEdge = e
			}
		}
		if wallEdge < 0 {
			return false, nil
		}
	}
	twin, ok := c.targetBool.TwinFacet(i, k)
	if !ok {
		return false, nil
	}
	twinEdge := -1
	for e := range 3 {
		if t, ok := c.targetBool.TwinFacet(twin, e); ok && t == i {
			twinEdge = e
		}
	}
	if twinEdge < 0 {
		return false, nil
	}
	budget := proofbound.NewStrictReplayBudget().WorkBudget(ctx)
	for _, facet := range [2][2]int{{i, k}, {twin, twinEdge}} {
		shared, err := directLoftSharedEdge(ctx, c.target, c.targetBool, facet[0], facet[1],
			c.loftBody, c.loftMesh, c.loftBool, wall, wallEdge, budget)
		if err != nil || !shared {
			return false, err
		}
		if !c.targetPlaneExact(facet[0]) {
			return false, nil
		}
	}
	side, err := directLoftCapSideSupport(ctx, c.loftBody, c.loftMesh, c.loftBool, wall, wallEdge, budget)
	if err != nil || !side {
		return false, err
	}
	tri := c.loftBool.Tris[wall]
	var prepared [3]r3.Vec
	for j, v := range tri {
		if v < 0 || v >= len(c.loftBool.Verts) {
			return false, nil
		}
		prepared[j] = c.loftBool.Verts[v]
	}
	if wall >= len(lp.Cell) || wall >= len(lp.Side) || lp.Cell[wall][0] != 0 {
		return false, nil
	}
	exact, err := lp.DirectStartWallTriangleExact(ctx, c.loftBody, c.loftMesh, wall,
		lp.Cell[wall][1], int(lp.Side[wall]), prepared, budget)
	if err != nil || !exact {
		return false, err
	}
	if !c.capFacetsInPlane(lp, wall, wallEdge) {
		return false, nil
	}
	return true, budget.Err()
}

// targetPlaneExact reports whether the target face of facet f lies in an exact
// plane: either its held facets are its true piece (zero face bound), or its
// retained plane tag states no displacement and holds the facet exactly.
func (c *directLoftEdgeCertifier) targetPlaneExact(f int) bool {
	if f >= len(c.target.Source) || f >= len(c.target.Triangles) || c.target.Triangles[f] != c.targetBool.Tris[f] {
		return false
	}
	face := c.target.Source[f]
	if face == nil {
		return false
	}
	tri := c.targetBool.Tris[f]
	var pts [3]r3.Vec
	for i, v := range tri {
		if v < 0 || v >= len(c.target.Vertices) || v >= len(c.targetBool.Verts) ||
			c.target.Vertices[v] != c.targetBool.Verts[v] || !proofbound.FiniteVec(c.target.Vertices[v]) {
			return false
		}
		pts[i] = c.target.Vertices[v]
	}
	if bound, ok := c.target.FaceBound[face]; ok && bound == 0 {
		return true
	}
	plane, ok := face.Surface.(surfacegeom.Plane)
	if !ok || !face.RetainedTag || face.ExtentDelta != 0 || face.NormalBound != 0 || face.TagDeparture != 0 ||
		face.HeldNormalBound != 0 {
		return false
	}
	return onPlane(plane.Frame, pts[:])
}

// capFacetsInPlane reports whether the held start-cap facet beside the wall
// edge lies exactly in the loft's first-section plane.
func (c *directLoftEdgeCertifier) capFacetsInPlane(lp loftrecord.Payload, wall, wallEdge int) bool {
	capFacet, ok := c.loftBool.TwinFacet(wall, wallEdge)
	if !ok {
		return false
	}
	tri := c.loftBool.Tris[capFacet]
	pts := [3]r3.Vec{c.loftBool.Verts[tri[0]], c.loftBool.Verts[tri[1]], c.loftBool.Verts[tri[2]]}
	return onPlane(lp.Frame0, pts[:])
}

// onPlane reports whether every point lies exactly in the plane of frame.
func onPlane(frame r3.Frame, pts []r3.Vec) bool {
	if !frame.IsValid() || !proofbound.FiniteVec(frame.Origin()) || !proofbound.FiniteVec(frame.U()) ||
		!proofbound.FiniteVec(frame.V()) {
		return false
	}
	origin := proof.DyVec(frame.Origin())
	normal := proof.DvCross(proof.DyVec(frame.U()), proof.DyVec(frame.V()))
	if proof.DvDot(normal, normal).Sign() == 0 {
		return false
	}
	for _, p := range pts {
		if !proofbound.FiniteVec(p) || proof.DvDot(normal, proof.DvSub(proof.DyVec(p), origin)).Sign() != 0 {
			return false
		}
	}
	return true
}
