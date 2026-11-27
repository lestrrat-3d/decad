package booleaneval

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// directLoftSharedEdge verifies that two held mesh edges overlap on a single
// retained target edge and an authenticated straight first-section loft edge.
// It says nothing about the incident faces or whether Boolean contact is safe.
func directLoftSharedEdge(ctx context.Context, target *topo.Mesh, targetBool *meshbool.BoolMesh,
	targetFacet, targetEdge int, loftBody *topo.Body, loftMesh *topo.Mesh, loftBool *meshbool.BoolMesh,
	loftFacet, loftEdge int, budget *proofbound.WorkBudget) (bool, error) {
	if ctx == nil || budget == nil {
		return false, fmt.Errorf("direct loft edge: %w", decaderr.ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	if loftBody == nil || loftMesh == nil || loftBool == nil || !loftMesh.BoundaryOK ||
		loftFacet < 0 || loftFacet >= len(loftBool.Tris) || loftEdge < 0 || loftEdge >= 3 ||
		loftFacet >= len(loftMesh.Triangles) || loftMesh.Triangles[loftFacet] != loftBool.Tris[loftFacet] {
		return false, nil
	}
	lp, ok := loftBody.Payload.(loftrecord.Payload)
	if !ok || loftFacet >= len(lp.Cell) || loftFacet >= len(lp.Side) || lp.Cell[loftFacet][0] != 0 {
		return false, nil
	}
	adjacent, ok := loftBool.TwinFacet(loftFacet, loftEdge)
	if !ok || adjacent < 0 || adjacent >= len(loftBool.Tris) || adjacent >= len(loftMesh.Triangles) ||
		loftMesh.Triangles[adjacent] != loftBool.Tris[adjacent] {
		return false, nil
	}
	prepared := func(facet int) ([3]r3.Vec, bool) {
		var points [3]r3.Vec
		for i, v := range loftBool.Tris[facet] {
			if v < 0 || v >= len(loftBool.Verts) {
				return points, false
			}
			points[i] = loftBool.Verts[v]
		}
		return points, true
	}
	wall, ok := prepared(loftFacet)
	if !ok {
		return false, nil
	}
	capPoints, ok := prepared(adjacent)
	if !ok || !lp.DirectCapFacetMatches(loftBody, loftMesh, adjacent, 0, capPoints) {
		return false, nil
	}
	segment, ok := lp.DirectWallFacetSourceSegment(loftBody, loftMesh, loftFacet,
		lp.Cell[loftFacet][1], int(lp.Side[loftFacet]), wall)
	if !ok {
		return false, nil
	}
	edge, err := meshbool.RetainedEdgeAtMeshEdge(ctx, target, targetBool, targetFacet, targetEdge, budget)
	if err != nil || edge == nil {
		return false, err
	}
	inside, err := lp.DirectSectionLineWithinEdge(ctx, loftBody, 0, segment, edge, budget)
	if err != nil || !inside {
		return false, err
	}
	loftPoints := [2]r3.Vec{wall[loftEdge], wall[(loftEdge+1)%3]}
	inside, err = lp.DirectSectionLineContainsPoints(ctx, loftBody, 0, segment, loftPoints, budget)
	if err != nil || !inside {
		return false, err
	}
	tri := targetBool.Tris[targetFacet]
	targetPoints := [2]r3.Vec{targetBool.Verts[tri[targetEdge]], targetBool.Verts[tri[(targetEdge+1)%3]]}
	if err := budget.ReserveBytes(64 << 10); err != nil {
		return false, err
	}
	if err := budget.Step(); err != nil {
		return false, err
	}
	result := collinearSegmentsOverlap(targetPoints, loftPoints)
	if err := budget.Err(); err != nil {
		return false, err
	}
	return result, nil
}

// The caller proves the two finite segments are collinear and nondegenerate.
func collinearSegmentsOverlap(a, b [2]r3.Vec) bool {
	xa, xb := [2]proof.Xpt{proof.XptOf(a[0]), proof.XptOf(a[1])}, [2]proof.Xpt{proof.XptOf(b[0]), proof.XptOf(b[1])}
	var first r3.Vec
	found := false
	for i, p := range [4]r3.Vec{a[0], a[1], b[0], b[1]} {
		other := xb
		if i >= 2 {
			other = xa
		}
		if !meshbool.PointOnSegment3D(other[0], other[1], proof.XptOf(p)) {
			continue
		}
		if found && p != first {
			return true
		}
		first, found = p, true
	}
	return false
}

// directLoftTargetFaceMeetsCapAtEdge proves that an exact target face lies on
// one side of the loft's first-section plane and meets that plane only on the
// shared retained edge. It does not classify the loft cap or Boolean contact.
func directLoftTargetFaceMeetsCapAtEdge(ctx context.Context, target *topo.Mesh, targetBool *meshbool.BoolMesh,
	targetFacet, targetEdge int, loftBody *topo.Body, loftMesh *topo.Mesh, loftBool *meshbool.BoolMesh,
	loftFacet, loftEdge int, budget *proofbound.WorkBudget) (bool, error) {
	shared, err := directLoftSharedEdge(ctx, target, targetBool, targetFacet, targetEdge,
		loftBody, loftMesh, loftBool, loftFacet, loftEdge, budget)
	if err != nil || !shared {
		return false, err
	}
	face := target.Source[targetFacet]
	bound, ok := target.FaceBound[face]
	if !ok || bound != 0 {
		return false, nil
	}
	edge, err := meshbool.RetainedEdgeAtMeshEdge(ctx, target, targetBool, targetFacet, targetEdge, budget)
	if err != nil || edge == nil {
		return false, err
	}
	lp, ok := loftBody.Payload.(loftrecord.Payload)
	if !ok {
		return false, nil
	}
	frame := lp.Frame0
	if !frame.IsValid() || !proofbound.FiniteVec(frame.Origin()) || !proofbound.FiniteVec(frame.U()) ||
		!proofbound.FiniteVec(frame.V()) || len(target.Source) != len(target.Triangles) {
		return false, nil
	}
	origin := proof.DyVec(frame.Origin())
	normal := proof.DvCross(proof.DyVec(frame.U()), proof.DyVec(frame.V()))
	if proof.DvDot(normal, normal).Sign() == 0 {
		return false, nil
	}
	xa, xb := proof.XptOf(edge.Start.Position), proof.XptOf(edge.End.Position)
	direction := proof.Xsub(xb, xa)
	side, on := 0, 0
	for i, src := range target.Source {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if src != face {
			continue
		}
		if i >= len(targetBool.Tris) || target.Triangles[i] != targetBool.Tris[i] {
			return false, nil
		}
		if err := budget.ReserveBytes(32 << 10); err != nil {
			return false, err
		}
		for _, v := range target.Triangles[i] {
			if v < 0 || v >= len(target.Vertices) || v >= len(targetBool.Verts) ||
				target.Vertices[v] != targetBool.Verts[v] || !proofbound.FiniteVec(target.Vertices[v]) {
				return false, nil
			}
			point := target.Vertices[v]
			s := proof.DvDot(normal, proof.DvSub(proof.DyVec(point), origin)).Sign()
			if s == 0 {
				x := proof.XptOf(point)
				cross := proof.Xcross(direction, proof.Xsub(x, xa))
				if cross.X.Sign() != 0 || cross.Y.Sign() != 0 || cross.Z.Sign() != 0 ||
					!meshbool.PointOnSegment3D(xa, xb, x) {
					return false, nil
				}
				on++
			} else if side == 0 {
				side = s
			} else if side != s {
				return false, nil
			}
		}
	}
	if err := budget.Err(); err != nil {
		return false, err
	}
	return side != 0 && on >= 2, nil
}

// directLoftCapEdgeSupport checks the whole original start-cap mesh and the
// source boundary against the same complete held edge. It proves their support
// relationship, not the adjacent walls or a Boolean admission.
func directLoftCapEdgeSupport(ctx context.Context, target *topo.Mesh, targetBool *meshbool.BoolMesh,
	targetFacet, targetEdge int, loftBody *topo.Body, loftMesh *topo.Mesh, loftBool *meshbool.BoolMesh,
	loftFacet, loftEdge int, budget *proofbound.WorkBudget) (bool, error) {
	matched, err := directLoftTargetFaceMeetsCapAtEdge(ctx, target, targetBool, targetFacet, targetEdge,
		loftBody, loftMesh, loftBool, loftFacet, loftEdge, budget)
	if err != nil || !matched {
		return false, err
	}
	return directLoftCapSideSupport(ctx, loftBody, loftMesh, loftBool, loftFacet, loftEdge, budget)
}

// directLoftCapSideSupport proves that the original start-cap mesh and the
// source boundary lie on one side of the loft wall edge's complete source
// line, so the cap's true half-plane at the edge points where the held cap
// facets do. It needs no target face.
func directLoftCapSideSupport(ctx context.Context, loftBody *topo.Body, loftMesh *topo.Mesh,
	loftBool *meshbool.BoolMesh, loftFacet, loftEdge int, budget *proofbound.WorkBudget) (bool, error) {
	lp, ok := loftBody.Payload.(loftrecord.Payload)
	if !ok {
		return false, nil
	}
	wall := loftBool.Tris[loftFacet]
	points := [2]r3.Vec{loftBool.Verts[wall[loftEdge]], loftBool.Verts[wall[(loftEdge+1)%3]]}
	cell, side := lp.Cell[loftFacet][1], int(lp.Side[loftFacet])
	var prepared [3]r3.Vec
	for i, v := range wall {
		prepared[i] = loftBool.Verts[v]
	}
	segment, ok := lp.DirectWallFacetSourceSegment(loftBody, loftMesh, loftFacet, cell, side, prepared)
	if !ok {
		return false, nil
	}
	matched, err := lp.DirectSectionLineMatchesPoints(ctx, loftBody, 0, segment, points, budget)
	if err != nil || !matched {
		return false, err
	}
	matched, err = lp.DirectSectionLineSupportsBoundary(ctx, loftBody, 0, segment, budget)
	if err != nil || !matched {
		return false, err
	}
	start, end, ok := lp.DirectCapFacetRange(loftBody, 0)
	if !ok || start < 0 || end > len(loftMesh.Triangles) || end > len(loftBool.Tris) ||
		len(loftMesh.Source) != len(loftMesh.Triangles) {
		return false, nil
	}
	capFace := loftMesh.Source[start]
	if !lp.DirectCapFaceMatches(loftBody, capFace, 0) {
		return false, nil
	}
	frame := lp.Frame0
	origin := proof.DyVec(frame.Origin())
	normal := proof.DvCross(proof.DyVec(frame.U()), proof.DyVec(frame.V()))
	a, b := proof.DyVec(points[0]), proof.DyVec(points[1])
	direction := proof.DvSub(b, a)
	supportNormal := proof.DvCross(normal, direction)
	if proof.DvDot(supportNormal, supportNormal).Sign() == 0 {
		return false, nil
	}
	xa, xb := proof.XptOf(points[0]), proof.XptOf(points[1])
	meshSide, on := 0, 0
	for i, face := range loftMesh.Source {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if i < start || i >= end {
			if face == capFace {
				return false, nil
			}
			continue
		}
		if loftMesh.Triangles[i] != loftBool.Tris[i] {
			return false, nil
		}
		if err := budget.ReserveBytes(32 << 10); err != nil {
			return false, err
		}
		for j, v := range loftBool.Tris[i] {
			if v < 0 || v >= len(loftBool.Verts) || !proofbound.FiniteVec(loftBool.Verts[v]) {
				return false, nil
			}
			prepared[j] = loftBool.Verts[v]
		}
		if !lp.DirectCapFacetMatches(loftBody, loftMesh, i, 0, prepared) {
			return false, nil
		}
		for _, point := range prepared {
			p := proof.DyVec(point)
			if proof.DvDot(normal, proof.DvSub(p, origin)).Sign() != 0 {
				return false, nil
			}
			s := proof.DvDot(supportNormal, proof.DvSub(p, a)).Sign()
			if s == 0 {
				if !meshbool.PointOnSegment3D(xa, xb, proof.XptOf(point)) {
					return false, nil
				}
				on++
			} else if meshSide == 0 {
				meshSide = s
			} else if meshSide != s {
				return false, nil
			}
		}
	}
	if err := budget.Err(); err != nil {
		return false, err
	}
	return meshSide != 0 && on >= 2, nil
}

// directLoftCapContactEvidence supplies the proximity gate with one narrow
// source proof. It is only used for an unshifted ordinary Cut. The mesh pass
// must still classify every contact, including the edge's incident walls.
func directLoftCapContactEvidence(ctx context.Context, target *topo.Mesh, targetBool *meshbool.BoolMesh,
	loftBody *topo.Body, loftMesh *topo.Mesh, loftBool *meshbool.BoolMesh) func(*topo.Face, *topo.Face) (bool, error) {
	lp, ok := loftBody.Payload.(loftrecord.Payload)
	if !ok || !lp.MatchesDirectSections(loftBody) || target == nil || targetBool == nil || loftMesh == nil || loftBool == nil {
		return nil
	}
	budget := proofbound.NewStrictReplayBudget().WorkBudget(ctx)
	return func(targetFace, loftFace *topo.Face) (bool, error) {
		if targetFace == nil || !lp.DirectCapFaceMatches(loftBody, loftFace, 0) || !target.BoundaryOK ||
			len(target.Source) != len(target.Triangles) || len(loftMesh.Source) != len(loftMesh.Triangles) {
			return false, nil
		}
		bound, ok := target.FaceBound[targetFace]
		if !ok || bound != 0 {
			return false, nil
		}
		for i, face := range target.Source {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if face != targetFace || i >= len(targetBool.Tris) || target.Triangles[i] != targetBool.Tris[i] {
				continue
			}
			for k := range 3 {
				tri := targetBool.Tris[i]
				u, v := tri[k], tri[(k+1)%3]
				if u < 0 || v < 0 || u >= len(targetBool.Verts) || v >= len(targetBool.Verts) {
					continue
				}
				targetPoints := [2]r3.Vec{targetBool.Verts[u], targetBool.Verts[v]}
				if !proofbound.FiniteVec(targetPoints[0]) || !proofbound.FiniteVec(targetPoints[1]) {
					continue
				}
				for j := range lp.Cell {
					if err := budget.Step(); err != nil {
						return false, err
					}
					if j >= len(loftBool.Tris) {
						return false, nil
					}
					for e := range 3 {
						twin, ok := loftBool.TwinFacet(j, e)
						if !ok || twin < 0 || twin >= len(loftMesh.Source) || loftMesh.Source[twin] != loftFace {
							continue
						}
						wall := loftBool.Tris[j]
						x, y := wall[e], wall[(e+1)%3]
						if x < 0 || y < 0 || x >= len(loftBool.Verts) || y >= len(loftBool.Verts) {
							continue
						}
						loftPoints := [2]r3.Vec{loftBool.Verts[x], loftBool.Verts[y]}
						if !proofbound.FiniteVec(loftPoints[0]) || !proofbound.FiniteVec(loftPoints[1]) ||
							!directEdgeBoxesOverlap(targetPoints, loftPoints) {
							continue
						}
						matched, err := directLoftCapEdgeSupport(ctx, target, targetBool, i, k,
							loftBody, loftMesh, loftBool, j, e, budget)
						if err != nil || matched {
							return matched, err
						}
					}
				}
			}
		}
		return false, budget.Err()
	}
}

func directEdgeBoxesOverlap(a, b [2]r3.Vec) bool {
	return min(a[0].X, a[1].X) <= max(b[0].X, b[1].X) && min(b[0].X, b[1].X) <= max(a[0].X, a[1].X) &&
		min(a[0].Y, a[1].Y) <= max(b[0].Y, b[1].Y) && min(b[0].Y, b[1].Y) <= max(a[0].Y, a[1].Y) &&
		min(a[0].Z, a[1].Z) <= max(b[0].Z, b[1].Z) && min(b[0].Z, b[1].Z) <= max(a[0].Z, a[1].Z)
}
