package loftrecord

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

type wallPlaneBound struct {
	plane surfacegeom.Plane
	delta float64
}

// StraightWallPlaneBound returns the normal departure proved for this native
// wall face and its original tag, before any Boolean trims the face.
func (p Payload) StraightWallPlaneBound(face *topo.Face, plane surfacegeom.Plane) (float64, bool) {
	bound, ok := p.wallPlanes[face]
	return bound.delta, ok && bound.plane == plane
}

// recordStraightWalls reads the cell correspondence directly from the pairing
// used to build these faces. A line-to-line cell denotes its two triangles.
// Even if the four exact corners are not coplanar, their convex hull contains
// both halves. Convexity bounds every point by the largest corner distance
// to the tag plus the assembly's certified corner displacement.
func recordStraightWalls(budget *proofbound.WorkBudget, pairs []loftmesh.LoopPair, a loftmesh.Assembly,
	walls []*topo.Face) (map[*topo.Face]wallPlaneBound, error) {
	var bounds map[*topo.Face]wallPlaneBound
	for k, wall := range walls {
		loop, cell := a.Cell[k][0], a.Cell[k][1]
		if !pairs[loop].Faceted[cell] {
			continue
		}
		if err := budget.SpendWork(4); err != nil {
			return nil, err
		}
		plane, ok := wall.Surface.(surfacegeom.Plane)
		if !ok {
			continue
		}
		low, high := a.VIdx[loop], a.WIdx[loop]
		next := (cell + 1) % len(low)
		corners := [4]r3.Vec{a.Verts[low[cell]], a.Verts[low[next]], a.Verts[high[cell]], a.Verts[high[next]]}
		delta, ok := straightWallDeparture(plane.Frame, corners, a.Delta)
		if ok {
			if bounds == nil {
				bounds = make(map[*topo.Face]wallPlaneBound)
			}
			if err := budget.ReserveBytes(256); err != nil {
				return nil, err
			}
			bounds[wall] = wallPlaneBound{plane, delta}
		}
	}
	return bounds, nil
}

func straightWallDeparture(frame r3.Frame, corners [4]r3.Vec, delta float64) (float64, bool) {
	normal := proof.DvCross(proof.DyVec(frame.U()), proof.DyVec(frame.V()))
	den := proof.DyZero()
	for _, coordinate := range normal {
		if abs := proof.DyAbs(coordinate); proof.DyCmp(abs, den) > 0 {
			den = abs
		}
	}
	if den.IsZero() {
		return 0, false
	}
	origin := proof.DyVec(frame.Origin())
	farthest := proof.DyZero()
	for _, corner := range corners {
		offset := proof.DvSub(proof.DyVec(corner), origin)
		if height := proof.DyAbs(proof.DvDot(normal, offset)); proof.DyCmp(height, farthest) > 0 {
			farthest = height
		}
	}
	// The largest absolute component is a lower bound on the normal length.
	height := proofbound.RatFloatUp(new(big.Rat).Quo(farthest.Rat(), den.Rat()))
	bound := proofbound.AbsSumUpper(height, delta)
	return bound, !proofbound.IsNonFinite(bound)
}
