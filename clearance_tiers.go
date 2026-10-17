package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/curvecells"
	"github.com/lestrrat-3d/decad/internal/clearance/tier"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file adapts internal clearance tiers to the pair kernel.

// feCell dispatches one face × edge pair through §4's curve-tier table.
func (k *pairKernel) feCell(f *clearance.CFace, e *clearance.CEdge, sink *cellSink) {
	cells := curvecells.New(k.ctx, k.tol, k.slack)
	cells.FaceEdge(f, e, sink)
	k.captureCurveCells(cells)
}

// eeCell dispatches one edge pair through §4's curve tiers.
func (k *pairKernel) eeCell(ea, eb *clearance.CEdge, sink *cellSink) {
	cells := curvecells.New(k.ctx, k.tol, k.slack)
	cells.EdgeEdge(ea, eb, sink)
	k.captureCurveCells(cells)
}

func (k *pairKernel) captureCurveCells(cells *curvecells.Kernel) {
	if err := cells.Err(); err != nil {
		k.err = err
	}
	k.clearanceRefused = k.clearanceRefused || cells.Refused()
}

// vertexTier runs one vertex (a topological vertex, a synthesized cone apex
// or a spindle axis-collapse point) against the other body's faces and
// edges — every cell closed form (§3's vertex tiers). It continues the
// enumerator's budget through every face, edge, and planar trim scan. A
// face or edge whose box lies beyond the sink's best upper bound is pruned
// (cellSink.Pruned).
func (k *pairKernel) vertexTier(budget *proofbound.WorkBudget, v r3.Vec, other *bodyGeom, sink *cellSink) error {
	cells := tier.New(k.ctx, k.tol, k.slack)
	err := cells.EnumerateVertex(budget, v, other.faces, other.edges, sink)
	k.captureTier(cells)
	return err
}

func (k *pairKernel) captureTier(cells *tier.Kernel) {
	if err := cells.Err(); err != nil {
		k.err = err
	}
	k.clearanceRefused = k.clearanceRefused || cells.Refused()
}

// rulingContactCertified scans the plane-cylinder and cylinder-cylinder
// face pairs for a §6 ruling certificate. The caller runs it only when both
// bodies' bodyGeom.delta are exactly zero, so every carrier value is the
// boundary it names and every comparison below is exact.
func (k *pairKernel) rulingContactCertified(ctx context.Context) (*clearance.RulingContact, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, fa := range k.a.faces {
		for _, fb := range k.b.faces {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			var ruling *clearance.RulingContact
			switch {
			case fa.Kind == clearance.CkPlane && fb.Kind == clearance.CkCylinder:
				ruling = k.planeCylinderRuling(ctx, fa, fb, k.a.body, k.b.body, false)
			case fa.Kind == clearance.CkCylinder && fb.Kind == clearance.CkPlane:
				ruling = k.planeCylinderRuling(ctx, fb, fa, k.b.body, k.a.body, true)
			case fa.Kind == clearance.CkCylinder && fb.Kind == clearance.CkCylinder:
				ruling = k.cylinderPairRuling(ctx, fa, fb)
			}
			if ruling != nil {
				return ruling, nil
			}
		}
	}
	return nil, ctx.Err()
}

func (k *pairKernel) planeCylinderRuling(ctx context.Context, plane, cyl *clearance.CFace,
	planeBody, cylBody *Body, cylinderFirst bool) *clearance.RulingContact {
	cells := tier.New(ctx, k.tol, k.slack)
	return cells.PlaneCylinderRuling(plane, cyl,
		func(dir r3.Vec) (float64, float64, bool) { return payloadExtent(ctx, planeBody, dir) },
		func(dir r3.Vec) (float64, float64, bool) { return payloadExtent(ctx, cylBody, dir) }, cylinderFirst)
}

func (k *pairKernel) cylinderPairRuling(ctx context.Context, ca, cb *clearance.CFace) *clearance.RulingContact {
	cells := tier.New(ctx, k.tol, k.slack)
	return cells.CylinderPairRuling(ca, cb,
		func(dir r3.Vec) (float64, float64, bool) { return payloadExtent(ctx, k.a.body, dir) },
		func(dir r3.Vec) (float64, float64, bool) { return payloadExtent(ctx, k.b.body, dir) })
}
