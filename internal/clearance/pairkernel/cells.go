package pairkernel

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/curvecells"
	"github.com/lestrrat-3d/decad/internal/clearance/facepair"
	"github.com/lestrrat-3d/decad/internal/clearance/tier"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file is the candidate enumeration of agent-docs/clearance-design.md §3 and
// the face-pair table of §4: six stationarity tiers over unordered boundary
// pairs, candidates computed on the unbounded carriers and admitted only when
// both feet lie within their faces' trims. Three of the five surfaces are
// constant offsets of a spine, so their cells reduce to spine-pair criticals
// (closed form, or P4/P8 certified brackets) with the four offset
// combinations per critical; the plane row is elementary; cone-involved
// pairs and spindle-torus pairs are outside the certified-cell set and
// contribute a coarse conservative enclosure instead (§8 — proven disjoint with a wide
// honest row when even the coarse lower bound clears zero, undecided when it
// does not). A discarded candidate is a candidate whose feet provably leave
// the trims — a lower tier holds its minimum; a candidate whose admission is
// in doubt is kept for the lower bound and never counted toward exactness.
// It also adapts the internal clearance tiers to the pair kernel.

// enumerate runs every tier over the pair, pruning per §5. One shared budget
// bounds cancellation latency across the cell queue build, the outer
// candidate walk, and the nested work performed by a vertex tier.
//
// The order is chosen so good upper bounds arrive early, and it is fixed, so
// a replay prunes identically: the vertex × vertex distances first (cheap and
// exact), then the vertex tiers (closed form), then every face/edge cell in
// ascending box distance, ties in the fixed face × face, face × edge,
// edge × face, edge × edge order. Within that last walk a cell is pruned
// exactly when its box distance, less the margin, exceeds the final best
// upper bound: every cell nearer than it has already run, the one holding
// the best hi among them.
func (k *pairKernel) enumerate() (*clearance.CellSink, error) {
	sink := clearance.NewPruningSink(k.slack)
	budget := proofbound.NewWorkBudget(k.ctx)
	a := clearance.FeatureSet{Faces: k.a.faces, Edges: k.a.edges, Vertices: k.a.verts}
	b := clearance.FeatureSet{Faces: k.b.faces, Edges: k.b.edges, Vertices: k.b.verts}
	visit := clearance.FeatureVisitor{
		Vertex: func(side int, budget *proofbound.WorkBudget, v r3.Vec, sink *clearance.CellSink) error {
			other := k.b
			if side == 1 {
				other = k.a
			}
			return k.vertexTier(budget, v, other, sink)
		},
		FaceFace: k.ffCell,
		FaceEdge: k.feCell,
		EdgeEdge: k.eeCell,
		Err:      func() error { return k.err },
	}
	if err := clearance.EnumerateFeatures(budget, k.tol, a, b, sink, visit); err != nil {
		return nil, err
	}
	if k.clearanceRefused {
		sink.Unsure = true
	}
	return sink, nil
}

// ffCell dispatches one face pair through the §4 table.
func (k *pairKernel) ffCell(f, g *clearance.CFace, sink *clearance.CellSink) {
	cells := facepair.New(k.ctx, k.tol, k.slack)
	cells.FaceCell(f, g, sink)
	k.captureFaceCells(cells)
}

func (k *pairKernel) captureFaceCells(cells *facepair.Kernel) {
	if err := cells.Err(); err != nil {
		k.err = err
	}
	k.clearanceRefused = k.clearanceRefused || cells.Refused()
}

// feCell dispatches one face × edge pair through §4's curve-tier table.
func (k *pairKernel) feCell(f *clearance.CFace, e *clearance.CEdge, sink *clearance.CellSink) {
	cells := curvecells.New(k.ctx, k.tol, k.slack)
	cells.FaceEdge(f, e, sink)
	k.captureCurveCells(cells)
}

// eeCell dispatches one edge pair through §4's curve tiers.
func (k *pairKernel) eeCell(ea, eb *clearance.CEdge, sink *clearance.CellSink) {
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
// (clearance.CellSink.Pruned).
func (k *pairKernel) vertexTier(budget *proofbound.WorkBudget, v r3.Vec, other *bodyGeom, sink *clearance.CellSink) error {
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
// bodies' bodyGeom.carrierDelta are exactly zero, so every carrier value is
// the boundary it names and every comparison below is exact.
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
	planeBody, cylBody *topo.Body, cylinderFirst bool) *clearance.RulingContact {
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
