package decad

import (
	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/facepair"
	"github.com/lestrrat-3d/decad/internal/clearance/spine"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// This file is the candidate enumeration of docs/clearance-design.md §3 and
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

// cellSink is the clearance kernel's contribution accumulator.
type cellSink = clearance.CellSink

func newPruningSink(margin float64) *cellSink {
	return clearance.NewPruningSink(margin)
}

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
func (k *pairKernel) enumerate() (*cellSink, error) {
	return k.enumerateInto(newPruningSink(k.slack))
}

// enumerateInto runs enumerate's walk into the given sink. A zero-value sink
// never prunes, so it runs every cell.
func (k *pairKernel) enumerateInto(sink *cellSink) (*cellSink, error) {
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
func (k *pairKernel) ffCell(f, g *clearance.CFace, sink *cellSink) {
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

func (k *pairKernel) spineEngine() *spine.Engine {
	return &spine.Engine{Context: k.ctx, Tolerance: k.tol, Slack: k.slack}
}

func (k *pairKernel) captureSpine(e *spine.Engine) {
	if e.Err != nil {
		k.err = e.Err
	}
	k.clearanceRefused = k.clearanceRefused || e.Refused
}

func (k *pairKernel) pointCircleCrits(p, c, axis, refU, refV r3.Vec, rad float64, win clearance.AngWindow) ([]clearance.SpineCrit, bool) {
	e := k.spineEngine()
	out, ok := e.PointCircleCrits(p, c, axis, refU, refV, rad, win)
	k.captureSpine(e)
	return out, ok
}

func (k *pairKernel) circleCircleCrits(f, g *clearance.CFace) ([]clearance.SpineCrit, bool) {
	e := k.spineEngine()
	out, ok := e.CircleCircleCrits(f, g)
	k.captureSpine(e)
	return out, ok
}

func (k *pairKernel) planeCrossesRevolved(f, g *clearance.CFace, sink *cellSink) {
	cells := facepair.New(k.ctx, k.tol, k.slack)
	cells.PlaneCrossesRevolved(f, g, sink)
	k.captureFaceCells(cells)
}

func (k *pairKernel) planeSphere(f, g *clearance.CFace, sink *cellSink) {
	cells := facepair.New(k.ctx, k.tol, k.slack)
	cells.PlaneSphere(f, g, sink)
	k.captureFaceCells(cells)
}
