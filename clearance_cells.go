package decad

import (
	"cmp"
	"slices"

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

// The feature-pair cell kinds enumerate sorts by box distance.
const (
	cellFF uint8 = iota // a face × b face
	cellFE              // a face × b edge
	cellEF              // b face × a edge
	cellEE              // a edge × b edge
)

// featureCell is one face/edge cell queued for the sorted walk: its kind,
// the two feature indices, and the distance between the features' boxes.
type featureCell struct {
	lb   float64
	kind uint8
	i, j int
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
	check := func() error {
		if k.err != nil {
			return k.err
		}
		return budget.Step()
	}
	for _, va := range k.a.verts {
		for _, vb := range k.b.verts {
			if err := check(); err != nil {
				return nil, err
			}
			sink.CandidateDist(k.tol, 1, clearance.PointPointDist(va, vb), va, vb)
		}
	}
	for _, va := range k.a.verts {
		if err := check(); err != nil {
			return nil, err
		}
		if err := k.vertexTier(budget, va, k.b, sink); err != nil {
			return nil, err
		}
	}
	for _, vb := range k.b.verts {
		if err := check(); err != nil {
			return nil, err
		}
		if err := k.vertexTier(budget, vb, k.a, sink); err != nil {
			return nil, err
		}
	}
	cells, err := k.featureCells(budget)
	if err != nil {
		return nil, err
	}
	for _, c := range cells {
		if err := check(); err != nil {
			return nil, err
		}
		if sink.Pruned(c.lb) {
			continue
		}
		switch c.kind {
		case cellFF:
			k.ffCell(k.a.faces[c.i], k.b.faces[c.j], sink)
		case cellFE:
			k.feCell(k.a.faces[c.i], k.b.edges[c.j], sink)
		case cellEF:
			k.feCell(k.b.faces[c.i], k.a.edges[c.j], sink)
		default:
			k.eeCell(k.a.edges[c.i], k.b.edges[c.j], sink)
		}
	}
	if k.err != nil {
		return nil, k.err
	}
	if err := budget.Err(); err != nil {
		return nil, err
	}
	if k.clearanceRefused {
		sink.Unsure = true
	}
	return sink, nil
}

// featureCells queues every face × face, face × edge, edge × face and
// edge × edge cell with its box distance, sorted ascending. The sort is
// stable, so ties keep the queue's own fixed order.
func (k *pairKernel) featureCells(budget *proofbound.WorkBudget) ([]featureCell, error) {
	a, b := k.a, k.b
	n := len(a.faces)*(len(b.faces)+len(b.edges)) + len(b.faces)*len(a.edges) + len(a.edges)*len(b.edges)
	cells := make([]featureCell, 0, n)
	push := func(kind uint8, i, j int, boxA, boxB [2]r3.Vec) error {
		if err := budget.Step(); err != nil {
			return err
		}
		cells = append(cells, featureCell{lb: clearance.ClrBoxDist(boxA, boxB), kind: kind, i: i, j: j})
		return nil
	}
	for i, fa := range a.faces {
		for j, fb := range b.faces {
			if err := push(cellFF, i, j, fa.Box, fb.Box); err != nil {
				return nil, err
			}
		}
	}
	for i, fa := range a.faces {
		for j, eb := range b.edges {
			if err := push(cellFE, i, j, fa.Box, eb.Box); err != nil {
				return nil, err
			}
		}
	}
	for i, fb := range b.faces {
		for j, ea := range a.edges {
			if err := push(cellEF, i, j, fb.Box, ea.Box); err != nil {
				return nil, err
			}
		}
	}
	for i, ea := range a.edges {
		for j, eb := range b.edges {
			if err := push(cellEE, i, j, ea.Box, eb.Box); err != nil {
				return nil, err
			}
		}
	}
	slices.SortStableFunc(cells, func(x, y featureCell) int { return cmp.Compare(x.lb, y.lb) })
	return cells, nil
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
