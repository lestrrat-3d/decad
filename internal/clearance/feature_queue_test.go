package clearance_test

import (
	"cmp"
	"errors"
	"fmt"
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// legacyPruned is CellSink.Pruned as one function, so the legacy walk below
// does not lean on the method the new walk shares.
func legacyPruned(s *clearance.CellSink, lb float64) bool {
	if !s.Prune || proofbound.IsNonFinite(lb) {
		return false
	}
	for _, c := range s.Contribs[s.Seen:] {
		if c.Hi < s.Best {
			s.Best = c.Hi
		}
	}
	s.Seen = len(s.Contribs)
	if lb-s.Margin <= s.Best {
		return false
	}
	s.Skipped++
	return true
}

// legacyFeatureCells queues every face and edge pair and sorts the whole
// queue stably by box distance.
func legacyFeatureCells(budget *proofbound.WorkBudget, aFaces, bFaces []*clearance.CFace,
	aEdges, bEdges []*clearance.CEdge) ([]clearance.FeatureCell, error) {
	var cells []clearance.FeatureCell
	push := func(kind clearance.FeatureKind, i, j int, boxA, boxB [2]r3.Vec) error {
		if err := budget.Step(); err != nil {
			return err
		}
		cells = append(cells, clearance.FeatureCell{LowerBound: clearance.ClrBoxDist(boxA, boxB), Kind: kind, I: i, J: j})
		return nil
	}
	for i, fa := range aFaces {
		for j, fb := range bFaces {
			if err := push(clearance.FaceFace, i, j, fa.Box, fb.Box); err != nil {
				return nil, err
			}
		}
	}
	for i, fa := range aFaces {
		for j, eb := range bEdges {
			if err := push(clearance.FaceEdge, i, j, fa.Box, eb.Box); err != nil {
				return nil, err
			}
		}
	}
	for i, fb := range bFaces {
		for j, ea := range aEdges {
			if err := push(clearance.EdgeFace, i, j, fb.Box, ea.Box); err != nil {
				return nil, err
			}
		}
	}
	for i, ea := range aEdges {
		for j, eb := range bEdges {
			if err := push(clearance.EdgeEdge, i, j, ea.Box, eb.Box); err != nil {
				return nil, err
			}
		}
	}
	slices.SortStableFunc(cells, func(x, y clearance.FeatureCell) int { return cmp.Compare(x.LowerBound, y.LowerBound) })
	return cells, nil
}

// legacyEnumerateFeatures is EnumerateFeatures over the fully sorted queue.
func legacyEnumerateFeatures(budget *proofbound.WorkBudget, tolerance float64, a, b clearance.FeatureSet,
	sink *clearance.CellSink, visit clearance.FeatureVisitor) error {
	check := func() error {
		if err := visit.Err(); err != nil {
			return err
		}
		return budget.Step()
	}
	for _, va := range a.Vertices {
		for _, vb := range b.Vertices {
			if err := check(); err != nil {
				return err
			}
			sink.CandidateDist(tolerance, 1, clearance.PointPointDist(va, vb), va, vb)
		}
	}
	for _, va := range a.Vertices {
		if err := check(); err != nil {
			return err
		}
		if err := visit.Vertex(0, budget, va, sink); err != nil {
			return err
		}
	}
	for _, vb := range b.Vertices {
		if err := check(); err != nil {
			return err
		}
		if err := visit.Vertex(1, budget, vb, sink); err != nil {
			return err
		}
	}
	cells, err := legacyFeatureCells(budget, a.Faces, b.Faces, a.Edges, b.Edges)
	if err != nil {
		return err
	}
	for _, cell := range cells {
		if err := check(); err != nil {
			return err
		}
		if legacyPruned(sink, cell.LowerBound) {
			continue
		}
		switch cell.Kind {
		case clearance.FaceFace:
			visit.FaceFace(a.Faces[cell.I], b.Faces[cell.J], sink)
		case clearance.FaceEdge:
			visit.FaceEdge(a.Faces[cell.I], b.Edges[cell.J], sink)
		case clearance.EdgeFace:
			visit.FaceEdge(b.Faces[cell.I], a.Edges[cell.J], sink)
		case clearance.EdgeEdge:
			visit.EdgeEdge(a.Edges[cell.I], b.Edges[cell.J], sink)
		}
	}
	if err := visit.Err(); err != nil {
		return err
	}
	return budget.Err()
}

// walkScene is one synthetic pair: features that carry boxes only, and the
// upper bound each visited cell contributes.
type walkScene struct {
	a, b       clearance.FeatureSet
	tolerance  float64
	prune      bool
	margin     float64
	failAt     int
	hiOf       map[[2]any]float64
	vertexHiOf map[r3.Vec]float64
}

func randomBox(rng *rand.Rand) [2]r3.Vec {
	// A coarse grid makes many box distances tie; an occasional infinite
	// corner puts a cell at +Inf.
	c := func() float64 { return float64(rng.IntN(12)) }
	lo := r3.NewVec(c(), c(), c())
	hi := lo.Add(r3.NewVec(float64(rng.IntN(3)), float64(rng.IntN(3)), float64(rng.IntN(3))))
	if rng.IntN(25) == 0 {
		lo.X, hi.X = math.Inf(1), math.Inf(1)
	}
	return [2]r3.Vec{lo, hi}
}

func randomWalkScene(rng *rand.Rand) walkScene {
	sc := walkScene{
		tolerance:  1e-9,
		prune:      rng.IntN(6) != 0,
		margin:     []float64{0, 1e-9, 0.5, 1}[rng.IntN(4)],
		hiOf:       map[[2]any]float64{},
		vertexHiOf: map[r3.Vec]float64{},
	}
	if rng.IntN(4) == 0 {
		sc.failAt = 1 + rng.IntN(400)
	}
	set := func() clearance.FeatureSet {
		var fs clearance.FeatureSet
		for range rng.IntN(7) {
			fs.Faces = append(fs.Faces, &clearance.CFace{Box: randomBox(rng)})
		}
		for range rng.IntN(13) {
			fs.Edges = append(fs.Edges, &clearance.CEdge{Box: randomBox(rng)})
		}
		for range rng.IntN(4) {
			v := r3.NewVec(float64(rng.IntN(40)), float64(rng.IntN(40)), float64(rng.IntN(40)))
			fs.Vertices = append(fs.Vertices, v)
			sc.vertexHiOf[v] = 2 + float64(rng.IntN(30))
		}
		return fs
	}
	sc.a, sc.b = set(), set()
	hi := func(box [2]r3.Vec, other [2]r3.Vec) float64 {
		switch rng.IntN(10) {
		case 0:
			return math.Inf(1)
		case 1:
			return math.NaN()
		}
		return clearance.ClrBoxDist(box, other) + float64(rng.IntN(4))
	}
	for _, fa := range sc.a.Faces {
		for _, fb := range sc.b.Faces {
			sc.hiOf[[2]any{fa, fb}] = hi(fa.Box, fb.Box)
		}
		for _, eb := range sc.b.Edges {
			sc.hiOf[[2]any{fa, eb}] = hi(fa.Box, eb.Box)
		}
	}
	for _, fb := range sc.b.Faces {
		for _, ea := range sc.a.Edges {
			sc.hiOf[[2]any{fb, ea}] = hi(fb.Box, ea.Box)
		}
	}
	for _, ea := range sc.a.Edges {
		for _, eb := range sc.b.Edges {
			sc.hiOf[[2]any{ea, eb}] = hi(ea.Box, eb.Box)
		}
	}
	return sc
}

// hasUnbounded reports whether any queued cell sits at +Inf.
func (sc walkScene) hasUnbounded() bool {
	var boxesA, boxesB [][2]r3.Vec
	for _, f := range sc.a.Faces {
		boxesA = append(boxesA, f.Box)
	}
	for _, f := range sc.b.Faces {
		boxesB = append(boxesB, f.Box)
	}
	for _, e := range sc.a.Edges {
		boxesA = append(boxesA, e.Box)
	}
	for _, e := range sc.b.Edges {
		boxesB = append(boxesB, e.Box)
	}
	for _, x := range boxesA {
		for _, y := range boxesB {
			if math.IsInf(clearance.ClrBoxDist(x, y), 1) {
				return true
			}
		}
	}
	return false
}

// walkRecord is what one walk did: each visit with the budget step it ran
// at, the walk's error and step count, and the sink it left.
type walkRecord struct {
	visits []string
	steps  int
	err    error
	sink   *clearance.CellSink
}

var errWalkBudget = errors.New("walk budget")

func (sc walkScene) run(walk func(*proofbound.WorkBudget, float64, clearance.FeatureSet, clearance.FeatureSet,
	*clearance.CellSink, clearance.FeatureVisitor) error) walkRecord {
	var rec walkRecord
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			rec.steps++
			if rec.steps == sc.failAt {
				return errWalkBudget
			}
			return nil
		},
		ErrFn: func() error { return nil },
	}
	sink := &clearance.CellSink{}
	if sc.prune {
		sink = clearance.NewPruningSink(sc.margin)
	}
	index := map[any]int{}
	for i, f := range sc.a.Faces {
		index[f] = i
	}
	for i, f := range sc.b.Faces {
		index[f] = 100 + i
	}
	for i, e := range sc.a.Edges {
		index[e] = 200 + i
	}
	for i, e := range sc.b.Edges {
		index[e] = 300 + i
	}
	cell := func(x, y any, s *clearance.CellSink) {
		rec.visits = append(rec.visits, fmt.Sprintf("%d×%d@%d", index[x], index[y], rec.steps))
		s.Contribs = append(s.Contribs, clearance.GapContrib{Lo: 1, Hi: sc.hiOf[[2]any{x, y}]})
	}
	visit := clearance.FeatureVisitor{
		Vertex: func(side int, budget *proofbound.WorkBudget, v r3.Vec, s *clearance.CellSink) error {
			rec.visits = append(rec.visits, fmt.Sprintf("v%d%v@%d", side, v, rec.steps))
			s.Contribs = append(s.Contribs, clearance.GapContrib{Lo: 1, Hi: sc.vertexHiOf[v]})
			return budget.Step()
		},
		FaceFace: func(a, b *clearance.CFace, s *clearance.CellSink) { cell(a, b, s) },
		FaceEdge: func(f *clearance.CFace, e *clearance.CEdge, s *clearance.CellSink) { cell(f, e, s) },
		EdgeEdge: func(a, b *clearance.CEdge, s *clearance.CellSink) { cell(a, b, s) },
		Err:      func() error { return nil },
	}
	rec.err = walk(budget, sc.tolerance, sc.a, sc.b, sink, visit)
	rec.sink = sink
	return rec
}

// TestEnumerateFeaturesMatchesFullSort walks seeded random scenes, many box
// distances tied, some at +Inf, with pruning on and off and the budget
// failing at random steps, through EnumerateFeatures and through the walk
// over the fully sorted queue, and requires the same visits at the same
// budget steps, the same error, the same step count and the same sink.
func TestEnumerateFeaturesMatchesFullSort(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(2617, 11))
	var visited, skipped, unbounded int
	for trial := range 3000 {
		sc := randomWalkScene(rng)
		want := sc.run(legacyEnumerateFeatures)
		got := sc.run(clearance.EnumerateFeatures)
		what := fmt.Sprintf("trial %d", trial)
		require.Equal(t, want.visits, got.visits, what)
		require.Equal(t, want.steps, got.steps, what)
		require.Equal(t, want.err, got.err, what)
		require.Equal(t, want.sink.Skipped, got.sink.Skipped, what)
		require.Equal(t, want.sink.Seen, got.sink.Seen, what)
		require.Equal(t, math.Float64bits(want.sink.Best), math.Float64bits(got.sink.Best), what)
		require.Equal(t, want.sink.Unsure, got.sink.Unsure, what)
		require.Len(t, got.sink.Contribs, len(want.sink.Contribs), what)
		for i, c := range want.sink.Contribs {
			require.Equal(t, math.Float64bits(c.Lo), math.Float64bits(got.sink.Contribs[i].Lo), "%s: contribution %d", what, i)
			require.Equal(t, math.Float64bits(c.Hi), math.Float64bits(got.sink.Contribs[i].Hi), "%s: contribution %d", what, i)
		}
		visited += len(want.visits)
		skipped += want.sink.Skipped
		if want.sink.Skipped > 0 && sc.hasUnbounded() {
			unbounded++
		}
	}
	require.Positive(t, visited)
	require.Positive(t, skipped)
	require.Positive(t, unbounded, "a walk that both prunes and holds a cell at +Inf")
}
