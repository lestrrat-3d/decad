package pointquery

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// historyProducer is one recorded Boolean producer in a historySource.
type historyProducer struct {
	op     meshbool.OperationKind
	inputs []*topo.Body
}

// historySource is a recorded history keyed by output body. It counts the
// lookups of each body so a test can see which operands a walk read.
type historySource struct {
	producers map[*topo.Body]historyProducer
	reads     map[*topo.Body]int
}

func newHistorySource() *historySource {
	return &historySource{producers: map[*topo.Body]historyProducer{}, reads: map[*topo.Body]int{}}
}

func (s *historySource) BooleanProducer(n *topo.Body) (meshbool.OperationKind, []*topo.Body, bool) {
	s.reads[n]++
	p, ok := s.producers[n]
	return p.op, p.inputs, ok
}

// result records a mesh Boolean result of op over inputs.
func (s *historySource) result(op meshbool.OperationKind, inputs ...*topo.Body) *topo.Body {
	n := &topo.Body{Payload: facetedrecord.Payload{}}
	s.producers[n] = historyProducer{op: op, inputs: inputs}
	return n
}

// cube returns the leaf [x0, x0+10] × [0, 10] × [0, 10]: the unit test square
// prism, translated by its placement.
func cube(t *testing.T, x0 float64) *topo.Body {
	t.Helper()
	pp := payloadOf(t, momentinput.Profile{Outer: squareLoop()})
	shift, err := r3.Translation(r3.NewVec(x0, 0, 0))
	require.NoError(t, err)
	pp.Xform = shift
	return &topo.Body{Payload: pp}
}

func requireBooleanLocations(t *testing.T, n *topo.Body, src *historySource, cases []prismCase) {
	t.Helper()
	requireLocationCases(t, cases, func(at r3.Vec) (Location, bool, error) {
		return LocateBoolean(t.Context(), at, n, src)
	})
}

// TestSetAlgebraTables checks meet, join and complement for every pair of
// answers against the tables of agent-docs/point-containment-design.md §8.3,
// written out here rather than derived from each other.
func TestSetAlgebraTables(t *testing.T) {
	t.Parallel()
	answers := []Location{Inside, OnBoundary, Outside, Undecided}
	wantMeet := [4][4]Location{
		{Inside, OnBoundary, Outside, Undecided},
		{OnBoundary, Undecided, Outside, Undecided},
		{Outside, Outside, Outside, Outside},
		{Undecided, Undecided, Outside, Undecided},
	}
	wantJoin := [4][4]Location{
		{Inside, Inside, Inside, Inside},
		{Inside, Undecided, OnBoundary, Undecided},
		{Inside, OnBoundary, Outside, Undecided},
		{Inside, Undecided, Undecided, Undecided},
	}
	wantComplement := [4]Location{Outside, OnBoundary, Inside, Undecided}
	for i, a := range answers {
		require.Equal(t, wantComplement[i], complement(a), "complement %v", a)
		for j, b := range answers {
			require.Equal(t, wantMeet[i][j], meet(a, b), "meet %v %v", a, b)
			require.Equal(t, wantJoin[i][j], join(a, b), "join %v %v", a, b)
		}
	}
}

// TestLocateBooleanCombines reads A = [0, 10]³ and B = A moved 5 along x
// under each operation. Each expected answer names the point's class in A
// and in B.
func TestLocateBooleanCombines(t *testing.T) {
	t.Parallel()
	t.Run("union", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		n := src.result(meshbool.OpUnion, cube(t, 0), cube(t, 5))
		requireBooleanLocations(t, n, src, []prismCase{
			{name: "in A only", at: r3.NewVec(2, 5, 5), want: Inside},
			{name: "in B only", at: r3.NewVec(12, 5, 5), want: Inside},
			{name: "A's face inside B", at: r3.NewVec(10, 5, 5), want: Inside},
			{name: "A's face outside B", at: r3.NewVec(0, 5, 5), want: OnBoundary},
			{name: "B's face outside A", at: r3.NewVec(15, 5, 5), want: OnBoundary},
			{name: "both tops", at: r3.NewVec(7, 5, 10), want: Undecided},
			{name: "beyond both", at: r3.NewVec(16, 5, 5), want: Outside},
		})
	})
	t.Run("intersect", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		n := src.result(meshbool.OpIntersect, cube(t, 0), cube(t, 5))
		requireBooleanLocations(t, n, src, []prismCase{
			{name: "in both", at: r3.NewVec(7, 5, 5), want: Inside},
			{name: "B's face inside A", at: r3.NewVec(5, 5, 5), want: OnBoundary},
			{name: "A's face inside B", at: r3.NewVec(10, 5, 5), want: OnBoundary},
			{name: "in A only", at: r3.NewVec(2, 5, 5), want: Outside},
			{name: "both tops", at: r3.NewVec(7, 5, 10), want: Undecided},
		})
	})
	t.Run("cut", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		n := src.result(meshbool.OpCut, cube(t, 0), cube(t, 5))
		requireBooleanLocations(t, n, src, []prismCase{
			{name: "in A only", at: r3.NewVec(2, 5, 5), want: Inside},
			{name: "in both", at: r3.NewVec(7, 5, 5), want: Outside},
			{name: "B's face inside A", at: r3.NewVec(5, 5, 5), want: OnBoundary},
			{name: "A's face outside B", at: r3.NewVec(0, 5, 5), want: OnBoundary},
			{name: "in B only", at: r3.NewVec(12, 5, 5), want: Outside},
			{name: "both bottoms", at: r3.NewVec(2, 5, 0), want: OnBoundary},
			{name: "A's top on B's face", at: r3.NewVec(5, 5, 10), want: Undecided},
		})
	})
}

// TestLocateBooleanWalksHistory reads a three-input Union, a Cut over a
// Union, and the stops that skip operands.
func TestLocateBooleanWalksHistory(t *testing.T) {
	t.Parallel()
	t.Run("three-input union", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		n := src.result(meshbool.OpUnion, cube(t, 0), cube(t, 20), cube(t, 40))
		requireBooleanLocations(t, n, src, []prismCase{
			{name: "in the last", at: r3.NewVec(45, 5, 5), want: Inside},
			{name: "on the last", at: r3.NewVec(50, 5, 5), want: OnBoundary},
			{name: "between", at: r3.NewVec(15, 5, 5), want: Outside},
		})
	})
	t.Run("cut over a union", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		joined := src.result(meshbool.OpUnion, cube(t, 0), cube(t, 20))
		n := src.result(meshbool.OpCut, joined, cube(t, 5))
		requireBooleanLocations(t, n, src, []prismCase{
			{name: "tool inside the first", at: r3.NewVec(7, 5, 5), want: Outside},
			{name: "tool face inside the first", at: r3.NewVec(5, 5, 5), want: OnBoundary},
			{name: "second beyond the tool", at: r3.NewVec(25, 5, 5), want: Inside},
			{name: "gap", at: r3.NewVec(17, 5, 5), want: Outside},
		})
	})
	t.Run("a cut reads no tool outside its target", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		tool := src.result(meshbool.OpUnion, cube(t, 0), cube(t, 5))
		n := src.result(meshbool.OpCut, cube(t, 20), tool)
		requireBooleanLocations(t, n, src, []prismCase{{name: "outside the target", at: r3.NewVec(5, 5, 5), want: Outside}})
		require.Zero(t, src.reads[tool])
		requireBooleanLocations(t, n, src, []prismCase{{name: "inside the target", at: r3.NewVec(25, 5, 5), want: Inside}})
		require.Equal(t, 1, src.reads[tool])
	})
	t.Run("a union stops inside", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		second := src.result(meshbool.OpUnion, cube(t, 20), cube(t, 40))
		n := src.result(meshbool.OpUnion, cube(t, 0), second)
		requireBooleanLocations(t, n, src, []prismCase{{name: "inside the first", at: r3.NewVec(5, 5, 5), want: Inside}})
		require.Zero(t, src.reads[second])
	})
	t.Run("an intersect stops outside", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		second := src.result(meshbool.OpUnion, cube(t, 0), cube(t, 5))
		n := src.result(meshbool.OpIntersect, cube(t, 20), second)
		requireBooleanLocations(t, n, src, []prismCase{{name: "outside the first", at: r3.NewVec(5, 5, 5), want: Outside}})
		require.Zero(t, src.reads[second])
	})
}

// TestLocateBooleanDeclines covers each body the walk does not combine. Every
// point lies inside both cubes, so a combining walk would read it inside.
func TestLocateBooleanDeclines(t *testing.T) {
	t.Parallel()
	inside := []prismCase{{name: "inside both", at: r3.NewVec(7, 5, 5), want: Undecided}}
	for name, adjust := range map[string]func(n *topo.Body){
		"point-cone record": func(n *topo.Body) {
			n.Payload = facetedrecord.Payload{PointCone: &facetedrecord.PointConeTrimRecord{}}
		},
		"gap shift": func(n *topo.Body) {
			n.OperationWarnings = []opdiagnostic.Record{{Code: opdiagnostic.CodeGapShiftApplied, Change: opdiagnostic.ChangeGapShift}}
		},
		"embedding reach": func(n *topo.Body) {
			n.OperationWarnings = []opdiagnostic.Record{{Code: opdiagnostic.CodeEmbeddingReachApplied, Change: opdiagnostic.ChangeEmbeddingReach}}
		},
		"analytic payload": func(n *topo.Body) { n.Payload = nil },
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			src := newHistorySource()
			n := src.result(meshbool.OpIntersect, cube(t, 0), cube(t, 5))
			adjust(n)
			requireBooleanLocations(t, n, src, inside)
		})
	}
	t.Run("a chain allowance moves nothing", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		n := src.result(meshbool.OpIntersect, cube(t, 0), cube(t, 5))
		n.OperationWarnings = []opdiagnostic.Record{{Code: opdiagnostic.CodeChainAllowanceApplied}}
		requireBooleanLocations(t, n, src, []prismCase{{name: "inside both", at: r3.NewVec(7, 5, 5), want: Inside}})
	})
	t.Run("no producer record", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		n := src.result(meshbool.OpIntersect, cube(t, 0), cube(t, 5))
		delete(src.producers, n)
		requireBooleanLocations(t, n, src, inside)
	})
	t.Run("a cut with one input", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		n := src.result(meshbool.OpCut, cube(t, 0))
		requireBooleanLocations(t, n, src, []prismCase{{name: "inside the target", at: r3.NewVec(2, 5, 5), want: Undecided}})
	})
	t.Run("an operand neither prism nor expandable", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		placed := &topo.Body{Payload: facetedrecord.Payload{}}
		n := src.result(meshbool.OpIntersect, cube(t, 0), placed)
		requireBooleanLocations(t, n, src, inside)
		u := src.result(meshbool.OpUnion, cube(t, 0), placed)
		requireBooleanLocations(t, u, src, []prismCase{
			{name: "inside the prism", at: r3.NewVec(7, 5, 5), want: Inside},
			{name: "outside the prism", at: r3.NewVec(17, 5, 5), want: Undecided},
		})
	})
	t.Run("a displaced prism operand", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		displaced := cube(t, 5)
		pp := displaced.Payload.(prismrecord.Payload)
		pp.SectionDelta = 1e-9
		displaced.Payload = pp
		n := src.result(meshbool.OpIntersect, cube(t, 0), displaced)
		requireBooleanLocations(t, n, src, inside)
	})
	t.Run("a canceled context", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		n := src.result(meshbool.OpIntersect, cube(t, 0), cube(t, 5))
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, decided, err := LocateBoolean(ctx, r3.NewVec(7, 5, 5), n, src)
		require.ErrorIs(t, err, context.Canceled)
		require.False(t, decided)
	})
}

// TestLocateBooleanStackedAndRevolveLeaves combines a stacked leaf and a
// revolve leaf with the unit cube. The bored slabs fill [0, 10]³ less a
// radius-2 pocket about (5, 5) over z ∈ [5, 10]; the revolve is a cylinder of
// radius 10 about the y axis over y ∈ [0, 20].
func TestLocateBooleanStackedAndRevolveLeaves(t *testing.T) {
	t.Parallel()
	t.Run("stacked target cut by a cube", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		n := src.result(meshbool.OpCut, &topo.Body{Payload: stackedOf(t, boredSlabs())}, cube(t, 8))
		requireBooleanLocations(t, n, src, []prismCase{
			{name: "pocket floor", at: r3.NewVec(5, 5, 5), want: OnBoundary},
			{name: "pocket wall", at: r3.NewVec(5, 7, 7), want: OnBoundary},
			{name: "cube face in the material", at: r3.NewVec(8, 5, 3), want: OnBoundary},
			{name: "inside the cube", at: r3.NewVec(9, 5, 3), want: Outside},
			{name: "under the pocket", at: r3.NewVec(5, 5, 3), want: Inside},
		})
	})
	t.Run("revolve united with a cube", func(t *testing.T) {
		t.Parallel()
		src := newHistorySource()
		n := src.result(meshbool.OpUnion, &topo.Body{Payload: revolveOf(t, cylinderMeridian())}, cube(t, 5))
		requireBooleanLocations(t, n, src, []prismCase{
			{name: "revolve wall outside the cube", at: r3.NewVec(0, 5, -10), want: OnBoundary},
			{name: "revolve wall inside the cube", at: r3.NewVec(6, 5, 8), want: Inside},
			{name: "axis", at: r3.NewVec(0, 15, 0), want: Inside},
			{name: "cube face beyond the revolve", at: r3.NewVec(15, 5, 5), want: OnBoundary},
		})
	})
}

func TestLocateBooleanDenotedPrism(t *testing.T) {
	src := newHistorySource()
	leaf := cube(t, 0)
	pp := leaf.Payload.(prismrecord.Payload)
	pp.Z1Delta = 1
	pp.Z1Denoted = prismplacement.DenoteLevel(big.NewRat(19, 2), 10, 1)
	leaf.Payload = pp
	n := src.result(meshbool.OpUnion, leaf, cube(t, 20))
	requireBooleanLocations(t, n, src, []prismCase{
		{"inside", r3.NewVec(5, 5, 9), Inside},
		{"exact cap", r3.NewVec(5, 5, 9.5), OnBoundary},
		{"held cap", r3.NewVec(5, 5, 10), Outside},
	})
}
