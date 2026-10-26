package boolchain_test

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type producer struct {
	op meshbool.OperationKind
	in []*topo.Body
}

type fakeSource map[*topo.Body]producer

func (f fakeSource) BooleanProducer(n *topo.Body) (meshbool.OperationKind, []*topo.Body, bool) {
	p, ok := f[n]
	return p.op, p.in, ok
}

func leaf() *topo.Body { return &topo.Body{} }

// result is a body eligible for expansion: a mesh Boolean result.
func result() *topo.Body { return &topo.Body{Payload: facetedrecord.Payload{}} }

func lits(c boolchain.Chain) []boolchain.Literal { return c.Literals }

func TestFlattenExpansionRules(t *testing.T) {
	t.Parallel()
	a, b, c, d := leaf(), leaf(), leaf(), leaf()

	t.Run("Cut of a Cut result", func(t *testing.T) {
		t.Parallel()
		n := result()
		chain, ok := boolchain.Flatten(meshbool.OpCut, n, c, fakeSource{n: {meshbool.OpCut, []*topo.Body{a, b}}})
		require.True(t, ok)
		require.False(t, chain.Disjunction)
		require.Equal(t, []boolchain.Literal{{Body: a}, {Body: b, Negated: true}, {Body: c, Negated: true}}, lits(chain))
	})
	t.Run("Intersect of an Intersect result", func(t *testing.T) {
		t.Parallel()
		n := result()
		chain, ok := boolchain.Flatten(meshbool.OpIntersect, n, c, fakeSource{n: {meshbool.OpIntersect, []*topo.Body{a, b}}})
		require.True(t, ok)
		require.Equal(t, []boolchain.Literal{{Body: a}, {Body: b}, {Body: c}}, lits(chain))
	})
	t.Run("Intersect of a Cut result", func(t *testing.T) {
		t.Parallel()
		n := result()
		chain, ok := boolchain.Flatten(meshbool.OpIntersect, n, c, fakeSource{n: {meshbool.OpCut, []*topo.Body{a, b}}})
		require.True(t, ok)
		require.Equal(t, []boolchain.Literal{{Body: a}, {Body: b, Negated: true}, {Body: c}}, lits(chain))
	})
	t.Run("Cut by a Union result negates every member", func(t *testing.T) {
		t.Parallel()
		n := result()
		chain, ok := boolchain.Flatten(meshbool.OpCut, a, n, fakeSource{n: {meshbool.OpUnion, []*topo.Body{b, c, d}}})
		require.True(t, ok)
		require.Equal(t, []boolchain.Literal{{Body: a}, {Body: b, Negated: true}, {Body: c, Negated: true},
			{Body: d, Negated: true}}, lits(chain))
	})
	t.Run("Union of Union results", func(t *testing.T) {
		t.Parallel()
		n, m := result(), result()
		chain, ok := boolchain.Flatten(meshbool.OpUnion, n, m, fakeSource{
			n: {meshbool.OpUnion, []*topo.Body{a, b}},
			m: {meshbool.OpUnion, []*topo.Body{c, d}},
		})
		require.True(t, ok)
		require.True(t, chain.Disjunction)
		require.Equal(t, []boolchain.Literal{{Body: a}, {Body: b}, {Body: c}, {Body: d}}, lits(chain))
	})
	t.Run("expansion is depth first", func(t *testing.T) {
		t.Parallel()
		inner, outer := result(), result()
		chain, ok := boolchain.Flatten(meshbool.OpCut, outer, d, fakeSource{
			outer: {meshbool.OpCut, []*topo.Body{inner, c}},
			inner: {meshbool.OpCut, []*topo.Body{a, b}},
		})
		require.True(t, ok)
		require.Equal(t, []boolchain.Literal{{Body: a}, {Body: b, Negated: true}, {Body: c, Negated: true},
			{Body: d, Negated: true}}, lits(chain))
	})
	t.Run("a Union result on the positive side stays a leaf", func(t *testing.T) {
		t.Parallel()
		n, m := result(), result()
		_, ok := boolchain.Flatten(meshbool.OpCut, n, c, fakeSource{n: {meshbool.OpUnion, []*topo.Body{a, b}}})
		require.False(t, ok)
		// The Cut result is a leaf inside a negative literal, but the Union beside it expands.
		chain, ok := boolchain.Flatten(meshbool.OpCut, a, m, fakeSource{
			m: {meshbool.OpUnion, []*topo.Body{n, b}},
			n: {meshbool.OpCut, []*topo.Body{c, d}},
		})
		require.True(t, ok)
		require.Equal(t, []boolchain.Literal{{Body: a}, {Body: n, Negated: true}, {Body: b, Negated: true}}, lits(chain))
	})
	t.Run("a Cut result inside a disjunction stays a leaf", func(t *testing.T) {
		t.Parallel()
		n := result()
		_, ok := boolchain.Flatten(meshbool.OpUnion, n, c, fakeSource{n: {meshbool.OpCut, []*topo.Body{a, b}}})
		require.False(t, ok)
	})
	t.Run("no expansion declines", func(t *testing.T) {
		t.Parallel()
		_, ok := boolchain.Flatten(meshbool.OpCut, a, b, fakeSource{})
		require.False(t, ok)
	})
}

func TestFlattenEligibility(t *testing.T) {
	t.Parallel()
	a, b, c := leaf(), leaf(), leaf()
	flatten := func(n *topo.Body) bool {
		_, ok := boolchain.Flatten(meshbool.OpCut, n, c, fakeSource{n: {meshbool.OpCut, []*topo.Body{a, b}}})
		return ok
	}

	require.True(t, flatten(result()))

	t.Run("an analytic payload is a leaf", func(t *testing.T) {
		t.Parallel()
		require.False(t, flatten(leaf()))
	})
	t.Run("a point-cone trim is a leaf", func(t *testing.T) {
		t.Parallel()
		n := &topo.Body{Payload: facetedrecord.Payload{PointCone: &facetedrecord.PointConeTrimRecord{}}}
		require.False(t, flatten(n))
	})
	for _, tc := range []struct {
		name   string
		code   opdiagnostic.Code
		change opdiagnostic.ChangeMechanism
		expand bool
	}{
		{"a gap shift", 0, opdiagnostic.ChangeGapShift, false},
		{"an embedding reach", 0, opdiagnostic.ChangeEmbeddingReach, false},
		{"a stitch merge", 0, opdiagnostic.ChangeStitchMerge, false},
		{"a chain allowance", opdiagnostic.CodeChainAllowanceApplied, opdiagnostic.ChangeUnknown, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			n := result()
			n.OperationWarnings = []opdiagnostic.Record{{Code: tc.code, Change: tc.change}}
			require.Equal(t, tc.expand, flatten(n))
		})
	}
	t.Run("a body with no producer is a leaf", func(t *testing.T) {
		t.Parallel()
		_, ok := boolchain.Flatten(meshbool.OpCut, result(), c, fakeSource{})
		require.False(t, ok)
	})
	t.Run("a Cut that records other than two inputs is a leaf", func(t *testing.T) {
		t.Parallel()
		n := result()
		_, ok := boolchain.Flatten(meshbool.OpCut, n, c, fakeSource{n: {meshbool.OpCut, []*topo.Body{a}}})
		require.False(t, ok)
	})
}

func TestFlattenLiteralLimit(t *testing.T) {
	t.Parallel()
	bodies := make([]*topo.Body, 8)
	for i := range bodies {
		bodies[i] = leaf()
	}
	n := result()
	src := fakeSource{n: {meshbool.OpUnion, bodies[:5]}}
	// Cut(a, Union(b1..b5)) has 6 literals: at the limit.
	chain, ok := boolchain.Flatten(meshbool.OpCut, bodies[7], n, src)
	require.True(t, ok)
	require.Len(t, chain.Literals, boolchain.MaxLiterals)

	m := result()
	src = fakeSource{m: {meshbool.OpUnion, bodies[:6]}}
	_, ok = boolchain.Flatten(meshbool.OpCut, bodies[7], m, src)
	require.False(t, ok, "seven literals decline")
}

func TestFlattenRejectsOtherOperations(t *testing.T) {
	t.Parallel()
	_, ok := boolchain.Flatten(meshbool.OperationKind(99), leaf(), leaf(), fakeSource{})
	require.False(t, ok)
}
