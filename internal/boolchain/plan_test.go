package boolchain_test

import (
	"context"
	"errors"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type call struct {
	op   meshbool.OperationKind
	x, y *topo.Body
}

// fakeSteps records calls. exactOK decides which Exact calls succeed; each success
// returns a fresh body recorded in made.
type fakeSteps struct {
	exactOK   func(op meshbool.OperationKind, x, y *topo.Body) bool
	fullOK    bool
	exactCall []call
	fullCall  []call
	made      []*topo.Body
	err       error
}

func (f *fakeSteps) steps() boolchain.Steps {
	return boolchain.Steps{
		Exact: func(_ context.Context, op meshbool.OperationKind, x, y *topo.Body) (*topo.Body, bool, error) {
			f.exactCall = append(f.exactCall, call{op, x, y})
			if f.err != nil {
				return nil, false, f.err
			}
			if !f.exactOK(op, x, y) {
				return nil, false, nil
			}
			n := leaf()
			f.made = append(f.made, n)
			return n, true, nil
		},
		Full: func(_ context.Context, op meshbool.OperationKind, x, y *topo.Body) (*topo.Body, bool, error) {
			f.fullCall = append(f.fullCall, call{op, x, y})
			if !f.fullOK {
				return nil, false, nil
			}
			n := leaf()
			f.made = append(f.made, n)
			return n, true, nil
		},
	}
}

func conj(first *topo.Body, rest ...boolchain.Literal) boolchain.Chain {
	return boolchain.Chain{Literals: append([]boolchain.Literal{{Body: first}}, rest...)}
}

func neg(b *topo.Body) boolchain.Literal { return boolchain.Literal{Body: b, Negated: true} }
func pos(b *topo.Body) boolchain.Literal { return boolchain.Literal{Body: b} }

func TestRunAbsorbOrderAndRestart(t *testing.T) {
	t.Parallel()
	a, b, c, d := leaf(), leaf(), leaf(), leaf()
	var f fakeSteps
	// Only the literal c is admitted against a; then b is admitted against the result.
	f.exactOK = func(op meshbool.OperationKind, x, y *topo.Body) bool {
		return (x == a && y == c) || (len(f.made) == 1 && x == f.made[0] && y == b)
	}
	f.fullOK = true
	got, ok, err := boolchain.Run(t.Context(), conj(a, neg(b), neg(c), pos(d)), f.steps())
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, []call{
		{meshbool.OpCut, a, b},
		{meshbool.OpCut, a, c},
		{meshbool.OpCut, f.made[0], b},
		{meshbool.OpIntersect, f.made[1], d},
	}, f.exactCall, "scan in order, restart after each success")
	require.Equal(t, []call{{meshbool.OpIntersect, f.made[1], d}}, f.fullCall)
	require.Same(t, f.made[2], got)
}

func TestRunMergesNegativesThenFull(t *testing.T) {
	t.Parallel()
	a, b, c := leaf(), leaf(), leaf()
	var f fakeSteps
	f.exactOK = func(op meshbool.OperationKind, x, y *topo.Body) bool { return op == meshbool.OpUnion }
	f.fullOK = true
	got, ok, err := boolchain.Run(t.Context(), conj(a, neg(b), neg(c)), f.steps())
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, f.fullCall, 1)
	require.Equal(t, call{meshbool.OpCut, a, f.made[0]}, f.fullCall[0])
	require.Same(t, f.made[1], got)
	require.Equal(t, []call{
		{meshbool.OpCut, a, b},
		{meshbool.OpCut, a, c},
		{meshbool.OpUnion, b, c},
		{meshbool.OpCut, a, f.made[0]},
	}, f.exactCall, "absorb misses, merge tries the first pair, absorb runs again")
}

func TestRunAllAbsorbedNeedsNoFull(t *testing.T) {
	t.Parallel()
	a, b := leaf(), leaf()
	f := fakeSteps{exactOK: func(meshbool.OperationKind, *topo.Body, *topo.Body) bool { return true }}
	got, ok, err := boolchain.Run(t.Context(), conj(a, neg(b)), f.steps())
	require.NoError(t, err)
	require.True(t, ok)
	require.Empty(t, f.fullCall)
	require.Same(t, f.made[0], got)
}

func TestRunNeverRepeatsAMissedKey(t *testing.T) {
	t.Parallel()
	a, b, c, d := leaf(), leaf(), leaf(), leaf()
	f := fakeSteps{exactOK: func(op meshbool.OperationKind, x, y *topo.Body) bool { return x == a && y == d }}
	_, _, err := boolchain.Run(t.Context(), conj(a, neg(b), neg(c), neg(d)), f.steps())
	require.NoError(t, err)
	seen := map[call]int{}
	for _, c := range f.exactCall {
		seen[c]++
		require.Equal(t, 1, seen[c], "call %v repeated", c)
	}
}

func TestRunDeclinesTwoPendingMixedLiterals(t *testing.T) {
	t.Parallel()
	a, b, c := leaf(), leaf(), leaf()
	f := fakeSteps{exactOK: func(meshbool.OperationKind, *topo.Body, *topo.Body) bool { return false }, fullOK: true}
	_, ok, err := boolchain.Run(t.Context(), conj(a, neg(b), pos(c)), f.steps())
	require.NoError(t, err)
	require.False(t, ok)
	require.Empty(t, f.fullCall)
}

func TestRunFullFailureDeclines(t *testing.T) {
	t.Parallel()
	a, b := leaf(), leaf()
	f := fakeSteps{exactOK: func(meshbool.OperationKind, *topo.Body, *topo.Body) bool { return false }}
	_, ok, err := boolchain.Run(t.Context(), conj(a, neg(b)), f.steps())
	require.NoError(t, err)
	require.False(t, ok)
	require.Len(t, f.fullCall, 1)
}

func TestRunDisjunction(t *testing.T) {
	t.Parallel()
	a, b, c := leaf(), leaf(), leaf()
	chain := boolchain.Chain{Disjunction: true, Literals: []boolchain.Literal{pos(a), pos(b), pos(c)}}

	t.Run("merge then Full", func(t *testing.T) {
		t.Parallel()
		f := fakeSteps{exactOK: func(op meshbool.OperationKind, x, y *topo.Body) bool { return x == a && y == c }, fullOK: true}
		_, ok, err := boolchain.Run(t.Context(), chain, f.steps())
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, []call{{meshbool.OpUnion, f.made[0], b}}, f.fullCall)
		require.Equal(t, []call{
			{meshbool.OpUnion, a, b},
			{meshbool.OpUnion, a, c},
			{meshbool.OpUnion, f.made[0], b},
		}, f.exactCall, "first pair first, restart after a merge")
	})
	t.Run("one item left", func(t *testing.T) {
		t.Parallel()
		f := fakeSteps{exactOK: func(meshbool.OperationKind, *topo.Body, *topo.Body) bool { return true }}
		got, ok, err := boolchain.Run(t.Context(), chain, f.steps())
		require.NoError(t, err)
		require.True(t, ok)
		require.Empty(t, f.fullCall)
		require.Same(t, f.made[len(f.made)-1], got)
	})
	t.Run("three items decline", func(t *testing.T) {
		t.Parallel()
		f := fakeSteps{exactOK: func(meshbool.OperationKind, *topo.Body, *topo.Body) bool { return false }, fullOK: true}
		_, ok, err := boolchain.Run(t.Context(), chain, f.steps())
		require.NoError(t, err)
		require.False(t, ok)
		require.Empty(t, f.fullCall)
	})
}

func TestRunReturnsCallbackErrorUnchanged(t *testing.T) {
	t.Parallel()
	boom := errors.New("boom")
	f := fakeSteps{err: boom}
	_, ok, err := boolchain.Run(t.Context(), conj(leaf(), neg(leaf())), f.steps())
	require.Equal(t, boom, err)
	require.False(t, ok)
	require.Len(t, f.exactCall, 1)
}

func TestRunCanceledContextCallsNothing(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	f := fakeSteps{exactOK: func(meshbool.OperationKind, *topo.Body, *topo.Body) bool { return true }, fullOK: true}
	_, ok, err := boolchain.Run(ctx, conj(leaf(), neg(leaf())), f.steps())
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
	require.Empty(t, f.exactCall)
	require.Empty(t, f.fullCall)
}

func TestRunChecksContextBeforeEachCallback(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithCancel(t.Context())
	a, b, c := leaf(), leaf(), leaf()
	calls := 0
	steps := boolchain.Steps{
		Exact: func(context.Context, meshbool.OperationKind, *topo.Body, *topo.Body) (*topo.Body, bool, error) {
			calls++
			cancel()
			return nil, false, nil
		},
		Full: func(context.Context, meshbool.OperationKind, *topo.Body, *topo.Body) (*topo.Body, bool, error) {
			calls++
			return leaf(), true, nil
		},
	}
	_, _, err := boolchain.Run(ctx, conj(a, neg(b), neg(c)), steps)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 1, calls)
}

func TestRunTwoPendingNegativeLiterals(t *testing.T) {
	t.Parallel()
	a, b, c := leaf(), leaf(), leaf()
	chain := conj(a, neg(b), neg(c))
	before := append([]boolchain.Literal(nil), chain.Literals...)
	f := fakeSteps{exactOK: func(meshbool.OperationKind, *topo.Body, *topo.Body) bool { return false }, fullOK: true}
	got, ok, err := boolchain.Run(t.Context(), chain, f.steps())
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, 2, len(f.fullCall))
	require.Equal(t, 2, len(f.made))
	require.Equal(t, meshbool.OpUnion, f.fullCall[0].op)
	require.True(t, f.fullCall[0].x == b && f.fullCall[0].y == c, "tool order changed")
	require.Equal(t, meshbool.OpCut, f.fullCall[1].op)
	require.True(t, f.fullCall[1].x == a && f.fullCall[1].y == f.made[0], "final cut operands changed")
	require.True(t, got == f.made[1], "the result must come from the final cut")
	for i, literal := range chain.Literals {
		require.True(t, literal.Body == before[i].Body, "input body %d changed", i)
		require.Equal(t, before[i].Negated, literal.Negated)
	}
}

func TestRunTwoNegativeAfterAnalyticAbsorb(t *testing.T) {
	t.Parallel()
	a, b, c, d := leaf(), leaf(), leaf(), leaf()
	f := fakeSteps{
		exactOK: func(op meshbool.OperationKind, x, y *topo.Body) bool {
			return op == meshbool.OpIntersect && x == a && y == d
		},
		fullOK: true,
	}
	got, ok, err := boolchain.Run(t.Context(), conj(a, pos(d), neg(b), neg(c)), f.steps())
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, 2, len(f.fullCall))
	require.Equal(t, 3, len(f.made))
	require.True(t, f.fullCall[0].x == b && f.fullCall[0].y == c)
	require.True(t, f.fullCall[1].x == f.made[0] && f.fullCall[1].y == f.made[1])
	require.True(t, got == f.made[2])
}

func TestRunNegativeFullStepsStayBounded(t *testing.T) {
	t.Parallel()
	for count := 2; count < boolchain.MaxLiterals; count++ {
		pending := make([]boolchain.Literal, count)
		for i := range pending {
			pending[i] = neg(leaf())
		}
		f := fakeSteps{exactOK: func(meshbool.OperationKind, *topo.Body, *topo.Body) bool { return false }, fullOK: true}
		got, ok, err := boolchain.Run(t.Context(), conj(leaf(), pending...), f.steps())
		require.NoError(t, err)
		if count == 2 {
			require.True(t, ok)
			require.Equal(t, 2, len(f.fullCall))
			continue
		}
		require.False(t, ok)
		require.True(t, got == nil)
		require.Zero(t, len(f.fullCall), "remaining negatives=%d", count)
	}
}

func TestRunDeclinesTwoPendingPositiveLiterals(t *testing.T) {
	t.Parallel()
	f := fakeSteps{exactOK: func(meshbool.OperationKind, *topo.Body, *topo.Body) bool { return false }, fullOK: true}
	got, ok, err := boolchain.Run(t.Context(), conj(leaf(), pos(leaf()), pos(leaf())), f.steps())
	require.NoError(t, err)
	require.False(t, ok)
	require.True(t, got == nil)
	require.Zero(t, len(f.fullCall))
}

func TestRunTwoNegativeFullFailures(t *testing.T) {
	t.Parallel()
	boom := errors.New("full step failed")
	for _, tc := range []struct {
		name string
		step int
		err  error
	}{
		{"union refusal", 1, nil}, {"union error", 1, boom},
		{"cut refusal", 2, nil}, {"cut error", 2, boom},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			a, b, c := leaf(), leaf(), leaf()
			calls := 0
			steps := boolchain.Steps{
				Exact: func(context.Context, meshbool.OperationKind, *topo.Body, *topo.Body) (*topo.Body, bool, error) {
					return nil, false, nil
				},
				Full: func(context.Context, meshbool.OperationKind, *topo.Body, *topo.Body) (*topo.Body, bool, error) {
					calls++
					if calls == tc.step {
						return nil, false, tc.err
					}
					return leaf(), true, nil
				},
			}
			got, ok, err := boolchain.Run(t.Context(), conj(a, neg(b), neg(c)), steps)
			require.False(t, ok)
			require.True(t, got == nil, "a failed plan returned a body")
			if tc.err == nil {
				require.NoError(t, err)
			} else {
				require.ErrorIs(t, err, tc.err)
			}
			require.Equal(t, tc.step, calls)
		})
	}
}

func TestRunTwoNegativeFullCancellation(t *testing.T) {
	t.Parallel()
	for _, stop := range []int{1, 2} {
		ctx, cancel := context.WithCancel(t.Context())
		calls := 0
		steps := boolchain.Steps{
			Exact: func(context.Context, meshbool.OperationKind, *topo.Body, *topo.Body) (*topo.Body, bool, error) {
				return nil, false, nil
			},
			Full: func(ctx context.Context, _ meshbool.OperationKind, _, _ *topo.Body) (*topo.Body, bool, error) {
				calls++
				if calls == stop {
					cancel()
					if stop == 2 {
						// Production dispatch checks cancellation after each full step.
						return nil, false, ctx.Err()
					}
				}
				return leaf(), true, nil
			},
		}
		got, ok, err := boolchain.Run(ctx, conj(leaf(), neg(leaf()), neg(leaf())), steps)
		require.ErrorIs(t, err, context.Canceled)
		require.False(t, ok)
		require.True(t, got == nil)
		require.Equal(t, stop, calls)
		cancel()
	}
}
