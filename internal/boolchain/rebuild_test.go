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

// unionLog is a full step that records each call and returns a fresh body.
type unionLog struct {
	calls [][2]*topo.Body
	out   []*topo.Body
	ops   []meshbool.OperationKind
	miss  int // the 1-based call that misses; 0 for none
	err   error
}

func (u *unionLog) step(_ context.Context, op meshbool.OperationKind, x, y *topo.Body) (*topo.Body, bool, error) {
	u.calls = append(u.calls, [2]*topo.Body{x, y})
	u.ops = append(u.ops, op)
	if len(u.calls) == u.miss {
		return nil, false, u.err
	}
	n := leaf()
	u.out = append(u.out, n)
	return n, true, nil
}

// requireCall checks one recorded call's operands by identity: leaf bodies are
// all equal by value.
func requireCall(t *testing.T, call [2]*topo.Body, x, y *topo.Body) {
	t.Helper()
	require.Same(t, x, call[0])
	require.Same(t, y, call[1])
}

func TestRebuildUnion(t *testing.T) {
	t.Parallel()

	t.Run("repeats the recorded grouping depth first", func(t *testing.T) {
		t.Parallel()
		// a = Union(Union(p, q), r); b = Union(s, t, w) of three inputs, where w is
		// a mesh Boolean result whose producer is a Cut.
		p, q, r, s, tt, v := leaf(), leaf(), leaf(), leaf(), leaf(), leaf()
		pq, a, b, w := result(), result(), result(), result()
		src := fakeSource{
			pq: {meshbool.OpUnion, []*topo.Body{p, q}},
			a:  {meshbool.OpUnion, []*topo.Body{pq, r}},
			b:  {meshbool.OpUnion, []*topo.Body{s, tt, w}},
			w:  {meshbool.OpCut, []*topo.Body{v, p}},
		}
		var log unionLog
		got, ok, err := boolchain.RebuildUnion(t.Context(), a, b, src, log.step)
		require.NoError(t, err)
		require.True(t, ok)
		require.Len(t, log.calls, 5)
		for _, op := range log.ops {
			require.Equal(t, meshbool.OpUnion, op)
		}
		requireCall(t, log.calls[0], p, q)
		requireCall(t, log.calls[1], log.out[0], r)
		requireCall(t, log.calls[2], s, tt)
		requireCall(t, log.calls[3], log.out[2], w) // a Cut result is used as it is
		requireCall(t, log.calls[4], log.out[1], log.out[3])
		require.Same(t, log.out[4], got)
	})

	t.Run("declines when nothing expands", func(t *testing.T) {
		t.Parallel()
		a, b := result(), leaf()
		var log unionLog
		_, ok, err := boolchain.RebuildUnion(t.Context(), a, b, fakeSource{}, log.step)
		require.NoError(t, err)
		require.False(t, ok)
		require.Empty(t, log.calls)
	})

	t.Run("declines past MaxRebuildSteps before any call", func(t *testing.T) {
		t.Parallel()
		src := fakeSource{}
		acc := leaf()
		for range boolchain.MaxRebuildSteps {
			n := result()
			src[n] = producer{meshbool.OpUnion, []*topo.Body{acc, leaf()}}
			acc = n
		}
		var log unionLog
		_, ok, err := boolchain.RebuildUnion(t.Context(), acc, leaf(), src, log.step)
		require.NoError(t, err)
		require.False(t, ok)
		require.Empty(t, log.calls)

		under := src[acc].in[0]
		_, ok, err = boolchain.RebuildUnion(t.Context(), under, leaf(), src, log.step)
		require.NoError(t, err)
		require.True(t, ok)
		require.Len(t, log.calls, boolchain.MaxRebuildSteps)
	})

	t.Run("a miss or an error stops the rebuild", func(t *testing.T) {
		t.Parallel()
		p, q, r := leaf(), leaf(), leaf()
		a := result()
		src := fakeSource{a: {meshbool.OpUnion, []*topo.Body{p, q}}}
		miss := unionLog{miss: 1}
		_, ok, err := boolchain.RebuildUnion(t.Context(), a, r, src, miss.step)
		require.NoError(t, err)
		require.False(t, ok)
		require.Len(t, miss.calls, 1)

		boom := errors.New("boom")
		failing := unionLog{miss: 2, err: boom}
		_, ok, err = boolchain.RebuildUnion(t.Context(), a, r, src, failing.step)
		require.ErrorIs(t, err, boom)
		require.False(t, ok)
	})

	t.Run("a canceled context returns its error", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		a := result()
		var log unionLog
		_, ok, err := boolchain.RebuildUnion(ctx, a, leaf(), fakeSource{a: {meshbool.OpUnion,
			[]*topo.Body{leaf(), leaf()}}}, log.step)
		require.ErrorIs(t, err, context.Canceled)
		require.False(t, ok)
		require.Empty(t, log.calls)
	})
}
