package unionmany_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/unionmany"
	"github.com/stretchr/testify/require"
)

func prismChainInputs(t *testing.T, env topo.Env) []*topo.Body {
	t.Helper()
	build := func(ref provenance.Producer, x0, y0, x1, y1, height float64) *topo.Body {
		env.Ref = ref
		return rectangle(t, env, x0, y0, x1, y1, height)
	}
	return []*topo.Body{
		build(1, 0, 0, 4, 4, 2),
		build(2, 3, 1, 6, 3, 2),
		build(3, 1, 3.5, 2.5, 6, 2),
	}
}

type unusedOperandMesher struct{ t *testing.T }

func (m unusedOperandMesher) TessellateOperand(context.Context, *topo.Body, float64) (*topo.Mesh, bool, error) {
	m.t.Fatal("the prism pair chain must use the analytic dispatcher")
	return nil, false, nil
}

func prismChainHooks(t *testing.T) booleandispatch.Hooks {
	t.Helper()
	return booleandispatch.Hooks{
		Verified: unusedMesher{t},
		Mesher:   unusedOperandMesher{t},
		AsBooleanError: func(_ meshbool.OperationKind, err error) error {
			return err
		},
	}
}

func TestPrismChainRealPrisms(t *testing.T) {
	t.Parallel()
	env := topo.Env{Doc: &topo.Document{}, Ref: 11, Alloc: topotest.Alloc{}}
	operands := prismChainInputs(t, env)
	joined, span, matched, err := unionmany.PrismChain(t.Context(), env, operands, true,
		prismChainHooks(t), func(error) bool { return false })
	require.NoError(t, err)
	require.True(t, matched)
	require.Equal(t, provenance.Producer(2), span)
	require.True(t, joined.Doc == env.Doc)
	require.Equal(t, provenance.Producer(12), joined.Origin.Producer)
	require.InDelta(t, 46, joined.Volume.Value.Base(), joined.Volume.Bound.Base()+1e-12)
	require.Empty(t, joined.OperationWarnings)
	for i, operand := range operands {
		require.Equal(t, provenance.Producer(i+1), operand.Origin.Producer)
	}
	require.Equal(t, 32.0, operands[0].Volume.Value.Base())
	require.Equal(t, 12.0, operands[1].Volume.Value.Base())
	require.Equal(t, 7.5, operands[2].Volume.Value.Base())
}

func TestPrismChainEligibilityAndCancellation(t *testing.T) {
	t.Parallel()
	env := topo.Env{Doc: &topo.Document{}, Ref: 11, Alloc: topotest.Alloc{}}
	operands := prismChainInputs(t, env)
	hooks := prismChainHooks(t)
	surface := func(error) bool { return false }

	t.Run("an unsupported payload declines before dispatch", func(t *testing.T) {
		other := &topo.Body{Payload: nil}
		body, span, matched, err := unionmany.PrismChain(t.Context(), env,
			[]*topo.Body{operands[0], operands[1], other}, true, hooks, surface)
		require.NoError(t, err)
		require.False(t, matched)
		require.True(t, body == nil)
		require.Zero(t, span)
	})
	t.Run("a Loft without source authentication declines", func(t *testing.T) {
		other := &topo.Body{Payload: loftrecord.Payload{}}
		body, span, matched, err := unionmany.PrismChain(t.Context(), env,
			[]*topo.Body{operands[0], operands[1], other}, true, hooks, surface)
		require.NoError(t, err)
		require.False(t, matched)
		require.True(t, body == nil)
		require.Zero(t, span)
	})
	t.Run("cancellation stops an eligible chain", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		body, span, matched, err := unionmany.PrismChain(ctx, env, operands, true, hooks, surface)
		require.ErrorIs(t, err, context.Canceled)
		require.True(t, matched)
		require.True(t, body == nil)
		require.Zero(t, span)
	})
}
