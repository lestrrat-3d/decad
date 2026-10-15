package topo_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

type bare struct{}

func (bare) Transform() r3.Transform { return r3.Identity() }
func (bare) Placed(context.Context, topo.Env, r3.Transform) (*topo.Body, error) {
	return &topo.Body{}, nil
}

type prover struct {
	bare
	answer bool
}

func (p prover) ProvesSimple(context.Context) bool { return p.answer }

// TestProvesSimpleAsksOnlyAnImplementer returns a SimpleProver's own answer
// and false for a nil payload and for one that does not implement it.
func TestProvesSimpleAsksOnlyAnImplementer(t *testing.T) {
	t.Parallel()
	require.True(t, topo.ProvesSimple(t.Context(), prover{answer: true}))
	require.False(t, topo.ProvesSimple(t.Context(), prover{answer: false}))
	require.False(t, topo.ProvesSimple(t.Context(), bare{}))
	require.False(t, topo.ProvesSimple(t.Context(), nil))
}
