package booleaneval_test

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

type neverMesher struct{ t *testing.T }

func (m neverMesher) TessellateOperand(context.Context, *topo.Body, float64) (*topo.Mesh, bool, error) {
	m.t.Fatal("a cancelled evaluation must not mesh an operand")
	return nil, false, nil
}

func TestEvaluateStopsOnACancelledContext(t *testing.T) {
	t.Parallel()

	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := booleaneval.Evaluate(cancelled, meshbool.OpUnion, &topo.Body{}, &topo.Body{}, neverMesher{t}, 1, nil, nil)
	require.ErrorIs(t, err, context.Canceled)
}

func TestVolumeReading(t *testing.T) {
	t.Parallel()

	audit := &facetproof.MeshAudit{CompVol: []*big.Rat{big.NewRat(3, 2), big.NewRat(1, 2)}}
	volume, bound, exact, err := booleaneval.VolumeReading(t.Context(), audit, 0.25)
	require.NoError(t, err)
	require.Equal(t, 2.0, volume)
	require.Equal(t, 0, exact.Cmp(big.NewRat(2, 1)))
	require.GreaterOrEqual(t, bound, 0.25, "the allowance is rounded up, never down")
	require.Less(t, bound, 0.26, "an exactly representable volume adds no rounding of its own")

	third := &facetproof.MeshAudit{CompVol: []*big.Rat{big.NewRat(1, 3)}}
	_, bound, _, err = booleaneval.VolumeReading(t.Context(), third, 0)
	require.NoError(t, err)
	require.Positive(t, bound, "a volume that is not a float carries its rounding")

	_, _, empty, err := booleaneval.VolumeReading(t.Context(), &facetproof.MeshAudit{}, 0)
	require.Nil(t, empty)
	require.ErrorIs(t, err, decaderr.ErrBooleanFailed)
	require.ErrorContains(t, err, "encloses no volume")

	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	_, _, none, err := booleaneval.VolumeReading(cancelled, audit, 0)
	require.Nil(t, none)
	require.ErrorIs(t, err, context.Canceled)
}

func TestPrepBoolMeshRefusesAnUnboundedVertex(t *testing.T) {
	t.Parallel()

	mesh := &topo.Mesh{
		Vertices:  []r3.Vec{{}, {X: 1}, {Y: math.Inf(1)}},
		Triangles: [][3]int{{0, 1, 2}},
	}
	_, err := booleaneval.PrepBoolMesh(t.Context(), mesh, []int{0})
	require.ErrorIs(t, err, decaderr.ErrBooleanFailed)
	require.ErrorContains(t, err, "not finite")
}

// lateCancel reports a cancelled context from its second Err call on.
type lateCancel struct {
	context.Context //nolint:containedctx // a test double that wraps a live context and cancels it on a later poll
	calls           int
}

func (c *lateCancel) Err() error {
	c.calls++
	if c.calls >= 2 {
		return context.Canceled
	}
	return nil
}

func TestVolumeReadingPollsAfterTheReading(t *testing.T) {
	t.Parallel()

	audit := &facetproof.MeshAudit{CompVol: []*big.Rat{big.NewRat(1, 1)}}
	ctx := &lateCancel{Context: t.Context()}
	_, _, rat, err := booleaneval.VolumeReading(ctx, audit, 0)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, rat)
	require.Equal(t, 2, ctx.calls)
}
