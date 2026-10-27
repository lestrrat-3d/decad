package pointquery

import (
	"context"
	"math"
	"math/rand/v2"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The private rational kernel is an independent reference for every region
// of the dyadic triangle and segment calculations.
func TestDyadicDistanceMatchesRationalAcrossExponents(t *testing.T) {
	rng := rand.New(rand.NewPCG(317, 331))
	for _, exponent := range []int{-1074, -1000, -500, -1, 0, 50, 500, 1000} {
		for sample := range 20 {
			vec := func() r3.Vec {
				coord := func() float64 { return math.Ldexp(float64(rng.IntN(17)-8), exponent) }
				return r3.NewVec(coord(), coord(), coord())
			}
			p, a, b, c := vec(), vec(), vec(), vec()
			if sample%5 == 0 {
				c = b
			}
			pd, ad, bd, cd := proof.DyVec(p), proof.DyVec(a), proof.DyVec(b), proof.DyVec(c)
			want, werr := triangleDistanceSquared(exactVec(p), exactVec(a), exactVec(b), exactVec(c))
			got, gerr := triangleDyDistanceSquared(pd, ad, bd, cd)
			require.Equal(t, werr == nil, gerr == nil, "exponent %d sample %d", exponent, sample)
			if werr == nil {
				require.Positive(t, got.d.Sign())
				require.Zero(t, got.rat().Cmp(want), "triangle exponent %d sample %d", exponent, sample)
			}
			edge := segmentDyDistanceSquared(pd, ad, bd)
			require.Positive(t, edge.d.Sign())
			require.Zero(t, edge.rat().Cmp(segmentDistanceSquared(exactVec(p), exactVec(a), exactVec(b))),
				"edge exponent %d sample %d", exponent, sample)
			box := boxDyDistanceSquared(pd, ad, bd, cd)
			require.Zero(t, box.Rat().Cmp(boxDistanceSquared(exactVec(p), exactVec(a), exactVec(b), exactVec(c))),
				"box exponent %d sample %d", exponent, sample)
		}
	}
}

type distanceCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation fixture for one test call.
	cancel          context.CancelFunc
	polls           atomic.Int32
	stop            int32
}

func (c *distanceCancelContext) Err() error {
	if c.polls.Add(1) >= c.stop {
		c.cancel()
	}
	return c.Context.Err()
}

func TestDyadicDistancePreservesEveryCancellationPoll(t *testing.T) {
	// There are three polls, one during vertex validation and two during
	// triangle traversal. Repeated triangles exercise the minimum's pruning.
	verts := make([]r3.Vec, 259)
	verts[1], verts[2] = r3.Vec{X: 1}, r3.Vec{Y: 1}
	tris := make([][3]int, 513)
	for i := range tris {
		tris[i] = [3]int{0, 1, 2}
	}
	for stop := int32(1); stop <= 4; stop++ {
		run := func(reference bool) (int32, error) {
			ctx, cancel := context.WithCancel(t.Context())
			defer cancel()
			counter := &distanceCancelContext{Context: ctx, cancel: cancel, stop: stop}
			fn := MeshDistanceSquared
			if reference {
				fn = eagerMeshDistanceSquared
			}
			_, err := fn(counter, r3.Vec{X: 0.25, Y: 0.25, Z: 1}, verts, tris)
			return counter.polls.Load(), err
		}
		want, werr := run(true)
		got, gerr := run(false)
		require.Equal(t, want, got, "stop %d", stop)
		if stop <= 3 {
			require.ErrorIs(t, werr, context.Canceled)
			require.ErrorIs(t, gerr, context.Canceled)
		} else {
			require.NoError(t, werr)
			require.NoError(t, gerr)
		}
	}
}
