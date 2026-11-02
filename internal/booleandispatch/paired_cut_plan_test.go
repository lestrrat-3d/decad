package booleandispatch_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestRunPairedCutPlanSequenceAndCancellation(t *testing.T) {
	t.Parallel()
	for _, second := range []bool{false, true} {
		for _, cancelAfter := range []int{0, 1, 3, 5, -1} {
			steps := 3
			if second {
				steps = 5
			}
			if cancelAfter > steps {
				continue
			}
			var nodes [9]topo.Body
			sources := [2]*topo.Body{&nodes[0], &nodes[1]}
			firstTool := &nodes[2]
			var secondTool *topo.Body
			if second {
				secondTool = &nodes[3]
			}
			type step struct {
				op    meshbool.OperationKind
				x, y  *topo.Body
				scale float64
			}
			want := []step{{meshbool.OpCut, sources[0], firstTool, .25},
				{meshbool.OpCut, sources[1], firstTool, .25},
				{meshbool.OpUnion, &nodes[4], &nodes[5], 1}}
			if second {
				want = []step{{meshbool.OpCut, sources[0], firstTool, .25},
					{meshbool.OpCut, &nodes[4], secondTool, .125},
					{meshbool.OpCut, sources[1], firstTool, .0625},
					{meshbool.OpCut, &nodes[6], secondTool, .125},
					{meshbool.OpUnion, &nodes[5], &nodes[7], 1}}
			}
			ctx, cancel := context.WithCancel(t.Context())
			if cancelAfter == 0 {
				cancel()
			}
			calls := 0
			run := func(op meshbool.OperationKind, x, y *topo.Body, scale float64) (*topo.Body, error) {
				require.Less(t, calls, len(want))
				expected := want[calls]
				require.Equal(t, expected.op, op, "step %d operation", calls)
				require.True(t, expected.x == x && expected.y == y, "step %d operands", calls)
				require.Equal(t, expected.scale, scale, "step %d chord scale", calls)
				result := &nodes[4+calls]
				calls++
				if calls == cancelAfter {
					cancel()
				}
				return result, nil
			}
			result, err := booleandispatch.RunPairedCutPlan(ctx, sources, firstTool, secondTool, run)
			cancel()
			if cancelAfter >= 0 {
				require.ErrorIs(t, err, context.Canceled)
				require.Equal(t, cancelAfter, calls)
			} else {
				require.NoError(t, err)
				require.Equal(t, steps, calls)
				require.True(t, result == &nodes[3+steps])
			}
		}
	}
}
