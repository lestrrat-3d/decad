package capband_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// reflexAdmission runs the cap-loop chamfer admission over loop with the
// resolver reporting an offset connector arc at corner 2.
func reflexAdmission(t *testing.T, loop sectionrecord.LoopRecord, reflex bool) error {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	refusal, err := capband.OccupiedVolumeAdmission(budget, "cap-loop chamfer", []sectionrecord.LoopRecord{loop},
		map[int]bool{0: true}, map[int]bool{},
		func(int, sectionrecord.LoopRecord) ([]survey2d.SideWalk, func() ([]bool, error), error) {
			cl, err := capblend.OneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
			if err != nil {
				return nil, nil, err
			}
			return cl.Walks, func() ([]bool, error) {
				arcs := make([]bool, len(cl.Walks))
				arcs[2] = true
				return arcs, nil
			}, nil
		},
		func(_ int, walks []survey2d.SideWalk) ([]float64, float64) {
			out := make([]float64, len(walks))
			for i := range out {
				out[i] = 0.5
			}
			return out, 0
		}, true, reflex)
	require.NoError(t, err)
	return refusal
}

// TestAdmissionReflexCorner pins which class admits a reflex corner: a prism
// cap-loop chamfer, whose mesh charges the connector, passes reflex; every
// other band keeps the refusal and its text.
func TestAdmissionReflexCorner(t *testing.T) {
	t.Parallel()
	require.NoError(t, reflexAdmission(t, squareLoop(), true))
	require.ErrorContains(t, reflexAdmission(t, squareLoop(), false), "a reflex corner")
}

// TestNearTangentChargesSkipsReflexCorners pins that a corner the chamfer's
// connector replaces is not read as a near-tangent join: an arc join the
// near-tangent gate refuses is skipped when flagged reflex, and the gate still
// refuses it otherwise.
func TestNearTangentChargesSkipsReflexCorners(t *testing.T) {
	t.Parallel()
	// wedgeLoop's arc turns past the near-tangent limit at corners 1 and 2.
	loop := wedgeLoop(30, 0.001)
	budget := proofbound.NewWorkBudget(t.Context())
	cl, err := capblend.OneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
	require.NoError(t, err)
	amounts := make([]float64, len(cl.Walks))
	for i := range amounts {
		amounts[i] = 0.4
	}
	_, err = capband.NearTangentCharges(budget, loop, cl.Walks, amounts, 0, false, nil)
	require.Error(t, err, `the corner is refused when it is not a connector`)
	flags := make([]bool, len(cl.Walks))
	flags[1], flags[2] = true, true
	charges, err := capband.NearTangentCharges(budget, loop, cl.Walks, amounts, 0, false, flags)
	require.NoError(t, err)
	require.Len(t, charges, len(cl.Walks))
}
