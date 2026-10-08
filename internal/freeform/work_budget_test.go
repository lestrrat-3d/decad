package freeform_test

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/stretchr/testify/require"
)

// TestFreeformWorkRaisedLimitBinds pins the raised exact-rational ceiling
// docs/loft-gear-bounds-design.md §7 gives a loft's record counters: a fresh
// counter reads the default, RaiseLimit only ever raises it, charges then run
// past the default up to the raised limit and refuse one unit beyond it, and
// the refusal names the raised limit.
//
// A charge at FreeformCostCeiling, where CostAdd and CostMul saturate, refuses
// under the raised limit even with room to spare: it stands for an estimate of
// unknown size. Shown to fail first: without Step's FreeformCostCeiling test
// the saturated charge is admitted.
func TestFreeformWorkRaisedLimitBinds(t *testing.T) {
	t.Parallel()
	work := freeform.NewFreeformWork()
	require.Equal(t, freeform.FreeformWorkLimit, work.WorkLimit(), "a fresh counter reads the default ceiling")
	work.RaiseLimit(freeform.FreeformWorkLimit / 2)
	require.Equal(t, freeform.FreeformWorkLimit, work.WorkLimit(), "RaiseLimit never lowers the ceiling")

	raised := 3 * freeform.FreeformWorkLimit
	work.RaiseLimit(raised)
	require.Equal(t, raised, work.WorkLimit())
	for range 3 {
		require.NoError(t, work.Step(freeform.FreeformWorkLimit), "charges run past the default up to the raised ceiling")
	}
	require.Equal(t, raised, work.Spent)
	err := work.Step(1)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, fmt.Sprintf("fixed work budget of %d", raised))
	require.Equal(t, raised, work.Spent, "a refused counter reads its ceiling")

	saturated := freeform.CostMul(freeform.FreeformCostCeiling, 2)
	require.Equal(t, freeform.FreeformCostCeiling, saturated, "the cost arithmetic saturates at FreeformCostCeiling")
	for _, row := range []struct {
		charge uint64
		admit  bool
	}{{saturated, false}, {freeform.FreeformCostCeiling - 1, true}} {
		roomy := freeform.NewFreeformWork()
		roomy.RaiseLimit(raised)
		err = roomy.Step(row.charge)
		if row.admit {
			require.NoError(t, err, "the largest unsaturated charge fits the raised ceiling")
			continue
		}
		require.ErrorIs(t, err, decaderr.ErrUnsupported, "a saturated estimate refuses under any ceiling")
	}
}
