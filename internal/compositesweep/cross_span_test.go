package compositesweep_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/stretchr/testify/require"
)

func TestCrossSpanContactAudit(t *testing.T) {
	triangles := [][3]int{{0, 1, 2}, {0, 2, 3}}
	t.Run("inconsistent records", func(t *testing.T) {
		err := compositesweep.CrossSpanContactAudit(proofbound.NewWorkBudget(t.Context()),
			make([]revolvemesh.RevolveAuditTri, 1), triangles, []int{0, 0}, 0)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "the composite sweep's facet audit has inconsistent triangle records")
	})
	t.Run("one span audits no pair", func(t *testing.T) {
		err := compositesweep.CrossSpanContactAudit(proofbound.NewWorkBudget(t.Context()),
			make([]revolvemesh.RevolveAuditTri, 2), triangles, []int{0, 0}, 0)
		require.NoError(t, err)
	})
}
