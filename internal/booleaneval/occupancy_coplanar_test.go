package booleaneval

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/meshbool"
)

// TestComposeOccupancyDropsCoplanar pins §6: a result whose pair the coplanar
// pass accepted a point of carries no occupancy record, and the guard runs
// before anything else is read, because the proximity re-run could otherwise
// defer an admitted pair no refusal backs.
//
// Leg shown to fail: dropping the guard dereferences the nil operands.
func TestComposeOccupancyDropsCoplanar(t *testing.T) {
	t.Parallel()
	for _, op := range []meshbool.OperationKind{meshbool.OpUnion, meshbool.OpCut} {
		require.Nil(t, composeOccupancy(t.Context(), op, nil, nil, nil, nil, nil, nil, nil, 0, 0, nil, true))
	}
}
