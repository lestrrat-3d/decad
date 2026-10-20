package revolveplan_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// The point-cone arm is accept-only: it answers false, with no error, for an
// operation other than Cut or Intersect and for a tool that is not an exact
// cone source. The accepting path runs in the apitest point-Loft cone tests,
// which need a real certified cone.
func TestPointConeBooleanDeclines(t *testing.T) {
	t.Parallel()

	target := &topo.Body{Payload: facetedrecord.Payload{}}
	for _, op := range []meshbool.OperationKind{meshbool.OpUnion, meshbool.OpCut, meshbool.OpIntersect} {
		_, ok, err := revolveplan.PointConeBoolean(t.Context(), op, topo.Env{}, target, &topo.Body{})
		require.NoError(t, err, op.String())
		require.False(t, ok, "%s with a tool that is no cone", op)
	}
	_, ok, err := revolveplan.PointConeBoolean(t.Context(), meshbool.OpUnion, topo.Env{}, target,
		&topo.Body{Payload: revolveplan.Payload{}})
	require.NoError(t, err)
	require.False(t, ok)
}
