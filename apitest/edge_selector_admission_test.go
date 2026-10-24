package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func testEdgeSelectorAdmission(t *testing.T, operation string, call edgeModifyCall) {
	t.Helper()
	t.Parallel()
	t.Run("BuiltInQuery", func(t *testing.T) {
		_, box := filletBox(t)

		_, err := call(t.Context(), box, verticalEdges(), units.Millimeters(5))
		require.NoError(t, err)
	})

	t.Run("ForeignRetiringImplementation", func(t *testing.T) {
		doc, box := filletBox(t)
		calls := 0
		foreign := retiringEdgeSelector{
			EdgeQuery: verticalEdges(),
			calls:     &calls,
		}

		_, err := call(t.Context(), box, foreign, units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Zero(t, calls, "%s rejects a foreign selector before it can retire the receiver", operation)
		require.Equal(t, []*decad.Body{box}, doc.Bodies(), "a rejected selector leaves the receiver live")
	})

	t.Run("TypedNilQuery", func(t *testing.T) {
		doc, box := filletBox(t)
		var query *decad.EdgeQuery
		var selector decad.EdgeSelector = query

		_, err := call(t.Context(), box, selector, units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, []*decad.Body{box}, doc.Bodies(), "a typed nil selector leaves the receiver live")
	})
}
