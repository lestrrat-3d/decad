package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestUnionManyOperationWarnings(t *testing.T) {
	for _, tc := range []struct {
		name    string
		ordinal int
	}{
		{name: "first pair warning survives the later pair", ordinal: 0},
		{name: "later pair keeps its original argument ordinal", ordinal: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			first := boxBody(t, doc, 0, 0, 10, 10, 10)
			// A round operand keeps the pair on the mesh route, where a coplanar face takes a gap shift: two boxes meet
			// exactly on a grid (agent-docs/orthogonal-boolean-design.md) and need none.
			touching := translated(t, discBody(t, doc, 5, 3, 10), 0, 5, 10)
			inside := translated(t, boxBody(t, doc, 2, 2, 4, 4, 2), 0, 0, 2)
			others := []*decad.Body{touching, inside}
			if tc.ordinal == 1 {
				others[0], others[1] = others[1], others[0]
			}

			joined, err := decad.Union(t.Context(), first, others...)
			require.NoError(t, err)
			requireVolumeWithin(t, joined, 1000+math.Pi*9*10)
			live := doc.Bodies()
			require.Len(t, live, 1)
			require.Same(t, joined, live[0])
			warnings := joined.OperationDiagnostics()
			require.Len(t, warnings, 1)
			warning := warnings[0]
			require.Equal(t, decad.OpDiagGapShiftApplied, warning.Code)
			require.Equal(t, decad.OperationUnion, warning.Operation)
			require.Equal(t, "others", warning.Subject.Name)
			require.NotNil(t, warning.Subject.Ordinal)
			require.Equal(t, tc.ordinal, *warning.Subject.Ordinal)
			require.Nil(t, warning.Subject.Body)
			require.NotNil(t, warning.Change)
			require.Equal(t, units.Length, warning.Change.DisplacementBound.Kind())
			require.Positive(t, warning.Change.DisplacementBound.Base())
			require.LessOrEqual(t, warning.Change.DisplacementBound.Base(), warning.Change.AllowedReach.Base())
			// Diagnostics belong to this successful call; the immutable inputs
			// do not acquire warnings from a transient Union pair.
			require.Empty(t, first.OperationDiagnostics())
			require.Empty(t, touching.OperationDiagnostics())
			require.Empty(t, inside.OperationDiagnostics())
		})
	}
}
