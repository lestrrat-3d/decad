package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/stretchr/testify/require"
)

func TestSeparatedOpposedHoleScene(t *testing.T) {
	first, err := offset2d.ReverseLoopRecordContext(t.Context(), rectangleRecord(-6, -2, -2, 2).Outer)
	require.NoError(t, err)
	for _, tc := range []struct {
		name           string
		x0, y0, x1, y1 float64
		want           bool
	}{
		{name: "separate", x0: 2, y0: -2, x1: 6, y1: 2, want: true},
		{name: "overlap", x0: -4, y0: -2, x1: 0, y1: 2},
		{name: "touch", x0: -2, y0: -2, x1: 2, y1: 2},
		{name: "nested", x0: -5, y0: -1, x1: -3, y1: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			second, err := offset2d.ReverseLoopRecordContext(t.Context(),
				rectangleRecord(tc.x0, tc.y0, tc.x1, tc.y1).Outer)
			require.NoError(t, err)
			boundary, got, err := stackedrecord.SeparatedOpposed(t.Context(), proofbound.NewWorkBudget(t.Context()),
				[]loopRecord{first}, []loopRecord{second})
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
			if tc.want {
				require.Len(t, boundary.LowerExposed, 1)
				require.Len(t, boundary.UpperExposed, 1)
			} else {
				require.Empty(t, boundary.LowerExposed)
				require.Empty(t, boundary.UpperExposed)
			}
		})
	}
}
