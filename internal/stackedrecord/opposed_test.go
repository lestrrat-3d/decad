package stackedrecord

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func rectangleLoop(u0, v0, u1, v1 float64) sectionrecord.LoopRecord {
	corners := []sectionrecord.Point2{{U: u0, V: v0}, {U: u1, V: v0}, {U: u1, V: v1}, {U: u0, V: v1}}
	segments := make([]sectionrecord.CurveSegment, len(corners))
	for i, corner := range corners {
		segments[i] = sectionrecord.LineSeg{Start: corner, End: corners[(i+1)%len(corners)], TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segments}
}

func TestSeparatedOpposedHoleScene(t *testing.T) {
	t.Parallel()
	first, err := offset2d.ReverseLoopRecordContext(t.Context(), rectangleLoop(-6, -2, -2, 2))
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
			t.Parallel()
			second, err := offset2d.ReverseLoopRecordContext(t.Context(), rectangleLoop(tc.x0, tc.y0, tc.x1, tc.y1))
			require.NoError(t, err)
			boundary, got, err := SeparatedOpposed(t.Context(), proofbound.NewWorkBudget(t.Context()),
				[]sectionrecord.LoopRecord{first}, []sectionrecord.LoopRecord{second})
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
