package filletband_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func contourSquare() sectionrecord.LoopRecord {
	pts := [][2]float64{{0, 0}, {4, 0}, {4, 4}, {0, 4}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		segs[i] = sectionrecord.LineSeg{
			Start:  sectionrecord.Point2{U: p[0], V: p[1]},
			End:    sectionrecord.Point2{U: q[0], V: q[1]},
			TStart: 0, TEnd: 1,
		}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func TestLoopContourDeltaCircleAndSquare(t *testing.T) {
	circle := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{
		Radius: units.Millimeters(2), CCW: true, TStart: 0, TEnd: 1,
	}}}
	got, err := filletband.LoopContourDelta(t.Context(), circle, 0.5, 0, 1e-9)
	require.NoError(t, err)
	want, err := capband.LoopContourDelta(t.Context(), circle, 0.5, 0, 1e-9)
	require.NoError(t, err)
	require.Equal(t, 0.0, got)
	require.Equal(t, want, got)

	budget := proofbound.NewWorkBudget(t.Context())
	walks, err := capband.LoopWalks(budget, contourSquare(), freeform.NewFreeformWork())
	require.NoError(t, err)
	wantSquare, err := filletband.ContourDelta(budget, walks, 0.5, 0.001, 1e-9)
	require.NoError(t, err)
	gotSquare, err := filletband.LoopContourDelta(t.Context(), contourSquare(), 0.5, 0.001, 1e-9)
	require.NoError(t, err)
	require.Equal(t, wantSquare, gotSquare)
}
