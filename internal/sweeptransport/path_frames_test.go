package sweeptransport

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestTransportFramesAcrossOrthogonalBends(t *testing.T) {
	start := r3.Vec{}
	segments := []sweepinput.Segment{
		{Arc: true, Through: r3.NewVec(2, 0, 4), End: r3.NewVec(5, 0, 5)},
		{End: r3.NewVec(10, 0, 5)},
		{Arc: true, Through: r3.NewVec(14, 2, 5), End: r3.NewVec(15, 5, 5)},
	}
	path := recordedPath(t, start, segments)
	plane := sectionrecord.PlaneRecord{Origin: start, U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
	frames, err := TransportSweepFramesContext(t.Context(), path, plane)
	require.NoError(t, err)
	require.Len(t, frames, 4)
	expected := []struct {
		origin, u, v, n r3.Vec
	}{
		{r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)},
		{r3.NewVec(5, 0, 5), r3.NewVec(0, 0, -1), r3.NewVec(0, 1, 0), r3.NewVec(1, 0, 0)},
		{r3.NewVec(10, 0, 5), r3.NewVec(0, 0, -1), r3.NewVec(0, 1, 0), r3.NewVec(1, 0, 0)},
		{r3.NewVec(15, 5, 5), r3.NewVec(0, 0, -1), r3.NewVec(-1, 0, 0), r3.NewVec(0, 1, 0)},
	}
	for index, want := range expected {
		got := frames[index]
		require.Zero(t, got.OriginBound)
		require.Zero(t, got.UBound)
		require.Zero(t, got.VBound)
		require.Zero(t, got.NBound)
		require.Equal(t, want.origin, got.Frame.Origin())
		require.Equal(t, want.u, got.Frame.U())
		require.Equal(t, want.v, got.Frame.V())
		require.Equal(t, want.n, got.Frame.N())
	}
}

func TestTransportFramesRejectsUnsupportedJoins(t *testing.T) {
	plane := sectionrecord.PlaneRecord{U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
	for name, segment := range map[string]sweepinput.Segment{
		"non-tangent": {Arc: false, End: r3.NewVec(5, 5, 5)},
		"reversed":    {End: r3.NewVec(0, 0, 5)},
	} {
		t.Run(name, func(t *testing.T) {
			path := recordedPath(t, r3.Vec{}, []sweepinput.Segment{
				{Arc: true, Through: r3.NewVec(2, 0, 4), End: r3.NewVec(5, 0, 5)}, segment,
			})
			_, err := TransportSweepFramesContext(t.Context(), path, plane)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
		})
	}
}

func recordedPath(t *testing.T, start r3.Vec, segments []sweepinput.Segment) *sweepinput.Path {
	t.Helper()
	path := &sweepinput.Path{Start: start, Segments: segments}
	current := start
	for index, segment := range segments {
		record, err := sweepinput.RecordSegment(current, segment, index)
		require.NoError(t, err)
		path.Records = append(path.Records, record)
		current = segment.End
	}
	path.End = current
	return path
}
