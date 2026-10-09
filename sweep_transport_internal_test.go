package decad

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestTransportSweepFramesAcrossOrthogonalBends(t *testing.T) {
	plane := planeRecord{
		Origin: r3.NewVec(0, 0, 0),
		U:      r3.NewVec(1, 0, 0),
		V:      r3.NewVec(0, 1, 0),
	}
	path, err := NewPath(
		r3.NewVec(0, 0, 0),
		ArcThrough{Through: r3.NewVec(2, 0, 4), End: r3.NewVec(5, 0, 5)},
		LineTo{End: r3.NewVec(10, 0, 5)},
		ArcThrough{Through: r3.NewVec(14, 2, 5), End: r3.NewVec(15, 5, 5)},
	)
	require.NoError(t, err)

	frames, err := transportSweepFramesContext(t.Context(), path, plane)
	require.NoError(t, err)
	require.Len(t, frames, 4)

	expected := []struct {
		origin r3.Vec
		u      r3.Vec
		v      r3.Vec
		n      r3.Vec
	}{
		{
			origin: r3.NewVec(0, 0, 0),
			u:      r3.NewVec(1, 0, 0),
			v:      r3.NewVec(0, 1, 0),
			n:      r3.NewVec(0, 0, 1),
		},
		{
			origin: r3.NewVec(5, 0, 5),
			u:      r3.NewVec(0, 0, -1),
			v:      r3.NewVec(0, 1, 0),
			n:      r3.NewVec(1, 0, 0),
		},
		{
			origin: r3.NewVec(10, 0, 5),
			u:      r3.NewVec(0, 0, -1),
			v:      r3.NewVec(0, 1, 0),
			n:      r3.NewVec(1, 0, 0),
		},
		{
			origin: r3.NewVec(15, 5, 5),
			u:      r3.NewVec(0, 0, -1),
			v:      r3.NewVec(-1, 0, 0),
			n:      r3.NewVec(0, 1, 0),
		},
	}
	for i, want := range expected {
		got := frames[i]
		require.Zero(t, got.OriginBound, "frame %d origin must stay exact", i)
		require.Zero(t, got.UBound, "frame %d U must stay exact", i)
		require.Zero(t, got.VBound, "frame %d V must stay exact", i)
		require.Zero(t, got.NBound, "frame %d N must stay exact", i)
		require.LessOrEqual(t, got.Frame.Origin().Sub(want.origin).Len(), got.OriginBound, "frame %d origin", i)
		require.LessOrEqual(t, got.Frame.U().Sub(want.u).Len(), got.UBound, "frame %d U", i)
		require.LessOrEqual(t, got.Frame.V().Sub(want.v).Len(), got.VBound, "frame %d V", i)
		require.LessOrEqual(t, got.Frame.N().Sub(want.n).Len(), got.NBound, "frame %d N", i)
	}

	require.Equal(t, frames[1].Frame.U(), frames[2].Frame.U())
	require.Equal(t, frames[1].Frame.V(), frames[2].Frame.V())
	require.Equal(t, frames[1].Frame.N(), frames[2].Frame.N())
}

func TestTransportSweepFramesRejectsNonTangentJoin(t *testing.T) {
	plane := planeRecord{
		Origin: r3.NewVec(0, 0, 0),
		U:      r3.NewVec(1, 0, 0),
		V:      r3.NewVec(0, 1, 0),
	}
	path, err := NewPath(
		r3.NewVec(0, 0, 0),
		ArcThrough{Through: r3.NewVec(2, 0, 4), End: r3.NewVec(5, 0, 5)},
		LineTo{End: r3.NewVec(5, 5, 5)},
	)
	require.NoError(t, err)

	_, err = transportSweepFramesContext(t.Context(), path, plane)
	require.Error(t, err)
	require.True(t, errors.Is(err, ErrUnsupported))
}

func TestTransportSweepFramesRejectsReversedTangentJoin(t *testing.T) {
	plane := planeRecord{
		Origin: r3.NewVec(0, 0, 0),
		U:      r3.NewVec(1, 0, 0),
		V:      r3.NewVec(0, 1, 0),
	}
	path, err := NewPath(
		r3.NewVec(0, 0, 0),
		ArcThrough{Through: r3.NewVec(2, 0, 4), End: r3.NewVec(5, 0, 5)},
		LineTo{End: r3.NewVec(0, 0, 5)},
	)
	require.NoError(t, err)

	_, err = transportSweepFramesContext(t.Context(), path, plane)
	require.Error(t, err)
	require.True(t, errors.Is(err, ErrUnsupported))
}
