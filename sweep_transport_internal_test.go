package decad

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestTransportSweepFramesAcrossOrthogonalBends(t *testing.T) {
	plane := PlaneRecord{
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
		require.Zero(t, got.originBound, "frame %d origin must stay exact", i)
		require.Zero(t, got.uBound, "frame %d U must stay exact", i)
		require.Zero(t, got.vBound, "frame %d V must stay exact", i)
		require.Zero(t, got.nBound, "frame %d N must stay exact", i)
		require.LessOrEqual(t, got.frame.Origin().Sub(want.origin).Len(), got.originBound, "frame %d origin", i)
		require.LessOrEqual(t, got.frame.U().Sub(want.u).Len(), got.uBound, "frame %d U", i)
		require.LessOrEqual(t, got.frame.V().Sub(want.v).Len(), got.vBound, "frame %d V", i)
		require.LessOrEqual(t, got.frame.N().Sub(want.n).Len(), got.nBound, "frame %d N", i)
	}

	require.Equal(t, frames[1].frame.U(), frames[2].frame.U())
	require.Equal(t, frames[1].frame.V(), frames[2].frame.V())
	require.Equal(t, frames[1].frame.N(), frames[2].frame.N())
}

func TestTransportSweepFramesRejectsNonTangentJoin(t *testing.T) {
	plane := PlaneRecord{
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
	plane := PlaneRecord{
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
