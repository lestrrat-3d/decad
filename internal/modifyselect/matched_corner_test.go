package modifyselect_test

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func walkAt(u, v float64) survey2d.SideWalk {
	return survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{StartU: u, StartV: v}}
}

func TestNewMatchedCorner(t *testing.T) {
	t.Parallel()

	edge := &topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexAt(3, 4, 0), End: vertexAt(3, 4, 9)}
	loop := survey2d.CornerLoop{Walks: []survey2d.SideWalk{walkAt(0, 0), walkAt(3, 4)}}
	got := modifyselect.NewMatchedCorner(2, edge, 5, 1, loop)
	require.Equal(t, 2, got.EdgeOrdinal)
	require.Same(t, edge, got.Edge)
	require.Equal(t, 5, got.Loop)
	require.Equal(t, 1, got.Corner)
	require.Equal(t, 3.0, got.Point.U)
	require.Equal(t, 4.0, got.Point.V)
	require.Equal(t, `selected edge[2] from (3,4,0) to (3,4,9) maps to loop 5 corner 1 at (u, v) = (3, 4)`, got.String())
}

func TestWrapAuditError(t *testing.T) {
	t.Parallel()

	edge := &topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexAt(3, 4, 0), End: vertexAt(3, 4, 9)}
	loop := survey2d.CornerLoop{Walks: []survey2d.SideWalk{walkAt(3, 4)}}
	matched := []modifyselect.MatchedCorner{
		modifyselect.NewMatchedCorner(0, edge, 0, 0, loop),
		modifyselect.NewMatchedCorner(1, edge, 0, 0, loop),
	}
	t.Run("keeps the cause first and the sentinel", func(t *testing.T) {
		t.Parallel()
		cause := errors.Join(decaderr.ErrDegenerate, errors.New("loop collapsed"))
		err := modifyselect.WrapAuditError("edges(all)", matched, cause)
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		require.Equal(t, cause.Error()+`; selector edges(all) matched [`+matched[0].String()+`; `+matched[1].String()+`]`,
			err.Error())
	})
	t.Run("no matches", func(t *testing.T) {
		t.Parallel()
		err := modifyselect.WrapAuditError("sel", nil, decaderr.ErrDegenerate)
		require.Equal(t, decaderr.ErrDegenerate.Error()+`; selector sel matched []`, err.Error())
	})
	t.Run("renders the detailed audit diagnostic", func(t *testing.T) {
		t.Parallel()
		cause := sectionaudit.NewError(decaderr.ErrDegenerate, "detailed reason")
		err := modifyselect.WrapAuditError("sel", nil, cause)
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		require.Equal(t, `detailed reason; selector sel matched []`, err.Error())
	})
}
