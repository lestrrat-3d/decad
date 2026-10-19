package prismcells_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestArrangedProfileRecordMatchesAuthenticated(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	addSquare := func(lo, hi float64) {
		points := []*sketch.Point{
			s.CreatePoint(lo, lo), s.CreatePoint(hi, lo), s.CreatePoint(hi, hi), s.CreatePoint(lo, hi),
		}
		for _, point := range points {
			s.Fix(point)
		}
		for i, point := range points {
			s.CreateLine(point, points[(i+1)%len(points)])
		}
	}
	addSquare(0, 10)
	addSquare(2, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 2)
	match := profiles[0]
	before := append([]sketch.BoundaryEdge(nil), match.Outer...)
	authenticated, err := prismcells.RecordProfileContext(t.Context(), s, match)
	require.NoError(t, err)
	arranged, err := prismcells.RecordArrangedProfileContext(t.Context(), match)
	require.NoError(t, err)
	require.Equal(t, authenticated, arranged)
	require.Equal(t, before, match.Outer)

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = prismcells.RecordProfileContext(canceled, s, match)
	require.ErrorIs(t, err, context.Canceled)
	_, err = prismcells.RecordArrangedProfileContext(canceled, match)
	require.ErrorIs(t, err, context.Canceled)
}
