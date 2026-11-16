package stackedunion

// An internal test: wholeEdgeAt is unexported, and the Partial edge it
// refuses comes from a sketch arrangement no public fixture pins down.

import (
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestWholeEdgeAtReadsOnlyAWholeEdge(t *testing.T) {
	t.Parallel()
	s := sketch.NewWorld()
	sk, err := s.CreateSketch(s.XY())
	require.NoError(t, err)
	arc := sk.CreateArc(sk.CreatePoint(0, 0), sk.CreatePoint(7, 0), sk.CreatePoint(0, 7))
	other := sk.CreateArc(sk.CreatePoint(0, 0), sk.CreatePoint(9, 0), sk.CreatePoint(0, 9))
	whole := sketch.BoundaryEdge{Entity: arc, TStart: 0, TEnd: 1}
	partialAtStart := sketch.BoundaryEdge{Entity: arc, Partial: true, TStart: 0, TEnd: 0.5}
	reversedWhole := sketch.BoundaryEdge{Entity: arc, Reversed: true, TStart: 1, TEnd: 0}
	for _, tc := range []struct {
		name  string
		edges []sketch.BoundaryEdge
		t     float64
		want  bool
	}{
		{"whole edge at its start", []sketch.BoundaryEdge{whole}, 0, true},
		{"whole edge at its end", []sketch.BoundaryEdge{whole}, 1, true},
		{"reversed whole edge", []sketch.BoundaryEdge{reversedWhole}, 1, true},
		{"a Partial edge at the bound", []sketch.BoundaryEdge{partialAtStart}, 0, false},
		{"another entity's whole edge", []sketch.BoundaryEdge{{Entity: other, TStart: 0, TEnd: 1}}, 0, false},
		{"no edge", nil, 0, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, wholeEdgeAt(tc.edges, arc, tc.t))
		})
	}
}
