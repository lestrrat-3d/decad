package sketchrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestAuthenticateChainMatchesByContentAfterRename(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	var lines []*sketch.Line
	for _, name := range []string{"A", "B", "C"} {
		line := s.CreateLine(s.CreatePoint(0, 0), s.CreatePoint(10, 0))
		line.SetName(name)
		lines = append(lines, line)
	}

	held := s.Chains()
	require.Len(t, held, 3)
	require.Same(t, lines[0], held[0].Entities[0])
	target := held[0]
	require.False(t, target.IsStale())

	lines[0].SetName("Z")
	require.False(t, target.IsStale())
	fresh := s.Chains()
	require.NotSame(t, target.Entities[0], fresh[0].Entities[0])
	require.False(t, sketchrecord.SameChainSnapshot(target, fresh[0]))

	trusted, err := sketchrecord.AuthenticateChain(s, target)
	require.NoError(t, err)
	require.Same(t, lines[0], trusted.Entities[0])
}
