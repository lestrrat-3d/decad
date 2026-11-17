package capband_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestJoinTurnSinSq(t *testing.T) {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	// The unit quarter arc ends at (0, 1) heading in -u.
	arc := sectionrecord.ArcSeg{Center: p(0, 0), Start: p(1, 0), End: p(0, 1), TStart: 0, TEnd: 1}
	line := func(u, v float64) sectionrecord.LineSeg {
		return sectionrecord.LineSeg{Start: p(0, 1), End: p(u, v), TStart: 0, TEnd: 1}
	}

	t.Run("an exactly tangent join turns by zero", func(t *testing.T) {
		got, ok := capband.JoinTurnSinSq(arc, line(-1, 1))
		require.True(t, ok)
		require.Zero(t, got.Sign())
		require.True(t, capband.JoinIsG1(arc, line(-1, 1)))
	})
	t.Run("a kink reads its exact sin squared", func(t *testing.T) {
		got, ok := capband.JoinTurnSinSq(arc, line(-1, 1.5))
		require.True(t, ok)
		// a = (-1, 0), b = (-1, 0.5): cross = -0.5, |a|² = 1, |b|² = 1.25.
		require.Zero(t, got.Cmp(big.NewRat(1, 5)))
	})
	t.Run("different ends read nothing", func(t *testing.T) {
		moved := sectionrecord.LineSeg{Start: p(0, 1.5), End: p(-1, 1.5), TStart: 0, TEnd: 1}
		_, ok := capband.JoinTurnSinSq(arc, moved)
		require.False(t, ok)
	})
	t.Run("a cusp reads nothing", func(t *testing.T) {
		_, ok := capband.JoinTurnSinSq(arc, line(1, 1))
		require.False(t, ok)
	})
	t.Run("a segment of another kind reads nothing", func(t *testing.T) {
		_, ok := capband.JoinTurnSinSq(arc, sectionrecord.CircleSeg{})
		require.False(t, ok)
	})
}
