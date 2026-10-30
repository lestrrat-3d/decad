package breprecord

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// Each root corner of the 20x20 boss pairs only with its own inset corner,
// 2 mm inside along both axes. The upper inset's next corner, a mispaired
// cap edge, must be refused.
func TestBossSeamCornerPaired(t *testing.T) {
	t.Parallel()
	boss := prismshell.BossRect{U0: 20, V0: 10, U1: 40, V1: 30}
	upper := boss.Inset(2)
	roots := prismshell.PrismUnionBossShellCorners(boss)
	insets := prismshell.PrismUnionBossShellCorners(upper)
	pt := func(c [2]float64) sectionrecord.Point2 { return sectionrecord.Point2{U: c[0], V: c[1]} }
	for k := range 4 {
		require.True(t, bossSeamCornerPaired(pt(roots[k]), pt(insets[k]), boss, 2), "corner %d", k)
		require.False(t, bossSeamCornerPaired(pt(roots[k]), pt(insets[(k+1)%4]), boss, 2),
			"corner %d paired with the next inset corner", k)
	}
	require.False(t, bossSeamCornerPaired(sectionrecord.Point2{U: 30, V: 10},
		sectionrecord.Point2{U: 32, V: 12}, boss, 2), "a root point that is no boss corner")
}

// The placed ring vertices of a sound seam lie t·√2 and t from its centre,
// even far from the origin; a cap or side vertex taken from the next corner
// is refused.
func TestBossSeamFalsified(t *testing.T) {
	t.Parallel()
	down := r3.NewVec(0, 0, -1)
	for _, origin := range []r3.Vec{{}, r3.NewVec(1e4, -3e3, 7e2)} {
		center := origin.Add(r3.NewVec(20, 10, 20))
		capPos := origin.Add(r3.NewVec(22, 12, 20))
		sidePos := origin.Add(r3.NewVec(20, 10, 18))
		require.False(t, bossSeamFalsified(capPos, sidePos, center, down, 2))
		nextCap := origin.Add(r3.NewVec(38, 12, 20))
		nextSide := origin.Add(r3.NewVec(40, 10, 18))
		require.True(t, bossSeamFalsified(nextCap, sidePos, center, down, 2), "mispaired cap vertex")
		require.True(t, bossSeamFalsified(capPos, nextSide, center, down, 2), "mispaired side vertex")
	}
	require.True(t, bossSeamFalsified(r3.NewVec(22, 12, 20), r3.NewVec(20, 10, 18),
		r3.NewVec(20, 10, 20), down, 2*(1+1e-6)), "a radius off by one part in a million")
}
