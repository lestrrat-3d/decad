package prismpattern

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestLineOutsideNeedsWholeChordOutside(t *testing.T) {
	outsideSq := RadiusSq(5)
	line := func(u0, v0, u1, v1 float64) sectionrecord.LineSeg {
		return sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: u0, V: v0}, End: sectionrecord.Point2{U: u1, V: v1}, TStart: 0, TEnd: 1,
		}
	}
	// Both ends and the whole chord stay beyond radius 5.
	require.True(t, lineOutside(line(6, -1, 6, 1), outsideSq))
	// Both ends are outside but the chord passes at 6/sqrt(2) < 5.
	require.False(t, lineOutside(line(6, 0, 0, 6), outsideSq))
	// One end is inside the circle.
	require.False(t, lineOutside(line(4, 0, 6, 0), outsideSq))
}

func TestIVPointArithmetic(t *testing.T) {
	point := func(x, y int64) IVPoint {
		return IVPointOf(big.NewRat(x, 1), big.NewRat(y, 1))
	}
	exact := func(t *testing.T, iv proofbound.RatInterval, want int64) {
		t.Helper()
		require.Zero(t, iv.Lo.Cmp(big.NewRat(want, 1)), "lo %s want %d", iv.Lo, want)
		require.Zero(t, iv.Hi.Cmp(big.NewRat(want, 1)), "hi %s want %d", iv.Hi, want)
	}
	p, q := point(1, 2), point(3, -1)

	sum := p.Add(q)
	exact(t, sum.X, 4)
	exact(t, sum.Y, 1)
	diff := p.Sub(q)
	exact(t, diff.X, -2)
	exact(t, diff.Y, 3)
	exact(t, p.Dot(q), 1)
	two := proofbound.PointInterval(big.NewRat(2, 1))
	scaled := p.Scale(two)
	exact(t, scaled.X, 2)
	exact(t, scaled.Y, 4)

	_, err := ExactAngle(point(0, 0))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "joined-prism fillet has no exact polar point")
}
