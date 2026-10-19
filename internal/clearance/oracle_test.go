package clearance

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestOracleClassifiesDegeneraciesWithoutGuessing(t *testing.T) {
	t.Parallel()
	oracle := Oracle{Tol: 1e-8}
	z := r3.NewVec(0, 0, 1)
	p := r3.NewVec(1, 2, 3)

	require.Equal(t, DegYes, oracle.Parallel(z, z.Scale(-4)))
	require.Equal(t, DegNo, oracle.Parallel(z, r3.NewVec(1, 0, 1)))
	require.Equal(t, DegUnknown, oracle.Parallel(z, r3.NewVec(1e-12, 0, 1)))

	require.Equal(t, DegYes, oracle.OnAxis(r3.NewVec(0, 0, -7), r3.NewVec(0, 0, 3), z))
	require.Equal(t, DegNo, oracle.OnAxis(r3.NewVec(0.5, 0, 0), r3.NewVec(0, 0, 3), z))
	require.Equal(t, DegUnknown, oracle.OnAxis(r3.NewVec(1e-12, 0, 0), r3.Vec{}, z))

	require.Equal(t, DegYes, oracle.Coincident(p, p))
	require.Equal(t, DegNo, oracle.Coincident(p, r3.NewVec(1, 2, 3.5)))
	require.Equal(t, DegUnknown, oracle.Coincident(p, r3.NewVec(1, 2, 3+1e-12)))
	require.Equal(t, DegUnknown, oracle.Parallel(r3.NewVec(math.Inf(1), 0, 0), z))
}

func TestOracleBoundsOnlyCertifiedSpineSuprema(t *testing.T) {
	t.Parallel()
	oracle := Oracle{Tol: 1e-8}
	z := r3.NewVec(0, 0, 1)
	line := func(anchor, axis r3.Vec) *CFace {
		return &CFace{Kind: CkCylinder, Anchor: anchor, Axis: axis, Radius: 1}
	}
	point := func(anchor r3.Vec, radius float64) *CFace {
		return &CFace{Kind: CkSphere, Anchor: anchor, Axis: z, Radius: radius}
	}

	_, ok := oracle.SpineSup(line(r3.NewVec(3, 0, 0), z), point(r3.Vec{}, 10))
	require.False(t, ok)
	sup, ok := oracle.SpineSup(point(r3.Vec{}, 10), line(r3.NewVec(3, 0, 0), z))
	require.True(t, ok)
	require.Equal(t, 3.0, sup)

	peg := line(r3.Vec{}, z)
	hole := line(r3.Vec{}, z)
	hole.Radius = 10
	peg.Radius = 5
	require.True(t, oracle.CertifiedContainment(peg, hole))
	require.True(t, oracle.CertifiedContainment(point(r3.NewVec(0, 0, 4), 5), hole))
	require.False(t, oracle.CertifiedContainment(point(r3.NewVec(0, 0, 4), 12), hole))
}
