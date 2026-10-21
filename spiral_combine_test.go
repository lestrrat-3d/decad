package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The gallery's recorded toe inputs (agent-docs/faceted-vertex-bounds-design.md §3.7).
// They are inputs, never bounds.
var (
	recordedToeRoot  = planePoint{U: 10.922588984322125, V: 9.7440776823445425}
	recordedToeOuter = planePoint{U: 15.500000000000004, V: 5.1666666666666643}
)

func recordedToeCone(t *testing.T) *pointConeLimit {
	t.Helper()
	slope, ok := new(big.Rat).SetString("17157737065234201/17157737065234200")
	require.True(t, ok)
	return &pointConeLimit{Apex: new(big.Rat).SetFloat64(20.666666666666668), Slope: slope}
}

// toeResidual is x/a + k*r/q - A for a copy whose X scale is a and whose radial
// scale is q. With a = q = 1 it is the original toe residual.
func toeResidual(cone *pointConeLimit, p planePoint, a, q *big.Rat) *big.Rat {
	x := new(big.Rat).Quo(new(big.Rat).SetFloat64(p.U), a)
	kr := new(big.Rat).Mul(cone.Slope, new(big.Rat).SetFloat64(p.V))
	kr.Quo(kr, q)
	return x.Add(x, kr).Sub(x, cone.Apex)
}

func TestSpiralCopyKeepsToeCertificateOnlyUnderExactRotation(t *testing.T) {
	t.Parallel()
	cone := recordedToeCone(t)

	// Leg 1: the inputs reproduce the certified original side.
	_, ok := spiralsector.ToolOuterGap(cone, recordedToeRoot, recordedToeOuter, 1)
	require.True(t, ok)
	original := toeResidual(cone, recordedToeRoot, big.NewRat(1, 1), big.NewRat(1, 1))
	require.Equal(t, 1, original.Sign())

	// Leg 2 is a witness, not a red-first leg: it passes with or without the
	// fix. The recorded v1 float matrix of copy i = 23 reverses the toe-root
	// sign: sign((a*k*r)^2 - (c*c+s*s)*(a*A-x)^2).
	a := big.NewRat(1, 1)
	c, s := new(big.Rat).SetFloat64(-0.050649168838713551), new(big.Rat).SetFloat64(-0.99871650717105287)
	ak := new(big.Rat).Mul(a, cone.Slope)
	akr := ak.Mul(ak, new(big.Rat).SetFloat64(recordedToeRoot.V))
	aAx := new(big.Rat).Sub(new(big.Rat).Mul(a, cone.Apex), new(big.Rat).SetFloat64(recordedToeRoot.U))
	require.Equal(t, 1, akr.Sign(), "precondition: a*k*r > 0")
	require.Equal(t, 1, aAx.Sign(), "precondition: a*A - x > 0")
	cs := new(big.Rat).Add(new(big.Rat).Mul(c, c), new(big.Rat).Mul(s, s))
	left := new(big.Rat).Mul(akr, akr)
	right := new(big.Rat).Mul(cs, new(big.Rat).Mul(aAx, aAx))
	require.Equal(t, -1, left.Cmp(right), "the float matrix reverses the toe sign")

	// Leg 3: no record, no admission.
	_, ok = spiralsector.AxisCopyMap(nil)
	require.False(t, ok)

	// Leg 4: an exact record is orthogonal, so the copied residual is the
	// original residual exactly.
	for i := 1; i <= 30; i++ {
		rec, _, err := exactrotation.New(r3.Vec{}, r3.Vec{X: 1}, units.Radians(2*math.Pi*float64(i)/31))
		require.NoError(t, err)
		m, ok := spiralsector.AxisCopyMap(rec)
		require.True(t, ok, "copy %d", i)
		requireOrthogonal(t, m, i)
		require.Zero(t, m[0][0].Cmp(big.NewRat(1, 1)), "copy %d", i)
		sq := new(big.Rat).Add(new(big.Rat).Mul(m[1][1], m[1][1]), new(big.Rat).Mul(m[2][1], m[2][1]))
		require.Zero(t, sq.Cmp(big.NewRat(1, 1)), "copy %d: q = 1 exactly", i)
		if i == 23 {
			got := toeResidual(cone, recordedToeRoot, m[0][0], sq)
			require.Zero(t, got.Cmp(original), "the copied toe residual equals the original exactly")
		}
	}
}

// requireOrthogonal checks Q^T Q = I exactly.
func requireOrthogonal(t *testing.T, m [3][3]*big.Rat, i int) {
	t.Helper()
	for r := range 3 {
		for c := range 3 {
			dot := new(big.Rat)
			for k := range 3 {
				dot.Add(dot, new(big.Rat).Mul(m[k][r], m[k][c]))
			}
			want := big.NewRat(0, 1)
			if r == c {
				want = big.NewRat(1, 1)
			}
			require.Zero(t, dot.Cmp(want), "copy %d: (Q^T Q)[%d][%d]", i, r, c)
		}
	}
}
