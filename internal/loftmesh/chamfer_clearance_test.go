package loftmesh

import (
	"math/big"
	"math/rand"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The old rational calculation is an independent oracle for the integer
// inequality, including strict equality at the clearance boundary.
func chamferAxisGapAboveRatReference(a, b [3]proofarith.Xpt,
	axis proofarith.Xpt, sourceBound *big.Rat) bool {
	norm2 := proofarith.XdotRat(axis, axis)
	if norm2.Sign() == 0 {
		return false
	}
	minA, maxA := proofarith.XdotRat(axis, a[0]), proofarith.XdotRat(axis, a[0])
	minB, maxB := proofarith.XdotRat(axis, b[0]), proofarith.XdotRat(axis, b[0])
	for i := 1; i < 3; i++ {
		p := proofarith.XdotRat(axis, a[i])
		if p.Cmp(minA) < 0 {
			minA = p
		}
		if p.Cmp(maxA) > 0 {
			maxA = p
		}
		p = proofarith.XdotRat(axis, b[i])
		if p.Cmp(minB) < 0 {
			minB = p
		}
		if p.Cmp(maxB) > 0 {
			maxB = p
		}
	}
	gap := new(big.Rat).Sub(minA, maxB)
	if reverse := new(big.Rat).Sub(minB, maxA); reverse.Cmp(gap) > 0 {
		gap = reverse
	}
	if gap.Sign() <= 0 {
		return false
	}
	gap2 := new(big.Rat).Mul(gap, gap)
	bound2 := new(big.Rat).Mul(sourceBound, sourceBound)
	return gap2.Cmp(new(big.Rat).Mul(bound2, norm2)) > 0
}

func TestChamferAxisGapIntegerMatchesRational(t *testing.T) {
	point := func(x, y, z float64) proofarith.Xpt {
		return proofarith.XptOf(r3.Vec{X: x, Y: y, Z: z})
	}
	a := [3]proofarith.Xpt{point(0, 0, 0), point(1, 0, 0), point(0, 1, 0)}
	b := [3]proofarith.Xpt{point(0, 0, 1), point(1, 0, 1), point(0, 1, 1)}
	axis := point(0, 0, 1)
	for _, tc := range []struct {
		name  string
		bound *big.Rat
		want  bool
	}{
		{name: "below gap", bound: big.NewRat(7, 8), want: true},
		{name: "equal gap", bound: big.NewRat(1, 1), want: false},
		{name: "above gap", bound: big.NewRat(9, 8), want: false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t, tc.want, chamferAxisGapAbove(a, b, axis, tc.bound))
			require.Equal(t, tc.want, chamferAxisGapAbove(b, a, axis, tc.bound))
		})
	}
	require.False(t, chamferAxisGapAbove(a, b, point(0, 0, 0), big.NewRat(0, 1)))

	rng := rand.New(rand.NewSource(72))
	randomPoint := func() proofarith.Xpt {
		return point(float64(rng.Intn(65)-32)/8,
			float64(rng.Intn(65)-32)/8, float64(rng.Intn(65)-32)/8)
	}
	for i := range 120 {
		var first, second [3]proofarith.Xpt
		for j := range 3 {
			first[j], second[j] = randomPoint(), randomPoint()
		}
		// Distinct weights exercise the cross-multiplied ordering and gap.
		first[1] = proofarith.Xlerp(first[0], first[1], big.NewInt(1), big.NewInt(3))
		second[2] = proofarith.Xlerp(second[0], second[2], big.NewInt(2), big.NewInt(5))
		direction := proofarith.Xcross(proofarith.Xsub(randomPoint(), randomPoint()),
			proofarith.Xsub(randomPoint(), randomPoint()))
		bound := big.NewRat(int64(rng.Intn(17)), 8)
		want := chamferAxisGapAboveRatReference(first, second, direction, bound)
		require.Equal(t, want, chamferAxisGapAbove(first, second, direction, bound),
			"random triangle pair %d", i)
	}
}
