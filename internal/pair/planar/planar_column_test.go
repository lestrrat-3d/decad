package planar

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// columnTriangleGap compares its gaps in the common-denominator form; this
// test holds it to the big.Rat evaluation over the box's four corners, kept
// below as columnTriangleGapRational. Every gap must be the identical rational,
// and both must refuse the same triangles. The boxes carry non-dyadic corners,
// as the rotating sweep's hulls do, with their ends in either order.

func columnTriangleGapRational(tri [3][2]proof.Dyadic, box [4][2]*big.Rat) (*big.Rat, bool) {
	one := proof.DyInt(1)
	axes := [][2]proof.Dyadic{{one, proof.DyZero()}, {proof.DyZero(), one}}
	for k := range 3 {
		from, to := tri[k], tri[(k+1)%3]
		axis := [2]proof.Dyadic{proof.DyNeg(proof.DySubScalar(to[1], from[1])), proof.DySubScalar(to[0], from[0])}
		if axis[0].Sign() == 0 && axis[1].Sign() == 0 {
			continue
		}
		axes = append(axes, axis)
	}
	var best *big.Rat
	for slot, axis := range axes {
		ax, ay := axis[0].Rat(), axis[1].Rat()
		var triLo, triHi, boxLo, boxHi *big.Rat
		for _, corner := range tri {
			value := proof.DyAdd(proof.DyMul(axis[0], corner[0]), proof.DyMul(axis[1], corner[1])).Rat()
			triLo, triHi = ratLower(triLo, value), ratUpper(triHi, value)
		}
		for _, corner := range box {
			value := new(big.Rat).Add(new(big.Rat).Mul(ax, corner[0]), new(big.Rat).Mul(ay, corner[1]))
			boxLo, boxHi = ratLower(boxLo, value), ratUpper(boxHi, value)
		}
		gap := new(big.Rat).Sub(triLo, boxHi)
		if other := new(big.Rat).Sub(boxLo, triHi); other.Cmp(gap) > 0 {
			gap = other
		}
		if gap.Sign() <= 0 {
			continue
		}
		// The box axes are unit vectors; an edge normal's length is bounded
		// above by an exactly checked float square root.
		if slot >= 2 {
			length := proof.DySqrtUp(proof.DyAdd(proof.DyMul(axis[0], axis[0]), proof.DyMul(axis[1], axis[1])))
			if math.IsInf(length, 0) || math.IsNaN(length) || length <= 0 {
				continue
			}
			gap.Quo(gap, proof.FloatRat(length))
		}
		if best == nil || gap.Cmp(best) > 0 {
			best = gap
		}
	}
	return best, best != nil
}

func ratLower(current, value *big.Rat) *big.Rat {
	if current == nil || value.Cmp(current) < 0 {
		return value
	}
	return current
}

func ratUpper(current, value *big.Rat) *big.Rat {
	if current == nil || value.Cmp(current) > 0 {
		return value
	}
	return current
}

func TestColumnTriangleGapMatchesRationalForm(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(3, 5))
	coordinate := func() proof.Dyadic {
		return proof.DyShift(proof.DyInt(int64(rng.IntN(33)-16)), -rng.IntN(3))
	}
	bound := func() *big.Rat {
		return big.NewRat(int64(rng.IntN(41)-20), int64(1+rng.IntN(9)))
	}
	separated, blocked := 0, 0
	for range 3000 {
		var tri [3][2]proof.Dyadic
		for k := range tri {
			tri[k] = [2]proof.Dyadic{coordinate(), coordinate()}
		}
		lo, hi := [2]*big.Rat{bound(), bound()}, [2]*big.Rat{bound(), bound()}
		corners := [4][2]*big.Rat{{lo[0], lo[1]}, {hi[0], lo[1]}, {lo[0], hi[1]}, {hi[0], hi[1]}}
		want, wantOK := columnTriangleGapRational(tri, corners)
		projected := newColumnBox(lo, hi)
		got, gotOK := columnTriangleGap(tri, projected, true)
		require.Equal(t, wantOK, gotOK)
		_, apart := columnTriangleGap(tri, projected, false)
		require.Equal(t, wantOK, apart)
		if !wantOK {
			blocked++
			continue
		}
		separated++
		require.Zero(t, want.Cmp(got), "%v vs %v", want, got)
	}
	require.Positive(t, separated, "premise: some triangles clear the box")
	require.Positive(t, blocked, "premise: some triangles meet the box")
	for range 1000 {
		var tri [3][2]proof.Dyadic
		fixed := rng.IntN(2)
		at := coordinate()
		for k := range tri {
			tri[k] = [2]proof.Dyadic{coordinate(), coordinate()}
			tri[k][fixed] = at
		}
		lo, hi := [2]*big.Rat{bound(), bound()}, [2]*big.Rat{bound(), bound()}
		corners := [4][2]*big.Rat{{lo[0], lo[1]}, {hi[0], lo[1]}, {lo[0], hi[1]}, {hi[0], hi[1]}}
		want, wantOK := columnTriangleGapRational(tri, corners)
		projected := newColumnBox(lo, hi)
		got, gotOK := columnTriangleGap(tri, projected, true)
		require.Equal(t, wantOK, gotOK)
		_, apart := columnTriangleGap(tri, projected, false)
		require.Equal(t, wantOK, apart)
		if wantOK {
			require.Zero(t, want.Cmp(got), "%v vs %v", want, got)
		}
	}
	// The diagonal is the only separating axis, but its float length is
	// unbounded. Both modes must keep that axis unproved.
	huge := proof.DyShift(proof.DyInt(1), 1024)
	threeHuge := proof.DyMul(proof.DyInt(3), huge)
	tri := [3][2]proof.Dyadic{{threeHuge, proof.DyZero()}, {proof.DyZero(), threeHuge}, {threeHuge, threeHuge}}
	zero, top := new(big.Rat), huge.Rat()
	box := newColumnBox([2]*big.Rat{zero, zero}, [2]*big.Rat{top, top})
	_, withClearance := columnTriangleGap(tri, box, true)
	_, apart := columnTriangleGap(tri, box, false)
	require.False(t, withClearance)
	require.Equal(t, withClearance, apart)
}
