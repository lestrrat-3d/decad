package occupancy

import (
	"math/big"
	"math/rand"
	"testing"

	"github.com/stretchr/testify/require"
)

// oldApply is the reference exactAffine.apply: one fresh product per term.
func oldApply(e exactAffine, p xvec) xvec {
	var out xvec
	for i := range 3 {
		s := new(big.Rat).Set(e.t[i])
		for j := range 3 {
			s.Add(s, new(big.Rat).Mul(e.m[i][j], p[j]))
		}
		out[i] = s
	}
	return out
}

func randRat(rng *rand.Rand) *big.Rat {
	switch rng.Intn(6) {
	case 0:
		return new(big.Rat)
	case 1:
		return big.NewRat(int64(rng.Intn(7)-3), 1)
	case 2:
		return big.NewRat(int64(rng.Intn(201)-100), int64(rng.Intn(50)+1))
	case 3:
		n := new(big.Int).Lsh(big.NewInt(int64(rng.Intn(1000)-500)), uint(rng.Intn(200)))
		d := new(big.Int).Lsh(big.NewInt(int64(rng.Intn(999)+1)), uint(rng.Intn(200)))
		return new(big.Rat).SetFrac(n, d)
	case 4:
		return new(big.Rat).SetFloat64(rng.NormFloat64() * 10)
	default:
		return new(big.Rat).SetFloat64(float64(rng.Intn(5) - 2))
	}
}

func TestExactAffineApplyMatchesReference(t *testing.T) {
	t.Parallel()
	check := func(t *testing.T, e exactAffine, p xvec) {
		t.Helper()
		wantIn := xvec{new(big.Rat).Set(p[0]), new(big.Rat).Set(p[1]), new(big.Rat).Set(p[2])}
		want := oldApply(e, p)
		got := e.apply(p)
		for i := range 3 {
			require.Zero(t, want[i].Cmp(got[i]), "component %d", i)
			require.Equal(t, want[i].Num().String(), got[i].Num().String(), "num %d", i)
			require.Equal(t, want[i].Denom().String(), got[i].Denom().String(), "denom %d", i)
			// The inputs are not modified, and the output shares nothing with them.
			require.Zero(t, wantIn[i].Cmp(p[i]), "input %d modified", i)
			require.NotSame(t, e.t[i], got[i])
			require.NotSame(t, p[i], got[i])
			for j := range 3 {
				require.NotSame(t, e.m[i][j], got[i])
			}
		}
		require.NotSame(t, got[0], got[1])
		require.NotSame(t, got[1], got[2])
	}

	identity := func() exactAffine {
		var e exactAffine
		for i := range 3 {
			for j := range 3 {
				e.m[i][j] = new(big.Rat)
			}
			e.m[i][i].SetInt64(1)
			e.t[i] = new(big.Rat)
		}
		return e
	}

	t.Run("random", func(t *testing.T) {
		t.Parallel()
		rng := rand.New(rand.NewSource(20261103))
		for range 3000 {
			var e exactAffine
			for i := range 3 {
				for j := range 3 {
					e.m[i][j] = randRat(rng)
				}
				e.t[i] = randRat(rng)
			}
			check(t, e, xvec{randRat(rng), randRat(rng), randRat(rng)})
		}
	})
	t.Run("identity", func(t *testing.T) {
		t.Parallel()
		check(t, identity(), xvec{big.NewRat(-3, 7), new(big.Rat), big.NewRat(5, 2)})
	})
	t.Run("translation only", func(t *testing.T) {
		t.Parallel()
		e := identity()
		e.t = xvec{big.NewRat(1, 3), big.NewRat(-2, 5), new(big.Rat)}
		check(t, e, xvec{big.NewRat(1, 7), big.NewRat(-1, 9), big.NewRat(4, 1)})
	})
	t.Run("singular", func(t *testing.T) {
		t.Parallel()
		var e exactAffine
		for i := range 3 {
			for j := range 3 {
				e.m[i][j] = big.NewRat(int64(i+1), 3)
			}
			e.t[i] = big.NewRat(int64(-i), 2)
		}
		check(t, e, xvec{big.NewRat(2, 3), big.NewRat(-5, 4), big.NewRat(7, 11)})
	})
	t.Run("nil factor panics", func(t *testing.T) {
		t.Parallel()
		// Column 1 is zero, so every product with p[1] has a zero factor.
		zeroColumn := identity()
		for i := range 3 {
			zeroColumn.m[i][0].SetInt64(1)
			zeroColumn.m[i][1].SetInt64(0)
			zeroColumn.m[i][2].SetInt64(2)
		}
		nilPoint := xvec{big.NewRat(1, 2), nil, big.NewRat(3, 1)}
		require.Panics(t, func() { oldApply(zeroColumn, nilPoint) }, "reference, nil p[1]")
		require.Panics(t, func() { zeroColumn.apply(nilPoint) }, "apply, nil p[1]")

		nilEntry := identity()
		nilEntry.m[0][1] = nil
		zeroPoint := xvec{big.NewRat(1, 2), new(big.Rat), big.NewRat(3, 1)}
		require.Panics(t, func() { oldApply(nilEntry, zeroPoint) }, "reference, nil m[0][1]")
		require.Panics(t, func() { nilEntry.apply(zeroPoint) }, "apply, nil m[0][1]")
	})
	t.Run("zero point", func(t *testing.T) {
		t.Parallel()
		e := identity()
		e.t = xvec{big.NewRat(1, 2), big.NewRat(3, 4), big.NewRat(-5, 6)}
		check(t, e, xvec{new(big.Rat), new(big.Rat), new(big.Rat)})
	})
}
