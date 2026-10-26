package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestXdotDiffSignMatchesExactSubtraction(t *testing.T) {
	t.Parallel()
	points := []r3.Vec{
		{},
		{X: 1, Y: -2, Z: 3},
		{X: 0.5, Y: 1, Z: -4},
		{X: math.Ldexp(1, -120), Y: 1, Z: 0},
		{X: -math.Ldexp(1, -70), Y: 1, Z: 0},
	}
	normals := []r3.Vec{{X: 1}, {X: 1, Y: 1, Z: -1}, {X: math.Ldexp(1, 80), Y: -1}}
	for _, nv := range normals {
		n := proof.XptOf(nv)
		for _, pv := range points {
			p := proof.XptOf(pv)
			for _, qv := range points {
				q := proof.XptOf(qv)
				for _, factor := range []int64{1, 3} {
					scaled := proof.Xpt{
						X: new(big.Int).Mul(q.X, big.NewInt(factor)),
						Y: new(big.Int).Mul(q.Y, big.NewInt(factor)),
						Z: new(big.Int).Mul(q.Z, big.NewInt(factor)),
						W: new(big.Int).Mul(q.W, big.NewInt(factor)),
					}
					want := proof.XdotSign(n, proof.Xsub(p, scaled))
					require.Equal(t, want, proof.XdotDiffSign(n, p, scaled), "n=%v p=%v q=%v factor=%d", nv, pv, qv, factor)
				}
			}
		}
	}
}

func TestXhpVecKeepsRationalFloatRounding(t *testing.T) {
	t.Parallel()
	check := func(num, den *big.Int) {
		t.Helper()
		positive, _ := new(big.Rat).SetFrac(num, den).Float64()
		negative, _ := new(big.Rat).SetFrac(new(big.Int).Neg(num), den).Float64()
		got := proof.XhpVec(proof.Xhp{
			X: num,
			Y: new(big.Int),
			Z: new(big.Int).Neg(num),
			W: den,
		})
		require.Equal(t, math.Float64bits(positive), math.Float64bits(got.X))
		require.Equal(t, uint64(0), math.Float64bits(got.Y))
		require.Equal(t, math.Float64bits(negative), math.Float64bits(got.Z))
	}

	check(big.NewInt(0), big.NewInt(1))
	check(big.NewInt(1), big.NewInt(3))
	check(new(big.Int).Add(new(big.Int).Lsh(big.NewInt(1), 53), big.NewInt(1)),
		new(big.Int).Lsh(big.NewInt(1), 53))
	check(big.NewInt(3), new(big.Int).Lsh(big.NewInt(1), 1075))
	check(new(big.Int).Lsh(big.NewInt(3), 500), new(big.Int).Lsh(big.NewInt(7), 500))
	for _, shift := range []uint{52, 53, 1022, 1074, 1075, 1100, 2048} {
		check(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), shift))
		check(new(big.Int).Lsh(big.NewInt(1), shift), big.NewInt(3))
	}
	rng := rand.New(rand.NewPCG(0x8db4c214, 0x54fc32a7))
	for range 1000 {
		num := new(big.Int).Lsh(new(big.Int).SetUint64(rng.Uint64()|1), uint(rng.IntN(1100)))
		num.Add(num, new(big.Int).SetUint64(rng.Uint64()))
		den := new(big.Int).Lsh(new(big.Int).SetUint64(rng.Uint64()|1), uint(rng.IntN(1100)))
		den.Add(den, new(big.Int).SetUint64(rng.Uint64()))
		check(num, den)
	}
}

func TestXptDenotesComparesExactPointToFloatVertex(t *testing.T) {
	t.Parallel()
	v := r3.Vec{X: 0.1, Y: -2.5, Z: 1e-3}
	p := proof.XptOf(v)
	require.True(t, proof.XptDenotes(p, v))
	for _, other := range []r3.Vec{
		{X: math.Nextafter(v.X, 1), Y: v.Y, Z: v.Z},
		{X: v.X, Y: math.Nextafter(v.Y, 0), Z: v.Z},
		{X: v.X, Y: v.Y, Z: -v.Z},
	} {
		require.False(t, proof.XptDenotes(p, other))
	}
	// A point spelled over a larger power-of-two denominator still denotes it.
	scaled := proof.Xpt{X: new(big.Int).Lsh(p.X, 3), Y: new(big.Int).Lsh(p.Y, 3),
		Z: new(big.Int).Lsh(p.Z, 3), W: new(big.Int).Lsh(p.W, 3)}
	require.True(t, proof.XptDenotes(scaled, v))
	// A third of a unit is not any float.
	third := proof.Xpt{X: big.NewInt(1), Y: big.NewInt(0), Z: big.NewInt(0), W: big.NewInt(3)}
	require.False(t, proof.XptDenotes(third, r3.Vec{X: 1.0 / 3}))
	require.True(t, proof.XptDenotes(proof.XptOf(r3.Vec{}), r3.Vec{}))
}

func TestXplaneCrossingMatchesRationalInterpolation(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x716d, 0x92ab))
	integer := func() *big.Int {
		n := new(big.Int).SetUint64(rng.Uint64() | 1)
		n.Lsh(n, uint(rng.IntN(300)))
		n.Add(n, new(big.Int).SetUint64(rng.Uint64()))
		return n
	}
	point := func() proof.Xpt {
		return proof.Xpt{X: integer(), Y: new(big.Int).Neg(integer()), Z: integer(), W: integer()}
	}
	for trial := range 500 {
		a, b := point(), point()
		an, bn := integer(), new(big.Int).Neg(integer())
		if trial%2 == 0 {
			an.Neg(an)
			bn.Neg(bn)
		}
		if trial%3 == 0 {
			b.W.Set(a.W)
		}
		if trial%5 == 0 {
			a.X.SetInt64(0)
			b.Z.SetInt64(0)
		}
		sources := []*big.Int{a.X, a.Y, a.Z, a.W, b.X, b.Y, b.Z, b.W, an, bn}
		copies := make([]*big.Int, len(sources))
		for i, src := range sources {
			copies[i] = new(big.Int).Set(src)
		}
		va := new(big.Rat).SetFrac(an, a.W)
		vb := new(big.Rat).SetFrac(bn, b.W)
		tt := new(big.Rat).Quo(va, new(big.Rat).Sub(va, vb))
		got := proof.XplaneCrossing(a, b, an, bn)
		require.Positive(t, got.W.Sign(), "trial %d: weight", trial)
		for axis, nums := range [3][3]*big.Int{{a.X, b.X, got.X}, {a.Y, b.Y, got.Y}, {a.Z, b.Z, got.Z}} {
			av := new(big.Rat).SetFrac(nums[0], a.W)
			bv := new(big.Rat).SetFrac(nums[1], b.W)
			want := new(big.Rat).Add(av, new(big.Rat).Mul(tt, new(big.Rat).Sub(bv, av)))
			actual := new(big.Rat).SetFrac(nums[2], got.W)
			require.Zero(t, want.Cmp(actual), "trial %d: coordinate %d", trial, axis)
		}
		for i, src := range sources {
			require.Zero(t, copies[i].Cmp(src), "trial %d: source %d modified", trial, i)
		}
		for _, result := range []*big.Int{got.X, got.Y, got.Z, got.W} {
			result.Add(result, big.NewInt(1))
		}
		for i, src := range sources {
			require.Zero(t, copies[i].Cmp(src), "trial %d: result aliases source %d", trial, i)
		}
	}
}
