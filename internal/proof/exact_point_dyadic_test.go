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

func TestXptDyV3OwnsExactCoordinates(t *testing.T) {
	t.Parallel()
	check := func(p proof.Xpt) {
		t.Helper()
		before := cloneXhp(proof.Xhp(p))
		got, ok := proof.XptDyV3(p)
		require.True(t, ok)
		var want [3]*big.Rat
		for i, n := range []*big.Int{p.X, p.Y, p.Z} {
			want[i] = new(big.Rat).SetFrac(n, p.W)
			require.Zero(t, want[i].Cmp(got[i].Rat()), "coordinate %d", i)
			if n.Sign() == 0 {
				require.Zero(t, got[i].Exp())
			} else {
				require.Equal(t, uint(1), got[i].Mant().Bit(0))
			}
		}
		requireSameXhp(t, before, proof.Xhp(p), "conversion input")
		for _, n := range []*big.Int{p.X, p.Y, p.Z, p.W} {
			n.SetInt64(-7)
			n.Lsh(n, 513)
		}
		for i := range got {
			require.Zero(t, want[i].Cmp(got[i].Rat()), "owned coordinate %d", i)
		}
	}
	for _, width := range []uint{1, 63, 64, 65, 126, 127, 128, 129, 257} {
		for _, shift := range []uint{0, 1, 31, 63, 64, 65, 127, 1080} {
			for _, sign := range []int64{1, -1} {
				n := new(big.Int).Sub(new(big.Int).Lsh(big.NewInt(1), width), big.NewInt(1))
				n.Lsh(n, shift)
				n.Mul(n, big.NewInt(sign))
				check(proof.Xpt{X: n, Y: new(big.Int).Neg(n), Z: new(big.Int),
					W: new(big.Int).Lsh(big.NewInt(1), shift/2+51)})
			}
		}
	}
	check(proof.XptOf(r3.Vec{X: math.MaxFloat64, Y: -math.SmallestNonzeroFloat64, Z: 0.1}))
	for _, weight := range []int64{0, -1, -2, 3, 6} {
		p := proof.Xpt{X: big.NewInt(3), Y: big.NewInt(-5), Z: new(big.Int), W: big.NewInt(weight)}
		before := cloneXhp(proof.Xhp(p))
		_, ok := proof.XptDyV3(p)
		require.False(t, ok, "weight %d", weight)
		requireSameXhp(t, before, proof.Xhp(p), "rejected weight")
	}
}

func TestXtriangleNormalMatchesHomogeneousArithmetic(t *testing.T) {
	t.Parallel()
	check := func(a, b, c proof.Xpt) {
		t.Helper()
		inputs := []proof.Xhp{proof.Xhp(a), proof.Xhp(b), proof.Xhp(c)}
		clones := []proof.Xhp{cloneXhp(inputs[0]), cloneXhp(inputs[1]), cloneXhp(inputs[2])}
		want := proof.Xcross(proof.Xsub(b, a), proof.Xsub(c, a))
		got := proof.XtriangleNormal(a, b, c)
		requireSameXhp(t, proof.Xhp(want), proof.Xhp(got), "normal")
		requireOwned(t, proof.Xhp(got), inputs, clones, "normal")
	}
	zero := proof.XptOf(r3.Vec{})
	check(zero, zero, zero)
	check(zero, proof.XptOf(r3.Vec{X: 1}), proof.XptOf(r3.Vec{X: 2}))
	check(proof.XptOf(r3.Vec{X: math.MaxFloat64, Y: math.SmallestNonzeroFloat64}),
		proof.XptOf(r3.Vec{X: -math.MaxFloat64, Z: math.SmallestNonzeroFloat64}),
		proof.XptOf(r3.Vec{Y: 0.1, Z: -math.MaxFloat64}))
	rng := rand.New(rand.NewPCG(0x1cd39a, 0x875df1))
	for range 200 {
		check(proof.Xpt(randXhp(rng, true)), proof.Xpt(randXhp(rng, true)), proof.Xpt(randXhp(rng, true)))
	}
	for range 80 {
		check(proof.Xpt(randXhp(rng, false)), proof.Xpt(randXhp(rng, false)), proof.Xpt(randXhp(rng, false)))
	}
	finite := func() float64 {
		for {
			f := math.Float64frombits(rng.Uint64())
			if !math.IsNaN(f) && !math.IsInf(f, 0) {
				return f
			}
		}
	}
	for range 40 {
		points := [3]proof.Xpt{}
		for i := range points {
			points[i] = proof.XptOf(r3.Vec{X: finite(), Y: finite(), Z: finite()})
		}
		check(points[0], points[1], points[2])
	}
	a, b, c := proof.XptOf(r3.Vec{X: 0.1}), proof.XptOf(r3.Vec{Y: 0.3}), proof.XptOf(r3.Vec{Z: -0.7})
	for _, denominator := range []int64{2, 3} {
		check(proof.Xlerp(a, b, big.NewInt(1), big.NewInt(denominator)),
			proof.Xlerp(b, c, big.NewInt(1), big.NewInt(denominator)), c)
	}
	image := func(p r3.Vec) proof.Xpt {
		return proof.ExactAffineImage(r3.Vec{X: 0.1, Y: 0.2}, r3.Vec{Y: 0.3, Z: -0.1},
			r3.Vec{X: -0.7, Z: 0.9}, r3.Vec{X: 0.4, Y: -0.2}, p)
	}
	check(image(r3.Vec{X: 0.1}), image(r3.Vec{Y: 0.3}), image(r3.Vec{Z: -0.7}))
	// Invalid weights must preserve the original expression's behavior too.
	for _, weight := range []int64{0, -1, -2, 3, 6} {
		p := proof.Xpt{X: big.NewInt(2), Y: big.NewInt(-3), Z: big.NewInt(5), W: big.NewInt(weight)}
		check(p, b, c)
		check(a, p, c)
		check(a, b, p)
	}
}
