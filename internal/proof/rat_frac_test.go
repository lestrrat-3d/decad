package proof_test

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func requireSameRat(t *testing.T, want, got *big.Rat, msg string) {
	t.Helper()
	require.Zero(t, want.Num().Cmp(got.Num()), "%s: numerator", msg)
	require.Zero(t, want.Denom().Cmp(got.Denom()), "%s: denominator", msg)
}

func TestRatFracMatchesSetFrac(t *testing.T) {
	rng := rand.New(rand.NewPCG(29, 31))
	for trial := range 5000 {
		n := new(big.Int)
		if rng.IntN(8) != 0 {
			for range rng.IntN(5) + 1 {
				n.Lsh(n, 64)
				n.Add(n, new(big.Int).SetUint64(rng.Uint64()))
			}
			n.Lsh(n, uint(rng.IntN(150)))
			if rng.IntN(2) == 0 {
				n.Neg(n)
			}
		}
		var d *big.Int
		if rng.IntN(3) != 0 {
			d = new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(301)))
		} else {
			d = new(big.Int).SetUint64(rng.Uint64()>>uint(rng.IntN(60)) | 1)
			if rng.IntN(2) == 0 {
				d.Lsh(d, uint(rng.IntN(20)))
			}
		}
		n0, d0 := new(big.Int).Set(n), new(big.Int).Set(d)
		want := new(big.Rat).SetFrac(n, d)
		got := proof.RatFrac(new(big.Rat), n, d)
		requireSameRat(t, want, got, "trial")
		require.Zero(t, n.Cmp(n0), "n modified, trial %d", trial)
		require.Zero(t, d.Cmp(d0), "d modified, trial %d", trial)
		// The result owns its storage: changing n must not change it.
		n.Add(n, big.NewInt(1))
		requireSameRat(t, want, got, "aliasing")
	}
}

func TestRatFracEdges(t *testing.T) {
	for _, tc := range []struct{ n, d int64 }{
		{0, 1}, {0, 2}, {0, 1 << 40}, {0, 3}, {1, 1}, {-1, 1}, {-1, 2}, {4, 8}, {-4, 8}, {8, 8}, {-8, 8},
		{6, 4}, {-6, 4}, {3, 6}, {-3, 6}, {7, 1 << 62}, {1 << 62, 1 << 62}, {12, 9}, {5, 1},
	} {
		n, d := big.NewInt(tc.n), big.NewInt(tc.d)
		requireSameRat(t, new(big.Rat).SetFrac(n, d), proof.RatFrac(new(big.Rat), n, d), "edge")
	}
	// A negative denominator takes the SetFrac path.
	n, d := big.NewInt(3), big.NewInt(-4)
	requireSameRat(t, new(big.Rat).SetFrac(n, d), proof.RatFrac(new(big.Rat), n, d), "negative denominator")
	// A receiver holding a previous fraction is overwritten correctly.
	z := big.NewRat(7, 12)
	requireSameRat(t, big.NewRat(5, 2), proof.RatFrac(z, big.NewInt(10), big.NewInt(4)), "reused receiver")
	requireSameRat(t, big.NewRat(0, 1), proof.RatFrac(z, big.NewInt(0), big.NewInt(4)), "reused receiver zero")
}

func TestXdotRatMatchesSetFrac(t *testing.T) {
	rng := rand.New(rand.NewPCG(37, 41))
	scaled := func() float64 {
		return (rng.Float64() - 0.5) * float64(uint64(1)<<uint(rng.IntN(40))) * 1e-3
	}
	pt := func() proof.Xpt {
		p := proof.XptOf(r3.Vec{X: scaled(), Y: scaled(), Z: scaled()})
		if rng.IntN(3) == 0 {
			// A homogeneous point with a non-power-of-two weight.
			w := big.NewInt(int64(rng.IntN(1000)*2 + 1))
			p = proof.Xpt{X: p.X, Y: p.Y, Z: p.Z, W: w}
		}
		return p
	}
	for range 300 {
		a, b := pt(), pt()
		den := new(big.Int).Mul(a.W, b.W)
		want := new(big.Rat).SetFrac(proof.XdotNum(a, b), den)
		requireSameRat(t, want, proof.XdotRat(a, b), "XdotRat")
	}
}

// denomStored reports whether r holds its denominator, so that Denom returns
// the same pointer on every call, rather than a fresh detached Int.
func denomStored(r *big.Rat) bool {
	first, second := r.Denom(), r.Denom()
	return first == second
}

func TestXhpRatMatchesSetFrac(t *testing.T) {
	rng := rand.New(rand.NewPCG(43, 47))
	coord := func() *big.Int {
		n := new(big.Int)
		if rng.IntN(6) != 0 {
			for range rng.IntN(4) + 1 {
				n.Lsh(n, 64).Add(n, new(big.Int).SetUint64(rng.Uint64()))
			}
			n.Lsh(n, uint(rng.IntN(100)))
			if rng.IntN(2) == 0 {
				n.Neg(n)
			}
		}
		return n
	}
	for range 3000 {
		var w *big.Int
		switch rng.IntN(3) {
		case 0:
			w = new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(300)))
		case 1:
			w = new(big.Int).SetUint64(rng.Uint64() | 1)
		default:
			w = new(big.Int).Lsh(big.NewInt(int64(rng.IntN(1000)*2+1)), uint(rng.IntN(20)))
		}
		p := proof.Xhp{X: coord(), Y: coord(), Z: coord(), W: w}
		x, y, z := proof.XhpRat(p)
		requireSameRat(t, new(big.Rat).SetFrac(p.X, p.W), x, "x")
		requireSameRat(t, new(big.Rat).SetFrac(p.Y, p.W), y, "y")
		requireSameRat(t, new(big.Rat).SetFrac(p.Z, p.W), z, "z")
		for _, c := range []struct {
			n   *big.Int
			got *big.Rat
		}{{p.X, x}, {p.Y, y}, {p.Z, z}} {
			want := new(big.Rat).SetFrac(c.n, p.W)
			require.Equal(t, want.IsInt(), c.got.IsInt())
			// SetFrac stores the denominator, so Denom returns one pointer.
			require.Equal(t, denomStored(want), denomStored(c.got), "stored denominator")
		}
		// The results own their storage: changing the point must not change them.
		wantX := new(big.Rat).Set(x)
		p.X.Add(p.X, big.NewInt(1))
		p.W.Add(p.W, big.NewInt(2))
		requireSameRat(t, wantX, x, "aliasing")
	}
}
