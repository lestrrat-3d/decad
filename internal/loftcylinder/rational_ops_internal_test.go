package loftcylinder

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func legacySourceDot(a, b XY) *big.Rat {
	return new(big.Rat).Add(new(big.Rat).Mul(a.X, b.X), new(big.Rat).Mul(a.Y, b.Y))
}

func legacySourceSub(a, b XY) XY {
	return XY{X: new(big.Rat).Sub(a.X, b.X), Y: new(big.Rat).Sub(a.Y, b.Y)}
}

func legacySourceLerp(a, b XY, t *big.Rat) XY {
	d := legacySourceSub(b, a)
	return XY{
		X: new(big.Rat).Add(a.X, new(big.Rat).Mul(t, d.X)),
		Y: new(big.Rat).Add(a.Y, new(big.Rat).Mul(t, d.Y)),
	}
}

func legacySourceRadiusMin2(a, b XY) *big.Rat {
	d := legacySourceSub(b, a)
	den := legacySourceDot(d, d)
	if den.Sign() == 0 {
		return legacySourceDot(a, a)
	}
	t := new(big.Rat).Quo(new(big.Rat).Neg(legacySourceDot(a, d)), den)
	if t.Sign() < 0 {
		t.SetInt64(0)
	} else if t.Cmp(big.NewRat(1, 1)) > 0 {
		t.SetInt64(1)
	}
	p := legacySourceLerp(a, b, t)
	return legacySourceDot(p, p)
}

func legacySourcePointAt(rays [2]sourceRay, s *big.Rat) XY {
	u := s
	ray := rays[0]
	if s.Cmp(big.NewRat(1, 1)) > 0 {
		ray = rays[1]
		u = new(big.Rat).Sub(s, big.NewRat(1, 1))
	}
	return XY{
		X: new(big.Rat).Add(ray.origin.X, new(big.Rat).Mul(u, ray.direction.X)),
		Y: new(big.Rat).Add(ray.origin.Y, new(big.Rat).Mul(u, ray.direction.Y)),
	}
}

func legacySourceProductRange(aLo, aHi, bLo, bHi *big.Rat) (*big.Rat, *big.Rat) {
	products := [4]*big.Rat{
		new(big.Rat).Mul(aLo, bLo), new(big.Rat).Mul(aLo, bHi),
		new(big.Rat).Mul(aHi, bLo), new(big.Rat).Mul(aHi, bHi),
	}
	lo, hi := products[0], products[0]
	for _, p := range products[1:] {
		if p.Cmp(lo) < 0 {
			lo = p
		}
		if p.Cmp(hi) > 0 {
			hi = p
		}
	}
	return lo, hi
}

func legacyRingDot(a, b RingVec) *big.Rat {
	q := new(big.Rat)
	for i := range a {
		q.Add(q, new(big.Rat).Mul(a[i], b[i]))
	}
	return q
}

func legacyRingCross(a, b RingVec) RingVec {
	return RingVec{
		new(big.Rat).Sub(new(big.Rat).Mul(a[1], b[2]), new(big.Rat).Mul(a[2], b[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[2], b[0]), new(big.Rat).Mul(a[0], b[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])),
	}
}

func requireSameRat(t *testing.T, want, got *big.Rat) {
	t.Helper()
	require.Zero(t, want.Num().Cmp(got.Num()), "numerator %v != %v", want, got)
	require.Zero(t, want.Denom().Cmp(got.Denom()), "denominator %v != %v", want, got)
}

func requireSameXY(t *testing.T, want, got XY) {
	t.Helper()
	requireSameRat(t, want.X, got.X)
	requireSameRat(t, want.Y, got.Y)
}

// TestLoftCylinderRationalOpsMatchBigRat compares the exact source and ring
// helpers, which compute through the proof rational helpers, with their
// big.Rat bodies on mixed dyadic, general, zero and negative operands.
func TestLoftCylinderRationalOpsMatchBigRat(t *testing.T) {
	rng := rand.New(rand.NewPCG(19, 23))
	maxDen := new(big.Int).Lsh(big.NewInt(1), 120)
	maxDen.Mul(maxDen, new(big.Int).Exp(big.NewInt(3), big.NewInt(20), nil))
	randRat := func() *big.Rat {
		var r *big.Rat
		switch rng.IntN(6) {
		case 0:
			r = new(big.Rat)
		case 1:
			r = proofarith.FloatRat(float64(rng.IntN(9) - 4))
		case 2, 3:
			f := (rng.Float64()*2 - 1) * math.Pow(2, float64(rng.IntN(40)-20))
			r = proofarith.FloatRat(f)
		default:
			q := big.NewInt(int64(rng.IntN(1<<30)) + 1)
			q.Mul(q, new(big.Int).Exp(big.NewInt(3), big.NewInt(int64(rng.IntN(21))), nil))
			q.Lsh(q, uint(rng.IntN(100)))
			if q.Cmp(maxDen) > 0 {
				q.Set(maxDen)
			}
			r = new(big.Rat).SetFrac(big.NewInt(int64(rng.IntN(1<<31))-1<<30), q)
		}
		return r
	}
	randXY := func() XY { return XY{randRat(), randRat()} }
	randRing := func() RingVec { return RingVec{randRat(), randRat(), randRat()} }
	for range 2000 {
		a, b, c := randXY(), randXY(), randXY()
		tt := randRat()
		requireSameRat(t, legacySourceDot(a, b), SourceDot(a, b))
		requireSameXY(t, legacySourceLerp(a, b, tt), sourceLerp(a, b, tt))
		requireSameRat(t, legacySourceRadiusMin2(a, b), sourceRadiusMin2(a, b))
		if rng.IntN(8) == 0 {
			requireSameRat(t, legacySourceRadiusMin2(a, a), sourceRadiusMin2(a, a))
		}
		rays := [2]sourceRay{{origin: a, direction: b}, {origin: b, direction: c}}
		for _, s := range []*big.Rat{tt, new(big.Rat).Add(new(big.Rat).Mul(tt, tt), big.NewRat(1, 1)), randRat()} {
			requireSameXY(t, legacySourcePointAt(rays, s), sourcePointAt(rays, s))
		}
		x, y, z, w := randRat(), randRat(), randRat(), randRat()
		wantLo, wantHi := legacySourceProductRange(x, y, z, w)
		gotLo, gotHi := sourceProductRange(x, y, z, w)
		requireSameRat(t, wantLo, gotLo)
		requireSameRat(t, wantHi, gotHi)
		p, q := randRing(), randRing()
		requireSameRat(t, legacyRingDot(p, q), RingDot(p, q))
		requireSameRat(t, legacyRingDot(p, p), RingDot(p, p))
		wantCross, gotCross := legacyRingCross(p, q), RingCross(p, q)
		for i := range wantCross {
			requireSameRat(t, wantCross[i], gotCross[i])
		}
	}
}
