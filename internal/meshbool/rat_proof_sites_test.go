package meshbool

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// adversarialRats returns zero, signs, equal values in several spellings,
// huge and tiny numerators and denominators, and power-of-two denominators.
func adversarialRats() []*big.Rat {
	huge := new(big.Int).Exp(big.NewInt(10), big.NewInt(40), nil)
	pow70 := new(big.Int).Lsh(big.NewInt(1), 70)
	pow70p1 := new(big.Int).Add(pow70, big.NewInt(1))
	var out []*big.Rat
	add := func(n, d *big.Int) { out = append(out, new(big.Rat).SetFrac(n, d)) }
	for _, n := range []int64{0, 1, -1, 3, -3, 6, 7, -14, 5} {
		for _, d := range []int64{1, 2, 7, 14, 4, 1024} {
			add(big.NewInt(n), big.NewInt(d))
		}
	}
	// Values not in lowest terms: numerator and denominator scaled in place,
	// as no production path does (see proof.AddRat). Every comparison below is
	// by value, so the representation of a result does not matter.
	for _, u := range []struct{ n, d, k int64 }{
		{1, 9, 3}, {3, 7, 2}, {-5, 8, 3}, {7, 2, 2}, {1, 3, 1 << 20}, {-1, 5, 7},
	} {
		r := big.NewRat(u.n, u.d)
		r.Num().Mul(r.Num(), big.NewInt(u.k))
		r.Denom().Mul(r.Denom(), big.NewInt(u.k))
		out = append(out, r)
	}
	big70 := new(big.Rat).SetFrac(big.NewInt(5), big.NewInt(7))
	big70.Num().Lsh(big70.Num(), 70)
	big70.Denom().Lsh(big70.Denom(), 70)
	out = append(out, big70)
	zeroOverFive := new(big.Rat).SetFrac(big.NewInt(0), big.NewInt(1))
	zeroOverFive.Denom().SetInt64(5)
	out = append(out, zeroOverFive)
	add(huge, big.NewInt(3))
	add(new(big.Int).Neg(huge), big.NewInt(3))
	add(big.NewInt(1), huge)
	add(big.NewInt(-1), huge)
	add(big.NewInt(1), pow70)
	add(big.NewInt(-5), pow70)
	add(pow70p1, big.NewInt(1))
	add(pow70p1, pow70)
	add(pow70p1, huge)
	add(huge, pow70p1)
	return out
}

func rawCross2x(a, b, c Xp2) *big.Rat {
	bu := new(big.Rat).Sub(b.U, a.U)
	bv := new(big.Rat).Sub(b.V, a.V)
	cu := new(big.Rat).Sub(c.U, a.U)
	cv := new(big.Rat).Sub(c.V, a.V)
	return new(big.Rat).Sub(new(big.Rat).Mul(bu, cv), new(big.Rat).Mul(bv, cu))
}

// rawOnSegment2 is OnSegment2 with big.Rat's own arithmetic.
func rawOnSegment2(a, b, p Xp2) (bool, bool) {
	if rawCross2x(a, b, p).Sign() != 0 {
		return false, false
	}
	du := new(big.Rat).Sub(b.U, a.U)
	dv := new(big.Rat).Sub(b.V, a.V)
	var lo, hi, x *big.Rat
	if du.Sign() != 0 {
		lo, hi, x = a.U, b.U, p.U
	} else if dv.Sign() != 0 {
		lo, hi, x = a.V, b.V, p.V
	} else {
		eq := p.U.Cmp(a.U) == 0 && p.V.Cmp(a.V) == 0
		return eq, false
	}
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	if x.Cmp(lo) < 0 || x.Cmp(hi) > 0 {
		return false, false
	}
	return true, x.Cmp(lo) > 0 && x.Cmp(hi) < 0
}

// rawCross2xSign is the exact sign of rawCross2x.
func rawCross2xSign(a, b, c Xp2) int { return rawCross2x(a, b, c).Sign() }

// TestRatPredicatesMatchRawRat draws 40000 point triples from a coordinate
// pool that repeats values (so the shared-coordinate shortcuts fire) and
// collinear triples (so OnSegment2 reaches its range checks), and compares
// Cross2x, Cross2xSign and OnSegment2 with big.Rat's own arithmetic.
func TestRatPredicatesMatchRawRat(t *testing.T) {
	pool := adversarialRats()
	rng := rand.New(rand.NewPCG(7, 11))
	pick := func() *big.Rat { return new(big.Rat).Set(pool[rng.IntN(len(pool))]) }
	point := func() Xp2 { return NewXP2(pick(), pick()) }
	for range 20000 {
		a, b, c := point(), point(), point()
		// Repeat a coordinate between two points on half the draws.
		switch rng.IntN(5) {
		case 0:
			b = NewXP2(a.U, b.V)
		case 1:
			b = NewXP2(b.U, a.V)
		case 2:
			c = NewXP2(a.U, c.V)
		case 3:
			c = NewXP2(c.U, a.V)
		}
		require.Zero(t, rawCross2x(a, b, c).Cmp(Cross2x(a, b, c)))
		require.Equal(t, rawCross2xSign(a, b, c), Cross2xSign(a, b, c))
		wantOn, wantIn := rawOnSegment2(a, b, c)
		gotOn, gotIn := OnSegment2(a, b, c)
		require.Equal(t, wantOn, gotOn)
		require.Equal(t, wantIn, gotIn)
	}
	ts := []*big.Rat{
		big.NewRat(0, 1), big.NewRat(1, 1), big.NewRat(1, 3), big.NewRat(1, 2),
		big.NewRat(2, 1), big.NewRat(-1, 1), big.NewRat(7, 5),
	}
	for range 20000 {
		a, b := point(), point()
		tt := ts[rng.IntN(len(ts))]
		pu := new(big.Rat).Add(a.U, new(big.Rat).Mul(tt, new(big.Rat).Sub(b.U, a.U)))
		pv := new(big.Rat).Add(a.V, new(big.Rat).Mul(tt, new(big.Rat).Sub(b.V, a.V)))
		p := NewXP2(pu, pv)
		wantOn, wantIn := rawOnSegment2(a, b, p)
		gotOn, gotIn := OnSegment2(a, b, p)
		require.Equal(t, wantOn, gotOn)
		require.Equal(t, wantIn, gotIn)
		require.Equal(t, rawCross2xSign(a, b, p), Cross2xSign(a, b, p))
	}
}

// TestOnSegment2ZeroLengthSegmentHoldsOnlyItsPoint pins the degenerate
// branch: a segment whose ends coincide holds exactly that point.
func TestOnSegment2ZeroLengthSegmentHoldsOnlyItsPoint(t *testing.T) {
	a := NewXP2(big.NewRat(1, 3), big.NewRat(-2, 7))
	b := NewXP2(big.NewRat(2, 6), big.NewRat(-4, 14))
	cases := []struct {
		name string
		p    Xp2
		want bool
	}{
		{"same point", NewXP2(big.NewRat(1, 3), big.NewRat(-2, 7)), true},
		{"same u, other v", NewXP2(big.NewRat(1, 3), big.NewRat(-1, 7)), false},
		{"other u, same v", NewXP2(big.NewRat(2, 3), big.NewRat(-2, 7)), false},
		{"other both", NewXP2(big.NewRat(2, 3), big.NewRat(1, 7)), false},
	}
	for _, tc := range cases {
		on, interior := OnSegment2(a, b, tc.p)
		require.Equal(t, tc.want, on, tc.name)
		require.False(t, interior, tc.name)
	}
}

func rawRoundBound(p proof.Xpt, pf r3.Vec) float64 {
	px, py, pz := proof.XhpRat(proof.Xhp(p))
	worst := new(big.Rat)
	for _, pair := range [][2]*big.Rat{{px, polynomial.MustRatOf(pf.X)}, {py, polynomial.MustRatOf(pf.Y)}, {pz, polynomial.MustRatOf(pf.Z)}} {
		d := new(big.Rat).Sub(pair[0], pair[1])
		d.Abs(d)
		if d.Cmp(worst) > 0 {
			worst = d
		}
	}
	if worst.Sign() == 0 {
		return 0
	}
	w, _ := worst.Float64()
	return proofbound.Radius3D(proofbound.ProvenUpRound(w))
}

func rawCoordDistance(p proof.Xpt, v r3.Vec) *big.Rat {
	px, py, pz := proof.XhpRat(proof.Xhp(p))
	d := new(big.Rat)
	for _, pair := range [][2]*big.Rat{{px, polynomial.MustRatOf(v.X)}, {py, polynomial.MustRatOf(v.Y)}, {pz, polynomial.MustRatOf(v.Z)}} {
		dd := new(big.Rat).Sub(pair[0], pair[1])
		dd.Abs(dd)
		if dd.Cmp(d) > 0 {
			d = dd
		}
	}
	return d
}

func rawExactRigidImage(b r3.Basis, t, p r3.Vec) proof.Xpt {
	coord := func(ex, ey, ez, tc float64) *big.Rat {
		s := new(big.Rat).Mul(polynomial.MustRatOf(ex), polynomial.MustRatOf(p.X))
		s.Add(s, new(big.Rat).Mul(polynomial.MustRatOf(ey), polynomial.MustRatOf(p.Y)))
		s.Add(s, new(big.Rat).Mul(polynomial.MustRatOf(ez), polynomial.MustRatOf(p.Z)))
		return s.Add(s, polynomial.MustRatOf(tc))
	}
	x := coord(b.EX.X, b.EY.X, b.EZ.X, t.X)
	y := coord(b.EX.Y, b.EY.Y, b.EZ.Y, t.Y)
	z := coord(b.EX.Z, b.EY.Z, b.EZ.Z, t.Z)
	w := new(big.Int).Set(x.Denom())
	for _, r := range []*big.Rat{y, z} {
		g := new(big.Int).GCD(nil, nil, w, r.Denom())
		w.Mul(w, new(big.Int).Quo(r.Denom(), g))
	}
	num := func(r *big.Rat) *big.Int {
		return new(big.Int).Mul(r.Num(), new(big.Int).Quo(w, r.Denom()))
	}
	return proof.Xpt(proof.XhpStripTwosOwned(proof.Xhp{X: num(x), Y: num(y), Z: num(z), W: w}))
}

func randomXpt(rng *rand.Rand) proof.Xpt {
	coord := func() *big.Int {
		v := big.NewInt(rng.Int64N(2001) - 1000)
		if rng.IntN(4) == 0 {
			v.Lsh(v, uint(rng.IntN(90)))
		}
		return v
	}
	w := big.NewInt(rng.Int64N(50) + 1)
	if rng.IntN(3) == 0 {
		w.Lsh(w, uint(rng.IntN(70)))
	}
	return proof.Xpt{X: coord(), Y: coord(), Z: coord(), W: w}
}

func ratSiteFloat(rng *rand.Rand) float64 {
	switch rng.IntN(6) {
	case 0:
		return 0
	case 1:
		return float64(rng.Int64N(21) - 10)
	case 2:
		return (rng.Float64() - 0.5) * 1e-12
	case 3:
		return (rng.Float64() - 0.5) * 1e12
	default:
		return rng.Float64()*20 - 10
	}
}

// TestRoundingBoundsMatchRawRat compares PointRoundBound, CoordDistance and
// ExactRigidImage with the big.Rat bodies on 6000 random points each.
func TestRoundingBoundsMatchRawRat(t *testing.T) {
	rng := rand.New(rand.NewPCG(3, 5))
	vec := func() r3.Vec { return r3.NewVec(ratSiteFloat(rng), ratSiteFloat(rng), ratSiteFloat(rng)) }
	for range 6000 {
		p := randomXpt(rng)
		pf := vec()
		if rng.IntN(3) == 0 {
			pf = p.Vec()
		}
		require.Equal(t, rawRoundBound(p, pf), PointRoundBound(p, pf))
		require.Zero(t, rawCoordDistance(p, pf).Cmp(CoordDistance(p, pf)))
	}
	for range 6000 {
		b := r3.Basis{EX: vec(), EY: vec(), EZ: vec()}
		if rng.IntN(4) == 0 {
			b = r3.Identity().Basis()
		}
		tr, p := vec(), vec()
		want := rawExactRigidImage(b, tr, p)
		got := ExactRigidImage(b, tr, p)
		require.Zero(t, want.X.Cmp(got.X))
		require.Zero(t, want.Y.Cmp(got.Y))
		require.Zero(t, want.Z.Cmp(got.Z))
		require.Zero(t, want.W.Cmp(got.W))
	}
}

// TestChainProbeMidpointOfNonReducedEndpoint pins the value of a midpoint
// whose endpoint U is the non-reduced 3/9. The value is 1/6 by Cmp. The
// representation is 3/18, where big.Rat gives 1/6, so its 2D identity
// would be the text "3/18|0" and not "1/6|0"; no production input has such an endpoint, because
// every coordinate comes from a constructor that returns lowest terms
// (proof.TestNoRatWritesThroughNumDenom scans for any other writer).
func TestChainProbeMidpointOfNonReducedEndpoint(t *testing.T) {
	third := big.NewRat(1, 9)
	third.Num().SetInt64(3) // 3/9
	tc := &TriCutter{Work: proofbound.NewWorkBudget(t.Context())}
	a := NewXP2(third, new(big.Rat))
	b := NewXP2(new(big.Rat), new(big.Rat))
	tc.AddVert(a, proof.XptOf(r3.NewVec(0, 0, 0)), true)
	tc.AddVert(b, proof.XptOf(r3.NewVec(1, 0, 0)), true)
	mid, err := tc.ChainProbe(ChainPath{Verts: []int{0, 1}})
	require.NoError(t, err)
	require.Zero(t, mid.U.Cmp(big.NewRat(1, 6)))
	require.Zero(t, mid.V.Sign())
}
