package proof_test

import (
	"math/big"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The oracles below are the bodies XhpCanon, XhpCross, Xsub and XhpOrientSign
// had before the power-of-two weight paths; the tests compare the shipped
// functions against them on exact values and on whether results alias.

func oracleCanon(p proof.Xhp) proof.Xhp {
	g := new(big.Int).Set(p.W)
	for _, v := range [3]*big.Int{p.X, p.Y, p.Z} {
		if v.Sign() == 0 {
			continue
		}
		g.GCD(nil, nil, g, v)
	}
	return proof.Xhp{
		X: new(big.Int).Quo(p.X, g),
		Y: new(big.Int).Quo(p.Y, g),
		Z: new(big.Int).Quo(p.Z, g),
		W: new(big.Int).Quo(p.W, g),
	}
}

func oracleCross(a, b proof.Xhp) proof.Xhp {
	var term big.Int
	axis := func(a0, b0, a1, b1 *big.Int) *big.Int {
		out := new(big.Int).Mul(a0, b0)
		return out.Sub(out, term.Mul(a1, b1))
	}
	return proof.Xhp{
		X: axis(a.Y, b.Z, a.Z, b.Y),
		Y: axis(a.Z, b.X, a.X, b.Z),
		Z: axis(a.X, b.Y, a.Y, b.X),
		W: new(big.Int).Mul(a.W, b.W),
	}
}

func oracleXsub(a, b proof.Xpt) proof.Xpt {
	return proof.Xpt(proof.XhpStripTwosOwned(proof.XhpSub(proof.Xhp(a), proof.Xhp(b))))
}

func oracleOrientSign(a, b, c, d proof.Xhp) int {
	ba, ca, da := proof.XhpSub(b, a), proof.XhpSub(c, a), proof.XhpSub(d, a)
	return proof.XhpDotNum(oracleCross(ba, ca), da).Sign()
}

// requireSameInt compares value, decimal spelling and limbs. Nil-ness of the
// limb slice of a zero is not part of the comparison; no caller can see it
// through the big.Int API except by comparing Bits() with nil.
func requireSameInt(t *testing.T, want, got *big.Int, msg string) {
	if sameIntRepresentation(want, got) {
		return
	}
	t.Helper()
	require.NotNil(t, got, msg)
	require.Zero(t, want.Cmp(got), msg)
	require.Equal(t, want.Sign(), got.Sign(), msg)
	require.Equal(t, want.String(), got.String(), msg)
	require.True(t, slices.Equal(want.Bits(), got.Bits()), msg)
}

func sameIntRepresentation(want, got *big.Int) bool {
	return got != nil && want.Cmp(got) == 0 && want.Sign() == got.Sign() &&
		want.String() == got.String() && slices.Equal(want.Bits(), got.Bits())
}

func requireSameXhp(t *testing.T, want, got proof.Xhp, msg string) {
	if sameIntRepresentation(want.X, got.X) && sameIntRepresentation(want.Y, got.Y) &&
		sameIntRepresentation(want.Z, got.Z) && sameIntRepresentation(want.W, got.W) {
		return
	}
	t.Helper()
	requireSameInt(t, want.X, got.X, msg+" X")
	requireSameInt(t, want.Y, got.Y, msg+" Y")
	requireSameInt(t, want.Z, got.Z, msg+" Z")
	requireSameInt(t, want.W, got.W, msg+" W")
}

func cloneXhp(p proof.Xhp) proof.Xhp {
	return proof.Xhp{
		X: new(big.Int).Set(p.X), Y: new(big.Int).Set(p.Y),
		Z: new(big.Int).Set(p.Z), W: new(big.Int).Set(p.W),
	}
}

// requireOwned checks that the four integers of got are distinct pointers from
// each other and from every integer of the inputs, and that changing them
// leaves the inputs equal to their clones.
func requireOwned(t *testing.T, got proof.Xhp, inputs []proof.Xhp, clones []proof.Xhp, msg string) {
	t.Helper()
	seen := map[*big.Int]struct{}{}
	for _, in := range inputs {
		for _, v := range []*big.Int{in.X, in.Y, in.Z, in.W} {
			seen[v] = struct{}{}
		}
	}
	for _, v := range []*big.Int{got.X, got.Y, got.Z, got.W} {
		_, dup := seen[v]
		if dup {
			require.False(t, dup, msg+" result integer is shared")
		}
		seen[v] = struct{}{}
	}
	for _, v := range []*big.Int{got.X, got.Y, got.Z, got.W} {
		v.Add(v, big.NewInt(1))
		v.Lsh(v, 3)
	}
	for i, in := range inputs {
		requireSameXhp(t, clones[i], in, msg+" input changed")
	}
}

func randInteger(r *rand.Rand) *big.Int {
	var v *big.Int
	switch r.IntN(8) {
	case 0:
		return new(big.Int)
	case 1:
		v = big.NewInt(int64(r.IntN(9) + 1))
	default:
		bits := r.IntN(260) + 1
		v = new(big.Int)
		for range bits/32 + 1 {
			v.Lsh(v, 32)
			v.Or(v, big.NewInt(int64(r.Uint32())))
		}
		v.Rsh(v, uint(((bits/32)+1)*32-bits))
		v.SetBit(v, bits-1, 1)
	}
	if r.IntN(3) == 0 {
		v.Lsh(v, uint(r.IntN(220)))
	}
	if r.IntN(2) == 0 {
		v.Neg(v)
	}
	return v
}

var pow2Exps = []uint{0, 1, 2, 3, 5, 31, 52, 53, 63, 64, 65, 127, 128, 129, 200, 1074, 1100}

// randWeight returns 2^k mostly, and a weight the power-of-two paths must
// leave to the general code (3, 6, 3*2^k, 1+2^k, -2^k) the rest of the time.
func randWeight(r *rand.Rand, pow2Only bool) *big.Int {
	k := pow2Exps[r.IntN(len(pow2Exps))]
	w := new(big.Int).Lsh(big.NewInt(1), k)
	if pow2Only {
		return w
	}
	switch r.IntN(8) {
	case 0:
		return big.NewInt(3)
	case 1:
		return w.Mul(w, big.NewInt(3))
	case 2:
		return w.Add(w, big.NewInt(1))
	case 3:
		return w.Neg(w)
	}
	return w
}

func randXhp(r *rand.Rand, pow2Only bool) proof.Xhp {
	return proof.Xhp{X: randInteger(r), Y: randInteger(r), Z: randInteger(r), W: randWeight(r, pow2Only)}
}

func TestXhpCanonPow2MatchesGCD(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(1, 2))
	cases := []proof.Xhp{
		{X: big.NewInt(0), Y: big.NewInt(0), Z: big.NewInt(0), W: big.NewInt(1)},
		{X: big.NewInt(0), Y: big.NewInt(0), Z: big.NewInt(0), W: big.NewInt(1 << 20)},
		{X: big.NewInt(0), Y: big.NewInt(0), Z: big.NewInt(0), W: big.NewInt(-8)},
		{X: big.NewInt(-6), Y: big.NewInt(4), Z: big.NewInt(0), W: big.NewInt(1)},
		{X: big.NewInt(-6), Y: big.NewInt(4), Z: big.NewInt(0), W: big.NewInt(2)},
		{X: big.NewInt(-6), Y: big.NewInt(4), Z: big.NewInt(0), W: big.NewInt(64)},
		{X: big.NewInt(-6), Y: big.NewInt(4), Z: big.NewInt(0), W: big.NewInt(-64)},
		{X: big.NewInt(-6), Y: big.NewInt(4), Z: big.NewInt(0), W: big.NewInt(6)},
		{X: big.NewInt(7), Y: big.NewInt(7), Z: big.NewInt(7), W: big.NewInt(7)},
		{X: big.NewInt(12), Y: big.NewInt(12), Z: big.NewInt(12), W: big.NewInt(12)},
	}
	for range 20000 {
		cases = append(cases, randXhp(r, r.IntN(4) != 0))
	}
	for i, p := range cases {
		clone := cloneXhp(p)
		allZero := p.X.Sign() == 0 && p.Y.Sign() == 0 && p.Z.Sign() == 0
		if allZero && p.W.Sign() == 0 {
			continue
		}
		want := oracleCanon(p)
		got := proof.XhpCanon(p)
		requireSameXhp(t, want, got, "canon")
		requireOwned(t, got, []proof.Xhp{p}, []proof.Xhp{clone}, "canon")
		if t.Failed() {
			t.Fatalf("case %d", i)
		}
	}
}

func TestXhpCanonPow2KeepsPointKey(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(3, 4))
	for range 2000 {
		v := r3.Vec{X: r.NormFloat64() * 1e3, Y: float64(r.IntN(9) - 4), Z: r.NormFloat64() * 1e-9}
		p := proof.XptOf(v)
		k := uint(r.IntN(150))
		scaled := proof.Xpt{
			X: new(big.Int).Lsh(p.X, k), Y: new(big.Int).Lsh(p.Y, k),
			Z: new(big.Int).Lsh(p.Z, k), W: new(big.Int).Lsh(p.W, k),
		}
		require.Equal(t, p.Key(), scaled.Key())
	}
}

func TestXhpCrossPow2WeightMatchesProduct(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(5, 6))
	for i := range 20000 {
		a, b := randXhp(r, r.IntN(3) != 0), randXhp(r, r.IntN(3) != 0)
		ca, cb := cloneXhp(a), cloneXhp(b)
		want := oracleCross(a, b)
		got := proof.XhpCross(a, b)
		requireSameXhp(t, want, got, "cross")
		requireOwned(t, got, []proof.Xhp{a, b}, []proof.Xhp{ca, cb}, "cross")
		if t.Failed() {
			t.Fatalf("case %d", i)
		}
	}
}

func TestXsubAlignedMatchesProductForm(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(7, 8))
	for i := range 20000 {
		a, b := randXhp(r, r.IntN(4) != 0), randXhp(r, r.IntN(4) != 0)
		if r.IntN(8) == 0 {
			b = proof.Xhp{X: new(big.Int).Set(a.X), Y: new(big.Int).Set(a.Y), Z: new(big.Int).Set(a.Z), W: new(big.Int).Set(a.W)}
		}
		ca, cb := cloneXhp(a), cloneXhp(b)
		want := oracleXsub(proof.Xpt(a), proof.Xpt(b))
		got := proof.Xsub(proof.Xpt(a), proof.Xpt(b))
		requireSameXhp(t, proof.Xhp(want), proof.Xhp(got), "xsub")
		requireOwned(t, proof.Xhp(got), []proof.Xhp{a, b}, []proof.Xhp{ca, cb}, "xsub")
		if t.Failed() {
			t.Fatalf("case %d", i)
		}
	}
}

func TestXsubAlignedOnFloatLifts(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(9, 10))
	scales := []float64{1, 1e-12, 1e9, 0x1p-80, 0x1p90}
	vec := func() r3.Vec {
		s := scales[r.IntN(len(scales))]
		return r3.Vec{X: r.NormFloat64() * s, Y: float64(r.IntN(5)-2) * s, Z: r.NormFloat64() * s * 0x1p-30}
	}
	for range 5000 {
		a, b := proof.XptOf(vec()), proof.XptOf(vec())
		requireSameXhp(t, proof.Xhp(oracleXsub(a, b)), proof.Xhp(proof.Xsub(a, b)), "lift")
		requireSameXhp(t, proof.Xhp(oracleXsub(a, a)), proof.Xhp(proof.Xsub(a, a)), "self")
	}
}

func TestXhpOrientSignPow2MatchesProductForm(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(11, 12))
	coord := func() float64 { return float64(r.IntN(9)-4) * float64(int(1)<<r.IntN(6)) }
	lift := func(v r3.Vec) proof.Xhp {
		p := proof.XhpOf(v)
		if k := uint(r.IntN(70)); r.IntN(2) == 0 {
			return proof.Xhp{
				X: p.X.Lsh(p.X, k), Y: p.Y.Lsh(p.Y, k), Z: p.Z.Lsh(p.Z, k), W: p.W.Lsh(p.W, k),
			}
		}
		return p
	}
	for i := range 20000 {
		pts := [4]r3.Vec{}
		for j := range pts {
			pts[j] = r3.Vec{X: coord(), Y: coord(), Z: coord()}
		}
		if i%3 == 0 {
			// d = a + s(b-a) + t(c-a) with small integers: coplanar.
			s, u := float64(r.IntN(7)-3), float64(r.IntN(7)-3)
			pts[3] = r3.Vec{
				X: pts[0].X + s*(pts[1].X-pts[0].X) + u*(pts[2].X-pts[0].X),
				Y: pts[0].Y + s*(pts[1].Y-pts[0].Y) + u*(pts[2].Y-pts[0].Y),
				Z: pts[0].Z + s*(pts[1].Z-pts[0].Z) + u*(pts[2].Z-pts[0].Z),
			}
		}
		if i%5 == 0 {
			pts[r.IntN(4)] = pts[r.IntN(4)]
		}
		a, b, c, d := lift(pts[0]), lift(pts[1]), lift(pts[2]), lift(pts[3])
		if i%7 == 0 {
			// A weight that is not a power of two takes the general path.
			c = proof.XhpLerp(a, c, big.NewInt(1), big.NewInt(3))
		}
		want := oracleOrientSign(a, b, c, d)
		require.Equal(t, want, proof.XhpOrientSign(a, b, c, d), "case %d", i)
		if i%3 == 0 && i%7 != 0 {
			require.Zero(t, want, "coplanar case %d", i)
		}
	}
}
