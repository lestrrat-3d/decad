package decad

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// ratHelperPool returns reduced rationals that stress every branch of
// proof.AddRat, SubRat, MulRat and QuoRat: zero, negatives, integers whose
// denominator is not stored (FloatRat of a whole float), power-of-two and odd
// denominators, equal and opposite pairs, and numerators and denominators of
// several hundred bits.
func ratHelperPool() []*big.Rat {
	pool := []*big.Rat{
		new(big.Rat), big.NewRat(1, 1), big.NewRat(-1, 1), big.NewRat(1, 2), big.NewRat(-1, 2),
		big.NewRat(1, 3), big.NewRat(-7, 5), big.NewRat(9, 4), big.NewRat(-9, 4), big.NewRat(2, 1),
		big.NewRat(1, 6), big.NewRat(5, 6), big.NewRat(-5, 12),
	}
	for _, f := range []float64{
		0.1, -0.1, 3, -3, 1, 0.5, 1e300, -1e300, 1e-300, 5e-324, -5e-324,
		math.MaxFloat64, math.SmallestNonzeroFloat64 * 3, 4503599627370496, 1 << 62,
	} {
		pool = append(pool, proofarith.FloatRat(f))
	}
	for _, k := range []uint{1, 5, 52, 53, 60, 64, 65, 127, 200, 1023, 1074} {
		one := big.NewInt(1)
		pool = append(pool,
			new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(one, k)),
			new(big.Rat).SetFrac(big.NewInt(-3), new(big.Int).Lsh(one, k)),
			new(big.Rat).SetInt(new(big.Int).Lsh(one, k)),
			new(big.Rat).SetFrac(new(big.Int).Lsh(one, k), big.NewInt(3)),
		)
	}
	big40 := new(big.Int).Exp(big.NewInt(10), big.NewInt(40), nil)
	pool = append(pool,
		new(big.Rat).SetFrac(big40, new(big.Int).Add(big40, big.NewInt(1))),
		new(big.Rat).SetFrac(new(big.Int).Neg(big40), new(big.Int).Sub(big40, big.NewInt(1))),
		new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Add(big40, big.NewInt(7))),
	)
	return pool
}

func randomReducedRat(rng *rand.Rand) *big.Rat {
	bits := func() *big.Int {
		n := new(big.Int)
		for range rng.IntN(5) {
			n.Lsh(n, 61).Add(n, big.NewInt(rng.Int64()))
		}
		return n.Add(n, big.NewInt(rng.Int64N(1000)))
	}
	n, d := bits(), bits()
	if d.Sign() == 0 {
		d.SetInt64(1)
	}
	if rng.IntN(2) == 0 {
		n.Neg(n)
	}
	return new(big.Rat).SetFrac(n, d)
}

func ratSnapshot(q *big.Rat) string { return q.Num().String() + "/" + q.Denom().String() }

type ratHelperCase struct {
	name string
	got  func(a, b *big.Rat) *big.Rat
	want func(a, b *big.Rat) *big.Rat
	div  bool
}

func ratHelperCases() []ratHelperCase {
	add := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
	sub := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
	mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
	quo := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Quo(a, b) }
	return []ratHelperCase{
		{"lfAdd", lfAdd, add, false}, {"lfSub", lfSub, sub, false},
		{"lfMul", lfMul, mul, false}, {"lfDiv", lfDiv, quo, true},
		{"pointPlaneAdd", pointPlaneAdd, add, false}, {"pointPlaneSub", pointPlaneSub, sub, false},
		{"pointPlaneMul", pointPlaneMul, mul, false},
	}
}

// requireRatPairMatches checks one operand pair for one helper: the value by
// Cmp, the stored numerator and denominator by text, and that neither operand
// changed.
func requireRatPairMatches(t *testing.T, c ratHelperCase, a, b *big.Rat) {
	t.Helper()
	if c.div && b.Sign() == 0 {
		return
	}
	beforeA, beforeB := ratSnapshot(a), ratSnapshot(b)
	got, want := c.got(a, b), c.want(a, b)
	require.Zero(t, got.Cmp(want), "%s(%s, %s)", c.name, beforeA, beforeB)
	require.Equal(t, ratSnapshot(want), ratSnapshot(got), "%s(%s, %s)", c.name, beforeA, beforeB)
	require.Equal(t, beforeA, ratSnapshot(a), "%s changed its first operand", c.name)
	require.Equal(t, beforeB, ratSnapshot(b), "%s changed its second operand", c.name)
	require.False(t, got == a || got == b, "%s must return a fresh Rat", c.name)
}

func TestLoftFaceRatHelpersMatchRawRat(t *testing.T) {
	pool := ratHelperPool()
	for _, c := range ratHelperCases() {
		t.Run(c.name, func(t *testing.T) {
			for _, a := range pool {
				for _, b := range pool {
					requireRatPairMatches(t, c, a, b)
				}
			}
			rng := rand.New(rand.NewPCG(7, 11))
			for range 1500 {
				requireRatPairMatches(t, c, randomReducedRat(rng), randomReducedRat(rng))
			}
			for _, a := range pool {
				requireRatPairMatches(t, c, a, a)
				requireRatPairMatches(t, c, a, new(big.Rat).Neg(a))
			}
		})
	}
}

func ratPanicValue(f func()) (value any) {
	defer func() { value = recover() }()
	f()
	return nil
}

func TestLoftFaceDivByZeroPanicsLikeQuo(t *testing.T) {
	for _, a := range []*big.Rat{new(big.Rat), big.NewRat(3, 7), big.NewRat(-1, 2), proofarith.FloatRat(2)} {
		zero := new(big.Rat)
		want := ratPanicValue(func() { new(big.Rat).Quo(a, zero) })
		require.NotNil(t, want)
		got := ratPanicValue(func() { lfDiv(a, zero) })
		require.Equal(t, fmt.Sprint(want), fmt.Sprint(got))
	}
}

// The slope in pointPlaneCapDroppedCoordinate is one of the three quotients
// pointPlaneCapCentroid takes; this test compares it with the raw expression.
func TestPointPlaneCapDroppedCoordinateMatchesRawRat(t *testing.T) {
	normals := []*big.Rat{
		big.NewRat(3, 5), big.NewRat(-4, 5), big.NewRat(1, 1), big.NewRat(-7, 2),
		proofarith.FloatRat(0.6), proofarith.FloatRat(-0.8), proofarith.FloatRat(2),
	}
	rng := rand.New(rand.NewPCG(3, 5))
	for _, nx := range normals {
		for _, ny := range normals {
			for keep := range 2 {
				plane := &pointPlaneLimit{nx: nx, ny: ny}
				coords := func() [3]*big.Rat {
					return [3]*big.Rat{randomReducedRat(rng), randomReducedRat(rng), randomReducedRat(rng)}
				}
				center, anchor := coords(), coords()
				drop := 1 - keep
				normal := [2]*big.Rat{nx, ny}
				if normal[drop].Sign() == 0 {
					continue
				}
				slope := new(big.Rat).Quo(normal[keep], normal[drop])
				diff := new(big.Rat).Sub(center[keep], anchor[keep])
				want := new(big.Rat).Sub(anchor[drop], new(big.Rat).Mul(slope, diff))
				got := pointPlaneCapDroppedCoordinate(center, anchor, plane, keep)
				require.Zero(t, got.Cmp(want))
				require.Equal(t, ratSnapshot(want), ratSnapshot(got))
			}
		}
	}
}

// nonReducedRat returns n/d stored as given, for example 3/9. Writing through
// Num is allowed in test files (the guard skips them). No production input
// reaches such a Rat: every producer reduces, so the helpers' lowest-terms
// precondition holds there. For these operands the helpers still return the
// exact value, but the stored Num and Denom may differ from big.Rat's, so only
// the value is compared.
func nonReducedRat(n, d int64) *big.Rat {
	q := big.NewRat(1, d)
	q.Num().SetInt64(n)
	return q
}

func TestLoftFaceRatHelpersAcceptNonReducedOperandsByValue(t *testing.T) {
	operands := []*big.Rat{
		nonReducedRat(3, 9), nonReducedRat(-3, 9), nonReducedRat(6, 4), nonReducedRat(0, 1), big.NewRat(1, 3),
	}
	for _, c := range ratHelperCases() {
		for _, a := range operands {
			for _, b := range operands {
				if c.div && b.Sign() == 0 {
					continue
				}
				require.Zero(t, c.got(a, b).Cmp(c.want(a, b)), "%s(%s, %s)", c.name, ratSnapshot(a), ratSnapshot(b))
			}
		}
	}
}

func TestPointPlaneCapDroppedCoordinateAcceptsNonReducedByValue(t *testing.T) {
	plane := &pointPlaneLimit{nx: nonReducedRat(3, 9), ny: nonReducedRat(-6, 9)}
	center := [3]*big.Rat{big.NewRat(2, 3), big.NewRat(5, 7), big.NewRat(-1, 4)}
	anchor := [3]*big.Rat{big.NewRat(1, 5), big.NewRat(-2, 3), big.NewRat(3, 8)}
	for keep := range 2 {
		drop := 1 - keep
		normal := [2]*big.Rat{plane.nx, plane.ny}
		slope := new(big.Rat).Quo(normal[keep], normal[drop])
		want := new(big.Rat).Sub(anchor[drop], new(big.Rat).Mul(slope, new(big.Rat).Sub(center[keep], anchor[keep])))
		got := pointPlaneCapDroppedCoordinate(center, anchor, plane, keep)
		require.Zero(t, got.Cmp(want))
	}
}
