package loftface

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

// legacyScalarRange is the ScalarRange body that the linear-form version
// replaced: it dots the row with the full map image at each corner. The
// factored ScalarRange must return the same lowest-terms rationals.
func legacyScalarRange(m Map, row RatVec, corners [4][2]*big.Rat) Interval {
	var out Interval
	for i, corner := range corners {
		value := VDot(row, m.At(corner[0], corner[1]))
		if i == 0 {
			out = Interval{value, value}
			continue
		}
		if value.Cmp(out.Lo) < 0 {
			out.Lo = value
		}
		if value.Cmp(out.Hi) > 0 {
			out.Hi = value
		}
	}
	return out
}

// tryScalarRange runs f and returns its interval, or the value it panicked with.
func tryScalarRange(f func() Interval) (out Interval, panicked any) {
	defer func() { panicked = recover() }()
	return f(), nil
}

// requireScalarRangeMatches requires the factored and legacy bodies to agree
// on the interval (numerator and denominator of both ends) or on the panic.
func requireScalarRangeMatches(t *testing.T, m Map, row RatVec, corners [4][2]*big.Rat) {
	t.Helper()
	before := snapshotScalarRangeInputs(m, row, corners)
	defer func() {
		t.Helper()
		after := snapshotScalarRangeInputs(m, row, corners)
		for i := range before {
			requireRatEqual(t, before[i], after[i])
		}
	}()
	want, wantPanic := tryScalarRange(func() Interval { return legacyScalarRange(m, row, corners) })
	got, gotPanic := tryScalarRange(func() Interval { return ScalarRange(m, row, corners) })
	require.Equal(t, wantPanic, gotPanic)
	if wantPanic != nil {
		return
	}
	requireRatEqual(t, want.Lo, got.Lo)
	requireRatEqual(t, want.Hi, got.Hi)
}

// snapshotScalarRangeInputs returns fresh copies (no aliasing) of every
// rational that ScalarRange reads, so a call that mutates one in place shows.
func snapshotScalarRangeInputs(m Map, row RatVec, corners [4][2]*big.Rat) []*big.Rat {
	var out []*big.Rat
	add := func(x *big.Rat) {
		out = append(out, new(big.Rat).SetFrac(new(big.Int).Set(x.Num()), new(big.Int).Set(x.Denom())))
	}
	for _, vec := range []RatVec{row, m.Constant, m.Q0, m.QU, m.QV} {
		for _, x := range vec {
			add(x)
		}
	}
	for _, x := range []*big.Rat{m.G0, m.GU, m.GV} {
		add(x)
	}
	for _, c := range corners {
		add(c[0])
		add(c[1])
	}
	return out
}

func randScalarRangeVec(rng *rand.Rand) RatVec {
	return RatVec{randRatOperand(rng), randRatOperand(rng), randRatOperand(rng)}
}

// randScalarRangeMap draws every coefficient, the denominator ones included,
// from randRatOperand: zero, negative, huge, tiny and mixed-denominator
// values. The denominator may be zero or negative at a corner; ScalarRange
// never checks its sign.
func randScalarRangeMap(rng *rand.Rand) Map {
	return Map{Constant: randScalarRangeVec(rng), Q0: randScalarRangeVec(rng),
		QU: randScalarRangeVec(rng), QV: randScalarRangeVec(rng),
		G0: randRatOperand(rng), GU: randRatOperand(rng), GV: randRatOperand(rng)}
}

func TestScalarRangeMatchesLegacyRandom(t *testing.T) {
	rng := rand.New(rand.NewPCG(211, 223))
	for range 1500 {
		var corners [4][2]*big.Rat
		for i := range corners {
			corners[i] = [2]*big.Rat{randRatOperand(rng), randRatOperand(rng)}
		}
		requireScalarRangeMatches(t, randScalarRangeMap(rng), randScalarRangeVec(rng), corners)
	}
}

func TestScalarRangeMatchesLegacyTiles(t *testing.T) {
	rng := rand.New(rand.NewPCG(227, 229))
	zero := RatVec{new(big.Rat), new(big.Rat), new(big.Rat)}
	for trial := range 300 {
		m := scaledGateMap(rng, []int{-120, -40, 0, 40, 120}[trial%5], []int{-60, 0, 60}[(trial/5)%3])
		first := gateCoefficientVec(rng, 4)
		rows := []RatVec{first, zero, first, gateCoefficientVec(rng, 1e100), gateCoefficientVec(rng, 1e-100),
			{ratAdd(first[0], ratF(1e-300)), first[1], first[2]}}
		u0, u1, v0, v1 := gateTestBox(rng)
		switch trial % 4 {
		case 1:
			// A tiny tile: nearly equal corners.
			u1, v1 = ratAdd(u0, ratF(1e-200)), ratAdd(v0, ratF(1e-200))
		case 2:
			// A point tile: one repeated corner, so a one-element range.
			u1, v1 = u0, v0
		case 3:
			// A reversed tile: u1 < u0.
			u0, u1 = u1, u0
		}
		if trial%6 == 0 {
			m.GU, m.GV = new(big.Rat), new(big.Rat)
		}
		if trial%6 == 1 {
			m.QU, m.QV = zero, zero
		}
		for _, row := range rows {
			requireScalarRangeMatches(t, m, row, TileCorners(u0, u1, v0, v1))
		}
	}
}

func TestScalarRangeMatchesLegacyEdges(t *testing.T) {
	rng := rand.New(rand.NewPCG(233, 239))
	half := big.NewRat(1, 2)
	one := big.NewRat(1, 1)
	m := randCornerMap(rng)
	row := randScalarRangeVec(rng)

	// Equal inputs: all four corners the same, and a single unit row.
	requireScalarRangeMatches(t, m, row, TileCorners(half, half, half, half))
	requireScalarRangeMatches(t, m, RatVec{one, new(big.Rat), new(big.Rat)}, TileCorners(half, one, half, one))

	// Parallel rows: a row and its multiple give proportional ranges.
	scaled := RatVec{ratMul(row[0], big.NewRat(7, 3)), ratMul(row[1], big.NewRat(7, 3)), ratMul(row[2], big.NewRat(7, 3))}
	requireScalarRangeMatches(t, m, scaled, TileCorners(new(big.Rat), one, new(big.Rat), one))

	// Unequal denominators: coefficients with unrelated denominators.
	odd := Map{Constant: RatVec{big.NewRat(1, 3), big.NewRat(-2, 5), big.NewRat(3, 7)},
		Q0: RatVec{big.NewRat(1, 11), big.NewRat(2, 13), big.NewRat(-3, 17)},
		QU: RatVec{big.NewRat(5, 19), big.NewRat(-7, 23), big.NewRat(1, 29)},
		QV: RatVec{big.NewRat(-1, 31), big.NewRat(3, 37), big.NewRat(11, 41)},
		G0: big.NewRat(43, 47), GU: big.NewRat(-1, 53), GV: big.NewRat(2, 59)}
	requireScalarRangeMatches(t, odd, RatVec{big.NewRat(1, 61), big.NewRat(-5, 67), big.NewRat(9, 71)},
		TileCorners(big.NewRat(-1, 73), big.NewRat(2, 79), big.NewRat(-3, 83), big.NewRat(4, 89)))

	// With initialised rationals, a zero denominator at one corner panics in both bodies with the same value.
	zeroG := Map{Constant: m.Constant, Q0: m.Q0, QU: m.QU, QV: m.QV, G0: new(big.Rat), GU: one, GV: new(big.Rat)}
	_, panicked := tryScalarRange(func() Interval {
		return ScalarRange(zeroG, row, TileCorners(new(big.Rat), one, half, one))
	})
	require.NotNil(t, panicked)
	requireScalarRangeMatches(t, zeroG, row, TileCorners(new(big.Rat), one, half, one))
}
