package polynomial

import (
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// stageCoeff draws a coefficient from the mix of values the restriction code
// sees: exact floats, fractions with 2, 3 and 5 smooth denominators, zero,
// small integers, and results of the proof sum and product helpers.
func stageCoeff(rng *rand.Rand, depth int) *big.Rat {
	switch rng.IntN(12) {
	case 0, 1:
		return new(big.Rat)
	case 2, 3:
		return big.NewRat(rng.Int64N(41)-20, 1)
	case 4, 5, 6:
		exp := rng.IntN(121) - 60
		value := (rng.Float64()*2 - 1) * float64(int64(1)<<uint(rng.IntN(30)))
		for range abs(exp) {
			if exp < 0 {
				value /= 2
			} else {
				value *= 2
			}
		}
		if r := proofarith.FloatRat(value); r != nil {
			return r
		}
		return new(big.Rat)
	case 7, 8:
		den := new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(65)))
		den.Mul(den, new(big.Int).Exp(big.NewInt(3), big.NewInt(rng.Int64N(21)), nil))
		den.Mul(den, new(big.Int).Exp(big.NewInt(5), big.NewInt(rng.Int64N(21)), nil))
		den.Mul(den, big.NewInt(2*rng.Int64N(1<<19)+1))
		num := big.NewInt(rng.Int64N(1<<40) - 1<<39)
		return new(big.Rat).SetFrac(num, den)
	default:
		if depth > 2 {
			return big.NewRat(rng.Int64N(2001)-1000, rng.Int64N(999)+1)
		}
		a, b := stageCoeff(rng, depth+1), stageCoeff(rng, depth+1)
		if rng.IntN(2) == 0 {
			return proofarith.AddRat(new(big.Rat), a, b)
		}
		return proofarith.MulRat(new(big.Rat), a, b)
	}
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

func stageRow(rng *rand.Rand, length int) []*big.Rat {
	row := make([]*big.Rat, length)
	for i := range row {
		row[i] = stageCoeff(rng, 0)
	}
	return row
}

func cloneRow(row []*big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(row))
	for i, c := range row {
		out[i] = new(big.Rat).Set(c)
	}
	return out
}

func requireRatsSame(t *testing.T, want, got []*big.Rat) {
	t.Helper()
	require.Len(t, got, len(want))
	for i := range want {
		require.Zero(t, want[i].Num().Cmp(got[i].Num()), "numerator %d: want %s, got %s", i, want[i], got[i])
		require.Zero(t, want[i].Denom().Cmp(got[i].Denom()), "denominator %d: want %s, got %s", i, want[i], got[i])
	}
}

func TestBernsteinRestrictUnitIntervalIsCopy(t *testing.T) {
	rng := rand.New(rand.NewPCG(83, 89))
	los := []*big.Rat{new(big.Rat), big.NewRat(0, 5)}
	his := []*big.Rat{big.NewRat(1, 1), big.NewRat(7, 7)}
	for trial := range 2000 {
		row := stageRow(rng, 1+rng.IntN(9))
		before := cloneRow(row)
		lo, hi := los[rng.IntN(len(los))], his[rng.IntN(len(his))]
		got := BernsteinRestrict(row, lo, hi)
		// legacyRestrict is the general path with no fast path in front.
		requireRatsSame(t, legacyRestrict(row, lo, hi), got)
		requireRatsSame(t, before, row)
		for i := range got {
			require.NotSame(t, row[i], got[i], "trial %d coefficient %d aliases its input", trial, i)
		}
		for i := range got {
			got[i].SetInt64(12345)
		}
		requireRatsSame(t, before, row)
	}
}

func legacyRectRestrict(rows [3][]*big.Rat, sLo, sHi, tLo, tHi *big.Rat) [3][]*big.Rat {
	var restricted [3][]*big.Rat
	for row := range restricted {
		restricted[row] = make([]*big.Rat, len(rows[row]))
	}
	for column := range rows[0] {
		coeff := legacyRestrict([]*big.Rat{rows[0][column], rows[1][column], rows[2][column]}, tLo, tHi)
		for row := range restricted {
			restricted[row][column] = coeff[row]
		}
	}
	for row := range restricted {
		restricted[row] = legacyRestrict(restricted[row], sLo, sHi)
	}
	return restricted
}

// stageInterval draws lo < hi in [0, 1]. Each end is exactly 0 or 1 in a
// quarter of the draws, dyadic in 40 %, and otherwise not dyadic.
func stageInterval(rng *rand.Rand) (*big.Rat, *big.Rat) {
	for {
		end := func(zeroOrOne *big.Rat) *big.Rat {
			switch d := rng.IntN(20); {
			case d < 5:
				return zeroOrOne
			case d < 13:
				return big.NewRat(rng.Int64N(1<<10), 1<<10)
			default:
				q := rng.Int64N(997) + 3
				return big.NewRat(rng.Int64N(q+1), q)
			}
		}
		lo, hi := end(new(big.Rat)), end(big.NewRat(1, 1))
		if lo.Cmp(hi) < 0 && lo.Cmp(big.NewRat(1, 1)) < 0 {
			return lo, hi
		}
	}
}

func requireRowsSame(t *testing.T, want, got [3][]*big.Rat) {
	t.Helper()
	for row := range want {
		requireRatsSame(t, want[row], got[row])
	}
}

func TestBernsteinRectStagesMatchLegacy(t *testing.T) {
	rng := rand.New(rand.NewPCG(109, 113))
	for range 500 {
		length := 2 + rng.IntN(7)
		var rows, before [3][]*big.Rat
		for r := range rows {
			rows[r] = stageRow(rng, length)
			before[r] = cloneRow(rows[r])
		}
		tLo, tHi := stageInterval(rng)
		sLo, sHi := stageInterval(rng)
		requireRowsSame(t, legacyRectRestrict(rows, sLo, sHi, tLo, tHi),
			BernsteinRectRestrict(rows, sLo, sHi, tLo, tHi))
		timeRows := BernsteinRectTimeRestrict(rows, tLo, tHi)
		timeBefore := [3][]*big.Rat{cloneRow(timeRows[0]), cloneRow(timeRows[1]), cloneRow(timeRows[2])}
		for range 5 {
			lo, hi := stageInterval(rng)
			requireRowsSame(t, legacyRectRestrict(rows, lo, hi, tLo, tHi),
				BernsteinRectSourceRestrict(timeRows, lo, hi))
		}
		requireRowsSame(t, before, rows)
		requireRowsSame(t, timeBefore, timeRows)
	}
}

func TestBernsteinBinomialTablesMatchFresh(t *testing.T) {
	check := func() {
		for n := range 41 {
			fresh := bernsteinBinomialsFresh(n)
			got := bernsteinBinomials(n)
			require.Len(t, got, len(fresh))
			for i := range fresh {
				require.Zero(t, fresh[i].Cmp(got[i]), "C(%d,%d)", n, i)
			}
			require.Zero(t, bernsteinBinomialLCMFresh(n).Cmp(bernsteinBinomialLCM(n)), "lcm %d", n)
		}
	}
	check()
	rng := rand.New(rand.NewPCG(127, 131))
	for range 100 {
		a, b := stageRow(rng, 1+rng.IntN(9)), stageRow(rng, 1+rng.IntN(9))
		BernsteinMul(a, b)
		BernsteinSign(a, big.NewRat(rng.Int64N(7), 7))
		BernsteinIntRowOf(a).Mul(BernsteinIntRowOf(b))
	}
	check()
}
