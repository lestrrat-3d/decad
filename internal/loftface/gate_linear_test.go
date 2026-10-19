package loftface

import (
	"fmt"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/stretchr/testify/require"
)

// legacyGateGrid is the node-by-node grid that gateGrid replaced: every node
// forms q = Numerator(u, v) and both derivative numerators from it. The
// linear-form gateGrid must equal it on every node.
func legacyGateGrid(m Map, rows [3]RatVec, us, vs []*big.Rat) ([][]gateNode, error) {
	var rowQu, rowQv, rowConstant [3]*big.Rat
	for r, row := range rows {
		rowQu[r], rowQv[r], rowConstant[r] = VDot(row, m.QU), VDot(row, m.QV), VDot(row, m.Constant)
	}
	grid := make([][]gateNode, len(us))
	for i, u := range us {
		grid[i] = make([]gateNode, len(vs))
		for j, v := range vs {
			g := m.Denominator(u, v)
			if g.Sign() <= 0 {
				return nil, fmt.Errorf("%w: loft cap ray denominator is not positive", decaderr.ErrUnsupported)
			}
			q := m.Numerator(u, v)
			inverseG := ratDiv(ratOne(), g)
			node := gateNode{g: g}
			for r, row := range rows {
				rowQ := VDot(row, q)
				node.deriv[r][0] = ratSub(ratMul(g, rowQu[r]), ratMul(m.GU, rowQ))
				node.deriv[r][1] = ratSub(ratMul(g, rowQv[r]), ratMul(m.GV, rowQ))
				node.scalar[r] = ratAdd(rowConstant[r], ratMul(rowQ, inverseG))
			}
			grid[i][j] = node
		}
	}
	return grid, nil
}

// gateCoefficient draws a coefficient: a dyadic float in [-scale, scale], a
// fraction with denominator 3*5*7, or zero (10 %).
func gateCoefficient(rng *rand.Rand, scale float64) *big.Rat {
	switch k := rng.IntN(10); {
	case k == 0:
		return new(big.Rat)
	case k < 5:
		return big.NewRat(int64(rng.IntN(2*105*4+1)-105*4), 105)
	default:
		return ratF((rng.Float64()*2 - 1) * scale)
	}
}

func gateCoefficientVec(rng *rand.Rand, scale float64) RatVec {
	return RatVec{gateCoefficient(rng, scale), gateCoefficient(rng, scale), gateCoefficient(rng, scale)}
}

// wideGateMap draws a map whose every field is a gateCoefficient and whose
// G0 is large against GU and GV, so the denominator usually stays positive.
func wideGateMap(rng *rand.Rand) Map {
	return Map{Constant: gateCoefficientVec(rng, 4), Q0: gateCoefficientVec(rng, 4),
		QU: gateCoefficientVec(rng, 4), QV: gateCoefficientVec(rng, 4),
		G0: ratF(4 + rng.Float64()*4), GU: ratF(rng.Float64()*2 - 1), GV: ratF(rng.Float64()*2 - 1), Stretch: 1}
}

// scaledGateMap multiplies every coefficient of a wide map by 2^e (huge or
// tiny magnitudes) and, for the denominator, by 2^d.
func scaledGateMap(rng *rand.Rand, e, d int) Map {
	m := wideGateMap(rng)
	scale := func(x *big.Rat, k int) *big.Rat {
		f := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(max(k, 0))))
		if k < 0 {
			f = new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), uint(-k)))
		}
		return ratMul(x, f)
	}
	vec := func(v RatVec) RatVec { return RatVec{scale(v[0], e), scale(v[1], e), scale(v[2], e)} }
	m.Constant, m.Q0, m.QU, m.QV = vec(m.Constant), vec(m.Q0), vec(m.QU), vec(m.QV)
	m.G0, m.GU, m.GV = scale(m.G0, d), scale(m.GU, d), scale(m.GV, d)
	return m
}

// gateAxisGrid builds n+1 coordinates over [lo, hi] as BandGate does.
func gateAxisGrid(lo, hi *big.Rat, n int) []*big.Rat {
	width := ratDiv(ratSub(hi, lo), big.NewRat(int64(n), 1))
	out := make([]*big.Rat, n+1)
	for i := range out {
		out[i] = ratAdd(lo, ratMul(big.NewRat(int64(i), 1), width))
	}
	return out
}

func requireGateRatEqual(t *testing.T, want, got *big.Rat, what string) {
	t.Helper()
	require.NotNil(t, got, what)
	require.Zero(t, want.Num().Cmp(got.Num()), "%s numerator", what)
	require.Zero(t, want.Denom().Cmp(got.Denom()), "%s denominator", what)
}

func requireGateGridsEqual(t *testing.T, m Map, rows [3]RatVec, us, vs []*big.Rat) {
	t.Helper()
	want, wantErr := legacyGateGrid(m, rows, us, vs)
	got, gotErr := gateGrid(m, rows, us, vs)
	if wantErr != nil {
		require.Error(t, gotErr)
		require.Equal(t, wantErr.Error(), gotErr.Error())
		return
	}
	require.NoError(t, gotErr)
	require.Len(t, got, len(want))
	for i := range want {
		require.Len(t, got[i], len(want[i]))
		for j := range want[i] {
			at := fmt.Sprintf("node (%d,%d)", i, j)
			requireGateRatEqual(t, want[i][j].g, got[i][j].g, at+" g")
			for r := range 3 {
				for a := range 2 {
					requireGateRatEqual(t, want[i][j].deriv[r][a], got[i][j].deriv[r][a],
						fmt.Sprintf("%s deriv[%d][%d]", at, r, a))
				}
				requireGateRatEqual(t, want[i][j].scalar[r], got[i][j].scalar[r],
					fmt.Sprintf("%s scalar[%d]", at, r))
			}
		}
	}
}

func gateTestBox(rng *rand.Rand) (*big.Rat, *big.Rat, *big.Rat, *big.Rat) {
	coord := func() *big.Rat {
		if rng.IntN(2) == 0 {
			return ratF(float64(rng.IntN(129)-64) / 32)
		}
		return big.NewRat(int64(rng.IntN(2*7*5*3*4+1)-7*5*3*4), 7*5*3*4*2)
	}
	for {
		u0, u1, v0, v1 := coord(), coord(), coord(), coord()
		if u0.Cmp(u1) < 0 && v0.Cmp(v1) < 0 {
			return u0, u1, v0, v1
		}
	}
}

func TestGateGridLinearMatchesLegacy(t *testing.T) {
	rng := rand.New(rand.NewPCG(139, 149))
	refused := 0
	for trial := range 10 {
		var m Map
		var rows [3]RatVec
		if trial%2 == 0 {
			m = randomGateMap(rng)
			rows = [3]RatVec{gateCoefficientVec(rng, 4), gateCoefficientVec(rng, 4), gateCoefficientVec(rng, 4)}
		} else {
			m = wideGateMap(rng)
			rows = [3]RatVec{gateCoefficientVec(rng, 4), gateCoefficientVec(rng, 4), gateCoefficientVec(rng, 4)}
		}
		if trial >= 8 {
			// Push G0 down so the denominator is not positive at an interior node.
			m.G0 = ratF(-rng.Float64() * 0.01)
			m.GU, m.GV = ratF(1+rng.Float64()), ratF(-1-rng.Float64())
		}
		u0, u1, v0, v1 := gateTestBox(rng)
		us, vs := gateAxisGrid(u0, u1, 16), gateAxisGrid(v0, v1, 16)
		if _, err := legacyGateGrid(m, rows, us, vs); err != nil {
			refused++
		}
		requireGateGridsEqual(t, m, rows, us, vs)
	}
	require.Positive(t, refused, "some trial must hit the denominator refusal")
	require.Less(t, refused, 10, "some trial must build a grid")
}

func TestGateGridLinearMatchesLegacyAdversarial(t *testing.T) {
	rng := rand.New(rand.NewPCG(151, 157))
	zero := RatVec{new(big.Rat), new(big.Rat), new(big.Rat)}
	for trial := range 10 {
		e, d := []int{-120, -40, 0, 40, 120}[trial%5], []int{-60, 0, 60}[(trial/5)%3]
		m := scaledGateMap(rng, e, d)
		first := gateCoefficientVec(rng, 4)
		// Rows: zero, parallel, nearly parallel (differ in the last bit) and generic.
		near := RatVec{ratAdd(first[0], ratF(1e-300)), first[1], first[2]}
		rows := [3]RatVec{first, zero, near}
		switch trial % 4 {
		case 1:
			rows = [3]RatVec{first, first, first}
		case 2:
			rows = [3]RatVec{zero, zero, zero}
		case 3:
			rows = [3]RatVec{gateCoefficientVec(rng, 1e100), gateCoefficientVec(rng, 1e-100), near}
		}
		if trial%6 == 0 {
			// Degenerate map: no u or v dependence, then a purely linear numerator.
			m.GU, m.GV = new(big.Rat), new(big.Rat)
		}
		if trial%6 == 1 {
			m.QU, m.QV = zero, zero
		}
		u0, u1, v0, v1 := gateTestBox(rng)
		if trial%3 == 0 {
			// A tiny box: near-degenerate band.
			u1 = ratAdd(u0, ratF(1e-200))
			v1 = ratAdd(v0, ratF(1e-200))
		}
		// Box ends and grid sizes: the callers use 16 tiles; check others too.
		for _, n := range []int{1, 2, 5} {
			requireGateGridsEqual(t, m, rows, gateAxisGrid(u0, u1, n), gateAxisGrid(v0, v1, n))
		}
		// Unequal sizes, including empty axes.
		requireGateGridsEqual(t, m, rows, gateAxisGrid(u0, u1, 5), gateAxisGrid(v0, v1, 16)[:0])
		requireGateGridsEqual(t, m, rows, gateAxisGrid(u0, u1, 3), gateAxisGrid(v0, v1, 9))
	}
}

func TestBandGateMatchesReferenceWide(t *testing.T) {
	rng := rand.New(rand.NewPCG(163, 167))
	matched, refused := 0, 0
	for band := range 1 {
		u0, u1, v0, v1 := gateTestBox(rng)
		box := [4]*big.Rat{u0, u1, v0, v1}
		first, second := wideGateMap(rng), wideGateMap(rng)
		if band%2 == 0 {
			// A nearby second map: a small shift and perturbation, so the
			// injectivity bound can succeed as well as refuse.
			second = first
			second.Constant = RatVec{ratAdd(first.Constant[0], ratF(1)), first.Constant[1], first.Constant[2]}
			second.QU = RatVec{first.QU[0], ratAdd(first.QU[1], ratF(0.01)), first.QU[2]}
		}
		if PositiveDenominator(first, box) != nil || PositiveDenominator(second, box) != nil {
			continue
		}
		wantRho, wantJac, wantSign, wantErr := bandGateRef(t.Context(), first, second, box)
		gotRho, gotJac, gotSign, gotErr := BandGate(t.Context(), first, second, box)
		require.Equal(t, wantRho, gotRho)
		require.Equal(t, wantJac, gotJac)
		require.Equal(t, wantSign, gotSign)
		if wantErr == nil {
			require.NoError(t, gotErr)
		} else {
			require.EqualError(t, gotErr, wantErr.Error())
			refused++
		}
		matched++
	}
	require.Positive(t, matched)
	require.Positive(t, matched-refused, "some band must pass the gate")
	t.Logf("matched %d bands, %d refused", matched, refused)
}
