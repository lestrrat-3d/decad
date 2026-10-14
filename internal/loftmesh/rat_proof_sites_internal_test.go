package loftmesh

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// siteRats returns reduced rationals: zero, signs, repeats of one value,
// huge and tiny numerators and denominators, and power-of-two denominators.
// SetFrac reduces, so every value is in lowest terms, as every producer in
// the repository leaves it (see proof.AddRat).
func siteRats() []*big.Rat {
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

func TestLoftFanHalfMatchesRawRat(t *testing.T) {
	oracle := func(v, p meshbool.Xp2) int {
		dv := p.V.Cmp(v.V)
		if dv > 0 || dv == 0 && p.U.Cmp(v.U) > 0 {
			return 0
		}
		return 1
	}
	pool := siteRats()
	rng := rand.New(rand.NewPCG(7, 11))
	pick := func() *big.Rat { return pool[rng.IntN(len(pool))] }
	for range 60_000 {
		v := meshbool.NewXP2(pick(), pick())
		p := meshbool.NewXP2(pick(), pick())
		require.Equal(t, oracle(v, p), loftFanHalf(v, p))
	}
	// A shared V and a shared U exercise the tie branches, and p == v the
	// level-and-equal case.
	for _, a := range pool {
		for _, b := range pool {
			v := meshbool.NewXP2(a, b)
			require.Equal(t, oracle(v, v), loftFanHalf(v, v))
			same := meshbool.NewXP2(pick(), b)
			require.Equal(t, oracle(v, same), loftFanHalf(v, same))
			up := meshbool.NewXP2(a, pick())
			require.Equal(t, oracle(v, up), loftFanHalf(v, up))
		}
	}
}

func TestChamferGridIndexMatchesRawQuo(t *testing.T) {
	oracle := func(coord, step *big.Rat) (int64, bool) {
		if coord == nil || step == nil || step.Sign() <= 0 {
			return 0, false
		}
		q := new(big.Rat).Quo(coord, step)
		i := new(big.Int).Div(q.Num(), q.Denom())
		return i.Int64(), i.IsInt64()
	}
	pool := siteRats()
	for _, coord := range pool {
		for _, step := range pool {
			wantI, wantOK := oracle(coord, step)
			gotI, gotOK := chamferGridIndex(coord, step)
			require.Equal(t, wantOK, gotOK)
			require.Equal(t, wantI, gotI)
		}
	}
	gotI, gotOK := chamferGridIndex(nil, big.NewRat(1, 2))
	require.False(t, gotOK)
	require.Zero(t, gotI)
}

func randomVec(rng *rand.Rand, grid bool) r3.Vec {
	coord := func() float64 {
		if grid {
			return float64(rng.IntN(7) - 3)
		}
		return (rng.Float64() - 0.5) * 20
	}
	return r3.Vec{X: coord(), Y: coord(), Z: coord()}
}

func TestChamferPointEdgeWithinMatchesRawRat(t *testing.T) {
	oracle := func(p proof.Xpt, edge ChamferSplitEdge, reach2 *big.Rat) bool {
		d := proof.Xsub(edge.B, edge.A)
		den := proof.XdotRat(d, d)
		if den.Sign() == 0 {
			return false
		}
		v := proof.Xsub(p, edge.A)
		tt := new(big.Rat).Quo(proof.XdotRat(v, d), den)
		if tt.Sign() < 0 {
			tt.SetInt64(0)
		} else if tt.Cmp(big.NewRat(1, 1)) > 0 {
			tt.SetInt64(1)
		}
		nearest := proof.Xlerp(edge.A, edge.B, new(big.Int).Set(tt.Num()), new(big.Int).Set(tt.Denom()))
		gap := proof.Xsub(p, nearest)
		return proof.XdotRat(gap, gap).Cmp(reach2) < 0
	}
	rng := rand.New(rand.NewPCG(3, 5))
	reaches := []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 4), big.NewRat(1, 1), big.NewRat(2, 1), big.NewRat(9, 1),
		big.NewRat(1, 1000), big.NewRat(400, 1)}
	var inside, outside int
	for n := range 12_000 {
		grid := n%2 == 0
		edge := ChamferSplitEdge{A: proof.XptOf(randomVec(rng, grid)), B: proof.XptOf(randomVec(rng, grid))}
		p := proof.XptOf(randomVec(rng, grid))
		reach2 := reaches[rng.IntN(len(reaches))]
		want := oracle(p, edge, reach2)
		require.Equal(t, want, chamferPointEdgeWithin(p, edge, reach2))
		if want {
			inside++
		} else {
			outside++
		}
	}
	require.Positive(t, inside)
	require.Positive(t, outside)
}

func TestWallTriangleAreaMatchesRawQuo(t *testing.T) {
	oracle := func(u, v proof.Xpt) (float64, float64) {
		w := proof.Xcross(u, v)
		q := proof.XdotRat(w, w)
		q.Quo(q, big.NewRat(4, 1))
		return proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q)
	}
	rng := rand.New(rand.NewPCG(13, 17))
	for n := range 6_000 {
		grid := n%2 == 0
		origin := proof.XptOf(randomVec(rng, grid))
		u := proof.Xsub(proof.XptOf(randomVec(rng, grid)), origin)
		v := proof.Xsub(proof.XptOf(randomVec(rng, grid)), origin)
		wantLo, wantHi := oracle(u, v)
		gotLo, gotHi := WallTriangleArea(u, v)
		require.Equal(t, wantLo, gotLo)
		require.Equal(t, wantHi, gotHi)
	}
}

func TestCapPolygonAreaRatMatchesRawRat(t *testing.T) {
	oracle := func(pts []sectionrecord.Point2, loopIdx [][]int) *big.Rat {
		toRat := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
		sum := new(big.Rat)
		for _, idx := range loopIdx {
			n := len(idx)
			for j := range n {
				p, q := pts[idx[j]], pts[idx[(j+1)%n]]
				term := new(big.Rat).Mul(toRat(p.U), toRat(q.V))
				term.Sub(term, new(big.Rat).Mul(toRat(q.U), toRat(p.V)))
				sum.Add(sum, term)
			}
		}
		return sum.Quo(sum, big.NewRat(2, 1))
	}
	rng := rand.New(rand.NewPCG(19, 23))
	for n := range 3_000 {
		count := 3 + rng.IntN(6)
		pts := make([]sectionrecord.Point2, count)
		loop := make([]int, count)
		for i := range pts {
			if n%2 == 0 {
				pts[i] = sectionrecord.Point2{U: float64(rng.IntN(9) - 4), V: float64(rng.IntN(9) - 4)}
			} else {
				pts[i] = sectionrecord.Point2{U: (rng.Float64() - 0.5) * 8, V: (rng.Float64() - 0.5) * 1e-3}
			}
			loop[i] = i
		}
		loops := [][]int{loop}
		if count >= 6 {
			loops = [][]int{loop[:3], loop[3:]}
		}
		want := oracle(pts, loops)
		got := CapPolygonAreaRat(pts, loops)
		require.Zero(t, want.Cmp(got))
		require.Equal(t, want.String(), got.String())
	}
}
