package proofbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func requireMemoMatches(t *testing.T, m *proofbound.TurnTrigMemo, turn *big.Rat) {
	t.Helper()
	wantS, wantC := proofbound.TurnSinCosInterval(turn)
	gotS, gotC := m.SinCos(turn)
	requireSameInterval(t, wantS, gotS, "sin "+turn.String())
	requireSameInterval(t, wantC, gotC, "cos "+turn.String())
}

// stationTurns lists the turns coilshell.Build passes: turns*j/N for
// N = ceil(turns*256), j = 0..N.
func stationTurns(turns *big.Rat) []*big.Rat {
	nr := new(big.Rat).Mul(turns, big.NewRat(256, 1))
	n := new(big.Int).Quo(nr.Num(), nr.Denom())
	if !nr.IsInt() {
		n.Add(n, big.NewInt(1))
	}
	out := make([]*big.Rat, 0, int(n.Int64())+1)
	for j := int64(0); j <= n.Int64(); j++ {
		f := new(big.Rat).Mul(turns, big.NewRat(j, 1))
		out = append(out, f.Quo(f, new(big.Rat).SetInt(n)))
	}
	return out
}

func circleTurns() []*big.Rat {
	out := make([]*big.Rat, 0, 2049)
	for i := range 2049 {
		out = append(out, big.NewRat(int64(i), 2048))
	}
	return out
}

func TestTurnTrigMemoMatchesTurnSinCosInterval(t *testing.T) {
	t.Parallel()
	for _, turns := range []*big.Rat{
		big.NewRat(1, 1), big.NewRat(2, 1), big.NewRat(3, 1), big.NewRat(5, 1), big.NewRat(23, 10),
	} {
		m := proofbound.NewTurnTrigMemo()
		for _, turn := range stationTurns(turns) {
			requireMemoMatches(t, m, turn)
		}
	}

	m := proofbound.NewTurnTrigMemo()
	for _, turn := range circleTurns() {
		requireMemoMatches(t, m, turn)
	}

	// Adversarial turns: zero, exact octant boundaries, tiny, near 1/2 and
	// near an octant boundary, negative, and beyond one period.
	adversarial := []*big.Rat{
		big.NewRat(0, 1),
		big.NewRat(1, 1000000007),
		big.NewRat(-1, 1000000007),
		big.NewRat(1, 2),
		new(big.Rat).Sub(big.NewRat(1, 2), big.NewRat(1, 1000000007)),
		new(big.Rat).Add(big.NewRat(1, 2), big.NewRat(1, 1000000007)),
		big.NewRat(-1, 2),
		big.NewRat(-7, 8),
		big.NewRat(40, 3),
	}
	for k := int64(-16); k <= 16; k++ {
		adversarial = append(adversarial, big.NewRat(k, 8))
		adversarial = append(adversarial, new(big.Rat).Add(big.NewRat(k, 8), big.NewRat(1, 1<<40)))
		adversarial = append(adversarial, new(big.Rat).Sub(big.NewRat(k, 8), big.NewRat(1, 1<<40)))
	}
	rng := rand.New(rand.NewPCG(1, 2))
	for range 200 {
		p := rng.Int64N(10001) - 5000
		q := rng.Int64N(997) + 1
		adversarial = append(adversarial, big.NewRat(p, q))
	}
	m = proofbound.NewTurnTrigMemo()
	for range 2 {
		for _, turn := range adversarial {
			requireMemoMatches(t, m, turn)
		}
	}
}

func TestTurnTrigMemoAnswersAreFresh(t *testing.T) {
	t.Parallel()
	turn := big.NewRat(1, 12)
	want, wantC := proofbound.TurnSinCosInterval(turn)
	m := proofbound.NewTurnTrigMemo()

	s, c := m.SinCos(turn)
	for _, x := range []*big.Rat{s.Lo, s.Hi, c.Lo, c.Hi} {
		x.SetInt64(7)
	}
	s2, c2 := m.SinCos(turn)
	requireSameInterval(t, want, s2, "sin after mutation")
	requireSameInterval(t, wantC, c2, "cos after mutation")

	// The same first-octant entry serves a turn one period on.
	s3, c3 := m.SinCos(big.NewRat(13, 12))
	requireSameInterval(t, want, s3, "sin one period on")
	requireSameInterval(t, wantC, c3, "cos one period on")

	// Mutating the first read after the entry exists must not change either.
	s2.Lo.SetInt64(9)
	s4, _ := m.SinCos(turn)
	requireSameInterval(t, want, s4, "sin after second mutation")
}

func TestTurnTrigMemoEvaluatesEachResidualOnce(t *testing.T) {
	t.Parallel()
	m := proofbound.NewTurnTrigMemo()
	require.Zero(t, proofbound.TurnTrigMemoLen(m))
	for _, turn := range stationTurns(big.NewRat(5, 1)) {
		m.SinCos(turn)
	}
	require.Equal(t, 33, proofbound.TurnTrigMemoLen(m))

	m = proofbound.NewTurnTrigMemo()
	for _, turn := range circleTurns() {
		m.SinCos(turn)
	}
	require.Equal(t, 257, proofbound.TurnTrigMemoLen(m))
}

func TestTurnTrigMemoIsPerCall(t *testing.T) {
	t.Parallel()
	a, b := proofbound.NewTurnTrigMemo(), proofbound.NewTurnTrigMemo()
	a.SinCos(big.NewRat(1, 12))
	require.Equal(t, 1, proofbound.TurnTrigMemoLen(a))
	require.Zero(t, proofbound.TurnTrigMemoLen(b))
	b.SinCos(big.NewRat(1, 10))
	require.Equal(t, 1, proofbound.TurnTrigMemoLen(a))

	// The memoized answers still enclose the true values.
	s, c := a.SinCos(big.NewRat(1, 12))
	sf, _ := s.Lo.Float64()
	cf, _ := c.Lo.Float64()
	require.InDelta(t, math.Sin(math.Pi/6), sf, 1e-12)
	require.InDelta(t, math.Cos(math.Pi/6), cf, 1e-12)
}
