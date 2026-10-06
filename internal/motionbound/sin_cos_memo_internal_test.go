package motionbound

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func requireSameInterval(t *testing.T, want, got proofbound.RatInterval, msg string, args ...any) {
	t.Helper()
	require.Zero(t, want.Lo.Cmp(got.Lo), append([]any{msg}, args...)...)
	require.Zero(t, want.Hi.Cmp(got.Hi), append([]any{msg}, args...)...)
}

// TestRadianSinCosMemoMatchesAfresh reads random exact angles through
// RadianSinCos three times each, the later reads interleaved with other
// angles, and holds every pair to radianSinCos computed afresh. Each served
// pair is written over before the next read, which must not see it. The
// angles are float rationals, as the sweeps' rate-times-time products are,
// and non-dyadic rationals of both signs.
//
// Legs shown to fail (each broken in turn, this test red, then restored): the
// key dropping the denominator; load handing out the stored endpoints.
func TestRadianSinCosMemoMatchesAfresh(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(151, 157))
	var angles []*big.Rat
	for range 64 {
		switch rng.IntN(3) {
		case 0:
			angles = append(angles, proofarith.FloatRat(math.Ldexp(rng.Float64()*2-1, rng.IntN(12)-8)))
		case 1:
			angles = append(angles, big.NewRat(rng.Int64N(2001)-1000, rng.Int64N(997)+1))
		default:
			// Same numerator, other denominators: a key that dropped either
			// half would collide.
			angles = append(angles, big.NewRat(7, 3), big.NewRat(7, 5), big.NewRat(-7, 3))
		}
	}
	sinCosMemo.mu.Lock()
	hitsBefore := sinCosMemo.hits
	sinCosMemo.mu.Unlock()
	for round := range 3 {
		for i, angle := range angles {
			if round > 0 && (i+round)%2 == 0 {
				continue
			}
			wantSin, wantCos := radianSinCos(angle)
			sin, cos := RadianSinCos(angle)
			requireSameInterval(t, wantSin, sin, "round %d sin %v", round, angle)
			requireSameInterval(t, wantCos, cos, "round %d cos %v", round, angle)
			sin.Lo.SetInt64(5)
			cos.Hi.Neg(cos.Hi)
		}
	}
	sinCosMemo.mu.Lock()
	hits := sinCosMemo.hits - hitsBefore
	sinCosMemo.mu.Unlock()
	require.Greater(t, hits, uint64(len(angles)/2), "the later rounds are the memo's")
}

// TestRadianSinCosMemoEvictsOldest fills a memo past its bound: it keeps
// exactly radianSinCosMemoCap entries, the oldest leaves first, and storing a
// key it holds keeps the first entry.
func TestRadianSinCosMemoEvictsOldest(t *testing.T) {
	t.Parallel()
	keyAt := func(i int) string { return string(AppendRatKey(nil, big.NewRat(int64(i)+1, 1))) }
	entryAt := func(i int) sinCosEntry {
		value := big.NewRat(int64(i), 1)
		return sinCosEntry{sin: proofbound.PointInterval(value), cos: proofbound.PointInterval(value)}
	}
	var memo radianSinCosMemo
	for i := range radianSinCosMemoCap + 3 {
		memo.store(keyAt(i), entryAt(i))
	}
	require.Len(t, memo.entries, radianSinCosMemoCap)
	for i := range 3 {
		_, ok := memo.load(keyAt(i))
		require.False(t, ok, "entry %d is among the oldest", i)
	}
	entry, ok := memo.load(keyAt(radianSinCosMemoCap + 2))
	require.True(t, ok)
	require.Zero(t, entry.sin.Lo.Cmp(big.NewRat(radianSinCosMemoCap+2, 1)))
	memo.store(keyAt(5), entryAt(0))
	entry, ok = memo.load(keyAt(5))
	require.True(t, ok)
	require.Zero(t, entry.sin.Lo.Cmp(big.NewRat(5, 1)), "a stored key keeps its first entry")
}

// TestAppendRatKeyIsExact requires equal keys exactly for equal rationals.
func TestAppendRatKeyIsExact(t *testing.T) {
	t.Parallel()
	key := func(r *big.Rat) string { return string(AppendRatKey(nil, r)) }
	require.Equal(t, key(big.NewRat(2, 6)), key(big.NewRat(1, 3)))
	require.NotEqual(t, key(big.NewRat(1, 3)), key(big.NewRat(-1, 3)))
	require.NotEqual(t, key(big.NewRat(1, 3)), key(big.NewRat(13, 1)))
	require.NotEqual(t, key(big.NewRat(0, 1)), key(big.NewRat(1, 1)))
	require.NotEqual(t, key(big.NewRat(1, 2))+key(big.NewRat(3, 4)), key(big.NewRat(1, 23))+key(big.NewRat(4, 1)))
}
