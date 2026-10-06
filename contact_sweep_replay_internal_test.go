package decad

import (
	"errors"
	"maps"
	"math"
	"math/big"
	"math/rand/v2"
	"slices"
	"sync"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// replayMemoTestSweeps sweeps four pairs whose replays take different
// routes: a knob sliding on an exact floor and the two turning together
// (rotational planar replay), a source box driven into another (affine
// impact bracket, refused past its left edge), and a disc sliding on a slab
// (source cylinder disk track).
func replayMemoTestSweeps(t *testing.T) map[string]*SweepReport {
	t.Helper()
	doc := New()
	floor := bandOctagonFloor(t, doc)
	knob := bandKnobBody(t, doc)
	left := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	right := internalBoxBody(t, doc, 20, 0, 30, 10, 10)
	slab := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
	disc := internalDiscBody(t, doc, 4, 6)
	req := SweepRequest{ContactRequest: ContactRequest{PointResolution: units.Millimeters(1e-3),
		NormalResolution: units.Degrees(1)}, TimeResolution: units.Seconds(math.Ldexp(1, -20)), MaxPoseEvaluations: 512}
	at := func(v r3.Vec) r3.Transform {
		pose, err := r3.Translation(v)
		require.NoError(t, err)
		return pose
	}
	still := PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(1)}
	rest := 8 + 1.5*knob.payload.(facetedPayload).meshBound
	drift := func(v r3.Vec, linear, angular float64, seconds float64) RigidDriftSegment {
		zero := units.MillimetersPerSecond(0)
		return RigidDriftSegment{From: at(v), LinearVelocity: QuantityVec{X: units.MillimetersPerSecond(linear),
			Y: zero, Z: zero}, AngularVelocity: QuantityVec{X: units.RadiansPerSecond(0),
			Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(angular)}, Duration: units.Seconds(seconds)}
	}
	spin := func(v r3.Vec) RigidDriftSegment { return drift(v, 0, 1, 1) }
	fine := req
	fine.TimeResolution, fine.MaxPoseEvaluations = units.Seconds(1e-9), 8
	sweep := func(a, b *Body, pathA, pathB PairPath) *SweepReport {
		t.Helper()
		use := req
		switch a {
		case left:
			use = fine
		case slab:
			use = fine
			use.StartPolicy = ContinueCertifiedTouch
		}
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, use)
		require.NoError(t, err)
		require.NotNil(t, report.replay, "outcome=%v cause=%v", report.Outcome, report.Cause)
		return report
	}
	sweeps := map[string]*SweepReport{
		"slide": sweep(floor, knob, still, PoseSegment{From: at(r3.Vec{Z: rest}),
			To: at(r3.Vec{X: 16, Z: rest}), Duration: units.Seconds(1)}),
		"turn":   sweep(floor, knob, spin(r3.Vec{}), spin(r3.Vec{Z: rest})),
		"impact": sweep(left, right, drift(r3.Vec{}, 100, 0, 0.2), drift(r3.Vec{}, 0, 0, 0.2)),
		"disc":   sweep(slab, disc, drift(r3.Vec{}, 0, 0, 1), drift(r3.Vec{Z: 10}, 8, 0, 1)),
	}
	for _, name := range []string{"slide", "turn"} {
		require.Equal(t, SweepClear, sweeps[name].Outcome, "premise: %s", name)
		require.NotNil(t, sweeps[name].replay.planar, "premise: %s replays its planar rotation", name)
	}
	require.Equal(t, SweepImpactBracket, sweeps["impact"].Outcome, "premise")
	require.Equal(t, SweepPersistentTouch, sweeps["disc"].Outcome, "premise")
	require.NotNil(t, sweeps["disc"].replay.cylinder, "premise: the disc replays its source cylinder")
	return sweeps
}

// replayMemoTestFractions returns small fractions, equal fractions in other
// terms (2/4 and 1/2 read one key), and random fractions with large terms,
// fewer distinct ones than replayPoseMemoCap so no entry is evicted.
func replayMemoTestFractions(rng *rand.Rand) []*big.Rat {
	var out []*big.Rat
	for _, n := range []int64{1, 2, 3, 4, 7} {
		for k := range n + 1 {
			out = append(out, big.NewRat(k, n))
		}
	}
	for range 6 {
		den := rng.Int64N(1<<40) + 1
		out = append(out, big.NewRat(rng.Int64N(den+1), den))
	}
	return out
}

func requireSameReplay(t *testing.T, wantA, wantB r3.Transform, wantErr error,
	gotA, gotB r3.Transform, gotErr error, msg string) {
	t.Helper()
	if wantErr != nil {
		require.Error(t, gotErr, msg)
		require.Equal(t, wantErr.Error(), gotErr.Error(), msg)
		require.Equal(t, errors.Is(wantErr, ErrUnsupported), errors.Is(gotErr, ErrUnsupported), msg)
		require.Equal(t, errors.Is(wantErr, ErrDegenerate), errors.Is(gotErr, ErrDegenerate), msg)
		return
	}
	require.NoError(t, gotErr, msg)
	require.Equal(t, poseBits(wantA), poseBits(gotA), msg)
	require.Equal(t, poseBits(wantB), poseBits(gotB), msg)
}

// TestReplayPoseMemoMatchesUncachedReplay replays each sweep at every
// fraction through the memo three times, the second read right after the
// first and the third after every other fraction, each from a fresh copy of
// the fraction. Every answer, poses and refusals alike, must equal the replay
// run without the memo bit for bit, and only the first read of each distinct
// fraction may miss.
func TestReplayPoseMemoMatchesUncachedReplay(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(3, 5))
	outcomes := map[bool]int{}
	sweeps := replayMemoTestSweeps(t)
	for _, name := range slices.Sorted(maps.Keys(sweeps)) {
		report := sweeps[name]
		fractions := replayMemoTestFractions(rng)
		distinct := map[string]struct{}{}
		check := func(f *big.Rat) {
			wantA, wantB, wantErr := report.replayPosesAtFraction(f)
			gotA, gotB, err := report.certifiedPosesAtFraction(new(big.Rat).Set(f))
			requireSameReplay(t, wantA, wantB, wantErr, gotA, gotB, err, name+" at "+f.RatString())
			outcomes[err == nil]++
			distinct[f.RatString()] = struct{}{}
		}
		for _, f := range fractions {
			check(f)
			check(f)
		}
		for _, f := range fractions {
			check(f)
		}
		memo := &report.replay.poses
		require.Less(t, len(distinct), replayPoseMemoCap, "premise: %s evicts nothing", name)
		require.Equal(t, uint64(len(distinct)), memo.misses, "%s misses once per distinct fraction", name)
		require.Equal(t, uint64(3*len(fractions)-len(distinct)), memo.hits, "%s serves every repeat", name)
	}
	require.Positive(t, outcomes[true], "premise: some fractions replay")
	require.Positive(t, outcomes[false], "premise: some fractions are refused")
}

// TestReplayPoseMemoIsSafeConcurrently replays one sweep from several
// goroutines at once, as concurrent Trace.Sample and Step calls may; run
// under -race it fails if the memo is read or written unguarded.
func TestReplayPoseMemoIsSafeConcurrently(t *testing.T) {
	t.Parallel()
	report := replayMemoTestSweeps(t)["impact"]
	fractions := replayMemoTestFractions(rand.New(rand.NewPCG(9, 1)))
	type answer struct {
		a, b r3.Transform
		err  error
	}
	want := make([]answer, len(fractions))
	for i, f := range fractions {
		want[i].a, want[i].b, want[i].err = report.replayPosesAtFraction(f)
	}
	var wg sync.WaitGroup
	got := make([][]answer, 4)
	for g := range got {
		got[g] = make([]answer, len(fractions))
		wg.Go(func() {
			for i, f := range fractions {
				got[g][i].a, got[g][i].b, got[g][i].err = report.certifiedPosesAtFraction(new(big.Rat).Set(f))
			}
		})
	}
	wg.Wait()
	for g := range got {
		for i, f := range fractions {
			requireSameReplay(t, want[i].a, want[i].b, want[i].err, got[g][i].a, got[g][i].b, got[g][i].err,
				f.RatString())
		}
	}
}

// TestReplayFractionKeyIsExact requires equal fractions in any terms to share
// a key and fractions that differ only in sign, numerator or denominator, or
// whose bytes concatenate alike, to differ.
func TestReplayFractionKeyIsExact(t *testing.T) {
	t.Parallel()
	require.Equal(t, replayFractionKey(big.NewRat(1, 2)), replayFractionKey(big.NewRat(4, 8)))
	keys := map[string]string{}
	for _, f := range []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 2), big.NewRat(-1, 2), big.NewRat(2, 1),
		big.NewRat(1, 3), big.NewRat(3, 2), big.NewRat(1, 258), big.NewRat(257, 2), big.NewRat(1, 1)} {
		key := replayFractionKey(f)
		other, ok := keys[key]
		require.False(t, ok, "%s and %s share a key", f.RatString(), other)
		keys[key] = f.RatString()
	}
}

// TestReplayPoseMemoEvictsOldest fills a memo past its bound: it keeps
// exactly replayPoseMemoCap answers, the oldest leaves first, and storing a
// key it holds keeps the first answer.
func TestReplayPoseMemoEvictsOldest(t *testing.T) {
	t.Parallel()
	keyAt := func(i int) string { return replayFractionKey(big.NewRat(int64(i), 1000)) }
	answerAt := func(i int) replayPoses {
		pose, err := r3.Translation(r3.Vec{X: float64(i)})
		require.NoError(t, err)
		return replayPoses{a: pose, b: pose}
	}
	var memo replayPoseMemo
	for i := range replayPoseMemoCap + 3 {
		memo.store(keyAt(i), answerAt(i))
	}
	require.Len(t, memo.entries, replayPoseMemoCap)
	for i := range 3 {
		_, ok := memo.load(keyAt(i))
		require.False(t, ok, "entry %d is among the oldest", i)
	}
	for _, i := range []int{3, replayPoseMemoCap, replayPoseMemoCap + 2} {
		value, ok := memo.load(keyAt(i))
		require.True(t, ok, "entry %d", i)
		require.Equal(t, poseBits(answerAt(i).a), poseBits(value.a))
	}
	memo.store(keyAt(5), answerAt(99))
	value, ok := memo.load(keyAt(5))
	require.True(t, ok)
	require.Equal(t, poseBits(answerAt(5).a), poseBits(value.a))
	require.Len(t, memo.entries, replayPoseMemoCap)
	memo.store(keyAt(0), answerAt(0))
	_, ok = memo.load(keyAt(3))
	require.False(t, ok, "the next oldest leaves for the next new key")
}
