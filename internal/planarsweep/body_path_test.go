package planarsweep_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/planarsweep"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sweepmemo"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// boxCorners lists an axis box's corners in the source-box order: corner i
// takes hi on axis k when bit k of i is set.
func boxCorners(lo, hi [3]float64) []proofarith.DyV3 {
	corners := make([]proofarith.DyV3, 8)
	for i := range corners {
		for k := range 3 {
			value := lo[k]
			if i&(1<<k) != 0 {
				value = hi[k]
			}
			corners[i][k] = proofarith.MustDyOf(value)
		}
	}
	return corners
}

// translatingBoxPath prepares a box translating by (0.1, 0.2, 0.3) mm over
// one second. A translating path never reads its body's radius, so it needs
// no topology node.
func translatingBoxPath(t *testing.T, corners []proofarith.DyV3) planarsweep.BodyPath {
	t.Helper()
	to, err := r3.Translation(r3.NewVec(0.1, 0.2, 0.3))
	require.NoError(t, err)
	path, err := sweeppath.Validate(sweeppath.PoseSegment{From: r3.Identity(), To: to, Duration: units.Seconds(1)})
	require.NoError(t, err)
	prepared, ok := planarsweep.PrepareBodyPath(nil, path)
	require.True(t, ok)
	prepared.StartPoints, prepared.SourcePoints = corners, corners
	return prepared
}

func requireSameSpans(t *testing.T, want, got sweepmemo.CornerSpans) {
	t.Helper()
	require.Equal(t, want.Len(), got.Len())
	for k := range 3 {
		require.Zero(t, want.Den[k].Cmp(got.Den[k]), "den axis %d", k)
	}
	for i := range want.Lo {
		for k := range 3 {
			require.Zero(t, want.Lo[i][k].Cmp(got.Lo[i][k]), "lo point %d axis %d", i, k)
			require.Zero(t, want.Hi[i][k].Cmp(got.Hi[i][k]), "hi point %d axis %d", i, k)
		}
	}
}

// spansEqual reports whether two span sets enclose the same exact intervals.
func spansEqual(a, b sweepmemo.CornerSpans) bool {
	if a.Len() != b.Len() {
		return false
	}
	for k := range 3 {
		if a.Den[k].Cmp(b.Den[k]) != 0 {
			return false
		}
	}
	for i := range a.Lo {
		for k := range 3 {
			if a.Lo[i][k].Cmp(b.Lo[i][k]) != 0 || a.Hi[i][k].Cmp(b.Hi[i][k]) != 0 {
				return false
			}
		}
	}
	return true
}

// pointsEqual reports whether two staged point sets are exactly equal.
func pointsEqual(a, b []proofarith.DyV3) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if !proofarith.DvEqual(a[i], b[i]) {
			return false
		}
	}
	return true
}

func TestBodyPathMemo(t *testing.T) {
	corners := boxCorners([3]float64{0, 0, 0}, [3]float64{1, 2, 3})
	fresh := translatingBoxPath(t, corners)
	f := big.NewRat(1, 3)
	pose, err := fresh.Path.RoundedPoseAt(f)
	require.NoError(t, err)

	wantPoints, wantBound, wantOK, err := fresh.PointDeviation(pose, f, planarsweep.NoPoll)
	require.NoError(t, err)
	require.True(t, wantOK)
	wantSpans := fresh.CornerSpan(new(big.Rat), f)
	wantDeviation, wantCharge, ok := fresh.ReplayDeviation(pose, f)
	require.True(t, ok)

	memoized := fresh
	planarsweep.AttachMemos(&memoized)
	require.True(t, memoized.Memo.Open())
	// The first reading fills the memo and the second is served from it.
	// Each charges the poll once per source point and equals the fresh one.
	for reading := range 2 {
		polls := 0
		points, bound, ok, err := memoized.PointDeviation(pose, f, func() error {
			polls++
			return nil
		})
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, len(corners), polls, "reading %d", reading)
		require.Equal(t, wantBound, bound, "reading %d", reading)
		require.Len(t, points, len(wantPoints))
		for i := range points {
			require.True(t, proofarith.DvEqual(wantPoints[i], points[i]), "reading %d point %d", reading, i)
		}
		requireSameSpans(t, wantSpans, memoized.CornerSpan(new(big.Rat), f))
		deviation, charge, ok := memoized.ReplayDeviation(pose, f)
		require.True(t, ok)
		require.Zero(t, wantDeviation.Cmp(deviation), "reading %d", reading)
		require.Zero(t, wantCharge.Cmp(charge), "reading %d", reading)
	}

	// A copy taken during the run shares the memo, so closing the run's
	// paths leaves the copy computing afresh.
	held := memoized
	planarsweep.CloseMemos(&memoized)
	require.False(t, held.Memo.Open())
	_, bound, ok, err := held.PointDeviation(pose, f, planarsweep.NoPoll)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, wantBound, bound)
	requireSameSpans(t, wantSpans, held.CornerSpan(new(big.Rat), f))
}

func TestBodyPathRoundedBox(t *testing.T) {
	t.Run("start pose reads the source corners", func(t *testing.T) {
		corners := boxCorners([3]float64{-1, 0.5, 2}, [3]float64{1, 2, 3})
		path := translatingBoxPath(t, corners)
		box, _, ok := path.RoundedBox(path.Path.From, new(big.Rat))
		require.True(t, ok)
		for i := range corners {
			require.True(t, proofarith.DvEqual(corners[i], box.Corner[i]), "corner %d", i)
		}
		for axis, far := range [3]int{1, 2, 4} {
			require.True(t, proofarith.DvEqual(proofarith.DvSub(corners[far], corners[0]), box.Edge[axis]),
				"edge %d", axis)
		}
	})
	t.Run("a zero edge refuses", func(t *testing.T) {
		corners := boxCorners([3]float64{0, 0, 0}, [3]float64{1, 0, 3})
		path := translatingBoxPath(t, corners)
		_, _, ok := path.RoundedBox(path.Path.From, new(big.Rat))
		require.False(t, ok)
	})
}

// TestBodyPathMemoKeys reads two different inputs through one memo, each
// twice. Every reading must equal the fresh one for its own input, so a memo
// key that dropped part of its input would serve the first input's reading
// for the second.
func TestBodyPathMemoKeys(t *testing.T) {
	corners := boxCorners([3]float64{0, 0, 0}, [3]float64{1, 2, 3})
	fresh := translatingBoxPath(t, corners)
	near, far := big.NewRat(1, 3), big.NewRat(2, 3)
	poseNear, err := fresh.Path.RoundedPoseAt(near)
	require.NoError(t, err)
	poseFar, err := fresh.Path.RoundedPoseAt(far)
	require.NoError(t, err)

	wantSpanNear := fresh.CornerSpan(new(big.Rat), near)
	wantSpanFar := fresh.CornerSpan(new(big.Rat), far)
	require.False(t, spansEqual(wantSpanNear, wantSpanFar), "the two spans must differ for the test to bite")
	wantNear, _, ok, err := fresh.PointDeviation(poseNear, near, planarsweep.NoPoll)
	require.NoError(t, err)
	require.True(t, ok)
	wantFar, _, ok, err := fresh.PointDeviation(poseFar, far, planarsweep.NoPoll)
	require.NoError(t, err)
	require.True(t, ok)
	require.False(t, pointsEqual(wantNear, wantFar), "the two readings must differ for the test to bite")

	memoized := fresh
	planarsweep.AttachMemos(&memoized)
	defer planarsweep.CloseMemos(&memoized)
	for reading := range 2 {
		require.True(t, spansEqual(wantSpanNear, memoized.CornerSpan(new(big.Rat), near)), "near span, reading %d", reading)
		require.True(t, spansEqual(wantSpanFar, memoized.CornerSpan(new(big.Rat), far)), "far span, reading %d", reading)
		points, _, ok, err := memoized.PointDeviation(poseNear, near, planarsweep.NoPoll)
		require.NoError(t, err)
		require.True(t, ok)
		require.True(t, pointsEqual(wantNear, points), "near deviation, reading %d", reading)
		points, _, ok, err = memoized.PointDeviation(poseFar, far, planarsweep.NoPoll)
		require.NoError(t, err)
		require.True(t, ok)
		require.True(t, pointsEqual(wantFar, points), "far deviation, reading %d", reading)
	}
}
