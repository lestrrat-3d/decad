package decad

import (
	"errors"
	"math"
	"math/big"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/sweeppath"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The sweep memos (contact_sweep_memo.go) must serve exactly what the reading
// they replace computes. These tests read every kind of prepared path through
// an open memo, repeatedly and interleaved, and hold each reading to the same
// reading computed afresh: the ideal pose by fraction, the staged points'
// deviation by pose and fraction, and the point spans by fraction span. Each
// served value is then written over, as a careless caller might, and the next
// reading must not see it.
//
// Legs shown to fail (each broken in turn, a test red, then restored):
//   - idealAt keyed by the fraction's numerator alone:
//     TestSweepPathMemoMatchesAfresh;
//   - pointDeviation keyed without the pose: TestSweepPathMemoMatchesAfresh;
//   - pointDeviation and cornerSpan keyed without the point set:
//     TestSweepPathMemoMatchesAfresh's swapped point sets;
//   - a hit that skips the poll: TestSweepPathMemoMatchesAfresh's poll counts;
//   - each table handing out its stored value instead of a copy:
//     TestSweepPathMemoMatchesAfresh;
//   - the sweep radius keyed without the From pose:
//     TestRotationalSweepRadiusMatchesPerCornerForm;
//   - a run's memos left open after it returns (close a no-op):
//     dynamics' TestSweepMemosChangeNoStep, the tumble subset's certificates.

// requireSameIdeal asserts two ideal poses hold the identical rationals.
func requireSameIdeal(t *testing.T, want, got sweepIdealPose, msg string, args ...any) {
	t.Helper()
	for i := range 3 {
		for j := range 3 {
			require.Zero(t, want.Rot.Entry(i, j).Lo.Cmp(got.Rot.Entry(i, j).Lo), append([]any{msg}, args...)...)
			require.Zero(t, want.Rot.Entry(i, j).Hi.Cmp(got.Rot.Entry(i, j).Hi), append([]any{msg}, args...)...)
		}
		require.Zero(t, want.Shift[i].Lo.Cmp(got.Shift[i].Lo), append([]any{msg}, args...)...)
		require.Zero(t, want.Shift[i].Hi.Cmp(got.Shift[i].Hi), append([]any{msg}, args...)...)
	}
}

// requireSameSpans asserts two span sets hold the identical rationals.
func requireSameSpans(t *testing.T, want, got cornerSpans, msg string, args ...any) {
	t.Helper()
	require.Equal(t, want.Len(), got.Len(), append([]any{msg}, args...)...)
	for index := range want.Len() {
		for axis := range 3 {
			w, g := want.Span(index, axis), got.Span(index, axis)
			require.Zero(t, w.Lo.Cmp(g.Lo), append([]any{msg}, args...)...)
			require.Zero(t, w.Hi.Cmp(g.Hi), append([]any{msg}, args...)...)
		}
	}
}

// countingPoll counts its calls and fails from call failAt on, when failAt
// is positive.
type countingPoll struct {
	calls, failAt int
}

var errPollStop = errors.New("poll stop")

func (p *countingPoll) poll() error {
	p.calls++
	if p.failAt > 0 && p.calls >= p.failAt {
		return errPollStop
	}
	return nil
}

func TestSweepPathMemoMatchesAfresh(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(131, 137))
	for name, path := range rotationFormPaths(t) {
		memoized := path
		memoized.memo = &sweepPathMemo{}
		// Copies sharing the memo whose point sets differ, as the rolling
		// column test's copy does.
		swapped := memoized
		swapped.startPoints = slices.Clone(path.startPoints)
		swapped.sourcePoints = slices.Clone(path.sourcePoints)
		for i := range swapped.startPoints {
			swapped.startPoints[i][0] = proofarith.DyAdd(swapped.startPoints[i][0], proofarith.MustDyOf(.5))
			swapped.sourcePoints[i][1] = proofarith.DyAdd(swapped.sourcePoints[i][1], proofarith.MustDyOf(.25))
		}
		var poses []r3.Transform
		for _, span := range rotationFormFractions() {
			pose, err := path.poseAt(span[1])
			require.NoError(t, err, name)
			poses = append(poses, pose)
		}
		poses = append(poses, path.path.From)
		for range 4 {
			moved, err := poses[0].Then(randomContactPose(t, rng))
			require.NoError(t, err, name)
			poses = append(poses, moved)
		}
		for round := range 3 {
			for k, span := range rotationFormFractions() {
				f := span[1]
				want, wantOK := path.idealAtAfresh(f)
				got, ok := memoized.idealAt(f)
				require.Equal(t, wantOK, ok, name)
				requireSameIdeal(t, want, got, "%s round %d ideal at %v", name, round, f)
				got.Rot.Lo[0][0].SetInt64(12345)
				got.Shift[1].Lo.SetInt64(-7)

				for _, p := range []rotationalSweepPath{memoized, swapped} {
					wantSpans := p.cornerSpanAfresh(span[0], span[1])
					gotSpans := p.cornerSpan(span[0], span[1])
					requireSameSpans(t, wantSpans, gotSpans, "%s round %d spans %v", name, round, span)
					gotSpans.Lo[0][0].SetInt64(99)
					gotSpans.Den[2].SetInt64(3)
				}

				// Each fraction reads its own pose, the start pose and two of
				// the random ones, in an order that changes with the round.
				for _, at := range []r3.Transform{poses[k], poses[len(rotationFormFractions())],
					poses[len(poses)-1-(k+round)%4], poses[len(poses)-1-(k+round+1)%4]} {
					for _, p := range []rotationalSweepPath{memoized, swapped} {
						var wantPoll, gotPoll countingPoll
						ideal, ok := p.idealAtAfresh(f)
						require.True(t, ok, name)
						wantPoints, wantBound, wantOK, wantErr := p.pointDeviationFrom(at, ideal, wantPoll.poll)
						gotPoints, gotBound, gotOK, gotErr := p.pointDeviation(at, f, gotPoll.poll)
						require.NoError(t, wantErr, name)
						require.NoError(t, gotErr, name)
						require.Equal(t, wantOK, gotOK, name)
						require.Equal(t, math.Float64bits(wantBound), math.Float64bits(gotBound), "%s round %d at %v", name, round, f)
						require.Equal(t, wantPoll.calls, gotPoll.calls, "%s: one poll per point, hit or miss", name)
						require.Len(t, gotPoints, len(wantPoints), name)
						for i := range wantPoints {
							requireDyV3Equal(t, wantPoints[i], gotPoints[i], "%s round %d point %d", name, round, i)
						}
						gotPoints[0] = proofarith.DyV3{}
					}
				}
			}
		}
		stats := memoized.memo.Stats
		require.Positive(t, stats.IdealHits, name)
		require.Positive(t, stats.DeviationHits, name)
		require.Positive(t, stats.SpanHits, name)
		require.Positive(t, stats.IdealMisses, name)

		// A poll that fails stops a served reading at the same point a
		// computed one stops.
		f := rotationFormFractions()[2][1]
		_, _, _, err := memoized.pointDeviation(poses[2], f, noSweepPoll)
		require.NoError(t, err, name)
		hits := memoized.memo.Stats.DeviationHits
		stop := countingPoll{failAt: 2}
		_, _, ok, err := memoized.pointDeviation(poses[2], f, stop.poll)
		require.ErrorIs(t, err, errPollStop, name)
		require.False(t, ok, name)
		require.Equal(t, 2, stop.calls, name)
		require.Equal(t, hits+1, memoized.memo.Stats.DeviationHits, "%s: premise: the failing read was served", name)

		// A closed memo serves nothing and stores nothing.
		memoized.memo.Close()
		got, ok := memoized.idealAt(f)
		require.True(t, ok, name)
		want, _ := path.idealAtAfresh(f)
		requireSameIdeal(t, want, got, "%s closed", name)
		require.Equal(t, sweepPathMemo{Closed: true}, *memoized.memo, name)
	}
}

func TestSweepPathMemoTablesStayBounded(t *testing.T) {
	t.Parallel()
	path := rotationFormPaths(t)["box drift"]
	path.memo = &sweepPathMemo{}
	for i := range sweepMemoCap + 3 {
		_, ok := path.idealAt(big.NewRat(int64(i), sweepMemoCap+3))
		require.True(t, ok)
	}
	require.Len(t, path.memo.Ideal, 3, "a full table empties and refills")
	require.Equal(t, sweepMemoCap+3, path.memo.Stats.IdealMisses)
}

// rotationalSweepRadiusPerCorner is rotationalSweepRadius before it chose the
// largest corner by its squared numerator: one big.Rat quotient per corner.
func rotationalSweepRadiusPerCorner(body *Body, from r3.Transform, center r3.Vec,
	axis motionbound.RatVec) (*big.Rat, bool) {
	corners, ok := inflatedBoundsCorners(body)
	if !ok {
		return nil, false
	}
	pivot := proofarith.DyVec(center)
	axisSquared := new(big.Rat)
	for k := range 3 {
		axisSquared.Add(axisSquared, new(big.Rat).Mul(axis[k], axis[k]))
	}
	if axisSquared.Sign() <= 0 {
		return nil, false
	}
	best := new(big.Rat)
	for _, corner := range corners {
		mapped := exactContactTransform(from, corner)
		delta := proofarith.DvSub(mapped, pivot)
		cross := [3]*big.Rat{
			new(big.Rat).Sub(new(big.Rat).Mul(delta[1].Rat(), axis[2]),
				new(big.Rat).Mul(delta[2].Rat(), axis[1])),
			new(big.Rat).Sub(new(big.Rat).Mul(delta[2].Rat(), axis[0]),
				new(big.Rat).Mul(delta[0].Rat(), axis[2])),
			new(big.Rat).Sub(new(big.Rat).Mul(delta[0].Rat(), axis[1]),
				new(big.Rat).Mul(delta[1].Rat(), axis[0])),
		}
		squared := new(big.Rat)
		for k := range 3 {
			squared.Add(squared, new(big.Rat).Mul(cross[k], cross[k]))
		}
		squared.Quo(squared, axisSquared)
		if squared.Cmp(best) > 0 {
			best = squared
		}
	}
	radius := proofbound.RatSqrtUp(best)
	if !finiteMeasurementValues(radius) {
		return nil, false
	}
	return proofarith.FloatRat(radius), true
}

// TestRotationalSweepRadiusMatchesPerCornerForm holds rotationalSweepRadius,
// read twice so the second read is the body memo's, to the per-corner form
// over pools of random poses, pivots and axes on a box, a disc and a
// chamfered block, and holds a non-dyadic axis's rational form to it too. A
// −0 From coordinate keys apart from a +0 one.
func TestRotationalSweepRadiusMatchesPerCornerForm(t *testing.T) {
	t.Parallel()
	doc := New()
	bodies := []*Body{internalBoxBody(t, doc, -5, -3, 7, 4, 9), internalDiscBody(t, doc, 4, 6),
		bandChamferedBlock(t, doc)}
	rng := rand.New(rand.NewPCG(139, 149))
	component := func() float64 {
		switch rng.IntN(4) {
		case 0:
			return 0
		case 1:
			return math.Ldexp(rng.Float64()*2-1, rng.IntN(60)-30)
		}
		return rng.Float64()*4 - 2
	}
	// Small pools of poses, pivots and axes, so that keys agreeing on all
	// but one field are read.
	var froms []r3.Transform
	var centers []r3.Vec
	var axes []motionbound.RatVec
	for range 3 {
		froms = append(froms, randomContactPose(t, rng))
		centers = append(centers, r3.Vec{X: component() * 50, Y: component() * 50, Z: component() * 50})
		var axis motionbound.RatVec
		for k := range 3 {
			axis[k] = proofarith.FloatRat(component())
		}
		axes = append(axes, axis)
	}
	compared := 0
	for range 160 {
		body := bodies[rng.IntN(len(bodies))]
		from, center, axis := froms[rng.IntN(3)], centers[rng.IntN(3)], axes[rng.IntN(3)]
		want, wantOK := rotationalSweepRadiusPerCorner(body, from, center, axis)
		for read := range 2 {
			got, ok := rotationalSweepRadius(body, from, center, axis)
			require.Equal(t, wantOK, ok, "read %d", read)
			if !ok {
				continue
			}
			require.Zero(t, want.Cmp(got), "read %d: %v vs %v", read, want, got)
			got.SetInt64(-1)
			compared++
		}
		third := new(big.Rat).SetFrac64(1, 3)
		nonDyadic := motionbound.RatVec{third, axis[1], axis[2]}
		want, wantOK = rotationalSweepRadiusPerCorner(body, from, center, nonDyadic)
		got, ok := rotationalSweepRadius(body, from, center, nonDyadic)
		require.Equal(t, wantOK, ok)
		if ok {
			require.Zero(t, want.Cmp(got))
		}
	}
	require.Greater(t, compared, 100)
	hits := 0
	for _, body := range bodies {
		body.sweepRadii.Mu.Lock()
		hits += body.sweepRadii.Hits
		body.sweepRadii.Mu.Unlock()
	}
	require.Greater(t, hits, 100, "the second reads are the memo's")

	plus, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	minus, err := r3.Translation(r3.Vec{X: 1, Y: math.Copysign(0, -1)})
	require.NoError(t, err)
	unit := proofarith.DyV3{proofarith.DyZero(), proofarith.DyZero(), proofarith.MustDyOf(1)}
	require.NotEqual(t, newSweepRadiusKey(plus, r3.Vec{}, unit), newSweepRadiusKey(minus, r3.Vec{}, unit))
}

func TestSweepRadiusMemoEvictsOldest(t *testing.T) {
	t.Parallel()
	keyAt := func(i int) sweepRadiusKey {
		pose, err := r3.Translation(r3.Vec{X: float64(i)})
		require.NoError(t, err)
		return newSweepRadiusKey(pose, r3.Vec{}, proofarith.DyV3{})
	}
	var memo sweepRadiusMemo
	for i := range sweepRadiusMemoCap + 3 {
		memo.Store(keyAt(i), float64(i), true)
	}
	require.Len(t, memo.Entries, sweepRadiusMemoCap)
	for i := range 3 {
		_, _, hit := memo.Load(keyAt(i))
		require.False(t, hit, "entry %d is among the oldest", i)
	}
	radius, ok, hit := memo.Load(keyAt(sweepRadiusMemoCap + 2))
	require.True(t, hit)
	require.True(t, ok)
	require.Equal(t, float64(sweepRadiusMemoCap+2), radius)
	memo.Store(keyAt(sweepRadiusMemoCap+2), 0, false)
	radius, _, _ = memo.Load(keyAt(sweepRadiusMemoCap + 2))
	require.Equal(t, float64(sweepRadiusMemoCap+2), radius, "a stored key keeps its first reading")
}

// TestSweepRunMemoChangesNoReport runs one rotating sweep, a spinning and
// falling box bracketing its impact on a floor, with its paths' memos open
// and again with them closed from the start, as a run with the memos
// switched off holds them. The run serves repeated ideal poses and
// deviations from the memo, and both reports are equal.
func TestSweepRunMemoChangesNoReport(t *testing.T) {
	t.Parallel()
	box := rotationFormPaths(t)["box drift"]
	low := math.Inf(1)
	for _, corner := range box.startBox.corner {
		z, _ := corner[2].Float64()
		low = min(low, z)
	}
	floor := internalOffsetBox(t, box.body.doc, -200, -200, 200, 200, low-.05-10,
		Distance{D: units.Millimeters(10), Dir: Along})
	still, err := sweeppath.Validate(PoseSegment{From: r3.Identity(), To: r3.Identity(),
		Duration: units.Seconds(1.0 / 256)})
	require.NoError(t, err)
	floorPath, ok := prepareRotationalSweepPath(floor, still)
	require.True(t, ok)
	req := SweepRequest{ContactRequest: ContactRequest{PointResolution: units.Millimeters(1e-3),
		NormalResolution: units.Degrees(1)}, MaxPoseEvaluations: 256}
	run := func(open bool) (*SweepReport, sweepMemoStats) {
		a, b := floorPath, box
		a.memo, b.memo = &sweepPathMemo{Closed: !open}, &sweepPathMemo{Closed: !open}
		sweep := &rotationalPairSweep{doc: floor.doc, a: a, b: b, req: req, resolution: big.NewRat(1, 1<<30),
			report: &SweepReport{}}
		report, err := sweep.execute(t.Context())
		require.NoError(t, err)
		stats := b.memo.Stats
		closeSweepMemos(&a, &b)
		return report, stats
	}
	on, stats := run(true)
	off, _ := run(false)
	require.Equal(t, SweepImpactBracket, on.Outcome, "premise: the sweep refines to a bracket")
	require.Positive(t, stats.IdealHits)
	require.Positive(t, stats.DeviationHits)
	require.Equal(t, off, on)
}
