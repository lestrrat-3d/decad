package decad

import (
	"math"
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The sweep's §10.4 transfer legs (docs/multibody-dynamics-design.md), shown
// on the transfer itself. Each was deleted in turn and its test went red:
//   - the pose deviation on a band: TestPlanarBandTransferChargesDeviation
//     publishes the rounded pose's width unchanged;
//   - δ on the overlap transfer's margin: TestPlanarOverlapTransferChargesDisplacement
//     carries a held overlap shallower than δ to the ideal path;
//   - δ on the vertex-hull gap: TestPlanarHullGapChargesDisplacement reads the
//     held hull gap whole, and clears a span whose held gap lies inside δ;
//   - the replay's transfer charge: TestReplayTransferChargeIsTheBasisDifference
//     reads a zero charge on its rotating path.
//
// A sweep's own ContactPair report already charges δ, so a public fixture
// reaches the overlap transfer only with a held depth between δ and δ plus a
// pose rounding of about 1e-10 mm, and a hull gap inside δ only between two
// samples the endpoints already separate; these tests hand the transfer
// those cases directly.

const bandTestOffset = 1 << 20

func bandOctagonFloor(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	corners := [][2]float64{{-64, -32}, {-32, -64}, {32, -64}, {64, -32}, {64, 32}, {32, 64}, {-32, 64}, {-64, 32}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	s.Fix(points[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(8), Dir: Along})
	require.NoError(t, err)
	return body
}

func bandChamferedBlock(t *testing.T, doc *Document) *Body {
	t.Helper()
	box := internalBoxBody(t, doc, -6, -6, 6, 6, 12)
	block, err := box.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(box))), units.Millimeters(.1))
	require.NoError(t, err)
	return block
}

func bandKnobBody(t *testing.T, doc *Document) *Body {
	t.Helper()
	base := internalBoxBody(t, doc, -5, -5, 5, 5, 10)
	disc := internalDiscBody(t, doc, 2, 4)
	lift, err := r3.Translation(r3.Vec{Z: 8})
	require.NoError(t, err)
	disc, err = disc.Placed(t.Context(), lift)
	require.NoError(t, err)
	union, err := Union(t.Context(), base, disc)
	require.NoError(t, err)
	return union
}

// bandSpin turns about Z at 1 rad/s through (bandTestOffset, 0, 0) for a
// second, starting from a translation.
func bandSpin(t *testing.T, at r3.Vec) RigidDriftSegment {
	t.Helper()
	from, err := r3.Translation(at)
	require.NoError(t, err)
	zero := units.MillimetersPerSecond(0)
	return RigidDriftSegment{From: from, Center: r3.Vec{X: bandTestOffset},
		LinearVelocity:  QuantityVec{X: zero, Y: zero, Z: zero},
		AngularVelocity: QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(1)},
		Duration:        units.Seconds(1)}
}

// bandRun prepares the general planar sweep of a pair as SweepPair does.
func bandRun(t *testing.T, doc *Document, a, b *Body, pathA, pathB PairPath) *rotationalPairSweep {
	t.Helper()
	budget := newWorkBudget(t.Context())
	var paths [2]rotationalSweepPath
	for i, side := range []struct {
		body *Body
		path PairPath
	}{{a, pathA}, {b, pathB}} {
		path, err := validatePairPath(side.path)
		require.NoError(t, err)
		solid, delta, ok, err := planarSolidAtPose(t.Context(), budget, side.body, r3.Identity())
		require.NoError(t, err)
		require.True(t, ok)
		paths[i], ok = preparePlanarSweepPath(side.body, path, &solid, delta)
		require.True(t, ok)
	}
	req := SweepRequest{ContactRequest: ContactRequest{PointResolution: units.Millimeters(1e-3),
		NormalResolution: units.Degrees(1)}}
	return &rotationalPairSweep{doc: doc, a: paths[0], b: paths[1], req: req,
		resolution: big.NewRat(1, 1<<20), report: &SweepReport{}, planar: true}
}

func TestPlanarBandTransferChargesDeviation(t *testing.T) {
	// A floor and a chamfered block resting on it spin together far from the
	// origin: held, they touch at every rounded pose, and the rounded poses
	// deviate from the ideal path.
	doc := New()
	floor := internalOffsetBox(t, doc, -100, -100, 100, 100, -10, Distance{D: units.Millimeters(10), Dir: Along})
	block := bandChamferedBlock(t, doc)
	run := bandRun(t, doc, floor, block, bandSpin(t, r3.Vec{X: bandTestOffset}), bandSpin(t, r3.Vec{X: bandTestOffset}))
	half := big.NewRat(1, 2)
	poseA, err := run.a.poseAt(half)
	require.NoError(t, err)
	poseB, err := run.b.poseAt(half)
	require.NoError(t, err)
	contact, err := doc.ContactPair(t.Context(), floor, block, poseA, poseB, run.req.ContactRequest)
	require.NoError(t, err)
	require.Equal(t, ContactBand, contact.Relation)
	_, etaA, okA, err := run.a.pointDeviation(poseA, half, noSweepPoll)
	require.NoError(t, err)
	_, etaB, okB, err := run.b.pointDeviation(poseB, half, noSweepPoll)
	require.NoError(t, err)
	require.True(t, okA && okB)
	require.Positive(t, etaA+etaB, "the far spin rounds its poses")

	event, err := run.planarIdealEvent(t.Context(), half, sweepInstant(half, big.NewRat(1, 1)), poseA, poseB, contact)
	require.NoError(t, err)
	require.Equal(t, ContactBand, event.Relation)
	require.Nil(t, event.Manifold)
	want := new(big.Rat).Add(proofarith.FloatRat(contact.Gap.Bound.Base()),
		new(big.Rat).Add(proofarith.FloatRat(etaA), proofarith.FloatRat(etaB)))
	require.GreaterOrEqual(t, proofarith.FloatRat(event.Gap.Bound.Base()).Cmp(want), 0)
}

func TestPlanarOverlapTransferChargesDisplacement(t *testing.T) {
	doc := New()
	floor := bandOctagonFloor(t, doc)
	knob := bandKnobBody(t, doc)
	delta := knob.payload.(facetedPayload).meshBound
	require.Greater(t, delta, math.Ldexp(1, -14))
	half := big.NewRat(1, 2)
	transfer := func(depth float64) ContactRelation {
		t.Helper()
		run := bandRun(t, doc, floor, knob, bandSpin(t, r3.Vec{X: bandTestOffset}),
			bandSpin(t, r3.Vec{X: bandTestOffset, Z: 8 - depth}))
		poseA, err := run.a.poseAt(half)
		require.NoError(t, err)
		poseB, err := run.b.poseAt(half)
		require.NoError(t, err)
		_, eta, ok, err := run.b.pointDeviation(poseB, half, noSweepPoll)
		require.NoError(t, err)
		require.True(t, ok)
		require.Positive(t, eta)
		require.Less(t, eta, math.Ldexp(1, -20), "the pose rounding alone is far under the depth")
		// The transfer's own check decides; the report only names the case.
		event, err := run.planarIdealEvent(t.Context(), half, sweepInstant(half, big.NewRat(1, 1)), poseA, poseB,
			&ContactReport{Relation: ContactOverlapping})
		require.NoError(t, err)
		return event.Relation
	}
	// Held 2^-14 mm deep, well past the pose rounding but inside δ: the true
	// bodies may not overlap at all.
	require.Equal(t, ContactUndecided, transfer(math.Ldexp(1, -14)))
	require.Equal(t, ContactOverlapping, transfer(1))
}

func TestPlanarHullGapChargesDisplacement(t *testing.T) {
	doc := New()
	floor := bandOctagonFloor(t, doc)
	knob := bandKnobBody(t, doc)
	delta := proofarith.FloatRat(knob.payload.(facetedPayload).meshBound)
	gap := func(z float64) *big.Rat {
		t.Helper()
		still := PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(1)}
		at, err := r3.Translation(r3.Vec{Z: 8 + z})
		require.NoError(t, err)
		run := bandRun(t, doc, floor, knob, still, PoseSegment{From: at, To: at, Duration: units.Seconds(1)})
		return run.intervalAxisGap(new(big.Rat), big.NewRat(1, 1))
	}
	wide := math.Ldexp(1, -10)
	require.Zero(t, new(big.Rat).Sub(proofarith.FloatRat(wide), delta).Cmp(gap(wide)))
	require.Nil(t, gap(math.Ldexp(1, -14)))
}

// TestReplayTransferChargeIsTheBasisDifference pins replay's §10.4 transfer
// charge ‖R_r − R_i‖_F·δ (rotationalSweepPath.transferCharge) over PR 18's
// knob resting at a held gap of 1.5·δ on the octagon floor. Leg shown to
// fail: with the charge zeroed, the rotating fixture's charge reads zero.
// The translating fixture also records that the triangle-inequality form
// (s + 1)·δ refuses every sampled fraction the transfer charge replays.
func TestReplayTransferChargeIsTheBasisDifference(t *testing.T) {
	doc := New()
	floor := bandOctagonFloor(t, doc)
	knob := bandKnobBody(t, doc)
	delta := knob.payload.(facetedPayload).meshBound
	require.Greater(t, delta, math.Ldexp(1, -14))
	rest := r3.Vec{Z: 8 + 1.5*delta}
	req := SweepRequest{ContactRequest: ContactRequest{PointResolution: units.Millimeters(1e-3),
		NormalResolution: units.Degrees(1)}, TimeResolution: units.Seconds(math.Ldexp(1, -20)), MaxPoseEvaluations: 512}
	const samples = 8
	sweep := func(pathA, pathB PairPath) *SweepReport {
		t.Helper()
		report, err := doc.SweepPair(t.Context(), floor, knob, pathA, pathB, req)
		require.NoError(t, err)
		require.Equal(t, SweepClear, report.Outcome, "cause=%v", report.Cause)
		require.NotNil(t, report.replay)
		require.NotNil(t, report.replay.planar)
		require.Zero(t, report.replay.rotation[0].delta.Sign(), "the floor is exact")
		require.Zero(t, proofarith.FloatRat(delta).Cmp(report.replay.rotation[1].delta.Rat()))
		return report
	}
	// replayAt replays fraction k/samples through the public entry point and
	// returns each body's pose deviation bound and transfer charge there.
	replayAt := func(report *SweepReport, k int) (r3.Transform, [2]*big.Rat, [2]*big.Rat) {
		t.Helper()
		f := big.NewRat(int64(k), samples)
		_, _, err := report.CertifiedPosesAt(units.Seconds(float64(k) / samples))
		require.NoError(t, err, "fraction %d/%d", k, samples)
		var pose r3.Transform
		var bound, charge [2]*big.Rat
		for i, path := range report.replay.rotation {
			var ok bool
			pose, err = path.poseAt(f)
			require.NoError(t, err)
			bound[i], charge[i], ok = path.replayDeviation(pose, f)
			require.True(t, ok)
		}
		return pose, bound, charge
	}

	// Sliding 16 mm along the floor, the knob's rounded basis is its From
	// basis exactly, so the transfer charge is zero and the proven lower gap,
	// about δ/2, replays every fraction. Charging (s + 1)·δ instead, about
	// 2δ, refuses each.
	still := PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(1)}
	from, err := r3.Translation(rest)
	require.NoError(t, err)
	to, err := r3.Translation(r3.Vec{X: 16, Z: rest.Z})
	require.NoError(t, err)
	slide := sweep(still, PoseSegment{From: from, To: to, Duration: units.Seconds(1)})
	for k := range samples + 1 {
		pose, bound, charge := replayAt(slide, k)
		require.Zero(t, charge[0].Sign())
		require.Zero(t, charge[1].Sign(), "a translating path charges nothing at %d/%d", k, samples)
		lower := slide.replay.planar.lowerGap(big.NewRat(int64(k), samples))
		require.NotNil(t, lower)
		stretch := proofarith.DyAdd(planarPoseScale(pose), proofarith.DyInt(1))
		triangle := new(big.Rat).Add(new(big.Rat).Add(bound[0], bound[1]), proofarith.DyMul(slide.replay.rotation[1].delta, stretch).Rat())
		require.LessOrEqual(t, lower.Cmp(triangle), 0, "the (s + 1)·δ form refuses %d/%d", k, samples)
	}

	// Turning both bodies together about Z at 1 rad/s, the knob's rounded
	// basis differs from the ideal rotation's enclosure by a few ulps. The
	// charge is the Frobenius norm of that difference, read at each entry's
	// farther enclosure endpoint so it bounds every member, times δ.
	spin := func(at r3.Vec) RigidDriftSegment {
		t.Helper()
		start, err := r3.Translation(at)
		require.NoError(t, err)
		zero := units.MillimetersPerSecond(0)
		return RigidDriftSegment{From: start,
			LinearVelocity:  QuantityVec{X: zero, Y: zero, Z: zero},
			AngularVelocity: QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(1)},
			Duration:        units.Seconds(1)}
	}
	turn := sweep(spin(r3.Vec{}), spin(rest))
	deltaRat := proofarith.FloatRat(delta)
	ceiling := new(big.Rat).Mul(proofarith.FloatRat(math.Ldexp(1, -40)), deltaRat)
	for k := 1; k <= samples; k++ {
		f := big.NewRat(int64(k), samples)
		pose, _, charge := replayAt(turn, k)
		ideal, ok := turn.replay.rotation[1].idealAt(f)
		require.True(t, ok)
		sin, cos := ratSinCosTaylor(f)
		for _, entry := range []struct {
			got  ratInterval
			want *big.Rat
		}{{ideal.rot.entry(0, 0), cos}, {ideal.rot.entry(1, 0), sin}, {ideal.rot.entry(0, 1), new(big.Rat).Neg(sin)}, {ideal.rot.entry(1, 1), cos}} {
			require.True(t, entry.got.lo.Cmp(entry.want) <= 0 && entry.want.Cmp(entry.got.hi) <= 0,
				"the enclosure holds the true rotation at %d/%d", k, samples)
		}
		basis := pose.Basis()
		columns := [3][3]float64{{basis.EX.X, basis.EX.Y, basis.EX.Z}, {basis.EY.X, basis.EY.Y, basis.EY.Z},
			{basis.EZ.X, basis.EZ.Y, basis.EZ.Z}}
		squared := new(big.Rat)
		for i := range 3 {
			for j := range 3 {
				rounded := proofarith.FloatRat(columns[j][i])
				enclosure := ideal.rot.entry(i, j)
				far := new(big.Rat).Abs(new(big.Rat).Sub(rounded, enclosure.lo))
				if other := new(big.Rat).Abs(new(big.Rat).Sub(rounded, enclosure.hi)); other.Cmp(far) > 0 {
					far = other
				}
				squared.Add(squared, far.Mul(far, far))
			}
		}
		// The charge is the norm rounded up to a float, times δ: its square
		// is at least the exact squared norm and exceeds it by no more than
		// the few ulps of the directed square root.
		norm := new(big.Rat).Quo(charge[1], deltaRat)
		normSquared := new(big.Rat).Mul(norm, norm)
		require.Positive(t, norm.Sign(), "a rotating path charges its basis difference at %d/%d", k, samples)
		require.GreaterOrEqual(t, normSquared.Cmp(squared), 0)
		slack := new(big.Rat).Mul(squared, new(big.Rat).Add(big.NewRat(1, 1), proofarith.FloatRat(math.Ldexp(1, -48))))
		require.LessOrEqual(t, normSquared.Cmp(slack), 0)
		require.Negative(t, charge[1].Cmp(ceiling), "the charge is a few ulps of δ at %d/%d", k, samples)
		require.Zero(t, charge[0].Sign())
	}
}

// ratSinCosTaylor returns sin x and cos x for a rational x in [0, 1] to
// within 1/80!, far inside the 1e-57-wide enclosure the sweep produces, by their
// Taylor series summed exactly.
func ratSinCosTaylor(x *big.Rat) (*big.Rat, *big.Rat) {
	sin, cos := new(big.Rat), new(big.Rat)
	term := big.NewRat(1, 1)
	for n := range 80 {
		if n > 0 {
			term.Mul(term, x)
			term.Quo(term, big.NewRat(int64(n), 1))
		}
		signed := new(big.Rat).Set(term)
		if (n/2)%2 == 1 {
			signed.Neg(signed)
		}
		if n%2 == 0 {
			cos.Add(cos, signed)
			continue
		}
		sin.Add(sin, signed)
	}
	return sin, cos
}
