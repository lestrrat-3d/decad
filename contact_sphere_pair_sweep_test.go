package decad_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSweepPairRotatingSourceSpheresDepart(t *testing.T) {
	doc := decad.New()
	a, b := ballBody(t, doc, 5), ballBody(t, doc, 5)
	before := doc.Bodies()
	fromA := contactPose(t, r3.Vec{X: -5})
	fromB := contactPose(t, r3.Vec{X: 5})
	contact, err := doc.ContactPair(t.Context(), a, b, fromA, fromB, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 1)

	// These are the sliding-Coulomb response velocities for 1 kg spheres with
	// 10 kg·mm² inertia, e=0.5, μ=0.02, and initial velocities (50,20),(-50,0).
	pathA := sweepDrift(r3.Vec{X: -25, Y: 18.5}, .1)
	pathB := sweepDrift(r3.Vec{X: 25, Y: 1.5}, .1)
	pathA.From, pathB.From = fromA, fromB
	pathA.Center, pathB.Center = r3.Vec{X: -5}, r3.Vec{X: 5}
	pathA.AngularVelocity.Z = units.RadiansPerSecond(-.75)
	pathB.AngularVelocity.Z = units.RadiansPerSecond(-.75)
	req := sweepRequest()
	req.StartPolicy = decad.ContinueSeparatingTouch

	for _, order := range []struct {
		name         string
		a, b         *decad.Body
		pathA, pathB decad.RigidDriftSegment
		normalX      float64
	}{{"forward", a, b, pathA, pathB, 1}, {"reverse", b, a, pathB, pathA, -1}} {
		t.Run(order.name, func(t *testing.T) {
			report, err := doc.SweepPair(t.Context(), order.a, order.b, order.pathA, order.pathB, req)
			require.NoError(t, err)
			require.Equal(t, decad.SweepDepartedClear, report.Outcome, "cause=%v", report.Cause)
			require.True(t, report.HasAffineReplayProof())
			require.NotNil(t, report.InitialEvent)
			require.Equal(t, order.normalX, report.InitialEvent.Manifold.Points[0].Normal.Value.X)
			require.Positive(t, report.Departure.GapAtUntil.Value.Base())
			for _, elapsed := range []float64{.025, .05, .1} {
				poseA, poseB, err := report.CertifiedPosesAt(units.Seconds(elapsed))
				require.NoError(t, err)
				require.NotEqual(t, order.pathA.From.Basis(), poseA.Basis())
				require.NotEqual(t, order.pathB.From.Basis(), poseB.Basis())
				observed, err := doc.ContactPair(t.Context(), order.a, order.b,
					poseA, poseB, contactRequest())
				require.NoError(t, err)
				require.Equal(t, decad.ContactSeparated, observed.Relation)
				require.Positive(t, observed.Gap.Value.Base()-observed.Gap.Bound.Base())
			}
		})
	}
	require.Equal(t, before, doc.Bodies())

	t.Run("off-center pivot", func(t *testing.T) {
		bad := pathA
		bad.Center.X = -4
		report, err := doc.SweepPair(t.Context(), a, b, bad, pathB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, report.Outcome)
		require.False(t, report.HasAffineReplayProof())
	})
	t.Run("touching tangent departure", func(t *testing.T) {
		tangentA := sweepDrift(r3.Vec{}, 1)
		tangentB := sweepDrift(r3.Vec{Y: 1}, 1)
		tangentB.From = contactPose(t, r3.Vec{X: 10})
		tangentB.Center = r3.Vec{X: 10}
		tangentB.AngularVelocity.Z = units.RadiansPerSecond(-.75)
		report, err := doc.SweepPair(t.Context(), a, b, tangentA, tangentB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepDepartedClear, report.Outcome, "cause=%v", report.Cause)
		_, _, err = report.CertifiedPosesAt(units.Seconds(.5))
		require.NoError(t, err)
	})
	t.Run("closing touch", func(t *testing.T) {
		closingA := pathA
		closingA.LinearVelocity.X = units.MillimetersPerSecond(50)
		report, err := doc.SweepPair(t.Context(), a, b, closingA, pathB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, report.Outcome)
		require.False(t, report.HasAffineReplayProof())
	})
	t.Run("separated rotating graze", func(t *testing.T) {
		grazeA := sweepDrift(r3.Vec{}, 1)
		grazeB := sweepDrift(r3.Vec{X: -40}, 1)
		grazeB.From = contactPose(t, r3.Vec{X: 20, Y: 10})
		grazeB.Center = r3.Vec{X: 20, Y: 10}
		grazeB.AngularVelocity.Y = units.RadiansPerSecond(1.5)
		grazeB.AngularVelocity.Z = units.RadiansPerSecond(-2)
		// ContinueSeparatingTouch needs a touching start.
		refused, err := doc.SweepPair(t.Context(), a, b, grazeA, grazeB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, refused.Outcome)
		require.False(t, refused.HasAffineReplayProof())
		// The spinning ball's center is rebuilt from its exact affine path,
		// so the dyadic tangent instant samples an exact touch.
		report, err := doc.SweepPair(t.Context(), a, b, grazeA, grazeB, sweepRequest())
		require.NoError(t, err)
		require.Equal(t, decad.SweepGrazingTouch, report.Outcome, "cause=%v", report.Cause)
		require.Equal(t, units.Scalar(.5), report.Event.At.Fraction)
		require.Equal(t, r3.Vec{Y: 1}, report.Event.Manifold.Points[0].Normal.Value)
		require.True(t, report.HasAffineReplayProof())
		for _, elapsed := range []float64{.25, .5, .75} {
			poseA, poseB, err := report.CertifiedPosesAt(units.Seconds(elapsed))
			require.NoError(t, err)
			require.NotEqual(t, grazeB.From.Basis(), poseB.Basis())
			observed, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, contactRequest())
			require.NoError(t, err)
			want := decad.ContactSeparated
			if elapsed == .5 {
				want = decad.ContactTouching
			}
			require.Equal(t, want, observed.Relation)
		}
	})
}

func TestSweepPairRotatingSourceSpheresClear(t *testing.T) {
	doc := decad.New()
	b, c := ballBody(t, doc, 5), ballBody(t, doc, 5)
	pathB := sweepDrift(r3.Vec{X: 7.1875, Y: .625}, .1)
	pathC := sweepDrift(r3.Vec{X: .625, Y: 7.1875}, .1)
	pathB.From = contactPose(t, r3.Vec{X: 10})
	pathC.From = contactPose(t, r3.Vec{Y: 10})
	pathB.Center = r3.Vec{X: 10}
	pathC.Center = r3.Vec{Y: 10}
	pathB.AngularVelocity.Z = units.RadiansPerSecond(-.3125)
	pathC.AngularVelocity.Z = units.RadiansPerSecond(.3125)
	req := sweepRequest()
	req.StartPolicy = decad.StopAtInitialContact

	initial, err := doc.ContactPair(t.Context(), b, c, pathB.From, pathC.From, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, initial.Relation)
	report, err := doc.SweepPair(t.Context(), b, c, pathB, pathC, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepClear, report.Outcome, "cause=%v", report.Cause)
	require.True(t, report.HasAffineReplayProof())
	for _, elapsed := range []float64{0, .025, .1} {
		poseB, poseC, replayErr := report.CertifiedPosesAt(units.Seconds(elapsed))
		require.NoError(t, replayErr, "time=%v", elapsed)
		observed, contactErr := doc.ContactPair(t.Context(), b, c, poseB, poseC, contactRequest())
		require.NoError(t, contactErr)
		require.Equal(t, decad.ContactSeparated, observed.Relation)
		require.Positive(t, observed.Gap.Value.Base()-observed.Gap.Bound.Base())
	}
	intervalB, intervalC, err := report.CertifiedPosesAtInterval(
		units.Seconds(.05), units.Seconds(0), units.Seconds(.1))
	require.NoError(t, err)
	intervalContact, err := doc.ContactPair(t.Context(), b, c, intervalB, intervalC, contactRequest())
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, intervalContact.Relation)

	reversed, err := doc.SweepPair(t.Context(), c, b, pathC, pathB, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepClear, reversed.Outcome)
	_, _, err = reversed.CertifiedPosesAt(units.Seconds(.05))
	require.NoError(t, err)

	t.Run("off-center pivot", func(t *testing.T) {
		bad := pathB
		bad.Center.X = 11
		undecided, err := doc.SweepPair(t.Context(), b, c, bad, pathC, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, undecided.Outcome)
		require.False(t, undecided.HasAffineReplayProof())
	})
	t.Run("impact", func(t *testing.T) {
		closing := pathB
		closing.LinearVelocity.X = units.MillimetersPerSecond(-40)
		closing.LinearVelocity.Y = units.MillimetersPerSecond(40)
		impact, err := doc.SweepPair(t.Context(), b, c, closing, pathC, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepImpactBracket, impact.Outcome, "cause=%v", impact.Cause)
		require.True(t, impact.HasAffineReplayProof())
	})
}

// spinningImpactPrecision keeps the rounding of every closed-form quantity
// far below the float spacing of the inputs, so it cannot decide a
// comparison against a sweep value.
const spinningImpactPrecision = 600

func exactFloat(v float64) *big.Float {
	return new(big.Float).SetPrec(spinningImpactPrecision).SetFloat64(v)
}

func exactVec(v r3.Vec) [3]*big.Float {
	return [3]*big.Float{exactFloat(v.X), exactFloat(v.Y), exactFloat(v.Z)}
}

// spinningCenter is the exact center of a ball spinning about itself under a
// drift: start + v·t, with no rotation term.
func spinningCenter(start, velocity r3.Vec, elapsed *big.Float) [3]*big.Float {
	center := exactVec(start)
	for i, v := range exactVec(velocity) {
		center[i].Add(center[i], new(big.Float).SetPrec(spinningImpactPrecision).Mul(v, elapsed))
	}
	return center
}

func exactDot(a, b [3]*big.Float) *big.Float {
	sum := exactFloat(0)
	for i := range 3 {
		sum.Add(sum, new(big.Float).SetPrec(spinningImpactPrecision).Mul(a[i], b[i]))
	}
	return sum
}

func exactSub(a, b [3]*big.Float) [3]*big.Float {
	var out [3]*big.Float
	for i := range 3 {
		out[i] = new(big.Float).SetPrec(spinningImpactPrecision).Sub(a[i], b[i])
	}
	return out
}

// exactDistanceTo is the Euclidean distance from a published float point to
// an exact point.
func exactDistanceTo(v r3.Vec, exact [3]*big.Float) *big.Float {
	d := exactSub(exactVec(v), exact)
	return new(big.Float).SetPrec(spinningImpactPrecision).Sqrt(exactDot(d, d))
}

// spinningFirstTouch is the closed-form earliest root of
// |pB−pA + (vB−vA)t|² = (rA+rB)².
func spinningFirstTouch(startA, startB, velocityA, velocityB r3.Vec, radius float64) *big.Float {
	p := exactSub(exactVec(startB), exactVec(startA))
	w := exactSub(exactVec(velocityB), exactVec(velocityA))
	r := exactFloat(radius)
	a := exactDot(w, w)
	b := exactDot(p, w)
	c := new(big.Float).SetPrec(spinningImpactPrecision).Sub(exactDot(p, p),
		new(big.Float).SetPrec(spinningImpactPrecision).Mul(r, r))
	discriminant := new(big.Float).SetPrec(spinningImpactPrecision).Sub(
		new(big.Float).SetPrec(spinningImpactPrecision).Mul(b, b),
		new(big.Float).SetPrec(spinningImpactPrecision).Mul(a, c))
	root := new(big.Float).SetPrec(spinningImpactPrecision).Sqrt(discriminant)
	root.Add(root, b)
	root.Neg(root)
	return root.Quo(root, a)
}

// spinningSeconds is a dyadic step, so each full displacement is exact.
const spinningSeconds = .25

// spinningDrift is a ball's drift about its own exact source center.
func spinningDrift(t *testing.T, start, velocity, spin r3.Vec) decad.RigidDriftSegment {
	t.Helper()
	path := sweepDrift(velocity, spinningSeconds)
	path.From, path.Center = contactPose(t, start), start
	path.AngularVelocity = decad.QuantityVec{X: units.RadiansPerSecond(spin.X),
		Y: units.RadiansPerSecond(spin.Y), Z: units.RadiansPerSecond(spin.Z)}
	return path
}

// TestSweepPairRotatingSourceSpheresImpact brackets the first impact of two
// balls that spin about their own centers. A ball's occupied set ignores its
// spin, so the exact closed-form contact time comes from the affine center
// paths alone. The starts and velocities are off the dyadic grid, so every
// sampled center rounds away from that path.
//
// Legs shown to fail by deleting each one, watching the named fixture go red,
// and restoring it:
//   - the pivot gate (each drift pivot equals its exact source center):
//     every "off-center pivot" subtest publishes an outcome for a circular
//     center path;
//   - the center path itself (each displacement is velocity times duration):
//     zeroing the Z displacement moves the transverse bracket off the
//     closed-form root;
//   - the bracket search over the exact squared center distance: dropping
//     the doubled cross term of squaredGap loses the transverse bracket and
//     TestSweepPairRotatingSourceSpheresDepart's rotating graze;
//   - the center-path deviation charged in transferManifold: neither the
//     separation bound nor the witness bounds cover the exact ideal pair;
//   - the sampled center rebuilt from the exact affine path
//     (sourceSpherePathPoseAt): the rotating graze, whose float rotation
//     about (20, 10, 0) moves its center by an ulp, loses its exact touch;
//   - replay's center deviation against PointResolution: "tight replay"
//     accepts a rounded pose off the dyadic grid.
func TestSweepPairRotatingSourceSpheresImpact(t *testing.T) {
	doc := decad.New()
	a, b := ballBody(t, doc, 5), ballBody(t, doc, 5)
	const seconds = spinningSeconds
	spinA, spinB := r3.Vec{Z: 3}, r3.Vec{Y: 1.5, Z: -2}
	for _, fixture := range []struct {
		name                 string
		startA, startB       r3.Vec
		velocityA, velocityB r3.Vec
	}{
		{"transverse", r3.Vec{X: 100.1, Y: 50.3, Z: 20.7}, r3.Vec{X: 120.1, Y: 56.3, Z: 20.7},
			r3.Vec{X: 40.1, Y: .3}, r3.Vec{X: -39.7, Y: -3.5, Z: .2}},
		{"axial", r3.Vec{X: 100.1, Y: 50.3, Z: 20.7}, r3.Vec{X: 120.3, Y: 50.3, Z: 20.7},
			r3.Vec{X: 40.1}, r3.Vec{X: -39.7}},
	} {
		for _, order := range []struct {
			name    string
			forward bool
		}{{"forward", true}, {"reverse", false}} {
			t.Run(fixture.name+"/"+order.name, func(t *testing.T) {
				startA, startB := fixture.startA, fixture.startB
				velocityA, velocityB := fixture.velocityA, fixture.velocityB
				first, second := a, b
				pathA := spinningDrift(t, startA, velocityA, spinA)
				pathB := spinningDrift(t, startB, velocityB, spinB)
				if !order.forward {
					startA, startB = startB, startA
					velocityA, velocityB = velocityB, velocityA
					first, second = b, a
					pathA, pathB = pathB, pathA
				}
				req := sweepRequest()
				report, err := doc.SweepPair(t.Context(), first, second, pathA, pathB, req)
				require.NoError(t, err)
				require.Equal(t, decad.SweepImpactBracket, report.Outcome, "cause=%v", report.Cause)
				require.True(t, report.HasAffineReplayProof())

				// The bracket holds the closed-form root, clear on its left.
				root := spinningFirstTouch(startA, startB, velocityA, velocityB, 10)
				from, to := report.Bracket.From, report.Bracket.To
				require.Zero(t, from.Elapsed.Bound.Base())
				require.Zero(t, to.Elapsed.Bound.Base())
				require.Equal(t, -1, exactFloat(from.Elapsed.Value.Base()).Cmp(root))
				require.Equal(t, -1, root.Cmp(exactFloat(to.Elapsed.Value.Base())))
				require.LessOrEqual(t, to.Elapsed.Value.Base()-from.Elapsed.Value.Base(),
					req.TimeResolution.Base())

				// The event describes the exact ideal pair at the right edge.
				event := report.Event
				require.Equal(t, decad.ContactOverlapping, event.Relation)
				require.Len(t, event.Manifold.Points, 1)
				point := event.Manifold.Points[0]
				right := exactFloat(to.Elapsed.Value.Base())
				centerA := spinningCenter(startA, velocityA, right)
				centerB := spinningCenter(startB, velocityB, right)
				line := exactSub(centerB, centerA)
				distance := new(big.Float).SetPrec(spinningImpactPrecision).Sqrt(exactDot(line, line))
				var normal, onA, onB [3]*big.Float
				for i := range 3 {
					normal[i] = new(big.Float).SetPrec(spinningImpactPrecision).Quo(line[i], distance)
					offset := new(big.Float).SetPrec(spinningImpactPrecision).Mul(normal[i], exactFloat(5))
					onA[i] = new(big.Float).SetPrec(spinningImpactPrecision).Add(centerA[i], offset)
					onB[i] = new(big.Float).SetPrec(spinningImpactPrecision).Sub(centerB[i], offset)
				}
				separation := new(big.Float).SetPrec(spinningImpactPrecision).Sub(distance, exactFloat(10))
				require.Negative(t, separation.Sign())
				separationError := new(big.Float).SetPrec(spinningImpactPrecision).Sub(
					exactFloat(point.Separation.Value.Base()), separation)
				require.LessOrEqual(t, separationError.Abs(separationError).Cmp(
					exactFloat(point.Separation.Bound.Base())), 0)
				require.LessOrEqual(t, exactDistanceTo(point.OnA.Value, onA).Cmp(
					exactFloat(point.OnA.Bound.Base())), 0)
				require.LessOrEqual(t, exactDistanceTo(point.OnB.Value, onB).Cmp(
					exactFloat(point.OnB.Bound.Base())), 0)
				require.LessOrEqual(t, exactDistanceTo(point.Normal.Value, normal).Cmp(
					exactFloat(point.Normal.Bound.Base())), 0)
				require.LessOrEqual(t, point.OnA.Bound.Base(), req.PointResolution.Base())

				// Replay keeps the spin and the certified relation up to the
				// bracket's right edge, and refuses beyond it.
				for _, elapsed := range []float64{seconds / 8, from.Elapsed.Value.Base(), to.Elapsed.Value.Base()} {
					poseA, poseB, err := report.CertifiedPosesAt(units.Seconds(elapsed))
					require.NoError(t, err, "elapsed=%v", elapsed)
					require.NotEqual(t, pathA.From.Basis(), poseA.Basis())
					require.NotEqual(t, pathB.From.Basis(), poseB.Basis())
					observed, err := doc.ContactPair(t.Context(), first, second, poseA, poseB, contactRequest())
					require.NoError(t, err)
					if elapsed == to.Elapsed.Value.Base() {
						require.Equal(t, decad.ContactOverlapping, observed.Relation)
						continue
					}
					require.Equal(t, decad.ContactSeparated, observed.Relation)
					require.Positive(t, observed.Gap.Value.Base()-observed.Gap.Bound.Base())
				}
				_, _, err = report.CertifiedPosesAt(units.Seconds(seconds))
				require.ErrorIs(t, err, decad.ErrUnsupported)
				_, _, err = report.CertifiedPosesAtInterval(units.Seconds(.01), units.Seconds(0),
					units.Seconds(seconds))
				require.NoError(t, err)
			})
		}
	}

	t.Run("off-center pivot", func(t *testing.T) {
		pathA := spinningDrift(t, r3.Vec{X: 100.1, Y: 50.3, Z: 20.7}, r3.Vec{X: 40}, spinA)
		pathB := spinningDrift(t, r3.Vec{X: 120.1, Y: 56.3, Z: 20.7}, r3.Vec{X: -40, Y: -3.5}, spinB)
		pathB.Center.Y += 1
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, sweepRequest())
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, report.Outcome)
		require.False(t, report.HasAffineReplayProof())
	})
	t.Run("separating policy needs touch", func(t *testing.T) {
		pathA := spinningDrift(t, r3.Vec{X: 100.1, Y: 50.3, Z: 20.7}, r3.Vec{X: 40}, spinA)
		pathB := spinningDrift(t, r3.Vec{X: 120.1, Y: 56.3, Z: 20.7}, r3.Vec{X: -40, Y: -3.5}, spinB)
		req := sweepRequest()
		req.StartPolicy = decad.ContinueSeparatingTouch
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, report.Outcome)
		require.Equal(t, decad.SweepContactUnsupported, report.Cause)
	})
	t.Run("tight replay", func(t *testing.T) {
		// Dyadic starts put every bracket sample on its exact center, so the
		// sweep certifies at a 1e-18 mm resolution; a replay time off that
		// grid rounds its centers by more and is refused.
		pathA := spinningDrift(t, r3.Vec{X: -10}, r3.Vec{X: 40}, spinA)
		pathB := spinningDrift(t, r3.Vec{X: 10}, r3.Vec{X: -40}, spinB)
		req := sweepRequest()
		req.PointResolution = units.Millimeters(1e-18)
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepImpactBracket, report.Outcome, "cause=%v", report.Cause)
		_, _, err = report.CertifiedPosesAt(report.Bracket.From.Elapsed.Value)
		require.NoError(t, err)
		_, _, err = report.CertifiedPosesAt(units.Seconds(.0301))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}
