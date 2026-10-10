package decad

import (
	"math"
	"math/big"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/motionoption"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// motionRunFor builds VerifyMotion's own run state for one call, so a test
// can drive a single pose through the production path.
func motionRunFor(t *testing.T, doc *Document, moving []*Body, m Motion) *motionRun {
	t.Helper()
	spec, err := resolveMotion(m)
	require.NoError(t, err)
	run := &motionRun{ctx: t.Context(), d: doc, spec: spec, cfg: motionConfig{Rel: 1e-3}, cache: &bodyGeomCache{}}
	run.setup(moving)
	return run
}

// TestMotionPoseDeviationReachesThePoseGap is the half of §9 test 8 the
// public report cannot isolate: the arm at 90°, stated in degrees, about the
// origin and about a pivot 1e6 mm away. η is nonzero at both — the float
// π/180, math.Sincos and Rodrigues' formula each round — and the far pivot's
// η carries the rounding of center − R·center at its magnitude, so it is
// orders larger. The same right angle stated in radians takes the π
// enclosure path instead and is charged the same way. The pose's published
// gap interval is the kernel's own interval for the same transient
// placement, widened by η on both sides.
//
// Legs seen to fail when deleted: charging η into the pose gap (the pose's
// interval then equals the kernel's); η's translation term (the far pivot's η
// falls to the near pivot's); and η's linear term (the near pivot's η is
// zero).
func TestMotionPoseDeviationReachesThePoseGap(t *testing.T) {
	t.Parallel()
	deviation := func(t *testing.T, cx float64, to units.Value) float64 {
		t.Helper()
		doc := New()
		arm := internalBoxBody(t, doc, 0, -14, 48, 14, 10)
		wall := internalBoxBody(t, doc, cx-20, -cx+58, cx+20, -cx+68, 10)
		swing := Revolute{Center: r3.NewVec(cx, 0, 0), Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: to}
		run := motionRunFor(t, doc, []*Body{arm}, swing)
		pose, err := run.evaluatePose(big.NewRat(1, 1), to)
		require.NoError(t, err)
		got := pose.pairs[0][0]
		require.True(t, got.hasGap)

		placement := arm.payload.transform()
		composed, err := placement.Then(pose.result.Pose)
		require.NoError(t, err)
		transient, err := arm.payload.placed(t.Context(), doc, transientProducer, composed)
		require.NoError(t, err)
		_, fast := clearanceAxisBoxes(transient, wall)
		require.False(t, fast, `a rotated arm is not an axis box`)
		res, err := clearancePair(t.Context(), transient, wall, boxesDisjoint(transient.bounds, wall.bounds))
		require.NoError(t, err)
		require.Equal(t, pairDisjoint, res.verdict)

		eta, _ := motionbound.PoseDeviation(composed, placement, run.spec.Frame.At(pose.param), run.movers[0].r0)
		require.Greater(t, eta, 0.0)
		require.Less(t, got.lo, res.lo, `η lowers the proven lower end`)
		require.Greater(t, got.hi, res.hi, `η raises the proven upper end`)
		require.LessOrEqual(t, got.lo, 10.0)
		require.GreaterOrEqual(t, got.hi, 10.0)
		return eta
	}
	near := deviation(t, 0, units.Degrees(90))
	far := deviation(t, 1e6, units.Degrees(90))
	require.Greater(t, deviation(t, 0, units.Radians(math.Pi/2)), 0.0)
	// The near pivot's η is the basis error times a ~100 mm record radius; the
	// far pivot's adds the pivot offset's rounding at 1e6 mm. Asserted as a
	// ratio, never as a literal: FMA contraction moves both by an ulp.
	require.Greater(t, far, 1000*near)
}

// TestMotionPoseDeviationIsZeroForAnExactPose: the identity pose at 0°, a
// right-angle pose about an axis whose rotation float arithmetic states
// exactly, and a dyadic translation along a unit axis are each the ideal
// pose bit for bit, so η is exactly zero and an Exact row stays Exact.
func TestMotionPoseDeviationIsZeroForAnExactPose(t *testing.T) {
	t.Parallel()
	doc := New()
	arm := internalBoxBody(t, doc, 0, -14, 48, 14, 10)
	cases := []struct {
		name string
		m    Motion
		at   units.Value
	}{
		{"identity rotation", Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Degrees(90)}, units.Degrees(0)},
		{"dyadic translation", Prismatic{Dir: r3.NewVec(0, 4, 0), From: units.Millimeters(0), To: units.Millimeters(30)}, units.Millimeters(30)},
		{"translation in another unit", Prismatic{Dir: r3.NewVec(1, 0, 0), From: units.Millimeters(0), To: units.Millimeters(30)}, units.Centimeters(2)},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			spec, err := resolveMotion(tc.m)
			require.NoError(t, err)
			pose, err := tc.m.PoseAt(tc.at)
			require.NoError(t, err)
			param, ok := motionbound.ExactMotionParam(tc.at)
			require.True(t, ok)
			placement := arm.payload.transform()
			composed, err := placement.Then(pose)
			require.NoError(t, err)
			eta, linear := motionbound.PoseDeviation(composed, placement, spec.Frame.At(param), math.Inf(1))
			require.Zero(t, eta)
			require.Zero(t, linear)
		})
	}
}

// TestMotionParamSinCosEnclosesTheAngle checks the ideal rotation's sine and
// cosine for each way an angle can be stated: a whole number of quarter turns
// is a point, a degree count is an exact turn, a radian count is enclosed
// through π, and an interpolation between the two mixes both. Each enclosure
// is far below float resolution and agrees with math.Sincos to float
// accuracy.
func TestMotionParamSinCosEnclosesTheAngle(t *testing.T) {
	t.Parallel()
	exact := []struct {
		at       units.Value
		sin, cos int64
	}{
		{units.Degrees(0), 0, 1},
		{units.Degrees(90), 1, 0},
		{units.Degrees(-180), 0, -1},
		{units.Degrees(630), -1, 0},
		{units.Radians(0), 0, 1},
	}
	for _, tc := range exact {
		p, ok := motionbound.ExactMotionParam(tc.at)
		require.True(t, ok)
		sin, cos := motionbound.ParamSinCos(p)
		require.Zero(t, sin.Lo.Cmp(big.NewRat(tc.sin, 1)), tc.at.String())
		require.Zero(t, sin.Hi.Cmp(big.NewRat(tc.sin, 1)), tc.at.String())
		require.Zero(t, cos.Lo.Cmp(big.NewRat(tc.cos, 1)), tc.at.String())
		require.Zero(t, cos.Hi.Cmp(big.NewRat(tc.cos, 1)), tc.at.String())
	}

	fromDeg, ok := motionbound.ExactMotionParam(units.Degrees(10))
	require.True(t, ok)
	toRad, ok := motionbound.ExactMotionParam(units.Radians(1))
	require.True(t, ok)
	approx := []struct {
		name  string
		p     motionbound.MotionParam
		angle float64
	}{
		{"degrees", fromDeg, 10 * math.Pi / 180},
		{"negative degrees", mustMotionParam(t, units.Degrees(-450.25)), -450.25 * math.Pi / 180},
		{"radians", toRad, 1},
		{"negative radians", mustMotionParam(t, units.Radians(-2.5)), -2.5},
		{"degrees to radians halfway", fromDeg.Lerp(toRad, big.NewRat(1, 2)), 5*math.Pi/180 + 0.5},
	}
	tiny := big.NewRat(1, 1)
	tiny.SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 150))
	for _, tc := range approx {
		sin, cos := motionbound.ParamSinCos(tc.p)
		for _, iv := range []struct {
			got  proofbound.RatInterval
			want float64
		}{{sin, math.Sin(tc.angle)}, {cos, math.Cos(tc.angle)}} {
			width := new(big.Rat).Sub(iv.got.Hi, iv.got.Lo)
			require.Equal(t, 1, tiny.Cmp(width), `%s: enclosure width`, tc.name)
			mid, _ := new(big.Rat).Add(iv.got.Lo, iv.got.Hi).Float64()
			require.InDelta(t, iv.want, mid/2, 4e-16, tc.name)
		}
	}
}

func mustMotionParam(t *testing.T, v units.Value) motionbound.MotionParam {
	t.Helper()
	p, ok := motionbound.ExactMotionParam(v)
	require.True(t, ok)
	return p
}

// TestMotionAxisRadiusReadsTheBox checks ρ_max on the arm of §9: its far
// corners (48, ±14) sit exactly 50 mm from the Z axis through the origin, and
// moving the axis to (0, −14) moves the far corner (48, 14) to exactly
// √(48² + 28²) mm from it.
func TestMotionAxisRadiusReadsTheBox(t *testing.T) {
	t.Parallel()
	doc := New()
	arm := internalBoxBody(t, doc, 0, -14, 48, 14, 10)
	for _, tc := range []struct {
		center r3.Vec
		want   float64
	}{
		{r3.Vec{}, 50},
		{r3.NewVec(0, -14, 0), math.Sqrt(48*48 + 28*28)},
	} {
		spec, err := resolveMotion(Revolute{Center: tc.center, Axis: r3.NewVec(0, 0, 3), From: units.Degrees(0), To: units.Degrees(90)})
		require.NoError(t, err)
		rho := motionbound.MoverAxisRadius(arm.bounds, spec.Frame)
		require.GreaterOrEqual(t, rho, tc.want)
		require.InDelta(t, tc.want, rho, 1e-12)
	}
}

// TestVerifyMotionKeepsTheNextProducerIdentity is §9 test 7's document
// identity half: a transient placement must mint no live producer, level, or
// curve identity. A Duplicate after the call receives the next producer.
func TestVerifyMotionKeepsTheNextProducerIdentity(t *testing.T) {
	t.Parallel()
	doc := New()
	cube := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	internalBoxBody(t, doc, 25, 2, 35, 8, 10)
	before := doc.nextProducerID()
	levelBefore, curveBefore := doc.nextLevel, doc.nextCurve
	report, err := doc.VerifyMotion(t.Context(), []*Body{cube}, Prismatic{Dir: r3.NewVec(1, 0, 0), From: units.Millimeters(0), To: units.Millimeters(30)}, WithResolution(units.Millimeters(100)))
	require.NoError(t, err)
	require.Len(t, report.Poses, 2)
	require.Equal(t, before, doc.nextProducerID())
	require.Equal(t, levelBefore, doc.nextLevel)
	require.Equal(t, curveBefore, doc.nextCurve)
	const readers = 4
	reports := make([]*MotionReport, readers)
	errs := make([]error, readers)
	var wg sync.WaitGroup
	for i := range reports {
		wg.Go(func() {
			reports[i], errs[i] = doc.VerifyMotion(t.Context(), []*Body{cube},
				Prismatic{Dir: r3.NewVec(1, 0, 0), From: units.Millimeters(0), To: units.Millimeters(30)},
				WithResolution(units.Millimeters(100)))
		})
	}
	wg.Wait()
	for i := range reports {
		require.NoError(t, errs[i])
		require.Equal(t, report, reports[i])
	}
	require.Equal(t, levelBefore, doc.nextLevel)
	require.Equal(t, curveBefore, doc.nextCurve)
	dup, err := cube.Duplicate(t.Context())
	require.NoError(t, err)
	require.Equal(t, before, dup.originProducer())
}

func TestVerifyMotionRevolveKeepsLiveDenotationIdentities(t *testing.T) {
	t.Parallel()
	doc := New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 8)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	mover, err := doc.Revolve(s, s.Profiles()[0],
		SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}, FullRevolution{})
	require.NoError(t, err)
	static := internalBoxBody(t, doc, 15, -10, 25, 10, 10)
	before := doc.Bodies()
	levelBefore, curveBefore := doc.nextLevel, doc.nextCurve
	report, err := doc.VerifyMotion(t.Context(), []*Body{mover},
		Prismatic{Dir: r3.NewVec(1, 0, 0), From: units.Millimeters(0), To: units.Millimeters(20)},
		WithResolution(units.Millimeters(100)))
	require.NoError(t, err)
	require.Len(t, report.Poses, 2)
	require.Equal(t, static, report.Against[0])
	require.Len(t, report.Poses[0].Diagnostics, 1)
	require.Equal(t, DiagUndecidedClearance, report.Poses[0].Diagnostics[0].Code)
	require.Equal(t, before, doc.Bodies())
	require.Equal(t, levelBefore, doc.nextLevel)
	require.Equal(t, curveBefore, doc.nextCurve)
}

// TestVerifyMotionRefusesABodyItDidNotBuild is §9 test 9's row the public
// API cannot reach: a mover with no payload is ErrUnsupported, as for Placed.
func TestVerifyMotionRefusesABodyItDidNotBuild(t *testing.T) {
	t.Parallel()
	doc := New()
	foreignBuilt := &Body{doc: doc, kind: BodySolid}
	doc.commit(foreignBuilt)
	before := doc.Bodies()
	report, err := doc.VerifyMotion(t.Context(), []*Body{foreignBuilt}, Prismatic{Dir: r3.NewVec(1, 0, 0), From: units.Millimeters(0), To: units.Millimeters(1)})
	require.ErrorIs(t, err, ErrUnsupported)
	require.Nil(t, report)
	require.Equal(t, before, doc.Bodies())
}

// farCornerFixture is the arm swinging 45° about a pivot (pivot, 0, 0) far
// from the origin, against a 20 mm block whose top face sits depth below the
// arm's lowest corner at 45°: the corner pokes into the block as a wedge of
// volume depth² × 10 mm³. The pivot's magnitude makes the pose's η — chiefly
// the rounding of center − R·center — large enough for the swept-volume
// allowance to matter. The block spans z ∈ [−5, 15], past the arm's caps, so
// the read-only proof can measure the overlap.
func farCornerFixture(t *testing.T, pivot, depth float64) (*Document, *Body, *Body, Revolute) {
	t.Helper()
	doc := New()
	arm := internalBoxBody(t, doc, 0, -14, 48, 14, 10)
	half := math.Sqrt(0.5)
	lowest := -(pivot + 14) * half
	cornerX := pivot - pivot*half + 14*half
	block := internalBoxBody(t, doc, -10, -20, 10, 0, 20)
	shift, err := r3.Translation(r3.NewVec(cornerX, lowest+depth, -5))
	require.NoError(t, err)
	block, err = block.Placed(t.Context(), shift)
	require.NoError(t, err)
	swing := Revolute{Center: r3.NewVec(pivot, 0, 0), Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Degrees(45)}
	return doc, arm, block, swing
}

// farCornerOverlap measures, through the production pieces, what the 45°
// pose of farCornerFixture proves: the read-only overlap volume at the float
// pose and the swept-volume allowance that pose's η charges.
func farCornerOverlap(t *testing.T, doc *Document, arm, block *Body, swing Revolute) (Measurement, interferenceOutcome, float64) {
	t.Helper()
	run := motionRunFor(t, doc, []*Body{arm}, swing)
	pose, err := swing.PoseAt(swing.To)
	require.NoError(t, err)
	placement := arm.payload.transform()
	composed, err := placement.Then(pose)
	require.NoError(t, err)
	transient, err := arm.payload.placed(t.Context(), doc, transientProducer, composed)
	require.NoError(t, err)
	eta, linear := motionbound.PoseDeviation(composed, placement, run.spec.Frame.At(run.spec.ToP), run.movers[0].r0)
	require.Greater(t, eta, 0.0)
	allowance := proofbound.SweptVolumeAllow(eta, motionbound.PathAreaUpper(run.movers[0].area, linear, run.movers[0].sigma, run.stretchEnd))
	res, err := clearancePair(t.Context(), transient, block, false)
	require.NoError(t, err)
	volume, outcome, err := measuredInterference(t.Context(), transient, block, res, pairMeshes{})
	require.NoError(t, err)
	return volume, outcome, allowance
}

// TestMotionCollisionTransferWidensTheBound is the transfer test of
// docs/motion-check-design.md §10: a 45° pose about a pivot 1e9 mm away has
// a nonzero η, and its 0.02 mm-deep corner overlap clears the swept-volume
// allowance by a wide margin, so it is published as a Collision whose Bound
// is the read-only proof's own widened by exactly that allowance.
//
// Legs seen to fail when deleted: the widening of the published bound (the
// published Bound equals the read-only proof's own).
func TestMotionCollisionTransferWidensTheBound(t *testing.T) {
	t.Parallel()
	doc, arm, block, swing := farCornerFixture(t, 1e9, 0.02)
	volume, outcome, allowance := farCornerOverlap(t, doc, arm, block, swing)
	require.Equal(t, interferenceMeasured, outcome)
	require.Greater(t, allowance, 0.0)
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(), allowance, `the fixture's overlap clears the allowance`)

	report, err := doc.VerifyMotion(t.Context(), []*Body{arm}, swing, WithResolution(units.Degrees(360)))
	require.NoError(t, err)
	require.Equal(t, Interfering, report.Status)
	require.Len(t, report.Collisions, 1)
	collision := report.Collisions[0]
	require.Equal(t, units.Degrees(45), collision.At)
	require.Equal(t, volume.Value, collision.Volume.Value)
	require.Equal(t, Approximate, collision.Volume.Exactness)
	require.Equal(t, proofbound.AbsSumUpper(volume.Bound.Base(), allowance), collision.Volume.Bound.Base())
	require.Greater(t, collision.Volume.Bound.Base(), volume.Bound.Base())
	require.Greater(t, collision.Volume.Value.Base()-collision.Volume.Bound.Base(), 0.0)
	end := report.Poses[len(report.Poses)-1]
	require.Equal(t, []Interference{{A: arm, B: block, Volume: collision.Volume}}, end.Interferences)
}

// TestMotionOverlapThatDoesNotTransferIsUndecided pins the two overlaps §5.1
// refuses to call a collision. A 0.008 mm-deep corner overlap at a pivot
// 2e9 mm away is measured positive at the float pose, but its proven lower
// end does not clear the allowance the pose's η charges; and an overlap the
// read-only proof cannot measure at all — a revolved block sharing the
// arm's cap planes, a contact it refuses — has no volume to carry. Each reads
// DiagUndecidedInterference at the pose, never a Collision, and leaves the
// interval undecided.
//
// Legs seen to fail when deleted: the allowance in the transfer comparison
// (the thin overlap transfers and the pose publishes a Collision).
func TestMotionOverlapThatDoesNotTransferIsUndecided(t *testing.T) {
	t.Parallel()
	requireUndecided := func(t *testing.T, doc *Document, arm, block *Body, swing Revolute) {
		t.Helper()
		before := doc.Bodies()
		report, err := doc.VerifyMotion(t.Context(), []*Body{arm}, swing, WithResolution(units.Degrees(360)))
		require.NoError(t, err)
		require.Equal(t, before, doc.Bodies())
		require.Empty(t, report.Collisions)
		require.Equal(t, Suspect, report.Status)
		require.Equal(t, IntervalUndecided, report.Intervals[0].Outcome)
		end := report.Poses[len(report.Poses)-1]
		require.Empty(t, end.Interferences)
		require.Empty(t, end.Clearances)
		require.Len(t, end.Diagnostics, 1)
		d := end.Diagnostics[0]
		require.Equal(t, DiagUndecidedInterference, d.Code)
		require.Equal(t, &DiagnosticPair{A: arm, B: block}, d.Pair)
		require.Equal(t, units.Degrees(45), *d.At)
		require.Equal(t, ReadingNone, d.Reading)
	}
	t.Run("measured below the allowance", func(t *testing.T) {
		t.Parallel()
		doc, arm, block, swing := farCornerFixture(t, 2e9, 0.008)
		volume, outcome, allowance := farCornerOverlap(t, doc, arm, block, swing)
		require.Equal(t, interferenceMeasured, outcome)
		lower := volume.Value.Base() - volume.Bound.Base()
		require.Greater(t, lower, 0.0, `the float pose's overlap is proven positive`)
		require.Less(t, lower, allowance, `but its lower end does not clear the allowance`)
		requireUndecided(t, doc, arm, block, swing)
	})
	t.Run("unmeasured", func(t *testing.T) {
		t.Parallel()
		doc := New()
		arm := internalBoxBody(t, doc, 0, -14, 48, 14, 10)
		// A revolved disc of radius 8 about (25, 40): G1 keeps a revolve off
		// the analytic paths, so the shared cap planes reach the mesh path's
		// coplanar refusal.
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XZ())
		require.NoError(t, err)
		rect := s.CreateRectangle(0, 0, 8, 10)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		disc, err := doc.Revolve(s, s.Profiles()[0], SketchLine{End: Point2{V: 1}}, FullRevolution{})
		require.NoError(t, err)
		move, err := r3.Translation(r3.NewVec(25, 40, 0))
		require.NoError(t, err)
		block, err := disc.Placed(t.Context(), move)
		require.NoError(t, err)
		swing := Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Degrees(45)}
		requireUndecided(t, doc, arm, block, swing)
	})
}

// TestMotionPathAreaUpper checks the area the swept-volume allowance is
// charged at: an exactly orthonormal rest placement has σ = 1, a float
// rotation's σ sits just below 1, and a pose whose linear part departs from
// the ideal one stretches the rest area by (1 + linear/σ)², never less.
//
// Legs seen to fail when deleted: the stretch factor (the stretched area
// equals the rest area).
func TestMotionPathAreaUpper(t *testing.T) {
	t.Parallel()
	require.Equal(t, 1.0, motionbound.BasisSigmaLower(r3.Identity()))
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	sigma := motionbound.BasisSigmaLower(rot)
	require.LessOrEqual(t, sigma, 1.0)
	require.Greater(t, sigma, 1-1e-12)

	require.Equal(t, 100.0, motionbound.PathAreaUpper(100, 0, sigma, 1))
	stretched := motionbound.PathAreaUpper(100, 1e-3, 0.5, 1)
	require.GreaterOrEqual(t, stretched, 100*(1+2e-3)*(1+2e-3))
	require.InDelta(t, 100*(1+2e-3)*(1+2e-3), stretched, 1e-9)
	require.True(t, math.IsInf(motionbound.PathAreaUpper(100, 1e-3, 0, 1), 1), `an unbounded σ refuses`)
}

// TestMotionExceedsResolution checks the resolution floor's comparison: an
// interval exactly one resolution wide stops when both are stated in the
// same terms, and a mixed degree/radian pair compares through π without
// stopping early.
func TestMotionExceedsResolution(t *testing.T) {
	t.Parallel()
	at := func(v units.Value) motionbound.MotionParam { return mustMotionParam(t, v) }
	require.False(t, motionbound.ExceedsResolution(at(units.Degrees(0)), at(units.Degrees(90.0/1024)), at(units.Degrees(90.0/1024))))
	require.True(t, motionbound.ExceedsResolution(at(units.Degrees(0)), at(units.Degrees(90.0/512)), at(units.Degrees(90.0/1024))))
	require.False(t, motionbound.ExceedsResolution(at(units.Millimeters(30)), at(units.Millimeters(0)), at(units.Centimeters(3))))
	require.True(t, motionbound.ExceedsResolution(at(units.Degrees(0)), at(units.Degrees(1)), at(units.Radians(0.017))))
	require.False(t, motionbound.ExceedsResolution(at(units.Degrees(0)), at(units.Degrees(1)), at(units.Radians(0.0175))))
}

// A one-subnormal-millimetre path is valid, but one 1024th of it cannot be
// carried by units.Value. The reported fallback must be accepted as an
// explicit option and name the floor actually used by the check.
func TestMotionDefaultResolutionUnderflowIsReusable(t *testing.T) {
	t.Parallel()
	doc := New()
	mover := internalBoxBody(t, doc, 0, 0, 1, 1, 1)
	motion := Prismatic{
		Dir:  r3.NewVec(1, 0, 0),
		From: units.Millimeters(0),
		To:   units.Millimeters(math.SmallestNonzeroFloat64),
	}
	spec, err := resolveMotion(motion)
	require.NoError(t, err)
	cfg, err := motionoption.Resolve(nil, spec.Domain)
	require.NoError(t, err)
	require.Equal(t, units.Millimeters(math.SmallestNonzeroFloat64), cfg.Resolution)
	reported, ok := motionbound.ExactMotionParam(cfg.Resolution)
	require.True(t, ok)
	require.Zero(t, reported.Base.Cmp(cfg.ResolutionP.Base))

	report, err := doc.VerifyMotion(t.Context(), []*Body{mover}, motion)
	require.NoError(t, err)
	require.Equal(t, cfg.Resolution, report.Request.Resolution)
	replayed, err := doc.VerifyMotion(t.Context(), []*Body{mover}, motion, WithResolution(report.Request.Resolution))
	require.NoError(t, err)
	require.Equal(t, report.Status, replayed.Status)

	// A degree needs a larger subnormal magnitude before conversion to the
	// radian base unit is nonzero. The report must also be reusable there.
	swing := Revolute{
		Axis: r3.NewVec(0, 0, 1),
		From: units.Degrees(0),
		To:   units.Degrees(math.SmallestNonzeroFloat64),
	}
	swingSpec, err := resolveMotion(swing)
	require.NoError(t, err)
	swingCfg, err := motionoption.Resolve(nil, swingSpec.Domain)
	require.NoError(t, err)
	require.Greater(t, swingCfg.Resolution.Mag(), math.SmallestNonzeroFloat64)
	swingReported, ok := motionbound.ExactMotionParam(swingCfg.Resolution)
	require.True(t, ok)
	require.Zero(t, swingReported.Turn.Cmp(swingCfg.ResolutionP.Turn))
	encoded, err := decodeMotionOptions([]MotionOption{WithResolution(swingCfg.Resolution)})
	require.NoError(t, err)
	_, err = motionoption.Resolve(encoded, swingSpec.Domain)
	require.NoError(t, err)

	// The nominal 1/1024 step can round to a positive degree magnitude that
	// still converts to zero radians. It must trigger the same fallback.
	swing.To = units.Degrees(1024 * math.SmallestNonzeroFloat64)
	swingSpec, err = resolveMotion(swing)
	require.NoError(t, err)
	swingCfg, err = motionoption.Resolve(nil, swingSpec.Domain)
	require.NoError(t, err)
	require.Greater(t, swingCfg.Resolution.Mag(), math.SmallestNonzeroFloat64)
	swingReported, ok = motionbound.ExactMotionParam(swingCfg.Resolution)
	require.True(t, ok)
	require.Zero(t, swingReported.Turn.Cmp(swingCfg.ResolutionP.Turn))
	swingReport, err := doc.VerifyMotion(t.Context(), []*Body{mover}, swing)
	require.NoError(t, err)
	require.Equal(t, swingCfg.Resolution, swingReport.Request.Resolution)
	_, err = doc.VerifyMotion(t.Context(), []*Body{mover}, swing, WithResolution(swingReport.Request.Resolution))
	require.NoError(t, err)
}

// TestMotionPathAreaUpperStretchBase pins motionbound.PathAreaUpper's stretch base
// (docs/motion-check-design.md §5.1): the base 1 a Revolute and a Prismatic
// pass reproduces their allowance exactly, and a base above 1 — a Between
// whose From is not exactly orthonormal — scales both the unscaled shortcut,
// by base², and the stretched allowance, whose stretch becomes
// base + linear/σ.
//
// Legs seen to fail when deleted: the base in the stretch (the stretched
// allowance with base 1.5 falls to the base-1 value) and the base in the
// unscaled shortcut (the exact-linear area with base 1.5 falls to the rest
// area).
func TestMotionPathAreaUpperStretchBase(t *testing.T) {
	t.Parallel()
	stretch := proofbound.AbsSumUpper(1, proofbound.DivUpper(1e-3, 0.5))
	require.Equal(t, proofbound.ProductUpper(100, proofbound.ProductUpper(stretch, stretch)), motionbound.PathAreaUpper(100, 1e-3, 0.5, 1),
		`the base 1 is the Revolute and Prismatic allowance unchanged`)
	require.Equal(t, 100.0, motionbound.PathAreaUpper(100, 0, 0.5, 1))

	unscaled := motionbound.PathAreaUpper(100, 0, 0.5, 1.5)
	require.GreaterOrEqual(t, unscaled, 100*1.5*1.5)
	require.InDelta(t, 225, unscaled, 1e-9)
	stretched := motionbound.PathAreaUpper(100, 1e-3, 0.5, 1.5)
	require.GreaterOrEqual(t, stretched, 100*(1.5+2e-3)*(1.5+2e-3))
	require.InDelta(t, 100*(1.5+2e-3)*(1.5+2e-3), stretched, 1e-9)

	// A real From: a float rotation's basis is orthonormal only to rounding,
	// so its stretch base sits strictly above 1 and still scales the area.
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	base := motionbound.BasisSigmaUpper(rot)
	require.Greater(t, motionbound.PathAreaUpper(100, 0, 1, base), 100.0)
	require.Greater(t, motionbound.PathAreaUpper(100, 1e-3, 0.5, base), motionbound.PathAreaUpper(100, 1e-3, 0.5, 1))
}

// TestMotionBasisSigmaUpper checks the stretch base's own bound: exactly 1
// for an exactly orthonormal basis, and strictly above 1 — while still an
// ulp-scale excess — for a float rotation whose columns are orthonormal only
// to rounding, where motionbound.BasisSigmaLower sits strictly below 1.
func TestMotionBasisSigmaUpper(t *testing.T) {
	t.Parallel()
	require.Equal(t, 1.0, motionbound.BasisSigmaUpper(r3.Identity()))
	quarter, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(90))
	require.NoError(t, err)
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	for _, tr := range []r3.Transform{quarter, rot} {
		require.Less(t, motionbound.BasisSigmaLower(tr), 1.0, `the fixture's columns are not exactly orthonormal`)
		upper := motionbound.BasisSigmaUpper(tr)
		require.Greater(t, upper, 1.0)
		require.Less(t, upper, 1+1e-12)
	}
}

// TestMotionBetweenFrameReachesTo checks the Between frame's ideal end
// T*(1) = S*(1) ∘ From against the stated To: it maps every rest-box corner of
// the mover to within 1e-9·(1 + |t(To)|) of its image under To. The screw arm
// rebuilds To from an axis through the origin, test 19's far pivot from an
// axis 1e6 mm out, and test 15's offset axis from a From and a screw that do
// not commute. This is a test, not an admission gate: it would show a frame
// composed in the wrong order, and admits nothing.
//
// Legs seen to fail when deleted: composing the screw before From instead of
// after it (test 15's offset-axis corners land about 70 mm from To's images;
// the screw arm's identity From and test 19's pivot-coaxial From commute with
// their screws and cannot see the order).
func TestMotionBetweenFrameReachesTo(t *testing.T) {
	t.Parallel()
	must := func(tr r3.Transform, err error) r3.Transform {
		t.Helper()
		require.NoError(t, err)
		return tr
	}
	z := r3.NewVec(0, 0, 1)
	quarter := func(center r3.Vec) r3.Transform { return must(r3.RotationAround(center, z, units.Degrees(90))) }
	offsetFrom := must(r3.Translation(r3.NewVec(50, 0, 0)))
	cases := []struct {
		name string
		m    Between
	}{
		{"the screw arm", Between{From: r3.Identity(), To: must(quarter(r3.Vec{}).Then(must(r3.Translation(r3.NewVec(0, 0, 20)))))}},
		{"test 19's far pivot", Between{From: must(quarter(r3.NewVec(1e6, 3e5, 0)).Inverse()), To: r3.Identity()}},
		{"test 15's offset axis", Between{From: offsetFrom, To: must(offsetFrom.Then(quarter(r3.NewVec(-50, 0, 0))))}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			spec, err := resolveMotion(tc.m)
			require.NoError(t, err)
			end := spec.Frame.At(spec.ToP)
			tol := 1e-9 * (1 + tc.m.To.Translation().Len())
			for _, corner := range []r3.Vec{
				r3.NewVec(0, -14, 0), r3.NewVec(48, -14, 0), r3.NewVec(0, 14, 0), r3.NewVec(48, 14, 0),
				r3.NewVec(0, -14, 10), r3.NewVec(48, -14, 10), r3.NewVec(0, 14, 10), r3.NewVec(48, 14, 10),
			} {
				x, ok := motionbound.RatVecOf(corner)
				require.True(t, ok)
				image := motionbound.IvVecAdd(end.Rot.Apply(motionbound.PointVec(x)), end.Shift)
				want := tc.m.To.Apply(corner)
				for i, w := range []float64{want.X, want.Y, want.Z} {
					lo, _ := image[i].Lo.Float64()
					hi, _ := image[i].Hi.Float64()
					require.InDelta(t, w, lo, tol, `corner %v axis %d`, corner, i)
					require.InDelta(t, w, hi, tol, `corner %v axis %d`, corner, i)
				}
			}
		})
	}
}
