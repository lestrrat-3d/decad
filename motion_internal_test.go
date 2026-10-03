package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// motionRunFor builds VerifyMotion's own run state for one call, so a test
// can drive a single pose through the production path.
func motionRunFor(t *testing.T, doc *Document, moving []*Body, m Motion) *motionRun {
	t.Helper()
	spec, err := resolveMotion(m)
	require.NoError(t, err)
	run := &motionRun{ctx: t.Context(), d: doc, spec: spec, cfg: motionConfig{rel: 1e-3}, cache: &bodyGeomCache{}}
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

		eta := poseDeviation(composed, placement, run.spec.frame.at(pose.param), run.movers[0].r0)
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
			param, ok := exactMotionParam(tc.at)
			require.True(t, ok)
			placement := arm.payload.transform()
			composed, err := placement.Then(pose)
			require.NoError(t, err)
			require.Zero(t, poseDeviation(composed, placement, spec.frame.at(param), math.Inf(1)))
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
		p, ok := exactMotionParam(tc.at)
		require.True(t, ok)
		sin, cos := paramSinCos(p)
		require.Zero(t, sin.lo.Cmp(big.NewRat(tc.sin, 1)), tc.at.String())
		require.Zero(t, sin.hi.Cmp(big.NewRat(tc.sin, 1)), tc.at.String())
		require.Zero(t, cos.lo.Cmp(big.NewRat(tc.cos, 1)), tc.at.String())
		require.Zero(t, cos.hi.Cmp(big.NewRat(tc.cos, 1)), tc.at.String())
	}

	fromDeg, ok := exactMotionParam(units.Degrees(10))
	require.True(t, ok)
	toRad, ok := exactMotionParam(units.Radians(1))
	require.True(t, ok)
	approx := []struct {
		name  string
		p     motionParam
		angle float64
	}{
		{"degrees", fromDeg, 10 * math.Pi / 180},
		{"negative degrees", mustMotionParam(t, units.Degrees(-450.25)), -450.25 * math.Pi / 180},
		{"radians", toRad, 1},
		{"negative radians", mustMotionParam(t, units.Radians(-2.5)), -2.5},
		{"degrees to radians halfway", fromDeg.lerp(toRad, big.NewRat(1, 2)), 5*math.Pi/180 + 0.5},
	}
	tiny := big.NewRat(1, 1)
	tiny.SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 150))
	for _, tc := range approx {
		sin, cos := paramSinCos(tc.p)
		for _, iv := range []struct {
			got  ratInterval
			want float64
		}{{sin, math.Sin(tc.angle)}, {cos, math.Cos(tc.angle)}} {
			width := new(big.Rat).Sub(iv.got.hi, iv.got.lo)
			require.Equal(t, 1, tiny.Cmp(width), `%s: enclosure width`, tc.name)
			mid, _ := new(big.Rat).Add(iv.got.lo, iv.got.hi).Float64()
			require.InDelta(t, iv.want, mid/2, 4e-16, tc.name)
		}
	}
}

func mustMotionParam(t *testing.T, v units.Value) motionParam {
	t.Helper()
	p, ok := exactMotionParam(v)
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
		rho := moverAxisRadius(arm, spec.frame)
		require.GreaterOrEqual(t, rho, tc.want)
		require.InDelta(t, tc.want, rho, 1e-12)
	}
}

// TestVerifyMotionKeepsTheNextProducerIdentity is §9 test 7's producer
// half: a Duplicate after the call receives exactly the identity it would
// have received before it, so no transient placement occupied one.
func TestVerifyMotionKeepsTheNextProducerIdentity(t *testing.T) {
	t.Parallel()
	doc := New()
	cube := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	internalBoxBody(t, doc, 25, 2, 35, 8, 10)
	before := doc.nextProducerID()
	_, err := doc.VerifyMotion(t.Context(), []*Body{cube}, Prismatic{Dir: r3.NewVec(1, 0, 0), From: units.Millimeters(0), To: units.Millimeters(30)})
	require.NoError(t, err)
	require.Equal(t, before, doc.nextProducerID())
	dup, err := cube.Duplicate(t.Context())
	require.NoError(t, err)
	require.Equal(t, before, dup.originProducer())
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
