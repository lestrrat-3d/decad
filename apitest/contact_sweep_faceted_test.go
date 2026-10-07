package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of docs/multibody-dynamics-design.md §13 PR 12. Every input is
// dyadic. The tumbling scene sits 2^20 mm from the origin, so composing a
// rotation onto the far placement rounds each float pose by up to an ULP of
// 2^20 (about 2e-10 mm), far above the gap reading's own sqrt enclosure.
//
// Legs shown to fail (each zeroed in turn, fixture red, then restored):
//   - the pose deviation (pointDeviation's vertex distance bound):
//     TestSweepPairPlanarTumblingWedge's separated samples publish gaps
//     whose enclosures miss the closed-form ideal gap;
//   - the deep-vertex margin: internal/pair/planar/planar_depth_test.go's vertex at
//     depth 2 is accepted at margin 2.
//
// The search, the §4.3 travel certificate and the vertex-span hull reuse the
// rotating source-box run, whose own tests cover them.

// tumbleOffset puts the tumbling scene far from the origin.
const tumbleOffset = 1 << 20

// tumblerBody is a right prism over the triangle (0,0), (8,0), (2,6), 4 mm
// tall. No two of its vertices share an x, so a tumble about Y lowers one
// vertex first.
func tumblerBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return prismBody(t, doc, [][2]float64{{0, 0}, {8, 0}, {2, 6}}, 4)
}

// tumbleScene is a wide floor whose top face is z = 0 and a wedge 10 mm above
// it, falling at 20 mm/s while turning at 2 rad/s about Y through the point
// (4, 2, 12) of its start pose, both moved tumbleOffset along X.
func tumbleScene(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body,
	decad.RigidDriftSegment, decad.RigidDriftSegment) {
	t.Helper()
	floor := boxBodyAtZ(t, doc, -100, -100, 100, 100, -10, 10)
	wedge := tumblerBody(t, doc)
	floorPath := sweepDrift(r3.Vec{}, 1)
	floorPath.From = contactPose(t, r3.Vec{X: tumbleOffset})
	wedgePath := sweepDrift(r3.Vec{Z: -20}, 1)
	wedgePath.From = contactPose(t, r3.Vec{X: tumbleOffset, Z: 10})
	wedgePath.Center = r3.Vec{X: tumbleOffset + 4, Y: 2, Z: 12}
	wedgePath.AngularVelocity.Y = units.RadiansPerSecond(2)
	return floor, wedge, floorPath, wedgePath
}

// tumbleGap is the ideal gap at time u: the lowest wedge vertex's height.
// Relative to the pivot, a vertex at (dx, ·, dz) sits at height
// -dx·sin(2u) + dz·cos(2u) after the turn; the pivot falls from 12 at
// 20 mm/s. The lowest vertex stays above the floor's top face, so the gap is
// its height.
func tumbleGap(u float64) float64 {
	sin, cos := math.Sincos(2 * u)
	gap := math.Inf(1)
	for _, dx := range []float64{-4, 4, -2} {
		for _, dz := range []float64{-2, 2} {
			gap = math.Min(gap, 12-20*u-dx*sin+dz*cos)
		}
	}
	return gap
}

// tumbleImpact solves tumbleGap(u) = 0 by bisection. The gap falls
// monotonically before the first impact, at about 0.41 s.
func tumbleImpact() float64 {
	lo, hi := 0.0, 1.0
	for range 200 {
		mid := lo + (hi-lo)/2
		if mid == lo || mid == hi {
			break
		}
		if tumbleGap(mid) > 0 {
			lo = mid
		} else {
			hi = mid
		}
	}
	return lo
}

func tumbleRequest() decad.SweepRequest {
	req := sweepRequest()
	req.TimeResolution = units.Seconds(math.Ldexp(1, -20))
	req.MaxPoseEvaluations = 512
	return req
}

func TestSweepPairPlanarTumblingWedge(t *testing.T) {
	doc := decad.New()
	floor, wedge, floorPath, wedgePath := tumbleScene(t, doc)
	before := doc.Bodies()
	req := tumbleRequest()
	impact := tumbleImpact()
	// The closed form runs in float64 over values below 20, so it is good
	// to well under this slack; the far placement's pose rounding is above it.
	const slack = 1e-13

	var brackets [2][2]float64
	for order, pair := range [][2]*decad.Body{{floor, wedge}, {wedge, floor}} {
		pathA, pathB := decad.PairPath(floorPath), decad.PairPath(wedgePath)
		if order == 1 {
			pathA, pathB = pathB, pathA
		}
		report, err := doc.SweepPair(t.Context(), pair[0], pair[1], pathA, pathB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepImpactBracket, report.Outcome, "order %d cause=%v", order, report.Cause)
		require.NotNil(t, report.Bracket)
		left := report.Bracket.From.Elapsed.Value.Base()
		right := report.Bracket.To.Elapsed.Value.Base()
		require.LessOrEqual(t, left, impact+slack)
		require.GreaterOrEqual(t, right, impact-slack)
		require.LessOrEqual(t, right-left, req.TimeResolution.Base())
		require.NotNil(t, report.Event)
		require.Equal(t, decad.ContactOverlapping, report.Event.Relation)
		brackets[order] = [2]float64{left, right}

		separated := 0
		for _, sample := range report.Samples {
			if sample.Ideal.Relation != decad.ContactSeparated {
				continue
			}
			separated++
			require.NotNil(t, sample.Ideal.Gap)
			ideal := tumbleGap(sample.At.Elapsed.Value.Base())
			require.InDelta(t, ideal, sample.Ideal.Gap.Value.Base(), sample.Ideal.Gap.Bound.Base()+slack,
				"order %d fraction %v", order, sample.At.Fraction.Base())
			require.Greater(t, sample.Ideal.Gap.Value.Base()-sample.Ideal.Gap.Bound.Base(), 0.0)
		}
		require.Greater(t, separated, 2)
	}
	require.Equal(t, brackets[0], brackets[1])
	require.Equal(t, before, doc.Bodies())
}

func TestSweepPairPlanarAffineWedgeTouchesFloor(t *testing.T) {
	doc := decad.New()
	floor := boxBodyAtZ(t, doc, -100, -100, 100, 100, -10, 10)
	wedge := tumblerBody(t, doc)
	wedgePath := sweepDrift(r3.Vec{Z: -20}, 1)
	wedgePath.From = contactPose(t, r3.Vec{Z: 10})
	req := tumbleRequest()
	report, err := doc.SweepPair(t.Context(), floor, wedge, sweepDrift(r3.Vec{}, 1), wedgePath, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, report.Outcome, "cause=%v", report.Cause)
	// The wedge's lower face reaches the floor at exactly 0.5 s, a dyadic
	// time, so the bracket ends on the exact touch.
	require.Equal(t, .5, report.Bracket.To.Elapsed.Value.Base())
	require.Equal(t, .5-req.TimeResolution.Base(), report.Bracket.From.Elapsed.Value.Base())
	require.False(t, report.BracketEndsAtDuration())
	require.Equal(t, decad.ContactTouching, report.Event.Relation)
	require.NotNil(t, report.Event.Manifold)
	require.Len(t, report.Event.Manifold.Points, 3)
	for _, point := range report.Event.Manifold.Points {
		require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
		require.Zero(t, point.OnA.Value.Z)
		require.Zero(t, point.OnB.Value.Z)
	}
}

func TestSweepPairPlanarWedgeStops(t *testing.T) {
	doc := decad.New()
	floor, wedge, floorPath, wedgePath := tumbleScene(t, doc)
	req := tumbleRequest()
	req.MaxPoseEvaluations = 4
	report, err := doc.SweepPair(t.Context(), floor, wedge, floorPath, wedgePath, req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepUndecided, report.Outcome)
	require.Equal(t, decad.SweepPoseBudget, report.Cause)
	require.Equal(t, uint64(4), report.PoseEvaluations)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	canceled, err := doc.SweepPair(ctx, floor, wedge, floorPath, wedgePath, tumbleRequest())
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, canceled)

	// A rotating planar pair that starts touching has no departure proof yet.
	touchPath := wedgePath
	touchPath.From = contactPose(t, r3.Vec{X: tumbleOffset})
	touchPath.Center.Z = 2
	touchReq := tumbleRequest()
	touchReq.StartPolicy = decad.ContinueSeparatingTouch
	touching, err := doc.SweepPair(t.Context(), floor, wedge, floorPath, touchPath, touchReq)
	require.NoError(t, err)
	require.Equal(t, decad.SweepUndecided, touching.Outcome)
	require.Equal(t, decad.SweepDepartureUnproved, touching.Cause)
	require.NotNil(t, touching.InitialEvent)
	require.Equal(t, decad.ContactTouching, touching.InitialEvent.Relation)
}

// TestSweepPairPlanarTumblingTetrahedronReplaysImpact drops
// mass_properties_mesh_test.go's stitched tetrahedron into the tray while it
// turns, the tumble scene's first impact in minimal form
// (docs/multibody-dynamics-design.md §10.1). The tetrahedron starts 10 mm
// above the tray floor, z = 0, falling at 20 mm/s and turning at 2 rad/s
// about Y through (3, 1, 11). Its travel bound T is above 20 mm per unit
// fraction, so a bracket TimeResolution wide carries more than 2e-5 mm of
// travel from its left edge, far above the left gap plus PointResolution
// (1e-6 mm). A step cuts the impact at the bracket's right edge, which
// replays only once the sweep has narrowed the bracket below that width.
//
// Legs shown to fail (each removed in turn, fixture red, then restored):
//   - the narrowing itself (narrowBracket returning its input): the right
//     edge's replay refuses with "rounded planar impact pose leaves the point
//     resolution of contact";
//   - the travel term of bracketRightReplays (T zeroed): narrowing stops at
//     once, with the same refusal;
//   - the pose-budget fallback (narrowBracket passing the budget error on):
//     the shorter budget below fails the sweep with that error.
//
// The gap and deviation terms only stop narrowing earlier when they grow;
// zeroing either narrows further, which replay still accepts, so neither can
// turn this fixture red. Replay's own check (bracketDepthWithin) stays the
// only admission, and TestRotatingBracketDepthChargesTravelAndDeviation
// covers each of its legs.
func TestSweepPairPlanarTumblingTetrahedronReplaysImpact(t *testing.T) {
	doc := decad.New()
	tray := trayBody(t, doc)
	tetrahedron, _ := stitchedTetrahedron(t, doc)
	trayPath := sweepDrift(r3.Vec{}, 1)
	tetPath := sweepDrift(r3.Vec{Z: -20}, 1)
	tetPath.From = contactPose(t, r3.Vec{Z: 10})
	tetPath.Center = r3.Vec{X: 3, Y: 1, Z: 11}
	tetPath.AngularVelocity.Y = units.RadiansPerSecond(2)
	req := tumbleRequest()
	resolution := req.PointResolution.Base()

	var brackets [2][2]float64
	for order, pair := range [][2]*decad.Body{{tray, tetrahedron}, {tetrahedron, tray}} {
		pathA, pathB := decad.PairPath(trayPath), decad.PairPath(tetPath)
		if order == 1 {
			pathA, pathB = pathB, pathA
		}
		report, err := doc.SweepPair(t.Context(), pair[0], pair[1], pathA, pathB, req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepImpactBracket, report.Outcome, "order %d cause=%v", order, report.Cause)
		require.NotNil(t, report.Bracket)
		left := report.Bracket.From.Fraction.Base()
		right := report.Bracket.To.Fraction.Base()
		require.Less(t, left, right)
		brackets[order] = [2]float64{left, right}

		// Narrowing samples last, so one evaluation fewer drops only its
		// final midpoint: the wider bracket stands, never an undecided one.
		short := req
		short.MaxPoseEvaluations = report.PoseEvaluations - 1
		wider, err := doc.SweepPair(t.Context(), pair[0], pair[1], pathA, pathB, short)
		require.NoError(t, err)
		require.Equal(t, decad.SweepImpactBracket, wider.Outcome, "order %d cause=%v", order, wider.Cause)
		require.Greater(t, wider.Bracket.To.Fraction.Base()-wider.Bracket.From.Fraction.Base(), right-left)

		// The step's cut: the right edge, replayed the way dynamics replays
		// a slice fraction.
		poseA, poseB, err := report.CertifiedPosesAtInterval(units.Seconds(right), units.Seconds(0), units.Seconds(1))
		require.NoError(t, err, "order %d", order)
		pose := poseB
		if order == 1 {
			pose = poseA
		}
		lowest := math.Inf(1)
		for _, vertex := range tetrahedron.Vertices() {
			lowest = math.Min(lowest, pose.Apply(vertex.Position().Value).Z)
		}
		// The replay places the pair within PointResolution of a separated
		// one, and the right edge meets the floor. Applying the float pose to
		// coordinates below 16 rounds far below this slack.
		const slack = 1e-12
		require.GreaterOrEqual(t, lowest, -resolution-slack, "order %d", order)
		require.LessOrEqual(t, lowest, slack, "order %d", order)
		require.Less(t, right-left, req.TimeResolution.Base(), "order %d: the bracket narrows below TimeResolution", order)
	}
	require.Equal(t, brackets[0], brackets[1])
}
