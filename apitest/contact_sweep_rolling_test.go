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

// The root fixtures of docs/multibody-dynamics-design.md §13 PR 20: a full
// source cylinder lying on a floor under a constant-ω drift keeps §10.4's
// rolling band track. Every length is dyadic; the cylinder's
// axis sits at y = rollingAxisY, so a rotation about it rounds the heights of
// its end centers and every replay deviation is nonzero.
//
// Each expected depth is recomputed here in float64 from the closed form
// |H'(0)|·h + (K + r·|ω×â|²)·h², and every published point is checked
// against the true lowest rim point of the ideal cylinder, computed here by
// Rodrigues' formula.
//
// Legs shown to fail (each deleted or zeroed in turn, fixture red, then
// restored):
//   - the contact rate |H'(0)|: TestSweepPairRollingCylinder's "closing" case
//     publishes a zero depth while its rims sink;
//   - the curvature K: the "orbit" case publishes a zero depth while its end
//     centers rise off the floor;
//   - the tilt term r·|ω×â|²: the "tilt" case's rising rim climbs above the
//     published depth;
//   - the lateral drift in the published balls: the "tilt" case's true rim
//     points fall outside their balls;
//   - the lateral drift in the foot check: TestSweepPairRollingLeavesFace's
//     track outlasts the rim's true reach toward the floor's edge;
//   - the foot check itself: TestSweepPairRollingLeavesFace reaches the
//     duration;
//   - the replay's resolution gate: TestSweepPairRollingReplayResolution
//     replays an interior pose whose deviation exceeds the request;
//   - the replay's deviation charge on the end heights: the "roll" case, whose
//     depth is exactly zero, refuses its own interior replays.

const rollingAxisY = 3.375

// rollingScene is a radius-10 source cylinder about the line y =
// rollingAxisY, z = 0, over x ∈ [−half, half], on a floor whose top face is
// z = −10 over x ∈ [x0, 100], y ∈ [−200, 200].
func rollingScene(t *testing.T, doc *decad.Document, half, x0 float64) (*decad.Body, *decad.Body) {
	t.Helper()
	cylinder := revolvedCylinder(t, doc, -half, half, rollingAxisY, 10)
	floor := boxBodyAtZ(t, doc, x0, -200, 100, 200, -20, 10)
	return floor, cylinder
}

// rollingCase is one drift of the cylinder: angular velocity omega about
// center, linear velocity v, over seconds.
type rollingCase struct {
	omega, v, center r3.Vec
	seconds          float64
	half             float64 // the cylinder's half length
}

func (c rollingCase) path() decad.RigidDriftSegment {
	path := sweepDrift(c.v, c.seconds)
	path.Center = c.center
	path.AngularVelocity = decad.QuantityVec{X: units.RadiansPerSecond(c.omega.X),
		Y: units.RadiansPerSecond(c.omega.Y), Z: units.RadiansPerSecond(c.omega.Z)}
	return path
}

// rotate turns d about omega by |omega|·t (Rodrigues).
func rotate(d, omega r3.Vec, t float64) r3.Vec {
	speed := omega.Len()
	if speed == 0 {
		return d
	}
	k := omega.Scale(1 / speed)
	angle := speed * t
	return d.Scale(math.Cos(angle)).Add(k.Cross(d).Scale(math.Sin(angle))).
		Add(k.Scale(k.Dot(d) * (1 - math.Cos(angle))))
}

// ends are the ideal end centers at t, in published order (−x first).
func (c rollingCase) ends(t float64) [2]r3.Vec {
	var out [2]r3.Vec
	for i, x := range []float64{-c.half, c.half} {
		start := r3.Vec{X: x, Y: rollingAxisY}
		out[i] = rotate(start.Sub(c.center), c.omega, t).Add(c.center).Add(c.v.Scale(t))
	}
	return out
}

// rims are the ideal lowest points of the two end disks at t: c − r·w, w the
// unit direction of Z less its component along the turned axis.
func (c rollingCase) rims(t float64) [2]r3.Vec {
	axis := rotate(r3.Vec{X: 1}, c.omega, t)
	w := r3.Vec{Z: 1}.Sub(axis.Scale(axis.Z))
	w = w.Scale(1 / w.Len())
	ends := c.ends(t)
	return [2]r3.Vec{ends[0].Sub(w.Scale(10)), ends[1].Sub(w.Scale(10))}
}

// depth is the closed form |H'(0)|·h + (K + r·|ω×â|²)·h² over both ends.
func (c rollingCase) depth(h float64) float64 {
	rate, k := 0.0, 0.0
	for _, x := range []float64{-c.half, c.half} {
		lever := c.omega.Cross(r3.Vec{X: x, Y: rollingAxisY}.Sub(c.center))
		rate = math.Max(rate, math.Abs(c.v.Add(lever).Z))
		k = math.Max(k, c.omega.Len()*lever.Len()/2)
	}
	tilt := c.omega.Cross(r3.Vec{X: 1}).Len()
	return rate*h + (k+10*tilt*tilt)*h*h
}

func TestSweepPairRollingCylinder(t *testing.T) {
	// The closed forms and Rodrigues' formula run in float64 over values
	// below 100 mm.
	const slack = 1e-9
	omega := 2 * math.Pi
	axis := r3.Vec{Y: rollingAxisY}
	roll := r3.Vec{Y: -omega * 10}
	for _, tc := range []struct {
		name string
		rollingCase
		resolution float64 // mm; zero keeps bandRequest's
	}{
		// One turn at 2π rad/s rolls π·20 mm along −y with the contact point
		// at rest: every height is exact and the track is an exact touch.
		{name: "roll", rollingCase: rollingCase{omega: r3.Vec{X: omega}, v: roll, center: axis,
			seconds: 1, half: 15}},
		// A quarter-millimetre-per-second sink adds its rate: |c|·h.
		{name: "closing", rollingCase: rollingCase{omega: r3.Vec{X: omega},
			v: r3.Vec{Y: roll.Y, Z: -.25}, center: axis, seconds: 1, half: 15}},
		// A pivot an eighth of a millimetre above the axis swings the end
		// centers on a small circle: K·h².
		{name: "orbit", rollingCase: rollingCase{omega: r3.Vec{X: omega}, v: roll,
			center: r3.Vec{Y: rollingAxisY, Z: .125}, seconds: 1, half: 15}},
		// A 4 mm disc tipping about Y lifts one rim by its radius's term
		// r·|ω×â|²·h², more than its rate and K allow alone. Its rim points
		// drift up to (3/2)·r·|ω×â|·h, about 1.9 mm, off the end centers'
		// feet, which a request must admit for interior manifolds.
		{name: "tilt", rollingCase: rollingCase{omega: r3.Vec{Y: .5}, center: axis, seconds: .25, half: 2},
			resolution: 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			floor, cylinder := rollingScene(t, doc, tc.half, -100)
			wall := surfaceFace[decad.Cylinder](t, cylinder, func(*decad.Face) bool { return true })
			top := surfaceFace[decad.Plane](t, floor, func(face *decad.Face) bool {
				reading, err := face.NormalAt(r3.Vec{Z: -10})
				return err == nil && reading.Value == r3.Vec{Z: 1}
			})
			floorPath, cylinderPath := sweepDrift(r3.Vec{}, tc.seconds), tc.path()
			req := bandRequest()
			if tc.resolution > 0 {
				req.PointResolution = units.Millimeters(tc.resolution)
			}
			before := doc.Bodies()
			for order := range 2 {
				a, b := floor, cylinder
				pathA, pathB := decad.PairPath(floorPath), decad.PairPath(cylinderPath)
				if order == 1 {
					a, b, pathA, pathB = cylinder, floor, pathB, pathA
				}
				report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, req)
				require.NoError(t, err)
				want := decad.SweepPersistentBand
				if tc.name == "roll" {
					want = decad.SweepPersistentTouch
				}
				require.Equal(t, want, report.Outcome, "order %d cause=%v", order, report.Cause)
				require.Equal(t, decad.ContactTouching, report.InitialEvent.Relation)
				track := report.ContactTrack
				require.NotNil(t, track)
				require.Equal(t, 1.0, track.End().Fraction.Base())
				h := track.End().Elapsed.Value.Base()
				depth := 0.0
				prefix, err := track.BandAt(units.Scalar(.5))
				require.NoError(t, err)
				if band := track.Band(); tc.name == "roll" {
					require.Nil(t, band)
					require.Nil(t, prefix)
				} else {
					require.NotNil(t, band)
					require.Equal(t, units.Length, band.Value.Kind())
					require.InDelta(t, tc.depth(h), band.Value.Base(), band.Bound.Base()+slack)
					depth = band.Value.Base() + band.Bound.Base()
					// The prefix through half the track reads the same closed form
					// at half its time.
					require.NotNil(t, prefix)
					require.InDelta(t, tc.depth(h/2), prefix.Value.Base(), prefix.Bound.Base()+slack)
					whole, err := track.BandAt(units.Scalar(1))
					require.NoError(t, err)
					require.Equal(t, *band, *whole)
				}
				normal := 1.0
				if order == 1 {
					normal = -1
				}
				require.Equal(t, r3.Vec{Z: normal}, track.Normal().Value)
				featureA, featureB := track.Features()
				onFloorFeature, onCylinderFeature := featureA, featureB
				if order == 1 {
					onFloorFeature, onCylinderFeature = featureB, featureA
				}
				require.Same(t, top, onFloorFeature.Face)
				require.Same(t, wall, onCylinderFeature.Face)

				for _, fraction := range []float64{0, .25, .5, .75, 1} {
					elapsed := fraction * tc.seconds
					manifold, err := track.ManifoldAt(units.Scalar(fraction))
					require.NoError(t, err, "fraction %v", fraction)
					require.Len(t, manifold.Points, 2)
					rims := tc.rims(elapsed)
					for i, point := range manifold.Points {
						onFloor, onCylinder := point.OnA, point.OnB
						if order == 1 {
							onFloor, onCylinder = point.OnB, point.OnA
						}
						rim := rims[i]
						// The published rim end holds the true lowest rim point,
						// and the floor witness its foot.
						require.InDelta(t, 0, onCylinder.Value.Sub(rim).Len(), onCylinder.Bound.Base()+slack,
							"fraction %v end %d", fraction, i)
						foot := r3.Vec{X: rim.X, Y: rim.Y, Z: -10}
						require.InDelta(t, 0, onFloor.Value.Sub(foot).Len(), onFloor.Bound.Base()+slack)
						// The true rim height lies inside the band.
						require.LessOrEqual(t, math.Abs(rim.Z+10), depth+slack, "fraction %v end %d", fraction, i)
						require.GreaterOrEqual(t, point.Separation.Bound.Base(), depth-slack)
						require.Equal(t, r3.Vec{Z: normal}, point.Normal.Value)
						if tc.name != "roll" {
							continue
						}
						// Rolling without slip: the material velocity at the
						// published contact point is zero within |ω| times its
						// ball, the drift's float speed rounding aside.
						pivot := tc.center.Add(tc.v.Scale(elapsed))
						speed := tc.v.Add(tc.omega.Cross(onCylinder.Value.Sub(pivot))).Len()
						require.LessOrEqual(t, speed, omega*onCylinder.Bound.Base()+slack, "fraction %v", fraction)
					}
				}
				for _, fraction := range []float64{.125, .3, .5, 1} {
					elapsed := fraction * tc.seconds
					poseA, poseB, err := report.CertifiedPosesAt(units.Seconds(elapsed))
					require.NoError(t, err, "order %d fraction %v", order, fraction)
					pose := poseB
					if order == 1 {
						pose = poseA
					}
					ends := tc.ends(elapsed)
					for i, x := range []float64{-tc.half, tc.half} {
						at := pose.Apply(r3.Vec{X: x, Y: rollingAxisY})
						require.InDelta(t, 0, at.Sub(ends[i]).Len(), 1e-6)
					}
				}
			}
			require.Equal(t, before, doc.Bodies())
		})
	}
}

func TestSweepPairRollingTravel(t *testing.T) {
	// One turn rolls the cylinder its circumference, π·20 mm, along −y.
	doc := decad.New()
	floor, cylinder := rollingScene(t, doc, 15, -100)
	omega := 2 * math.Pi
	c := rollingCase{omega: r3.Vec{X: omega}, v: r3.Vec{Y: -omega * 10},
		center: r3.Vec{Y: rollingAxisY}, seconds: 1, half: 15}
	report, err := doc.SweepPair(t.Context(), floor, cylinder, sweepDrift(r3.Vec{}, 1), c.path(), bandRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, report.Outcome, "cause=%v", report.Cause)
	start, err := report.ContactTrack.ManifoldAt(units.Scalar(0))
	require.NoError(t, err)
	end, err := report.ContactTrack.ManifoldAt(units.Scalar(1))
	require.NoError(t, err)
	for i := range start.Points {
		travel := start.Points[i].OnB.Value.Sub(end.Points[i].OnB.Value)
		bound := start.Points[i].OnB.Bound.Base() + end.Points[i].OnB.Bound.Base()
		require.InDelta(t, math.Pi*20, travel.Y, bound+1e-12)
		require.InDelta(t, 0, travel.X, bound)
		require.InDelta(t, 0, travel.Z, bound)
	}
}

func TestSweepPairRollingLeavesFace(t *testing.T) {
	// The tipping disc of the "tilt" case, over half a second, on a floor
	// whose x = −4 rim lies 2 mm past the disc's x = −2 face. The foot box
	// grows by the depth t + 2.75·t² and the lateral drift 7.5·t, so it
	// reaches the rim at the root of 2.75·t² + 8.5·t = 2, before the half
	// second. The end centers' interval boxes only add to the growth.
	doc := decad.New()
	floor, cylinder := rollingScene(t, doc, 2, -4)
	c := rollingCase{omega: r3.Vec{Y: .5}, center: r3.Vec{Y: rollingAxisY}, seconds: .5, half: 2}
	root := (-8.5 + math.Sqrt(8.5*8.5+4*2.75*2)) / (2 * 2.75)
	report, err := doc.SweepPair(t.Context(), floor, cylinder, sweepDrift(r3.Vec{}, c.seconds), c.path(), bandRequest())
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentBand, report.Outcome, "cause=%v", report.Cause)
	end := report.ContactTrack.End().Elapsed.Value.Base()
	require.LessOrEqual(t, end, root)
	require.Greater(t, end, root-.01)
	// The true lowest rim point stays on the floor through the track.
	for _, rim := range c.rims(end) {
		require.Greater(t, rim.X, -4.0)
	}
	_, _, err = report.CertifiedPosesAt(units.Seconds(c.seconds))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestSweepPairRollingReplayResolution(t *testing.T) {
	// The start is exact, so its manifold publishes at any resolution; an
	// interior pose off the dyadic grid deviates from the ideal one by a few
	// ULPs, which a 2^-60 mm request refuses.
	doc := decad.New()
	floor, cylinder := rollingScene(t, doc, 15, -100)
	omega := 2 * math.Pi
	c := rollingCase{omega: r3.Vec{X: omega}, v: r3.Vec{Y: -omega * 10},
		center: r3.Vec{Y: rollingAxisY}, seconds: 1, half: 15}
	req := bandRequest()
	req.PointResolution = units.Millimeters(math.Ldexp(1, -60))
	report, err := doc.SweepPair(t.Context(), floor, cylinder, sweepDrift(r3.Vec{}, 1), c.path(), req)
	require.NoError(t, err)
	require.Equal(t, decad.SweepPersistentTouch, report.Outcome, "cause=%v", report.Cause)
	_, _, err = report.CertifiedPosesAt(units.Seconds(0))
	require.NoError(t, err)
	_, _, err = report.CertifiedPosesAt(units.Seconds(.3))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = report.ContactTrack.ManifoldAt(units.Scalar(.3))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestSweepPairRollingRefusals(t *testing.T) {
	omega := 2 * math.Pi
	roll := rollingCase{omega: r3.Vec{X: omega}, v: r3.Vec{Y: -omega * 10},
		center: r3.Vec{Y: rollingAxisY}, seconds: 1, half: 15}

	t.Run("start policies", func(t *testing.T) {
		doc := decad.New()
		floor, cylinder := rollingScene(t, doc, 15, -100)
		req := bandRequest()
		req.StartPolicy = decad.StopAtInitialContact
		report, err := doc.SweepPair(t.Context(), floor, cylinder, sweepDrift(r3.Vec{}, 1), roll.path(), req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepInitiallyTouching, report.Outcome)
		require.NotNil(t, report.Event.Manifold)
		require.Len(t, report.Event.Manifold.Points, 2)

		req.StartPolicy = decad.ContinueSeparatingTouch
		report, err = doc.SweepPair(t.Context(), floor, cylinder, sweepDrift(r3.Vec{}, 1), roll.path(), req)
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, report.Outcome)
		require.Equal(t, decad.SweepDepartureUnproved, report.Cause)
	})
	t.Run("separated start", func(t *testing.T) {
		doc := decad.New()
		cylinder := revolvedCylinder(t, doc, -15, 15, rollingAxisY, 10)
		floor := boxBodyAtZ(t, doc, -100, -200, 100, 200, -21, 10)
		report, err := doc.SweepPair(t.Context(), floor, cylinder, sweepDrift(r3.Vec{}, 1), roll.path(), bandRequest())
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, report.Outcome)
		require.Equal(t, decad.SweepMissingBound, report.Cause)
		require.Nil(t, report.ContactTrack)
	})
	t.Run("sunk placed start", func(t *testing.T) {
		// A long-rolled start a micrometre into the floor is beyond its band,
		// so ContactPair proves no start relation.
		doc := decad.New()
		cylinder := revolvedCylinder(t, doc, -15, 15, 0, 10)
		floor := boxBodyAtZ(t, doc, -100, -200, 100, 200, -20, 10)
		path := placedRoll(t, rolledPose(t, 1<<16, .7))
		var err error
		path.From, err = path.From.Then(contactPose(t, r3.Vec{Z: -1e-6}))
		require.NoError(t, err)
		report, err := doc.SweepPair(t.Context(), floor, cylinder, sweepDrift(r3.Vec{}, 1), path, bandRequest())
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, report.Outcome)
		require.Equal(t, decad.SweepPoseRelation, report.Cause)
		require.Nil(t, report.ContactTrack)
	})
	t.Run("standing cylinder", func(t *testing.T) {
		// A cylinder spinning on its end disk has no ruling on the floor.
		doc := decad.New()
		floor := boxBodyAtZ(t, doc, -100, -100, 100, 100, -15, 10)
		cylinder := verticalCylinder(t, doc, 0, 0, 8)
		spin := rollingCase{omega: r3.Vec{Z: 1}, seconds: 1}
		report, err := doc.SweepPair(t.Context(), floor, cylinder, sweepDrift(r3.Vec{}, 1), spin.path(), bandRequest())
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, report.Outcome)
		require.Nil(t, report.ContactTrack)
	})
	t.Run("canceled", func(t *testing.T) {
		doc := decad.New()
		floor, cylinder := rollingScene(t, doc, 15, -100)
		canceled, cancel := context.WithCancel(t.Context())
		cancel()
		report, err := doc.SweepPair(canceled, floor, cylinder, sweepDrift(r3.Vec{}, 1), roll.path(), bandRequest())
		require.ErrorIs(t, err, context.Canceled)
		require.Nil(t, report)
	})
}

// placedRoll rolls a radius-10 cylinder whose axis is the world X axis at its
// identity pose one turn at 2π rad/s from the given start pose, about the
// pose's image of the origin.
func placedRoll(t *testing.T, from r3.Transform) decad.RigidDriftSegment {
	t.Helper()
	omega := 2 * math.Pi
	path := rollingCase{omega: r3.Vec{X: omega}, v: r3.Vec{Y: -omega * 10}, center: from.Translation(),
		seconds: 1}.path()
	path.From = from
	return path
}

// The placed starts of docs/multibody-dynamics-design.md §10.4 ("Rolling"):
// a start pose other than the identity. A signed-axis start is an exact
// touch, as at the identity. Any other start is ContactPair's band of the
// pose basis's rounding (contact_analytic_manifold_test.go), and the track
// charges it: its depth starts at max|H±| + r·gram + r·α², and its rim balls
// at r·(3·gram + (3/2)·|α|), α the staged axis column's normal component.
//
// Legs shown to fail (each deleted or zeroed in turn, fixture red, then
// restored):
//   - the start heights max|H±| in the depth: "tilted" publishes a band below
//     its rising end at the start;
//   - the section term r·gram + r·α² in the depth: "long roll" publishes a
//     start band that misses the true least height;
//   - the rim balls' start drift: "long roll" and "tilted" publish start balls
//     that miss the true lowest rim points;
//   - the |α| + β·t <= 1/4 gate on the rim drift: "gate" outlasts it.
//
// The rate's cross term 2·r·β·|α| cannot be shown failing: the section term
// reads 1 − √(1 − x) <= x, about twice the true r·(1 − |y|) for small x, and
// within the 1/4 gate r·α² + r·β²·t² covers the true term to within 2% of
// r·α², which the other terms' slack absorbs in every fixture. It stays as
// the bound the derivation needs.
func TestSweepPairRollingPlacedStart(t *testing.T) {
	const slack = 1e-9
	scene := func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
		doc := decad.New()
		cylinder := revolvedCylinder(t, doc, -15, 15, 0, 10)
		floor := boxBodyAtZ(t, doc, -100, -200, 100, 200, -20, 10)
		return doc, floor, cylinder
	}
	sweep := func(t *testing.T, doc *decad.Document, floor, cylinder *decad.Body, path decad.RigidDriftSegment,
		req decad.SweepRequest, order int) *decad.SweepReport {
		t.Helper()
		a, b := floor, cylinder
		pathA, pathB := decad.PairPath(sweepDrift(r3.Vec{}, pathDurationOf(path))), decad.PairPath(path)
		if order == 1 {
			a, b, pathA, pathB = cylinder, floor, pathB, pathA
		}
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, req)
		require.NoError(t, err)
		return report
	}
	// requireStart checks the track's start against the true occupied set of
	// the start pose: the band holds both ends' least heights, and each rim
	// ball its true lowest point.
	requireStart := func(t *testing.T, track *decad.SweepContactTrack, from r3.Transform, order int) {
		t.Helper()
		band, err := track.BandAt(units.Scalar(0))
		require.NoError(t, err)
		require.NotNil(t, band)
		manifold, err := track.ManifoldAt(units.Scalar(0))
		require.NoError(t, err)
		require.Len(t, manifold.Points, 2)
		for i, x := range []float64{-15, 15} {
			height, rim := trueRim(from, x, 0)
			requireWithin(t, height, band.Value.Base()+band.Bound.Base(), "start band")
			onCylinder := manifold.Points[i].OnB
			if order == 1 {
				onCylinder = manifold.Points[i].OnA
			}
			requireBallHolds(t, onCylinder, rim, "start rim")
		}
	}

	for order := range 2 {
		t.Run(map[int]string{0: "floor first", 1: "cylinder first"}[order], func(t *testing.T) {
			t.Run("translated", func(t *testing.T) {
				// A signed-axis start is the exact touch of the identity case:
				// one turn rolls the contact point π·20 mm along −y.
				doc, floor, cylinder := scene(t)
				path := placedRoll(t, contactPose(t, r3.Vec{X: 1, Y: 2}))
				report := sweep(t, doc, floor, cylinder, path, bandRequest(), order)
				require.Equal(t, decad.SweepPersistentTouch, report.Outcome, "cause=%v", report.Cause)
				require.Equal(t, decad.ContactTouching, report.InitialEvent.Relation)
				start, err := report.ContactTrack.ManifoldAt(units.Scalar(0))
				require.NoError(t, err)
				end, err := report.ContactTrack.ManifoldAt(units.Scalar(1))
				require.NoError(t, err)
				for i := range start.Points {
					from, to := start.Points[i].OnB, end.Points[i].OnB
					if order == 1 {
						from, to = start.Points[i].OnA, end.Points[i].OnA
					}
					require.Equal(t, r3.Vec{X: float64(2*i-1)*15 + 1, Y: 2, Z: -10}, from.Value)
					require.InDelta(t, math.Pi*20, from.Value.Y-to.Value.Y, from.Bound.Base()+to.Bound.Base()+1e-12)
				}
			})
			t.Run("long roll", func(t *testing.T) {
				// The start pose of 65536 turns of 0.7 rad: ContactPair's band
				// of the basis's drift starts the track, which rolls one more
				// turn within it.
				doc, floor, cylinder := scene(t)
				from := rolledPose(t, 1<<16, .7)
				report := sweep(t, doc, floor, cylinder, placedRoll(t, from), bandRequest(), order)
				require.Equal(t, decad.SweepPersistentBand, report.Outcome, "cause=%v", report.Cause)
				require.Equal(t, decad.ContactBand, report.InitialEvent.Relation)
				track := report.ContactTrack
				require.Equal(t, 1.0, track.End().Fraction.Base())
				requireStart(t, track, from, order)
				// The band is the basis's drift: negligible against the request.
				band := track.Band()
				require.NotNil(t, band)
				require.Less(t, band.Value.Base()+band.Bound.Base(), 1e-9)
				for _, fraction := range []float64{.25, .5, 1} {
					_, _, err := report.CertifiedPosesAt(units.Seconds(fraction))
					require.NoError(t, err, "fraction %v", fraction)
					manifold, err := track.ManifoldAt(units.Scalar(fraction))
					require.NoError(t, err)
					for i, point := range manifold.Points {
						onCylinder := point.OnB
						if order == 1 {
							onCylinder = point.OnA
						}
						// The rim rolls with the axis: it stays at y = −20π·t,
						// z = −10, within the ball and the slack of this float
						// closed form.
						want := r3.Vec{X: float64(2*i-1) * 15, Y: -math.Pi * 20 * fraction, Z: -10}
						require.InDelta(t, 0, onCylinder.Value.Sub(want).Len(), onCylinder.Bound.Base()+slack)
					}
				}
			})
			t.Run("tilted", func(t *testing.T) {
				// The "tilt" drift of TestSweepPairRollingCylinder from a start
				// tipped 2^-10 rad about Y: the −X end starts about 0.029 mm
				// up, and the tip about Y raises the tilt as the disc turns.
				doc, floor, cylinder := scene(t)
				from := tiltedPose(t, math.Ldexp(1, -10), 0)
				path := rollingCase{omega: r3.Vec{Y: .5}, center: from.Translation(), seconds: .25}.path()
				path.From = from
				req := bandRequest()
				req.PointResolution = units.Millimeters(2)
				report := sweep(t, doc, floor, cylinder, path, req, order)
				require.Equal(t, decad.SweepPersistentBand, report.Outcome, "cause=%v", report.Cause)
				require.Equal(t, decad.ContactBand, report.InitialEvent.Relation)
				track := report.ContactTrack
				require.Equal(t, 1.0, track.End().Fraction.Base())
				requireStart(t, track, from, order)
				axis0 := from.ApplyDir(r3.Vec{X: 1})
				for _, fraction := range []float64{.25, .5, .75, 1} {
					elapsed := fraction * .25
					band, err := track.BandAt(units.Scalar(fraction))
					require.NoError(t, err)
					manifold, err := track.ManifoldAt(units.Scalar(fraction))
					require.NoError(t, err)
					axis := rotate(axis0, r3.Vec{Y: .5}, elapsed)
					w := r3.Vec{Z: 1}.Sub(axis.Scale(axis.Z))
					w = w.Scale(1 / w.Len())
					for i, x := range []float64{-15, 15} {
						end := rotate(from.Apply(r3.Vec{X: x}).Sub(from.Translation()), r3.Vec{Y: .5}, elapsed).
							Add(from.Translation())
						rim := end.Sub(w.Scale(10))
						require.LessOrEqual(t, math.Abs(rim.Z+10), band.Value.Base()+band.Bound.Base()+slack,
							"fraction %v end %d", fraction, i)
						onCylinder := manifold.Points[i].OnB
						if order == 1 {
							onCylinder = manifold.Points[i].OnA
						}
						require.InDelta(t, 0, onCylinder.Value.Sub(rim).Len(), onCylinder.Bound.Base()+slack,
							"fraction %v end %d", fraction, i)
					}
				}
			})
			t.Run("gate", func(t *testing.T) {
				// Over a whole second the tip would raise |α| past 1/4, where
				// the rim drift bound stops holding: the track ends where
				// |α| + β·t reaches it, about 0.498 s.
				doc, floor, cylinder := scene(t)
				from := tiltedPose(t, math.Ldexp(1, -10), 0)
				path := rollingCase{omega: r3.Vec{Y: .5}, center: from.Translation(), seconds: 1}.path()
				path.From = from
				req := bandRequest()
				req.PointResolution = units.Millimeters(16)
				report := sweep(t, doc, floor, cylinder, path, req, order)
				require.Equal(t, decad.SweepPersistentBand, report.Outcome, "cause=%v", report.Cause)
				gate := (.25 - math.Sin(math.Ldexp(1, -10))) / .5
				end := report.ContactTrack.End().Elapsed.Value.Base()
				require.LessOrEqual(t, end, gate)
				require.Greater(t, end, gate-.01)
			})
		})
	}
}

func pathDurationOf(path decad.RigidDriftSegment) float64 { return path.Duration.Base() }

// The rolling track on the §2 tray's floor, a face-local plane
// (docs/multibody-dynamics-design.md §10.6): the track runs the column test
// over the coordinate box of the cylinder's eight staged corners' ideal paths
// at every grid fraction of its band search. The tray is posed so its floor's
// top face is z = −10, the plain floor's of rollingScene.
//
// Legs shown to fail (each deleted or zeroed in turn, fixture red, then
// restored):
//   - the column test in the band search: TestSweepPairRollingTowardTrayWall's
//     track reaches the duration with the cylinder's side through the wall;
//   - the radius reach of the end centers' box: the same track reaches the
//     duration, its box shrunk to the axis.
//
// The reach's gram term, r·(√(1 + gram) − 1), is about 1e-12·r at the most
// rounded valid pose and has no red fixture; it follows the bound
// |B·w|² <= (1 + gram)·|w|². With the end centers' box deleted, the corners'
// box alone ends TestSweepPairRollingOnTrayFloor's whole turn a third of the
// way round: a capability, not a soundness, leg.

// trayRollingScene is rollingScene's cylinder with the §2 tray in place of the
// floor, and the tray's path, at rest at the given translation.
func trayRollingScene(t *testing.T, doc *decad.Document, half float64, at r3.Vec,
	seconds float64) (*decad.Body, *decad.Body, decad.RigidDriftSegment) {
	t.Helper()
	tray := sceneTrayBody(t, doc)
	cylinder := revolvedCylinder(t, doc, -half, half, rollingAxisY, 10)
	path := sweepDrift(r3.Vec{}, seconds)
	path.From = contactPose(t, at)
	return tray, cylinder, path
}

func TestSweepPairRollingOnTrayFloor(t *testing.T) {
	// One turn of the roll, closing and orbit drifts of
	// TestSweepPairRollingCylinder, on the tray's floor with the walls at
	// y = −110 and y = 50: every corner path stays more than 30 mm from them,
	// and the track publishes the plain floor's outcome, depths and manifold.
	omega := 2 * math.Pi
	axis := r3.Vec{Y: rollingAxisY}
	roll := r3.Vec{Y: -omega * 10}
	for _, tc := range []struct {
		name  string
		touch bool // an exact touch track, with no band
		rollingCase
	}{
		{name: "without slip", touch: true,
			rollingCase: rollingCase{omega: r3.Vec{X: omega}, v: roll, center: axis, seconds: 1, half: 15}},
		{name: "sinking", rollingCase: rollingCase{omega: r3.Vec{X: omega},
			v: r3.Vec{Y: roll.Y, Z: -.25}, center: axis, seconds: 1, half: 15}},
		{name: "off-axis pivot", rollingCase: rollingCase{omega: r3.Vec{X: omega}, v: roll,
			center: r3.Vec{Y: rollingAxisY, Z: .125}, seconds: 1, half: 15}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			floor, plainCylinder := rollingScene(t, doc, tc.half, -100)
			tray, cylinder, trayPath := trayRollingScene(t, doc, tc.half, r3.Vec{Y: -30, Z: -10}, tc.seconds)
			for order := range 2 {
				sweep := func(s *decad.Body, sPath decad.RigidDriftSegment, m *decad.Body) *decad.SweepReport {
					a, b := s, m
					pathA, pathB := decad.PairPath(sPath), decad.PairPath(tc.path())
					if order == 1 {
						a, b, pathA, pathB = m, s, pathB, pathA
					}
					report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, bandRequest())
					require.NoError(t, err)
					return report
				}
				plain := sweep(floor, sweepDrift(r3.Vec{}, tc.seconds), plainCylinder)
				report := sweep(tray, trayPath, cylinder)
				require.Equal(t, plain.Outcome, report.Outcome, "order %d cause=%v", order, report.Cause)
				want := decad.SweepPersistentBand
				if tc.touch {
					want = decad.SweepPersistentTouch
				}
				require.Equal(t, want, report.Outcome)
				track, plainTrack := report.ContactTrack, plain.ContactTrack
				require.Equal(t, 1.0, track.End().Fraction.Base())
				require.Equal(t, plainTrack.Band(), track.Band())
				for _, fraction := range []float64{.25, .5, 1} {
					band, err := track.BandAt(units.Scalar(fraction))
					require.NoError(t, err)
					plainBand, err := plainTrack.BandAt(units.Scalar(fraction))
					require.NoError(t, err)
					require.Equal(t, plainBand, band, "fraction %v", fraction)
					manifold, err := track.ManifoldAt(units.Scalar(fraction))
					require.NoError(t, err)
					plainManifold, err := plainTrack.ManifoldAt(units.Scalar(fraction))
					require.NoError(t, err)
					require.Len(t, manifold.Points, 2)
					for i, point := range manifold.Points {
						want := plainManifold.Points[i]
						require.Equal(t, want.OnA, point.OnA, "fraction %v end %d", fraction, i)
						require.Equal(t, want.OnB, point.OnB, "fraction %v end %d", fraction, i)
						require.Equal(t, want.Separation, point.Separation)
						require.Equal(t, want.Normal, point.Normal)
					}
				}
				if tc.touch {
					require.Nil(t, track.Band())
				} else {
					require.NotNil(t, track.Band())
					require.InDelta(t, tc.depth(1), track.Band().Value.Base(), track.Band().Bound.Base()+1e-9)
				}
			}
		})
	}
}

func TestSweepPairRollingTowardTrayWall(t *testing.T) {
	// The roll drift of TestSweepPairRollingCylinder over a sixteenth of a
	// second, toward the tray's wall at y = −7.625, 1 mm from the cylinder's
	// side at the start. The cylinder turns about its own axis, so its end
	// centers only translate and the column box is their path box grown by
	// the radius: its −y side is 3.375 − 10 − |v|·t, which reaches the wall's
	// projection at |v|·t = 1. The track ends at the last grid fraction
	// before that. The corners' own path box, which turns with them, reaches
	// the wall far sooner and never decides.
	const seconds = 1.0 / 16
	// The grid step of a 2⁻⁴ s sweep at the 2⁻²⁰ s resolution.
	const step = 1.0 / (1 << 16)
	omega := 2 * math.Pi
	c := rollingCase{omega: r3.Vec{X: omega}, v: r3.Vec{Y: -omega * 10}, center: r3.Vec{Y: rollingAxisY},
		seconds: seconds, half: 15}
	doc := decad.New()
	tray, cylinder, trayPath := trayRollingScene(t, doc, 15, r3.Vec{Y: 72.375, Z: -10}, seconds)
	const wall = -7.625
	speed := math.Abs(c.v.Y)
	// 1/(|v|·T) lies far from every grid point, so its float floor is exact.
	want := gridFloor(1/(speed*seconds), step)
	for order := range 2 {
		a, b := tray, cylinder
		pathA, pathB := decad.PairPath(trayPath), decad.PairPath(c.path())
		if order == 1 {
			a, b, pathA, pathB = cylinder, tray, pathB, pathA
		}
		report, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, bandRequest())
		require.NoError(t, err)
		require.Equal(t, decad.SweepPersistentBand, report.Outcome, "order %d cause=%v", order, report.Cause)
		require.Equal(t, decad.ContactTouching, report.InitialEvent.Relation)
		track := report.ContactTrack
		require.Equal(t, want, track.End().Fraction.Base())
		elapsed := track.End().Elapsed.Value.Base()
		// The true side stays clear of the wall through the track.
		require.Greater(t, rollingAxisY-10-speed*elapsed, wall)
		// The exact roll keeps a zero depth through the track's end.
		require.NotNil(t, track.Band())
		require.Zero(t, track.Band().Value.Base())
		_, _, err = report.CertifiedPosesAt(units.Seconds(elapsed))
		require.NoError(t, err)
		_, _, err = report.CertifiedPosesAt(units.Seconds(seconds))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	}
}
