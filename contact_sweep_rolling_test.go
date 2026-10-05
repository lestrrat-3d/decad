package decad_test

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
	t.Run("placed start", func(t *testing.T) {
		// ContactPair proves the ruling touch only at identity query poses.
		doc := decad.New()
		floor, cylinder := rollingScene(t, doc, 15, -100)
		path := roll.path()
		path.From = contactPose(t, r3.Vec{X: 1})
		path.Center.X = 1
		report, err := doc.SweepPair(t.Context(), floor, cylinder, sweepDrift(r3.Vec{}, 1), path, bandRequest())
		require.NoError(t, err)
		require.Equal(t, decad.SweepUndecided, report.Outcome)
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
