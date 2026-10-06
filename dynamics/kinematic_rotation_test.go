package dynamics_test

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func rotatingNormalDriver(t *testing.T, translation float64) decad.PoseSegment {
	t.Helper()
	turn, err := r3.RotationAround(r3.Vec{X: 5, Y: 5, Z: 5}, r3.Vec{X: 1}, units.Degrees(90))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: translation})
	require.NoError(t, err)
	end, err := turn.Then(shift)
	require.NoError(t, err)
	return decad.PoseSegment{From: r3.Identity(), To: end, Duration: units.Seconds(1)}
}

func TestKinematicRotatingDriverInteriorImpactUsesProductionGeometry(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 19, 0, 29, 10, 0, 10)
	before := doc.Bodies()
	path := rotatingNormalDriver(t, 20)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	sweep, err := doc.SweepPair(t.Context(), driver, box, path,
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t), Duration: path.Duration},
		decad.SweepRequest{ContactRequest: decad.ContactRequest{
			PointResolution: units.Millimeters(1e-6), NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
	require.Nil(t, sweep.Event.Manifold)
	require.InDelta(t, .45, sweep.Bracket.To.Elapsed.Value.Base(), 1e-8)
	for _, sample := range sweep.Samples {
		if sample.At.Fraction != sweep.Bracket.To.Fraction {
			continue
		}
		full, readErr := path.To.Screw()
		require.NoError(t, readErr)
		inverse, readErr := sample.PoseA.Inverse()
		require.NoError(t, readErr)
		relative, readErr := inverse.Then(path.To)
		require.NoError(t, readErr)
		sliced, readErr := relative.Screw()
		require.NoError(t, readErr)
		require.Equal(t, full.Axis, sliced.Axis)
		require.NotEqual(t, full.Point.Z, sliced.Point.Z)
		require.InDelta(t, full.Point.Z, sliced.Point.Z, 1e-12)
		middle := (1 + sample.At.Fraction.Base()) / 2
		fullTurn, readErr := full.At(middle)
		require.NoError(t, readErr)
		originalMiddle, readErr := path.From.Then(fullTurn)
		require.NoError(t, readErr)
		slicedTurn, readErr := sliced.At(.5)
		require.NoError(t, readErr)
		slicedMiddle, readErr := sample.PoseA.Then(slicedTurn)
		require.NoError(t, readErr)
		require.InDelta(t, originalMiddle.Translation().X, slicedMiddle.Translation().X, 1e-8)
	}

	w := kinematicImpactWorld(t, doc, driver, box, false, 2)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver, Path: path}}}, path.Duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Next)
	require.Len(t, report.Events, 1)
	require.Equal(t, r3.Vec{X: 1}, report.Events[0].Manifold.Points[0].Normal.Value)
	require.InDelta(t, 30, report.Events[0].NormalImpulse.Base(), 1e-6)
	require.InDelta(t, .45, report.Events[0].Time.Base(), 1e-8)
	require.InDelta(t, 600, report.Conservation.KinematicWork.Value.Base(), 1e-6)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 30, finalBox.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 16.5, finalBox.Pose.Translation().X, 1e-5)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	require.Equal(t, path.To, finalDriver.Pose)
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
	checkpoint, err := report.Trace.Sample(report.Events[0].Time)
	require.NoError(t, err)
	postBox, ok := checkpoint.Body(box)
	require.True(t, ok)
	require.Equal(t, report.Events[0].PostVelocity, postBox.LinearVelocity)
	require.Equal(t, before, doc.Bodies())
}

func TestKinematicRotatingDriverMovesClearPair(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 100, 0, 110, 10, 0, 10)
	path := rotatingNormalDriver(t, 20)
	w := kinematicBoxWorld(t, doc, driver, box)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver, Path: path}}}, path.Duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	require.Equal(t, path.To, finalDriver.Pose)
	require.Equal(t, zeroVelocity(), finalDriver.LinearVelocity)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, r3.Identity(), finalBox.Pose)
}

func TestKinematicRotatingDriverInteriorImpactReverseWorldOrder(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	driver := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 19, 0, 29, 10, 0, 10)
	path := rotatingNormalDriver(t, 20)
	w := kinematicImpactWorld(t, doc, driver, box, true, 2)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: driver, Path: path}}}, path.Duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Equal(t, r3.Vec{X: -1}, report.Events[0].Manifold.Points[0].Normal.Value)
	require.InDelta(t, 30, report.Events[0].NormalImpulse.Base(), 1e-6)
	finalBox, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 30, finalBox.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 16.5, finalBox.Pose.Translation().X, 1e-5)
	finalDriver, ok := report.Next.Body(driver)
	require.True(t, ok)
	require.Equal(t, path.To, finalDriver.Pose)
}

// TestKinematicHingedPaddleStrikesWithItsField drives a paddle about a hinge
// at (5, 0) on the z axis, clockwise at 0.1 rad/s. Its tip edge strikes the
// face of a free 2 kg box 0.1 mm away, about 0.025 s in. The struck point
// moves at the field ω × (x − hinge), about 4 mm/s along the face normal,
// not at the paddle's center, so the closed-form impulse is
// (1 + e)·v_n / (1/m + (r × n)_z²/I_zz), with r the lever from the box's
// mass center, and the driver delivers that impulse times v_n as work. The
// root package proves no departure of an edge from a face while the driver
// turns, so the step keeps the impact as its certified prefix and stops
// with StepPairUndecided.
//
// Legs of the rotating driver's field (docs/multibody-dynamics-design.md
// §6.4), each shown to fail by deleting it, watching this test go red, then
// restoring it:
//   - driverMotionOf's −ω × p term, which moves the field's axis from the
//     hinge to the world origin: the driver's reported velocity misses the
//     field at its pose origin by 0.5 mm/s.
//   - the kinematic lever in nominalPoints and in the certificate's
//     constraint, and the kinematic branch of both pointVelocity readings:
//     the proposal or the certificate reads the field at the world origin,
//     where the paddle moves at 0.5 mm/s along y only, and no impact is
//     published.
//   - the certificate's reading of the field's angular part (liftDriver), and its
//     kinematic work at the contact point: the certificate refuses the
//     proposal and no impact is published.
//   - islandKinematicWork's field at the contact point: the work reading
//     misses λ·v_n.
//   - the event's driver spin (PreAngularVelocityA): the reported spin is
//     zero.
//
// Legs not shown to fail: the witness spread of islandKinematicWork and the
// driver spin term of closingSpeedUpper only widen a bound, so deleting one
// tightens it and can never admit a value; the angular comparison of the
// warm-start cache only refuses a restart, and a restarted proposal is
// certified again.
func TestKinematicHingedPaddleStrikesWithItsField(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	paddle := makeBox(t, doc, 0, 0, 10, 40, 0, 10)
	box := makeBox(t, doc, 10.1, 30, 20.1, 50, 0, 10)
	hinge := r3.Vec{X: 5}
	turn, err := r3.RotationAround(hinge, r3.Vec{Z: -1}, units.Radians(.01))
	require.NoError(t, err)
	path := decad.PoseSegment{From: r3.Identity(), To: turn, Duration: units.Seconds(.1)}
	w := kinematicImpactWorld(t, doc, paddle, box, false, 2)
	start, err := w.NewState([]dynamics.BodyState{
		{Body: paddle, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := w.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: paddle, Path: path}}}, path.Duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Equal(t, dynamics.BodyPair{A: paddle, B: box}, event.Pair)
	// The tip reaches the face when 5·cos θ + 40·sin θ = 5.1, at
	// θ ≈ 0.0025 rad, a quarter of the turn.
	require.InDelta(t, .025, event.Time.Base(), .001)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepPairUndecided, report.Diagnostics[0].Code)
	require.Equal(t, event.Time, report.Diagnostics[0].From)
	require.Contains(t, report.Diagnostics[0].Reason, fmt.Sprintf("(%d)", decad.SweepDepartureUnproved))

	omega := r3.Vec{Z: -.1}
	field := func(x r3.Vec) r3.Vec { return omega.Cross(x.Sub(hinge)) }
	const mass, restitution, inertia = 2.0, .5, 2 * (100 + 400) / 12.0
	center := r3.Vec{X: 15.1, Y: 40, Z: 5}
	require.Len(t, event.Manifold.Points, 2)
	normal := event.Manifold.Points[0].Normal.Value
	require.InDelta(t, 1, normal.X, 1e-12)
	// The tip edge meets the face along z at one height, so both points
	// share one speed and one lever about z.
	tip := event.Manifold.Points[0].OnA.Value
	speed := field(tip).Dot(normal)
	require.InDelta(t, 4, speed, .01)
	lever := tip.Sub(center).Cross(normal).Z
	impulse := (1 + restitution) * speed / (1/mass + lever*lever/inertia)
	const tolerance = 1e-6 // the world's ImpulseResidual
	require.InDelta(t, impulse, event.NormalImpulse.Base(), tolerance)
	require.InDelta(t, impulse/mass, event.PostVelocityB.X.Base(), 1e-6)
	require.InDelta(t, impulse*lever/inertia, event.PostAngularVelocityB.Z.Base(), 1e-6)
	require.InDelta(t, field(event.PoseA.Translation()).X, event.PreVelocityA.X.Base(), 1e-12)
	require.InDelta(t, field(event.PoseA.Translation()).Y, event.PreVelocityA.Y.Base(), 1e-12)
	require.InDelta(t, -.1, event.PreAngularVelocityA.Z.Base(), 1e-15)
	work, ok := dynamics.KinematicWork(w, report.Events)
	require.True(t, ok)
	require.InDelta(t, impulse*speed, work.Value.Base(), 1e-6)
}
