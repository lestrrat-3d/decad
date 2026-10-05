package dynamics_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These tests run closed-form fixtures of the two- and three-body steps
// through the island solver of a world of four or more bodies, each with
// extra fixed boxes far away that the broad phase excludes, and assert
// the closed forms' numbers (docs/multibody-dynamics-design.md §6.2, §6.5):
//   - TestIslandDirectStartRestsAStack: the direct start lets eight sweeps
//     certify a two-box stack. Removing the direct start (directStart
//     reporting false) refuses it with the non-penetration gate after eight
//     sweeps.
//   - TestIslandStickStartStopsAFrictionalImpact: the sticking start with
//     the proportional friction split stops a box whose friction exactly
//     removes its slide. Without the proportional split the minimum-norm
//     split leaves two corners outside their disks, the sweeps start from
//     the frictionless state, and complementarity fails after 64 sweeps.
//   - TestIslandRollingSphereKeepsItsSpin: a sphere sticking to a floor and
//     a wall rolls along the corner. Snapping its 2.2 rad/s spin under the
//     coarse 100 rad/s AngularVelocityResidual without the lever bound
//     publishes no spin and fails the stick gate; taking the slice-end pose
//     from the drift path instead of the sphere certificates refuses the
//     step with "rounded slice poses differ from the pair certificate".

// farBox is a 10 mm fixed box x mm away along X.
func farBox(t *testing.T, doc *decad.Document, x float64) *decad.Body {
	t.Helper()
	return makeBox(t, doc, x-5, -5, x+5, 5, 0, 10)
}

// TestIslandDirectStartRestsAStack is TestTwoDynamicBoxStackUsesBothMasses
// as a four-body world: a 1 kg box on the floor under a 2 kg box, gravity
// −1000 mm/s² over 0.05 s. The floor delivers 150 kg·mm/s and the lower box
// 100 kg·mm/s to the upper one, and both stop, within MaxIterations 8.
func TestIslandDirectStartRestsAStack(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	lower := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	upper := makeBox(t, doc, -5, -5, 5, 5, 10, 10)
	far := farBox(t, doc, 1000)
	lowerDensity := units.KilogramsPerCubicMillimeter(.001)
	upperDensity := units.KilogramsPerCubicMillimeter(.002)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	config := pairMaterialStepConfig()
	config.MaxIterations, config.MaxEvents = 8, 3
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: floor, Role: dynamics.Fixed, Material: material},
		{Body: lower, Role: dynamics.Dynamic, Density: &lowerDensity, Material: material},
		{Body: upper, Role: dynamics.Dynamic, Density: &upperDensity, Material: material},
		{Body: far, Role: dynamics.Fixed, Material: material},
	}, Step: config})
	require.NoError(t, err)
	var entries []dynamics.BodyState
	for _, body := range []*decad.Body{floor, lower, upper, far} {
		entries = append(entries, dynamics.BodyState{Body: body, Pose: r3.Identity(),
			LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)})
	}
	state, err := world.NewState(entries)
	require.NoError(t, err)
	report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: gravityZ(-1000)}, units.Seconds(.05))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 2)
	require.Equal(t, dynamics.BodyPair{A: floor, B: lower}, report.Events[0].Pair)
	require.Equal(t, dynamics.BodyPair{A: lower, B: upper}, report.Events[1].Pair)
	require.InDelta(t, 150, report.Events[0].NormalImpulse.Base(), 1e-6)
	require.InDelta(t, 100, report.Events[1].NormalImpulse.Base(), 1e-6)
	require.InDelta(t, 150, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	require.LessOrEqual(t, report.Islands[0].Solver.Iterations, 8)
	for _, body := range []*decad.Body{lower, upper} {
		entry, found := report.Next.Body(body)
		require.True(t, found)
		require.Equal(t, zeroVelocity(), entry.LinearVelocity)
		require.Equal(t, r3.Identity(), entry.Pose)
	}
}

// TestIslandStickStartStopsAFrictionalImpact is the floor-first case of
// TestFixedFloorInteriorFrictionImpactUsesRealGeometry as a four-body world
// (two far boxes):
// a 1 kg box falls at (40, 0, −160) mm/s from 10 mm onto the floor with
// friction 0.25 and no restitution. It lands at 1/16 s; the floor delivers
// 160 kg·mm/s, friction 0.25·160 = 40 kg·mm/s removes the slide exactly, and
// the box stops 2.5 mm along X with no spin.
func TestIslandStickStartStopsAFrictionalImpact(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -100, -100, 100, 100, -10, 10)
	box := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	far, farther := farBox(t, doc, 1000), farBox(t, doc, 2000)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(.25)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: floor, Role: dynamics.Fixed, Material: material},
		{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
		{Body: far, Role: dynamics.Fixed, Material: material},
		{Body: farther, Role: dynamics.Fixed, Material: material},
	}, Step: pairMaterialStepConfig()})
	require.NoError(t, err)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(40), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(-160)}
	state, err := world.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: pose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
		{Body: far, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: farther, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()},
		units.Seconds(.125))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.InDelta(t, .0625, event.Time.Base(), 1e-9)
	require.InDelta(t, 160, event.NormalImpulse.Base(), 1e-6)
	require.InDelta(t, -40, event.TangentImpulse.X.Base(), 1e-6)
	require.Len(t, event.PointImpulses, 4)
	require.LessOrEqual(t, event.Solver.AngularUpper.Base(), 1e-6)
	for _, point := range event.PointImpulses {
		require.LessOrEqual(t, math.Abs(point.Tangent.X.Base()), .25*point.Normal.Base()+1e-6)
	}
	require.InDelta(t, -40, report.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
	require.InDelta(t, 160, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	end, ok := report.Next.Body(box)
	require.True(t, ok)
	require.Equal(t, zeroVelocity(), end.LinearVelocity)
	require.Equal(t, zeroAngular(t), end.AngularVelocity)
	require.InDelta(t, 2.5, end.Pose.Translation().X, 1e-6)
	require.InDelta(t, 0, end.Pose.Translation().Z, 1e-6)
}

// TestIslandRollingSphereKeepsItsSpin is TestThreeBodyFrictionIslandRealPath
// as a four-body world: a 1 kg, 5 mm sphere touching a floor and a wall
// arrives at (−100, 20, −100) mm/s with friction 0.5 and no restitution.
// Both contacts stick: the sphere leaves at (0, 100/9, 0) mm/s rolling with
// spin (−20/9, 0, 20/9) rad/s, each face delivering 100 kg·mm/s normal and
// 40/9 kg·mm/s of friction against Y. The step's AngularVelocityResidual is
// 100 rad/s.
func TestIslandRollingSphereKeepsItsSpin(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	wall := makeBox(t, doc, -10, -20, 0, 20, -10, 30)
	ball := makeBall(t, doc)
	far := farBox(t, doc, 1000)
	pose, err := r3.Translation(r3.Vec{X: 5, Z: 5})
	require.NoError(t, err)
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(.5)}
	config := pairMaterialStepConfig()
	config.AngularVelocityResidual, config.MaxEvents = units.RadiansPerSecond(100), 4
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: wall, Role: dynamics.Fixed, Material: material},
			{Body: far, Role: dynamics.Fixed, Material: material}},
		Excluded: []dynamics.BodyPair{{A: floor, B: wall}}, Step: config})
	require.NoError(t, err)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(-100), Y: units.MillimetersPerSecond(20),
		Z: units.MillimetersPerSecond(-100)}
	state, err := world.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: pose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
		{Body: wall, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: far, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	duration := units.Seconds(.1)
	report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 2)
	require.Equal(t, dynamics.BodyPair{A: floor, B: ball}, report.Events[0].Pair)
	require.Equal(t, dynamics.BodyPair{A: ball, B: wall}, report.Events[1].Pair)
	for _, event := range report.Events {
		require.InDelta(t, 100, event.NormalImpulse.Base(), 1e-9)
		require.InDelta(t, 40.0/9, math.Abs(event.TangentImpulse.Y.Base()), 1e-9)
		require.LessOrEqual(t, event.Solver.TangentResidual.Base(), config.VelocityResidual.Base())
	}
	final, found := report.Next.Body(ball)
	require.True(t, found)
	require.Equal(t, 0.0, final.LinearVelocity.X.Base())
	require.InDelta(t, 100.0/9, final.LinearVelocity.Y.Base(), 1e-9)
	require.Equal(t, 0.0, final.LinearVelocity.Z.Base())
	require.InDelta(t, -20.0/9, final.AngularVelocity.X.Base(), 1e-9)
	require.Equal(t, 0.0, final.AngularVelocity.Y.Base())
	require.InDelta(t, 20.0/9, final.AngularVelocity.Z.Base(), 1e-9)
	// AngularUpper bounds the published spin from above.
	spin := math.Hypot(final.AngularVelocity.X.Base(), final.AngularVelocity.Z.Base())
	require.GreaterOrEqual(t, report.Islands[0].Solver.AngularUpper.Base(), spin)
	require.InDelta(t, spin, report.Islands[0].Solver.AngularUpper.Base(), 1e-12)
	require.InDelta(t, 10.0/9, final.Pose.Translation().Y, 1e-12)
	require.InDelta(t, 5, final.Pose.Translation().X, 1e-12)
	require.InDelta(t, 5, final.Pose.Translation().Z, 1e-12)
	replayed, err := report.Trace.Sample(duration)
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), replayed.Entries())
	for _, at := range []units.Value{units.Seconds(.025), units.Seconds(.075)} {
		sample, err := report.Trace.Sample(at)
		require.NoError(t, err)
		ballAt, ok := sample.Body(ball)
		require.True(t, ok)
		for _, fixed := range []*decad.Body{floor, wall} {
			contact, err := doc.ContactPair(t.Context(), fixed, ball, r3.Identity(), ballAt.Pose, config.Contact)
			require.NoError(t, err)
			require.Equal(t, decad.ContactTouching, contact.Relation)
		}
	}
}
