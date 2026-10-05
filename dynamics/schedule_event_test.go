package dynamics_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These tests drive events inside a slice of the step of a world of four or
// more bodies (docs/multibody-dynamics-design.md §5, §6.6, §7.1, §13 PR 6)
// through the real producers: SweepPair brackets every impact, the island
// solver certifies every response, and Trace.Sample replays every slice.
//
// Legs shown to fail (each deleted, its test watched go red, then restored):
//   - the common velocity of co-moving island bodies (commonVelocities):
//     TestScheduledStepBouncesAStack stops with StepPairUndecided, since the
//     stack's float velocities differ by about 1e-14 mm/s and SweepPair
//     cannot prove their persistent touch (SweepContactTrackUnproved);
//   - the co-moving tests of that rule (the VelocityResidual/8 pair test and
//     the VelocityResidual/16 group test, both deleted):
//     TestIslandTwoSphereImpactThroughGeneralPath stops with
//     StepIslandResidual, since the two spheres that leave apart would share
//     one velocity the linear law refuses;
//   - the group correction of bodies joined by persistent tracks:
//     TestScheduledStepBouncesAStack stops with StepCorrectionFailed, since
//     lifting the lower box alone drives it into the upper one;
//   - the bracket travel in the correction allowance:
//     TestScheduledStepBounces/"bracket travel" stops with
//     StepCorrectionFailed at the first impact, whose rounded poses
//     penetrate by about 1.2e-7 mm while that subtest's ContactSlop is
//     1e-8 mm;
//   - kinematic work in the island energy gate:
//     TestScheduledStepKinematicIsland stops with StepIslandResidual
//     ("energy"), since the sphere gains 1152 kg·mm²/s² of kinetic energy;
//   - the per-sample check of box-excluded pairs at rounded poses:
//     TestScheduledStepBoxExclusionAtRoundedPoses, at the slice end ("fixed")
//     and inside a slice ("moving").
//
// The closing-speed bound behind the bracket travel adds each body's spin
// times its lever; every fixture here translates, so that term is zero and
// cannot be observed. It only widens an allowance the correction check
// (§6.6's touching and sweep proofs) still judges.

// bounceStepConfig stops a bounce with ImpactSpeed 16 mm/s: an incoming
// normal speed of 16 mm/s or less targets zero.
func bounceStepConfig() dynamics.StepConfig {
	config := pairMaterialStepConfig()
	config.ImpactSpeed = units.MillimetersPerSecond(16)
	config.MaxEvents = 16
	return config
}

// bracketTravelConfig lowers ContactSlop below the first impact's rounded
// penetration, so only the bracket travel admits its correction.
func bracketTravelConfig() dynamics.StepConfig {
	config := bounceStepConfig()
	config.ContactSlop = units.Millimeters(1e-8)
	return config
}

// bounceScene is a radius-5 source sphere with the exact supplied mass of
// exactSphereMass (1 kg) over a fixed 40 mm floor whose top is at z = 0,
// under an optional fixed ceiling, with fixed parked boxes 500 mm away so
// the world has four bodies. Every material has restitution 0.5 and no
// friction.
type bounceScene struct {
	doc            *decad.Document
	floor, ceiling *decad.Body
	ball           *decad.Body
	world          *dynamics.World
	state          dynamics.State
}

// newBounceScene centers the sphere at (0, 0, z) with Z velocity vz. A
// positive ceiling puts the ceiling's lower face at that height.
func newBounceScene(t *testing.T, ceiling, z, vz float64, config dynamics.StepConfig) bounceScene {
	t.Helper()
	scene := bounceScene{doc: decad.New()}
	scene.floor = makeBox(t, scene.doc, -20, -20, 20, 20, -10, 10)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	bodies := []dynamics.RigidBody{{Body: scene.floor, Role: dynamics.Fixed, Material: material}}
	if ceiling > 0 {
		scene.ceiling = makeBox(t, scene.doc, -20, -20, 20, 20, ceiling, 10)
		bodies = append(bodies, dynamics.RigidBody{Body: scene.ceiling, Role: dynamics.Fixed, Material: material})
	}
	for x := 500.0; len(bodies) < 3; x += 100 {
		bodies = append(bodies, dynamics.RigidBody{Body: makeBox(t, scene.doc, x, 0, x+10, 10, 0, 10),
			Role: dynamics.Fixed, Material: material})
	}
	scene.ball = makeBall(t, scene.doc)
	mass := exactSphereMass()
	bodies = append(bodies, dynamics.RigidBody{Body: scene.ball, Role: dynamics.Dynamic, Supplied: &mass,
		Material: material})
	var err error
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: bodies, Step: config})
	require.NoError(t, err)
	start, err := r3.Translation(r3.Vec{Z: z})
	require.NoError(t, err)
	var entries []dynamics.BodyState
	for _, body := range bodies[:len(bodies)-1] {
		entries = append(entries, dynamics.BodyState{Body: body.Body, Pose: r3.Identity(),
			LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)})
	}
	entries = append(entries, dynamics.BodyState{Body: scene.ball, Pose: start,
		LinearVelocity: dynamics.QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(vz)}, AngularVelocity: zeroAngular(t)})
	scene.state, err = scene.world.NewState(entries)
	require.NoError(t, err)
	return scene
}

func (s bounceScene) ballZ(t *testing.T, state dynamics.State) (float64, float64) {
	t.Helper()
	entry, ok := state.Body(s.ball)
	require.True(t, ok)
	return entry.Pose.Translation().Z, entry.LinearVelocity.Z.Base()
}

// The floor-and-ceiling bounce: the sphere's center moves between z = 5
// (floor touch) and z = 7 (ceiling touch, lower face at 12), starting at
// z = 6 with −256 mm/s and no gravity. Restitution 0.5 halves the speed at
// every impact: 256 → 128 at 1/256 s on the floor, 128 → 64 at 5/256 s on
// the ceiling, 64 → 32 at 13/256 s, 32 → 16 at 29/256 s, and at 61/256 s the
// incoming 16 mm/s meets ImpactSpeed and targets zero, so the sphere rests on
// the floor through the end of the 64/256 s step (§5.1).
var floorCeilingBounces = []struct {
	at       float64 // exact impact time, in 1/256 s
	floor    bool
	pre      float64
	post     float64
	impulse  float64
	position float64 // sphere center height at the impact
}{
	{1, true, -256, 128, 384, 5},
	{5, false, 128, -64, 192, 7},
	{13, true, -64, 32, 96, 5},
	{29, false, 32, -16, 48, 7},
	{61, true, -16, 0, 16, 5},
}

// floorCeilingHeight is the sphere's exact center height at time t in 1/256 s.
func floorCeilingHeight(at float64) float64 {
	z, v, last := 6.0, -256.0, 0.0
	for _, bounce := range floorCeilingBounces {
		if at <= bounce.at {
			break
		}
		z, v, last = bounce.position, bounce.post, bounce.at
	}
	return z + v*(at-last)/256
}

// TestScheduledStepBounces runs the five events of the floor-and-ceiling
// bounce in one step. Each event lies at its SweepPair bracket's right
// sample, at most one TimeResolution after the slice's exact impact time, and
// the correction puts the sphere back in exact touch there, so the k-th
// event lies at most k·TimeResolution after its closed-form time. Positions
// inside the slices carry that delay times the speed: below 256 mm/s ·
// 5e-9 s ≈ 1.3e-6 mm, so the slack is 2e-6 mm.
func TestScheduledStepBounces(t *testing.T) {
	for name, config := range map[string]dynamics.StepConfig{
		"slop": bounceStepConfig(), "bracket travel": bracketTravelConfig()} {
		t.Run(name, func(t *testing.T) {
			scene := newBounceScene(t, 12, 6, -256, config)
			dt := units.Seconds(.25)
			report, err := scene.world.Step(t.Context(), scene.state, dynamics.StepInput{Gravity: zeroAcceleration()}, dt)
			require.NoError(t, err)
			require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
			require.Len(t, report.Events, len(floorCeilingBounces))
			require.Len(t, report.Islands, len(floorCeilingBounces))
			resolution := config.TimeResolution.Base()
			for k, bounce := range floorCeilingBounces {
				event := report.Events[k]
				exact := bounce.at / 256
				require.Equal(t, dynamics.ContactImpact, event.Kind, "event %d", k)
				require.Equal(t, k, event.Island, "event %d", k)
				require.GreaterOrEqual(t, event.Time.Base(), exact, "event %d", k)
				require.InDelta(t, exact, event.Time.Base(), float64(k+1)*resolution, "event %d", k)
				other := scene.ceiling
				if bounce.floor {
					other = scene.floor
				}
				require.Equal(t, dynamics.BodyPair{A: other, B: scene.ball}, event.Pair, "event %d", k)
				require.InDelta(t, bounce.pre, event.PreVelocityB.Z.Base(), 1e-9, "event %d", k)
				require.InDelta(t, bounce.post, event.PostVelocityB.Z.Base(), 1e-6, "event %d", k)
				require.InDelta(t, bounce.impulse, event.NormalImpulse.Base(), 1e-6, "event %d", k)
				require.Equal(t, report.Islands[k].Time, event.Time, "event %d", k)
				// The correction removes the rounded poses' penetration, which
				// the bracket bounds by speed times TimeResolution.
				require.LessOrEqual(t, math.Abs(event.PositionChangeB.Z), 256*resolution, "event %d", k)
				// Sampling at the event returns the post-event state, in touch.
				state, err := report.Trace.Sample(event.Time)
				require.NoError(t, err, "event %d", k)
				z, v := scene.ballZ(t, state)
				require.Equal(t, bounce.position, z, "event %d", k)
				require.InDelta(t, bounce.post, v, 1e-6, "event %d", k)
			}
			// Six slices: one before each event and the rest after the last.
			proofs := dynamics.TraceSliceProofs(report.Trace)
			require.Len(t, proofs, len(floorCeilingBounces)+1)
			resting := false
			for _, proof := range proofs[len(proofs)-1] {
				if proof.Pair == (dynamics.BodyPair{A: scene.floor, B: scene.ball}) {
					require.NotNil(t, proof.Sweep)
					require.Equal(t, decad.SweepPersistentTouch, proof.Sweep.Outcome)
					resting = true
				}
			}
			require.True(t, resting)
			// Interior samples on every slice follow the closed form.
			for _, at := range []float64{0.5, 3, 9, 21, 45, 62, 63.5} {
				state, err := report.Trace.Sample(units.Seconds(at / 256))
				require.NoError(t, err, "sample at %g/256 s", at)
				z, _ := scene.ballZ(t, state)
				require.InDelta(t, floorCeilingHeight(at), z, 2e-6, "sample at %g/256 s", at)
			}
			z, v := scene.ballZ(t, *report.Next)
			require.Equal(t, 5.0, z)
			require.Equal(t, 0.0, v)
			// The floor and the ceiling deliver every impulse: the sphere's
			// momentum goes from −256 to 0 through +384 −192 +96 −48 +16.
			require.NotNil(t, report.Conservation)
			require.InDelta(t, 256, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-5)
			require.InDelta(t, 0, report.Conservation.Completion.LinearMomentum.Value.Z.Base(), 1e-6)
		})
	}
}

// TestScheduledStepEventBudget stops the bounce when its third event reaches
// MaxEvents 3 with time remaining (§5.1, §12). The report is Undecided,
// keeps the three published events and islands, and its Trace replays the
// certified prefix up to the third impact.
func TestScheduledStepEventBudget(t *testing.T) {
	config := bounceStepConfig()
	config.MaxEvents = 3
	scene := newBounceScene(t, 12, 6, -256, config)
	bodies := scene.doc.Bodies()
	dt := units.Seconds(.25)
	report, err := scene.world.Step(t.Context(), scene.state, dynamics.StepInput{Gravity: zeroAcceleration()}, dt)
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Nil(t, report.Conservation)
	require.Len(t, report.Diagnostics, 1)
	diagnostic := report.Diagnostics[0]
	require.Equal(t, dynamics.StepEventBudget, diagnostic.Code)
	require.Equal(t, units.Scalar(3), diagnostic.Limit)
	require.InDelta(t, 13.0/256, diagnostic.From.Base(), 4*config.TimeResolution.Base())
	require.Equal(t, dt, diagnostic.To)
	require.Len(t, report.Events, 3)
	require.Len(t, report.Islands, 3)
	require.Equal(t, bodies, scene.doc.Bodies())
	// The prefix replays every certified time up to the third impact, and
	// nothing after it.
	for _, at := range []float64{0, 0.5, 3, 9, 12} {
		state, err := report.Trace.Sample(units.Seconds(at / 256))
		require.NoError(t, err, "sample at %g/256 s", at)
		z, _ := scene.ballZ(t, state)
		require.InDelta(t, floorCeilingHeight(at), z, 2e-6, "sample at %g/256 s", at)
	}
	state, err := report.Trace.Sample(diagnostic.From)
	require.NoError(t, err)
	z, _ := scene.ballZ(t, state)
	require.InDelta(t, 5, z, 2e-6)
	_, err = report.Trace.Sample(units.Seconds(14.0 / 256))
	require.ErrorIs(t, err, dynamics.ErrInvalidInput)
}

// TestScheduledStepBouncesAStack drops a stack of two 10 mm boxes (1 kg
// each) onto the floor at 256 mm/s with no gravity. The upper box rests
// exactly on the lower one, so the pair touches at the step start and joins
// the contact set; at the floor impact (1/256 s) both boxes form one island
// with the floor pair. Restitution 0.5 sends the stack up together at
// 128 mm/s: the floor delivers (1 + 0.5)·2·256 = 768 kg·mm/s and the lower
// box passes 384 kg·mm/s to the upper one. Both boxes publish exactly the
// same velocity (the common-velocity rule), are corrected together, and the
// upper pair continues in persistent touch.
func TestScheduledStepBouncesAStack(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	parked := makeBox(t, doc, 500, 0, 510, 10, 0, 10)
	lower := makeBox(t, doc, 0, 0, 10, 10, 1, 10)
	upper := makeBox(t, doc, 0, 0, 10, 10, 11, 10)
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	config := bounceStepConfig()
	config.MaxIterations = 4096
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Step: config, Bodies: []dynamics.RigidBody{
		{Body: floor, Role: dynamics.Fixed, Material: material},
		{Body: parked, Role: dynamics.Fixed, Material: material},
		{Body: lower, Role: dynamics.Dynamic, Density: &density, Material: material},
		{Body: upper, Role: dynamics.Dynamic, Density: &density, Material: material},
	}})
	require.NoError(t, err)
	down := dynamics.QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(-256)}
	state, err := world.NewState([]dynamics.BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: parked, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: lower, Pose: r3.Identity(), LinearVelocity: down, AngularVelocity: zeroAngular(t)},
		{Body: upper, Pose: r3.Identity(), LinearVelocity: down, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()},
		units.Seconds(1.0/64))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	// The touching pair joins the contact set silently at the step start
	// (§6.1: zero speed, zero impulse); the floor impact and the pair it
	// lifts are solved as one island.
	require.Len(t, report.Events, 2)
	floorEvent, stackEvent := report.Events[0], report.Events[1]
	require.Equal(t, dynamics.BodyPair{A: floor, B: lower}, floorEvent.Pair)
	require.Equal(t, dynamics.BodyPair{A: lower, B: upper}, stackEvent.Pair)
	require.Equal(t, floorEvent.Island, stackEvent.Island)
	require.Equal(t, []*decad.Body{floor, lower, upper}, report.Islands[floorEvent.Island].Bodies)
	require.InDelta(t, 1.0/256, floorEvent.Time.Base(), config.TimeResolution.Base())
	require.InDelta(t, 768, floorEvent.NormalImpulse.Base(), 1e-5)
	require.InDelta(t, 384, stackEvent.NormalImpulse.Base(), 1e-5)
	// One exactly shared velocity, and one shared correction.
	require.Equal(t, stackEvent.PostVelocityA, stackEvent.PostVelocityB)
	require.InDelta(t, 128, stackEvent.PostVelocityB.Z.Base(), 1e-6)
	require.Equal(t, floorEvent.PositionChangeB, stackEvent.PositionChangeB)
	require.Positive(t, stackEvent.PositionChangeB.Z)
	lowerEnd, ok := report.Next.Body(lower)
	require.True(t, ok)
	upperEnd, ok := report.Next.Body(upper)
	require.True(t, ok)
	require.Equal(t, lowerEnd.LinearVelocity, upperEnd.LinearVelocity)
	require.Equal(t, lowerEnd.Pose, upperEnd.Pose)
	// The lower box's bottom left the floor at 1/256 s and rose at 128 mm/s
	// for 3/256 s: 1.5 mm, less the bracket delay times the speed.
	require.InDelta(t, 0.5, lowerEnd.Pose.Translation().Z, 1e-6)
	proofs := dynamics.TraceSliceProofs(report.Trace)
	require.Len(t, proofs, 2)
	for _, proof := range proofs[1] {
		switch proof.Pair {
		case dynamics.BodyPair{A: lower, B: upper}:
			require.Equal(t, decad.SweepPersistentTouch, proof.Sweep.Outcome)
		case dynamics.BodyPair{A: floor, B: lower}:
			require.Equal(t, decad.SweepDepartedClear, proof.Sweep.Outcome)
		}
	}
}

// TestScheduledStepKinematicIsland lifts a kinematic platform (a 40 mm
// slab, top at z = 0) at 32 mm/s into a sphere at rest 1 mm above it, with
// no gravity. The impact at 1/32 s is solved with the platform's driver
// velocity: restitution 0.5 sends the sphere up at 32 + 0.5·32 = 48 mm/s, an
// impulse of 48 kg·mm/s, and the driver delivers 48·32 = 1536 kg·mm²/s² of
// work while the sphere gains ½·48² = 1152.
func TestScheduledStepKinematicIsland(t *testing.T) {
	doc := decad.New()
	platform := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	parkedA := makeBox(t, doc, 500, 0, 510, 10, 0, 10)
	parkedB := makeBox(t, doc, 600, 0, 610, 10, 0, 10)
	ball := makeBall(t, doc)
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Step: bounceStepConfig(),
		Bodies: []dynamics.RigidBody{
			{Body: platform, Role: dynamics.Kinematic, Material: material},
			{Body: parkedA, Role: dynamics.Fixed, Material: material},
			{Body: parkedB, Role: dynamics.Fixed, Material: material},
			{Body: ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		}})
	require.NoError(t, err)
	start, err := r3.Translation(r3.Vec{Z: 6})
	require.NoError(t, err)
	lift, err := r3.Translation(r3.Vec{Z: 2})
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: platform, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: parkedA, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: parkedB, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: ball, Pose: start, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	dt := units.Seconds(1.0 / 16)
	report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: platform,
			Path: decad.PoseSegment{From: r3.Identity(), To: lift, Duration: dt}}}}, dt)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Equal(t, dynamics.BodyPair{A: platform, B: ball}, event.Pair)
	require.InDelta(t, 1.0/32, event.Time.Base(), bounceStepConfig().TimeResolution.Base())
	require.Equal(t, units.MillimetersPerSecond(32), event.PreVelocityA.Z)
	require.Equal(t, units.MillimetersPerSecond(32), event.PostVelocityA.Z)
	require.InDelta(t, 48, event.NormalImpulse.Base(), 1e-6)
	require.InDelta(t, 48, event.PostVelocityB.Z.Base(), 1e-6)
	require.Equal(t, []*decad.Body{platform, ball}, report.Islands[0].Bodies)
	require.InDelta(t, 1152, report.Islands[0].Solver.EnergyResidual.Base(), 1e-3)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, 1536, report.Conservation.KinematicWork.Value.Base(), 1e-3)
	// The platform ends on its driver; the sphere rose 1.5 mm after the
	// impact at z = 6.
	platformEnd, ok := report.Next.Body(platform)
	require.True(t, ok)
	require.Equal(t, lift, platformEnd.Pose)
	ballEnd, ok := report.Next.Body(ball)
	require.True(t, ok)
	require.InDelta(t, 7.5, ballEnd.Pose.Translation().Z, 1e-6)
	require.InDelta(t, 48, ballEnd.LinearVelocity.Z.Base(), 1e-6)
	// Mid-approach the platform has risen 0.5 mm and the sphere is still.
	mid, err := report.Trace.Sample(units.Seconds(1.0 / 64))
	require.NoError(t, err)
	platformMid, _ := mid.Body(platform)
	ballMid, _ := mid.Body(ball)
	require.InDelta(t, 0.5, platformMid.Pose.Translation().Z, 1e-12)
	require.Equal(t, 6.0, ballMid.Pose.Translation().Z)
}

// TestScheduledStepGraze runs the two-body graze through the step of a world
// of four bodies: a sphere passing a fixed sphere at center distance 10 mm
// (the sum of the radii) touches it at 0.5 s with zero normal speed. The
// graze publishes a zero-impulse ContactGraze without ending the slice,
// since the pair's sweep replays the whole step.
func TestScheduledStepGraze(t *testing.T) {
	doc := decad.New()
	fixed, moving := makeBall(t, doc), makeBall(t, doc)
	parkedA := makeBox(t, doc, 500, 0, 510, 10, 0, 10)
	parkedB := makeBox(t, doc, 600, 0, 610, 10, 0, 10)
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Step: bounceStepConfig(),
		Bodies: []dynamics.RigidBody{
			{Body: fixed, Role: dynamics.Fixed, Material: material},
			{Body: moving, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
			{Body: parkedA, Role: dynamics.Fixed, Material: material},
			{Body: parkedB, Role: dynamics.Fixed, Material: material},
		}})
	require.NoError(t, err)
	start, err := r3.Translation(r3.Vec{X: 20, Y: 10})
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: fixed, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: moving, Pose: start, LinearVelocity: dynamics.QuantityVec{X: units.MillimetersPerSecond(-40),
			Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
		{Body: parkedA, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: parkedB, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Equal(t, dynamics.ContactGraze, event.Kind)
	require.Equal(t, dynamics.BodyPair{A: fixed, B: moving}, event.Pair)
	require.Equal(t, units.Seconds(.5), event.Time)
	require.Equal(t, -1, event.Island)
	require.Zero(t, event.NormalImpulse.Base())
	require.Len(t, event.PointImpulses, 1)
	require.Zero(t, event.PointImpulses[0].Normal.Base())
	require.Equal(t, r3.Vec{Y: 10}, event.PoseB.Translation())
	require.Empty(t, report.Islands)
	require.Len(t, dynamics.TraceSliceProofs(report.Trace), 1)
	sample, err := report.Trace.Sample(units.Seconds(.5))
	require.NoError(t, err)
	entry, _ := sample.Body(moving)
	require.Equal(t, r3.Vec{Y: 10}, entry.Pose.Translation())
	end, _ := report.Next.Body(moving)
	require.Equal(t, r3.Vec{X: -20, Y: 10}, end.Pose.Translation())
	require.Equal(t, units.MillimetersPerSecond(-40), end.LinearVelocity.X)
}

// TestScheduledStepEdgeTransition slides a 10 mm box at 5 mm/s off the
// 10 mm top of a fixed box, with no gravity. The touching pair joins the
// contact set silently at the step start (§6.1: zero speed, zero impulse),
// continues on a persistent track, and its transition bracket at 2 s publishes a
// zero-impulse ContactTransition; the box then drifts clear to x = 15.
func TestScheduledStepEdgeTransition(t *testing.T) {
	doc := decad.New()
	base := makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	box := makeBox(t, doc, 0, 0, 10, 10, 10, 10)
	parkedA := makeBox(t, doc, 500, 0, 510, 10, 0, 10)
	parkedB := makeBox(t, doc, 600, 0, 610, 10, 0, 10)
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Step: bounceStepConfig(),
		Bodies: []dynamics.RigidBody{
			{Body: base, Role: dynamics.Fixed, Material: material},
			{Body: box, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: parkedA, Role: dynamics.Fixed, Material: material},
			{Body: parkedB, Role: dynamics.Fixed, Material: material},
		}})
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: base, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: box, Pose: r3.Identity(), LinearVelocity: dynamics.QuantityVec{X: units.MillimetersPerSecond(5),
			Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)},
		{Body: parkedA, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: parkedB, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()}, units.Seconds(3))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Empty(t, report.Islands)
	transition := report.Events[0]
	require.Equal(t, dynamics.ContactTransition, transition.Kind)
	require.Equal(t, dynamics.BodyPair{A: base, B: box}, transition.Pair)
	require.Equal(t, -1, transition.Island)
	require.InDelta(t, 2, transition.Time.Base(), bounceStepConfig().TimeResolution.Base())
	require.Zero(t, transition.NormalImpulse.Base())
	proofs := dynamics.TraceSliceProofs(report.Trace)
	require.Len(t, proofs, 2)
	end, _ := report.Next.Body(box)
	require.InDelta(t, 15, end.Pose.Translation().X, 1e-8)
	require.Zero(t, end.Pose.Translation().Z)
	mid, err := report.Trace.Sample(units.Seconds(1))
	require.NoError(t, err)
	entry, _ := mid.Body(box)
	require.Equal(t, r3.Vec{X: 5}, entry.Pose.Translation())
	late, err := report.Trace.Sample(units.Seconds(2.5))
	require.NoError(t, err)
	entry, _ = late.Body(box)
	require.InDelta(t, 12.5, entry.Pose.Translation().X, 1e-8)
}

// boxExclusionScene places two 10 mm boxes at x ≈ 2^30 mm, where the float
// spacing u is 2^-22 mm, and drifts them along X for 1 s. Box A starts at
// x = 2^30 and moves 0.75u; box B's lower X face starts at 2^30 + 10 + u.
// Their swept boxes stay 0.25u apart, so the broad phase excludes the pair,
// yet A's rounded translation reaches 2^30 + u once 0.75u·f passes half a
// spacing, putting its upper face on B's lower face at x = 2^30 + 10 + u.
// When B moves 0.625u, its rounded translation stays at its start until
// 0.625u·f passes half a spacing (f = 0.8): at f = 0.75 the two rounded
// boxes meet, at f = 1 they are apart again. Two fixed boxes 1000 mm away
// along Y complete the world.
func boxExclusionScene(t *testing.T, moving bool) (*dynamics.World, dynamics.State, [2]*decad.Body) {
	t.Helper()
	const x0 = 1 << 30
	u := math.Ldexp(1, -22)
	doc := decad.New()
	a, b := makeBox(t, doc, 0, 0, 10, 10, 0, 10), makeBox(t, doc, 0, 0, 10, 10, 0, 10)
	parkedA := makeBox(t, doc, 0, 1000, 10, 1010, 0, 10)
	parkedB := makeBox(t, doc, 100, 1000, 110, 1010, 0, 10)
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	roleB := dynamics.Fixed
	var densityB *units.Value
	if moving {
		roleB, densityB = dynamics.Dynamic, &density
	}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Step: bounceStepConfig(),
		Bodies: []dynamics.RigidBody{
			{Body: a, Role: dynamics.Dynamic, Density: &density, Material: material},
			{Body: b, Role: roleB, Density: densityB, Material: material},
			{Body: parkedA, Role: dynamics.Fixed, Material: material},
			{Body: parkedB, Role: dynamics.Fixed, Material: material},
		}})
	require.NoError(t, err)
	poseA, err := r3.Translation(r3.Vec{X: x0})
	require.NoError(t, err)
	poseB, err := r3.Translation(r3.Vec{X: x0 + 10 + u})
	require.NoError(t, err)
	velocity := func(x float64) dynamics.QuantityVec {
		return dynamics.QuantityVec{X: units.MillimetersPerSecond(x), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}
	}
	speedB := 0.0
	if moving {
		speedB = 0.625 * u
	}
	state, err := world.NewState([]dynamics.BodyState{
		{Body: a, Pose: poseA, LinearVelocity: velocity(0.75 * u), AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: poseB, LinearVelocity: velocity(speedB), AngularVelocity: zeroAngular(t)},
		{Body: parkedA, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: parkedB, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	return world, state, [2]*decad.Body{a, b}
}

// TestScheduledStepBoxExclusionAtRoundedPoses checks §7.1's per-sample check
// of box-excluded pairs at both places a rounded pose is published: the
// slice end, and a replay sample inside the slice.
func TestScheduledStepBoxExclusionAtRoundedPoses(t *testing.T) {
	u := math.Ldexp(1, -22)
	t.Run("fixed", func(t *testing.T) {
		// B stays put, so A's rounded end pose meets it: the step refuses to
		// publish an end state no certificate covers.
		world, state, bodies := boxExclusionScene(t, false)
		report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()},
			units.Seconds(1))
		require.NoError(t, err)
		require.Equal(t, dynamics.Undecided, report.Status)
		require.Len(t, report.Diagnostics, 1)
		require.Equal(t, dynamics.StepPairUndecided, report.Diagnostics[0].Code)
		require.Equal(t, dynamics.BodyPair{A: bodies[0], B: bodies[1]}, report.Diagnostics[0].Pair)
	})
	t.Run("moving", func(t *testing.T) {
		world, state, bodies := boxExclusionScene(t, true)
		report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration()},
			units.Seconds(1))
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
		proofs := dynamics.TraceSliceProofs(report.Trace)
		require.Len(t, proofs, 1)
		for _, proof := range proofs[0] {
			if proof.Pair == (dynamics.BodyPair{A: bodies[0], B: bodies[1]}) {
				require.True(t, proof.BoxClear)
			}
		}
		const x0 = 1 << 30
		// Half way, both rounded translations sit at their starts, u apart.
		half, err := report.Trace.Sample(units.Seconds(.5))
		require.NoError(t, err)
		entryA, _ := half.Body(bodies[0])
		entryB, _ := half.Body(bodies[1])
		require.Equal(t, float64(x0), entryA.Pose.Translation().X)
		require.Equal(t, x0+10+u, entryB.Pose.Translation().X)
		// At three quarters A has rounded up by u and meets B.
		_, err = report.Trace.Sample(units.Seconds(.75))
		require.ErrorIs(t, err, dynamics.ErrUnsupported)
		// At the end both have rounded up by u, apart again.
		entryA, _ = report.Next.Body(bodies[0])
		entryB, _ = report.Next.Body(bodies[1])
		require.Equal(t, x0+u, entryA.Pose.Translation().X)
		require.Equal(t, x0+10+2*u, entryB.Pose.Translation().X)
	})
}

// TestScheduledStepDiagnosticsNameTheIsland reruns the two-sphere island
// with a supplied mass known only to 2^-10 kg, which the linear law refuses
// (TestIslandUncertainMassRefuses). The diagnostic names the island's bodies,
// the event time, and the gate's limit ImpulseResidual + m_hi·VelocityResidual
// = 1e-6 + (1 + 2^-10)·1e-6 kg·mm/s, rounded up to a float; the trace holds
// the certified prefix, the start state at time zero.
func TestScheduledStepDiagnosticsNameTheIsland(t *testing.T) {
	mass := exactSphereMass()
	mass.Mass.Bound = units.Kilograms(1.0 / 1024)
	mass.Mass.Exactness = decad.Approximate
	scene := newTwoSphereIsland(t, mass, pairMaterialStepConfig())
	report := scene.step(t)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Len(t, report.Diagnostics, 1)
	diagnostic := report.Diagnostics[0]
	require.Equal(t, dynamics.StepIslandResidual, diagnostic.Code)
	require.Equal(t, []*decad.Body{scene.a, scene.b}, diagnostic.Bodies)
	require.Equal(t, units.Seconds(0), diagnostic.From)
	require.Equal(t, units.Seconds(0), diagnostic.To)
	require.Equal(t, units.Impulse, diagnostic.Limit.Kind())
	require.InDelta(t, 1e-6+(1+1.0/1024)*1e-6, diagnostic.Limit.Base(), 1e-18)
	require.Empty(t, report.Events)
	start, err := report.Trace.Sample(units.Seconds(0))
	require.NoError(t, err)
	require.Equal(t, scene.state.Entries(), start.Entries())
	_, err = report.Trace.Sample(units.Seconds(.05))
	require.ErrorIs(t, err, dynamics.ErrInvalidInput)
}

// TestScheduledStepRotatingDriverIslandRefuses rests a 10 mm box on the
// kinematic platform of TestScheduledStepKinematicIsland while the platform
// turns a quarter turn about Z. The touching pair forms an island at the
// step start, and a rotating driver has no single contact-point velocity, so
// the step stops with StepUnsupported and names the pair.
func TestScheduledStepTurntableTouchRefuses(t *testing.T) {
	doc := decad.New()
	platform := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	parkedA := makeBox(t, doc, 500, 0, 510, 10, 0, 10)
	parkedB := makeBox(t, doc, 600, 0, 610, 10, 0, 10)
	block := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Step: bounceStepConfig(),
		Bodies: []dynamics.RigidBody{
			{Body: platform, Role: dynamics.Kinematic, Material: material},
			{Body: parkedA, Role: dynamics.Fixed, Material: material},
			{Body: parkedB, Role: dynamics.Fixed, Material: material},
			{Body: block, Role: dynamics.Dynamic, Density: &density, Material: material},
		}})
	require.NoError(t, err)
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(90))
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: platform, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: parkedA, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: parkedB, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: block, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	dt := units.Seconds(1.0 / 16)
	report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: zeroAcceleration(),
		Drivers: []dynamics.KinematicDriver{{Body: platform,
			Path: decad.PoseSegment{From: r3.Identity(), To: turn, Duration: dt}}}}, dt)
	require.NoError(t, err)
	// The island holds the block at rest on the turning platform: the field
	// ω × x is horizontal across the patch, so its normal speed is zero and
	// the zero-speed island publishes no event (§6.1). The root package
	// proves no touch track under a turning platform, so the pair's sweep
	// refuses the whole slice.
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Empty(t, report.Events)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepPairUndecided, report.Diagnostics[0].Code, "%+v", report.Diagnostics)
	require.Equal(t, dynamics.BodyPair{A: platform, B: block}, report.Diagnostics[0].Pair)
	require.Equal(t, units.Seconds(0), report.Diagnostics[0].From)
	require.Equal(t, dt, report.Diagnostics[0].To)
	require.Contains(t, report.Diagnostics[0].Reason, fmt.Sprintf("(%d)", decad.SweepContactTrackUnproved))
	// The certified prefix is the start: the block sits on the platform's
	// top face, touching with the normal +z, and nothing past it replays.
	held, err := report.Trace.Sample(units.Seconds(0))
	require.NoError(t, err)
	onPlatform, ok := held.Body(platform)
	require.True(t, ok)
	resting, ok := held.Body(block)
	require.True(t, ok)
	require.Equal(t, r3.Identity(), resting.Pose)
	require.Zero(t, resting.LinearVelocity.Z.Base())
	pair, err := doc.ContactPair(t.Context(), platform, block, onPlatform.Pose, resting.Pose, bounceStepConfig().Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, pair.Relation)
	require.NotEmpty(t, pair.Manifold.Points)
	for _, point := range pair.Manifold.Points {
		require.InDelta(t, 1, point.Normal.Value.Z, 1e-12)
		require.InDelta(t, 0, point.OnB.Value.Z, 1e-9)
	}
	_, err = report.Trace.Sample(units.Seconds(1.0 / 32))
	require.Error(t, err)
}
