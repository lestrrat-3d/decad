package dynamics

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func conservationBox(t *testing.T, doc *decad.Document, x0, x1 float64) *decad.Body {
	t.Helper()
	scene := sketch.NewWorld()
	profile, err := scene.CreateSketch(scene.XY())
	require.NoError(t, err)
	rectangle := profile.CreateRectangle(x0, -5, x1, 5)
	profile.Fix(rectangle.A)
	_, err = profile.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(profile, profile.Profiles()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	})
	require.NoError(t, err)
	return body
}

func TestSpinEventConservationUsesRealPointImpulses(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := conservationBox(t, doc, -5, 5)
	initial := conservationBox(t, doc, -5, 5)
	placement, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	b, err := initial.Placed(t.Context(), placement)
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := Material{Restitution: units.Scalar(.5), Friction: units.Scalar(.5)}
	w, err := NewWorld(t.Context(), doc, WorldConfig{
		Bodies: []RigidBody{{Body: a, Role: Dynamic, Density: &density, Material: material},
			{Body: b, Role: Dynamic, Density: &density, Material: material}},
		Step: StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	zeroLinear := QuantityVec{X: units.MillimetersPerSecond(0),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	zeroAngular := QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
	start, err := w.NewState([]BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: zeroLinear, AngularVelocity: zeroAngular},
		{Body: b, Pose: r3.Identity(), LinearVelocity: QuantityVec{
			X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroAngular},
	})
	require.NoError(t, err)
	zeroGravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
	report, err := w.Step(t.Context(), start,
		StepInput{Gravity: zeroGravity}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Len(t, event.PointImpulses, len(event.Manifold.Points))
	require.NotZero(t, event.PostAngularVelocityB.Y.Base())

	// The island certificate holds the event to its point impulses: a spin
	// the impulses do not explain, or one point's normal impulse changed by
	// 1 kg·mm/s, fails its angular and linear rows.
	gates := func(tamper func(*IslandProposal)) []string {
		t.Helper()
		names, gateErr := IslandProposalGates(t.Context(), w, start, zeroGravity, units.Seconds(.1), tamper)
		require.NoError(t, gateErr)
		return names
	}
	require.Empty(t, gates(func(*IslandProposal) {}))
	require.Contains(t, gates(func(p *IslandProposal) {
		p.Angular[1].Y = units.RadiansPerSecond(p.Angular[1].Y.Base() + 1)
	}), "angular momentum")
	require.Contains(t, gates(func(p *IslandProposal) { p.Lambda[0]++ }), "linear law")
}

func TestTorqueDrivenFloorImpactChecksAngularEvent(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	scene := sketch.NewWorld()
	plane, err := scene.CreateOffsetPlane(scene.XY(), -10)
	require.NoError(t, err)
	profile, err := scene.CreateSketch(plane)
	require.NoError(t, err)
	rectangle := profile.CreateRectangle(-20, -20, 20, 20)
	profile.Fix(rectangle.A)
	_, err = profile.Solve(t.Context())
	require.NoError(t, err)
	floor, err := doc.Extrude(profile, profile.Profiles()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	box := conservationBox(t, doc, -5, 5)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	w, err := NewWorld(t.Context(), doc, WorldConfig{
		Bodies: []RigidBody{{Body: floor, Role: Fixed, Material: material},
			{Body: box, Role: Dynamic, Density: &density, Material: material}},
		Step: StepConfig{Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)}, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096},
	})
	require.NoError(t, err)
	pose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	zeroLinear := QuantityVec{X: units.MillimetersPerSecond(0),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	zeroAngular := QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
	fall := zeroLinear
	fall.Z = units.MillimetersPerSecond(-100)
	start, err := w.NewState([]BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroLinear, AngularVelocity: zeroAngular},
		{Body: box, Pose: pose, LinearVelocity: fall, AngularVelocity: zeroAngular},
	})
	require.NoError(t, err)
	zeroGravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
	torque := QuantityVec{X: units.KilogramSquareMillimetersPerSecondSquared(0),
		Y: units.KilogramSquareMillimetersPerSecondSquared(0),
		Z: units.KilogramSquareMillimetersPerSecondSquared(50)}
	force := QuantityVec{X: units.KilogramMillimetersPerSecondSquared(0),
		Y: units.KilogramMillimetersPerSecondSquared(0),
		Z: units.KilogramMillimetersPerSecondSquared(0)}
	report, err := w.Step(t.Context(), start, StepInput{Gravity: zeroGravity,
		Loads: []BodyLoad{{Body: box, Force: force, Torque: torque}}}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Len(t, event.PointImpulses, len(event.Manifold.Points))
	// The torque kick spins the box about Z at 50/I_zz·0.2 rad/s; the
	// frictionless floor exerts no torque about Z, so the event keeps it.
	require.InDelta(t, event.PreAngularVelocityB.Z.Base(), event.PostAngularVelocityB.Z.Base(), 1e-9)
	require.Positive(t, event.PostAngularVelocityB.Z.Base())
}

func TestEventConservationRejectsEnergyGainWithBalancedMomentum(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := conservationBox(t, doc, 0, 10)
	b := conservationBox(t, doc, 35, 45)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	w, err := NewWorld(t.Context(), doc, WorldConfig{
		Bodies: []RigidBody{{Body: a, Role: Dynamic, Density: &density, Material: material},
			{Body: b, Role: Dynamic, Density: &density, Material: material}},
		Step: StepConfig{
			Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)},
			TimeResolution: units.Seconds(1e-9), ContactSlop: units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096,
		},
	})
	require.NoError(t, err)
	velocity := func(x float64) QuantityVec {
		return QuantityVec{X: units.MillimetersPerSecond(x),
			Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	}
	angular := QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
	start, err := w.NewState([]BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: velocity(100), AngularVelocity: angular},
		{Body: b, Pose: r3.Identity(), LinearVelocity: velocity(-100), AngularVelocity: angular},
	})
	require.NoError(t, err)
	zeroGravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
	report, err := w.Step(t.Context(), start, StepInput{Gravity: zeroGravity}, units.Seconds(.25))
	require.NoError(t, err)
	require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	// Equal 1 kg boxes closing at 200 mm/s with restitution 0.5 take
	// 150 kg·mm/s and leave at ∓50 mm/s, so kinetic energy falls from
	// 10000 to 2500.
	require.InDelta(t, 150, event.NormalImpulse.Base(), 1e-6)
	require.InDelta(t, -50, event.PostVelocityA.X.Base(), 1e-6)
	require.InDelta(t, 50, event.PostVelocityB.X.Base(), 1e-6)
	require.InDelta(t, 2500, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-6)
}

func TestKinematicEventConservationRejectsGainBeyondDriverWork(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	driver := conservationBox(t, doc, 0, 10)
	box := conservationBox(t, doc, 20, 30)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	w, err := NewWorld(t.Context(), doc, WorldConfig{
		Bodies: []RigidBody{{Body: driver, Role: Kinematic, Material: material},
			{Body: box, Role: Dynamic, Density: &density, Material: material}},
		Step: StepConfig{Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)}, TimeResolution: units.Seconds(1e-9),
			ContactSlop:             units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096},
	})
	require.NoError(t, err)
	zeroVelocity := QuantityVec{X: units.MillimetersPerSecond(0),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	zeroAngular := QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
	start, err := w.NewState([]BodyState{
		{Body: driver, Pose: r3.Identity(), LinearVelocity: zeroVelocity, AngularVelocity: zeroAngular},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity, AngularVelocity: zeroAngular},
	})
	require.NoError(t, err)
	endDriver, err := r3.Translation(r3.Vec{X: 20})
	require.NoError(t, err)
	duration := units.Seconds(.25)
	zeroGravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
	report, err := w.Step(t.Context(), start, StepInput{Gravity: zeroGravity,
		Drivers: []KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: endDriver, Duration: duration}}}}, duration)
	require.NoError(t, err)
	require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 9600, report.Conservation.KinematicWork.Value.Base(), 1e-3)
	// The driver at 80 mm/s strikes the box with restitution 0.5: the box
	// leaves at 120 mm/s from a 120 kg·mm/s impulse, and the driver's work
	// 120·80 = 9600 covers the box's 7200 of kinetic energy.
	require.InDelta(t, 120, report.Events[0].NormalImpulse.Base(), 1e-4)
	require.InDelta(t, 120, report.Events[0].PostVelocityB.X.Base(), 1e-4)
	require.LessOrEqual(t, report.Conservation.Completion.KineticEnergy.Value.Base(),
		report.Conservation.KinematicWork.Value.Base())
}

func TestKinematicEventReportsNegativeDriverWork(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := conservationBox(t, doc, 0, 10)
	driver := conservationBox(t, doc, 20, 30)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	w, err := NewWorld(t.Context(), doc, WorldConfig{
		Bodies: []RigidBody{{Body: box, Role: Dynamic, Density: &density, Material: material},
			{Body: driver, Role: Kinematic, Material: material}},
		Step: StepConfig{Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)}, TimeResolution: units.Seconds(1e-9),
			ContactSlop:             units.Millimeters(1e-6),
			VelocityResidual:        units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6),
			ImpactSpeed:             units.MillimetersPerSecond(0),
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096},
	})
	require.NoError(t, err)
	velocity := func(x float64) QuantityVec {
		return QuantityVec{X: units.MillimetersPerSecond(x),
			Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	}
	angular := QuantityVec{X: units.RadiansPerSecond(0),
		Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
	start, err := w.NewState([]BodyState{
		{Body: box, Pose: r3.Identity(), LinearVelocity: velocity(160), AngularVelocity: angular},
		{Body: driver, Pose: r3.Identity(), LinearVelocity: velocity(0), AngularVelocity: angular},
	})
	require.NoError(t, err)
	endDriver, err := r3.Translation(r3.Vec{X: 20})
	require.NoError(t, err)
	duration := units.Seconds(.25)
	gravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
	report, err := w.Step(t.Context(), start, StepInput{Gravity: gravity,
		Drivers: []KinematicDriver{{Body: driver,
			Path: decad.PoseSegment{From: r3.Identity(), To: endDriver, Duration: duration}}}}, duration)
	require.NoError(t, err)
	require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 120, report.Events[0].NormalImpulse.Base(), 1e-4)
	require.InDelta(t, -9600, report.Conservation.KinematicWork.Value.Base(), 1e-3)
	require.Equal(t, units.Torque, report.Conservation.KinematicWork.Bound.Kind())
	require.Less(t, report.Conservation.KinematicWork.Bound.Base(), 1e-6)
}
