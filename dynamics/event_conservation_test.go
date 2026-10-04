package dynamics

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCrossProductErrorEnclosesArmAndNormalBounds(t *testing.T) {
	arm := [3]*big.Rat{big.NewRat(2, 1), big.NewRat(3, 1), new(big.Rat)}
	action := [3]*big.Rat{big.NewRat(5, 1), big.NewRat(-7, 1), new(big.Rat)}
	armError := [3]*big.Rat{big.NewRat(1, 4), big.NewRat(1, 2), new(big.Rat)}
	normalError := [3]*big.Rat{big.NewRat(1, 10), big.NewRat(1, 5), new(big.Rat)}
	bound := crossProductError(arm, action, armError, normalError, 2)

	// Both upper arm and normal deviations increase the magnitude of torque.
	actual := new(big.Rat).Sub(
		new(big.Rat).Mul(big.NewRat(9, 4), big.NewRat(-36, 5)),
		new(big.Rat).Mul(big.NewRat(7, 2), big.NewRat(51, 10)))
	nominal := big.NewRat(-29, 1)
	deviation := absRat(new(big.Rat).Sub(actual, nominal))
	require.Zero(t, deviation.Cmp(bound))
	require.Equal(t, "101/20", bound.RatString())
}

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
			MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 2,
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
	require.Empty(t, w.eventConservationFailure(event))
	require.Len(t, event.PointImpulses, len(event.Manifold.Points))
	require.NotZero(t, event.PostAngularVelocityB.Y.Base())

	changed := event
	changed.PostAngularVelocityB.Y = units.RadiansPerSecond(
		event.PostAngularVelocityB.Y.Base() + 1)
	require.Equal(t, "contact event angular momentum exceeds point impulse residual",
		w.eventConservationFailure(changed))

	changed = event
	changed.PointImpulses = append([]ContactPointImpulse(nil), event.PointImpulses...)
	changed.PointImpulses[0].Normal = units.KilogramMillimetersPerSecond(
		changed.PointImpulses[0].Normal.Base() + 1)
	require.Equal(t, "contact event point impulses do not match aggregate impulse",
		w.eventConservationFailure(changed))

	changed = event
	changed.PointImpulses = append([]ContactPointImpulse(nil), event.PointImpulses...)
	changed.PointImpulses[0].Normal = units.KilogramMillimetersPerSecond(math.Inf(1))
	require.Equal(t, "contact event has invalid point impulse",
		w.eventConservationFailure(changed))

	changed.PointImpulses[0].Normal = units.KilogramMillimetersPerSecond(math.NaN())
	require.Equal(t, "contact event has invalid point impulse",
		w.eventConservationFailure(changed))

	changed.PointImpulses[0] = event.PointImpulses[0]
	changed.PointImpulses[0].Tangent.X = units.KilogramMillimetersPerSecond(math.Inf(1))
	require.Equal(t, "contact event has invalid point impulse",
		w.eventConservationFailure(changed))
}

func TestTorqueDrivenFloorImpactChecksAngularEvent(t *testing.T) {
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
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2},
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
	require.Empty(t, w.eventConservationFailure(event))
	require.Len(t, event.PointImpulses, len(event.Manifold.Points))
	changed := event
	changed.PostAngularVelocityB.Z = units.RadiansPerSecond(event.PostAngularVelocityB.Z.Base() + 1)
	require.Equal(t, "contact event angular momentum exceeds point impulse residual",
		w.eventConservationFailure(changed))
	changed = event
	changed.PointImpulses = nil
	require.Equal(t, "contact event lacks point impulses for angular response",
		w.eventConservationFailure(changed))
}

func TestEventConservationRejectsEnergyGainWithBalancedMomentum(t *testing.T) {
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
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2,
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
	require.Empty(t, w.eventConservationFailure(event))

	// This changed pair retains total momentum and each body's stated impulse.
	// Its post-event energy exceeds the real event's pre-event energy.
	gaining := event
	gaining.NormalImpulse = units.KilogramMillimetersPerSecond(250)
	gaining.PostVelocityA = velocity(-150)
	gaining.PostVelocityB = velocity(150)
	require.Equal(t, "contact event increases kinetic energy beyond numerical residual",
		w.eventConservationFailure(gaining))

	// A wider held mass interval must not excuse the same numerical impulse.
	w.parts[0].mass.Mass.Bound = units.Kilograms(.1)
	require.Equal(t, "contact event linear momentum exceeds impulse residual",
		w.eventConservationFailure(event))
}

func TestKinematicEventConservationRejectsGainBeyondDriverWork(t *testing.T) {
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
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2},
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
	event := report.Events[0]
	require.Empty(t, w.eventConservationFailure(event))
	require.InDelta(t, 9600, report.Conservation.KinematicWork.Value.Base(), 1e-3)

	// The changed event balances momentum but gives the box more energy than
	// the driver's published speed and impulse can supply.
	gaining := event
	gaining.NormalImpulse = units.KilogramMillimetersPerSecond(250)
	gaining.PostVelocityB.X = units.MillimetersPerSecond(250)
	require.Equal(t, "contact event increases kinetic energy beyond work and numerical residual",
		w.eventConservationFailure(gaining))

	w.parts[1].mass.Mass.Bound = units.Kilograms(.01)
	w.step.ImpulseResidual = units.KilogramMillimetersPerSecond(3)
	require.Empty(t, w.eventConservationFailure(event))
	require.Equal(t, "contact event increases kinetic energy beyond work and numerical residual",
		w.eventConservationFailure(gaining))
}

func TestKinematicEventReportsNegativeDriverWork(t *testing.T) {
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
			MaxPoseEvaluations:      128, MaxIterations: 8, MaxEvents: 2},
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
	require.Empty(t, w.eventConservationFailure(report.Events[0]))
	require.InDelta(t, 120, report.Events[0].NormalImpulse.Base(), 1e-4)
	require.InDelta(t, -9600, report.Conservation.KinematicWork.Value.Base(), 1e-3)
	require.Equal(t, units.Torque, report.Conservation.KinematicWork.Bound.Kind())
	require.Less(t, report.Conservation.KinematicWork.Bound.Base(), 1e-6)
}
