package dynamics

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFixedFloorFrictionStepUsesRealGeometry(t *testing.T) {
	doc := decad.New()
	floor := sourceBoxForFriction(t, doc, -100, -100, 100, 100, -10)
	box := sourceBoxForFriction(t, doc, -5, -5, 5, 5, 0)
	floorBefore, err := floor.Bounds()
	require.NoError(t, err)
	boxBefore, err := box.Bounds()
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 4)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := Material{Restitution: units.Scalar(0), Friction: units.Scalar(.5)}
	cfg := StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 2}
	w, err := NewWorld(t.Context(), doc, WorldConfig{Bodies: []RigidBody{
		{Body: floor, Role: Fixed, Material: material},
		{Body: box, Role: Dynamic, Density: &density, Material: material},
	}, Step: cfg})
	require.NoError(t, err)
	zeroV := QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(0)}
	zeroW := QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
		Z: units.RadiansPerSecond(0)}
	start, err := w.NewState([]BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroV, AngularVelocity: zeroW},
		{Body: box, Pose: r3.Identity(), LinearVelocity: QuantityVec{
			X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroW},
	})
	require.NoError(t, err)
	gravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(-1000)}
	report, err := w.Step(t.Context(), start, StepInput{Gravity: gravity}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Next)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, 5000, report.Conservation.Input.KineticEnergy.Value.Base(), 1e-6)
	require.InDelta(t, 10000, report.Conservation.AfterKick.KineticEnergy.Value.Base(), 1e-6)
	require.InDelta(t, 1250, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, -100, report.Conservation.GravityImpulse.Value.Z.Base(), 1e-6)
	require.InDelta(t, -50, report.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
	require.InDelta(t, 100, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Equal(t, ContactImpact, event.Kind)
	require.Zero(t, event.Time.Base())
	require.InDelta(t, 100, event.NormalImpulse.Base(), 1e-6)
	require.InDelta(t, -50, event.TangentImpulse.X.Base(), 1e-6)
	require.LessOrEqual(t, math.Abs(event.TangentImpulse.Y.Base()), cfg.ImpulseResidual.Base())
	require.Zero(t, event.TangentImpulse.Z.Base())
	require.Equal(t, units.Impulse, event.TangentImpulse.X.Kind())
	require.NotNil(t, event.Solver)
	require.Positive(t, event.Solver.Iterations)
	require.Len(t, event.PointImpulses, len(event.Manifold.Points))
	var pointNormal, pointTangent float64
	for _, impulse := range event.PointImpulses {
		pointNormal += impulse.Normal.Base()
		pointTangent += impulse.Tangent.X.Base()
		require.Equal(t, units.Impulse, impulse.Tangent.Y.Kind())
	}
	require.InDelta(t, event.NormalImpulse.Base(), pointNormal, 1e-6)
	require.InDelta(t, event.TangentImpulse.X.Base(), pointTangent, 1e-6)
	final, ok := report.Next.Body(box)
	require.True(t, ok)
	require.InDelta(t, 50, final.LinearVelocity.X.Base(), 1e-6)
	require.Zero(t, final.LinearVelocity.Z.Base())
	require.InDelta(t, 5, final.Pose.Translation().X, 1e-6)
	require.Zero(t, final.Pose.Translation().Z)
	initialContact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), final.Pose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, initialContact.Relation)
	postAtZero, err := report.Trace.Sample(units.Seconds(0))
	require.NoError(t, err)
	postBody, ok := postAtZero.Body(box)
	require.True(t, ok)
	require.Equal(t, final.LinearVelocity, postBody.LinearVelocity)
	endAtDuration, err := report.Trace.Sample(units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), endAtDuration.Entries())
	mass, err := box.MassProperties(t.Context(), density)
	require.NoError(t, err)
	stationary := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(.1)}
	sweepRequest := decad.SweepRequest{ContactRequest: request, TimeResolution: cfg.TimeResolution,
		MaxPoseEvaluations: cfg.MaxPoseEvaluations, StartPolicy: decad.ContinueCertifiedTouch}
	ideal, err := doc.SweepPair(t.Context(), floor, box, stationary,
		decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: final.LinearVelocity, AngularVelocity: zeroW,
			Duration: units.Seconds(.1)}, sweepRequest)
	require.NoError(t, err)
	rounded, err := doc.SweepPair(t.Context(), floor, box, stationary,
		decad.PoseSegment{From: r3.Identity(), To: final.Pose, Duration: units.Seconds(.1)}, sweepRequest)
	require.NoError(t, err)
	for _, sweep := range []*decad.SweepReport{ideal, rounded} {
		require.Equal(t, decad.SweepPersistentTouch, sweep.Outcome)
		require.NotNil(t, sweep.ContactTrack)
		_, err := sweep.ContactTrack.ManifoldAt(units.Scalar(.5))
		require.NoError(t, err)
	}
	staticStart, err := w.NewState([]BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroV, AngularVelocity: zeroW},
		{Body: box, Pose: r3.Identity(), LinearVelocity: zeroV, AngularVelocity: zeroW},
	})
	require.NoError(t, err)
	staticReport, err := w.Step(t.Context(), staticStart, StepInput{Gravity: gravity}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, Advanced, staticReport.Status, "%+v", staticReport.Diagnostics)
	require.NotNil(t, staticReport.Conservation)
	require.InDelta(t, 100, staticReport.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	require.Len(t, staticReport.Events, 1)
	require.InDelta(t, 100, staticReport.Events[0].NormalImpulse.Base(), 1e-6)
	require.Zero(t, staticReport.Events[0].TangentImpulse.X.Base())
	require.Equal(t, units.Impulse, staticReport.Events[0].TangentImpulse.X.Kind())
	staticFinal, ok := staticReport.Next.Body(box)
	require.True(t, ok)
	require.Zero(t, staticFinal.LinearVelocity.X.Base())
	require.Zero(t, staticFinal.LinearVelocity.Z.Base())
	require.Equal(t, r3.Identity(), staticFinal.Pose)
	limited := cfg
	limited.MaxEvents = 1
	limitedWorld, err := NewWorld(t.Context(), doc, WorldConfig{Bodies: []RigidBody{
		{Body: floor, Role: Fixed, Material: material},
		{Body: box, Role: Dynamic, Density: &density, Material: material},
	}, Step: limited})
	require.NoError(t, err)
	limitedStart, err := limitedWorld.NewState(start.Entries())
	require.NoError(t, err)
	limitedReport, err := limitedWorld.Step(t.Context(), limitedStart,
		StepInput{Gravity: gravity}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, Undecided, limitedReport.Status)
	require.Nil(t, limitedReport.Next)
	require.Equal(t, []*decad.Body{floor, box}, doc.Bodies())
	floorAfter, err := floor.Bounds()
	require.NoError(t, err)
	boxAfter, err := box.Bounds()
	require.NoError(t, err)
	require.Equal(t, floorBefore, floorAfter)
	require.Equal(t, boxBefore, boxAfter)
}

func TestReverseFixedFloorFrictionStepUsesRealGeometry(t *testing.T) {
	doc := decad.New()
	floor := sourceBoxForFriction(t, doc, -100, -100, 100, 100, -10)
	box := sourceBoxForFriction(t, doc, -5, -5, 5, 5, 0)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), box, floor, r3.Identity(), r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 4)
	for _, point := range contact.Manifold.Points {
		require.Equal(t, r3.Vec{Z: -1}, point.Normal.Value)
	}
	density := units.KilogramsPerCubicMillimeter(.001)
	material := Material{Restitution: units.Scalar(0), Friction: units.Scalar(.5)}
	cfg := StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 2}
	w, err := NewWorld(t.Context(), doc, WorldConfig{Bodies: []RigidBody{
		{Body: box, Role: Dynamic, Density: &density, Material: material},
		{Body: floor, Role: Fixed, Material: material},
	}, Step: cfg})
	require.NoError(t, err)
	zeroV := QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(0)}
	zeroW := QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
		Z: units.RadiansPerSecond(0)}
	gravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(-1000)}
	for _, slip := range []float64{100, 0} {
		start, stateErr := w.NewState([]BodyState{
			{Body: box, Pose: r3.Identity(), LinearVelocity: QuantityVec{
				X: units.MillimetersPerSecond(slip), Y: units.MillimetersPerSecond(0),
				Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroW},
			{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroV, AngularVelocity: zeroW},
		})
		require.NoError(t, stateErr)
		report, stepErr := w.Step(t.Context(), start, StepInput{Gravity: gravity}, units.Seconds(.1))
		require.NoError(t, stepErr)
		require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
		require.Len(t, report.Events, 1)
		event := report.Events[0]
		require.Equal(t, BodyPair{A: box, B: floor}, event.Pair)
		require.Len(t, event.Manifold.Points, 4)
		require.Len(t, event.PointImpulses, 4)
		require.InDelta(t, 100, event.NormalImpulse.Base(), 1e-6)
		require.InDelta(t, 50*slip/100, event.TangentImpulse.X.Base(), 1e-6)
		require.Equal(t, zeroV, event.PostVelocityB)
		require.InDelta(t, -50*slip/100, report.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
		require.InDelta(t, 100, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
		var normal, tangent float64
		for i, point := range event.Manifold.Points {
			require.Equal(t, r3.Vec{Z: -1}, point.Normal.Value)
			normal += event.PointImpulses[i].Normal.Base()
			tangent += event.PointImpulses[i].Tangent.X.Base()
		}
		require.InDelta(t, event.NormalImpulse.Base(), normal, 1e-6)
		require.InDelta(t, event.TangentImpulse.X.Base(), tangent, 1e-6)
		final, exists := report.Next.Body(box)
		require.True(t, exists)
		require.InDelta(t, slip/2, final.LinearVelocity.X.Base(), 1e-6)
		require.Zero(t, final.LinearVelocity.Z.Base())
		require.InDelta(t, slip/20, final.Pose.Translation().X, 1e-6)
		endContact, contactErr := doc.ContactPair(t.Context(), box, floor, final.Pose, r3.Identity(), request)
		require.NoError(t, contactErr)
		require.Equal(t, decad.ContactTouching, endContact.Relation)
	}
}

func TestFixedFloorFrictionRepeatsAtTranslatedPose(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "floor-first", true: "box-first"}[reverse], func(t *testing.T) {
			doc := decad.New()
			floor := sourceBoxForFriction(t, doc, -100, -100, 100, 100, -10)
			box := sourceBoxForFriction(t, doc, -5, -5, 5, 5, 0)
			density := units.KilogramsPerCubicMillimeter(.001)
			material := Material{Restitution: units.Scalar(0), Friction: units.Scalar(.5)}
			cfg := StepConfig{Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)}, TimeResolution: units.Seconds(1e-9),
				ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
				AngularVelocityResidual: units.RadiansPerSecond(1e-6),
				ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
				PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
				MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 2}
			bodies := []RigidBody{{Body: floor, Role: Fixed, Material: material},
				{Body: box, Role: Dynamic, Density: &density, Material: material}}
			if reverse {
				bodies[0], bodies[1] = bodies[1], bodies[0]
			}
			world, err := NewWorld(t.Context(), doc, WorldConfig{Bodies: bodies, Step: cfg})
			require.NoError(t, err)
			zeroV := QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
				Z: units.MillimetersPerSecond(0)}
			zeroW := QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
				Z: units.RadiansPerSecond(0)}
			entries := []BodyState{{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroV, AngularVelocity: zeroW},
				{Body: box, Pose: r3.Identity(), LinearVelocity: QuantityVec{
					X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
					Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroW}}
			if reverse {
				entries[0], entries[1] = entries[1], entries[0]
			}
			state, err := world.NewState(entries)
			require.NoError(t, err)
			gravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
				Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(-1000)}
			for step, wantX := range []float64{5, 6.25} {
				duration := []float64{.1, .05}[step]
				report, stepErr := world.Step(t.Context(), state, StepInput{Gravity: gravity}, units.Seconds(duration))
				require.NoError(t, stepErr)
				require.Equal(t, Advanced, report.Status, "step %d: %+v", step, report.Diagnostics)
				require.Len(t, report.Events, 1)
				require.InDelta(t, 100*duration/.1, report.Events[0].NormalImpulse.Base(), 1e-6)
				wantTangent := -50 * duration / .1
				if reverse {
					wantTangent = -wantTangent
				}
				require.InDelta(t, wantTangent, report.Events[0].TangentImpulse.X.Base(), 1e-6)
				boxState, ok := report.Next.Body(box)
				require.True(t, ok)
				require.InDelta(t, wantX, boxState.Pose.Translation().X, 1e-6)
				require.InDelta(t, 50-float64(step)*25, boxState.LinearVelocity.X.Base(), 1e-6)
				contact, contactErr := doc.ContactPair(t.Context(), floor, box, r3.Identity(), boxState.Pose, cfg.Contact)
				require.NoError(t, contactErr)
				require.Equal(t, decad.ContactTouching, contact.Relation)
				state = *report.Next
			}
			translated, ok := state.Body(box)
			require.True(t, ok)
			translated.LinearVelocity = zeroV
			for i := range entries {
				if entries[i].Body == box {
					entries[i] = translated
				}
			}
			staticState, err := world.NewState(entries)
			require.NoError(t, err)
			staticReport, err := world.Step(t.Context(), staticState,
				StepInput{Gravity: gravity}, units.Seconds(.05))
			require.NoError(t, err)
			require.Equal(t, Advanced, staticReport.Status, "%+v", staticReport.Diagnostics)
			require.Len(t, staticReport.Events, 1)
			staticBox, ok := staticReport.Next.Body(box)
			require.True(t, ok)
			require.Equal(t, translated.Pose, staticBox.Pose)
			require.Equal(t, zeroV, staticBox.LinearVelocity)
		})
	}
}

func TestFixedFloorFrictionRejectsUnsupportedMaterialsAndPatch(t *testing.T) {
	doc := decad.New()
	floor := sourceBoxForFriction(t, doc, -2, -5, 2, 5, -10)
	box := sourceBoxForFriction(t, doc, -5, -5, 5, 5, 0)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := Material{Restitution: units.Scalar(0), Friction: units.Scalar(.5)}
	cfg := StepConfig{Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}, TimeResolution: units.Seconds(1e-9),
		ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
		AngularVelocityResidual: units.RadiansPerSecond(1e-6),
		ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
		PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
		MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 2}
	contact, err := doc.ContactPair(t.Context(), floor, box, r3.Identity(), r3.Identity(), cfg.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 4)
	w, err := NewWorld(t.Context(), doc, WorldConfig{Bodies: []RigidBody{
		{Body: floor, Role: Fixed, Material: material},
		{Body: box, Role: Dynamic, Density: &density, Material: material},
	}, Step: cfg})
	require.NoError(t, err)
	zeroV := QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(0)}
	zeroW := QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
		Z: units.RadiansPerSecond(0)}
	start, err := w.NewState([]BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroV, AngularVelocity: zeroW},
		{Body: box, Pose: r3.Identity(), LinearVelocity: QuantityVec{
			X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroW},
	})
	require.NoError(t, err)
	gravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(-1000)}
	report, err := w.Step(t.Context(), start, StepInput{Gravity: gravity}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, Undecided, report.Status)
	require.Nil(t, report.Next)
	clearPose, err := r3.Translation(r3.Vec{Z: 20})
	require.NoError(t, err)
	clearStart, err := w.NewState([]BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroV, AngularVelocity: zeroW},
		{Body: box, Pose: clearPose, LinearVelocity: QuantityVec{
			X: units.MillimetersPerSecond(100), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroW},
	})
	require.NoError(t, err)
	zeroGravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
	clearReport, err := w.Step(t.Context(), clearStart, StepInput{Gravity: zeroGravity}, units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, Advanced, clearReport.Status, "%+v", clearReport.Diagnostics)
	require.Empty(t, clearReport.Events)
	impactPose, err := r3.Translation(r3.Vec{Z: 10})
	require.NoError(t, err)
	impactStart, err := w.NewState([]BodyState{
		{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroV, AngularVelocity: zeroW},
		{Body: box, Pose: impactPose, LinearVelocity: QuantityVec{
			X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(-100)}, AngularVelocity: zeroW},
	})
	require.NoError(t, err)
	impactReport, err := w.Step(t.Context(), impactStart,
		StepInput{Gravity: zeroGravity}, units.Seconds(.2))
	require.NoError(t, err)
	require.Equal(t, Undecided, impactReport.Status)
	require.Nil(t, impactReport.Next)
	unequal := Material{Restitution: units.Scalar(0), Friction: units.Scalar(.25)}
	_, err = NewWorld(t.Context(), doc, WorldConfig{Bodies: []RigidBody{
		{Body: floor, Role: Fixed, Material: material},
		{Body: box, Role: Dynamic, Density: &density, Material: unequal},
	}, Step: cfg})
	require.NoError(t, err)
	_, err = NewWorld(t.Context(), doc, WorldConfig{Bodies: []RigidBody{
		{Body: box, Role: Dynamic, Density: &density, Material: material},
		{Body: floor, Role: Fixed, Material: material},
	}, Step: cfg})
	require.NoError(t, err)
}
