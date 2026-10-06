package dynamics

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFixedFloorInteriorFrictionImpactUsesRealGeometry(t *testing.T) {
	t.Parallel()
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "floor first", true: "box first"}[reverse], func(t *testing.T) {
			doc := decad.New()
			floor := sourceBoxForFriction(t, doc, -100, -100, 100, 100, -10)
			box := sourceBoxForFriction(t, doc, -5, -5, 5, 5, 0)
			pose, err := r3.Translation(r3.Vec{Z: 10})
			require.NoError(t, err)
			request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)}
			a, b, poseA, poseB := floor, box, r3.Identity(), pose
			if reverse {
				a, b, poseA, poseB = b, a, poseB, poseA
			}
			contact, err := doc.ContactPair(t.Context(), a, b, poseA, poseB, request)
			require.NoError(t, err)
			require.Equal(t, decad.ContactSeparated, contact.Relation)
			density := units.KilogramsPerCubicMillimeter(.001)
			mass, err := box.MassProperties(t.Context(), density)
			require.NoError(t, err)
			zeroV := QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
				Z: units.MillimetersPerSecond(0)}
			zeroW := QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
				Z: units.RadiansPerSecond(0)}
			velocity := QuantityVec{X: units.MillimetersPerSecond(40), Y: units.MillimetersPerSecond(0),
				Z: units.MillimetersPerSecond(-160)}
			dt := units.Seconds(.125)
			floorPath := decad.RigidDriftSegment{From: r3.Identity(), LinearVelocity: zeroV,
				AngularVelocity: zeroW, Duration: dt}
			boxPath := decad.RigidDriftSegment{From: pose, Center: pose.Apply(mass.Center.Value),
				LinearVelocity: velocity, AngularVelocity: zeroW, Duration: dt}
			pathA, pathB := decad.PairPath(floorPath), decad.PairPath(boxPath)
			if reverse {
				pathA, pathB = pathB, pathA
			}
			sweep, err := doc.SweepPair(t.Context(), a, b, pathA, pathB,
				decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
					MaxPoseEvaluations: 128})
			require.NoError(t, err)
			require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
			require.NotNil(t, sweep.Event)
			require.Len(t, sweep.Event.Manifold.Points, 4)
			require.Equal(t, decad.ContactOverlapping, sweep.Event.Relation)
			material := Material{Restitution: units.Scalar(0), Friction: units.Scalar(.25)}
			definitions := []RigidBody{
				{Body: floor, Role: Fixed, Material: material},
				{Body: box, Role: Dynamic, Density: &density, Material: material},
			}
			states := []BodyState{
				{Body: floor, Pose: r3.Identity(), LinearVelocity: zeroV, AngularVelocity: zeroW},
				{Body: box, Pose: pose, LinearVelocity: velocity, AngularVelocity: zeroW},
			}
			if reverse {
				definitions[0], definitions[1] = definitions[1], definitions[0]
				states[0], states[1] = states[1], states[0]
			}
			cfg := StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
				ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
				AngularVelocityResidual: units.RadiansPerSecond(1e-6),
				ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
				PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
				MaxPoseEvaluations: 128, MaxIterations: 64, MaxEvents: 2, MaxPairSweeps: 4096}
			w, err := NewWorld(t.Context(), doc, WorldConfig{Bodies: definitions, Step: cfg})
			require.NoError(t, err)
			start, err := w.NewState(states)
			require.NoError(t, err)
			gravity := QuantityVec{X: units.MillimetersPerSecondSquared(0),
				Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
			report, err := w.Step(t.Context(), start, StepInput{Gravity: gravity}, dt)
			require.NoError(t, err)
			require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
			require.NotNil(t, report.Conservation)
			require.Len(t, report.Events, 1)
			event := report.Events[0]
			require.Equal(t, ContactImpact, event.Kind)
			require.InDelta(t, .0625, event.Time.Base(), 1e-9)
			require.InDelta(t, 160, event.NormalImpulse.Base(), 1e-6)
			tangent := -40.0
			if reverse {
				tangent = 40
			}
			require.InDelta(t, tangent, event.TangentImpulse.X.Base(), 1e-6)
			require.Len(t, event.PointImpulses, 4)
			require.NotNil(t, event.Solver)
			require.LessOrEqual(t, event.Solver.AngularUpper.Base(), 1e-6)
			var summedNormal, summedTangent float64
			for _, point := range event.PointImpulses {
				summedNormal += point.Normal.Base()
				summedTangent += point.Tangent.X.Base()
				require.LessOrEqual(t, math.Abs(point.Tangent.X.Base()), .25*point.Normal.Base()+1e-6)
			}
			require.InDelta(t, event.NormalImpulse.Base(), summedNormal, 1e-6)
			require.InDelta(t, event.TangentImpulse.X.Base(), summedTangent, 1e-6)
			require.InDelta(t, -40, report.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
			require.InDelta(t, 160, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
			boxEnd, ok := report.Next.Body(box)
			require.True(t, ok)
			require.Equal(t, zeroV, boxEnd.LinearVelocity)
			require.Equal(t, zeroW, boxEnd.AngularVelocity)
			require.InDelta(t, 2.5, boxEnd.Pose.Translation().X, 1e-6)
			require.InDelta(t, 0, boxEnd.Pose.Translation().Z, 1e-6)
			for _, sampleTime := range []float64{.03125, .1, .125} {
				sample, sampleErr := report.Trace.Sample(units.Seconds(sampleTime))
				require.NoError(t, sampleErr)
				boxSample, exists := sample.Body(box)
				require.True(t, exists)
				if sampleTime < event.Time.Base() {
					require.Equal(t, velocity, boxSample.LinearVelocity)
					require.InDelta(t, 10-160*sampleTime, boxSample.Pose.Translation().Z, 1e-6)
				} else {
					require.Equal(t, zeroV, boxSample.LinearVelocity)
					require.InDelta(t, 0, boxSample.Pose.Translation().Z, 1e-6)
				}
			}
			limited := cfg
			limited.MaxEvents = 1
			limitedWorld, err := NewWorld(t.Context(), doc, WorldConfig{Bodies: definitions, Step: limited})
			require.NoError(t, err)
			limitedState, err := limitedWorld.NewState(states)
			require.NoError(t, err)
			limitedStep, err := limitedWorld.Step(t.Context(), limitedState, StepInput{Gravity: gravity}, dt)
			require.NoError(t, err)
			require.Equal(t, Undecided, limitedStep.Status)
			require.Nil(t, limitedStep.Next)
			bouncy := append([]RigidBody(nil), definitions...)
			for i := range bouncy {
				bouncy[i].Material.Restitution = units.Scalar(.5)
			}
			bouncyWorld, err := NewWorld(t.Context(), doc, WorldConfig{Bodies: bouncy, Step: cfg})
			require.NoError(t, err)
			bouncyState, err := bouncyWorld.NewState(states)
			require.NoError(t, err)
			bouncyStep, err := bouncyWorld.Step(t.Context(), bouncyState, StepInput{Gravity: gravity}, dt)
			require.NoError(t, err)
			// Restitution 0.5 sends the box up at 80 mm/s with 1.5·160 kg·mm/s;
			// friction 0.25·240 could take 60, and 40 stops the slide.
			require.Equal(t, Advanced, bouncyStep.Status, "%+v", bouncyStep.Diagnostics)
			require.Len(t, bouncyStep.Events, 1)
			require.InDelta(t, .0625, bouncyStep.Events[0].Time.Base(), 1e-9)
			require.InDelta(t, 240, bouncyStep.Events[0].NormalImpulse.Base(), 1e-6)
			require.InDelta(t, tangent, bouncyStep.Events[0].TangentImpulse.X.Base(), 1e-6)
			bounced, ok := bouncyStep.Next.Body(box)
			require.True(t, ok)
			require.Zero(t, bounced.LinearVelocity.X.Base())
			require.InDelta(t, 80, bounced.LinearVelocity.Z.Base(), 1e-6)
			require.Equal(t, zeroW, bounced.AngularVelocity)
		})
	}
}
