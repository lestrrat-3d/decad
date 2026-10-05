package dynamics_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func makeCylinder(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.CreateCircle(center, 5)
	s.Fix(center)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	cylinder, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	})
	require.NoError(t, err)
	return cylinder
}

func TestCylinderClearStepUsesProductionSweep(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(map[bool]string{false: "floor first", true: "cylinder first"}[reverse], func(t *testing.T) {
			doc := decad.New()
			floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
			cylinder := makeCylinder(t, doc)
			pose, err := r3.Translation(r3.Vec{Z: 1})
			require.NoError(t, err)
			request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
				NormalResolution: units.Radians(1e-6)}
			a, b := floor, cylinder
			pa, pb := r3.Identity(), pose
			if reverse {
				a, b, pa, pb = b, a, pb, pa
			}
			contact, err := doc.ContactPair(t.Context(), a, b, pa, pb, request)
			require.NoError(t, err)
			require.Equal(t, decad.ContactSeparated, contact.Relation)
			zeroAngular := decad.QuantityVec{X: units.RadiansPerSecond(0),
				Y: units.RadiansPerSecond(0), Z: units.RadiansPerSecond(0)}
			zeroLinear := zeroVelocity()
			up := decad.QuantityVec{X: units.MillimetersPerSecond(0),
				Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(1)}
			floorPath := decad.RigidDriftSegment{From: r3.Identity(), LinearVelocity: zeroLinear,
				AngularVelocity: zeroAngular, Duration: units.Seconds(1)}
			cylinderPath := decad.RigidDriftSegment{From: pose, LinearVelocity: up,
				AngularVelocity: zeroAngular, Duration: units.Seconds(1)}
			pathA, pathB := decad.PairPath(floorPath), decad.PairPath(cylinderPath)
			if reverse {
				pathA, pathB = pathB, pathA
			}
			sweep, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, decad.SweepRequest{
				ContactRequest: request, TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
			require.NoError(t, err)
			require.Equal(t, decad.SweepClear, sweep.Outcome)
			require.True(t, sweep.HasAffineReplayProof())
			replayA, replayB, err := sweep.CertifiedPosesAt(units.Seconds(.5))
			require.NoError(t, err)
			cylinderReplay := replayB
			if reverse {
				cylinderReplay = replayA
			}
			require.InDelta(t, 1.5, cylinderReplay.Translation().Z, 1e-9)
			material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
			density := units.KilogramsPerCubicMillimeter(0.001)
			mass, err := cylinder.MassProperties(t.Context(), density)
			require.NoError(t, err)
			require.InDelta(t, math.Pi*25*10*density.Base(), mass.Mass.Value.Base(), 1e-12)
			require.InDelta(t, 0.5*mass.Mass.Value.Base()*25, mass.Inertia.ZZ.Value.Base(), 1e-12)
			require.InDelta(t, mass.Mass.Value.Base()*175/12, mass.Inertia.XX.Value.Base(), 1e-12)
			require.InDelta(t, mass.Inertia.XX.Value.Base(), mass.Inertia.YY.Value.Base(), 1e-12)
			require.LessOrEqual(t, math.Abs(mass.Inertia.XY.Value.Base()), mass.Inertia.XY.Bound.Base())
			require.Less(t, mass.Mass.Bound.Base(), mass.Mass.Value.Base())
			require.InDelta(t, 5, mass.Center.Value.Z, 1e-12)
			defs := []dynamics.RigidBody{{Body: floor, Role: dynamics.Fixed, Material: material},
				{Body: cylinder, Role: dynamics.Dynamic, Density: &density, Material: material}}
			states := []dynamics.BodyState{{Body: floor, Pose: r3.Identity(),
				LinearVelocity: zeroLinear, AngularVelocity: zeroAngular},
				{Body: cylinder, Pose: pose, LinearVelocity: up, AngularVelocity: zeroAngular}}
			if reverse {
				defs[0], defs[1] = defs[1], defs[0]
				states[0], states[1] = states[1], states[0]
			}
			world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: defs,
				Step: dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
					ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
					AngularVelocityResidual: units.RadiansPerSecond(1e-6),
					ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
					PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
					MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2}})
			require.NoError(t, err)
			start, err := world.NewState(states)
			require.NoError(t, err)
			step, err := world.Step(t.Context(), start, dynamics.StepInput{Gravity: zeroAcceleration()},
				units.Seconds(1))
			require.NoError(t, err)
			require.Equal(t, dynamics.Advanced, step.Status, "%+v", step.Diagnostics)
			require.Empty(t, step.Events)
			require.NotNil(t, step.Conservation)
			require.InDelta(t, mass.Mass.Value.Base(),
				step.Conservation.Input.LinearMomentum.Value.Z.Base(), 1e-12)
			require.InDelta(t, mass.Mass.Value.Base()/2,
				step.Conservation.Input.KineticEnergy.Value.Base(), 1e-12)
			final, ok := step.Next.Body(cylinder)
			require.True(t, ok)
			require.InDelta(t, 2, final.Pose.Translation().Z, 1e-9)
			mid, err := step.Trace.Sample(units.Seconds(.5))
			require.NoError(t, err)
			middle, ok := mid.Body(cylinder)
			require.True(t, ok)
			require.InDelta(t, 1.5, middle.Pose.Translation().Z, 1e-9)
		})
	}
}

func TestCylinderClearSweepRefusesNearContactAndCrossing(t *testing.T) {
	doc := decad.New()
	floor := makeBox(t, doc, -20, -20, 20, 20, -10, 10)
	cylinder := makeCylinder(t, doc)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	zero := decad.QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
		Z: units.RadiansPerSecond(0)}
	stationary := decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: units.Seconds(1)}
	for _, tc := range []struct {
		name     string
		x        float64
		start    float64
		velocity float64
	}{
		{name: "below point resolution", start: 1e-7, velocity: 1},
		{name: "crosses floor", start: 1, velocity: -2},
		{name: "outside floor face", x: 30, start: 1, velocity: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			pose, err := r3.Translation(r3.Vec{X: tc.x, Z: tc.start})
			require.NoError(t, err)
			if tc.x != 0 {
				contact, err := doc.ContactPair(t.Context(), floor, cylinder, r3.Identity(), pose, request)
				require.NoError(t, err)
				require.Equal(t, decad.ContactUndecided, contact.Relation)
			}
			velocity := decad.QuantityVec{X: units.MillimetersPerSecond(0),
				Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(tc.velocity)}
			moving := decad.RigidDriftSegment{From: pose, LinearVelocity: velocity,
				AngularVelocity: zero, Duration: units.Seconds(1)}
			for _, reverse := range []bool{false, true} {
				a, b := floor, cylinder
				pathA, pathB := decad.PairPath(stationary), decad.PairPath(moving)
				if reverse {
					a, b, pathA, pathB = b, a, pathB, pathA
				}
				sweep, err := doc.SweepPair(t.Context(), a, b, pathA, pathB, decad.SweepRequest{
					ContactRequest: request, TimeResolution: units.Seconds(1e-9), MaxPoseEvaluations: 128})
				require.NoError(t, err)
				require.Equal(t, decad.SweepUndecided, sweep.Outcome)
				require.False(t, sweep.HasAffineReplayProof())
			}
		})
	}
}
