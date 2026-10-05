package dynamics

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func grazingSphere(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	scene := sketch.NewWorld()
	profile, err := scene.CreateSketch(scene.XY())
	require.NoError(t, err)
	left, right, center := profile.CreatePoint(-5, 0), profile.CreatePoint(5, 0), profile.CreatePoint(0, 0)
	profile.Fix(left)
	profile.CreateLine(left, right)
	profile.CreateArc(center, right, left)
	_, err = profile.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Revolve(profile, profile.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

func TestGrazingEventConservationRejectsChangedState(t *testing.T) {
	doc := decad.New()
	fixed, moving := grazingSphere(t, doc), grazingSphere(t, doc)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	w, err := NewWorld(t.Context(), doc, WorldConfig{
		Bodies: []RigidBody{{Body: fixed, Role: Fixed, Material: material},
			{Body: moving, Role: Dynamic, Density: &density, Material: material}},
		Step: StepConfig{Contact: decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
			NormalResolution: units.Radians(1e-6)}, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-6),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096},
	})
	require.NoError(t, err)
	start, err := r3.Translation(r3.Vec{X: 20, Y: 10})
	require.NoError(t, err)
	zeroV := QuantityVec{X: units.MillimetersPerSecond(0), Y: units.MillimetersPerSecond(0),
		Z: units.MillimetersPerSecond(0)}
	motion := zeroV
	motion.X = units.MillimetersPerSecond(-40)
	zeroW := QuantityVec{X: units.RadiansPerSecond(0), Y: units.RadiansPerSecond(0),
		Z: units.RadiansPerSecond(0)}
	state, err := w.NewState([]BodyState{
		{Body: fixed, Pose: r3.Identity(), LinearVelocity: zeroV, AngularVelocity: zeroW},
		{Body: moving, Pose: start, LinearVelocity: motion, AngularVelocity: zeroW},
	})
	require.NoError(t, err)
	zeroG := QuantityVec{X: units.MillimetersPerSecondSquared(0),
		Y: units.MillimetersPerSecondSquared(0), Z: units.MillimetersPerSecondSquared(0)}
	report, err := w.Step(t.Context(), state, StepInput{Gravity: zeroG}, units.Seconds(1))
	require.NoError(t, err)
	require.Equal(t, Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	valid := report.Events[0]
	require.Empty(t, w.eventConservationFailure(valid))
	mutations := []struct {
		name   string
		change func(*ContactEvent)
	}{
		{"aggregate normal impulse", func(e *ContactEvent) {
			e.NormalImpulse = units.KilogramMillimetersPerSecond(1)
		}},
		{"aggregate tangent impulse", func(e *ContactEvent) {
			e.TangentImpulse.X = units.KilogramMillimetersPerSecond(1)
		}},
		{"point normal impulse", func(e *ContactEvent) {
			e.PointImpulses[0].Normal = units.KilogramMillimetersPerSecond(1)
		}},
		{"point tangent impulse", func(e *ContactEvent) {
			e.PointImpulses[0].Tangent.X = units.KilogramMillimetersPerSecond(1)
		}},
		{"linear velocity", func(e *ContactEvent) {
			e.PostVelocityB.X = units.MillimetersPerSecond(-39)
		}},
		{"reported velocity", func(e *ContactEvent) {
			e.PreVelocity.X = units.MillimetersPerSecond(-39)
			e.PostVelocity.X = units.MillimetersPerSecond(-39)
		}},
		{"angular velocity", func(e *ContactEvent) {
			e.PostAngularVelocityB.Z = units.RadiansPerSecond(1)
		}},
		{"position change", func(e *ContactEvent) {
			e.PositionChangeB.X = 1
		}},
		{"elapsed bracket endpoint", func(e *ContactEvent) {
			e.Bracket.To.Elapsed.Bound = units.Seconds(1)
		}},
		{"invalid pose", func(e *ContactEvent) {
			e.PoseB = r3.Transform{}
		}},
		{"unbounded point", func(e *ContactEvent) {
			e.Manifold.Points[0].OnA.Bound = units.Millimeters(1)
		}},
		{"missing point impulse", func(e *ContactEvent) {
			e.PointImpulses = nil
		}},
		{"solver report", func(e *ContactEvent) {
			e.Solver = &ContactSolverReport{}
		}},
	}
	for _, tc := range mutations {
		t.Run(tc.name, func(t *testing.T) {
			changed := valid
			changed.PointImpulses = append([]ContactPointImpulse(nil), valid.PointImpulses...)
			changed.Manifold = cloneManifold(valid.Manifold)
			tc.change(&changed)
			require.NotEmpty(t, w.eventConservationFailure(changed))
		})
	}
}
