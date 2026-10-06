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

func TestGrazingSpherePassesWithZeroImpulse(t *testing.T) {
	t.Parallel()
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
	// The sphere passes the fixed sphere's equator tangentially: a graze
	// with no impulse, after which it keeps moving at −40 mm/s.
	graze := report.Events[0]
	require.Equal(t, ContactGraze, graze.Kind)
	require.Zero(t, graze.NormalImpulse.Base())
	require.Len(t, graze.Manifold.Points, 1)
	end, ok := report.Next.Body(moving)
	require.True(t, ok)
	require.Equal(t, motion, end.LinearVelocity)
	require.InDelta(t, -20, end.Pose.Translation().X, 1e-9)
	require.InDelta(t, 10, end.Pose.Translation().Y, 1e-9)
}
