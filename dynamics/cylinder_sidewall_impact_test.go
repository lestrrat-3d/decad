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

func TestCylinderSidewallImpactUsesRealContactSweepMassAndTrace(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	wall := makeBox(t, doc, -10, -20, 0, 20, -10, 30)
	cylinder := makeCylinder(t, doc)
	startPose, err := r3.Translation(r3.Vec{X: 6})
	require.NoError(t, err)
	touchPose, err := r3.Translation(r3.Vec{X: 5})
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	startContact, err := doc.ContactPair(t.Context(), wall, cylinder,
		r3.Identity(), startPose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, startContact.Relation)
	require.InDelta(t, 1, startContact.Gap.Value.Base(), 1e-12)
	touchContact, err := doc.ContactPair(t.Context(), wall, cylinder,
		r3.Identity(), touchPose, request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, touchContact.Relation)
	require.Len(t, touchContact.Manifold.Points, 1)
	point := touchContact.Manifold.Points[0]
	require.Contains(t, wall.Faces(), point.FaceA)
	require.Contains(t, cylinder.Faces(), point.FaceB)
	require.IsType(t, decad.Cylinder{}, point.FaceB.Surface())
	require.Equal(t, r3.Vec{X: 1}, point.Normal.Value)
	require.Equal(t, r3.Vec{X: 0, Y: 0, Z: 5}, point.OnA.Value)
	require.Equal(t, point.OnA.Value, point.OnB.Value)
	zero := zeroAngular(t)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(-10),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	duration := units.Seconds(.2)
	sweep, err := doc.SweepPair(t.Context(), wall, cylinder,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration},
		decad.RigidDriftSegment{From: startPose, LinearVelocity: velocity,
			AngularVelocity: zero, Duration: duration},
		decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
	require.True(t, sweep.HasAffineReplayProof())
	require.Len(t, sweep.Event.Manifold.Points, 1)
	require.Same(t, point.FaceB, sweep.Event.Manifold.Points[0].FaceB)
	require.InDelta(t, .1, sweep.Bracket.To.Elapsed.Value.Base(), 1e-9)
	_, _, err = sweep.CertifiedPosesAt(units.Seconds(.05))
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(.001)
	mass, err := cylinder.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.InDelta(t, .25*math.Pi, mass.Mass.Value.Base(), 1e-12)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: wall, Role: dynamics.Fixed, Material: material},
			{Body: cylinder, Role: dynamics.Dynamic, Density: &density, Material: material}},
		Step: dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-5),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096}})
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: wall, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zero},
		{Body: cylinder, Pose: startPose, LinearVelocity: velocity, AngularVelocity: zero},
	})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), state,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.InDelta(t, 15*mass.Mass.Value.Base(), report.Events[0].NormalImpulse.Base(), 1e-5)
	for _, sample := range []struct{ at, x, vx float64 }{
		{.05, 5.5, -10}, {.15, 5.25, 5}, {.2, 5.5, 5},
	} {
		at, sampleErr := report.Trace.Sample(units.Seconds(sample.at))
		require.NoError(t, sampleErr)
		body, ok := at.Body(cylinder)
		require.True(t, ok)
		require.InDelta(t, sample.x, body.Pose.Translation().X, 1e-6)
		require.InDelta(t, sample.vx, body.LinearVelocity.X.Base(), 1e-6)
	}
}

func TestCylinderSidewallImpactReversesPairNormal(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	wall := makeBox(t, doc, -10, -20, 0, 20, -10, 30)
	cylinder := makeCylinder(t, doc)
	startPose, err := r3.Translation(r3.Vec{X: 6})
	require.NoError(t, err)
	touchPose, err := r3.Translation(r3.Vec{X: 5})
	require.NoError(t, err)
	request := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	contact, err := doc.ContactPair(t.Context(), cylinder, wall,
		touchPose, r3.Identity(), request)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	require.Len(t, contact.Manifold.Points, 1)
	require.Equal(t, r3.Vec{X: -1}, contact.Manifold.Points[0].Normal.Value)
	require.IsType(t, decad.Cylinder{}, contact.Manifold.Points[0].FaceA.Surface())
	zero := zeroAngular(t)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(-10),
		Y: units.MillimetersPerSecond(0), Z: units.MillimetersPerSecond(0)}
	duration := units.Seconds(.2)
	sweep, err := doc.SweepPair(t.Context(), cylinder, wall,
		decad.RigidDriftSegment{From: startPose, LinearVelocity: velocity,
			AngularVelocity: zero, Duration: duration},
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration},
		decad.SweepRequest{ContactRequest: request, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome, "cause=%v", sweep.Cause)
	require.Equal(t, r3.Vec{X: -1}, sweep.Event.Manifold.Points[0].Normal.Value)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{
		Bodies: []dynamics.RigidBody{{Body: cylinder, Role: dynamics.Dynamic, Density: &density,
			Material: material}, {Body: wall, Role: dynamics.Fixed, Material: material}},
		Step: dynamics.StepConfig{Contact: request, TimeResolution: units.Seconds(1e-9),
			ContactSlop: units.Millimeters(1e-6), VelocityResidual: units.MillimetersPerSecond(1e-6),
			AngularVelocityResidual: units.RadiansPerSecond(1e-6),
			ImpulseResidual:         units.KilogramMillimetersPerSecond(1e-5),
			PenetrationResidual:     units.Millimeters(1e-6), ImpactSpeed: units.MillimetersPerSecond(0),
			MaxPoseEvaluations: 128, MaxIterations: 8, MaxEvents: 2, MaxPairSweeps: 4096}})
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: cylinder, Pose: startPose, LinearVelocity: velocity, AngularVelocity: zero},
		{Body: wall, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zero},
	})
	require.NoError(t, err)
	report, err := world.Step(t.Context(), state,
		dynamics.StepInput{Gravity: zeroAcceleration()}, duration)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	require.Equal(t, r3.Vec{X: -1}, report.Events[0].Manifold.Points[0].Normal.Value)
	sample, err := report.Trace.Sample(units.Seconds(.15))
	require.NoError(t, err)
	entry, ok := sample.Body(cylinder)
	require.True(t, ok)
	require.InDelta(t, 5.25, entry.Pose.Translation().X, 1e-6)
	require.InDelta(t, 5, entry.LinearVelocity.X.Base(), 1e-6)
}

// requireRulingEnds checks a ruling touch whose two published ends lie on
// both bodies exactly.
func requireRulingEnds(t *testing.T, report *decad.ContactReport, ends [2]r3.Vec) {
	t.Helper()
	require.Equal(t, decad.ContactTouching, report.Relation, "reason=%v", report.Reason)
	require.NotNil(t, report.Manifold)
	require.Len(t, report.Manifold.Points, 2)
	for i, point := range report.Manifold.Points {
		require.Equal(t, ends[i], point.OnA.Value)
		require.Equal(t, ends[i], point.OnB.Value)
		require.Zero(t, point.OnA.Bound.Base())
	}
}

func TestCylinderSidewallRefusesUnprovedCorridors(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	narrowWall := makeBox(t, doc, -10, -5, 0, 5, -10, 30)
	shortWall := makeBox(t, doc, -10, -20, 0, 20, -10, 15)
	broadWall := makeBox(t, doc, -10, -20, 0, 20, -10, 30)
	cylinder := makeCylinder(t, doc)
	req := decad.ContactRequest{PointResolution: units.Millimeters(1e-6),
		NormalResolution: units.Radians(1e-6)}
	touch, err := r3.Translation(r3.Vec{X: 5})
	require.NoError(t, err)
	// The sidewall corridor needs the transverse diameter strictly inside
	// the wall face, which the narrow wall's edges meet; the placed ruling
	// (docs/contact-geometry-design.md §4.5) needs only the contact ruling
	// inside it, so it proves the touch along x = 0, z ∈ [0, 10].
	nearEdge, err := doc.ContactPair(t.Context(), narrowWall, cylinder,
		r3.Identity(), touch, req)
	require.NoError(t, err)
	requireRulingEnds(t, nearEdge, [2]r3.Vec{{}, {Z: 10}})
	// A wall whose face ends at z = 5 holds only half the ruling.
	overhang, err := doc.ContactPair(t.Context(), shortWall, cylinder,
		r3.Identity(), touch, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactUndecided, overhang.Relation)
	require.Nil(t, overhang.Manifold)
	start, err := r3.Translation(r3.Vec{X: 6})
	require.NoError(t, err)
	duration := units.Seconds(.2)
	zero := zeroAngular(t)
	verticalDrift, err := doc.SweepPair(t.Context(), broadWall, cylinder,
		decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: duration},
		decad.RigidDriftSegment{From: start, LinearVelocity: dynamics.QuantityVec{
			X: units.MillimetersPerSecond(-10), Y: units.MillimetersPerSecond(0),
			Z: units.MillimetersPerSecond(1)}, AngularVelocity: zero, Duration: duration},
		decad.SweepRequest{ContactRequest: req, TimeResolution: units.Seconds(1e-9),
			MaxPoseEvaluations: 128})
	require.NoError(t, err)
	require.Equal(t, decad.SweepUndecided, verticalDrift.Outcome)
	require.Equal(t, decad.SweepContactUnsupported, verticalDrift.Cause)

	// A revolved sidewall has no box corridor; the placed ruling proves its
	// touch along y = 0, z = 0, x ∈ [0, 10], and refuses a wall face that
	// starts at x = 2.
	revolvedWall := makeBox(t, doc, -10, -10, 20, 0, -20, 40)
	shortRevolvedWall := makeBox(t, doc, 2, -10, 20, 0, -20, 40)
	revolved := makeRevolvedCylinder(t, doc, decad.FullRevolution{}, 0)
	revolvedTouch, err := r3.Translation(r3.Vec{Y: 5})
	require.NoError(t, err)
	proved, err := doc.ContactPair(t.Context(), revolvedWall, revolved,
		r3.Identity(), revolvedTouch, req)
	require.NoError(t, err)
	requireRulingEnds(t, proved, [2]r3.Vec{{}, {X: 10}})
	unproved, err := doc.ContactPair(t.Context(), shortRevolvedWall, revolved,
		r3.Identity(), revolvedTouch, req)
	require.NoError(t, err)
	require.Equal(t, decad.ContactUndecided, unproved.Relation)
	require.Nil(t, unproved.Manifold)
}
