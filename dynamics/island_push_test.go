package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Legs of the separating push (docs/multibody-dynamics-design.md §6.6),
// each shown to fail by deleting it in island.go, watching the named test go
// red, and restoring it:
//   - the push itself: TestIslandPushSeparatesACurvedBounce refuses with
//     "corrected pair relation is 3, not touching";
//   - the doubling that makes a sub-ulp push move the rounded pose: the same
//     refusal;
//   - the separating requirement, which keeps a pair the solve does not
//     separate both from the push and from ending separated: deleting the
//     two conditions together lets TestIslandPushLeavesARestingPairAlone
//     advance with the resting sphere an ulp off the face. Each condition
//     alone is the other's backstop: an unpushed resting pair is still
//     overlapping, and a pushed one is refused as separated;
//   - the correction allowance: TestCorrectedRelationPushRespectsAllowance
//     (island_push_internal_test.go) pushes a sphere past an allowance it
//     does not fit. The bounce fixtures cannot reach that leg: their
//     allowance carries the bracket travel, about 1e-7 mm, and their push is
//     an ulp.
//
// pushLimit bounds the passes over an event's islands and is not a gate:
// without it a pair that keeps overlapping is pushed until the allowance
// refuses it, so deleting it never admits a pose.

// pushScene is the rotated-face bounce of TestSourceSphereRotatedBoxFace as
// a four-body world: a 1 kg, 5 mm sphere leaves 20 mm along the normal of a
// fixed box face tilted 45° about Y and approaches it at 100 mm/s with
// restitution 0.5; two fixed boxes 1000 and 2000 mm away along X complete
// the world. The sphere touches the face at 0.15/1 of the 0.06 s step and
// leaves at 50 mm/s.
type pushScene struct {
	doc         *decad.Document
	face, ball  *decad.Body
	world       *dynamics.World
	state       dynamics.State
	normal      r3.Vec
	config      dynamics.StepConfig
	initialPose r3.Transform
}

func newPushScene(t *testing.T, restitution float64, config dynamics.StepConfig) pushScene {
	t.Helper()
	scene := pushScene{doc: decad.New(), config: config}
	scene.face = makeBox(t, scene.doc, -20, -20, 20, 20, -10, 20)
	scene.ball = makeBall(t, scene.doc)
	far, farther := farBox(t, scene.doc, 1000), farBox(t, scene.doc, 2000)
	turn, err := r3.Rotation(r3.Vec{Y: 1}, units.Degrees(45))
	require.NoError(t, err)
	scene.normal = turn.ApplyDir(r3.Vec{Z: 1})
	scene.initialPose, err = r3.Translation(scene.normal.Scale(20))
	require.NoError(t, err)
	material := dynamics.Material{Restitution: units.Scalar(restitution), Friction: units.Scalar(0)}
	mass := exactSphereMass()
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: scene.face, Role: dynamics.Fixed, Material: material},
		{Body: scene.ball, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		{Body: far, Role: dynamics.Fixed, Material: material},
		{Body: farther, Role: dynamics.Fixed, Material: material},
	}, Step: config})
	require.NoError(t, err)
	velocity := dynamics.QuantityVec{X: units.MillimetersPerSecond(-100 * scene.normal.X),
		Y: units.MillimetersPerSecond(-100 * scene.normal.Y), Z: units.MillimetersPerSecond(-100 * scene.normal.Z)}
	scene.state, err = scene.world.NewState([]dynamics.BodyState{
		{Body: scene.face, Pose: turn, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: scene.ball, Pose: scene.initialPose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
		{Body: far, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: farther, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	return scene
}

func (s pushScene) step(t *testing.T) *dynamics.StepReport {
	t.Helper()
	report, err := s.world.Step(t.Context(), s.state, dynamics.StepInput{Gravity: zeroAcceleration()},
		units.Seconds(.06))
	require.NoError(t, err)
	return report
}

// TestIslandPushSeparatesACurvedBounce asserts the closed form's numbers:
// the sphere leaves along the normal at 50 mm/s, is 17 mm out along the
// normal at 0.03 s and 15.25 mm out at 0.055 s. The correction that removes
// the bracket's overlap leaves the sphere overlapping the tilted face by
// about an ulp; the push leaves it just separated, by less than its
// correction allowance, and the pair then drifts clear.
func TestIslandPushSeparatesACurvedBounce(t *testing.T) {
	scene := newPushScene(t, .5, pairMaterialStepConfig())
	report := scene.step(t)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Equal(t, dynamics.BodyPair{A: scene.face, B: scene.ball}, event.Pair)
	require.InDelta(t, 150, event.NormalImpulse.Base(), 1e-6)
	final, ok := report.Next.Body(scene.ball)
	require.True(t, ok)
	require.InDelta(t, 50*scene.normal.X, final.LinearVelocity.X.Base(), 1e-6)
	require.InDelta(t, 50*scene.normal.Z, final.LinearVelocity.Z.Base(), 1e-6)
	before, err := report.Trace.Sample(units.Seconds(.03))
	require.NoError(t, err)
	beforeBall, ok := before.Body(scene.ball)
	require.True(t, ok)
	require.InDelta(t, 17, beforeBall.Pose.Translation().Dot(scene.normal), 1e-6)
	after, err := report.Trace.Sample(units.Seconds(.055))
	require.NoError(t, err)
	afterBall, ok := after.Body(scene.ball)
	require.True(t, ok)
	require.InDelta(t, 15.25, afterBall.Pose.Translation().Dot(scene.normal), 1e-5)
	// The event's post pose is the corrected and pushed one: separated from
	// the face, by a gap far below the correction allowance (ContactSlop
	// 1e-6 mm plus the bracket travel, about 1e-7 mm here).
	post, err := report.Trace.Sample(event.Time)
	require.NoError(t, err)
	postBall, ok := post.Body(scene.ball)
	require.True(t, ok)
	facePose, ok := post.Body(scene.face)
	require.True(t, ok)
	contact, err := scene.doc.ContactPair(t.Context(), scene.face, scene.ball, facePose.Pose, postBall.Pose,
		scene.config.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	require.NotNil(t, contact.Gap)
	require.Less(t, contact.Gap.Value.Base(), scene.config.ContactSlop.Base())
	moved := event.PositionChangeB
	require.Positive(t, moved.Dot(scene.normal))
	require.Less(t, moved.Dot(scene.normal), scene.config.ContactSlop.Base())
}

// TestCorrectedRelationPushRespectsAllowance places the bounce's sphere
// 2^-40 mm (about 9.1e-13 mm) into the tilted face, a shallow overlap with
// a bounded manifold, and pushes it apart as a separating pair. The push
// needs about that much travel: an allowance of 1e-11 mm admits it and
// leaves the pair apart, one of 1e-13 mm refuses it.
func TestCorrectedRelationPushRespectsAllowance(t *testing.T) {
	scene := newPushScene(t, .5, pairMaterialStepConfig())
	sunk, err := r3.Translation(scene.normal.Scale(15 - 0x1p-40))
	require.NoError(t, err)
	var entries []dynamics.BodyState
	for _, entry := range scene.state.Entries() {
		if entry.Body == scene.ball {
			entry.Pose = sunk
		}
		entries = append(entries, entry)
	}
	state, err := scene.world.NewState(entries)
	require.NoError(t, err)
	face, _ := state.Body(scene.face)
	contact, err := scene.doc.ContactPair(t.Context(), scene.face, scene.ball, face.Pose, sunk, scene.config.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactOverlapping, contact.Relation)
	reason, pushed, err := dynamics.PushApart(t.Context(), scene.world, state, state, scene.face, scene.ball,
		scene.ball, units.Millimeters(1e-11))
	require.NoError(t, err)
	require.Empty(t, reason)
	ball, _ := pushed.Body(scene.ball)
	contact, err = scene.doc.ContactPair(t.Context(), scene.face, scene.ball, face.Pose, ball.Pose, scene.config.Contact)
	require.NoError(t, err)
	require.NotEqual(t, decad.ContactOverlapping, contact.Relation)
	moved := ball.Pose.Translation().Sub(sunk.Translation()).Dot(scene.normal)
	require.InDelta(t, 0x1p-40, moved, 0x1p-42)
	reason, _, err = dynamics.PushApart(t.Context(), scene.world, state, state, scene.face, scene.ball,
		scene.ball, units.Millimeters(1e-13))
	require.NoError(t, err)
	require.Contains(t, reason, "exceeds its correction allowance")
}

// TestIslandPushLeavesARestingPairAlone runs the bounce with no restitution:
// the sphere stops on the tilted face, so the solve does not separate the
// pair, and the correction's ulp of overlap cannot be pushed away without
// breaking the exact touch its continuation needs. The step refuses with
// StepCorrectionFailed.
func TestIslandPushLeavesARestingPairAlone(t *testing.T) {
	scene := newPushScene(t, 0, pairMaterialStepConfig())
	report := scene.step(t)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	t.Logf("%+v", report.Diagnostics)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepCorrectionFailed, report.Diagnostics[0].Code, "%+v", report.Diagnostics)
	require.Contains(t, report.Diagnostics[0].Reason, "not touching")
}
