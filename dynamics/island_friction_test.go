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

// These tests drive the Coulomb rows of the island solver
// (docs/multibody-dynamics-design.md §6.2–§6.4, §13 PR 5) through the real
// producers in a world of four bodies.

// frictionSlideScene is the four-corner Coulomb slide of
// friction_step_test.go run through the general path: a fixed 200×200×10 mm
// floor with its top at z = 0, a 10 mm dynamic box (1 kg at density
// 0.001 kg/mm³) resting on it, and two fixed boxes 500 and 1000 mm away along
// X that the broad phase excludes. Restitution is zero and friction 0.5.
// Gravity −1000 mm/s² over dt = 0.1 s kicks the box by −100 mm/s, so the
// floor delivers a normal impulse of 100 kg·mm/s and friction removes at most
// 50 kg·mm/s of a 100 mm/s slide.
type frictionSlideScene struct {
	doc        *decad.Document
	floor, box *decad.Body
	far        [2]*decad.Body
	world      *dynamics.World
	config     dynamics.StepConfig
}

func newFrictionSlide(t *testing.T, boxFirst bool, config dynamics.StepConfig) frictionSlideScene {
	t.Helper()
	scene := frictionSlideScene{doc: decad.New(), config: config}
	scene.floor = makeBox(t, scene.doc, -100, -100, 100, 100, -10, 10)
	scene.box = makeBox(t, scene.doc, -5, -5, 5, 5, 0, 10)
	scene.far = [2]*decad.Body{makeBox(t, scene.doc, 495, -5, 505, 5, 0, 10),
		makeBox(t, scene.doc, 995, -5, 1005, 5, 0, 10)}
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(.5)}
	floor := dynamics.RigidBody{Body: scene.floor, Role: dynamics.Fixed, Material: material}
	box := dynamics.RigidBody{Body: scene.box, Role: dynamics.Dynamic, Density: &density, Material: material}
	bodies := []dynamics.RigidBody{floor, box}
	if boxFirst {
		bodies = []dynamics.RigidBody{box, floor}
	}
	for _, far := range scene.far {
		bodies = append(bodies, dynamics.RigidBody{Body: far, Role: dynamics.Fixed, Material: material})
	}
	var err error
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: bodies, Step: config})
	require.NoError(t, err)
	return scene
}

// state starts the box at rest on the floor, sliding along X at slip mm/s.
func (s frictionSlideScene) state(t *testing.T, slip float64) dynamics.State {
	t.Helper()
	var entries []dynamics.BodyState
	for _, body := range s.world.Bodies() {
		entry := dynamics.BodyState{Body: body.Body, Pose: r3.Identity(), LinearVelocity: zeroVelocity(),
			AngularVelocity: zeroAngular(t)}
		if body.Body == s.box {
			entry.LinearVelocity.X = units.MillimetersPerSecond(slip)
		}
		entries = append(entries, entry)
	}
	state, err := s.world.NewState(entries)
	require.NoError(t, err)
	return state
}

func (s frictionSlideScene) step(t *testing.T, slip float64) *dynamics.StepReport {
	t.Helper()
	report, err := s.world.Step(t.Context(), s.state(t, slip), dynamics.StepInput{Gravity: gravityZ(-1000)},
		units.Seconds(.1))
	require.NoError(t, err)
	return report
}

// TestIslandFrictionSlideThroughGeneralPath asserts the numbers
// TestFixedFloorFrictionStepUsesRealGeometry asserts for the two-body world,
// here produced by the island solver's Coulomb rows: the slide keeps half its
// speed, every corner slips on the cone, and the box then slides 5 mm on a
// persistent track.
func TestIslandFrictionSlideThroughGeneralPath(t *testing.T) {
	t.Parallel()
	scene := newFrictionSlide(t, false, pairMaterialStepConfig())
	report := scene.step(t, 100)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Next)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, 5000, report.Conservation.Input.KineticEnergy.Value.Base(), 1e-6)
	require.InDelta(t, 10000, report.Conservation.AfterKick.KineticEnergy.Value.Base(), 1e-6)
	require.InDelta(t, 1250, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-5)
	require.InDelta(t, -100, report.Conservation.GravityImpulse.Value.Z.Base(), 1e-6)
	require.InDelta(t, -50, report.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
	require.InDelta(t, 100, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	require.Len(t, report.Events, 1)
	require.Len(t, report.Islands, 1)
	event := report.Events[0]
	require.Equal(t, dynamics.ContactImpact, event.Kind)
	require.Equal(t, dynamics.BodyPair{A: scene.floor, B: scene.box}, event.Pair)
	require.Zero(t, event.Time.Base())
	require.InDelta(t, 100, event.NormalImpulse.Base(), 1e-6)
	require.InDelta(t, -50, event.TangentImpulse.X.Base(), 1e-6)
	require.LessOrEqual(t, math.Abs(event.TangentImpulse.Y.Base()), scene.config.ImpulseResidual.Base())
	require.Zero(t, event.TangentImpulse.Z.Base())
	require.Equal(t, units.Impulse, event.TangentImpulse.X.Kind())
	require.NotNil(t, event.Solver)
	require.Positive(t, event.Solver.Iterations)
	require.LessOrEqual(t, event.Solver.ConeResidual.Base(), scene.config.ImpulseResidual.Base())
	require.Len(t, event.PointImpulses, len(event.Manifold.Points))
	require.Len(t, event.PointImpulses, 4)
	var pointNormal, pointTangent float64
	for i, impulse := range event.PointImpulses {
		pointNormal += impulse.Normal.Base()
		pointTangent += impulse.Tangent.X.Base()
		require.Equal(t, units.Impulse, impulse.Tangent.Y.Kind())
		// Every corner slips, so its friction lies on the cone and opposes the
		// slide.
		require.InDelta(t, .5*impulse.Normal.Base(), -impulse.Tangent.X.Base(), 1e-6, "point %d", i)
	}
	require.InDelta(t, event.NormalImpulse.Base(), pointNormal, 1e-6)
	require.InDelta(t, event.TangentImpulse.X.Base(), pointTangent, 1e-6)
	final, ok := report.Next.Body(scene.box)
	require.True(t, ok)
	require.InDelta(t, 50, final.LinearVelocity.X.Base(), 1e-6)
	require.Zero(t, final.LinearVelocity.Z.Base())
	require.Equal(t, zeroAngular(t), final.AngularVelocity)
	require.InDelta(t, 5, final.Pose.Translation().X, 1e-6)
	require.Zero(t, final.Pose.Translation().Z)
	contact, err := scene.doc.ContactPair(t.Context(), scene.floor, scene.box, r3.Identity(), final.Pose,
		scene.config.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	postAtZero, err := report.Trace.Sample(units.Seconds(0))
	require.NoError(t, err)
	postBody, ok := postAtZero.Body(scene.box)
	require.True(t, ok)
	require.Equal(t, final.LinearVelocity, postBody.LinearVelocity)
	midway, err := report.Trace.Sample(units.Seconds(.05))
	require.NoError(t, err)
	midBody, ok := midway.Body(scene.box)
	require.True(t, ok)
	require.InDelta(t, 2.5, midBody.Pose.Translation().X, 1e-6)
	end, err := report.Trace.Sample(units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), end.Entries())

	// At rest the corners stick: the floor stops the kick and friction
	// delivers nothing.
	static := scene.step(t, 0)
	require.Equal(t, dynamics.Advanced, static.Status, "%+v", static.Diagnostics)
	require.InDelta(t, 100, static.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	require.Len(t, static.Events, 1)
	require.InDelta(t, 100, static.Events[0].NormalImpulse.Base(), 1e-6)
	require.Zero(t, static.Events[0].TangentImpulse.X.Base())
	require.LessOrEqual(t, static.Events[0].Solver.TangentResidual.Base(), scene.config.VelocityResidual.Base())
	staticFinal, ok := static.Next.Body(scene.box)
	require.True(t, ok)
	require.Zero(t, staticFinal.LinearVelocity.X.Base())
	require.Zero(t, staticFinal.LinearVelocity.Z.Base())
	require.Equal(t, r3.Identity(), staticFinal.Pose)
}

// TestIslandFrictionSlideBoxFirst inserts the box before the floor, so the
// pair's normal points down from the box and the floor receives the
// friction: TestReverseFixedFloorFrictionStepUsesRealGeometry's numbers.
func TestIslandFrictionSlideBoxFirst(t *testing.T) {
	t.Parallel()
	scene := newFrictionSlide(t, true, pairMaterialStepConfig())
	for _, slip := range []float64{100, 0} {
		report := scene.step(t, slip)
		require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
		require.Len(t, report.Events, 1)
		event := report.Events[0]
		require.Equal(t, dynamics.BodyPair{A: scene.box, B: scene.floor}, event.Pair)
		require.Len(t, event.PointImpulses, 4)
		require.InDelta(t, 100, event.NormalImpulse.Base(), 1e-6)
		require.InDelta(t, 50*slip/100, event.TangentImpulse.X.Base(), 1e-6)
		require.Equal(t, zeroVelocity(), event.PostVelocityB)
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
		final, ok := report.Next.Body(scene.box)
		require.True(t, ok)
		require.InDelta(t, slip/2, final.LinearVelocity.X.Base(), 1e-6)
		require.Zero(t, final.LinearVelocity.Z.Base())
		require.InDelta(t, slip/20, final.Pose.Translation().X, 1e-6)
	}
}

// frictionStackScene slides a 10 mm box (1 kg) at 100 mm/s from the left half
// of the top of a 20×20×10 mm box (4 kg) resting on the fixed floor to its
// middle, with one distant fixed
// box making the world four bodies. Friction 0.5 everywhere, restitution
// zero, gravity −1000 mm/s² over 0.1 s: the floor delivers 500 kg·mm/s, the
// lower box 100 to the upper one, whose slide loses 50 kg·mm/s to friction.
// That friction pushes the lower box along +X; the floor can hold up to 250
// kg·mm/s there, so the lower box sticks.
type frictionStackScene struct {
	doc                 *decad.Document
	floor, lower, upper *decad.Body
	world               *dynamics.World
	state               dynamics.State
}

// frictionStackConfig lets the stack's two events at the step start and the
// solve's zero-time continuation stay within MaxEvents with time remaining.
func frictionStackConfig() dynamics.StepConfig {
	config := pairMaterialStepConfig()
	config.MaxIterations = 4096
	config.MaxEvents = 3
	return config
}

func newFrictionStack(t *testing.T) frictionStackScene {
	t.Helper()
	scene := frictionStackScene{doc: decad.New()}
	scene.floor = makeBox(t, scene.doc, -100, -100, 100, 100, -10, 10)
	scene.lower = makeBox(t, scene.doc, -10, -10, 10, 10, 0, 10)
	scene.upper = makeBox(t, scene.doc, -10, -5, 0, 5, 10, 10)
	far := makeBox(t, scene.doc, 495, -5, 505, 5, 0, 10)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(.5)}
	var err error
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: scene.floor, Role: dynamics.Fixed, Material: material},
		{Body: scene.lower, Role: dynamics.Dynamic, Density: &density, Material: material},
		{Body: scene.upper, Role: dynamics.Dynamic, Density: &density, Material: material},
		{Body: far, Role: dynamics.Fixed, Material: material},
	}, Step: frictionStackConfig()})
	require.NoError(t, err)
	slide := zeroVelocity()
	slide.X = units.MillimetersPerSecond(100)
	scene.state, err = scene.world.NewState([]dynamics.BodyState{
		{Body: scene.floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: scene.lower, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: scene.upper, Pose: r3.Identity(), LinearVelocity: slide, AngularVelocity: zeroAngular(t)},
		{Body: far, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	return scene
}

// TestIslandFrictionStackSlipsOverASticker solves the upper box's slip and
// the lower box's stick in one island: the dynamic/dynamic pair slips on its
// cone while the floor pair holds the lower box at rest against the upper
// box's friction.
func TestIslandFrictionStackSlipsOverASticker(t *testing.T) {
	t.Parallel()
	scene := newFrictionStack(t)
	config := pairMaterialStepConfig()
	report, err := scene.world.Step(t.Context(), scene.state, dynamics.StepInput{Gravity: gravityZ(-1000)},
		units.Seconds(.1))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Islands, 1)
	require.Equal(t, []*decad.Body{scene.floor, scene.lower, scene.upper}, report.Islands[0].Bodies)
	require.Len(t, report.Events, 2)
	floorEvent, stackEvent := report.Events[0], report.Events[1]
	require.Equal(t, dynamics.BodyPair{A: scene.floor, B: scene.lower}, floorEvent.Pair)
	require.Equal(t, dynamics.BodyPair{A: scene.lower, B: scene.upper}, stackEvent.Pair)
	require.InDelta(t, 500, floorEvent.NormalImpulse.Base(), 1e-6)
	require.InDelta(t, 100, stackEvent.NormalImpulse.Base(), 1e-6)
	// The upper box slips: friction 0.5·100 opposes its slide at every corner.
	require.InDelta(t, -50, stackEvent.TangentImpulse.X.Base(), 1e-6)
	for i, impulse := range stackEvent.PointImpulses {
		require.InDelta(t, .5*impulse.Normal.Base(), -impulse.Tangent.X.Base(), 1e-6, "point %d", i)
	}
	// The floor holds the lower box against the +50 the upper box hands it,
	// strictly inside its 250 kg·mm/s cone.
	require.InDelta(t, -50, floorEvent.TangentImpulse.X.Base(), 1e-6)
	require.InDelta(t, -50, report.Conservation.ContactImpulse.Value.X.Base(), 1e-6)
	require.InDelta(t, 500, report.Conservation.ContactImpulse.Value.Z.Base(), 1e-6)
	solver := report.Islands[0].Solver
	require.LessOrEqual(t, solver.TangentResidual.Base(), config.VelocityResidual.Base())
	require.LessOrEqual(t, solver.ConeResidual.Base(), config.ImpulseResidual.Base())
	lower, ok := report.Next.Body(scene.lower)
	require.True(t, ok)
	require.Equal(t, zeroVelocity(), lower.LinearVelocity)
	require.Equal(t, zeroAngular(t), lower.AngularVelocity)
	require.Equal(t, r3.Identity(), lower.Pose)
	upper, ok := report.Next.Body(scene.upper)
	require.True(t, ok)
	require.InDelta(t, 50, upper.LinearVelocity.X.Base(), 1e-6)
	require.Zero(t, upper.LinearVelocity.Z.Base())
	require.Equal(t, zeroAngular(t), upper.AngularVelocity)
	require.InDelta(t, 5, upper.Pose.Translation().X, 1e-6)
	require.Zero(t, upper.Pose.Translation().Z)
	contact, err := scene.doc.ContactPair(t.Context(), scene.lower, scene.upper, lower.Pose, upper.Pose,
		config.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
}

// Gates and legs of the Coulomb rows (docs/multibody-dynamics-design.md
// §6.3), each shown to fail by deleting it in island_certify.go, watching
// the named test go red, and restoring it:
//   - cone: TestIslandFrictionCertificateGates/"cone";
//   - the cone's μ_lo·λn allowance (zeroed): the untampered slide proposal
//     of TestIslandFrictionCertificateGates refuses;
//   - stick (the branch removed, so every point takes the slip row):
//     TestIslandFrictionCertificateGates/"stick";
//   - slip: TestIslandFrictionCertificateGates/"slip";
//   - the slip row's ‖λt‖·‖w'_t‖ term: TestIslandFrictionCertificateGates/"slip",
//     whose friction is turned at right angles to the slide so λt·w'_t is
//     zero;
//   - the slip row's λt·w'_t term: the untampered slide proposal refuses;
//   - the tangent impulse in the linear and angular laws (dropped from J):
//     the untampered slide proposal refuses the linear law;
//   - the tangent impulse in the step's contact-impulse reading
//     (islandContactImpulse) is the −50 kg·mm/s
//     TestIslandFrictionSlideThroughGeneralPath asserts.
//
// Legs not shown to fail: the normal ball and the lever intervals enter
// w'_t only through a normal bound or a spin times a witness bound, both
// exactly zero on the exact source-box faces of these fixtures. The limits
// ImpulseResidual and VelocityResidual of the three rows only admit a
// rounding residual; deleting one tightens the gate and can never admit a
// proposal.

// TestIslandFrictionCertificateGates tampers with the certified proposal of
// the sliding and the resting box and requires the named gate among the
// refusals. Both untampered proposals pass every gate.
func TestIslandFrictionCertificateGates(t *testing.T) {
	t.Parallel()
	scene := newFrictionSlide(t, false, pairMaterialStepConfig())
	gates := func(slip float64, tamper func(*dynamics.IslandProposal)) []string {
		t.Helper()
		names, err := dynamics.IslandProposalGates(t.Context(), scene.world, scene.state(t, slip), gravityZ(-1000),
			units.Seconds(.1), tamper)
		require.NoError(t, err)
		return names
	}
	require.Empty(t, gates(100, func(*dynamics.IslandProposal) {}))
	require.Empty(t, gates(0, func(*dynamics.IslandProposal) {}))
	for _, tc := range []struct {
		gate   string
		slip   float64
		tamper func(*dynamics.IslandProposal)
	}{
		// Twice the friction at every corner: 1.0·λn against μ = 0.5.
		{"cone", 100, func(p *dynamics.IslandProposal) {
			for k := range p.Tangent {
				p.Tangent[k] = p.Tangent[k].Scale(2)
			}
		}},
		// The friction turned to −Y: on the cone, but at right angles to the
		// +X slide it should oppose.
		{"slip", 100, func(p *dynamics.IslandProposal) {
			for k, tangent := range p.Tangent {
				p.Tangent[k] = r3.Vec{Y: tangent.X}
			}
		}},
		// The resting box leaves sliding at 1e-3 mm/s with no friction
		// spent: a sticking point may not slide.
		{"stick", 0, func(p *dynamics.IslandProposal) {
			for slot, body := range p.Bodies {
				if body == scene.box {
					p.Linear[slot].X = units.MillimetersPerSecond(1e-3)
				}
			}
		}},
	} {
		t.Run(tc.gate, func(t *testing.T) {
			require.Contains(t, gates(tc.slip, tc.tamper), tc.gate)
		})
	}
}
