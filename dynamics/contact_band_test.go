package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The dynamics fixtures of docs/multibody-dynamics-design.md §13 PR 18: a
// ContactBand (§10.4) is a touch whose penetration bound, its gap band, must
// lie within PenetrationResidual. The displaced body is a 10 mm box Union a
// radius-2 disc standing 2 mm proud of its top, whose held mesh stands for
// its true boundary within δ, the disc's chord sagitta (about 3.7e-4 mm): its
// band is 2δ, about 7.3e-4 mm. An octagonal prism rests on the disc's top,
// so its own exact lower face supplies the contact normal.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - the initial contact's band admission (contactBandWithin on the
//     initial event): TestContactBandRestsWithinResidual's 1e-4 mm world
//     solves the landing on a band it does not admit, and only its later
//     band track stops it, with StepTrackUnproved;
//   - the fixed pair's band check: TestFixedPairContactBandAgainstResidual's
//     1e-4 mm world advances;
//   - the contact set's band tracks (restingContacts' continuesTrack): the
//     carried state's step makes as many pair calls as the rebuilt one.

// knobBody is the box Union disc, moved 1/8 mm along X so no exact support
// proof survives, and its displacement.
func knobBody(t *testing.T, doc *decad.Document) (*decad.Body, float64) {
	t.Helper()
	base := makeBox(t, doc, -5, -5, 5, 5, 0, 10)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	disc, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	require.NoError(t, err)
	disc, err = disc.Placed(t.Context(), translation(t, r3.Vec{Z: 8}))
	require.NoError(t, err)
	union, err := decad.Union(t.Context(), base, disc)
	require.NoError(t, err)
	knob, err := union.Placed(t.Context(), translation(t, r3.Vec{X: .125}))
	require.NoError(t, err)
	mesh, err := knob.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	delta := mesh.Bound().Base()
	require.Greater(t, 2*delta, 1e-4, "the 1e-4 mm residual lies below the band")
	require.Less(t, 2*delta, 1e-3, "and the 1e-3 mm residual above it")
	return knob, delta
}

func translation(t *testing.T, v r3.Vec) r3.Transform {
	t.Helper()
	pose, err := r3.Translation(v)
	require.NoError(t, err)
	return pose
}

// octagonBody is an 8 mm tall prism over the octagon cut from an 8 mm square
// by 2 mm corner triangles: 56 mm² of section, 448 mm³.
func octagonBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	pts := [][2]float64{{-4, -2}, {-2, -4}, {2, -4}, {4, -2}, {4, 2}, {2, 4}, {-2, 4}, {-4, 2}}
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		points[i] = s.CreatePoint(p[0], p[1])
	}
	s.Fix(points[0])
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// bandConfig resolves the band's charged witness balls and puts the
// penetration residual at residual.
func bandConfig(residual float64) dynamics.StepConfig {
	config := pairMaterialStepConfig()
	config.Contact = decad.ContactRequest{PointResolution: units.Millimeters(1e-3), NormalResolution: units.Degrees(1)}
	config.PenetrationResidual = units.Millimeters(residual)
	config.ImpactSpeed = units.MillimetersPerSecond(64)
	config.MaxEvents = 8
	return config
}

type bandScene struct {
	doc           *decad.Document
	knob, octagon *decad.Body
	world         *dynamics.World
	state         dynamics.State
	delta         float64
}

// newBandScene rests the octagon on the knob's disc top, z = 12, with two far
// fixed boxes; role is the octagon's.
func newBandScene(t *testing.T, residual float64, role dynamics.BodyRole) bandScene {
	t.Helper()
	scene := bandScene{doc: decad.New()}
	scene.knob, scene.delta = knobBody(t, scene.doc)
	scene.octagon = octagonBody(t, scene.doc)
	far, farther := farBox(t, scene.doc, 1000), farBox(t, scene.doc, 2000)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	octagon := dynamics.RigidBody{Body: scene.octagon, Role: role, Material: material}
	farBody := dynamics.RigidBody{Body: far, Role: dynamics.Fixed, Material: material}
	if role == dynamics.Dynamic {
		octagon.Density = &density
	} else {
		// A world needs a dynamic body; the far one moves nothing near.
		farBody.Role, farBody.Density = dynamics.Dynamic, &density
	}
	var err error
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: scene.knob, Role: dynamics.Fixed, Material: material},
		octagon,
		farBody,
		{Body: farther, Role: dynamics.Fixed, Material: material},
	}, Step: bandConfig(residual)})
	require.NoError(t, err)
	scene.state, err = scene.world.NewState([]dynamics.BodyState{
		{Body: scene.knob, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: scene.octagon, Pose: translation(t, r3.Vec{X: .125, Z: 12}), LinearVelocity: zeroVelocity(),
			AngularVelocity: zeroAngular(t)},
		{Body: far, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: farther, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	return scene
}

func TestContactBandRestsWithinResidual(t *testing.T) {
	dt := units.Seconds(1.0 / 256)
	scene := newBandScene(t, 1e-3, dynamics.Dynamic)
	contact, err := scene.doc.ContactPair(t.Context(), scene.knob, scene.octagon, r3.Identity(),
		translation(t, r3.Vec{X: .125, Z: 12}), bandConfig(1e-3).Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactBand, contact.Relation)
	require.Equal(t, 2*scene.delta, contact.Gap.Bound.Base())

	// Each step's kick lands the octagon on the band: the disc delivers the
	// kick's momentum, 0.448 kg · 9810/256 mm/s, and a band track carries
	// the rest of the step within the 1e-3 mm residual.
	state := scene.state
	for range 2 {
		report, err := scene.world.Step(t.Context(), state, dynamics.StepInput{Gravity: gravityZ(-9810)}, dt)
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
		require.Len(t, report.Events, 1)
		event := report.Events[0]
		require.Zero(t, event.Time.Base())
		require.InDelta(t, .448*9810/256, event.NormalImpulse.Base(), 1e-6)
		for _, point := range event.Manifold.Points {
			require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value, "the octagon's exact face supplies the normal")
			require.Equal(t, 2*scene.delta, point.Separation.Bound.Base())
		}
		proofs := dynamics.TraceSliceProofs(report.Trace)
		last := proofs[len(proofs)-1]
		var band *decad.Measurement
		for _, proof := range last {
			if proof.Sweep != nil && proof.Sweep.Outcome == decad.SweepPersistentBand {
				band = proof.Sweep.ContactTrack.Band()
				require.Equal(t, 1.0, proof.Sweep.ContactTrack.End().Fraction.Base())
			}
		}
		require.NotNil(t, band, "a band track carries the rest of the step")
		require.GreaterOrEqual(t, band.Value.Base()+band.Bound.Base(), 2*scene.delta)
		require.LessOrEqual(t, band.Value.Base()+band.Bound.Base(), 1e-3)
		resting, ok := report.Next.Body(scene.octagon)
		require.True(t, ok)
		require.Equal(t, zeroVelocity(), resting.LinearVelocity)
		require.Equal(t, translation(t, r3.Vec{X: .125, Z: 12}), resting.Pose)
		state = *report.Next
	}

	// Without gravity the published contact set carries the band pair into
	// the next step, which continues its band track with no initial-contact
	// stop: one pair call fewer than the same entries rebuilt without it.
	// Both start without the reuse cache, so only the contact set differs.
	carried, err := scene.world.Step(t.Context(), dynamics.WithoutCache(state),
		dynamics.StepInput{Gravity: zeroAcceleration()}, dt)
	require.NoError(t, err)
	rebuiltState, err := scene.world.NewState(state.Entries())
	require.NoError(t, err)
	rebuilt, err := scene.world.Step(t.Context(), rebuiltState, dynamics.StepInput{Gravity: zeroAcceleration()}, dt)
	require.NoError(t, err)
	for _, report := range []*dynamics.StepReport{carried, rebuilt} {
		require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
		require.Empty(t, report.Events)
	}
	require.Equal(t, rebuilt.Next.Entries(), carried.Next.Entries())
	require.Less(t, dynamics.TracePairCalls(carried.Trace), dynamics.TracePairCalls(rebuilt.Trace))

	// Below the band the same landing is undecided at its first contact.
	tight := newBandScene(t, 1e-4, dynamics.Dynamic)
	report, err := tight.world.Step(t.Context(), tight.state, dynamics.StepInput{Gravity: gravityZ(-9810)}, dt)
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepPairUndecided, report.Diagnostics[0].Code)
	require.Equal(t, units.Millimeters(1e-4), report.Diagnostics[0].Limit)
	require.Empty(t, report.Events)
	require.Nil(t, report.Next)
}

func TestFixedPairContactBandAgainstResidual(t *testing.T) {
	// The octagon fixed on the knob is a Fixed/Fixed pair in its band: it may
	// overlap by 2δ, so the residual decides it.
	dt := units.Seconds(1.0 / 256)
	loose := newBandScene(t, 1e-3, dynamics.Fixed)
	report, err := loose.world.Step(t.Context(), loose.state, dynamics.StepInput{Gravity: zeroAcceleration()}, dt)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)

	tight := newBandScene(t, 1e-4, dynamics.Fixed)
	report, err = tight.world.Step(t.Context(), tight.state, dynamics.StepInput{Gravity: zeroAcceleration()}, dt)
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepFixedPairRelation, report.Diagnostics[0].Code)
	require.Equal(t, dynamics.BodyPair{A: tight.knob, B: tight.octagon}, report.Diagnostics[0].Pair)
	require.Equal(t, units.Millimeters(1e-4), report.Diagnostics[0].Limit)
}

// partsBinConfig is §2's Phase 3 step configuration: PenetrationResidual
// 0.1 mm, SupportBand 0.05 mm, HeldChord 0.03 mm and PointResolution 0.1 mm,
// since a displaced body's witness balls carry its δ.
func partsBinConfig() dynamics.StepConfig {
	config := tumbleStepConfig()
	config.PenetrationResidual = units.Millimeters(.1)
	config.Contact.SupportBand = units.Millimeters(.05)
	config.Contact.HeldChord = units.Millimeters(.03)
	config.Contact.PointResolution = units.Millimeters(.1)
	return config
}

// TestDisplacedBlockRestsOnTray is docs/multibody-dynamics-design.md §13
// PR 20c's dynamics fixture: §2's 12 mm block, its top loop chamfered
// 2.1 mm so its held mesh carries δ ≈ 1e-15 mm, dropped 8 mm onto the tray.
// Its held gap first enters the lifted band b = max(SupportBand, δ) as a
// four-point ContactBand, which brackets the landing; the block bounces at
// restitution 0.3 and rests on that band with both velocities exactly zero.
//
// Leg shown to fail: the displaced pair's lifted band deleted, the landing's
// right sample is a held overlap of the block's flat face, which neither
// §9.3 (the tray is not convex) nor §9.6 (the deepest set is a face)
// publishes, and the landing step stops with StepManifoldMissing.
func TestDisplacedBlockRestsOnTray(t *testing.T) {
	doc := decad.New()
	tray := tumbleTray(t, doc)
	box := makeBox(t, doc, -6, -6, 6, 6, 0, 12)
	block, err := box.Chamfer(t.Context(), decad.Edges(decad.CreatedBy(decad.CapEnd(box))), units.Millimeters(2.1))
	require.NoError(t, err)
	mesh, err := block.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	delta := mesh.Bound().Base()
	require.Positive(t, delta)
	config := partsBinConfig()
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(.3), Friction: units.Scalar(.4)}
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: tray, Role: dynamics.Fixed, Material: material},
		{Body: block, Role: dynamics.Dynamic, Density: &density, Material: material},
	}, Step: config})
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: tray, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: block, Pose: translation(t, r3.Vec{Z: 8}), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)

	landed, rested := false, -1
	for k := range 32 {
		report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: gravityZ(-9810)}, pyramidDt())
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", k, report.Diagnostics)
		for _, event := range report.Events {
			require.Len(t, event.Manifold.Points, 4, "step %d", k)
			for _, point := range event.Manifold.Points {
				// Each point is a lifted corner: an exact height inside the
				// band, charged with δ, under the floor's exact normal.
				require.Positive(t, point.Separation.Value.Base())
				require.LessOrEqual(t, point.Separation.Value.Base(), config.Contact.SupportBand.Base())
				require.GreaterOrEqual(t, point.Separation.Bound.Base(), delta)
				require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
			}
			landed = true
		}
		next, ok := report.Next.Body(block)
		require.True(t, ok)
		state = *report.Next
		if !landed || next.LinearVelocity != zeroVelocity() || next.AngularVelocity != zeroAngular(t) {
			rested = -1
			continue
		}
		if rested < 0 {
			rested = k
		}
	}
	require.True(t, landed)
	require.GreaterOrEqual(t, rested, 0, "the block rests within 32 steps")

	// At rest the block hovers on its four lifted corners.
	resting, ok := state.Body(block)
	require.True(t, ok)
	contact, err := doc.ContactPair(t.Context(), tray, block, r3.Identity(), resting.Pose, config.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactBand, contact.Relation, "reason=%v", contact.Reason)
	require.NotNil(t, contact.Manifold)
	require.Len(t, contact.Manifold.Points, 4)
	require.LessOrEqual(t, contact.Gap.Bound.Base(), delta+config.Contact.SupportBand.Base())
}
