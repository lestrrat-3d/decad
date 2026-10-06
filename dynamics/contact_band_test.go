package dynamics_test

import (
	"math"
	"math/big"
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
// 0.125 mm, SupportBand 0.05 mm, HeldChord 0.03 mm and PointResolution
// 0.1 mm, since a displaced body's witness balls carry its δ.
func partsBinConfig() dynamics.StepConfig {
	config := tumbleStepConfig()
	config.PenetrationResidual = units.Millimeters(.125)
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

// bottleBody is §2's revolved bottle: the full revolve of a line-and-arc
// half-profile, a Ø16 mm base, a quarter-circle shoulder to a Ø8 mm neck and
// 24 mm tall. Contact reads it as a held mesh at the request's HeldChord,
// whose displacement δ is the curved faces' chord sagitta.
func bottleBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(8, 0)
	c := s.CreatePoint(8, 14)
	center := s.CreatePoint(4, 14)
	d := s.CreatePoint(4, 18)
	e := s.CreatePoint(4, 24)
	f := s.CreatePoint(0, 24)
	for _, p := range []*sketch.Point{a, b, c, center, d, e, f} {
		s.Fix(p)
	}
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateArc(center, c, d)
	s.CreateLine(d, e)
	s.CreateLine(e, f)
	s.CreateLine(f, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := doc.Revolve(s, s.Profiles()[0], decad.SketchLine{
		Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// bottleScene is the bottle over §2's tray. delta is the held mesh's
// displacement and base the number of its vertices on the base plane z = 0.
type bottleScene struct {
	doc          *decad.Document
	tray, bottle *decad.Body
	delta        float64
	base         int
}

func newBottleScene(t *testing.T) bottleScene {
	t.Helper()
	scene := bottleScene{doc: decad.New()}
	scene.tray = tumbleTray(t, scene.doc)
	scene.bottle = bottleBody(t, scene.doc)
	mesh, err := scene.bottle.Tessellate(t.Context(), partsBinConfig().Contact.HeldChord)
	require.NoError(t, err)
	scene.delta = mesh.Bound().Base()
	for _, vertex := range mesh.Vertices() {
		if vertex.Z == 0 {
			scene.base++
		}
	}
	return scene
}

// world builds the scene's world under config, the bottle at pose with
// velocity (0, 0, vz).
func (s bottleScene) world(t *testing.T, config dynamics.StepConfig, pose r3.Transform,
	vz float64) (*dynamics.World, dynamics.State) {
	t.Helper()
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(.3), Friction: units.Scalar(.4)}
	world, err := dynamics.NewWorld(t.Context(), s.doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: s.tray, Role: dynamics.Fixed, Material: material},
		{Body: s.bottle, Role: dynamics.Dynamic, Density: &density, Material: material},
	}, Step: config})
	require.NoError(t, err)
	velocity := zeroVelocity()
	velocity.Z = units.MillimetersPerSecond(vz)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: s.tray, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: s.bottle, Pose: pose, LinearVelocity: velocity, AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	return world, state
}

// exactFloat is the exact rational a float64 denotes.
func exactFloat(t *testing.T, f float64) *big.Rat {
	t.Helper()
	r := new(big.Rat).SetFloat64(f)
	require.NotNil(t, r, "%v is not finite", f)
	return r
}

// roundedUp is the smallest float64 at or above x.
func roundedUp(x *big.Rat) float64 {
	f, _ := x.Float64()
	if new(big.Rat).SetFloat64(f).Cmp(x) < 0 {
		f = math.Nextafter(f, math.Inf(1))
	}
	return f
}

// witnessTorqueOnB recomputes §6.3's witness torque T_β = Σ b_k·|J_k|_1 of
// an event's side-B body, exactly, from the event's manifold and point
// impulses: b_k is OnB.Bound, and |J_k|_1 the L1 norm at its upper end of the
// impulse λ_k·n + λt_k over the normal ball n ± (Normal.Bound + NormalAngle),
// which per component is |λ_k·n_i + λt_i| + λ_k·(Normal.Bound + NormalAngle).
func witnessTorqueOnB(t *testing.T, event dynamics.ContactEvent) *big.Rat {
	t.Helper()
	require.Len(t, event.PointImpulses, len(event.Manifold.Points))
	torque := new(big.Rat)
	for k, point := range event.Manifold.Points {
		lambda := exactFloat(t, event.PointImpulses[k].Normal.Base())
		ball := new(big.Rat).Add(exactFloat(t, point.Normal.Bound.Base()), exactFloat(t, point.NormalAngle.Base()))
		tangent := event.PointImpulses[k].Tangent
		l1 := new(big.Rat)
		for _, pair := range [3][2]float64{{point.Normal.Value.X, tangent.X.Base()},
			{point.Normal.Value.Y, tangent.Y.Base()}, {point.Normal.Value.Z, tangent.Z.Base()}} {
			component := new(big.Rat).Mul(lambda, exactFloat(t, pair[0]))
			component.Add(component, exactFloat(t, pair[1]))
			l1.Add(l1, component.Abs(component))
			l1.Add(l1, new(big.Rat).Mul(lambda, ball))
		}
		torque.Add(torque, new(big.Rat).Mul(exactFloat(t, point.OnB.Bound.Base()), l1))
	}
	return torque
}

// TestDisplacedBottleRestsOnTray is docs/multibody-dynamics-design.md §13
// PR 20g's fixture: §2's revolved bottle, read as a held mesh whose δ is
// about 0.03 mm, dropped 8 mm onto §2's tray at §2's Phase 3 residuals. It
// lands on a ContactBand on every vertex of its held base, rebounds at
// restitution 0.3, lands again, and comes to rest within 32 steps. Each
// landing solve certifies through §6.3's witness torque: every lifted point's
// witness ball carries δ, so the landing impulse's torque about the mass
// center is known only to δ times the impulse, which the angular law's limit
// carries and the island publishes as WitnessTorque and WitnessSpin.
//
// Legs shown to fail (each changed in turn, fixture red, then restored):
//   - T_β zeroed: the landing step is StepIslandResidual at the angular law,
//     its residual about 55 kg·mm²/s against a limit of about 1.2e-4;
//   - T_β left out of the angular-momentum limit alone: the same landing is
//     refused at the angular momentum row;
//   - WitnessTorque published from PointResolution in place of the attained
//     witness balls: the recomputation reads a published torque about 3.4
//     times the recomputed one.
//
// The mass-center ball added to b_k is the fourth leg; it is shown on
// TestFacetedFloorImpactRefusesUncertifiedResponse's "mass center
// uncertainty", which then advances.
func TestDisplacedBottleRestsOnTray(t *testing.T) {
	scene := newBottleScene(t)
	config := partsBinConfig()
	require.Positive(t, scene.delta)
	require.Less(t, config.Contact.SupportBand.Base()+2*scene.delta, config.PenetrationResidual.Base(),
		"the band track that carries the rest fits the residual")
	require.Positive(t, scene.base)
	world, state := scene.world(t, config, translation(t, r3.Vec{Z: 8}), 0)
	floor := dynamics.CertifiedInertiaFloor(world, scene.bottle)
	require.NotNil(t, floor)

	var landing *dynamics.ContactEvent
	var landingState dynamics.State
	rebounds, rested := 0, -1
	for k := range 32 {
		report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: gravityZ(-9810)}, pyramidDt())
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", k, report.Diagnostics)
		next, ok := report.Next.Body(scene.bottle)
		require.True(t, ok)
		atRest := next.LinearVelocity == zeroVelocity() && next.AngularVelocity == zeroAngular(t)
		for _, island := range report.Islands {
			require.Len(t, island.Events, 1)
			event := report.Events[island.Events[0]]
			require.Equal(t, dynamics.BodyPair{A: scene.tray, B: scene.bottle}, event.Pair)
			require.Len(t, event.Manifold.Points, scene.base, "step %d: every base vertex is lifted", k)
			for _, point := range event.Manifold.Points {
				// Each point is a lifted base vertex: a height inside the band,
				// charged with δ, under the floor's exact normal.
				require.GreaterOrEqual(t, point.Separation.Value.Base(), 0.0)
				require.LessOrEqual(t, point.Separation.Value.Base(), config.Contact.SupportBand.Base())
				require.GreaterOrEqual(t, point.Separation.Bound.Base(), scene.delta)
				require.GreaterOrEqual(t, point.OnB.Bound.Base(), scene.delta)
				require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
			}
			// The published witness torque is the exact T_β rounded up, and
			// its spin that over the bottle's certified lower eigenvalue.
			torque := witnessTorqueOnB(t, event)
			require.Equal(t, roundedUp(torque), island.Solver.WitnessTorque.Base(), "step %d", k)
			require.Equal(t, roundedUp(new(big.Rat).Quo(torque, floor)), island.Solver.WitnessSpin.Base(), "step %d", k)
			pre, post := event.PreVelocityB.Z.Base(), event.PostVelocityB.Z.Base()
			if pre < -config.ImpactSpeed.Base() {
				require.InDelta(t, -.3*pre, post, 1e-9*-pre, "step %d rebounds at restitution 0.3", k)
				rebounds++
			}
			if landing == nil {
				landing, landingState = &event, state
				require.Less(t, island.Solver.WitnessSpin.Base(), 1.0)
			}
			if atRest {
				require.Less(t, island.Solver.WitnessSpin.Base(), .05, "step %d", k)
			}
		}
		state = *report.Next
		if landing == nil || !atRest {
			require.Negative(t, rested, "step %d leaves rest", k)
			continue
		}
		if rested >= 0 {
			break
		}
		rested = k
	}
	require.NotNil(t, landing)
	require.GreaterOrEqual(t, rebounds, 2, "the bottle rebounds at the landing and again")
	require.GreaterOrEqual(t, rested, 0, "the bottle rests within 32 steps")

	// At rest the pair is a ContactBand whose gap bound is at most δ +
	// SupportBand.
	resting, ok := state.Body(scene.bottle)
	require.True(t, ok)
	contact, err := scene.doc.ContactPair(t.Context(), scene.tray, scene.bottle, r3.Identity(), resting.Pose,
		config.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactBand, contact.Relation, "reason=%v", contact.Reason)
	require.LessOrEqual(t, contact.Gap.Bound.Base(), scene.delta+config.Contact.SupportBand.Base())

	// At PointResolution 1e-6 mm the lifted points' δ-wide witness balls are
	// too coarse: the landing pose's band is published without its manifold
	// (ContactPointTooCoarse), so the landing step brackets the impact and
	// stops with StepManifoldMissing at the rounded event poses.
	fine := config
	fine.Contact.PointResolution = units.Millimeters(1e-6)
	coarse, err := scene.doc.ContactPair(t.Context(), scene.tray, scene.bottle, r3.Identity(), landing.PoseB,
		fine.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactPointTooCoarse, coarse.Reason)
	require.Nil(t, coarse.Manifold)
	landed, ok := landingState.Body(scene.bottle)
	require.True(t, ok)
	fineWorld, fineState := scene.world(t, fine, landed.Pose, landed.LinearVelocity.Z.Base())
	report, err := fineWorld.Step(t.Context(), fineState, dynamics.StepInput{Gravity: gravityZ(-9810)}, pyramidDt())
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepManifoldMissing, report.Diagnostics[0].Code, "%+v", report.Diagnostics)
	require.Equal(t, dynamics.BodyPair{A: scene.tray, B: scene.bottle}, report.Diagnostics[0].Pair)
	require.Empty(t, report.Events)

	// At PenetrationResidual 0.1 mm the landing solves, but the band track
	// after the rebound holds the lifted base at SupportBand + 2δ, beyond the
	// residual.
	tight := config
	tight.PenetrationResidual = units.Millimeters(.1)
	tightWorld, tightState := scene.world(t, tight, landed.Pose, landed.LinearVelocity.Z.Base())
	report, err = tightWorld.Step(t.Context(), tightState, dynamics.StepInput{Gravity: gravityZ(-9810)}, pyramidDt())
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepTrackUnproved, report.Diagnostics[0].Code, "%+v", report.Diagnostics)
	require.Equal(t, tight.PenetrationResidual, report.Diagnostics[0].Limit)
}
