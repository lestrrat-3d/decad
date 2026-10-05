package dynamics_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The dynamics fixtures of docs/multibody-dynamics-design.md §13 PR 14e
// (§10.7, continuation inside the band): a tumble body falls onto a plain
// 240 mm source-box floor under the §2 material (restitution 0.3, friction
// 0.4), dt = 1/256 s, the stack-and-drop residuals and SupportBand =
// PenetrationResidual/2, and comes to rest face down through band tracks.
// A pair an event leaves inside a ContactBand within the residual continues
// under ContinueCertifiedTouch, so the band track over its support set
// carries it to the next vertex's arrival, where the island solves every
// vertex the rounded event poses hold inside the band.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - the band rule (island.go's continuations, every pair keeping the policy
//     the solve assigns): the prism stops in its step 30, 65.8 µs in,
//     StepPairUndecided with SweepTimeFloor, at the bracket
//     [ContactBand, Overlapping] of its pivot vertex's arrival;
//   - a solved pair whose corrected poses read a ContactBand continued under
//     ContinueCertifiedTouch (continuations' first rule): the same stop;
//
// The Separated drop (a band-end or track pair whose rounded event poses read
// Separated leaves the contact set) changes nothing in this run when deleted:
// the pair then keeps ContinueSeparatingTouch, and a planar sweep that starts
// Separated runs the same clear search under every start policy
// (contact_sweep_rotation.go), so only the §5.3 reuse key differs.

// tumbleRestScene is the floor, the tumble bodies and two far fixed boxes.
// body and mass name the first tumble body.
type tumbleRestScene struct {
	doc         *decad.Document
	floor, body *decad.Body
	bodies      []*decad.Body
	world       *dynamics.World
	state       dynamics.State
	config      dynamics.StepConfig
	mass        float64
	masses      map[*decad.Body]float64
}

// tumbleBody is one dynamic body of a tumble scene at its release.
type tumbleBody struct {
	body func(doc *decad.Document) *decad.Body
	pose r3.Transform
	spin dynamics.QuantityVec
}

// tumbleRestConfig is the stack-and-drop step with SupportBand =
// PenetrationResidual/2.
func tumbleRestConfig() dynamics.StepConfig {
	config := stackAndDropConfig()
	config.MaxPoseEvaluations = 512
	config.Contact.SupportBand = units.Millimeters(config.PenetrationResidual.Base() / 2)
	return config
}

func newTumbleRestScene(t *testing.T, body func(doc *decad.Document) *decad.Body, pose r3.Transform,
	spin dynamics.QuantityVec) tumbleRestScene {
	t.Helper()
	return newTumbleScene(t, tumbleRestConfig(), tumbleBody{body: body, pose: pose, spin: spin})
}

// newTumbleScene builds the world in order: the floor, the tumble bodies,
// then the two far boxes.
func newTumbleScene(t *testing.T, config dynamics.StepConfig, bodies ...tumbleBody) tumbleRestScene {
	t.Helper()
	scene := tumbleRestScene{doc: decad.New(), config: config, masses: map[*decad.Body]float64{}}
	scene.floor = makeBox(t, scene.doc, -120, -120, 120, 120, -10, 10)
	density := units.KilogramsPerCubicMillimeter(.001)
	material := dynamics.Material{Restitution: units.Scalar(.3), Friction: units.Scalar(.4)}
	rigid := []dynamics.RigidBody{{Body: scene.floor, Role: dynamics.Fixed, Material: material}}
	entries := []dynamics.BodyState{
		{Body: scene.floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	}
	for _, release := range bodies {
		body := release.body(scene.doc)
		properties, err := body.MassProperties(t.Context(), density)
		require.NoError(t, err)
		scene.bodies = append(scene.bodies, body)
		scene.masses[body] = properties.Mass.Value.Base()
		rigid = append(rigid, dynamics.RigidBody{Body: body, Role: dynamics.Dynamic, Density: &density, Material: material})
		entries = append(entries, dynamics.BodyState{Body: body, Pose: release.pose, LinearVelocity: zeroVelocity(),
			AngularVelocity: release.spin})
	}
	scene.body, scene.mass = scene.bodies[0], scene.masses[scene.bodies[0]]
	for _, x := range []float64{1000, 2000} {
		far := farBox(t, scene.doc, x)
		rigid = append(rigid, dynamics.RigidBody{Body: far, Role: dynamics.Fixed, Material: material})
		entries = append(entries, dynamics.BodyState{Body: far, Pose: r3.Identity(), LinearVelocity: zeroVelocity(),
			AngularVelocity: zeroAngular(t)})
	}
	var err error
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: rigid, Step: config})
	require.NoError(t, err)
	scene.state, err = scene.world.NewState(entries)
	require.NoError(t, err)
	return scene
}

// tumblePrism extrudes a closed polygon in the XY plane by height along Z.
func tumblePrism(t *testing.T, doc *decad.Document, corners [][2]float64, height float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(corners))
	for i, corner := range corners {
		points[i] = s.CreatePoint(corner[0], corner[1])
	}
	s.Fix(points[0])
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// tumbleRelease turns a body by degrees about axis and places it at at.
func tumbleRelease(t *testing.T, axis r3.Vec, degrees float64, at r3.Vec) r3.Transform {
	t.Helper()
	turn, err := r3.Rotation(axis, units.Degrees(degrees))
	require.NoError(t, err)
	pose, err := r3.FromBasis(turn.Basis(), at)
	require.NoError(t, err)
	return pose
}

// vertexHeights stages every vertex of body through pose exactly and returns
// its height above the floor's top face z = 0.
func vertexHeights(body *decad.Body, pose r3.Transform) []*big.Rat {
	basis, at := pose.Basis(), pose.Translation()
	out := make([]*big.Rat, 0, len(body.Vertices()))
	for _, vertex := range body.Vertices() {
		p := vertex.Position().Value
		z := new(big.Rat).Add(rat(at.Z), new(big.Rat).Mul(rat(basis.EX.Z), rat(p.X)))
		z.Add(z, new(big.Rat).Mul(rat(basis.EY.Z), rat(p.Y)))
		z.Add(z, new(big.Rat).Mul(rat(basis.EZ.Z), rat(p.Z)))
		out = append(out, z)
	}
	return out
}

// tumbleImpacts counts each body's ContactImpact events by manifold size.
type tumbleImpacts map[*decad.Body]map[int]int

// runTumbleRest advances the scene for steps steps, each Advanced, checks
// every impact's linear law, calls each, when non-nil, after every step, and
// returns each body's impact counts by manifold size and the last step's
// report.
func runTumbleRest(t *testing.T, scene tumbleRestScene, steps int,
	each func(step int, report *dynamics.StepReport)) (tumbleImpacts, *dynamics.StepReport) {
	t.Helper()
	timeline, err := dynamics.NewTimeline(scene.world, scene.state)
	require.NoError(t, err)
	impacts := tumbleImpacts{}
	var last *dynamics.StepReport
	for step := range steps {
		report, err := timeline.Advance(t.Context(), dynamics.StepInput{Gravity: gravityZ(-9810)}, pyramidDt())
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", step, report.Diagnostics)
		for _, event := range report.Events {
			if event.Kind != dynamics.ContactImpact {
				continue
			}
			require.Equal(t, scene.floor, event.Pair.A)
			require.Len(t, event.PointImpulses, len(event.Manifold.Points))
			var applied r3.Vec
			for i, point := range event.Manifold.Points {
				require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value)
				require.GreaterOrEqual(t, event.PointImpulses[i].Normal.Base(), 0.0)
				tangent := event.PointImpulses[i].Tangent
				applied = applied.Add(r3.Vec{X: tangent.X.Base(), Y: tangent.Y.Base(),
					Z: tangent.Z.Base() + event.PointImpulses[i].Normal.Base()})
			}
			// The certificate's linear gate is ImpulseResidual + m·VelocityResidual;
			// twice that absorbs this float re-reading of the published numbers.
			mass := scene.masses[event.Pair.B]
			slack := 2 * (scene.config.ImpulseResidual.Base() + mass*scene.config.VelocityResidual.Base())
			pre, post := event.PreVelocityB, event.PostVelocityB
			require.InDelta(t, applied.X, mass*(post.X.Base()-pre.X.Base()), slack, "step %d", step)
			require.InDelta(t, applied.Y, mass*(post.Y.Base()-pre.Y.Base()), slack, "step %d", step)
			require.InDelta(t, applied.Z, mass*(post.Z.Base()-pre.Z.Base()), slack, "step %d", step)
			if impacts[event.Pair.B] == nil {
				impacts[event.Pair.B] = map[int]int{}
			}
			impacts[event.Pair.B][len(event.Manifold.Points)]++
		}
		if each != nil {
			each(step, report)
		}
		last = report
	}
	return impacts, last
}

// requireTumbleRest checks a face-down rest of body at the end of a run:
// both velocities exactly zero, every vertex's exact staged height at or above
// the floor, at least three of them (a face) within PenetrationResidual of
// it, and the last slice certifying the floor pair by swept boxes strictly
// apart or a band track through the step's end.
func requireTumbleRest(t *testing.T, scene tumbleRestScene, body *decad.Body, last *dynamics.StepReport) {
	t.Helper()
	residual := rat(scene.config.PenetrationResidual.Base())
	entry, ok := last.Next.Body(body)
	require.True(t, ok)
	for _, v := range []dynamics.QuantityVec{entry.LinearVelocity, entry.AngularVelocity} {
		require.Zero(t, v.X.Base())
		require.Zero(t, v.Y.Base())
		require.Zero(t, v.Z.Base())
	}
	face := 0
	for _, height := range vertexHeights(body, entry.Pose) {
		require.GreaterOrEqual(t, height.Sign(), 0)
		if height.Cmp(residual) <= 0 {
			face++
		}
	}
	require.GreaterOrEqual(t, face, 3, "the body rests on a face")
	slices := dynamics.TraceSliceProofs(last.Trace)
	require.NotEmpty(t, slices)
	for _, proof := range slices[len(slices)-1] {
		if proof.Pair.A != scene.floor || proof.Pair.B != body || proof.BoxClear {
			continue
		}
		require.Equal(t, decad.SweepPersistentBand, proof.Sweep.Outcome)
		require.Equal(t, 1.0, proof.Sweep.ContactTrack.End().Fraction.Base())
		require.LessOrEqual(t, proof.Sweep.ContactTrack.Band().Value.Base(), scene.config.PenetrationResidual.Base())
	}
}

func TestHexagonalPrismRestsFromVertex(t *testing.T) {
	// The §2 prism, 20 mm across flats and 12 mm tall, released on a vertex:
	// turned 37° about −Y, its lowest vertex about 7 mm above the floor.
	hex := func(doc *decad.Document) *decad.Body {
		return tumblePrism(t, doc, [][2]float64{{0, 0}, {5.75, -10}, {17.25, -10}, {23, 0}, {17.25, 10}, {5.75, 10}}, 12)
	}
	scene := newTumbleRestScene(t, hex, tumbleRelease(t, r3.Vec{Y: -1}, 37, r3.Vec{X: -55, Z: 20}), zeroAngular(t))
	// The prism lands on one vertex and bounces on it; the band end at the
	// next arrival solves three vertices of its hexagonal cap at once, and
	// every later step absorbs its kick on all six.
	impacts, last := runTumbleRest(t, scene, 256, nil)
	require.Positive(t, impacts[scene.body][1], "a vertex impact")
	faces := 0
	for points, count := range impacts[scene.body] {
		if points >= 3 {
			faces += count
		}
	}
	require.Positive(t, faces, "a face impact")
	require.Len(t, last.Events, 1)
	require.Len(t, last.Events[0].Manifold.Points, 6)
	requireTumbleRest(t, scene, scene.body, last)
}

// tumbleHexBody, tumbleWedgeBody and tumbleBoxBody are the §2 hexagonal
// prism (20 mm across flats, 12 mm tall), triangular wedge and 20 mm box.
func tumbleHexBody(t *testing.T) func(doc *decad.Document) *decad.Body {
	return func(doc *decad.Document) *decad.Body {
		return tumblePrism(t, doc, [][2]float64{{0, 0}, {5.75, -10}, {17.25, -10}, {23, 0}, {17.25, 10}, {5.75, 10}}, 12)
	}
}

func tumbleWedgeBody(t *testing.T) func(doc *decad.Document) *decad.Body {
	return func(doc *decad.Document) *decad.Body {
		return tumblePrism(t, doc, [][2]float64{{0, 0}, {16, 0}, {4, 12}}, 8)
	}
}

func tumbleBoxBody(t *testing.T) func(doc *decad.Document) *decad.Body {
	return func(doc *decad.Document) *decad.Body { return makeBox(t, doc, -10, -10, 10, 10, -10, 20) }
}

// tumbleBoxSpin is the §2 boxes' release spin, (2, 1, 0) rad/s.
func tumbleBoxSpin() dynamics.QuantityVec {
	return dynamics.QuantityVec{X: units.RadiansPerSecond(2), Y: units.RadiansPerSecond(1), Z: units.RadiansPerSecond(0)}
}

// requirePublishedHeights checks every pose a step publishes for body, its
// event poses and its end pose: no vertex's exact staged height lies below
// −PenetrationResidual. It reports whether some vertex lies below the floor.
func requirePublishedHeights(t *testing.T, scene tumbleRestScene, body *decad.Body, step int,
	report *dynamics.StepReport) bool {
	t.Helper()
	floor := rat(-scene.config.PenetrationResidual.Base())
	entry, ok := report.Next.Body(body)
	require.True(t, ok)
	poses := []r3.Transform{entry.Pose}
	for _, event := range report.Events {
		if event.Pair.B == body {
			poses = append(poses, event.PoseB)
		}
	}
	below := false
	for _, pose := range poses {
		for _, height := range vertexHeights(body, pose) {
			require.GreaterOrEqual(t, height.Cmp(floor), 0, "step %d", step)
			below = below || height.Sign() < 0
		}
	}
	return below
}

// The dynamics fixtures of docs/multibody-dynamics-design.md §13 PR 14f
// (§10.8, rest inside the band): the step fills SweepRequest.RestSpeed with
// VelocityResidual, so a lifted vertex the solve rested is held on both
// sides of the floor; the band track reads §10.8's per-vertex curvature K_p;
// an overlapping initial contact solves on the manifold ContactPair publishes
// at the slice-start poses; an initial contact the previous step left in its
// contact set may correct the penetration that step admitted; and a band
// track that another pair's event cuts
// where its rounded poses read Overlapping is gathered as a band end. The
// step and time records below are amd64 runs; each assertion leaves slack
// for another architecture's rounding.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - RestSpeed left the zero Value in sweepRequest: TestWedgeRestsFromVertex
//     stops StepEventBudget in its step 29 on §10.7's band-end chain;
//   - the cut rule (trackPairs gathering every covering band track as a
//     track pair): TestWedgeRestsFromVertex stops StepTrackUnproved in its
//     step 29, 701.7 µs in, at the prism's impact;
//   - the overlapping initial contact (solveEvent): TestBoxSpinsOnCornerInsideBand
//     stops StepManifoldMissing in its step 49;
//   - the carried allowance (carriedPenetration returning zero):
//     TestBoxSpinsOnCornerInsideBand stops StepCorrectionFailed in its step
//     49, the corner the previous step left 4e-11 mm below the floor beyond
//     its 1e-12 mm ContactSlop;
//   - K_p replaced by §10.2's global K: TestBoxSpinsOnCornerInsideBand stops
//     StepEventBudget in its step 48, its band ends 46 to 62 µs apart;
//   - the §2 band itself, SupportBand = 0.5 nm with PenetrationResidual =
//     1 nm: TestBoxBouncesOnEdgeAndRestsFlat still rocks in its step 64, six
//     events in that step, its mass center leaving the step at about
//     1.26 mm/s (§10.8).
//
// The island rule that sends a gathered pair whose rounded poses read
// Overlapping into the solve whether or not a point closes changes none of
// these runs when deleted: every overlapping pair they gather has a closing
// point.

func TestWedgeRestsFromVertex(t *testing.T) {
	// The §2 wedge, released beside the §2 prism, at the shipped residuals.
	// It lands on one vertex and comes to rest on its triangular cap, which a
	// rested vertex reaches without the band-end chain of §10.7.
	scene := newTumbleScene(t, tumbleRestConfig(),
		tumbleBody{body: tumbleHexBody(t), pose: tumbleRelease(t, r3.Vec{Y: -1}, 37, r3.Vec{X: -55, Z: 20}),
			spin: zeroAngular(t)},
		tumbleBody{body: tumbleWedgeBody(t), pose: tumbleRelease(t, r3.Vec{X: 1, Y: 2, Z: 3}, 50, r3.Vec{X: 40, Z: 25}),
			spin: zeroAngular(t)})
	prism, wedge := scene.bodies[0], scene.bodies[1]
	impacts, last := runTumbleRest(t, scene, 256, nil)
	require.Positive(t, impacts[wedge][1], "a vertex impact")
	require.Positive(t, impacts[wedge][3], "a cap impact")
	require.Len(t, last.Events, 2)
	points := map[*decad.Body]int{}
	for _, event := range last.Events {
		points[event.Pair.B] = len(event.Manifold.Points)
	}
	require.Equal(t, map[*decad.Body]int{prism: 6, wedge: 3}, points)
	requireTumbleRest(t, scene, prism, last)
	requireTumbleRest(t, scene, wedge, last)
	entry, ok := last.Next.Body(wedge)
	require.True(t, ok)
	residual := rat(scene.config.PenetrationResidual.Base())
	onCap := 0
	for _, height := range vertexHeights(wedge, entry.Pose) {
		if height.Cmp(residual) <= 0 {
			onCap++
		}
	}
	require.Equal(t, 3, onCap, "the cap's three vertices within the residual, the rest above it")
}

func TestBoxSpinsOnCornerInsideBand(t *testing.T) {
	// The §2 60° box, released spinning at (2, 1, 0) rad/s, at the shipped
	// residuals. It lands on a corner and spins on it with that corner's band
	// track continued over many band ends; a corner the solve rested may sink
	// below the floor within the track's depth, and the step's published poses
	// show it there. The run records the overlapping initial contact in step
	// 49 and a corner below the floor in steps 48 and 49. Fifty steps cover
	// both legs; the corner then rocks with six events a step for the rest of
	// a 256-step run, at 0.8 s a step, past the package's race budget. A
	// ContactSlop of 1e-12 mm, below that corner's 4e-11 mm depth at the start
	// of step 49, leaves its correction to the allowance the previous step
	// carries (carriedPenetration).
	config := tumbleRestConfig()
	config.ContactSlop = units.Millimeters(1e-12)
	scene := newTumbleScene(t, config, tumbleBody{body: tumbleBoxBody(t),
		pose: tumbleRelease(t, r3.Vec{X: 1, Y: 1}, 60, r3.Vec{X: -45, Y: 45, Z: 40}), spin: tumbleBoxSpin()})
	below := false
	impacts, _ := runTumbleRest(t, scene, 50, func(step int, report *dynamics.StepReport) {
		if requirePublishedHeights(t, scene, scene.body, step, report) {
			below = true
		}
	})
	require.Positive(t, impacts[scene.body][1], "a corner impact")
	require.True(t, below, "a published pose holds a corner below the floor")
}

func TestBoxBouncesOnEdgeAndRestsFlat(t *testing.T) {
	// The §2 30° box, released spinning at (2, 1, 0) rad/s, at the §2 scene's
	// PenetrationResidual = 10 µm and SupportBand = 5 µm. It lands on a
	// corner, bounces onto an edge, and once a kick lands it on all four lower
	// corners it rests: the run records one four-point event in every step
	// from its 46th, and the assertion reads every step from the 65th.
	config := tumbleRestConfig()
	config.PenetrationResidual = units.Millimeters(.01)
	config.Contact.SupportBand = units.Millimeters(.005)
	scene := newTumbleScene(t, config, tumbleBody{body: tumbleBoxBody(t),
		pose: tumbleRelease(t, r3.Vec{X: 1, Y: 1}, 30, r3.Vec{X: -45, Y: -45, Z: 40}), spin: tumbleBoxSpin()})
	bandStart := false
	impacts, last := runTumbleRest(t, scene, 256, func(step int, report *dynamics.StepReport) {
		requirePublishedHeights(t, scene, scene.body, step, report)
		for _, slice := range dynamics.TraceSliceProofs(report.Trace) {
			for _, proof := range slice {
				sweep := proof.Sweep
				if sweep != nil && sweep.Request.StartPolicy == decad.ContinueCertifiedTouch &&
					sweep.InitialEvent != nil && sweep.InitialEvent.Relation == decad.ContactBand {
					bandStart = true
				}
			}
		}
		if step < 64 {
			return
		}
		// Equal, not Len: a failure prints the count, not every event.
		require.Equal(t, 1, len(report.Events), "step %d", step)
		require.Len(t, report.Events[0].Manifold.Points, 4, "step %d", step)
	})
	for _, points := range []int{1, 2, 4} {
		require.Positive(t, impacts[scene.body][points], "a %d-point impact", points)
	}
	require.True(t, bandStart, "a slice continues a ContactBand start under ContinueCertifiedTouch")
	requireTumbleRest(t, scene, scene.body, last)
	entry, ok := last.Next.Body(scene.body)
	require.True(t, ok)
	residual := rat(config.PenetrationResidual.Base())
	lower := 0
	for _, height := range vertexHeights(scene.body, entry.Pose) {
		require.GreaterOrEqual(t, height.Sign(), 0)
		if height.Cmp(residual) <= 0 {
			lower++
		}
	}
	require.Equal(t, 4, lower, "the four lower corners within the residual, the upper four above it")
}
