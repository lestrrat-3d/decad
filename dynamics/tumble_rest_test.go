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

// tumbleRestScene is the floor, one tumble body and two far fixed boxes.
type tumbleRestScene struct {
	doc         *decad.Document
	floor, body *decad.Body
	world       *dynamics.World
	state       dynamics.State
	config      dynamics.StepConfig
	mass        float64
}

func newTumbleRestScene(t *testing.T, body func(doc *decad.Document) *decad.Body, pose r3.Transform,
	spin dynamics.QuantityVec) tumbleRestScene {
	t.Helper()
	scene := tumbleRestScene{doc: decad.New()}
	scene.floor = makeBox(t, scene.doc, -120, -120, 120, 120, -10, 10)
	scene.body = body(scene.doc)
	far, farther := farBox(t, scene.doc, 1000), farBox(t, scene.doc, 2000)
	density := units.KilogramsPerCubicMillimeter(.001)
	properties, err := scene.body.MassProperties(t.Context(), density)
	require.NoError(t, err)
	scene.mass = properties.Mass.Value.Base()
	material := dynamics.Material{Restitution: units.Scalar(.3), Friction: units.Scalar(.4)}
	scene.config = stackAndDropConfig()
	scene.config.MaxPoseEvaluations = 512
	scene.config.Contact.SupportBand = units.Millimeters(scene.config.PenetrationResidual.Base() / 2)
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: scene.floor, Role: dynamics.Fixed, Material: material},
		{Body: scene.body, Role: dynamics.Dynamic, Density: &density, Material: material},
		{Body: far, Role: dynamics.Fixed, Material: material},
		{Body: farther, Role: dynamics.Fixed, Material: material},
	}, Step: scene.config})
	require.NoError(t, err)
	scene.state, err = scene.world.NewState([]dynamics.BodyState{
		{Body: scene.floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: scene.body, Pose: pose, LinearVelocity: zeroVelocity(), AngularVelocity: spin},
		{Body: far, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: farther, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
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

// runTumbleRest advances the scene for steps steps, each Advanced, checks
// every impact's linear law, and returns the impact counts by manifold size
// (1, 2, and 3 or more points) and the last step's report.
func runTumbleRest(t *testing.T, scene tumbleRestScene, steps int) ([3]int, *dynamics.StepReport) {
	t.Helper()
	timeline, err := dynamics.NewTimeline(scene.world, scene.state)
	require.NoError(t, err)
	// The certificate's linear gate is ImpulseResidual + m·VelocityResidual;
	// twice that absorbs this float re-reading of the published numbers.
	slack := 2 * (scene.config.ImpulseResidual.Base() + scene.mass*scene.config.VelocityResidual.Base())
	var impacts [3]int
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
			pre, post := event.PreVelocityB, event.PostVelocityB
			require.InDelta(t, applied.X, scene.mass*(post.X.Base()-pre.X.Base()), slack, "step %d", step)
			require.InDelta(t, applied.Y, scene.mass*(post.Y.Base()-pre.Y.Base()), slack, "step %d", step)
			require.InDelta(t, applied.Z, scene.mass*(post.Z.Base()-pre.Z.Base()), slack, "step %d", step)
			impacts[min(len(event.Manifold.Points), 3)-1]++
		}
		last = report
	}
	return impacts, last
}

// requireTumbleRest checks a face-down rest at the end of a run: both
// velocities exactly zero, every vertex's exact staged height at or above
// the floor, at least three of them (a face) within PenetrationResidual of
// it, and the last slice certifying the floor pair by swept boxes strictly
// apart or a band track through the step's end.
func requireTumbleRest(t *testing.T, scene tumbleRestScene, last *dynamics.StepReport) {
	t.Helper()
	residual := rat(scene.config.PenetrationResidual.Base())
	entry, ok := last.Next.Body(scene.body)
	require.True(t, ok)
	for _, v := range []dynamics.QuantityVec{entry.LinearVelocity, entry.AngularVelocity} {
		require.Zero(t, v.X.Base())
		require.Zero(t, v.Y.Base())
		require.Zero(t, v.Z.Base())
	}
	face := 0
	for _, height := range vertexHeights(scene.body, entry.Pose) {
		require.GreaterOrEqual(t, height.Sign(), 0)
		if height.Cmp(residual) <= 0 {
			face++
		}
	}
	require.GreaterOrEqual(t, face, 3, "the body rests on a face")
	slices := dynamics.TraceSliceProofs(last.Trace)
	require.NotEmpty(t, slices)
	for _, proof := range slices[len(slices)-1] {
		if proof.Pair.A != scene.floor || proof.Pair.B != scene.body || proof.BoxClear {
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
	impacts, last := runTumbleRest(t, scene, 256)
	require.Positive(t, impacts[0], "a vertex impact")
	require.Positive(t, impacts[2], "a face impact")
	require.Len(t, last.Events, 1)
	require.Len(t, last.Events[0].Manifold.Points, 6)
	requireTumbleRest(t, scene, last)
}
