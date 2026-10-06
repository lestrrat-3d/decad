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

// Legs of the push that proves a sub-ulp gap and of the anchored correction
// (docs/multibody-dynamics-design.md §6.6), each shown to fail by deleting it
// in island.go, watching the named tests go red, and restoring it:
//   - the push of a separating pair that reads ContactNoGapProof:
//     TestIslandSphereLandsOnSphere, TestIslandSphereGlancesOffSphere and
//     TestPushApartProvesASubUlpGap refuse with "corrected pair relation is
//     0";
//   - its doubling search: TestPushApartProvesASubUlpGap, whose spheres need
//     two ulps, finds no provable separation;
//   - its allowance: TestPushApartProvesASubUlpGap admits the 1e-16 mm
//     allowance;
//   - the anchored group (a body resting on the floor through a persistent
//     track takes no share of another pair's correction):
//     TestIslandSphereLandsOnSphere pushes the lower sphere into the floor
//     and refuses with "corrected pair relation is 3";
//   - a separated pair leaving the contact set:
//     TestIslandSphereGlancesOffSphere refuses its next slice, whose
//     rotating sphere-pair sweep under ContinueSeparatingTouch needs a
//     touching start.

// sphereColumn is the sphere column of the Phase 1 stack-and-drop scene
// (docs/multibody-dynamics-design.md §2) reduced to two spheres: a fixed
// 200×200×10 mm floor with its top at z = 0, two 8 mm source spheres
// (density 0.001 kg/mm³) released at rest with their centers at z = 60 and
// z = 90 over x = 120, and one fixed box 1000 mm away along Y. The spheres
// carry restitution 0.6 and the boxes 0.3, so the floor pair mixes to 0.3;
// friction is 0.4 everywhere; gravity −9810 mm/s², steps of 1/256 s.
type sphereColumn struct {
	doc          *decad.Document
	floor        *decad.Body
	lower, upper *decad.Body
	top          *decad.Body
	timeline     *dynamics.Timeline
	config       dynamics.StepConfig
}

func newSphereColumn(t *testing.T, y [3]float64) *sphereColumn {
	t.Helper()
	scene := &sphereColumn{doc: decad.New()}
	scene.floor = makeBox(t, scene.doc, -20, -90, 180, 110, -10, 10)
	scene.lower, scene.upper, scene.top = sceneBall(t, scene.doc), sceneBall(t, scene.doc),
		sceneBall(t, scene.doc)
	far := makeBox(t, scene.doc, 115, 995, 125, 1005, 0, 10)
	density := units.KilogramsPerCubicMillimeter(.001)
	ball := dynamics.Material{Restitution: units.Scalar(.6), Friction: units.Scalar(.4)}
	box := dynamics.Material{Restitution: units.Scalar(.3), Friction: units.Scalar(.4)}
	scene.config = pairMaterialStepConfig()
	scene.config.ImpactSpeed = units.MillimetersPerSecond(64)
	scene.config.MaxIterations, scene.config.MaxEvents = 4096, 64
	world, err := dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: scene.floor, Role: dynamics.Fixed, Material: box},
		{Body: scene.lower, Role: dynamics.Dynamic, Density: &density, Material: ball},
		{Body: scene.upper, Role: dynamics.Dynamic, Density: &density, Material: ball},
		{Body: scene.top, Role: dynamics.Dynamic, Density: &density, Material: ball},
		{Body: far, Role: dynamics.Fixed, Material: box},
	}, Step: scene.config})
	require.NoError(t, err)
	at := func(i int) r3.Transform {
		pose, err := r3.Translation(r3.Vec{X: 120, Y: y[i], Z: 60 + 30*float64(i)})
		require.NoError(t, err)
		return pose
	}
	state, err := world.NewState([]dynamics.BodyState{
		{Body: scene.floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: scene.lower, Pose: at(0), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: scene.upper, Pose: at(1), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: scene.top, Pose: at(2), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: far, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	scene.timeline, err = dynamics.NewTimeline(world, state)
	require.NoError(t, err)
	return scene
}

// sceneBall is the scene's 8 mm source sphere centered at the origin.
func sceneBall(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	const radius = 8
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	left := s.CreatePoint(-radius, 0)
	s.Fix(left)
	right := s.CreatePoint(radius, 0)
	center := s.CreatePoint(0, 0)
	s.CreateLine(left, right)
	s.CreateArc(center, right, left)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ball, err := doc.Revolve(s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return ball
}

// TestIslandSphereLandsOnSphere drops the column for half a second (128
// steps): the lower sphere bounces on the floor, the upper one lands on it,
// and the two bounce and settle on the floor in a column. Every sphere-pair
// impact above ImpactSpeed leaves with the restitution target −0.6 times the
// incoming relative normal speed, and the two spheres never overlap at a
// step end by more than PenetrationResidual. The landing's correction leaves
// the pair apart by less than ContactPair can prove; the step pushes it
// provably apart within its allowance.
func TestIslandSphereLandsOnSphere(t *testing.T) {
	t.Parallel()
	scene := newSphereColumn(t, [3]float64{})
	impacts := 0
	for step := range 48 {
		report, err := scene.timeline.Advance(t.Context(), dynamics.StepInput{Gravity: gravityZ(-9810)},
			units.Seconds(1.0/256))
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", step, report.Diagnostics)
		for _, event := range report.Events {
			if event.Pair != (dynamics.BodyPair{A: scene.lower, B: scene.upper}) || event.Kind != dynamics.ContactImpact {
				continue
			}
			before := event.PreVelocityB.Z.Base() - event.PreVelocityA.Z.Base()
			after := event.PostVelocityB.Z.Base() - event.PostVelocityA.Z.Base()
			if -before > scene.config.ImpactSpeed.Base() {
				impacts++
				require.InDelta(t, -.6*before, after, 1e-5, "step %d", step)
			}
		}
		lower, ok := report.Next.Body(scene.lower)
		require.True(t, ok)
		upper, ok := report.Next.Body(scene.upper)
		require.True(t, ok)
		gap := upper.Pose.Translation().Z - lower.Pose.Translation().Z - 16
		require.GreaterOrEqual(t, gap, -scene.config.PenetrationResidual.Base(), "step %d", step)
	}
	require.Positive(t, impacts)
	end, err := scene.timeline.Sample(scene.timeline.End())
	require.NoError(t, err)
	lower, ok := end.Body(scene.lower)
	require.True(t, ok)
	upper, ok := end.Body(scene.upper)
	require.True(t, ok)
	require.InDelta(t, 120, upper.Pose.Translation().X, 1e-9)
	require.InDelta(t, 120, lower.Pose.Translation().X, 1e-9)
}

// TestPushApartProvesASubUlpGap places two 8 mm spheres 16 mm + 2^-80 mm²
// worth of squared distance apart: B's center sits at (16, 2^-40, 0), so
// the pair is exactly separated but ContactPair can prove no gap
// (ContactNoGapProof). Pushed apart along X as a separating pair, the
// spheres need two ulps of travel at 16 mm before a gap can be proved: a
// 1e-9 mm allowance admits that, a 1e-16 mm one refuses it.
func TestPushApartProvesASubUlpGap(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a, b := sceneBall(t, doc), sceneBall(t, doc)
	far, farther := farBox(t, doc, 1000), farBox(t, doc, 2000)
	mass := exactSphereMass()
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	config := pairMaterialStepConfig()
	world, err := dynamics.NewWorld(t.Context(), doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: a, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		{Body: b, Role: dynamics.Dynamic, Supplied: &mass, Material: material},
		{Body: far, Role: dynamics.Fixed, Material: material},
		{Body: farther, Role: dynamics.Fixed, Material: material},
	}, Step: config})
	require.NoError(t, err)
	poseB, err := r3.Translation(r3.Vec{X: 16, Y: 0x1p-40})
	require.NoError(t, err)
	state, err := world.NewState([]dynamics.BodyState{
		{Body: a, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: b, Pose: poseB, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: far, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: farther, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	contact, err := doc.ContactPair(t.Context(), a, b, r3.Identity(), poseB, config.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactUndecided, contact.Relation)
	require.Equal(t, decad.ContactNoGapProof, contact.Reason)
	allow := func(limit float64) map[*decad.Body]units.Value {
		return map[*decad.Body]units.Value{a: units.Millimeters(limit), b: units.Millimeters(limit)}
	}
	reason, pushed, err := dynamics.PushApart(t.Context(), world, state, state, a, b, allow(1e-9), r3.Vec{X: 1})
	require.NoError(t, err)
	require.Empty(t, reason)
	endA, _ := pushed.Body(a)
	endB, _ := pushed.Body(b)
	contact, err = doc.ContactPair(t.Context(), a, b, endA.Pose, endB.Pose, config.Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactSeparated, contact.Relation)
	gap := endB.Pose.Translation().X - endA.Pose.Translation().X - 16
	require.Positive(t, gap)
	require.LessOrEqual(t, gap, 0x1p-46)
	require.Zero(t, endA.Pose.Translation().Y)
	reason, _, err = dynamics.PushApart(t.Context(), world, state, state, a, b, allow(1e-16), r3.Vec{X: 1})
	require.NoError(t, err)
	require.Contains(t, reason, "exceeds its correction allowance")
}

// TestIslandSphereGlancesOffSphere offsets the column along Y (0, 4 and
// −4 mm): the upper sphere strikes the lower one off its center line, so
// friction sets both spinning, and the pair separates. The correction leaves
// the pair apart, out of the contact set, so the next slice sweeps it from a
// clear start. The two spinning spheres then meet again, and the top sphere
// lands on both: each of these impacts between spinning spheres is
// bracketed by the root package's rotating sphere-pair sweep. A spinning
// ball's center follows its drift's straight line, so the exact first-touch
// time of the two center lines must lie inside the event's bracket. The
// first 80 steps (0.3125 s) advance; the top sphere then rolls off the
// floor's edge.
func TestIslandSphereGlancesOffSphere(t *testing.T) {
	t.Parallel()
	scene := newSphereColumn(t, [3]float64{0, 4, -4})
	spheres := map[*decad.Body]struct{}{scene.lower: {}, scene.upper: {}, scene.top: {}}
	spun, spinning := false, 0
	previous, err := scene.timeline.Sample(units.Seconds(0))
	require.NoError(t, err)
	for step := range 80 {
		report, err := scene.timeline.Advance(t.Context(), dynamics.StepInput{Gravity: gravityZ(-9810)},
			units.Seconds(1.0/256))
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", step, report.Diagnostics)
		for _, event := range report.Events {
			_, sphereA := spheres[event.Pair.A]
			_, sphereB := spheres[event.Pair.B]
			if !sphereA || !sphereB || event.Kind != dynamics.ContactImpact {
				continue
			}
			if event.Pair == (dynamics.BodyPair{A: scene.lower, B: scene.upper}) && !spun &&
				event.PostAngularVelocityB.X.Base() != 0 {
				spun = true
				// Friction spins both spheres the same way about X.
				require.Equal(t, event.PostAngularVelocityA.X.Base() > 0, event.PostAngularVelocityB.X.Base() > 0)
				require.NotZero(t, event.TangentImpulse.Y.Base())
				continue
			}
			if event.PreAngularVelocityA.X.Base() == 0 || event.PreAngularVelocityB.X.Base() == 0 ||
				event.SliceStart.Base() != 0 {
				continue
			}
			spinning++
			// Gravity kicks both spheres alike, so the relative center path
			// over the slice is the step-start offset plus the relative
			// drift velocity.
			startA, ok := previous.Body(event.Pair.A)
			require.True(t, ok)
			startB, ok := previous.Body(event.Pair.B)
			require.True(t, ok)
			root := sphereFirstTouch(startA.Pose.Translation(), startB.Pose.Translation(),
				event.PreVelocityA, event.PreVelocityB, 16)
			require.Equal(t, -1, exactSceneFloat(event.Bracket.From.Elapsed.Value.Base()).Cmp(root), "step %d", step)
			require.LessOrEqual(t, root.Cmp(exactSceneFloat(event.Bracket.To.Elapsed.Value.Base())), 0, "step %d", step)
			require.Len(t, event.Manifold.Points, 1)
			normal := event.Manifold.Points[0].Normal.Value
			relative := func(a, b dynamics.QuantityVec) float64 {
				return (b.X.Base()-a.X.Base())*normal.X + (b.Y.Base()-a.Y.Base())*normal.Y +
					(b.Z.Base()-a.Z.Base())*normal.Z
			}
			before := relative(event.PreVelocityA, event.PreVelocityB)
			require.Negative(t, before, "step %d", step)
			if -before > scene.config.ImpactSpeed.Base() {
				require.InDelta(t, -.6*before, relative(event.PostVelocityA, event.PostVelocityB), 1e-5, "step %d", step)
			}
		}
		for first := range spheres {
			for second := range spheres {
				if first == second {
					continue
				}
				a, ok := report.Next.Body(first)
				require.True(t, ok)
				b, ok := report.Next.Body(second)
				require.True(t, ok)
				require.GreaterOrEqual(t, b.Pose.Translation().Sub(a.Pose.Translation()).Len()-16,
					-scene.config.PenetrationResidual.Base(), "step %d", step)
			}
		}
		previous = *report.Next
	}
	require.True(t, spun)
	require.GreaterOrEqual(t, spinning, 2)
}

func exactSceneFloat(v float64) *big.Float { return new(big.Float).SetPrec(600).SetFloat64(v) }

// sphereFirstTouch is the closed-form earliest time at which the center
// lines a+vA·t and b+vB·t come within distance of each other.
func sphereFirstTouch(a, b r3.Vec, velocityA, velocityB dynamics.QuantityVec, distance float64) *big.Float {
	p := [3]*big.Float{exactSceneFloat(b.X), exactSceneFloat(b.Y), exactSceneFloat(b.Z)}
	w := [3]*big.Float{exactSceneFloat(velocityB.X.Base()), exactSceneFloat(velocityB.Y.Base()),
		exactSceneFloat(velocityB.Z.Base())}
	for i, v := range [3]float64{a.X, a.Y, a.Z} {
		p[i].Sub(p[i], exactSceneFloat(v))
	}
	for i, v := range [3]float64{velocityA.X.Base(), velocityA.Y.Base(), velocityA.Z.Base()} {
		w[i].Sub(w[i], exactSceneFloat(v))
	}
	dot := func(x, y [3]*big.Float) *big.Float {
		sum := exactSceneFloat(0)
		for i := range 3 {
			sum.Add(sum, new(big.Float).SetPrec(600).Mul(x[i], y[i]))
		}
		return sum
	}
	qa, qb := dot(w, w), dot(p, w)
	qc := new(big.Float).SetPrec(600).Sub(dot(p, p), exactSceneFloat(distance*distance))
	root := new(big.Float).SetPrec(600).Sub(new(big.Float).SetPrec(600).Mul(qb, qb),
		new(big.Float).SetPrec(600).Mul(qa, qc))
	root.Sqrt(root)
	root.Add(root, qb)
	root.Neg(root)
	return root.Quo(root, qa)
}
