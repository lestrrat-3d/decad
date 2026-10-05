package dynamics_test

import (
	"context"
	"math"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These tests drive the islands and the frictionless certified solver of a
// world of four or more bodies (docs/multibody-dynamics-design.md §6.1–§6.3
// and §6.6, §13 PR 4) through the real producers: Body.MassProperties for
// mass, SweepPair for every initial manifold and continuation, and the
// step's own certificate for every published impulse and velocity.

// islandStepConfig is the residual set of the pyramid fixtures. MaxEvents
// covers the pyramid's nine pairs. ImpactSpeed 64 mm/s lies above the
// 9810/256 mm/s kick, so the box restitution 0.3 targets zero: the stack
// rests rather than bouncing.
func islandStepConfig() dynamics.StepConfig {
	config := pairMaterialStepConfig()
	config.ImpactSpeed = units.MillimetersPerSecond(64)
	config.MaxIterations = 4096
	config.MaxEvents = 16
	return config
}

// The 3-2-1 pyramid of docs/multibody-dynamics-design.md §2's Phase 1 scene:
// a fixed 200×200×10 mm floor with its top at z = 0, three 20 mm boxes on it
// at x = 0, 25 and 50, two boxes bridging the 5 mm gaps on top of them, and
// one box on the top row. Every coordinate is dyadic.
var pyramidBoxes = [6][3]float64{ // x0, y0, z0 of each 20 mm box
	{0, 0, 0}, {25, 0, 0}, {50, 0, 0}, // bottom row
	{12.5, 0, 20}, {37.5, 0, 20}, // bridging row
	{25, 0, 40}, // top
}

// pyramidSupports names, for each box, the bodies it rests on: -1 is the
// floor, other values index pyramidBoxes.
var pyramidSupports = [6][]int{{-1}, {-1}, {-1}, {0, 1}, {1, 2}, {3, 4}}

type pyramidScene struct {
	doc     *decad.Document
	floor   *decad.Body
	boxes   [6]*decad.Body
	world   *dynamics.World
	state   dynamics.State
	density units.Value
}

// newPyramid builds the scene with the bodies inserted in order; order[k]
// is the body inserted k-th, -1 standing for the floor.
func newPyramid(t *testing.T, order [7]int, config dynamics.StepConfig) pyramidScene {
	t.Helper()
	scene := pyramidScene{doc: decad.New(), density: units.KilogramsPerCubicMillimeter(0.001)}
	scene.floor = makeBox(t, scene.doc, -65, -90, 135, 110, -10, 10)
	for i, corner := range pyramidBoxes {
		scene.boxes[i] = makeBox(t, scene.doc, corner[0], corner[1], corner[0]+20, corner[1]+20, corner[2], 20)
	}
	box := dynamics.Material{Restitution: units.Scalar(0.3), Friction: units.Scalar(0)}
	var bodies []dynamics.RigidBody
	var entries []dynamics.BodyState
	for _, i := range order {
		body := dynamics.RigidBody{Body: scene.floor, Role: dynamics.Fixed, Material: box}
		if i >= 0 {
			body = dynamics.RigidBody{Body: scene.boxes[i], Role: dynamics.Dynamic, Density: &scene.density,
				Material: box}
		}
		bodies = append(bodies, body)
		entries = append(entries, dynamics.BodyState{Body: body.Body, Pose: r3.Identity(),
			LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)})
	}
	var err error
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: bodies, Step: config})
	require.NoError(t, err)
	scene.state, err = scene.world.NewState(entries)
	require.NoError(t, err)
	return scene
}

func gravityZ(g float64) dynamics.QuantityVec {
	return dynamics.QuantityVec{X: units.MillimetersPerSecondSquared(0), Y: units.MillimetersPerSecondSquared(0),
		Z: units.MillimetersPerSecondSquared(g)}
}

// pyramidDt is dyadic, so the kick g·dt = −9810/256 mm/s is exact.
func pyramidDt() units.Value { return units.Seconds(1.0 / 256) }

func (s pyramidScene) step(t *testing.T) *dynamics.StepReport {
	t.Helper()
	report, err := s.world.Step(t.Context(), s.state, dynamics.StepInput{Gravity: gravityZ(-9810)}, pyramidDt())
	require.NoError(t, err)
	return report
}

// supportImpulse sums the published normal impulses of the island events
// between the two bodies, as the impulse the lower one delivers upward.
func supportImpulse(t *testing.T, report *dynamics.StepReport, lower, upper *decad.Body) float64 {
	t.Helper()
	for _, event := range report.Events {
		if event.Pair != (dynamics.BodyPair{A: lower, B: upper}) && event.Pair != (dynamics.BodyPair{A: upper, B: lower}) {
			continue
		}
		require.Len(t, event.PointImpulses, len(event.Manifold.Points))
		sum := 0.0
		for i, point := range event.PointImpulses {
			require.Equal(t, units.Impulse, point.Normal.Kind())
			require.GreaterOrEqual(t, point.Normal.Base(), 0.0, "point %d", i)
			sum += point.Normal.Base()
		}
		require.InDelta(t, sum, event.NormalImpulse.Base(), 1e-9)
		// The published normal is the lower body's top face, oriented A to B.
		normal := event.Manifold.Points[0].Normal.Value
		if event.Pair.A == lower {
			require.Equal(t, r3.Vec{Z: 1}, normal)
		} else {
			require.Equal(t, r3.Vec{Z: -1}, normal)
		}
		return sum
	}
	require.Fail(t, "no island event between the two bodies")
	return 0
}

// TestIslandPyramidRestsUnderGravity is §13 PR 4's fixture. The kick gives
// every box −9810/256 mm/s, below ImpactSpeed, so every target is zero:
// the island must stop all six boxes. The nine touching pairs (three floor
// supports, four bridge supports, two top supports) form one island of 36
// manifold points, coupled through the bridging boxes; the floor attaches to
// it without being a vertex. Each box's supports must deliver exactly its own
// weight impulse m·g·dt plus what it passes on, within the certified linear
// law limit ImpulseResidual + m_hi·VelocityResidual (9e-6 kg·mm/s here, so
// 1e-5 is the slack per body).
func TestIslandPyramidRestsUnderGravity(t *testing.T) {
	scene := newPyramid(t, [7]int{-1, 0, 1, 2, 3, 4, 5}, islandStepConfig())
	report := scene.step(t)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Islands, 1)
	isl := report.Islands[0]
	require.Equal(t, units.Seconds(0), isl.Time)
	require.Equal(t, append([]*decad.Body{scene.floor}, scene.boxes[:]...), isl.Bodies)
	require.Len(t, isl.Pairs, 9)
	require.Len(t, report.Events, 9)
	require.Equal(t, []int{0, 1, 2, 3, 4, 5, 6, 7, 8}, isl.Events)
	points := 0
	for _, event := range report.Events {
		require.Equal(t, dynamics.ContactImpact, event.Kind)
		require.Equal(t, 0, event.Island)
		require.Equal(t, units.Seconds(0), event.Time)
		points += len(event.Manifold.Points)
	}
	require.Equal(t, 36, points)
	mass, err := scene.boxes[0].MassProperties(t.Context(), scene.density)
	require.NoError(t, err)
	weight := mass.Mass.Value.Base() * 9810 / 256 // 306.5625 kg·mm/s, exact
	const slack = 1e-5
	// Equilibrium of every box: supports below minus loads above equal its
	// own weight impulse.
	carried := [6]float64{}
	for i := 5; i >= 0; i-- {
		below := 0.0
		for _, support := range pyramidSupports[i] {
			lower := scene.floor
			if support >= 0 {
				lower = scene.boxes[support]
			}
			below += supportImpulse(t, report, lower, scene.boxes[i])
		}
		require.InDelta(t, weight+carried[i], below, slack*float64(6-i), "box %d", i)
		for _, support := range pyramidSupports[i] {
			if support >= 0 {
				carried[support] += supportImpulse(t, report, scene.boxes[support], scene.boxes[i])
			}
		}
	}
	// The bridging patches carry the top box: its two half-face patches sum
	// to its weight, and each bridge's two lower patches sum to its own
	// weight plus its share of the top.
	top := supportImpulse(t, report, scene.boxes[3], scene.boxes[5]) +
		supportImpulse(t, report, scene.boxes[4], scene.boxes[5])
	require.InDelta(t, weight, top, slack)
	for bridge, feet := range map[int][2]int{3: {0, 1}, 4: {1, 2}} {
		lower := supportImpulse(t, report, scene.boxes[feet[0]], scene.boxes[bridge]) +
			supportImpulse(t, report, scene.boxes[feet[1]], scene.boxes[bridge])
		require.InDelta(t, weight+supportImpulse(t, report, scene.boxes[bridge], scene.boxes[5]), lower,
			2*slack, "bridge %d", bridge)
	}
	floor := 0.0
	for i := range 3 {
		floor += supportImpulse(t, report, scene.floor, scene.boxes[i])
	}
	require.InDelta(t, 6*weight, floor, 6*slack)

	// Every box rests: exactly zero velocity and its start pose at the end.
	require.NotNil(t, report.Next)
	for i, box := range scene.boxes {
		entry, ok := report.Next.Body(box)
		require.True(t, ok)
		require.Equal(t, zeroVelocity(), entry.LinearVelocity, "box %d", i)
		require.Equal(t, zeroAngular(t), entry.AngularVelocity, "box %d", i)
		require.Equal(t, r3.Identity(), entry.Pose, "box %d", i)
	}
	// The solver report states every gate within its limit.
	solver := isl.Solver
	require.Positive(t, solver.Iterations)
	require.LessOrEqual(t, solver.NormalResidual.Base(), 1e-6)
	require.LessOrEqual(t, solver.LinearResidual.Base(), 9e-6)
	require.Equal(t, units.Impulse, solver.LinearResidual.Kind())
	require.Equal(t, units.AngularMomentum, solver.AngularResidual.Kind())
	require.LessOrEqual(t, solver.EnergyResidual.Base(), 0.0, "the stop removes kinetic energy")
	require.Equal(t, 0.0, solver.PenetrationResidual.Base())

	// The floor delivers the whole stack's weight impulse; the dynamic
	// pairs cancel in the world total.
	require.NotNil(t, report.Conservation)
	require.InDelta(t, 6*weight, report.Conservation.ContactImpulse.Value.Z.Base(), 6*slack)
	require.InDelta(t, 0, report.Conservation.Completion.KineticEnergy.Value.Base(), 1e-9)

	// Replay: the event's post state at zero, the resting poses inside the
	// slice and at its end.
	for _, seconds := range []float64{0, 1.0 / 512, 1.0 / 256} {
		sample, err := report.Trace.Sample(units.Seconds(seconds))
		require.NoError(t, err, "sample at %g s", seconds)
		for i, box := range scene.boxes {
			entry, ok := sample.Body(box)
			require.True(t, ok)
			require.Equal(t, r3.Identity(), entry.Pose, "box %d at %g s", i, seconds)
			require.Equal(t, zeroVelocity(), entry.LinearVelocity, "box %d at %g s", i, seconds)
		}
	}
	// Each continued pair rests on a persistent track over the whole slice.
	proofs := dynamics.TraceSliceProofs(report.Trace)
	require.Len(t, proofs, 1)
	touching := 0
	for _, proof := range proofs[0] {
		if proof.Sweep != nil && proof.Sweep.Outcome == decad.SweepPersistentTouch {
			touching++
		}
	}
	require.Equal(t, 9, touching)
}

// TestIslandPyramidIgnoresInsertionOrder permutes world insertion order:
// the island holds the same bodies and pairs, and every box's equilibrium
// holds. Individual point impulses may differ, since the four-corner
// patches leave the split statically indeterminate and the solve order
// follows world order.
func TestIslandPyramidIgnoresInsertionOrder(t *testing.T) {
	for _, order := range [][7]int{{5, 4, 3, 2, 1, 0, -1}, {3, -1, 5, 0, 4, 2, 1}} {
		scene := newPyramid(t, order, islandStepConfig())
		report := scene.step(t)
		require.Equal(t, dynamics.Advanced, report.Status, "order %v: %+v", order, report.Diagnostics)
		require.Len(t, report.Islands, 1)
		require.Len(t, report.Islands[0].Pairs, 9)
		require.ElementsMatch(t, append([]*decad.Body{scene.floor}, scene.boxes[:]...), report.Islands[0].Bodies)
		mass, err := scene.boxes[0].MassProperties(t.Context(), scene.density)
		require.NoError(t, err)
		weight := mass.Mass.Value.Base() * 9810 / 256
		top := supportImpulse(t, report, scene.boxes[3], scene.boxes[5]) +
			supportImpulse(t, report, scene.boxes[4], scene.boxes[5])
		require.InDelta(t, weight, top, 1e-5, "order %v", order)
		floor := 0.0
		for i := range 3 {
			floor += supportImpulse(t, report, scene.floor, scene.boxes[i])
		}
		require.InDelta(t, 6*weight, floor, 6e-5, "order %v", order)
		for i, box := range scene.boxes {
			entry, ok := report.Next.Body(box)
			require.True(t, ok)
			require.Equal(t, zeroVelocity(), entry.LinearVelocity, "order %v box %d", order, i)
		}
	}
}

// twoSphereIslandScene is the two-sphere 37.5 kg·mm/s fixture of
// sphere_normal_step_test.go run through the general path: a world of four
// bodies, the touching pair plus two spheres 500 and 1000 mm away along X
// that the broad phase excludes. Sphere A rests at the origin; sphere B
// touches it at (6, 8, 0) moving at (−30, −40, 0) mm/s; all four carry the
// exact supplied 1 kg mass. With restitution 0.5 the closing normal speed
// 50 mm/s targets 25 mm/s, so J = (25 + 50)/(1 + 1) = 37.5 kg·mm/s.
type twoSphereIslandScene struct {
	doc      *decad.Document
	a, b     *decad.Body
	far      [2]*decad.Body
	world    *dynamics.World
	state    dynamics.State
	poseB    r3.Transform
	velocity dynamics.QuantityVec
}

func newTwoSphereIsland(t *testing.T, mass decad.MassProperties, config dynamics.StepConfig) twoSphereIslandScene {
	t.Helper()
	scene := twoSphereIslandScene{doc: decad.New(), velocity: dynamics.QuantityVec{
		X: units.MillimetersPerSecond(-30), Y: units.MillimetersPerSecond(-40), Z: units.MillimetersPerSecond(0)}}
	scene.a, scene.b = makeBall(t, scene.doc), makeBall(t, scene.doc)
	scene.far = [2]*decad.Body{makeBall(t, scene.doc), makeBall(t, scene.doc)}
	var err error
	scene.poseB, err = r3.Translation(r3.Vec{X: 6, Y: 8})
	require.NoError(t, err)
	material := dynamics.Material{Restitution: units.Scalar(.5), Friction: units.Scalar(0)}
	bodies := []*decad.Body{scene.a, scene.b, scene.far[0], scene.far[1]}
	config.MaxIterations = 8
	cfg := dynamics.WorldConfig{Step: config}
	for _, body := range bodies {
		cfg.Bodies = append(cfg.Bodies, dynamics.RigidBody{Body: body, Role: dynamics.Dynamic, Supplied: &mass,
			Material: material})
	}
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, cfg)
	require.NoError(t, err)
	far0, err := r3.Translation(r3.Vec{X: 500})
	require.NoError(t, err)
	far1, err := r3.Translation(r3.Vec{X: 1000})
	require.NoError(t, err)
	scene.state, err = scene.world.NewState([]dynamics.BodyState{
		{Body: scene.a, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: scene.b, Pose: scene.poseB, LinearVelocity: scene.velocity, AngularVelocity: zeroAngular(t)},
		{Body: scene.far[0], Pose: far0, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: scene.far[1], Pose: far1, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
	})
	require.NoError(t, err)
	return scene
}

func (s twoSphereIslandScene) step(t *testing.T) *dynamics.StepReport {
	t.Helper()
	report, err := s.world.Step(t.Context(), s.state, dynamics.StepInput{Gravity: zeroAcceleration()},
		units.Seconds(.1))
	require.NoError(t, err)
	return report
}

// TestIslandTwoSphereImpactThroughGeneralPath asserts the numbers
// TestSourceSpherePairInitialDiagonalImpact asserts for the two-body world,
// here produced by the island solver: the 37.5 kg·mm/s impulse, the post
// velocities, the momentum, and the replayed positions.
func TestIslandTwoSphereImpactThroughGeneralPath(t *testing.T) {
	scene := newTwoSphereIsland(t, exactSphereMass(), pairMaterialStepConfig())
	report := scene.step(t)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Islands, 1)
	require.Equal(t, []*decad.Body{scene.a, scene.b}, report.Islands[0].Bodies)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Equal(t, dynamics.ContactImpact, event.Kind)
	require.Equal(t, dynamics.BodyPair{A: scene.a, B: scene.b}, event.Pair)
	require.Equal(t, units.Seconds(0), event.Time)
	require.Equal(t, event.Bracket.From, event.Bracket.To)
	require.InDelta(t, 37.5, event.NormalImpulse.Base(), 1e-6)
	require.Len(t, event.PointImpulses, 1)
	require.InDelta(t, 37.5, event.PointImpulses[0].Normal.Base(), 1e-6)
	require.NotNil(t, event.Solver)
	require.LessOrEqual(t, event.Solver.NormalResidual.Base(), 1e-6)
	require.NotNil(t, report.Conservation)
	require.InDelta(t, -30, report.Conservation.Completion.LinearMomentum.Value.X.Base(), 1e-6)
	require.InDelta(t, -40, report.Conservation.Completion.LinearMomentum.Value.Y.Base(), 1e-6)
	// A dynamic pair delivers no external impulse.
	require.InDelta(t, 0, report.Conservation.ContactImpulse.Value.X.Base(), 1e-12)
	for _, sample := range []struct {
		at        float64
		positionA r3.Vec
		positionB r3.Vec
	}{{0, r3.Vec{}, r3.Vec{X: 6, Y: 8}},
		{.05, r3.Vec{X: -1.125, Y: -1.5}, r3.Vec{X: 5.625, Y: 7.5}},
		{.1, r3.Vec{X: -2.25, Y: -3}, r3.Vec{X: 5.25, Y: 7}}} {
		state, err := report.Trace.Sample(units.Seconds(sample.at))
		require.NoError(t, err)
		stateA, ok := state.Body(scene.a)
		require.True(t, ok)
		stateB, ok := state.Body(scene.b)
		require.True(t, ok)
		require.InDelta(t, sample.positionA.X, stateA.Pose.Translation().X, 1e-8)
		require.InDelta(t, sample.positionA.Y, stateA.Pose.Translation().Y, 1e-8)
		require.InDelta(t, sample.positionB.X, stateB.Pose.Translation().X, 1e-8)
		require.InDelta(t, sample.positionB.Y, stateB.Pose.Translation().Y, 1e-8)
		require.InDelta(t, -22.5, stateA.LinearVelocity.X.Base(), 1e-6)
		require.InDelta(t, -30, stateA.LinearVelocity.Y.Base(), 1e-6)
		require.InDelta(t, -7.5, stateB.LinearVelocity.X.Base(), 1e-6)
		require.InDelta(t, -10, stateB.LinearVelocity.Y.Base(), 1e-6)
		if sample.at > 0 {
			pair, err := scene.doc.ContactPair(t.Context(), scene.a, scene.b, stateA.Pose, stateB.Pose,
				pairMaterialStepConfig().Contact)
			require.NoError(t, err)
			require.Equal(t, decad.ContactSeparated, pair.Relation)
		}
	}
	// The distant spheres are excluded by their swept boxes and stay put.
	for i, body := range scene.far {
		start, _ := scene.state.Body(body)
		end, ok := report.Next.Body(body)
		require.True(t, ok)
		require.Equal(t, start.Pose, end.Pose, "far sphere %d", i)
	}
}

// Gates and legs of the island certificate (docs/multibody-dynamics-design.md
// §6.3, frictionless rows), each shown to fail by deleting it in
// island_certify.go, watching the named test go red, and restoring it:
//   - linear law: TestIslandCertificateGates/"linear law" and
//     TestIslandUncertainMassRefuses (the latter also with the mass interval
//     replaced by its nominal value);
//   - angular law: TestIslandCertificateGates/"angular law";
//   - normal sign: TestIslandCertificateGates/"normal sign";
//   - restitution target (the straddle refusal): TestIslandImpactSpeedStraddleRefuses;
//   - non-penetration: TestIslandCertificateGates/"non-penetration";
//   - complementarity: TestIslandCertificateGates/"complementarity";
//   - energy: TestIslandCertificateGates/"energy";
//   - linear and angular momentum: TestIslandCertificateGates/"linear
//     momentum" and "angular momentum".
//
// The momentum gates are implied by the per-body laws: summing m·Δv − ΣJ
// over the island cancels each dynamic pair's two impulses, and summing
// I·Δω + c×m·Δv gives Σ(angular-law residual) + Σ c×(linear-law residual),
// which the gate's limit covers term by term. They are shown to fail only on
// a tampered proposal that the laws refuse as well.
//
// Legs not shown to fail: the inertia component bounds, the mass-center and
// witness balls, and the orthonormality-defect widening enter only through
// a velocity or spin change times a bound of about 1e-14 relative, or are
// exactly zero (identity poses, exact box witnesses and centers), so no
// face fixture of this PR can observe them; the normal ball of the sphere
// fixture is of the same order. Each is the interval the producer states
// and widens toward refusal only. The limits ImpulseResidual,
// m_hi·VelocityResidual, ImpulseResidual·ρ and λ_lo·AngularVelocityResidual
// only admit a rounding residual; deleting one tightens the gate and can
// never admit a proposal.

// TestIslandCertificateGates tampers with the certified proposal of the
// two-sphere island and requires the named gate among the refusals. The
// untampered proposal passes every gate.
func TestIslandCertificateGates(t *testing.T) {
	scene := newTwoSphereIsland(t, exactSphereMass(), pairMaterialStepConfig())
	gates := func(tamper func(*dynamics.IslandProposal)) []string {
		t.Helper()
		names, err := dynamics.IslandProposalGates(t.Context(), scene.world, scene.state, zeroAcceleration(),
			units.Seconds(.1), tamper)
		require.NoError(t, err)
		return names
	}
	require.Empty(t, gates(func(*dynamics.IslandProposal) {}))
	shift := func(v dynamics.QuantityVec, x float64) dynamics.QuantityVec {
		v.X = units.MillimetersPerSecond(v.X.Base() + x)
		return v
	}
	for _, tc := range []struct {
		gate   string
		tamper func(*dynamics.IslandProposal)
	}{
		// B leaves 1e-3 mm/s faster along X than its impulse allows.
		{"linear law", func(p *dynamics.IslandProposal) { p.Linear[1] = shift(p.Linear[1], 1e-3) }},
		{"complementarity", func(p *dynamics.IslandProposal) { p.Linear[1] = shift(p.Linear[1], 1e-3) }},
		{"linear momentum", func(p *dynamics.IslandProposal) { p.Linear[1] = shift(p.Linear[1], 1e-3) }},
		// A spins with no torque to explain it.
		{"angular law", func(p *dynamics.IslandProposal) { p.Angular[0].Z = units.RadiansPerSecond(1e-3) }},
		{"angular momentum", func(p *dynamics.IslandProposal) { p.Angular[0].Z = units.RadiansPerSecond(1e-3) }},
		{"normal sign", func(p *dynamics.IslandProposal) { p.Lambda[0] = -p.Lambda[0] }},
		// B keeps A's velocity: the pair closes at 25 mm/s below its target.
		{"non-penetration", func(p *dynamics.IslandProposal) { p.Linear[1] = p.Linear[0] }},
		// Twice the impulse, applied consistently: the laws hold, the pair
		// separates at 75 mm/s, and kinetic energy grows from 1250 to 3125.
		{"energy", func(p *dynamics.IslandProposal) {
			p.Lambda[0] = 75
			p.Linear[0] = dynamics.QuantityVec{X: units.MillimetersPerSecond(-45),
				Y: units.MillimetersPerSecond(-60), Z: units.MillimetersPerSecond(0)}
			p.Linear[1] = dynamics.QuantityVec{X: units.MillimetersPerSecond(15),
				Y: units.MillimetersPerSecond(20), Z: units.MillimetersPerSecond(0)}
		}},
	} {
		t.Run(tc.gate, func(t *testing.T) {
			require.Contains(t, gates(tc.tamper), tc.gate)
		})
	}
}

// TestIslandUncertainMassRefuses gives every sphere a supplied mass known
// only to 2^-10 kg. The impulse changes B's momentum by (22.5, 30) mm/s
// times a mass anywhere in that interval, about 0.03 kg·mm/s wide, far past
// the linear law's 2e-6 kg·mm/s limit, so the island is refused.
func TestIslandUncertainMassRefuses(t *testing.T) {
	mass := exactSphereMass()
	mass.Mass.Bound = units.Kilograms(1.0 / 1024)
	mass.Mass.Exactness = decad.Approximate
	scene := newTwoSphereIsland(t, mass, pairMaterialStepConfig())
	report := scene.step(t)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepIslandResidual, report.Diagnostics[0].Code)
	require.Contains(t, report.Diagnostics[0].Reason, "linear law")
}

// TestIslandImpactSpeedStraddleRefuses sets ImpactSpeed to the pair's
// nominal 50 mm/s closing speed. The normal ball of the sphere manifold
// makes the enclosed speed straddle −ImpactSpeed, so neither the restitution
// target nor the zero target can be selected for every admitted normal.
func TestIslandImpactSpeedStraddleRefuses(t *testing.T) {
	config := pairMaterialStepConfig()
	config.ImpactSpeed = units.MillimetersPerSecond(50)
	scene := newTwoSphereIsland(t, exactSphereMass(), config)
	report := scene.step(t)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepIslandResidual, report.Diagnostics[0].Code)
	require.Contains(t, report.Diagnostics[0].Reason, "restitution target")
	// Below the closing speed the restitution target applies unambiguously.
	config.ImpactSpeed = units.MillimetersPerSecond(49)
	scene = newTwoSphereIsland(t, exactSphereMass(), config)
	report = scene.step(t)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.InDelta(t, 37.5, report.Events[0].NormalImpulse.Base(), 1e-6)
}

// TestIslandBudgets exhausts the island's two budgets on the pyramid: two
// sweeps cannot certify its 36 coupled points, and its nine events exceed
// a MaxEvents of eight. Neither refusal changes the document.
func TestIslandBudgets(t *testing.T) {
	config := islandStepConfig()
	config.MaxIterations = 2
	scene := newPyramid(t, [7]int{-1, 0, 1, 2, 3, 4, 5}, config)
	bodies := scene.doc.Bodies()
	report := scene.step(t)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepIslandResidual, report.Diagnostics[0].Code)
	require.Equal(t, bodies, scene.doc.Bodies())

	config = islandStepConfig()
	config.MaxEvents = 8
	scene = newPyramid(t, [7]int{-1, 0, 1, 2, 3, 4, 5}, config)
	report = scene.step(t)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepEventBudget, report.Diagnostics[0].Code)
}

// TestIslandCancellation cancels the pyramid step at every context poll:
// before the kick, in either slice's box and sweep loops, and between the
// island solves. Each returns the context error and no report.
func TestIslandCancellation(t *testing.T) {
	scene := newPyramid(t, [7]int{-1, 0, 1, 2, 3, 4, 5}, islandStepConfig())
	input := dynamics.StepInput{Gravity: gravityZ(-9810)}
	var calls atomic.Int64
	report, err := scene.world.Step(countdownContext{Context: t.Context(), calls: &calls, limit: math.MaxInt64},
		scene.state, input, pyramidDt())
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status)
	polls := calls.Load()
	for limit := int64(0); limit < polls; limit += 7 {
		var calls atomic.Int64
		report, err := scene.world.Step(countdownContext{Context: t.Context(), calls: &calls, limit: limit},
			scene.state, input, pyramidDt())
		require.ErrorIs(t, err, context.Canceled, "cancelled at poll %d of %d", limit, polls)
		require.Nil(t, report, "cancelled at poll %d of %d", limit, polls)
	}
}

// correctionScene places one 20 mm box sunk depth mm into the fixed floor,
// a fixed ceiling box ceilingGap mm above its top (none when negative), a
// second dynamic box resting exactly on it when stacked, and distant fixed
// boxes so the world has at least four bodies.
type correctionScene struct {
	doc            *decad.Document
	floor, box     *decad.Body
	ceiling, upper *decad.Body
	world          *dynamics.World
	state          dynamics.State
}

func newCorrectionScene(t *testing.T, depth, ceilingGap float64, stacked bool,
	config dynamics.StepConfig) correctionScene {
	t.Helper()
	scene := correctionScene{doc: decad.New()}
	scene.floor = makeBox(t, scene.doc, -65, -90, 135, 110, -10, 10)
	scene.box = makeBox(t, scene.doc, 0, 0, 20, 20, 0, 20)
	density := units.KilogramsPerCubicMillimeter(0.001)
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	sunk, err := r3.Translation(r3.Vec{Z: -depth})
	require.NoError(t, err)
	bodies := []dynamics.RigidBody{{Body: scene.floor, Role: dynamics.Fixed, Material: material},
		{Body: scene.box, Role: dynamics.Dynamic, Density: &density, Material: material}}
	entries := []dynamics.BodyState{
		{Body: scene.floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)},
		{Body: scene.box, Pose: sunk, LinearVelocity: zeroVelocity(), AngularVelocity: zeroAngular(t)}}
	add := func(body *decad.Body, role dynamics.BodyRole, pose r3.Transform) {
		entry := dynamics.RigidBody{Body: body, Role: role, Material: material}
		if role == dynamics.Dynamic {
			entry.Density = &density
		}
		bodies = append(bodies, entry)
		entries = append(entries, dynamics.BodyState{Body: body, Pose: pose, LinearVelocity: zeroVelocity(),
			AngularVelocity: zeroAngular(t)})
	}
	if ceilingGap >= 0 {
		scene.ceiling = makeBox(t, scene.doc, 0, 0, 20, 20, 20+ceilingGap-depth, 10)
		add(scene.ceiling, dynamics.Fixed, r3.Identity())
	}
	if stacked {
		scene.upper = makeBox(t, scene.doc, 0, 0, 20, 20, 20, 20)
		add(scene.upper, dynamics.Dynamic, sunk)
	}
	for len(bodies) < 4 {
		x := 100 + 30*float64(len(bodies))
		add(makeBox(t, scene.doc, x, 0, x+10, 10, 0, 10), dynamics.Fixed, r3.Identity())
	}
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: bodies, Step: config})
	require.NoError(t, err)
	scene.state, err = scene.world.NewState(entries)
	require.NoError(t, err)
	return scene
}

func (s correctionScene) step(t *testing.T) *dynamics.StepReport {
	t.Helper()
	report, err := s.world.Step(t.Context(), s.state, dynamics.StepInput{Gravity: gravityZ(-9810)}, pyramidDt())
	require.NoError(t, err)
	return report
}

// correctionDepth is 2^-21 mm, inside the 1e-6 mm ContactSlop allowance.
const correctionDepth = 1.0 / (1 << 21)

// TestIslandCorrectsInitialPenetration is §6.6 at an initial contact: the
// box starts 2^-21 mm inside the floor, the overlap manifold reports that
// depth, and the box alone takes the whole correction, so it ends exactly
// on the floor at rest. The event records the correction, and the slice
// replays the corrected pose.
//
// Legs shown to fail (each deleted in island.go, this test or the named one
// watched go red, then restored):
//   - the correction itself (no translation): the continuation sweep starts
//     overlapping and the step stops;
//   - ContactSlop in the allowance: this fixture is refused, since its
//     exact witnesses carry no geometry bound;
//   - the touching check of corrected island pairs:
//     TestIslandCorrectionRefusals/"stacked" reports StepUnsupported from
//     the continuation instead of StepCorrectionFailed;
//   - the correction sweep of other pairs: TestIslandCorrectionRefusals/
//     "ceiling" likewise.
//
// The bracket-travel leg is zero at an initial contact and the geometry leg
// is zero for exact boxes; neither can be observed here.
func TestIslandCorrectsInitialPenetration(t *testing.T) {
	scene := newCorrectionScene(t, correctionDepth, -1, false, islandStepConfig())
	report := scene.step(t)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Equal(t, dynamics.BodyPair{A: scene.floor, B: scene.box}, event.Pair)
	require.Equal(t, r3.Vec{}, event.PositionChangeA)
	require.Equal(t, r3.Vec{Z: correctionDepth}, event.PositionChangeB)
	require.Equal(t, correctionDepth, report.Islands[0].Solver.PenetrationResidual.Base())
	// The box's weight impulse is m·g·dt = 8·9810/256 kg·mm/s.
	require.InDelta(t, 8*9810.0/256, event.NormalImpulse.Base(), 1e-5)
	entry, ok := report.Next.Body(scene.box)
	require.True(t, ok)
	require.Equal(t, r3.Identity(), entry.Pose)
	require.Equal(t, zeroVelocity(), entry.LinearVelocity)
	contact, err := scene.doc.ContactPair(t.Context(), scene.floor, scene.box, r3.Identity(), entry.Pose,
		islandStepConfig().Contact)
	require.NoError(t, err)
	require.Equal(t, decad.ContactTouching, contact.Relation)
	for _, seconds := range []float64{0, 1.0 / 512} {
		sample, err := report.Trace.Sample(units.Seconds(seconds))
		require.NoError(t, err)
		box, ok := sample.Body(scene.box)
		require.True(t, ok)
		require.Equal(t, r3.Identity(), box.Pose, "at %g s", seconds)
	}
}

// TestIslandCorrectionRefusals: a penetration beyond the allowance, a
// correction that drives the box into a box stacked on it, and one that
// lifts it onto a fixed ceiling exactly the correction depth above it are
// all StepCorrectionFailed.
func TestIslandCorrectionRefusals(t *testing.T) {
	for _, tc := range []struct {
		name    string
		depth   float64
		ceiling float64
		stacked bool
	}{
		{"too deep", 4 * correctionDepth, -1, false},
		{"stacked", correctionDepth, -1, true},
		{"ceiling", correctionDepth, correctionDepth, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			scene := newCorrectionScene(t, tc.depth, tc.ceiling, tc.stacked, islandStepConfig())
			report := scene.step(t)
			require.Equal(t, dynamics.Undecided, report.Status)
			require.Nil(t, report.Next)
			require.NotEmpty(t, report.Diagnostics)
			require.Equal(t, dynamics.StepCorrectionFailed, report.Diagnostics[0].Code, "%+v", report.Diagnostics)
		})
	}
}
