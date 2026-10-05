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

// These tests run the Phase 1 exit scene of docs/multibody-dynamics-design.md
// §2, stack-and-drop, end to end through the real producers: Body.MassProperties
// for every mass, ContactPair and SweepPair for every manifold and sweep,
// World.Step for every event, and Timeline for the chain of steps. The
// _gallery module renders the same scene (§11.3); this copy runs it inside the
// module so CI checks it without the gallery's toolchain.

// stackAndDrop is the scene: a fixed 360×2000×10 mm source-box floor with its
// top at z = 0, spanning x = −60…300 and y = −990…1010 so the spheres, which
// glance off one another and roll along y with no rolling resistance, stay
// inside its top face for the whole 2 s; the 3-2-1 pyramid of pyramidBoxes, three radius-8 source
// spheres released at rest with centers at z = 60, 90 and 120 over x = 120,
// offset in y so the second lands on the first and the third on them, and a
// Ø20×30 source cylinder released axially with its lower disk at z = 80 over
// x = 160. Every coordinate is dyadic.
type stackAndDrop struct {
	doc      *decad.Document
	floor    *decad.Body
	boxes    [6]*decad.Body
	spheres  [3]*decad.Body
	cylinder *decad.Body
	density  units.Value
	config   dynamics.StepConfig
	world    *dynamics.World
	state    dynamics.State
}

// stackAndDropSpheres are the spheres' release centers.
var stackAndDropSpheres = [3]r3.Vec{{X: 120, Y: 10, Z: 60}, {X: 120, Y: 14, Z: 90}, {X: 120, Y: 6, Z: 120}}

// stackAndDropCylinder is the cylinder's release translation: its modeled
// lower disk is centered on the origin.
var stackAndDropCylinder = r3.Vec{X: 160, Y: 10, Z: 80}

// stackAndDropSteps is the scene's 2 s of motion in steps of pyramidDt
// (1/256 s, so every gravity kick of −9810/256 mm/s is exact).
const stackAndDropSteps = 512

// stackAndDropConfig is the scene's step configuration. ImpactSpeed 64 mm/s
// lies above the 9810/256 mm/s kick, so a body resting under gravity targets
// zero speed every step instead of bouncing, and a bounce sequence ends once
// its incoming speed falls below it (§5.1).
func stackAndDropConfig() dynamics.StepConfig {
	config := pairMaterialStepConfig()
	config.ImpactSpeed = units.MillimetersPerSecond(64)
	config.MaxIterations = 4096
	config.MaxEvents = 64
	config.MaxPairSweeps = 1 << 16
	return config
}

func sceneSphere(t *testing.T, doc *decad.Document, radius float64) *decad.Body {
	t.Helper()
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

func sceneCylinder(t *testing.T, doc *decad.Document, radius, height float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.CreateCircle(center, radius)
	s.Fix(center)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	cylinder, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	return cylinder
}

// newStackAndDrop builds the scene. Boxes rest with restitution 0.3 and
// spheres bounce with 0.6; a pair takes the smaller of its two values, so
// the floor and the boxes meet the spheres at 0.3 and the spheres meet each
// other at 0.6. Friction is 0.4 everywhere, density 0.001 kg/mm³.
func newStackAndDrop(t *testing.T) stackAndDrop {
	t.Helper()
	scene := stackAndDrop{doc: decad.New(), density: units.KilogramsPerCubicMillimeter(0.001),
		config: stackAndDropConfig()}
	scene.floor = makeBox(t, scene.doc, -60, -990, 300, 1010, -10, 10)
	for i, corner := range pyramidBoxes {
		scene.boxes[i] = makeBox(t, scene.doc, corner[0], corner[1], corner[0]+20, corner[1]+20, corner[2], 20)
	}
	for i := range scene.spheres {
		scene.spheres[i] = sceneSphere(t, scene.doc, 8)
	}
	scene.cylinder = sceneCylinder(t, scene.doc, 10, 30)
	box := dynamics.Material{Restitution: units.Scalar(0.3), Friction: units.Scalar(0.4)}
	ball := dynamics.Material{Restitution: units.Scalar(0.6), Friction: units.Scalar(0.4)}
	bodies := []dynamics.RigidBody{{Body: scene.floor, Role: dynamics.Fixed, Material: box}}
	entries := []dynamics.BodyState{{Body: scene.floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(),
		AngularVelocity: zeroAngular(t)}}
	add := func(body *decad.Body, material dynamics.Material, at r3.Vec) {
		bodies = append(bodies, dynamics.RigidBody{Body: body, Role: dynamics.Dynamic, Density: &scene.density,
			Material: material})
		pose, err := r3.Translation(at)
		require.NoError(t, err)
		entries = append(entries, dynamics.BodyState{Body: body, Pose: pose, LinearVelocity: zeroVelocity(),
			AngularVelocity: zeroAngular(t)})
	}
	for _, b := range scene.boxes {
		add(b, box, r3.Vec{})
	}
	for i, s := range scene.spheres {
		add(s, ball, stackAndDropSpheres[i])
	}
	add(scene.cylinder, box, stackAndDropCylinder)
	var err error
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc,
		dynamics.WorldConfig{Bodies: bodies, Step: scene.config})
	require.NoError(t, err)
	scene.state, err = scene.world.NewState(entries)
	require.NoError(t, err)
	return scene
}

// stackAndDropTimeline advances the scene's timeline through its 2 s,
// failing at the first Undecided step with its diagnostics.
func stackAndDropTimeline(t *testing.T, scene stackAndDrop) *dynamics.Timeline {
	t.Helper()
	timeline, err := dynamics.NewTimeline(scene.world, scene.state)
	require.NoError(t, err)
	for k := range stackAndDropSteps {
		report, err := timeline.Advance(t.Context(), dynamics.StepInput{Gravity: gravityZ(-9810)}, pyramidDt())
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", k, report.Diagnostics)
	}
	return timeline
}

// pairImpulse sums the published normal impulses of one step's events
// between two bodies, and reports whether any event joined them.
func pairImpulse(report *dynamics.StepReport, a, b *decad.Body) (float64, bool) {
	sum, found := 0.0, false
	for _, event := range report.Events {
		if event.Pair != (dynamics.BodyPair{A: a, B: b}) && event.Pair != (dynamics.BodyPair{A: b, B: a}) {
			continue
		}
		found = true
		sum += event.NormalImpulse.Base()
	}
	return sum, found
}

// TestStackAndDropScene asserts §2's Phase 1 exit criteria on the computed
// trace.
func TestStackAndDropScene(t *testing.T) {
	scene := newStackAndDrop(t)
	timeline := stackAndDropTimeline(t, scene)
	require.Nil(t, timeline.Stopped())
	require.Equal(t, units.Seconds(2), timeline.End())
	steps := timeline.Steps()
	require.Len(t, steps, stackAndDropSteps)

	boxMass, err := scene.boxes[0].MassProperties(t.Context(), scene.density)
	require.NoError(t, err)
	weight := boxMass.Mass.Value.Base() * 9810 / 256 // a box's weight impulse per step, exact
	config := scene.config

	for k, report := range steps {
		// Every step's conservation readings balance: the dynamic bodies'
		// linear momentum changes by the gravity and contact impulses, within
		// the readings' own bounds plus each island's certified linear-law
		// limit, ImpulseResidual + m_hi·VelocityResidual per body (under
		// 1e-5 kg·mm/s for these masses, so 1e-4 covers all ten bodies).
		c := report.Conservation
		require.NotNil(t, c, "step %d", k)
		for axis, get := range []func(dynamics.QuantityVec) units.Value{
			func(v dynamics.QuantityVec) units.Value { return v.X },
			func(v dynamics.QuantityVec) units.Value { return v.Y },
			func(v dynamics.QuantityVec) units.Value { return v.Z },
		} {
			change := get(c.Completion.LinearMomentum.Value).Base() - get(c.Input.LinearMomentum.Value).Base()
			applied := get(c.GravityImpulse.Value).Base() + get(c.ContactImpulse.Value).Base()
			slack := get(c.Completion.LinearMomentum.Bound).Base() + get(c.Input.LinearMomentum.Bound).Base() +
				get(c.GravityImpulse.Bound).Base() + get(c.ContactImpulse.Bound).Base() + 1e-4
			require.InDelta(t, applied, change, slack, "step %d axis %d", k, axis)
		}

		// The pyramid is solved every step, since gravity kicks its pairs
		// (§3.2). The top box's two half-face patches on the bridging boxes
		// each carry a positive share and together its weight; each bridging
		// box's two patches carry its own weight plus its share of the top,
		// within the linear-law slack of TestIslandPyramidRestsUnderGravity.
		const slack = 1e-5
		left, ok := pairImpulse(report, scene.boxes[3], scene.boxes[5])
		require.True(t, ok, "step %d", k)
		right, ok := pairImpulse(report, scene.boxes[4], scene.boxes[5])
		require.True(t, ok, "step %d", k)
		require.Positive(t, left, "step %d", k)
		require.Positive(t, right, "step %d", k)
		require.InDelta(t, weight, left+right, slack, "step %d", k)
		for bridge, share := range map[int]float64{3: left, 4: right} {
			feet := [2]int{bridge - 3, bridge - 2}
			a, ok := pairImpulse(report, scene.boxes[feet[0]], scene.boxes[bridge])
			require.True(t, ok, "step %d bridge %d", k, bridge)
			b, ok := pairImpulse(report, scene.boxes[feet[1]], scene.boxes[bridge])
			require.True(t, ok, "step %d bridge %d", k, bridge)
			require.Positive(t, a, "step %d bridge %d", k, bridge)
			require.Positive(t, b, "step %d bridge %d", k, bridge)
			require.InDelta(t, weight+share, a+b, 2*slack, "step %d bridge %d", k, bridge)
		}
	}

	// The six pyramid boxes end within PenetrationResidual of their start
	// poses: every corner of every box, so a rotation would show too.
	end, err := timeline.Sample(timeline.End())
	require.NoError(t, err)
	residual := config.PenetrationResidual.Base()
	for i, box := range scene.boxes {
		entry, ok := end.Body(box)
		require.True(t, ok)
		corner := pyramidBoxes[i]
		for _, dx := range []float64{0, 20} {
			for _, dy := range []float64{0, 20} {
				for _, dz := range []float64{0, 20} {
					p := r3.Vec{X: corner[0] + dx, Y: corner[1] + dy, Z: corner[2] + dz}
					q := entry.Pose.Apply(p)
					require.InDelta(t, p.X, q.X, residual, "box %d", i)
					require.InDelta(t, p.Y, q.Y, residual, "box %d", i)
					require.InDelta(t, p.Z, q.Z, residual, "box %d", i)
				}
			}
		}
	}

	// The cylinder ends at rest on its lower disk at its release x and y.
	cylinder, ok := end.Body(scene.cylinder)
	require.True(t, ok)
	at := cylinder.Pose.Apply(r3.Vec{})
	require.InDelta(t, stackAndDropCylinder.X, at.X, residual)
	require.InDelta(t, stackAndDropCylinder.Y, at.Y, residual)
	require.InDelta(t, 0, at.Z, residual)
	require.Equal(t, zeroVelocity(), cylinder.LinearVelocity)
	require.Equal(t, zeroAngular(t), cylinder.AngularVelocity)

	// The spheres glance off one another and end rolling along y on the
	// floor, which models no rolling resistance: each center rests 8 mm up at
	// x = 120, inside the floor's y span by more than its radius, and its
	// floor contact point is still within VelocityResidual (v_y = −ω_x·r).
	for i, sphere := range scene.spheres {
		entry, ok := end.Body(sphere)
		require.True(t, ok)
		center := entry.Pose.Translation()
		require.InDelta(t, 120, center.X, residual, "sphere %d", i)
		require.InDelta(t, 8, center.Z, residual, "sphere %d", i)
		require.Greater(t, center.Y, -990.0+16, "sphere %d", i)
		require.Less(t, center.Y, 1010.0-16, "sphere %d", i)
		slip := entry.LinearVelocity.Y.Base() + entry.AngularVelocity.X.Base()*8
		require.InDelta(t, 0, slip, config.VelocityResidual.Base(), "sphere %d", i)
		require.Zero(t, entry.LinearVelocity.Z.Base(), "sphere %d", i)
	}

	// The first sphere's first floor impact lies at the free-fall time of
	// the step's discrete law (one exact kick of −9810/256 mm/s per step, then
	// drift), within TimeResolution after it: the impact is its bracket's
	// right sample (§5.1).
	first := firstFloorImpact(t, steps, scene.floor, scene.spheres[0])
	exact := discreteFreeFall(stackAndDropSpheres[0].Z - 8)
	require.GreaterOrEqual(t, first.Cmp(exact), 0, "impact %s before free fall %s",
		first.FloatString(12), exact.FloatString(12))
	late := new(big.Rat).Sub(first, exact)
	require.LessOrEqual(t, late.Cmp(new(big.Rat).SetFloat64(config.TimeResolution.Base())), 0,
		"impact %s s after free fall", late.FloatString(15))

	// Every frame time of the 60 fps clip replays a certified state.
	for frame := range 120 {
		_, err := timeline.Sample(units.Seconds(float64(frame) / 60))
		require.NoError(t, err, "frame %d", frame)
	}
}

// firstFloorImpact is the exact timeline time of the first ContactImpact
// between floor and body.
func firstFloorImpact(t *testing.T, steps []*dynamics.StepReport, floor, body *decad.Body) *big.Rat {
	t.Helper()
	for k, report := range steps {
		for _, event := range report.Events {
			if event.Kind != dynamics.ContactImpact || event.Pair != (dynamics.BodyPair{A: floor, B: body}) {
				continue
			}
			at := new(big.Rat).SetFrac64(int64(k), 256)
			return at.Add(at, new(big.Rat).SetFloat64(event.Time.Base()))
		}
	}
	require.Fail(t, "no floor impact")
	return nil
}

// discreteFreeFall is the exact time a body released at rest falls drop mm
// under the step law: after k steps of 1/256 s its speed is k·9810/256 mm/s
// and it has fallen 9810/65536·k(k+1)/2 mm, and in step k+1 it drifts the
// rest of the way at (k+1)·9810/256 mm/s.
func discreteFreeFall(drop float64) *big.Rat {
	kick := big.NewRat(9810, 256)
	dt := big.NewRat(1, 256)
	fallen := new(big.Rat)
	for k := int64(0); ; k++ {
		speed := new(big.Rat).Mul(kick, big.NewRat(k+1, 1))
		next := new(big.Rat).Add(fallen, new(big.Rat).Mul(speed, dt))
		remaining := new(big.Rat).Sub(new(big.Rat).SetFloat64(drop), fallen)
		if next.Cmp(new(big.Rat).SetFloat64(drop)) >= 0 {
			at := new(big.Rat).Mul(big.NewRat(k, 1), dt)
			return at.Add(at, remaining.Quo(remaining, speed))
		}
		fallen = next
	}
}
