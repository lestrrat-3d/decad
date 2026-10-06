package dynamics_test

import (
	"math"
	"math/big"
	"os"
	"slices"
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
	t.Parallel()
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

// The Phase 2 exit scene of docs/multibody-dynamics-design.md §2, tumble,
// end to end through the same producers: a fixed zero-bound Cut tray, four
// spinning boxes turned about (1, 1, 0), a hexagonal prism, a wedge and a
// stitched tetrahedron, each released on a corner or a vertex, all landing
// on the tray's floor and coming to rest face down inside the band (§10.8).
// The _gallery module renders the same scene.
//
// The whole scene's 768 steps spend nearly all of their time in the first
// 80 steps, where the rotating sweeps dominate, and take longer on the CI
// runners than the dynamics package's ten-minute test budget allows. CI
// therefore runs it in the _gallery module (TestTumbleTimeline there, and
// TestTumbleClip, which also renders its first frame), and TestTumbleScene
// here runs it only when
// DECAD_TUMBLE_FULL is set. TestTumbleSceneSubset runs four of its bodies,
// one of each shape family, through the same assertions in 128 steps, about
// fifteen seconds on an amd64 workstation, on the legs without the race
// detector, which would take it several times past the package budget the
// rest of the package already fills.

// tumbleScene is a tumble world: the tray as its one fixed body, so no
// other fixed body touches it, and the dynamic bodies in release order.
type tumbleScene struct {
	doc     *decad.Document
	tray    *decad.Body
	names   []string
	bodies  []*decad.Body
	boxes   []*decad.Body
	masses  map[*decad.Body]float64
	density units.Value
	config  dynamics.StepConfig
	world   *dynamics.World
	state   dynamics.State
}

// tumbleBodyRelease is one dynamic body of the scene at its release: its
// builder, its pose, its spin and whether it is one of the four boxes.
type tumbleBodyRelease struct {
	name  string
	build func(t *testing.T, doc *decad.Document) *decad.Body
	pose  func(t *testing.T) r3.Transform
	box   bool
}

// tumbleReleases are §2's seven bodies. The boxes are centered on their own
// origin, turned about (1, 1, 0) by 30°, 45°, 60° and 75° and released with
// their centers 40 mm above the floor, 45 mm from the tray's center along
// each diagonal, so each starts at least 20 mm inside the walls (§10.6). The
// prism turns 37° about −Y about its corner at the origin, which every other
// corner then stands above; the wedge and the tetrahedron turn about generic
// axes; each lands on one vertex.
var tumbleReleases = []tumbleBodyRelease{
	tumbleBoxRelease("box0", 30, r3.Vec{X: -45, Y: -45, Z: 40}),
	tumbleBoxRelease("box1", 45, r3.Vec{X: 45, Y: -45, Z: 40}),
	tumbleBoxRelease("box2", 60, r3.Vec{X: -45, Y: 45, Z: 40}),
	tumbleBoxRelease("box3", 75, r3.Vec{X: 45, Y: 45, Z: 40}),
	{name: "hexagon",
		build: func(t *testing.T, doc *decad.Document) *decad.Body { return tumbleHexBody(t)(doc) },
		pose:  func(t *testing.T) r3.Transform { return tumbleRelease(t, r3.Vec{Y: -1}, 37, r3.Vec{X: -55, Z: 20}) }},
	{name: "wedge",
		build: func(t *testing.T, doc *decad.Document) *decad.Body { return tumbleWedgeBody(t)(doc) },
		pose: func(t *testing.T) r3.Transform {
			return tumbleRelease(t, r3.Vec{X: 1, Y: 2, Z: 3}, 50, r3.Vec{X: 40, Z: 25})
		}},
	{name: "tetrahedron", build: tumbleTetrahedron,
		pose: func(t *testing.T) r3.Transform { return tumbleRelease(t, r3.Vec{X: 3, Y: -1, Z: 2}, 40, r3.Vec{Z: 25}) }},
}

func tumbleBoxRelease(name string, degrees float64, center r3.Vec) tumbleBodyRelease {
	return tumbleBodyRelease{name: name, box: true,
		build: func(t *testing.T, doc *decad.Document) *decad.Body { return tumbleBoxBody(t)(doc) },
		pose:  func(t *testing.T) r3.Transform { return tumbleRelease(t, r3.Vec{X: 1, Y: 1}, degrees, center) }}
}

// tumbleStepConfig is the stack-and-drop step at §2's tumble residuals:
// PenetrationResidual 10 µm and SupportBand 5 µm, so a kick that lands a
// box on all four corners leaves it at rest (§10.8).
func tumbleStepConfig() dynamics.StepConfig {
	config := stackAndDropConfig()
	config.MaxPoseEvaluations = 512
	config.PenetrationResidual = units.Millimeters(.01)
	config.Contact.SupportBand = units.Millimeters(.005)
	return config
}

// tumbleTrayBox is a source box over [x0, x1]×[y0, y1]×[z0, z0+h], extruded
// from the XY plane and translated, which keeps the Cut exact.
func tumbleTrayBox(t *testing.T, doc *decad.Document, x0, y0, x1, y1, z0, h float64) *decad.Body {
	t.Helper()
	body := makeBox(t, doc, x0, y0, x1, y1, 0, h)
	shift, err := r3.Translation(r3.Vec{Z: z0})
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), shift)
	require.NoError(t, err)
	return placed
}

// tumbleTray is §2's tray: [−90, 90]²×[−10, 40] minus [−80, 80]²×[0, 50],
// a 10 mm floor with its top face at z = 0 and four walls around the
// 160×160 mm inside. Every crossing of the operands' facets is dyadic, so
// the Cut is a zero-bound Boolean (§9).
func tumbleTray(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	tray, err := decad.Cut(t.Context(), tumbleTrayBox(t, doc, -90, -90, 90, 90, -10, 50),
		tumbleTrayBox(t, doc, -80, -80, 80, 80, 0, 50))
	require.NoError(t, err)
	return tray
}

// tumbleTriangle patches the triangle with the given plane-local corners.
func tumbleTriangle(t *testing.T, doc *decad.Document, w *sketch.World, plane *sketch.Plane,
	local [3][2]float64) *decad.Body {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	var corners [3]*sketch.Point
	for i, p := range local {
		corners[i] = s.CreatePoint(p[0], p[1])
		s.Fix(corners[i])
	}
	for i := range corners {
		s.CreateLine(corners[i], corners[(i+1)%3])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	patch, err := doc.Patch(t.Context(), s, s.Profiles()[0])
	require.NoError(t, err)
	return patch
}

// tumbleTetrahedron stitches the tetrahedron with corners a·(1, 0, 0), the
// origin, a·(0, 1, 0) and a·(1, 0, 1), a = 16·s with s = 1/√2 as r3 holds
// it: the faces on the planes x + y = a and z = x take frames holding one
// cardinal axis and the diagonal (±s, ±s), and their diagonal corners sit at
// plane-local 16, so every corner lands exactly (apitest/mass_properties_mesh_test.go
// stitches the same solid at half the size).
func tumbleTetrahedron(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	probe, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 0, 1), r3.NewVec(-1, 1, 0))
	require.NoError(t, err)
	a := 16 * probe.V().Y
	slanted, err := r3.NewFrame(r3.NewVec(a, 0, 0), r3.NewVec(0, 0, 1), r3.NewVec(-1, 1, 0))
	require.NoError(t, err)
	diagonal, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(1, 0, 1))
	require.NoError(t, err)
	slantedPlane, err := w.CreatePlaneFromFrame(slanted)
	require.NoError(t, err)
	diagonalPlane, err := w.CreatePlaneFromFrame(diagonal)
	require.NoError(t, err)
	// XY's frame is (+X, +Y) and XZ's (+X, +Z).
	tetrahedron, err := decad.Stitch(t.Context(),
		tumbleTriangle(t, doc, w, w.XY(), [3][2]float64{{a, 0}, {0, 0}, {0, a}}),
		tumbleTriangle(t, doc, w, w.XZ(), [3][2]float64{{a, 0}, {0, 0}, {a, a}}),
		tumbleTriangle(t, doc, w, slantedPlane, [3][2]float64{{0, 0}, {0, 16}, {a, 0}}),
		tumbleTriangle(t, doc, w, diagonalPlane, [3][2]float64{{0, 0}, {a, 0}, {0, 16}}))
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, tetrahedron.Kind())
	want := map[r3.Vec]struct{}{{X: a}: {}, {}: {}, {Y: a}: {}, {X: a, Z: a}: {}}
	for _, v := range tetrahedron.Vertices() {
		require.Contains(t, want, v.Position().Value, "premise: every corner lands exactly")
	}
	return tetrahedron
}

// newTumble builds the tray and the named bodies of tumbleReleases, every
// one released at rest but for the boxes' (2, 1, 0) rad/s spin, with
// restitution 0.3 and friction 0.4 at density 0.001 kg/mm³.
func newTumble(t *testing.T, names ...string) tumbleScene {
	t.Helper()
	scene := tumbleScene{doc: decad.New(), density: units.KilogramsPerCubicMillimeter(0.001),
		config: tumbleStepConfig(), masses: map[*decad.Body]float64{}}
	scene.tray = tumbleTray(t, scene.doc)
	material := dynamics.Material{Restitution: units.Scalar(0.3), Friction: units.Scalar(0.4)}
	bodies := []dynamics.RigidBody{{Body: scene.tray, Role: dynamics.Fixed, Material: material}}
	entries := []dynamics.BodyState{{Body: scene.tray, Pose: r3.Identity(), LinearVelocity: zeroVelocity(),
		AngularVelocity: zeroAngular(t)}}
	for _, name := range names {
		var release tumbleBodyRelease
		for _, candidate := range tumbleReleases {
			if candidate.name == name {
				release = candidate
			}
		}
		require.NotNil(t, release.build, name)
		body := release.build(t, scene.doc)
		properties, err := body.MassProperties(t.Context(), scene.density)
		require.NoError(t, err, name)
		scene.masses[body] = properties.Mass.Value.Base()
		scene.names = append(scene.names, name)
		scene.bodies = append(scene.bodies, body)
		spin := zeroAngular(t)
		if release.box {
			scene.boxes = append(scene.boxes, body)
			spin = tumbleBoxSpin()
		}
		bodies = append(bodies, dynamics.RigidBody{Body: body, Role: dynamics.Dynamic, Density: &scene.density,
			Material: material})
		entries = append(entries, dynamics.BodyState{Body: body, Pose: release.pose(t), LinearVelocity: zeroVelocity(),
			AngularVelocity: spin})
	}
	var err error
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: bodies, Step: scene.config})
	require.NoError(t, err)
	scene.state, err = scene.world.NewState(entries)
	require.NoError(t, err)
	return scene
}

// tumbleTimeline advances the scene's timeline through the given number of
// steps of 1/256 s, failing at the first Undecided step with its
// diagnostics.
func tumbleTimeline(t *testing.T, scene tumbleScene, steps int) *dynamics.Timeline {
	t.Helper()
	timeline, err := dynamics.NewTimeline(scene.world, scene.state)
	require.NoError(t, err)
	for k := range steps {
		report, err := timeline.Advance(t.Context(), dynamics.StepInput{Gravity: gravityZ(-9810)}, pyramidDt())
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", k, report.Diagnostics)
	}
	return timeline
}

// tumbleBandEntry is the time into a step of 1/256 s at which the lowest
// vertex of body, starting the step at entry, first stands band above the
// tray floor z = 0 on the step's drift: one exact gravity kick of
// −9810/256 mm/s, then a constant angular velocity about the mass center,
// given in body coordinates, and a constant translation, the path dynamics'
// drift follows. It brackets the first sign change of the lowest
// vertex's height less band on a 4096-sample grid, then bisects it to a
// float. ok is false when the body does not enter the band in the step.
func tumbleBandEntry(t *testing.T, body *decad.Body, center r3.Vec, entry dynamics.BodyState,
	band float64) (float64, bool) {
	t.Helper()
	dt := 1.0 / 256
	fall := entry.LinearVelocity.Z.Base() - 9810.0/256
	omega := r3.Vec{X: entry.AngularVelocity.X.Base(), Y: entry.AngularVelocity.Y.Base(),
		Z: entry.AngularVelocity.Z.Base()}
	rate := math.Hypot(omega.X, math.Hypot(omega.Y, omega.Z))
	pivot := entry.Pose.Apply(center)
	above := func(u float64) float64 {
		pose := entry.Pose
		if rate != 0 {
			turn, err := r3.RotationAround(pivot, omega, units.Radians(rate*u))
			require.NoError(t, err)
			pose, err = pose.Then(turn)
			require.NoError(t, err)
		}
		lowest := math.Inf(1)
		for _, v := range body.Vertices() {
			lowest = math.Min(lowest, pose.Apply(v.Position().Value).Z+fall*u)
		}
		return lowest - band
	}
	const samples = 4096
	if above(0) <= 0 {
		return 0, false
	}
	for i := 1; i <= samples; i++ {
		right := dt * float64(i) / samples
		if above(right) > 0 {
			continue
		}
		left := dt * float64(i-1) / samples
		for {
			mid := (left + right) / 2
			if mid <= left || mid >= right {
				return right, true
			}
			if above(mid) > 0 {
				left = mid
			} else {
				right = mid
			}
		}
	}
	return 0, false
}

// tumbleFirstImpact is the step index and event of the first ContactImpact
// between tray and body.
func tumbleFirstImpact(t *testing.T, steps []*dynamics.StepReport, tray, body *decad.Body) (int, dynamics.ContactEvent) {
	t.Helper()
	for k, report := range steps {
		for _, event := range report.Events {
			if event.Kind == dynamics.ContactImpact && event.Pair == (dynamics.BodyPair{A: tray, B: body}) {
				return k, event
			}
		}
	}
	require.Fail(t, "no tray impact")
	return 0, dynamics.ContactEvent{}
}

// requireTumbleExit asserts §2's Phase 2 exit criteria on a tumble timeline
// advanced steps steps.
func requireTumbleExit(t *testing.T, scene tumbleScene, timeline *dynamics.Timeline, steps int) {
	t.Helper()
	require.Nil(t, timeline.Stopped())
	require.Equal(t, units.Seconds(float64(steps)/256), timeline.End())
	reports := timeline.Steps()
	require.Len(t, reports, steps)
	config := scene.config
	residual := config.PenetrationResidual.Base()

	impacts := map[int]int{}
	for k, report := range reports {
		// Every step's linear momentum balances: the dynamic bodies' change
		// equals the gravity and contact impulses, within the readings' own
		// bounds plus, for every island, each dynamic body's certified
		// linear-law limit ImpulseResidual + m·VelocityResidual.
		c := report.Conservation
		require.NotNil(t, c, "step %d", k)
		limit := 0.0
		for _, island := range report.Islands {
			for _, body := range island.Bodies {
				if mass, ok := scene.masses[body]; ok {
					limit += config.ImpulseResidual.Base() + mass*config.VelocityResidual.Base()
				}
			}
		}
		for axis, get := range []func(dynamics.QuantityVec) units.Value{
			func(v dynamics.QuantityVec) units.Value { return v.X },
			func(v dynamics.QuantityVec) units.Value { return v.Y },
			func(v dynamics.QuantityVec) units.Value { return v.Z },
		} {
			change := get(c.Completion.LinearMomentum.Value).Base() - get(c.Input.LinearMomentum.Value).Base()
			applied := get(c.GravityImpulse.Value).Base() + get(c.ContactImpulse.Value).Base()
			slack := get(c.Completion.LinearMomentum.Bound).Base() + get(c.Input.LinearMomentum.Bound).Base() +
				get(c.GravityImpulse.Bound).Base() + get(c.ContactImpulse.Bound).Base() + limit
			require.InDelta(t, applied, change, slack, "step %d axis %d", k, axis)
		}
		// Every event joins a body and the tray's floor, whose normal is +Z:
		// no body reaches another one or a wall.
		for _, event := range report.Events {
			require.Equal(t, scene.tray, event.Pair.A, "step %d", k)
			for _, point := range event.Manifold.Points {
				require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value, "step %d", k)
			}
			if event.Kind == dynamics.ContactImpact {
				impacts[len(event.Manifold.Points)]++
			}
		}
	}

	// The trace carries a vertex impact, an edge impact and a face impact.
	require.Positive(t, impacts[1], "a one-point (vertex) impact")
	require.Positive(t, impacts[2], "a two-point (edge) impact")
	faces := 0
	for points, count := range impacts {
		if points >= 4 {
			faces += count
		}
	}
	require.Positive(t, faces, "a face impact of four or more points")

	// Each box's first impact lies where the drift of its lowest corner
	// first enters the SupportBand, which is where the falling pair's first
	// ContactBand sample lies (§2, §10.5): the impact is its bracket's right
	// end, at most TimeResolution after that entry. The entry is bisected to
	// a float from float drift poses, whose height errors are ulps of the
	// box's 40 mm coordinates, under 1e-13 mm, and the corner falls at more
	// than 100 mm/s, so 1e-12 s covers the entry's own error.
	band := config.Contact.SupportBand.Base()
	for i, box := range scene.boxes {
		k, event := tumbleFirstImpact(t, reports, scene.tray, box)
		start := scene.state
		if k > 0 {
			start = *reports[k-1].Next
		}
		entry, ok := start.Body(box)
		require.True(t, ok)
		at, ok := tumbleBandEntry(t, box, r3.Vec{}, entry, band)
		require.True(t, ok, "box %d enters the band in step %d", i, k)
		got := event.Time.Base()
		require.GreaterOrEqual(t, got, at-1e-12, "box %d", i)
		require.LessOrEqual(t, got-at, config.TimeResolution.Base()+1e-12, "box %d", i)
	}

	// Every body ends at rest face down: both velocities within
	// VelocityResidual of zero, every vertex's exact height, staged through
	// the published pose, at least −PenetrationResidual, three or more (a
	// face) within PenetrationResidual of the floor, and every vertex inside
	// the walls. The last step's last slice certifies each floor pair by
	// swept boxes strictly apart, the body hovering inside the band, or by a
	// band or persistent track through the step's end.
	last := reports[len(reports)-1]
	floor := rat(-residual)
	for i, body := range scene.bodies {
		name := scene.names[i]
		entry, ok := last.Next.Body(body)
		require.True(t, ok)
		for _, v := range []dynamics.QuantityVec{entry.LinearVelocity, entry.AngularVelocity} {
			require.InDelta(t, 0, v.X.Base(), config.VelocityResidual.Base(), name)
			require.InDelta(t, 0, v.Y.Base(), config.VelocityResidual.Base(), name)
			require.InDelta(t, 0, v.Z.Base(), config.VelocityResidual.Base(), name)
		}
		face := 0
		for _, height := range vertexHeights(body, entry.Pose) {
			require.GreaterOrEqual(t, height.Cmp(floor), 0, name)
			if height.Cmp(rat(residual)) <= 0 {
				face++
			}
		}
		require.GreaterOrEqual(t, face, 3, "%s rests on a face", name)
		for _, v := range body.Vertices() {
			p := entry.Pose.Apply(v.Position().Value)
			require.Less(t, math.Max(math.Abs(p.X), math.Abs(p.Y)), 80.0, name)
		}
	}
	slices := dynamics.TraceSliceProofs(last.Trace)
	require.NotEmpty(t, slices)
	for _, proof := range slices[len(slices)-1] {
		if proof.BoxClear || proof.Pair.A != scene.tray {
			continue
		}
		require.Contains(t, []decad.SweepOutcome{decad.SweepPersistentBand, decad.SweepPersistentTouch},
			proof.Sweep.Outcome)
		require.Equal(t, 1.0, proof.Sweep.ContactTrack.End().Fraction.Base())
		require.LessOrEqual(t, proof.Sweep.ContactTrack.Band().Value.Base(), residual)
	}

	// Every frame time of a 60 fps clip replays a certified state.
	for frame := range steps * 60 / 256 {
		_, err := timeline.Sample(units.Seconds(float64(frame) / 60))
		require.NoError(t, err, "frame %d", frame)
	}
}

// tumbleSteps is the scene's 3 s in steps of 1/256 s.
const tumbleSteps = 768

// TestTumbleScene runs the whole scene for its 3 s and asserts §2's Phase 2
// exit criteria. It runs when DECAD_TUMBLE_FULL is set; see the comment at
// the top of this section.
func TestTumbleScene(t *testing.T) {
	t.Parallel()
	if os.Getenv("DECAD_TUMBLE_FULL") == "" {
		t.Skip("set DECAD_TUMBLE_FULL to run the whole tumble scene")
	}
	names := make([]string, len(tumbleReleases))
	for i, release := range tumbleReleases {
		names[i] = release.name
	}
	scene := newTumble(t, names...)
	requireTumbleExit(t, scene, tumbleTimeline(t, scene, tumbleSteps), tumbleSteps)
}

// TestTumbleSceneSubset runs the 30° box, the prism, the wedge and the
// tetrahedron at their §2 releases in the §2 tray for 0.5 s, by which each
// rests (the run records the box at rest from its step 45 and the others
// from steps 29 to 32), and asserts the same criteria.
func TestTumbleSceneSubset(t *testing.T) {
	t.Parallel()
	if raceDetector {
		t.Skip("the race detector takes this run past the package's test budget")
	}
	scene := newTumble(t, "box0", "hexagon", "wedge", "tetrahedron")
	requireTumbleExit(t, scene, tumbleTimeline(t, scene, 128), 128)
}

// The Phase 3 exit scene of docs/multibody-dynamics-design.md §2, parts-bin,
// end to end through the same producers: §2's tray with a shelled cup, a
// block whose top loop is chamfered 2.3 mm, a loft, a straight hexagon sweep
// and a revolved bottle dropped 8 mm onto its floor, and a source cylinder
// rolling on its side. The cup, loft and sweep are exact; the block's held
// mesh carries its contour's rounding as δ and the bottle's its chord
// sagitta at HeldChord, so both land and rest through §10.4's lifted band.
// The _gallery module renders the same scene.
//
// The whole scene's 1024 steps take about four minutes on an amd64
// workstation without the race detector, nearly all of it in the bottle: its
// three impact steps take about five seconds each, and every resting step
// about 0.2 s for its 49 lifted base vertices. TestPartsBinScene therefore
// runs it only when DECAD_PARTSBIN_FULL is set, and CI runs it in the
// _gallery module (TestPartsBinTimeline there, and TestPartsBinClip, which
// also renders its first frame), as tumble's whole scene runs. TestPartsBinSceneSubset runs every body but the
// bottle through the same assertions for 0.125 s, by which each dropped body
// rests: about three seconds, and ten under the race detector. The bottle's
// drop onto the same tray at the same residuals is
// TestDisplacedBottleRestsOnTray's fixture.

// partsBinScene is a parts-bin world: the tray as its one fixed body and the
// dynamic bodies in release order, with each body's held-mesh δ at the
// scene's HeldChord (zero for an exact body).
type partsBinScene struct {
	doc      *decad.Document
	tray     *decad.Body
	names    []string
	bodies   []*decad.Body
	byName   map[string]*decad.Body
	masses   map[*decad.Body]float64
	deltas   map[*decad.Body]float64
	density  units.Value
	config   dynamics.StepConfig
	world    *dynamics.World
	state    dynamics.State
	cylinder partsBinRoll
}

// partsBinRoll is the cylinder's release: the axis at partsBinCylinderStart,
// rolling along +y without slip, v = −ω × (−r·ẑ) = (0, ω·r, 0) for
// ω = (−partsBinOmega, 0, 0).
type partsBinRoll struct {
	start r3.Vec
	omega float64
}

// partsBinOmega is the cylinder's spin about −x, in rad/s, and
// partsBinCylinderStart its axis's start. The Ø20 cylinder rolls 15 mm/s,
// 60 mm in 4 s, its axis ending at y = 20; its 30 mm length spans x = 30…60,
// so it keeps 20 mm inside the walls and clear of every other body.
const partsBinOmega = 1.5

var partsBinCylinderStart = r3.Vec{X: 45, Y: -40, Z: 10}

// partsBinRelease is one dropped body: its builder and where its modeled
// base, on its own z = 0, is released.
type partsBinRelease struct {
	name  string
	build func(t *testing.T, doc *decad.Document) *decad.Body
	at    r3.Vec
}

// partsBinReleases are §2's dropped bodies, each released at rest 8 mm above
// the floor at a translation pose, at least 20 mm inside the walls.
var partsBinReleases = []partsBinRelease{
	{name: "cup", build: partsBinCup, at: r3.Vec{X: -45, Y: -45, Z: 8}},
	{name: "block", build: partsBinBlock, at: r3.Vec{Y: -45, Z: 8}},
	{name: "loft", build: partsBinLoft, at: r3.Vec{X: -45, Z: 8}},
	{name: "sweep", build: partsBinSweep, at: r3.Vec{X: -45, Y: 45, Z: 8}},
	{name: "bottle", build: bottleBody, at: r3.Vec{Y: 45, Z: 8}},
}

// partsBinSteps is the scene's 4 s in steps of 1/256 s.
const partsBinSteps = 1024

// partsBinCup is §2's cup: a 24×16×16 mm box shelled 2 mm through its top,
// a non-convex exact cupPayload.
func partsBinCup(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	box := makeBox(t, doc, -12, -8, 12, 8, 0, 16)
	cup, err := box.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapEnd(box))), units.Millimeters(2))
	require.NoError(t, err)
	return cup
}

// partsBinBlock is §2's 12 mm cube with its top loop chamfered 2.3 mm: its
// feet are not dyadic, so its held mesh carries its contour's rounding as δ,
// and that mesh carries §9.2's convexity certificate.
func partsBinBlock(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	box := makeBox(t, doc, -6, -6, 6, 6, 0, 12)
	block, err := box.Chamfer(t.Context(), decad.Edges(decad.CreatedBy(decad.CapEnd(box))), units.Millimeters(2.3))
	require.NoError(t, err)
	return block
}

// partsBinPolygon sketches the closed polygon corners on plane, every corner
// fixed.
func partsBinPolygon(t *testing.T, w *sketch.World, plane *sketch.Plane, corners [][2]float64) (*sketch.Sketch,
	*sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

// partsBinLoft is §2's loft, 16 mm tall, from a square with each side pushed
// out to a point 8.5 mm from the axis up to a regular-sided octagon, every
// corner dyadic, so the loft is exact.
func partsBinLoft(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	top, err := w.CreateOffsetPlane(w.XY(), 16)
	require.NoError(t, err)
	s0, p0 := partsBinPolygon(t, w, w.XY(), [][2]float64{
		{8.5, 0}, {8, 8}, {0, 8.5}, {-8, 8}, {-8.5, 0}, {-8, -8}, {0, -8.5}, {8, -8},
	})
	s1, p1 := partsBinPolygon(t, w, top, [][2]float64{
		{6, -2.5}, {6, 2.5}, {2.5, 6}, {-2.5, 6}, {-6, 2.5}, {-6, -2.5}, {-2.5, -6}, {2.5, -6},
	})
	loft, err := doc.Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	return loft
}

// partsBinSweep is §2's straight 14 mm sweep of a hexagon 20 mm across its
// flats, which reads as its prism.
func partsBinSweep(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, p := partsBinPolygon(t, w, w.XY(), [][2]float64{
		{-11.5, 0}, {-5.75, -10}, {5.75, -10}, {11.5, 0}, {5.75, 10}, {-5.75, 10},
	})
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 14)})
	require.NoError(t, err)
	sweep, err := doc.Sweep(t.Context(), s, p, path)
	require.NoError(t, err)
	return sweep
}

// newPartsBin builds the tray, the named bodies of partsBinReleases and, when
// cylinder is set, the rolling cylinder, at restitution 0.3 and friction 0.4
// with density 0.001 kg/mm³ under §2's Phase 3 step (partsBinConfig). Every
// body's mass properties publish here.
func newPartsBin(t *testing.T, cylinder bool, names ...string) partsBinScene {
	t.Helper()
	scene := partsBinScene{doc: decad.New(), density: units.KilogramsPerCubicMillimeter(0.001),
		config: partsBinConfig(), byName: map[string]*decad.Body{}, masses: map[*decad.Body]float64{},
		deltas: map[*decad.Body]float64{}, cylinder: partsBinRoll{start: partsBinCylinderStart, omega: partsBinOmega}}
	scene.tray = tumbleTray(t, scene.doc)
	material := dynamics.Material{Restitution: units.Scalar(0.3), Friction: units.Scalar(0.4)}
	bodies := []dynamics.RigidBody{{Body: scene.tray, Role: dynamics.Fixed, Material: material}}
	entries := []dynamics.BodyState{{Body: scene.tray, Pose: r3.Identity(), LinearVelocity: zeroVelocity(),
		AngularVelocity: zeroAngular(t)}}
	add := func(name string, body *decad.Body, pose r3.Transform, linear, spin dynamics.QuantityVec) {
		properties, err := body.MassProperties(t.Context(), scene.density)
		require.NoError(t, err, name)
		require.Positive(t, properties.Mass.Value.Base(), name)
		scene.masses[body] = properties.Mass.Value.Base()
		mesh, err := body.Tessellate(t.Context(), scene.config.Contact.HeldChord)
		require.NoError(t, err, name)
		scene.deltas[body] = mesh.Bound().Base()
		scene.names = append(scene.names, name)
		scene.bodies = append(scene.bodies, body)
		scene.byName[name] = body
		bodies = append(bodies, dynamics.RigidBody{Body: body, Role: dynamics.Dynamic, Density: &scene.density,
			Material: material})
		entries = append(entries, dynamics.BodyState{Body: body, Pose: pose, LinearVelocity: linear,
			AngularVelocity: spin})
	}
	for _, name := range names {
		var release partsBinRelease
		for _, candidate := range partsBinReleases {
			if candidate.name == name {
				release = candidate
			}
		}
		require.NotNil(t, release.build, name)
		add(name, release.build(t, scene.doc), translation(t, release.at), zeroVelocity(), zeroAngular(t))
	}
	if cylinder {
		linear := zeroVelocity()
		linear.Y = units.MillimetersPerSecond(scene.cylinder.omega * 10)
		spin := zeroAngular(t)
		spin.X = units.RadiansPerSecond(-scene.cylinder.omega)
		add("cylinder", sideCylinder(t, scene.doc), translation(t, scene.cylinder.start), linear, spin)
	}
	var err error
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: bodies, Step: scene.config})
	require.NoError(t, err)
	scene.state, err = scene.world.NewState(entries)
	require.NoError(t, err)
	return scene
}

// partsBinTimeline advances the scene's timeline through the given number of
// steps of 1/256 s, failing at the first Undecided step with its
// diagnostics.
func partsBinTimeline(t *testing.T, scene partsBinScene, steps int) *dynamics.Timeline {
	t.Helper()
	timeline, err := dynamics.NewTimeline(scene.world, scene.state)
	require.NoError(t, err)
	for k := range steps {
		report, err := timeline.Advance(t.Context(), dynamics.StepInput{Gravity: gravityZ(-9810)}, pyramidDt())
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", k, report.Diagnostics)
	}
	return timeline
}

// requirePartsBinExit asserts §2's Phase 3 exit criteria on a parts-bin
// timeline advanced steps steps, for the bodies the scene holds.
func requirePartsBinExit(t *testing.T, scene partsBinScene, timeline *dynamics.Timeline, steps int) {
	t.Helper()
	require.Nil(t, timeline.Stopped())
	require.Equal(t, units.Seconds(float64(steps)/256), timeline.End())
	reports := timeline.Steps()
	require.Len(t, reports, steps)
	config := scene.config
	band := config.Contact.SupportBand.Base()
	velocityResidual := config.VelocityResidual.Base()
	cylinder := scene.byName["cylinder"]
	bottle := scene.byName["bottle"]

	landed := map[*decad.Body]bool{}
	for k, report := range reports {
		// Every step's linear momentum balances: the dynamic bodies' change
		// equals the gravity and contact impulses, within the readings' own
		// bounds plus, for every island, each dynamic body's certified
		// linear-law limit ImpulseResidual + m·VelocityResidual.
		c := report.Conservation
		require.NotNil(t, c, "step %d", k)
		limit := 0.0
		for _, island := range report.Islands {
			for _, body := range island.Bodies {
				if mass, ok := scene.masses[body]; ok {
					limit += config.ImpulseResidual.Base() + mass*velocityResidual
				}
			}
		}
		for axis, get := range []func(dynamics.QuantityVec) units.Value{
			func(v dynamics.QuantityVec) units.Value { return v.X },
			func(v dynamics.QuantityVec) units.Value { return v.Y },
			func(v dynamics.QuantityVec) units.Value { return v.Z },
		} {
			change := get(c.Completion.LinearMomentum.Value).Base() - get(c.Input.LinearMomentum.Value).Base()
			applied := get(c.GravityImpulse.Value).Base() + get(c.ContactImpulse.Value).Base()
			slack := get(c.Completion.LinearMomentum.Bound).Base() + get(c.Input.LinearMomentum.Bound).Base() +
				get(c.GravityImpulse.Bound).Base() + get(c.ContactImpulse.Bound).Base() + limit
			require.InDelta(t, applied, change, slack, "step %d axis %d", k, axis)
		}

		// Every event joins a body and the tray's floor, whose normal is +Z:
		// no body reaches another one or a wall.
		for _, event := range report.Events {
			require.Equal(t, scene.tray, event.Pair.A, "step %d", k)
			for _, point := range event.Manifold.Points {
				require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value, "step %d", k)
			}
		}

		// The bottle's landing island publishes a WitnessSpin below 1 rad/s,
		// and every island that leaves it at rest one below 0.05 rad/s: the
		// spin its δ leaves uncertain in its published zero (§6.3).
		if bottle != nil {
			next, ok := report.Next.Body(bottle)
			require.True(t, ok)
			atRest := next.LinearVelocity == zeroVelocity() && next.AngularVelocity == zeroAngular(t)
			for _, island := range report.Islands {
				if !slices.Contains(island.Bodies, bottle) {
					continue
				}
				spin := island.Solver.WitnessSpin.Base()
				require.Positive(t, spin, "step %d: the bottle's δ leaves a witness spin", k)
				if !landed[bottle] {
					require.Less(t, spin, 1.0, "step %d: the landing's witness spin", k)
				}
				if atRest {
					require.Less(t, spin, .05, "step %d: a resting island's witness spin", k)
				}
				landed[bottle] = true
			}
		}

		// The rolling cylinder rides a rotating band track on the tray's
		// floor through every step's last slice: an exact touch from its
		// signed-axis release, a band of the turned pose's rounding after it.
		// Halfway along the track its two ruling ends are at rest on the
		// cylinder within VelocityResidual, beyond |ω| times each point's
		// ball.
		if cylinder != nil {
			requirePartsBinRolls(t, scene, report, k)
		}
	}

	// Every dropped body ends at rest, both velocities within
	// VelocityResidual of zero, inside the walls, every vertex staged through
	// the published pose at least −PenetrationResidual above the floor.
	last := reports[len(reports)-1]
	residual := config.PenetrationResidual.Base()
	for i, body := range scene.bodies {
		name := scene.names[i]
		entry, ok := last.Next.Body(body)
		require.True(t, ok)
		for _, v := range body.Vertices() {
			p := entry.Pose.Apply(v.Position().Value)
			require.Less(t, math.Max(math.Abs(p.X), math.Abs(p.Y)), 80.0, name)
		}
		for _, height := range vertexHeights(body, entry.Pose) {
			require.GreaterOrEqual(t, height.Cmp(rat(-residual)), 0, name)
		}
		if body == cylinder {
			continue
		}
		for _, v := range []dynamics.QuantityVec{entry.LinearVelocity, entry.AngularVelocity} {
			require.InDelta(t, 0, v.X.Base(), velocityResidual, name)
			require.InDelta(t, 0, v.Y.Base(), velocityResidual, name)
			require.InDelta(t, 0, v.Z.Base(), velocityResidual, name)
		}

		// At rest each displaced body hovers on a ContactBand whose
		// published gap bound is at most its δ plus SupportBand, every
		// manifold point a lifted vertex inside the band; the cup rests on
		// its four lifted bottom corners.
		contact, err := scene.doc.ContactPair(t.Context(), scene.tray, body, r3.Identity(), entry.Pose, config.Contact)
		require.NoError(t, err, name)
		require.NotNil(t, contact.Manifold, "%s reason=%v", name, contact.Reason)
		for _, point := range contact.Manifold.Points {
			require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value, name)
			require.GreaterOrEqual(t, point.Separation.Value.Base(), 0.0, name)
			require.LessOrEqual(t, point.Separation.Value.Base(), band, name)
		}
		switch name {
		case "block", "bottle":
			require.Positive(t, scene.deltas[body], name)
			require.Equal(t, decad.ContactBand, contact.Relation, "%s reason=%v", name, contact.Reason)
			require.LessOrEqual(t, contact.Gap.Bound.Base(), scene.deltas[body]+band, name)
		case "cup":
			require.Equal(t, decad.ContactBand, contact.Relation, "%s reason=%v", name, contact.Reason)
			require.Len(t, contact.Manifold.Points, 4, name)
		}
	}

	// Every frame time of a 60 fps clip replays a certified state.
	for frame := range steps * 60 / 256 {
		_, err := timeline.Sample(units.Seconds(float64(frame) / 60))
		require.NoError(t, err, "frame %d", frame)
	}
}

// requirePartsBinRolls asserts that step k rolls the cylinder without slip on
// the tray's floor: its axis stays 10 mm up and travels ω·r per second along
// +y, the step's last slice continues the pair on a rolling track to the
// step's end, and halfway along it the track's ruling ends are at rest on
// the cylinder.
func requirePartsBinRolls(t *testing.T, scene partsBinScene, report *dynamics.StepReport, k int) {
	t.Helper()
	config := scene.config
	cylinder := scene.byName["cylinder"]
	pair := dynamics.BodyPair{A: scene.tray, B: cylinder}
	omega, start := scene.cylinder.omega, scene.cylinder.start
	// The closed forms run in float64 over coordinates below 100 mm, every
	// pose a float composition of up to 1024 turns.
	const slack = 1e-8
	entry, ok := report.Next.Body(cylinder)
	require.True(t, ok)
	at := entry.Pose.Translation()
	require.InDelta(t, start.X, at.X, slack, "step %d", k)
	require.InDelta(t, start.Z, at.Z, slack, "step %d", k)
	require.InDelta(t, start.Y+omega*10*float64(k+1)/256, at.Y, slack, "step %d", k)
	require.InDelta(t, omega*10, entry.LinearVelocity.Y.Base(), config.VelocityResidual.Base(), "step %d", k)
	require.InDelta(t, -omega, entry.AngularVelocity.X.Base(), config.AngularVelocityResidual.Base(), "step %d", k)

	proofs := dynamics.TraceSliceProofs(report.Trace)
	require.NotEmpty(t, proofs, "step %d", k)
	var sweep *decad.SweepReport
	for _, proof := range proofs[len(proofs)-1] {
		if proof.Pair == pair {
			sweep = proof.Sweep
		}
	}
	require.NotNil(t, sweep, "step %d", k)
	track := sweep.ContactTrack
	require.NotNil(t, track, "step %d", k)
	require.Equal(t, 1.0, track.End().Fraction.Base(), "step %d", k)
	if k == 0 {
		require.Equal(t, decad.SweepPersistentTouch, sweep.Outcome)
	} else {
		require.Equal(t, decad.SweepPersistentBand, sweep.Outcome, "step %d", k)
		require.NotNil(t, track.Band(), "step %d", k)
		require.LessOrEqual(t, track.Band().Value.Base()+track.Band().Bound.Base(), config.PenetrationResidual.Base(),
			"step %d", k)
	}

	// The track's fractions run over the slice's drift of the cylinder,
	// which starts at the slice's own start, the step's last event: its
	// center at fraction one half is Center + v·Duration/2, and its velocity
	// is the drift's.
	path, ok := sweep.PathB.(decad.RigidDriftSegment)
	require.True(t, ok, "step %d: %T", k, sweep.PathB)
	v := r3.Vec{X: path.LinearVelocity.X.Base(), Y: path.LinearVelocity.Y.Base(), Z: path.LinearVelocity.Z.Base()}
	w := r3.Vec{X: path.AngularVelocity.X.Base(), Y: path.AngularVelocity.Y.Base(), Z: path.AngularVelocity.Z.Base()}
	center := path.Center.Add(v.Scale(path.Duration.Base() / 2))
	manifold, err := track.ManifoldAt(units.Scalar(.5))
	require.NoError(t, err, "step %d", k)
	require.Len(t, manifold.Points, 2, "step %d", k)
	for i, point := range manifold.Points {
		speed := v.Add(w.Cross(point.OnB.Value.Sub(center))).Len()
		require.LessOrEqual(t, speed, config.VelocityResidual.Base()+omega*point.OnB.Bound.Base(),
			"step %d end %d", k, i)
	}
}

// TestPartsBinScene runs the whole scene for its 4 s and asserts §2's Phase 3
// exit criteria. It runs when DECAD_PARTSBIN_FULL is set; see the comment at
// the top of this section.
func TestPartsBinScene(t *testing.T) {
	t.Parallel()
	if os.Getenv("DECAD_PARTSBIN_FULL") == "" {
		t.Skip("set DECAD_PARTSBIN_FULL to run the whole parts-bin scene")
	}
	names := make([]string, len(partsBinReleases))
	for i, release := range partsBinReleases {
		names[i] = release.name
	}
	scene := newPartsBin(t, true, names...)
	requirePartsBinExit(t, scene, partsBinTimeline(t, scene, partsBinSteps), partsBinSteps)
}

// TestPartsBinSceneSubset runs the cup, the chamfered block, the loft, the
// sweep and the rolling cylinder at their §2 releases in the §2 tray for
// 0.125 s, by which each dropped body rests (the run records all four at
// rest from step 17), and asserts the same criteria.
func TestPartsBinSceneSubset(t *testing.T) {
	t.Parallel()
	scene := newPartsBin(t, true, "cup", "block", "loft", "sweep")
	requirePartsBinExit(t, scene, partsBinTimeline(t, scene, 32), 32)
}
