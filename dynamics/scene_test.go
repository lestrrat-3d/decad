package dynamics_test

import (
	"math"
	"math/big"
	"os"
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

// The Phase 2 exit scene of docs/multibody-dynamics-design.md §2, tumble,
// end to end through the same producers: a fixed zero-bound Cut tray, four
// spinning boxes turned about (1, 1, 0), a hexagonal prism, a wedge and a
// stitched tetrahedron, each released on a corner or a vertex, all landing
// on the tray's floor and coming to rest face down inside the band (§10.8).
// The _gallery module renders the same scene.
//
// The whole scene's 768 steps take about six minutes on an amd64
// workstation, nearly all of it in the first 80 steps, where the rotating
// sweeps' exact interval arithmetic dominates, and longer on the CI runners
// than the dynamics package's ten-minute test budget allows. CI therefore
// runs it in the _gallery module (TestTumbleTimeline there, and the
// smoke render), and TestTumbleScene here runs it only when
// DECAD_TUMBLE_FULL is set. TestTumbleSceneSubset runs four of its bodies,
// one of each shape family, through the same assertions in 128 steps, about
// forty seconds, on the legs without the race detector, which would take it
// several times past the package budget the rest of the package already
// fills.

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
// plane-local 16, so every corner lands exactly (mass_properties_mesh_test.go
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
	if os.Getenv("DECAD_TUMBLE_FULL") == "" {
		t.Skip("set DECAD_TUMBLE_FULL to run the whole tumble scene (about six minutes)")
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
	if raceDetector {
		t.Skip("the race detector takes this run past the package's test budget")
	}
	scene := newTumble(t, "box0", "hexagon", "wedge", "tetrahedron")
	requireTumbleExit(t, scene, tumbleTimeline(t, scene, 128), 128)
}
