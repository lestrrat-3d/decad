package main

import (
	"math"
	"math/big"
	"slices"
	"sync"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/kinetograph"
	"github.com/lestrrat-3d/kinetograph/render"
)

// These tests run the stack-and-drop exit scene of
// docs/multibody-dynamics-design.md §2 through the real producers and the
// real viewer (§11.3): the scene's dynamics.Timeline, the timelineTrack
// bridge, kinetograph's driven nodes and one rendered frame. dynamics'
// scene_test.go asserts the same criteria inside the decad module.

// stackAndDropFPS is the frame rate §2 films the scene at.
const stackAndDropFPS = 60

// stackAndDropRun is the scene and its timeline advanced to the clip length,
// computed once for every test here, since the 512 steps dominate their time.
// The first test to ask computes it under its own context, which outlives
// the computation.
var stackAndDropRun struct {
	once     sync.Once
	scene    *dynamicsScene
	timeline *dynamics.Timeline
	err      error
}

type stackAndDropResult struct {
	scene    *dynamicsScene
	timeline *dynamics.Timeline
}

// part is the scene's body named name.
func (r *stackAndDropResult) part(t *testing.T, name string) *decad.Body {
	t.Helper()
	for _, p := range r.scene.parts {
		if p.name == name {
			return p.body
		}
	}
	require.Fail(t, "no part "+name)
	return nil
}

func stackAndDrop(t *testing.T) *stackAndDropResult {
	t.Helper()
	run := &stackAndDropRun
	run.once.Do(func() {
		run.scene, run.err = stackAndDropScene(t.Context())
		if run.err == nil {
			run.timeline, run.err = run.scene.advance(t.Context())
		}
	})
	require.NoError(t, run.err)
	require.NoError(t, run.scene.stopError(run.timeline))
	return &stackAndDropResult{scene: run.scene, timeline: run.timeline}
}

// stepImpulse sums one step's published normal impulses between a and b.
func stepImpulse(report *dynamics.StepReport, a, b *decad.Body) (float64, bool) {
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

// TestStackAndDropTimeline asserts §2's Phase 1 exit criteria on the trace.
func TestStackAndDropTimeline(t *testing.T) {
	run := stackAndDrop(t)
	timeline, config := run.timeline, run.scene.config.Step
	require.Nil(t, timeline.Stopped())
	require.Equal(t, units.Seconds(2), timeline.End())
	steps := timeline.Steps()
	require.Len(t, steps, 512)

	var boxes [6]*decad.Body
	for i := range boxes {
		boxes[i] = run.part(t, "box"+string(rune('0'+i)))
	}
	mass, err := boxes[0].MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(0.001))
	require.NoError(t, err)
	weight := mass.Mass.Value.Base() * 9810 / 256 // a box's weight impulse per step, exact

	for k, report := range steps {
		// Linear momentum balances every step: the dynamic bodies' change
		// equals the gravity and contact impulses within the readings' bounds
		// plus each island's linear-law limit, under 1e-5 kg·mm/s per body
		// for these masses.
		c := report.Conservation
		require.NotNil(t, c, "step %d", k)
		change := c.Completion.LinearMomentum.Value.Z.Base() - c.Input.LinearMomentum.Value.Z.Base()
		applied := c.GravityImpulse.Value.Z.Base() + c.ContactImpulse.Value.Z.Base()
		slack := c.Completion.LinearMomentum.Bound.Z.Base() + c.Input.LinearMomentum.Bound.Z.Base() +
			c.GravityImpulse.Bound.Z.Base() + c.ContactImpulse.Bound.Z.Base() + 1e-4
		require.InDelta(t, applied, change, slack, "step %d", k)

		// The top box's two half-face patches each carry a share of its
		// weight, and the shares sum to it.
		left, ok := stepImpulse(report, boxes[3], boxes[5])
		require.True(t, ok, "step %d", k)
		right, ok := stepImpulse(report, boxes[4], boxes[5])
		require.True(t, ok, "step %d", k)
		require.Positive(t, left, "step %d", k)
		require.Positive(t, right, "step %d", k)
		require.InDelta(t, weight, left+right, 1e-5, "step %d", k)
	}

	// The pyramid ends within PenetrationResidual of its start poses, at
	// every corner.
	end, err := timeline.Sample(timeline.End())
	require.NoError(t, err)
	residual := config.PenetrationResidual.Base()
	for i, box := range boxes {
		entry, ok := end.Body(box)
		require.True(t, ok)
		corner := stackAndDropBoxes[i]
		for _, d := range []r3.Vec{{}, {X: 20, Y: 20, Z: 20}, {X: 20}, {Y: 20, Z: 20}} {
			p := corner.Add(d)
			q := entry.Pose.Apply(p)
			require.InDelta(t, p.X, q.X, residual, "box %d", i)
			require.InDelta(t, p.Y, q.Y, residual, "box %d", i)
			require.InDelta(t, p.Z, q.Z, residual, "box %d", i)
		}
	}

	// The first sphere's first floor impact lies at the discrete free-fall
	// time, the step law's one exact kick per 1/256 s then drift, within
	// TimeResolution after it.
	floor, sphere := run.part(t, "floor"), run.part(t, "sphere0")
	var first *big.Rat
	for k, report := range steps {
		for _, event := range report.Events {
			if first == nil && event.Pair == (dynamics.BodyPair{A: floor, B: sphere}) {
				first = new(big.Rat).SetFrac64(int64(k), 256)
				first.Add(first, new(big.Rat).SetFloat64(event.Time.Base()))
			}
		}
	}
	require.NotNil(t, first)
	exact := freeFallAfterKicks(stackAndDropSpheres[0].Z - 8)
	require.GreaterOrEqual(t, first.Cmp(exact), 0)
	late := new(big.Rat).Sub(first, exact)
	require.LessOrEqual(t, late.Cmp(new(big.Rat).SetFloat64(config.TimeResolution.Base())), 0,
		"impact %s s after free fall", late.FloatString(15))
}

// freeFallAfterKicks is the exact time a body released at rest falls drop mm
// under one kick of −9810/256 mm/s per 1/256 s step followed by drift.
func freeFallAfterKicks(drop float64) *big.Rat {
	dt := big.NewRat(1, 256)
	target := new(big.Rat).SetFloat64(drop)
	fallen := new(big.Rat)
	for k := int64(0); ; k++ {
		speed := new(big.Rat).Mul(big.NewRat(9810, 256), big.NewRat(k+1, 1))
		next := new(big.Rat).Add(fallen, new(big.Rat).Mul(speed, dt))
		if next.Cmp(target) >= 0 {
			remaining := new(big.Rat).Sub(target, fallen)
			at := new(big.Rat).Mul(big.NewRat(k, 1), dt)
			return at.Add(at, remaining.Quo(remaining, speed))
		}
		fallen = next
	}
}

// TestStackAndDropClip films the timeline: the clip holds §2's 120 frames,
// every frame's part transforms are exactly the timeline's certified poses at
// the frame time, concurrent evaluation returns the same frames, and the
// first frame renders.
func TestStackAndDropClip(t *testing.T) {
	run := stackAndDrop(t)
	scene, err := run.scene.clipScene(run.timeline)
	require.NoError(t, err)
	clip, err := kinetograph.NewClip(scene, stackAndDropFPS, run.scene.length)
	require.NoError(t, err)
	require.Equal(t, 120, clip.FrameCount())

	frames := make([]*kinetograph.Frame, clip.FrameCount())
	for i := range frames {
		frames[i], err = clip.Frame(t.Context(), i)
		require.NoError(t, err, "frame %d", i)
		state, err := run.timeline.Sample(units.Seconds(float64(clip.FrameTime(i)) / 1e9))
		require.NoError(t, err, "frame %d", i)
		require.Len(t, frames[i].Poses, len(run.scene.parts))
		for k, part := range run.scene.parts {
			entry, ok := state.Body(part.body)
			require.True(t, ok)
			require.Equal(t, part.name, frames[i].Poses[k].Name)
			require.Equal(t, entry.Pose, frames[i].Poses[k].Transform, "frame %d part %s", i, part.name)
		}
	}
	// The sphere falls in the clip: its frame-0 pose is its release
	// translation, and a later frame shows it lower.
	sphere := frames[0].Poses[7]
	require.Equal(t, "sphere0", sphere.Name)
	require.Equal(t, stackAndDropSpheres[0], sphere.Transform.Translation())
	require.Less(t, frames[6].Poses[7].Transform.Translation().Z, stackAndDropSpheres[0].Z)

	// kinetograph calls At from several goroutines; every call sees the
	// same certified frame.
	var wg sync.WaitGroup
	got := make([][]*kinetograph.Frame, 4)
	errs := make([][]error, len(got))
	for w := range got {
		wg.Go(func() {
			got[w] = make([]*kinetograph.Frame, len(frames))
			errs[w] = make([]error, len(frames))
			for k := range frames {
				// Each worker walks every frame, from its own start and
				// in alternating directions.
				i := k
				if w%2 == 1 {
					i = len(frames) - 1 - k
				}
				i = (i + 37*w) % len(frames)
				got[w][i], errs[w][i] = clip.Frame(t.Context(), i)
			}
		})
	}
	wg.Wait()
	for w := range got {
		for i := range frames {
			require.NoError(t, errs[w][i])
			require.Equal(t, frames[i].Poses, got[w][i].Poses, "worker %d frame %d", w, i)
		}
	}

	// A smoke render of the first frame draws the scene: some pixels differ
	// from the background.
	renderer, err := render.New(t.Context(), clip, run.scene.style(smokeWidth, smokeHeight))
	require.NoError(t, err)
	img, err := renderer.Frame(t.Context(), 0)
	require.NoError(t, err)
	require.Equal(t, smokeWidth, img.Bounds().Dx())
	require.Equal(t, smokeHeight, img.Bounds().Dy())
	background := img.RGBAAt(0, 0)
	drawn := 0
	for y := range smokeHeight {
		for x := range smokeWidth {
			if img.RGBAAt(x, y) != background {
				drawn++
			}
		}
	}
	require.Positive(t, drawn)
}

// TestTimelineTrackStopsAtCertifiedEnd films a timeline advanced only four
// steps, to 1/64 s, in a 2 s clip: frames before the certified end show the
// sampled poses, the first frame past it fails with the timeline's
// ErrUnsupported, and the track never holds the last pose.
func TestTimelineTrackStopsAtCertifiedEnd(t *testing.T) {
	ctx := t.Context()
	scene, err := stackAndDropScene(ctx)
	require.NoError(t, err)
	timeline, err := dynamics.NewTimeline(scene.world, scene.start)
	require.NoError(t, err)
	for range 4 {
		report, err := timeline.Advance(ctx, scene.input, scene.dt)
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	}
	require.Equal(t, units.Seconds(1.0/64), timeline.End())

	sphere := scene.parts[7].body
	track := timelineTrack{timeline: timeline, body: sphere}
	pose, err := track.At(time.Second / 128)
	require.NoError(t, err)
	state, err := timeline.Sample(units.Seconds(1.0 / 128))
	require.NoError(t, err)
	entry, ok := state.Body(sphere)
	require.True(t, ok)
	require.Equal(t, entry.Pose, pose)
	_, err = track.At(time.Second / 32)
	require.ErrorIs(t, err, dynamics.ErrUnsupported)

	clipScene, err := scene.clipScene(timeline)
	require.NoError(t, err)
	clip, err := kinetograph.NewClip(clipScene, stackAndDropFPS, scene.length)
	require.NoError(t, err)
	_, err = clip.Frame(ctx, 0)
	require.NoError(t, err)
	_, err = clip.Frame(ctx, 1) // 1/60 s, past 1/64 s
	require.ErrorIs(t, err, dynamics.ErrUnsupported)

	other := timelineTrack{timeline: timeline, body: decadBodyOutsideWorld(t)}
	_, err = other.At(0)
	require.ErrorIs(t, err, errBodyNotInTimeline)
}

func decadBodyOutsideWorld(t *testing.T) *decad.Body {
	t.Helper()
	body, err := extrudedBox(t.Context(), decad.New(), 0, 0, 1, 1, 0, 1)
	require.NoError(t, err)
	return body
}

// These tests run the tumble exit scene of docs/multibody-dynamics-design.md
// §2 through the real producers and the real viewer (§11.3): the scene's
// dynamics.Timeline, the timelineTrack bridge and kinetograph's driven nodes.
// dynamics' scene_test.go asserts every exit criterion inside the decad
// module, on four of the bodies in CI and on the whole scene when
// DECAD_TUMBLE_FULL is set; CI runs the whole scene here, and these check
// its end, its events and its clip.

// tumbleRun is the scene and its timeline advanced to the clip length,
// computed once for every test here.
var tumbleRun struct {
	once     sync.Once
	scene    *dynamicsScene
	timeline *dynamics.Timeline
	err      error
}

func tumble(t *testing.T) (*dynamicsScene, *dynamics.Timeline) {
	t.Helper()
	run := &tumbleRun
	run.once.Do(func() {
		run.scene, run.err = tumbleScene(t.Context())
		if run.err == nil {
			run.timeline, run.err = run.scene.advance(t.Context())
		}
	})
	require.NoError(t, run.err)
	require.NoError(t, run.scene.stopError(run.timeline))
	return run.scene, run.timeline
}

// TestTumbleTimeline checks the gallery's scene reaches its 3 s with every
// event a body on the tray's floor, a one-, a two- and a four-or-more-point
// impact among them, and every dynamic body at rest face down on the tray:
// both velocities within VelocityResidual of zero and at least three
// vertices within PenetrationResidual of the floor's top face z = 0, none
// below it by more.
func TestTumbleTimeline(t *testing.T) {
	scene, timeline := tumble(t)
	require.Nil(t, timeline.Stopped())
	require.Equal(t, units.Seconds(3), timeline.End())
	require.Len(t, timeline.Steps(), 768)
	tray := scene.parts[0].body
	impacts := map[int]int{}
	for k, report := range timeline.Steps() {
		for _, event := range report.Events {
			require.Equal(t, tray, event.Pair.A, "step %d", k)
			for _, point := range event.Manifold.Points {
				require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value, "step %d", k)
			}
			if event.Kind == dynamics.ContactImpact {
				impacts[min(len(event.Manifold.Points), 4)]++
			}
		}
	}
	for _, points := range []int{1, 2, 4} {
		require.Positive(t, impacts[points], "a %d-point impact", points)
	}
	end, err := timeline.Sample(timeline.End())
	require.NoError(t, err)
	config := scene.config.Step
	residual := new(big.Rat).SetFloat64(config.PenetrationResidual.Base())
	floor := new(big.Rat).Neg(residual)
	for _, part := range scene.parts[1:] {
		entry, ok := end.Body(part.body)
		require.True(t, ok)
		for _, v := range []dynamics.QuantityVec{entry.LinearVelocity, entry.AngularVelocity} {
			for _, c := range []units.Value{v.X, v.Y, v.Z} {
				require.LessOrEqual(t, math.Abs(c.Base()), config.VelocityResidual.Base(), part.name)
			}
		}
		face := 0
		for _, height := range stagedHeights(part.body, entry) {
			require.GreaterOrEqual(t, height.Cmp(floor), 0, part.name)
			if height.Cmp(residual) <= 0 {
				face++
			}
		}
		require.GreaterOrEqual(t, face, 3, "%s rests on a face", part.name)
	}
}

// stagedHeights stages every vertex of body through entry's pose exactly
// and returns its height above z = 0.
func stagedHeights(body *decad.Body, entry dynamics.BodyState) []*big.Rat {
	basis, at := entry.Pose.Basis(), entry.Pose.Translation()
	exact := func(v float64) *big.Rat { return new(big.Rat).SetFloat64(v) }
	out := make([]*big.Rat, 0, len(body.Vertices()))
	for _, vertex := range body.Vertices() {
		p := vertex.Position().Value
		z := exact(at.Z)
		z.Add(z, new(big.Rat).Mul(exact(basis.EX.Z), exact(p.X)))
		z.Add(z, new(big.Rat).Mul(exact(basis.EY.Z), exact(p.Y)))
		z.Add(z, new(big.Rat).Mul(exact(basis.EZ.Z), exact(p.Z)))
		out = append(out, z)
	}
	return out
}

// TestTumbleClip films the timeline: the 60 fps clip holds 180 frames, and
// every frame's part transforms are exactly the timeline's certified poses
// at the frame time.
func TestTumbleClip(t *testing.T) {
	scene, timeline := tumble(t)
	clipScene, err := scene.clipScene(timeline)
	require.NoError(t, err)
	clip, err := kinetograph.NewClip(clipScene, 60, scene.length)
	require.NoError(t, err)
	require.Equal(t, 180, clip.FrameCount())
	for i := range clip.FrameCount() {
		frame, err := clip.Frame(t.Context(), i)
		require.NoError(t, err, "frame %d", i)
		state, err := timeline.Sample(units.Seconds(float64(clip.FrameTime(i)) / 1e9))
		require.NoError(t, err, "frame %d", i)
		require.Len(t, frame.Poses, len(scene.parts))
		for k, part := range scene.parts {
			entry, ok := state.Body(part.body)
			require.True(t, ok)
			require.Equal(t, part.name, frame.Poses[k].Name)
			require.Equal(t, entry.Pose, frame.Poses[k].Transform, "frame %d part %s", i, part.name)
		}
	}
}

// These tests run the parts-bin exit scene of
// docs/multibody-dynamics-design.md §2 through the real producers and the
// real viewer (§11.3), as the tumble tests above run theirs. dynamics'
// scene_test.go asserts every exit criterion inside the decad module, on two
// of the bodies in CI and on the whole scene when DECAD_PARTSBIN_FULL is set;
// CI runs the whole scene here, and these check its end, its events, its
// rests and rolls, and its clip.

// partsBinRun is the scene and its timeline advanced to the clip length,
// computed once for every test here.
var partsBinRun struct {
	once     sync.Once
	scene    *dynamicsScene
	timeline *dynamics.Timeline
	err      error
}

func partsBin(t *testing.T) (*dynamicsScene, *dynamics.Timeline) {
	t.Helper()
	run := &partsBinRun
	run.once.Do(func() {
		run.scene, run.err = partsBinScene(t.Context())
		if run.err == nil {
			run.timeline, run.err = run.scene.advance(t.Context())
		}
	})
	require.NoError(t, run.err)
	require.NoError(t, run.scene.stopError(run.timeline))
	return run.scene, run.timeline
}

// TestPartsBinTimeline checks the gallery's scene reaches its 4 s with every
// body's mass published and every event a body on the tray's floor; the
// bottle's landing island publishes a WitnessSpin below 1 rad/s and every
// island that leaves it at rest one below 0.05 rad/s; every dropped body ends
// at rest, the chamfered block and the bottle each on a ContactBand whose gap
// bound is at most its held mesh's δ plus SupportBand and the cup on four
// lifted points; and the cylinder ends still rolling without slip, its
// contact point's speed within VelocityResidual.
func TestPartsBinTimeline(t *testing.T) {
	scene, timeline := partsBin(t)
	require.Nil(t, timeline.Stopped())
	require.Equal(t, units.Seconds(4), timeline.End())
	require.Len(t, timeline.Steps(), 1024)
	config := scene.config.Step
	density := units.KilogramsPerCubicMillimeter(0.001)
	parts := map[string]*decad.Body{}
	for _, part := range scene.parts {
		parts[part.name] = part.body
		properties, err := part.body.MassProperties(t.Context(), density)
		require.NoError(t, err, part.name)
		require.Positive(t, properties.Mass.Value.Base(), part.name)
	}
	tray, bottle := parts["tray"], parts["bottle"]

	landed := false
	for k, report := range timeline.Steps() {
		for _, event := range report.Events {
			require.Equal(t, tray, event.Pair.A, "step %d", k)
			for _, point := range event.Manifold.Points {
				require.Equal(t, r3.Vec{Z: 1}, point.Normal.Value, "step %d", k)
			}
		}
		next, ok := report.Next.Body(bottle)
		require.True(t, ok)
		atRest := next.LinearVelocity == millimetersPerSecond(0, 0, 0) && next.AngularVelocity == radiansPerSecond(0, 0, 0)
		for _, island := range report.Islands {
			if !slices.Contains(island.Bodies, bottle) {
				continue
			}
			spin := island.Solver.WitnessSpin.Base()
			if !landed {
				require.Less(t, spin, 1.0, "step %d: the landing's witness spin", k)
			}
			if atRest {
				require.Less(t, spin, .05, "step %d: a resting island's witness spin", k)
			}
			landed = true
		}
	}
	require.True(t, landed)

	end, err := timeline.Sample(timeline.End())
	require.NoError(t, err)
	band := config.Contact.SupportBand.Base()
	for _, name := range []string{"cup", "block", "loft", "sweep", "bottle"} {
		entry, ok := end.Body(parts[name])
		require.True(t, ok)
		for _, v := range []dynamics.QuantityVec{entry.LinearVelocity, entry.AngularVelocity} {
			for _, c := range []units.Value{v.X, v.Y, v.Z} {
				require.LessOrEqual(t, math.Abs(c.Base()), config.VelocityResidual.Base(), name)
			}
		}
		contact, err := scene.doc.ContactPair(t.Context(), tray, parts[name], r3.Identity(), entry.Pose, config.Contact)
		require.NoError(t, err, name)
		require.NotNil(t, contact.Manifold, "%s reason=%v", name, contact.Reason)
		switch name {
		case "block", "bottle":
			mesh, err := parts[name].Tessellate(t.Context(), config.Contact.HeldChord)
			require.NoError(t, err, name)
			require.Equal(t, decad.ContactBand, contact.Relation, name)
			require.LessOrEqual(t, contact.Gap.Bound.Base(), mesh.Bound().Base()+band, name)
		case "cup":
			require.Equal(t, decad.ContactBand, contact.Relation, name)
			require.Len(t, contact.Manifold.Points, 4, name)
		}
	}

	// The cylinder rolls on: its axis 10 mm up, 60 mm along +y from its
	// release, and its lowest line at rest, v + ω × (−10·ẑ) within
	// VelocityResidual.
	cylinder, ok := end.Body(parts["cylinder"])
	require.True(t, ok)
	at := cylinder.Pose.Translation()
	require.InDelta(t, partsBinCylinderStart.Y+4*partsBinOmega*10, at.Y, 1e-8)
	require.InDelta(t, 10, at.Z, 1e-8)
	slip := cylinder.LinearVelocity.Y.Base() + cylinder.AngularVelocity.X.Base()*10
	require.LessOrEqual(t, math.Abs(slip), config.VelocityResidual.Base())
}

// TestPartsBinClip films the timeline: the 60 fps clip holds 240 frames, and
// every frame's part transforms are exactly the timeline's certified poses
// at the frame time.
func TestPartsBinClip(t *testing.T) {
	scene, timeline := partsBin(t)
	clipScene, err := scene.clipScene(timeline)
	require.NoError(t, err)
	clip, err := kinetograph.NewClip(clipScene, 60, scene.length)
	require.NoError(t, err)
	require.Equal(t, 240, clip.FrameCount())
	for i := range clip.FrameCount() {
		frame, err := clip.Frame(t.Context(), i)
		require.NoError(t, err, "frame %d", i)
		state, err := timeline.Sample(units.Seconds(float64(clip.FrameTime(i)) / 1e9))
		require.NoError(t, err, "frame %d", i)
		require.Len(t, frame.Poses, len(scene.parts))
		for k, part := range scene.parts {
			entry, ok := state.Body(part.body)
			require.True(t, ok)
			require.Equal(t, part.name, frame.Poses[k].Name)
			require.Equal(t, entry.Pose, frame.Poses[k].Transform, "frame %d part %s", i, part.name)
		}
	}
}
