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

// These tests drive the step of a world of four or more bodies
// (docs/multibody-dynamics-design.md §4.3 and §5, §13 PR 3) through the real
// producers: Document.SweptBox for the broad phase, Document.SweepPair for
// every candidate pair, and SweepReport.CertifiedPosesAtInterval for replay.
//
// This PR adds no bound of its own. A pair leaves the schedule only on PR 2's
// SweptBox certificate; the one comparison the step adds is the sort-and-sweep
// walk's strict "upper X below lower X" test that retires an active body. The
// meeting-boxes fixture below was shown to fail with that test relaxed to
// "upper X at or below lower X": the meeting pair was dropped as box-clear and
// the step advanced with no island instead of solving the touch.

// sixBoxMotion places one 10 mm source box at x0 (y and z from 0 to 10) and
// gives it a velocity in mm/s and a spin in rad/s about its mass center.
type sixBoxMotion struct {
	x0       float64
	fixed    bool
	velocity r3.Vec
	spin     r3.Vec
}

// sixBoxMotions is the PR 3 scene: bodies 0 and 1 co-translate along +X
// 0.5 mm apart, body 2 spins about Z 0.9375 mm from fixed body 3, and body 5
// spins about Y 0.9375 mm from fixed body 4. Each spinning box's corners reach
// 5·(cos θ + sin θ) − 5 ≈ 0.894 mm toward its neighbor by θ = 0.2 rad, while
// its swept box grows by |ω|·ρ·dt ≈ 1.414 mm: the three near pairs overlap as
// boxes and stay clear as solids. A one-interval corner enclosure of the turn
// reaches 5·sin 0.2 ≈ 0.993 mm, past the gap, so each spinning sweep must
// refine its interval. The groups sit about 90 mm apart along X.
var sixBoxMotions = [6]sixBoxMotion{
	{x0: 0, velocity: r3.Vec{X: 8}},
	{x0: 10.5, velocity: r3.Vec{X: 8}},
	{x0: 100, spin: r3.Vec{Z: 2}},
	{x0: 110.9375, fixed: true},
	{x0: 200, fixed: true},
	{x0: 210.9375, spin: r3.Vec{Y: 2}},
}

var sixBoxNearPairs = [][2]int{{0, 1}, {2, 3}, {4, 5}}

// sixBoxSampleTimes are dyadic interior times of the 0.1 s step.
var sixBoxSampleTimes = []float64{1.0 / 32, 1.0 / 16, 3.0 / 32}

func sixBoxDt() units.Value { return units.Seconds(0.1) }

type sixBoxScene struct {
	doc     *decad.Document
	bodies  [6]*decad.Body
	logical map[*decad.Body]int
	world   *dynamics.World
	state   dynamics.State
	density units.Value
}

// newSixBoxScene builds the scene with the world's insertion order given by
// order: order[k] is the logical box inserted k-th.
func newSixBoxScene(t *testing.T, motions [6]sixBoxMotion, order [6]int,
	step dynamics.StepConfig) sixBoxScene {
	t.Helper()
	scene := sixBoxScene{doc: decad.New(), logical: make(map[*decad.Body]int, 6),
		density: units.KilogramsPerCubicMillimeter(0.001)}
	for i, motion := range motions {
		scene.bodies[i] = makeBox(t, scene.doc, motion.x0, 0, motion.x0+10, 10, 0, 10)
		scene.logical[scene.bodies[i]] = i
	}
	material := dynamics.Material{Restitution: units.Scalar(0), Friction: units.Scalar(0)}
	config := dynamics.WorldConfig{Step: step}
	entries := make([]dynamics.BodyState, 0, 6)
	for _, i := range order {
		body := dynamics.RigidBody{Body: scene.bodies[i], Role: dynamics.Dynamic, Density: &scene.density,
			Material: material}
		if motions[i].fixed {
			body.Role, body.Density = dynamics.Fixed, nil
		}
		config.Bodies = append(config.Bodies, body)
		v, w := motions[i].velocity, motions[i].spin
		entries = append(entries, dynamics.BodyState{Body: scene.bodies[i], Pose: r3.Identity(),
			LinearVelocity: dynamics.QuantityVec{X: units.MillimetersPerSecond(v.X),
				Y: units.MillimetersPerSecond(v.Y), Z: units.MillimetersPerSecond(v.Z)},
			AngularVelocity: dynamics.QuantityVec{X: units.RadiansPerSecond(w.X),
				Y: units.RadiansPerSecond(w.Y), Z: units.RadiansPerSecond(w.Z)}})
	}
	var err error
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, config)
	require.NoError(t, err)
	scene.state, err = scene.world.NewState(entries)
	require.NoError(t, err)
	return scene
}

func (s sixBoxScene) step(ctx context.Context) (*dynamics.StepReport, error) {
	return s.world.Step(ctx, s.state, dynamics.StepInput{Gravity: zeroAcceleration()}, sixBoxDt())
}

// pairOf names an unordered pair by logical box indices, smaller first.
func (s sixBoxScene) pairOf(pair dynamics.BodyPair) [2]int {
	a, b := s.logical[pair.A], s.logical[pair.B]
	if a > b {
		a, b = b, a
	}
	return [2]int{a, b}
}

// driftCorner is the closed-form drift of a box corner: a turn about the
// box's mass center by |ω|·t (Rodrigues) followed by v·t.
func driftCorner(motion sixBoxMotion, corner r3.Vec, seconds float64) r3.Vec {
	center := r3.Vec{X: motion.x0 + 5, Y: 5, Z: 5}
	offset := corner.Sub(center)
	if rate := motion.spin.Len(); rate != 0 {
		axis := motion.spin.Scale(1 / rate)
		angle := rate * seconds
		offset = offset.Scale(math.Cos(angle)).Add(axis.Cross(offset).Scale(math.Sin(angle))).
			Add(axis.Scale(axis.Dot(offset) * (1 - math.Cos(angle))))
	}
	return center.Add(offset).Add(motion.velocity.Scale(seconds))
}

// requireDriftPose compares a published pose against the closed-form drift at
// every corner of the box. The slack is 1e-9 mm: the step evaluates each pose
// with a handful of float operations on coordinates below 250 mm, whose
// rounding stays near 1e-13 mm, while a wrong angle, center or velocity moves
// a corner by far more than 1e-9 mm.
func requireDriftPose(t *testing.T, motion sixBoxMotion, pose r3.Transform, seconds float64) {
	t.Helper()
	for index := range 8 {
		corner := r3.Vec{X: motion.x0 + 10*float64(index&1), Y: 10 * float64((index>>1)&1),
			Z: 10 * float64((index>>2)&1)}
		got, want := pose.Apply(corner), driftCorner(motion, corner, seconds)
		require.InDelta(t, want.X, got.X, 1e-9, "corner %d x at %g s", index, seconds)
		require.InDelta(t, want.Y, got.Y, 1e-9, "corner %d y at %g s", index, seconds)
		require.InDelta(t, want.Z, got.Z, 1e-9, "corner %d z at %g s", index, seconds)
	}
}

func TestScheduledStepSweepsOnlyNearPairs(t *testing.T) {
	scene := newSixBoxScene(t, sixBoxMotions, [6]int{0, 1, 2, 3, 4, 5}, pairMaterialStepConfig())
	report, err := scene.step(t.Context())
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.NotNil(t, report.Next)
	require.Empty(t, report.Events)
	require.Empty(t, report.Diagnostics)
	require.NotNil(t, report.Conservation)

	// Fourteen pairs are scheduled: every pair but the fixed pair (3,4). Only
	// the three near pairs are swept; every other one carries the swept-box
	// exclusion and no pose evaluation.
	near := make(map[[2]int]struct{}, len(sixBoxNearPairs))
	for _, pair := range sixBoxNearPairs {
		near[pair] = struct{}{}
	}
	proofs := dynamics.TraceSliceProofs(report.Trace)
	require.Len(t, proofs, 1)
	require.Len(t, proofs[0], 14)
	sweptPairs := 0
	for _, proof := range proofs[0] {
		pair := scene.pairOf(proof.Pair)
		require.NotEqual(t, [2]int{3, 4}, pair, "a fixed pair is outside the schedule")
		var evaluations uint64
		if proof.Sweep != nil {
			evaluations = proof.Sweep.PoseEvaluations
		}
		if _, ok := near[pair]; !ok {
			require.True(t, proof.BoxClear, "pair %v", pair)
			require.Nil(t, proof.Sweep, "pair %v", pair)
			require.Zero(t, evaluations, "pair %v", pair)
			continue
		}
		sweptPairs++
		require.False(t, proof.BoxClear, "pair %v", pair)
		require.NotNil(t, proof.Sweep, "pair %v", pair)
		require.Equal(t, decad.SweepClear, proof.Sweep.Outcome, "pair %v", pair)
		require.Positive(t, evaluations, "pair %v", pair)
	}
	require.Equal(t, len(sixBoxNearPairs), sweptPairs)

	// The exclusions are PR 2's certificate: each body's swept box along its
	// drift is strictly disjoint from its partner's exactly for the pairs the
	// step did not sweep.
	boxes := make([]decad.SweptBox, 6)
	for i, motion := range sixBoxMotions {
		path := decad.PairPath(decad.PoseSegment{From: r3.Identity(), To: r3.Identity(), Duration: sixBoxDt()})
		if !motion.fixed {
			mass, err := scene.bodies[i].MassProperties(t.Context(), scene.density)
			require.NoError(t, err)
			entry, ok := scene.state.Body(scene.bodies[i])
			require.True(t, ok)
			path = decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
				LinearVelocity: entry.LinearVelocity, AngularVelocity: entry.AngularVelocity,
				Duration: sixBoxDt()}
		}
		boxes[i], err = scene.doc.SweptBox(t.Context(), scene.bodies[i], path)
		require.NoError(t, err)
	}
	for a := range 6 {
		for b := a + 1; b < 6; b++ {
			_, isNear := near[[2]int{a, b}]
			require.Equal(t, !isNear, boxes[a].StrictlyDisjoint(boxes[b]), "pair (%d,%d)", a, b)
		}
	}

	// The published end poses and every interior sample follow the exact
	// drift; velocities stay the slice's start velocities.
	dt := sixBoxDt().Base()
	for i, motion := range sixBoxMotions {
		entry, ok := report.Next.Body(scene.bodies[i])
		require.True(t, ok)
		requireDriftPose(t, motion, entry.Pose, dt)
		start, _ := scene.state.Body(scene.bodies[i])
		require.Equal(t, start.LinearVelocity, entry.LinearVelocity)
		require.Equal(t, start.AngularVelocity, entry.AngularVelocity)
	}
	for _, seconds := range sixBoxSampleTimes {
		sample, err := report.Trace.Sample(units.Seconds(seconds))
		require.NoError(t, err, "sample at %g s", seconds)
		for i, motion := range sixBoxMotions {
			entry, ok := sample.Body(scene.bodies[i])
			require.True(t, ok)
			requireDriftPose(t, motion, entry.Pose, seconds)
		}
	}
	first, err := report.Trace.Sample(units.Seconds(0))
	require.NoError(t, err)
	require.Equal(t, scene.state.Entries(), first.Entries())
	last, err := report.Trace.Sample(sixBoxDt())
	require.NoError(t, err)
	require.Equal(t, report.Next.Entries(), last.Entries())
}

// sixBoxOutcome is a step's result keyed by logical box, so two insertion
// orders compare directly.
type sixBoxOutcome struct {
	status      dynamics.StepStatus
	next        [6]r3.Transform
	samples     [][6]r3.Transform
	swept       map[[2]int]decad.SweepOutcome
	evaluations map[[2]int]uint64
	diagnostics map[[2]int]dynamics.StepReason
}

func (s sixBoxScene) outcome(t *testing.T, report *dynamics.StepReport) sixBoxOutcome {
	t.Helper()
	out := sixBoxOutcome{status: report.Status, swept: map[[2]int]decad.SweepOutcome{},
		evaluations: map[[2]int]uint64{}, diagnostics: map[[2]int]dynamics.StepReason{}}
	for _, diagnostic := range report.Diagnostics {
		out.diagnostics[s.pairOf(diagnostic.Pair)] = diagnostic.Code
	}
	if report.Next == nil {
		return out
	}
	for _, entry := range report.Next.Entries() {
		out.next[s.logical[entry.Body]] = entry.Pose
	}
	for _, seconds := range sixBoxSampleTimes {
		sample, err := report.Trace.Sample(units.Seconds(seconds))
		require.NoError(t, err)
		var poses [6]r3.Transform
		for _, entry := range sample.Entries() {
			poses[s.logical[entry.Body]] = entry.Pose
		}
		out.samples = append(out.samples, poses)
	}
	for _, proof := range dynamics.TraceSliceProofs(report.Trace)[0] {
		if proof.Sweep != nil {
			out.swept[s.pairOf(proof.Pair)] = proof.Sweep.Outcome
			out.evaluations[s.pairOf(proof.Pair)] = proof.Sweep.PoseEvaluations
		}
	}
	return out
}

func TestScheduledStepIgnoresInsertionOrder(t *testing.T) {
	closing := sixBoxMotions
	closing[1].velocity = r3.Vec{}
	for _, tc := range []struct {
		name    string
		motions [6]sixBoxMotion
	}{
		{"clear drift", sixBoxMotions},
		{"closing pair", closing},
	} {
		t.Run(tc.name, func(t *testing.T) {
			reference := newSixBoxScene(t, tc.motions, [6]int{0, 1, 2, 3, 4, 5}, pairMaterialStepConfig())
			report, err := reference.step(t.Context())
			require.NoError(t, err)
			want := reference.outcome(t, report)
			for _, order := range [][6]int{{5, 4, 3, 2, 1, 0}, {3, 5, 0, 4, 1, 2}} {
				scene := newSixBoxScene(t, tc.motions, order, pairMaterialStepConfig())
				report, err := scene.step(t.Context())
				require.NoError(t, err)
				require.Equal(t, want, scene.outcome(t, report), "insertion order %v", order)
			}
		})
	}
}

func TestScheduledStepSolvesAnInteriorImpact(t *testing.T) {
	// Body 0 closes the 0.5 mm gap at 8 mm/s and strikes a still body 1 at
	// 1/16 s, inside the 0.1 s slice. The step advances both boxes to the
	// impact, solves the pair as an island with restitution zero (both leave
	// at the common 4 mm/s, an impulse of 4 kg·mm/s on 1 kg boxes), and
	// continues them in persistent touch to the end of the step.
	closing := sixBoxMotions
	closing[1].velocity = r3.Vec{}
	scene := newSixBoxScene(t, closing, [6]int{0, 1, 2, 3, 4, 5}, pairMaterialStepConfig())
	report, err := scene.step(t.Context())
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Len(t, report.Events, 1)
	event := report.Events[0]
	require.Equal(t, dynamics.ContactImpact, event.Kind)
	require.Equal(t, dynamics.BodyPair{A: scene.bodies[0], B: scene.bodies[1]}, event.Pair)
	// The event lies at the sweep's bracket right sample, within one
	// TimeResolution after the exact 1/16 s and never before it.
	require.GreaterOrEqual(t, event.Time.Base(), 1.0/16)
	require.InDelta(t, 1.0/16, event.Time.Base(), pairMaterialStepConfig().TimeResolution.Base())
	require.Equal(t, units.Seconds(0), event.SliceStart)
	require.Equal(t, sixBoxDt(), event.SliceDuration)
	require.InDelta(t, 4, event.NormalImpulse.Base(), 1e-6)
	require.InDelta(t, 4, event.PostVelocityA.X.Base(), 1e-6)
	require.InDelta(t, 4, event.PostVelocityB.X.Base(), 1e-6)
	require.Len(t, report.Islands, 1)
	require.Equal(t, []*decad.Body{scene.bodies[0], scene.bodies[1]}, report.Islands[0].Bodies)
	require.Equal(t, event.Time, report.Islands[0].Time)

	// Two slices meet at the event: the approach and the shared drift.
	proofs := dynamics.TraceSliceProofs(report.Trace)
	require.Len(t, proofs, 2)
	for _, proof := range proofs[1] {
		if scene.pairOf(proof.Pair) == [2]int{0, 1} {
			require.NotNil(t, proof.Sweep)
			require.Equal(t, decad.SweepPersistentTouch, proof.Sweep.Outcome)
		}
	}
	// Positions: body 0 travels 0.5 mm in 1/16 s, then both drift at
	// 4 mm/s. The slack covers the event's bracket delay (below 1e-9 s at
	// 8 mm/s) and its correction (below 1e-8 mm).
	for _, sample := range []struct {
		at   float64
		x0   float64
		x1   float64
		next bool
	}{{1.0 / 32, 0.25, 10.5, false}, {3.0 / 32, 0.625, 10.625, false}, {0.1, 0.65, 10.65, true}} {
		state, err := report.Trace.Sample(units.Seconds(sample.at))
		require.NoError(t, err, "sample at %g s", sample.at)
		if sample.next {
			state = *report.Next
		}
		for i, want := range [2]float64{sample.x0, sample.x1 - 10.5} {
			entry, ok := state.Body(scene.bodies[i])
			require.True(t, ok)
			require.InDelta(t, want, entry.Pose.Translation().X, 1e-8, "body %d at %g s", i, sample.at)
		}
	}

	// The pair's own sweep along the full-step drifts reports the event.
	var paths [2]decad.PairPath
	for side := range paths {
		mass, err := scene.bodies[side].MassProperties(t.Context(), scene.density)
		require.NoError(t, err)
		entry, _ := scene.state.Body(scene.bodies[side])
		paths[side] = decad.RigidDriftSegment{From: r3.Identity(), Center: mass.Center.Value,
			LinearVelocity: entry.LinearVelocity, AngularVelocity: entry.AngularVelocity,
			Duration: sixBoxDt()}
	}
	config := pairMaterialStepConfig()
	sweep, err := scene.doc.SweepPair(t.Context(), scene.bodies[0], scene.bodies[1], paths[0], paths[1],
		decad.SweepRequest{ContactRequest: config.Contact, TimeResolution: config.TimeResolution,
			MaxPoseEvaluations: config.MaxPoseEvaluations, StartPolicy: decad.StopAtInitialContact})
	require.NoError(t, err)
	require.Equal(t, decad.SweepImpactBracket, sweep.Outcome)
	require.InDelta(t, event.Time.Base(), sweep.Bracket.To.Elapsed.Value.Base(), 1e-15)
}

func TestScheduledStepSolvesMeetingBoxes(t *testing.T) {
	// Two still boxes that merely meet touch from the start with zero
	// relative normal speed. The pair's island certifies four zero impulses
	// and changes no velocity, so it publishes nothing (§6.1), and the pair
	// rests on a persistent track.
	meeting := sixBoxMotions
	meeting[0].velocity, meeting[1].velocity = r3.Vec{}, r3.Vec{}
	meeting[1].x0 = 10
	scene := newSixBoxScene(t, meeting, [6]int{0, 1, 2, 3, 4, 5}, pairMaterialStepConfig())
	report, err := scene.step(t.Context())
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Islands)
	require.Empty(t, report.Events)
	for _, i := range []int{0, 1} {
		entry, ok := report.Next.Body(scene.bodies[i])
		require.True(t, ok)
		require.Equal(t, r3.Identity(), entry.Pose)
	}
	for _, proof := range dynamics.TraceSliceProofs(report.Trace)[0] {
		if scene.pairOf(proof.Pair) == [2]int{0, 1} {
			require.NotNil(t, proof.Sweep)
			require.Equal(t, decad.SweepPersistentTouch, proof.Sweep.Outcome)
		}
	}

	// The published state carries the pair in its contact set (§5 step 2),
	// and no kick changes it: the next step continues the pair on its
	// persistent track from the start, with no initial-contact stop at zero.
	// A state rebuilt from the same entries has no contact set, so its step
	// meets the touch afresh: it sweeps the pair under StopAtInitialContact,
	// solves its silent island, and then continues it the same way. Both
	// publish nothing; the carried step makes exactly one SweepPair call
	// fewer, without its cache as well. The spinning boxes are stopped here,
	// so the second step sweeps nothing but the meeting pair.
	meeting[2].spin, meeting[5].spin = r3.Vec{}, r3.Vec{}
	scene = newSixBoxScene(t, meeting, [6]int{0, 1, 2, 3, 4, 5}, pairMaterialStepConfig())
	report, err = scene.step(t.Context())
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	rebuilt, err := scene.world.NewState(report.Next.Entries())
	require.NoError(t, err)
	var calls [2]uint64
	for k, tc := range []struct {
		name string
		from dynamics.State
	}{{"carried", dynamics.WithoutCache(*report.Next)}, {"rebuilt", rebuilt}} {
		second, err := scene.world.Step(t.Context(), tc.from, dynamics.StepInput{Gravity: zeroAcceleration()},
			sixBoxDt())
		require.NoError(t, err, tc.name)
		require.Equal(t, dynamics.Advanced, second.Status, "%s: %+v", tc.name, second.Diagnostics)
		require.Empty(t, second.Events, tc.name)
		require.Empty(t, second.Islands, tc.name)
		proofs := dynamics.TraceSliceProofs(second.Trace)
		require.Len(t, proofs, 1, tc.name)
		for _, proof := range proofs[0] {
			if scene.pairOf(proof.Pair) == [2]int{0, 1} {
				require.NotNil(t, proof.Sweep, tc.name)
				require.Equal(t, decad.SweepPersistentTouch, proof.Sweep.Outcome, tc.name)
			}
		}
		for _, i := range []int{0, 1} {
			entry, ok := second.Next.Body(scene.bodies[i])
			require.True(t, ok)
			require.Equal(t, r3.Identity(), entry.Pose, tc.name)
		}
		calls[k] = dynamics.TracePairCalls(second.Trace)
	}
	require.Equal(t, calls[0]+1, calls[1])
}

func TestScheduledStepPoseBudget(t *testing.T) {
	// Two pose evaluations cannot resolve either spinning pair. The step stops
	// with StepPairUndecided on those pairs and leaves the document as it was.
	config := pairMaterialStepConfig()
	config.MaxPoseEvaluations = 2
	scene := newSixBoxScene(t, sixBoxMotions, [6]int{0, 1, 2, 3, 4, 5}, config)
	bodies := scene.doc.Bodies()
	var bounds [6]decad.Box
	for i, body := range scene.bodies {
		box, err := body.Bounds()
		require.NoError(t, err)
		bounds[i] = box
	}
	report, err := scene.step(t.Context())
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.NotEmpty(t, report.Diagnostics)
	stopped := make(map[[2]int]struct{}, len(report.Diagnostics))
	for _, diagnostic := range report.Diagnostics {
		require.Equal(t, dynamics.StepPairUndecided, diagnostic.Code, "%+v", diagnostic)
		stopped[scene.pairOf(diagnostic.Pair)] = struct{}{}
	}
	require.Contains(t, stopped, [2]int{2, 3})
	require.Contains(t, stopped, [2]int{4, 5})
	require.Equal(t, bodies, scene.doc.Bodies())
	for i, body := range scene.bodies {
		box, err := body.Bounds()
		require.NoError(t, err)
		require.Equal(t, bounds[i], box)
	}
}

// countdownContext reports cancellation once more than limit Err calls have
// been made. It wraps its parent context, as every derived context does.
type countdownContext struct {
	context.Context //nolint:containedctx // a context wrapper holds its parent
	calls           *atomic.Int64
	limit           int64
}

func (c countdownContext) Err() error {
	if c.calls.Add(1) > c.limit {
		return context.Canceled
	}
	return c.Context.Err()
}

func TestScheduledStepCancellation(t *testing.T) {
	scene := newSixBoxScene(t, sixBoxMotions, [6]int{0, 1, 2, 3, 4, 5}, pairMaterialStepConfig())
	var calls atomic.Int64
	report, err := scene.step(countdownContext{Context: t.Context(), calls: &calls, limit: math.MaxInt64})
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status)
	polls := calls.Load()
	require.Positive(t, polls)
	// Cancelling at any poll — before the kick, inside the fixed-pair, swept-box
	// and sweep loops, or inside a sweep — returns the context error and no report.
	for limit := range polls {
		var calls atomic.Int64
		report, err := scene.step(countdownContext{Context: t.Context(), calls: &calls, limit: limit})
		require.ErrorIs(t, err, context.Canceled, "cancelled at poll %d of %d", limit, polls)
		require.Nil(t, report, "cancelled at poll %d of %d", limit, polls)
	}
}

// cachePyramid is the 3-2-1 pyramid of island_test.go plus two 10 mm boxes
// drifting at 8 mm/s along X far from it, 1/64 mm apart. Their swept boxes
// overlap, so their pair is swept every slice; gravity changes their paths
// every step, while the resting pyramid repeats the same paths.
type cachePyramid struct {
	pyramidScene
	drift [2]*decad.Body
}

func newCachePyramid(t *testing.T, config dynamics.StepConfig) cachePyramid {
	t.Helper()
	scene := cachePyramid{pyramidScene: pyramidScene{doc: decad.New(),
		density: units.KilogramsPerCubicMillimeter(0.001)}}
	scene.floor = makeBox(t, scene.doc, -65, -90, 135, 110, -10, 10)
	for i, corner := range pyramidBoxes {
		scene.boxes[i] = makeBox(t, scene.doc, corner[0], corner[1], corner[0]+20, corner[1]+20, corner[2], 20)
	}
	scene.drift[0] = makeBox(t, scene.doc, 160, 0, 170, 10, 100, 10)
	scene.drift[1] = makeBox(t, scene.doc, 170+1.0/64, 0, 180+1.0/64, 10, 100, 10)
	material := dynamics.Material{Restitution: units.Scalar(0.3), Friction: units.Scalar(0)}
	bodies := []dynamics.RigidBody{{Body: scene.floor, Role: dynamics.Fixed, Material: material}}
	entries := []dynamics.BodyState{{Body: scene.floor, Pose: r3.Identity(), LinearVelocity: zeroVelocity(),
		AngularVelocity: zeroAngular(t)}}
	for _, box := range scene.boxes {
		bodies = append(bodies, dynamics.RigidBody{Body: box, Role: dynamics.Dynamic, Density: &scene.density,
			Material: material})
		entries = append(entries, dynamics.BodyState{Body: box, Pose: r3.Identity(), LinearVelocity: zeroVelocity(),
			AngularVelocity: zeroAngular(t)})
	}
	for _, box := range scene.drift {
		bodies = append(bodies, dynamics.RigidBody{Body: box, Role: dynamics.Dynamic, Density: &scene.density,
			Material: material})
		entries = append(entries, dynamics.BodyState{Body: box, Pose: r3.Identity(),
			LinearVelocity: dynamics.QuantityVec{X: units.MillimetersPerSecond(8), Y: units.MillimetersPerSecond(0),
				Z: units.MillimetersPerSecond(0)}, AngularVelocity: zeroAngular(t)})
	}
	var err error
	scene.world, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: bodies, Step: config})
	require.NoError(t, err)
	scene.state, err = scene.world.NewState(entries)
	require.NoError(t, err)
	return scene
}

func (s cachePyramid) stepFrom(ctx context.Context, from dynamics.State) (*dynamics.StepReport, error) {
	return s.world.Step(ctx, from, dynamics.StepInput{Gravity: gravityZ(-9810)}, pyramidDt())
}

// withoutIterations clears the sweep count of every island report and every
// event's solver report, the one published value a warm start changes, and
// drops the trace and the next state, which the callers compare by samples
// and entries.
func withoutIterations(report *dynamics.StepReport) *dynamics.StepReport {
	out := *report
	out.Islands = append([]dynamics.IslandReport(nil), report.Islands...)
	for i := range out.Islands {
		out.Islands[i].Solver.Iterations = 0
	}
	out.Events = append([]dynamics.ContactEvent(nil), report.Events...)
	for i := range out.Events {
		if out.Events[i].Solver != nil {
			solver := *out.Events[i].Solver
			solver.Iterations = 0
			out.Events[i].Solver = &solver
		}
	}
	// The next state holds the step's cache, which records the sweep count;
	// callers compare its entries.
	out.Trace, out.Next = dynamics.Trace{}, nil
	return &out
}

// TestScheduledStepReusesCertificates is §13 PR 8's fixture. The resting
// pyramid's second step repeats the first: the same kick from the same
// poses gives every body the same slice paths, so every pyramid pair's
// report is the first step's report (§5.3) and the island's problem is the
// first step's problem, whose proposal restarts at its fixed point (§6.2).
// The drifting pair's paths change with the kick, so it alone is swept
// again. Without the cache the second step makes every call again and solves
// the island cold, and publishes the same bits.
func TestScheduledStepReusesCertificates(t *testing.T) {
	scene := newCachePyramid(t, islandStepConfig())
	first, err := scene.stepFrom(t.Context(), scene.state)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, first.Status, "%+v", first.Diagnostics)
	require.Len(t, first.Islands, 1)
	require.Len(t, first.Events, 9)
	require.Greater(t, first.Islands[0].Solver.Iterations, 1)
	firstCalls := dynamics.TracePairCalls(first.Trace)
	require.Positive(t, firstCalls)

	second, err := scene.stepFrom(t.Context(), *first.Next)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, second.Status, "%+v", second.Diagnostics)
	require.Len(t, second.Islands, 1)
	require.Len(t, second.Events, 9)
	// A cold solve that reached a fixed point restarts there and confirms it
	// in one sweep. Whether float rounding lets the pyramid's sweeps reach an
	// exact fixed point before MaxIterations depends on the host's FMA use; a
	// solve that ran to MaxIterations republishes its proposal with its own
	// sweep count (TestScheduledStepRepublishesUnsettledIsland).
	if first.Islands[0].Solver.Iterations < islandStepConfig().MaxIterations {
		require.Equal(t, 1, second.Islands[0].Solver.Iterations, "the island restarts at its fixed point")
	} else {
		require.Equal(t, first.Islands[0].Solver.Iterations, second.Islands[0].Solver.Iterations)
	}

	// Every pyramid pair holds the first step's report in each slice; the
	// drifting pair holds a new one, the only SweepPair call of the step.
	drift := dynamics.BodyPair{A: scene.drift[0], B: scene.drift[1]}
	firstProofs, secondProofs := dynamics.TraceSliceProofs(first.Trace), dynamics.TraceSliceProofs(second.Trace)
	require.Len(t, secondProofs, len(firstProofs))
	reused, fresh := 0, 0
	for i := range secondProofs {
		require.Len(t, secondProofs[i], len(firstProofs[i]))
		for k, proof := range secondProofs[i] {
			require.Equal(t, firstProofs[i][k].Pair, proof.Pair)
			require.Equal(t, firstProofs[i][k].BoxClear, proof.BoxClear, "pair %v", proof.Pair)
			if proof.Sweep == nil {
				continue
			}
			if proof.Pair == drift {
				require.NotSame(t, firstProofs[i][k].Sweep, proof.Sweep, "slice %d", i)
				require.Equal(t, decad.SweepClear, proof.Sweep.Outcome)
				fresh++
				continue
			}
			require.Same(t, firstProofs[i][k].Sweep, proof.Sweep, "slice %d pair %v", i, proof.Pair)
			reused++
		}
	}
	// The trace records one slice: the initial contacts cut a slice of zero
	// length at the start, which replays nothing, and the rest follows the
	// island's solve. Its nine pyramid pairs rest on reused tracks.
	require.Len(t, secondProofs, 1)
	require.Equal(t, 9, reused)
	require.Equal(t, 1, fresh)
	// The second step calls decad only for the drifting boxes: one swept box
	// each and one sweep of their pair, which the slice after the event at
	// zero reuses, and each box's stationary box at its new end pose for the
	// box-exclusion check. Every other box and sweep is the first step's.
	require.Equal(t, uint64(5), dynamics.TracePairCalls(second.Trace))

	// Without the cache the second step repeats the first step's calls and
	// cold solve, and publishes the same events, islands, state and readings.
	cold, err := scene.stepFrom(t.Context(), dynamics.WithoutCache(*first.Next))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, cold.Status, "%+v", cold.Diagnostics)
	require.Equal(t, firstCalls, dynamics.TracePairCalls(cold.Trace))
	require.Equal(t, first.Islands[0].Solver.Iterations, cold.Islands[0].Solver.Iterations)
	require.Equal(t, withoutIterations(cold), withoutIterations(second))
	require.Equal(t, cold.Next.Entries(), second.Next.Entries())
	for _, seconds := range []float64{0, 1.0 / 1024, 1.0 / 512, 1.0 / 256} {
		want, err := cold.Trace.Sample(units.Seconds(seconds))
		require.NoError(t, err, "sample at %g s", seconds)
		got, err := second.Trace.Sample(units.Seconds(seconds))
		require.NoError(t, err, "sample at %g s", seconds)
		require.Equal(t, want.Entries(), got.Entries(), "sample at %g s", seconds)
	}

	// Cancelling the cached step at any poll returns the context error.
	var calls atomic.Int64
	_, err = scene.stepFrom(countdownContext{Context: t.Context(), calls: &calls, limit: math.MaxInt64}, *first.Next)
	require.NoError(t, err)
	polls := calls.Load()
	for limit := range polls {
		var calls atomic.Int64
		report, err := scene.stepFrom(countdownContext{Context: t.Context(), calls: &calls, limit: limit}, *first.Next)
		require.ErrorIs(t, err, context.Canceled, "cancelled at poll %d of %d", limit, polls)
		require.Nil(t, report, "cancelled at poll %d of %d", limit, polls)
	}
}

// TestScheduledStepRepublishesUnsettledIsland caps the pyramid's sweeps at
// 64, fewer than its solve needs to reach a fixed point on any host, where
// the certificate already passes. The second step's island matches the
// first exactly, so it republishes the first step's proposal with its 64
// sweeps rather than sweeping on from it, and every published value matches
// the cache-free step.
func TestScheduledStepRepublishesUnsettledIsland(t *testing.T) {
	config := islandStepConfig()
	config.MaxIterations = 64
	scene := newCachePyramid(t, config)
	first, err := scene.stepFrom(t.Context(), scene.state)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, first.Status, "%+v", first.Diagnostics)
	require.Len(t, first.Islands, 1)
	require.Equal(t, 64, first.Islands[0].Solver.Iterations)
	second, err := scene.stepFrom(t.Context(), *first.Next)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, second.Status, "%+v", second.Diagnostics)
	require.Equal(t, 64, second.Islands[0].Solver.Iterations)
	cold, err := scene.stepFrom(t.Context(), dynamics.WithoutCache(*first.Next))
	require.NoError(t, err)
	require.Equal(t, withoutIterations(cold), withoutIterations(second))
	require.Equal(t, cold.Islands[0].Solver, second.Islands[0].Solver)
	require.Equal(t, cold.Next.Entries(), second.Next.Entries())
}

// TestScheduledStepPairBudget charges MaxPairSweeps with every SweptBox and
// SweepPair call (§3.3, §12). A budget one call short of the first step's
// stops it with StepPairBudget, its limit as a scalar, and the document
// unchanged; a budget of exactly that many calls advances, and the second
// step, which reuses the first step's certificates, fits in it as well.
func TestScheduledStepPairBudget(t *testing.T) {
	reference := newCachePyramid(t, islandStepConfig())
	first, err := reference.stepFrom(t.Context(), reference.state)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, first.Status, "%+v", first.Diagnostics)
	calls := dynamics.TracePairCalls(first.Trace)

	config := islandStepConfig()
	config.MaxPairSweeps = calls - 1
	scene := newCachePyramid(t, config)
	bodies := scene.doc.Bodies()
	report, err := scene.stepFrom(t.Context(), scene.state)
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Nil(t, report.Next)
	require.Len(t, report.Diagnostics, 1)
	diagnostic := report.Diagnostics[0]
	require.Equal(t, dynamics.StepPairBudget, diagnostic.Code, "%+v", diagnostic)
	require.Equal(t, units.Scalar(float64(calls-1)), diagnostic.Limit)
	require.Equal(t, pyramidDt(), diagnostic.To)
	require.Equal(t, calls-1, dynamics.TracePairCalls(report.Trace))
	require.Equal(t, bodies, scene.doc.Bodies())

	config.MaxPairSweeps = calls
	scene = newCachePyramid(t, config)
	report, err = scene.stepFrom(t.Context(), scene.state)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	second, err := scene.stepFrom(t.Context(), *report.Next)
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, second.Status, "%+v", second.Diagnostics)
	require.Less(t, dynamics.TracePairCalls(second.Trace), calls)

	_, err = dynamics.NewWorld(t.Context(), scene.doc, dynamics.WorldConfig{Bodies: []dynamics.RigidBody{
		{Body: scene.floor, Role: dynamics.Fixed}, {Body: scene.boxes[0], Role: dynamics.Dynamic,
			Density: &scene.density}}, Step: func() dynamics.StepConfig {
		config := islandStepConfig()
		config.MaxPairSweeps = 0
		return config
	}()})
	require.ErrorIs(t, err, dynamics.ErrInvalidInput)
}
