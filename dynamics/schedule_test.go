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
// the step advanced instead of stopping at the touch.

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

func TestScheduledStepStopsAtAnEvent(t *testing.T) {
	// Body 0 closes the 0.5 mm gap at 8 mm/s and strikes a still body 1 at
	// 1/16 s; two still boxes that merely meet touch from the start. Both are
	// events, and the N-body step has no island solver yet.
	closing := sixBoxMotions
	closing[1].velocity = r3.Vec{}
	meeting := sixBoxMotions
	meeting[0].velocity, meeting[1].velocity = r3.Vec{}, r3.Vec{}
	meeting[1].x0 = 10
	for _, tc := range []struct {
		name    string
		motions [6]sixBoxMotion
		outcome decad.SweepOutcome
	}{
		{"closing pair", closing, decad.SweepImpactBracket},
		{"meeting boxes", meeting, decad.SweepInitiallyTouching},
	} {
		t.Run(tc.name, func(t *testing.T) {
			scene := newSixBoxScene(t, tc.motions, [6]int{0, 1, 2, 3, 4, 5}, pairMaterialStepConfig())
			report, err := scene.step(t.Context())
			require.NoError(t, err)
			require.Equal(t, dynamics.Undecided, report.Status)
			require.Nil(t, report.Next)
			require.Empty(t, report.Events)
			require.Len(t, report.Diagnostics, 1, "%+v", report.Diagnostics)
			diagnostic := report.Diagnostics[0]
			require.Equal(t, dynamics.StepUnsupported, diagnostic.Code)
			require.Equal(t, dynamics.BodyPair{A: scene.bodies[0], B: scene.bodies[1]}, diagnostic.Pair)

			// The pair's own sweep along the same drifts reports the event.
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
			require.Equal(t, tc.outcome, sweep.Outcome)
		})
	}
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
