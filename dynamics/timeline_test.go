package dynamics_test

import (
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These tests chain steps through dynamics.Timeline
// (docs/multibody-dynamics-design.md §7.2, §13 PR 6).

// gravityBounceInput is gravity −8192 mm/s², so each 1/32 s step kicks the
// sphere by exactly −256 mm/s.
func gravityBounceInput() dynamics.StepInput {
	return dynamics.StepInput{Gravity: gravityZ(-8192)}
}

func gravityBounceDt() units.Value { return units.Seconds(1.0 / 32) }

// The gravity bounce, in closed form for the step's discrete law: one kick
// per step, then drift at constant velocity. The sphere's center starts at
// rest at z = 9, 4 mm above floor touch (z = 5).
//   - Step 1: the kick gives −256 mm/s; impact at 4/256 = 1/64 s; restitution
//     0.5 leaves at +128 mm/s and the step ends at z = 5 + 128/64 = 7.
//   - Step 2: the kick leaves −128 mm/s; impact after 2/128 = 1/64 s, at
//     3/64 s; leaves at +64 mm/s; ends at z = 6.
//   - Step 3: the kick leaves −192 mm/s; impact after 1/192 s, at 13/192 s;
//     leaves at +96 mm/s; ends at z = 5 + 96·(5/192) = 7.5.
var gravityBounces = []struct {
	at      float64 // exact impact time from the timeline start
	pre     float64
	post    float64
	impulse float64
}{
	{1.0 / 64, -256, 128, 384},
	{3.0 / 64, -128, 64, 192},
	{13.0 / 192, -192, 96, 288},
}

// gravityBounceHeight is the sphere's exact center height and velocity at
// time t from the timeline start, during the three steps.
func gravityBounceHeight(at float64) (float64, float64) {
	starts := []struct{ z, v float64 }{{9, 0}, {7, 128}, {6, 64}}
	step := min(int(at*32), 2)
	if at*32 == float64(step) {
		// A step boundary belongs to the later step's start, before its kick.
		return starts[step].z, starts[step].v
	}
	local := at - float64(step)/32
	v := starts[step].v - 256
	bounce := gravityBounces[step]
	impact := bounce.at - float64(step)/32
	if local <= impact {
		return starts[step].z + v*local, v
	}
	return 5 + bounce.post*(local-impact), bounce.post
}

// TestTimelineGravityBounce is §13 PR 6's fixture: a sphere bouncing on a
// floor with restitution 0.5 over three steps of a world of four bodies. Each
// bounce lies at its closed-form time within TimeResolution, since every
// step starts from a state its predecessor certified and each impact lies at
// its SweepPair bracket's right sample. Positions carry that delay times the
// speed, below 256 mm/s · 1e-9 s, so their slack is 1e-6 mm.
func TestTimelineGravityBounce(t *testing.T) {
	scene := newBounceScene(t, 0, 9, 0, bounceStepConfig())
	timeline, err := dynamics.NewTimeline(scene.world, scene.state)
	require.NoError(t, err)
	require.Equal(t, units.Seconds(0), timeline.End())
	resolution := bounceStepConfig().TimeResolution.Base()
	for k, bounce := range gravityBounces {
		report, err := timeline.Advance(t.Context(), gravityBounceInput(), gravityBounceDt())
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", k, report.Diagnostics)
		require.Len(t, report.Events, 1, "step %d", k)
		event := report.Events[0]
		start := float64(k) / 32
		require.Equal(t, dynamics.BodyPair{A: scene.floor, B: scene.ball}, event.Pair, "step %d", k)
		require.GreaterOrEqual(t, start+event.Time.Base(), bounce.at, "step %d", k)
		require.InDelta(t, bounce.at, start+event.Time.Base(), resolution, "step %d", k)
		require.InDelta(t, bounce.pre, event.PreVelocityB.Z.Base(), 1e-9, "step %d", k)
		require.InDelta(t, bounce.post, event.PostVelocityB.Z.Base(), 1e-6, "step %d", k)
		require.InDelta(t, bounce.impulse, event.NormalImpulse.Base(), 1e-6, "step %d", k)
		require.Equal(t, units.Seconds(float64(k+1)/32), timeline.End(), "step %d", k)
	}
	require.Nil(t, timeline.Stopped())
	steps := timeline.Steps()
	require.Len(t, steps, 3)
	z, v := scene.ballZ(t, *steps[2].Next)
	require.InDelta(t, 7.5, z, 1e-6)
	require.InDelta(t, 96, v, 1e-6)

	// Samples across the timeline, the two step boundaries included.
	for _, at := range []float64{0, 1.0 / 128, 3.0 / 128, 1.0/32 - 1.0/256, 1.0 / 32, 1.0/32 + 1.0/256,
		1.0/16 - 1.0/1024, 1.0 / 16, 1.0/16 + 1.0/512, 1.0/16 + 1.0/64, 3.0 / 32} {
		state, err := timeline.Sample(units.Seconds(at))
		require.NoError(t, err, "sample at %g s", at)
		wantZ, wantV := gravityBounceHeight(at)
		z, v := scene.ballZ(t, state)
		require.InDelta(t, wantZ, z, 1e-6, "sample at %g s", at)
		require.InDelta(t, wantV, v, 1e-6, "sample at %g s", at)
	}
	// A boundary is the earlier step's end and the later step's start.
	boundary, err := timeline.Sample(units.Seconds(1.0 / 32))
	require.NoError(t, err)
	require.Equal(t, steps[0].Next.Entries(), boundary.Entries())
	// Sample(End()) is the last step's certified end state.
	last, err := timeline.Sample(timeline.End())
	require.NoError(t, err)
	require.Equal(t, steps[2].Next.Entries(), last.Entries())
	// Times outside [0, End()] have no certified state.
	for _, at := range []float64{-1.0 / 1024, 3.0/32 + 1.0/1024} {
		_, err := timeline.Sample(units.Seconds(at))
		require.ErrorIs(t, err, dynamics.ErrUnsupported, "sample at %g s", at)
	}
}

// TestTimelineStopsAtEventBudget advances the floor-and-ceiling bounce of
// TestScheduledStepBounces with MaxEvents 3. A first short step ends before
// the first impact; the second needs five events and stops with
// StepEventBudget. The timeline keeps its certified end and refuses to
// advance further.
func TestTimelineStopsAtEventBudget(t *testing.T) {
	config := bounceStepConfig()
	config.MaxEvents = 3
	scene := newBounceScene(t, 12, 6, -256, config)
	timeline, err := dynamics.NewTimeline(scene.world, scene.state)
	require.NoError(t, err)
	input := dynamics.StepInput{Gravity: zeroAcceleration()}
	report, err := timeline.Advance(t.Context(), input, units.Seconds(1.0/512))
	require.NoError(t, err)
	require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	require.Empty(t, report.Events)
	report, err = timeline.Advance(t.Context(), input, units.Seconds(.25))
	require.NoError(t, err)
	require.Equal(t, dynamics.Undecided, report.Status)
	require.Len(t, report.Diagnostics, 1)
	require.Equal(t, dynamics.StepEventBudget, report.Diagnostics[0].Code)
	require.Same(t, report, timeline.Stopped())
	require.Equal(t, units.Seconds(1.0/512), timeline.End())
	require.Len(t, timeline.Steps(), 1)
	_, err = timeline.Advance(t.Context(), input, units.Seconds(.25))
	require.ErrorIs(t, err, dynamics.ErrTimelineStopped)
	// The certified end is the first step's end: the sphere fell 0.5 mm.
	end, err := timeline.Sample(timeline.End())
	require.NoError(t, err)
	z, v := scene.ballZ(t, end)
	require.Equal(t, 5.5, z)
	require.Equal(t, -256.0, v)
	_, err = timeline.Sample(units.Seconds(1.0 / 256))
	require.ErrorIs(t, err, dynamics.ErrUnsupported)
}

// TestTimelineConcurrentSample samples one advanced timeline from many
// goroutines at once (run it with -race). Sample writes nothing, so every
// call returns the state a sequential call returns for the same time.
func TestTimelineConcurrentSample(t *testing.T) {
	scene := newBounceScene(t, 0, 9, 0, bounceStepConfig())
	timeline, err := dynamics.NewTimeline(scene.world, scene.state)
	require.NoError(t, err)
	for range 3 {
		report, err := timeline.Advance(t.Context(), gravityBounceInput(), gravityBounceDt())
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "%+v", report.Diagnostics)
	}
	var times []units.Value
	for i := range 97 {
		times = append(times, units.Seconds(float64(i)*(3.0/32)/96))
	}
	want := make([][]dynamics.BodyState, len(times))
	for i, at := range times {
		state, err := timeline.Sample(at)
		require.NoError(t, err, "sample %d", i)
		want[i] = state.Entries()
	}
	const workers = 8
	got := make([][][]dynamics.BodyState, workers)
	errs := make([][]error, workers)
	var wg sync.WaitGroup
	for worker := range workers {
		wg.Go(func() {
			got[worker] = make([][]dynamics.BodyState, len(times))
			errs[worker] = make([]error, len(times))
			// Each worker walks the times in a different order.
			for k := range times {
				i := (k*(worker+1) + worker) % len(times)
				state, err := timeline.Sample(times[i])
				got[worker][i], errs[worker][i] = state.Entries(), err
			}
		})
	}
	wg.Wait()
	for worker := range workers {
		for i := range times {
			require.NoError(t, errs[worker][i], "worker %d sample %d", worker, i)
			require.Equal(t, want[i], got[worker][i], "worker %d sample %d", worker, i)
		}
	}
}

func TestNewTimelineRejectsForeignState(t *testing.T) {
	scene := newBounceScene(t, 0, 9, 0, bounceStepConfig())
	other := newBounceScene(t, 0, 9, 0, bounceStepConfig())
	_, err := dynamics.NewTimeline(scene.world, other.state)
	require.ErrorIs(t, err, dynamics.ErrInvalidInput)
	_, err = dynamics.NewTimeline(nil, scene.state)
	require.ErrorIs(t, err, dynamics.ErrInvalidInput)
}
