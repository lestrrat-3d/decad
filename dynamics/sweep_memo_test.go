package dynamics_test

import (
	"reflect"
	"testing"

	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// sweepMemoRun steps world from state for the given number of steps with the
// sweep memos (motionbound.SetMemos) switched on or off, and returns every
// report.
func sweepMemoRun(t *testing.T, world *dynamics.World, state dynamics.State, steps int,
	on bool) []*dynamics.StepReport {
	t.Helper()
	previous := motionbound.SetMemos(on)
	defer motionbound.SetMemos(previous)
	var reports []*dynamics.StepReport
	for k := range steps {
		report, err := world.Step(t.Context(), state, dynamics.StepInput{Gravity: gravityZ(-9810)}, pyramidDt())
		require.NoError(t, err)
		require.Equal(t, dynamics.Advanced, report.Status, "step %d: %+v", k, report.Diagnostics)
		reports = append(reports, report)
		state = *report.Next
	}
	return reports
}

// TestSweepMemosChangeNoStep runs the rotating scenes with the sweep memos
// off and then on, from the same state on the same bodies, and requires
// every step's published state, events and scheduled-pair certificates
// equal: the memos (contact_sweep_memo.go, motionbound's RadianSinCos memo)
// change no value. Each run is cut short of its test's full length but past
// the steps whose sweeps read the memos: the wedge's rotating sweeps from
// step 15, the box's from step 17, the bottle's band sweeps from step 0, the
// stack's swept boxes (the sweep radius memo) and the tumble subset's
// spinning boxes. The scheduled-pair certificates hold the sweeps' proof
// copies of their paths, which a memo left open after its run makes differ:
// with close a no-op, the wedge, box, bottle and tumble runs fail.
//
// It flips the process-wide memo switch and is the only test that does, so
// each of its runs reads the setting it chose; a test running meanwhile
// computes the same values either way, only slower while the memos are off.
func TestSweepMemosChangeNoStep(t *testing.T) {
	t.Parallel()
	if raceDetector {
		t.Skip("the race detector takes these runs past the package's test budget")
	}
	compare := func(t *testing.T, world *dynamics.World, state dynamics.State, steps int) {
		t.Helper()
		off := sweepMemoRun(t, world, state, steps, false)
		on := sweepMemoRun(t, world, state, steps, true)
		for k := range steps {
			require.Equal(t, off[k].Next.Entries(), on[k].Next.Entries(), "step %d", k)
			require.Equal(t, off[k].Events, on[k].Events, "step %d", k)
			// DeepEqual, not Equal: a failure must not print the certificates'
			// whole proof graphs. Any error they hold must match in value too.
			same := reflect.DeepEqual(dynamics.TraceSliceProofs(off[k].Trace), //nolint:govet // see above
				dynamics.TraceSliceProofs(on[k].Trace))
			require.True(t, same, "step %d: the scheduled-pair certificates differ", k)
		}
	}
	t.Run("wedge", func(t *testing.T) {
		scene := newTumbleScene(t, tumbleRestConfig(),
			tumbleBody{body: tumbleHexBody(t), pose: tumbleRelease(t, r3.Vec{Y: -1}, 37, r3.Vec{X: -55, Z: 20}),
				spin: zeroAngular(t)},
			tumbleBody{body: tumbleWedgeBody(t), pose: tumbleRelease(t, r3.Vec{X: 1, Y: 2, Z: 3}, 50,
				r3.Vec{X: 40, Z: 25}), spin: zeroAngular(t)})
		compare(t, scene.world, scene.state, 24)
	})
	t.Run("box spins", func(t *testing.T) {
		config := tumbleRestConfig()
		config.ContactSlop = units.Millimeters(1e-12)
		scene := newTumbleScene(t, config, tumbleBody{body: tumbleBoxBody(t),
			pose: tumbleRelease(t, r3.Vec{X: 1, Y: 1}, 60, r3.Vec{X: -45, Y: 45, Z: 40}), spin: tumbleBoxSpin()})
		compare(t, scene.world, scene.state, 30)
	})
	t.Run("bottle", func(t *testing.T) {
		scene := newBottleScene(t)
		world, state := scene.world(t, partsBinConfig(), translation(t, r3.Vec{Z: 8}), 0)
		compare(t, world, state, 8)
	})
	t.Run("stack", func(t *testing.T) {
		scene := newStackAndDrop(t)
		compare(t, scene.world, scene.state, 8)
	})
	t.Run("tumble", func(t *testing.T) {
		scene := newTumble(t, "box0", "hexagon", "wedge", "tetrahedron")
		compare(t, scene.world, scene.state, 4)
	})
}
