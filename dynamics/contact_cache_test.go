package dynamics_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/dynamics"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// bandReadingRun is what one requireBandReadings run saw.
type bandReadingRun struct {
	readings int // band readings the published caches held
	cuts     int // readings that cut the slice before its end
	reused   int // readings of a report the previous step's cache held
}

// requireBandReadings steps world from state once per input, twice over: once
// carrying each published state's reuse cache, and once dropping it before
// every step, so every band is read afresh. After each carried step every
// band reading the published cache holds must equal the reading
// World.bandEnd returns for the same report afresh, and both runs must
// publish the same status, states and events. It stops after the first step
// that does not advance.
func requireBandReadings(t *testing.T, world *dynamics.World, state dynamics.State, dt units.Value,
	inputs []dynamics.StepInput) bandReadingRun {
	t.Helper()
	var run bandReadingRun
	cached, fresh := state, state
	for k, input := range inputs {
		previous := map[any]struct{}{}
		for _, reading := range dynamics.CachedBandReadings(cached) {
			previous[reading.Sweep] = struct{}{}
		}
		report, err := world.Step(t.Context(), cached, input, dt)
		require.NoError(t, err)
		uncached, err := world.Step(t.Context(), dynamics.WithoutCache(fresh), input, dt)
		require.NoError(t, err)
		require.Equal(t, report.Status, uncached.Status, "step %d", k)
		require.Equal(t, report.Events, uncached.Events, "step %d", k)
		if report.Status != dynamics.Advanced {
			return run
		}
		require.Equal(t, report.Next.Entries(), uncached.Next.Entries(), "step %d", k)
		for _, reading := range dynamics.CachedBandReadings(*report.Next) {
			run.readings++
			require.Equal(t, reading.FreshFull, reading.HeldFull, "step %d", k)
			require.Equal(t, reading.FreshOK, reading.HeldOK, "step %d", k)
			require.Equal(t, reading.FreshCut == nil, reading.HeldCut == nil, "step %d", k)
			if reading.FreshCut != nil {
				run.cuts++
				require.Zero(t, reading.FreshCut.Cmp(reading.HeldCut), "step %d", k)
				require.Equal(t, reading.FreshCut.RatString(), reading.HeldCut.RatString(), "step %d", k)
			}
			if _, ok := previous[reading.Sweep]; ok {
				run.reused++
			}
		}
		cached, fresh = *report.Next, *uncached.Next
	}
	return run
}

// TestBandReadingsMatchFreshReadings checks the reuse cache's band readings
// (contact_cache.go's bandEnd) against fresh ones on two band scenes. The
// knob-and-octagon scene (contact_band_test.go) lands the octagon on its band
// each step under gravity and then carries the resting band pair without
// it, so a later step reuses an earlier step's reading. The tipping cube
// (tip_test.go) rides band tracks that end inside the slice, so its readings
// carry cuts.
//
// Legs shown to fail (each perturbed in turn, test red, then restored):
//   - a reading served from the input state's cache turned into a refusal:
//     the knob's second step publishes a different status from the run
//     without the cache;
//   - a computed reading stored with its cut moved by 2⁻²⁰: the tipping
//     cube's held cut differs from the fresh one.
func TestBandReadingsMatchFreshReadings(t *testing.T) {
	t.Parallel()
	gravity := dynamics.StepInput{Gravity: gravityZ(-9810)}
	still := dynamics.StepInput{Gravity: zeroAcceleration()}
	dt := units.Seconds(1.0 / 256)
	t.Run("knob", func(t *testing.T) {
		t.Parallel()
		scene := newBandScene(t, 1e-3, dynamics.Dynamic)
		run := requireBandReadings(t, scene.world, scene.state, dt,
			[]dynamics.StepInput{gravity, gravity, gravity, still, still, still})
		require.Positive(t, run.readings)
		require.Positive(t, run.reused, "a later step reuses an earlier step's band reading")
	})
	t.Run("tip", func(t *testing.T) {
		t.Parallel()
		scene := newTipScene(t, units.Value{})
		run := requireBandReadings(t, scene.world, scene.state, dt,
			[]dynamics.StepInput{gravity, gravity, gravity, gravity})
		require.Positive(t, run.cuts, "a band track ends inside its slice")
	})
}
