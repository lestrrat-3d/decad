package topo_test

// CI: isolated-tests

import (
	"runtime"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// TestStepProducerBlock checks that a step's producer cursor refuses an advance its block cannot hold, changes
// nothing when it refuses, and that the refusal is sticky for later mints.
func TestStepProducerBlock(t *testing.T) {
	doc := &topo.Document{}
	run, err := doc.BeginRun(2, 4)
	require.NoError(t, err)
	s := run.Step(1)
	require.EqualValues(t, 4, s.ProducerNext(), "step 1's block starts after step 0's")
	require.NoError(t, s.AdvanceProducer(3))
	require.NoError(t, s.AdvanceProducer(1))
	require.ErrorIs(t, s.AdvanceProducer(1), decaderr.ErrStepIdentities)
	require.EqualValues(t, 8, s.ProducerNext(), "a refused advance moves nothing")
	require.True(t, s.Exhausted())
	s.RewindProducer(5)
	require.ErrorIs(t, s.AdvanceProducer(1), decaderr.ErrStepIdentities, "the exhausted flag stays set")
	doc.EndRun(run, nil)
	require.EqualValues(t, 8, doc.NextProducer)
	require.EqualValues(t, 8, doc.NextLevel)
	require.EqualValues(t, 8, doc.NextCurve)
	require.Nil(t, doc.ActiveRun())
}

// TestBeginRunRefusals checks the refusals of BeginRun: a second run, a degenerate size and a total past the
// identity range.
func TestBeginRunRefusals(t *testing.T) {
	doc := &topo.Document{}
	_, err := doc.BeginRun(0, 4)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	_, err = doc.BeginRun(1, 0)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	_, err = doc.BeginRun(4, 1<<62)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	run, err := doc.BeginRun(1, 4)
	require.NoError(t, err)
	_, err = doc.BeginRun(1, 4)
	require.ErrorIs(t, err, decaderr.ErrStepScope)
	doc.EndRun(run, nil)
	_, err = doc.BeginRun(1, 4)
	require.NoError(t, err, "a run can start once the last ended")
}

// TestStepAdvanceProducerOverlapFailsRun checks that several goroutines advancing one step's cursor at once,
// which would hand two commits the same identity, mark the run failed.
func TestStepAdvanceProducerOverlapFailsRun(t *testing.T) {
	doc := &topo.Document{}
	run, err := doc.BeginRun(1, 1<<30)
	require.NoError(t, err)
	s := run.Step(0)
	if runtime.GOMAXPROCS(0) < 2 {
		t.Skip("overlap needs goroutines running in parallel")
	}
	start := make(chan struct{})
	var wg sync.WaitGroup
	for range 8 {
		wg.Go(func() {
			<-start
			for range 100000 {
				_ = s.AdvanceProducer(1)
				if run.Fault() != nil {
					return
				}
			}
		})
	}
	close(start)
	wg.Wait()
	require.Error(t, run.Fault(), "overlapping advances in one step must fail the run")
	doc.EndRun(run, nil)
}

// TestStepAdvanceProducerSerialStaysQuiet checks that one goroutine advancing the cursor never fails the run.
func TestStepAdvanceProducerSerialStaysQuiet(t *testing.T) {
	doc := &topo.Document{}
	run, err := doc.BeginRun(1, 1<<30)
	require.NoError(t, err)
	s := run.Step(0)
	for range 20000 {
		require.NoError(t, s.AdvanceProducer(1))
	}
	require.EqualValues(t, 20000, s.ProducerNext())
	require.NoError(t, run.Fault())
	doc.EndRun(run, nil)
}

// TestStepExhaustionFailsRun checks that running past the producer block marks the run failed, so a route that
// swallows ErrStepIdentities cannot yield a result that differs from a serial build.
func TestStepExhaustionFailsRun(t *testing.T) {
	doc := &topo.Document{}
	run, err := doc.BeginRun(1, 2)
	require.NoError(t, err)
	s := run.Step(0)
	require.NoError(t, s.AdvanceProducer(2))
	require.NoError(t, run.Fault())
	require.ErrorIs(t, s.AdvanceProducer(1), decaderr.ErrStepIdentities)
	require.ErrorIs(t, run.Fault(), decaderr.ErrStepIdentities)
	require.NotErrorIs(t, run.Fault(), decaderr.ErrStepScope)
	doc.EndRun(run, nil)
}
