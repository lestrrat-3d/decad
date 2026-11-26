package topo_test

// CI: isolated-tests

import (
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
