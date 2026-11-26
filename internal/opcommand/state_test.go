package opcommand_test

import (
	"errors"
	"sync"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/stretchr/testify/require"
)

func TestStateRecordKeepsFirstFault(t *testing.T) {
	t.Parallel()
	var s opcommand.State
	subject, err := s.Fault()
	require.NoError(t, err)
	require.Empty(t, subject)

	first := errors.New("first fault")
	second := errors.New("second fault")
	s.Record("tool", nil)
	s.Record("RefineTo", first)
	s.Record("others", second)

	subject, err = s.Fault()
	require.Same(t, first, err)
	require.Equal(t, "RefineTo", subject)
}

func TestStateConsumeSucceedsOnce(t *testing.T) {
	t.Parallel()
	var s opcommand.State
	require.True(t, s.Consume())
	require.False(t, s.Consume())
	require.False(t, s.Consume())
}

func TestStateConsumeRaceHasOneWinner(t *testing.T) {
	t.Parallel()
	const callers = 64
	var s opcommand.State
	var wins atomic.Int64
	var start, done sync.WaitGroup
	start.Add(1)
	for range callers {
		done.Go(func() {
			start.Wait()
			if s.Consume() {
				wins.Add(1)
			}
		})
	}
	start.Done()
	done.Wait()
	require.Equal(t, int64(1), wins.Load())
	require.False(t, s.Consume())
}
