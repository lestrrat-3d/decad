package decad

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/stretchr/testify/require"
)

// probeCommand stands in for an operation command: it embeds the shared state
// and counts how often its operation body runs. runCommand is unexported, so
// the probe has to live in package decad.
type probeCommand struct {
	state opcommand.State
	runs  atomic.Int64
	value int
	err   error
}

func (c *probeCommand) Do(ctx context.Context) (int, error) {
	return runCommand(ctx, &c.state, OperationUnion, c.run)
}

func (c *probeCommand) run(context.Context) (int, error) {
	c.runs.Add(1)
	return c.value, c.err
}

// requireTagged asserts err carries exactly one input-stage diagnostic naming
// the probe's kind and subject, without dumping the error graph on failure.
func requireTagged(t *testing.T, err error, subject string) {
	t.Helper()
	diagnosed, ok := errors.AsType[OperationDiagnosticError](err)
	require.True(t, ok, "the error must carry operation diagnostics")
	diagnostics := diagnosed.OperationDiagnostics()
	require.Len(t, diagnostics, 1)
	require.Equal(t, OperationUnion, diagnostics[0].Operation)
	require.Equal(t, OperationStageInput, diagnostics[0].Stage)
	require.Equal(t, OpDiagInputRejected, diagnostics[0].Code)
	require.Equal(t, subject, diagnostics[0].Subject.Name)
}

func TestRunCommand(t *testing.T) {
	t.Parallel()

	t.Run("success returns the operation's value", func(t *testing.T) {
		t.Parallel()
		cmd := &probeCommand{value: 7}
		got, err := cmd.Do(t.Context())
		require.NoError(t, err)
		require.Equal(t, 7, got)
		require.Equal(t, int64(1), cmd.runs.Load())
	})

	t.Run("a second Do returns ErrConsumed without running", func(t *testing.T) {
		t.Parallel()
		cmd := &probeCommand{value: 7}
		_, err := cmd.Do(t.Context())
		require.NoError(t, err)
		got, err := cmd.Do(t.Context())
		require.ErrorIs(t, err, ErrConsumed)
		require.Zero(t, got)
		requireTagged(t, err, "")
		require.Equal(t, int64(1), cmd.runs.Load())
	})

	t.Run("a refused Do still consumes the command", func(t *testing.T) {
		t.Parallel()
		cmd := &probeCommand{err: fmt.Errorf("%w: refused", ErrUnsupported)}
		_, err := cmd.Do(t.Context())
		require.ErrorIs(t, err, ErrUnsupported)
		_, err = cmd.Do(t.Context())
		require.ErrorIs(t, err, ErrConsumed)
		require.NotErrorIs(t, err, ErrUnsupported)
		require.Equal(t, int64(1), cmd.runs.Load())
	})

	t.Run("consumption is checked before a nil ctx", func(t *testing.T) {
		t.Parallel()
		cmd := &probeCommand{}
		_, err := cmd.Do(t.Context())
		require.NoError(t, err)
		//nolint:staticcheck // SA1012: the nil context is the input under test.
		_, err = cmd.Do(nil)
		require.ErrorIs(t, err, ErrConsumed)
		require.NotErrorIs(t, err, ErrDegenerate)
	})

	t.Run("a nil ctx is refused before a recorded fault and consumes", func(t *testing.T) {
		t.Parallel()
		cmd := &probeCommand{}
		cmd.state.Record("RefineTo", fmt.Errorf("%w: a recorded fault", ErrNegativeMagnitude))
		//nolint:staticcheck // SA1012: the nil context is the input under test.
		_, err := cmd.Do(nil)
		require.ErrorIs(t, err, ErrDegenerate)
		require.NotErrorIs(t, err, ErrNegativeMagnitude)
		requireTagged(t, err, "")
		require.Zero(t, cmd.runs.Load())
		_, err = cmd.Do(t.Context())
		require.ErrorIs(t, err, ErrConsumed)
	})

	t.Run("a recorded fault surfaces before the operation runs", func(t *testing.T) {
		t.Parallel()
		cmd := &probeCommand{value: 7}
		cmd.state.Record("RefineTo", fmt.Errorf("%w: a recorded fault", ErrNegativeMagnitude))
		got, err := cmd.Do(t.Context())
		require.ErrorIs(t, err, ErrNegativeMagnitude)
		require.Zero(t, got)
		requireTagged(t, err, "RefineTo")
		require.Zero(t, cmd.runs.Load())
		_, err = cmd.Do(t.Context())
		require.ErrorIs(t, err, ErrConsumed)
	})

	t.Run("a cancellation error is returned by identity", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		cmd := &probeCommand{err: ctx.Err()}
		_, err := cmd.Do(ctx)
		require.Equal(t, context.Canceled, err)
		_, diagnosed := errors.AsType[OperationDiagnosticError](err)
		require.False(t, diagnosed, "cancellation passes through untagged")

		deadline := &probeCommand{err: context.DeadlineExceeded}
		_, err = deadline.Do(t.Context())
		require.Equal(t, context.DeadlineExceeded, err)
	})

	t.Run("an operation error is tagged with the command's kind", func(t *testing.T) {
		t.Parallel()
		cmd := &probeCommand{err: fmt.Errorf("%w: a bad operand", ErrRetiredBody)}
		_, err := cmd.Do(t.Context())
		require.ErrorIs(t, err, ErrRetiredBody)
		requireTagged(t, err, "")
	})

	t.Run("racing Do calls run the operation once", func(t *testing.T) {
		t.Parallel()
		const callers = 64
		cmd := &probeCommand{value: 7}
		var successes, consumed atomic.Int64
		var start, done sync.WaitGroup
		start.Add(1)
		for range callers {
			done.Go(func() {
				start.Wait()
				_, err := cmd.Do(t.Context())
				switch {
				case err == nil:
					successes.Add(1)
				case errors.Is(err, ErrConsumed):
					consumed.Add(1)
				}
			})
		}
		start.Done()
		done.Wait()
		require.Equal(t, int64(1), cmd.runs.Load())
		require.Equal(t, int64(1), successes.Load())
		require.Equal(t, int64(callers-1), consumed.Load())
	})
}
