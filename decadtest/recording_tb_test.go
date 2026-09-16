package decadtest_test

import (
	"fmt"
	"runtime"
	"strings"
	"sync"
	"testing"

	"github.com/stretchr/testify/require"
)

// recordingTB captures what a decadtest helper reports instead of failing
// the real test, so a helper can be SHOWN to fail. testing.TB is embedded so
// the struct satisfies the interface's unexported method; the embedded value
// is the real *testing.T, which is what makes tb.Context() usable inside
// Region.
type recordingTB struct {
	testing.TB
	mu       sync.Mutex
	messages []string
	failed   bool
}

// Helper is a no-op override: recordingTB is not the real test, so there is
// no frame to hide from a failure line.
func (r *recordingTB) Helper() {}

// Fail marks the recorder failed, without terminating the calling
// goroutine — the override that Errorf's contract needs.
func (r *recordingTB) Fail() {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.failed = true
}

// Failed reports whether Fail, Errorf or Fatalf has run.
func (r *recordingTB) Failed() bool {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.failed
}

// FailNow marks the recorder failed, then calls runtime.Goexit. FailNow is
// defined to call runtime.Goexit, which terminates the calling goroutine, so
// a helper invoked inline (not on its own goroutine) would abandon the test
// function that called it; captureFailure runs fn on its own goroutine for
// exactly this reason.
func (r *recordingTB) FailNow() {
	r.Fail()
	runtime.Goexit()
}

// Errorf appends the formatted message and marks the recorder failed,
// without stopping the calling goroutine.
func (r *recordingTB) Errorf(format string, args ...any) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.messages = append(r.messages, fmt.Sprintf(format, args...))
	r.failed = true
}

// Fatalf appends the formatted message, then calls FailNow — the same
// runtime.Goexit unwind Fatalf performs on a real *testing.T.
func (r *recordingTB) Fatalf(format string, args ...any) {
	r.mu.Lock()
	r.messages = append(r.messages, fmt.Sprintf(format, args...))
	r.mu.Unlock()
	r.FailNow()
}

// output joins every captured message with a newline.
func (r *recordingTB) output() string {
	r.mu.Lock()
	defer r.mu.Unlock()
	return strings.Join(r.messages, "\n")
}

// captureFailure runs fn against a recordingTB on its own goroutine and
// returns everything fn reported. FailNow is defined to call runtime.Goexit,
// which terminates the calling goroutine, so fn must run on a goroutine of
// its own rather than inline, or a Fatalf inside fn would abandon t itself.
// captureFailure fails t when fn did NOT fail the recorder, which is what
// makes a red test a proof rather than a hope.
func captureFailure(t *testing.T, fn func(testing.TB)) string {
	t.Helper()

	rec := &recordingTB{TB: t}
	done := make(chan struct{})
	go func() {
		defer close(done)
		fn(rec)
	}()
	<-done

	require.True(t, rec.failed, "the helper did not fail; a red test that never goes red proves nothing")
	return rec.output()
}
