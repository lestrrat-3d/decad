package prismcells

import "errors"

// WorkLimitError keeps the existing refusal and the arrangement's measured
// segment count together for callers that expose structured diagnostics.
type WorkLimitError struct {
	Observed int
	Maximum  int
	err      error
}

func (e *WorkLimitError) Error() string { return e.err.Error() }
func (e *WorkLimitError) Unwrap() error { return e.err }

// IsWorkLimit reports whether err is an arrangement-cap refusal. A caller
// whose own over-cap answer is a silent miss maps it to that miss
// (agent-docs/carrier-record-design.md §4).
func IsWorkLimit(err error) bool {
	var limit *WorkLimitError
	return errors.As(err, &limit)
}

// WithWorkLimit marks an existing arrangement-cap refusal with its measured
// resource counts while preserving its text and error chain.
func WithWorkLimit(err error, observed int) error {
	return &WorkLimitError{Observed: observed, Maximum: MaxArrangementSegments, err: err}
}
