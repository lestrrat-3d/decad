package prismcells

// WorkLimitError keeps the existing refusal and the arrangement's measured
// segment count together for callers that expose structured diagnostics.
type WorkLimitError struct {
	Observed int
	Maximum  int
	err      error
}

func (e *WorkLimitError) Error() string { return e.err.Error() }
func (e *WorkLimitError) Unwrap() error { return e.err }

// WithWorkLimit marks an existing arrangement-cap refusal with its measured
// resource counts while preserving its text and error chain.
func WithWorkLimit(err error, observed int) error {
	return &WorkLimitError{Observed: observed, Maximum: MaxArrangementSegments, err: err}
}
