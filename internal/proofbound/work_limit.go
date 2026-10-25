package proofbound

import "errors"

// WorkLimitError preserves an existing refusal together with the exact counts
// known at its resource gate. Observed counts the requested or visited work,
// not an estimate of additional work an unfinished proof might need.
type WorkLimitError struct {
	Resource string
	Observed uint64
	Maximum  uint64
	err      error
}

func (e *WorkLimitError) Error() string { return e.err.Error() }
func (e *WorkLimitError) Unwrap() error { return e.err }

// WithWorkLimit attaches known resource counts without changing the refusal's
// text or error chain. Callers must not pass overflowed or inferred counts.
func WithWorkLimit(err error, resource string, observed, maximum uint64) error {
	return &WorkLimitError{Resource: resource, Observed: observed, Maximum: maximum, err: err}
}

// PreserveWorkLimit carries source's known counts through an existing error
// translation. The translated error keeps its own text and error chain.
func PreserveWorkLimit(translated, source error) error {
	var detail *WorkLimitError
	if !errors.As(source, &detail) {
		return translated
	}
	return WithWorkLimit(translated, detail.Resource, detail.Observed, detail.Maximum)
}
