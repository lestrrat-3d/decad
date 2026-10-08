package tessellation

// ExpectedError marks a valid operand whose requested chording cannot prove
// its topology. The public error still unwraps to ErrDegenerate.
type ExpectedError struct{ err error }

// NewExpectedError wraps a chording refusal without changing its public cause.
func NewExpectedError(err error) *ExpectedError { return &ExpectedError{err: err} }

func (e *ExpectedError) Error() string { return e.err.Error() }
func (e *ExpectedError) Unwrap() error { return e.err }
