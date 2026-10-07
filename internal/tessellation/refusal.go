package tessellation

import "fmt"

// Sentinel names the root package error an audit refusal maps onto.
type Sentinel uint8

const (
	// Degenerate is the root's ErrDegenerate: the input contradicts itself.
	Degenerate Sentinel = iota + 1
	// Unsupported is the root's ErrUnsupported: this evaluator cannot state the result.
	Unsupported
)

// AuditError is a mesh audit's own refusal. Detail is the text the root
// package writes after the sentinel it selects, so the published message is
// the audit's own, word for word.
type AuditError struct {
	Sentinel Sentinel
	Detail   string
}

func (e *AuditError) Error() string { return e.Detail }

func degenerate(format string, args ...any) error {
	return &AuditError{Sentinel: Degenerate, Detail: fmt.Sprintf(format, args...)}
}

// DegenerateError reports a contradiction in a mesh input.
func DegenerateError(format string, args ...any) error { return degenerate(format, args...) }

func unsupported(format string, args ...any) error {
	return &AuditError{Sentinel: Unsupported, Detail: fmt.Sprintf(format, args...)}
}

// UnsupportedError reports a mesh proof the evaluator cannot state.
func UnsupportedError(format string, args ...any) error { return unsupported(format, args...) }
