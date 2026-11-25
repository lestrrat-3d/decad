package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/opcommand"
)

// runCommand is the shared Do of every operation command
// (agent-docs/command-api-design.md §3). It consumes the command, refuses a
// nil ctx, returns a fault a constructor or method recorded, and only then
// calls run, which holds the operation's own gate order, build and commit.
// Every error except cancellation leaves tagged with kind.
//
// It is a root adapter because the tags it applies are root-owned: it calls
// operationError for the OperationKind and OperationError, and names a
// recorded subject in OperationSubject. The consumed flag and the recorded
// fault live in internal/opcommand.
func runCommand[T any](ctx context.Context, s *opcommand.State, kind OperationKind,
	run func(context.Context) (T, error)) (T, error) {
	var zero T
	if !s.Consume() {
		return zero, operationError(kind, fmt.Errorf(`%w: build a new %s command to run it again`, ErrConsumed, kind))
	}
	if ctx == nil {
		return zero, operationError(kind, fmt.Errorf(`%w: a nil context cannot control a %s command`, ErrDegenerate, kind))
	}
	if subject, err := s.Fault(); err != nil {
		return zero, subjectError(kind, subject, err)
	}
	out, err := run(ctx)
	if err != nil {
		return out, operationError(kind, err)
	}
	return out, nil
}

// subjectError tags err with kind and names subject — the constructor
// argument or method that recorded err — in each of its diagnostics that
// names no subject yet.
func subjectError(kind OperationKind, subject string, err error) error {
	tagged := operationError(kind, err)
	opErr, ok := tagged.(*OperationError)
	if !ok || subject == "" {
		return tagged
	}
	diagnostics := opErr.OperationDiagnostics()
	for i := range diagnostics {
		if diagnostics[i].Subject.Name == "" {
			diagnostics[i].Subject.Name = subject
		}
	}
	return &OperationError{err: err, diagnostics: diagnostics}
}
