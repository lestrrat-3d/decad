// Package opcommand holds the per-command state every public operation
// command embeds: the consumed-once flag and the first fault a constructor or
// method recorded (agent-docs/command-api-design.md §3).
//
// The package knows no decad type. The root package wraps what it returns
// with the command's operation kind and turns a recorded subject into an
// OperationSubject.
package opcommand

import "sync/atomic"

// State is the consumed-once flag and the first recorded fault of one
// command. Its zero value is a fresh command with no fault.
//
// A State is not safe for concurrent use, with one exception: Consume may race
// with itself, and exactly one racing caller wins.
type State struct {
	consumed atomic.Bool
	subject  string
	fault    error
}

// Record stores err as the command's fault, naming subject as the argument or
// method that caused it. Only the first non-nil fault is kept; a later call
// and a nil err change nothing.
func (s *State) Record(subject string, err error) {
	if err == nil || s.fault != nil {
		return
	}
	s.subject, s.fault = subject, err
}

// Fault returns the first recorded fault and its subject, or a nil error when
// nothing was recorded.
func (s *State) Fault() (string, error) {
	return s.subject, s.fault
}

// Consume marks the command consumed. It reports true to the one caller that
// changed the flag and false to every caller after it, including callers that
// race the winner.
func (s *State) Consume() bool {
	return s.consumed.CompareAndSwap(false, true)
}
