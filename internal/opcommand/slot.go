package opcommand

// Slot holds the value of one validating method and the error that validating
// it produced. A method stores both with Set, so a second call replaces the
// first call's value and its error alike (agent-docs/command-api-design.md
// §5, rule M2). The command's gate reads the slot at the position today's
// order puts that method's check (§4, rule A6).
//
// The zero Slot is unset.
type Slot[T any] struct {
	value T
	err   error
	set   bool
}

// Set stores value and the error its validation returned, replacing any
// earlier call. A nil err means the value is valid.
func (s *Slot[T]) Set(value T, err error) {
	s.value, s.err, s.set = value, err, true
}

// IsSet reports whether Set was called.
func (s *Slot[T]) IsSet() bool {
	return s.set
}

// Get returns the stored value and the error of the latest Set. An unset Slot
// returns the zero value and a nil error.
func (s *Slot[T]) Get() (T, error) {
	return s.value, s.err
}
