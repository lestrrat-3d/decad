package proofbound

// Moved from the root package's segment_walk.go.

// WalkEndBound is the proven error bound on a walk endpoint's two components,
// stated PER COMPONENT and never merged into one number. The two are
// independent readings and an endpoint routinely proves one exactly while the
// other carries error: a whole circle's own end is exactly that shape, since
// math.Cos returns 1 at the end angle while math.Sin does not return 0 there.
// Merging them would spend the exact axis's zero on the other axis's error, and
// a reading along the exact axis alone would then publish a width its own
// arithmetic never committed.
//
// A component the recorded data cannot enclose reads +Inf, the underivable
// bound every consumer refuses on, and never zero.
type WalkEndBound struct {
	U, V float64
}

// derivable reports whether both components state a bound at all.
func (b WalkEndBound) Derivable() bool { return !IsNonFinite(b.U) && !IsNonFinite(b.V) }
