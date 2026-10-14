package topo

// Loop is the data of the root Loop handle, which holds one by
// value.
type Loop struct {
	// Outer reports whether this is the face's outer boundary; false for a hole.
	Outer bool
}
