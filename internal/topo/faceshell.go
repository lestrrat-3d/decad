package topo

// FaceShell is the data of the root FaceShell handle, which holds one by
// value.
type FaceShell struct {
	// Void reports whether the shell bounds an internal cavity.
	Void bool
	// Open reports whether the shell has at least one free edge. Void and Open
	// are independent (docs/surface-design.md §2.2).
	Open bool
}
