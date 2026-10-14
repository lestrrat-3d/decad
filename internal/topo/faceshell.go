package topo

// FaceShell is the data of the root FaceShell handle, which holds one by
// value.
type FaceShell struct {
	// Void reports whether the shell bounds an internal cavity.
	Void bool
	// Open reports whether the shell has at least one free edge. Void and Open
	// are independent (docs/surface-design.md §2.2).
	Open bool
	// Faces are the nodes of the shell's faces. Root FaceShell.Faces maps
	// them back to their handles.
	Faces []*Face
	// Owner is the root FaceShell handle that holds this node by value. Root
	// sets it when the handle is born and maps a node back to its handle
	// through it; nothing else writes it.
	Owner any
}
