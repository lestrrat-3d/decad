package topo

// Lump is the data of the root Lump handle, which holds one by
// value.
type Lump struct {
	// Shells are the nodes of the lump's shells, the outer shell first. Root
	// Lump.Shells maps them back to their handles.
	Shells []*FaceShell
	// Owner is the root Lump handle that holds this node by value. Root sets
	// it when the handle is born and maps a node back to its handle through
	// it; nothing else writes it.
	Owner any
}
