package topo

// Shell is the data of the root Shell handle, which holds one by
// value.
type Shell struct {
	// Faces are the shell's faces.
	Faces []*Face
	// Owner is the root handle that holds this node (decad.Shell); only root
	// sets it.
	Owner any
	Void  bool
	Open  bool
}

// Lump is the data of the root Lump handle, which holds one by value.
type Lump struct {
	// Shells are the lump's shells, the outer shell first.
	Shells []*Shell
	// Owner is the root handle that holds this node (decad.Lump); only root
	// sets it.
	Owner any
}
