package topo

// Loop is the data of the root Loop handle, which holds one by
// value.
type Loop struct {
	// Outer reports whether this is the face's outer boundary; false for a hole.
	Outer bool
	// CoEdges are the loop's directed edge uses in walk order. Root
	// Loop.CoEdges wraps each in a public CoEdge value.
	CoEdges []CoEdge
	// Owner is the root Loop handle that holds this node by value. Root
	// sets it when the handle is born and maps a node back to its handle
	// through it; nothing else writes it.
	Owner any
}
