package topo

// Loop is the data of the root Loop handle, which holds one by
// value.
type Loop struct {
	// Outer reports whether this is the face's outer boundary; false for a hole.
	Outer bool
	// CoEdges are the loop's directed edge uses in walk order. Root
	// Loop.CoEdges wraps each in a public CoEdge value.
	CoEdges []CoEdge
}
