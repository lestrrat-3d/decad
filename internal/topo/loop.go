package topo

// CoEdge is one directed use of an Edge by a Loop. It is a value: root's
// CoEdge wraps one per element and needs no Owner.
type CoEdge struct {
	Edge    *Edge
	Forward bool
}

// Loop is the data of the root Loop handle, which holds one by
// value.
type Loop struct {
	// CoEdges are the loop's directed edge uses in walk order.
	CoEdges []CoEdge
	// Owner is the root handle that holds this node (decad.Loop); only root
	// sets it.
	Owner any
	Outer bool
}
