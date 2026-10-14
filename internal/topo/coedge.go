package topo

// CoEdge is the data of one directed use of an edge by a loop: the root
// CoEdge value holds one, and a Loop node holds its uses by value in walk
// order. It has no Owner: nothing links to a use, so nothing maps one back.
type CoEdge struct {
	// Edge is the node of the edge this use traverses. Root CoEdge.Edge maps
	// it back to its handle.
	Edge *Edge
	// Forward reports whether the use walks from Edge.Start to Edge.End;
	// false means it walks from Edge.End to Edge.Start.
	Forward bool
}
