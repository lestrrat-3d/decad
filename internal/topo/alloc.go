package topo

// Alloc allocates topology nodes for a builder. Root implements it: each node
// is born inside its root handle, which sets the node's Owner, so a node
// pointer a builder stores stays the handle's own node for the handle's whole
// life. Internal unit tests use topotest.Alloc, whose nodes have no Owner.
type Alloc interface {
	NewBody() *Body
	NewLump() *Lump
	NewFaceShell() *FaceShell
	NewFace() *Face
	NewLoop() *Loop
	NewEdge() *Edge
	NewVertex() *Vertex
}
