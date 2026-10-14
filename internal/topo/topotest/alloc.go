// Package topotest provides a node allocator for internal unit tests.
package topotest

import "github.com/lestrrat-3d/decad/internal/topo"

// Alloc returns bare nodes with no Owner. A node it built reaching root is an
// invariant failure, and root's mapping panics on it.
type Alloc struct{}

// NewBody returns a bare body node.
func (Alloc) NewBody() *topo.Body { return &topo.Body{} }

// NewLump returns a bare lump node.
func (Alloc) NewLump() *topo.Lump { return &topo.Lump{} }

// NewShell returns a bare shell node.
func (Alloc) NewShell() *topo.Shell { return &topo.Shell{} }

// NewFace returns a bare face node.
func (Alloc) NewFace() *topo.Face { return &topo.Face{} }

// NewLoop returns a bare loop node.
func (Alloc) NewLoop() *topo.Loop { return &topo.Loop{} }

// NewEdge returns a bare edge node.
func (Alloc) NewEdge() *topo.Edge { return &topo.Edge{} }

// NewVertex returns a bare vertex node.
func (Alloc) NewVertex() *topo.Vertex { return &topo.Vertex{} }
