package trimgraph

import (
	"errors"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// ErrNotLiftable marks a receiver outside the graph's admitted vocabulary. Callers decline on it.
var ErrNotLiftable = errors.New("trimgraph: the receiver is outside the trim graph")

// Graph is the lifted receiver. Faces[i] is the i-th face of topobuild.Faces(b), so fp.FaceOf indexes it.
type Graph struct {
	Faces    []Face
	Edges    []Edge
	Vertices []Vertex
}

// Face is one receiver face on its exact tag.
type Face struct {
	Node     *topo.Face
	Group    int                 // the fp.Groups index its facets carry (fp.Src)
	Tag      surfacegeom.Surface // Plane in increment 1
	Reversed bool
	Loops    [][]Use // outer loop first, each in its topo walk order
}

// Use is one directed use of a graph edge in a loop.
type Use struct {
	Edge    int
	Forward bool
}

// Edge is one receiver edge between two distinct faces.
type Edge struct {
	Node       *topo.Edge
	Faces      [2]int // Faces[0] walks it Forward
	Start, End int    // vertex indices
	Convex     bool   // decided from the tags (L6), never from held facets
	Curve      CurveKind
}

// CurveKind names the geometry an edge lies on.
type CurveKind int

const (
	// CurveLine is the line of two plane tags; later increments add kinds.
	CurveLine CurveKind = iota
)

// Vertex is one receiver vertex of valence 3 or more.
type Vertex struct {
	Node  *topo.Vertex
	Pos   r3.Vec  // held position
	Bound float64 // held bound, mm
	Edges []int   // incident graph edges
	// Faces are the incident faces in cyclic order around the vertex: each face is followed by the other face of
	// the edge its outer loop arrives at the vertex by.
	Faces []int
}
