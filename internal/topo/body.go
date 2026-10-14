package topo

import (
	"sync"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/planarsnapshot"
	"github.com/lestrrat-3d/decad/internal/provenance"
)

// BodyKind states what a body IS; its values equal root's BodyKind constants.
type BodyKind int

const (
	// BodySolid is a body that encloses a material region.
	BodySolid BodyKind = iota
	// BodySheet is a body that encloses no material region.
	BodySheet
)

// Body is the data of the root Body handle, which holds one by
// value.
type Body struct {
	// Doc is the document node that owns (or owned) this body.
	Doc *Document
	// Origin is the feature role that created this body.
	Origin provenance.Ref
	// Lumps are the body's disjoint pieces.
	Lumps []*Lump
	// Volume, Area, Centroid and Bounds are the measurements the builder
	// computed; a body never recomputes them.
	Volume   measurement.Measurement
	Area     measurement.Measurement
	Centroid measurement.VecMeasurement
	Bounds   measurement.Box
	Kind     BodyKind
	// Payload is the evaluator's own record of how this body was built.
	Payload Payload
	// Owner is the root handle that holds this node (decad.Body); only root
	// sets it.
	Owner any
	Solid bool
	// PlanarConvexity caches the exact planar convexity certificate and
	// planarSnapshot the exact held snapshot it is read off, each keyed by
	// the held chord when the snapshot depends on it
	// (contact_faceted_pair.go); like the tessellation cache neither changes
	// the body's logical geometry.
	PlanarConvexity atomic.Pointer[planarsnapshot.ConvexityEntry]
	PlanarSnapshot  atomic.Pointer[planarsnapshot.SnapshotEntry]
	// PlanarHints maps each partner body to the nearest candidate pair the
	// last exact planar relation with this body first found
	// (contact_faceted_pair.go). It only speeds the next relation's
	// distance scan, never changes its result.
	PlanarHints sync.Map
}

// Faces returns the body's faces across all lumps and shells, in order. It
// returns nil for a body with none.
func (b *Body) Faces() []*Face {
	var out []*Face
	for _, l := range b.Lumps {
		for _, s := range l.Shells {
			out = append(out, s.Faces...)
		}
	}
	return out
}

// Edges returns the body's edges in face and loop order, deduplicated. It
// returns nil for a body with none.
func (b *Body) Edges() []*Edge {
	seen := map[*Edge]struct{}{}
	var out []*Edge
	for _, f := range b.Faces() {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				if _, ok := seen[ce.Edge]; ok {
					continue
				}
				seen[ce.Edge] = struct{}{}
				out = append(out, ce.Edge)
			}
		}
	}
	return out
}

// Vertices returns the body's vertices in edge order, deduplicated. It
// returns nil for a body with none.
func (b *Body) Vertices() []*Vertex {
	seen := map[*Vertex]struct{}{}
	var out []*Vertex
	for _, e := range b.Edges() {
		for _, v := range [2]*Vertex{e.Start, e.End} {
			if v == nil {
				continue
			}
			if _, ok := seen[v]; ok {
				continue
			}
			seen[v] = struct{}{}
			out = append(out, v)
		}
	}
	return out
}
