package topo

import (
	"sync"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/planarsnapshot"
	"github.com/lestrrat-3d/decad/internal/provenance"
)

// Body is the data of the root Body handle, which holds one by
// value. The handle keeps the payload and the other caches, which name root
// types.
type Body struct {
	// Solid reports whether the body is a valid solid, a decided answer
	// (docs/api-design.md core §6).
	Solid bool
	// PlanarConvexity caches the exact planar convexity certificate and
	// PlanarSnapshot the exact held snapshot it is read off, each keyed by
	// the held chord when the snapshot depends on it
	// (root contact_faceted_pair.go); like the tessellation cache neither
	// changes the body's logical geometry.
	PlanarConvexity atomic.Pointer[planarsnapshot.ConvexityEntry]
	PlanarSnapshot  atomic.Pointer[planarsnapshot.SnapshotEntry]
	// PlanarHints maps each partner body to the nearest candidate pair the
	// last exact planar relation with this body first found
	// (root contact_faceted_pair.go). It only speeds the next relation's
	// distance scan, never changes its result.
	PlanarHints sync.Map

	// Origin is the producing feature's role. Root Body.Origin converts it to
	// the public FeatureRef form.
	Origin provenance.Ref
	// The evaluator computes measurements at build time; a Body never
	// recomputes (immutability is what makes it goroutine-safe to read).
	// Volume is the body's enclosed volume. Root Body.Volume converts it to
	// the public Measurement form.
	Volume measurement.Measurement
	// Area is the body's total surface area. Root Body.Area converts it to
	// the public Measurement form.
	Area measurement.Measurement
	// Centroid is the body's volume centroid. Root Body.Centroid converts it
	// to the public VecMeasurement form.
	Centroid measurement.VecMeasurement
	// Bounds is the body's axis-aligned bounding box. Root Body.Bounds
	// converts it to the public Box form.
	Bounds measurement.Box
	// Kind says whether the body is a solid or a sheet. Root Body.Kind
	// converts it to the public BodyKind form.
	Kind BodyKind
	// Doc is the node of the document that owns (or owned) the body, nil for
	// a body built outside any document. Root Body.Document maps it back to
	// its handle.
	Doc *Document
	// Lumps are the nodes of the body's disjoint pieces. Root Body.Lumps
	// maps them back to their handles.
	Lumps []*Lump
	// Owner is the root Body handle that holds this node by value. Root
	// sets it when the handle is born and maps a node back to its handle
	// through it; nothing else writes it.
	Owner any
}

// BodyKind is the root BodyKind's node form, with the same constant values:
// whether the body's boundary encloses a material region.
type BodyKind int

const (
	// BodySolid is root BodySolid.
	BodySolid BodyKind = iota
	// BodySheet is root BodySheet.
	BodySheet
)
