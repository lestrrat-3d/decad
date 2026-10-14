package topo

import (
	"sync"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/planarsnapshot"
)

// Body is the data of the root Body handle, which holds one by
// value. The handle keeps the document link, origin, lumps, measurements, kind, payload and the
// other caches, which name root types.
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
}
