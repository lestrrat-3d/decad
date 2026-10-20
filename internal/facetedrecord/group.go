// Package facetedrecord holds the faceted payload (Payload): the record of a
// mesh-built body, its source groups, the source records and support types
// that structural rewrites and copies read, and the builder that assembles a
// Faceted body over internal/topo nodes from a held mesh, with its placement
// (docs/evaluator-design.md §8, §9). Root facetedPayload, facetGroup and the
// record names alias these types.
//
// BuildWithProof turns a stitched boolean mesh into a Faceted Body
// (docs/evaluator-design.md §9, core §6.1): facets grouped into one Face per
// CONNECTED PATCH of a source analytic face — the source's origins ride onto
// every patch of it, so provenance survives the boolean, while a source the
// boolean cut into disconnected pieces reports each piece as the separate face
// it now is — real Face/Loop/Edge/Vertex topology chained along the face
// boundaries, shells and voids decided by exact signed volume and exact
// containment parity, and measurements integrated exactly over the held mesh,
// reported Approximate with the proven composed bounds. An ordinary Boolean
// face is Faceted; a structural source with its own surface proof can retain
// that analytic tag while the polygons remain the held boundary.
package facetedrecord

import (
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
)

// Group is one source face's provenance, carried into the payload so a
// rebuild (Transform) reproduces the same origins.
type Group struct {
	Origins []provenance.Ref
	// Centroid is available only when this entire source group has a separate
	// certificate. A disconnected group cannot give that reading to a patch.
	Centroid    measurement.VecMeasurement
	HasCentroid bool
	// Surface is set by a source whose authenticated surface is retained,
	// including structural trims and the smooth three-section loft. Ordinary
	// mesh booleans leave it nil.
	Surface surfacegeom.Surface
	// Retained marks a Surface that the generic mesh Boolean copied from its
	// operand face (SourceTag), as opposed to one a structural builder
	// authenticated. A retained curved tag is dropped by a similarity scale
	// instead of refusing it, and AssembleBody derives the face's normal
	// bound from NormalBound (the tag's own departure) plus the held
	// displacement.
	Retained bool
	Denoted  *surfacenormal.Revolved
	// NormalBound covers the tag's normal departure from its denoted patch.
	// A placed planar tag conservatively uses the maximum distance of two
	// unit normals when its rounded frame has no tighter certificate.
	NormalBound float64
	// Reversed records a retained analytic surface whose natural normal
	// points into the result, as on the cone exposed by a Cut.
	Reversed bool
	// Planar records whether the source analytic face is a plane: the rim
	// between two planar sources is a straight line, whose chord length is
	// exact; any curved source makes the rim's true length unboundable
	// without curvature knowledge, and Edge.Length must refuse.
	Planar bool
}
