// Package facetedrecord holds the faceted body's source groups and the
// builder that assembles a Faceted body over internal/topo nodes from a held
// mesh (docs/evaluator-design.md §9), with the placement helpers the faceted
// payload reads. Root facetGroup aliases Group.
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
	Denoted *surfacenormal.Revolved
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
