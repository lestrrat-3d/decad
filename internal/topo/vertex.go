package topo

import (
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Vertex is the data of the root Vertex handle, which holds one by
// value.
type Vertex struct {
	// Position is the vertex position in millimetres.
	Position r3.Vec
	// Bound is the proven displacement of Position.
	Bound units.Value
	// Level is the LEVEL half of the shared-denotation certificate
	// (root denotation.go): non-zero only for a vertex a builder stamped at one
	// denoted sweep level, and zero ("no certificate") for every vertex no
	// builder in this package mints one for.
	Level denotation.LevelToken
	// Denot is the CURVE half of the shared-denotation certificate
	// (root denotation.go): non-zero only for a vertex a builder minted one for —
	// today a straight prism's rim corner (internal/prismrecord/build.go) and a revolve's
	// own internal junction vertex (revolve_build.go; a partial revolve's
	// own SEAM — its boundary copy at phi0/phi1 — mints none) — and
	// propagated unchanged,
	// composing xform, by every copier that reproduces the same point
	// (stitch.go's rebuildStitchTopology, unstitch.go's
	// copyFaceUnderContext, patch_body.go's copyPatchFacesUnder). Zero ("no
	// certificate") for every vertex no builder or copier stamps.
	Denot denotation.CurveToken
	// Owner is the root Vertex handle that holds this node by value. Root
	// sets it when the handle is born and maps a node back to its handle
	// through it; nothing else writes it.
	Owner any
}
