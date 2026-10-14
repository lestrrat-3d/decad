package topo

import (
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// Edge is the data of the root Edge handle, which holds one by
// value.
type Edge struct {
	// A source-proven straight generator can have a faceted held curve.
	// ParallelTo reads this direction without changing Curve or its bound.
	SelectorDirection r3.Vec
	// SelectorDirectionProven reports whether SelectorDirection is source-proven.
	SelectorDirectionProven bool
	// Convex is the walked-boundary convexity — see IsConvex.
	Convex bool
	// Length is the held length in millimetres for the analytic curve set.
	Length float64
	// LengthBound is the proven error bound on Length: zero only when the
	// analytic curve's held result is exact, otherwise its evaluation bound;
	// boolean-built chains carry their proven chord bound.
	LengthBound float64
	// LengthUnbounded marks an edge whose true length this evaluator cannot
	// bound, so Length refuses rather than understate: a boolean rim on a
	// curved source, whose chord chain understates the true curve's length
	// by an amount no chord bound can cover, or a cap-blend miter ruling
	// adjacent to a circular wall whose corner-foot locus this evaluator
	// cannot enclose (docs/modify-reach-design.md §8.3).
	LengthUnbounded bool
	// CurveBound, valid only where CurveBounded is set, bounds how far the
	// curve a Circle3 or Arc3 edge denotes lies from its held circle — the
	// circle of the held Radius about the held Center, normal to the held
	// Axis: every denoted point is within CurveBound of that circle, and
	// CurveBound is below half the held radius, so projecting radially about
	// the held centre carries the denoted curve continuously onto the held
	// circle. It says nothing about where along the circle the curve ends;
	// the end vertices' own bounds say that. A builder that proves it sets
	// both (prismPayload.CircleCurveBound); every copier carries it through
	// its placement (surfaceplace.PlacedCurveBound). An edge no builder bounds
	// leaves CurveBounded false, and a reading that needs the bound refuses.
	CurveBound   float64
	CurveBounded bool
	// Level is the LEVEL half of the shared-denotation certificate
	// (root denotation.go): non-zero only for a rim edge a builder stamped at one
	// denoted sweep level, and zero ("no certificate") for every edge no
	// builder in this package mints one for.
	Level denotation.LevelToken
	// Denot is the CURVE half of the shared-denotation certificate
	// (root denotation.go): non-zero only for an edge a builder minted one for —
	// today a straight prism's rim edge (root prism_build.go) and a revolve's
	// junction edge (root revolve_build.go) — and propagated unchanged, composing
	// xform, by every copier that reproduces the same curve (root stitch.go's
	// rebuildStitchTopology, root unstitch.go's copyFaceUnderContext,
	// root patch_body.go's copyPatchFacesUnder). It is what root stitch_weld.go's
	// buildStitchWeldPlan reads to lift Table J's J5 for a bounded pair. Zero
	// ("no certificate") for every edge no builder or copier stamps.
	Denot denotation.CurveToken
	// Curve is the edge's tagged geometry in its internal variant. Root
	// Edge.Curve converts it to the public variant with publicCurve.
	Curve surfacegeom.Curve
	// Start and End are the edge's end vertices. Root Edge.Start and
	// Edge.End map them back to their handles.
	Start *Vertex
	End   *Vertex
	// Faces are the nodes of the faces adjacent to the edge, in the order
	// they were linked. Root Edge.Faces maps them back to their handles.
	Faces []*Face
}
