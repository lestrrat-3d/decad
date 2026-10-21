package loftmesh

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// AssembleTriangulated lifts every recorded point once, emits the 2*sum(n_i) wall
// triangles in Table B's order and winding, triangulates both caps through
// internal/triangulation's polygon-with-holes triangulator with capStart's
// triples reversed and capEnd's retained (§5's cap seeding), and orients the
// complete shell once from the signed tetrahedron sum anchored at the placed
// p0 origin (§5's whole-shell rule). It also owns Table S row S13: every
// placed coordinate it emits, the anchor among them, is proven finite before
// any of them is lifted into an exact dyadic.
//
// stationRound is loftmesh.PairRecords' own accumulated Table S row S14 term
// (a10-plan.md Part 3 PR 6): the proven rounding every COMPUTED circular
// station commits, composed into delta beside the placement's own
// proofbound.RigidRoundAllow term.
func AssembleTriangulated(ctx context.Context, pairs []LoopPair, f0, f1 r3.Frame, plane0 sectionrecord.PlaneRecord, xform r3.Transform, stationRound float64) (Assembly, error) {
	triangulate := func(ctx context.Context, pts []sectionrecord.Point2, loops [][]int) ([][3]int, error) {
		tris, err := triangulation.Triangulate(ctx, pts, loops)
		return tris, triangulation.WrapLoftError(err)
	}
	return Assemble(ctx, pairs, f0, f1, plane0, xform, stationRound,
		triangulate, ErrPointUnrepresentable)
}

// ErrPointUnrepresentable is agent-docs/loft-design.md Table S row S13: a
// coordinate this build emits — a recorded section point lifted through its
// own frame and carried by the composed placement, or the orientation anchor
// — runs past the representable float64 range.
//
// The sentinel is ErrUnsupported and never ErrNotFinite. Every INPUT is
// finite: both records' coordinates cleared the seam gates, the plane origins
// are recorded floats, and r3 validates a Transform's own composed
// translation before it ever reaches this evaluator. What runs off float64 is
// decad's OWN evaluation of the lift, and the body EXISTS — it is the rigid
// image of a body this evaluator already built — so modify §1's existence
// test reads "a body this evaluator cannot build". internal/freeform/spline_length.go's R15 and
// spline_fit.go's R16 draw the identical line for a finite input whose
// derived magnitude runs off float64; errors.go scopes ErrNotFinite to a
// non-finite PARAMETER or a derived non-finite MEASUREMENT, and
// topobuild.ValidateLoftBodyMeasurements already owns that second case.
//
// The gate runs BEFORE the first exact-dyadic lift, never after it:
// tessellation.OrientationSign lifts the anchor and every vertex through dyVec, whose
// mustDyOf PANICS on a non-finite float, so a check placed any later is a
// panic out of a public method rather than a returned error.
func ErrPointUnrepresentable(what string) error {
	return fmt.Errorf(`%w: the loft's %s runs past the representable float64 range`, decaderr.ErrUnsupported, what)
}
