package prismcells

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// OverlapVolume is docs/prism-boolean-design.md §4.5's overlap-area reading:
// a volume, never a body. Verify's interference path needs a proven overlap
// volume and the proven-overlap verdict for a coplanar, co-directional prism
// pair whose 2D outlines overlap in two or more disjoint regions — a pair of
// meshing gears, which always engages several teeth at once — and a
// ProfileRecord carries a single Outer loop, so no such selection can ever
// become one assembled section (§4.4). This reading answers the volume
// without assembling anything: it measures each arrangement cell both
// operands claim through the prism record builder, unchanged, and returns the
// charged sum of the cells' own volumes and its bound.
//
// It runs only from Verify's read-only interference path, never from a public
// boolean: a public boolean owes its caller a Body, and this reading builds
// none. §13 records that asymmetry as a deliberate decision. newEnv mints the
// build environment of one cell's scratch body, taking a fresh producer
// identity each time.
//
// ok=false always carries err=nil, matching the analytic Try's own contract:
// it means "not admitted" or "answers nothing" — every gate miss, an
// unresolved classification, an empty selection (the exactly-tangent case),
// and a selected cell whose own section the region integrals refuse as
// degenerate or unsupported all decline silently, and the caller falls back
// to the mesh path unchanged. A non-nil err is a genuine refusal past §3.4's
// point of no return: the arrangement work cap (RB7, wrapped so
// measuredInterference reads it exactly as booleandispatch.AnalyticIntersect's own
// RB7 wrapping), a recordEdge rejection or a non-closing cell loop
// (RB8/RB9) on cuts that carry no amplified displacement, or the caller's
// own cancellation. A crossing too close to tangent for A6's charge declines
// silently.
func OverlapVolume(ctx context.Context, a, b topo.Payload, newEnv func() topo.Env) (float64, float64, bool, error) {
	// Entry (§4.5's own "Entry" paragraph): Intersect's own preamble, shared
	// and unchanged via Task 1's extracted helper — G1-G4, the
	// trimmed-circular refusal, G6's hole-free arms, G5's Intersect
	// z-relation, the arrangement cap, and the re-expression.
	budget, pa, pb, reexpress, ok, err := AdmitIntersectPair(ctx, a, b)
	if err != nil {
		if errors.Is(err, decaderr.ErrUnsupported) {
			// RB7 (§9): the arrangement cap. Wrapped exactly as
			// booleandispatch.AnalyticIntersect wraps its own RB7, so
			// measuredInterference reads it as reportvocab.InterferenceUnsupportedPipeline
			// and the report says Suspect.
			return 0, 0, false, meshbool.ExpectedBoolean(meshbool.BooleanExpectedUnsupported, err)
		}
		return 0, 0, false, err
	}
	if !ok {
		return 0, 0, false, nil
	}

	// Selected cells: Task 1's first half of the crossing resolution, under
	// Intersect's own keep — the arrangement's own cells that Classify puts on
	// BOTH operands' material sides.
	selected, sceneDelta, resolved, err := ResolveCrossingCells(ctx, budget, pa.Profile, pb.Profile,
		pa.SectionDelta, pb.SectionDelta, reexpress,
		func(a, b bool) bool { return a && b })
	if err != nil {
		return 0, 0, false, err
	}
	if !resolved {
		return 0, 0, false, nil
	}
	if len(selected) == 0 {
		// The exactly-tangent case: the arrangement puts no cell on both
		// operands' material sides. §4.5 requires this to decline rather
		// than publish a zero-volume answer.
		return 0, 0, false, nil
	}

	// §3.2's Intersect z-interval and per-end axial displacement, shared by
	// every measured cell — the pair's own relation, not a per-cell one.
	pbZ0, pbZ1 := prismplacement.AdmittedShiftedInterval(pa.PlacementOperand(), pb.PlacementOperand())
	z0, z0Delta := prismplacement.IntersectLowerEnd(pa.Z0, pa.Z0Delta, pbZ0, pb.Z0Delta)
	z1, z1Delta := prismplacement.IntersectUpperEnd(pa.Z1, pa.Z1Delta, pbZ1, pb.Z1Delta)

	values := make([]float64, 0, len(selected))
	boundTerms := make([]float64, 0, len(selected))
	for _, p := range selected {
		if err := budget.Step(); err != nil {
			return 0, 0, false, err
		}

		// Per-cell measurement: the cell's own closed directed walk,
		// recorded through the existing recordEdge/edgeJoin/CutDelta sequence
		// Merge already uses — no count, drop or chain, since a cell is
		// already one loop in sketch's own order. RB8/RB9 propagate here,
		// exactly as they do on the body path, unless the cuts carry an
		// amplified displacement (AmplifiedFallback), which sends the pair to
		// the mesh path.
		cellProfile, cutDelta, err := RecordOverlapCell(budget, p.Outer)
		if fallBack, err := AmplifiedFallback(sceneDelta.Amplified, err); fallBack || err != nil {
			return 0, 0, false, err
		}

		// §7's formula with A6's crossing term, byte-identical to
		// ResolveAndBuildIntersectCrossing's, taken over THIS cell's own
		// cutDelta.
		sectionDelta := sceneDelta.Merged(pa.SectionDelta, pb.SectionDelta, reexpress.Delta, cutDelta)
		pp := prismrecord.Payload{
			Profile:      cellProfile,
			Frame:        pa.Frame,
			Xform:        pa.Xform,
			Z0:           z0,
			Z1:           z1,
			Z0Delta:      z0Delta,
			Z1Delta:      z1Delta,
			SectionDelta: sectionDelta,
		}

		body, err := prismrecord.Build(ctx, newEnv(), pp, freeform.NewFreeformWork())
		if err != nil {
			if errors.Is(err, decaderr.ErrDegenerate) || errors.Is(err, decaderr.ErrUnsupported) {
				// The region integrals refuse this cell's own section as
				// degenerate or unsupported: §4.5 declines the WHOLE
				// reading rather than publish a partial sum over the cells
				// that did measure.
				return 0, 0, false, nil
			}
			return 0, 0, false, err
		}
		values = append(values, body.Volume.Value.Base())
		boundTerms = append(boundTerms, body.Volume.Bound.Base())
	}

	// The sum (§4.5's own "The sum" paragraph): the value is the cells'
	// float sum; the bound is proofbound.AbsSumUpper over the cells' own bounds and
	// that sum's own accumulated rounding (internal/proofbound/bounds.go's proofbound.ExactSumRound, the
	// mechanism's one owner — no new helper is added here).
	sum := 0.0
	for _, v := range values {
		sum += v
	}
	rounding := proofbound.ExactSumRound(sum, values...)
	bound := proofbound.AbsSumUpper(append(boundTerms, rounding)...)
	return sum, bound, true, nil
}
