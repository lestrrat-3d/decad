package decad

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// This file is docs/prism-boolean-design.md §4.5's overlap-area reading: a
// volume, never a body. Verify's interference path needs a proven overlap
// volume and the proven-overlap verdict for a coplanar, co-directional prism
// pair whose 2D outlines overlap in two or more disjoint regions — a pair of
// meshing gears, which always engages several teeth at once — and a
// ProfileRecord carries a single Outer loop, so no such selection can ever
// become one assembled section (§4.4). This reading answers the volume
// without assembling anything: it measures each arrangement cell both
// operands claim through evalPrism, unchanged, and publishes the charged sum
// of the cells' own volumes.
//
// prismOverlapVolume runs only from Verify's read-only interference path
// (interference.go's measuredInterference), never from performBoolean: a
// public boolean owes its caller a Body, and this reading builds none. §13
// records that asymmetry as a deliberate decision.
//
// ok=false always carries err=nil, matching tryPrismBoolean's own contract:
// it means "not admitted" or "answers nothing" — every gate miss, an
// unresolved classification, an empty selection (the exactly-tangent case),
// and a selected cell whose own section the region integrals refuse as
// degenerate or unsupported all decline silently, and the caller falls back
// to the mesh path unchanged. A non-nil err is a genuine refusal past §3.4's
// point of no return: the arrangement work cap (RB7, wrapped so
// measuredInterference reads it exactly as evaluateAnalyticIntersect's own
// RB7 wrapping), a recordEdge rejection or a non-closing cell loop (RB8/RB9),
// or the caller's own cancellation.
func prismOverlapVolume(ctx context.Context, a, b *Body) (Measurement, bool, error) {
	// Entry (§4.5's own "Entry" paragraph): Intersect's own preamble, shared
	// and unchanged via Task 1's extracted helper — G1-G4, the
	// trimmed-circular refusal, G6's hole-free arms, G5's Intersect
	// z-relation, the arrangement cap, and the re-expression.
	budget, pa, pb, reexpress, ok, err := admitPrismIntersectPair(ctx, a, b)
	if err != nil {
		if errors.Is(err, ErrUnsupported) {
			// RB7 (§9): the arrangement cap. Wrapped exactly as
			// evaluateAnalyticIntersect wraps its own RB7, so
			// measuredInterference reads it as interferenceUnsupportedPipeline
			// and the report says Suspect.
			return Measurement{}, false, expectedBoolean(booleanExpectedUnsupported, err)
		}
		return Measurement{}, false, err
	}
	if !ok {
		return Measurement{}, false, nil
	}

	// Selected cells: Task 1's first half of the crossing resolution, under
	// Intersect's own keep — the arrangement's own cells that
	// classifyPrismCells puts on BOTH operands' material sides.
	selected, sceneDelta, resolved, err := resolvePrismCrossingCells(ctx, budget, pa, pb, reexpress,
		func(a, b bool) bool { return a && b })
	if err != nil {
		return Measurement{}, false, err
	}
	if !resolved {
		return Measurement{}, false, nil
	}
	if len(selected) == 0 {
		// The exactly-tangent case: the arrangement puts no cell on both
		// operands' material sides. §4.5 requires this to decline rather
		// than publish a zero-volume answer.
		return Measurement{}, false, nil
	}

	// §3.2's Intersect z-interval and per-end axial displacement, shared by
	// every measured cell — the pair's own relation, not a per-cell one.
	shift := prismZShift(pa, pb)
	pbZ0, pbZ1 := pb.z0+shift, pb.z1+shift
	z0, z0Delta := prismIntersectEnd(pa.z0, pa.z0Delta, pbZ0, pb.z0Delta, func(x, y float64) bool { return x > y })
	z1, z1Delta := prismIntersectEnd(pa.z1, pa.z1Delta, pbZ1, pb.z1Delta, func(x, y float64) bool { return x < y })

	d := a.doc
	values := make([]float64, 0, len(selected))
	boundTerms := make([]float64, 0, len(selected))
	for _, p := range selected {
		if err := budget.step(); err != nil {
			return Measurement{}, false, err
		}

		// Per-cell measurement: the cell's own closed directed walk,
		// recorded through the existing recordEdge/edgeJoin/
		// prismUnionCutDelta sequence mergePrismCells already uses — no
		// count, drop or chain, since a cell is already one loop in
		// sketch's own order. RB8/RB9 propagate here, exactly as they do on
		// the body path.
		cellProfile, cutDelta, err := recordPrismOverlapCell(budget, p.Outer)
		if err != nil {
			return Measurement{}, false, err
		}

		// §7's formula, byte-identical to
		// resolveAndBuildPrismIntersectCrossing's, taken over THIS cell's
		// own cutDelta.
		sectionDelta := absSumUpper(
			max(
				absSumUpper(pa.sectionDelta, sceneDelta.a),
				absSumUpper(pb.sectionDelta, sceneDelta.b, reexpress.delta),
			),
			cutDelta,
		)
		pp := prismPayload{
			profile:      cellProfile,
			frame:        pa.frame,
			xform:        pa.xform,
			z0:           z0,
			z1:           z1,
			z0Delta:      z0Delta,
			z1Delta:      z1Delta,
			sectionDelta: sectionDelta,
		}

		body, err := evalPrismContext(ctx, d, d.nextStepRef(), pp, newFreeformWork())
		if err != nil {
			if errors.Is(err, ErrDegenerate) || errors.Is(err, ErrUnsupported) {
				// The region integrals refuse this cell's own section as
				// degenerate or unsupported: §4.5 declines the WHOLE
				// reading rather than publish a partial sum over the cells
				// that did measure.
				return Measurement{}, false, nil
			}
			return Measurement{}, false, err
		}
		values = append(values, body.volume.Value.Base())
		boundTerms = append(boundTerms, body.volume.Bound.Base())
	}

	// The sum (§4.5's own "The sum" paragraph): the value is the cells'
	// float sum; the bound is absSumUpper over the cells' own bounds and
	// that sum's own accumulated rounding (bounds.go's exactSumRound, the
	// mechanism's one owner — no new helper is added here).
	sum := 0.0
	for _, v := range values {
		sum += v
	}
	rounding := exactSumRound(sum, values...)
	bound := absSumUpper(append(boundTerms, rounding)...)

	return Measurement{
		Value:     units.CubicMillimeters(sum),
		Exactness: exactnessOf(bound),
		Bound:     units.CubicMillimeters(bound),
	}, true, nil
}

// recordPrismOverlapCell records one arrangement cell's own Outer boundary
// edges into a ProfileRecord, the same recordEdge/edgeJoin/prismUnionCutDelta
// sequence mergePrismCells (prism_boolean.go) already runs over its merged
// chain — minus the count, drop and chain steps, which exist only to build
// one loop out of many. A cell is already one closed directed walk in
// sketch's own order (§4.5), so §5's authentication claim 2 is discharged for
// free here, exactly as it is for the clean-nesting match. cutDelta is the
// maximum §7 cut-parameter charge over the cell's own edges, zero when every
// edge is whole.
func recordPrismOverlapCell(budget *workBudget, edges []sketch.BoundaryEdge) (ProfileRecord, float64, error) {
	segs := make([]CurveSegment, len(edges))
	joins := make([]loopJoin, len(edges))
	cutDelta := 0.0
	for i, e := range edges {
		if err := budget.step(); err != nil {
			return ProfileRecord{}, 0, err
		}
		seg, err := recordEdge(e)
		if err != nil {
			return ProfileRecord{}, 0, err
		}
		segs[i] = seg
		join, err := edgeJoin(e, seg)
		if err != nil {
			return ProfileRecord{}, 0, err
		}
		joins[i] = join
		delta, err := prismUnionCutDelta(e, seg)
		if err != nil {
			return ProfileRecord{}, 0, err
		}
		cutDelta = math.Max(cutDelta, delta)
	}
	// RB9 (§9): the seam's own junction falsifier, run on this cell's own
	// recorded coordinates.
	if err := falsifyLoopJoins("overlap cell", joins); err != nil {
		return ProfileRecord{}, 0, err
	}
	return ProfileRecord{Outer: LoopRecord{Segments: segs}}, cutDelta, nil
}
