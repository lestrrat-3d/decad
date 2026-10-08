package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismextent"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file answers the extent questions asked OF a finished prism: how far
// the solid reaches along a direction, and the axis-aligned box that contains
// it.
//
// Every answer is a bounded interval, not a float: the coefficients of the
// direction in the payload's own frame round, the section and axial
// displacements move the boundary the interval is read from, and a boundary
// extreme riding a walked endpoint or a computed arc radius carries that
// walk's own bound. A direction whose extreme cannot be bracketed refuses
// rather than publishing the held value. See docs/evaluator-design.md §5.

// extentAlong is the through-all stop's reading of the prism (stops.go): the
// extent interval along an arbitrary world direction g — the lifted linear
// functional point·g = origin·g + u·(U'·g) + v·(V'·g) + z·(N'·g), primes the
// placed directions, extremized over the region boundary and the sweep —
// beside the proven displacement extentBoundedAlong states for its two ends.
// The stop charges that displacement to the level it resolves and decides its
// own in-path test outside it, so a boundary extreme held by a bracket
// (docs/spline-design.md §6.2) still answers rather than refusing.
//
// The section displacement is NOT one of the terms this reading carries — it
// moves a coordinate IN the plane, and the interval is stated over the
// recorded section — so a prism holding one refuses instead
// (docs/prism-boolean-design.md §12). prismBoundsContext reads
// extentBoundedAlong directly and composes both terms into its own outward
// bound.
// The stop and clearance callers hold no preflight counter for this record, so
// the interface forms open the record's own — one per extent reading, never one
// per segment.
func (pp prismPayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	if pp.sectionDelta != 0 {
		return 0, 0, 0, fmt.Errorf(`%w: a through-all stop cannot use a prism with a proven section displacement`, ErrUnsupported)
	}
	return pp.extentBoundedAlong(context.Background(), g, freeform.NewFreeformWork(), nil)
}

func (pp prismPayload) extentAlongContext(ctx context.Context, g r3.Vec) (float64, float64, error) {
	return pp.extentAlongWork(ctx, g, freeform.NewFreeformWork())
}

// extentAlongWork is extentBoundedAlong's refusing wrapper, mirroring
// revolvePayload.extentAlongWork word for word: the reading for a consumer that
// takes the interval as an exact one and has nowhere to put a displacement.
// clearance.go's payloadExtent is that consumer — its separating-plane
// short-circuit compares two bodies' intervals and simply loses the
// short-circuit where it cannot get an exact one — so a direction whose extreme
// only a bracket holds refuses here rather than publish a held coordinate as the
// one it denotes. Which candidate holds it does not matter and the refusal never
// names a kind: a free-form span's enclosure, a computed arc radius and a walked
// endpoint the record does not state all reach this wrapper the same way,
// through one nonzero bound. A through-all stop does not read through this wrapper:
// it consumes the bounded reading and charges the displacement to its own level
// (stops.go, docs/spline-design.md §6.4).
func (pp prismPayload) extentAlongWork(ctx context.Context, g r3.Vec, work *freeform.FreeformWork) (float64, float64, error) {
	lo, hi, bound, err := pp.extentBoundedAlong(ctx, g, work, nil)
	if err != nil {
		return 0, 0, err
	}
	if bound != 0 {
		return 0, 0, fmt.Errorf(`%w: this prism's extent along this direction is known only to a proven displacement of %v mm; this reading has no bound to widen`, ErrUnsupported, bound)
	}
	return lo, hi, nil
}

// extentBoundedAlong is the bounded reading itself: the interval AND its
// proven half-width, folded from boundaryExtremesBoundedContext's own
// per-candidate enclosures (docs/spline-design.md §6.2) AND from the frame and
// placement's own rounding (prismPlacementCoeffAllow). The boundary-scan term
// follows the CANDIDATES the extremes are held by, never the section's kind: a
// section whose extremes are all values the record states — straight walls,
// and an arc or circle read where its own recorded endpoint or its exactly
// representable apex wins — carries only zero-width candidates, while a
// trimmed circular endpoint, a computed arc radius or a free-form span's
// enclosure each publish the width their own construction owes. The frame and
// placement term is independent of it and composes outward: a straight-walled
// section under a tilted placement still widens, and an unplaced or
// axis-aligned section still reports zero for this term, which is what keeps
// an ordinary prism's box Exact.
//
// A THIRD term composes outward with both: the reading's own final summation
// base + lo + zlo, charged exactly against the same terms by proofbound.ExactSumRound
// (internal/proofbound/bounds.go). It is not covered by either of the other two — a pure
// translation leaves every coefficient exactly right and every multiply exact,
// and the addition that follows still rounds — and it is zero exactly where
// that addition is exactly representable, so an unplaced prism's box stays
// Exact.
//
// walks is pp.profile's pre-resolved segment walks, or nil to resolve as
// before through boundaryExtremesBoundedContext and momentinput.CoordinateEnvelope's
// own walkOf calls. prismBoundsContext passes the same *momentinput.ProfileWalks to every
// one of its three per-axis calls, so the record's boundary walks resolve
// once for the whole box rather than once per axis (this file's momentinput.ProfileWalks
// doc comment).
func (pp prismPayload) extentBoundedAlong(ctx context.Context, g r3.Vec, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (float64, float64, float64, error) {
	base := pp.xform.Apply(pp.frame.Origin()).Dot(g)
	gu := pp.dir(1, 0, 0).Dot(g)
	gv := pp.dir(0, 1, 0).Dot(g)
	gz := pp.dir(0, 0, 1).Dot(g)
	lo, hi, bound, err := boundaryExtremesBoundedContext(ctx, pp.profile, gu, gv, work, walks)
	if err != nil {
		return 0, 0, 0, err
	}
	zlo := math.Min(pp.z0*gz, pp.z1*gz)
	zhi := math.Max(pp.z0*gz, pp.z1*gz)
	coordUpper, err := momentinput.CoordinateEnvelope(pp.profile, work, walks)
	if err != nil {
		return 0, 0, 0, err
	}
	zUpper := math.Max(math.Abs(pp.z0), math.Abs(pp.z1))
	placeAllow := prismPlacementCoeffAllow(pp, g, base, gu, gv, gz, coordUpper, zUpper)
	// The recombination is charged per ENDPOINT and composed outward: the two
	// ends are summed from different terms and round by different amounts, while
	// the scan's and the placement's terms speak for both ends alike, so the
	// reading publishes the larger of the two per-end totals — the same shape
	// revolvePayload.extentBoundedAlong states for its own per-end composition.
	loEnd, hiEnd := base+lo+zlo, base+hi+zhi
	sumAllow := math.Max(
		proofbound.ExactSumRound(loEnd, base, lo, zlo),
		proofbound.ExactSumRound(hiEnd, base, hi, zhi),
	)
	bound = proofbound.AbsSumUpper(bound, placeAllow, sumAllow)
	return loEnd, hiEnd, bound, nil
}

// prismPlacementCoeffAllow adapts the held prism frame for prismextent.
func prismPlacementCoeffAllow(pp prismPayload, g r3.Vec, base, gu, gv, gz, coordUpper, zUpper float64) float64 {
	return prismextent.PlacementCoeffAllow(pp.xform, pp.frame, g, base, gu, gv, gz, coordUpper, zUpper)
}

func prismDecompositionRoundAllow(gu, gv, gz, base, coordUpper, zUpper float64) float64 {
	return prismextent.DecompositionRoundAllow(gu, gv, gz, base, coordUpper, zUpper)
}

// prismBoundsContext computes the exact axis-aligned bounds of the placed prism:
// for each world axis, the directional extreme of the region boundary under
// the lifted linear functional, plus the sweep's own extreme
// (docs/evaluator-design.md §5).
//
// walks is pp.profile's pre-resolved segment walks, or nil to resolve each
// segment through walkOf as before. Passed straight to all three per-axis
// extentBoundedAlong calls below (this file's momentinput.ProfileWalks doc comment), so a
// non-nil walks resolves the record's boundary once for the whole box instead
// of once per axis.
func prismBoundsContext(ctx context.Context, pp prismPayload, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (Box, error) {
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	var minC, maxC [3]float64
	extremeBound := 0.0
	for i, axis := range axes {
		if err := ctx.Err(); err != nil {
			return Box{}, err
		}
		lo, hi, bound, err := pp.extentBoundedAlong(ctx, axis, work, walks)
		if err != nil {
			return Box{}, err
		}
		minC[i] = lo
		maxC[i] = hi
		extremeBound = math.Max(extremeBound, bound)
	}
	// A displaced section displaces every extreme it holds, so the box's own
	// error carries the section displacement itself — δ outward on every face
	// (docs/prism-boolean-design.md §7) — summed with the boundary's own
	// directional-extreme bracket bound (docs/spline-design.md §6.2) and with
	// the frame and placement's own rounding (prismPlacementCoeffAllow) and the
	// endpoint summation's (proofbound.ExactSumRound), both folded into
	// extentBoundedAlong's own returned bound above: the frame and
	// placement ARE isometries in exact arithmetic, but their FLOAT evaluation
	// rounds wherever the frame is not axis-aligned or the placement is not the
	// identity, and adding the resulting terms into one published coordinate
	// rounds again — for a pure translation it is the ONLY rounding there is —
	// so a box that reads either as an exact leaf can miss
	// the true extreme by a representable amount. All three terms are zero for
	// a caller-drawn, unplaced, axis-aligned payload whose extremes are all
	// values its record states, which keeps the ordinary prism's box Exact as
	// before. The bracket's own term is what decides the rest, never the
	// section's kind: a straight-walled section reports zero, an analytic one whose extreme is
	// held by a trimmed circular endpoint or a computed arc radius reports that
	// candidate's own width, and a free-form section whose extremes along
	// these three axes are all held by exactly representable candidate values
	// reports a zero width and stays Exact too (a span monotone along an axis
	// contributes its two exactly interpolated endpoints and nothing else),
	// while an extreme held by an irrational interior root publishes that
	// bracket's width and is Approximate — §6.2's own stated contract
	// consequence. The sum only
	// goes through proofbound.AbsSumUpper's own per-term rounding where there are two
	// genuine terms to compose: bumping a lone sectionDelta a second time for
	// an always-zero extremeBound term would grow the box's bound past the
	// single proofbound.UpRound tessellate.go's own mesh bound composes it against.
	//
	// The sweep's own ends enter the same way. Each axis reading takes the
	// levels through zlo/zhi scaled by |gz| ≤ 1 (a unit axis against a placed
	// unit normal), so the larger end displacement bounds the box face either
	// level can move, and it composes with the section's term because the two
	// displace along different axes.
	axial := pp.axialDelta()
	terms := make([]float64, 0, 3)
	if pp.sectionDelta != 0 {
		terms = append(terms, pp.sectionDelta)
	}
	if extremeBound != 0 {
		terms = append(terms, extremeBound)
	}
	if axial != 0 {
		terms = append(terms, axial)
	}
	bound := 0.0
	switch len(terms) {
	case 0:
	case 1:
		bound = terms[0]
	default:
		bound = proofbound.AbsSumUpper(terms...)
	}
	return Box{
		Min:       r3.NewVec(minC[0], minC[1], minC[2]),
		Max:       r3.NewVec(maxC[0], maxC[1], maxC[2]),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// boundaryExtremesBoundedContext adapts recorded profile walks for prismextent.
func boundaryExtremesBoundedContext(ctx context.Context, profile ProfileRecord, gu, gv float64, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (float64, float64, float64, error) {
	if err := freeform.RequireFiniteDirection(gu, gv); err != nil {
		return 0, 0, 0, err
	}
	if walks != nil && !walks.Matches(profile) {
		return 0, 0, 0, momentinput.ErrResolvedWalksMismatch
	}
	loops := append([]LoopRecord{profile.Outer}, profile.Holes...)
	counts := make([]int, len(loops))
	for li, loop := range loops {
		counts[li] = len(loop.Segments)
	}
	return prismextent.BoundaryExtremesBoundedContext(ctx, gu, gv, work, counts, func(li, si int) (survey2d.SegmentWalk, error) {
		return momentinput.ResolveOrRead(loops[li].Segments[si], work, walks, li, si)
	})
}
