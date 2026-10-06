package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is the exact planar arm of Verify's pair partition
// (docs/interference-design.md §3.2, docs/clearance-design.md §7): a pair the
// analytic clearance kernel leaves undecided — because an operand has no
// carrier model there (a mitred sweep, a faceted Boolean result), or because
// its enumeration could not settle the pair — is handed to the exact planar
// relation of docs/multibody-dynamics-design.md §9, over the same admitted
// snapshots Document.ContactPair reads (planarSolidAtPose,
// contact_faceted_pair.go). Every predicate there is an exact sign over
// dyadic coordinates, so a verdict it returns is a certificate, never a
// float estimate, and the one rounding — a positive gap's square root — is
// enclosed outward.

// planarPairAdmits reports whether the exact planar arm serves this pair.
// It exists for the two payload classes the analytic kernel has no carrier
// model for — a mitred sweep (docs/sweep-design.md Table DM row DM5) and a
// faceted Boolean result (docs/payload-verification-design.md §7) — each read
// off its own held triangle set with that set's displacement, and it admits a
// prism or a stitched solid only as their partner, read exactly off its own
// record (contact_faceted_pair.go's planarPrismSolid and planarStitchSolid).
// A pair of prisms or stitched solids keeps the analytic kernel's answer,
// and every other payload keeps its own design's staging: a cup is never
// tessellated for verification (payload verification §1), and a loft, a
// composite sweep and a cap-loop chamfer stay undecided until their own
// adapters land.
func planarPairAdmits(a, b *Body) bool {
	served := false
	for _, body := range []*Body{a, b} {
		switch body.payload.(type) {
		case mitredSweepPayload, facetedPayload:
			served = true
		case prismPayload, stitchPayload:
		default:
			return false
		}
	}
	return served
}

// planarPairVerdict decides an admitted pair (planarPairAdmits) through the
// exact planar relation and §10.4's displacement band. ok is false when the
// pair is not served, or when either body is refused a planar snapshot at
// its own placement: a sheet, a displaced section, or a held mesh the
// tessellator refuses.
//
// With δ the summed displacement of the two held boundaries, every true
// boundary point lies within δ of the held pair's, and a point off the held
// boundary by more than its body's displacement keeps its side of it
// (docs/payload-verification-design.md §2.3's 1-Lipschitz rule). So:
//
//   - a held gap g, enclosed in [gLo, gHi], proves the true solids disjoint
//     only where gLo − δ > 0, and the true gap then lies in
//     [gLo − δ, gHi + δ], rounded outward; a held gap within δ proves
//     nothing and stays undecided;
//   - a held touch is pairTouching only at δ = 0, where the held boundary IS
//     the true one and the kernel's local-separation certificate is an
//     exact material-side claim; a touch within a band stays undecided;
//   - a held overlap is pairOverlapping at δ = 0, and otherwise only through
//     a vertex proven deeper than δ inside the other solid
//     (pair.PlanarDeepVertex), which survives any displacement within δ.
//
// The pair diameter the §7 gate reads is interferencePairDiameter's, the
// same reading the interference path forms for this pair. The verdict never
// loosens the analytic kernel's: the caller consults this arm only for a
// pairUndecided result.
func planarPairVerdict(ctx context.Context, a, b *Body) (pairResult, bool, error) {
	if !planarPairAdmits(a, b) {
		return pairResult{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return pairResult{}, false, err
	}
	// A zero chord admits no curved body (planarHeldMeshSolid): those keep
	// the analytic kernel's answer, and this arm reads only an all-planar
	// held boundary, which that chord reads exactly.
	sa, deltaA, okA, err := planarSolidAtPose(ctx, budget, a, r3.Identity(), 0)
	if err != nil || !okA {
		return pairResult{}, false, err
	}
	sb, deltaB, okB, err := planarSolidAtPose(ctx, budget, b, r3.Identity(), 0)
	if err != nil || !okB {
		return pairResult{}, false, err
	}
	result, err := pair.ClassifyPlanar(&sa, &sb, budget.Step)
	if err != nil {
		return pairResult{}, false, err
	}
	diam, err := interferencePairDiameter(ctx, a, b)
	if err != nil {
		return pairResult{}, false, err
	}
	delta := proofarith.DyAdd(deltaA, deltaB)
	undecided := pairResult{diam: diam}
	switch result.Relation {
	case pair.Separated:
		res, ok := planarDisjointResult(result.Gap, delta.Rat(), diam)
		if !ok {
			return undecided, true, nil
		}
		return res, true, nil
	case pair.Touching:
		if delta.Sign() != 0 {
			return undecided, true, nil
		}
		return pairResult{verdict: pairTouching, exact: true, diam: diam}, true, nil
	case pair.Overlapping:
		if delta.Sign() == 0 {
			return pairResult{verdict: pairOverlapping, diam: diam}, true, nil
		}
		deep, err := pair.PlanarDeepVertex(&sa, &sb, delta, budget.Step)
		if err != nil {
			return pairResult{}, false, err
		}
		if !deep {
			return undecided, true, nil
		}
		return pairResult{verdict: pairOverlapping, diam: diam}, true, nil
	default:
		return undecided, true, nil
	}
}

// planarDisjointResult widens the held gap's enclosure by delta, in exact
// rationals, and publishes the pair disjoint only where the widened lower
// end stays positive after rounding down. exact holds only when the held
// gap is a single float and delta is zero, so the interval is a point.
func planarDisjointResult(gap *pair.ScalarReading, delta *big.Rat, diam float64) (pairResult, bool) {
	if gap == nil {
		return pairResult{}, false
	}
	value, bound := proofarith.FloatRat(gap.ValueMM), proofarith.FloatRat(gap.BoundMM)
	loRat := new(big.Rat).Sub(value, bound)
	loRat.Sub(loRat, delta)
	hiRat := new(big.Rat).Add(value, bound)
	hiRat.Add(hiRat, delta)
	lo, hi := proofbound.RatFloatDown(loRat), proofbound.RatFloatUp(hiRat)
	if !(lo > 0) || !finiteMeasurementValues(lo, hi) {
		return pairResult{}, false
	}
	return pairResult{verdict: pairDisjoint, lo: lo, hi: hi, exact: lo == hi, diam: diam}, true
}
