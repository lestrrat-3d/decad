package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/momentline"
	"github.com/lestrrat-3d/decad/internal/momentregion"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file adapts recorded regions to the mass-property engine of
// docs/evaluator-design.md §4. sketch decides topology and admissibility;
// internal/momentregion integrates admitted records by closed-form boundary
// integrals (Green's theorem). Line and Tier A free-form walks
// (docs/spline-design.md Table F) integrate to exact rationals, so a region
// built only from them is published as its own rational rounded ONCE and
// retains a zero bound wherever that rational is representable; circular
// evaluations have no exact rational and carry outward bounds instead.
// Every other free-form kind is unsupported.
//
// Three sibling files carry the machinery this engine integrates with, each
// with its own doc comment: internal/proofbound/bounded.go the bounded-scalar arithmetic every
// published reading is composed in, internal/proofbound/rat_interval.go the exact rational
// interval arithmetic the certified terms are proven in, and
// moments_circular.go the circular segment's own enclosures.

// SecondMoments is a region's bounded second moments of area about the plane
// origin. Each reading has Kind SecondMomentOfArea (mm⁴). Re-reference them to
// another axis with the region's Area and Centroid.
type SecondMoments struct {
	// UU is ∫u² dA, VV is ∫v² dA, and UV is ∫uv dA.
	UU Measurement
	UV Measurement
	VV Measurement
}

// regionIntegrals is the section accumulator shared by root evaluators.
type regionIntegrals = momentinput.Integrals

func newExactMoments() freeform.ExactMoments { return momentregion.NewExactMoments() }

func ratScale(value *big.Rat, num, den int64) *big.Rat {
	return momentline.RatScale(value, num, den)
}

func ratLerp(start, end, t float64) *big.Rat {
	return momentline.RatLerp(start, end, t)
}

// lerp2 returns the point at parameter t on the segment start→end.
//
// At the two natural bounds the answer is the record's own coordinate: the
// parameterization is P(t) = start + t·(end − start), so P(0) is start and P(1)
// is end, exactly. Those two cases therefore return the endpoint verbatim
// instead of evaluating the formula, whose float rounding need not land back on
// it — start + (end − start) can miss end by an ulp whenever the difference
// itself rounds. That is not a repair of the input: it is the same value the
// exact-rational twin ratLerp already returns at both bounds, and the same rule
// sketchrecord.EdgeJoin already applies when it reads an uncut bound
// (TStart == 0 or TEnd == 1) off the record rather than off sketch's node.
//
// Reproducing the endpoint matters to every consumer that rebuilds geometry
// from a walk and then compares it against the record: buildPrismScene
// (prism_boolean.go) creates one sketch point per walked endpoint, so a walk
// that missed a whole segment's own vertex by an ulp would hand sketch two
// distinct points where the record states one, and the region sketch then
// admits on its proximity threshold would fail the seam's loop-closure
// falsifier at RecordProfile.
func lerp2(start, end Point2, t float64) (float64, float64) {
	return momentregion.Lerp2(start, end, t)
}

func arcRadiusUpper(seg arcSeg) float64 {
	// The exact coordinate differences can each be no larger than the sum of
	// their input magnitudes, and hypot is no larger than the L1 norm.
	return proofbound.AbsSumUpper(seg.Start.U, seg.Center.U, seg.Start.V, seg.Center.V)
}
