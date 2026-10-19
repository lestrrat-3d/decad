package decad

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
// internal/circularbounds/moments.go the circular segment's own enclosures.

// SecondMoments is a region's bounded second moments of area about the plane
// origin. Each reading has Kind SecondMomentOfArea (mm⁴). Re-reference them to
// another axis with the region's Area and Centroid.
type SecondMoments struct {
	// UU is ∫u² dA, VV is ∫v² dA, and UV is ∫uv dA.
	UU Measurement
	UV Measurement
	VV Measurement
}
