package decad

import "github.com/lestrrat-3d/decad/internal/freeform"

// This file adapts docs/spline-design.md §5.1's exact integration into the
// region accumulator. Every integrand over the Bézier spans is a polynomial,
// so the span integral is an exact rational.
//
// The polynomial machinery is polynomial.RatPoly from internal/polynomial
// (spline design §6.2).

// addFreeformTo accumulates one converted free-form curve's contribution. The
// exact rational goes into the REGION's own accumulator, which is what the
// published float is rounded from — once, after the complete signed
// outer-minus-holes sum (moments.go). Rounding here instead would make a
// multi-segment region's held float a sum of per-curve roundings, and the
// single-rounding bound docs/spline-design.md §3 requires would hold only for a
// region whose whole boundary is one segment. The float accumulation below is
// what the region publishes instead when some OTHER segment — a circular walk —
// leaves it with no exact rational at all.
//
// The chain arrives already converted, re-anchored and CHARGED by the
// record-level preflight, so nothing here consults the work counter.
func (ig *regionIntegrals) addFreeformTo(spans []freeform.BezierSpan, reversed bool, order freeform.MomentIntegralOrder) {
	ig.state().AddFreeform(spans, reversed, order)
}
