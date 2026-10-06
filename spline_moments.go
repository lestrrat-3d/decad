package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// This file is docs/spline-design.md §5.1's exact integration: the
// Green's-theorem boundary forms over the Bézier spans spline_bezier.go
// produced. Every integrand is a POLYNOMIAL in the span parameter, so every
// integral is an exact rational — which is what earns a Tier A kind its zero
// bound, and why §5.2 forbids a quadrature fallback here.
//
// The polynomial machinery is internal/freeform/clearance_poly.go's freeform.RatPoly, reused rather than
// forked (spline design §6.2): that file already owns dense rational
// polynomials with the products and derivatives these forms need.

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
func (ig *regionIntegrals) addFreeformTo(spans []survey2d.BezierSpan, reversed bool, order freeform.MomentIntegralOrder) {
	exact := freeform.ExactFreeformMoments(spans, reversed, order)
	if extent := freeform.FreeformControlExtent(spans); extent > ig.coordUpper {
		ig.coordUpper = extent
	}
	moments := []struct {
		value *float64
		bound *float64
		exact *big.Rat
	}{
		{&ig.area, &ig.areaBound, exact.Area},
		{&ig.mu, &ig.muBound, exact.Mu},
		{&ig.mv, &ig.mvBound, exact.Mv},
		{&ig.muu, &ig.muuBound, exact.Muu},
		{&ig.muv, &ig.muvBound, exact.Muv},
		{&ig.mvv, &ig.mvvBound, exact.Mvv},
	}
	if order < freeform.MomentSecondOrder {
		moments = moments[:3]
	}
	for _, moment := range moments {
		held, _ := moment.exact.Float64()
		accumulateMoment(moment.value, moment.bound, held, proofarith.RationalFloatError(moment.exact, held))
	}
	ig.addExact(exact)
}
