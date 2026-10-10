package decad

import (
	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/r3"
)

// axisBoxPrism admits a prism only when its recorded section is one complete
// rectangle, its frame preserves the world axes, and it has no placement.
// The body's exact Bounds then give the six actual planes of the solid.
func axisBoxPrism(b *Body) bool {
	if !b.solid || b.kind != BodySolid || b.bounds.Exactness != Exact || b.bounds.Bound.Base() != 0 {
		return false
	}
	pp, ok := b.payload.(prismPayload)
	if !ok || pp.surfaceResult || pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 {
		return false
	}
	if !box.CardinalBasis(pp.frame.U(), pp.frame.V(), pp.frame.N()) {
		return false
	}
	basis := pp.xform.Basis()
	if basis.EX != (r3.Vec{X: 1}) || basis.EY != (r3.Vec{Y: 1}) ||
		basis.EZ != (r3.Vec{Z: 1}) || pp.xform.Translation() != (r3.Vec{}) {
		return false
	}
	return box.RectangularProfile(pp.profile)
}

// clearanceAxisBoxes gives the closed-form gap of two certified rectangular
// solids. A positive box gap also excludes nesting. Contact and near-contact
// keep the general kernel's existing certificate and tolerance decisions.
func clearanceAxisBoxes(a, b *Body) (pairResult, bool) {
	if !axisBoxPrism(a) || !axisBoxPrism(b) {
		return pairResult{}, false
	}
	ab, bb := a.bounds, b.bounds
	gap, ok := box.CertifiedDisjointGap(ab.Min, ab.Max, bb.Min, bb.Max)
	if !ok {
		return pairResult{}, false
	}
	return pairResult{verdict: pairDisjoint, lo: gap.Lo, hi: gap.Hi, exact: gap.Exact, diam: gap.Diameter}, true
}
