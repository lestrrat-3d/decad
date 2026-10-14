package decad

import (
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// axisBoxPrism admits a prism only when its recorded section is one complete
// rectangle, its frame preserves the world axes, and it has no placement.
// The body's exact Bounds then give the six actual planes of the solid.
func axisBoxPrism(b *Body) bool {
	if !b.n.Solid || b.n.Kind != topo.BodySolid || b.n.Bounds.Exactness != measurement.Exact || b.n.Bounds.Bound.Base() != 0 {
		return false
	}
	pp, ok := b.n.Payload.(prismPayload)
	if !ok || pp.SurfaceResult || pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 {
		return false
	}
	if !box.CardinalBasis(pp.Frame.U(), pp.Frame.V(), pp.Frame.N()) {
		return false
	}
	basis := pp.Xform.Basis()
	if basis.EX != (r3.Vec{X: 1}) || basis.EY != (r3.Vec{Y: 1}) ||
		basis.EZ != (r3.Vec{Z: 1}) || pp.Xform.Translation() != (r3.Vec{}) {
		return false
	}
	return box.RectangularProfile(pp.Profile)
}

// clearanceAxisBoxes gives the closed-form gap of two certified rectangular
// solids. A positive box gap also excludes nesting. Contact and near-contact
// keep the general kernel's existing certificate and tolerance decisions.
func clearanceAxisBoxes(a, b *Body) (pairResult, bool) {
	if !axisBoxPrism(a) || !axisBoxPrism(b) {
		return pairResult{}, false
	}
	ab, bb := a.n.Bounds, b.n.Bounds
	gap, ok := box.CertifiedDisjointGap(ab.Min, ab.Max, bb.Min, bb.Max)
	if !ok {
		return pairResult{}, false
	}
	return pairResult{verdict: reportvocab.PairDisjoint, lo: gap.Lo, hi: gap.Hi, exact: gap.Exact, diam: gap.Diameter}, true
}
