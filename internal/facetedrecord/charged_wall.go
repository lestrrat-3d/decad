package facetedrecord

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
)

// chargedWall reports whether g is a retained Cylinder group that states a
// positive wall charge (AxialDelta) beside its tag.
func chargedWall(g Group) bool {
	if _, cyl := g.Surface.(surfacegeom.Cylinder); !cyl {
		return false
	}
	return g.HasAxialDelta && g.AxialDelta > 0
}

// chargedWallTerm returns the normal bound a charged Cylinder tag adds for
// facets within delta of the true patch, and whether the tag is admitted. A
// held point lies within delta_f + delta of the tag's radial line, where
// delta_f is the facets' own displacement and delta the wall charge, and the
// true radius is at least R - delta, so the true normal moves by at most
// 2(delta_f + delta)/(R - delta). That is at most 4(delta_f + delta)/R while the
// term stays below 1, which HeldNormalTerm enforces. The argument is
// 2(delta_f + delta), rounded up (agent-docs/retained-edge-blend-design.md
// §6.1, "Publishing a charged wall").
func chargedWallTerm(surface surfacegeom.Surface, delta, axial float64) (float64, bool) {
	return HeldNormalTerm(surface, proofbound.ProductUpper(2, proofbound.AbsSumUpper(delta, axial)), 0)
}

// heldGroupTerm is the held normal term of a retained group: HeldNormalTerm at
// the facets' displacement, or chargedWallTerm for a charged Cylinder.
func heldGroupTerm(g Group, delta, rhoLower float64) (float64, bool) {
	if chargedWall(g) {
		return chargedWallTerm(g.Surface, delta, g.AxialDelta)
	}
	return HeldNormalTerm(g.Surface, delta, rhoLower)
}

// scaleDisplacement carries a retained group's stated displacement through a
// similarity of scale s whose tag ScaleTag moved. A displacement stated with no
// record (a Cylinder, or a Plane with no CapRecord) scales by |s|, rounded up;
// one stated against a record drops with the record.
func scaleDisplacement(g *Group, s float64) {
	if g.HasAxialDelta && g.CapRecord == nil {
		g.AxialDelta = proofbound.ProductUpper(math.Abs(s), g.AxialDelta)
		return
	}
	g.CapRecord, g.AxialDelta, g.HasAxialDelta, g.ViewCharge = nil, 0, false, false
}
