package decad

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This measures only the part of the inset rim in a radial uncertainty band
// around the source root. Complete cap source coverage is checked separately.
func loftCylinderChamferCapRadialReach(source loftCylinderChamferSource,
	verts []r3.Vec, ring []int, capIndex int, outerRadius float64) (float64, error) {
	if source.blankRadius <= 0 || proofbound.IsNonFinite(source.blankRadius) ||
		proofbound.IsNonFinite(outerRadius) || capIndex < 0 || capIndex > 1 ||
		outerRadius <= source.blankRadius {
		return 0, fmt.Errorf("%w: cap radial band has invalid limits", ErrUnsupported)
	}
	outer := proofarith.FloatRat(outerRadius)
	outer2 := new(big.Rat).Mul(outer, outer)
	for _, tooth := range source.teeth {
		if tooth == nil {
			return 0, fmt.Errorf("%w: cap radial band lacks a source tooth", ErrUnsupported)
		}
		lp, ok := tooth.n.Payload.(loftPayload)
		if !ok || !lp.authenticatedReconstruction {
			return 0, fmt.Errorf("%w: cap slit lacks an authenticated tooth", ErrUnsupported)
		}
		profile := lp.profile0
		if capIndex == 1 {
			profile = lp.profile1
		}
		if len(profile.Outer.Segments) <= 5 {
			return 0, fmt.Errorf("%w: cap slit lacks the source root arc", ErrUnsupported)
		}
		arc, ok := profile.Outer.Segments[5].(arcSeg)
		if !ok {
			return 0, fmt.Errorf("%w: cap slit lacks the source root arc", ErrUnsupported)
		}
		p := loftcylinder.SourcePoint(lp.xform.Basis().EX, arc.Start)
		if q := loftcylinder.SourceDot(p, p); q.Cmp(outer2) > 0 {
			return 0, fmt.Errorf("%w: cap radial band misses a source root", ErrUnsupported)
		}
	}
	return loftcylinder.ChamferCapRadialRun(verts, ring, source.blankRadius, source.boreRadius, outerRadius)
}
