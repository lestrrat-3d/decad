package revolveproof

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// CellSlack is one meridian cell's Ecell
// (docs/tessellation-design.md §10.2), stated ONCE for the cell and multiplied
// by the angular count by the caller.
//
// One evaluation answers for every angular interval because the ideal samples
// at interval l are the EXACT rotation, about the axis by l·dφ, of those at
// interval 0. A rotation is an isometry, so the true patch and the held facets
// alike are congruent across intervals and their area densities are equal — the
// enclosures differ in width alone, and this reads interval 0's.
//
// A STRAIGHT generator's densities collapse to a difference that is linear in
// the meridian parameter, so its cell is decomposed in closed form
// (revolvemesh.RevolveCellAreaSlack, tess §15's T2 choice). A CIRCULAR generator's does
// not, so its cell takes certified interval subdivision instead
// (revolvemesh.RevolveArcCellSlack, tess §15's T3 choice). coord is the composed coordinate
// displacement, which the circular arms widen their meridian model by.
func CellSlack(b revolvemesh.RevolveBasis3Iv, angular revolvemesh.RevolveAngular, lo, hi revolvemesh.RevMeridian, coord float64) (float64, error) {
	corner := func(s revolvemesh.RevMeridian, l int) survey2d.IvVec3 {
		return revolvemesh.RevolveIdealPoint(b, s.ZIv, s.RhoIv, angular.CosIv[l], angular.SinIv[l])
	}
	p00, p01 := corner(lo, 0), corner(lo, 1)
	p10, p11 := corner(hi, 0), corner(hi, 1)
	if lo.Arc != nil {
		switch {
		case lo.OnAxis:
			area, ok := revolvemesh.IvTwoTriangleArea(p00, p10, p11)
			if !ok {
				return 0, revolvemesh.ErrRevolveArcCellSlack
			}
			return revolvemesh.RevolveArcFanSlack(*lo.Arc, true, angular.Step, area, coord)
		case hi.OnAxis:
			area, ok := revolvemesh.IvTwoTriangleArea(p00, p10, p01)
			if !ok {
				return 0, revolvemesh.ErrRevolveArcCellSlack
			}
			return revolvemesh.RevolveArcFanSlack(*lo.Arc, false, angular.Step, area, coord)
		default:
			lowHalf, ok0 := revolvemesh.IvTwoTriangleArea(p00, p10, p11)
			highHalf, ok1 := revolvemesh.IvTwoTriangleArea(p00, p11, p01)
			if !ok0 || !ok1 {
				return 0, revolvemesh.ErrRevolveArcCellSlack
			}
			return revolvemesh.RevolveArcCellSlack(*lo.Arc, angular.Step, [2]proofbound.RatInterval{lowHalf, highHalf}, coord)
		}
	}

	dz, drho := proofarith.FloatRat(hi.Z-lo.Z), proofarith.FloatRat(hi.Rho-lo.Rho)
	if dz == nil || drho == nil {
		return 0, errRevolveCellSlack
	}
	lenSq := proofbound.IntervalAdd(proofbound.IntervalSquare(proofbound.PointInterval(dz)), proofbound.IntervalSquare(proofbound.PointInterval(drho)))
	meridian, ok := proofbound.IntervalSqrt(lenSq)
	if !ok {
		return 0, errRevolveCellSlack
	}
	switch {
	case lo.OnAxis:
		area, ok := revolvemesh.IvTwoTriangleArea(p00, p10, p11)
		if !ok {
			return 0, errRevolveCellSlack
		}
		return revolvemesh.RevolveFanAreaSlack(hi.Rho, true, meridian, angular.Step, area), nil
	case hi.OnAxis:
		area, ok := revolvemesh.IvTwoTriangleArea(p00, p10, p01)
		if !ok {
			return 0, errRevolveCellSlack
		}
		return revolvemesh.RevolveFanAreaSlack(lo.Rho, false, meridian, angular.Step, area), nil
	default:
		lowHalf, ok0 := revolvemesh.IvTwoTriangleArea(p00, p10, p11)
		highHalf, ok1 := revolvemesh.IvTwoTriangleArea(p00, p11, p01)
		if !ok0 || !ok1 {
			return 0, errRevolveCellSlack
		}
		return revolvemesh.RevolveCellAreaSlack(lo.Rho, hi.Rho, meridian, angular.Step, [2]proofbound.RatInterval{lowHalf, highHalf}), nil
	}
}

var errRevolveCellSlack = fmt.Errorf(`%w: a revolve cell states no enclosure of the area its held facets and the patch they stand for differ by`, decaderr.ErrUnsupported)
