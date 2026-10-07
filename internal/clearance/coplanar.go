package clearance

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// CoplanarRelation classifies two parallel-plane trims in projection along
// the normal: +1 proven positive-area overlap (with a witness point in f's
// frame), −1 provenly apart, 0 ambiguous. Sample-based in the sufficient
// direction, boundary-clearance-based in the exclusion direction — never a
// blessed ambiguity.
func CoplanarRelation(budget *proofbound.WorkBudget, f, g *CFace) (int, [2]float64, error) {
	if err := budget.Err(); err != nil {
		return 0, [2]float64{}, err
	}
	ge := make([]survey2d.SurveyElem, 0, len(g.Region.Elems))
	for _, e := range g.Region.Elems {
		if err := budget.Step(); err != nil {
			return 0, [2]float64{}, err
		}
		ge = append(ge, TransformElem(e, g, f))
	}
	greg, err := NewRegion2Budget(budget, ge)
	if err != nil {
		return 0, [2]float64{}, err
	}
	tol := math.Max(f.Region.Tol(), greg.Tol())

	probeInto := func(src, dst Region2) (int, [2]float64, error) {
		p, ok, err := RegionInteriorPointBudget(budget, src)
		if err != nil {
			return 0, [2]float64{}, err
		}
		if ok {
			class, err := RegionClassifyBudget(budget, dst, p[0], p[1], tol)
			if err != nil {
				return 0, [2]float64{}, err
			}
			if class == 1 {
				return 1, p, nil
			}
		}
		samples, err := RegionSamplesBudget(budget, src)
		if err != nil {
			return 0, [2]float64{}, err
		}
		for _, s := range samples {
			if err := budget.Step(); err != nil {
				return 0, [2]float64{}, err
			}
			class, err := RegionClassifyBudget(budget, dst, s[0], s[1], tol)
			if err != nil {
				return 0, [2]float64{}, err
			}
			if class == 1 {
				return 1, s, nil
			}
		}
		return 0, [2]float64{}, nil
	}
	r, w, err := probeInto(greg, f.Region)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if r == 1 {
		return 1, w, nil
	}
	r, w, err = probeInto(f.Region, greg)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if r == 1 {
		return 1, w, nil
	}
	// Exclusion: boundaries clear each other and neither contains the other.
	clearing, err := CoplanarBoundaryClearanceBudget(budget, f.Region, greg)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if clearing <= tol {
		return 0, [2]float64{}, budget.Err()
	}
	outsideFG, err := RegionSampleOutsideBudget(budget, f.Region, greg, tol)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if !outsideFG {
		return 0, [2]float64{}, budget.Err()
	}
	outsideGF, err := RegionSampleOutsideBudget(budget, greg, f.Region, tol)
	if err != nil {
		return 0, [2]float64{}, err
	}
	if outsideGF {
		return -1, [2]float64{}, nil
	}
	return 0, [2]float64{}, budget.Err()
}
