package loftcontact

import (
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// FitPreflight charges the fit-point scan and the established
// interpolant cost before the source reader allocates per-cap span arrays.
// This source-only stage does not solve the interpolant; a later consumer
// must still charge and validate any conversion it actually performs.
func FitPreflight(budget *proofbound.WorkBudget,
	prism, lower, upper []sectionrecord.CurveSegment) (uint64, bool, error) {
	var visits uint64
	for _, loop := range [4][]sectionrecord.CurveSegment{prism, lower, upper, prism} {
		for _, segment := range loop {
			fit, ok := segment.(sectionrecord.FitSplineSeg)
			if !ok {
				continue
			}
			if len(fit.Fit) < 2 {
				return 0, false, nil
			}
			cost, valid := proofbound.WallCheckedAdd(
				freeform.FitInterpolantCost(len(fit.Fit)), uint64(len(fit.Fit)))
			if !valid {
				return 0, false, proofbound.ErrWallWorkBudget
			}
			if _, err := budget.StepN(cost); err != nil {
				return 0, false, err
			}
			visits += cost
		}
	}
	return visits, true, nil
}

func SameKind(first, second sectionrecord.CurveSegment) bool {
	switch first.(type) {
	case sectionrecord.LineSeg:
		_, ok := second.(sectionrecord.LineSeg)
		return ok
	case sectionrecord.CircleSeg:
		_, ok := second.(sectionrecord.CircleSeg)
		return ok
	case sectionrecord.ArcSeg:
		_, ok := second.(sectionrecord.ArcSeg)
		return ok
	case sectionrecord.FitSplineSeg:
		_, ok := second.(sectionrecord.FitSplineSeg)
		return ok
	default:
		return false
	}
}

func SceneRangeFits(start, end float64) bool {
	if proofbound.IsNonFinite(start) || proofbound.IsNonFinite(end) ||
		start < 0 || end > 1 || start >= end {
		return false
	}
	return VolumeRatFits(proofarith.FloatRat(start)) &&
		VolumeRatFits(proofarith.FloatRat(end))
}
