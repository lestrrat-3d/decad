package spherepath

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
)

// PairReplay checks rounded source centers against the exact affine path and
// the producer's certified relation. An empty result means the geometry fits;
// the caller must still check the source features of a persistent point track.
func PairReplay(m PairMotion, observedA, observedB proofarith.DyV3, outcome reportvocab.SweepOutcome,
	f, resolution, lo, hi, grazingAt *big.Rat, hasTrack bool) string {
	idealA := pairIdealCenter(m.CenterA, m.DeltaA, f)
	idealB := pairIdealCenter(m.CenterB, m.DeltaB, f)
	deviation := new(big.Rat)
	for _, pair := range []struct {
		ideal    [3]*big.Rat
		observed proofarith.DyV3
	}{{idealA, observedA}, {idealB, observedB}} {
		for axis := range 3 {
			difference := new(big.Rat).Sub(pair.observed[axis].Rat(), pair.ideal[axis])
			deviation.Add(deviation, difference.Abs(difference))
		}
	}
	if deviation.Cmp(resolution) > 0 {
		return "rounded sphere-pair pose exceeds point resolution"
	}
	idealDistance2 := pairCenterDistance2(idealA, idealB)
	actualDistance2 := pairCenterDistance2(pairHeldCenter(observedA), pairHeldCenter(observedB))
	radius := proofarith.DyAdd(m.RadiusA, m.RadiusB).Rat()
	radius2 := new(big.Rat).Mul(radius, radius)
	idealRelation := idealDistance2.Cmp(radius2)
	actualRelation := actualDistance2.Cmp(radius2)
	clearMargin := new(big.Rat).Add(radius, deviation)
	clearMargin.Mul(clearMargin, clearMargin)
	strictClear := actualDistance2.Cmp(clearMargin) > 0
	switch outcome {
	case reportvocab.SweepClear:
		if idealRelation <= 0 || !strictClear {
			return "rounded sphere-pair clear path loses its gap"
		}
	case reportvocab.SweepDepartedClear:
		if f.Sign() == 0 {
			if idealRelation != 0 || actualRelation != 0 {
				return "sphere-pair departure start is not touching"
			}
		} else if idealRelation <= 0 || !strictClear {
			return "rounded sphere-pair departure loses its gap"
		}
	case reportvocab.SweepPersistentTouch:
		if !hasTrack || idealRelation != 0 || actualRelation != 0 {
			return "sphere-pair replay loses its exact touch"
		}
	case reportvocab.SweepImpactBracket:
		if lo == nil || hi == nil {
			return "sphere-pair impact lacks its exact bracket"
		}
		if f.Cmp(lo) < 0 && (idealRelation <= 0 || !strictClear) {
			return "sphere-pair impact prefix is not clear"
		}
		if f.Cmp(hi) == 0 && (idealRelation > 0 || actualDistance2.Cmp(clearMargin) > 0) {
			return "sphere-pair impact right pose is not near contact"
		}
	case reportvocab.SweepGrazingTouch:
		if grazingAt == nil {
			return "sphere-pair graze lacks exact time"
		}
		if f.Cmp(grazingAt) == 0 {
			if idealRelation != 0 || actualRelation != 0 {
				return "rounded sphere-pair graze loses exact touch"
			}
		} else if idealRelation <= 0 || actualRelation <= 0 {
			return "rounded sphere-pair graze loses separation"
		}
	default:
		return "sphere-pair outcome has no replay proof"
	}
	return ""
}

func pairIdealCenter(center proofarith.DyV3, delta [3]proofarith.Dyadic, f *big.Rat) [3]*big.Rat {
	ideal := pairHeldCenter(center)
	for axis := range 3 {
		ideal[axis].Add(ideal[axis], new(big.Rat).Mul(delta[axis].Rat(), f))
	}
	return ideal
}

func pairHeldCenter(center proofarith.DyV3) [3]*big.Rat {
	var held [3]*big.Rat
	for axis := range 3 {
		held[axis] = center[axis].Rat()
	}
	return held
}

func pairCenterDistance2(a, b [3]*big.Rat) *big.Rat {
	squared := new(big.Rat)
	for axis := range 3 {
		delta := new(big.Rat).Sub(b[axis], a[axis])
		squared.Add(squared, new(big.Rat).Mul(delta, delta))
	}
	return squared
}
