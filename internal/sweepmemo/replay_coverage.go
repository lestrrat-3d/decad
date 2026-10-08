package sweepmemo

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
)

// OrientedSphereRelationCovered checks the rounded sphere's relation to its
// certified face corridor at a replay fraction.
func OrientedSphereRelationCovered(outcome reportvocab.SweepOutcome, f, lo, hi *big.Rat,
	ideal, observed int) bool {
	switch outcome {
	case reportvocab.SweepClear:
		return ideal > 0 && observed > 0
	case reportvocab.SweepDepartedClear:
		if f.Sign() == 0 {
			return ideal == 0 && observed == 0
		}
		return ideal > 0 && observed > 0
	case reportvocab.SweepImpactBracket:
		if lo == nil || hi == nil {
			return false
		}
		if f.Cmp(lo) < 0 && (ideal <= 0 || observed <= 0) {
			return false
		}
		return f.Cmp(hi) != 0 || ideal <= 0
	default:
		return false
	}
}

// SphereRelationCovered checks the affine support gap against the reported
// outcome and the rounded source sphere's gap.
func SphereRelationCovered(outcome reportvocab.SweepOutcome, f, lo, hi, ideal, observed,
	resolution *big.Rat) bool {
	switch outcome {
	case reportvocab.SweepClear:
		return ideal.Sign() > 0 && observed.Sign() > 0
	case reportvocab.SweepDepartedClear:
		if f.Sign() == 0 {
			return ideal.Sign() == 0 && observed.Sign() == 0
		}
		return ideal.Sign() > 0 && observed.Sign() > 0
	case reportvocab.SweepPersistentTouch:
		return ideal.Sign() == 0 && new(big.Rat).Abs(observed).Cmp(resolution) <= 0
	case reportvocab.SweepImpactBracket:
		if lo == nil || hi == nil {
			return false
		}
		if f.Cmp(lo) < 0 && ideal.Sign() <= 0 {
			return false
		}
		return f.Cmp(hi) != 0 || ideal.Sign() <= 0
	default:
		return false
	}
}

// FractionCovered reports whether a replay fraction belongs to the certified
// prefix. A non-nil trackEnd marks a planar or rolling contact track.
func FractionCovered(outcome reportvocab.SweepOutcome, f, trackEnd, lo, hi, gap *big.Rat,
	rotating bool) bool {
	if trackEnd != nil {
		return f.Sign() >= 0 && f.Cmp(trackEnd) <= 0
	}
	if rotating && outcome == reportvocab.SweepImpactBracket {
		if gap != nil && hi != nil {
			return lo != nil && f.Cmp(hi) <= 0
		}
		return lo != nil && f.Cmp(lo) <= 0
	}
	switch outcome {
	case reportvocab.SweepClear, reportvocab.SweepDepartedClear,
		reportvocab.SweepPersistentTouch, reportvocab.SweepGrazingTouch:
		return true
	case reportvocab.SweepImpactBracket, reportvocab.SweepContactTransitionBracket:
		return hi != nil && f.Cmp(hi) <= 0
	default:
		return false
	}
}

// RelationCovered checks a rounded source-box relation against the sweep's
// certified relation and point resolution.
func RelationCovered(outcome reportvocab.SweepOutcome, f, lo, resolution *big.Rat,
	relation reportvocab.ContactRelation, a, b box.AxisBox) bool {
	switch outcome {
	case reportvocab.SweepClear:
		return relation == reportvocab.ContactSeparated
	case reportvocab.SweepDepartedClear:
		if f.Sign() == 0 {
			return relation == reportvocab.ContactTouching
		}
		return relation == reportvocab.ContactSeparated
	case reportvocab.SweepPersistentTouch:
		return relation == reportvocab.ContactTouching ||
			(relation == reportvocab.ContactSeparated || relation == reportvocab.ContactOverlapping) &&
				box.RelationDistanceWithin(a, b, resolution)
	case reportvocab.SweepImpactBracket:
		if lo == nil {
			return false
		}
		return relation == reportvocab.ContactSeparated || relation == reportvocab.ContactTouching ||
			relation == reportvocab.ContactOverlapping && box.RelationDistanceWithin(a, b, resolution)
	case reportvocab.SweepContactTransitionBracket:
		return relation == reportvocab.ContactTouching || relation == reportvocab.ContactSeparated ||
			relation == reportvocab.ContactOverlapping && box.RelationDistanceWithin(a, b, resolution)
	default:
		return false
	}
}

// BracketDepthWithin checks the maximum contact depth inside a rotating
// impact bracket after charging both bodies' travel and pose deviation.
func BracketDepthWithin(f, deviation, resolution, lo, hi, gap, travel *big.Rat) bool {
	if gap == nil || travel == nil || hi == nil || f.Cmp(hi) > 0 {
		return false
	}
	depth := new(big.Rat).Mul(new(big.Rat).Sub(f, lo), travel)
	depth.Sub(depth, gap)
	depth.Add(depth, deviation)
	return depth.Cmp(resolution) <= 0
}
