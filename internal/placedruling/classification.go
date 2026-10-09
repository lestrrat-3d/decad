package placedruling

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Relation is the result of a placed cylinder's support-plane proof.
type Relation uint8

const (
	Separated Relation = iota + 1
	Touching
	Band
)

// Classification contains the exact gap interval or the outward band width.
// Lateral bounds the distance of each true lowest rim point from its ideal foot.
type Classification struct {
	Relation     Relation
	GapLo, GapHi *big.Rat
	BandWidth    float64
	Lateral      *big.Rat
}

// Classify proves a relation after Support has selected a face. A deep
// crossing or a rim whose foot can leave that face has no classification.
func Classify(c Cylinder, plane Plane) (Classification, bool) {
	exact := c.Gram.Sign() == 0 && plane.Alpha.Sign() == 0
	lateral := new(big.Rat)
	if !exact {
		lateral = c.RimDrift(plane.Alpha.Rat())
	}
	if !footInside(c, plane, lateral) {
		return Classification{}, false
	}
	low := minDy(plane.Heights[0], plane.Heights[1])
	// The least height is low + r·(1 − ρ), where ρ = |P·Bᵀn̂|.
	// An exact pose has ρ = 1; otherwise the square-root bounds enclose it.
	sigmaLo, sigmaHi := low.Rat(), low.Rat()
	slack := new(big.Rat)
	if !exact {
		var rhoSquared proofarith.Dyadic
		for k := range 3 {
			if k != c.Axis {
				component := proofarith.DvDot(plane.Normal, c.Columns[k])
				rhoSquared = proofarith.DyAdd(rhoSquared, proofarith.DyMul(component, component))
			}
		}
		rhoLo, rhoHi := proofarith.DySqrtDown(rhoSquared), proofarith.DySqrtUp(rhoSquared)
		if !finite(rhoLo, rhoHi) {
			return Classification{}, false
		}
		r := c.Radius.Rat()
		sigmaLo = new(big.Rat).Sub(proofbound.RatAdd(sigmaLo, r), proofbound.RatMul(r, proofarith.FloatRat(rhoHi)))
		sigmaHi = new(big.Rat).Sub(proofbound.RatAdd(sigmaHi, r), proofbound.RatMul(r, proofarith.FloatRat(rhoLo)))
		slack = proofbound.RatAdd(c.SectionDrift(plane.Alpha).Rat(),
			proofbound.RatMul(new(big.Rat).Abs(plane.Alpha.Rat()), c.Length.Rat()))
	}
	switch {
	case sigmaLo.Cmp(slack) > 0:
		// Material in front of a face-local plane is at least Clearance away.
		// The gap's lower end must include that nearer material.
		if plane.Clearance != nil {
			sigmaLo = proofbound.RatMin(sigmaLo, plane.Clearance)
		}
		return Classification{Relation: Separated, GapLo: sigmaLo, GapHi: sigmaHi, Lateral: lateral}, true
	case sigmaHi.Cmp(new(big.Rat).Neg(slack)) < 0:
		return Classification{}, false
	case exact:
		if !clearsBand(plane, new(big.Rat)) {
			return Classification{}, false
		}
		return Classification{Relation: Touching, Lateral: lateral}, true
	default:
		band := proofbound.RatAdd(maxDy(proofarith.DyAbs(plane.Heights[0]),
			proofarith.DyAbs(plane.Heights[1])).Rat(), c.SectionDrift(plane.Alpha).Rat())
		width := proofbound.RatFloatUp(band)
		if !finite(width) || !clearsBand(plane, proofarith.FloatRat(width)) {
			return Classification{}, false
		}
		return Classification{Relation: Band, BandWidth: width, Lateral: lateral}, true
	}
}

func footInside(c Cylinder, plane Plane, lateral *big.Rat) bool {
	var lo, hi [2]*big.Rat
	for slot, axis := range [2]int{(plane.Face.Drop + 1) % 3, (plane.Face.Drop + 2) % 3} {
		a, b := c.Centers[0][axis].Rat(), c.Centers[1][axis].Rat()
		lo[slot] = new(big.Rat).Sub(proofbound.RatMin(a, b), lateral)
		hi[slot] = new(big.Rat).Add(proofbound.RatMax(a, b), lateral)
	}
	return plane.Face.HoldsBox(lo, hi)
}

func clearsBand(plane Plane, width *big.Rat) bool {
	return plane.Clearance == nil || plane.Clearance.Cmp(width) > 0
}

func finite(values ...float64) bool {
	for _, v := range values {
		if math.IsInf(v, 0) || math.IsNaN(v) {
			return false
		}
	}
	return true
}

func minDy(a, b proofarith.Dyadic) proofarith.Dyadic {
	if proofarith.DyCmp(a, b) <= 0 {
		return a
	}
	return b
}
