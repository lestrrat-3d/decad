package prismcells

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Reexpression maps operand B's recorded section into operand A's frame.
// Delta accumulates the largest rounding allowance of any mapped point.
// Identity copies the record without computing new coordinates. A reflection
// reverses loop winding, so callers must rewind B's scene before classifying it.
type Reexpression struct {
	relative  prismplacement.Relative
	Identity  bool
	Reflected bool
	Delta     float64
}

// NewReexpression composes the two placements once, so each recorded point
// rounds at the operands' relative offset rather than at their world position.
func NewReexpression(a, b prismplacement.Operand) (*Reexpression, error) {
	relative, err := prismplacement.Compose(a, b)
	if err != nil {
		return nil, err
	}
	return &Reexpression{
		relative:  relative,
		Identity:  relative.Identity,
		Reflected: relative.Reflection,
	}, nil
}

// Reflection reports whether the relative map reverses loop winding.
func (re *Reexpression) Reflection() bool { return re.Reflected }

// MapPoint maps one plane-local point and charges the coordinate rounding.
// The composed map rounds at the relative offset, avoiding separate trips
// through world coordinates. prismplacement.Point charges RigidRoundAllow at
// the input coordinate and composed translation, covering both composition
// and application: roughly 48 ulps of input and 40 of translation remain below
// its allowances of roughly 110 and 55 ulps, respectively.
func (re *Reexpression) MapPoint(p sectionrecord.Point2) sectionrecord.Point2 {
	if re.Identity {
		return p
	}
	return prismplacement.Point(re.relative, &re.Delta, p)
}

// Rewind maps a reflected region and reverses each loop's walk order.
// Outer and hole indices stay fixed. RewindLoop preserves the per-kind walk
// rules, and a narrowed line's walked endpoints add their own charge.
func (re *Reexpression) Rewind(budget *proofbound.WorkBudget, region SceneProfile) (SceneProfile, float64, error) {
	mapPoint := func(p sectionrecord.Point2) (sectionrecord.Point2, error) { return re.MapPoint(p), nil }
	outer, charge, err := RewindLoop(budget, region.Outer, mapPoint)
	if err != nil {
		return SceneProfile{}, 0, err
	}
	result := SceneProfile{Outer: outer}
	for _, hole := range region.Holes {
		rewoundHole, holeCharge, err := RewindLoop(budget, hole, mapPoint)
		if err != nil {
			return SceneProfile{}, 0, err
		}
		charge = math.Max(charge, holeCharge)
		result.Holes = append(result.Holes, rewoundHole)
	}
	return result, charge, nil
}
