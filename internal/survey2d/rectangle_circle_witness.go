package survey2d

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// RectangleCircleWitness proves that a disk larger than the requested wall
// fits a rectangular section with circular holes. It tries nine exact-rational
// grid centers. A failed search leaves the complete inradius survey to decide.
// radiusInMM converts the recorded radius and its conversion displacement.
func RectangleCircleWitness(budget *proofbound.WorkBudget, holesRecord []sectionrecord.LoopRecord,
	loops [][]SideWalk, thickness, thicknessDelta, tol float64,
	radiusInMM func(units.Value) (float64, float64, error)) (bool, error) {
	if len(loops) == 0 || len(loops[0]) != 4 {
		return false, nil
	}
	outer := loops[0]
	minX, maxX := outer[0].StartU, outer[0].StartU
	minY, maxY := outer[0].StartV, outer[0].StartV
	for _, side := range outer {
		w := side.SegmentWalk
		if w.Kind != WalkLine || w.StartBound != (proofbound.WalkEndBound{}) || w.EndBound != (proofbound.WalkEndBound{}) {
			return false, nil
		}
		minX = math.Min(minX, w.StartU)
		maxX = math.Max(maxX, w.StartU)
		minY = math.Min(minY, w.StartV)
		maxY = math.Max(maxY, w.StartV)
	}
	if !(minX < maxX && minY < maxY) ||
		proofbound.IsNonFinite(minX) || proofbound.IsNonFinite(maxX) || proofbound.IsNonFinite(minY) || proofbound.IsNonFinite(maxY) {
		return false, nil
	}
	var sides uint8
	for _, side := range outer {
		w := side.SegmentWalk
		var bit uint8
		switch {
		case w.StartU == minX && w.EndU == minX &&
			((w.StartV == minY && w.EndV == maxY) || (w.StartV == maxY && w.EndV == minY)):
			bit = 1
		case w.StartU == maxX && w.EndU == maxX &&
			((w.StartV == minY && w.EndV == maxY) || (w.StartV == maxY && w.EndV == minY)):
			bit = 2
		case w.StartV == minY && w.EndV == minY &&
			((w.StartU == minX && w.EndU == maxX) || (w.StartU == maxX && w.EndU == minX)):
			bit = 4
		case w.StartV == maxY && w.EndV == maxY &&
			((w.StartU == minX && w.EndU == maxX) || (w.StartU == maxX && w.EndU == minX)):
			bit = 8
		default:
			return false, nil
		}
		if sides&bit != 0 {
			return false, nil
		}
		sides |= bit
	}
	if sides != 15 {
		return false, nil
	}
	type circle struct{ x, y, radius *big.Rat }
	holes := make([]circle, 0, len(loops)-1)
	for i, loop := range loops[1:] {
		if len(loop) != 1 || i >= len(holesRecord) || len(holesRecord[i].Segments) != 1 {
			return false, nil
		}
		segment, ok := holesRecord[i].Segments[0].(sectionrecord.CircleSeg)
		if !ok || segment.CCW {
			return false, nil
		}
		w := loop[0].SegmentWalk
		if w.Kind != WalkCircular || !w.Closed || w.RadiusBound != 0 ||
			proofbound.IsNonFinite(w.CU) || proofbound.IsNonFinite(w.CV) || proofbound.IsNonFinite(w.Radius) || w.Radius <= 0 {
			return false, nil
		}
		radius, radiusDelta, err := radiusInMM(segment.Radius)
		if err != nil {
			return false, err
		}
		if radius != w.Radius || proofbound.IsNonFinite(radiusDelta) {
			return false, nil
		}
		radiusUpper := new(big.Rat).Add(proofarith.FloatRat(radius), proofarith.FloatRat(radiusDelta))
		holes = append(holes, circle{proofarith.FloatRat(w.CU), proofarith.FloatRat(w.CV), radiusUpper})
	}
	xlo, xhi, ylo, yhi := proofarith.FloatRat(minX), proofarith.FloatRat(maxX), proofarith.FloatRat(minY), proofarith.FloatRat(maxY)
	width := new(big.Rat).Sub(xhi, xlo)
	height := new(big.Rat).Sub(yhi, ylo)
	upper := new(big.Rat).Set(width)
	if height.Cmp(upper) < 0 {
		upper.Set(height)
	}
	upper.Quo(upper, big.NewRat(2, 1))
	if upper.Cmp(big.NewRat(1, 1)) < 0 {
		upper.SetInt64(1)
	}
	// Inradius is at most half the rectangle's narrower side. This threshold
	// therefore includes the full tolerance margin even though the true
	// inradius has not been computed.
	need := new(big.Rat).Add(proofarith.FloatRat(thickness), proofarith.FloatRat(thicknessDelta))
	need.Add(need, new(big.Rat).Mul(proofarith.FloatRat(tol), upper))
	quarters := [...]*big.Rat{big.NewRat(1, 4), big.NewRat(1, 2), big.NewRat(3, 4)}
	for _, u := range quarters {
		x := new(big.Rat).Add(xlo, new(big.Rat).Mul(width, u))
		for _, v := range quarters {
			if err := WallBudgetStep(budget); err != nil {
				return false, err
			}
			y := new(big.Rat).Add(ylo, new(big.Rat).Mul(height, v))
			fits := true
			for _, edge := range []*big.Rat{
				new(big.Rat).Sub(x, xlo), new(big.Rat).Sub(xhi, x),
				new(big.Rat).Sub(y, ylo), new(big.Rat).Sub(yhi, y),
			} {
				if edge.Cmp(need) <= 0 {
					fits = false
					break
				}
			}
			if !fits {
				continue
			}
			for _, hole := range holes {
				if err := WallBudgetStep(budget); err != nil {
					return false, err
				}
				dx, dy := new(big.Rat).Sub(x, hole.x), new(big.Rat).Sub(y, hole.y)
				distance2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
				separation := new(big.Rat).Add(hole.radius, need)
				if distance2.Cmp(new(big.Rat).Mul(separation, separation)) <= 0 {
					fits = false
					break
				}
			}
			if fits {
				return true, WallBudgetErr(budget)
			}
		}
	}
	return false, WallBudgetErr(budget)
}
