package tessellation

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// LoftCapBandFiniteMotion replaces only the two cap-level copies of each
// reflex connector station. The caller appends one motion entry with each
// vertex, so its vertex indices also index motion.
func LoftCapBandFiniteMotion(arcCount, capArcStart, capLoV, capHiV []int,
	capPointCount int, vertices []r3.Vec, motion []float64) (float64, error) {
	if len(motion) != len(vertices) || len(capLoV) != capPointCount ||
		len(capHiV) != capPointCount {
		return 0, fmt.Errorf("%w: loft cap-band motion does not match its vertices", decaderr.ErrUnsupported)
	}
	reflex := make([]bool, len(motion))
	for i, count := range arcCount {
		if count == 0 {
			continue
		}
		start := capArcStart[i]
		if start < 0 || start+count > capPointCount {
			return 0, fmt.Errorf("%w: loft cap-band reflex stations are incomplete", decaderr.ErrUnsupported)
		}
		for k := start; k < start+count; k++ {
			lo, hi := capLoV[k], capHiV[k]
			if lo < 0 || lo >= len(motion) || hi < 0 || hi >= len(motion) || lo == hi || reflex[lo] || reflex[hi] {
				return 0, fmt.Errorf("%w: loft cap-band reflex vertices are incomplete", decaderr.ErrUnsupported)
			}
			reflex[lo], reflex[hi] = true, true
		}
	}
	worst := 0.0
	for i, value := range motion {
		if reflex[i] {
			if !math.IsInf(value, 1) {
				return 0, fmt.Errorf("%w: loft cap-band reflex motion is not an apex sample", decaderr.ErrUnsupported)
			}
			motion[i] = 0
			continue
		}
		if proofbound.IsNonFinite(value) || value < 0 {
			return 0, fmt.Errorf("%w: loft cap-band vertex motion is not finite", decaderr.ErrUnsupported)
		}
		worst = math.Max(worst, value)
	}
	return worst, nil
}

// LoftCapBandSlabVolume bounds the uncertain part by two axial slabs inside
// one cylinder. A world-space sphere enclosing the body and mesh gives a
// radius for the cylinder around any sweep axis, including after placement.
// The body box's own error is charged at each coordinate; the mesh vertices
// are the exact held points. Both cap levels are widened by their inherited
// axial error and the mesh's full displacement, so the slab includes all
// floating-point level motion as well as every reflex apex fan.
func LoftCapBandSlabVolume(ctx context.Context, center, boxMin, boxMax r3.Vec,
	boxError float64, vertices []r3.Vec, axialDelta, meshBound, startAxial, endAxial float64) (float64, error) {
	axisMax := 0.0
	include := func(p [3]float64, errorBound float64) {
		coords := [3]float64{center.X, center.Y, center.Z}
		for i := range p {
			diff := proofbound.BoundedSub(proofbound.MeasuredScalar(p[i], errorBound),
				proofbound.ExactScalar(coords[i]))
			axisMax = math.Max(axisMax, proofbound.AbsSumUpper(diff.Value, diff.Bound))
		}
	}
	include([3]float64{boxMin.X, boxMin.Y, boxMin.Z}, boxError)
	include([3]float64{boxMax.X, boxMax.Y, boxMax.Z}, boxError)
	for i, p := range vertices {
		if i%1024 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, err
			}
		}
		include([3]float64{p.X, p.Y, p.Z}, 0)
	}
	radius := proofbound.Radius3D(axisMax)
	// The true band and its mesh can each move on both sides of a nominal
	// level. Two copies of the inherited axial and tessellation allowances
	// cover those opposite motions before the two cap slabs are summed.
	levelError := proofbound.AbsSumUpper(axialDelta, meshBound)
	startHeight := proofbound.AbsSumUpper(startAxial, proofbound.ProductUpper(2, levelError))
	endHeight := proofbound.AbsSumUpper(endAxial, proofbound.ProductUpper(2, levelError))
	area := proofbound.ProductUpper(4, proofbound.ProductUpper(radius, radius))
	volume := proofbound.ProductUpper(area, proofbound.AbsSumUpper(startHeight, endHeight))
	if proofbound.IsNonFinite(volume) {
		return 0, fmt.Errorf("%w: loft cap-band slab has no finite volume bound", decaderr.ErrUnsupported)
	}
	return volume, nil
}
