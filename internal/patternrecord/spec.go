package patternrecord

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func finiteVec(v r3.Vec) bool {
	return !math.IsNaN(v.X) && !math.IsNaN(v.Y) && !math.IsNaN(v.Z) &&
		!math.IsInf(v.X, 0) && !math.IsInf(v.Y, 0) && !math.IsInf(v.Z, 0)
}

// ResolveLinear validates the data of a linear pattern.
func ResolveLinear(dir r3.Vec, step units.Value, count int) (Spec, error) {
	if count < 2 {
		return Spec{}, fmt.Errorf(`%w: a pattern needs a Count of at least 2, got %d (a pattern of one is the receiver)`,
			decaderr.ErrDegenerate, count)
	}
	if !finiteVec(dir) {
		return Spec{}, fmt.Errorf(`%w: the pattern direction %v is not finite`, decaderr.ErrNotFinite, dir)
	}
	if dir == (r3.Vec{}) {
		return Spec{}, fmt.Errorf(`%w: a zero pattern direction names no line`, decaderr.ErrDegenerate)
	}
	stepMM, err := sectionrecord.MagnitudeIn(step, units.Length, units.Millimeter, "the pattern step")
	if err != nil {
		return Spec{}, err
	}
	if stepMM == 0 {
		return Spec{}, fmt.Errorf(`%w: a zero pattern step stacks every instance on the receiver`, decaderr.ErrDegenerate)
	}
	exact := extent.ExactConversion(step, units.Millimeter)
	if exact == nil {
		return Spec{}, fmt.Errorf(`%w: the pattern step is not representable`, decaderr.ErrNotFinite)
	}
	return Spec{Count: count, Dir: dir, StepMM: stepMM, StepRat: exact}, nil
}

// ResolveCircular validates the data of a circular pattern.
func ResolveCircular(center, axis r3.Vec, count int) (Spec, error) {
	if count < 2 {
		return Spec{}, fmt.Errorf(`%w: a pattern needs a Count of at least 2, got %d (a pattern of one is the receiver)`,
			decaderr.ErrDegenerate, count)
	}
	if !finiteVec(center) || !finiteVec(axis) {
		return Spec{}, fmt.Errorf(`%w: the pattern axis (%v, %v) is not finite`,
			decaderr.ErrNotFinite, center, axis)
	}
	if axis == (r3.Vec{}) {
		return Spec{}, fmt.Errorf(`%w: a zero pattern axis names no rotation`, decaderr.ErrDegenerate)
	}
	return Spec{Count: count, Circular: true, Center: center, Axis: axis}, nil
}

// WorldMotion is instance i's rigid motion in world space.
func (rp Spec) WorldMotion(i int) (r3.Transform, error) {
	if rp.Circular {
		if rp.Center == (r3.Vec{}) && rp.Axis.X == 0 && rp.Axis.Y == 0 && rp.Axis.Z != 0 {
			if motion, ok := exactAxialMotion(rp, i); ok {
				return motion, nil
			}
		}
		return r3.RotationAround(rp.Center, rp.Axis, units.Degrees(360*float64(i)/float64(rp.Count)))
	}
	k := float64(i) * rp.StepMM / rp.Dir.Len()
	return r3.Translation(rp.Dir.Scale(k))
}

// exactAxialMotion rebuilds an origin-centred Z rotation with an exact axial
// basis. Rodrigues' formula can round the unchanged Z axis away from +Z.
func exactAxialMotion(rp Spec, i int) (r3.Transform, bool) {
	angle := units.Degrees(360 * float64(i) / float64(rp.Count))
	requested, err := r3.RotationAround(rp.Center, rp.Axis, angle)
	if err != nil {
		return r3.Transform{}, false
	}
	requestedBasis := requested.Basis()
	cos, sin := requestedBasis.EX.X, requestedBasis.EX.Y
	// FromBasis normalizes columns separately. Keep only a nearby pair that
	// survives construction with the exact symmetry the radial proof requires.
	for radius := range 17 {
		for dc := -radius; dc <= radius; dc++ {
			for ds := -radius; ds <= radius; ds++ {
				if max(absSteps(dc), absSteps(ds)) != radius {
					continue
				}
				c, s := floatSteps(cos, dc), floatSteps(sin, ds)
				basis := r3.Basis{
					EX: r3.Vec{X: c, Y: s},
					EY: r3.Vec{X: -s, Y: c},
					EZ: r3.Vec{Z: 1},
				}
				if (r3.Vec{X: c, Y: s}).Len() != 1 {
					continue
				}
				motion, err := r3.FromBasis(basis, r3.Vec{})
				if err != nil || !hasExactAxialBasis(motion) {
					continue
				}
				return motion, true
			}
		}
	}
	return r3.Transform{}, false
}

func hasExactAxialBasis(motion r3.Transform) bool {
	if !motion.IsValid() || motion.IsReflection() || motion.Translation() != (r3.Vec{}) {
		return false
	}
	b := motion.Basis()
	return b.EZ == (r3.Vec{Z: 1}) && b.EX.Z == 0 && b.EY.Z == 0 &&
		b.EX.X == b.EY.Y && b.EX.Y == -b.EY.X
}
func floatSteps(value float64, steps int) float64 {
	direction := math.Inf(1)
	if steps < 0 {
		direction = math.Inf(-1)
		steps = -steps
	}
	for ; steps > 0; steps-- {
		value = math.Nextafter(value, direction)
	}
	return value
}

func absSteps(value int) int {
	if value < 0 {
		return -value
	}
	return value
}
