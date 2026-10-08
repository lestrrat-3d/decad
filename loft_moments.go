package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file adapts internal/loftmesh's mass sums to decad's bounded
// measurements and maps built loft cells to chord proofs.
// See docs/loft-design.md §5.2, §8 and §12.

// loftMassAccumulator adapts loftmesh's exact mass sums to body readings.
type loftMassAccumulator struct {
	*loftmesh.MassAccumulator
}

func newLoftMassAccumulator(anchor r3.Vec, delta, sectionDelta, sectionMatchedDelta float64) *loftMassAccumulator {
	return &loftMassAccumulator{MassAccumulator: loftmesh.NewMassAccumulator(anchor, delta, sectionDelta, sectionMatchedDelta)}
}

func (m *loftMassAccumulator) add(a, b, c r3.Vec, wall bool) {
	m.Add(a, b, c, wall)
}

func (m *loftMassAccumulator) volume(verts []r3.Vec, tris [][3]int) Measurement {
	value, bound := m.Volume(verts, tris)
	return Measurement{
		Value: units.CubicMillimeters(value), Exactness: exactnessOf(bound),
		Bound: units.CubicMillimeters(bound),
	}
}

func (m *loftMassAccumulator) centroid(verts []r3.Vec, tris [][3]int) (VecMeasurement, error) {
	value, bound, err := m.Centroid(verts, tris)
	if err != nil {
		return VecMeasurement{}, err
	}
	return VecMeasurement{
		Value: value, Exactness: exactnessOf(bound), Bound: units.Millimeters(bound),
	}, nil
}

func (m *loftMassAccumulator) bounds() (Box, bool) {
	lo, hi, bound, ok := m.Bounds()
	if !ok {
		return Box{}, false
	}
	return Box{Min: lo, Max: hi, Exactness: exactnessOf(bound), Bound: units.Millimeters(bound)}, true
}

func (m *loftMassAccumulator) area(capAreas ...*big.Rat) Measurement {
	value, bound := m.Area(capAreas...)
	return Measurement{
		Value: units.SquareMillimeters(value), Exactness: Approximate,
		Bound: units.SquareMillimeters(bound),
	}
}
