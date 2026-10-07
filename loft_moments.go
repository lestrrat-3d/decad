package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"

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

func (m *loftMassAccumulator) addTriangle(a, b, c r3.Vec, wall bool, indices [3]int, distances []loftmesh.LoftVertexDistance) {
	m.AddTriangle(a, b, c, wall, indices, distances)
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

// computeLoftChordedAllow maps built loft cells to the neutral chord proof.
func computeLoftChordedAllow(pairs []loftLoopPair, vIdx, wIdx [][]int, verts []r3.Vec, anchor r3.Vec, matchedDelta, delta, distUpper float64, reversed bool) (loftmesh.LoftChordedAllow, error) {
	// Derive cap1's offset before lifting any wall cell into exact rationals.
	// Production has already refused non-finite vertices at S13, while direct
	// internal callers still receive S14's existing derivation refusal instead
	// of reaching freeform.MustRatOf with a NaN.
	// h1Upper (cap1's own offset from anchor) is bounded by the distance to
	// the CLOSEST held cap1 vertex to anchor, never an arbitrary one: a
	// plane's own perpendicular offset from a point is at most the distance
	// to ANY point on that plane, so the minimum over every held vertex is
	// the tightest such bound this evaluator can read off the assembly
	// without a fresh plane-distance computation of its own.
	//
	// Either half of that reading failing is §5.2's cap planeOffsetUpper row
	// answering +Inf, and this function REFUSES on it rather than publishing
	// a number: a vertex whose coordinates ratSquaredDistance3 cannot read as
	// exact rationals (a non-finite coordinate) states no distance at all, and
	// an assembly whose every cap1 vertex overflows proofbound.RatSqrtUp leaves the
	// minimum at +Inf, as does an assembly stating no cap1 vertex. §5.2's own
	// closing rule — an enclosure the record cannot state answers +Inf and the
	// build refuses at Table S row S14, "never a finite substitute and never a
	// published zero" — is what forbids the obvious alternative of assigning
	// h1Upper = 0 here. That zero is not a bound: proofbound.CapAreaVolumeAllow takes its
	// planeOffsetUpper <= 0 arm on it and publishes capVolumeUpper = 0, the
	// SMALLEST possible number standing in for a quantity this evaluator could
	// not derive, in a term every consumer reads as an upper bound. Nothing
	// about the surrounding legs is allowed to excuse it: whether a sibling leg
	// happens to saturate on the same assembly is that leg's own business, and
	// a bound may not rest on another term's value to stay sound.
	h1Upper := math.Inf(1)
	for _, row := range wIdx {
		for _, idx := range row {
			v := verts[idx]
			d2 := ratSquaredDistance3(anchor.X, anchor.Y, anchor.Z, v.X, v.Y, v.Z)
			if d2 == nil {
				return loftmesh.LoftChordedAllow{}, loftmesh.ErrLoftCapOffsetUnderivable
			}
			h1Upper = math.Min(h1Upper, proofbound.RatSqrtUp(d2))
		}
	}
	if proofbound.IsNonFinite(h1Upper) {
		return loftmesh.LoftChordedAllow{}, loftmesh.ErrLoftCapOffsetUnderivable
	}

	neutral := make([]loftmesh.LoftChordPair, len(pairs))
	for i, p := range pairs {
		neutral[i] = loftmesh.LoftChordPair{
			Cells: len(p.v), ArcUpperV: p.arcUpperV, ArcUpperW: p.arcUpperW,
			MatchedDelta: p.matchedDelta, TangentEnergyV: p.tangentEnergyV,
			TangentEnergyW: p.tangentEnergyW,
		}
	}
	return loftmesh.ComputeLoftChordedAllow(
		neutral, vIdx, wIdx, verts, anchor, h1Upper, matchedDelta, delta, distUpper, reversed,
	), nil
}
