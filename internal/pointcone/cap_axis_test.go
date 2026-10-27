package pointcone

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// cellGeometryAxisRat is the general-axis cell geometry that converts each
// A-scaled value through Axis.ScaledDown on a big.Rat, re-reading the norm
// bracket per value. It is the oracle cellGeometryAxis must match bit for bit.
func cellGeometryAxisRat(tri [3]r3.Vec, vertexReach [3]float64, axis coneaxis.Axis) CellGeom {
	reach, h := 0.0, 0.0
	sMin := axis.Axial(tri[0])
	for i, p := range tri {
		sMin = proofarith.DyMin(sMin, axis.Axial(p))
		reach = max(reach, vertexReach[i])
		h = max(h, proofbound.DvLenUpper(proofbound.HeldDelta(p, tri[(i+1)%3])))
	}
	geom := CellGeom{reach: reach, h: h}
	xMin, ok := axis.ScaledDown(sMin.Rat())
	if !ok {
		return geom
	}
	sigma, ok := axis.RadialLower(tri)
	if !ok {
		return geom
	}
	rhoMin, ok := axis.ScaledDown(proofarith.FloatRat(sigma))
	if !ok {
		return geom
	}
	geom.xMin, geom.rhoMin = xMin, rhoMin
	return geom
}

// capGeometryAxisTestAxes are general axes in a coordinate plane, one whose
// norm bracket overflows (every conversion refuses), and one outside every
// coordinate plane (the radial bound refuses).
var capGeometryAxisTestAxes = []r3.Vec{
	{X: 1, Y: 2}, {Y: 1}, {X: 3, Y: 4}, {X: 1, Z: 2}, {X: 2}, {X: -0.5, Z: 3},
	{X: 1e-300, Y: 3e-300}, {X: 1e300, Z: 7e299},
	{X: math.MaxFloat64, Y: math.MaxFloat64}, {X: 1, Y: 1, Z: 1},
}

// TestCellGeometryAxisMatchesRatConversion holds the general-axis cell
// geometry to the big.Rat conversion bit for bit, on a subdivided fan and on
// random cells spread across scales, about every test axis. Under the race
// detector the fan is subdivided twice instead of three times and it draws
// 150 random cells instead of 600.
func TestCellGeometryAxisMatchesRatConversion(t *testing.T) {
	t.Parallel()
	rounds, cells := 3, 600
	if raceDetector {
		rounds, cells = 2, 150
	}
	r := rand.New(rand.NewPCG(53, 59))
	far, caps := capTestFan(t, rounds)
	randomFar, randomCaps := capTestRandom(r, cells)
	for i := range randomFar {
		// Spread the random vertices across the axis and across scales, so
		// axial minima of both signs and every binade reach the conversion.
		scale := math.Ldexp(1, r.IntN(80)-40)
		randomFar[i] = r3.Vec{X: (randomFar[i].X - 3) * scale, Y: (randomFar[i].Y - 2) * scale, Z: randomFar[i].Z * scale}
	}
	fans := []struct {
		far  []r3.Vec
		caps [][3]int
	}{{far, caps}, {randomFar, randomCaps}}
	kinds := map[string]int{}
	for _, v := range capGeometryAxisTestAxes {
		axis := mustAxis(t, v)
		for _, f := range fans {
			got := capGeometry(f.far, f.caps, axis, 1)
			require.Len(t, got, len(f.caps))
			for k, tri := range f.caps {
				cell := [3]r3.Vec{f.far[tri[0]], f.far[tri[1]], f.far[tri[2]]}
				var reach [3]float64
				for i, p := range cell {
					reach[i] = proofbound.DvLenUpper(proofbound.HeldDelta(p, r3.Vec{}))
				}
				want := cellGeometryAxisRat(cell, reach, axis)
				requireSameGeom(t, []CellGeom{want}, got[k:k+1])
				requireSameGeom(t, []CellGeom{want}, []CellGeom{CellGeometry(cell, axis)})
				switch {
				case want.rhoMin == 0:
					kinds["refused"]++
				case want.xMin < 0:
					kinds["negative"]++
				default:
					kinds["positive"]++
				}
			}
			requireSameGeom(t, got, capGeometry(f.far, f.caps, axis, 4))
		}
	}
	require.Positive(t, kinds["refused"])
	require.Positive(t, kinds["negative"])
	require.Positive(t, kinds["positive"])

	// Cells whose millimetre axial minimum or radial bound overflows while the
	// other conversion succeeds: each refuses with xMin and rhoMin both zero.
	for _, c := range []struct {
		name string
		axis r3.Vec
		tri  [3]r3.Vec
	}{
		{"axial overflow", r3.Vec{X: 0x1p-10, Y: 0x1p-10},
			[3]r3.Vec{{X: 1.5e308, Y: 1.5e308, Z: 1}, {X: 1.5e308, Y: 1.5e308, Z: 2}, {X: 1.6e308, Y: 1.5e308, Z: 1}}},
		{"radial overflow", r3.Vec{X: 0x1p-10},
			[3]r3.Vec{{X: 1, Y: 1.5e308, Z: 1.5e308}, {X: 2, Y: 1.5e308, Z: 1.5e308}, {X: 1, Y: 1.6e308, Z: 1.5e308}}},
	} {
		axis := mustAxis(t, c.axis)
		want := CellGeometry(c.tri, axis)
		var reach [3]float64
		for i, p := range c.tri {
			reach[i] = proofbound.DvLenUpper(proofbound.HeldDelta(p, r3.Vec{}))
		}
		requireSameGeom(t, []CellGeom{cellGeometryAxisRat(c.tri, reach, axis)}, []CellGeom{want})
		require.Zero(t, want.xMin, c.name)
		require.Zero(t, want.rhoMin, c.name)
		require.Positive(t, want.h, c.name)
	}
}

// BenchmarkCapGeometryGeneralAxis reads the cell geometry of a subdivided fan
// about the irrational-length axis (1, 2, 0) on one goroutine, the
// cellGeometryAxis path every general-axis cap takes.
func BenchmarkCapGeometryGeneralAxis(b *testing.B) {
	far, caps := capTestFan(b, 4)
	axis := mustAxis(b, r3.Vec{X: 1, Y: 2})
	b.ReportAllocs()
	for b.Loop() {
		_ = capGeometry(far, caps, axis, 1)
	}
}
