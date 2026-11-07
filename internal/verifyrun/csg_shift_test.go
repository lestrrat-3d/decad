package verifyrun

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func testBox(lo, hi r3.Vec, bound float64) measurement.Box {
	return measurement.Box{Min: lo, Max: hi, Bound: units.Millimeters(bound)}
}

func testShift(x, y, z float64) shift {
	return shift{v: &[3]*big.Rat{ratOf(x), ratOf(y), ratOf(z)}}
}

// TestBoxesDisjointShifted holds the shifted box test to the unshifted one: it
// agrees with measurement.BoxesDisjoint at zero shifts, boxes whose inflated
// ends touch are disjoint, and boxes that overlap by one float step are not,
// with and without shifts.
func TestBoxesDisjointShifted(t *testing.T) {
	t.Parallel()
	unit := testBox(r3.Vec{}, r3.Vec{X: 1, Y: 1, Z: 1}, 0)
	for _, tc := range []struct {
		name   string
		b      measurement.Box
		sa, sb shift
		apart  bool
	}{
		{"zero shifts, apart", testBox(r3.Vec{X: 2}, r3.Vec{X: 3, Y: 1, Z: 1}, 0), shift{}, shift{}, true},
		{"zero shifts, touching", testBox(r3.Vec{X: 1}, r3.Vec{X: 2, Y: 1, Z: 1}, 0), shift{}, shift{}, true},
		{"zero shifts, one step of overlap", testBox(r3.Vec{X: 1 - 0x1p-53}, r3.Vec{X: 2, Y: 1, Z: 1}, 0), shift{}, shift{}, false},
		{"zero shifts, overlapping", testBox(r3.Vec{X: 0.5}, r3.Vec{X: 2, Y: 1, Z: 1}, 0), shift{}, shift{}, false},
		{"a bound reaches into the other box", testBox(r3.Vec{X: 1.25}, r3.Vec{X: 2, Y: 1, Z: 1}, 0.5), shift{}, shift{}, false},
		{"bounds just touch", testBox(r3.Vec{X: 1.5}, r3.Vec{X: 2, Y: 1, Z: 1}, 0.5), shift{}, shift{}, true},
		{"shift moves b clear", testBox(r3.Vec{}, r3.Vec{X: 1, Y: 1, Z: 1}, 0), shift{}, testShift(1, 0, 0), true},
		{"shift moves b into a", testBox(r3.Vec{X: 2}, r3.Vec{X: 3, Y: 1, Z: 1}, 0), shift{}, testShift(-1.5, 0, 0), false},
		{"shift moves a clear, other axis", testBox(r3.Vec{}, r3.Vec{X: 1, Y: 1, Z: 1}, 0), testShift(0, 0, -1), shift{}, true},
		{"shifts equal, boxes overlapping", testBox(r3.Vec{X: 0.5}, r3.Vec{X: 2, Y: 1, Z: 1}, 0), testShift(5, 5, 5), testShift(5, 5, 5), false},
		{"shift leaves one step of overlap", testBox(r3.Vec{X: 2}, r3.Vec{X: 3, Y: 1, Z: 1}, 0), shift{}, testShift(-1-0x1p-52, 0, 0), false},
		{"shift leaves exact touching", testBox(r3.Vec{X: 2}, r3.Vec{X: 3, Y: 1, Z: 1}, 0), shift{}, testShift(-1, 0, 0), true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.apart, boxesDisjointShifted(unit, tc.sa, tc.b, tc.sb))
			// Disjointness is symmetric.
			require.Equal(t, tc.apart, boxesDisjointShifted(tc.b, tc.sb, unit, tc.sa))
			if !tc.sa.equal(tc.sb) {
				return
			}
			// Equal shifts move both boxes alike, so they agree with the raw test.
			require.Equal(t, measurement.BoxesDisjoint(unit, tc.b), boxesDisjointShifted(unit, tc.sa, tc.b, tc.sb))
		})
	}
}
