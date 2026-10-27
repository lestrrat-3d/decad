package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin agent-docs/general-boolean-design.md §3 "Flush cap
// contact" for a pin running exactly between two target faces, both caps
// flush: a pin bridging the gap of a U-shaped plate, and a pin from one wall
// of a plate to the opposite wall.

// uPlate is a 10 mm U-shaped plate: a base y in [0, 5] under two prongs, x in
// [0, 10] and [30, 40], up to y = 20. Its gap's walls x = 10 and x = 30 face
// each other, and it encloses 500 mm² × 10 = 5000 mm³.
func uPlate(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return polyPrismBody(t, doc, [][2]float64{{0, 0}, {40, 0}, {40, 20}, {30, 20}, {30, 5}, {10, 5}, {10, 20}, {0, 20}}, 10)
}

// TestClassBBridgingPinUnion bridges the U's gap with a Ø6 pin from x = 10 to
// 30, drawn from either wall, with either operand first. Both walls take the
// pin's section as a hole and the pin adds its wall alone: 11 faces, volume
// 5000 + 180π.
func TestClassBBridgingPinUnion(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name     string
		planeX   float64
		dir      decad.Direction
		pinFirst bool
	}{
		{name: "along", planeX: 10, dir: decad.Along},
		{name: "along_pin_first", planeX: 10, dir: decad.Along, pinFirst: true},
		{name: "against", planeX: 30, dir: decad.Against},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			a, b := uPlate(t, doc), pinFromYZ(t, doc, tc.planeX, tc.dir, 20)
			if tc.pinFirst {
				a, b = b, a
			}
			got, err := decad.Union(t.Context(), a, b)
			require.NoError(t, err)
			requireExactFlushResult(t, got, 11, 5000+180*math.Pi)
		})
	}
}

// TestClassBThroughPinCutAndIntersect runs a Ø6 pin from the 40×20×10
// plate's wall x = 0 to its wall x = 40, drawn from either wall. Cut drills a
// through hole: the plate's six faces, two holed, and the hole's wall, 7
// faces, 8000 − 360π. Intersect is the pin itself with either operand first:
// 3 faces, 360π.
func TestClassBThroughPinCutAndIntersect(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		planeX float64
		dir    decad.Direction
	}{
		{name: "along", planeX: 0, dir: decad.Along},
		{name: "against", planeX: 40, dir: decad.Against},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			got, err := decad.Cut(t.Context(), boxBody(t, doc, 0, 0, 40, 20, 10), pinFromYZ(t, doc, tc.planeX, tc.dir, 40))
			require.NoError(t, err)
			requireExactFlushResult(t, got, 7, 8000-360*math.Pi)
		})
	}
	for _, pinFirst := range []bool{false, true} {
		doc := decad.New()
		a, b := boxBody(t, doc, 0, 0, 40, 20, 10), pinAlongX(t, doc, 0, 40)
		if pinFirst {
			a, b = b, a
		}
		got, err := decad.Intersect(t.Context(), a, b)
		require.NoError(t, err)
		requireExactFlushResult(t, got, 3, 360*math.Pi)
	}
}

// TestClassBBridgingPinOneUlpShort stops the bridging pin one ulp short of
// the far wall x = 30. Its tube meets the near wall alone, so the one-face
// flush reach stands it on that wall with its far cap kept: 12 faces, volume
// 5000 + 9π·L for the pin's recorded length L.
func TestClassBBridgingPinOneUlpShort(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	length := math.Nextafter(20, 0)
	got, err := decad.Union(t.Context(), uPlate(t, doc), pinAlongX(t, doc, 10, length))
	require.NoError(t, err)
	requireExactFlushResult(t, got, 12, 5000+9*math.Pi*length)
}

// TestClassBBridgingPinMisses keeps on the mesh path every pair between two
// faces that neither the flush reach nor the sweep walk builds: a pin whose
// section crosses the plate's top, a Cut of the bridging pin (the plate
// whole), and a Union of the through pin (the plate itself).
func TestClassBBridgingPinMisses(t *testing.T) {
	t.Parallel()
	type pair func(*testing.T, *decad.Document) (*decad.Body, *decad.Body)
	union := func(t *testing.T, a, b *decad.Body) (*decad.Body, error) { return decad.Union(t.Context(), a, b) }
	cut := func(t *testing.T, a, b *decad.Body) (*decad.Body, error) { return decad.Cut(t.Context(), a, b) }
	uAnd := func(x0 float64, length units.Value, z float64) pair {
		return func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return uPlate(t, doc), pinAlongXAt(t, doc, x0, z, length)
		}
	}
	for _, tc := range []struct {
		name string
		pair pair
		op   func(*testing.T, *decad.Body, *decad.Body) (*decad.Body, error)
		want float64
	}{
		{name: "union_section_crossing_top", pair: uAnd(10, units.Millimeters(20), 8), op: union, want: 5000 + 180*math.Pi},
		{name: "union_of_through_pin", pair: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return boxBody(t, doc, 0, 0, 40, 20, 10), pinAlongX(t, doc, 0, 40)
		}, op: union, want: 8000},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			a, b := tc.pair(t, doc)
			got, err := tc.op(t, a, b)
			require.NoError(t, err)
			require.True(t, tookMeshPath(got))
			volume, err := got.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-tc.want), boundMM3(t, volume))
		})
	}
	// A pin bridging the gap meets the plate in its two end disks alone. The
	// mesh path decides its Cut and Intersect.
	for _, op := range []func(*testing.T, *decad.Body, *decad.Body) (*decad.Body, error){
		cut,
		func(t *testing.T, a, b *decad.Body) (*decad.Body, error) { return decad.Intersect(t.Context(), a, b) },
	} {
		doc := decad.New()
		a, b := uAnd(10, units.Millimeters(20), 5)(t, doc)
		got, err := op(t, a, b)
		require.Nil(t, got)
		var boolean *decad.BooleanError
		require.ErrorAs(t, err, &boolean)
		require.Equal(t, decad.BooleanUnsupportedContact, boolean.Code)
	}
}
