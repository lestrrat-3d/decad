package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin agent-docs/general-boolean-design.md §3 "Pin across
// several faces": a Ø6 pin whose tube meets several faces of the target, or
// two in a pattern the through and flush reaches do not build, read layer by
// layer along its sweep.

// slottedPlate is a 10 mm plate x in [0, 40], y in [0, 20] with a slot
// x in [15, 25] from y = 5 up: 650 mm² × 10 = 6500 mm³. A pin along x at
// y = 10 crosses its walls x = 0, 15, 25 and 40.
func slottedPlate(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return polyPrismBody(t, doc, [][2]float64{{0, 0}, {40, 0}, {40, 20}, {25, 20}, {25, 5}, {15, 5}, {15, 20}, {0, 20}}, 10)
}

// TestClassBPinIntoFarProng bridges the U plate's gap with a pin from the
// near prong's inner wall x = 10 to x = 35, 5 mm into the far prong, drawn
// from either end. Union holes both inner walls and adds the pin's wall over
// the gap: 11 faces, 5000 + 180π. Cut keeps the near wall whole and drills a
// blind hole 5 deep into the far prong: 12 faces, 5000 − 45π. Intersect is
// the pin over [30, 35]: 3 faces, 45π. Union and Intersect take either
// operand first.
func TestClassBPinIntoFarProng(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name     string
		planeX   float64
		dir      decad.Direction
		pinFirst bool
	}{
		{name: "along", planeX: 10, dir: decad.Along},
		{name: "along_pin_first", planeX: 10, dir: decad.Along, pinFirst: true},
		{name: "against", planeX: 35, dir: decad.Against},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			pair := func(doc *decad.Document) (*decad.Body, *decad.Body) {
				a, b := uPlate(t, doc), pinFromYZ(t, doc, tc.planeX, tc.dir, 25)
				if tc.pinFirst {
					return b, a
				}
				return a, b
			}
			a, b := pair(decad.New())
			union, err := decad.Union(t.Context(), a, b)
			require.NoError(t, err)
			requireExactFlushResult(t, union, 11, 5000+180*math.Pi)

			a, b = pair(decad.New())
			common, err := decad.Intersect(t.Context(), a, b)
			require.NoError(t, err)
			requireExactFlushResult(t, common, 3, 45*math.Pi)
			if tc.pinFirst {
				return
			}
			a, b = pair(decad.New())
			cut, err := decad.Cut(t.Context(), a, b)
			require.NoError(t, err)
			requireExactFlushResult(t, cut, 12, 5000-45*math.Pi)
		})
	}
	// One ulp past the far wall the pin still meets it strictly inside its
	// interval: Union reads as above, and Cut drills a blind hole one ulp deep.
	long := math.Nextafter(20, 21)
	doc := decad.New()
	union, err := decad.Union(t.Context(), uPlate(t, doc), pinAlongX(t, doc, 10, long))
	require.NoError(t, err)
	requireExactFlushResult(t, union, 11, 5000+180*math.Pi)

	doc = decad.New()
	cut, err := decad.Cut(t.Context(), uPlate(t, doc), pinAlongX(t, doc, 10, long))
	require.NoError(t, err)
	requireExactFlushResult(t, cut, 12, 5000-9*math.Pi*(long-20))
}

// TestClassBPinToFarOuterWall runs the pin from x = 10 to the far prong's
// outer wall x = 40, and one ulp short of and past it. Exactly at the wall
// its far cap is flush: Union keeps that wall whole (11 faces, 5000 + 180π),
// Cut keeps the near wall whole and drills through the far prong (11 faces,
// 5000 − 90π), and Intersect is the pin over [30, 40] (90π). One ulp short,
// the outer wall is apart from the tube and Cut leaves a blind hole with a
// floor (12 faces), and Union reads as at the wall with the outer wall
// untouched. One ulp long, the outer wall lies strictly inside the tube: Cut
// and Intersect read the same solids as at the wall, and Union holes the
// outer wall and adds a one-ulp stub of the pin with its cap (13 faces).
func TestClassBPinToFarOuterWall(t *testing.T) {
	t.Parallel()
	short, long := math.Nextafter(40, 0), math.Nextafter(40, 41)
	pin := func(doc *decad.Document, end float64) *decad.Body {
		return pinAlongX(t, doc, 10, end-10)
	}
	t.Run("flush", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		union, err := decad.Union(t.Context(), uPlate(t, doc), pin(doc, 40))
		require.NoError(t, err)
		requireExactFlushResult(t, union, 11, 5000+180*math.Pi)

		doc = decad.New()
		cut, err := decad.Cut(t.Context(), uPlate(t, doc), pin(doc, 40))
		require.NoError(t, err)
		requireExactFlushResult(t, cut, 11, 5000-90*math.Pi)

		doc = decad.New()
		common, err := decad.Intersect(t.Context(), uPlate(t, doc), pin(doc, 40))
		require.NoError(t, err)
		requireExactFlushResult(t, common, 3, 90*math.Pi)
	})
	t.Run("one_ulp_short", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		cut, err := decad.Cut(t.Context(), uPlate(t, doc), pin(doc, short))
		require.NoError(t, err)
		requireExactFlushResult(t, cut, 12, 5000-9*math.Pi*(short-30))

		doc = decad.New()
		union, err := decad.Union(t.Context(), uPlate(t, doc), pin(doc, short))
		require.NoError(t, err)
		requireExactFlushResult(t, union, 11, 5000+180*math.Pi)
	})
	t.Run("one_ulp_long", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		cut, err := decad.Cut(t.Context(), uPlate(t, doc), pin(doc, long))
		require.NoError(t, err)
		requireExactFlushResult(t, cut, 11, 5000-90*math.Pi)

		doc = decad.New()
		common, err := decad.Intersect(t.Context(), uPlate(t, doc), pin(doc, long))
		require.NoError(t, err)
		requireExactFlushResult(t, common, 3, 90*math.Pi)

		doc = decad.New()
		union, err := decad.Union(t.Context(), uPlate(t, doc), pin(doc, long))
		require.NoError(t, err)
		requireExactFlushResult(t, union, 13, 5000+9*math.Pi*(20+(long-40)))
	})
}

// TestClassBPinThroughSlottedPlate runs a pin along x through the slotted
// plate's four walls. From x = −5 to 45, Union holes all four and adds the
// pin's wall over [−5, 0], [15, 25] and [40, 45] with both caps: 15 faces,
// 6500 + 180π. Cut drills both lands: 12 faces, 6500 − 270π. From x = 0 to
// 40, both caps flush, Union keeps the outer walls whole and bridges the
// slot: 11 faces, 6500 + 90π. Cut holes all four walls: 12 faces,
// 6500 − 270π.
func TestClassBPinThroughSlottedPlate(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name        string
		x0, length  float64
		unionFaces  int
		unionVolume float64
	}{
		{name: "past_both_walls", x0: -5, length: 50, unionFaces: 15, unionVolume: 6500 + 180*math.Pi},
		{name: "flush_both_walls", x0: 0, length: 40, unionFaces: 11, unionVolume: 6500 + 90*math.Pi},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			union, err := decad.Union(t.Context(), slottedPlate(t, doc), pinAlongX(t, doc, tc.x0, tc.length))
			require.NoError(t, err)
			requireExactFlushResult(t, union, tc.unionFaces, tc.unionVolume)

			doc = decad.New()
			cut, err := decad.Cut(t.Context(), slottedPlate(t, doc), pinAlongX(t, doc, tc.x0, tc.length))
			require.NoError(t, err)
			requireExactFlushResult(t, cut, 12, 6500-270*math.Pi)
		})
	}
}

// TestClassBMultiFacePinMisses keeps on the mesh path every several-face pair
// the sweep walk does not build: an Intersect through both of the slotted
// plate's lands (two separate prisms), a far cap displaced by an inch length
// that converts to 25 mm only after rounding, and a section crossing the U
// plate's top. Over the far prong that section rises 1 mm above the top, a
// circular segment of area 9·acos(2/3) − 2√5 along 5 mm.
func TestClassBMultiFacePinMisses(t *testing.T) {
	t.Parallel()
	type pair func(*testing.T, *decad.Document) (*decad.Body, *decad.Body)
	union := func(t *testing.T, a, b *decad.Body) (*decad.Body, error) { return decad.Union(t.Context(), a, b) }
	for _, tc := range []struct {
		name string
		pair pair
		op   func(*testing.T, *decad.Body, *decad.Body) (*decad.Body, error)
		want float64
	}{
		{name: "intersect_two_lands", pair: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return slottedPlate(t, doc), pinAlongX(t, doc, -5, 50)
		}, op: func(t *testing.T, a, b *decad.Body) (*decad.Body, error) {
			return decad.Intersect(t.Context(), a, b)
		}, want: 270 * math.Pi},
		{name: "union_displaced_far_cap", pair: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return uPlate(t, doc), pinAlongXAt(t, doc, 10, 5, units.Inches(25.0/25.4))
		}, op: union, want: 5000 + 180*math.Pi},
		{name: "union_section_crossing_top", pair: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return uPlate(t, doc), pinAlongXAt(t, doc, 10, 8, units.Millimeters(25))
		}, op: union, want: 5000 + 180*math.Pi + 5*(9*math.Acos(2.0/3)-2*math.Sqrt(5))},
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
}
