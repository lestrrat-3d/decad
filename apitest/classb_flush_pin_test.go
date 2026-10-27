package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin agent-docs/general-boolean-design.md §3 "Flush cap
// contact": a pin whose end cap lies in a wall of the target builds class B's
// brep, and a contact the flush reach does not cover keeps the mesh path.

// bossAlongZ extrudes a Ø6 circle about (x, y) from z0 up by length.
func bossAlongZ(t *testing.T, doc *decad.Document, x, y, z0, length float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(x, y)
	s.Fix(center)
	s.CreateCircle(center, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(length), Dir: decad.Along,
	})
	require.NoError(t, err)
	return body
}

// pinAlongXAt is pinAlongX with the section centre at height z and the
// length stated as a units.Value.
func pinAlongXAt(t *testing.T, doc *decad.Document, x0, z float64, length units.Value) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.YZ(), x0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(10, z)
	s.Fix(center)
	s.CreateCircle(center, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: length, Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// wantRounding covers the rounding of a closed form such as 8000 + 135π
// evaluated in float64: two operations, each within one ulp of its result.
func wantRounding(want float64) float64 {
	return 2 * (math.Nextafter(math.Abs(want), math.Inf(1)) - math.Abs(want))
}

// requireExactFlushResult asserts the class-B brep reading of a flush pair:
// no faceted face, faceCount faces, every edge on two faces, a volume within
// its bound of want, a mesh whose occupied volume VerifyAll proves, and a
// Sound document.
func requireExactFlushResult(t *testing.T, got *decad.Body, faceCount int, want float64) {
	t.Helper()
	require.False(t, tookMeshPath(got))
	require.Len(t, got.Faces(), faceCount)
	for _, f := range got.Faces() {
		_, faceted := f.Surface().(decad.Faceted)
		require.False(t, faceted)
	}
	requireEveryEdgeOnTwoFaces(t, got)
	volume, err := got.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), boundMM3(t, volume)+wantRounding(want))
	require.Less(t, boundMM3(t, volume), 1e-9)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	requireWatertight(t, mesh)
	report, err := got.Document().Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

// TestClassBFlushPinUnion stands a Ø6 pin on the 40×20×10 plate's wall
// x = 40, from x = 40 to 55. The wall takes the pin's section as a hole and
// the pin adds its wall and far cap: 8 faces, volume 8000 + 135π, with either
// operand first.
func TestClassBFlushPinUnion(t *testing.T) {
	t.Parallel()
	for _, pinFirst := range []bool{false, true} {
		doc := decad.New()
		plate := boxBody(t, doc, 0, 0, 40, 20, 10)
		pin := pinAlongX(t, doc, 40, 15)
		a, b := plate, pin
		if pinFirst {
			a, b = pin, plate
		}
		got, err := decad.Union(t.Context(), a, b)
		require.NoError(t, err)
		requireExactFlushResult(t, got, 8, 8000+135*math.Pi)
	}
}

// TestClassBFlushPinCutAndIntersect runs the pin from x = 30 to the wall
// x = 40, its end cap flush with that wall from inside. Cut opens a blind
// hole in the wall: 8 faces, 8000 − 90π. Intersect is the pin itself: 3
// faces, 90π.
func TestClassBFlushPinCutAndIntersect(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 40, 20, 10)
	pin := pinAlongX(t, doc, 30, 10)
	cut, err := decad.Cut(t.Context(), plate, pin)
	require.NoError(t, err)
	requireExactFlushResult(t, cut, 8, 8000-90*math.Pi)

	doc = decad.New()
	plate = boxBody(t, doc, 0, 0, 40, 20, 10)
	pin = pinAlongX(t, doc, 30, 10)
	common, err := decad.Intersect(t.Context(), plate, pin)
	require.NoError(t, err)
	requireExactFlushResult(t, common, 3, 90*math.Pi)
}

// TestClassBFlushBossOnBrep stands a Ø6 boss along z on the top of a plate
// already cross-drilled along x, a class-B brep: 9 faces, volume
// 8000 − 360π + 135π.
func TestClassBFlushBossOnBrep(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 40, 20, 10)
	drilled, err := decad.Cut(t.Context(), plate, pinAlongX(t, doc, -5, 50))
	require.NoError(t, err)
	require.False(t, tookMeshPath(drilled))
	got, err := decad.Union(t.Context(), drilled, bossAlongZ(t, doc, 20, 10, 10, 15))
	require.NoError(t, err)
	requireExactFlushResult(t, got, 9, 8000-225*math.Pi)
}

// notchedPlate is an L-shaped 10 mm plate: x in [0, 40] for y in [0, 20] and
// x in [0, 25] for y in [20, 40]. Its wall x = 25 lies far from a pin at
// y = 10, in the plane of that pin's cap at x = 25.
func notchedPlate(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return polyPrismBody(t, doc, [][2]float64{{0, 0}, {40, 0}, {40, 20}, {25, 20}, {25, 40}, {0, 40}}, 10)
}

// TestClassBFlushPinBothCapsInTargetPlanes runs the pin from x = 25 to the
// wall x = 40 of the notched plate. Both caps lie in planes of the plate's
// walls, but only the cap at x = 40 meets a face, so the Cut builds the blind
// hole: 10 faces, 13000 − 135π.
func TestClassBFlushPinBothCapsInTargetPlanes(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	got, err := decad.Cut(t.Context(), notchedPlate(t, doc), pinAlongX(t, doc, 25, 15))
	require.NoError(t, err)
	requireExactFlushResult(t, got, 10, 13000-135*math.Pi)
}

// TestClassBFlushPinMisses keeps on the mesh path every flush pair whose
// solid the flush reach does not build: a pin inside the plate unioned, a pin
// outside it cut or intersected, a far cap plane that is not the contact, a
// pin one ulp off the wall, a section touching the plate's top, and a pin
// whose flush cap carries a level
// displacement (an inch length that converts to exactly 10 mm only after
// rounding).
func TestClassBFlushPinMisses(t *testing.T) {
	t.Parallel()
	type pair func(*testing.T, *decad.Document) (*decad.Body, *decad.Body)
	plateAnd := func(x0, length float64) pair {
		return func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return boxBody(t, doc, 0, 0, 40, 20, 10), pinAlongX(t, doc, x0, length)
		}
	}
	union := func(t *testing.T, a, b *decad.Body) (*decad.Body, error) { return decad.Union(t.Context(), a, b) }
	cut := func(t *testing.T, a, b *decad.Body) (*decad.Body, error) { return decad.Cut(t.Context(), a, b) }
	for _, tc := range []struct {
		name string
		pair pair
		op   func(*testing.T, *decad.Body, *decad.Body) (*decad.Body, error)
		want float64
	}{
		{name: "union_of_pin_inside", pair: plateAnd(30, 10), op: union, want: 8000},
		{name: "cut_of_pin_outside", pair: plateAnd(40, 15), op: cut, want: 8000},
		{name: "union_with_far_cap_plane", pair: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return notchedPlate(t, doc), pinAlongX(t, doc, 25, 15)
		}, op: union, want: 13000},
		{name: "union_one_ulp_off", pair: plateAnd(math.Nextafter(40, 41), 15), op: union, want: 8000 + 135*math.Pi},
		{name: "union_section_touching_top", pair: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return boxBody(t, doc, 0, 0, 40, 20, 10), pinAlongXAt(t, doc, 40, 7, units.Millimeters(15))
		}, op: union, want: 8000 + 135*math.Pi},
		{name: "cut_with_displaced_cap", pair: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			return boxBody(t, doc, 0, 0, 40, 20, 10), pinAlongXAt(t, doc, 30, 5, units.Inches(10.0/25.4))
		}, op: cut, want: 8000 - 90*math.Pi},
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
	// A pin outside the plate shares only the disk with it.
	doc := decad.New()
	common, err := decad.Intersect(t.Context(), boxBody(t, doc, 0, 0, 40, 20, 10), pinAlongX(t, doc, 40, 15))
	require.Nil(t, common)
	var boolean *decad.BooleanError
	require.ErrorAs(t, err, &boolean)
	require.Equal(t, decad.BooleanEmpty, boolean.Code)
}
