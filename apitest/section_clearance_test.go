package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/section-outline-design.md §5's public-surface tests
// for SectionOutline.Clearance.

func clearanceFrame(t *testing.T) r3.Frame {
	t.Helper()
	return outlinePlane(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1))
}

// outlineLoop returns the rectangle [x0,x1] x [y0,y1] as a loop, counter-
// clockwise (an outer loop) or clockwise (a hole).
func outlineLoop(x0, y0, x1, y1 float64, hole bool) []r3.Vec {
	loop := []r3.Vec{r3.NewVec(x0, y0, 0), r3.NewVec(x1, y0, 0), r3.NewVec(x1, y1, 0), r3.NewVec(x0, y1, 0)}
	if hole {
		loop[1], loop[3] = loop[3], loop[1]
	}
	return loop
}

func outlineOf(t *testing.T, off float64, loops ...[]r3.Vec) decad.SectionOutline {
	t.Helper()
	return decad.SectionOutline{Plane: clearanceFrame(t), Loops: loops, OffSurface: units.Millimeters(off)}
}

func TestSectionClearanceOfTwoSquaresIsTheirGap(t *testing.T) {
	t.Parallel()
	a := outlineOf(t, 0, outlineLoop(0, 0, 10, 10, false))
	b := outlineOf(t, 0, outlineLoop(13, 2, 20, 8, false))
	got, err := a.Clearance(b)
	require.NoError(t, err)
	require.False(t, got.Overlaps)
	require.InDelta(t, 3, got.Distance.Value.Base(), 1e-12)
	require.Equal(t, decad.Approximate, got.Distance.Exactness)
	require.InDelta(t, 10, got.A.X, 1e-12)
	require.InDelta(t, 13, got.B.X, 1e-12)
	require.InDelta(t, got.A.Y, got.B.Y, 1e-12)
	require.Greater(t, got.Lower().Base(), 2.99)
}

func TestSectionClearanceOfACornerGapIsDiagonal(t *testing.T) {
	t.Parallel()
	a := outlineOf(t, 0, outlineLoop(0, 0, 10, 10, false))
	b := outlineOf(t, 0, outlineLoop(13, 14, 20, 20, false))
	got, err := a.Clearance(b)
	require.NoError(t, err)
	require.InDelta(t, 5, got.Distance.Value.Base(), 1e-12)
	require.InDelta(t, 10, got.A.X, 1e-12)
	require.InDelta(t, 10, got.A.Y, 1e-12)
	require.InDelta(t, 13, got.B.X, 1e-12)
	require.InDelta(t, 14, got.B.Y, 1e-12)
}

func TestSectionClearanceOfTouchingSquaresIsZeroWithoutOverlap(t *testing.T) {
	t.Parallel()
	a := outlineOf(t, 0, outlineLoop(0, 0, 10, 10, false))
	b := outlineOf(t, 0, outlineLoop(10, 0, 20, 10, false))
	got, err := a.Clearance(b)
	require.NoError(t, err)
	require.False(t, got.Overlaps)
	require.InDelta(t, 0, got.Distance.Value.Base(), 1e-12)
	require.Equal(t, 0.0, got.Lower().Base())
}

func TestSectionClearanceOfOverlappingSquaresOverlaps(t *testing.T) {
	t.Parallel()
	a := outlineOf(t, 0.01, outlineLoop(0, 0, 10, 10, false))
	b := outlineOf(t, 0.01, outlineLoop(5, 5, 15, 15, false))
	got, err := a.Clearance(b)
	require.NoError(t, err)
	require.True(t, got.Overlaps)
	require.Equal(t, 0.0, got.Distance.Value.Base())
	require.Equal(t, 0.0, got.Lower().Base())
	require.True(t, got.A.Equal(got.B, 1e-9), "the outlines cross at one point")
	atA := got.A.Equal(r3.NewVec(10, 5, 0), 1e-9)
	atB := got.A.Equal(r3.NewVec(5, 10, 0), 1e-9)
	require.True(t, atA || atB, "a corner of the overlap: %v", got.A)
}

func TestSectionClearanceOfASquareInsideASquareOverlaps(t *testing.T) {
	t.Parallel()
	a := outlineOf(t, 0, outlineLoop(0, 0, 10, 10, false))
	b := outlineOf(t, 0, outlineLoop(4, 4, 6, 6, false))
	for _, pair := range [][2]decad.SectionOutline{{a, b}, {b, a}} {
		got, err := pair[0].Clearance(pair[1])
		require.NoError(t, err)
		require.True(t, got.Overlaps)
		require.Equal(t, 0.0, got.Distance.Value.Base())
	}
}

func TestSectionClearanceOfASquareInAHoleIsTheHoleGap(t *testing.T) {
	t.Parallel()
	plate := outlineOf(t, 0, outlineLoop(0, 0, 20, 20, false), outlineLoop(5, 5, 15, 15, true))
	pin := outlineOf(t, 0, outlineLoop(7, 6, 12, 14, false))
	for _, pair := range [][2]decad.SectionOutline{{plate, pin}, {pin, plate}} {
		got, err := pair[0].Clearance(pair[1])
		require.NoError(t, err)
		require.False(t, got.Overlaps)
		require.InDelta(t, 1, got.Distance.Value.Base(), 1e-12, "the pin's y=6 side to the hole's y=5 side")
	}
	// A square larger than the hole has its material in the plate's.
	lid := outlineOf(t, 0, outlineLoop(3, 3, 17, 17, false))
	got, err := plate.Clearance(lid)
	require.NoError(t, err)
	require.True(t, got.Overlaps)
	// A square that fits in the hole's rim is in the plate's material.
	rim := outlineOf(t, 0, outlineLoop(1, 1, 3, 3, false))
	got, err = plate.Clearance(rim)
	require.NoError(t, err)
	require.True(t, got.Overlaps)
}

func TestSectionClearanceOfNestedLoops(t *testing.T) {
	t.Parallel()
	// A ring with an island: material in [0,30]^2 minus [5,25]^2, plus [10,20]^2.
	ring := outlineOf(t, 0, outlineLoop(0, 0, 30, 30, false), outlineLoop(5, 5, 25, 25, true),
		outlineLoop(10, 10, 20, 20, false))
	// In the gap between ring and island: clearance to the nearer wall.
	got, err := ring.Clearance(outlineOf(t, 0, outlineLoop(6, 12, 8, 14, false)))
	require.NoError(t, err)
	require.False(t, got.Overlaps)
	require.InDelta(t, 1, got.Distance.Value.Base(), 1e-12, "x=6 to the hole wall at x=5")
	// On top of the island: inside its material.
	got, err = ring.Clearance(outlineOf(t, 0, outlineLoop(14, 14, 16, 16, false)))
	require.NoError(t, err)
	require.True(t, got.Overlaps)
	// Outside the ring altogether.
	got, err = ring.Clearance(outlineOf(t, 0, outlineLoop(33, 0, 40, 5, false)))
	require.NoError(t, err)
	require.False(t, got.Overlaps)
	require.InDelta(t, 3, got.Distance.Value.Base(), 1e-12)
}

func TestSectionClearanceBoundSumsTheOffSurfaceBounds(t *testing.T) {
	t.Parallel()
	a := outlineOf(t, 0.25, outlineLoop(0, 0, 10, 10, false))
	b := outlineOf(t, 0.5, outlineLoop(11, 0, 20, 10, false))
	got, err := a.Clearance(b)
	require.NoError(t, err)
	require.InDelta(t, 1, got.Distance.Value.Base(), 1e-12)
	bound := got.Distance.Bound.Base()
	require.GreaterOrEqual(t, bound, 0.75, "no less than the OffSurface sum")
	require.Less(t, bound, 0.75+1e-6, "and only a few units of rounding beyond it")
	// The gap of 1 exceeds no bound of 0.75: certified. Widen the bound and it is not.
	require.InDelta(t, 1-bound, got.Lower().Base(), 1e-15)
	require.Positive(t, got.Lower().Base())

	wide := outlineOf(t, 0.5, outlineLoop(0, 0, 10, 10, false))
	got, err = wide.Clearance(b)
	require.NoError(t, err)
	require.GreaterOrEqual(t, got.Distance.Bound.Base(), 1.0)
	require.Equal(t, 0.0, got.Lower().Base(), "a bound that reaches the reading certifies no gap")
}

func TestSectionClearanceRefusals(t *testing.T) {
	t.Parallel()
	a := outlineOf(t, 0, outlineLoop(0, 0, 10, 10, false))
	u, v := a.Plane.U(), a.Plane.V()
	other, err := r3.NewFrame(r3.NewVec(0, 0, 1), u, v)
	require.NoError(t, err)
	shifted := decad.SectionOutline{Plane: other, Loops: a.Loops}
	nudged := a
	nudged.Plane, err = r3.NewFrame(r3.NewVec(1e-12, 0, 0), u, v)
	require.NoError(t, err)
	flipped, err := r3.NewFrame(r3.NewVec(0, 0, 0), v, u)
	require.NoError(t, err)

	for name, b := range map[string]decad.SectionOutline{
		"another origin":        shifted,
		"another axes":          {Plane: flipped, Loops: a.Loops},
		"no loop":               {Plane: a.Plane},
		"a two-point loop":      {Plane: a.Plane, Loops: [][]r3.Vec{a.Loops[0][:2]}},
		"a non-finite point":    {Plane: a.Plane, Loops: [][]r3.Vec{{r3.NewVec(0, 0, 0), r3.NewVec(math.NaN(), 1, 0), r3.NewVec(1, 1, 0)}}},
		"a negative OffSurface": {Plane: a.Plane, Loops: a.Loops, OffSurface: units.Millimeters(-1)},
		"the zero outline":      {},
	} {
		_, err := a.Clearance(b)
		require.ErrorIs(t, err, decad.ErrDegenerate, name)
	}
	_, err = a.Clearance(nudged)
	require.NoError(t, err, "planes within the stated tolerance are one plane")
}

func TestSectionClearanceOfTwoBoxesIsTheirGap(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	left := boxBody(t, doc, 0, 0, 20, 20, 10)
	right := boxBody(t, doc, 23.5, 4, 40, 16, 10)
	plane := outlinePlane(t, r3.NewVec(0, 0, 5), r3.NewVec(0, 0, 1))
	a, err := left.SectionOutline(t.Context(), plane, units.Millimeters(0.1))
	require.NoError(t, err)
	b, err := right.SectionOutline(t.Context(), plane, units.Millimeters(0.1))
	require.NoError(t, err)
	got, err := a.Clearance(b)
	require.NoError(t, err)
	require.False(t, got.Overlaps)
	require.InDelta(t, 3.5, got.Distance.Value.Base(), 1e-9)
	require.InDelta(t, 20, got.A.X, 1e-9)
	require.InDelta(t, 23.5, got.B.X, 1e-9)
	require.InDelta(t, 5, got.A.Z, 1e-9)
	require.Greater(t, got.Lower().Base(), 3.49, "a box's mesh departs from it by rounding alone")

	// Move the second box into the first: the outlines cross.
	inside := boxBody(t, doc, 18, 4, 40, 16, 10)
	c, err := inside.SectionOutline(t.Context(), plane, units.Millimeters(0.1))
	require.NoError(t, err)
	got, err = a.Clearance(c)
	require.NoError(t, err)
	require.True(t, got.Overlaps)
	require.Equal(t, 0.0, got.Distance.Value.Base())
}
