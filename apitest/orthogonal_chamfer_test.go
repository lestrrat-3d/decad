package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md B17's public-surface tests: a chamfer of several convex edges of
// a mesh solid that share corners, which the retained blend route refuses, is the solid less the wedge of each edge.

// The volume the chamfer of a 20 x 20 x 10 box removes is the wedges' ½·d²·L along each chosen edge less what two or three
// of them share at a corner: ∫∫min(a, b) = d³/3 for two wedges, and for the three of a corner d³/4 of the corner cube d³
// is kept, so the union holds 3d³/4 where the three wedges separately hold 3d³/2.

func TestChamferOfEightEdgesSharingCornersRemovesTheirWedges(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	b := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10))
	// The eight edges of the top and bottom faces are 20 mm long; each corner of the box has two of them.
	out, err := decad.Chamfer(t.Context(), b, decad.Edges(decad.Convex(), decad.LongerThan(units.Millimeters(15))), units.Millimeters(1))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	volume, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, 4000-(0.5*8*20-8.0/3), volume.Value.Base(), volume.Bound.Base()+1e-9)
	box, err := out.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 10, box.Max.Z, 1e-6)
	// The chamfer plane is the points whose distances from the top face and the front face add to 1: a point 0.2 mm from
	// the top and 0.2 from the front is removed, one 0.2 from the top and 0.9 from the front is material, and so is one
	// far from the edge.
	for point, want := range map[r3.Vec]decad.PointLocation{
		r3.NewVec(10, 0.2, 9.8): decad.PointOutside,
		r3.NewVec(10, 0.9, 9.8): decad.PointInside,
		r3.NewVec(10, 0.8, 9.9): decad.PointOutside,
		r3.NewVec(10, 0.2, 5):   decad.PointInside,
	} {
		got, err := out.LocatePoint(t.Context(), point, units.Millimeters(0.05))
		require.NoError(t, err)
		require.Equal(t, want, got, "%v", point)
	}
}

func TestChamferOfEveryEdgeOfABoxLeavesTheCornersAsCutByThreePlanes(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	b := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10))
	out, err := decad.Chamfer(t.Context(), b, decad.Edges(decad.Convex()), units.Millimeters(1))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	// 12 wedges: ½·(4·20 + 4·20 + 4·10) = 100, less six d³ for the eight corners where three meet: 4000 - 94.
	require.InDelta(t, 4000-(100.0-6), volume.Value.Base(), volume.Bound.Base()+1e-9)
}

func TestChamferOfEdgesSharingCornersRefusesWhatItWouldOvercut(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	b := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10))
	// A setback deeper than the 10 mm the box is tall: the wedge would be wider than the material behind its edge.
	_, err := decad.Chamfer(t.Context(), b, decad.Edges(decad.Convex(), decad.LongerThan(units.Millimeters(15))), units.Millimeters(11))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{b}, doc.Bodies(), "a refusal leaves the box live")
}

func TestChamferOfEdgesMeetingAtAReflexCornerIsRefused(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	// An L: a 20 x 10 slab and a 10 x 20 slab, 10 mm tall. The two top edges at its inside corner (10, 10, 10) run on
	// into material past that corner, so a wedge longer than the edge would cut more than the wedge.
	a := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 10, 10))
	c := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 20, 10))
	l, err := decad.Union(t.Context(), a, c)
	require.NoError(t, err)
	sel := decad.Edges(decad.Convex(), decad.EndpointAt(r3.NewVec(10, 10, 10)))
	_, err = decad.Chamfer(t.Context(), l, sel, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{l}, doc.Bodies(), "a refusal leaves the L live")
}

func pocketSlab(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	slab := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(30, 30, 10))
	tool := blockIn(t, doc, r3.NewVec(10, 10, 5), r3.NewVec(20, 20, 12))
	out, err := decad.Cut(t.Context(), slab, tool)
	require.NoError(t, err)
	return out
}

func TestChamferOfAPocketsFloorEdgesFillsTheirWedges(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	pocket := pocketSlab(t, doc)
	// The four 10 mm floor edges end in the pocket's walls; the 5 mm wall corners end at the open mouth.
	sel := decad.Edges(decad.Concave(), decad.LongerThan(units.Millimeters(7)))
	out, err := decad.Chamfer(t.Context(), pocket, sel, units.Millimeters(1))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	volume, err := out.Volume()
	require.NoError(t, err)
	// The 10 x 10 x 5 pocket leaves 9000 - 500. Four wedges of 5 mm³ add 20, less d³/3 shared at each of the four floor
	// corners where two wedges meet.
	require.InDelta(t, 8500+20-4.0/3, volume.Value.Base(), volume.Bound.Base()+1e-9)
	// Beside the pocket's x = 10 wall, 0.2 mm above the floor and 0.2 in from the wall is now material, 0.9 in from it
	// is still void, and the pocket's middle is void.
	for point, want := range map[r3.Vec]decad.PointLocation{
		r3.NewVec(10.2, 15, 5.2): decad.PointInside,
		r3.NewVec(10.9, 15, 5.2): decad.PointOutside,
		r3.NewVec(15, 15, 5.2):   decad.PointOutside,
	} {
		got, err := out.LocatePoint(t.Context(), point, units.Millimeters(0.05))
		require.NoError(t, err)
		require.Equal(t, want, got, "%v", point)
	}
}

func TestChamferOfEveryEdgeOfAPocketStopsFlushAtTheMouth(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	pocket := pocketSlab(t, doc)
	// The four wall corners end at the pocket's mouth, where the fill's tool stops flush with the slab's top; the
	// four floor edges run on into the walls. All eight together are 8500 + 30 − 3 mm³, as in the floor-only case less
	// what the three wedges meeting at each floor corner share.
	out, err := decad.Chamfer(t.Context(), pocket, decad.Edges(decad.Concave()), units.Millimeters(1))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	volume, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, 8527, volume.Value.Base(), volume.Bound.Base()+1e-6)
	require.Less(t, volume.Bound.Base(), 0.1, "the flush ends are charged their shift and no more")
	box, err := out.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 10, box.Max.Z, 0.01, "nothing stands above the slab")
}

func TestChamferOfAGrooveRunsFlushOutOfBothEnds(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	slab := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(30, 30, 10))
	tool := blockIn(t, doc, r3.NewVec(-1, 10, 5), r3.NewVec(31, 20, 12))
	groove, err := decad.Cut(t.Context(), slab, tool)
	require.NoError(t, err)
	out, err := decad.Chamfer(t.Context(), groove, decad.Edges(decad.Concave(), decad.LongerThan(units.Millimeters(20))),
		units.Millimeters(1))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	// The 30 x 10 x 5 groove leaves 7500; the two 30 mm floor edges add a wedge of 15 each.
	require.InDelta(t, 7530, volume.Value.Base(), volume.Bound.Base()+1e-6)
	require.Less(t, volume.Bound.Base(), 0.1)
}

func TestChamferOfConcaveEdgesLargerThanTheirWallsIsRefused(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	pocket := pocketSlab(t, doc)
	// The pocket's walls are 5 mm tall, so a 6 mm fill up the wall would stand above the slab.
	_, err := decad.Chamfer(t.Context(), pocket, decad.Edges(decad.Concave(), decad.LongerThan(units.Millimeters(7))),
		units.Millimeters(6))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{pocket}, doc.Bodies(), "a refusal leaves the pocket live")
}
