package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/retained-edge-blend-design.md's mitre-run tests: a fillet of straight convex edges that meet
// end to end on one face of a mesh box, the cylinders meeting in the plane that bisects the two walls.

// topEdge selects the top edge of the 20 x 20 x 10 block that starts at p and runs along dir.
func topEdge(p, dir r3.Vec) *decad.EdgeQuery {
	return decad.Edges(decad.EndpointAt(p), decad.ParallelTo(dir))
}

// filletRemoved is the volume a fillet of radius r takes from the box's top edges, by summing the thickness each
// column of a 1000 x 1000 grid loses: the height above the arc at the column's distance d from the nearest selected
// wall, and for a column near two selected walls the larger of the two (the mitre between the cylinders).
func filletRemoved(r float64, nearX0, nearX20, nearY0, nearY20 bool) float64 {
	thick := func(d float64) float64 {
		if d >= r {
			return 0
		}
		return r - math.Sqrt(r*r-(r-d)*(r-d))
	}
	const n = 1000
	h := 20.0 / n
	total := 0.0
	for i := range n {
		x := (float64(i) + 0.5) * h
		tx := 0.0
		if nearX0 {
			tx = math.Max(tx, thick(x))
		}
		if nearX20 {
			tx = math.Max(tx, thick(20-x))
		}
		for j := range n {
			y := (float64(j) + 0.5) * h
			ty := 0.0
			if nearY0 {
				ty = math.Max(ty, thick(y))
			}
			if nearY20 {
				ty = math.Max(ty, thick(20-y))
			}
			total += math.Max(tx, ty) * h * h
		}
	}
	return total
}

func TestFilletOfTheTopLoopOfAMeshBoxMeetsInMitres(t *testing.T) {
	t.Parallel()
	x, y := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	loop := topEdge(r3.NewVec(0, 0, 10), x).Or(decad.EndpointAt(r3.NewVec(0, 0, 10)), decad.ParallelTo(y)).
		Or(decad.EndpointAt(r3.NewVec(20, 20, 10)), decad.ParallelTo(x)).
		Or(decad.EndpointAt(r3.NewVec(20, 20, 10)), decad.ParallelTo(y))
	doc := decad.New()
	box := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10))
	out, err := decad.Fillet(t.Context(), box, loop, units.Millimeters(2))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	volume, err := out.Volume()
	require.NoError(t, err)
	want := 4000 - filletRemoved(2, true, true, true, true)
	require.InDelta(t, want, volume.Value.Base(), volume.Bound.Base()+1e-3)
	require.Less(t, volume.Bound.Base(), 1.5)
	require.Len(t, out.Faces(), 10, "the box's six faces and a strip for each of the four edges")
	// Cut 0.1 mm under the top, the rounded box is 20 - 2r(1 - sqrt(1 - 0.95²))... each wall has moved in by
	// r - sqrt(r² - (z - (10 - r))²) = 2 - sqrt(4 - 1.9²) at z = 9.9, and the corners are mitred: the outline's corner
	// points lie on the diagonals.
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 9.9), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	outline, err := out.SectionOutline(t.Context(), frame, units.Millimeters(0.05))
	require.NoError(t, err)
	require.Len(t, outline.Loops, 1)
	in := 2 - math.Sqrt(4-1.9*1.9)
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, p := range outline.Loops[0] {
		lo, hi = math.Min(lo, math.Min(p.X, p.Y)), math.Max(hi, math.Max(p.X, p.Y))
	}
	require.InDelta(t, in, lo, 0.01)
	require.InDelta(t, 20-in, hi, 0.01)
	corners := 0
	for _, p := range outline.Loops[0] {
		if math.Abs(p.X-p.Y) < 1e-6 && math.Abs(p.X-in) < 0.01 {
			corners++
		}
	}
	require.GreaterOrEqual(t, corners, 1, "the lower-left corner of the outline lies on the mitre plane x = y")
	for point, want := range map[r3.Vec]decad.PointLocation{
		r3.NewVec(10.3, 10.7, 5):  decad.PointInside,
		r3.NewVec(3.3, 3.7, 7.5):  decad.PointInside,
		r3.NewVec(10.3, 10.7, 12): decad.PointOutside,
	} {
		got, err := out.LocatePoint(t.Context(), point, units.Millimeters(0.01))
		require.NoError(t, err)
		require.Equal(t, want, got, "%v", point)
	}
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

func TestFilletOfTwoTopEdgesAtACornerMeetsInAMitre(t *testing.T) {
	t.Parallel()
	x, y := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	pair := topEdge(r3.NewVec(0, 0, 10), x).Or(decad.EndpointAt(r3.NewVec(0, 0, 10)), decad.ParallelTo(y))
	doc := decad.New()
	box := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10))
	out, err := decad.Fillet(t.Context(), box, pair, units.Millimeters(3))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	want := 4000 - filletRemoved(3, true, false, true, false)
	require.InDelta(t, want, volume.Value.Base(), volume.Bound.Base()+1e-3)
	require.Len(t, out.Faces(), 8, "six faces and a strip for each of the two edges")
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

func TestFilletOfThreeTopEdgesWhoseFreeEndsShareAFaceIsRefused(t *testing.T) {
	t.Parallel()
	x, y := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	// The edges along y = 0, x = 0 and x = 20: a U round the top face whose two free ends meet the wall y = 20.
	u := topEdge(r3.NewVec(0, 0, 10), x).Or(decad.EndpointAt(r3.NewVec(0, 0, 10)), decad.ParallelTo(y)).
		Or(decad.EndpointAt(r3.NewVec(20, 0, 10)), decad.ParallelTo(y))
	doc := decad.New()
	box := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10))
	_, err := decad.Fillet(t.Context(), box, u, units.Millimeters(1.5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{box}, doc.Bodies())
}

func TestFilletOfMitredEdgesRefusesAFaceBoundaryInTheirReach(t *testing.T) {
	t.Parallel()
	x, y := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	pair := topEdge(r3.NewVec(0, 0, 10), x).Or(decad.EndpointAt(r3.NewVec(0, 0, 10)), decad.ParallelTo(y))
	doc := decad.New()
	box := blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10))
	// A pocket 1.5 mm from the wall x = 0 sits inside the 2 mm the fillet takes from the top face.
	pocket := blockIn(t, doc, r3.NewVec(1.5, 8, 8), r3.NewVec(4.5, 11, 12))
	cut, err := decad.Cut(t.Context(), box, pocket)
	require.NoError(t, err)
	_, err = decad.Fillet(t.Context(), cut, pair, units.Millimeters(2))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{cut}, doc.Bodies())
}
