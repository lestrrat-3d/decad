package pair_test

import (
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// The exact clip of docs/multibody-dynamics-design.md §9.4 and the manifold
// of §9.3 at the snapshot level. Every coordinate is dyadic.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - the counterclockwise orientation in ClipConvex:
//     TestClipConvexEitherWinding's clockwise clip keeps nothing;
//   - the dropped axis chosen by the largest normal component, fixed to z
//     instead: TestPlaneFrameDropsLargestAxis' vertical plane lifts to the
//     wrong point (a division by its zero z component);
//   - the pass-through vertex removal in the published corners:
//     TestPlanarTouchManifoldFacePatch publishes the vertex inside a face
//     edge;
//   - the shallow support's clip of the edge's feet to the face:
//     TestPlanarPenetrationManifoldEdge's wedge past the floor's rim
//     publishes a foot off the face;
//   - the shallow support's lift onto the face plane: the same test's feet
//     sit at the wedge's own depth.

func rat(num, den int64) *big.Rat { return big.NewRat(num, den) }

func p2(x, y float64) pair.Point2 {
	return pair.Point2{X: new(big.Rat).SetFloat64(x), Y: new(big.Rat).SetFloat64(y)}
}

// crossAt is the exact crossing of lines pq and uw.
func crossAt(p, q, u, w pair.Point2) pair.Point2 {
	sub := func(a, b pair.Point2) pair.Point2 {
		return pair.Point2{X: new(big.Rat).Sub(a.X, b.X), Y: new(big.Rat).Sub(a.Y, b.Y)}
	}
	cross := func(a, b pair.Point2) *big.Rat {
		out := new(big.Rat).Mul(a.X, b.Y)
		return out.Sub(out, new(big.Rat).Mul(a.Y, b.X))
	}
	d, e := sub(q, p), sub(w, u)
	t := new(big.Rat).Quo(cross(sub(u, p), e), cross(d, e))
	return pair.Point2{X: new(big.Rat).Add(p.X, new(big.Rat).Mul(t, d.X)),
		Y: new(big.Rat).Add(p.Y, new(big.Rat).Mul(t, d.Y))}
}

func requireSamePoints(t *testing.T, want, got []pair.Point2) {
	t.Helper()
	require.Len(t, got, len(want))
	for _, w := range want {
		found := false
		for _, g := range got {
			if g.X.Cmp(w.X) == 0 && g.Y.Cmp(w.Y) == 0 {
				found = true
			}
		}
		require.True(t, found, "missing (%s, %s)", w.X.RatString(), w.Y.RatString())
	}
}

var hexagon = []pair.Point2{p2(0, 0), p2(5.75, -10), p2(17.25, -10), p2(23, 0), p2(17.25, 10), p2(5.75, 10)}

// tiltedSquare is a square turned in the plane with exact dyadic corners:
// (0,-17) + a·(17.25,10) + b·(-10,17.25) for a, b in {0, 1}.
var tiltedSquare = []pair.Point2{p2(0, -17), p2(17.25, -7), p2(7.25, 10.25), p2(-10, 0.25)}

func TestClipConvexHexagonOverhang(t *testing.T) {
	got, err := pair.ClipConvex(hexagon, tiltedSquare, noPoll)
	require.NoError(t, err)
	h, f := hexagon, tiltedSquare
	want := []pair.Point2{h[0], h[1], crossAt(h[1], h[2], f[0], f[1]), f[1],
		crossAt(h[4], h[5], f[1], f[2]), crossAt(h[4], h[5], f[2], f[3]), crossAt(h[5], h[0], f[2], f[3])}
	requireSamePoints(t, want, got)
	// The double area by the shoelace over the hand-computed corners, in
	// their boundary order.
	area := new(big.Rat)
	for i, p := range want {
		q := want[(i+1)%len(want)]
		area.Add(area, new(big.Rat).Sub(new(big.Rat).Mul(p.X, q.Y), new(big.Rat).Mul(p.Y, q.X)))
	}
	require.Positive(t, area.Sign())
	require.Zero(t, area.Cmp(pair.DoubleArea(got)))
}

func TestClipConvexEitherWinding(t *testing.T) {
	reverse := func(polygon []pair.Point2) []pair.Point2 {
		out := make([]pair.Point2, len(polygon))
		for i, p := range polygon {
			out[len(polygon)-1-i] = p
		}
		return out
	}
	want, err := pair.ClipConvex(hexagon, tiltedSquare, noPoll)
	require.NoError(t, err)
	for _, tc := range []struct{ subject, clip []pair.Point2 }{
		{reverse(hexagon), tiltedSquare},
		{hexagon, reverse(tiltedSquare)},
		{reverse(hexagon), reverse(tiltedSquare)},
	} {
		got, err := pair.ClipConvex(tc.subject, tc.clip, noPoll)
		require.NoError(t, err)
		requireSamePoints(t, want, got)
	}
}

func TestClipConvexThirdCrossing(t *testing.T) {
	// The triangle's edge y = 3x crosses the box's edge y = 1 at x = 1/3.
	got, err := pair.ClipConvex([]pair.Point2{p2(0, 0), p2(8, 0), p2(1, 3)},
		[]pair.Point2{p2(0, 0), p2(4, 0), p2(4, 1), p2(0, 1)}, noPoll)
	require.NoError(t, err)
	requireSamePoints(t, []pair.Point2{p2(0, 0), p2(4, 0), p2(4, 1), {X: rat(1, 3), Y: rat(1, 1)}}, got)
	require.Zero(t, pair.DoubleArea(got).Cmp(rat(23, 3)))
}

func TestClipConvexPolls(t *testing.T) {
	stop := errors.New("stop")
	calls := 0
	_, err := pair.ClipConvex(hexagon, tiltedSquare, func() error {
		calls++
		if calls == 5 {
			return stop
		}
		return nil
	})
	require.ErrorIs(t, err, stop)
}

func TestPlaneFrameDropsLargestAxis(t *testing.T) {
	// The wall x + z/4 = 10 is nearly vertical: z is its smallest normal
	// component, so the frame drops x and lifts every point exactly.
	frame := pair.NewPlaneFrame(vec(1, 0, 0.25), vec(10, 0, 0))
	require.Equal(t, 0, frame.K)
	at := pair.Point3{rat(9, 1), rat(3, 1), rat(4, 1)}
	lifted := frame.Lift(frame.Project(at))
	for axis := range 3 {
		require.Zero(t, lifted[axis].Cmp(at[axis]))
	}
}

// facedBox is boxSolid with one face id per box face, numbered as boxTris
// lists them: x-, x+, y-, y+, z-, z+.
func facedBox(lo, hi [3]float64) pair.PlanarSolid {
	s := boxSolid(lo, hi)
	for i := range s.Tris {
		s.Faces = append(s.Faces, i/2)
	}
	return s
}

// splitBottomEdge adds a vertex at the middle of a facedBox's edge from
// corner 0 to corner 1, splitting the two triangles that hold it, as a
// Boolean's mesh leaves a vertex inside a straight face edge.
func splitBottomEdge(s pair.PlanarSolid) pair.PlanarSolid {
	mid := vec(0, 0, 0)
	for axis := range 3 {
		mid[axis] = proof.DyShift(proof.DyAdd(s.Verts[0][axis], s.Verts[1][axis]), -1)
	}
	s.Verts = append(s.Verts, mid)
	var tris [][3]int
	var faces []int
	for i, tri := range s.Tris {
		switch tri {
		case [3]int{0, 1, 5}:
			tris = append(tris, [3]int{0, 8, 5}, [3]int{8, 1, 5})
			faces = append(faces, s.Faces[i], s.Faces[i])
		case [3]int{0, 3, 1}:
			tris = append(tris, [3]int{0, 3, 8}, [3]int{8, 3, 1})
			faces = append(faces, s.Faces[i], s.Faces[i])
		default:
			tris = append(tris, tri)
			faces = append(faces, s.Faces[i])
		}
	}
	s.Tris, s.Faces = tris, faces
	return s
}

func TestPlanarTouchManifoldFacePatch(t *testing.T) {
	// A box resting inside a wider one's top face; its bottom face carries a
	// vertex in the middle of one edge, which is no corner of the patch.
	a := facedBox([3]float64{0, 0, 0}, [3]float64{4, 4, 1})
	b := splitBottomEdge(facedBox([3]float64{1, 1, 1}, [3]float64{3, 3, 2}))
	ok, err := pair.CheckPlanarSolid(&b, noPoll)
	require.NoError(t, err)
	require.True(t, ok)
	result, err := pair.ClassifyPlanar(&a, &b, noPoll)
	require.NoError(t, err)
	require.Equal(t, pair.Touching, result.Relation)
	manifold, err := pair.PlanarTouchManifold(&a, &b, result.Contacts, true, true, noPoll)
	require.NoError(t, err)
	require.Equal(t, pair.NoReason, manifold.Reason)
	var got []pair.Point2
	for _, point := range manifold.Points {
		require.Equal(t, point.OnA, point.OnB)
		require.Zero(t, point.OnA[2].Cmp(rat(1, 1)))
		require.Equal(t, pair.FeatureFacet, point.A.Kind)
		require.Equal(t, []int{5}, point.A.Faces)
		require.Equal(t, []int{4}, point.B.Faces)
		require.Zero(t, point.Normal[0].Sign())
		require.Zero(t, point.Normal[1].Sign())
		require.Positive(t, point.Normal[2].Sign(), "the normal leaves A toward B")
		got = append(got, pair.Point2{X: point.OnA[0], Y: point.OnA[1]})
	}
	requireSamePoints(t, []pair.Point2{p2(1, 1), p2(3, 1), p2(3, 3), p2(1, 3)}, got)
}

func TestPlanarPenetrationManifold(t *testing.T) {
	a := facedBox([3]float64{0, 0, 0}, [3]float64{4, 4, 1})
	b := facedBox([3]float64{1, 1, 0.75}, [3]float64{2, 2, 3})
	points, err := pair.PlanarPenetrationManifold(&a, &b, noPoll)
	require.NoError(t, err)
	require.Len(t, points, 4)
	for _, point := range points {
		require.Zero(t, point.OnA[2].Cmp(rat(1, 1)))
		require.Zero(t, point.OnB[2].Cmp(rat(3, 4)))
		require.Equal(t, -0.25, point.Separation.ValueMM)
		require.Zero(t, point.Separation.BoundMM)
		require.Equal(t, []int{5}, point.A.Faces)
		require.Equal(t, []int{4}, point.B.Faces)
	}
}

func TestPlanarManifoldsPoll(t *testing.T) {
	stop := errors.New("stop")
	a := facedBox([3]float64{0, 0, 0}, [3]float64{4, 4, 1})
	b := splitBottomEdge(facedBox([3]float64{1, 1, 1}, [3]float64{3, 3, 2}))
	result, err := pair.ClassifyPlanar(&a, &b, noPoll)
	require.NoError(t, err)
	sunk := facedBox([3]float64{1, 1, 0.75}, [3]float64{2, 2, 3})
	runs := map[string]func(poll func() error) error{
		"touch": func(poll func() error) error {
			_, err := pair.PlanarTouchManifold(&a, &b, result.Contacts, true, true, poll)
			return err
		},
		"penetration": func(poll func() error) error {
			_, err := pair.PlanarPenetrationManifold(&a, &sunk, poll)
			return err
		},
	}
	for name, run := range runs {
		calls := 0
		require.NoError(t, run(func() error { calls++; return nil }))
		require.Greater(t, calls, 10, name)
		// A poll failing at any call, the last included, stops the run with
		// that error.
		for limit := 1; limit <= calls; limit++ {
			n := 0
			err := run(func() error {
				if n++; n == limit {
					return stop
				}
				return nil
			})
			require.ErrorIs(t, err, stop, "%s limit=%d", name, limit)
		}
	}
}

// facedWedge is a prism along y over [y0, y1] whose section, in (x, z), is
// the triangle (1, 0.75), (2, 2), (0, 2): its lower edge points down. Its
// faces are the two caps and the three sides.
func facedWedge(y0, y1 float64) pair.PlanarSolid {
	section := [][2]float64{{1, .75}, {2, 2}, {0, 2}}
	var s pair.PlanarSolid
	for _, y := range []float64{y0, y1} {
		for _, p := range section {
			s.Verts = append(s.Verts, vec(p[0], y, p[1]))
		}
	}
	// The section runs counterclockwise in (x, z), so seen from +y it runs
	// clockwise: the y0 cap faces −y with the order 0, 1, 2.
	s.Tris = [][3]int{{0, 1, 2}, {3, 5, 4}, {0, 3, 4}, {0, 4, 1}, {1, 4, 5}, {1, 5, 2}, {2, 5, 3}, {2, 3, 0}}
	s.Faces = []int{0, 1, 2, 2, 3, 3, 4, 4}
	return s
}

func TestPlanarPenetrationManifoldEdge(t *testing.T) {
	// The wedge's lower edge pokes 1/4 mm into the floor's top face z = 1.
	// No wedge face is parallel to it, so §9.3 publishes the edge's two ends,
	// each with its foot on the face, at depth 1/4, in both orders.
	floor := facedBox([3]float64{0, 0, 0}, [3]float64{4, 4, 1})
	wedge := facedWedge(1, 2)
	ok, err := pair.CheckPlanarSolid(&wedge, noPoll)
	require.NoError(t, err)
	require.True(t, ok, "the wedge is a closed outward-wound solid")
	for order := range 2 {
		a, b := floor, wedge
		if order == 1 {
			a, b = wedge, floor
		}
		points, err := pair.PlanarPenetrationManifold(&a, &b, noPoll)
		require.NoError(t, err)
		require.Len(t, points, 2, "order %d", order)
		var ys []float64
		for _, point := range points {
			onFloor, onWedge, floorFeature, wedgeFeature := point.OnA, point.OnB, point.A, point.B
			normal := 1
			if order == 1 {
				onFloor, onWedge, floorFeature, wedgeFeature = point.OnB, point.OnA, point.B, point.A
				normal = -1
			}
			require.Zero(t, onFloor[0].Cmp(rat(1, 1)))
			require.Zero(t, onFloor[2].Cmp(rat(1, 1)), "the foot lies on the face plane")
			require.Zero(t, onWedge[2].Cmp(rat(3, 4)))
			require.Zero(t, onFloor[1].Cmp(onWedge[1]))
			y, _ := onWedge[1].Float64()
			ys = append(ys, y)
			require.Equal(t, -0.25, point.Separation.ValueMM)
			require.Zero(t, point.Separation.BoundMM)
			require.Equal(t, normal, point.Normal[2].Sign())
			require.Zero(t, point.Normal[0].Sign())
			require.Equal(t, pair.FeatureFacet, floorFeature.Kind)
			require.Equal(t, []int{5}, floorFeature.Faces)
			require.Equal(t, pair.FeatureEdge, wedgeFeature.Kind)
			require.Equal(t, []int{2, 4}, wedgeFeature.Faces, "the sides through the lower edge")
		}
		require.ElementsMatch(t, []float64{1, 2}, ys)
	}

	// The same wedge reaching past the floor's y = 4 rim publishes only the
	// part of its edge whose feet lie on the face: y from 3 to 4.
	past := facedWedge(3, 5)
	points, err := pair.PlanarPenetrationManifold(&floor, &past, noPoll)
	require.NoError(t, err)
	require.Len(t, points, 2)
	var ys []float64
	for _, point := range points {
		y, _ := point.OnA[1].Float64()
		ys = append(ys, y)
	}
	require.ElementsMatch(t, []float64{3, 4}, ys)
}
