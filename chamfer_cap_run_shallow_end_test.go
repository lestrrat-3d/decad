package decad_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// turnAt is the angle, in radians, by which the walk a -> b -> c turns at b.
func turnAt(a, b, c [2]float64) float64 {
	u := [2]float64{b[0] - a[0], b[1] - a[1]}
	w := [2]float64{c[0] - b[0], c[1] - b[1]}
	return math.Atan2(math.Abs(u[0]*w[1]-u[1]*w[0]), u[0]*w[0]+u[1]*w[1])
}

// TestChamferCapRunShallowEnd chamfers runs of a prism's cap loop whose convex end turns by less than 45 degrees, or
// whose next edge is shorter than the cutter's reach along it but holds the bevel's run-out. Each selection is the
// bottom edge and the closing edge, mitred at the origin. Each convex end removes d^3 cot(turn)/6 beyond the
// perpendicular end's wedge, and the right-angle mitre gives back d^3/3.
func TestChamferCapRunShallowEnd(t *testing.T) {
	t.Parallel()
	const d = 0.5
	rad := func(deg float64) float64 { return deg * math.Pi / 180 }
	at := func(deg, l float64) [2]float64 { return [2]float64{30 + l*math.Cos(rad(deg)), l * math.Sin(rad(deg))} }
	cases := []struct {
		name string
		pts  [][2]float64
	}{
		{"convex end at 30 degrees", [][2]float64{{0, 0}, {30, 0}, at(30, 10), {0, 20}}},
		{"convex end at 15 degrees", [][2]float64{{0, 0}, {30, 0}, at(15, 10), {0, 20}}},
		{"convex end at 11 degrees", [][2]float64{{0, 0}, {30, 0}, at(11, 10), {0, 20}}},
		// The next edge holds the bevel's run-out, d/sin(60) = 0.577, but not the cutter's reach along it.
		{"next edge of 1 mm at 60 degrees", [][2]float64{{0, 0}, {30, 0}, at(60, 1), {0, 20}}},
		// A blade's gable: the next edge, 1.22 mm at 42.5 degrees, turns back at the apex to the blade's other side.
		{"next edge of a gable at 42.5 degrees", [][2]float64{{0, 0}, {30, 0}, at(42.5, 1.22),
			{30, 2 * 1.22 * math.Sin(rad(42.5))}, {0, 2 * 1.22 * math.Sin(rad(42.5))}}},
	}
	for _, c := range cases {
		for _, sx := range []float64{1, -1} {
			t.Run(fmt.Sprintf("%s, mirror %v", c.name, sx), func(t *testing.T) {
				t.Parallel()
				body := polyBody(t, sx, c.pts)
				n := len(c.pts)
				sel := topEdges(lineEdge(sx, c.pts[0], c.pts[1]), lineEdge(sx, c.pts[n-1], c.pts[0]))
				out, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(d))
				require.NoError(t, err)
				require.True(t, out.IsSolid())
				end := turnAt(c.pts[0], c.pts[1], c.pts[2])
				start := turnAt(c.pts[n-2], c.pts[n-1], c.pts[0])
				length := 30 + math.Hypot(c.pts[n-1][0], c.pts[n-1][1])
				want := volumeOf(t, body) - (length*d*d/2 - d*d*d/3 + d*d*d*(1/math.Tan(end)+1/math.Tan(start))/6)
				require.InDelta(t, want, volumeOf(t, out), 1e-4)
			})
		}
	}
}

// bladeProfile is an outline in (x, z): a stem whose side meets an arm's underside in a concave round of radius 2, the
// arm's end, a head whose top meets a blade in concave rounds of radius 1, and the blade's gable, whose sides turn 39.8
// degrees off the blade's sides. It returns the outline's corners, in walk order, and the rounds by corner index.
func bladeProfile() ([][2]float64, map[int]float64) {
	return [][2]float64{{-5, 0}, {5, 0}, {5, 40}, {25, 40}, {25, 44}, {14, 44}, {14, 58}, {13, 59.2}, {12, 58},
		{12, 44}, {-5, 44}}, map[int]float64{2: 2, 5: 1, 9: 1}
}

// bladeBody extrudes bladeProfile on the plane y = 0 along +y by 6 mm. With joined set a block stands on the head
// top inside the faces, so the result is a mesh Boolean result.
func bladeBody(t *testing.T, joined bool) *decad.Body {
	t.Helper()
	doc := decad.New()
	v, rounds := bladeProfile()
	w := sketch.NewWorld()
	f, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	pts := make([]*sketch.Point, len(v))
	for i, q := range v {
		pts[i] = s.CreatePoint(q[0], q[1])
		s.Fix(pts[i])
	}
	ls := make([]*sketch.Line, len(v))
	for i := range v {
		ls[i] = s.CreateLine(pts[i], pts[(i+1)%len(v)])
	}
	for i, r := range rounds {
		_, err := sketch.CreateFillet(s, ls[(i+len(v)-1)%len(v)], ls[i], pts[i], r)
		require.NoError(t, err)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var pr *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid {
			pr = p
		}
	}
	require.NotNil(t, pr)
	// The sketch plane's normal is -y, so Against extrudes along +y.
	body, err := decad.Extrude(t.Context(), doc, s, pr, decad.Distance{D: units.Millimeters(6), Dir: decad.Against})
	require.NoError(t, err)
	if !joined {
		return body
	}
	out, err := decad.Union(t.Context(), body, box(t, doc, [3]float64{20, 2, 43}, [3]float64{22, 4, 46}))
	require.NoError(t, err)
	return out
}

// TestChamferCapRunsWithSharedCornerAndShallowEnds chamfers the long runs of bladeProfile's face y = 0, on the prism
// and on a mesh Boolean result: the head top's run (the blade's side, its root round and the head top) meets the
// stem's far side across a sharp corner, where the two mitre in one cutter; the stem's near side runs round the arm's
// root to its underside; the arm's head top runs round the blade's other root up its side. Both blade-side runs end at
// the gable, turned 39.8 degrees, beside an edge of 1.56 mm.
func TestChamferCapRunsWithSharedCornerAndShallowEnds(t *testing.T) {
	t.Parallel()
	const d = 0.5
	v, _ := bladeProfile()
	gable := turnAt(v[6], v[7], v[8])
	// Each straight stretch between its round's tangent points, and each round's band, whose wedge centroid lies d/3
	// outside its radius (the rounds are concave in the face).
	lines := (58 - 45) + (12 - 1 + 5) + 44 + (40 - 2) + (25 - 7) + (25 - 15) + (58 - 45)
	rounds := math.Pi / 2 * ((2 + d/3) + 2*(1+d/3))
	removed := (float64(lines)+rounds)*d*d/2 - d*d*d/3 + 2*d*d*d/math.Tan(gable)/6
	for _, joined := range []bool{false, true} {
		t.Run(fmt.Sprintf("joined %v", joined), func(t *testing.T) {
			t.Parallel()
			body := bladeBody(t, joined)
			var q *decad.EdgeQuery
			n := 0
			for _, e := range body.Edges() {
				a, b := e.Start().Position().Value, e.End().Position().Value
				if a.Y != 0 || b.Y != 0 {
					continue
				}
				lo, hi := r3.Vec{X: math.Min(a.X, b.X), Z: math.Min(a.Z, b.Z)}, r3.Vec{X: math.Max(a.X, b.X), Z: math.Max(a.Z, b.Z)}
				// Leave out the bottom, the arm's end and the gable.
				if hi.Z <= 1e-9 || (lo.X >= 25-1e-9 && hi.Z <= 44+1e-9 && lo.Z >= 40-1e-9) || lo.Z >= 58-1e-9 {
					continue
				}
				p := []decad.EdgePredicate{decad.EndpointAt(a), decad.EndpointAt(b)}
				if q == nil {
					q = decad.Edges(p...)
				} else {
					q = q.Or(p...)
				}
				n++
			}
			require.Equal(t, 10, n, "the face's long runs hold ten edges")
			out, err := decad.Chamfer(t.Context(), body, q.Exactly(n), units.Millimeters(d))
			require.NoError(t, err)
			require.True(t, out.IsSolid())
			before, err := body.Volume()
			require.NoError(t, err)
			after, err := out.Volume()
			require.NoError(t, err)
			// The cutter's chords lie within 0.0002 mm of the rounds' bevels, which moves the volume by far less than 0.01.
			slack := before.Bound.Base() + after.Bound.Base() + 0.01
			require.InDelta(t, removed, before.Value.Base()-after.Value.Base(), slack)
		})
	}
}
