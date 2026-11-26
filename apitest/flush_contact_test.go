package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// flushProfilePrism extrudes the closed outline pts, drawn on the plane through origin spanned by u and v, by length
// toward dir. A side from (1, 6) to (-1, 6) is drawn as a true arc about (0, 6); every other side is a line.
func flushProfilePrism(t *testing.T, doc *decad.Document, origin, u, v r3.Vec, dir decad.Direction, pts [][2]float64,
	length float64) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	verts := make([]*sketch.Point, len(pts))
	for i, q := range pts {
		verts[i] = s.CreatePoint(q[0], q[1])
		s.Fix(verts[i])
	}
	c := s.CreatePoint(0, 6)
	s.Fix(c)
	for i := range verts {
		j := (i + 1) % len(verts)
		if pts[i] == [2]float64{1, 6} && pts[j] == [2]float64{-1, 6} {
			s.CreateArc(c, verts[i], verts[j])
			continue
		}
		s.CreateLine(verts[i], verts[j])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.Extrude(doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(length), Dir: dir}).Do(t.Context())
	require.NoError(t, err)
	return body
}

// flushTongue is a strip x in [-1, 1] rising from feet at z = -2 to a true arc of radius 1 about (0, 6), with an
// 8-chord quarter round of radius 0.7 at each root, and its area above z = 0.
func flushTongue() ([][2]float64, float64) {
	pts := [][2]float64{{-2.4, -2}, {2.4, -2}, {2.4, -0.5}}
	for i := range 9 {
		a := (270 - 90*float64(i)/8) * math.Pi / 180
		pts = append(pts, [2]float64{1.7 + 0.7*math.Cos(a), 0.2 + 0.7*math.Sin(a)})
	}
	pts = append(pts, [2]float64{1, 6}, [2]float64{-1, 6})
	for i := range 9 {
		a := (0 - 90*float64(i)/8) * math.Pi / 180
		pts = append(pts, [2]float64{-1.7 + 0.7*math.Cos(a), 0.2 + 0.7*math.Sin(a)})
	}
	pts = append(pts, [2]float64{-2.4, -0.5})
	// The polygon clipped to z >= 0, plus the half disc the arc closes.
	var clipped [][2]float64
	for i, a := range pts {
		b := pts[(i+1)%len(pts)]
		if a[1] >= 0 {
			clipped = append(clipped, a)
		}
		if (a[1] < 0) != (b[1] < 0) {
			s := a[1] / (a[1] - b[1])
			clipped = append(clipped, [2]float64{a[0] + s*(b[0]-a[0]), 0})
		}
	}
	area := 0.0
	for i, a := range clipped {
		b := clipped[(i+1)%len(clipped)]
		area += a[0]*b[1] - b[0]*a[1]
	}
	return pts, math.Abs(area)/2 + math.Pi/2
}

// TestFlushContactArcCap: a tool whose start cap lies flush on a box face, and whose outline holds a true arc far from
// the box's outline, joins the box with one planar face on the flush plane and no operation diagnostic
// (agent-docs/flush-contact-design.md §10.3).
func TestFlushContactArcCap(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	x, y, z := r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
	host := flushProfilePrism(t, doc, r3.Vec{}, x, y, decad.Against, [][2]float64{{-10, 0}, {10, 0}, {10, 16}, {-10, 16}}, 10)
	pts, above := flushTongue()
	// The sketch's normal x × z is −y, so Against runs toward +y.
	tool := flushProfilePrism(t, doc, r3.Vec{}, x, z, decad.Against, pts, 4)
	joined, err := decad.Union(host, tool).Do(t.Context())
	require.NoError(t, err)
	require.Empty(t, joined.OperationDiagnostics())
	faces, err := decad.Faces(decad.Facing(y.Scale(-1))).Exactly(1).SelectFaces(joined)
	require.NoError(t, err)
	_, plane := faces[0].Surface().(decad.Plane)
	require.True(t, plane, "the flush face is a Plane")
	v, err := joined.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(v.Value.Base()-(3200+4*above)), v.Bound.Base())
}
