package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// sharedCapHeight is the height of every lower body of the shared-cap tests: their caps lie on z = sharedCapHeight.
const sharedCapHeight = 1.25

// sharedCapCircle sketches one circle of radius r about (c[0], c[1]) on the plane z.
func sharedCapCircle(t *testing.T, w *sketch.World, z float64, c [2]float64, r float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	plane, err := w.CreateOffsetPlane(w.XY(), z)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(c[0], c[1])
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

// sharedCapFrustum lofts the circle of radius r0 about c0 on z = 0 to the circle of radius r1 about the z axis on
// z = sharedCapHeight.
func sharedCapFrustum(t *testing.T, doc *Document, c0 [2]float64, r0, r1 float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s0, p0 := sharedCapCircle(t, w, 0, c0, r0)
	s1, p1 := sharedCapCircle(t, w, sharedCapHeight, [2]float64{}, r1)
	body, err := Loft(t.Context(), doc, s0, p0, s1, p1)
	require.NoError(t, err)
	return body
}

// sharedCapBoss extrudes the circle of radius r about c on z = sharedCapHeight up by height: a boss standing on the
// cap plane.
func sharedCapBoss(t *testing.T, doc *Document, c [2]float64, r, height float64) *Body {
	t.Helper()
	s, p := sharedCapCircle(t, sketch.NewWorld(), sharedCapHeight, c, r)
	body, err := Extrude(t.Context(), doc, s, p, Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	return body
}

// sharedCapCone revolves a truncated cone about the vertical axis through c: its base circle of radius r lies on
// z = sharedCapHeight, where the lower body's cap lies, and it narrows to radius tip at length above it.
func sharedCapCone(t *testing.T, doc *Document, c [2]float64, r, tip, length float64) *Body {
	t.Helper()
	return sharedCapRevolve(t, doc, c, [][2]float64{{0, 0}, {0, r}, {length, tip}, {length, 0}})
}

// sharedCapRevolve revolves the closed meridian polygon pts, each point (height above z = sharedCapHeight, radius),
// about the vertical axis through c.
func sharedCapRevolve(t *testing.T, doc *Document, c [2]float64, pts [][2]float64) *Body {
	t.Helper()
	origin := r3.Vec{X: c[0], Y: c[1], Z: sharedCapHeight}
	f, err := r3.NewFrame(origin, r3.Vec{Z: 1}, r3.Vec{X: 1})
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	points := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		points[i] = s.CreatePoint(p[0], p[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := Revolve(t.Context(), doc, s, s.Profiles()[0], ConstructionAxis{Origin: origin, Dir: r3.Vec{Z: 1}},
		FullRevolution{})
	require.NoError(t, err)
	return body
}

// sharedCapObliqueBase is the centre of the oblique frustum's base: 1.9 from the axis at 235 degrees, where the slider
// tongue's drive cone has it.
func sharedCapObliqueBase() [2]float64 {
	th := 235 * math.Pi / 180
	return [2]float64{1.9 * math.Cos(th), 1.9 * math.Sin(th)}
}

// requireDisjointUnion asserts that joined, the Union of lower and upper, is one solid with no gap shift, whose volume
// equals the sum of the operands' volumes within the three bounds and whose box spans both operands' boxes within the
// three box bounds.
func requireDisjointUnion(t *testing.T, lower, upper, joined *Body) {
	t.Helper()
	require.True(t, joined.IsSolid())
	require.Len(t, joined.Lumps(), 1)
	requireNoGapShift(t, joined)
	lv, err := lower.Volume()
	require.NoError(t, err)
	uv, err := upper.Volume()
	require.NoError(t, err)
	jv, err := joined.Volume()
	require.NoError(t, err)
	slack := jv.Bound.Base() + lv.Bound.Base() + uv.Bound.Base()
	require.InDelta(t, lv.Value.Base()+uv.Value.Base(), jv.Value.Base(), slack, "volume of the union vs the sum of its operands")

	lb, err := lower.Bounds()
	require.NoError(t, err)
	ub, err := upper.Bounds()
	require.NoError(t, err)
	jb, err := joined.Bounds()
	require.NoError(t, err)
	reach := jb.Bound.Base() + lb.Bound.Base() + ub.Bound.Base()
	for _, pair := range [][2]float64{
		{math.Min(lb.Min.X, ub.Min.X), jb.Min.X}, {math.Min(lb.Min.Y, ub.Min.Y), jb.Min.Y},
		{math.Min(lb.Min.Z, ub.Min.Z), jb.Min.Z}, {math.Max(lb.Max.X, ub.Max.X), jb.Max.X},
		{math.Max(lb.Max.Y, ub.Max.Y), jb.Max.Y}, {math.Max(lb.Max.Z, ub.Max.Z), jb.Max.Z},
	} {
		require.InDelta(t, pair[0], pair[1], reach, "box of the union vs the operands' boxes")
	}
}
