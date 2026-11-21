package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The end of a straight flange sits on a circular pad. The prior cut leaves
// a straight run of cap edge between its stopping wall and the pad tangent.
func TestChamferStraightRunAtCircularSupport(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	doc := New()
	w := sketch.NewWorld()
	section := func(y float64) *sketch.Sketch {
		f, err := r3.NewFrame(r3.Vec{Y: y}, r3.Vec{X: 1}, r3.Vec{Z: 1})
		require.NoError(t, err)
		pl, err := w.CreatePlaneFromFrame(f)
		require.NoError(t, err)
		s, err := w.CreateSketch(pl)
		require.NoError(t, err)
		return s
	}
	point := func(s *sketch.Sketch, x, z float64) *sketch.Point { v := s.CreatePoint(x, z); s.Fix(v); return v }
	solve := func(s *sketch.Sketch) *sketch.Profile {
		_, err := s.Solve(ctx)
		require.NoError(t, err)
		require.NotEmpty(t, s.Profiles())
		return s.Profiles()[0]
	}
	flange := section(0)
	a, b, c, d := point(flange, -8, -5), point(flange, 0, -5), point(flange, -3, 4), point(flange, -8, .25)
	flange.CreateLine(a, b)
	// The short arc ends inside the pad. Round inward so both recorded
	// arc endpoints are in the closed native disk.
	const tipX = -2.99
	tip := point(flange, tipX, math.Nextafter(math.Sqrt(25-tipX*tipX), 0))
	flange.CreateLine(b, tip)
	flange.CreateArc(flange.Origin(), tip, c)
	flange.CreateLine(c, d)
	flange.CreateLine(d, a)
	plate, err := Extrude(ctx, doc, flange, solve(flange), Distance{D: units.Millimeters(1), Dir: Against})
	require.NoError(t, err)
	// A transverse web makes the input a mesh Boolean, as required by this operation.
	plate, err = Union(ctx, plate, internalBoxBodyAtZ(t, doc, -7.5, .75, -6.5, 2, -.5, 1))
	require.NoError(t, err)
	disk := section(-1)
	disk.AddConstraint(sketch.NewRadius(disk.CreateCircle(disk.Origin(), 5), 5))
	boss, err := Extrude(ctx, doc, disk, solve(disk), Distance{D: units.Millimeters(1.75), Dir: Against})
	require.NoError(t, err)
	stock, err := Union(ctx, plate, boss)
	require.NoError(t, err)
	var ss [2]*sketch.Sketch
	var pp [2]*sketch.Profile
	for i, y := range []float64{-.125, .375} {
		s := section(y)
		pts := []*sketch.Point{point(s, -9, -.5+1.25*(y-.25)), point(s, -4.25, 3.0625+1.25*(y-.25)), point(s, -4.25, 6), point(s, -9, 6)}
		for j, v := range pts {
			s.CreateLine(v, pts[(j+1)%len(pts)])
		}
		ss[i], pp[i] = s, solve(s)
	}
	tool, err := Loft(ctx, doc, ss[0], pp[0], ss[1], pp[1])
	require.NoError(t, err)
	cut, err := Cut(ctx, stock, tool)
	require.NoError(t, err)
	sel := Edges(EndpointAt(r3.Vec{X: -4.25, Z: 3.0625}), EndpointAt(r3.Vec{X: -3, Z: 4})).Exactly(1)
	_, err = Chamfer(ctx, cut, sel, units.Millimeters(.05))
	require.ErrorIs(t, err, ErrUnsupported)
	out, err := Chamfer(ctx, cut, sel, units.Millimeters(.25))
	require.NoError(t, err)
	// A vertical ray at x=-4 crosses the untouched side at z=3.25. At
	// y=0.1, the 3:4 slope gives a 5:4 multiplier on the remaining setback.
	outerZ := func(body *Body) float64 {
		mesh, err := body.Tessellate(ctx, units.Millimeters(.1), WithVerification(VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified() && mesh.VolumeVerified())
		vs := mesh.Vertices()
		top := math.Inf(-1)
		for _, tri := range mesh.Triangles() {
			a, b, c := vs[tri[0]], vs[tri[1]], vs[tri[2]]
			u, v := b.Sub(a), c.Sub(a)
			det := u.X*v.Y - u.Y*v.X
			if det == 0 {
				continue
			}
			x, y := -4-a.X, .1-a.Y
			t, s := (x*v.Y-y*v.X)/det, (u.X*y-u.Y*x)/det
			if t >= 0 && s >= 0 && t+s <= 1 {
				top = max(top, a.Z+t*u.Z+s*v.Z)
			}
		}
		return top
	}
	require.InDelta(t, 3.25, outerZ(cut), 1e-9)
	require.InDelta(t, 3.25-1.25*(.25-.1), outerZ(out), 1e-9)
}
