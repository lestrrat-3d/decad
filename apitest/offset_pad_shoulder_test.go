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

func TestOffsetPadShoulderChamfer(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	d := decad.New()
	frame := func(o, u, v r3.Vec) *sketch.Sketch {
		f, e := r3.NewFrame(o, u, v)
		require.NoError(t, e)
		w := sketch.NewWorld()
		p, e := w.CreatePlaneFromFrame(f)
		require.NoError(t, e)
		s, e := w.CreateSketch(p)
		require.NoError(t, e)
		return s
	}
	x, y, z := r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
	profile := func(s *sketch.Sketch) *sketch.Profile {
		_, e := s.Solve(ctx)
		require.NoError(t, e)
		for _, p := range s.Profiles() {
			if p.Valid {
				return p
			}
		}
		t.Fatal("invalid profile")
		return nil
	}
	extrude := func(s *sketch.Sketch, h float64, dir decad.Direction) *decad.Body {
		b, e := decad.Extrude(ctx, d, s, profile(s), decad.Distance{D: units.Millimeters(h), Dir: dir})
		require.NoError(t, e)
		return b
	}
	point := func(s *sketch.Sketch, a, b float64) *sketch.Point {
		p := s.CreatePoint(a, b)
		s.Fix(p)
		return p
	}
	stock := frame(r3.Vec{X: 8}, x, z)
	a, b := point(stock, -1.5, 2), point(stock, -1.5, -2)
	stock.CreateArc(stock.Origin(), a, b)
	c, f := point(stock, 4.5, -6.5), point(stock, 4.5, 6.5)
	stock.CreateLine(b, c)
	mid := point(stock, 5, 4.5)
	stock.CreateLine(c, mid)
	stock.CreateLine(mid, f)
	stock.CreateLine(f, a)
	plate := extrude(stock, 1, decad.Along)
	// Cross drill away from the round to produce faceted stock with a retained shoulder.
	drill := frame(r3.Vec{X: 11, Y: -.5, Z: -8}, x, y)
	drill.CreateCircle(drill.Origin(), .2)
	plate, e := decad.Cut(plate, extrude(drill, 16, decad.Along)).Do(ctx)
	require.NoError(t, e)
	padSketch := frame(r3.Vec{X: 8.008, Y: -.5}, x, z)
	padSketch.CreateCircle(padSketch.Origin(), 2.5)
	// The pad chamfer ends 0.00001 mm above the flange. Its cylindrical side
	// protects the flange through the exposed shoulder cut.
	pad := extrude(padSketch, .75001, decad.Against)
	joined, e := decad.Union(plate, pad).Do(ctx)
	require.NoError(t, e)
	rounded, e := decad.Chamfer(ctx, joined, decad.Edges(decad.OuterLoopOf(decad.CapStart(pad))).Exactly(1), units.Millimeters(.25))
	require.NoError(t, e)
	// The four-sided loft stops the existing bevel at z = 3.125.
	var ss [2]*sketch.Sketch
	var pp [2]*sketch.Profile
	for i, h := range []float64{-.375, .125} {
		s := frame(r3.Vec{X: 8, Y: h}, x, z)
		ss[i] = s
		// In local coordinates the side has -3x + 4z = 12.5. Its unit normal is (-3, 4)/5.
		edge := func(v float64) float64 { return (4*v - 12.5 + 5*(.25+h)) / 3 }
		var pts []*sketch.Point
		for _, q := range [][2]float64{{edge(3.125), 3.125}, {edge(8), 8}, {-5, 8}, {-5, 3.125}} {
			pts = append(pts, point(s, q[0], q[1]))
		}
		for j, p := range pts {
			s.CreateLine(p, pts[(j+1)%len(pts)])
		}
		pp[i] = profile(s)
	}
	loft, e := decad.Loft(ctx, d, ss[0], pp[0], ss[1], pp[1])
	require.NoError(t, e)
	trimmed, e := decad.Cut(rounded, loft).Do(ctx)
	require.NoError(t, e)

	sel := decad.Edges(decad.EndpointAt(r3.Vec{X: 8, Z: 3.125}), decad.EndpointAt(r3.Vec{X: 6.5, Z: 2})).Exactly(1)
	_, e = decad.Chamfer(ctx, trimmed, sel, units.Millimeters(.5))
	require.ErrorIs(t, e, decad.ErrUnsupported)
	res, e := decad.Chamfer(ctx, trimmed, sel, units.Millimeters(.25))
	require.NoError(t, e)

	// A vertical ray through the plane-only portion of the shoulder meets the
	// cap before the cut and the 45-degree bevel afterwards. At this point the
	// perpendicular inset from the side is 0.06 mm.
	const px, pz = 7.8, 2.9
	for _, row := range []struct {
		body   *decad.Body
		height float64
	}{{trimmed, 0}, {res, -.19}} {
		m, e := row.body.Tessellate(ctx, units.Millimeters(.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, e)
		require.True(t, m.VolumeVerified())
		require.True(t, m.BoundaryVerified())
		require.InDelta(t, row.height, highestMeshYAt(m, px, pz), 1e-8)
	}
}

func highestMeshYAt(m *decad.Mesh, px, pz float64) float64 {
	vs := m.Vertices()
	height := math.Inf(-1)
	for _, tri := range m.Triangles() {
		a, b, c := vs[tri[0]], vs[tri[1]], vs[tri[2]]
		u, v := b.Sub(a), c.Sub(a)
		det := u.X*v.Z - u.Z*v.X
		if det == 0 {
			continue
		}
		weightB := ((px-a.X)*v.Z - (pz-a.Z)*v.X) / det
		weightC := ((pz-a.Z)*u.X - (px-a.X)*u.Z) / det
		if weightB >= 0 && weightC >= 0 && weightB+weightC <= 1 {
			height = max(height, a.Y+weightB*u.Y+weightC*v.Y)
		}
	}

	return height
}
