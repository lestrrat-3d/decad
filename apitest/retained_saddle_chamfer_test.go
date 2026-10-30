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

// The fixtures of agent-docs/retained-edge-blend-design.md §4.9 (R7): a
// Revolve shaft of radius saddleR, 80 mm long on the x axis, cut by an
// Extrude pin of radius r along z through x = 40 (optionally moved off the
// shaft's axis along y). The pin's two mouths are closed saddle edges between
// the shaft's retained cylinder and the bore's.

const saddleR = 10.0

// saddleShaft builds the shaft with a cross hole of radius r whose axis runs
// along z through (40, y, ·), and returns it with its scene.
func saddleShaft(t *testing.T, r, y float64, opts ...decad.DocumentOption) (*decad.Body, *retentionScene) {
	t.Helper()
	sc := &retentionScene{w: sketch.NewWorld(), doc: decad.New(opts...)}
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 80, saddleR)
	s.Fix(rect.A)
	s.Fix(rect.C)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	shaft, err := decad.Revolve(t.Context(), sc.doc, s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	body, err := decad.Cut(t.Context(), shaft, saddlePin(t, sc, r, y))
	require.NoError(t, err)
	return body, sc
}

// saddlePin extrudes a circle of radius r about (40, y) from z = −15 to 15.
func saddlePin(t *testing.T, sc *retentionScene, r, y float64) *decad.Body {
	t.Helper()
	plane, err := sc.w.CreateOffsetPlane(sc.w.XY(), -15)
	require.NoError(t, err)
	s, err := sc.w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(40, y)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	pin, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(30), Dir: decad.Along})
	require.NoError(t, err)
	return pin
}

// saddleMouths selects the closed edges of body that lie on the shaft wall
// and on a cylinder of radius r about the pin axis through (40, y).
func saddleMouths(t *testing.T, body *decad.Body, r, y float64) decad.EdgeSelector {
	t.Helper()
	var q *decad.EdgeQuery
	n := 0
	for _, e := range body.Edges() {
		p := e.Start().Position().Value
		if p != e.End().Position().Value || math.Abs(math.Hypot(p.X-40, p.Y-y)-r) > 0.05 ||
			math.Abs(math.Hypot(p.Y, p.Z)-saddleR) > 0.05 {
			continue
		}
		if q == nil {
			q = decad.Edges(decad.EndpointAt(p))
		} else {
			q = q.Or(decad.EndpointAt(p))
		}
		n++
	}
	require.Equal(t, 2, n, "both mouths of the cross hole")
	return q.Exactly(2)
}

// saddleSection is the exact section of the chamfer wedge at angle θ around
// the bore, in the fixed pose of agent-docs/cross-cylinder-bore-chamfer-design.md
// with the bore axis at z = 0: the seam point P, the outer foot O on the shaft
// wall and the bore foot B on the bore wall, each d from P along its own
// unrolled cylinder.
type saddleSection struct{ p, o, b r3.Vec }

func saddleSectionAt(R, r, d, theta float64) saddleSection {
	s, c := math.Sincos(theta)
	x := math.Sqrt(R*R - r*r*c*c)
	q := math.Sqrt(R*R - r*r*c*c*c*c)
	do := d * c * x / (R * q)
	db := d * c * s / q
	return saddleSection{
		p: r3.NewVec(x, r*c, r*s),
		o: r3.NewVec(x*math.Cos(do)-r*c*math.Sin(do), x*math.Sin(do)+r*c*math.Cos(do), r*s+d*R*s/q),
		b: r3.NewVec(x*(1-d/q), r*(c*math.Cos(db)-s*math.Sin(db)), r*(s*math.Cos(db)+c*math.Sin(db))),
	}
}

// saddleWedgeVolume is the volume one mouth's chamfer removes, by the
// divergence theorem over the wedge's closed boundary: the ruled band from B to
// O, the shaft wall from O back to P (its angle and height linear between the
// two) and the bore wall from P to B (its angle and axial place linear). Each
// piece is integrated as the flux of p/3 on an n × m midpoint grid in (θ, s).
func saddleWedgeVolume(R, r, d float64) float64 {
	const n, m = 4000, 40
	band := func(theta, u float64) r3.Vec {
		w := saddleSectionAt(R, r, d, theta)
		return w.b.Add(w.o.Sub(w.b).Scale(u))
	}
	outer := func(theta, u float64) r3.Vec {
		s, c := math.Sincos(theta)
		x := math.Sqrt(R*R - r*r*c*c)
		q := math.Sqrt(R*R - r*r*c*c*c*c)
		phi := math.Atan2(r*c, x) + u*d*c*x/(R*q)
		return r3.NewVec(R*math.Cos(phi), R*math.Sin(phi), r*s+u*d*R*s/q)
	}
	bore := func(theta, u float64) r3.Vec {
		s, c := math.Sincos(theta)
		x := math.Sqrt(R*R - r*r*c*c)
		q := math.Sqrt(R*R - r*r*c*c*c*c)
		psi := theta + u*d*c*s/q
		return r3.NewVec(x-u*x*d/q, r*math.Cos(psi), r*math.Sin(psi))
	}
	flux := func(f func(theta, u float64) r3.Vec) float64 {
		total := 0.0
		for i := range n {
			t0, t1 := 2*math.Pi*float64(i)/n, 2*math.Pi*float64(i+1)/n
			for j := range m {
				u0, u1 := float64(j)/m, float64(j+1)/m
				p00, p10, p01, p11 := f(t0, u0), f(t1, u0), f(t0, u1), f(t1, u1)
				dt := p10.Sub(p00).Add(p11.Sub(p01)).Scale(0.5)
				du := p01.Sub(p00).Add(p11.Sub(p10)).Scale(0.5)
				total += f((t0+t1)/2, (u0+u1)/2).Dot(dt.Cross(du))
			}
		}
		return total
	}
	return math.Abs(flux(band)-flux(outer)+flux(bore)) / 3
}

// saddleWorld maps a point of the fixed pose to the world for the mouth on
// the side sign of z: the bore axis is world z, the shaft axis world x.
func saddleWorld(p r3.Vec, sign float64) r3.Vec {
	return r3.NewVec(40+p.Z, p.Y, sign*p.X)
}

func TestRetainedSaddleChamfer(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		r, d float64
		// meaningful is set where the removed volume exceeds the two
		// bodies' bounds, so the volume alone tells a chamfer from none.
		meaningful bool
	}{
		{"r3 d1.5", 3, 1.5, true},
		{"r3 d0.5", 3, 0.5, false},
		{"r7 d0.5", 7, 0.5, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			body, _ := saddleShaft(t, tc.r, 0)
			before, err := body.Volume()
			require.NoError(t, err)
			sel := saddleMouths(t, body, tc.r, 0)
			stations := make([]saddleSection, 8)
			for k := range stations {
				stations[k] = saddleSectionAt(saddleR, tc.r, tc.d, 2*math.Pi*(float64(k)+0.25)/8)
			}
			for _, sign := range []float64{1, -1} {
				for _, w := range stations {
					centroid := w.p.Add(w.o).Add(w.b).Scale(1.0 / 3)
					in, err := body.LocatePoint(t.Context(), saddleWorld(centroid, sign), units.Millimeters(0.01))
					require.NoError(t, err)
					require.Equal(t, decad.PointInside, in, "the wedge is material before the chamfer")
				}
			}

			out, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(tc.d))
			require.NoError(t, err)
			after, err := out.Volume()
			require.NoError(t, err)
			want := 2 * saddleWedgeVolume(saddleR, tc.r, tc.d)
			removed := before.Value.Base() - after.Value.Base()
			require.InDelta(t, want, removed, before.Bound.Base()+after.Bound.Base(),
				"the chamfer removes both wedges")
			if tc.meaningful {
				require.Greater(t, want, before.Bound.Base()+after.Bound.Base(),
					"the removed volume exceeds both bounds")
			}

			kinds := map[string]int{}
			for _, f := range out.Faces() {
				switch f.Surface().(type) {
				case decad.Cylinder:
					kinds["cylinder"]++
				case decad.Plane:
					kinds["plane"]++
				case decad.Faceted:
					kinds["faceted"]++
				default:
					kinds["other"]++
				}
			}
			require.Equal(t, map[string]int{"cylinder": 2, "plane": 2, "faceted": 2}, kinds,
				"the walls keep their cylinders and each band is faceted")

			mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())

			// The body's occupied-volume bound keeps LocatePoint from deciding
			// a point this near its boundary, so the points are read against
			// the verified mesh itself. An outside point farther from the mesh
			// than its proven boundary bound is outside the true solid too.
			held, err := decad.NewMeshBody(t.Context(), mesh.Vertices(), mesh.Triangles())
			require.NoError(t, err)
			for _, sign := range []float64{1, -1} {
				for _, w := range stations {
					centroid := w.p.Add(w.o).Add(w.b).Scale(1.0 / 3)
					got, err := held.LocatePoint(t.Context(), saddleWorld(centroid, sign))
					require.NoError(t, err)
					require.Equal(t, decad.PointOutside, got, "the wedge is gone")
					gap, err := held.DistanceToPoint(t.Context(), saddleWorld(centroid, sign))
					require.NoError(t, err)
					require.Greater(t, gap.Value.Base()-gap.Bound.Base(), mesh.Bound().Base(),
						"the wedge's centroid clears the true boundary")
					mid := w.o.Add(w.b).Scale(0.5)
					past := mid.Add(mid.Sub(w.p).Scale(0.3 / mid.Sub(w.p).Len()))
					got, err = held.LocatePoint(t.Context(), saddleWorld(past, sign))
					require.NoError(t, err)
					require.Equal(t, decad.PointInside, got, "the material past the band stays")
				}
			}
		})
	}
}

func TestRetainedSaddleChamferRefusals(t *testing.T) {
	t.Parallel()
	t.Run("a setback past half the bore's radius", func(t *testing.T) {
		t.Parallel()
		body, _ := saddleShaft(t, 3, 0)
		_, err := decad.Chamfer(t.Context(), body, saddleMouths(t, body, 3, 0), units.Millimeters(1.6))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "a saddle chamfer's setback is past half the bore's radius")
	})
	t.Run("a pin off the shaft's axis", func(t *testing.T) {
		t.Parallel()
		body, _ := saddleShaft(t, 3, 0.5)
		_, err := decad.Chamfer(t.Context(), body, saddleMouths(t, body, 3, 0.5), units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, crossingCylindersReason)
	})
	t.Run("a bore at or past 1/sqrt(2) of the shaft", func(t *testing.T) {
		t.Parallel()
		body, _ := saddleShaft(t, 7.5, 0)
		_, err := decad.Chamfer(t.Context(), body, saddleMouths(t, body, 7.5, 0), units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, crossingCylindersReason)
	})
	t.Run("a pin joined through the shaft", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		s, err := sc.w.CreateSketch(sc.w.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(0, 0, 80, saddleR)
		s.Fix(rect.A)
		s.Fix(rect.C)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		shaft, err := decad.Revolve(t.Context(), sc.doc, s, s.Profiles()[0], uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		body, err := decad.Union(t.Context(), shaft, saddlePin(t, sc, 3, 0))
		require.NoError(t, err)
		_, err = decad.Chamfer(t.Context(), body, saddleMouths(t, body, 3, 0), units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, crossingCylindersReason)
	})
	t.Run("a flat milled beside the mouth", func(t *testing.T) {
		t.Parallel()
		// The flat's Cut meets the shaft wall the hole's Cut already held, so
		// the document allows that wall's bound at the contact.
		body, sc := saddleShaft(t, 3, 0, decad.WithChainTolerance(units.Millimeters(0.01)))
		s, err := sc.w.CreateSketch(sc.w.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(44.5, -20, 60, 20)
		s.Fix(rect.A)
		s.Fix(rect.C)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		box, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
		require.NoError(t, err)
		move, err := r3.Translation(r3.Vec{Z: 9.8})
		require.NoError(t, err)
		flat, err := decad.Transform(t.Context(), box, move)
		require.NoError(t, err)
		body, err = decad.Cut(t.Context(), body, flat)
		require.NoError(t, err)
		_, err = decad.Chamfer(t.Context(), body, saddleMouths(t, body, 3, 0), units.Millimeters(1.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "another face of the body comes within the saddle chamfer's reach")
	})
}
