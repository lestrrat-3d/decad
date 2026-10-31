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
	body, err := decad.Cut(t.Context(), saddleRevolve(t, sc), saddlePin(t, sc, r, y))
	require.NoError(t, err)
	return body, sc
}

// saddleShaftWithFlatBesideMouth cuts a flat against the bore cut's held wall,
// so the document allows the wall's bound at the contact.
func saddleShaftWithFlatBesideMouth(t *testing.T) *decad.Body {
	t.Helper()
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
	return body
}

// saddleRevolve revolves the rectangle x in [0, 80], y in [0, saddleR] about
// the x axis.
func saddleRevolve(t *testing.T, sc *retentionScene) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 80, saddleR)
	s.Fix(rect.A)
	s.Fix(rect.C)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	shaft, err := decad.Revolve(t.Context(), sc.doc, s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return shaft
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

// saddleWedge holds the closed boundary of one mouth's chamfer wedge, each
// piece a map of (θ, s) ∈ [0, 2π] × [0, 1]: the ruled band from B to O, the
// shaft wall from P to O (its angle and height linear between the two) and
// the bore wall from P to B (its angle and axial place linear).
type saddleWedge struct{ band, outer, bore func(theta, u float64) r3.Vec }

func saddleWedgeOf(R, r, d float64) saddleWedge {
	return saddleWedge{
		band: func(theta, u float64) r3.Vec {
			w := saddleSectionAt(R, r, d, theta)
			return w.b.Add(w.o.Sub(w.b).Scale(u))
		},
		outer: func(theta, u float64) r3.Vec {
			s, c := math.Sincos(theta)
			x := math.Sqrt(R*R - r*r*c*c)
			q := math.Sqrt(R*R - r*r*c*c*c*c)
			phi := math.Atan2(r*c, x) + u*d*c*x/(R*q)
			return r3.NewVec(R*math.Cos(phi), R*math.Sin(phi), r*s+u*d*R*s/q)
		},
		bore: func(theta, u float64) r3.Vec {
			s, c := math.Sincos(theta)
			x := math.Sqrt(R*R - r*r*c*c)
			q := math.Sqrt(R*R - r*r*c*c*c*c)
			psi := theta + u*d*c*s/q
			return r3.NewVec(x-u*x*d/q, r*math.Cos(psi), r*math.Sin(psi))
		},
	}
}

// saddleGrid sums f over an n × m midpoint grid in (θ, s) of piece p, with
// the cell's two edge vectors.
func saddleGrid(p func(theta, u float64) r3.Vec, f func(mid, dt, du r3.Vec) float64) float64 {
	const n, m = 4000, 40
	total := 0.0
	for i := range n {
		t0, t1 := 2*math.Pi*float64(i)/n, 2*math.Pi*float64(i+1)/n
		for j := range m {
			u0, u1 := float64(j)/m, float64(j+1)/m
			p00, p10, p01, p11 := p(t0, u0), p(t1, u0), p(t0, u1), p(t1, u1)
			dt := p10.Sub(p00).Add(p11.Sub(p01)).Scale(0.5)
			du := p01.Sub(p00).Add(p11.Sub(p10)).Scale(0.5)
			total += f(p((t0+t1)/2, (u0+u1)/2), dt, du)
		}
	}
	return total
}

// saddleWedgeVolume is the volume one mouth's chamfer removes, by the
// divergence theorem over the wedge's closed boundary: the flux of p/3
// through each piece.
func saddleWedgeVolume(R, r, d float64) float64 {
	w := saddleWedgeOf(R, r, d)
	flux := func(p func(theta, u float64) r3.Vec) float64 {
		return saddleGrid(p, func(mid, dt, du r3.Vec) float64 { return mid.Dot(dt.Cross(du)) })
	}
	return math.Abs(flux(w.band)-flux(w.outer)+flux(w.bore)) / 3
}

// saddleAreaChange is the area one mouth's chamfer removes: the shaft wall
// from the seam to O and the bore wall from the seam to B, less the band.
func saddleAreaChange(R, r, d float64) float64 {
	w := saddleWedgeOf(R, r, d)
	area := func(p func(theta, u float64) r3.Vec) float64 {
		return saddleGrid(p, func(_, dt, du r3.Vec) float64 { return dt.Cross(du).Len() })
	}
	return area(w.outer) + area(w.bore) - area(w.band)
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
		// bodies' bounds, so the volume alone tells a chamfer from none. At
		// d = 0.5 (r = 3 and r = 7) the bounds sum to about 15 mm³ against a
		// removal of 4.9 and 10.5 mm³: the volume assertion there proves
		// nothing, and the point checks below carry the test.
		meaningful bool
		// sound is set where Verify reads the result Sound, its area bound
		// within the relative tolerance. At r = 7 the band rims' length
		// bounds keep the area bound past it.
		sound bool
	}{
		{"r3 d1.5", 3, 1.5, true, true},
		{"r3 d0.5", 3, 0.5, false, true},
		{"r7 d0.5", 7, 0.5, false, false},
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
			areaBefore, err := body.Area()
			require.NoError(t, err)
			areaAfter, err := out.Area()
			require.NoError(t, err)
			wantArea := 2 * saddleAreaChange(saddleR, tc.r, tc.d)
			areaBounds := areaBefore.Bound.Base() + areaAfter.Bound.Base()
			require.InDelta(t, wantArea, areaBefore.Value.Base()-areaAfter.Value.Base(), areaBounds,
				"the chamfer trades both walls' strips for the bands")
			if tc.sound {
				require.Greater(t, wantArea, areaBounds, "the area change exceeds both bounds")
				report, err := out.Document().Verify(t.Context())
				require.NoError(t, err)
				reading, err := report.ForBody(out)
				require.NoError(t, err)
				require.Equal(t, decad.ToleranceSatisfied, reading.Area.Tolerance.State)
				require.Equal(t, decad.Sound, reading.Status)
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
		body, err := decad.Union(t.Context(), saddleRevolve(t, sc), saddlePin(t, sc, 3, 0))
		require.NoError(t, err)
		_, err = decad.Chamfer(t.Context(), body, saddleMouths(t, body, 3, 0), units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, crossingCylindersReason)
	})
	t.Run("a flat milled beside the mouth", func(t *testing.T) {
		t.Parallel()
		body := saddleShaftWithFlatBesideMouth(t)
		_, err := decad.Chamfer(t.Context(), body, saddleMouths(t, body, 3, 0), units.Millimeters(1.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "another face of the body comes within the saddle chamfer's reach")
	})
}

// TestRetainedSaddleChamferExtrudeShaft chamfers both mouths of an Extrude
// shaft (radius 10, length 40, centred on the origin) cut by a radius-3
// Extrude pin, under each assignment of the two axes to world coordinates and
// under a translation that leaves the axes meeting off the origin. Under the
// race detector only the moved assignment runs, and only on the mouth at the
// bore axis's negative end; dropping the mouth-side flip or permuting the
// pose's axes in readSaddle fails it.
func TestRetainedSaddleChamferExtrudeShaft(t *testing.T) {
	t.Parallel()
	const r, d = 3.0, 1.5
	wedge := saddleWedgeVolume(saddleR, r, d)
	signs := []float64{1, -1}
	if raceDetector {
		signs = []float64{-1}
	}
	for _, tc := range []struct {
		name        string
		shaft, bore func(w *sketch.World) *sketch.Plane
		// ib, ic and ik are the world coordinates of the shaft's axis, the
		// bore's axis and the third axis.
		ib, ic, ik int
		move       r3.Vec
		race       bool
	}{
		{"shaft z bore x", (*sketch.World).XY, (*sketch.World).YZ, 2, 0, 1, r3.Vec{}, false},
		{"shaft y bore z", (*sketch.World).XZ, (*sketch.World).XY, 1, 2, 0, r3.Vec{}, false},
		{"shaft x bore y", (*sketch.World).YZ, (*sketch.World).XZ, 0, 1, 2, r3.Vec{}, false},
		{"shaft z bore x moved", (*sketch.World).XY, (*sketch.World).YZ, 2, 0, 1, r3.Vec{X: 0.1, Y: 0.2, Z: 0.3}, true},
	} {
		if raceDetector && !tc.race {
			continue
		}
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			sc := newRetentionScene()
			cylinder := func(plane *sketch.Plane, radius, length float64) *decad.Body {
				s, err := sc.w.CreateSketch(plane)
				require.NoError(t, err)
				c := s.CreatePoint(0, 0)
				s.Fix(c)
				s.CreateCircle(c, radius)
				_, err = s.Solve(t.Context())
				require.NoError(t, err)
				b, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
					decad.Symmetric{D: units.Millimeters(length)})
				require.NoError(t, err)
				if tc.move == (r3.Vec{}) {
					return b
				}
				m, err := r3.Translation(tc.move)
				require.NoError(t, err)
				b, err = decad.Transform(t.Context(), b, m)
				require.NoError(t, err)
				return b
			}
			body, err := decad.Cut(t.Context(), cylinder(tc.shaft(sc.w), saddleR, 40), cylinder(tc.bore(sc.w), r, 30))
			require.NoError(t, err)
			at := func(p r3.Vec, i int) float64 { return [3]float64{p.X, p.Y, p.Z}[i] }
			var q *decad.EdgeQuery
			n := 0
			for _, e := range body.Edges() {
				p := e.Start().Position().Value
				if p != e.End().Position().Value {
					continue
				}
				l := p.Sub(tc.move)
				if math.Abs(math.Hypot(at(l, tc.ic), at(l, tc.ik))-saddleR) > 0.05 ||
					math.Abs(math.Hypot(at(l, tc.ik), at(l, tc.ib))-r) > 0.05 ||
					(len(signs) == 1 && at(l, tc.ic)*signs[0] < 0) {
					continue
				}
				if q == nil {
					q = decad.Edges(decad.EndpointAt(p))
				} else {
					q = q.Or(decad.EndpointAt(p))
				}
				n++
			}
			require.Equal(t, len(signs), n, "the selected mouths of the cross hole")
			before, err := body.Volume()
			require.NoError(t, err)
			out, err := decad.Chamfer(t.Context(), body, q.Exactly(n), units.Millimeters(d))
			require.NoError(t, err)
			after, err := out.Volume()
			require.NoError(t, err)
			want := float64(n) * wedge
			require.InDelta(t, want, before.Value.Base()-after.Value.Base(), before.Bound.Base()+after.Bound.Base(),
				"the chamfer removes each selected wedge")
			require.Greater(t, want, before.Bound.Base()+after.Bound.Base(), "the removed volume exceeds both bounds")

			mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			held, err := decad.NewMeshBody(t.Context(), mesh.Vertices(), mesh.Triangles())
			require.NoError(t, err)
			for _, sign := range signs {
				world := func(l r3.Vec) r3.Vec {
					var a [3]float64
					a[tc.ic], a[tc.ik], a[tc.ib] = sign*l.X, l.Y, l.Z
					return r3.Vec{X: a[0], Y: a[1], Z: a[2]}.Add(tc.move)
				}
				for k := range 8 {
					w := saddleSectionAt(saddleR, r, d, 2*math.Pi*(float64(k)+0.25)/8)
					got, err := held.LocatePoint(t.Context(), world(w.p.Add(w.o).Add(w.b).Scale(1.0/3)))
					require.NoError(t, err)
					require.Equal(t, decad.PointOutside, got, "the wedge is gone")
					mid := w.o.Add(w.b).Scale(0.5)
					got, err = held.LocatePoint(t.Context(), world(mid.Add(mid.Sub(w.p).Scale(0.3/mid.Sub(w.p).Len()))))
					require.NoError(t, err)
					require.Equal(t, decad.PointInside, got, "the material past the band stays")
				}
			}
		})
	}
}
