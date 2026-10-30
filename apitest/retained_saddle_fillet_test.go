package apitest_test

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of agent-docs/retained-edge-blend-design.md §4.10: the R7
// shafts of retained_saddle_chamfer_test.go, filleted. In the fixed pose of
// agent-docs/cross-cylinder-bore-chamfer-design.md (shaft x² + y² = R² about
// z, bore y² + z² = r² about x, mouth at x > 0) a ball of radius ρ inside the
// shaft and outside the bore has its centre on the closed curve
// c(θ) = (√(R'² − r'²cos²θ), r'cos θ, r'sin θ), R' = R − ρ, r' = r + ρ, and
// touches the shaft along a1 = (c_x, c_y, 0)/R' and the bore along
// a2 = −(0, cos θ, sin θ).

// saddleBall is the ball's section at angle θ: its centre, the two contact
// directions and the unit vector e that completes a1 to an orthonormal pair
// spanning the arc from a1 to a2, whose angle is phi.
type saddleBall struct {
	c, a1, a2, e r3.Vec
	phi          float64
}

func saddleBallAt(R, r, rho, theta float64) saddleBall {
	Rp, rp := R-rho, r+rho
	s, co := math.Sincos(theta)
	x := math.Sqrt(Rp*Rp - rp*rp*co*co)
	b := saddleBall{c: r3.NewVec(x, rp*co, rp*s), a1: r3.NewVec(x/Rp, rp*co/Rp, 0), a2: r3.NewVec(0, -co, -s)}
	cphi := b.a1.Dot(b.a2)
	b.phi = math.Acos(cphi)
	b.e = b.a2.Sub(b.a1.Scale(cphi)).Scale(1 / math.Sin(b.phi))
	return b
}

// dir is the unit vector at angle alpha from a1 toward a2.
func (b saddleBall) dir(alpha float64) r3.Vec {
	return b.a1.Scale(math.Cos(alpha)).Add(b.e.Scale(math.Sin(alpha)))
}

// exit is the distance from the centre along u to the solid's boundary
// before the fillet: the first point past the ball where the ray leaves the
// shaft (x² + y² = R²) or enters the bore (y² + z² = r²), each an exact
// quadratic root.
func (b saddleBall) exit(R, r, rho float64, u r3.Vec) float64 {
	shaft, bore := b.hits(R, r, rho, u)
	return math.Min(shaft, bore)
}

// hits returns the distances along u at which the ray leaves the shaft and
// enters the bore.
func (b saddleBall) hits(R, r, rho float64, u r3.Vec) (float64, float64) {
	hit := func(pi, pj, ui, uj, rad float64) float64 {
		qa := ui*ui + uj*uj
		qb := 2 * (pi*ui + pj*uj)
		qc := pi*pi + pj*pj - rad*rad
		disc := qb*qb - 4*qa*qc
		best := math.Inf(1)
		if qa == 0 || disc < 0 {
			return best
		}
		for _, s := range []float64{(-qb - math.Sqrt(disc)) / (2 * qa), (-qb + math.Sqrt(disc)) / (2 * qa)} {
			if s >= rho*(1-1e-12) && s < best {
				best = s
			}
		}
		return best
	}
	return hit(b.c.X, b.c.Y, u.X, u.Y, R), hit(b.c.Y, b.c.Z, u.Y, u.Z, r)
}

// gaussLegendre returns the n Gauss–Legendre nodes and weights on [−1, 1].
func gaussLegendre(n int) ([]float64, []float64) {
	x, w := make([]float64, n), make([]float64, n)
	legendre := func(z float64) (float64, float64) {
		p0, p1 := 1.0, z
		for k := 2; k <= n; k++ {
			p0, p1 = p1, ((2*float64(k)-1)*z*p1-(float64(k)-1)*p0)/float64(k)
		}
		return p1, float64(n) * (z*p1 - p0) / (z*z - 1)
	}
	for i := range n {
		z := math.Cos(math.Pi * (float64(i) + 0.75) / (float64(n) + 0.5))
		for range 100 {
			p, dp := legendre(z)
			dz := p / dp
			z -= dz
			if math.Abs(dz) < 1e-15 {
				break
			}
		}
		_, dp := legendre(z)
		x[i], w[i] = z, 2/((1-z*z)*dp*dp)
	}
	return x, w
}

// saddleFilletVolume is the volume one mouth's fillet removes: the region
// between the ball's arc and the edge, in the tube coordinates of the centre
// curve, p = c(θ) + s·u(θ, α) with u the direction at angle α from a1, whose
// volume element is s·|c′|·(1 − s·κ_u) with κ_u = (u·c″)/|c′|². Each ray runs
// from the arc, s = ρ, to its exact exit s_h, so the volume is
//
//	∫dθ ∫₀^φ dα |c′|·[(s_h² − ρ²)/2 − κ_u·(s_h³ − ρ³)/3].
//
// The exit switches walls at the ray through the edge, where the integrand
// has a kink; each θ splits α there (found by bisection) and integrates both
// sides by Gauss–Legendre, over 32 panels in θ.
func saddleFilletVolume(R, r, rho float64) float64 {
	const panels, nodes = 32, 16
	xs, ws := gaussLegendre(nodes)
	rp := r + rho
	total := 0.0
	for p := range panels {
		t0, t1 := 2*math.Pi*float64(p)/panels, 2*math.Pi*float64(p+1)/panels
		for i, xt := range xs {
			theta := (t0+t1)/2 + (t1-t0)/2*xt
			weight := ws[i] * (t1 - t0) / 2
			b := saddleBallAt(R, r, rho, theta)
			s, co := math.Sincos(theta)
			d1 := r3.NewVec(rp*rp*co*s/b.c.X, -rp*s, rp*co)
			d2 := r3.NewVec((rp*rp*(co*co-s*s)-d1.X*d1.X)/b.c.X, -rp*co, -rp*s)
			speed := d1.Len()
			// Near a1 the ray leaves through the shaft, near a2 into the bore.
			shaftFirst := func(alpha float64) bool {
				shaft, bore := b.hits(R, r, rho, b.dir(alpha))
				return shaft < bore
			}
			lo, hi := 0.0, b.phi
			for range 100 {
				mid := (lo + hi) / 2
				if shaftFirst(mid) {
					lo = mid
				} else {
					hi = mid
				}
			}
			kink := (lo + hi) / 2
			for _, seg := range [][2]float64{{0, kink}, {kink, b.phi}} {
				for j, xa := range xs {
					alpha := (seg[0]+seg[1])/2 + (seg[1]-seg[0])/2*xa
					u := b.dir(alpha)
					sh := b.exit(R, r, rho, u)
					ku := u.Dot(d2) / (speed * speed)
					f := speed * ((sh*sh-rho*rho)/2 - ku*(sh*sh*sh-rho*rho*rho)/3)
					total += weight * ws[j] * (seg[1] - seg[0]) / 2 * f
				}
			}
		}
	}
	return total
}

// saddleFilletPoints returns, at angle θ, a point midway along the arc's
// bisector between the arc and the edge, in the removed region, and a point
// 0.3·ρ inside the ball, which stays.
func saddleFilletPoints(R, r, rho, theta float64) (r3.Vec, r3.Vec) {
	b := saddleBallAt(R, r, rho, theta)
	u := b.dir(b.phi / 2)
	out := b.c.Add(u.Scale((rho + b.exit(R, r, rho, u)) / 2))
	in := b.c.Add(u.Scale(0.7 * rho))
	return out, in
}

func TestSaddleFilletHandVolume(t *testing.T) {
	t.Parallel()
	// The values of agent-docs/retained-edge-blend-design.md §4.10, which a
	// rolling-ball grid (a point is removed when it is material and farther
	// than ρ from the material eroded by ρ) matched to 0.5 % and 0.1 %.
	for _, tc := range []struct{ rho, want float64 }{{0.5, 1.635978}, {1.0, 7.317201}, {1.5, 18.593228}} {
		require.InDelta(t, tc.want, saddleFilletVolume(saddleR, 3, tc.rho), 1e-6)
	}
}

// requireFilletedMouths asserts what a filleted pair of mouths of body shows:
// the volume falls by twice the hand value within both bodies' bounds, and
// on the VerifyAll mesh the removed region's sample points read outside,
// farther than the mesh bound, and the ball's read inside, at 8 stations of
// each mouth. world maps a local point of the mouth on side sign to the
// world.
func requireFilletedMouths(t *testing.T, before, out *decad.Body, r, rho float64, meaningful bool,
	signs []float64, world func(p r3.Vec, sign float64) r3.Vec) {
	t.Helper()
	vb, err := before.Volume()
	require.NoError(t, err)
	va, err := out.Volume()
	require.NoError(t, err)
	want := float64(len(signs)) * saddleFilletVolume(saddleR, r, rho)
	bounds := vb.Bound.Base() + va.Bound.Base()
	require.InDelta(t, want, vb.Value.Base()-va.Value.Base(), bounds, "the fillet removes each mouth's edge region")
	if meaningful {
		require.Greater(t, want, bounds, "the removed volume exceeds both bounds")
	}
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	held, err := decad.NewMeshBody(t.Context(), mesh.Vertices(), mesh.Triangles())
	require.NoError(t, err)
	for _, sign := range signs {
		for k := range 8 {
			gone, kept := saddleFilletPoints(saddleR, r, rho, 2*math.Pi*(float64(k)+0.25)/8)
			// The Hole body's bound leaves this point undecided; no body puts
			// it outside.
			in, err := before.LocatePoint(t.Context(), world(gone, sign), units.Millimeters(0.01))
			require.NoError(t, err)
			require.NotEqual(t, decad.PointOutside, in, "the edge region is material before the fillet")
			got, err := held.LocatePoint(t.Context(), world(gone, sign))
			require.NoError(t, err)
			require.Equal(t, decad.PointOutside, got, "the edge region is gone")
			gap, err := held.DistanceToPoint(t.Context(), world(gone, sign))
			require.NoError(t, err)
			require.Greater(t, gap.Value.Base()-gap.Bound.Base(), mesh.Bound().Base(),
				"the removed point clears the true boundary")
			got, err = held.LocatePoint(t.Context(), world(kept, sign))
			require.NoError(t, err)
			require.Equal(t, decad.PointInside, got, "the ball's material stays")
		}
	}
}

func TestRetainedSaddleFillet(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		r, rho float64
		// meaningful is set where the removed volume exceeds the two bodies'
		// bounds; below that the sample points carry the test.
		meaningful bool
		race       bool
	}{
		{"r3 rho1.5", 3, 1.5, true, true},
		{"r3 rho0.5", 3, 0.5, false, false},
		{"r7 rho0.5", 7, 0.5, false, false},
	} {
		if raceDetector && !tc.race {
			continue
		}
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			body, _ := saddleShaft(t, tc.r, 0)
			out, err := decad.Fillet(t.Context(), body, saddleMouths(t, body, tc.r, 0), units.Millimeters(tc.rho))
			require.NoError(t, err)
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
				"the walls keep their cylinders and each fillet is faceted")
			requireFilletedMouths(t, body, out, tc.r, tc.rho, tc.meaningful, []float64{1, -1}, saddleWorld)
			if tc.name != "r3 rho0.5" {
				return
			}
			// The same call on the same shaft builds the same mesh: no merge of
			// near cut points depends on map order. The Fillet retired the
			// first body, so the shaft is built again.
			body, _ = saddleShaft(t, tc.r, 0)
			again, err := decad.Fillet(t.Context(), body, saddleMouths(t, body, tc.r, 0), units.Millimeters(tc.rho))
			require.NoError(t, err)
			first, err := out.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			second, err := again.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			require.True(t, slices.Equal(first.Vertices(), second.Vertices()), "the vertex lists agree")
			require.True(t, slices.Equal(first.Triangles(), second.Triangles()), "the triangle lists agree")
		})
	}
	t.Run("a Hole on the shaft's wall", func(t *testing.T) {
		t.Parallel()
		if raceDetector {
			t.Skip("the Cut fixtures carry the race build")
		}
		sc := newRetentionScene()
		body, err := decad.Hole(t.Context(), saddleRevolve(t, sc), decad.HoleSpec{
			Face: decad.Faces(decad.Cylindrical()), At: r3.NewVec(40, 0, saddleR),
			Diameter: units.Millimeters(6), Depth: decad.ThroughHole{}})
		require.NoError(t, err)
		out, err := decad.Fillet(t.Context(), body, saddleMouths(t, body, 3, 0), units.Millimeters(1.5))
		require.NoError(t, err)
		requireFilletedMouths(t, body, out, 3, 1.5, true, []float64{1, -1}, saddleWorld)
	})
}

// saddleTubeRoot is a tube, the rectangle x in [0, 80], y in [10, 20]
// revolved about x, joined to a radius-3 pin along z through (40, 0) from
// z = 2 to 15, which stands in the tube's bore and roots in its wall.
func saddleTubeRoot(t *testing.T) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, saddleR, 80, 2*saddleR)
	s.Fix(rect.A)
	s.Fix(rect.C)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tube, err := decad.Revolve(t.Context(), sc.doc, s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	plane, err := sc.w.CreateOffsetPlane(sc.w.XY(), 2)
	require.NoError(t, err)
	ps, err := sc.w.CreateSketch(plane)
	require.NoError(t, err)
	c := ps.CreatePoint(40, 0)
	ps.Fix(c)
	ps.CreateCircle(c, 3)
	_, err = ps.Solve(t.Context())
	require.NoError(t, err)
	pin, err := decad.Extrude(t.Context(), sc.doc, ps, ps.Profiles()[0],
		decad.Distance{D: units.Millimeters(13), Dir: decad.Along})
	require.NoError(t, err)
	body, err := decad.Union(t.Context(), tube, pin)
	require.NoError(t, err)
	return body
}

func TestRetainedSaddleFilletRefusals(t *testing.T) {
	t.Parallel()
	t.Run("a ball that cannot fit between the walls", func(t *testing.T) {
		t.Parallel()
		body, _ := saddleShaft(t, 3, 0)
		_, err := decad.Fillet(t.Context(), body, saddleMouths(t, body, 3, 0), units.Millimeters(3.5))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "no ball of that radius touches both walls of the saddle")
	})
	t.Run("a pipe that folds", func(t *testing.T) {
		t.Parallel()
		body, _ := saddleShaft(t, 3, 0)
		_, err := decad.Fillet(t.Context(), body, saddleMouths(t, body, 3, 0), units.Millimeters(3.4))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "folds: the pipe is not regular there")
	})
	t.Run("a pin off the shaft's axis", func(t *testing.T) {
		t.Parallel()
		body, _ := saddleShaft(t, 3, 0.5)
		_, err := decad.Fillet(t.Context(), body, saddleMouths(t, body, 3, 0.5), units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, crossingCylindersReason)
	})
	t.Run("a pin joined through the shaft", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		body, err := decad.Union(t.Context(), saddleRevolve(t, sc), saddlePin(t, sc, 3, 0))
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), body, saddleMouths(t, body, 3, 0), units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, crossingCylindersReason)
	})
	t.Run("a pin's root inside a tube's bore", func(t *testing.T) {
		t.Parallel()
		// The root is concave between the tube's bore (Reversed, radius 10)
		// and the pin's wall (radius 3): both Reversed flags and the Convex
		// flag are a cross hole mouth's flipped, so the ball's sides read as
		// the mouth's, but a blend here adds material and neither route
		// builds it.
		body := saddleTubeRoot(t)
		var sel *decad.EdgeQuery
		for _, e := range body.Edges() {
			p := e.Start().Position().Value
			if p == e.End().Position().Value && math.Abs(math.Hypot(p.X-40, p.Y)-3) < 0.05 &&
				math.Abs(math.Hypot(p.Y, p.Z)-saddleR) < 0.05 {
				require.Nil(t, sel, "one root edge")
				sel = decad.Edges(decad.EndpointAt(p)).Exactly(1)
			}
		}
		require.NotNil(t, sel)
		_, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, crossingCylindersReason)
		_, err = decad.Fillet(t.Context(), body, sel, units.Millimeters(0.5))
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
		_, err = decad.Fillet(t.Context(), body, saddleMouths(t, body, 3, 0), units.Millimeters(1.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "another face of the body comes within the saddle fillet's reach")
	})
}

// TestRetainedSaddleFilletExtrudeShaft fillets both mouths of an Extrude shaft
// (radius 10, length 40, centred on the origin) cut by a radius-3 Extrude
// pin, under each assignment of the two axes to world coordinates and under a
// translation that leaves the axes meeting off the origin. Every one of these
// poses keeps its hand; the Revolve shafts of TestRetainedSaddleFillet take a
// left-handed pose, so the strip's winding flip shows there. Under the race
// detector only the moved assignment runs, on the mouth at the bore axis's
// negative end; dropping the mouth-side flip in readSaddle fails it.
func TestRetainedSaddleFilletExtrudeShaft(t *testing.T) {
	t.Parallel()
	const r, rho = 3.0, 1.5
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
			out, err := decad.Fillet(t.Context(), body, q.Exactly(n), units.Millimeters(rho))
			require.NoError(t, err)
			// One mouth's 18.6 mm³ does not exceed the Extrude bodies' bounds;
			// two mouths' does.
			requireFilletedMouths(t, body, out, r, rho, n == 2, signs, func(l r3.Vec, sign float64) r3.Vec {
				var a [3]float64
				a[tc.ic], a[tc.ik], a[tc.ib] = sign*l.X, l.Y, l.Z
				return r3.Vec{X: a[0], Y: a[1], Z: a[2]}.Add(tc.move)
			})
		})
	}
}
