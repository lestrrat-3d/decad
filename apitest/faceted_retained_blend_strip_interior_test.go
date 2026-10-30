package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of agent-docs/retained-edge-blend-design.md §6.3, "Strip
// triangles": every facet of a blend face, the band beside the wall's cut
// included, is sampled on a barycentric grid against the exact blend surface,
// and every sample must lie within the mesh's published Bound of it. The
// surfaces come from each fixture's own construction in closed form: a
// profile in the meridian half-plane about the axis, or a sphere. A sample
// farther than Bound disproves the bound; samples within it prove nothing.

// blendProfile is the blend's profile in the meridian half-plane about the
// vertical axis through (cx, cy): the arc of radius r about (rc, zc) whose
// directions run from a0 counter-clockwise through span, or, for a chamfer,
// the segment from p0 to p1.
type blendProfile struct {
	cx, cy    float64
	rc, zc, r float64
	a0, span  float64
	seg       bool
	p0, p1    [2]float64
}

// dist is the distance of p from the surface the profile sweeps about the
// axis: a surface of revolution is as far from p as its profile is from p's
// image in the meridian half-plane.
func (m blendProfile) dist(p r3.Vec) float64 {
	rho, z := math.Hypot(p.X-m.cx, p.Y-m.cy), p.Z
	if m.seg {
		ux, uz := m.p1[0]-m.p0[0], m.p1[1]-m.p0[1]
		s := math.Min(1, math.Max(0, ((rho-m.p0[0])*ux+(z-m.p0[1])*uz)/(ux*ux+uz*uz)))
		return math.Hypot(rho-m.p0[0]-s*ux, z-m.p0[1]-s*uz)
	}
	dx, dz := rho-m.rc, z-m.zc
	th := math.Atan2(dz, dx) - m.a0
	th -= 2 * math.Pi * math.Floor(th/(2*math.Pi))
	if th <= m.span {
		return math.Abs(math.Hypot(dx, dz) - m.r)
	}
	end := func(a float64) float64 {
		return math.Hypot(rho-m.rc-m.r*math.Cos(a), z-m.zc-m.r*math.Sin(a))
	}
	return math.Min(end(m.a0), end(m.a0+m.span))
}

// originsOf collects the origins of every face of b, before a blend consumes
// it.
func originsOf(b *decad.Body) map[decad.FeatureRef]bool {
	out := map[decad.FeatureRef]bool{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			out[o] = true
		}
	}
	return out
}

// madeFaces returns the faces of after that the blend made: those none of
// whose origins appear in old, the origins of the body it was made from.
func madeFaces(old map[decad.FeatureRef]bool, after *decad.Body) map[*decad.Face]bool {
	out := map[*decad.Face]bool{}
	for _, f := range after.Faces() {
		fresh := true
		for _, o := range f.Origins() {
			fresh = fresh && !old[o]
		}
		if fresh {
			out[f] = true
		}
	}
	return out
}

// requireBlendFacetsCovered samples every facet of out's mesh whose source
// face keep accepts on a 12×12 barycentric grid and requires every sample
// within the mesh's Bound of the surface dist measures. band counts the facets
// with a vertex off the surface, the band beside the wall's cut; at least one
// must be sampled when wantBand is set. tau is the blend's chord tolerance,
// logged as the worst sample's multiple of it.
func requireBlendFacetsCovered(t *testing.T, out *decad.Body, keep func(*decad.Face) bool,
	dist func(r3.Vec) float64, tau float64, wantBand bool) {
	t.Helper()
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	verts, src := mesh.Vertices(), mesh.SourceFaces()
	worst, sampled, band := 0.0, 0, 0
	for i, tri := range mesh.Triangles() {
		if !keep(src[i]) {
			continue
		}
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		sampled++
		if dist(a) > 1e-9 || dist(b) > 1e-9 || dist(c) > 1e-9 {
			band++
		}
		const g = 12
		for x := 0; x <= g; x++ {
			for y := 0; x+y <= g; y++ {
				p := a.Add(b.Sub(a).Scale(float64(x) / g)).Add(c.Sub(a).Scale(float64(y) / g))
				worst = math.Max(worst, dist(p))
			}
		}
	}
	bound := mesh.Bound().Base()
	t.Logf("blend facets within %.6g mm (%.3f τ) of the surface, Bound %.6g mm (%.3f of it), %d blend facets, "+
		"%d in the band, %d facets", worst, worst/tau, bound, worst/bound, sampled, band, len(mesh.Triangles()))
	require.Positive(t, sampled)
	if wantBand {
		require.Positive(t, band, "the band beside the wall's cut is sampled")
	}
	require.LessOrEqual(t, worst, bound)
}

// bossTop is the plate with a boss of radius bossR about (20, 10) whose top
// cap lies at z = 30.001, extruded down from it and lifted, so the cap's level
// is the exact float the lift states.
func bossTop(t *testing.T, bossR float64) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(20, 10)
	s.Fix(c)
	s.CreateCircle(c, bossR)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	boss, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10.5), Dir: decad.Against})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), blendPlate(t, sc), translated(t, boss, 0, 0, 30.001))
	require.NoError(t, err)
	return joined
}

func TestFacetedRetainedBlendStripInterior(t *testing.T) {
	t.Parallel()

	// The rounded box corner cut back to x = gx (the chain route, κ = −1): the
	// plane's row, the circle of radius 3 − r, meets x = gx at the sine q. The
	// rows nearer the axis sweep much wider angles than the rim does, and the
	// end's azimuth moves fast between rows near grazing.
	for _, tc := range []struct{ r, q float64 }{{2, 0.4}, {2, 0.2}, {2.5, 0.4}, {2.5, 0.2}, {2.5, 0.1}} {
		t.Run(fmt.Sprintf("a boss corner's chain fillet, r = %g, q = %g", tc.r, tc.q), func(t *testing.T) {
			t.Parallel()
			h := (openBoxR - tc.r) * math.Sqrt(1-tc.q*tc.q)
			gx := openBoxR - h
			sc := newRetentionScene()
			block := placed(t, rectBox(t, sc.w, sc.doc, -5, -5, gx, 25, 30), -5)
			trimmed, err := decad.Cut(t.Context(), roundedCornerBox(t, sc), block)
			require.NoError(t, err)
			seed := arcSeedAt(t, trimmed, 20)
			old := originsOf(trimmed)
			out, err := decad.Fillet(t.Context(), trimmed, seed, units.Millimeters(tc.r), decad.WithTangentChain())
			require.NoError(t, err)
			made := madeFaces(old, out)
			keep := func(f *decad.Face) bool {
				// The corner's own face: the straight entries' faces lie past
				// x = 3 or y = 3.
				p := firstVertex(t, f)
				return made[f] && p.X < openBoxR+1e-6 && p.Y < openBoxR+1e-6
			}
			profile := blendProfile{cx: openBoxR, cy: openBoxR, rc: openBoxR - tc.r, zc: 20 - tc.r, r: tc.r, span: math.Pi / 2}
			requireBlendFacetsCovered(t, out, keep, profile.dist, math.Ldexp(tc.r, -10), true)
		})
	}

	// A Ø8 bore whose axis lies h from the plate's face x = 0 breaks out
	// through it (the arc route, κ = +1): the wall's row meets x = 0 at the
	// sine c. A fillet's strip triangle adds the tube's sagitta, the azimuth's
	// and the shift of the rows' ends together.
	for _, c := range []float64{0.4, 0.2} {
		for _, d := range []float64{1, 2, 4} {
			t.Run(fmt.Sprintf("a hole mouth's arc fillet, d = %g, c = %g", d, c), func(t *testing.T) {
				t.Parallel()
				const bore = 4.0
				h := bore * math.Sqrt(1-c*c)
				sc := newRetentionScene()
				bored, err := decad.Cut(t.Context(), blendPlate(t, sc), zCylinderAt(t, sc, h, 10, bore, -30, 30))
				require.NoError(t, err)
				old := originsOf(bored)
				out, err := decad.Fillet(t.Context(), bored, arcSeedAt(t, bored, 20), units.Millimeters(d))
				require.NoError(t, err)
				made := madeFaces(old, out)
				profile := blendProfile{cx: h, cy: 10, rc: bore + d, zc: 20 - d, r: d, a0: math.Pi / 2, span: math.Pi / 2}
				requireBlendFacetsCovered(t, out, func(f *decad.Face) bool { return made[f] }, profile.dist,
					math.Ldexp(d, -10), true)
			})
		}
	}

	// A boss of radius R on the plate, its closed top rim filleted (R2,
	// κ = −1, the ball inside the boss): the tube's sagitta and the azimuth's
	// both move a strip triangle toward the axis, so they add.
	for _, tc := range []struct{ bossR, r float64 }{{3, 1}, {3, 2.5}, {4, 1}, {4, 2.5}} {
		t.Run(fmt.Sprintf("a boss's top rim fillet, R = %g, r = %g", tc.bossR, tc.r), func(t *testing.T) {
			t.Parallel()
			const top = 30.001
			joined := bossTop(t, tc.bossR)
			old := originsOf(joined)
			out, err := decad.Fillet(t.Context(), joined, rimAt(t, joined, top), units.Millimeters(tc.r))
			require.NoError(t, err)
			made := madeFaces(old, out)
			profile := blendProfile{cx: 20, cy: 10, rc: tc.bossR - tc.r, zc: top - tc.r, r: tc.r, span: math.Pi / 2}
			requireBlendFacetsCovered(t, out, func(f *decad.Face) bool { return made[f] }, profile.dist,
				math.Ldexp(tc.r, -10), true)
		})
	}

	// The cap of the ball of radius 2 below z = −1, its rim filleted (R6): the
	// wall's cut vertices lie on the sphere's held facets, which the cut plane
	// meets at a slant, and the band's triangles span the held facets' azimuth
	// steps.
	t.Run("a sphere rim's fillet, r = 0.4", func(t *testing.T) {
		t.Parallel()
		const r = 0.4
		cut := flatBall(t, 0, 0)
		old := originsOf(cut)
		out, err := decad.Fillet(t.Context(), cut, discRim(t, cut), units.Millimeters(r))
		require.NoError(t, err)
		made := madeFaces(old, out)
		// The ball touches the plane z = −1 from below and the sphere from
		// inside: its centre lies at height −1 − r, R − r from the sphere's
		// centre, and the arc runs from straight up to the direction away
		// from the sphere's centre.
		zc := ballCut - r
		rc := math.Sqrt((ballR-r)*(ballR-r) - zc*zc)
		a0 := math.Atan2(zc, rc)
		profile := blendProfile{rc: rc, zc: zc, r: r, a0: a0, span: math.Pi/2 - a0}
		requireBlendFacetsCovered(t, out, func(f *decad.Face) bool { return made[f] }, profile.dist,
			math.Ldexp(r, -10), true)
	})

	// Three edges at a box's corner filleted (§4.8): the sphere octant's
	// triangles, whose sides are the strips' end rows, lie farther from the
	// sphere than those rows' chords do.
	for _, r := range []float64{1, 2, 3} {
		t.Run(fmt.Sprintf("a box corner's sphere patch, r = %g", r), func(t *testing.T) {
			t.Parallel()
			x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
			corner := r3.NewVec(0, 0, 10)
			three := decad.Edges(decad.EndpointAt(corner), decad.ParallelTo(x)).
				Or(decad.EndpointAt(corner), decad.ParallelTo(y)).Or(decad.EndpointAt(corner), decad.ParallelTo(z))
			box := blockIn(t, decad.New(), r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10))
			out, err := decad.Fillet(t.Context(), box, three, units.Millimeters(r))
			require.NoError(t, err)
			centre := r3.NewVec(r, r, 10-r)
			// The octant is the face whose corners all lie r from its centre.
			octant := func(f *decad.Face) bool {
				for _, l := range f.Loops() {
					for _, ce := range l.CoEdges() {
						if math.Abs(ce.Start().Position().Value.Sub(centre).Len()-r) > 1e-9 {
							return false
						}
					}
				}
				return true
			}
			requireBlendFacetsCovered(t, out, octant, func(p r3.Vec) float64 { return math.Abs(p.Sub(centre).Len() - r) },
				math.Ldexp(r, -13), false)
		})
	}
}
