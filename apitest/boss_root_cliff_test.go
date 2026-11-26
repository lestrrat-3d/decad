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

// spokedDisc is spokedHubDisc's body: the disc of radius 21 with four windows whose inner arcs have radius 7.
func spokedDisc(t *testing.T) *decad.Body {
	t.Helper()
	_, body := spokedHubDisc(t)
	return body
}

// holedDisc extrudes a disc of radius 20, 5 thick from z = 0, with one round hole for each {x, y, radius} of holes.
func holedDisc(t *testing.T, holes ...[3]float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.AddConstraint(sketch.NewRadius(s.CreateCircle(s.Origin(), 20), 20))
	for _, h := range holes {
		centre := s.CreatePoint(h[0], h[1])
		s.Fix(centre)
		s.CreateCircle(centre, h[2])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == len(holes) {
			profile = p
		}
	}
	require.NotNil(t, profile)
	body, err := decad.Extrude(decad.New(), s, profile, decad.Distance{D: units.Millimeters(5), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	return body
}

func bossOnTop(t *testing.T, body *decad.Body, diameter, rootFillet float64) (*decad.Body, error) {
	t.Helper()
	face := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(body))).Exactly(1)
	spec := decad.BossSpec{Face: face, At: r3.Vec{Z: 5}, Diameter: units.Millimeters(diameter), Height: units.Millimeters(1)}
	if rootFillet > 0 {
		spec.RootFillet = units.Millimeters(rootFillet)
	}
	return decad.Boss(t.Context(), body, spec)
}

func torusFaces(t *testing.T, body *decad.Body) (majors, minors []float64) {
	t.Helper()
	for _, f := range body.Faces() {
		tor, ok := f.Surface().(decad.Torus)
		if !ok {
			continue
		}
		major, err := tor.Major.In(units.Millimeter)
		require.NoError(t, err)
		minor, err := tor.Minor.In(units.Millimeter)
		require.NoError(t, err)
		majors, minors = append(majors, major), append(minors, minor)
	}
	return majors, minors
}

// bossFilletVolume is the volume of the root fillet of a Ø15 boss on spokedDisc(t), by numeric integration.
//
// The bead stands on the disc's top face at radius r = R + s, R = 7.5, 0 <= s <= rho = 0.5, up to height
// h(s) = rho - sqrt(rho^2 - (s - rho)^2). It exists only over the part of the top face outside the four windows,
// whose angular measure at radius r is L(r).
//
// Derivation of L(r). Each window corner next to the hub is a round of radius 2 tangent to the hub arc (radius 7)
// and to the arm's side line, which lies 2 from the arm's radial line. So the round's centre is 7 + 2 = 9 from the
// axis and 2 + 2 = 4 from the radial line, which makes its angle from the arm asin(4/9). The arm's radial line is
// pi/4 from the window's middle direction, so the centre is alpha0 = pi/4 - asin(4/9) from that direction. At
// radius r in [7, 8.307] the window's boundary is the round, and the point at angle a beyond the centre satisfies
// the law of cosines on the triangle axis, round centre, point: 81 + r^2 - 2*9*r*cos a = 4, so
// a(r) = acos((r^2 + 77) / (18 r)). The window's half width at r is alpha0 + a(r). The four windows take
// 8 (alpha0 + a(r)) of the circle, leaving L(r) = 2 pi - 8 (alpha0 + a(r)).
//
// V_W = integral over r of h(r - R) L(r) r dr. Substituting s = rho (1 - cos t) gives h = rho (1 - sin t) and
// ds = rho sin t dt, and Simpson's rule integrates it over t in [0, pi/2] on 20,000 intervals.
func bossFilletVolume() float64 {
	const (
		root = 7.5
		rho  = 0.5
		n    = 20000
	)
	alpha0 := math.Pi/4 - math.Asin(4.0/9)
	f := func(tt float64) float64 {
		s := rho * (1 - math.Cos(tt))
		r := root + s
		h := rho * (1 - math.Sin(tt))
		l := 2*math.Pi - 8*(alpha0+math.Acos((r*r+77)/(18*r)))
		return h * l * r * rho * math.Sin(tt)
	}
	step := math.Pi / 2 / n
	sum := f(0) + f(math.Pi/2)
	for i := 1; i < n; i++ {
		if i%2 == 1 {
			sum += 4 * f(float64(i)*step)
		} else {
			sum += 2 * f(float64(i)*step)
		}
	}
	return sum * step / 3
}

// simpson integrates f over [lo, hi] on n (even) intervals by Simpson's rule.
func simpson(f func(float64) float64, lo, hi float64, n int) float64 {
	step := (hi - lo) / float64(n)
	sum := f(lo) + f(hi)
	for i := 1; i < n; i++ {
		if i%2 == 1 {
			sum += 4 * f(lo+float64(i)*step)
		} else {
			sum += 2 * f(lo+float64(i)*step)
		}
	}
	return sum * step / 3
}

// bossRootChangeAreas are the exact areas the Ø15 root fillet changes, which bound the held error of the change in
// volume: the top face's lost region A_lost, the band B_band the boss wall loses, and the eight caps G_cap the window
// rounds gain.
//
// L(r) is the angular measure of the top face's part outside the windows at radius r (bossFilletVolume's derivation).
// The top face loses {R <= r <= R + rho} of it: A_lost = integral of L(r) r dr. The wall loses the band of height rho
// over the same arc: B_band = rho R L(R). One cap is the region of a window round G (radius 2, axis 9 from the boss
// axis) between the top face, the generator through the arc's end and the fillet's section by G. The torus row at
// tube angle t has radius 8 - 0.5 sin t at height 0.5 (1 - cos t) above the face, and meets G at the angle
// psi(t) = acos((81 + 4 - rho^2)/36) at G's axis, so unrolled along G's circle (x = 2 psi) the end curve is
// (x(t), z(t)) = (2 (psi(t) - psi(pi/2)), 0.5 (1 - cos t)) and the cap's area is the integral of z dx.
func bossRootChangeAreas() (aLost, bBand, gCap float64) {
	const (
		root = 7.5
		rho  = 0.5
		n    = 20000
	)
	alpha0 := math.Pi/4 - math.Asin(4.0/9)
	l := func(r float64) float64 { return 2*math.Pi - 8*(alpha0+math.Acos((r*r+77)/(18*r))) }
	aLost = simpson(func(r float64) float64 { return l(r) * r }, root, root+rho, n)
	bBand = rho * root * l(root)
	row := func(t float64) float64 { return root + rho - rho*math.Sin(t) }
	// dx/dt = 2 psi'(t), psi' = (2 rho_t rho_t'/36) / sqrt(1 - u^2), u = (85 - rho_t^2)/36.
	dx := func(t float64) float64 {
		rt := row(t)
		u := (85 - rt*rt) / 36
		return 2 * (2 * rt * (-rho * math.Cos(t)) / 36) / math.Sqrt(1-u*u)
	}
	gCap = 8 * math.Abs(simpson(func(t float64) float64 {
		return 0.5 * (1 - math.Cos(t)) * dx(t)
	}, 0, math.Pi/2, n))
	return aLost, bBand, gCap
}

func TestBossRootFilletSpokedHub(t *testing.T) {
	t.Parallel()
	t.Run("12 builds one torus", func(t *testing.T) {
		t.Parallel()
		disc := spokedDisc(t)
		out, err := bossOnTop(t, disc, 12, 0.5)
		require.NoError(t, err)
		majors, minors := torusFaces(t, out)
		require.Len(t, majors, 1)
		require.InDelta(t, 6.5, majors[0], 1e-9)
		require.InDelta(t, 0.5, minors[0], 1e-9)
	})
	t.Run("13 is refused", func(t *testing.T) {
		t.Parallel()
		disc := spokedDisc(t)
		_, err := bossOnTop(t, disc, 13, 0.5)
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
	// At 14 each root arc ends where a window corner round touches the boss wall (boss-root-overflow C3, §5.5). The
	// volume gains rule CO's bead, integrated in boss_root_tangent_test.go.
	t.Run("14 builds with tangent ends", func(t *testing.T) {
		t.Parallel()
		plain := spokedDisc(t)
		s, err := bossOnTop(t, plain, 14, 0)
		require.NoError(t, err)
		sv, sb := volumeAndBound(t, s)
		disc := spokedDisc(t)
		out, err := bossOnTop(t, disc, 14, 0.5)
		require.NoError(t, err)
		require.True(t, out.IsSolid())
		alpha0, bead, quadErr := spokedHubBead(0.5)
		rv, rb := volumeAndBound(t, out)
		t.Logf("V(S) %v (bound %v), bead %v, V(result) %v (bound %v)", sv, sb, bead, rv, rb)
		require.Less(t, quadErr, 1e-9, "the quadrature resolves the bead far below the bound")
		require.InDelta(t, sv+bead, rv, rb+sb+quadErr)
		requireSpokedHubTangentEnds(t, out, alpha0)
	})
	t.Run("15 builds four capped tori", func(t *testing.T) {
		t.Parallel()
		plain := spokedDisc(t)
		s, err := bossOnTop(t, plain, 15, 0)
		require.NoError(t, err)
		sv, err := s.Volume()
		require.NoError(t, err)
		disc := spokedDisc(t)
		out, err := bossOnTop(t, disc, 15, 0.5)
		require.NoError(t, err)
		majors, minors := torusFaces(t, out)
		require.Len(t, majors, 4)
		for i := range majors {
			require.InDelta(t, 8, majors[i], 1e-9)
			require.InDelta(t, 0.5, minors[i], 1e-9)
		}
		rv, err := out.Volume()
		require.NoError(t, err)
		vw := bossFilletVolume()
		t.Logf("V(S) %v (bound %v), V_W %v, V(result) %v (bound %v)",
			sv.Value.Base(), sv.Bound.Base(), vw, rv.Value.Base(), rv.Bound.Base())
		require.InDelta(t, sv.Value.Base()+vw, rv.Value.Base(), rv.Bound.Base())

		// The change itself: the result's volume gains V_W, within what the held errors can move. A result that adds
		// no fillet moves it by V_W, which the allowance must be too small to hide.
		// S is a faceted body whose minimum mesh bound is 0.0034221 mm: a tolerance just above it restates S's held
		// facets, so the mesh's bound is S's largest facet bound.
		const heldTol = 0.0035
		mesh, err := plain.Tessellate(t.Context(), units.Millimeters(heldTol))
		require.NoError(t, err)
		meshBound := mesh.Bound().Base()
		require.Greater(t, meshBound, 0.003)
		require.LessOrEqual(t, meshBound, heldTol)
		aLost, bBand, gCap := bossRootChangeAreas()
		allowance := (rv.Bound.Base() - sv.Bound.Base()) + meshBound*(aLost+bBand+gCap) + 1e-9
		change := rv.Value.Base() - sv.Value.Base()
		t.Logf("bound(S) %v, bound(result) %v, mesh bound(S) %v, A_lost %v, B_band %v, G_cap %v (one cap %v)",
			sv.Bound.Base(), rv.Bound.Base(), meshBound, aLost, bBand, gCap, gCap/8)
		t.Logf("change %v, V_W %v, |change - V_W| %v, allowance %v", change, vw, math.Abs(change-vw), allowance)
		require.LessOrEqual(t, math.Abs(change-vw), allowance)
		require.Less(t, allowance, vw/2)

		// The cap on the corner round G, a cylinder of radius 2 about the axis through the point 9 from the boss axis
		// at azimuth -alpha0, is observed through ClosestPoint: a probe over the window, 0.6 degrees beyond the
		// window boundary at r = 7.6, has the cap as its nearest surface.
		const r = 7.6
		alpha0 := math.Pi/4 - math.Asin(4.0/9)
		boundary := -(alpha0 + math.Acos((r*r+77)/(18*r)))
		gx, gy := 9*math.Cos(-alpha0), 9*math.Sin(-alpha0)
		const deg = math.Pi / 180
		resultMesh, err := out.Tessellate(t.Context(), units.Millimeters(0.01))
		require.NoError(t, err)
		resultBound := resultMesh.Bound().Base()
		for _, c := range []struct {
			name  string
			theta float64
		}{
			{"in the bead beside the cap", boundary - 0.6*deg},
			{"over the window beyond the cap", boundary + 0.6*deg},
		} {
			p := r3.Vec{X: r * math.Cos(c.theta), Y: r * math.Sin(c.theta), Z: 5.02}
			cp, err := out.ClosestPoint(t.Context(), p, units.Millimeters(0.01))
			require.NoError(t, err, c.name)
			planG := func(q r3.Vec) float64 { return math.Hypot(q.X-gx, q.Y-gy) }
			off := cp.OffSurface.Base()
			t.Logf("%s: probe %v, closest %v, plan distance to G's axis %v, distance %v (bound %v), off surface %v, "+
				"exact distance to G %v, mesh bound %v", c.name, p, cp.Point, planG(cp.Point), cp.Distance.Value.Base(),
				cp.Distance.Bound.Base(), off, math.Abs(planG(p)-2), resultBound)
			// The nearest point lies on G's cylinder at the probe's own height: a vertical face there, which only the
			// cap provides above the face's plane. Its distance from the probe is the probe's exact distance to G.
			require.InDelta(t, 2, planG(cp.Point), off+resultBound, c.name)
			require.InDelta(t, p.Z, cp.Point.Z, off+resultBound, c.name)
			require.InDelta(t, math.Abs(planG(p)-2), cp.Point.Sub(p).Len(), 2*(off+resultBound), c.name)
			require.InDelta(t, math.Abs(planG(p)-2), cp.Distance.Value.Base(), cp.Distance.Bound.Base()+off, c.name)
		}
	})
}

func TestBossRootFilletRefusesArcEndsOnOneFace(t *testing.T) {
	t.Parallel()
	_, err := bossOnTop(t, holedDisc(t, [3]float64{5, 0, 11}), 15, 0)
	require.NoError(t, err)
	_, err = bossOnTop(t, holedDisc(t, [3]float64{5, 0, 11}), 15, 0.5)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestBossRootFilletRefusesHalfCircleArc(t *testing.T) {
	t.Parallel()
	holes := [][3]float64{{7.5, 0, 1.5}, {7.5 * math.Cos(1), 7.5 * math.Sin(1), 1.5}}
	_, err := bossOnTop(t, holedDisc(t, holes...), 15, 0)
	require.NoError(t, err)
	_, err = bossOnTop(t, holedDisc(t, holes...), 15, 0.5)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "spans half its circle or more")
}
