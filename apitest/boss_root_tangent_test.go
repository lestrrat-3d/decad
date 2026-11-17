package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers the C3 tangent ends of agent-docs/boss-root-overflow-design.md §5.5: a root arc that ends where a
// window's corner round touches the boss wall (the spoked disc at Ø14) or where the fork notch's gap round touches the
// notch wall. Rule CO (§2.2) gives the result S ∪ reg(W ∩ P): S is the body without the fillet, and the fillet's bead
// stands over the floor between the wall and the rounds' extensions. The tests integrate the bead's volume
// independently of production code and observe the caps on the rounds through point containment.

// panelQuadrature integrates f over [lo, hi] with panels equal panels of the 8-point Gauss–Legendre rule.
func panelQuadrature(f func(float64) float64, lo, hi float64, panels int) float64 {
	nodes, weights := gaussLegendre(8)
	sum := 0.0
	w := (hi - lo) / float64(panels)
	for p := range panels {
		mid := lo + (float64(p)+0.5)*w
		for i, x := range nodes {
			sum += weights[i] * f(mid+x*w/2)
		}
	}
	return sum * w / 2
}

// filletBeadHeight is h(s) = ρ − √(ρ² − (ρ − s)²), the height of a fillet of radius rho above the floor at distance s
// from the wall, written for s = u² as ρ − u·√(2ρ − u²).
func filletBeadHeight(rho, u float64) float64 { return rho - u*math.Sqrt(2*rho-u*u) }

// roundCrossing is α(r) = acos((r² + d² − r_G²)/(2rd)), the angle at the wall's axis between the round's centre line and
// the point where the circle of radius r meets the round of radius rG centred d from the axis, from
// sin²(α/2) = (rG − d + r)(rG + d − r)/(4rd), whose two factors the caller gives as lo·hi so that the one that vanishes
// at the wall is exact.
func roundCrossing(lo, hi, r, d float64) float64 {
	return 2 * math.Asin(math.Sqrt(lo*hi/(4*r*d)))
}

// beadIntegral integrates the bead's volume over one arc: ∫_0^ρ h(s)·span(s)·r(s) ds, with s = u², and returns it with
// the difference between a 4-panel and an 8-panel rule, the quadrature error estimate.
func beadIntegral(rho float64, span, radius func(u float64) float64) (float64, float64) {
	f := func(u float64) float64 { return filletBeadHeight(rho, u) * span(u) * radius(u) * 2 * u }
	coarse := panelQuadrature(f, 0, math.Sqrt(rho), 4)
	fine := panelQuadrature(f, 0, math.Sqrt(rho), 8)
	return fine, math.Abs(fine - coarse)
}

// requireSolidVerified requires out to be a solid that tessellates with every audit, and returns that mesh.
func requireSolidVerified(t *testing.T, out *decad.Body, tol float64) *decad.Mesh {
	t.Helper()
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(tol), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	return mesh
}

// meshContains reports whether p lies inside the closed mesh, by the parity of its crossings with a ray. Three rays in
// unrelated directions must agree, which excludes a ray that grazes an edge. LocatePoint cannot answer here: it needs a
// ball of radius distance − bound around p whose volume exceeds the result's volume bound, which is larger than the
// whole cap, so the test reads the verified mesh, which lies within Bound of the body, itself.
func meshContains(t *testing.T, mesh *decad.Mesh, p r3.Vec) bool {
	t.Helper()
	verts, tris := mesh.Vertices(), mesh.Triangles()
	dirs := []r3.Vec{{X: 0.5773, Y: 0.3217, Z: 0.7501}, {X: -0.2713, Y: 0.8419, Z: -0.4647}, {X: 0.1161, Y: -0.6143, Z: 0.7807}}
	var votes []bool
	for _, d := range dirs {
		crossings := 0
		for _, tri := range tris {
			a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
			e1, e2 := b.Sub(a), c.Sub(a)
			h := d.Cross(e2)
			det := e1.Dot(h)
			if math.Abs(det) < 1e-14 {
				continue
			}
			s := p.Sub(a)
			u := s.Dot(h) / det
			if u < 0 || u > 1 {
				continue
			}
			q := s.Cross(e1)
			v := d.Dot(q) / det
			if v < 0 || u+v > 1 {
				continue
			}
			if e2.Dot(q)/det > 0 {
				crossings++
			}
		}
		votes = append(votes, crossings%2 == 1)
	}
	require.Equal(t, votes[0], votes[1], "rays disagree at %v", p)
	require.Equal(t, votes[0], votes[2], "rays disagree at %v", p)
	return votes[0]
}

const capOffset = 0.03

// requireNearestIsCap requires the body's nearest surface point to p, a point inside the round of radius rG about
// (gx, gy), to lie on that round's cylinder at p's own height, within the held bounds. Only a cap standing above the
// floor provides a vertical face there: the floor's own edge lies 0.04 lower.
func requireNearestIsCap(t *testing.T, out *decad.Body, p r3.Vec, gx, gy, rG, meshBound float64) {
	t.Helper()
	cp, err := out.ClosestPoint(t.Context(), p, units.Millimeters(0.01))
	require.NoError(t, err)
	slack := cp.OffSurface.Base() + meshBound
	t.Logf("nearest surface of %v: %v, off surface %.3g, mesh bound %.3g", p, cp.Point, cp.OffSurface.Base(), meshBound)
	require.InDelta(t, rG, math.Hypot(cp.Point.X-gx, cp.Point.Y-gy), slack, "the nearest point lies on the round")
	require.InDelta(t, p.Z, cp.Point.Z, slack, "the nearest point lies at the probe's height")
	require.InDelta(t, rG-math.Hypot(p.X-gx, p.Y-gy), cp.Point.Sub(p).Len(), 2*slack, "its distance is the offset")
}

// Geometry of spokedHubDisc at Ø14: the boss wall is the windows' inner wall, radius 7, and each window corner round has
// radius 2 and touches it from outside, so its centre is 9 from the axis.
const (
	tangentHub    = 7.0
	tangentRound  = 2.0
	tangentCentre = tangentHub + tangentRound
	tangentArm    = 2.0 // half width of an arm
	tangentTop    = 5.0 // the arm top
)

// spokedHubBead integrates, over the four arms, the volume rule CO adds to the Ø14 boss for a root fillet of radius
// rho, and returns it with the quadrature's error estimate. The round's centre is tangentRound from the side line,
// which is tangentArm from the arm's radial line, so tangentRound + tangentArm from it, and tangentCentre from the
// axis: the angle asin((tangentRound+tangentArm)/tangentCentre) from the arm's radial line, alpha0 from the window's
// middle. The root arc is the arm's part of the wall's circle: π/2 less the window's half width alpha0 on each side.
func spokedHubBead(rho float64) (alpha0, bead, quadErr float64) {
	alpha0 = math.Pi/4 - math.Asin((tangentRound+tangentArm)/tangentCentre)
	phi0 := math.Pi/2 - 2*alpha0
	arc, quad := beadIntegral(rho,
		func(u float64) float64 {
			return phi0 - 2*roundCrossing(u*u, 2*tangentRound-u*u, tangentHub+u*u, tangentCentre)
		},
		func(u float64) float64 { return tangentHub + u*u })
	return alpha0, 4 * arc, 4 * quad
}

// TestBossRootTangentSpokedHub builds the Ø14 boss with a 0.5 root fillet on the spoked disc. Each of the four root
// arcs ends where a window corner round touches the boss wall.
func TestBossRootTangentSpokedHub(t *testing.T) {
	t.Parallel()
	const (
		hub, rG, dRound, top = tangentHub, tangentRound, tangentCentre, tangentTop
		rho                  = 0.5
	)
	alpha0, bead, quadErr := spokedHubBead(rho)

	_, plain := spokedHubDisc(t)
	s, err := bossOnTop(t, plain, 14, 0)
	require.NoError(t, err)
	vs, bs := volumeAndBound(t, s)

	_, disc := spokedHubDisc(t)
	out, err := bossOnTop(t, disc, 14, 0.5)
	require.NoError(t, err)
	require.True(t, out.IsSolid())
	mesh := requireSolidVerified(t, out, 0.01)
	meshBound := mesh.Bound().Base()

	v, bound := volumeAndBound(t, out)
	t.Logf("V(S) %.9g (bound %g), bead %.9g (quadrature error %.3g), V(result) %.9g (bound %g), "+
		"|V(result) - V(S) - bead| %.3g, allowance %.3g, mesh bound %g",
		vs, bs, bead, quadErr, v, bound, math.Abs(v-vs-bead), bound+bs+quadErr, meshBound)
	require.Less(t, quadErr, 1e-9, "the quadrature resolves the bead far below the bound")
	require.Greater(t, bead, 1.0, "a result without the bead moves the volume by more than the allowance")
	require.LessOrEqual(t, math.Abs(v-(vs+bead)), bound+bs+quadErr)

	require.Greater(t, capOffset, meshBound)
	// The caps: at radius 7.2 the window's boundary is the round, and the bead stands 0.1 high there. A point 0.03
	// into the window is outside the solid and one 0.03 toward the material is inside, at height 0.04 above the arm.
	const r = 7.2
	for k := range 4 {
		mid := float64(k) * math.Pi / 2 // window k's middle direction
		for _, side := range []float64{-1, 1} {
			gx, gy := dRound*math.Cos(mid+side*alpha0), dRound*math.Sin(mid+side*alpha0)
			theta := mid + side*(alpha0+roundCrossing(r-hub, 2*rG-(r-hub), r, dRound))
			px, py := r*math.Cos(theta), r*math.Sin(theta)
			ux, uy := (px-gx)/rG, (py-gy)/rG
			require.InDelta(t, 1, math.Hypot(ux, uy), 1e-9)
			at := func(off float64) r3.Vec {
				return r3.Vec{X: gx + (rG+off)*ux, Y: gy + (rG+off)*uy, Z: top + 0.04}
			}
			require.False(t, meshContains(t, mesh, at(-capOffset)), "into the window beyond the cap")
			requireNearestIsCap(t, out, at(-capOffset), gx, gy, rG, meshBound)
			require.True(t, meshContains(t, mesh, at(capOffset)), "in the bead beside the cap")
		}
	}
	// The fillet itself: at 0.1 from the wall it stands 0.2 above an arm's top.
	for k := range 4 {
		az := math.Pi/4 + float64(k)*math.Pi/2
		x, y := 7.1*math.Cos(az), 7.1*math.Sin(az)
		require.True(t, meshContains(t, mesh, r3.Vec{X: x, Y: y, Z: top + 0.1}), "inside the fillet")
		require.False(t, meshContains(t, mesh, r3.Vec{X: x, Y: y, Z: top + 0.3}), "above the fillet")
	}
}

// TestBossRootTangentFork cuts the lever fork's notch and fillets the notch floor's arc, which ends where the gap round
// (radius r_G, centre d = r_n − r_G from the notch's axis) touches the notch wall (radius r_n) from inside.
func TestBossRootTangentFork(t *testing.T) {
	t.Parallel()
	const (
		ky     = 38.874650636459869 // leverForkSketch's centre line
		rn     = 1.6676711944252536 // the notch wall's radius, and the gap round's outer end
		gc     = 1.1338355972126268 // the gap round's centre from the notch's axis
		rho    = 0.5                // the fillet's radius
		rG     = rn - gc            // the gap round's radius
		dG     = gc                 // the gap round's centre distance
		floorZ = 2.5                // the notch tool's lower face
	)
	mm := units.Millimeters
	doc := decad.New()
	fork := forkBody(t, doc, 0, mm(6.5), 0, leverForkSketch(false, forkOptions{}))
	notch := forkBody(t, doc, 2.5, mm(5), 0, leverForkSketch(true, forkOptions{}))
	cut, err := decad.Cut(t.Context(), fork, notch)
	require.NoError(t, err)
	vc, bc := volumeAndBound(t, cut)

	out, err := decad.Fillet(t.Context(), cut, decad.Edges(decad.Reflex(), decad.Circular()).Exactly(1), mm(rho))
	require.NoError(t, err)
	require.True(t, out.IsSolid())
	mesh := requireSolidVerified(t, out, 0.01)
	meshBound := mesh.Bound().Base()

	// The ring stands over the floor between r_n − ρ and r_n; its angular extent is the semicircle less the round's
	// reach α(r) at each end, where the round's circle of radius r meets the floor row.
	arc, quad := beadIntegral(rho,
		func(u float64) float64 {
			return math.Pi - 2*roundCrossing(rG-dG+rn-u*u, u*u, rn-u*u, dG)
		},
		func(u float64) float64 { return rn - u*u })
	v, bound := volumeAndBound(t, out)
	t.Logf("V(cut) %.9g (bound %g), bead %.9g (quadrature error %.3g), V(result) %.9g (bound %g), "+
		"|V(result) - V(cut) - bead| %.3g, allowance %.3g, mesh bound %g",
		vc, bc, arc, quad, v, bound, math.Abs(v-vc-arc), bound+bc+quad, meshBound)
	require.Less(t, quad, 1e-9)
	require.Greater(t, arc, 0.1)
	require.LessOrEqual(t, math.Abs(v-(vc+arc)), bound+bc+quad)
	require.Greater(t, capOffset, meshBound)

	// The caps: at row radius r_n − 0.2 the bead stands 0.1 high over the floor, and the gap round's extension above the
	// floor bounds it where the row meets the round.
	r := rn - 0.2
	alpha := roundCrossing(rG-dG+r, rG+dG-r, r, dG)
	for _, side := range []float64{-1, 1} {
		gx := side * dG
		px, py := side*r*math.Cos(alpha), ky-r*math.Sin(alpha)
		ux, uy := (px-gx)/rG, (py-ky)/rG
		require.InDelta(t, 1, math.Hypot(ux, uy), 1e-9)
		at := func(off float64) r3.Vec {
			return r3.Vec{X: gx + (rG+off)*ux, Y: ky + (rG+off)*uy, Z: floorZ + 0.04}
		}
		require.False(t, meshContains(t, mesh, at(-capOffset)), "into the gap beyond the cap")
		requireNearestIsCap(t, out, at(-capOffset), gx, ky, rG, meshBound)
		require.True(t, meshContains(t, mesh, at(capOffset)), "in the bead beside the cap")
	}
	// The fillet itself, mid-semicircle: 0.1 from the wall it stands 0.2 above the floor.
	require.True(t, meshContains(t, mesh, r3.Vec{X: 0, Y: ky - (rn - 0.1), Z: floorZ + 0.1}), "inside the fillet")
	require.False(t, meshContains(t, mesh, r3.Vec{X: 0, Y: ky - (rn - 0.1), Z: floorZ + 0.3}), "above the fillet")
}

// kinkedHubDisc is spokedHubDisc's disc with every window drawn from fixed points, and with the window's corner
// rounds at the hub of radius 2 tangent to the side line but centred shrink closer to the axis than the round tangent to
// the hub arc (radius 7) would be, so that each crosses the hub arc transversally at the joint. The rim's corner rounds
// stay tangent to the rim arc (radius 18).
func kinkedHubDisc(t *testing.T, shrink float64) *decad.Body {
	t.Helper()
	const (
		hub, rim = 7.0, 18.0
		arm      = 2.0 // an arm's half width
		round    = 2.0
	)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	fixed := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	o := fixed(0, 0)
	s.AddConstraint(sketch.NewRadius(s.CreateCircle(o, 21), 21))
	// minor draws the minor arc about c between p and q.
	minor := func(c, p, q *sketch.Point) {
		pa := math.Atan2(p.Y()-c.Y(), p.X()-c.X())
		qa := math.Atan2(q.Y()-c.Y(), q.X()-c.X())
		if math.Mod(qa-pa+4*math.Pi, 2*math.Pi) > math.Pi {
			p, q = q, p
		}
		s.CreateArc(c, p, q)
	}
	// corner builds one window corner round for the arm in direction a0 with the window on side sgn (+1 toward larger
	// azimuth), against the circle of radius rad whose round centre lies dc from the axis. It returns the joint with the
	// circle and the tangent point on the side line, and draws the round between them.
	corner := func(a0, sgn, rad, dc float64) (joint, onLine *sketch.Point) {
		ux, uy := math.Cos(a0), math.Sin(a0)
		nx, ny := -sgn*uy, sgn*ux
		a := math.Sqrt(dc*dc - (arm+round)*(arm+round))
		cx, cy := a*ux+(arm+round)*nx, a*uy+(arm+round)*ny
		centre := fixed(cx, cy)
		onLine = fixed(a*ux+arm*nx, a*uy+arm*ny)
		ca := math.Atan2(cy, cx)
		beta := math.Acos(math.Min(1, (rad*rad+dc*dc-round*round)/(2*rad*dc)))
		// Of the two points where the round meets the circle, the joint is the one from which the round runs to the
		// side line outside the circle.
		for _, h := range []float64{ca - beta, ca + beta} {
			hx, hy := rad*math.Cos(h), rad*math.Sin(h)
			pa := math.Atan2(hy-cy, hx-cx)
			qa := math.Atan2(onLine.Y()-cy, onLine.X()-cx)
			sweep := math.Mod(qa-pa+4*math.Pi, 2*math.Pi)
			if sweep > math.Pi {
				sweep -= 2 * math.Pi
			}
			outside := true
			minR := math.Inf(1)
			for i := range 21 {
				ang := pa + sweep*float64(i)/20
				px, py := cx+round*math.Cos(ang), cy+round*math.Sin(ang)
				minR = math.Min(minR, math.Hypot(px, py))
				// the hub's circle bounds the window from below, the rim's from above
				if (rad == hub && math.Hypot(px, py) < rad-min(1e-9, dc*1e-3*shrink+1e-14)) || (rad == rim && math.Hypot(px, py) > rad+1e-9) {
					outside = false
				}
			}
			if outside {
				joint = fixed(hx, hy)
				break
			}
		}
		require.NotNil(t, joint, "the round meets the circle of radius %g nowhere it can end", rad)
		minor(centre, joint, onLine)
		return joint, onLine
	}
	for k := range 4 {
		s1 := math.Pi/4 + math.Pi/2*float64(k)
		s2 := s1 + math.Pi/2
		h1, t1 := corner(s1, 1, hub, hub+round-shrink)
		h2, t2 := corner(s2, -1, hub, hub+round-shrink)
		r1, u1 := corner(s1, 1, rim, rim-round)
		r2, u2 := corner(s2, -1, rim, rim-round)
		s.CreateArc(o, h1, h2)
		s.CreateArc(o, r1, r2)
		s.CreateLine(t1, u1)
		s.CreateLine(t2, u2)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 4 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	body, err := decad.Extrude(t.Context(), decad.New(), s, profile, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestBossRootTangentRefusesKinkedJoint draws the spoked disc's hub corners as rounds that cross the hub arc at an
// angle instead of touching it, from the hand-drawn disc whose shrink 0 is tangent and builds. The Ø14 boss then ends
// each root arc at a joint that neither tangent proof admits, so the fillet refuses. A kink of 1e-8 is still read by
// the junction rule as nearly tangent; 3e-8 reaches the charged-arc gate that only a proven corner passes; a kink the
// union's shared-span gate cannot resolve takes the mesh path, whose boss has no root edge to fillet. The sketch's
// arrangement does not return a valid profile for kinks between 1e-7 and 3e-5.
func TestBossRootTangentRefusesKinkedJoint(t *testing.T) {
	t.Parallel()
	t.Run("tangent control builds", func(t *testing.T) {
		t.Parallel()
		out, err := bossOnTop(t, kinkedHubDisc(t, 0), 14, 0.5)
		require.NoError(t, err)
		require.True(t, out.IsSolid())
	})
	for _, tc := range []struct {
		shrink float64
		text   string
	}{
		{1e-8, "is nearly tangent and the tags cannot decide it"},
		{3e-8, "an arc on a wall whose tag states a displacement is not built"},
		{6e-5, "the boss's root circle is not an edge of the result"},
	} {
		t.Run(fmt.Sprintf("shrink %g", tc.shrink), func(t *testing.T) {
			t.Parallel()
			plain, err := bossOnTop(t, kinkedHubDisc(t, tc.shrink), 14, 0)
			require.NoError(t, err)
			require.True(t, plain.IsSolid())
			_, err = bossOnTop(t, kinkedHubDisc(t, tc.shrink), 14, 0.5)
			t.Logf("refusal: %v", err)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, tc.text)
		})
	}
}
