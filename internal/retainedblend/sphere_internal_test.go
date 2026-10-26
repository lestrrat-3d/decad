package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestCrossing pins the grazing gate of the plane-sphere rim
// (agent-docs/retained-edge-blend-design.md §4.5): a mesh Boolean cannot place a
// plane within 1e-7 of a sphere's tangent plane, so the gate is shown on the
// rim's numbers directly.
func TestCrossing(t *testing.T) {
	t.Parallel()
	const radius = 2.0
	require.NoError(t, crossing(3, radius), "a 60° crossing")
	for _, rr := range []float64{0, -1, (0.5 * minSine * radius) * (0.5 * minSine * radius)} {
		err := crossing(rr, radius)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "the plane meets the sphere at a grazing angle")
	}
}

// TestSphereWallInnerVertexBound pins the bound of a kept inner vertex of a
// curved wall (agent-docs/retained-edge-blend-design.md §4.5): a held vertex just
// beyond the cut plane whose true point lies in the removed zone must be
// bounded by its distance to the new boundary, not by its old bound. The wall
// is a hand-built band of a radius-2 sphere below the plane z = −1: its rim
// ring, an inner ring held 0.05 outside the sphere off points just inside the
// zone, and a lower boundary ring.
func TestSphereWallInnerVertexBound(t *testing.T) {
	t.Parallel()
	const (
		radius, plane, n = 2.0, -1.0, 12
		beta, r          = 0.05, 0.4
	)
	s := sphereRim{sphereRadius: radius}
	s.center, s.n, s.radius = r3.NewVec(0, 0, plane), r3.NewVec(0, 0, 1), math.Sqrt(radius*radius-plane*plane)
	s.u, _ = perpendicular(s.n)
	s.w = s.n.Cross(s.u)
	s.kappa, s.side, s.convex, s.scale = -1, -1, true, radius
	spec := blendSpec{fillet: true, r: r}
	w0 := newWork(facetedrecord.Payload{}, s.rim, spec)
	m, err := s.section(spec, w0.tau)
	require.NoError(t, err)

	// The inner ring's true points sit 0.005 inside the zone; the held
	// vertices stand beta outside the sphere off them.
	zp := plane + m.zB + 0.005
	rp := math.Sqrt(radius*radius - zp*zp)
	zLow := -1.9
	rLow := math.Sqrt(radius*radius - zLow*zLow)
	var fp facetedrecord.Payload
	for k := range 3 * n {
		ring, a := k/n, 2*math.Pi*float64(k%n)/n
		var rho, z, b float64
		switch ring {
		case 0:
			rho, z, b = s.radius, plane, 1e-6
		case 1:
			rho, z, b = rp*(1+beta/radius), zp*(1+beta/radius), beta
		default:
			rho, z, b = rLow, zLow, 1e-6
		}
		fp.Verts = append(fp.Verts, r3.NewVec(rho*math.Cos(a), rho*math.Sin(a), z))
		fp.VertexBound = append(fp.VertexBound, b)
	}
	for ring := range 2 {
		for k := range n {
			a, b := ring*n+k, ring*n+(k+1)%n
			c, d := a+n, b+n
			fp.Tris = append(fp.Tris, [3]int{a, b, d}, [3]int{a, d, c})
		}
	}
	fp.FaceOf = make([]int, len(fp.Tris))
	fp.Src = make([]int, len(fp.Tris))
	w := newWork(fp, s.rim, spec)
	w.dir = 1 // rewritePlane sets the rim loop's sense
	_, _, err = w.rewriteWall(0, s.side*m.zB, radius, func(p r3.Vec) float64 { return m.dist(s.meridianOf(p)) })
	require.NoError(t, err)

	// The new boundary in the meridian: the torus arc from T_A to T_B and the
	// sphere's arc from T_B down to its pole. The nearest point of a circular
	// arc is the radial foot when the point's angle lies on the arc, else an
	// end.
	arcDist := func(rho, z, cr, cz, rad, lo, hi float64) float64 {
		lo, hi = min(lo, hi), max(lo, hi)
		if a := math.Atan2(z-cz, rho-cr); a >= lo && a <= hi {
			return math.Abs(math.Hypot(rho-cr, z-cz) - rad)
		}
		return math.Min(math.Hypot(rho-cr-rad*math.Cos(lo), z-cz-rad*math.Sin(lo)),
			math.Hypot(rho-cr-rad*math.Cos(hi), z-cz-rad*math.Sin(hi)))
	}
	alpha0 := math.Atan2(m.zA-m.zC, m.rhoA-m.rhoC)
	alpha1 := math.Atan2(m.zB-m.zC, m.rhoB-m.rhoC)
	theta0 := math.Atan2(m.zB+plane, m.rhoB)
	for v := n; v < 2*n; v++ {
		rho, z := s.meridianOf(fp.Verts[v])
		nearest := math.Min(arcDist(rho, z, m.rhoC, m.zC, r, alpha0, alpha1),
			arcDist(rho, z, 0, -plane, radius, -math.Pi/2, theta0))
		require.Greater(t, nearest, beta, "the fixture puts the new boundary beyond the old bound")
		require.GreaterOrEqual(t, w.beta[v], nearest)
	}
}
