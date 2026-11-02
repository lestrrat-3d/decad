package occupancy_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/occupancy"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// torusFrame places a spine circle of radius 2.4 about a skew axis far from
// the origin.
func torusFrame() (o, w, e0, e1 r3.Vec) {
	w, _ = r3.Vec{X: 1, Y: -2, Z: 2}.Normalize()
	e0, _ = r3.Vec{X: 2, Y: 1}.Normalize()
	e1 = w.Cross(e0)
	return r3.Vec{X: -140.5, Y: 22.25, Z: 600.125}, w, e0, e1
}

func torusPoint(o, w, e0, e1 r3.Vec, radius, rho, phi, theta float64) r3.Vec {
	s, c := math.Sincos(phi)
	radial := e0.Scale(c).Add(e1.Scale(s))
	st, ct := math.Sincos(theta)
	return o.Add(radial.Scale(radius + rho*ct)).Add(w.Scale(rho * st))
}

func insideSomePiece(t *testing.T, p r3.Vec, pieces []*occupancy.Piece) bool {
	t.Helper()
	for _, pc := range pieces {
		ok, err := occupancy.Misses([3]r3.Vec{p, p, p}, r3.Vec{}, pc, &occupancy.Budget{Limit: 1 << 40})
		require.NoError(t, err)
		if !ok {
			return true
		}
	}
	return false
}

func TestArcTubeLeafPiecesContainTheTube(t *testing.T) {
	t.Parallel()
	const radius, rho = 2.4, 0.6
	o, w, e0, e1 := torusFrame()
	for _, tc := range []struct {
		name       string
		phi0, phi1 float64
		full       bool
	}{{"full turn", 0, 0, true}, {"quarter arc", 0.3, 0.3 + math.Pi/2, false}} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			leaf, err := occupancy.ArcTubeLeaf(o, w, e0, e1, radius, rho, tc.phi0, tc.phi1, tc.full, 0, 1e-4, 64, 0.01)
			require.NoError(t, err)
			pieces, err := leaf.Pieces()
			require.NoError(t, err)
			span := tc.phi1 - tc.phi0
			if tc.full {
				span = 2 * math.Pi
			}
			for k := range 41 {
				phi := tc.phi0 + span*float64(k)/40
				for j := range 12 {
					theta := 2 * math.Pi * float64(j) / 12
					p := torusPoint(o, w, e0, e1, radius, rho*(1-1e-6), phi, theta)
					require.True(t, insideSomePiece(t, p, pieces), "true tube point at φ %v θ %v escaped every piece", phi, theta)
				}
			}
			// A point well outside the tube lies in no piece.
			out := torusPoint(o, w, e0, e1, radius, rho+0.01, tc.phi0+span/3, 1.0)
			require.False(t, insideSomePiece(t, out, pieces))
		})
	}
}

func TestArcTubeLeafBulges(t *testing.T) {
	t.Parallel()
	const radius, rho, delta = 2.4, 0.6, 0.05
	o, w, e0, e1 := torusFrame()
	leaf, err := occupancy.ArcTubeLeaf(o, w, e0, e1, radius, rho, 0, 0, true, 0, 1e-4, 64, delta)
	require.NoError(t, err)
	rec := occupancy.LeafRecord(leaf)
	const d = 0.1 // the facets' angular cell
	at := func(phi, theta float64) r3.Vec { return torusPoint(o, w, e0, e1, radius, rho, phi, theta) }
	tris := [][3]r3.Vec{
		// The outer equator: a locally convex patch, so a facet on its
		// stations stays inside the tube.
		{at(0, -d), at(d, -d), at(d, d)},
		// Beside the inner equator: the edge between the two corners at θ = π − d
		// leaves the tube at its midpoint (see inner below).
		{at(0, math.Pi-d), at(d, math.Pi+d), at(d, math.Pi-d)},
	}
	verts := make([]r3.Vec, 0, 6)
	idx := make([][3]int, 0, 2)
	for _, tr := range tris {
		idx = append(idx, [3]int{len(verts), len(verts) + 1, len(verts) + 2})
		verts = append(verts, tr[:]...)
	}
	bulges := rec.Bulges(verts, idx, []float64{delta, delta})
	// The bound charges the outer side c²/(2ρ) for the axis-distance defect
	// c = r·(1 − cos(d/2)), against the inner side's c.
	c := (radius + rho) * (1 - math.Cos(d/2))
	require.Less(t, bulges[0], c*c/rho, "an outer-equator facet barely pokes out")
	// That edge's corners sit at axis distance r1 = R − ρ·cos d and height
	// −ρ·sin d; its midpoint is closer to the axis by e = r1·(1 − cos(d/2)), so
	// it lies sqrt(ρ² + 2ρ·e·cos d + e²) from the spine.
	r1 := radius - rho*math.Cos(d)
	e := r1 * (1 - math.Cos(d/2))
	inner := math.Sqrt(rho*rho+2*rho*e*math.Cos(d)+e*e) - rho
	require.GreaterOrEqual(t, bulges[1], inner, "an inner-side facet pokes out at least as far as its edge midpoint")
	require.Less(t, bulges[1], 2*inner+1e-4)

	// A facet beyond a partial arc's end reads the facet's own bound.
	arc, err := occupancy.ArcTubeLeaf(o, w, e0, e1, radius, rho, 0, math.Pi/2, false, 0, 1e-4, 64, delta)
	require.NoError(t, err)
	past := occupancy.LeafRecord(arc).Bulges(
		[]r3.Vec{at(math.Pi, -d), at(math.Pi+d, -d), at(math.Pi+d, d)}, [][3]int{{0, 1, 2}}, []float64{delta})
	require.Equal(t, delta, past[0])
}

func TestHelixTubeLeafPiecesContainTheCoil(t *testing.T) {
	t.Parallel()
	const rc, zc, rho, turns, pitch = 2.1, 0.3, 0.4, 1.5, 5.0
	k := pitch / (2 * math.Pi)
	o, n, er, _ := torusFrame()
	et := n.Cross(er)
	leaf, err := occupancy.HelixTubeLeaf(o, n, er, et, -1, k, rc, zc, rho, 2*math.Pi*turns, 0, 1e-4, 32, 0.01)
	require.NoError(t, err)
	pieces, err := leaf.Pieces()
	require.NoError(t, err)
	phi := func(r, z, theta float64) r3.Vec {
		s, c := math.Sincos(theta)
		return o.Add(er.Scale(r * c)).Add(et.Scale(-r * s)).Add(n.Scale(z + k*theta))
	}
	for i := range 31 {
		theta := 2 * math.Pi * turns * float64(i) / 30
		for j := range 8 {
			a := 2 * math.Pi * float64(j) / 8
			sa, ca := math.Sincos(a)
			p := phi(rc+rho*(1-1e-6)*ca, zc+rho*(1-1e-6)*sa, theta)
			require.True(t, insideSomePiece(t, p, pieces), "coil point at θ %v, section angle %v escaped every piece", theta, a)
		}
	}
	// A point between two turns, on the centre radius, lies in no piece.
	require.False(t, insideSomePiece(t, phi(rc, zc+pitch/2, math.Pi), pieces))
}
