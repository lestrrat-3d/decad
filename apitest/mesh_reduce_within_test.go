package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-import-design.md §12's public-surface tests for ReduceMeshWithin.

// uvSphere returns a closed UV sphere of the given radius about the origin, with rings of latitude and
// segments of longitude, wound outward.
func uvSphere(t *testing.T, radius float64, rings, segments int) *decad.MeshBody {
	t.Helper()
	v := []r3.Vec{{Z: radius}}
	for i := 1; i < rings; i++ {
		phi := math.Pi * float64(i) / float64(rings)
		for j := range segments {
			theta := 2 * math.Pi * float64(j) / float64(segments)
			v = append(v, r3.NewVec(radius*math.Sin(phi)*math.Cos(theta), radius*math.Sin(phi)*math.Sin(theta), radius*math.Cos(phi)))
		}
	}
	v = append(v, r3.Vec{Z: -radius})
	south := len(v) - 1
	ring := func(i, j int) int { return 1 + (i-1)*segments + j%segments }
	var tris [][3]int
	for j := range segments {
		tris = append(tris, [3]int{0, ring(1, j), ring(1, j+1)})
	}
	for i := 1; i < rings-1; i++ {
		for j := range segments {
			a, b, c, d := ring(i, j), ring(i+1, j), ring(i+1, j+1), ring(i, j+1)
			tris = append(tris, [3]int{a, b, c}, [3]int{a, c, d})
		}
	}
	for j := range segments {
		tris = append(tris, [3]int{south, ring(rings-1, j+1), ring(rings-1, j)})
	}
	m, err := decad.NewMeshBody(t.Context(), v, tris)
	require.NoError(t, err)
	return m
}

// closestOnTriangle is the distance from p to the triangle abc.
func closestOnTriangle(p, a, b, c r3.Vec) float64 {
	ab, ac, ap := b.Sub(a), c.Sub(a), p.Sub(a)
	d1, d2 := ab.Dot(ap), ac.Dot(ap)
	if d1 <= 0 && d2 <= 0 {
		return p.Sub(a).Len()
	}
	bp := p.Sub(b)
	d3, d4 := ab.Dot(bp), ac.Dot(bp)
	if d3 >= 0 && d4 <= d3 {
		return p.Sub(b).Len()
	}
	vc := d1*d4 - d3*d2
	if vc <= 0 && d1 >= 0 && d3 <= 0 {
		return p.Sub(a.Add(ab.Scale(d1 / (d1 - d3)))).Len()
	}
	cp := p.Sub(c)
	d5, d6 := ab.Dot(cp), ac.Dot(cp)
	if d6 >= 0 && d5 <= d6 {
		return p.Sub(c).Len()
	}
	vb := d5*d2 - d1*d6
	if vb <= 0 && d2 >= 0 && d6 <= 0 {
		return p.Sub(a.Add(ac.Scale(d2 / (d2 - d6)))).Len()
	}
	va := d3*d6 - d5*d4
	if va <= 0 && d4-d3 >= 0 && d5-d6 >= 0 {
		return p.Sub(b.Add(c.Sub(b).Scale((d4 - d3) / ((d4 - d3) + (d5 - d6))))).Len()
	}
	denom := 1 / (va + vb + vc)
	return p.Sub(a.Add(ab.Scale(vb * denom)).Add(ac.Scale(vc * denom))).Len()
}

// surfaceSamples returns the vertices, edge midpoints and triangle centroids of m.
func surfaceSamples(m *decad.MeshBody) []r3.Vec {
	v, tris := m.Vertices(), m.Triangles()
	var samples []r3.Vec
	samples = append(samples, v...)
	for _, t := range tris {
		a, b, c := v[t[0]], v[t[1]], v[t[2]]
		samples = append(samples, a.Add(b).Scale(0.5), b.Add(c).Scale(0.5), c.Add(a).Scale(0.5), a.Add(b).Add(c).Scale(1.0/3))
	}
	return samples
}

// farthest is the greatest distance from a surface sample of m to the surface of other.
func farthest(m, other *decad.MeshBody) float64 {
	ov, ot := other.Vertices(), other.Triangles()
	worst := 0.0
	for _, p := range surfaceSamples(m) {
		best := math.Inf(1)
		for _, t := range ot {
			best = math.Min(best, closestOnTriangle(p, ov[t[0]], ov[t[1]], ov[t[2]]))
		}
		worst = math.Max(worst, best)
	}
	return worst
}

// samplesWithin reports whether every surface sample of m lies within bound of other.
func samplesWithin(m, other *decad.MeshBody, bound float64) bool {
	if math.IsNaN(bound) || math.IsInf(bound, 0) {
		return false
	}
	ov, ot := other.Vertices(), other.Triangles()
	for _, p := range surfaceSamples(m) {
		found := false
		for _, tri := range ot {
			distance := closestOnTriangle(p, ov[tri[0]], ov[tri[1]], ov[tri[2]])
			if math.IsNaN(distance) || math.IsInf(distance, 0) {
				return false
			}
			if distance <= bound {
				found = true
				break
			}
		}
		if !found {
			return false
		}
	}
	return true
}

func TestReduceMeshWithinStaysWithinItsStatedDistance(t *testing.T) {
	t.Parallel()
	// Small meshes retain the exhaustive maximum as an independent comparison.
	small, larger := uvSphere(t, 10, 4, 8), uvSphere(t, 11, 4, 8)
	for _, pair := range [][2]*decad.MeshBody{{small, larger}, {larger, small}} {
		maxDistance := farthest(pair[0], pair[1])
		for _, b := range []float64{math.Nextafter(maxDistance, math.Inf(-1)), maxDistance,
			math.Nextafter(maxDistance, math.Inf(1))} {
			require.Equal(t, maxDistance <= b, samplesWithin(pair[0], pair[1], b), "bound=%g", b)
		}
		require.False(t, samplesWithin(pair[0], pair[1], math.Inf(1)))
		require.False(t, samplesWithin(pair[0], pair[1], math.NaN()))
	}
	sphere := uvSphere(t, 10, 40, 80)
	const distance = 1.0
	out, report, err := decad.ReduceMeshWithin(t.Context(), sphere, units.Millimeters(distance))
	require.NoError(t, err)
	require.Less(t, len(out.Triangles()), len(sphere.Triangles())/2, "the sphere thins well")
	bound := report.SurfaceBound.Base()
	require.Positive(t, bound)
	require.LessOrEqual(t, bound, distance)
	require.GreaterOrEqual(t, bound, report.MaxVertexMove.Base(), "the bound is above the greatest move")
	// Checked from outside, by brute force, in both directions.
	require.True(t, samplesWithin(sphere, out, bound), "the input is within the bound of the result")
	require.True(t, samplesWithin(out, sphere, bound), "and the result is within the bound of the input")
	// A tighter ask keeps more of the sphere.
	finer, fr, err := decad.ReduceMeshWithin(t.Context(), sphere, units.Millimeters(0.6))
	require.NoError(t, err)
	require.Greater(t, len(finer.Triangles()), len(out.Triangles()))
	require.LessOrEqual(t, fr.SurfaceBound.Base(), 0.6)
	require.True(t, samplesWithin(sphere, finer, fr.SurfaceBound.Base()))
}

func TestReduceMeshWithinThinsAFinelyCutBlock(t *testing.T) {
	t.Parallel()
	// A 20 x 20 x 4 block with each big face cut into an 8 x 8 grid of quads.
	v, tris := cubeMesh(r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 4))
	fine := subdividedBlock(t, v, tris)
	out, report, err := decad.ReduceMeshWithin(t.Context(), fine, units.Millimeters(2))
	require.NoError(t, err)
	require.Less(t, len(out.Triangles()), len(fine.Triangles())/2)
	require.True(t, samplesWithin(fine, out, report.SurfaceBound.Base()))
	require.True(t, samplesWithin(out, fine, report.SurfaceBound.Base()))
}

// subdividedBlock splits each triangle of the closed mesh into 64 by midpoint subdivision, three rounds.
func subdividedBlock(t *testing.T, v []r3.Vec, tris [][3]int) *decad.MeshBody {
	t.Helper()
	for range 3 {
		mid := map[[2]int]int{}
		midpoint := func(a, b int) int {
			key := [2]int{min(a, b), max(a, b)}
			if i, ok := mid[key]; ok {
				return i
			}
			v = append(v, v[a].Add(v[b]).Scale(0.5))
			mid[key] = len(v) - 1
			return len(v) - 1
		}
		var next [][3]int
		for _, tr := range tris {
			ab, bc, ca := midpoint(tr[0], tr[1]), midpoint(tr[1], tr[2]), midpoint(tr[2], tr[0])
			next = append(next, [3]int{tr[0], ab, ca}, [3]int{ab, tr[1], bc}, [3]int{ca, bc, tr[2]}, [3]int{ab, bc, ca})
		}
		tris = next
	}
	m, err := decad.NewMeshBody(t.Context(), v, tris)
	require.NoError(t, err)
	return m
}

func TestReduceMeshWithinRefusals(t *testing.T) {
	t.Parallel()
	sphere := uvSphere(t, 10, 12, 24)
	cases := map[string]struct {
		distance units.Value
		want     error
	}{
		"a zero distance":                {units.Millimeters(0), decad.ErrDegenerate},
		"a negative distance":            {units.Millimeters(-1), decad.ErrNegativeMagnitude},
		"a distance of angle kind":       {units.Degrees(5), decad.ErrUnitKind},
		"a distance past the whole mesh": {units.Millimeters(1000), decad.ErrUnsupported},
	}
	for name, tc := range cases {
		_, _, err := decad.ReduceMeshWithin(t.Context(), sphere, tc.distance)
		require.ErrorIs(t, err, tc.want, name)
	}
	_, _, err := decad.ReduceMeshWithin(nil, sphere, units.Millimeters(1)) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.ReduceMeshWithin(t.Context(), nil, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
