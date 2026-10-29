package retainedblend_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

const axialProducer provenance.Producer = 9

func axialEnv() topo.Env {
	return topo.Env{Doc: &topo.Document{}, Ref: axialProducer, Alloc: topotest.Alloc{}}
}

func axialPlane(t *testing.T, role string, p0, p1, p2 r3.Vec) facetedrecord.Group {
	t.Helper()
	f, err := r3.NewFrame(p0, p1.Sub(p0), p2.Sub(p0))
	require.NoError(t, err)
	return facetedrecord.Group{
		Origins:  []provenance.Ref{{Producer: axialProducer, Role: role}},
		Surface:  surfacegeom.Plane{Frame: f},
		Retained: true,
		Planar:   true,
	}
}

// bevelledPlate is a 40 x 20 x 20 plate whose y=0, z=20 edge is cut by the
// plane z - y = 18, as a retained faceted payload. The top face is group 2;
// topDelta is the AxialDelta that group states.
func bevelledPlate(t *testing.T, topDelta float64) (*topo.Body, facetedrecord.Payload) {
	t.Helper()
	section := [][2]float64{{0, 0}, {20, 0}, {20, 20}, {2, 20}, {0, 18}}
	n := len(section)
	var fp facetedrecord.Payload
	for _, x := range []float64{0, 40} {
		for _, p := range section {
			fp.Verts = append(fp.Verts, r3.NewVec(x, p[0], p[1]))
			fp.VertexBound = append(fp.VertexBound, 0)
		}
	}
	tri := func(g, a, b, c int) {
		fp.Tris = append(fp.Tris, [3]int{a, b, c})
		fp.Src = append(fp.Src, g)
	}
	for i := range n {
		j := (i + 1) % n
		fp.Groups = append(fp.Groups, axialPlane(t, "side"+string(rune('0'+i)), fp.Verts[i], fp.Verts[j], fp.Verts[i+n]))
		tri(i, i, j, j+n)
		tri(i, i, j+n, i+n)
	}
	end, start := len(fp.Groups), len(fp.Groups)+1
	fp.Groups = append(fp.Groups,
		axialPlane(t, "capEnd", r3.NewVec(40, 0, 0), r3.NewVec(40, 1, 0), r3.NewVec(40, 0, 1)),
		axialPlane(t, "capStart", r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), r3.NewVec(0, 1, 0)))
	for k := 1; k < n-1; k++ {
		tri(end, n, n+k, n+k+1)
		tri(start, 0, k+1, k)
	}
	fp.Groups[2].HasAxialDelta, fp.Groups[2].AxialDelta = true, topDelta
	fp.DPair = r3.NewVec(40, 40, 40).Len()
	b, err := facetedrecord.Build(t.Context(), axialEnv(), fp)
	require.NoError(t, err)
	built, ok := b.Payload.(facetedrecord.Payload)
	require.True(t, ok)
	return b, built
}

func topBevelEdge(t *testing.T, b *topo.Body) *topo.Edge {
	t.Helper()
	for _, e := range topobuild.Edges(b) {
		a, c := e.Start.Position, e.End.Position
		if math.Abs(a.Y-2) < 1e-9 && math.Abs(a.Z-20) < 1e-9 && math.Abs(c.Y-2) < 1e-9 && a.X != c.X {
			return e
		}
	}
	require.FailNow(t, "no top bevel edge")
	return nil
}

func payloadOf(t *testing.T, b *topo.Body) facetedrecord.Payload {
	t.Helper()
	fp, ok := b.Payload.(facetedrecord.Payload)
	require.True(t, ok)
	return fp
}

// meshVolume is the signed volume of the held mesh.
func meshVolume(fp facetedrecord.Payload) float64 {
	vol := 0.0
	for _, tri := range fp.Tris {
		vol += fp.Verts[tri[0]].Dot(fp.Verts[tri[1]].Cross(fp.Verts[tri[2]])) / 6
	}
	return vol
}

// blendGroup is the group the rewrite added: the last one of the payload.
func blendGroup(fp facetedrecord.Payload) facetedrecord.Group { return fp.Groups[len(fp.Groups)-1] }

// requireSameShape checks that out has the vertices of exact to within tol and its volume.
func requireSameShape(t *testing.T, exact, out facetedrecord.Payload, tol float64) {
	t.Helper()
	require.Len(t, out.Verts, len(exact.Verts))
	for i, v := range exact.Verts {
		require.LessOrEqual(t, out.Verts[i].Sub(v).Len(), tol, "vertex %d", i)
	}
	require.InDelta(t, meshVolume(exact), meshVolume(out), tol*1e3)
}

// requireChargedBounds checks that every vertex bound of out covers its exact twin's and that the largest grew by
// at least delta.
func requireChargedBounds(t *testing.T, exact, out facetedrecord.Payload, delta float64) {
	t.Helper()
	require.Len(t, out.VertexBound, len(exact.VertexBound))
	grew, worst := 0, 0.0
	for i, b := range exact.VertexBound {
		require.GreaterOrEqual(t, out.VertexBound[i], b, "vertex %d", i)
		if out.VertexBound[i] > b {
			grew++
		}
		worst = max(worst, out.VertexBound[i]-b)
	}
	require.Positive(t, grew, "no vertex bound carries the plane displacement")
	require.GreaterOrEqual(t, worst, delta, "no vertex bound grew by the plane displacement")
}

func TestFilletChargesPlaneAxialDelta(t *testing.T) {
	t.Parallel()
	fillet := func(t *testing.T, delta float64) (*topo.Body, bool, error) {
		t.Helper()
		b, built := bevelledPlate(t, delta)
		return retainedblend.Fillet(t.Context(), axialEnv(), b, built, []*topo.Edge{topBevelEdge(t, b)}, 3, 0)
	}
	build := func(t *testing.T, delta float64) facetedrecord.Payload {
		t.Helper()
		out, recognized, err := fillet(t, delta)
		require.True(t, recognized)
		require.NoError(t, err, "a stated delta is charged, not refused")
		require.NotNil(t, out)
		return payloadOf(t, out)
	}

	t.Run("no delta publishes the retained cylinder", func(t *testing.T) {
		t.Parallel()
		blend := blendGroup(build(t, 0))
		require.True(t, blend.Retained)
		require.IsType(t, surfacegeom.Cylinder{}, blend.Surface)
		require.False(t, blend.HasAxialDelta)
	})
	for name, delta := range map[string]float64{"a unit-conversion sized": 2.9e-16, "a stated": 1e-6} {
		t.Run(name+" delta is charged and the cylinder is not published", func(t *testing.T) {
			t.Parallel()
			exact, out := build(t, 0), build(t, delta)
			requireChargedBounds(t, exact, out, delta)
			requireSameShape(t, exact, out, 1e-9)
			blend := blendGroup(out)
			require.False(t, blend.Retained, "a cylinder has no field for the displacement of its planes")
			require.Nil(t, blend.Surface)
		})
	}
	for name, delta := range map[string]float64{"negative": -1, "infinite": math.Inf(1), "NaN": math.NaN()} {
		t.Run("a "+name+" delta names no plane", func(t *testing.T) {
			t.Parallel()
			out, recognized, err := fillet(t, delta)
			require.False(t, recognized && err == nil, "a displacement no bound can cover was admitted")
			require.Nil(t, out)
		})
	}
}

func TestChamferChargesPlaneAxialDelta(t *testing.T) {
	t.Parallel()
	chamfer := func(t *testing.T, delta float64) facetedrecord.Payload {
		t.Helper()
		b, built := bevelledPlate(t, delta)
		out, recognized, err := retainedblend.Chamfer(t.Context(), axialEnv(), b, built,
			[]*topo.Edge{topBevelEdge(t, b)}, 1, 0, nil)
		require.True(t, recognized)
		require.NoError(t, err)
		return payloadOf(t, out)
	}

	exact := chamfer(t, 0)
	blend := blendGroup(exact)
	require.True(t, blend.Retained)
	require.IsType(t, surfacegeom.Plane{}, blend.Surface)
	require.False(t, blend.HasAxialDelta, "no charge, no stated displacement")

	for _, delta := range []float64{2.9e-16, 1e-6} {
		out := chamfer(t, delta)
		requireChargedBounds(t, exact, out, delta)
		requireSameShape(t, exact, out, 1e-9)
		blend := blendGroup(out)
		require.True(t, blend.Retained)
		require.IsType(t, surfacegeom.Plane{}, blend.Surface)
		require.True(t, blend.HasAxialDelta, "the bevel plane is built from displaced planes")
		require.GreaterOrEqual(t, blend.AxialDelta, delta)
	}
}
