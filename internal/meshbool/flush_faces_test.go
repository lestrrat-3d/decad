package meshbool

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/flushface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// flushBoxTop is the source id of the box's top face; with a split top, the second top facet is face flushBoxTop+1.
const flushBoxTop = 5

// flushBox is the box [0, 2] × [0, 3] × [0, 1] held as a mesh whose faces are Plane faces, with vertex bounds beta.
// Corner i has x = 2 when bit 0 is set, y = 3 when bit 1 is, z = 1 when bit 2 is. carrier lists the faces whose facets
// read as carrier-exact (a zero slab, as a retained plane's facets do); every other facet keeps its facet bound as
// its slab. A split top gives the top's second facet its own source face. perVertex publishes beta as the mesh's
// per-vertex record; without it the bounds read as an analytic payload's.
func flushBox(t *testing.T, beta []float64, carrier map[int]bool, splitTop, perVertex bool) *flushOperand {
	t.Helper()
	verts := make([]r3.Vec, 8)
	for i := range verts {
		verts[i] = r3.Vec{X: float64(i&1) * 2, Y: float64(i>>1&1) * 3, Z: float64(i >> 2 & 1)}
	}
	quads := [][4]int{{0, 4, 6, 2}, {1, 3, 7, 5}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 2, 3, 1}, {4, 5, 7, 6}}
	var tris [][3]int
	var src []int
	var faces []*topo.Face
	sourceOf := map[int]*topo.Face{}
	face := func(s int, q [4]int) *topo.Face {
		if f, ok := sourceOf[s]; ok {
			return f
		}
		a, b, d := verts[q[0]], verts[q[1]], verts[q[3]]
		frame, err := r3.NewFrame(a, b.Sub(a), d.Sub(a))
		require.NoError(t, err)
		f := &topo.Face{Surface: surfacegeom.Plane{Frame: frame}}
		sourceOf[s] = f
		return f
	}
	for s, q := range quads {
		second := s
		if splitTop && s == flushBoxTop {
			second = flushBoxTop + 1
		}
		tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
		src = append(src, s, second)
		faces = append(faces, face(s, q), face(second, q))
	}
	bm, err := PrepContext(t.Context(), verts, tris, src)
	require.NoError(t, err)
	bm.SetVertexBounds(beta)
	for f, s := range src {
		if carrier[s] {
			bm.SlabBound[f] = 0
		}
	}
	m := &topo.Mesh{Vertices: verts, Triangles: tris, Source: faces}
	if perVertex {
		m.VertexBound = beta
	}
	return newFlushOperand(m, bm, func(*topo.Face) bool { return true })
}

// flushOutline reads op's top face as a carrier face and returns its outline.
func flushOutline(t *testing.T, op *flushOperand) flushface.Outline {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	f := op.head(flushBoxTop)
	require.NotNil(t, f)
	carrier, err := op.carrierOf(budget, f)
	require.NoError(t, err)
	require.True(t, carrier, "the top is a carrier face")
	require.NoError(t, op.readOutline(budget, f))
	return f.outline
}

// flushEdge returns the outline edge from a to b, in either direction.
func flushEdge(t *testing.T, o flushface.Outline, a, b r3.Vec) flushface.Edge {
	t.Helper()
	for _, e := range o.Edges {
		if e.A == a && e.B == b || e.A == b && e.B == a {
			return e
		}
	}
	require.Failf(t, "no outline edge", "%v to %v", a, b)
	return flushface.Edge{}
}

// flushRadius returns the radius the outline gives the point v.
func flushRadius(o flushface.Outline, v r3.Vec) float64 {
	for _, e := range o.Edges {
		if e.A == v {
			return e.RA
		}
		if e.B == v {
			return e.RB
		}
	}
	return math.NaN()
}

func TestFlushOutline(t *testing.T) {
	t.Parallel()
	all := map[int]bool{0: true, 1: true, 2: true, 3: true, 4: true, 5: true, 6: true}
	uniform := make([]float64, 8)
	for i := range uniform {
		uniform[i] = 1e-6
	}
	top := [4]r3.Vec{{Z: 1}, {X: 2, Z: 1}, {X: 2, Y: 3, Z: 1}, {Y: 3, Z: 1}}
	t.Run("an edge between carrier planes is a line edge", func(t *testing.T) {
		t.Parallel()
		o := flushOutline(t, flushBox(t, uniform, all, false, true))
		require.Len(t, o.Edges, 4)
		require.Equal(t, 2, o.Axis)
		for k := range top {
			e := flushEdge(t, o, top[k], top[(k+1)%4])
			require.True(t, e.Line, "edge %d", k)
			require.True(t, e.Straight, "edge %d", k)
		}
		// Two line edges on different lines fix each corner exactly.
		for _, v := range top {
			require.Zero(t, flushRadius(o, v))
		}
		require.Zero(t, o.Reach())
	})
	t.Run("the seam between coplanar top faces is not straight", func(t *testing.T) {
		t.Parallel()
		o := flushOutline(t, flushBox(t, uniform, all, true, true))
		seam := flushEdge(t, o, top[0], top[2])
		require.False(t, seam.Straight)
		require.False(t, seam.Line)
		// The rims to the side faces stay line edges.
		require.True(t, flushEdge(t, o, top[0], top[1]).Line)
	})
	raised := make([]float64, 8)
	const b = 1e-4
	raised[7] = b
	// The top reads as carrier-exact; the side faces beside vertex 7 carry its bound and do not, so the top's rims to
	// them are straight by the side tags but no line edges.
	sides := map[int]bool{flushBoxTop: true}
	t.Run("a faceted corner over a seam takes the seam radius", func(t *testing.T) {
		t.Parallel()
		o := flushOutline(t, flushBox(t, raised, sides, false, true))
		for k := range top {
			e := flushEdge(t, o, top[k], top[(k+1)%4])
			require.True(t, e.Straight, "edge %d", k)
		}
		require.False(t, flushEdge(t, o, top[1], top[2]).Line)
		require.False(t, flushEdge(t, o, top[2], top[3]).Line)
		// The side faces meet along the vertical edge from the corner to vertex 3, a seam as wide as b whose far end
		// lies 1 below the top: ρ = b·(1 + 1/1), under §4.3's √3·(b + b).
		rho := flushRadius(o, top[2])
		want := 2 * b
		require.GreaterOrEqual(t, rho, want)
		require.LessOrEqual(t, rho, want*(1+1e-8))
		require.Zero(t, flushRadius(o, top[0]))
		require.Equal(t, rho, o.Reach())
	})
	t.Run("a faceted corner's radius is the strip bound", func(t *testing.T) {
		t.Parallel()
		// Vertex 3, the seam's far end, carries a bound past the box's height, so the seam gives no reading.
		wide := slices.Clone(raised)
		wide[3] = 2
		o := flushOutline(t, flushBox(t, wide, sides, false, true))
		// The corner's two rims are at right angles, each as wide as b: ρ = √3·(b + b)/sin φ_lo with sin φ_lo a few
		// ulps under 1.
		rho := flushRadius(o, top[2])
		want := math.Sqrt(3) * 2 * b
		require.GreaterOrEqual(t, rho, want)
		require.LessOrEqual(t, rho, want*(1+1e-12))
		require.Zero(t, flushRadius(o, top[0]))
		require.Equal(t, rho, o.Reach())
	})
	t.Run("an analytic corner's radius is its bound", func(t *testing.T) {
		t.Parallel()
		o := flushOutline(t, flushBox(t, raised, sides, false, false))
		require.Equal(t, b, flushRadius(o, top[2]))
		require.Equal(t, b, o.Reach())
	})
}
