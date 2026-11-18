package pointquery

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// boxMesh collects quads into a triangle mesh, each wound to face its outward direction.
type boxMesh struct {
	verts []r3.Vec
	tris  [][3]int
}

func (m *boxMesh) quad(out r3.Vec, ps ...r3.Vec) {
	base := len(m.verts)
	m.verts = append(m.verts, ps...)
	for _, t := range [][3]int{{0, 1, 2}, {0, 2, 3}} {
		a, b, c := ps[t[0]], ps[t[1]], ps[t[2]]
		tri := [3]int{base + t[0], base + t[1], base + t[2]}
		if b.Sub(a).Cross(c.Sub(a)).Dot(out) < 0 {
			tri[1], tri[2] = tri[2], tri[1]
		}
		m.tris = append(m.tris, tri)
	}
}

// cube adds the faces of [-h, h] × [-l, l] × [-h, h], facing out when sign is 1 and in when it is -1, leaving out
// the ±y faces when open.
func (m *boxMesh) cube(h, l, sign float64, open bool) {
	v := func(x, y, z float64) r3.Vec { return r3.NewVec(x*h, y*l, z*h) }
	m.quad(r3.NewVec(sign, 0, 0), v(1, -1, -1), v(1, 1, -1), v(1, 1, 1), v(1, -1, 1))
	m.quad(r3.NewVec(-sign, 0, 0), v(-1, -1, -1), v(-1, 1, -1), v(-1, 1, 1), v(-1, -1, 1))
	m.quad(r3.NewVec(0, 0, sign), v(-1, -1, 1), v(1, -1, 1), v(1, 1, 1), v(-1, 1, 1))
	m.quad(r3.NewVec(0, 0, -sign), v(-1, -1, -1), v(1, -1, -1), v(1, 1, -1), v(-1, 1, -1))
	if !open {
		m.quad(r3.NewVec(0, sign, 0), v(-1, 1, -1), v(1, 1, -1), v(1, 1, 1), v(-1, 1, 1))
		m.quad(r3.NewVec(0, -sign, 0), v(-1, -1, -1), v(1, -1, -1), v(1, -1, 1), v(-1, -1, 1))
	}
}

// tunnelled is the cube [-5, 5]³ with a square tunnel |x|, |z| < 1 through it along y: the ±y faces are rings of
// four quads.
func tunnelled() *boxMesh {
	m := &boxMesh{}
	m.cube(5, 5, 1, true)
	m.cube(1, 5, -1, true)
	for _, y := range []float64{-5, 5} {
		out := r3.NewVec(0, y/5, 0)
		m.quad(out, r3.NewVec(-5, y, -5), r3.NewVec(5, y, -5), r3.NewVec(1, y, -1), r3.NewVec(-1, y, -1))
		m.quad(out, r3.NewVec(5, y, -5), r3.NewVec(5, y, 5), r3.NewVec(1, y, 1), r3.NewVec(1, y, -1))
		m.quad(out, r3.NewVec(5, y, 5), r3.NewVec(-5, y, 5), r3.NewVec(-1, y, 1), r3.NewVec(1, y, 1))
		m.quad(out, r3.NewVec(-5, y, 5), r3.NewVec(-5, y, -5), r3.NewVec(-1, y, -1), r3.NewVec(-1, y, 1))
	}
	return m
}

// TestLocateByFreePath checks the free-path proof of point-containment §2: a symmetric-difference bound far larger
// than any ball leaves only the path to decide.
func TestLocateByFreePath(t *testing.T) {
	t.Parallel()
	const huge = 1e6
	t.Run("a point in a tunnel escapes along it", func(t *testing.T) {
		t.Parallel()
		m := tunnelled()
		for _, p := range []r3.Vec{{}, r3.NewVec(0.5, 3, -0.5)} {
			got, err := Locate(t.Context(), p, m.verts, m.tris, 0.1, huge)
			require.NoError(t, err)
			require.Equal(t, Outside, got, "at %v", p)
		}
		// Inside the wall the held mesh says inside, which no free path decides.
		got, err := Locate(t.Context(), r3.NewVec(3, 0, 0), m.verts, m.tris, 0.1, huge)
		require.NoError(t, err)
		require.Equal(t, Undecided, got)
	})
	t.Run("a point in a closed cavity has no path out", func(t *testing.T) {
		t.Parallel()
		m := &boxMesh{}
		m.cube(5, 5, 1, false)
		m.cube(1, 1, -1, false)
		got, err := Locate(t.Context(), r3.Vec{}, m.verts, m.tris, 0.1, huge)
		require.NoError(t, err)
		require.Equal(t, Undecided, got)
		// A symmetric-difference bound below the ball's volume decides it by the ball alone.
		got, err = Locate(t.Context(), r3.Vec{}, m.verts, m.tris, 0.1, 0.5)
		require.NoError(t, err)
		require.Equal(t, Outside, got)
	})
	t.Run("a path that comes within the boundary bound of the mesh proves nothing", func(t *testing.T) {
		t.Parallel()
		m := tunnelled()
		// The tunnel's wall lies 1 from the axis; with B = 1 no ball about the axis is clear of it.
		got, err := Locate(t.Context(), r3.Vec{}, m.verts, m.tris, 1, huge)
		require.NoError(t, err)
		require.Equal(t, Undecided, got)
	})
	t.Run("an unsound boundary bound gives a wrong answer", func(t *testing.T) {
		t.Parallel()
		// Read the tunnelled mesh as a solid cube whose tunnel was never cut: the true boundary is the cube's, 4
		// from the tunnel's walls. The sound B = 4 leaves the axis undecided; an understated B = 0.1 lets the path
		// pass and answers outside, where the true cube holds the point. The proof is only as sound as B.
		m := tunnelled()
		got, err := Locate(t.Context(), r3.Vec{}, m.verts, m.tris, 4, huge)
		require.NoError(t, err)
		require.Equal(t, Undecided, got)
		got, err = Locate(t.Context(), r3.Vec{}, m.verts, m.tris, 0.1, huge)
		require.NoError(t, err)
		require.Equal(t, Outside, got, "an understated bound transfers the held answer to a body it does not bound")
	})
}
