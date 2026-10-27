package tessellation_test

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// windingCube is the closed unit cube [0,1]³ wound outward, its eight corners
// indexed by their x, y and z bits.
func windingCube() ([]r3.Vec, [][3]int) {
	vertices := make([]r3.Vec, 8)
	for i := range vertices {
		vertices[i] = r3.NewVec(float64(i&1), float64(i>>1&1), float64(i>>2&1))
	}
	quads := [][4]int{
		{0, 2, 3, 1}, // z = 0, outward -z
		{4, 5, 7, 6}, // z = 1, outward +z
		{0, 1, 5, 4}, // y = 0, outward -y
		{2, 6, 7, 3}, // y = 1, outward +y
		{0, 4, 6, 2}, // x = 0, outward -x
		{1, 3, 7, 5}, // x = 1, outward +x
	}
	var triangles [][3]int
	for _, q := range quads {
		triangles = append(triangles, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
	}
	return vertices, triangles
}

// inverted walks every triangle the other way.
func inverted(triangles [][3]int) [][3]int {
	out := make([][3]int, len(triangles))
	for i, t := range triangles {
		out[i] = [3]int{t[0], t[2], t[1]}
	}
	return out
}

// shellIndex names shell s for every one of n facets.
func shellIndex(n, s int) []int {
	out := make([]int, n)
	for i := range out {
		out[i] = s
	}
	return out
}

// TestRequireShellWinding reads each arm of the per-shell winding audit on
// the unit cube: an outer shell must enclose a positive signed volume and a
// void a negative one, a shell whose facets do not close and a facet of no
// shell decide nothing, and a shell index list that does not match the facets
// is a degenerate call.
func TestRequireShellWinding(t *testing.T) {
	t.Parallel()
	vertices, outward := windingCube()
	inward := inverted(outward)
	n := len(outward)

	tests := []struct {
		name      string
		triangles [][3]int
		shellOf   []int
		void      []bool
		// wantWinding is the refusal's text, empty when the audit passes.
		wantWinding string
	}{
		{"an outward outer shell", outward, shellIndex(n, 0), []bool{false}, ""},
		{"an inward void shell", inward, shellIndex(n, 0), []bool{true}, ""},
		{"an outward void shell", outward, shellIndex(n, 0), []bool{true}, "do not wind into the cavity"},
		{"an inward outer shell", inward, shellIndex(n, 0), []bool{false}, "do not wind outward"},
		{"a shell whose facets do not close decides nothing", inward[:n-1], shellIndex(n-1, 0), []bool{false}, ""},
		{"a facet of no closed shell decides nothing", inward, shellIndex(n, -1), []bool{false}, ""},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			err := tessellation.RequireShellWinding(t.Context(), vertices, tc.triangles, tc.shellOf, tc.void)
			if tc.wantWinding == "" {
				require.NoError(t, err)
				return
			}
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			winding, ok := errors.AsType[*tessellation.ShellWindingError](err)
			require.True(t, ok)
			require.Contains(t, winding.Msg, tc.wantWinding)
		})
	}

	t.Run("one index per facet", func(t *testing.T) {
		t.Parallel()
		err := tessellation.RequireShellWinding(t.Context(), vertices, outward, shellIndex(n-1, 0), []bool{false})
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		require.ErrorContains(t, err, "shell indices for")
	})
	t.Run("a shell index past the shells", func(t *testing.T) {
		t.Parallel()
		err := tessellation.RequireShellWinding(t.Context(), vertices, outward, shellIndex(n, 1), []bool{false})
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		require.ErrorContains(t, err, "names shell 1 of 1")
	})
}
