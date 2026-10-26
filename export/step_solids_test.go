package export

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// meshInput is the real tessellation of a closed-shell cube, a body of one
// outer shell and one void shell, as meshSolids receives it.
type meshInput struct {
	body       *decad.Body
	sources    []*decad.Face
	vertices   []r3.Vec
	triangles  [][3]int
	components []int
	count      int
	outer, cav int // the facet sets of the outer and the void shell
}

func closedCubeInput(t *testing.T) meshInput {
	t.Helper()
	cube := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(20))
	body, err := decad.Shell(t.Context(), cube, nil, units.Millimeters(3), decad.WithNoOpenings())
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	in := meshInput{body: body, sources: mesh.SourceFaces(), vertices: mesh.Vertices(), triangles: mesh.Triangles()}
	in.components, in.count, err = checkMesh(t.Context(), in.vertices, in.triangles)
	require.NoError(t, err)
	require.Equal(t, 2, in.count)
	voidFaces := map[*decad.Face]struct{}{}
	for _, shell := range body.Shells() {
		if !shell.IsVoid() {
			continue
		}
		for _, f := range shell.Faces() {
			voidFaces[f] = struct{}{}
		}
	}
	in.outer, in.cav = -1, -1
	for i, f := range in.sources {
		if _, ok := voidFaces[f]; ok {
			in.cav = in.components[i]
		} else {
			in.outer = in.components[i]
		}
	}
	require.NotEqual(t, -1, in.outer)
	require.NotEqual(t, -1, in.cav)
	return in
}

func (in meshInput) solids() ([]meshSolid, error) {
	return meshSolids(in.body, in.sources, in.vertices, in.triangles, in.components, in.count)
}

// flipSet returns the triangles with those of facet set c wound the other way.
func (in meshInput) flipSet(c int) [][3]int {
	out := slices.Clone(in.triangles)
	for i, tri := range out {
		if in.components[i] == c {
			out[i] = [3]int{tri[0], tri[2], tri[1]}
		}
	}
	return out
}

func TestMeshSolidsPairsEachShellWithItsFacets(t *testing.T) {
	t.Parallel()
	in := closedCubeInput(t)
	solids, err := in.solids()
	require.NoError(t, err)
	require.Equal(t, []meshSolid{{outer: in.outer, voids: []int{in.cav}}}, solids)
}

func TestMeshSolidsRefusesAMeshThatDoesNotMatchTheBody(t *testing.T) {
	t.Parallel()
	other := decadtest.NewBlock(t, decad.New(), 0, 0, 5, 5, units.Millimeters(5))

	type refusal struct {
		mutate func(in *meshInput)
		want   string
	}
	cases := map[string]refusal{
		"sources shorter than facets": refusal{func(in *meshInput) { in.sources = in.sources[:len(in.sources)-1] }, "does not map its facets"},
		"facet of a foreign face":     refusal{func(in *meshInput) { in.sources[0] = other.Faces()[0] }, "no face in the body"},
		"one set spans two shells": refusal{func(in *meshInput) {
			in.components = make([]int, len(in.components))
			in.count = 1
		}, "spans two shells"},
		"one shell spans two sets": refusal{func(in *meshInput) {
			in.components = slices.Clone(in.components)
			for i, c := range in.components {
				if c == in.outer {
					in.components[i] = 2
					break
				}
			}
			in.count = 3
		}, "spans two connected facet sets"},
		"a shell without facets": refusal{func(in *meshInput) {
			var sources []*decad.Face
			var triangles [][3]int
			var components []int
			for i, c := range in.components {
				if c != in.outer {
					continue
				}
				sources = append(sources, in.sources[i])
				triangles = append(triangles, in.triangles[i])
				components = append(components, 0)
			}
			in.sources, in.triangles, in.components, in.count = sources, triangles, components, 1
		}, "has no facets"},
		"a cavity wound outward":  refusal{func(in *meshInput) { in.triangles = in.flipSet(in.cav) }, "do not wind into it"},
		"an outer shell wound in": refusal{func(in *meshInput) { in.triangles = in.flipSet(in.outer) }, "do not wind outward"},
	}
	for name, c := range cases {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			in := closedCubeInput(t)
			in.sources = slices.Clone(in.sources)
			c.mutate(&in)
			_, err := in.solids()
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Contains(t, err.Error(), c.want)
		})
	}
}
