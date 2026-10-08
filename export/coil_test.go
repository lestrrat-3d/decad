package export_test

import (
	"bufio"
	"bytes"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// coilSpring is docs/helix-design.md §13's square spring: the section
// ρ ∈ [2, 3], ζ ∈ [0, 1] screwed two turns at pitch 1.5 about the sketch's V
// axis.
func coilSpring(t *testing.T) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	corners := [][2]float64{{2, 0}, {3, 0}, {3, 1}, {2, 1}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.New().Coil(t.Context(), s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{V: 1}},
		units.Millimeters(1.5), units.Scalar(2))
	require.NoError(t, err)
	return body
}

// requireClosedOriented asserts every directed edge of the triangle set
// appears exactly once and its reverse exactly once.
func requireClosedOriented(t *testing.T, tris [][3]int) {
	t.Helper()
	directed := map[[2]int]int{}
	for _, tri := range tris {
		require.True(t, tri[0] != tri[1] && tri[1] != tri[2] && tri[2] != tri[0])
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	for e, n := range directed {
		require.Equal(t, 1, n, "directed edge %v", e)
		require.Equal(t, 1, directed[[2]int{e[1], e[0]}], "directed edge %v has no opposing use", e)
	}
}

// TestCoilExportRoundTrips writes the spring as STL, 3MF and STEP. The STL
// facets weld back, by their printed coordinates alone, into the closed
// oriented mesh Tessellate holds, and so do the 3MF triangles over their own
// vertex list; the STEP writer emits one planar face per held triangle.
func TestCoilExportRoundTrips(t *testing.T) {
	t.Parallel()
	body := coilSpring(t)
	tol := units.Millimeters(0.01)
	mesh, err := body.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	var stl bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &stl, body, tol))
	index := map[string]int{}
	var tris [][3]int
	var facet []int
	sc := bufio.NewScanner(&stl)
	for sc.Scan() {
		line := strings.TrimSpace(sc.Text())
		if !strings.HasPrefix(line, "vertex ") {
			continue
		}
		key := strings.TrimPrefix(line, "vertex ")
		i, ok := index[key]
		if !ok {
			i = len(index)
			index[key] = i
		}
		facet = append(facet, i)
		if len(facet) == 3 {
			tris = append(tris, [3]int{facet[0], facet[1], facet[2]})
			facet = facet[:0]
		}
	}
	require.NoError(t, sc.Err())
	require.Len(t, tris, len(mesh.Triangles()))
	require.Len(t, index, len(mesh.Vertices()))
	requireClosedOriented(t, tris)

	var threeMF bytes.Buffer
	require.NoError(t, export.ThreeMF(t.Context(), &threeMF, body, tol))
	model := readThreeMFModel(t, threeMF.Bytes())
	require.Equal(t, "model", model.Object.Type)
	require.Len(t, model.Object.Vertices, len(mesh.Vertices()))
	tris = tris[:0]
	for _, tri := range model.Object.Triangles {
		tris = append(tris, [3]int{tri.V1, tri.V2, tri.V3})
	}
	requireClosedOriented(t, tris)

	f, err := export.NewSTEPFile(t.Context(), body, tol, header())
	require.NoError(t, err)
	faces := 0
	for _, entity := range f.Entities {
		if entity.Name == "ADVANCED_FACE" {
			faces++
		}
	}
	require.Equal(t, len(mesh.Triangles()), faces)
}
