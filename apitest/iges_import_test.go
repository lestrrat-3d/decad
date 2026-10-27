package apitest_test

import (
	"fmt"
	"slices"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/iges-import-design.md's public-surface tests for ReadIGES. The files are written
// here, record by record, from the IGES 5.3 fixed format, so the reader is tested against the format's own
// layout and not against its own output.

// igesWriter builds an IGES file: entities are added in order, and each one's directory sequence number
// (the number other entities point at it with) is returned.
type igesWriter struct {
	dir    []string
	data   []string
	global string
}

func newIGESWriter(unitFlag int, unitName string) *igesWriter {
	// The two delimiters spelled out, a few of the global parameters, the model space scale (13) and the unit (14, 15).
	fields := []string{"1H,", "1H;", "4HTEST", "8Hbox.iges", "5Hdecad", "3H1.0", "32", "38", "6", "308", "15", "4HTEST",
		"1.0", fmt.Sprint(unitFlag), fmt.Sprintf("%dH%s", len(unitName), unitName), "1", "0.01", "13H271027.120000",
		"1.0E-6", "100.0", "6Hauthor", "3Horg", "11", "0"}
	return &igesWriter{global: strings.Join(fields, ",") + ";"}
}

func pad(s string, n int) string { return s + strings.Repeat(" ", n-len(s)) }

// add writes an entity of the type and form with the parameters (after the type number) and returns its
// directory sequence number.
func (w *igesWriter) add(typ, form int, params ...string) int {
	seq := len(w.dir) + 1
	text := strings.Join(append([]string{fmt.Sprint(typ)}, params...), ",") + ";"
	var lines []string
	for len(text) > 0 {
		n := min(64, len(text))
		lines = append(lines, text[:n])
		text = text[n:]
	}
	first := len(w.data) + 1
	for _, l := range lines {
		w.data = append(w.data, fmt.Sprintf("%s%8dP%7d", pad(l, 64), seq, len(w.data)+1))
	}
	f := func(v int) string { return fmt.Sprintf("%8d", v) }
	w.dir = append(w.dir,
		f(typ)+f(first)+f(0)+f(0)+f(0)+f(0)+f(0)+f(0)+"00000000"+fmt.Sprintf("D%7d", seq),
		f(typ)+f(0)+f(0)+f(len(lines))+f(form)+strings.Repeat(" ", 8*4)+fmt.Sprintf("D%7d", seq+1))
	return seq
}

func (w *igesWriter) String() string {
	var out []string
	out = append(out, fmt.Sprintf("%sS%7d", pad("decad test file", 72), 1))
	g := w.global
	n := 0
	for len(g) > 0 {
		k := min(72, len(g))
		n++
		out = append(out, fmt.Sprintf("%sG%7d", pad(g[:k], 72), n))
		g = g[k:]
	}
	out = append(out, w.dir...)
	out = append(out, w.data...)
	out = append(out, fmt.Sprintf("S%7dG%7dD%7dP%7d%sT%7d", 1, n, len(w.dir), len(w.data), strings.Repeat(" ", 40), 1))
	return strings.Join(out, "\n") + "\n"
}

func real3(x, y, z float64) string {
	return fmt.Sprintf("%.12g,%.12g,%.12g", x, y, z)
}

// igesBox writes the box lo..hi as a manifold solid B-rep: its shell and the solid that owns it.
func igesBox(w *igesWriter, lo, hi r3.Vec, flip map[int]bool) {
	shell := igesShell(w, lo, hi, flip)
	w.add(186, 0, fmt.Sprint(shell), "1", "0")
}

// igesShell writes the box lo..hi as a closed shell: a vertex list, twelve lines, an edge list, six planes,
// loops and faces. A face in flip has its loop written the other way round. It returns the shell's sequence number.
func igesShell(w *igesWriter, lo, hi r3.Vec, flip map[int]bool) int {
	x0, y0, z0, x1, y1, z1 := lo.X, lo.Y, lo.Z, hi.X, hi.Y, hi.Z
	corners := [8][3]float64{{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0}, {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}}
	var vlist []string
	for _, c := range corners {
		vlist = append(vlist, real3(c[0], c[1], c[2]))
	}
	vl := w.add(502, 1, append([]string{"8"}, vlist...)...)
	// The edges, from one corner to another (indices from 1).
	ends := [12][2]int{{1, 2}, {2, 3}, {3, 4}, {4, 1}, {5, 6}, {6, 7}, {7, 8}, {8, 5}, {1, 5}, {2, 6}, {3, 7}, {4, 8}}
	var elist []string
	for _, e := range ends {
		a, b := corners[e[0]-1], corners[e[1]-1]
		line := w.add(110, 0, real3(a[0], a[1], a[2]), real3(b[0], b[1], b[2]))
		elist = append(elist, fmt.Sprint(line), fmt.Sprint(vl), fmt.Sprint(e[0]), fmt.Sprint(vl), fmt.Sprint(e[1]))
	}
	el := w.add(504, 1, append([]string{"12"}, elist...)...)
	// Each face: the plane, and its loop as (edge, same-sense) pairs running counter-clockwise seen from outside.
	type use struct {
		edge int
		same bool
	}
	faces := []struct {
		plane [4]float64
		loop  []use
	}{
		{[4]float64{0, 0, -1, -z0}, []use{{4, false}, {3, false}, {2, false}, {1, false}}},
		{[4]float64{0, 0, 1, z1}, []use{{5, true}, {6, true}, {7, true}, {8, true}}},
		{[4]float64{0, -1, 0, -y0}, []use{{1, true}, {10, true}, {5, false}, {9, false}}},
		{[4]float64{0, 1, 0, y1}, []use{{3, true}, {12, true}, {7, false}, {11, false}}},
		{[4]float64{-1, 0, 0, -x0}, []use{{9, true}, {8, false}, {12, false}, {4, true}}},
		{[4]float64{1, 0, 0, x1}, []use{{2, true}, {11, true}, {6, false}, {10, false}}},
	}
	var shell []string
	for i, f := range faces {
		plane := w.add(108, 0, fmt.Sprintf("%.12g,%.12g,%.12g,%.12g", f.plane[0], f.plane[1], f.plane[2], f.plane[3]), "0")
		loop := f.loop
		if flip[i] {
			loop = nil
			for _, u := range slices.Backward(f.loop) {
				loop = append(loop, use{u.edge, !u.same})
			}
		}
		params := []string{"4"}
		for _, u := range loop {
			same := 0
			if u.same {
				same = 1
			}
			params = append(params, "0", fmt.Sprint(el), fmt.Sprint(u.edge), fmt.Sprint(same), "0")
		}
		lp := w.add(508, 1, params...)
		face := w.add(510, 1, fmt.Sprint(plane), "1", "1", fmt.Sprint(lp))
		shell = append(shell, fmt.Sprint(face), "1")
	}
	return w.add(514, 1, append([]string{"6"}, shell...)...)
}

func readIGES(t *testing.T, w *igesWriter) (*decad.MeshBody, error) {
	t.Helper()
	return decad.ReadIGES(t.Context(), strings.NewReader(w.String()))
}

func TestReadIGESReadsABoxAsASolid(t *testing.T) {
	t.Parallel()
	w := newIGESWriter(2, "MM")
	igesBox(w, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6), nil)
	m, err := readIGES(t, w)
	require.NoError(t, err)
	report := m.Report()
	require.True(t, report.Closed)
	require.Zero(t, report.NonManifoldEdges)
	require.False(t, report.Inverted, "every face winds outward")
	require.Equal(t, 8, report.Vertices)
	require.Equal(t, 12, report.Triangles)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, 480, volume.Value.Base(), 1e-9)
	box, err := decad.SolidFromMesh(t.Context(), decad.New(), m)
	require.NoError(t, err)
	require.Len(t, box.Faces(), 6, "six planar faces")
	bounds, err := box.Bounds()
	require.NoError(t, err)
	require.True(t, bounds.Max.Equal(r3.NewVec(10, 8, 6), 1e-12))
}

func TestReadIGESWindsFacesWrittenBothWays(t *testing.T) {
	t.Parallel()
	// Two faces have their loops run the other way round: the mesh is still wound outward, and a solid.
	w := newIGESWriter(2, "MM")
	igesBox(w, r3.NewVec(1, 2, 3), r3.NewVec(5, 7, 9), map[int]bool{1: true, 4: true})
	m, err := readIGES(t, w)
	require.NoError(t, err)
	require.False(t, m.Report().Inverted)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, 4*5*6, volume.Value.Base(), 1e-9)
	_, err = decad.SolidFromMesh(t.Context(), decad.New(), m)
	require.NoError(t, err)
}

func TestReadIGESConvertsTheFilesUnit(t *testing.T) {
	t.Parallel()
	for flag, scale := range map[int]float64{1: 25.4, 2: 1, 6: 1000, 10: 10, 4: 304.8} {
		w := newIGESWriter(flag, "U")
		igesBox(w, r3.NewVec(0, 0, 0), r3.NewVec(1, 2, 3), nil)
		m, err := readIGES(t, w)
		require.NoError(t, err, "flag %d", flag)
		volume, err := m.Volume()
		require.NoError(t, err)
		require.InEpsilon(t, 6*scale*scale*scale, volume.Value.Base(), 1e-12, "flag %d", flag)
	}
}

func TestReadIGESReadsASolidWithAVoid(t *testing.T) {
	t.Parallel()
	w := newIGESWriter(2, "MM")
	// A 10 mm block with a 4 mm cubic cavity: the solid's shell, the void's shell, and one solid entity naming both.
	outer := igesShell(w, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 10), nil)
	cavity := igesShell(w, r3.NewVec(3, 3, 3), r3.NewVec(7, 7, 7), nil)
	w.add(186, 0, fmt.Sprint(outer), "1", "1", fmt.Sprint(cavity), "1")
	m, err := readIGES(t, w)
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, 1000-64, volume.Value.Base(), 1e-9, "the void's shell winds into its cavity")
	solid, err := decad.SolidFromMesh(t.Context(), decad.New(), m)
	require.NoError(t, err)
	inside, err := solid.LocatePoint(t.Context(), r3.NewVec(5, 5, 5), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, inside, "the cavity is empty")
	wall, err := solid.LocatePoint(t.Context(), r3.NewVec(1.5, 5, 5), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, wall)
}

func TestReadIGESRefusals(t *testing.T) {
	t.Parallel()
	good := func() *igesWriter {
		w := newIGESWriter(2, "MM")
		igesBox(w, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6), nil)
		return w
	}
	// A face on a surface that is not a plane is refused, not guessed at: the planes of the box are rewritten as
	// ruled surfaces (118).
	curved := good()
	for i, line := range curved.dir {
		if strings.HasPrefix(line, "     108") && i%2 == 0 {
			curved.dir[i] = "     118" + line[8:]
			curved.dir[i+1] = "     118" + curved.dir[i+1][8:]
		}
	}
	for i, d := range curved.data {
		if strings.HasPrefix(d, "108,") {
			curved.data[i] = "118," + d[4:]
		}
	}
	_, err := readIGES(t, curved)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	// An edge that is a circular arc (100) is refused.
	arc := good()
	for i, line := range arc.dir {
		if strings.HasPrefix(line, "     110") && i%2 == 0 {
			arc.dir[i] = "     100" + line[8:]
			arc.dir[i+1] = "     100" + arc.dir[i+1][8:]
			break
		}
	}
	for i, d := range arc.data {
		if strings.HasPrefix(d, "110,") {
			arc.data[i] = "100," + d[4:]
			break
		}
	}
	_, err = readIGES(t, arc)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	// A model space that is not at scale 1, and a transformation pointer, are refused.
	scaled := good()
	scaled.global = strings.Replace(scaled.global, ",1.0,2,", ",2.0,2,", 1)
	_, err = readIGES(t, scaled)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	placed := good()
	placed.dir[0] = placed.dir[0][:48] + fmt.Sprintf("%8d", 3) + placed.dir[0][56:]
	_, err = readIGES(t, placed)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	// Files that are not IGES, or whose entities do not hold together, are malformed.
	for name, text := range map[string]string{
		"empty":          "",
		"no terminate":   strings.Join(strings.Split(good().String(), "\n")[:5], "\n") + "\n",
		"not iges":       strings.Repeat("x", 100) + "\n",
		"missing params": strings.Join(strings.Split(good().String(), "\n")[:20], "\n") + "\nS      1G      1D      1P      1" + strings.Repeat(" ", 40) + "T      1\n",
	} {
		_, err = decad.ReadIGES(t.Context(), strings.NewReader(text))
		require.ErrorIs(t, err, decad.ErrMalformedMesh, name)
	}
	// A file with no solid B-rep entity: lines alone make no solid.
	lines := newIGESWriter(2, "MM")
	lines.add(110, 0, real3(0, 0, 0), real3(1, 0, 0))
	_, err = readIGES(t, lines)
	require.ErrorIs(t, err, decad.ErrMalformedMesh)
	_, err = decad.ReadIGES(nil, strings.NewReader("")) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.ReadIGES(t.Context(), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.ReadIGES(t.Context(), strings.NewReader(good().String()), decad.WithMaxMeshTriangles(5))
	require.ErrorIs(t, err, decad.ErrMeshTooLarge)
}

// igesFrame writes a rectangular frame as a closed shell: the block lo..hi with a square hole through its z extent,
// hlo..hhi in x and y. Its top and bottom faces have an inner loop. It returns the shell's sequence number.
func igesFrame(w *igesWriter, lo, hi, hlo, hhi r3.Vec) int {
	// Sixteen vertices: the outer ring then the hole ring, at the bottom then the top.
	ring := func(a, b r3.Vec, z float64) [4][3]float64 {
		return [4][3]float64{{a.X, a.Y, z}, {b.X, a.Y, z}, {b.X, b.Y, z}, {a.X, b.Y, z}}
	}
	var corners [][3]float64
	for _, z := range []float64{lo.Z, hi.Z} {
		for _, c := range ring(lo, hi, z) {
			corners = append(corners, c)
		}
		for _, c := range ring(hlo, hhi, z) {
			corners = append(corners, c)
		}
	}
	var vlist []string
	for _, c := range corners {
		vlist = append(vlist, real3(c[0], c[1], c[2]))
	}
	vl := w.add(502, 1, append([]string{fmt.Sprint(len(corners))}, vlist...)...)
	// Vertex numbers (from one): outer bottom 1-4, hole bottom 5-8, outer top 9-12, hole top 13-16.
	type edge struct{ a, b int }
	var edges []edge
	index := map[edge]int{}
	addEdge := func(a, b int) {
		index[edge{a, b}] = len(edges) + 1
		edges = append(edges, edge{a, b})
	}
	for _, base := range []int{1, 5, 9, 13} {
		for k := range 4 {
			addEdge(base+k, base+(k+1)%4)
		}
	}
	for k := range 8 {
		addEdge(1+k, 9+k) // each vertical, from bottom to top
	}
	var elist []string
	for _, e := range edges {
		a, b := corners[e.a-1], corners[e.b-1]
		line := w.add(110, 0, real3(a[0], a[1], a[2]), real3(b[0], b[1], b[2]))
		elist = append(elist, fmt.Sprint(line), fmt.Sprint(vl), fmt.Sprint(e.a), fmt.Sprint(vl), fmt.Sprint(e.b))
	}
	el := w.add(504, 1, append([]string{fmt.Sprint(len(edges))}, elist...)...)
	// A loop walks vertices in order, using each edge in the sense it was stored or against it.
	loop := func(vertices ...int) int {
		params := []string{fmt.Sprint(len(vertices))}
		for k, a := range vertices {
			b := vertices[(k+1)%len(vertices)]
			id, same := index[edge{a, b}], 1
			if id == 0 {
				id, same = index[edge{b, a}], 0
			}
			if id == 0 {
				panic(fmt.Sprintf("no edge between %d and %d", a, b))
			}
			params = append(params, "0", fmt.Sprint(el), fmt.Sprint(id), fmt.Sprint(same), "0")
		}
		return w.add(508, 1, params...)
	}
	plane := w.add(108, 0, "0,0,1,0", "0")
	var shell []string
	face := func(loops ...int) {
		params := []string{fmt.Sprint(plane), fmt.Sprint(len(loops)), "1"}
		for _, l := range loops {
			params = append(params, fmt.Sprint(l))
		}
		shell = append(shell, fmt.Sprint(w.add(510, 1, params...)), "1")
	}
	// Bottom (outward -z): the outer ring clockwise seen from above, the hole counter-clockwise. Top: the reverse.
	face(loop(1, 4, 3, 2), loop(5, 6, 7, 8))
	face(loop(9, 10, 11, 12), loop(13, 16, 15, 14))
	// Outer walls, outward, then the hole's walls, whose outward side faces into the hole.
	for k := range 4 {
		a, b := 1+k, 1+(k+1)%4
		face(loop(a, b, b+8, a+8))
	}
	for k := range 4 {
		a, b := 5+k, 5+(k+1)%4
		face(loop(b, a, a+8, b+8))
	}
	return w.add(514, 1, append([]string{"10"}, shell...)...)
}

func TestReadIGESCutsFacesWithHoles(t *testing.T) {
	t.Parallel()
	w := newIGESWriter(2, "MM")
	shell := igesFrame(w, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 5), r3.NewVec(6, 6, 0), r3.NewVec(14, 14, 5))
	w.add(186, 0, fmt.Sprint(shell), "1", "0")
	m, err := readIGES(t, w)
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	require.False(t, m.Report().Inverted)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, (400-64)*5, volume.Value.Base(), 1e-9, "the frame's volume, less its hole")
	solid, err := decad.SolidFromMesh(t.Context(), decad.New(), m)
	require.NoError(t, err)
	require.Len(t, solid.Faces(), 10, "two faces with a hole and eight walls")
	hole, err := solid.LocatePoint(t.Context(), r3.NewVec(10, 10, 2.5), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, hole)
	rim, err := solid.LocatePoint(t.Context(), r3.NewVec(3, 10, 2.5), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, rim)
}
