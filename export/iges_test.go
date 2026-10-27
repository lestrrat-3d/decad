package export_test

import (
	"bytes"
	"math"
	"strconv"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/iges-export-design.md's tests for IGES: each file is read back through ReadIGES, the reader
// the format's own records were written against, and its mesh compared with the body's tessellation.

func writeIGES(t *testing.T, body *decad.Body) string {
	t.Helper()
	var out bytes.Buffer
	require.NoError(t, export.IGES(t.Context(), &out, body, units.Millimeters(0.01)))
	return out.String()
}

func readIGES(t *testing.T, file string) *decad.MeshBody {
	t.Helper()
	m, err := decad.ReadIGES(t.Context(), strings.NewReader(file))
	require.NoError(t, err)
	return m
}

// requireRoundTrip reads body's IGES file back and requires the mesh the file holds to be the tessellation the file was
// written from: the same vertices, exactly, and the same triangle count, a closed mesh of the body's volume.
func requireRoundTrip(t *testing.T, body *decad.Body) *decad.MeshBody {
	t.Helper()
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	got := readIGES(t, writeIGES(t, body))
	require.True(t, got.Report().Closed)
	require.Len(t, got.Triangles(), len(mesh.Triangles()))
	want := map[r3.Vec]bool{}
	for _, v := range mesh.Vertices() {
		want[v] = true
	}
	require.Len(t, got.Vertices(), len(want), "vertices are welded by coordinate")
	for _, v := range got.Vertices() {
		require.True(t, want[v], "vertex %v is a vertex of the tessellation, to the last digit", v)
	}
	return got
}

func TestIGESWritesABlockThatReadsBackAsItself(t *testing.T) {
	t.Parallel()
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 10, units.Millimeters(6))
	got := requireRoundTrip(t, block)
	volume, err := got.Volume()
	require.NoError(t, err)
	require.InDelta(t, 1200, volume.Value.Base(), 1e-9)
	box := got.Bounds()
	require.True(t, box.Min.Equal(r3.Vec{}, 1e-12))
	require.True(t, box.Max.Equal(r3.Vec{X: 20, Y: 10, Z: 6}, 1e-12))
}

func TestIGESRecordsAreWellFormed(t *testing.T) {
	t.Parallel()
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 10, units.Millimeters(6))
	file := writeIGES(t, block)
	lines := strings.Split(strings.TrimSuffix(file, "\n"), "\n")
	counts := map[byte]int{}
	order := ""
	for i, line := range lines {
		require.Len(t, line, 80, "record %d is 80 columns", i+1)
		section := line[72]
		seq, err := strconv.Atoi(strings.TrimSpace(line[73:]))
		require.NoError(t, err)
		if section == 'T' {
			require.Equal(t, 1, seq)
			continue
		}
		counts[section]++
		require.Equal(t, counts[section], seq, "section %c is numbered from one", section)
		if !strings.HasSuffix(order, string(section)) {
			order += string(section)
		}
	}
	require.Equal(t, "SGDP", order, "the sections come in order")
	terminate := lines[len(lines)-1]
	require.Equal(t, 'T', rune(terminate[72]))
	for _, section := range []byte("SGDP") {
		n, err := strconv.Atoi(strings.TrimSpace(terminate[strings.IndexByte(terminate, section)+1 : strings.IndexByte(terminate, section)+8]))
		require.NoError(t, err)
		require.Equal(t, counts[section], n, "the terminate record counts section %c", section)
	}
	// A block is twelve triangles: 6 planes (coplanar triangles share one), 12 loops, 12 faces, a shell and a solid,
	// with one vertex list, one edge list and 18 lines, two directory records to each of the 52 entities.
	require.Equal(t, 2*(1+18+1+6+12+12+1+1), counts['D'])
}

func TestIGESWritesACurvedSolidAndACavity(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateCircle(s.CreatePoint(0, 0), 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	cylinder, err := decad.Extrude(t.Context(), decad.New(), s, s.Profiles()[0], decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	got := requireRoundTrip(t, cylinder)
	volume, err := got.Volume()
	require.NoError(t, err)
	// The faceted cylinder is inside the true one: its volume is within a per cent of π r² h.
	require.InEpsilon(t, math.Pi*25*8, volume.Value.Base(), 0.01)

	cube := cavityCube(t)
	cavity := requireRoundTrip(t, cube)
	cavityVolume, err := cavity.Volume()
	require.NoError(t, err)
	require.InDelta(t, 8000-1000, cavityVolume.Value.Base(), 1e-9, "the 10 mm cavity of a 20 mm cube is a void shell of the solid")
}

func TestIGESWritesOneSolidForEachLump(t *testing.T) {
	t.Parallel()
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 10, units.Millimeters(6))
	pair, err := decad.Patterned(t.Context(), block, decad.LinearPattern{Dir: r3.Vec{X: 1}, Step: units.Millimeters(30), Count: 2})
	require.NoError(t, err)
	require.Len(t, pair.Lumps(), 2)
	got := requireRoundTrip(t, pair)
	volume, err := got.Volume()
	require.NoError(t, err)
	require.InDelta(t, 2400, volume.Value.Base(), 1e-9)
	require.Equal(t, 2, strings.Count(writeIGES(t, pair), "186,"), "a solid for each lump")
}

func TestIGESRefusals(t *testing.T) {
	t.Parallel()
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 10, units.Millimeters(6))
	tol := units.Millimeters(0.01)
	require.ErrorIs(t, export.IGES(nil, &bytes.Buffer{}, block, tol), decad.ErrDegenerate) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, export.IGES(t.Context(), nil, block, tol), decad.ErrDegenerate)
	require.ErrorIs(t, export.IGES(t.Context(), &bytes.Buffer{}, nil, tol), decad.ErrDegenerate)
	var out bytes.Buffer
	require.Error(t, export.IGES(t.Context(), &out, block, units.Degrees(1)), "a tolerance that is not a length")
	require.Empty(t, out.String(), "a refusal writes nothing")
}
