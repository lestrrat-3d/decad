package decad_test

import (
	"bytes"
	"strconv"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSTLPlate(t *testing.T) {
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	var buf bytes.Buffer
	require.NoError(t, body.STL(&buf))
	out := buf.String()

	lines := strings.Split(strings.TrimSuffix(out, "\n"), "\n")
	require.Equal(t, "solid decad", lines[0])
	require.Equal(t, "endsolid decad", lines[len(lines)-1])
	require.Equal(t, 12, strings.Count(out, "facet normal"), `an all-planar plate is 12 facets at any tolerance`)
	require.Equal(t, 12, strings.Count(out, "endfacet"))
	require.Equal(t, 12, strings.Count(out, "outer loop"))
	require.Equal(t, 36, strings.Count(out, "vertex "))

	// Every numeric field parses, and each facet normal is unit length.
	for _, line := range lines {
		fields := strings.Fields(line)
		if len(fields) == 0 {
			continue
		}
		var nums []float64
		for _, f := range fields {
			v, err := strconv.ParseFloat(f, 64)
			if err != nil {
				continue
			}
			nums = append(nums, v)
		}
		switch fields[0] {
		case "facet":
			require.Len(t, nums, 3)
			require.InDelta(t, 1.0, nums[0]*nums[0]+nums[1]*nums[1]+nums[2]*nums[2], 1e-12)
		case "vertex":
			require.Len(t, nums, 3)
		}
	}

	// Deterministic: a second write is byte-identical.
	var again bytes.Buffer
	require.NoError(t, body.STL(&again))
	require.Equal(t, out, again.String())
}

func TestSTLChordTolerance(t *testing.T) {
	body := holedPlateBody(t)

	// The explicit tolerance drives the tessellation: same facet count as
	// Tessellate at that tolerance.
	mesh, err := body.Tessellate(units.Millimeters(0.5))
	require.NoError(t, err)

	var buf bytes.Buffer
	require.NoError(t, body.STL(&buf, decad.WithChordTolerance(units.Millimeters(0.5))))
	require.Equal(t, len(mesh.Triangles()), strings.Count(buf.String(), "facet normal"))

	// A finer tolerance takes more chords.
	var fine bytes.Buffer
	require.NoError(t, body.STL(&fine, decad.WithChordTolerance(units.Millimeters(0.05))))
	require.Greater(t, strings.Count(fine.String(), "facet normal"), len(mesh.Triangles()))

	// The tolerance is validated exactly like Tessellate's.
	require.ErrorIs(t, body.STL(&buf, decad.WithChordTolerance(units.Millimeters(0))), decad.ErrDegenerate)
	require.ErrorIs(t, body.STL(&buf, decad.WithChordTolerance(units.Degrees(1))), decad.ErrUnitKind)
}

func TestSTLDefaultChordTolerance(t *testing.T) {
	t.Parallel()
	sizeDefault := func(t *testing.T, body *decad.Body) units.Value {
		t.Helper()
		bounds, err := body.Bounds()
		require.NoError(t, err)
		return units.Millimeters(bounds.Max.Sub(bounds.Min).Len() / 1000)
	}

	t.Run("analytic body uses size default", func(t *testing.T) {
		body := holedPlateBody(t)
		tol := sizeDefault(t, body)

		var explicit bytes.Buffer
		require.NoError(t, body.STL(&explicit, decad.WithChordTolerance(tol)))
		var automatic bytes.Buffer
		require.NoError(t, body.STL(&automatic))
		require.Equal(t, explicit.String(), automatic.String())
	})

	t.Run("faceted body honors retained bound", func(t *testing.T) {
		doc := decad.New()
		plate := boxBody(t, doc, 0, 0, 20, 20, 8)
		tool := translated(t, diskBody(t, doc, 10, 10, 2), 0, 0, -6)
		cut, err := decad.Cut(plate, tool)
		require.NoError(t, err)

		faceted := translated(t, cut, 1e13, -2e13, 3e13)
		tol := sizeDefault(t, faceted)
		held, err := faceted.Tessellate(units.Millimeters(1))
		require.NoError(t, err)
		require.Greater(t, held.Bound().Mag(), tol.Mag())
		_, err = faceted.Tessellate(tol)
		require.ErrorIs(t, err, decad.ErrUnsupported)

		var explicit bytes.Buffer
		require.NoError(t, faceted.STL(&explicit, decad.WithChordTolerance(held.Bound())))
		var automatic bytes.Buffer
		require.NoError(t, faceted.STL(&automatic))
		require.Equal(t, explicit.String(), automatic.String())
	})
}

func TestOBJPlate(t *testing.T) {
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	var buf bytes.Buffer
	require.NoError(t, body.OBJ(&buf))
	out := buf.String()

	var vs, fs int
	for line := range strings.SplitSeq(strings.TrimSuffix(out, "\n"), "\n") {
		fields := strings.Fields(line)
		require.Len(t, fields, 4, `OBJ lines are "v x y z" or "f a b c"`)
		switch fields[0] {
		case "v":
			require.Equal(t, 0, fs, `all vertices precede the first face`)
			vs++
			for _, f := range fields[1:] {
				_, err := strconv.ParseFloat(f, 64)
				require.NoError(t, err)
			}
		case "f":
			fs++
			for _, f := range fields[1:] {
				idx, err := strconv.Atoi(f)
				require.NoError(t, err)
				require.GreaterOrEqual(t, idx, 1, `OBJ indices are 1-based`)
				require.LessOrEqual(t, idx, 8)
			}
		default:
			require.Fail(t, "unexpected OBJ line", line)
		}
	}
	require.Equal(t, 8, vs)
	require.Equal(t, 12, fs)

	// Deterministic: a second write is byte-identical.
	var again bytes.Buffer
	require.NoError(t, body.OBJ(&again))
	require.Equal(t, out, again.String())
}

func TestOBJChordTolerance(t *testing.T) {
	body := holedPlateBody(t)
	mesh, err := body.Tessellate(units.Millimeters(0.5))
	require.NoError(t, err)

	var buf bytes.Buffer
	require.NoError(t, body.OBJ(&buf, decad.WithChordTolerance(units.Millimeters(0.5))))
	out := buf.String()
	require.Equal(t, len(mesh.Vertices()), strings.Count(out, "v "))
	require.Equal(t, len(mesh.Triangles()), strings.Count(out, "f "))

	require.ErrorIs(t, body.OBJ(&buf, decad.WithChordTolerance(units.Millimeters(-1))), decad.ErrNegativeMagnitude)
}
