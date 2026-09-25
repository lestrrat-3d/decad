package export_test

import (
	"bytes"
	"context"
	"errors"
	"strings"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/step/ap214"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func box(t *testing.T, sheet bool) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 2, 3)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	if sheet {
		body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(4), Dir: decad.Along}, decad.WithSurfaceResult())
		require.NoError(t, err)
		return body
	}
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func header() step.Header {
	return step.Header{
		Description:         []string{"faceted AP214 test"},
		Name:                "box.step",
		Timestamp:           time.Date(2026, 9, 25, 0, 0, 0, 0, time.UTC),
		Authors:             []string{"Decad"},
		Organizations:       []string{"Decad"},
		PreprocessorVersion: "decad export",
		OriginatingSystem:   "decad",
	}
}

func TestNewSTEPFileBoxTopology(t *testing.T) {
	t.Parallel()
	body := box(t, false)
	f, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.1), header())
	require.NoError(t, err)
	require.Equal(t, []string{ap214.Schema}, f.Header.Schemas)
	counts := map[string]int{}
	points := map[[3]step.Real]struct{}{}
	edgeUses := map[step.Reference][]step.Enumeration{}
	for _, entity := range f.Entities {
		counts[entity.Name]++
		switch entity.Name {
		case "CARTESIAN_POINT":
			xyz := entity.Parameters[1].(step.List)
			points[[3]step.Real{xyz[0].(step.Real), xyz[1].(step.Real), xyz[2].(step.Real)}] = struct{}{}
		case "ORIENTED_EDGE":
			edge := entity.Parameters[3].(step.Reference)
			edgeUses[edge] = append(edgeUses[edge], entity.Parameters[4].(step.Enumeration))
		}
	}
	require.Equal(t, 8, counts["VERTEX_POINT"])
	require.Equal(t, 8, counts["CARTESIAN_POINT"])
	require.Equal(t, 18, counts["EDGE_CURVE"])
	require.Equal(t, 36, counts["ORIENTED_EDGE"])
	require.Equal(t, 12, counts["ADVANCED_FACE"])
	require.Equal(t, 1, counts["CLOSED_SHELL"])
	require.Equal(t, 1, counts["MANIFOLD_SOLID_BREP"])
	require.Equal(t, 1, counts["ADVANCED_BREP_SHAPE_REPRESENTATION"])
	require.Equal(t, 8, len(points))
	for _, p := range [][3]step.Real{{0, 0, 0}, {2, 0, 0}, {0, 3, 0}, {2, 3, 0}, {0, 0, 4}, {2, 0, 4}, {0, 3, 4}, {2, 3, 4}} {
		_, ok := points[p]
		require.Truef(t, ok, "missing corner %v", p)
	}
	for _, senses := range edgeUses {
		require.ElementsMatch(t, []step.Enumeration{"T", "F"}, senses)
	}
	data, err := f.Marshal()
	require.NoError(t, err)
	require.Contains(t, string(data), "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));")
	require.Contains(t, string(data), "MANIFOLD_SOLID_BREP")
}

func TestWriteDeterministicAndUntouchedOnInvalidHeader(t *testing.T) {
	t.Parallel()
	body := box(t, false)
	var first, second bytes.Buffer
	require.NoError(t, export.STEP(t.Context(), &first, body, units.Millimeters(0.1), header()))
	require.NoError(t, export.STEP(t.Context(), &second, body, units.Millimeters(0.1), header()))
	require.Equal(t, first.Bytes(), second.Bytes())
	require.Equal(t, 12, strings.Count(first.String(), "=ADVANCED_FACE("))

	bad := header()
	bad.Timestamp = time.Time{}
	var untouched bytes.Buffer
	untouched.WriteString("original")
	require.Error(t, export.STEP(t.Context(), &untouched, body, units.Millimeters(0.1), bad))
	require.Equal(t, "original", untouched.String())
}

func TestNewSTEPFileRefusals(t *testing.T) {
	t.Parallel()
	_, err := export.NewSTEPFile(t.Context(), nil, units.Millimeters(0.1), header())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = export.NewSTEPFile(t.Context(), box(t, true), units.Millimeters(0.1), header())
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = export.NewSTEPFile(t.Context(), box(t, false), units.Millimeters(0), header())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = export.NewSTEPFile(ctx, box(t, false), units.Millimeters(0.1), header())
	require.True(t, errors.Is(err, context.Canceled))
}
