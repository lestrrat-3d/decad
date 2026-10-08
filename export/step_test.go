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
	require.Equal(t, 14, counts["CARTESIAN_POINT"])
	require.Equal(t, 12, counts["EDGE_CURVE"])
	require.Equal(t, 24, counts["ORIENTED_EDGE"])
	require.Equal(t, 6, counts["ADVANCED_FACE"])
	require.Equal(t, 6, counts["PLANE"])
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

func TestNewSTEPFilePlateWithHoleAnalytic(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 30, 40)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(20, 15), 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	body, err := decad.New().Extrude(s, profile, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	f, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.1), header())
	require.NoError(t, err)
	counts := map[string]int{}
	uses := map[step.Reference][]step.Enumeration{}
	points := map[[3]step.Real]struct{}{}
	for _, entity := range f.Entities {
		counts[entity.Name]++
		switch entity.Name {
		case "CARTESIAN_POINT":
			xyz := entity.Parameters[1].(step.List)
			points[[3]step.Real{xyz[0].(step.Real), xyz[1].(step.Real), xyz[2].(step.Real)}] = struct{}{}
		case "CIRCLE", "CYLINDRICAL_SURFACE":
			require.Equal(t, step.Real(5), entity.Parameters[2])
		case "ORIENTED_EDGE":
			edge := entity.Parameters[3].(step.Reference)
			uses[edge] = append(uses[edge], entity.Parameters[4].(step.Enumeration))
		}
	}
	require.Equal(t, 7, counts["ADVANCED_FACE"])
	require.Equal(t, 6, counts["PLANE"])
	require.Equal(t, 1, counts["CYLINDRICAL_SURFACE"])
	require.Equal(t, 2, counts["CIRCLE"])
	require.Equal(t, 15, counts["EDGE_CURVE"])
	require.Equal(t, 10, counts["VERTEX_POINT"])
	require.Equal(t, 2, counts["FACE_BOUND"])
	for _, xyz := range [][3]step.Real{{0, 0, 0}, {30, 40, 8}, {20, 15, 0}, {20, 15, 8}} {
		_, ok := points[xyz]
		require.Truef(t, ok, "missing STEP point %v", xyz)
	}
	for _, senses := range uses {
		require.ElementsMatch(t, []step.Enumeration{"T", "F"}, senses)
	}
	_, err = f.Marshal()
	require.NoError(t, err)
}

func TestNewSTEPFileCylinderAnalytic(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.CreateCircle(center, 5)
	s.Fix(center)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.New().Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	f, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.1), header())
	require.NoError(t, err)
	counts := map[string]int{}
	uses := map[step.Reference][]step.Enumeration{}
	for _, entity := range f.Entities {
		counts[entity.Name]++
		if entity.Name == "ORIENTED_EDGE" {
			edge := entity.Parameters[3].(step.Reference)
			uses[edge] = append(uses[edge], entity.Parameters[4].(step.Enumeration))
		}
	}
	require.Equal(t, 3, counts["ADVANCED_FACE"])
	require.Equal(t, 2, counts["PLANE"])
	require.Equal(t, 1, counts["CYLINDRICAL_SURFACE"])
	require.Equal(t, 2, counts["CIRCLE"])
	require.Equal(t, 3, counts["EDGE_CURVE"])
	for _, senses := range uses {
		require.ElementsMatch(t, []step.Enumeration{"T", "F"}, senses)
	}
}

// TestNewSTEPFileArcPrismsAnalytic writes two arc-bearing prisms through the
// analytic arm: a half disc of radius 10 swept 5 mm (a convex partial
// cylinder wall, a planar wall, two caps whose loops are an arc and a line)
// and a 20×10 plate swept 4 mm with a radius-3 semicircular notch in its top
// edge (a concave partial wall). Each file carries one CYLINDRICAL_SURFACE,
// one CIRCLE per arc edge, and uses every EDGE_CURVE exactly twice with
// opposite senses, which a closed shell whose faces' loops are each oriented
// about their own face normal requires. Shown to fail with
// supportsAnalyticPartialWall refusing every wall (both files took the
// faceted writer) and with partialWallSense's reversal decision inverted
// (each wall then used its rim and side edges in its neighbours' sense).
func TestNewSTEPFileArcPrismsAnalytic(t *testing.T) {
	t.Parallel()
	build := func(t *testing.T, h float64, draw func(s *sketch.Sketch, fixed func(u, v float64) *sketch.Point)) *decad.Body {
		t.Helper()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		draw(s, func(u, v float64) *sketch.Point {
			p := s.CreatePoint(u, v)
			s.Fix(p)
			return p
		})
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		profiles := s.Profiles()
		require.Len(t, profiles, 1)
		body, err := decad.New().Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
		require.NoError(t, err)
		return body
	}
	for _, tc := range []struct {
		name          string
		h             float64
		draw          func(s *sketch.Sketch, fixed func(u, v float64) *sketch.Point)
		faces, planes int
	}{
		{"half disc", 5, func(s *sketch.Sketch, fixed func(u, v float64) *sketch.Point) {
			c, a, b := fixed(0, 0), fixed(10, 0), fixed(-10, 0)
			s.CreateArc(c, a, b)
			s.CreateLine(b, a)
		}, 4, 3},
		{"notched plate", 4, func(s *sketch.Sketch, fixed func(u, v float64) *sketch.Point) {
			p0, p1, p2, p3 := fixed(0, 0), fixed(20, 0), fixed(20, 10), fixed(13, 10)
			q0, q1, nc := fixed(7, 10), fixed(0, 10), fixed(10, 10)
			s.CreateLine(p0, p1)
			s.CreateLine(p1, p2)
			s.CreateLine(p2, p3)
			s.CreateArc(nc, q0, p3)
			s.CreateLine(q0, q1)
			s.CreateLine(q1, p0)
		}, 8, 7},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := build(t, tc.h, tc.draw)
			f, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.1), header())
			require.NoError(t, err)
			counts, uses := entityUses(f)
			require.Equal(t, tc.faces, counts["ADVANCED_FACE"])
			require.Equal(t, tc.planes, counts["PLANE"])
			require.Equal(t, 1, counts["CYLINDRICAL_SURFACE"])
			require.Equal(t, 2, counts["CIRCLE"], "one circle per arc edge, top and bottom")
			for _, senses := range uses {
				require.ElementsMatch(t, []step.Enumeration{"T", "F"}, senses)
			}
			data, err := f.Marshal()
			require.NoError(t, err)
			require.Contains(t, string(data), "analytic decad solid")
		})
	}
}

// entityUses counts a file's entities by name and lists, per EDGE_CURVE, the
// sense of every ORIENTED_EDGE that uses it.
func entityUses(f step.File) (map[string]int, map[step.Reference][]step.Enumeration) {
	counts := map[string]int{}
	uses := map[step.Reference][]step.Enumeration{}
	for _, entity := range f.Entities {
		counts[entity.Name]++
		if entity.Name == "ORIENTED_EDGE" {
			edge := entity.Parameters[3].(step.Reference)
			uses[edge] = append(uses[edge], entity.Parameters[4].(step.Enumeration))
		}
	}
	return counts, uses
}

// lPrism extrudes the L of 20×5 and 5×15 (its one reflex corner at (5, 5))
// drawn as six fixed lines, the corners listed from start onward.
func lPrism(t *testing.T, start int) *decad.Body {
	t.Helper()
	corners := [][2]float64{{0, 0}, {20, 0}, {20, 5}, {5, 5}, {5, 20}, {0, 20}}
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := make([]*sketch.Point, len(corners))
	for i := range corners {
		c := corners[(start+i)%len(corners)]
		pts[i] = s.CreatePoint(c[0], c[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	body, err := decad.New().Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// firstCornerReflex reports whether the face's outer loop turns against the
// face's outward normal at its first vertex: a reflex corner there.
func firstCornerReflex(t *testing.T, face *decad.Face) bool {
	t.Helper()
	coedges := face.Loops()[0].CoEdges()
	last := coedges[len(coedges)-1]
	in := last.End().Position().Value.Sub(last.Start().Position().Value)
	out := coedges[0].End().Position().Value.Sub(coedges[0].Start().Position().Value)
	normal, err := face.NormalAt(coedges[0].Start().Position().Value)
	require.NoError(t, err)
	return in.Cross(out).Dot(normal.Value) < 0
}

// TestNewSTEPFileLPrismWithReflexFirstCorner writes the L prism drawn from
// each of its six corners. Wherever a cap's outer loop starts at the reflex
// corner, the plane placement cannot read the loop's sense off its first turn;
// the signed-area placement reads it off the whole loop. Every file must use
// each EDGE_CURVE once in each sense, and at least one start must put a cap's
// first vertex on the reflex corner, or the fixture tests nothing. Shown to
// fail with planarSTEPPlacement reading a line loop's axis off its first turn
// (a reflex-first cap reversed its sense, so its edges' uses matched their
// walls').
func TestNewSTEPFileLPrismWithReflexFirstCorner(t *testing.T) {
	t.Parallel()
	reflexFirst := 0
	for start := range 6 {
		body := lPrism(t, start)
		for _, face := range body.Faces() {
			if _, ok := face.Surface().(decad.Plane); ok && len(face.Loops()[0].CoEdges()) == 6 && firstCornerReflex(t, face) {
				reflexFirst++
			}
		}
		f, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.1), header())
		require.NoError(t, err)
		counts, uses := entityUses(f)
		require.Equal(t, 8, counts["ADVANCED_FACE"])
		for _, senses := range uses {
			require.ElementsMatch(t, []step.Enumeration{"T", "F"}, senses, "start %d", start)
		}
	}
	require.Positive(t, reflexFirst, "some start must put a cap's first vertex on the reflex corner")
}

// classBCut is general-boolean §9's class B fixture through the public API:
// a 40×20×20 box cut by a tool drawn on an XZ plane moved to y = 10 and
// extruded 11 mm to each side, so it runs through the box along y.
func classBCut(t *testing.T, draw func(s *sketch.Sketch)) *decad.Body {
	t.Helper()
	doc := decad.New()
	w := sketch.NewWorld()
	base, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := base.CreateRectangle(0, 0, 40, 20)
	base.Fix(rect.A)
	_, err = base.Solve(t.Context())
	require.NoError(t, err)
	box, err := doc.Extrude(base, base.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	plane, err := w.CreateOffsetPlane(w.XZ(), -10)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	draw(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := doc.Extrude(s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(11)})
	require.NoError(t, err)
	result, err := decad.Cut(t.Context(), box, tool)
	require.NoError(t, err)
	return result
}

// TestNewSTEPFileClassBCutAnalytic writes two class B Cut results — brep
// bodies a public boolean builds — through the analytic arm: S1, the box
// drilled Ø6 along y through (20, ·, 10) (6 planes and one full cylinder
// wall), and B1, the box cut by a 10×10 slot along y (10 planes). Each file
// is the analytic one with the face count the result has, and uses every
// EDGE_CURVE exactly once in each sense, which a closed shell whose faces'
// loops each run about their own face normal requires. Shown to fail with
// addPlanarFace's face sense inverted (S1's planes then disagreed with its
// cylinder wall at both rims) and with supportsAnalyticSTEP refusing every
// body (both files took the faceted writer).
func TestNewSTEPFileClassBCutAnalytic(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name                     string
		draw                     func(s *sketch.Sketch)
		faces, planes, cylinders int
	}{
		{"S1 cross drill", func(s *sketch.Sketch) {
			c := s.CreatePoint(20, 10)
			s.Fix(c)
			s.CreateCircle(c, 3)
		}, 7, 6, 1},
		{"B1 cross slot", func(s *sketch.Sketch) {
			r := s.CreateRectangle(15, 5, 25, 15)
			s.Fix(r.A)
		}, 10, 10, 0},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := classBCut(t, tc.draw)
			require.Len(t, body.Faces(), tc.faces)
			f, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.1), header())
			require.NoError(t, err)
			counts, uses := entityUses(f)
			require.Equal(t, tc.faces, counts["ADVANCED_FACE"])
			require.Equal(t, tc.planes, counts["PLANE"])
			require.Equal(t, tc.cylinders, counts["CYLINDRICAL_SURFACE"])
			for _, senses := range uses {
				require.ElementsMatch(t, []step.Enumeration{"T", "F"}, senses)
			}
			data, err := f.Marshal()
			require.NoError(t, err)
			require.Contains(t, string(data), "analytic decad solid")
		})
	}
}

func TestNewSTEPFileConeUsesFacetedFallback(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	apex := s.CreatePoint(10, 0)
	top := s.CreatePoint(0, 5)
	s.CreateLine(origin, apex)
	s.CreateLine(apex, top)
	s.CreateLine(top, origin)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	body, err := decad.New().Revolve(s, s.Profiles()[0], axis, decad.FullRevolution{})
	require.NoError(t, err)
	f, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.5), header())
	require.NoError(t, err)
	counts := map[string]int{}
	for _, entity := range f.Entities {
		counts[entity.Name]++
	}
	require.Greater(t, counts["ADVANCED_FACE"], 3)
	require.Equal(t, counts["ADVANCED_FACE"], counts["PLANE"])
	require.Zero(t, counts["CYLINDRICAL_SURFACE"])
	data, err := f.Marshal()
	require.NoError(t, err)
	require.Contains(t, string(data), "faceted decad solid")
}

func TestSTEPBlindPocketOneShell(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	w := sketch.NewWorld()
	plateSketch, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	plateRect := plateSketch.CreateRectangle(0, 0, 10, 10)
	plateSketch.Fix(plateRect.A)
	_, err = plateSketch.Solve(t.Context())
	require.NoError(t, err)
	plate, err := doc.Extrude(plateSketch, plateSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	toolPlane, err := w.CreateOffsetPlane(w.XY(), 6)
	require.NoError(t, err)
	toolSketch, err := w.CreateSketch(toolPlane)
	require.NoError(t, err)
	toolRect := toolSketch.CreateRectangle(3, 3, 7, 7)
	toolSketch.Fix(toolRect.A)
	_, err = toolSketch.Solve(t.Context())
	require.NoError(t, err)
	tool, err := doc.Extrude(toolSketch, toolSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	require.NoError(t, err)
	pocket, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	f, err := export.NewSTEPFile(t.Context(), pocket, units.Millimeters(0.1), header())
	require.NoError(t, err)
	shells := 0
	for _, entity := range f.Entities {
		if entity.Name == "CLOSED_SHELL" {
			shells++
		}
	}
	require.Equal(t, 1, shells)
}

func TestWriteDeterministicAndUntouchedOnInvalidHeader(t *testing.T) {
	t.Parallel()
	body := box(t, false)
	var first, second bytes.Buffer
	opts := []export.STEPOption{
		export.WithSTEPName("box.step"),
		export.WithSTEPTimestamp(time.Date(2026, 9, 25, 0, 0, 0, 0, time.UTC)),
		export.WithSTEPAuthor("Decad"),
		export.WithSTEPOrganization("Decad"),
	}
	require.NoError(t, export.STEP(t.Context(), &first, body, units.Millimeters(0.1), opts...))
	require.NoError(t, export.STEP(t.Context(), &second, body, units.Millimeters(0.1), opts...))
	require.Equal(t, first.Bytes(), second.Bytes())
	require.Equal(t, 6, strings.Count(first.String(), "=ADVANCED_FACE("))
	require.Contains(t, first.String(), "analytic decad solid")
	require.Contains(t, first.String(), "decad export")

	full := header()
	full.Name = "override.step"
	var overridden bytes.Buffer
	require.NoError(t, export.STEP(t.Context(), &overridden, body, units.Millimeters(0.1),
		export.WithSTEPHeader(header()), export.WithSTEPHeader(full)))
	require.Contains(t, overridden.String(), "override.step")
	require.Contains(t, overridden.String(), "faceted AP214 test")
	require.Contains(t, overridden.String(), "FILE_DESCRIPTION(('faceted AP214 test')")
	require.NotContains(t, overridden.String(), "FILE_DESCRIPTION(('decad solid')")

	bad := header()
	bad.Timestamp = time.Time{}
	var untouched bytes.Buffer
	untouched.WriteString("original")
	require.Error(t, export.STEP(t.Context(), &untouched, body, units.Millimeters(0.1), export.WithSTEPHeader(bad)))
	require.Equal(t, "original", untouched.String())
	require.ErrorIs(t, export.STEP(t.Context(), &untouched, body, units.Millimeters(0.1)), decad.ErrDegenerate)
	require.Equal(t, "original", untouched.String())
}

func TestSTEPHeaderUsesEntireValue(t *testing.T) {
	t.Parallel()
	body := box(t, false)
	base := step.Header{
		Description:   []string{"full header"},
		Name:          "base.step",
		Timestamp:     time.Date(2026, 9, 25, 0, 0, 0, 0, time.UTC),
		Authors:       []string{"Original Author"},
		Organizations: []string{"Original Organization"},
	}
	var out bytes.Buffer
	require.NoError(t, export.STEP(t.Context(), &out, body, units.Millimeters(0.1), export.WithSTEPHeader(base)))
	require.Contains(t, out.String(), "FILE_DESCRIPTION(('full header')")
	require.Contains(t, out.String(), "FILE_NAME('base.step','2026-09-25T00:00:00Z',('Original Author'),('Original Organization'),'','','')")

	out.Reset()
	out.WriteString("original")
	require.ErrorIs(t, export.STEP(t.Context(), &out, body, units.Millimeters(0.1),
		export.WithSTEPHeader(base), export.WithSTEPName("changed.step")), decad.ErrDegenerate)
	require.Equal(t, "original", out.String())
}

func TestSTEPDefaultTimestampAndRequiredFields(t *testing.T) {
	t.Parallel()
	body := box(t, false)
	var out bytes.Buffer
	require.NoError(t, export.STEP(t.Context(), &out, body, units.Millimeters(0.1),
		export.WithSTEPName("box.step"), export.WithSTEPAuthor("Decad"), export.WithSTEPOrganization("Decad")))
	require.Contains(t, out.String(), "FILE_NAME('box.step','")
	require.NotContains(t, out.String(), "FILE_NAME('box.step','0001-")

	out.Reset()
	require.ErrorContains(t, export.STEP(t.Context(), &out, body, units.Millimeters(0.1),
		export.WithSTEPName("box.step"), export.WithSTEPOrganization("Decad")), "author")
	require.Empty(t, out.String())
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

// flushBossUnion is general-boolean §9's A1 brep fixture through the public
// API: a 40×40×10 plate on XY unioned with a boss standing on its top at
// z = 10..25, drawn by draw on the offset plane.
func flushBossUnion(t *testing.T, draw func(s *sketch.Sketch)) *decad.Body {
	t.Helper()
	doc := decad.New()
	w := sketch.NewWorld()
	base, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := base.CreateRectangle(-20, -20, 20, 20)
	base.Fix(rect.A)
	_, err = base.Solve(t.Context())
	require.NoError(t, err)
	plate, err := doc.Extrude(base, base.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	plane, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	draw(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	boss, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(15), Dir: decad.Along})
	require.NoError(t, err)
	result, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	return result
}

// TestNewSTEPFileFlushBossUnionAnalytic writes two A1 brep results through
// the analytic arm: the flush corner boss (9 planes, two of them L-shaped
// merged walls) and the round boss crossing the plate's outline (8 planes and
// the boss's wall as two partial cylinders with arc rims). Each file uses
// every EDGE_CURVE exactly once in each sense.
func TestNewSTEPFileFlushBossUnionAnalytic(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name                     string
		draw                     func(s *sketch.Sketch)
		faces, planes, cylinders int
	}{
		{"flush corner boss", func(s *sketch.Sketch) {
			r := s.CreateRectangle(10, -20, 20, -10)
			s.Fix(r.A)
		}, 9, 9, 0},
		{"crossing round boss", func(s *sketch.Sketch) {
			c := s.CreatePoint(18, 0)
			s.Fix(c)
			s.CreateCircle(c, 5)
		}, 10, 8, 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := flushBossUnion(t, tc.draw)
			require.Len(t, body.Faces(), tc.faces)
			f, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.1), header())
			require.NoError(t, err)
			counts, uses := entityUses(f)
			require.Equal(t, tc.faces, counts["ADVANCED_FACE"])
			require.Equal(t, tc.planes, counts["PLANE"])
			require.Equal(t, tc.cylinders, counts["CYLINDRICAL_SURFACE"])
			for _, senses := range uses {
				require.ElementsMatch(t, []step.Enumeration{"T", "F"}, senses)
			}
			data, err := f.Marshal()
			require.NoError(t, err)
			require.Contains(t, string(data), "analytic decad solid")
		})
	}
}

// TestNewSTEPFileRootedCrossingBossAnalytic writes the rooted round crossing
// boss (general-boolean §9): 8 planes and two partial cylinders, the
// overhanging one bounded by six edges since its side lines split at the
// plate's top, each file using every EDGE_CURVE once in each sense. Shown to
// fail with supportsAnalyticPartialWall's four-edge rule restored (the file
// took the faceted writer).
func TestNewSTEPFileRootedCrossingBossAnalytic(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	w := sketch.NewWorld()
	base, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := base.CreateRectangle(-20, -20, 20, 20)
	base.Fix(rect.A)
	_, err = base.Solve(t.Context())
	require.NoError(t, err)
	plate, err := doc.Extrude(base, base.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	plane, err := w.CreateOffsetPlane(w.XY(), 5)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(18, 0)
	s.Fix(c)
	s.CreateCircle(c, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	boss, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	body, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	require.Len(t, body.Faces(), 10)
	f, err := export.NewSTEPFile(t.Context(), body, units.Millimeters(0.1), header())
	require.NoError(t, err)
	counts, uses := entityUses(f)
	require.Equal(t, 10, counts["ADVANCED_FACE"])
	require.Equal(t, 8, counts["PLANE"])
	require.Equal(t, 2, counts["CYLINDRICAL_SURFACE"])
	for _, senses := range uses {
		require.ElementsMatch(t, []step.Enumeration{"T", "F"}, senses)
	}
	data, err := f.Marshal()
	require.NoError(t, err)
	require.Contains(t, string(data), "analytic decad solid")
}
