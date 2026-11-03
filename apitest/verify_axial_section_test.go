package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin agent-docs/interference-design.md §3.2's axial section
// separator: a turned screw about the y axis, its D flat milled by a box
// tool that sweeps across the axis, sits in a knob whose hole is no circle
// of the screw's. The screw less its tool lies in the hole over the knob's
// levels, though the whole turned cylinder does not.

// axialScrew revolves a Ø8 shaft along y from 0 to 40.
func axialScrew(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, profile := polygonSketch(t, [][2]float64{{0, 0}, {4, 0}, {4, 40}, {0, 40}})
	body, err := decad.Revolve(t.Context(), doc, s, profile,
		decad.SketchLine{End: decad.Point2{V: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// screwFlatTool is the box x in [3, 5], y in [10, y1], z in [-5, 5].
func screwFlatTool(t *testing.T, doc *decad.Document, y1 float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateRectangle(3, 10, 5, y1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(5)})
	require.NoError(t, err)
	return body
}

// axialKnob is a Ø24 knob 8 mm long from y = 12 about the y axis, its hole the
// rectangle x in [hx0, hx1], z in [-hz, hz], or a circle of radius round when
// round is positive.
func axialKnob(t *testing.T, doc *decad.Document, hx0, hx1, hz, round float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	// The XZ plane's normal is -y: the knob hangs 8 mm below the plane at y = 20.
	plane, err := w.CreateOffsetPlane(w.XZ(), -(12 + 8))
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 12)
	if round > 0 {
		inner := s.CreatePoint(0, 0)
		s.Fix(inner)
		s.CreateCircle(inner, round)
	} else {
		s.CreateRectangle(hx0, -hz, hx1, hz)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var ring *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			ring = p
		}
	}
	require.NotNil(t, ring, "the knob's section has one hole")
	body, err := decad.Extrude(t.Context(), doc, s, ring,
		decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestVerifyAxialSectionSeparator holds the separator to what the screw less
// its flat lies in: a rectangular hole whose wall stands 0.001 mm past the
// flat holds the cut screw and decides the pair, a gap the held meshes cannot
// certify; a hole that clips the D, a tool that stops short of
// the knob's levels, and the screw with no flat are not decided.
//
// Leg shown to fail: reading no cutting tool at the Cut leaves the D-flat
// pair with a pair diagnostic.
func TestVerifyAxialSectionSeparator(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		hx1       float64
		toolEnd   float64
		flat      bool
		round     float64
		separated bool
	}{
		{"flat screw in a hole 0.001 mm past the D", 3.001, 45, true, 0, true},
		{"hole clips the D", 2.9, 45, true, 0, false},
		{"tool stops inside the knob", 3.001, 16, true, 0, false},
		{"screw without the flat", 3.001, 45, false, 0, false},
		{"flat screw in a round bore of its own radius", 0, 45, true, 4, true},
		{"flat screw in a round bore 0.1 mm too small", 0, 45, true, 3.9, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			screw := axialScrew(t, doc)
			if tc.flat {
				var err error
				screw, err = decad.Cut(t.Context(), screw, screwFlatTool(t, doc, tc.toolEnd))
				require.NoError(t, err)
			}
			knob := axialKnob(t, doc, -4.5, tc.hx1, 4.5, tc.round)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, screw, knob)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the axial section separator decides the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping or unproven pair is never called separated")
		})
	}
}

// TestVerifyAxialSectionTurnedMate turns the knob and the screw's flat tool by
// one lift about the screw's axis, an angle no float holds, while the screw
// keeps its own. The screw's lift is not exactly a rotation of the knob's, so
// its cover widens by the proven rounding, a margin far below 0.001 mm. A hole
// wall 0.001 mm past the flat is decided; a wall on the flat, whose boundaries
// may cross by that rounding, is not.
//
// Leg shown to fail: reading the cover only across exactly orthonormal lifts
// leaves the 0.001 mm pair with a pair diagnostic.
func TestVerifyAxialSectionTurnedMate(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		hx1       float64
		separated bool
	}{
		{"wall 0.001 mm past the flat", 3.001, true},
		{"wall on the flat", 3, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			screw := axialScrew(t, doc)
			turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Y: 1}, units.Radians(0.7))
			require.NoError(t, err)
			tool, err := decad.Transform(t.Context(), screwFlatTool(t, doc, 45), turn)
			require.NoError(t, err)
			cut, err := decad.Cut(t.Context(), screw, tool)
			require.NoError(t, err)
			knob, err := decad.Transform(t.Context(), axialKnob(t, doc, -4.5, tc.hx1, 4.5, 0), turn)
			require.NoError(t, err)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, cut, knob)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the widened cover decides the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "a wall the cover may cross is never called separated")
		})
	}
}

// TestVerifyAxialSectionShoulderAtLevel puts a Ø20 shoulder of the turned
// shaft exactly on the knob's top level, y = 20, and mills the flat along the
// shaft. The shoulder bounds no volume inside the knob's levels, so the Ø8
// shaft below it is the whole cover.
//
// Leg shown to fail: counting the shoulder's vertex in the closed slab widens
// the cover to its Ø20 and leaves the pair with a diagnostic.
func TestVerifyAxialSectionShoulderAtLevel(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, profile := polygonSketch(t, [][2]float64{{0, 0}, {4, 0}, {4, 20}, {10, 20}, {10, 22}, {4, 22}, {4, 40}, {0, 40}})
	shaft, err := decad.Revolve(t.Context(), doc, s, profile,
		decad.SketchLine{End: decad.Point2{V: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	shaft, err = decad.Cut(t.Context(), shaft, screwFlatTool(t, doc, 45))
	require.NoError(t, err)
	knob := axialKnob(t, doc, -4.5, 3.001, 4.5, 0)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	interferences, diagnostics := pairFindings(report, shaft, knob)
	require.Zero(t, interferences)
	require.Zero(t, diagnostics)
}

// roundShaft extrudes a Ø8 circle along y from 0 to 40, a prism rather than a
// Revolve, about the y axis.
func roundShaft(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XZ(), -40)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 4)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(40), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestVerifyAxialSectionRoundPrismTurnedMate seats a round prism shaft, milled with a flat, in a bore
// on a lift turned about the shaft's axis by an angle no float holds. The shaft is
// read as a cylinder about its axis, widened by the proven rounding, so a bore
// 0.001 mm wider is decided and a bore 0.001 mm narrower, which the shaft
// crosses, is not.
//
// Leg shown to fail: reading a round prism on another lift as nothing leaves the
// 0.001 mm pair with a pair diagnostic.
func TestVerifyAxialSectionRoundPrismTurnedMate(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		bore      float64
		separated bool
	}{
		{"bore 0.001 mm wider", 4.001, true},
		{"bore 0.001 mm narrower", 3.999, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			shaft, err := decad.Cut(t.Context(), roundShaft(t, doc), screwFlatTool(t, doc, 45))
			require.NoError(t, err)
			turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Y: 1}, units.Radians(0.7))
			require.NoError(t, err)
			knob, err := decad.Transform(t.Context(), axialKnob(t, doc, 0, 0, 0, tc.bore), turn)
			require.NoError(t, err)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, shaft, knob)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the widened cylinder lies in the bore")
				return
			}
			require.Positive(t, interferences+diagnostics, "a bore the shaft crosses is never called separated")
		})
	}
}
