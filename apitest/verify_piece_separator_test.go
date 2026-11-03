package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin agent-docs/interference-design.md §3.2's piece separator:
// a bracket prism swept along z with a knuckle arc standing beside a vertical
// screw hole, and a turned screw whose shank sits in the hole and whose head
// sits flush on the bracket's top face. The head's revolution reaches the
// knuckle's height, and the shank lies in the hole, so neither the meridian nor
// the axial section separator decides the pair; the screw's slabs do.

// pieceBracket is the bracket section in (x, y): the block x in [-10, 12],
// y in [0, 10] with a knuckle arc of radius 4 about (8, 10) standing on its
// right end, swept 15 mm along z about z = 0, less a vertical hole of radius
// hole about the y axis that runs 5 mm past both faces.
func pieceBracket(t *testing.T, doc *decad.Document, hole float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a, b := s.CreatePoint(-10, 0), s.CreatePoint(12, 0)
	c, d := s.CreatePoint(12, 10), s.CreatePoint(4, 10)
	e := s.CreatePoint(-10, 10)
	centre := s.CreatePoint(8, 10)
	s.Fix(a)
	s.Fix(centre)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateArc(centre, c, d)
	s.CreateLine(d, e)
	s.CreateLine(e, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(7.5)})
	require.NoError(t, err)
	return pieceHole(t, doc, body, hole)
}

func pieceHole(t *testing.T, doc *decad.Document, host *decad.Body, radius float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	// The XZ plane's normal is -y: the tool hangs 25 mm below the plane at y = 20.
	plane, err := w.CreateOffsetPlane(w.XZ(), -20)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(25), Dir: decad.Along})
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), host, tool)
	require.NoError(t, err)
	return out
}

// pieceScrew is a screw turned about the y axis: a shank of radius 1.5 from
// y = -8 to y = 10 and a head of radius 3 from y = 10 up by headHeight.
func pieceScrew(t *testing.T, doc *decad.Document, headBase, headHeight float64) *decad.Body {
	t.Helper()
	s, profile := polygonSketch(t, [][2]float64{{0, -8}, {1.5, -8}, {1.5, headBase}, {3, headBase},
		{3, headBase + headHeight}, {0, headBase + headHeight}})
	body, err := decad.Revolve(t.Context(), doc, s, profile,
		decad.SketchLine{End: decad.Point2{V: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// TestVerifyPieceSeparator holds the separator to what each slab of the screw
// lies in: a head flush on the bracket and a shank in the hole is decided; a
// head sunk 0.5 mm into the bracket, and a hole narrower than the shank, are
// not.
//
// Leg shown to fail: reading no slabs of the screw leaves the flush pair with a
// pair diagnostic.
func TestVerifyPieceSeparator(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		hole      float64
		headBase  float64
		separated bool
	}{
		{"head flush, shank in the hole", 1.6, 10, true},
		{"head sunk into the bracket", 1.6, 9.5, false},
		{"hole narrower than the shank", 1.4, 10, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			bracket := pieceBracket(t, doc, tc.hole)
			screw := pieceScrew(t, doc, tc.headBase, 1.5)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, bracket, screw)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the piece separator decides the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping pair is never called separated")
		})
	}
}
