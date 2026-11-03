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
func pieceBracket(t *testing.T, doc *decad.Document, hole, s float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	sk, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a, b := sk.CreatePoint(-10*s, 0), sk.CreatePoint(12*s, 0)
	c, d := sk.CreatePoint(12*s, 10*s), sk.CreatePoint(4*s, 10*s)
	e := sk.CreatePoint(-10*s, 10*s)
	centre := sk.CreatePoint(8*s, 10*s)
	sk.Fix(a)
	sk.Fix(centre)
	sk.CreateLine(a, b)
	sk.CreateLine(b, c)
	sk.CreateArc(centre, c, d)
	sk.CreateLine(d, e)
	sk.CreateLine(e, a)
	_, err = sk.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, sk.Profiles(), 1)
	body, err := decad.Extrude(t.Context(), doc, sk, sk.Profiles()[0], decad.Symmetric{D: units.Millimeters(7.5 * s)})
	require.NoError(t, err)
	return pieceHole(t, doc, body, hole, s)
}

func pieceHole(t *testing.T, doc *decad.Document, host *decad.Body, radius, s float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	// The XZ plane's normal is -y: the tool hangs 25 mm below the plane at y = 20.
	plane, err := w.CreateOffsetPlane(w.XZ(), -20*s)
	require.NoError(t, err)
	sk, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := sk.CreatePoint(0, 0)
	sk.Fix(c)
	sk.CreateCircle(c, radius)
	_, err = sk.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, sk, sk.Profiles()[0], decad.Distance{D: units.Millimeters(25 * s), Dir: decad.Along})
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), host, tool)
	require.NoError(t, err)
	return out
}

// pieceScrew is a screw turned about the y axis: a shank of radius 1.5 from
// y = -8 to y = 10 and a head of radius 3 from y = 10 up by headHeight.
func pieceScrew(t *testing.T, doc *decad.Document, headBase, headHeight, s float64) *decad.Body {
	t.Helper()
	sk, profile := polygonSketch(t, [][2]float64{{0, -8 * s}, {1.5 * s, -8 * s}, {1.5 * s, headBase * s}, {3 * s, headBase * s},
		{3 * s, (headBase + headHeight) * s}, {0, (headBase + headHeight) * s}})
	body, err := decad.Revolve(t.Context(), doc, sk, profile,
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
		scale     float64
		separated bool
	}{
		{"head flush, shank in the hole", 1.6, 10, 1, true},
		{"head flush at scale 0.8, arc ends off their circle by rounding", 1.6, 10, 0.8, true},
		{"head sunk into the bracket", 1.6, 9.5, 1, false},
		{"hole narrower than the shank", 1.4, 10, 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			bracket := pieceBracket(t, doc, tc.hole*tc.scale, tc.scale)
			screw := pieceScrew(t, doc, tc.headBase, 1.5, tc.scale)
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

// pieceLeaf is a hinge-leaf-like target: a plate x in [-10, 6], y in [0, 3] and a
// knuckle disc of radius 5 about (10, 3), both swept 15 mm along z about z = 0
// and joined, less a vertical hole of radius 1.6 about the y axis from y = -5 up
// to holeTop. The knuckle stands higher than the hole's top, but clear of the
// screw, so only the plate lies where the hole is.
func pieceLeaf(t *testing.T, doc *decad.Document, holeTop float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateRectangle(-10, 0, 6, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	plate, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(7.5)})
	require.NoError(t, err)
	w2 := sketch.NewWorld()
	s2, err := w2.CreateSketch(w2.XY())
	require.NoError(t, err)
	c := s2.CreatePoint(10, 3)
	s2.Fix(c)
	s2.CreateCircle(c, 5)
	_, err = s2.Solve(t.Context())
	require.NoError(t, err)
	knuckle, err := decad.Extrude(t.Context(), doc, s2, s2.Profiles()[0], decad.Symmetric{D: units.Millimeters(7.5)})
	require.NoError(t, err)
	both, err := decad.Union(t.Context(), plate, knuckle)
	require.NoError(t, err)
	w3 := sketch.NewWorld()
	// The XZ plane's normal is -y: the tool hangs below the plane at y = holeTop.
	plane, err := w3.CreateOffsetPlane(w3.XZ(), -holeTop)
	require.NoError(t, err)
	s3, err := w3.CreateSketch(plane)
	require.NoError(t, err)
	hc := s3.CreatePoint(0, 0)
	s3.Fix(hc)
	s3.CreateCircle(hc, 1.6)
	_, err = s3.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, s3, s3.Profiles()[0],
		decad.Distance{D: units.Millimeters(holeTop + 5), Dir: decad.Along})
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), both, tool)
	require.NoError(t, err)
	return out
}

// TestVerifyPieceSeparatorStripReach holds the separator to the target's reach
// inside the slab's strip. The leaf's knuckle stands above the hole's top, so the
// target's whole reach along the screw's axis leaves the hole's levels, but the
// plate is all that lies across the shank: a hole that runs past the plate is
// decided, and one that stops 0.5 mm inside it is not.
//
// Leg shown to fail: reading the target's whole axial reach instead of its
// reach within the strip leaves the first case with a pair diagnostic.
func TestVerifyPieceSeparatorStripReach(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		holeTop   float64
		separated bool
	}{
		{"hole runs past the plate", 3.4, true},
		{"hole stops inside the plate", 2.5, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			leaf := pieceLeaf(t, doc, tc.holeTop)
			screw := pieceScrew(t, doc, 3, 1.5, 1)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, leaf, screw)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the strip reach decides the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping pair is never called separated")
		})
	}
}

// TestVerifyBoxInRecess seats a plate, swept along y, flush in a recess its bar's
// tool milled, a box swept along z: the plate's five faces lie in the recess's.
// The plate lies in the tool's closed solid whichever way each is swept, so the
// bar's Cut result misses it. A plate 0.5 mm wider than the recess overlaps the
// bar and is not decided.
//
// Leg shown to fail: containing only boxes on one frame leaves the flush plate
// with a pair diagnostic.
func TestVerifyBoxInRecess(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		x0        float64
		separated bool
	}{
		{"plate flush in the recess", 5, true},
		{"plate 0.5 mm too wide", 4.5, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			s.CreateRectangle(0, 0, 40, 20)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			bar, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.NoError(t, err)

			w2 := sketch.NewWorld()
			up, err := w2.CreateOffsetPlane(w2.XY(), 5)
			require.NoError(t, err)
			s2, err := w2.CreateSketch(up)
			require.NoError(t, err)
			s2.CreateRectangle(5, 5, 25, 15)
			_, err = s2.Solve(t.Context())
			require.NoError(t, err)
			recess, err := decad.Extrude(t.Context(), doc, s2, s2.Profiles()[0], decad.Distance{D: units.Millimeters(6), Dir: decad.Along})
			require.NoError(t, err)
			bar, err = decad.Cut(t.Context(), bar, recess)
			require.NoError(t, err)

			w3 := sketch.NewWorld()
			// The XZ plane's normal is -y: the plate hangs 10 mm below the plane at y = 15.
			side, err := w3.CreateOffsetPlane(w3.XZ(), -15)
			require.NoError(t, err)
			s3, err := w3.CreateSketch(side)
			require.NoError(t, err)
			s3.CreateRectangle(tc.x0, 5, 25, 8)
			_, err = s3.Solve(t.Context())
			require.NoError(t, err)
			plate, err := decad.Extrude(t.Context(), doc, s3, s3.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.NoError(t, err)

			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, bar, plate)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the plate lies in the recess's tool")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping pair is never called separated")
		})
	}
}

// TestVerifyPieceTangentKnuckle seats a screw's head under a hinge leaf as the
// scaled gallery model does: a plate x in [2.4, 11.2], y in [0, 2.4] joined to a
// knuckle disc of radius 5 about the origin, a hole of radius 1.6 about x = 8,
// and a screw about that axis whose head of radius 3 lies flush under the plate.
// The head's rim reaches x = 5, the disc's far side, so the two touch along a
// line. An arrangement cannot certify a tangency, but the head's footprint and
// the disc lie on either side of x = 5, which decides the pair. A screw 0.5 mm
// nearer the knuckle overlaps it and is not decided.
//
// Leg shown to fail: leaving the tangent knuckle to the arrangement alone leaves
// the touching case with a pair diagnostic.
func TestVerifyPieceTangentKnuckle(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		axisX     float64
		separated bool
	}{
		{"head rim tangent to the knuckle", 8, true},
		{"head rim 0.5 mm into the knuckle", 7.5, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			s.CreateRectangle(2.4, 0, 11.2, 2.4)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			plate, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(6)})
			require.NoError(t, err)
			w2 := sketch.NewWorld()
			s2, err := w2.CreateSketch(w2.XY())
			require.NoError(t, err)
			c := s2.CreatePoint(0, 0)
			s2.Fix(c)
			s2.CreateCircle(c, 5)
			_, err = s2.Solve(t.Context())
			require.NoError(t, err)
			knuckle, err := decad.Extrude(t.Context(), doc, s2, s2.Profiles()[0], decad.Symmetric{D: units.Millimeters(6)})
			require.NoError(t, err)
			leaf, err := decad.Union(t.Context(), plate, knuckle)
			require.NoError(t, err)
			w3 := sketch.NewWorld()
			// The XZ plane's normal is -y: the hole tool hangs below the plane at y = 3.4.
			plane, err := w3.CreateOffsetPlane(w3.XZ(), -3.4)
			require.NoError(t, err)
			s3, err := w3.CreateSketch(plane)
			require.NoError(t, err)
			hc := s3.CreatePoint(8, 0)
			s3.Fix(hc)
			s3.CreateCircle(hc, 1.6)
			_, err = s3.Solve(t.Context())
			require.NoError(t, err)
			tool, err := decad.Extrude(t.Context(), doc, s3, s3.Profiles()[0], decad.Distance{D: units.Millimeters(8.4), Dir: decad.Along})
			require.NoError(t, err)
			leaf, err = decad.Cut(t.Context(), leaf, tool)
			require.NoError(t, err)

			x := tc.axisX
			sk, profile := polygonSketch(t, [][2]float64{{x, -1.5}, {x + 3, -1.5}, {x + 3, 0}, {x + 1.5, 0}, {x + 1.5, 11}, {x, 11}})
			screw, err := decad.Revolve(t.Context(), doc, sk, profile,
				decad.SketchLine{Start: decad.Point2{U: x}, End: decad.Point2{U: x, V: 1}}, decad.FullRevolution{})
			require.NoError(t, err)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, leaf, screw)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the boxes on either side of x = 5 decide the pair")
				return
			}
			require.Positive(t, interferences+diagnostics, "an overlapping pair is never called separated")
		})
	}
}
