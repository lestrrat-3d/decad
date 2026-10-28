package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// stopTool extrudes the 4x4 square 8..12 on XY by e, into doc.
func stopTool(t *testing.T, doc *decad.Document, e decad.Extent) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(8, 8, 12, 12)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], e)
	require.NoError(t, err)
	return tool
}

// upPlate extrudes the 20x20 square on XY up by top.
func upPlate(t *testing.T, doc *decad.Document, top units.Value) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	plate, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: top, Dir: decad.Along})
	require.NoError(t, err)
	return plate
}

// facing selects the one planar face of b whose normal is n.
func facing(n r3.Vec) *decad.FaceQuery {
	return decad.Faces(decad.Planar(), decad.Facing(n))
}

// requireDenotedThroughHole requires part to be the analytic 20x20 plate with
// one 4x4 through hole, its volume bound covering 384 mm² over height.
func requireDenotedThroughHole(t *testing.T, part *decad.Body, height *big.Rat) {
	t.Helper()
	require.False(t, tookMeshPath(part), "the stop denotes the plate's cap, so the span is proven")
	require.Len(t, part.Lumps(), 1)
	require.Len(t, part.Faces(), 10)
	requireRationalVolumeCovered(t, part, new(big.Rat).Mul(big.NewRat(384, 1), height))
}

// requireDeclinedSpan requires a Cut that proves no span: the mesh path's
// result covering want.
func requireDeclinedSpan(t *testing.T, part *decad.Body, err error, want *big.Rat) {
	t.Helper()
	require.NoError(t, err)
	require.True(t, tookMeshPath(part), "an undenoted stop within its displacement of the cap is no proven span")
	requireRationalVolumeCovered(t, part, want)
}

// TestThroughStopDenotedLevelFixtures pins that 0.2 in and 0.3 in convert
// with a rounding, so a plate swept by either carries a displaced top.
func TestThroughStopDenotedLevelFixtures(t *testing.T) {
	t.Parallel()
	for _, v := range []units.Value{units.Inches(0.2), units.Inches(0.3)} {
		_, exact := denotedDepth(v).Float64()
		require.False(t, exact, "%v converts with a rounding", v)
	}
}

// TestThroughStopDenotedLevelThroughHole cuts a plate whose top is a
// displaced inch level with a tool whose stops are the plate's own caps: a
// ThroughAll, a ThroughAll on both sides, a ToFace and a ToFace on both
// sides. Each stop denotes the cap it stops at, so the cut is the analytic
// through hole.
func TestThroughStopDenotedLevelThroughHole(t *testing.T) {
	t.Parallel()
	for _, top := range []units.Value{units.Inches(0.2), units.Inches(0.3)} {
		for _, tc := range []struct {
			name     string
			twoSided bool
			tool     func(plate *decad.Body) decad.Extent
		}{
			{"through all", false, func(*decad.Body) decad.Extent { return decad.ThroughAll{Dir: decad.Along} }},
			{"through all both sides", true, func(*decad.Body) decad.Extent {
				return decad.TwoSided{One: decad.ThroughAllSide{}, Two: decad.ThroughAllSide{}}
			}},
			{"to face", false, func(plate *decad.Body) decad.Extent {
				return decad.ToFace{Body: plate, Face: facing(r3.NewVec(0, 0, 1))}
			}},
			{"to face both sides", true, func(plate *decad.Body) decad.Extent {
				return decad.TwoSided{One: decad.ToFace{Body: plate, Face: facing(r3.NewVec(0, 0, 1))},
					Two: decad.ToFace{Body: plate, Face: facing(r3.NewVec(0, 0, -1))}}
			}},
		} {
			t.Run(top.String()+" "+tc.name, func(t *testing.T) {
				t.Parallel()
				doc := decad.New()
				height := denotedDepth(top)
				var plate *decad.Body
				if tc.twoSided {
					// The plate runs from z=-3 to top, so both stops lie off the
					// sketch plane.
					plate = twoSidedRect(t, doc, 0, 0, 20, 20, top, units.Millimeters(3))
					height = new(big.Rat).Add(height, big.NewRat(3, 1))
				} else {
					plate = upPlate(t, doc, top)
				}
				tool := stopTool(t, doc, tc.tool(plate))
				part, err := decad.Cut(t.Context(), plate, tool)
				require.NoError(t, err)
				requireDenotedThroughHole(t, part, height)
			})
		}
	}
}

// TestThroughStopDenotedLevelMisses keeps a stop that records no denoted
// level declining: a ThroughAll whose farthest far side another body's
// matches within the displacements.
func TestThroughStopDenotedLevelMisses(t *testing.T) {
	t.Parallel()
	top := units.Inches(0.2)
	t.Run("through all with a competing far side", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		plate := twoSidedRect(t, doc, 0, 0, 20, 20, top, units.Millimeters(3))
		// A second live body in the path holds the plate top's float, 5.08
		// mm, and denotes a level above it, so the stop's true level is
		// that body's, not the plate cap's.
		twoSidedRect(t, doc, 9, 9, 11, 11, units.Millimeters(5.08), units.Millimeters(1))
		tool := stopTool(t, doc, decad.TwoSided{One: decad.ThroughAllSide{}, Two: decad.ThroughAllSide{}})
		part, err := decad.Cut(t.Context(), plate, tool)
		// The tool is flush with both faces of the plate, so the shifted retry grows it
		// (agent-docs/gap-tolerance-design.md §5.2 item 5) and the mesh path builds the part.
		requireDeclinedSpan(t, part, err, skinnedPlateVolume(denotedDepth(top), denotedDepth(top), 1))
	})
}

// TestThroughStopDenotedLevelStackedTop cuts a plate with a tool that stops
// at a blind-cut stack's top, which denotes 0.2 in. The stack's top cap
// records its slab's denoted level, so over a 0.2 in plate the stop proves the
// span and the cut is the analytic through hole. A 5.08 mm plate top holds the
// same float but denotes another plane, so no span is proven.
func TestThroughStopDenotedLevelStackedTop(t *testing.T) {
	t.Parallel()
	inch := units.Inches(0.2)
	for _, tc := range []struct {
		top      units.Value
		analytic bool
	}{{inch, true}, {units.Millimeters(5.08), false}} {
		t.Run(tc.top.String(), func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			stopPlate := twoSidedRect(t, doc, 30, 0, 50, 20, inch, units.Millimeters(3))
			// A pocket from below leaves the plate's top the only face facing +z.
			pocket := twoSidedRect(t, doc, 32, 2, 36, 6, units.Millimeters(2), units.Millimeters(10))
			stacked, err := decad.Cut(t.Context(), stopPlate, pocket)
			require.NoError(t, err)
			require.False(t, tookMeshPath(stacked), "the pocket builds a stack")
			plate := twoSidedRect(t, doc, 0, 0, 20, 20, tc.top, units.Millimeters(3))
			tool := stopTool(t, doc, decad.TwoSided{One: decad.ToFace{Body: stacked, Face: facing(r3.NewVec(0, 0, 1))},
				Two: decad.DistanceSide{D: units.Millimeters(20)}})
			part, err := decad.Cut(t.Context(), plate, tool)
			if tc.analytic {
				require.NoError(t, err)
				requireDenotedThroughHole(t, part, new(big.Rat).Add(denotedDepth(inch), big.NewRat(3, 1)))
				return
			}
			requireDeclinedSpan(t, part, err, skinnedPlateVolume(denotedDepth(tc.top), denotedDepth(inch), 1))
		})
	}
}

// TestThroughStopDenotedLevelUnionKeepsDenotation unions staggered plates
// whose tops all denote 0.2 in, then cuts the union with a tool swept to 0.2
// in. The union keeps the shared denoted level, so the cut is the analytic
// through hole. With one 5.08 mm top among them, the union proves no shared
// top.
func TestThroughStopDenotedLevelUnionKeepsDenotation(t *testing.T) {
	t.Parallel()
	inch, mm := units.Inches(0.2), units.Millimeters(5.08)
	three := big.NewRat(3, 1)
	height := func(v units.Value) *big.Rat { return new(big.Rat).Add(denotedDepth(v), three) }
	for _, tc := range []struct {
		name string
		tops []units.Value
		// area is the union's plan area; mmArea marks a set with a 5.08 mm
		// plate.
		area, mmArea int64
	}{
		{"two inch plates", []units.Value{inch, inch}, 572, 0},
		{"three inch plates", []units.Value{inch, inch, inch}, 744, 0},
		{"three plates, one millimetre top", []units.Value{inch, mm, inch}, 344, 400},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			// Plate i covers x 8i..8i+20 and y i..i+20, so every pair overlaps
			// and no two edges share a carrier.
			plates := make([]*decad.Body, len(tc.tops))
			for i, top := range tc.tops {
				o := float64(i)
				plates[i] = twoSidedRect(t, doc, 8*o, o, 8*o+20, o+20, top, units.Millimeters(3))
			}
			joined, err := decad.Union(t.Context(), plates[0], plates[1:]...)
			if tc.mmArea != 0 {
				// No analytic route proves one top, and the mesh path cannot
				// decide tops 7e-17 mm apart.
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.ErrorContains(t, err, "come within the chord tolerance")
				return
			}
			require.NoError(t, err)
			require.False(t, tookMeshPath(joined))
			requireRationalVolumeCovered(t, joined, new(big.Rat).Mul(big.NewRat(tc.area, 1), height(inch)))
			tool := twoSidedRect(t, doc, 2, 8, 6, 12, inch, units.Millimeters(20))
			part, err := decad.Cut(t.Context(), joined, tool)
			require.NoError(t, err)
			require.False(t, tookMeshPath(part), "the union keeps the denoted top")
			requireRationalVolumeCovered(t, part, new(big.Rat).Mul(big.NewRat(tc.area-16, 1), height(inch)))
		})
	}
}

// symmetricTool extrudes the rectangle x0..x1 x y0..y1 on XY by a full-length
// Symmetric 0.4 in: over -0.2 in..0.2 in, each end charged the whole 0.4 in
// conversion bound, which is wider than the rounding of 0.2 in itself.
func symmetricTool(t *testing.T, doc *decad.Document, x0, y0, x1, y1 float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Symmetric{D: units.Inches(0.4), FullLength: true})
	require.NoError(t, err)
	return tool
}

// TestThroughStopDenotedLevelGroupToolCut cuts a plate over -3 mm..0.2 in with
// a prism group of two 4x4 tools over -0.2 in..0.2 in. The tools' displacement
// is wider than the gap between 0.2 in and its float, so only the denoted top
// the group keeps from its operands proves the span; the cut builds the
// analytic plate with two through holes.
func TestThroughStopDenotedLevelGroupToolCut(t *testing.T) {
	t.Parallel()
	inch := units.Inches(0.2)
	doc := decad.New()
	plate := twoSidedRect(t, doc, 0, 0, 20, 20, inch, units.Millimeters(3))
	left := symmetricTool(t, doc, 2, 8, 6, 12)
	right := symmetricTool(t, doc, 14, 8, 18, 12)
	group, err := decad.Union(t.Context(), left, right)
	require.NoError(t, err)
	require.False(t, tookMeshPath(group))
	require.Len(t, group.Lumps(), 2)
	part, err := decad.Cut(t.Context(), plate, group)
	require.NoError(t, err)
	require.False(t, tookMeshPath(part), "the group keeps the denoted top, so the span is proven")
	require.Len(t, part.Lumps(), 1)
	// Four plate walls, eight hole walls, two caps.
	require.Len(t, part.Faces(), 14)
	requireRationalVolumeCovered(t, part, new(big.Rat).Mul(big.NewRat(400-32, 1),
		new(big.Rat).Add(denotedDepth(inch), big.NewRat(3, 1))))
}
