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

// rectOnPlane extrudes the rectangle x0..x1 x y0..y1 on the plane z=at by e.
func rectOnPlane(t *testing.T, doc *decad.Document, at, x0, y0, x1, y1 float64, e decad.Extent) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), at)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], e)
	require.NoError(t, err)
	return body
}

// requireNoAnalyticClaim requires a Boolean that proves nothing analytic:
// either the mesh path's result, its volume bound covering want, or a
// refusal with the given message.
func requireNoAnalyticClaim(t *testing.T, got *decad.Body, err error, want *big.Rat, refusal string) {
	t.Helper()
	if refusal != "" {
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, refusal)
		return
	}
	require.NoError(t, err)
	require.True(t, tookMeshPath(got), "a level within its displacement of another proves no analytic shape")
	requireRationalVolumeCovered(t, got, want)
}

// TestBlindLevelRangeOuterEnd cuts a 20x20 plate with a 4x4 tool whose outer
// end holds the float of the plate's cap. Where the tool's end denotes a level
// inside the plate, a skin about 7e-17 mm thick closes the pocket into a void,
// so no open pocket is proven. Where both denote 0.2 in, the pocket opens at
// the cap and builds analytically.
func TestBlindLevelRangeOuterEnd(t *testing.T) {
	t.Parallel()
	mm, inch, one, three := units.Millimeters(5.08), units.Inches(0.2), units.Millimeters(1), units.Millimeters(3)
	r := func(v units.Value) *big.Rat { return denotedDepth(v) }
	for _, tc := range []struct {
		name                         string
		plateUp, plateDown           units.Value
		toolUp, toolDown             units.Value
		analytic                     bool
		plateHeight, toolHeightInPlt *big.Rat
	}{
		{"top skin", mm, three, inch, one, false,
			new(big.Rat).Add(r(mm), r(three)), new(big.Rat).Add(r(inch), r(one))},
		{"bottom skin", three, mm, one, inch, false,
			new(big.Rat).Add(r(mm), r(three)), new(big.Rat).Add(r(inch), r(one))},
		{"both tops 0.2 in", inch, three, inch, one, true,
			new(big.Rat).Add(r(inch), r(three)), new(big.Rat).Add(r(inch), r(one))},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := twoSidedRect(t, doc, 0, 0, 20, 20, tc.plateUp, tc.plateDown)
			tool := twoSidedRect(t, doc, 8, 8, 12, 12, tc.toolUp, tc.toolDown)
			part, err := decad.Cut(t.Context(), plate, tool)
			want := new(big.Rat).Mul(big.NewRat(400, 1), tc.plateHeight)
			want.Sub(want, new(big.Rat).Mul(big.NewRat(16, 1), tc.toolHeightInPlt))
			if !tc.analytic {
				requireNoAnalyticClaim(t, part, err, want, "")
				return
			}
			require.NoError(t, err)
			require.False(t, tookMeshPath(part))
			require.Len(t, part.Lumps(), 1)
			require.Len(t, part.Faces(), 11)
			requireRationalVolumeCovered(t, part, want)
		})
	}
}

// TestBlindLevelRangeStackedOuterEnd pockets a plate from its top to z=-1,
// then cuts a second 4x4 pocket to z=-1 whose top holds the plate top's
// float. Over a 5.08 mm top the second tool's 0.2 in top denotes a level
// below it, so no open pocket is proven; over a 0.2 in top both denote one
// level and the second pocket reuses the first one's floor.
func TestBlindLevelRangeStackedOuterEnd(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name     string
		top      units.Value
		analytic bool
	}{
		{"millimetre plate top", units.Millimeters(5.08), false},
		{"inch plate top", units.Inches(0.2), true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := twoSidedRect(t, doc, 0, 0, 20, 20, tc.top, units.Millimeters(3))
			first := twoSidedRect(t, doc, 2, 2, 6, 6, units.Millimeters(10), units.Millimeters(1))
			stacked, err := decad.Cut(t.Context(), plate, first)
			require.NoError(t, err)
			require.False(t, tookMeshPath(stacked), "the first pocket builds a stack")
			second := twoSidedRect(t, doc, 12, 12, 16, 16, units.Inches(0.2), units.Millimeters(1))
			part, err := decad.Cut(t.Context(), stacked, second)
			// Each pocket removes 16 mm² from z=-1 up to its own top.
			top := new(big.Rat).Add(denotedDepth(tc.top), big.NewRat(1, 1))
			want := new(big.Rat).Mul(big.NewRat(400, 1), new(big.Rat).Add(denotedDepth(tc.top), big.NewRat(3, 1)))
			want.Sub(want, new(big.Rat).Mul(big.NewRat(16, 1), top))
			want.Sub(want, new(big.Rat).Mul(big.NewRat(16, 1), new(big.Rat).Add(denotedDepth(units.Inches(0.2)), big.NewRat(1, 1))))
			if !tc.analytic {
				requireNoAnalyticClaim(t, part, err, want, "")
				return
			}
			require.NoError(t, err)
			require.False(t, tookMeshPath(part), "the second pocket ends at the first one's floor")
			require.Len(t, part.Lumps(), 1)
			requireRationalVolumeCovered(t, part, want)
		})
	}
}

// TestBlindLevelRangeStackedUnion unites a 20x20 plate with a 10x10 block
// drawn on the plane z=5.08 mm and swept up 10 mm. Over a 0.2 in plate top the
// block starts about 7e-17 mm above the plate, so the true result is two
// lumps and no touching stack is proven. Over a 5.08 mm top the two touch
// exactly and build one analytic stack.
func TestBlindLevelRangeStackedUnion(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name     string
		top      units.Value
		analytic bool
	}{
		{"inch plate top", units.Inches(0.2), false},
		{"millimetre plate top", units.Millimeters(5.08), true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := twoSidedRect(t, doc, 0, 0, 20, 20, tc.top, units.Millimeters(3))
			block := rectOnPlane(t, doc, 5.08, 5, 5, 15, 15, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			got, err := decad.Union(t.Context(), plate, block)
			want := new(big.Rat).Mul(big.NewRat(400, 1), new(big.Rat).Add(denotedDepth(tc.top), big.NewRat(3, 1)))
			want.Add(want, big.NewRat(100*10, 1))
			if !tc.analytic {
				requireNoAnalyticClaim(t, got, err, want, "")
				return
			}
			require.NoError(t, err)
			require.False(t, tookMeshPath(got))
			require.Len(t, got.Lumps(), 1)
			require.Len(t, got.Faces(), 11)
			requireRationalVolumeCovered(t, got, want)
		})
	}
}

// TestBlindLevelRangeToFaceOffset stops a tool at a plate's 0.2 in cap
// overshot by a 3 mm Offset, and cuts a target swept 0.2 in from the plane 3
// mm past that cap. The stop denotes the cap's level plus the exact offset,
// which is the target's far cap, so the cut is the analytic through hole: up
// from z=0 to 3 mm + 0.2 in, and down from z=0 to -(3 mm + 0.2 in).
func TestBlindLevelRangeToFaceOffset(t *testing.T) {
	t.Parallel()
	inch := units.Inches(0.2)
	for _, tc := range []struct {
		name string
		dir  decad.Direction
		at   float64
		n    r3.Vec
	}{
		{"up", decad.Along, 3, r3.NewVec(0, 0, 1)},
		{"down", decad.Against, -3, r3.NewVec(0, 0, -1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			stop := rectOnPlane(t, doc, 0, 30, 0, 50, 20, decad.Distance{D: inch, Dir: tc.dir})
			target := rectOnPlane(t, doc, tc.at, 0, 0, 20, 20, decad.Distance{D: inch, Dir: tc.dir})
			tool := stopTool(t, doc, decad.ToFace{Body: stop, Face: facing(tc.n), Offset: units.Millimeters(3)})
			part, err := decad.Cut(t.Context(), target, tool)
			require.NoError(t, err)
			requireDenotedThroughHole(t, part, denotedDepth(inch))
		})
	}
}
