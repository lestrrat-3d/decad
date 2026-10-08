package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestTangentJoinedOffsetsBuildBothFeatures covers the non-dyadic setbacks
// whose tangent line-arc carrier intersections formerly missed their double
// root or selected a point past the arc's recorded window.
func TestTangentJoinedOffsetsBuildBothFeatures(t *testing.T) {
	t.Parallel()
	const length, width, height, radius = 96.0, 68.0, 16.0, 12.0
	baseArea := length*width - (4-math.Pi)*radius*radius
	perimeter := 2*(length+width) - 2*(4-math.Pi)*radius
	for _, setback := range []float64{0.0003, 0.001, 0.505, 0.51, 0.55, 1.005, 11.9} {
		t.Run(units.Millimeters(setback).String(), func(t *testing.T) {
			t.Parallel()
			roundedBox := func() *decad.Body {
				w := sketch.NewWorld()
				s, err := w.CreateSketch(w.XY())
				require.NoError(t, err)
				rect := s.CreateRectangle(-length/2, -width/2, length/2, width/2)
				s.Fix(rect.A)
				_, err = s.Solve(t.Context())
				require.NoError(t, err)
				box, err := decad.New().Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
				require.NoError(t, err)
				rounded, err := box.Fillet(t.Context(), decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(radius))
				require.NoError(t, err)
				return rounded
			}

			rounded := roundedBox()
			chamfered, err := rounded.Chamfer(t.Context(), capLoopEdges(rounded), units.Millimeters(setback))
			require.NoError(t, err)
			requireManifold(t, chamfered)
			chamferVolume, err := chamfered.Volume()
			require.NoError(t, err)
			wantChamfer := baseArea*height - perimeter*setback*setback/2 + math.Pi*setback*setback*setback/3
			require.InDelta(t, wantChamfer, chamferVolume.Value.Mag(), 1e-6)

			rounded = roundedBox()
			shelled, err := rounded.Shell(t.Context(), topCap(rounded), units.Millimeters(setback))
			require.NoError(t, err)
			requireManifold(t, shelled)
			shellVolume, err := shelled.Volume()
			require.NoError(t, err)
			innerArea := baseArea - perimeter*setback + math.Pi*setback*setback
			require.InDelta(t, baseArea*height-innerArea*(height-setback), shellVolume.Value.Mag(), 1e-6)
		})
	}
}

func TestTangentJoinedSlotChamferAtOffsetOrigin(t *testing.T) {
	t.Parallel()
	const centerSpan, radius, height = 44.0, 12.0, 16.0
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	slot, err := s.CreateSlot(36, -22, 36, 22, radius)
	require.NoError(t, err)
	s.Fix(slot.C1)
	s.Fix(slot.C2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.New().Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)

	for _, setback := range []float64{0.505, 1.005} {
		chamfered, err := body.Chamfer(t.Context(), capLoopEdges(body), units.Millimeters(setback))
		require.NoError(t, err)
		requireManifold(t, chamfered)
		volume, err := chamfered.Volume()
		require.NoError(t, err)
		area := 2*radius*centerSpan + math.Pi*radius*radius
		perimeter := 2*centerSpan + 2*math.Pi*radius
		want := area*height - perimeter*setback*setback/2 + math.Pi*setback*setback*setback/3
		require.InDelta(t, want, volume.Value.Mag(), 1e-6)
		// A feature consumes its receiver. Rebuild from the still-live sketch
		// before the next setback.
		if setback != 1.005 {
			body, err = decad.New().Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
			require.NoError(t, err)
		}
	}
}

// TestTangentJoinedRotatedSlotChamfer pins a cap-loop chamfer on a slot whose
// axis runs from (0, 0) to (24, 7), so every arc starts and ends off its
// centre's axes. Each arc's record states Start and Center only, so its walk
// holds the math.Hypot of Start − Center under a nonzero RadiusBound. All four
// line-arc corners are tangent, so each is a G1 join, whose corner foot moves
// along the shared normal and never reads the miter locus. The volume is the
// closed form of a smooth convex prism chamfered on one cap:
// A·h − P·d²/2 + π·d³/3, with A = 2rL + πr² and P = 2L + 2πr for L = 25.
//
// Shown to fail: with capBlendCornerLocusGap reading a G1 join's ruling as a
// miter locus, Tessellate refuses at both setbacks, since the enclosure of a
// tangent corner's discriminant reaches zero.
func TestTangentJoinedRotatedSlotChamfer(t *testing.T) {
	t.Parallel()
	const span, radius, height = 25.0, 12.0, 16.0
	for _, setback := range []float64{0.505, 1.005} {
		t.Run(units.Millimeters(setback).String(), func(t *testing.T) {
			t.Parallel()
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			slot, err := s.CreateSlot(0, 0, 24, 7, radius)
			require.NoError(t, err)
			s.Fix(slot.C1)
			s.Fix(slot.C2)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			require.Len(t, s.Profiles(), 1)
			body, err := decad.New().Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
			require.NoError(t, err)
			for _, v := range body.Vertices() {
				p := v.Position().Value
				for _, c := range [][2]float64{{0, 0}, {24, 7}} {
					require.NotZero(t, p.X-c[0], `every corner must sit off both centres' axes`)
					require.NotZero(t, p.Y-c[1], `every corner must sit off both centres' axes`)
				}
			}

			chamfered, err := body.Chamfer(t.Context(), capLoopEdges(body), units.Millimeters(setback))
			require.NoError(t, err)
			requireManifold(t, chamfered)
			volume, err := chamfered.Volume()
			require.NoError(t, err)
			area := 2*radius*span + math.Pi*radius*radius
			perimeter := 2*span + 2*math.Pi*radius
			want := area*height - perimeter*setback*setback/2 + math.Pi*setback*setback*setback/3
			require.InDelta(t, want, volume.Value.Mag(), 1e-6)
			_, err = chamfered.Tessellate(t.Context(), units.Millimeters(0.5))
			require.NoError(t, err)
		})
	}
}
