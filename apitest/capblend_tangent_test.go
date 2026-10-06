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
