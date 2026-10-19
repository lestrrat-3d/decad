package loftcylinder_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestBlankRadius reads the radius of an unplaced centered CCW circle blank
// and refuses each blank that breaks one admission condition.
func TestBlankRadius(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err, "XY frame must build")
	blank := func(center sectionrecord.Point2, ccw bool, z1 float64) prismrecord.Payload {
		circle := sectionrecord.CircleSeg{Center: center, Radius: units.Millimeters(3), CCW: ccw}
		return prismrecord.Payload{
			Profile: momentinput.Profile{Outer: momentinput.LoopRecord{
				Segments: []sectionrecord.CurveSegment{circle}}},
			Frame: frame, Z0: 0, Z1: z1, Xform: r3.Identity(),
		}
	}
	radius, ok := loftcylinder.BlankRadius(blank(sectionrecord.Point2{}, true, 2))
	require.True(t, ok, "a centered CCW circle blank must be admitted")
	require.Equal(t, 3.0, radius, "the blank radius must be the circle's radius")

	holed := blank(sectionrecord.Point2{}, true, 2)
	holed.Profile.Holes = []momentinput.LoopRecord{holed.Profile.Outer}
	for _, tc := range []struct {
		name string
		pp   prismrecord.Payload
	}{
		{"off center", blank(sectionrecord.Point2{U: 0.5}, true, 2)},
		{"clockwise", blank(sectionrecord.Point2{}, false, 2)},
		{"empty height", blank(sectionrecord.Point2{}, true, 0)},
		{"holed", holed},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, ok := loftcylinder.BlankRadius(tc.pp)
			require.False(t, ok, "the blank must be refused")
		})
	}
}

// TestToothRoot finds the first station of the cap's final root-arc run and
// refuses a run that splits, starts too early or holds one station.
func TestToothRoot(t *testing.T) {
	t.Parallel()
	tooth := func(segment []int) loftrecord.Payload {
		n := len(segment)
		return loftrecord.Payload{
			Xform: r3.Identity(), Verts: make([]r3.Vec, 2*n),
			Tris: make([][3]int, 4*n-4), Walls: 2 * n, Segment: [][]int{segment},
		}
	}
	root, ok := loftcylinder.ToothRoot(tooth([]int{0, 1, 2, 5, 5, 5}))
	require.True(t, ok, "one final root run must be admitted")
	require.Equal(t, 3, root, "the root must start at the run's first station")

	for _, tc := range []struct {
		name    string
		segment []int
	}{
		{"split run", []int{0, 1, 5, 2, 5, 5}},
		{"station after the root", []int{0, 1, 2, 5, 5, 4}},
		{"root too early", []int{0, 5, 5, 5, 5, 5}},
		{"single root station", []int{0, 1, 2, 3, 4, 5}},
		{"no root", []int{0, 1, 2, 3, 4, 4}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, ok := loftcylinder.ToothRoot(tooth(tc.segment))
			require.False(t, ok, "the held layout must be refused")
		})
	}
}

// separateTooth is an untwisted tooth centered at angle center: a sector of
// half-width 0.2 on both caps, held vertices 0.05 rad either side of center.
func separateTooth(operand int, center float64) loftcylinder.Tooth {
	var verts []r3.Vec
	for _, radius := range []float64{3, 4} {
		for _, angle := range []float64{center - 0.05, center + 0.05} {
			verts = append(verts, r3.Vec{X: radius * math.Cos(angle), Y: radius * math.Sin(angle)})
		}
	}
	sector := [2]float64{center - 0.2, center + 0.2}
	return loftcylinder.Tooth{
		Operand: operand, Loft: loftrecord.Payload{Verts: verts},
		MeshBound: 0.01, Bottom: sector, Top: sector,
	}
}

// TestSeparate orders teeth by their bottom sector and proves them apart
// with a fixed through-axis plane and a neighbouring cap-sector gap.
func TestSeparate(t *testing.T) {
	t.Parallel()
	t.Run("opposite teeth are ordered", func(t *testing.T) {
		t.Parallel()
		teeth := []loftcylinder.Tooth{separateTooth(0, math.Pi), separateTooth(1, 0)}
		require.True(t, loftcylinder.Separate(teeth), "opposite teeth must be separated")
		require.Equal(t, []int{1, 0}, []int{teeth[0].Operand, teeth[1].Operand},
			"teeth must be ordered by bottom sector start")
	})
	t.Run("neighbouring sectors keep their gap", func(t *testing.T) {
		t.Parallel()
		teeth := []loftcylinder.Tooth{separateTooth(0, 0), separateTooth(1, 0.45)}
		require.True(t, loftcylinder.Separate(teeth), "a 0.05 rad sector gap must be admitted")
	})
	t.Run("neighbouring sectors too close", func(t *testing.T) {
		t.Parallel()
		teeth := []loftcylinder.Tooth{separateTooth(0, 0), separateTooth(1, 0.41)}
		require.False(t, loftcylinder.Separate(teeth), "a 0.01 rad sector gap must be refused")
	})
	t.Run("no teeth", func(t *testing.T) {
		t.Parallel()
		require.False(t, loftcylinder.Separate(nil), "an empty tooth list must be refused")
	})
}
