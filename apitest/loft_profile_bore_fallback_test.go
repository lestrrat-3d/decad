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

func TestLoftLineAndArcProfileThroughBorePreservesExactSections(t *testing.T) {
	for _, tc := range []struct {
		name      string
		area      float64
		pts       [][2]float64
		draw      func(*sketch.Sketch)
		edgeCount int
	}{
		{name: "square", area: 4, pts: [][2]float64{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}, edgeCount: 4},
		{name: "hexagon", area: 6 * math.Sqrt(3), pts: [][2]float64{
			{2, 0}, {1, math.Sqrt(3)}, {-1, math.Sqrt(3)},
			{-2, 0}, {-1, -math.Sqrt(3)}, {1, -math.Sqrt(3)},
		}, edgeCount: 6},
		{name: "keyway", area: 17, pts: [][2]float64{
			{-2, -2}, {2, -2}, {2, -0.5}, {3, -0.5},
			{3, 0.5}, {2, 0.5}, {2, 2}, {-2, 2},
		}, edgeCount: 8},
		{
			name: "round bore with keyway", area: 10*math.Pi/3 + 6 - math.Sqrt(3), edgeCount: 4,
			draw: func(s *sketch.Sketch) {
				root3 := math.Sqrt(3)
				center := s.CreatePoint(0, 0)
				top := s.CreatePoint(root3, 1)
				bottom := s.CreatePoint(root3, -1)
				slotBottom := s.CreatePoint(3, -1)
				slotTop := s.CreatePoint(3, 1)
				for _, p := range []*sketch.Point{center, top, bottom, slotBottom, slotTop} {
					s.Fix(p)
				}
				s.CreateArc(center, top, bottom)
				s.CreateLine(bottom, slotBottom)
				s.CreateLine(slotBottom, slotTop)
				s.CreateLine(slotTop, top)
			},
		},
		{
			name: "rounded slot", area: 6 + math.Pi, edgeCount: 4,
			draw: func(s *sketch.Sketch) {
				left := s.CreatePoint(-1.5, 0)
				right := s.CreatePoint(1.5, 0)
				bottomLeft := s.CreatePoint(-1.5, -1)
				topLeft := s.CreatePoint(-1.5, 1)
				bottomRight := s.CreatePoint(1.5, -1)
				topRight := s.CreatePoint(1.5, 1)
				for _, p := range []*sketch.Point{left, right, bottomLeft, topLeft, bottomRight, topRight} {
					s.Fix(p)
				}
				s.CreateLine(bottomLeft, bottomRight)
				s.CreateArc(right, bottomRight, topRight)
				s.CreateLine(topRight, topLeft)
				s.CreateArc(left, topLeft, bottomLeft)
			},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s0, p0, s1, p1 := loftSquaresAt(t, r3.NewVec(0, 0, 0), 5, 5, 2)
			doc := decad.New(decad.ExactOnly())
			blank, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1)
			require.NoError(t, err)

			world := sketch.NewWorld()
			frame, err := r3.NewFrame(r3.NewVec(0, 0, -1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
			require.NoError(t, err)
			plane, err := world.CreatePlaneFromFrame(frame)
			require.NoError(t, err)
			toolSketch, err := world.CreateSketch(plane)
			require.NoError(t, err)
			if tc.draw != nil {
				tc.draw(toolSketch)
			} else {
				points := make([]*sketch.Point, len(tc.pts))
				for i, point := range tc.pts {
					points[i] = toolSketch.CreatePoint(point[0], point[1])
				}
				toolSketch.Fix(points[0])
				for i := range points {
					toolSketch.CreateLine(points[i], points[(i+1)%len(points)])
				}
			}
			_, err = toolSketch.Solve(t.Context())
			require.NoError(t, err)
			require.Len(t, toolSketch.Profiles(), 1)
			tool, err := decad.Extrude(t.Context(), doc, toolSketch, toolSketch.Profiles()[0],
				decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
			require.NoError(t, err)

			cut, err := decad.Cut(t.Context(), blank, tool)
			require.NoError(t, err)
			require.True(t, cut.IsSolid())
			volume, err := cut.Volume()
			require.NoError(t, err)
			expected := 200 - 2*tc.area
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-expected), volume.Bound.Base())
			sectionFrame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
			require.NoError(t, err)
			section, sections, sectionErr := cut.Section(t.Context(), sectionFrame)
			require.NoError(t, sectionErr)
			require.Len(t, sections, 1)
			require.True(t, sections[0].Valid)
			require.Len(t, sections[0].Holes, 1)
			require.Len(t, sections[0].Holes[0], tc.edgeCount)
			sectionProfile, err := decad.MeasureProfile(section, sections[0])
			require.NoError(t, err)
			area, err := sectionProfile.Area()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(area.Value.Base()-(100-tc.area)), area.Bound.Base()+1e-12)
		})
	}
}
