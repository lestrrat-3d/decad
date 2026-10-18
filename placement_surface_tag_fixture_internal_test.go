package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// curvedLoftFlankSections builds the three fitted-flank sections of a smooth
// LoftSections body whose two circular-arc sides carry Cylinder tags.
func curvedLoftFlankSections(t *testing.T, mids, tips, heights [3]float64) [3]LoftSection {
	t.Helper()
	w := sketch.NewWorld()
	var sections [3]LoftSection
	for i, z := range heights {
		plane := w.XY()
		if i != 0 {
			var err error
			plane, err = w.CreateOffsetPlane(w.XY(), z)
			require.NoError(t, err)
		}
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		point := func(r, angle float64) *sketch.Point {
			p := s.CreatePoint(r*math.Cos(angle), r*math.Sin(angle))
			s.Fix(p)
			return p
		}
		origin := s.CreatePoint(0, 0)
		s.Fix(origin)
		r0, r1, r2 := point(4, -0.3), point(6, mids[i]), point(tips[i], -0.2)
		l0, l1, l2 := point(4, 0.3), point(6, 0.24), point(tips[i], 0.2)
		_, err = s.CreateFitSpline(r0, r1, r2)
		require.NoError(t, err)
		s.CreateArc(origin, r2, l2)
		_, err = s.CreateFitSpline(l2, l1, l0)
		require.NoError(t, err)
		s.CreateArc(origin, l0, r0)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			if !profile.Valid || len(profile.Outer) != 4 || len(profile.Holes) != 0 {
				continue
			}
			fits, arcs := 0, 0
			for _, edge := range profile.Outer {
				switch edge.Entity.(type) {
				case *sketch.FitSpline:
					fits++
				case *sketch.Arc:
					arcs++
				}
			}
			if fits == 2 && arcs == 2 {
				sections[i] = LoftSection{Sketch: s, Profile: profile}
				break
			}
		}
		require.NotNil(t, sections[i].Profile)
	}
	return sections
}

// curvedLoftBody lofts the flank sections into a smooth faceted body.
func curvedLoftBody(t *testing.T) *Body {
	t.Helper()
	sections := curvedLoftFlankSections(t, [3]float64{-0.24, -0.23, -0.24}, [3]float64{8, 8, 8}, [3]float64{0, 5, 10})
	body, err := LoftSections(t.Context(), New(), sections[:]...)
	require.NoError(t, err)
	return body
}
