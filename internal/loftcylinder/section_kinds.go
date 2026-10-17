package loftcylinder

import (
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// SectionKinds checks the recorded section kinds and connector endpoints of a
// loft-cylinder tooth. Callers first check that both profiles have six segments.
func SectionKinds(lp loftrecord.Payload) bool {
	for _, profile := range []momentinput.Profile{lp.Profile0, lp.Profile1} {
		segments := profile.Outer.Segments
		start, ok := segments[0].(sectionrecord.LineSeg)
		if !ok {
			return false
		}
		if _, ok := segments[1].(sectionrecord.FitSplineSeg); !ok {
			return false
		}
		if _, ok := segments[2].(sectionrecord.ArcSeg); !ok {
			return false
		}
		if _, ok := segments[3].(sectionrecord.FitSplineSeg); !ok {
			return false
		}
		end, ok := segments[4].(sectionrecord.LineSeg)
		if !ok {
			return false
		}
		arc, ok := segments[5].(sectionrecord.ArcSeg)
		if !ok || arc.Center != (sectionrecord.Point2{}) || arc.TStart != 1 || arc.TEnd != 0 {
			return false
		}
		if start.TStart != 0 || start.TEnd != 1 || end.TStart != 0 || end.TEnd != 1 ||
			start.Start != arc.Start || end.End != arc.End {
			return false
		}
	}
	return true
}
