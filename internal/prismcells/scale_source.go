package prismcells

import "github.com/lestrrat-3d/decad/internal/momentinput"

// ScaleSourceProfile admits only complete source edges. A partial Sketch edge
// needs its own source certificate even if its visible curve is circular.
func ScaleSourceProfile(profile momentinput.Profile) (curved, eligible bool) {
	loops := append([]LoopRecord{profile.Outer}, profile.Holes...)
	for _, loop := range loops {
		if len(loop.Segments) == 0 {
			return false, false
		}
		for _, segment := range loop.Segments {
			var start, end float64
			switch s := segment.(type) {
			case LineSeg:
				start, end = s.TStart, s.TEnd
			case CircleSeg:
				if len(loop.Segments) != 1 {
					return false, false
				}
				start, end, curved = s.TStart, s.TEnd, true
			case ArcSeg:
				start, end, curved = s.TStart, s.TEnd, true
			default:
				return false, false
			}
			if !((start == 0 && end == 1) || (start == 1 && end == 0)) {
				return false, false
			}
		}
	}
	return curved, true
}
