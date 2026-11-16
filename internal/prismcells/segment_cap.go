package prismcells

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// RegionsWithinSegmentCap counts the densified segments over every region a
// private scene will hold, and reports whether the count stays within limit.
// A prism group's lumps are separate regions. RegionsWithinWorkCap is this
// count under MaxArrangementSegments.
func RegionsWithinSegmentCap(budget *proofbound.WorkBudget, limit int, profiles ...momentinput.Profile) (int, bool, error) {
	segments := 0
	for _, profile := range profiles {
		for _, loop := range append([]momentinput.LoopRecord{profile.Outer}, profile.Holes...) {
			for _, seg := range loop.Segments {
				if err := budget.Step(); err != nil {
					return segments, false, err
				}
				switch seg.(type) {
				case momentinput.LineSeg:
					segments++
				case momentinput.CircleSeg, momentinput.ArcSeg:
					segments += 256
				default:
					return segments, false, nil // G4 already excluded this case.
				}
				if segments > limit {
					return segments, false, nil
				}
			}
		}
	}
	return segments, true, nil
}
