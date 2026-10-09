package prismcells

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// MaxArrangementSegments bounds the private sketch arrangement before
// Profiles starts. The pinned arranger densifies each line to one tiny
// segment and each admitted circle or arc to no more than 256. It compares
// every tiny-segment pair in one quadratic pass. Profiles has no context, so
// this cap bounds the longest stretch a cancelled caller must wait through.
// The pass costs about 8.3e-5 ms per segment squared, so this cap bounds
// one arrangement at roughly 1.4 seconds and peak memory at twice the cap
// under 16 MB. It admits a rectangular plate with fourteen circular holes
// and one circular tool (docs/prism-boolean-design.md §10).
const MaxArrangementSegments = 4096

// RegionsWithinWorkCap counts the densified segments over every region a
// private scene will hold. A prism group's lumps are separate regions.
func RegionsWithinWorkCap(budget *proofbound.WorkBudget, profiles ...momentinput.Profile) (int, bool, error) {
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
				if segments > MaxArrangementSegments {
					return segments, false, nil
				}
			}
		}
	}
	return segments, true, nil
}
