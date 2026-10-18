package spiralsector

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// StationSide uses the source station departure to enclose each complete
// nonfitted edge. A crossing or an undecidable cell refuses; no sampled sign
// grants admission.
func StationSide(ctx context.Context, trim *loftFacesTrimRecord,
	m, b *big.Rat, segment int) int {
	source := trim.Source
	n := len(source.SourceStation)
	if n == 0 || len(trim.StationGroup) != n {
		return 0
	}
	inside, outside, seen := true, true, false
	for band := 0; band+1 < len(trim.Maps); band++ {
		lo, hi := trim.Bounds[band], trim.Bounds[band+1]
		if len(lo.SourceDeparture) != n || len(hi.SourceDeparture) != n ||
			len(lo.ProjectiveDeparture) != n || len(hi.ProjectiveDeparture) != n {
			return 0
		}
		for j, station := range source.SourceStation {
			if trim.StationGroup[j] != segment {
				continue
			}
			if j%64 == 0 && ctx.Err() != nil {
				return 0
			}
			seen = true
			departure := max(
				proofbound.AbsSumUpper(lo.SourceDeparture[j], lo.ProjectiveDeparture[j]),
				proofbound.AbsSumUpper(hi.SourceDeparture[j], hi.ProjectiveDeparture[j]))
			if proofbound.IsNonFinite(departure) || departure < 0 {
				return 0
			}
			next := source.SourceStation[(j+1)%n]
			if inside && !SpiralCellSide(trim.Maps[band], trim.Maps[band+1], station,
				next, seamRatF(departure), m, b, false, seamRatZero(), seamRatOne(), 8) {
				inside = false
			}
			if outside && !SpiralCellSide(trim.Maps[band], trim.Maps[band+1], station,
				next, seamRatF(departure), m, b, true, seamRatZero(), seamRatOne(), 8) {
				outside = false
			}
			if !inside && !outside {
				return 0
			}
		}
	}
	if !seen || inside == outside {
		return 0
	}
	if inside {
		return -1
	}
	return 1
}
