package revolveloftfaces

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/sketch"
)

// CapsAvoidToolRootCircles checks that the first cap lies wholly before its
// tool/root circle and the last cap's plane misses the full heel circle.
func CapsAvoidToolRootCircles(trim *loftFacesTrimRecord,
	m, b, toeX, heelX *big.Rat) bool {
	box, err := LfSourceBox(trim.Source)
	if err != nil || len(trim.Maps) < 2 {
		return false
	}
	first, last := trim.Maps[0], trim.Maps[len(trim.Maps)-1]
	if loftface.PositiveDenominator(first, box) != nil || loftface.PositiveDenominator(last, box) != nil {
		return false
	}
	corners := loftface.BoxCorners(box)
	xAxis := loftface.RatVec{big.NewRat(1, 1), new(big.Rat), new(big.Rat)}
	return loftface.ScalarRange(first, xAxis, corners).Hi.Cmp(toeX) < 0 &&
		loftface.CapPlaneAvoidsRootCircle(last, corners, m, b, heelX)
}

// SegmentKind checks that a live Sketch line or arc matches its source record.
func SegmentKind(trim *loftFacesTrimRecord, index int) bool {
	segment := trim.Source.Profile.Outer.Segments[index]
	entity := trim.Source.Source.Entities[index]
	switch entity.(type) {
	case *sketch.Line:
		_, ok := segment.(lineSeg)
		return ok
	case *sketch.Arc:
		_, ok := segment.(arcSeg)
		return ok
	default:
		return false
	}
}
