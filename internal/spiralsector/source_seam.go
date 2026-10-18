package spiralsector

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

type loftFaceMap = loftface.Map
type loftFaceInterval = loftface.Interval
type loftFaceRatVec = loftface.RatVec
type loftFacesTrimRecord = facetedrecord.LoftFacesTrimRecord
type spiralFirstJoinSource = facetedrecord.SpiralFirstJoinSource
type spiralHeelRootSeam = facetedrecord.SpiralHeelRootSeam
type arcSeg = sectionrecord.ArcSeg
type planePoint = sectionrecord.Point2

// SpiralTangentRecordCurrent reports whether the cone-tangent record of trim
// is present on both the trim and its base cap record, equal on both by
// exact value, built for exactly the heel cone's apex and slope, and
// reproduces the recorded final cap map. Every comparison is an exact Cmp of
// immutable values, never a pointer or a flag. It is false when either
// record is missing, so the tangent branches run only for a matching record;
// a trim with no tangent record on either side is not tangent, and the
// strict branches decide it.
func SpiralTangentRecordCurrent(trim *loftFacesTrimRecord) bool {
	if trim == nil || trim.Tangent == nil || trim.Heel == nil || len(trim.Maps) == 0 ||
		trim.Base.LoftFacesCap == nil || trim.Base.LoftFacesCap.Tangent == nil {
		return false
	}
	end := trim.Tangent
	return end.Equal(*trim.Base.LoftFacesCap.Tangent) &&
		end.MatchesCone(trim.Heel.Apex, trim.Heel.Slope) &&
		loftface.MapEqual(end.Map(), trim.Maps[len(trim.Maps)-1])
}

// SpiralCellSide proves one mapped source cell stays on the requested side.
func SpiralCellSide(first, second loftFaceMap, a, z planePoint, departure, m, b *big.Rat,
	outside bool, tLo, tHi *big.Rat, depth int) bool {
	firstA := first.At(seamRatF(a.U), seamRatF(a.V))
	firstZ := first.At(seamRatF(z.U), seamRatF(z.V))
	secondA := second.At(seamRatF(a.U), seamRatF(a.V))
	secondZ := second.At(seamRatF(z.U), seamRatF(z.V))
	blend := func(x, y loftFaceRatVec, t *big.Rat) loftFaceRatVec {
		return loftface.VAdd(loftface.VScale(x, seamRatSub(seamRatOne(), t)), loftface.VScale(y, t))
	}
	corners := [4]loftFaceRatVec{
		blend(firstA, secondA, tLo), blend(firstZ, secondZ, tLo),
		blend(firstA, secondA, tHi), blend(firstZ, secondZ, tHi),
	}
	var box [3]loftFaceInterval
	for axis := range box {
		minimum, maximum := corners[0][axis], corners[0][axis]
		for _, corner := range corners[1:] {
			if corner[axis].Cmp(minimum) < 0 {
				minimum = corner[axis]
			}
			if corner[axis].Cmp(maximum) > 0 {
				maximum = corner[axis]
			}
		}
		box[axis] = loftFaceInterval{Lo: seamRatSub(minimum, departure), Hi: seamRatAdd(maximum, departure)}
	}
	r0, r1 := seamRatAdd(seamRatMul(m, box[0].Lo), b), seamRatAdd(seamRatMul(m, box[0].Hi), b)
	if r0.Cmp(r1) > 0 {
		r0, r1 = r1, r0
	}
	radius := loftFaceInterval{Lo: r0, Hi: r1}
	minimumF := seamRatSub(seamRatAdd(loftface.IntervalSquareMin(box[1]), loftface.IntervalSquareMin(box[2])),
		loftface.IntervalSquareMax(radius))
	maximumF := seamRatSub(seamRatAdd(loftface.IntervalSquareMax(box[1]), loftface.IntervalSquareMax(box[2])),
		loftface.IntervalSquareMin(radius))
	if outside && minimumF.Sign() > 0 || !outside && maximumF.Sign() < 0 {
		return true
	}
	if depth == 0 {
		return false
	}
	mid := seamRatMul(seamRatAdd(tLo, tHi), big.NewRat(1, 2))
	return SpiralCellSide(first, second, a, z, departure, m, b, outside, tLo, mid, depth-1) &&
		SpiralCellSide(first, second, a, z, departure, m, b, outside, mid, tHi, depth-1)
}
