package spiralsector

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
)

// The toe cap lies wholly before the toe tool/root circle in X. The heel
// cap may span heelX, so its full supporting plane must miss the entire
// heel tool/root circle. For A+B*cos(theta)+C*sin(theta), A²>B²+C² is an
// exact rational separation proof for every angle, including cap interiors.
func OriginalCapsAvoidToolRootCircles(trim *loftFacesTrimRecord,
	m, b, toeX, heelX *big.Rat) bool {
	box, err := revolveloftfaces.LfSourceBox(trim.Source)
	if err != nil || len(trim.Maps) != 9 {
		return false
	}
	for _, end := range [...]int{0, 8} {
		if loftface.PositiveDenominator(trim.Maps[end], box) != nil {
			return false
		}
	}
	return toeCapBeforeToeCircle(trim, box, toeX) &&
		loftface.CapPlaneAvoidsRootCircle(trim.Maps[8], loftface.BoxCorners(box), m, b, heelX)
}

// toeCapBeforeToeCircle reports whether the whole original cap 0 lies
// below toeX. The caller has proved cap 0's denominator positive over box.
func toeCapBeforeToeCircle(trim *loftFacesTrimRecord, box [4]*big.Rat, toeX *big.Rat) bool {
	xAxis := loftFaceRatVec{seamRatOne(), seamRatZero(), seamRatZero()}
	return loftface.ScalarRange(trim.Maps[0], xAxis, loftface.BoxCorners(box)).Hi.Cmp(toeX) < 0
}

// TangentCapTouchesHeelCircle is the cone-tangent branch of the heel
// half of OriginalCapsAvoidToolRootCircles. It runs only when the trim
// record carries a cone-tangent end that matches the heel cone and the final
// cap map exactly. The cap-8 plane then touches the heel tool/root circle at
// one point instead of missing it, which loftface.CapPlaneTouchesRootCircle
// decides exactly, with the circle on the side of the preceding cap.
func TangentCapTouchesHeelCircle(trim *loftFacesTrimRecord, m, b, toeX, heelX *big.Rat) bool {
	if trim == nil || trim.Tangent == nil || !SpiralTangentRecordCurrent(trim) || len(trim.Maps) != 9 {
		return false
	}
	box, err := revolveloftfaces.LfSourceBox(trim.Source)
	if err != nil {
		return false
	}
	for _, end := range [...]int{0, 7, 8} {
		if loftface.PositiveDenominator(trim.Maps[end], box) != nil {
			return false
		}
	}
	return toeCapBeforeToeCircle(trim, box, toeX) &&
		loftface.CapPlaneTouchesRootCircle(trim.Maps[8], loftface.BoxCorners(box), m, b, heelX, trim.Maps[7])
}

// The four non-flank source edges cannot create another root crossing.
// Each authenticated source contour cell lies within its cap's certified
// source and projective departure from the chord joining exact mapped
// stations. Band interpolation is convex in the two caps, so the entire
// ruled cell lies in the four-corner box enlarged by that departure.
func OtherSides(ctx context.Context, trim *loftFacesTrimRecord, m, b *big.Rat) bool {
	source := trim.Source
	n := len(source.SourceStation)
	if n == 0 || len(trim.StationGroup) != n {
		return false
	}
	var seen [6]int
	for band := range 8 {
		lo, hi := trim.Bounds[band], trim.Bounds[band+1]
		if len(lo.SourceDeparture) != n || len(hi.SourceDeparture) != n ||
			len(lo.ProjectiveDeparture) != n || len(hi.ProjectiveDeparture) != n {
			return false
		}
		for j, station := range source.SourceStation {
			if j%64 == 0 && ctx.Err() != nil {
				return false
			}
			group := trim.StationGroup[j]
			if group < 0 || group >= len(seen) {
				return false
			}
			if group == 0 || group == 2 {
				continue
			}
			seen[group]++
			departure := max(
				proofbound.AbsSumUpper(lo.SourceDeparture[j], lo.ProjectiveDeparture[j]),
				proofbound.AbsSumUpper(hi.SourceDeparture[j], hi.ProjectiveDeparture[j]))
			if proofbound.IsNonFinite(departure) || departure < 0 {
				return false
			}
			next := source.SourceStation[(j+1)%n]
			if !SpiralCellSide(trim.Maps[band], trim.Maps[band+1], station, next,
				seamRatF(departure), m, b, group == 1, seamRatZero(), seamRatOne(), 8) {
				return false
			}
		}
	}
	return seen[1] > 0 && seen[3] > 0 && seen[4] > 0 && seen[5] > 0
}

func IsolateJunction(ctx context.Context, maps []loftFaceMap, span freeform.BezierSpan,
	m, b *big.Rat, caps [9]loftface.RootCrossing, targetX *big.Rat) (facetedrecord.SpiralJunction, bool) {
	band := -1
	for i := range 8 {
		left, right := loftface.CapXRange(maps[i], span, caps[i]),
			loftface.CapXRange(maps[i+1], span, caps[i+1])
		if left.Hi.Cmp(targetX) < 0 && right.Lo.Cmp(targetX) > 0 {
			if band >= 0 {
				return facetedrecord.SpiralJunction{}, false
			}
			band = i
		}
	}
	if band < 0 {
		return facetedrecord.SpiralJunction{}, false
	}
	q := loftface.MeridianQRows(maps[band], maps[band+1], span, m, b)
	tLo, tHi := seamRatZero(), seamRatOne()
	for step := range 48 {
		if step%8 == 0 && ctx.Err() != nil {
			return facetedrecord.SpiralJunction{}, false
		}
		mid := seamRatMul(seamRatAdd(tLo, tHi), big.NewRat(1, 2))
		fixed := polynomial.BernsteinRowsAt(q, mid)
		root := loftface.IsolateRootN(fixed, 64)
		x := loftface.BandXRange(maps[band], maps[band+1], span, root, mid)
		switch {
		case x.Hi.Cmp(targetX) < 0:
			tLo = mid
		case x.Lo.Cmp(targetX) > 0:
			tHi = mid
		default:
			root = loftface.IsolateRootN(fixed, 128)
			x = loftface.BandXRange(maps[band], maps[band+1], span, root, mid)
			if x.Hi.Cmp(targetX) < 0 {
				tLo = mid
			} else if x.Lo.Cmp(targetX) > 0 {
				tHi = mid
			} else {
				return facetedrecord.SpiralJunction{}, false
			}
		}
	}
	leftRoot := loftface.IsolateRootN(polynomial.BernsteinRowsAt(q, tLo), 64)
	rightRoot := loftface.IsolateRootN(polynomial.BernsteinRowsAt(q, tHi), 64)
	sLo, sHi := leftRoot.Lo, leftRoot.Hi
	if rightRoot.Lo.Cmp(sLo) < 0 {
		sLo = rightRoot.Lo
	}
	if rightRoot.Hi.Cmp(sHi) > 0 {
		sHi = rightRoot.Hi
	}
	margin := big.NewRat(1, 1<<40)
	sLo, sHi = seamRatSub(sLo, margin), seamRatAdd(sHi, margin)
	if sLo.Sign() <= 0 || sHi.Cmp(seamRatOne()) >= 0 ||
		!JunctionRootBracket(q, sLo, sHi, tLo, tHi) {
		return facetedrecord.SpiralJunction{}, false
	}
	box, ok := loftface.JunctionBox(maps[band], maps[band+1], span, sLo, sHi, tLo, tHi)
	if !ok || box[0].Lo.Cmp(targetX) > 0 || box[0].Hi.Cmp(targetX) < 0 {
		return facetedrecord.SpiralJunction{}, false
	}
	box[0] = loftFaceInterval{Lo: targetX, Hi: targetX}
	return facetedrecord.SpiralJunction{Band: band, SLo: sLo, SHi: sHi, TLo: tLo, THi: tHi, Box: box}, true
}

func JunctionRootBracket(q [3][]*big.Rat, sLo, sHi, tLo, tHi *big.Rat) bool {
	for _, entry := range []struct {
		s    *big.Rat
		sign int
	}{{sLo, -1}, {sHi, 1}} {
		if !polynomial.BernsteinRowsSignAt(q, entry.s, tLo, tHi, entry.sign) {
			return false
		}
	}
	return true
}
