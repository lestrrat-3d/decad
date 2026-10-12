package decad

import (
	"context"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/sketch"
)

// A heel-region event names one transverse intersection of the live source
// loop with the moving heel-cone circle. A source vertex has two adjacent
// cells but owns only one intersection.
type revolveLoftFacesHeelRegionEvent struct {
	sign    int // sign of the radial residual along the CCW source walk
	members []revolveLoftFacesMappedCell
	point   *sketch.Point
	kind    revolveLoftFacesJointKind
}

type revolveLoftFacesHeelRegionSlice struct {
	band   int
	lo, hi *big.Rat
	point  bool
	events [2]revolveLoftFacesHeelRegionEvent
}

type revolveLoftFacesHeelRegionEvents struct {
	lower, upper *revolveLoftFacesToolCapOccupiedArc
	meridian     revolveSectionSpan
	slices       []revolveLoftFacesHeelRegionSlice
}

type revolveLoftFacesHeelSourceKey struct{ segment, span int }

// This is a source-only graph. Every exact time point and open interval
// between the two authenticated heel circles has a complete source-cell
// partition and exactly two opposite transverse intersections. It does not
// select held facets or admit Union.
func revolveLoftFacesHeelRegionEventGraph(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate) (*revolveLoftFacesHeelRegionEvents, bool, error) {
	if ctx == nil || proof == nil || proof.trim == nil || proof.trim.heel == nil {
		return nil, false, nil
	}
	census, ok, err := revolveLoftFacesCensus(ctx, proof)
	if err != nil || !ok {
		return nil, false, err
	}
	var arcs []*revolveLoftFacesToolCapOccupiedArc
	for index := range census.pairs {
		pair := &census.pairs[index]
		if pair.verdict == revolveLoftFacesPairToolArc {
			if pair.toolArc == nil || pair.toolArc.circle.tool != 1 {
				return nil, false, nil
			}
			arcs = append(arcs, pair.toolArc)
		}
	}
	if len(arcs) != 2 || arcs[0].circle.x == nil || arcs[1].circle.x == nil {
		return nil, false, nil
	}
	if arcs[0].circle.x.Cmp(arcs[1].circle.x) > 0 {
		arcs[0], arcs[1] = arcs[1], arcs[0]
	}
	lower, upper := arcs[0], arcs[1]
	if lower.circle.x.Cmp(upper.circle.x) >= 0 ||
		lower.band < 0 || lower.band > upper.band ||
		upper.band >= len(proof.trim.maps)-1 ||
		lower.band == upper.band && lower.time.Cmp(upper.time) >= 0 ||
		lower.throughPositiveY != upper.throughPositiveY {
		return nil, false, nil
	}
	loop, simple, err := revolveLoftFacesSimpleSourceLoop(ctx, proof.trim)
	if err != nil || !simple || loop == nil || !loop.ccw {
		return nil, false, err
	}
	meridian, ok := revolveLoftFacesHeelRegionMeridian(proof.trim.heel, lower, upper)
	if !ok {
		return nil, false, nil
	}
	for band := lower.band; band <= upper.band; band++ {
		if !revolveLoftFacesHeelRegionAffine(proof.trim, band) {
			return nil, false, nil
		}
	}
	cells, ok, err := revolveLoftFacesFiniteSpanCells(ctx, proof.trim, meridian)
	if err != nil || !ok {
		return nil, false, err
	}
	result := &revolveLoftFacesHeelRegionEvents{lower: lower, upper: upper, meridian: meridian}
	for band := lower.band; band <= upper.band; band++ {
		lo, hi := lfZero(), lfOne()
		if band == lower.band {
			lo = lower.time
		}
		if band == upper.band {
			hi = upper.time
		}
		if lo.Cmp(hi) >= 0 {
			return nil, false, nil
		}
		bounds := []*big.Rat{lo, hi}
		for _, cell := range cells {
			if cell.band != band || cell.tLo == nil || cell.tHi == nil {
				continue
			}
			if cell.tLo.Cmp(lo) > 0 && cell.tLo.Cmp(hi) < 0 {
				bounds = append(bounds, cell.tLo)
			}
			if cell.tHi.Cmp(lo) > 0 && cell.tHi.Cmp(hi) < 0 {
				bounds = append(bounds, cell.tHi)
			}
		}
		sort.Slice(bounds, func(i, j int) bool { return bounds[i].Cmp(bounds[j]) < 0 })
		unique := bounds[:0]
		for _, bound := range bounds {
			if len(unique) == 0 || unique[len(unique)-1].Cmp(bound) != 0 {
				unique = append(unique, bound)
			}
		}
		if len(unique) > 256 {
			return nil, false, nil
		}
		for index, bound := range unique {
			if err := ctx.Err(); err != nil {
				return nil, false, err
			}
			point, ok := revolveLoftFacesHeelRegionSliceAt(proof.trim, loop, meridian,
				cells, band, bound, bound, true)
			if !ok {
				return nil, false, nil
			}
			result.slices = append(result.slices, point)
			if index+1 == len(unique) {
				continue
			}
			open, ok := revolveLoftFacesHeelRegionSliceAt(proof.trim, loop, meridian,
				cells, band, bound, unique[index+1], false)
			if !ok || !revolveLoftFacesHeelRegionLinked(point, open) {
				return nil, false, nil
			}
			result.slices = append(result.slices, open)
		}
	}
	for index := 1; index < len(result.slices); index++ {
		previous, current := result.slices[index-1], result.slices[index]
		if previous.band == current.band && previous.point && !current.point {
			continue // checked when the open interval was appended
		}
		if !revolveLoftFacesHeelRegionLinked(previous, current) {
			return nil, false, nil
		}
	}
	if len(result.slices) == 0 ||
		!revolveLoftFacesHeelRegionArcEnds(result.slices[0], lower) ||
		!revolveLoftFacesHeelRegionArcEnds(result.slices[len(result.slices)-1], upper) {
		return nil, false, nil
	}
	return result, true, nil
}

// Use only exact float-representable points of the live cone meridian.
// The upper cap circle is the finite endpoint; the lower witness lies less
// than one cap-interval width before the lower circle.
func revolveLoftFacesHeelRegionMeridian(heel *pointConeLimit,
	lower, upper *revolveLoftFacesToolCapOccupiedArc) (revolveSectionSpan, bool) {
	var empty revolveSectionSpan
	if heel == nil || heel.apex == nil || heel.far == nil || heel.slope == nil ||
		heel.slope.Sign() <= 0 || lower == nil || upper == nil ||
		lower.circle.x == nil || upper.circle.x == nil || upper.circle.radius == nil ||
		heel.far.Cmp(lower.circle.x) >= 0 || upper.circle.x.Cmp(heel.apex) >= 0 {
		return empty, false
	}
	if lfAdd(upper.circle.x, lfMul(heel.slope, upper.circle.radius)).Cmp(heel.apex) != 0 {
		return empty, false
	}
	ux, ur, ok := revolveLoftFacesExactFloatPoint(upper.circle.x, upper.circle.radius)
	if !ok {
		return empty, false
	}
	minimum := lfSub(lower.circle.x, lfSub(upper.circle.x, lower.circle.x))
	distance := lfSub(heel.apex, heel.far)
	for power := uint(1); power <= 16; power++ {
		denominator := int64(1) << power
		fraction := lfDiv(lfSub(lower.circle.x, heel.far), distance)
		scaled := lfMul(fraction, big.NewRat(denominator, 1))
		floor := new(big.Int).Quo(scaled.Num(), scaled.Denom())
		for _, numerator := range [...]int64{floor.Int64(), floor.Int64() - 1} {
			if numerator <= 0 || numerator >= denominator {
				continue
			}
			part := big.NewRat(numerator, denominator)
			x := lfAdd(heel.far, lfMul(part, distance))
			if x.Cmp(lower.circle.x) >= 0 || x.Cmp(minimum) <= 0 {
				continue
			}
			r := lfDiv(lfSub(heel.apex, x), heel.slope)
			lx, lr, exact := revolveLoftFacesExactFloatPoint(x, r)
			if !exact || lx >= ux || lr <= ur {
				continue
			}
			section := revolveSectionSpan{start: Point2{U: lx, V: lr},
				end: Point2{U: ux, V: ur}}
			m, b := revolveLoftFacesMeridian(section)
			if m.Cmp(lfDiv(big.NewRat(-1, 1), heel.slope)) == 0 &&
				b.Cmp(lfDiv(heel.apex, heel.slope)) == 0 {
				return section, true
			}
		}
	}
	return empty, false
}

func revolveLoftFacesExactFloatPoint(x, radius *big.Rat) (float64, float64, bool) {
	if x == nil || radius == nil || radius.Sign() <= 0 {
		return 0, 0, false
	}
	a, _ := x.Float64()
	b, _ := radius.Float64()
	return a, b, !math.IsInf(a, 0) && !math.IsInf(b, 0) &&
		!math.IsNaN(a) && !math.IsNaN(b) &&
		lfF(a).Cmp(x) == 0 && lfF(b).Cmp(radius) == 0
}

func revolveLoftFacesHeelRegionAffine(trim *loftFacesTrimRecord, band int) bool {
	if trim == nil || trim.source == nil || band < 0 || band+1 >= len(trim.maps) {
		return false
	}
	box, err := lfSourceBox(trim.source)
	if err != nil || lfPositiveDenominator(trim.maps[band], box) != nil ||
		lfPositiveDenominator(trim.maps[band+1], box) != nil {
		return false
	}
	a, okA := revolveLoftFacesConstantX(trim.maps[band])
	b, okB := revolveLoftFacesConstantX(trim.maps[band+1])
	if !okA || !okB || a.Cmp(b) >= 0 {
		return false
	}
	first, okA := revolveLoftFacesVerticalAffineMap(trim.maps[band], trim.maps[band+1], lfZero())
	middle, okM := revolveLoftFacesVerticalAffineMap(trim.maps[band], trim.maps[band+1], big.NewRat(1, 2))
	last, okB := revolveLoftFacesVerticalAffineMap(trim.maps[band], trim.maps[band+1], lfOne())
	if !okA || !okM || !okB {
		return false
	}
	control := lfSub(lfMul(big.NewRat(2, 1), middle.det),
		lfDiv(lfAdd(first.det, last.det), big.NewRat(2, 1)))
	return first.det.Sign() > 0 && control.Sign() > 0 && last.det.Sign() > 0
}
