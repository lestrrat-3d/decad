package decad

import (
	"context"
	"math/big"
	"sort"
)

type revolveLoftFacesHeelBlankSlice struct {
	lo, hi *big.Rat
	outer  int
}

// The source event graph proves which part of each heel circle belongs to
// the trimmed LoftFaces body. The exact Revolve section proves that every
// such circle is strictly inside the blank between the two contact circles.
// Equality at the lower endpoint belongs only to its authenticated cap arc;
// the upper endpoint belongs to the authenticated vertical cap arc.
type revolveLoftFacesHeelRegionCertificate struct {
	*revolveLoftFacesHeelRegionEvents
	blank []revolveLoftFacesHeelBlankSlice
}

func certifyRevolveLoftFacesHeelRegion(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate) (
	*revolveLoftFacesHeelRegionCertificate, bool, error,
) {
	graph, ok, err := revolveLoftFacesHeelRegionEventGraph(ctx, proof)
	if err != nil || !ok {
		return nil, false, err
	}
	blank, ok := revolveLoftFacesHeelRegionBlankInside(ctx, proof, graph.lower, graph.upper)
	if !ok {
		return nil, false, nil
	}
	if !revolveLoftFacesHeelRegionTrimOpen(proof.trim, graph.lower.circle.x,
		graph.upper.circle.x) {
		return nil, false, nil
	}
	return &revolveLoftFacesHeelRegionCertificate{
		revolveLoftFacesHeelRegionEvents: graph, blank: blank}, true, nil
}

func revolveLoftFacesHeelRegionBlankInside(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	lower, upper *revolveLoftFacesToolCapOccupiedArc) (
	[]revolveLoftFacesHeelBlankSlice, bool,
) {
	if ctx == nil || proof == nil || proof.blank == nil || proof.trim == nil ||
		proof.trim.heel == nil || lower == nil || upper == nil ||
		lower.circle.x == nil || upper.circle.x == nil ||
		lower.circle.x.Cmp(upper.circle.x) >= 0 ||
		!revolveLoftFacesSectionCurrent(proof) {
		return nil, false
	}
	lowerCurrent, err := revolveLoftFacesToolCapOccupiedArcCurrent(ctx, proof, lower)
	if err != nil || !lowerCurrent {
		return nil, false
	}
	upperCurrent, err := revolveLoftFacesToolCapOccupiedArcCurrent(ctx, proof, upper)
	if err != nil || !upperCurrent {
		return nil, false
	}
	section := proof.section
	heel := proof.trim.heel
	if section.axisSpan < 0 || section.axisSpan >= len(section.spans) ||
		lower.circle.span < 0 || lower.circle.span >= len(section.spans) ||
		upper.circle.span < 0 || upper.circle.span >= len(section.spans) {
		return nil, false
	}
	axis := section.spans[section.axisSpan]
	if axis.start.V != 0 || axis.end.V != 0 ||
		lfF(axis.start.U).Cmp(lower.circle.x) >= 0 ||
		lfF(axis.end.U).Cmp(upper.circle.x) < 0 ||
		section.spans[upper.circle.span].start.U !=
			section.spans[upper.circle.span].end.U ||
		lfF(section.spans[upper.circle.span].start.U).Cmp(upper.circle.x) != 0 {
		return nil, false
	}
	limits := []*big.Rat{lower.circle.x, upper.circle.x}
	for index, span := range section.spans {
		if index == section.axisSpan {
			continue
		}
		x0, x1 := lfF(span.start.U), lfF(span.end.U)
		if x0.Cmp(x1) == 0 && x0.Cmp(lower.circle.x) > 0 &&
			x0.Cmp(upper.circle.x) < 0 {
			return nil, false // an interior radial step needs its own region proof
		}
		for _, x := range [...](*big.Rat){x0, x1} {
			if x.Cmp(lower.circle.x) > 0 && x.Cmp(upper.circle.x) < 0 {
				limits = append(limits, x)
			}
		}
	}
	sort.Slice(limits, func(i, j int) bool { return limits[i].Cmp(limits[j]) < 0 })
	unique := limits[:0]
	for _, value := range limits {
		if len(unique) == 0 || unique[len(unique)-1].Cmp(value) != 0 {
			unique = append(unique, value)
		}
	}
	if len(unique) < 2 || len(unique) > 130 {
		return nil, false
	}
	var result []revolveLoftFacesHeelBlankSlice
	for index := 1; index < len(unique); index++ {
		lo, hi := unique[index-1], unique[index]
		mid := lfDiv(lfAdd(lo, hi), big.NewRat(2, 1))
		outer := -1
		for spanIndex, span := range section.spans {
			if spanIndex == section.axisSpan {
				continue
			}
			x0, x1 := lfF(span.start.U), lfF(span.end.U)
			if x0.Cmp(mid) > 0 && x1.Cmp(mid) < 0 {
				if outer >= 0 || span.face == nil || span.start.V <= 0 || span.end.V <= 0 {
					return nil, false
				}
				outer = spanIndex
			} else if x0.Cmp(mid) < 0 && x1.Cmp(mid) > 0 {
				return nil, false // an additional crossing changes the filled radial interval
			}
		}
		if outer < 0 || index == 1 && outer != lower.circle.span {
			return nil, false
		}
		for _, x := range [...](*big.Rat){lo, hi} {
			outerRadius := revolveLoftFacesHeelRegionSpanRadius(section.spans[outer], x)
			heelRadius := lfDiv(lfSub(heel.apex, x), heel.slope)
			if outerRadius == nil || heelRadius.Sign() <= 0 {
				return nil, false
			}
			gap := lfSub(outerRadius, heelRadius)
			if x.Cmp(lower.circle.x) == 0 {
				if outer != lower.circle.span || gap.Sign() != 0 ||
					outerRadius.Cmp(lower.circle.radius) != 0 {
					return nil, false
				}
			} else if gap.Sign() <= 0 {
				return nil, false
			}
		}
		result = append(result, revolveLoftFacesHeelBlankSlice{lo: lo, hi: hi, outer: outer})
	}
	vertical := section.spans[upper.circle.span]
	bottom, top := lfF(vertical.start.V), lfF(vertical.end.V)
	if bottom.Cmp(top) > 0 {
		bottom, top = top, bottom
	}
	if bottom.Cmp(upper.circle.radius) >= 0 || top.Cmp(upper.circle.radius) <= 0 {
		return nil, false
	}
	return result, len(result) > 0
}

func revolveLoftFacesHeelRegionSpanRadius(span revolveSectionSpan, x *big.Rat) *big.Rat {
	if x == nil || span.start.U == span.end.U {
		return nil
	}
	x0, x1 := lfF(span.start.U), lfF(span.end.U)
	fraction := lfDiv(lfSub(x, x0), lfSub(x1, x0))
	if fraction.Sign() < 0 || fraction.Cmp(lfOne()) > 0 {
		return nil
	}
	return lfAdd(lfF(span.start.V), lfMul(fraction,
		lfSub(lfF(span.end.V), lfF(span.start.V))))
}

func revolveLoftFacesHeelRegionTrimOpen(trim *loftFacesTrimRecord, lo, hi *big.Rat) bool {
	if trim == nil || trim.toe == nil || trim.heel == nil ||
		trim.toe.apex == nil || trim.toe.far == nil || trim.toe.slope == nil ||
		trim.heel.apex == nil || trim.heel.far == nil || trim.heel.slope == nil ||
		trim.toe.slope.Sign() <= 0 || trim.heel.slope.Sign() <= 0 ||
		lo == nil || hi == nil || lo.Cmp(hi) >= 0 ||
		lo.Cmp(trim.toe.far) <= 0 || lo.Cmp(trim.heel.far) <= 0 ||
		hi.Cmp(trim.heel.apex) >= 0 {
		return false
	}
	points := []*big.Rat{lo, hi}
	if trim.toe.apex.Cmp(lo) > 0 && trim.toe.apex.Cmp(hi) < 0 {
		points = append(points, trim.toe.apex)
	}
	for _, x := range points {
		upper := lfDiv(lfSub(trim.heel.apex, x), trim.heel.slope)
		lower := lfZero()
		if x.Cmp(trim.toe.apex) < 0 {
			lower = lfDiv(lfSub(trim.toe.apex, x), trim.toe.slope)
		}
		if upper.Sign() <= 0 || upper.Cmp(lower) <= 0 {
			return false
		}
	}
	return true
}
