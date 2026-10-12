package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
)

// A member box encloses one part of a heel-cap event. A source joint can
// occupy either of its two live members during a time slab.
type revolveLoftFacesHeelRootMember struct {
	segment, span, band int
	ref                 FeatureRef
	face                *Face
	source              loftFaceInterval
	box                 [3]loftFaceInterval
}

type revolveLoftFacesHeelRootEnclosure struct {
	time    loftFaceInterval
	members []revolveLoftFacesHeelRootMember
	box     [3]loftFaceInterval
}

// The public-facing helper rederives the source graph before using a stored
// event. A builder that already holds a freshly rederived graph can call the
// lower helper for several stations without repeating that source audit.
func revolveLoftFacesHeelEventBox(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, region *revolveLoftFacesHeelRegionEvents,
	sliceIndex, eventIndex int, tLo, tHi, budget *big.Rat) (
	*revolveLoftFacesHeelRootEnclosure, bool, error,
) {
	if ctx == nil || proof == nil || region == nil ||
		sliceIndex < 0 || sliceIndex >= len(region.slices) ||
		eventIndex < 0 || eventIndex >= 2 {
		return nil, false, nil
	}
	fresh, ok, err := revolveLoftFacesHeelRegionEventGraph(ctx, proof)
	if err != nil || !ok || fresh == nil ||
		sliceIndex >= len(fresh.slices) ||
		!revolveLoftFacesSameHeelSlice(region.slices[sliceIndex], fresh.slices[sliceIndex]) {
		return nil, false, err
	}
	return revolveLoftFacesHeelEventBoxFromCurrent(ctx, proof, fresh,
		sliceIndex, eventIndex, tLo, tHi, budget)
}

func revolveLoftFacesSameHeelSlice(a, b revolveLoftFacesHeelRegionSlice) bool {
	if a.band != b.band || a.point != b.point ||
		!revolveLoftFacesSameRat(a.lo, b.lo) || !revolveLoftFacesSameRat(a.hi, b.hi) {
		return false
	}
	for i := range a.events {
		left, right := a.events[i], b.events[i]
		if left.sign != right.sign || left.kind != right.kind ||
			left.point != right.point || len(left.members) != len(right.members) {
			return false
		}
		for j := range left.members {
			if !revolveLoftFacesSameMappedCell(left.members[j], right.members[j]) {
				return false
			}
		}
	}
	return true
}

// The caller must supply a graph just rederived from the live producer.
// Every returned box is an exact enclosure of the unique event root over
// the entire closed time slab. A narrow requested budget is a refusal, not
// permission to pick a sampled root or enlarge the source allowance.
func revolveLoftFacesHeelEventBoxFromCurrent(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, region *revolveLoftFacesHeelRegionEvents,
	sliceIndex, eventIndex int, tLo, tHi, budget *big.Rat) (
	*revolveLoftFacesHeelRootEnclosure, bool, error,
) {
	if ctx == nil || proof == nil || proof.tooth == nil || proof.trim == nil || region == nil ||
		sliceIndex < 0 || sliceIndex >= len(region.slices) ||
		eventIndex < 0 || eventIndex >= 2 || tLo == nil || tHi == nil ||
		budget == nil || budget.Sign() <= 0 || tLo.Cmp(tHi) > 0 {
		return nil, false, nil
	}
	slice := region.slices[sliceIndex]
	if slice.lo == nil || slice.hi == nil || tLo.Cmp(slice.lo) < 0 ||
		tHi.Cmp(slice.hi) > 0 || slice.point &&
		(tLo.Cmp(slice.lo) != 0 || tHi.Cmp(slice.lo) != 0) {
		return nil, false, nil
	}
	event := slice.events[eventIndex]
	if event.sign == 0 || len(event.members) < 1 || len(event.members) > 2 ||
		len(event.members) == 2 && event.point == nil {
		return nil, false, nil
	}
	loop, simple, err := revolveLoftFacesSimpleSourceLoop(ctx, proof.trim)
	if err != nil || !simple || loop == nil || !loop.ccw {
		return nil, false, err
	}
	index := make(map[revolveLoftFacesHeelSourceKey]int, len(loop.cells))
	for i, source := range loop.cells {
		key := revolveLoftFacesHeelSourceKey{source.segment, source.span}
		if source.arc != nil {
			key.span = -1
		}
		index[key] = i
	}
	if len(event.members) == 2 &&
		!revolveLoftFacesHeelRegionJointCrosses(proof.trim, region.meridian, index, event) {
		return nil, false, nil
	}
	var spans []revolveLoftFacesHeelRootSpan
	for _, cell := range event.members {
		part, ok := revolveLoftFacesHeelRootSpanOf(proof, region.meridian,
			cell, tLo, tHi, event.sign)
		if !ok {
			return nil, false, nil
		}
		spans = append(spans, part)
	}
	if len(spans) == 2 {
		first, foundFirst := index[revolveLoftFacesHeelSourceKey{spans[0].cell.segment,
			spans[0].cell.span}]
		second, foundSecond := index[revolveLoftFacesHeelSourceKey{spans[1].cell.segment,
			spans[1].cell.span}]
		if !foundFirst || !foundSecond {
			return nil, false, nil
		}
		if (second+1)%len(loop.cells) == first {
			spans[0], spans[1] = spans[1], spans[0]
		} else if (first+1)%len(loop.cells) != second {
			return nil, false, nil
		}
	}
	left, right := lfZero(), big.NewRat(int64(len(spans)), 1)
	if revolveLoftFacesHeelRootSign(spans, left) != -event.sign ||
		revolveLoftFacesHeelRootSign(spans, right) != event.sign {
		return nil, false, nil
	}
	for range 48 {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		enclosure, ok := revolveLoftFacesHeelRootBox(spans, left, right, tLo, tHi)
		if !ok {
			return nil, false, nil
		}
		radius := revolveLoftFacesHeelBoxRadius(enclosure.box)
		if radius == nil {
			return nil, false, nil
		}
		if radius.Cmp(budget) <= 0 {
			return enclosure, true, nil
		}
		mid := lfDiv(lfAdd(left, right), big.NewRat(2, 1))
		candidates := []*big.Rat{mid,
			lfDiv(lfAdd(left, mid), big.NewRat(2, 1)),
			lfDiv(lfAdd(mid, right), big.NewRat(2, 1))}
		changed := false
		for _, candidate := range candidates {
			if candidate.Cmp(left) <= 0 || candidate.Cmp(right) >= 0 {
				continue
			}
			sign := revolveLoftFacesHeelRootSign(spans, candidate)
			switch sign {
			case -event.sign:
				left = candidate
				changed = true
			case event.sign:
				right = candidate
				changed = true
			}
		}
		if !changed {
			return nil, false, nil
		}
	}
	return nil, false, nil
}

type revolveLoftFacesHeelRootSpan struct {
	cell     revolveLoftFacesMappedCell
	span     freeform.BezierSpan
	q        [3][]*big.Rat
	time     loftFaceInterval
	ref      FeatureRef
	face     *Face
	reversed bool
	first    loftFaceMap
	last     loftFaceMap
}

func revolveLoftFacesHeelRootSpanOf(proof *revolveLoftFacesSourceCertificate,
	meridian revolveSectionSpan, cell revolveLoftFacesMappedCell,
	tLo, tHi *big.Rat, direction int) (revolveLoftFacesHeelRootSpan, bool) {
	var empty revolveLoftFacesHeelRootSpan
	trim := proof.trim
	if trim.source == nil || trim.source.source == nil ||
		trim.source.source.profile == nil ||
		cell.segment < 0 || cell.segment >= len(trim.source.profile.Outer.Segments) ||
		cell.band < 0 || cell.band+1 >= len(trim.maps) ||
		cell.band >= len(proof.sourceRefs) ||
		cell.segment >= len(proof.sourceRefs[cell.band]) ||
		cell.sLo == nil || cell.sHi == nil || cell.tLo == nil || cell.tHi == nil ||
		cell.sLo.Cmp(cell.sHi) >= 0 || tLo.Cmp(cell.tLo) < 0 ||
		tHi.Cmp(cell.tHi) > 0 ||
		cell.side != revolveLoftFacesCrossed && cell.side != revolveLoftFacesJointContact ||
		revolveLoftFacesHeelRegionCrossingSign(trim, meridian, cell) != direction {
		return empty, false
	}
	spans, ok := revolveLoftFacesStraightOrFitSpans(trim, cell.segment,
		trim.source.profile.Outer.Segments[cell.segment])
	if !ok || cell.span < 0 || cell.span >= len(spans) {
		return empty, false
	}
	ref := proof.sourceRefs[cell.band][cell.segment]
	face := revolveLoftFacesVerticalLiveFace(proof.tooth, ref)
	if face == nil {
		return empty, false
	}
	first, last := trim.maps[cell.band], trim.maps[cell.band+1]
	m, b := revolveLoftFacesMeridian(meridian)
	q := spiralRootPolynomial(first, last, spans[cell.span], m, b)
	return revolveLoftFacesHeelRootSpan{cell: cell, span: spans[cell.span],
		q: q, time: loftFaceInterval{lo: tLo, hi: tHi}, ref: ref, face: face,
		reversed: trim.source.source.profile.Outer[cell.segment].Reversed,
		first:    first, last: last}, true
}

func revolveLoftFacesHeelRootParameter(span revolveLoftFacesHeelRootSpan,
	walk *big.Rat) *big.Rat {
	width := lfSub(span.cell.sHi, span.cell.sLo)
	if span.reversed {
		return lfSub(span.cell.sHi, lfMul(width, walk))
	}
	return lfAdd(span.cell.sLo, lfMul(width, walk))
}

func revolveLoftFacesHeelRootSign(spans []revolveLoftFacesHeelRootSpan,
	global *big.Rat) int {
	if len(spans) == 0 || global == nil || global.Sign() < 0 ||
		global.Cmp(big.NewRat(int64(len(spans)), 1)) > 0 {
		return 0
	}
	index := 0
	if len(spans) == 2 && global.Cmp(lfOne()) > 0 {
		index = 1
	}
	local := lfSub(global, big.NewRat(int64(index), 1))
	span := spans[index]
	parameter := revolveLoftFacesHeelRootParameter(span, local)
	if span.time.lo.Cmp(span.time.hi) == 0 {
		coefficients := make([]*big.Rat, len(span.q[0]))
		for column := range coefficients {
			coefficients[column] = spiralBernValue([]*big.Rat{
				span.q[0][column], span.q[1][column], span.q[2][column]},
				span.time.lo)
		}
		return spiralBernValue(coefficients, parameter).Sign()
	}
	return revolveLoftFacesTimePointSign(revolveLoftFacesRestrictBernstein(
		span.q, lfZero(), lfOne(), span.time.lo, span.time.hi), parameter)
}

func revolveLoftFacesHeelRootBox(spans []revolveLoftFacesHeelRootSpan,
	left, right, tLo, tHi *big.Rat) (*revolveLoftFacesHeelRootEnclosure, bool) {
	result := &revolveLoftFacesHeelRootEnclosure{time: loftFaceInterval{lo: tLo, hi: tHi}}
	for index, span := range spans {
		start := big.NewRat(int64(index), 1)
		end := big.NewRat(int64(index+1), 1)
		if left.Cmp(end) >= 0 || right.Cmp(start) <= 0 {
			continue
		}
		lo, hi := left, right
		if lo.Cmp(start) < 0 {
			lo = start
		}
		if hi.Cmp(end) > 0 {
			hi = end
		}
		s0 := revolveLoftFacesHeelRootParameter(span, lfSub(lo, start))
		s1 := revolveLoftFacesHeelRootParameter(span, lfSub(hi, start))
		if s0.Cmp(s1) > 0 {
			s0, s1 = s1, s0
		}
		box, ok := spiralJunctionBox(span.first, span.last, span.span,
			s0, s1, tLo, tHi)
		if !ok {
			return nil, false
		}
		member := revolveLoftFacesHeelRootMember{segment: span.cell.segment,
			span: span.cell.span, band: span.cell.band, ref: span.ref,
			face: span.face, source: loftFaceInterval{lo: s0, hi: s1}, box: box}
		result.members = append(result.members, member)
		if len(result.members) == 1 {
			result.box = box
		} else {
			for axis := range result.box {
				result.box[axis] = lfIntervalHull(result.box[axis], box[axis])
			}
		}
	}
	return result, len(result.members) > 0
}

func revolveLoftFacesHeelBoxRadius(box [3]loftFaceInterval) *big.Rat {
	total := lfZero()
	for _, axis := range box {
		if axis.lo == nil || axis.hi == nil || axis.lo.Cmp(axis.hi) > 0 {
			return nil
		}
		total = lfAdd(total, lfSub(axis.hi, axis.lo))
	}
	return lfDiv(total, big.NewRat(2, 1))
}
