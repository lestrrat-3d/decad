package decad

import (
	"math/big"
	"sort"
)

// Selecting a certified time cell by an exact midpoint is only interval
// bookkeeping. The chosen cell must then cover the entire open interval;
// all source and derivative claims are exact Bernstein bounds on that cell.
func revolveLoftFacesHeelRegionSliceAt(trim *loftFacesTrimRecord,
	loop *revolveLoftFacesSimpleLoop, meridian revolveSectionSpan,
	cells []revolveLoftFacesMappedCell, band int, lo, hi *big.Rat,
	point bool) (revolveLoftFacesHeelRegionSlice, bool) {
	var result revolveLoftFacesHeelRegionSlice
	if trim == nil || loop == nil || lo == nil || hi == nil ||
		lo.Cmp(hi) > 0 || point != (lo.Cmp(hi) == 0) {
		return result, false
	}
	time := lo
	if !point {
		time = lfDiv(lfAdd(lo, hi), big.NewRat(2, 1))
	}
	active := make(map[revolveLoftFacesHeelSourceKey][]revolveLoftFacesMappedCell)
	for _, cell := range cells {
		if cell.band != band || !revolveLoftFacesTimeInCell(time, cell) {
			continue
		}
		if !point && (cell.tLo.Cmp(lo) > 0 || cell.tHi.Cmp(hi) < 0) {
			return result, false
		}
		key := revolveLoftFacesHeelSourceKey{cell.segment, cell.span}
		active[key] = append(active[key], cell)
	}
	loopIndex := make(map[revolveLoftFacesHeelSourceKey]int, len(loop.cells))
	for index, source := range loop.cells {
		key := revolveLoftFacesHeelSourceKey{source.segment, source.span}
		if source.arc != nil {
			key.span = -1
		}
		if _, duplicate := loopIndex[key]; duplicate {
			return result, false
		}
		loopIndex[key] = index
		parts := active[key]
		if len(parts) == 0 {
			return result, false
		}
		sort.Slice(parts, func(i, j int) bool {
			return parts[i].sLo.Cmp(parts[j].sLo) < 0
		})
		end := lfZero()
		for _, part := range parts {
			if part.sLo == nil || part.sHi == nil ||
				part.sLo.Cmp(end) != 0 || part.sHi.Cmp(part.sLo) <= 0 ||
				part.sHi.Cmp(lfOne()) > 0 {
				return result, false
			}
			end = part.sHi
		}
		if end.Cmp(lfOne()) != 0 {
			return result, false
		}
	}
	if len(active) != len(loop.cells) {
		return result, false
	}
	var events []revolveLoftFacesHeelRegionEvent
	for _, source := range loop.cells {
		key := revolveLoftFacesHeelSourceKey{source.segment, source.span}
		if source.arc != nil {
			key.span = -1
		}
		for _, cell := range active[key] {
			switch cell.side {
			case revolveLoftFacesInside, revolveLoftFacesOutside, revolveLoftFacesAxialMiss:
				continue
			case revolveLoftFacesCrossed:
				events = append(events, revolveLoftFacesHeelRegionEvent{
					members: []revolveLoftFacesMappedCell{cell}})
			case revolveLoftFacesJointContact:
				if cell.jointPoint == nil || cell.jointKind == revolveLoftFacesWholeArcContact ||
					cell.jointKind == 0 {
					return result, false
				}
				found := -1
				for index := range events {
					if events[index].kind == cell.jointKind &&
						events[index].point == cell.jointPoint {
						found = index
						break
					}
				}
				if found < 0 {
					events = append(events, revolveLoftFacesHeelRegionEvent{
						kind: cell.jointKind, point: cell.jointPoint})
					found = len(events) - 1
				}
				events[found].members = append(events[found].members, cell)
			default:
				return result, false
			}
		}
	}
	if len(events) != 2 {
		return result, false
	}
	for index := range events {
		event := &events[index]
		if event.point != nil {
			if len(event.members) != 2 {
				return result, false
			}
			first := event.members[0]
			second := event.members[1]
			a := loopIndex[revolveLoftFacesHeelSourceKey{first.segment, first.span}]
			b := loopIndex[revolveLoftFacesHeelSourceKey{second.segment, second.span}]
			if (a+1)%len(loop.cells) != b && (b+1)%len(loop.cells) != a ||
				first.tLo.Cmp(second.tLo) != 0 || first.tHi.Cmp(second.tHi) != 0 {
				return result, false
			}
		} else if len(event.members) != 1 {
			return result, false
		}
		for _, cell := range event.members {
			sign := revolveLoftFacesHeelRegionCrossingSign(trim, meridian, cell)
			if sign == 0 || event.sign != 0 && event.sign != sign {
				return result, false
			}
			event.sign = sign
		}
		if event.point != nil &&
			!revolveLoftFacesHeelRegionJointCrosses(trim, meridian, loopIndex, *event) {
			return result, false
		}
	}
	if events[0].sign == events[1].sign {
		return result, false
	}
	if events[0].sign > events[1].sign {
		events[0], events[1] = events[1], events[0]
	}
	result = revolveLoftFacesHeelRegionSlice{band: band, lo: lo, hi: hi, point: point,
		events: [2]revolveLoftFacesHeelRegionEvent{events[0], events[1]}}
	return result, true
}

func revolveLoftFacesHeelRegionCrossingSign(trim *loftFacesTrimRecord,
	meridian revolveSectionSpan, cell revolveLoftFacesMappedCell) int {
	if trim == nil || trim.source == nil || trim.source.source == nil ||
		cell.segment < 0 || cell.segment >= len(trim.source.profile.Outer.Segments) ||
		cell.band < 0 || cell.band+1 >= len(trim.maps) || cell.span < 0 ||
		cell.sLo == nil || cell.sHi == nil || cell.tLo == nil || cell.tHi == nil {
		return 0
	}
	spans, ok := revolveLoftFacesStraightOrFitSpans(trim, cell.segment,
		trim.source.profile.Outer.Segments[cell.segment])
	if !ok || cell.span >= len(spans) {
		return 0
	}
	m, b := revolveLoftFacesMeridian(meridian)
	q := spiralRootPolynomial(trim.maps[cell.band], trim.maps[cell.band+1],
		spans[cell.span], m, b)
	q = revolveLoftFacesRestrictBernstein(q, cell.sLo, cell.sHi, cell.tLo, cell.tHi)
	var derivative [3][]*big.Rat
	for row := range q {
		derivative[row] = spiralDeriv(q[row])
	}
	sign := 0
	if spiralAllPositive(derivative) {
		sign = 1
	} else if spiralAllPositive(revolveLoftFacesNegateRows(derivative)) {
		sign = -1
	}
	walkStart, walkEnd := lfZero(), lfOne()
	if trim.source.source.profile.Outer[cell.segment].Reversed {
		sign = -sign
		walkStart, walkEnd = walkEnd, walkStart
	}
	if sign != 0 && cell.side == revolveLoftFacesCrossed &&
		(revolveLoftFacesTimePointSign(q, walkStart) != -sign ||
			revolveLoftFacesTimePointSign(q, walkEnd) != sign) {
		return 0
	}
	return sign
}

// A joint root may lie on either adjacent source cell as time changes.
// Strict monotonicity on each cell and opposite far-end signs prove that
// their concatenated walk owns exactly one crossing at every root time.
func revolveLoftFacesHeelRegionJointCrosses(trim *loftFacesTrimRecord,
	meridian revolveSectionSpan,
	loopIndex map[revolveLoftFacesHeelSourceKey]int,
	event revolveLoftFacesHeelRegionEvent) bool {
	if trim == nil || len(loopIndex) < 2 || len(event.members) != 2 || event.point == nil ||
		event.sign == 0 || trim.source == nil || trim.source.source == nil {
		return false
	}
	first, second := event.members[0], event.members[1]
	if first.side != revolveLoftFacesJointContact ||
		second.side != revolveLoftFacesJointContact ||
		first.jointKind != event.kind || second.jointKind != event.kind ||
		first.jointPoint != event.point || second.jointPoint != event.point ||
		first.band != second.band || first.tLo == nil || first.tHi == nil ||
		second.tLo == nil || second.tHi == nil ||
		first.tLo.Cmp(second.tLo) != 0 || first.tHi.Cmp(second.tHi) != 0 {
		return false
	}
	a, foundA := loopIndex[revolveLoftFacesHeelSourceKey{first.segment, first.span}]
	b, foundB := loopIndex[revolveLoftFacesHeelSourceKey{second.segment, second.span}]
	if !foundA || !foundB ||
		(a+1)%len(loopIndex) != b && (b+1)%len(loopIndex) != a {
		return false
	}
	if (a+1)%len(loopIndex) != b {
		first, second = second, first
	}
	m, intercept := revolveLoftFacesMeridian(meridian)
	var ends [2][3]*big.Rat
	for index, cell := range [...]revolveLoftFacesMappedCell{first, second} {
		spans, ok := revolveLoftFacesStraightOrFitSpans(trim, cell.segment,
			trim.source.profile.Outer.Segments[cell.segment])
		if !ok || cell.span < 0 || cell.span >= len(spans) {
			return false
		}
		q := spiralRootPolynomial(trim.maps[cell.band], trim.maps[cell.band+1],
			spans[cell.span], m, intercept)
		q = revolveLoftFacesRestrictBernstein(q, lfZero(), lfOne(), cell.tLo, cell.tHi)
		walkStart, walkEnd := lfZero(), lfOne()
		if trim.source.source.profile.Outer[cell.segment].Reversed {
			walkStart, walkEnd = walkEnd, walkStart
		}
		outer, shared := walkStart, walkEnd
		want := -event.sign
		if index == 1 {
			outer, shared = walkEnd, walkStart
			want = event.sign
		}
		if revolveLoftFacesTimePointSign(q, outer) != want {
			return false
		}
		for row := range q {
			ends[index][row] = spiralBernValue(q[row], shared)
		}
	}
	for row := range ends[0] {
		if ends[0][row].Cmp(ends[1][row]) != 0 {
			return false
		}
	}
	return true
}

func revolveLoftFacesHeelRegionLinked(a, b revolveLoftFacesHeelRegionSlice) bool {
	for index := range a.events {
		left, right := a.events[index], b.events[index]
		if left.sign != right.sign {
			return false
		}
		linked := false
		for _, first := range left.members {
			for _, second := range right.members {
				if first.segment == second.segment && first.span == second.span {
					linked = true
				}
			}
		}
		if !linked {
			return false
		}
	}
	return true
}

func revolveLoftFacesHeelRegionArcEnds(slice revolveLoftFacesHeelRegionSlice,
	arc *revolveLoftFacesToolCapOccupiedArc) bool {
	if arc == nil || !slice.point || slice.band != arc.band ||
		slice.lo.Cmp(arc.time) != 0 {
		return false
	}
	seen := [2]bool{}
	for _, boundary := range [...]revolveLoftFacesToolCapBoundary{arc.negativeZ, arc.positiveZ} {
		index := 0
		if boundary.outward {
			index = 1
		}
		if seen[index] {
			return false
		}
		for _, member := range slice.events[index].members {
			if member.segment == boundary.segment && member.span == boundary.span {
				seen[index] = true
			}
		}
		if !seen[index] {
			return false
		}
	}
	return seen[0] && seen[1]
}
