package decad

import (
	"context"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/sketch"
)

// The two moving heel-cap crossings end on opposite live endpoints of one
// whole radial Arc. The true terminal time lies in root; the source is empty
// on the heel circle after that root through endTime. This is a source-only
// certificate and does not select any held cap facets.
type revolveLoftFacesHeelTerminal struct {
	upper    *revolveLoftFacesToolCapOccupiedArc
	meridian revolveSectionSpan
	arc      *sketch.Arc
	ends     [2]*sketch.Point
	lines    [2]int
	root     loftFaceInterval
	rootX    loftFaceInterval
	endTime  *big.Rat
	before   revolveLoftFacesHeelRegionSlice
	capFace  *Face
}

func certifyRevolveLoftFacesHeelTerminal(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate) (
	*revolveLoftFacesHeelTerminal, bool, error,
) {
	if ctx == nil || proof == nil || proof.trim == nil || proof.trim.heel == nil ||
		!revolveLoftFacesSectionCurrent(proof) {
		return nil, false, nil
	}
	region, ok, err := certifyRevolveLoftFacesHeelRegion(ctx, proof)
	if err != nil || !ok || region == nil || region.upper == nil {
		return nil, false, err
	}
	upper := region.upper
	if !revolveLoftFacesHeelTerminalBlankMiss(proof, upper) {
		return nil, false, nil
	}
	meridian, endTime, ok := revolveLoftFacesHeelTerminalMeridian(proof.trim, upper)
	if !ok {
		return nil, false, nil
	}
	loop, ok, err := revolveLoftFacesSimpleSourceLoop(ctx, proof.trim)
	if err != nil || !ok || loop == nil || !loop.ccw {
		return nil, false, err
	}
	cells, ok, err := revolveLoftFacesFiniteSpanCells(ctx, proof.trim, meridian)
	if err != nil || !ok {
		return nil, false, err
	}
	var terminal *revolveLoftFacesHeelTerminal
	entities := proof.trim.source.source.entities
	for arcIndex, entity := range entities {
		arc, isArc := entity.(*sketch.Arc)
		if !isArc {
			continue
		}
		left := (arcIndex - 1 + len(entities)) % len(entities)
		right := (arcIndex + 1) % len(entities)
		if _, ok := entities[left].(*sketch.Line); !ok {
			continue
		}
		if _, ok := entities[right].(*sketch.Line); !ok {
			continue
		}
		first, firstOK := certifyRevolveLoftFacesLineArcJoint(ctx, proof.trim,
			meridian, left, arcIndex, upper.band, true)
		second, secondOK := certifyRevolveLoftFacesLineArcJoint(ctx, proof.trim,
			meridian, right, arcIndex, upper.band, false)
		if !firstOK || !secondOK || first.vertex.arc != arc || second.vertex.arc != arc ||
			first.vertex.point == second.vertex.point {
			continue
		}
		for _, a := range first.roots {
			for _, b := range second.roots {
				if !a.finite || !b.finite || a.time.lo.Cmp(b.time.lo) != 0 ||
					a.time.hi.Cmp(b.time.hi) != 0 ||
					a.time.lo.Cmp(upper.time) <= 0 || a.time.hi.Cmp(endTime) >= 0 {
					continue
				}
				whole, radialOK := mappedArcRadialTimeBand(ctx, proof.trim,
					arcIndex, upper.band, meridian)
				if !radialOK || whole.arc != arc {
					continue
				}
				x := loftFaceInterval{
					lo: lfAdd(lfMul(lfSub(lfOne(), a.time.lo), whole.x[0]),
						lfMul(a.time.lo, whole.x[1])),
					hi: lfAdd(lfMul(lfSub(lfOne(), a.time.hi), whole.x[0]),
						lfMul(a.time.hi, whole.x[1])),
				}
				if x.lo.Cmp(upper.circle.x) <= 0 ||
					x.hi.Cmp(lfF(meridian.end.U)) >= 0 ||
					terminal != nil {
					return nil, false, nil
				}
				terminal = &revolveLoftFacesHeelTerminal{
					upper: upper, meridian: meridian, arc: arc,
					ends:  [2]*sketch.Point{first.vertex.point, second.vertex.point},
					lines: [2]int{left, right},
					root:  a.time, rootX: x, endTime: endTime,
					capFace: proof.heelFace,
				}
			}
		}
	}
	if terminal == nil || !revolveLoftFacesHeelTerminalCells(
		proof.trim, loop, cells, terminal) {
		return nil, false, nil
	}
	upperPoint, pointOK := revolveLoftFacesHeelRegionSliceAt(proof.trim, loop,
		meridian, cells, upper.band, upper.time, upper.time, true)
	before, beforeOK := revolveLoftFacesHeelRegionSliceAt(proof.trim, loop,
		meridian, cells, upper.band, upper.time, terminal.root.lo, false)
	if !pointOK || !beforeOK ||
		!revolveLoftFacesHeelRegionArcEnds(upperPoint, upper) ||
		!revolveLoftFacesHeelRegionLinked(upperPoint, before) ||
		!revolveLoftFacesHeelTerminalBefore(before, terminal) ||
		!revolveLoftFacesHeelTerminalPositiveY(proof.trim, terminal) {
		return nil, false, nil
	}
	terminal.before = before
	return terminal, true, nil
}

// The authenticated full Revolve meridian has no source point beyond this
// live vertical contact circle. The cap circle itself remains owned by the
// existing occupied-arc record.
func revolveLoftFacesHeelTerminalBlankMiss(proof *revolveLoftFacesSourceCertificate,
	upper *revolveLoftFacesToolCapOccupiedArc) bool {
	if proof == nil || upper == nil || upper.circle.x == nil ||
		upper.circle.span < 0 || upper.circle.span >= len(proof.section.spans) ||
		!revolveLoftFacesSectionCurrent(proof) {
		return false
	}
	vertical := proof.section.spans[upper.circle.span]
	if vertical.face != upper.circle.blankFace ||
		vertical.start.U != vertical.end.U ||
		lfF(vertical.start.U).Cmp(upper.circle.x) != 0 {
		return false
	}
	for _, span := range proof.section.spans {
		if lfF(span.start.U).Cmp(upper.circle.x) > 0 ||
			lfF(span.end.U).Cmp(upper.circle.x) > 0 {
			return false
		}
	}
	return true
}

// Search a bounded dyadic grid for a float-exact witness on the live heel
// meridian inside the cap band after the upper contact circle.
func revolveLoftFacesHeelTerminalMeridian(trim *loftFacesTrimRecord,
	upper *revolveLoftFacesToolCapOccupiedArc) (revolveSectionSpan, *big.Rat, bool) {
	var empty revolveSectionSpan
	if trim == nil || trim.heel == nil || trim.heel.apex == nil ||
		trim.heel.slope == nil || trim.heel.slope.Sign() <= 0 ||
		upper == nil || upper.circle.x == nil || upper.circle.radius == nil ||
		upper.band < 0 || upper.band+1 >= len(trim.maps) ||
		!revolveLoftFacesHeelRegionAffine(trim, upper.band) {
		return empty, nil, false
	}
	x0, ok0 := revolveLoftFacesConstantX(trim.maps[upper.band])
	x1, ok1 := revolveLoftFacesConstantX(trim.maps[upper.band+1])
	if !ok0 || !ok1 || x0.Cmp(upper.circle.x) >= 0 ||
		upper.circle.x.Cmp(x1) >= 0 || x1.Cmp(trim.heel.apex) >= 0 {
		return empty, nil, false
	}
	startX, startR, exact := revolveLoftFacesExactFloatPoint(
		upper.circle.x, upper.circle.radius)
	if !exact {
		return empty, nil, false
	}
	distance := lfSub(trim.heel.apex, upper.circle.x)
	for power := uint(1); power <= 12; power++ {
		denominator := int64(1) << power
		scaled := lfMul(lfDiv(lfSub(x1, upper.circle.x), distance),
			big.NewRat(denominator, 1))
		numerator := new(big.Int).Quo(scaled.Num(), scaled.Denom()).Int64()
		if numerator <= 0 || numerator >= denominator {
			continue
		}
		x := lfAdd(upper.circle.x, lfMul(distance,
			big.NewRat(numerator, denominator)))
		if x.Cmp(x1) >= 0 {
			continue
		}
		r := lfDiv(lfSub(trim.heel.apex, x), trim.heel.slope)
		endX, endR, exact := revolveLoftFacesExactFloatPoint(x, r)
		if !exact || endX <= startX || endR <= 0 || endR >= startR {
			continue
		}
		section := revolveSectionSpan{
			start: Point2{U: startX, V: startR},
			end:   Point2{U: endX, V: endR},
		}
		m, b := revolveLoftFacesMeridian(section)
		if m.Cmp(lfDiv(big.NewRat(-1, 1), trim.heel.slope)) != 0 ||
			b.Cmp(lfDiv(trim.heel.apex, trim.heel.slope)) != 0 {
			continue
		}
		time := lfDiv(lfSub(x, x0), lfSub(x1, x0))
		if time.Cmp(upper.time) <= 0 || time.Cmp(lfOne()) >= 0 {
			continue
		}
		return section, time, true
	}
	return empty, nil, false
}

// At the isolated root bracket, exactly two live connector cells and one
// whole radial Arc touch the heel circle; all other source spans are strict
// outside. On the following open slab every source span is strict outside.
func revolveLoftFacesHeelTerminalCells(trim *loftFacesTrimRecord,
	loop *revolveLoftFacesSimpleLoop, cells []revolveLoftFacesMappedCell,
	terminal *revolveLoftFacesHeelTerminal) bool {
	if trim == nil || loop == nil || terminal == nil || terminal.root.lo == nil ||
		terminal.root.hi == nil || terminal.endTime == nil {
		return false
	}
	contacts := [3]bool{}
	for _, phase := range [...]struct {
		lo, hi *big.Rat
		root   bool
	}{{terminal.root.lo, terminal.root.hi, true},
		{terminal.root.hi, terminal.endTime, false}} {
		seen := make(map[revolveLoftFacesHeelSourceKey]struct{}, len(loop.cells))
		for _, source := range loop.cells {
			key := revolveLoftFacesHeelSourceKey{source.segment, source.span}
			if source.arc != nil {
				key.span = -1
			}
			if _, duplicate := seen[key]; duplicate {
				return false
			}
			seen[key] = struct{}{}
			var parts []revolveLoftFacesMappedCell
			for _, cell := range cells {
				if cell.segment != key.segment || cell.span != key.span ||
					cell.band != terminal.upper.band || cell.tLo == nil || cell.tHi == nil ||
					cell.sLo == nil || cell.sHi == nil ||
					cell.tLo.Cmp(phase.lo) > 0 || cell.tHi.Cmp(phase.hi) < 0 {
					continue
				}
				parts = append(parts, cell)
			}
			sort.Slice(parts, func(i, j int) bool {
				return parts[i].sLo.Cmp(parts[j].sLo) < 0
			})
			end := lfZero()
			for _, part := range parts {
				if part.sLo == nil || part.sHi == nil ||
					part.sLo.Cmp(end) != 0 || part.sHi.Cmp(part.sLo) <= 0 ||
					part.sHi.Cmp(lfOne()) > 0 {
					return false
				}
				end = part.sHi
				if !phase.root {
					if part.side != revolveLoftFacesOutside ||
						part.tHi.Cmp(phase.hi) == 0 && part.tHiOpen {
						return false
					}
					continue
				}
				switch part.side {
				case revolveLoftFacesOutside:
					if part.tLo.Cmp(phase.lo) == 0 && part.tLoOpen ||
						part.tHi.Cmp(phase.hi) == 0 && part.tHiOpen {
						return false
					}
				case revolveLoftFacesJointContact:
					if part.tLo.Cmp(terminal.root.lo) != 0 ||
						part.tHi.Cmp(terminal.root.hi) != 0 ||
						part.tLoOpen || part.tHiOpen {
						return false
					}
					if part.jointKind == revolveLoftFacesWholeArcContact &&
						part.jointArc == terminal.arc && source.arc != nil {
						if contacts[2] {
							return false
						}
						contacts[2] = true
					} else if part.jointKind == revolveLoftFacesLineArcContact &&
						part.jointArc == terminal.arc && source.arc == nil {
						index := -1
						for i := range terminal.ends {
							if part.jointPoint == terminal.ends[i] &&
								source.segment == terminal.lines[i] {
								index = i
							}
						}
						if index < 0 || contacts[index] {
							return false
						}
						contacts[index] = true
					} else {
						return false
					}
				default:
					return false
				}
			}
			if end.Cmp(lfOne()) != 0 {
				return false
			}
		}
	}
	return contacts[0] && contacts[1] && contacts[2]
}

func revolveLoftFacesHeelTerminalBefore(before revolveLoftFacesHeelRegionSlice,
	terminal *revolveLoftFacesHeelTerminal) bool {
	if terminal == nil || before.point || before.band != terminal.upper.band ||
		before.lo.Cmp(terminal.upper.time) != 0 ||
		before.hi.Cmp(terminal.root.lo) != 0 {
		return false
	}
	seen := [2]bool{}
	for _, event := range before.events {
		if event.point != nil || len(event.members) != 1 ||
			event.members[0].side != revolveLoftFacesCrossed {
			return false
		}
		matched := false
		for index, segment := range terminal.lines {
			if event.members[0].segment == segment && !seen[index] {
				seen[index] = true
				matched = true
			}
		}
		if !matched {
			return false
		}
	}
	return seen[0] && seen[1]
}

func revolveLoftFacesHeelTerminalPositiveY(trim *loftFacesTrimRecord,
	terminal *revolveLoftFacesHeelTerminal) bool {
	if trim == nil || terminal == nil || !terminal.upper.throughPositiveY {
		return false
	}
	box, err := lfSourceBox(trim.source)
	if err != nil {
		return false
	}
	corners := lfBoxCorners(box)
	yAxis := loftFaceRatVec{lfZero(), lfOne(), lfZero()}
	first := lfScalarRange(trim.maps[terminal.upper.band], yAxis, corners)
	last := lfScalarRange(trim.maps[terminal.upper.band+1], yAxis, corners)
	for _, time := range [...](*big.Rat){terminal.upper.time, terminal.endTime} {
		if mappedArcBlend(first, last, time).lo.Sign() <= 0 {
			return false
		}
	}
	return true
}
