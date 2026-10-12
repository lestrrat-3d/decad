package decad

import (
	"context"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/sketch"
)

// A radial tip Arc first reaches the heel cone at one exact time root. Before
// that root, the whole live source contour is strictly inside the cone circle.
// Two fitted flanks leave the Arc endpoints and bound the occupied cap patch
// up to the first authenticated blank contact circle. This is source-only.
type revolveLoftFacesHeelBirth struct {
	lower    *revolveLoftFacesToolCapOccupiedArc
	meridian revolveSectionSpan
	arc      *sketch.Arc
	ends     [2]*sketch.Point
	fits     [2]int
	root     loftFaceInterval
	rootX    loftFaceInterval
	slices   []revolveLoftFacesHeelRegionSlice
	capFace  *Face
}

func certifyRevolveLoftFacesHeelBirth(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate) (
	*revolveLoftFacesHeelBirth, bool, error,
) {
	if ctx == nil || proof == nil || proof.trim == nil || proof.trim.heel == nil ||
		!revolveLoftFacesSectionCurrent(proof) {
		return nil, false, nil
	}
	region, ok, err := certifyRevolveLoftFacesHeelRegion(ctx, proof)
	if err != nil || !ok || region == nil || region.lower == nil || region.upper == nil {
		return nil, false, err
	}
	lower := region.lower
	if !revolveLoftFacesHeelBirthBlankMiss(ctx, proof, lower) {
		return nil, false, nil
	}
	meridian, ok := revolveLoftFacesHeelBirthMeridian(proof.trim.heel, region.upper)
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
	var birth *revolveLoftFacesHeelBirth
	entities := proof.trim.source.source.entities
	for arcIndex, entity := range entities {
		arc, isArc := entity.(*sketch.Arc)
		if !isArc {
			continue
		}
		left := (arcIndex - 1 + len(entities)) % len(entities)
		right := (arcIndex + 1) % len(entities)
		if _, ok := entities[left].(*sketch.FitSpline); !ok {
			continue
		}
		if _, ok := entities[right].(*sketch.FitSpline); !ok {
			continue
		}
		first, firstOK := certifyRevolveLoftFacesRadialJoint(ctx, proof.trim,
			meridian, left, arcIndex, lower.band, true)
		second, secondOK := certifyRevolveLoftFacesRadialJoint(ctx, proof.trim,
			meridian, right, arcIndex, lower.band, false)
		if !firstOK || !secondOK || first.radial.arc != arc || second.radial.arc != arc ||
			first.vertex.point == second.vertex.point {
			continue
		}
		for _, a := range first.roots {
			for _, b := range second.roots {
				if !a.finite || !b.finite || a.time.lo.Cmp(b.time.lo) != 0 ||
					a.time.hi.Cmp(b.time.hi) != 0 || a.time.hi.Cmp(lower.time) >= 0 ||
					a.time.lo.Sign() <= 0 {
					continue
				}
				whole, radialOK := mappedArcRadialTimeBand(ctx, proof.trim,
					arcIndex, lower.band, meridian)
				if !radialOK || whole.arc != arc {
					continue
				}
				x := loftFaceInterval{
					lo: lfAdd(lfMul(lfSub(lfOne(), a.time.lo), whole.x[0]),
						lfMul(a.time.lo, whole.x[1])),
					hi: lfAdd(lfMul(lfSub(lfOne(), a.time.hi), whole.x[0]),
						lfMul(a.time.hi, whole.x[1])),
				}
				if x.lo.Cmp(lfF(meridian.start.U)) <= 0 ||
					x.hi.Cmp(lower.circle.x) >= 0 || birth != nil {
					return nil, false, nil
				}
				birth = &revolveLoftFacesHeelBirth{lower: lower, meridian: meridian,
					arc: arc, ends: [2]*sketch.Point{first.vertex.point, second.vertex.point},
					fits: [2]int{left, right}, root: a.time, rootX: x,
					capFace: proof.heelFace}
			}
		}
	}
	if birth == nil || !revolveLoftFacesHeelRegionTrimOpen(proof.trim,
		birth.rootX.lo, lower.circle.x) ||
		!revolveLoftFacesHeelBirthCells(proof.trim, loop, cells, birth) ||
		!revolveLoftFacesHeelBirthPositiveY(proof.trim, birth) {
		return nil, false, nil
	}
	slices, ok := revolveLoftFacesHeelBirthSlices(proof.trim, loop, cells, birth)
	if !ok {
		return nil, false, nil
	}
	birth.slices = slices
	return birth, true, nil
}

func revolveLoftFacesHeelBirthMeridian(heel *pointConeLimit,
	upper *revolveLoftFacesToolCapOccupiedArc) (revolveSectionSpan, bool) {
	var empty revolveSectionSpan
	if heel == nil || heel.apex == nil || heel.far == nil || heel.slope == nil ||
		heel.slope.Sign() <= 0 || upper == nil || upper.circle.x == nil ||
		upper.circle.radius == nil || heel.far.Cmp(upper.circle.x) >= 0 {
		return empty, false
	}
	startR := lfDiv(lfSub(heel.apex, heel.far), heel.slope)
	startX, startRadius, first := revolveLoftFacesExactFloatPoint(heel.far, startR)
	endX, endRadius, second := revolveLoftFacesExactFloatPoint(
		upper.circle.x, upper.circle.radius)
	if !first || !second || startX >= endX || startRadius <= endRadius {
		return empty, false
	}
	section := revolveSectionSpan{start: Point2{U: startX, V: startRadius},
		end: Point2{U: endX, V: endRadius}}
	m, b := revolveLoftFacesMeridian(section)
	return section, m.Cmp(lfDiv(big.NewRat(-1, 1), heel.slope)) == 0 &&
		b.Cmp(lfDiv(heel.apex, heel.slope)) == 0
}

// The complete live blank section lies strictly below the heel cone before
// the lower contact. Linear section spans and the cone half-plane make the
// endpoint checks sufficient for every point in the Revolve solid.
func revolveLoftFacesHeelBirthBlankMiss(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	lower *revolveLoftFacesToolCapOccupiedArc) bool {
	if ctx == nil || proof == nil || proof.trim == nil || proof.trim.heel == nil ||
		lower == nil || lower.circle.x == nil || lower.circle.radius == nil ||
		lower.circle.span < 0 || lower.circle.span >= len(proof.section.spans) ||
		!revolveLoftFacesSectionCurrent(proof) {
		return false
	}
	current, err := revolveLoftFacesToolCapOccupiedArcCurrent(ctx, proof, lower)
	if err != nil || !current {
		return false
	}
	heel := proof.trim.heel
	if heel.apex == nil || heel.slope == nil || heel.slope.Sign() <= 0 ||
		heel.far == nil || heel.far.Cmp(lower.circle.x) >= 0 ||
		lfDiv(lfSub(heel.apex, lower.circle.x), heel.slope).Cmp(lower.circle.radius) != 0 {
		return false
	}
	contact := 0
	for index, span := range proof.section.spans {
		for _, vertex := range [...]Point2{span.start, span.end} {
			x := lfF(vertex.U)
			if x.Cmp(lower.circle.x) == 0 {
				if lfF(vertex.V).Cmp(lower.circle.radius) >= 0 {
					return false // this bounded class requires an interior side contact
				}
				continue
			}
			if x.Cmp(lower.circle.x) > 0 {
				continue
			}
			cone := lfDiv(lfSub(heel.apex, x), heel.slope)
			if cone.Sign() <= 0 || lfF(vertex.V).Cmp(cone) >= 0 {
				return false
			}
		}
		x0, x1 := lfF(span.start.U), lfF(span.end.U)
		if (x0.Cmp(lower.circle.x) < 0 && x1.Cmp(lower.circle.x) > 0) ||
			(x1.Cmp(lower.circle.x) < 0 && x0.Cmp(lower.circle.x) > 0) {
			fraction := lfDiv(lfSub(lower.circle.x, x0), lfSub(x1, x0))
			radius := lfAdd(lfF(span.start.V), lfMul(fraction,
				lfSub(lfF(span.end.V), lfF(span.start.V))))
			comparison := radius.Cmp(lower.circle.radius)
			if comparison > 0 || comparison == 0 && index != lower.circle.span {
				return false
			}
			if comparison == 0 {
				contact++
			}
		}
	}
	return contact == 1
}

func revolveLoftFacesHeelBirthCells(trim *loftFacesTrimRecord,
	loop *revolveLoftFacesSimpleLoop, cells []revolveLoftFacesMappedCell,
	birth *revolveLoftFacesHeelBirth) bool {
	if trim == nil || loop == nil || birth == nil || birth.lower == nil ||
		birth.root.lo == nil || birth.root.hi == nil ||
		birth.root.lo.Cmp(birth.root.hi) >= 0 {
		return false
	}
	for band := 0; band <= birth.lower.band; band++ {
		hi := lfOne()
		if band == birth.lower.band {
			hi = birth.root.lo
		}
		if !revolveLoftFacesHeelBirthWholeSide(loop, cells, band,
			lfZero(), hi, revolveLoftFacesInside) {
			return false
		}
	}
	seen := [3]bool{}
	for _, source := range loop.cells {
		key := revolveLoftFacesHeelSourceKey{source.segment, source.span}
		if source.arc != nil {
			key.span = -1
		}
		parts := revolveLoftFacesHeelBirthParts(cells, key,
			birth.lower.band, birth.root.lo, birth.root.hi)
		if !revolveLoftFacesHeelBirthSourceCover(parts, birth.root.lo, birth.root.hi) {
			return false
		}
		for _, part := range parts {
			if part.side == revolveLoftFacesInside {
				continue
			}
			if part.side != revolveLoftFacesJointContact ||
				part.tLo.Cmp(birth.root.lo) != 0 ||
				part.tHi.Cmp(birth.root.hi) != 0 || part.tLoOpen || part.tHiOpen ||
				source.arc != nil && part.jointArc != birth.arc {
				return false
			}
			index := 2
			if source.arc == nil {
				index = -1
				for i := range birth.fits {
					if source.segment == birth.fits[i] &&
						part.jointPoint == birth.ends[i] &&
						part.jointKind == revolveLoftFacesRadialArcContact {
						index = i
					}
				}
			} else if part.jointKind != revolveLoftFacesWholeArcContact {
				return false
			}
			if index < 0 || seen[index] {
				return false
			}
			seen[index] = true
		}
	}
	return seen[0] && seen[1] && seen[2] &&
		revolveLoftFacesHeelBirthRootPoint(loop, cells, birth, birth.root.lo) &&
		revolveLoftFacesHeelBirthRootPoint(loop, cells, birth, birth.root.hi)
}

// The strict cells adjoining a root bracket may have open time ends. Check
// each rational bracket endpoint against the cells that actually own it.
func revolveLoftFacesHeelBirthRootPoint(loop *revolveLoftFacesSimpleLoop,
	cells []revolveLoftFacesMappedCell, birth *revolveLoftFacesHeelBirth,
	time *big.Rat) bool {
	if loop == nil || birth == nil || time == nil {
		return false
	}
	seen := [3]bool{}
	for _, source := range loop.cells {
		key := revolveLoftFacesHeelSourceKey{source.segment, source.span}
		if source.arc != nil {
			key.span = -1
		}
		var parts []revolveLoftFacesMappedCell
		for _, cell := range cells {
			if cell.band == birth.lower.band && cell.segment == key.segment &&
				cell.span == key.span && revolveLoftFacesTimeInCell(time, cell) {
				if cell.sLo == nil || cell.sHi == nil {
					return false
				}
				parts = append(parts, cell)
			}
		}
		sort.Slice(parts, func(i, j int) bool { return parts[i].sLo.Cmp(parts[j].sLo) < 0 })
		if !revolveLoftFacesHeelBirthSourceCover(parts, time, time) {
			return false
		}
		for _, part := range parts {
			if part.side == revolveLoftFacesInside {
				continue
			}
			if part.side != revolveLoftFacesJointContact ||
				part.tLo.Cmp(birth.root.lo) != 0 ||
				part.tHi.Cmp(birth.root.hi) != 0 {
				return false
			}
			index := 2
			if source.arc == nil {
				index = -1
				for i, fit := range birth.fits {
					if part.segment == fit && part.jointPoint == birth.ends[i] &&
						part.jointKind == revolveLoftFacesRadialArcContact {
						index = i
					}
				}
			} else if part.jointKind != revolveLoftFacesWholeArcContact ||
				part.jointArc != birth.arc {
				return false
			}
			if index < 0 || seen[index] {
				return false
			}
			seen[index] = true
		}
	}
	return seen[0] && seen[1] && seen[2]
}

func revolveLoftFacesHeelBirthWholeSide(loop *revolveLoftFacesSimpleLoop,
	cells []revolveLoftFacesMappedCell, band int, lo, hi *big.Rat, side int) bool {
	if loop == nil || lo == nil || hi == nil || lo.Cmp(hi) >= 0 {
		return false
	}
	for _, source := range loop.cells {
		key := revolveLoftFacesHeelSourceKey{source.segment, source.span}
		if source.arc != nil {
			key.span = -1
		}
		parts := revolveLoftFacesHeelBirthParts(cells, key, band, lo, hi)
		if !revolveLoftFacesHeelBirthSourceCover(parts, lo, hi) {
			return false
		}
		for _, part := range parts {
			if part.side != side || part.tLo.Cmp(lo) == 0 && part.tLoOpen ||
				part.tHi.Cmp(hi) == 0 && part.tHiOpen && hi.Cmp(lfOne()) == 0 {
				return false
			}
		}
	}
	return true
}

func revolveLoftFacesHeelBirthParts(cells []revolveLoftFacesMappedCell,
	key revolveLoftFacesHeelSourceKey, band int, lo, hi *big.Rat) []revolveLoftFacesMappedCell {
	var parts []revolveLoftFacesMappedCell
	for _, cell := range cells {
		if cell.segment == key.segment && cell.span == key.span && cell.band == band &&
			cell.sLo != nil && cell.sHi != nil && cell.tLo != nil && cell.tHi != nil &&
			cell.tLo.Cmp(lo) <= 0 && cell.tHi.Cmp(hi) >= 0 {
			parts = append(parts, cell)
		}
	}
	sort.Slice(parts, func(i, j int) bool { return parts[i].sLo.Cmp(parts[j].sLo) < 0 })
	return parts
}

func revolveLoftFacesHeelBirthSourceCover(parts []revolveLoftFacesMappedCell,
	lo, hi *big.Rat) bool {
	if len(parts) == 0 || lo == nil || hi == nil {
		return false
	}
	end := lfZero()
	for _, part := range parts {
		if part.sLo.Cmp(end) != 0 || part.sHi.Cmp(part.sLo) <= 0 ||
			part.sHi.Cmp(lfOne()) > 0 || part.tLo.Cmp(lo) > 0 || part.tHi.Cmp(hi) < 0 {
			return false
		}
		end = part.sHi
	}
	return end.Cmp(lfOne()) == 0
}

func revolveLoftFacesHeelBirthPositiveY(trim *loftFacesTrimRecord,
	birth *revolveLoftFacesHeelBirth) bool {
	if trim == nil || birth == nil || !birth.lower.throughPositiveY ||
		birth.lower.band < 0 || birth.lower.band+1 >= len(trim.maps) {
		return false
	}
	box, err := lfSourceBox(trim.source)
	if err != nil {
		return false
	}
	corners := lfBoxCorners(box)
	yAxis := loftFaceRatVec{lfZero(), lfOne(), lfZero()}
	first := lfScalarRange(trim.maps[birth.lower.band], yAxis, corners)
	last := lfScalarRange(trim.maps[birth.lower.band+1], yAxis, corners)
	for _, time := range [...](*big.Rat){birth.root.lo, birth.lower.time} {
		if mappedArcBlend(first, last, time).lo.Sign() <= 0 {
			return false
		}
	}
	return true
}

func revolveLoftFacesHeelBirthSlices(trim *loftFacesTrimRecord,
	loop *revolveLoftFacesSimpleLoop, cells []revolveLoftFacesMappedCell,
	birth *revolveLoftFacesHeelBirth) ([]revolveLoftFacesHeelRegionSlice, bool) {
	band := birth.lower.band
	bounds := []*big.Rat{birth.root.hi, birth.lower.time}
	for _, cell := range cells {
		if cell.band != band || cell.tLo == nil || cell.tHi == nil {
			continue
		}
		for _, value := range [...](*big.Rat){cell.tLo, cell.tHi} {
			if value.Cmp(birth.root.hi) > 0 && value.Cmp(birth.lower.time) < 0 {
				bounds = append(bounds, value)
			}
		}
	}
	sort.Slice(bounds, func(i, j int) bool { return bounds[i].Cmp(bounds[j]) < 0 })
	unique := bounds[:0]
	for _, value := range bounds {
		if len(unique) == 0 || unique[len(unique)-1].Cmp(value) != 0 {
			unique = append(unique, value)
		}
	}
	if len(unique) < 2 || len(unique) > 256 {
		return nil, false
	}
	var slices []revolveLoftFacesHeelRegionSlice
	for index := 0; index+1 < len(unique); index++ {
		open, ok := revolveLoftFacesHeelRegionSliceAt(trim, loop, birth.meridian,
			cells, band, unique[index], unique[index+1], false)
		if !ok {
			return nil, false
		}
		if index == 0 {
			seen := [2]bool{}
			for _, event := range open.events {
				if event.point != nil || len(event.members) != 1 ||
					event.members[0].side != revolveLoftFacesCrossed {
					return nil, false
				}
				for i, fit := range birth.fits {
					rootSpan := -1
					for _, cell := range cells {
						if cell.band == band && cell.segment == fit &&
							cell.side == revolveLoftFacesJointContact &&
							cell.jointKind == revolveLoftFacesRadialArcContact &&
							cell.jointPoint == birth.ends[i] &&
							cell.tLo.Cmp(birth.root.lo) == 0 &&
							cell.tHi.Cmp(birth.root.hi) == 0 {
							rootSpan = cell.span
						}
					}
					if event.members[0].segment == fit &&
						event.members[0].span == rootSpan && rootSpan >= 0 && !seen[i] {
						seen[i] = true
					}
				}
			}
			if !seen[0] || !seen[1] {
				return nil, false
			}
		} else if !revolveLoftFacesHeelRegionLinked(slices[len(slices)-1], open) {
			return nil, false
		}
		slices = append(slices, open)
		point, pointOK := revolveLoftFacesHeelRegionSliceAt(trim, loop,
			birth.meridian, cells, band, unique[index+1], unique[index+1], true)
		if !pointOK || !revolveLoftFacesHeelRegionLinked(open, point) {
			return nil, false
		}
		slices = append(slices, point)
	}
	if len(slices) == 0 || !revolveLoftFacesHeelRegionArcEnds(
		slices[len(slices)-1], birth.lower) {
		return nil, false
	}
	return slices, true
}
