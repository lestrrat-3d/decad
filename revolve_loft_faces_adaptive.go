package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
)

const (
	revolveLoftFacesMaxBandDepth = 12
	revolveLoftFacesMaxBandCells = 256
	revolveLoftFacesAxialMiss    = 3
	revolveLoftFacesJointContact = 4
)

// Each accepted time slab has either one strict side or one moving source
// root. For the latter, strict left/right boxes and a positive source
// derivative in the middle prove uniqueness over every time in the slab.
// Dyadic cuts are only proof cells; they do not move any source or held point.
func revolveLoftFacesAdaptiveBezierCells(ctx context.Context, trim *loftFacesTrimRecord,
	m, b *big.Rat, segment, spanIndex, band int, span freeform.BezierSpan,
	section revolveSectionSpan) (
	[]revolveLoftFacesMappedCell, bool, error,
) {
	if ctx == nil || trim == nil || band < 0 || band+1 >= len(trim.maps) {
		return nil, false, nil
	}
	first, last := trim.maps[band], trim.maps[band+1]
	q := spiralRootPolynomial(first, last, span, m, b)
	j := spiralRootXOrder(first, last, span, q)
	remaining := revolveLoftFacesMaxBandCells
	return revolveLoftFacesTimeSlab(ctx, q, j, first, last, span, section, segment, spanIndex, band,
		lfZero(), lfOne(), 0, &remaining)
}

func revolveLoftFacesTimeSlab(ctx context.Context, q, j [3][]*big.Rat,
	first, last loftFaceMap, sourceSpan freeform.BezierSpan, section revolveSectionSpan,
	segment, span, band int, tLo, tHi *big.Rat, depth int,
	remaining *int) ([]revolveLoftFacesMappedCell, bool, error) {
	if *remaining <= 0 {
		return nil, false, fmt.Errorf("adaptive Bernstein time-cell budget exhausted at t=[%s,%s]", tLo, tHi)
	}
	*remaining--
	if *remaining%32 == 0 {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
	}
	box := revolveLoftFacesRestrictBernstein(q, lfZero(), lfOne(), tLo, tHi)
	kind, _ := revolveLoftFacesClassifyRows(box)
	whole := revolveLoftFacesMappedCell{segment: segment, span: span,
		band: band, side: kind, sLo: lfZero(), sHi: lfOne(), tLo: tLo, tHi: tHi}
	if kind == revolveLoftFacesInside || kind == revolveLoftFacesOutside {
		return []revolveLoftFacesMappedCell{whole}, true, nil
	}
	if revolveLoftFacesSpanAxialMiss(first, last, sourceSpan, section,
		lfZero(), lfOne(), tLo, tHi) {
		whole.side = revolveLoftFacesAxialMiss
		return []revolveLoftFacesMappedCell{whole}, true, nil
	}
	if cells, ok := revolveLoftFacesBracketTimeRoot(q, j,
		segment, span, band, tLo, tHi); ok {
		return cells, true, nil
	}
	if depth >= revolveLoftFacesMaxBandDepth {
		low, high := spiralRootAtBandT(q, tLo), spiralRootAtBandT(q, tHi)
		return nil, false, fmt.Errorf("adaptive Bernstein time depth exhausted at t=[%s,%s], endpoint signs %d/%d to %d/%d",
			tLo, tHi, low[0].Sign(), low[len(low)-1].Sign(),
			high[0].Sign(), high[len(high)-1].Sign())
	}
	mid := lfMul(lfAdd(tLo, tHi), big.NewRat(1, 2))
	left, ok, err := revolveLoftFacesTimeSlab(ctx, q, j, first, last, sourceSpan, section,
		segment, span, band,
		tLo, mid, depth+1, remaining)
	if err != nil || !ok {
		return nil, false, err
	}
	right, ok, err := revolveLoftFacesTimeSlab(ctx, q, j, first, last, sourceSpan, section,
		segment, span, band,
		mid, tHi, depth+1, remaining)
	return append(left, right...), ok, err
}

// A Bernstein rectangle with nonnegative controls is strictly positive for
// every interior band time when each source column has a positive control.
// Closed time endpoints additionally require every source control in that
// endpoint row to be strict. The root bracket owns excluded endpoints.
func revolveLoftFacesOpenStrictSide(q [3][]*big.Rat,
	tLo, tHi *big.Rat, loOpen, hiOpen bool) int {
	if tLo == nil || tHi == nil || tLo.Cmp(tHi) >= 0 || len(q[0]) == 0 ||
		len(q[1]) != len(q[0]) || len(q[2]) != len(q[0]) {
		return 0
	}
	box := revolveLoftFacesRestrictBernstein(q, lfZero(), lfOne(), tLo, tHi)
	for _, side := range [...]int{1, -1} {
		valid := true
		for col := range box[0] {
			positive := false
			for row := range box {
				sign := box[row][col].Sign() * side
				if sign < 0 || row == 0 && !loOpen && sign == 0 ||
					row == 2 && !hiOpen && sign == 0 {
					valid = false
					break
				}
				positive = positive || sign > 0
			}
			if !valid || !positive {
				valid = false
				break
			}
		}
		if valid {
			return side
		}
	}
	return 0
}

func revolveLoftFacesOpenTimeSlab(ctx context.Context, q, j [3][]*big.Rat,
	first, last loftFaceMap, span freeform.BezierSpan, section revolveSectionSpan,
	segment, spanIndex, band int, tLo, tHi *big.Rat,
	loOpen, hiOpen bool, remaining *int) ([]revolveLoftFacesMappedCell, bool, error) {
	if ctx == nil || remaining == nil || *remaining <= 0 {
		return nil, false, nil
	}
	if side := revolveLoftFacesOpenStrictSide(q, tLo, tHi, loOpen, hiOpen); side != 0 {
		*remaining--
		return []revolveLoftFacesMappedCell{{segment: segment, span: spanIndex,
			band: band, side: side, sLo: lfZero(), sHi: lfOne(),
			tLo: tLo, tHi: tHi, tLoOpen: loOpen, tHiOpen: hiOpen}}, true, nil
	}
	cells, ok, err := revolveLoftFacesTimeSlab(ctx, q, j, first, last, span,
		section, segment, spanIndex, band, tLo, tHi, 0, remaining)
	for i := range cells {
		cells[i].tLoOpen = loOpen && cells[i].tLo.Cmp(tLo) == 0
		cells[i].tHiOpen = hiOpen && cells[i].tHi.Cmp(tHi) == 0
	}
	return cells, ok, err
}

func revolveLoftFacesBracketTimeRoot(q, j [3][]*big.Rat,
	segment, span, band int, tLo, tHi *big.Rat) ([]revolveLoftFacesMappedCell, bool) {
	box := revolveLoftFacesRestrictBernstein(q, lfZero(), lfOne(), tLo, tHi)
	for _, orientation := range []int{1, -1} {
		oriented := box
		if orientation < 0 {
			oriented = revolveLoftFacesNegateRows(box)
		}
		if revolveLoftFacesTimePointSign(oriented, lfZero()) != -1 ||
			revolveLoftFacesTimePointSign(oriented, lfOne()) != 1 {
			continue
		}
		sLo, sHi := lfZero(), lfOne()
		for range 10 {
			mid := lfMul(lfAdd(sLo, sHi), big.NewRat(1, 2))
			switch revolveLoftFacesTimePointSign(oriented, mid) {
			case -1:
				sLo = mid
			case 1:
				sHi = mid
			default:
				// The root moves across this dyadic source point during
				// the time slab. Keep the wider bracket for the exact test.
				goto check
			}
		}
	check:
		middle := revolveLoftFacesRestrictBernstein(q, sLo, sHi, tLo, tHi)
		if orientation < 0 {
			middle = revolveLoftFacesNegateRows(middle)
		}
		if !spiralCrossingSigns(middle) {
			continue
		}
		if sLo.Sign() > 0 {
			left := revolveLoftFacesRestrictBernstein(q, lfZero(), sLo, tLo, tHi)
			if orientation > 0 {
				left = revolveLoftFacesNegateRows(left)
			}
			if !spiralAllPositive(left) {
				continue
			}
		}
		if sHi.Cmp(lfOne()) < 0 {
			right := revolveLoftFacesRestrictBernstein(q, sHi, lfOne(), tLo, tHi)
			if orientation < 0 {
				right = revolveLoftFacesNegateRows(right)
			}
			if !spiralAllPositive(right) {
				continue
			}
		}
		order := revolveLoftFacesRestrictBernstein(j, sLo, sHi, tLo, tHi)
		if orientation < 0 {
			order = revolveLoftFacesNegateRows(order)
		}
		xDirection := 0
		if spiralAllPositive(order) {
			xDirection = 1
		} else if spiralAllPositive(revolveLoftFacesNegateRows(order)) {
			xDirection = -1
		}
		if xDirection == 0 {
			continue
		}
		makeCell := func(side int, a, z *big.Rat) revolveLoftFacesMappedCell {
			return revolveLoftFacesMappedCell{segment: segment, span: span,
				band: band, side: side, sLo: a, sHi: z, tLo: tLo, tHi: tHi}
		}
		var cells []revolveLoftFacesMappedCell
		if sLo.Sign() > 0 {
			cells = append(cells, makeCell(-orientation, lfZero(), sLo))
		}
		crossing := makeCell(revolveLoftFacesCrossed, sLo, sHi)
		crossing.xDirection = xDirection
		cells = append(cells, crossing)
		if sHi.Cmp(lfOne()) < 0 {
			cells = append(cells, makeCell(orientation, sHi, lfOne()))
		}
		return cells, true
	}
	return nil, false
}

// A strict sign at fixed source s for all times follows from the exact
// quadratic Bernstein coefficients in band time.
func revolveLoftFacesTimePointSign(q [3][]*big.Rat, s *big.Rat) int {
	negative, positive := true, true
	for _, row := range q {
		sign := spiralBernSign(row, s)
		negative = negative && sign < 0
		positive = positive && sign > 0
	}
	if negative {
		return -1
	}
	if positive {
		return 1
	}
	return 0
}

func revolveLoftFacesRestrictBernstein(rows [3][]*big.Rat,
	sLo, sHi, tLo, tHi *big.Rat) [3][]*big.Rat {
	var restricted [3][]*big.Rat
	for row := range restricted {
		restricted[row] = make([]*big.Rat, len(rows[row]))
	}
	for column := range rows[0] {
		coeff := spiralBernRestrict([]*big.Rat{rows[0][column], rows[1][column],
			rows[2][column]}, tLo, tHi)
		for row := range restricted {
			restricted[row][column] = coeff[row]
		}
	}
	for row := range restricted {
		restricted[row] = spiralBernRestrict(restricted[row], sLo, sHi)
	}
	return restricted
}
