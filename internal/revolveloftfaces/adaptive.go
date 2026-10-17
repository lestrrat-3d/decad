package revolveloftfaces

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

const (
	revolveLoftFacesMaxBandDepth = 12
	RevolveLoftFacesMaxBandCells = 256
	RevolveLoftFacesAxialMiss    = 3
	RevolveLoftFacesJointContact = 4
)

// Each accepted time slab has either one strict side or one moving source
// root. For the latter, strict left/right boxes and a positive source
// derivative in the middle prove uniqueness over every time in the slab.
// Dyadic cuts are only proof cells; they do not move any source or held point.
func revolveLoftFacesAdaptiveBezierCells(ctx context.Context, trim *loftFacesTrimRecord,
	m, b *big.Rat, segment, spanIndex, band int, span freeform.BezierSpan,
	section revolveSectionSpan) (
	[]RevolveLoftFacesMappedCell, bool, error,
) {
	if ctx == nil || trim == nil || band < 0 || band+1 >= len(trim.Maps) {
		return nil, false, nil
	}
	first, last := trim.Maps[band], trim.Maps[band+1]
	q := RevolveLoftFacesMeridianMemoOf(ctx).QRows(first, last, span, m, b)
	j := loftface.MeridianJRows(first, last, span, q)
	remaining := RevolveLoftFacesMaxBandCells
	return revolveLoftFacesTimeSlab(ctx, q, j, first, last, span, section, segment, spanIndex, band,
		new(big.Rat), big.NewRat(1, 1), 0, &remaining)
}

func revolveLoftFacesTimeSlab(ctx context.Context, q, j [3][]*big.Rat,
	first, last loftFaceMap, sourceSpan freeform.BezierSpan, section revolveSectionSpan,
	segment, span, band int, tLo, tHi *big.Rat, depth int,
	remaining *int) ([]RevolveLoftFacesMappedCell, bool, error) {
	if *remaining <= 0 {
		return nil, false, fmt.Errorf("adaptive Bernstein time-cell budget exhausted at t=[%s,%s]", tLo, tHi)
	}
	*remaining--
	if *remaining%32 == 0 {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
	}
	timeRows := polynomial.BernsteinRectTimeRestrict(q, tLo, tHi)
	box := polynomial.BernsteinRectSourceRestrict(timeRows, new(big.Rat), big.NewRat(1, 1))
	kind, _ := loftface.ClassifyRows(box)
	whole := RevolveLoftFacesMappedCell{Segment: segment, Span: span,
		Band: band, Side: kind, SLo: new(big.Rat), SHi: big.NewRat(1, 1), TLo: tLo, THi: tHi}
	if kind == loftface.SideInside || kind == loftface.SideOutside {
		return []RevolveLoftFacesMappedCell{whole}, true, nil
	}
	if RevolveLoftFacesSpanAxialMiss(first, last, sourceSpan, section,
		new(big.Rat), big.NewRat(1, 1), tLo, tHi) {
		whole.Side = RevolveLoftFacesAxialMiss
		return []RevolveLoftFacesMappedCell{whole}, true, nil
	}
	if cells, ok := RevolveLoftFacesBracketTimeRoot(timeRows, j,
		segment, span, band, tLo, tHi); ok {
		return cells, true, nil
	}
	if depth >= revolveLoftFacesMaxBandDepth {
		low, high := polynomial.BernsteinRowsAt(q, tLo), polynomial.BernsteinRowsAt(q, tHi)
		return nil, false, fmt.Errorf("adaptive Bernstein time depth exhausted at t=[%s,%s], endpoint signs %d/%d to %d/%d",
			tLo, tHi, low[0].Sign(), low[len(low)-1].Sign(),
			high[0].Sign(), high[len(high)-1].Sign())
	}
	mid := proofarith.MulRat(new(big.Rat), proofarith.AddRat(new(big.Rat), tLo, tHi), big.NewRat(1, 2))
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

func RevolveLoftFacesOpenTimeSlab(ctx context.Context, q, j [3][]*big.Rat,
	first, last loftFaceMap, span freeform.BezierSpan, section revolveSectionSpan,
	segment, spanIndex, band int, tLo, tHi *big.Rat,
	loOpen, hiOpen bool, remaining *int) ([]RevolveLoftFacesMappedCell, bool, error) {
	if ctx == nil || remaining == nil || *remaining <= 0 {
		return nil, false, nil
	}
	if side := polynomial.BernsteinRectOpenSide(q, tLo, tHi, loOpen, hiOpen); side != 0 {
		*remaining--
		return []RevolveLoftFacesMappedCell{{Segment: segment, Span: spanIndex,
			Band: band, Side: side, SLo: new(big.Rat), SHi: big.NewRat(1, 1),
			TLo: tLo, THi: tHi, TLoOpen: loOpen, THiOpen: hiOpen}}, true, nil
	}
	cells, ok, err := revolveLoftFacesTimeSlab(ctx, q, j, first, last, span,
		section, segment, spanIndex, band, tLo, tHi, 0, remaining)
	for i := range cells {
		cells[i].TLoOpen = loOpen && cells[i].TLo.Cmp(tLo) == 0
		cells[i].THiOpen = hiOpen && cells[i].THi.Cmp(tHi) == 0
	}
	return cells, ok, err
}

// revolveLoftFacesBracketTimeRoot looks for one source bracket where the
// meridian rows cross. timeRows are the meridian rows already restricted to
// band time [tLo, tHi]; j is restricted here.
func RevolveLoftFacesBracketTimeRoot(timeRows, j [3][]*big.Rat,
	segment, span, band int, tLo, tHi *big.Rat) ([]RevolveLoftFacesMappedCell, bool) {
	box := polynomial.BernsteinRectSourceRestrict(timeRows, new(big.Rat), big.NewRat(1, 1))
	for _, orientation := range []int{1, -1} {
		oriented := box
		if orientation < 0 {
			oriented = loftface.NegateRows(box)
		}
		if polynomial.BernsteinRowsPointSign(oriented, new(big.Rat)) != -1 ||
			polynomial.BernsteinRowsPointSign(oriented, big.NewRat(1, 1)) != 1 {
			continue
		}
		sLo, sHi := new(big.Rat), big.NewRat(1, 1)
		for range 10 {
			mid := proofarith.MulRat(new(big.Rat), proofarith.AddRat(new(big.Rat), sLo, sHi), big.NewRat(1, 2))
			switch polynomial.BernsteinRowsPointSign(oriented, mid) {
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
		middle := polynomial.BernsteinRectSourceRestrict(timeRows, sLo, sHi)
		if orientation < 0 {
			middle = loftface.NegateRows(middle)
		}
		if !polynomial.BernsteinRowsCross(middle) {
			continue
		}
		if sLo.Sign() > 0 {
			left := polynomial.BernsteinRectSourceRestrict(timeRows, new(big.Rat), sLo)
			if orientation > 0 {
				left = loftface.NegateRows(left)
			}
			if !polynomial.BernsteinRowsPositive(left) {
				continue
			}
		}
		if sHi.Cmp(big.NewRat(1, 1)) < 0 {
			right := polynomial.BernsteinRectSourceRestrict(timeRows, sHi, big.NewRat(1, 1))
			if orientation < 0 {
				right = loftface.NegateRows(right)
			}
			if !polynomial.BernsteinRowsPositive(right) {
				continue
			}
		}
		order := polynomial.BernsteinRectRestrict(j, sLo, sHi, tLo, tHi)
		if orientation < 0 {
			order = loftface.NegateRows(order)
		}
		xDirection := 0
		if polynomial.BernsteinRowsPositive(order) {
			xDirection = 1
		} else if polynomial.BernsteinRowsPositive(loftface.NegateRows(order)) {
			xDirection = -1
		}
		if xDirection == 0 {
			continue
		}
		makeCell := func(side int, a, z *big.Rat) RevolveLoftFacesMappedCell {
			return RevolveLoftFacesMappedCell{Segment: segment, Span: span,
				Band: band, Side: side, SLo: a, SHi: z, TLo: tLo, THi: tHi}
		}
		var cells []RevolveLoftFacesMappedCell
		if sLo.Sign() > 0 {
			cells = append(cells, makeCell(-orientation, new(big.Rat), sLo))
		}
		crossing := makeCell(loftface.SideCrossed, sLo, sHi)
		crossing.XDirection = xDirection
		cells = append(cells, crossing)
		if sHi.Cmp(big.NewRat(1, 1)) < 0 {
			cells = append(cells, makeCell(orientation, sHi, big.NewRat(1, 1)))
		}
		return cells, true
	}
	return nil, false
}
