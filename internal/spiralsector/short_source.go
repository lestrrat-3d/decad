package spiralsector

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// Each record proves one complete source contour cell has exactly one
// transverse intersection with a finite blank cap generator. The remaining
// output-chain proof compares its exposed source arc with the held mesh.
type ShortCapCrossing struct {
	Span freeform.BezierSpan
	Q    [3][]*big.Rat
	TLo  *big.Rat
	X    loftFaceInterval
	Box  [3]loftFaceInterval
}

type ShortCapFamily struct {
	Band       int
	Group      int
	Pieces     []ShortCapCrossing
	X          loftFaceInterval
	FiniteClip facetedrecord.SpiralJunction
	FiniteEnd  int
}

func ShortCapSpan(source *facetedrecord.PointSectionRecord, groups []int,
	group int) (freeform.BezierSpan, bool) {
	if source == nil || len(source.SourceStation) != len(groups) {
		return nil, false
	}
	index := -1
	for j, id := range groups {
		if id == group {
			if index >= 0 {
				return nil, false
			}
			index = j
		}
	}
	if index < 0 {
		return nil, false
	}
	a := source.SourceStation[index]
	z := source.SourceStation[(index+1)%len(groups)]
	line, ok := source.Profile.Outer.Segments[group].(sectionrecord.LineSeg)
	if !ok || a != line.Start && z != line.Start || a != line.End && z != line.End {
		return nil, false
	}
	return freeform.BezierSpan{{U: seamRatF(a.U), V: seamRatF(a.V)},
		{U: seamRatF(z.U), V: seamRatF(z.V)}}, true
}

func ShortCapCrossingProof(first, second loftFaceMap, span freeform.BezierSpan,
	m, b, capStart, capEnd *big.Rat, requireFullFinite bool) (ShortCapCrossing, bool) {
	if len(span) < 2 || m == nil || b == nil || capStart == nil || capEnd == nil ||
		capStart.Cmp(capEnd) >= 0 {
		return ShortCapCrossing{}, false
	}
	q := loftface.MeridianQRows(first, second, span, m, b)
	for k := range q[0] {
		if q[0][k].Sign() >= 0 || q[2][k].Sign() <= 0 ||
			shortRatSub(q[1][k], q[0][k]).Sign() <= 0 ||
			shortRatSub(q[2][k], q[1][k]).Sign() <= 0 {
			return ShortCapCrossing{}, false
		}
	}
	// The crossing root lies in [tLo, 1] when Q is negative at tLo. The nearest
	// tLo to the span end that holds is used, since a narrower bracket gives the
	// X range and boxes that follow less to cover.
	var tLo *big.Rat
	for sixteenths := int64(15); sixteenths >= 1; sixteenths-- {
		candidate := big.NewRat(sixteenths, 16)
		oneMinus := shortRatSub(shortRatOne(), candidate)
		holds := true
		for k := range q[0] {
			value := shortRatAdd(shortRatAdd(shortRatMul(shortRatMul(oneMinus, oneMinus), q[0][k]),
				shortRatMul(shortRatMul(shortRatMul(big.NewRat(2, 1), oneMinus), candidate), q[1][k])),
				shortRatMul(shortRatMul(candidate, candidate), q[2][k]))
			if value.Sign() >= 0 {
				holds = false
				break
			}
		}
		if holds {
			tLo = candidate
			break
		}
	}
	if tLo == nil {
		return ShortCapCrossing{}, false
	}
	x, ok := ShortCapXRange(first, second, span, tLo)
	if !ok || requireFullFinite && (x.Lo.Cmp(capStart) <= 0 || x.Hi.Cmp(capEnd) >= 0) ||
		shortRatAdd(shortRatMul(m, x.Lo), b).Sign() <= 0 ||
		shortRatAdd(shortRatMul(m, x.Hi), b).Sign() <= 0 {
		return ShortCapCrossing{}, false
	}
	certificate := ShortCapCrossing{Span: span, Q: q, TLo: tLo, X: x}
	if len(span) == 2 {
		t, ok := ShortCapTBracket(q, tLo)
		if !ok {
			return ShortCapCrossing{}, false
		}
		certificate.Box, ok = ShortLineBox(first, second, span, t)
		if !ok {
			return ShortCapCrossing{}, false
		}
	}
	return certificate, true
}
