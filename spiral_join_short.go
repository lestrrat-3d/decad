package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
)

// Each record proves one complete source contour cell has exactly one
// transverse intersection with a finite blank cap generator. The remaining
// output-chain proof compares its exposed source arc with the held mesh.
type spiralShortCapCrossing struct {
	span freeform.BezierSpan
	q    [3][]*big.Rat
	tLo  *big.Rat
	x    loftFaceInterval
	box  [3]loftFaceInterval
}

type spiralShortCapFamily struct {
	band       int
	group      int
	pieces     []spiralShortCapCrossing
	x          loftFaceInterval
	finiteClip spiralJunction
	finiteEnd  int
}

func spiralShortCapEndpointX(first, second loftFaceMap, span freeform.BezierSpan,
	q [3][]*big.Rat, endpoint int) (loftFaceInterval, bool) {
	if endpoint < 0 || endpoint > 1 || len(span) < 2 || len(q[0]) == 0 {
		return loftFaceInterval{}, false
	}
	index := 0
	point := span[0]
	if endpoint == 1 {
		index = len(q[0]) - 1
		point = span[len(span)-1]
	}
	curve := []*big.Rat{q[0][index], q[1][index], q[2][index]}
	if curve[0].Sign() >= 0 || curve[2].Sign() <= 0 {
		return loftFaceInterval{}, false
	}
	root := spiralIsolateRootN(curve, 64)
	x0 := first.at(point.U, point.V)[0]
	x1 := second.at(point.U, point.V)[0]
	if x0.Cmp(x1) >= 0 {
		return loftFaceInterval{}, false
	}
	return loftFaceInterval{
		lo: lfAdd(lfMul(lfSub(lfOne(), root.lo), x0), lfMul(root.lo, x1)),
		hi: lfAdd(lfMul(lfSub(lfOne(), root.hi), x0), lfMul(root.hi, x1)),
	}, true
}

func spiralShortCapTBracket(q [3][]*big.Rat) (loftFaceInterval, bool) {
	if len(q[0]) == 0 || len(q[0]) != len(q[1]) || len(q[0]) != len(q[2]) {
		return loftFaceInterval{}, false
	}
	lo, hi := big.NewRat(15, 16), lfOne()
	for range 64 {
		mid := lfMul(lfAdd(lo, hi), big.NewRat(1, 2))
		coeff := spiralRootAtBandT(q, mid)
		negative, positive := true, true
		for _, value := range coeff {
			negative = negative && value.Sign() < 0
			positive = positive && value.Sign() > 0
		}
		switch {
		case negative:
			lo = mid
		case positive:
			hi = mid
		default:
			return loftFaceInterval{lo: lo, hi: hi}, true
		}
	}
	return loftFaceInterval{lo: lo, hi: hi}, true
}

// A projective cap map restricted to one line has coordinate extrema at
// its endpoints when its denominator stays positive. Band interpolation
// is affine in t, so these endpoint ranges enclose the whole crossing.
func spiralShortLineBox(first, second loftFaceMap, span freeform.BezierSpan,
	t loftFaceInterval) ([3]loftFaceInterval, bool) {
	var box [3]loftFaceInterval
	if len(span) != 2 || t.lo == nil || t.hi == nil ||
		t.lo.Sign() < 0 || t.lo.Cmp(t.hi) > 0 || t.hi.Cmp(lfOne()) > 0 {
		return box, false
	}
	var caps [2][3]loftFaceInterval
	for index, mapped := range [...]loftFaceMap{first, second} {
		for _, source := range span {
			if mapped.denominator(source.U, source.V).Sign() <= 0 {
				return box, false
			}
			point := mapped.at(source.U, source.V)
			for axis, value := range point {
				interval := &caps[index][axis]
				if interval.lo == nil || value.Cmp(interval.lo) < 0 {
					interval.lo = value
				}
				if interval.hi == nil || value.Cmp(interval.hi) > 0 {
					interval.hi = value
				}
			}
		}
	}
	for axis := range box {
		for _, time := range []*big.Rat{t.lo, t.hi} {
			for _, a := range []*big.Rat{caps[0][axis].lo, caps[0][axis].hi} {
				for _, z := range []*big.Rat{caps[1][axis].lo, caps[1][axis].hi} {
					value := lfAdd(lfMul(lfSub(lfOne(), time), a), lfMul(time, z))
					if box[axis].lo == nil || value.Cmp(box[axis].lo) < 0 {
						box[axis].lo = value
					}
					if box[axis].hi == nil || value.Cmp(box[axis].hi) > 0 {
						box[axis].hi = value
					}
				}
			}
		}
	}
	return box, true
}

func spiralShortCubicBox(ctx context.Context, first, second loftFaceMap,
	span freeform.BezierSpan, q [3][]*big.Rat, clip spiralJunction,
	finiteEnd int) ([3]loftFaceInterval, bool, error) {
	var result [3]loftFaceInterval
	if ctx == nil || len(span) != 4 || clip.sLo == nil || clip.sHi == nil ||
		clip.sLo.Sign() <= 0 || clip.sLo.Cmp(clip.sHi) >= 0 ||
		clip.sHi.Cmp(lfOne()) >= 0 || finiteEnd < 0 || finiteEnd > 1 {
		return result, false, nil
	}
	sLo, sHi := lfZero(), clip.sHi
	if finiteEnd == 1 {
		sLo, sHi = clip.sLo, lfOne()
	}
	width := lfDiv(lfSub(sHi, sLo), big.NewRat(16, 1))
	for cell := range 16 {
		if cell%4 == 0 {
			if err := ctx.Err(); err != nil {
				return result, false, err
			}
		}
		left := lfAdd(sLo, lfMul(big.NewRat(int64(cell), 1), width))
		right := lfAdd(left, width)
		var restricted [3][]*big.Rat
		for row := range restricted {
			restricted[row] = spiralBernRestrict(q[row], left, right)
		}
		t, ok := spiralShortCapTBracket(restricted)
		if !ok {
			return result, false, nil
		}
		box, ok := spiralJunctionBox(first, second, span, left, right, t.lo, t.hi)
		if !ok {
			return result, false, nil
		}
		for axis := range result {
			if result[axis].lo == nil || box[axis].lo.Cmp(result[axis].lo) < 0 {
				result[axis].lo = box[axis].lo
			}
			if result[axis].hi == nil || box[axis].hi.Cmp(result[axis].hi) > 0 {
				result[axis].hi = box[axis].hi
			}
		}
	}
	return result, true, nil
}

func spiralShortCapSpan(source *pointSectionRecord, groups []int,
	group int) (freeform.BezierSpan, bool) {
	if source == nil || len(source.sourceStation) != len(groups) {
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
	a := source.sourceStation[index]
	z := source.sourceStation[(index+1)%len(groups)]
	line, ok := source.profile.Outer.Segments[group].(lineSeg)
	if !ok || a != line.Start && z != line.Start || a != line.End && z != line.End {
		return nil, false
	}
	return freeform.BezierSpan{{U: lfF(a.U), V: lfF(a.V)},
		{U: lfF(z.U), V: lfF(z.V)}}, true
}

func spiralShortCapXRange(first, second loftFaceMap, span freeform.BezierSpan,
	tLo *big.Rat) (loftFaceInterval, bool) {
	if spiralCapXDecreases(first, span) && spiralCapXDecreases(second, span) {
		var out loftFaceInterval
		for i, point := range [...]freeform.RatPoint{span[0], span[len(span)-1]} {
			x0 := first.at(point.U, point.V)[0]
			x1 := second.at(point.U, point.V)[0]
			xLo := lfAdd(lfMul(lfSub(lfOne(), tLo), x0), lfMul(tLo, x1))
			for _, x := range []*big.Rat{xLo, x1} {
				if i == 0 && out.lo == nil || x.Cmp(out.lo) < 0 {
					out.lo = x
				}
				if i == 0 && out.hi == nil || x.Cmp(out.hi) > 0 {
					out.hi = x
				}
			}
		}
		return out, true
	}
	capRange := func(m loftFaceMap) (loftFaceInterval, bool) {
		g, h := spiralCapBern(m, span)
		if len(g) == 0 {
			return loftFaceInterval{}, false
		}
		var out loftFaceInterval
		for i := range g {
			if g[i].Sign() <= 0 {
				return loftFaceInterval{}, false
			}
			x := lfDiv(h[0][i], g[i])
			if i == 0 || x.Cmp(out.lo) < 0 {
				out.lo = x
			}
			if i == 0 || x.Cmp(out.hi) > 0 {
				out.hi = x
			}
		}
		return out, true
	}
	a, okA := capRange(first)
	z, okZ := capRange(second)
	if !okA || !okZ {
		return loftFaceInterval{}, false
	}
	blend := func(x, y, t *big.Rat) *big.Rat {
		return lfAdd(lfMul(lfSub(lfOne(), t), x), lfMul(t, y))
	}
	lo, hi := blend(a.lo, z.lo, tLo), blend(a.hi, z.hi, tLo)
	if z.lo.Cmp(lo) < 0 {
		lo = z.lo
	}
	if z.hi.Cmp(hi) > 0 {
		hi = z.hi
	}
	return loftFaceInterval{lo: lo, hi: hi}, true
}

func spiralShortCapCrossingProof(first, second loftFaceMap, span freeform.BezierSpan,
	m, b, capStart, capEnd *big.Rat, requireFullFinite bool) (spiralShortCapCrossing, bool) {
	if len(span) < 2 || m == nil || b == nil || capStart == nil || capEnd == nil ||
		capStart.Cmp(capEnd) >= 0 {
		return spiralShortCapCrossing{}, false
	}
	q := spiralRootPolynomial(first, second, span, m, b)
	for k := range q[0] {
		if q[0][k].Sign() >= 0 || q[2][k].Sign() <= 0 ||
			lfSub(q[1][k], q[0][k]).Sign() <= 0 ||
			lfSub(q[2][k], q[1][k]).Sign() <= 0 {
			return spiralShortCapCrossing{}, false
		}
	}
	tLo := big.NewRat(15, 16)
	oneMinus := lfSub(lfOne(), tLo)
	for k := range q[0] {
		value := lfAdd(lfAdd(lfMul(lfMul(oneMinus, oneMinus), q[0][k]),
			lfMul(lfMul(lfMul(big.NewRat(2, 1), oneMinus), tLo), q[1][k])),
			lfMul(lfMul(tLo, tLo), q[2][k]))
		if value.Sign() >= 0 {
			return spiralShortCapCrossing{}, false
		}
	}
	x, ok := spiralShortCapXRange(first, second, span, tLo)
	if !ok || requireFullFinite && (x.lo.Cmp(capStart) <= 0 || x.hi.Cmp(capEnd) >= 0) ||
		lfAdd(lfMul(m, x.lo), b).Sign() <= 0 ||
		lfAdd(lfMul(m, x.hi), b).Sign() <= 0 {
		return spiralShortCapCrossing{}, false
	}
	certificate := spiralShortCapCrossing{span: span, q: q, tLo: tLo, x: x}
	if len(span) == 2 {
		t, ok := spiralShortCapTBracket(q)
		if !ok {
			return spiralShortCapCrossing{}, false
		}
		certificate.box, ok = spiralShortLineBox(first, second, span, t)
		if !ok {
			return spiralShortCapCrossing{}, false
		}
	}
	return certificate, true
}

// spiralShortCapFamilies authenticates all four short crossing families at
// each cap. It refuses if a connector no longer coincides with its recorded
// line segment or if any crossing reaches beyond the blank's finite cap.
func spiralShortCapFamilies(ctx context.Context, proof *spiralFirstJoinSource) (
	[2][4]spiralShortCapFamily, bool, error) {
	var out [2][4]spiralShortCapFamily
	if ctx == nil || proof == nil || proof.blank == nil || proof.tooth == nil ||
		proof.tooth.source == nil || len(proof.tooth.maps) != 9 {
		return out, false, nil
	}
	groups := [...]int{0, 2, 3, 5}
	for end := range 2 {
		if err := ctx.Err(); err != nil {
			return out, false, err
		}
		band := 1
		first, last := proof.blank.toeRoot, proof.blank.toeOuter
		if end == 1 {
			band = 7
			first, last = proof.blank.ded, proof.blank.heelOuter
		}
		m := lfDiv(lfSub(lfF(last.V), lfF(first.V)), lfSub(lfF(last.U), lfF(first.U)))
		b := lfSub(lfF(first.V), lfMul(m, lfF(first.U)))
		capStart, capEnd := lfF(first.U), lfF(last.U)
		if m.Cmp(proof.rootM) >= 0 ||
			lfAdd(lfMul(proof.rootM, capStart), proof.rootB).Cmp(lfF(first.V)) != 0 ||
			lfAdd(lfMul(proof.rootM, capEnd), proof.rootB).Sign() <= 0 ||
			lfF(first.V).Sign() <= 0 || lfF(last.V).Sign() <= 0 {
			return out, false, nil
		}
		for index, group := range groups {
			var spans []freeform.BezierSpan
			if index >= 2 {
				span, ok := spiralShortCapSpan(proof.tooth.source, proof.tooth.stationGroup, group)
				if !ok {
					return out, false, nil
				}
				spans = []freeform.BezierSpan{span}
			} else {
				fit, ok := proof.tooth.source.profile.Outer.Segments[group].(fitSplineSeg)
				if !ok {
					return out, false, nil
				}
				work := freeform.NewFreeformWork()
				work.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
				var err error
				spans, err = splinebezier.FitSplineBezierSpans(fit, work)
				if err != nil {
					return out, false, err
				}
				if len(spans) == 0 || len(spans) > 64 ||
					len(spans[0]) != len(proof.flank[index]) {
					return out, false, nil
				}
				for j := range spans[0] {
					if spans[0][j].U.Cmp(proof.flank[index][j].U) != 0 ||
						spans[0][j].V.Cmp(proof.flank[index][j].V) != 0 {
						return out, false, nil
					}
				}
			}
			family := spiralShortCapFamily{band: band, group: group}
			for piece, span := range spans {
				if piece%4 == 0 {
					if err := ctx.Err(); err != nil {
						return out, false, err
					}
				}
				if index < 2 && piece > 0 {
					// Beyond the first fitted span the entire source cell lies
					// outside the root. The finite cap is strictly inward of
					// that root for every X after their shared start.
					q := spiralRootPolynomial(proof.tooth.maps[band],
						proof.tooth.maps[band+1], span, proof.rootM, proof.rootB)
					if !spiralAllPositive(q) {
						return out, false, nil
					}
					continue
				}
				certificate, ok := spiralShortCapCrossingProof(proof.tooth.maps[band],
					proof.tooth.maps[band+1], span, m, b, capStart, capEnd, index >= 2)
				if !ok {
					return out, false, nil
				}
				if piece == 0 || certificate.x.lo.Cmp(family.x.lo) < 0 {
					family.x.lo = certificate.x.lo
				}
				if piece == 0 || certificate.x.hi.Cmp(family.x.hi) > 0 {
					family.x.hi = certificate.x.hi
				}
				family.pieces = append(family.pieces, certificate)
			}
			if index < 2 {
				left, leftOK := spiralShortCapEndpointX(proof.tooth.maps[band],
					proof.tooth.maps[band+1], spans[0], family.pieces[0].q, 0)
				right, rightOK := spiralShortCapEndpointX(proof.tooth.maps[band],
					proof.tooth.maps[band+1], spans[0], family.pieces[0].q, 1)
				if !leftOK || !rightOK {
					return out, false, nil
				}
				switch {
				case left.hi.Cmp(capStart) < 0 && right.lo.Cmp(capStart) > 0 &&
					family.pieces[0].x.hi.Cmp(capEnd) < 0:
					family.finiteEnd = 1
					family.x = loftFaceInterval{lo: capStart, hi: family.pieces[0].x.hi}
				case right.hi.Cmp(capStart) < 0 && left.lo.Cmp(capStart) > 0 &&
					family.pieces[0].x.hi.Cmp(capEnd) < 0:
					family.finiteEnd = 0
					family.x = loftFaceInterval{lo: capStart, hi: family.pieces[0].x.hi}
				default:
					return out, false, nil
				}
				// At capStart the blank cap and root meridians coincide. The
				// authenticated root crossing is unique and X-ordered throughout
				// this band, so this is the only finite-start clip of the cap arc.
				clip, ok := spiralIsolateJunction(ctx, proof.tooth.maps, spans[0],
					proof.rootM, proof.rootB, proof.crossings[index], capStart)
				if !ok || clip.band != band {
					if err := ctx.Err(); err != nil {
						return out, false, err
					}
					return out, false, nil
				}
				family.finiteClip = clip
				box, ok, err := spiralShortCubicBox(ctx, proof.tooth.maps[band],
					proof.tooth.maps[band+1], spans[0], family.pieces[0].q,
					clip, family.finiteEnd)
				if err != nil || !ok {
					return out, false, err
				}
				family.pieces[0].box = box
			}
			out[end][index] = family
		}
	}
	for end := range out {
		for index := range out[end] {
			if len(out[end][index].pieces) == 0 {
				return out, false, nil
			}
		}
	}
	return out, true, nil
}
