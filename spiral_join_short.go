package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
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

func spiralShortCapSpan(source *pointSectionRecord, groups []int,
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
	line, ok := source.Profile.Outer.Segments[group].(lineSeg)
	if !ok || a != line.Start && z != line.Start || a != line.End && z != line.End {
		return nil, false
	}
	return freeform.BezierSpan{{U: lfF(a.U), V: lfF(a.V)},
		{U: lfF(z.U), V: lfF(z.V)}}, true
}

func spiralShortCapCrossingProof(first, second loftFaceMap, span freeform.BezierSpan,
	m, b, capStart, capEnd *big.Rat, requireFullFinite bool) (spiralShortCapCrossing, bool) {
	if len(span) < 2 || m == nil || b == nil || capStart == nil || capEnd == nil ||
		capStart.Cmp(capEnd) >= 0 {
		return spiralShortCapCrossing{}, false
	}
	q := loftface.MeridianQRows(first, second, span, m, b)
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
	x, ok := spiralsector.ShortCapXRange(first, second, span, tLo)
	if !ok || requireFullFinite && (x.Lo.Cmp(capStart) <= 0 || x.Hi.Cmp(capEnd) >= 0) ||
		lfAdd(lfMul(m, x.Lo), b).Sign() <= 0 ||
		lfAdd(lfMul(m, x.Hi), b).Sign() <= 0 {
		return spiralShortCapCrossing{}, false
	}
	certificate := spiralShortCapCrossing{span: span, q: q, tLo: tLo, x: x}
	if len(span) == 2 {
		t, ok := spiralsector.ShortCapTBracket(q)
		if !ok {
			return spiralShortCapCrossing{}, false
		}
		certificate.box, ok = spiralsector.ShortLineBox(first, second, span, t)
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
	if ctx == nil || proof == nil || proof.Blank == nil || proof.Tooth == nil ||
		proof.Tooth.Source == nil || len(proof.Tooth.Maps) != 9 {
		return out, false, nil
	}
	groups := [...]int{0, 2, 3, 5}
	for end := range 2 {
		if err := ctx.Err(); err != nil {
			return out, false, err
		}
		band := 1
		first, last := proof.Blank.ToeRoot, proof.Blank.ToeOuter
		if end == 1 {
			band = 7
			first, last = proof.Blank.Ded, proof.Blank.HeelOuter
		}
		m := lfDiv(lfSub(lfF(last.V), lfF(first.V)), lfSub(lfF(last.U), lfF(first.U)))
		b := lfSub(lfF(first.V), lfMul(m, lfF(first.U)))
		capStart, capEnd := lfF(first.U), lfF(last.U)
		if m.Cmp(proof.RootM) >= 0 ||
			lfAdd(lfMul(proof.RootM, capStart), proof.RootB).Cmp(lfF(first.V)) != 0 ||
			lfAdd(lfMul(proof.RootM, capEnd), proof.RootB).Sign() <= 0 ||
			lfF(first.V).Sign() <= 0 || lfF(last.V).Sign() <= 0 {
			return out, false, nil
		}
		for index, group := range groups {
			var spans []freeform.BezierSpan
			if index >= 2 {
				span, ok := spiralShortCapSpan(proof.Tooth.Source, proof.Tooth.StationGroup, group)
				if !ok {
					return out, false, nil
				}
				spans = []freeform.BezierSpan{span}
			} else {
				fit, ok := proof.Tooth.Source.Profile.Outer.Segments[group].(fitSplineSeg)
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
					len(spans[0]) != len(proof.Flank[index]) {
					return out, false, nil
				}
				for j := range spans[0] {
					if spans[0][j].U.Cmp(proof.Flank[index][j].U) != 0 ||
						spans[0][j].V.Cmp(proof.Flank[index][j].V) != 0 {
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
					q := loftface.MeridianQRows(proof.Tooth.Maps[band],
						proof.Tooth.Maps[band+1], span, proof.RootM, proof.RootB)
					if !polynomial.BernsteinRowsPositive(q) {
						return out, false, nil
					}
					continue
				}
				certificate, ok := spiralShortCapCrossingProof(proof.Tooth.Maps[band],
					proof.Tooth.Maps[band+1], span, m, b, capStart, capEnd, index >= 2)
				if !ok {
					return out, false, nil
				}
				if piece == 0 || certificate.x.Lo.Cmp(family.x.Lo) < 0 {
					family.x.Lo = certificate.x.Lo
				}
				if piece == 0 || certificate.x.Hi.Cmp(family.x.Hi) > 0 {
					family.x.Hi = certificate.x.Hi
				}
				family.pieces = append(family.pieces, certificate)
			}
			if index < 2 {
				left, leftOK := spiralsector.ShortCapEndpointX(proof.Tooth.Maps[band],
					proof.Tooth.Maps[band+1], spans[0], family.pieces[0].q, 0)
				right, rightOK := spiralsector.ShortCapEndpointX(proof.Tooth.Maps[band],
					proof.Tooth.Maps[band+1], spans[0], family.pieces[0].q, 1)
				if !leftOK || !rightOK {
					return out, false, nil
				}
				switch {
				case left.Hi.Cmp(capStart) < 0 && right.Lo.Cmp(capStart) > 0 &&
					family.pieces[0].x.Hi.Cmp(capEnd) < 0:
					family.finiteEnd = 1
					family.x = loftFaceInterval{Lo: capStart, Hi: family.pieces[0].x.Hi}
				case right.Hi.Cmp(capStart) < 0 && left.Lo.Cmp(capStart) > 0 &&
					family.pieces[0].x.Hi.Cmp(capEnd) < 0:
					family.finiteEnd = 0
					family.x = loftFaceInterval{Lo: capStart, Hi: family.pieces[0].x.Hi}
				default:
					return out, false, nil
				}
				// At capStart the blank cap and root meridians coincide. The
				// authenticated root crossing is unique and X-ordered throughout
				// this band, so this is the only finite-start clip of the cap arc.
				clip, ok := spiralIsolateJunction(ctx, proof.Tooth.Maps, spans[0],
					proof.RootM, proof.RootB, proof.Crossings[index], capStart)
				if !ok || clip.Band != band {
					if err := ctx.Err(); err != nil {
						return out, false, err
					}
					return out, false, nil
				}
				family.finiteClip = clip
				box, ok, err := spiralsector.ShortCubicBox(ctx, proof.Tooth.Maps[band],
					proof.Tooth.Maps[band+1], spans[0], family.pieces[0].q,
					clip.SLo, clip.SHi, family.finiteEnd)
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
