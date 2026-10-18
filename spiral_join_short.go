package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
)

// spiralShortCapFamilies authenticates all four short crossing families at
// each cap. It refuses if a connector no longer coincides with its recorded
// line segment or if any crossing reaches beyond the blank's finite cap.
func spiralShortCapFamilies(ctx context.Context, proof *spiralFirstJoinSource) (
	[2][4]spiralsector.ShortCapFamily, bool, error) {
	var out [2][4]spiralsector.ShortCapFamily
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
				span, ok := spiralsector.ShortCapSpan(proof.Tooth.Source, proof.Tooth.StationGroup, group)
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
			family := spiralsector.ShortCapFamily{Band: band, Group: group}
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
				certificate, ok := spiralsector.ShortCapCrossingProof(proof.Tooth.Maps[band],
					proof.Tooth.Maps[band+1], span, m, b, capStart, capEnd, index >= 2)
				if !ok {
					return out, false, nil
				}
				if piece == 0 || certificate.X.Lo.Cmp(family.X.Lo) < 0 {
					family.X.Lo = certificate.X.Lo
				}
				if piece == 0 || certificate.X.Hi.Cmp(family.X.Hi) > 0 {
					family.X.Hi = certificate.X.Hi
				}
				family.Pieces = append(family.Pieces, certificate)
			}
			if index < 2 {
				left, leftOK := spiralsector.ShortCapEndpointX(proof.Tooth.Maps[band],
					proof.Tooth.Maps[band+1], spans[0], family.Pieces[0].Q, 0)
				right, rightOK := spiralsector.ShortCapEndpointX(proof.Tooth.Maps[band],
					proof.Tooth.Maps[band+1], spans[0], family.Pieces[0].Q, 1)
				if !leftOK || !rightOK {
					return out, false, nil
				}
				switch {
				case left.Hi.Cmp(capStart) < 0 && right.Lo.Cmp(capStart) > 0 &&
					family.Pieces[0].X.Hi.Cmp(capEnd) < 0:
					family.FiniteEnd = 1
					family.X = loftFaceInterval{Lo: capStart, Hi: family.Pieces[0].X.Hi}
				case right.Hi.Cmp(capStart) < 0 && left.Lo.Cmp(capStart) > 0 &&
					family.Pieces[0].X.Hi.Cmp(capEnd) < 0:
					family.FiniteEnd = 0
					family.X = loftFaceInterval{Lo: capStart, Hi: family.Pieces[0].X.Hi}
				default:
					return out, false, nil
				}
				// At capStart the blank cap and root meridians coincide. The
				// authenticated root crossing is unique and X-ordered throughout
				// this band, so this is the only finite-start clip of the cap arc.
				clip, ok := spiralsector.IsolateJunction(ctx, proof.Tooth.Maps, spans[0],
					proof.RootM, proof.RootB, proof.Crossings[index], capStart)
				if !ok || clip.Band != band {
					if err := ctx.Err(); err != nil {
						return out, false, err
					}
					return out, false, nil
				}
				family.FiniteClip = clip
				box, ok, err := spiralsector.ShortCubicBox(ctx, proof.Tooth.Maps[band],
					proof.Tooth.Maps[band+1], spans[0], family.Pieces[0].Q,
					clip.SLo, clip.SHi, family.FiniteEnd)
				if err != nil || !ok {
					return out, false, err
				}
				family.Pieces[0].Box = box
			}
			out[end][index] = family
		}
	}
	for end := range out {
		for index := range out[end] {
			if len(out[end][index].Pieces) == 0 {
				return out, false, nil
			}
		}
	}
	return out, true, nil
}
