package revolvecontact

import (
	"context"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// A vertical source crossing is taken from the authenticated LoftFaces map,
// never from a held mesh vertex. The two radial roots bracket any places
// where the retained boundary enters or leaves the finite Revolve face.
type VerticalCell struct {
	Segment, Span, Band int
	Source              freeform.BezierSpan // nil for a whole live Sketch arc
	Ref                 provenance.Ref
	Face                *topo.Face
	Lower, Upper        loftface.VerticalLimit
	Retained            []loftface.VerticalPiece
}

type VerticalSide struct {
	*ConstantXSource
	Face    *topo.Face
	Origins []provenance.Ref
}

// The caller supplies exact X and radial limits from authenticated surfaces.
// This record only classifies the LoftFaces source at that cross section.
type ConstantXSource struct {
	Band         int
	Time, X      *big.Rat
	Lower, Upper *big.Rat
	YzAffine     *loftface.VerticalAffine
	Cells        []VerticalCell
	HasRetained  bool
}

// A strict gap between the authenticated radial intervals proves the entire
// blank side misses the trimmed source. Equality remains undecided.
func VerticalSideClassify(ctx context.Context,
	proof *SourceCertificate, sectionIndex int) (
	*VerticalSide, revolveloftfaces.VerticalOutcome, error,
) {
	if ctx == nil || proof == nil || proof.Trim == nil || proof.Blank == nil ||
		proof.Tooth == nil || sectionIndex < 0 || sectionIndex >= len(proof.Section.Spans) {
		return nil, revolveloftfaces.VerticalRefusal, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, revolveloftfaces.VerticalRefusal, err
	}
	trim := proof.Trim
	section := proof.Section.Spans[sectionIndex]
	if section.Start.U != section.End.U || section.Start.V == section.End.V ||
		section.Start.V < 0 || section.End.V < 0 || section.Face == nil ||
		len(section.Origins) == 0 || !SectionCurrent(proof) {
		return nil, revolveloftfaces.VerticalRefusal, nil
	}
	face, ok := revolveshell.SpanFace(proof.Blank, section)
	if !ok || face != section.Face || !revolveloftfaces.OriginsMatch(face.Origins, section.Origins) {
		return nil, revolveloftfaces.VerticalRefusal, nil
	}
	payload, ok := proof.Tooth.Payload.(facetedrecord.Payload)
	if !ok || payload.LoftFacesTrim != trim || !TrimCurrent(proof.Tooth, trim) ||
		!SourceRefsCurrent(trim, proof.Tooth, proof.SourceRefs) {
		return nil, revolveloftfaces.VerticalRefusal, nil
	}
	x := lfF(section.Start.U)
	lower, upper, limitOutcome := revolveloftfaces.VerticalLimits(trim, section, x)
	if limitOutcome != revolveloftfaces.VerticalContact {
		return nil, limitOutcome, nil
	}
	source, ok, err := ConstantXSourceCells(ctx, proof, x, lower, upper)
	if err != nil || !ok || !source.HasRetained {
		return nil, revolveloftfaces.VerticalRefusal, err
	}
	return &VerticalSide{
		ConstantXSource: source, Face: face,
		Origins: append([]provenance.Ref(nil), section.Origins...),
	}, revolveloftfaces.VerticalContact, nil
}

// A retained face origin alone does not bind the section coordinates.
// Rebuild the Revolve source walk before using its radial extent as a miss.
func SectionCurrent(proof *SourceCertificate) bool {
	if proof == nil || proof.Blank == nil {
		return false
	}
	want := proof.Section
	for _, live := range revolveshell.RootCandidates(proof.Blank) {
		if live.Blank != want.Blank || live.RootSpan != want.RootSpan ||
			live.AxisSpan != want.AxisSpan || live.Frame != want.Frame ||
			live.RootStart != want.RootStart || live.RootEnd != want.RootEnd ||
			live.RootCone != want.RootCone || live.ConeGap != want.ConeGap ||
			len(live.Spans) != len(want.Spans) {
			continue
		}
		current := true
		for i, span := range live.Spans {
			recorded := want.Spans[i]
			if span.Start != recorded.Start || span.End != recorded.End ||
				span.Face != recorded.Face ||
				!slices.Equal(span.SourceEdges, recorded.SourceEdges) ||
				!slices.Equal(span.Origins, recorded.Origins) {
				current = false
				break
			}
		}
		if current {
			return true
		}
	}
	return false
}

// ConstantXSourceCells classifies every live source segment
// at an exact constant-X section between caller-supplied radial limits. The
// caller must authenticate those limits and any face contact independently.
// A successful result with hasRetained=false proves the source contour misses
// the open radial interval; it does not prove the denoted solid misses it.
func ConstantXSourceCells(ctx context.Context,
	proof *SourceCertificate, x, lower, upper *big.Rat) (
	*ConstantXSource, bool, error,
) {
	if ctx == nil || proof == nil || proof.Trim == nil || proof.Tooth == nil ||
		x == nil || lower == nil || upper == nil || lower.Sign() < 0 ||
		lower.Cmp(upper) >= 0 {
		return nil, false, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	trim := proof.Trim
	payload, ok := proof.Tooth.Payload.(facetedrecord.Payload)
	if !ok || payload.LoftFacesTrim != trim || !TrimCurrent(proof.Tooth, trim) ||
		!SourceRefsCurrent(trim, proof.Tooth, proof.SourceRefs) {
		return nil, false, nil
	}
	box, err := revolveloftfaces.LfSourceBox(trim.Source)
	if err != nil {
		return nil, false, err
	}
	band, time, ok := loftface.ConstantXBand(trim.Maps, box, x)
	if !ok || len(proof.SourceRefs[band]) != len(trim.Source.Profile.Outer.Segments) {
		return nil, false, nil
	}
	result := &ConstantXSource{Band: band,
		Time: time, X: x, Lower: lower, Upper: upper}
	result.YzAffine, _ = loftface.VerticalAffineMap(trim.Maps[band],
		trim.Maps[band+1], time)
	for segment, source := range trim.Source.Profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		ref := proof.SourceRefs[band][segment]
		spans, ok := revolveloftfaces.RevolveLoftFacesStraightOrFitSpans(trim, segment, source)
		if !ok {
			if _, arc := source.(sectionrecord.ArcSeg); !arc {
				return nil, false, nil
			}
			cell, certified := VerticalArc(ctx, trim, x, segment,
				band, time, lower, upper, ref)
			if !certified {
				return nil, false, nil
			}
			if len(cell.Retained) > 0 {
				cell.Face = VerticalLiveFace(proof.Tooth, ref)
				if cell.Face == nil {
					return nil, false, nil
				}
			}
			result.Cells = append(result.Cells, cell)
			result.HasRetained = result.HasRetained || len(cell.Retained) > 0
			continue
		}
		for spanIndex, span := range spans {
			cell, certified := VerticalBezierWith(revolveloftfaces.RevolveLoftFacesMeridianMemoOf(ctx), trim, segment, spanIndex,
				band, time, lower, upper, ref, span)
			if !certified {
				return nil, false, nil
			}
			if len(cell.Retained) > 0 {
				cell.Face = VerticalLiveFace(proof.Tooth, ref)
				if cell.Face == nil {
					return nil, false, nil
				}
			}
			result.Cells = append(result.Cells, cell)
			result.HasRetained = result.HasRetained || len(cell.Retained) > 0
		}
	}
	return result, true, nil
}

// A proof's cached refs can only name the live source segment for their
// band. Rebuild from the producer groups before exporting any contact cell.
func SourceRefsCurrent(trim *facetedrecord.LoftFacesTrimRecord, tooth *topo.Body,
	bound [][]provenance.Ref) bool {
	if trim == nil || tooth == nil {
		return false
	}
	current, ok := SourceRefs(trim, tooth)
	if !ok || len(current) != len(bound) {
		return false
	}
	for band := range current {
		if len(current[band]) != len(bound[band]) {
			return false
		}
		for segment, ref := range current[band] {
			if ref != bound[band][segment] {
				return false
			}
		}
	}
	return true
}

func VerticalLiveFace(tooth *topo.Body, ref provenance.Ref) *topo.Face {
	if tooth == nil {
		return nil
	}
	payload, ok := tooth.Payload.(facetedrecord.Payload)
	if !ok {
		return nil
	}
	faces := topobuild.Faces(tooth)
	index, valid := revolveloftfaces.RefFaceIndex(payload, ref, len(faces))
	face, originValid := UniqueOriginFace(tooth, ref)
	if !valid || !originValid || index < 0 || face == nil || faces[index] != face {
		return nil
	}
	return face
}

func VerticalBezier(trim *facetedrecord.LoftFacesTrimRecord,
	segment, spanIndex, band int, time, lower, upper *big.Rat,
	ref provenance.Ref, span freeform.BezierSpan) (VerticalCell, bool) {
	return VerticalBezierWith(nil, trim, segment, spanIndex, band,
		time, lower, upper, ref, span)
}

func VerticalBezierWith(memo *loftface.MeridianMemo, trim *facetedrecord.LoftFacesTrimRecord,
	segment, spanIndex, band int, time, lower, upper *big.Rat,
	ref provenance.Ref, span freeform.BezierSpan) (VerticalCell, bool) {
	cell := VerticalCell{Segment: segment, Span: spanIndex,
		Band: band, Source: span, Ref: ref}
	first, last := trim.Maps[band], trim.Maps[band+1]
	loQ := polynomial.BernsteinRowsAt(memo.QRows(first, last, span, lfZero(), lower), time)
	hiQ := polynomial.BernsteinRowsAt(memo.QRows(first, last, span, lfZero(), upper), time)
	var ok bool
	cell.Lower, ok = loftface.VerticalClassify(loQ)
	if !ok {
		return cell, false
	}
	cell.Upper, ok = loftface.VerticalClassify(hiQ)
	if !ok {
		return cell, false
	}
	for _, limit := range []*loftface.VerticalLimit{&cell.Lower, &cell.Upper} {
		if limit.Root.Lo == nil {
			continue
		}
		limit.Box, ok = loftface.JunctionBox(first, last, span,
			limit.Root.Lo, limit.Root.Hi, time, time)
		if !ok {
			return cell, false
		}
	}
	cell.Retained, ok = loftface.VerticalPieces(first, last, span,
		time, loQ, hiQ, cell.Lower.Root, cell.Upper.Root)
	return cell, ok
}

// A live whole arc may only be used when its authenticated map gives one
// exact squared radius across all of its source parameters at this time.
func VerticalArc(ctx context.Context, trim *facetedrecord.LoftFacesTrimRecord,
	x *big.Rat, segment, band int, time, lower, upper *big.Rat,
	ref provenance.Ref) (VerticalCell, bool) {
	cell := VerticalCell{Segment: segment, Band: band, Ref: ref}
	unit := facetedrecord.RevolveSectionSpan{Start: sectionrecord.Point2{V: 1}, End: sectionrecord.Point2{U: 1, V: 1}}
	radial, ok := revolveloftfaces.MappedArcRadialTimeBand(ctx, trim, segment, band, unit)
	if !ok || radial.Arc == nil {
		return cell, false
	}
	radius2 := lfAdd(polynomial.BernsteinValue(radial.Q[:], time), lfOne())
	if radius2.Sign() <= 0 {
		return cell, false
	}
	lo2, hi2 := lfMul(lower, lower), lfMul(upper, upper)
	loSign, hiSign := lfSub(radius2, lo2).Sign(), lfSub(radius2, hi2).Sign()
	if loSign == 0 || hiSign == 0 {
		return cell, false
	}
	cell.Lower.Side, cell.Upper.Side = loSign, hiSign
	if loSign > 0 && hiSign < 0 {
		box, valid := revolveloftfaces.VerticalArcBox(radial.Arc, trim, band, time)
		if !valid || box[0].Lo.Cmp(x) != 0 || box[0].Hi.Cmp(x) != 0 {
			return cell, false
		}
		cell.Retained = []loftface.VerticalPiece{{
			Source: loftface.Interval{Lo: lfZero(), Hi: lfOne()}, Box: box}}
	}
	return cell, true
}
