package decad

import (
	"context"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/sketch"
)

// A vertical source crossing is taken from the authenticated LoftFaces map,
// never from a held mesh vertex. The two radial roots bracket any places
// where the retained boundary enters or leaves the finite Revolve face.
type revolveLoftFacesVerticalCell struct {
	segment, span, band int
	source              freeform.BezierSpan // nil for a whole live Sketch arc
	ref                 provenance.Ref
	face                *Face
	lower, upper        loftface.VerticalLimit
	retained            []loftface.VerticalPiece
}

type revolveLoftFacesVerticalSide struct {
	*revolveLoftFacesConstantXSource
	face    *Face
	origins []provenance.Ref
}

type revolveLoftFacesVerticalOutcome uint8

const (
	revolveLoftFacesVerticalRefusal revolveLoftFacesVerticalOutcome = iota
	revolveLoftFacesVerticalMiss
	revolveLoftFacesVerticalContact
)

// The caller supplies exact X and radial limits from authenticated surfaces.
// This record only classifies the LoftFaces source at that cross section.
type revolveLoftFacesConstantXSource struct {
	band         int
	time, x      *big.Rat
	lower, upper *big.Rat
	yzAffine     *loftface.VerticalAffine
	cells        []revolveLoftFacesVerticalCell
	hasRetained  bool
}

// A strict gap between the authenticated radial intervals proves the entire
// blank side misses the trimmed source. Equality remains undecided.
func revolveLoftFacesVerticalSideClassify(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, sectionIndex int) (
	*revolveLoftFacesVerticalSide, revolveLoftFacesVerticalOutcome, error,
) {
	if ctx == nil || proof == nil || proof.trim == nil || proof.blank == nil ||
		proof.tooth == nil || sectionIndex < 0 || sectionIndex >= len(proof.section.spans) {
		return nil, revolveLoftFacesVerticalRefusal, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, revolveLoftFacesVerticalRefusal, err
	}
	trim := proof.trim
	section := proof.section.spans[sectionIndex]
	if section.start.U != section.end.U || section.start.V == section.end.V ||
		section.start.V < 0 || section.end.V < 0 || section.face == nil ||
		len(section.origins) == 0 || !revolveLoftFacesSectionCurrent(proof) {
		return nil, revolveLoftFacesVerticalRefusal, nil
	}
	face, ok := revolveSpanFace(proof.blank, section)
	if !ok || face != section.face || !verticalOriginsMatch(face.originRefs(), section.origins) {
		return nil, revolveLoftFacesVerticalRefusal, nil
	}
	payload, ok := proof.tooth.n.Payload.(facetedPayload)
	if !ok || payload.LoftFacesTrim != trim || !revolveLoftFacesTrimCurrent(proof.tooth, trim) ||
		!revolveLoftFacesSourceRefsCurrent(trim, proof.tooth, proof.sourceRefs) {
		return nil, revolveLoftFacesVerticalRefusal, nil
	}
	x := lfF(section.start.U)
	lower, upper, limitOutcome := revolveLoftFacesVerticalLimits(trim, section, x)
	if limitOutcome != revolveLoftFacesVerticalContact {
		return nil, limitOutcome, nil
	}
	source, ok, err := revolveLoftFacesConstantXSourceCells(ctx, proof, x, lower, upper)
	if err != nil || !ok || !source.hasRetained {
		return nil, revolveLoftFacesVerticalRefusal, err
	}
	return &revolveLoftFacesVerticalSide{
		revolveLoftFacesConstantXSource: source, face: face,
		origins: append([]provenance.Ref(nil), section.origins...),
	}, revolveLoftFacesVerticalContact, nil
}

// A retained face origin alone does not bind the section coordinates.
// Rebuild the Revolve source walk before using its radial extent as a miss.
func revolveLoftFacesSectionCurrent(proof *revolveLoftFacesSourceCertificate) bool {
	if proof == nil || proof.blank == nil {
		return false
	}
	want := proof.section
	for _, live := range revolveRootCandidates(proof.blank) {
		if live.blank != want.blank || live.rootSpan != want.rootSpan ||
			live.axisSpan != want.axisSpan || live.frame != want.frame ||
			live.rootStart != want.rootStart || live.rootEnd != want.rootEnd ||
			live.rootCone != want.rootCone || live.coneGap != want.coneGap ||
			len(live.spans) != len(want.spans) {
			continue
		}
		current := true
		for i, span := range live.spans {
			recorded := want.spans[i]
			if span.start != recorded.start || span.end != recorded.end ||
				span.face != recorded.face ||
				!slices.Equal(span.sourceEdges, recorded.sourceEdges) ||
				!slices.Equal(span.origins, recorded.origins) {
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

// revolveLoftFacesConstantXSourceCells classifies every live source segment
// at an exact constant-X section between caller-supplied radial limits. The
// caller must authenticate those limits and any face contact independently.
// A successful result with hasRetained=false proves the source contour misses
// the open radial interval; it does not prove the denoted solid misses it.
func revolveLoftFacesConstantXSourceCells(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, x, lower, upper *big.Rat) (
	*revolveLoftFacesConstantXSource, bool, error,
) {
	if ctx == nil || proof == nil || proof.trim == nil || proof.tooth == nil ||
		x == nil || lower == nil || upper == nil || lower.Sign() < 0 ||
		lower.Cmp(upper) >= 0 {
		return nil, false, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	trim := proof.trim
	payload, ok := proof.tooth.n.Payload.(facetedPayload)
	if !ok || payload.LoftFacesTrim != trim || !revolveLoftFacesTrimCurrent(proof.tooth, trim) ||
		!revolveLoftFacesSourceRefsCurrent(trim, proof.tooth, proof.sourceRefs) {
		return nil, false, nil
	}
	box, err := lfSourceBox(trim.Source)
	if err != nil {
		return nil, false, err
	}
	band, time, ok := loftface.ConstantXBand(trim.Maps, box, x)
	if !ok || len(proof.sourceRefs[band]) != len(trim.Source.profile.Outer.Segments) {
		return nil, false, nil
	}
	result := &revolveLoftFacesConstantXSource{band: band,
		time: time, x: x, lower: lower, upper: upper}
	result.yzAffine, _ = loftface.VerticalAffineMap(trim.Maps[band],
		trim.Maps[band+1], time)
	for segment, source := range trim.Source.profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		ref := proof.sourceRefs[band][segment]
		spans, ok := revolveLoftFacesStraightOrFitSpans(trim, segment, source)
		if !ok {
			if _, arc := source.(arcSeg); !arc {
				return nil, false, nil
			}
			cell, certified := revolveLoftFacesVerticalArc(ctx, trim, x, segment,
				band, time, lower, upper, ref)
			if !certified {
				return nil, false, nil
			}
			if len(cell.retained) > 0 {
				cell.face = revolveLoftFacesVerticalLiveFace(proof.tooth, ref)
				if cell.face == nil {
					return nil, false, nil
				}
			}
			result.cells = append(result.cells, cell)
			result.hasRetained = result.hasRetained || len(cell.retained) > 0
			continue
		}
		for spanIndex, span := range spans {
			cell, certified := revolveLoftFacesVerticalBezierWith(revolveLoftFacesMeridianMemoOf(ctx), trim, segment, spanIndex,
				band, time, lower, upper, ref, span)
			if !certified {
				return nil, false, nil
			}
			if len(cell.retained) > 0 {
				cell.face = revolveLoftFacesVerticalLiveFace(proof.tooth, ref)
				if cell.face == nil {
					return nil, false, nil
				}
			}
			result.cells = append(result.cells, cell)
			result.hasRetained = result.hasRetained || len(cell.retained) > 0
		}
	}
	return result, true, nil
}

// A proof's cached refs can only name the live source segment for their
// band. Rebuild from the producer groups before exporting any contact cell.
func revolveLoftFacesSourceRefsCurrent(trim *loftFacesTrimRecord, tooth *Body,
	bound [][]provenance.Ref) bool {
	if trim == nil || tooth == nil {
		return false
	}
	current, ok := revolveLoftFacesSourceRefs(trim, tooth)
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

func revolveLoftFacesVerticalLiveFace(tooth *Body, ref provenance.Ref) *Face {
	if tooth == nil {
		return nil
	}
	payload, ok := tooth.n.Payload.(facetedPayload)
	if !ok {
		return nil
	}
	faces := tooth.Faces()
	index, valid := revolveLoftFacesRefFaceIndex(payload, ref, len(faces))
	face, originValid := revolveLoftFacesUniqueOriginFace(tooth, ref)
	if !valid || !originValid || index < 0 || face == nil || faces[index] != face {
		return nil
	}
	return face
}

func verticalOriginsMatch(actual, recorded []provenance.Ref) bool {
	if len(actual) != len(recorded) {
		return false
	}
	for i := range actual {
		if actual[i] != recorded[i] {
			return false
		}
	}
	return true
}

// The Cut keeps the strict outside of its cone. Intersect keeps the strict
// inside of its cone. On an X-constant plane these become radial limits.
func revolveLoftFacesVerticalLimits(trim *loftFacesTrimRecord,
	section revolveSectionSpan, x *big.Rat) (*big.Rat, *big.Rat, revolveLoftFacesVerticalOutcome) {
	if trim == nil || trim.Toe == nil || trim.Heel == nil || x == nil ||
		trim.Toe.apex == nil || trim.Heel.apex == nil ||
		trim.Toe.far == nil || trim.Heel.far == nil ||
		trim.Toe.slope == nil || trim.Heel.slope == nil ||
		trim.Toe.slope.Sign() <= 0 || trim.Heel.slope.Sign() <= 0 ||
		x.Cmp(trim.Toe.far) <= 0 || x.Cmp(trim.Heel.far) <= 0 ||
		x.Cmp(trim.Heel.apex) >= 0 {
		return nil, nil, revolveLoftFacesVerticalRefusal
	}
	lower, upper := lfF(section.start.V), lfF(section.end.V)
	if lower.Cmp(upper) > 0 {
		lower, upper = upper, lower
	}
	if x.Cmp(trim.Toe.apex) < 0 {
		toe := lfDiv(lfSub(trim.Toe.apex, x), trim.Toe.slope)
		if lower.Cmp(toe) < 0 {
			lower = toe
		}
	}
	heel := lfDiv(lfSub(trim.Heel.apex, x), trim.Heel.slope)
	if upper.Cmp(heel) > 0 {
		upper = heel
	}
	if lower.Sign() < 0 || upper.Sign() < 0 || lower.Cmp(upper) == 0 {
		return lower, upper, revolveLoftFacesVerticalRefusal
	}
	if lower.Cmp(upper) > 0 {
		return lower, upper, revolveLoftFacesVerticalMiss
	}
	return lower, upper, revolveLoftFacesVerticalContact
}

func revolveLoftFacesVerticalBezier(trim *loftFacesTrimRecord,
	segment, spanIndex, band int, time, lower, upper *big.Rat,
	ref provenance.Ref, span freeform.BezierSpan) (revolveLoftFacesVerticalCell, bool) {
	return revolveLoftFacesVerticalBezierWith(nil, trim, segment, spanIndex, band,
		time, lower, upper, ref, span)
}

func revolveLoftFacesVerticalBezierWith(memo *loftface.MeridianMemo, trim *loftFacesTrimRecord,
	segment, spanIndex, band int, time, lower, upper *big.Rat,
	ref provenance.Ref, span freeform.BezierSpan) (revolveLoftFacesVerticalCell, bool) {
	cell := revolveLoftFacesVerticalCell{segment: segment, span: spanIndex,
		band: band, source: span, ref: ref}
	first, last := trim.Maps[band], trim.Maps[band+1]
	loQ := polynomial.BernsteinRowsAt(memo.QRows(first, last, span, lfZero(), lower), time)
	hiQ := polynomial.BernsteinRowsAt(memo.QRows(first, last, span, lfZero(), upper), time)
	var ok bool
	cell.lower, ok = loftface.VerticalClassify(loQ)
	if !ok {
		return cell, false
	}
	cell.upper, ok = loftface.VerticalClassify(hiQ)
	if !ok {
		return cell, false
	}
	for _, limit := range []*loftface.VerticalLimit{&cell.lower, &cell.upper} {
		if limit.Root.Lo == nil {
			continue
		}
		limit.Box, ok = loftface.JunctionBox(first, last, span,
			limit.Root.Lo, limit.Root.Hi, time, time)
		if !ok {
			return cell, false
		}
	}
	cell.retained, ok = loftface.VerticalPieces(first, last, span,
		time, loQ, hiQ, cell.lower.Root, cell.upper.Root)
	return cell, ok
}

// A live whole arc may only be used when its authenticated map gives one
// exact squared radius across all of its source parameters at this time.
func revolveLoftFacesVerticalArc(ctx context.Context, trim *loftFacesTrimRecord,
	x *big.Rat, segment, band int, time, lower, upper *big.Rat,
	ref provenance.Ref) (revolveLoftFacesVerticalCell, bool) {
	cell := revolveLoftFacesVerticalCell{segment: segment, band: band, ref: ref}
	unit := revolveSectionSpan{start: planePoint{V: 1}, end: planePoint{U: 1, V: 1}}
	radial, ok := mappedArcRadialTimeBand(ctx, trim, segment, band, unit)
	if !ok || radial.arc == nil {
		return cell, false
	}
	radius2 := lfAdd(polynomial.BernsteinValue(radial.q[:], time), lfOne())
	if radius2.Sign() <= 0 {
		return cell, false
	}
	lo2, hi2 := lfMul(lower, lower), lfMul(upper, upper)
	loSign, hiSign := lfSub(radius2, lo2).Sign(), lfSub(radius2, hi2).Sign()
	if loSign == 0 || hiSign == 0 {
		return cell, false
	}
	cell.lower.Side, cell.upper.Side = loSign, hiSign
	if loSign > 0 && hiSign < 0 {
		box, valid := revolveLoftFacesVerticalArcBox(radial.arc, trim, band, time)
		if !valid || box[0].Lo.Cmp(x) != 0 || box[0].Hi.Cmp(x) != 0 {
			return cell, false
		}
		cell.retained = []loftface.VerticalPiece{{
			Source: loftFaceInterval{Lo: lfZero(), Hi: lfOne()}, Box: box}}
	}
	return cell, true
}

func revolveLoftFacesVerticalArcBox(arc *sketch.Arc, trim *loftFacesTrimRecord,
	band int, time *big.Rat) ([3]loftFaceInterval, bool) {
	var box [3]loftFaceInterval
	u, v, err := arc.CertifiedSpanBounds(0, 1)
	if err != nil {
		return box, false
	}
	sourceBox, err := lfSourceBox(trim.Source)
	if err != nil || lfF(u.Lo).Cmp(sourceBox[0]) < 0 ||
		lfF(u.Hi).Cmp(sourceBox[1]) > 0 ||
		lfF(v.Lo).Cmp(sourceBox[2]) < 0 || lfF(v.Hi).Cmp(sourceBox[3]) > 0 {
		return box, false
	}
	corners := loftface.TileCorners(lfF(u.Lo), lfF(u.Hi), lfF(v.Lo), lfF(v.Hi))
	first, last := loftface.ArcCapRanges(trim.Maps[band], corners),
		loftface.ArcCapRanges(trim.Maps[band+1], corners)
	for axis := range box {
		box[axis] = loftface.ArcBlend(first[axis], last[axis], time)
	}
	return box, true
}
