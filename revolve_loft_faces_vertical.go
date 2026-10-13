package decad

import (
	"context"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/sketch"
)

// A vertical source crossing is taken from the authenticated LoftFaces map,
// never from a held mesh vertex. The two radial roots bracket any places
// where the retained boundary enters or leaves the finite Revolve face.
type revolveLoftFacesVerticalCell struct {
	segment, span, band int
	source              freeform.BezierSpan // nil for a whole live Sketch arc
	ref                 FeatureRef
	face                *Face
	lower, upper        revolveLoftFacesVerticalLimit
	retained            []revolveLoftFacesVerticalPiece
}

type revolveLoftFacesVerticalLimit struct {
	// side is the sign of radius squared minus the limit squared when there
	// is no root. A root has strict signs on both sides of its bracket.
	side, before, after int
	root                loftFaceInterval
	box                 [3]loftFaceInterval
}

type revolveLoftFacesVerticalPiece struct {
	source         loftFaceInterval
	loOpen, hiOpen bool // equality with either radial limit excludes the endpoint
	box            [3]loftFaceInterval
}

type revolveLoftFacesVerticalSide struct {
	*revolveLoftFacesConstantXSource
	face    *Face
	origins []FeatureRef
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
	yzAffine     *revolveLoftFacesVerticalAffine
	cells        []revolveLoftFacesVerticalCell
	hasRetained  bool
}

// At the certified band time, Y/Z are an orientation-preserving affine map
// of the live Sketch U/V coordinates when both cap denominators are constant.
// A nil record makes no claim about angular order or similarity.
type revolveLoftFacesVerticalAffine struct {
	offset [2]*big.Rat
	column [2][2]*big.Rat // [source U or V][world Y or Z]
	det    *big.Rat
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
	if !ok || face != section.face || !verticalOriginsMatch(face.Origins(), section.origins) {
		return nil, revolveLoftFacesVerticalRefusal, nil
	}
	payload, ok := proof.tooth.payload.(facetedPayload)
	if !ok || payload.loftFacesTrim != trim || !revolveLoftFacesTrimCurrent(proof.tooth, trim) ||
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
		origins: append([]FeatureRef(nil), section.origins...),
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
	payload, ok := proof.tooth.payload.(facetedPayload)
	if !ok || payload.loftFacesTrim != trim || !revolveLoftFacesTrimCurrent(proof.tooth, trim) ||
		!revolveLoftFacesSourceRefsCurrent(trim, proof.tooth, proof.sourceRefs) {
		return nil, false, nil
	}
	box, err := lfSourceBox(trim.source)
	if err != nil {
		return nil, false, err
	}
	band, time, ok := revolveLoftFacesConstantXBand(trim.maps, box, x)
	if !ok || len(proof.sourceRefs[band]) != len(trim.source.profile.Outer.Segments) {
		return nil, false, nil
	}
	result := &revolveLoftFacesConstantXSource{band: band,
		time: time, x: x, lower: lower, upper: upper}
	result.yzAffine, _ = revolveLoftFacesVerticalAffineMap(trim.maps[band],
		trim.maps[band+1], time)
	for segment, source := range trim.source.profile.Outer.Segments {
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
			cell, certified := revolveLoftFacesVerticalBezier(trim, segment, spanIndex,
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
	bound [][]FeatureRef) bool {
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

func revolveLoftFacesConstantXBand(maps []loftFaceMap, box [4]*big.Rat,
	x *big.Rat) (int, *big.Rat, bool) {
	if len(maps) < 2 || x == nil {
		return -1, nil, false
	}
	capX := make([]*big.Rat, len(maps))
	for i, mapped := range maps {
		if lfPositiveDenominator(mapped, box) != nil {
			return -1, nil, false
		}
		mappedX, ok := revolveLoftFacesConstantX(mapped)
		capX[i] = mappedX
		if !ok || i > 0 && capX[i].Cmp(capX[i-1]) <= 0 || capX[i].Cmp(x) == 0 {
			return -1, nil, false
		}
	}
	band := -1
	for i := 0; i+1 < len(capX); i++ {
		if capX[i].Cmp(x) < 0 && capX[i+1].Cmp(x) > 0 {
			band = i
			break
		}
	}
	if band < 0 {
		return -1, nil, false
	}
	time := lfDiv(lfSub(x, capX[band]), lfSub(capX[band+1], capX[band]))
	return band, time, true
}

func revolveLoftFacesVerticalLiveFace(tooth *Body, ref FeatureRef) *Face {
	if tooth == nil {
		return nil
	}
	payload, ok := tooth.payload.(facetedPayload)
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

func verticalOriginsMatch(actual, recorded []FeatureRef) bool {
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

// A linear-fractional X map is constant on the entire source rectangle
// exactly when its numerator is that constant times its denominator.
func revolveLoftFacesConstantX(mapped loftFaceMap) (*big.Rat, bool) {
	if mapped.g0 == nil || mapped.g0.Sign() <= 0 || mapped.constant[0] == nil ||
		mapped.q0[0] == nil || mapped.qu[0] == nil || mapped.qv[0] == nil ||
		mapped.gu == nil || mapped.gv == nil {
		return nil, false
	}
	ratio := lfDiv(mapped.q0[0], mapped.g0)
	if mapped.qu[0].Cmp(lfMul(ratio, mapped.gu)) != 0 ||
		mapped.qv[0].Cmp(lfMul(ratio, mapped.gv)) != 0 {
		return nil, false
	}
	return lfAdd(mapped.constant[0], ratio), true
}

func revolveLoftFacesVerticalAffineMap(first, last loftFaceMap,
	time *big.Rat) (*revolveLoftFacesVerticalAffine, bool) {
	if time == nil || time.Sign() < 0 || time.Cmp(lfOne()) > 0 {
		return nil, false
	}
	var mappedCaps [2]revolveLoftFacesVerticalAffine
	for i, mapped := range [...]loftFaceMap{first, last} {
		if mapped.g0 == nil || mapped.g0.Sign() <= 0 ||
			mapped.gu == nil || mapped.gu.Sign() != 0 ||
			mapped.gv == nil || mapped.gv.Sign() != 0 {
			return nil, false
		}
		for axis := range mappedCaps[i].offset {
			worldAxis := axis + 1
			if mapped.constant[worldAxis] == nil || mapped.q0[worldAxis] == nil ||
				mapped.qu[worldAxis] == nil || mapped.qv[worldAxis] == nil {
				return nil, false
			}
			mappedCaps[i].offset[axis] = lfAdd(mapped.constant[worldAxis],
				lfDiv(mapped.q0[worldAxis], mapped.g0))
			mappedCaps[i].column[0][axis] = lfDiv(mapped.qu[worldAxis], mapped.g0)
			mappedCaps[i].column[1][axis] = lfDiv(mapped.qv[worldAxis], mapped.g0)
		}
	}
	blend := func(a, b *big.Rat) *big.Rat {
		return lfAdd(lfMul(lfSub(lfOne(), time), a), lfMul(time, b))
	}
	result := &revolveLoftFacesVerticalAffine{}
	for axis := range result.offset {
		result.offset[axis] = blend(mappedCaps[0].offset[axis], mappedCaps[1].offset[axis])
		for sourceAxis := range result.column {
			result.column[sourceAxis][axis] = blend(mappedCaps[0].column[sourceAxis][axis],
				mappedCaps[1].column[sourceAxis][axis])
		}
	}
	result.det = lfSub(lfMul(result.column[0][0], result.column[1][1]),
		lfMul(result.column[0][1], result.column[1][0]))
	if result.det.Sign() <= 0 {
		return nil, false
	}
	return result, true
}

// The Cut keeps the strict outside of its cone. Intersect keeps the strict
// inside of its cone. On an X-constant plane these become radial limits.
func revolveLoftFacesVerticalLimits(trim *loftFacesTrimRecord,
	section revolveSectionSpan, x *big.Rat) (*big.Rat, *big.Rat, revolveLoftFacesVerticalOutcome) {
	if trim == nil || trim.toe == nil || trim.heel == nil || x == nil ||
		trim.toe.apex == nil || trim.heel.apex == nil ||
		trim.toe.far == nil || trim.heel.far == nil ||
		trim.toe.slope == nil || trim.heel.slope == nil ||
		trim.toe.slope.Sign() <= 0 || trim.heel.slope.Sign() <= 0 ||
		x.Cmp(trim.toe.far) <= 0 || x.Cmp(trim.heel.far) <= 0 ||
		x.Cmp(trim.heel.apex) >= 0 {
		return nil, nil, revolveLoftFacesVerticalRefusal
	}
	lower, upper := lfF(section.start.V), lfF(section.end.V)
	if lower.Cmp(upper) > 0 {
		lower, upper = upper, lower
	}
	if x.Cmp(trim.toe.apex) < 0 {
		toe := lfDiv(lfSub(trim.toe.apex, x), trim.toe.slope)
		if lower.Cmp(toe) < 0 {
			lower = toe
		}
	}
	heel := lfDiv(lfSub(trim.heel.apex, x), trim.heel.slope)
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
	ref FeatureRef, span freeform.BezierSpan) (revolveLoftFacesVerticalCell, bool) {
	cell := revolveLoftFacesVerticalCell{segment: segment, span: spanIndex,
		band: band, source: span, ref: ref}
	first, last := trim.maps[band], trim.maps[band+1]
	loQ := polynomial.BernsteinRowsAt(spiralRootPolynomial(first, last, span, lfZero(), lower), time)
	hiQ := polynomial.BernsteinRowsAt(spiralRootPolynomial(first, last, span, lfZero(), upper), time)
	var ok bool
	cell.lower, ok = revolveLoftFacesVerticalClassify(loQ)
	if !ok {
		return cell, false
	}
	cell.upper, ok = revolveLoftFacesVerticalClassify(hiQ)
	if !ok {
		return cell, false
	}
	for _, limit := range []*revolveLoftFacesVerticalLimit{&cell.lower, &cell.upper} {
		if limit.root.lo == nil {
			continue
		}
		limit.box, ok = spiralJunctionBox(first, last, span,
			limit.root.lo, limit.root.hi, time, time)
		if !ok {
			return cell, false
		}
	}
	cell.retained, ok = revolveLoftFacesVerticalPieces(first, last, span,
		time, loQ, hiQ, cell.lower.root, cell.upper.root)
	return cell, ok
}

// Bernstein control signs prove a strict side. Strict endpoint and
// derivative signs prove one transverse root and exclude all others.
func revolveLoftFacesVerticalClassify(q []*big.Rat) (revolveLoftFacesVerticalLimit, bool) {
	var result revolveLoftFacesVerticalLimit
	if len(q) < 2 || q[0].Sign() == 0 || q[len(q)-1].Sign() == 0 {
		return result, false
	}
	positive, negative := true, true
	for _, coefficient := range q {
		positive = positive && coefficient.Sign() > 0
		negative = negative && coefficient.Sign() < 0
	}
	if positive {
		result.side = 1
		return result, true
	}
	if negative {
		result.side = -1
		return result, true
	}
	signed := q
	if q[0].Sign() > 0 {
		signed = revolveLoftFacesNegateRow(q)
	}
	if !polynomial.BernsteinRowsCross([3][]*big.Rat{signed, signed, signed}) {
		return result, false
	}
	root := spiralIsolateRootN(signed, 80)
	if root.lo.Sign() <= 0 || root.hi.Cmp(lfOne()) >= 0 {
		return result, false
	}
	result.root = loftFaceInterval{lo: root.lo, hi: root.hi}
	result.before, result.after = q[0].Sign(), q[len(q)-1].Sign()
	return result, true
}

func revolveLoftFacesVerticalPieces(first, last loftFaceMap,
	span freeform.BezierSpan, time *big.Rat, loQ, hiQ []*big.Rat,
	loRoot, hiRoot loftFaceInterval) ([]revolveLoftFacesVerticalPiece, bool) {
	if loRoot.lo != nil && hiRoot.lo != nil &&
		loRoot.lo.Cmp(hiRoot.hi) <= 0 && hiRoot.lo.Cmp(loRoot.hi) <= 0 {
		return nil, false
	}
	endpoints := []*big.Rat{lfZero(), lfOne()}
	for _, root := range [...]loftFaceInterval{loRoot, hiRoot} {
		if root.lo != nil {
			endpoints = append(endpoints, root.lo, root.hi)
		}
	}
	sortRats(endpoints)
	for i := 1; i < len(endpoints); i++ {
		if endpoints[i].Cmp(endpoints[i-1]) == 0 {
			return nil, false
		}
	}
	var pieces []revolveLoftFacesVerticalPiece
	for i := 0; i+1 < len(endpoints); i++ {
		a, z := endpoints[i], endpoints[i+1]
		if verticalWithinRoot(a, z, loRoot) || verticalWithinRoot(a, z, hiRoot) {
			continue
		}
		mid := lfMul(lfAdd(a, z), big.NewRat(1, 2))
		if polynomial.BernsteinSign(loQ, mid) <= 0 || polynomial.BernsteinSign(hiQ, mid) >= 0 {
			continue
		}
		box, ok := spiralJunctionBox(first, last, span, a, z, time, time)
		if !ok {
			return nil, false
		}
		loLower, loUpper := polynomial.BernsteinSign(loQ, a), polynomial.BernsteinSign(hiQ, a)
		hiLower, hiUpper := polynomial.BernsteinSign(loQ, z), polynomial.BernsteinSign(hiQ, z)
		if loLower < 0 || loUpper > 0 || hiLower < 0 || hiUpper > 0 {
			return nil, false
		}
		pieces = append(pieces, revolveLoftFacesVerticalPiece{
			source: loftFaceInterval{lo: a, hi: z},
			loOpen: loLower == 0 || loUpper == 0,
			hiOpen: hiLower == 0 || hiUpper == 0,
			box:    box})
	}
	return pieces, true
}

func verticalWithinRoot(a, z *big.Rat, root loftFaceInterval) bool {
	return root.lo != nil && a.Cmp(root.lo) >= 0 && z.Cmp(root.hi) <= 0
}

func sortRats(values []*big.Rat) {
	for i := 1; i < len(values); i++ {
		for j := i; j > 0 && values[j].Cmp(values[j-1]) < 0; j-- {
			values[j], values[j-1] = values[j-1], values[j]
		}
	}
}

// A live whole arc may only be used when its authenticated map gives one
// exact squared radius across all of its source parameters at this time.
func revolveLoftFacesVerticalArc(ctx context.Context, trim *loftFacesTrimRecord,
	x *big.Rat, segment, band int, time, lower, upper *big.Rat,
	ref FeatureRef) (revolveLoftFacesVerticalCell, bool) {
	cell := revolveLoftFacesVerticalCell{segment: segment, band: band, ref: ref}
	unit := revolveSectionSpan{start: Point2{V: 1}, end: Point2{U: 1, V: 1}}
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
	cell.lower.side, cell.upper.side = loSign, hiSign
	if loSign > 0 && hiSign < 0 {
		box, valid := revolveLoftFacesVerticalArcBox(radial.arc, trim, band, time)
		if !valid || box[0].lo.Cmp(x) != 0 || box[0].hi.Cmp(x) != 0 {
			return cell, false
		}
		cell.retained = []revolveLoftFacesVerticalPiece{{
			source: loftFaceInterval{lo: lfZero(), hi: lfOne()}, box: box}}
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
	sourceBox, err := lfSourceBox(trim.source)
	if err != nil || lfF(u.Lo).Cmp(sourceBox[0]) < 0 ||
		lfF(u.Hi).Cmp(sourceBox[1]) > 0 ||
		lfF(v.Lo).Cmp(sourceBox[2]) < 0 || lfF(v.Hi).Cmp(sourceBox[3]) > 0 {
		return box, false
	}
	corners := lfTileCorners(lfF(u.Lo), lfF(u.Hi), lfF(v.Lo), lfF(v.Hi))
	first, last := mappedArcCapRanges(trim.maps[band], corners),
		mappedArcCapRanges(trim.maps[band+1], corners)
	for axis := range box {
		box[axis] = mappedArcBlend(first[axis], last[axis], time)
	}
	return box, true
}
