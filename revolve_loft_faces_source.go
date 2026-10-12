package decad

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// revolveLoftFacesCrossing binds one source fitted curve to the unique span
// that crosses the candidate conical Revolve face in every mapped band.
type revolveLoftFacesCrossing struct {
	segment int
	span    freeform.BezierSpan
	index   int
	first   int // signed implicit side before the crossing in profile walk order
	last    int
	caps    []spiralRootCrossing
	x       []loftFaceInterval
	faces   []*Face
}

// revolveLoftFacesSourceCertificate describes a certified curved contact
// before any held seam is built. Source faces are bound by their producer
// origins; the certificate never promotes a faceted edge to a Sketch curve.
type revolveLoftFacesSourceCertificate struct {
	blank      *Body
	tooth      *Body
	section    revolveSectionCertificate
	trim       *loftFacesTrimRecord
	rootM      *big.Rat
	rootB      *big.Rat
	toeX       *big.Rat
	heelX      *big.Rat
	crossings  [2]revolveLoftFacesCrossing
	sides      []int
	sourceRefs [][]FeatureRef
	toeFace    *Face
	heelFace   *Face
}

func certifyRevolveLoftFacesSource(ctx context.Context, blank, tooth *Body) (
	*revolveLoftFacesSourceCertificate, bool, error,
) {
	if ctx == nil || blank == nil || tooth == nil {
		return nil, false, nil
	}
	p, ok := tooth.payload.(facetedPayload)
	if !ok || p.xform != r3.Identity() || p.loftFacesTrim == nil ||
		!revolveLoftFacesTrimCurrent(p, p.loftFacesTrim) {
		return nil, false, nil
	}
	trim := p.loftFacesTrim
	var selected *revolveLoftFacesSourceCertificate
	for _, section := range revolveRootCandidates(blank) {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		proof, ok, err := certifyRevolveLoftFacesAgainstRoot(ctx, blank, tooth, trim, section)
		if err != nil {
			return nil, false, err
		}
		if !ok {
			continue
		}
		if selected != nil {
			return nil, false, nil
		}
		selected = proof
	}
	return selected, selected != nil, nil
}

// The source gate precedes a separate held-seam construction. It never asks
// the mesh Boolean to bless a contact that its ordinary chain-depth gate
// cannot resolve.
func tryRevolveLoftFacesUnion(ctx context.Context, op meshbool.OperationKind,
	a, b *Body) (bool, error) {
	if op != meshbool.OpUnion {
		return false, nil
	}
	proof, ok, err := certifyRevolveLoftFacesSource(ctx, a, b)
	if err != nil {
		return true, err
	}
	if !ok {
		proof, ok, err = certifyRevolveLoftFacesSource(ctx, b, a)
		if err != nil {
			return true, err
		}
	}
	if !ok || proof == nil {
		return false, nil
	}
	return true, fmt.Errorf("%w: certified curved LoftFaces contact needs a held-seam builder", ErrUnsupported)
}

func revolveLoftFacesTrimCurrent(p facetedPayload, trim *loftFacesTrimRecord) bool {
	if trim == nil || trim.source == nil || !pointSourceCurrent(trim.source) ||
		trim.toe == nil || trim.heel == nil || len(trim.maps) < 3 || len(trim.maps) > 64 ||
		len(trim.bounds) != len(trim.maps) ||
		!orderedLoftFacesConeLimits(trim.toe, trim.heel, trim.base) ||
		!trim.base.loftFacesPrimitive || trim.base.loftFacesCap == nil ||
		trim.base.loftFacesCap.source != trim.source ||
		len(trim.base.loftFacesCap.maps) != len(trim.maps) ||
		len(trim.base.loftFacesCap.bounds) != len(trim.bounds) ||
		len(trim.stationGroup) != len(trim.source.sourceStation) ||
		len(trim.base.loftFacesCap.stationGroup) != len(trim.stationGroup) ||
		len(trim.source.profile.Holes) != 0 ||
		len(trim.source.profile.Outer.Segments) < 4 ||
		len(trim.source.profile.Outer.Segments) > 128 ||
		len(trim.source.profile.Outer.Segments) != len(trim.source.source.entities) ||
		!spiralTrimCapGroupsCurrent(p, trim) {
		return false
	}
	if !slices.Equal(trim.stationGroup, trim.base.loftFacesCap.stationGroup) {
		return false
	}
	for i, bound := range trim.bounds {
		if !revolveLoftFacesSameCapBound(bound, trim.base.loftFacesCap.bounds[i]) {
			return false
		}
	}
	for _, edge := range trim.source.source.profile.Outer {
		if edge.Partial {
			return false
		}
	}
	box, err := lfSourceBox(trim.source)
	if err != nil {
		return false
	}
	for i, mapped := range trim.maps {
		if lfPositiveDenominator(mapped, box) != nil ||
			mapped != trim.base.loftFacesCap.maps[i] {
			return false
		}
	}
	return true
}

func revolveLoftFacesSameCapBound(a, b loftFaceCapBounds) bool {
	return a.stationRound == b.stationRound &&
		slices.Equal(a.projectiveDeparture, b.projectiveDeparture) &&
		slices.Equal(a.polygonSpeed, b.polygonSpeed) &&
		slices.Equal(a.sourceSpeed, b.sourceSpeed) &&
		slices.Equal(a.sourceDeparture, b.sourceDeparture)
}

func certifyRevolveLoftFacesAgainstRoot(ctx context.Context, blank, tooth *Body,
	trim *loftFacesTrimRecord, section revolveSectionCertificate) (
	*revolveLoftFacesSourceCertificate, bool, error,
) {
	m, b := section.meridianEquation()
	if !spiralSourcePhysicalSector(trim, m, b) {
		return nil, false, nil
	}
	toeX, ok := spiralRootToolX(m, b, trim.toe)
	if !ok {
		return nil, false, nil
	}
	heelX, ok := spiralRootToolX(m, b, trim.heel)
	if !ok || toeX.Cmp(heelX) >= 0 ||
		lfF(section.rootStart.U).Cmp(toeX) <= 0 ||
		lfF(section.rootEnd.U).Cmp(heelX) >= 0 ||
		!spiralFiniteConeSide(trim.toe, section.rootStart, section.rootEnd, 1) ||
		!spiralFiniteConeSide(trim.heel, section.rootStart, section.rootEnd, -1) ||
		!revolveLoftFacesCapsAvoidToolRootCircles(trim, m, b, toeX, heelX) {
		return nil, false, nil
	}
	segments := trim.source.profile.Outer.Segments
	proof := &revolveLoftFacesSourceCertificate{blank: blank, tooth: tooth,
		section: section, trim: trim, rootM: m, rootB: b, toeX: toeX, heelX: heelX,
		sides: make([]int, len(segments))}
	start, end := make([]int, len(segments)), make([]int, len(segments))
	for i, segment := range segments {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		if fit, isFit := segment.(fitSplineSeg); isFit {
			crossing, side, ok := revolveLoftFacesFitSide(trim, m, b, i, fit)
			if !ok {
				return nil, false, nil
			}
			if crossing != nil {
				index := 0
				if proof.crossings[0].span != nil {
					index = 1
				}
				if index >= len(proof.crossings) {
					return nil, false, nil
				}
				proof.crossings[index] = *crossing
				start[i], end[i] = crossing.first, crossing.last
				continue
			}
			start[i], end[i], proof.sides[i] = side, side, side
			continue
		}
		if !revolveLoftFacesSegmentKind(trim, i) {
			return nil, false, nil
		}
		side := revolveLoftFacesStationSide(ctx, trim, m, b, i)
		if side == 0 {
			return nil, false, nil
		}
		start[i], end[i], proof.sides[i] = side, side, side
	}
	if proof.crossings[0].span == nil || proof.crossings[1].span == nil {
		return nil, false, nil
	}
	for i := range segments {
		if end[i] == 0 || end[i] != start[(i+1)%len(segments)] {
			return nil, false, nil
		}
	}
	for _, crossing := range proof.crossings {
		if crossing.x[0].hi.Cmp(toeX) >= 0 ||
			crossing.x[len(crossing.x)-1].lo.Cmp(heelX) <= 0 {
			return nil, false, nil
		}
	}
	refs, ok := revolveLoftFacesSourceRefs(trim, tooth)
	if !ok {
		return nil, false, nil
	}
	proof.sourceRefs = refs
	p, _ := tooth.payload.(facetedPayload)
	faces := tooth.Faces()
	for i := range proof.crossings {
		crossing := &proof.crossings[i]
		crossing.faces = make([]*Face, len(refs))
		retained := false
		for band := range refs {
			ref := refs[band][crossing.segment]
			index, valid := revolveLoftFacesRefFaceIndex(p, ref, len(faces))
			face, originValid := revolveLoftFacesUniqueOriginFace(tooth, ref)
			if !valid || !originValid || (index >= 0) != (face != nil) ||
				index >= 0 && faces[index] != face {
				return nil, false, nil
			}
			crossing.faces[band] = face
			retained = retained || index >= 0
		}
		if !retained {
			return nil, false, nil
		}
	}
	proof.toeFace = uniqueResultSourceFace(tooth, p.loftFacesTrim.toeCapGroup)
	proof.heelFace = uniqueResultSourceFace(tooth, p.loftFacesTrim.heelCapGroup)
	if proof.toeFace == nil || proof.heelFace == nil || proof.toeFace == proof.heelFace {
		return nil, false, nil
	}
	return proof, true, nil
}

// The first cap lies wholly before its tool/root circle. The last cap's
// supporting plane misses the full heel circle, including its interior.
func revolveLoftFacesCapsAvoidToolRootCircles(trim *loftFacesTrimRecord,
	m, b, toeX, heelX *big.Rat) bool {
	box, err := lfSourceBox(trim.source)
	if err != nil || len(trim.maps) < 2 {
		return false
	}
	first, last := trim.maps[0], trim.maps[len(trim.maps)-1]
	if lfPositiveDenominator(first, box) != nil || lfPositiveDenominator(last, box) != nil {
		return false
	}
	corners := lfBoxCorners(box)
	xAxis := loftFaceRatVec{lfOne(), lfZero(), lfZero()}
	return lfScalarRange(first, xAxis, corners).hi.Cmp(toeX) < 0 &&
		spiralCapPlaneAvoidsRootCircle(last, corners, m, b, heelX)
}

func revolveLoftFacesSegmentKind(trim *loftFacesTrimRecord, index int) bool {
	segment := trim.source.profile.Outer.Segments[index]
	entity := trim.source.source.entities[index]
	switch entity.(type) {
	case *sketch.Line:
		_, ok := segment.(lineSeg)
		return ok
	case *sketch.Arc:
		_, ok := segment.(arcSeg)
		return ok
	default:
		return false
	}
}

// The source station departure encloses each complete nonfitted edge. A
// crossing or an undecidable cell refuses; no sampled sign grants admission.
func revolveLoftFacesStationSide(ctx context.Context, trim *loftFacesTrimRecord,
	m, b *big.Rat, segment int) int {
	source := trim.source
	n := len(source.sourceStation)
	if n == 0 || len(trim.stationGroup) != n {
		return 0
	}
	inside, outside, seen := true, true, false
	for band := 0; band+1 < len(trim.maps); band++ {
		lo, hi := trim.bounds[band], trim.bounds[band+1]
		if len(lo.sourceDeparture) != n || len(hi.sourceDeparture) != n ||
			len(lo.projectiveDeparture) != n || len(hi.projectiveDeparture) != n {
			return 0
		}
		for j, station := range source.sourceStation {
			if trim.stationGroup[j] != segment {
				continue
			}
			if j%64 == 0 && ctx.Err() != nil {
				return 0
			}
			seen = true
			departure := max(
				proofbound.AbsSumUpper(lo.sourceDeparture[j], lo.projectiveDeparture[j]),
				proofbound.AbsSumUpper(hi.sourceDeparture[j], hi.projectiveDeparture[j]))
			if proofbound.IsNonFinite(departure) || departure < 0 {
				return 0
			}
			next := source.sourceStation[(j+1)%n]
			if inside && !spiralCellSide(trim.maps[band], trim.maps[band+1], station,
				next, lfF(departure), m, b, false, lfZero(), lfOne(), 8) {
				inside = false
			}
			if outside && !spiralCellSide(trim.maps[band], trim.maps[band+1], station,
				next, lfF(departure), m, b, true, lfZero(), lfOne(), 8) {
				outside = false
			}
			if !inside && !outside {
				return 0
			}
		}
	}
	if !seen || inside == outside {
		return 0
	}
	if inside {
		return -1
	}
	return 1
}

func revolveLoftFacesSourceRefs(trim *loftFacesTrimRecord, tooth *Body) ([][]FeatureRef, bool) {
	n := len(trim.source.profile.Outer.Segments)
	if len(trim.base.groups) != (len(trim.maps)-1)*n+2 {
		return nil, false
	}
	refs := make([][]FeatureRef, len(trim.maps)-1)
	var producer producerID
	for band := range refs {
		refs[band] = make([]FeatureRef, n)
		for segment := range n {
			group := trim.base.groups[band*n+segment]
			if len(group.origins) != 1 ||
				group.origins[0].Role != fmt.Sprintf("side(%d,%d)", band, segment) {
				return nil, false
			}
			ref := group.origins[0]
			if band == 0 && segment == 0 {
				producer = ref.producer
			} else if ref.producer != producer {
				return nil, false
			}
			refs[band][segment] = ref
		}
	}
	for _, face := range tooth.Faces() {
		for _, origin := range face.Origins() {
			if origin.producer != producer {
				continue
			}
			found := false
			for _, band := range refs {
				for _, ref := range band {
					found = found || ref == origin
				}
			}
			if !found {
				return nil, false
			}
		}
	}
	return refs, true
}

func revolveLoftFacesUniqueOriginFace(body *Body, want FeatureRef) (*Face, bool) {
	var result *Face
	for _, face := range body.Faces() {
		for _, origin := range face.Origins() {
			if origin != want {
				continue
			}
			if face.body != body || result != nil {
				return nil, false
			}
			result = face
		}
	}
	return result, true
}

func revolveLoftFacesRefFaceIndex(p facetedPayload, want FeatureRef, faceCount int) (int, bool) {
	if len(p.src) != len(p.tris) || len(p.faceOf) != len(p.tris) {
		return -1, false
	}
	index := -1
	for i, group := range p.src {
		if group < 0 || group >= len(p.groups) {
			return -1, false
		}
		if slices.Contains(p.groups[group].origins, want) {
			if p.faceOf[i] < 0 || p.faceOf[i] >= faceCount ||
				index >= 0 && index != p.faceOf[i] {
				return -1, false
			}
			index = p.faceOf[i]
		}
	}
	return index, true
}
