package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
)

// revolveLoftFacesCrossing binds one source fitted curve to the unique span
// that crosses the candidate conical Revolve face in every mapped band.
type revolveLoftFacesCrossing struct {
	segment int
	span    freeform.BezierSpan
	index   int
	first   int // signed implicit side before the crossing in profile walk order
	last    int
	caps    []loftface.RootCrossing
	x       []loftFaceInterval
	faces   []*Face
}

// revolveLoftFacesSourceCertificate describes a certified curved contact
// before any held seam is built. Source faces are bound by their producer
// origins; the certificate never promotes a faceted edge to a Sketch curve.
type revolveLoftFacesSourceCertificate struct {
	blank      *Body
	tooth      *Body
	toolProof  *revolveLoftFacesToolPayloadSnapshot
	section    revolveSectionCertificate
	trim       *loftFacesTrimRecord
	rootM      *big.Rat
	rootB      *big.Rat
	toeX       *big.Rat
	heelX      *big.Rat
	crossings  [2]revolveLoftFacesCrossing
	sides      []int
	sourceRefs [][]provenance.Ref
	toeFace    *Face
	heelFace   *Face
}

func certifyRevolveLoftFacesSource(ctx context.Context, blank, tooth *Body) (
	*revolveLoftFacesSourceCertificate, bool, error,
) {
	if ctx == nil || blank == nil || tooth == nil {
		return nil, false, nil
	}
	p, ok := tooth.n.Payload.(facetedPayload)
	if !ok || p.Xform != r3.Identity() || p.LoftFacesTrim == nil ||
		!revolveLoftFacesTrimCurrent(tooth, p.LoftFacesTrim) {
		return nil, false, nil
	}
	trim := p.LoftFacesTrim
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
	if selected != nil {
		selected.toolProof = newRevolveLoftFacesToolPayloadSnapshot(tooth, p)
	}
	return selected, selected != nil, nil
}

// The source gate precedes a separate held-seam construction. An unfinished
// source proof cannot veto the ordinary mesh Boolean's independent gates.
func tryRevolveLoftFacesUnion(ctx context.Context, op meshbool.OperationKind,
	a, b *Body) (bool, error) {
	if op != meshbool.OpUnion {
		return false, nil
	}
	proof, ok, err := certifyRevolveLoftFacesSource(ctx, a, b)
	if err != nil {
		return revolveLoftFacesSourceProofError(ctx, err)
	}
	if !ok {
		proof, ok, err = certifyRevolveLoftFacesSource(ctx, b, a)
		if err != nil {
			return revolveLoftFacesSourceProofError(ctx, err)
		}
	}
	if !ok || proof == nil {
		return false, nil
	}
	_, certified, manifestErr := revolveLoftFacesCensus(ctx, proof)
	if manifestErr != nil {
		return revolveLoftFacesSourceProofError(ctx, manifestErr)
	}
	if !certified {
		return false, nil
	}
	return true, fmt.Errorf("%w: certified curved LoftFaces contact needs a held-seam builder", ErrUnsupported)
}

func revolveLoftFacesSourceProofError(ctx context.Context, err error) (bool, error) {
	if ctx.Err() != nil || errors.Is(err, context.Canceled) ||
		errors.Is(err, context.DeadlineExceeded) {
		return true, err
	}
	return false, nil
}

func revolveLoftFacesTrimCurrent(tooth *Body, trim *loftFacesTrimRecord) bool {
	if tooth == nil {
		return false
	}
	p, ok := tooth.n.Payload.(facetedPayload)
	if !ok || p.LoftFacesTrim != trim {
		return false
	}
	// A cone-tangent tooth is decided by the first-join certificate only;
	// this generic route defers it explicitly.
	if trim != nil && (trim.Tangent != nil ||
		(trim.Base.LoftFacesCap != nil && trim.Base.LoftFacesCap.Tangent != nil)) {
		return false
	}
	if p.Xform != r3.Identity() || trim == nil || trim.Source == nil ||
		!facetedrecord.PointSourceCurrent(trim.Source) ||
		!revolveLoftFacesConeCurrent(bodyHandle(trim.ToeTool), trim.Toe) ||
		!revolveLoftFacesConeCurrent(bodyHandle(trim.HeelTool), trim.Heel) ||
		len(trim.Maps) < 3 || len(trim.Maps) > 64 ||
		len(trim.Bounds) != len(trim.Maps) ||
		!orderedLoftFacesConeLimits(trim.Toe, trim.Heel, trim.Base) ||
		!trim.Base.LoftFacesPrimitive || trim.Base.LoftFacesCap == nil ||
		trim.Base.LoftFacesCap.Source != trim.Source ||
		trim.Source.SourceBoxIssued != trim.Source.SourceBox ||
		len(trim.Base.LoftFacesCap.Maps) != len(trim.Maps) ||
		len(trim.Base.LoftFacesCap.Bounds) != len(trim.Bounds) ||
		len(trim.StationGroup) != len(trim.Source.SourceStation) ||
		len(trim.Base.LoftFacesCap.StationGroup) != len(trim.StationGroup) ||
		len(trim.Source.Profile.Holes) != 0 ||
		len(trim.Source.Profile.Outer.Segments) < 4 ||
		len(trim.Source.Profile.Outer.Segments) > 128 ||
		len(trim.Source.Profile.Outer.Segments) != len(trim.Source.Source.Entities) ||
		!facetedtopology.CapGroupsPresent(p.Src, p.FaceOf, len(p.Groups), trim.ToeCapGroup, trim.HeelCapGroup) ||
		!facetedtopology.CapGroupsPure(p.Src, p.FaceOf, len(p.Tris), len(tooth.Faces()),
			trim.ToeCapGroup, trim.HeelCapGroup) {
		return false
	}
	if !slices.Equal(trim.StationGroup, trim.Base.LoftFacesCap.StationGroup) {
		return false
	}
	for i, bound := range trim.Bounds {
		if !revolveLoftFacesSameCapBound(bound, trim.Base.LoftFacesCap.Bounds[i]) {
			return false
		}
	}
	for _, edge := range trim.Source.Source.Profile.Outer {
		if edge.Partial {
			return false
		}
	}
	box, err := revolveloftfaces.LfSourceBox(trim.Source)
	if err != nil {
		return false
	}
	for i, mapped := range trim.Maps {
		if loftface.PositiveDenominator(mapped, box) != nil ||
			mapped != trim.Base.LoftFacesCap.Maps[i] {
			return false
		}
	}
	return true
}

// Boolean source proofs read exact cone parameters from the live tool
// meridians. The retained trim scalars alone are mutable cached data.
func revolveLoftFacesConeCurrent(tool *Body, recorded *pointConeLimit) bool {
	if tool == nil || recorded == nil || recorded.Apex == nil ||
		recorded.Far == nil || recorded.Slope == nil {
		return false
	}
	live, ok := exactPointCone(tool)
	return ok && recorded.Apex.Cmp(live.Apex) == 0 &&
		recorded.Far.Cmp(live.Far) == 0 && recorded.Slope.Cmp(live.Slope) == 0
}

func revolveLoftFacesSameCapBound(a, b loftFaceCapBounds) bool {
	return a.StationRound == b.StationRound &&
		slices.Equal(a.ProjectiveDeparture, b.ProjectiveDeparture) &&
		slices.Equal(a.PolygonSpeed, b.PolygonSpeed) &&
		slices.Equal(a.SourceSpeed, b.SourceSpeed) &&
		slices.Equal(a.SourceDeparture, b.SourceDeparture)
}

func certifyRevolveLoftFacesAgainstRoot(ctx context.Context, blank, tooth *Body,
	trim *loftFacesTrimRecord, section revolveSectionCertificate) (
	*revolveLoftFacesSourceCertificate, bool, error,
) {
	m, b := section.MeridianEquation()
	if !spiralsector.SourcePhysicalSector(trim, m, b) {
		return nil, false, nil
	}
	toeX, ok := spiralsector.RootToolX(m, b, trim.Toe)
	if !ok {
		return nil, false, nil
	}
	heelX, ok := spiralsector.RootToolX(m, b, trim.Heel)
	if !ok || toeX.Cmp(heelX) >= 0 ||
		lfF(section.RootStart.U).Cmp(toeX) <= 0 ||
		lfF(section.RootEnd.U).Cmp(heelX) >= 0 ||
		!spiralsector.FiniteConeSide(trim.Toe, section.RootStart, section.RootEnd, 1) ||
		!spiralsector.FiniteConeSide(trim.Heel, section.RootStart, section.RootEnd, -1) ||
		!revolveloftfaces.CapsAvoidToolRootCircles(trim, m, b, toeX, heelX) {
		return nil, false, nil
	}
	segments := trim.Source.Profile.Outer.Segments
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
		if !revolveloftfaces.SegmentKind(trim, i) {
			return nil, false, nil
		}
		side := spiralsector.StationSide(ctx, trim, m, b, i)
		if side == 0 {
			return nil, false, nil
		}
		if _, arc := segment.(arcSeg); arc {
			finiteSide, certified := revolveloftfaces.MappedArcFiniteSide(ctx, trim, i,
				section.Spans[section.RootSpan])
			if !certified || finiteSide != loftface.ArcMiss && finiteSide != side {
				return nil, false, nil
			}
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
		if crossing.x[0].Hi.Cmp(toeX) >= 0 ||
			crossing.x[len(crossing.x)-1].Lo.Cmp(heelX) <= 0 {
			return nil, false, nil
		}
	}
	refs, ok := revolveLoftFacesSourceRefs(trim, tooth)
	if !ok {
		return nil, false, nil
	}
	proof.sourceRefs = refs
	p, _ := tooth.n.Payload.(facetedPayload)
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
	proof.toeFace = uniqueResultSourceFace(tooth, p.LoftFacesTrim.ToeCapGroup)
	proof.heelFace = uniqueResultSourceFace(tooth, p.LoftFacesTrim.HeelCapGroup)
	if proof.toeFace == nil || proof.heelFace == nil || proof.toeFace == proof.heelFace {
		return nil, false, nil
	}
	return proof, true, nil
}

func revolveLoftFacesSourceRefs(trim *loftFacesTrimRecord, tooth *Body) ([][]provenance.Ref, bool) {
	n := len(trim.Source.Profile.Outer.Segments)
	if len(trim.Base.Groups) != (len(trim.Maps)-1)*n+2 {
		return nil, false
	}
	refs := make([][]provenance.Ref, len(trim.Maps)-1)
	var producer producerID
	for band := range refs {
		refs[band] = make([]provenance.Ref, n)
		for segment := range n {
			group := trim.Base.Groups[band*n+segment]
			if len(group.Origins) != 1 ||
				group.Origins[0].Role != fmt.Sprintf("side(%d,%d)", band, segment) {
				return nil, false
			}
			ref := group.Origins[0]
			if band == 0 && segment == 0 {
				producer = ref.Producer
			} else if ref.Producer != producer {
				return nil, false
			}
			refs[band][segment] = ref
		}
	}
	for _, face := range tooth.Faces() {
		for _, origin := range face.originRefs() {
			if origin.Producer != producer {
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

func revolveLoftFacesUniqueOriginFace(body *Body, want provenance.Ref) (*Face, bool) {
	var result *Face
	for _, face := range body.Faces() {
		for _, origin := range face.originRefs() {
			if origin != want {
				continue
			}
			if face.n.Body != bodyNode(body) || result != nil {
				return nil, false
			}
			result = face
		}
	}
	return result, true
}

func revolveLoftFacesRefFaceIndex(p facetedPayload, want provenance.Ref, faceCount int) (int, bool) {
	if len(p.Src) != len(p.Tris) || len(p.FaceOf) != len(p.Tris) {
		return -1, false
	}
	index := -1
	for i, group := range p.Src {
		if group < 0 || group >= len(p.Groups) {
			return -1, false
		}
		if slices.Contains(p.Groups[group].Origins, want) {
			if p.FaceOf[i] < 0 || p.FaceOf[i] >= faceCount ||
				index >= 0 && index != p.FaceOf[i] {
				return -1, false
			}
			index = p.FaceOf[i]
		}
	}
	return index, true
}
