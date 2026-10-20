package revolvecontact

import (
	"context"
	"errors"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// Crossing binds one source fitted curve to the unique span
// that crosses the candidate conical Revolve face in every mapped band.
type Crossing struct {
	Segment int
	Span    freeform.BezierSpan
	Index   int
	First   int // signed implicit side before the crossing in profile walk order
	Last    int
	Caps    []loftface.RootCrossing
	X       []loftface.Interval
	Faces   []*topo.Face
}

// SourceCertificate describes a certified curved contact
// before any held seam is built. Source faces are bound by their producer
// origins; the certificate never promotes a faceted edge to a Sketch curve.
type SourceCertificate struct {
	Blank      *topo.Body
	Tooth      *topo.Body
	ToolProof  *ToolPayloadSnapshot
	Section    facetedrecord.RevolveSectionCertificate
	Trim       *facetedrecord.LoftFacesTrimRecord
	RootM      *big.Rat
	RootB      *big.Rat
	ToeX       *big.Rat
	HeelX      *big.Rat
	Crossings  [2]Crossing
	Sides      []int
	SourceRefs [][]provenance.Ref
	ToeFace    *topo.Face
	HeelFace   *topo.Face
}

func CertifySource(ctx context.Context, blank, tooth *topo.Body) (
	*SourceCertificate, bool, error,
) {
	if ctx == nil || blank == nil || tooth == nil {
		return nil, false, nil
	}
	p, ok := tooth.Payload.(facetedrecord.Payload)
	if !ok || p.Xform != r3.Identity() || p.LoftFacesTrim == nil ||
		!TrimCurrent(tooth, p.LoftFacesTrim) {
		return nil, false, nil
	}
	trim := p.LoftFacesTrim
	var selected *SourceCertificate
	for _, section := range revolveshell.RootCandidates(blank) {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		proof, ok, err := CertifyAgainstRoot(ctx, blank, tooth, trim, section)
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
		selected.ToolProof = NewToolPayloadSnapshot(tooth, p)
	}
	return selected, selected != nil, nil
}

// The source gate precedes a separate held-seam construction. An unfinished
// source proof cannot veto the ordinary mesh Boolean's independent gates.
func TryUnion(ctx context.Context, op meshbool.OperationKind,
	a, b *topo.Body) (bool, error) {
	if op != meshbool.OpUnion {
		return false, nil
	}
	proof, ok, err := CertifySource(ctx, a, b)
	if err != nil {
		return SourceProofError(ctx, err)
	}
	if !ok {
		proof, ok, err = CertifySource(ctx, b, a)
		if err != nil {
			return SourceProofError(ctx, err)
		}
	}
	if !ok || proof == nil {
		return false, nil
	}
	_, certified, manifestErr := Census(ctx, proof)
	if manifestErr != nil {
		return SourceProofError(ctx, manifestErr)
	}
	if !certified {
		return false, nil
	}
	return true, fmt.Errorf("%w: certified curved LoftFaces contact needs a held-seam builder", decaderr.ErrUnsupported)
}

func SourceProofError(ctx context.Context, err error) (bool, error) {
	if ctx.Err() != nil || errors.Is(err, context.Canceled) ||
		errors.Is(err, context.DeadlineExceeded) {
		return true, err
	}
	return false, nil
}

func TrimCurrent(tooth *topo.Body, trim *facetedrecord.LoftFacesTrimRecord) bool {
	if tooth == nil {
		return false
	}
	p, ok := tooth.Payload.(facetedrecord.Payload)
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
		!ConeCurrent(trim.ToeTool, trim.Toe) ||
		!ConeCurrent(trim.HeelTool, trim.Heel) ||
		len(trim.Maps) < 3 || len(trim.Maps) > 64 ||
		len(trim.Bounds) != len(trim.Maps) ||
		!orderedConeLimits(trim.Toe, trim.Heel, trim.Base) ||
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
		!facetedtopology.CapGroupsPure(p.Src, p.FaceOf, len(p.Tris), len(topobuild.Faces(tooth)),
			trim.ToeCapGroup, trim.HeelCapGroup) {
		return false
	}
	if !slices.Equal(trim.StationGroup, trim.Base.LoftFacesCap.StationGroup) {
		return false
	}
	for i, bound := range trim.Bounds {
		if !SameCapBound(bound, trim.Base.LoftFacesCap.Bounds[i]) {
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
func ConeCurrent(tool *topo.Body, recorded *facetedrecord.PointConeLimit) bool {
	if tool == nil || recorded == nil || recorded.Apex == nil ||
		recorded.Far == nil || recorded.Slope == nil {
		return false
	}
	live, ok := revolveplan.PointConeSource(tool)
	return ok && recorded.Apex.Cmp(live.Apex) == 0 &&
		recorded.Far.Cmp(live.Far) == 0 && recorded.Slope.Cmp(live.Slope) == 0
}

func SameCapBound(a, b facetedrecord.LoftFaceCapBounds) bool {
	return a.StationRound == b.StationRound &&
		slices.Equal(a.ProjectiveDeparture, b.ProjectiveDeparture) &&
		slices.Equal(a.PolygonSpeed, b.PolygonSpeed) &&
		slices.Equal(a.SourceSpeed, b.SourceSpeed) &&
		slices.Equal(a.SourceDeparture, b.SourceDeparture)
}

func CertifyAgainstRoot(ctx context.Context, blank, tooth *topo.Body,
	trim *facetedrecord.LoftFacesTrimRecord, section facetedrecord.RevolveSectionCertificate) (
	*SourceCertificate, bool, error,
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
	proof := &SourceCertificate{Blank: blank, Tooth: tooth,
		Section: section, Trim: trim, RootM: m, RootB: b, ToeX: toeX, HeelX: heelX,
		Sides: make([]int, len(segments))}
	start, end := make([]int, len(segments)), make([]int, len(segments))
	for i, segment := range segments {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		if fit, isFit := segment.(sectionrecord.FitSplineSeg); isFit {
			fitCrossing, side, ok := revolveloftfaces.FitSide(trim, m, b, i, fit)
			if !ok {
				return nil, false, nil
			}
			if fitCrossing != nil {
				crossing := Crossing{
					Segment: fitCrossing.Segment, Span: fitCrossing.Span,
					Index: fitCrossing.Index, First: fitCrossing.First, Last: fitCrossing.Last,
					Caps: fitCrossing.Caps, X: fitCrossing.X,
				}
				index := 0
				if proof.Crossings[0].Span != nil {
					index = 1
				}
				if index >= len(proof.Crossings) {
					return nil, false, nil
				}
				proof.Crossings[index] = crossing
				start[i], end[i] = crossing.First, crossing.Last
				continue
			}
			start[i], end[i], proof.Sides[i] = side, side, side
			continue
		}
		if !revolveloftfaces.SegmentKind(trim, i) {
			return nil, false, nil
		}
		side := spiralsector.StationSide(ctx, trim, m, b, i)
		if side == 0 {
			return nil, false, nil
		}
		if _, arc := segment.(sectionrecord.ArcSeg); arc {
			finiteSide, certified := revolveloftfaces.MappedArcFiniteSide(ctx, trim, i,
				section.Spans[section.RootSpan])
			if !certified || finiteSide != loftface.ArcMiss && finiteSide != side {
				return nil, false, nil
			}
		}
		start[i], end[i], proof.Sides[i] = side, side, side
	}
	if proof.Crossings[0].Span == nil || proof.Crossings[1].Span == nil {
		return nil, false, nil
	}
	for i := range segments {
		if end[i] == 0 || end[i] != start[(i+1)%len(segments)] {
			return nil, false, nil
		}
	}
	for _, crossing := range proof.Crossings {
		if crossing.X[0].Hi.Cmp(toeX) >= 0 ||
			crossing.X[len(crossing.X)-1].Lo.Cmp(heelX) <= 0 {
			return nil, false, nil
		}
	}
	refs, ok := SourceRefs(trim, tooth)
	if !ok {
		return nil, false, nil
	}
	proof.SourceRefs = refs
	p, _ := tooth.Payload.(facetedrecord.Payload)
	faces := topobuild.Faces(tooth)
	for i := range proof.Crossings {
		crossing := &proof.Crossings[i]
		crossing.Faces = make([]*topo.Face, len(refs))
		retained := false
		for band := range refs {
			ref := refs[band][crossing.Segment]
			index, valid := revolveloftfaces.RefFaceIndex(p, ref, len(faces))
			face, originValid := UniqueOriginFace(tooth, ref)
			if !valid || !originValid || (index >= 0) != (face != nil) ||
				index >= 0 && faces[index] != face {
				return nil, false, nil
			}
			crossing.Faces[band] = face
			retained = retained || index >= 0
		}
		if !retained {
			return nil, false, nil
		}
	}
	proof.ToeFace = UniqueResultSourceFace(tooth, p.LoftFacesTrim.ToeCapGroup)
	proof.HeelFace = UniqueResultSourceFace(tooth, p.LoftFacesTrim.HeelCapGroup)
	if proof.ToeFace == nil || proof.HeelFace == nil || proof.ToeFace == proof.HeelFace {
		return nil, false, nil
	}
	return proof, true, nil
}

func SourceRefs(trim *facetedrecord.LoftFacesTrimRecord, tooth *topo.Body) ([][]provenance.Ref, bool) {
	n := len(trim.Source.Profile.Outer.Segments)
	if len(trim.Base.Groups) != (len(trim.Maps)-1)*n+2 {
		return nil, false
	}
	refs := make([][]provenance.Ref, len(trim.Maps)-1)
	var producer provenance.Producer
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
	for _, face := range topobuild.Faces(tooth) {
		for _, origin := range face.Origins {
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

func UniqueOriginFace(body *topo.Body, want provenance.Ref) (*topo.Face, bool) {
	var result *topo.Face
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			if origin != want {
				continue
			}
			if face.Body != body || result != nil {
				return nil, false
			}
			result = face
		}
	}
	return result, true
}

// UniqueResultSourceFace returns the one live face of body that source group
// names, or nil when the group is not a single face.
func UniqueResultSourceFace(body *topo.Body, source int) *topo.Face {
	p, ok := body.Payload.(facetedrecord.Payload)
	if !ok {
		return nil
	}
	faces := topobuild.Faces(body)
	index, ok := facetedtopology.UniqueGroupFace(p.Src, p.FaceOf, len(faces), source)
	if !ok {
		return nil
	}
	return faces[index]
}

func orderedConeLimits(toe, heel *facetedrecord.PointConeLimit, base facetedrecord.Payload) bool {
	return pointcone.ToeHeelOrdered(toe.Apex, toe.Slope, heel.Apex, heel.Slope,
		base.Verts, base.MeshBound)
}
