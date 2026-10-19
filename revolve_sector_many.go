package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

const (
	revolveSectorMaxAuditTriangles = 1_200_000
	revolveSectorStreamTriangles   = 1_000_000
)

type revolveSectorVolumeRefineError struct{ gain, bound float64 }

type revolveSectorCapRefineError struct{ limit float64 }

func (r *revolveSectorCapRefineError) Error() string {
	return fmt.Sprintf("%v: joined sectors need a finer cone cap", ErrUnsupported)
}

func (r *revolveSectorVolumeRefineError) Error() string {
	return fmt.Sprintf("%v: joined sectors have no certified occupied volume", ErrUnsupported)
}

type revolveSectorContactRefineError struct{ cause error }

func (r *revolveSectorContactRefineError) Error() string { return r.cause.Error() }
func (r *revolveSectorContactRefineError) Unwrap() error { return r.cause }

// buildRevolveLoftSectorsJoin reconstructs all admitted sectors against one
// authenticated Revolve source. The joined mesh is never an input geometry.
func buildRevolveLoftSectorsJoin(ctx context.Context, d *Document, ref producerID,
	cert *revolveLoftSectorCertificate, sources []facetedPayload) (*Body, *revolvedSectionRing, error) {
	if cert == nil || cert.Trim == nil || cert.Blank == nil ||
		len(sources) == 0 || len(sources) > 128 ||
		!facetedrecord.PointSourceCurrent(cert.Trim.Base.PointSection) {
		return nil, nil, fmt.Errorf("%w: sector source is incomplete or stale", ErrUnsupported)
	}
	sag := 0.01
	capLimit := math.Inf(1)
	for attempt := range 8 {
		body, ring, err := buildRevolveLoftSectorsAtSag(ctx, d, ref, cert, sources, sag, capLimit)
		if err == nil {
			return body, ring, nil
		}
		if refine, ok := err.(*revolveSectorCapRefineError); ok {
			if attempt == 7 || !(refine.limit > 0 && refine.limit < capLimit) {
				return nil, nil, err
			}
			capLimit = refine.limit
			continue
		}
		if _, contact := err.(*revolveSectorContactRefineError); contact {
			if attempt == 7 {
				return nil, nil, err
			}
			sag *= 0.25
			continue
		}
		refine, ok := err.(*revolveSectorVolumeRefineError)
		if !ok || refine.gain <= 0 || refine.bound <= 0 || attempt == 7 {
			return nil, nil, err
		}
		factor := max(0.1, min(0.5, 0.5*refine.gain/refine.bound))
		next := sag * factor
		if next <= 0 || next >= sag || math.IsNaN(next) {
			return nil, nil, err
		}
		sag = next
	}
	return nil, nil, fmt.Errorf("%w: sector angular refinement ended without a volume proof", ErrUnsupported)
}

func buildRevolveLoftSectorsAtSag(ctx context.Context, d *Document, ref producerID,
	cert *revolveLoftSectorCertificate, sources []facetedPayload,
	sagAllowance, capLimit float64) (*Body, *revolvedSectionRing, error) {
	section, record, base := cert.Section, cert.Trim, cert.Base
	regularSource, sourceSector, err := revolveshell.RevolveSectionAngles(section, cert.Sector, sagAllowance)
	if err != nil {
		return nil, nil, err
	}
	path := make([]pointcone.SectorNode, len(cert.Loop))
	for i, node := range cert.Loop {
		path[i] = pointcone.SectorNode{P: node.Point, GroupToNext: node.GroupToNext,
			Crossing: node.Crossing, RootBoundary: node.RootBoundary, Angle: node.Angle}
	}
	original, err := revolveshell.RevolveSectorCapBoundary(path, section,
		base.PointSection.Source.Frame, regularSource, sourceSector)
	if err != nil {
		return nil, nil, err
	}
	caps, err := pointcone.SectorTriangulate(ctx, original, base.PointSection.Source.Frame)
	if err != nil {
		return nil, nil, triangulation.WrapLoftError(err)
	}
	original, caps, edges, farRound, err := pointcone.SectorSubdivide(ctx, original, caps, 4)
	if err != nil {
		return nil, nil, err
	}
	if !math.IsInf(capLimit, 1) {
		maxCells := revolveSectorMaxAuditTriangles / (2 * len(sources))
		for round := range 4 {
			upper, lower, capErr := revolveshell.SectorConeCapBounds(original, caps, record)
			if capErr != nil {
				return nil, nil, capErr
			}
			if max(upper, lower) <= capLimit {
				break
			}
			if round == 3 {
				return nil, nil, fmt.Errorf("%w: sector cap refinement is exhausted", ErrUnsupported)
			}
			var extraRound float64
			original, caps, edges, extraRound, err = revolveshell.SectorRefineCap(ctx,
				original, caps, edges, record, capLimit, maxCells)
			if err != nil {
				return nil, nil, err
			}
			farRound = proofbound.AbsSumUpper(farRound, extraRound)
		}
	}
	motions := make([]r3.Transform, len(sources))
	charges := make([]massmoment.MapCharge, len(sources))
	motions[0] = r3.Identity()
	if sources[0].PointCone != record || sources[0].Xform != r3.Identity() {
		return nil, nil, fmt.Errorf("%w: first sector lost its source", ErrUnsupported)
	}
	for i := 1; i < len(sources); i++ {
		placed := sources[i]
		if placed.RevolveLoftSector == nil || placed.RevolveLoftSector.Source != record ||
			placed.Xform != placed.RevolveLoftSector.Motion {
			return nil, nil, fmt.Errorf("%w: placed sector lost its source", ErrUnsupported)
		}
		motions[i] = placed.RevolveLoftSector.Motion
		charges[i], err = massmoment.XAxisPlacementCharge(motions[i])
		if err != nil {
			return nil, nil, err
		}
	}
	seam, err := revolveshell.RevolveSectorSeam(cert.Loop, motions)
	if err != nil {
		return nil, nil, err
	}
	sourceMeshBounds := make([]float64, len(sources))
	for i := range sources {
		sourceMeshBounds[i] = sources[i].MeshBound
	}
	plans, angles, err := revolveshell.PrepareSectorJoinPlans(revolveshell.SectorJoinInput{
		Original: original, RootContactIndex: len(path) - 1, Edges: edges,
		Motions: motions, Charges: charges, SourceCrossingBound: cert.CrossingBound,
		SourceMeshBounds: sourceMeshBounds,
		Seam:             seam, MinRadius: min(section.RootStart.V, section.RootEnd.V),
		RegularSource: regularSource,
	})
	if err != nil {
		return nil, nil, err
	}
	sectors := make([][2]float64, len(plans))
	for i := range plans {
		sectors[i] = plans[i].Sector
	}
	mesh := &joinedMesh{}
	ring, sectionSag, err := revolveshell.BuildRevolvedSectionRing(mesh, section, angles, sectors, sagAllowance)
	if err != nil {
		return nil, nil, err
	}
	toolGap, ok := revolveshell.RevolveSectionToolGap(section, record)
	if !ok {
		return nil, nil, fmt.Errorf("%w: cone tools do not meet the recorded meridian", ErrUnsupported)
	}
	far := make([]r3.Vec, len(original))
	for i, node := range original {
		far[i] = node.P
	}
	capGeom := pointcone.CapGeometry(far, caps)
	upperBound, upperLip, err := pointcone.CapBound(capGeom, record.Upper.Constants())
	if err != nil {
		return nil, nil, err
	}
	lowerBound, lowerLip, err := pointcone.CapBound(capGeom, record.Lower.Constants())
	if err != nil {
		return nil, nil, err
	}
	lip := proofbound.AbsSumUpper(1, upperLip, lowerLip)
	capArea := pointcone.CapArea(far, caps)
	capFactor := max(1, proofbound.ProductUpper(upperLip, upperLip),
		proofbound.ProductUpper(lowerLip, lowerLip))
	baseArea, err := proofbound.PerturbedAreaUpperContext(ctx, base.Verts,
		base.Tris, base.MeshBound)
	if err != nil {
		return nil, nil, err
	}
	boundInput := revolveshell.SectorJoinBoundInput{
		BaseMeshBound: base.MeshBound, BaseVolSymDiff: base.VolSymDiff,
		BaseAreaSlack: base.AreaSlack, BaseArea: baseArea,
		SectionSag: sectionSag, ConeGap: section.ConeGap, ToolGap: toolGap,
		FarRound: farRound, Lip: lip, UpperBound: upperBound, LowerBound: lowerBound,
		UpperLip: upperLip, LowerLip: lowerLip, CapArea: capArea, CapFactor: capFactor,
	}
	bounds := revolveshell.NewSectorJoinBounds()
	var samples revolveshell.SectorSamples
	for i := range plans {
		plan := &plans[i]
		var placedGroups []facetGroup
		if i != 0 {
			placedGroups = sources[i].Groups
		}
		baseGroup := len(mesh.Groups)
		plan.CapGroups = [2]int{baseGroup + len(record.Base.Groups) - 1,
			baseGroup + len(record.Base.Groups)}
		plan.NodeRound, plan.Snap, err = revolveshell.BuildRevolveSectorMesh(mesh, ring, section,
			plan.Nodes, original, caps, edges, plan.RootNode, record, plan.Motion, placedGroups, &samples)
		if err != nil {
			return nil, nil, err
		}
		if err := revolveshell.AccumulateSectorJoinBounds(&bounds, *plan, boundInput, i); err != nil {
			return nil, nil, err
		}
	}
	bound := proofbound.AbsSumUpper(sectionSag, bounds.LocalBound)
	if proofbound.IsNonFinite(bound) || bound > 0.1 {
		if bounds.CapRefinePossible && bounds.RefineCapLimit > 0 &&
			bounds.RefineCapLimit < capLimit && max(upperBound, lowerBound) > bounds.RefineCapLimit {
			return nil, nil, &revolveSectorCapRefineError{limit: bounds.RefineCapLimit}
		}
		return nil, nil, fmt.Errorf("%w: joined sectors cannot meet the 0.1 mm boundary proof", ErrUnsupported)
	}
	if err := tessellation.OrientJoinedMesh(mesh.Verts, mesh.Tris); err != nil {
		return nil, nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.Verts), mesh.Tris); err != nil {
		return nil, nil, err
	}
	count := len(mesh.Tris)
	ringLimit := uint64(2) * uint64(len(section.Spans)-1) * uint64(len(angles))
	capTriangleLimit := uint64(len(sources)) * uint64(2*len(caps)+2*len(edges))
	maxTriangles, sumOK := proofbound.WallCheckedAdd(ringLimit, capTriangleLimit)
	if !sumOK || count == 0 || uint64(count) > maxTriangles || maxTriangles > revolveSectorMaxAuditTriangles {
		return nil, nil, fmt.Errorf("%w: joined sectors exceed the audit mesh limit", ErrUnsupported)
	}
	pairCount, pairOK := proofbound.WallChoose2(uint64(count))
	workCeiling, workOK := proofbound.WallCheckedMul(25_000_000, uint64(len(sources)))
	if !pairOK || !workOK {
		return nil, nil, fmt.Errorf("%w: joined sectors exceed the audit work limit", ErrUnsupported)
	}
	scans := min(pairCount, workCeiling)
	patches := make([]loftmesh.ProjectedPatch, 2*len(plans))
	patchByGroup := make(map[int]int, len(patches))
	for i, plan := range plans {
		for side, group := range plan.CapGroups {
			patch := 2*i + side
			patches[patch].Axes = [2]int{1, 2} // The admitted Revolve source has world-X axis.
			patchByGroup[group] = patch
		}
	}
	for i, group := range mesh.Src {
		if patch, ok := patchByGroup[group]; ok {
			patches[patch].Triangles = append(patches[patch].Triangles, i)
		}
	}
	audit := loftmesh.LoftCrossingAuditBoundedProjectedPatches
	if count > revolveSectorStreamTriangles {
		audit = loftmesh.LoftCrossingAuditBoundedProjectedPatchesStreaming
	}
	if err := audit(proofbound.NewWorkBudget(ctx), mesh.Verts, mesh.Tris, patches, count, scans); err != nil {
		if revolveshell.RootCapContact(mesh.Src, ring.Groups, section.RootSpan, plans, err) {
			return nil, nil, &revolveSectorContactRefineError{cause: err}
		}
		return nil, nil, err
	}
	roundEnvelope := proofbound.AbsSumUpper(sectionSag, bounds.LocalRound)
	area, areaForArea, err := proofbound.PerturbedAreaUpperPairContext(ctx, mesh.Verts,
		mesh.Tris, roundEnvelope, bound)
	if err != nil {
		return nil, nil, err
	}
	volumeGap := proofbound.AbsSumUpper(bounds.SourceVolume,
		proofbound.SweptVolumeAllow(roundEnvelope, area))
	blankArea, err := bodyHandle(cert.Blank).Area()
	if err != nil {
		return nil, nil, err
	}
	areaSlack := proofbound.AbsSumUpper(areaForArea,
		blankArea.Value.Base(), blankArea.Bound.Base(), bounds.SourceArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, nil, fmt.Errorf("%w: joined sectors have no finite measure proof", ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.Verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	payload := facetedPayload{Verts: mesh.Verts, Tris: mesh.Tris, Src: mesh.Src,
		Groups: mesh.Groups, VertexBound: vertexBound, ContactAudited: true,
		MeshBound: bound, VolSymDiff: volumeGap, AreaSlack: areaSlack,
		DPair: base.DPair, Xform: r3.Identity()}
	body, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), payload))
	if err != nil {
		return nil, nil, err
	}
	blankVolume, err := bodyHandle(cert.Blank).Volume()
	if err != nil {
		return nil, nil, err
	}
	joinedVolume, err := body.Volume()
	if err != nil {
		return nil, nil, err
	}
	gain := joinedVolume.Value.Base() - blankVolume.Value.Base() - blankVolume.Bound.Base()
	if gain <= joinedVolume.Bound.Base() {
		return nil, nil, &revolveSectorVolumeRefineError{gain: gain, bound: joinedVolume.Bound.Base()}
	}
	return body, &ring, nil
}
