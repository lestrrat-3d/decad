package revolveshell

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

const (
	sectorJoinMaxAuditTriangles = 1_200_000
	sectorJoinStreamTriangles   = 1_000_000
)

type sectorVolumeRefineError struct{ gain, bound float64 }

type sectorCapRefineError struct{ limit float64 }

// sectorAuditLimitError reports that the joined mesh exceeds an audit ceiling,
// its triangle count or its crossing-scan work, so a coarser cap subdivision
// may still fit. cause is the refusal to report when none does.
type sectorAuditLimitError struct{ cause error }

func (e sectorAuditLimitError) Error() string { return e.cause.Error() }

func (e sectorAuditLimitError) Unwrap() error { return e.cause }

// sectorCapRounds is the finest cap subdivision a sector starts from.
const sectorCapRounds = 4

func (r *sectorCapRefineError) Error() string {
	return fmt.Sprintf("%v: joined sectors need a finer cone cap", decaderr.ErrUnsupported)
}

func (r *sectorVolumeRefineError) Error() string {
	return fmt.Sprintf("%v: joined sectors have no certified occupied volume", decaderr.ErrUnsupported)
}

type sectorContactRefineError struct{ cause error }

func (r *sectorContactRefineError) Error() string { return r.cause.Error() }
func (r *sectorContactRefineError) Unwrap() error { return r.cause }

// TraceSectorLoop sets cert's exterior loop, its root-contact sector angles
// and its crossing bound from the certified source boundary.
func TraceSectorLoop(cert *facetedrecord.RevolveLoftSectorCertificate) error {
	path, bound, err := revolveproof.RevolveExteriorPath(cert.Base, cert.Boundary,
		cert.Trim.Base.PointSection.Source, cert.Section)
	if err != nil {
		return err
	}
	lo, hi, ok := revolveproof.RevolveRootAngles(path)
	if !ok {
		return decaderr.ErrUnsupported
	}
	cert.Sector, cert.CrossingBound = [2]float64{lo, hi}, bound
	cert.Loop = path
	return nil
}

// BuildSectorsJoin reconstructs all admitted sectors against one
// authenticated Revolve source and builds the joined body in env. The joined
// mesh is never an input geometry. sources[0] is the unplaced trimmed source;
// the rest are its placed copies. The result's payload retains cert, its
// Revolve, the sources and the shared section ring, so a later Union rebuilds
// from them and a later Cut reads the ring.
func BuildSectorsJoin(ctx context.Context, env topo.Env,
	cert *facetedrecord.RevolveLoftSectorCertificate, sources []facetedrecord.Payload) (*topo.Body, error) {
	if cert == nil || cert.Trim == nil || cert.Blank == nil ||
		len(sources) == 0 || len(sources) > 128 ||
		!facetedrecord.PointSourceCurrent(cert.Trim.Base.PointSection) {
		return nil, fmt.Errorf("%w: sector source is incomplete or stale", decaderr.ErrUnsupported)
	}
	body, ring, err := buildSectorsJoinRefined(ctx, env, cert, sources)
	if err != nil {
		return nil, err
	}
	payload, ok := body.Payload.(facetedrecord.Payload)
	if !ok {
		return nil, decaderr.ErrUnsupported
	}
	payload.RevolveLoftJoin = &facetedrecord.RevolveLoftJoinRecord{Certificate: cert,
		Blank: cert.Blank, First: sources[0],
		Placed: append([]facetedrecord.Payload(nil), sources[1:]...), CapRing: ring,
		BoundaryBound: payload.MeshBound, VolumeBound: payload.VolSymDiff,
		AreaBound: payload.AreaSlack}
	body.Payload = payload
	return body, nil
}

// buildSectorsJoinRefined makes at most eight construction attempts. A cap
// refusal lowers the cone cap limit, a cap/root contact quarters the angular
// sag allowance, and an occupied-volume refusal scales it by a factor from
// 0.1 to 0.5 set by the measured margin before the next attempt.
func buildSectorsJoinRefined(ctx context.Context, env topo.Env,
	cert *facetedrecord.RevolveLoftSectorCertificate,
	sources []facetedrecord.Payload) (*topo.Body, *facetedrecord.RevolvedSectionRing, error) {
	sag := 0.01
	capLimit := math.Inf(1)
	rounds := sectorCapRounds
	attempt := -1
	for {
		attempt++
		body, ring, err := buildSectorsJoinAtSag(ctx, env, cert, sources, sag, capLimit, rounds)
		if err == nil {
			return body, ring, nil
		}
		// Every sector carries the same cap, so the joined mesh grows with the
		// sector count. A coarser start fits more sectors; the cap bound check
		// and the cap refinement below add cells back only where they are
		// needed. This step does not use up one of the eight attempts.
		if _, limit := err.(sectorAuditLimitError); limit && rounds > 0 {
			rounds--
			attempt--
			continue
		}
		if refine, ok := err.(*sectorCapRefineError); ok {
			if attempt == 7 || !(refine.limit > 0 && refine.limit < capLimit) {
				return nil, nil, err
			}
			capLimit = refine.limit
			continue
		}
		_, snap := err.(*sectorSnapError)
		if _, contact := err.(*sectorContactRefineError); contact || snap {
			if attempt == 7 {
				return nil, nil, err
			}
			sag *= 0.25
			continue
		}
		refine, ok := err.(*sectorVolumeRefineError)
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
}

func buildSectorsJoinAtSag(ctx context.Context, env topo.Env,
	cert *facetedrecord.RevolveLoftSectorCertificate, sources []facetedrecord.Payload,
	sagAllowance, capLimit float64, capRounds int) (*topo.Body, *facetedrecord.RevolvedSectionRing, error) {
	section, record, base := cert.Section, cert.Trim, cert.Base
	regularSource, sourceSector, err := RevolveSectionAngles(section, cert.Sector, sagAllowance)
	if err != nil {
		return nil, nil, err
	}
	path := make([]pointcone.SectorNode, len(cert.Loop))
	for i, node := range cert.Loop {
		path[i] = pointcone.SectorNode{P: node.Point, GroupToNext: node.GroupToNext,
			Crossing: node.Crossing, RootBoundary: node.RootBoundary, Angle: node.Angle}
	}
	original, err := RevolveSectorCapBoundary(path, section,
		base.PointSection.Source.Frame, regularSource, sourceSector)
	if err != nil {
		return nil, nil, err
	}
	caps, err := pointcone.SectorTriangulate(ctx, original, base.PointSection.Source.Frame)
	if err != nil {
		return nil, nil, triangulation.WrapLoftError(err)
	}
	original, caps, edges, farRound, err := pointcone.SectorSubdivide(ctx, original, caps, capRounds)
	if err != nil {
		return nil, nil, err
	}
	if !math.IsInf(capLimit, 1) {
		maxCells := sectorJoinMaxAuditTriangles / (2 * len(sources))
		for round := range 4 {
			upper, lower, capErr := SectorConeCapBounds(original, caps, record)
			if capErr != nil {
				return nil, nil, capErr
			}
			if max(upper, lower) <= capLimit {
				break
			}
			if round == 3 {
				return nil, nil, fmt.Errorf("%w: sector cap refinement is exhausted", decaderr.ErrUnsupported)
			}
			var extraRound float64
			original, caps, edges, extraRound, err = SectorRefineCap(ctx,
				original, caps, edges, record, capLimit, maxCells)
			if err != nil {
				return nil, nil, err
			}
			farRound = proofbound.AbsSumUpper(farRound, extraRound)
		}
	}
	motions := make([]r3.Transform, len(sources))
	charges := make([]massmoment.MapCharge, len(sources))
	drifts := make([]float64, len(sources))
	motions[0] = r3.Identity()
	if sources[0].PointCone != record || sources[0].Xform != r3.Identity() {
		return nil, nil, fmt.Errorf("%w: first sector lost its source", decaderr.ErrUnsupported)
	}
	for i := 1; i < len(sources); i++ {
		placed := sources[i]
		if placed.RevolveLoftSector == nil || placed.RevolveLoftSector.Source != record ||
			placed.Xform != placed.RevolveLoftSector.Motion {
			return nil, nil, fmt.Errorf("%w: placed sector lost its source", decaderr.ErrUnsupported)
		}
		motions[i] = placed.RevolveLoftSector.Motion
		charges[i], drifts[i], err = massmoment.AxisPlacementCharge(motions[i], section.Axis)
		if err != nil {
			return nil, nil, err
		}
	}
	seam, err := RevolveSectorSeam(section.AngleBasis(), cert.Loop, motions)
	if err != nil {
		return nil, nil, err
	}
	sourceMeshBounds := make([]float64, len(sources))
	for i := range sources {
		sourceMeshBounds[i] = sources[i].MeshBound
	}
	planInput := SectorJoinInput{
		Original: original, RootContactIndex: len(path) - 1, Edges: edges,
		Motions: motions, Charges: charges, Drifts: drifts, SourceCrossingBound: cert.CrossingBound,
		SourceMeshBounds: sourceMeshBounds,
		Seam:             seam, MinRadius: min(section.RootStart.V, section.RootEnd.V),
		RegularSource: regularSource,
	}
	if section.General != nil {
		if err := GeneralPlanInput(&planInput, section); err != nil {
			return nil, nil, err
		}
	}
	plans, angles, err := PrepareSectorJoinPlans(planInput)
	if err != nil {
		return nil, nil, err
	}
	sectors := make([][2]float64, len(plans))
	for i := range plans {
		sectors[i] = plans[i].Sector
	}
	mesh := &facetedrecord.JoinedMesh{}
	ring, sectionSag, err := BuildRevolvedSectionRing(mesh, section, angles, sectors, sagAllowance)
	if err != nil {
		return nil, nil, err
	}
	toolGap, ok := RevolveSectionToolGap(section, record)
	if !ok {
		return nil, nil, fmt.Errorf("%w: cone tools do not meet the recorded meridian", decaderr.ErrUnsupported)
	}
	far := make([]r3.Vec, len(original))
	for i, node := range original {
		far[i] = node.P
	}
	capGeom := pointcone.CapGeometry(far, caps, record.Upper.Axis)
	upperBound, upperLip, err := pointcone.CapBound(capGeom, record.Upper.BoundConstants())
	if err != nil {
		return nil, nil, err
	}
	lowerBound, lowerLip, err := pointcone.CapBound(capGeom, record.Lower.BoundConstants())
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
	boundInput := SectorJoinBoundInput{
		BaseMeshBound: base.MeshBound, BaseVolSymDiff: base.VolSymDiff,
		BaseAreaSlack: base.AreaSlack, BaseArea: baseArea,
		SectionSag: sectionSag, ConeGap: section.ConeGap, ToolGap: toolGap,
		FarRound: farRound, Lip: lip, UpperBound: upperBound, LowerBound: lowerBound,
		UpperLip: upperLip, LowerLip: lowerLip, CapArea: capArea, CapFactor: capFactor,
	}
	bounds := NewSectorJoinBounds()
	var samples SectorSamples
	for i := range plans {
		plan := &plans[i]
		var placedGroups []facetedrecord.Group
		if i != 0 {
			placedGroups = sources[i].Groups
		}
		baseGroup := len(mesh.Groups)
		plan.CapGroups = [2]int{baseGroup + len(record.Base.Groups) - 1,
			baseGroup + len(record.Base.Groups)}
		plan.NodeRound, plan.Snap, err = BuildRevolveSectorMesh(mesh, ring, section,
			plan.Nodes, original, caps, edges, plan.RootNode, record, plan.Motion, placedGroups, &samples)
		if err != nil {
			return nil, nil, err
		}
		if err := AccumulateSectorJoinBounds(&bounds, *plan, boundInput, i); err != nil {
			return nil, nil, err
		}
	}
	bound := proofbound.AbsSumUpper(sectionSag, bounds.LocalBound)
	if proofbound.IsNonFinite(bound) || bound > 0.1 {
		if bounds.CapRefinePossible && bounds.RefineCapLimit > 0 &&
			bounds.RefineCapLimit < capLimit && max(upperBound, lowerBound) > bounds.RefineCapLimit {
			return nil, nil, &sectorCapRefineError{limit: bounds.RefineCapLimit}
		}
		return nil, nil, fmt.Errorf("%w: joined sectors cannot meet the 0.1 mm boundary proof", decaderr.ErrUnsupported)
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
	if !sumOK || count == 0 || uint64(count) > maxTriangles || maxTriangles > sectorJoinMaxAuditTriangles {
		return nil, nil, sectorAuditLimitError{cause: fmt.Errorf(
			"%w: joined sectors exceed the audit mesh limit", decaderr.ErrUnsupported)}
	}
	pairCount, pairOK := proofbound.WallChoose2(uint64(count))
	workCeiling, workOK := proofbound.WallCheckedMul(25_000_000, uint64(len(sources)))
	if !pairOK || !workOK {
		return nil, nil, fmt.Errorf("%w: joined sectors exceed the audit work limit", decaderr.ErrUnsupported)
	}
	scans := min(pairCount, workCeiling)
	patches := make([]loftmesh.ProjectedPatch, 2*len(plans))
	patchByGroup := make(map[int]int, len(patches))
	axes := capPatchAxes(section.Axis)
	for i, plan := range plans {
		for side, group := range plan.CapGroups {
			patch := 2*i + side
			patches[patch].Axes = axes
			patchByGroup[group] = patch
		}
	}
	for i, group := range mesh.Src {
		if patch, ok := patchByGroup[group]; ok {
			patches[patch].Triangles = append(patches[patch].Triangles, i)
		}
	}
	audit := loftmesh.LoftCrossingAuditBoundedProjectedPatches
	if count > sectorJoinStreamTriangles {
		audit = loftmesh.LoftCrossingAuditBoundedProjectedPatchesStreaming
	}
	if err := audit(proofbound.NewWorkBudget(ctx), mesh.Verts, mesh.Tris, patches, count, scans); err != nil {
		if RootCapContact(mesh.Src, ring.Groups, section.RootSpan, plans, err) {
			return nil, nil, &sectorContactRefineError{cause: err}
		}
		if errors.Is(err, loftmesh.ErrLoftAuditCeiling) {
			return nil, nil, sectorAuditLimitError{cause: err}
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
	blankArea := cert.Blank.Area
	areaSlack := proofbound.AbsSumUpper(areaForArea,
		blankArea.Value.Base(), blankArea.Bound.Base(), bounds.SourceArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, nil, fmt.Errorf("%w: joined sectors have no finite measure proof", decaderr.ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.Verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	payload := facetedrecord.Payload{Verts: mesh.Verts, Tris: mesh.Tris, Src: mesh.Src,
		Groups: mesh.Groups, VertexBound: vertexBound, ContactAudited: true,
		MeshBound: bound, VolSymDiff: volumeGap, AreaSlack: areaSlack,
		DPair: base.DPair, Xform: r3.Identity()}
	body, err := facetedrecord.Build(ctx, env, payload)
	if err != nil {
		return nil, nil, err
	}
	blankVolume, err := solidVolume(cert.Blank)
	if err != nil {
		return nil, nil, err
	}
	joinedVolume, err := solidVolume(body)
	if err != nil {
		return nil, nil, err
	}
	gain := joinedVolume.Value.Base() - blankVolume.Value.Base() - blankVolume.Bound.Base()
	if gain <= joinedVolume.Bound.Base() {
		return nil, nil, &sectorVolumeRefineError{gain: gain, bound: joinedVolume.Bound.Base()}
	}
	return body, &ring, nil
}

// capPatchAxes returns the coordinate plane the sector cap patches project
// onto: the plane that drops the index of the axis's largest |A_k|, {1, 2}
// for +X (agent-docs/revolve-axis-design.md §5.8). A patch whose projection
// proof fails keeps its pairs in the exact contact audit, so the choice
// changes the audit's work and never its answer.
func capPatchAxes(axis coneaxis.Axis) [2]int {
	if axis.Canonical() {
		return [2]int{1, 2}
	}
	a := axis.Vec()
	components := [3]float64{math.Abs(a.X), math.Abs(a.Y), math.Abs(a.Z)}
	drop := 0
	for k := 1; k < 3; k++ {
		if components[k] > components[drop] {
			drop = k
		}
	}
	return [3][2]int{{1, 2}, {0, 2}, {0, 1}}[drop]
}

// solidVolume reads n's enclosed volume. A body that is not a solid encloses
// none, and the reading is decaderr.ErrNotSolid.
func solidVolume(n *topo.Body) (measurement.Measurement, error) {
	if !n.Solid {
		return measurement.Measurement{}, decaderr.ErrNotSolid
	}
	return n.Volume, nil
}
