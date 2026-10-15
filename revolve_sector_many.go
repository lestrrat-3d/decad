package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

type revolveSectorPlan struct {
	nodes                      []pointcone.SectorNode
	sector                     [2]float64
	rootNode                   []int
	motion                     r3.Transform
	charge                     massmoment.MapCharge
	placedRound, crossingBound float64
	nodeRound, snap            float64
	capGroups                  [2]int
}

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
		!pointSourceCurrent(cert.Trim.Base.PointSection) {
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
	regularSource, sourceSector, err := revolveSectionAngles(section, cert.Sector, sagAllowance)
	if err != nil {
		return nil, nil, err
	}
	path := make([]pointcone.SectorNode, len(cert.Loop))
	for i, node := range cert.Loop {
		path[i] = pointcone.SectorNode{P: node.Point, GroupToNext: node.GroupToNext,
			Crossing: node.Crossing, RootBoundary: node.RootBoundary, Angle: node.Angle}
	}
	original, err := revolveSectorCapBoundary(path, section,
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
			upper, lower, capErr := sectorConeCapBounds(original, caps, record)
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
			original, caps, edges, extraRound, err = sectorRefineCap(ctx,
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
	seam, err := revolveSectorSeam(cert.Loop, motions)
	if err != nil {
		return nil, nil, err
	}
	plans := make([]revolveSectorPlan, len(sources))
	for i := range plans {
		plan := &plans[i]
		plan.motion, plan.charge = motions[i], charges[i]
		plan.nodes = append([]pointcone.SectorNode(nil), original...)
		for j, node := range original {
			point := motions[i].Apply(node.P)
			if !proofbound.FiniteVec(point) {
				return nil, nil, fmt.Errorf("%w: placed cap has a non-finite station", ErrUnsupported)
			}
			plan.nodes[j].P = point
			if node.RootBoundary {
				plan.nodes[j].Angle = facetproof.AngleAt(point, seam)
			}
			if i != 0 {
				maxAbs := max(math.Abs(node.P.X), math.Abs(node.P.Y), math.Abs(node.P.Z))
				plan.placedRound = max(plan.placedRound, proofbound.AbsSumUpper(
					proofbound.RigidRoundAllow(maxAbs, 0),
					proofbound.ProductUpper(charges[i].Stretch, proofbound.Radius3D(maxAbs))))
			}
		}
		a, b := plan.nodes[0].Angle, plan.nodes[len(path)-1].Angle
		if a == b || math.Abs(a-b) >= math.Pi {
			return nil, nil, fmt.Errorf("%w: placed root contacts cross the angular seam", ErrUnsupported)
		}
		plan.sector = [2]float64{min(a, b), max(a, b)}
		plan.crossingBound = proofbound.AbsSumUpper(
			massmoment.PlacedBound(cert.CrossingBound, charges[i]), plan.placedRound)
		if !pointcone.SectorRootEdgeOrdered(plan.nodes, edges, plan.sector) {
			return nil, nil, fmt.Errorf("%w: placed root stations lose angular order", ErrUnsupported)
		}
	}
	minRadius := min(section.RootStart.V, section.RootEnd.V)
	for i := range plans {
		for j := i + 1; j < len(plans); j++ {
			seamCost := proofbound.AbsSumUpper(plans[i].crossingBound,
				plans[j].crossingBound,
				massmoment.PlacedBound(sources[i].MeshBound, charges[i]),
				massmoment.PlacedBound(sources[j].MeshBound, charges[j]))
			if !(minRadius > 0 && seamCost < minRadius/2) {
				return nil, nil, fmt.Errorf("%w: root sectors have no angular error bound", ErrUnsupported)
			}
			angleCost := proofbound.DivUpper(proofbound.ProductUpper(2, seamCost), minRadius)
			if !(facetproof.SectorAngularGap(plans[i].sector, plans[j].sector) > angleCost) {
				return nil, nil, fmt.Errorf("%w: root sectors are not separate", ErrUnsupported)
			}
		}
	}
	stations := len(regularSource) - 2
	if stations < 16 || stations > 8192 {
		return nil, nil, fmt.Errorf("%w: sector has no bounded angular plan", ErrUnsupported)
	}
	regular := make([]float64, 0, stations+2*len(plans))
	for i := range stations {
		regular = append(regular, seam+2*math.Pi*float64(i)/float64(stations))
	}
	for _, plan := range plans {
		regular = append(regular, plan.sector[:]...)
	}
	outside := regular[:0]
	for _, angle := range regular {
		covered := false
		for _, plan := range plans {
			covered = covered || plan.sector[0] < angle && angle < plan.sector[1]
		}
		if !covered {
			outside = append(outside, angle)
		}
	}
	angles := outside
	for i := range plans {
		angles, _, err = pointcone.SectorMergedAngles(angles, plans[i].nodes)
		if err != nil {
			return nil, nil, err
		}
	}
	for i := range plans {
		_, plans[i].rootNode, err = pointcone.SectorMergedAngles(angles, plans[i].nodes)
		if err != nil {
			return nil, nil, err
		}
	}
	sectors := make([][2]float64, len(plans))
	for i := range plans {
		sectors[i] = plans[i].sector
	}
	mesh := &joinedMesh{}
	ring, sectionSag, err := buildRevolvedSectionRing(mesh, section, angles, sectors, sagAllowance)
	if err != nil {
		return nil, nil, err
	}
	toolGap, ok := revolveSectionToolGap(section, record)
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
	localBound, localRound, sourceVolume, sourceArea := 0.0, 0.0, 0.0, 0.0
	refineCapLimit, capRefinePossible := math.Inf(1), true
	var samples revolveSectorSamples
	for i := range plans {
		plan := &plans[i]
		var placedGroups []facetGroup
		if i != 0 {
			placedGroups = sources[i].Groups
		}
		baseGroup := len(mesh.Groups)
		plan.capGroups = [2]int{baseGroup + len(record.Base.Groups) - 1,
			baseGroup + len(record.Base.Groups)}
		plan.nodeRound, plan.snap, err = buildRevolveSectorMesh(mesh, ring, section,
			plan.nodes, original, caps, edges, plan.rootNode, record, plan.motion, placedGroups, &samples)
		if err != nil {
			return nil, nil, err
		}
		seamAllowance := proofbound.AbsSumUpper(base.MeshBound, sectionSag,
			section.ConeGap, plan.crossingBound, toolGap, plan.placedRound)
		if plan.snap > seamAllowance {
			return nil, nil, fmt.Errorf("%w: placed cap misses the finite root patch (sector %d snap %.9g allowance %.9g)",
				ErrUnsupported, i, plan.snap, seamAllowance)
		}
		stretch := proofbound.AbsSumUpper(1, plan.charge.Stretch)
		farTerm := proofbound.ProductUpper(proofbound.ProductUpper(farRound, stretch), lip)
		baseTerm := proofbound.ProductUpper(proofbound.AbsSumUpper(
			massmoment.PlacedBound(base.MeshBound, plan.charge),
			plan.crossingBound, plan.placedRound), lip)
		capTerm := proofbound.ProductUpper(max(upperBound, lowerBound), stretch)
		otherBound := proofbound.AbsSumUpper(plan.snap, plan.nodeRound,
			farTerm, baseTerm, section.ConeGap, toolGap)
		remaining := 0.1 - sectionSag - otherBound
		if remaining <= 0 || proofbound.IsNonFinite(remaining) {
			capRefinePossible = false
		} else {
			refineCapLimit = min(refineCapLimit, 0.9*remaining/stretch)
		}
		localBound = max(localBound, proofbound.AbsSumUpper(plan.snap, plan.nodeRound,
			farTerm, baseTerm, capTerm, section.ConeGap, toolGap))
		localRound = max(localRound, proofbound.AbsSumUpper(plan.snap, plan.nodeRound,
			proofbound.ProductUpper(proofbound.ProductUpper(farRound, stretch), lip),
			proofbound.ProductUpper(proofbound.AbsSumUpper(
				plan.crossingBound, plan.placedRound), lip), section.ConeGap, toolGap))
		volumePart := proofbound.AbsSumUpper(base.VolSymDiff,
			proofbound.ProductUpper(proofbound.ProductUpper(capArea,
				proofbound.AbsSumUpper(upperBound, lowerBound)), capFactor))
		areaPart := proofbound.AbsSumUpper(baseArea, base.AreaSlack,
			proofbound.ProductUpper(proofbound.AbsSumUpper(1,
				proofbound.ProductUpper(upperLip, upperLip),
				proofbound.ProductUpper(lowerLip, lowerLip)),
				proofbound.AbsSumUpper(capArea, base.AreaSlack)))
		if i != 0 {
			volumePart = proofbound.ProductUpper(
				proofbound.AbsSumUpper(1, plan.charge.Volume), volumePart)
			areaPart = proofbound.ProductUpper(stretch, areaPart)
		}
		sourceVolume = proofbound.AbsSumUpper(sourceVolume, volumePart)
		sourceArea = proofbound.AbsSumUpper(sourceArea, areaPart)
	}
	bound := proofbound.AbsSumUpper(sectionSag, localBound)
	if proofbound.IsNonFinite(bound) || bound > 0.1 {
		if capRefinePossible && refineCapLimit > 0 &&
			refineCapLimit < capLimit && max(upperBound, lowerBound) > refineCapLimit {
			return nil, nil, &revolveSectorCapRefineError{limit: refineCapLimit}
		}
		return nil, nil, fmt.Errorf("%w: joined sectors cannot meet the 0.1 mm boundary proof", ErrUnsupported)
	}
	if err := orientJoinedMesh(mesh); err != nil {
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
		for side, group := range plan.capGroups {
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
		if revolveRootCapContact(mesh, ring, section, plans, err) {
			return nil, nil, &revolveSectorContactRefineError{cause: err}
		}
		return nil, nil, err
	}
	roundEnvelope := proofbound.AbsSumUpper(sectionSag, localRound)
	area, areaForArea, err := proofbound.PerturbedAreaUpperPairContext(ctx, mesh.Verts,
		mesh.Tris, roundEnvelope, bound)
	if err != nil {
		return nil, nil, err
	}
	volumeGap := proofbound.AbsSumUpper(sourceVolume,
		proofbound.SweptVolumeAllow(roundEnvelope, area))
	blankArea, err := bodyHandle(cert.Blank).Area()
	if err != nil {
		return nil, nil, err
	}
	areaSlack := proofbound.AbsSumUpper(areaForArea,
		blankArea.Value.Base(), blankArea.Bound.Base(), sourceArea)
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
	body, err := buildFacetedBody(ctx, d, ref, payload)
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

// A retry is permitted only for a mapped cap contacting the Revolve face
// adjacent to its authenticated root span. Every candidate faces the full
// contact, topology, and measure audit again.
func revolveRootCapContact(mesh *joinedMesh, ring revolvedSectionRing,
	section revolveSectionCertificate, plans []revolveSectorPlan, err error) bool {
	var contact *loftmesh.LoftContactError
	if !errors.As(err, &contact) ||
		contact.Reason != "share no recorded vertex, but make contact" ||
		contact.I < 0 || contact.J < 0 ||
		contact.I >= len(mesh.Src) || contact.J >= len(mesh.Src) ||
		len(plans) == 0 || section.RootSpan < 0 ||
		section.RootSpan >= len(ring.Groups) {
		return false
	}
	prev := (section.RootSpan + len(ring.Groups) - 1) % len(ring.Groups)
	next := (section.RootSpan + 1) % len(ring.Groups)
	isNeighbor := func(group int) bool {
		return group >= 0 && (group == ring.Groups[prev] || group == ring.Groups[next])
	}
	isCap := func(group int) bool {
		for _, plan := range plans {
			if group == plan.capGroups[0] || group == plan.capGroups[1] {
				return true
			}
		}
		return false
	}
	a, b := mesh.Src[contact.I], mesh.Src[contact.J]
	return isCap(a) && isNeighbor(b) || isCap(b) && isNeighbor(a)
}
