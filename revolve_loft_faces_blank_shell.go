package decad

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
)

// A source-aligned blank shell is the internal shell mesh bound to its live
// sources. It stays in the root because its cap rings keep the root cap-arc
// records and side ties that later graph checks compare.
type revolveLoftFacesBlankSeamShell struct {
	revolveloftfaces.ShellMesh
	seam       []int
	seamIDs    []int
	seamRunIDs [][]int
	capRings   []revolveLoftFacesShellCapRing
}

type revolveLoftFacesShellSeamSelector struct {
	sectionIndex, cellIndex int
}

type revolveLoftFacesShellCapRing struct {
	revolveloftfaces.ShellCapRing
	arc              *revolveLoftFacesToolCapOccupiedArc
	endpointProof    [2]revolveLoftFacesCapSeamVertex
	ties             *revolveLoftFacesHeelArcSideTies
	endpointGraphIDs [2]int
	endpointCellIDs  [2]int
}

// A cap cut keeps the root occupied-arc record that certified it and the
// side ties derived for it.
type revolveLoftFacesShellCut struct {
	revolveloftfaces.ShellCut
	arc  *revolveLoftFacesToolCapOccupiedArc
	ties *revolveLoftFacesHeelArcSideTies
}

// revolveLoftFacesBlankShellAtRunsWithCuts builds every authenticated Revolve
// wall and end cap on one angular grid. Each selected whole-arc source contact
// splits its finite meridian side. Source cells select the ring; held output
// edges never supply a crossing location.
func revolveLoftFacesBlankShellAtRunsWithCuts(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	manifest *revolveLoftFacesContactManifest,
	selectors []revolveLoftFacesShellSeamSelector, capCuts []revolveLoftFacesShellCut,
) (*revolveLoftFacesBlankSeamShell, bool, error) {
	if ctx == nil || proof == nil || proof.trim == nil || proof.tooth == nil || manifest == nil ||
		len(selectors) == 0 ||
		!revolveLoftFacesSectionCurrent(proof) {
		return nil, false, nil
	}
	payload, payloadOK := proof.tooth.n.Payload.(facetedPayload)
	if !payloadOK || payload.LoftFacesTrim != proof.trim ||
		!revolveLoftFacesTrimCurrent(proof.tooth, proof.trim) {
		return nil, false, nil
	}
	census, currentOK, err := revolveLoftFacesCensusIn(ctx, proof)
	if err != nil || !currentOK {
		return nil, false, err
	}
	current := census.manifest
	if !revolveLoftFacesSameShellManifest(manifest, current) {
		return nil, false, nil
	}
	manifest = current
	seams := make([][]revolveloftfaces.ArcSeamVertex, len(selectors))
	seamCuts := make([]revolveloftfaces.ShellCut, len(selectors))
	seenSelectors := make(map[revolveLoftFacesShellSeamSelector]struct{}, len(selectors))
	for run, selector := range selectors {
		if _, duplicate := seenSelectors[selector]; duplicate {
			return nil, false, nil
		}
		seenSelectors[selector] = struct{}{}
		cut, vertices, valid, cutErr := revolveLoftFacesShellSeamCutOf(ctx, proof, manifest, selector)
		if cutErr != nil || !valid {
			return nil, false, cutErr
		}
		cut.SeamRun = run
		seamCuts[run], seams[run] = cut, vertices
	}
	n, chordBound, sized := revolveloftfaces.ShellGridSize(proof.section.Spans, proof.section.AxisSpan)
	if !sized {
		return nil, false, nil
	}
	capVertices := make([]revolveLoftFacesCapSeamVertex, 0, 2*len(capCuts))
	anchors := revolveloftfaces.ShellAnchors{Seams: seams,
		Caps: make([]revolveloftfaces.ShellRayCap, 0, 2*len(capCuts))}
	for i := range capCuts {
		vertices, built, err := revolveLoftFacesCapArcSeamVerticesIn(ctx, proof, census, capCuts[i].arc)
		if err != nil || !built {
			return nil, false, err
		}
		ties, tied, err := revolveLoftFacesHeelArcSideTiesIn(ctx, proof, census, capCuts[i].arc)
		if err != nil || !tied {
			return nil, false, err
		}
		capCuts[i].CapIDs = [2]int{len(capVertices), len(capVertices) + 1}
		capCuts[i].ThroughPositiveY = capCuts[i].arc.throughPositiveY
		capCuts[i].ties = ties
		capVertices = append(capVertices, vertices[:]...)
		for _, vertex := range vertices {
			anchors.Caps = append(anchors.Caps, revolveloftfaces.ShellRayCap{
				Point: vertex.point, BlankBound: vertex.blankBound,
				Box: revolveloftfaces.RayBox{Y: vertex.boundary.box[1], Z: vertex.boundary.box[2]}})
		}
	}
	rays, positiveYAxis, gridOK := revolveloftfaces.ShellGrid(n, anchors, len(proof.section.Spans))
	if !gridOK {
		return nil, false, nil
	}
	groups := make([]int, len(proof.section.Spans))
	var meshGroups []facetGroup
	for i, span := range proof.section.Spans {
		groups[i] = -1
		if span.Face == nil {
			if i != proof.section.AxisSpan {
				return nil, false, nil
			}
			continue
		}
		if i >= len(manifest.sideOutcome) {
			return nil, false, nil
		}
		current, err := revolveLoftFacesShellSideCurrent(ctx, proof, manifest, i, span)
		if err != nil || !current {
			return nil, false, err
		}
		groups[i] = len(meshGroups)
		meshGroups = append(meshGroups,
			facetGroup{Origins: append([]provenance.Ref(nil), span.Origins...), Planar: faceHandle(span.Face).isPlanar()})
	}
	meshCaps := make([]revolveloftfaces.ShellCut, len(capCuts))
	for i, cut := range capCuts {
		if cut.arc == nil || cut.Section < 0 ||
			cut.Section >= len(proof.section.Spans) ||
			cut.X == nil || cut.Radius == nil || cut.Seam ||
			cut.arc.circle.x == nil || cut.arc.circle.radius == nil ||
			cut.arc.circle.span != cut.Section ||
			cut.arc.circle.x.Cmp(cut.X) != 0 ||
			cut.arc.circle.radius.Cmp(cut.Radius) != 0 {
			return nil, false, nil
		}
		current, err := revolveLoftFacesToolCapOccupiedArcCurrentIn(ctx, proof, census, cut.arc)
		if err != nil || !current {
			return nil, false, err
		}
		meshCaps[i] = cut.ShellCut
	}
	mesh, rings, built, err := revolveloftfaces.BuildShellMesh(ctx, revolveloftfaces.ShellMeshInput{
		Spans: proof.section.Spans, Groups: groups, MeshGroups: meshGroups,
		Anchors: anchors, Rays: rays, PositiveYAxis: positiveYAxis, ChordBound: chordBound,
		SeamCuts: seamCuts, CapCuts: meshCaps})
	if err != nil || !built {
		return nil, false, err
	}
	out := &revolveLoftFacesBlankSeamShell{ShellMesh: mesh,
		capRings: make([]revolveLoftFacesShellCapRing, len(rings))}
	out.seam = out.Seams[0]
	for i, ring := range rings {
		cut := capCuts[ring.Cut]
		out.capRings[i] = revolveLoftFacesShellCapRing{ShellCapRing: ring, arc: cut.arc, ties: cut.ties,
			endpointProof: [2]revolveLoftFacesCapSeamVertex{
				capVertices[cut.CapIDs[0]], capVertices[cut.CapIDs[1]]}}
	}
	return out, true, nil
}

func revolveLoftFacesSameArcContact(a, b revolveloftfaces.RevolveLoftFacesMappedCell) bool {
	return a.Segment == b.Segment && a.Span == b.Span && a.Band == b.Band &&
		a.Side == revolveloftfaces.RevolveLoftFacesJointContact && b.Side == a.Side &&
		a.JointKind == revolveloftfaces.RevolveLoftFacesWholeArcContact && b.JointKind == a.JointKind &&
		a.JointArc == b.JointArc && a.JointPoint == b.JointPoint &&
		a.SLo != nil && b.SLo != nil && a.SLo.Cmp(b.SLo) == 0 &&
		a.SHi != nil && b.SHi != nil && a.SHi.Cmp(b.SHi) == 0 &&
		a.TLo != nil && b.TLo != nil && a.TLo.Cmp(b.TLo) == 0 &&
		a.THi != nil && b.THi != nil && a.THi.Cmp(b.THi) == 0 &&
		a.TLoOpen == b.TLoOpen && a.THiOpen == b.THiOpen
}

// A candidate must carry its retained source cells; a marked outcome alone
// cannot turn an empty nonvertical face into certified contact.
func revolveLoftFacesShellSideCurrent(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	manifest *revolveLoftFacesContactManifest,
	index int, span revolveSectionSpan) (bool, error) {
	switch manifest.sideOutcome[index] {
	case revolveLoftFacesSideSourceCandidate:
		if span.Start.U != span.End.U {
			for _, side := range manifest.nonvertical {
				if side.sectionIndex == index {
					return faceNode(side.face) == span.Face && len(side.cells) > 0 &&
						slices.Equal(side.origins, span.Origins), nil
				}
			}
			return false, nil
		}
		side, outcome, err := revolveLoftFacesVerticalSideClassify(ctx, proof, index)
		if err != nil || outcome != revolveLoftFacesVerticalContact ||
			side == nil || faceNode(side.face) != span.Face || len(side.cells) == 0 {
			return false, err
		}
		for _, entry := range manifest.vertical {
			if entry.sectionIndex == index {
				return entry.outcome == revolveLoftFacesVerticalContact &&
					entry.side != nil && faceNode(entry.side.face) == span.Face &&
					len(entry.side.cells) > 0, nil
			}
		}
	case revolveLoftFacesSideMiss:
		if span.Start.U != span.End.U {
			return false, nil
		}
		side, outcome, err := revolveLoftFacesVerticalSideClassify(ctx, proof, index)
		if err != nil || outcome != revolveLoftFacesVerticalMiss || side != nil {
			return false, err
		}
		for _, entry := range manifest.vertical {
			if entry.sectionIndex == index {
				return entry.outcome == revolveLoftFacesVerticalMiss && entry.side == nil, nil
			}
		}
	}
	return false, nil
}

// Each selected source cell must reprove its whole-arc contact before its
// exact meridian coordinate can add a cut to the shared blank shell.
func revolveLoftFacesShellSeamCutOf(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, manifest *revolveLoftFacesContactManifest,
	selector revolveLoftFacesShellSeamSelector,
) (revolveloftfaces.ShellCut, []revolveloftfaces.ArcSeamVertex, bool, error) {
	var empty revolveloftfaces.ShellCut
	sectionIndex, cellIndex := selector.sectionIndex, selector.cellIndex
	if sectionIndex < 0 || sectionIndex >= len(proof.section.Spans) ||
		sectionIndex >= len(manifest.sideOutcome) ||
		manifest.sideOutcome[sectionIndex] != revolveLoftFacesSideSourceCandidate {
		return empty, nil, false, nil
	}
	section := proof.section.Spans[sectionIndex]
	if section.Face == nil || section.Start.U == section.End.U ||
		section.Start.V <= 0 || section.End.V <= 0 {
		return empty, nil, false, nil
	}
	var side *revolveLoftFacesNonverticalSide
	for i := range manifest.nonvertical {
		if manifest.nonvertical[i].sectionIndex == sectionIndex {
			if side != nil {
				return empty, nil, false, nil
			}
			side = &manifest.nonvertical[i]
		}
	}
	if side == nil || faceNode(side.face) != section.Face ||
		!slices.Equal(side.origins, section.Origins) ||
		cellIndex < 0 || cellIndex >= len(side.cells) {
		return empty, nil, false, nil
	}
	cell := side.cells[cellIndex]
	mapped := cell.mapped
	if mapped.Side != revolveloftfaces.RevolveLoftFacesJointContact ||
		mapped.JointKind != revolveloftfaces.RevolveLoftFacesWholeArcContact || mapped.JointArc == nil ||
		mapped.TLo == nil || mapped.THi == nil || mapped.TLoOpen || mapped.THiOpen ||
		mapped.Segment < 0 || mapped.Segment >= len(proof.trim.Source.Source.Entities) ||
		proof.trim.Source.Source.Entities[mapped.Segment] != mapped.JointArc ||
		mapped.Band < 0 || mapped.Band+1 >= len(proof.trim.Maps) {
		return empty, nil, false, nil
	}
	if mapped.Band >= len(proof.sourceRefs) ||
		mapped.Segment >= len(proof.sourceRefs[mapped.Band]) ||
		cell.ref != proof.sourceRefs[mapped.Band][mapped.Segment] {
		return empty, nil, false, nil
	}
	liveFace, unique := revolveLoftFacesUniqueOriginFace(proof.tooth, cell.ref)
	if !unique || liveFace == nil || liveFace != cell.face {
		return empty, nil, false, nil
	}
	arcCells, certified, err := revolveloftfaces.RevolveLoftFacesArcSpanCells(ctx, proof.trim, section, mapped.Segment)
	if err != nil || !certified {
		return empty, nil, false, err
	}
	matched := false
	for _, source := range arcCells {
		matched = matched || revolveLoftFacesSameArcContact(source, mapped)
	}
	if !matched {
		return empty, nil, false, nil
	}
	stations, complete := revolveLoftFacesCompleteArcRun(proof.trim, mapped.Segment)
	if !complete {
		return empty, nil, false, nil
	}
	seam, _, _, built, err := revolveLoftFacesCertifiedArcPatches(ctx, proof.blank, proof.tooth,
		sectionIndex, mapped.Segment, mapped.Band,
		loftFaceInterval{Lo: mapped.TLo, Hi: mapped.THi}, stations)
	if err != nil || !built {
		return empty, nil, false, err
	}
	x := seam[0].Exact[0]
	for _, vertex := range seam[1:] {
		if vertex.Exact[0].Cmp(x) != 0 {
			return empty, nil, false, nil
		}
	}
	m, b := revolveloftfaces.RevolveLoftFacesMeridian(section)
	radius := lfAdd(lfMul(m, x), b)
	if radius.Sign() <= 0 {
		return empty, nil, false, nil
	}
	return revolveloftfaces.ShellCut{Section: sectionIndex, X: x,
		Radius: radius, Seam: true}, seam, true, nil
}

// This graph consumer places every certified whole-arc run and occupied heel
// arc on one closed Revolve shell. It makes no retained-face decision.
func revolveLoftFacesBlankShellWithAllRuns(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
) (*revolveLoftFacesBlankSeamShell, *revolveLoftFacesContactGraph, bool, error) {
	ctx = revolveLoftFacesWithCallScope(ctx)
	graph, ok, err := revolveLoftFacesContactGraphOf(ctx, proof)
	if err != nil || !ok || graph == nil || len(graph.runs) == 0 {
		return nil, nil, false, err
	}
	selectors := make([]revolveLoftFacesShellSeamSelector, 0, len(graph.runs))
	for _, run := range graph.runs {
		if run.cellID < 0 || run.cellID >= len(graph.cells) {
			return nil, nil, false, nil
		}
		cell := graph.cells[run.cellID]
		if cell.source == nil || cell.pairID < 0 ||
			graph.pairs[cell.pairID].verdict != revolveLoftFacesPairSourceCandidate {
			return nil, nil, false, nil
		}
		selectors = append(selectors, revolveLoftFacesShellSeamSelector{
			sectionIndex: cell.sectionIndex, cellIndex: cell.cellIndex})
	}
	capCuts, valid := revolveLoftFacesGraphCapCuts(graph)
	if !valid {
		return nil, nil, false, nil
	}
	shell, built, err := revolveLoftFacesBlankShellAtRunsWithCuts(ctx, proof,
		graph.manifest, selectors, capCuts)
	if err != nil || !built || shell == nil || shell.FaceBound >= 0.1 ||
		len(shell.Seams) != len(graph.runs) ||
		len(shell.TriangleCells) != len(shell.Mesh.Tris) {
		return nil, nil, false, err
	}
	shell.seamRunIDs = make([][]int, len(graph.runs))
	for i := range graph.runs {
		view := *shell
		view.seam = shell.Seams[i]
		bound, bindErr := graph.bindBlankShell(ctx, proof, &graph.runs[i], &view)
		if bindErr != nil || !bound {
			return nil, nil, false, bindErr
		}
		shell.seamRunIDs[i] = view.seamIDs
	}
	shell.seamIDs = shell.seamRunIDs[0]
	return shell, graph, true, nil
}
