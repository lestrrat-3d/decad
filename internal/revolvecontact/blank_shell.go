package revolvecontact

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
)

// A source-aligned blank shell is the internal shell mesh bound to its live
// sources. It stays in the root because its cap rings keep the root cap-arc
// records and side ties that later graph checks compare.
type BlankSeamShell struct {
	revolveloftfaces.ShellMesh
	Seam       []int
	SeamIDs    []int
	SeamRunIDs [][]int
	CapRings   []ShellCapRing
}

type ShellSeamSelector struct {
	SectionIndex, CellIndex int
}

type ShellCapRing struct {
	revolveloftfaces.ShellCapRing
	Arc              *ToolCapOccupiedArc
	EndpointProof    [2]CapSeamVertex
	Ties             *HeelArcSideTies
	EndpointGraphIDs [2]int
	EndpointCellIDs  [2]int
}

// A cap cut keeps the root occupied-arc record that certified it and the
// side ties derived for it.
type ShellCut struct {
	revolveloftfaces.ShellCut
	Arc  *ToolCapOccupiedArc
	Ties *HeelArcSideTies
}

// BlankShellAtRunsWithCuts builds every authenticated Revolve
// wall and end cap on one angular grid. Each selected whole-arc source contact
// splits its finite meridian side. Source cells select the ring; held output
// edges never supply a crossing location.
func BlankShellAtRunsWithCuts(ctx context.Context,
	proof *SourceCertificate,
	manifest *ContactManifest,
	selectors []ShellSeamSelector, capCuts []ShellCut,
) (*BlankSeamShell, bool, error) {
	if ctx == nil || proof == nil || proof.Trim == nil || proof.Tooth == nil || manifest == nil ||
		len(selectors) == 0 ||
		!SectionCurrent(proof) {
		return nil, false, nil
	}
	payload, payloadOK := proof.Tooth.Payload.(facetedrecord.Payload)
	if !payloadOK || payload.LoftFacesTrim != proof.Trim ||
		!TrimCurrent(proof.Tooth, proof.Trim) {
		return nil, false, nil
	}
	census, currentOK, err := CensusIn(ctx, proof)
	if err != nil || !currentOK {
		return nil, false, err
	}
	current := census.Manifest
	if !SameShellManifest(manifest, current) {
		return nil, false, nil
	}
	manifest = current
	seams := make([][]revolveloftfaces.ArcSeamVertex, len(selectors))
	seamCuts := make([]revolveloftfaces.ShellCut, len(selectors))
	seenSelectors := make(map[ShellSeamSelector]struct{}, len(selectors))
	for run, selector := range selectors {
		if _, duplicate := seenSelectors[selector]; duplicate {
			return nil, false, nil
		}
		seenSelectors[selector] = struct{}{}
		cut, vertices, valid, cutErr := ShellSeamCutOf(ctx, proof, manifest, selector)
		if cutErr != nil || !valid {
			return nil, false, cutErr
		}
		cut.SeamRun = run
		seamCuts[run], seams[run] = cut, vertices
	}
	n, chordBound, sized := revolveloftfaces.ShellGridSize(proof.Section.Spans, proof.Section.AxisSpan)
	if !sized {
		return nil, false, nil
	}
	capVertices := make([]CapSeamVertex, 0, 2*len(capCuts))
	anchors := revolveloftfaces.ShellAnchors{Seams: seams,
		Caps: make([]revolveloftfaces.ShellRayCap, 0, 2*len(capCuts))}
	for i := range capCuts {
		vertices, built, err := CapArcSeamVerticesIn(ctx, proof, census, capCuts[i].Arc)
		if err != nil || !built {
			return nil, false, err
		}
		ties, tied, err := HeelArcSideTiesIn(ctx, proof, census, capCuts[i].Arc)
		if err != nil || !tied {
			return nil, false, err
		}
		capCuts[i].CapIDs = [2]int{len(capVertices), len(capVertices) + 1}
		capCuts[i].ThroughPositiveY = capCuts[i].Arc.ThroughPositiveY
		capCuts[i].Ties = ties
		capVertices = append(capVertices, vertices[:]...)
		for _, vertex := range vertices {
			anchors.Caps = append(anchors.Caps, revolveloftfaces.ShellRayCap{
				Point: vertex.Point, BlankBound: vertex.BlankBound,
				Box: revolveloftfaces.RayBox{Y: vertex.Boundary.Box[1], Z: vertex.Boundary.Box[2]}})
		}
	}
	rays, positiveYAxis, gridOK := revolveloftfaces.ShellGrid(n, anchors, len(proof.Section.Spans))
	if !gridOK {
		return nil, false, nil
	}
	groups := make([]int, len(proof.Section.Spans))
	var meshGroups []facetedrecord.Group
	for i, span := range proof.Section.Spans {
		groups[i] = -1
		if span.Face == nil {
			if i != proof.Section.AxisSpan {
				return nil, false, nil
			}
			continue
		}
		if i >= len(manifest.SideOutcome) {
			return nil, false, nil
		}
		current, err := ShellSideCurrent(ctx, proof, manifest, i, span)
		if err != nil || !current {
			return nil, false, err
		}
		groups[i] = len(meshGroups)
		meshGroups = append(meshGroups,
			facetedrecord.Group{Origins: append([]provenance.Ref(nil), span.Origins...), Planar: span.Face.Surface.Kind() == surfacegeom.KindPlane || span.Face.HeldPlanar})
	}
	meshCaps := make([]revolveloftfaces.ShellCut, len(capCuts))
	for i, cut := range capCuts {
		if cut.Arc == nil || cut.Section < 0 ||
			cut.Section >= len(proof.Section.Spans) ||
			cut.X == nil || cut.Radius == nil || cut.Seam ||
			cut.Arc.Circle.X == nil || cut.Arc.Circle.Radius == nil ||
			cut.Arc.Circle.Span != cut.Section ||
			cut.Arc.Circle.X.Cmp(cut.X) != 0 ||
			cut.Arc.Circle.Radius.Cmp(cut.Radius) != 0 {
			return nil, false, nil
		}
		current, err := ToolCapOccupiedArcCurrentIn(ctx, proof, census, cut.Arc)
		if err != nil || !current {
			return nil, false, err
		}
		meshCaps[i] = cut.ShellCut
	}
	mesh, rings, built, err := revolveloftfaces.BuildShellMesh(ctx, revolveloftfaces.ShellMeshInput{
		Spans: proof.Section.Spans, Groups: groups, MeshGroups: meshGroups,
		Anchors: anchors, Rays: rays, PositiveYAxis: positiveYAxis, ChordBound: chordBound,
		SeamCuts: seamCuts, CapCuts: meshCaps})
	if err != nil || !built {
		return nil, false, err
	}
	out := &BlankSeamShell{ShellMesh: mesh,
		CapRings: make([]ShellCapRing, len(rings))}
	out.Seam = out.Seams[0]
	for i, ring := range rings {
		cut := capCuts[ring.Cut]
		out.CapRings[i] = ShellCapRing{ShellCapRing: ring, Arc: cut.Arc, Ties: cut.Ties,
			EndpointProof: [2]CapSeamVertex{
				capVertices[cut.CapIDs[0]], capVertices[cut.CapIDs[1]]}}
	}
	return out, true, nil
}

// A candidate must carry its retained source cells; a marked outcome alone
// cannot turn an empty nonvertical face into certified contact.
func ShellSideCurrent(ctx context.Context,
	proof *SourceCertificate,
	manifest *ContactManifest,
	index int, span facetedrecord.RevolveSectionSpan) (bool, error) {
	switch manifest.SideOutcome[index] {
	case SideSourceCandidate:
		if span.Start.U != span.End.U {
			for _, side := range manifest.Nonvertical {
				if side.SectionIndex == index {
					return side.Face == span.Face && len(side.Cells) > 0 &&
						slices.Equal(side.Origins, span.Origins), nil
				}
			}
			return false, nil
		}
		side, outcome, err := VerticalSideClassify(ctx, proof, index)
		if err != nil || outcome != revolveloftfaces.VerticalContact ||
			side == nil || side.Face != span.Face || len(side.Cells) == 0 {
			return false, err
		}
		for _, entry := range manifest.Vertical {
			if entry.SectionIndex == index {
				return entry.Outcome == revolveloftfaces.VerticalContact &&
					entry.Side != nil && entry.Side.Face == span.Face &&
					len(entry.Side.Cells) > 0, nil
			}
		}
	case SideMiss:
		if span.Start.U != span.End.U {
			return false, nil
		}
		side, outcome, err := VerticalSideClassify(ctx, proof, index)
		if err != nil || outcome != revolveloftfaces.VerticalMiss || side != nil {
			return false, err
		}
		for _, entry := range manifest.Vertical {
			if entry.SectionIndex == index {
				return entry.Outcome == revolveloftfaces.VerticalMiss && entry.Side == nil, nil
			}
		}
	}
	return false, nil
}

// Each selected source cell must reprove its whole-arc contact before its
// exact meridian coordinate can add a cut to the shared blank shell.
func ShellSeamCutOf(ctx context.Context,
	proof *SourceCertificate, manifest *ContactManifest,
	selector ShellSeamSelector,
) (revolveloftfaces.ShellCut, []revolveloftfaces.ArcSeamVertex, bool, error) {
	var empty revolveloftfaces.ShellCut
	sectionIndex, cellIndex := selector.SectionIndex, selector.CellIndex
	if sectionIndex < 0 || sectionIndex >= len(proof.Section.Spans) ||
		sectionIndex >= len(manifest.SideOutcome) ||
		manifest.SideOutcome[sectionIndex] != SideSourceCandidate {
		return empty, nil, false, nil
	}
	section := proof.Section.Spans[sectionIndex]
	if section.Face == nil || section.Start.U == section.End.U ||
		section.Start.V <= 0 || section.End.V <= 0 {
		return empty, nil, false, nil
	}
	var side *NonverticalSide
	for i := range manifest.Nonvertical {
		if manifest.Nonvertical[i].SectionIndex == sectionIndex {
			if side != nil {
				return empty, nil, false, nil
			}
			side = &manifest.Nonvertical[i]
		}
	}
	if side == nil || side.Face != section.Face ||
		!slices.Equal(side.Origins, section.Origins) ||
		cellIndex < 0 || cellIndex >= len(side.Cells) {
		return empty, nil, false, nil
	}
	cell := side.Cells[cellIndex]
	mapped := cell.Mapped
	if mapped.Side != revolveloftfaces.RevolveLoftFacesJointContact ||
		mapped.JointKind != revolveloftfaces.RevolveLoftFacesWholeArcContact || mapped.JointArc == nil ||
		mapped.TLo == nil || mapped.THi == nil || mapped.TLoOpen || mapped.THiOpen ||
		mapped.Segment < 0 || mapped.Segment >= len(proof.Trim.Source.Source.Entities) ||
		proof.Trim.Source.Source.Entities[mapped.Segment] != mapped.JointArc ||
		mapped.Band < 0 || mapped.Band+1 >= len(proof.Trim.Maps) {
		return empty, nil, false, nil
	}
	if mapped.Band >= len(proof.SourceRefs) ||
		mapped.Segment >= len(proof.SourceRefs[mapped.Band]) ||
		cell.Ref != proof.SourceRefs[mapped.Band][mapped.Segment] {
		return empty, nil, false, nil
	}
	liveFace, unique := UniqueOriginFace(proof.Tooth, cell.Ref)
	if !unique || liveFace == nil || liveFace != cell.Face {
		return empty, nil, false, nil
	}
	arcCells, certified, err := revolveloftfaces.RevolveLoftFacesArcSpanCells(ctx, proof.Trim, section, mapped.Segment)
	if err != nil || !certified {
		return empty, nil, false, err
	}
	matched := false
	for _, source := range arcCells {
		matched = matched || revolveloftfaces.SameArcContact(source, mapped)
	}
	if !matched {
		return empty, nil, false, nil
	}
	stations, complete := revolveloftfaces.CompleteArcRun(proof.Trim, mapped.Segment)
	if !complete {
		return empty, nil, false, nil
	}
	seam, _, _, built, err := CertifiedArcPatches(ctx, proof.Blank, proof.Tooth,
		sectionIndex, mapped.Segment, mapped.Band,
		loftface.Interval{Lo: mapped.TLo, Hi: mapped.THi}, stations)
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
func BlankShellWithAllRuns(ctx context.Context,
	proof *SourceCertificate,
) (*BlankSeamShell, *ContactGraph, bool, error) {
	ctx = WithCallScope(ctx)
	graph, ok, err := ContactGraphOf(ctx, proof)
	if err != nil || !ok || graph == nil || len(graph.Runs) == 0 {
		return nil, nil, false, err
	}
	selectors := make([]ShellSeamSelector, 0, len(graph.Runs))
	for _, run := range graph.Runs {
		if run.CellID < 0 || run.CellID >= len(graph.Cells) {
			return nil, nil, false, nil
		}
		cell := graph.Cells[run.CellID]
		if cell.Source == nil || cell.PairID < 0 ||
			graph.Pairs[cell.PairID].Verdict != PairSourceCandidate {
			return nil, nil, false, nil
		}
		selectors = append(selectors, ShellSeamSelector{
			SectionIndex: cell.SectionIndex, CellIndex: cell.CellIndex})
	}
	capCuts, valid := GraphCapCuts(graph)
	if !valid {
		return nil, nil, false, nil
	}
	shell, built, err := BlankShellAtRunsWithCuts(ctx, proof,
		graph.Manifest, selectors, capCuts)
	if err != nil || !built || shell == nil || shell.FaceBound >= 0.1 ||
		len(shell.Seams) != len(graph.Runs) ||
		len(shell.TriangleCells) != len(shell.Mesh.Tris) {
		return nil, nil, false, err
	}
	shell.SeamRunIDs = make([][]int, len(graph.Runs))
	for i := range graph.Runs {
		view := *shell
		view.Seam = shell.Seams[i]
		bound, bindErr := graph.BindBlankShell(ctx, proof, &graph.Runs[i], &view)
		if bindErr != nil || !bound {
			return nil, nil, false, bindErr
		}
		shell.SeamRunIDs[i] = view.SeamIDs
	}
	shell.SeamIDs = shell.SeamRunIDs[0]
	return shell, graph, true, nil
}
