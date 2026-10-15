package decad

import "context"

// The fresh source census supplies every occupied heel circle. Each circle
// splits its authenticated Revolve meridian before any retained-face cut is
// attempted. This is still a complete blank shell, not a Union result.
func revolveLoftFacesBlankShellWithCapRings(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, sectionIndex, cellIndex int,
) (*revolveLoftFacesBlankSeamShell, bool, error) {
	ctx = revolveLoftFacesWithCallScope(ctx)
	graph, ok, err := revolveLoftFacesContactGraphOf(ctx, proof)
	if err != nil || !ok || graph == nil || graph.manifest == nil {
		return nil, false, err
	}
	if sectionIndex < 0 || sectionIndex >= len(proof.section.Spans) {
		return nil, false, nil
	}
	var selected *revolveLoftFacesGraphRun
	for i := range graph.runs {
		run := &graph.runs[i]
		cell := graph.cells[run.cellID]
		if cell.sectionIndex == sectionIndex && cell.cellIndex == cellIndex &&
			cell.source != nil && cell.blankFace == proof.section.Spans[sectionIndex].Face {
			if selected != nil {
				return nil, false, nil
			}
			selected = run
		}
	}
	if selected == nil {
		return nil, false, nil
	}
	cuts, valid := revolveLoftFacesGraphCapCuts(graph)
	if !valid {
		return nil, false, nil
	}
	shell, built, err := revolveLoftFacesBlankShellAtArcWithCuts(ctx, proof, graph.manifest,
		sectionIndex, cellIndex, cuts)
	if err != nil || !built {
		return nil, false, err
	}
	if shell == nil || shell.faceBound >= 0.1 {
		return nil, false, nil
	}
	bound, bindErr := graph.bindBlankShell(ctx, proof, selected, shell)
	if bindErr != nil || !bound {
		return nil, false, bindErr
	}
	return shell, true, nil
}

func revolveLoftFacesGraphCapCuts(graph *revolveLoftFacesContactGraph) (
	[]revolveLoftFacesShellCut, bool,
) {
	if graph == nil {
		return nil, false
	}
	var cuts []revolveLoftFacesShellCut
	for _, event := range graph.arcs {
		if event.pairID < 0 || event.pairID >= len(graph.pairs) {
			return nil, false
		}
		pair := graph.pairs[event.pairID]
		arc := event.arc
		if arc == nil || arc.circle.tool != 1 || arc.circle.sourceMiss ||
			arc.circle.blankFace != pair.blankFace ||
			arc.circle.capFace != pair.trimmedFace ||
			arc.circle.span != pair.sectionIndex ||
			event.endpointIDs[0] == event.endpointIDs[1] ||
			event.endpointCellIDs[0] == event.endpointCellIDs[1] ||
			arc.circle.x == nil || arc.circle.radius == nil ||
			arc.circle.radius.Sign() <= 0 {
			return nil, false
		}
		cuts = append(cuts, revolveLoftFacesShellCut{
			sectionIndex: pair.sectionIndex, x: arc.circle.x,
			radius: arc.circle.radius, arc: arc})
	}
	if len(cuts) == 0 {
		return nil, false
	}
	return cuts, true
}

// The certified cap arc selects one of the two walks between its source
// endpoints. The positive Y axis distinguishes the occupied walk.
func revolveLoftFacesOccupiedRingPath(ring []int, endpoints [2]int,
	positiveYAxis int, throughPositiveY bool) ([]int, bool) {
	if len(ring) < 4 || positiveYAxis < 0 || positiveYAxis >= len(ring) ||
		endpoints[0] == endpoints[1] {
		return nil, false
	}
	indices := [2]int{-1, -1}
	for index, vertex := range ring {
		for side, endpoint := range endpoints {
			if vertex != endpoint {
				continue
			}
			if indices[side] >= 0 {
				return nil, false
			}
			indices[side] = index
		}
	}
	if indices[0] < 0 || indices[1] < 0 ||
		indices[0] == positiveYAxis || indices[1] == positiveYAxis {
		return nil, false
	}
	walk := func(direction int) ([]int, bool) {
		var result []int
		for index := indices[0]; ; index = (index + direction + len(ring)) % len(ring) {
			result = append(result, ring[index])
			if index == indices[1] {
				break
			}
			if len(result) >= len(ring) {
				return nil, false
			}
		}
		containsAxis := false
		for _, vertex := range result {
			containsAxis = containsAxis || vertex == ring[positiveYAxis]
		}
		return result, containsAxis
	}
	forward, hasAxis := walk(1)
	if len(forward) == 0 {
		return nil, false
	}
	if hasAxis == throughPositiveY {
		return forward, true
	}
	backward, hasAxis := walk(-1)
	return backward, len(backward) >= 2 && hasAxis == throughPositiveY
}
