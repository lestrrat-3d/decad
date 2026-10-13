package decad

import (
	"context"
	"slices"
)

// revolveLoftFacesCertifiedArcPatches is the source-to-held gate for one
// complete radial Sketch arc. It rederives the live face-pair census before
// using a root bracket or station run supplied by a caller. The returned
// pieces remain open and cannot be used as a Boolean result by themselves.
func revolveLoftFacesCertifiedArcPatches(ctx context.Context, blank, tooth *Body,
	sectionIndex, segment, band int, root loftFaceInterval, stations []int,
) ([]revolveLoftFacesSeamVertex, revolveLoftFacesSeamPatch,
	revolveLoftFacesSeamPatch, bool, error) {
	var empty revolveLoftFacesSeamPatch
	if ctx == nil || root.Lo == nil || root.Hi == nil ||
		sectionIndex < 0 || segment < 0 || band < 0 {
		return nil, empty, empty, false, nil
	}
	proof, ok, err := certifyRevolveLoftFacesSource(ctx, blank, tooth)
	if err != nil || !ok {
		return nil, empty, empty, false, err
	}
	graph, ok, err := revolveLoftFacesContactGraphOf(ctx, proof)
	if err != nil || !ok || sectionIndex >= len(proof.section.spans) {
		return nil, empty, empty, false, err
	}
	section := proof.section.spans[sectionIndex]
	if section.face == nil || section.start.U == section.end.U {
		return nil, empty, empty, false, nil
	}
	var selected *revolveLoftFacesGraphRun
	for i := range graph.runs {
		run := &graph.runs[i]
		cell := graph.cells[run.cellID]
		if cell.sectionIndex != sectionIndex || cell.blankFace != section.face ||
			cell.pairID < 0 || cell.source == nil ||
			graph.pairs[cell.pairID].verdict != revolveLoftFacesPairSourceCandidate {
			continue
		}
		mapped := cell.source.mapped
		if mapped.segment != segment || mapped.band != band ||
			mapped.jointArc == nil || mapped.tLo == nil || mapped.tHi == nil ||
			mapped.tLo.Cmp(root.Lo) != 0 || mapped.tHi.Cmp(root.Hi) != 0 {
			continue
		}
		if selected != nil {
			return nil, empty, empty, false, nil
		}
		selected = run
	}
	if selected == nil || !slices.Equal(stations, selected.stations) {
		return nil, empty, empty, false, nil
	}
	seam, ok, err := revolveLoftFacesArcSeamVerticesUnchecked(ctx, proof.trim,
		band, selected.stations, root, section)
	if err != nil || !ok {
		return nil, empty, empty, false, err
	}
	blankPatch, loftPatch, ok, err := revolveLoftFacesArcPatchesUnchecked(ctx, proof.trim,
		band, segment, section, seam)
	if err != nil || !ok {
		return nil, empty, empty, false, err
	}
	if !graph.bindOpenPatches(selected, seam, &blankPatch, &loftPatch) {
		return nil, empty, empty, false, nil
	}
	return seam, blankPatch, loftPatch, true, nil
}

// The station group labels identify source chords. A complete cyclic run
// contains every chord of this Sketch arc and its next shared endpoint.
func revolveLoftFacesCompleteArcRun(trim *loftFacesTrimRecord, segment int) ([]int, bool) {
	if trim == nil || trim.source == nil || segment < 0 ||
		segment >= len(trim.source.profile.Outer.Segments) ||
		len(trim.stationGroup) != len(trim.source.sourceStation) ||
		len(trim.stationGroup) < 3 {
		return nil, false
	}
	groups := trim.stationGroup
	start, runs := -1, 0
	for index, group := range groups {
		if group == segment && groups[(index+len(groups)-1)%len(groups)] != segment {
			start, runs = index, runs+1
		}
	}
	if runs != 1 {
		return nil, false
	}
	stations := make([]int, 0, len(groups)+1)
	for index := start; groups[index] == segment; index = (index + 1) % len(groups) {
		stations = append(stations, index)
		if len(stations) >= len(groups) {
			return nil, false
		}
	}
	return append(stations, (stations[len(stations)-1]+1)%len(groups)), true
}
