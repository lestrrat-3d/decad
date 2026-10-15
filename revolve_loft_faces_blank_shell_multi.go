package decad

import (
	"context"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Each selected source cell must reprove its whole-arc contact before its
// exact meridian coordinate can add a cut to the shared blank shell.
func revolveLoftFacesShellSeamCutOf(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, manifest *revolveLoftFacesContactManifest,
	selector revolveLoftFacesShellSeamSelector,
) (revolveLoftFacesShellCut, []revolveLoftFacesSeamVertex, bool, error) {
	var empty revolveLoftFacesShellCut
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
	if mapped.side != revolveLoftFacesJointContact ||
		mapped.jointKind != revolveLoftFacesWholeArcContact || mapped.jointArc == nil ||
		mapped.tLo == nil || mapped.tHi == nil || mapped.tLoOpen || mapped.tHiOpen ||
		mapped.segment < 0 || mapped.segment >= len(proof.trim.Source.Source.Entities) ||
		proof.trim.Source.Source.Entities[mapped.segment] != mapped.jointArc ||
		mapped.band < 0 || mapped.band+1 >= len(proof.trim.Maps) {
		return empty, nil, false, nil
	}
	if mapped.band >= len(proof.sourceRefs) ||
		mapped.segment >= len(proof.sourceRefs[mapped.band]) ||
		cell.ref != proof.sourceRefs[mapped.band][mapped.segment] {
		return empty, nil, false, nil
	}
	liveFace, unique := revolveLoftFacesUniqueOriginFace(proof.tooth, cell.ref)
	if !unique || liveFace == nil || liveFace != cell.face {
		return empty, nil, false, nil
	}
	arcCells, certified, err := revolveLoftFacesArcSpanCells(ctx, proof.trim, section, mapped.segment)
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
	stations, complete := revolveLoftFacesCompleteArcRun(proof.trim, mapped.segment)
	if !complete {
		return empty, nil, false, nil
	}
	seam, _, _, built, err := revolveLoftFacesCertifiedArcPatches(ctx, proof.blank, proof.tooth,
		sectionIndex, mapped.segment, mapped.band,
		loftFaceInterval{Lo: mapped.tLo, Hi: mapped.tHi}, stations)
	if err != nil || !built {
		return empty, nil, false, err
	}
	x := seam[0].exact[0]
	for _, vertex := range seam[1:] {
		if vertex.exact[0].Cmp(x) != 0 {
			return empty, nil, false, nil
		}
	}
	m, b := revolveLoftFacesMeridian(section)
	radius := lfAdd(lfMul(m, x), b)
	if radius.Sign() <= 0 {
		return empty, nil, false, nil
	}
	return revolveLoftFacesShellCut{sectionIndex: sectionIndex, x: x,
		radius: radius, seam: true}, seam, true, nil
}

// One angular ray may carry station anchors from several different meridian
// cuts. Their event IDs remain separate even when their exact rays coincide.
func revolveLoftFacesShellRaysForRuns(n int, seams [][]revolveLoftFacesSeamVertex,
	capVertices []revolveLoftFacesCapSeamVertex) []revolveLoftFacesRay {
	rays := revolveLoftFacesShellRays(n, nil, capVertices)
	if rays == nil {
		return nil
	}
	for run, seam := range seams {
		for station, point := range seam {
			rays = append(rays, revolveLoftFacesRay{y: point.exact[1], z: point.exact[2],
				seamIndex: -1, seamRefs: []revolveLoftFacesRaySeamRef{{run: run, station: station}}})
		}
	}
	sort.Slice(rays, func(i, j int) bool { return revolveLoftFacesRayBefore(rays[i], rays[j]) })
	unique := rays[:0]
	for _, ray := range rays {
		if len(unique) == 0 || !revolveLoftFacesSameRay(unique[len(unique)-1], ray) {
			unique = append(unique, ray)
			continue
		}
		previous := &unique[len(unique)-1]
		if len(ray.capIDs) != 0 || len(previous.capIDs) != 0 {
			return nil
		}
		for _, ref := range ray.seamRefs {
			for _, existing := range previous.seamRefs {
				if existing.run == ref.run {
					return nil
				}
			}
			previous.seamRefs = append(previous.seamRefs, ref)
		}
	}
	return unique
}

// Every source anchor must stay in the same cyclic angular order as the
// shared ring grid. A cap endpoint and a seam station cannot share a ray.
func revolveLoftFacesAllRayOrderBounded(rays []revolveLoftFacesRay,
	seams [][]revolveLoftFacesSeamVertex, capVertices []revolveLoftFacesCapSeamVertex) bool {
	if len(rays) < 4 {
		return false
	}
	boxes := make([][]revolveLoftFacesRayBox, len(rays))
	seenCaps := make([]bool, len(capVertices))
	seenStations := make([][]bool, len(seams))
	for run := range seams {
		seenStations[run] = make([]bool, len(seams[run]))
	}
	for index, ray := range rays {
		if ray.y == nil || ray.z == nil || len(ray.capIDs) > 1 ||
			len(ray.capIDs) > 0 && len(ray.seamRefs) > 0 {
			return false
		}
		if len(ray.capIDs) == 0 && len(ray.seamRefs) == 0 {
			boxes[index] = []revolveLoftFacesRayBox{{
				y: loftFaceInterval{Lo: ray.y, Hi: ray.y},
				z: loftFaceInterval{Lo: ray.z, Hi: ray.z}}}
		}
		for _, id := range ray.capIDs {
			if id < 0 || id >= len(capVertices) || seenCaps[id] {
				return false
			}
			seenCaps[id] = true
			boundary := capVertices[id].boundary
			boxes[index] = append(boxes[index], revolveLoftFacesRayBox{
				y: boundary.box[1], z: boundary.box[2]})
		}
		for _, ref := range ray.seamRefs {
			if ref.run < 0 || ref.run >= len(seams) ||
				ref.station < 0 || ref.station >= len(seams[ref.run]) ||
				seenStations[ref.run][ref.station] {
				return false
			}
			seenStations[ref.run][ref.station] = true
			point := seams[ref.run][ref.station]
			if proofbound.IsNonFinite(point.loftBound) || point.loftBound < 0 {
				return false
			}
			bound := lfF(point.loftBound)
			y, z := lfF(point.point.Y), lfF(point.point.Z)
			boxes[index] = append(boxes[index], revolveLoftFacesRayBox{
				y: loftFaceInterval{Lo: lfSub(y, bound), Hi: lfAdd(y, bound)},
				z: loftFaceInterval{Lo: lfSub(z, bound), Hi: lfAdd(z, bound)}})
		}
		for _, box := range boxes[index] {
			if !revolveLoftFacesRayBoxValid(box) {
				return false
			}
		}
	}
	for _, seen := range seenCaps {
		if !seen {
			return false
		}
	}
	for _, run := range seenStations {
		for _, seen := range run {
			if !seen {
				return false
			}
		}
	}
	for index := range rays {
		next := (index + 1) % len(rays)
		for _, first := range boxes[index] {
			for _, second := range boxes[next] {
				if revolveLoftFacesRayCrossRange(first, second).Lo.Sign() <= 0 {
					return false
				}
			}
		}
	}
	return true
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
	if err != nil || !built || shell == nil || shell.faceBound >= 0.1 ||
		len(shell.seams) != len(graph.runs) ||
		len(shell.triangleCells) != len(shell.mesh.Tris) {
		return nil, nil, false, err
	}
	shell.seamRunIDs = make([][]int, len(graph.runs))
	for i := range graph.runs {
		view := *shell
		view.seam = shell.seams[i]
		bound, bindErr := graph.bindBlankShell(ctx, proof, &graph.runs[i], &view)
		if bindErr != nil || !bound {
			return nil, nil, false, bindErr
		}
		shell.seamRunIDs[i] = view.seamIDs
	}
	shell.seamIDs = shell.seamRunIDs[0]
	return shell, graph, true, nil
}
