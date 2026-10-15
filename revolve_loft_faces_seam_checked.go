package decad

import (
	"context"
	"slices"
	"sync"
)

// revolveLoftFacesCertifiedArcPatches is the source-to-held gate for one
// complete radial Sketch arc. It compares the caller's root bracket and
// station run with the live source certificate and contact graph of the
// bodies, built once per top-level call, before using them. The returned
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
	proof, graph, ok, err := revolveLoftFacesLiveGraph(ctx, blank, tooth)
	if err != nil || !ok || sectionIndex >= len(proof.section.Spans) {
		return nil, empty, empty, false, err
	}
	section := proof.section.Spans[sectionIndex]
	if section.Face == nil || section.Start.U == section.End.U {
		return nil, empty, empty, false, nil
	}
	var selected *revolveLoftFacesGraphRun
	for i := range graph.runs {
		run := &graph.runs[i]
		cell := graph.cells[run.cellID]
		if cell.sectionIndex != sectionIndex || cell.blankFace != section.Face ||
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
	if trim == nil || trim.Source == nil || segment < 0 ||
		segment >= len(trim.Source.Profile.Outer.Segments) ||
		len(trim.StationGroup) != len(trim.Source.SourceStation) ||
		len(trim.StationGroup) < 3 {
		return nil, false
	}
	groups := trim.StationGroup
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

type revolveLoftFacesCallScopeKey struct{}

// A call scope holds the records that one top-level consumer call has
// already derived from the live bodies. It lives only in that call's
// context and is never stored anywhere else. Only successes are recorded.
type revolveLoftFacesCallScope struct {
	mu       sync.Mutex
	live     map[[2]*Body]revolveLoftFacesScopedGraph
	censuses map[*revolveLoftFacesSourceCertificate]*revolveLoftFacesPairCensus
}

type revolveLoftFacesScopedGraph struct {
	proof *revolveLoftFacesSourceCertificate
	graph *revolveLoftFacesContactGraph
}

// revolveLoftFacesWithCallScope opens a call scope for the outermost
// top-level consumer. A context that already carries one is returned
// unchanged, so nested consumers share the outer scope.
func revolveLoftFacesWithCallScope(ctx context.Context) context.Context {
	if ctx == nil || revolveLoftFacesCallScopeOf(ctx) != nil {
		return ctx
	}
	return context.WithValue(ctx, revolveLoftFacesCallScopeKey{}, &revolveLoftFacesCallScope{
		live:     make(map[[2]*Body]revolveLoftFacesScopedGraph),
		censuses: make(map[*revolveLoftFacesSourceCertificate]*revolveLoftFacesPairCensus),
	})
}

func revolveLoftFacesCallScopeOf(ctx context.Context) *revolveLoftFacesCallScope {
	if ctx == nil {
		return nil
	}
	scope, _ := ctx.Value(revolveLoftFacesCallScopeKey{}).(*revolveLoftFacesCallScope)
	return scope
}

// revolveLoftFacesLiveGraph certifies blank and tooth and builds the contact
// graph of that certificate. Inside a call scope the first success is kept
// and returned to every later check of the same pair. The result is only for
// reading: a caller must never hold or edit a scoped graph, whose cells point
// into its census.
func revolveLoftFacesLiveGraph(ctx context.Context, blank, tooth *Body) (
	*revolveLoftFacesSourceCertificate, *revolveLoftFacesContactGraph, bool, error) {
	scope := revolveLoftFacesCallScopeOf(ctx)
	key := [2]*Body{blank, tooth}
	if scope != nil {
		scope.mu.Lock()
		hit, found := scope.live[key]
		scope.mu.Unlock()
		if found {
			return hit.proof, hit.graph, true, nil
		}
	}
	proof, ok, err := certifyRevolveLoftFacesSource(ctx, blank, tooth)
	if err != nil || !ok {
		return nil, nil, false, err
	}
	graph, ok, err := revolveLoftFacesContactGraphOf(ctx, proof)
	if err != nil || !ok {
		return proof, nil, false, err
	}
	if scope != nil {
		scope.mu.Lock()
		scope.live[key] = revolveLoftFacesScopedGraph{proof: proof, graph: graph}
		scope.mu.Unlock()
	}
	return proof, graph, true, nil
}

// revolveLoftFacesCensusIn derives the census of proof, once per source
// certificate inside a call scope.
func revolveLoftFacesCensusIn(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate) (*revolveLoftFacesPairCensus, bool, error) {
	scope := revolveLoftFacesCallScopeOf(ctx)
	if scope != nil && proof != nil {
		scope.mu.Lock()
		hit, found := scope.censuses[proof]
		scope.mu.Unlock()
		if found {
			return hit, true, nil
		}
	}
	census, ok, err := revolveLoftFacesCensus(ctx, proof)
	if err == nil && ok && scope != nil && proof != nil {
		scope.mu.Lock()
		scope.censuses[proof] = census
		scope.mu.Unlock()
	}
	return census, ok, err
}
