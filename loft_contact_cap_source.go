package decad

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// loftCapSourceEdge pairs one ordered Sketch fragment with its original
// producer segment. tStart and tEnd are parameters of the private scene
// entity, whose full walk was built from span.curve. They do not name a
// selected output edge or an occupied source face at this axial level.
type loftCapSourceEdge struct {
	span           loftSourceCapSpan
	tStart, tEnd   float64
	reversed       bool
	partial        bool
	walkWithSource bool
}

type loftCapSourceCell struct {
	prism, loft bool
	edges       []loftCapSourceEdge
}

// loftCapSourceOrder is a transient reading of both exact Sketch cap scenes.
// It does not certify output patch ownership, boundary coverage, or volume.
type loftCapSourceOrder struct {
	source  loftSourceSpanCaps
	caps    [2][]loftCapSourceCell
	quarter [2][]loftQuarterSourceEvent
	visits  uint64
}

// certifyLoftCapSourceOrder reads each cap scene under the consumer's source
// wall budget. Sketch supplies cut ranges and walk order; every resulting
// fragment must authenticate to one original LineSeg or ArcSeg. Coincident
// source spans await a separate two-owner certificate and safely decline.
// The 4096-segment scene preflight bounds Sketch publication; tag scans and
// the later profile-record walk are separately charged to the source wall.
func certifyLoftCapSourceOrder(ctx context.Context, budget *proofbound.WorkBudget,
	source loftSourceSpanCaps, quarter ...*loftQuarterWork) (loftCapSourceOrder, bool, error) {
	var no loftCapSourceOrder
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if budget == nil || source.prism == nil || source.loft == nil ||
		len(quarter) > 1 || len(quarter) == 1 &&
		(quarter[0] == nil || quarter[0].budget != budget) {
		return no, false, nil
	}
	pp, prismOK := source.prism.n.Payload.(prismPayload)
	lp, loftOK := source.loft.n.Payload.(loftPayload)
	if !prismOK || !loftOK || pp.SectionDelta != 0 {
		return no, false, nil
	}
	result := loftCapSourceOrder{source: source}
	shared := [2]bool{}
	for cap := range 2 {
		loftLoop := lp.Profile0.Outer
		if cap == 1 {
			loftLoop = lp.Profile1.Outer
		}
		if len(source.prismCaps[cap]) != len(pp.Profile.Outer.Segments) ||
			len(source.loftCaps[cap]) != len(loftLoop.Segments) {
			return no, false, nil
		}
		_, withinCap, err := prismcells.RegionsWithinWorkCap(budget,
			pp.Profile, profileRecord{Outer: loftLoop})
		if err != nil || !withinCap {
			return no, false, err
		}
		scene, tags, _, err := prismcells.BuildSceneRegions(budget,
			[]prismcells.SceneProfile{{Outer: pp.Profile.Outer}},
			[]prismcells.SceneProfile{{Outer: loftLoop}},
			&prismcells.Reexpression{Identity: true})
		if err != nil {
			return no, false, err
		}
		// TaggedSegments scans the tags and both source-index arrays.
		_, err = budget.StepN(uint64(len(tags) + len(source.prismCaps[cap]) +
			len(source.loftCaps[cap])))
		if err != nil {
			return no, false, err
		}
		if !prismcells.TaggedSegments(tags, len(source.prismCaps[cap]),
			len(source.loftCaps[cap])) {
			return no, false, nil
		}
		var cells []*sketch.Profile
		if len(quarter) == 0 {
			cells, err = prismcells.CellProfiles(ctx, budget, scene)
			if err != nil {
				return no, false, err
			}
		} else {
			raw, readErr := prismcells.ProfilesContext(ctx, scene.Profiles)
			if readErr != nil {
				return no, false, readErr
			}
			quarter[0].cap = cap
			quarter[0].tags = tags
			witnesses, ok, certErr := scene.CertifyQuarterArcJunctions(
				ctx, raw, quarter[0])
			if certErr != nil || !ok {
				return no, false, certErr
			}
			split, lineage, resolved, splitErr :=
				prismcells.SplitRunsWithLineage(budget, raw)
			if splitErr != nil || !resolved {
				return no, false, splitErr
			}
			result.quarter[cap], ok, certErr = mapLoftQuarterWitnesses(
				ctx, budget, raw, split, lineage, tags, source, cap, witnesses)
			if certErr != nil || !ok {
				return no, false, certErr
			}
			cells = split
		}
		if len(cells) == 0 {
			return no, false, nil
		}
		matterPrism, matterLoft, resolved, err := prismcells.Classify(budget, tags, cells)
		if err != nil || !resolved {
			return no, false, err
		}
		coincident, resolved, err := prismcells.CoincidentEdges(budget, tags, cells)
		if err != nil || !resolved {
			return no, false, err
		}
		if len(coincident.Spans) != 0 {
			return no, false, nil
		}
		result.caps[cap] = make([]loftCapSourceCell, len(cells))
		for i, cell := range cells {
			if err := budget.Step(); err != nil {
				return no, false, err
			}
			owned := loftCapSourceCell{prism: matterPrism[i], loft: matterLoft[i],
				edges: make([]loftCapSourceEdge, len(cell.Outer))}
			for j, edge := range cell.Outer {
				if err := budget.Step(); err != nil {
					return no, false, err
				}
				origin, ok := tags[edge.Entity]
				if !ok || origin.Region != 0 || origin.Hole != -1 ||
					!loftCapSceneRangeFits(edge.TStart, edge.TEnd) ||
					(edge.Partial && !edge.TExact) {
					return no, false, nil
				}
				spans := source.prismCaps[cap]
				if origin.IsB {
					spans = source.loftCaps[cap]
				}
				if origin.Segment < 0 || origin.Segment >= len(spans) {
					return no, false, nil
				}
				owned.edges[j] = loftCapSourceEdge{
					span: spans[origin.Segment], tStart: edge.TStart,
					tEnd: edge.TEnd, reversed: edge.Reversed,
					partial:        edge.Partial,
					walkWithSource: edge.Reversed == origin.AuthoredReversed,
				}
				result.visits++
			}
			// RecordProfileLoops records and joins each edge, then checks closure.
			_, err := budget.StepN(uint64(len(cell.Outer))*2 + 1)
			if err != nil {
				return no, false, err
			}
			if _, err := prismcells.RecordArrangedProfileContext(ctx, cell); err != nil {
				if errors.Is(err, ErrUnrecordableProfile) {
					return no, false, nil
				}
				return no, false, err
			}
			shared[cap] = shared[cap] || owned.prism && owned.loft
			result.caps[cap][i] = owned
			result.visits++
		}
		if !shared[cap] {
			return no, false, nil
		}
		if len(quarter) == 1 {
			pairsOK, pairErr := quarterSourcePairs(
				budget, quarter[0], source, cap, result.quarter[cap])
			if pairErr != nil || !pairsOK {
				return no, false, pairErr
			}
			intervalsOK, intervalErr := quarterSourceIntervals(
				budget, source, cap, result.caps[cap])
			if intervalErr != nil || !intervalsOK {
				return no, false, intervalErr
			}
		}
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	return result, true, nil
}

func loftCapSceneRangeFits(start, end float64) bool {
	if proofbound.IsNonFinite(start) || proofbound.IsNonFinite(end) ||
		start < 0 || end > 1 || start >= end {
		return false
	}
	return loftVolumeRatFits(proofarith.FloatRat(start)) &&
		loftVolumeRatFits(proofarith.FloatRat(end))
}
