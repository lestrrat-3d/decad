package revolveplan

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/sketch"
)

// PrepareCommonAxis reduces two common-axis full-Revolve meridians through
// their recorded profiles. A gate miss returns admitted=false and no error.
// An empty Cut or Intersect returns admitted=true, empty=true for root's
// existing public BooleanError mapping. A prepared payload is not built here.
func PrepareCommonAxis(ctx context.Context, op meshbool.OperationKind,
	a, b *topo.Body) (Payload, bool, bool, error) {
	ra, aok := a.Payload.(Payload)
	rb, bok := b.Payload.(Payload)
	if !aok || !bok || !CommonAxisPair(ra, rb) {
		return Payload{}, false, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, p := range []momentinput.Profile{ra.Profile, rb.Profile} {
		if len(p.Holes) != 0 || len(p.Outer.Segments) == 0 {
			return Payload{}, false, false, nil
		}
		for _, seg := range p.Outer.Segments {
			if err := budget.Step(); err != nil {
				return Payload{}, false, false, err
			}
			switch seg.(type) {
			case sectionrecord.LineSeg, sectionrecord.ArcSeg, sectionrecord.CircleSeg:
			default:
				return Payload{}, false, false, nil
			}
		}
	}
	if _, within, err := prismcells.RegionsWithinWorkCap(budget, ra.Profile, rb.Profile); err != nil {
		return Payload{}, false, false, err
	} else if !within {
		return Payload{}, false, false, nil
	}
	reexpress, err := prismcells.NewReexpression(ra.Meridian().PlacementOperand(), rb.Meridian().PlacementOperand())
	if err != nil {
		return Payload{}, false, false, err
	}
	if !reexpress.Identity {
		return Payload{}, false, false, nil
	}
	s, tags, sceneDelta, err := prismcells.BuildScene(budget, ra.Profile, rb.Profile, reexpress)
	if err != nil {
		return Payload{}, false, false, err
	}
	// Provenance by segment index requires a distinct scene entity for each
	// source segment. The scene may deduplicate segments within an operand.
	if !prismcells.TaggedSegments(tags, len(ra.Profile.Outer.Segments), len(rb.Profile.Outer.Segments)) {
		return Payload{}, false, false, nil
	}
	profiles, err := prismcells.CellProfiles(ctx, budget, s)
	if err != nil {
		return Payload{}, false, false, err
	}
	if len(profiles) == 0 {
		return Payload{}, false, false, nil
	}
	coincident, ok, err := prismcells.CoincidentEdges(budget, tags, profiles)
	if err != nil || !ok {
		return Payload{}, false, false, err
	}
	if exact, err := coincident.ExactCarriers(budget); err != nil || !exact {
		return Payload{}, false, false, err
	}
	matterA, matterB, resolved, err := prismcells.Classify(budget, tags, profiles)
	if err != nil || !resolved {
		return Payload{}, false, false, err
	}
	selected, err := prismcells.Select(budget, profiles, matterA, matterB, commonAxisKeep(op))
	if err != nil {
		return Payload{}, false, false, err
	}
	if len(selected) == 0 {
		if op == meshbool.OpIntersect || op == meshbool.OpCut {
			return Payload{}, true, true, nil
		}
		return Payload{}, false, false, nil
	}
	if ok, err := sceneDelta.ChargeCrossings(budget, tags, profiles, 0, 0, reexpress.Delta); err != nil || !ok {
		return Payload{}, false, false, err
	}
	if ok, err := sceneDelta.SharedSpansBounded(budget, selected); err != nil || !ok {
		return Payload{}, false, false, err
	}
	profile, cutDelta, chain, resolved, err := prismcells.MergeWithChain(budget, selected, op.String())
	if err != nil || !resolved {
		return Payload{}, false, false, err
	}
	if err := prismcells.AuditMergeSection(budget, ra.Meridian(), profile); err != nil {
		return Payload{}, false, false, err
	}
	sources, ok, err := revolveBooleanSources(ctx, a, b, ra, rb, tags, coincident, chain)
	if err != nil || !ok {
		return Payload{}, false, false, err
	}
	line := revolveaxis.Line2{
		AU: ra.Ax.AU, AV: ra.Ax.AV, AUBound: ra.Ax.AUBound, AVBound: ra.Ax.AVBound,
		DU: ra.Ax.DU, DV: ra.Ax.DV, DUBound: ra.Ax.DUBound, DVBound: ra.Ax.DVBound,
	}
	ax, side, err := ResolveAxisSide(ctx, profile, line, freeform.NewFreeformWork())
	if err != nil || side != 1 {
		return Payload{}, false, false, err
	}
	result := ra
	result.Profile = profile
	result.Ax = ax
	result.SectionDelta = sceneDelta.Merged(0, 0, reexpress.Delta, cutDelta)
	result.SectionWhole = false
	result.RadialProof = ax.RadialProof
	result.SourceSideOrigins = [][][]provenance.Ref{sources}
	result.BlendSegs = nil
	result.BlendKind = ""
	return result, true, false, nil
}

func commonAxisKeep(op meshbool.OperationKind) func(bool, bool) bool {
	switch op {
	case meshbool.OpUnion:
		return func(a, b bool) bool { return a || b }
	case meshbool.OpCut:
		return func(a, b bool) bool { return a && !b }
	default:
		return func(a, b bool) bool { return a && b }
	}
}

func revolveBooleanSources(ctx context.Context, a, b *topo.Body, ra, rb Payload,
	tags map[sketch.Entity]prismcells.Origin, coincident prismcells.CoincidentReading,
	chain []sketch.BoundaryEdge) ([][]provenance.Ref, bool, error) {
	sources := make([][]provenance.Ref, len(chain))
	for i, edge := range chain {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		origin, ok := tags[edge.Entity]
		if !ok {
			return nil, false, nil
		}
		origins := []prismcells.Origin{origin}
		if shared, ok := coincident.Lookup(edge.Entity, edge.TStart, edge.TEnd); ok {
			partner, ok := tags[shared.Partner]
			if !ok || partner.IsB == origin.IsB {
				return nil, false, nil
			}
			origins = append(origins, partner)
		}
		for _, sourceOrigin := range origins {
			refs, ok, err := revolveBooleanSourceFace(a, b, ra, rb, sourceOrigin)
			if err != nil || !ok {
				return nil, false, err
			}
			sources[i] = append(sources[i], refs...)
		}
	}
	return sources, true, nil
}

func revolveBooleanSourceFace(a, b *topo.Body, ra, rb Payload,
	origin prismcells.Origin) ([]provenance.Ref, bool, error) {
	if origin.Region != 0 || origin.Hole != -1 {
		return nil, false, nil
	}
	body, source := a, ra
	if origin.IsB {
		body, source = b, rb
	}
	if origin.Segment < 0 || origin.Segment >= len(source.Profile.Outer.Segments) {
		return nil, false, nil
	}
	seg := source.Profile.Outer.Segments[origin.Segment]
	walk, err := boundarywalk.WalkOf(seg, nil)
	if err != nil {
		return nil, false, err
	}
	if source.Ax.IsAxis(source.Ax.Walk(walk)) {
		return nil, true, nil
	}
	ref := provenance.Ref{Producer: body.Origin.Producer, Role: fmt.Sprintf("side(%d,%d)", 0, origin.Segment)}
	for _, face := range topobuild.Faces(body) {
		if slices.Contains(face.Origins, ref) {
			return slices.Clone(face.Origins), true, nil
		}
	}
	return nil, false, nil
}
