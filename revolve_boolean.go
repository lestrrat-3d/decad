package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/sketch"
)

// tryCommonAxisRevolveBoolean reduces a pair of full revolutions about one
// recorded axis to a Sketch boolean of their meridians. Every retained 2D
// edge traces to a source wall; the analytic revolve builder then constructs
// the 3D result. A missed gate leaves the mesh path unchanged.
func tryCommonAxisRevolveBoolean(ctx context.Context, op meshbool.OperationKind, d *Document,
	ref producerID, a, b *Body) (*Body, bool, error) {
	ra, aok := a.n.Payload.(revolvePayload)
	rb, bok := b.n.Payload.(revolvePayload)
	if !aok || !bok || !commonAxisRevolvePair(ra, rb) {
		return nil, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, p := range []profileRecord{ra.Profile, rb.Profile} {
		if len(p.Holes) != 0 || len(p.Outer.Segments) == 0 {
			return nil, false, nil
		}
		for _, seg := range p.Outer.Segments {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			if _, ok := seg.(lineSeg); !ok {
				return nil, false, nil
			}
		}
	}
	if _, within, err := prismcells.RegionsWithinWorkCap(budget, ra.Profile, rb.Profile); err != nil {
		return nil, false, err
	} else if !within {
		return nil, false, nil
	}
	reexpress, err := prismcells.NewReexpression(ra.Meridian().PlacementOperand(), rb.Meridian().PlacementOperand())
	if err != nil {
		return nil, false, err
	}
	if !reexpress.Identity {
		return nil, false, nil
	}
	s, tags, sceneDelta, err := prismcells.BuildScene(budget, ra.Profile, rb.Profile, reexpress)
	if err != nil {
		return nil, false, err
	}
	// Provenance by segment index requires a distinct scene entity for each
	// source segment. The scene may deduplicate segments within an operand.
	if !revolveBooleanTaggedSegments(tags, len(ra.Profile.Outer.Segments), len(rb.Profile.Outer.Segments)) {
		return nil, false, nil
	}
	profiles, err := prismcells.CellProfiles(ctx, budget, s)
	if err != nil {
		return nil, false, err
	}
	if len(profiles) == 0 {
		return nil, false, nil
	}
	coincident, ok, err := prismcells.CoincidentEdges(budget, tags, profiles)
	if err != nil || !ok {
		return nil, false, err
	}
	if exact, err := coincident.ExactLineCarriers(budget); err != nil || !exact {
		return nil, false, err
	}
	matterA, matterB, resolved, err := prismcells.Classify(budget, tags, profiles)
	if err != nil || !resolved {
		return nil, false, err
	}
	selected, err := prismcells.Select(budget, profiles, matterA, matterB, commonAxisKeep(op))
	if err != nil {
		return nil, false, err
	}
	if len(selected) == 0 {
		if op == meshbool.OpIntersect || op == meshbool.OpCut {
			return nil, true, asBooleanError(op, meshbool.ExpectedBoolean(meshbool.BooleanExpectedEmpty,
				fmt.Errorf("%w: the common-axis result contains no volume", ErrBooleanFailed)))
		}
		return nil, false, nil
	}
	if ok, err := sceneDelta.ChargeCrossings(budget, tags, profiles, 0, 0, reexpress.Delta); err != nil || !ok {
		return nil, false, err
	}
	if ok, err := sceneDelta.SharedSpansBounded(budget, selected); err != nil || !ok {
		return nil, false, err
	}
	profile, cutDelta, chain, resolved, err := prismcells.MergeWithChain(budget, selected, op.String())
	if err != nil || !resolved {
		return nil, false, err
	}
	if err := prismcells.AuditMergeSection(budget, ra.Meridian(), profile); err != nil {
		return nil, false, err
	}
	sources, ok, err := revolveBooleanSources(ctx, a, b, ra, rb, tags, coincident, chain)
	if err != nil || !ok {
		return nil, false, err
	}
	line := revolveaxis.Line2{
		AU: ra.Ax.AU, AV: ra.Ax.AV, AUBound: ra.Ax.AUBound, AVBound: ra.Ax.AVBound,
		DU: ra.Ax.DU, DV: ra.Ax.DV, DUBound: ra.Ax.DUBound, DVBound: ra.Ax.DVBound,
	}
	ax, side, err := resolveAxisSide(ctx, profile, line, freeform.NewFreeformWork())
	if err != nil || side != 1 {
		return nil, false, err
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
	body, err := bodyResult(revolveplan.Build(ctx, envOf(documentNode(d), ref), result, freeform.NewFreeformWork()))
	if err != nil {
		return nil, false, err
	}
	return body, true, nil
}

func commonAxisRevolvePair(a, b revolvePayload) bool {
	if !a.Full || !b.Full || a.SurfaceResult || b.SurfaceResult ||
		a.SectionDelta != 0 || b.SectionDelta != 0 || a.SectionWhole || b.SectionWhole ||
		a.Frame != b.Frame || a.Xform != b.Xform || a.Phi0 != b.Phi0 || a.Phi1 != b.Phi1 {
		return false
	}
	return a.Ax.AU == b.Ax.AU && a.Ax.AV == b.Ax.AV &&
		a.Ax.DU == b.Ax.DU && a.Ax.DV == b.Ax.DV &&
		a.Ax.AUBound == b.Ax.AUBound && a.Ax.AVBound == b.Ax.AVBound &&
		a.Ax.DUBound == b.Ax.DUBound && a.Ax.DVBound == b.Ax.DVBound
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

func revolveBooleanTaggedSegments(tags map[sketch.Entity]prismcells.Origin, countA, countB int) bool {
	seenA, seenB := make([]bool, countA), make([]bool, countB)
	for _, origin := range tags {
		if origin.Region != 0 || origin.Hole != -1 {
			return false
		}
		seen := seenA
		if origin.IsB {
			seen = seenB
		}
		if origin.Segment < 0 || origin.Segment >= len(seen) || seen[origin.Segment] {
			return false
		}
		seen[origin.Segment] = true
	}
	for _, seen := range append(seenA, seenB...) {
		if !seen {
			return false
		}
	}
	return true
}

func revolveBooleanSources(ctx context.Context, a, b *Body, ra, rb revolvePayload,
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

func revolveBooleanSourceFace(a, b *Body, ra, rb revolvePayload,
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
	ref := provenance.Ref{Producer: body.n.Origin.Producer, Role: prismWallRole(0, origin.Segment)}
	for _, face := range body.Faces() {
		if slices.Contains(face.n.Origins, ref) {
			return slices.Clone(face.n.Origins), true, nil
		}
	}
	return nil, false, nil
}
