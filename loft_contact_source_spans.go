package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
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
					!loftcontact.SceneRangeFits(edge.TStart, edge.TEnd) ||
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

// loftSourceCapSpan borrows one authenticated producer segment for this call.
// wallFaces names the original Loft wall cells paired to it. Neither this
// record nor its parent claims a selected Boolean output patch.
type loftSourceCapSpan struct {
	body      *Body
	capFace   provenance.Ref
	segment   int
	curve     sectionrecord.CurveSegment
	tStart    float64
	tEnd      float64
	reversed  bool
	wallFaces []provenance.Ref
}

// loftSourceSpanCaps is the source-only stage before cap intersection,
// ownership, two-way boundary coverage, and occupied-volume proofs. Consumers
// must refuse a curved Boolean until those later stages certify output patches.
type loftSourceSpanCaps struct {
	prism, loft *Body
	levels      [3]*big.Rat
	prismCaps   [2][]loftSourceCapSpan
	loftCaps    [2][]loftSourceCapSpan
	visits      uint64
}

// certifyLoftSourceSpanCaps reads original, unplaced producer records. budget
// must be the same 4096-step source wall used by the private consumer.
func certifyLoftSourceSpanCaps(ctx context.Context, budget *proofbound.WorkBudget,
	prism, loft *Body) (loftSourceSpanCaps, bool, error) {
	var no loftSourceSpanCaps
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if budget == nil || prism == nil || loft == nil || prism == loft ||
		prism.n.Doc == nil || loft.n.Doc == nil ||
		prism.n.Owner != prism || loft.n.Owner != loft ||
		prism.n.Origin.Role != provenance.RoleBody ||
		loft.n.Origin.Role != provenance.RoleBody ||
		(prism.n.Doc == loft.n.Doc &&
			prism.n.Origin.Producer == loft.n.Origin.Producer) {
		return no, false, nil
	}
	pp, prismOK := prism.n.Payload.(prismPayload)
	lp, loftOK := loft.n.Payload.(loftPayload)
	if !prismOK || !loftOK || !lp.AuthenticatedReconstruction ||
		lp.SourceIdentity == nil || lp.SourceIdentity.Origin != loft.n.Origin.Producer ||
		lp.SurfaceResult || lp.ThroughBoreCut || lp.AnalyticBore ||
		lp.ConstructionDelta != 0 || len(lp.BlendSegs) != 0 ||
		len(lp.Profile0.Holes) != 0 || len(lp.Profile1.Holes) != 0 ||
		pp.SurfaceResult || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		len(pp.BlendSegs) != 0 || len(pp.Profile.Holes) != 0 ||
		pp.Xform != r3.Identity() || lp.Xform != r3.Identity() ||
		pp.Z0 != 0 || pp.Z1 <= 0 || lp.Frame0 != pp.Frame ||
		lp.Plane0 != (sectionrecord.PlaneRecord{Origin: lp.Frame0.Origin(),
			U: lp.Frame0.U(), V: lp.Frame0.V()}) ||
		lp.Plane1 != (sectionrecord.PlaneRecord{Origin: lp.Frame1.Origin(),
			U: lp.Frame1.U(), V: lp.Frame1.V()}) ||
		lp.Frame0.U() != lp.Frame1.U() || lp.Frame0.V() != lp.Frame1.V() ||
		len(lp.Alignment) > 1 {
		return no, false, nil
	}
	for _, offset := range lp.Alignment {
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		if offset != 0 {
			return no, false, nil
		}
	}
	axis := prismplacement.SharedAxisOf(
		prismplacement.Operand{Frame: pp.Frame, Xform: pp.Xform},
		prismplacement.Operand{Frame: lp.Frame1, Xform: lp.Xform})
	prismTop := proofarith.FloatRat(pp.Z1)
	if !axis.OK || !loftcontact.VolumeRatFits(axis.Shift) ||
		!loftcontact.VolumeRatFits(prismTop) || axis.Shift.Sign() <= 0 ||
		axis.Shift.Cmp(prismTop) >= 0 {
		return no, false, nil
	}
	prismSegments := pp.Profile.Outer.Segments
	lower, upper := lp.Profile0.Outer.Segments, lp.Profile1.Outer.Segments
	if len(prismSegments) == 0 || len(prismSegments) > 512 ||
		len(lower) == 0 || len(lower) > 512 || len(lower) != len(upper) ||
		len(lp.Segment) != 1 || len(lp.Segment[0]) == 0 ||
		len(lp.Segment[0]) > 512 || lp.Walls != 2*len(lp.Segment[0]) ||
		len(lp.Cell) != lp.Walls || len(lp.Side) != lp.Walls ||
		lp.Walls > len(lp.Tris) || len(lp.Tris) > 512 || len(lp.Verts) > 512 {
		return no, false, nil
	}
	fitVisits, fitOK, err := loftcontact.FitPreflight(budget, prismSegments, lower, upper)
	if err != nil || !fitOK {
		return no, false, err
	}
	result := loftSourceSpanCaps{prism: prism, loft: loft,
		levels: [3]*big.Rat{big.NewRat(0, 1), new(big.Rat).Set(axis.Shift), prismTop},
		visits: fitVisits}
	for cap := range 2 {
		prismRole := provenance.RoleCapStart
		loftRole := provenance.RoleCapStart
		if cap == 1 {
			prismRole, loftRole = provenance.RoleCapEnd, provenance.RoleCapEnd
		}
		result.prismCaps[cap] = make([]loftSourceCapSpan, len(prismSegments))
		result.loftCaps[cap] = make([]loftSourceCapSpan, len(lower))
		loftSegments := lower
		if cap == 1 {
			loftSegments = upper
		}
		for i, curve := range prismSegments {
			if err := budget.Step(); err != nil {
				return no, false, err
			}
			span, ok := loftSourceCapSpanOf(prism, prismRole, i, curve)
			if !ok {
				return no, false, nil
			}
			result.prismCaps[cap][i] = span
			result.visits++
		}
		for i, curve := range loftSegments {
			if err := budget.Step(); err != nil {
				return no, false, err
			}
			if !loftcontact.SameKind(lower[i], upper[i]) {
				return no, false, nil
			}
			span, ok := loftSourceCapSpanOf(loft, loftRole, i, curve)
			if !ok {
				return no, false, nil
			}
			result.loftCaps[cap][i] = span
			result.visits++
		}
	}
	seen := make([][2]uint8, len(lp.Segment[0]))
	for triangle := range lp.Walls {
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		cell := lp.Cell[triangle]
		side := lp.Side[triangle]
		if cell[0] != 0 || cell[1] < 0 || cell[1] >= len(seen) ||
			side > 1 || seen[cell[1]][side] != 0 {
			return no, false, nil
		}
		seen[cell[1]][side]++
		result.visits++
	}
	for cell, source := range lp.Segment[0] {
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		if source < 0 || source >= len(lower) || seen[cell] != [2]uint8{1, 1} {
			return no, false, nil
		}
		for cap := range 2 {
			role := fmt.Sprintf("side(0,%d,%d)", cell, cap)
			span := &result.loftCaps[cap][source]
			span.wallFaces = append(span.wallFaces, provenance.Ref{
				Producer: lp.SourceIdentity.Origin, Role: role})
		}
		result.visits++
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	return result, true, nil
}

func loftSourceCapSpanOf(body *Body, role string, index int,
	curve sectionrecord.CurveSegment) (loftSourceCapSpan, bool) {
	var no loftSourceCapSpan
	var start, end float64
	var points []sectionrecord.Point2
	switch c := curve.(type) {
	case sectionrecord.LineSeg:
		start, end = c.TStart, c.TEnd
		points = []sectionrecord.Point2{c.Start, c.End}
	case sectionrecord.CircleSeg:
		start, end = c.TStart, c.TEnd
		radius, err := c.Radius.In(units.Millimeter)
		if err != nil || proofbound.IsNonFinite(radius) || radius <= 0 ||
			!loftcontact.VolumeRatFits(proofarith.FloatRat(radius)) ||
			!((start == 0 && end == 1) || (start == 1 && end == 0)) ||
			c.CCW != (start < end) {
			return no, false
		}
		points = []sectionrecord.Point2{c.Center}
	case sectionrecord.ArcSeg:
		start, end = c.TStart, c.TEnd
		points = []sectionrecord.Point2{c.Center, c.Start, c.End}
	case sectionrecord.FitSplineSeg:
		if len(c.Fit) < 2 {
			return no, false
		}
		start, end = c.TStart, c.TEnd
		points = c.Fit
	default:
		return no, false
	}
	if proofbound.IsNonFinite(start) || proofbound.IsNonFinite(end) ||
		start == end || start < 0 || start > 1 || end < 0 || end > 1 {
		return no, false
	}
	for _, point := range points {
		if proofbound.IsNonFinite(point.U) || proofbound.IsNonFinite(point.V) ||
			!loftcontact.VolumeRatFits(proofarith.FloatRat(point.U)) ||
			!loftcontact.VolumeRatFits(proofarith.FloatRat(point.V)) {
			return no, false
		}
	}
	return loftSourceCapSpan{body: body,
		capFace: provenance.Ref{Producer: body.n.Origin.Producer, Role: role},
		segment: index, curve: curve, tStart: start, tEnd: end,
		reversed: start > end}, true
}
