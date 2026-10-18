package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/prismplacement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

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
	if !axis.OK || !loftVolumeRatFits(axis.Shift) ||
		!loftVolumeRatFits(prismTop) || axis.Shift.Sign() <= 0 ||
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
	result := loftSourceSpanCaps{prism: prism, loft: loft,
		levels: [3]*big.Rat{big.NewRat(0, 1), new(big.Rat).Set(axis.Shift), prismTop}}
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
			if !loftSourceSameKind(lower[i], upper[i]) {
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
	case sectionrecord.ArcSeg:
		start, end = c.TStart, c.TEnd
		points = []sectionrecord.Point2{c.Center, c.Start, c.End}
	default:
		return no, false
	}
	if proofbound.IsNonFinite(start) || proofbound.IsNonFinite(end) ||
		start == end || start < 0 || start > 1 || end < 0 || end > 1 {
		return no, false
	}
	for _, point := range points {
		if proofbound.IsNonFinite(point.U) || proofbound.IsNonFinite(point.V) ||
			!loftVolumeRatFits(proofarith.FloatRat(point.U)) ||
			!loftVolumeRatFits(proofarith.FloatRat(point.V)) {
			return no, false
		}
	}
	return loftSourceCapSpan{body: body,
		capFace: provenance.Ref{Producer: body.n.Origin.Producer, Role: role},
		segment: index, curve: curve, tStart: start, tEnd: end,
		reversed: start > end}, true
}

func loftSourceSameKind(first, second sectionrecord.CurveSegment) bool {
	switch first.(type) {
	case sectionrecord.LineSeg:
		_, ok := second.(sectionrecord.LineSeg)
		return ok
	case sectionrecord.ArcSeg:
		_, ok := second.(sectionrecord.ArcSeg)
		return ok
	default:
		return false
	}
}
