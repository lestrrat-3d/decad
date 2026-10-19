package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func tryLoftMirrorCylinderUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	var blank *Body
	var pairs []*Body
	var source *loftMirrorJoinRecord
	if pp, ok := a.n.Payload.(prismPayload); ok && loftcylinder.MirrorBlank(pp) {
		blank, pairs = a, []*Body{b}
	} else if pp, ok := b.n.Payload.(prismPayload); ok && loftcylinder.MirrorBlank(pp) {
		blank, pairs = b, []*Body{a}
	} else if fp, ok := a.n.Payload.(facetedPayload); ok &&
		fp.LoftMirrorCylinderJoin != nil && fp.Xform == r3.Identity() {
		blank = bodyHandle(fp.LoftMirrorCylinderJoin.Blank)
		source = fp.LoftMirrorCylinderJoin.Source
		pairs = append(bodyHandlesCopy(fp.LoftMirrorCylinderJoin.Pairs), b)
	} else if fp, ok := b.n.Payload.(facetedPayload); ok &&
		fp.LoftMirrorCylinderJoin != nil && fp.Xform == r3.Identity() {
		blank = bodyHandle(fp.LoftMirrorCylinderJoin.Blank)
		source = fp.LoftMirrorCylinderJoin.Source
		pairs = append(bodyHandlesCopy(fp.LoftMirrorCylinderJoin.Pairs), a)
	} else {
		return nil, false, nil
	}
	return tryLoftMirrorCylinderPairs(ctx, d, ref, blank, pairs, source)
}

func tryLoftMirrorCylinderPairs(ctx context.Context, d *Document, ref producerID,
	blank *Body, pairs []*Body, source *loftMirrorJoinRecord) (*Body, bool, error) {
	return loftMirrorCylinderPairs(ctx, d, ref, blank, pairs, source, orderedwork.MaxEachWorkers)
}

// loftMirrorCylinderPairs is tryLoftMirrorCylinderPairs with the cap on the
// goroutines that place the lower teeth. The placements are independent, so
// the cap changes the wall time and nothing else. It stays in root because
// the lower slab is certified by tryLoftCylinderUnionTeethWithUpperBridge,
// which verifies meshes through root Body.Tessellate.
func loftMirrorCylinderPairs(ctx context.Context, d *Document, ref producerID,
	blank *Body, pairs []*Body, source *loftMirrorJoinRecord, placeWorkers int) (*Body, bool, error) {
	pp, blankOK := blank.n.Payload.(prismPayload)
	if !blankOK || !loftcylinder.MirrorBlank(pp) || len(pairs) == 0 || len(pairs) > 32 {
		return nil, false, nil
	}
	first, firstOK := pairs[0].n.Payload.(facetedPayload)
	if !firstOK || first.LoftMirrorJoin == nil || first.Xform != r3.Identity() ||
		!first.ContactAudited || source != nil && source != first.LoftMirrorJoin {
		return nil, false, nil
	}
	record := first.LoftMirrorJoin
	if record.Lower == nil || record.Upper == nil || record.StationCount < 3 ||
		record.Lower.Doc != documentNode(d) || record.Upper.Doc != documentNode(d) {
		return nil, false, nil
	}
	lower, lowOK := record.Lower.Payload.(loftPayload)
	upper, upOK := record.Upper.Payload.(loftPayload)
	if !lowOK || !upOK || !lower.AuthenticatedReconstruction ||
		!upper.AuthenticatedReconstruction || lower.SourceIdentity == nil ||
		lower.SourceIdentity != upper.SourceIdentity ||
		lower.Xform != r3.Identity() || !loftMirrorAxialSource(lower) {
		return nil, false, nil
	}
	reflection, err := r3.Reflection(lower.Frame1)
	if err != nil {
		return nil, true, fmt.Errorf("%w: mirrored cylinder source has no reflection: %s", ErrUnsupported, err)
	}
	if upper.Xform != reflection {
		return nil, false, nil
	}
	z0, middle, z2 := pp.Z0, lower.Frame1.Origin().Z, pp.Z1
	if lower.Frame0.Origin().Z != z0 || !(z0 < middle && middle < z2) ||
		!loftcylinder.MirrorLevels(z0, middle, z2) {
		return nil, false, nil
	}
	// The first-stage source relation proves upper=reflection(lower). The
	// blank has the same circle and exact caps over both half intervals.
	half := pp
	half.Z1 = middle
	halfBlank, err := bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), half, freeform.NewFreeformWork()))
	if err != nil {
		return nil, true, err
	}
	checked := loftcylinder.CheckMirrorPairs(documentNode(d), bodyNodes(pairs), record, middle)
	workers := min(placeWorkers, orderedwork.EachWorkers(checked.Stop, 2))
	lowerTeeth, err := loftcylinder.PlaceTeeth(checked.Stop, workers, func(i int) (*Body, error) {
		if checked.Motions[i] == r3.Identity() {
			return bodyHandle(record.Lower), nil
		}
		return placePayload(ctx, lower, d, ref, checked.Motions[i])
	})
	if err != nil {
		return nil, true, err
	}
	if checked.Stop < len(pairs) {
		return nil, checked.Admit, checked.Err
	}
	lowerJoin, admitted, err := tryLoftCylinderUnionTeethWithUpperBridge(
		ctx, d, ref, halfBlank, lowerTeeth, checked.UpperBridges)
	if err != nil {
		return nil, true, err
	}
	if !admitted {
		return nil, true, fmt.Errorf("%w: mirrored cylinder's lower slab lacks a certified circular Join", ErrUnsupported)
	}
	result, err := bodyResult(loftcylinder.BuildMirrorJoin(ctx, envOf(documentNode(d), ref), bodyNode(blank),
		bodyNodes(pairs), record, bodyNode(lowerJoin), reflection, checked.BridgeBound, checked.BridgeVolume))
	return result, true, err
}

// tryLoftMirrorCylinderBoreCut cuts the exact centered cylinder from both
// reflected source halves. The lower half keeps the authenticated circular
// Join record, so its existing bore proof supplies the inner wall and annular
// caps. Reflection then transfers that proof to the upper half. It stays in
// root because the lower half's bore goes through the root
// tryLoftCylinderBoreCut dispatcher, whose placed branch certifies through
// root Body.Tessellate.
func tryLoftMirrorCylinderBoreCut(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, target, tool *Body) (*Body, bool, error) {
	if op != meshbool.OpCut {
		return nil, false, nil
	}
	joined, ok := target.n.Payload.(facetedPayload)
	if !ok || joined.LoftMirrorCylinderJoin == nil || !joined.ContactAudited ||
		joined.Xform != r3.Identity() {
		return nil, false, nil
	}
	record := joined.LoftMirrorCylinderJoin
	if record.Blank == nil || record.Source == nil || record.LowerHalf == nil ||
		len(record.Pairs) == 0 || record.LowerHalf.Doc != documentNode(d) || record.Blank.Doc != documentNode(d) ||
		record.BridgeBound < 0 || record.BridgeVolume < 0 ||
		proofbound.IsNonFinite(record.BridgeBound) || proofbound.IsNonFinite(record.BridgeVolume) {
		return nil, true, fmt.Errorf("%w: mirrored circular Join lost its bore sources", ErrUnsupported)
	}
	lower, ok := record.LowerHalf.Payload.(facetedPayload)
	if !ok || lower.LoftCylinderJoin == nil || !lower.ContactAudited ||
		lower.Xform != r3.Identity() {
		return nil, true, fmt.Errorf("%w: mirrored circular Join lost its lower-half proof", ErrUnsupported)
	}
	blank, ok := record.Blank.Payload.(prismPayload)
	if !ok || !loftcylinder.MirrorBlank(blank) ||
		lower.LoftCylinderJoin.Z0 != blank.Z0 ||
		!loftcylinder.MirrorLevels(blank.Z0, lower.LoftCylinderJoin.Z1, blank.Z1) {
		return nil, true, fmt.Errorf("%w: mirrored circular Join lost its exact axial levels", ErrUnsupported)
	}
	for cap, group := range record.CapGroups {
		level := blank.Z0
		if cap == 1 {
			level = blank.Z1
		}
		if group < 0 || group >= len(joined.Groups) ||
			!loftcylinder.AuditCap(ctx, joined.Verts, joined.Tris, joined.Src, record.CapOutlines[cap], group, level, cap == 0) {
			return nil, true, fmt.Errorf("%w: mirrored circular Join lost an outer cap disk", ErrUnsupported)
		}
	}
	lowerBore, matched, err := tryLoftCylinderBoreCut(ctx, op, d, ref, bodyHandle(record.LowerHalf), tool)
	if err != nil || !matched {
		return lowerBore, matched, err
	}
	prism, ok := tool.n.Payload.(prismPayload)
	if !ok {
		return nil, true, fmt.Errorf("%w: mirrored circular bore lost its cutter source", ErrUnsupported)
	}
	toolEnd := new(big.Rat).Add(proofarith.FloatRat(prism.Frame.Origin().Z), proofarith.FloatRat(prism.Z1))
	if toolEnd.Cmp(proofarith.FloatRat(blank.Z1)) <= 0 {
		return nil, true, fmt.Errorf("%w: mirrored circular bore tool ends inside the upper half", ErrUnsupported)
	}
	body, err := bodyResult(loftcylinder.BuildMirrorBore(ctx, envOf(documentNode(d), ref), record, bodyNode(lowerBore)))
	return body, true, err
}
