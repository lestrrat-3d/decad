package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// tryLoftMirrorCylinderBoreCut cuts the exact centered cylinder from both
// reflected source halves. The lower half keeps the authenticated circular
// Join record, so its existing bore proof supplies the inner wall and annular
// caps. Reflection then transfers that proof to the upper half.
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
	if !ok || !loftMirrorCylinderBlank(blank) ||
		lower.LoftCylinderJoin.Z0 != blank.Z0 ||
		!loftMirrorCylinderLevels(blank.Z0, lower.LoftCylinderJoin.Z1, blank.Z1) {
		return nil, true, fmt.Errorf("%w: mirrored circular Join lost its exact axial levels", ErrUnsupported)
	}
	source := &joinedMesh{Verts: joined.Verts, Tris: joined.Tris, Src: joined.Src}
	for cap, group := range record.CapGroups {
		level := blank.Z0
		if cap == 1 {
			level = blank.Z1
		}
		if group < 0 || group >= len(joined.Groups) ||
			!loftCylinderAuditCap(ctx, source, record.CapOutlines[cap], group, level, cap == 0) {
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
	body, err := buildLoftMirrorCylinderBore(ctx, d, ref, record, lowerBore)
	return body, true, err
}

func buildLoftMirrorCylinderBore(ctx context.Context, d *Document, ref producerID,
	record *loftMirrorCylinderJoinRecord, lowerBody *Body) (*Body, error) {
	lower, ok := lowerBody.n.Payload.(facetedPayload)
	if !ok || lower.LoftCylinderBore == nil || !lower.ContactAudited ||
		lower.Xform != r3.Identity() || record.Source.Lower == nil {
		return nil, fmt.Errorf("%w: mirrored circular bore lost its lower annulus", ErrUnsupported)
	}
	loft, ok := record.Source.Lower.Payload.(loftPayload)
	if !ok || !loft.AuthenticatedReconstruction {
		return nil, fmt.Errorf("%w: mirrored circular bore lost its Loft source", ErrUnsupported)
	}
	reflection, err := r3.Reflection(loft.Frame1)
	if err != nil {
		return nil, fmt.Errorf("%w: mirrored circular bore has no source reflection: %s", ErrUnsupported, err)
	}
	upperBody, err := placePayload(ctx, lower, d, ref, reflection)
	if err != nil {
		return nil, err
	}
	upper, ok := upperBody.n.Payload.(facetedPayload)
	if !ok || !upper.ContactAudited || len(lower.Verts) != len(upper.Verts) ||
		len(lower.Tris) != len(upper.Tris) || len(lower.Src) != len(upper.Src) {
		return nil, fmt.Errorf("%w: reflected circular bore lacks a matching held shell", ErrUnsupported)
	}
	shell, err := loftcylinder.AssembleMirrorBore(ctx, lower, upper,
		record.BridgeBound, record.BridgeVolume)
	if err != nil {
		return nil, err
	}
	fullBlank, ok := record.Blank.Payload.(prismPayload)
	if !ok || !loftMirrorCylinderBlank(fullBlank) {
		return nil, fmt.Errorf("%w: reflected circular bore lost its exact blank", ErrUnsupported)
	}
	mesh := &joinedMesh{Verts: shell.Payload.Verts, Tris: shell.Payload.Tris, Src: shell.Payload.Src}
	for cap, group := range shell.CapGroups {
		z := fullBlank.Z0
		if cap == 1 {
			z = fullBlank.Z1
		}
		if !loftCylinderAuditAnnulus(ctx, mesh, shell.Outer[cap], shell.Inner[cap], group, z, cap == 0) {
			return nil, fmt.Errorf("%w: reflected circular bore cap does not tile its annulus", ErrUnsupported)
		}
	}
	fp := shell.Payload
	fp.LoftMirrorCylinderBore = &loftMirrorCylinderBoreRecord{
		Join: record, LowerBore: bodyNode(lowerBody), CapGroups: shell.CapGroups,
		Outer: shell.Outer, Inner: shell.Inner, WallGroups: shell.WallGroups,
	}
	if err := loftMirrorCylinderCapOrigins(&fp, bodyHandle(record.Blank), record.Pairs, shell.CapGroups); err != nil {
		return nil, err
	}
	if err := loftCylinderTagCaps(&fp, bodyHandle(record.Blank), shell.CapGroups,
		[2]float64{fullBlank.Z0, fullBlank.Z1}, ref); err != nil {
		return nil, err
	}
	loftCylinderTagBridgedTop(&fp, shell.CapGroups[1], []float64{record.BridgeBound})
	return bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), fp))
}
