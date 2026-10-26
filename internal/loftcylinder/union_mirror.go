package loftcylinder

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/verifiedmesh"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// MirrorUnion joins a mirrored-loft blank with its mirror pairs, or adds one
// more pair to an existing mirrored join, as one certified faceted body.
func MirrorUnion(ctx context.Context, op meshbool.OperationKind,
	env topo.Env, mesher verifiedmesh.Mesher, a, b *topo.Body) (*topo.Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	var blank *topo.Body
	var pairs []*topo.Body
	var source *facetedrecord.LoftMirrorJoinRecord
	if pp, ok := a.Payload.(prismrecord.Payload); ok && MirrorBlank(pp) {
		blank, pairs = a, []*topo.Body{b}
	} else if pp, ok := b.Payload.(prismrecord.Payload); ok && MirrorBlank(pp) {
		blank, pairs = b, []*topo.Body{a}
	} else if fp, ok := a.Payload.(facetedrecord.Payload); ok &&
		fp.LoftMirrorCylinderJoin != nil && fp.Xform == r3.Identity() {
		blank = fp.LoftMirrorCylinderJoin.Blank
		source = fp.LoftMirrorCylinderJoin.Source
		pairs = append(slices.Clone(fp.LoftMirrorCylinderJoin.Pairs), b)
	} else if fp, ok := b.Payload.(facetedrecord.Payload); ok &&
		fp.LoftMirrorCylinderJoin != nil && fp.Xform == r3.Identity() {
		blank = fp.LoftMirrorCylinderJoin.Blank
		source = fp.LoftMirrorCylinderJoin.Source
		pairs = append(slices.Clone(fp.LoftMirrorCylinderJoin.Pairs), a)
	} else {
		return nil, false, nil
	}
	return MirrorPairs(ctx, env, mesher, blank, pairs, source)
}

// MirrorPairs is the mirrored join of a blank and its pairs with every
// available worker placing the lower teeth.
func MirrorPairs(ctx context.Context, env topo.Env, mesher verifiedmesh.Mesher,
	blank *topo.Body, pairs []*topo.Body, source *facetedrecord.LoftMirrorJoinRecord) (*topo.Body, bool, error) {
	return MirrorPairsWithWorkers(ctx, env, mesher, blank, pairs, source, orderedwork.MaxEachWorkers)
}

// MirrorPairsWithWorkers is MirrorPairs with the cap on the
// goroutines that place the lower teeth. The placements are independent, so
// the cap changes the wall time and nothing else. The lower slab is certified
// by UnionTeethWithUpperBridge.
func MirrorPairsWithWorkers(ctx context.Context, env topo.Env, mesher verifiedmesh.Mesher,
	blank *topo.Body, pairs []*topo.Body, source *facetedrecord.LoftMirrorJoinRecord, placeWorkers int) (*topo.Body, bool, error) {
	pp, blankOK := blank.Payload.(prismrecord.Payload)
	if !blankOK || !MirrorBlank(pp) || len(pairs) == 0 || len(pairs) > 32 {
		return nil, false, nil
	}
	first, firstOK := pairs[0].Payload.(facetedrecord.Payload)
	if !firstOK || first.LoftMirrorJoin == nil || first.Xform != r3.Identity() ||
		!first.ContactAudited || source != nil && source != first.LoftMirrorJoin {
		return nil, false, nil
	}
	record := first.LoftMirrorJoin
	if record.Lower == nil || record.Upper == nil || record.StationCount < 3 ||
		record.Lower.Doc != env.Doc || record.Upper.Doc != env.Doc {
		return nil, false, nil
	}
	lower, lowOK := record.Lower.Payload.(loftrecord.Payload)
	upper, upOK := record.Upper.Payload.(loftrecord.Payload)
	if !lowOK || !upOK || !lower.AuthenticatedReconstruction ||
		!upper.AuthenticatedReconstruction || lower.SourceIdentity == nil ||
		lower.SourceIdentity != upper.SourceIdentity ||
		lower.Xform != r3.Identity() || !lower.MirrorAxialSource() {
		return nil, false, nil
	}
	reflection, err := r3.Reflection(lower.Frame1)
	if err != nil {
		return nil, true, fmt.Errorf("%w: mirrored cylinder source has no reflection: %s", decaderr.ErrUnsupported, err)
	}
	if upper.Xform != reflection {
		return nil, false, nil
	}
	z0, middle, z2 := pp.Z0, lower.Frame1.Origin().Z, pp.Z1
	if lower.Frame0.Origin().Z != z0 || !(z0 < middle && middle < z2) ||
		!MirrorLevels(z0, middle, z2) {
		return nil, false, nil
	}
	// The first-stage source relation proves upper=reflection(lower). The
	// blank has the same circle and exact caps over both half intervals.
	half := pp
	half.Z1 = middle
	halfBlank, err := prismrecord.Build(ctx, env, half, freeform.NewFreeformWork())
	if err != nil {
		return nil, true, err
	}
	checked := CheckMirrorPairs(env.Doc, pairs, record, middle)
	workers := min(placeWorkers, orderedwork.EachWorkers(checked.Stop, 2))
	lowerTeeth, err := PlaceTeeth(checked.Stop, workers, func(i int) (*topo.Body, error) {
		if checked.Motions[i] == r3.Identity() {
			return record.Lower, nil
		}
		return lower.Placed(ctx, env, checked.Motions[i])
	})
	if err != nil {
		return nil, true, err
	}
	if checked.Stop < len(pairs) {
		return nil, checked.Admit, checked.Err
	}
	lowerJoin, admitted, err := UnionTeethWithUpperBridge(
		ctx, env, mesher, halfBlank, lowerTeeth, checked.UpperBridges)
	if err != nil {
		return nil, true, err
	}
	if !admitted {
		return nil, true, fmt.Errorf("%w: mirrored cylinder's lower slab lacks a certified circular Join", decaderr.ErrUnsupported)
	}
	result, err := BuildMirrorJoin(ctx, env, blank,
		pairs, record, lowerJoin, reflection, checked.BridgeBound, checked.BridgeVolume)
	return result, true, err
}

// MirrorBoreCut cuts the exact centered cylinder from both
// reflected source halves. The lower half keeps the authenticated circular
// Join record, so its existing bore proof supplies the inner wall and annular
// caps. Reflection then transfers that proof to the upper half. The lower
// half's bore goes through JoinBoreCut, whose placed branch certifies through
// the mesher.
func MirrorBoreCut(ctx context.Context, op meshbool.OperationKind,
	env topo.Env, mesher verifiedmesh.Mesher, target, tool *topo.Body) (*topo.Body, bool, error) {
	if op != meshbool.OpCut {
		return nil, false, nil
	}
	joined, ok := target.Payload.(facetedrecord.Payload)
	if !ok || joined.LoftMirrorCylinderJoin == nil || !joined.ContactAudited ||
		joined.Xform != r3.Identity() {
		return nil, false, nil
	}
	record := joined.LoftMirrorCylinderJoin
	if record.Blank == nil || record.Source == nil || record.LowerHalf == nil ||
		len(record.Pairs) == 0 || record.LowerHalf.Doc != env.Doc || record.Blank.Doc != env.Doc ||
		record.BridgeBound < 0 || record.BridgeVolume < 0 ||
		proofbound.IsNonFinite(record.BridgeBound) || proofbound.IsNonFinite(record.BridgeVolume) {
		return nil, true, fmt.Errorf("%w: mirrored circular Join lost its bore sources", decaderr.ErrUnsupported)
	}
	lower, ok := record.LowerHalf.Payload.(facetedrecord.Payload)
	if !ok || lower.LoftCylinderJoin == nil || !lower.ContactAudited ||
		lower.Xform != r3.Identity() {
		return nil, true, fmt.Errorf("%w: mirrored circular Join lost its lower-half proof", decaderr.ErrUnsupported)
	}
	blank, ok := record.Blank.Payload.(prismrecord.Payload)
	if !ok || !MirrorBlank(blank) ||
		lower.LoftCylinderJoin.Z0 != blank.Z0 ||
		!MirrorLevels(blank.Z0, lower.LoftCylinderJoin.Z1, blank.Z1) {
		return nil, true, fmt.Errorf("%w: mirrored circular Join lost its exact axial levels", decaderr.ErrUnsupported)
	}
	for endCap, group := range record.CapGroups {
		level := blank.Z0
		if endCap == 1 {
			level = blank.Z1
		}
		if group < 0 || group >= len(joined.Groups) ||
			!AuditCap(ctx, joined.Verts, joined.Tris, joined.Src, record.CapOutlines[endCap], group, level, endCap == 0) {
			return nil, true, fmt.Errorf("%w: mirrored circular Join lost an outer cap disk", decaderr.ErrUnsupported)
		}
	}
	lowerBore, matched, err := JoinBoreCut(ctx, op, env, mesher, record.LowerHalf, tool)
	if err != nil || !matched {
		return lowerBore, matched, err
	}
	prism, ok := tool.Payload.(prismrecord.Payload)
	if !ok {
		return nil, true, fmt.Errorf("%w: mirrored circular bore lost its cutter source", decaderr.ErrUnsupported)
	}
	toolEnd := new(big.Rat).Add(proofarith.FloatRat(prism.Frame.Origin().Z), proofarith.FloatRat(prism.Z1))
	if toolEnd.Cmp(proofarith.FloatRat(blank.Z1)) <= 0 {
		return nil, true, fmt.Errorf("%w: mirrored circular bore tool ends inside the upper half", decaderr.ErrUnsupported)
	}
	body, err := BuildMirrorBore(ctx, env, record, lowerBore)
	return body, true, err
}
