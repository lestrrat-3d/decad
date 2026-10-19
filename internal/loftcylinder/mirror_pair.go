package loftcylinder

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// MirrorBlank reports whether pp is the blank a mirrored Loft/cylinder Join
// accepts: an unplaced, solid, single-circle prism centered on the world Z
// axis in the XY frame, with no recorded cap or section deltas and a finite
// positive radius.
func MirrorBlank(pp prismrecord.Payload) bool {
	if pp.SurfaceResult || pp.Xform != r3.Identity() || pp.Z0Delta != 0 ||
		pp.Z1Delta != 0 || pp.SectionDelta != 0 || pp.Z1 <= pp.Z0 ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		pp.Frame.Origin() != (r3.Vec{}) || pp.Frame.U() != (r3.Vec{X: 1}) ||
		pp.Frame.V() != (r3.Vec{Y: 1}) {
		return false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	return ok && circle.CCW && circle.Center == (sectionrecord.Point2{}) &&
		circle.Radius.Base() > 0 && !proofbound.IsNonFinite(circle.Radius.Base())
}

// MirrorPairMotion returns the placement of a mirrored pair over source: the
// identity for the source pair itself, or the recorded motion of a rigid copy
// of it whose held mesh carries exactly that motion and whose motion is an
// axial or near-axial placement. It reports false for any other pair.
func MirrorPairMotion(fp facetedrecord.Payload,
	source *facetedrecord.LoftMirrorJoinRecord) (r3.Transform, bool) {
	if fp.LoftMirrorJoin == source && fp.Xform == r3.Identity() {
		return r3.Identity(), true
	}
	if fp.LoftMirrorCopy == nil || fp.LoftMirrorCopy.Source != source ||
		fp.Xform != fp.LoftMirrorCopy.Motion ||
		!AxialPlacement(fp.LoftMirrorCopy.Motion) &&
			!loftmesh.NearAxialPlacement(fp.LoftMirrorCopy.Motion) {
		return r3.Transform{}, false
	}
	return fp.LoftMirrorCopy.Motion, true
}

// MirrorUpperBridge returns the upper source shift and its swept-volume charge
// for a pair placed by motion about the plane z=middle. For a lower source L
// reflected about that plane, the actual upper copy is T(F(L)). The reflected
// lower-half Join represents F(T(L)); the pair's certified area bounds the
// upper source area along the constant translation between them
// (loftmesh.MirrorUpperBridgeShift). Held placement roundoff remains in the
// lower Join. It reports false when either reading has no bound.
func MirrorUpperBridge(pair *topo.Body, motion r3.Transform, middle float64) (float64, float64, bool) {
	shift, ok := loftmesh.MirrorUpperBridgeShift(motion, middle)
	if !ok {
		return 0, 0, false
	}
	swept, ok := loftmesh.MirrorUpperBridgeSwept(shift, pair.Area.Value.Base(), pair.Area.Bound.Base())
	if !ok {
		return 0, 0, false
	}
	return shift, swept, true
}

// MirrorChamferUpperShifts recomputes each source tooth's exact upper-tooth
// translation from the authenticated pair placements of record's Join, in the
// order of source.OriginalTeeth (ChamferBottomCap's reflectedShift). The
// Join's accumulated bridge bound and swept volume must still contain them.
// It returns nil when no pair is bridged.
func MirrorChamferUpperShifts(record *facetedrecord.LoftMirrorCylinderBoreRecord,
	source ChamferSource) ([]*big.Rat, error) {
	if record == nil || record.Join == nil || record.Join.Source == nil ||
		record.Join.Source.Lower == nil || len(record.Join.Pairs) == 0 ||
		len(record.Join.Pairs) != len(source.OriginalTeeth) {
		return nil, fmt.Errorf("%w: mirrored cap chamfer lost its upper tooth source", decaderr.ErrUnsupported)
	}
	loft, ok := record.Join.Source.Lower.Payload.(loftrecord.Payload)
	if !ok || !loft.AuthenticatedReconstruction {
		return nil, fmt.Errorf("%w: mirrored cap chamfer lost its source Loft", decaderr.ErrUnsupported)
	}
	middle := loft.Frame1.Origin().Z
	byMotion := make(map[r3.Transform]*big.Rat, len(record.Join.Pairs))
	var sweptTotal, greatest float64
	for i, pair := range record.Join.Pairs {
		fp, ok := pair.Payload.(facetedrecord.Payload)
		if !ok || !fp.ContactAudited {
			return nil, fmt.Errorf("%w: mirrored cap tooth %d lost its source", decaderr.ErrUnsupported, i)
		}
		motion, ok := MirrorPairMotion(fp, record.Join.Source)
		if !ok {
			return nil, fmt.Errorf("%w: mirrored cap tooth %d lost its placement", decaderr.ErrUnsupported, i)
		}
		if _, duplicate := byMotion[motion]; duplicate {
			return nil, fmt.Errorf("%w: mirrored cap has duplicate tooth placement", decaderr.ErrUnsupported)
		}
		if AxialPlacement(motion) {
			byMotion[motion] = new(big.Rat)
			continue
		}
		shift, swept, proved := MirrorUpperBridge(pair, motion, middle)
		if !proved {
			return nil, fmt.Errorf("%w: mirrored cap tooth %d has no axial bridge", decaderr.ErrUnsupported, i)
		}
		exactShift := new(big.Rat).Mul(
			new(big.Rat).Mul(big.NewRat(2, 1), proofarith.FloatRat(middle)),
			new(big.Rat).Sub(big.NewRat(1, 1), proofarith.FloatRat(motion.Basis().EZ.Z)))
		if exactShift.Sign() <= 0 || exactShift.Cmp(proofarith.FloatRat(shift)) > 0 {
			return nil, fmt.Errorf("%w: mirrored cap tooth %d exceeds its exact bridge", decaderr.ErrUnsupported, i)
		}
		byMotion[motion] = exactShift
		greatest = max(greatest, shift)
		sweptTotal = proofbound.AbsSumUpper(sweptTotal, swept)
	}
	if greatest > record.Join.BridgeBound || sweptTotal > record.Join.BridgeVolume {
		return nil, fmt.Errorf("%w: mirrored cap source exceeds the inherited bridge", decaderr.ErrUnsupported)
	}
	if greatest == 0 {
		return nil, nil
	}
	// Circular Join sorts teeth by polar position. Match the authenticated
	// placed lower Loft, rather than relying on the caller's pair order.
	shifts := make([]*big.Rat, len(source.OriginalTeeth))
	for i, tooth := range source.OriginalTeeth {
		lp, ok := tooth.Payload.(loftrecord.Payload)
		if !ok || !lp.AuthenticatedReconstruction ||
			lp.SourceIdentity != loft.SourceIdentity {
			return nil, fmt.Errorf("%w: mirrored cap tooth %d lost its source identity", decaderr.ErrUnsupported, i)
		}
		shift, matched := byMotion[lp.Xform]
		if !matched || shift == nil {
			return nil, fmt.Errorf("%w: mirrored cap tooth %d has no matching placement", decaderr.ErrUnsupported, i)
		}
		shifts[i] = shift
		delete(byMotion, lp.Xform)
	}
	if len(byMotion) != 0 {
		return nil, fmt.Errorf("%w: mirrored cap has unmatched pair placements", decaderr.ErrUnsupported)
	}
	return shifts, nil
}
