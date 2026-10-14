package decad

import (
	"context"
	"fmt"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Recompute the original upper-tooth translations from the authenticated
// source placements. The Join's accumulated bounds must still contain them.
func loftMirrorCylinderChamferUpperShifts(record *loftMirrorCylinderBoreRecord,
	source loftCylinderChamferSource) ([]*big.Rat, error) {
	if record == nil || record.join == nil || record.join.source == nil ||
		record.join.source.lower == nil || len(record.join.pairs) == 0 ||
		len(record.join.pairs) != len(source.originalTeeth) {
		return nil, fmt.Errorf("%w: mirrored cap chamfer lost its upper tooth source", ErrUnsupported)
	}
	loft, ok := record.join.source.lower.n.Payload.(loftPayload)
	if !ok || !loft.authenticatedReconstruction {
		return nil, fmt.Errorf("%w: mirrored cap chamfer lost its source Loft", ErrUnsupported)
	}
	middle := loft.frame1.Origin().Z
	byMotion := make(map[r3.Transform]*big.Rat, len(record.join.pairs))
	var sweptTotal, greatest float64
	for i, pair := range record.join.pairs {
		fp, ok := pair.n.Payload.(facetedPayload)
		if !ok || !fp.contactAudited {
			return nil, fmt.Errorf("%w: mirrored cap tooth %d lost its source", ErrUnsupported, i)
		}
		motion, ok := loftMirrorCylinderPairMotion(fp, record.join.source)
		if !ok {
			return nil, fmt.Errorf("%w: mirrored cap tooth %d lost its placement", ErrUnsupported, i)
		}
		if _, duplicate := byMotion[motion]; duplicate {
			return nil, fmt.Errorf("%w: mirrored cap has duplicate tooth placement", ErrUnsupported)
		}
		if loftCylinderAxialPlacement(motion) {
			byMotion[motion] = new(big.Rat)
			continue
		}
		shift, swept, proved, err := loftMirrorCylinderUpperBridge(pair, motion, middle)
		if err != nil {
			return nil, err
		}
		if !proved {
			return nil, fmt.Errorf("%w: mirrored cap tooth %d has no axial bridge", ErrUnsupported, i)
		}
		exactShift := new(big.Rat).Mul(
			new(big.Rat).Mul(big.NewRat(2, 1), proofarith.FloatRat(middle)),
			new(big.Rat).Sub(big.NewRat(1, 1), proofarith.FloatRat(motion.Basis().EZ.Z)))
		if exactShift.Sign() <= 0 || exactShift.Cmp(proofarith.FloatRat(shift)) > 0 {
			return nil, fmt.Errorf("%w: mirrored cap tooth %d exceeds its exact bridge", ErrUnsupported, i)
		}
		byMotion[motion] = exactShift
		greatest = max(greatest, shift)
		sweptTotal = proofbound.AbsSumUpper(sweptTotal, swept)
	}
	if greatest > record.join.bridgeBound || sweptTotal > record.join.bridgeVolume {
		return nil, fmt.Errorf("%w: mirrored cap source exceeds the inherited bridge", ErrUnsupported)
	}
	if greatest == 0 {
		return nil, nil
	}
	// Circular Join sorts teeth by polar position. Match the authenticated
	// placed lower Loft, rather than relying on the caller's pair order.
	shifts := make([]*big.Rat, len(source.originalTeeth))
	for i, tooth := range source.originalTeeth {
		lp, ok := tooth.n.Payload.(loftPayload)
		if !ok || !lp.authenticatedReconstruction ||
			lp.sourceIdentity != loft.sourceIdentity {
			return nil, fmt.Errorf("%w: mirrored cap tooth %d lost its source identity", ErrUnsupported, i)
		}
		shift, matched := byMotion[lp.xform]
		if !matched || shift == nil {
			return nil, fmt.Errorf("%w: mirrored cap tooth %d has no matching placement", ErrUnsupported, i)
		}
		shifts[i] = shift
		delete(byMotion, lp.xform)
	}
	if len(byMotion) != 0 {
		return nil, fmt.Errorf("%w: mirrored cap has unmatched pair placements", ErrUnsupported)
	}
	return shifts, nil
}

// The upper source is the reflected lower source shifted down by each
// tooth's authenticated bridge. The one-cap split rings were independently
// checked at these physical levels. Its remaining top sliver is covered by
// an exact transverse section of the actual upper cap band.
func loftMirrorCylinderChamferUpperCapBridge(ctx context.Context, old, upper facetedPayload,
	record *loftMirrorCylinderBoreRecord, half loftCylinderOneCapData, setback float64) (float64, error) {
	shift := record.join.bridgeBound
	if shift == 0 {
		return 0, nil
	}
	blank, ok := record.join.blank.n.Payload.(prismPayload)
	if !ok || !loftMirrorCylinderBlank(blank) || len(half.ideal) != len(upper.verts) ||
		half.band[0] >= half.band[1] || len(record.outer[1]) < 3 {
		return 0, fmt.Errorf("%w: mirrored cap bridge lost its upper band", ErrUnsupported)
	}
	loft, ok := record.join.source.lower.n.Payload.(loftPayload)
	if !ok || !loft.authenticatedReconstruction {
		return 0, fmt.Errorf("%w: mirrored cap bridge lost its source frame", ErrUnsupported)
	}
	twiceMiddle := new(big.Rat).Mul(big.NewRat(2, 1),
		proofarith.FloatRat(loft.frame1.Origin().Z))
	ideal := make([]proofarith.Xpt, len(half.ideal))
	for i, p := range half.ideal {
		if p.W == nil || p.W.Sign() <= 0 {
			return 0, fmt.Errorf("%w: mirrored cap bridge lost an exact band vertex", ErrUnsupported)
		}
		z := new(big.Rat).Sub(twiceMiddle, new(big.Rat).SetFrac(p.Z, p.W))
		ideal[i] = proofarith.Xpt{
			X: new(big.Int).Mul(p.X, z.Denom()),
			Y: new(big.Int).Mul(p.Y, z.Denom()),
			Z: new(big.Int).Mul(p.W, z.Num()),
			W: new(big.Int).Mul(p.W, z.Denom()),
		}
	}
	mesh := &joinedMesh{verts: append([]r3.Vec(nil), upper.verts...), tris: upper.tris}
	outer := make([]int, len(record.outer[1]))
	for i, index := range record.outer[1] {
		if index < 0 || index >= len(old.verts) {
			return 0, fmt.Errorf("%w: mirrored cap bridge lost its old outer rim", ErrUnsupported)
		}
		outer[i] = len(mesh.verts)
		mesh.verts = append(mesh.verts, old.verts[index])
		ideal = append(ideal, proofarith.XptOf(old.verts[index]))
	}
	source := loftCylinderChamferSource{
		z1: blank.Z1, bridgeBound: shift,
		topSourceLevel: new(big.Rat).Sub(proofarith.FloatRat(blank.Z1), proofarith.FloatRat(shift)),
	}
	source.outer[1] = outer
	return loftCylinderChamferBridgeCap(ctx, old, source, mesh, ideal, half.band,
		half.outer[0], setback)
}
