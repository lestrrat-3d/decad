package massdispatch

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/prooflineage"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CaptureExactCopyMoments freezes and checks contained-Cut moments for an
// admitted direct exact-rotation Copy. Unsupported sources omit this optional
// evidence and keep the existing verified-mesh path.
func CaptureExactCopyMoments(ctx context.Context, source, result *topo.Body,
	rotation *exactrotation.Rotation) (*facetedrecord.ExactCopyMomentRecord, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if source == nil || result == nil || rotation == nil {
		//nolint:nilnil // Missing optional inputs keep the verified-mesh path.
		return nil, nil
	}
	if !result.Solid || result.Kind != topo.BodySolid {
		return nil, booleanEvidenceError(errors.New("exact-copy result is not a solid"))
	}
	sourcePayload, ok := source.Payload.(facetedrecord.Payload)
	if !ok || !source.Solid || source.Kind != topo.BodySolid || sourcePayload.Xform != r3.Identity() ||
		sourcePayload.CopyLineage != nil || sourcePayload.ExactCopyMoments != nil || sourcePayload.ExactPlaced ||
		source.VolumeLineage == nil || source.VolumeLineage.Moments == nil {
		//nolint:nilnil // Excluded source classes keep the verified-mesh path.
		return nil, nil
	}
	resultPayload, ok := result.Payload.(facetedrecord.Payload)
	if !ok || resultPayload.CopyLineage == nil {
		//nolint:nilnil // Missing optional direct-copy binding keeps the verified-mesh path.
		return nil, nil
	}
	lineage := resultPayload.CopyLineage
	if lineage.Source != source || lineage.SourceProducer != source.Origin.Producer ||
		lineage.SourcePlacement != sourcePayload.Xform || lineage.TargetPlacement != resultPayload.Xform ||
		!lineage.Direct || lineage.Exact != rotation {
		return nil, booleanEvidenceError(errors.New("exact-copy source binding disagrees with the rebuilt copy"))
	}
	if _, _, _, err := source.VolumeLineage.Moments.Read(int(source.Origin.Producer), source.VolumeLineage); err != nil {
		return nil, booleanEvidenceError(err)
	}
	record, err := facetedrecord.NewExactCopyMomentRecord(source.Origin.Producer, result.Origin.Producer,
		sourcePayload.Xform, resultPayload.Xform, source.VolumeLineage, rotation)
	if err != nil {
		return nil, booleanEvidenceError(err)
	}
	difference, linear, translation, err := record.Read(result.Origin.Producer, resultPayload.Xform)
	if err != nil {
		return nil, booleanEvidenceError(err)
	}
	transported, err := transportExactCopyMoments(ctx, difference, linear, translation)
	if err != nil {
		if ctx.Err() != nil {
			return nil, ctx.Err()
		}
		if errors.Is(err, decaderr.ErrUnsupported) || errors.Is(err, decaderr.ErrNotFinite) {
			//nolint:nilnil // A finite anchor or center that cannot be represented is optional.
			return nil, nil
		}
		return nil, booleanEvidenceError(err)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if !cachedVolumeOverlaps(transported.Volume, result.Volume) {
		return nil, booleanEvidenceError(errors.New("transported source Volume disagrees with copied body Volume"))
	}
	center, err := momentCenter(transported)
	if err != nil {
		if ctx.Err() != nil {
			return nil, ctx.Err()
		}
		if errors.Is(err, decaderr.ErrUnsupported) || errors.Is(err, decaderr.ErrNotFinite) {
			//nolint:nilnil // A finite source center that cannot be represented is optional.
			return nil, nil
		}
		return nil, booleanEvidenceError(err)
	}
	if result.Centroid.Bound.Unit() != units.Millimeter ||
		result.Centroid.Exactness != measurement.ExactnessOf(result.Centroid.Bound.Base()) {
		return nil, booleanEvidenceError(errors.New("copied body centroid reading is malformed"))
	}
	if !centroidBallsOverlap(center, result.Centroid) {
		return nil, booleanEvidenceError(errors.New("transported source center disagrees with copied body centroid"))
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return record, nil
}

// transportExactCopyMoments transports V/P/Q through the exact rotation, then
// encloses the rounded held anchor. It computes V*=V, P*=Q P, Q*=Q Q_source
// Q^T; D_i=[-e,e]; P'_i=P*_i+V*D_i and Q'_ij=Q*_ij+D_iP*_j+P*_iD_j+V*D_iD_j.
func transportExactCopyMoments(ctx context.Context, source prooflineage.MomentSnapshot,
	linear [3][3]*big.Rat, translation [3]*big.Rat) (prooflineage.MomentSnapshot, error) {
	if err := ctx.Err(); err != nil {
		return prooflineage.MomentSnapshot{}, err
	}
	rotated := massmoment.Transform(massmoment.Moments{
		Volume: source.Volume, First: source.First, Second: source.Second,
	}, linear)
	anchorReading := measurement.VecMeasurement{
		Value: source.Anchor, Bound: units.Millimeters(0), Exactness: measurement.Exact,
	}
	imageAnchor, err := facetedrecord.PlacedCapCentroidBy(anchorReading, linear, translation)
	if err != nil {
		return prooflineage.MomentSnapshot{}, err
	}
	rotated, err = massmoment.WidenTranslation(rotated, imageAnchor.Bound.Base())
	if err != nil {
		return prooflineage.MomentSnapshot{}, err
	}
	if err := ctx.Err(); err != nil {
		return prooflineage.MomentSnapshot{}, err
	}
	return prooflineage.MomentSnapshot{
		Anchor: imageAnchor.Value, Volume: rotated.Volume, First: rotated.First, Second: rotated.Second,
	}, nil
}

func cachedVolumeOverlaps(volume proofbound.RatInterval, cached measurement.Measurement) bool {
	if cached.Value.Unit() != units.CubicMillimeter || cached.Bound.Unit() != units.CubicMillimeter ||
		!measurement.FiniteValues(cached.Value.Base(), cached.Bound.Base()) || cached.Bound.Base() < 0 {
		return false
	}
	reading, ok := readingInterval(cached)
	if !ok {
		return false
	}
	return intervalsOverlap(volume, reading)
}

func sourceMomentSnapshot(ctx context.Context, body *topo.Body,
	payload facetedrecord.Payload) (prooflineage.MomentSnapshot, bool, error) {
	lineageHas := body.VolumeLineage != nil && body.VolumeLineage.Moments != nil
	copyHas := payload.ExactCopyMoments != nil
	if lineageHas && copyHas {
		return prooflineage.MomentSnapshot{}, false,
			booleanEvidenceError(fmt.Errorf("original and exact-copy moment evidence are both present"))
	}
	if !lineageHas && !copyHas {
		return prooflineage.MomentSnapshot{}, false, nil
	}
	if err := ctx.Err(); err != nil {
		return prooflineage.MomentSnapshot{}, false, err
	}
	if lineageHas {
		_, _, difference, err := body.VolumeLineage.Moments.Read(int(body.Origin.Producer), body.VolumeLineage)
		if err != nil {
			return prooflineage.MomentSnapshot{}, false, booleanEvidenceError(err)
		}
		return difference, true, nil
	}
	snapshot, linear, translation, err := payload.ExactCopyMoments.Read(body.Origin.Producer, payload.Xform)
	if err != nil {
		return prooflineage.MomentSnapshot{}, false, booleanEvidenceError(err)
	}
	transported, err := transportExactCopyMoments(ctx, snapshot, linear, translation)
	if err != nil {
		if ctx.Err() != nil {
			return prooflineage.MomentSnapshot{}, false, ctx.Err()
		}
		if errors.Is(err, decaderr.ErrUnsupported) || errors.Is(err, decaderr.ErrNotFinite) {
			return prooflineage.MomentSnapshot{}, false, nil
		}
		return prooflineage.MomentSnapshot{}, false, booleanEvidenceError(err)
	}
	return transported, true, nil
}
