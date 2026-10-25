package massdispatch

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/prooflineage"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CaptureContainedCutMoments captures source V/P/Q evidence for the admitted
// contained Cut pair. Unsupported source classes keep the existing mesh path.
func CaptureContainedCutMoments(ctx context.Context, result, target, tool *topo.Body,
	lineage *prooflineage.VolumeLineage) (*prooflineage.ContainedCutMoments, error) {
	if result == nil || target == nil || tool == nil || lineage == nil || lineage.Containment.BoundaryGapLower.Base() <= 0 {
		//nolint:nilnil // Missing optional evidence keeps the existing mesh path.
		return nil, nil
	}
	targetRecord, targetOK := admittedCutPrism(target, true)
	toolRecord, toolOK := admittedCutPrism(tool, false)
	if !targetOK || !toolOK {
		//nolint:nilnil // Unsupported source classes keep the existing mesh path.
		return nil, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	anchor := target.Centroid.Value
	targetMoments, err := massmoment.PrismWorldMoments(ctx, targetRecord.MassRecord(), anchor)
	if err != nil {
		return optionalMomentError(ctx, err)
	}
	toolMoments, err := massmoment.PrismWorldMoments(ctx, toolRecord.MassRecord(), anchor)
	if err != nil {
		return optionalMomentError(ctx, err)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	record, err := prooflineage.NewContainedCutMoments(int(result.Origin.Producer), lineage,
		prooflineage.SourceMoments{Producer: int(target.Origin.Producer), Snapshot: snapshotOf(anchor, targetMoments)},
		prooflineage.SourceMoments{Producer: int(tool.Origin.Producer), Snapshot: snapshotOf(anchor, toolMoments)})
	if errors.Is(err, prooflineage.ErrUnprovedContainedCutVolume) {
		//nolint:nilnil // An unproved positive difference is optional evidence.
		return nil, nil
	}
	if err != nil {
		return nil, err
	}
	return record, nil
}

func optionalMomentError(ctx context.Context, err error) (*prooflineage.ContainedCutMoments, error) {
	if ctx.Err() != nil {
		return nil, ctx.Err()
	}
	if errors.Is(err, decaderr.ErrUnsupported) {
		//nolint:nilnil // An unsupported integral keeps the existing mesh path.
		return nil, nil
	}
	return nil, err
}

func admittedCutPrism(body *topo.Body, target bool) (prismrecord.Payload, bool) {
	if body == nil || !body.Solid || body.Kind != topo.BodySolid {
		return prismrecord.Payload{}, false
	}
	pp, ok := body.Payload.(prismrecord.Payload)
	if !ok || pp.SurfaceResult || len(pp.Profile.Holes) != 0 || !pp.Frame.IsValid() || !pp.Xform.IsValid() ||
		pp.Reflected() || proofbound.IsNonFinite(pp.Z0) || proofbound.IsNonFinite(pp.Z1) || pp.Z0 >= pp.Z1 ||
		pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		!proofbound.FiniteVec(pp.Xform.Translation()) {
		return prismrecord.Payload{}, false
	}
	if target {
		basis := pp.Xform.Basis()
		if !pairbox.RectangularProfile(pp.Profile) ||
			!pairbox.CardinalBasis(pp.Frame.U(), pp.Frame.V(), pp.Frame.N()) ||
			!pairbox.CardinalBasis(basis.EX, basis.EY, basis.EZ) {
			return prismrecord.Payload{}, false
		}
		return pp, true
	}
	if len(pp.Profile.Outer.Segments) != 1 {
		return prismrecord.Payload{}, false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 ||
		!measurement.FiniteValues(circle.Center.U, circle.Center.V, circle.Radius.Base()) ||
		circle.Radius.Unit() != units.Millimeter || circle.Radius.Base() <= 0 {
		return prismrecord.Payload{}, false
	}
	return pp, true
}

func snapshotOf(anchor r3.Vec, moments massmoment.Moments) prooflineage.MomentSnapshot {
	return prooflineage.MomentSnapshot{
		Anchor: anchor, Volume: moments.Volume, First: moments.First, Second: moments.Second,
	}
}

func containedCutMassProperties(ctx context.Context, mesher verifiedmesh.Mesher, body *topo.Body,
	payload facetedrecord.Payload, density units.Value) (massmoment.MassReadings, error) {
	ordinary, err := facetedMassProperties(ctx, mesher, body, payload, density)
	if err != nil {
		return massmoment.MassReadings{}, err
	}
	if err := ctx.Err(); err != nil {
		return massmoment.MassReadings{}, err
	}
	moments, present, err := sourceMomentSnapshot(ctx, body, payload)
	if err != nil {
		return massmoment.MassReadings{}, err
	}
	if !present {
		if err := ctx.Err(); err != nil {
			return massmoment.MassReadings{}, err
		}
		return ordinary, nil
	}
	derived, err := publishContainedCutMoments(ctx, moments, density)
	if err != nil {
		if ctx.Err() != nil {
			return massmoment.MassReadings{}, ctx.Err()
		}
		if errors.Is(err, decaderr.ErrUnsupported) || errors.Is(err, decaderr.ErrNotFinite) {
			return ordinary, nil
		}
		return massmoment.MassReadings{}, booleanEvidenceError(err)
	}
	if !massCandidatesOverlap(ordinary, derived) ||
		!centroidBallsOverlap(ordinary.Center, derived.Center) ||
		!centroidBallsOverlap(derived.Center, body.Centroid) {
		return massmoment.MassReadings{}, booleanEvidenceError(errors.New("source and audited mass-property bounds are disjoint"))
	}
	if sourceMassPropertiesStrictlyBetter(ordinary, derived) {
		if err := ctx.Err(); err != nil {
			return massmoment.MassReadings{}, err
		}
		return derived, nil
	}
	if err := ctx.Err(); err != nil {
		return massmoment.MassReadings{}, err
	}
	return ordinary, nil
}

// publishContainedCutMoments publishes retained source V/P/Q for an original
// contained Cut or its certified direct exact-rotation Copy.
func publishContainedCutMoments(ctx context.Context, moments prooflineage.MomentSnapshot,
	density units.Value) (massmoment.MassReadings, error) {
	var local massmoment.Moments
	local.Volume, local.First, local.Second = moments.Volume, moments.First, moments.Second
	center, err := momentCenter(moments)
	if err != nil {
		return massmoment.MassReadings{}, err
	}
	identity := [3][3]*big.Rat{
		{big.NewRat(1, 1), new(big.Rat), new(big.Rat)},
		{new(big.Rat), big.NewRat(1, 1), new(big.Rat)},
		{new(big.Rat), new(big.Rat), big.NewRat(1, 1)},
	}
	tensor, mass, err := massmoment.AffineInertia(local, identity, density)
	if err != nil {
		return massmoment.MassReadings{}, err
	}
	return massmoment.Publish(ctx, center, mass, tensor)
}

func momentCenter(moments prooflineage.MomentSnapshot) (measurement.VecMeasurement, error) {
	if moments.Volume.Lo == nil || moments.Volume.Hi == nil || moments.Volume.Lo.Sign() <= 0 {
		return measurement.VecMeasurement{}, fmt.Errorf("%w: source moment volume is not positive", decaderr.ErrUnsupported)
	}
	anchor := [3]float64{moments.Anchor.X, moments.Anchor.Y, moments.Anchor.Z}
	var center [3]float64
	bound := 0.0
	for i := range 3 {
		offset, ok := proofbound.IntervalQuo(moments.First[i], moments.Volume)
		if !ok {
			return measurement.VecMeasurement{}, fmt.Errorf("%w: source moment center has no finite quotient", decaderr.ErrUnsupported)
		}
		interval := proofbound.IntervalAdd(proofbound.PointInterval(proof.FloatRat(anchor[i])), offset)
		center[i], _ = new(big.Rat).Quo(new(big.Rat).Add(interval.Lo, interval.Hi), big.NewRat(2, 1)).Float64()
		if proofbound.IsNonFinite(center[i]) {
			return measurement.VecMeasurement{}, fmt.Errorf("%w: source moment center is not finite", decaderr.ErrNotFinite)
		}
		bound = math.Max(bound, proofbound.IntervalFloatError(interval, center[i]))
	}
	bound = proofbound.Radius3D(bound)
	if proofbound.IsNonFinite(bound) {
		return measurement.VecMeasurement{}, fmt.Errorf("%w: source moment center bound is not finite", decaderr.ErrNotFinite)
	}
	return measurement.VecMeasurement{
		Value: r3.Vec{X: center[0], Y: center[1], Z: center[2]},
		Bound: units.Millimeters(bound), Exactness: measurement.ExactnessOf(bound),
	}, nil
}

func massCandidatesOverlap(a, b massmoment.MassReadings) bool {
	if !measurementIntervalsOverlap(a.Mass, b.Mass) {
		return false
	}
	for i := range a.Tensor {
		if !measurementIntervalsOverlap(a.Tensor[i], b.Tensor[i]) {
			return false
		}
	}
	return true
}

func measurementIntervalsOverlap(a, b measurement.Measurement) bool {
	left, okLeft := readingInterval(a)
	right, okRight := readingInterval(b)
	return okLeft && okRight && intervalsOverlap(left, right)
}

func readingInterval(reading measurement.Measurement) (proofbound.RatInterval, bool) {
	if !measurement.FiniteValues(reading.Value.Base(), reading.Bound.Base()) || reading.Bound.Base() < 0 ||
		reading.Exactness != measurement.ExactnessOf(reading.Bound.Base()) {
		return proofbound.RatInterval{}, false
	}
	center, radius := proof.FloatRat(reading.Value.Base()), proof.FloatRat(reading.Bound.Base())
	if center == nil || radius == nil {
		return proofbound.RatInterval{}, false
	}
	return proofbound.Interval(new(big.Rat).Sub(center, radius), new(big.Rat).Add(center, radius)), true
}

func intervalsOverlap(a, b proofbound.RatInterval) bool {
	return a.Lo.Cmp(b.Hi) <= 0 && b.Lo.Cmp(a.Hi) <= 0
}

func centroidBallsOverlap(a, b measurement.VecMeasurement) bool {
	if !proofbound.FiniteVec(a.Value) || !proofbound.FiniteVec(b.Value) ||
		!measurement.FiniteValues(a.Bound.Base(), b.Bound.Base()) || a.Bound.Base() < 0 || b.Bound.Base() < 0 {
		return false
	}
	radii := proofbound.AbsSumUpper(a.Bound.Base(), b.Bound.Base())
	return !proofbound.IsNonFinite(radii) && proofbound.DvLenAtLeast(radii, proofbound.HeldDelta(a.Value, b.Value))
}

func sourceMassPropertiesStrictlyBetter(ordinary, source massmoment.MassReadings) bool {
	strict := source.Mass.Bound.Base() < ordinary.Mass.Bound.Base()
	if source.Mass.Bound.Base() > ordinary.Mass.Bound.Base() || source.Center.Bound.Base() > ordinary.Center.Bound.Base() {
		return false
	}
	strict = strict || source.Center.Bound.Base() < ordinary.Center.Bound.Base()
	for i := range source.Tensor {
		sourceBound, ordinaryBound := source.Tensor[i].Bound.Base(), ordinary.Tensor[i].Bound.Base()
		if sourceBound > ordinaryBound {
			return false
		}
		strict = strict || sourceBound < ordinaryBound
	}
	return strict
}

func booleanEvidenceError(err error) error {
	return fmt.Errorf("%w: %v", decaderr.ErrBooleanFailed, err)
}
