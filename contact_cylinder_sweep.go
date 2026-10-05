package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// sourceCylinderClearSweep proves only a separated path. Both source solids
// translate affinely, and one axis separates their full swept outer boxes.
func (d *Document) sourceCylinderClearSweep(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, report *SweepReport,
	cylinder sourceCylinderContactProof, box sourceBoxContactProof, cylinderFirst bool) (*SweepReport, error) {
	firstBox, secondBox := box, cylinder.box
	if cylinderFirst {
		firstBox, secondBox = cylinder.box, box
	}
	fullA, fullB := sweptAffineBox(firstBox, pa.delta), sweptAffineBox(secondBox, pb.delta)
	resolution, _ := exactBaseValue(req.PointResolution)
	axis := cylinder.axis
	axisGap := func(a, b sourceBoxContactProof) (int, *big.Rat, bool) {
		if gap := new(big.Rat).Sub(b.lo[axis].rat(), a.hi[axis].rat()); gap.Cmp(resolution) > 0 {
			return 1, gap, true
		}
		if gap := new(big.Rat).Sub(a.lo[axis].rat(), b.hi[axis].rat()); gap.Cmp(resolution) > 0 {
			return -1, gap, true
		}
		return 0, nil, false
	}
	sign, gap, ok := axisGap(fullA, fullB)
	startCylinder, startBox := secondBox, firstBox
	cylinderDelta, boxDelta := pb.delta, pa.delta
	if cylinderFirst {
		startCylinder, startBox = firstBox, secondBox
		cylinderDelta, boxDelta = pa.delta, pb.delta
	}
	endCylinder := translatedAffineBox(startCylinder, cylinderDelta)
	endBox := translatedAffineBox(startBox, boxDelta)
	if !ok || !cylinderInsideBoxFace(startCylinder, startBox, axis) ||
		!cylinderInsideBoxFace(endCylinder, endBox, axis) {
		return cylinderSweepUndecided(report, pa.duration), nil
	}
	startA, startB := pa.from, pb.from
	endA, err := pa.poseAt(big.NewRat(1, 1))
	if err != nil {
		return nil, err
	}
	endB, err := pb.poseAt(big.NewRat(1, 1))
	if err != nil {
		return nil, err
	}
	for index, poses := range [][2]r3.Transform{{startA, startB}, {endA, endB}} {
		contact, err := d.ContactPair(ctx, a, b, poses[0], poses[1], req.ContactRequest)
		if err != nil {
			return nil, err
		}
		if contact.Relation != ContactSeparated {
			return cylinderSweepUndecided(report, pa.duration), nil
		}
		fraction := big.NewRat(int64(index), 1)
		observedA, okA := translatedReplayBox(firstBox, pa.from, poses[0])
		observedB, okB := translatedReplayBox(secondBox, pb.from, poses[1])
		if !okA || !okB {
			return cylinderSweepUndecided(report, pa.duration), nil
		}
		deviation := boxPoseDeviation(firstBox, observedA, pa.delta, fraction)
		deviation.Add(deviation, boxPoseDeviation(secondBox, observedB, pb.delta, fraction))
		if deviation.Cmp(resolution) > 0 || !outerBoxGapExceeds(observedA, observedB, axis, sign, deviation) {
			return cylinderSweepUndecided(report, pa.duration), nil
		}
		at := sweepInstant(fraction, pa.duration)
		ideal := SweepEvent{At: at, Relation: ContactSeparated, Gap: contact.Gap}
		report.Samples = append(report.Samples, SweepSample{At: at, PoseA: poses[0], PoseB: poses[1],
			FloatContact: contact, Ideal: ideal, exactFraction: fraction})
		report.PoseEvaluations++
	}
	report.Outcome, report.BoxExcluded = SweepClear, true
	report.replay = &sweepReplayProof{pa: pa, pb: pb, request: req.ContactRequest,
		cylinder: &cylinder, cylinderFirst: cylinderFirst, boxA: firstBox, boxB: secondBox,
		clearAxis: axis, clearSign: sign, clearGap: gap}
	report.replay.snapshot(report)
	return report, nil
}

func cylinderSweepUndecided(report *SweepReport, duration *big.Rat) *SweepReport {
	report.Outcome, report.Cause = SweepUndecided, SweepContactUnsupported
	report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), duration),
		To: sweepInstant(big.NewRat(1, 1), duration)}
	return report
}

func sweptAffineBox(start sourceBoxContactProof, delta [3]dyadic) sourceBoxContactProof {
	for axis := range 3 {
		endLo := dyAdd(start.lo[axis], delta[axis])
		endHi := dyAdd(start.hi[axis], delta[axis])
		start.lo[axis] = dyMin(start.lo[axis], endLo)
		start.hi[axis] = dyMax(start.hi[axis], endHi)
	}
	return start
}

func translatedAffineBox(start sourceBoxContactProof, delta [3]dyadic) sourceBoxContactProof {
	for axis := range 3 {
		start.lo[axis] = dyAdd(start.lo[axis], delta[axis])
		start.hi[axis] = dyAdd(start.hi[axis], delta[axis])
	}
	return start
}

func outerBoxGapExceeds(a, b sourceBoxContactProof, axis, sign int, minimum *big.Rat) bool {
	if sign > 0 {
		return new(big.Rat).Sub(b.lo[axis].rat(), a.hi[axis].rat()).Cmp(minimum) > 0
	}
	return new(big.Rat).Sub(a.lo[axis].rat(), b.hi[axis].rat()).Cmp(minimum) > 0
}
