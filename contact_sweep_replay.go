package decad

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sweepReplayProof keeps the source boxes used by the continuous sweep. A
// replayed float pose is checked against those boxes, rather than inferred
// from the sweep's endpoint samples.
type sweepReplayProof struct {
	pa, pb     affinePairPath
	boxA, boxB sourceBoxContactProof
	request    ContactRequest
	outcome    SweepOutcome
	bracketLo  *big.Rat
	bracketHi  *big.Rat
}

func (p *sweepReplayProof) snapshot(r *SweepReport) {
	p.outcome = r.Outcome
	if r.Bracket != nil {
		p.bracketLo = exactReplayFraction(r.Bracket.From.Fraction)
		p.bracketHi = exactReplayFraction(r.Bracket.To.Fraction)
	}
}

// CertifiedPosesAt evaluates the recorded affine paths at elapsed time and
// checks their rounded placements against the sweep's exact source-box proof.
// It reads no Document geometry and performs no new contact query. A pose whose
// rounding can change the reported relation beyond PointResolution is refused.
func (r *SweepReport) CertifiedPosesAt(elapsed units.Value) (r3.Transform, r3.Transform, error) {
	if r == nil || r.replay == nil || elapsed.Kind() != units.Time ||
		!finiteMeasurementValues(elapsed.Base()) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: sweep has no affine replay proof", ErrUnsupported)
	}
	t, ok := exactBaseValue(elapsed)
	if !ok || t.Sign() < 0 || t.Cmp(r.replay.pa.duration) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is outside the sweep", ErrDegenerate)
	}
	f := new(big.Rat).Quo(t, r.replay.pa.duration)
	if !r.replayFractionCovered(f) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay time is outside the certified sweep prefix", ErrUnsupported)
	}
	poseA, err := r.replay.pa.poseAt(f)
	if err != nil {
		return r3.Transform{}, r3.Transform{}, err
	}
	poseB, err := r.replay.pb.poseAt(f)
	if err != nil {
		return r3.Transform{}, r3.Transform{}, err
	}
	actualA, ok := translatedReplayBox(r.replay.boxA, r.replay.pa.from, poseA)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay pose A is not an affine translation", ErrUnsupported)
	}
	actualB, ok := translatedReplayBox(r.replay.boxB, r.replay.pb.from, poseB)
	if !ok {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: replay pose B is not an affine translation", ErrUnsupported)
	}
	deviation := boxPoseDeviation(r.replay.boxA, actualA, r.replay.pa.delta, f)
	deviation.Add(deviation, boxPoseDeviation(r.replay.boxB, actualB, r.replay.pb.delta, f))
	resolution, ok := exactBaseValue(r.replay.request.PointResolution)
	if !ok || deviation.Cmp(resolution) > 0 {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose exceeds point resolution", ErrUnsupported)
	}
	contact := &ContactReport{Request: r.replay.request}
	classifySourceBoxes(contact, actualA, actualB)
	if !r.replayRelationCovered(f, contact.Relation, actualA, actualB, resolution) {
		return r3.Transform{}, r3.Transform{}, fmt.Errorf("%w: rounded replay pose changes the certified relation", ErrUnsupported)
	}
	return poseA, poseB, nil
}

func translatedReplayBox(box sourceBoxContactProof, from, at r3.Transform) (sourceBoxContactProof, bool) {
	if !at.IsValid() || at.Basis() != from.Basis() {
		return sourceBoxContactProof{}, false
	}
	start, end := from.Translation(), at.Translation()
	before := [3]float64{start.X, start.Y, start.Z}
	after := [3]float64{end.X, end.Y, end.Z}
	for i := range 3 {
		if !finiteMeasurementValues(before[i], after[i]) {
			return sourceBoxContactProof{}, false
		}
		move := dySubScalar(mustDyOf(after[i]), mustDyOf(before[i]))
		box.lo[i], box.hi[i] = dyAdd(box.lo[i], move), dyAdd(box.hi[i], move)
	}
	return box, true
}

func (r *SweepReport) replayFractionCovered(f *big.Rat) bool {
	switch r.replay.outcome {
	case SweepClear, SweepDepartedClear, SweepPersistentTouch:
		return true
	case SweepImpactBracket, SweepContactTransitionBracket:
		return r.replay.bracketHi != nil && f.Cmp(r.replay.bracketHi) <= 0
	default:
		return false
	}
}

func exactReplayFraction(v units.Value) *big.Rat {
	f, _ := exactBaseValue(v)
	return f
}

func (r *SweepReport) replayRelationCovered(f *big.Rat, relation ContactRelation,
	a, b sourceBoxContactProof, resolution *big.Rat) bool {
	switch r.replay.outcome {
	case SweepClear:
		return relation == ContactSeparated
	case SweepDepartedClear:
		if f.Sign() == 0 {
			return relation == ContactTouching
		}
		return relation == ContactSeparated
	case SweepPersistentTouch:
		return relation == ContactTouching ||
			(relation == ContactSeparated || relation == ContactOverlapping) &&
				boxRelationDistanceWithin(a, b, resolution)
	case SweepImpactBracket:
		if r.replay.bracketLo == nil {
			return false
		}
		return relation == ContactSeparated || relation == ContactTouching ||
			relation == ContactOverlapping && boxRelationDistanceWithin(a, b, resolution)
	case SweepContactTransitionBracket:
		return relation == ContactTouching || relation == ContactSeparated ||
			relation == ContactOverlapping && boxRelationDistanceWithin(a, b, resolution)
	default:
		return false
	}
}

// For axis-aligned boxes, the least support gap (separated case) or least
// penetration depth (overlap case) is bounded directly from exact endpoints.
func boxRelationDistanceWithin(a, b sourceBoxContactProof, limit *big.Rat) bool {
	var distance *big.Rat
	for axis := range 3 {
		for _, candidate := range []*big.Rat{
			new(big.Rat).Abs(new(big.Rat).Sub(a.hi[axis].rat(), b.lo[axis].rat())),
			new(big.Rat).Abs(new(big.Rat).Sub(b.hi[axis].rat(), a.lo[axis].rat())),
		} {
			if distance == nil || candidate.Cmp(distance) < 0 {
				distance = candidate
			}
		}
	}
	return distance != nil && distance.Cmp(limit) <= 0
}
