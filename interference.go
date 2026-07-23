package decad

import (
	"context"
	"math"
	"reflect"

	"github.com/lestrrat-3d/r3"
)

// analyticBodiesEqual is the exact set-identity fast path for evaluator
// payloads whose records are already in their normalized value form. Exact
// structural equality is deliberately only a sufficient certificate: a
// different record that might describe the same set stays undecided.
//
// The only unbounded represented-set fields compared here are the
// ProfileRecords, so they are walked loop by loop and segment by segment under
// the budget, leaving reflect.DeepEqual as a leaf over one segment — a bounded
// exact predicate, which is what §7.2 allows to stay context-free. The remaining
// comparisons cover the fixed-size fields that determine the represented point
// set. They may deliberately ignore derived certificate metadata, such as
// cupPayload.thickness and cupPayload.sense, because it does not change set
// identity.
func analyticBodiesEqual(budget *workBudget, a, b *Body) (bool, error) {
	switch pa := a.payload.(type) {
	case prismPayload:
		pb, ok := b.payload.(prismPayload)
		if !ok || pa.frame != pb.frame || pa.z0 != pb.z0 || pa.z1 != pb.z1 || pa.xform != pb.xform {
			return false, nil
		}
		return profileRecordsEqual(budget, pa.profile, pb.profile)
	case cupPayload:
		pb, ok := b.payload.(cupPayload)
		if !ok || pa.frame != pb.frame || pa.zOpen != pb.zOpen || pa.zOuter != pb.zOuter ||
			pa.zCav != pb.zCav || pa.xform != pb.xform {
			return false, nil
		}
		same, err := profileRecordsEqual(budget, pa.outer, pb.outer)
		if err != nil || !same {
			return false, err
		}
		return profileRecordsEqual(budget, pa.cavity, pb.cavity)
	case revolvePayload:
		pb, ok := b.payload.(revolvePayload)
		if !ok || pa.frame != pb.frame || pa.ax != pb.ax || pa.phi0 != pb.phi0 ||
			pa.phi1 != pb.phi1 || pa.full != pb.full || pa.xform != pb.xform {
			return false, nil
		}
		return profileRecordsEqual(budget, pa.profile, pb.profile)
	default:
		return false, nil
	}
}

// profileRecordsEqual reports exact structural equality of two recorded
// profiles, stepping the budget once per segment compared.
func profileRecordsEqual(budget *workBudget, a, b ProfileRecord) (bool, error) {
	if err := budget.step(); err != nil {
		return false, err
	}
	if len(a.Holes) != len(b.Holes) || (a.Holes == nil) != (b.Holes == nil) {
		return false, nil
	}
	same, err := loopRecordsEqual(budget, a.Outer, b.Outer)
	if err != nil || !same {
		return false, err
	}
	for i := range a.Holes {
		same, err := loopRecordsEqual(budget, a.Holes[i], b.Holes[i])
		if err != nil || !same {
			return false, err
		}
	}
	return true, nil
}

// loopRecordsEqual compares one loop segment by segment. The nil-versus-empty
// slice check keeps this exactly as strict as a whole-record DeepEqual, which
// holds a nil slice unequal to an empty one.
func loopRecordsEqual(budget *workBudget, a, b LoopRecord) (bool, error) {
	if len(a.Segments) != len(b.Segments) || (a.Segments == nil) != (b.Segments == nil) {
		return false, nil
	}
	for i := range a.Segments {
		if err := budget.step(); err != nil {
			return false, err
		}
		if !reflect.DeepEqual(a.Segments[i], b.Segments[i]) {
			return false, nil
		}
	}
	return true, nil
}

// interferenceOutcome distinguishes why the overlap volume could not be
// measured, so Verify can pick the matching diagnostic (verification §1.1): a
// staged payload or contact is DiagUnsupportedPair, any other unmeasured
// result DiagUndecidedPair (or DiagUndecidedInterference for a proven overlap).
type interferenceOutcome int

const (
	// interferenceMeasured — a positive bounded overlap volume was proven.
	interferenceMeasured interferenceOutcome = iota
	// interferenceUndecided — the read-only proof resolved neither way.
	interferenceUndecided
	// interferenceUnsupported — a staged payload or contact (core §8).
	interferenceUnsupported
)

// measuredInterference returns the pair's bounded overlap volume. Strict
// containment and exact analytic equality prove the set identity directly;
// every other pair uses the read-only mesh intersection and its positive
// lower-volume gate. The outcome names why an unmeasured result is unmeasured.
func measuredInterference(ctx context.Context, a, b *Body, res pairResult) (Measurement, interferenceOutcome, error) {
	if res.contained != nil {
		return res.contained.volume, interferenceMeasured, nil
	}
	equal, err := analyticBodiesEqual(newWorkBudget(ctx), a, b)
	if err != nil {
		return Measurement{}, interferenceUndecided, err
	}
	if equal {
		// Stable pair order chooses A when the represented sets are equal.
		return a.volume, interferenceMeasured, nil
	}
	eval, err := evaluateBoolean(ctx, OpIntersect, a, b)
	if err != nil {
		if expected, ok := asExpectedBoolean(err); ok {
			// A staged payload (a revolve or cup operand this evaluator cannot
			// tessellate, booleanExpectedStaging) AND a staged boolean contact
			// (core §8) are both DiagUnsupportedPair per verification §1.1 — a
			// staged payload or a coplanar/tangent contact is an unsupported
			// pair, not an undecided partition.
			if expected.kind == booleanExpectedUnsupported ||
				expected.kind == booleanExpectedContact ||
				expected.kind == booleanExpectedStaging {
				return Measurement{}, interferenceUnsupported, nil
			}
			return Measurement{}, interferenceUndecided, nil
		}
		return Measurement{}, interferenceUndecided, err
	}
	value := math.Abs(eval.volume.Value.Base())
	if value-eval.volume.Bound.Base() <= 0 {
		return Measurement{}, interferenceUndecided, nil
	}
	return eval.volume, interferenceMeasured, nil
}

// interferencePairDiameter reads the greatest supported point distance from
// the pair. Analytic bodies contribute their exact support set from the
// clearance model; a faceted payload contributes every held mesh vertex,
// including interior tessellation vertices that have no B-rep Vertex. An
// incomplete set can only understate D and tighten the noise floor, never
// admit a coarse answer.
func interferencePairDiameter(ctx context.Context, a, b *Body) (float64, error) {
	budget := newWorkBudget(ctx)
	var points []r3.Vec
	for _, body := range []*Body{a, b} {
		if payload, ok := body.payload.(facetedPayload); ok {
			points = append(points, payload.verts...)
			continue
		}
		geom, ok, err := newBodyGeomBudget(budget, body)
		if err != nil {
			return 0, err
		}
		if ok {
			points = append(points, geom.supports...)
			continue
		}
		for _, vertex := range body.Vertices() {
			if err := budget.step(); err != nil {
				return 0, err
			}
			points = append(points, vertex.position)
		}
	}
	best := 0.0
	for i := range points {
		for j := i + 1; j < len(points); j++ {
			if err := budget.step(); err != nil {
				return 0, err
			}
			best = math.Max(best, points[i].Sub(points[j]).Len())
		}
	}
	return best, ctx.Err()
}

// interferenceToleranceRef applies the pair-local volume gate and returns the
// reference it formed (verification §1.1's Required). The overlap boundary lies
// on the operands' skins, so the noise quantum uses their summed surface areas
// rather than the transient intersection mesh.
func interferenceToleranceRef(volume Measurement, a, b *Body, pairD, rel float64) (bool, float64, bool) {
	const eps = 1e-9
	area := math.Abs(a.area.Value.Base()) + math.Abs(b.area.Value.Base())
	quantum := eps * pairD * area
	return scalarToleranceRef(volume, rel, func(value float64) (float64, bool) {
		ref := math.Max(value, quantum)
		return ref, usableMagnitude(ref)
	})
}
