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
		if pa.sectionDelta != 0 || pb.sectionDelta != 0 {
			// Equal records are a certificate of equal SETS only while each
			// record IS the set it denotes. A payload carrying a section
			// displacement (docs/prism-boolean-design.md §7) denotes a set its
			// record is only within that displacement of, and two such records
			// being equal says nothing about the two sets. Undecided.
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
	if err := wallBudgetStep(budget); err != nil {
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
		if err := wallBudgetStep(budget); err != nil {
			return false, err
		}
		if !reflect.DeepEqual(a.Segments[i], b.Segments[i]) {
			return false, nil
		}
	}
	return true, nil
}

// interferenceOutcome distinguishes why the overlap volume could not be
// measured, so Verify can pick the matching diagnostic (verification §1.1).
// Payload staging retains the first operand that failed; contact policy and
// in-pipeline reach remain separate from it and from an undecided proof.
type interferenceOutcome int

const (
	// interferenceMeasured — a positive bounded overlap volume was proven.
	interferenceMeasured interferenceOutcome = iota
	// interferenceUndecided — the read-only proof resolved neither way.
	interferenceUndecided
	// interferenceUnsupportedPayloadFirst — the first operand cannot enter the
	// read-only intersection pipeline.
	interferenceUnsupportedPayloadFirst
	// interferenceUnsupportedPayloadSecond — the second operand cannot enter
	// the read-only intersection pipeline.
	interferenceUnsupportedPayloadSecond
	// interferenceUnsupportedContact — the exact boolean contact policy refused
	// the pair.
	interferenceUnsupportedContact
	// interferenceUnsupportedPipeline — both operands entered the pipeline, but
	// later geometry exceeded its supported reach.
	interferenceUnsupportedPipeline
)

// measuredInterference returns the pair's bounded overlap volume. Strict
// containment and exact analytic equality prove the set identity directly.
// An admitted coplanar, co-directional prism pair next resolves through the
// same read-only analytic OpIntersect dispatch performBoolean uses
// (evaluateAnalyticIntersect, docs/prism-boolean-design.md §14 PR4;
// docs/interference-design.md §5.2) — never consuming either operand. That
// path publishes the resolved intersection payload's OWN volume measurement,
// so the row reads Exact with a zero bound only where neither operand's
// section carries a displacement; an operand whose section does
// (docs/prism-boolean-design.md §7) makes the row Approximate over the
// composed displacement the payload publishes (§8), which the report's
// tolerance gate then judges like any other reading. Every other pair falls
// back to the read-only mesh intersection unchanged. Both
// paths share §6's positive lower-volume gate (positiveVolume). The outcome
// names why an unmeasured result is unmeasured.
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

	analytic, ok, err := evaluateAnalyticIntersect(ctx, a, b)
	if err != nil {
		if expected, ok := asExpectedBoolean(err); ok {
			return Measurement{}, interferenceOutcomeForExpected(expected), nil
		}
		return Measurement{}, interferenceUndecided, err
	}
	if ok {
		if !positiveVolume(analytic.volume) {
			return Measurement{}, interferenceUndecided, nil
		}
		return analytic.volume, interferenceMeasured, nil
	}

	eval, err := evaluateBoolean(ctx, OpIntersect, a, b)
	if err != nil {
		if expected, ok := asExpectedBoolean(err); ok {
			return Measurement{}, interferenceOutcomeForExpected(expected), nil
		}
		return Measurement{}, interferenceUndecided, err
	}
	if !positiveVolume(eval.volume) {
		return Measurement{}, interferenceUndecided, nil
	}
	return eval.volume, interferenceMeasured, nil
}

// positiveVolume is interference design §6's positive-volume gate: the true
// overlap volume proves positive only when the measurement's own proven
// interval [value-bound, value+bound] excludes zero. Both the analytic and
// mesh OpIntersect paths apply it to their own result before
// measuredInterference reports it as measured.
func positiveVolume(v Measurement) bool {
	value := math.Abs(v.Value.Base())
	return value-v.Bound.Base() > 0
}

// interferenceOutcomeForExpected preserves the read-only boolean's private
// reason taxonomy for Verify's cause-specific diagnostics.
func interferenceOutcomeForExpected(expected *booleanExpectedError) interferenceOutcome {
	switch expected.kind {
	case booleanExpectedContact:
		return interferenceUnsupportedContact
	case booleanExpectedUnsupported:
		return interferenceUnsupportedPipeline
	case booleanExpectedStaging:
		if expected.operand == 1 {
			return interferenceUnsupportedPayloadSecond
		}
		return interferenceUnsupportedPayloadFirst
	default:
		return interferenceUndecided
	}
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
