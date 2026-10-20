package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/tolerance"
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
// cupPayload.Thickness and cupPayload.Sense, because it does not change set
// identity.
func analyticBodiesEqual(budget *proofbound.WorkBudget, a, b *Body) (bool, error) {
	switch pa := a.n.Payload.(type) {
	case prismPayload:
		if pb, ok := b.n.Payload.(prismPayload); ok {
			return pa.SameSet(budget, pb)
		}
	case cupPayload:
		if other, ok := b.n.Payload.(cupPayload); ok {
			return pa.View().SameSet(budget, other.View())
		}
	case revolvePayload:
		if pb, ok := b.n.Payload.(revolvePayload); ok {
			return pa.SameSet(budget, pb)
		}
	}
	return false, nil
}

// measuredInterference returns the pair's bounded overlap volume. Strict
// containment and exact analytic equality prove the set identity directly.
// An admitted coplanar, co-directional prism pair next resolves through the
// same read-only analytic meshbool.OpIntersect dispatch performBoolean uses
// (booleandispatch.AnalyticIntersect, docs/prism-boolean-design.md §14 PR4;
// docs/interference-design.md §5.2) — never consuming either operand. That
// path publishes the resolved intersection payload's OWN volume measurement,
// so the row reads Exact with a zero bound only where neither operand's
// section carries a displacement; an operand whose section does
// (docs/prism-boolean-design.md §7) makes the row Approximate over the
// composed displacement the payload publishes (§8), which the report's
// tolerance gate then judges like any other reading. A pair that twin does
// not admit next reaches §4.5's overlap-area reading (prismOverlapVolume,
// prism_overlap.go), which measures a selection covering any number of
// disjoint regions and publishes a volume with no body at all — the
// two-step order is what keeps every pair the twin already answers
// byte-identical (docs/prism-boolean-design.md §12's Interference row).
// Every pair neither analytic path admits falls back to the read-only mesh
// intersection unchanged. All three paths share §6's positive lower-volume
// gate (positiveVolume). The outcome names why an unmeasured result is
// unmeasured.
//
// meshes supplies the mesh path's operand meshes: Verify's per-call cache,
// or pairMeshes{} where a caller measures one pair on its own.
func measuredInterference(ctx context.Context, a, b *Body, res pairkernel.Result, meshes booleaneval.OperandMesher) (Measurement, reportvocab.InterferenceOutcome, error) {
	if res.Contained != nil {
		return measurementFromInternal(res.Contained.Volume), reportvocab.InterferenceMeasured, nil
	}
	equal, err := analyticBodiesEqual(proofbound.NewWorkBudget(ctx), a, b)
	if err != nil {
		return Measurement{}, reportvocab.InterferenceUndecided, err
	}
	if equal {
		// Stable pair order chooses A when the represented sets are equal.
		return measurementFromInternal(a.n.Volume), reportvocab.InterferenceMeasured, nil
	}

	analyticNode, ok, err := booleandispatch.AnalyticIntersect(ctx, envOf(a.n.Doc, a.n.Doc.NextProducer), &a.n, &b.n)
	if err != nil {
		if expected, ok := asExpectedBoolean(err); ok {
			return Measurement{}, reportvocab.InterferenceOutcomeForExpected(expected), nil
		}
		return Measurement{}, reportvocab.InterferenceUndecided, err
	}
	if ok {
		if !positiveVolume(measurementFromInternal(analyticNode.Volume)) {
			return Measurement{}, reportvocab.InterferenceUndecided, nil
		}
		return measurementFromInternal(analyticNode.Volume), reportvocab.InterferenceMeasured, nil
	}

	overlap, ok, err := prismOverlapVolume(ctx, a, b)
	if err != nil {
		if expected, ok := asExpectedBoolean(err); ok {
			return Measurement{}, reportvocab.InterferenceOutcomeForExpected(expected), nil
		}
		return Measurement{}, reportvocab.InterferenceUndecided, err
	}
	if ok {
		if !positiveVolume(overlap) {
			return Measurement{}, reportvocab.InterferenceUndecided, nil
		}
		return overlap, reportvocab.InterferenceMeasured, nil
	}

	eval, err := evaluateBooleanMeshes(ctx, meshbool.OpIntersect, a, b, meshes, 1)
	if err != nil {
		if expected, ok := asExpectedBoolean(err); ok {
			return Measurement{}, reportvocab.InterferenceOutcomeForExpected(expected), nil
		}
		return Measurement{}, reportvocab.InterferenceUndecided, err
	}
	if !positiveVolume(eval.volume) {
		return Measurement{}, reportvocab.InterferenceUndecided, nil
	}
	return eval.volume, reportvocab.InterferenceMeasured, nil
}

// positiveVolume is interference design §6's positive-volume gate: the true
// overlap volume proves positive only when the measurement's own proven
// interval [value-bound, value+bound] excludes zero. Both the analytic and
// mesh meshbool.OpIntersect paths apply it to their own result before
// measuredInterference reports it as measured.
func positiveVolume(v Measurement) bool {
	value := math.Abs(v.Value.Base())
	return value-v.Bound.Base() > 0
}

// interferenceToleranceRef applies the pair-local volume gate and returns the
// reference it formed (verification §1.1's Required). The overlap boundary lies
// on the operands' skins, so the noise quantum uses their summed surface areas
// rather than the transient intersection mesh.
func interferenceToleranceRef(volume Measurement, a, b *Body, pairD, rel float64) (bool, float64, bool) {
	const eps = 1e-9
	area := math.Abs(a.n.Area.Value.Base()) + math.Abs(b.n.Area.Value.Base())
	quantum := eps * pairD * area
	return tolerance.Scalar(volume.Value, volume.Bound, rel, func(value float64) (float64, bool) {
		ref := math.Max(value, quantum)
		return ref, tolerance.UsableMagnitude(ref)
	})
}
