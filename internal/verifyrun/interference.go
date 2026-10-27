package verifyrun

import (
	"context"
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	vocab "github.com/lestrrat-3d/decad/report"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/boundaryaudit"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
// cupwall.Payload.Thickness and cupwall.Payload.Sense, because it does not change set
// identity.
func analyticBodiesEqual(budget *proofbound.WorkBudget, a, b *topo.Body) (bool, error) {
	switch pa := a.Payload.(type) {
	case prismrecord.Payload:
		if pb, ok := b.Payload.(prismrecord.Payload); ok {
			return pa.SameSet(budget, pb)
		}
	case cupwall.Payload:
		if other, ok := b.Payload.(cupwall.Payload); ok {
			return pa.View().SameSet(budget, other.View())
		}
	case revolveplan.Payload:
		if pb, ok := b.Payload.(revolveplan.Payload); ok {
			return pa.SameSet(budget, pb)
		}
	}
	return false, nil
}

// MeasuredInterference returns the pair's bounded overlap volume. Strict
// containment and exact analytic equality prove the set identity directly.
// An admitted coplanar, co-directional prism pair next resolves through the
// same read-only analytic meshbool.OpIntersect dispatch performBoolean uses
// (booleandispatch.AnalyticIntersect, agent-docs/prism-boolean-design.md §14 PR4;
// agent-docs/interference-design.md §5.2) — never consuming either operand. That
// path publishes the resolved intersection payload's OWN volume measurement,
// so the row reads Exact with a zero bound only where neither operand's
// section carries a displacement; an operand whose section does
// (agent-docs/prism-boolean-design.md §7) makes the row Approximate over the
// composed displacement the payload publishes (§8), which the report's
// tolerance gate then judges like any other reading. A pair that twin does
// not admit next reaches §4.5's overlap-area reading (prismOverlapVolume,
// prism_overlap.go), which measures a selection covering any number of
// disjoint regions and publishes a volume with no body at all — the
// two-step order is what keeps every pair the twin already answers
// byte-identical (agent-docs/prism-boolean-design.md §12's Interference row).
// Every pair neither analytic path admits falls back to the read-only mesh
// intersection unchanged. All three paths share §6's positive lower-volume
// gate (positiveVolume). The outcome names why an unmeasured result is
// unmeasured.
//
// meshes supplies the mesh path's operand meshes: Verify's per-call cache,
// or pairMeshes{} where a caller measures one pair on its own.
func MeasuredInterference(ctx context.Context, alloc topo.Alloc, a, b *topo.Body, res pairkernel.Result, meshes booleaneval.OperandMesher) (vocab.Measurement, boundaryaudit.InterferenceOutcome, error) {
	if res.Containment != nil {
		return res.Containment.Inner.Volume, boundaryaudit.InterferenceMeasured, nil
	}
	equal, err := analyticBodiesEqual(proofbound.NewWorkBudget(ctx), a, b)
	if err != nil {
		return vocab.Measurement{}, boundaryaudit.InterferenceUndecided, err
	}
	if equal {
		// Stable pair order chooses A when the represented sets are equal.
		return a.Volume, boundaryaudit.InterferenceMeasured, nil
	}

	analyticNode, ok, err := booleandispatch.AnalyticIntersect(ctx, topo.Env{Doc: a.Doc, Ref: a.Doc.NextProducer, Alloc: alloc}, a, b)
	if err != nil {
		if expected, ok := asExpectedBoolean(err); ok {
			return vocab.Measurement{}, boundaryaudit.InterferenceOutcomeForExpected(expected), nil
		}
		return vocab.Measurement{}, boundaryaudit.InterferenceUndecided, err
	}
	if ok {
		if !positiveVolume(analyticNode.Volume) {
			return vocab.Measurement{}, boundaryaudit.InterferenceUndecided, nil
		}
		return analyticNode.Volume, boundaryaudit.InterferenceMeasured, nil
	}

	overlap, ok, err := prismOverlapVolume(ctx, alloc, a, b)
	if err != nil {
		if expected, ok := asExpectedBoolean(err); ok {
			return vocab.Measurement{}, boundaryaudit.InterferenceOutcomeForExpected(expected), nil
		}
		return vocab.Measurement{}, boundaryaudit.InterferenceUndecided, err
	}
	if ok {
		if !positiveVolume(overlap) {
			return vocab.Measurement{}, boundaryaudit.InterferenceUndecided, nil
		}
		return overlap, boundaryaudit.InterferenceMeasured, nil
	}

	if overlap, ok := orthogonalOverlapVolume(a, b); ok {
		if !positiveVolume(overlap) {
			return vocab.Measurement{}, boundaryaudit.InterferenceUndecided, nil
		}
		return overlap, boundaryaudit.InterferenceMeasured, nil
	}

	eval, err := evaluateBooleanMeshes(ctx, meshbool.OpIntersect, a, b, meshes, 1)
	if err != nil {
		if expected, ok := asExpectedBoolean(err); ok {
			return vocab.Measurement{}, boundaryaudit.InterferenceOutcomeForExpected(expected), nil
		}
		return vocab.Measurement{}, boundaryaudit.InterferenceUndecided, err
	}
	if !positiveVolume(eval.volume) {
		return vocab.Measurement{}, boundaryaudit.InterferenceUndecided, nil
	}
	return eval.volume, boundaryaudit.InterferenceMeasured, nil
}

// positiveVolume is interference design §6's positive-volume gate: the true
// overlap volume proves positive only when the measurement's own proven
// interval [value-bound, value+bound] excludes zero. Both the analytic and
// mesh meshbool.OpIntersect paths apply it to their own result before
// MeasuredInterference reports it as measured.
func positiveVolume(v vocab.Measurement) bool {
	value := math.Abs(v.Value.Base())
	return value-v.Bound.Base() > 0
}

// InterferenceToleranceRef applies the pair-local volume gate and returns the
// reference it formed (verification §1.1's Required). The overlap boundary lies
// on the operands' skins, so the noise quantum uses their summed surface areas
// rather than the transient intersection mesh.
func InterferenceToleranceRef(volume vocab.Measurement, a, b *topo.Body, pairD, rel float64) (bool, float64, bool) {
	const eps = 1e-9
	area := math.Abs(a.Area.Value.Base()) + math.Abs(b.Area.Value.Base())
	quantum := eps * pairD * area
	return tolerance.Scalar(volume.Value, volume.Bound, rel, func(value float64) (float64, bool) {
		ref := math.Max(value, quantum)
		return ref, tolerance.UsableMagnitude(ref)
	})
}

// asExpectedBoolean reports whether err is the mesh boolean's expected
// outcome (an empty, contact or staging result), not an invariant failure.
func asExpectedBoolean(err error) (*meshbool.BooleanExpectedError, bool) {
	var expected *meshbool.BooleanExpectedError
	return expected, errors.As(err, &expected)
}

// meshEvaluation is the geometry-only result of one mesh boolean.
type meshEvaluation struct {
	volume vocab.Measurement
}

// evaluateBooleanMeshes runs the mesh-boolean pipeline (internal/booleaneval,
// agent-docs/evaluator-design.md §9) over a and b, with the operand meshes drawn
// from meshes, and states its occupied volume as a measurement. Verify passes
// its per-call cache (verifyMeshCache), which meshes each body once at one
// chord for every pair it takes part in (agent-docs/interference-design.md §5.3).
func evaluateBooleanMeshes(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body,
	meshes booleaneval.OperandMesher, chordScale float64) (meshEvaluation, error) {
	ev, err := booleaneval.Evaluate(ctx, op, a, b, meshes, chordScale, nil, nil)
	if err != nil {
		return meshEvaluation{}, err
	}
	return meshEvaluation{volume: vocab.Measurement{
		Value:     units.CubicMillimeters(ev.Volume),
		Exactness: measurement.ExactnessOf(ev.VolumeBound),
		Bound:     units.CubicMillimeters(ev.VolumeBound),
	}}, nil
}

// prismOverlapVolume is agent-docs/prism-boolean-design.md §4.5's overlap-area
// reading (prismcells.OverlapVolume) as a measurement. Each cell's scratch
// body takes a fresh producer identity from a's document.
func prismOverlapVolume(ctx context.Context, alloc topo.Alloc, a, b *topo.Body) (vocab.Measurement, bool, error) {
	sum, bound, ok, err := prismcells.OverlapVolume(ctx, a.Payload, b.Payload, func() topo.Env {
		return topo.Env{Doc: a.Doc, Ref: a.Doc.NextProducer, Alloc: alloc}
	})
	if err != nil || !ok {
		return vocab.Measurement{}, false, err
	}
	return vocab.Measurement{
		Value:     units.CubicMillimeters(sum),
		Exactness: measurement.ExactnessOf(bound),
		Bound:     units.CubicMillimeters(bound),
	}, true, nil
}

// orthogonalOverlapVolume reads the volume two faceted bodies share, each of which has every face square to a world axis
// and no vertex bound, exactly on their common grid (agent-docs/orthogonal-boolean-design.md): the sum of the cells
// both occupy, with the bound of its own rounding. It also reads a pair that shares a face plane, which the mesh route
// refuses as contact. ok is false when either body is outside that class, and the mesh route takes the pair.
func orthogonalOverlapVolume(a, b *topo.Body) (vocab.Measurement, bool) {
	pa, okA := a.Payload.(facetedrecord.Payload)
	pb, okB := b.Payload.(facetedrecord.Payload)
	if !okA || !okB || !zeroBound(pa) || !zeroBound(pb) {
		return vocab.Measurement{}, false
	}
	sum, bound, err := orthoshell.OverlapVolume(pa.Verts, pa.Tris, pb.Verts, pb.Tris)
	if err != nil {
		return vocab.Measurement{}, false
	}
	return vocab.Measurement{
		Value:     units.CubicMillimeters(sum),
		Exactness: measurement.ExactnessOf(bound),
		Bound:     units.CubicMillimeters(bound),
	}, true
}

// zeroBound reports whether every vertex of the faceted payload is held exactly.
func zeroBound(pl facetedrecord.Payload) bool {
	for _, beta := range pl.VertexBound {
		if beta != 0 {
			return false
		}
	}
	return true
}
