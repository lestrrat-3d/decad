package decad

import (
	"context"
	"math"
	"reflect"

	"github.com/lestrrat-3d/decad/internal/meshbool"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tolerance"

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
func analyticBodiesEqual(budget *proofbound.WorkBudget, a, b *Body) (bool, error) {
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
		other, ok := b.payload.(cupPayload)
		if !ok {
			return false, nil
		}
		return cupViewsEqual(budget, pa.view(), other.view())
	case revolvePayload:
		pb, ok := b.payload.(revolvePayload)
		if !ok || pa.frame != pb.frame || pa.ax != pb.ax || pa.phi0 != pb.phi0 ||
			pa.phi1 != pb.phi1 || pa.full != pb.full || pa.xform != pb.xform {
			return false, nil
		}
		if pa.sectionDelta != 0 || pb.sectionDelta != 0 {
			// The prism arm's own reasoning: equal records certify equal sets
			// only while each record is the set it denotes.
			return false, nil
		}
		return profileRecordsEqual(budget, pa.profile, pb.profile)
	default:
		return false, nil
	}
}

// cupViewsEqual is analyticBodiesEqual's cup arm over two cup views: one
// frame, one placement, the same three levels and the same two regions.
func cupViewsEqual(budget *proofbound.WorkBudget, pa, pb cupView) (bool, error) {
	if pa.frame != pb.frame || pa.zOpen != pb.zOpen || pa.zOuter != pb.zOuter ||
		pa.zCav != pb.zCav || pa.xform != pb.xform {
		return false, nil
	}
	if pa.offsetDelta != 0 || pb.offsetDelta != 0 {
		// The prism arm's rule for a displaced section: an offset region
		// recorded only within offsetDelta of the one it denotes makes two
		// equal records say nothing about the two sets. Undecided.
		return false, nil
	}
	same, err := profileRecordsEqual(budget, pa.outer, pb.outer)
	if err != nil || !same {
		return false, err
	}
	return profileRecordsEqual(budget, pa.cavity, pb.cavity)
}

// profileRecordsEqual reports exact structural equality of two recorded
// profiles, stepping the budget once per segment compared.
func profileRecordsEqual(budget *proofbound.WorkBudget, a, b ProfileRecord) (bool, error) {
	if err := survey2d.WallBudgetStep(budget); err != nil {
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
func loopRecordsEqual(budget *proofbound.WorkBudget, a, b LoopRecord) (bool, error) {
	if len(a.Segments) != len(b.Segments) || (a.Segments == nil) != (b.Segments == nil) {
		return false, nil
	}
	for i := range a.Segments {
		if err := survey2d.WallBudgetStep(budget); err != nil {
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
	// interferenceUnsupportedVolumeProofFirst — the first operand meshes, but its
	// mesh carries no occupied-volume proof, so the read-only intersection cannot
	// compose it.
	interferenceUnsupportedVolumeProofFirst
	// interferenceUnsupportedVolumeProofSecond — the same for the second operand.
	interferenceUnsupportedVolumeProofSecond
)

// measuredInterference returns the pair's bounded overlap volume. Strict
// containment and exact analytic equality prove the set identity directly.
// An admitted coplanar, co-directional prism pair next resolves through the
// same read-only analytic meshbool.OpIntersect dispatch performBoolean uses
// (evaluateAnalyticIntersect, docs/prism-boolean-design.md §14 PR4;
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
func measuredInterference(ctx context.Context, a, b *Body, res pairResult, meshes operandMeshes) (Measurement, interferenceOutcome, error) {
	if res.contained != nil {
		return res.contained.volume, interferenceMeasured, nil
	}
	equal, err := analyticBodiesEqual(proofbound.NewWorkBudget(ctx), a, b)
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

	overlap, ok, err := prismOverlapVolume(ctx, a, b)
	if err != nil {
		if expected, ok := asExpectedBoolean(err); ok {
			return Measurement{}, interferenceOutcomeForExpected(expected), nil
		}
		return Measurement{}, interferenceUndecided, err
	}
	if ok {
		if !positiveVolume(overlap) {
			return Measurement{}, interferenceUndecided, nil
		}
		return overlap, interferenceMeasured, nil
	}

	eval, err := evaluateBooleanMeshes(ctx, meshbool.OpIntersect, a, b, meshes)
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
// mesh meshbool.OpIntersect paths apply it to their own result before
// measuredInterference reports it as measured.
func positiveVolume(v Measurement) bool {
	value := math.Abs(v.Value.Base())
	return value-v.Bound.Base() > 0
}

// interferenceOutcomeForExpected preserves the read-only boolean's private
// reason taxonomy for Verify's cause-specific diagnostics.
func interferenceOutcomeForExpected(expected *meshbool.BooleanExpectedError) interferenceOutcome {
	switch expected.Kind {
	case meshbool.BooleanExpectedContact:
		return interferenceUnsupportedContact
	case meshbool.BooleanExpectedUnsupported:
		return interferenceUnsupportedPipeline
	case meshbool.BooleanExpectedStaging:
		if expected.Operand == 1 {
			return interferenceUnsupportedPayloadSecond
		}
		return interferenceUnsupportedPayloadFirst
	case meshbool.BooleanExpectedVolumeProof:
		if expected.Operand == 1 {
			return interferenceUnsupportedVolumeProofSecond
		}
		return interferenceUnsupportedVolumeProofFirst
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
	budget := proofbound.NewWorkBudget(ctx)
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
			if err := budget.Step(); err != nil {
				return 0, err
			}
			points = append(points, vertex.position)
		}
	}
	best := 0.0
	for i := range points {
		for j := i + 1; j < len(points); j++ {
			if err := budget.Step(); err != nil {
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
	return tolerance.Scalar(volume.Value, volume.Bound, rel, func(value float64) (float64, bool) {
		ref := math.Max(value, quantum)
		return ref, tolerance.UsableMagnitude(ref)
	})
}
