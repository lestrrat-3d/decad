package decad

import (
	"context"
	"errors"
	"fmt"
	"testing"

	"github.com/stretchr/testify/require"

	"math/big"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func TestRegroupSurfacesInvariantBreaks(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		err  error
		want bool
	}{
		{"an evaluator failure surfaces", newBooleanError(meshbool.OpCut, BooleanEvaluatorFailure, ErrBooleanFailed, fmt.Errorf("broken")), true},
		{"an empty result is a miss", newBooleanError(meshbool.OpCut, BooleanEmpty, ErrBooleanFailed, fmt.Errorf("empty")), false},
		{"an unsupported contact is a miss", newBooleanError(meshbool.OpCut, BooleanUnsupportedContact, ErrUnsupported, fmt.Errorf("contact")), false},
		{"a bare boolean failure surfaces", fmt.Errorf("%w: raw", ErrBooleanFailed), true},
		{"a plain unsupported is a miss", fmt.Errorf("%w: staging", ErrUnsupported), false},
	} {
		require.Equal(t, tc.want, regroupSurfaces(tc.err), tc.name)
	}
	held := &meshbool.CoarseHeldContactError{Operand: 0}
	staging := meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedStaging, 0, held)
	wrongSide := meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedStaging, 1,
		&meshbool.CoarseHeldContactError{Operand: 1})
	wrongKind := meshbool.ExpectedBooleanForOperand(meshbool.BooleanExpectedContact, 0, held)
	for _, tc := range []struct {
		name string
		op   meshbool.OperationKind
		cfg  booleanConfig
		err  error
		want bool
	}{
		{"target staging", meshbool.OpCut, booleanConfig{}, staging, true},
		{"wrapped target staging", meshbool.OpCut, booleanConfig{}, fmt.Errorf("wrapped: %w", staging), true},
		{"tool staging", meshbool.OpCut, booleanConfig{}, wrongSide, false},
		{"contact", meshbool.OpCut, booleanConfig{}, wrongKind, false},
		{"plain unsupported", meshbool.OpCut, booleanConfig{}, ErrUnsupported, false},
		{"matching faces", meshbool.OpCut, booleanConfig{matchingFaces: true}, staging, false},
		{"union", meshbool.OpUnion, booleanConfig{}, staging, false},
		{"joined cancellation", meshbool.OpCut, booleanConfig{}, errors.Join(staging, context.Canceled), false},
		{"conflicting tags", meshbool.OpCut, booleanConfig{}, errors.Join(staging, wrongSide), false},
	} {
		require.Equal(t, tc.want, coarseTargetCut(tc.op, tc.cfg, tc.err), tc.name)
	}
	t.Run("plan cancellation boundaries", func(t *testing.T) {
		for _, paired := range []bool{false, true} {
			count := 3
			if paired {
				count = 5
			}
			for cancelledAfter := 0; cancelledAfter <= count; cancelledAfter++ {
				ctx, cancel := context.WithCancel(t.Context())
				if cancelledAfter == 0 {
					cancel()
				}
				var bodies [9]topo.Body
				sources := [2]*topo.Body{&bodies[0], &bodies[1]}
				firstTool := &bodies[2]
				var secondTool *topo.Body
				if paired {
					secondTool = &bodies[3]
				}
				calls := 0
				run := func(_ meshbool.OperationKind, _, _ *topo.Body, _ float64) (*topo.Body, error) {
					calls++
					if calls == cancelledAfter {
						cancel()
					}
					return &bodies[3+calls], nil
				}
				_, err := booleandispatch.RunPairedCutPlan(ctx, sources, firstTool, secondTool, run)
				require.ErrorIs(t, err, context.Canceled, "paired=%v boundary=%d", paired, cancelledAfter)
				require.Equal(t, cancelledAfter, calls, "paired=%v boundary=%d", paired, cancelledAfter)
				cancel()
			}
			var bodies [9]topo.Body
			type call struct {
				op    meshbool.OperationKind
				a, b  *topo.Body
				scale float64
			}
			var calls []call
			run := func(op meshbool.OperationKind, a, b *topo.Body, scale float64) (*topo.Body, error) {
				calls = append(calls, call{op, a, b, scale})
				return &bodies[3+len(calls)], nil
			}
			var secondTool *topo.Body
			if paired {
				secondTool = &bodies[3]
			}
			_, err := booleandispatch.RunPairedCutPlan(t.Context(), [2]*topo.Body{&bodies[0], &bodies[1]}, &bodies[2], secondTool, run)
			require.NoError(t, err)
			want := []call{
				{meshbool.OpCut, &bodies[0], &bodies[2], 0.25},
				{meshbool.OpCut, &bodies[1], &bodies[2], 0.25},
				{meshbool.OpUnion, &bodies[4], &bodies[5], 1},
			}
			if paired {
				want = []call{
					{meshbool.OpCut, &bodies[0], &bodies[2], 0.25},
					{meshbool.OpCut, &bodies[4], &bodies[3], 0.125},
					{meshbool.OpCut, &bodies[1], &bodies[2], 0.0625},
					{meshbool.OpCut, &bodies[6], &bodies[3], 0.125},
					{meshbool.OpUnion, &bodies[5], &bodies[7], 1},
				}
			}
			require.Len(t, calls, len(want))
			for i := range want {
				require.Equal(t, want[i].op, calls[i].op)
				require.Same(t, want[i].a, calls[i].a)
				require.Same(t, want[i].b, calls[i].b)
				require.Equal(t, want[i].scale, calls[i].scale)
			}
		}
	})
	t.Run("public ancestry", func(t *testing.T) {
		d := New()
		left := internalBoxBody(t, d, 0, 0, 2, 2, 2)
		right := internalBoxBody(t, d, 1, 0, 3, 2, 2)
		target, err := Union(t.Context(), left, right)
		require.NoError(t, err)
		tool := internalBoxBody(t, d, 0.5, 0.5, 2.5, 1.5, 2)
		originalPayload := target.n.Payload
		target.n.Payload = facetedrecord.Payload{}
		ancestry, ok := pairedCutSources(t.Context(), d, target, tool)
		require.True(t, ok)
		require.Same(t, left, ancestry.sources[0])
		require.Same(t, right, ancestry.sources[1])
		require.Nil(t, ancestry.firstTool)
		beforeHistory := len(d.OperationHistory().Operations())
		beforeBodies := d.Bodies()
		beforeID := d.nextProducerID()
		sources := []*Body{left, right, tool}
		meshes := make([]*topo.TessEntry, len(sources))
		faces := make([][]*Face, len(sources))
		payloads := make([]prismPayload, len(sources))
		for i, source := range sources {
			meshes[i] = source.n.Tess.Load()
			faces[i] = source.Faces()
			payloads[i] = source.n.Payload.(prismPayload)
		}
		first, err := booleanBodyWith(t.Context(), meshbool.OpCut, left, tool, beforeID, booleanConfig{})
		require.NoError(t, err)
		second, err := booleanBodyWith(t.Context(), meshbool.OpCut, right, tool, beforeID+1, booleanConfig{})
		require.NoError(t, err)
		require.Empty(t, first.n.OperationWarnings)
		require.Empty(t, second.n.OperationWarnings)
		joined, err := booleanBodyWith(t.Context(), meshbool.OpUnion, first, second, beforeID+2, booleanConfig{})
		require.NoError(t, err)
		require.Len(t, joined.n.OperationWarnings, 1)
		require.Equal(t, opdiagnostic.CodeGapShiftApplied, joined.n.OperationWarnings[0].Code)
		result, span, _, used, err := pairedCutContinuation(t.Context(), d, target, tool, booleanConfig{}, staging)
		require.NoError(t, err)
		require.False(t, used)
		require.Nil(t, result)
		require.Zero(t, span)
		for i, source := range sources {
			require.True(t, meshes[i] == source.n.Tess.Load())
			currentFaces := source.Faces()
			require.Len(t, currentFaces, len(faces[i]))
			for j, face := range currentFaces {
				require.Same(t, faces[i][j], face)
			}
			payload := source.n.Payload.(prismPayload)
			require.Equal(t, payloads[i].Z0, payload.Z0)
			require.Equal(t, payloads[i].Z1, payload.Z1)
			require.True(t, payloads[i].Walks == payload.Walks)
		}
		require.Equal(t, beforeID, d.nextProducerID())
		require.Len(t, d.OperationHistory().Operations(), beforeHistory)
		cancelled, cancel := context.WithCancel(t.Context())
		cancel()
		err = commitPairedCut(cancelled, d, target, tool, target, 3, beforeID, ancestry, ErrUnsupported)
		require.ErrorIs(t, err, context.Canceled)
		require.Equal(t, beforeID, d.nextProducerID())
		require.Len(t, d.OperationHistory().Operations(), beforeHistory)
		require.Len(t, d.Bodies(), len(beforeBodies))
		for i, body := range d.Bodies() {
			require.Same(t, beforeBodies[i], body)
		}
		op := ancestry.union
		op.inputs[0].Role = operationRoleTool
		_, ok = pairedCutSources(t.Context(), d, target, tool)
		require.False(t, ok)
		err = commitPairedCut(t.Context(), d, target, tool, target, 3, beforeID, ancestry, ErrUnsupported)
		require.ErrorIs(t, err, ErrUnsupported)
		require.Equal(t, beforeID, d.nextProducerID())
		require.Len(t, d.OperationHistory().Operations(), beforeHistory)
		require.Len(t, d.Bodies(), len(beforeBodies))
		for i, body := range d.Bodies() {
			require.Same(t, beforeBodies[i], body)
		}
		op.inputs[0].Role = "first"
		op.inputs[0].Body, op.inputs[1].Body = op.inputs[1].Body, op.inputs[0].Body
		_, ok = pairedCutSources(t.Context(), d, target, tool)
		require.False(t, ok)
		op.inputs[0].Body, op.inputs[1].Body = op.inputs[1].Body, op.inputs[0].Body
		op.inputs[0].Body = tool
		_, ok = pairedCutSources(t.Context(), d, target, tool)
		require.False(t, ok)
		op.inputs[0].Body = left
		op.outputs[0] = tool
		_, ok = pairedCutSources(t.Context(), d, target, tool)
		require.False(t, ok)
		op.outputs[0] = target
		cloned := op.Inputs()
		cloned[0].Body = tool
		_, ok = pairedCutSources(t.Context(), d, target, tool)
		require.True(t, ok)
		err = commitPairedCut(t.Context(), d, target, tool, target, 3, beforeID+1, ancestry, ErrUnsupported)
		require.ErrorIs(t, err, ErrUnsupported)
		require.Equal(t, beforeID, d.nextProducerID())
		require.Len(t, d.OperationHistory().Operations(), beforeHistory)
		originalProducer := d.producers[left]
		delete(d.producers, left)
		_, ok = pairedCutSources(t.Context(), d, target, tool)
		require.False(t, ok)
		d.producers[left] = originalProducer
		target.n.Payload = facetedrecord.Payload{PointCone: &facetedrecord.PointConeTrimRecord{}}
		_, ok = pairedCutSources(t.Context(), d, target, tool)
		require.False(t, ok)
		target.n.Payload = facetedrecord.Payload{}
		target.n.OperationWarnings = []opdiagnostic.Record{{
			Code: opdiagnostic.CodeGapShiftApplied, Change: opdiagnostic.ChangeGapShift,
		}}
		_, ok = pairedCutSources(t.Context(), d, target, tool)
		require.False(t, ok)
		target.n.OperationWarnings = nil
		private, _ := privateConstructionContext(t.Context())
		_, ok = pairedCutSources(private, d, target, tool)
		require.False(t, ok)
		target.n.Payload = originalPayload
		cut, err := Cut(t.Context(), target, tool)
		require.NoError(t, err)
		nextTool := internalBoxBody(t, d, 0.25, 0.25, 2.75, 0.4, 2)
		target.n.Payload = facetedrecord.Payload{}
		cut.n.Payload = facetedrecord.Payload{}
		paired, ok := pairedCutSources(t.Context(), d, cut, nextTool)
		require.True(t, ok)
		require.Same(t, left, paired.sources[0])
		require.Same(t, right, paired.sources[1])
		require.Same(t, tool, paired.firstTool)
		paired.prior.inputs[0].Role = operationRoleTool
		_, ok = pairedCutSources(t.Context(), d, cut, nextTool)
		require.False(t, ok)
	})
}

// TestExactShiftProducerGate calls the history's exact-shift reading
// directly, so a refusal is the gate's own and not a later geometry rule's.
// Only a direct WithCopyLiftTranslation copy of a faceted body reads, as its
// source and lift·(to − from) exactly; a placement, rotation or plain copy, a
// copy of a copy, a Transform of a copy, and a copy of a prism do not.
//
// Leg shown to fail: dropping the IsTranslation check reads the rotation copy,
// and dropping both the Direct and the source checks reads the copy of a copy.
func TestExactShiftProducerGate(t *testing.T) {
	t.Parallel()
	doc := New()
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(30))
	require.NoError(t, err)
	disc, err := Transform(t.Context(), internalDiscBody(t, doc, 1, 6), turn)
	require.NoError(t, err)
	source, err := Union(t.Context(), internalBoxBody(t, doc, 0, -2, 4, 2, 2), disc)
	require.NoError(t, err)
	_, faceted := source.n.Payload.(facetedPayload)
	require.True(t, faceted, "the source must be faceted for an exact copy record")
	prism := internalBoxBody(t, doc, 10, 0, 12, 2, 2)

	lift, err := r3.RotationAround(r3.Vec{X: 3, Y: -2, Z: 1}, r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(0.4))
	require.NoError(t, err)
	from, to := r3.Vec{X: 1}, r3.Vec{X: 21.5, Y: -3, Z: 2}
	exact := WithCopyLiftTranslation(lift, from, to)
	copyOf := func(b *Body, opts ...CopyOption) *Body {
		c, err := Copy(t.Context(), b, opts...)
		require.NoError(t, err)
		return c
	}
	direct := copyOf(source, exact)
	moved, err := r3.Translation(r3.Vec{X: 5})
	require.NoError(t, err)
	h := historyProducers{documentHandle(source.n.Doc)}

	src, shift, ok := h.ExactShiftProducer(&direct.n)
	require.True(t, ok, "a direct exact lift copy reads")
	require.Same(t, &source.n, src)
	b := lift.Basis()
	cols := [3]r3.Vec{b.EX, b.EY, b.EZ}
	delta := [3]*big.Rat{
		new(big.Rat).Sub(new(big.Rat).SetFloat64(to.X), new(big.Rat).SetFloat64(from.X)),
		new(big.Rat).Sub(new(big.Rat).SetFloat64(to.Y), new(big.Rat).SetFloat64(from.Y)),
		new(big.Rat).Sub(new(big.Rat).SetFloat64(to.Z), new(big.Rat).SetFloat64(from.Z)),
	}
	for i := range 3 {
		want := new(big.Rat)
		for j, col := range cols {
			c := [3]float64{col.X, col.Y, col.Z}
			want.Add(want, new(big.Rat).Mul(new(big.Rat).SetFloat64(c[i]), delta[j]))
		}
		require.Zero(t, want.Cmp(shift[i]), "component %d is lift·(to − from) exactly", i)
	}

	transformed, err := Transform(t.Context(), copyOf(source, exact), moved)
	require.NoError(t, err)
	for _, tc := range []struct {
		name string
		body *Body
	}{
		{"plain copy", copyOf(source)},
		{"placement copy", copyOf(source, WithCopyPlacement(moved))},
		{"rotation copy", copyOf(source, WithCopyRotation(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(90)))},
		{"copy of a copy", copyOf(direct, WithCopyLiftTranslation(lift, to, from))},
		{"Transform of a copy", transformed},
		{"lift copy of a prism", copyOf(prism, exact)},
	} {
		_, _, ok := h.ExactShiftProducer(&tc.body.n)
		require.False(t, ok, tc.name)
	}
}
