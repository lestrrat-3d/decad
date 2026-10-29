package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The point-cone arm accepts a Cut or Intersect of an unplaced point-Loft by
// an exact cone, and a later cone only where the limits stay ordered: a Cut
// limit sits strictly later than the previous Cut limit, an Intersect limit
// strictly earlier than the previous Intersect limit, and the Cut limit stays
// before the Intersect limit.
func TestPointConeBooleanGates(t *testing.T) {
	doc := New()
	tooth := pointLoftTooth(t, doc)
	d := documentHandle(tooth.n.Doc)
	run := func(op meshbool.OperationKind, target, tool *topo.Body) (*topo.Body, bool, error) {
		t.Helper()
		out, ok, err := revolveplan.PointConeBoolean(t.Context(), op, envOf(documentNode(d), d.nextProducerID()),
			target, tool, false)
		return out, ok, err
	}
	cone := func(apex float64) *topo.Body { return bodyNode(pointLoftConeTool(t, doc, apex)) }

	base := bodyNode(tooth)
	_, ok, err := run(meshbool.OpUnion, base, cone(5))
	require.NoError(t, err)
	require.False(t, ok, "a Union is not a trim")
	_, ok, err = run(meshbool.OpCut, base, &topo.Body{})
	require.NoError(t, err)
	require.False(t, ok, "the tool is no cone")
	_, ok, err = run(meshbool.OpCut, &topo.Body{}, cone(5))
	require.NoError(t, err)
	require.False(t, ok, "the target is no faceted body")

	placed := &topo.Body{Payload: base.Payload}
	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)
	fp := placed.Payload.(facetedPayload)
	fp.Xform = move
	placed.Payload = fp
	_, ok, err = run(meshbool.OpCut, placed, cone(5))
	require.NoError(t, err)
	require.False(t, ok, "a placed point-Loft is not the class")

	noSource := &topo.Body{Payload: base.Payload}
	fp = noSource.Payload.(facetedPayload)
	fp.PointSection, fp.PointCone = nil, nil
	noSource.Payload = fp
	_, ok, err = run(meshbool.OpCut, noSource, cone(5))
	require.NoError(t, err)
	require.False(t, ok, "a body with no point-section record is not the class")

	offApex := &topo.Body{Payload: base.Payload}
	fp = offApex.Payload.(facetedPayload)
	section := *fp.PointSection
	section.Apex = r3.Vec{X: 1}
	fp.PointSection = &section
	offApex.Payload = fp
	_, ok, err = run(meshbool.OpCut, offApex, cone(5))
	require.NoError(t, err)
	require.False(t, ok, "the apex must sit at the origin")

	for _, first := range []struct {
		op   meshbool.OperationKind
		apex float64
		then map[[2]any]bool
	}{
		{meshbool.OpCut, 5, map[[2]any]bool{
			{meshbool.OpCut, 2.0}: false, {meshbool.OpCut, 5.0}: false, {meshbool.OpCut, 8.0}: true, {meshbool.OpCut, 11.0}: true,
			{meshbool.OpIntersect, 2.0}: false, {meshbool.OpIntersect, 5.0}: false,
			{meshbool.OpIntersect, 8.0}: true, {meshbool.OpIntersect, 11.0}: true,
		}},
		{meshbool.OpIntersect, 8, map[[2]any]bool{
			{meshbool.OpCut, 2.0}: true, {meshbool.OpCut, 5.0}: true, {meshbool.OpCut, 8.0}: false, {meshbool.OpCut, 11.0}: false,
			{meshbool.OpIntersect, 2.0}: true, {meshbool.OpIntersect, 5.0}: true,
			{meshbool.OpIntersect, 8.0}: false, {meshbool.OpIntersect, 11.0}: false,
		}},
	} {
		trimmed, ok, err := run(first.op, base, cone(first.apex))
		require.NoError(t, err)
		require.True(t, ok)
		for next, want := range first.then {
			nextOp, nextApex := next[0].(meshbool.OperationKind), next[1].(float64)
			_, ok, err := run(nextOp, trimmed, cone(nextApex))
			label := "after %v %v, then %v %v"
			empty := first.op == meshbool.OpCut && nextOp == meshbool.OpIntersect && nextApex == 2 ||
				first.op == meshbool.OpIntersect && nextOp == meshbool.OpCut && nextApex == 11
			if empty {
				require.True(t, ok, "the exact empty proof is an admitted result")
				var expected *meshbool.BooleanExpectedError
				require.ErrorAs(t, err, &expected, label, first.op, first.apex, next[0], next[1])
				require.Equal(t, meshbool.BooleanExpectedEmpty, expected.Kind)
				require.ErrorIs(t, err, ErrBooleanFailed)
				continue
			}
			require.NoError(t, err, label, first.op, first.apex, next[0], next[1])
			require.Equal(t, want, ok, label, first.op, first.apex, next[0], next[1])
		}
	}
}
