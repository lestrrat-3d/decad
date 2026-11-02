package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
)

func splitTestBox(t *testing.T, doc *Document, x0, x1 float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(x0, 0, x1, 10)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	return body
}

type cancelOperandMesher struct {
	cancel context.CancelFunc
	called *bool
}

func (m cancelOperandMesher) TessellateOperand(ctx context.Context, _ *topo.Body, _ float64) (*topo.Mesh, bool, error) {
	*m.called = true
	m.cancel()
	return nil, false, ctx.Err()
}

// TestBooleanDispatchAnalyticPrefix pins the split of booleandispatch.Body into the
// analytic route prefix and the mesh tail (agent-docs/boolean-chain-regroup-design.md).
func TestBooleanDispatchAnalyticPrefix(t *testing.T) {
	t.Parallel()

	t.Run("an analytic pair hits", func(t *testing.T) {
		t.Parallel()
		doc := New()
		a, b := splitTestBox(t, doc, 0, 10), splitTestBox(t, doc, 5, 15)
		ctx, err := doc.evalContext(t.Context())
		require.NoError(t, err)
		env := envOf(documentNode(doc), doc.nextProducerID())
		n, ok, err := booleandispatchAnalytic(ctx, meshbool.OpUnion, env, a, b)
		require.NoError(t, err)
		require.True(t, ok)
		_, isPrism := n.Payload.(prismPayload)
		require.True(t, isPrism)
		v, err := bodyHandle(n).Volume()
		require.NoError(t, err)
		mm3, err := v.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.InDelta(t, 1500.0, mm3, v.Bound.Mag()+1e-9)
	})

	t.Run("a mesh-only pair misses and Body takes the mesh tail", func(t *testing.T) {
		t.Parallel()
		doc := New()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		r := s.CreateRectangle(0, 0, 40, 20)
		s.Fix(r.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		prism, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(20), Dir: Along})
		require.NoError(t, err)
		box, err := Fillet(t.Context(), prism, Edges(ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4), units.Millimeters(3))
		require.NoError(t, err)

		ts, err := w.CreateSketch(w.XZ())
		require.NoError(t, err)
		c := ts.CreatePoint(20, 10)
		ts.Fix(c)
		ts.CreateCircle(c, 4)
		_, err = ts.Solve(t.Context())
		require.NoError(t, err)
		cyl, err := Extrude(t.Context(), doc, ts, ts.Profiles()[0], Distance{D: units.Millimeters(60), Dir: Along})
		require.NoError(t, err)
		move, err := r3.Translation(r3.Vec{Y: 30})
		require.NoError(t, err)
		tool, err := Transform(t.Context(), cyl, move)
		require.NoError(t, err)

		ctx, err := doc.evalContext(t.Context())
		require.NoError(t, err)
		env := envOf(documentNode(doc), doc.nextProducerID())
		n, ok, err := booleandispatchAnalytic(ctx, meshbool.OpCut, env, box, tool)
		require.NoError(t, err)
		require.False(t, ok)
		require.Nil(t, n)

		cancelled, cancel := context.WithCancel(ctx)
		called := false
		hooks := booleanHooks(booleanConfig{})
		hooks.Mesher = cancelOperandMesher{cancel: cancel, called: &called}
		beforeID := doc.nextProducerID()
		beforeHistory := len(doc.OperationHistory().Operations())
		beforeBodies := doc.Bodies()
		boxMesh, toolMesh := box.n.Tess.Load(), tool.n.Tess.Load()
		boxFaces, toolFaces := box.Faces(), tool.Faces()
		_, err = booleandispatch.BodyScaled(cancelled, meshbool.OpCut, env, &box.n, &tool.n, hooks, 0.25)
		require.True(t, called)
		require.ErrorIs(t, err, context.Canceled)
		require.Equal(t, beforeID, doc.nextProducerID())
		require.Len(t, doc.OperationHistory().Operations(), beforeHistory)
		require.Len(t, doc.Bodies(), len(beforeBodies))
		for i, body := range doc.Bodies() {
			require.Same(t, beforeBodies[i], body)
		}
		require.True(t, boxMesh == box.n.Tess.Load())
		require.True(t, toolMesh == tool.n.Tess.Load())
		for i, face := range box.Faces() {
			require.Same(t, boxFaces[i], face)
		}
		for i, face := range tool.Faces() {
			require.Same(t, toolFaces[i], face)
		}

		got, err := booleanBodyWith(ctx, meshbool.OpCut, box, tool, doc.nextProducerID(), booleanConfig{})
		require.NoError(t, err)
		_, isFaceted := got.n.Payload.(facetedPayload)
		require.True(t, isFaceted)
		v, err := got.Volume()
		require.NoError(t, err)
		mm3, err := v.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		want := (800-4*(1-math.Pi/4)*9)*20 - 320*math.Pi
		require.InDelta(t, want, mm3, v.Bound.Mag()+1e-6)
	})
}

func booleandispatchAnalytic(ctx context.Context, op meshbool.OperationKind, env topo.Env, a, b *Body) (*topo.Body, bool, error) {
	return booleandispatch.Analytic(ctx, op, env, &a.n, &b.n, booleanHooks(booleanConfig{}))
}
