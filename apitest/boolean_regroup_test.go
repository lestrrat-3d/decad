package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// regroupTilted is D4's second tool: t2 rotated 0.01 rad about the x axis through (22, 10, 10).
func regroupTilted(t *testing.T, sc *retentionScene) *decad.Body {
	t.Helper()
	tr, err := r3.RotationAround(r3.NewVec(22, 10, 10), r3.NewVec(1, 0, 0), units.Radians(0.01))
	require.NoError(t, err)
	b, err := decad.Transform(t.Context(), slabYTool(t, sc, 22, 3), tr)
	require.NoError(t, err)
	return b
}

// requireRegroupHistory checks that result is the only output of the last record, which
// consumed a and b with the given roles, and that each leaf behind them stays consumed
// only by the earlier Boolean that took it.
// regroupCutRoles are the roles a Cut records for its inputs.
var regroupCutRoles = [2]string{regroupTargetRole, "tool"}

const regroupTargetRole = "target"

func requireRegroupHistory(t *testing.T, doc *decad.Document, kind decad.OperationKind, roles [2]string,
	a, b, result *decad.Body, leaves []*decad.Body) {
	t.Helper()
	h := doc.OperationHistory()
	ops := h.Operations()
	last := ops[len(ops)-1]
	require.Equal(t, kind, last.Kind())
	in := last.Inputs()
	require.Len(t, in, 2)
	require.Same(t, a, in[0].Body)
	require.Same(t, b, in[1].Body)
	require.Equal(t, roles, [2]string{in[0].Role, in[1].Role})
	require.Equal(t, decad.InputConsumed, in[0].Use)
	require.Equal(t, decad.InputConsumed, in[1].Use)
	require.Equal(t, []*decad.Body{result}, last.Outputs())
	for _, leaf := range leaves {
		consumers, err := h.Consumers(leaf)
		require.NoError(t, err)
		require.Len(t, consumers, 1, "a leaf is consumed once, by the earlier Boolean")
		require.NotSame(t, last, consumers[0])
	}
	live := doc.Bodies()
	require.Contains(t, live, result)
	require.NotContains(t, live, a)
	require.NotContains(t, live, b)
}

func requireRegroupSolid(t *testing.T, got *decad.Body) {
	t.Helper()
	require.True(t, got.IsSolid())
	require.Len(t, got.Lumps(), 1)
}

func requireRefusalUnchanged(t *testing.T, doc *decad.Document, err error, historyLen int, bodies []*decad.Body) {
	t.Helper()
	require.ErrorIs(t, err, decad.ErrUnsupported)
	var failure decad.OperationDiagnosticError
	require.ErrorAs(t, err, &failure)
	diagnostics := failure.OperationDiagnostics()
	require.Len(t, diagnostics, 1)
	require.Equal(t, decad.OpDiagOperandBoundExceeded, diagnostics[0].Code)
	require.Len(t, doc.OperationHistory().Operations(), historyLen)
	require.Equal(t, bodies, doc.Bodies())
}

func TestBooleanRegroupConjunction(t *testing.T) {
	t.Parallel()

	t.Run("F1 two overlapping parallel holes", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		box, t1, t2 := sc.roundedBox(t), sc.tool(t, 20), slabYTool(t, sc, 22, 3)
		first, err := decad.Cut(t.Context(), box, t1)
		require.NoError(t, err)
		got, err := decad.Cut(t.Context(), first, t2)
		require.NoError(t, err)
		requireRegroupSolid(t, got)
		requireRegroupResult(t, got, regroupBoxVolume-20*(25*math.Pi-regroupLens(4, 3, 2)))
		requireRegroupHistory(t, sc.doc, decad.OperationCut, regroupCutRoles, first, t2, got,
			[]*decad.Body{box, t1})

		// The only vertices on the faces are the two crossings of the holes' rims and the second
		// hole's far point. A vertex within 0.05 mm of a hole's rim states a bound that
		// covers its distance from the true circle.
		both := 0
		for _, v := range got.Vertices() {
			p := v.Position()
			if math.Abs(p.Value.Y) > 1e-9 && math.Abs(p.Value.Y-20) > 1e-9 {
				continue
			}
			onRims := 0
			for _, rim := range []struct{ cx, r float64 }{{20, 4}, {22, 3}} {
				off := math.Abs(math.Hypot(p.Value.X-rim.cx, p.Value.Z-10) - rim.r)
				if off > 0.05 {
					continue
				}
				onRims++
				require.LessOrEqual(t, off, p.Bound.Mag()+1e-9, "vertex %v", p.Value)
			}
			if onRims == 2 {
				both++
			}
		}
		require.Equal(t, 4, both, "both rim crossings on each face lie on both rims")
	})

	t.Run("F2 a straight hole into an earlier hole", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		box, t1, t3 := sc.roundedBox(t), sc.tool(t, 20), regroupCylinderZ(t, sc)
		first, err := decad.Cut(t.Context(), box, t1)
		require.NoError(t, err)
		got, err := decad.Cut(t.Context(), first, t3)
		require.NoError(t, err)
		requireRegroupSolid(t, got)
		requireRegroupResult(t, got, regroupBoxVolume-320*math.Pi-180*math.Pi+regroupCrossVolume(4096))
		requireRegroupHistory(t, sc.doc, decad.OperationCut, regroupCutRoles, first, t3, got,
			[]*decad.Body{box, t1})
	})

	t.Run("F4 an analytic intersect after a mesh cut", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		box, t1, c := sc.roundedBox(t), sc.tool(t, 20), regroupSlabC(t, sc)
		first, err := decad.Cut(t.Context(), box, t1)
		require.NoError(t, err)
		got, err := decad.Intersect(t.Context(), first, c)
		require.NoError(t, err)
		requireRegroupSolid(t, got)
		requireRegroupResult(t, got, 2560-16*(32*math.Pi/3+4*math.Sqrt(3)))
		requireRegroupHistory(t, sc.doc, decad.OperationIntersect, [2]string{"a", "b"}, first, c, got,
			[]*decad.Body{box, t1})
	})

	// The declines leave the document as it was and return the sequential refusal.
	t.Run("D1 a Copy is a leaf", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		built, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		cp, err := decad.Copy(t.Context(), built)
		require.NoError(t, err)
		tool := slabYTool(t, sc, 22, 3)
		historyLen := len(sc.doc.OperationHistory().Operations())
		bodies := sc.doc.Bodies()
		_, err = decad.Cut(t.Context(), cp, tool)
		requireRefusalUnchanged(t, sc.doc, err, historyLen, bodies)
	})

	t.Run("D3 matching faces skips the regroup", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		first, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		tool := slabYTool(t, sc, 22, 3)
		historyLen := len(sc.doc.OperationHistory().Operations())
		bodies := sc.doc.Bodies()
		_, err = decad.Cut(t.Context(), first, tool, decad.WithMatchingFaces())
		requireRefusalUnchanged(t, sc.doc, err, historyLen, bodies)
	})

	t.Run("D4 two pending literals decline", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		first, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		tool := regroupTilted(t, sc)
		historyLen := len(sc.doc.OperationHistory().Operations())
		bodies := sc.doc.Bodies()
		_, err = decad.Cut(t.Context(), first, tool)
		requireRefusalUnchanged(t, sc.doc, err, historyLen, bodies)
	})
}
