package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/param"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestDocumentParams(t *testing.T) {
	t.Run("a document owns a table when none is given", func(t *testing.T) {
		d := decad.New()
		require.NoError(t, d.Params().SetExpr("module", "2", units.Millimeter))
		require.NoError(t, d.Params().SetExpr("pitch", "6.2831853071795862", units.Millimeter))
		require.NoError(t, d.Params().SetExpr("root", "pitch - 1.25 * module", units.Millimeter))
		v, err := d.Param("root")
		require.NoError(t, err)
		require.InDelta(t, 6.2831853071795862-2.5, v.Base(), 1e-12)
		require.Equal(t, units.Length, v.Kind())
	})

	t.Run("the World table is shared", func(t *testing.T) {
		w := sketch.NewWorld()
		d := decad.New(decad.WithParams(w.Params()), decad.ExactOnly())
		require.Same(t, w.Params(), d.Params())
		require.NoError(t, w.Params().SetExpr("h", "5", units.Millimeter))
		v, err := d.Param("h")
		require.NoError(t, err)
		require.InDelta(t, 5, v.Base(), 0)
	})

	t.Run("an edit is seen by the next read", func(t *testing.T) {
		d := decad.New()
		require.NoError(t, d.Params().SetExpr("a", "10", units.Millimeter))
		require.NoError(t, d.Params().SetExpr("b", "a * 2", units.Millimeter))
		v, err := d.Param("b")
		require.NoError(t, err)
		require.InDelta(t, 20, v.Base(), 0)
		require.NoError(t, d.Params().SetExpr("a", "15", units.Millimeter))
		v, err = d.Param("b")
		require.NoError(t, err)
		require.InDelta(t, 30, v.Base(), 0)
	})

	t.Run("errors are the table's", func(t *testing.T) {
		d := decad.New()
		_, err := d.Param("missing")
		require.ErrorIs(t, err, param.ErrUndefined)
		require.NoError(t, d.Params().Set("x", "y"))
		require.NoError(t, d.Params().Set("y", "x"))
		_, err = d.Param("x")
		require.ErrorIs(t, err, param.ErrCycle)
		require.Empty(t, d.Bindings(), "a failed read records nothing")
	})

	t.Run("Expr evaluates an ad-hoc expression in the base unit", func(t *testing.T) {
		d := decad.New()
		require.NoError(t, d.Params().SetExpr("w", "40", units.Millimeter))
		v, err := d.Expr("w / 4")
		require.NoError(t, err)
		require.Equal(t, units.Length, v.Kind())
		require.InDelta(t, 10, v.Base(), 0)
	})

	t.Run("ParamInt takes only whole dimensionless values", func(t *testing.T) {
		d := decad.New()
		require.NoError(t, d.Params().Set("teeth", "17"))
		require.NoError(t, d.Params().Set("half", "teeth / 2"))
		require.NoError(t, d.Params().SetExpr("len", "3", units.Millimeter))
		n, err := d.ParamInt("teeth")
		require.NoError(t, err)
		require.Equal(t, 17, n)
		_, err = d.ParamInt("half")
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = d.ParamInt("len")
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})

	t.Run("Bindings lists reads in order and is a copy", func(t *testing.T) {
		d := decad.New()
		require.NoError(t, d.Params().SetExpr("m", "2", units.Millimeter))
		require.NoError(t, d.Params().SetExpr("p", "m * 3", units.Millimeter))
		_, err := d.Param("p")
		require.NoError(t, err)
		_, err = d.Expr("m + m")
		require.NoError(t, err)
		got := d.Bindings()
		require.Len(t, got, 2)
		require.Equal(t, "p", got[0].Name)
		require.Equal(t, "m * 3", got[0].Expr)
		require.Equal(t, []string{"m"}, got[0].Dependencies)
		require.InDelta(t, 6, got[0].Value.Base(), 0)
		require.Empty(t, got[1].Name)
		require.Equal(t, "m + m", got[1].Expr)
		got[0].Dependencies[0] = "changed"
		require.Equal(t, []string{"m"}, d.Bindings()[0].Dependencies)
	})
}
