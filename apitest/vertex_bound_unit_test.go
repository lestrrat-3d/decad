package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestEveryVertexBoundIsALength reads every vertex's published position bound
// in millimetres, on one body of each kind that publishes vertices of its own:
// a prism, a revolve, a brep body (class B's drilled tombstone), a cap-loop
// chamfer and a loft. An Exact vertex's zero bound is a length like any other,
// so the read never fails and never answers a negative number.
//
// Shown-to-fail: with evalBrepContext building its vertices without a
// millimetre bound, every Exact vertex of the brep body publishes a
// dimensionless zero and In fails with "cannot express dimensionless in
// length".
func TestEveryVertexBoundIsALength(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name      string
		body      func(t *testing.T) *decad.Body
		someExact bool
	}{
		{`prism`, func(t *testing.T) *decad.Body { return boxBody(t, decad.New(), 0, 0, 10, 10, 5) }, true},
		{`revolve`, func(t *testing.T) *decad.Body {
			return revolveMeridian(t, decad.New(), shaftMeridian, decad.FullRevolution{})
		}, false},
		{`brep`, func(t *testing.T) *decad.Body {
			body, _ := drilledTombstone(t, decad.New())
			return body
		}, true},
		{`cap-loop chamfer`, func(t *testing.T) *decad.Body {
			_, box := capBlendBox(t)
			chamfered, err := box.Chamfer(t.Context(), capLoopEdges(box), units.Millimeters(1))
			require.NoError(t, err)
			return chamfered
		}, false},
		{`loft`, func(t *testing.T) *decad.Body { return partsBinLoft(t, decad.New()) }, false},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			body := tc.body(t)
			vertices := body.Vertices()
			require.NotEmpty(t, vertices)
			exact := 0
			for _, v := range vertices {
				pos := v.Position()
				bound, err := pos.Bound.In(units.Millimeter)
				require.NoErrorf(t, err, `vertex %v (%v) publishes its bound in millimetres`, pos.Value, pos.Exactness)
				require.GreaterOrEqual(t, bound, 0.0)
				if pos.Exactness == decad.Exact {
					require.Zero(t, bound, `an Exact vertex %v publishes a zero bound`, pos.Value)
					exact++
				}
			}
			if tc.someExact {
				require.Positive(t, exact, `the body holds an Exact vertex, so the zero bound is read`)
			}
		})
	}
}
