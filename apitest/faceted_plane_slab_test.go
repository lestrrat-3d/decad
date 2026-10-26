package apitest_test

import (
	"errors"
	"math"
	"regexp"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Increment 5 of agent-docs/faceted-source-retention-design.md §11: a retained
// plane facet states its true piece's distance from its own plane, so a second
// cut that lands on the first cut's rim facets is admitted.

func slabYTool(t *testing.T, sc *retentionScene, cx, r float64) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XZ())
	require.NoError(t, err)
	c := s.CreatePoint(cx, 10)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	cyl, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(60), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Y: 30})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), cyl, move)
	require.NoError(t, err)
	return placed
}

func TestFacetedPlaneSlab(t *testing.T) {
	t.Parallel()

	for _, tc := range []struct {
		name   string
		cx, r  float64
		radius float64
	}{
		{"a hole beside the first hole", 30, 4, 4},
		{"a thin hole beside the first hole", 28.5, 3, 4},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			sc := newRetentionScene()
			first, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
			require.NoError(t, err)
			second, err := decad.Cut(t.Context(), first, slabYTool(t, sc, tc.cx, tc.r))
			require.NoError(t, err)

			// Both holes run the full 20 mm through y and do not touch, so
			// the part loses 20 mm times the area of the two discs.
			discs := math.Pi*tc.radius*tc.radius + math.Pi*tc.r*tc.r
			want := (800-4*(1-math.Pi/4)*9)*20 - 20*discs
			got, err := second.Volume()
			require.NoError(t, err)
			require.InDelta(t, want, got.Value.Mag(), got.Bound.Mag()+1e-6)
			require.Less(t, got.Bound.Mag()/got.Value.Mag(), 1e-3, "the volume bound stays usable")

			// A vertex where the new wall meets a face of the part states a
			// bound that covers its distance from the true circle.
			checked := 0
			for _, v := range second.Vertices() {
				p := v.Position()
				if math.Abs(p.Value.Y) > 1e-9 && math.Abs(p.Value.Y-20) > 1e-9 {
					continue
				}
				off := math.Abs(math.Hypot(p.Value.X-tc.cx, p.Value.Z-10) - tc.r)
				if off > 0.05 {
					continue
				}
				checked++
				require.LessOrEqual(t, off, p.Bound.Mag()+1e-9, "vertex %v", p.Value)
				// The plane facets it was cut from hold 0.00297 mm at the first
				// hole's rim; their slab is far thinner, so the rim states a
				// bound below that.
				require.Less(t, p.Bound.Mag(), 0.00297, "vertex %v", p.Value)
			}
			require.Positive(t, checked, "the new wall leaves rim vertices on the faces")
		})
	}

	t.Run("a contact on a curved rim still refuses", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		built, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		// A Copy ends the operation history's Boolean chain, so the refusal under test is
		// the sequential gate, not a chain the regroup rebuilds.
		first, err := decad.Copy(t.Context(), built)
		require.NoError(t, err)
		held, err := mustTessellate(t, first, units.Millimeters(1)).Bound().In(units.Millimeter)
		require.NoError(t, err)
		// The tool's wall crosses the first hole's wall, whose facets hold
		// the rim's bound: a cylinder states no thinner slab.
		_, err = decad.Cut(t.Context(), first, slabYTool(t, sc, 22, 3))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		var be *decad.BooleanError
		require.False(t, errors.As(err, &be), "the chain-depth refusal is a plain ErrUnsupported, not a BooleanError")
		require.ErrorContains(t, err, "a boolean takes no tolerance")
		require.ErrorContains(t, err, "cut's target")
		m := regexp.MustCompile(`within a bound of (\S+) mm, coarser than this pair's chord tolerance (\S+) mm`).FindStringSubmatch(err.Error())
		require.Len(t, m, 3, "the refusal quotes the touched bound and the pair's chord tolerance")
		touched, err := strconv.ParseFloat(m[1], 64)
		require.NoError(t, err)
		tol, err := strconv.ParseFloat(m[2], 64)
		require.NoError(t, err)
		require.Greater(t, touched, tol)
		require.LessOrEqual(t, touched, held, "a touched facet's bound is one of the target's own")
	})
}
