package apitest_test

import (
	"math"
	"regexp"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// chainedCut returns the error of the second Cut of the curved-rim chain of
// TestFacetedPlaneSlab: its tool's wall crosses the first hole's wall, whose
// facets hold the first Cut's rim bound.
func chainedCut(t *testing.T, opts ...decad.DocumentOption) (*decad.Body, error) {
	t.Helper()
	sc := &retentionScene{w: sketch.NewWorld(), doc: decad.New(opts...)}
	first, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
	require.NoError(t, err)
	return decad.Cut(t.Context(), first, slabYTool(t, sc, 22, 3))
}

func TestChainToleranceAdmitsAContactOnACoarserRim(t *testing.T) {
	t.Parallel()

	_, err := chainedCut(t)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	m := regexp.MustCompile(`within a bound of (\S+) mm, coarser than this pair's chord tolerance (\S+) mm`).FindStringSubmatch(err.Error())
	require.Len(t, m, 3)
	touched, err := strconv.ParseFloat(m[1], 64)
	require.NoError(t, err)

	t.Run("a tolerance below the touched bound still refuses", func(t *testing.T) {
		t.Parallel()
		_, err := chainedCut(t, decad.WithChainTolerance(units.Millimeters(touched/2)))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "a boolean takes no tolerance")
	})

	t.Run("a tolerance at the touched bound admits it", func(t *testing.T) {
		t.Parallel()
		cut, err := chainedCut(t, decad.WithChainTolerance(units.Millimeters(touched*1.01)))
		require.NoError(t, err)
		require.True(t, cut.IsSolid())

		// Two Ø8 and Ø6 holes through a 40x20x20 box with 3 mm corner fillets: the
		// holes overlap, so the removed volume is at most the sum of the two bores.
		got, err := cut.Volume()
		require.NoError(t, err)
		whole := (800 - 4*(1-math.Pi/4)*9) * 20
		discs := math.Pi*4*4 + math.Pi*3*3
		require.Less(t, got.Value.Base(), whole)
		require.Greater(t, got.Value.Base(), whole-20*discs-got.Bound.Base())
		require.Less(t, got.Bound.Base(), 0.05*whole, "the admitted result still states a usable bound")
	})

	t.Run("an invalid tolerance is reported when an operation starts", func(t *testing.T) {
		t.Parallel()
		for _, tc := range []struct {
			tol  units.Value
			want error
		}{
			{units.Millimeters(-1), decad.ErrNegativeMagnitude},
			{units.Degrees(3), decad.ErrUnitKind},
			{units.Millimeters(math.Inf(1)), decad.ErrNotFinite},
		} {
			doc := decad.New(decad.WithChainTolerance(tc.tol))
			box := func(size float64) *decad.Body {
				w := sketch.NewWorld()
				s, err := w.CreateSketch(w.XY())
				require.NoError(t, err)
				r := s.CreateRectangle(0, 0, size, size)
				s.Fix(r.A)
				_, err = s.Solve(t.Context())
				require.NoError(t, err)
				b, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
					decad.Distance{D: units.Millimeters(size), Dir: decad.Along})
				require.NoError(t, err)
				return b
			}
			_, err := decad.Cut(t.Context(), box(4), box(2))
			require.ErrorIs(t, err, tc.want, "%v", tc.tol)
		}
	})
}

// TestBooleanResultKeepsTheHeldFloorOfItsPrimitive trims a certified LoftFaces
// solid with two cones, then cuts the trimmed result with a box. The pair's chord
// tolerance the refusal quotes is the primitive's held bound, not the far smaller
// diameter-derived one: the result still holds the primitive's fixed mesh.
func TestBooleanResultKeepsTheHeldFloorOfItsPrimitive(t *testing.T) {
	doc := decad.New()
	_, tooth := spiralBlankAndTooth(t, doc)
	bb, err := tooth.Bounds()
	require.NoError(t, err)

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	cx := (bb.Min.X + bb.Max.X) / 2
	corners := []*sketch.Point{
		s.CreatePoint(cx-1, bb.Min.Y-5), s.CreatePoint(cx+1, bb.Min.Y-5),
		s.CreatePoint(cx+1, bb.Max.Y+5), s.CreatePoint(cx-1, bb.Max.Y+5),
	}
	for _, p := range corners {
		s.Fix(p)
	}
	for i := range corners {
		s.CreateLine(corners[i], corners[(i+1)%len(corners)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(bb.Max.Z - bb.Min.Z + 10), Dir: decad.Along})
	require.NoError(t, err)

	_, err = decad.Cut(t.Context(), tooth, box)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	m := regexp.MustCompile(`coarser than this pair's chord tolerance (\S+) mm`).FindStringSubmatch(err.Error())
	require.Len(t, m, 2, "%v", err)
	tol, err := strconv.ParseFloat(m[1], 64)
	require.NoError(t, err)
	require.Greater(t, tol, 5e-3, "the pair tolerance is raised to the held floor of the solid the tooth was trimmed from")
}
