package decad

import (
	"context"
	"errors"
	"math"
	"regexp"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// slabScene builds retention §1's P3 scene through the public API.
type slabScene struct {
	w   *sketch.World
	doc *Document
}

func (sc *slabScene) roundedBox(ctx context.Context, t *testing.T) *Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(r.A)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	box, err := Extrude(ctx, sc.doc, s, s.Profiles()[0],
		Distance{D: units.Millimeters(20), Dir: Along})
	require.NoError(t, err)
	rounded, err := Fillet(ctx, box,
		Edges(ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4), units.Millimeters(3))
	require.NoError(t, err)
	return rounded
}

// yTool builds a y-axis cylinder of radius r through (cx, ·, 10), placed by a
// pure translation so the Cut is not a same-placement pair.
func (sc *slabScene) yTool(ctx context.Context, t *testing.T, cx, r float64) *Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XZ())
	require.NoError(t, err)
	c := s.CreatePoint(cx, 10)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	cyl, err := Extrude(ctx, sc.doc, s, s.Profiles()[0],
		Distance{D: units.Millimeters(60), Dir: Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Y: 30})
	require.NoError(t, err)
	placed, err := Transform(ctx, cyl, move)
	require.NoError(t, err)
	return placed
}

// slabRun cuts the first bore, copies it, then cuts the second at (cx, r), in a fresh document.
func slabRun(t *testing.T, ctx context.Context, cx, r float64) (*Body, error) {
	t.Helper()
	sc := &slabScene{w: sketch.NewWorld(), doc: New()}
	built, err := Cut(ctx, sc.roundedBox(ctx, t), sc.yTool(ctx, t, 20, 4))
	require.NoError(t, err)
	// A Copy ends the operation history's Boolean chain, so the control's refusal is
	// the sequential gate, not a chain the regroup rebuilds.
	first, err := Copy(ctx, built)
	require.NoError(t, err)
	return Cut(ctx, first, sc.yTool(ctx, t, cx, r))
}

// TestFacetedSourcePatchSlabControl measures retention §11's retained-plane
// slab against the full-displacement control (booleaneval.WithoutRetainedPlaneSlab)
// on the real Cut-then-Cut producer. The control refuses every second cut on the
// first cut's plane FacetBound, so the comparison is a real build against the
// control's refusal.
func TestFacetedSourcePatchSlabControl(t *testing.T) {
	t.Parallel()

	for _, tc := range []struct {
		name  string
		cx, r float64
	}{
		{"x=30 r=4", 30, 4},
		{"x=33 r=2", 33, 2},
		{"x=10 r=1", 10, 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			second, err := slabRun(t, t.Context(), tc.cx, tc.r)
			require.NoError(t, err)

			_, err = slabRun(t, booleaneval.WithoutRetainedPlaneSlab(t.Context()), tc.cx, tc.r)
			require.ErrorIs(t, err, ErrUnsupported)
			var be *BooleanError
			require.False(t, errors.As(err, &be), "the chain-depth refusal is a plain ErrUnsupported")
			m := regexp.MustCompile(`within a bound of (\S+) mm, coarser than this pair's chord tolerance (\S+) mm`).
				FindStringSubmatch(err.Error())
			require.Len(t, m, 3, "the refusal quotes the touched bound and the pair's chord tolerance")
			touched, err := strconv.ParseFloat(m[1], 64)
			require.NoError(t, err)
			tol, err := strconv.ParseFloat(m[2], 64)
			require.NoError(t, err)
			require.Greater(t, touched, tol)

			// Volume: both holes run the full 20 mm through y and do not touch.
			discs := math.Pi*16 + math.Pi*tc.r*tc.r
			want := (800-4*(1-math.Pi/4)*9)*20 - 20*discs
			got, err := second.Volume()
			require.NoError(t, err)
			require.InDelta(t, want, got.Value.Mag(), got.Bound.Mag()+1e-6)

			// Every rim vertex lies within its bound of the exact circle, and
			// the largest bound is strictly below what the control would hold.
			checked := 0
			maxBound := 0.0
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
				b := p.Bound.Mag()
				require.LessOrEqual(t, off, b+1e-9, "vertex %v", p.Value)
				maxBound = math.Max(maxBound, b)
			}
			require.Positive(t, checked, "the new wall leaves rim vertices on the faces")
			require.Less(t, maxBound, touched, "the slab states a rim bound below the control's touched bound")
		})
	}
}
