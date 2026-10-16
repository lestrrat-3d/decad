package meshbool

import (
	"context"
	"fmt"
	"math"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// Frozen original control flow is the scalar oracle for output and failure behavior.
func originalConformOnce(ctx context.Context, verts []proof.Xpt, tris *[][3]int, src *[]int, bounds *[]float64, xbeta []float64) (bool, error) {
	// One counter spans the facet walk, the grid-cell candidate scan nested
	// under it, and the along-edge ordering: the candidate vertices a single
	// facet edge sweeps are the candidate operations §7.2 counts, not the
	// facets.
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return false, err
	}
	scan, err := NewConformScan(budget, verts)
	if err != nil {
		return false, err
	}

	var outTris [][3]int
	var outSrc []int
	var outBounds []float64
	splitAny := false
	for ti, tri := range *tris {
		if err := budget.Step(); err != nil {
			return false, err
		}
		inserted := [3][]int{}
		for k := range 3 {
			a, b := tri[k], tri[(k+1)%3]
			hits, err := scan.EdgeInteriorHits(budget, a, b, tri)
			if err != nil {
				return false, err
			}
			if len(hits) > 0 {
				if err := SortAlongEdge(budget, verts, a, b, hits); err != nil {
					return false, err
				}
				inserted[k] = hits
			}
		}
		if inserted[0] == nil && inserted[1] == nil && inserted[2] == nil {
			outTris = append(outTris, tri)
			outSrc = append(outSrc, (*src)[ti])
			outBounds = append(outBounds, (*bounds)[ti])
			continue
		}
		splitAny = true
		for _, hits := range inserted {
			for _, h := range hits {
				xbeta[h] = max(xbeta[h], (*bounds)[ti])
			}
		}
		poly := []int{tri[0]}
		poly = append(poly, inserted[0]...)
		poly = append(poly, tri[1])
		poly = append(poly, inserted[1]...)
		poly = append(poly, tri[2])
		poly = append(poly, inserted[2]...)
		newTris, err := TriangulatePlanarPolygon(ctx, verts, poly)
		if err != nil {
			return false, err
		}
		for _, nt := range newTris {
			outTris = append(outTris, nt)
			outSrc = append(outSrc, (*src)[ti])
			outBounds = append(outBounds, (*bounds)[ti])
		}
	}
	*tris = outTris
	*src = outSrc
	*bounds = outBounds
	return splitAny, nil
}

type conformPollContext struct{ calls, stop int }

func (*conformPollContext) Deadline() (time.Time, bool) { return time.Time{}, false }
func (*conformPollContext) Done() <-chan struct{}       { return nil }
func (*conformPollContext) Value(any) any               { return nil }
func (c *conformPollContext) Err() error {
	c.calls++
	if c.stop > 0 && c.calls >= c.stop {
		return context.Canceled
	}
	return nil
}

type conformOutputCase struct {
	name              string
	tris              [][3]int
	src               []int
	bounds            []float64
	alias             bool
	nilSrc, nilBounds bool
}
type conformOutputResult struct {
	tris           [][3]int
	src            []int
	bounds, beta   []float64
	split          bool
	err, panicText string
	polls          int
	replaced       [3]bool
}

func runConformOutput(c conformOutputCase, stop int, original bool) (out conformOutputResult) {
	verts := []proof.Xpt{}
	for _, v := range []r3.Vec{{}, {X: 2}, {Y: 2}, {X: 1}, {X: 4}, {X: 5}, {X: 4, Y: 1}} {
		verts = append(verts, proof.XptOf(v))
	}
	// Preserve nonnil empty slices and give each invocation independent backing storage.
	tris := append(make([][3]int, 0, len(c.tris)), c.tris...)
	src := append(make([]int, 0, len(c.src)), c.src...)
	bounds := append(make([]float64, 0, len(c.bounds)), c.bounds...)
	beta := make([]float64, len(verts))
	if c.alias {
		copy(beta, bounds)
		bounds = beta[:len(bounds)]
	}
	tp, sp, bp := &tris, &src, &bounds
	if c.nilSrc {
		sp = nil
	}
	if c.nilBounds {
		bp = nil
	}
	var triFirst *[3]int
	var srcFirst *int
	var boundFirst *float64
	if len(tris) > 0 {
		triFirst = &tris[0]
	}
	if len(src) > 0 {
		srcFirst = &src[0]
	}
	if len(bounds) > 0 {
		boundFirst = &bounds[0]
	}
	ctx := &conformPollContext{stop: stop}
	defer func() {
		if value := recover(); value != nil {
			out.panicText = fmt.Sprintf("%T: %.160s", value, fmt.Sprint(value))
		}
		out.tris, out.src, out.bounds, out.beta, out.polls = tris, src, bounds, beta, ctx.calls
		out.replaced = [3]bool{len(tris) > 0 && &tris[0] != triFirst, len(src) > 0 && &src[0] != srcFirst, len(bounds) > 0 && &bounds[0] != boundFirst}
	}()
	var err error
	if original {
		out.split, err = originalConformOnce(ctx, verts, tp, sp, bp, beta)
	} else {
		out.split, err = ConformOnce(ctx, verts, tp, sp, bp, beta)
	}
	if err != nil {
		out.err = err.Error()
	}
	return
}
func requireConformOutput(t *testing.T, want, got conformOutputResult) {
	t.Helper()
	require.Equal(t, want.split, got.split)
	require.Equal(t, want.err, got.err)
	require.Equal(t, want.panicText, got.panicText)
	require.Equal(t, want.polls, got.polls)
	for k := range 3 {
		if want.err == "" && want.panicText == "" && !want.split && len(want.tris) != 0 {
			require.True(t, want.replaced[k], "original replacement %d", k)
			require.False(t, got.replaced[k], "unchanged storage %d", k)
		} else {
			require.Equal(t, want.replaced[k], got.replaced[k], "replacement %d", k)
		}
	}
	require.Equal(t, want.tris == nil, got.tris == nil)
	require.Equal(t, len(want.tris), len(got.tris))
	for i := range want.tris {
		for k := range 3 {
			require.Equal(t, want.tris[i][k], got.tris[i][k], "tri %d corner %d", i, k)
		}
	}
	require.Equal(t, want.src == nil, got.src == nil)
	require.Equal(t, len(want.src), len(got.src))
	for i := range want.src {
		require.Equal(t, want.src[i], got.src[i], "source %d", i)
	}
	for k, pair := range [][2][]float64{{want.bounds, got.bounds}, {want.beta, got.beta}} {
		require.Equal(t, pair[0] == nil, pair[1] == nil)
		require.Equal(t, len(pair[0]), len(pair[1]))
		for i := range pair[0] {
			require.Equal(t, math.Float64bits(pair[0][i]), math.Float64bits(pair[1][i]), "float row %d index %d", k, i)
		}
	}
}
func TestConformUnchangedOutputMatchesOriginal(t *testing.T) {
	cases := []conformOutputCase{
		{name: "empty", tris: [][3]int{}, src: []int{}, bounds: []float64{}},
		{name: "unchanged", tris: [][3]int{{4, 5, 6}}, src: []int{9, 10}, bounds: []float64{0.25, 0.5}},
		{name: "split_first", tris: [][3]int{{0, 1, 2}, {4, 5, 6}}, src: []int{8, 9}, bounds: []float64{0.25, 0.5}},
		{name: "split_last", tris: [][3]int{{4, 5, 6}, {0, 1, 2}}, src: []int{9, 8}, bounds: []float64{0.5, 0.25}},
		{name: "aliased_bounds", tris: [][3]int{{4, 5, 6}, {4, 5, 6}, {4, 5, 6}, {4, 5, 6}, {0, 1, 2}}, src: []int{0, 1, 2, 3, 4}, bounds: []float64{0, 0, 0, 0, 0.5}, alias: true},
		{name: "short_source_before_split", tris: [][3]int{{4, 5, 6}, {0, 1, 2}}, bounds: []float64{0.25, 0.5}},
		{name: "short_bounds_before_split", tris: [][3]int{{4, 5, 6}, {0, 1, 2}}, src: []int{8, 9}},
		{name: "short_source_after_split", tris: [][3]int{{0, 1, 2}}, bounds: []float64{0.5}},
		{name: "short_bounds_at_split", tris: [][3]int{{0, 1, 2}}, src: []int{8}},
		{name: "nil_source_at_split", tris: [][3]int{{0, 1, 2}}, bounds: []float64{0.5}, nilSrc: true},
		{name: "nil_bounds_at_split", tris: [][3]int{{0, 1, 2}}, src: []int{8}, nilBounds: true},
		{name: "empty_nil_source", nilSrc: true},
		{name: "invalid_vertex", tris: [][3]int{{4, 5, 99}}, src: []int{8}, bounds: []float64{0.5}},
		{name: "degenerate_split", tris: [][3]int{{0, 1, 0}}, src: []int{8}, bounds: []float64{0.5}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			want, got := runConformOutput(c, 0, true), runConformOutput(c, 0, false)
			requireConformOutput(t, want, got)
			switch c.name {
			case "empty":
				require.Empty(t, got.err)
				require.Empty(t, got.panicText)
				require.Nil(t, got.tris)
				require.Nil(t, got.src)
				require.Nil(t, got.bounds)
			case "unchanged", "split_first", "split_last", "aliased_bounds":
				require.Empty(t, got.err)
				require.Empty(t, got.panicText)
				require.Equal(t, c.name != "unchanged", got.split)
				for k := range 3 {
					require.Equal(t, c.name != "unchanged", got.replaced[k], "replacement %d", k)
				}
			}
		})
	}
}
func TestConformUnchangedOutputCancellationMatchesOriginal(t *testing.T) {
	c := conformOutputCase{name: "polls"}
	for i := 0; i < 300; i++ {
		tri := [3]int{4, 5, 6}
		if i == 2 {
			tri = [3]int{0, 1, 2}
		}
		c.tris = append(c.tris, tri)
		c.src = append(c.src, i)
		c.bounds = append(c.bounds, 0.5)
	}
	canceledAfterUpdate := false
	for stop := 1; stop <= 12; stop++ {
		t.Run(fmt.Sprint(stop), func(t *testing.T) {
			want, got := runConformOutput(c, stop, true), runConformOutput(c, stop, false)
			require.Equal(t, context.Canceled.Error(), want.err, "oracle must cancel")
			require.Equal(t, context.Canceled.Error(), got.err, "candidate must cancel")
			require.Empty(t, want.panicText)
			require.Empty(t, got.panicText)
			require.Equal(t, stop, want.polls)
			require.Equal(t, stop, got.polls)
			requireConformOutput(t, want, got)
			if got.beta[3] == 0.5 {
				canceledAfterUpdate = true
			}
		})
	}
	require.True(t, canceledAfterUpdate, "exercise cancellation after a bound update")
}
