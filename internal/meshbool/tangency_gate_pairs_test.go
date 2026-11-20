package meshbool

import (
	"context"
	"errors"
	"math"
	"slices"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// gridPrism is the closed box [lo, hi] with each side cut into an n×n grid of
// quads, two facets each. Every quad is its own source face, and face k (in
// facet order) has the bound deltaOf(k).
func gridPrism(t *testing.T, lo, hi r3.Vec, n int, deltaOf func(k int) float64) (*topo.Mesh, *BoolMesh) {
	t.Helper()
	index := map[r3.Vec]int{}
	var verts []r3.Vec
	vertex := func(v r3.Vec) int {
		if i, ok := index[v]; ok {
			return i
		}
		index[v] = len(verts)
		verts = append(verts, v)
		return len(verts) - 1
	}
	lerp := func(a, b float64, k int) float64 { return a + (b-a)*float64(k)/float64(n) }
	// Each side is (axis, at high end); u and v run over the other two axes
	// in the order that winds the quads outward.
	type side struct {
		axis int
		high bool
	}
	at := func(s side, u, v int) r3.Vec {
		c := [3]float64{}
		los := [3]float64{lo.X, lo.Y, lo.Z}
		his := [3]float64{hi.X, hi.Y, hi.Z}
		a, b := (s.axis+1)%3, (s.axis+2)%3
		if !s.high {
			a, b = b, a
		}
		c[s.axis] = los[s.axis]
		if s.high {
			c[s.axis] = his[s.axis]
		}
		c[a] = lerp(los[a], his[a], u)
		c[b] = lerp(los[b], his[b], v)
		return r3.NewVec(c[0], c[1], c[2])
	}
	var tris [][3]int
	var source []*topo.Face
	bounds := map[*topo.Face]float64{}
	for _, s := range []side{{0, false}, {0, true}, {1, false}, {1, true}, {2, false}, {2, true}} {
		for u := range n {
			for v := range n {
				q := [4]int{vertex(at(s, u, v)), vertex(at(s, u+1, v)), vertex(at(s, u+1, v+1)), vertex(at(s, u, v+1))}
				face := &topo.Face{}
				bounds[face] = deltaOf(len(bounds))
				tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
				source = append(source, face, face)
			}
		}
	}
	bm, err := PrepContext(t.Context(), verts, tris, nil)
	require.NoError(t, err)
	return &topo.Mesh{Vertices: verts, Triangles: tris, Source: source, FaceBound: bounds}, bm
}

// gatePairVisit is one examine call: the face indices, the slack's bits, the
// certified flag, and how many budget steps preceded it.
type gatePairVisit struct {
	a, b      int
	slack     uint64
	certified bool
	steps     uint64
}

// referenceGateFacePairs is the all-pairs scan gateFacePairs replaces, kept
// verbatim as the definition its row order, rejections and steps must match.
func referenceGateFacePairs(budget *proofbound.WorkBudget, fa, fb []faceFacets, capFace, coneFace *topo.Face,
	examine func(ga, gb faceFacets, slack float64, certified bool) error) error {
	for _, ga := range fa {
		for _, gb := range fb {
			if err := budget.Step(); err != nil {
				return err
			}
			if capFace != nil && ga.source == capFace && gb.source == coneFace {
				if err := examine(ga, gb, 0, true); err != nil {
					return err
				}
				continue
			}
			slack := ga.delta + gb.delta
			if slack <= 0 {
				continue
			}
			if !BoxesWithin(ga.box, gb.box, slack*(1+1e-9)) {
				continue
			}
			if err := examine(ga, gb, slack, false); err != nil {
				return err
			}
		}
	}
	return nil
}

type gatePairScan func(*proofbound.WorkBudget, []faceFacets, []faceFacets, *topo.Face, *topo.Face,
	func(faceFacets, faceFacets, float64, bool) error) error

// runGatePairScan runs scan with a counting budget that fails its failAt-th
// step (never when failAt is zero) and an examine that fails its stopAt-th
// call (never when stopAt is zero). It returns the visits, the total steps and
// the error.
func runGatePairScan(scan gatePairScan, fa, fb []faceFacets, capFace, coneFace *topo.Face,
	failAt uint64, stopAt int) ([]gatePairVisit, uint64, error) {
	errStep := errors.New("step refused")
	errStop := errors.New("examine stopped")
	errOverrun := errors.New("more steps than face pairs")
	limit := uint64(len(fa) * len(fb))
	var steps uint64
	budget := &proofbound.WorkBudget{StepFn: func() error {
		steps++
		if failAt > 0 && steps == failAt {
			return errStep
		}
		// A scan that steps past every face pair has broken its count;
		// stop it rather than let it run on.
		if steps > limit {
			return errOverrun
		}
		return nil
	}}
	position := func(faces []faceFacets, f *topo.Face) int {
		for k, g := range faces {
			if g.source == f {
				return k
			}
		}
		return -1
	}
	var visits []gatePairVisit
	err := scan(budget, fa, fb, capFace, coneFace, func(ga, gb faceFacets, slack float64, certified bool) error {
		visits = append(visits, gatePairVisit{a: position(fa, ga.source), b: position(fb, gb.source),
			slack: math.Float64bits(slack), certified: certified, steps: steps})
		if stopAt > 0 && len(visits) == stopAt {
			return errStop
		}
		return nil
	})
	return visits, steps, err
}

// requireSameVisitsf compares two visit lists and reports only their lengths
// and the first difference.
func requireSameVisitsf(t *testing.T, want, got []gatePairVisit, format string, args ...any) {
	t.Helper()
	for k := range min(len(want), len(got)) {
		if want[k] != got[k] {
			require.Failf(t, "visits differ", "at %d of %d/%d: want %+v, got %+v; "+format,
				append([]any{k, len(want), len(got), want[k], got[k]}, args...)...)
		}
	}
	require.Equal(t, len(want), len(got), append([]any{"visit count; " + format}, args...)...)
}

// gatePollCounter counts its Err polls and cancels from the cancelAt-th one.
// Past limit polls it fails with errPollOverrun, which bounds a scan that
// steps past its face pairs.
type gatePollCounter struct {
	context.Context //nolint:containedctx // deterministic cancellation fixture for one scan.
	polls           atomic.Int64
	cancelAt        int64
	limit           int64
}

var errPollOverrun = errors.New("more polls than face pairs")

func (c *gatePollCounter) Err() error {
	polls := c.polls.Add(1)
	if c.cancelAt > 0 && polls >= c.cancelAt {
		return context.Canceled
	}
	if polls > c.limit {
		return errPollOverrun
	}
	return c.Context.Err()
}

// TestGateFacePairsMatchesAllPairsScan holds the tangency gate's face-pair
// enumeration to the all-pairs scan on gridded prisms: the same examined pairs
// in the same order, with the same slack and certified flag, after the same
// number of budget steps, and the same total. A failing step, a failing
// examine call and a cancelled context stop both at the same place.
//
// Legs shown to fail: dropping the ascending sort of the tree's candidates,
// dropping gateFacePair's box test, skipping a row's last candidate, and
// dropping the row's trailing StepN.
func TestGateFacePairsMatchesAllPairsScan(t *testing.T) {
	t.Parallel()
	mixed := func(k int) float64 { return [...]float64{0, 0.01, 0.05, 0.002}[k%4] }
	faces := func(t *testing.T, m *topo.Mesh, bm *BoolMesh) []faceFacets {
		t.Helper()
		out, err := facesOfMesh(proofbound.NewWorkBudget(t.Context()), m, bm.Boxes, bm.FaceSurface)
		require.NoError(t, err)
		return out
	}
	shifted := func(t *testing.T, shift r3.Vec, n int, deltaOf func(int) float64) []faceFacets {
		t.Helper()
		m, bm := gridPrism(t, shift, r3.NewVec(1, 1, 1).Add(shift), n, deltaOf)
		return faces(t, m, bm)
	}
	unit := func(t *testing.T, n int, deltaOf func(int) float64) []faceFacets {
		t.Helper()
		return shifted(t, r3.Vec{}, n, deltaOf)
	}
	type scanCase struct {
		name             string
		fa, fb           []faceFacets
		capFace, coneFac *topo.Face
		certified        bool
	}
	flushA, flushB := unit(t, 3, mixed), shifted(t, r3.NewVec(0, 0, 1), 3, mixed)
	nearA, nearB := unit(t, 4, func(int) float64 { return 0.02 }), shifted(t, r3.NewVec(0.3, 0.2, 1.03), 4,
		func(k int) float64 { return 0.02 + 0.001*float64(k%3) })
	crossA, crossB := unit(t, 3, mixed), shifted(t, r3.NewVec(0.5, 0.5, 0.5), 3, mixed)
	infB := shifted(t, r3.NewVec(0, 0, 1), 3, mixed)
	infB[7].delta = math.Inf(1)
	nanA := unit(t, 3, mixed)
	nanA[5].delta = math.NaN()
	cases := []scanCase{
		{name: "flush prisms", fa: flushA, fb: flushB},
		{name: "near-miss prisms", fa: nearA, fb: nearB},
		{name: "crossing prisms", fa: crossA, fb: crossB},
		{name: "a certified cap pair", fa: flushA, fb: flushB, capFace: flushA[40].source, coneFac: flushB[10].source,
			certified: true},
		{name: "a cap face with no cone partner", fa: flushA, fb: flushB, capFace: flushA[40].source,
			coneFac: &topo.Face{}},
		{name: "an infinite delta in the second operand", fa: flushA, fb: infB},
		{name: "a NaN delta in the first operand", fa: nanA, fb: flushB},
		{name: "a second operand within one tree leaf", fa: flushA, fb: shifted(t, r3.NewVec(0, 0, 1), 1, mixed)},
		{name: "an empty first operand", fb: flushB},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			want, wantSteps, err := runGatePairScan(referenceGateFacePairs, c.fa, c.fb, c.capFace, c.coneFac, 0, 0)
			require.NoError(t, err)
			require.Equal(t, uint64(len(c.fa)*len(c.fb)), wantSteps)
			require.Equal(t, c.certified, slices.ContainsFunc(want, func(v gatePairVisit) bool { return v.certified }))
			got, gotSteps, err := runGatePairScan(gateFacePairs, c.fa, c.fb, c.capFace, c.coneFac, 0, 0)
			require.NoError(t, err)
			requireSameVisitsf(t, want, got, "full scan")
			require.Equal(t, wantSteps, gotSteps)
			if len(want) == 0 {
				return
			}
			// Stop at a failing step inside the scan, and at a failing examine call.
			for _, failAt := range []uint64{1, want[len(want)/2].steps, want[len(want)-1].steps, wantSteps} {
				want, wantSteps, wantErr := runGatePairScan(referenceGateFacePairs, c.fa, c.fb, c.capFace, c.coneFac, failAt, 0)
				got, gotSteps, gotErr := runGatePairScan(gateFacePairs, c.fa, c.fb, c.capFace, c.coneFac, failAt, 0)
				require.Equal(t, wantErr.Error(), gotErr.Error(), "fail at step %d", failAt)
				requireSameVisitsf(t, want, got, "fail at step %d", failAt)
				require.Equal(t, wantSteps, gotSteps, "fail at step %d", failAt)
			}
			for _, stopAt := range []int{1, (len(want) + 1) / 2, len(want)} {
				want, wantSteps, wantErr := runGatePairScan(referenceGateFacePairs, c.fa, c.fb, c.capFace, c.coneFac, 0, stopAt)
				got, gotSteps, gotErr := runGatePairScan(gateFacePairs, c.fa, c.fb, c.capFace, c.coneFac, 0, stopAt)
				require.Equal(t, wantErr.Error(), gotErr.Error(), "stop at call %d", stopAt)
				requireSameVisitsf(t, want, got, "stop at call %d", stopAt)
				require.Equal(t, wantSteps, gotSteps, "stop at call %d", stopAt)
			}
		})
	}
	t.Run("a context budget polls and cancels at the same counts", func(t *testing.T) {
		t.Parallel()
		scan := func(scan gatePairScan, cancelAt int64) (int, int64, error) {
			ctx := &gatePollCounter{Context: t.Context(), cancelAt: cancelAt,
				limit: int64(len(nearA) * len(nearB) / proofbound.WorkPollInterval)}
			calls := 0
			err := scan(proofbound.NewWorkBudget(ctx), nearA, nearB, nil, nil,
				func(faceFacets, faceFacets, float64, bool) error {
					calls++
					return nil
				})
			return calls, ctx.polls.Load(), err
		}
		wantCalls, wantPolls, err := scan(referenceGateFacePairs, 0)
		require.NoError(t, err)
		require.Positive(t, wantCalls)
		require.Equal(t, int64(len(nearA)*len(nearB)/proofbound.WorkPollInterval), wantPolls)
		gotCalls, gotPolls, err := scan(gateFacePairs, 0)
		require.NoError(t, err)
		require.Equal(t, wantCalls, gotCalls)
		require.Equal(t, wantPolls, gotPolls)
		for _, cancelAt := range []int64{1, wantPolls / 2, wantPolls} {
			wantCalls, wantPolls, wantErr := scan(referenceGateFacePairs, cancelAt)
			require.ErrorIs(t, wantErr, context.Canceled)
			gotCalls, gotPolls, gotErr := scan(gateFacePairs, cancelAt)
			require.ErrorIs(t, gotErr, context.Canceled)
			require.Equal(t, wantCalls, gotCalls, "cancel at poll %d", cancelAt)
			require.Equal(t, wantPolls, gotPolls, "cancel at poll %d", cancelAt)
		}
	})
}
