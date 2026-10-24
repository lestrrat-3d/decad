package meshbool

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/rand/v2"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// legacyGatherNearContacts is GatherNearContacts as it was before the distance
// moved to the batch workers: 256-pair batches, and TriTriDistance called in
// the consume callback on the calling goroutine.
func legacyGatherNearContacts(ctx context.Context, bmA *BoolMesh, fis []int, bmB *BoolMesh, fjs []int, slack float64, memo *ContactMemo) (NearContacts, bool, error) {
	var nc NearContacts
	seenA := map[int]bool{}
	seenB := map[int]bool{}
	work := 0
	contacts := NewContactBatchExecutor(ctx, bmA, bmB, memo, ContactWorkers(ctx), func(pair ContactPair, c TriContact) error {
		i, j := pair.I, pair.J
		if c.Kind == ContactRegion {
			return ErrContactBatchStop
		}
		if c.Kind == ContactNone && TriTriDistance(TriCorners(bmA, i), TriCorners(bmB, j)) > slack {
			return nil
		}
		if c.Kind == ContactSegment {
			nc.Spans = append(nc.Spans, ContactSpan{I: i, J: j, P0: c.P0, P1: c.P1})
		}
		if !seenA[i] {
			seenA[i] = true
			nc.CloseA = append(nc.CloseA, i)
		}
		if !seenB[j] {
			seenB[j] = true
			nc.CloseB = append(nc.CloseB, j)
		}
		return nil
	})
	contacts.initialCapacity = contactBatchInitialCapacity
	for _, i := range fis {
		for _, j := range fjs {
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return NearContacts{}, false, err
				}
			}
			if !BoxesWithin(bmA.Boxes[i], bmB.Boxes[j], slack) {
				continue
			}
			if err := contacts.Add(i, j); err != nil {
				if err == ErrContactBatchStop {
					return NearContacts{}, true, nil
				}
				return NearContacts{}, false, err
			}
		}
	}
	if err := contacts.Done(); err != nil {
		if err == ErrContactBatchStop {
			return NearContacts{}, true, nil
		}
		return NearContacts{}, false, err
	}
	return nc, false, nil
}

// nearGridMesh is an n x n grid of quads, two triangles each, with corner
// heights z(x, y) over [ox, ox+n] x [oy, oy+n].
func nearGridMesh(t *testing.T, n int, ox, oy float64, z func(x, y float64) float64) *BoolMesh {
	t.Helper()
	verts := make([]r3.Vec, 0, (n+1)*(n+1))
	for iy := 0; iy <= n; iy++ {
		for ix := 0; ix <= n; ix++ {
			x, y := ox+float64(ix), oy+float64(iy)
			verts = append(verts, r3.NewVec(x, y, z(x, y)))
		}
	}
	tris := make([][3]int, 0, 2*n*n)
	for iy := range n {
		for ix := range n {
			v00 := iy*(n+1) + ix
			v10, v01, v11 := v00+1, v00+n+1, v00+n+2
			tris = append(tris, [3]int{v00, v10, v11}, [3]int{v00, v11, v01})
		}
	}
	bm, err := PrepContext(t.Context(), verts, tris, nil)
	require.NoError(t, err)
	return bm
}

func allFacets(bm *BoolMesh) []int {
	out := make([]int, len(bm.Tris))
	for i := range out {
		out[i] = i
	}
	return out
}

func requireSameNearContacts(t *testing.T, want, got NearContacts, label string) {
	t.Helper()
	require.Equal(t, want.CloseA, got.CloseA, label)
	require.Equal(t, want.CloseB, got.CloseB, label)
	require.Len(t, got.Spans, len(want.Spans), label)
	for k := range want.Spans {
		w, g := want.Spans[k], got.Spans[k]
		require.Equal(t, [2]int{w.I, w.J}, [2]int{g.I, g.J}, label)
		for _, pt := range [][2]proof.Xpt{{w.P0, g.P0}, {w.P1, g.P1}} {
			require.Zero(t, pt[0].X.Cmp(pt[1].X), label)
			require.Zero(t, pt[0].Y.Cmp(pt[1].Y), label)
			require.Zero(t, pt[0].Z.Cmp(pt[1].Z), label)
			require.Zero(t, pt[0].W.Cmp(pt[1].W), label)
		}
	}
}

// nearGatePairs returns mesh pairs that exercise every outcome: facets that
// stay apart (ContactNone, near and far), crossing facets (ContactSegment) and
// a coplanar overlap (ContactRegion, so the gate defers).
func nearGatePairs(t *testing.T, n int) map[string][2]*BoolMesh {
	t.Helper()
	wavy := func(x, _ float64) float64 { return 0.01 * math.Sin(x/3) }
	return map[string][2]*BoolMesh{
		"apart": {
			nearGridMesh(t, n, 0, 0, wavy),
			nearGridMesh(t, n, 0.37, 0.21, func(x, y float64) float64 { return 0.3 + 0.004*x + 0.003*y }),
		},
		"crossing": {
			nearGridMesh(t, n, 0, 0, wavy),
			nearGridMesh(t, n, 0.37, 0.21, func(x, y float64) float64 { return -2.1 + 0.37*x + 0.11*y }),
		},
		"coplanar": {
			nearGridMesh(t, n, 0, 0, func(_, _ float64) float64 { return 0 }),
			nearGridMesh(t, n, 0.5, 0.5, func(_, _ float64) float64 { return 0 }),
		},
	}
}

func TestGatherNearContactsMatchesLegacy(t *testing.T) {
	t.Parallel()
	sawEmpty, sawClose, sawSpans, sawDeferred := false, false, false, false
	for name, pair := range nearGatePairs(t, 6) {
		ma, mb := pair[0], pair[1]
		fa, fb := allFacets(ma), allFacets(mb)
		for _, workers := range []int{1, 4} {
			ctx := WithContactWorkers(t.Context(), workers)
			for k := range 12 {
				slack := 0.02 * math.Pow(1.6, float64(k))
				label := fmt.Sprintf("%s workers=%d slack=%g", name, workers, slack)
				want, wantDeferred, wantErr := legacyGatherNearContacts(ctx, ma, fa, mb, fb, slack, NewContactMemo(ma, mb))
				got, gotDeferred, gotErr := GatherNearContacts(ctx, ma, fa, mb, fb, slack, NewContactMemo(ma, mb))
				require.Equal(t, wantErr, gotErr, label)
				require.Equal(t, wantDeferred, gotDeferred, label)
				requireSameNearContacts(t, want, got, label)
				sawEmpty = sawEmpty || (!wantDeferred && len(want.CloseA) == 0)
				sawClose = sawClose || len(want.CloseA) > 0
				sawSpans = sawSpans || len(want.Spans) > 0
				sawDeferred = sawDeferred || wantDeferred
			}
		}
	}
	require.True(t, sawEmpty, "the sweep never saw an empty CloseA")
	require.True(t, sawClose, "the sweep never saw a non-empty CloseA")
	require.True(t, sawSpans, "the sweep never saw a contact segment")
	require.True(t, sawDeferred, "the sweep never saw a coplanar overlap")
}

func TestGatherNearContactsTreeMatchesLegacy(t *testing.T) {
	t.Parallel()
	pair := nearGatePairs(t, 8)["apart"]
	ma, mb := pair[0], pair[1]
	fis, fjs := allFacets(ma), allFacets(mb)
	fis = append(fis, 0)
	fjs = append(fjs, 0)
	for _, slack := range []float64{0.05, 0.5} {
		ctx := WithContactWorkers(t.Context(), 2)
		want, wantDeferred, wantErr := legacyGatherNearContacts(
			ctx, ma, fis, mb, fjs, slack, NewContactMemo(ma, mb))
		got, gotDeferred, gotErr := GatherNearContacts(
			ctx, ma, fis, mb, fjs, slack, NewContactMemo(ma, mb))
		require.Equal(t, wantErr, gotErr)
		require.Equal(t, wantDeferred, gotDeferred)
		requireSameNearContacts(t, want, got, "tree prefilter")
	}
}

func TestGatherNearContactsMemoHitsMatch(t *testing.T) {
	t.Parallel()
	pair := nearGatePairs(t, 12)["apart"]
	ma, mb := pair[0], pair[1]
	fa, fb := allFacets(ma), allFacets(mb)
	for _, workers := range []int{1, 4} {
		ctx := WithContactWorkers(t.Context(), workers)
		for _, slack := range []float64{0.1, 0.32, 0.5} {
			memo := NewContactMemo(ma, mb)
			first, firstDeferred, err := GatherNearContacts(ctx, ma, fa, mb, fb, slack, memo)
			require.NoError(t, err)
			// Every pair of the second call is a memo hit, so every distance
			// is measured by the consume callback, not by a worker.
			second, secondDeferred, err := GatherNearContacts(ctx, ma, fa, mb, fb, slack, memo)
			require.NoError(t, err)
			require.Equal(t, firstDeferred, secondDeferred)
			requireSameNearContacts(t, first, second, fmt.Sprintf("workers=%d slack=%g", workers, slack))
		}
	}
}

// pollCtx cancels itself once Err has been asked more than limit times.
type pollCtx struct {
	context.Context //nolint:containedctx // test double wrapping a call context.
	limit           int64
	polls           atomic.Int64
	cancelled       atomic.Bool
}

func (c *pollCtx) Err() error {
	if c.polls.Add(1) > c.limit {
		c.cancelled.Store(true)
		return context.Canceled
	}
	return nil
}

// TestGatherNearContactsCancelAtEveryPoll cancels the call after its k-th
// cancellation poll for many k. A call whose context was cancelled never
// returns success; a call that finished first returns the legacy result.
func TestGatherNearContactsCancelAtEveryPoll(t *testing.T) {
	t.Parallel()
	pair := nearGatePairs(t, 4)["crossing"]
	ma, mb := pair[0], pair[1]
	fa, fb := allFacets(ma), allFacets(mb)
	const slack = 0.8
	for _, workers := range []int{1, 4} {
		for _, limit := range []int{16, ContactBatchSize} {
			base := WithContactWorkers(t.Context(), workers)
			want, wantDeferred, err := legacyGatherNearContacts(base, ma, fa, mb, fb, slack, NewContactMemo(ma, mb))
			require.NoError(t, err)
			require.NotEmpty(t, want.CloseA)
			free := &pollCtx{Context: base, limit: math.MaxInt64}
			_, _, err = gatherNearContacts(free, ma, fa, mb, fb, slack, NewContactMemo(ma, mb), NearContactRunner(), limit)
			require.NoError(t, err)
			total := free.polls.Load()
			stride := max(1, total/200)
			for k := int64(0); k <= total+2; k += stride {
				c := &pollCtx{Context: base, limit: k}
				label := fmt.Sprintf("workers=%d limit=%d cancel after %d of %d polls", workers, limit, k, total)
				got, deferred, err := gatherNearContacts(c, ma, fa, mb, fb, slack, NewContactMemo(ma, mb), NearContactRunner(), limit)
				if c.cancelled.Load() {
					require.ErrorIs(t, err, context.Canceled, label)
					require.False(t, deferred, label)
					require.Equal(t, NearContacts{}, got, label)
					continue
				}
				require.NoError(t, err, label)
				require.Equal(t, wantDeferred, deferred, label)
				requireSameNearContacts(t, want, got, label)
			}
		}
	}
}

// TestGatherNearContactsCancelBetweenBatches cancels the context from the
// runner after its first batch has run, with a mesh of several batches.
func TestGatherNearContactsCancelBetweenBatches(t *testing.T) {
	t.Parallel()
	pair := nearGatePairs(t, 12)["apart"]
	ma, mb := pair[0], pair[1]
	fa, fb := allFacets(ma), allFacets(mb)
	for _, workers := range []int{1, 4} {
		ctx, cancel := context.WithCancel(WithContactWorkers(t.Context(), workers))
		inner := NearContactRunner()
		batches := 0
		run := func(c context.Context, a, b *BoolMesh, pairs []ContactPair, results []ContactBatchResult, w int) error {
			batches++
			if err := inner(c, a, b, pairs, results, w); err != nil {
				return err
			}
			cancel()
			return nil
		}
		got, deferred, err := gatherNearContacts(ctx, ma, fa, mb, fb, 3, NewContactMemo(ma, mb), run, ContactBatchSize)
		cancel()
		require.ErrorIs(t, err, context.Canceled, "workers=%d", workers)
		require.False(t, deferred)
		require.Equal(t, NearContacts{}, got)
		require.Equal(t, 1, batches, "the gate went on after the cancel")
	}
}

// TestRunContactBatchReportsCancelAfterWorkers lets the batch's prepare loop pass and cancels at the first
// worker poll, so only the check after the workers is left to report it.
func TestRunContactBatchReportsCancelAfterWorkers(t *testing.T) {
	t.Parallel()
	pair := nearGatePairs(t, 4)["apart"]
	ma, mb := pair[0], pair[1]
	pairs := make([]ContactPair, 0, 16)
	for i := range 16 {
		pairs = append(pairs, ContactPair{I: i, J: i})
	}
	for _, workers := range []int{1, 4} {
		c := &pollCtx{Context: WithContactWorkers(t.Context(), workers), limit: 1}
		results := make([]ContactBatchResult, len(pairs))
		err := NearContactRunner()(c, ma, mb, pairs, results, workers)
		require.ErrorIs(t, err, context.Canceled, "workers=%d", workers)
	}
}

// TestNearContactExecutorOrderAndBatches drives the executor with the gate's
// runner over N pairs of a larger product, N around the batch boundaries, and
// requires the consumed sequence and the batch lengths to be what a serial
// walk produces.
func TestNearContactExecutorOrderAndBatches(t *testing.T) {
	t.Parallel()
	pair := nearGatePairs(t, 8)["crossing"]
	ma, mb := pair[0], pair[1]
	require.Greater(t, len(ma.Tris)*len(mb.Tris), 4097)
	all := make([]ContactPair, 0, len(ma.Tris)*len(mb.Tris))
	for i := range ma.Tris {
		for j := range mb.Tris {
			all = append(all, ContactPair{I: i, J: j})
		}
	}
	type consumed struct {
		pair     ContactPair
		contact  TriContact
		measured bool
		dist     float64
	}
	expect := func(p ContactPair, hit bool) consumed {
		ma.PrepareFloatNormal(p.I)
		mb.PrepareFloatNormal(p.J)
		r := classifyContactPair(ma, mb, p)
		require.NoError(t, r.Err)
		c := consumed{pair: p, contact: r.Contact}
		if r.Contact.Kind == ContactNone && !hit {
			c.measured = true
			c.dist = TriTriDistance(TriCorners(ma, p.I), TriCorners(mb, p.J))
		}
		return c
	}
	// Every 5th pair is a memo hit: the executor must keep it in order.
	hit := func(k int) bool { return k%5 == 2 }
	for _, workers := range []int{1, 4} {
		for _, limit := range []int{4096, ContactBatchSize} {
			for _, n := range []int{0, 1, 2, 255, 256, 257, 4095, 4096, 4097} {
				label := fmt.Sprintf("workers=%d limit=%d n=%d", workers, limit, n)
				memo := NewContactMemo(ma, mb)
				for k := range n {
					if hit(k) {
						p := all[k]
						ma.PrepareFloatNormal(p.I)
						mb.PrepareFloatNormal(p.J)
						r := classifyContactPair(ma, mb, p)
						require.NoError(t, r.Err)
						memo.Store(p.I, p.J, r.Contact)
					}
				}
				var got []consumed
				var batches []int
				inner := NearContactRunner()
				e := NewContactBatchExecutor(t.Context(), ma, mb, memo, workers, nil)
				e.Limit = limit
				e.Run = func(ctx context.Context, a, b *BoolMesh, ps []ContactPair, rs []ContactBatchResult, w int) error {
					batches = append(batches, len(ps))
					return inner(ctx, a, b, ps, rs, w)
				}
				e.ConsumeResult = func(p ContactPair, r ContactBatchResult) error {
					got = append(got, consumed{pair: p, contact: r.Contact, measured: r.Measured, dist: r.Dist})
					return nil
				}
				for k := range n {
					require.NoError(t, e.Add(all[k].I, all[k].J), label)
				}
				require.NoError(t, e.Done(), label)
				require.Len(t, got, n, label)
				for k := range n {
					want := expect(all[k], hit(k))
					require.Equal(t, want.pair, got[k].pair, label)
					require.Equal(t, want.contact.Kind, got[k].contact.Kind, label)
					require.Equal(t, want.measured, got[k].measured, "%s pair %d", label, k)
					require.Equal(t, want.dist, got[k].dist, "%s pair %d", label, k)
				}
				// Runner batches hold only the misses of each full or final
				// batch of Limit queued pairs.
				var wantBatches []int
				for start := 0; start < n; start += limit {
					misses := 0
					for k := start; k < min(start+limit, n); k++ {
						if !hit(k) {
							misses++
						}
					}
					if misses > 0 {
						wantBatches = append(wantBatches, misses)
					}
				}
				require.Equal(t, wantBatches, batches, label)
			}
		}
	}
}

// TestNearContactExecutorFirstErrorWins puts errors in two slots of a batch
// and requires the earlier slot's error, with exactly the pairs before it
// consumed, whatever order the workers finished in.
func TestNearContactExecutorFirstErrorWins(t *testing.T) {
	t.Parallel()
	pair := nearGatePairs(t, 6)["apart"]
	ma, mb := pair[0], pair[1]
	errFirst, errSecond := errors.New("first"), errors.New("second")
	rng := rand.New(rand.NewPCG(5, 9))
	for _, workers := range []int{1, 4} {
		for trial := range 12 {
			n := 300 + rng.IntN(300)
			first := rng.IntN(n - 1)
			second := first + 1 + rng.IntN(n-first-1)
			label := fmt.Sprintf("workers=%d first=%d second=%d n=%d", workers, first, second, n)
			inner := NearContactRunner()
			e := NewContactBatchExecutor(t.Context(), ma, mb, NewContactMemo(ma, mb), workers, nil)
			e.Limit = 1 << 20 // one batch: the flush sees both errors at once
			e.Run = func(ctx context.Context, a, b *BoolMesh, ps []ContactPair, rs []ContactBatchResult, w int) error {
				if err := inner(ctx, a, b, ps, rs, w); err != nil {
					return err
				}
				rs[second].Err = errSecond
				rs[first].Err = errFirst
				return nil
			}
			var seen []ContactPair
			e.ConsumeResult = func(p ContactPair, _ ContactBatchResult) error {
				seen = append(seen, p)
				return nil
			}
			for k := range n {
				require.NoError(t, e.Add(k%len(ma.Tris), (k*7+trial)%len(mb.Tris)), label)
			}
			require.Equal(t, errFirst, e.Done(), label)
			require.Len(t, seen, first, label)
		}
	}
}

// pollRecorder counts ctx.Err polls and cancels from the (limit+1)-th on.
type pollRecorder struct {
	context.Context //nolint:containedctx // test double wrapping a call context.
	cancel          context.CancelFunc
	limit           int64
	polls           atomic.Int64
}

func (c *pollRecorder) Err() error {
	if c.polls.Add(1) > c.limit {
		c.cancel()
	}
	return c.Context.Err()
}

func newPollRecorder(parent context.Context, limit int64) *pollRecorder {
	ctx, cancel := context.WithCancel(parent)
	return &pollRecorder{Context: ctx, cancel: cancel, limit: limit}
}

// TestGatherNearContactsPollScheduleMatchesLegacy cancels the gate and the
// legacy gate at the same poll N, for every N around the end of the walk, on
// a face pair of 256 disjoint facets followed by a coplanar overlap. The
// gate's batch size fixes where it polls relative to each read, so a changed
// schedule gives a different result or poll count at some N.
func TestGatherNearContactsPollScheduleMatchesLegacy(t *testing.T) {
	t.Parallel()
	flat := func(_, _ float64) float64 { return 0 }
	ma := nearGridMesh(t, 20, 0, 0, flat)
	mb := nearGridMesh(t, 20, 0.5, 0.5, flat)
	far := len(mb.Tris) - 1
	ma.PrepareFloatNormal(0)
	mb.PrepareFloatNormal(far)
	mb.PrepareFloatNormal(0)
	c := classifyContactPair(ma, mb, ContactPair{I: 0, J: far})
	require.NoError(t, c.Err)
	require.Equal(t, ContactNone, c.Contact.Kind)
	c = classifyContactPair(ma, mb, ContactPair{I: 0, J: 0})
	require.NoError(t, c.Err)
	require.Equal(t, ContactRegion, c.Contact.Kind)
	for _, tail := range []int{256, 300, 513} {
		fjs := make([]int, tail+1)
		for i := range tail {
			fjs[i] = far
		}
		fjs[tail] = 0 // the coplanar pair comes last
		base := WithContactWorkers(t.Context(), 1)
		free := newPollRecorder(base, math.MaxInt64)
		_, freeDeferred, err := legacyGatherNearContacts(free, ma, []int{0}, mb, fjs, 100, NewContactMemo(ma, mb))
		require.NoError(t, err)
		require.True(t, freeDeferred)
		total := free.polls.Load()
		for n := int64(0); n <= total+3; n++ {
			label := fmt.Sprintf("tail=%d cancel at poll %d of %d", tail, n, total)
			oldCtx := newPollRecorder(base, n)
			newCtx := newPollRecorder(base, n)
			_, od, oe := legacyGatherNearContacts(oldCtx, ma, []int{0}, mb, fjs, 100, NewContactMemo(ma, mb))
			_, nd, ne := GatherNearContacts(newCtx, ma, []int{0}, mb, fjs, 100, NewContactMemo(ma, mb))
			require.Equal(t, od, nd, label)
			require.Equal(t, fmt.Sprint(oe), fmt.Sprint(ne), label)
			require.Equal(t, oldCtx.polls.Load(), newCtx.polls.Load(), label)
		}
	}
}
