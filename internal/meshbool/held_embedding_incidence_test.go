package meshbool

import (
	"context"
	"runtime"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// Private construction and stamp state are the contracts under test here.
func originalHeldIncidence(vertices int, tris [][3]int) [][]int {
	inc := make([][]int, vertices)
	for i, tri := range tris {
		for _, v := range tri {
			inc[v] = append(inc[v], i)
		}
	}
	return inc
}

func requireHeldIncidence(t *testing.T, got, want [][]int) {
	t.Helper()
	require.Equal(t, len(want), len(got), "vertex count")
	for v := range want {
		require.Equal(t, want[v] == nil, got[v] == nil, "nil row %d", v)
		require.Equal(t, len(want[v]), len(got[v]), "row length %d", v)
		for k, f := range want[v] {
			require.Equal(t, f, got[v][k], "vertex %d occurrence %d", v, k)
		}
	}
}

func TestHeldIncidenceMatchesAppend(t *testing.T) {
	cases := []struct {
		name     string
		vertices int
		tris     [][3]int
	}{
		{"empty", 0, nil},
		{"unused", 7, nil},
		{"single", 4, [][3]int{{2, 0, 1}}},
		{"duplicates", 5, [][3]int{{2, 0, 2}, {1, 2, 0}, {2, 0, 2}, {3, 3, 3}}},
		{"disconnected", 7, [][3]int{{0, 1, 2}, {3, 4, 5}}},
	}
	fan := make([][3]int, 64)
	for i := range fan {
		fan[i] = [3]int{0, i + 1, (i+1)%64 + 1}
	}
	cases = append(cases, struct {
		name     string
		vertices int
		tris     [][3]int
	}{"fan", 66, fan})
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			h := HeldRounding{Verts: make([]r3.Vec, tc.vertices), Tris: tc.tris}
			want := originalHeldIncidence(tc.vertices, tc.tris)
			for _, memo := range []bool{false, true} {
				e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), memo)
				requireHeldIncidence(t, e.inc, want)
				for v, row := range e.inc {
					require.Equal(t, len(row), cap(row), "capacity %d", v)
					if len(row) == 0 {
						continue
					}
					e.inc[v] = append(row, -1)
					e.inc[v][0] = -2
					for w := range want {
						if w == v {
							continue
						}
						for k, f := range want[w] {
							require.Equal(t, f, e.inc[w][k], "append isolation %d %d", w, k)
						}
					}
					e.inc[v] = row
				}
				other := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), memo)
				for v, row := range e.inc {
					if len(row) == 0 {
						continue
					}
					old := row[0]
					e.inc[v][0] = -3
					requireHeldIncidence(t, other.inc, want)
					e.inc[v][0] = old
				}
				requireHeldIncidence(t, other.inc, want)
			}
		})
	}
}

func heldIncidencePanicDetail(t *testing.T, operation func()) string {
	t.Helper()
	var detail string
	func() {
		defer func() {
			value := recover()
			err, ok := value.(runtime.Error)
			require.True(t, ok, "expected runtime bounds panic")
			detail = err.Error()
			require.LessOrEqual(t, len(detail), 160, "panic diagnostic length")
		}()
		operation()
	}()
	require.NotEmpty(t, detail)
	return detail
}

func TestHeldIncidenceInvalidIndex(t *testing.T) {
	cases := [][][3]int{
		{{0, 1, 2}, {0, -1, 3}},
		{{0, 1, 2}, {0, 3, -1}},
		{{0, -1, 2}, {3, 1, 2}},
		{{0, 3, 2}, {-1, 1, 2}},
	}
	for i, tris := range cases {
		want := heldIncidencePanicDetail(t, func() { originalHeldIncidence(3, tris) })
		got := heldIncidencePanicDetail(t, func() {
			newHeldEmbedding(HeldRounding{Verts: make([]r3.Vec, 3), Tris: tris},
				proofbound.NewWorkBudget(t.Context()), true)
		})
		require.Equal(t, want, got, "first invalid index case %d", i)
	}
}

func TestHeldIncidenceSize(t *testing.T) {
	maxTriangles := int(^uint(0)>>1) / (3 * (strconv.IntSize / 8))
	for _, n := range []int{0, 1, 64, maxTriangles} {
		entries, ok := heldIncidenceSize(n)
		require.True(t, ok, "triangle count %d", n)
		require.Equal(t, n, entries/3)
		require.LessOrEqual(t, entries, int(^uint(0)>>1)/(strconv.IntSize/8))
	}
	for _, n := range []int{-1, maxTriangles + 1, int(^uint(0) >> 1)} {
		entries, ok := heldIncidenceSize(n)
		require.False(t, ok, "triangle count %d", n)
		require.Zero(t, entries)
	}
}

func TestHeldIncidenceGenerationAndStamps(t *testing.T) {
	h := HeldRounding{Verts: []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}},
		Tris: [][3]int{{0, 1, 0}, {1, 2, 0}, {0, 1, 0}, {1, 2, 3}}}
	e := newHeldEmbedding(h, proofbound.NewWorkBudget(t.Context()), true)
	want := originalHeldIncidence(len(h.Verts), h.Tris)
	for f := range e.gen {
		e.gen[f] = uint64(10 + f)
	}
	saved := e.stamps(0)
	for k, f := range want[0] {
		require.Equal(t, uint64(10+f), saved[k], "stamp %d", k)
	}
	e.setVert(0, r3.Vec{X: 2})
	expected := []uint64{10, 11, 12, 13}
	for k, f := range want[0] {
		expected[f] = uint64(k + 1)
	}
	for f := range expected {
		require.Equal(t, expected[f], e.gen[f], "generation %d", f)
	}
	require.Equal(t, uint64(len(want[0])+1), e.nextGen)
	for k, f := range want[0] {
		require.Equal(t, uint64(10+f), saved[k], "snapshot %d", k)
	}
	e.restamp(0, saved)
	for f := range e.gen {
		require.Equal(t, uint64(10+f), e.gen[f], "restamp %d", f)
	}
	requireHeldIncidence(t, e.inc, want)
}

func TestHeldIncidenceBudgetAndCancellation(t *testing.T) {
	steps, polls := 0, 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error { steps++; return context.Canceled },
		ErrFn:  func() error { polls++; return context.Canceled },
	}
	h := HeldRounding{Verts: make([]r3.Vec, 3), Tris: [][3]int{{0, 1, 2}}}
	e := newHeldEmbedding(h, budget, true)
	requireHeldIncidence(t, e.inc, originalHeldIncidence(3, h.Tris))
	require.Zero(t, steps)
	require.Zero(t, polls)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	// A canceled caller must return before even malformed indices are inspected.
	h.Tris[0][0] = -1
	n, err := EnforceHeldEmbedding(ctx, h)
	require.ErrorIs(t, err, context.Canceled)
	require.Zero(t, n)
}

// Err calls make the unchanged initial and every-256-step polls observable.
type heldIncidenceCancelContext struct {
	context.Context
	polls int
	stop  int
}

func (c *heldIncidenceCancelContext) Err() error {
	c.polls++
	if c.polls >= c.stop {
		return context.Canceled
	}
	return nil
}

func TestHeldIncidenceCancellationPollOrder(t *testing.T) {
	for _, count := range []int{proofbound.WorkPollInterval - 1, proofbound.WorkPollInterval,
		2 * proofbound.WorkPollInterval} {
		for _, stop := range []int{1, 2, 3, 4} {
			ctx := &heldIncidenceCancelContext{Context: t.Context(), stop: stop}
			h := HeldRounding{Verts: make([]r3.Vec, 3), Moved: make([]bool, 3), Tris: make([][3]int, count)}
			for i := range h.Tris {
				h.Tris[i] = [3]int{0, 1, 2}
			}
			n, err := EnforceHeldEmbedding(ctx, h)
			polls := 1 + count/proofbound.WorkPollInterval
			if stop <= polls {
				require.ErrorIs(t, err, context.Canceled, "count %d stop %d", count, stop)
				polls = stop
			} else {
				require.NoError(t, err, "count %d stop %d", count, stop)
			}
			require.Zero(t, n)
			require.Equal(t, polls, ctx.polls, "count %d stop %d", count, stop)
		}
	}
}
