package meshbool

import (
	"math"
	"reflect"
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// skipGateRun is everything a tangency-gate run can show: its error, the
// contacts of a refusal, how many times it polled the context, and the memo it
// left behind.
type skipGateRun struct {
	err      error
	contacts []TouchContact
	polls    int64
	memo     *ContactMemo
}

// skipGateCase is one operand pair for the gate.
type skipGateCase struct {
	name   string
	ma, mb *topo.Mesh
	a, b   *BoolMesh
}

// skipGateCases builds gridded prisms that touch, nearly touch, cross and
// stand apart, with mixed per-face bounds so some face pairs pass the gate's
// cheap rejections and some do not. Every mesh has its float normals prepared,
// so concurrent subtests only read them.
func skipGateCases(t *testing.T) []skipGateCase {
	t.Helper()
	mixed := func(k int) float64 { return [...]float64{0, 0.01, 0.05, 0.002}[k%4] }
	flat := func(int) float64 { return 0.02 }
	var cases []skipGateCase
	add := func(name string, n int, shift r3.Vec, deltaA, deltaB func(int) float64) {
		ma, a := gridPrism(t, r3.Vec{}, r3.NewVec(1, 1, 1), n, deltaA)
		mb, b := gridPrism(t, shift, r3.NewVec(1, 1, 1).Add(shift), n, deltaB)
		for _, side := range []struct {
			m  *topo.Mesh
			bm *BoolMesh
		}{{ma, a}, {mb, b}} {
			// A vertex's bound is the largest bound of a face that holds it.
			beta := make([]float64, len(side.m.Vertices))
			for i, tri := range side.m.Triangles {
				for _, v := range tri {
					beta[v] = max(beta[v], side.m.FaceBound[side.m.Source[i]])
				}
			}
			side.bm.SetVertexBounds(beta)
			for i := range side.bm.Tris {
				side.bm.PrepareFloatNormal(i)
			}
		}
		cases = append(cases, skipGateCase{name: name, ma: ma, mb: mb, a: a, b: b})
	}
	add("near miss", 4, r3.NewVec(0.3, 0.2, 1.03), flat, flat)
	add("flush", 3, r3.NewVec(0, 0, 1), mixed, mixed)
	add("crossing", 3, r3.NewVec(0.5, 0.5, 0.5), mixed, mixed)
	add("far apart", 3, r3.NewVec(5, 5, 5), mixed, mixed)
	add("tree sized near miss", 12, r3.NewVec(0.3, 0.2, 1.01), flat, mixed)
	// The first gate run over a mesh fills lazy caches that change how many
	// times it polls; run each once so every compared run starts warm.
	for _, c := range cases {
		runSkipGate(t, c, nil, false, 0, nil)
	}
	return cases
}

// runSkipGate runs the gate over c on a fresh memo, or on the given one, with
// skipEmptyMemo as given, cancelling at the cancelAt-th poll when it is
// positive.
func runSkipGate(t *testing.T, c skipGateCase, memo *ContactMemo, skip bool, cancelAt int64,
	evidence func(*topo.Face, *topo.Face) (bool, error)) skipGateRun {
	t.Helper()
	ctx := &gatePollCounter{Context: WithContactWorkers(t.Context(), 1), cancelAt: cancelAt, limit: math.MaxInt64}
	if memo == nil {
		memo = NewContactMemo(c.a, c.b)
	}
	err := refuseProximity(ctx, c.ma, c.mb, c.a, c.b, memo, nil, nil, nil, evidence, skip)
	run := skipGateRun{err: err, polls: ctx.polls.Load(), memo: memo}
	if refusal, ok := err.(*TouchContactError); ok {
		run.contacts = refusal.Contacts
	}
	return run
}

// requireSameSkipRun compares two runs field by field, and the memos by their
// stored contents. It reports bounded text, never a dump of a memo.
func requireSameSkipRun(t *testing.T, want, got skipGateRun, label string) {
	t.Helper()
	require.Equal(t, errText(want.err), errText(got.err), "%s: error", label)
	require.Equal(t, want.polls, got.polls, "%s: context polls", label)
	require.True(t, reflect.DeepEqual(want.contacts, got.contacts), "%s: refusal contacts differ", label)
	requireSameMemo(t, want.memo, got.memo, label)
}

func errText(err error) string {
	if err == nil {
		return ""
	}
	return err.Error()
}

// requireSameMemo requires equal stored contents, field by field.
func requireSameMemo(t *testing.T, want, got *ContactMemo, label string) {
	t.Helper()
	require.Equal(t, len(want.Values), len(got.Values), "%s: memo values", label)
	require.Equal(t, len(want.Sparse), len(got.Sparse), "%s: memo sparse entries", label)
	require.Equal(t, len(want.Far), len(got.Far), "%s: memo far entries", label)
	require.True(t, reflect.DeepEqual(want.Dense, got.Dense), "%s: dense table differs", label)
	require.True(t, reflect.DeepEqual(want.Sparse, got.Sparse), "%s: sparse map differs", label)
	require.True(t, reflect.DeepEqual(want.Far, got.Far), "%s: far map differs", label)
	require.True(t, reflect.DeepEqual(want.BlockIndex, got.BlockIndex), "%s: block index differs", label)
	require.True(t, reflect.DeepEqual(want.Arena, got.Arena), "%s: arena differs", label)
	require.True(t, reflect.DeepEqual(want.Values, got.Values), "%s: memo values differ", label)
}
