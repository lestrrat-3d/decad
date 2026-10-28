package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The ref* functions are the depth-witness search as it stood before
// depthBeyond: every candidate scans every facet of the other mesh and always
// pays PointRoundBound. They pin what the shortcuts must reproduce.

func refCertifiedInteriorDepth(p proof.Xpt, other *BoolMesh) float64 {
	pf := p.Vec()
	best := math.Inf(1)
	for i := range other.Tris {
		if PointBoxDist(pf, other.Boxes[i]) >= best {
			continue
		}
		best = math.Min(best, PointTriDistance(pf, TriCorners(other, i)))
	}
	if best <= 0 || proofbound.IsNonFinite(best) {
		return 0
	}
	return best*(1-1e-12) - PointRoundBound(p, pf)
}

func refDeepWitnessAt(ctx context.Context, p proof.Xpt, other *BoolMesh, b float64) (bool, error) {
	if refCertifiedInteriorDepth(p, other) <= b {
		return false, nil
	}
	inside, onBoundary, err := MeshParityAllContext(ctx, p, other.Parity)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return false, ctxErr
		}
		return false, nil
	}
	return inside && !onBoundary, nil
}

func refDeepWitnessInside(ctx context.Context, m *BoolMesh, closeFacets []int, other *BoolMesh, b float64) (bool, error) {
	for n, i := range closeFacets {
		if n >= MaxDepthWitnessFacets {
			break
		}
		if err := ctx.Err(); err != nil {
			return false, err
		}
		tri := m.Tris[i]
		for _, p := range facetSamplePoints(m.Xverts[tri[0]], m.Xverts[tri[1]], m.Xverts[tri[2]]) {
			deep, err := refDeepWitnessAt(ctx, p, other, b)
			if err != nil || deep {
				return deep, err
			}
		}
	}
	return false, nil
}

func refWalkFacetFromSpan(ctx context.Context, mid proof.Xpt, m *BoolMesh, fi int, other *BoolMesh, fj int, b float64) (bool, error) {
	ot := other.Tris[fj]
	origin, normal := other.Xverts[ot[0]], other.Norms[fj]
	midF := mid.Vec()
	one := big.NewInt(1)
	for _, vi := range m.Tris[fi] {
		c := m.Xverts[vi]
		if proof.XdotSign(normal, proof.Xsub(c, origin)) >= 0 {
			continue
		}
		reach := m.Verts[vi].Sub(midF).Len()
		den := big.NewInt(1)
		for range SpanWalkSteps {
			den = new(big.Int).Lsh(den, 1)
			reach /= 2
			if reach*(1+1e-6) <= b {
				break
			}
			p := proof.Xlerp(mid, c, one, den)
			deep, err := refDeepWitnessAt(ctx, p, other, b)
			if err != nil || deep {
				return deep, err
			}
		}
	}
	return false, nil
}

func refSpanWitness(ctx context.Context, bmA, bmB *BoolMesh, spans []ContactSpan, b float64) (bool, error) {
	if len(spans) == 0 {
		return false, nil
	}
	one, two := big.NewInt(1), big.NewInt(2)
	for n, s := range spans {
		if n >= MaxDepthWitnessSpans {
			break
		}
		if err := ctx.Err(); err != nil {
			return false, err
		}
		mid := proof.Xlerp(s.P0, s.P1, one, two)
		deep, err := refWalkFacetFromSpan(ctx, mid, bmA, s.I, bmB, s.J, b)
		if err != nil || deep {
			return deep, err
		}
		deep, err = refWalkFacetFromSpan(ctx, mid, bmB, s.J, bmA, s.I, b)
		if err != nil || deep {
			return deep, err
		}
	}
	return false, nil
}

func refProvenDepthExceeds(ctx context.Context, bmA *BoolMesh, closeA []int, bmB *BoolMesh, closeB []int, spans []ContactSpan, b float64) (bool, error) {
	deep, err := refDeepWitnessInside(ctx, bmA, closeA, bmB, b)
	if err != nil || deep {
		return deep, err
	}
	deep, err = refDeepWitnessInside(ctx, bmB, closeB, bmA, b)
	if err != nil || deep {
		return deep, err
	}
	return refSpanWitness(ctx, bmA, bmB, spans, b)
}

// depthCylinder is parityCylinder(n) moved by shift and prepared with its
// parity holder.
func depthCylinder(t testing.TB, n int, shift r3.Vec) *BoolMesh {
	t.Helper()
	verts, tris := parityCylinder(n)
	for i := range verts {
		verts[i] = verts[i].Add(shift)
	}
	bm, err := PrepContext(context.Background(), verts, tris, nil)
	require.NoError(t, err)
	bm.SetVertexBounds(make([]float64, len(verts)))
	return bm
}

// depthQueries are exact points on and around m: float points, every vertex,
// and exact non-float lerps between random vertex pairs, so PointRoundBound is
// both zero and positive across them.
func depthQueries(rng *rand.Rand, m *BoolMesh, count int) []proof.Xpt {
	var out []proof.Xpt
	for range count {
		out = append(out, proof.XptOf(r3.NewVec(10*rng.Float64()-5, 10*rng.Float64()-5, 5*rng.Float64()-2)))
	}
	out = append(out, m.Xverts...)
	for range count {
		a, c := m.Xverts[rng.IntN(len(m.Xverts))], m.Xverts[rng.IntN(len(m.Xverts))]
		out = append(out, proof.Xlerp(a, c, big.NewInt(int64(1+rng.IntN(6))), big.NewInt(7)))
	}
	return out
}

var depthBounds = []float64{
	math.Inf(-1), -1, math.Copysign(0, -1), 0, 1e-300, 1e-6, 0.01, 0.125, 0.5, 1, 3, 100,
	math.Inf(1), math.NaN(),
}

// TestDepthBeyondMatchesCertifiedDepth compares depthBeyond against the full
// scan for every bound, with no hint, out-of-range hints and every facet as
// the hint. Under the race detector it draws fewer random queries and, on the
// finer cylinder, takes every fifth facet as the hint.
func TestDepthBeyondMatchesCertifiedDepth(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(5, 23))
	meshes := map[string]*BoolMesh{
		"cylinder8":  depthCylinder(t, 8, r3.Vec{}),
		"cylinder32": depthCylinder(t, 32, r3.NewVec(0.375, -1.25, 0.5)),
	}
	queries, hintStride := 48, map[string]int{"cylinder8": 1, "cylinder32": 1}
	if raceDetector {
		queries, hintStride["cylinder32"] = 8, 5
	}
	for name, m := range meshes {
		hints := []int{-1, -7, len(m.Tris), len(m.Tris) + 3}
		for f := 0; f < len(m.Tris); f += hintStride[name] {
			hints = append(hints, f)
		}
		for qi, p := range depthQueries(rng, m, queries) {
			ref := refCertifiedInteriorDepth(p, m)
			for _, b := range depthBounds {
				want := !(ref <= b)
				require.Equal(t, want, depthBeyond(p, m, b, nil), "%s query %d bound %g, no hint", name, qi, b)
				for _, h := range hints {
					hint := h
					require.Equal(t, want, depthBeyond(p, m, b, &hint), "%s query %d bound %g hint %d", name, qi, b, h)
					if hint != h {
						require.False(t, want, "%s query %d: a settled hint answers shallow", name, qi)
						require.GreaterOrEqual(t, hint, 0)
						require.Less(t, hint, len(m.Tris))
					}
				}
			}
		}
	}
}

// TestDepthBeyondHintSkippedByScan covers a hint facet the scan skips on its
// box: its box is moved away from the facet, so its box distance, not its
// facet distance, bounds the scan's minimum.
func TestDepthBeyondHintSkippedByScan(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{
		{X: 5, Y: 0, Z: 0}, {X: 5, Y: 1, Z: 0}, {X: 5, Y: 0, Z: 1},
		{X: 0.25, Y: -1, Z: -1}, {X: 0.25, Y: 1, Z: -1}, {X: 0.25, Y: 0, Z: 1},
	}
	tris := [][3]int{{0, 1, 2}, {3, 4, 5}}
	m, err := PrepContext(t.Context(), verts, tris, nil)
	require.NoError(t, err)
	m.SetVertexBounds(make([]float64, len(verts)))
	m.Boxes[1] = [2]r3.Vec{{X: 20, Y: 20, Z: 20}, {X: 21, Y: 21, Z: 21}}
	m.setDepthBlocks()
	p := proof.XptOf(r3.Vec{})
	// The scan reads facet 0 at distance 5 and skips facet 1 on its box, so
	// the certified depth is about 5 although facet 1 lies 0.25 away.
	ref := refCertifiedInteriorDepth(p, m)
	require.InDelta(t, 5, ref, 1e-9)
	for _, b := range []float64{0.5, 1, 4.9} {
		hint := 1
		require.True(t, depthBeyond(p, m, b, &hint), "bound %g", b)
		require.Equal(t, 1, hint)
	}
	hint := 1
	require.False(t, depthBeyond(p, m, 5.5, &hint))
}

// depthPair is a face-pair input to the witness search: two cylinders and the
// near contacts their facet sets make within slack.
type depthPair struct {
	name  string
	a, b  *BoolMesh
	nc    NearContacts
	slack float64
}

func depthPairs(t testing.TB, n int) []depthPair {
	t.Helper()
	a := depthCylinder(t, n, r3.Vec{})
	var out []depthPair
	shifts := map[string]r3.Vec{
		"deep":    {X: 3, Z: 0.5},
		"shallow": {X: 5.9, Y: 0.1, Z: 0.125},
		"touch":   {X: 6, Z: 0.25},
		"capped":  {X: 0.5, Z: 2.9},
		"apart":   {X: 6.05, Z: 0.375},
	}
	for name, s := range shifts {
		b := depthCylinder(t, n, s)
		memo := NewContactMemo(a, b)
		for _, slack := range []float64{0.02, 0.125, 0.4, 1.5} {
			nc, deferred, err := GatherNearContacts(t.Context(), a, AllFacets(a), b, AllFacets(b), slack, memo)
			require.NoError(t, err)
			// Every shift keeps the caps off each other's planes, so no pair
			// is left to the mesh pass's coplanar refusal.
			require.False(t, deferred, "%s/%g", name, slack)
			out = append(out, depthPair{name: fmt.Sprintf("%s/%g", name, slack), a: a, b: b, nc: nc, slack: slack})
		}
	}
	return out
}

// TestProvenDepthExceedsMatchesReference runs every witness search against
// its reference copy and requires the same verdict, error and number of
// context polls. Under the race detector the cylinders are coarser and the
// search is cancelled at fewer poll counts.
func TestProvenDepthExceedsMatchesReference(t *testing.T) {
	t.Parallel()
	sides, cancels := 24, int64(16)
	if raceDetector {
		sides, cancels = 8, 2
	}
	verdicts := map[bool]int{}
	for _, pc := range depthPairs(t, sides) {
		type search func(context.Context) (bool, error)
		cases := map[string][2]search{
			"proven": {
				func(ctx context.Context) (bool, error) {
					return ProvenDepthExceeds(ctx, pc.a, pc.nc.CloseA, pc.b, pc.nc.CloseB, pc.nc.Spans, pc.slack)
				},
				func(ctx context.Context) (bool, error) {
					return refProvenDepthExceeds(ctx, pc.a, pc.nc.CloseA, pc.b, pc.nc.CloseB, pc.nc.Spans, pc.slack)
				},
			},
			"insideA": {
				func(ctx context.Context) (bool, error) {
					return DeepWitnessInside(ctx, pc.a, pc.nc.CloseA, pc.b, pc.slack)
				},
				func(ctx context.Context) (bool, error) {
					return refDeepWitnessInside(ctx, pc.a, pc.nc.CloseA, pc.b, pc.slack)
				},
			},
			"insideB": {
				func(ctx context.Context) (bool, error) {
					return DeepWitnessInside(ctx, pc.b, pc.nc.CloseB, pc.a, pc.slack)
				},
				func(ctx context.Context) (bool, error) {
					return refDeepWitnessInside(ctx, pc.b, pc.nc.CloseB, pc.a, pc.slack)
				},
			},
			"span": {
				func(ctx context.Context) (bool, error) { return SpanWitness(ctx, pc.a, pc.b, pc.nc.Spans, pc.slack) },
				func(ctx context.Context) (bool, error) { return refSpanWitness(ctx, pc.a, pc.b, pc.nc.Spans, pc.slack) },
			},
		}
		for name, c := range cases {
			gotCtx := newPollRecorder(t.Context(), math.MaxInt64)
			got, gotErr := c[0](gotCtx)
			wantCtx := newPollRecorder(t.Context(), math.MaxInt64)
			want, wantErr := c[1](wantCtx)
			require.Equal(t, wantErr, gotErr, "%s %s", pc.name, name)
			require.Equal(t, want, got, "%s %s", pc.name, name)
			require.Equal(t, wantCtx.polls.Load(), gotCtx.polls.Load(), "%s %s polls", pc.name, name)
			verdicts[want]++
			// Cancelling after every poll count up to the full run fails both
			// at the same poll.
			full := wantCtx.polls.Load()
			for k := int64(0); k < full; k += max(1, full/cancels) {
				gotCtx, wantCtx := newPollRecorder(t.Context(), k), newPollRecorder(t.Context(), k)
				got, gotErr := c[0](gotCtx)
				want, wantErr := c[1](wantCtx)
				require.Equal(t, wantErr, gotErr, "%s %s cancel at %d", pc.name, name, k)
				require.Equal(t, want, got, "%s %s cancel at %d", pc.name, name, k)
				require.Equal(t, wantCtx.polls.Load(), gotCtx.polls.Load(), "%s %s cancel at %d polls", pc.name, name, k)
			}
		}
	}
	// The fixtures reach both verdicts.
	require.Positive(t, verdicts[true])
	require.Positive(t, verdicts[false])
}
