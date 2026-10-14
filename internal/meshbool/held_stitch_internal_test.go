package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"runtime"
	"slices"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// stitchSliver is two coplanar facets sharing the edge A-B: A, C, B below the
// line y = x, and A, B, P whose exact apex P lies 2^-60 off that line on the
// side the sign of offset names. Nearest rounding puts P on the line, which
// flattens the sliver; the embedding search then moves P.
func stitchSliver(offset int64) (xverts []proof.Xpt, out *StitchedMesh) {
	exactY := new(big.Rat).Add(big.NewRat(1, 2), new(big.Rat).SetFrac(big.NewInt(offset), new(big.Int).Lsh(big.NewInt(1), 60)))
	w := new(big.Int).Lsh(big.NewInt(1), 61)
	p := proof.Xpt{
		X: new(big.Int).Rsh(w, 1),
		Y: new(big.Int).Div(new(big.Int).Mul(exactY.Num(), w), exactY.Denom()),
		Z: big.NewInt(0),
		W: w,
	}
	verts := []r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 1, Y: 1, Z: 0}, {X: 1, Y: 0, Z: 0}, p.Vec()}
	xverts = []proof.Xpt{proof.XptOf(verts[0]), proof.XptOf(verts[1]), proof.XptOf(verts[2]), p}
	return xverts, &StitchedMesh{Verts: verts, Tris: [][3]int{{0, 2, 1}, {0, 1, 3}}}
}

// requireGapsMatchFinalPositions runs keepRoundedEmbedded and requires every
// returned gap to equal the exact distance to the vertex where it now sits.
// It returns whether the search moved a vertex and whether it refused.
func requireGapsMatchFinalPositions(t *testing.T, name string, xverts []proof.Xpt, remap []int, out *StitchedMesh) (moved, refused bool) {
	t.Helper()
	before := slices.Clone(out.Verts)
	gaps, err := keepRoundedEmbedded(t.Context(), xverts, remap, out)
	if err != nil {
		require.Nil(t, gaps, name)
		return false, true
	}
	require.Len(t, gaps, len(xverts), name)
	for i, p := range xverts {
		require.Zero(t, CoordDistance(p, out.Verts[remap[i]]).Cmp(gaps[i]), `%s: gap of exact vertex %d`, name, i)
	}
	return !slices.Equal(before, out.Verts), false
}

func TestKeepRoundedEmbeddedGapsMatchFinalPositions(t *testing.T) {
	t.Run("sliver that moves a vertex", func(t *testing.T) {
		xverts, out := stitchSliver(1)
		moved, refused := requireGapsMatchFinalPositions(t, "sliver+1", xverts, []int{0, 1, 2, 3}, out)
		require.True(t, moved, `the fixture must move a vertex, so the gaps are recomputed`)
		require.False(t, refused)
	})
	t.Run("sliver that moves the other way", func(t *testing.T) {
		xverts, out := stitchSliver(-1)
		requireGapsMatchFinalPositions(t, "sliver-1", xverts, []int{0, 1, 2, 3}, out)
	})
	t.Run("welded vertices", func(t *testing.T) {
		// Two exact vertices share held vertex 3: it is checked where it
		// sits and never moved, so the gaps stay as first computed.
		xverts, out := stitchSliver(1)
		xverts = append(xverts, xverts[3])
		moved, _ := requireGapsMatchFinalPositions(t, "welded", xverts, []int{0, 1, 2, 3, 3}, out)
		require.False(t, moved)
	})
	t.Run("seeded layered slivers", func(t *testing.T) {
		var movedCases, refusedCases, passedCases int
		for seed := range uint64(300) {
			h := stitchLayered(rand.New(rand.NewPCG(seed, 0)), 2+int(seed%4))
			remap := make([]int, len(h.Verts))
			for i := range remap {
				remap[i] = i
			}
			out := &StitchedMesh{Verts: slices.Clone(h.Verts), Tris: h.Tris}
			moved, refused := requireGapsMatchFinalPositions(t, fmt.Sprintf("seed %d", seed), h.Exact, remap, out)
			switch {
			case refused:
				refusedCases++
			case moved:
				movedCases++
			default:
				passedCases++
			}
		}
		require.Positive(t, movedCases)
		require.Positive(t, refusedCases)
		require.Positive(t, passedCases)
	})
}

// stitchLayered stacks sliver pairs in planes one ulp of z apart (the shape
// of the layered slivers in held_embedding_test.go), each apex 2^-60 to a
// random side of its shared edge's line, other vertices nudged one ulp at
// random. The held vertices are the nearest floats or the nudged ones.
func stitchLayered(rng *rand.Rand, layers int) HeldRounding {
	var h HeldRounding
	w := new(big.Int).Lsh(big.NewInt(1), 61)
	ratOf := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	num := func(r *big.Rat) *big.Int {
		return new(big.Int).Set(new(big.Rat).Mul(r, new(big.Rat).SetInt(w)).Num())
	}
	add := func(x, y, z float64, dy int64) int {
		ey := new(big.Rat).Add(ratOf(y), new(big.Rat).SetFrac(big.NewInt(dy), new(big.Int).Lsh(big.NewInt(1), 60)))
		exact := proof.Xpt{X: num(ratOf(x)), Y: num(ey), Z: num(ratOf(z)), W: new(big.Int).Set(w)}
		held := exact.Vec()
		switch {
		case dy != 0 && rng.IntN(2) == 0:
			held.Y = math.Nextafter(held.Y, float64(-dy)*math.Inf(1))
		case rng.IntN(6) == 0:
			dir := math.Inf(1)
			if rng.IntN(2) == 0 {
				dir = math.Inf(-1)
			}
			switch rng.IntN(3) {
			case 0:
				held.X = math.Nextafter(held.X, dir)
			case 1:
				held.Y = math.Nextafter(held.Y, dir)
			default:
				held.Z = math.Nextafter(held.Z, dir)
			}
		}
		h.Verts = append(h.Verts, held)
		h.Exact = append(h.Exact, exact)
		return len(h.Verts) - 1
	}
	for k := range layers {
		z := 1 + float64(k)*math.Ldexp(1, -52)
		sx, sy := 0.25*float64(rng.IntN(3)), 0.25*float64(rng.IntN(2))
		a := add(sx, sy, z, 0)
		b := add(sx+1, sy+1, z, 0)
		c := add(sx+1, sy, z, 0)
		p := add(sx+0.5, sy+0.5, z, int64(rng.IntN(3)-1))
		h.Tris = append(h.Tris, [3]int{a, c, b}, [3]int{a, b, p})
	}
	for i, v := range h.Verts {
		moved := CoordDistance(h.Exact[i], v).Sign() != 0
		h.Moved = append(h.Moved, moved)
		h.Movable = append(h.Movable, moved)
	}
	return h
}

func TestHeldVertexBoundsFromGapsMatches(t *testing.T) {
	rng := rand.New(rand.NewPCG(7, 11))
	for round := range 200 {
		n := 1 + rng.IntN(40)
		xverts := make([]proof.Xpt, n)
		xbeta := make([]float64, n)
		remap := make([]int, n)
		held := make([]r3.Vec, 1+rng.IntN(n))
		for i := range held {
			held[i] = r3.Vec{X: rng.NormFloat64(), Y: rng.NormFloat64(), Z: rng.NormFloat64()}
		}
		for i := range xverts {
			remap[i] = rng.IntN(len(held))
			h := held[remap[i]]
			// An exact point a random fraction of an ulp-scale step off its
			// float, or exactly on it.
			off := func(c float64) *big.Rat {
				r := new(big.Rat).SetFloat64(c)
				if rng.IntN(3) > 0 {
					r.Add(r, big.NewRat(rng.Int64N(2001)-1000, 1<<(40+rng.IntN(20))))
				}
				return r
			}
			xverts[i] = ratXpt(off(h.X), off(h.Y), off(h.Z))
			xbeta[i] = rng.Float64() * 1e-6
		}
		want, err := HeldVertexBounds(t.Context(), xverts, xbeta, remap, held)
		require.NoError(t, err)
		gaps := make([]*big.Rat, n)
		for i, p := range xverts {
			gaps[i] = CoordDistance(p, held[remap[i]])
		}
		got, err := heldVertexBoundsFromGaps(t.Context(), gaps, xbeta, remap, len(held))
		require.NoError(t, err)
		for i := range want {
			require.Equal(t, math.Float64bits(want[i]), math.Float64bits(got[i]), `round %d vertex %d`, round, i)
		}
		// Reference: the pre-change loop, one CoordDistance per use.
		ref := make([]float64, len(held))
		for i, p := range xverts {
			beta := xbeta[i]
			if gap := CoordDistance(p, held[remap[i]]); gap.Sign() > 0 {
				g, _ := gap.Float64()
				beta = proofbound.AbsSumUpper(beta, proofbound.Radius3D(proofbound.ProvenUpRound(g)))
			}
			ref[remap[i]] = max(ref[remap[i]], beta)
		}
		require.Equal(t, ref, got, `round %d`, round)
	}
}

// ratXpt is the exact point with the three rational coordinates.
func ratXpt(x, y, z *big.Rat) proof.Xpt {
	w := big.NewInt(1)
	for _, r := range []*big.Rat{x, y, z} {
		w = new(big.Int).Mul(w, r.Denom())
	}
	num := func(r *big.Rat) *big.Int {
		return new(big.Int).Mul(r.Num(), new(big.Int).Quo(w, r.Denom()))
	}
	return proof.Xpt{X: num(x), Y: num(y), Z: num(z), W: w}
}

// boxMesh is an eight-corner box facet-triangulated with outward winding,
// each corner pushed through rot and shifted by at.
func boxMesh(t *testing.T, half r3.Vec, rot func(r3.Vec) r3.Vec, at r3.Vec) *BoolMesh {
	t.Helper()
	var verts []r3.Vec
	for i := range 8 {
		c := r3.Vec{X: half.X, Y: half.Y, Z: half.Z}
		if i&1 == 0 {
			c.X = -c.X
		}
		if i&2 == 0 {
			c.Y = -c.Y
		}
		if i&4 == 0 {
			c.Z = -c.Z
		}
		p := rot(c)
		verts = append(verts, r3.Vec{X: p.X + at.X, Y: p.Y + at.Y, Z: p.Z + at.Z})
	}
	quads := [][4]int{{0, 4, 6, 2}, {1, 3, 7, 5}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 2, 3, 1}, {4, 5, 7, 6}}
	var tris [][3]int
	var src []int
	for f, q := range quads {
		tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
		src = append(src, f, f)
	}
	bm, err := PrepContext(t.Context(), verts, tris, src)
	require.NoError(t, err)
	// Distinct non-zero bounds make a wrong key visible in the kept betas.
	beta := make([]float64, len(verts))
	for i := range beta {
		beta[i] = float64(i+1) * 1e-7
	}
	bm.SetVertexBounds(beta)
	return bm
}

// TestKeepSideKeyMemoMatchesKey holds KeepSide's pointer-keyed memo to a key
// computed per corner with Xpt.Key: the kept facets and their betas must be
// identical, for every operation, on rotated boxes whose cut corners are
// shared by several pieces.
func TestKeepSideKeyMemoMatchesKey(t *testing.T) {
	rotate := func(a, b float64) func(r3.Vec) r3.Vec {
		return func(v r3.Vec) r3.Vec {
			x := v.X*math.Cos(a) - v.Y*math.Sin(a)
			y := v.X*math.Sin(a) + v.Y*math.Cos(a)
			z := v.Z*math.Cos(b) - y*math.Sin(b)
			y = v.Z*math.Sin(b) + y*math.Cos(b)
			return r3.Vec{X: x, Y: y, Z: z}
		}
	}
	identity := func(v r3.Vec) r3.Vec { return v }
	plain := func() func(proof.Xpt) string { return proof.Xpt.Key }
	for _, tc := range []struct {
		name string
		rot  func(r3.Vec) r3.Vec
		at   r3.Vec
	}{
		{"tilted", rotate(0.3, 0.2), r3.Vec{X: 0.9, Y: 0.4, Z: 0.2}},
		{"yawed", rotate(0.7, 0), r3.Vec{X: 1.1, Y: 0.3, Z: 0.1}},
		{"axis aligned offset", identity, r3.Vec{X: 0.8, Y: 0.6, Z: 0.4}},
	} {
		for _, op := range []OperationKind{OpUnion, OpCut, OpIntersect} {
			t.Run(fmt.Sprintf("%s/%v", tc.name, op), func(t *testing.T) {
				run := func(newKeyOf func() func(proof.Xpt) string) ([]KeptFacet, float64) {
					a := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, identity, r3.Vec{})
					b := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, tc.rot, tc.at)
					kept, rim, err := meshBoolean(context.Background(), op, a, b, NewContactMemo(a, b), newKeyOf)
					require.NoError(t, err)
					return kept, rim
				}
				want, wantRim := run(plain)
				got, gotRim := run(newKeyMemo)
				require.NotEmpty(t, got)
				require.Equal(t, wantRim, gotRim)
				require.Len(t, got, len(want))
				for i := range want {
					require.Equal(t, want[i].Src, got[i].Src, `facet %d`, i)
					require.Equal(t, want[i].Beta, got[i].Beta, `facet %d`, i)
					for k := range 3 {
						require.Equal(t, want[i].V[k].Key(), got[i].V[k].Key(), `facet %d corner %d`, i, k)
					}
				}
			})
		}
	}
}

// errCountCtx counts Err calls and reports cancellation from the cancelAt-th
// call on (never when cancelAt is 0).
type errCountCtx struct {
	context.Context //nolint:containedctx // a test wrapper counting Err calls
	calls           int
	cancelAt        int
	// skipScan leaves the polls serialClean's own step count makes
	// uncounted, so calls and cancelAt number the repair path's polls only.
	skipScan bool
}

func inSerialClean() bool {
	pcs := make([]uintptr, 32)
	frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
	for {
		f, more := frames.Next()
		if strings.HasSuffix(f.Function, ".serialClean") {
			return true
		}
		if !more {
			return false
		}
	}
}

func (c *errCountCtx) Err() error {
	if c.skipScan && inSerialClean() {
		return nil
	}
	c.calls++
	if c.cancelAt > 0 && c.calls >= c.cancelAt {
		return context.Canceled
	}
	return nil
}

func TestMaxGapPollsContext(t *testing.T) {
	gaps := make([]*big.Rat, 1000)
	for i := range gaps {
		gaps[i] = big.NewRat(int64(i), 7)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := maxGap(ctx, gaps)
	require.ErrorIs(t, err, context.Canceled)

	counter := &errCountCtx{Context: t.Context()}
	worst, err := maxGap(counter, gaps)
	require.NoError(t, err)
	require.Zero(t, worst.Cmp(big.NewRat(999, 7)))
	require.Equal(t, 4, counter.calls, `one poll per 256 vertices`)
}

// TestSerialCleanFallbackKeepsRepairPolling: when the memo-free first scan
// finds a failing pair, the repair path runs with the step count and polling
// points it has without that scan. A large layered case that falls back
// polls the context exactly as often as the path that never scans first, and
// a cancellation landing at any poll gives the same outcome.
func TestSerialCleanFallbackKeepsRepairPolling(t *testing.T) {
	var h HeldRounding
	for seed := range uint64(200) {
		c := stitchLayered(rand.New(rand.NewPCG(seed, 1)), 6+int(seed%40))
		probe := &errCountCtx{Context: t.Context(), skipScan: true}
		n, err := enforceHeldEmbedding(probe, cloneStitchHeld(c), false)
		if (err != nil || n > 0) && probe.calls > 4 {
			h = c
			break
		}
	}
	require.NotEmpty(t, h.Tris, `a seeded case must fall back to repair and poll more than once`)
	run := func(memo bool, cancelAt int) (int, int, error) {
		ctx := &errCountCtx{Context: t.Context(), cancelAt: cancelAt, skipScan: true}
		n, err := enforceHeldEmbedding(ctx, cloneStitchHeld(h), memo)
		return n, ctx.calls, err
	}
	_, plainCalls, _ := run(false, 0)
	_, memoCalls, _ := run(true, 0)
	require.Equal(t, plainCalls, memoCalls, `Err call count`)
	for k := 1; k <= plainCalls; k++ {
		n0, c0, err0 := run(false, k)
		n1, c1, err1 := run(true, k)
		require.Equal(t, err0 == nil, err1 == nil, `cancel at poll %d`, k)
		require.Equal(t, n0, n1, `cancel at poll %d`, k)
		require.Equal(t, c0, c1, `cancel at poll %d`, k)
	}
}

func cloneStitchHeld(h HeldRounding) HeldRounding {
	c := h
	c.Verts = slices.Clone(h.Verts)
	return c
}
