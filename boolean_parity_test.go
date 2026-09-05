package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// referenceMeshParityContext is the PRE-OPTIMIZATION reference copy of
// meshParityContext, captured verbatim before the query projection was hoisted
// out of the triangle loop: same arithmetic, same branch order, same returns,
// and the same per-triangle newXP2 construction the hoist removes.
//
// It is frozen on purpose. A later production edit must NEVER be mirrored into
// this function — the whole value of the comparison is that the reference
// still states the behaviour production had before the change. If production
// and this function disagree, production changed a classification and the
// change is wrong, not this copy.
func referenceMeshParityContext(ctx context.Context, p xpt, verts []r3.Vec, tris [][3]int, subset []int) (bool, bool, error) {
	for _, ray := range axisRays {
		crossings := 0
		ambiguous := false
		onBoundary := false
		for i, ti := range subset {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return false, false, err
				}
			}
			tri := tris[ti]
			a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
			pa := newXP2(ratCoordOf(p, ray.u), ratCoordOf(p, ray.v))
			qa := newXP2(mustRatOf(coordOf(a, ray.u)), mustRatOf(coordOf(a, ray.v)))
			qb := newXP2(mustRatOf(coordOf(b, ray.u)), mustRatOf(coordOf(b, ray.v)))
			qc := newXP2(mustRatOf(coordOf(c, ray.u)), mustRatOf(coordOf(c, ray.v)))
			s1 := cross2xSign(qa, qb, pa)
			s2 := cross2xSign(qb, qc, pa)
			s3 := cross2xSign(qc, qa, pa)
			neg := s1 < 0 || s2 < 0 || s3 < 0
			pos := s1 > 0 || s2 > 0 || s3 > 0
			if neg && pos {
				continue // strictly outside the projection
			}
			if s1 == 0 || s2 == 0 || s3 == 0 {
				// On the projected boundary: the ray may graze an edge or a
				// vertex, and the count would be unreliable — try another axis.
				ambiguous = true
				break
			}
			// Strictly inside the projection: the projected area is nonzero,
			// so the plane normal's swept component cannot vanish.
			xa, xb, xc := xptOf(a), xptOf(b), xptOf(c)
			n := xcross(xsub(xb, xa), xsub(xc, xa))
			nAxis := xIntCoordOf(n, ray.axis)
			if nAxis.Sign() == 0 {
				ambiguous = true
				break
			}
			tNum := xdotNum(xsub(xa, p), n)
			switch s := tNum.Sign() * nAxis.Sign() * ray.dir; {
			case s > 0:
				crossings++
			case tNum.Sign() == 0:
				onBoundary = true
			}
			if onBoundary {
				break
			}
		}
		if onBoundary {
			return false, true, nil
		}
		if ambiguous {
			continue
		}
		return crossings%2 == 1, false, nil
	}
	return false, false, fmt.Errorf(`%w: every parity ray was ambiguous`, ErrBooleanFailed)
}

// parityCubeVerts returns the eight unit-cube corners translated by o, in the
// index order every cube fixture in this file assumes.
func parityCubeVerts(o r3.Vec) []r3.Vec {
	return []r3.Vec{
		r3.NewVec(o.X, o.Y, o.Z),
		r3.NewVec(o.X+1, o.Y, o.Z),
		r3.NewVec(o.X+1, o.Y+1, o.Z),
		r3.NewVec(o.X, o.Y+1, o.Z),
		r3.NewVec(o.X, o.Y, o.Z+1),
		r3.NewVec(o.X+1, o.Y, o.Z+1),
		r3.NewVec(o.X+1, o.Y+1, o.Z+1),
		r3.NewVec(o.X, o.Y+1, o.Z+1),
	}
}

// parityCubeTris returns the twelve consistently outward-wound cube facets,
// with every vertex index shifted by base so several cubes can share one
// vertex buffer.
func parityCubeTris(base int) [][3]int {
	src := [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
		{1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},
	}
	out := make([][3]int, len(src))
	for i, t := range src {
		out[i] = [3]int{t[0] + base, t[1] + base, t[2] + base}
	}
	return out
}

// parityTetraVerts returns the four corners of the closed unit tetrahedron.
func parityTetraVerts() []r3.Vec {
	return []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0),
		r3.NewVec(0, 0, 1),
	}
}

// parityTetraTris returns the tetrahedron's four outward-wound facets.
func parityTetraTris() [][3]int {
	return [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
}

// paritySoupVerts returns a triangle soup's vertices whose first entry is the
// origin every facet shares.
func paritySoupVerts() []r3.Vec {
	return []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0),
		r3.NewVec(0, 0, 1),
		r3.NewVec(1, 1, 1),
	}
}

// paritySoupTris returns facets that all carry vertex 0, so a query at the
// origin projects onto a facet vertex on every axis and the kernel must refuse
// rather than count an unreliable crossing.
func paritySoupTris() [][3]int {
	return [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {0, 1, 4}, {0, 4, 2}}
}

// parityIdentitySubset selects every facet in stored order.
func parityIdentitySubset(n int) []int {
	out := make([]int, n)
	for i := range n {
		out[i] = i
	}
	return out
}

// parityStrideSubset selects every facet exactly once in a deterministic
// rotated order. stride must be coprime with n, which the caller picks.
func parityStrideSubset(n, stride int) []int {
	out := make([]int, n)
	for i := range n {
		out[i] = (i * stride) % n
	}
	return out
}

// parityReverseWinding flips every facet's orientation, turning an outward
// mesh into an inward one without moving a single coordinate.
func parityReverseWinding(tris [][3]int) [][3]int {
	out := make([][3]int, len(tris))
	for i, t := range tris {
		out[i] = [3]int{t[0], t[2], t[1]}
	}
	return out
}

// parityRat is the offset o plus num/den, exactly — a query coordinate no
// float64 represents. Every fixture offset in this file is a whole number, so
// the integer conversion loses nothing.
func parityRat(o float64, num, den int64) *big.Rat {
	return new(big.Rat).Add(big.NewRat(int64(o), 1), big.NewRat(num, den))
}

// parityCase is one query point under a stable name.
type parityCase struct {
	name string
	p    xpt
}

// parityCubeQueries builds the query points for the unit cube translated by o.
// The Nextafter pair straddles that cube's own x = o.X face rather than a fixed
// coordinate, so the sub-ulp fixture keeps its meaning after translation.
func parityCubeQueries(o r3.Vec) []parityCase {
	v := parityCubeVerts(o)
	return []parityCase{
		{`interior`, xptOf(r3.NewVec(o.X+0.25, o.Y+0.375, o.Z+0.5625))},
		{`exterior`, xptOf(r3.NewVec(o.X+2, o.Y+0.375, o.Z+0.5625))},
		{`face interior`, xptOf(r3.NewVec(o.X, o.Y+0.25, o.Z+0.375))},
		{`edge`, xptOf(r3.NewVec(o.X, o.Y, o.Z+0.375))},
		{`vertex`, xptOf(v[0])},
		{`one ulp inside the face`, xptOf(r3.NewVec(math.Nextafter(o.X, o.X+1), o.Y+0.25, o.Z+0.375))},
		{`one ulp outside the face`, xptOf(r3.NewVec(math.Nextafter(o.X, o.X-1), o.Y+0.25, o.Z+0.375))},
		{`facet centroid`, xCentroid(xptOf(v[0]), xptOf(v[2]), xptOf(v[1]))},
		{`rational interior`, xptFromRat(
			parityRat(o.X, 1, 3), parityRat(o.Y, 1, 7), parityRat(o.Z, 5, 11))},
	}
}

// requireParityMatches runs the frozen reference and the production kernel on
// one fixture and requires every observable to agree: both booleans, whether an
// error came back, which sentinel it wraps, and its exact text. It returns the
// reference's own answer so a caller can assert a known outcome on top.
func requireParityMatches(ctx context.Context, t *testing.T, p xpt, verts []r3.Vec, tris [][3]int, subset []int) (bool, bool) {
	t.Helper()
	wantIn, wantBoundary, wantErr := referenceMeshParityContext(ctx, p, verts, tris, subset)
	gotIn, gotBoundary, gotErr := meshParityContext(ctx, p, verts, tris, subset)
	require.Equal(t, wantErr == nil, gotErr == nil,
		`error presence must match the reference (reference %v, production %v)`, wantErr, gotErr)
	if wantErr != nil {
		require.Equal(t, wantErr.Error(), gotErr.Error(), `error text must match the reference`)
		for _, sentinel := range []error{ErrBooleanFailed, ErrUnsupported, ErrDegenerate, context.Canceled, context.DeadlineExceeded} {
			require.Equal(t, errors.Is(wantErr, sentinel), errors.Is(gotErr, sentinel),
				`sentinel identity must match the reference for %v`, sentinel)
		}
	}
	require.Equal(t, wantIn, gotIn, `inside must match the reference`)
	require.Equal(t, wantBoundary, gotBoundary, `onBoundary must match the reference`)
	return wantIn, wantBoundary
}

// requireCubeQueriesMatch replays every cube query against one facet buffer and
// requires the production kernel to reproduce the reference on all of them.
func requireCubeQueriesMatch(ctx context.Context, t *testing.T, o r3.Vec, verts []r3.Vec, tris [][3]int, subset []int) {
	t.Helper()
	for _, c := range parityCubeQueries(o) {
		t.Run(c.name, func(t *testing.T) {
			requireParityMatches(ctx, t, c.p, verts, tris, subset)
		})
	}
}

func TestParityQueryProjectionMatchesReference(t *testing.T) {
	ctx := t.Context()
	origin := r3.NewVec(0, 0, 0)
	// 2^20 on every axis: far from the origin, yet every fixture coordinate
	// stays exactly representable, so the translated run is a true repeat.
	const far = 1048576
	farVec := r3.NewVec(far, far, far)

	t.Run(`unit cube`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		subset := parityIdentitySubset(len(tris))
		requireCubeQueriesMatch(ctx, t, origin, verts, tris, subset)

		// The known outcomes, asserted independently of the reference.
		inside, onBoundary := requireParityMatches(ctx, t,
			xptOf(r3.NewVec(0.25, 0.375, 0.5625)), verts, tris, subset)
		require.True(t, inside, `(0.25, 0.375, 0.5625) is interior to the unit cube`)
		require.False(t, onBoundary, `an interior point is not on the boundary`)

		inside, onBoundary = requireParityMatches(ctx, t,
			xptOf(r3.NewVec(2, 0.375, 0.5625)), verts, tris, subset)
		require.False(t, inside, `(2, 0.375, 0.5625) is outside the unit cube`)
		require.False(t, onBoundary, `an exterior point is not on the boundary`)

		inside, onBoundary = requireParityMatches(ctx, t,
			xptOf(r3.NewVec(0, 0.25, 0.375)), verts, tris, subset)
		require.False(t, inside, `a boundary point is not reported inside`)
		require.True(t, onBoundary, `(0, 0.25, 0.375) lies on the cube's x = 0 face`)
	})

	t.Run(`cube translated far from the origin`, func(t *testing.T) {
		verts, tris := parityCubeVerts(farVec), parityCubeTris(0)
		requireCubeQueriesMatch(ctx, t, farVec, verts, tris, parityIdentitySubset(len(tris)))
	})

	t.Run(`reversed winding`, func(t *testing.T) {
		verts := parityCubeVerts(origin)
		tris := parityReverseWinding(parityCubeTris(0))
		requireCubeQueriesMatch(ctx, t, origin, verts, tris, parityIdentitySubset(len(tris)))
	})

	t.Run(`permuted subset order`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		requireCubeQueriesMatch(ctx, t, origin, verts, tris, parityStrideSubset(len(tris), 7))
	})

	t.Run(`tetrahedron`, func(t *testing.T) {
		verts, tris := parityTetraVerts(), parityTetraTris()
		queries := []parityCase{
			{`interior`, xptOf(r3.NewVec(0.125, 0.1875, 0.25))},
			{`exterior`, xptOf(r3.NewVec(2, 0.1875, 0.25))},
			{`slanted face`, xptOf(r3.NewVec(0.25, 0.25, 0.5))},
			{`vertex`, xptOf(r3.NewVec(0, 0, 0))},
			{`facet centroid`, xCentroid(xptOf(verts[1]), xptOf(verts[2]), xptOf(verts[3]))},
			{`rational interior`, xptFromRat(big.NewRat(1, 7), big.NewRat(1, 11), big.NewRat(1, 13))},
		}
		for _, order := range []struct {
			name   string
			facets [][3]int
			subset []int
		}{
			{`stored order`, tris, parityIdentitySubset(len(tris))},
			{`permuted order`, tris, parityStrideSubset(len(tris), 3)},
			{`reversed winding`, parityReverseWinding(tris), parityIdentitySubset(len(tris))},
		} {
			t.Run(order.name, func(t *testing.T) {
				for _, c := range queries {
					t.Run(c.name, func(t *testing.T) {
						requireParityMatches(ctx, t, c.p, verts, order.facets, order.subset)
					})
				}
			})
		}
	})

	t.Run(`two disjoint cubes in one buffer`, func(t *testing.T) {
		second := r3.NewVec(3, 0, 0)
		verts := append(parityCubeVerts(origin), parityCubeVerts(second)...)
		tris := append(parityCubeTris(0), parityCubeTris(8)...)
		first, rest := parityIdentitySubset(12), parityStrideSubset(12, 7)
		for i := range rest {
			rest[i] += 12
		}

		t.Run(`first component`, func(t *testing.T) {
			requireCubeQueriesMatch(ctx, t, origin, verts, tris, first)
		})
		t.Run(`second component`, func(t *testing.T) {
			requireCubeQueriesMatch(ctx, t, second, verts, tris, rest)
		})
		t.Run(`first query against the second component`, func(t *testing.T) {
			inside, onBoundary := requireParityMatches(ctx, t,
				xptOf(r3.NewVec(0.25, 0.375, 0.5625)), verts, tris, rest)
			require.False(t, inside, `a point in the first cube is outside the second`)
			require.False(t, onBoundary, `it is not on the second cube's boundary either`)
		})
	})

	t.Run(`empty subset`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		inside, onBoundary := requireParityMatches(ctx, t,
			xptOf(r3.NewVec(0.25, 0.375, 0.5625)), verts, tris, nil)
		require.False(t, inside, `no facet can enclose the query`)
		require.False(t, onBoundary, `no facet can carry the query`)
	})

	t.Run(`context canceled before entry`, func(t *testing.T) {
		canceled, cancel := context.WithCancel(ctx)
		cancel()
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		subset := parityIdentitySubset(len(tris))
		p := xptOf(r3.NewVec(0.25, 0.375, 0.5625))

		_, _, wantErr := referenceMeshParityContext(canceled, p, verts, tris, subset)
		require.ErrorIs(t, wantErr, context.Canceled, `the reference reports the cancellation`)
		requireParityMatches(canceled, t, p, verts, tris, subset)

		// An empty subset never enters the triangle loop, so cancellation is
		// not observed there — on either side of the change.
		_, _, emptyErr := referenceMeshParityContext(canceled, p, verts, tris, nil)
		require.NoError(t, emptyErr, `an empty subset returns before any context check`)
		requireParityMatches(canceled, t, p, verts, tris, nil)
	})

	t.Run(`every ray hits a facet vertex`, func(t *testing.T) {
		verts, tris := paritySoupVerts(), paritySoupTris()
		subset := parityIdentitySubset(len(tris))
		p := xptOf(r3.NewVec(0, 0, 0))

		_, _, wantErr := referenceMeshParityContext(ctx, p, verts, tris, subset)
		require.ErrorIs(t, wantErr, ErrBooleanFailed, `the reference refuses rather than guessing`)
		require.EqualError(t, wantErr,
			`decad: boolean operation failed: every parity ray was ambiguous`,
			`the reference names the ambiguity`)
		requireParityMatches(ctx, t, p, verts, tris, subset)
	})
}

// parityBenchMesh builds one vertex/triangle buffer holding that many disjoint
// closed unit cubes spaced along x, plus a query interior to the first.
func parityBenchMesh(cubes int) ([]r3.Vec, [][3]int, []int, xpt) {
	verts := make([]r3.Vec, 0, cubes*8)
	tris := make([][3]int, 0, cubes*12)
	for k := range cubes {
		verts = append(verts, parityCubeVerts(r3.NewVec(float64(3*k), 0, 0))...)
		tris = append(tris, parityCubeTris(8*k)...)
	}
	return verts, tris, parityIdentitySubset(len(tris)), xptOf(r3.NewVec(0.25, 0.375, 0.5625))
}

// BenchmarkParityQueryProjection measures the frozen reference against
// production on one prebuilt mesh and query, so the per-triangle query
// projection the hoist removes shows up as an allocation difference. Both arms
// check the error and the classification, so a faster wrong answer cannot win.
func BenchmarkParityQueryProjection(b *testing.B) {
	const cubes = 96
	verts, tris, subset, p := parityBenchMesh(cubes)
	require.GreaterOrEqual(b, len(tris), 1024, `the benchmark mesh must exercise at least 1024 triangles`)
	ctx := b.Context()

	for _, arm := range []struct {
		name string
		fn   func(context.Context, xpt, []r3.Vec, [][3]int, []int) (bool, bool, error)
	}{
		{`reference`, referenceMeshParityContext},
		{`production`, meshParityContext},
	} {
		b.Run(arm.name, func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				inside, onBoundary, err := arm.fn(ctx, p, verts, tris, subset)
				if err != nil {
					b.Fatal(err)
				}
				if !inside || onBoundary {
					b.Fatalf(`want inside, got inside=%t onBoundary=%t`, inside, onBoundary)
				}
			}
		})
	}
}
