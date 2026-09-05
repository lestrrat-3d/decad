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

// parityFar translates a fixture 2^20 along every axis: far from the origin,
// yet every fixture coordinate stays exactly representable, so the translated
// run is a true repeat of the untranslated one.
const parityFar = 1048576

// parityBenchCubes is how many disjoint closed cubes the benchmark mesh holds —
// enough to carry well past 1,024 triangles.
const parityBenchCubes = 96

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

// paritySentinels are the sentinels a parity answer could conceivably wrap.
// Every arm must agree on each one, so a candidate cannot swap which failure it
// reports while keeping the text.
var paritySentinels = []error{ErrBooleanFailed, ErrUnsupported, ErrDegenerate, context.Canceled, context.DeadlineExceeded}

// parityOutcome is one kernel's complete answer, so two arms compare field by
// field rather than through three separate returns.
type parityOutcome struct {
	inside     bool
	onBoundary bool
	err        error
}

// requireParityOutcome requires one arm to reproduce the reference exactly:
// both booleans, whether an error came back, which sentinel it wraps, and its
// exact text. arm names the arm so a failure says which one drifted.
func requireParityOutcome(t *testing.T, arm string, want, got parityOutcome) {
	t.Helper()
	require.Equal(t, want.err == nil, got.err == nil,
		`%s: error presence must match the reference (reference %v, candidate %v)`, arm, want.err, got.err)
	if want.err != nil {
		require.Equal(t, want.err.Error(), got.err.Error(), `%s: error text must match the reference`, arm)
		for _, sentinel := range paritySentinels {
			require.Equal(t, errors.Is(want.err, sentinel), errors.Is(got.err, sentinel),
				`%s: sentinel identity must match the reference for %v`, arm, sentinel)
		}
	}
	require.Equal(t, want.inside, got.inside, `%s: inside must match the reference`, arm)
	require.Equal(t, want.onBoundary, got.onBoundary, `%s: onBoundary must match the reference`, arm)
}

// parityTetraQueries builds the query points for the unit tetrahedron.
func parityTetraQueries() []parityCase {
	v := parityTetraVerts()
	return []parityCase{
		{`interior`, xptOf(r3.NewVec(0.125, 0.1875, 0.25))},
		{`exterior`, xptOf(r3.NewVec(2, 0.1875, 0.25))},
		{`slanted face`, xptOf(r3.NewVec(0.25, 0.25, 0.5))},
		{`vertex`, xptOf(r3.NewVec(0, 0, 0))},
		{`facet centroid`, xCentroid(xptOf(v[1]), xptOf(v[2]), xptOf(v[3]))},
		{`rational interior`, xptFromRat(big.NewRat(1, 7), big.NewRat(1, 11), big.NewRat(1, 13))},
	}
}

// requireParityMatches runs the frozen reference and every production entry
// point on one fixture and requires them all to agree. The prepared kernel is
// asked twice on ONE cache — once with it empty, once with every projection it
// needs already materialized — because a cached entry rebuilt wrong or used as
// an arithmetic destination would only show on the warm pass. It returns the
// reference's own answer so a caller can assert a known outcome on top.
func requireParityMatches(ctx context.Context, t *testing.T, p xpt, verts []r3.Vec, tris [][3]int, subset []int) (bool, bool) {
	t.Helper()
	wantIn, wantBoundary, wantErr := referenceMeshParityContext(ctx, p, verts, tris, subset)
	want := parityOutcome{inside: wantIn, onBoundary: wantBoundary, err: wantErr}

	gotIn, gotBoundary, gotErr := meshParityContext(ctx, p, verts, tris, subset)
	requireParityOutcome(t, `raw wrapper`, want,
		parityOutcome{inside: gotIn, onBoundary: gotBoundary, err: gotErr})

	prepared := newParityMesh(verts, tris)
	coldIn, coldBoundary, coldErr := meshParityPreparedContext(ctx, p, prepared, subset)
	requireParityOutcome(t, `prepared cold`, want,
		parityOutcome{inside: coldIn, onBoundary: coldBoundary, err: coldErr})
	warmIn, warmBoundary, warmErr := meshParityPreparedContext(ctx, p, prepared, subset)
	requireParityOutcome(t, `prepared warm`, want,
		parityOutcome{inside: warmIn, onBoundary: warmBoundary, err: warmErr})

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
	farVec := r3.NewVec(parityFar, parityFar, parityFar)

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
		queries := parityTetraQueries()
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

// parityNamedSubset is one facet selection under a stable name.
type parityNamedSubset struct {
	name   string
	facets []int
}

// parityReuseFixture is one geometry with every subset and query the reuse test
// replays against a SINGLE prepared object built from it.
type parityReuseFixture struct {
	name    string
	verts   []r3.Vec
	tris    [][3]int
	subsets []parityNamedSubset
	queries []parityCase
}

// parityReuseFixtures returns every geometry, subset and query fixture the
// reference comparison uses, regrouped so each geometry is a single prepared
// object the whole group shares.
func parityReuseFixtures() []parityReuseFixture {
	origin := r3.NewVec(0, 0, 0)
	farVec := r3.NewVec(parityFar, parityFar, parityFar)
	second := r3.NewVec(3, 0, 0)
	cube := parityCubeTris(0)
	tetra := parityTetraTris()

	identity12 := parityNamedSubset{`stored order`, parityIdentitySubset(12)}
	stride12 := parityNamedSubset{`permuted order`, parityStrideSubset(12, 7)}

	secondCube := parityStrideSubset(12, 7)
	for i := range secondCube {
		secondCube[i] += 12
	}

	return []parityReuseFixture{
		{
			name:    `unit cube`,
			verts:   parityCubeVerts(origin),
			tris:    cube,
			subsets: []parityNamedSubset{identity12, stride12},
			queries: parityCubeQueries(origin),
		},
		{
			name:    `cube translated far from the origin`,
			verts:   parityCubeVerts(farVec),
			tris:    cube,
			subsets: []parityNamedSubset{identity12},
			queries: parityCubeQueries(farVec),
		},
		{
			name:    `reversed winding`,
			verts:   parityCubeVerts(origin),
			tris:    parityReverseWinding(cube),
			subsets: []parityNamedSubset{identity12},
			queries: parityCubeQueries(origin),
		},
		{
			name:  `tetrahedron`,
			verts: parityTetraVerts(),
			tris:  tetra,
			subsets: []parityNamedSubset{
				{`stored order`, parityIdentitySubset(len(tetra))},
				{`permuted order`, parityStrideSubset(len(tetra), 3)},
			},
			queries: parityTetraQueries(),
		},
		{
			name:  `two disjoint cubes in one buffer`,
			verts: append(parityCubeVerts(origin), parityCubeVerts(second)...),
			tris:  append(parityCubeTris(0), parityCubeTris(8)...),
			subsets: []parityNamedSubset{
				{`first component`, parityIdentitySubset(12)},
				{`second component`, secondCube},
				{`empty`, nil},
			},
			queries: append(parityCubeQueries(origin), parityCubeQueries(second)...),
		},
		{
			name:    `every ray hits a facet vertex`,
			verts:   paritySoupVerts(),
			tris:    paritySoupTris(),
			subsets: []parityNamedSubset{{`stored order`, parityIdentitySubset(len(paritySoupTris()))}},
			queries: []parityCase{{`origin`, xptOf(r3.NewVec(0, 0, 0))}},
		},
	}
}

// requirePreparedMatchesReference requires one query against one prepared
// object to reproduce the frozen reference on the same buffers.
func requirePreparedMatchesReference(ctx context.Context, t *testing.T, arm string, prepared *parityMesh, p xpt, subset []int) {
	t.Helper()
	wantIn, wantBoundary, wantErr := referenceMeshParityContext(ctx, p, prepared.verts, prepared.tris, subset)
	gotIn, gotBoundary, gotErr := meshParityPreparedContext(ctx, p, prepared, subset)
	requireParityOutcome(t, arm,
		parityOutcome{inside: wantIn, onBoundary: wantBoundary, err: wantErr},
		parityOutcome{inside: gotIn, onBoundary: gotBoundary, err: gotErr})
}

// parityAllocatedAxes reports which of the three axis projection slices the
// cache has allocated at all.
func parityAllocatedAxes(pm *parityMesh) [3]bool {
	var out [3]bool
	for axis := range out {
		out[axis] = pm.projections[axis] != nil
	}
	return out
}

// parityFilledVertices returns, in index order, the vertices whose projection
// on axis is materialized. A nil u is the unfilled marker; an exact zero
// coordinate is a non-nil rational, so a filled entry never reads as unfilled.
func parityFilledVertices(pm *parityMesh, axis int) []int {
	var out []int
	for vi, e := range pm.projections[axis] {
		if e.u != nil {
			out = append(out, vi)
		}
	}
	return out
}

// parityCacheEntry is one materialized projection, kept for a later comparison:
// its identity in the cache, the two rational pointers the entry holds, and
// their exact printed values.
type parityCacheEntry struct {
	axis, vi int
	u, v     *big.Rat
	us, vs   string
}

// parityCacheSnapshot records every materialized entry of the cache.
func parityCacheSnapshot(pm *parityMesh) []parityCacheEntry {
	var out []parityCacheEntry
	for axis := range pm.projections {
		for _, vi := range parityFilledVertices(pm, axis) {
			e := pm.projections[axis][vi]
			out = append(out, parityCacheEntry{
				axis: axis, vi: vi,
				u: e.u, v: e.v,
				us: e.u.RatString(), vs: e.v.RatString(),
			})
		}
	}
	return out
}

// requireSnapshotPreserved requires every entry recorded earlier to still be
// present, still be the SAME *big.Rat, and still hold the same value: a later
// query must neither rebuild an entry nor use one as an arithmetic destination.
func requireSnapshotPreserved(t *testing.T, want []parityCacheEntry, pm *parityMesh) {
	t.Helper()
	require.NotEmpty(t, want, `the snapshot must cover at least one materialized entry`)
	for _, e := range want {
		require.NotNil(t, pm.projections[e.axis], `axis %d must still be allocated`, e.axis)
		got := pm.projections[e.axis][e.vi]
		require.Same(t, e.u, got.u, `axis %d vertex %d must keep its u rational`, e.axis, e.vi)
		require.Same(t, e.v, got.v, `axis %d vertex %d must keep its v rational`, e.axis, e.vi)
		require.Equal(t, e.us, got.u.RatString(), `axis %d vertex %d must keep its u value`, e.axis, e.vi)
		require.Equal(t, e.vs, got.v.RatString(), `axis %d vertex %d must keep its v value`, e.axis, e.vi)
	}
}

// parityCancelAfterErr wraps a parent context so that Err reports cancellation
// only from the check after clean onward. The parity scan then stops at a
// deterministic facet INSIDE the subset rather than before the first one. The
// wrapper counts, so each arm needs its own: sharing one would cancel the two
// arms at different facets.
type parityCancelAfterErr struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	checks          int
	clean           int
}

func (c *parityCancelAfterErr) Err() error {
	c.checks++
	if c.checks > c.clean {
		return context.Canceled
	}
	return c.Context.Err()
}

func TestPreparedParityProjectionReuse(t *testing.T) {
	ctx := t.Context()
	origin := r3.NewVec(0, 0, 0)

	t.Run(`every fixture through one prepared object`, func(t *testing.T) {
		for _, fixture := range parityReuseFixtures() {
			t.Run(fixture.name, func(t *testing.T) {
				// ONE cache for the whole group: every later query in it reads
				// projections an earlier query materialized.
				prepared := newParityMesh(fixture.verts, fixture.tris)
				for _, subset := range fixture.subsets {
					for _, c := range fixture.queries {
						requirePreparedMatchesReference(ctx, t,
							subset.name+`/`+c.name, prepared, c.p, subset.facets)
					}
				}
			})
		}
	})

	t.Run(`interleaved queries repeat identically`, func(t *testing.T) {
		second := r3.NewVec(3, 0, 0)
		verts := append(parityCubeVerts(origin), parityCubeVerts(second)...)
		tris := append(parityCubeTris(0), parityCubeTris(8)...)
		first := parityIdentitySubset(12)
		rest := parityStrideSubset(12, 7)
		for i := range rest {
			rest[i] += 12
		}
		prepared := newParityMesh(verts, tris)
		p := xptOf(r3.NewVec(0.25, 0.375, 0.5625))

		firstIn, firstBoundary, firstErr := meshParityPreparedContext(ctx, p, prepared, first)
		require.NoError(t, firstErr)
		require.True(t, firstIn, `the query is interior to the first cube`)
		require.False(t, firstBoundary)

		// Different queries against different subsets, in between.
		for _, subset := range []parityNamedSubset{{`first`, first}, {`second`, rest}, {`empty`, nil}} {
			for _, c := range append(parityCubeQueries(origin), parityCubeQueries(second)...) {
				requirePreparedMatchesReference(ctx, t,
					subset.name+`/`+c.name, prepared, c.p, subset.facets)
			}
		}

		againIn, againBoundary, againErr := meshParityPreparedContext(ctx, p, prepared, first)
		require.NoError(t, againErr, `the repeated query must not start failing`)
		require.Equal(t, firstIn, againIn, `the repeated query must return its first answer`)
		require.Equal(t, firstBoundary, againBoundary, `the repeated query must keep its boundary verdict`)
	})

	t.Run(`each coordinate projection is allocated only when its ray is swept`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		subset := parityIdentitySubset(len(tris))
		for _, tc := range []struct {
			name  string
			query r3.Vec
			axes  [3]bool
		}{
			// (y, z) projects clear of every cube-face diagonal, so the first
			// ray decides and no later axis is ever swept.
			{`first ray decides`, r3.NewVec(0.25, 0.375, 0.5625), [3]bool{true, false, false}},
			// y + z = 1 puts the projection on the x-face diagonal, so the x
			// rays are ambiguous and the y rays decide.
			{`falls back to the second axis`, r3.NewVec(0.25, 0.375, 0.625), [3]bool{true, true, false}},
			// y + z = 1 and z = x: both the x and the y rays are ambiguous, and
			// the z rays decide.
			{`falls back to the third axis`, r3.NewVec(0.25, 0.75, 0.25), [3]bool{true, true, true}},
		} {
			t.Run(tc.name, func(t *testing.T) {
				prepared := newParityMesh(verts, tris)
				p := xptOf(tc.query)
				requirePreparedMatchesReference(ctx, t, tc.name, prepared, p, subset)
				require.Equal(t, tc.axes, parityAllocatedAxes(prepared),
					`only the axes actually swept may hold a projection slice`)

				inside, onBoundary, err := meshParityPreparedContext(ctx, p, prepared, subset)
				require.NoError(t, err)
				require.True(t, inside, `the query is interior to the unit cube`)
				require.False(t, onBoundary)
			})
		}
	})

	t.Run(`cached rationals survive later queries`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		subset := parityIdentitySubset(len(tris))
		prepared := newParityMesh(verts, tris)

		// Sweep all three axes first, so the snapshot covers every slot.
		requirePreparedMatchesReference(ctx, t, `seed`, prepared,
			xptOf(r3.NewVec(0.25, 0.75, 0.25)), subset)
		snapshot := parityCacheSnapshot(prepared)
		require.Equal(t, [3]bool{true, true, true}, parityAllocatedAxes(prepared))

		for _, c := range parityCubeQueries(origin) {
			requirePreparedMatchesReference(ctx, t, c.name, prepared, c.p, subset)
		}
		requireSnapshotPreserved(t, snapshot, prepared)
	})

	t.Run(`two prepared meshes keep their own coordinates`, func(t *testing.T) {
		second := r3.NewVec(3, 0, 0)
		firstMesh := newParityMesh(parityCubeVerts(origin), parityCubeTris(0))
		secondMesh := newParityMesh(parityCubeVerts(second), parityCubeTris(0))
		subset := parityIdentitySubset(12)

		for _, c := range parityCubeQueries(origin) {
			requirePreparedMatchesReference(ctx, t, `first/`+c.name, firstMesh, c.p, subset)
			requirePreparedMatchesReference(ctx, t, `second/`+c.name, secondMesh, c.p, subset)
		}
		for _, c := range parityCubeQueries(second) {
			requirePreparedMatchesReference(ctx, t, `first/`+c.name, firstMesh, c.p, subset)
			requirePreparedMatchesReference(ctx, t, `second/`+c.name, secondMesh, c.p, subset)
		}

		// Vertex 6 is (1, 1, 1) in the first mesh and (4, 1, 1) in the second.
		// Axis 0 projects onto (y, z), which the offset leaves alone, so the
		// separation shows on an axis the offset does move.
		firstIn, _, err := meshParityPreparedContext(ctx,
			xptOf(r3.NewVec(3.25, 0.375, 0.5625)), firstMesh, subset)
		require.NoError(t, err)
		require.False(t, firstIn, `a point in the second cube is outside the first`)
		secondIn, _, err := meshParityPreparedContext(ctx,
			xptOf(r3.NewVec(3.25, 0.375, 0.5625)), secondMesh, subset)
		require.NoError(t, err)
		require.True(t, secondIn, `and interior to the second`)
	})

	t.Run(`an empty subset allocates nothing`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		prepared := newParityMesh(verts, tris)
		inside, onBoundary, err := meshParityPreparedContext(ctx,
			xptOf(r3.NewVec(0.25, 0.375, 0.5625)), prepared, nil)
		require.NoError(t, err)
		require.False(t, inside)
		require.False(t, onBoundary)
		require.Equal(t, [3]bool{false, false, false}, parityAllocatedAxes(prepared),
			`no ray was swept, so no axis may hold a projection slice`)
	})

	t.Run(`a canceled context returns before the cache fills`, func(t *testing.T) {
		canceled, cancel := context.WithCancel(ctx)
		cancel()
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		subset := parityIdentitySubset(len(tris))
		prepared := newParityMesh(verts, tris)

		_, _, err := meshParityPreparedContext(canceled,
			xptOf(r3.NewVec(0.25, 0.375, 0.5625)), prepared, subset)
		require.ErrorIs(t, err, context.Canceled, `a nonempty subset observes the cancellation`)
		require.Equal(t, [3]bool{false, false, false}, parityAllocatedAxes(prepared),
			`the context check runs before the first projection`)
	})

	t.Run(`a component query leaves the other component unprojected`, func(t *testing.T) {
		second := r3.NewVec(3, 0, 0)
		verts := append(parityCubeVerts(origin), parityCubeVerts(second)...)
		tris := append(parityCubeTris(0), parityCubeTris(8)...)
		prepared := newParityMesh(verts, tris)

		requirePreparedMatchesReference(ctx, t, `first component`, prepared,
			xptOf(r3.NewVec(0.25, 0.375, 0.5625)), parityIdentitySubset(12))

		for axis := range prepared.projections {
			if prepared.projections[axis] == nil {
				continue
			}
			require.Equal(t, []int{0, 1, 2, 3, 4, 5, 6, 7}, parityFilledVertices(prepared, axis),
				`axis %d must project the queried component's vertices and no others`, axis)
		}
	})

	t.Run(`cancellation inside the scan leaves the cache usable`, func(t *testing.T) {
		verts, tris, subset, p := parityBenchMesh(parityBenchCubes)
		require.Greater(t, len(subset), 256,
			`the subset must outrun one context-check block for the cancellation to land inside the scan`)

		// One fresh counting context per arm: it is stateful, so sharing it
		// would cancel the two arms at different facets.
		refCtx := &parityCancelAfterErr{Context: ctx, clean: 1}
		//nolint:contextcheck // the counting wrapper IS the fixture: it derives from ctx and only delays the cancellation.
		_, _, wantErr := referenceMeshParityContext(refCtx, p, verts, tris, subset)
		require.ErrorIs(t, wantErr, context.Canceled, `the reference stops mid-scan`)

		prepared := newParityMesh(verts, tris)
		gotCtx := &parityCancelAfterErr{Context: ctx, clean: 1}
		//nolint:contextcheck // same wrapper, its own counter, so both arms stop at the same facet.
		_, _, gotErr := meshParityPreparedContext(gotCtx, p, prepared, subset)
		require.ErrorIs(t, gotErr, context.Canceled, `the prepared kernel stops mid-scan too`)
		require.Equal(t, wantErr.Error(), gotErr.Error(), `and reports the same error`)
		require.NotEmpty(t, parityFilledVertices(prepared, 0),
			`the facets before the cancellation still filled their projections`)

		// The half-filled cache must still answer correctly.
		requirePreparedMatchesReference(ctx, t, `after cancellation`, prepared, p, subset)
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

// requireBenchInside checks one benchmark iteration's answer, so a faster wrong
// answer cannot win an arm.
func requireBenchInside(b *testing.B, inside, onBoundary bool, err error) {
	b.Helper()
	if err != nil {
		b.Fatal(err)
	}
	if !inside || onBoundary {
		b.Fatalf(`want inside, got inside=%t onBoundary=%t`, inside, onBoundary)
	}
}

// BenchmarkParityQueryProjection measures the frozen reference against every
// production entry point on one prebuilt mesh and query, so the projections each
// step removes show up as an allocation difference.
//
// The prepared arms separate the two costs the cache trades between: `prepared
// warm` builds the cache once outside the timing, which is what a real operand
// does across the many queries of one operation, while `prepared cold` rebuilds
// it every iteration so setup cost cannot hide behind reuse.
func BenchmarkParityQueryProjection(b *testing.B) {
	verts, tris, subset, p := parityBenchMesh(parityBenchCubes)
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
				requireBenchInside(b, inside, onBoundary, err)
			}
		})
	}

	b.Run(`prepared warm`, func(b *testing.B) {
		prepared := newParityMesh(verts, tris)
		b.ReportAllocs()
		for b.Loop() {
			inside, onBoundary, err := meshParityPreparedContext(ctx, p, prepared, subset)
			requireBenchInside(b, inside, onBoundary, err)
		}
	})

	b.Run(`prepared cold`, func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			inside, onBoundary, err := meshParityPreparedContext(ctx, p, newParityMesh(verts, tris), subset)
			requireBenchInside(b, inside, onBoundary, err)
		}
	})
}
