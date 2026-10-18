package meshbool_test

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// xptFromRat builds an exact proof point from three rational coordinates.
func xptFromRat(x, y, z *big.Rat) proof.Xpt {
	den := new(big.Int).Set(x.Denom())
	for _, d := range []*big.Int{y.Denom(), z.Denom()} {
		gcd := new(big.Int).GCD(nil, nil, den, d)
		den.Mul(den, new(big.Int).Quo(new(big.Int).Set(d), gcd))
	}
	coord := func(r *big.Rat) *big.Int {
		scale := new(big.Int).Quo(new(big.Int).Set(den), r.Denom())
		return new(big.Int).Mul(r.Num(), scale)
	}
	h := proof.XhpCanon(proof.Xhp{X: coord(x), Y: coord(y), Z: coord(z), W: den})
	return proof.Xpt(h)
}

func referenceMeshParityContext(ctx context.Context, p proof.Xpt, verts []r3.Vec, tris [][3]int, subset []int) (bool, bool, error) {
	for _, ray := range meshbool.AxisRays {
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
			pa := meshbool.NewXP2(meshbool.RatCoordOf(p, ray.U), meshbool.RatCoordOf(p, ray.V))
			qa := meshbool.NewXP2(polynomial.MustRatOf(meshbool.CoordOf(a, ray.U)), polynomial.MustRatOf(meshbool.CoordOf(a, ray.V)))
			qb := meshbool.NewXP2(polynomial.MustRatOf(meshbool.CoordOf(b, ray.U)), polynomial.MustRatOf(meshbool.CoordOf(b, ray.V)))
			qc := meshbool.NewXP2(polynomial.MustRatOf(meshbool.CoordOf(c, ray.U)), polynomial.MustRatOf(meshbool.CoordOf(c, ray.V)))
			s1 := meshbool.Cross2xSign(qa, qb, pa)
			s2 := meshbool.Cross2xSign(qb, qc, pa)
			s3 := meshbool.Cross2xSign(qc, qa, pa)
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
			xa, xb, xc := proof.XptOf(a), proof.XptOf(b), proof.XptOf(c)
			n := proof.Xcross(proof.Xsub(xb, xa), proof.Xsub(xc, xa))
			nAxis := meshbool.XIntCoordOf(n, ray.Axis)
			if nAxis.Sign() == 0 {
				ambiguous = true
				break
			}
			tNum := proof.XdotNum(proof.Xsub(xa, p), n)
			switch s := tNum.Sign() * nAxis.Sign() * ray.Dir; {
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
	return false, false, fmt.Errorf(`%w: every parity ray was ambiguous`, decaderr.ErrBooleanFailed)
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
	p    proof.Xpt
}

// parityCubeQueries builds the query points for the unit cube translated by o.
// The Nextafter pair straddles that cube's own x = o.X face rather than a fixed
// coordinate, so the sub-ulp fixture keeps its meaning after translation.
func parityCubeQueries(o r3.Vec) []parityCase {
	v := parityCubeVerts(o)
	return []parityCase{
		{`interior`, proof.XptOf(r3.NewVec(o.X+0.25, o.Y+0.375, o.Z+0.5625))},
		{`exterior`, proof.XptOf(r3.NewVec(o.X+2, o.Y+0.375, o.Z+0.5625))},
		{`face interior`, proof.XptOf(r3.NewVec(o.X, o.Y+0.25, o.Z+0.375))},
		{`edge`, proof.XptOf(r3.NewVec(o.X, o.Y, o.Z+0.375))},
		{`vertex`, proof.XptOf(v[0])},
		{`one ulp inside the face`, proof.XptOf(r3.NewVec(math.Nextafter(o.X, o.X+1), o.Y+0.25, o.Z+0.375))},
		{`one ulp outside the face`, proof.XptOf(r3.NewVec(math.Nextafter(o.X, o.X-1), o.Y+0.25, o.Z+0.375))},
		{`facet centroid`, meshbool.XCentroid(proof.XptOf(v[0]), proof.XptOf(v[2]), proof.XptOf(v[1]))},
		{`rational interior`, xptFromRat(
			parityRat(o.X, 1, 3), parityRat(o.Y, 1, 7), parityRat(o.Z, 5, 11))},
	}
}

// parityOutcome is one kernel's complete answer, so two arms compare field by
// field rather than through three separate returns.
type parityOutcome struct {
	inside     bool
	onBoundary bool
	err        error
}

// requireParityOutcome compares an arm's result with the reference result.
func requireParityOutcome(t *testing.T, arm string, want, got parityOutcome) {
	t.Helper()
	require.Equal(t, want.err == nil, got.err == nil,
		`%s: error presence must match the reference (reference %v, candidate %v)`, arm, want.err, got.err)
	require.Equal(t, want.inside, got.inside, `%s: inside must match the reference`, arm)
	require.Equal(t, want.onBoundary, got.onBoundary, `%s: onBoundary must match the reference`, arm)
}

// parityTetraQueries builds the query points for the unit tetrahedron.
func parityTetraQueries() []parityCase {
	v := parityTetraVerts()
	return []parityCase{
		{`interior`, proof.XptOf(r3.NewVec(0.125, 0.1875, 0.25))},
		{`exterior`, proof.XptOf(r3.NewVec(2, 0.1875, 0.25))},
		{`slanted face`, proof.XptOf(r3.NewVec(0.25, 0.25, 0.5))},
		{`vertex`, proof.XptOf(r3.NewVec(0, 0, 0))},
		{`facet centroid`, meshbool.XCentroid(proof.XptOf(v[1]), proof.XptOf(v[2]), proof.XptOf(v[3]))},
		{`rational interior`, xptFromRat(big.NewRat(1, 7), big.NewRat(1, 11), big.NewRat(1, 13))},
	}
}

// requireParityMatches runs the frozen reference and every production entry
// point on one fixture and requires them all to agree. The prepared kernel is
// asked twice on ONE cache — once with it empty, once with every projection it
// needs already materialized — because a cached entry rebuilt wrong or used as
// an arithmetic destination would only show on the warm pass. It returns the
// reference's own answer so a caller can assert a known outcome on top.
func requireParityMatches(ctx context.Context, t *testing.T, p proof.Xpt, verts []r3.Vec, tris [][3]int, subset []int) (bool, bool) {
	t.Helper()
	wantIn, wantBoundary, wantErr := referenceMeshParityContext(ctx, p, verts, tris, subset)
	want := parityOutcome{inside: wantIn, onBoundary: wantBoundary, err: wantErr}

	gotIn, gotBoundary, gotErr := meshbool.MeshParityContext(ctx, p, verts, tris, subset)
	requireParityOutcome(t, `raw wrapper`, want,
		parityOutcome{inside: gotIn, onBoundary: gotBoundary, err: gotErr})

	prepared := meshbool.NewParityMesh(verts, tris)
	coldIn, coldBoundary, coldErr := meshbool.MeshParityPreparedContext(ctx, p, prepared, subset)
	requireParityOutcome(t, `prepared cold`, want,
		parityOutcome{inside: coldIn, onBoundary: coldBoundary, err: coldErr})
	warmIn, warmBoundary, warmErr := meshbool.MeshParityPreparedContext(ctx, p, prepared, subset)
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
	t.Parallel()
	ctx := t.Context()
	origin := r3.NewVec(0, 0, 0)
	farVec := r3.NewVec(parityFar, parityFar, parityFar)

	t.Run(`unit cube`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		subset := parityIdentitySubset(len(tris))
		requireCubeQueriesMatch(ctx, t, origin, verts, tris, subset)

		// The known outcomes, asserted independently of the reference.
		inside, onBoundary := requireParityMatches(ctx, t,
			proof.XptOf(r3.NewVec(0.25, 0.375, 0.5625)), verts, tris, subset)
		require.True(t, inside, `(0.25, 0.375, 0.5625) is interior to the unit cube`)
		require.False(t, onBoundary, `an interior point is not on the boundary`)

		inside, onBoundary = requireParityMatches(ctx, t,
			proof.XptOf(r3.NewVec(2, 0.375, 0.5625)), verts, tris, subset)
		require.False(t, inside, `(2, 0.375, 0.5625) is outside the unit cube`)
		require.False(t, onBoundary, `an exterior point is not on the boundary`)

		inside, onBoundary = requireParityMatches(ctx, t,
			proof.XptOf(r3.NewVec(0, 0.25, 0.375)), verts, tris, subset)
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
				proof.XptOf(r3.NewVec(0.25, 0.375, 0.5625)), verts, tris, rest)
			require.False(t, inside, `a point in the first cube is outside the second`)
			require.False(t, onBoundary, `it is not on the second cube's boundary either`)
		})
	})

	t.Run(`empty subset`, func(t *testing.T) {
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		inside, onBoundary := requireParityMatches(ctx, t,
			proof.XptOf(r3.NewVec(0.25, 0.375, 0.5625)), verts, tris, nil)
		require.False(t, inside, `no facet can enclose the query`)
		require.False(t, onBoundary, `no facet can carry the query`)
	})

	t.Run(`context canceled before entry`, func(t *testing.T) {
		canceled, cancel := context.WithCancel(ctx)
		cancel()
		verts, tris := parityCubeVerts(origin), parityCubeTris(0)
		subset := parityIdentitySubset(len(tris))
		p := proof.XptOf(r3.NewVec(0.25, 0.375, 0.5625))

		_, _, wantErr := referenceMeshParityContext(canceled, p, verts, tris, subset)
		require.ErrorIs(t, wantErr, context.Canceled, `the reference reports the cancellation`)
		_, _, gotErr := meshbool.MeshParityContext(canceled, p, verts, tris, subset)
		require.ErrorIs(t, gotErr, context.Canceled, `the production kernel reports the cancellation`)
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
		p := proof.XptOf(r3.NewVec(0, 0, 0))

		_, _, wantErr := referenceMeshParityContext(ctx, p, verts, tris, subset)
		require.Error(t, wantErr, `the reference refuses rather than guessing`)
		requireParityMatches(ctx, t, p, verts, tris, subset)
	})
}

// parityCancelAfterErr delays cancellation until a scan reaches its second
// periodic context check.
type parityCancelAfterErr struct {
	context.Context //nolint:containedctx // deterministic cancellation fixture for one test call.
	checks          int
}

func (c *parityCancelAfterErr) Err() error {
	c.checks++
	if c.checks > 1 {
		return context.Canceled
	}
	return c.Context.Err()
}

func requirePreparedParityMatchesReference(ctx context.Context, t *testing.T, prepared *meshbool.ParityMesh, p proof.Xpt, subset []int) {
	t.Helper()
	wantIn, wantBoundary, wantErr := referenceMeshParityContext(ctx, p, prepared.Verts, prepared.Tris, subset)
	gotIn, gotBoundary, gotErr := meshbool.MeshParityPreparedContext(ctx, p, prepared, subset)
	requireParityOutcome(t, `prepared after cancellation`,
		parityOutcome{inside: wantIn, onBoundary: wantBoundary, err: wantErr},
		parityOutcome{inside: gotIn, onBoundary: gotBoundary, err: gotErr})
}

func TestPreparedParityStillAnswersAfterScanCancellation(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	verts, tris := parityScatterMesh(9)
	subset := parityIdentitySubset(len(tris))
	p := proof.XptOf(parityScatterInterior())

	refCtx := &parityCancelAfterErr{Context: ctx}
	_, _, wantErr := referenceMeshParityContext(refCtx, p, verts, tris, subset)
	require.ErrorIs(t, wantErr, context.Canceled, `the reference cancels inside its scan`)

	prepared := meshbool.NewParityMesh(verts, tris)
	gotCtx := &parityCancelAfterErr{Context: ctx}
	_, _, gotErr := meshbool.MeshParityPreparedContext(gotCtx, p, prepared, subset)
	require.ErrorIs(t, gotErr, context.Canceled, `the prepared scan cancels at the same checkpoint`)

	requirePreparedParityMatchesReference(ctx, t, prepared, p, subset)
}

// parityNamedSubset is one facet selection under a stable name.
type parityNamedSubset struct {
	name   string
	facets []int
}

// parityScatterVerts is a closed tetrahedron skewed off every axis, so none of
// its four facets projects onto a segment on any of the three planes the rays
// leave. That matters: the axis-aligned fixtures the file already had make the
// very first facet of a scan ambiguous, which ends the scan before a filter can
// skip anything, so they say nothing about how much work the filter removes.
// Every coordinate is a binary fraction and so exactly representable.
func parityScatterVerts() []r3.Vec {
	return []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0.25, 0.125),
		r3.NewVec(0.25, 1, 0.375),
		r3.NewVec(0.375, 0.5, 1),
	}
}

func parityScatterInterior() r3.Vec { return r3.NewVec(0.40625, 0.4375, 0.375) }

// parityScatterMesh builds a side×side grid of those tetrahedra spread across
// the plane axis 0 projects onto, so a query near one of them leaves most
// facets provably outside their own projected box — the case the box filter is
// for.
func parityScatterMesh(side int) ([]r3.Vec, [][3]int) {
	verts := make([]r3.Vec, 0, side*side*4)
	tris := make([][3]int, 0, side*side*4)
	for i := range side {
		for j := range side {
			base := 4 * (i*side + j)
			for _, v := range parityScatterVerts() {
				verts = append(verts, r3.NewVec(v.X, v.Y+float64(3*i), v.Z+float64(3*j)))
			}
			for _, t := range parityTetraTris() {
				tris = append(tris, [3]int{t[0] + base, t[1] + base, t[2] + base})
			}
		}
	}
	return verts, tris
}
func parityProjectedCorners(verts []r3.Vec, tri [3]int, u, v int) [3]meshbool.Xp2 {
	var out [3]meshbool.Xp2
	for k, vi := range tri {
		out[k] = meshbool.NewXP2(polynomial.MustRatOf(meshbool.CoordOf(verts[vi], u)), polynomial.MustRatOf(meshbool.CoordOf(verts[vi], v)))
	}
	return out
}

// parityFloatAreaAbstains recomputes meshbool.ParityMesh.BuildFacetBox's float area filter, so a
// case can assert that the float leg could NOT decide and the exact leg is what
// answered.
func parityFloatAreaAbstains(verts []r3.Vec, tri [3]int, u, v int) bool {
	a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
	au, av := meshbool.CoordOf(a, u), meshbool.CoordOf(a, v)
	bu, bv := meshbool.CoordOf(b, u), meshbool.CoordOf(b, v)
	cu, cv := meshbool.CoordOf(c, u), meshbool.CoordOf(c, v)
	left, right := (bu-au)*(cv-av), (bv-av)*(cu-au)
	bound := meshbool.ParityAreaErrCoef*(math.Abs(left)+math.Abs(right)) + meshbool.ParityAreaFloor
	det := left - right
	return !(det > bound || det < -bound)
}

// parityFixture names one vertex/triangle buffer.
type parityFixture struct {
	name  string
	verts []r3.Vec
	tris  [][3]int
}

// parityBoxFixtures are the meshes the box filter is asserted over: the
// axis-aligned ones the file already used, plus the scattered tetrahedra that
// actually make the filter fire.
func parityBoxFixtures() []parityFixture {
	scatterVerts, scatterTris := parityScatterMesh(4)
	farVerts, farTris := parityScatterMesh(3)
	for i, v := range farVerts {
		farVerts[i] = r3.NewVec(v.X+parityFar, v.Y+parityFar, v.Z+parityFar)
	}
	return []parityFixture{
		{`unit cube`, parityCubeVerts(r3.NewVec(0, 0, 0)), parityCubeTris(0)},
		{`unit tetrahedron`, parityTetraVerts(), parityTetraTris()},
		{`triangle soup sharing the origin`, paritySoupVerts(), paritySoupTris()},
		{`scattered tetrahedra`, scatterVerts, scatterTris},
		{`scattered tetrahedra far from the origin`, farVerts, farTris},
	}
}

// TestParityFacetBoxStatesTheProjection pins what a box says about its facet:
// the four bounds are the exact extremes of the projected triangle, and
// nondegenerate is the EXACT area sign, not the float filter's guess.
func TestParityFacetBoxStatesTheProjection(t *testing.T) {
	t.Parallel()

	t.Run(`bounds and area match the exact projection`, func(t *testing.T) {
		for _, fx := range parityBoxFixtures() {
			t.Run(fx.name, func(t *testing.T) {
				pm := meshbool.NewParityMesh(fx.verts, fx.tris)
				for _, ray := range meshbool.AxisRays {
					for ti := range fx.tris {
						box := pm.BuildFacetBox(ray.Axis, ray.U, ray.V, ti)
						q := parityProjectedCorners(fx.verts, fx.tris[ti], ray.U, ray.V)
						require.True(t, box.Built, `a built box must say so`)
						require.Equal(t, math.Min(q[0].Fu, math.Min(q[1].Fu, q[2].Fu)), box.MinU,
							`axis %d facet %d: minU is the smallest projected u`, ray.Axis, ti)
						require.Equal(t, math.Max(q[0].Fu, math.Max(q[1].Fu, q[2].Fu)), box.MaxU,
							`axis %d facet %d: maxU is the largest projected u`, ray.Axis, ti)
						require.Equal(t, math.Min(q[0].Fv, math.Min(q[1].Fv, q[2].Fv)), box.MinV,
							`axis %d facet %d: minV is the smallest projected v`, ray.Axis, ti)
						require.Equal(t, math.Max(q[0].Fv, math.Max(q[1].Fv, q[2].Fv)), box.MaxV,
							`axis %d facet %d: maxV is the largest projected v`, ray.Axis, ti)
						require.Equal(t, meshbool.Cross2xSign(q[0], q[1], q[2]) != 0, box.Nondegenerate,
							`axis %d facet %d: nondegenerate must be the exact area sign`, ray.Axis, ti)
					}
				}
			})
		}
	})

	t.Run(`a projection with no area is never usable`, func(t *testing.T) {
		for _, tc := range []struct {
			name  string
			verts []r3.Vec
		}{
			{`three collinear corners`, []r3.Vec{
				r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 2), r3.NewVec(0, 3, 6)}},
			{`a repeated corner`, []r3.Vec{
				r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 4)}},
			{`a facet flat against the swept axis`, []r3.Vec{
				r3.NewVec(0, 0, 5), r3.NewVec(1, 1, 5), r3.NewVec(2, 4, 5)}},
		} {
			t.Run(tc.name, func(t *testing.T) {
				pm := meshbool.NewParityMesh(tc.verts, [][3]int{{0, 1, 2}})
				box := pm.BuildFacetBox(0, 1, 2, 0)
				require.False(t, box.Nondegenerate,
					`a projection whose exact area is zero may never license a skip`)
				require.False(t, box.Rejects(-1e9, -1e9),
					`an unusable box rejects nothing, however far away the query is`)
			})
		}
	})

	t.Run(`a non-finite coordinate leaves the box unusable`, func(t *testing.T) {
		for _, bad := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
			verts := []r3.Vec{
				r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, bad)}
			pm := meshbool.NewParityMesh(verts, [][3]int{{0, 1, 2}})
			box := pm.BuildFacetBox(0, 1, 2, 0)
			require.False(t, box.Nondegenerate,
				`a box built over %v bounds nothing`, bad)
			require.False(t, box.Rejects(-1e9, -1e9))
		}
	})

	t.Run(`the exact leg decides an area the float filter cannot`, func(t *testing.T) {
		// The projected corners are (0, 0), (1, 1) and (2⁴⁰, 2⁴⁰+1): a true
		// area of 1 against a permanent above 2⁴¹, which is three decades
		// under the float filter's own threshold.
		const big = 1 << 40
		verts := []r3.Vec{
			r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 1), r3.NewVec(0, big, big+1)}
		tri := [3]int{0, 1, 2}
		require.True(t, parityFloatAreaAbstains(verts, tri, 1, 2),
			`the case is only meaningful while the float filter abstains`)
		q := parityProjectedCorners(verts, tri, 1, 2)
		require.NotEqual(t, 0, meshbool.Cross2xSign(q[0], q[1], q[2]),
			`and while the exact area is genuinely nonzero`)

		pm := meshbool.NewParityMesh(verts, [][3]int{tri})
		box := pm.BuildFacetBox(0, 1, 2, 0)
		require.True(t, box.Nondegenerate,
			`the exact leg must recover the area the float filter could not prove`)
		require.True(t, box.Rejects(-1, 0), `a query below minU is rejected`)
		require.False(t, box.Rejects(0, 0), `a query on the box boundary is not`)
	})
}

// parityBoxQueries returns the query points the box filter is asserted over on
// one fixture: the hand-built cube and tetrahedron cases, plus points placed
// exactly at, one ulp inside and one ulp outside a real facet's own box bounds
// on every axis — the boundary the filter's strict comparison decides.
func parityBoxQueries(fx parityFixture) []parityCase {
	out := append(parityCubeQueries(r3.NewVec(0, 0, 0)), parityTetraQueries()...)
	pm := meshbool.NewParityMesh(fx.verts, fx.tris)
	for _, ray := range meshbool.AxisRays {
		box := pm.BuildFacetBox(ray.Axis, ray.U, ray.V, 0)
		for _, edge := range []float64{box.MinU, box.MaxU} {
			for _, at := range []float64{edge,
				math.Nextafter(edge, math.Inf(-1)), math.Nextafter(edge, math.Inf(1))} {
				c := make([]float64, 3)
				c[ray.U] = at
				c[ray.V] = box.MinV
				out = append(out, parityCase{
					fmt.Sprintf(`axis %d facet 0 u-bound %v`, ray.Axis, at),
					proof.XptOf(r3.NewVec(c[0], c[1], c[2])),
				})
				// The same place reached as an exact rational a hair off the
				// bound, which is what a midpoint or centroid witness is.
				out = append(out, parityCase{
					fmt.Sprintf(`axis %d facet 0 u-bound %v minus 1/3 ulp`, ray.Axis, at),
					xptFromRat(
						parityBoxOffsetRat(c[0], ray.U == 0, at),
						parityBoxOffsetRat(c[1], ray.U == 1, at),
						parityBoxOffsetRat(c[2], ray.U == 2, at)),
				})
			}
		}
	}
	return out
}

// parityBoxOffsetRat is coordinate c exactly, except on the swept-plane axis
// the caller marks, where it is the bound minus a third of the gap to the next
// float below it — a value no float64 names, sitting inside the last ulp.
func parityBoxOffsetRat(c float64, offset bool, at float64) *big.Rat {
	r := polynomial.MustRatOf(c)
	if !offset {
		return r
	}
	gap := new(big.Rat).Sub(polynomial.MustRatOf(at), polynomial.MustRatOf(math.Nextafter(at, math.Inf(-1))))
	return new(big.Rat).Sub(r, new(big.Rat).Quo(gap, big.NewRat(3, 1)))
}

// TestParityBoxFilterOnlySkipsStrictlyOutsideFacets asserts the filter's whole
// contract at facet granularity: every facet it skips is one the unfiltered
// classification would have passed over as STRICTLY OUTSIDE. A skip that landed
// on an ambiguous or a counted facet would change the answer, and this test
// catches it on the offending facet rather than on a mesh-wide verdict that
// happens to survive it.
func TestParityBoxFilterOnlySkipsStrictlyOutsideFacets(t *testing.T) {
	t.Parallel()
	skips := 0
	for _, fx := range parityBoxFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			pm := meshbool.NewParityMesh(fx.verts, fx.tris)
			for _, c := range parityBoxQueries(fx) {
				for _, ray := range meshbool.AxisRays {
					pa := meshbool.NewXP2(meshbool.RatCoordOf(c.p, ray.U), meshbool.RatCoordOf(c.p, ray.V))
					if !pa.FloatFinite {
						continue
					}
					for ti := range fx.tris {
						box := pm.BuildFacetBox(ray.Axis, ray.U, ray.V, ti)
						if !box.Rejects(pa.Fu, pa.Fv) {
							continue
						}
						skips++
						q := parityProjectedCorners(fx.verts, fx.tris[ti], ray.U, ray.V)
						s1 := meshbool.Cross2xSign(q[0], q[1], pa)
						s2 := meshbool.Cross2xSign(q[1], q[2], pa)
						s3 := meshbool.Cross2xSign(q[2], q[0], pa)
						neg := s1 < 0 || s2 < 0 || s3 < 0
						pos := s1 > 0 || s2 > 0 || s3 > 0
						require.True(t, neg && pos,
							`%s: axis %d facet %d was skipped, but its exact signs (%d, %d, %d) are not the strictly-outside pattern`,
							c.name, ray.Axis, ti, s1, s2, s3)
					}
				}
			}
		})
	}
	require.Positive(t, skips,
		`the fixtures must actually exercise the filter, or the property above is vacuous`)
}

// TestParityBoxFilterMatchesTheUnfilteredKernel is the end-to-end differential:
// the same prepared mesh answered with the filter and with it switched off, and
// the frozen pre-filter reference beside both, over hand-built and fixed-seed
// random queries and over subsets of every shape the callers use.
func TestParityBoxFilterMatchesTheUnfilteredKernel(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	verts, tris := parityScatterMesh(4)
	n := len(tris)

	rng := rand.New(rand.NewPCG(0x5ee1, 0xb0c5))
	queries := append(parityBoxQueries(parityFixture{`scattered`, verts, tris}),
		parityCubeQueries(r3.NewVec(0, 3, 3))...)
	for range 64 {
		queries = append(queries, parityCase{
			fmt.Sprintf(`random %d`, len(queries)),
			xptFromRat(
				big.NewRat(rng.Int64N(4000)-500, 100),
				big.NewRat(rng.Int64N(4000)-500, 100),
				big.NewRat(rng.Int64N(4000)-500, 100)),
		})
	}
	// A handful of queries placed exactly on a mesh vertex, which is where the
	// unfiltered kernel reports ambiguity and a boundary.
	for range 8 {
		queries = append(queries, parityCase{
			fmt.Sprintf(`vertex %d`, len(queries)),
			proof.XptOf(verts[rng.IntN(len(verts))]),
		})
	}

	subsets := []parityNamedSubset{
		{`identity`, parityIdentitySubset(n)},
		{`empty`, nil},
		{`singleton`, []int{n / 2}},
		{`stride`, parityStrideSubset(n, 7)},
	}
	for k := range 4 {
		shuffled := parityIdentitySubset(n)
		rng.Shuffle(len(shuffled), func(i, j int) {
			shuffled[i], shuffled[j] = shuffled[j], shuffled[i]
		})
		subsets = append(subsets, parityNamedSubset{
			fmt.Sprintf(`shuffled prefix %d`, k), shuffled[:1+rng.IntN(n)]})
	}

	// ONE cache per arm across every query, so a box built for an earlier query
	// is what a later one reads.
	filtered := meshbool.NewParityMesh(verts, tris)
	plain := meshbool.NewParityMesh(verts, tris)
	plain.Unfiltered = true

	for _, subset := range subsets {
		for _, c := range queries {
			arm := subset.name + `/` + c.name
			wantIn, wantBoundary, wantErr := referenceMeshParityContext(ctx, c.p, verts, tris, subset.facets)
			want := parityOutcome{inside: wantIn, onBoundary: wantBoundary, err: wantErr}

			plainIn, plainBoundary, plainErr := meshbool.MeshParityPreparedContext(ctx, c.p, plain, subset.facets)
			requireParityOutcome(t, `unfiltered/`+arm, want,
				parityOutcome{inside: plainIn, onBoundary: plainBoundary, err: plainErr})

			gotIn, gotBoundary, gotErr := meshbool.MeshParityPreparedContext(ctx, c.p, filtered, subset.facets)
			requireParityOutcome(t, `filtered/`+arm, want,
				parityOutcome{inside: gotIn, onBoundary: gotBoundary, err: gotErr})
		}
	}
}
