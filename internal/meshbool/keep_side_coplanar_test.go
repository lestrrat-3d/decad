package meshbool

import (
	"errors"
	"math/big"
	"sort"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// keptVolume is the exact volume the kept facets enclose: the sum over facets
// of the signed tetrahedron against the origin. It is the whole result when
// the kept facets close, which these fixtures' contacts guarantee.
func keptVolume(kept []KeptFacet) *big.Rat {
	total := new(big.Rat)
	for _, f := range kept {
		var p [3][3]*big.Rat
		for k := range 3 {
			p[k] = [3]*big.Rat{RatCoordOf(f.V[k], 0), RatCoordOf(f.V[k], 1), RatCoordOf(f.V[k], 2)}
		}
		mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
		sub := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
		// a . (b x c)
		cx := sub(mul(p[1][1], p[2][2]), mul(p[1][2], p[2][1]))
		cy := sub(mul(p[1][2], p[2][0]), mul(p[1][0], p[2][2]))
		cz := sub(mul(p[1][0], p[2][1]), mul(p[1][1], p[2][0]))
		det := new(big.Rat).Add(new(big.Rat).Add(mul(p[0][0], cx), mul(p[0][1], cy)), mul(p[0][2], cz))
		total.Add(total, det)
	}
	return total.Quo(total, big.NewRat(6, 1))
}

func requireNoDuplicateFacets(t *testing.T, kept []KeptFacet) {
	t.Helper()
	seen := map[[3]string]struct{}{}
	for i, f := range kept {
		keys := []string{f.V[0].Key(), f.V[1].Key(), f.V[2].Key()}
		sort.Strings(keys)
		k := [3]string{keys[0], keys[1], keys[2]}
		_, dup := seen[k]
		require.False(t, dup, "kept facet %d repeats an earlier facet's corners", i)
		seen[k] = struct{}{}
	}
}

type coplanarCase struct {
	loA, hiA     r3.Vec
	loB, hiB     r3.Vec
	union        int64
	intersect    int64
	cut          int64
	noDuplicates bool
}

func runCoplanarBoolean(t *testing.T, tc coplanarCase, op OperationKind) ([]KeptFacet, Stats, error) {
	t.Helper()
	a := exactBoxMesh(t, tc.loA, tc.hiA)
	b := exactBoxMesh(t, tc.loB, tc.hiB)
	return MeshBooleanStats(coplanarCtx(t), op, a, b, NewContactMemo(a, b))
}

// TestKeepSideCoplanarEqualFootprints pins boxes meeting face to face: the
// coincident patches are boundary-only contact, so the kept facets enclose
// the hand-computed volume for each operation.
func TestKeepSideCoplanarEqualFootprints(t *testing.T) {
	tc := coplanarCase{
		loA: r3.Vec{}, hiA: r3.Vec{X: 10, Y: 10, Z: 10},
		loB: r3.Vec{Z: 10}, hiB: r3.Vec{X: 10, Y: 10, Z: 20},
		union: 2000, intersect: 0, cut: 1000,
	}
	requireCoplanarVolumes(t, tc)
}

// TestKeepSideCoplanarPartial pins a partial footprint overlap: the top of A is
// partly covered, and the kept facets carry no repeated corner triple.
func TestKeepSideCoplanarPartial(t *testing.T) {
	tc := coplanarCase{
		loA: r3.Vec{}, hiA: r3.Vec{X: 10, Y: 10, Z: 10},
		loB: r3.Vec{X: 5, Y: 5, Z: 10}, hiB: r3.Vec{X: 15, Y: 15, Z: 20},
		union: 2000, intersect: 0, cut: 1000, noDuplicates: true,
	}
	requireCoplanarVolumes(t, tc)
}

// TestKeepSideCoplanarInteriorCorner pins a partner patch corner strictly
// inside a facet with a partner diagonal ending there: B's bottom diagonal runs
// from its minimum corner (3, 3), which lies inside A's top. The segments of
// the patch rim meet at that corner without a branch, and the volume is the
// hand value.
func TestKeepSideCoplanarInteriorCorner(t *testing.T) {
	tc := coplanarCase{
		loA: r3.Vec{}, hiA: r3.Vec{X: 10, Y: 10, Z: 10},
		loB: r3.Vec{X: 3, Y: 3, Z: 10}, hiB: r3.Vec{X: 7, Y: 7, Z: 12},
		union: 1000 + 32, intersect: 0, cut: 1000, noDuplicates: true,
	}
	requireCoplanarVolumes(t, tc)
}

// TestKeepSideCoplanarSameFacing pins a box recessed in another with its top
// flush: both tops face up. Cut drops both copies of the covered top, Union
// keeps A's only.
func TestKeepSideCoplanarSameFacing(t *testing.T) {
	tc := coplanarCase{
		loA: r3.Vec{}, hiA: r3.Vec{X: 10, Y: 10, Z: 10},
		loB: r3.Vec{X: 2, Y: 2, Z: 5}, hiB: r3.Vec{X: 8, Y: 8, Z: 10},
		union: 1000, intersect: 180, cut: 820, noDuplicates: true,
	}
	requireCoplanarVolumes(t, tc)
	// Cut: neither copy of the flush top survives, so no kept facet of the cut
	// lies on z = 10 inside the pocket's footprint.
	kept, _, err := runCoplanarBoolean(t, tc, OpCut)
	require.NoError(t, err)
	for _, f := range kept {
		onTop := true
		for _, v := range f.V {
			onTop = onTop && RatCoordOf(v, 2).Cmp(big.NewRat(10, 1)) == 0
		}
		if !onTop {
			continue
		}
		c := XCentroid(f.V[0], f.V[1], f.V[2]).Vec()
		inside := c.X > 2 && c.X < 8 && c.Y > 2 && c.Y < 8
		require.False(t, inside, "a top facet of the pocket's footprint survives at %v", c)
	}
}

func requireCoplanarVolumes(t *testing.T, tc coplanarCase) {
	t.Helper()
	for _, op := range []struct {
		op   OperationKind
		want int64
	}{{OpUnion, tc.union}, {OpIntersect, tc.intersect}, {OpCut, tc.cut}} {
		kept, _, err := runCoplanarBoolean(t, tc, op.op)
		require.NoError(t, err, "%v", op.op)
		require.Zero(t, keptVolume(kept).Cmp(big.NewRat(op.want, 1)), "%v volume %v, want %d", op.op, keptVolume(kept), op.want)
		if tc.noDuplicates {
			requireNoDuplicateFacets(t, kept)
		}
	}
}

// TestKeepSideCoplanarMixedSides pins covers that disagree on the facing: the
// pass refuses with an expected contact instead of choosing a side.
func TestKeepSideCoplanarMixedSides(t *testing.T) {
	m := &BoolMesh{Norms: []proof.Xpt{proof.XptOf(r3.Vec{Z: 1})}}
	other := &BoolMesh{Norms: []proof.Xpt{proof.XptOf(r3.Vec{Z: 1}), proof.XptOf(r3.Vec{Z: -1})}}
	_, err := coverFacing(m, 0, other, []int{0, 1})
	require.Error(t, err)
	require.True(t, errors.Is(err, decaderr.ErrUnsupported), "%v", err)
	same, err := coverFacing(m, 0, other, []int{0})
	require.NoError(t, err)
	require.True(t, same)
	same, err = coverFacing(m, 0, other, []int{1})
	require.NoError(t, err)
	require.False(t, same)
}

// TestKeepSideCoplanarNoSegment pins a facet whose coplanar partner leaves no
// surviving segment (equal footprints): the facet is one region and the cover
// decides it, and a partner that shares only the facet's diagonal is no cover.
func TestKeepSideCoplanarNoSegment(t *testing.T) {
	a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
	b := exactBoxMesh(t, r3.Vec{Z: 10}, r3.Vec{X: 10, Y: 10, Z: 20})
	res, err := pairCuts(coplanarCtx(t), a, b, NewContactMemo(a, b), newKeyMemo())
	require.NoError(t, err)
	covered := 0
	for i, js := range res.CoplanarA {
		if len(res.CutsA[i]) == 0 && len(js) > 0 {
			covered++
		}
	}
	require.Positive(t, covered, "a facet has partners and no segment")
	requireCoplanarVolumes(t, coplanarCase{
		loA: r3.Vec{}, hiA: r3.Vec{X: 10, Y: 10, Z: 10},
		loB: r3.Vec{Z: 10}, hiB: r3.Vec{X: 10, Y: 10, Z: 20},
		union: 2000, intersect: 0, cut: 1000,
	})
}

// TestCoplanarKeepTable pins §3.4's table cell by cell.
func TestCoplanarKeepTable(t *testing.T) {
	for _, tc := range []struct {
		op                  OperationKind
		second, same, keeps bool
	}{
		{OpUnion, false, true, true}, {OpUnion, true, true, false}, {OpUnion, false, false, false}, {OpUnion, true, false, false},
		{OpIntersect, false, true, true}, {OpIntersect, true, true, false}, {OpIntersect, false, false, false}, {OpIntersect, true, false, false},
		{OpCut, false, true, false}, {OpCut, true, true, false}, {OpCut, false, false, true}, {OpCut, true, false, false},
	} {
		require.Equal(t, tc.keeps, coplanarKeep(tc.op, tc.second, tc.same), "%v second=%v same=%v", tc.op, tc.second, tc.same)
	}
}
