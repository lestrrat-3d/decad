package meshbool_test

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// exactEarBlocked checks endpoint identity and triangle containment with only
// rationals. It charges one budget step per index, including ear corners.
func exactEarBlocked(budget *proofbound.WorkBudget, pts []meshbool.Xp2, idx []int, i int) (bool, error) {
	n := len(idx)
	ip, in := (i-1+n)%n, (i+1)%n
	a, b, c := pts[idx[ip]], pts[idx[i]], pts[idx[in]]
	for j := range n {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if j == ip || j == i || j == in {
			continue
		}
		p := pts[idx[j]]
		if p.U.Cmp(a.U) == 0 && p.V.Cmp(a.V) == 0 {
			continue
		}
		if p.U.Cmp(c.U) == 0 && p.V.Cmp(c.V) == 0 {
			continue
		}
		d1 := meshbool.Cross2x(a, b, p).Sign()
		d2 := meshbool.Cross2x(b, c, p).Sign()
		d3 := meshbool.Cross2x(c, a, p).Sign()
		hasNeg := d1 < 0 || d2 < 0 || d3 < 0
		hasPos := d1 > 0 || d2 > 0 || d3 > 0
		if !hasNeg || !hasPos {
			return true, nil
		}
	}
	return false, nil
}

func TestEarBlockedXMatchesExactEndpointReference(t *testing.T) {
	pt := func(u, v *big.Rat) meshbool.Xp2 { return meshbool.NewXP2(u, v) }
	base := []meshbool.Xp2{
		pt(big.NewRat(0, 1), big.NewRat(0, 1)),
		pt(big.NewRat(4, 1), big.NewRat(0, 1)),
		pt(big.NewRat(0, 1), big.NewRat(4, 1)),
	}
	with := func(points ...meshbool.Xp2) []meshbool.Xp2 {
		return append(append([]meshbool.Xp2(nil), base...), points...)
	}
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 1075))
	nearOne := new(big.Rat).Add(big.NewRat(1, 1),
		new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 100)))
	huge := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 1100))
	poisoned := pt(big.NewRat(0, 1), big.NewRat(0, 1))
	poisoned.Fu = math.NaN()
	unknown := meshbool.Xp2{U: big.NewRat(0, 1), V: big.NewRat(0, 1),
		Fu: 1, Fv: 1, FloatFinite: true}
	negativeZero := pt(big.NewRat(0, 1), big.NewRat(0, 1))
	negativeZero.Fu = math.Copysign(0, -1)
	rawHalf := big.NewRat(1, 2)
	rawHalf.Num().SetInt64(2)
	rawHalf.Denom().SetInt64(4)
	shifted := []meshbool.Xp2{
		pt(big.NewRat(1, 2), big.NewRat(1, 2)),
		pt(big.NewRat(4, 1), big.NewRat(1, 2)),
		pt(big.NewRat(1, 2), big.NewRat(4, 1)),
		pt(rawHalf, big.NewRat(1, 2)),
	}
	cases := []struct {
		name string
		pts  []meshbool.Xp2
	}{
		{"interior", with(pt(big.NewRat(1, 1), big.NewRat(1, 1)))},
		{"outside triangle inside float box", with(pt(big.NewRat(3, 1), big.NewRat(3, 1)))},
		{"equal start", with(base[0])},
		{"equal end", with(base[2])},
		{"equal start and end", with(base[0], base[2])},
		{"equal rational with unreduced denominator", shifted},
		{"distinct rationals with equal rounded floats", with(pt(nearOne, big.NewRat(0, 1)))},
		{"signed zero", with(negativeZero)},
		{"subnormal underflow", with(pt(tiny, big.NewRat(0, 1)))},
		{"nonfinite conversion", []meshbool.Xp2{base[0], pt(huge, big.NewRat(0, 1)),
			pt(big.NewRat(0, 1), huge), pt(new(big.Rat).Quo(huge, big.NewRat(4, 1)),
				new(big.Rat).Quo(huge, big.NewRat(4, 1)))}},
		{"nonfinite changed cache", with(poisoned)},
		{"homogeneous projection", with(meshbool.NewXP2FromXpt(proof.Xpt{
			X: big.NewInt(2), Y: big.NewInt(2), Z: big.NewInt(0), W: big.NewInt(2)}, 0, 1))},
		{"unknown literal cache", with(unknown)},
		{"missing literal cache", with(meshbool.Xp2{U: big.NewRat(1, 1), V: big.NewRat(1, 1)})},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			idx := make([]int, len(tc.pts))
			for i := range idx {
				idx[i] = i
			}
			want, err := exactEarBlocked(proofbound.NewWorkBudget(t.Context()), tc.pts, idx, 1)
			require.NoError(t, err)
			got, err := meshbool.EarBlockedX(proofbound.NewWorkBudget(t.Context()), tc.pts, idx, 1)
			require.NoError(t, err)
			require.Equal(t, want, got)
		})
	}
}

func exactEarClip(budget *proofbound.WorkBudget, pts []meshbool.Xp2, poly []int) ([][3]int, error) {
	idx := append([]int(nil), poly...)
	tris := make([][3]int, 0, len(idx))
	for len(idx) > 3 {
		clipped := false
		for i := range idx {
			n := len(idx)
			ia, ib, ic := idx[(i-1+n)%n], idx[i], idx[(i+1)%n]
			if meshbool.Cross2x(pts[ia], pts[ib], pts[ic]).Sign() <= 0 {
				continue
			}
			blocked, err := exactEarBlocked(budget, pts, idx, i)
			if err != nil {
				return nil, err
			}
			if blocked {
				continue
			}
			tris = append(tris, [3]int{ia, ib, ic})
			idx = append(idx[:i], idx[i+1:]...)
			clipped = true
			break
		}
		if !clipped {
			return nil, errors.New("reference ear clipping stalled")
		}
	}
	if meshbool.Cross2x(pts[idx[0]], pts[idx[1]], pts[idx[2]]).Sign() > 0 {
		tris = append(tris, [3]int{idx[0], idx[1], idx[2]})
	}
	return tris, nil
}

func TestEarClipXMatchesExactTriangleOrder(t *testing.T) {
	for _, coords := range [][][2]int64{
		{{0, 0}, {3, 0}, {4, 0}, {4, 4}, {2, 4}, {0, 4}},
		{{0, 0}, {4, 0}, {4, 4}, {2, 2}, {0, 4}},
	} {
		pts := make([]meshbool.Xp2, len(coords))
		idx := make([]int, len(coords))
		for i, xy := range coords {
			pts[i] = meshbool.NewXP2(big.NewRat(xy[0], 1), big.NewRat(xy[1], 1))
			idx[i] = i
		}
		want, err := exactEarClip(proofbound.NewWorkBudget(t.Context()), pts, idx)
		require.NoError(t, err)
		got, err := meshbool.EarClipX(proofbound.NewWorkBudget(t.Context()), pts, idx)
		require.NoError(t, err)
		require.Equal(t, want, got)
	}
}

func TestEarBlockedXRetainsBudgetAndCancellation(t *testing.T) {
	pts := []meshbool.Xp2{
		meshbool.NewXP2(big.NewRat(0, 1), big.NewRat(0, 1)),
		meshbool.NewXP2(big.NewRat(4, 1), big.NewRat(0, 1)),
		meshbool.NewXP2(big.NewRat(0, 1), big.NewRat(4, 1)),
		meshbool.NewXP2(big.NewRat(3, 1), big.NewRat(3, 1)),
		meshbool.NewXP2(big.NewRat(3, 1), big.NewRat(3, 1)),
	}
	idx := []int{0, 1, 2, 3, 4}
	for _, limit := range []int{1, 3, 5, 6} {
		t.Run(big.NewInt(int64(limit)).String(), func(t *testing.T) {
			stop := errors.New("budget exhausted")
			budget := func(count *int) *proofbound.WorkBudget {
				return &proofbound.WorkBudget{StepFn: func() error {
					*count++
					if *count == limit {
						return stop
					}
					return nil
				}, ErrFn: t.Context().Err}
			}
			var gotSteps, wantSteps int
			want, wantErr := exactEarBlocked(budget(&wantSteps), pts, idx, 1)
			got, gotErr := meshbool.EarBlockedX(budget(&gotSteps), pts, idx, 1)
			require.Equal(t, wantSteps, gotSteps)
			require.Equal(t, want, got)
			require.ErrorIs(t, gotErr, wantErr)
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	budget := &proofbound.WorkBudget{StepFn: ctx.Err, ErrFn: ctx.Err}
	_, err := meshbool.EarBlockedX(budget, pts, idx, 1)
	require.ErrorIs(t, err, context.Canceled)
}

func BenchmarkEarBlockedXEndpointCandidates(b *testing.B) {
	pts := []meshbool.Xp2{
		meshbool.NewXP2(big.NewRat(0, 1), big.NewRat(0, 1)),
		meshbool.NewXP2(big.NewRat(4, 1), big.NewRat(0, 1)),
		meshbool.NewXP2(big.NewRat(0, 1), big.NewRat(4, 1)),
	}
	for i := range 64 {
		den := new(big.Int).Add(new(big.Int).Lsh(big.NewInt(1), 96), big.NewInt(int64(i+1)))
		v := new(big.Rat).Add(big.NewRat(3, 1), new(big.Rat).SetFrac(big.NewInt(1), den))
		pts = append(pts, meshbool.NewXP2(v, v))
	}
	idx := make([]int, len(pts))
	for i := range idx {
		idx[i] = i
	}
	budget := &proofbound.WorkBudget{StepFn: func() error { return nil }, ErrFn: b.Context().Err}
	b.ReportAllocs()
	for b.Loop() {
		blocked, err := meshbool.EarBlockedX(budget, pts, idx, 1)
		require.NoError(b, err)
		require.False(b, blocked)
	}
}
