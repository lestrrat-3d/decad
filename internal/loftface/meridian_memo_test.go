package loftface_test

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/hwfault"
	"github.com/lestrrat-3d/decad/internal/loftface"
)

type memoInput struct {
	first, second loftface.Map
	span          freeform.BezierSpan
	m, b          *big.Rat
}

func memoRat(r *rand.Rand) *big.Rat {
	return big.NewRat(r.Int64N(41)-20, r.Int64N(7)+1)
}

func memoVec(r *rand.Rand) loftface.RatVec {
	return loftface.RatVec{memoRat(r), memoRat(r), memoRat(r)}
}

func memoMap(r *rand.Rand) loftface.Map {
	return loftface.Map{Constant: memoVec(r), Q0: memoVec(r), QU: memoVec(r), QV: memoVec(r),
		G0: big.NewRat(r.Int64N(5)+3, 1), GU: memoRat(r), GV: memoRat(r)}
}

func memoInputOf(r *rand.Rand) memoInput {
	span := make(freeform.BezierSpan, r.IntN(3)+2)
	for i := range span {
		span[i] = freeform.RatPoint{U: memoRat(r), V: memoRat(r)}
	}
	return memoInput{first: memoMap(r), second: memoMap(r), span: span, m: memoRat(r), b: memoRat(r)}
}

func copyRat(x *big.Rat) *big.Rat { return new(big.Rat).Set(x) }

func copyVec(v loftface.RatVec) loftface.RatVec {
	return loftface.RatVec{copyRat(v[0]), copyRat(v[1]), copyRat(v[2])}
}

func copyMemoMap(mp loftface.Map) loftface.Map {
	return loftface.Map{Constant: copyVec(mp.Constant), Q0: copyVec(mp.Q0), QU: copyVec(mp.QU),
		QV: copyVec(mp.QV), G0: copyRat(mp.G0), GU: copyRat(mp.GU), GV: copyRat(mp.GV),
		Stretch: mp.Stretch}
}

func (in memoInput) fresh() memoInput {
	span := make(freeform.BezierSpan, len(in.span))
	for i, p := range in.span {
		span[i] = freeform.RatPoint{U: copyRat(p.U), V: copyRat(p.V)}
	}
	return memoInput{first: copyMemoMap(in.first), second: copyMemoMap(in.second), span: span,
		m: copyRat(in.m), b: copyRat(in.b)}
}

func requireSameRows(t *testing.T, want, got [3][]*big.Rat) {
	t.Helper()
	for i := range want {
		require.Len(t, got[i], len(want[i]))
		for j := range want[i] {
			require.Zero(t, want[i][j].Cmp(got[i][j]), "row %d entry %d", i, j)
		}
	}
}

func TestMeridianMemoMatchesDirect(t *testing.T) {
	r := rand.New(rand.NewPCG(59, 61))
	memo := loftface.NewMeridianMemo()
	var seen []memoInput
	for i := range 100 {
		in := memoInputOf(r)
		if i%3 == 2 {
			in = seen[r.IntN(len(seen))].fresh()
		}
		seen = append(seen, in)
		want := loftface.MeridianQRows(in.first, in.second, in.span, in.m, in.b)
		requireSameRows(t, want, memo.QRows(in.first, in.second, in.span, in.m, in.b))
		repeat := in.fresh()
		requireSameRows(t, want, memo.QRows(repeat.first, repeat.second, repeat.span, repeat.m, repeat.b))
		var none *loftface.MeridianMemo
		requireSameRows(t, want, none.QRows(in.first, in.second, in.span, in.m, in.b))
	}
}

func TestMeridianMemoReturnsPrivateRows(t *testing.T) {
	r := rand.New(rand.NewPCG(59, 61))
	memo := loftface.NewMeridianMemo()
	for range 20 {
		in := memoInputOf(r)
		want := loftface.MeridianQRows(in.first, in.second, in.span, in.m, in.b)
		rows := memo.QRows(in.first, in.second, in.span, in.m, in.b)
		rows[0][0].SetInt64(12345)
		hit := memo.QRows(in.first, in.second, in.span, in.m, in.b)
		requireSameRows(t, want, hit)
		hit[1][0].SetInt64(-777)
		requireSameRows(t, want, memo.QRows(in.first, in.second, in.span, in.m, in.b))
	}
}

func TestMeridianMemoIgnoresLaterInputChanges(t *testing.T) {
	r := rand.New(rand.NewPCG(59, 61))
	memo := loftface.NewMeridianMemo()
	for range 20 {
		in := memoInputOf(r)
		equal := in.fresh()
		want := loftface.MeridianQRows(in.first, in.second, in.span, in.m, in.b)
		requireSameRows(t, want, memo.QRows(in.first, in.second, in.span, in.m, in.b))
		in.m.SetInt64(9999)
		in.first.G0.SetInt64(9999)
		in.span[0].U.SetInt64(9999)
		requireSameRows(t, want, memo.QRows(equal.first, equal.second, equal.span, equal.m, equal.b))
	}
}

func TestMeridianMemoSeparatesDifferingInputs(t *testing.T) {
	r := rand.New(rand.NewPCG(59, 61))
	memo := loftface.NewMeridianMemo()
	for range 20 {
		in := memoInputOf(r)
		memo.QRows(in.first, in.second, in.span, in.m, in.b)
		for name, tweak := range map[string]func(*memoInput){
			"m":        func(x *memoInput) { x.m.Add(x.m, big.NewRat(1, 1)) },
			"b":        func(x *memoInput) { x.b.Add(x.b, big.NewRat(1, 1)) },
			"span":     func(x *memoInput) { x.span[1].V.Add(x.span[1].V, big.NewRat(1, 1)) },
			"map":      func(x *memoInput) { x.second.QU[2].Add(x.second.QU[2], big.NewRat(1, 1)) },
			"capacity": func(x *memoInput) { x.first.GV.Add(x.first.GV, big.NewRat(1, 1)) },
		} {
			other := in.fresh()
			tweak(&other)
			want := loftface.MeridianQRows(other.first, other.second, other.span, other.m, other.b)
			requireSameRows(t, want, memo.QRows(other.first, other.second, other.span, other.m, other.b))
			if name == "m" || name == "b" {
				orig := loftface.MeridianQRows(in.first, in.second, in.span, in.m, in.b)
				differs := false
				for i := range want {
					for j := range want[i] {
						differs = differs || want[i][j].Cmp(orig[i][j]) != 0
					}
				}
				require.True(t, differs, "changing %s must change the rows", name)
			}
		}
	}
}

func TestMeridianMemoNilInputBypassesMemo(t *testing.T) {
	hwfault.SkipOnWindows(t)
	r := rand.New(rand.NewPCG(59, 61))
	in := memoInputOf(r)
	in.first.GV = nil
	memo := loftface.NewMeridianMemo()
	directPanics := func() (panicked bool) {
		defer func() { panicked = recover() != nil }()
		loftface.MeridianQRows(in.first, in.second, in.span, in.m, in.b)
		return false
	}()
	if directPanics {
		require.Panics(t, func() { memo.QRows(in.first, in.second, in.span, in.m, in.b) })
		return
	}
	requireSameRows(t, loftface.MeridianQRows(in.first, in.second, in.span, in.m, in.b),
		memo.QRows(in.first, in.second, in.span, in.m, in.b))
}
