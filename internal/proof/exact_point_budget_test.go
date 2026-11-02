package proof

import (
	"errors"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestXptOfBounded(t *testing.T) {
	refused := errors.New("record limit")
	for _, v := range []r3.Vec{{}, {X: 1, Y: -2, Z: 3}, {X: math.MaxFloat64, Y: math.SmallestNonzeroFloat64, Z: -1},
		{X: math.Float64frombits(0x000fffffffffffff), Y: -0.5, Z: math.SmallestNonzeroFloat64}} {
		var charged uint64
		p, err := XptOfBounded(v, func(n uint64) error { charged = n; return nil })
		require.NoError(t, err)
		require.Positive(t, charged)
		want := XptOf(v)
		require.Zero(t, p.X.Cmp(want.X))
		require.Zero(t, p.Y.Cmp(want.Y))
		require.Zero(t, p.Z.Cmp(want.Z))
		require.Zero(t, p.W.Cmp(want.W))
		_, err = XptOfBounded(v, func(n uint64) error { require.Equal(t, charged, n); return refused })
		require.ErrorIs(t, err, refused)
	}
	_, err := XptOfBounded(r3.Vec{X: math.Inf(1)}, func(uint64) error { t.Fatal("nonfinite input was charged"); return nil })
	require.Error(t, err)
}

func TestTriangleNormalReplayWideSpan(t *testing.T) {
	for _, tri := range [][3]r3.Vec{
		{{X: 1, Y: math.Ldexp(1, -119)}, {X: 2, Y: 1}, {X: 3, Z: -2}},
		{{X: 1, Y: math.Ldexp(1, -120)}, {X: 2, Y: 1}, {X: 3, Z: 2}},
		{{X: math.Copysign(0, -1), Y: math.Ldexp(1, -121)}, {X: -2, Y: 1}, {X: 3, Z: 2}},
		{{X: 1, Y: math.Ldexp(1, -144)}, {X: 2, Y: 1}, {X: 3, Z: 2}},
		{{X: math.MaxFloat64, Y: math.SmallestNonzeroFloat64}, {Y: 1}, {Z: 1}},
	} {
		var records, work uint64
		got, err := TriangleNormalReplay(tri[0], tri[1], tri[2], func(n uint64) error { records += n; return nil }, func(n uint64) error { work += n; return nil })
		if err != nil {
			t.Fatal(err)
		}
		a, b, c := DyVec(tri[0]), DyVec(tri[1]), DyVec(tri[2])
		d := DvCross(DvSub(b, a), DvSub(c, a))
		want := Xpt(XhpStripTwosOwned(XhpOfDyadics(d[0], d[1], d[2])))
		if got.X.Cmp(want.X) != 0 || got.Y.Cmp(want.Y) != 0 || got.Z.Cmp(want.Z) != 0 || got.W.Cmp(want.W) != 0 || records == 0 || work == 0 {
			t.Fatal("wide normal or charges differ")
		}
		refused := errors.New("work limit")
		if _, err := TriangleNormalReplay(tri[0], tri[1], tri[2], func(uint64) error { return nil }, func(uint64) error { return refused }); !errors.Is(err, refused) {
			t.Fatalf("work refusal: %v", err)
		}
	}
}

func TestTriangleNormalReplayChargesBeforeEachStage(t *testing.T) {
	tri := [3]r3.Vec{{X: 1, Y: math.SmallestNonzeroFloat64}, {X: math.Ldexp(1, 100), Y: 1}, {X: 3, Z: 2}}
	var reserveCalls, workCalls int
	_, err := TriangleNormalReplay(tri[0], tri[1], tri[2], func(uint64) error { reserveCalls++; return nil }, func(uint64) error { workCalls++; return nil })
	if err != nil {
		t.Fatal(err)
	}
	stop := errors.New("stopped")
	for stage := 1; stage <= reserveCalls; stage++ {
		calls := 0
		_, err := TriangleNormalReplay(tri[0], tri[1], tri[2], func(uint64) error {
			calls++
			if calls == stage {
				return stop
			}
			return nil
		}, func(uint64) error { return nil })
		if !errors.Is(err, stop) || calls != stage {
			t.Fatalf("record stage %d was not stopped", stage)
		}
	}
	for stage := 1; stage <= workCalls; stage++ {
		calls := 0
		_, err := TriangleNormalReplay(tri[0], tri[1], tri[2], func(uint64) error { return nil }, func(uint64) error {
			calls++
			if calls == stage {
				return stop
			}
			return nil
		})
		if !errors.Is(err, stop) || calls != stage {
			t.Fatalf("work stage %d was not stopped", stage)
		}
	}
	if err := normalReserveLimbs(func(uint64) error { return nil }, ^uint64(0), 1); err == nil {
		t.Fatal("limb multiplication overflow was accepted")
	}
	if err := normalReserveLimbs(func(uint64) error { return nil }, 1, ^uint64(0)); err == nil {
		t.Fatal("object count overflow was accepted")
	}
}

func TestTriangleNormalReplayMixedWidths(t *testing.T) {
	rng := rand.New(rand.NewPCG(45, 91))
	for sample := range 200 {
		var tri [3]r3.Vec
		for i := range tri {
			values := [3]*float64{&tri[i].X, &tri[i].Y, &tri[i].Z}
			for _, p := range values {
				*p = math.Ldexp(rng.Float64()-.5, rng.IntN(1200)-1000)
			}
		}
		got, err := TriangleNormalReplay(tri[0], tri[1], tri[2], func(uint64) error { return nil }, func(uint64) error { return nil })
		if err != nil {
			t.Fatal(err)
		}
		a, b, c := DyVec(tri[0]), DyVec(tri[1]), DyVec(tri[2])
		d := DvCross(DvSub(b, a), DvSub(c, a))
		want := Xpt(XhpStripTwosOwned(XhpOfDyadics(d[0], d[1], d[2])))
		if got.X.Cmp(want.X) != 0 || got.Y.Cmp(want.Y) != 0 || got.Z.Cmp(want.Z) != 0 || got.W.Cmp(want.W) != 0 {
			t.Fatalf("mixed-width normal %d differs: x=%d y=%d z=%d w=%d", sample, got.X.Cmp(want.X), got.Y.Cmp(want.Y), got.Z.Cmp(want.Z), got.W.Cmp(want.W))
		}
	}
}

func TestTriangleNormalBounded(t *testing.T) {
	refused := errors.New("record limit")
	for _, tri := range [][3]r3.Vec{
		{{}, {X: 1}, {Y: 1}},
		{{X: 100, Y: 200, Z: 300}, {X: 101, Y: 201, Z: 300}, {X: 101, Y: 200, Z: 302}},
		{{X: math.Ldexp(1, 900)}, {X: math.Ldexp(1, 900), Y: math.Ldexp(1, 850)}, {X: math.Ldexp(1, 900), Z: math.Ldexp(1, 850)}},
	} {
		var charge uint64
		got, err := TriangleNormalBounded(tri[0], tri[1], tri[2], func(n uint64) error { charge = n; return nil })
		require.NoError(t, err)
		require.Positive(t, charge)
		a, b, c := DyVec(tri[0]), DyVec(tri[1]), DyVec(tri[2])
		d := DvCross(DvSub(b, a), DvSub(c, a))
		want := Xpt(XhpStripTwosOwned(XhpOfDyadics(d[0], d[1], d[2])))
		require.Zero(t, got.X.Cmp(want.X))
		require.Zero(t, got.Y.Cmp(want.Y))
		require.Zero(t, got.Z.Cmp(want.Z))
		require.Zero(t, got.W.Cmp(want.W))
		_, err = TriangleNormalBounded(tri[0], tri[1], tri[2], func(n uint64) error { require.Equal(t, charge, n); return refused })
		require.ErrorIs(t, err, refused)
	}
	_, err := TriangleNormalBounded(r3.Vec{X: math.MaxFloat64}, r3.Vec{Y: math.SmallestNonzeroFloat64}, r3.Vec{Z: 1}, func(uint64) error { t.Fatal("wide span was charged"); return nil })
	require.Error(t, err)
}
