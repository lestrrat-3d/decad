package proof

import (
	"context"
	"errors"
	"math/big"
	"math/rand"
	"testing"

	"github.com/stretchr/testify/require"
)

type keyTestBudget struct{ work, bytes, fail, failAt uint64 }

func (b *keyTestBudget) ReserveRecords(_ context.Context, n uint64) error {
	b.fail++
	if b.fail == b.failAt {
		return errKeyRefused
	}
	return nil
}
func (b *keyTestBudget) ReserveBytes(_ context.Context, n uint64) error {
	b.fail++
	if b.fail == b.failAt {
		return errKeyRefused
	}
	b.bytes += n
	return nil
}
func (b *keyTestBudget) SpendWork(_ context.Context, n uint64) error {
	b.fail++
	if b.fail == b.failAt {
		return errKeyRefused
	}
	b.work += n
	return nil
}

var errKeyRefused = errors.New("key reservation refused")

func TestReplayExactKeyMatches(t *testing.T) {
	r := rand.New(rand.NewSource(2))
	for i := range 120 {
		values := [4]*big.Int{}
		for j := range values {
			values[j] = new(big.Int).Rand(r, new(big.Int).Lsh(big.NewInt(1), uint(1+i%513)))
			if j < 3 && (i+j)%3 == 0 {
				values[j].Neg(values[j])
			}
		}
		if values[3].Sign() == 0 {
			values[3].SetInt64(1)
		}
		if i%5 == 0 {
			values[3].Lsh(big.NewInt(1), uint(i%300))
		}
		if i%7 == 0 {
			for _, x := range values {
				x.Mul(x, big.NewInt(315))
			}
		}
		p := Xpt{X: values[0], Y: values[1], Z: values[2], W: values[3]}
		budget := new(keyTestBudget)
		got, err := ReplayExactKey(t.Context(), p, budget)
		if err != nil || got != p.Key() {
			t.Fatalf("case %d: key mismatch or error %v", i, err)
		}
	}
}

func TestReplayExactKeyRefusals(t *testing.T) {
	p := Xpt{X: big.NewInt(-12345), Y: big.NewInt(6789), Z: big.NewInt(0), W: big.NewInt(98765)}
	full := new(keyTestBudget)
	if _, err := ReplayExactKey(t.Context(), p, full); err != nil {
		t.Fatal(err)
	}
	for i := uint64(1); i <= full.fail; i++ {
		_, err := ReplayExactKey(t.Context(), p, &keyTestBudget{failAt: i})
		if !errors.Is(err, errKeyRefused) {
			t.Fatalf("charge %d: %v", i, err)
		}
	}
}

func TestReplayExactKeyWideFibonacciAndScaling(t *testing.T) {
	a, b := big.NewInt(1), big.NewInt(1)
	for range 1000 {
		a, b = b, new(big.Int).Add(a, b)
	}
	p := Xpt{X: new(big.Int).Neg(new(big.Int).Set(a)), Y: new(big.Int).Set(b), Z: big.NewInt(0), W: new(big.Int).Add(a, b)}
	base, err := ReplayExactKey(t.Context(), p, new(keyTestBudget))
	if err != nil || base != p.Key() {
		t.Fatalf("wide Fibonacci key: %v", err)
	}
	factor := new(big.Int).Lsh(big.NewInt(315), 500)
	scaled := Xpt{X: new(big.Int).Mul(p.X, factor), Y: new(big.Int).Mul(p.Y, factor), Z: new(big.Int).Mul(p.Z, factor), W: new(big.Int).Mul(p.W, factor)}
	got, err := ReplayExactKey(t.Context(), scaled, new(keyTestBudget))
	if err != nil || got != scaled.Key() || got != base {
		t.Fatalf("scaled Fibonacci key: %v", err)
	}
}

func TestReplayExactKeyOrdering(t *testing.T) {
	points := []Xpt{
		{X: big.NewInt(-9), Y: big.NewInt(0), Z: big.NewInt(8), W: big.NewInt(15)},
		{X: big.NewInt(0), Y: big.NewInt(1), Z: big.NewInt(8), W: big.NewInt(16)},
		{X: big.NewInt(21), Y: big.NewInt(-8), Z: big.NewInt(3), W: big.NewInt(35)},
	}
	keys := make([]string, len(points))
	for i, p := range points {
		var err error
		keys[i], err = ReplayExactKey(t.Context(), p, new(keyTestBudget))
		if err != nil {
			t.Fatal(err)
		}
	}
	for i := range points {
		for j := range points {
			if (keys[i] < keys[j]) != (points[i].Key() < points[j].Key()) || (keys[i] == keys[j]) != (points[i].Key() == points[j].Key()) {
				t.Fatalf("order mismatch at %d, %d", i, j)
			}
		}
	}
}

type chargeTestBudget struct{ records, bytes, work uint64 }

func (b *chargeTestBudget) ReserveRecords(_ context.Context, n uint64) error {
	b.records += n
	return nil
}
func (b *chargeTestBudget) ReserveBytes(_ context.Context, n uint64) error {
	b.bytes += n
	return nil
}
func (b *chargeTestBudget) SpendWork(_ context.Context, n uint64) error {
	b.work += n
	return nil
}

func replayKeyChargeCases() []struct {
	name                 string
	p                    Xpt
	records, bytes, work uint64
} {
	huge := new(big.Int).Lsh(big.NewInt(1), 700)
	hugeM1 := new(big.Int).Sub(huge, big.NewInt(1))
	return []struct {
		name                 string
		p                    Xpt
		records, bytes, work uint64
	}{
		{name: "zero", p: Xpt{X: big.NewInt(0), Y: big.NewInt(0), Z: big.NewInt(0), W: big.NewInt(1)}, records: 3, bytes: 788, work: 53},
		{name: "negative", p: Xpt{X: big.NewInt(-12345), Y: big.NewInt(6789), Z: big.NewInt(0), W: big.NewInt(98765)}, records: 3, bytes: 812, work: 503},
		{name: "scaled", p: Xpt{X: big.NewInt(-9 * 315), Y: big.NewInt(0), Z: big.NewInt(8 * 315), W: big.NewInt(15 * 315)}, records: 3, bytes: 808, work: 352},
		{name: "pow2", p: Xpt{X: big.NewInt(21), Y: big.NewInt(-8), Z: big.NewInt(3), W: big.NewInt(64)}, records: 3, bytes: 800, work: 56},
		{name: "large", p: Xpt{X: new(big.Int).Neg(hugeM1), Y: new(big.Int).Set(huge), Z: big.NewInt(7), W: new(big.Int).Add(huge, big.NewInt(1))}, records: 3, bytes: 3124, work: 118685},
	}
}

// TestReplayExactKeyCharges pins the budget charges and key bytes. The want
// values are fixed numbers, so changing any reservation or work charge fails
// it. It also checks the key length against the raw input size that
// ReplayExactKey pre-grows its buffer to.
func TestReplayExactKeyCharges(t *testing.T) {
	for _, tc := range replayKeyChargeCases() {
		t.Run(tc.name, func(t *testing.T) {
			b := new(chargeTestBudget)
			got, err := ReplayExactKey(t.Context(), tc.p, b)
			require.NoError(t, err)
			require.True(t, got == tc.p.Key(), "key bytes differ from Key")
			raw := 36
			for _, v := range []*big.Int{tc.p.X, tc.p.Y, tc.p.Z, tc.p.W} {
				raw += (v.BitLen() + 7) / 8
			}
			require.LessOrEqual(t, len(got), raw, "key longer than its raw input size")
			require.Equal(t, [3]uint64{tc.records, tc.bytes, tc.work}, [3]uint64{b.records, b.bytes, b.work},
				"charges [records bytes work]")
		})
	}
}

func BenchmarkReplayExactKey(b *testing.B) {
	huge := new(big.Int).Lsh(big.NewInt(1), 300)
	p := Xpt{X: new(big.Int).Neg(huge), Y: new(big.Int).Add(huge, big.NewInt(3)), Z: big.NewInt(7), W: new(big.Int).Lsh(huge, 1)}
	b.ReportAllocs()
	for b.Loop() {
		if _, err := ReplayExactKey(b.Context(), p, new(chargeTestBudget)); err != nil {
			b.Fatal(err)
		}
	}
}
