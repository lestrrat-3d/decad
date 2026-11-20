package prismcells_test

import (
	"errors"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func carrierLine(x0, y0, x1, y1 float64) sectionrecord.LineSeg {
	return sectionrecord.LineSeg{
		Start: sectionrecord.Point2{U: x0, V: y0},
		End:   sectionrecord.Point2{U: x1, V: y1},
	}
}

func TestSameLineCarrier(t *testing.T) {
	base := carrierLine(0, 0, 2, 2)
	for _, tc := range []struct {
		name string
		line sectionrecord.LineSeg
		want bool
	}{
		{"separate span", carrierLine(3, 3, 4, 4), true},
		{"reversed span", carrierLine(4, 4, 3, 3), true},
		{"parallel offset", carrierLine(0, 1, 2, 3), false},
		{"crossing", carrierLine(1, 0, 1, 2), false},
		{"one endpoint only", carrierLine(1, 1, 1, 2), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t, tc.want, prismcells.SameLineCarrier(base, tc.line))
		})
	}
}

// referenceSameLineCarrier is the all-rational per-pair test: the cross
// product (a.End − a.Start) × (p − a.Start) must be exactly zero at both
// endpoints p of b.
func referenceSameLineCarrier(a, b sectionrecord.LineSeg) bool {
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	sub := func(x, y float64) *big.Rat { return new(big.Rat).Sub(rat(x), rat(y)) }
	dx, dy := sub(a.End.U, a.Start.U), sub(a.End.V, a.Start.V)
	for _, p := range []sectionrecord.Point2{b.Start, b.End} {
		ex, ey := sub(p.U, a.Start.U), sub(p.V, a.Start.V)
		cross := new(big.Rat).Sub(new(big.Rat).Mul(dx, ey), new(big.Rat).Mul(dy, ex))
		if cross.Sign() != 0 {
			return false
		}
	}
	return true
}

// referenceSharedLineCarrier is the per-pair scan over
// referenceSameLineCarrier, charging one step per cross-owner pair.
func referenceSharedLineCarrier(budget *proofbound.WorkBudget, lines []prismcells.OwnedLine) (bool, error) {
	for i, line := range lines {
		for _, prior := range lines[:i] {
			if prior.Owner == line.Owner {
				continue
			}
			if err := budget.Step(); err != nil {
				return false, err
			}
			if referenceSameLineCarrier(prior.Line, line.Line) {
				return true, nil
			}
		}
	}
	return false, nil
}

// countingBudget counts Step calls and fails the call after limit steps
// succeeded, when limit is nonzero.
func countingBudget(limit int) (*proofbound.WorkBudget, *int) {
	steps := 0
	return &proofbound.WorkBudget{StepFn: func() error {
		if limit > 0 && steps == limit {
			return errStepLimit
		}
		steps++
		return nil
	}}, &steps
}

var errStepLimit = errors.New("step limit")

func TestSharedLineCarrierMatchesReference(t *testing.T) {
	up := func(x float64) float64 { return math.Nextafter(x, math.Inf(1)) }
	owned := func(owner int, line sectionrecord.LineSeg) prismcells.OwnedLine {
		return prismcells.OwnedLine{Owner: owner, Line: line}
	}
	// Each box edge is its own carrier; the sets below add one line per case.
	box := func(owner int, x0, y0, x1, y1 float64) []prismcells.OwnedLine {
		return []prismcells.OwnedLine{
			owned(owner, carrierLine(x0, y0, x1, y0)), owned(owner, carrierLine(x1, y0, x1, y1)),
			owned(owner, carrierLine(x1, y1, x0, y1)), owned(owner, carrierLine(x0, y1, x0, y0)),
		}
	}
	with := func(base []prismcells.OwnedLine, extra ...prismcells.OwnedLine) []prismcells.OwnedLine {
		return append(append([]prismcells.OwnedLine(nil), base...), extra...)
	}
	diag := []prismcells.OwnedLine{owned(0, carrierLine(0.1, 0.1, 0.7, 0.7)), owned(0, carrierLine(0.7, 0.7, 0.1, 0.9))}
	half := []prismcells.OwnedLine{owned(0, carrierLine(0.2, 0.1, 0.6, 0.3)), owned(0, carrierLine(0.6, 0.3, 0.3, 0.9))}
	for _, tc := range []struct {
		name   string
		lines  []prismcells.OwnedLine
		shared bool
	}{
		{"identical lines", with(box(0, 0, 0, 2, 2), owned(1, carrierLine(0, 0, 2, 0))), true},
		{"collinear overlapping", with(box(0, 0, 0, 2, 2), owned(1, carrierLine(3, 2, 1, 2))), true},
		{"same owner only", with(box(0, 0, 0, 2, 2), owned(0, carrierLine(5, 0, 6, 0))), false},
		{"parallel offset", with(box(0, 0, 0, 2, 2), owned(1, carrierLine(0, up(2), 2, up(2)))), false},
		{"nearly collinear start", with(box(0, 0, 0, 2, 2), owned(1, carrierLine(3, up(2), 1, 2))), false},
		{"nearly collinear diagonal", with(box(0, 0, 0, 2, 2), owned(1, carrierLine(0.5, 0.5, 3, up(3)))), false},
		{"non-dyadic collinear", with(diag, owned(1, carrierLine(0.3, 0.3, 1.9, 1.9))), true},
		{"non-dyadic half slope", with(half, owned(1, carrierLine(1.4, 0.7, 0.8, 0.4))), true},
		{"non-dyadic half slope off by an ulp", with(half, owned(1, carrierLine(1.4, up(0.7), 0.8, 0.4))), false},
		{"non-dyadic crossing", with(diag, owned(1, carrierLine(0.3, 0.7, 3.7, 0.1))), false},
		{"huge span", with(box(0, 0, 0, 1, 1), owned(1, carrierLine(0, 0, 1e300, 1e300)),
			owned(2, carrierLine(3e299, 3e299, 7e299, 7e299))), true},
		{"subnormal span", with(box(0, 0, 0, 1, 1), owned(1, carrierLine(0, 0, 5e-324, 5e-324)),
			owned(2, carrierLine(1e-323, 1e-323, 2, 2))), true},
		{"subnormal miss", with(box(0, 0, 0, 1, 1), owned(1, carrierLine(0, 0, 5e-324, 1e-323)),
			owned(2, carrierLine(1e-323, 1e-323, 2, 2))), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			wantBudget, wantSteps := countingBudget(0)
			want, err := referenceSharedLineCarrier(wantBudget, tc.lines)
			require.NoError(t, err)
			require.Equal(t, tc.shared, want, "reference result")

			gotBudget, gotSteps := countingBudget(0)
			got, err := prismcells.SharedLineCarrier(gotBudget, tc.lines)
			require.NoError(t, err)
			require.Equal(t, want, got, "shared carrier")
			require.Equal(t, *wantSteps, *gotSteps, "step charge")
			requirePairsMatch(t, tc.lines)
		})
	}
}

// TestSharedLineCarrierRandomSets compares generated non-dyadic sets, where
// a share is planted on the line y = k·x by scaling one point exactly.
func TestSharedLineCarrierRandomSets(t *testing.T) {
	rng := rand.New(rand.NewPCG(7, 11))
	coord := func() float64 { return float64(rng.IntN(2000)-1000) / 10 }
	for set := range 200 {
		var lines []prismcells.OwnedLine
		for owner := range 3 {
			for range 6 {
				lines = append(lines, prismcells.OwnedLine{Owner: owner,
					Line: carrierLine(coord(), coord(), coord(), coord())})
			}
		}
		if set%4 == 0 {
			x, k := coord(), float64(1+rng.IntN(4))
			lines[1].Line = carrierLine(x, k*x, 2*x, 2*k*x)
			if set%8 == 0 {
				lines[13].Line = carrierLine(4*x, 4*k*x, 8*x, 8*k*x)
			} else {
				lines[13].Line = carrierLine(4*x, math.Nextafter(4*k*x, 0), 8*x, 8*k*x)
			}
		}
		wantBudget, wantSteps := countingBudget(0)
		want, err := referenceSharedLineCarrier(wantBudget, lines)
		require.NoError(t, err)
		gotBudget, gotSteps := countingBudget(0)
		got, err := prismcells.SharedLineCarrier(gotBudget, lines)
		require.NoError(t, err)
		require.Equal(t, want, got, "set %d shared carrier", set)
		require.Equal(t, *wantSteps, *gotSteps, "set %d step charge", set)
		requirePairsMatch(t, lines)
	}
}

func TestSharedLineCarrierBudgetStop(t *testing.T) {
	lines := []prismcells.OwnedLine{
		{Owner: 0, Line: carrierLine(0, 0, 1, 0)}, {Owner: 0, Line: carrierLine(1, 0, 1, 1)},
		{Owner: 1, Line: carrierLine(0, 2, 1, 3)}, {Owner: 1, Line: carrierLine(1, 3, 0, 3)},
		{Owner: 2, Line: carrierLine(5, 5, 6, 7)}, {Owner: 2, Line: carrierLine(6, 7, 5, 9)},
	}
	fullBudget, full := countingBudget(0)
	shared, err := referenceSharedLineCarrier(fullBudget, lines)
	require.NoError(t, err)
	require.False(t, shared)
	require.Equal(t, 12, *full, "cross-owner pairs")
	for limit := 1; limit < *full; limit++ {
		budget, steps := countingBudget(limit)
		_, err := prismcells.SharedLineCarrier(budget, lines)
		require.ErrorIs(t, err, errStepLimit, "limit %d", limit)
		require.Equal(t, limit, *steps, "limit %d", limit)
	}
}

// requirePairsMatch compares every ordered pair through SameLineCarrier and
// the all-rational reference, reporting only the first differing pair.
func requirePairsMatch(t *testing.T, lines []prismcells.OwnedLine) {
	t.Helper()
	for i, a := range lines {
		for j, b := range lines {
			want := referenceSameLineCarrier(a.Line, b.Line)
			got := prismcells.SameLineCarrier(a.Line, b.Line)
			require.Equal(t, want, got, "pair (%d, %d)", i, j)
		}
	}
}
