package loftmesh

import (
	"errors"
	"math"
	"runtime"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/stretchr/testify/require"
)

// rowFillCase is a scan that fills its list by rows, with its S8 count.
type rowFillCase struct {
	name string
	scan interface {
		pairScan
		loftRowScan
	}
	members []int
	count   sweepCount
}

// rowFillCases builds the cases: every grouped fixture, then a plain tree. With
// first set it stops after the first.
func rowFillCases(t *testing.T, first bool) []rowFillCase {
	var cases []rowFillCase
	fixtures := groupFixtures()
	for _, f := range fixtures {
		if first && len(cases) == 1 {
			return cases
		}
		scan := f.scan()
		members := allMembers(len(f.boxes))
		sc, err := sweepCandidateCountsWorkers(unlimitedBudget(t), scan, len(f.boxes), keepEveryPair, math.MaxUint64, 1)
		require.NoError(t, err)
		cases = append(cases, rowFillCase{name: f.name, scan: scan, members: members, count: sc})
	}
	// A plain tree scan, over the boxes of the "only -1" fixture.
	only := fixtures[3]
	members := allMembers(len(only.boxes))
	scan := newTreeScan(only.boxes, members)
	sc, err := sweepCandidateCountsWorkers(unlimitedBudget(t), scan, len(only.boxes), keepEveryPair, math.MaxUint64, 1)
	require.NoError(t, err)
	return append(cases, rowFillCase{name: "plain tree", scan: scan, members: members, count: sc})
}

func TestRowFillMatchesSweepCandidates(t *testing.T) {
	for _, c := range rowFillCases(t, false) {
		t.Run(c.name, func(t *testing.T) {
			sc := c.count
			require.Positive(t, sc.count)
			want, err := sweepCandidates(unlimitedBudget(t), c.scan, sc.starts, sc.count, keepEveryPair)
			require.NoError(t, err)
			got, err := sweepCandidatesRowsWorkers(unlimitedBudget(t), c.scan, c.members, sc.starts, sc.count,
				sc.scanned, 4)
			require.NoError(t, err)
			require.Equal(t, want.starts, got.starts)
			require.Equal(t, want.js, got.js)
			for i := range len(got.starts) - 1 {
				row := got.js[got.starts[i]:got.starts[i+1]]
				require.True(t, slices.IsSorted(row), "row %d", i)
				require.Equal(t, len(row), len(slices.Compact(slices.Clone(row))), "row %d", i)
			}

			// A budget that fails part-way fails at the same step with the same
			// error as the pair walk. Two cases cover it, to keep the test fast.
			if c.name != "walls between fans" && c.name != "plain tree" {
				return
			}
			for _, k := range []uint64{0, sc.scanned / 2, sc.scanned} {
				var wantSteps, gotSteps uint64
				want, wantErr := sweepCandidates(countingBudget(k, &wantSteps), c.scan, sc.starts, sc.count,
					keepEveryPair)
				got, gotErr := sweepCandidatesRowsWorkers(countingBudget(k, &gotSteps), c.scan, c.members,
					sc.starts, sc.count, sc.scanned, 4)
				require.Equal(t, want, got, "budget %d", k)
				require.Equal(t, wantErr, gotErr, "budget %d", k)
				require.Equal(t, wantSteps, gotSteps, "budget %d", k)
			}
		})
	}
}

func TestRowFillRefusesCountMismatch(t *testing.T) {
	c := rowFillCases(t, true)[0]
	starts := slices.Clone(c.count.starts)
	moved := false
	for i := 0; i+2 < len(starts); i++ {
		if starts[i+1] > starts[i] {
			starts[i+1]--
			moved = true
			break
		}
	}
	require.True(t, moved)
	_, err := sweepCandidatesRowsWorkers(unlimitedBudget(t), c.scan, c.members, starts, c.count.count,
		c.count.scanned, 4)
	require.Error(t, err)
	require.True(t, errors.Is(err, decaderr.ErrUnsupported))
	require.ErrorContains(t, err, "differ from S8 count")
}

func TestRowFillDispatch(t *testing.T) {
	c := rowFillCases(t, true)[0]
	sc := c.count
	want, err := sweepCandidates(unlimitedBudget(t), c.scan, sc.starts, sc.count, keepEveryPair)
	require.NoError(t, err)

	t.Run("few members", func(t *testing.T) {
		got, err := sweepCandidatesByRows(unlimitedBudget(t), c.scan, c.members[:loftRowParallelMin-1], sc.starts,
			sc.count, sc.scanned)
		require.NoError(t, err)
		require.Equal(t, want, got)
	})
	t.Run("not a row scan", func(t *testing.T) {
		var scan pairScan = newSweepOrder(groupFixtures()[3].boxes, c.members)
		_, isRows := scan.(loftRowScan)
		require.False(t, isRows)
		sweepCount, err := sweepCandidateCountsWorkers(unlimitedBudget(t), scan, len(c.members), keepEveryPair,
			math.MaxUint64, 1)
		require.NoError(t, err)
		wantSweep, err := sweepCandidates(unlimitedBudget(t), scan, sweepCount.starts, sweepCount.count, keepEveryPair)
		require.NoError(t, err)
		got, err := sweepCandidatesByRows(unlimitedBudget(t), scan, c.members, sweepCount.starts, sweepCount.count,
			sweepCount.scanned)
		require.NoError(t, err)
		require.Equal(t, wantSweep, got)
	})
	t.Run("row scan", func(t *testing.T) {
		if runtime.GOMAXPROCS(0) < 2 {
			t.Skip("needs GOMAXPROCS >= 2")
		}
		got, err := sweepCandidatesByRows(unlimitedBudget(t), c.scan, c.members, sc.starts, sc.count, sc.scanned)
		require.NoError(t, err)
		require.Equal(t, want, got)
	})
}
