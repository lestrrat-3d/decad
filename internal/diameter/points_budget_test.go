package diameter

import (
	"context"
	"errors"
	"math"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// pointsWithStepBudget is PointsWithBudget charging each skipped pair with
// its own Step call, the reference the differential tests hold the StepN
// charge to. skipFail reports whether the step that failed was a skipped
// pair's.
func pointsWithStepBudget(budget *proofbound.WorkBudget, points []r3.Vec) (float64, bool, bool, error) {
	if len(points) == 0 {
		return 0, false, false, nil
	}
	best := 0.0
	bestI, bestJ := 0, 0
	skipSq := 0.0
	var blocks []pointBlock
	if len(points) >= 512 && parallelScanSafe(points) {
		blocks = pointBlocks(points)
	}
	for i := range points {
		for j := i + 1; j < len(points); {
			end := len(points)
			if blocks != nil {
				block := j / pointBlockSize
				end = min((block+1)*pointBlockSize, end)
				if skipSq > 0 && pointBlockDistanceUpperSq(points[i], blocks[block]) < skipSq {
					for ; j < end; j++ {
						if err := budget.Step(); err != nil {
							return 0, false, true, err
						}
					}
					continue
				}
			}
			for ; j < end; j++ {
				if err := budget.Step(); err != nil {
					return 0, false, false, err
				}
				delta := points[i].Sub(points[j])
				if skipSq > 0 && delta.X*delta.X+delta.Y*delta.Y+delta.Z*delta.Z < skipSq {
					continue
				}
				distance := delta.Len()
				if !usableMagnitude(distance) {
					return 0, false, false, nil
				}
				if distance > best {
					best, bestI, bestJ = distance, i, j
					if best > 1e-100 && best < 1e100 {
						limit := best * skipMargin
						skipSq = limit * limit
					} else {
						skipSq = 0
					}
				}
			}
		}
	}
	if err := budget.Err(); err != nil {
		return 0, false, false, err
	}
	if best == 0 {
		return 0, true, false, nil
	}
	d, ok := pairDistanceDown(points[bestI], points[bestJ])
	return d, ok, false, nil
}

// budgetRimPoints is a toothed rim on two caps: past the first rows almost
// every block is skipped, so failures land inside skipped blocks.
func budgetRimPoints(n int) []r3.Vec {
	points := make([]r3.Vec, 0, 2*n)
	for _, z := range []float64{0, 5} {
		for k := range n {
			theta := 2 * math.Pi * float64(k) / float64(n)
			r := 50.0
			if k/8%2 == 0 {
				r = 54
			}
			points = append(points, r3.NewVec(r*math.Cos(theta), r*math.Sin(theta), z))
		}
	}
	return points
}

// budgetCountingContext counts Err calls and fails from call failFrom on;
// failFrom 0 never fails.
type budgetCountingContext struct {
	calls    int
	failFrom int
}

func (c *budgetCountingContext) Deadline() (time.Time, bool) { return time.Time{}, false }
func (c *budgetCountingContext) Done() <-chan struct{}       { return nil }
func (c *budgetCountingContext) Value(any) any               { return nil }

func (c *budgetCountingContext) Err() error {
	c.calls++
	if c.failFrom != 0 && c.calls >= c.failFrom {
		return context.Canceled
	}
	return nil
}

type budgetReading struct {
	bits uint64
	ok   bool
	err  error
}

func newReading(d float64, ok bool, err error) budgetReading {
	return budgetReading{bits: math.Float64bits(d), ok: ok, err: err}
}

// TestPointsWithBudgetMatchesStepCharge holds PointsWithBudget on a poll
// budget to the per-pair Step reference on NewWorkBudget, with the context
// failing at several polls: the reading, the error, the context's Err calls
// and the budget's later steps all agree.
func TestPointsWithBudgetMatchesStepCharge(t *testing.T) {
	t.Parallel()
	points := budgetRimPoints(600)
	pairs := len(points) * (len(points) - 1) / 2
	polls := pairs / proofbound.WorkPollInterval
	var sawSkipFail bool
	for _, failFrom := range []int{1, 2, 7, polls / 3, polls / 2, polls - 1, polls, polls + 1, polls + 2, 0} {
		want := &budgetCountingContext{failFrom: failFrom}
		got := &budgetCountingContext{failFrom: failFrom}
		wantB, gotB := proofbound.NewWorkBudget(want), proofbound.NewPollWorkBudget(got)
		d, ok, skipFail, err := pointsWithStepBudget(wantB, points)
		wantR := newReading(d, ok, err)
		sawSkipFail = sawSkipFail || skipFail
		d, ok, err = PointsWithBudget(gotB, points)
		require.Equal(t, wantR, newReading(d, ok, err), "fail from poll %d", failFrom)
		require.Equal(t, want.calls, got.calls, "fail from poll %d", failFrom)
		for range 2 * proofbound.WorkPollInterval {
			require.Equal(t, wantB.Step(), gotB.Step(), "fail from poll %d", failFrom)
		}
		require.Equal(t, want.calls, got.calls, "fail from poll %d", failFrom)
		if failFrom == 0 {
			require.NoError(t, wantR.err)
			require.True(t, wantR.ok)
		}
	}
	require.True(t, sawSkipFail, "premise: some cancel lands inside a skipped block")
}

// TestPointsWithBudgetMatchesStepChargeAtLimit stops a counting budget at
// many steps, inside and outside skipped blocks, and on a wall budget's
// limit: the error, the step it arrives at and the budget's state after
// agree with the per-pair Step reference.
func TestPointsWithBudgetMatchesStepChargeAtLimit(t *testing.T) {
	t.Parallel()
	points := budgetRimPoints(600)
	pairs := len(points) * (len(points) - 1) / 2
	stop := errors.New("stop")
	counting := func(stopAt int) (*proofbound.WorkBudget, *int) {
		count := 0
		return &proofbound.WorkBudget{
			StepFn: func() error {
				count++
				if count == stopAt {
					return stop
				}
				return nil
			},
			ErrFn: func() error { return nil },
		}, &count
	}
	stops := []int{1, 2, 63, 64, 65, 600, 1199, 1200, 1201, 1263}
	for k := 1; k < 40; k++ {
		stops = append(stops, k*pairs/40+k)
	}
	stops = append(stops, pairs-1, pairs, pairs+1)
	var sawSkipFail, sawPairFail bool
	for _, stopAt := range stops {
		wantB, wantCount := counting(stopAt)
		gotB, gotCount := counting(stopAt)
		d, ok, skipFail, err := pointsWithStepBudget(wantB, points)
		wantR := newReading(d, ok, err)
		if errors.Is(err, stop) {
			sawSkipFail = sawSkipFail || skipFail
			sawPairFail = sawPairFail || !skipFail
		}
		d, ok, err = PointsWithBudget(gotB, points)
		require.Equal(t, wantR, newReading(d, ok, err), "stop at %d", stopAt)
		require.Equal(t, *wantCount, *gotCount, "stop at %d", stopAt)
		if stopAt <= pairs {
			require.ErrorIs(t, wantR.err, stop)
			require.Equal(t, stopAt, *gotCount)
		}

		wall := func() *proofbound.WorkBudget { return proofbound.NewWallWorkBudget(uint64(stopAt)) }
		wantW, gotW := wall(), wall()
		d, ok, _, err = pointsWithStepBudget(wantW, points)
		wantR = newReading(d, ok, err)
		d, ok, err = PointsWithBudget(gotW, points)
		require.Equal(t, wantR, newReading(d, ok, err), "wall limit %d", stopAt)
		require.Equal(t, wantW.Err(), gotW.Err(), "wall limit %d", stopAt)
		require.Equal(t, wantW.Step(), gotW.Step(), "wall limit %d", stopAt)
	}
	require.True(t, sawSkipFail, "premise: some stop lands inside a skipped block")
	require.True(t, sawPairFail, "premise: some stop lands on a computed pair")
}

// BenchmarkPointsWithBudgetGearRim reads a gear-sized witness set, where
// almost every block is skipped, through each budget a caller can pass.
func BenchmarkPointsWithBudgetGearRim(b *testing.B) {
	points := budgetRimPoints(4160)
	for _, c := range []struct {
		name   string
		budget func(context.Context) *proofbound.WorkBudget
	}{
		{"work", proofbound.NewWorkBudget},
		{"poll", proofbound.NewPollWorkBudget},
	} {
		b.Run(c.name, func(b *testing.B) {
			ctx := b.Context()
			b.ReportAllocs()
			for b.Loop() {
				if _, ok, err := PointsWithBudget(c.budget(ctx), points); err != nil || !ok {
					b.Fatal("no diameter")
				}
			}
		})
	}
}
