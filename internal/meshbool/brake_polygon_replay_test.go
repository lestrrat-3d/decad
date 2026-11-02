package meshbool

import (
	"context"
	"errors"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func brakePolygonPoints(coords [][2]int64) []Xp2 {
	out := make([]Xp2, len(coords))
	for i, p := range coords {
		out[i] = NewXP2(big.NewRat(p[0], 1), big.NewRat(p[1], 1))
	}
	return out
}

func TestBrakeReplayPolygonParity(t *testing.T) {
	ctx := t.Context()
	for _, coords := range [][][2]int64{
		{{0, 0}, {4, 0}, {4, 4}, {0, 4}},
		{{0, 0}, {4, 0}, {4, 4}, {2, 2}, {0, 4}},
		{{0, 0}, {2, 0}, {4, 0}, {4, 4}, {0, 4}},
	} {
		pts := brakePolygonPoints(coords)
		poly := make([]int, len(pts))
		for i := range poly {
			poly[i] = i
		}
		want, err := EarClipX(proofbound.NewWorkBudget(ctx), pts, poly)
		if err != nil {
			t.Fatalf("ordinary ears: %v", err)
		}
		meter := proofbound.NewBrakeReplayBudget()
		got, err := replayEarClip(ctx, meter.WorkBudget(ctx), pts, poly)
		if err != nil || !slices.Equal(got, want) {
			t.Fatalf("replay ear triangles differ: %v", err)
		}
		for _, probe := range brakePolygonPoints([][2]int64{{1, 1}, {0, 2}, {6, 3}, {2, 2}}) {
			inside, boundary, err := PointInPoly2(proofbound.NewWorkBudget(ctx), pts, probe)
			if err != nil {
				t.Fatal(err)
			}
			meter := proofbound.NewBrakeReplayBudget()
			gotInside, gotBoundary, err := replayPointInPoly2(ctx, meter.WorkBudget(ctx), pts, probe)
			if err != nil || gotInside != inside || gotBoundary != boundary {
				t.Fatalf("containment differs: %v", err)
			}
		}
		wantArea, err := PolyArea2Sign(proofbound.NewWorkBudget(ctx), pts)
		if err != nil {
			t.Fatal(err)
		}
		meter = proofbound.NewBrakeReplayBudget()
		gotArea, err := replayPolyArea2Sign(ctx, meter.WorkBudget(ctx), pts)
		if err != nil || gotArea != wantArea {
			t.Fatalf("area sign differs: %v", err)
		}
	}
}

func TestBrakeReplayPolygonBudgetLimits(t *testing.T) {
	ctx := t.Context()
	pts := brakePolygonPoints([][2]int64{{0, 0}, {4, 0}, {4, 4}, {0, 4}})
	for i := range pts {
		pts[i].FloatFinite = false
	}
	poly := []int{0, 1, 2, 3}
	probe := brakePolygonPoints([][2]int64{{1, 1}})[0]
	probe.FloatFinite = false
	cases := []struct {
		name string
		run  func(context.Context, *proofbound.WorkBudget) error
	}{
		{"ears", func(ctx context.Context, b *proofbound.WorkBudget) error {
			_, err := replayEarClip(ctx, b, pts, poly)
			return err
		}},
		{"area", func(ctx context.Context, b *proofbound.WorkBudget) error {
			_, err := replayPolyArea2Sign(ctx, b, pts)
			return err
		}},
		{"containment", func(ctx context.Context, b *proofbound.WorkBudget) error {
			_, _, err := replayPointInPoly2(ctx, b, pts, probe)
			return err
		}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			measure := proofbound.NewBrakeReplayBudget()
			if err := c.run(ctx, measure.WorkBudget(ctx)); err != nil {
				t.Fatal(err)
			}
			work, records := measure.Usage()
			bytes := measure.Bytes()
			if work == 0 || records == 0 || bytes == 0 {
				t.Fatal("exact predicate did not charge all limits")
			}
			for _, short := range []string{brakeTestExact, brakeTestWork, brakeTestRecords, brakeTestBytes} {
				t.Run(short, func(t *testing.T) {
					meter := proofbound.NewBrakeReplayBudget()
					w, r, b := proofbound.BrakeReplayWork-work, proofbound.BrakeReplayRecords-records, proofbound.BrakeReplayBytes-bytes
					switch short {
					case brakeTestWork:
						w++
					case brakeTestRecords:
						r++
					case brakeTestBytes:
						b++
					}
					if err := meter.SpendWork(ctx, w); err != nil {
						t.Fatal(err)
					}
					if err := meter.ReserveRecords(ctx, r); err != nil {
						t.Fatal(err)
					}
					if err := meter.ReserveBytes(ctx, b); err != nil {
						t.Fatal(err)
					}
					err := c.run(ctx, meter.WorkBudget(ctx))
					if short == brakeTestExact && err != nil {
						t.Fatalf("exact: %v", err)
					}
					if short != brakeTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
						t.Fatalf("one-short: %v", err)
					}
				})
			}
			cancelled, cancel := context.WithCancel(ctx)
			cancel()
			meter := proofbound.NewBrakeReplayBudget()
			if err := c.run(cancelled, meter.WorkBudget(cancelled)); !errors.Is(err, context.Canceled) {
				t.Fatalf("cancellation: %v", err)
			}
		})
	}
}
