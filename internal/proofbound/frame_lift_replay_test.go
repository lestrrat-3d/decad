package proofbound

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

func TestExactFrameLiftRoundReplay(t *testing.T) {
	axis, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	if err != nil {
		t.Fatal(err)
	}
	far, err := r3.NewFrame(r3.Vec{X: 1e12 + .1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	if err != nil {
		t.Fatal(err)
	}
	shifted, err := r3.FromBasis(r3.Basis{EX: r3.Vec{Y: -1}, EY: r3.Vec{X: 1}, EZ: r3.Vec{Z: 1}},
		r3.Vec{X: 10.1, Y: -7.25, Z: 525})
	if err != nil {
		t.Fatal(err)
	}
	for _, tc := range []struct {
		name    string
		frame   r3.Frame
		xform   r3.Transform
		u, v, z float64
	}{
		{"exact", axis, r3.Identity(), 1, 2, 3},
		{"placed", axis, shifted, .1, -.25, 7},
		{"far cancellation", far, shifted, -1e12, .1, 2},
		{"subnormal", axis, r3.Identity(), math.SmallestNonzeroFloat64, 0, 0},
		{"near maximum", axis, r3.Identity(), math.MaxFloat64 / 4, 1, 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			meter := NewReplayTrialBudget()
			point, bound, err := ExactFrameLiftRoundReplay(t.Context(), tc.frame, tc.xform, tc.u, tc.v, tc.z, meter)
			if err != nil {
				t.Fatal(err)
			}
			local := tc.frame.ToWorldUV(tc.u, tc.v)
			want := tc.xform.Apply(local.Add(tc.frame.N().Scale(tc.z)))
			wantBound := ExactFrameLiftRound(tc.frame, tc.xform, tc.u, tc.v, tc.z, want)
			gotBits := [4]uint64{math.Float64bits(point.X), math.Float64bits(point.Y), math.Float64bits(point.Z), math.Float64bits(bound)}
			wantBits := [4]uint64{math.Float64bits(want.X), math.Float64bits(want.Y), math.Float64bits(want.Z), math.Float64bits(wantBound)}
			if gotBits != wantBits {
				t.Fatalf("point and bound bits=%x want=%x", gotBits, wantBits)
			}
			work, records := meter.Usage()
			bytes := meter.Bytes()
			for _, limit := range []struct {
				name    string
				w, r, b uint64
			}{
				{"work short", work - 1, records, bytes},
				{"records short", work, records - 1, bytes},
				{"bytes short", work, records, bytes - 1},
			} {
				t.Run(limit.name, func(t *testing.T) {
					short, err := NewReplayBudgetLimits(limit.w, limit.r, limit.b)
					if err != nil {
						t.Fatal(err)
					}
					_, _, err = ExactFrameLiftRoundReplay(t.Context(), tc.frame, tc.xform, tc.u, tc.v, tc.z, short)
					if !errors.Is(err, decaderr.ErrUnsupported) {
						t.Fatalf("one-short error=%v", err)
					}
					w, r := short.Usage()
					if w != 0 || r != 0 || short.Bytes() != 0 {
						t.Fatalf("one-short counters=%d,%d,%d", w, r, short.Bytes())
					}
				})
			}
		})
	}
	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	meter := NewReplayTrialBudget()
	if _, _, err := ExactFrameLiftRoundReplay(cancelled, axis, r3.Identity(), 1, 2, 3, meter); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancelled error=%v", err)
	}
	w, r := meter.Usage()
	if w != 0 || r != 0 || meter.Bytes() != 0 {
		t.Fatalf("cancelled counters=%d,%d,%d", w, r, meter.Bytes())
	}
	if _, _, err := ExactFrameLiftRoundReplay(t.Context(), axis, r3.Identity(), math.Inf(1), 0, 0, NewReplayTrialBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("nonfinite input error=%v", err)
	}
}
