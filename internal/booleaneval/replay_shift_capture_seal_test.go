package booleaneval

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
)

const shiftBudgetBytesCase = "bytes"

func captureSealFixture(t *testing.T) *ShiftCertificate {
	t.Helper()
	c, _, _, _ := shiftSnapshotFixture() //nolint:dogsled // Only the stored certificate is needed.
	c.groups[0].Source = &provenance.SourcePatch{Producer: 2, Parent: &provenance.SourcePatch{Producer: 1}}
	c.groups[0].CapRecord = &topo.CapRecord{ZDenoted: big.NewRat(7, 3)}
	c.groups[0].Denoted = &surfacenormal.Revolved{Valid: true}
	c.groups[0].Denoted.Radius.Lo = big.NewRat(1, 3)
	c.groups[0].Denoted.Radius.Hi = big.NewRat(2, 3)
	resealShiftFixture(t, c)
	return c
}

func TestShiftCaptureSealRejectsPriorMutation(t *testing.T) {
	cases := map[string]func(*ShiftCertificate){
		"source":             func(c *ShiftCertificate) { c.groups[0].Source.Parent.Ordinal++ },
		"origin":             func(c *ShiftCertificate) { c.groups[0].Origins[0].Role = "changed" },
		"group scalar":       func(c *ShiftCertificate) { c.groups[0].NormalBound = 1 },
		"cap level pointer":  func(c *ShiftCertificate) { c.groups[0].CapRecord.ZDenoted = big.NewRat(7, 3) },
		"cap":                func(c *ShiftCertificate) { c.groups[0].CapRecord.ZDenoted.SetInt64(8) },
		"denotation":         func(c *ShiftCertificate) { c.groups[0].Denoted.Radius.Lo.SetInt64(2) },
		"cap pointer":        func(c *ShiftCertificate) { v := *c.groups[0].CapRecord; c.groups[0].CapRecord = &v },
		"denotation pointer": func(c *ShiftCertificate) { v := *c.groups[0].Denoted; c.groups[0].Denoted = &v },
	}
	for name, mutate := range cases {
		t.Run(name, func(t *testing.T) {
			c := captureSealFixture(t)
			mutate(c)
			_, err := SnapshotShiftCertificate(t.Context(), proofbound.NewBrakeReplayBudget(), c)
			if !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("changed capture accepted: %v", err)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err := SnapshotShiftCertificate(ctx, proofbound.NewBrakeReplayBudget(), nil); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancelled nil: %v", err)
	}
}

func TestShiftCaptureSealBudgetLimits(t *testing.T) {
	c := captureSealFixture(t)
	measure := proofbound.NewReplayTrialBudget()
	if _, err := captureShiftGroupSeal(t.Context(), c.groups, measure); err != nil {
		t.Fatal(err)
	}
	work, records := measure.Usage()
	bytes := measure.Bytes()
	for _, short := range []string{shiftBudgetExactCase, shiftBudgetWorkCase, shiftBudgetRecordsCase, shiftBudgetBytesCase} {
		t.Run(short, func(t *testing.T) {
			meter := proofbound.NewReplayTrialBudget()
			w, r, z := proofbound.ReplayTrialWork-work, proofbound.ReplayTrialRecords-records, proofbound.ReplayTrialBytes-bytes
			switch short {
			case shiftBudgetWorkCase:
				w++
			case shiftBudgetRecordsCase:
				r++
			case shiftBudgetBytesCase:
				z++
			}
			if err := meter.SpendWork(t.Context(), w); err != nil {
				t.Fatal(err)
			}
			if err := meter.ReserveRecords(t.Context(), r); err != nil {
				t.Fatal(err)
			}
			if err := meter.ReserveBytes(t.Context(), z); err != nil {
				t.Fatal(err)
			}
			seal, err := captureShiftGroupSeal(t.Context(), c.groups, meter)
			if short == shiftBudgetExactCase && (err != nil || seal == nil) {
				t.Fatalf("exact: %v", err)
			}
			if short != shiftBudgetExactCase && (!errors.Is(err, decaderr.ErrUnsupported) || seal != nil) {
				t.Fatalf("one-short %s: %v", short, err)
			}
		})
	}
}

func TestShiftCaptureSealTamperRefusalAccounting(t *testing.T) {
	ctx := t.Context()
	c, _, _, _ := shiftSnapshotFixture() //nolint:dogsled // Only the stored certificate is needed.
	c.groups[0].Source = &provenance.SourcePatch{Producer: 2, Ordinal: 1}
	resealShiftFixture(t, c)
	c.groups[0].Source.Ordinal++
	meter := proofbound.NewBrakeReplayBudget()
	if err := validateShiftCaptureSeal(ctx, c, meter); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("tamper: %v", err)
	}
	cost, _ := meter.Usage()
	if cost == 0 {
		t.Fatal("tamper validation did not charge work")
	}
	if n := testing.AllocsPerRun(100, func() {
		if !errors.Is(validateShiftCaptureSeal(ctx, c, meter), decaderr.ErrUnsupported) {
			panic("tamper refusal")
		}
	}); n != 0 {
		t.Fatalf("tamper refusal allocated %.0f objects", n)
	}
	for _, short := range []bool{false, true} {
		budget := proofbound.NewBrakeReplayBudget()
		left := cost
		if short {
			left--
		}
		if err := budget.SpendWork(ctx, proofbound.BrakeReplayWork-left); err != nil {
			t.Fatal(err)
		}
		if err := budget.ReserveRecords(ctx, proofbound.BrakeReplayRecords); err != nil {
			t.Fatal(err)
		}
		if err := budget.ReserveBytes(ctx, proofbound.BrakeReplayBytes); err != nil {
			t.Fatal(err)
		}
		err := validateShiftCaptureSeal(ctx, c, budget)
		_, limited := errors.AsType[*proofbound.WorkLimitError](err)
		if short != limited || !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("short=%t: %v", short, err)
		}
	}
}
