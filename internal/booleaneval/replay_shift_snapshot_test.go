package booleaneval

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func shiftSnapshotFixture() (*ShiftCertificate, *topo.Body, *topo.Body, *topo.Body) {
	a := &topo.Body{Origin: provenance.Ref{Producer: 1}}
	b := &topo.Body{Origin: provenance.Ref{Producer: 2}}
	r := &topo.Body{Origin: provenance.Ref{Producer: 3}}
	face := &topo.Face{}
	m := &topo.Mesh{Vertices: []r3.Vec{{}, {X: 1}, {Y: 1}}, Triangles: [][3]int{{0, 1, 2}}, Source: []*topo.Face{face}, VertexBound: []float64{0, 0, 0}, FaceBound: map[*topo.Face]float64{face: 0}, BoundaryOK: true, SymDiffOK: true}
	c := &ShiftCertificate{target: a, tool: b, targetID: 1, toolID: 2, resultID: 3, sealed: true, selected: m, decision: m, charged: m, targetMesh: m, component: pieces{count: 1, ofVertex: []int{0, 0, 0}, ofFacet: []int{0}}, shift: pieceShift{Move: r3.Vec{X: 1}}, groups: []facetedrecord.Group{{Origins: []provenance.Ref{{Producer: 2, Role: "side"}}}}}
	r.PrivateShift = c
	var err error
	c.captureSeal, err = captureShiftGroupSeal(context.Background(), c.groups, proofbound.NewReplayTrialBudget())
	if err != nil {
		panic(err)
	}
	return c, r, a, b
}

func TestShiftAuthTamperRefusalsDoNotAllocate(t *testing.T) {
	ctx := t.Context()
	c, result, target, tool := shiftSnapshotFixture()
	budget := proofbound.NewBrakeReplayBudget()
	if err := budget.SpendWork(ctx, proofbound.BrakeReplayWork); err != nil {
		t.Fatal(err)
	}
	if err := budget.ReserveRecords(ctx, proofbound.BrakeReplayRecords); err != nil {
		t.Fatal(err)
	}
	if err := budget.ReserveBytes(ctx, proofbound.BrakeReplayBytes); err != nil {
		t.Fatal(err)
	}
	checks := []struct {
		name string
		run  func() error
	}{
		{"growth", func() error {
			_, err := authenticateStoredGrowthSnapshotCopy(ctx, c, result, target, tool, opdiagnostic.Record{}, budget)
			return err
		}},
		{"warning", func() error {
			_, err := c.AuthenticatePrivateShift(ctx, target, tool, meshbool.OpCut, []opdiagnostic.Record{{}}, false, budget)
			return err
		}},
		{"enclosure", func() error { _, err := replayBoxes(ctx, nil, nil, r3.Vec{}, budget); return err }},
	}
	for _, check := range checks {
		t.Run(check.name, func(t *testing.T) {
			if err := check.run(); !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("tamper refusal: %v", err)
			}
			if n := testing.AllocsPerRun(100, func() {
				if !errors.Is(check.run(), decaderr.ErrUnsupported) {
					panic("tamper refusal")
				}
			}); n != 0 {
				t.Fatalf("tamper refusal allocated %.0f objects", n)
			}
		})
	}
}
func resealShiftFixture(t *testing.T, c *ShiftCertificate) {
	t.Helper()
	var err error
	c.captureSeal, err = captureShiftGroupSeal(t.Context(), c.groups, proofbound.NewReplayTrialBudget())
	if err != nil {
		t.Fatal(err)
	}
}

const shiftBudgetWorkCase = "work"
const shiftBudgetExactCase = "exact"
const shiftBudgetRecordsCase = "records"

func TestSnapshotStoredShift(t *testing.T) {
	c, r, a, b := shiftSnapshotFixture()
	budget := proofbound.NewBrakeReplayBudget()
	got, err := snapshotStoredShift(t.Context(), c, r, a, b, budget)
	if err != nil {
		t.Fatal(err)
	}
	w, n := budget.Usage()
	z := budget.Bytes()
	t.Logf("work=%d records=%d bytes=%d", w, n, z)
	if got.selected == c.selected || got.selected.Source[0] != c.selected.Source[0] {
		t.Fatal("mesh identity")
	}
	c.selected.Vertices[0].X = 7
	c.groups[0].Origins[0].Role = "changed"
	if got.selected.Vertices[0].X != 0 || got.groups[0].Origins[0].Role != "side" {
		t.Fatal("snapshot aliased")
	}
	for _, short := range []string{"", shiftBudgetWorkCase, shiftBudgetRecordsCase, shiftBudgetBytesCase} {
		c, r, a, b = shiftSnapshotFixture()
		m := proofbound.NewBrakeReplayBudget()
		w0, n0, z0 := proofbound.BrakeReplayWork-w, proofbound.BrakeReplayRecords-n, proofbound.BrakeReplayBytes-z
		switch short {
		case shiftBudgetWorkCase:
			w0++
		case shiftBudgetRecordsCase:
			n0++
		case shiftBudgetBytesCase:
			z0++
		}
		if err := m.SpendWork(t.Context(), w0); err != nil {
			t.Fatal(err)
		}
		if err := m.ReserveRecords(t.Context(), n0); err != nil {
			t.Fatal(err)
		}
		if err := m.ReserveBytes(t.Context(), z0); err != nil {
			t.Fatal(err)
		}
		_, err := snapshotStoredShift(t.Context(), c, r, a, b, m)
		if short == "" && err != nil || short != "" && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", short, err)
		}
	}
	c, r, a, b = shiftSnapshotFixture()
	r.PrivateShift = &ShiftCertificate{}
	if _, err := snapshotStoredShift(t.Context(), c, r, a, b, proofbound.NewBrakeReplayBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("stale: %v", err)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, err := snapshotStoredShift(ctx, c, r, a, b, proofbound.NewBrakeReplayBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel: %v", err)
	}
}

func TestShiftSnapshotRevalidate(t *testing.T) {
	c, r, a, b := shiftSnapshotFixture()
	c.groups[0].Source = &provenance.SourcePatch{Producer: 2, Ordinal: 1}
	resealShiftFixture(t, c)
	budget := proofbound.NewBrakeReplayBudget()
	snap, err := SnapshotShiftCertificate(t.Context(), budget, c)
	if err != nil {
		t.Fatal(err)
	}
	if err := snap.Revalidate(t.Context(), budget, r, a, b); err != nil {
		t.Fatal(err)
	}
	originalMesh := c.selected
	replacementMesh := *originalMesh
	c.selected = &replacementMesh
	if err := snap.Revalidate(t.Context(), budget, r, a, b); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("mesh pointer replacement: %v", err)
	}
	c.selected = originalMesh
	c.selected.Vertices[0].X = 2
	if err := snap.Revalidate(t.Context(), budget, r, a, b); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("mesh mutation: %v", err)
	}
	c.selected.Vertices[0].X = 0
	c.groups[0].Source.Ordinal++
	if err := snap.Revalidate(t.Context(), budget, r, a, b); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("source mutation: %v", err)
	}
	if err := snap.Revalidate(t.Context(), proofbound.NewBrakeReplayBudget(), r, a, b); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("different budget: %v", err)
	}
}

func TestShiftSnapshotNestedValues(t *testing.T) {
	c, r, a, b := shiftSnapshotFixture()
	c.groups[0].CapRecord = &topo.CapRecord{ZDenoted: big.NewRat(7, 3)}
	c.groups[0].Denoted = &surfacenormal.Revolved{Valid: true}
	c.groups[0].Denoted.Radius.Lo = big.NewRat(1, 3)
	c.groups[0].Denoted.Radius.Hi = big.NewRat(2, 3)
	resealShiftFixture(t, c)
	budget := proofbound.NewBrakeReplayBudget()
	snap, err := SnapshotShiftCertificate(t.Context(), budget, c)
	if err != nil {
		t.Fatal(err)
	}
	if err = snap.Revalidate(t.Context(), budget, r, a, b); err != nil {
		t.Fatal(err)
	}
	original := c.groups[0].CapRecord
	replacement := *original
	c.groups[0].CapRecord = &replacement
	if err = snap.Revalidate(t.Context(), budget, r, a, b); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("cap pointer replacement: %v", err)
	}
	c.groups[0].CapRecord = original
	c.groups[0].CapRecord.ZDenoted.SetInt64(9)
	if err = snap.Revalidate(t.Context(), budget, r, a, b); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("cap mutation: %v", err)
	}
	c.groups[0].CapRecord.ZDenoted.SetFrac64(7, 3)
	c.groups[0].Denoted.Radius.Lo.SetInt64(4)
	if err = snap.Revalidate(t.Context(), budget, r, a, b); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("denotation mutation: %v", err)
	}
}

func TestShiftSnapshotLimits(t *testing.T) {
	run := func(m *proofbound.ReplayBudget) error {
		c, r, a, b := shiftSnapshotFixture()
		c.groups[0].Source = &provenance.SourcePatch{Producer: 2, Parent: &provenance.SourcePatch{Producer: 1}}
		c.groups[0].CapRecord = &topo.CapRecord{ZDenoted: big.NewRat(7, 3)}
		c.groups[0].Denoted = &surfacenormal.Revolved{Valid: true}
		c.groups[0].Denoted.Radius.Lo = big.NewRat(1, 3)
		c.groups[0].Denoted.Radius.Hi = big.NewRat(2, 3)
		resealShiftFixture(t, c)
		s, err := SnapshotShiftCertificate(t.Context(), m, c)
		if err != nil {
			return err
		}
		return s.Revalidate(t.Context(), m, r, a, b)
	}
	measured := proofbound.NewBrakeReplayBudget()
	if err := run(measured); err != nil {
		t.Fatal(err)
	}
	work, records := measured.Usage()
	bytes := measured.Bytes()
	t.Logf("work=%d records=%d bytes=%d", work, records, bytes)
	for _, short := range []string{"", shiftBudgetWorkCase, shiftBudgetRecordsCase, shiftBudgetBytesCase} {
		m := proofbound.NewBrakeReplayBudget()
		w, n, z := proofbound.BrakeReplayWork-work, proofbound.BrakeReplayRecords-records, proofbound.BrakeReplayBytes-bytes
		switch short {
		case shiftBudgetWorkCase:
			w++
		case shiftBudgetRecordsCase:
			n++
		case shiftBudgetBytesCase:
			z++
		}
		if err := m.SpendWork(t.Context(), w); err != nil {
			t.Fatal(err)
		}
		if err := m.ReserveRecords(t.Context(), n); err != nil {
			t.Fatal(err)
		}
		if err := m.ReserveBytes(t.Context(), z); err != nil {
			t.Fatal(err)
		}
		err := run(m)
		if short == "" && err != nil || short != "" && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", short, err)
		}
	}
}

func TestShiftSnapshotReplayFaceBounds(t *testing.T) {
	c, r, a, b := shiftSnapshotFixture()
	m := c.selected
	face := m.Source[0]
	setup := proofbound.NewBrakeReplayBudget()
	table, err := topo.NewFaceBoundTable(t.Context(), 1, setup)
	if err != nil {
		t.Fatal(err)
	}
	if err = table.Set(t.Context(), face, 0, setup); err != nil {
		t.Fatal(err)
	}
	m.FaceBound = nil
	m.ReplayFaceBounds = table
	budget := proofbound.NewBrakeReplayBudget()
	s, err := SnapshotShiftCertificate(t.Context(), budget, c)
	if err != nil {
		t.Fatal(err)
	}
	if s.copy.selected.ReplayFaceBounds == table {
		t.Fatal("face table aliased")
	}
	if err = s.Revalidate(t.Context(), budget, r, a, b); err != nil {
		t.Fatal(err)
	}
	if err = table.Set(t.Context(), face, 1, setup); err != nil {
		t.Fatal(err)
	}
	if err = s.Revalidate(t.Context(), budget, r, a, b); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("face-bound mutation: %v", err)
	}
}

func TestShiftSnapshotTamperRefusalAccounting(t *testing.T) {
	ctx := t.Context()
	makeSnapshot := func() (*ShiftCertificate, *ShiftCertificateSnapshot, *proofbound.ReplayBudget, *topo.Body, *topo.Body, *topo.Body) {
		c, result, target, tool := shiftSnapshotFixture() //nolint:contextcheck // The fixture creates immutable local inputs.
		c.groups[0].Source = &provenance.SourcePatch{Producer: 2, Ordinal: 1}
		resealShiftFixture(t, c) //nolint:contextcheck // The seal uses the test context.
		budget := proofbound.NewBrakeReplayBudget()
		snap, err := SnapshotShiftCertificate(ctx, budget, c)
		if err != nil {
			t.Fatal(err)
		}
		c.groups[0].Source.Ordinal++
		return c, snap, budget, result, target, tool
	}
	_, snap, budget, result, target, tool := makeSnapshot()
	before, _ := budget.Usage()
	if err := snap.Revalidate(ctx, budget, result, target, tool); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("tamper: %v", err)
	}
	after, _ := budget.Usage()
	cost := after - before
	if cost == 0 {
		t.Fatal("tamper validation did not charge work")
	}
	if n := testing.AllocsPerRun(100, func() {
		if !errors.Is(snap.Revalidate(ctx, budget, result, target, tool), decaderr.ErrUnsupported) {
			panic("tamper refusal")
		}
	}); n != 0 {
		t.Fatalf("tamper refusal allocated %.0f objects", n)
	}
	for _, name := range []string{shiftBudgetExactCase, shiftBudgetWorkCase, shiftBudgetRecordsCase, shiftBudgetBytesCase} {
		t.Run(name, func(t *testing.T) {
			_, snap, budget, result, target, tool := makeSnapshot()
			work, records := budget.Usage()
			bytes := budget.Bytes()
			left := cost
			if name == shiftBudgetWorkCase {
				left--
			}
			if err := budget.SpendWork(ctx, proofbound.BrakeReplayWork-work-left); err != nil {
				t.Fatal(err)
			}
			if err := budget.ReserveRecords(ctx, proofbound.BrakeReplayRecords-records); err != nil {
				t.Fatal(err)
			}
			if err := budget.ReserveBytes(ctx, proofbound.BrakeReplayBytes-bytes); err != nil {
				t.Fatal(err)
			}
			err := snap.Revalidate(ctx, budget, result, target, tool)
			var limit *proofbound.WorkLimitError
			if name == shiftBudgetWorkCase {
				if !errors.As(err, &limit) {
					t.Fatalf("work one-short: %v", err)
				}
			} else if !errors.Is(err, decaderr.ErrUnsupported) || errors.As(err, &limit) {
				t.Fatalf("tamper %s: %v", name, err)
			}
		})
	}
}
