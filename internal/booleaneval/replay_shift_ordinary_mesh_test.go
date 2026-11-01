package booleaneval

import (
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
)

func TestOrdinaryShiftMeshCaptureLimitsAndShape(t *testing.T) {
	ctx := t.Context()
	fixture := func() *topo.Mesh { c, _, _, _ := shiftSnapshotFixture(); return c.selected }
	measured := proofbound.NewReplayTrialBudget()
	if _, err := captureOrdinaryShiftMesh(ctx, fixture(), measured); err != nil {
		t.Fatal(err)
	}
	work, records := measured.Usage()
	bytes := measured.Bytes()
	for _, short := range []string{shiftBudgetExactCase, shiftBudgetWorkCase, shiftBudgetRecordsCase, shiftBudgetBytesCase} {
		t.Run(short, func(t *testing.T) {
			b := proofbound.NewReplayTrialBudget()
			w, r, z := proofbound.ReplayTrialWork-work, proofbound.ReplayTrialRecords-records, proofbound.ReplayTrialBytes-bytes
			switch short {
			case shiftBudgetWorkCase:
				w++
			case shiftBudgetRecordsCase:
				r++
			case shiftBudgetBytesCase:
				z++
			}
			if err := b.SpendWork(ctx, w); err != nil {
				t.Fatal(err)
			}
			if err := b.ReserveRecords(ctx, r); err != nil {
				t.Fatal(err)
			}
			if err := b.ReserveBytes(ctx, z); err != nil {
				t.Fatal(err)
			}
			result, err := captureOrdinaryShiftMesh(ctx, fixture(), b)
			if short == shiftBudgetExactCase && (err != nil || result == nil) {
				t.Fatalf("exact: %v", err)
			}
			if short != shiftBudgetExactCase && (!errors.Is(err, decaderr.ErrUnsupported) || result != nil) {
				t.Fatalf("one-short %s: %v", short, err)
			}
		})
	}
	cases := map[string]func(*topo.Mesh){
		"source length":       func(m *topo.Mesh) { m.Source = nil },
		"source cap":          func(m *topo.Mesh) { m.Source = make([]*topo.Face, 65537) },
		"vertex bound cap":    func(m *topo.Mesh) { m.VertexBound = make([]float64, 65537) },
		"vertex bound length": func(m *topo.Mesh) { m.VertexBound = m.VertexBound[:1] },
		"missing face bound":  func(m *topo.Mesh) { m.FaceBound = map[*topo.Face]float64{} },
		"face bound length": func(m *topo.Mesh) {
			faces := make([]topo.Face, 65537)
			m.FaceBound = make(map[*topo.Face]float64, len(faces))
			for i := range faces {
				m.FaceBound[&faces[i]] = 0
			}
		},
		"nil source":   func(m *topo.Mesh) { m.Source[0] = nil },
		"replay table": func(m *topo.Mesh) { m.ReplayFaceBounds = &topo.FaceBoundTable{} },
	}
	for name, mutate := range cases {
		t.Run(name, func(t *testing.T) {
			m := fixture()
			mutate(m)
			result, err := captureOrdinaryShiftMesh(ctx, m, proofbound.NewReplayTrialBudget())
			if !errors.Is(err, decaderr.ErrUnsupported) || result != nil {
				t.Fatalf("malformed capture accepted: %v", err)
			}
		})
	}
}

func TestOrdinaryShiftCertificateCaptureSealsSource(t *testing.T) {
	c, _, target, tool := shiftSnapshotFixture()
	c.groups[0].Source = &provenance.SourcePatch{Producer: 7}
	c.groups[0].CapRecord = &topo.CapRecord{ZDenoted: big.NewRat(7, 3)}
	c.groups[0].Denoted = &surfacenormal.Revolved{Valid: true}
	c.groups[0].Denoted.Radius.Lo = big.NewRat(1, 3)
	capture := func(pc pieces) *ShiftCertificate {
		return captureShiftCertificate(t.Context(), target, tool, c.selected, c.decision, c.charged, pc, c.groups,
			map[int]pieceShift{0: c.shift}, 1, 1, 1)
	}
	stored := capture(c.component)
	if stored == nil {
		t.Fatal("valid capture was refused")
	}
	if err := validateShiftCaptureSeal(t.Context(), stored, proofbound.NewBrakeReplayBudget()); err != nil {
		t.Fatal(err)
	}
	c.groups[0].CapRecord.ZDenoted.SetInt64(9)
	c.groups[0].Denoted.Radius.Lo.SetInt64(8)
	if stored.groups[0].CapRecord.ZDenoted.Cmp(big.NewRat(7, 3)) != 0 || stored.groups[0].Denoted.Radius.Lo.Cmp(big.NewRat(1, 3)) != 0 {
		t.Fatal("capture retained mutable source metadata")
	}
	if err := validateShiftCaptureSeal(t.Context(), stored, proofbound.NewBrakeReplayBudget()); err != nil {
		t.Fatalf("detached metadata seal: %v", err)
	}
	c.groups[0].Source.Ordinal++
	if err := validateShiftCaptureSeal(t.Context(), stored, proofbound.NewBrakeReplayBudget()); !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("prior source mutation accepted: %v", err)
	}
	malformed := c.component
	malformed.ofVertex = malformed.ofVertex[:1]
	if capture(malformed) != nil {
		t.Fatal("malformed component accepted")
	}
}
