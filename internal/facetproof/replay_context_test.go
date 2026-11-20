package facetproof_test

import (
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestReplayFacetProofContextParityAndLimits(t *testing.T) {
	const exact = "exact"
	ctx := t.Context()
	t.Run("analytic tetrahedron volume", func(t *testing.T) {
		vertices := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
		facets := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
		audit, err := facetproof.AuditFacetedMesh(t.Context(), vertices, facets)
		if err != nil {
			t.Fatal(err)
		}
		meter := proofbound.NewReplayTrialBudget()
		volume, bound, exact, ok, err := facetproof.VolumeFromAuditReplay(t.Context(), meter.WorkBudget(t.Context()), audit, 0)
		if err != nil || !ok || exact == nil || exact.Cmp(big.NewRat(1, 6)) != 0 || math.Abs(volume-1.0/6.0) > bound {
			t.Fatalf("tetrahedron volume: value=%.17g bound=%.17g valid=%t error=%v", volume, bound, ok, err)
		}
	})
	verts, tris := tetraMesh()
	audit, err := facetproof.AuditFacetedMesh(ctx, verts, tris)
	if err != nil {
		t.Fatal(err)
	}
	v, b, rat, ok, err := facetproof.VolumeFromAuditBounded(proofbound.NewWorkBudget(ctx), audit, 0)
	if err != nil || !ok {
		t.Fatalf("ordinary volume: %v", err)
	}
	run := func(m *proofbound.ReplayBudget) (float64, float64, *big.Rat, bool, error) {
		return facetproof.VolumeFromAuditReplay(ctx, m.WorkBudget(ctx), audit, 0)
	}
	m := proofbound.NewReplayTrialBudget()
	rv, rb, rr, ok, err := run(m)
	if err != nil || !ok || math.Float64bits(rv) != math.Float64bits(v) || math.Float64bits(rb) != math.Float64bits(b) || rr.Cmp(rat) != 0 {
		t.Fatalf("volume parity: %v", err)
	}
	work, records := m.Usage()
	bytes := m.Bytes()
	for _, short := range []string{exact, "work", "records", "bytes"} {
		t.Run(short, func(t *testing.T) {
			m := proofbound.NewReplayTrialBudget()
			w, r, z := proofbound.ReplayTrialWork-work, proofbound.ReplayTrialRecords-records, proofbound.ReplayTrialBytes-bytes
			switch short {
			case "work":
				w++
			case "records":
				r++
			case "bytes":
				z++
			}
			if err := m.SpendWork(ctx, w); err != nil {
				t.Fatal(err)
			}
			if err := m.ReserveRecords(ctx, r); err != nil {
				t.Fatal(err)
			}
			if err := m.ReserveBytes(ctx, z); err != nil {
				t.Fatal(err)
			}
			_, _, _, _, err := run(m)
			if short == exact && err != nil {
				t.Fatalf("exact: %v", err)
			}
			if short != exact && !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("one-short %s: %v", short, err)
			}
		})
	}
	ordinary, err := facetproof.FirstMomentsFromAudit(proofbound.NewWorkBudget(ctx), audit, tris)
	if err != nil {
		t.Fatal(err)
	}
	replay, err := facetproof.FirstMomentsReplay(ctx, proofbound.NewReplayTrialBudget().WorkBudget(ctx), audit, tris)
	if err != nil {
		t.Fatal(err)
	}
	for i := range ordinary {
		if replay[i].Cmp(ordinary[i]) != 0 {
			t.Fatalf("moment %d changed", i)
		}
	}
	// A stale determinant cache must not alter replay of the supplied facets.
	audit.Det[0] = proof.DyInt(99)
	recomputed, err := facetproof.FirstMomentsReplay(ctx, proofbound.NewReplayTrialBudget().WorkBudget(ctx), audit, tris)
	if err != nil {
		t.Fatal(err)
	}
	for i := range ordinary {
		if recomputed[i].Cmp(ordinary[i]) != 0 {
			t.Fatalf("uncached moment %d changed", i)
		}
	}
	momentMeter := proofbound.NewReplayTrialBudget()
	if _, err := facetproof.FirstMomentsReplay(ctx, momentMeter.WorkBudget(ctx), audit, tris); err != nil {
		t.Fatal(err)
	}
	mw, mr := momentMeter.Usage()
	mb := momentMeter.Bytes()
	for _, short := range []string{exact, "work", "records", "bytes"} {
		w, r, z := mw, mr, mb
		switch short {
		case "work":
			w--
		case "records":
			r--
		case "bytes":
			z--
		}
		limited, err := proofbound.NewReplayBudgetLimits(w, r, z)
		if err != nil {
			t.Fatal(err)
		}
		_, err = facetproof.FirstMomentsReplay(ctx, limited.WorkBudget(ctx), audit, tris)
		if short == exact && err != nil || short != exact && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("moment budget %s: %v", short, err)
		}
	}
	beta := make([]float64, len(verts))
	lo, hi := r3.Vec{}, r3.Vec{X: 1, Y: 1, Z: 1}
	normal, err := facetproof.FacetedExtremeError(proofbound.NewWorkBudget(ctx), verts, beta, beta, lo, hi)
	if err != nil {
		t.Fatal(err)
	}
	bounded, err := facetproof.FacetedExtremeErrorReplay(ctx, proofbound.NewReplayTrialBudget().WorkBudget(ctx), verts, beta, beta, lo, hi)
	if err != nil || math.Float64bits(normal) != math.Float64bits(bounded) {
		t.Fatalf("extrema parity: %v", err)
	}
}
