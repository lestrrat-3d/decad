package decad

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
)

func TestBrakePrivateAncestryPreservesPublicSource(t *testing.T) {
	leaf := &facetedrecord.SourcePatch{Producer: 7, Ordinal: 2}
	private := &facetedrecord.SourcePatch{Producer: 100, Operand: 1, Parent: leaf}
	outer := &facetedrecord.SourcePatch{Producer: 8, Operand: 0, Ordinal: 1, Parent: private}
	face := &topo.Face{Source: leaf}
	makeRemapper := func() *brakeSourceRemapper {
		return &brakeSourceRemapper{
			ctx: t.Context(), budget: proofbound.NewBrakeReplayBudget(),
			private: [2]*topo.Body{{Origin: provenance.Ref{Producer: 100}}, {Origin: provenance.Ref{Producer: 101}}},
			faces:   [2][]*topo.Face{{face}, nil}, resultID: 102,
			original: make([]*facetedrecord.SourcePatch, 0, 8), mapped: make([]*facetedrecord.SourcePatch, 0, 8), path: make([]*facetedrecord.SourcePatch, 0, 8),
		}
	}
	got, err := makeRemapper().strip(outer)
	if err != nil || got == nil || got == outer || got.Producer != outer.Producer || got.Operand != outer.Operand || got.Ordinal != outer.Ordinal || got.Parent != leaf {
		t.Fatalf("public ancestry changed: %v", err)
	}
	face.Source = &facetedrecord.SourcePatch{Producer: 7, Ordinal: 2}
	if _, err := makeRemapper().strip(outer); !errors.Is(err, ErrUnsupported) {
		t.Fatalf("changed source accepted: %v", err)
	}
	cycle := &facetedrecord.SourcePatch{Producer: 9}
	cycle.Parent = cycle
	if _, err := makeRemapper().strip(cycle); !errors.Is(err, ErrUnsupported) {
		t.Fatalf("cycle accepted: %v", err)
	}
}

func TestBrakeFinalWarningProof(t *testing.T) {
	ctx := t.Context()
	meter := proofbound.NewBrakeReplayBudget()
	allowance := 1.25
	w := opdiagnostic.Record{Operation: int(meshbool.OpCut), Code: opdiagnostic.CodeChainAllowanceApplied, Operand: 0, PairChordLimit: 1, TouchedBound: 1.1, ChainAllowance: &allowance}
	used := [2]bool{}
	if err := brakeCheckFinalChain(ctx, w, 1, [2]float64{allowance, allowance}, &used, meter); err != nil || !used[0] {
		t.Fatalf("valid warning: %v", err)
	}
	if err := brakeCheckFinalChain(ctx, w, 1, [2]float64{allowance, allowance}, &used, meter); !errors.Is(err, ErrUnsupported) {
		t.Fatalf("duplicate warning accepted: %v", err)
	}
	summaries, err := opdiagnostic.NewReplayUnionSummaries(ctx, meter, 2)
	if err != nil {
		t.Fatal(err)
	}
	first := opdiagnostic.Record{Code: opdiagnostic.CodeGapShiftApplied, Operand: 0, DisplacementBound: .01, AllowedReach: .1}
	second := opdiagnostic.Record{Code: opdiagnostic.CodeGapShiftApplied, Operand: 1, DisplacementBound: .02, AllowedReach: .1}
	if err := summaries.AddPair(ctx, []opdiagnostic.Record{first, second}, 1); err != nil {
		t.Fatal(err)
	}
	records, err := summaries.Records(ctx)
	if err != nil || len(records) != 2 || records[0].Operand != 0 || records[1].Operand != 1 || records[0].DisplacementBound != .01 || records[1].DisplacementBound != .02 {
		t.Fatalf("warning mapping failed: count=%d error=%v", len(records), err)
	}
}
