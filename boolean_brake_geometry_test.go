package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func TestBrakeRetryGeometryAndHistory(t *testing.T) {
	trace := &brakeReplayTrace{}
	ctx := context.WithValue(t.Context(), brakeReplayTraceKey{}, trace)
	var meter *proofbound.ReplayBudget
	ctx = context.WithValue(ctx, brakeReplayMeterKey{}, &meter)
	doc, target, clevis := brakeGalleryFixture(t, ctx)
	trace.stage = ""
	trace.err = nil
	meter = nil
	history := doc.OperationHistory().Operations()
	sources := make(map[producerID][]*topo.Face)
	for _, op := range history {
		for _, b := range op.Outputs() {
			if b == nil {
				continue
			}
			id := b.n.Origin.Producer
			if _, exists := sources[id]; exists {
				t.Fatalf("duplicate source producer %d", id)
			}
			for _, f := range b.Faces() {
				sources[id] = append(sources[id], &f.n)
			}
		}
	}
	// Derive the original pair chord directly from the frozen source bounds.
	// Both operands are faceted with no primitive chord floor.
	a, b := target.n.Bounds, clevis.n.Bounds
	aPad := r3.NewVec(a.Bound.Base(), a.Bound.Base(), a.Bound.Base())
	bPad := r3.NewVec(b.Bound.Base(), b.Bound.Base(), b.Bound.Base())
	alo, ahi := a.Min.Sub(aPad), a.Max.Add(aPad)
	blo, bhi := b.Min.Sub(bPad), b.Max.Add(bPad)
	lo := r3.NewVec(math.Min(alo.X, blo.X), math.Min(alo.Y, blo.Y), math.Min(alo.Z, blo.Z))
	hi := r3.NewVec(math.Max(ahi.X, bhi.X), math.Max(ahi.Y, bhi.Y), math.Max(ahi.Z, bhi.Z))
	expectedChord := hi.Sub(lo).Len() * meshbool.ChordFactor
	replayChord, _, chordErr := booloperand.PairChordReplay(ctx, &target.n, &clevis.n, proofbound.NewBrakeReplayBudget())
	if chordErr != nil || math.Float64bits(replayChord) != math.Float64bits(expectedChord) {
		t.Fatalf("source chord mismatch: got=%.17g want=%.17g error=%v", replayChord, expectedChord, chordErr)
	}
	before := len(history)
	privateStart := doc.nextProducerID()
	result, err := Union(ctx, target, clevis)
	if err != nil {
		var work, records, bytes uint64
		if meter != nil {
			work, records = meter.Usage()
			bytes = meter.Bytes()
		}
		t.Fatalf("public Union: %v; first replay gate=%s: %.512v; work=%d records=%d bytes=%d", err, trace.stage, trace.err, work, records, bytes)
	}
	if meter == nil {
		t.Fatal("public Union did not use the replay meter")
	}
	work, records := meter.Usage()
	t.Logf("public route work=%d records=%d bytes=%d", work, records, meter.Bytes())
	if len(doc.OperationHistory().Operations()) != before+1 {
		t.Fatal("private operations entered public history")
	}
	op, err := doc.OperationHistory().Producer(result)
	if err != nil {
		t.Fatal(err)
	}
	inputs := op.Inputs()
	if op.Kind() != OperationUnion || len(inputs) != 2 || inputs[0].Body != target || inputs[1].Body != clevis {
		t.Fatal("public Union input order changed")
	}
	shifts := [2]bool{}
	for _, w := range result.n.OperationWarnings {
		if w.PublicOperation != int(OperationUnion) || w.Operand < 0 || w.Operand > 1 {
			t.Fatal("public warning identity changed")
		}
		if w.Code == opdiagnostic.CodeGapShiftApplied {
			if shifts[w.Operand] {
				t.Fatal("duplicate public shift warning")
			}
			shifts[w.Operand] = true
		}
	}
	if !shifts[0] || !shifts[1] {
		t.Fatal("public shift warnings missing")
	}
	payload, ok := result.n.Payload.(facetedrecord.Payload)
	if !ok {
		t.Fatal("public Union payload missing")
	}
	for i, g := range payload.Groups {
		if g.Source == nil {
			t.Fatalf("group %d has no source", i)
		}
		for node, depth := g.Source, 0; node != nil; node, depth = node.Parent, depth+1 {
			if depth > brakeSourceNodeLimit || node.Producer >= privateStart && node.Producer <= result.n.Origin.Producer {
				t.Fatal("private producer entered public ancestry")
			}
			sourceFaces, ok := sources[node.Producer]
			if !ok || node.Blend || node.Ordinal < 0 || node.Ordinal >= len(sourceFaces) {
				t.Fatalf("group %d unknown source producer=%d ordinal=%d blend=%t", i, node.Producer, node.Ordinal, node.Blend)
			}
			if sourceFaces[node.Ordinal].Source != node.Parent {
				t.Fatalf("group %d source parent mismatch producer=%d ordinal=%d", i, node.Producer, node.Ordinal)
			}
		}
	}
	for i, f := range result.Faces() {
		found := false
		for _, g := range payload.Groups {
			if f.n.Source == g.Source {
				found = true
				break
			}
		}
		if !found || f.n.Source == nil {
			t.Fatalf("face %d has no exact group source", i)
		}
	}
	volume, err := result.Volume()
	if err != nil {
		t.Fatalf("result volume: %v", err)
	}
	// The blank contains the final three millimeters of the shaft. The slot
	// and pin lie beyond that overlap. The pin lies entirely inside the slot
	// in XY; only its portions above and below the slot remove extra material.
	// The shaft's first seven millimeters lose a circular cap at x=right+1.3.
	right := -140.0 + (42 + 0.6*math.Sqrt(5))
	d := (right + 1.3) - right
	circularCap := 4*math.Acos(d/2) - d*math.Sqrt(4-d*d)
	blank := 7 * (2.8 + 6) * 6
	slot := 7 * (2.8 + 3) * 1.6
	pinOutsideSlot := math.Pi * 1.6 * 1.6 * (6 - 1.6)
	expectedVolume := 4*math.Pi*(34-3) - 7*circularCap + blank - slot - pinOutsideSlot
	if math.Abs(volume.Value.Base()-expectedVolume) > volume.Bound.Base()+1e-9 {
		t.Fatalf("volume mismatch: got=%.17g bound=%.17g expected=%.17g", volume.Value.Base(), volume.Bound.Base(), expectedVolume)
	}
	t.Logf("source chord=%.17g volume=%.17g bound=%.17g analytic=%.17g", expectedChord, volume.Value.Base(), volume.Bound.Base(), expectedVolume)

	mesh, err := result.Tessellate(ctx, units.Millimeters(.12))
	if err != nil {
		t.Fatalf("VerifyAll: %v", err)
	}
	if !mesh.BoundaryVerified() || !mesh.VolumeVerified() {
		t.Fatal("result lacks VerifyAll proof")
	}
}
