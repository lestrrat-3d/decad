package prismtess

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func TestCircleGeometryReplay(t *testing.T) {
	frame, e := r3.NewFrame(r3.Vec{X: 4, Y: -2, Z: 7}, r3.Vec{X: 1, Y: 1}, r3.Vec{X: -1, Y: 1})
	if e != nil {
		t.Fatal(e)
	}
	x, e := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(.71))
	if e != nil {
		t.Fatal(e)
	}
	p := prismrecord.Payload{Frame: frame, Xform: x, Z0: 0, Z1: 4}
	faces := [3]*topo.Face{new(topo.Face), new(topo.Face), new(topo.Face)}
	seg := sectionrecord.CircleSeg{Radius: units.Millimeters(3), CCW: true, TEnd: 1}
	local, lift, charge, e := massmoment.ReplayPrismChord(t.Context(), frame, x, .2, proofbound.NewReplayTrialBudget())
	if e != nil {
		t.Fatal(e)
	}
	sample, e := tessellation.CircleSamplesReplay(t.Context(), seg, faces[0], local, 4, proofbound.NewReplayTrialBudget())
	if e != nil {
		t.Fatal(e)
	}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) (*topo.Mesh, []float64, error) {
		return circleGeometryReplay(ctx, p, sample, faces, lift, b)
	}
	b := proofbound.NewReplayTrialBudget()
	mesh, store, e := run(t.Context(), b)
	if e != nil {
		t.Fatal(e)
	}
	n := len(sample.Samples)
	if len(mesh.Vertices) != 2*n || len(mesh.Triangles) != 4*n-4 || len(store) != 2*n || mesh.BoundaryOK || mesh.SymDiffOK {
		t.Fatal("invalid geometry dimensions or proof status")
	}
	if len(mesh.VertexBound) != 2*n || mesh.ReplayFaceBoundCount() != 3 || mesh.Bound <= 0 {
		t.Fatal("missing bounded face or vertex records")
	}
	for i, face := range faces {
		bound, ok, err := mesh.ReplayFaceBound(t.Context(), face, proofbound.NewReplayTrialBudget())
		if err != nil || !ok || bound <= 0 || bound > mesh.Bound {
			t.Fatalf("face %d bound=%g found=%t error=%v", i, bound, ok, err)
		}
	}
	for i, pt := range sample.Samples {
		if mesh.Vertices[2*i] != p.Point(pt.U, pt.V, p.Z0) || mesh.Vertices[2*i+1] != p.Point(pt.U, pt.V, p.Z1) {
			t.Fatalf("vertex %d differs", i)
		}
	}
	for i := range 2 * n {
		if mesh.Source[i] != faces[0] {
			t.Fatalf("wall %d source differs", i)
		}
	}
	measureBudget := proofbound.NewReplayTrialBudget()
	area, volume, err := circleMeasuresReplay(t.Context(), p, seg, sample, mesh, store, charge, measureBudget)
	if err != nil || area <= 0 || volume <= 0 {
		t.Fatalf("area=%g volume=%g error=%v", area, volume, err)
	}
	mw, mr := measureBudget.Usage()
	mb := measureBudget.Bytes()
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		if mode == replayBudgetTestRecords && mr == 0 || mode == replayBudgetTestBytes && mb == 0 {
			continue
		}
		m := proofbound.NewReplayTrialBudget()
		ww, rr, bb := proofbound.ReplayTrialWork-mw, proofbound.ReplayTrialRecords-mr, proofbound.ReplayTrialBytes-mb
		switch mode {
		case replayBudgetTestWork:
			ww++
		case replayBudgetTestRecords:
			rr++
		case replayBudgetTestBytes:
			bb++
		}
		if e = m.SpendWork(t.Context(), ww); e != nil {
			t.Fatal(e)
		}
		if e = m.ReserveRecords(t.Context(), rr); e != nil {
			t.Fatal(e)
		}
		if e = m.ReserveBytes(t.Context(), bb); e != nil {
			t.Fatal(e)
		}
		_, _, err = circleMeasuresReplay(t.Context(), p, seg, sample, mesh, store, charge, m)
		if mode == replayBudgetTestExact && err != nil || mode != replayBudgetTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("measure %s: %v", mode, err)
		}
	}
	w, r := b.Usage()
	bytes := b.Bytes()
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		m := proofbound.NewReplayTrialBudget()
		ww, rr, bb := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-bytes
		switch mode {
		case replayBudgetTestWork:
			ww++
		case replayBudgetTestRecords:
			rr++
		case replayBudgetTestBytes:
			bb++
		}
		if e = m.SpendWork(t.Context(), ww); e != nil {
			t.Fatal(e)
		}
		if e = m.ReserveRecords(t.Context(), rr); e != nil {
			t.Fatal(e)
		}
		if e = m.ReserveBytes(t.Context(), bb); e != nil {
			t.Fatal(e)
		}
		out, _, err := run(t.Context(), m)
		if mode == replayBudgetTestExact && err != nil || mode != replayBudgetTestExact && (!errors.Is(err, decaderr.ErrUnsupported) || out != nil) {
			t.Fatalf("%s: %v", mode, err)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if _, _, e = run(ctx, proofbound.NewReplayTrialBudget()); !errors.Is(e, context.Canceled) {
		t.Fatalf("cancel: %v", e)
	}
}
