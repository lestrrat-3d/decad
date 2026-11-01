package decad

import (
	"context"
	"errors"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func TestCircleSourceCandidateProducer(t *testing.T) {
	body := publicCirclePrimitive(t)
	doc := body.Document()
	turn, err := r3.Rotation(r3.Vec{X: -2, Y: 3, Z: 1}, units.Radians(-.23))
	if err != nil {
		t.Fatal(err)
	}
	body, err = Transform(t.Context(), body, turn)
	if err != nil {
		t.Fatal(err)
	}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) error {
		source, err := captureCircleSource(ctx, doc, body, b)
		if err != nil {
			return err
		}
		if _, err := source.compositionBound(ctx, doc, b); err != nil {
			return err
		}
		base, err := captureCircleSource(ctx, doc, source.base, b)
		if err != nil {
			return err
		}
		_, err = withCirclePrimitiveCandidate(ctx, base, .2, b, func(mesh *topo.Mesh, faces [3]*topo.Face, b *proofbound.ReplayBudget) (bool, error) {
			old, err := publicCircleCandidate(ctx, source.base, b)
			if err != nil {
				return false, err
			}
			if err := b.SpendWork(ctx, uint64(8*(len(mesh.Vertices)+len(mesh.Triangles)+len(mesh.Source)+len(mesh.VertexBound))+64)); err != nil {
				return false, err
			}
			if mesh.BoundaryOK || mesh.SymDiffOK || len(mesh.Source) == 0 || !slices.Equal(mesh.Vertices, old.Vertices) || !slices.Equal(mesh.Triangles, old.Triangles) || !slices.Equal(mesh.Source, old.Source) || !slices.Equal(mesh.VertexBound, old.VertexBound) || math.Float64bits(mesh.Bound) != math.Float64bits(old.Bound) || math.Float64bits(mesh.CoordBound) != math.Float64bits(old.CoordBound) || math.Float64bits(mesh.AreaSlack) != math.Float64bits(old.AreaSlack) || math.Float64bits(mesh.VolSymDiff) != math.Float64bits(old.VolSymDiff) {
				t.Fatal("candidate output differs from original helper")
			}
			for _, face := range faces {
				a, ok, err := mesh.ReplayFaceBounds.Lookup(ctx, face, b)
				if err != nil {
					return false, err
				}
				if !ok {
					t.Fatal("missing candidate face bound")
				}
				c, ok, err := old.ReplayFaceBounds.Lookup(ctx, face, b)
				if err != nil {
					return false, err
				}
				if !ok || math.Float64bits(a) != math.Float64bits(c) {
					t.Fatal("face bound differs")
				}
			}
			return true, nil
		})
		if err != nil {
			return err
		}
		_, err = withCirclePrimitiveCandidate(ctx, source, .2, b, func(mesh *topo.Mesh, faces [3]*topo.Face, b *proofbound.ReplayBudget) (bool, error) {
			t.Logf("placed candidate chord=%g bound=%g", .2, mesh.Bound)
			if mesh.Bound < .18 {
				t.Fatal("candidate did not exercise a near-threshold chord")
			}
			old, err := publicCircleCandidate(ctx, body, b)
			if err != nil {
				return false, err
			}
			if err := b.SpendWork(ctx, uint64(8*(len(mesh.Vertices)+len(mesh.Triangles)+len(mesh.Source)+len(mesh.VertexBound))+64)); err != nil {
				return false, err
			}
			if mesh.BoundaryOK || mesh.SymDiffOK || mesh.Bound > .2 || !slices.Equal(mesh.Vertices, old.Vertices) || !slices.Equal(mesh.Triangles, old.Triangles) || !slices.Equal(mesh.Source, old.Source) || mesh.Bound < old.Bound || mesh.CoordBound < old.CoordBound || mesh.AreaSlack < old.AreaSlack || mesh.VolSymDiff < old.VolSymDiff || len(mesh.VertexBound) != len(old.VertexBound) {
				t.Fatal("placed candidate changed geometry or narrowed bounds")
			}
			for i := range mesh.VertexBound {
				if mesh.VertexBound[i] < old.VertexBound[i] {
					t.Fatal("placed candidate narrowed vertex bound")
				}
			}
			for _, face := range faces {
				a, ok, err := mesh.ReplayFaceBounds.Lookup(ctx, face, b)
				if err != nil {
					return false, err
				}
				if !ok {
					t.Fatal("missing placed face bound")
				}
				c, ok, err := old.ReplayFaceBounds.Lookup(ctx, face, b)
				if err != nil {
					return false, err
				}
				if !ok || a < c || a > .2 {
					t.Fatal("placed face bound narrowed")
				}
			}
			return true, nil
		})
		if err != nil {
			return err
		}
		return nil
	}
	b := proofbound.NewCircleReplayBudget()
	if err := run(t.Context(), b); err != nil {
		t.Fatal(err)
	}
	w, r := b.Usage()
	z := b.Bytes()
	t.Logf("source candidate work=%d records=%d bytes=%d", w, r, z)
	for _, mode := range []string{circleTestExact, circleTestWork, circleTestRecords, circleTestBytes} {
		t.Run(mode, func(t *testing.T) {
			m := proofbound.NewCircleReplayBudget()
			ww, rr, zz := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-z
			switch mode {
			case circleTestWork:
				ww++
			case circleTestRecords:
				rr++
			case circleTestBytes:
				zz++
			}
			if err := m.SpendWork(t.Context(), ww); err != nil {
				t.Fatal(err)
			}
			if err := m.ReserveRecords(t.Context(), rr); err != nil {
				t.Fatal(err)
			}
			if err := m.ReserveBytes(t.Context(), zz); err != nil {
				t.Fatal(err)
			}
			err := run(t.Context(), m)
			if mode == circleTestExact && err != nil || mode != circleTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("%s: %v", mode, err)
			}
			if mode != circleTestExact {
				if _, ok := errors.AsType[*proofbound.WorkLimitError](err); !ok {
					t.Fatalf("%s limit: %v", mode, err)
				}
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if err := run(ctx, proofbound.NewCircleReplayBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel: %v", err)
	}
}

func TestCircleSourceCandidateMutationDuringReplay(t *testing.T) {
	placed := publicCirclePrimitive(t)
	doc := placed.Document()
	base := doc.OperationHistory().Operations()[0].Outputs()[0]
	budget := proofbound.NewCircleReplayBudget()
	source, err := captureCircleSource(t.Context(), doc, base, budget)
	if err != nil {
		t.Fatal(err)
	}
	face := base.n.Lumps[0].Shells[0].Faces[0]
	old := face.Area
	ctx := &circlePrimitiveMutationContext{Context: t.Context(), mutate: func() { face.Area++ }}
	called := false
	_, err = withCirclePrimitiveCandidate(ctx, source, .2, budget, func(*topo.Mesh, [3]*topo.Face, *proofbound.ReplayBudget) (bool, error) {
		called = true
		return true, nil
	})
	face.Area = old
	if ctx.polls < 500 || called || !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("source mutation: called=%t polls=%d error=%v", called, ctx.polls, err)
	}
}

func TestCircleSourceCandidateChordConsumedByPlacement(t *testing.T) {
	body := publicCirclePrimitive(t)
	doc := body.Document()
	motion, err := r3.Translation(r3.Vec{X: .1, Y: .2, Z: .3})
	if err != nil {
		t.Fatal(err)
	}
	body, err = Transform(t.Context(), body, motion)
	if err != nil {
		t.Fatal(err)
	}
	budget := proofbound.NewCircleReplayBudget()
	source, err := captureCircleSource(t.Context(), doc, body, budget)
	if err != nil {
		t.Fatal(err)
	}
	d, _, err := source.exactAffineAllowance(t.Context(), budget)
	if err != nil || !(d > 0) {
		t.Fatalf("placement displacement=%g error=%v", d, err)
	}
	called := false
	_, err = withCirclePrimitiveCandidate(t.Context(), source, d, budget, func(*topo.Mesh, [3]*topo.Face, *proofbound.ReplayBudget) (bool, error) {
		called = true
		return true, nil
	})
	if called || !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("consumed chord: called=%t error=%v", called, err)
	}
}

func TestCircleAffineDeltaBoundary(t *testing.T) {
	placed := publicCirclePrimitive(t)
	doc := placed.Document()
	base := doc.OperationHistory().Operations()[0].Outputs()[0]
	budget := proofbound.NewCircleReplayBudget()
	source, err := captureCircleSource(t.Context(), doc, base, budget)
	if err != nil {
		t.Fatal(err)
	}
	view, err := source.circleReplayView(t.Context(), budget)
	if err != nil {
		t.Fatal(err)
	}
	_, err = withCirclePrimitiveCandidate(t.Context(), source, .2, budget, func(mesh *topo.Mesh, _ [3]*topo.Face, b *proofbound.ReplayBudget) (bool, error) {
		if err := widenCirclePrimitive(t.Context(), view, mesh, 2, 0, math.Nextafter(.25, 0), b); err != nil {
			return false, err
		}
		for _, delta := range []float64{.25, math.Nextafter(.25, 1)} {
			if err := widenCirclePrimitive(t.Context(), view, mesh, 2, 0, delta, b); !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("delta %g: %v", delta, err)
			}
		}
		return true, nil
	})
	if err != nil {
		t.Fatal(err)
	}
}

func TestCircleSourceCandidateNearChord(t *testing.T) {
	body := publicCirclePrimitive(t)
	turn, err := r3.Rotation(r3.Vec{X: -2, Y: 3, Z: 1}, units.Radians(-.23))
	if err != nil {
		t.Fatal(err)
	}
	body, err = Transform(t.Context(), body, turn)
	if err != nil {
		t.Fatal(err)
	}
	budget := proofbound.NewCircleReplayBudget()
	source, err := captureCircleSource(t.Context(), body.Document(), body, budget)
	if err != nil {
		t.Fatal(err)
	}
	const chord = .183
	_, err = withCirclePrimitiveCandidate(t.Context(), source, chord, budget, func(mesh *topo.Mesh, faces [3]*topo.Face, b *proofbound.ReplayBudget) (bool, error) {
		if mesh.Bound > chord || mesh.Bound < .9*chord {
			t.Fatalf("near chord bound=%g chord=%g", mesh.Bound, chord)
		}
		for _, face := range faces {
			bound, ok, err := mesh.ReplayFaceBounds.Lookup(t.Context(), face, b)
			if err != nil {
				return false, err
			}
			if !ok || bound > chord {
				t.Fatalf("near chord face bound=%g", bound)
			}
		}
		return true, nil
	})
	if err != nil {
		t.Fatal(err)
	}
}
