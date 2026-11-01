package decad

import (
	"context"
	"errors"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

const (
	circleTestBytes   = "source-bytes"
	circleTestExact   = "source-exact"
	circleTestWork    = "source-work"
	circleTestRecords = "source-records"
)

func TestCirclePlacementReplaySnapshot(t *testing.T) {
	world := sketch.NewWorld()
	doc := New()
	sk, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatal(err)
	}
	center := sk.CreatePoint(0, 0)
	sk.Fix(center)
	sk.CreateCircle(center, 2)
	if _, err := sk.Solve(t.Context()); err != nil {
		t.Fatal(err)
	}
	source, err := Extrude(t.Context(), doc, sk, sk.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along})
	if err != nil {
		t.Fatal(err)
	}
	rot, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Degrees(31))
	if err != nil {
		t.Fatal(err)
	}
	shift, err := r3.Translation(r3.Vec{X: 3, Y: -2, Z: 1})
	if err != nil {
		t.Fatal(err)
	}
	motion, err := rot.Then(shift)
	if err != nil {
		t.Fatal(err)
	}
	placed, err := Transform(t.Context(), source, motion)
	if err != nil {
		t.Fatal(err)
	}
	beforeLive := doc.Bodies()
	beforeHistory := doc.OperationHistory().Operations()
	for _, body := range []*Body{source, placed} {
		p := body.n.Payload.(prismrecord.Payload)
		if _, err := prismrecord.CaptureCircleBuildFaces(t.Context(), &body.n, doc.circleBuilds[body], proofbound.NewCircleReplayBudget()); err != nil {
			t.Fatalf("circle producer: witness=%t walks=%t z0den=%t z1den=%t deltas=%g,%g,%g error=%v", doc.circleBuilds[body] != nil,
				p.Walks != nil, p.Z0Denoted != (prismplacement.LevelDenotation{}), p.Z1Denoted != (prismplacement.LevelDenotation{}),
				p.SectionDelta, p.Z0Delta, p.Z1Delta, err)
		}
	}
	run := func(ctx context.Context, m *proofbound.ReplayBudget) error {
		s, err := captureCirclePlacement(ctx, doc, placed, m)
		if err != nil {
			return err
		}
		bound, err := s.compositionBound(ctx, doc, m)
		if err != nil {
			return err
		}
		if !(bound > 0) {
			t.Fatal("composition bound is not positive")
		}
		return nil
	}
	measured := proofbound.NewCircleReplayBudget()
	if err := run(t.Context(), measured); err != nil {
		t.Fatal(err)
	}
	w, r := measured.Usage()
	z := measured.Bytes()
	t.Logf("placement work=%d records=%d bytes=%d", w, r, z)
	for _, kind := range []string{circleTestExact, circleTestWork, circleTestRecords, circleTestBytes} {
		t.Run(kind, func(t *testing.T) {
			m := proofbound.NewCircleReplayBudget()
			ww, rr, zz := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-z
			switch kind {
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
			if kind == circleTestExact && err != nil || kind != circleTestExact && !errors.Is(err, ErrUnsupported) {
				t.Fatalf("budget: %v", err)
			}
			if kind != circleTestExact {
				var limit *proofbound.WorkLimitError
				resource := map[string]string{circleTestWork: "multiway_work", circleTestRecords: "multiway_records", circleTestBytes: "replay_bytes"}[kind]
				if !errors.As(err, &limit) || limit.Resource != resource {
					t.Fatalf("wrong budget refusal: %v", err)
				}
			}
		})
	}
	m := proofbound.NewCircleReplayBudget()
	s, err := captureCirclePlacement(t.Context(), doc, placed, m)
	if err != nil {
		t.Fatal(err)
	}
	bound, err := s.compositionBound(t.Context(), doc, m)
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("composition rounding bound=%g mm", bound)
	for _, p := range []r3.Vec{{}, {X: 2, Y: -2, Z: 4}, {X: -2, Y: 2}} {
		if !circleExactCompositionWithin(source.n.Payload.Transform(), motion, placed.n.Payload.Transform(), p, bound) {
			t.Fatal("composition error exceeds bound")
		}
	}
	witness := doc.circlePlacements[placed]
	oldMotion := witness.motion
	witness.motion = r3.Identity()
	if err := s.revalidate(t.Context(), doc, m); !errors.Is(err, ErrUnsupported) {
		t.Fatal("accepted changed motion")
	}
	witness.motion = oldMotion
	oldComposed := witness.composed
	witness.composed = r3.Identity()
	if err := s.revalidate(t.Context(), doc, m); !errors.Is(err, ErrUnsupported) {
		t.Fatal("accepted changed composition")
	}
	witness.composed = oldComposed
	op := doc.producers[placed]
	oldSource := op.inputs[0].Body
	op.inputs[0].Body = placed
	if err := s.revalidate(t.Context(), doc, m); !errors.Is(err, ErrUnsupported) {
		t.Fatal("accepted changed input")
	}
	op.inputs[0].Body = oldSource
	oldRole := op.inputs[0].Role
	op.inputs[0].Role = "changed"
	if err := s.revalidate(t.Context(), doc, m); !errors.Is(err, ErrUnsupported) {
		t.Fatal("accepted changed input role")
	}
	op.inputs[0].Role = oldRole
	payload := placed.n.Payload.(prismrecord.Payload)
	changed := payload
	changed.Z1++
	placed.n.Payload = changed
	if err := s.revalidate(t.Context(), doc, m); !errors.Is(err, ErrUnsupported) {
		t.Fatal("accepted changed source payload")
	}
	placed.n.Payload = payload
	oldLength := payload.Walks.Outer[0].Length
	payload.Walks.Outer[0].Length++
	if err := s.revalidate(t.Context(), doc, m); !errors.Is(err, ErrUnsupported) {
		t.Fatal("accepted changed recorded walk")
	}
	payload.Walks.Outer[0].Length = oldLength
	face := placed.n.Lumps[0].Shells[0].Faces[0]
	oldArea := face.Area
	face.Area++
	if err := s.revalidate(t.Context(), doc, m); !errors.Is(err, ErrUnsupported) {
		t.Fatal("accepted changed construction face")
	}
	face.Area = oldArea
	if err := s.revalidate(t.Context(), doc, proofbound.NewCircleReplayBudget()); !errors.Is(err, ErrUnsupported) {
		t.Fatal("accepted changed meter")
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if err := s.revalidate(ctx, doc, m); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancellation: %v", err)
	}
	if !slices.Equal(beforeLive, doc.Bodies()) || !slices.Equal(beforeHistory, doc.OperationHistory().Operations()) {
		t.Fatal("placement replay changed the document")
	}
	secondMotion, err := r3.Translation(r3.Vec{X: -1, Y: 5})
	if err != nil {
		t.Fatal(err)
	}
	placedAgain, err := Transform(t.Context(), placed, secondMotion)
	if err != nil {
		t.Fatal(err)
	}
	runChain := func(ctx context.Context, budget *proofbound.ReplayBudget) error {
		chain, err := captureCircleSource(ctx, doc, placedAgain, budget)
		if err != nil {
			return err
		}
		if chain.base != source || len(chain.steps) != 2 {
			t.Fatal("wrong source chain")
		}
		view, err := chain.circleReplayView(ctx, budget)
		if err != nil {
			return err
		}
		faces, err := chain.steps[0].placedFaces.Faces(ctx, &placedAgain.n, budget)
		if err != nil {
			return err
		}
		if view.body != &placedAgain.n || view.faces != faces {
			t.Fatal("source view changed builder faces")
		}
		_, err = chain.compositionBound(ctx, doc, budget)
		if err != nil {
			return err
		}
		_, err = chain.linearCompositionBound(ctx, doc, budget)
		if err != nil {
			return err
		}
		_, err = chain.liftRoundBound(ctx, doc, budget)
		return err
	}
	chainBudget := proofbound.NewCircleReplayBudget()
	if err := runChain(t.Context(), chainBudget); err != nil {
		t.Fatal(err)
	}
	cw, cr := chainBudget.Usage()
	cz := chainBudget.Bytes()
	t.Logf("source chain work=%d records=%d bytes=%d", cw, cr, cz)
	for _, kind := range []string{circleTestExact, circleTestWork, circleTestRecords, circleTestBytes} {
		t.Run("chain_"+kind, func(t *testing.T) {
			budget := proofbound.NewCircleReplayBudget()
			ww, rr, zz := proofbound.ReplayTrialWork-cw, proofbound.ReplayTrialRecords-cr, proofbound.ReplayTrialBytes-cz
			switch kind {
			case circleTestWork:
				ww++
			case circleTestRecords:
				rr++
			case circleTestBytes:
				zz++
			}
			if err := budget.SpendWork(t.Context(), ww); err != nil {
				t.Fatal(err)
			}
			if err := budget.ReserveRecords(t.Context(), rr); err != nil {
				t.Fatal(err)
			}
			if err := budget.ReserveBytes(t.Context(), zz); err != nil {
				t.Fatal(err)
			}
			err := runChain(t.Context(), budget)
			if kind == circleTestExact && err != nil || kind != circleTestExact && !errors.Is(err, ErrUnsupported) {
				t.Fatalf("chain budget: %v", err)
			}
			if kind != circleTestExact {
				var limit *proofbound.WorkLimitError
				resource := map[string]string{circleTestWork: "multiway_work", circleTestRecords: "multiway_records", circleTestBytes: "replay_bytes"}[kind]
				if !errors.As(err, &limit) || limit.Resource != resource {
					t.Fatalf("wrong chain limit: %v", err)
				}
			}
		})
	}
	chain, err := captureCircleSource(t.Context(), doc, placedAgain, proofbound.NewCircleReplayBudget())
	if err != nil {
		t.Fatal(err)
	}
	view, err := chain.circleReplayView(t.Context(), chain.budget)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := chain.circleReplayView(t.Context(), proofbound.NewCircleReplayBudget()); !errors.Is(err, ErrUnsupported) {
		t.Fatal("source view accepted another meter")
	}
	cancelled, stop := context.WithCancel(t.Context())
	stop()
	if _, err := chain.circleReplayView(cancelled, chain.budget); !errors.Is(err, context.Canceled) {
		t.Fatalf("source view cancellation: %v", err)
	}
	capRecord := view.faces[1].CapRecord
	denoted := capRecord.ZDenoted
	capRecord.ZDenoted = new(big.Rat).Set(denoted)
	if _, err := chain.circleReplayView(t.Context(), chain.budget); !errors.Is(err, ErrUnsupported) {
		t.Fatal("source view accepted substituted cap denotation")
	}
	capRecord.ZDenoted = denoted
	chainBound, err := chain.compositionBound(t.Context(), doc, chain.budget)
	if err != nil {
		t.Fatal(err)
	}
	linearBound, err := chain.linearCompositionBound(t.Context(), doc, chain.budget)
	if err != nil {
		t.Fatal(err)
	}
	for axis := range 3 {
		if !circleExactLinearWithin([]r3.Transform{source.n.Payload.Transform(), motion, secondMotion}, placedAgain.n.Payload.Transform(), axis, linearBound) {
			t.Fatal("source chain linear composition error exceeds bound")
		}
	}
	for _, p := range []r3.Vec{{}, {X: 2, Y: -2, Z: 4}, {X: -2, Y: 2}} {
		if !circleExactTransformsWithin([]r3.Transform{source.n.Payload.Transform(), motion, secondMotion}, placedAgain.n.Payload.Transform(), p, chainBound) {
			t.Fatal("source chain composition error exceeds bound")
		}
	}
	liftBound, err := chain.liftRoundBound(t.Context(), doc, chain.budget)
	if err != nil {
		t.Fatal(err)
	}
	placedRecord := placedAgain.n.Payload.(prismrecord.Payload)
	for _, p := range []r3.Vec{{X: 2}, {Y: 2, Z: 4}, {X: -2, Z: 2}} {
		if !circleExactLiftWithin(placedRecord, p.X, p.Y, p.Z, liftBound) {
			t.Fatal("frame lift error exceeds bound")
		}
	}
	baseOp := doc.producers[source]
	baseKind := baseOp.kind
	baseOp.kind = OperationCopy
	if err := chain.revalidate(t.Context(), doc, chain.budget); !errors.Is(err, ErrUnsupported) {
		t.Fatal("accepted changed base producer")
	}
	baseOp.kind = baseKind
	ctx2, cancel2 := context.WithCancel(t.Context())
	cancel2()
	if err := chain.revalidate(ctx2, doc, chain.budget); !errors.Is(err, context.Canceled) {
		t.Fatalf("chain cancellation: %v", err)
	}
}

// The comparison evaluates both affine compositions over exact rationals.
func circleExactCompositionWithin(first, next, held r3.Transform, point r3.Vec, bound float64) bool {
	return circleExactTransformsWithin([]r3.Transform{first, next}, held, point, bound)
}

func circleExactLinearWithin(steps []r3.Transform, held r3.Transform, axis int, bound float64) bool {
	rat := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	vector := [3]*big.Rat{rat(0), rat(0), rat(0)}
	vector[axis] = rat(1)
	apply := func(t r3.Transform, p [3]*big.Rat) [3]*big.Rat {
		b := t.Basis()
		columns := [3]r3.Vec{b.EX, b.EY, b.EZ}
		var out [3]*big.Rat
		for i := range out {
			out[i] = rat(0)
			for j, column := range columns {
				values := [3]float64{column.X, column.Y, column.Z}
				out[i].Add(out[i], new(big.Rat).Mul(rat(values[i]), p[j]))
			}
		}
		return out
	}
	initial := vector
	for _, step := range steps {
		vector = apply(step, vector)
	}
	got := apply(held, initial)
	sum := rat(0)
	for i := range vector {
		d := new(big.Rat).Sub(vector[i], got[i])
		sum.Add(sum, d.Mul(d, d))
	}
	b := rat(bound)
	return sum.Cmp(b.Mul(b, b)) <= 0
}

func circleExactTransformsWithin(steps []r3.Transform, held r3.Transform, point r3.Vec, bound float64) bool {
	rat := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	vec := func(v r3.Vec) [3]*big.Rat { return [3]*big.Rat{rat(v.X), rat(v.Y), rat(v.Z)} }
	apply := func(t r3.Transform, p [3]*big.Rat) [3]*big.Rat {
		b := t.Basis()
		x, y, z, tr := vec(b.EX), vec(b.EY), vec(b.EZ), vec(t.Translation())
		var out [3]*big.Rat
		for i := range out {
			out[i] = new(big.Rat).Set(tr[i])
			for j, axis := range [3][3]*big.Rat{x, y, z} {
				out[i].Add(out[i], new(big.Rat).Mul(axis[i], p[j]))
			}
		}
		return out
	}
	exact := vec(point)
	for _, step := range steps {
		exact = apply(step, exact)
	}
	got := apply(held, vec(point))
	sum := new(big.Rat)
	for i := range exact {
		d := new(big.Rat).Sub(exact[i], got[i])
		sum.Add(sum, d.Mul(d, d))
	}
	b := rat(bound)
	return sum.Cmp(b.Mul(b, b)) <= 0
}

func circleExactLiftWithin(p prismrecord.Payload, u, v, z, bound float64) bool {
	rat := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	values := func(v r3.Vec) [3]float64 { return [3]float64{v.X, v.Y, v.Z} }
	o, fu, fv, fn := values(p.Frame.Origin()), values(p.Frame.U()), values(p.Frame.V()), values(p.Frame.N())
	var local [3]*big.Rat
	for i := range local {
		local[i] = rat(o[i])
		for _, term := range [3][2]float64{{fu[i], u}, {fv[i], v}, {fn[i], z}} {
			local[i].Add(local[i], new(big.Rat).Mul(rat(term[0]), rat(term[1])))
		}
	}
	basis := p.Xform.Basis()
	x, y, zz, tr := values(basis.EX), values(basis.EY), values(basis.EZ), values(p.Xform.Translation())
	held := values(p.Point(u, v, z))
	sum := new(big.Rat)
	for i := range held {
		exact := rat(tr[i])
		for j, column := range [3][3]float64{x, y, zz} {
			exact.Add(exact, new(big.Rat).Mul(rat(column[i]), local[j]))
		}
		d := new(big.Rat).Sub(exact, rat(held[i]))
		sum.Add(sum, d.Mul(d, d))
	}
	b := rat(bound)
	return sum.Cmp(b.Mul(b, b)) <= 0
}
