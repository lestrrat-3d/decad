package topo

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const replayTestCaseExact = "exact"

type sealCancelContext struct {
	context.Context //nolint:containedctx // The context is confined to one synchronous replay or test.
	polls           int
	at              int
}

func (c *sealCancelContext) Err() error {
	c.polls++
	if c.polls >= c.at {
		return context.Canceled
	}
	return c.Context.Err()
}

func TestSealPrismBuildFacesCopyBudget(t *testing.T) {
	run := func(ctx context.Context, numerator *big.Int, budget *proofbound.ReplayBudget) (bool, error) {
		body, face, _, _ := prismSealFixture(t) //nolint:contextcheck // The test fixture uses t.Context().
		body.prismBuildSeal = nil
		face.CapRecord.ZDenoted = new(big.Rat).SetInt(numerator)
		ok, err := body.SealPrismBuildFacesReplay(ctx, []*Face{face}, budget)
		if !ok && body.prismBuildSeal != nil {
			t.Fatal("failed copy published a seal")
		}
		return ok, err
	}
	numerator := new(big.Int).Lsh(big.NewInt(1), 4095)
	trial := proofbound.NewReplayTrialBudget()
	ok, err := run(t.Context(), numerator, trial)
	if err != nil || !ok {
		t.Fatalf("seal admitted=%t error=%v", ok, err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	if work < 128+8+64+uint64(len(numerator.Bits())) {
		t.Fatalf("copy undercharged: work=%d", work)
	}
	small := proofbound.NewReplayTrialBudget()
	if ok, err := run(t.Context(), big.NewInt(1), small); err != nil || !ok {
		t.Fatalf("small seal admitted=%t error=%v", ok, err)
	}
	smallWork, _ := small.Usage()
	if work-smallWork != uint64(len(numerator.Bits())-1) {
		t.Fatalf("limb charge difference=%d", work-smallWork)
	}
	for _, tc := range []struct {
		name string
		work uint64
		want bool
	}{{replayTestCaseExact, work, true}, {"one short", work - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(tc.work, records, bytes)
			if err != nil {
				t.Fatal(err)
			}
			ok, err := run(t.Context(), numerator, budget)
			if ok != tc.want || (err == nil) != tc.want {
				t.Fatalf("seal admitted=%t error=%v", ok, err)
			}
		})
	}
	oversize := new(big.Int).Lsh(big.NewInt(1), 4096)
	if ok, err := run(t.Context(), oversize, proofbound.NewReplayTrialBudget()); ok || err != nil {
		t.Fatalf("oversize seal admitted=%t error=%v", ok, err)
	}
	ctx := &sealCancelContext{Context: t.Context(), at: 15}
	if ok, err := run(ctx, numerator, proofbound.NewReplayTrialBudget()); ok || !errors.Is(err, context.Canceled) {
		t.Fatalf("cancelled seal admitted=%t error=%v", ok, err)
	}
}

func TestSealPrismBuildFacesPreallocationBudget(t *testing.T) {
	const validation = 3
	const preallocation = 1 + 64 + 8 + 64
	for _, tc := range []struct {
		name     string
		limit    uint64
		wantWork uint64
	}{{replayTestCaseExact, validation + preallocation, validation + preallocation},
		{"one short", validation + preallocation - 1, validation}} {
		t.Run(tc.name, func(t *testing.T) {
			body, face, _, _ := prismSealFixture(t)
			body.prismBuildSeal = nil
			budget, err := proofbound.NewReplayBudgetLimits(tc.limit, 1, 1)
			if err != nil {
				t.Fatal(err)
			}
			ok := body.SealPrismBuildFaces([]*Face{face}, func(n uint64) error {
				return budget.SpendWork(t.Context(), n)
			})
			work, _ := budget.Usage()
			if ok || body.prismBuildSeal != nil || work != tc.wantWork {
				t.Fatalf("seal admitted=%t work=%d want=%d", ok, work, tc.wantWork)
			}
		})
	}
	body, face, _, _ := prismSealFixture(t)
	body.prismBuildSeal = nil
	ctx := &sealCancelContext{Context: t.Context(), at: 4}
	budget := proofbound.NewReplayTrialBudget()
	ok := body.SealPrismBuildFaces([]*Face{face}, func(n uint64) error {
		return budget.SpendWork(ctx, n)
	})
	work, _ := budget.Usage()
	if ok || body.prismBuildSeal != nil || work != validation || !errors.Is(ctx.Err(), context.Canceled) {
		t.Fatalf("cancelled seal admitted=%t work=%d", ok, work)
	}
}

func TestStrictPrismBuildSealReplayBudgetAndRollback(t *testing.T) {
	run := func(ctx context.Context, budget *proofbound.ReplayBudget) (*Body, bool, error) {
		body, face, profile, frame := prismSealFixture(t) //nolint:contextcheck // The test fixture uses t.Context().
		body.prismBuildSeal = nil
		ok, err := body.SealPrismBuildFacesReplay(ctx, []*Face{face}, budget)
		if err != nil || !ok {
			return body, ok, err
		}
		ok, err = body.SealPrismBuildPayloadReplay(ctx, frame, r3.Identity(), 0, 1, 0, 0, 0,
			prismplacement.LevelDenotation{}, prismplacement.LevelDenotation{},
			[]momentinput.Profile{profile}, budget)
		return body, ok, err
	}
	trial := proofbound.NewReplayTrialBudget()
	body, ok, err := run(t.Context(), trial)
	if err != nil || !ok || !body.HasStrictPrismBuildSeal() {
		t.Fatalf("trial seal admitted=%t strict=%t err=%v", ok, body.HasStrictPrismBuildSeal(), err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	if work == 0 || records == 0 || bytes == 0 {
		t.Fatalf("missing seal charge: work=%d records=%d bytes=%d", work, records, bytes)
	}
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		want                 bool
	}{
		{replayTestCaseExact, work, records, bytes, true},
		{"work short", work - 1, records, bytes, false},
		{"records short", work, records - 1, bytes, false},
		{"bytes short", work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			if err != nil {
				t.Fatal(err)
			}
			body, ok, err := run(t.Context(), budget)
			if tc.want {
				if err != nil || !ok || !body.HasStrictPrismBuildSeal() {
					t.Fatalf("exact seal admitted=%t strict=%t err=%v", ok, body.HasStrictPrismBuildSeal(), err)
				}
			} else if err == nil || ok || body.HasStrictPrismBuildSeal() {
				t.Fatalf("short seal admitted=%t strict=%t err=%v", ok, body.HasStrictPrismBuildSeal(), err)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	body, ok, err = run(ctx, proofbound.NewReplayTrialBudget())
	if err == nil || ok || body.HasStrictPrismBuildSeal() {
		t.Fatalf("cancelled seal admitted=%t strict=%t err=%v", ok, body.HasStrictPrismBuildSeal(), err)
	}
}

func prismSealFixture(t *testing.T) (*Body, *Face, momentinput.Profile, r3.Frame) {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	if err != nil {
		t.Fatal(err)
	}
	body := &Body{}
	vertex := &Vertex{Position: r3.Vec{X: 1}, Bound: units.Millimeters(0.1)}
	edge := &Edge{Start: vertex, End: vertex,
		Curve: surfacegeom.Circle3{Center: r3.Vec{}, Axis: r3.Vec{Z: 1}, Radius: units.Millimeters(1)}}
	face := &Face{Body: body, Origins: []provenance.Ref{{Producer: 1, Role: "capStart"}},
		Surface: surfacegeom.Plane{Frame: frame}, HasAxialDelta: true,
		CapRecord: &CapRecord{Frame: frame, Xform: r3.Identity(), ZDenoted: big.NewRat(5, 1)},
		Loops:     []*Loop{{Outer: true, CoEdges: []CoEdge{{Edge: edge}}}}}
	edge.Faces = []*Face{face}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: sectionrecord.Point2{}, End: sectionrecord.Point2{U: 1}, TEnd: 1},
	}}}
	spend := func(uint64) error { return nil }
	if !body.SealPrismBuildFaces([]*Face{face}, spend) ||
		!body.SealPrismBuildPayload(frame, r3.Identity(), 0, 1, 0, 0, 0,
			prismplacement.LevelDenotation{}, prismplacement.LevelDenotation{},
			[]momentinput.Profile{profile}, spend) || !body.HasStrictPrismBuildSeal() {
		t.Fatal("source fixture did not seal")
	}
	ok, err := body.ReplayPrismBuildPayload(t.Context(), frame, r3.Identity(), 0, 1, 0, 0, 0,
		prismplacement.LevelDenotation{}, prismplacement.LevelDenotation{},
		[]momentinput.Profile{profile}, proofbound.NewReplayTrialBudget())
	if err != nil || !ok {
		t.Fatalf("clean source payload admitted=%t err=%v", ok, err)
	}
	return body, face, profile, frame
}

func TestStrictPrismBuildSealBindsCapPointersAndContents(t *testing.T) {
	body, face, _, _ := prismSealFixture(t)
	check := func(want bool) {
		t.Helper()
		ok, err := body.ReplayPrismBuiltFace(t.Context(), face, proofbound.NewReplayTrialBudget())
		if err != nil || ok != want {
			t.Fatalf("source face admitted=%t err=%v", ok, err)
		}
	}
	check(true)
	original := face.CapRecord
	copyCap := *original
	face.CapRecord = &copyCap
	check(false)
	face.CapRecord = original
	copyRat := new(big.Rat).Set(original.ZDenoted)
	face.CapRecord.ZDenoted = copyRat
	check(false)
	face.CapRecord.ZDenoted = body.prismBuildSeal.faces[0].capDenotedSource
	face.CapRecord.ZDenoted.SetInt64(6)
	check(false)
}

func TestReplayPrismBuiltFaceComparisonBudget(t *testing.T) {
	body, face, _, _ := prismSealFixture(t)
	trial := proofbound.NewReplayTrialBudget()
	ok, err := body.ReplayPrismBuiltFace(t.Context(), face, trial)
	if err != nil || !ok {
		t.Fatalf("face admitted=%t error=%v", ok, err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	if work < 1+1+128+4+64+1 {
		t.Fatalf("face and edge comparisons undercharged: work=%d", work)
	}
	for _, tc := range []struct {
		name string
		work uint64
		want bool
	}{{replayTestCaseExact, work, true}, {"one short", work - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(tc.work, records, bytes)
			if err != nil {
				t.Fatal(err)
			}
			ok, err := body.ReplayPrismBuiltFace(t.Context(), face, budget)
			if ok != tc.want || (err == nil) != tc.want {
				t.Fatalf("face admitted=%t error=%v", ok, err)
			}
		})
	}
}

func TestStrictPrismBuildSealBindsPayloadDenotations(t *testing.T) {
	z0 := prismplacement.DenoteLevel(big.NewRat(0, 1), 0, 0)
	z1 := prismplacement.DenoteLevel(big.NewRat(1, 1), 1, 0)
	run := func(ctx context.Context, budget *proofbound.ReplayBudget) (*Body, bool, error) {
		body, _, profile, frame := prismSealFixture(t) //nolint:contextcheck // The test fixture uses t.Context().
		body.prismBuildSeal.payload = nil
		ok, err := body.SealPrismBuildPayloadReplay(ctx, frame, r3.Identity(), 0, 1, 0, 0, 0,
			z0, z1, []momentinput.Profile{profile}, budget)
		return body, ok, err
	}
	meter := proofbound.NewReplayTrialBudget()
	body, ok, err := run(t.Context(), meter)
	if err != nil || !ok {
		t.Fatalf("denoted seal admitted=%t error=%v", ok, err)
	}
	w, r := meter.Usage()
	b := meter.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		want    bool
	}{
		{replayTestCaseExact, w, r, b, true},
		{"work short", w - 1, r, b, false},
		{"records short", w, r - 1, b, false},
		{"bytes short", w, r, b - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			limit, e := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if e != nil {
				t.Fatal(e)
			}
			built, admitted, e := run(t.Context(), limit)
			if tc.want {
				if e != nil || !admitted || !built.HasStrictPrismBuildSeal() {
					t.Fatalf("exact admitted=%t strict=%t error=%v", admitted, built.HasStrictPrismBuildSeal(), e)
				}
			} else if e == nil || admitted || built.HasStrictPrismBuildSeal() {
				t.Fatalf("short admitted=%t strict=%t error=%v", admitted, built.HasStrictPrismBuildSeal(), e)
			}
		})
	}
	_, _, profile, frame := prismSealFixture(t)
	check := func(a, b prismplacement.LevelDenotation, want bool) {
		t.Helper()
		ok, err := body.ReplayPrismBuildPayload(t.Context(), frame, r3.Identity(), 0, 1, 0, 0, 0,
			a, b, []momentinput.Profile{profile}, proofbound.NewReplayTrialBudget())
		if err != nil || ok != want {
			t.Fatalf("denoted replay admitted=%t error=%v", ok, err)
		}
	}
	check(z0, z1, true)
	check(prismplacement.DenoteLevel(big.NewRat(0, 1), 0, 0), z1, false)
	check(z0, prismplacement.DenoteLevel(big.NewRat(1, 1), 1, 0), false)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if built, admitted, e := run(ctx, proofbound.NewReplayTrialBudget()); e == nil || admitted || built.HasStrictPrismBuildSeal() {
		t.Fatalf("cancelled admitted=%t strict=%t error=%v", admitted, built.HasStrictPrismBuildSeal(), e)
	}
}

func TestStrictPrismBuildSealBindsProducerBounds(t *testing.T) {
	body, _, _, _ := prismSealFixture(t) //nolint:dogsled // This test checks only the refusal and its error.
	meter := proofbound.NewReplayTrialBudget()
	box, ok, err := body.ReplayPrismBuiltBounds(t.Context(), meter)
	if err != nil || !ok || box != body.Bounds {
		t.Fatalf("original box admitted=%t err=%v", ok, err)
	}
	work, records := meter.Usage()
	if work != 64 || records != 0 || meter.Bytes() != 0 {
		t.Fatalf("box charges work=%d records=%d bytes=%d", work, records, meter.Bytes())
	}
	short, err := proofbound.NewReplayBudgetLimits(63, 1, 1)
	if err != nil {
		t.Fatal(err)
	}
	if _, ok, err := body.ReplayPrismBuiltBounds(t.Context(), short); err == nil || ok {
		t.Fatalf("one-short box admitted=%t err=%v", ok, err)
	}
	body.Bounds.Min.X = math.Copysign(0, -1)
	if _, ok, err := body.ReplayPrismBuiltBounds(t.Context(), proofbound.NewReplayTrialBudget()); err != nil || ok {
		t.Fatalf("signed-zero box mutation admitted=%t err=%v", ok, err)
	}
}

func TestStrictPrismBuildSealRejectsSignedZeroMutations(t *testing.T) {
	negativeZero := math.Copysign(0, -1)
	checkFace := func(mutate func(*Face)) {
		t.Helper()
		body, face, _, _ := prismSealFixture(t)
		mutate(face)
		ok, err := body.ReplayPrismBuiltFace(t.Context(), face, proofbound.NewReplayTrialBudget())
		if err != nil || ok {
			t.Fatalf("signed-zero face mutation admitted=%t err=%v", ok, err)
		}
	}
	checkFace(func(face *Face) { face.AxialDelta = negativeZero })
	checkFace(func(face *Face) { face.CapRecord.Z = negativeZero })
	checkFace(func(face *Face) {
		f, err := r3.NewFrame(r3.Vec{X: negativeZero}, r3.Vec{X: 1}, r3.Vec{Y: 1})
		if err != nil {
			t.Fatal(err)
		}
		face.Surface = surfacegeom.Plane{Frame: f}
	})
	checkFace(func(face *Face) {
		edge := face.Loops[0].CoEdges[0].Edge
		edge.Start.Position.X = negativeZero
	})
	checkFace(func(face *Face) {
		edge := face.Loops[0].CoEdges[0].Edge
		curve := edge.Curve.(surfacegeom.Circle3)
		curve.Center.X = negativeZero
		edge.Curve = curve
	})

	checkPayload := func(mutate func(momentinput.Profile, r3.Frame, r3.Transform) (momentinput.Profile, r3.Frame, r3.Transform,
		float64, float64)) {
		t.Helper()
		body, _, profile, frame := prismSealFixture(t)
		profile, frame, xform, z0, section := mutate(profile, frame, r3.Identity())
		ok, err := body.ReplayPrismBuildPayload(t.Context(), frame, xform, z0, 1, 0, 0, section,
			prismplacement.LevelDenotation{}, prismplacement.LevelDenotation{},
			[]momentinput.Profile{profile}, proofbound.NewReplayTrialBudget())
		if err != nil || ok {
			t.Fatalf("signed-zero payload mutation admitted=%t err=%v", ok, err)
		}
	}
	checkPayload(func(p momentinput.Profile, f r3.Frame, x r3.Transform) (momentinput.Profile, r3.Frame, r3.Transform, float64, float64) {
		line := p.Outer.Segments[0].(sectionrecord.LineSeg)
		line.Start.U = negativeZero
		p.Outer.Segments[0] = line
		return p, f, x, 0, 0
	})
	checkPayload(func(p momentinput.Profile, _ r3.Frame, x r3.Transform) (momentinput.Profile, r3.Frame, r3.Transform, float64, float64) {
		f, _ := r3.NewFrame(r3.Vec{X: negativeZero}, r3.Vec{X: 1}, r3.Vec{Y: 1})
		return p, f, x, 0, 0
	})
	checkPayload(func(p momentinput.Profile, f r3.Frame, _ r3.Transform) (momentinput.Profile, r3.Frame, r3.Transform, float64, float64) {
		x, _ := r3.Translation(r3.Vec{X: negativeZero})
		return p, f, x, 0, 0
	})
	checkPayload(func(p momentinput.Profile, f r3.Frame, x r3.Transform) (momentinput.Profile, r3.Frame, r3.Transform, float64, float64) {
		return p, f, x, negativeZero, 0
	})
	checkPayload(func(p momentinput.Profile, f r3.Frame, x r3.Transform) (momentinput.Profile, r3.Frame, r3.Transform, float64, float64) {
		return p, f, x, 0, negativeZero
	})
	for _, tc := range []struct {
		name                              string
		z0, z1, z0Delta, z1Delta, section float64
	}{
		{"z0", negativeZero, 1, 0, 0, 0},
		{"z0 delta", 0, 1, negativeZero, 0, 0},
		{"z1 delta", 0, 1, 0, negativeZero, 0},
		{"section delta", 0, 1, 0, 0, negativeZero},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body, _, profile, frame := prismSealFixture(t)
			ok, err := body.ReplayPrismBuildPayload(t.Context(), frame, r3.Identity(),
				tc.z0, tc.z1, tc.z0Delta, tc.z1Delta, tc.section,
				prismplacement.LevelDenotation{}, prismplacement.LevelDenotation{},
				[]momentinput.Profile{profile}, proofbound.NewReplayTrialBudget())
			if err != nil || ok {
				t.Fatalf("signed-zero %s admitted=%t err=%v", tc.name, ok, err)
			}
		})
	}
	body, _, profile, frame := prismSealFixture(t)
	body.prismBuildSeal.payload = nil
	if !body.SealPrismBuildPayload(frame, r3.Identity(), 0, 0, 0, 0, 0,
		prismplacement.LevelDenotation{}, prismplacement.LevelDenotation{},
		[]momentinput.Profile{profile}, func(uint64) error { return nil }) {
		t.Fatal("zero end level did not seal")
	}
	ok, err := body.ReplayPrismBuildPayload(t.Context(), frame, r3.Identity(), 0, negativeZero,
		0, 0, 0, prismplacement.LevelDenotation{}, prismplacement.LevelDenotation{},
		[]momentinput.Profile{profile}, proofbound.NewReplayTrialBudget())
	if err != nil || ok {
		t.Fatalf("signed-zero z1 admitted=%t err=%v", ok, err)
	}
}
