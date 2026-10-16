package apitest_test

import (
	"context"
	"encoding/json"
	"math"
	"os"
	"runtime"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func validationRodBody(b testing.TB, doc *decad.Document, cx, cy, r float64) *decad.Body {
	b.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(b, err)
	center := s.CreatePoint(cx, cy)
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(b.Context())
	require.NoError(b, err)
	body, err := decad.Extrude(b.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(b, err)
	tr, err := r3.Translation(r3.Vec{X: 0, Y: 0, Z: -6})
	require.NoError(b, err)
	moved, err := decad.Transform(b.Context(), body, tr)
	require.NoError(b, err)
	return moved
}

func validationConsumer(t testing.TB) (func(context.Context) (*decad.Body, error), func(*decad.Body)) {
	t.Helper()
	doc := decad.New()
	if os.Getenv("VALIDATION_CONSUMER") == "washer" {
		target := discBody(t, doc, 0, 15, 10)
		tool := washerBodySymmetric(t, doc, 8, 3, 11)
		return func(ctx context.Context) (*decad.Body, error) { return decad.Cut(ctx, target, tool) }, func(got *decad.Body) {
			if got == nil || !got.IsSolid() || !anyFaceIsFaceted(got) {
				t.Fatal("washer output is not a faceted solid")
			}
			if n := len(got.Lumps()); n != 2 {
				t.Fatalf("washer lumps: %d", n)
			}
			m, err := got.Volume()
			require.NoError(t, err)
			want := math.Pi * (15*15 - 8*8 + 3*3) * 10
			v, bound := m.Value.Base(), m.Bound.Base()
			if math.IsNaN(v) || math.IsInf(v, 0) || math.IsNaN(bound) || math.IsInf(bound, 0) || bound < 0 || math.Abs(v-want) > bound {
				t.Fatalf("washer volume=%g bound=%g want=%g", v, bound, want)
			}
		}
	}
	if os.Getenv("VALIDATION_CONSUMER") != "rods" {
		t.Fatal("unknown consumer")
	}
	a := validationRodBody(t, doc, 0, 0, 4)
	c := validationRodBody(t, doc, 0, 0, 3)
	rot, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(90))
	require.NoError(t, err)
	tr, err := r3.Translation(r3.Vec{Y: -4, Z: 2})
	require.NoError(t, err)
	xf, err := rot.Then(tr)
	require.NoError(t, err)
	c, err = decad.Transform(t.Context(), c, xf)
	require.NoError(t, err)
	interval := func(body *decad.Body) (float64, float64) {
		m, err := body.Volume()
		require.NoError(t, err)
		v, b := m.Value.Base(), m.Bound.Base()
		if math.IsNaN(v) || math.IsInf(v, 0) || v <= 0 || math.IsNaN(b) || math.IsInf(b, 0) || b < 0 {
			t.Fatal("invalid rods volume")
		}
		return math.Nextafter(v-b, math.Inf(-1)), math.Nextafter(v+b, math.Inf(1))
	}
	alo, ahi := interval(a)
	clo, chi := interval(c)
	return func(ctx context.Context) (*decad.Body, error) { return decad.Union(ctx, a, c) }, func(got *decad.Body) {
		if got == nil || !got.IsSolid() || !anyFaceIsFaceted(got) {
			t.Fatal("rods output is not a faceted solid")
		}
		if n := len(got.Lumps()); n != 1 {
			t.Fatalf("rods lumps: %d", n)
		}
		lo, hi := interval(got)
		if !(lo > max(ahi, chi) && hi < math.Nextafter(alo+clo, math.Inf(-1))) {
			t.Fatalf("rods interval [%g,%g] operands [%g,%g] [%g,%g]", lo, hi, alo, ahi, clo, chi)
		}
	}
}

func BenchmarkValidationOwnerConsumer(b *testing.B) {
	mode := os.Getenv("VALIDATION_MODE")
	if mode != "warm" && mode != "cold" && mode != "gc" {
		b.Fatal("unknown mode")
	}
	b.StopTimer()
	if mode == "warm" {
		op, check := validationConsumer(b)
		got, err := op(b.Context())
		require.NoError(b, err)
		check(got)
	}
	b.StartTimer()
	for b.Loop() {
		b.StopTimer()
		op, check := validationConsumer(b)
		if mode == "gc" {
			runtime.GC()
			runtime.GC()
		}
		b.StartTimer()
		got, err := op(b.Context())
		b.StopTimer()
		require.NoError(b, err)
		check(got)
		b.StartTimer()
	}
}

func TestValidationConsumerAdmission(t *testing.T) {
	op, check := validationConsumer(t)
	got, err := op(t.Context())
	require.NoError(t, err)
	check(got)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(1000), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	if !mesh.BoundaryVerified() {
		t.Fatal("boundary verification missing")
	}
}

func TestValidationConsumerHeap(t *testing.T) {
	// One fresh process per sample. Both sources keep identical operands/results alive.
	op, check := validationConsumer(t)
	read := func(phase string) {
		var m runtime.MemStats
		runtime.ReadMemStats(&m)
		v := struct {
			Phase                                                                string
			HeapAlloc, HeapInuse, HeapObjects, TotalAlloc, Mallocs, PauseTotalNs uint64
			NumGC                                                                uint32
			GCCPUFraction                                                        float64
		}{phase, m.HeapAlloc, m.HeapInuse, m.HeapObjects, m.TotalAlloc, m.Mallocs, m.PauseTotalNs, m.NumGC, m.GCCPUFraction}
		data, err := json.Marshal(v)
		require.NoError(t, err)
		t.Logf("HEAP %s", data)
	}
	runtime.GC()
	runtime.GC()
	read("before")
	got, err := op(t.Context())
	require.NoError(t, err)
	read("after_call")
	runtime.GC()
	read("one_gc")
	runtime.GC()
	read("two_gc")
	runtime.KeepAlive(got)
	runtime.KeepAlive(op)
	check(got)
	got = nil
	op = nil
	check = nil
	runtime.GC()
	runtime.GC()
	read("released")
}
