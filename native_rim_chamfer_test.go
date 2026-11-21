package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func TestRetainedRimFiniteBandChart(t *testing.T) {
	t.Parallel()
	center := rimVec(r3.Vec{})
	rim := func() facetedrecord.Payload {
		var p facetedrecord.Payload
		for _, y := range []float64{0, .25} {
			for _, v := range []r3.Vec{{X: 2}, {Z: 2}, {X: -2}, {Z: -2}} {
				v = v.Scale(1 - y/2)
				v.Y = y
				p.Verts = append(p.Verts, v)
			}
		}
		add := func(g int, f [3]int) { p.Src = append(p.Src, g); p.Tris = append(p.Tris, f) }
		for i := range 4 {
			j := (i + 1) % 4
			add(2, [3]int{i, i + 4, j})
			add(2, [3]int{j, i + 4, j + 4})
		}
		add(0, [3]int{0, 1, 2})
		add(0, [3]int{0, 2, 3})
		add(1, [3]int{4, 7, 6})
		add(1, [3]int{4, 6, 5})
		return p
	}
	p := rim()
	check := func(p *facetedrecord.Payload) bool {
		v, e := rimBandChart(t.Context(), p, 2, center, proofbound.NewStrictReplayBudget())
		if e != nil {
			t.Fatal(e)
		}
		return v
	}
	if !check(&p) || !rimCapChart(t.Context(), &p, 2, map[int]bool{1: true}, map[int]bool{0: true}) {
		t.Fatal("valid cone and cap charts rejected")
	}
	caps := map[int]bool{1: true}
	cheek := map[int]bool{0: true}
	if !rimCapChartAt(t.Context(), &p, 2, caps, nil, cheek, rimRat(0)) {
		t.Fatal("aligned cheek at the lower ring rejected")
	}
	if rimCapChartAt(t.Context(), &p, 2, caps, nil, cheek, rimRat(.00001)) {
		t.Fatal("cheek displaced from the certified heel accepted")
	}
	if rimCapChartAt(t.Context(), &p, 2, caps, nil, map[int]bool{3: true}, rimRat(0)) {
		t.Fatal("unregistered lower face accepted")
	}
	p.Verts[5].Y = .26
	if check(&p) {
		t.Fatal("nonplanar cap ring accepted")
	}
	p = rim()
	p.Tris[0][1], p.Tris[0][2] = p.Tris[0][2], p.Tris[0][1]
	if check(&p) {
		t.Fatal("reversed cone facet accepted")
	}
	p = rim()
	p.Tris[10][1], p.Tris[10][2] = p.Tris[10][2], p.Tris[10][1]
	if rimCapChart(t.Context(), &p, 2, map[int]bool{1: true}, map[int]bool{0: true}) {
		t.Fatal("reversed cap facet accepted")
	}
	p = rim()
	p.Verts[1] = p.Verts[3]
	if check(&p) {
		t.Fatal("collapsed angular map accepted")
	}
}

func TestRetainedRimRejectsNarrowStockAboveHinge(t *testing.T) {
	t.Parallel()
	frame, e := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: -1})
	if e != nil {
		t.Fatal(e)
	}
	group := func(y float64) facetedrecord.Group {
		return facetedrecord.Group{Retained: true, Surface: surfacegeom.Plane{Frame: frame}, CapRecord: &topo.CapRecord{Frame: frame, Xform: r3.Identity(), Z: y, ZDenoted: rimRat(y)}}
	}
	b := &topo.Body{Solid: true}
	p := facetedrecord.Payload{Verts: []r3.Vec{{X: 1}, {X: -1}, {Z: 1}, {Y: .001}, {X: .000001, Y: .001}, {Z: .000001, Y: .001}}, Tris: [][3]int{{0, 1, 2}}, Src: []int{0}, MeshBound: .002, Groups: []facetedrecord.Group{group(0), group(.001)}}
	box := measurement.Box{Min: r3.Vec{X: -2, Z: -2}, Max: r3.Vec{X: 2, Z: 2}}
	b.Payload = p
	if !upperRimStock(t.Context(), b, box, rimRat(.0005)) {
		t.Fatal("separated stock declined")
	}
	p.Tris = append(p.Tris, [3]int{3, 4, 5})
	p.Src = append(p.Src, 1)
	b.Payload = p
	if upperRimStock(t.Context(), b, box, rimRat(.0005)) {
		t.Fatal("thin upper stock admitted")
	}
	p.Groups[1] = group(.0005)
	b.Payload = p
	if upperRimStock(t.Context(), b, box, rimRat(.0005)) {
		t.Fatal("zero-width source collar admitted")
	}
}

func TestRetainedRimProtectsMovedTool(t *testing.T) {
	t.Parallel()
	pad := &topo.Body{Bounds: measurement.Box{Min: r3.Vec{X: 0, Y: 0, Z: 0}, Max: r3.Vec{X: 1, Y: 1, Z: 1}}}
	tool := &topo.Body{Bounds: measurement.Box{Min: r3.Vec{X: 1.005, Y: 0, Z: 0}, Max: r3.Vec{X: 2, Y: 1, Z: 1}}}
	stock := rimMovedBounds(pad, r3.Vec{})
	if !measurement.BoxesDisjoint(stock, rimMovedBounds(tool, r3.Vec{})) {
		t.Fatal("nominal tool overlaps")
	}
	if measurement.BoxesDisjoint(stock, rimMovedBounds(tool, r3.Vec{X: -.01})) {
		t.Fatal("translated tool intrusion missed")
	}
}

func TestRetainedRimSupportCancellation(t *testing.T) {
	t.Parallel()
	one := big.NewRat(1, 1)
	eps := big.NewRat(1, 1<<55)
	lo, _ := rimSupportInterval(rimSub(one, eps), one)
	_, hi := rimSupportInterval(new(big.Rat).Neg(rimSub(one, eps)), one)
	if lo != -math.Ldexp(1, -55) || hi != math.Ldexp(1, -55) {
		t.Fatalf("support interval bounds %g, %g", lo, hi)
	}
}

func TestRetainedRimExhaustedPublication(t *testing.T) {
	t.Parallel()
	p := facetedrecord.Payload{Verts: []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}, Tris: [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}}
	budget := proofbound.NewStrictReplayBudget()
	if e := budget.SpendWork(t.Context(), proofbound.ReplayTrialWork); e != nil {
		t.Fatal(e)
	}
	out, ok, err := publishRim(t.Context(), topo.Env{}, p, p.Tris, budget)
	var limit *proofbound.WorkLimitError
	if ok || out.Body != nil || !errors.As(err, &limit) {
		t.Fatalf("published %t, error %v", ok, err)
	}
}

func TestRetainedRimProtectedTriangle(t *testing.T) {
	t.Parallel()
	old := facetedrecord.Payload{Verts: []r3.Vec{{}, {X: 1}, {Y: 1}, {X: 4}, {X: 5}, {X: 4, Y: 1}}, Tris: [][3]int{{0, 1, 2}, {3, 4, 5}}}
	classify := func(p facetedrecord.Payload) bool {
		local, ok, err := rimLocalTriangles(t.Context(), old, p, r3.Vec{}, r3.Vec{X: 1, Y: 1, Z: 1}, proofbound.NewStrictReplayBudget())
		if err != nil {
			t.Fatal(err)
		}
		return ok && len(local) == 1
	}
	if !classify(old) {
		t.Fatal("original outside triangle declined")
	}
	edited := old
	edited.Verts = append([]r3.Vec(nil), old.Verts...)
	edited.Verts[3].Y = .1
	if classify(edited) {
		t.Fatal("modified outside triangle accepted")
	}
	edited = old
	edited.Tris = old.Tris[:1]
	if classify(edited) {
		t.Fatal("missing outside triangle accepted")
	}
}

func TestRetainedRimSeamContact(t *testing.T) {
	t.Parallel()
	old := facetedrecord.Payload{Verts: []r3.Vec{{}, {X: 2}, {Y: 1}, {Y: -1}}, Tris: [][3]int{{0, 1, 2}, {1, 0, 3}}}
	p := old
	p.Verts = append([]r3.Vec(nil), old.Verts...)
	p.Verts[2] = r3.Vec{X: .25, Y: -.5}
	budget := proofbound.NewStrictReplayBudget()
	local, ok, err := rimLocalTriangles(t.Context(), old, p, r3.Vec{Y: -.5}, r3.Vec{X: 2, Y: 1}, budget)
	if !ok || err != nil || len(local) != 2 {
		t.Fatalf("local facets %d, error %v", len(local), err)
	}
	if err := loftmesh.LoftCrossingAuditBounded(budget.WorkBudget(t.Context()), p.Verts, local, loftmesh.MaxLoftAuditTriangles, solidFromMeshScans); err == nil {
		t.Fatal("overlap across shared edge admitted")
	}
}

func TestRetainedRimCancelledPublication(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	p := facetedrecord.Payload{Verts: []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}, Tris: [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}}
	out, ok, err := publishRim(ctx, topo.Env{}, p, p.Tris, proofbound.NewStrictReplayBudget())
	if ok || out.Body != nil || !errors.Is(err, context.Canceled) {
		t.Fatalf("published %t, error %v", ok, err)
	}
}

func TestRetainedRimPublicationCounters(t *testing.T) {
	t.Parallel()
	b := proofbound.NewStrictReplayBudget()
	ctx := t.Context()
	if err := b.SpendWork(ctx, 197); err != nil {
		t.Fatal(err)
	}
	if ok, e := b.BeginRetainedRimPublication(ctx, 16385, 8); ok || e != nil {
		t.Fatal("oversized shell admitted")
	}
	w, r := b.Usage()
	n := b.Bytes()
	if ok, e := b.BeginRetainedRimPublication(ctx, 4, 4); !ok || e != nil {
		t.Fatalf("admission %t, %v", ok, e)
	}
	w2, r2 := b.Usage()
	if w != w2 || r != r2 || n != b.Bytes() {
		t.Fatal("admission reset cumulative counters")
	}
	if ok, e := b.BeginRetainedRimPublication(ctx, 4, 4); ok || e != nil {
		t.Fatal("second admission succeeded")
	}
	var limit *proofbound.WorkLimitError
	if e := b.SpendWork(ctx, 320000000); !errors.As(e, &limit) {
		t.Fatalf("exhaustion %v", e)
	}
	if ok, e := b.BeginRetainedRimPublication(ctx, 4, 4); ok || !errors.As(e, &limit) {
		t.Fatal("exhausted meter recovered")
	}
}

func TestRetainedRimCancellationDuringPublication(t *testing.T) {
	t.Parallel()
	mid := newCancelAfterContext(t.Context(), 5)
	p := facetedrecord.Payload{Verts: []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}, Tris: [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}}
	out, ok, err := publishRim(mid, topo.Env{}, p, p.Tris, proofbound.NewStrictReplayBudget())
	if ok || out.Body != nil || !errors.Is(err, context.Canceled) {
		t.Fatalf("published %t, error %v", ok, err)
	}
}
