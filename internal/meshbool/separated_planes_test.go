package meshbool

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func TestSeparatedCarrierFaces(t *testing.T) {
	frame := func(z float64) r3.Frame {
		f, e := r3.NewFrame(r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
		if e != nil {
			t.Fatal(e)
		}
		return f
	}
	tag := surfacegeom.Plane{Frame: frame(0)}
	a := &topo.Face{Surface: tag, RetainedTag: true, PlacedPlane: &tag}
	b := &topo.Face{Surface: surfacegeom.Plane{Frame: frame(1)}, CapRecord: &topo.CapRecord{Frame: frame(1), Xform: r3.Identity()}, HasAxialDelta: true}
	ma := &BoolMesh{Verts: []r3.Vec{{}, {X: 1}, {Y: 1}}, Tris: [][3]int{{0, 1, 2}}}
	mb := &BoolMesh{Verts: []r3.Vec{{Z: 1}, {X: 1, Z: 1}, {Y: 1, Z: 1}}, Tris: [][3]int{{0, 1, 2}}}
	test := func(want bool) {
		t.Helper()
		got, err := separatedCarrierFaces(faceFacets{source: a, facets: []int{0}}, faceFacets{source: b, facets: []int{0}}, ma, mb, proofbound.NewWorkBudget(context.Background()))
		if err != nil || got != want {
			t.Fatalf("got=%t want=%t error=%v", got, want, err)
		}
	}
	test(true)
	limited, e := proofbound.NewReplayBudgetLimits(1, 100, 1<<20)
	if e != nil {
		t.Fatal(e)
	}
	_, e = separatedCarrierFaces(faceFacets{source: a, facets: []int{0}}, faceFacets{source: b, facets: []int{0}}, ma, mb, limited.WorkBudget(t.Context()))
	if _, ok := errors.AsType[*proofbound.WorkLimitError](e); !ok {
		t.Fatalf("expected a shared work limit, got %v", e)
	}
	canceled, cancel := context.WithCancel(context.Background())
	cancel()
	_, err := separatedCarrierFaces(faceFacets{source: a, facets: []int{0}}, faceFacets{source: b, facets: []int{0}}, ma, mb, proofbound.NewWorkBudget(canceled))
	if !errors.Is(err, context.Canceled) {
		t.Fatalf("canceled proof returned %v", err)
	}
	ma.Verts[0].Z = .01
	test(false)
	ma.Verts[0].Z = 0
	a.PlacedPlane = nil
	test(false)
	a.PlacedPlane = &tag
	b.HasAxialDelta = false
	test(false)
	b.HasAxialDelta = true
	b.Surface = surfacegeom.Cylinder{}
	test(false)
	b.Surface = surfacegeom.Plane{Frame: frame(1)}
	b.CapRecord.Z = 1.1
	b.CapRecord.ZDenoted = big.NewRat(0, 1)
	b.AxialDelta = .1
	test(true)
	b.CapRecord.ZDenoted = nil
	test(false)
	b.CapRecord.Z = 0
	b.AxialDelta = 0
	other, err := r3.NewFrame(r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	if err != nil {
		t.Fatal(err)
	}
	b.CapRecord.Frame = other
	test(false)
	b.CapRecord.Frame = frame(0)
	test(false)
}
