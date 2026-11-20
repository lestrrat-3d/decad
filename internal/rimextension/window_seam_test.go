package rimextension

// CI: isolated-tests

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestWindowNativeSeamAtTranslatedMaximum(t *testing.T) {
	p := loftrecord.Payload{SourceIdentity: new(loftrecord.SourceIdentity), AuthenticatedReconstruction: true, Xform: r3.Identity()}
	for i, pr := range []*momentinput.Profile{&p.Profile0, &p.Profile1} {
		z := .25 + .5*float64(i)
		vs := []momentinput.Point2{{U: -1, V: z}, {U: 1, V: z}, {U: 1, V: 2}, {U: -1, V: 2}}
		for j, a := range vs {
			pr.Outer.Segments = append(pr.Outer.Segments, momentinput.LineSeg{Start: a, End: vs[(j+1)%len(vs)], TEnd: 1})
		}
		frame, err := r3.NewFrame(r3.Vec{Y: float64(i)}, r3.Vec{X: 1}, r3.Vec{Z: 1})
		if err != nil {
			t.Fatal(err)
		}
		if i == 0 {
			p.Frame0 = frame
		} else {
			p.Frame1 = frame
		}
	}
	run := func(want bool) {
		t.Helper()
		seam, ok, e := loftWindowSeam(context.Background(), &p, r3.Vec{X: 8, Y: 4, Z: 2}, scalar(4.25), scalar(2.5), scalar(2), scalar(3), 1, proofbound.NewStrictReplayBudget())
		if ok != want || e != nil {
			t.Fatalf("accepted %t error %v", ok, e)
		}
		if ok && (seam.x.Cmp(scalar(9)) != 0 || seam.toeZ.Cmp(scalar(2.375)) != 0 || seam.heelY.Cmp(scalar(4.5)) != 0) {
			t.Fatal("wrong native stop, toe or heel")
		}
	}
	run(true)
	p.ConstructionDelta = 1e-8
	run(false)
	p.ConstructionDelta = 0
	p.Alignment = []int{1}
	run(false)
	p.Alignment = nil
	p.Profile1.Outer.Segments[1] = momentinput.LineSeg{Start: momentinput.Point2{U: 1, V: .75}, End: momentinput.Point2{U: .95, V: 2}, TEnd: 1}
	run(false)
}

func TestWindowForeignFaceTouchesExpandedBox(t *testing.T) {
	w := SourceWindow{Side0: r3.Vec{}, Side1: r3.Vec{X: 1, Z: 1}, XMin: 0, XMax: 1, YMin: 0, YMax: 1, OffsetMin: 0, OffsetMax: .5}
	v := []r3.Vec{{X: .25, Y: .5, Z: 0}, {X: .5, Y: .5, Z: .25}, {X: .25, Y: .75, Z: 0}}
	run := func(errBound float64, want bool) {
		t.Helper()
		ok, e := WindowSourcesCovered(context.Background(), v, [][3]int{{0, 1, 2}}, []int{1}, []float64{errBound, 0, 0}, map[int]bool{0: true}, w, proofbound.NewStrictReplayBudget())
		if ok != want || e != nil {
			t.Fatalf("bound %g covered %t error %v", errBound, ok, e)
		}
	}
	run(.0625, true)
	run(.125, false)
}

func TestWindowWitnessRequiresCapAndBossInTheirOpenCells(t *testing.T) {
	w := SourceWindow{Side0: r3.Vec{Z: 2}, Side1: r3.Vec{X: 1, Z: 2}, XMin: -.5, XMax: .5, YMin: -1, YMax: 1, OffsetMin: -1.25, OffsetMax: .25}
	mesh := Mesh{
		Vertices:  []r3.Vec{{X: -.45, Z: 1.4}, {X: -.30, Z: 1.4}, {X: -.30, Z: 1.6}, {X: -.05, Y: -.75, Z: 1}, {X: .05, Y: -.75, Z: 1}, {Y: -.25, Z: 1}},
		Triangles: [][3]int{{0, 1, 2}, {3, 4, 5}}, Sources: []int{1, 2},
	}
	beta := []float64{.025, .025, .025, .025, .025, .025}
	run := func(want bool) {
		t.Helper()
		ok, e := WindowWitnesses(context.Background(), mesh, beta, Groups{Cap: 1, Boss: 2}, w, r3.Vec{}, 1, 0, 1, 1, proofbound.NewStrictReplayBudget())
		if ok != want || e != nil {
			t.Fatalf("witnesses %t error %v", ok, e)
		}
	}
	run(true)
	mesh.Sources[0] = 2
	run(false)
	mesh.Sources[0] = 1
	for i := 3; i < 6; i++ {
		mesh.Vertices[i].Y = -mesh.Vertices[i].Y
	}
	run(false)
	for i := 3; i < 6; i++ {
		mesh.Vertices[i].Y = -mesh.Vertices[i].Y
	}
	beta[0] = .55
	run(false)
	beta[0] = .025
	for i := range 3 {
		mesh.Vertices[i].Z -= .55
	}
	run(false)
}
