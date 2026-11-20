package rimextension

// CI: isolated-tests

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestShoulderArcHalfspace(t *testing.T) {
	arc := momentinput.ArcSeg{Start: momentinput.Point2{U: -4, V: 3}, End: momentinput.Point2{U: -4, V: -3}, TEnd: 1}
	for _, wing := range []int{-1, 1} {
		end := arc.Start
		if wing < 0 {
			end = arc.End
		}
		if !ShoulderRoundBeyondEnd(arc, end, wing) {
			t.Fatal("excluded valid quadrant", wing)
		}
	}
	arc.Start, arc.End = arc.End, arc.Start
	if ShoulderRoundBeyondEnd(arc, arc.Start, 1) {
		t.Fatal("accepted the major sweep")
	}
	s := momentinput.ArcSeg{Start: momentinput.Point2{U: -4, V: 3}, End: momentinput.Point2{U: 0, V: -5}, TEnd: 1}
	if windowLoftSegmentUpper(s).Sign() != 0 {
		t.Fatal("finite arc did not certify the stop")
	}
	s.Start, s.End = s.End, s.Start
	if windowLoftSegmentUpper(s).Cmp(scalar(5)) != 0 {
		t.Fatal("major arc escaped the claimed support")
	}
}

func TestShoulderPlanePrefixAndFiniteTerminal(t *testing.T) {
	cx, cz := scalar(0), scalar(0)
	n := windowNative{cx: cx, cz: cz, r: scalar(5), x0: scalar(-3), z0: scalar(4.01), last: scalar(-3), m: scalar(.75), out: 1}
	check := func(want bool) {
		t.Helper()
		d, ok, e := shoulderDiskDomain(context.Background(), n, scalar(-6), scalar(.375), proofbound.NewStrictReplayBudget())
		if e != nil || ok != want {
			t.Fatalf("domain %t %v", ok, e)
		}
		if ok && (d.switchX.Cmp(scalar(-5)) <= 0 || d.minOrdinate.Sign() <= 0 || d.terminalGapUpper.Cmp(scalar(.009)) <= 0) {
			t.Fatal("missing prefix or terminal width")
		}
	}
	check(true)
	n.z0 = scalar(4)
	check(false)
	n.z0 = scalar(5)
	check(false)
}

func TestShoulderWitnessMustClearConeHinge(t *testing.T) {
	w := SourceWindow{Side0: r3.Vec{Z: 2}, Side1: r3.Vec{X: 1, Z: 2}, XMin: -.5, XMax: .5, YMin: -1, YMax: 1, OffsetMin: -1.25, OffsetMax: .25}
	m := Mesh{Vertices: []r3.Vec{{X: -.45, Z: 1.4}, {X: -.30, Z: 1.4}, {X: -.30, Z: 1.6}, {X: -.05, Y: .75, Z: 1}, {X: .05, Y: .75, Z: 1}, {Y: .25, Z: 1}},
		Triangles: [][3]int{{0, 1, 2}, {3, 4, 5}}, Sources: []int{1, 2}}
	beta := []float64{.025, .025, .025, .025, .025, .025}
	for _, tc := range []struct {
		hinge float64
		want  bool
	}{{.4, true}, {.49, false}, {.6, false}} {
		ok, e := windowWitnessesAt(context.Background(), m, beta, Groups{Cap: 1, Boss: 2}, w, scalar(0), scalar(0), scalar(1), scalar(0), scalar(tc.hinge), 1, -1, proofbound.NewStrictReplayBudget())
		if ok != tc.want || e != nil {
			t.Fatalf("hinge %g accepted %t error %v", tc.hinge, ok, e)
		}
	}
}
