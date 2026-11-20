package rimextension

// CI: isolated-tests

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func TestTransverseWitnessRequiresCompleteDisplacementBall(t *testing.T) {
	// The cylinder axis is Y in this chart. The interval x in (1.4,2.02)
	// lies in one quadrant. Cap Y in the world is chart Z=1.
	w := SourceWindow{Side0: r3.NewVec(2, 0, 1), Side1: r3.NewVec(1.7, 0, 1), XMin: 1.4, XMax: 2.02, YMin: -.35, YMax: .15, OffsetMin: -.32, OffsetMax: .03}
	n := windowNative{cx: scalar(0), cz: scalar(0), r: scalar(2)}
	for _, tc := range []struct {
		name string
		cap  bool
		tri  []r3.Vec
	}{
		{"plane", true, []r3.Vec{r3.NewVec(1.75, -.25, 1), r3.NewVec(1.95, -.25, 1), r3.NewVec(1.82, -.05, 1)}},
		{"cylinder", false, []r3.Vec{r3.NewVec(1.8, .03, .8), r3.NewVec(1.8, .14, .8), r3.NewVec(1.8, .1, .98)}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			mesh := Mesh{Vertices: tc.tri, Triangles: [][3]int{{0, 1, 2}}, Sources: []int{3}}
			for _, test := range []struct {
				beta float64
				want bool
			}{{.001, true}, {.06, false}} {
				ok, err := transverseWitness(context.Background(), mesh, []float64{test.beta, test.beta, test.beta}, 3, tc.cap, w, n, lift(w.Side0), 1, scalar(.3), proofbound.NewStrictReplayBudget())
				if err != nil || ok != test.want {
					t.Fatalf("beta %g accepted=%t err=%v", test.beta, ok, err)
				}
			}
		})
	}
}

func TestTransverseStopRolesRequireTheSelectedSourceWall(t *testing.T) {
	var p loftrecord.Payload
	for i, pr := range []*momentinput.Profile{&p.Profile0, &p.Profile1} {
		f, err := r3.NewFrame(r3.Vec{Y: float64(i)}, r3.Vec{X: 1}, r3.Vec{Z: 1})
		if err != nil {
			t.Fatal(err)
		}
		if i == 0 {
			p.Frame0 = f
		} else {
			p.Frame1 = f
		}
		x := 1 - float64(i)
		join := momentinput.Point2{U: x, V: 2}
		pr.Outer.Segments = []momentinput.CurveSegment{
			momentinput.LineSeg{Start: join, End: momentinput.Point2{U: 3, V: 2}, TEnd: 1},
			momentinput.LineSeg{Start: join, End: momentinput.Point2{U: x, V: 4}, TEnd: 1},
		}
	}
	p.Walls = 4
	p.Cell = [][2]int{{0, 0}, {0, 0}, {0, 1}, {0, 1}}
	p.Side = []uint8{0, 1, 0, 1}
	p.Segment = [][]int{{0, 1}}
	b := new(topo.Body)
	wall := func(role string) BoundFace {
		v := &topo.Face{Body: b, Origins: []provenance.Ref{{Role: role}}}
		return BoundFace{Body: b, Final: v, Native: v}
	}
	a := Admission{Stop: []BoundFace{wall("side(0,0,0)"), wall("side(0,1,1)")}}
	c := Candidate{StopWall: a.Stop[0].Final}
	run := func(want bool) {
		t.Helper()
		ok, err := transverseStopRoles(t.Context(), a, c, p, 1, scalar(2), scalar(.75), scalar(.5), scalar(.25), scalar(.5), scalar(2), proofbound.NewStrictReplayBudget())
		if ok != want || err != nil {
			t.Fatalf("accepted %t, error %v", ok, err)
		}
	}
	run(true)
	a.Stop[0].Native, a.Stop[1].Native = a.Stop[1].Native, a.Stop[0].Native
	run(false)
	a.Stop[0].Native = a.Stop[1].Native
	run(false)
}
