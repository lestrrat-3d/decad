package booleaneval

import (
	"context"
	"errors"
	"fmt"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismtess"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func TestInsetCylinderOperandBounds(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{X: 7, Y: 9, Z: -5}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	if err != nil {
		t.Fatal(err)
	}
	radius := 2.0
	height := 3.0
	circle := sectionrecord.CircleSeg{Radius: units.Millimeters(radius), CCW: true, TEnd: 1}
	p := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z1: height, Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}}}
	body, err := prismrecord.Build(t.Context(), topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}}, p, freeform.NewFreeformWork())
	if err != nil {
		t.Fatal(err)
	}
	// A native disk as the first operand must surface an unexpected mesher failure.
	peer := topo.Body{Lumps: body.Lumps, Payload: facetedrecord.Payload{}, Bounds: body.Bounds}
	injected := errors.New("injected operand tessellation failure")
	_, accepted, retryErr := retryRadialInsetUnion(t.Context(), body, &peer, insetErrorMesher{err: injected}, &operandTwins{}, nil, .025, 10, 1)
	if accepted || !errors.Is(retryErr, injected) {
		t.Fatalf("accepted=%t, mesher error=%v", accepted, retryErr)
	}
	role := func(loop, seg int) string { return fmt.Sprintf("side(%d,%d)", loop, seg) }
	m, err := prismtess.Build(t.Context(), body, p, role, .025, true, nil, 0)
	if err != nil {
		t.Fatal(err)
	}
	// Root Tessellate marks this after its VerifyAll boundary checks.
	m.BoundaryOK = true
	before := slices.Clone(m.Vertices)
	originalBound := m.Bound
	alt, ok, err := radialInsetMesh(t.Context(), body, m, .01, proofbound.NewReplayTrialBudget())
	if err != nil || !ok {
		t.Fatalf("alternate mesh accepted=%t error=%v", ok, err)
	}
	if !slices.Equal(before, m.Vertices) || m.Bound != originalBound || alt == m {
		t.Fatal("retry mutated the original mesh")
	}
	if len(alt.Vertices) != len(m.Vertices) || len(alt.Triangles) != len(m.Triangles) || !slices.Equal(alt.Triangles, m.Triangles) {
		t.Fatal("source incidence changed")
	}
	worst := 0.0
	for i, v := range before {
		q := alt.Vertices[i]
		if q.Y != v.Y {
			t.Fatalf("cap moved at vertex %d", i)
		}
		d := proofbound.DvLenUpper(proofbound.HeldDelta(v, q))
		worst = max(worst, d)
		if d > .01 {
			t.Fatalf("vertex %d moved %g", i, d)
		}
	}
	if !(worst > 0 && alt.Bound >= originalBound+worst) {
		t.Fatalf("largest motion=%g mesh bound=%g baseline=%g", worst, alt.Bound, originalBound)
	}
	// Compute areas and volume independently using the unchanged two-level
	// column structure. The lower cap's regular polygon has n rim stations.
	n := len(alt.Vertices) / 2
	rho := math.Hypot(alt.Vertices[0].X-frame.Origin().X, alt.Vertices[0].Z-frame.Origin().Z)
	heldCap := float64(n) * rho * rho * math.Sin(2*math.Pi/float64(n)) / 2
	heldArea := 2*heldCap + float64(n)*2*rho*math.Sin(math.Pi/float64(n))*height
	trueArea := 2 * math.Pi * radius * (radius + height)
	if math.Abs(math.Pi*radius*radius*height-heldCap*height) > alt.VolSymDiff {
		t.Fatal("true volume is outside published bound")
	}
	if math.Abs(trueArea-heldArea) > alt.AreaSlack {
		t.Fatal("true area is outside published bound")
	}
	if alt.Planar() != nil {
		t.Fatal("alternate retained the old planar-chord proof")
	}
	bad := *m
	bad.Triangles = slices.Clone(m.Triangles)
	bad.Triangles[0][0] = len(m.Vertices)
	bad.VertexBound = nil
	if _, ok, err := radialInsetMesh(t.Context(), body, &bad, .01, proofbound.NewReplayTrialBudget()); ok || err != nil {
		t.Fatalf("invalid corner accepted=%v error=%v", ok, err)
	}
	if _, ok, err := radialInsetMesh(t.Context(), body, m, 0, proofbound.NewReplayTrialBudget()); ok || err != nil {
		t.Fatalf("zero reach accepted=%v error=%v", ok, err)
	}
	canceled, stop := context.WithCancel(t.Context())
	stop()
	if _, _, err := radialInsetMesh(canceled, body, m, .01, proofbound.NewReplayTrialBudget()); err != context.Canceled {
		t.Fatalf("cancellation=%v", err)
	}
}

type insetErrorMesher struct {
	OperandMesher
	err error
}

func (m insetErrorMesher) TessellateOperand(context.Context, *topo.Body, float64) (*topo.Mesh, bool, error) {
	return nil, false, m.err
}
