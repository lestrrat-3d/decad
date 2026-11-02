package prismrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

func TestPrismSourceWitnessRetainsStrictBuildSeal(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	if err != nil {
		t.Fatal(err)
	}
	corners := []sectionrecord.Point2{{}, {U: 2}, {U: 2, V: 1}, {V: 1}}
	segments := make([]sectionrecord.CurveSegment, 4)
	for i, corner := range corners {
		segments[i] = sectionrecord.LineSeg{Start: corner, End: corners[(i+1)%4], TEnd: 1}
	}
	pp := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z1: 2,
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}}}
	var witness *prismrecord.PrismSourceWitness
	ctx := prismrecord.WithPrismSourceCapture(t.Context(), func(w *prismrecord.PrismSourceWitness) {
		witness = w
	})
	body, err := prismrecord.Build(ctx, topo.Env{Doc: &topo.Document{}, Ref: 5,
		Alloc: topotest.Alloc{}}, pp, freeform.NewFreeformWork())
	if err != nil || body == nil || witness == nil || witness.Body != body ||
		!body.HasStrictPrismBuildSeal() || len(witness.Walls) != 4 {
		t.Fatalf("body=%t witness=%t walls=%d error=%v",
			body != nil, witness != nil, func() int {
				if witness != nil {
					return len(witness.Walls)
				}
				return 0
			}(), err)
	}
	faces := topobuild.Faces(body)
	for _, face := range append(append([]*topo.Face{}, witness.Walls...), witness.CapStart, witness.CapEnd) {
		found := false
		for _, built := range faces {
			found = found || face == built
		}
		if face == nil || !found {
			t.Fatal("source witness names a missing built face")
		}
	}
}
