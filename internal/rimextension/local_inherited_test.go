package rimextension

// CI: isolated-tests

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestLocalInheritedBoundIncludesCrossingAndSubdividedFacets(t *testing.T) {
	w := SourceWindow{Side0: r3.Vec{}, Side1: r3.Vec{X: 1, Z: 1}, XMin: 0, XMax: 1, YMin: 0, YMax: 1, OffsetMin: 0, OffsetMax: 1}
	var old facetedrecord.Payload
	current := map[[3]int]bool{}
	add := func(p r3.Vec, r float64, kept bool) {
		i := len(old.Verts)
		t := [3]int{i, i + 1, i + 2}
		old.Tris = append(old.Tris, t)
		// A signed shoulder chart maps world -X to transverse Z.
		for _, v := range []r3.Vec{p, {X: p.X, Y: p.Y + .125, Z: p.Z}, {X: p.X + .125, Y: p.Y, Z: p.Z + .125}} {
			old.Verts = append(old.Verts, r3.NewVec(-v.Z, v.Y, -v.X))
			old.VertexBound = append(old.VertexBound, r)
		}
		current[t] = kept
	}
	check := func(want float64) {
		t.Helper()
		got, ok, e := localInheritedBound(context.Background(), old, current, w, 1, proofbound.NewStrictReplayBudget())
		if !ok || e != nil || got != want {
			t.Fatalf("bound %g want %g accepted %t error %v", got, want, ok, e)
		}
	}
	add(r3.Vec{X: .25, Y: .5, Z: .375}, .125, true)
	add(r3.Vec{X: .25, Y: .5, Z: 30}, 4, true)
	check(.125)
	// Even though its Y extent lies outside, the error ball touches Y=1.
	add(r3.Vec{X: .25, Y: 1.5, Z: .5}, .5, true)
	check(.5)
	// A split propagates into a separated facet. Keep that parent's bound.
	add(r3.Vec{X: 9, Y: .5, Z: 8}, .75, false)
	check(.75)
}
