package booleaneval

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func TestStationPrismEmbedding(t *testing.T) {
	m := topo.Mesh{Vertices: []r3.Vec{{}, {Y: 1}, {X: 1}, {X: 1, Y: 1}, {X: 1, Z: 1}, {X: 1, Y: 1, Z: 1}, {Z: 1}, {Y: 1, Z: 1}}}
	m.Triangles = append(make([][3]int, 8), [3]int{0, 2, 4}, [3]int{0, 4, 6}, [3]int{1, 5, 3}, [3]int{1, 7, 5})
	check := func(want bool) {
		t.Helper()
		got, err := stationEmbedded(t.Context(), &m, proofbound.NewStrictReplayBudget())
		if err != nil || got != want {
			t.Fatalf("embedded=%v, want %v, err=%v", got, want, err)
		}
	}
	check(true)
	m.Triangles[9] = [3]int{0, 6, 4}
	check(false)
	m.Triangles[9] = [3]int{0, 4, 6}
	m.Vertices[2], m.Vertices[4] = m.Vertices[4], m.Vertices[2]
	m.Vertices[3], m.Vertices[5] = m.Vertices[5], m.Vertices[3]
	check(false)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := stationEmbedded(ctx, &m, proofbound.NewStrictReplayBudget())
	if !errors.Is(err, context.Canceled) {
		t.Fatalf("canceled embedding returned %v", err)
	}
}

func TestStationRootsRefuseAmbiguousRadii(t *testing.T) {
	for _, x := range []float64{1.5, math.Nextafter(1.5, 2), 1.45} {
		arc := sectionrecord.ArcSeg{Start: sectionrecord.Point2{U: x, V: 2}, End: sectionrecord.Point2{U: x, V: -2}, TEnd: 1}
		_, ok, err := stationRoots(t.Context(), nil, nil, prismrecord.Payload{}, prismrecord.Payload{}, 0, arc, 2.5, proofbound.NewStrictReplayBudget())
		if err != nil || ok {
			t.Fatalf("admitted radius through (%g,2): ok=%v err=%v", x, ok, err)
		}
	}
}

func TestStationMesherRejectsUnprovenOperand(t *testing.T) {
	a, b := new(topo.Body), new(topo.Body)
	ma, mb := new(topo.Mesh), new(topo.Mesh)
	m := stationMesher{plate: stationOperand{a, ma, .01}, disk: stationOperand{b, mb, .01}}
	if !m.matches(a, b, ma, mb) || !m.matches(b, a, mb, ma) {
		t.Fatal("lost native mesh binding")
	}
	if m.matches(a, b, new(topo.Mesh), mb) || m.matches(a, b, mb, ma) || m.matches(new(topo.Body), b, ma, mb) {
		t.Fatal("accepted unrelated mesh or source body")
	}
}
