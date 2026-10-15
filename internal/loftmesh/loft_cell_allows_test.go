package loftmesh

import (
	"math"
	"slices"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func chordedAllowCase() ([]LoopPair, [][]int, [][]int, []r3.Vec) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0),
		r3.NewVec(2, 2, 0), r3.NewVec(0, 2, 0),
		r3.NewVec(0, 0, 3), r3.NewVec(1.8, 0.1, 3.2),
		r3.NewVec(1.9, 1.7, 2.9), r3.NewVec(0.1, 1.8, 3.1),
	}
	p := LoopPair{
		V: make([]sectionrecord.Point2, 4), W: make([]sectionrecord.Point2, 4),
		ArcUpperV: []float64{3, 3, 3, 3}, ArcUpperW: []float64{3, 3, 3, 3},
		MatchedDelta:   []float64{0.1, 0.1, 0.1, 0.1},
		TangentEnergyV: []float64{1, 1, 1, 1}, TangentEnergyW: []float64{1, 1, 1, 1},
		Faceted: make([]bool, 4),
	}
	return []LoopPair{p}, [][]int{{0, 1, 2, 3}}, [][]int{{4, 5, 6, 7}}, verts
}

func cloneChordedPairs(pairs []LoopPair) []LoopPair {
	out := slices.Clone(pairs)
	for i := range out {
		out[i].V = slices.Clone(out[i].V)
		out[i].W = slices.Clone(out[i].W)
		out[i].ArcUpperV = slices.Clone(out[i].ArcUpperV)
		out[i].ArcUpperW = slices.Clone(out[i].ArcUpperW)
		out[i].MatchedDelta = slices.Clone(out[i].MatchedDelta)
		out[i].TangentEnergyV = slices.Clone(out[i].TangentEnergyV)
		out[i].TangentEnergyW = slices.Clone(out[i].TangentEnergyW)
		out[i].Faceted = slices.Clone(out[i].Faceted)
	}
	return out
}

func requireChordedAllowExact(t *testing.T, want, got LoftChordedAllow) {
	t.Helper()
	for _, field := range []struct {
		name      string
		want, got float64
	}{
		{"wall area", want.WallAreaUpper, got.WallAreaUpper},
		{"wall leg", want.WallLeg, got.WallLeg},
		{"skirt", want.SkirtLeg, got.SkirtLeg},
		{"twist volume upper", want.TwistVolumeUpper, got.TwistVolumeUpper},
		{"twist offset", want.MaxTwistOffsetUpper, got.MaxTwistOffsetUpper},
		{"area correction", want.AreaCorrection, got.AreaCorrection},
		{"area correction bound", want.AreaCorrectionBound, got.AreaCorrectionBound},
		{"bilinear area bound", want.BilinearAreaBound, got.BilinearAreaBound},
		{"area excess", want.AreaExcess, got.AreaExcess},
		{"twist area", want.TwistAreaAllow, got.TwistAreaAllow},
		{"cap area", want.CapAreaExcess, got.CapAreaExcess},
	} {
		require.Equal(t, math.Float64bits(field.want), math.Float64bits(field.got), field.name)
	}
	require.Zero(t, want.TwistVolumeCorrection.Cmp(got.TwistVolumeCorrection))
	for axis := range want.TwistMomentCorrection {
		require.Zero(t, want.TwistMomentCorrection[axis].Cmp(got.TwistMomentCorrection[axis]), "moment axis %d", axis)
	}
}

// TestComputeLoftChordedAllowMatchesSeparateBounds compares every published
// bit and exact correction against the pre-sharing wall-cell implementation.
func TestComputeLoftChordedAllowMatchesSeparateBounds(t *testing.T) {
	for _, tc := range []struct {
		name     string
		edit     func([]LoopPair, []r3.Vec)
		reversed bool
	}{
		{name: "curved cells"},
		{name: "reversed winding", reversed: true},
		{name: "mixed faceted cells", edit: func(p []LoopPair, _ []r3.Vec) {
			p[0].Faceted[0], p[0].MatchedDelta[0] = true, 0
		}},
		{name: "all faceted", edit: func(p []LoopPair, _ []r3.Vec) {
			for i := range p[0].Faceted {
				p[0].Faceted[i], p[0].MatchedDelta[i] = true, 0
			}
		}},
		{name: "degenerate corners", edit: func(_ []LoopPair, v []r3.Vec) {
			for i := range v {
				v[i] = r3.NewVec(1, 2, 3)
			}
		}},
		{name: "signed zero corners", edit: func(_ []LoopPair, v []r3.Vec) {
			v[0].X = math.Copysign(0, -1)
			v[4].Y = math.Copysign(0, -1)
		}},
		{name: "broken arc claim", edit: func(p []LoopPair, _ []r3.Vec) {
			p[0].ArcUpperV[0] = 0.01
		}},
		{name: "unstateable matched claim on faceted cell", edit: func(p []LoopPair, _ []r3.Vec) {
			p[0].Faceted[0], p[0].MatchedDelta[0] = true, math.Inf(1)
		}},
		{name: "nonfinite scalar claim", edit: func(p []LoopPair, _ []r3.Vec) {
			p[0].TangentEnergyV[0] = math.Inf(1)
		}},
		{name: "cancelling twist", edit: func(p []LoopPair, v []r3.Vec) {
			v[0] = r3.Vec{}
			v[1] = r3.NewVec(0.1, 0.2, 0.3)
			v[4] = r3.NewVec(0.7, 1.1, 1.3)
			v[5] = v[1].Add(v[4])
			for i := range p[0].ArcUpperV {
				p[0].ArcUpperV[i], p[0].ArcUpperW[i] = 10, 10
			}
		}},
		{name: "long dyadic denominators", edit: func(p []LoopPair, v []r3.Vec) {
			v[1] = r3.NewVec(math.Nextafter(1, 2), math.Ldexp(1, -400), 0)
			v[5] = r3.NewVec(1, math.Ldexp(1, -400), 3)
			for i := range p[0].ArcUpperV {
				p[0].ArcUpperV[i], p[0].ArcUpperW[i] = 10, 10
			}
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			pairs, vIdx, wIdx, verts := chordedAllowCase()
			if tc.edit != nil {
				tc.edit(pairs, verts)
			}
			originalPairs, originalVerts := cloneChordedPairs(pairs), slices.Clone(verts)
			anchor := r3.NewVec(0.25, -0.5, 0.75)
			want := referenceComputeLoftChordedAllow(pairs, vIdx, wIdx, verts, anchor, 0.25, 0.01, tc.reversed)
			got := ComputeLoftChordedAllow(pairs, vIdx, wIdx, verts, anchor, 0.25, 0.01, tc.reversed)
			requireChordedAllowExact(t, want, got)
			var wallAreas []WallArea
			for i, p := range pairs {
				for j := range p.V {
					jn := (j + 1) % len(p.V)
					vLo := proofarith.XptOf(verts[vIdx[i][j]])
					vHi := proofarith.XptOf(verts[vIdx[i][jn]])
					wLo := proofarith.XptOf(verts[wIdx[i][j]])
					wHi := proofarith.XptOf(verts[wIdx[i][jn]])
					lo0, hi0 := WallTriangleArea(proofarith.Xsub(vHi, vLo), proofarith.Xsub(wHi, vLo))
					lo1, hi1 := WallTriangleArea(proofarith.Xsub(wHi, vLo), proofarith.Xsub(wLo, vLo))
					wallAreas = append(wallAreas, WallArea{Lo: lo0, Hi: hi0}, WallArea{Lo: lo1, Hi: hi1})
				}
			}
			cached := ComputeLoftChordedAllowWithWallAreas(pairs, vIdx, wIdx, verts, anchor, 0.25, 0.01, tc.reversed, wallAreas)
			requireChordedAllowExact(t, want, cached)
			require.Equal(t, originalPairs, pairs)
			require.Equal(t, originalVerts, verts)
		})
	}
}

func TestComputeLoftChordedAllowNaNArcMatchesSeparateBounds(t *testing.T) {
	pairs, vIdx, wIdx, verts := chordedAllowCase()
	pairs[0].ArcUpperV[0] = math.NaN()
	anchor := r3.NewVec(0.25, -0.5, 0.75)
	want := referenceComputeLoftChordedAllow(pairs, vIdx, wIdx, verts, anchor, 0.25, 0.01, false)
	got := ComputeLoftChordedAllow(pairs, vIdx, wIdx, verts, anchor, 0.25, 0.01, false)
	requireChordedAllowExact(t, want, got)
}

func TestComputeLoftChordedAllowNonfiniteCornerKeepsSeamPanic(t *testing.T) {
	const finiteLiftPanic = "decad: exact Dyadic lift requires a finite float"
	readPanic := func(run func()) (caught any) {
		defer func() { caught = recover() }()
		run()
		return nil
	}
	for _, faceted := range []bool{false, true} {
		name := "charged"
		if faceted {
			name = "faceted uncharged"
		}
		t.Run(name, func(t *testing.T) {
			pairs, vIdx, wIdx, verts := chordedAllowCase()
			verts[0].X = math.Inf(1)
			if faceted {
				pairs[0].Faceted[0], pairs[0].MatchedDelta[0] = true, 0
			}
			anchor := r3.NewVec(0.25, -0.5, 0.75)
			for _, path := range []struct {
				name string
				run  func()
			}{
				{"separate bounds", func() {
					referenceComputeLoftChordedAllow(pairs, vIdx, wIdx, verts, anchor, 0.25, 0.01, false)
				}},
				{"shared bounds", func() {
					ComputeLoftChordedAllow(pairs, vIdx, wIdx, verts, anchor, 0.25, 0.01, false)
				}},
			} {
				caught := readPanic(path.run)
				message, ok := caught.(string)
				if !ok {
					t.Fatalf("%s: expected a string panic, got %T", path.name, caught)
				}
				require.Equal(t, finiteLiftPanic, message, path.name)
			}
		})
	}
}
