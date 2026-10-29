package loftmesh

import (
	"fmt"
	"math"
	"slices"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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

// chordedAllowMesh closes chordedAllowCase's four wall cells into a solid:
// two triangles per wall cell, then a two-triangle cap on each ring. It
// returns the triangles and the wall count.
func chordedAllowMesh() ([][3]int, int) {
	var tris [][3]int
	for j := range 4 {
		jn := (j + 1) % 4
		tris = append(tris, [3]int{j, jn, 4 + jn}, [3]int{j, 4 + jn, 4 + j})
	}
	walls := len(tris)
	tris = append(tris, [3]int{0, 2, 1}, [3]int{0, 3, 2}, [3]int{4, 5, 6}, [3]int{4, 6, 7})
	return tris, walls
}

// chordedCellMatched is each cell's own matched departure, as
// ComputeLoftChordedAllow composes it, and the largest of them.
func chordedCellMatched(p LoopPair, delta float64) ([]float64, float64) {
	cells := make([]float64, len(p.MatchedDelta))
	worst := 0.0
	for j, md := range p.MatchedDelta {
		cells[j] = ChordCellDeltaUpper(md, delta)
		worst = math.Max(worst, cells[j])
	}
	return cells, worst
}

// TestLoftVolumeResidualIsPerCellWallLegPlusSkirt pins Volume's chorded
// residual (agent-docs/loft-gear-bounds-design.md §2) on four wall cells with
// departures 0.1, 0.01, 0.001 and 0, the last one faceted:
//
//   - WallLeg sums each charged cell's own departure times its wall area and
//     is below the build-wide maximum departure times the summed wall area;
//   - SkirtLeg is matchedDelta·delta times the seam perimeter over EVERY
//     cell, the faceted one included: exactly 0 at delta == 0, positive above;
//   - Volume.Bound is the rounding, the vertex sweep, WallLeg and SkirtLeg
//     composed in that order, bit for bit, and the sweep is absent at
//     delta == 0.
//
// Shown to fail: charging every cell at the build-wide maximum fails the
// per-cell sum; dropping SkirtLeg from VolumeAllow fails the composition at
// delta > 0.
func TestLoftVolumeResidualIsPerCellWallLegPlusSkirt(t *testing.T) {
	t.Parallel()
	anchor := r3.NewVec(0.25, -0.5, 0.75)
	tris, walls := chordedAllowMesh()
	for _, delta := range []float64{0, 0.01} {
		t.Run(fmt.Sprintf("delta=%g", delta), func(t *testing.T) {
			pairs, vIdx, wIdx, verts := chordedAllowCase()
			p := &pairs[0]
			copy(p.MatchedDelta, []float64{0.1, 0.01, 0.001, 0})
			p.Faceted[3] = true
			cells, matchedDelta := chordedCellMatched(*p, delta)
			got := ComputeLoftChordedAllow(pairs, vIdx, wIdx, verts, anchor, matchedDelta, delta, false)

			var wallLeg, perimeter float64
			for j := range p.V {
				jn := (j + 1) % len(p.V)
				vLo, vHi := verts[vIdx[0][j]], verts[vIdx[0][jn]]
				wLo, wHi := verts[wIdx[0][j]], verts[wIdx[0][jn]]
				perimeter = proofbound.AbsSumUpper(perimeter,
					math.Max(p.ArcUpperV[j], proofbound.CellSpanUpper(vLo, vHi)),
					math.Max(p.ArcUpperW[j], proofbound.CellSpanUpper(wLo, wHi)),
				)
				if p.Faceted[j] && p.MatchedDelta[j] <= 0 {
					continue
				}
				cellWall := proofbound.CellChordCurveAreaUpper(vLo, vHi, wLo, wHi, p.ArcUpperV[j], p.ArcUpperW[j], cells[j])
				wallLeg = proofbound.AbsSumUpper(wallLeg, proofbound.ProductUpper(cells[j], cellWall))
			}
			require.Equal(t, math.Float64bits(wallLeg), math.Float64bits(got.WallLeg), "the wall leg is the per-cell sum")
			require.Less(t, got.WallLeg, proofbound.ProductUpper(matchedDelta, got.WallAreaUpper),
				"each cell is charged at its own departure, below the worst cell's")
			if delta == 0 {
				require.Zero(t, got.SkirtLeg, "a held seam on its cap plane has no skirt")
			} else {
				require.Positive(t, got.SkirtLeg)
				want := proofbound.ProductUpper(proofbound.ProductUpper(matchedDelta, delta), perimeter)
				require.Equal(t, math.Float64bits(want), math.Float64bits(got.SkirtLeg),
					"the skirt runs along every seam cell, the faceted one included")
			}

			m := NewMassAccumulator(anchor, delta, matchedDelta, matchedDelta)
			m.AddMesh(verts, tris, walls)
			m.Chorded = got
			vol := m.correctedVolume()
			value, bound := m.Volume(verts, tris)
			wantValue, _ := vol.Float64()
			require.Equal(t, wantValue, value)
			allow := 0.0
			if delta > 0 {
				allow = proofbound.SweptVolumeAllow(delta, proofbound.PerturbedAreaUpper(verts, tris, delta))
				require.Positive(t, allow)
			}
			allow = proofbound.AbsSumUpper(allow, got.WallLeg, got.SkirtLeg)
			want := proofbound.AbsSumUpper(proofarith.RationalFloatError(vol, value), allow)
			require.Equal(t, math.Float64bits(want), math.Float64bits(bound),
				"Volume.Bound is rounding + sweptLeg + wallLeg + skirtLeg (want %g, got %g)", want, bound)
		})
	}
}

// TestLoftAreaCapTubeIsPerCell pins Area's cap tube
// (agent-docs/loft-gear-bounds-design.md §4): Σ over charged cells and both
// sides of 2·cellMatched·arcLenUpper + π⁺·cellMatched². With every cell at
// the same departure it equals the build-wide maximum form
// proofbound.SectionDisplacementArea to rounding; with departures 0.1, 0.01,
// 0.001 and 0.05 it is below it.
//
// Shown to fail: charging every cell at the build-wide maximum fails the
// per-cell sum on the unequal row.
func TestLoftAreaCapTubeIsPerCell(t *testing.T) {
	t.Parallel()
	const delta = 0.01
	anchor := r3.NewVec(0.25, -0.5, 0.75)
	piUp := math.Nextafter(math.Pi, math.Inf(1))
	for _, tc := range []struct {
		name       string
		departures []float64
	}{
		{"uniform", []float64{0.1, 0.1, 0.1, 0.1}},
		{"unequal", []float64{0.1, 0.01, 0.001, 0.05}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			pairs, vIdx, wIdx, verts := chordedAllowCase()
			p := pairs[0]
			copy(p.MatchedDelta, tc.departures)
			cells, matchedDelta := chordedCellMatched(p, delta)
			got := ComputeLoftChordedAllow(pairs, vIdx, wIdx, verts, anchor, matchedDelta, delta, false)

			var tube, perimV, perimW float64
			for j := range p.V {
				perimV = proofbound.AbsSumUpper(perimV, p.ArcUpperV[j])
				perimW = proofbound.AbsSumUpper(perimW, p.ArcUpperW[j])
				joint := proofbound.ProductUpper(piUp, proofbound.ProductUpper(cells[j], cells[j]))
				twice := proofbound.ProductUpper(2, cells[j])
				tube = proofbound.AbsSumUpper(tube,
					proofbound.ProductUpper(twice, p.ArcUpperV[j]), joint,
					proofbound.ProductUpper(twice, p.ArcUpperW[j]), joint)
			}
			require.Equal(t, math.Float64bits(tube), math.Float64bits(got.CapAreaExcess), "the cap tube is the per-cell sum")
			maxForm := proofbound.AbsSumUpper(
				proofbound.SectionDisplacementArea(matchedDelta, len(p.V), perimV),
				proofbound.SectionDisplacementArea(matchedDelta, len(p.V), perimW),
			)
			if tc.name == "uniform" {
				require.InEpsilon(t, maxForm, got.CapAreaExcess, 1e-9, "equal departures read the maximum form")
				return
			}
			require.Less(t, got.CapAreaExcess, maxForm, "each cell is charged at its own departure, below the worst cell's")
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
