package meshbool

import (
	"context"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// refDepthBeyond is depthBeyond as it stood before the depth blocks: the hint
// reading, then one pass over every facet with the box prune. The block scan
// must return the same answer and leave the same hint for every input.
func refDepthBeyond(p proof.Xpt, other *BoolMesh, b float64, hint *int) bool {
	pf := p.Vec()
	if hint != nil && *hint >= 0 && *hint < len(other.Tris) {
		h := *hint
		up := math.Max(PointBoxDist(pf, other.Boxes[h]), PointTriDistance(pf, TriCorners(other, h)))
		if up*(1-1e-12) <= b {
			return false
		}
	}
	best := math.Inf(1)
	for i := range other.Tris {
		// The same distance prune CertifiedInteriorDepth applies, in the same
		// facet order, so the scan visits the same facets up to the stop.
		if PointBoxDist(pf, other.Boxes[i]) >= best {
			continue
		}
		best = math.Min(best, PointTriDistance(pf, TriCorners(other, i)))
		if best*(1-1e-12) <= b {
			if hint != nil {
				*hint = i
			}
			return false
		}
	}
	depth := 0.0
	if best > 0 && !proofbound.IsNonFinite(best) {
		depth = best*(1-1e-12) - PointRoundBound(p, pf)
	}
	// Negated rather than written as depth > b, so a NaN b answers as
	// DeepWitnessAt's original "depth <= b" test answered it.
	return !(depth <= b)
}

// depthMesh prepares verts and tris with zero vertex bounds.
func depthMesh(t testing.TB, verts []r3.Vec, tris [][3]int) *BoolMesh {
	t.Helper()
	bm, err := PrepContext(context.Background(), verts, tris, nil)
	require.NoError(t, err)
	bm.SetVertexBounds(make([]float64, len(verts)))
	return bm
}

// depthCubeGrid is a k×k×k grid of unit cubes with gaps, every cube turned by
// frame f when f is non-nil, its facets listed cube by cube.
func depthCubeGrid(t testing.TB, k int, f *r3.Frame) *BoolMesh {
	t.Helper()
	corners := [][3]int{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}}
	faces := [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
		{1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},
	}
	var verts []r3.Vec
	var tris [][3]int
	for x := range k {
		for y := range k {
			for z := range k {
				base := len(verts)
				for _, c := range corners {
					v := r3.NewVec(1.5*float64(x)+float64(c[0]), 1.5*float64(y)+float64(c[1]), 1.5*float64(z)+float64(c[2]))
					if f != nil {
						v = f.ToWorld(v)
					}
					verts = append(verts, v)
				}
				for _, fc := range faces {
					tris = append(tris, [3]int{fc[0] + base, fc[1] + base, fc[2] + base})
				}
			}
		}
	}
	return depthMesh(t, verts, tris)
}

// depthSlivers is a soup of thin, nearly collinear triangles in random order.
func depthSlivers(t testing.TB, rng *rand.Rand, n int) *BoolMesh {
	t.Helper()
	var verts []r3.Vec
	var tris [][3]int
	for i := range n {
		a := r3.NewVec(8*rng.Float64()-4, 8*rng.Float64()-4, 4*rng.Float64()-2)
		d := r3.NewVec(rng.Float64()-0.5, rng.Float64()-0.5, rng.Float64()-0.5)
		e := r3.NewVec(1e-9*(rng.Float64()-0.5), 1e-7*(rng.Float64()-0.5), 1e-8*rng.Float64())
		if i%5 == 0 {
			e = r3.NewVec(0, 0, 1e-15)
		}
		verts = append(verts, a, a.Add(d), a.Add(d.Scale(0.5)).Add(e))
		tris = append(tris, [3]int{3 * i, 3*i + 1, 3*i + 2})
	}
	return depthMesh(t, verts, tris)
}

// depthShuffled is m's facets in a random order, prepared afresh.
func depthShuffled(t testing.TB, rng *rand.Rand, m *BoolMesh) *BoolMesh {
	t.Helper()
	tris := append([][3]int(nil), m.Tris...)
	rng.Shuffle(len(tris), func(i, j int) { tris[i], tris[j] = tris[j], tris[i] })
	return depthMesh(t, append([]r3.Vec(nil), m.Verts...), tris)
}

// depthOddBoxes is m with a NaN, an inverted and an infinite facet box, its
// depth blocks rebuilt over them.
func depthOddBoxes(t testing.TB, m *BoolMesh) *BoolMesh {
	t.Helper()
	odd := depthMesh(t, append([]r3.Vec(nil), m.Verts...), append([][3]int(nil), m.Tris...))
	odd.Boxes[1][0].Y = math.NaN()
	odd.Boxes[6] = [2]r3.Vec{odd.Boxes[6][1], odd.Boxes[6][0]}
	odd.Boxes[9][1].X = math.Inf(1)
	odd.Boxes[len(odd.Boxes)-1][0].Z = math.Inf(-1)
	odd.setDepthBlocks()
	return odd
}

// depthSamplePoints are the facet sample points DeepWitnessInside asks about,
// over every stride-th facet of m.
func depthSamplePoints(m *BoolMesh, stride int) []proof.Xpt {
	var out []proof.Xpt
	for i := 0; i < len(m.Tris); i += stride {
		tri := m.Tris[i]
		out = append(out, facetSamplePoints(m.Xverts[tri[0]], m.Xverts[tri[1]], m.Xverts[tri[2]])...)
	}
	return out
}

// TestDepthBeyondBlocksMatchFacetScan compares depthBeyond against
// refDepthBeyond over cylinders, cube grids, turned cube grids, slivers,
// shuffled facets, odd boxes, blockless meshes and a near-touching pair. Every
// query runs with no hint, out-of-range hints, sampled valid hints and a hint
// carried from query to query, and must return the same answer and leave the
// same hint. Under the race detector it draws fewer queries, hints, bounds
// and pair sample points.
func TestDepthBeyondBlocksMatchFacetScan(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(11, 29))
	frame, err := r3.NewFrame(r3.NewVec(0.3, -0.2, 0.1), r3.NewVec(0.6, 0.64, 0.48), r3.NewVec(-0.8, 0.48, 0.36))
	require.NoError(t, err)
	queries, hintCount, pairStride := 10, 5, 4
	if raceDetector {
		queries, hintCount, pairStride = 3, 2, 16
	}
	cyl := depthCylinder(t, 64, r3.NewVec(0.375, -1.25, 0.5))
	near := depthCylinder(t, 64, r3.NewVec(6.0005, 0.001, 0.25))
	blockless := depthCylinder(t, 40, r3.Vec{})
	blockless.depthLevels = nil
	misfit := depthCylinder(t, 40, r3.Vec{})
	misfit.depthLevels = misfit.depthLevels[:len(misfit.depthLevels)-1]
	meshes := map[string]*BoolMesh{
		"cylinder8":   depthCylinder(t, 8, r3.Vec{}),
		"cylinder64":  cyl,
		"cubes":       depthCubeGrid(t, 3, nil),
		"turnedCubes": depthCubeGrid(t, 3, &frame),
		"slivers":     depthSlivers(t, rng, 60),
		"shuffled":    depthShuffled(t, rng, cyl),
		"oddBoxes":    depthOddBoxes(t, depthCylinder(t, 24, r3.Vec{})),
		"blockless":   blockless,
		"misfit":      misfit,
		"oneFacet":    depthMesh(t, []r3.Vec{{X: 1}, {Y: 1}, {Z: 1}}, [][3]int{{0, 1, 2}}),
	}
	bounds := append([]float64{1e-4, 2e-4, 1e-3, 2e-3, 0.0105, 0.05}, depthBounds...)
	if raceDetector {
		bounds = []float64{1e-4, 0.0105, math.Copysign(0, -1), 0.5, 3, math.Inf(1), math.NaN()}
	}
	far := []proof.Xpt{
		proof.XptOf(r3.NewVec(1e200, -1e200, 3)),
		proof.XptOf(r3.NewVec(-1e300, 0, 0)),
		proof.XptOf(r3.NewVec(5e-324, -5e-324, 0)),
	}
	pairQueries := depthSamplePoints(cyl, pairStride)
	compared, deep, shallow, moved := 0, 0, 0, 0
	for name, m := range meshes {
		qs := append(depthQueries(rng, m, queries), far...)
		hints := []int{-1, -7, len(m.Tris), len(m.Tris) + 3}
		for range hintCount {
			hints = append(hints, rng.IntN(len(m.Tris)))
		}
		for _, b := range bounds {
			carriedRef, carriedGot := -1, -1
			for qi, p := range qs {
				want := refDepthBeyond(p, m, b, nil)
				require.Equal(t, want, depthBeyond(p, m, b, nil), "%s query %d bound %g, no hint", name, qi, b)
				for _, h := range hints {
					hRef, hGot := h, h
					require.Equal(t, want, refDepthBeyond(p, m, b, &hRef))
					require.Equal(t, want, depthBeyond(p, m, b, &hGot), "%s query %d bound %g hint %d", name, qi, b, h)
					require.Equal(t, hRef, hGot, "%s query %d bound %g hint %d", name, qi, b, h)
					if hRef != h {
						moved++
					}
				}
				require.Equal(t, refDepthBeyond(p, m, b, &carriedRef), depthBeyond(p, m, b, &carriedGot), "%s query %d bound %g carried", name, qi, b)
				require.Equal(t, carriedRef, carriedGot, "%s query %d bound %g carried hint", name, qi, b)
				compared++
				if want {
					deep++
				} else {
					shallow++
				}
			}
		}
	}
	// The near-touching pair asks about one cylinder's facet sample points
	// against the other, as the witness search does.
	for _, b := range bounds {
		carriedRef, carriedGot := -1, -1
		for qi, p := range pairQueries {
			require.Equal(t, refDepthBeyond(p, near, b, &carriedRef), depthBeyond(p, near, b, &carriedGot), "near query %d bound %g", qi, b)
			require.Equal(t, carriedRef, carriedGot, "near query %d bound %g hint", qi, b)
		}
	}
	// The fixtures reach both answers and move the hint.
	t.Logf("compared %d queries: %d deep, %d shallow, %d hint moves", compared, deep, shallow, moved)
	require.Positive(t, deep)
	require.Positive(t, shallow)
	require.Positive(t, moved)
}

// TestDepthBlocksContainTheirBoxes checks every depth block against the boxes
// it covers: containment for ordered boxes, everywhereBox for a run holding an
// unordered one, and the shape depthBlocksFit expects.
func TestDepthBlocksContainTheirBoxes(t *testing.T) {
	t.Parallel()
	for name, m := range map[string]*BoolMesh{
		"cylinder": depthCylinder(t, 37, r3.Vec{}),
		"oddBoxes": depthOddBoxes(t, depthCylinder(t, 24, r3.Vec{})),
	} {
		require.True(t, m.depthBlocksFit(len(m.Tris)), name)
		require.Len(t, m.depthLevels[len(m.depthLevels)-1], 1, name)
		below := m.Boxes
		for li, level := range m.depthLevels {
			for j, block := range level {
				for _, box := range below[j*depthFanout : min((j+1)*depthFanout, len(below))] {
					if !boxOrdered(box) {
						require.Equal(t, everywhereBox, block, "%s level %d block %d", name, li, j)
						continue
					}
					require.True(t, block[0].X <= box[0].X && block[0].Y <= box[0].Y && block[0].Z <= box[0].Z &&
						block[1].X >= box[1].X && block[1].Y >= box[1].Y && block[1].Z >= box[1].Z,
						"%s level %d block %d", name, li, j)
				}
			}
			below = level
		}
	}
}

// depthDisc is a disc of radius r over z in [0, 2] with n wall sides: the
// wall facets in order around it, then each cap as a zigzag strip across the
// disc, so the cap facets are long and thin like an ear-clipped polygon's.
func depthDisc(t testing.TB, n int, r float64) *BoolMesh {
	t.Helper()
	var verts []r3.Vec
	for k := range n {
		th := 2 * math.Pi * float64(k) / float64(n)
		verts = append(verts, r3.NewVec(r*math.Cos(th), r*math.Sin(th), 0), r3.NewVec(r*math.Cos(th), r*math.Sin(th), 2))
	}
	var tris [][3]int
	for k := range n {
		a0, a1, b0, b1 := 2*k, 2*k+1, 2*((k+1)%n), 2*((k+1)%n)+1
		tris = append(tris, [3]int{a0, b0, b1}, [3]int{a0, b1, a1})
	}
	for _, top := range []int{0, 1} {
		lo, hi := 0, n-1
		for lo+1 < hi {
			tris = append(tris, [3]int{2*lo + top, 2*(lo+1) + top, 2*hi + top})
			lo++
			if lo+1 < hi {
				tris = append(tris, [3]int{2*lo + top, 2*(hi-1) + top, 2*hi + top})
				hi--
			}
		}
	}
	return depthMesh(t, verts, tris)
}

// depthLobe is the annular sector r in [r0, r1], angle in [-half, half], z in
// [0, 2], each of its six sides gridded into m×m quads.
func depthLobe(t testing.TB, r0, r1, half float64, m int) *BoolMesh {
	t.Helper()
	at := func(u, v, w float64) r3.Vec {
		r, th := r0+(r1-r0)*u, -half+2*half*v
		return r3.NewVec(r*math.Cos(th), r*math.Sin(th), 2*w)
	}
	sides := []func(a, b float64) r3.Vec{
		func(a, b float64) r3.Vec { return at(0, a, b) },
		func(a, b float64) r3.Vec { return at(1, a, b) },
		func(a, b float64) r3.Vec { return at(a, 0, b) },
		func(a, b float64) r3.Vec { return at(a, 1, b) },
		func(a, b float64) r3.Vec { return at(a, b, 0) },
		func(a, b float64) r3.Vec { return at(a, b, 1) },
	}
	var verts []r3.Vec
	var tris [][3]int
	for _, side := range sides {
		base := len(verts)
		for i := range m + 1 {
			for j := range m + 1 {
				verts = append(verts, side(float64(i)/float64(m), float64(j)/float64(m)))
			}
		}
		for i := range m {
			for j := range m {
				c := base + i*(m+1) + j
				tris = append(tris, [3]int{c, c + m + 1, c + m + 2}, [3]int{c, c + m + 2, c + 1})
			}
		}
	}
	return depthMesh(t, verts, tris)
}

// BenchmarkDepthBeyond runs the witness search's depth query as
// DeepWitnessInside asks it in the PatternJoin lobe tests: the facet sample
// points of a lobe resting on a disc's wall, against a 900-sided disc of 3596
// facets (the largest operand there holds 3668), with the hint carried from
// point to point and b at the tests' commonest slack. "facetScan" is
// refDepthBeyond.
func BenchmarkDepthBeyond(b *testing.B) {
	disc := depthDisc(b, 900, 3)
	lobe := depthLobe(b, 3, 3.65, 0.1, 6)
	qs := depthSamplePoints(lobe, 1)
	for name, run := range map[string]func(proof.Xpt, *BoolMesh, float64, *int) bool{
		"facetScan": refDepthBeyond,
		"blocks":    depthBeyond,
	} {
		b.Run(name, func(b *testing.B) {
			for b.Loop() {
				hint := -1
				for _, p := range qs {
					run(p, disc, 0.0105, &hint)
				}
			}
		})
	}
}
