package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// vertexBoundSketch fixes a closed polygon through pts on the XY plane and
// returns its one profile.
func vertexBoundSketch(t *testing.T, pts [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		points[i] = s.CreatePoint(p[0], p[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

// vertexBoundPolygonSketch fixes an n-gon of circumradius r about the origin.
func vertexBoundPolygonSketch(t *testing.T, n int, r float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	poly, err := s.CreatePolygon(0, 0, n, r)
	require.NoError(t, err)
	s.Fix(poly.Center)
	for _, v := range poly.Vertices {
		s.Fix(v)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

// vertexBoundRim is what the test recomputes for one rim vertex from the
// operands' own facet pairs: its exact point, the largest per-pair bound
// (δ(t_A) + δ(t_B)) / sin θ over the contact segments ending there, and the
// same figure divided by the pair-wide shallowest sine instead.
type vertexBoundRim struct {
	p        proof.Xpt
	own      float64
	pairWide float64
}

// vertexBoundFixture is one union's operands, prepared and classified the way
// evaluateBoolean prepares and classifies them, and its result payload.
type vertexBoundFixture struct {
	ma, mb  *Mesh
	rims    map[string]*vertexBoundRim
	sinMin  float64
	result  facetedPayload
	heldIdx map[r3.Vec]int
	operand map[r3.Vec]float64
}

// unionVertexBoundFixture tessellates a and b at the pair's chord tolerance,
// recomputes every rim vertex's per-pair bound from the exact contact of each
// facet pair, and unions them.
func unionVertexBoundFixture(t *testing.T, a, b *Body) vertexBoundFixture {
	t.Helper()
	tol, _, err := pairChordTolerance(t.Context(), a, b)
	require.NoError(t, err)
	ma, err := tessellateContext(t.Context(), a, units.Millimeters(tol), VerifyAll)
	require.NoError(t, err)
	mb, err := tessellateContext(t.Context(), b, units.Millimeters(tol), VerifyAll)
	require.NoError(t, err)
	bmA, err := prepBoolMeshContext(t.Context(), ma, make([]int, len(ma.triangles)))
	require.NoError(t, err)
	bmB, err := prepBoolMeshContext(t.Context(), mb, make([]int, len(mb.triangles)))
	require.NoError(t, err)
	betaA, err := ma.vertexBounds()
	require.NoError(t, err)
	betaB, err := mb.vertexBounds()
	require.NoError(t, err)
	facet := func(beta []float64, tri [3]int) float64 { return max(beta[tri[0]], beta[tri[1]], beta[tri[2]]) }

	type segment struct {
		p0, p1 proof.Xpt
		sum    float64
		sin2   *big.Rat
	}
	var segs []segment
	var minSin2 *big.Rat
	for i := range bmA.Tris {
		for j := range bmB.Tris {
			if !meshbool.BoxesOverlap(bmA.Boxes[i], bmB.Boxes[j]) {
				continue
			}
			c, err := meshbool.TriTriClassify(meshbool.TriCorners(bmA, i), meshbool.TriCorners(bmB, j),
				meshbool.XtriCorners(bmA, i), meshbool.XtriCorners(bmB, j), bmA.Norms[i], bmB.Norms[j])
			require.NoError(t, err)
			if c.Sin2 != nil && (minSin2 == nil || c.Sin2.Cmp(minSin2) < 0) {
				minSin2 = c.Sin2
			}
			if c.Kind != meshbool.ContactSegment {
				continue
			}
			segs = append(segs, segment{
				p0:   c.P0,
				p1:   c.P1,
				sum:  proofbound.AbsSumUpper(facet(betaA, ma.triangles[i]), facet(betaB, mb.triangles[j])),
				sin2: c.Sin2,
			})
		}
	}
	require.NotEmpty(t, segs)
	sinMin := meshbool.SinLowerBound(minSin2)
	rims := map[string]*vertexBoundRim{}
	for _, s := range segs {
		own := proofbound.DivUpper(s.sum, meshbool.SinLowerBound(s.sin2))
		pairWide := proofbound.DivUpper(s.sum, sinMin)
		for _, p := range []proof.Xpt{s.p0, s.p1} {
			key := p.Key()
			r, ok := rims[key]
			if !ok {
				r = &vertexBoundRim{p: p}
				rims[key] = r
			}
			r.own, r.pairWide = max(r.own, own), max(r.pairWide, pairWide)
		}
	}

	operand := map[r3.Vec]float64{}
	for i, v := range ma.vertices {
		operand[v] = betaA[i]
	}
	for i, v := range mb.vertices {
		operand[v] = max(operand[v], betaB[i])
	}
	u, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	result, ok := u.payload.(facetedPayload)
	require.True(t, ok)
	require.Len(t, result.vertexBound, len(result.verts))
	held := map[r3.Vec]int{}
	for i, v := range result.verts {
		held[v] = i
	}
	return vertexBoundFixture{ma: ma, mb: mb, rims: rims, sinMin: sinMin, result: result, heldIdx: held, operand: operand}
}

// held finds the result vertex that stands for the exact point p: its own
// nearest float, or, failing that, the held vertex nearest it per coordinate.
func (f vertexBoundFixture) held(t *testing.T, p proof.Xpt) int {
	t.Helper()
	if i, ok := f.heldIdx[p.Vec()]; ok {
		return i
	}
	best, bestGap := -1, (*big.Rat)(nil)
	for i, v := range f.result.verts {
		if gap := meshbool.CoordDistance(p, v); bestGap == nil || gap.Cmp(bestGap) < 0 {
			best, bestGap = i, gap
		}
	}
	require.GreaterOrEqual(t, best, 0)
	return best
}

// weldOf is the per-vertex weld docs/faceted-vertex-bounds-design.md §3.4
// charges: p's largest per-coordinate distance to the float that holds it,
// widened to a 3D radius and rounded up, and zero when the float is exact.
func weldOf(p proof.Xpt, v r3.Vec) float64 {
	gap := meshbool.CoordDistance(p, v)
	if gap.Sign() == 0 {
		return 0
	}
	g, _ := gap.Float64()
	return proofbound.Radius3D(proofbound.ProvenUpRound(g))
}

// TestBooleanVertexBoundsComposePerPair is docs/faceted-vertex-bounds-design.md
// §3 over two tapered 16-gon mitred branches whose leaning spans cross in an X
// (apitest's TestSweepMitredTaperedBranchesCross). Every vertex of both
// operands carries its own rounding gap. A surviving operand vertex keeps that
// gap to the bit; every rim vertex's bound is its own facet pair's
// (δ(t_A) + δ(t_B)) / sin θ plus its own weld, recomputed here from the
// operands' facets; and the result's meshBound is at most the global
// composition, (Δ_A + Δ_B) / sin θ_min plus the largest weld. Some rim's own
// crossing is steeper than the pair's shallowest, so its bound sits strictly
// under the pair-wide figure.
func TestBooleanVertexBoundsComposePerPair(t *testing.T) {
	t.Parallel()
	doc := New()
	s, profile := vertexBoundPolygonSketch(t, 16, 1)
	path, err := NewPath(r3.NewVec(0, 0, 0), LineTo{End: r3.NewVec(0, 0, 2)}, LineTo{End: r3.NewVec(8, 0, 10)})
	require.NoError(t, err)
	scale := WithSectionScale(units.Scalar(0.9), units.Scalar(0.6))
	a, err := doc.Sweep(t.Context(), s, profile, path, WithMitredJoins(), scale)
	require.NoError(t, err)
	b, err := doc.Sweep(t.Context(), s, profile, path, WithMitredJoins(), scale)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(4, -4, 0), r3.NewVec(0, 1, 0), r3.NewVec(-1, 0, 0))
	require.NoError(t, err)
	turn, err := r3.FromFrame(frame)
	require.NoError(t, err)
	b, err = b.Placed(t.Context(), turn)
	require.NoError(t, err)
	f := unionVertexBoundFixture(t, a, b)
	require.Positive(t, f.ma.bound)
	require.Positive(t, f.mb.bound)

	// Two rim points closer than an ulp weld into one held vertex, which then
	// takes the larger of their two sums.
	want := map[int]float64{}
	maxWeld, steeper := 0.0, 0
	for _, r := range f.rims {
		p := r.p
		i := f.held(t, p)
		pre := r.own
		if beta, ok := f.operand[p.Vec()]; ok && meshbool.CoordDistance(p, p.Vec()).Sign() == 0 {
			pre = max(pre, beta)
		}
		weld := weldOf(p, f.result.verts[i])
		maxWeld = max(maxWeld, weld)
		if weld > 0 {
			pre = proofbound.AbsSumUpper(pre, weld)
		}
		want[i] = max(want[i], pre)
		if r.own < r.pairWide {
			steeper++
		}
	}
	for i, w := range want {
		require.Equal(t, w, f.result.vertexBound[i], `rim vertex %v carries its own pair's bound plus its own weld`, f.result.verts[i])
	}
	require.Positive(t, steeper, `some rim crosses more steeply than the pair's shallowest contact`)

	kept := 0
	for v, beta := range f.operand {
		i, ok := f.heldIdx[v]
		if !ok {
			continue
		}
		if _, rim := f.rims[proof.XptOf(v).Key()]; rim {
			continue
		}
		require.Equal(t, beta, f.result.vertexBound[i], `operand vertex %v keeps its own bound`, v)
		kept++
	}
	require.Positive(t, kept)

	today := proofbound.AbsSumUpper(proofbound.UpRound((f.ma.bound+f.mb.bound)/f.sinMin), maxWeld)
	require.LessOrEqual(t, f.result.meshBound, today)
	require.Equal(t, facetBoundMax(f.result.tris, f.result.vertexBound), f.result.meshBound)
	t.Logf("meshBound %.3g, global composition %.3g, operand deltas %.3g / %.3g", f.result.meshBound, today, f.ma.bound, f.mb.bound)
}

// TestBooleanVertexBoundsChargeEachRimItsWeld is docs/tessellation-design.md
// §14's "rational intersection vertices round inexactly" fixture, per vertex:
// two exactly held mitred prisms (every operand vertex is its own float, so
// every δ is zero) whose slanted walls cross the other's at points no float
// holds. Each such rim vertex carries no trim amplification, so its whole
// bound is its own weld, and that bound covers the exact distance from the
// rim point to the float the result holds for it.
func TestBooleanVertexBoundsChargeEachRimItsWeld(t *testing.T) {
	t.Parallel()
	doc := New()
	sa, pa := vertexBoundSketch(t, [][2]float64{{0, 0}, {2, 0}, {2, 2}, {0, 2}})
	path, err := NewPath(r3.NewVec(0, 0, 0), LineTo{End: r3.NewVec(0, 0, 10)})
	require.NoError(t, err)
	a, err := doc.Sweep(t.Context(), sa, pa, path, WithMitredJoins())
	require.NoError(t, err)
	sb, pb := vertexBoundSketch(t, [][2]float64{{-1, 0}, {5, 1}, {-1, 4}})
	b, err := doc.Sweep(t.Context(), sb, pb, path, WithMitredJoins())
	require.NoError(t, err)
	down, err := r3.Translation(r3.NewVec(0, 0, -2))
	require.NoError(t, err)
	b, err = b.Placed(t.Context(), down)
	require.NoError(t, err)
	f := unionVertexBoundFixture(t, a, b)
	require.Zero(t, f.ma.bound)
	require.Zero(t, f.mb.bound)

	inexact := 0
	for _, r := range f.rims {
		p := r.p
		require.Zero(t, r.own)
		i := f.held(t, p)
		v := f.result.verts[i]
		px, py, pz := proof.XhpRat(proof.Xhp(p))
		d2 := new(big.Rat)
		for _, pair := range [][2]*big.Rat{{px, new(big.Rat).SetFloat64(v.X)}, {py, new(big.Rat).SetFloat64(v.Y)}, {pz, new(big.Rat).SetFloat64(v.Z)}} {
			d := new(big.Rat).Sub(pair[0], pair[1])
			d2.Add(d2, d.Mul(d, d))
		}
		if d2.Sign() == 0 {
			continue
		}
		inexact++
		beta := new(big.Rat).SetFloat64(f.result.vertexBound[i])
		require.GreaterOrEqual(t, new(big.Rat).Mul(beta, beta).Cmp(d2), 0, `rim vertex %v's bound covers its rounding`, v)
	}
	require.Positive(t, inexact, `some rim vertex rounds inexactly`)
	require.Positive(t, f.result.meshBound)
}

// TestHeldGateReadsFacetsWithinThePrePassSlack is docs/faceted-vertex-bounds-
// design.md §5's gate on the pre-pass's half: two tetrahedra face each other
// across a 1e-3 mm gap, so no facet meets the other operand, and every vertex
// of the first carries a 1e-2 mm bound. Within the 1.5e-2 mm slack a gated
// first operand whose tolerance is finer than that bound is refused by the
// gate, quoting the bound and the tolerance; a tolerance above it, or an
// ungated operand, leaves the near-miss to the pre-pass's own refusal.
// Shown to fail: with facesNearMiss's gate calls removed, the gated case
// returned the near-miss verdict and no CoarseHeldContactError.
func TestHeldGateReadsFacetsWithinThePrePassSlack(t *testing.T) {
	t.Parallel()
	const gap, beta, slack = 1e-3, 1e-2, 1.5e-2
	tetra := func(mirror, bound float64) *meshbool.BoolMesh {
		verts := []r3.Vec{
			r3.NewVec(0, 0, 0), r3.NewVec(mirror, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1),
		}
		tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
		if mirror < 0 {
			for i := range verts {
				verts[i].X -= gap
			}
			for i, tr := range tris {
				tris[i] = [3]int{tr[0], tr[2], tr[1]}
			}
		}
		m := &Mesh{vertices: verts, triangles: tris, source: make([]*Face, len(tris))}
		m.setVertexBounds([]float64{bound, bound, bound, bound})
		bm, err := prepBoolMeshContext(t.Context(), m, make([]int, len(tris)))
		require.NoError(t, err)
		return bm
	}
	all := []int{0, 1, 2, 3}
	testcases := []struct {
		name string
		gate float64
	}{
		{name: "a tolerance finer than the bound refuses", gate: 5e-3},
		{name: "a tolerance above the bound passes the gate", gate: 2e-2},
		{name: "an ungated operand passes the gate", gate: 0},
	}
	for _, tc := range testcases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			bmA, bmB := tetra(1, beta), tetra(-1, 0)
			bmA.Gate = tc.gate
			near, err := meshbool.FacesNearMiss(t.Context(), bmA, all, bmB, all, slack, meshbool.NewContactMemo(bmA, bmB))
			if tc.gate > 0 && tc.gate < beta {
				var coarse *meshbool.CoarseHeldContactError
				require.ErrorAs(t, err, &coarse)
				require.Equal(t, meshbool.CoarseHeldContactError{Operand: 0, Bound: beta, Tol: tc.gate}, *coarse)
				return
			}
			require.NoError(t, err)
			require.True(t, near, "the facets come within the slack without meeting, which the pre-pass refuses")
		})
	}
}

// TestHeldGateCertifiesEachUnionsRimCeiling is docs/faceted-vertex-bounds-
// design.md §5's certificate over twenty unions of §6's tree: before each
// union it prepares both operands the way evaluateBoolean does, at their own
// request tolerances, and walks every facet pair the classification reports
// as meeting. Each meeting facet holds a bound within the pair's chord
// tolerance, and every contact segment's rim bound is at most 2·tol/sin θ of
// its own facet pair, the ceiling a rim of two freshly chorded operands has.
// Every union then succeeds.
func TestHeldGateCertifiesEachUnionsRimCeiling(t *testing.T) {
	t.Parallel()
	unions := 0
	_, body := vertexBoundTree(t, 20, func(tree, branch *Body) {
		ctx := t.Context()
		tol, _, err := pairChordTolerance(ctx, tree, branch)
		require.NoError(t, err)
		ma, restatingA, err := tessellateBooleanOperand(ctx, tree, tol)
		require.NoError(t, err)
		mb, restatingB, err := tessellateBooleanOperand(ctx, branch, tol)
		require.NoError(t, err)
		require.True(t, restatingA && restatingB, `the tree and its mitred branch both restate held meshes`)
		bmA, err := prepBoolMeshContext(ctx, ma, make([]int, len(ma.triangles)))
		require.NoError(t, err)
		bmB, err := prepBoolMeshContext(ctx, mb, make([]int, len(mb.triangles)))
		require.NoError(t, err)
		segments := 0
		contacts := meshbool.NewContactBatchExecutor(ctx, bmA, bmB, meshbool.NewContactMemo(bmA, bmB), meshbool.ContactWorkers(ctx),
			func(pair meshbool.ContactPair, c meshbool.TriContact) error {
				if c.Kind == meshbool.ContactNone {
					return nil
				}
				i, j := pair.I, pair.J
				require.LessOrEqual(t, bmA.FacetBound[i], tol, `union %d: a meeting tree facet holds within the pair tolerance`, unions+1)
				require.LessOrEqual(t, bmB.FacetBound[j], tol, `union %d: a meeting branch facet holds within the pair tolerance`, unions+1)
				if c.Kind != meshbool.ContactSegment {
					return nil
				}
				segments++
				ceiling := proofbound.DivUpper(proofbound.ProductUpper(2, tol), meshbool.SinLowerBound(c.Sin2))
				require.LessOrEqual(t, meshbool.RimBound(bmA.FacetBound[i], bmB.FacetBound[j], c.Sin2), ceiling,
					`union %d: a rim stays under 2·tol/sin θ of its own pair`, unions+1)
				return nil
			})
		for i := range bmA.Tris {
			for j := range bmB.Tris {
				if meshbool.BoxesOverlap(bmA.Boxes[i], bmB.Boxes[j]) {
					require.NoError(t, contacts.Add(i, j))
				}
			}
		}
		require.NoError(t, contacts.Done())
		require.Positive(t, segments, `union %d makes rims`, unions+1)
		unions++
	})
	require.Equal(t, 20, unions)
	_, ok := body.payload.(facetedPayload)
	require.True(t, ok)
}
