package decad

import (
	"context"
	"math"
	"math/big"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type internalBooleanBuildCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	target          string
	entered         bool
}

func (c *internalBooleanBuildCancelContext) Err() error {
	pcs := make([]uintptr, 32)
	frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
	inBuild, inTarget := false, false
	for {
		frame, more := frames.Next()
		inBuild = inBuild || strings.HasSuffix(frame.Function, ".buildFacetedBody")
		inTarget = inTarget || strings.HasSuffix(frame.Function, "."+c.target)
		if !more {
			break
		}
	}
	if inBuild && inTarget {
		c.entered = true
		return context.Canceled
	}
	return nil
}

func TestBooleanContextCancelsFacetedBodyFinishing(t *testing.T) {
	for _, target := range []string{"auditFacetedMesh", "meshVolumeMeasurement"} {
		t.Run(target, func(t *testing.T) {
			doc := New()
			a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
			b := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
			tr, err := r3.Translation(r3.Vec{X: 5, Y: 5, Z: 5})
			require.NoError(t, err)
			b, err = b.Placed(tr)
			require.NoError(t, err)
			beforeRecipe := doc.Recipe()
			beforeBodies := doc.Bodies()
			ctx := &internalBooleanBuildCancelContext{Context: t.Context(), target: target}

			_, err = UnionContext(ctx, a, b)
			require.ErrorIs(t, err, context.Canceled)
			require.True(t, ctx.entered)
			require.Equal(t, beforeRecipe, doc.Recipe())
			require.Equal(t, beforeBodies, doc.Bodies())
		})
	}
}

// classify runs the pair classifier on two float triangles, lifting them the
// way the mesh pass does.
func classify(t *testing.T, ta, tb [3]r3.Vec) triContact {
	t.Helper()
	xta := [3]xpt{xptOf(ta[0]), xptOf(ta[1]), xptOf(ta[2])}
	xtb := [3]xpt{xptOf(tb[0]), xptOf(tb[1]), xptOf(tb[2])}
	na := xcross(xsub(xta[1], xta[0]), xsub(xta[2], xta[0]))
	nb := xcross(xsub(xtb[1], xtb[0]), xsub(xtb[2], xtb[0]))
	c, err := triTriClassify(ta, tb, xta, xtb, na, nb)
	require.NoError(t, err)
	return c
}

func TestTriTriClassifyIsSymmetric(t *testing.T) {
	// The pair below is the one the old branch grid dropped: two of A's vertices
	// lie on B's plane, and B's vertex (0, 0, 0) sits strictly INSIDE the A edge
	// they span. The old code, having entered the "two of A's vertices are on the
	// plane" cell, only ever looked for a touch among A's OWN vertices — and
	// neither of them lies on B — so it reported no contact at all, dropping a
	// real point contact. Asking what the intersection IS, rather than whose
	// geometry to look on, cannot make that mistake: it is one point, whichever
	// way round the pair is handed in.
	a := [3]r3.Vec{{X: 0, Y: 0, Z: -5}, {X: 0, Y: 0, Z: 5}, {X: 12, Y: -4, Z: 5}}
	b := [3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: -12, Y: 0, Z: -6}, {X: -12, Y: 0, Z: 5}}

	fwd := classify(t, a, b)
	require.Equal(t, contactPoint, fwd.kind)
	require.Equal(t, r3.Vec{}, fwd.p0.vec(), `the contact is the origin`)
	// The point lies on A's boundary (inside its edge) AND on B's (its corner).
	require.True(t, fwd.p0OnA)
	require.True(t, fwd.p0OnB)

	rev := classify(t, b, a)
	require.Equal(t, contactPoint, rev.kind)
	require.Equal(t, fwd.p0.vec(), rev.p0.vec(), `the answer does not depend on the argument order`)
	require.Equal(t, fwd.p0OnA, rev.p0OnB)
	require.Equal(t, fwd.p0OnB, rev.p0OnA)
}

func TestTriTriClassifyNamesTheInPlaneEdge(t *testing.T) {
	// A's edge 0 lies exactly in B's plane (z = 0) and crosses B's interior: the
	// contact is a segment, and it runs ALONG that edge. The pair reports WHICH
	// edge and stops there — whether the edge grazes B or crosses it is decided
	// by the edge's two adjacent facets, which no pair can see.
	a := [3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 4, Y: 0, Z: 0}, {X: 2, Y: 1, Z: 3}}
	b := [3]r3.Vec{{X: -2, Y: -2, Z: 0}, {X: 6, Y: -2, Z: 0}, {X: 2, Y: 6, Z: 0}}

	c := classify(t, a, b)
	require.Equal(t, contactSegment, c.kind)
	require.Equal(t, 0, c.edgeA, `the contact runs along A's edge 0`)
	require.Equal(t, -1, c.edgeB, `it crosses B's interior, along no edge of B`)
	// Both endpoints are A's own corners, so both lie on A's boundary; both lie
	// strictly inside B, so neither is on B's.
	require.True(t, c.p0OnA)
	require.True(t, c.p1OnA)
	require.False(t, c.p0OnB)
	require.False(t, c.p1OnB)
}

// singleFacetBoolMesh prepares a one-triangle operand mesh, the smallest input
// facesNearMiss can be asked about.
func singleFacetBoolMesh(t *testing.T, tri [3]r3.Vec) *boolMesh {
	t.Helper()
	bm, err := prepBoolMeshContext(t.Context(), &Mesh{vertices: tri[:], triangles: [][3]int{{0, 1, 2}}}, []int{0})
	require.NoError(t, err)
	return bm
}

// TestContactMemoRepeatsTheClassifier pins that contactMemo.classify serves a
// repeat ask for the same facet pair from its store rather than recomputing
// it, and that the served answer is coordinate-identical to a direct
// triTriClassify call — for a genuine contact and for a miss alike.
//
// A repeat ask that recomputes returns the same answer as one served from the
// store, so no comparison of the two answers can tell them apart, and neither
// can the store's own size: a recompute writes back the same key. What
// separates them is WHICH facets the second ask reads. So each half below
// rebinds the memo's operand mesh between the two asks, to one whose facet 0
// classifies DIFFERENTLY against the same facet of ma — proven by a direct
// triTriClassify on the swapped pair. A second ask that recomputed would have
// to report that different answer; reporting the first one is only possible
// from the store. Rebinding a live memo is a probe this test alone performs:
// production binds ma/mb once per evaluateBoolean call precisely so a stored
// answer can never be read back for another operand pair.
func TestContactMemoRepeatsTheClassifier(t *testing.T) {
	// A facet held in the plane z = 0 and one held in the plane x = 0: the
	// planes meet along the y axis, and each triangle's own chord along that
	// axis overlaps the other's, so the pair meets in a positive-length
	// segment running along neither facet's own edge.
	a := [3]r3.Vec{{X: -5, Y: -5, Z: 0}, {X: 5, Y: -5, Z: 0}, {X: 0, Y: 5, Z: 0}}
	b := [3]r3.Vec{{X: 0, Y: -3, Z: -3}, {X: 0, Y: -3, Z: 3}, {X: 0, Y: 3, Z: 0}}
	// Far enough from a that the pair cannot meet at all.
	miss := [3]r3.Vec{{X: 100, Y: 0, Z: 0}, {X: 101, Y: 0, Z: 0}, {X: 100, Y: 1, Z: 0}}

	direct := classify(t, a, b)
	require.Equal(t, contactSegment, direct.kind)
	require.Equal(t, contactNone, classify(t, a, miss).kind, `the two operands below give the same facet of A opposite answers`)

	bmA, bmB, bmMiss := singleFacetBoolMesh(t, a), singleFacetBoolMesh(t, b), singleFacetBoolMesh(t, miss)

	requireSameContact := func(want, got triContact) {
		t.Helper()
		require.Equal(t, want.kind, got.kind)
		require.Equal(t, want.edgeA, got.edgeA)
		require.Equal(t, want.edgeB, got.edgeB)
		require.Zero(t, want.p0.x.Cmp(got.p0.x))
		require.Zero(t, want.p0.y.Cmp(got.p0.y))
		require.Zero(t, want.p0.z.Cmp(got.p0.z))
		require.Zero(t, want.p1.x.Cmp(got.p1.x))
		require.Zero(t, want.p1.y.Cmp(got.p1.y))
		require.Zero(t, want.p1.z.Cmp(got.p1.z))
		require.Zero(t, want.sin2.Cmp(got.sin2))
	}

	memo := newContactMemo(bmA, bmB)
	first, err := memo.classify(0, 0)
	require.NoError(t, err)
	requireSameContact(direct, first)

	memo.mb = bmMiss
	second, err := memo.classify(0, 0)
	require.NoError(t, err)
	require.Equal(t, contactSegment, second.kind, `the second ask was served from the store: a recompute would report the swapped operand's miss`)
	requireSameContact(first, second)
	require.Len(t, memo.m, 1, `the pair keeps one entry, under the one key`)

	// A pair that misses is stored too, so a repeat of a non-contact does not
	// reclassify either. The swap runs the other way round here: the second ask
	// would report the contact if it recomputed.
	missMemo := newContactMemo(bmA, bmMiss)
	first, err = missMemo.classify(0, 0)
	require.NoError(t, err)
	require.Equal(t, contactNone, first.kind)

	missMemo.mb = bmB
	second, err = missMemo.classify(0, 0)
	require.NoError(t, err)
	require.Equal(t, contactNone, second.kind, `the stored miss was served: a recompute would report the swapped operand's segment`)
	require.Len(t, missMemo.m, 1, `the pair keeps one entry, under the one key`)
}

// inscribedFan is the held outline of a circle of radius r centred at (cx, 0)
// in the plane z = 0: the inscribed n-gon, fan-triangulated. The vertices sit
// at the half-step angles, so an EDGE — not a vertex — faces the y axis at both
// ends of the diameter, which is where a rim tangency lands in the test below.
func inscribedFan(cx, r float64, n int) [][3]r3.Vec {
	pts := make([]r3.Vec, n)
	for k := range n {
		a := 2 * math.Pi * (float64(k) + 0.5) / float64(n)
		pts[k] = r3.Vec{X: cx + r*math.Cos(a), Y: r * math.Sin(a)}
	}
	fan := make([][3]r3.Vec, 0, n-2)
	for k := 1; k < n-1; k++ {
		fan = append(fan, [3]r3.Vec{pts[0], pts[k], pts[k+1]})
	}
	return fan
}

// TestCoplanarCarrierPairIsNotSettledByCoplanarityAlone pins the two facts
// docs/interference-design.md §5.2 records about the hidden-tangency gate and a
// coplanar carrier pair, which together are why §11's PR4 must settle the
// near-miss question before it removes the mesh pass's coplanar refusal.
func TestCoplanarCarrierPairIsNotSettledByCoplanarityAlone(t *testing.T) {
	t.Run("a positive-area coplanar overlap defers to the mesh pass", func(t *testing.T) {
		// Two opposed facets sharing the plane z = 0 and overlapping over a
		// positive area: what a cap-on-cap tangency looks like to the gate.
		ta := [3]r3.Vec{{X: 0, Y: 0}, {X: 10, Y: 0}, {X: 0, Y: 10}}
		tb := [3]r3.Vec{{X: 0, Y: 0}, {X: 0, Y: 10}, {X: 10, Y: 0}}
		require.Equal(t, contactRegion, classify(t, ta, tb).kind)

		bmA, bmB := singleFacetBoolMesh(t, ta), singleFacetBoolMesh(t, tb)
		near, err := facesNearMiss(t.Context(), bmA, []int{0}, bmB, []int{0}, 1, newContactMemo(bmA, bmB))
		require.NoError(t, err)
		// The gate answers "no near miss" for the whole face pair without
		// proving one: the pair is left to the mesh pass's own refusal of an
		// unclassifiable coplanar contact. Whatever replaces that refusal owes
		// this pair a near-miss answer of its own.
		require.False(t, near, `the gate defers the coplanar overlap rather than deciding it`)
	})

	t.Run("tangent curved rims leave no positive-area cell", func(t *testing.T) {
		const (
			n = 16
			r = 5.0
		)
		// Circles of radius r centred at the origin and at (2r, 0) are
		// externally tangent at (r, 0): the true rims touch, in one exact
		// shared plane. Their held outlines are inscribed, so they do not.
		a := inscribedFan(0, r, n)
		b := inscribedFan(2*r, r, n)

		gap := math.Inf(1)
		for _, ta := range a {
			for _, tb := range b {
				require.Equal(t, contactNone, classify(t, ta, tb).kind, `no held facet pair meets, so the arrangement has no positive-area cell to classify`)
				gap = math.Min(gap, triTriDistance(ta, tb))
			}
		}
		// The true touch falls in the gap the two chords leave — two sagittas
		// wide — which is the allowance a coplanar carrier pair does not
		// dispose of.
		require.InDelta(t, 2*r*(1-math.Cos(math.Pi/n)), gap, 1e-9)
		require.Positive(t, gap)
	})
}

// TestNearMissKeepsACrossingTheDistanceRoutineMisreads pins the order the
// proximity gate asks its two questions in: the EXACT classifier decides
// whether a facet pair meets, and triTriDistance is consulted only afterwards,
// on a pair the classifier has already proven disjoint (contactNone), where its
// own disjointness precondition holds.
//
// The pair below is why the order matters. triTriDistance minimises over nine
// edge-edge and six vertex-to-triangle distances, which is where two DISJOINT
// convex sets attain their minimum. An intersecting pair attains it in the
// interiors instead, so that candidate set misses it entirely and the routine
// reports a value far above the slack for a pair that provably crosses. Asked
// FIRST, it would drop the crossing from the gate's contact set, and a face
// pair whose solid penetration is at or below the slack — the case
// provenDepthExceeds cannot certify — would clear the gate with no proof
// behind it, its topology decided by where the chords fell.
func TestNearMissKeepsACrossingTheDistanceRoutineMisreads(t *testing.T) {
	// A needle piercing a plate's interior: tb runs from z = -0.5 to z = 0.5
	// through the plane z = 0, strictly inside ta.
	ta := [3]r3.Vec{{X: -5, Y: -5, Z: 0}, {X: 5, Y: -5, Z: 0}, {X: 0, Y: 5, Z: 0}}
	tb := [3]r3.Vec{{X: -2, Y: 0, Z: -0.5}, {X: 2, Y: 0, Z: -0.5}, {X: 0, Y: 0, Z: 0.5}}

	const slack = 0.1
	require.Equal(t, contactSegment, classify(t, ta, tb).kind, `the exact classifier proves the pair crosses`)
	require.Greater(t, triTriDistance(ta, tb), slack,
		`the distance routine reads the crossing pair as far apart, so it can never be the gate's first question`)

	bmA, bmB := singleFacetBoolMesh(t, ta), singleFacetBoolMesh(t, tb)
	near, err := facesNearMiss(t.Context(), bmA, []int{0}, bmB, []int{0}, slack, newContactMemo(bmA, bmB))
	require.NoError(t, err)
	require.True(t, near, `a proven crossing the gate cannot certify deeper than the slack stays undecidable`)
}

// tinyOffset is a displacement far below one ulp at the coordinates below, so
// two exact points a tinyOffset apart round to the SAME float64 vertex — which
// is what makes the stitcher weld them, and the facets they span collapse.
func tinyOffset() *big.Rat {
	return new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Exp(big.NewInt(10), big.NewInt(20), nil))
}

// xptFromRat builds an exact point directly from three big.Rat coordinates —
// used only where a test needs sub-ulp control production's float-only entry
// point (xptOf) cannot express, over one shared homogeneous denominator the
// same way xhpOf lifts a float vertex.
func xptFromRat(x, y, z *big.Rat) xpt {
	dx, dy, dz := x.Denom(), y.Denom(), z.Denom()
	return xpt{
		x: new(big.Int).Mul(x.Num(), new(big.Int).Mul(dy, dz)),
		y: new(big.Int).Mul(y.Num(), new(big.Int).Mul(dx, dz)),
		z: new(big.Int).Mul(z.Num(), new(big.Int).Mul(dx, dy)),
		w: new(big.Int).Mul(dx, new(big.Int).Mul(dy, dz)),
	}
}

// xat is an exact point from whole millimetres, optionally nudged by a
// sub-ulp offset on one axis.
func xat(x, y, z float64, nudge int) xpt {
	rx, ry, rz := mustRatOf(x), mustRatOf(y), mustRatOf(z)
	switch nudge {
	case 0:
		rx = new(big.Rat).Add(rx, tinyOffset())
	case 1:
		ry = new(big.Rat).Add(ry, tinyOffset())
	case 2:
		rz = new(big.Rat).Add(rz, tinyOffset())
	}
	return xptFromRat(rx, ry, rz)
}

// splitApexTetra is a closed tetra A,B,C,D whose apex D is split into the edge
// D1–D2, a tinyOffset long: the two facets bridging that edge (B,D2,D1) and
// (A,D1,D2) collapse under the weld, while the component they belong to
// survives as the tetra. Every directed edge pairs with its reverse, so the
// exact closure audit passes before the rounding ever runs.
func splitApexTetra() []keptFacet {
	a, b, c := xptOf(r3.NewVec(0, 0, 0)), xptOf(r3.NewVec(10, 0, 0)), xptOf(r3.NewVec(0, 10, 0))
	d1 := xptOf(r3.NewVec(2, 2, 9))
	d2 := xat(2, 2, 9, 0)
	return []keptFacet{
		{v: [3]xpt{a, c, b}},
		{v: [3]xpt{a, b, d1}},
		{v: [3]xpt{b, c, d2}},
		{v: [3]xpt{c, a, d2}},
		{v: [3]xpt{b, d2, d1}},
		{v: [3]xpt{a, d1, d2}},
	}
}

// subUlpTetra is a closed tetra whose four vertices all round to the SAME
// float64 vertex: every one of its facets collapses under the weld, so the
// whole component is welded out of existence.
func subUlpTetra() []keptFacet {
	p := xptOf(r3.NewVec(40, 40, 40))
	q, r, s := xat(40, 40, 40, 0), xat(40, 40, 40, 1), xat(40, 40, 40, 2)
	return []keptFacet{
		{v: [3]xpt{p, r, q}},
		{v: [3]xpt{p, q, s}},
		{v: [3]xpt{q, r, s}},
		{v: [3]xpt{r, p, s}},
	}
}

func TestStitchRefusesAWeldedAwayComponent(t *testing.T) {
	// The whole tiny component rounds onto one float vertex, so every facet of
	// it collapses and it disappears from the held mesh — a lump gone from the
	// body, with its volume, its place in Lumps() and its reach in the bounds
	// box. The closure audit does not see it: the component that remains still
	// closes. Nothing downstream would report it either, so the stitcher
	// refuses here.
	_, err := stitchFacetsContext(t.Context(), append(splitApexTetra(), subUlpTetra()...))
	require.ErrorIs(t, err, ErrUnsupported)

	// It is the SURVIVING company that made the loss silent: a result that is
	// nothing but the tiny component has no extent left at all, and the stitcher
	// already refused that outright.
	_, err = stitchFacetsContext(t.Context(), subUlpTetra())
	require.ErrorIs(t, err, ErrBooleanFailed)
}

func TestStitchChargesTheFacetsTheWeldDrops(t *testing.T) {
	// A collapse INSIDE a surviving component is not refused — it is an edge
	// contraction, and the surface that remains is the tetra. But the two facets
	// it drops were not zero-area before the weld, and both of the things they
	// carried are charged: their swept volume, against the PRE-ROUND surface
	// (preArea), and the area the held mesh can no longer report (dropArea).
	got, err := stitchFacetsContext(t.Context(), splitApexTetra())
	require.NoError(t, err)
	require.Len(t, got.tris, 4, `the two bridging facets collapse; the tetra survives`)

	held := meshAreaUpper(got.verts, got.tris)
	require.Greater(t, got.preArea, held, `the rounding is charged against the surface it acted on, not the one that survived it`)
	require.Positive(t, got.dropArea, `the dropped facets' own area is charged`)
	require.Positive(t, got.round)
	// The volume the weld can have moved is bounded by the displacement times
	// the pre-round area — a strictly larger charge than the held mesh's own.
	require.Greater(t, sweptVolumeAllow(got.round, got.preArea), sweptVolumeAllow(got.round, held))
}

// TestBooleanVolumesAreUnchangedByTheKernelRewrite is fu163's end-to-end
// proof: the mesh boolean's reported volume, centroid and bounds must be
// bit-identical to what the math/big.Rat kernel this change replaces
// reported — a tolerance would hide a real change, since the whole claim is
// that the signs, and therefore the topology and the exact integrals built on
// them, are unchanged. Two boxes translated off every shared face plane
// (5, 5, 5) force the mesh path rather than the analytic prism-boolean
// reduction (TestBooleanContextCancelsFacetedBodyFinishing above cancels
// inside buildFacetedBody on the identical fixture, which is mesh-path-only).
// The expected numbers were captured from this same fixture on the
// pre-rewrite math/big.Rat kernel before this change landed.
func TestBooleanVolumesAreUnchangedByTheKernelRewrite(t *testing.T) {
	type want struct {
		volume           float64
		cx, cy, cz       float64
		minX, minY, minZ float64
		maxX, maxY, maxZ float64
	}
	testcases := []struct {
		name string
		op   func(a, b *Body) (*Body, error)
		want want
	}{
		{"Union", Union, want{1875, 7.5, 7.5, 7.5, 0, 0, 0, 15, 15, 15}},
		{"Cut", Cut, want{875, 4.642857142857143, 4.642857142857143, 4.642857142857143, 0, 0, 0, 10, 10, 10}},
		{"Intersect", Intersect, want{125, 7.5, 7.5, 7.5, 5, 5, 5, 10, 10, 10}},
	}
	for _, tc := range testcases {
		t.Run(tc.name, func(t *testing.T) {
			doc := New()
			a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
			b := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
			tr, err := r3.Translation(r3.Vec{X: 5, Y: 5, Z: 5})
			require.NoError(t, err)
			b, err = b.Placed(tr)
			require.NoError(t, err)

			result, err := tc.op(a, b)
			require.NoError(t, err)

			volM, err := result.Volume()
			require.NoError(t, err)
			vol, err := volM.Value.In(units.CubicMillimeter)
			require.NoError(t, err)
			require.Equal(t, tc.want.volume, vol, `volume must be bit-identical to the pre-rewrite kernel`)

			cen, err := result.Centroid()
			require.NoError(t, err)
			require.Equal(t, r3.Vec{X: tc.want.cx, Y: tc.want.cy, Z: tc.want.cz}, cen.Value,
				`centroid must be bit-identical to the pre-rewrite kernel`)

			bounds, err := result.Bounds()
			require.NoError(t, err)
			require.Equal(t, r3.Vec{X: tc.want.minX, Y: tc.want.minY, Z: tc.want.minZ}, bounds.Min,
				`bounds.Min must be bit-identical to the pre-rewrite kernel`)
			require.Equal(t, r3.Vec{X: tc.want.maxX, Y: tc.want.maxY, Z: tc.want.maxZ}, bounds.Max,
				`bounds.Max must be bit-identical to the pre-rewrite kernel`)
		})
	}
}

func TestPrepRefusesACollapsedOperandFacet(t *testing.T) {
	// A rigid placement's own rounding can collapse a facet of an already
	// faceted body. A collapsed facet has no plane and no interior, so every
	// contact predicate here is blind to it: a point or tangent contact made on
	// it would be classified by nothing at all. The operand is refused.
	m := &Mesh{
		vertices:  []r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 4, Y: 0, Z: 0}, {X: 2, Y: 0, Z: 0}},
		triangles: [][3]int{{0, 1, 2}},
	}
	_, err := prepBoolMeshContext(t.Context(), m, []int{0})
	require.ErrorIs(t, err, ErrUnsupported, `three collinear corners span no plane`)
}

// internalDiscBody extrudes a radius-r circle centered on the origin into an
// h mm prism — the internal-package twin of prism_boolean_bounds_test.go's
// discBody, needed here because boolMesh and its prep helpers are unexported
// and so this fixture cannot be built from the decad_test package. It takes
// testing.TB so a benchmark can build the same fixture as a test.
func internalDiscBody(t testing.TB, doc *Document, r, h float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	return body
}

// internalWasherBodySymmetric extrudes a circular annulus (outer radius
// outer, inner hole radius inner, centered on the origin) symmetrically about
// its own sketch plane, spanning [-half, +half] — the internal-package twin
// of boolean_test.go's washerBodySymmetric.
func internalWasherBodySymmetric(t testing.TB, doc *Document, outer, inner, half float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, outer)
	s.CreateCircle(center, inner)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof, `the washer's holed region should exist`)
	body, err := doc.Extrude(s, prof, Symmetric{D: units.Millimeters(half)})
	require.NoError(t, err)
	return body
}

// buildCircularWasherMeshes tessellates fu158's disc/washer fixture into the
// two boolMeshes the mesh boolean itself would build for the pair's own Cut —
// the same chord tolerance pairChordTolerance derives — so the corpus below
// is the one the real evaluator classifies, not an approximation of it.
func buildCircularWasherMeshes(t testing.TB) (*boolMesh, *boolMesh) {
	t.Helper()
	doc := New()
	target := internalDiscBody(t, doc, 15, 10)
	tool := internalWasherBodySymmetric(t, doc, 8, 3, 11)
	tolMM, _, err := pairChordTolerance(target, tool)
	require.NoError(t, err)
	ma, err := tessellateContext(t.Context(), target, units.Millimeters(tolMM))
	require.NoError(t, err)
	mb, err := tessellateContext(t.Context(), tool, units.Millimeters(tolMM))
	require.NoError(t, err)
	bmA, err := prepBoolMeshContext(t.Context(), ma, make([]int, len(ma.triangles)))
	require.NoError(t, err)
	bmB, err := prepBoolMeshContext(t.Context(), mb, make([]int, len(mb.triangles)))
	require.NoError(t, err)
	return bmA, bmB
}

// requireSameTriContact asserts two triContacts are identical field for
// field, comparing every exact rational by big.Rat.Cmp — never by float
// equality — which is the "the fast path returns what the slow path
// returned" proof over an exact corpus (fu158, .tmp/followup-tasks/fu158-tasks.md §5).
func requireSameTriContact(t *testing.T, exact, filtered triContact) {
	t.Helper()
	require.Equal(t, exact.kind, filtered.kind)
	if exact.kind == contactNone {
		return
	}
	require.Zero(t, exact.p0.x.Cmp(filtered.p0.x))
	require.Zero(t, exact.p0.y.Cmp(filtered.p0.y))
	require.Zero(t, exact.p0.z.Cmp(filtered.p0.z))
	require.Zero(t, exact.p1.x.Cmp(filtered.p1.x))
	require.Zero(t, exact.p1.y.Cmp(filtered.p1.y))
	require.Zero(t, exact.p1.z.Cmp(filtered.p1.z))
	require.Equal(t, exact.p0OnA, filtered.p0OnA)
	require.Equal(t, exact.p1OnA, filtered.p1OnA)
	require.Equal(t, exact.p0OnB, filtered.p0OnB)
	require.Equal(t, exact.p1OnB, filtered.p1OnB)
	require.Equal(t, exact.edgeA, filtered.edgeA)
	require.Equal(t, exact.edgeB, filtered.edgeB)
	if exact.sin2 == nil {
		require.Nil(t, filtered.sin2)
		return
	}
	require.NotNil(t, filtered.sin2)
	require.Zero(t, exact.sin2.Cmp(filtered.sin2))
}

// TestTriTriClassifyFilterAgreesWithTheExactPath is fu158's own pin: every
// AABB-surviving facet pair of the disc/washer fixture must classify
// identically with triTriMissesFilter on and off. The filter changes no
// verdict, so this is what stands between "faster" and "a different answer"
// (.tmp/followup-tasks/fu158-tasks.md §6).
func TestTriTriClassifyFilterAgreesWithTheExactPath(t *testing.T) {
	t.Cleanup(func() { triTriFilterEnabled = true })

	ma, mb := buildCircularWasherMeshes(t)
	pairs, nonNone := 0, 0
	for i := range ma.tris {
		for j := range mb.tris {
			if !boxesOverlap(ma.boxes[i], mb.boxes[j]) {
				continue
			}
			pairs++
			ta, tb := triCorners(ma, i), triCorners(mb, j)
			xta, xtb := xtriCorners(ma, i), xtriCorners(mb, j)
			na, nb := ma.norms[i], mb.norms[j]

			triTriFilterEnabled = true
			filtered, err := triTriClassify(ta, tb, xta, xtb, na, nb)
			require.NoError(t, err)
			triTriFilterEnabled = false
			exact, err := triTriClassify(ta, tb, xta, xtb, na, nb)
			require.NoError(t, err)

			requireSameTriContact(t, exact, filtered)
			if exact.kind != contactNone {
				nonNone++
			}
		}
	}
	require.Positive(t, pairs, `the AABB-surviving corpus must be non-empty`)
	require.NotZero(t, nonNone, `a filter that rejected every real contact must not pass`)
}

// TestTriTriClassifyFilterAgreesAtAShallowDihedralAngle is the risk section's
// own named case (.tmp/followup-tasks/fu158-tasks.md §7): two facets sharing
// an edge but meeting at a dihedral angle of about 1e-6 rad. dir = na × nb is
// nearly zero there, and the crossing-point projections carry wide
// intervals — exactly where triSpanOnLine's own doc comment says it must
// abstain rather than resolve. Sharing an edge keeps the contact itself
// unambiguous (a positive-length segment along it), so the pair exercises the
// near-parallel path while still landing on a real answer.
func TestTriTriClassifyFilterAgreesAtAShallowDihedralAngle(t *testing.T) {
	t.Cleanup(func() { triTriFilterEnabled = true })

	a := [3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 10, Y: 0, Z: 0}, {X: 5, Y: 10, Z: 0}}
	b := [3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 10, Y: 0, Z: 0}, {X: 5, Y: -10, Z: 1e-6}}
	xta := [3]xpt{xptOf(a[0]), xptOf(a[1]), xptOf(a[2])}
	xtb := [3]xpt{xptOf(b[0]), xptOf(b[1]), xptOf(b[2])}
	na := xcross(xsub(xta[1], xta[0]), xsub(xta[2], xta[0]))
	nb := xcross(xsub(xtb[1], xtb[0]), xsub(xtb[2], xtb[0]))

	triTriFilterEnabled = true
	filtered, err := triTriClassify(a, b, xta, xtb, na, nb)
	require.NoError(t, err)
	triTriFilterEnabled = false
	exact, err := triTriClassify(a, b, xta, xtb, na, nb)
	require.NoError(t, err)

	require.Equal(t, contactSegment, exact.kind, `the shared edge is a real, unambiguous contact`)
	requireSameTriContact(t, exact, filtered)
}

// BenchmarkTriTriClassifyCircularPairs isolates fu158's fix from the rest of
// the mesh-boolean pipeline: it builds the disc/washer fixture's two
// boolMeshes once, harvests every AABB-surviving facet-pair index once, then
// classifies the whole corpus per iteration — the cost triTriMissesFilter
// exists to cut (docs/evaluator-design.md §9).
func BenchmarkTriTriClassifyCircularPairs(b *testing.B) {
	ma, mb := buildCircularWasherMeshes(b)
	type facetPair struct{ i, j int }
	var pairs []facetPair
	for i := range ma.tris {
		for j := range mb.tris {
			if boxesOverlap(ma.boxes[i], mb.boxes[j]) {
				pairs = append(pairs, facetPair{i: i, j: j})
			}
		}
	}
	require.NotEmpty(b, pairs, `the AABB-surviving corpus must be non-empty`)

	for b.Loop() {
		for _, p := range pairs {
			ta, tb := triCorners(ma, p.i), triCorners(mb, p.j)
			_, err := triTriClassify(ta, tb, xtriCorners(ma, p.i), xtriCorners(mb, p.j), ma.norms[p.i], mb.norms[p.j])
			if err != nil {
				b.Fatal(err)
			}
		}
	}
}

func TestFacetFaceIndicesMapsConsistentFaces(t *testing.T) {
	f0, f1 := &Face{}, &Face{}
	got, err := facetFaceIndices(t.Context(), []*Face{f0, f1}, []*Face{f1, f0, f1})
	require.NoError(t, err)
	require.Equal(t, []int{1, 0, 1}, got,
		`each facet must map to its face's index in the built body's Faces() order`)
}

func TestFacetFaceIndicesRejectsUnmappedFacet(t *testing.T) {
	f0, f1 := &Face{}, &Face{}
	orphan := &Face{} // a face absent from the built body's Faces()
	_, err := facetFaceIndices(t.Context(), []*Face{f0, f1}, []*Face{f0, orphan})
	// Without the miss guard, a Go map lookup yields the zero value 0 and the
	// facet is silently attributed to face 0; the guard turns that invariant
	// break into an error instead.
	require.ErrorIs(t, err, ErrBooleanFailed)
}

func TestFacetedPlacementRebuildsCachedDiameter(t *testing.T) {
	doc := New()
	body, err := buildFacetedBody(t.Context(), doc, StepRef(0), facetedPayload{
		verts: []r3.Vec{
			r3.NewVec(0, 0, 0),
			r3.NewVec(3, 0, 0),
			r3.NewVec(0, 4, 0),
			r3.NewVec(0, 0, 5),
		},
		tris: [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}},
		src:  []int{0, 1, 2, 3},
		groups: []facetGroup{
			{planar: true}, {planar: true}, {planar: true}, {planar: true},
		},
		dPair: 10,
		xform: r3.Identity(),
	})
	require.NoError(t, err)

	before := body.payload.(facetedPayload)
	rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	translation, err := r3.Translation(r3.NewVec(1e12, -2e12, 3e12))
	require.NoError(t, err)
	placement, err := rotation.Then(translation)
	require.NoError(t, err)

	placed, err := before.placed(t.Context(), doc, StepRef(1), placement)
	require.NoError(t, err)
	after := placed.payload.(facetedPayload)
	want, ok := pointSetDiameter(after.verts)
	require.True(t, ok)
	require.NotEqual(t, before.diameter, want, "the placement must make a stale cached diameter observable")
	require.Equal(t, want, after.diameter)
}

// TestBooleanComposesTheOperandsOwnSymmetricDifferenceProofs is
// docs/tessellation-reach-design.md §3's boolean half: the result's
// occupied-volume bound is composed from each operand mesh's OWN volSymDiff
// proof, never from the `Mesh.Bound × held area` product
// docs/tessellation-design.md §11 forbids.
func TestBooleanComposesTheOperandsOwnSymmetricDifferenceProofs(t *testing.T) {
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 20, 20, 10)
	disc := internalDiscBody(t, doc, 4, 10)
	tr, err := r3.Translation(r3.NewVec(10, 10, 5))
	require.NoError(t, err)
	pin, err := disc.Placed(tr)
	require.NoError(t, err)

	tolMM, _, err := pairChordTolerance(plate, pin)
	require.NoError(t, err)
	ma, err := tessellateContext(t.Context(), plate, units.Millimeters(tolMM))
	require.NoError(t, err)
	mb, err := tessellateContext(t.Context(), pin, units.Millimeters(tolMM))
	require.NoError(t, err)

	symA, err := operandSymDiff(ma)
	require.NoError(t, err)
	symB, err := operandSymDiff(mb)
	require.NoError(t, err)
	require.Equal(t, ma.volSymDiff, symA, `the boolean reads the mesh's own proof, not a substitution`)
	require.Equal(t, mb.volSymDiff, symB)
	require.Zero(t, symA, `an all-planar box at exact coordinates differs from its mesh by nothing`)
	require.Positive(t, symB, `a chorded cylinder omits its own circular segments`)

	// The forbidden product, for comparison: strictly larger than the proof the
	// operand actually carries.
	substituted := mb.bound * meshAreaUpper(mb.vertices, mb.triangles)
	require.Greater(t, substituted, symB)

	eval, err := evaluateBoolean(t.Context(), OpUnion, plate, pin)
	require.NoError(t, err)
	// Step 6 of docs/tessellation-design.md §11: the operands' own bounds plus
	// the final weld's swept volume, which is non-negative and nothing else.
	require.GreaterOrEqual(t, eval.payload.volSymDiff, symA+symB)
	require.Less(t, eval.payload.volSymDiff, symA+substituted,
		`the result carries the operand's proof, not the product that would have stood in for it`)
	volMM, err := eval.volume.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.GreaterOrEqual(t, volMM, eval.payload.volSymDiff)
}

func TestOperandSymDiffRefusesAMeshWithNoOccupiedVolumeProof(t *testing.T) {
	// An export-only mesh — one whose payload class has no occupied-volume proof
	// yet — is refused as a staging limit, never composed as a zero.
	_, err := operandSymDiff(&Mesh{volSymDiff: 17, symDiffOK: false})
	require.ErrorIs(t, err, ErrUnsupported)

	got, err := operandSymDiff(&Mesh{volSymDiff: 17, symDiffOK: true})
	require.NoError(t, err)
	require.Equal(t, 17.0, got)
}

func TestFacesOfMeshReadsTheMeshProofRecord(t *testing.T) {
	stated, omitted := &Face{}, &Face{}
	m := &Mesh{
		triangles: [][3]int{{0, 1, 2}, {0, 2, 3}},
		source:    []*Face{stated, stated},
		faceBound: map[*Face]float64{stated: 0.25},
	}
	got, err := facesOfMesh(newWorkBudget(t.Context()), m)
	require.NoError(t, err)
	require.Len(t, got, 1)
	require.Equal(t, 0.25, got[0].delta, `the gate charges the face what the mesh proved for it`)
	require.Equal(t, []int{0, 1}, got[0].facets)

	// A source face the record omits is a broken evaluator, not a staged
	// capability, and never a zero the gate would read as "held exactly".
	m.source[1] = omitted
	_, err = facesOfMesh(newWorkBudget(t.Context()), m)
	require.ErrorIs(t, err, ErrBooleanFailed)
}
