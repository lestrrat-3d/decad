package decad

import (
	"context"
	"math"
	"math/big"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/polynomial"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
		inBuild = inBuild || strings.HasSuffix(frame.Function, ".buildFacetedBodyWithProof")
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
	t.Parallel()
	for _, target := range []string{"PointsContext", "facetFaceIndices"} {
		t.Run(target, func(t *testing.T) {
			doc := New()
			a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
			b := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
			tr, err := r3.Translation(r3.Vec{X: 5, Y: 5, Z: 5})
			require.NoError(t, err)
			b, err = b.Placed(t.Context(), tr)
			require.NoError(t, err)
			beforeProducer := doc.nextProducer
			beforeBodies := doc.Bodies()
			ctx := &internalBooleanBuildCancelContext{Context: t.Context(), target: target}

			_, err = Union(ctx, a, b)
			require.ErrorIs(t, err, context.Canceled)
			require.True(t, ctx.entered)
			require.Equal(t, beforeProducer, doc.nextProducer)
			require.Equal(t, beforeBodies, doc.Bodies())
		})
	}
}

// classify runs the pair classifier on two float triangles, lifting them the
// way the mesh pass does.
func classify(t *testing.T, ta, tb [3]r3.Vec) meshbool.TriContact {
	t.Helper()
	xta := [3]proofarith.Xpt{proofarith.XptOf(ta[0]), proofarith.XptOf(ta[1]), proofarith.XptOf(ta[2])}
	xtb := [3]proofarith.Xpt{proofarith.XptOf(tb[0]), proofarith.XptOf(tb[1]), proofarith.XptOf(tb[2])}
	na := proofarith.Xcross(proofarith.Xsub(xta[1], xta[0]), proofarith.Xsub(xta[2], xta[0]))
	nb := proofarith.Xcross(proofarith.Xsub(xtb[1], xtb[0]), proofarith.Xsub(xtb[2], xtb[0]))
	c, err := meshbool.TriTriClassify(ta, tb, xta, xtb, na, nb)
	require.NoError(t, err)
	return c
}

func TestTriTriClassifyIsSymmetric(t *testing.T) {
	t.Parallel()
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
	require.Equal(t, meshbool.ContactPoint, fwd.Kind)
	require.Equal(t, r3.Vec{}, fwd.P0.Vec(), `the contact is the origin`)
	// The point lies on A's boundary (inside its edge) AND on B's (its corner).
	require.True(t, fwd.P0OnA)
	require.True(t, fwd.P0OnB)

	rev := classify(t, b, a)
	require.Equal(t, meshbool.ContactPoint, rev.Kind)
	require.Equal(t, fwd.P0.Vec(), rev.P0.Vec(), `the answer does not depend on the argument order`)
	require.Equal(t, fwd.P0OnA, rev.P0OnB)
	require.Equal(t, fwd.P0OnB, rev.P0OnA)
}

func TestTriTriClassifyNamesTheInPlaneEdge(t *testing.T) {
	t.Parallel()
	// A's edge 0 lies exactly in B's plane (z = 0) and crosses B's interior: the
	// contact is a segment, and it runs ALONG that edge. The pair reports WHICH
	// edge and stops there — whether the edge grazes B or crosses it is decided
	// by the edge's two adjacent facets, which no pair can see.
	a := [3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 4, Y: 0, Z: 0}, {X: 2, Y: 1, Z: 3}}
	b := [3]r3.Vec{{X: -2, Y: -2, Z: 0}, {X: 6, Y: -2, Z: 0}, {X: 2, Y: 6, Z: 0}}

	c := classify(t, a, b)
	require.Equal(t, meshbool.ContactSegment, c.Kind)
	require.Equal(t, 0, c.EdgeA, `the contact runs along A's edge 0`)
	require.Equal(t, -1, c.EdgeB, `it crosses B's interior, along no edge of B`)
	// Both endpoints are A's own corners, so both lie on A's boundary; both lie
	// strictly inside B, so neither is on B's.
	require.True(t, c.P0OnA)
	require.True(t, c.P1OnA)
	require.False(t, c.P0OnB)
	require.False(t, c.P1OnB)
}

// singleFacetBoolMesh prepares a one-triangle operand mesh, the smallest input
// facesNearMiss can be asked about.
func singleFacetBoolMesh(t *testing.T, tri [3]r3.Vec) *meshbool.BoolMesh {
	t.Helper()
	face := &Face{}
	bm, err := prepBoolMeshContext(t.Context(), &Mesh{
		vertices: tri[:], triangles: [][3]int{{0, 1, 2}},
		source: []*Face{face}, faceBound: map[*Face]float64{face: 0},
	}, []int{0})
	require.NoError(t, err)
	return bm
}

// TestContactMemoRepeatsTheClassifier pins that meshbool.ContactMemo.classify serves a
// repeat ask for the same facet pair from its store rather than recomputing
// it, and that the served answer is coordinate-identical to a direct
// meshbool.TriTriClassify call — for a genuine contact and for a miss alike.
//
// A repeat ask that recomputes returns the same answer as one served from the
// store, so no comparison of the two answers can tell them apart, and neither
// can the store's own size: a recompute writes back the same key. What
// separates them is WHICH facets the second ask reads. So each half below
// rebinds the memo's operand mesh between the two asks, to one whose facet 0
// classifies DIFFERENTLY against the same facet of ma — proven by a direct
// meshbool.TriTriClassify on the swapped pair. A second ask that recomputed would have
// to report that different answer; reporting the first one is only possible
// from the store. Rebinding a live memo is a probe this test alone performs:
// production binds ma/mb once per evaluateBoolean call precisely so a stored
// answer can never be read back for another operand pair.
func TestContactMemoRepeatsTheClassifier(t *testing.T) {
	t.Parallel()
	// A facet held in the plane z = 0 and one held in the plane x = 0: the
	// planes meet along the y axis, and each triangle's own chord along that
	// axis overlaps the other's, so the pair meets in a positive-length
	// segment running along neither facet's own edge.
	a := [3]r3.Vec{{X: -5, Y: -5, Z: 0}, {X: 5, Y: -5, Z: 0}, {X: 0, Y: 5, Z: 0}}
	b := [3]r3.Vec{{X: 0, Y: -3, Z: -3}, {X: 0, Y: -3, Z: 3}, {X: 0, Y: 3, Z: 0}}
	// Far enough from a that the pair cannot meet at all.
	miss := [3]r3.Vec{{X: 100, Y: 0, Z: 0}, {X: 101, Y: 0, Z: 0}, {X: 100, Y: 1, Z: 0}}

	direct := classify(t, a, b)
	require.Equal(t, meshbool.ContactSegment, direct.Kind)
	require.Equal(t, meshbool.ContactNone, classify(t, a, miss).Kind, `the two operands below give the same facet of A opposite answers`)

	bmA, bmB, bmMiss := singleFacetBoolMesh(t, a), singleFacetBoolMesh(t, b), singleFacetBoolMesh(t, miss)

	requireSameContact := func(want, got meshbool.TriContact) {
		t.Helper()
		require.Equal(t, want.Kind, got.Kind)
		require.Equal(t, want.EdgeA, got.EdgeA)
		require.Equal(t, want.EdgeB, got.EdgeB)
		require.Zero(t, want.P0.X.Cmp(got.P0.X))
		require.Zero(t, want.P0.Y.Cmp(got.P0.Y))
		require.Zero(t, want.P0.Z.Cmp(got.P0.Z))
		require.Zero(t, want.P1.X.Cmp(got.P1.X))
		require.Zero(t, want.P1.Y.Cmp(got.P1.Y))
		require.Zero(t, want.P1.Z.Cmp(got.P1.Z))
		require.Zero(t, want.Sin2.Cmp(got.Sin2))
	}

	memo := meshbool.NewContactMemo(bmA, bmB)
	first, err := memo.Classify(0, 0)
	require.NoError(t, err)
	requireSameContact(direct, first)

	memo.Mb = bmMiss
	second, err := memo.Classify(0, 0)
	require.NoError(t, err)
	require.Equal(t, meshbool.ContactSegment, second.Kind, `the second ask was served from the store: a recompute would report the swapped operand's miss`)
	requireSameContact(first, second)

	// A pair that misses is stored too, so a repeat of a non-contact does not
	// reclassify either. The swap runs the other way round here: the second ask
	// would report the contact if it recomputed.
	missMemo := meshbool.NewContactMemo(bmA, bmMiss)
	first, err = missMemo.Classify(0, 0)
	require.NoError(t, err)
	require.Equal(t, meshbool.ContactNone, first.Kind)

	missMemo.Mb = bmB
	second, err = missMemo.Classify(0, 0)
	require.NoError(t, err)
	require.Equal(t, meshbool.ContactNone, second.Kind, `the stored miss was served: a recompute would report the swapped operand's segment`)
}

func TestContactMemoPromotesDenseWithoutChangingEntries(t *testing.T) {
	t.Parallel()
	const side = 64
	ma := &meshbool.BoolMesh{Tris: make([][3]int, side)}
	mb := &meshbool.BoolMesh{Tris: make([][3]int, side)}
	memo := meshbool.NewContactMemo(ma, mb)
	contacts := make([]meshbool.TriContact, side)
	for i := range side {
		contacts[i] = meshbool.TriContact{Kind: meshbool.ContactSegment, EdgeA: i % 3, EdgeB: (i + 1) % 3}
		if i%2 == 0 {
			contacts[i] = meshbool.TriContact{EdgeA: -1, EdgeB: -1}
		}
		memo.Store(i, i, contacts[i])
	}
	require.Nil(t, memo.Sparse)
	require.Len(t, memo.Dense, side*side)
	for i, want := range contacts {
		got, ok := memo.Lookup(i, i)
		require.True(t, ok)
		require.Equal(t, want, got)
	}
	_, ok := memo.Lookup(0, 1)
	require.False(t, ok)
}

func TestContactBatchPreparedNormalsAgreeWithStandalone(t *testing.T) {
	t.Parallel()
	a := [3]r3.Vec{{X: -5, Y: -5}, {X: 5, Y: -5}, {Y: 5}}
	b := [3]r3.Vec{{Y: -3, Z: -3}, {Y: -3, Z: 3}, {Y: 3}}
	miss := [3]r3.Vec{{X: 100}, {X: 101}, {X: 100, Y: 1}}

	bmA := singleFacetBoolMesh(t, a)
	for _, bmB := range []*meshbool.BoolMesh{singleFacetBoolMesh(t, b), singleFacetBoolMesh(t, miss)} {
		standalone, err := meshbool.TriTriClassify(meshbool.TriCorners(bmA, 0), meshbool.TriCorners(bmB, 0), meshbool.XtriCorners(bmA, 0), meshbool.XtriCorners(bmB, 0),
			bmA.Norms[0], bmB.Norms[0])
		require.NoError(t, err)
		results := make([]meshbool.ContactBatchResult, 1)
		err = meshbool.RunContactBatch(t.Context(), bmA, bmB, []meshbool.ContactPair{{}}, results, 1)
		require.NoError(t, err)
		require.Len(t, results, 1)
		require.True(t, bmA.FnormsReady[0])
		require.True(t, bmB.FnormsReady[0])
		requireSameTriContact(t, standalone, results[0].Contact)
	}
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
	t.Parallel()
	t.Run("a positive-area coplanar overlap defers to the mesh pass", func(t *testing.T) {
		// Two opposed facets sharing the plane z = 0 and overlapping over a
		// positive area: what a cap-on-cap tangency looks like to the gate.
		ta := [3]r3.Vec{{X: 0, Y: 0}, {X: 10, Y: 0}, {X: 0, Y: 10}}
		tb := [3]r3.Vec{{X: 0, Y: 0}, {X: 0, Y: 10}, {X: 10, Y: 0}}
		require.Equal(t, meshbool.ContactRegion, classify(t, ta, tb).Kind)

		bmA, bmB := singleFacetBoolMesh(t, ta), singleFacetBoolMesh(t, tb)
		near, err := meshbool.FacesNearMiss(t.Context(), bmA, []int{0}, bmB, []int{0}, 1, meshbool.NewContactMemo(bmA, bmB))
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
				require.Equal(t, meshbool.ContactNone, classify(t, ta, tb).Kind, `no held facet pair meets, so the arrangement has no positive-area cell to classify`)
				gap = math.Min(gap, meshbool.TriTriDistance(ta, tb))
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
// whether a facet pair meets, and meshbool.TriTriDistance is consulted only afterwards,
// on a pair the classifier has already proven disjoint (meshbool.ContactNone), where its
// own disjointness precondition holds.
//
// The pair below is why the order matters. meshbool.TriTriDistance minimises over nine
// edge-edge and six vertex-to-triangle distances, which is where two DISJOINT
// convex sets attain their minimum. An intersecting pair attains it in the
// interiors instead, so that candidate set misses it entirely and the routine
// reports a value far above the slack for a pair that provably crosses. Asked
// FIRST, it would drop the crossing from the gate's contact set, and a face
// pair whose solid penetration is at or below the slack — the case
// provenDepthExceeds cannot certify — would clear the gate with no proof
// behind it, its topology decided by where the chords fell.
func TestNearMissKeepsACrossingTheDistanceRoutineMisreads(t *testing.T) {
	t.Parallel()
	// A needle piercing a plate's interior: tb runs from z = -0.5 to z = 0.5
	// through the plane z = 0, strictly inside ta.
	ta := [3]r3.Vec{{X: -5, Y: -5, Z: 0}, {X: 5, Y: -5, Z: 0}, {X: 0, Y: 5, Z: 0}}
	tb := [3]r3.Vec{{X: -2, Y: 0, Z: -0.5}, {X: 2, Y: 0, Z: -0.5}, {X: 0, Y: 0, Z: 0.5}}

	const slack = 0.1
	require.Equal(t, meshbool.ContactSegment, classify(t, ta, tb).Kind, `the exact classifier proves the pair crosses`)
	require.Greater(t, meshbool.TriTriDistance(ta, tb), slack,
		`the distance routine reads the crossing pair as far apart, so it can never be the gate's first question`)

	bmA, bmB := singleFacetBoolMesh(t, ta), singleFacetBoolMesh(t, tb)
	near, err := meshbool.FacesNearMiss(t.Context(), bmA, []int{0}, bmB, []int{0}, slack, meshbool.NewContactMemo(bmA, bmB))
	require.NoError(t, err)
	require.True(t, near, `a proven crossing the gate cannot certify deeper than the slack stays undecidable`)
}

// internalAxisFrame is the rigid motion taking local +Z to dir and the origin
// to a, built the way apitest/boolean_proximity_test.go's axisFrame builds it.
func internalAxisFrame(t *testing.T, a, dir r3.Vec) r3.Transform {
	t.Helper()
	d, ok := dir.Normalize()
	require.True(t, ok)
	ref := r3.Vec{Y: 1}
	if math.Abs(d.Y) > 0.9 {
		ref = r3.Vec{X: 1}
	}
	p, ok := d.Cross(ref).Normalize()
	require.True(t, ok)
	frame, err := r3.FromBasis(r3.Basis{EX: p.Cross(d), EY: p, EZ: d}, a)
	require.NoError(t, err)
	return frame
}

// internalOctagonPrism is the internal-package twin of
// apitest/boolean_proximity_test.go's placed prism: a regular octagon of circumradius r,
// turned by phase, extruded l along local +Z and placed by frame.
func internalOctagonPrism(t *testing.T, doc *Document, frame r3.Transform, l, r, phase float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	const n = 8
	pts := make([]*sketch.Point, n)
	for i := range n {
		th := 2*math.Pi*float64(i)/n + phase
		pts[i] = s.CreatePoint(r*math.Cos(th), r*math.Sin(th))
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%n])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(l), Dir: Along})
	require.NoError(t, err)
	body, err = body.Placed(t.Context(), frame)
	require.NoError(t, err)
	return body
}

// gateFacePair is one face pair the hidden-tangency gate examines, with the
// contacts it gathered and the bound it decides them against.
type gateFacePair struct {
	slack float64
	nc    meshbool.NearContacts
}

// closeGateFacePairs prepares a and b the way evaluateBoolean does and returns
// every face pair whose facets meet or come within the pair's bound — the
// pairs refuseUndecidableProximity must decide — with the prepared meshes.
func closeGateFacePairs(t *testing.T, a, b *Body) (*meshbool.BoolMesh, *meshbool.BoolMesh, []gateFacePair) {
	t.Helper()
	tolMM, _, err := pairChordTolerance(t.Context(), a, b)
	require.NoError(t, err)
	ma, err := tessellateContext(t.Context(), a, units.Millimeters(tolMM), VerifyAll)
	require.NoError(t, err)
	mb, err := tessellateContext(t.Context(), b, units.Millimeters(tolMM), VerifyAll)
	require.NoError(t, err)
	bmA, err := prepBoolMeshContext(t.Context(), ma, make([]int, len(ma.triangles)))
	require.NoError(t, err)
	bmB, err := prepBoolMeshContext(t.Context(), mb, make([]int, len(mb.triangles)))
	require.NoError(t, err)
	budget := proofbound.NewWorkBudget(t.Context())
	fa, err := facesOfMesh(budget, ma)
	require.NoError(t, err)
	fb, err := facesOfMesh(budget, mb)
	require.NoError(t, err)

	memo := meshbool.NewContactMemo(bmA, bmB)
	var out []gateFacePair
	for _, ga := range fa {
		for _, gb := range fb {
			slack := (ga.delta + gb.delta) * (1 + 1e-9)
			if slack <= 0 {
				continue
			}
			nc, deferred, err := meshbool.GatherNearContacts(t.Context(), bmA, ga.facets, bmB, gb.facets, slack, memo)
			require.NoError(t, err)
			require.False(t, deferred, `no two faces of the pair overlap in one plane`)
			if len(nc.CloseA) == 0 {
				continue
			}
			out = append(out, gateFacePair{slack: slack, nc: nc})
		}
	}
	return bmA, bmB, out
}

// TestProximityGateWalksPastCornerSamples pins why the hidden-tangency gate
// walks from contact segments. The fixture is TestUnionOfTiltedPrismsCrossingMidWall's
// pair: two placed octagonal prisms whose long walls cross near their middles.
// For every face pair whose facets come within the bound, the gate must find a
// deep witness, or the union is refused. At least one such pair has no witness
// among its contacting facets' corners, edge midpoints and centroids on either
// side (deepWitnessInside): those seven points sit at the far ends of facets
// 11 mm long. The walk from the pair's contact segments (meshbool.SpanWitness) proves it.
func TestProximityGateWalksPastCornerSamples(t *testing.T) {
	t.Parallel()
	doc := New()
	joint := r3.Vec{Z: 10}
	lean := r3.Vec{X: math.Sin(math.Pi / 6), Z: math.Cos(math.Pi / 6)}
	a := internalOctagonPrism(t, doc, internalAxisFrame(t, r3.Vec{}, r3.Vec{Z: 1}), 11, 1.5, 0)
	b := internalOctagonPrism(t, doc, internalAxisFrame(t, joint.Sub(lean.Scale(1.5)), lean), 11.5, 1.2, 0.2)

	bmA, bmB, pairs := closeGateFacePairs(t, a, b)
	require.NotEmpty(t, pairs, `the walls come within the bound, so the gate has face pairs to decide`)
	walkOnly := 0
	for _, fp := range pairs {
		sampled, err := meshbool.DeepWitnessInside(t.Context(), bmA, fp.nc.CloseA, bmB, fp.slack)
		require.NoError(t, err)
		if !sampled {
			sampled, err = meshbool.DeepWitnessInside(t.Context(), bmB, fp.nc.CloseB, bmA, fp.slack)
			require.NoError(t, err)
		}
		walked, err := meshbool.SpanWitness(t.Context(), bmA, bmB, fp.nc.Spans, fp.slack)
		require.NoError(t, err)
		require.True(t, sampled || walked, `every close face pair of a genuine crossing is proven deep`)
		if !sampled {
			walkOnly++
		}
	}
	require.Positive(t, walkOnly, `some face pair is refused by the fixed samples alone and proven only by the walk`)
}

// TestProximityGateWalkAdmitsNoGraze pins that the segment walk certifies no
// witness on a pair that only touches or crosses no deeper than the bound. The
// fixture is TestUnionRefusesPlacedPrismGrazingAlongAnEdge's: an octagonal
// prism lying on its side, turned about Z, whose lowest long edge rests on a
// slab top (touching) or one ulp below it (shallow). Its wall faces meet the
// slab top along contact segments, and the walk steps across the slab top from
// them. In the touching case those points are far deeper than the bound from
// the prism's boundary but outside it, so the exact parity must reject each. In
// the shallow case the prism's edge vertices lie inside the slab no deeper
// than the bound, so the certified depth must reject each.
func TestProximityGateWalkAdmitsNoGraze(t *testing.T) {
	t.Parallel()
	const r = 0.25
	testcases := []struct {
		Name  string
		AxisZ float64
		Depth float64
	}{
		{Name: "touching", AxisZ: 1 + r},
		{Name: "crossing shallower than the bound", AxisZ: 1 + r - 0x1p-52, Depth: 0x1p-52},
	}
	for _, tc := range testcases {
		t.Run(tc.Name, func(t *testing.T) {
			t.Parallel()
			doc := New()
			a := internalBoxBody(t, doc, 0, 0, 20, 20, 1)
			c, s := math.Cos(0.3), math.Sin(0.3)
			frame, err := r3.FromBasis(r3.Basis{EX: r3.Vec{X: -s, Y: c}, EY: r3.Vec{Z: 1}, EZ: r3.Vec{X: c, Y: s}}, r3.Vec{X: 4, Y: 5, Z: tc.AxisZ})
			require.NoError(t, err)
			b := internalOctagonPrism(t, doc, frame, 10, r, 0)

			bmA, bmB, pairs := closeGateFacePairs(t, a, b)
			walked := 0
			for _, fp := range pairs {
				if len(fp.nc.Spans) == 0 {
					continue
				}
				walked++
				require.Greater(t, fp.slack, tc.Depth, `the held facets cross no deeper than the bound`)
				deep, err := meshbool.SpanWitness(t.Context(), bmA, bmB, fp.nc.Spans, fp.slack)
				require.NoError(t, err)
				require.False(t, deep, `the walk certifies no point deeper than the bound inside the other solid`)
				deep, err = meshbool.ProvenDepthExceeds(t.Context(), bmA, fp.nc.CloseA, bmB, fp.nc.CloseB, fp.nc.Spans, fp.slack)
				require.NoError(t, err)
				require.False(t, deep)
			}
			require.Positive(t, walked, `the edge meets the slab top along contact segments the walk starts from`)
		})
	}
}

// tinyOffset is a displacement far below one ulp at the coordinates below, so
// two exact points a tinyOffset apart round to the SAME float64 vertex — which
// is what makes the stitcher weld them, and the facets they span collapse.
func tinyOffset() *big.Rat {
	return new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Exp(big.NewInt(10), big.NewInt(20), nil))
}

// xptFromRat builds an exact point directly from three big.Rat coordinates —
// used only where a test needs sub-ulp control production's float-only entry
// point (proofarith.XptOf) cannot express, over one shared homogeneous denominator the
// same way proofarith.XhpOf lifts a float vertex.
func xptFromRat(x, y, z *big.Rat) proofarith.Xpt {
	dx, dy, dz := x.Denom(), y.Denom(), z.Denom()
	return proofarith.Xpt{
		X: new(big.Int).Mul(x.Num(), new(big.Int).Mul(dy, dz)),
		Y: new(big.Int).Mul(y.Num(), new(big.Int).Mul(dx, dz)),
		Z: new(big.Int).Mul(z.Num(), new(big.Int).Mul(dx, dy)),
		W: new(big.Int).Mul(dx, new(big.Int).Mul(dy, dz)),
	}
}

// xat is an exact point from whole millimetres, optionally nudged by a
// sub-ulp offset on one axis.
func xat(x, y, z float64, nudge int) proofarith.Xpt {
	rx, ry, rz := polynomial.MustRatOf(x), polynomial.MustRatOf(y), polynomial.MustRatOf(z)
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
func splitApexTetra() []meshbool.KeptFacet {
	a, b, c := proofarith.XptOf(r3.NewVec(0, 0, 0)), proofarith.XptOf(r3.NewVec(10, 0, 0)), proofarith.XptOf(r3.NewVec(0, 10, 0))
	d1 := proofarith.XptOf(r3.NewVec(2, 2, 9))
	d2 := xat(2, 2, 9, 0)
	return []meshbool.KeptFacet{
		{V: [3]proofarith.Xpt{a, c, b}},
		{V: [3]proofarith.Xpt{a, b, d1}},
		{V: [3]proofarith.Xpt{b, c, d2}},
		{V: [3]proofarith.Xpt{c, a, d2}},
		{V: [3]proofarith.Xpt{b, d2, d1}},
		{V: [3]proofarith.Xpt{a, d1, d2}},
	}
}

// subUlpTetra is a closed tetra whose four vertices all round to the SAME
// float64 vertex: every one of its facets collapses under the weld, so the
// whole component is welded out of existence.
func subUlpTetra() []meshbool.KeptFacet {
	p := proofarith.XptOf(r3.NewVec(40, 40, 40))
	q, r, s := xat(40, 40, 40, 0), xat(40, 40, 40, 1), xat(40, 40, 40, 2)
	return []meshbool.KeptFacet{
		{V: [3]proofarith.Xpt{p, r, q}},
		{V: [3]proofarith.Xpt{p, q, s}},
		{V: [3]proofarith.Xpt{q, r, s}},
		{V: [3]proofarith.Xpt{r, p, s}},
	}
}

func TestStitchRefusesAWeldedAwayComponent(t *testing.T) {
	t.Parallel()
	// The whole tiny component rounds onto one float vertex, so every facet of
	// it collapses and it disappears from the held mesh — a lump gone from the
	// body, with its volume, its place in Lumps() and its reach in the bounds
	// box. The closure audit does not see it: the component that remains still
	// closes. Nothing downstream would report it either, so the stitcher
	// refuses here.
	_, err := meshbool.StitchFacetsContext(t.Context(), append(splitApexTetra(), subUlpTetra()...))
	require.ErrorIs(t, err, ErrUnsupported)

	// It is the SURVIVING company that made the loss silent: a result that is
	// nothing but the tiny component has no extent left at all, and the stitcher
	// already refused that outright.
	_, err = meshbool.StitchFacetsContext(t.Context(), subUlpTetra())
	require.ErrorIs(t, err, ErrBooleanFailed)
}

func TestStitchChargesTheFacetsTheWeldDrops(t *testing.T) {
	t.Parallel()
	// A collapse INSIDE a surviving component is not refused — it is an edge
	// contraction, and the surface that remains is the tetra. But the two facets
	// it drops were not zero-area before the weld, and both of the things they
	// carried are charged: their swept volume, against the PRE-ROUND surface
	// (preArea), and the area the held mesh can no longer report (dropArea).
	got, err := meshbool.StitchFacetsContext(t.Context(), splitApexTetra())
	require.NoError(t, err)
	require.Len(t, got.Tris, 4, `the two bridging facets collapse; the tetra survives`)

	// The surviving surface is read through the same estimator PreArea uses, so
	// the comparison isolates which surface the charge is taken against.
	held := proofbound.PerturbedAreaUpper(got.Verts, got.Tris, got.Round)
	require.Greater(t, got.PreArea, held, `the rounding is charged against the surface it acted on, not the one that survived it`)
	require.Positive(t, got.DropArea, `the dropped facets' own area is charged`)
	require.Positive(t, got.Round)
	// The volume the weld can have moved is bounded by the displacement times
	// the pre-round area — a strictly larger charge than the held mesh's own.
	require.Greater(t, proofbound.SweptVolumeAllow(got.Round, got.PreArea), proofbound.SweptVolumeAllow(got.Round, held))
}

func TestBooleanRoundingUnderflowKeepsProofPositive(t *testing.T) {
	// The sloped edge intersects y=scale/2 at x=SmallestNonzeroFloat64/2.
	// That exact crossing rounds to zero, but still moves the held vertex.
	// Scaling the solids makes the positive swept volume flush to zero as float64.
	const scale = 1.0 / 64
	doc := New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := [3]*sketch.Point{
		s.CreatePoint(0, 0),
		s.CreatePoint(math.SmallestNonzeroFloat64, scale),
		s.CreatePoint(scale, 0),
	}
	s.Fix(points[0])
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	wedge, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(10 * scale), Dir: Along})
	require.NoError(t, err)

	s2, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s2.CreateRectangle(-scale, 0.5*scale, scale, 1.5*scale)
	s2.Fix(rect.A)
	_, err = s2.Solve(t.Context())
	require.NoError(t, err)
	bar, err := doc.Extrude(s2, s2.Profiles()[0], Symmetric{D: units.Millimeters(5 * scale)})
	require.NoError(t, err)
	for _, operand := range []*Body{wedge, bar} {
		mesh, meshErr := tessellateContext(t.Context(), operand, units.Millimeters(1), VerifyAll)
		require.NoError(t, meshErr)
		require.Zero(t, mesh.bound)
		require.Zero(t, mesh.volSymDiff)
		if operand == wedge {
			require.Contains(t, mesh.vertices, r3.Vec{X: math.SmallestNonzeroFloat64, Y: scale})
		}
	}

	union, err := Union(t.Context(), wedge, bar)
	require.NoError(t, err)
	proof, ok := union.payload.(facetedPayload)
	require.True(t, ok)
	require.Contains(t, proof.verts, r3.Vec{Y: 0.5 * scale})
	require.Positive(t, proof.meshBound)
	require.Positive(t, proof.volSymDiff)
	volume, err := union.Volume()
	require.NoError(t, err)
	require.Positive(t, volume.Bound.Base())
	require.Equal(t, Approximate, volume.Exactness)
}

func TestStitchRoundingUnderflowKeepsPositiveBound(t *testing.T) {
	// One exact apex is a quarter of the smallest subnormal away from its
	// held float. The rational-to-float reading of that displacement is zero.
	subnormal := new(big.Rat).SetFloat64(math.SmallestNonzeroFloat64)
	offset := new(big.Rat).Quo(subnormal, big.NewRat(4, 1))
	a, b, c := proofarith.XptOf(r3.NewVec(0, 0, 0)), proofarith.XptOf(r3.NewVec(10, 0, 0)), proofarith.XptOf(r3.NewVec(0, 10, 0))
	d1 := proofarith.XptOf(r3.NewVec(2, 2, 9))
	d2 := xptFromRat(new(big.Rat).Add(big.NewRat(2, 1), offset), big.NewRat(2, 1), big.NewRat(9, 1))
	kept := []meshbool.KeptFacet{
		{V: [3]proofarith.Xpt{a, c, b}},
		{V: [3]proofarith.Xpt{a, b, d1}},
		{V: [3]proofarith.Xpt{b, c, d2}},
		{V: [3]proofarith.Xpt{c, a, d2}},
		{V: [3]proofarith.Xpt{b, d2, d1}},
		{V: [3]proofarith.Xpt{a, d1, d2}},
	}
	got, err := meshbool.StitchFacetsContext(t.Context(), kept)
	require.NoError(t, err)
	require.Len(t, got.Tris, 4)
	require.Positive(t, got.Round)
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(got.Round).Cmp(offset), 0)
	require.Positive(t, proofbound.SweptVolumeAllow(got.Round, got.PreArea))
	require.Positive(t, meshbool.PointRoundBound(d2, r3.NewVec(2, 2, 9)))
}

func TestFacetedMeasurementSumsEncloseSmallAllowances(t *testing.T) {
	const tiny = math.SmallestNonzeroFloat64
	t.Run("volume", func(t *testing.T) {
		gap := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 54))
		exactVolume := new(big.Rat).Add(big.NewRat(1, 1), gap)
		x := new(big.Rat).Mul(exactVolume, big.NewRat(6, 1))
		verts := []proofarith.Xpt{
			proofarith.XptOf(r3.Vec{}),
			xptFromRat(x, big.NewRat(0, 1), big.NewRat(0, 1)),
			proofarith.XptOf(r3.NewVec(0, 1, 0)),
			proofarith.XptOf(r3.NewVec(0, 0, 1)),
		}
		tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
		reading, gotVolume, err := meshVolumeMeasurement(t.Context(), verts, tris, tiny)
		require.NoError(t, err)
		require.Equal(t, 0, exactVolume.Cmp(gotVolume))
		wantBound := new(big.Rat).Add(gap, new(big.Rat).SetFloat64(tiny))
		require.GreaterOrEqual(t, new(big.Rat).SetFloat64(reading.Bound.Base()).Cmp(wantBound), 0)
	})

	t.Run("centroid", func(t *testing.T) {
		width := math.Ldexp(1, -52)
		verts := []r3.Vec{{X: 1}, {X: 1 + width}, {X: 1, Y: 1}, {X: 1, Z: 1}}
		tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
		payload := facetedPayload{
			verts: verts, vertexBound: make([]float64, len(verts)), tris: tris, src: []int{0, 1, 2, 3},
			groups:     []facetGroup{{planar: true}, {planar: true}, {planar: true}, {planar: true}},
			volSymDiff: tiny, dPair: 2, xform: r3.Identity(),
		}
		body, err := buildFacetedBody(t.Context(), New(), producerID(0), payload)
		require.NoError(t, err)
		require.Equal(t, 1.0, body.centroid.Value.X)
		trueCenterX := new(big.Rat).Add(big.NewRat(1, 1),
			new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 54)))
		round := proofbound.Radius3D(proofbound.RatAbsDiff(trueCenterX, body.centroid.Value.X))
		volume := new(big.Rat).Quo(new(big.Rat).SetFloat64(width), big.NewRat(6, 1))
		allowance := facetedCentroidAllowance(tiny, payload.dPair, volFloor(volume, tiny))
		wantBound := new(big.Rat).Add(new(big.Rat).SetFloat64(round), new(big.Rat).SetFloat64(allowance))
		require.GreaterOrEqual(t, new(big.Rat).SetFloat64(body.centroid.Bound.Base()).Cmp(wantBound), 0)
	})
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
	t.Parallel()
	type want struct {
		volume           float64
		cx, cy, cz       float64
		minX, minY, minZ float64
		maxX, maxY, maxZ float64
	}
	testcases := []struct {
		name string
		op   func(ctx context.Context, a, b *Body) (*Body, error)
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
			b, err = b.Placed(t.Context(), tr)
			require.NoError(t, err)

			var op meshbool.OperationKind
			switch tc.name {
			case "Union":
				op = meshbool.OpUnion
			case "Cut":
				op = meshbool.OpCut
			case "Intersect":
				op = meshbool.OpIntersect
			}
			eval, err := evaluateBoolean(t.Context(), op, a, b)
			require.NoError(t, err)
			require.NotNil(t, eval.audit)
			require.NotNil(t, eval.volumeRat)

			result, err := tc.op(t.Context(), a, b)
			require.NoError(t, err)

			volM, err := result.Volume()
			require.NoError(t, err)
			require.Equal(t, eval.volume, volM, `the body must publish the evaluator's exact measurement and bound`)
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
	t.Parallel()
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
// discBody, needed here because meshbool.BoolMesh and its prep helpers are unexported
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
// of apitest/boolean_test.go's washerBodySymmetric.
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
func buildCircularWasherMeshes(t testing.TB) (*meshbool.BoolMesh, *meshbool.BoolMesh) {
	t.Helper()
	doc := New()
	target := internalDiscBody(t, doc, 15, 10)
	tool := internalWasherBodySymmetric(t, doc, 8, 3, 11)
	tolMM, _, err := pairChordTolerance(t.Context(), target, tool)
	require.NoError(t, err)
	ma, err := tessellateContext(t.Context(), target, units.Millimeters(tolMM), VerifyAll)
	require.NoError(t, err)
	mb, err := tessellateContext(t.Context(), tool, units.Millimeters(tolMM), VerifyAll)
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
func requireSameTriContact(t *testing.T, exact, filtered meshbool.TriContact) {
	t.Helper()
	require.Equal(t, exact.Kind, filtered.Kind)
	if exact.Kind == meshbool.ContactNone {
		return
	}
	require.Zero(t, exact.P0.X.Cmp(filtered.P0.X))
	require.Zero(t, exact.P0.Y.Cmp(filtered.P0.Y))
	require.Zero(t, exact.P0.Z.Cmp(filtered.P0.Z))
	require.Zero(t, exact.P1.X.Cmp(filtered.P1.X))
	require.Zero(t, exact.P1.Y.Cmp(filtered.P1.Y))
	require.Zero(t, exact.P1.Z.Cmp(filtered.P1.Z))
	require.Equal(t, exact.P0OnA, filtered.P0OnA)
	require.Equal(t, exact.P1OnA, filtered.P1OnA)
	require.Equal(t, exact.P0OnB, filtered.P0OnB)
	require.Equal(t, exact.P1OnB, filtered.P1OnB)
	require.Equal(t, exact.EdgeA, filtered.EdgeA)
	require.Equal(t, exact.EdgeB, filtered.EdgeB)
	if exact.Sin2 == nil {
		require.Nil(t, filtered.Sin2)
		return
	}
	require.NotNil(t, filtered.Sin2)
	require.Zero(t, exact.Sin2.Cmp(filtered.Sin2))
}

// classifyPair runs one facet pair through the classifier with the early miss
// filter on or off, which is what the two equivalence tests below compare. The
// choice travels as an argument, so these tests decide nothing for any other
// test running beside them.
func classifyPair(ta, tb [3]r3.Vec, xta, xtb [3]proofarith.Xpt, na, nb proofarith.Xpt, useFilter bool) (meshbool.TriContact, error) {
	return meshbool.TriTriClassifyWithProjections(ta, tb, xta, xtb, na, nb, nil, nil, nil, nil, useFilter)
}

// TestTriTriClassifyFilterAgreesWithTheExactPath is fu158's own pin: every
// AABB-surviving facet pair of the disc/washer fixture must classify
// identically with meshbool.TriTriMissesFilter on and off. The filter changes no
// verdict, so this is what stands between "faster" and "a different answer"
// (.tmp/followup-tasks/fu158-tasks.md §6).
func TestTriTriClassifyFilterAgreesWithTheExactPath(t *testing.T) {
	t.Parallel()

	ma, mb := buildCircularWasherMeshes(t)
	pairs, nonNone := 0, 0
	for i := range ma.Tris {
		for j := range mb.Tris {
			if !meshbool.BoxesOverlap(ma.Boxes[i], mb.Boxes[j]) {
				continue
			}
			pairs++
			ta, tb := meshbool.TriCorners(ma, i), meshbool.TriCorners(mb, j)
			xta, xtb := meshbool.XtriCorners(ma, i), meshbool.XtriCorners(mb, j)
			na, nb := ma.Norms[i], mb.Norms[j]

			filtered, err := classifyPair(ta, tb, xta, xtb, na, nb, true)
			require.NoError(t, err)
			exact, err := classifyPair(ta, tb, xta, xtb, na, nb, false)
			require.NoError(t, err)

			requireSameTriContact(t, exact, filtered)
			if exact.Kind != meshbool.ContactNone {
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
// intervals — exactly where meshbool.TriSpanOnLine's own doc comment says it must
// abstain rather than resolve. Sharing an edge keeps the contact itself
// unambiguous (a positive-length segment along it), so the pair exercises the
// near-parallel path while still landing on a real answer.
func TestTriTriClassifyFilterAgreesAtAShallowDihedralAngle(t *testing.T) {
	t.Parallel()

	a := [3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 10, Y: 0, Z: 0}, {X: 5, Y: 10, Z: 0}}
	b := [3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 10, Y: 0, Z: 0}, {X: 5, Y: -10, Z: 1e-6}}
	xta := [3]proofarith.Xpt{proofarith.XptOf(a[0]), proofarith.XptOf(a[1]), proofarith.XptOf(a[2])}
	xtb := [3]proofarith.Xpt{proofarith.XptOf(b[0]), proofarith.XptOf(b[1]), proofarith.XptOf(b[2])}
	na := proofarith.Xcross(proofarith.Xsub(xta[1], xta[0]), proofarith.Xsub(xta[2], xta[0]))
	nb := proofarith.Xcross(proofarith.Xsub(xtb[1], xtb[0]), proofarith.Xsub(xtb[2], xtb[0]))

	filtered, err := classifyPair(a, b, xta, xtb, na, nb, true)
	require.NoError(t, err)
	exact, err := classifyPair(a, b, xta, xtb, na, nb, false)
	require.NoError(t, err)

	require.Equal(t, meshbool.ContactSegment, exact.Kind, `the shared edge is a real, unambiguous contact`)
	requireSameTriContact(t, exact, filtered)

	// The classifier's prepared plane signs must match its original adaptive
	// predicate, including copies and points exactly on a nearly shared plane.
	near := math.Ldexp(1, -40)
	for _, tc := range []struct {
		name string
		tri  [3]r3.Vec
	}{
		{name: `shared edge`, tri: b},
		{name: `copied facet`, tri: a},
		{name: `near coplanar`, tri: [3]r3.Vec{
			{X: 0, Y: 0, Z: 0}, {X: 10, Y: 0, Z: near}, {X: 5, Y: 10, Z: -near},
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			xtri := [3]proofarith.Xpt{proofarith.XptOf(tc.tri[0]), proofarith.XptOf(tc.tri[1]), proofarith.XptOf(tc.tri[2])}
			ntri := proofarith.Xcross(proofarith.Xsub(xtri[1], xtri[0]), proofarith.Xsub(xtri[2], xtri[0]))
			uncertain := 0
			for i := range 3 {
				_, certainA := proofarith.OrientSignFloat(a[0], a[1], a[2], tc.tri[i])
				_, certainB := proofarith.OrientSignFloat(tc.tri[0], tc.tri[1], tc.tri[2], a[i])
				if !certainA {
					uncertain++
				}
				if !certainB {
					uncertain++
				}
				require.Equal(t, proofarith.OrientSign(a[0], a[1], a[2], tc.tri[i]),
					proofarith.OrientSignPrepared(a[0], a[1], a[2], tc.tri[i], xta[0], xtri[i], na))
				require.Equal(t, proofarith.OrientSign(tc.tri[0], tc.tri[1], tc.tri[2], a[i]),
					proofarith.OrientSignPrepared(tc.tri[0], tc.tri[1], tc.tri[2], a[i], xtri[0], xta[i], ntri))
			}
			require.Positive(t, uncertain, `this case must exercise the exact fallback`)
		})
	}
}

// BenchmarkTriTriClassifyCircularPairs isolates fu158's fix from the rest of
// the mesh-boolean pipeline: it builds the disc/washer fixture's two
// boolMeshes once, harvests every AABB-surviving facet-pair index once, then
// classifies the whole corpus per iteration — the cost meshbool.TriTriMissesFilter
// exists to cut (docs/evaluator-design.md §9).
func BenchmarkTriTriClassifyCircularPairs(b *testing.B) {
	ma, mb := buildCircularWasherMeshes(b)
	type facetPair struct{ i, j int }
	var pairs []facetPair
	for i := range ma.Tris {
		for j := range mb.Tris {
			if meshbool.BoxesOverlap(ma.Boxes[i], mb.Boxes[j]) {
				pairs = append(pairs, facetPair{i: i, j: j})
			}
		}
	}
	require.NotEmpty(b, pairs, `the AABB-surviving corpus must be non-empty`)

	for b.Loop() {
		for _, p := range pairs {
			ta, tb := meshbool.TriCorners(ma, p.i), meshbool.TriCorners(mb, p.j)
			_, err := meshbool.TriTriClassify(ta, tb, meshbool.XtriCorners(ma, p.i), meshbool.XtriCorners(mb, p.j), ma.Norms[p.i], mb.Norms[p.j])
			if err != nil {
				b.Fatal(err)
			}
		}
	}
}

func TestFacetFaceIndicesMapsConsistentFaces(t *testing.T) {
	t.Parallel()
	f0, f1 := &Face{}, &Face{}
	got, err := facetFaceIndices(t.Context(), []*Face{f0, f1}, []*Face{f1, f0, f1})
	require.NoError(t, err)
	require.Equal(t, []int{1, 0, 1}, got,
		`each facet must map to its face's index in the built body's Faces() order`)
}

func TestFacetFaceIndicesRejectsUnmappedFacet(t *testing.T) {
	t.Parallel()
	f0, f1 := &Face{}, &Face{}
	orphan := &Face{} // a face absent from the built body's Faces()
	_, err := facetFaceIndices(t.Context(), []*Face{f0, f1}, []*Face{f0, orphan})
	// Without the miss guard, a Go map lookup yields the zero value 0 and the
	// facet is silently attributed to face 0; the guard turns that invariant
	// break into an error instead.
	require.ErrorIs(t, err, ErrBooleanFailed)
}

func TestFacetedPlacementRebuildsCachedDiameter(t *testing.T) {
	t.Parallel()
	doc := New()
	body, err := buildFacetedBody(t.Context(), doc, producerID(0), facetedPayload{
		verts: []r3.Vec{
			r3.NewVec(0, 0, 0),
			r3.NewVec(3, 0, 0),
			r3.NewVec(0, 4, 0),
			r3.NewVec(0, 0, 5),
		},
		vertexBound: make([]float64, 4),
		tris:        [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}},
		src:         []int{0, 1, 2, 3},
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

	placed, err := before.placed(t.Context(), doc, producerID(1), placement)
	require.NoError(t, err)
	after := placed.payload.(facetedPayload)
	want, ok := diameter.Points(after.verts)
	require.True(t, ok)
	require.NotEqual(t, before.diameter, want, "the placement must make a stale cached diameter observable")
	require.Equal(t, want, after.diameter)
}

func TestBooleanProofBoundsEncloseExactTermSums(t *testing.T) {
	// Each small term is a valid positive bound, even when adding it to one
	// rounds back to one. The certificate must enclose the exact sum.
	const tiny = math.SmallestNonzeroFloat64
	volume, area := booleanProofBounds(1, tiny, tiny, 1, tiny, tiny)
	for _, tc := range []struct {
		got       float64
		tinyTerms int64
	}{
		{volume, 2},
		{area, 2},
	} {
		want := new(big.Rat).Add(big.NewRat(1, 1),
			new(big.Rat).Mul(big.NewRat(tc.tinyTerms, 1), new(big.Rat).SetFloat64(tiny)))
		require.GreaterOrEqual(t, proofarith.FloatRat(tc.got).Cmp(want), 0)
	}
}

func TestFacetedPlacementEnclosesExactPriorAndMotionBounds(t *testing.T) {
	// The Boolean producer supplies an exact held box union. Giving its
	// certificate a tiny extra allowance remains sound, and makes a lost
	// low-order term in the placement's two sums observable.
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	shift, err := r3.Translation(r3.NewVec(5, 5, 5))
	require.NoError(t, err)
	b, err = b.Placed(t.Context(), shift)
	require.NoError(t, err)
	union, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	before, ok := union.payload.(facetedPayload)
	require.True(t, ok)
	require.Zero(t, before.meshBound)
	require.Zero(t, before.volSymDiff)
	const tiny = math.SmallestNonzeroFloat64
	before.meshBound, before.volSymDiff = tiny, tiny

	move, err := r3.Translation(r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	placed, err := before.placed(t.Context(), doc, producerID(0), move)
	require.NoError(t, err)
	after := placed.payload.(facetedPayload)
	maxInput := 0.0
	for _, v := range before.verts {
		maxInput = math.Max(maxInput, math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z))))
	}
	allow := proofbound.RigidRoundAllow(maxInput, 1)
	areaUpper, err := proofbound.PerturbedAreaUpperContext(t.Context(), after.verts, after.tris, allow)
	require.NoError(t, err)
	roundVol := proofbound.SweptVolumeAllow(allow, areaUpper)
	for _, tc := range []struct{ got, increment float64 }{
		{after.meshBound, allow},
		{after.volSymDiff, roundVol},
	} {
		require.Positive(t, tc.increment)
		want := new(big.Rat).Add(new(big.Rat).SetFloat64(tiny), new(big.Rat).SetFloat64(tc.increment))
		require.GreaterOrEqual(t, proofarith.FloatRat(tc.got).Cmp(want), 0)
	}
}

// TestBooleanComposesTheOperandsOwnSymmetricDifferenceProofs is
// docs/tessellation-reach-design.md §3's boolean half: the result's
// occupied-volume bound is composed from each operand mesh's OWN volSymDiff
// proof, never from the `Mesh.Bound × held area` product
// docs/tessellation-design.md §11 forbids.
func TestBooleanComposesTheOperandsOwnSymmetricDifferenceProofs(t *testing.T) {
	t.Parallel()
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 20, 20, 10)
	disc := internalDiscBody(t, doc, 4, 10)
	tr, err := r3.Translation(r3.NewVec(10, 10, 5))
	require.NoError(t, err)
	pin, err := disc.Placed(t.Context(), tr)
	require.NoError(t, err)

	tolMM, _, err := pairChordTolerance(t.Context(), plate, pin)
	require.NoError(t, err)
	ma, err := tessellateContext(t.Context(), plate, units.Millimeters(tolMM), VerifyAll)
	require.NoError(t, err)
	mb, err := tessellateContext(t.Context(), pin, units.Millimeters(tolMM), VerifyAll)
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

	eval, err := evaluateBoolean(t.Context(), meshbool.OpUnion, plate, pin)
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
	t.Parallel()
	// An export-only mesh — one whose payload class has no occupied-volume proof
	// yet — is refused as a staging limit, never composed as a zero. Its
	// boundary audits all ran, so the refusal names the missing volume proof.
	_, err := operandSymDiff(&Mesh{volSymDiff: 17, symDiffOK: false, boundaryOK: true})
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "no proof of the volume")

	got, err := operandSymDiff(&Mesh{volSymDiff: 17, symDiffOK: true, boundaryOK: true})
	require.NoError(t, err)
	require.Equal(t, 17.0, got)

	// A mesh whose facet-contact audit never ran is refused ONE STEP EARLIER
	// and by its own cause: every homotopy the volume proof composes has that
	// audit as an antecedent (docs/tessellation-design.md §11), so blaming the
	// payload class here would misstate why this operand cannot be composed.
	_, err = operandSymDiff(&Mesh{volSymDiff: 17, symDiffOK: true, boundaryOK: false})
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "no facet-contact audit")
}

// The same gate over a REAL unverified mesh rather than a hand-written one: the
// tessellator's own VerifyNone output, through the boolean's own operand gate.
func TestOperandSymDiffRefusesARealUnverifiedMesh(t *testing.T) {
	t.Parallel()
	body := internalCylinderBody(t)
	tol := units.Millimeters(0.2)

	drawn, err := tessellateContext(t.Context(), body, tol, VerifyNone)
	require.NoError(t, err)
	require.False(t, drawn.BoundaryVerified())
	require.False(t, drawn.VolumeVerified())
	_, err = operandSymDiff(drawn)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "no facet-contact audit")

	// The same body at the same tolerance, proven, is admitted — so the
	// refusal is the level's doing and not the body's.
	proven, err := tessellateContext(t.Context(), body, tol, VerifyAll)
	require.NoError(t, err)
	sym, err := operandSymDiff(proven)
	require.NoError(t, err)
	require.Positive(t, sym)
}

func TestFacesOfMeshReadsTheMeshProofRecord(t *testing.T) {
	t.Parallel()
	stated, omitted := &Face{}, &Face{}
	m := &Mesh{
		triangles: [][3]int{{0, 1, 2}, {0, 2, 3}},
		source:    []*Face{stated, stated},
		faceBound: map[*Face]float64{stated: 0.25},
	}
	got, err := facesOfMesh(proofbound.NewWorkBudget(t.Context()), m)
	require.NoError(t, err)
	require.Len(t, got, 1)
	require.Equal(t, 0.25, got[0].delta, `the gate charges the face what the mesh proved for it`)
	require.Equal(t, []int{0, 1}, got[0].facets)

	// A source face the record omits is a broken evaluator, not a staged
	// capability, and never a zero the gate would read as "held exactly".
	m.source[1] = omitted
	_, err = facesOfMesh(proofbound.NewWorkBudget(t.Context()), m)
	require.ErrorIs(t, err, ErrBooleanFailed)
}
