package decad

import (
	"context"
	"math/big"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/r3"
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

// tinyOffset is a displacement far below one ulp at the coordinates below, so
// two exact points a tinyOffset apart round to the SAME float64 vertex — which
// is what makes the stitcher weld them, and the facets they span collapse.
func tinyOffset() *big.Rat {
	return new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Exp(big.NewInt(10), big.NewInt(20), nil))
}

// xat is an exact point from whole millimetres, optionally nudged by a
// sub-ulp offset on one axis.
func xat(x, y, z float64, nudge int) xpt {
	p := xpt{mustRatOf(x), mustRatOf(y), mustRatOf(z)}
	switch nudge {
	case 0:
		p.x = new(big.Rat).Add(p.x, tinyOffset())
	case 1:
		p.y = new(big.Rat).Add(p.y, tinyOffset())
	case 2:
		p.z = new(big.Rat).Add(p.z, tinyOffset())
	}
	return p
}

// splitApexTetra is a closed tetra A,B,C,D whose apex D is split into the edge
// D1–D2, a tinyOffset long: the two facets bridging that edge (B,D2,D1) and
// (A,D1,D2) collapse under the weld, while the component they belong to
// survives as the tetra. Every directed edge pairs with its reverse, so the
// exact closure audit passes before the rounding ever runs.
func splitApexTetra() []keptFacet {
	a, b, c := xpt{mustRatOf(0), mustRatOf(0), mustRatOf(0)}, xpt{mustRatOf(10), mustRatOf(0), mustRatOf(0)}, xpt{mustRatOf(0), mustRatOf(10), mustRatOf(0)}
	d1 := xpt{mustRatOf(2), mustRatOf(2), mustRatOf(9)}
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
	p := xpt{mustRatOf(40), mustRatOf(40), mustRatOf(40)}
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
