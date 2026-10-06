package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"

	"github.com/stretchr/testify/require"
)

func TestFacetedAreaBoundCompositionRoundsOutward(t *testing.T) {
	t.Parallel()
	// Each edge's stored length plus its proven bound is a rational number.
	// Ten short edges disappear in the old float perimeter accumulation.
	short := math.Ldexp(1, -54)
	lengths := []float64{1, 1}
	for range 10 {
		lengths = append(lengths, short)
	}
	face := &Face{loops: []*Loop{{}}}
	exactPerimeter := new(big.Rat)
	oldPerimeter := 0.0
	for _, length := range lengths {
		bound := proofbound.ChainLengthBound(1, 0, length)
		face.loops[0].coedges = append(face.loops[0].coedges, coedge{edge: &Edge{
			length: length, lengthBound: bound,
		}})
		exactPerimeter.Add(exactPerimeter, new(big.Rat).SetFloat64(length))
		exactPerimeter.Add(exactPerimeter, new(big.Rat).SetFloat64(bound))
		oldPerimeter += length + bound
	}

	perimeter, err := facetedFacePerimeterUpper(face, proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(perimeter).Cmp(exactPerimeter), 0)

	meshBound := 0.25
	areaSlack := proofbound.SumSlop(12, short)
	areaBound := proofbound.AbsSumUpper(facetedAreaGeom(meshBound, perimeter, math.Inf(1)), areaSlack)
	required := new(big.Rat).Mul(new(big.Rat).SetFloat64(meshBound), exactPerimeter)
	required.Add(required, new(big.Rat).SetFloat64(areaSlack))
	oldBound := proofbound.UpRound(meshBound*oldPerimeter + areaSlack)
	require.Less(t, new(big.Rat).SetFloat64(oldBound).Cmp(required), 0,
		"the former perimeter and final rounding understate the required area allowance")
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(areaBound).Cmp(required), 0)

	// Body.Area counts each face's perimeter, including short later faces.
	bodyPerimeter := proofbound.AbsSumUpper(perimeter, short)
	exactBodyPerimeter := new(big.Rat).Add(exactPerimeter, new(big.Rat).SetFloat64(short))
	bodyRequired := new(big.Rat).Mul(new(big.Rat).SetFloat64(meshBound), exactBodyPerimeter)
	bodyRequired.Add(bodyRequired, new(big.Rat).SetFloat64(areaSlack))
	bodyBound := proofbound.AbsSumUpper(facetedAreaGeom(meshBound, bodyPerimeter, math.Inf(1)), areaSlack)
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(bodyBound).Cmp(bodyRequired), 0)

	// The exact zero remains exact, and every finite term stays finite.
	require.Zero(t, facetedAreaGeom(0, 0, math.Inf(1)))
	require.Zero(t, proofbound.UpRound(math.SmallestNonzeroFloat64*math.SmallestNonzeroFloat64),
		"the former final rounding cannot recover a positive product that underflows")
	require.Positive(t, facetedAreaGeom(math.SmallestNonzeroFloat64, math.SmallestNonzeroFloat64, math.Inf(1)))
	// The per-facet sum caps the perimeter term when it is the smaller.
	require.Equal(t, 0.5, facetedAreaGeom(meshBound, perimeter, 0.5))
	require.False(t, math.IsInf(areaBound, 0))
}

// vertexBoundTree is docs/faceted-vertex-bounds-design.md §6's tree with its
// first n branches unioned in: a 16-gon mitred trunk of circumradius 5 mm
// along (0,0,0)→(0,0,30)→(3,0,60), scaled 1 then 0.9, and tapering 16-gon
// mitred branches tilted 40° off the trunk axis, each root 2.6 mm out and
// 1.1 mm above the last, turned by the golden angle. It returns the trunk's
// own tessellation beside the result.
func vertexBoundTree(t *testing.T, n int) (*Mesh, *Body) {
	t.Helper()
	path := func(pts ...r3.Vec) *Path {
		segs := make([]PathSegment, 0, len(pts)-1)
		for _, p := range pts[1:] {
			segs = append(segs, LineTo{End: p})
		}
		p, err := NewPath(pts[0], segs...)
		require.NoError(t, err)
		return p
	}
	doc := New()
	ts, tp := vertexBoundPolygonSketch(t, 16, 5)
	tree, err := doc.Sweep(t.Context(), ts, tp,
		path(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 30), r3.NewVec(3, 0, 60)),
		WithMitredJoins(), WithSectionScale(units.Scalar(1), units.Scalar(0.9)))
	require.NoError(t, err)
	trunk, err := tessellateContext(t.Context(), tree, units.Millimeters(1), VerifyAll)
	require.NoError(t, err)
	bs, bp := vertexBoundPolygonSketch(t, 16, 1.5)
	golden := math.Pi * (3 - math.Sqrt(5))
	alpha := 40 * math.Pi / 180
	for k := range n {
		phi := golden * float64(k)
		u := r3.NewVec(math.Cos(phi), math.Sin(phi), 0)
		unit, err := doc.Sweep(t.Context(), bs, bp,
			path(r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 2.5), r3.NewVec(0.8, 0.3, 6), r3.NewVec(2.5, 0.6, 9.5)),
			WithMitredJoins(), WithSectionScale(units.Scalar(1.375/1.5), units.Scalar(1.2/1.5), units.Scalar(1/1.5)))
		require.NoError(t, err)
		d := u.Scale(math.Sin(alpha)).Add(r3.NewVec(0, 0, math.Cos(alpha)))
		fu := r3.NewVec(0, 0, 1).Cross(d)
		fu = fu.Scale(1 / fu.Len())
		frame, err := r3.NewFrame(u.Scale(2.6).Add(r3.NewVec(0, 0, 4+1.1*float64(k))), fu, d.Cross(fu))
		require.NoError(t, err)
		xf, err := r3.FromFrame(frame)
		require.NoError(t, err)
		branch, err := unit.Placed(t.Context(), xf)
		require.NoError(t, err)
		tree, err = Union(t.Context(), tree, branch)
		require.NoError(t, err)
	}
	return trunk, tree
}

// TestFacetedReadingsReportEachVertexsOwnBound is docs/faceted-vertex-bounds-
// design.md §4.2 over three unions of §6's tree. A trunk vertex no branch
// touches reports the trunk's own rounding gap as its Vertex bound, a face
// whose every vertex is such a trunk vertex reports the largest of them as its
// Faceted bound, and both sit under the body-wide bound the rims carry. The
// box bound is the six-extreme reading recomputed here from the restated
// mesh's own vertices and record.
func TestFacetedReadingsReportEachVertexsOwnBound(t *testing.T) {
	t.Parallel()
	trunk, body := vertexBoundTree(t, 3)
	fp, ok := body.payload.(facetedPayload)
	require.True(t, ok)
	trunkBeta, err := trunk.vertexBounds()
	require.NoError(t, err)
	own := map[r3.Vec]float64{}
	for i, v := range trunk.vertices {
		own[v] = trunkBeta[i]
	}

	kept, finer := 0, 0
	for _, v := range body.Vertices() {
		beta, ok := own[v.position]
		if !ok {
			continue
		}
		require.Equal(t, units.Millimeters(beta), v.Position().Bound, `trunk vertex %v reports its own bound`, v.position)
		kept++
		if beta < fp.meshBound {
			finer++
		}
	}
	require.Positive(t, kept)
	require.Positive(t, finer, `some untouched trunk vertex sits under the body-wide bound`)

	faceVerts := map[int][]int{}
	for i, fi := range fp.faceOf {
		faceVerts[fi] = append(faceVerts[fi], fp.tris[i][:]...)
	}
	faces := body.Faces()
	untouched, finerFaces := 0, 0
	for fi, vs := range faceVerts {
		want, all := 0.0, true
		for _, v := range vs {
			beta, ok := own[fp.verts[v]]
			if !ok {
				all = false
				break
			}
			want = max(want, beta)
		}
		if !all {
			continue
		}
		s, ok := faces[fi].Surface().(Faceted)
		require.True(t, ok)
		require.Equal(t, units.Millimeters(want), s.Bound, `an untouched trunk face reports its own facets' bound`)
		untouched++
		if want < fp.meshBound {
			finerFaces++
		}
	}
	require.Positive(t, untouched)
	require.Positive(t, finerFaces, `some untouched trunk face sits under the body-wide bound`)

	// The box: the true extreme on each side is at most the farthest any
	// vertex's facet bound reaches past the held one, and at least the
	// farthest any vertex's own bound proves a true point stands.
	mesh, err := tessellateContext(t.Context(), body, units.Millimeters(fp.meshBound), VerifyAll)
	require.NoError(t, err)
	reachOf := make([]float64, len(mesh.vertices))
	for _, tri := range mesh.triangles {
		d := max(mesh.vertexBound[tri[0]], mesh.vertexBound[tri[1]], mesh.vertexBound[tri[2]])
		for _, v := range tri {
			reachOf[v] = max(reachOf[v], d)
		}
	}
	box, err := body.Bounds()
	require.NoError(t, err)
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	worst := new(big.Rat)
	for axis := range 3 {
		lo, hi := rat(meshbool.CoordOf(box.Min, axis)), rat(meshbool.CoordOf(box.Max, axis))
		for i, v := range mesh.vertices {
			c := rat(meshbool.CoordOf(v, axis))
			for _, gap := range []*big.Rat{
				new(big.Rat).Sub(lo, new(big.Rat).Sub(c, rat(reachOf[i]))),
				new(big.Rat).Sub(new(big.Rat).Add(c, rat(reachOf[i])), hi),
			} {
				if gap.Cmp(worst) > 0 {
					worst = gap
				}
			}
		}
		// The held extreme is attained by a held vertex, so the nearest
		// proven true point inside it is that vertex's own bound or less.
		heldLo, heldHi := (*big.Rat)(nil), (*big.Rat)(nil)
		for i, v := range mesh.vertices {
			c := rat(meshbool.CoordOf(v, axis))
			if in := new(big.Rat).Add(c, rat(mesh.vertexBound[i])); heldLo == nil || in.Cmp(heldLo) < 0 {
				heldLo = in
			}
			if in := new(big.Rat).Sub(c, rat(mesh.vertexBound[i])); heldHi == nil || in.Cmp(heldHi) > 0 {
				heldHi = in
			}
		}
		for _, gap := range []*big.Rat{new(big.Rat).Sub(heldLo, lo), new(big.Rat).Sub(hi, heldHi)} {
			if gap.Cmp(worst) > 0 {
				worst = gap
			}
		}
	}
	e := 0.0
	if worst.Sign() > 0 {
		w, _ := worst.Float64()
		e = proofbound.ProvenUpRound(w)
	}
	require.Equal(t, units.Millimeters(proofbound.Radius3D(e)), box.Bound)
	require.Less(t, box.Bound.Mag(), proofbound.Radius3D(fp.meshBound), `the box reads each extreme's own bounds, not the body-wide one`)
}

// TestFacetedExtremeErrorCoversAnInnerVertexsReach is docs/faceted-vertex-
// bounds-design.md §4.2's box reading on a fixture whose largest-bound vertex
// sits inside the held box, 1e-3 mm short of its +X face, with a 1e-2 mm bound
// that reaches past it: the true point it stands for, 9e-3 mm beyond the held
// extreme, is one of the fixture's exact sources. The extreme vertex itself is
// exact, so only the inner vertex's reach can widen the +X error.
func TestFacetedExtremeErrorCoversAnInnerVertexsReach(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0),
		r3.NewVec(0, 0, 1),
		r3.NewVec(0, 0, 0),
		r3.NewVec(0.999, 0.5, 0.5),
	}
	beta := []float64{0, 0, 0, 0, 0.01}
	reach := []float64{0, 0, 0, 0, 0.01}
	exact := make([][3]*big.Rat, len(verts))
	for i, v := range verts {
		exact[i] = [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
	}
	// The inner vertex's true point: its own bound along +X, past the +X face.
	exact[4][0] = new(big.Rat).Add(new(big.Rat).SetFloat64(verts[4].X), new(big.Rat).SetFloat64(0.01))
	require.Equal(t, 1, exact[4][0].Cmp(big.NewRat(1, 1)), `the true point stands past the held +X face`)

	lo, hi := r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1)
	e, err := facetedExtremeError(proofbound.NewWorkBudget(t.Context()), verts, beta, reach, lo, hi)
	require.NoError(t, err)
	for _, p := range exact {
		for axis := range 3 {
			lower := new(big.Rat).Sub(new(big.Rat).SetFloat64(meshbool.CoordOf(lo, axis)), new(big.Rat).SetFloat64(e))
			upper := new(big.Rat).Add(new(big.Rat).SetFloat64(meshbool.CoordOf(hi, axis)), new(big.Rat).SetFloat64(e))
			require.True(t, p[axis].Cmp(lower) >= 0 && p[axis].Cmp(upper) <= 0,
				`exact source coordinate %s on axis %d lies within the published extreme error %g`, p[axis].FloatString(20), axis, e)
		}
	}
	require.Less(t, e, 0.01, `the error is the reach past the extreme, not the whole bound`)
}

// TestFacetedPlacementChargesEachVertexAtItsOwnMagnitude is docs/faceted-
// vertex-bounds-design.md §4.3 on a drilled 20 mm plate turned 30° about Z and
// lifted. Every vertex's bound grows by at least the rigid-motion rounding at
// its own largest coordinate magnitude, and the plate corner at the origin —
// which the motion moves to the exact translation, so the embedding check
// never moves it — grows by exactly that, strictly less than the body-wide
// magnitude would charge.
func TestFacetedPlacementChargesEachVertexAtItsOwnMagnitude(t *testing.T) {
	t.Parallel()
	doc := New()
	plate := internalBoxBody(t, doc, 0, 0, 20, 20, 8)
	tool := internalDiscBody(t, doc, 2, 30)
	lift, err := r3.Translation(r3.NewVec(10, 10, -10))
	require.NoError(t, err)
	tool, err = tool.Placed(t.Context(), lift)
	require.NoError(t, err)
	drilled, err := Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	src, ok := drilled.payload.(facetedPayload)
	require.True(t, ok)

	turn, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(30))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	motion, err := turn.Then(shift)
	require.NoError(t, err)
	placed, err := drilled.Placed(t.Context(), motion)
	require.NoError(t, err)
	next, ok := placed.payload.(facetedPayload)
	require.True(t, ok)
	require.Len(t, next.vertexBound, len(src.vertexBound))

	const maxTrans = 3.0
	magOf := func(v r3.Vec) float64 { return math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z))) }
	maxIn := 0.0
	for _, v := range src.verts {
		maxIn = math.Max(maxIn, magOf(v))
	}
	origin := -1
	for i, v := range src.verts {
		own := proofbound.AbsSumUpper(src.vertexBound[i], proofbound.RigidRoundAllow(magOf(v), maxTrans))
		require.GreaterOrEqual(t, next.vertexBound[i], own, `vertex %v is charged at least its own rounding`, v)
		if v == (r3.Vec{}) {
			origin = i
		}
	}
	require.GreaterOrEqual(t, origin, 0)
	require.Equal(t, r3.NewVec(1, 2, 3), next.verts[origin])
	require.Zero(t, src.vertexBound[origin], `the plate corner is held exactly`)
	require.Equal(t, proofbound.AbsSumUpper(0, proofbound.RigidRoundAllow(0, maxTrans)), next.vertexBound[origin])
	require.Less(t, next.vertexBound[origin], proofbound.AbsSumUpper(0, proofbound.RigidRoundAllow(maxIn, maxTrans)))
}
