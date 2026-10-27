package facetedrecord

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// cubeTris are the outward triangles of the cube whose corner i sits at
// (i&1, i>>1&1, i>>2&1), two per side, sides in -Z, +Z, -Y, +Y, -X, +X order.
var cubeTris = [][3]int{
	{0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}, {0, 1, 5}, {0, 5, 4},
	{2, 6, 7}, {2, 7, 3}, {0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5},
}

// addCube appends the cube [lo, lo+side]^3 to m, one source group per side;
// inward reverses every winding, as a cavity's boundary faces.
func addCube(m *Mesh, lo, side float64, inward bool, producer provenance.Producer) {
	base := len(m.Verts)
	for i := range 8 {
		m.Verts = append(m.Verts, r3.NewVec(lo+side*float64(i&1), lo+side*float64(i>>1&1), lo+side*float64(i>>2&1)))
		m.VertexBound = append(m.VertexBound, 0)
	}
	group := len(m.Groups)
	for range 6 {
		m.Groups = append(m.Groups, Group{
			Origins: []provenance.Ref{{Producer: producer, Role: provenance.RoleBody}}, Planar: true,
		})
	}
	for i, t := range cubeTris {
		if inward {
			t[1], t[2] = t[2], t[1]
		}
		m.Tris = append(m.Tris, [3]int{base + t[0], base + t[1], base + t[2]})
		m.Src = append(m.Src, group+i/2)
	}
}

func assemble(t *testing.T, m Mesh) (*topo.Body, []int, float64) {
	t.Helper()
	body, faceOf, diameter, err := AssembleBody(t.Context(),
		topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}, m, nil, measurement.Measurement{}, nil)
	require.NoError(t, err)
	return body, faceOf, diameter
}

func TestAssembleBodyCube(t *testing.T) {
	t.Parallel()
	var m Mesh
	addCube(&m, 0, 2, false, 3)
	body, faceOf, diameter := assemble(t, m)

	require.True(t, body.Solid)
	require.Equal(t, provenance.Ref{Producer: 9, Role: provenance.RoleBody}, body.Origin)
	require.Equal(t, 8.0, body.Volume.Value.Base())
	require.Equal(t, measurement.Exact, body.Volume.Exactness)
	require.InDelta(t, 24, body.Area.Value.Base(), 1e-12)
	require.Positive(t, body.Area.Bound.Base(), `a float area sum is never exact`)
	require.Equal(t, r3.NewVec(1, 1, 1), body.Centroid.Value)
	require.Equal(t, r3.NewVec(0, 0, 0), body.Bounds.Min)
	require.Equal(t, r3.NewVec(2, 2, 2), body.Bounds.Max)
	require.InDelta(t, 2*math.Sqrt(3), diameter, 1e-12)

	require.Len(t, body.Lumps, 1)
	require.Len(t, body.Lumps[0].Shells, 1)
	require.False(t, body.Lumps[0].Shells[0].Void)
	faces := topobuild.Faces(body)
	require.Len(t, faces, 6)
	require.Len(t, topobuild.Edges(body), 12)
	require.Len(t, topobuild.Vertices(body), 8)
	for i, f := range faces {
		require.Same(t, body, f.Body)
		require.True(t, f.HeldPlanar)
		require.InDelta(t, 4, f.Area, 1e-12, `face %d`, i)
		require.Equal(t, surfacegeom.Faceted{Bound: units.Millimeters(0)}, f.Surface, `a zero-bound face reports a zero facet bound`)
		require.Len(t, f.Loops, 1)
		require.True(t, f.Loops[0].Outer)
		require.Len(t, f.Loops[0].CoEdges, 4)
	}
	for _, e := range topobuild.Edges(body) {
		require.Equal(t, 2.0, e.Length)
		require.Len(t, e.Faces, 2)
	}
	// Facet i belongs to side i/2, and the sides are the faces in facet order.
	require.Len(t, faceOf, len(m.Tris))
	for i, fi := range faceOf {
		require.Equal(t, i/2, fi)
	}
}

func TestAssembleBodyVoidShell(t *testing.T) {
	t.Parallel()
	var m Mesh
	addCube(&m, 0, 4, false, 3)
	addCube(&m, 1, 2, true, 4)
	body, faceOf, _ := assemble(t, m)

	require.Equal(t, 56.0, body.Volume.Value.Base())
	require.Equal(t, r3.NewVec(2, 2, 2), body.Centroid.Value)
	require.Len(t, body.Lumps, 1, `the cavity is a void of the one lump, not a lump of its own`)
	require.Len(t, body.Lumps[0].Shells, 2)
	outer, cavity := body.Lumps[0].Shells[0], body.Lumps[0].Shells[1]
	require.False(t, outer.Void)
	require.True(t, cavity.Void)
	require.Len(t, outer.Faces, 6)
	require.Len(t, cavity.Faces, 6)
	require.Len(t, topobuild.Faces(body), 12)
	require.Len(t, topobuild.Edges(body), 24)
	for _, f := range cavity.Faces {
		require.Equal(t, provenance.Producer(4), f.Origins[0].Producer)
	}
	require.Len(t, faceOf, 24)
}

func TestAssembleBodyRefusesMalformedPayload(t *testing.T) {
	t.Parallel()
	var cube Mesh
	addCube(&cube, 0, 2, false, 3)
	env := topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}
	for _, tc := range []struct {
		name, text string
		mutate     func(m *Mesh)
	}{
		{"short vertex bounds", "a faceted payload carries 7 vertex bounds for 8 vertices",
			func(m *Mesh) { m.VertexBound = m.VertexBound[:7] }},
		{"source past the groups", "a facet names no source group",
			func(m *Mesh) { m.Src = append([]int{len(m.Groups)}, m.Src[1:]...) }},
		{"empty mesh", "the held boundary has no usable diameter",
			func(m *Mesh) { *m = Mesh{} }},
	} {
		m := cube
		tc.mutate(&m)
		_, _, _, err := AssembleBody(t.Context(), env, m, nil, measurement.Measurement{}, nil)
		require.ErrorContains(t, err, tc.text, tc.name)
	}
}

// AssembleBody audits the mesh beside its diameter when no audit is supplied.
// The body equals the one assembled from the same audit supplied, which takes
// the serial path, field for field.
func TestAssembleBodyAuditBesideDiameterMatchesSuppliedAudit(t *testing.T) {
	t.Parallel()
	var m Mesh
	addCube(&m, 0.1, math.Sqrt2*3, false, 3)
	addCube(&m, 1.3, math.Pi/2, true, 4)
	env := func() topo.Env { return topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}} }
	got, gotFaceOf, gotDiameter, err := AssembleBody(t.Context(), env(), m, nil, measurement.Measurement{}, nil)
	require.NoError(t, err)
	audit, err := facetproof.AuditFacetedMesh(t.Context(), m.Verts, m.Tris)
	require.NoError(t, err)
	want, wantFaceOf, wantDiameter, err := AssembleBody(t.Context(), env(), m, audit, measurement.Measurement{}, nil)
	require.NoError(t, err)

	require.Equal(t, math.Float64bits(wantDiameter), math.Float64bits(gotDiameter))
	require.Equal(t, wantFaceOf, gotFaceOf)
	require.Equal(t, want.Volume, got.Volume)
	require.Equal(t, want.Area, got.Area)
	require.Equal(t, want.Centroid, got.Centroid)
	require.Equal(t, want.Bounds, got.Bounds)
	require.Len(t, got.Lumps, len(want.Lumps))
	wantFaces, gotFaces := topobuild.Faces(want), topobuild.Faces(got)
	require.Len(t, gotFaces, len(wantFaces))
	for i := range wantFaces {
		require.Equal(t, wantFaces[i].Area, gotFaces[i].Area, "face %d area", i)
		require.Equal(t, wantFaces[i].AreaBound, gotFaces[i].AreaBound, "face %d area bound", i)
		require.Equal(t, wantFaces[i].Surface, gotFaces[i].Surface, "face %d surface", i)
	}
}

// A serial assembly never audits a mesh whose diameter fails. The audit that
// runs beside the diameter holds back its panic: a failed diameter reports
// its own error, and a good one audits again on the caller, which panics
// there.
func TestAssembleBodyHoldsBackAuditPanic(t *testing.T) {
	t.Parallel()
	var cube Mesh
	addCube(&cube, 0, 2, false, 3)
	env := topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}

	// The facets close up but name vertices the mesh does not hold, so the
	// audit panics on its exact lifts.
	noVerts := cube
	noVerts.Verts, noVerts.VertexBound = nil, nil
	body, _, _, err := AssembleBody(t.Context(), env, noVerts, nil, measurement.Measurement{}, nil)
	require.ErrorContains(t, err, "the held boundary has no usable diameter")
	require.Nil(t, body)

	shifted := cube
	shifted.Tris = make([][3]int, len(cube.Tris))
	for i, tri := range cube.Tris {
		shifted.Tris[i] = [3]int{tri[0] + 8, tri[1] + 8, tri[2] + 8}
	}
	require.Panics(t, func() {
		_, _, _, _ = AssembleBody(t.Context(), env, shifted, nil, measurement.Measurement{}, nil)
	})
}

func TestFacetedAreaBoundCompositionRoundsOutward(t *testing.T) {
	t.Parallel()
	// Each edge's stored length plus its proven bound is a rational number.
	// Ten short edges disappear in the old float perimeter accumulation.
	short := math.Ldexp(1, -54)
	lengths := []float64{1, 1}
	for range 10 {
		lengths = append(lengths, short)
	}
	face := &topo.Face{Loops: []*topo.Loop{{}}}
	exactPerimeter := new(big.Rat)
	oldPerimeter := 0.0
	for _, length := range lengths {
		bound := proofbound.ChainLengthBound(1, 0, length)
		face.Loops[0].CoEdges = append(face.Loops[0].CoEdges, topo.CoEdge{Edge: &topo.Edge{
			Length: length, LengthBound: bound,
		}})
		exactPerimeter.Add(exactPerimeter, new(big.Rat).SetFloat64(length))
		exactPerimeter.Add(exactPerimeter, new(big.Rat).SetFloat64(bound))
		oldPerimeter += length + bound
	}

	perimeter, err := facePerimeterUpper(face, proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(perimeter).Cmp(exactPerimeter), 0)

	meshBound := 0.25
	areaSlack := proofbound.SumSlop(12, short)
	areaBound := proofbound.AbsSumUpper(facetproof.FacetedAreaGeom(meshBound, perimeter, math.Inf(1)), areaSlack)
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
	bodyBound := proofbound.AbsSumUpper(facetproof.FacetedAreaGeom(meshBound, bodyPerimeter, math.Inf(1)), areaSlack)
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(bodyBound).Cmp(bodyRequired), 0)

	// The exact zero remains exact, and every finite term stays finite.
	require.Zero(t, facetproof.FacetedAreaGeom(0, 0, math.Inf(1)))
	require.Zero(t, proofbound.UpRound(math.SmallestNonzeroFloat64*math.SmallestNonzeroFloat64),
		"the former final rounding cannot recover a positive product that underflows")
	require.Positive(t, facetproof.FacetedAreaGeom(math.SmallestNonzeroFloat64, math.SmallestNonzeroFloat64, math.Inf(1)))
	// The per-facet sum caps the perimeter term when it is the smaller.
	require.Equal(t, 0.5, facetproof.FacetedAreaGeom(meshBound, perimeter, 0.5))
	require.False(t, math.IsInf(areaBound, 0))
}
