package apitest_test

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func coedgeBox(t *testing.T) *decad.Body {
	t.Helper()
	s, p := plateSketch(t)
	body, err := decad.Extrude(t.Context(), decad.New(), s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func TestLoopCoEdgesMatchEdgesCompatibilityView(t *testing.T) {
	t.Parallel()
	body := coedgeBox(t)

	for _, face := range body.Faces() {
		for _, loop := range face.Loops() {
			directed := loop.CoEdges()
			undirected := loop.Edges()
			require.Len(t, directed, len(undirected))
			require.NotEmpty(t, directed)
			for i, use := range directed {
				require.Same(t, undirected[i], use.Edge())
			}
		}
	}
}

func TestLoopCoEdgesFollowDirectedBoundary(t *testing.T) {
	t.Parallel()
	body := coedgeBox(t)
	uses := make(map[*decad.Edge][]decad.CoEdge)
	sawReverse := false

	for _, face := range body.Faces() {
		for _, loop := range face.Loops() {
			directed := loop.CoEdges()
			require.NotEmpty(t, directed)
			for i, use := range directed {
				require.Same(t, use.End(), directed[(i+1)%len(directed)].Start())
				if use.IsForward() {
					require.Same(t, use.Edge().Start(), use.Start())
					require.Same(t, use.Edge().End(), use.End())
				} else {
					sawReverse = true
					require.Same(t, use.Edge().End(), use.Start())
					require.Same(t, use.Edge().Start(), use.End())
				}
				uses[use.Edge()] = append(uses[use.Edge()], use)
			}
		}
	}

	require.True(t, sawReverse, `the box exposes edge uses that oppose global edge orientation`)
	require.Len(t, uses, len(body.Edges()))
	for _, edge := range body.Edges() {
		require.Len(t, uses[edge], 2, `each manifold edge has two loop uses`)
	}
}

func TestLoopCoEdgesReturnImmutableView(t *testing.T) {
	t.Parallel()
	loop := coedgeBox(t).Faces()[0].Loops()[0]
	directed := loop.CoEdges()
	undirected := loop.Edges()
	require.NotEmpty(t, directed)

	wantEdge := directed[0].Edge()
	wantStart := directed[0].Start()
	wantEnd := directed[0].End()
	wantForward := directed[0].IsForward()
	directed[0] = decad.CoEdge{}
	undirected[0] = nil

	got := loop.CoEdges()[0]
	require.Same(t, wantEdge, got.Edge())
	require.Same(t, wantStart, got.Start())
	require.Same(t, wantEnd, got.End())
	require.Equal(t, wantForward, got.IsForward())
	require.Same(t, wantEdge, loop.Edges()[0])
}

// TestNURBSSurfaceReportsKindNURBS pins docs/spline-design.md §7: a
// NURBSSurface is a tagged, opaque Surface variant reporting the existing
// KindNURBS discriminant, carrying no exported geometry of its own.
func TestNURBSSurfaceReportsKindNURBS(t *testing.T) {
	t.Parallel()
	var s decad.Surface = decad.NURBSSurface{}
	require.Equal(t, decad.KindNURBS, s.Kind())
}

// TestNURBSCurveSealsIntoCurve pins docs/spline-design.md §7: a NURBSCurve is
// NURBSSurface's one-dimensional analog. Curve is sealed by its marker method
// alone and declares no Kind, so assigning a NURBSCurve to a Curve variable
// is the whole of what the variant publishes.
func TestNURBSCurveSealsIntoCurve(t *testing.T) {
	t.Parallel()
	var c decad.Curve = decad.NURBSCurve{}
	require.NotNil(t, c)
}

// requireSolidTopologyBaseline asserts the sheet-body-vocabulary readings
// that hold on every solid this evaluator builds today (docs/surface-design.md
// §2.1, §2.2): the body reports BodySolid without any builder ever writing
// `kind: BodySolid` — the seven &Body{...} literals rely on BodySolid being
// the iota zero — every edge bounds exactly two faces and none is free, and
// the body's single shell is neither open nor void.
func requireSolidTopologyBaseline(t *testing.T, body *decad.Body, wantFaces, wantEdges int) {
	t.Helper()
	require.Equal(t, decad.BodySolid, body.Kind())
	require.True(t, body.IsSolid())
	require.Len(t, body.Faces(), wantFaces)
	edges := body.Edges()
	require.Len(t, edges, wantEdges)
	for _, e := range edges {
		require.Len(t, e.Faces(), 2, `a solid's edge bounds exactly two faces`)
		require.False(t, e.IsFree(), `a solid has no free edge`)
	}
	require.Len(t, body.Lumps(), 1)
	shells := body.Shells()
	require.Len(t, shells, 1)
	require.False(t, shells[0].IsOpen(), `a solid shell has no free edge to be open over`)
	require.False(t, shells[0].IsVoid(), `an outer shell bounds no internal cavity`)
}

// TestSolidBodiesReportSolidKindAndNoFreeBoundary pins docs/surface-design.md
// §2.1's Table K first row and §2.2's Edge.IsFree/Shell.IsOpen readings
// against three solids the evaluator already builds: a box extrude, a holed
// extrude and a full-revolution annulus. None of today's builders produces a
// free edge or an open shell, so every one of these reads false — the
// negative space Table K and §2.2 both require before a later PR's
// WithSurfaceResult can ever make it read true.
func TestSolidBodiesReportSolidKindAndNoFreeBoundary(t *testing.T) {
	t.Parallel()

	t.Run("CoedgeBox", func(t *testing.T) {
		t.Parallel()
		body := coedgeBox(t)
		requireSolidTopologyBaseline(t, body, 6, 12)

		vol, err := body.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, vol.Exactness)
		gotVol, err := vol.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.Equal(t, 60000.0, gotVol)

		box, err := body.Bounds()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, box.Exactness)
		require.Equal(t, r3.NewVec(0, 0, 0), box.Min)
		require.Equal(t, r3.NewVec(100, 60, 10), box.Max)
	})

	t.Run("HoledExtrude", func(t *testing.T) {
		t.Parallel()
		body := holePlateBody(t)
		requireSolidTopologyBaseline(t, body, 7, 14)

		// A 100x60x8 plate less a diameter-20 through hole: the circle's
		// area is irrational, so the volume is Approximate, never Exact.
		vol, err := body.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, vol.Exactness)
		gotVol, err := vol.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.InDelta(t, (100*60-math.Pi*100)*8, gotVol, 1e-6)

		box, err := body.Bounds()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, box.Exactness, `the plate's box is the extrude's own recorded extents`)
		require.Equal(t, r3.NewVec(0, 0, 0), box.Min)
		require.Equal(t, r3.NewVec(100, 60, 8), box.Max)
	})

	t.Run("FullRevolve", func(t *testing.T) {
		t.Parallel()
		body := annularRevolveBody(t)
		requireSolidTopologyBaseline(t, body, 4, 4)

		// A full revolution of the v-in-[5,15], u-in-[0,10] rectangle about
		// the u axis: a tube of outer radius 15, inner radius 5, height 10.
		vol, err := body.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, vol.Exactness)
		gotVol, err := vol.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.InDelta(t, math.Pi*(15*15-5*5)*10, gotVol, 1e-6)

		box, err := body.Bounds()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, box.Exactness)
		require.Equal(t, r3.NewVec(0, -15, -15), box.Min)
		require.Equal(t, r3.NewVec(10, 15, 15), box.Max)
	})
}

// TestEdgeVerticesAreTheBodyVertices pins vertex identity across the topology
// accessors: a box's 12 edges share its 8 vertices, every Edge.Start and
// Edge.End is (by ==) one of the vertices Body.Vertices returns, a second read
// returns the same pointer, and the zero CoEdge has no vertices.
func TestEdgeVerticesAreTheBodyVertices(t *testing.T) {
	t.Parallel()
	body := coedgeBox(t)
	vertices := body.Vertices()
	require.Len(t, vertices, 8)
	edges := body.Edges()
	require.Len(t, edges, 12)
	uses := map[*decad.Vertex]int{}
	for _, e := range edges {
		for _, v := range []*decad.Vertex{e.Start(), e.End()} {
			require.NotNil(t, v)
			require.True(t, slices.Contains(vertices, v), "an edge end is one of the body's own vertices")
			uses[v]++
		}
		require.Same(t, e.Start(), e.Start())
		require.Same(t, e.End(), e.End())
	}
	require.Len(t, uses, 8)
	for _, n := range uses {
		require.Equal(t, 3, n, "each box corner ends three edges")
	}
	require.Nil(t, decad.CoEdge{}.Start())
	require.Nil(t, decad.CoEdge{}.End())
	require.Nil(t, decad.CoEdge{}.Edge())
}

// TestEdgeFacesAreTheBodyFaces pins face identity across the edge links: on a
// box every Edge.Faces element is (by ==) one of the faces Body.Faces returns,
// each edge has two distinct ones and each face bounds four edges, and a
// second read returns the same pointers in a fresh slice. On an open sheet a
// rim edge has exactly one face, and the zero Edge has nil faces.
func TestEdgeFacesAreTheBodyFaces(t *testing.T) {
	t.Parallel()
	body := coedgeBox(t)
	faces := body.Faces()
	require.Len(t, faces, 6)
	edges := body.Edges()
	require.Len(t, edges, 12)
	uses := map[*decad.Face]int{}
	for _, e := range edges {
		adjacent := e.Faces()
		require.Len(t, adjacent, 2)
		require.NotSame(t, adjacent[0], adjacent[1])
		for i, f := range adjacent {
			require.True(t, slices.Contains(faces, f), "an edge's face is one of the body's own faces")
			require.Same(t, f, e.Faces()[i])
			uses[f]++
		}
		adjacent[0] = nil
		require.NotNil(t, e.Faces()[0], "Faces returns a fresh slice")
		require.False(t, e.IsFree())
	}
	require.Len(t, uses, 6)
	for _, n := range uses {
		require.Equal(t, 4, n, "each box face bounds four edges")
	}

	s, p := plateSketch(t)
	sheet, err := decad.Extrude(t.Context(), decad.New(), s, p,
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	sheetFaces := sheet.Faces()
	require.Len(t, sheetFaces, 4)
	free := 0
	for _, e := range sheet.Edges() {
		adjacent := e.Faces()
		for _, f := range adjacent {
			require.True(t, slices.Contains(sheetFaces, f), "a sheet edge's face is one of the sheet's own faces")
		}
		if e.IsFree() {
			require.Len(t, adjacent, 1)
			free++
			continue
		}
		require.Len(t, adjacent, 2)
	}
	require.Equal(t, 8, free, "the open tube's two rims are its free edges")

	var zero decad.Edge
	require.Nil(t, zero.Faces())
	require.False(t, zero.IsFree())
}

// TestLoopCoEdgeEdgesAreTheBodyEdges pins edge identity across the loop
// links: on a box Body.Edges lists each of its 12 edges once, a second read
// returns the same pointers, and every CoEdge.Edge and Loop.Edges element is
// (by ==) one of them, two uses per edge. The zero Loop has no uses: CoEdges
// is nil and Edges is empty.
func TestLoopCoEdgeEdgesAreTheBodyEdges(t *testing.T) {
	t.Parallel()
	body := coedgeBox(t)
	edges := body.Edges()
	require.Len(t, edges, 12)
	for i, e := range edges {
		require.NotNil(t, e)
		require.False(t, slices.Contains(edges[:i], e), "Body.Edges lists each edge once")
		require.Same(t, e, body.Edges()[i])
	}
	uses := map[*decad.Edge]int{}
	for _, face := range body.Faces() {
		for _, loop := range face.Loops() {
			undirected := loop.Edges()
			for i, use := range loop.CoEdges() {
				require.True(t, slices.Contains(edges, use.Edge()), "a loop's edge is one of the body's own edges")
				require.Same(t, use.Edge(), loop.CoEdges()[i].Edge())
				require.Same(t, use.Edge(), undirected[i])
				uses[use.Edge()]++
			}
		}
	}
	require.Len(t, uses, 12)
	for _, n := range uses {
		require.Equal(t, 2, n, "each box edge has two loop uses")
	}

	var zero decad.Loop
	require.Nil(t, zero.CoEdges())
	require.NotNil(t, zero.Edges())
	require.Empty(t, zero.Edges())
}

// TestFaceLoopsKeepTheirIdentity pins loop identity across the face links: on
// a box each face has one outer loop, a second Face.Loops read returns the
// same pointers in a fresh slice, and Face.Edges is its loops' edges in order.
// The zero Face has nil loops and nil edges.
func TestFaceLoopsKeepTheirIdentity(t *testing.T) {
	t.Parallel()
	body := coedgeBox(t)
	edges := body.Edges()
	seen := map[*decad.Loop]struct{}{}
	for _, face := range body.Faces() {
		loops := face.Loops()
		require.Len(t, loops, 1)
		require.True(t, loops[0].IsOuter())
		var want []*decad.Edge
		for i, loop := range loops {
			require.Same(t, loop, face.Loops()[i])
			_, dup := seen[loop]
			require.False(t, dup, "no two faces share a loop")
			seen[loop] = struct{}{}
			for _, e := range loop.Edges() {
				require.True(t, slices.Contains(edges, e), "a face loop's edge is one of the body's own edges")
				want = append(want, e)
			}
		}
		got := face.Edges()
		require.Len(t, got, len(want))
		for i := range want {
			require.Same(t, want[i], got[i])
		}
		loops[0] = nil
		require.NotNil(t, face.Loops()[0], "Loops returns a fresh slice")
	}
	require.Len(t, seen, 6)

	var zero decad.Face
	require.Nil(t, zero.Loops())
	require.Nil(t, zero.Edges())
}

// TestBodyLumpsAndShellsKeepTheirIdentity pins identity across the body, lump
// and shell links: on a box a second Body.Lumps, Lump.Shells or
// FaceShell.Faces read returns the same pointers in a fresh slice,
// Body.Shells is the lumps' shells in order and Body.Faces the shells' faces
// in order. The zero Body, Lump and FaceShell have nil lumps, shells and
// faces.
func TestBodyLumpsAndShellsKeepTheirIdentity(t *testing.T) {
	t.Parallel()
	body := coedgeBox(t)
	lumps := body.Lumps()
	require.Len(t, lumps, 1)
	var shells []*decad.FaceShell
	for i, l := range lumps {
		require.Same(t, l, body.Lumps()[i])
		for j, s := range l.Shells() {
			require.Same(t, s, l.Shells()[j])
			shells = append(shells, s)
		}
	}
	require.Len(t, shells, 1)
	bodyShells := body.Shells()
	require.Len(t, bodyShells, len(shells))
	var faces []*decad.Face
	for i, s := range shells {
		require.Same(t, s, bodyShells[i])
		require.False(t, s.IsVoid())
		require.False(t, s.IsOpen())
		for j, f := range s.Faces() {
			require.Same(t, f, s.Faces()[j])
			faces = append(faces, f)
		}
	}
	require.Len(t, faces, 6)
	bodyFaces := body.Faces()
	require.Len(t, bodyFaces, len(faces))
	for i := range faces {
		require.Same(t, faces[i], bodyFaces[i])
	}
	lumps[0] = nil
	require.NotNil(t, body.Lumps()[0], "Lumps returns a fresh slice")
	shellsOf := body.Lumps()[0].Shells()
	shellsOf[0] = nil
	require.NotNil(t, body.Lumps()[0].Shells()[0], "Shells returns a fresh slice")
	facesOf := shells[0].Faces()
	facesOf[0] = nil
	require.NotNil(t, shells[0].Faces()[0], "Faces returns a fresh slice")

	var zeroBody decad.Body
	require.Nil(t, zeroBody.Lumps())
	require.Nil(t, zeroBody.Shells())
	require.Nil(t, zeroBody.Faces())
	var zeroLump decad.Lump
	require.Nil(t, zeroLump.Shells())
	var zeroShell decad.FaceShell
	require.Nil(t, zeroShell.Faces())
}

// TestBodyDocumentIsTheDocument pins identity across the body and document
// links: every body built in a document answers that same document, and so
// do a Transform result and a Copy; Bodies lists the live bodies in commit
// order in a fresh slice (a moved source leaves, a copy's source stays); a
// removed body still names its document; a document with no live body, new
// or emptied by Remove, has nil Bodies. The zero Body has a nil Document.
func TestBodyDocumentIsTheDocument(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	require.Nil(t, doc.Bodies())
	a := boxBody(t, doc, 0, 0, 10, 10, 5)
	b := boxBody(t, doc, 20, 0, 30, 10, 5)
	require.Same(t, doc, a.Document())
	require.Same(t, doc, b.Document())

	up, err := r3.Translation(r3.Vec{X: 0, Y: 0, Z: 50})
	require.NoError(t, err)
	moved, err := decad.Transform(t.Context(), a, up)
	require.NoError(t, err)
	require.Same(t, doc, moved.Document())
	copied, err := decad.Copy(t.Context(), b)
	require.NoError(t, err)
	require.Same(t, doc, copied.Document())

	live := doc.Bodies()
	require.Len(t, live, 3)
	for i, want := range []*decad.Body{b, moved, copied} {
		require.Same(t, want, live[i], "the live set lists bodies in commit order")
	}
	require.False(t, slices.Contains(live, a), "a moved source leaves the live set")
	live[0] = nil
	require.NotNil(t, doc.Bodies()[0], "Bodies returns a fresh slice")
	_, err = decad.Transform(t.Context(), a, up)
	require.ErrorIs(t, err, decad.ErrRetiredBody)

	for _, body := range doc.Bodies() {
		require.NoError(t, decad.Remove(body))
	}
	require.Nil(t, doc.Bodies())
	require.Same(t, doc, b.Document(), "a removed body still names its document")
	require.ErrorIs(t, decad.Remove(b), decad.ErrRetiredBody)

	var zero decad.Body
	require.Nil(t, zero.Document())
}

// TestZeroAndNilBodyOperandsAreRejected passes a zero Body (it belongs to no
// document) and a nil *Body as target, tool and subject. Each call answers the
// same error as before bodies reached their document through a node, and none
// panics.
func TestZeroAndNilBodyOperandsAreRejected(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 5)
	other := boxBody(t, doc, 20, 0, 30, 10, 5)
	move, err := r3.Translation(r3.Vec{Z: 1})
	require.NoError(t, err)
	ctx := t.Context()
	for name, tc := range map[string]struct {
		call func() (*decad.Body, error)
		want error
		text string
	}{
		"union zero target": {func() (*decad.Body, error) { return decad.Union(ctx, &decad.Body{}, box, other) },
			decad.ErrDegenerate, "the first operand belongs to no document"},
		"union zero only": {func() (*decad.Body, error) { return decad.Union(ctx, &decad.Body{}, &decad.Body{}) },
			decad.ErrDegenerate, "the first operand belongs to no document"},
		"union zero tool": {func() (*decad.Body, error) { return decad.Union(ctx, box, &decad.Body{}, other) },
			decad.ErrForeignBody, "body is owned by a different document"},
		"union nil tool": {func() (*decad.Body, error) { return decad.Union(ctx, box, nil, other) },
			decad.ErrDegenerate, "a nil body names nothing to operate on"},
		"cut zero target": {func() (*decad.Body, error) { return decad.Cut(ctx, &decad.Body{}, box) },
			decad.ErrDegenerate, "the first operand belongs to no document"},
		"cut zero tool": {func() (*decad.Body, error) { return decad.Cut(ctx, box, &decad.Body{}) },
			decad.ErrForeignBody, "body is owned by a different document"},
		"cut nil tool": {func() (*decad.Body, error) { return decad.Cut(ctx, box, nil) },
			decad.ErrDegenerate, "a nil body names nothing to operate on"},
		"intersect zero": {func() (*decad.Body, error) { return decad.Intersect(ctx, &decad.Body{}, box) },
			decad.ErrDegenerate, "the first operand belongs to no document"},
		"copy zero": {func() (*decad.Body, error) { return decad.Copy(ctx, &decad.Body{}) },
			decad.ErrDegenerate, "the body belongs to no document"},
		"copy nil": {func() (*decad.Body, error) { return decad.Copy(ctx, nil) },
			decad.ErrDegenerate, "the body belongs to no document"},
		"transform zero": {func() (*decad.Body, error) { return decad.Transform(ctx, &decad.Body{}, move) },
			decad.ErrDegenerate, "the body belongs to no document"},
		"transform nil": {func() (*decad.Body, error) { return decad.Transform(ctx, nil, move) },
			decad.ErrDegenerate, "the body belongs to no document"},
	} {
		t.Run(name, func(t *testing.T) {
			require.NotPanics(t, func() {
				got, err := tc.call()
				require.Nil(t, got)
				require.ErrorIs(t, err, tc.want)
				require.ErrorContains(t, err, tc.text)
			})
		})
	}
	require.Len(t, doc.Bodies(), 2, "a rejected call leaves the live set unchanged")
}

// TestMeshSourceFacesAreTheBodyFaces pins identity across the mesh links: a
// box mesh's SourceFaces runs parallel to Triangles, every element is one of
// the body's own faces and every face sources a facet; a second read is the
// same faces in a fresh slice; a second Tessellate at one tolerance and level
// is the cached *Mesh itself, and another level is another mesh; a Transform
// result's mesh sources the result's faces, never the moved source's. The
// zero Mesh has nil SourceFaces.
func TestMeshSourceFacesAreTheBodyFaces(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 100, 60, 10)
	tol := units.Millimeters(0.5)
	mesh, err := box.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	faces := box.Faces()
	require.Len(t, faces, 6)
	source := mesh.SourceFaces()
	require.Len(t, source, len(mesh.Triangles()))
	used := map[*decad.Face]struct{}{}
	for _, f := range source {
		require.True(t, slices.Contains(faces, f), "a facet's source is one of the body's own faces")
		used[f] = struct{}{}
	}
	require.Len(t, used, len(faces), "every face of the box sources a facet")
	again := mesh.SourceFaces()
	for i := range source {
		require.Same(t, source[i], again[i])
	}
	again[0] = nil
	require.NotNil(t, mesh.SourceFaces()[0], "SourceFaces returns a fresh slice")

	cached, err := box.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	require.Same(t, mesh, cached, "a second Tessellate at one tolerance and level is the cached mesh")
	drawn, err := box.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	require.NotSame(t, mesh, drawn, "another verification level is another mesh")
	require.False(t, drawn.VolumeVerified())

	up, err := r3.Translation(r3.Vec{X: 0, Y: 0, Z: 50})
	require.NoError(t, err)
	moved, err := decad.Transform(t.Context(), box, up)
	require.NoError(t, err)
	movedMesh, err := moved.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	require.NotSame(t, mesh, movedMesh)
	movedFaces := moved.Faces()
	for _, f := range movedMesh.SourceFaces() {
		require.True(t, slices.Contains(movedFaces, f), "a moved body's facets source the moved body's faces")
		require.False(t, slices.Contains(faces, f), "a moved body's facets never source the moved source's faces")
	}

	var zero decad.Mesh
	require.Nil(t, zero.SourceFaces())
}
