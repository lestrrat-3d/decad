package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

var (
	faceNegY = decad.Facing(r3.NewVec(0, -1, 0))
	facePosY = decad.Facing(r3.NewVec(0, 1, 0))
	facePosZ = decad.Facing(r3.NewVec(0, 0, 1))
)

// requireNoSeam asserts that every face of body is planar and that no edge joins two faces on one oriented plane.
// The tolerances are test-side only; production never uses one.
func requireNoSeam(t *testing.T, body *decad.Body) {
	t.Helper()
	for i, f := range body.Faces() {
		_, ok := f.Surface().(decad.Plane)
		require.True(t, ok, "face %d is planar", i)
	}
	// outward is the face's outward unit normal at p and offset its plane offset along that normal.
	outward := func(f *decad.Face, p r3.Vec) (r3.Vec, float64) {
		n, err := f.NormalAt(p)
		require.NoError(t, err)
		plane := f.Surface().(decad.Plane)
		u := n.Value
		sign := 1.0
		if plane.Frame.N().Dot(u) < 0 {
			sign = -1
		}
		return u, sign * plane.Frame.N().Dot(plane.Frame.Origin())
	}
	for i, e := range body.Edges() {
		faces := e.Faces()
		if len(faces) != 2 {
			continue
		}
		p := e.Start().Position().Value
		n0, o0 := outward(faces[0], p)
		n1, o1 := outward(faces[1], p)
		same := n0.Dot(n1) > 1-1e-12 && math.Abs(o0-o1) < 1e-9
		require.False(t, same, "edge %d has both faces on one plane", i)
	}
}

// requireOneFaceWithRoles asserts that sel resolves to one face and that every origin of that face selects it.
func requireOneFaceWithRoles(t *testing.T, body *decad.Body, pred decad.FacePredicate) *decad.Face {
	t.Helper()
	faces, err := decad.Faces(pred).Exactly(1).SelectFaces(body)
	require.NoError(t, err)
	origins := faces[0].Origins()
	require.GreaterOrEqual(t, len(origins), 2, "the face lists the roles of both operands")
	for _, o := range origins {
		got, err := decad.Faces(decad.FaceCreatedBy(o)).Exactly(1).SelectFaces(body)
		require.NoError(t, err, "origin %v", o)
		require.Same(t, faces[0], got[0])
	}
	return faces[0]
}

func requireNoDiagnostics(t *testing.T, b *decad.Body) {
	t.Helper()
	require.Empty(t, b.OperationDiagnostics())
}

// mergeFlushBoxes is a 10×10×10 box united with a copy moved to (dx, dy, 10).
func mergeFlushBoxes(t *testing.T, dx, dy float64) *decad.Body {
	t.Helper()
	return mergeFlushBoxesIn(t, decad.New(), dx, dy)
}

func mergeFlushBoxesIn(t *testing.T, doc *decad.Document, dx, dy float64) *decad.Body {
	t.Helper()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), dx, dy, 10)
	joined, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	return joined
}

// xzPrism extrudes the polygon, given as (x, z) points on the plane through the origin whose normal is -y, by d
// against that normal (toward +y).
func xzPrism(t *testing.T, doc *decad.Document, pts [][2]float64, d float64) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	verts := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		verts[i] = s.CreatePoint(p[0], p[1])
		s.Fix(verts[i])
	}
	for i := range verts {
		s.CreateLine(verts[i], verts[(i+1)%len(verts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(d), Dir: decad.Against})
	require.NoError(t, err)
	return body
}

func TestUnionFaceMergeFlushBoxes(t *testing.T) {
	t.Parallel()
	joined := mergeFlushBoxes(t, 0.5, 0)
	require.Len(t, joined.Faces(), 10)
	requireNoDiagnostics(t, joined)
	requireVolumeNear(t, joined, 2000)
	requireNoSeam(t, joined)
	face := requireOneFaceWithRoles(t, joined, faceNegY)
	_, ok := face.Surface().(decad.Plane)
	require.True(t, ok)
	requireOneFaceWithRoles(t, joined, facePosY)
}

func TestUnionFaceMergeDovetailEnds(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	host := boxBodyAtZ(t, doc, -10, 0, 10, 16, -10, 10)
	tool := xzPrism(t, doc, [][2]float64{{-5, -1}, {5, -1}, {5, 0}, {6, 6}, {-6, 6}, {-5, 0}}, 16)
	joined, err := decad.Union(t.Context(), host, tool)
	require.NoError(t, err)
	require.Len(t, joined.Faces(), 10)
	requireNoSeam(t, joined)
	requireVolumeNear(t, joined, 4256)
	for _, pred := range []decad.FacePredicate{faceNegY, facePosY} {
		f := requireOneFaceWithRoles(t, joined, pred)
		require.Len(t, f.Loops(), 1)
		require.True(t, f.Loops()[0].IsOuter())
	}
}

func TestUnionFaceMergeCutStepDown(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	target := xzPrism(t, doc, [][2]float64{{0, 0}, {20, 0}, {19, 10}, {10, 10}, {10, 5}, {0, 5}}, 10)
	tool := translated(t, boxBody(t, doc, 0, 0, 11, 12, 6), 10, -1, 5)
	cut, err := decad.Cut(t.Context(), target, tool)
	require.NoError(t, err)
	require.Len(t, cut.Faces(), 6)
	requireNoSeam(t, cut)
	requireOneFaceWithRoles(t, cut, facePosZ)
	requireVolumeNear(t, cut, 987.5)
}

func TestUnionFaceMergeIntersectSteppedPrisms(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := xzPrism(t, doc, [][2]float64{{0, 0}, {12, 0}, {12, 20}, {6, 20}, {6, 10}, {0, 10}}, 10)
	b := translated(t, xzPrism(t, doc, [][2]float64{{1, 0}, {13, 0}, {13, 10}, {4, 10}, {4, 20}, {1, 20}}, 10), 1, 0, 0)
	both, err := decad.Intersect(t.Context(), a, b)
	require.NoError(t, err)
	require.Len(t, both.Faces(), 6)
	requireNoSeam(t, both)
	requireOneFaceWithRoles(t, both, facePosZ)
	requireVolumeNear(t, both, 1000)
}

func TestUnionFaceMergeKeepsOffsetPlanesApart(t *testing.T) {
	t.Parallel()
	joined := mergeFlushBoxes(t, 0.5, math.Ldexp(1, -40))
	faces, err := decad.Faces(faceNegY).AtLeast(2).SelectFaces(joined)
	require.NoError(t, err)
	require.Len(t, faces, 2, "the two y = 0 walls stay separate")
}

func TestUnionFaceMergeThenChamfer(t *testing.T) {
	t.Parallel()
	joined := mergeFlushBoxes(t, 0.5, 0)
	edge := decad.Edges(decad.ParallelTo(r3.Vec{X: 1}), decad.EndpointAt(r3.Vec{X: 0.5, Z: 20}))
	chamfered, err := decad.Chamfer(t.Context(), joined, edge, units.Millimeters(1))
	require.NoError(t, err)
	before, err := joined.Volume()
	require.NoError(t, err)
	after, err := chamfered.Volume()
	require.NoError(t, err)
	// The chamfer removes a 1×1 triangle along the 10 mm edge.
	require.InDelta(t, volumeMM(t, before)-5, volumeMM(t, after), before.Bound.Base()+after.Bound.Base()+1e-9)
	requireNoSeam(t, chamfered)
}

func TestUnionFaceMergeThenCut(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	joined := mergeFlushBoxesIn(t, doc, 0.5, 0)
	pocket := translated(t, boxBody(t, doc, 0, 0, 2, 2, 2), 4, 4, 19)
	cut, err := decad.Cut(t.Context(), joined, pocket)
	require.NoError(t, err)
	// This Cut takes the orthogonal route, which groups facets by exact plane and mints fresh roles, so
	// the walls keep no operand roles (design §12 Q2 covers the analytic routes). The flush walls are one face
	// each: the outer wall and the pocket's wall make two faces on each y side.
	require.Len(t, cut.Faces(), 15)
	requireNoSeam(t, cut)
	for _, pred := range []decad.FacePredicate{faceNegY, facePosY} {
		faces, err := decad.Faces(pred).Exactly(2).SelectFaces(cut)
		require.NoError(t, err)
		require.Len(t, faces, 2)
	}
}
