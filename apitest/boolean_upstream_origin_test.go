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

// This file is agent-docs/prism-boolean-design.md §11's public-surface tests: an
// analytic Boolean result's planar face inherits an operand's origin roles only
// where exact plane identity proves it lies inside exactly one operand face.

// levelBox extrudes the rectangle x0..x1, y0..y1 up from height z0 by h, sketched
// on the XY plane offset to z0.
func levelBox(t *testing.T, doc *decad.Document, x0, y0, x1, y1, z0, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane := w.XY()
	if z0 != 0 {
		var err error
		plane, err = w.CreateOffsetPlane(w.XY(), z0)
		require.NoError(t, err)
	}
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	r := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(r.A)
	s.Fix(r.C)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return b
}

// upstreamFace selects the one face of b created by ref and checks its area and
// that every vertex of its loops lies at height z.
func upstreamFace(t *testing.T, b *decad.Body, ref decad.FeatureRef, area, z float64) *decad.Face {
	t.Helper()
	faces, err := decad.Faces(decad.FaceCreatedBy(ref)).Exactly(1).SelectFaces(b)
	require.NoError(t, err)
	got, err := faces[0].Area()
	require.NoError(t, err)
	require.InDelta(t, area, got.Value.Base(), got.Bound.Base()+1e-9)
	for _, loop := range faces[0].Loops() {
		for _, ce := range loop.CoEdges() {
			require.InDelta(t, z, ce.Edge().Start().Position().Value.Z, 1e-9)
		}
	}
	return faces[0]
}

// TestUnionBossTopKeepsTheBossCapRole is the plate-and-boss Union: the boss top
// at z = 25 is the boss's CapEnd, its outer loop is the four top edges, and the
// plate top around the boss is the plate's CapEnd.
func TestUnionBossTopKeepsTheBossCapRole(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := levelBox(t, doc, 0, 0, 80, 60, 0, 10)
	boss := levelBox(t, doc, 30, 20, 50, 40, 10, 15)
	u, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)

	top := upstreamFace(t, u, decad.CapEnd(boss), 400, 25)
	edges, err := decad.Edges(decad.OuterLoopOf(decad.CapEnd(boss))).Exactly(4).SelectEdges(u)
	require.NoError(t, err)
	for _, e := range edges {
		require.InDelta(t, 25, e.Start().Position().Value.Z, 1e-9)
		require.InDelta(t, 25, e.End().Position().Value.Z, 1e-9)
	}
	upstreamFace(t, u, decad.CapEnd(plate), 80*60-20*20, 10)
	upstreamFace(t, u, decad.CapStart(plate), 80*60, 0)
	// The result's own cap role still names the same face.
	own, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(u))).Exactly(1).SelectFaces(u)
	require.NoError(t, err)
	require.Same(t, top, own[0])
	require.Contains(t, top.Origins(), decad.CapEnd(boss))
	require.Contains(t, top.Origins(), decad.CapEnd(u))
	// The boss's root sits inside the plate, so no face of the result is its
	// start cap.
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapStart(boss))).SelectFaces(u)
	require.ErrorIs(t, err, decad.ErrNoMatch)
}

// TestUnionBoxBottomKeepsTheBoxCapRole is the box with a boss on top: the
// bottom face is the box's CapStart with the box's whole footprint.
func TestUnionBoxBottomKeepsTheBoxCapRole(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := levelBox(t, doc, 0, 0, 60, 40, 0, 20)
	boss := levelBox(t, doc, 20, 10, 40, 30, 20, 10)
	u, err := decad.Union(t.Context(), box, boss)
	require.NoError(t, err)
	upstreamFace(t, u, decad.CapStart(box), 60*40, 0)
	upstreamFace(t, u, decad.CapEnd(boss), 20*20, 30)
}

// TestUnionLBracketKeepsTheBaseTopRole is the L bracket: the base top beside the
// upright is the base's CapEnd, and the bottom both operands share at z = 0
// names neither.
func TestUnionLBracketKeepsTheBaseTopRole(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	base := levelBox(t, doc, 0, 0, 60, 40, 0, 8)
	upright := levelBox(t, doc, 0, 0, 8, 40, 0, 50)
	u, err := decad.Union(t.Context(), base, upright)
	require.NoError(t, err)
	upstreamFace(t, u, decad.CapEnd(base), 52*40, 8)
	upstreamFace(t, u, decad.CapEnd(upright), 8*40, 50)
	for _, ref := range []decad.FeatureRef{decad.CapStart(base), decad.CapStart(upright)} {
		_, err = decad.Faces(decad.FaceCreatedBy(ref)).SelectFaces(u)
		require.ErrorIs(t, err, decad.ErrNoMatch)
	}
}

// TestUnionChainKeepsRolesThroughBrepOperands joins two blocks to the L
// bracket's base one Union at a time: each brep result's faces keep every role
// they inherited, so the base top still names the base's end cap two Unions
// later.
func TestUnionChainKeepsRolesThroughBrepOperands(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	base := levelBox(t, doc, 0, 0, 60, 40, 0, 8)
	upright := levelBox(t, doc, 0, 0, 8, 40, 0, 50)
	u, err := decad.Union(t.Context(), base, upright)
	require.NoError(t, err)
	first := levelBox(t, doc, 30, 10, 40, 20, 8, 4)
	v, err := decad.Union(t.Context(), u, first)
	require.NoError(t, err)
	second := levelBox(t, doc, 45, 25, 55, 35, 8, 6)
	w, err := decad.Union(t.Context(), v, second)
	require.NoError(t, err)
	volume, err := w.Volume()
	require.NoError(t, err)
	require.InDelta(t, 60*40*8+8*40*42+400+600, volume.Value.Base(), volume.Bound.Base()+1e-9)
	upstreamFace(t, w, decad.CapEnd(base), 52*40-100-100, 8)
	upstreamFace(t, w, decad.CapEnd(upright), 8*40, 50)
	upstreamFace(t, w, decad.CapEnd(first), 100, 12)
	upstreamFace(t, w, decad.CapEnd(second), 100, 14)
}

// TestUnionFlushBoxesShareNoCapRole is two overlapping boxes flush at both
// levels: every cap face lies on both operands' caps, so it names neither.
func TestUnionFlushBoxesShareNoCapRole(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := levelBox(t, doc, 0, 0, 10, 10, 0, 10)
	b := levelBox(t, doc, 5, 5, 15, 15, 0, 10)
	u, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	for _, ref := range []decad.FeatureRef{decad.CapStart(a), decad.CapEnd(a), decad.CapStart(b), decad.CapEnd(b)} {
		_, err = decad.Faces(decad.FaceCreatedBy(ref)).SelectFaces(u)
		require.ErrorIs(t, err, decad.ErrNoMatch)
	}
	faces, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(u))).Exactly(1).SelectFaces(u)
	require.NoError(t, err)
	area, err := faces[0].Area()
	require.NoError(t, err)
	require.InDelta(t, 175, area.Value.Base(), area.Bound.Base()+1e-9)
}

// TestTransformKeepsUpstreamCapRoles moves a Union: the placed copy's faces keep
// the operands' roles, at the moved height.
func TestTransformKeepsUpstreamCapRoles(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := levelBox(t, doc, 0, 0, 80, 60, 0, 10)
	boss := levelBox(t, doc, 30, 20, 50, 40, 10, 15)
	u, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(0, 0, 5))
	require.NoError(t, err)
	moved, err := decad.Transform(t.Context(), u, move)
	require.NoError(t, err)
	upstreamFace(t, moved, decad.CapEnd(boss), 400, 30)
	upstreamFace(t, moved, decad.CapEnd(plate), 80*60-20*20, 15)
	upstreamFace(t, moved, decad.CapEnd(moved), 400, 30)
	copied, err := decad.Copy(t.Context(), moved)
	require.NoError(t, err)
	upstreamFace(t, copied, decad.CapEnd(boss), 400, 30)
}

// TestCutPocketFloorKeepsTheToolStartRole cuts a blind pocket: its floor faces
// up and lies inside the tool's start cap, which faces down.
func TestCutPocketFloorKeepsTheToolStartRole(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := levelBox(t, doc, 0, 0, 40, 40, 0, 10)
	tool := levelBox(t, doc, 10, 10, 20, 20, 5, 10)
	out, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	floor := upstreamFace(t, out, decad.CapStart(tool), 100, 5)
	normal, err := floor.NormalAt(r3.NewVec(15, 15, 5))
	require.NoError(t, err)
	require.InDelta(t, 1, normal.Value.Z, 1e-9)
	upstreamFace(t, out, decad.CapEnd(plate), 40*40-10*10, 10)
	upstreamFace(t, out, decad.CapStart(plate), 40*40, 0)
}

// TestIntersectKeepsTheCapOfTheLowerTop intersects two boxes of different
// heights: the top lies inside the lower box's end cap alone.
func TestIntersectKeepsTheCapOfTheLowerTop(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := levelBox(t, doc, 0, 0, 20, 20, 0, 10)
	b := levelBox(t, doc, 5, 5, 15, 15, 0, 20)
	out, err := decad.Intersect(t.Context(), a, b)
	require.NoError(t, err)
	upstreamFace(t, out, decad.CapEnd(a), 100, 10)
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapEnd(b))).SelectFaces(out)
	require.ErrorIs(t, err, decad.ErrNoMatch)
}

// TestTransformKeepsUpstreamRolesOfEveryRecord moves the L bracket and the
// Intersect result: each placed copy keeps the operands' roles at the moved
// height.
func TestTransformKeepsUpstreamRolesOfEveryRecord(t *testing.T) {
	t.Parallel()
	move, err := r3.Translation(r3.NewVec(0, 0, 5))
	require.NoError(t, err)
	t.Run("L bracket", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		base := levelBox(t, doc, 0, 0, 60, 40, 0, 8)
		upright := levelBox(t, doc, 0, 0, 8, 40, 0, 50)
		u, err := decad.Union(t.Context(), base, upright)
		require.NoError(t, err)
		moved, err := decad.Transform(t.Context(), u, move)
		require.NoError(t, err)
		upstreamFace(t, moved, decad.CapEnd(base), 52*40, 13)
	})
	t.Run("Intersect", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		a := levelBox(t, doc, 0, 0, 20, 20, 0, 10)
		b := levelBox(t, doc, 5, 5, 15, 15, 0, 20)
		out, err := decad.Intersect(t.Context(), a, b)
		require.NoError(t, err)
		moved, err := decad.Transform(t.Context(), out, move)
		require.NoError(t, err)
		upstreamFace(t, moved, decad.CapEnd(a), 100, 15)
	})
}

// TestMeshUnionCarriesUpstreamRoles unions the analytic plate-and-boss result
// with a mesh wedge, which takes the mesh path: the mesh result's faces keep
// every role their operand faces carried, the boss's cap among them.
func TestMeshUnionCarriesUpstreamRoles(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := levelBox(t, doc, 0, 0, 80, 60, 0, 10)
	boss := levelBox(t, doc, 30, 20, 50, 40, 10, 15)
	u, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	verts := []r3.Vec{
		r3.NewVec(70, 50, 0), r3.NewVec(90, 50, 0), r3.NewVec(90, 70, 0), r3.NewVec(70, 70, 0),
		r3.NewVec(70, 50, 10), r3.NewVec(80, 50, 10), r3.NewVec(80, 70, 10), r3.NewVec(70, 70, 10),
	}
	var tris [][3]int
	for _, q := range [][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}} {
		tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
	}
	m, err := decad.NewMeshBody(t.Context(), verts, tris)
	require.NoError(t, err)
	block, err := decad.SolidFromMesh(t.Context(), doc, m)
	require.NoError(t, err)
	out, err := decad.Union(t.Context(), u, block)
	require.NoError(t, err)
	upstreamFace(t, out, decad.CapEnd(boss), 400, 25)
}

// TestModifyResultsDropUpstreamRoles shells the box-and-boss Union and fillets
// the L bracket's inner corner edge: each modify op rewrites the record and mints its
// own roles, so neither its result nor a placed copy names an operand's cap.
func TestModifyResultsDropUpstreamRoles(t *testing.T) {
	t.Parallel()
	move, err := r3.Translation(r3.NewVec(0, 0, 5))
	require.NoError(t, err)
	t.Run("Shell", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		box := levelBox(t, doc, 0, 0, 60, 40, 0, 20)
		boss := levelBox(t, doc, 20, 10, 40, 30, 20, 10)
		u, err := decad.Union(t.Context(), box, boss)
		require.NoError(t, err)
		shelled, err := decad.Shell(t.Context(), u, decad.Faces(decad.FaceCreatedBy(decad.CapEnd(boss))), units.Millimeters(2))
		require.NoError(t, err)
		moved, err := decad.Transform(t.Context(), shelled, move)
		require.NoError(t, err)
		for _, b := range []*decad.Body{shelled, moved} {
			_, err = decad.Faces(decad.FaceCreatedBy(decad.CapStart(box))).SelectFaces(b)
			require.ErrorIs(t, err, decad.ErrNoMatch)
		}
	})
	t.Run("Fillet", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		base := levelBox(t, doc, 0, 0, 60, 40, 0, 8)
		upright := levelBox(t, doc, 0, 0, 8, 40, 0, 50)
		u, err := decad.Union(t.Context(), base, upright)
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), u,
			decad.Edges(decad.ParallelTo(r3.NewVec(0, 1, 0)), decad.EndpointAt(r3.NewVec(8, 0, 8))).Exactly(1), units.Millimeters(2))
		require.NoError(t, err)
		moved, err := decad.Transform(t.Context(), filleted, move)
		require.NoError(t, err)
		for _, b := range []*decad.Body{filleted, moved} {
			_, err = decad.Faces(decad.FaceCreatedBy(decad.CapEnd(base))).SelectFaces(b)
			require.ErrorIs(t, err, decad.ErrNoMatch)
		}
	})
}

// TestFacetedNestedBossShellTakesAnInheritedCapBoss makes the boss of the
// faceted nested-boss shell as an Intersect, so its top carries the narrow
// box's end cap as an inherited role. The mesh Union keeps that role apart
// from the face's own, the nested-boss shell route still recognizes the boss
// top as its opening, and the shell has the volume the plain boss gives.
func TestFacetedNestedBossShellTakesAnInheritedCapBoss(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 4)
	narrow := boxBodyAtZ(t, doc, -4, -3, 4, 3, 3, 5)
	wide := boxBodyAtZ(t, doc, -6, -5, 6, 5, 0, 20)
	boss, err := decad.Intersect(t.Context(), narrow, wide)
	require.NoError(t, err)
	turn, err := r3.RotationAround(r3.Vec{X: 1, Y: 0.5}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	boss, err = decad.Transform(t.Context(), boss, turn)
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	top, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(narrow))).Exactly(1).SelectFaces(joined)
	require.NoError(t, err)
	require.Contains(t, top[0].Origins(), decad.CapEnd(boss))
	shell, err := decad.Shell(t.Context(), joined,
		decad.Faces(decad.FaceCreatedBy(decad.CapEnd(boss))).Exactly(1), units.Millimeters(1))
	require.NoError(t, err)
	volume, err := shell.Volume()
	require.NoError(t, err)
	// The same solid as TestFacetedUnionNestedBossShellVerified's.
	want := 1792.0 - (648 + 96 + 152.0/3 - 7*math.Pi)
	require.LessOrEqual(t, math.Abs(volume.Value.Mag()-want), volume.Bound.Mag())
}

// TestSheetOpsKeepUpstreamRoles splits the plate-and-boss Union into one sheet
// per face, patches the boss top's sheet, and stitches the sheets back: each
// copies the faces it keeps, inherited roles included.
func TestSheetOpsKeepUpstreamRoles(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := levelBox(t, doc, 0, 0, 80, 60, 0, 10)
	boss := levelBox(t, doc, 30, 20, 50, 40, 10, 15)
	u, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	sheets, err := decad.Unstitch(t.Context(), u)
	require.NoError(t, err)
	var top *decad.Body
	for _, sheet := range sheets {
		if _, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(boss))).SelectFaces(sheet); err == nil {
			require.Nil(t, top, "one sheet holds the boss top")
			top = sheet
		}
	}
	require.NotNil(t, top)
	upstreamFace(t, top, decad.CapEnd(boss), 400, 25)
	stitched, err := decad.Stitch(t.Context(), sheets...)
	require.NoError(t, err)
	upstreamFace(t, stitched, decad.CapEnd(boss), 400, 25)
	upstreamFace(t, stitched, decad.CapEnd(plate), 80*60-20*20, 10)

	doc2 := decad.New()
	plate2 := levelBox(t, doc2, 0, 0, 80, 60, 0, 10)
	boss2 := levelBox(t, doc2, 30, 20, 50, 40, 10, 15)
	u2, err := decad.Union(t.Context(), plate2, boss2)
	require.NoError(t, err)
	sheets2, err := decad.Unstitch(t.Context(), u2)
	require.NoError(t, err)
	var patchedCount int
	for _, sheet := range sheets2 {
		if _, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(boss2))).SelectFaces(sheet); err != nil {
			continue
		}
		patched, err := decad.PatchEdges(t.Context(), sheet, decad.Edges(decad.Free()).Exactly(4))
		require.NoError(t, err)
		upstreamFace(t, patched, decad.CapEnd(boss2), 400, 25)
		patchedCount++
	}
	require.Equal(t, 1, patchedCount)
}

// TestScaledAtKeepsUpstreamRoles doubles the plate-and-boss Union about the
// origin: the faceted image keeps the boss top's inherited role, scaled with
// the body.
func TestScaledAtKeepsUpstreamRoles(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := levelBox(t, doc, 0, 0, 80, 60, 0, 10)
	boss := levelBox(t, doc, 30, 20, 50, 40, 10, 15)
	u, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	scaled, err := decad.ScaledAt(t.Context(), u, r3.Vec{}, units.Scalar(2))
	require.NoError(t, err)
	upstreamFace(t, scaled, decad.CapEnd(boss), 4*400, 50)
}
