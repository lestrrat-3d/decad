package apitest_test

import (
	"bytes"
	"math"
	"math/big"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin route L of docs/modify-general-design.md (§4, §9)
// through the public API: Body.Chamfer of a complete loop of a planar face
// of a stacked or brep boolean result builds a brep body whose band patches
// are faces of the body, and the mesh, STEP and boolean consumers read them.

// loopEdgeQuery selects exactly the edges of loop l: each line by its two
// ends and direction, each arc or circle by its ends among circular edges.
func loopEdgeQuery(l *decad.Loop) *decad.EdgeQuery {
	var q *decad.EdgeQuery
	edges := l.Edges()
	for _, e := range edges {
		a, b := e.Start().Position().Value, e.End().Position().Value
		preds := []decad.EdgePredicate{decad.EndpointAt(a), decad.EndpointAt(b)}
		if _, line := e.Curve().(decad.Line3); line {
			preds = append(preds, decad.ParallelTo(b.Sub(a)))
		} else {
			preds = append(preds, decad.Circular())
		}
		if q == nil {
			q = decad.Edges(preds...)
			continue
		}
		q.Or(preds...)
	}
	return q.Exactly(len(edges))
}

// planeFacing finds the one planar face of body whose outward normal is n at
// its first vertex and whose plane passes through p.
func planeFacing(t *testing.T, body *decad.Body, n, p r3.Vec) *decad.Face {
	t.Helper()
	var found []*decad.Face
	for _, f := range body.Faces() {
		pl, ok := f.Surface().(decad.Plane)
		if !ok || math.Abs(pl.Frame.Origin().Sub(p).Dot(n)) > 1e-9 {
			continue
		}
		at := f.Loops()[0].CoEdges()[0].Start().Position().Value
		normal, err := f.NormalAt(at)
		require.NoError(t, err)
		if normal.Value.Sub(n).Len() < 1e-12 {
			found = append(found, f)
		}
	}
	require.Len(t, found, 1, "one planar face faces %v through %v", n, p)
	return found[0]
}

// chamferLoopPatches lists the faces carrying a chamferLoop(f,l,p) role.
func chamferLoopPatches(body *decad.Body) []*decad.Face {
	var out []*decad.Face
	for _, f := range body.Faces() {
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamferLoop(") {
				out = append(out, f)
			}
		}
	}
	return out
}

// TestBrepLoopChamferPublicPlateTopLoop chamfers the pocketed plate's top
// loop, the 40×40 outer loop of its top face, by 1.5: the plate is a stacked
// receiver, so the chamfer takes route L and builds a closed brep of the
// receiver's 11 faces and four planar patches. Every coordinate is a float,
// so the volume 15000 − (80d² − 4d³/3) is Exact. The body tessellates to a
// closed mesh whose facets are the same volume, STEP writes it through the
// analytic arm (planes alone, one ADVANCED_FACE per face), and a Cut by a box
// over its corner composes the mesh. The receiver is retired.
func TestBrepLoopChamferPublicPlateTopLoop(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 40, 40, 10)
	pocket, err := decad.Cut(t.Context(), plate, boxBodyAtZ(t, doc, 10, 15, 30, 25, 5, 5))
	require.NoError(t, err)
	top := planeFacing(t, pocket, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 10))
	got, err := pocket.Chamfer(t.Context(), loopEdgeQuery(top.Loops()[0]), units.Millimeters(1.5))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1, "the receiver is retired")
	require.Len(t, got.Faces(), 15)
	requireEveryEdgeOnTwoFaces(t, got)
	patches := chamferLoopPatches(got)
	require.Len(t, patches, 4)
	for _, f := range patches {
		_, ok := f.Surface().(decad.Plane)
		require.True(t, ok, "a straight wall's band patch is a plane")
	}
	volume, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, 14824.5, volumeMM(t, volume))

	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.InDelta(t, 14824.5, meshVolume(mesh), 1e-9)

	var buf bytes.Buffer
	require.NoError(t, export.STEP(t.Context(), &buf, got, units.Millimeters(0.1),
		export.WithSTEPName("plate"), export.WithSTEPAuthor("apitest"), export.WithSTEPOrganization("decad")))
	text := buf.String()
	require.Contains(t, text, "analytic decad solid")
	require.Equal(t, 15, strings.Count(text, "=ADVANCED_FACE("), "one analytic face per face of the body")

	// The corner box removes 250 − (25d − (5³ − (5 − d)³)/3) = 239.875 of the
	// chamfered corner's volume.
	cut, err := decad.Cut(t.Context(), got, boxBodyAtZ(t, doc, 35, 35, 45, 45, -5, 30))
	require.NoError(t, err)
	remaining, err := cut.Volume()
	require.NoError(t, err)
	require.InDelta(t, 14584.625, volumeMM(t, remaining), remaining.Bound.Base()+1e-9)
}

// TestBrepLoopChamferPublicBossRoot chamfers a round boss's root by 1: the
// Ø10 boss stands 15 tall on the 40×40×10 plate, and its root is the plate
// top's hole loop around it, whose wall rises off the plate. The band is one
// Cone that fills the concave corner with the ring of section ½ at radius
// 5 + 1/3, so the volume is 16000 + 375π + 16π/3, and the cone's outward
// normal points up and away from the boss.
func TestBrepLoopChamferPublicBossRoot(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 40, 40, 10)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(20, 20)
	s.Fix(c)
	s.CreateCircle(c, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	boss, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(15), Dir: decad.Along})
	require.NoError(t, err)
	part, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)

	top := planeFacing(t, part, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, 10))
	require.Len(t, top.Loops(), 2)
	got, err := part.Chamfer(t.Context(), loopEdgeQuery(top.Loops()[1]), units.Millimeters(1))
	require.NoError(t, err)
	requireEveryEdgeOnTwoFaces(t, got)
	patches := chamferLoopPatches(got)
	require.Len(t, patches, 1)
	_, ok := patches[0].Surface().(decad.Cone)
	require.True(t, ok)
	n, err := patches[0].NormalAt(r3.NewVec(25.5, 20, 10.5))
	require.NoError(t, err)
	require.Positive(t, n.Value.Dot(r3.NewVec(1, 0, 1)))

	volume, err := got.Volume()
	require.NoError(t, err)
	piLo, _ := new(big.Rat).SetString("3.14159265358979323846")
	piHi, _ := new(big.Rat).SetString("3.14159265358979323847")
	at := func(pi *big.Rat) *big.Rat {
		return new(big.Rat).Add(big.NewRat(16000, 1), new(big.Rat).Mul(big.NewRat(1141, 3), pi))
	}
	held := new(big.Rat).SetFloat64(volume.Value.Base())
	bound := new(big.Rat).SetFloat64(volume.Bound.Base())
	require.LessOrEqual(t, new(big.Rat).Sub(held, bound).Cmp(at(piLo)), 0)
	require.GreaterOrEqual(t, new(big.Rat).Add(held, bound).Cmp(at(piHi)), 0)
}

// roundedDrilledPlate is docs/modify-general-design.md §1's P8 through the
// public API: the 40×20×20 box with its four vertical edges filleted r = 3,
// then cut by a Ø6 hole along y through (20, ·, 10).
func roundedDrilledPlate(t *testing.T) *decad.Body {
	t.Helper()
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	rounded, err := box.Fillet(t.Context(), decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4), units.Millimeters(3))
	require.NoError(t, err)

	plane, err := w.CreateOffsetPlane(w.XZ(), -10)
	require.NoError(t, err)
	ds, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := ds.CreatePoint(20, 10)
	ds.Fix(c)
	ds.CreateCircle(c, 3)
	_, err = ds.Solve(t.Context())
	require.NoError(t, err)
	drill, err := doc.Extrude(ds, ds.Profiles()[0], decad.Symmetric{D: units.Millimeters(11)})
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), rounded, drill)
	require.NoError(t, err)
	return out
}

// TestBrepLoopChamferPublicRoundedPlateExports chamfers P8's top loop by 1 (four
// Plane and four Cone patches at G1 joins, 15232 − 8π/3) and its hole rim by 1
// (one Cone, 15280 − 10π/3). Each body tessellates to a closed mesh whose
// volume lies within the chord times the curved faces' area of the closed form, Verify reads it Sound, and STEP writes it faceted, a cone sending the
// whole body to the faceted writer: one ADVANCED_FACE per mesh triangle.
func TestBrepLoopChamferPublicRoundedPlateExports(t *testing.T) {
	t.Parallel()
	up := r3.NewVec(0, 0, 1)
	for _, tc := range []struct {
		name   string
		face   func(t *testing.T, body *decad.Body) *decad.Face
		loop   int
		cones  int
		planes int
		a, b   *big.Rat
	}{
		{"top loop", func(t *testing.T, body *decad.Body) *decad.Face {
			return planeFacing(t, body, up, r3.NewVec(0, 0, 20))
		}, 0, 4, 4, big.NewRat(15232, 1), big.NewRat(-8, 3)},
		{"hole rim", func(t *testing.T, body *decad.Body) *decad.Face {
			return planeFacing(t, body, r3.NewVec(0, -1, 0), r3.Vec{})
		}, 1, 1, 0, big.NewRat(15280, 1), big.NewRat(-10, 3)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			plate := roundedDrilledPlate(t)
			doc := plate.Document()
			got, err := plate.Chamfer(t.Context(), loopEdgeQuery(tc.face(t, plate).Loops()[tc.loop]), units.Millimeters(1))
			require.NoError(t, err)
			requireEveryEdgeOnTwoFaces(t, got)
			planes, cones := 0, 0
			for _, f := range chamferLoopPatches(got) {
				switch f.Surface().(type) {
				case decad.Plane:
					planes++
				case decad.Cone:
					cones++
				}
			}
			require.Equal(t, [2]int{tc.planes, tc.cones}, [2]int{planes, cones})

			rep, err := doc.Verify(t.Context())
			require.NoError(t, err)
			br, err := rep.ForBody(got)
			require.NoError(t, err)
			require.Equal(t, decad.Sound, br.Status)

			mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			piLo, _ := new(big.Rat).SetString("3.14159265358979323846")
			piHi, _ := new(big.Rat).SetString("3.14159265358979323847")
			at := func(pi *big.Rat) *big.Rat { return new(big.Rat).Add(tc.a, new(big.Rat).Mul(tc.b, pi)) }
			lo, hi := at(piLo), at(piHi)
			if lo.Cmp(hi) > 0 {
				lo, hi = hi, lo
			}
			// A chord polygon lies within its sagitta of the surface it chords,
			// so the mesh differs from the body by at most the chord times the
			// area of the body's curved faces.
			curved := 0.0
			for _, f := range got.Faces() {
				switch f.Surface().(type) {
				case decad.Cone, decad.Cylinder:
					area, err := f.Area()
					require.NoError(t, err)
					curved += area.Value.Base()
				}
			}
			held := new(big.Rat).SetFloat64(meshVolume(mesh))
			slack := new(big.Rat).SetFloat64(0.05 * curved)
			require.LessOrEqual(t, new(big.Rat).Sub(held, slack).Cmp(lo), 0)
			require.GreaterOrEqual(t, new(big.Rat).Add(held, slack).Cmp(hi), 0)

			var buf bytes.Buffer
			require.NoError(t, export.STEP(t.Context(), &buf, got, units.Millimeters(0.05),
				export.WithSTEPName("plate"), export.WithSTEPAuthor("apitest"), export.WithSTEPOrganization("decad")))
			text := buf.String()
			require.Contains(t, text, "faceted decad solid")
			require.Equal(t, len(mesh.Triangles()), strings.Count(text, "=ADVANCED_FACE("))
		})
	}
}
