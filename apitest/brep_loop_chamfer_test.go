package apitest_test

import (
	"io"
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
// are faces of the body, and the consumers PR L-1 does not yet read refuse.

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
// so the volume 15000 − (80d² − 4d³/3) is Exact. Tessellate, STEP export and
// a boolean with the result each refuse naming PR L-2, and the receiver is
// retired.
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

	_, err = got.Tessellate(t.Context(), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "modify-general L-2")
	err = export.STEP(t.Context(), io.Discard, got, units.Millimeters(0.1),
		export.WithSTEPName("plate"), export.WithSTEPAuthor("apitest"), export.WithSTEPOrganization("decad"))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "modify-general L-2")
	_, err = decad.Cut(t.Context(), got, boxBody(t, doc, 35, 35, 45, 45, 20))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "modify-general L-2")
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
