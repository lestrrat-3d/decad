package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md B19's public-surface tests: an Extrude to a planar face of a mesh
// solid that is tilted from the sweep.

// rampSolid is a 20 x 10 mesh block whose top slopes from 4 mm at x = 0 to 14 mm at x = 20 (z = 4 + x/2).
func rampSolid(t *testing.T, doc *decad.Document, ramp float64) (*decad.Body, decad.FaceSelector) {
	t.Helper()
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(20, 0, 0), r3.NewVec(20, 10, 0), r3.NewVec(0, 10, 0),
		r3.NewVec(0, 0, 4), r3.NewVec(20, 0, 4+ramp), r3.NewVec(20, 10, 4+ramp), r3.NewVec(0, 10, 4),
	}
	var tris [][3]int
	for _, q := range [][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}} {
		tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
	}
	m, err := decad.NewMeshBody(t.Context(), verts, tris)
	require.NoError(t, err)
	body, err := decad.SolidFromMesh(t.Context(), doc, m)
	require.NoError(t, err)
	// The sloped face is the one whose area centroid is the middle of the slope.
	for _, f := range body.Faces() {
		c, err := f.Centroid()
		require.NoError(t, err)
		if c.Value.Sub(r3.NewVec(10, 5, 4+ramp/2)).Len() < 1e-9 {
			return body, decad.Faces(decad.FaceCreatedBy(f.Origins()[0]))
		}
	}
	require.FailNow(t, "the ramp has no sloped face")
	return nil, nil
}

func rectOnXY(t *testing.T, u0, v0, u1, v1 float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(u0, v0, u1, v1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

func TestExtrudeToATiltedMeshFaceStopsOnTheFacesPlane(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	stop, face := rampSolid(t, doc, 10)
	s, p := rectOnXY(t, 5, 2, 15, 8)
	out, err := decad.Extrude(t.Context(), doc, s, p, decad.ToFace{Body: stop, Face: face})
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	// The prism runs from z = 0 up to z = 4 + x/2 over x in [5, 15], y in [2, 8]: 6 * (4 * 10 + 0.25 * (225 - 25)).
	require.InDelta(t, 540, volume.Value.Base(), volume.Bound.Base()+1e-9)
	require.ElementsMatch(t, []*decad.Body{stop, out}, doc.Bodies(), "the extrusion and the cut leave the stop and the result live")
	box, err := out.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 11.5, box.Max.Z, box.Bound.Base()+1e-9)
	require.InDelta(t, 6.5, box.Min.Z+6.5, 1e-9)
	for point, want := range map[r3.Vec]decad.PointLocation{
		r3.NewVec(10, 5, 8.9):  decad.PointInside,
		r3.NewVec(10, 5, 9.1):  decad.PointOutside,
		r3.NewVec(6, 3, 6.9):   decad.PointInside,
		r3.NewVec(6, 3, 7.1):   decad.PointOutside,
		r3.NewVec(14, 7, 10.9): decad.PointInside,
		r3.NewVec(14, 7, 11.1): decad.PointOutside,
	} {
		got, err := out.LocatePoint(t.Context(), point, units.Millimeters(0.01))
		require.NoError(t, err)
		require.Equal(t, want, got, "%v", point)
	}
}

func TestExtrudeToATiltedMeshFaceRefusesAFootprintThePolygonDoesNotHold(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	stop, face := rampSolid(t, doc, 10)
	// The footprint reaches x = 25, past the ramp's 20 mm: the face does not stop the whole profile.
	s, p := rectOnXY(t, 5, 2, 25, 8)
	_, err := decad.Extrude(t.Context(), doc, s, p, decad.ToFace{Body: stop, Face: face})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{stop}, doc.Bodies(), "a refusal leaves no half-built extrusion behind")
}

func TestExtrudeToATiltedMeshFaceFromAboveSweepsDown(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	stop, face := rampSolid(t, doc, 10)
	// A rectangle on the plane z = 20, above the whole ramp, swept down onto it: the prism between z = 4 + x/2 and 20.
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(mustFrame(t, r3.NewVec(0, 0, 20), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
	require.NoError(t, err)
	s, err := w.CreateSketch(pl)
	require.NoError(t, err)
	rect := s.CreateRectangle(5, 2, 15, 8)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	out, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.ToFace{Body: stop, Face: face})
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	// 6 * (20 * 10 - 90): the box of height 20 over the footprint less the 540 under the ramp.
	require.InDelta(t, 6*(200-90), volume.Value.Base(), volume.Bound.Base()+1e-9)
}

func mustFrame(t *testing.T, origin, u, v r3.Vec) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	return f
}
