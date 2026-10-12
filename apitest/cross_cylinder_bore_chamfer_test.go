package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func crossCylinderBore(t *testing.T, radius, bore, height, centerZ float64) (*decad.Document, *decad.Body, *decad.Body) {
	t.Helper()
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	target, err := doc.Extrude(s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)

	tw := sketch.NewWorld()
	ts, err := tw.CreateSketch(tw.YZ())
	require.NoError(t, err)
	toolCenter := ts.CreatePoint(0, centerZ)
	ts.Fix(toolCenter)
	ts.CreateCircle(toolCenter, bore)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, ts.Profiles(), 1)
	tool, err := doc.Extrude(ts, ts.Profiles()[0],
		decad.Symmetric{D: units.Millimeters(2*radius + 2)})
	require.NoError(t, err)
	bored, err := decad.Cut(t.Context(), target, tool)
	require.NoError(t, err)
	return doc, target, bored
}

func crossBoreMouth(t *testing.T, body *decad.Body, centerZ, bore float64, positive bool) decad.EdgeSelector {
	t.Helper()
	for _, edge := range body.Edges() {
		p := edge.Start().Position().Value
		if math.Abs(p.Z-centerZ) > bore+0.1 || (p.X > 0) != positive {
			continue
		}
		selection := decad.Edges(decad.EndpointAt(p)).Exactly(1)
		selected, err := selection.SelectEdges(body)
		require.NoError(t, err)
		require.Equal(t, edge, selected[0])
		return selection
	}
	t.Fatal("the cross-bore mouth loop is missing")
	return nil
}

func TestCrossCylinderBoreMouthChamferVerified(t *testing.T) {
	for _, tc := range []struct {
		name                string
		R, r, H, c, setback float64
	}{
		{name: "five by one", R: 5, r: 1, H: 10, c: 5, setback: 0.5},
		{name: "eight by one and half", R: 8, r: 1.5, H: 16, c: 8, setback: 0.1875},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc, target, bored := crossCylinderBore(t, tc.R, tc.r, tc.H, tc.c)
			out, err := bored.Chamfer(t.Context(),
				crossBoreMouth(t, bored, tc.c, tc.r, true), units.Millimeters(tc.setback))
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{out}, doc.Bodies())
			for _, cap := range []decad.FeatureRef{decad.CapStart(target), decad.CapEnd(target)} {
				faces, err := decad.Faces(decad.FaceCreatedBy(cap)).Exactly(1).SelectFaces(out)
				require.NoError(t, err)
				require.Len(t, faces, 1)
			}
			blendFaces := 0
			for _, face := range out.Faces() {
				for _, origin := range face.Origins() {
					if origin.Role == "boreMouthChamfer" {
						blendFaces++
					}
				}
			}
			require.Equal(t, 1, blendFaces)
			mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			require.Less(t, mesh.Bound().Base(), 0.1)
			volume, err := out.Volume()
			require.NoError(t, err)
			require.Positive(t, volume.Value.Base()-volume.Bound.Base())
			_, err = out.Area()
			require.NoError(t, err)
			_, err = out.Centroid()
			require.NoError(t, err)
		})
	}
}

func TestCrossCylinderBoreMouthChamferRefusesUnprovedCases(t *testing.T) {
	for _, tc := range []struct {
		name     string
		positive bool
		setback  float64
	}{
		{name: "other mouth", positive: false, setback: 0.5},
		{name: "large setback", positive: true, setback: 0.6},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc, _, bored := crossCylinderBore(t, 5, 1, 10, 5)
			_, err := bored.Chamfer(t.Context(),
				crossBoreMouth(t, bored, 5, 1, tc.positive), units.Millimeters(tc.setback))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Equal(t, []*decad.Body{bored}, doc.Bodies())
		})
	}
}
