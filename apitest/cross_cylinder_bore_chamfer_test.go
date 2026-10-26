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

func crossCylinderBore(t *testing.T, radius, bore, height, centerZ float64) (*decad.Document, *decad.Body, *decad.Body) {
	return crossCylinderBoreAt(t, radius, bore, height, centerZ, 0)
}

func crossCylinderBoreAt(t *testing.T, radius, bore, height, centerZ, toolY float64) (
	*decad.Document, *decad.Body, *decad.Body) {
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
	target, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)

	tw := sketch.NewWorld()
	ts, err := tw.CreateSketch(tw.YZ())
	require.NoError(t, err)
	toolCenter := ts.CreatePoint(toolY, centerZ)
	ts.Fix(toolCenter)
	ts.CreateCircle(toolCenter, bore)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, ts.Profiles(), 1)
	tool, err := decad.Extrude(t.Context(), doc, ts, ts.Profiles()[0],
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
			out, err := decad.Chamfer(t.Context(), bored,
				crossBoreMouth(t, bored, tc.c, tc.r, true), units.Millimeters(tc.setback))
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{out}, doc.Bodies())
			for _, endCap := range []decad.FeatureRef{decad.CapStart(target), decad.CapEnd(target)} {
				faces, err := decad.Faces(decad.FaceCreatedBy(endCap)).Exactly(1).SelectFaces(out)
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
		name                    string
		positive                bool
		setback, centerZ, toolY float64
		asymmetric, placed      bool
	}{
		{name: "other mouth", positive: false, setback: 0.5, centerZ: 5},
		{name: "large setback", positive: true, setback: 0.6, centerZ: 5},
		{name: "cap contact", positive: true, setback: 0.5, centerZ: 1.5},
		{name: "offset tool", positive: true, setback: 0.5, centerZ: 5, toolY: 0.5},
		{name: "asymmetric setback", positive: true, setback: 0.5, centerZ: 5, asymmetric: true},
		{name: "placed result", positive: true, setback: 0.5, centerZ: 5, placed: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc, target, bored := crossCylinderBoreAt(t, 5, 1, 10, tc.centerZ, tc.toolY)
			receiver := bored
			centerZ := tc.centerZ
			if tc.placed {
				motion, err := r3.Translation(r3.NewVec(0, 0, 1))
				require.NoError(t, err)
				receiver, err = decad.Copy(t.Context(), bored, decad.WithCopyPlacement(motion))
				require.NoError(t, err)
				centerZ++
			}
			var opts []decad.ChamferOption
			if tc.asymmetric {
				sides, err := decad.Faces(decad.Cylindrical()).Exactly(1).SelectFaces(target)
				require.NoError(t, err)
				require.Len(t, sides[0].Origins(), 1)
				side := sides[0].Origins()[0]
				opts = append(opts, decad.WithAsymmetricChamfer(
					decad.Faces(decad.FaceCreatedBy(side)).Exactly(1), units.Millimeters(0.25)))
			}
			_, err := decad.Chamfer(t.Context(), receiver,
				crossBoreMouth(t, receiver, centerZ, 1, tc.positive), units.Millimeters(tc.setback), opts...)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Equal(t, receiver, doc.Bodies()[len(doc.Bodies())-1])
		})
	}
}
