package export_test

import (
	"archive/zip"
	"bytes"
	"context"
	"encoding/xml"
	"errors"
	"io"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type threeMFVertex struct {
	X float64 `xml:"x,attr"`
	Y float64 `xml:"y,attr"`
	Z float64 `xml:"z,attr"`
}

type threeMFTriangle struct {
	V1 int `xml:"v1,attr"`
	V2 int `xml:"v2,attr"`
	V3 int `xml:"v3,attr"`
}

type failingWriter struct{ err error }

func (w failingWriter) Write([]byte) (int, error) { return 0, w.err }

type threeMFModel struct {
	XMLName xml.Name `xml:"model"`
	Unit    string   `xml:"unit,attr"`
	Object  struct {
		ID        int               `xml:"id,attr"`
		Type      string            `xml:"type,attr"`
		Vertices  []threeMFVertex   `xml:"mesh>vertices>vertex"`
		Triangles []threeMFTriangle `xml:"mesh>triangles>triangle"`
	} `xml:"resources>object"`
	Build struct {
		Item struct {
			ObjectID int `xml:"objectid,attr"`
		} `xml:"item"`
	} `xml:"build"`
}

func threeMFPart(t *testing.T, archive *zip.Reader, name string) []byte {
	t.Helper()
	for _, file := range archive.File {
		if file.Name != name {
			continue
		}
		r, err := file.Open()
		require.NoError(t, err)
		data, err := io.ReadAll(r)
		require.NoError(t, err)
		require.NoError(t, r.Close())
		return data
	}
	require.FailNow(t, "missing 3MF part", name)
	return nil
}

func readThreeMFModel(t *testing.T, data []byte) threeMFModel {
	t.Helper()
	archive, err := zip.NewReader(bytes.NewReader(data), int64(len(data)))
	require.NoError(t, err)
	var model threeMFModel
	require.NoError(t, xml.Unmarshal(threeMFPart(t, archive, "3D/3dmodel.model"), &model))
	return model
}

func TestThreeMFBoxPackage(t *testing.T) {
	t.Parallel()
	body := box(t, false)
	var out bytes.Buffer
	require.NoError(t, export.ThreeMF(t.Context(), &out, body, units.Millimeters(0.1)))
	archive, err := zip.NewReader(bytes.NewReader(out.Bytes()), int64(out.Len()))
	require.NoError(t, err)
	require.Len(t, archive.File, 3)
	require.Equal(t, "[Content_Types].xml", archive.File[0].Name)
	require.Equal(t, "_rels/.rels", archive.File[1].Name)
	require.Equal(t, "3D/3dmodel.model", archive.File[2].Name)
	require.Contains(t, string(threeMFPart(t, archive, "[Content_Types].xml")),
		"application/vnd.ms-package.3dmanufacturing-3dmodel+xml")
	require.Contains(t, string(threeMFPart(t, archive, "_rels/.rels")),
		`Target="/3D/3dmodel.model"`)

	var model threeMFModel
	require.NoError(t, xml.Unmarshal(threeMFPart(t, archive, "3D/3dmodel.model"), &model))
	require.Equal(t, "http://schemas.microsoft.com/3dmanufacturing/core/2015/02", model.XMLName.Space)
	require.Equal(t, "millimeter", model.Unit)
	require.Equal(t, 1, model.Object.ID)
	require.Equal(t, "model", model.Object.Type)
	require.Equal(t, 1, model.Build.Item.ObjectID)
	require.Len(t, model.Object.Vertices, 8)
	require.Len(t, model.Object.Triangles, 12)
	points := map[threeMFVertex]struct{}{}
	for _, v := range model.Object.Vertices {
		points[v] = struct{}{}
	}
	for _, p := range []threeMFVertex{{0, 0, 0}, {2, 0, 0}, {0, 3, 0}, {2, 3, 0},
		{0, 0, 4}, {2, 0, 4}, {0, 3, 4}, {2, 3, 4}} {
		_, ok := points[p]
		require.Truef(t, ok, "missing corner %v", p)
	}
	var sixTimesVolume float64
	for _, tri := range model.Object.Triangles {
		a := model.Object.Vertices[tri.V1]
		b := model.Object.Vertices[tri.V2]
		c := model.Object.Vertices[tri.V3]
		sixTimesVolume += a.X*(b.Y*c.Z-b.Z*c.Y) + a.Y*(b.Z*c.X-b.X*c.Z) + a.Z*(b.X*c.Y-b.Y*c.X)
	}
	require.InDelta(t, 24, sixTimesVolume/6, 1e-12)
}

func TestThreeMFSheetAndVerification(t *testing.T) {
	t.Parallel()
	body := box(t, true)
	var out bytes.Buffer
	require.NoError(t, export.ThreeMF(t.Context(), &out, body, units.Millimeters(0.1)))
	model := readThreeMFModel(t, out.Bytes())
	require.Equal(t, "surface", model.Object.Type)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	require.Len(t, model.Object.Vertices, len(mesh.Vertices()))
	require.Len(t, model.Object.Triangles, len(mesh.Triangles()))
	for i, v := range mesh.Vertices() {
		require.Equal(t, threeMFVertex{v.X, v.Y, v.Z}, model.Object.Vertices[i])
	}
	for i, tri := range mesh.Triangles() {
		require.Equal(t, threeMFTriangle{tri[0], tri[1], tri[2]}, model.Object.Triangles[i])
	}

	var stronger bytes.Buffer
	require.NoError(t, export.ThreeMF(t.Context(), &stronger, body, units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyBoundary)))
	require.Equal(t, out.Bytes(), stronger.Bytes())
}

func TestThreeMFChordToleranceAndDeterminism(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	body, err := decad.New().Extrude(s, profile, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)

	var coarse, again, fine bytes.Buffer
	require.NoError(t, export.ThreeMF(t.Context(), &coarse, body, units.Millimeters(0.5)))
	require.NoError(t, export.ThreeMF(t.Context(), &again, body, units.Millimeters(0.5)))
	require.Equal(t, coarse.Bytes(), again.Bytes())
	require.NoError(t, export.ThreeMF(t.Context(), &fine, body, units.Millimeters(0.05)))
	require.Greater(t, len(readThreeMFModel(t, fine.Bytes()).Object.Triangles),
		len(readThreeMFModel(t, coarse.Bytes()).Object.Triangles))
}

func TestThreeMFErrorsLeaveWriterUntouched(t *testing.T) {
	t.Parallel()
	body := box(t, false)
	var out bytes.Buffer
	out.WriteString("original")
	tol := units.Millimeters(0.1)
	var nilContext context.Context
	require.ErrorIs(t, export.ThreeMF(nilContext, &out, body, tol), decad.ErrDegenerate)
	require.ErrorIs(t, export.ThreeMF(t.Context(), nil, body, tol), decad.ErrDegenerate)
	require.ErrorIs(t, export.ThreeMF(t.Context(), &out, nil, tol), decad.ErrDegenerate)
	require.ErrorIs(t, export.ThreeMF(t.Context(), &out, body, units.Millimeters(0)), decad.ErrDegenerate)
	require.ErrorIs(t, export.ThreeMF(t.Context(), &out, body, units.Degrees(1)), decad.ErrUnitKind)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.ErrorIs(t, export.ThreeMF(ctx, &out, body, tol), context.Canceled)
	require.Equal(t, "original", out.String())

	writeErr := errors.New("writer failed")
	require.ErrorIs(t, export.ThreeMF(t.Context(), failingWriter{writeErr}, body, tol), writeErr)
}
