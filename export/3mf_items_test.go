package export_test

import (
	"archive/zip"
	"bytes"
	"encoding/xml"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type threeMFItemsModel struct {
	Objects []struct {
		ID        int               `xml:"id,attr"`
		Type      string            `xml:"type,attr"`
		Name      string            `xml:"name,attr"`
		Vertices  []threeMFVertex   `xml:"mesh>vertices>vertex"`
		Triangles []threeMFTriangle `xml:"mesh>triangles>triangle"`
	} `xml:"resources>object"`
	Items []struct {
		ObjectID int `xml:"objectid,attr"`
	} `xml:"build>item"`
}

func readItems(t *testing.T, data []byte) threeMFItemsModel {
	t.Helper()
	archive, err := zip.NewReader(bytes.NewReader(data), int64(len(data)))
	require.NoError(t, err)
	var model threeMFItemsModel
	require.NoError(t, xml.Unmarshal(threeMFPart(t, archive, "3D/3dmodel.model"), &model))
	return model
}

func translation(t *testing.T, v r3.Vec) r3.Transform {
	t.Helper()
	tr, err := r3.Translation(v)
	require.NoError(t, err)
	return tr
}

func TestThreeMFItems(t *testing.T) {
	t.Parallel()
	body := box(t, false) // 2 x 3 x 4 at the origin
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(90))
	require.NoError(t, err)
	turnThenLift, err := turn.Then(translation(t, r3.Vec{X: 10, Z: 5}))
	require.NoError(t, err)
	items := []export.ThreeMFItem{
		{Name: `left & "one"`, Body: body, Placement: translation(t, r3.Vec{X: 100})},
		{Name: "second", Body: body, Placement: turnThenLift},
	}
	var out bytes.Buffer
	require.NoError(t, export.ThreeMFItems(t.Context(), &out, items, units.Millimeters(0.1)))
	model := readItems(t, out.Bytes())
	require.Len(t, model.Objects, 2)
	require.Len(t, model.Items, 2)
	require.Equal(t, 1, model.Items[0].ObjectID)
	require.Equal(t, 2, model.Items[1].ObjectID)
	require.Equal(t, `left & "one"`, model.Objects[0].Name)
	require.Equal(t, "second", model.Objects[1].Name)
	for _, o := range model.Objects {
		require.Equal(t, "model", o.Type)
		require.Len(t, o.Vertices, 8)
		require.Len(t, o.Triangles, 12)
	}

	// The first box moves 100 mm along X: x spans 100..102.
	lo, hi := bounds(model.Objects[0].Vertices)
	require.Equal(t, r3.Vec{X: 100}, lo)
	require.Equal(t, r3.Vec{X: 102, Y: 3, Z: 4}, hi)
	// The second turns 90 degrees about Z (x, y) -> (-y, x), then moves by (10, 0, 5).
	lo, hi = bounds(model.Objects[1].Vertices)
	require.InDelta(t, 7, lo.X, 1e-9)
	require.InDelta(t, 0, lo.Y, 1e-9)
	require.InDelta(t, 5, lo.Z, 1e-9)
	require.InDelta(t, 10, hi.X, 1e-9)
	require.InDelta(t, 2, hi.Y, 1e-9)
	require.InDelta(t, 9, hi.Z, 1e-9)
	// A proper motion keeps the winding: the volume stays +24.
	for _, o := range model.Objects {
		var six float64
		for _, tri := range o.Triangles {
			a, b, c := o.Vertices[tri.V1], o.Vertices[tri.V2], o.Vertices[tri.V3]
			six += a.X*(b.Y*c.Z-b.Z*c.Y) + a.Y*(b.Z*c.X-b.X*c.Z) + a.Z*(b.X*c.Y-b.Y*c.X)
		}
		require.InDelta(t, 24, six/6, 1e-9)
	}

	// The same input writes the same bytes.
	var again bytes.Buffer
	require.NoError(t, export.ThreeMFItems(t.Context(), &again, items, units.Millimeters(0.1)))
	require.Equal(t, out.Bytes(), again.Bytes())
}

func bounds(vs []threeMFVertex) (r3.Vec, r3.Vec) {
	lo := r3.Vec{X: vs[0].X, Y: vs[0].Y, Z: vs[0].Z}
	hi := lo
	for _, v := range vs {
		lo = r3.Vec{X: min(lo.X, v.X), Y: min(lo.Y, v.Y), Z: min(lo.Z, v.Z)}
		hi = r3.Vec{X: max(hi.X, v.X), Y: max(hi.Y, v.Y), Z: max(hi.Z, v.Z)}
	}
	return lo, hi
}

func TestThreeMFItemsRefusals(t *testing.T) {
	t.Parallel()
	body := box(t, false)
	id := r3.Identity()
	plane, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	mirror, err := r3.Reflection(plane)
	require.NoError(t, err)
	good := []export.ThreeMFItem{{Name: "a", Body: body, Placement: id}}
	var out bytes.Buffer
	for name, items := range map[string][]export.ThreeMFItem{
		"none":       nil,
		"nil body":   {{Name: "a", Placement: id}},
		"reflection": {{Name: "a", Body: body, Placement: mirror}},
		"zero value": {{Name: "a", Body: body}},
	} {
		require.ErrorIs(t, export.ThreeMFItems(t.Context(), &out, items, units.Millimeters(0.1)), decad.ErrDegenerate, name)
	}
	require.Zero(t, out.Len(), "a refusal leaves the writer untouched")
	require.ErrorIs(t, export.ThreeMFItems(t.Context(), nil, good, units.Millimeters(0.1)), decad.ErrDegenerate)
	require.ErrorIs(t, export.ThreeMFItems(nil, &out, good, units.Millimeters(0.1)), decad.ErrDegenerate) //nolint:staticcheck
	require.ErrorIs(t, export.ThreeMFItems(t.Context(), &out, good, units.Degrees(1)), decad.ErrUnitKind)
}
