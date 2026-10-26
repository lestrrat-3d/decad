package apitest_test

import (
	"archive/zip"
	"bytes"
	"context"
	"fmt"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-import-design.md §8's public-surface tests for
// ReadThreeMF. The cube object is 2 mm on a side, so its volume is 8 in the
// model's unit cubed.

const threeMFCubeObject = `<object id="1" type="model"><mesh><vertices>
<vertex x="0" y="0" z="0"/><vertex x="2" y="0" z="0"/><vertex x="2" y="2" z="0"/><vertex x="0" y="2" z="0"/>
<vertex x="0" y="0" z="2"/><vertex x="2" y="0" z="2"/><vertex x="2" y="2" z="2"/><vertex x="0" y="2" z="2"/>
</vertices><triangles>
<triangle v1="0" v2="2" v3="1"/><triangle v1="0" v2="3" v3="2"/><triangle v1="4" v2="5" v3="6"/><triangle v1="4" v2="6" v3="7"/>
<triangle v1="0" v2="1" v3="5"/><triangle v1="0" v2="5" v3="4"/><triangle v1="2" v2="3" v3="7"/><triangle v1="2" v2="7" v3="6"/>
<triangle v1="1" v2="2" v3="6"/><triangle v1="1" v2="6" v3="5"/><triangle v1="3" v2="0" v3="4"/><triangle v1="3" v2="4" v3="7"/>
</triangles></mesh></object>`

// threeMFPackage zips a model part, with the standard relationship, into a package.
func threeMFPackage(t *testing.T, model string) []byte {
	t.Helper()
	var buf bytes.Buffer
	zw := zip.NewWriter(&buf)
	for name, content := range map[string]string{
		"_rels/.rels": `<?xml version="1.0"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">` +
			`<Relationship Id="r0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" Target="/3D/3dmodel.model"/></Relationships>`,
		"3D/3dmodel.model": model,
	} {
		w, err := zw.Create(name)
		require.NoError(t, err)
		_, err = w.Write([]byte(content))
		require.NoError(t, err)
	}
	require.NoError(t, zw.Close())
	return buf.Bytes()
}

// threeMFModel wraps resources and a build in a model of the given unit.
func threeMFModel(unit, resources, build string) string {
	return fmt.Sprintf(`<?xml version="1.0"?><model unit="%s" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">`+
		`<resources>%s</resources><build>%s</build></model>`, unit, resources, build)
}

func readThreeMF(t *testing.T, model string, opts ...decad.MeshImportOption) (*decad.MeshBody, error) {
	t.Helper()
	return decad.ReadThreeMF(t.Context(), bytes.NewReader(threeMFPackage(t, model)), opts...)
}

func TestReadThreeMFReadsACube(t *testing.T) {
	t.Parallel()
	m, err := readThreeMF(t, threeMFModel("millimeter", threeMFCubeObject, `<item objectid="1"/>`))
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	require.Equal(t, 8, m.Report().Vertices)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, 8, volume.Value.Base(), 1e-12)
}

func TestReadThreeMFRoundTripsAnExportedBody(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	var out bytes.Buffer
	require.NoError(t, export.ThreeMF(t.Context(), &out, body, units.Millimeters(0.1)))
	m, err := decad.ReadThreeMF(t.Context(), &out)
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, 6000, volume.Value.Base(), 1e-6)
	box := m.Bounds()
	require.True(t, box.Max.Equal(r3.NewVec(20, 30, 10), 1e-9))
}

func TestReadThreeMFScalesTheModelUnit(t *testing.T) {
	t.Parallel()
	m, err := readThreeMF(t, threeMFModel("inch", threeMFCubeObject, `<item objectid="1"/>`))
	require.NoError(t, err)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, 8*25.4*25.4*25.4, volume.Value.Base(), 1e-6)
	m, err = readThreeMF(t, threeMFModel("micron", threeMFCubeObject, `<item objectid="1"/>`))
	require.NoError(t, err)
	require.True(t, m.Bounds().Max.Equal(r3.NewVec(0.002, 0.002, 0.002), 1e-12))
}

func TestReadThreeMFAppliesTheItemTransform(t *testing.T) {
	t.Parallel()
	// A quarter turn about +Z (a row vector turns as x' = -y, y' = x) and a move of 10 along X.
	m, err := readThreeMF(t, threeMFModel("millimeter", threeMFCubeObject,
		`<item objectid="1" transform="0 1 0 -1 0 0 0 0 1 10 0 0"/>`))
	require.NoError(t, err)
	box := m.Bounds()
	require.True(t, box.Min.Equal(r3.NewVec(8, 0, 0), 1e-9), "min %v", box.Min)
	require.True(t, box.Max.Equal(r3.NewVec(10, 2, 2), 1e-9), "max %v", box.Max)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, 8, volume.Value.Base(), 1e-9)
}

func TestReadThreeMFKeepsAMirroredCubeRightWayOut(t *testing.T) {
	t.Parallel()
	m, err := readThreeMF(t, threeMFModel("millimeter", threeMFCubeObject,
		`<item objectid="1" transform="-1 0 0 0 1 0 0 0 1 0 0 0"/>`))
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	require.False(t, m.Report().Inverted, "a reflection reverses the winding so the cube stays outward")
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, 8, volume.Value.Base(), 1e-9)
	require.InDelta(t, -2, m.Bounds().Min.X, 1e-12)
}

func TestReadThreeMFFollowsComponents(t *testing.T) {
	t.Parallel()
	resources := threeMFCubeObject +
		`<object id="2"><components><component objectid="1"/><component objectid="1" transform="1 0 0 0 1 0 0 0 1 5 0 0"/></components></object>`
	m, err := readThreeMF(t, threeMFModel("millimeter", resources, `<item objectid="2" transform="0 1 0 -1 0 0 0 0 1 0 0 3"/>`))
	require.NoError(t, err)
	require.Equal(t, 2, m.Report().Shells)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, 16, volume.Value.Base(), 1e-9)
	box := m.Bounds()
	// The second cube moves 5 along X first, then the item's quarter turn takes
	// that to +Y and lifts both 3: the order of the two transforms shows.
	require.True(t, box.Min.Equal(r3.NewVec(-2, 0, 3), 1e-9), "min %v", box.Min)
	require.True(t, box.Max.Equal(r3.NewVec(0, 7, 5), 1e-9), "max %v", box.Max)
}

func TestReadThreeMFSkipsSupports(t *testing.T) {
	t.Parallel()
	support := strings.Replace(threeMFCubeObject, `id="1" type="model"`, `id="2" type="support"`, 1)
	m, err := readThreeMF(t, threeMFModel("millimeter", threeMFCubeObject+support, `<item objectid="1"/><item objectid="2"/>`))
	require.NoError(t, err)
	require.Equal(t, 12, m.Report().Triangles)
}

func TestReadThreeMFRefusals(t *testing.T) {
	t.Parallel()
	build := `<item objectid="1"/>`
	cases := []struct {
		name  string
		model string
		want  error
	}{
		{"a scaling transform", threeMFModel("millimeter", threeMFCubeObject, `<item objectid="1" transform="2 0 0 0 2 0 0 0 2 0 0 0"/>`), decad.ErrUnsupported},
		{"a transform of eleven numbers", threeMFModel("millimeter", threeMFCubeObject, `<item objectid="1" transform="1 0 0 0 1 0 0 0 1 0 0"/>`), decad.ErrMalformedMesh},
		{"an unknown unit", threeMFModel("furlong", threeMFCubeObject, build), decad.ErrMalformedMesh},
		{"a missing object", threeMFModel("millimeter", threeMFCubeObject, `<item objectid="9"/>`), decad.ErrMalformedMesh},
		{"no build item", threeMFModel("millimeter", threeMFCubeObject, ``), decad.ErrMalformedMesh},
		{"a triangle index out of range", threeMFModel("millimeter", strings.Replace(threeMFCubeObject, `v1="0" v2="2" v3="1"`, `v1="0" v2="2" v3="9"`, 1), build), decad.ErrMalformedMesh},
		{"a vertex that is not a number", threeMFModel("millimeter", strings.Replace(threeMFCubeObject, `x="2" y="0" z="0"`, `x="two" y="0" z="0"`, 1), build), decad.ErrMalformedMesh},
		{"xml that is cut off", `<model unit="millimeter"><resources><object id="1"`, decad.ErrMalformedMesh},
		{"components that loop", threeMFModel("millimeter", `<object id="1"><components><component objectid="1"/></components></object>`, build), decad.ErrMalformedMesh},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, err := readThreeMF(t, tc.model)
			require.ErrorIs(t, err, tc.want)
		})
	}
	t.Run("not a zip", func(t *testing.T) {
		t.Parallel()
		_, err := decad.ReadThreeMF(t.Context(), strings.NewReader("this is not a package"))
		require.ErrorIs(t, err, decad.ErrMalformedMesh)
	})
	t.Run("no model part", func(t *testing.T) {
		t.Parallel()
		var buf bytes.Buffer
		zw := zip.NewWriter(&buf)
		w, err := zw.Create("readme.txt")
		require.NoError(t, err)
		_, err = w.Write([]byte("hello"))
		require.NoError(t, err)
		require.NoError(t, zw.Close())
		_, err = decad.ReadThreeMF(t.Context(), &buf)
		require.ErrorIs(t, err, decad.ErrMalformedMesh)
	})
	t.Run("past the triangle limit", func(t *testing.T) {
		t.Parallel()
		_, err := readThreeMF(t, threeMFModel("millimeter", threeMFCubeObject, build), decad.WithMaxMeshTriangles(5))
		require.ErrorIs(t, err, decad.ErrMeshTooLarge)
	})
	t.Run("past the byte limit", func(t *testing.T) {
		t.Parallel()
		_, err := readThreeMF(t, threeMFModel("millimeter", threeMFCubeObject, build), decad.WithMaxMeshBytes(100))
		require.ErrorIs(t, err, decad.ErrMeshTooLarge)
	})
	t.Run("nil arguments and a canceled context", func(t *testing.T) {
		t.Parallel()
		var nilContext context.Context
		_, err := decad.ReadThreeMF(nilContext, strings.NewReader(""))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		_, err = decad.ReadThreeMF(t.Context(), nil)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		canceled, cancel := context.WithCancel(t.Context())
		cancel()
		_, err = decad.ReadThreeMF(canceled, bytes.NewReader(threeMFPackage(t, threeMFModel("millimeter", threeMFCubeObject, build))))
		require.ErrorIs(t, err, context.Canceled)
	})
}
