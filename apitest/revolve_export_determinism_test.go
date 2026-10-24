package apitest_test

import (
	"bytes"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func requireRevolveExportsByteIdentical(t *testing.T, body *decad.Body) {
	t.Helper()
	var stl1, stl2, obj1, obj2 bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &stl1, body, units.Millimeters(0.1)))
	require.NoError(t, export.STL(t.Context(), &stl2, body, units.Millimeters(0.1)))
	require.Equal(t, stl1.String(), stl2.String())
	require.NotEmpty(t, stl1.String())
	require.NoError(t, export.OBJ(t.Context(), &obj1, body, units.Millimeters(0.1)))
	require.NoError(t, export.OBJ(t.Context(), &obj2, body, units.Millimeters(0.1)))
	require.Equal(t, obj1.String(), obj2.String())
	require.NotEmpty(t, obj1.String())
}
