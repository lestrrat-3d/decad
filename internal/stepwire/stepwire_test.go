package stepwire_test

import (
	"bytes"
	"encoding/binary"
	"io"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/internal/stepwire"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRealCylinderRoundTrip(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(decad.New(), s, s.Profiles()[0], decad.Distance{D: units.Millimeters(8), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	var frame bytes.Buffer
	require.NoError(t, stepwire.Write(t.Context(), &frame, body, units.Millimeters(0.1)))
	received, err := stepwire.Read(t.Context(), &frame)
	require.NoError(t, err)
	previous := 0
	for _, tol := range []units.Value{units.Millimeters(0.5), units.Millimeters(0.05)} {
		mesh, err := received.Tessellate(t.Context(), tol)
		require.NoError(t, err)
		require.Greater(t, len(mesh.Triangles), previous)
		previous = len(mesh.Triangles)
		m, err := decad.NewMeshBody(t.Context(), mesh.Vertices, mesh.Triangles)
		require.NoError(t, err)
		require.True(t, m.Report().Closed)
		volume, err := m.Volume()
		require.NoError(t, err)
		require.Greater(t, volume.Value.Base(), 0.8*math.Pi*25*8)
		require.Less(t, volume.Value.Base(), math.Pi*25*8)
		for _, v := range mesh.Vertices {
			require.InDelta(t, 5, math.Hypot(v.X, v.Y), 1e-12)
			require.True(t, v.Z == 0 || v.Z == 8)
		}
	}
}

func TestFacetedFallbackWithCavity(t *testing.T) {
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 10, 10, units.Millimeters(10))
	body, err := decad.ShellClosed(body, units.Millimeters(1)).Do(t.Context())
	require.NoError(t, err)
	var frame bytes.Buffer
	require.NoError(t, stepwire.Write(t.Context(), &frame, body, units.Millimeters(0.1)))
	require.Equal(t, "DCS2", string(frame.Bytes()[:4]))
	received, err := stepwire.Read(t.Context(), &frame)
	require.NoError(t, err)
	mesh, err := received.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	m, err := decad.NewMeshBody(t.Context(), mesh.Vertices, mesh.Triangles)
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, 488, volume.Value.Base(), 1e-9)
}

func TestFrameStreamAndRefusals(t *testing.T) {
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 2, 3, units.Millimeters(4))
	var stream bytes.Buffer
	require.NoError(t, stepwire.Write(t.Context(), &stream, body, units.Millimeters(0.1)))
	frame := bytes.Clone(stream.Bytes())
	require.NoError(t, stepwire.Write(t.Context(), &stream, body, units.Millimeters(0.1)))
	_, err := stepwire.Read(t.Context(), &stream)
	require.NoError(t, err)
	require.Equal(t, len(frame), stream.Len())
	_, err = stepwire.Read(t.Context(), &stream)
	require.NoError(t, err)
	require.Zero(t, stream.Len())
	for name, mutate := range map[string]func([]byte) []byte{
		"magic":              func(b []byte) []byte { b[0] = 0; return b },
		"reserved":           func(b []byte) []byte { b[12] = 1; return b },
		"oversize":           func(b []byte) []byte { binary.LittleEndian.PutUint32(b[8:], 1<<31); return b },
		"short uncompressed": func(b []byte) []byte { binary.LittleEndian.PutUint32(b[8:], 1); return b },
		"bad gzip":           func(b []byte) []byte { b[16] = 0; return b },
		"truncated":          func(b []byte) []byte { return b[:len(b)-1] },
	} {
		t.Run(name, func(t *testing.T) {
			_, err := stepwire.Read(t.Context(), bytes.NewReader(mutate(bytes.Clone(frame))))
			require.Error(t, err)
		})
	}
	_, err = stepwire.Read(t.Context(), &stream)
	require.ErrorIs(t, err, io.EOF)
}
