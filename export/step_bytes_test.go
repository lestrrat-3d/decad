package export_test

import (
	"crypto/sha256"
	"encoding/hex"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// stepHash is the SHA-256 of a Part 21 file without its FILE_NAME line, the
// one header line that carries the timestamp.
func stepHash(t *testing.T, body *decad.Body, tol units.Value) string {
	t.Helper()
	f, err := export.NewSTEPFile(t.Context(), body, tol, header())
	require.NoError(t, err)
	data, err := f.Marshal()
	require.NoError(t, err)
	var kept []string
	for _, line := range strings.SplitAfter(string(data), "\n") {
		if !strings.HasPrefix(line, "FILE_NAME(") {
			kept = append(kept, line)
		}
	}
	sum := sha256.Sum256([]byte(strings.Join(kept, "")))
	return hex.EncodeToString(sum[:])
}

func facetedCutPlate(t *testing.T) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	doc := decad.New()
	plateSketch, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := plateSketch.CreateRectangle(0, 0, 30, 40)
	plateSketch.Fix(rect.A)
	_, err = plateSketch.Solve(t.Context())
	require.NoError(t, err)
	plate, err := decad.Extrude(t.Context(), doc, plateSketch, plateSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)

	toolSketch, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	centre := toolSketch.CreatePoint(20, 15)
	toolSketch.Fix(centre)
	toolSketch.CreateCircle(centre, 5)
	_, err = toolSketch.Solve(t.Context())
	require.NoError(t, err)
	pin, err := decad.Extrude(t.Context(), doc, toolSketch, toolSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	down, err := r3.Translation(r3.Vec{Z: -6})
	require.NoError(t, err)
	tool, err := decad.Transform(t.Context(), pin, down)
	require.NoError(t, err)
	body, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	return body
}

// TestNewSTEPFileBytesArePinned pins the exact output of NewSTEPFile for the
// analytic arm, the faceted arm and a cavity, so a refactor of the per-body
// writer cannot change a byte. The hashes were captured before the writer was
// split into planSTEPBody and addSTEPSolids.
func TestNewSTEPFileBytesArePinned(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name string
		body func(*testing.T) *decad.Body
		tol  units.Value
		want string
		// The faceted mesh has different pinned bytes on macOS.
		platformWant map[string]string
	}{
		{
			name: "AnalyticBox",
			body: func(t *testing.T) *decad.Body { return box(t, false) },
			tol:  units.Millimeters(0.1),
			want: "2a0b0decbc4b08da870bbd7fbd8cad92a36c39d90648ef085beef569740e8088",
		},
		{
			name: "FacetedBoolean",
			body: facetedCutPlate,
			tol:  units.Millimeters(0.1),
			want: "6e071286f415ab18c2b46a91a36248f7b6ca2548bb702318a0e4b356b372abc9",
			platformWant: map[string]string{
				"darwin": "21edf38d638363e6c3dcdfe9ebdea5f12a4c7b94c0392b63818ddf977c73926c",
			},
		},
		{
			name: "CavityCube",
			body: cavityCube,
			tol:  units.Millimeters(0.1),
			want: "f4763c9e1135ae40826492b858fb8443c332f4bfb8c9528540c6bc35441e5653",
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			want := c.want
			if platformWant, ok := c.platformWant[runtime.GOOS]; ok {
				want = platformWant
			}
			require.Equal(t, want, stepHash(t, c.body(t), c.tol))
		})
	}
}
