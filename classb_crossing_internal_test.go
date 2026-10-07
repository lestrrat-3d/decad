package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// requireCrossingBrep asserts a Cut built the crossing reach's brep: a closed
// one-lump body, at least one face carrying a keyed crossing's displacement,
// and a volume within its bound of want.
func requireCrossingBrep(t *testing.T, result *Body, want float64) brepPayload {
	t.Helper()
	bp, ok := result.payload.(brepPayload)
	require.True(t, ok, `the pair builds a brep body, got %T`, result.payload)
	requireClosedTopology(t, result)
	require.InDelta(t, want, result.volume.Value.Base(), result.volume.Bound.Base()+1e-9*want)
	// The mesh closes and its occupied-volume proof covers its distance from
	// the closed form.
	mesh, err := tessellateContext(t.Context(), result, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	held, _ := internalMeshVolumeRat(mesh).Float64()
	require.InDelta(t, want, held, mesh.volSymDiff+1e-9*want)
	report, err := result.doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := report.ForBody(result)
	require.NoError(t, err)
	require.Equal(t, ValidityValid, br.Validity.Outcome)
	return bp
}

// requireKeyedCharge asserts that the faces using keyed crossings carry the
// cut displacement, and that the volume bound covers each such wall's band:
// twice that displacement over the wall's length, over its height.
func requireKeyedCharge(t *testing.T, result *Body, bp brepPayload) {
	t.Helper()
	charged := 0.0
	for _, f := range bp.faces {
		charged = math.Max(charged, f.delta)
	}
	require.Positive(t, charged, `a face meeting a keyed crossing carries its displacement`)
	band := 0.0
	for _, f := range bp.faces {
		if f.planar() || f.delta == 0 {
			continue
		}
		w, err := walkOf(f.wall, nil)
		require.NoError(t, err)
		band = math.Max(band, 2*f.delta*w.Length*(f.z1-f.z0))
	}
	require.GreaterOrEqual(t, result.volume.Bound.Base(), band)
}

func TestClassBCrossingKeyway(t *testing.T) {
	t.Parallel()
	doc := New()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XZ(), -20)
	require.NoError(t, err)
	rod := internalClassBTool(t, doc, w, plane, 20, func(s *sketch.Sketch) {
		c := s.CreatePoint(0, 0)
		s.Fix(c)
		s.CreateCircle(c, 10)
	})
	key := internalBoxBodyAtZ(t, doc, -2, -1, 2, 41, 0, 12)
	result, err := Cut(t.Context(), rod, key)
	require.NoError(t, err)
	// §9's B2: π·100·40 − 40·(2·√96 + 100·asin(0.2)).
	want := math.Pi*100*40 - 40*(2*math.Sqrt(96)+100*math.Asin(0.2))
	bp := requireCrossingBrep(t, result, want)
	requireKeyedCharge(t, result, bp)
	// Two notched caps, the key's floor and two walls, and the rod's wall
	// less the strip the key removes.
	require.Len(t, result.Faces(), 6)
}

func TestClassBCrossingBlindCrossHole(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XZ(), -5)
	require.NoError(t, err)
	drill := internalClassBTool(t, doc, w, plane, 6, func(s *sketch.Sketch) {
		c := s.CreatePoint(20, 10)
		s.Fix(c)
		s.CreateCircle(c, 3)
	})
	result, err := Cut(t.Context(), box, drill)
	require.NoError(t, err)
	// The drill reaches y = 11: 16000 − 9π·11.
	lo, hi := piEnclosed(big.NewRat(16000, 1), big.NewRat(-99, 1))
	_, ok := result.payload.(brepPayload)
	require.True(t, ok, `got %T`, result.payload)
	requireClosedTopology(t, result)
	requireCoversInterval(t, result.volume, lo, hi)
	// The hole's floor is a whole disc at y = 11, and its wall runs from the
	// box's face at y = 0 to it.
	require.Len(t, result.Faces(), 8)
}

func TestClassBCrossingHoleBreakingOut(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	drill := internalDrillAlongY(t, doc, 20, 18, 3)
	result, err := Cut(t.Context(), box, drill)
	require.NoError(t, err)
	// The disc below z = 20, over the box's 20 mm: the segment above the top
	// face is 9·acos(2/3) − 2·√5.
	below := 9*math.Pi - (9*math.Acos(2.0/3) - 2*math.Sqrt(5))
	bp := requireCrossingBrep(t, result, 16000-20*below)
	requireKeyedCharge(t, result, bp)
}

func TestClassBCrossingToolCrossingAWall(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XZ(), -4.65)
	require.NoError(t, err)
	slot := internalClassBTool(t, doc, w, plane, 5.65, func(s *sketch.Sketch) {
		r := s.CreateRectangle(35, 5, 45, 15)
		s.Fix(r.A)
	})
	result, err := Cut(t.Context(), box, slot)
	require.NoError(t, err)
	_, ok := result.payload.(brepPayload)
	require.True(t, ok, `got %T`, result.payload)
	requireClosedTopology(t, result)
	// The slot removes x 35..40 and z 5..15 of the box from y = 0 to where
	// it ends, 4.65 + 5.65 along y.
	end := 4.65 + 5.65
	require.InDelta(t, 16000-5*10*end, result.volume.Value.Base(), result.volume.Bound.Base()+1e-9)
}
