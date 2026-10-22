package spiralsector_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The scene is a 45° cone about the X axis, x + rho = coneLevel, seen near the
// y axis. The blank's band lies on the cone below rho 3; a tooth cap lies on it
// above. The proxy lifted every cap vertex by capShift in Y, which is 0.0226 mm
// along the cone's normal and 0.0226 mm along the cone.
const (
	coneLevel = 10.0
	capShift  = 0.032
	bandGroup = 0
	capGroup  = 1
	blankOnly = 1 // held groups below this are the blank's
)

// onCone is the point of the cone at (y, z).
func onCone(y, z float64) r3.Vec {
	return r3.Vec{X: coneLevel - math.Hypot(y, z), Y: y, Z: z}
}

// scene collects the vertices and facets of one test mesh. Vertices are shared
// by position.
type scene struct {
	proxy, home []r3.Vec
	bound       []float64
	tris        [][3]int
	src         []int
	index       map[r3.Vec]int
}

func newScene() *scene { return &scene{index: map[r3.Vec]int{}} }

// vertex returns the vertex at home position p, as the proxy holds it: lifted
// when lift is set.
func (s *scene) vertex(p r3.Vec, lift bool) int {
	q := p
	if lift {
		q.Y += capShift
	}
	if i, ok := s.index[q]; ok {
		return i
	}
	s.index[q] = len(s.proxy)
	s.proxy, s.home = append(s.proxy, q), append(s.home, p)
	s.bound = append(s.bound, 0.05)
	return len(s.proxy) - 1
}

// grid adds the facets of a grid over y in ys and z in zs, wound so the cone's
// outward normal is the facets', in group g.
func (s *scene) grid(ys, zs []float64, g int, lift bool) {
	for i := 0; i+1 < len(ys); i++ {
		for j := 0; j+1 < len(zs); j++ {
			a := s.vertex(onCone(ys[i], zs[j]), lift)
			b := s.vertex(onCone(ys[i+1], zs[j]), lift)
			c := s.vertex(onCone(ys[i+1], zs[j+1]), lift)
			d := s.vertex(onCone(ys[i], zs[j+1]), lift)
			s.tris = append(s.tris, [3]int{a, b, c}, [3]int{a, c, d})
			s.src = append(s.src, g, g)
		}
	}
}

// capHome is the CapHome of the scene, its home bounds a fifth of the proxy's.
func (s *scene) capHome(caps ...int) *spiralsector.CapHome {
	bound := make([]float64, len(s.bound))
	for i, b := range s.bound {
		bound[i] = b / 5
	}
	h := spiralsector.NewCapHome(s.proxy, s.home, bound, s.tris)
	for _, g := range caps {
		h.WithCap(g, bandGroup)
	}
	return h
}

func (s *scene) mesh() spiralsector.Mesh {
	return spiralsector.Mesh{
		Verts: append([]r3.Vec(nil), s.proxy...), Tris: s.tris, Src: s.src,
		VertexBound: append([]float64(nil), s.bound...),
	}
}

// off is how far p lies from the cone along its normal.
func off(p r3.Vec) float64 {
	return (p.X + math.Hypot(p.Y, p.Z) - coneLevel) / math.Sqrt2
}

func TestRestoreCaps(t *testing.T) {
	t.Parallel()

	ys, zs := []float64{3.2, 3.6, 4.0}, []float64{-0.2, 0, 0.2}
	band := func(s *scene) { s.grid([]float64{2.0, 2.5, 3.0}, []float64{-0.2, 0, 0.2}, bandGroup, false) }

	t.Run("lowers a lifted cap onto its cone", func(t *testing.T) {
		t.Parallel()
		s := newScene()
		band(s)
		s.grid(ys, zs, capGroup, true)
		require.Greater(t, off(s.proxy[len(s.proxy)-1]), 0.02, "the proxy cap is lifted")

		got, ok, err := spiralsector.RestoreCaps(t.Context(), s.capHome(capGroup), s.mesh(), blankOnly)
		require.NoError(t, err)
		require.True(t, ok)
		require.Len(t, got.Verts, len(s.proxy))
		require.Positive(t, got.VolumeCharge)
		require.Positive(t, got.AreaCharge)

		lowered := 0
		for f, tri := range s.tris {
			for _, v := range tri {
				p := got.Verts[v]
				if s.src[f] == capGroup {
					require.InDelta(t, spiralsector.CapStand, off(p), 1e-3, "the cap vertex at %v stands on the cone", p)
					lowered++
				} else {
					require.Equal(t, s.proxy[v], p, "a blank vertex stays")
				}
			}
		}
		require.Positive(t, lowered)
	})

	t.Run("does not modify the held vertices", func(t *testing.T) {
		t.Parallel()
		s := newScene()
		band(s)
		s.grid(ys, zs, capGroup, true)
		m := s.mesh()
		held := append([]r3.Vec(nil), m.Verts...)
		_, ok, err := spiralsector.RestoreCaps(t.Context(), s.capHome(capGroup), m, blankOnly)
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, held, m.Verts)
	})

	t.Run("leaves a cap it cannot restore soundly and restores the others", func(t *testing.T) {
		t.Parallel()
		s := newScene()
		band(s)
		s.grid(ys, zs, capGroup, true)
		const unsound = 2
		s.grid([]float64{6.0, 6.4}, zs, unsound, true)
		m := s.mesh()
		// One vertex of the second cap has a bound below its source bound.
		m.VertexBound[len(m.VertexBound)-1] = 0
		got, ok, err := spiralsector.RestoreCaps(t.Context(), s.capHome(capGroup, unsound), m, blankOnly)
		require.NoError(t, err)
		require.True(t, ok)
		restored, kept := 0, 0
		for f, tri := range s.tris {
			for _, v := range tri {
				p := got.Verts[v]
				switch s.src[f] {
				case capGroup:
					require.InDelta(t, spiralsector.CapStand, off(p), 1e-3)
					restored++
				case unsound:
					require.Equal(t, s.proxy[v], p, "the second cap keeps its proxy positions")
					kept++
				}
			}
		}
		require.Positive(t, restored)
		require.Positive(t, kept)
	})

	t.Run("refuses what it cannot restore soundly", func(t *testing.T) {
		t.Parallel()
		s := newScene()
		band(s)
		s.grid(ys, zs, capGroup, true)

		_, ok, err := spiralsector.RestoreCaps(t.Context(), nil, s.mesh(), blankOnly)
		require.NoError(t, err)
		require.False(t, ok, "no home")

		_, ok, err = spiralsector.RestoreCaps(t.Context(), s.capHome(), s.mesh(), blankOnly)
		require.NoError(t, err)
		require.False(t, ok, "no cap group")

		_, ok, err = spiralsector.RestoreCaps(t.Context(), spiralsector.NewCapHome(s.proxy, s.proxy, s.bound, s.tris).
			WithCap(capGroup, bandGroup), s.mesh(), blankOnly)
		require.NoError(t, err)
		require.False(t, ok, "nothing moved")

		h := s.capHome(capGroup)
		h.Append(s.capHome(capGroup))
		_, ok, err = spiralsector.RestoreCaps(t.Context(), h, s.mesh(), blankOnly)
		require.NoError(t, err)
		require.False(t, ok, "two moved vertices at one position")

		low := s.mesh()
		low.VertexBound = make([]float64, len(low.Verts))
		_, ok, err = spiralsector.RestoreCaps(t.Context(), s.capHome(capGroup), low, blankOnly)
		require.NoError(t, err)
		require.False(t, ok, "every cap holds a vertex bound below the bound of the position it takes")
	})

	t.Run("pairs only slices of one length", func(t *testing.T) {
		t.Parallel()
		s := newScene()
		band(s)
		s.grid(ys, zs, capGroup, true)
		require.Zero(t, spiralsector.NewCapHome(s.proxy, s.home[:3], s.bound, s.tris).Len())
		require.Equal(t, len(ys)*len(zs), spiralsector.NewCapHome(s.proxy, s.home, s.bound, s.tris).Len(),
			"every cap vertex moved and no band vertex")
	})
}
