package shellseparation

import (
	"math"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// octagonShell builds a prism along X from a fixed octagon (no trig, so the
// vertices are identical on every architecture), offset by (0, dy, 0).
func octagonShell(rings int, dy float64) Shell {
	ring := [8][2]float64{{1, 0}, {0.5, 0.75}, {0, 1}, {-0.5, 0.75}, {-1, 0}, {-0.5, -0.75}, {0, -1}, {0.5, -0.75}}
	s := Shell{MeshBound: 1.0 / 1024}
	for r := 0; r <= rings; r++ {
		x := 10 * float64(r) / float64(rings)
		for _, p := range ring {
			s.Verts = append(s.Verts, r3.Vec{X: x, Y: dy + p[0], Z: p[1]})
			s.VertexBound = append(s.VertexBound, 1.0/1024)
		}
	}
	for r := range rings {
		for k := range 8 {
			a, b := r*8+k, r*8+(k+1)%8
			c, d := (r+1)*8+k, (r+1)*8+(k+1)%8
			s.Tris = append(s.Tris, [3]int{a, b, c}, [3]int{b, d, c})
		}
	}
	return s
}

// TestSlabsClearancePinnedBits pins the clearance bits recorded from the
// implementation that tested every triangle's vertices per slab.
func TestSlabsClearancePinnedBits(t *testing.T) {
	for _, tc := range []struct {
		name  string
		extra float64
		bits  uint64
	}{
		{"plain", 0, 0x3fefeffffffffffe},
		{"axial", 0.25, 0x3fdfdffffffffffb},
	} {
		pa, pb := octagonShell(20, 0), octagonShell(20, 3)
		got, ok := slabsClearance(t.Context(), pa, pb, tc.extra)
		require.True(t, ok, tc.name)
		require.Equal(t, tc.bits, math.Float64bits(got), "%s: got %v", tc.name, got)
	}
}

// slabFacetsReference is the straightforward per-triangle vertex-loading
// selection that slabFacets replaced.
func slabFacetsReference(p Shell, lo, hi, axialExtra float64) ([]int, r3.Vec, bool) {
	reach := p.MeshBound
	if axialExtra > 0 {
		reach = proofbound.AbsSumUpper(reach, axialExtra)
	}
	searchLo := math.Nextafter(lo-reach, math.Inf(-1))
	searchHi := math.Nextafter(hi+reach, math.Inf(1))
	selected := make([]int, 0, len(p.Tris)/8)
	center := r3.Vec{}
	for i, tri := range p.Tris {
		a, b, c := p.Verts[tri[0]], p.Verts[tri[1]], p.Verts[tri[2]]
		if max(a.X, b.X, c.X) < searchLo || min(a.X, b.X, c.X) > searchHi {
			continue
		}
		selected = append(selected, i)
		center = center.Add(a).Add(b).Add(c)
	}
	if len(selected) == 0 {
		return nil, r3.Vec{}, true
	}
	center = center.Scale(1 / float64(3*len(selected)))
	return selected, center, proofbound.FiniteVec(center)
}

func TestSlabFacetsMatchesReference(t *testing.T) {
	rng := rand.New(rand.NewSource(7))
	for trial := range 200 {
		nv := 3 + rng.Intn(40)
		p := Shell{MeshBound: rng.Float64() * 0.3}
		for range nv {
			p.Verts = append(p.Verts, r3.Vec{X: rng.Float64()*10 - 5, Y: rng.NormFloat64(), Z: rng.NormFloat64()})
		}
		for range 1 + rng.Intn(80) {
			p.Tris = append(p.Tris, [3]int{rng.Intn(nv), rng.Intn(nv), rng.Intn(nv)})
		}
		extra := 0.0
		if trial%2 == 1 {
			extra = rng.Float64() * 0.2
		}
		xr := triangleXRanges(p)
		for range 20 {
			lo := rng.Float64()*12 - 6
			hi := lo + rng.Float64()*2
			wantF, wantC, wantOK := slabFacetsReference(p, lo, hi, extra)
			gotF, gotC, gotOK := slabFacets(p, xr, lo, hi, extra)
			require.Equal(t, wantOK, gotOK)
			require.Equal(t, len(wantF), len(gotF))
			for i := range wantF {
				require.Equal(t, wantF[i], gotF[i])
			}
			require.Equal(t, math.Float64bits(wantC.X), math.Float64bits(gotC.X))
			require.Equal(t, math.Float64bits(wantC.Y), math.Float64bits(gotC.Y))
			require.Equal(t, math.Float64bits(wantC.Z), math.Float64bits(gotC.Z))
		}
	}
}
