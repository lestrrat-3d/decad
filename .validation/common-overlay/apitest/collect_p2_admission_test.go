package apitest_test

import (
	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
	"math"
	"testing"
)

// Temporary admission-only fixture adapted from reviewed run 38769182347.
func BenchmarkCollectP2CrossedRodsAdmission(b *testing.B) {
	const qualify = true
	volumeInterval := func(body *decad.Body) (float64, float64) {
		m, err := body.Volume()
		if err != nil {
			b.Fatalf("crossed-rods volume read failed: %v", err)
		}
		v, bound := m.Value.Base(), m.Bound.Base()
		if math.IsNaN(v) || math.IsInf(v, 0) || v <= 0 ||
			math.IsNaN(bound) || math.IsInf(bound, 0) || bound < 0 {
			b.Fatalf("crossed-rods volume is invalid: value=%g bound=%g", v, bound)
		}
		lo := math.Nextafter(v-bound, math.Inf(-1))
		hi := math.Nextafter(v+bound, math.Inf(1))
		if math.IsNaN(lo) || math.IsInf(lo, 0) || lo <= 0 ||
			math.IsNaN(hi) || math.IsInf(hi, 0) {
			b.Fatalf("crossed-rods volume interval is invalid: low=%g high=%g", lo, hi)
		}
		return lo, hi
	}
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		a := benchRodBody(b, doc, 0, 0, 4)
		c := benchRodBody(b, doc, 0, 0, 3)
		rot, err := r3.Rotation(r3.Vec{X: 1, Y: 0, Z: 0}, units.Degrees(90))
		require.NoError(b, err)
		tr, err := r3.Translation(r3.Vec{X: 0, Y: -4, Z: 2})
		require.NoError(b, err)
		xf, err := rot.Then(tr)
		require.NoError(b, err)
		c, err = decad.Transform(b.Context(), c, xf)
		require.NoError(b, err)
		var aLo, aHi, cLo, cHi float64
		if qualify {
			aLo, aHi = volumeInterval(a)
			cLo, cHi = volumeInterval(c)
		}
		b.StartTimer()
		got, err := decad.Union(b.Context(), a, c)
		if err != nil {
			b.Fatalf("crossed-rods Union failed: %v", err)
		}
		if got == nil {
			b.Fatal("crossed-rods Union returned no body")
		}
		if !qualify {
			continue
		}
		b.StopTimer()
		if !got.IsSolid() {
			b.Fatal("crossed-rods Union returned a non-solid body")
		}
		if lumps := len(got.Lumps()); lumps != 1 {
			b.Fatalf("crossed-rods Union has %d lumps, want 1", lumps)
		}
		mesh, err := got.Tessellate(b.Context(), units.Millimeters(1000), decad.WithVerification(decad.VerifyBoundary))
		if err != nil {
			b.Fatalf("crossed-rods boundary verification failed: %v", err)
		}
		if !mesh.BoundaryVerified() {
			b.Fatal("crossed-rods boundary was not verified")
		}
		verts, tris := mesh.Vertices(), mesh.Triangles()
		if len(verts) == 0 || len(tris) == 0 {
			b.Fatalf("crossed-rods mesh is empty: vertices=%d triangles=%d", len(verts), len(tris))
		}
		directed := make(map[[2]int]int, len(tris)*3)
		for ti, tri := range tris {
			if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
				b.Fatalf("crossed-rods triangle %d repeats a vertex: %v", ti, tri)
			}
			for k := range 3 {
				a, z := tri[k], tri[(k+1)%3]
				if a < 0 || a >= len(verts) || z < 0 || z >= len(verts) {
					b.Fatalf("crossed-rods triangle %d has invalid edge indices: %d %d", ti, a, z)
				}
				directed[[2]int{a, z}]++
			}
		}
		for edge, n := range directed {
			if n != 1 || directed[[2]int{edge[1], edge[0]}] != 1 {
				b.Fatalf("crossed-rods edge %v has counts %d and %d", edge, n,
					directed[[2]int{edge[1], edge[0]}])
			}
		}
		uLo, uHi := volumeInterval(got)
		lowerSum := math.Nextafter(aLo+cLo, math.Inf(-1))
		if math.IsNaN(lowerSum) || math.IsInf(lowerSum, 0) ||
			!(uLo > max(aHi, cHi) && uHi < lowerSum) {
			b.Fatalf("crossed-rods union volume intervals do not prove overlap: "+
				"a=[%g,%g] c=[%g,%g] union=[%g,%g]", aLo, aHi, cLo, cHi, uLo, uHi)
		}
		b.Logf("CROSSED_RODS_QUALIFIED vertices=%d triangles=%d edges=%d "+
			"a_lo=%g a_hi=%g c_lo=%g c_hi=%g union_lo=%g union_hi=%g",
			len(verts), len(tris), len(directed), aLo, aHi, cLo, cHi, uLo, uHi)
		b.StartTimer()
	}
}
