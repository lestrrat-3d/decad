package crossbore_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestProveToolBand proves the saddle chamfer's band for a radius-10 shaft
// and a radius-3 bore under a placed pose. It checks the chord bound against
// sampled points of the held band triangles, each row's area interval against
// a fine midpoint-rule area of the exact band
// (TestStationSampleError reads a moved inner row).
func TestProveToolBand(t *testing.T) {
	t.Parallel()
	const R, r, d = 10.0, 3.0, 1.5
	pose := crossbore.Pose{Axis: [3]int{2, 1, 0}, Sign: [3]float64{-1, -1, 1}, Shift: r3.Vec{X: 40}}
	held := func(j, i int) r3.Vec {
		o, b := crossbore.BandEnds(R, r, d, j%crossbore.ToolStations)
		return pose.World(crossbore.BandRow(o, b, i))
	}
	proof, err := crossbore.ProveToolBand(t.Context(), R, r, d, pose, held)
	require.NoError(t, err)
	require.Positive(t, proof.Chord)
	require.Less(t, proof.SampleError, 1e-12, "the held rows are the exact rows rounded")
	require.Less(t, proof.Chord, d/100, "the chord bound is a small part of the setback")

	// exact is the extended band F(θ, t) = B⁺(θ) + t·(O⁺ − B⁺)(θ), local.
	exact := func(theta, tt float64) r3.Vec {
		o, b := crossbore.FeetAt(R, r, 0, d, theta)
		band := o.Sub(b).Scale(1 + 2*crossbore.BandExtension)
		return b.Sub(o.Sub(b).Scale(crossbore.BandExtension)).Add(band.Scale(tt))
	}
	const n, rows = crossbore.ToolStations, crossbore.ToolRows
	h, k := 2*math.Pi/n, 1.0/rows
	// Each held triangle pairs with the exact band at the same parameters:
	// (P_j, Q_j, Q_j+1) over u ≤ s and (P_j, Q_j+1, P_j+1) over s ≤ u, with P
	// row i, Q row i+1, u the station fraction and s the row fraction.
	worst := 0.0
	const grid = 6
	for j := 0; j < n; j += 7 {
		for i := range rows {
			pj, qj := held(j, i), held(j, i+1)
			pn, qn := held(j+1, i), held(j+1, i+1)
			for a := 0; a <= grid; a++ {
				for b := 0; b <= grid; b++ {
					u, s := float64(a)/grid, float64(b)/grid
					var x r3.Vec
					if u <= s {
						x = pj.Scale(1 - s).Add(qj.Scale(s - u)).Add(qn.Scale(u))
					} else {
						x = pj.Scale(1 - u).Add(qn.Scale(s)).Add(pn.Scale(u - s))
					}
					f := pose.World(exact(2*math.Pi*float64(j)/n+u*h, float64(i)*k+s*k))
					worst = max(worst, x.Sub(f).Len())
				}
			}
		}
	}
	require.Positive(t, worst)
	require.LessOrEqual(t, worst, proof.Chord, "the chord bound covers the sampled held triangles")

	// A midpoint-rule area of each row of the exact band, from a grid twice
	// as fine as the stations and four times as fine as the rows.
	const ft, fs = 2 * n, 4
	sum := 0.0
	for i := range rows {
		var pts [fs + 1][ft + 1]r3.Vec
		for b := range fs + 1 {
			for a := range ft + 1 {
				pts[b][a] = exact(2*math.Pi*float64(a)/ft, (float64(i)+float64(b)/fs)*k)
			}
		}
		area := 0.0
		for b := range fs {
			for a := range ft {
				p00, p10, p01, p11 := pts[b][a], pts[b][a+1], pts[b+1][a], pts[b+1][a+1]
				dt := p10.Sub(p00).Add(p11.Sub(p01)).Scale(0.5)
				ds := p01.Sub(p00).Add(p11.Sub(p10)).Scale(0.5)
				area += dt.Cross(ds).Len()
			}
		}
		require.Less(t, proof.RowLo[i], area, "row %d's interval holds its area", i)
		require.Greater(t, proof.RowHi[i], area, "row %d's interval holds its area", i)
		sum += area
	}
	require.Less(t, proof.AreaLo, sum)
	require.Greater(t, proof.AreaHi, sum)
	require.Less(t, proof.AreaHi-proof.AreaLo, sum/100, "the band's area interval is narrow")
}
