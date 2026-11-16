package brepmodify

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestLayeredCutterBevelWithinItsBound measures the layered cutter's bevel facets against the exact bevel of a D keyhole
// (the rim's moved circle and flat, at the setback of each point's height) and compares the facets' departure with the
// cutter's declared bound. The facets' horizontal departure is the quantity layerDeparture's derivation bounds, and the
// distance in 3D is the quantity the mesh solid's bound means.
func TestLayeredCutterBevelWithinItsBound(t *testing.T) {
	t.Parallel()
	for _, c := range []struct{ r, flat float64 }{{4, 3}, {4.05, 3.5}, {2.55, 1.5}, {4, 1.5}, {2.55, 2}} {
		for _, d := range []float64{0.3, 0.5} {
			for _, over := range []float64{0.5 * d, 0.35 * d, 0.7 * d} {
				run := chamferRun{loop: keyholeLoop(c.r, c.flat), first: 2, count: 3}
				m, err := run.cutterMesh(t.Context(), d, over)
				require.NoError(t, err)
				isInner := func(v int) bool {
					p := m.verts[v]
					tt := d + p.Z
					f := max(math.Hypot(p.X, p.Y)-(c.r+tt), p.Y-(c.flat+tt))
					return math.Abs(f) < 0.01
				}
				worst, worst3 := 0.0, 0.0
				for _, tr := range m.tris {
					if !isInner(tr[0]) || !isInner(tr[1]) || !isInner(tr[2]) {
						continue
					}
					p0, p1, p2 := m.verts[tr[0]], m.verts[tr[1]], m.verts[tr[2]]
					if p0.Z == p1.Z && p1.Z == p2.Z {
						continue // a cap triangle
					}
					const N = 4
					for i := 0; i <= N; i++ {
						for j := 0; i+j <= N; j++ {
							a, b := float64(i)/N, float64(j)/N
							q := p0.Scale(1 - a - b).Add(p1.Scale(a)).Add(p2.Scale(b))
							tt := d + q.Z
							dist2 := func(tt float64) float64 {
								R, fl := c.r+tt, c.flat+tt
								xc := math.Sqrt(R*R - fl*fl)
								cd := math.Min(math.Hypot(q.X-xc, q.Y-fl), math.Hypot(q.X+xc, q.Y-fl))
								best := cd
								hy := math.Hypot(q.X, q.Y)
								if q.Y*R/hy <= fl {
									best = math.Min(best, math.Abs(hy-R))
								}
								if math.Abs(q.X) <= xc {
									best = math.Min(best, math.Abs(q.Y-fl))
								}
								return best
							}
							h2 := dist2(tt)
							d3 := h2
							for k := -30; k <= 30; k++ {
								dh := float64(k) * 2e-5
								d3 = math.Min(d3, math.Hypot(dh, dist2(tt+dh)))
							}
							worst = max(worst, h2)
							worst3 = max(worst3, d3)
						}
					}
				}
				require.LessOrEqual(t, worst3, m.bound, "3D departure, r %v flat %v d %v over %v", c.r, c.flat, d, over)
				require.LessOrEqual(t, worst, m.bound, "horizontal departure, r %v flat %v d %v over %v", c.r, c.flat, d, over)
			}
		}
	}
}
