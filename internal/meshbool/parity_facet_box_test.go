package meshbool_test

// CI: isolated-tests

import (
	"fmt"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// maxFacetBoxReports bounds how many failing cases the test prints.
const maxFacetBoxReports = 4

// facetBoxValues are the coordinates the hand-picked facets and queries draw
// from: both zeros, subnormals, ordinary and very large magnitudes.
var facetBoxValues = []float64{
	0, math.Copysign(0, -1), 1, -1, 2.5, -7.25,
	5e-324, -5e-324, 1e-310, 1e150, -1e150, 1e300, -1e300,
}

func setAxis(v r3.Vec, axis int, x float64) r3.Vec {
	switch axis {
	case 0:
		v.X = x
	case 1:
		v.Y = x
	default:
		v.Z = x
	}
	return v
}

// facetBoxFacets returns triangles, each a case for one swept axis (u, v):
// segments on a line of constant v and of constant u (corners in several
// orders, some corners coincident), single-point projections, and general
// triangles.
func facetBoxFacets(axis, u, v int, rng *rand.Rand) [][3]r3.Vec {
	pick := func() float64 {
		if rng.IntN(3) == 0 {
			return rng.NormFloat64() * math.Pow(10, float64(rng.IntN(7)-3))
		}
		return facetBoxValues[rng.IntN(len(facetBoxValues))]
	}
	corner := func(cu, cv float64) r3.Vec {
		return setAxis(setAxis(setAxis(r3.Vec{}, u, cu), v, cv), axis, pick())
	}
	var out [][3]r3.Vec
	for range 160 {
		c, a, b, d := pick(), pick(), pick(), pick()
		out = append(out,
			[3]r3.Vec{corner(a, c), corner(b, c), corner(d, c)},
			[3]r3.Vec{corner(c, a), corner(c, b), corner(c, d)},
			[3]r3.Vec{corner(a, c), corner(a, c), corner(b, c)},
			[3]r3.Vec{corner(c, a), corner(c, b), corner(c, b)},
			[3]r3.Vec{corner(a, c), corner(a, c), corner(a, c)},
			[3]r3.Vec{corner(a, b), corner(c, d), corner(pick(), pick())},
		)
	}
	return out
}

// facetBoxCoords lists the query coordinates tried against a facet: the
// facet's own coordinates, their float neighbours, zeros and random values.
func facetBoxCoords(f [3]r3.Vec, axis int, rng *rand.Rand) []float64 {
	var out []float64
	for _, c := range f {
		x := meshbool.CoordOf(c, axis)
		out = append(out, x, math.Nextafter(x, math.Inf(1)), math.Nextafter(x, math.Inf(-1)))
	}
	out = append(out, 0, math.Copysign(0, -1), rng.NormFloat64(), facetBoxValues[rng.IntN(len(facetBoxValues))])
	return out
}

// TestParityFacetBoxRejectionIsOutside checks every facet box against the real
// classifier. A rejected query must classify strictly outside, and a query on
// an axis-segment box's line must never be rejected. Single-point projections
// must not be flagged as axis segments.
func TestParityFacetBoxRejectionIsOutside(t *testing.T) {
	rng := rand.New(rand.NewPCG(11, 12))
	var facets, segments, points, queries, rejected, onLine int
	var failures []string
	fail := func(format string, args ...any) {
		if len(failures) < maxFacetBoxReports {
			failures = append(failures, fmt.Sprintf(format, args...))
		}
	}
	nfail := 0
	for ray, r := range meshbool.AxisRays {
		for fi, f := range facetBoxFacets(r.Axis, r.U, r.V, rng) {
			pm := meshbool.NewParityMesh([]r3.Vec{f[0], f[1], f[2]}, [][3]int{{0, 1, 2}})
			box := pm.BuildFacetBox(r.Axis, r.U, r.V, 0)
			facets++
			singlePoint := box.MinU == box.MaxU && box.MinV == box.MaxV
			if singlePoint {
				points++
				if box.AxisSegment {
					nfail++
					fail("ray %d facet %d: single-point projection flagged AxisSegment, box %+v", ray, fi, box)
				}
			}
			if box.AxisSegment {
				segments++
			}
			for _, qu := range facetBoxCoords(f, r.U, rng) {
				for _, qv := range facetBoxCoords(f, r.V, rng) {
					q := setAxis(setAxis(setAxis(r3.Vec{}, r.U, qu), r.V, qv), r.Axis, rng.NormFloat64())
					p := proof.XptOf(q)
					pa := meshbool.NewXP2(meshbool.RatCoordOf(p, r.U), meshbool.RatCoordOf(p, r.V))
					if !pa.FloatFinite {
						continue
					}
					queries++
					rejects := box.Rejects(pa.Fu, pa.Fv)
					if box.AxisSegment {
						line := box.MinV == box.MaxV && qv == box.MinV || box.MinU == box.MaxU && qu == box.MinU
						if line {
							onLine++
							if rejects {
								nfail++
								fail("ray %d facet %d: query (%g, %g) on the line was rejected, box %+v", ray, fi, qu, qv, box)
							}
						}
					}
					if !rejects {
						continue
					}
					rejected++
					if !meshbool.ParityFacetOutside(pm, p, ray, 0) {
						nfail++
						fail("ray %d facet %d: rejected query (%g, %g) is not outside, box %+v", ray, fi, qu, qv, box)
					}
				}
			}
		}
	}
	require.Empty(t, failures, "%d failures over %d facets, %d queries; first cases", nfail, facets, queries)
	require.Positive(t, segments, "no axis-segment facet was built")
	require.Positive(t, points, "no single-point facet was built")
	require.Positive(t, onLine, "no query lay on a segment's line")
	require.Positive(t, rejected, "no query was rejected")
}
