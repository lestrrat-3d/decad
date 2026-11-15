package retainedblend

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// planeSlant is the slant of a plane wall that meets the end plane at sine s: the
// distance along the wall to the plane's crossing is (b + e)/s. No production band
// has a plane wall at a slant; the flat harness stands one in.
type planeSlant struct{ s float64 }

func (p planeSlant) reach(b, e float64) (float64, bool) { return (b + e) / p.s, true }
func (planeSlant) tilt(r3.Vec) (float64, bool)          { return 0, true }
func (planeSlant) sameSide(_, _ r3.Vec, _, _ float64) bool {
	return true
}

// slantedFlatBand is tiltedBand with its end plane marked slanted, as a route marks
// the end of an arc.
func slantedFlatBand(s float64, verts ...r3.Vec) (*work, *wallBand, faceLoops) {
	w, b, f := tiltedBand(s, verts...)
	b.ends[0].slanted, b.ends[0].slant = true, planeSlant{s}
	return w, b, f
}

// TestCrossingBoundCoversSlant makes the cut points of an end plane that a flat
// wall meets at sine s, and the band's corner there, publish a bound that reaches
// the true boundary the held bounds allow: the wall may lie h = ±0.001 off the held
// facet. Drop the slant at the held edge's crossing and the rows with s ≤ 0.4
// fail; drop it at the chord corner and the corner rows fail.
func TestCrossingBoundCoversSlant(t *testing.T) {
	t.Parallel()

	const beta = 0.001
	for _, s := range []float64{1, 0.866, 0.66, 0.5, 0.4, 0.2} {
		t.Run(fmt.Sprintf("edge s=%v", s), func(t *testing.T) {
			t.Parallel()
			// Vertex 0 lies 0.01 outside the end plane, so no sign reading happens and the
			// cut points on the plane carry the crossing bound.
			w, b, f := slantedFlatBand(s, r3.NewVec(0.5, -0.01/s, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
			_, err := w.cutBand(f, b)
			require.NoError(t, err)
			made := 0
			for id := 3; id < len(w.verts); id++ {
				for _, h := range []float64{beta, -beta} {
					made++
					truth := keptDistance(w.verts[id], s, h)
					require.GreaterOrEqual(t, w.beta[id], truth, "s %v cut %d h %v", s, id, h)
				}
			}
			require.Positive(t, made)
		})
		t.Run(fmt.Sprintf("corner s=%v", s), func(t *testing.T) {
			t.Parallel()
			c := math.Sqrt(1 - s*s)
			// One facet holds the band's corner, where the main plane x = 1 meets the
			// end plane, inside it. The true corner is (1, −c·h/s, h).
			w, b, f := slantedFlatBand(s, r3.NewVec(0.6, -0.5, 0), r3.NewVec(1.6, -0.4, 0), r3.NewVec(1.2, 0.9, 0))
			cuts, err := w.cutBand(f, b)
			require.NoError(t, err, "s %v", s)
			id := cuts.corner[0]
			require.GreaterOrEqual(t, id, 0, "s %v: the corner is made", s)
			for _, h := range []float64{beta, -beta} {
				truth := w.verts[id].Sub(r3.NewVec(1, -c*h/s, h)).Len()
				require.GreaterOrEqual(t, w.beta[id], truth, "s %v h %v", s, h)
			}
		})
	}
}

// slantedCylBand is cylBand with its end plane marked slanted and given the section
// bound that wallSlant reads from the wall's tag, as the arc route does; square marks
// it as a square plane instead.
func slantedCylBand(t *testing.T, hr, side float64, square bool) (*work, *wallBand, faceLoops, float64) {
	t.Helper()
	w, b, f, h := cylBand(hr, side)
	if square {
		return w, b, f, h
	}
	cyl := surfacegeom.Cylinder{Origin: r3.Vec{}, Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(3)}
	slant := wallWork(tagged(cyl)).wallSlant(0, b.ends[0].g)
	require.NotNil(t, slant)
	b.ends[0].slanted, b.ends[0].slant = true, slant
	return w, b, f, h
}

// TestCrossingBoundCoversCylinder cuts the arc route's shape (a cylinder wall
// parallel to the end plane, band on either side of it) and reads every cut point on
// the end plane against the true walls the held bounds allow, radius 3 ± 0.0014: a
// cut point of the plane against the kept part of the wall, the band's corner against
// the true corner. The worst ratio of the true distance to the published bound stays
// at most 1 and reaches 0.98, so the bound is tight. Replace the section bound by the
// plain distance and the far side fails from sine 0.6; drop the corner's offset from
// the setback plane and the tight rows fail; drop the offset e from the hypotenuse
// and the rows fail.
func TestCrossingBoundCoversCylinder(t *testing.T) {
	t.Parallel()

	const radius = 3.0
	worst, rows := 0.0, 0
	for _, slack := range []float64{0, 0.0002} {
		for _, side := range []float64{-1, 1} {
			for _, hr := range []float64{0.7, 0.8, 0.866, 0.9, 0.95, 0.98} {
				w, b, f, h := slantedCylBand(t, hr, side, false)
				b.slack = slack
				cuts, err := w.cutBand(f, b)
				if err != nil && slack > 0 {
					// A larger slack puts a cut point of the setback plane within tolerance of
					// the end plane on some rows, which refuse as they did before.
					require.ErrorIs(t, err, decaderr.ErrUnsupported)
					continue
				}
				require.NoError(t, err, "side %v h/R %v", side, hr)
				require.GreaterOrEqual(t, cuts.corner[0], 0)
				rows++
				checked := 0
				for id := 12; id < len(w.verts); id++ {
					p := w.verts[id]
					if math.Abs(p.X-h) > 1e-9 {
						continue
					}
					onMain := math.Abs(p.Z-1) < 1e-9
					for _, rt := range []float64{radius + 0.0014, radius - 0.0014} {
						yt := math.Sqrt(rt*rt - h*h)
						var truth float64
						switch {
						case onMain:
							// The setback plane may lie its slack off its place along the axis.
							for _, zt := range []float64{1 - slack, 1 + slack} {
								truth = math.Max(truth, p.Sub(r3.NewVec(h, yt, zt)).Len())
							}
						default:
							rho := math.Hypot(p.X, p.Y)
							if side*(p.X*rt/rho-h) <= 0 {
								truth = math.Abs(rho - rt)
							} else {
								truth = math.Hypot(p.X-h, p.Y-yt)
							}
						}
						checked++
						if slack == 0 {
							worst = math.Max(worst, truth/w.beta[id])
						}
						require.GreaterOrEqual(t, w.beta[id], truth, "slack %v side %v h/R %v cut %d at %v", slack, side, hr, id, p)
					}
				}
				require.Positive(t, checked)
			}
		}
	}
	require.Greater(t, rows, 12, "the rows with a slack build")
	require.GreaterOrEqual(t, worst, 0.98, "the bound is tight")
	require.LessOrEqual(t, worst, 1.0)
}

// TestSlantSquarePlanesKeepBytes leaves the bound of a cut on a plane the wall meets
// square exactly as it was: the held bound plus the slack plus τ. The same cylinder
// band marked slanted publishes more, so the mark decides.
func TestSlantSquarePlanesKeepBytes(t *testing.T) {
	t.Parallel()

	published := func(square bool) []float64 {
		w, b, f, _ := slantedCylBand(t, 0.98, 1, square)
		_, err := w.cutBand(f, b)
		require.NoError(t, err)
		return append([]float64(nil), w.beta[12:]...)
	}
	sq, sl := published(true), published(false)
	require.Len(t, sl, len(sq))
	grew := false
	for i, v := range sq {
		require.GreaterOrEqual(t, sl[i], v, "a slanted bound never falls below the square one")
		grew = grew || sl[i] > v
	}
	require.True(t, grew, "the slant raises a bound here")
	w, b, f := flatBand(r3.NewVec(0.5, -0.01, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
	_, err := w.cutBand(f, b)
	require.NoError(t, err)
	for id := 3; id < len(w.verts); id++ {
		if p := w.verts[id]; math.Abs(p.Y) < 1e-12 {
			require.Equal(t, 0.001+0+0.001, w.beta[id], "the cut of the square end plane")
		}
	}
}

// TestCornerBandChargeCarriesSlant notes the band's corner on a slanted end plane for
// the strip's band charge with the part of its distance to its true place that
// cutOffset's two slacks do not add, so the charge covers it. Note the facet bound
// again and the delta falls short.
func TestCornerBandChargeCarriesSlant(t *testing.T) {
	t.Parallel()

	w, b, f, _ := slantedCylBand(t, 0.98, 1, false)
	b.noteCuts, b.slack = true, 1e-5
	cuts, err := w.cutBand(f, b)
	require.NoError(t, err)
	id := cuts.corner[0]
	require.GreaterOrEqual(t, id, 0)
	delta, ok := w.cutDelta[id]
	require.True(t, ok)
	require.Greater(t, delta, 0.002, "the facet bound alone is 0.002")
	require.GreaterOrEqual(t, w.cutOffset(id, delta, b.slack), w.beta[id]-1e-12, "the charge covers the published bound")
}

// TestSlantRefusals refuses a cut on a slanted end plane that the tags cannot bound:
// a plane with no section, a corner of the plane and the guard, and a corner of two
// end planes. A square end plane keeps the cut, and so does a corner it makes with a
// plane normal to the axis.
func TestSlantRefusals(t *testing.T) {
	t.Parallel()

	const text = "the wall meets an end face's plane at a slant the tags cannot bound"
	t.Run("no section", func(t *testing.T) {
		t.Parallel()
		w, b, f, _ := slantedCylBand(t, 0.8, 1, false)
		b.ends[0].slant = nil
		_, err := w.cutBand(f, b)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, text)
	})
	t.Run("the section fails", func(t *testing.T) {
		t.Parallel()
		w, b, f, _ := slantedCylBand(t, 0.8, 1, false)
		b.ends[0].slant = &cylSlant{radius: 3, hc: 3.5, axis: r3.NewVec(0, 0, 1)}
		_, err := w.cutBand(f, b)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, text)
	})
	t.Run("a corner with the guard", func(t *testing.T) {
		t.Parallel()
		w, b, _, _ := slantedCylBand(t, 0.8, 1, false)
		b.w = w
		// The main plane becomes a plane through the axis, as the guard is.
		b.main.g = plane{n: r3.NewVec(0, 1, 0), o: r3.Vec{}}
		_, _, err := b.slantBound(r3.NewVec(2.4, 1.8, 1), bandGen, bandEnd0, 0.002, 0.003)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, text)
	})
	t.Run("a corner of two end planes", func(t *testing.T) {
		t.Parallel()
		w, b, _, _ := slantedCylBand(t, 0.8, 1, false)
		b.w = w
		b.ends[1] = &bandEnd{g: plane{n: r3.NewVec(0, 1, 0), o: r3.Vec{}}, sign: -1, slanted: true, slant: b.ends[0].slant}
		_, _, err := b.slantBound(r3.NewVec(2.4, 1.8, 1), bandEnd1, bandEnd0, 0.002, 0.003)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, text)
	})
	t.Run("a corner with a plane normal to the axis", func(t *testing.T) {
		t.Parallel()
		w, b, _, _ := slantedCylBand(t, 0.8, 1, false)
		b.w = w
		bound, pos, err := b.slantBound(r3.NewVec(2.4, 1.8, 1), bandGen, bandEnd0, 0.002, 0.003)
		require.NoError(t, err)
		require.Greater(t, bound, 0.003)
		require.Equal(t, bound, pos)
	})
}

// edgeAlongG is the cylinder band whose held edge runs along the end plane x = h
// between two points on the cylinder, ends at y1 and y2, and is cut by the main
// plane z = 1.
func edgeAlongG(t *testing.T, y1, y2 float64) (*work, *wallBand, faceLoops) {
	t.Helper()
	w, b, _, h := slantedCylBand(t, 0.8, 1, false)
	w.verts = []r3.Vec{r3.NewVec(h, y1, 0.55), r3.NewVec(h, y2, 1.45), r3.NewVec(h+0.05, y1, 1.45)}
	w.beta = []float64{0.002, 0.002, 0.002}
	w.fp = facetedrecord.Payload{Tris: [][3]int{{0, 1, 2}}, FaceOf: []int{0}}
	b.ends[0].on = map[int]bool{0: true, 1: true}
	b.noteCuts, b.slack = true, 1e-6
	return w, b, faceLoops{facets: []int{0}}
}

// TestCornerOnHeldEdgeAlongSlantedPlane cuts a held edge that runs along a slanted end
// plane by the main plane. Both ends on one side of the axis lie on one generator,
// so the cut keeps its old bound; ends that may lie on different generators publish
// the corner's bound. Drop the same-side check and the first keeps no bytes; always
// take the old bound and the second falls short.
func TestCornerOnHeldEdgeAlongSlantedPlane(t *testing.T) {
	t.Parallel()

	corner := func(w *work) int {
		for id := 3; id < len(w.verts); id++ {
			if math.Abs(w.verts[id].Z-1) < 1e-9 && math.Abs(w.verts[id].X-2.4) < 1e-9 {
				return id
			}
		}
		return -1
	}
	t.Run("one side", func(t *testing.T) {
		t.Parallel()
		w, b, f := edgeAlongG(t, 1.5, 1.7)
		_, err := w.cutBand(f, b)
		require.NoError(t, err)
		id := corner(w)
		require.GreaterOrEqual(t, id, 0)
		require.Equal(t, 0.002+b.slack+w.tau, w.beta[id], "the old bound")
	})
	t.Run("an end within its bound of the foot", func(t *testing.T) {
		t.Parallel()
		w, b, f := edgeAlongG(t, 0.001, 1.7)
		_, err := w.cutBand(f, b)
		require.NoError(t, err)
		id := corner(w)
		require.GreaterOrEqual(t, id, 0)
		require.Greater(t, w.beta[id], 0.002+b.slack+w.tau, "the corner's bound")
	})
	t.Run("across the axis", func(t *testing.T) {
		t.Parallel()
		w, b, f := edgeAlongG(t, -1.5, 1.7)
		_, err := w.cutBand(f, b)
		require.NoError(t, err)
		id := corner(w)
		require.GreaterOrEqual(t, id, 0)
		require.Greater(t, w.beta[id], 0.002+b.slack+w.tau, "the corner's bound")
	})
}

// TestSectionBoundCoversTheCrossing checks cylSlant.reach against a dense search: the
// distance from any point of the annulus R ± b within e of the end plane to the
// generator on its side, for the axis on either side of the plane, its distance from
// the plane uncertain by r, and the point's side of the foot on either side. Swap the
// outward ends of the section's ranges, use h for y⁻, or drop e from the hypotenuse
// and a row fails.
func TestSectionBoundCoversTheCrossing(t *testing.T) {
	t.Parallel()

	const radius = 3.0
	worst := 0.0
	for _, hr := range []float64{0, 0.005, 0.1, 0.5, 0.866, 0.98} {
		hc := hr * radius
		for _, r := range []float64{0, 0.004} {
			sl := &cylSlant{radius: radius, hc: hc, r: r, axis: r3.NewVec(0, 0, 1)}
			for _, beta := range []float64{1e-4, 1e-3, 1e-2} {
				for _, e := range []float64{0, beta / 8, beta / 2, beta} {
					d, ok := sl.reach(beta, e)
					if !ok {
						continue
					}
					for _, h := range []float64{hc - r, hc, hc + r} {
						if h < 0 {
							continue
						}
						z := math.Sqrt(radius*radius - h*h)
						for ir := -8; ir <= 8; ir++ {
							rho := radius + beta*float64(ir)/8
							for iy := -8; iy <= 8; iy++ {
								y := h + e*float64(iy)/8
								if rho*rho <= y*y {
									continue
								}
								x := math.Sqrt(rho*rho - y*y)
								dist := math.Hypot(x-z, y-h)
								worst = math.Max(worst, dist/d)
								require.LessOrEqual(t, dist, d, "h/R %v r %v β %v e %v ρ %v y %v h %v", hr, r, beta, e, rho, y, h)
							}
						}
					}
				}
			}
		}
	}
	require.Greater(t, worst, 0.9, "the search reaches the bound")
}

// TestSlantBoundCorners checks the corner's bound against its parts: the plane's
// offset and the tilt of its normal against the crossing line's direction both add,
// a normal tilted past parallelTol refuses.
func TestSlantBoundCorners(t *testing.T) {
	t.Parallel()

	w, b, _, _ := slantedCylBand(t, 0.8, 1, false)
	b.slack = 1e-4
	b.w = w
	p := r3.NewVec(2.4, 0, 1)
	_, edge, err := b.slantBound(p, bandEnd0, -1, 0.002, 0)
	require.NoError(t, err)
	require.Zero(t, edge, "a cut point on the plane alone has no corner position")
	tilted := func(tilt float64) (float64, float64, error) {
		b.main.g = plane{n: r3.NewVec(0, math.Sin(tilt), math.Cos(tilt)), o: r3.NewVec(0, 0, 1)}
		return b.slantBound(p, bandGen, bandEnd0, 0.002, 0)
	}
	_, flat, err := tilted(0)
	require.NoError(t, err)
	d, ok := b.ends[0].slant.reach(proofbound.AbsSumUpper(0.002, b.slack), b.slack)
	require.True(t, ok)
	require.GreaterOrEqual(t, flat, d+b.slack, "the corner adds its offset's slack to the section bound")
	_, near, err := tilted(5e-10)
	require.NoError(t, err)
	require.GreaterOrEqual(t, near, flat+4e-10*d, "the tilt adds a share of the section bound")
	_, _, err = tilted(2e-9)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

// cylBand is the arc route's shape on a cylinder of radius 3 about z: the end plane
// G is x = h, parallel to the axis, with the band on x ≤ h (side −1) or x ≥ h; the
// setback plane z = 1 (band below) and the rim plane z = 0 (band above). The held wall
// is a strip of chord facets between z = 0.55 and 1.45 at angles straddling G's
// crossing at half steps, every vertex on the cylinder with bound 0.002 (the
// sagitta is 0.0006). The end is not yet marked slanted.
func cylBand(hr, side float64) (*work, *wallBand, faceLoops, float64) {
	const radius, step = 3.0, 0.04
	h := hr * radius
	thG := math.Acos(hr)
	var verts []r3.Vec
	for _, z := range []float64{0.55, 1.45} {
		for k := range 6 {
			th := thG + (float64(k)-2.5)*step
			verts = append(verts, r3.NewVec(radius*math.Cos(th), radius*math.Sin(th), z))
		}
	}
	var tris [][3]int
	for k := range 5 {
		a, bb, c, d := k, k+1, 6+k, 6+k+1
		tris = append(tris, [3]int{a, bb, d}, [3]int{a, d, c})
	}
	beta := make([]float64, len(verts))
	for i := range beta {
		beta[i] = 0.002
	}
	facets := make([]int, len(tris))
	for i := range facets {
		facets[i] = i
	}
	w := &work{
		fp:    facetedrecord.Payload{Tris: tris, FaceOf: make([]int, len(tris))},
		verts: verts, beta: beta, tau: 0.001, cuts: map[[2]int]int{},
	}
	z := r3.NewVec(0, 0, 1)
	b := &wallBand{
		main:   &bandEnd{g: plane{n: z, o: r3.NewVec(0, 0, 1)}, sign: -1},
		others: []*bandEnd{{g: plane{n: z, o: r3.Vec{}}, sign: 1}},
	}
	b.ends[0] = &bandEnd{g: plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(h, 0, 0)}, sign: side}
	return w, b, faceLoops{facets: facets}, h
}
