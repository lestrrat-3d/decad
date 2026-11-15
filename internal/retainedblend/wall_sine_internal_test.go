package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// wallWork is a work whose face 0 is f.
func wallWork(f *topo.Face) *work {
	return &work{faceIx: map[*topo.Face]int{f: 0}, r: rim{scale: 10}}
}

func tagged(s surfacegeom.Surface) *topo.Face { return &topo.Face{Surface: s, RetainedTag: true} }

// TestWallSine reads the sine of the wall's slant to an end plane from the wall's
// retained tag, and has none where the tag gives no bound.
func TestWallSine(t *testing.T) {
	t.Parallel()

	g := plane{n: r3.NewVec(0, 1, 0), o: r3.NewVec(0, 1.5, 0)}

	t.Run("a plane wall meets the plane at the sine of its normal's angle", func(t *testing.T) {
		t.Parallel()
		fr, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0.6, 0.8))
		require.NoError(t, err)
		// The wall's normal is (0, -0.8, 0.6) about: sin with g's normal (0, 1, 0).
		sine, planar := wallWork(tagged(surfacegeom.Plane{Frame: fr})).wallSine(0, g)
		require.True(t, planar)
		require.NotNil(t, sine)
		n := fr.N()
		want := n.Cross(g.n).Len()
		require.InDelta(t, want, sine(1), 1e-12)
		require.LessOrEqual(t, sine(1), want)
	})
	t.Run("a cylinder wall parallel to the plane, the axis on either side", func(t *testing.T) {
		t.Parallel()
		// R = 3, the axis 1.5 from the plane: sin = √(R²−h²)/R = 0.866 at the plane, less within reach.
		for _, y := range []float64{0, 3} {
			cyl := surfacegeom.Cylinder{Origin: r3.NewVec(0, y, 3), Axis: r3.NewVec(1, 0, 0), Radius: units.Millimeters(3)}
			sine, planar := wallWork(tagged(cyl)).wallSine(0, g)
			require.False(t, planar)
			require.NotNil(t, sine)
			require.InDelta(t, math.Sqrt(1-0.25), sine(0), 1e-9, "axis at y = %v", y)
			require.Less(t, sine(0.3), sine(0))
			require.InDelta(t, math.Sqrt(1-(1.8/3)*(1.8/3)), sine(0.3), 1e-9)
		}
	})
	t.Run("a bore wall is the same surface", func(t *testing.T) {
		t.Parallel()
		cyl := surfacegeom.Cylinder{Origin: r3.NewVec(0, 1.5-0.4, 0), Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(1)}
		sine, planar := wallWork(tagged(cyl)).wallSine(0, g)
		require.False(t, planar)
		require.NotNil(t, sine)
		require.InDelta(t, math.Sqrt(1-0.16), sine(0), 1e-9)
		require.Zero(t, sine(0.7), "the reach leaves the cylinder's width")
	})
	t.Run("no bound", func(t *testing.T) {
		t.Parallel()
		tilted := surfacegeom.Cylinder{Origin: r3.Vec{}, Axis: r3.NewVec(0, 0.1, math.Sqrt(0.99)), Radius: units.Millimeters(3)}
		sine, _ := wallWork(tagged(tilted)).wallSine(0, g)
		require.Nil(t, sine, "an axis not parallel to the plane")
		cone := surfacegeom.Cone{Origin: r3.Vec{}, Axis: r3.NewVec(1, 0, 0), Radius: units.Millimeters(3), HalfAngle: units.Radians(0.1)}
		sine, _ = wallWork(tagged(cone)).wallSine(0, g)
		require.Nil(t, sine)
		cyl := surfacegeom.Cylinder{Origin: r3.NewVec(0, 0, 3), Axis: r3.NewVec(1, 0, 0), Radius: units.Millimeters(3)}
		for _, mod := range []func(f *topo.Face){
			func(f *topo.Face) { f.RetainedTag = false },
			func(f *topo.Face) { f.TagDeparture = 1e-9 },
			func(f *topo.Face) { f.NormalBound = -1 },
			func(f *topo.Face) { f.HasAxialDelta, f.AxialDelta = true, 1e-9 },
		} {
			f := tagged(cyl)
			mod(f)
			sine, _ = wallWork(f).wallSine(0, g)
			require.Nil(t, sine)
		}
		sine, _ = wallWork(tagged(cyl)).wallSine(1, g)
		require.Nil(t, sine, "no such face")
	})
}

// TestWallTilt reads from the wall's retained tag how far the wall's generators lean
// from the main plane's normal, and has none where the tag gives no bound.
func TestWallTilt(t *testing.T) {
	t.Parallel()

	up := r3.NewVec(0, 0, 1)

	t.Run("a plane wall normal to the base leans by the tag's normal bound alone", func(t *testing.T) {
		t.Parallel()
		fr, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0))
		require.NoError(t, err)
		f := tagged(surfacegeom.Plane{Frame: fr})
		f.NormalBound = 0.01
		tilt, ok := wallWork(f).wallTilt(0, up)
		require.True(t, ok)
		require.InDelta(t, 0.01, tilt, 1e-9)
		require.GreaterOrEqual(t, tilt, 0.01)
	})
	t.Run("a plane wall tilted from the normal leans by its angle", func(t *testing.T) {
		t.Parallel()
		// The wall's normal (0, cos a, sin a) leans the generators by a from up.
		const a = 0.05
		fr, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, math.Sin(a), -math.Cos(a)))
		require.NoError(t, err)
		tilt, ok := wallWork(tagged(surfacegeom.Plane{Frame: fr})).wallTilt(0, up)
		require.True(t, ok)
		require.GreaterOrEqual(t, tilt, a)
		require.InDelta(t, a, tilt, 2e-4)
	})
	t.Run("a cylinder leans by its axis's angle to the normal", func(t *testing.T) {
		t.Parallel()
		const a = 0.03
		cyl := surfacegeom.Cylinder{Origin: r3.Vec{}, Axis: r3.NewVec(math.Sin(a), 0, math.Cos(a)), Radius: units.Millimeters(3)}
		tilt, ok := wallWork(tagged(cyl)).wallTilt(0, up)
		require.True(t, ok)
		require.GreaterOrEqual(t, tilt, a)
		require.InDelta(t, a, tilt, 2e-4)
		cyl.Axis = up
		tilt, ok = wallWork(tagged(cyl)).wallTilt(0, up)
		require.True(t, ok)
		require.Less(t, tilt, 1e-12)
	})
	t.Run("no bound", func(t *testing.T) {
		t.Parallel()
		cyl := surfacegeom.Cylinder{Origin: r3.Vec{}, Axis: up, Radius: units.Millimeters(3)}
		for name, mod := range map[string]func(f *topo.Face){
			"no tag":           func(f *topo.Face) { f.RetainedTag = false },
			"a departure":      func(f *topo.Face) { f.TagDeparture = 1e-9 },
			"no normal bound":  func(f *topo.Face) { f.NormalBound = -1 },
			"an axial delta":   func(f *topo.Face) { f.HasAxialDelta, f.AxialDelta = true, 1e-9 },
			"a lean above max": func(f *topo.Face) { f.NormalBound = 0.2 },
			"another surface": func(f *topo.Face) {
				f.Surface = surfacegeom.Cone{Axis: up, Radius: units.Millimeters(3), HalfAngle: units.Radians(0.1)}
			},
			"a leaning cylinder": func(f *topo.Face) {
				f.Surface = surfacegeom.Cylinder{Axis: r3.NewVec(1, 0, 0), Radius: units.Millimeters(3)}
			},
		} {
			f := tagged(cyl)
			mod(f)
			_, ok := wallWork(f).wallTilt(0, up)
			require.False(t, ok, name)
		}
		_, ok := wallWork(tagged(cyl)).wallTilt(1, up)
		require.False(t, ok, "no such face")
	})
}

// arcDistance is the distance from p to the arc of the circle of radius r about
// the origin that keep accepts, whose ends are the two points of the circle on
// the line x = xG: p's radial projection when that lies on the arc, else the
// nearer end.
func arcDistance(p [2]float64, r, xG float64, keep func(x, y float64) bool) float64 {
	norm := math.Hypot(p[0], p[1])
	if qx, qy := r*p[0]/norm, r*p[1]/norm; keep(qx, qy) {
		return math.Abs(norm - r)
	}
	y := math.Sqrt(r*r - xG*xG)
	return math.Min(math.Hypot(p[0]-xG, p[1]-y), math.Hypot(p[0]-xG, p[1]+y))
}

// TestSlantBoundCoversCylinderExactly publishes, for a held point near an end
// plane that a cylinder wall meets at a slant, a bound that reaches the exact
// distance to the kept arc of the circle. The plane G is parallel to the axis at
// distance h, with the band on either side of it and the axis on either side, the
// held point lies a radial offset t, |t| ≤ β, from a foot on the circle in the
// band (the bad orientation: the foot is on the band's side, the held point read
// outside), inside and outside the circle (a convex wall and a bore). The
// published bound is the one signRead publishes: β + c/s_lo on a curved wall, with
// s_lo(c) from wallSine.
func TestSlantBoundCoversCylinderExactly(t *testing.T) {
	t.Parallel()

	const radius = 3.0
	checked, refused, badFoot := 0, 0, 0
	for _, hr := range []float64{0.005, 0.1, 0.3, 0.5, 0.7, 0.866, 0.95} {
		for _, side := range []float64{1, -1} { // the band's side of G
			for _, axisSide := range []float64{1, -1} { // the axis's side of G
				h := hr * radius
				// G: x = xG with n = (1, 0, 0); the axis is at the origin.
				xG := -axisSide * h
				g := plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(xG, 0, 0)}
				cyl := surfacegeom.Cylinder{Origin: r3.Vec{}, Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(radius)}
				sine, planar := wallWork(tagged(cyl)).wallSine(0, g)
				require.NotNil(t, sine)
				require.False(t, planar)
				inBand := func(x float64) float64 { return side * (x - xG) }
				for _, beta := range []float64{1e-4, 1e-3, 1e-2} {
					for _, eps := range []float64{0, 0.25 * beta, beta, 2 * beta} {
						c := proofbound.AbsSumUpper(beta, eps)
						bound, ok := slantBound(c, sine)
						if !ok {
							refused++
							continue
						}
						published := proofbound.AbsSumUpper(beta, bound)
						// The held point lies eps outside the band: inBand = -eps. Its foot u
						// on the circle lies within beta radially, with the foot in the band.
						for _, tr := range []float64{-beta, -beta / 2, 0, beta / 2, beta} {
							// The held point is the foot's radial offset by tr, on the line
							// inBand = -eps: (R + tr)·cos a = xG - side·eps.
							cosA := (xG - side*eps) / (radius + tr)
							if math.Abs(cosA) >= 1 {
								continue
							}
							for _, a := range []float64{math.Acos(cosA), -math.Acos(cosA)} {
								vx, vy := (radius+tr)*math.Cos(a), (radius+tr)*math.Sin(a)
								keep := func(x, y float64) bool { return inBand(x) <= 0 }
								d := arcDistance([2]float64{vx, vy}, radius, xG, keep)
								footIn := inBand(radius*math.Cos(a)) > 0
								checked++
								if footIn {
									badFoot++
								}
								require.GreaterOrEqual(t, published, d,
									"h/R %v side %v axis %v β %v ε %v t %v angle %v foot in band %v",
									hr, side, axisSide, beta, eps, tr, a, footIn)
							}
						}
					}
				}
			}
		}
	}
	require.Positive(t, checked)
	require.Positive(t, badFoot, "some held point has its foot on the band's side")
	t.Logf("checked %d points (%d with the foot on the band's side), refused %d (c, sine) pairs", checked, badFoot, refused)
}

// TestSetbackBoundCoversKeptCylinder reads the sign reading's published bound, the
// smaller of β + c/s_lo and the setback bound, against the exact distance to the kept
// wall of a cylinder in three dimensions. The wall's axis is the setback plane's normal
// (z), the band lies between the base z = 0 and the setback plane z = 1 and on one side
// of G, a plane parallel to the axis, and the kept wall is everything above the setback
// plane and past G. The held point lies a radial offset t, |t| ≤ β, from a foot on the
// wall inside the band, at height 1 − d, read within ε of G, with the axis on either
// side. The distance is exact: radial and axial to the setback line, or around the
// circle to G's end of the kept arc, whichever is nearer. Drop the setback bound's
// offset d and the rows near the setback plane fail; the slant term alone fails
// nowhere here, and the setback bound alone fails where d is large.
func TestSetbackBoundCoversKeptCylinder(t *testing.T) {
	t.Parallel()

	const radius, setback = 3.0, 1.0
	z := r3.NewVec(0, 0, 1)
	checked, capUsed, worst := 0, 0, 0.0
	for _, hr := range []float64{0.005, 0.1, 0.3, 0.5, 0.7, 0.866, 0.95} {
		for _, side := range []float64{1, -1} {
			for _, axisSide := range []float64{1, -1} {
				h := hr * radius
				xG := -axisSide * h
				g := plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(xG, 0, 0)}
				cyl := surfacegeom.Cylinder{Origin: r3.Vec{}, Axis: z, Radius: units.Millimeters(radius)}
				w := wallWork(tagged(cyl))
				sine, planar := w.wallSine(0, g)
				require.NotNil(t, sine)
				require.False(t, planar)
				lean, ok := w.wallTilt(0, z)
				require.True(t, ok)
				b := &wallBand{main: &bandEnd{g: plane{n: z, o: r3.NewVec(0, 0, setback)}, sign: -1}, lean: lean, leanOK: true}
				inBand := func(x float64) float64 { return side * (x - xG) }
				for _, beta := range []float64{1e-4, 1e-3, 1e-2} {
					for _, eps := range []float64{0, 0.25 * beta, beta, 2 * beta} {
						bound, ok := slantBound(proofbound.AbsSumUpper(beta, eps), sine)
						if !ok {
							continue
						}
						slant := proofbound.AbsSumUpper(beta, bound)
						for _, d := range []float64{0, 0.3 * beta, beta, 3 * beta, 0.05, 0.4} {
							for _, tr := range []float64{-beta, 0, beta} {
								cosA := (xG - side*eps) / (radius + tr)
								if math.Abs(cosA) >= 1 {
									continue
								}
								for _, a := range []float64{math.Acos(cosA), -math.Acos(cosA)} {
									vx, vy := (radius+tr)*math.Cos(a), (radius+tr)*math.Sin(a)
									pt := r3.NewVec(vx, vy, setback-d)
									near, ok := b.setbackBound(pt, beta)
									require.True(t, ok)
									published := math.Min(slant, near)
									keep := func(x, y float64) bool { return inBand(x) <= 0 }
									toG := arcDistance([2]float64{vx, vy}, radius, xG, keep)
									toSetback := math.Hypot(math.Hypot(vx, vy)-radius, d)
									truth := math.Min(toG, toSetback)
									checked++
									if near < slant {
										capUsed++
									}
									worst = math.Max(worst, truth/published)
									require.LessOrEqual(t, truth/published, 1+1e-9, // the oracle's own rounding, relative to β
										"h/R %v side %v axis %v β %v ε %v d %v t %v angle %v", hr, side, axisSide, beta, eps, d, tr, a)
								}
							}
						}
					}
				}
			}
		}
	}
	require.Positive(t, capUsed, "the setback bound is the smaller one somewhere")
	require.GreaterOrEqual(t, worst, 0.9, "the published bound is tight")
	t.Logf("checked %d points, the setback bound published at %d, worst ratio %.6f", checked, capUsed, worst)
}

// TestRoutesSetTheWallLean makes the arc route's and the chain route's band carry the
// lean of their wall from its tag, so a sign reading there can publish the setback
// bound. Leave either route's assignment out and its band has none.
func TestRoutesSetTheWallLean(t *testing.T) {
	t.Parallel()

	z := r3.NewVec(0, 0, 1)
	cyl := surfacegeom.Cylinder{Origin: r3.Vec{}, Axis: z, Radius: units.Millimeters(3)}
	t.Run("the arc route", func(t *testing.T) {
		t.Parallel()
		w := wallWork(tagged(cyl))
		a := arcRim{
			center: r3.Vec{}, n: z, u: r3.NewVec(1, 0, 0), w: r3.NewVec(0, 1, 0), radius: 3, side: 1, scale: 10,
			sign: 1, theta0: 0, span: 1,
			ends: [2]lineEnd{{g: plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(1, 0, 0)}, v: 0},
				{g: plane{n: r3.NewVec(0, 1, 0), o: r3.NewVec(0, 1, 0)}, v: 1}},
		}
		band := a.newWallBand(w, faceLoops{index: 0}, 1, edgeRun{}, [2]sideKind{extends, extends})
		require.True(t, band.leanOK)
		require.Less(t, band.lean, 1e-12)
	})
	t.Run("the chain route", func(t *testing.T) {
		t.Parallel()
		w := wallWork(tagged(cyl))
		en := chainEntry{wallIx: 0, p0: r3.NewVec(0, 0, 0), p1: r3.NewVec(5, 0, 0)}
		ends := [2]*chainEndCut{{g: plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(1, 0, 0)}, extendsW: true}, nil}
		band, err := w.newChainBand(en, nil, z, r3.Vec{}, 1, map[int]bool{}, ends, 1e-12)
		require.NoError(t, err)
		require.True(t, band.leanOK)
		require.Less(t, band.lean, 1e-12)
	})
}
