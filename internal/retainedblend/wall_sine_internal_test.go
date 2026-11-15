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
