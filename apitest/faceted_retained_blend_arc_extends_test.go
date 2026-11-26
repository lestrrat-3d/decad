package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of agent-docs/modify-design.md Part E R4 and Table E for
// the arc whose wall extends past its end planes: a rib stands on a round, so
// the rib's end face x = 0 meets the round's cylinder (axis along x) in an arc
// whose ends lie on the rib's two side faces y = ±earHalf. The round continues
// past those faces, and behind the end face where the rib is not, so the wall
// lies on both sides of the end face's plane. Both side faces are convex at the
// arc's ends and take the blend's end cap.

const (
	earHalf = 1.0 // the rib's half width
	earLen  = 6.0 // the rib's length along x
)

// earBody is a round bar (r = 3 along x, from x = -15 to 15) with a 6 x 2 box
// notched into its underside by a Union: the box reaches from z = -5 to -1.
func earBody(t *testing.T) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	bar := xCylinderAt(t, sc.doc, 0, 0, 3, -15, 15)
	box := placed(t, rectBox(t, sc.w, sc.doc, 0, -earHalf, earLen, earHalf, 4), -5)
	body, err := decad.Union(bar, box).Do(t.Context())
	require.NoError(t, err)
	return body
}

// earTube is a tube (bore r = 3, outer r = 6 along x, from x = -15 to 15) with a
// 6 x 2 rib on the bore's underside, from z = -4 to -1: the plane of the rib's
// end face lies inside the arc's circle.
func earTube(t *testing.T) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.YZ())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 6)
	s.CreateCircle(c, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var ring *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 1 {
			ring = p
		}
	}
	require.NotNil(t, ring)
	pipe, err := decad.Extrude(t.Context(), sc.doc, s, ring, decad.Distance{D: units.Millimeters(30), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{X: -15 + 0.001})
	require.NoError(t, err)
	tube, err := decad.Transform(t.Context(), pipe, move)
	require.NoError(t, err)
	rib := placed(t, rectBox(t, sc.w, sc.doc, 0, -earHalf, earLen, earHalf, 3), -4)
	body, err := decad.Union(tube, rib).Do(t.Context())
	require.NoError(t, err)
	return body
}

// earArc selects the arc of b whose centre lies at x = 0: the root of the end
// face x = 0.
func earArc(t *testing.T, b *decad.Body) decad.EdgeSelector {
	t.Helper()
	for _, e := range b.Edges() {
		if arc, ok := e.Curve().(decad.Arc3); ok && math.Abs(arc.Center.X) < 1e-6 {
			return decad.Edges(decad.Circular(), decad.EndpointAt(e.Start().Position().Value)).Exactly(1)
		}
	}
	require.Fail(t, "no arc at x = 0")
	return nil
}

// earPlanes are the two end planes of an arc, as the axis of their normal and
// their offset along it, and sweep the angle about the round's axis that the
// region between them subtends at the distance rho from it.
type earPlanes struct {
	planes [2][2]float64
	sweep  func(rho float64) float64
}

var (
	// earRib's end planes are the rib's side faces y = ±1.
	earRib = earPlanes{[2][2]float64{{1, earHalf}, {1, -earHalf}}, func(rho float64) float64 {
		return 2 * math.Asin(earHalf/rho)
	}}
	// earCornerEnds are the plate's faces x = 0 and y = 0 about the rod at (-1, -1).
	earCornerEnds = earPlanes{[2][2]float64{{0, 0}, {1, 0}}, func(rho float64) float64 {
		return math.Pi/2 - 2*math.Asin(1/rho)
	}}
)

// earRegion integrates f over the blend's corner region in the meridian
// coordinates (rho, h): rho the distance from the bar's axis and h the distance
// from the end face along x, away from the rib. The region lies between the
// round at rho = radius and the end face h = 0, on the side kappa of the round,
// out to size, and is the part of the square the blend fills: outside the
// fillet's circle, or below the chamfer's chord.
func earRegion(radius, kappa, size float64, fillet bool, f func(rho float64) float64) float64 {
	const n = 3000
	sum := 0.0
	for i := range n {
		d := size * (float64(i) + 0.5) / n // distance from the round
		for j := range n {
			h := size * (float64(j) + 0.5) / n
			in := d+h <= size
			if fillet {
				in = (d-size)*(d-size)+(h-size)*(h-size) >= size*size
			}
			if in {
				rho := radius + kappa*d
				sum += f(rho) * (size / n) * (size / n)
			}
		}
	}
	return sum
}

// earCornerRod is a plate (40 x 20 x 20, its corner at the origin) with a rod
// r = 5 along z from z = 10 to 40 standing at (-1, -1), outside its corner: the
// plate's top z = 20 meets the rod in an arc from the plate's face x = 0 to its
// face y = 0, and the rod runs on past both faces and below the top's plane.
func earCornerRod(t *testing.T) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	body, err := decad.Union(blendPlate(t, sc), zCylinderAt(t, sc, -1, -1, 5, 10, 40)).Do(t.Context())
	require.NoError(t, err)
	return body
}

// earTopArc selects the arc of b at the plate's top, z = 20.
func earTopArc(t *testing.T, b *decad.Body) decad.EdgeSelector { return arcSeedAt(t, b, 20) }

// earFaceArea is the area of the one face of b in the plane whose normal is the
// axis-th coordinate axis and whose offset is coord.
func earFaceArea(t *testing.T, b *decad.Body, axis int, coord float64) decad.Measurement {
	t.Helper()
	var found *decad.Face
	for _, f := range b.Faces() {
		pl, ok := f.Surface().(decad.Plane)
		if !ok {
			continue
		}
		n, o := pl.Frame.N(), pl.Frame.Origin()
		comp := [3]float64{n.X, n.Y, n.Z}
		off := [3]float64{o.X, o.Y, o.Z}
		if math.Abs(math.Abs(comp[axis])-1) > 1e-12 || math.Abs(off[axis]-coord) > 1e-9 {
			continue
		}
		require.Nil(t, found, "two faces in the plane %d = %g", axis, coord)
		found = f
	}
	require.NotNil(t, found)
	area, err := found.Area()
	require.NoError(t, err)
	return area
}

func TestFacetedRetainedBlendArcExtends(t *testing.T) {
	t.Parallel()

	// radius is the round's radius and kappa +1 when the rib's end face lies
	// outside the arc's circle, -1 inside it. A fillet of the tube's bore at a
	// small size closes its end cap in a cusp the held facets of the bore reach
	// past, so the rows' last gap is widened (cuspParam).
	for _, c := range []struct {
		name   string
		build  func(*testing.T) *decad.Body
		arc    func(*testing.T, *decad.Body) decad.EdgeSelector
		ends   earPlanes
		radius float64
		kappa  float64
		size   float64
		fillet bool
	}{
		{"a rib notched into a round bar, fillet", earBody, earArc, earRib, 3, 1, 1, true},
		{"a rib notched into a round bar, chamfer", earBody, earArc, earRib, 3, 1, 1, false},
		{"a rib in a tube's bore, fillet", earTube, earArc, earRib, 3, -1, 1, true},
		{"a rib in a tube's bore, chamfer", earTube, earArc, earRib, 3, -1, 1, false},
		{"a rib in a tube's bore, small fillet", earTube, earArc, earRib, 3, -1, 0.3, true},
		{"a rib notched into a round bar, small fillet", earBody, earArc, earRib, 3, 1, 0.1, true},
		{"a rod beside a plate's corner, fillet", earCornerRod, earTopArc, earCornerEnds, 5, 1, 1, true},
		{"a rod beside a plate's corner, chamfer", earCornerRod, earTopArc, earCornerEnds, 5, 1, 1, false},
		{"a rod beside a plate's corner, small fillet", earCornerRod, earTopArc, earCornerEnds, 5, 1, 0.2, true},
	} {
		{
			fillet, size := c.fillet, c.size
			t.Run(c.name, func(t *testing.T) {
				t.Parallel()
				body := c.build(t)
				seed := c.arc(t, body)
				volBefore, err := body.Volume()
				require.NoError(t, err)
				var areaBefore [2]decad.Measurement
				for i, pl := range c.ends.planes {
					areaBefore[i] = earFaceArea(t, body, int(pl[0]), pl[1])
				}
				var out *decad.Body
				if fillet {
					out, err = decad.Fillet(t.Context(), body, seed, units.Millimeters(size))
				} else {
					out, err = decad.Chamfer(t.Context(), body, seed, units.Millimeters(size))
				}
				require.NoError(t, err)

				// The blend adds the corner region swept about the axis between the
				// end planes: ∫∫ rho·Δφ(rho).
				added := earRegion(c.radius, c.kappa, size, fillet, func(rho float64) float64 {
					return rho * c.ends.sweep(rho)
				})
				volAfter, err := out.Volume()
				require.NoError(t, err)
				requireVolumeChange(t, volBefore, volAfter, -added)

				// Each side face takes the cap, the corner region cut by its plane.
				capArea := earRegion(c.radius, c.kappa, size, fillet, func(rho float64) float64 {
					return rho / math.Sqrt(rho*rho-earHalf*earHalf)
				})
				for i, pl := range c.ends.planes {
					requireAreaChange(t, areaBefore[i], earFaceArea(t, out, int(pl[0]), pl[1]), capArea)
				}

				if fillet {
					tori := 0
					for _, f := range out.Faces() {
						if tor, ok := f.Surface().(decad.Torus); ok {
							tori++
							require.InDelta(t, c.radius+c.kappa*size, tor.Major.Base(), 1e-9)
							require.InDelta(t, size, tor.Minor.Base(), 1e-12)
						}
					}
					require.Equal(t, 1, tori)
				}

				// The blend's bound grows by the strip's own terms past the held mesh's:
				// the size over 1024 and the azimuth and sagitta of its rows, as for a
				// hole mouth's arc (1.8 times that at size 1). A small blend's cut
				// vertices state the held facets' bound, and a widened last gap states
				// its chord's, so only the unit blend is held to it.
				held, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
				require.NoError(t, err)
				mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
				require.NoError(t, err)
				if size == 1 {
					require.LessOrEqual(t, revolveMeshBound(t, mesh), revolveMeshBound(t, held)+2*math.Ldexp(size, -10))
				}
				require.True(t, mesh.BoundaryVerified())
				require.True(t, mesh.VolumeVerified())
				rep, err := out.Document().Verify(t.Context())
				require.NoError(t, err)
				br, err := rep.ForBody(out)
				require.NoError(t, err)
				require.Equal(t, decad.Sound, br.Status)
			})
		}
	}

	t.Run("a hole through the round inside the band is refused", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		bar := xCylinderAt(t, sc.doc, 0, 0, 3, -15, 15)
		box := placed(t, rectBox(t, sc.w, sc.doc, 0, -earHalf, earLen, earHalf, 4), -5)
		body, err := decad.Union(bar, box).Do(t.Context())
		require.NoError(t, err)
		body, err = decad.Cut(body, zCylinderAt(t, sc, -0.5, 0, 0.25, -10, 10)).Do(t.Context())
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), body, earArc(t, body), units.Millimeters(1))
		require.ErrorContains(t, err, "another part of the wall lies within the blend's reach of the edge")
	})

	t.Run("a blend too large for the rib's end face is refused", func(t *testing.T) {
		t.Parallel()
		body := earBody(t)
		_, err := decad.Fillet(t.Context(), body, earArc(t, body), units.Millimeters(2.5))
		require.ErrorContains(t, err, "the blend consumes the plane beside the arc")
	})

	t.Run("a solid floating across the cap's plane is refused", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		bar := xCylinderAt(t, sc.doc, 0, 0, 3, -15, 15)
		box := placed(t, rectBox(t, sc.w, sc.doc, 0, -earHalf, earLen, earHalf, 4), -5)
		// A 0.1 mm cube clear of the round, across the plane y = 1 where the end
		// face takes the cap, inside the fillet's corner region and clear of its
		// strip.
		cube := placed(t, rectBox(t, sc.w, sc.doc, -0.15, 0.95, -0.05, 1.05, 0.1), -3.05)
		feet, err := decad.Union(box, cube).Do(t.Context())
		require.NoError(t, err)
		body, err := decad.Union(bar, feet).Do(t.Context())
		require.NoError(t, err)
		var seed decad.EdgeSelector
		for _, e := range body.Edges() {
			if arc, ok := e.Curve().(decad.Arc3); ok && math.Abs(arc.Center.X) < 1e-6 && e.Start().Position().Value.X == 0 {
				seed = decad.Edges(decad.Circular(), decad.EndpointAt(e.Start().Position().Value)).Exactly(1)
			}
		}
		require.NotNil(t, seed)
		_, err = decad.Fillet(t.Context(), body, seed, units.Millimeters(1))
		require.ErrorContains(t, err, "the cap an end face takes meets another facet of the body")
	})
}
