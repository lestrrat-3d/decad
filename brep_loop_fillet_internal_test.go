package decad

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// rowSL1 is the row name every route L refusal of a selection that is no set
// of complete loops carries.
const rowSL1 = "modify-general SL1"

// piPoly is a + b·π + c·π².
type piPoly [3]*big.Rat

func pp(a, b, c *big.Rat) piPoly { return piPoly{a, b, c} }

func q(n, d int64) *big.Rat { return big.NewRat(n, d) }

// at evaluates the polynomial at the rational p.
func (y piPoly) at(p *big.Rat) *big.Rat {
	out := new(big.Rat).Add(y[0], new(big.Rat).Mul(y[1], p))
	return out.Add(out, new(big.Rat).Mul(y[2], new(big.Rat).Mul(p, p)))
}

// enclosed is the polynomial's value at the two ends of the proven π
// bracket, low first. Every closed form here is monotone across a bracket that
// narrow, so the two ends enclose it.
func (y piPoly) enclosed() (*big.Rat, *big.Rat) {
	lo, hi := y.at(proofbound.PiLower), y.at(proofbound.PiUpper)
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	return lo, hi
}

// requireCentroidPi asserts the published centroid ball holds the exact
// centroid M/V, M and V polynomials in π, at both ends of the π bracket,
// shifted by offset.
func requireCentroidPi(t *testing.T, reading VecMeasurement, m [3]piPoly, v piPoly, offset [3]*big.Rat) {
	t.Helper()
	for _, p := range []*big.Rat{proofbound.PiLower, proofbound.PiUpper} {
		var exact [3]*big.Rat
		for i := range exact {
			exact[i] = new(big.Rat).Add(new(big.Rat).Quo(m[i].at(p), v.at(p)), offset[i])
		}
		requireCentroidCovers(t, reading, exact)
	}
}

func noOffset() [3]*big.Rat { return [3]*big.Rat{q(0, 1), q(0, 1), q(0, 1)} }

// filletLoopOfFace fillets loop li of body's planar face facing n through p
// by r millimetres and requires a closed brep result carrying one fillet band
// more than the receiver, the receiver retired.
func filletLoopOfFace(t *testing.T, body *Body, n, p r3.Vec, li int, r float64) (*Body, brepPayload) {
	t.Helper()
	f := planarBodyFace(t, body, n, p)
	return filletSelection(t, body, loopQuery(f.Loops()[li]), r, 1)
}

// filletSelection fillets sel by r and requires a closed brep result carrying
// bands more fillet bands than the receiver, the receiver retired.
func filletSelection(t *testing.T, body *Body, sel *EdgeQuery, r float64, bands int) (*Body, brepPayload) {
	t.Helper()
	before := 0
	if bp, ok := body.payload.(brepPayload); ok {
		before = len(bp.loopBands)
	}
	out, err := body.Fillet(t.Context(), sel, units.Millimeters(r))
	require.NoError(t, err)
	require.Error(t, body.doc.requireLive(body), "the receiver is retired")
	bp, ok := out.payload.(brepPayload)
	require.True(t, ok, "route L builds a brep, got %T", out.payload)
	require.Nil(t, bp.stack)
	require.Len(t, bp.loopBands, before+bands)
	for _, b := range bp.loopBands[before:] {
		require.Equal(t, brepBandFillet, b.kind)
	}
	requireClosedTopology(t, out)
	require.Len(t, bp.loopPatches, len(bp.loopBands))
	return out, bp
}

// requireVolumePi asserts the published volume's interval holds the closed
// form, and that it is Approximate: π enters every fillet's volume.
func requireVolumePi(t *testing.T, body *Body, v piPoly) {
	t.Helper()
	lo, hi := v.enclosed()
	requireCoversInterval(t, body.volume, lo, hi)
	require.Equal(t, Approximate, body.volume.Exactness)
}

// filletPatchesOf lists the body's faces carrying a filletLoop(f,l,p) role.
func filletPatchesOf(body *Body) []*Face {
	return blendFaces(body, "filletLoop")
}

// requireFilletPatches asserts the body's fillet patches are the given count
// of cylinders and of tori, and nothing else, each carrying a zero normal
// bound.
func requireFilletPatches(t *testing.T, body *Body, cylinders, tori int) []*Face {
	t.Helper()
	patches := filletPatchesOf(body)
	got := [2]int{}
	for _, f := range patches {
		switch f.Surface().(type) {
		case Cylinder:
			got[0]++
		case Torus:
			got[1]++
		default:
			t.Fatalf("a fillet patch is a %T", f.Surface())
		}
		require.Zero(t, f.normalBound, "a fillet patch's tag is its built surface")
	}
	require.Equal(t, [2]int{cylinders, tori}, got, "cylinder and torus patches")
	return patches
}

// filletEndCurves lists the edges that run between two fillet patches, by
// curve kind.
func filletEndCurves(body *Body) (ellipses []Ellipse3, meridians []Arc3, edges []*Edge) {
	patch := map[*Face]struct{}{}
	for _, f := range filletPatchesOf(body) {
		patch[f] = struct{}{}
	}
	for _, e := range body.Edges() {
		fs := e.Faces()
		if len(fs) != 2 {
			continue
		}
		_, a := patch[fs[0]]
		_, b := patch[fs[1]]
		if !a || !b {
			continue
		}
		edges = append(edges, e)
		switch c := e.Curve().(type) {
		case Ellipse3:
			ellipses = append(ellipses, c)
		case Arc3:
			meridians = append(meridians, c)
		}
	}
	return ellipses, meridians, edges
}

// filletRoles lists the filletLoop roles the body's faces carry.
func filletRoles(body *Body) []string {
	var out []string
	for _, f := range filletPatchesOf(body) {
		for _, o := range f.Origins() {
			out = append(out, o.Role)
		}
	}
	return out
}

// requireRolesOf asserts the band's patches carry filletLoop(f,l,0..n−1).
func requireRolesOf(t *testing.T, body *Body, b brepLoopBand, n int) {
	t.Helper()
	got := map[string]struct{}{}
	for _, r := range filletRoles(body) {
		got[r] = struct{}{}
	}
	for p := range n {
		_, ok := got[fmt.Sprintf("filletLoop(%d,%d,%d)", b.face, b.loop, p)]
		require.True(t, ok, "patch %d of band (%d, %d) carries its role", p, b.face, b.loop)
	}
}

// requireNormalFaces asserts the patch through p, a point of its surface,
// has an outward normal there with a positive dot product with dir.
func requireNormalFaces(t *testing.T, patches []*Face, p, dir r3.Vec) {
	t.Helper()
	for _, f := range patches {
		var d float64
		switch s := f.Surface().(type) {
		case Cylinder:
			rel := p.Sub(s.Origin)
			d = rel.Sub(s.Axis.Scale(rel.Dot(s.Axis))).Len() - s.Radius.Base()
		case Torus:
			rel := p.Sub(s.Center)
			radial := rel.Sub(s.Axis.Scale(rel.Dot(s.Axis)))
			ring, ok := radial.Normalize()
			if !ok {
				continue
			}
			d = p.Sub(s.Center.Add(ring.Scale(s.Major.Base()))).Len() - s.Minor.Base()
		}
		if math.Abs(d) > 1e-9 {
			continue
		}
		n, err := f.NormalAt(p)
		require.NoError(t, err)
		require.Positive(t, n.Value.Dot(dir), "the patch through %v faces %v", p, dir)
		return
	}
	t.Fatalf("no fillet patch passes through %v", p)
}

// requireBandEdgesConvex asserts every edge of a fillet patch reads the
// band's sense: convex where the band removes material, concave where it
// fills.
func requireBandEdgesConvex(t *testing.T, body *Body, convex bool) {
	t.Helper()
	for _, f := range filletPatchesOf(body) {
		for _, e := range f.Edges() {
			require.Equal(t, convex, e.IsConvex(), "edge %v of a fillet patch", e.Curve())
		}
	}
}

// TestBrepLoopFilletCrossDrilled fillets P1, S1's 40×20×20 box drilled Ø6
// along y (docs/loop-fillet-design.md §8). Its top loop at r = 2 removes the
// strip 120·J₁ − 4·J₂, so the volume is 46720/3 − 76π; the four quarter
// cylinders meet along four Ellipse3 edges with semi-axes 2√2 and 2 centred on
// the offset corners (2, 2, 18) and its images; the x walls (sw) end at
// z = 18 and the y walls (pl) hold their top segment there; the area is
// 3568 + 206π (the receiver's 4000 + 102π less the top strip 224 and the
// walls' 240, plus the patches' 120π − 16(π − 2)); the centroid is (20, 10,
// Mz/V) with Mz = 456496/3 + 72π; the box is the receiver's, exactly; every
// patch edge is convex. Both loops in one call give 45440/3 + 28π, centroid
// height 10. The y = 0 wall's hole rim at r = 1 is one whole-turn torus about
// ŷ of major radius 4, bounded by a circle of radius 4 on the wall and one of
// radius 3 at y = 1, and removes 23π/3 − 2π².
func TestBrepLoopFilletCrossDrilled(t *testing.T) {
	t.Parallel()
	t.Run("top loop", func(t *testing.T) {
		t.Parallel()
		_, s1 := internalCrossDrilled(t)
		out, bp := filletLoopOfFace(t, s1, routeEZ, r3.NewVec(0, 0, 20), 0, 2)
		v := pp(q(46720, 3), q(-76, 1), q(0, 1))
		requireVolumePi(t, out, v)
		requireCentroidPi(t, out.centroid, [3]piPoly{
			pp(q(20*46720, 3), q(-20*76, 1), q(0, 1)),
			pp(q(10*46720, 3), q(-10*76, 1), q(0, 1)),
			pp(q(456496, 3), q(72, 1), q(0, 1)),
		}, v, noOffset())
		lo, hi := pp(q(3568, 1), q(206, 1), q(0, 1)).enclosed()
		requireCoversInterval(t, out.area, lo, hi)
		require.Equal(t, Exact, out.bounds.Exactness)
		require.Equal(t, r3.Vec{}, out.bounds.Min)
		require.Equal(t, r3.NewVec(40, 20, 20), out.bounds.Max)

		patches := requireFilletPatches(t, out, 4, 0)
		ellipses, meridians, _ := filletEndCurves(out)
		require.Empty(t, meridians)
		require.Len(t, ellipses, 4)
		var centres []r3.Vec
		for _, e := range ellipses {
			centres = append(centres, e.Center)
			require.InDelta(t, 2*math.Sqrt2, e.SemiMajor.Base(), 1e-12)
			require.Equal(t, 2.0, e.SemiMinor.Base())
		}
		require.ElementsMatch(t, []r3.Vec{{X: 2, Y: 2, Z: 18}, {X: 38, Y: 2, Z: 18}, {X: 38, Y: 18, Z: 18}, {X: 2, Y: 18, Z: 18}}, centres)
		for _, wall := range []curveSegment{
			lineSeg{Start: Point2{U: 40, V: 0}, End: Point2{U: 40, V: 20}, TStart: 0, TEnd: 1},
			lineSeg{Start: Point2{U: 0, V: 20}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
		} {
			require.Equal(t, [][2]float64{{0, 18}}, sweptFaceLevels(bp, wall), "x wall %v", wall)
		}
		yWall := planarFaceAt(t, bp, r3.NewVec(0, -1, 0), 0)
		requireChord(t, yWall.region.Outer, Point2{U: 40, V: 18}, Point2{U: 0, V: 18})
		requireRolesOf(t, out, bp.loopBands[0], 4)
		requireBandEdgesConvex(t, out, true)
		requireNormalFaces(t, patches, r3.NewVec(38+math.Sqrt2, 10, 18+math.Sqrt2), r3.NewVec(1, 0, 1))
		for _, f := range patches {
			c := f.Surface().(Cylinder)
			require.Equal(t, 2.0, c.Radius.Base())
		}
	})
	t.Run("both loops", func(t *testing.T) {
		t.Parallel()
		_, s1 := internalCrossDrilled(t)
		top := planarBodyFace(t, s1, routeEZ, r3.NewVec(0, 0, 20))
		bottom := planarBodyFace(t, s1, routeEZ.Scale(-1), r3.Vec{})
		out, _ := filletSelection(t, s1, edgesQuery(append(top.Loops()[0].Edges(), bottom.Loops()[0].Edges()...)), 2, 2)
		v := pp(q(45440, 3), q(28, 1), q(0, 1))
		requireVolumePi(t, out, v)
		requireCentroidPi(t, out.centroid, [3]piPoly{
			pp(q(20*45440, 3), q(20*28, 1), q(0, 1)),
			pp(q(10*45440, 3), q(10*28, 1), q(0, 1)),
			pp(q(10*45440, 3), q(10*28, 1), q(0, 1)),
		}, v, noOffset())
		requireFilletPatches(t, out, 8, 0)
		ellipses, _, _ := filletEndCurves(out)
		require.Len(t, ellipses, 8)
	})
	t.Run("hole rim", func(t *testing.T) {
		t.Parallel()
		_, s1 := internalCrossDrilled(t)
		out, bp := filletLoopOfFace(t, s1, r3.NewVec(0, -1, 0), r3.Vec{}, 1, 1)
		requireVolumePi(t, out, pp(q(16000, 1), q(-563, 3), q(2, 1)))
		patches := requireFilletPatches(t, out, 0, 1)
		tor := patches[0].Surface().(Torus)
		require.Equal(t, r3.NewVec(20, 1, 10), tor.Center)
		require.InDelta(t, 1, math.Abs(tor.Axis.Dot(r3.NewVec(0, 1, 0))), 1e-15)
		require.Equal(t, 4.0, tor.Major.Base())
		require.Equal(t, 1.0, tor.Minor.Base())
		var radii []float64
		for _, e := range patches[0].Edges() {
			c, ok := e.Curve().(Circle3)
			require.True(t, ok, "a whole-turn patch is bounded by circles")
			radii = append(radii, c.Radius.Base())
			if c.Radius.Base() == 3 {
				require.Equal(t, 1.0, c.Center.Y)
			}
		}
		require.ElementsMatch(t, []float64{3, 4}, radii)
		requireRolesOf(t, out, bp.loopBands[0], 1)
		requireBandEdgesConvex(t, out, true)
		// At the hole's top meridian the patch faces out of the material,
		// away from its ball centre (20, 1, 14): along −y and −z.
		requireNormalFaces(t, patches, r3.NewVec(20, 1-math.Sqrt2/2, 10+4-math.Sqrt2/2), r3.NewVec(0, -1, -1))
	})
}

// TestBrepLoopFilletPocket fillets P2, the 40×40×10 plate with the blind
// 20×10 pocket 5 deep, at r = 1.5 (§8; the receiver holds 15000). Its mouth,
// the top face's hole loop, has four reflex corners: four cylinders and four
// horn tori of major and minor radius 1.5 joined by eight Arc3 meridians,
// the pocket walls ending at z = 8.5, and 14865 + 225π/8 + 27π²/16 with the
// centroid (20, 20, Mz/V), Mz = 285275/4 + 15273π/64 + 459π²/32. Its top loop
// gives 29325/2 + 333π/4, both in one call 29055/2 + 891π/8 + 27π²/16. The
// floor loop rises off the floor: the band fills the concave corner,
// 30225/2 − 27π, the walls starting at z = 6.5, four concave cylinders
// meeting along concave Ellipse3 edges.
func TestBrepLoopFilletPocket(t *testing.T) {
	t.Parallel()
	t.Run("mouth", func(t *testing.T) {
		t.Parallel()
		_, pocket := internalRouteEPocket(t)
		out, bp := filletLoopOfFace(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 1, 1.5)
		v := pp(q(14865, 1), q(225, 8), q(27, 16))
		requireVolumePi(t, out, v)
		requireCentroidPi(t, out.centroid, [3]piPoly{
			pp(q(20*14865, 1), q(20*225, 8), q(20*27, 16)),
			pp(q(20*14865, 1), q(20*225, 8), q(20*27, 16)),
			pp(q(285275, 4), q(15273, 64), q(459, 32)),
		}, v, noOffset())
		patches := requireFilletPatches(t, out, 4, 4)
		ellipses, meridians, _ := filletEndCurves(out)
		require.Empty(t, ellipses)
		require.Len(t, meridians, 8)
		for _, m := range meridians {
			require.Equal(t, 1.5, m.Radius.Base())
		}
		var corners []r3.Vec
		for _, f := range patches {
			tor, ok := f.Surface().(Torus)
			if !ok {
				continue
			}
			require.Equal(t, 1.5, tor.Major.Base())
			require.Equal(t, 1.5, tor.Minor.Base())
			corners = append(corners, tor.Center)
			// The horn torus's apex is the corner itself, on its own axis:
			// the normal there does not exist (SF2).
			_, err := f.NormalAt(tor.Center)
			require.ErrorIs(t, err, ErrDegenerate)
		}
		require.ElementsMatch(t, []r3.Vec{{X: 10, Y: 15, Z: 8.5}, {X: 30, Y: 15, Z: 8.5}, {X: 30, Y: 25, Z: 8.5}, {X: 10, Y: 25, Z: 8.5}}, corners)
		walls := 0
		for _, f := range bp.faces {
			if !f.planar() && f.z0 == 5 {
				walls++
				require.Equal(t, 8.5, f.z1)
			}
		}
		require.Equal(t, 4, walls)
		requireRolesOf(t, out, bp.loopBands[0], 8)
		requireBandEdgesConvex(t, out, true)
		// The x = 10 wall's patch faces into the pocket and up.
		requireNormalFaces(t, patches, r3.NewVec(10-1.5+1.5/math.Sqrt2, 20, 8.5+1.5/math.Sqrt2), r3.NewVec(1, 0, 1))
	})
	t.Run("top loop", func(t *testing.T) {
		t.Parallel()
		_, pocket := internalRouteEPocket(t)
		out, _ := filletLoopOfFace(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 0, 1.5)
		requireVolumePi(t, out, pp(q(29325, 2), q(333, 4), q(0, 1)))
		requireFilletPatches(t, out, 4, 0)
	})
	t.Run("both loops", func(t *testing.T) {
		t.Parallel()
		_, pocket := internalRouteEPocket(t)
		top := planarBodyFace(t, pocket, routeEZ, r3.NewVec(0, 0, 10))
		out, _ := filletSelection(t, pocket, edgesQuery(append(top.Loops()[0].Edges(), top.Loops()[1].Edges()...)), 1.5, 2)
		requireVolumePi(t, out, pp(q(29055, 2), q(891, 8), q(27, 16)))
		requireFilletPatches(t, out, 8, 4)
	})
	t.Run("floor loop", func(t *testing.T) {
		t.Parallel()
		_, pocket := internalRouteEPocket(t)
		out, bp := filletLoopOfFace(t, pocket, routeEZ, r3.NewVec(0, 0, 5), 0, 1.5)
		requireVolumePi(t, out, pp(q(30225, 2), q(-27, 1), q(0, 1)))
		require.Equal(t, sigmaRise, bp.loopBands[0].sigma)
		patches := requireFilletPatches(t, out, 4, 0)
		ellipses, _, edges := filletEndCurves(out)
		require.Len(t, ellipses, 4)
		for _, e := range edges {
			require.False(t, e.IsConvex(), "the fill's ellipse is the receiver's concave vertical edge's")
		}
		requireBandEdgesConvex(t, out, false)
		walls := 0
		for _, f := range bp.faces {
			if !f.planar() && f.z1 == 10 && f.z0 != 0 {
				walls++
				require.Equal(t, 6.5, f.z0)
			}
		}
		require.Equal(t, 4, walls)
		// The fill on the x = 10 wall faces into the pocket and up, away
		// from the ball centre (11.5, 20, 6.5).
		requireNormalFaces(t, patches, r3.NewVec(11.5-1.5/math.Sqrt2, 20, 6.5-1.5/math.Sqrt2), r3.NewVec(1, 0, 1))
	})
}

// TestBrepLoopFilletRoundBoss fillets P3, the 40×40×10 plate unioned with the
// Ø10 boss 15 tall, at r = 1. Its root rises off the plate: one whole-turn
// torus of major radius 6 about (20, 20, 10), filling 35π/3 − 3π², so
// 16000 + 1160π/3 − 3π² with the centroid (20, 20, Mz/V),
// Mz = 80000 + 80269π/12 − 33π², the plate top's hole widened to 6 and the
// boss wall starting at z = 11. Its rim descends: one torus of major radius
// 4, 16000 + 1100π/3 + 2π². Both in one call: 16000 + 1135π/3 − π².
func TestBrepLoopFilletRoundBoss(t *testing.T) {
	t.Parallel()
	t.Run("root", func(t *testing.T) {
		t.Parallel()
		out, bp := filletLoopOfFace(t, internalRoundBoss(t), routeEZ, r3.NewVec(0, 0, 10), 1, 1)
		v := pp(q(16000, 1), q(1160, 3), q(-3, 1))
		requireVolumePi(t, out, v)
		requireCentroidPi(t, out.centroid, [3]piPoly{
			pp(q(20*16000, 1), q(20*1160, 3), q(-60, 1)),
			pp(q(20*16000, 1), q(20*1160, 3), q(-60, 1)),
			pp(q(80000, 1), q(80269, 12), q(-33, 1)),
		}, v, noOffset())
		patches := requireFilletPatches(t, out, 0, 1)
		tor := patches[0].Surface().(Torus)
		require.Equal(t, r3.NewVec(20, 20, 11), tor.Center)
		require.Equal(t, 6.0, tor.Major.Base())
		plateTop := planarFaceAt(t, bp, routeEZ, 10)
		hole, ok := plateTop.region.Holes[0].Segments[0].(circleSeg)
		require.True(t, ok)
		require.Equal(t, 6.0, hole.Radius.Base())
		requireBandEdgesConvex(t, out, false)
		// The fill faces up and away from the boss, toward its ball centre
		// ring at radius 6, z = 11.
		requireNormalFaces(t, patches, r3.NewVec(26-1/math.Sqrt2, 20, 11-1/math.Sqrt2), r3.NewVec(1, 0, 1))
	})
	t.Run("rim", func(t *testing.T) {
		t.Parallel()
		out, _ := filletLoopOfFace(t, internalRoundBoss(t), routeEZ, r3.NewVec(0, 0, 25), 0, 1)
		requireVolumePi(t, out, pp(q(16000, 1), q(1100, 3), q(2, 1)))
		patches := requireFilletPatches(t, out, 0, 1)
		require.Equal(t, 4.0, patches[0].Surface().(Torus).Major.Base())
		requireNormalFaces(t, patches, r3.NewVec(24+1/math.Sqrt2, 20, 24+1/math.Sqrt2), r3.NewVec(1, 0, 1))
	})
	t.Run("both", func(t *testing.T) {
		t.Parallel()
		boss := internalRoundBoss(t)
		root := planarBodyFace(t, boss, routeEZ, r3.NewVec(0, 0, 10)).Loops()[1].Edges()
		rim := planarBodyFace(t, boss, routeEZ, r3.NewVec(0, 0, 25)).Loops()[0].Edges()
		out, _ := filletSelection(t, boss, edgesQuery(append(root, rim...)), 1, 2)
		requireVolumePi(t, out, pp(q(16000, 1), q(1135, 3), q(-1, 1)))
		requireFilletPatches(t, out, 0, 2)
	})
}

// internalFourHolePlate is §8's P4: the 60×40 plate with four Ø5 holes at
// (10, 10), (50, 10), (50, 30), (10, 30), extruded 8 — a prism receiver.
func internalFourHolePlate(t *testing.T) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 60, 40)
	s.Fix(rect.A)
	for _, c := range [][2]float64{{10, 10}, {50, 10}, {50, 30}, {10, 30}} {
		p := s.CreatePoint(c[0], c[1])
		s.Fix(p)
		s.CreateCircle(p, 2.5)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 4 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	body, err := New().Extrude(s, prof, Distance{D: units.Millimeters(8), Dir: Along})
	require.NoError(t, err)
	_, ok := body.payload.(prismPayload)
	require.True(t, ok)
	return body
}

// TestBrepLoopFilletFourHolePrism fillets P4, a prism receiver (RF3): the
// call reads the prism through its face view and returns a brep body. Its
// four hole mouths at r = 1 in one call carry four whole-turn tori of major
// radius 3.5 and give 19200 − 680π/3 + 7π²; its top loop gives
// 57020/3 − 152π.
func TestBrepLoopFilletFourHolePrism(t *testing.T) {
	t.Parallel()
	t.Run("hole mouths", func(t *testing.T) {
		t.Parallel()
		plate := internalFourHolePlate(t)
		top := planarBodyFace(t, plate, routeEZ, r3.NewVec(0, 0, 8))
		var edges []*Edge
		for _, l := range top.Loops()[1:] {
			edges = append(edges, l.Edges()...)
		}
		out, _ := filletSelection(t, plate, edgesQuery(edges), 1, 4)
		requireVolumePi(t, out, pp(q(19200, 1), q(-680, 3), q(7, 1)))
		patches := requireFilletPatches(t, out, 0, 4)
		for _, f := range patches {
			require.Equal(t, 3.5, f.Surface().(Torus).Major.Base())
		}
	})
	t.Run("top loop", func(t *testing.T) {
		t.Parallel()
		out, _ := filletLoopOfFace(t, internalFourHolePlate(t), routeEZ, r3.NewVec(0, 0, 8), 0, 1)
		requireVolumePi(t, out, pp(q(57020, 3), q(-152, 1), q(0, 1)))
		requireFilletPatches(t, out, 4, 0)
	})
}

// TestBrepLoopFilletBlindPortMouth fillets P6c's port mouth on the x = 60
// wall at r = 1.5: P2's mouth on a wall face, 69265 + 225π/8 + 27π²/16.
func TestBrepLoopFilletBlindPortMouth(t *testing.T) {
	t.Parallel()
	out, _ := filletLoopOfFace(t, internalBlindPort(t), r3.NewVec(1, 0, 0), r3.NewVec(60, 0, 0), 1, 1.5)
	requireVolumePi(t, out, pp(q(69265, 1), q(225, 8), q(27, 16)))
	patches := requireFilletPatches(t, out, 4, 4)
	requireNormalFaces(t, patches, r3.NewVec(58.5+1.5/math.Sqrt2, 20, 10-1.5+1.5/math.Sqrt2), r3.NewVec(1, 0, 1))
}

// TestBrepLoopFilletLBracket fillets P7, the L bracket drilled Ø6 along x. Its
// top loop at r = 1 has five convex corners and one reflex one at (8, 8):
// six cylinders, five Ellipse3 edges, one horn torus with its two meridians,
// a₂ = −5 + π/4, so 51385/3 − 419π/12 + π²/8. One of the hole's rims, on the
// upright leg's x = 0 face, at r = 1: one torus about x̂ through the hole
// centre, 17280 − 239π/3 + 2π².
func TestBrepLoopFilletLBracket(t *testing.T) {
	t.Parallel()
	t.Run("top loop", func(t *testing.T) {
		t.Parallel()
		out, _ := filletLoopOfFace(t, internalLBracket(t), routeEZ, r3.NewVec(0, 0, 30), 0, 1)
		requireVolumePi(t, out, pp(q(51385, 3), q(-419, 12), q(1, 8)))
		requireFilletPatches(t, out, 6, 1)
		ellipses, meridians, _ := filletEndCurves(out)
		require.Len(t, ellipses, 5)
		require.Len(t, meridians, 2)
	})
	t.Run("hole rim", func(t *testing.T) {
		t.Parallel()
		out, _ := filletLoopOfFace(t, internalLBracket(t), r3.NewVec(-1, 0, 0), r3.Vec{}, 1, 1)
		requireVolumePi(t, out, pp(q(17280, 1), q(-239, 3), q(2, 1)))
		patches := requireFilletPatches(t, out, 0, 1)
		tor := patches[0].Surface().(Torus)
		require.Equal(t, r3.NewVec(1, 24, 15), tor.Center)
		require.InDelta(t, 1, math.Abs(tor.Axis.Dot(r3.NewVec(1, 0, 0))), 1e-15)
		require.Equal(t, 4.0, tor.Major.Base())
	})
}

// TestBrepLoopFilletRoundedPlate fillets P8's top loop at r = 2: four lines
// joining four r = 3 arcs at G1 joins, so four cylinders and four spindle
// tori of major radius 1 and minor 2, joined by eight Arc3 meridians and no
// ellipse; 14896 + 256π/3 + 2π².
func TestBrepLoopFilletRoundedPlate(t *testing.T) {
	t.Parallel()
	out, _ := filletLoopOfFace(t, internalRoundedPlate(t), routeEZ, r3.NewVec(0, 0, 20), 0, 2)
	requireVolumePi(t, out, pp(q(14896, 1), q(256, 3), q(2, 1)))
	patches := requireFilletPatches(t, out, 4, 4)
	for _, f := range patches {
		if tor, ok := f.Surface().(Torus); ok {
			require.Equal(t, 1.0, tor.Major.Base())
			require.Equal(t, 2.0, tor.Minor.Base())
		}
	}
	ellipses, meridians, _ := filletEndCurves(out)
	require.Empty(t, ellipses)
	require.Len(t, meridians, 8)
	// The corner torus about (3, 3) bulges out along (−1, −1, √2) at its
	// 45° meridian.
	d := r3.NewVec(-1, -1, 0).Scale(1 / math.Sqrt2)
	p := r3.NewVec(3, 3, 18).Add(d.Scale(1)).Add(d.Scale(2 / math.Sqrt2)).Add(r3.NewVec(0, 0, 2/math.Sqrt2))
	requireNormalFaces(t, patches, p, r3.NewVec(-1, -1, math.Sqrt2))
}

// The four radius-3 side arcs collapse to the centres of four spherical
// octants when P8's top loop is filleted at the same radius.
func TestBrepLoopFilletSphereAtEqualRadius(t *testing.T) {
	t.Parallel()
	out, _ := filletLoopOfFace(t, internalRoundedPlate(t), routeEZ, r3.NewVec(0, 0, 20), 0, 3)
	requireVolumePi(t, out, pp(q(14416, 1), q(207, 1), q(0, 1)))
	patches := filletPatchesOf(out)
	require.Len(t, patches, 8)
	var spheres int
	for _, f := range patches {
		if sphere, ok := f.Surface().(Sphere); ok {
			spheres++
			require.Equal(t, 3.0, sphere.Radius.Base())
		}
	}
	require.Equal(t, 4, spheres)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())
}

func TestVertexBlendBoxAllEdges(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	out, err := box.Fillet(t.Context(), Edges().Exactly(12), units.Millimeters(2))
	require.NoError(t, err)
	requireClosedTopology(t, out)
	requireVolumePi(t, out, pp(q(14848, 1), q(848, 3), q(0, 1)))
	var spheres int
	for _, f := range filletPatchesOf(out) {
		if _, ok := f.Surface().(Sphere); ok {
			spheres++
		}
	}
	require.Equal(t, 8, spheres)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())
}

func TestVertexBlendBoxTopAndVerticalEdges(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	sel := Edges(EndpointAt(r3.NewVec(0, 0, 20))).Or(EndpointAt(r3.NewVec(40, 20, 20))).Or(ParallelTo(routeEZ)).Exactly(8)
	out, _ := filletSelection(t, box, sel, 2, 1)
	requireVolumePi(t, out, pp(q(15264, 1), q(544, 3), q(0, 1)))
	var spheres int
	for _, f := range filletPatchesOf(out) {
		if _, ok := f.Surface().(Sphere); ok {
			spheres++
		}
	}
	require.Equal(t, 4, spheres)
}

func TestVertexBlendSinglePrismCapEdge(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	out, err := box.Fillet(t.Context(), Edges(ParallelTo(routeEX), EndpointAt(r3.NewVec(0, 0, 20))).Exactly(1), units.Millimeters(2))
	require.NoError(t, err)
	requireClosedTopology(t, out)
	requireVolumePi(t, out, pp(q(15840, 1), q(40, 1), q(0, 1)))
}

func TestVertexBlendPocketFloor(t *testing.T) {
	t.Parallel()
	_, pocket := internalRouteEPocket(t)
	sel := Edges(EndpointAt(r3.NewVec(10, 15, 5))).Or(EndpointAt(r3.NewVec(30, 25, 5))).Or(ParallelTo(routeEZ), Concave()).Exactly(8)
	out, _ := filletSelection(t, pocket, sel, 1.5, 1)
	requireVolumePi(t, out, pp(q(15153, 1), q(-297, 8), q(0, 1)))
	var spheres int
	for _, f := range filletPatchesOf(out) {
		if _, ok := f.Surface().(Sphere); ok {
			spheres++
		}
	}
	require.Equal(t, 4, spheres)
}

func TestVertexBlendBoxPartialCornerSet(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	sel := Edges(ParallelTo(routeEX), EndpointAt(r3.NewVec(0, 0, 20))).
		Or(ParallelTo(r3.NewVec(0, 1, 0)), EndpointAt(r3.NewVec(0, 0, 20))).
		Or(ParallelTo(routeEX), EndpointAt(r3.NewVec(40, 20, 20))).
		Or(ParallelTo(r3.NewVec(0, 1, 0)), EndpointAt(r3.NewVec(40, 20, 20))).
		Or(ParallelTo(routeEZ), EndpointAt(r3.NewVec(0, 0, 0))).
		Or(ParallelTo(routeEZ), EndpointAt(r3.NewVec(40, 0, 0))).Exactly(6)
	out, _ := filletSelection(t, box, sel, 2, 1)
	var spheres, ellipses int
	for _, f := range filletPatchesOf(out) {
		if _, ok := f.Surface().(Sphere); ok {
			spheres++
		}
	}
	for _, e := range out.Edges() {
		if _, ok := e.Curve().(Ellipse3); ok {
			ellipses++
		}
	}
	require.Equal(t, 2, spheres)
	require.Equal(t, 2, ellipses)
}

// The slanted corner's pinned arc endpoints do not both lie exactly one
// millimetre from its centre, so a radius-one sphere cannot close against it.
func TestVertexBlendTrapezoidRefusal(t *testing.T) {
	t.Parallel()
	doc := New()
	body := internalPolygonPrism(t, doc, [][2]float64{{0, 0}, {100, 0}, {72, 45}, {28, 45}}, 10)
	_, err := body.Fillet(t.Context(), Edges().Exactly(12), units.Millimeters(1))
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "recorded endpoints to lie exactly at the fillet radius")
	require.NoError(t, doc.requireLive(body))
}

func TestVertexBlendObliqueTopEdge(t *testing.T) {
	t.Parallel()
	doc := New()
	body := internalPolygonPrism(t, doc, [][2]float64{{0, 0}, {100, 0}, {72, 45}, {28, 45}}, 10)
	sel := Edges(EndpointAt(r3.NewVec(100, 0, 10)), EndpointAt(r3.NewVec(72, 45, 10))).Exactly(1)
	out, err := body.Fillet(t.Context(), sel, units.Millimeters(1))
	require.NoError(t, err)
	require.Error(t, doc.requireLive(body))
	require.Equal(t, []*Body{out}, doc.Bodies())
	_, faceted := out.payload.(facetedPayload)
	require.True(t, faceted)
	wantVolume := 32400 - 53*(1-math.Pi/4)
	require.LessOrEqual(t, math.Abs(out.volume.Value.Base()-wantVolume), out.volume.Bound.Base()+1e-6)
	requireClosedTopology(t, out)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, ValidityValid, reading.Validity.Outcome)
}

func TestVertexBlendEdgeBesideObliqueWalls(t *testing.T) {
	t.Parallel()
	doc := New()
	body := internalPolygonPrism(t, doc, [][2]float64{{0, 0}, {100, 0}, {72, 45}, {28, 45}}, 10)
	sel := Edges(EndpointAt(r3.NewVec(28, 45, 10)), EndpointAt(r3.NewVec(72, 45, 10))).Exactly(1)
	out, err := body.Fillet(t.Context(), sel, units.Millimeters(1))
	require.NoError(t, err)
	require.Error(t, doc.requireLive(body))
	require.Equal(t, []*Body{out}, doc.Bodies())
	_, faceted := out.payload.(facetedPayload)
	require.True(t, faceted)
	wantVolume := 32400 - 44*(1-math.Pi/4) - (28.0/45.0)*(5.0/3.0-math.Pi/2)
	require.LessOrEqual(t, math.Abs(out.volume.Value.Base()-wantVolume), out.volume.Bound.Base()+1e-6)
	requireClosedTopology(t, out)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, ValidityValid, reading.Validity.Outcome)
}

func TestVertexBlendObliqueBottomEdge(t *testing.T) {
	t.Parallel()
	doc := New()
	body := internalPolygonPrism(t, doc, [][2]float64{{0, 0}, {100, 0}, {72, 45}, {28, 45}}, 10)
	sel := Edges(EndpointAt(r3.NewVec(100, 0, 0)), EndpointAt(r3.NewVec(72, 45, 0))).Exactly(1)
	out, err := body.Fillet(t.Context(), sel, units.Millimeters(1))
	require.NoError(t, err)
	requireClosedTopology(t, out)
	wantVolume := 32400 - 53*(1-math.Pi/4)
	require.LessOrEqual(t, math.Abs(out.volume.Value.Base()-wantVolume), out.volume.Bound.Base()+1e-6)
}

func TestVertexBlendCrossFaceTwoEdgeCorner(t *testing.T) {
	t.Parallel()
	doc := New()
	body := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	sel := Edges(ParallelTo(routeEX), EndpointAt(r3.NewVec(0, 0, 20))).
		Or(ParallelTo(routeEZ), EndpointAt(r3.NewVec(0, 0, 20))).Exactly(2)
	out, err := body.Fillet(t.Context(), sel, units.Millimeters(2))
	require.NoError(t, err)
	requireClosedTopology(t, out)
	requireVolumePi(t, out, pp(q(47320, 3), q(56, 1), q(0, 1)))
	requireFilletPatches(t, out, 2, 0)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, Sound, reading.Status)
}

func TestVertexBlendCrossFaceBrepCorner(t *testing.T) {
	t.Parallel()
	doc, body := internalCrossDrilled(t)
	sel := Edges(ParallelTo(routeEX), EndpointAt(r3.NewVec(0, 0, 20))).
		Or(ParallelTo(routeEZ), EndpointAt(r3.NewVec(0, 0, 20))).Exactly(2)
	out, err := body.Fillet(t.Context(), sel, units.Millimeters(2))
	require.NoError(t, err)
	requireClosedTopology(t, out)
	requireVolumePi(t, out, pp(q(47320, 3), q(-124, 1), q(0, 1)))
	requireFilletPatches(t, out, 2, 0)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, Sound, reading.Status)
}

func TestVertexBlendThreeEdgesAtOneBoxVertex(t *testing.T) {
	t.Parallel()
	doc := New()
	body := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	corner := r3.NewVec(0, 0, 20)
	sel := Edges(ParallelTo(routeEX), EndpointAt(corner)).
		Or(ParallelTo(r3.NewVec(0, 1, 0)), EndpointAt(corner)).
		Or(ParallelTo(routeEZ), EndpointAt(corner)).Exactly(3)
	out, err := body.Fillet(t.Context(), sel, units.Millimeters(2))
	require.NoError(t, err)
	requireClosedTopology(t, out)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())
	requireCornerSphere(t, out)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, Sound, reading.Status)
}

func TestVertexBlendPartialSphereAfterLateralEdge(t *testing.T) {
	t.Parallel()
	doc := New()
	body := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	corner := r3.NewVec(0, 0, 20)
	step, err := body.Fillet(t.Context(), Edges(ParallelTo(routeEZ), EndpointAt(corner)).Exactly(1), units.Millimeters(2))
	require.NoError(t, err)
	top := planarBodyFace(t, step, routeEZ, corner).Loops()[0].Edges()
	require.Len(t, top, 5)
	out, err := step.Fillet(t.Context(), edgesQuery([]*Edge{top[0], top[3], top[4]}), units.Millimeters(2))
	require.NoError(t, err)
	requireClosedTopology(t, out)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())
	requireCornerSphere(t, out)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, Sound, reading.Status)
}

func requireCornerSphere(t *testing.T, body *Body) {
	t.Helper()
	spheres := 0
	for _, face := range filletPatchesOf(body) {
		if _, ok := face.Surface().(Sphere); ok {
			spheres++
		}
	}
	require.Equal(t, 1, spheres)
}

func TestPartialFilletContourCollapsesSelectedArc(t *testing.T) {
	t.Parallel()
	doc := New()
	body := internalBoxBody(t, doc, 0, 0, 40, 20, 20)
	corner := r3.NewVec(0, 0, 20)
	step, err := body.Fillet(t.Context(), Edges(ParallelTo(routeEZ), EndpointAt(corner)).Exactly(1), units.Millimeters(2))
	require.NoError(t, err)
	top := planarBodyFace(t, step, routeEZ, corner).Loops()[0].Edges()
	bp, err := brepOfPrism(step.payload.(prismPayload))
	require.NoError(t, err)
	r, err := newBrepLoopRead(t.Context(), bp, brepModifyRequest{edges: []*Edge{top[0], top[3], top[4]}})
	require.NoError(t, err)
	require.NoError(t, r.matchEdges())
	sel, selected, ok := r.partialSelectedLoop()
	require.True(t, ok)
	cl, err := oneLoopCornerLoop(r.budget, bp.faces[sel.face].regionLoop(sel.loop), freeform.NewFreeformWork())
	require.NoError(t, err)
	amounts := make([]float64, len(selected))
	sphere := make([]bool, len(selected))
	for i, on := range selected {
		if on {
			amounts[i] = 2
			sphere[i] = filletSphereWalk(cl.walks[i], 2)
		}
	}
	segs, joins, capWalk, _, err := partialFilletContour(r.budget, cl.walks, selected, sphere, amounts)
	require.NoError(t, err)
	require.Len(t, segs, 4)
	for i, w := range cl.walks {
		if !w.IsCircular() {
			continue
		}
		require.True(t, selected[i])
		require.Equal(t, -1, capWalk[i])
		pole := Point2{U: w.CU, V: w.CV}
		require.Equal(t, pole, Point2{U: joins[i].M.U, V: joins[i].M.V})
		require.Equal(t, pole, Point2{U: joins[(i+1)%len(joins)].M.U, V: joins[(i+1)%len(joins)].M.V})
	}
}

func TestFilletSphereWalkRequiresExactRecordedRadius(t *testing.T) {
	t.Parallel()
	w := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		Kind: survey2d.WalkCircular, Radius: 1, StartU: 1, EndV: 1, Th1: math.Pi / 2,
	}}
	require.True(t, filletSphereWalk(w, 1))
	w.EndU, w.EndV = 0.6, 0.8
	require.Equal(t, 1.0, math.Hypot(w.EndU, w.EndV))
	require.False(t, filletSphereWalk(w, 1))
}

func TestVertexBlendCrossDrilledBar(t *testing.T) {
	t.Parallel()
	_, body := internalCrossDrilled(t)
	sel := Edges(ParallelTo(routeEX)).Or(ParallelTo(r3.NewVec(0, 1, 0))).Or(ParallelTo(routeEZ)).Exactly(12)
	out, err := body.Fillet(t.Context(), sel, units.Millimeters(2))
	require.NoError(t, err)
	requireClosedTopology(t, out)
	requireVolumePi(t, out, pp(q(14848, 1), q(308, 3), q(0, 1)))
	var spheres int
	for _, f := range filletPatchesOf(out) {
		if _, ok := f.Surface().(Sphere); ok {
			spheres++
		}
	}
	require.Equal(t, 8, spheres)
}

// trapezoidStrip is the bound fixture's strip polynomial derived from the
// polygon alone: the trapezoid's area and first moment less its offset by t,
// whose corners are the exact meets of the offset lines, at t = 1, 2, 3,
// solved for A(t) = a₁t + a₂t² and M(t) = m₁t + m₂t² + m₃t³.
func trapezoidStrip() (a [2]*big.Rat, m [2][3]*big.Rat) {
	pts := [][2]int64{{0, 0}, {100, 0}, {72, 45}, {28, 45}}
	lengths := []int64{100, 53, 44, 53}
	moment := func(poly [][2]*big.Rat) (*big.Rat, [2]*big.Rat) {
		area, mx, my := new(big.Rat), new(big.Rat), new(big.Rat)
		for i, p := range poly {
			r := poly[(i+1)%len(poly)]
			cr := new(big.Rat).Sub(new(big.Rat).Mul(p[0], r[1]), new(big.Rat).Mul(r[0], p[1]))
			area.Add(area, cr)
			mx.Add(mx, new(big.Rat).Mul(cr, new(big.Rat).Add(p[0], r[0])))
			my.Add(my, new(big.Rat).Mul(cr, new(big.Rat).Add(p[1], r[1])))
		}
		return area.Mul(area, q(1, 2)), [2]*big.Rat{mx.Mul(mx, q(1, 6)), my.Mul(my, q(1, 6))}
	}
	offset := func(t int64) [][2]*big.Rat {
		type line struct{ a, b, c *big.Rat }
		lines := make([]line, len(pts))
		for i, p := range pts {
			r := pts[(i+1)%len(pts)]
			dx, dy := r[0]-p[0], r[1]-p[1]
			lines[i] = line{q(-dy, 1), q(dx, 1), q(-dy*p[0]+dx*p[1]+t*lengths[i], 1)}
		}
		out := make([][2]*big.Rat, len(pts))
		for i := range lines {
			l0, l1 := lines[(i+len(lines)-1)%len(lines)], lines[i]
			det := new(big.Rat).Sub(new(big.Rat).Mul(l0.a, l1.b), new(big.Rat).Mul(l0.b, l1.a))
			out[i] = [2]*big.Rat{
				new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(l0.c, l1.b), new(big.Rat).Mul(l0.b, l1.c)), det),
				new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(l0.a, l1.c), new(big.Rat).Mul(l0.c, l1.a)), det),
			}
		}
		return out
	}
	a0, m0 := moment(offset(0))
	var as [3]*big.Rat
	var ms [3][2]*big.Rat
	for i := range 3 {
		ai, mi := moment(offset(int64(i + 1)))
		as[i] = new(big.Rat).Sub(a0, ai)
		ms[i] = [2]*big.Rat{new(big.Rat).Sub(m0[0], mi[0]), new(big.Rat).Sub(m0[1], mi[1])}
	}
	// A(1) = a₁ + a₂, A(2) = 2a₁ + 4a₂.
	a[1] = new(big.Rat).Quo(new(big.Rat).Sub(as[1], new(big.Rat).Mul(q(2, 1), as[0])), q(2, 1))
	a[0] = new(big.Rat).Sub(as[0], a[1])
	// M(t)/t = m₁ + m₂t + m₃t² through t = 1, 2, 3.
	for k := range 2 {
		y1, y2, y3 := ms[0][k], new(big.Rat).Quo(ms[1][k], q(2, 1)), new(big.Rat).Quo(ms[2][k], q(3, 1))
		d1 := new(big.Rat).Sub(y2, y1)
		d2 := new(big.Rat).Sub(y3, y2)
		m3 := new(big.Rat).Quo(new(big.Rat).Sub(d2, d1), q(2, 1))
		m2 := new(big.Rat).Sub(d1, new(big.Rat).Mul(q(3, 1), m3))
		m1 := new(big.Rat).Sub(new(big.Rat).Sub(y1, m2), m3)
		m[k] = [3]*big.Rat{m1, m2, m3}
	}
	return a, m
}

// TestBrepLoopFilletBoundFixture is §8's bound fixture: the trapezoid
// (0, 0), (100, 0), (72, 45), (28, 45) extruded 10 with a blind 20×20 pocket
// 5 deep (30400), its top loop filleted r = 1. Table CF gives a₁ = 250 and
// a₂ = −212/45, which the polygon's own exact offsets reproduce
// (trapezoidStrip), so the volume is 30400 − 250(1 − π/4) + (212/45)(5/3 −
// π/2).
//
// Leg 1: every cap-level vertex's published bound reaches its exact rational
// contour corner. Shown to fail with rewriteLoopFaces' charge of the contour
// displacement into F.delta deleted: all four corners, (9/5, 1),
// (491/5, 1), (643/9, 44) and (257/9, 44), then published zero bounds and
// missed their corners.
//
// Leg 2: the centroid of the body placed 10⁶ mm along every axis encloses the
// placed exact centroid, (Mx, My, Mz)/V + (10⁶, 10⁶, 10⁶), with M the
// receiver's moments
// less the strip's, m₁J₁ + m₂J₂ + m₃J₃ in plane and 9·V_strip + a₁/6 + a₂/12
// along z. Shown to fail with prismPointBoundWith's frame-lift rounding term
// deleted: the published bound then held the centroid's own enclosure alone,
// about 5e-15, below the lift's rounding at 10⁶. The centroid's x, exactly
// 50 by symmetry, lifts to 10⁶ + 50 with no rounding, so a placement along x
// alone would not exercise the term.
//
// The volume's own bound is the enclosures' reach alone: no single term of it
// can be deleted to turn it red, which this test records rather than pins.
func TestBrepLoopFilletBoundFixture(t *testing.T) {
	t.Parallel()
	a, m := trapezoidStrip()
	require.Zero(t, a[0].Cmp(q(250, 1)))
	require.Zero(t, a[1].Cmp(q(-212, 45)))
	j1 := pp(q(1, 1), q(-1, 4), q(0, 1))
	j2 := pp(q(5, 3), q(-1, 2), q(0, 1))
	j3 := pp(q(3, 1), q(-15, 16), q(0, 1))
	scale := func(y piPoly, s *big.Rat) piPoly {
		return pp(new(big.Rat).Mul(y[0], s), new(big.Rat).Mul(y[1], s), new(big.Rat).Mul(y[2], s))
	}
	add := func(ys ...piPoly) piPoly {
		out := pp(q(0, 1), q(0, 1), q(0, 1))
		for _, y := range ys {
			for i := range out {
				out[i] = new(big.Rat).Add(out[i], y[i])
			}
		}
		return out
	}
	strip := add(scale(j1, a[0]), scale(j2, a[1]))
	v := add(pp(q(30400, 1), q(0, 1), q(0, 1)), scale(strip, q(-1, 1)))

	body := internalTrapezoidPocket(t)
	out, bp := filletLoopOfFace(t, body, routeEZ, r3.NewVec(0, 0, 10), 0, 1)
	requireVolumePi(t, out, v)
	require.Positive(t, planarFaceAt(t, bp, routeEZ, 10).delta, "the contour is a float solve")
	requireFilletPatches(t, out, 4, 0)

	// Leg 1.
	pts := [][2]int64{{0, 0}, {100, 0}, {72, 45}, {28, 45}}
	lengths := []int64{100, 53, 44, 53}
	type line struct{ a, b, c *big.Rat }
	lines := make([]line, len(pts))
	for i, p := range pts {
		r := pts[(i+1)%len(pts)]
		dx, dy := r[0]-p[0], r[1]-p[1]
		lines[i] = line{q(-dy, 1), q(dx, 1), q(-dy*p[0]+dx*p[1]+lengths[i], 1)}
	}
	vertices := map[*Vertex]struct{}{}
	for _, e := range out.Edges() {
		vertices[e.start], vertices[e.end] = struct{}{}, struct{}{}
	}
	for i := range lines {
		l0, l1 := lines[(i+len(lines)-1)%len(lines)], lines[i]
		det := new(big.Rat).Sub(new(big.Rat).Mul(l0.a, l1.b), new(big.Rat).Mul(l0.b, l1.a))
		x := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(l0.c, l1.b), new(big.Rat).Mul(l0.b, l1.c)), det)
		y := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(l0.a, l1.c), new(big.Rat).Mul(l0.c, l1.a)), det)
		fx, _ := x.Float64()
		fy, _ := y.Float64()
		var held *Vertex
		for vv := range vertices {
			if vv.position.Sub(r3.NewVec(fx, fy, 10)).Len() < 1e-9 {
				require.Nil(t, held)
				held = vv
			}
		}
		require.NotNil(t, held, "a vertex sits at the contour corner (%v, %v)", fx, fy)
		requireCentroidCovers(t, VecMeasurement{Value: held.position, Bound: held.bound}, [3]*big.Rat{x, y, q(10, 1)})
	}

	// Leg 2. The receiver's moments: the trapezoid prism less the pocket.
	sec := [][2]*big.Rat{{q(0, 1), q(0, 1)}, {q(100, 1), q(0, 1)}, {q(72, 1), q(45, 1)}, {q(28, 1), q(45, 1)}}
	var my *big.Rat
	{
		acc := new(big.Rat)
		for i, p := range sec {
			r := sec[(i+1)%len(sec)]
			cr := new(big.Rat).Sub(new(big.Rat).Mul(p[0], r[1]), new(big.Rat).Mul(r[0], p[1]))
			acc.Add(acc, new(big.Rat).Mul(cr, new(big.Rat).Add(p[1], r[1])))
		}
		my = acc.Mul(acc, q(10, 6))
	}
	recv := [3]*big.Rat{
		q(50*30400, 1),
		new(big.Rat).Sub(my, q(2000*20, 1)),
		q(32400*5-2000*15/2, 1),
	}
	var mBody [3]piPoly
	for k := range 2 {
		in := add(scale(j1, m[k][0]), scale(j2, m[k][1]), scale(j3, m[k][2]))
		mBody[k] = add(pp(recv[k], q(0, 1), q(0, 1)), scale(in, q(-1, 1)))
	}
	axial := add(scale(strip, q(9, 1)), pp(new(big.Rat).Add(new(big.Rat).Quo(a[0], q(6, 1)), new(big.Rat).Quo(a[1], q(12, 1))), q(0, 1), q(0, 1)))
	mBody[2] = add(pp(recv[2], q(0, 1), q(0, 1)), scale(axial, q(-1, 1)))
	requireCentroidPi(t, out.centroid, mBody, v, noOffset())

	move, err := r3.Translation(r3.NewVec(1e6, 1e6, 1e6))
	require.NoError(t, err)
	placed, err := out.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	requireCentroidPi(t, placed.centroid, mBody, v, [3]*big.Rat{q(1000000, 1), q(1000000, 1), q(1000000, 1)})
}

// internalSemicircularBite builds a plate whose top loop has two sharp
// line-arc corners.
func internalSemicircularBite(t *testing.T) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	var pts []*sketch.Point
	for _, c := range [][2]float64{{0, 0}, {40, 0}, {40, 20}, {25, 20}, {15, 20}, {0, 20}} {
		p := s.CreatePoint(c[0], c[1])
		s.Fix(p)
		pts = append(pts, p)
	}
	centre := s.CreatePoint(20, 20)
	s.Fix(centre)
	s.CreateLine(pts[0], pts[1])
	s.CreateLine(pts[1], pts[2])
	s.CreateLine(pts[2], pts[3])
	s.CreateArc(centre, pts[4], pts[3])
	s.CreateLine(pts[4], pts[5])
	s.CreateLine(pts[5], pts[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, err)
	return body
}

func TestBrepLoopFilletSharpLineArcCorners(t *testing.T) {
	t.Parallel()
	body := internalSemicircularBite(t)
	top := planarBodyFace(t, body, routeEZ, r3.NewVec(0, 0, 10))
	out, _ := filletSelection(t, body, loopQuery(top.Loops()[0]), 1, 1)
	// At offset t the inset section is a rectangle less the circular bite.
	// Integrate the strip against the quarter-circle height derivative. This
	// reference uses the actual offset section, not the filletband formulas.
	const steps = 16384
	removed := 0.0
	for i := range steps {
		phi := (float64(i) + 0.5) * math.Pi / (2 * steps)
		offset := 1 - math.Cos(phi)
		a := 5 + offset
		bite := math.Pi*a*a/2 - offset*math.Sqrt(a*a-offset*offset) - a*a*math.Asin(offset/a)
		inset := (40-2*offset)*(20-2*offset) - bite
		strip := 800 - 25*math.Pi/2 - inset
		removed += strip * math.Cos(phi) * math.Pi / (2 * steps)
	}
	wantVolume := 10*(800-25*math.Pi/2) - removed
	require.LessOrEqual(t, math.Abs(out.volume.Value.Base()-wantVolume), out.volume.Bound.Base()+1e-5)
	require.Less(t, out.volume.Bound.Base(), 0.01*wantVolume)
	box, err := out.Bounds()
	require.NoError(t, err)
	require.LessOrEqual(t, box.Min.X-box.Bound.Base(), 0.0)
	require.GreaterOrEqual(t, box.Max.X+box.Bound.Base(), 40.0)
	require.LessOrEqual(t, box.Min.Y-box.Bound.Base(), 0.0)
	require.GreaterOrEqual(t, box.Max.Y+box.Bound.Base(), 20.0)
	report, err := out.doc.Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, ValidityValid, reading.Validity.Outcome)
	require.NotEqual(t, Unsound, reading.Status)
	patches := requireFilletPatches(t, out, 5, 1)
	straightArea, curvedArea := 0.0, 0.0
	for i := range steps {
		phi := (float64(i) + 0.5) * math.Pi / (2 * steps)
		offset := 1 - math.Cos(phi)
		foot := math.Sqrt(25 + 10*offset)
		straightArea += (20 - offset - foot) * math.Pi / (2 * steps)
		angle := math.Atan2(offset, foot)
		curvedArea += (5 + offset) * (math.Pi - 2*angle) * math.Pi / (2 * steps)
	}
	straightCount, curvedCount := 0, 0
	for _, patch := range patches {
		switch surf := patch.Surface().(type) {
		case Cylinder:
			if math.Abs(surf.Axis.X) < 0.9 || math.Abs(surf.Origin.Y-20) > 2 {
				continue
			}
			require.LessOrEqual(t, math.Abs(patch.area-straightArea), patch.areaBound+1e-5)
			straightCount++
		case Torus:
			require.LessOrEqual(t, math.Abs(patch.area-curvedArea), patch.areaBound+1e-5)
			curvedCount++
		}
	}
	require.Equal(t, 2, straightCount)
	require.Equal(t, 1, curvedCount)
	var seams []*Edge
	for _, edge := range out.Edges() {
		if _, ok := edge.Curve().(FilletMiter3); ok {
			seams = append(seams, edge)
		}
	}
	require.Len(t, seams, 2)
	wantLength := 0.0
	for i := range steps {
		phi := (float64(i) + 0.5) * math.Pi / (2 * steps)
		offset := 1 - math.Cos(phi)
		wantLength += math.Sqrt(1+25*math.Sin(phi)*math.Sin(phi)/(25+10*offset)) * math.Pi / (2 * steps)
	}
	for _, edge := range seams {
		require.Greater(t, edge.length, 1.0)
		require.Greater(t, edge.lengthBound, 0.0)
		require.LessOrEqual(t, math.Abs(edge.length-wantLength), edge.lengthBound+1e-6)
		require.InDelta(t, 9, edge.end.position.Z, 1e-8)
		require.InDelta(t, 10, edge.start.position.Z, 1e-8)
	}
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())
	incidence := make([]uint8, len(mesh.vertices))
	for ti, tri := range mesh.triangles {
		var bit uint8
		switch mesh.source[ti].Surface().(type) {
		case Cylinder:
			bit = 1
		case Torus:
			bit = 2
		}
		if bit == 0 {
			continue
		}
		for _, vi := range tri {
			incidence[vi] |= bit
		}
	}
	checked := 0
	for vi, mask := range incidence {
		p := mesh.vertices[vi]
		if mask != 3 || p.Z <= 9+1e-8 || p.Z >= 10-1e-8 || p.Y >= 20-1e-8 {
			continue
		}
		offset := 20 - p.Y
		require.InDelta(t, math.Sqrt(25+10*offset), math.Abs(p.X-20), 1e-8)
		checked++
	}
	require.Positive(t, checked)
}

// TestBrepLoopFilletRefusals pins Table SF and the rows the fillet arm keeps
// (§6, §8). Each refusal names its row and leaves the receiver live and the
// document's body set unchanged: part of P2's mouth is SL1;
// P1's top loop with one vertical edge, which is one segment of its
// y = 0 wall's loop, is SL1; P8's top loop at r = 3
// at r = 4 is SX6 (ErrDegenerate), the corner arcs' offsets crossing; P2's mouth at
// r = 5 is SX7, the band reaching the pocket's floor; and a stacked receiver
// whose section carries a displacement is SB1.
func TestBrepLoopFilletRefusals(t *testing.T) {
	t.Parallel()
	pocket := func(t *testing.T) *Body {
		_, body := internalRouteEPocket(t)
		return body
	}
	displaced := func(t *testing.T) *Body {
		doc := New()
		plate := internalBoxBody(t, doc, -20, -20, 20, 20, 10)
		boss := internalCircleBody(t, doc, -0.1, 5, 0, Distance{D: units.Millimeters(25), Dir: Along})
		move, err := r3.Translation(r3.NewVec(0.1, 0, 0))
		require.NoError(t, err)
		boss, err = boss.Placed(t.Context(), move)
		require.NoError(t, err)
		union, err := Union(t.Context(), plate, boss)
		require.NoError(t, err)
		return union
	}
	topLoop := func(z float64) func(*testing.T, *Body) []*Edge {
		return func(t *testing.T, b *Body) []*Edge {
			return planarBodyFace(t, b, routeEZ, r3.NewVec(0, 0, z)).Loops()[0].Edges()
		}
	}
	mouth := func(t *testing.T, b *Body) []*Edge {
		return planarBodyFace(t, b, routeEZ, r3.NewVec(0, 0, 10)).Loops()[1].Edges()
	}
	for _, tc := range []struct {
		name string
		body func(*testing.T) *Body
		sel  func(*testing.T, *Body) []*Edge
		r    float64
		is   error
		want []string
	}{
		{"a contour that crosses", func(t *testing.T) *Body { return internalRoundedPlate(t) }, topLoop(20), 4, ErrDegenerate,
			[]string{"no regular cap contour"}},
		{"a band reaching the floor", pocket, mouth, 5, ErrUnsupported, []string{"modify-reach SX7", "the fillet band"}},
		{"a displaced receiver", displaced, topLoop(10), 1, ErrUnsupported, []string{"brep-modify SB1"}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			body := tc.body(t)
			before := body.doc.Bodies()
			_, err := body.Fillet(t.Context(), edgesQuery(tc.sel(t, body)), units.Millimeters(tc.r))
			require.ErrorIs(t, err, tc.is)
			for _, w := range tc.want {
				require.ErrorContains(t, err, w)
			}
			require.NoError(t, body.doc.requireLive(body))
			require.Equal(t, before, body.doc.Bodies())
		})
	}
}

func TestVertexBlendPocketMouthOpenChain(t *testing.T) {
	t.Parallel()
	doc, body := internalRouteEPocket(t)
	mouth := planarBodyFace(t, body, routeEZ, r3.NewVec(0, 0, 10)).Loops()[1].Edges()
	out, err := body.Fillet(t.Context(), edgesQuery(mouth[:3]), units.Millimeters(1.5))
	require.NoError(t, err)
	requireClosedTopology(t, out)
	var cylinders, tori int
	for _, f := range filletPatchesOf(out) {
		switch f.Surface().(type) {
		case Cylinder:
			cylinders++
		case Torus:
			tori++
		}
	}
	require.Equal(t, 3, cylinders)
	require.Equal(t, 2, tori)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, Sound, reading.Status)
}

// TestBrepLoopFilletConsumers runs Table DF's F-1 readers over P1's top-loop
// fillet. Verify is Sound; a translated copy re-attaches the band and carries
// the same volume, area and roles; under a rotation about z the box still
// holds every patch, read against the oblique extents; the through-all
// extent along (1, 0, 1) reaches the x = 40 patch's stationary point
// 56 + 2√2 past the record's own faces; and the mesh closes with its proof.
// Shown to fail with brepBoundsContext's band
// extents deleted (the rotated box then missed the patches' bulge) and with
// extentAlong's band arm deleted (the stop then read the record's faces
// alone, 58/√2).
func TestBrepLoopFilletConsumers(t *testing.T) {
	t.Parallel()
	doc, s1 := internalCrossDrilled(t)
	out, bp := filletLoopOfFace(t, s1, routeEZ, r3.NewVec(0, 0, 20), 0, 2)

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := rep.ForBody(out)
	require.NoError(t, err)
	require.Equal(t, Sound, br.Status)

	move, err := r3.Translation(r3.NewVec(5, -7, 3))
	require.NoError(t, err)
	placed, err := out.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	require.Equal(t, out.volume.Value.Base(), placed.volume.Value.Base())
	require.InDelta(t, out.area.Value.Base(), placed.area.Value.Base(), 1e-9)
	require.ElementsMatch(t, filletRoles(out), filletRoles(placed))

	lo, hi, bound, err := bp.extentAlong(r3.NewVec(1, 0, 1).Scale(1 / math.Sqrt2))
	require.NoError(t, err)
	want := (56 + 2*math.Sqrt2) / math.Sqrt2
	require.LessOrEqual(t, hi-bound, want)
	require.GreaterOrEqual(t, hi+bound, want)
	require.InDelta(t, 0, lo, 1e-9)

	turn, err := r3.Rotation(r3.NewVec(0, 1, 0), units.Degrees(45))
	require.NoError(t, err)
	turned, err := out.PlacedCopy(t.Context(), turn)
	require.NoError(t, err)
	// The x = 40 patch's farthest point along x after the turn is its
	// (1, 0, 1)/√2 stationary point.
	require.GreaterOrEqual(t, turned.bounds.Max.X+turned.bounds.Bound.Base(), want-1e-9)
	for _, f := range filletPatchesOf(turned) {
		for _, e := range f.Edges() {
			for _, v := range []*Vertex{e.Start(), e.End()} {
				p := v.position
				require.GreaterOrEqual(t, p.X, turned.bounds.Min.X-1e-9)
				require.LessOrEqual(t, p.X, turned.bounds.Max.X+1e-9)
			}
		}
	}

	// The readers of PR F-2 (tessellation, the mesh boolean, the surveys) have
	// their own tests in tessellate_brep_fillet_internal_test.go.
	m, err := tessellateContext(t.Context(), out, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	require.True(t, m.symDiffOK)
}
