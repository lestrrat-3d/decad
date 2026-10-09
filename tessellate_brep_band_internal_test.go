package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file pins modify-general Table DG's route L consumers: the brep
// tessellator's reading of the bands (DG3), the mesh boolean's admission of a
// banded operand (DG4), the undercut and concave-radius surveys' reading of
// the patches (DG7, DG8) and placement (DG12). Every fixture is a §9 body.

// exactMeshVolume is the mesh's signed volume in exact rational arithmetic
// over its held vertices: the divergence sum of the triple products.
func exactMeshVolume(m *Mesh) *big.Rat {
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	mul := func(x, y, z float64) *big.Rat { return new(big.Rat).Mul(new(big.Rat).Mul(rat(x), rat(y)), rat(z)) }
	sum := new(big.Rat)
	for _, tri := range m.triangles {
		a, b, c := m.vertices[tri[0]], m.vertices[tri[1]], m.vertices[tri[2]]
		// a · (b × c)
		for _, term := range []*big.Rat{
			mul(a.X, b.Y, c.Z), mul(a.Y, b.Z, c.X), mul(a.Z, b.X, c.Y),
		} {
			sum.Add(sum, term)
		}
		for _, term := range []*big.Rat{
			mul(a.X, b.Z, c.Y), mul(a.Y, b.X, c.Z), mul(a.Z, b.Y, c.X),
		} {
			sum.Sub(sum, term)
		}
	}
	return sum.Quo(sum, big.NewRat(6, 1))
}

// requireMeshCovers asserts |V − c| <= within for every c in [lo, hi]: the
// proven symmetric difference of the mesh and the body bounds the mesh's
// volume from the body's closed form.
func requireMeshCovers(t *testing.T, v *big.Rat, within float64, lo, hi *big.Rat) {
	t.Helper()
	s := new(big.Rat).SetFloat64(within)
	require.LessOrEqual(t, new(big.Rat).Sub(v, s).Cmp(lo), 0, "mesh volume %s ± %g misses %s", v.FloatString(9), within, lo.FloatString(9))
	require.GreaterOrEqual(t, new(big.Rat).Add(v, s).Cmp(hi), 0, "mesh volume %s ± %g misses %s", v.FloatString(9), within, hi.FloatString(9))
}

// requireMeshOfBody asserts the mesh is closed and meshes every face of the
// body, the band patches among them.
func requireMeshOfBody(t *testing.T, body *Body, m *Mesh) {
	t.Helper()
	require.NoError(t, tessellation.RequireClosedMesh(m.triangles))
	seen := map[*Face]struct{}{}
	for _, f := range m.source {
		seen[f] = struct{}{}
	}
	for _, f := range body.Faces() {
		require.Contains(t, seen, f, "face %v is in the mesh", f.Origins())
	}
}

// bandFixture is one §9 route L body beside its closed-form volume a + bπ.
// proved is whether DG4's admission proves its occupied volume: false for a
// band holding an apex cone at a reflex corner.
type bandFixture struct {
	name   string
	build  func(t *testing.T) *Body
	a, b   *big.Rat
	proved bool
}

func bandFixtures() []bandFixture {
	return []bandFixture{
		{"P2 mouth", func(t *testing.T) *Body {
			_, pocket := internalRouteEPocket(t)
			out, _ := chamferLoopOf(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 1, 1.5)
			return out
		}, big.NewRat(29865, 2), big.NewRat(-9, 8), false},
		{"P2 top loop", func(t *testing.T) *Body {
			_, pocket := internalRouteEPocket(t)
			out, _ := chamferLoopOf(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 0, 1.5)
			return out
		}, big.NewRat(29649, 2), new(big.Rat), true},
		{"P3 rim", func(t *testing.T) *Body {
			out, _ := chamferLoopOf(t, internalRoundBoss(t), routeEZ, r3.NewVec(0, 0, 25), 0, 1)
			return out
		}, big.NewRat(16000, 1), big.NewRat(1111, 3), true},
		{"P3 root", func(t *testing.T) *Body {
			out, _ := chamferLoopOf(t, internalRoundBoss(t), routeEZ, r3.NewVec(0, 0, 10), 1, 1)
			return out
		}, big.NewRat(16000, 1), big.NewRat(1141, 3), true},
		{"P6c port mouth", func(t *testing.T) *Body {
			port := internalBlindPort(t)
			out, _ := chamferLoopOf(t, port, r3.NewVec(1, 0, 0), r3.NewVec(60, 0, 0), 1, 1.5)
			return out
		}, big.NewRat(138665, 2), big.NewRat(-9, 8), false},
		{"P7 top loop", func(t *testing.T) *Body {
			out, _ := chamferLoopOf(t, internalLBracket(t), routeEZ, r3.NewVec(0, 0, 30), 0, 1)
			return out
		}, big.NewRat(51605, 3), big.NewRat(-865, 12), false},
		{"P8 hole rim", func(t *testing.T) *Body {
			out, _ := chamferLoopOf(t, internalRoundedPlate(t), r3.NewVec(0, -1, 0), r3.Vec{}, 1, 1)
			return out
		}, big.NewRat(15280, 1), big.NewRat(-10, 3), true},
		{"P8 top loop", func(t *testing.T) *Body {
			out, _ := chamferLoopOf(t, internalRoundedPlate(t), routeEZ, r3.NewVec(0, 0, 20), 0, 1)
			return out
		}, big.NewRat(15232, 1), big.NewRat(-8, 3), true},
	}
}

// TestTessellateBrepBands tessellates every §9 route L body at 0.05 mm
// (modify-general Table DG's DG3). Each mesh is closed over every face of the
// body, patches included, and its exact volume lies within the occupied-volume
// proof of the closed form where the band is admitted (DG4's rule: whole turns,
// line-line miters, exact G1 joins) — P2's top loop, whose bands are planes,
// is exactly 15000 − 175.5. A band with an apex cone at a reflex corner is
// export-only: its mesh states no proof, and the mesh agrees with the closed
// form to within the chord times the cones' area. Verify reads each Sound.
// Shown to fail with tessellateBrep's band placement deleted (the patches then
// named vertices no strip reached and the mesh stayed open), with
// tessellateBrep's imposed wall samples deleted (a wall in another frame than
// its band, P3's root and P8's hole rim, then took samples of its own and the
// mesh stayed open), with a rising band's winding turn deleted (P3's root then
// meshed inside out) and with brepBandChordVolume's height zeroed (P3's rim and
// root then lay outside their proofs). The per-vertex motion array is pinned by
// TestBrepBandMotionCoversContourDisplacement: these fixtures' bound is far
// looser than the gap, so they stay covered with the store in its place.
func TestTessellateBrepBands(t *testing.T) {
	t.Parallel()
	const chord = 0.05
	for _, fx := range bandFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			t.Parallel()
			out := fx.build(t)
			rep, err := out.doc.Verify(t.Context())
			require.NoError(t, err)
			br, err := rep.ForBody(out)
			require.NoError(t, err)
			require.Equal(t, Sound, br.Status)

			m, err := tessellateContext(t.Context(), out, units.Millimeters(chord), VerifyAll)
			require.NoError(t, err)
			requireMeshOfBody(t, out, m)
			v := exactMeshVolume(m)
			lo, hi := piEnclosed(fx.a, fx.b)
			require.Equal(t, fx.proved, m.symDiffOK)
			if !fx.proved {
				// A chord polygon lies within its sagitta of the curved surface it
				// chords, so the mesh differs from the body by at most the chord
				// times the area of the body's curved faces.
				curved := 0.0
				for _, f := range out.Faces() {
					switch f.Surface().(type) {
					case Cone, Cylinder:
						area, err := f.Area()
						require.NoError(t, err)
						curved += area.Value.Base()
					}
				}
				require.Positive(t, curved)
				requireMeshCovers(t, v, chord*curved, lo, hi)
				return
			}
			requireMeshCovers(t, v, m.volSymDiff, lo, hi)
			if fx.b.Sign() == 0 {
				require.Zero(t, m.volSymDiff)
				require.Zero(t, v.Cmp(fx.a), "planes alone: %s", v.FloatString(9))
			}
		})
	}
}

// TestTessellateBrepBandsShareOneCount pins tessellation-reach §7's rule on
// P3's rim chamfer: the boss wall's rim at the side level z = 24 and the cone's
// cap circle at z = 25 are chorded at one count, so the cone's cells are
// n quads between matching stations and every triangle of the cone has two
// vertices on one circle. Shown to fail with the count taken from the wall's
// own SampleLoop alone (the rim then held fewer stations than the cap ring and
// the mesh stayed open).
func TestTessellateBrepBandsShareOneCount(t *testing.T) {
	t.Parallel()
	out, _ := chamferLoopOf(t, internalRoundBoss(t), routeEZ, r3.NewVec(0, 0, 25), 0, 1)
	m, err := tessellateContext(t.Context(), out, units.Millimeters(0.05), VerifyAll)
	require.NoError(t, err)
	requireMeshOfBody(t, out, m)
	onRing := func(z, radius float64) int {
		n := 0
		for _, v := range m.vertices {
			if v.Z == z && abs(v.Sub(r3.NewVec(20, 20, z)).Len()-radius) < 1e-9 {
				n++
			}
		}
		return n
	}
	side, capRing := onRing(24, 5), onRing(25, 4)
	require.Greater(t, side, 8)
	require.Equal(t, side, capRing, "one count on the cone's two circles")
	cone := requirePatchKinds(t, out, 0, 1)[0]
	cells := 0
	for i, f := range m.source {
		if f == cone {
			cells++
			a := m.triangles[i]
			zs := map[float64]int{}
			for _, vi := range a {
				zs[m.vertices[vi].Z]++
			}
			require.Len(t, zs, 2, "a cell spans the two circles")
		}
	}
	require.Equal(t, 2*side, cells, "two triangles per station")
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

// TestTessellateBrepBandsPlaced pins DG12: a translated, a mirrored and a
// pattern copy of a route L body re-attach their bands (the re-evaluation
// re-derives them) and tessellate to the closed mesh of the same volume.
func TestTessellateBrepBandsPlaced(t *testing.T) {
	t.Parallel()
	const chord = 0.05
	_, pocket := internalRouteEPocket(t)
	out, _ := chamferLoopOf(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 0, 1.5)
	base, err := tessellateContext(t.Context(), out, units.Millimeters(chord), VerifyAll)
	require.NoError(t, err)
	want := exactMeshVolume(base)

	move, err := r3.Translation(r3.NewVec(5, -7, 3))
	require.NoError(t, err)
	moved, err := out.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	plane, err := r3.NewFrame(r3.NewVec(0, 0, -3), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	mirrored, err := out.MirroredCopy(t.Context(), MirrorFrame{Frame: plane})
	require.NoError(t, err)
	for name, body := range map[string]*Body{"translated": moved, "mirrored": mirrored} {
		t.Run(name, func(t *testing.T) {
			require.Len(t, loopPatches(body), 4)
			m, err := tessellateContext(t.Context(), body, units.Millimeters(chord), VerifyAll)
			require.NoError(t, err)
			requireMeshOfBody(t, body, m)
			require.True(t, m.symDiffOK)
			require.Zero(t, exactMeshVolume(m).Cmp(want), "planes alone are exact: %s", exactMeshVolume(m).FloatString(9))
		})
	}

	boss := internalRoundBoss(t)
	rim, _ := chamferLoopOf(t, boss, routeEZ, r3.NewVec(0, 0, 25), 0, 1)
	copyOf, err := rim.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	m, err := tessellateContext(t.Context(), copyOf, units.Millimeters(chord), VerifyAll)
	require.NoError(t, err)
	requireMeshOfBody(t, copyOf, m)
	lo, hi := piEnclosed(big.NewRat(16000, 1), big.NewRat(1111, 3))
	require.True(t, m.symDiffOK)
	requireMeshCovers(t, exactMeshVolume(m), m.volSymDiff, lo, hi)
}

// TestBrepBandBooleanAdmission pins DG4: a route L body whose bands the
// cap-loop chamfer's rule admits is a mesh-boolean operand, and one holding an
// apex cone at a reflex corner is refused with the admission's own reason.
// P2's top loop (planes) cut by a box over its corner leaves
// 14824.5 − (250 − (25d − (5³ − (5 − d)³)/3)) = 14584.625. Shown to fail with
// requireVolumeProvingPayload's brep arm deleted (the refusal then read the
// mesh's generic "carries no proof" text).
func TestBrepBandBooleanAdmission(t *testing.T) {
	t.Parallel()
	_, pocket := internalRouteEPocket(t)
	top, _ := chamferLoopOf(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 0, 1.5)
	corner := internalOffsetBox(t, top.doc, 35, 35, 45, 45, -5, Distance{D: units.Millimeters(30), Dir: Along})
	got, err := Cut(t.Context(), top, corner)
	require.NoError(t, err)
	want := big.NewRat(14584625, 1000)
	requireCoversInterval(t, got.volume, want, want)

	_, pocket = internalRouteEPocket(t)
	mouth, _ := chamferLoopOf(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 1, 1.5)
	tool := internalOffsetBox(t, mouth.doc, 35, 35, 45, 45, -5, Distance{D: units.Millimeters(30), Dir: Along})
	_, err = Cut(t.Context(), mouth, tool)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "reflex corner")
	require.ErrorContains(t, err, "no boolean may compose it")
}

// TestBrepBandSurveys pins DG7 and DG8. P3's rim cone faces up and out, so a
// pull along −z has it opposing the pull and lists it, and a pull along +z
// clears the body. P2's top-loop chamfer, planes alone, decides the
// concave-radius survey with no concave feature; P2's mouth chamfer and P3's
// root chamfer hold Cone patches whose stamped normal departure is not exactly
// zero (capBlendMinRadius's rule), so the survey leaves them undecided. Shown to
// fail with the staged arms of runSurveys restored: the undercut survey then
// reported the payload staged (no coverage) and the radius survey unavailable
// on P2's top loop.
func TestBrepBandSurveys(t *testing.T) {
	t.Parallel()
	rim, _ := chamferLoopOf(t, internalRoundBoss(t), routeEZ, r3.NewVec(0, 0, 25), 0, 1)
	for _, tc := range []struct {
		pull  r3.Vec
		lists int
	}{{r3.NewVec(0, 0, 1), 0}, {r3.NewVec(0, 0, -1), 1}} {
		rep, err := rim.doc.Verify(t.Context(), WithPullDirection(tc.pull))
		require.NoError(t, err)
		br, err := rep.ForBody(rim)
		require.NoError(t, err)
		require.Equal(t, CoverageComplete, br.Undercut.Coverage, "pull %v", tc.pull)
		require.Len(t, br.Undercut.Faces, tc.lists, "pull %v", tc.pull)
		if tc.lists > 0 {
			require.Equal(t, loopPatches(rim), br.Undercut.Faces)
		}
	}

	radius := func(body *Body) ScalarOutcome {
		rep, err := body.doc.Verify(t.Context(), WithConcaveRadius())
		require.NoError(t, err)
		br, err := rep.ForBody(body)
		require.NoError(t, err)
		return br.ConcaveRadius.Outcome
	}
	_, pocket := internalRouteEPocket(t)
	top, _ := chamferLoopOf(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 0, 1.5)
	require.Equal(t, ScalarAbsent, radius(top))
	for _, body := range []*Body{
		func() *Body {
			_, p := internalRouteEPocket(t)
			out, _ := chamferLoopOf(t, p, routeEZ, r3.NewVec(0, 0, 10), 1, 1.5)
			return out
		}(),
		func() *Body {
			out, _ := chamferLoopOf(t, internalRoundBoss(t), routeEZ, r3.NewVec(0, 0, 10), 1, 1)
			return out
		}(),
	} {
		require.Equal(t, ScalarUndecided, radius(body))
	}
}

// TestBrepBandMotionCoversContourDisplacement pins the per-vertex motion array
// of the VerifyAll proof (brepBandMotion). The trapezoid pocket's top loop,
// chamfered by 1, has cap-contour corners no float holds, so its top face
// carries a contour displacement F.delta > 0 (modify-general §7). A cap
// contour vertex's store is its station's gap from the held offset circle plus
// its rounding, which is zero for a line-line miter foot; the foot's real
// distance from the ideal polyhedron's vertex is the contour displacement. So
// the store understates that motion by F.delta, and the motion array must
// carry it. Shown to fail with brepBandMotion returning the store: every cap
// vertex then reads zero, below F.delta.
func TestBrepBandMotionCoversContourDisplacement(t *testing.T) {
	t.Parallel()
	body := internalTrapezoidPocket(t)
	out, _ := chamferLoopOf(t, body, routeEZ, r3.NewVec(0, 0, 10), 0, 1)
	bp := out.payload.(brepPayload)
	delta := bp.faces[bp.loopBands[0].face].delta
	require.Positive(t, delta)

	topo, err := brepTopologyContext(t.Context(), bp)
	require.NoError(t, err)
	bands, _, err := brepChordBands(t.Context(), bp, topo, 0.05)
	require.NoError(t, err)
	n := 0
	bands[0].place(topo.embeds[bands[0].band.face], func([3]float64, proofbound.WalkEndBound) int { n++; return n - 1 })
	store, round := make([]float64, n), make([]float64, n)
	motion, err := brepBandMotion(t.Context(), bands, store, round)
	require.NoError(t, err)
	require.NotEmpty(t, bands[0].capV)
	for _, vi := range bands[0].capV {
		require.GreaterOrEqual(t, motion[vi], delta, "cap vertex %d", vi)
		require.Greater(t, motion[vi], store[vi])
	}
}
