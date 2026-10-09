package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// filletMeshFixture is one loop-fillet §8 body beside its closed-form volume.
type filletMeshFixture struct {
	name   string
	build  func(t *testing.T) *Body
	volume piPoly
}

func filletMeshFixtures() []filletMeshFixture {
	loop := func(body func(*testing.T) *Body, n, p r3.Vec, li int, r float64) func(*testing.T) *Body {
		return func(t *testing.T) *Body {
			out, _ := filletLoopOfFace(t, body(t), n, p, li, r)
			return out
		}
	}
	crossDrilled := func(t *testing.T) *Body { _, b := internalCrossDrilled(t); return b }
	pocket := func(t *testing.T) *Body { _, b := internalRouteEPocket(t); return b }
	return []filletMeshFixture{
		{"P1 top loop", loop(crossDrilled, routeEZ, r3.NewVec(0, 0, 20), 0, 2), pp(q(46720, 3), q(-76, 1), q(0, 1))},
		{"P1 hole rim", loop(crossDrilled, r3.NewVec(0, -1, 0), r3.Vec{}, 1, 1), pp(q(16000, 1), q(-563, 3), q(2, 1))},
		{"P2 mouth", loop(pocket, routeEZ, r3.NewVec(0, 0, 10), 1, 1.5), pp(q(14865, 1), q(225, 8), q(27, 16))},
		{"P2 top loop", loop(pocket, routeEZ, r3.NewVec(0, 0, 10), 0, 1.5), pp(q(29325, 2), q(333, 4), q(0, 1))},
		{"P2 floor loop", loop(pocket, routeEZ, r3.NewVec(0, 0, 5), 0, 1.5), pp(q(30225, 2), q(-27, 1), q(0, 1))},
		{"P3 root", loop(internalRoundBoss, routeEZ, r3.NewVec(0, 0, 10), 1, 1), pp(q(16000, 1), q(1160, 3), q(-3, 1))},
		{"P3 rim", loop(internalRoundBoss, routeEZ, r3.NewVec(0, 0, 25), 0, 1), pp(q(16000, 1), q(1100, 3), q(2, 1))},
		{"P4 top loop", loop(internalFourHolePlate, routeEZ, r3.NewVec(0, 0, 8), 0, 1), pp(q(57020, 3), q(-152, 1), q(0, 1))},
		{"P6c port mouth", loop(internalBlindPort, r3.NewVec(1, 0, 0), r3.NewVec(60, 0, 0), 1, 1.5), pp(q(69265, 1), q(225, 8), q(27, 16))},
		{"P7 top loop", loop(internalLBracket, routeEZ, r3.NewVec(0, 0, 30), 0, 1), pp(q(51385, 3), q(-419, 12), q(1, 8))},
		{"P7 hole rim", loop(internalLBracket, r3.NewVec(-1, 0, 0), r3.Vec{}, 1, 1), pp(q(17280, 1), q(-239, 3), q(2, 1))},
		{"P8 top loop", loop(internalRoundedPlate, routeEZ, r3.NewVec(0, 0, 20), 0, 2), pp(q(14896, 1), q(256, 3), q(2, 1))},
	}
}

// TestTessellateBrepFilletBands tessellates every loop-fillet §8 body at
// 0.05 mm (Table DF's DF4). Each mesh is closed over every face of the body,
// the pipe patches among them, and its exact volume lies within the published
// occupied-volume proof of the closed form.
func TestTessellateBrepFilletBands(t *testing.T) {
	t.Parallel()
	for _, fx := range filletMeshFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			t.Parallel()
			out := fx.build(t)
			m, err := tessellateContext(t.Context(), out, units.Millimeters(0.05), VerifyAll)
			require.NoError(t, err)
			requireMeshOfBody(t, out, m)
			require.True(t, m.symDiffOK)
			require.Positive(t, m.volSymDiff)
			lo, hi := fx.volume.enclosed()
			requireMeshCovers(t, exactMeshVolume(m), m.volSymDiff, lo, hi)

			// The mesh's total facet area lies within the published slack of
			// the body's area.
			var meshArea float64
			for _, tri := range m.triangles {
				a, b, c := m.vertices[tri[0]], m.vertices[tri[1]], m.vertices[tri[2]]
				meshArea += b.Sub(a).Cross(c.Sub(a)).Len() / 2
			}
			require.LessOrEqual(t, math.Abs(meshArea-out.area.Value.Base()), m.areaSlack+out.area.Bound.Base())
		})
	}
}

// filletSurfaceDistance is how far p lies from the surface a fillet patch's
// tag names.
func filletSurfaceDistance(t *testing.T, f *Face, p r3.Vec) float64 {
	t.Helper()
	switch s := f.Surface().(type) {
	case Cylinder:
		rel := p.Sub(s.Origin)
		return math.Abs(rel.Sub(s.Axis.Scale(rel.Dot(s.Axis))).Len() - s.Radius.Base())
	case Torus:
		rel := p.Sub(s.Center)
		z := rel.Dot(s.Axis)
		rho := rel.Sub(s.Axis.Scale(z)).Len()
		return math.Abs(math.Hypot(rho-s.Major.Base(), z) - s.Minor.Base())
	}
	t.Fatalf("a fillet patch is a %T", f.Surface())
	return 0
}

// TestTessellateBrepFilletPatchesWithinBound checks the displacement bound a
// mesh publishes for each pipe patch against the geometry: every vertex, edge
// midpoint and centroid of the patch's cells lies within it of the surface the
// patch's tag names. The quarter-circle chord sagitta is the largest part, so
// a bound built without it is exceeded at a cell's centroid. Shown to fail
// with the sagitta term of emitFillet's patch deviation zeroed (P2's mouth and
// P6c's port mouth, whose patches are the straight-walled ones, read red), and
// with the in-plane ring sagitta zeroed (the rim and root bosses). The cells'
// twist term cannot be made to fail on these bodies: a straight wall's cells
// are planar and a circular wall's twist is far below the sagitta beside it.
func TestTessellateBrepFilletPatchesWithinBound(t *testing.T) {
	t.Parallel()
	for _, fx := range filletMeshFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			t.Parallel()
			out := fx.build(t)
			m, err := tessellateContext(t.Context(), out, units.Millimeters(0.05), VerifyAll)
			require.NoError(t, err)
			patches := map[*Face]struct{}{}
			for _, f := range filletPatchesOf(out) {
				patches[f] = struct{}{}
			}
			for i, f := range m.source {
				if _, ok := patches[f]; !ok {
					continue
				}
				bound, ok := m.sourceBound(f)
				require.True(t, ok)
				tri := m.triangles[i]
				a, b, c := m.vertices[tri[0]], m.vertices[tri[1]], m.vertices[tri[2]]
				for _, p := range []r3.Vec{a, b, c, a.Add(b).Scale(0.5), b.Add(c).Scale(0.5), a.Add(c).Scale(0.5),
					a.Add(b).Add(c).Scale(1.0 / 3)} {
					require.LessOrEqual(t, filletSurfaceDistance(t, f, p), bound, "cell %d of %v", i, f.Origins())
				}
			}
		})
	}
}

// TestBrepFilletMotionCoversEnds pins the per-vertex motion array of the
// VerifyAll proof for a fillet band (brepBandMotion). The trapezoid pocket's
// top loop, filleted by 1, has cap-contour corners no float holds, so its top
// face carries a contour displacement F.delta > 0. An interior ring vertex is
// interpolated between a side vertex and a cap vertex, so it moves at least as
// far as the larger of the two plus its own interpolation error. Shown to fail
// with the interior ring arm of brepBandMotion deleted (each interior vertex
// then kept its store, below F.delta) and with its interpolation term deleted
// (the vertices then read exactly their ends' motion).
func TestBrepFilletMotionCoversEnds(t *testing.T) {
	t.Parallel()
	out, _ := filletLoopOfFace(t, internalTrapezoidPocket(t), routeEZ, r3.NewVec(0, 0, 10), 0, 1)
	bp := out.payload.(brepPayload)
	delta := bp.faces[bp.loopBands[0].face].delta
	require.Positive(t, delta)

	topo, err := brepTopologyContext(t.Context(), bp)
	require.NoError(t, err)
	bands, _, err := brepChordBands(t.Context(), bp, topo, 0.05)
	require.NoError(t, err)
	n := 0
	add := func([3]float64, proofbound.WalkEndBound) int { n++; return n - 1 }
	e := topo.embeds[bands[0].band.face]
	bands[0].place(e, add)
	require.NoError(t, bands[0].placeRings(e, add))
	store, round := make([]float64, n), make([]float64, n)
	motion, err := brepBandMotion(t.Context(), bands, store, round)
	require.NoError(t, err)
	fr := bands[0].fillet
	require.Greater(t, fr.n, 2)
	for k := 1; k < fr.n; k++ {
		for c, vi := range fr.ringV[k] {
			ends := math.Max(motion[fr.ringV[0][c]], motion[fr.ringV[fr.n][c]])
			require.GreaterOrEqual(t, motion[vi], delta, "ring %d sample %d", k, c)
			require.Greater(t, motion[vi], ends, "ring %d sample %d", k, c)
		}
	}
}

// TestBrepFilletBooleanOperand pins Table DF's DF5. P1's top-loop fillet cut by
// a box over its x = 40 edge builds through the mesh path: the box removes the
// body in x ≥ 35, z ≥ 15, which holds 300 mm³ below the strip and
// ∫(5 − t)(20 − 2t) dh = 320/3 + 22π of it above, so 45500/3 − 98π remains.
// P2's mouth fillet, whose reflex corners the chamfer's admission refuses,
// is an operand too: a box over the plate's untouched x ≤ 5 end removes
// 2000 mm³ and leaves the closed form less 2000. Shown to fail with the
// fillet arm of brepBandsOccupiedVolumeAdmission deleted (the mouth's cut
// then refused for its reflex corner).
func TestBrepFilletBooleanOperand(t *testing.T) {
	t.Parallel()
	t.Run("P1 corner", func(t *testing.T) {
		t.Parallel()
		doc, s1 := internalCrossDrilled(t)
		out, _ := filletLoopOfFace(t, s1, routeEZ, r3.NewVec(0, 0, 20), 0, 2)
		tool := internalOffsetBox(t, doc, 35, -5, 45, 25, 15, Distance{D: units.Millimeters(10), Dir: Along})
		got, err := Cut(t.Context(), out, tool)
		require.NoError(t, err)
		lo, hi := pp(q(45500, 3), q(-98, 1), q(0, 1)).enclosed()
		requireCoversInterval(t, got.volume, lo, hi)
	})
	t.Run("P2 mouth", func(t *testing.T) {
		t.Parallel()
		_, pocket := internalRouteEPocket(t)
		out, _ := filletLoopOfFace(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 1, 1.5)
		tool := internalOffsetBox(t, out.doc, -5, -5, 5, 45, -5, Distance{D: units.Millimeters(20), Dir: Along})
		got, err := Cut(t.Context(), out, tool)
		require.NoError(t, err)
		lo, hi := pp(q(12865, 1), q(225, 8), q(27, 16)).enclosed()
		requireCoversInterval(t, got.volume, lo, hi)
	})
}

// TestTessellateBrepFilletPlaced pins Table DF's DF12: a translated, a rotated,
// a mirrored and a pattern copy of a fillet-banded body re-attach their bands
// (the re-evaluation re-derives them), mesh closed over every face with the
// same occupied-volume proof.
func TestTessellateBrepFilletPlaced(t *testing.T) {
	t.Parallel()
	_, pocket := internalRouteEPocket(t)
	out, _ := filletLoopOfFace(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 1, 1.5)
	lo, hi := pp(q(14865, 1), q(225, 8), q(27, 16)).enclosed()

	move, err := r3.Translation(r3.NewVec(5, -7, 3))
	require.NoError(t, err)
	turn, err := r3.Rotation(r3.NewVec(1, 1, 0), units.Degrees(30))
	require.NoError(t, err)
	plane, err := r3.NewFrame(r3.NewVec(0, 0, -3), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	moved, err := out.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	turned, err := out.PlacedCopy(t.Context(), turn)
	require.NoError(t, err)
	mirrored, err := out.MirroredCopy(t.Context(), MirrorFrame{Frame: plane})
	require.NoError(t, err)
	patterned, err := out.PatternCopies(t.Context(), LinearPattern{Dir: r3.NewVec(1, 0, 0), Step: units.Millimeters(50), Count: 2})
	require.NoError(t, err)
	require.Len(t, patterned, 1)
	for name, body := range map[string]*Body{"translated": moved, "turned": turned, "mirrored": mirrored, "pattern": patterned[0]} {
		t.Run(name, func(t *testing.T) {
			require.Len(t, filletPatchesOf(body), 8)
			m, err := tessellateContext(t.Context(), body, units.Millimeters(0.05), VerifyAll)
			require.NoError(t, err)
			requireMeshOfBody(t, body, m)
			require.True(t, m.symDiffOK)
			requireMeshCovers(t, exactMeshVolume(m), m.volSymDiff, lo, hi)
		})
	}
}

// TestBrepFilletSurveys pins Table DF's DF7 and DF8. P3's root fillet faces up
// and outward, so a pull along −z has it opposing the pull and lists it, and a
// pull along +z clears the body. P2's mouth lists all eight of its patches
// under −z. P2's floor loop is a fill whose tube radius 1.5 is the
// concave-radius survey's reading exactly; P2's mouth, which removes material,
// and P1's top loop add no concave radius. Shown to fail with the staged arms
// restored (the surveys then reported no coverage and no reading).
func TestBrepFilletSurveys(t *testing.T) {
	t.Parallel()
	root, _ := filletLoopOfFace(t, internalRoundBoss(t), routeEZ, r3.NewVec(0, 0, 10), 1, 1)
	for _, tc := range []struct {
		pull  r3.Vec
		lists int
	}{{r3.NewVec(0, 0, 1), 0}, {r3.NewVec(0, 0, -1), 1}} {
		rep, err := root.doc.Verify(t.Context(), WithPullDirection(tc.pull))
		require.NoError(t, err)
		br, err := rep.ForBody(root)
		require.NoError(t, err)
		require.Equal(t, CoverageComplete, br.Undercut.Coverage, "pull %v", tc.pull)
		require.Len(t, br.Undercut.Faces, tc.lists, "pull %v", tc.pull)
		if tc.lists > 0 {
			require.Equal(t, filletPatchesOf(root), br.Undercut.Faces)
		}
	}
	_, pocket := internalRouteEPocket(t)
	mouth, _ := filletLoopOfFace(t, pocket, routeEZ, r3.NewVec(0, 0, 10), 1, 1.5)
	rep, err := mouth.doc.Verify(t.Context(), WithPullDirection(r3.NewVec(0, 0, -1)))
	require.NoError(t, err)
	br, err := rep.ForBody(mouth)
	require.NoError(t, err)
	require.Equal(t, CoverageComplete, br.Undercut.Coverage)
	listed := 0
	for _, f := range br.Undercut.Faces {
		if _, ok := f.Surface().(Cylinder); ok || isTorus(f) {
			listed++
		}
	}
	require.Equal(t, 8, listed)

	_, pocket = internalRouteEPocket(t)
	floor, _ := filletLoopOfFace(t, pocket, routeEZ, r3.NewVec(0, 0, 5), 0, 1.5)
	rep, err = floor.doc.Verify(t.Context(), WithConcaveRadius())
	require.NoError(t, err)
	br, err = rep.ForBody(floor)
	require.NoError(t, err)
	require.Equal(t, ScalarMeasured, br.ConcaveRadius.Outcome)
	require.NotNil(t, br.ConcaveRadius.Minimum)
	require.InDelta(t, 1.5, br.ConcaveRadius.Minimum.Value.Base(), 1e-12)

	rep, err = mouth.doc.Verify(t.Context(), WithConcaveRadius())
	require.NoError(t, err)
	br, err = rep.ForBody(mouth)
	require.NoError(t, err)
	require.Equal(t, ScalarAbsent, br.ConcaveRadius.Outcome)

	// P1's drilled holes are Ø6: its only concave radius is 3, and the top
	// loop's convex fillet adds none.
	_, s1 := internalCrossDrilled(t)
	top, _ := filletLoopOfFace(t, s1, routeEZ, r3.NewVec(0, 0, 20), 0, 2)
	rep, err = top.doc.Verify(t.Context(), WithConcaveRadius())
	require.NoError(t, err)
	br, err = rep.ForBody(top)
	require.NoError(t, err)
	require.Equal(t, ScalarMeasured, br.ConcaveRadius.Outcome)
	require.InDelta(t, 3, br.ConcaveRadius.Minimum.Value.Base(), 1e-12)
}

func isTorus(f *Face) bool {
	_, ok := f.Surface().(Torus)
	return ok
}

// TestBrepFilletUndercutAgainstSamples checks DF7's verdict for each patch
// against the patch's own normals: a patch the survey clears has no sampled
// normal strictly between the pull's perpendicular and its antiparallel, and
// a patch it lists has one. The normals are Face.NormalAt at the mesh's cell
// vertices, which lie on the surface (a chord midpoint lies off it, where the
// normal near a patch's tangent plane can flip). Shown to fail with CornerRange's window
// reversed (the horn tori then cleared pulls they oppose) and with
// PatchPullVerdict's A taken without the face's outward sign.
func TestBrepFilletUndercutAgainstSamples(t *testing.T) {
	t.Parallel()
	pulls := []r3.Vec{{X: 0, Y: 0, Z: -1}, {X: 0, Y: 0, Z: 1}, {X: 1, Y: 0, Z: 0}, {X: -1, Y: 0, Z: 0}, {X: 0, Y: 1, Z: 0},
		{X: 1, Y: 1, Z: 1}, {X: 1, Y: 0, Z: -1}, {X: 0.3, Y: -0.5, Z: -0.8}, {X: -1, Y: 1, Z: 0.2}}
	for _, fx := range filletMeshFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			t.Parallel()
			out := fx.build(t)
			m, err := tessellateContext(t.Context(), out, units.Millimeters(0.05), VerifyAll)
			require.NoError(t, err)
			patches := map[*Face]struct{}{}
			for _, f := range filletPatchesOf(out) {
				patches[f] = struct{}{}
			}
			for _, pull := range pulls {
				p, _ := pull.Normalize()
				rep, err := out.doc.Verify(t.Context(), WithPullDirection(pull))
				require.NoError(t, err)
				br, err := rep.ForBody(out)
				require.NoError(t, err)
				listed := map[*Face]struct{}{}
				for _, f := range br.Undercut.Faces {
					listed[f] = struct{}{}
				}
				opposes := map[*Face]bool{}
				for i, f := range m.source {
					if _, ok := patches[f]; !ok {
						continue
					}
					tri := m.triangles[i]
					a, b, c := m.vertices[tri[0]], m.vertices[tri[1]], m.vertices[tri[2]]
					for _, pt := range []r3.Vec{a, b, c} {
						n, err := f.NormalAt(pt)
						if err != nil {
							continue
						}
						if d := n.Value.Dot(p); d < -1e-6 && d > -1+1e-6 {
							opposes[f] = true
						}
					}
				}
				for f := range patches {
					_, isListed := listed[f]
					if opposes[f] {
						// A sampled normal opposes: the survey must not have
						// cleared the patch (listed or undecided).
						if !isListed {
							require.NotEqual(t, CoverageComplete, br.Undercut.Coverage, "pull %v patch %v opposes yet reads clear", pull, f.Origins())
						}
						continue
					}
					// No sampled normal opposes, so the patch is not listed.
					require.False(t, isListed, "pull %v lists patch %v with no sampled opposing normal", pull, f.Origins())
				}
			}
		})
	}
}
