package apitest_test

import (
	"math"
	"math/big"
	"sort"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/draft-design.md §11's PR 2 fixture set: a draft body's
// tessellation (Table DD row DD1) and the readings it opens — booleans (DD2),
// export (DD3, DD4), interference (DD10), mass properties (DD11) and Patterned
// (DD13). Closed forms reuse extrude_taper_test.go's 256-bit helpers, with
// d = h·tan α from the exact angle the stated taper denotes.

// draftMeshTolerances are the three chord tolerances every fixture meshes at.
var draftMeshTolerances = []float64{0.25, 0.05, 0.01}

// draftCase is one F fixture: how to build it and, for a curved wall, the
// true wall points a falsifier samples.
type draftCase struct {
	name  string
	build func(*testing.T, *decad.Document) *decad.Body
	// walls samples the true drafted surface at axial fraction s and azimuth
	// th; nil for an all-Plane body.
	walls []func(s, th float64) r3.Vec
	// nearRadius is the radius of each circular wall at the sketch plane, read
	// to measure the near ring's own sagitta.
	nearRadius []float64
}

// draftConeWall returns the point at axial fraction s and azimuth th of the cone
// between the circle of radius r0 about (cu, 0) at z = 0 and radius r1 at
// z = h.
func draftConeWall(cu, r0, r1, h float64) func(s, th float64) r3.Vec {
	return func(s, th float64) r3.Vec {
		rr := r0 + (r1-r0)*s
		return r3.NewVec(cu+rr*math.Cos(th), rr*math.Sin(th), h*s)
	}
}

func draftCases() []draftCase {
	square := func(deg float64, dir decad.Direction) func(*testing.T, *decad.Document) *decad.Body {
		return func(t *testing.T, doc *decad.Document) *decad.Body {
			s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
			return taperExtrude(t, doc, s, p, boxHeight, deg, dir)
		}
	}
	d2, _ := bfMul(bf(10), taperTan(10)).Float64()
	d7, _ := bfMul(bf(10), taperTan(5)).Float64()
	return []draftCase{
		{name: "F1 box", build: square(5, decad.Along)},
		{name: "F2 frustum", build: func(t *testing.T, doc *decad.Document) *decad.Body {
			s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawDisk(10))
			return taperExtrude(t, doc, s, p, 10, 10, decad.Along)
		}, walls: []func(s, th float64) r3.Vec{draftConeWall(0, 10, 10-d2, 10)}, nearRadius: []float64{10}},
		{name: "F3 slot", build: func(t *testing.T, doc *decad.Document) *decad.Body {
			s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSlot(0, 0))
			return taperExtrude(t, doc, s, p, 8, 3, decad.Along)
		}},
		{name: "F4 L-section", build: func(t *testing.T, doc *decad.Document) *decad.Body {
			s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawPolygon([2]float64{0, 0}, [2]float64{20, 0},
				[2]float64{20, 10}, [2]float64{10, 10}, [2]float64{10, 20}, [2]float64{0, 20}))
			return taperExtrude(t, doc, s, p, 10, 5, decad.Along)
		}},
		{name: "F5 flare", build: square(-5, decad.Along)},
		{name: "F6 against", build: square(5, decad.Against)},
		{name: "F7 ring", build: func(t *testing.T, doc *decad.Document) *decad.Body {
			s, p := taperSketch(t, r3.NewVec(0, 0, 0), 1, func(s *sketch.Sketch) {
				c := s.CreatePoint(0, 0)
				s.CreateCircle(c, 10)
				s.CreateCircle(c, 4)
				s.Fix(c)
			})
			return taperExtrude(t, doc, s, p, 10, 5, decad.Along)
		}, walls: []func(s, th float64) r3.Vec{draftConeWall(0, 10, 10-d7, 10), draftConeWall(0, 4, 4+d7, 10)},
			nearRadius: []float64{10, 4}},
	}
}

// nearRingSagitta is the largest gap between a circle of radius r about the
// z axis and the chords its mesh vertices at z = 0 and that radius cut: the
// widest angular step between consecutive vertices, read as r(1 − cos(Δ/2)).
func nearRingSagitta(mesh *decad.Mesh, r float64) float64 {
	var angles []float64
	for _, v := range mesh.Vertices() {
		if v.Z != 0 || math.Abs(math.Hypot(v.X, v.Y)-r) > 1e-9 {
			continue
		}
		angles = append(angles, math.Atan2(v.Y, v.X))
	}
	if len(angles) < 3 {
		return math.Inf(1)
	}
	sort.Float64s(angles)
	widest := angles[0] + 2*math.Pi - angles[len(angles)-1]
	for i := 1; i < len(angles); i++ {
		widest = math.Max(widest, angles[i]-angles[i-1])
	}
	return r * (1 - math.Cos(widest/2))
}

// TestDraftMesh is §11's PR 2 tessellation fixture: F1–F7 at three chord
// tolerances, each mesh closed and consistently wound, its boundary and
// volume proofs published, every facet attributed to a face of the body, and
// Bound covering the chording it took: at least each circular wall's near-ring
// sagitta, and no sample of a true drafted cone farther from the mesh than
// Bound.
func TestDraftMesh(t *testing.T) {
	t.Parallel()
	for _, tc := range draftCases() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			b := tc.build(t, decad.New())
			faces := map[*decad.Face]struct{}{}
			for _, f := range b.Faces() {
				faces[f] = struct{}{}
			}
			for _, tol := range draftMeshTolerances {
				mesh, err := b.Tessellate(t.Context(), units.Millimeters(tol))
				require.NoError(t, err, "tol %g", tol)
				requireWatertight(t, mesh)
				require.True(t, mesh.BoundaryVerified(), "tol %g", tol)
				require.True(t, mesh.VolumeVerified(), "tol %g: a draft band of miters and exact G1 joins proves its volume", tol)
				require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
				for _, f := range mesh.SourceFaces() {
					_, ok := faces[f]
					require.True(t, ok, "tol %g: a facet names a face the body does not hold", tol)
				}
				bound := mesh.Bound().Mag()
				require.Positive(t, bound, "tol %g: the far section is never exact, so no draft mesh is", tol)
				for _, r := range tc.nearRadius {
					require.LessOrEqual(t, nearRingSagitta(mesh, r), bound, "tol %g: Bound below the ring's own sagitta", tol)
				}
				for _, wall := range tc.walls {
					for i := range 17 {
						for j := range 61 {
							p := wall(float64(i)/16, 2*math.Pi*float64(j)/61)
							require.LessOrEqual(t, distanceToMesh(mesh, p), bound,
								"tol %g: true wall sample %v is farther from the mesh than Bound", tol, p)
						}
					}
				}
			}
		})
	}
}

// TestDraftMeshExports pins DD3 and DD4: STL, OBJ and 3MF write F1 and F2,
// STL and OBJ byte-identically on a second run, and STEP writes both through
// the faceted writer. F1's drafted walls are Planes and F2's wall is a full
// Cone between two circles, which the analytic face admission takes, but each
// drafted wall's tag departs from the wall by a stated bound (its NormalBound
// is not zero), and the analytic writer refuses a body with such a face
// (agent-docs/step-export-design.md). Shown to fail with supportsAnalyticSTEP's
// tag gate deleted (both bodies were written analytic).
func TestDraftMeshExports(t *testing.T) {
	t.Parallel()
	cases := draftCases()
	for _, tc := range []struct {
		fixture draftCase
		step    string
	}{{cases[0], "faceted decad solid"}, {cases[1], "faceted decad solid"}} {
		t.Run(tc.fixture.name, func(t *testing.T) {
			t.Parallel()
			b := tc.fixture.build(t, decad.New())
			tol := units.Millimeters(0.1)
			var stlA, stlB, objA, objB, mf strings.Builder
			require.NoError(t, export.STL(t.Context(), &stlA, b, tol))
			require.NoError(t, export.STL(t.Context(), &stlB, b, tol))
			require.NoError(t, export.OBJ(t.Context(), &objA, b, tol))
			require.NoError(t, export.OBJ(t.Context(), &objB, b, tol))
			require.NoError(t, export.ThreeMF(t.Context(), &mf, b, tol))
			require.NotEmpty(t, stlA.String())
			require.NotEmpty(t, objA.String())
			require.NotEmpty(t, mf.String())
			require.Equal(t, stlA.String(), stlB.String())
			require.Equal(t, objA.String(), objB.String())

			var stp strings.Builder
			require.NoError(t, export.STEP(t.Context(), &stp, b, tol,
				export.WithSTEPName("draft"), export.WithSTEPAuthor("apitest"), export.WithSTEPOrganization("decad")))
			require.Contains(t, stp.String(), tc.step)
		})
	}
}

// requireVolumeCovers requires a boolean result's volume within its own
// bound of the closed form want, with the bound below ceiling·want.
func requireVolumeCovers(t *testing.T, what string, b *decad.Body, want *big.Float, ceiling float64) {
	t.Helper()
	vol, err := b.Volume()
	require.NoError(t, err)
	w, _ := want.Float64()
	gap, _ := new(big.Float).Abs(bfSub(bf(volumeMM(t, vol)), want)).Float64()
	bound := boundMM3(t, vol)
	require.LessOrEqualf(t, gap, bound, "%s: volume %v sits %g from its closed form %v, outside its bound %g", what, volumeMM(t, vol), gap, w, bound)
	require.LessOrEqualf(t, bound, ceiling*w, "%s: bound %g above %g·%v", what, bound, ceiling, w)
}

// TestDraftBooleanWithStraightPrism pins DD2: F1 composed on the mesh path
// with a 4 mm square post through it from z = −5 to z = 15. The post's middle
// 10 mm lies inside the box at every level (its far cap's half side is
// 10 − d > 2), so Intersect is that 160 mm³, Cut removes it, and Union adds
// the post's 160 mm³ outside the box.
func TestDraftBooleanWithStraightPrism(t *testing.T) {
	t.Parallel()
	d := bfMul(bf(boxHeight), taperTan(5))
	box := boxVolume(d)
	for _, tc := range []struct {
		name string
		want *big.Float
		run  func(a, b *decad.Body) (*decad.Body, error)
	}{
		{"union", bfAdd(box, bf(160)), func(a, b *decad.Body) (*decad.Body, error) { return decad.Union(t.Context(), a, b) }},
		{"cut", bfSub(box, bf(160)), func(a, b *decad.Body) (*decad.Body, error) { return decad.Cut(t.Context(), a, b) }},
		{"intersect", bf(160), func(a, b *decad.Body) (*decad.Body, error) { return decad.Intersect(t.Context(), a, b) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			drafted := draftCases()[0].build(t, doc)
			post := boxBodyAtZ(t, doc, -2, -2, 2, 2, -5, 20)
			got, err := tc.run(drafted, post)
			require.NoError(t, err)
			requireVolumeCovers(t, tc.name, got, tc.want, 1e-9)
			requireBodyWatertight(t, got)
		})
	}
}

// draftShift is how far the second of two overlapping F1 boxes is moved. The
// lift keeps the two boxes' caps out of each other's planes, which a mesh
// boolean cannot classify.
var draftShift = r3.NewVec(15, 5, 2)

// draftOverlap is the volume two F1 boxes share when the second is moved by
// draftShift. Write k = tan α = d/h and u = 2z − 2. Above z = 2 the two
// sections are squares of side 20 − 2kz and 20 − 2k(z − 2), so the shared
// rectangle is (5 − ku) by (15 − ku), and the volume is
// ½∫ (75 − 20ku + k²u²) du over u from 2 to 18: ½[G(18) − G(2)] with
// G(u) = 75u − 10ku² + k²u³/3.
func draftOverlap(d *big.Float) *big.Float {
	k := bfQuo(d, bf(boxHeight))
	g := func(u float64) *big.Float {
		U := bf(u)
		return bfAdd(bfSub(bfMul(bf(75), U), bfMul(bf(10), k, U, U)), bfQuo(bfMul(k, k, U, U, U), bf(3)))
	}
	return bfQuo(bfSub(g(18), g(2)), bf(2))
}

// TestDraftOverlapIsMeasured pins DD10: Verify measures the overlap of two F1
// boxes, the second a copy moved by draftShift, and publishes it as an
// Interference row whose bound covers the closed form.
func TestDraftOverlapIsMeasured(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := draftCases()[0].build(t, doc)
	shift, err := r3.Translation(draftShift)
	require.NoError(t, err)
	b, err := decad.Copy(t.Context(), a, decad.WithCopyPlacement(shift))
	require.NoError(t, err)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.False(t, hasDiagnostic(report, decad.DiagUnsupportedPairPayload))
	require.Len(t, report.Interferences, 1)
	row := report.Interferences[0]
	require.ElementsMatch(t, []*decad.Body{a, b}, []*decad.Body{row.A, row.B})
	want := draftOverlap(bfMul(bf(boxHeight), taperTan(5)))
	w, _ := want.Float64()
	require.LessOrEqual(t, math.Abs(volumeMM(t, row.Volume)-w), boundMM3(t, row.Volume))
	require.Less(t, boundMM3(t, row.Volume), 1e-9*w)
}

// TestDraftPatterned pins DD13: two F1 instances along draftShift at a step
// of its length, the second moved by draftShift up to the step's rounding,
// combine by Union into one body whose volume is twice the box less their
// overlap.
func TestDraftPatterned(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := draftCases()[0].build(t, doc)
	got, err := decad.Patterned(t.Context(), a, decad.LinearPattern{Dir: draftShift, Step: units.Millimeters(draftShift.Len()), Count: 2})
	require.NoError(t, err)
	d := bfMul(bf(boxHeight), taperTan(5))
	requireVolumeCovers(t, "patterned union", got, bfSub(bfMul(bf(2), boxVolume(d)), draftOverlap(d)), 1e-9)
	requireBodyWatertight(t, got)
	require.Equal(t, []*decad.Body{got}, doc.Bodies())
}

// TestDraftMassProperties pins DD11: F1's mass properties integrate its
// VerifyAll mesh. With s(z) = a − 2z·tan α the side at height z, the mass is
// ρV, the centre sits on the axis at §11's axial centroid, and the moment
// about that axis is ρ∫ s⁴/6 dz = ρh(a⁵ − (a − 2d)⁵)/(60d).
func TestDraftMassProperties(t *testing.T) {
	t.Parallel()
	const rho = 1.0 / 1024
	b := draftCases()[0].build(t, decad.New())
	got, err := b.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(rho))
	require.NoError(t, err)

	d := bfMul(bf(boxHeight), taperTan(5))
	A, H := bf(boxSide), bf(boxHeight)
	vol := boxVolume(d)
	moment := bfMul(H, H, bfAdd(bfSub(bfQuo(bfMul(A, A), bf(2)), bfQuo(bfMul(bf(4), A, d), bf(3))), bfMul(d, d)))
	top := bfSub(A, bfMul(bf(2), d))
	pow5 := func(x *big.Float) *big.Float { return bfMul(x, x, x, x, x) }
	zz := bfQuo(bfMul(bf(rho), H, bfSub(pow5(A), pow5(top))), bfMul(bf(60), d))

	requireMassCovers := func(what string, m decad.Measurement, want *big.Float) {
		t.Helper()
		gap, _ := new(big.Float).Abs(bfSub(bf(m.Value.Base()), want)).Float64()
		require.LessOrEqualf(t, gap, m.Bound.Base(), "%s: %v sits %g from its closed form, outside its bound %g", what, m.Value.Base(), gap, m.Bound.Base())
	}
	requireMassCovers("mass", got.Mass, bfMul(bf(rho), vol))
	requireMassCovers("inertia zz", got.Inertia.ZZ, zz)
	w, _ := zz.Float64()
	require.Less(t, got.Inertia.ZZ.Bound.Base(), 1e-3*w)
	c := bfQuo(moment, vol)
	for i, want := range []*big.Float{bf(0), bf(0), c} {
		value := [3]float64{got.Center.Value.X, got.Center.Value.Y, got.Center.Value.Z}[i]
		gap, _ := new(big.Float).Abs(bfSub(bf(value), want)).Float64()
		require.LessOrEqual(t, gap, got.Center.Bound.Base(), "center axis %d", i)
	}
	require.Less(t, got.Center.Bound.Base(), 1e-3)
}

// TestDraftMeshPlacedAndMirrored meshes F2 and F3 under a rotation placement
// and as a mirrored copy (Table DD row DD12): every mesh stays closed and
// outward-wound and proves its volume, and the volume it encloses sits within
// the 0.05 mm chording's deficit, under 1%, of the body's.
func TestDraftMeshPlacedAndMirrored(t *testing.T) {
	t.Parallel()
	cases := draftCases()
	for _, fx := range []draftCase{cases[1], cases[2]} {
		t.Run(fx.name, func(t *testing.T) {
			t.Parallel()
			b := fx.build(t, decad.New())
			vol, err := b.Volume()
			require.NoError(t, err)
			placed, err := decad.Copy(t.Context(), b, decad.WithCopyPlacement(composedRotation(t, r3.NewVec(2, -3, 5), r3.NewVec(1, 1, 1), 30, 8)))
			require.NoError(t, err)
			mirror, err := r3.NewFrame(r3.NewVec(30, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
			require.NoError(t, err)
			mirrored, err := decad.MirroredCopy(t.Context(), b, decad.MirrorFrame{Frame: mirror})
			require.NoError(t, err)
			for _, c := range []*decad.Body{placed, mirrored} {
				mesh, err := c.Tessellate(t.Context(), units.Millimeters(0.05))
				require.NoError(t, err)
				requireWatertight(t, mesh)
				require.True(t, mesh.VolumeVerified())
				got := meshVolume(mesh)
				require.Positive(t, got, "the copy's facets wind outward")
				require.InEpsilon(t, vol.Value.Base(), got, 0.01)
			}
		})
	}
}

// TestDraftMeshOfBodyDraft meshes a Draft result about its end cap, whose
// far section is the start cap: the same band mesh serves it, closed and
// proving its volume, which sits at F1's closed form.
func TestDraftMeshOfBodyDraft(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := plainBox(t, doc, decad.Along)
	got, err := decad.Draft(t.Context(), box, decad.Faces(decad.Walls(box)), capNeutral(box, decad.CapEnd), units.Degrees(5))
	require.NoError(t, err)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.True(t, mesh.VolumeVerified())
	want, _ := boxVolume(bfMul(bf(boxHeight), taperTan(5))).Float64()
	require.InDelta(t, want, meshVolume(mesh), 1e-9)
}

// nearTangentSlot draws F3's slot with its right semicircle's centre moved
// shift mm along u. Its two ends stay equidistant from that centre, so the
// arc still passes through both, but its tangent at each end turns about
// shift/5 off the line it meets: tangent by the build's held-tangent rule, a
// sliver corner over the rationals.
func nearTangentSlot(shift float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		pt := func(u, v float64) *sketch.Point {
			p := s.CreatePoint(u, v)
			s.Fix(p)
			return p
		}
		a, b, c, d := pt(-15, -5), pt(15, -5), pt(15, 5), pt(-15, 5)
		s.CreateLine(a, b)
		s.CreateArc(pt(15+shift, 0), b, c)
		s.CreateLine(c, d)
		s.CreateArc(pt(-15, 0), d, a)
	}
}

// TestDraftNearTangentJoinBooleans pins agent-docs/draft-design.md §9.1.1 on
// a join the build reads as tangent and the record does not make exactly
// tangent: the slot's mesh proves its volume at VerifyAll, charging the
// sliver at its two near-tangent corners, and a Cut by a 4 mm square post
// through it removes the post's 128 mm³ from the closed form
// h[2ℓ(R − d/2) + π(R² − Rd + d²/3)] with ℓ = 30 + 2⁻⁴⁰, within the result's
// bound. Without the near-tangent arm in capband.OccupiedVolumeAdmission the
// mesh carries no proof and the Cut refuses ErrUnsupported.
func TestDraftNearTangentJoinBooleans(t *testing.T) {
	t.Parallel()
	shift := math.Ldexp(1, -40)
	doc := decad.New()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, nearTangentSlot(shift))
	b := taperExtrude(t, doc, s, p, 8, 3, decad.Along)
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.True(t, mesh.VolumeVerified())
	var stl strings.Builder
	require.NoError(t, export.STL(t.Context(), &stl, b, units.Millimeters(0.1)))

	post := boxBodyAtZ(t, doc, -2, -2, 2, 2, -5, 20)
	got, err := decad.Cut(t.Context(), b, post)
	require.NoError(t, err)
	d := bfMul(bf(8), taperTan(3))
	straight := bfMul(bf(2), bfAdd(bf(30), bf(shift)), bfSub(bf(5), bfQuo(d, bf(2))))
	round := bfMul(bigPi, bfAdd(bfSub(bf(25), bfMul(bf(5), d)), bfQuo(bfMul(d, d), bf(3))))
	requireVolumeCovers(t, "cut", got, bfSub(bfMul(bf(8), bfAdd(straight, round)), bf(128)), 1e-3)
	requireBodyWatertight(t, got)
}

// requirePastLimitNearTangentCutRefusal checks that a near-tangent extrusion
// without a volume proof refuses a Cut and names its corner and limit.
func requirePastLimitNearTangentCutRefusal(t *testing.T, draw func(*sketch.Sketch), cornerPattern string) {
	t.Helper()
	doc := decad.New()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, draw)
	b := taperExtrude(t, doc, s, p, 8, 3, decad.Along)
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.False(t, mesh.VolumeVerified())

	post := boxBodyAtZ(t, doc, -2, -2, 2, 2, -5, 20)
	_, err = decad.Cut(t.Context(), b, post)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "no proof of the volume")
	require.ErrorContains(t, err, "tapered extrude")
	require.ErrorContains(t, err, "exceeds the admitted sin²θ ≤ 2⁻⁶⁴")
	require.Regexp(t, cornerPattern, err.Error())
}

// TestDraftNearTangentJoinPastLimitRefuses pins §9.1.1's stated class: with
// the slot's centre moved 5·2⁻³¹ mm the build still reads both joins as
// tangent, but each turns by about 2⁻³¹ rad, past the admitted 2⁻³², so the
// mesh carries no volume proof and a boolean refuses it naming the corner and
// the limit. Without the limit gate in capband.NearTangentCorner the Cut
// builds.
func TestDraftNearTangentJoinPastLimitRefuses(t *testing.T) {
	t.Parallel()
	requirePastLimitNearTangentCutRefusal(t, nearTangentSlot(5*math.Ldexp(1, -31)),
		`the corner \(15, (-5|5)\) between recorded segments \d+ and \d+`)
}

// splitDiscSketch draws a radius-5 circle as two half arcs: the upper one
// about the origin and the lower one about (0, shift). Both recorded ends of
// the lower arc lie √(25 + shift²) from its centre, so each half meets the
// other a sliver off tangent, sin θ ≈ shift/5, at (±5, 0): tangent by the
// build's held-tangent rule, two near-tangent corners of two arcs over the
// rationals.
func splitDiscSketch(shift float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		pt := func(u, v float64) *sketch.Point {
			p := s.CreatePoint(u, v)
			s.Fix(p)
			return p
		}
		a, b := pt(5, 0), pt(-5, 0)
		s.CreateArc(pt(0, 0), a, b)
		s.CreateArc(pt(0, shift), b, a)
	}
}

// TestDraftTwoArcNearTangentJoinBooleans pins agent-docs/draft-design.md
// §9.1.1 on corners of two arcs whose centres sit a few ulps apart: the split
// disc with its lower centre moved 2⁻⁴⁸ mm (four ulps of 5) meshes with its
// volume proof at VerifyAll, charging the sliver at its two corners, and a
// Cut by a 4 mm square post through it removes the post's 128 mm³ from the
// closed form πh(25 − 5d + d²/3) − 2δh(5 − d/2), δ the shift, up to terms of
// order δ² (below 1e-27 mm³), within the result's bound. Shown to fail
// first: with the two-arc branch removed from capband.NearTangentCorner, the
// mesh carried no volume proof.
func TestDraftTwoArcNearTangentJoinBooleans(t *testing.T) {
	t.Parallel()
	shift := math.Ldexp(1, -48)
	doc := decad.New()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, splitDiscSketch(shift))
	b := taperExtrude(t, doc, s, p, 8, 3, decad.Along)
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.True(t, mesh.VolumeVerified())

	post := boxBodyAtZ(t, doc, -2, -2, 2, 2, -5, 20)
	got, err := decad.Cut(t.Context(), b, post)
	require.NoError(t, err)
	d := bfMul(bf(8), taperTan(3))
	round := bfMul(bigPi, bfAdd(bfSub(bf(25), bfMul(bf(5), d)), bfQuo(bfMul(d, d), bf(3))))
	lift := bfMul(bf(2), bf(shift), bfSub(bf(5), bfQuo(d, bf(2))))
	requireVolumeCovers(t, "cut", got, bfSub(bfMul(bf(8), bfSub(round, lift)), bf(128)), 1e-3)
	requireBodyWatertight(t, got)
}

// TestDraftTwoArcNearTangentPastLimitRefuses pins §9.1.1's stated class at
// corners of two arcs: with the lower centre moved 5·2⁻³¹ mm the build still
// reads both joins as tangent, but each turns by about 2⁻³¹ rad, past the
// admitted 2⁻³², so the mesh carries no volume proof and a boolean refuses
// it naming the corner and the limit. Shown to fail first: without the limit
// gate in capband.NearTangentCorner, the mesh carried the proof.
func TestDraftTwoArcNearTangentPastLimitRefuses(t *testing.T) {
	t.Parallel()
	requirePastLimitNearTangentCutRefusal(t, splitDiscSketch(5*math.Ldexp(1, -31)),
		`the corner \((-5|5), 0\) between recorded segments \d+ and \d+`)
}
