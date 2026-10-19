package draftrecord_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file pins docs/draft-design.md Table DD row DD1's occupied-volume
// proof on the production mesh path (Tessellate at verifyAll): every fixture
// is meshed at three tolerances, and the exact signed volume of the held mesh
// must lie within the published VolSymDiff of the closed form §8 states for
// the body, computed here in 256-bit arithmetic from d = h·tan α and never
// read back from the body.

const meshVolumePrec = 256

// meshVolumePi is π to 76 digits.
var meshVolumePi, _ = new(big.Float).SetPrec(meshVolumePrec).SetString("3.141592653589793238462643383279502884197169399375105820974944592307816406286")

func bf(x float64) *big.Float { return new(big.Float).SetPrec(meshVolumePrec).SetFloat64(x) }

func bfSum(xs ...*big.Float) *big.Float {
	out := new(big.Float).SetPrec(meshVolumePrec)
	for _, x := range xs {
		out.Add(out, x)
	}
	return out
}

func bfProd(xs ...*big.Float) *big.Float {
	out := new(big.Float).SetPrec(meshVolumePrec).SetInt64(1)
	for _, x := range xs {
		out.Mul(out, x)
	}
	return out
}

func bfQuo(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(meshVolumePrec).Quo(a, b) }

func bfNeg(a *big.Float) *big.Float { return new(big.Float).SetPrec(meshVolumePrec).Neg(a) }

// taperTan is tan of the exact angle units.Degrees(deg) denotes: the Taylor
// series of sin and cos at deg times the float degree factor.
func taperTan(deg float64) *big.Float {
	x := bfProd(bf(deg), bf(units.Degree.Factor()))
	x2 := bfProd(x, x)
	series := func(term *big.Float, first int64) *big.Float {
		total := new(big.Float).SetPrec(meshVolumePrec).Set(term)
		eps := new(big.Float).SetMantExp(big.NewFloat(1), -300)
		for k := first; ; k += 2 {
			term = bfQuo(bfProd(term, x2), new(big.Float).SetInt64(-k*(k+1)))
			total.Add(total, term)
			if new(big.Float).Abs(term).Cmp(eps) < 0 {
				return total
			}
		}
	}
	return bfQuo(series(x, 2), series(bf(1), 1))
}

// sketchSection solves draw on the XY plane and records the region with the
// given hole count, as Extrude records it.
func sketchSection(t *testing.T, holes int, draw func(*sketch.Sketch)) (momentinput.Profile, r3.Frame) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	draw(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, p := range s.Profiles() {
		if len(p.Holes) != holes {
			continue
		}
		profile, plane, err := momentinput.RecordProfile(s, p)
		require.NoError(t, err)
		frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
		require.NoError(t, err)
		return profile, frame
	}
	require.FailNow(t, "the sketch holds no region with the requested hole count")
	return momentinput.Profile{}, r3.Frame{}
}

func polygon(pts ...[2]float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		ps := make([]*sketch.Point, len(pts))
		for i, p := range pts {
			ps[i] = s.CreatePoint(p[0], p[1])
			s.Fix(ps[i])
		}
		for i := range ps {
			s.CreateLine(ps[i], ps[(i+1)%len(ps)])
		}
	}
}

func circles(radii ...float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		c := s.CreatePoint(0, 0)
		for _, r := range radii {
			s.CreateCircle(c, r)
		}
		s.Fix(c)
	}
}

func slot(s *sketch.Sketch) {
	sl, err := s.CreateSlot(-15, 0, 15, 0, 5)
	if err != nil {
		panic(err)
	}
	s.Fix(sl.C1)
	s.Fix(sl.C2)
}

var (
	squareSection = polygon([2]float64{-10, -10}, [2]float64{10, -10}, [2]float64{10, 10}, [2]float64{-10, 10})
	ellSection    = polygon([2]float64{0, 0}, [2]float64{20, 0}, [2]float64{20, 10},
		[2]float64{10, 10}, [2]float64{10, 20}, [2]float64{0, 20})
)

// taperPayload is the record a tapered Extrude of h mm at deg degrees
// builds: along +n from the sketch plane, or against it.
func taperPayload(t *testing.T, profile momentinput.Profile, frame r3.Frame, h, deg float64, against bool) draftrecord.Payload {
	t.Helper()
	alpha, alphaDelta, err := extent.DisplacementIn(units.Degrees(deg), units.Angle, units.Radian, "the taper")
	require.NoError(t, err)
	z0, z1 := 0.0, h
	if against {
		z0, z1 = -h, 0
	}
	return draftrecord.Payload{
		Profile: profile, Frame: frame, Z0: z0, Z1: z1, NearStart: z0 == 0,
		Taper: alpha, TaperDelta: alphaDelta, Xform: r3.Identity(),
	}
}

// meshVolumeFixture is one body with its closed-form volume.
type meshVolumeFixture struct {
	name    string
	payload func(*testing.T) draftrecord.Payload
	volume  *big.Float
}

// exactMeshVolume is the held mesh's signed volume over the exact rationals
// of its float coordinates.
func exactMeshVolume(m *topo.Mesh) *big.Float {
	total := new(big.Rat)
	for _, tri := range m.Triangles {
		var p [3][3]*big.Rat
		for k, vi := range tri {
			v := m.Vertices[vi]
			p[k] = [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
		}
		minor := func(a, b, c, d *big.Rat) *big.Rat {
			return new(big.Rat).Sub(new(big.Rat).Mul(a, d), new(big.Rat).Mul(b, c))
		}
		det := new(big.Rat).Mul(p[0][0], minor(p[1][1], p[1][2], p[2][1], p[2][2]))
		det.Sub(det, new(big.Rat).Mul(p[0][1], minor(p[1][0], p[1][2], p[2][0], p[2][2])))
		det.Add(det, new(big.Rat).Mul(p[0][2], minor(p[1][0], p[1][1], p[2][0], p[2][1])))
		total.Add(total, det)
	}
	total.Quo(total, big.NewRat(6, 1))
	return new(big.Float).SetPrec(meshVolumePrec).SetRat(total)
}

// meshContactAudit proves the held facets embedded: every facet has
// positive area, adjacent facets meet only along the vertex or edge their
// indices share, and no other pair touches. It is the revolve tessellator's
// exact facet-pair audit (docs/tessellation-design.md §9) run at a zero
// displacement, so it reads the held coordinates and nothing else.
func meshContactAudit(t *testing.T, verts []r3.Vec, tris [][3]int) error {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	data, err := revolvemesh.RequireRevolveFacetAreas(budget, verts, tris, 0)
	if err != nil {
		return err
	}
	return revolvemesh.RevolveContactAudit(budget, data, tris, 0)
}

// requireVolumeProofCovers meshes fx at three tolerances and checks each
// mesh closed and embedded, its proof admitted, the held mesh's exact volume
// within VolSymDiff of the closed form, and VolSymDiff below a tenth of the
// volume, so the proof is not vacuous.
func requireVolumeProofCovers(t *testing.T, fx meshVolumeFixture) {
	t.Helper()
	env := topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}}
	body, err := draftrecord.Build(t.Context(), env, fx.payload(t))
	require.NoError(t, err)
	dp, ok := body.Payload.(draftrecord.Payload)
	require.True(t, ok)
	want, _ := fx.volume.Float64()
	for _, tol := range []float64{0.25, 0.05, 0.01} {
		m, err := draftrecord.Tessellate(t.Context(), body, dp, tol, true)
		require.NoError(t, err, "tol %g", tol)
		require.NoError(t, tessellation.RequireClosedMesh(m.Triangles), "tol %g", tol)
		require.NoError(t, meshContactAudit(t, m.Vertices, m.Triangles), "tol %g", tol)
		require.True(t, m.SymDiffOK, "tol %g: a draft band of miters and exact G1 joins is admitted", tol)
		gap, _ := new(big.Float).Abs(bfSum(exactMeshVolume(m), bfNeg(fx.volume))).Float64()
		require.LessOrEqualf(t, gap, m.VolSymDiff,
			"tol %g: the mesh volume sits %g from the closed form %v, outside VolSymDiff %g", tol, gap, want, m.VolSymDiff)
		require.LessOrEqualf(t, m.VolSymDiff, 0.1*want, "tol %g: VolSymDiff %g is no proof of a %v mm³ body", tol, m.VolSymDiff, want)
	}
}

// draftFixtures returns docs/draft-design.md §11's F1–F7 records with their
// closed-form volumes.
func draftFixtures() []meshVolumeFixture {
	box := func(a, h float64, d *big.Float) *big.Float {
		A := bf(a)
		return bfProd(bf(h), bfSum(bfProd(A, A), bfProd(bf(-2), A, d), bfQuo(bfProd(bf(4), d, d), bf(3))))
	}
	section := func(holes int, draw func(*sketch.Sketch), h, deg float64, against bool) func(*testing.T) draftrecord.Payload {
		return func(t *testing.T) draftrecord.Payload {
			profile, frame := sketchSection(t, holes, draw)
			return taperPayload(t, profile, frame, h, deg, against)
		}
	}
	d1 := bfProd(bf(10), taperTan(5))
	d2 := bfProd(bf(10), taperTan(10))
	d3 := bfProd(bf(8), taperTan(3))
	r2 := bfSum(bf(10), bfNeg(d2))
	r3c := bf(5)
	return []meshVolumeFixture{
		{"F1 box", section(0, squareSection, 10, 5, false), box(20, 10, d1)},
		{"F2 frustum", section(0, circles(10), 10, 10, false),
			bfQuo(bfProd(meshVolumePi, bf(10), bfSum(bf(100), bfProd(bf(10), r2), bfProd(r2, r2))), bf(3))},
		{"F3 slot", section(0, slot, 8, 3, false),
			bfProd(bf(8), bfSum(bfProd(bf(60), bfSum(r3c, bfQuo(bfNeg(d3), bf(2)))),
				bfProd(meshVolumePi, bfSum(bfProd(r3c, r3c), bfNeg(bfProd(r3c, d3)), bfQuo(bfProd(d3, d3), bf(3))))))},
		{"F4 L-section", section(0, ellSection, 10, 5, false),
			bfProd(bf(10), bfSum(bf(300), bfProd(bf(-40), d1), bfQuo(bfProd(bf(4), d1, d1), bf(3))))},
		{"F5 flare", section(0, squareSection, 10, -5, false), box(20, 10, bfNeg(d1))},
		{"F6 against", section(0, squareSection, 10, 5, true), box(20, 10, d1)},
		{"F7 ring", section(1, circles(10, 4), 10, 5, false),
			bfProd(meshVolumePi, bf(10), bfSum(bf(84), bfNeg(bfProd(d1, bf(14)))))},
	}
}

// TestDraftMeshVolumeProofCoversTheClosedForm is docs/draft-design.md §11's
// PR 2 fixture set on the volume proof: F1–F7, each at three chord
// tolerances, the held mesh's exact volume within VolSymDiff of the body's
// closed form.
func TestDraftMeshVolumeProofCoversTheClosedForm(t *testing.T) {
	t.Parallel()
	for _, fx := range draftFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			t.Parallel()
			requireVolumeProofCovers(t, fx)
		})
	}
}

// TestDraftSubsetMeshVolumeProofCoversTheClosedForm is the subset draft's
// mesh (docs/draft-design.md §10.2 over Table DD row DD1): F1's box with one
// wall drafted alone, h·a(a − d/2); F4's L with its notch wall u = 10 drafted
// alone, h(300 − 5d); and F7's hole drafted alone, πh(R² − r² − rd − d²/3).
// Each meshes closed at three tolerances, its proof admitted, and the held
// mesh's exact volume lies within VolSymDiff of the closed form.
func TestDraftSubsetMeshVolumeProofCoversTheClosedForm(t *testing.T) {
	t.Parallel()
	d := bfProd(bf(10), taperTan(5))
	// subset drafts the walls drafted picks and keeps every other wall, as
	// Draft about the start cap records them.
	subset := func(holes int, draw func(*sketch.Sketch), drafted func(li int, seg sectionrecord.CurveSegment) bool) func(*testing.T) draftrecord.Payload {
		return func(t *testing.T) draftrecord.Payload {
			profile, frame := sketchSection(t, holes, draw)
			dp := taperPayload(t, profile, frame, 10, 5, false)
			dp.Kept = map[offset2d.DraftWall]struct{}{}
			loops := append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...)
			moved := 0
			for li, loop := range loops {
				for j, seg := range loop.Segments {
					if drafted(li, seg) {
						moved++
						continue
					}
					dp.Kept[offset2d.DraftWall{Loop: li, Seg: j}] = struct{}{}
				}
			}
			require.Positive(t, moved, "the fixture drafts at least one wall")
			require.NotEmpty(t, dp.Kept, "the fixture keeps at least one wall")
			return dp
		}
	}
	lineAt := func(u float64) func(int, sectionrecord.CurveSegment) bool {
		return func(_ int, seg sectionrecord.CurveSegment) bool {
			l, ok := seg.(sectionrecord.LineSeg)
			return ok && l.Start.U == u && l.End.U == u
		}
	}
	for _, fx := range []meshVolumeFixture{
		{"one wall", subset(0, squareSection, lineAt(10)),
			bfProd(bf(10), bf(20), bfSum(bf(20), bfQuo(bfNeg(d), bf(2))))},
		{"notch wall", subset(0, ellSection, lineAt(10)),
			bfProd(bf(10), bfSum(bf(300), bfProd(bf(-5), d)))},
		{"hole alone", subset(1, circles(10, 4), func(li int, _ sectionrecord.CurveSegment) bool { return li == 1 }),
			bfProd(meshVolumePi, bf(10), bfSum(bf(84), bfProd(bf(-4), d), bfQuo(bfNeg(bfProd(d, d)), bf(3))))},
	} {
		t.Run(fx.name, func(t *testing.T) {
			t.Parallel()
			requireVolumeProofCovers(t, fx)
		})
	}
}

// TestDraftMeshEmbeddedCheckRefusesACrossing keeps the embedded check from
// passing vacuously: F1's mesh with one far corner pushed through the near
// cap, so its two walls cross the cap's facets, refuses.
func TestDraftMeshEmbeddedCheckRefusesACrossing(t *testing.T) {
	t.Parallel()
	profile, frame := sketchSection(t, 0, squareSection)
	env := topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}}
	body, err := draftrecord.Build(t.Context(), env, taperPayload(t, profile, frame, 10, 5, false))
	require.NoError(t, err)
	dp, ok := body.Payload.(draftrecord.Payload)
	require.True(t, ok)
	m, err := draftrecord.Tessellate(t.Context(), body, dp, 0.1, true)
	require.NoError(t, err)
	require.NoError(t, meshContactAudit(t, m.Vertices, m.Triangles))
	verts := append([]r3.Vec(nil), m.Vertices...)
	moved := false
	for i, v := range verts {
		if v.Z == 10 {
			verts[i] = r3.NewVec(v.X, v.Y, -5)
			moved = true
			break
		}
	}
	require.True(t, moved)
	require.ErrorIs(t, meshContactAudit(t, verts, m.Triangles), decaderr.ErrUnsupported)
}

// TestDraftMoverRecordRadiusEnclosesTheBody pins Table DD row DD17's record
// radius: finite, at least the L1 norm of every vertex of the body's mesh,
// which samples both sections and every wall, and at most twice the largest.
func TestDraftMoverRecordRadiusEnclosesTheBody(t *testing.T) {
	t.Parallel()
	env := topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}}
	for _, fx := range draftFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			t.Parallel()
			body, err := draftrecord.Build(t.Context(), env, fx.payload(t))
			require.NoError(t, err)
			dp, ok := body.Payload.(draftrecord.Payload)
			require.True(t, ok)
			radius := dp.MoverRecordRadius()
			m, err := draftrecord.Tessellate(t.Context(), body, dp, 0.1, true)
			require.NoError(t, err)
			worst := 0.0
			for _, v := range m.Vertices {
				worst = max(worst, math.Abs(v.X)+math.Abs(v.Y)+math.Abs(v.Z))
			}
			require.False(t, math.IsInf(radius, 1), "a draft body states a finite record radius")
			require.LessOrEqualf(t, worst, radius, "record radius %g below a vertex at %g", radius, worst)
			require.LessOrEqualf(t, radius, 2*worst, "record radius %g past twice the largest vertex at %g", radius, worst)
		})
	}
}
