package decad

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file pins docs/draft-design.md Table DD row DD1's occupied-volume
// proof on the production path: every fixture F1–F7 of §11 is meshed at
// VerifyAll at three tolerances, and the exact signed volume of the held mesh
// must lie within the published volSymDiff of the closed form §8 states for
// the body, computed here in 256-bit arithmetic from d = h·tan α and never
// read back from the body.

const draftMeshPrec = 256

// draftMeshPi is π to 76 digits.
var draftMeshPi, _ = new(big.Float).SetPrec(draftMeshPrec).SetString("3.141592653589793238462643383279502884197169399375105820974944592307816406286")

func dmf(x float64) *big.Float { return new(big.Float).SetPrec(draftMeshPrec).SetFloat64(x) }

func dmAdd(xs ...*big.Float) *big.Float {
	out := new(big.Float).SetPrec(draftMeshPrec)
	for _, x := range xs {
		out.Add(out, x)
	}
	return out
}

func dmMul(xs ...*big.Float) *big.Float {
	out := new(big.Float).SetPrec(draftMeshPrec).SetInt64(1)
	for _, x := range xs {
		out.Mul(out, x)
	}
	return out
}

func dmQuo(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(draftMeshPrec).Quo(a, b) }

func dmNeg(a *big.Float) *big.Float { return new(big.Float).SetPrec(draftMeshPrec).Neg(a) }

// draftMeshTan is tan of the exact angle units.Degrees(deg) denotes: the
// Taylor series of sin and cos at deg times the float degree factor.
func draftMeshTan(deg float64) *big.Float {
	x := dmMul(dmf(deg), dmf(units.Degree.Factor()))
	x2 := dmMul(x, x)
	series := func(term *big.Float, first int64) *big.Float {
		total := new(big.Float).SetPrec(draftMeshPrec).Set(term)
		eps := new(big.Float).SetMantExp(big.NewFloat(1), -300)
		for k := first; ; k += 2 {
			term = dmQuo(dmMul(term, x2), new(big.Float).SetInt64(-k*(k+1)))
			total.Add(total, term)
			if new(big.Float).Abs(term).Cmp(eps) < 0 {
				return total
			}
		}
	}
	return dmQuo(series(x, 2), series(dmf(1), 1))
}

// draftMeshBody extrudes the sketch region with the given hole count h mm at
// a taper of deg degrees.
func draftMeshBody(t *testing.T, holes int, draw func(*sketch.Sketch), h, deg float64, dir Direction) *Body {
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
		b, err := New().Extrude(s, p, Distance{D: units.Millimeters(h), Dir: dir}, WithTaper(units.Degrees(deg)))
		require.NoError(t, err)
		return b
	}
	require.FailNow(t, "the sketch holds no region with the requested hole count")
	return nil
}

func draftMeshPolygon(pts ...[2]float64) func(*sketch.Sketch) {
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

func draftMeshCircles(radii ...float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		c := s.CreatePoint(0, 0)
		for _, r := range radii {
			s.CreateCircle(c, r)
		}
		s.Fix(c)
	}
}

func draftMeshSlot(s *sketch.Sketch) {
	slot, err := s.CreateSlot(-15, 0, 15, 0, 5)
	if err != nil {
		panic(err)
	}
	s.Fix(slot.C1)
	s.Fix(slot.C2)
}

// draftMeshFixture is one of docs/draft-design.md §11's F fixtures with its
// closed-form volume.
type draftMeshFixture struct {
	name   string
	build  func(*testing.T) *Body
	volume *big.Float
}

// draftMeshFixtures returns F1–F7.
func draftMeshFixtures() []draftMeshFixture {
	box := func(a, h float64, d *big.Float) *big.Float {
		A := dmf(a)
		return dmMul(dmf(h), dmAdd(dmMul(A, A), dmMul(dmf(-2), A, d), dmQuo(dmMul(dmf(4), d, d), dmf(3))))
	}
	square := draftMeshPolygon([2]float64{-10, -10}, [2]float64{10, -10}, [2]float64{10, 10}, [2]float64{-10, 10})
	d1 := dmMul(dmf(10), draftMeshTan(5))
	d2 := dmMul(dmf(10), draftMeshTan(10))
	d3 := dmMul(dmf(8), draftMeshTan(3))
	r2 := dmAdd(dmf(10), dmNeg(d2))
	R3 := dmf(5)
	return []draftMeshFixture{
		{"F1 box", func(t *testing.T) *Body { return draftMeshBody(t, 0, square, 10, 5, Along) }, box(20, 10, d1)},
		{"F2 frustum", func(t *testing.T) *Body { return draftMeshBody(t, 0, draftMeshCircles(10), 10, 10, Along) },
			dmQuo(dmMul(draftMeshPi, dmf(10), dmAdd(dmf(100), dmMul(dmf(10), r2), dmMul(r2, r2))), dmf(3))},
		{"F3 slot", func(t *testing.T) *Body { return draftMeshBody(t, 0, draftMeshSlot, 8, 3, Along) },
			dmMul(dmf(8), dmAdd(dmMul(dmf(60), dmAdd(R3, dmQuo(dmNeg(d3), dmf(2)))),
				dmMul(draftMeshPi, dmAdd(dmMul(R3, R3), dmNeg(dmMul(R3, d3)), dmQuo(dmMul(d3, d3), dmf(3))))))},
		{"F4 L-section", func(t *testing.T) *Body {
			return draftMeshBody(t, 0, draftMeshPolygon([2]float64{0, 0}, [2]float64{20, 0}, [2]float64{20, 10},
				[2]float64{10, 10}, [2]float64{10, 20}, [2]float64{0, 20}), 10, 5, Along)
		}, dmMul(dmf(10), dmAdd(dmf(300), dmMul(dmf(-40), d1), dmQuo(dmMul(dmf(4), d1, d1), dmf(3))))},
		{"F5 flare", func(t *testing.T) *Body { return draftMeshBody(t, 0, square, 10, -5, Along) }, box(20, 10, dmNeg(d1))},
		{"F6 against", func(t *testing.T) *Body { return draftMeshBody(t, 0, square, 10, 5, Against) }, box(20, 10, d1)},
		{"F7 ring", func(t *testing.T) *Body { return draftMeshBody(t, 1, draftMeshCircles(10, 4), 10, 5, Along) },
			dmMul(draftMeshPi, dmf(10), dmAdd(dmf(84), dmNeg(dmMul(d1, dmf(14)))))},
	}
}

// TestDraftMeshVolumeProofCoversTheClosedForm is the PR 2 fixture set of
// docs/draft-design.md §11 on the volume proof: F1–F7, each at three chord
// tolerances, mesh closed by the production audits, the proof admitted, and
// the held mesh's exact volume within volSymDiff of the body's closed form.
// volSymDiff stays below a relative ceiling, so the proof is not vacuous.
//
// Shown to fail first: with the draft's band height in capBlendChordVolume
// zeroed (draftBandHeightUpper's term), F2 at tol 0.25 published a volSymDiff
// of 1.4e-11 against a 76 mm³ chord deficit, and F3 and F7 failed alike; with
// the vertex-motion leg (SweptVolumeAllow) omitted, F1 published a zero
// volSymDiff against the 2.9e-13 its far corners' rounding moves the volume,
// and F4, F5 and F6 failed alike.
func TestDraftMeshVolumeProofCoversTheClosedForm(t *testing.T) {
	t.Parallel()
	for _, fx := range draftMeshFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			t.Parallel()
			b := fx.build(t)
			for _, tol := range []float64{0.25, 0.05, 0.01} {
				mesh, err := b.Tessellate(t.Context(), units.Millimeters(tol))
				require.NoError(t, err, "tol %g", tol)
				require.NoError(t, tessellation.RequireClosedMesh(mesh.triangles))
				require.True(t, mesh.symDiffOK, "tol %g: a draft band of miters and exact G1 joins is admitted", tol)
				got := new(big.Float).SetPrec(draftMeshPrec).SetRat(internalMeshVolumeRat(mesh))
				gap, _ := new(big.Float).Abs(dmAdd(got, dmNeg(fx.volume))).Float64()
				want, _ := fx.volume.Float64()
				require.LessOrEqualf(t, gap, mesh.volSymDiff, "tol %g: the mesh volume sits %g from the closed form %v, outside volSymDiff %g", tol, gap, want, mesh.volSymDiff)
				require.LessOrEqualf(t, mesh.volSymDiff, 0.1*want, "tol %g: volSymDiff %g is no proof of a %v mm³ body", tol, mesh.volSymDiff, want)
			}
		})
	}
}

// TestDraftMeshSharesTheNearRing pins the structure that makes a draft mesh
// watertight with no trimmed wall: the near cap's rim is written once and
// read by both the near cap and the walls, so F1 holds exactly its eight
// corners and two triangles per face, and every source face is the body's
// own.
func TestDraftMeshSharesTheNearRing(t *testing.T) {
	t.Parallel()
	b := draftMeshFixtures()[0].build(t)
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Len(t, mesh.vertices, 8)
	require.Len(t, mesh.triangles, 12)
	faces := map[*Face]int{}
	for _, f := range mesh.source {
		faces[f]++
	}
	require.Len(t, faces, 6)
	for _, f := range b.Faces() {
		require.Equal(t, 2, faces[f], "face %v", f.Origins())
	}
}

// TestDraftMoverRecordRadiusEnclosesTheBody pins Table DD row DD17's record
// radius: finite, and at least the L1 norm of every vertex of the body's mesh,
// which samples both sections and every wall. Without the draftPayload arm it
// read +Inf, as every unlisted payload class does.
func TestDraftMoverRecordRadiusEnclosesTheBody(t *testing.T) {
	t.Parallel()
	for _, fx := range draftMeshFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			t.Parallel()
			b := fx.build(t)
			radius := moverRecordRadius(t.Context(), b)
			mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			worst := 0.0
			for _, v := range mesh.vertices {
				worst = max(worst, vecL1(v))
			}
			require.False(t, math.IsInf(radius, 1), "a draft body states a finite record radius")
			require.LessOrEqual(t, worst, radius, fmt.Sprintf("%s: record radius %g below a vertex at %g", fx.name, radius, worst))
		})
	}
}
