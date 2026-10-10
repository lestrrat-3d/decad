package decad

import (
	"math/big"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures are docs/helix-design.md §13's thread rows: a 60° V groove of
// depth 0.9 at pitch 1.5, 8 turns, cut from an R = 5, L = 20 cylinder or
// bored block. The coil's δ, near 9.2e-4 mm, is above the pair's chord
// tolerance 2e-5·diameter, so the pair tolerance rises to δ
// (heldPrimitiveFloorOf, docs/faceted-vertex-bounds-design.md §5) and the
// chain-depth gate admits the groove's facets. Shown to fail: without the
// coil arm in heldPrimitiveFloorOf both cuts refused at the chain-depth gate.

// threadHalfWidth is tan 30° times the 1.2 mm the groove spans radially, held
// as a float; the recorded profile and the closed form both read this float.
const threadHalfWidth = 0.6928203230275509

// threadClippedMoment is Q = ∫ρ dA over the part of the V groove with root
// (root, z0) and mouth corners (mouth, z0 ± threadHalfWidth) that lies on
// the root's side of radius r: a triangle with apex at the root, cut by the
// line ρ = r at the fraction t = (r − root)/(mouth − root) of its height, so
// its area is t·w·|r − root| and its mean radius (root + 2r)/3. Every
// coordinate is the recorded float, read exactly.
func threadClippedMoment(root, mouth, r float64) *big.Rat {
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	run := new(big.Rat).Sub(rat(r), rat(root))
	t := new(big.Rat).Quo(run, new(big.Rat).Sub(rat(mouth), rat(root)))
	area := new(big.Rat).Mul(t, rat(threadHalfWidth))
	area.Mul(area, new(big.Rat).Abs(run))
	mean := new(big.Rat).Add(rat(root), new(big.Rat).Mul(big.NewRat(2, 1), rat(r)))
	mean.Quo(mean, big.NewRat(3, 1))
	return area.Mul(area, mean)
}

// threadTool coils the V groove with root at radius root, mouth at radius
// mouth and centre height z0 about the sketch's V axis, 8 turns at pitch 1.5.
func threadTool(t *testing.T, doc *Document, root, mouth, z0 float64) *Body {
	t.Helper()
	gs, gp := coilLoopsSketch(t, [][2]float64{{root, z0}, {mouth, z0 - threadHalfWidth}, {mouth, z0 + threadHalfWidth}})
	tool, err := doc.Coil(t.Context(), gs, gp, coilAxisV, units.Millimeters(1.5), units.Scalar(8))
	require.NoError(t, err)
	return tool
}

// requireThreadResult asserts the cut's published volume encloses
// π·(volume/π) where exactOverPi = base − 16·Q, its box holds the true
// extremes within its own bound, its mesh is watertight, and Verify proves
// validity.
func requireThreadResult(t *testing.T, doc *Document, res *Body, exactOverPi *big.Rat, lo, hi r3.Vec) *Report {
	t.Helper()
	require.Len(t, res.Lumps(), 1)
	require.Len(t, res.Shells(), 1)
	vol, err := res.Volume()
	require.NoError(t, err)
	exact := new(big.Float).SetPrec(512).Mul(bigPi(), new(big.Float).SetPrec(512).SetRat(exactOverPi))
	requireEnclosesBig(t, vol.Value.Base(), vol.Bound.Base(), exact, "threaded volume")

	box, err := res.Bounds()
	require.NoError(t, err)
	b := box.Bound.Base()
	for axis := range 3 {
		get := func(v r3.Vec) float64 { return [3]float64{v.X, v.Y, v.Z}[axis] }
		require.InDelta(t, get(lo), get(box.Min), b, "box min on axis %d", axis)
		require.InDelta(t, get(hi), get(box.Max), b, "box max on axis %d", axis)
	}

	mesh, err := res.Tessellate(t.Context(), units.Millimeters(0.01))
	require.NoError(t, err)
	require.NoError(t, tessellation.RequireClosedMesh(mesh.Triangles()), "the threaded mesh is watertight")

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, rep.Bodies, 1)
	require.Equal(t, ValidityValid, rep.Bodies[0].Validity.Outcome)
	return rep
}

// requireOnlyBeyond asserts every diagnostic the report carries is a reading
// beyond the tolerance, and that the readings named are exactly want.
func requireOnlyBeyond(t *testing.T, rep *Report, want ...string) {
	t.Helper()
	require.Equal(t, Suspect, rep.Status)
	var got []string
	for _, d := range rep.Diagnostics {
		require.Equal(t, DiagMeasurementBeyondTolerance, d.Code, d.Message)
		got = append(got, strings.Fields(d.Message)[1])
	}
	require.ElementsMatch(t, want, got)
}

// TestCoilExternalThread cuts the groove, root at 4.1 and mouth at 5.3,
// from a revolved cylinder: the cut removes the screw sweep of the groove's
// part inside ρ = 5, Θ·Q = 16π·Q, from π·5²·20. The volume reading sits
// within the default tolerance; the area and the centroid do not, so the
// report reads Suspect.
func TestCoilExternalThread(t *testing.T) {
	doc := New()
	cs, cpf := coilLoopsSketch(t, [][2]float64{{0, 0}, {5, 0}, {5, 20}, {0, 20}})
	cylinder, err := doc.Revolve(cs, cpf, coilAxisV, FullRevolution{})
	require.NoError(t, err)
	tool := threadTool(t, doc, 4.1, 5.3, 3)
	res, err := Cut(t.Context(), cylinder, tool)
	require.NoError(t, err)

	exact := new(big.Rat).Mul(big.NewRat(16, 1), threadClippedMoment(4.1, 5.3, 5))
	exact.Sub(big.NewRat(500, 1), exact)
	rep := requireThreadResult(t, doc, res, exact, r3.NewVec(-5, 0, -5), r3.NewVec(5, 20, 5))
	requireOnlyBeyond(t, rep, "area", "centroid")
}

// TestCoilInternalThread cuts the groove, root at 5.9 and mouth at 4.7, from
// an Extrude of the annulus 5 ≤ ρ ≤ 10 on the XZ plane, which runs from
// y = −20 to 0: the cut removes the screw sweep of the groove's part outside
// ρ = 5 from π·(10² − 5²)·20. The volume reading sits within the default
// tolerance; the area does not, so the report reads Suspect.
func TestCoilInternalThread(t *testing.T) {
	doc := New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 10)
	s.CreateCircle(c, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var annulus *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			annulus = p
		}
	}
	require.NotNil(t, annulus)
	block, err := doc.Extrude(s, annulus, Distance{D: units.Millimeters(20), Dir: Along})
	require.NoError(t, err)
	tool := threadTool(t, doc, 5.9, 4.7, -17)
	res, err := Cut(t.Context(), block, tool)
	require.NoError(t, err)

	exact := new(big.Rat).Mul(big.NewRat(16, 1), threadClippedMoment(5.9, 4.7, 5))
	exact.Sub(big.NewRat(1500, 1), exact)
	rep := requireThreadResult(t, doc, res, exact, r3.NewVec(-10, -20, -10), r3.NewVec(10, 0, 10))
	requireOnlyBeyond(t, rep, "area")
}

// TestCoilThreadPartnerKeepsItsBounds reads the pair tolerance the external
// thread's Cut runs at — the coil's δ, above 2e-5 times the pair diameter —
// and meshes the cylinder there, as the boolean does. That mesh is coarser
// than the diameter-derived tolerance would give, and its own proofs still
// hold: its signed volume lies within volSymDiff of π·5²·20 and its area
// within areaSlack of the cylinder's 250π.
func TestCoilThreadPartnerKeepsItsBounds(t *testing.T) {
	doc := New()
	cs, cpf := coilLoopsSketch(t, [][2]float64{{0, 0}, {5, 0}, {5, 20}, {0, 20}})
	cylinder, err := doc.Revolve(cs, cpf, coilAxisV, FullRevolution{})
	require.NoError(t, err)
	tool := threadTool(t, doc, 4.1, 5.3, 3)
	delta := tool.payload.(coilPayload).delta

	tol, dPair, err := pairChordTolerance(t.Context(), cylinder, tool)
	require.NoError(t, err)
	require.Equal(t, delta, tol, "the pair tolerance rises to the coil's δ")
	require.Less(t, dPair*meshbool.ChordFactor, tol)

	fine, err := cylinder.Tessellate(t.Context(), units.Millimeters(dPair*meshbool.ChordFactor))
	require.NoError(t, err)
	mesh, err := cylinder.Tessellate(t.Context(), units.Millimeters(tol))
	require.NoError(t, err)
	require.Less(t, len(mesh.Triangles()), len(fine.Triangles()), "the partner's mesh is coarser")
	require.True(t, mesh.VolumeVerified())

	vol := new(big.Float).SetPrec(512).Mul(bigPi(), big.NewFloat(500))
	requireEnclosesBig(t, meshSignedVolume(mesh.Vertices(), mesh.Triangles()), mesh.volSymDiff, vol, "partner mesh volume")
	area := new(big.Float).SetPrec(512).Mul(bigPi(), big.NewFloat(250))
	requireEnclosesBig(t, meshArea(mesh.Vertices(), mesh.Triangles()), mesh.areaSlack+1e-9, area, "partner mesh area")
}

// polygonMoments returns ∫ρ dA and ∫ρζ dA over a simple polygon given as
// (ρ, ζ) corners, by the shoelace forms of the first and mixed moments.
func polygonMoments(pts [][2]*big.Rat) (*big.Rat, *big.Rat) {
	q, m := new(big.Rat), new(big.Rat)
	mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
	for i := range pts {
		x0, y0 := pts[i][0], pts[i][1]
		x1, y1 := pts[(i+1)%len(pts)][0], pts[(i+1)%len(pts)][1]
		c := new(big.Rat).Sub(mul(x0, y1), mul(x1, y0))
		q.Add(q, mul(new(big.Rat).Add(x0, x1), c))
		mixed := new(big.Rat).Add(mul(x0, y1), mul(x1, y0))
		mixed.Add(mixed, mul(big.NewRat(2, 1), new(big.Rat).Add(mul(x0, y0), mul(x1, y1))))
		m.Add(m, mul(mixed, c))
	}
	q.Quo(q, big.NewRat(6, 1))
	m.Quo(m, big.NewRat(24, 1))
	if q.Sign() < 0 {
		q.Neg(q)
		m.Neg(m)
	}
	return q, m
}

// clipPolygon keeps the part of a polygon where keep(p) ≥ 0 for an affine
// keep, Sutherland–Hodgman over exact rationals.
func clipPolygon(pts [][2]*big.Rat, keep func(p [2]*big.Rat) *big.Rat) [][2]*big.Rat {
	var out [][2]*big.Rat
	for i := range pts {
		a, b := pts[i], pts[(i+1)%len(pts)]
		fa, fb := keep(a), keep(b)
		if fa.Sign() >= 0 {
			out = append(out, a)
		}
		if fa.Sign()*fb.Sign() < 0 {
			t := new(big.Rat).Quo(fa, new(big.Rat).Sub(fa, fb))
			var p [2]*big.Rat
			for k := range 2 {
				p[k] = new(big.Rat).Add(a[k], new(big.Rat).Mul(t, new(big.Rat).Sub(b[k], a[k])))
			}
			out = append(out, p)
		}
	}
	return out
}

// TestCoilThreadRunsOutOfTheEnd starts the external groove pitch/4 below the
// cylinder's end plane y = 0, so the coil's first turn crosses that plane
// and the cut trims it. The material removed is ∫_Ω ρ·(Θ − max(0, −ζ/k)) dA
// over the groove's part inside ρ = 5: Θ·Q + M⁻/k, with M⁻ = ∫ρζ dA over
// that part below ζ = 0 and k = pitch/2π, so the volume is
// π·(500 − 6Q − (2/1.5)·M⁻) at three turns.
func TestCoilThreadRunsOutOfTheEnd(t *testing.T) {
	doc := New()
	cs, cpf := coilLoopsSketch(t, [][2]float64{{0, 0}, {5, 0}, {5, 20}, {0, 20}})
	cylinder, err := doc.Revolve(cs, cpf, coilAxisV, FullRevolution{})
	require.NoError(t, err)
	const z0 = -0.375
	gs, gp := coilLoopsSketch(t, [][2]float64{{4.1, z0}, {5.3, z0 - threadHalfWidth}, {5.3, z0 + threadHalfWidth}})
	tool, err := doc.Coil(t.Context(), gs, gp, coilAxisV, units.Millimeters(1.5), units.Scalar(3))
	require.NoError(t, err)
	res, err := Cut(t.Context(), cylinder, tool)
	require.NoError(t, err)
	require.Len(t, res.Lumps(), 1)

	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	groove := [][2]*big.Rat{
		{rat(4.1), rat(z0)},
		{rat(5.3), new(big.Rat).Sub(rat(z0), rat(threadHalfWidth))},
		{rat(5.3), new(big.Rat).Add(rat(z0), rat(threadHalfWidth))},
	}
	inside := clipPolygon(groove, func(p [2]*big.Rat) *big.Rat { return new(big.Rat).Sub(rat(5), p[0]) })
	q, _ := polygonMoments(inside)
	_, below := polygonMoments(clipPolygon(inside, func(p [2]*big.Rat) *big.Rat { return new(big.Rat).Neg(p[1]) }))
	exact := new(big.Rat).Mul(big.NewRat(6, 1), q)
	exact.Add(exact, new(big.Rat).Mul(big.NewRat(4, 3), below))
	exact.Sub(big.NewRat(500, 1), exact)

	vol, err := res.Volume()
	require.NoError(t, err)
	requireEnclosesBig(t, vol.Value.Base(), vol.Bound.Base(),
		new(big.Float).SetPrec(512).Mul(bigPi(), new(big.Float).SetPrec(512).SetRat(exact)), "run-out volume")
}

// TestCoilCapFlushWithAnAxialFaceRefuses cuts the groove from a half
// cylinder revolved 180° from the sketch plane. Every cap of a coil lies in a
// plane through its axis, so the coil's start cap lies in the same half-plane
// as the half cylinder's start cap: a coplanar contact the mesh boolean's
// hidden-tangency gate refuses (docs/general-boolean-design.md §2), for
// either sense of the half turn.
func TestCoilCapFlushWithAnAxialFaceRefuses(t *testing.T) {
	for _, dir := range []Direction{Along, Against} {
		doc := New()
		cs, cpf := coilLoopsSketch(t, [][2]float64{{0, 0}, {5, 0}, {5, 20}, {0, 20}})
		half, err := doc.Revolve(cs, cpf, coilAxisV, AngleExtent{A: units.Degrees(180), Dir: dir})
		require.NoError(t, err)
		gs, gp := coilLoopsSketch(t, [][2]float64{{4.1, 3}, {5.3, 3 - threadHalfWidth}, {5.3, 3 + threadHalfWidth}})
		tool, err := doc.Coil(t.Context(), gs, gp, coilAxisV, units.Millimeters(1.5), units.Scalar(2))
		require.NoError(t, err)
		_, err = Cut(t.Context(), half, tool)
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "come within the chord tolerance without provably interpenetrating")
		require.Len(t, doc.Bodies(), 2, "a refused cut leaves both operands live")
	}
}
