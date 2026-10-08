package decad

import (
	"math/big"
	"strings"
	"testing"

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
	require.Less(t, dPair*boolChordFactor, tol)

	fine, err := cylinder.Tessellate(t.Context(), units.Millimeters(dPair*boolChordFactor))
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
