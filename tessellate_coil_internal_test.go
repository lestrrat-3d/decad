package decad

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures assert docs/helix-design.md §13's PR 2 rows against the
// production path. Bound legs shown to fail by deleting them and watching the
// named assertion go red, then restoring them:
//
//   - volSymDiff's wall homotopy (coil.CellProof.Swept summed per segment):
//     TestCoilMeshProofsEncloseTheBody's volume check and
//     TestCoilMassPropertiesSettleAtTheFirstStep's mass went red;
//   - the held-to-true-corner rounding homotopy (proofbound.SweptVolumeAllow
//     at the largest station rounding) is below every fixture's slack, about
//     1e-13 mm³, so deleting it left them green;
//   - areaSlack's legs are each far above the true area gap on these
//     fixtures, so no single deletion turns the area check red.

// meshSignedVolume is the held triangles' tetrahedron sum.
func meshSignedVolume(verts []r3.Vec, tris [][3]int) float64 {
	total := 0.0
	for _, t := range tris {
		total += verts[t[0]].Dot(verts[t[1]].Cross(verts[t[2]])) / 6
	}
	return total
}

// meshArea is the held triangles' area.
func meshArea(verts []r3.Vec, tris [][3]int) float64 {
	total := 0.0
	for _, t := range tris {
		total += verts[t[1]].Sub(verts[t[0]]).Cross(verts[t[2]].Sub(verts[t[0]])).Len() / 2
	}
	return total
}

func TestCoilMeshRestatesTheHeldShell(t *testing.T) {
	s, p := coilSquare(t)
	b, err := New().Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2))
	require.NoError(t, err)
	cp := b.payload.(coilPayload)
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(cp.delta))
	require.NoError(t, err)

	require.Equal(t, cp.verts, mesh.Vertices())
	require.Equal(t, cp.tris, mesh.Triangles())
	require.Equal(t, cp.vertexBound, mesh.vertexBound)
	require.Equal(t, cp.delta, mesh.Bound().Base())
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())

	// Walls attribute to their segment's face, ring after ring, then the two
	// caps; every face's bound is the largest β over its own triangles.
	walls := 2 * 512 * 4
	faceMax := map[*Face]float64{}
	for k, tri := range mesh.Triangles() {
		f := mesh.source[k]
		want := roleCapEnd
		switch {
		case k < walls:
			want = fmt.Sprintf("side(0,%d)", (k/2)%4)
		case k < walls+2:
			want = roleCapStart
		}
		require.Equal(t, want, f.Origins()[0].Role, "triangle %d", k)
		faceMax[f] = max(faceMax[f], cp.vertexBound[tri[0]], cp.vertexBound[tri[1]], cp.vertexBound[tri[2]])
	}
	for f, want := range faceMax {
		got, ok := mesh.sourceBound(f)
		require.True(t, ok)
		require.Equal(t, want, got)
	}

	_, err = b.Tessellate(t.Context(), units.Millimeters(math.Nextafter(cp.delta, 0)))
	require.ErrorIs(t, err, ErrUnsupported)
	drawn, err := b.Tessellate(t.Context(), units.Millimeters(1), WithVerification(VerifyNone))
	require.NoError(t, err)
	require.False(t, drawn.VolumeVerified())
	require.Equal(t, cp.tris, drawn.Triangles())
}

// TestCoilMeshProofsEncloseTheBody checks what each proof implies about the
// held mesh against the closed forms: the signed volume differs from the
// true volume by at most volSymDiff, and the held area from the true area by
// at most areaSlack plus the area reading's own bound. Both are falsifiers;
// the proofs are docs/helix-design.md §8.1 and §8.2.
func TestCoilMeshProofsEncloseTheBody(t *testing.T) {
	for _, c := range []struct {
		name  string
		loop  [][2]float64
		turns float64
		// exact volume / π
		volume float64
	}{
		{"square spring", [][2]float64{{2, 0}, {3, 0}, {3, 1}, {2, 1}}, 2, 10},
		// Q = (4² − 2²)/2 = 6, Θ = 3π.
		{"wide profile", [][2]float64{{2, 0}, {4, 0}, {4, 1}, {2, 1}}, 1.5, 18},
		// Q = (3² − 0.5²)/2 = 4.375, Θ = 2π.
		{"near the axis", [][2]float64{{0.5, 0}, {3, 0}, {3, 1}, {0.5, 1}}, 1, 8.75},
	} {
		t.Run(c.name, func(t *testing.T) {
			s, p := coilLoopsSketch(t, c.loop)
			b, err := New().Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(c.turns))
			require.NoError(t, err)
			mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			held := meshSignedVolume(mesh.Vertices(), mesh.Triangles())
			exact := new(big.Float).SetPrec(512).Mul(bigPi(), big.NewFloat(c.volume))
			requireEnclosesBig(t, held, mesh.volSymDiff+1e-12, exact, "held volume")

			area, err := b.Area()
			require.NoError(t, err)
			gap := math.Abs(meshArea(mesh.Vertices(), mesh.Triangles()) - area.Value.Base())
			require.LessOrEqual(t, gap, mesh.areaSlack+area.Bound.Base()+1e-9)
		})
	}
}

func TestCoilMassPropertiesSettleAtTheFirstStep(t *testing.T) {
	s, p := coilSquare(t)
	b, err := New().Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2))
	require.NoError(t, err)
	got, err := b.MassProperties(t.Context(), meshLadderDensity)
	require.NoError(t, err)
	first, err := meshLadderStep(t, b, meshLadderFirst)
	require.NoError(t, err)
	require.Equal(t, first, got)
	rho := new(big.Rat).SetFloat64(meshLadderDensity.Base())
	requireMeshReadingCovers(t, got.Mass, new(big.Rat).Mul(rho, new(big.Rat).Mul(meshLadderPi, big.NewRat(10, 1))))
	// The centre of mass sits on the axis at M/Q + pitch·turns/2 = 2.
	require.LessOrEqual(t, got.Center.Value.Sub(r3.NewVec(0, 2, 0)).Len(), got.Center.Bound.Base())
}

// TestCoilInterferenceAndUnionWithACore overlaps the square spring with a
// coaxial cylinder of radius 2.5 through its core, 60 mm long so that the
// pair's chord tolerance 2e-5·diameter sits above the coil's δ and the
// boolean's chain-depth gate admits the coil's facets. Verify measures the
// overlap Θ·Q(Ω ∩ {ρ ≤ 2.5}) = 4π·(2.5² − 2²)/2 = 4.5π, and the union reads
// V_cylinder + Θ·Q(Ω ∩ {ρ ≥ 2.5}) = 375π + 5.5π and verifies Sound. With the
// coil's twist-area leg read through proofbound.CellTwistAreaAllow instead
// of the projected arm, the union's area bound rose to 2.6 mm² and Verify
// read Suspect.
func TestCoilInterferenceAndUnionWithACore(t *testing.T) {
	doc := New()
	cs, cpf := coilLoopsSketch(t, [][2]float64{{0, -28}, {2.5, -28}, {2.5, 32}, {0, 32}})
	core, err := doc.Revolve(cs, cpf, coilAxisV, FullRevolution{})
	require.NoError(t, err)
	s, p := coilSquare(t)
	spring, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2))
	require.NoError(t, err)

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, Interfering, rep.Status)
	require.Len(t, rep.Interferences, 1)
	overlap := rep.Interferences[0].Volume
	requireEnclosesBig(t, overlap.Value.Base(), overlap.Bound.Base(), new(big.Float).SetPrec(512).Mul(bigPi(), big.NewFloat(4.5)), "overlap")

	u, err := Union(t.Context(), core, spring)
	require.NoError(t, err)
	require.Len(t, u.Lumps(), 1)
	require.Len(t, u.Shells(), 1)
	vol, err := u.Volume()
	require.NoError(t, err)
	requireEnclosesBig(t, vol.Value.Base(), vol.Bound.Base(), new(big.Float).SetPrec(512).Mul(bigPi(), big.NewFloat(380.5)), "union volume")

	// Every reading of the union, its area included, sits inside the default
	// tolerance.
	rep, err = doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, Sound, rep.Status)
}

// TestCoilThreadRefusals pins the two refusals docs/helix-design.md §9's
// thread fixture meets today. The 60° V groove of depth 0.9 at pitch 1.5 on
// a radius-5 cylinder, 8 turns, refuses at its own build: the crossing
// audit's sweep scans more box pairs than the facet-pair ceiling allows
// (CS9). Three turns build, and Cut then refuses at the boolean's
// chain-depth gate: the groove's facets hold β near 9e-4 mm where the pair's
// chord tolerance, 2e-5 times its diameter, is near 5e-4 mm.
func TestCoilThreadRefusals(t *testing.T) {
	groove := [][2]float64{{4.1, 3}, {5.3, 3 - 0.6928}, {5.3, 3 + 0.6928}}
	gs, gp := coilLoopsSketch(t, groove)
	doc := New()
	_, err := doc.Coil(t.Context(), gs, gp, coilAxisV, units.Millimeters(1.5), units.Scalar(8))
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "candidate pair count exceeds the fixed work ceiling")

	cs, cpf := coilLoopsSketch(t, [][2]float64{{0, 0}, {5, 0}, {5, 20}, {0, 20}})
	cylinder, err := doc.Revolve(cs, cpf, coilAxisV, FullRevolution{})
	require.NoError(t, err)
	tool, err := doc.Coil(t.Context(), gs, gp, coilAxisV, units.Millimeters(1.5), units.Scalar(3))
	require.NoError(t, err)
	_, err = Cut(t.Context(), cylinder, tool)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "cut's tool holds its mesh")
	require.Len(t, doc.Bodies(), 2)
}
