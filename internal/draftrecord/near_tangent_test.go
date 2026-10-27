package draftrecord_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/stretchr/testify/require"
)

// This file pins agent-docs/draft-design.md §9.1.1 on the production mesh
// path: a draft of nearTangentSlot, whose right arc meets both lines a sliver
// off tangent, meshes with the occupied-volume proof, and its near-tangent
// corner charge covers the area its ideal polyhedron's slices leave between
// themselves and the true sections.

// nearTangentShift moves the slot's right centre far enough that the charge
// is measurable, sin θ ≈ 2⁻³⁴, inside both the build's dead zone and the
// admitted 2⁻³².
var nearTangentShift = 5 * math.Ldexp(1, -34)

// nearTangentSlotVolume is the closed-form volume of nearTangentSlot(shift)
// tapered inward by d over h: each true section is the stadium whose straight
// length is 30 + shift and whose radius is 5 − t, up to terms of order shift²
// (below 1e-17 mm³ here), so the volume is h[π(25 − 5d + d²/3) + 2(30 +
// shift)(5 − d/2)].
func nearTangentSlotVolume(shift float64, d *big.Float, h float64) *big.Float {
	disk := bfProd(meshVolumePi, bfSum(bf(25), bfProd(bf(-5), d), bfQuo(bfProd(d, d), bf(3))))
	band := bfProd(bf(2), bfSum(bf(30), bf(shift)), bfSum(bf(5), bfQuo(bfNeg(d), bf(2))))
	return bfProd(bf(h), bfSum(disk, band))
}

// TestDraftNearTangentMeshVolumeProofCoversTheClosedForm meshes the slot at
// its two shifts and three tolerances: closed, embedded, its proof admitted,
// and the held mesh's exact volume within VolSymDiff of the closed form.
// Removing the near-tangent arm from capband.OccupiedVolumeAdmission makes
// every case refuse the proof (SymDiffOK false).
func TestDraftNearTangentMeshVolumeProofCoversTheClosedForm(t *testing.T) {
	t.Parallel()
	d := bfProd(bf(8), taperTan(3))
	for _, shift := range []float64{math.Ldexp(1, -40), nearTangentShift} {
		requireVolumeProofCovers(t, meshVolumeFixture{
			name: "near-tangent slot",
			payload: func(t *testing.T) draftrecord.Payload {
				return taperPayload(t, nearTangentSlot(shift), xyFrame(t), 8, 3, false)
			},
			volume: nearTangentSlotVolume(shift, d, 8),
		})
	}
}

// slotCornerMesh meshes the slot at shift and returns the band view, its
// mesh and the corner charge NearTangentVolume composes for it.
func slotCornerMesh(t *testing.T, shift float64) (capblend.Payload, *topo.Mesh, float64) {
	t.Helper()
	env := topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}}
	body, err := draftrecord.Build(t.Context(), env, taperPayload(t, nearTangentSlot(shift), xyFrame(t), 8, 3, false))
	require.NoError(t, err)
	dp, ok := body.Payload.(draftrecord.Payload)
	require.True(t, ok)
	cbp := dp.Band()
	mesh := new(topo.Mesh)
	lms, _, err := capblend.TessellateMesh(t.Context(), body, cbp, 0.05, true, mesh)
	require.NoError(t, err)
	require.True(t, mesh.SymDiffOK)
	charge, err := capblend.NearTangentVolume(proofbound.NewWorkBudget(t.Context()), cbp, lms)
	require.NoError(t, err)
	return cbp, mesh, charge
}

// nearTangentSliceArea is the area §9.1.1's Lemma 2 charges at one level of
// the slot, amount t of the full d, in 256-bit arithmetic: twice (the slot is
// symmetric about v = 0) the bottom line's path f₁ → q → f₂ → m₂ → f₁, its
// left end exactly tangent, taken over either diagonal of the line's cell,
// plus the circular segment between f₂ and m₂.
func nearTangentSliceArea(shift float64, d *big.Float, frac float64) float64 {
	sqrt := func(a *big.Float) *big.Float { return new(big.Float).SetPrec(meshVolumePrec).Sqrt(a) }
	type pt struct{ u, v *big.Float }
	lerp := func(a, b pt, s *big.Float) pt {
		w := bfSum(bf(1), bfNeg(s))
		return pt{bfSum(bfProd(w, a.u), bfProd(s, b.u)), bfSum(bfProd(w, a.v), bfProd(s, b.v))}
	}
	area := func(ps ...pt) float64 {
		total := bf(0)
		for i, p := range ps {
			q := ps[(i+1)%len(ps)]
			total = bfSum(total, bfProd(p.u, q.v), bfNeg(bfProd(q.u, p.v)))
		}
		out, _ := new(big.Float).Abs(bfQuo(total, bf(2))).Float64()
		return out
	}
	s, f := bf(shift), bf(frac)
	radius := sqrt(bfSum(bf(25), bfProd(s, s)))
	t := bfProd(f, d)
	foot := func(amount *big.Float) pt {
		return pt{bfSum(bf(15), bfQuo(bfProd(amount, s), radius)), bfSum(bf(-5), bfQuo(bfProd(amount, bf(5)), radius))}
	}
	a, b := pt{bf(-15), bf(-5)}, pt{bf(15), bf(-5)}
	capA, capB := pt{bf(-15), bfSum(bf(-5), d)}, foot(d)
	f1, f2 := lerp(a, capA, f), foot(t)
	moved, half := bfSum(radius, bfNeg(t)), bfSum(bf(5), bfNeg(t))
	m2 := pt{bfSum(bf(15), s, bfNeg(sqrt(bfSum(bfProd(moved, moved), bfNeg(bfProd(half, half)))))), bfSum(t, bf(-5))}
	path := math.Max(area(f1, lerp(a, capB, f), f2, m2), area(f1, lerp(b, capA, f), f2, m2))
	du, dv := bfSum(f2.u, bfNeg(m2.u)), bfSum(f2.v, bfNeg(m2.v))
	chord, _ := sqrt(bfSum(bfProd(du, du), bfProd(dv, dv))).Float64()
	r, _ := moved.Float64()
	segment := chord * chord * chord / (6 * r)
	return 2 * (path + segment)
}

// TestDraftNearTangentMeshChargesTheCorner pins the charge itself. The exactly
// tangent slot charges nothing, so its proof is the sum it was; the shifted
// slot charges a positive volume, its VolSymDiff exceeds the exact slot's,
// and at every level its charge per unit height covers the area Lemma 2 says
// the ideal polyhedron's slice leaves at the two near-tangent corners.
//
// Shown to fail first: with the strip term 3ελ dropped from NearTangentVolume,
// the charge per unit height (9.4e-22, the two discs) fell below the slice
// area from the first level on (5.0e-21 at an eighth of the sweep). The disc
// term's segment sits below 1e-33 mm² here, so no slice area pins it;
// TestDraftNearTangentChargeCoversEveryTerm pins its place in the sum.
func TestDraftNearTangentMeshChargesTheCorner(t *testing.T) {
	t.Parallel()
	_, exact, exactCharge := slotCornerMesh(t, 0)
	require.Zero(t, exactCharge, "an exactly tangent slot carries no near-tangent charge")
	cbp, near, charge := slotCornerMesh(t, nearTangentShift)
	require.Positive(t, charge)
	require.Greater(t, near.VolSymDiff, exact.VolSymDiff, "the near-tangent slot's proof charges more than the exact one's")
	perHeight := charge / cbp.DraftBandHeightUpper()
	d := bfProd(bf(8), taperTan(3))
	for k := 1; k <= 8; k++ {
		frac := float64(k) / 8
		got := nearTangentSliceArea(nearTangentShift, d, frac)
		require.Positive(t, got)
		require.LessOrEqualf(t, got, perHeight, "level %g: the slice leaves %g mm² past the charge %g", frac, got, perHeight)
	}
}

// TestDraftNearTangentChargeJoinsTheProofSum pins the composition: the
// shifted slot's VolSymDiff is at least the 256-bit sum of the chord term, the
// vertex-motion term and the corner charge. AbsSumUpper rounds up at every
// add, so an appended positive term raises the sum even where it sits far
// below the chord term's last place.
//
// Shown to fail first: with the charge left out of the sum in
// capblend.TessellateMesh, VolSymDiff equalled the two-term sum and fell
// short of it by the charge.
func TestDraftNearTangentChargeJoinsTheProofSum(t *testing.T) {
	t.Parallel()
	env := topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}}
	body, err := draftrecord.Build(t.Context(), env, taperPayload(t, nearTangentSlot(nearTangentShift), xyFrame(t), 8, 3, false))
	require.NoError(t, err)
	dp, ok := body.Payload.(draftrecord.Payload)
	require.True(t, ok)
	cbp := dp.Band()
	mesh := new(topo.Mesh)
	lms, motion, err := capblend.TessellateMesh(t.Context(), body, cbp, 0.05, true, mesh)
	require.NoError(t, err)
	require.True(t, mesh.SymDiffOK)
	motionMax, err := tessellation.StoreMax(motion)
	require.NoError(t, err)
	base, err := tessellation.SymDiff([]float64{
		capblend.ChordVolume(cbp, lms),
		proofbound.SweptVolumeAllow(motionMax, proofbound.PerturbedAreaUpper(mesh.Vertices, mesh.Triangles, motionMax)),
	})
	require.NoError(t, err)
	charge, err := capblend.NearTangentVolume(proofbound.NewWorkBudget(t.Context()), cbp, lms)
	require.NoError(t, err)
	require.Positive(t, charge)
	want := bfSum(bf(base), bf(charge))
	require.GreaterOrEqualf(t, bf(mesh.VolSymDiff).Cmp(want), 0,
		"VolSymDiff %v is below the chord and motion terms %v plus the charge %g", mesh.VolSymDiff, base, charge)
}
