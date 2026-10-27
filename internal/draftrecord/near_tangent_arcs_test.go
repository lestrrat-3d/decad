package draftrecord_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/stretchr/testify/require"
)

// This file pins agent-docs/draft-design.md §9.1.1 at corners of two arcs on
// the production mesh path: a draft of splitDisc, a radius-5 circle drawn as
// two half arcs whose second centre sits shift off the first, meshes with the
// occupied-volume proof, and its corner charge covers the area its ideal
// polyhedron's slices leave at the two near-tangent corners.

// splitDisc is the radius-5 circle about the origin drawn as two half arcs:
// the upper one about the origin from (5, 0) to (−5, 0), and the lower one
// about (0, shift) back to (5, 0). Both recorded ends of the lower arc lie
// √(25 + shift²) from its centre, so each half meets the other near tangent,
// sin θ ≈ shift/5, at both corners. A zero shift is the exactly tangent disc.
func splitDisc(shift float64) momentinput.Profile {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Center: p(0, 0), Start: p(5, 0), End: p(-5, 0), TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: p(0, shift), Start: p(-5, 0), End: p(5, 0), TStart: 0, TEnd: 1},
	}}}
}

// splitDiscShift moves the lower centre far enough that the charge is
// measurable, sin θ ≈ 2⁻³⁴, inside both the build's dead zone and the
// admitted 2⁻³².
var splitDiscShift = 5 * math.Ldexp(1, -34)

// splitDiscVolume is the closed-form volume of splitDisc(shift) tapered inward
// by d over h. At amount t the true section is circle 1 (radius r₁ = 5 − t)
// above the radical line y₀ = shift·t/(5 + R₂) and circle 2 (radius
// r₂ = R₂ − t, R₂ = √(25 + shift²), centre shift up) below it. Each part is a
// circular segment, r²·acos(y/r) − y√(r² − y²) = πr²/2 − 2ry + O(y³/r), so
// A(t) = π(r₁² + r₂²)/2 − 2r₁y₀ − 2r₂(shift − y₀) + O(shift³), and
// 2y₀(r₂ − r₁) = 2y₀(R₂ − 5) is O(shift³) too. Integrated over the sweep,
// with Q(R) = R² − Rd + d²/3, the volume is
// h[π(Q(5) + Q(R₂))/2 − 2·shift(R₂ − d/2)], up to terms below 1e-27 mm³.
func splitDiscVolume(shift float64, d *big.Float, h float64) *big.Float {
	r2 := new(big.Float).SetPrec(meshVolumePrec).Sqrt(bfSum(bf(25), bfProd(bf(shift), bf(shift))))
	q := func(r *big.Float) *big.Float {
		return bfSum(bfProd(r, r), bfNeg(bfProd(r, d)), bfQuo(bfProd(d, d), bf(3)))
	}
	round := bfQuo(bfProd(meshVolumePi, bfSum(q(bf(5)), q(r2))), bf(2))
	lift := bfProd(bf(2), bf(shift), bfSum(r2, bfNeg(bfQuo(d, bf(2)))))
	return bfProd(bf(h), bfSum(round, bfNeg(lift)))
}

// TestDraftTwoArcMeshVolumeProofCoversTheClosedForm meshes the split disc at a
// few ulps' shift (2⁻⁴⁸ mm, four ulps of 5) and at splitDiscShift, at three
// tolerances: closed, embedded, its proof admitted, and the held mesh's exact
// volume within VolSymDiff of the closed form. Shown to fail first: with the
// two-arc branch removed from capband.NearTangentCorner, every case refused
// the proof (SymDiffOK false).
func TestDraftTwoArcMeshVolumeProofCoversTheClosedForm(t *testing.T) {
	t.Parallel()
	d := bfProd(bf(8), taperTan(3))
	for _, shift := range []float64{math.Ldexp(1, -48), splitDiscShift} {
		requireVolumeProofCovers(t, meshVolumeFixture{
			name: "split disc",
			payload: func(t *testing.T) draftrecord.Payload {
				return taperPayload(t, splitDisc(shift), xyFrame(t), 8, 3, false)
			},
			volume: splitDiscVolume(shift, d, 8),
		})
	}
}

// splitDiscMeshed is the split disc meshed through the band tessellator: the
// band view, its loop meshes, the mesh, the vertex-motion store and the
// corner charge NearTangentVolume composes for it.
type splitDiscMeshed struct {
	cbp    capblend.Payload
	lms    []capblend.LoopMesh
	mesh   *topo.Mesh
	motion []float64
	charge float64
}

// splitDiscMesh meshes the split disc at shift.
func splitDiscMesh(t *testing.T, shift float64) splitDiscMeshed {
	t.Helper()
	env := topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}}
	body, err := draftrecord.Build(t.Context(), env, taperPayload(t, splitDisc(shift), xyFrame(t), 8, 3, false))
	require.NoError(t, err)
	dp, ok := body.Payload.(draftrecord.Payload)
	require.True(t, ok)
	cbp := dp.Band()
	mesh := new(topo.Mesh)
	lms, motion, err := capblend.TessellateMesh(t.Context(), body, cbp, 0.05, true, mesh)
	require.NoError(t, err)
	require.True(t, mesh.SymDiffOK)
	charge, err := capblend.NearTangentVolume(proofbound.NewWorkBudget(t.Context()), cbp, lms)
	require.NoError(t, err)
	return splitDiscMeshed{cbp: cbp, lms: lms, mesh: mesh, motion: motion, charge: charge}
}

// bfPt is a plane-local point in 256-bit arithmetic.
type bfPt struct{ u, v *big.Float }

// bfDist is |a − b|.
func bfDist(a, b bfPt) *big.Float {
	du, dv := bfSum(a.u, bfNeg(b.u)), bfSum(a.v, bfNeg(b.v))
	return new(big.Float).SetPrec(meshVolumePrec).Sqrt(bfSum(bfProd(du, du), bfProd(dv, dv)))
}

// bfArea is the area a closed polygon encloses, by the shoelace formula.
func bfArea(ps ...bfPt) float64 {
	total := bf(0)
	for i, p := range ps {
		q := ps[(i+1)%len(ps)]
		total = bfSum(total, bfProd(p.u, q.v), bfNeg(bfProd(q.u, p.v)))
	}
	out, _ := new(big.Float).Abs(bfQuo(total, bf(2))).Float64()
	return out
}

// splitDiscArc is one half of splitDisc: its centre, recorded radius, its
// window's start azimuth and span, and its chord count n.
type splitDiscArc struct {
	c         bfPt
	r         *big.Float
	th0, span float64
	n         int
}

// at is the arc's moved circle point at azimuth th; the azimuth is read in
// float64, which moves a station by about 1e-15 mm and the polygon's area by
// orders less than the charge.
func (a splitDiscArc) at(radius *big.Float, th float64) bfPt {
	return bfPt{bfSum(a.c.u, bfProd(radius, bf(math.Cos(th)))), bfSum(a.c.v, bfProd(radius, bf(math.Sin(th))))}
}

// foot is the arc's radial foot through v at amount t.
func (a splitDiscArc) foot(v bfPt, moved *big.Float) bfPt {
	s := bfQuo(moved, a.r)
	return bfPt{bfSum(a.c.u, bfProd(s, bfSum(v.u, bfNeg(a.c.u)))), bfSum(a.c.v, bfProd(s, bfSum(v.v, bfNeg(a.c.v))))}
}

// twoArcSliceArea is the area Lemma 2 of agent-docs/draft-design.md §9.1.1
// charges at one corner of two arcs, prev arriving at v and cur leaving it, at
// level frac of the full amount d: the larger over the two diagonals of
// prev's last cell of the polygon P → q → f₂ → f₁, plus the curvilinear
// triangle m f₁ f₂ (its straight triangle and an upper bound c³/(6r) on each
// arc's segment). m is where the two moved circles meet nearest f₂.
func twoArcSliceArea(prev, cur splitDiscArc, v bfPt, d *big.Float, frac float64) float64 {
	sqrt := func(a *big.Float) *big.Float { return new(big.Float).SetPrec(meshVolumePrec).Sqrt(a) }
	s := bf(frac)
	t := bfProd(s, d)
	lerp := func(a, b bfPt) bfPt {
		w := bfSum(bf(1), bfNeg(s))
		return bfPt{bfSum(bfProd(w, a.u), bfProd(s, b.u)), bfSum(bfProd(w, a.v), bfProd(s, b.v))}
	}
	r1, r2 := bfSum(prev.r, bfNeg(t)), bfSum(cur.r, bfNeg(t))
	f1, f2 := prev.foot(v, r1), cur.foot(v, r2)
	th := prev.th0 + prev.span*float64(prev.n-1)/float64(prev.n)
	side, capPt := prev.at(prev.r, th), prev.at(bfSum(prev.r, bfNeg(d)), th)
	p := prev.at(r1, th)
	capFoot := cur.foot(v, bfSum(cur.r, bfNeg(d)))
	twist := math.Max(bfArea(p, lerp(side, capFoot), f2, f1), bfArea(p, lerp(v, capPt), f2, f1))

	wu, wv := bfSum(cur.c.u, bfNeg(prev.c.u)), bfSum(cur.c.v, bfNeg(prev.c.v))
	dist := sqrt(bfSum(bfProd(wu, wu), bfProd(wv, wv)))
	a := bfQuo(bfSum(bfProd(dist, dist), bfProd(r1, r1), bfNeg(bfProd(r2, r2))), bfProd(bf(2), dist))
	h := sqrt(bfSum(bfProd(r1, r1), bfNeg(bfProd(a, a))))
	base := bfPt{bfSum(prev.c.u, bfQuo(bfProd(a, wu), dist)), bfSum(prev.c.v, bfQuo(bfProd(a, wv), dist))}
	off := bfPt{bfQuo(bfProd(h, wv), dist), bfQuo(bfProd(h, wu), dist)}
	m := bfPt{bfSum(base.u, bfNeg(off.u)), bfSum(base.v, off.v)}
	if other := (bfPt{bfSum(base.u, off.u), bfSum(base.v, bfNeg(off.v))}); bfDist(other, f2).Cmp(bfDist(m, f2)) < 0 {
		m = other
	}
	segment := func(c, radius *big.Float) float64 {
		cc, _ := c.Float64()
		rr, _ := radius.Float64()
		return cc * cc * cc / (6 * rr)
	}
	return twist + bfArea(m, f1, f2) + segment(bfDist(m, f1), r1) + segment(bfDist(m, f2), r2)
}

// TestDraftTwoArcMeshChargesTheCorner pins the charge at corners of two arcs.
// The exactly tangent disc charges nothing, so its proof is the sum it was;
// the shifted disc charges a positive volume, its VolSymDiff exceeds the
// exact disc's, and at every eighth of the sweep its charge per unit height
// covers the area Lemma 2 says the ideal polyhedron's slice leaves at its two
// corners: the twisted last cell of each half and the curvilinear triangle
// at the true corner.
//
// Shown to fail first: with the ρλ term dropped from NearTangentVolume, the
// charge per unit height (the 2.08ρ² terms, 2.5e-21 mm²) fell below the
// slice area from the first eighth of the sweep (9.7e-13 mm²). Dropping the
// 2.08ρ² term leaves no fixture: the curvilinear triangle it covers is at
// most 1.3e-21 mm² per corner here, far under ρλ.
func TestDraftTwoArcMeshChargesTheCorner(t *testing.T) {
	t.Parallel()
	exact := splitDiscMesh(t, 0)
	require.Zero(t, exact.charge, "an exactly tangent disc carries no near-tangent charge")
	near := splitDiscMesh(t, splitDiscShift)
	charge, lms := near.charge, near.lms
	require.Positive(t, charge)
	require.Greater(t, near.mesh.VolSymDiff, exact.mesh.VolSymDiff, "the split disc's proof charges more than the exact one's")
	require.Len(t, lms, 1)
	require.Len(t, lms[0].Walks, 2)
	counts := map[int]int{}
	for i, w := range lms[0].Walks {
		counts[w.Segs[0]] = lms[0].Count[i]
	}
	r2 := new(big.Float).SetPrec(meshVolumePrec).Sqrt(bfSum(bf(25), bfProd(bf(splitDiscShift), bf(splitDiscShift))))
	upper := splitDiscArc{c: bfPt{bf(0), bf(0)}, r: bf(5), th0: 0, span: math.Pi, n: counts[0]}
	lowerStart := math.Atan2(-splitDiscShift, -5)
	lower := splitDiscArc{c: bfPt{bf(0), bf(splitDiscShift)}, r: r2, th0: lowerStart,
		span: math.Atan2(-splitDiscShift, 5) - lowerStart, n: counts[1]}
	perHeight := charge / near.cbp.DraftBandHeightUpper()
	d := bfProd(bf(8), taperTan(3))
	for k := 1; k <= 8; k++ {
		frac := float64(k) / 8
		got := twoArcSliceArea(upper, lower, bfPt{bf(-5), bf(0)}, d, frac) +
			twoArcSliceArea(lower, upper, bfPt{bf(5), bf(0)}, d, frac)
		require.Positive(t, got)
		require.LessOrEqualf(t, got, perHeight, "level %g: the slice leaves %g mm² past the charge %g", frac, got, perHeight)
	}
}

// TestDraftTwoArcChargeJoinsTheProofSum pins the composition at corners of
// two arcs: the split disc's VolSymDiff is at least the 256-bit sum of the
// chord term, the vertex-motion term and the corner charge.
//
// Shown to fail first: with the charge left out of the sum in
// capblend.TessellateMesh, VolSymDiff equalled the two-term sum and fell
// short of it by the charge.
func TestDraftTwoArcChargeJoinsTheProofSum(t *testing.T) {
	t.Parallel()
	got := splitDiscMesh(t, splitDiscShift)
	motionMax, err := tessellation.StoreMax(got.motion)
	require.NoError(t, err)
	base, err := tessellation.SymDiff([]float64{
		capblend.ChordVolume(got.cbp, got.lms),
		proofbound.SweptVolumeAllow(motionMax, proofbound.PerturbedAreaUpper(got.mesh.Vertices, got.mesh.Triangles, motionMax)),
	})
	require.NoError(t, err)
	require.Positive(t, got.charge)
	want := bfSum(bf(base), bf(got.charge))
	require.GreaterOrEqualf(t, bf(got.mesh.VolSymDiff).Cmp(want), 0,
		"VolSymDiff %v is below the chord and motion terms %v plus the charge %g", got.mesh.VolSymDiff, base, got.charge)
}
