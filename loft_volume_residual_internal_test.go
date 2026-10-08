package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file tests docs/loft-gear-bounds-design.md §2's volume residual —
// sweptLeg + wallLeg + skirtLeg — and §4's per-cell cap tube, over the
// production path: loftmesh.ValidateLoftRecords, loftmesh.PairRecords, assembleLoft and
// buildLoftMass, the same steps evalLoft runs, at a chord target the test
// chooses.

// loftMassBuild is one build's accumulator and the inputs it was folded from.
type loftMassBuild struct {
	mass                *loftMassAccumulator
	a                   loftAssembly
	pairs               []loftmesh.LoopPair
	sectionDelta        float64
	sectionMatchedDelta float64
}

// matchedDelta is buildLoftMass's own composition of §5.2's matchedDelta row.
func (b loftMassBuild) matchedDelta() float64 {
	if b.sectionDelta <= 0 && b.sectionMatchedDelta <= 0 {
		return 0
	}
	return loftmesh.ChordCellDeltaUpper(b.sectionMatchedDelta, b.a.delta)
}

// stationsPerLoop is the number of wall cells on the outer loop.
func (b loftMassBuild) stationsPerLoop() int { return len(b.pairs[0].V) }

// loftMassAtTarget runs evalLoft's own record, pairing, assembly and mass
// steps at chord target target; a target of 0 reads the production target.
func loftMassAtTarget(t *testing.T, pl loftPayload, target float64) loftMassBuild {
	t.Helper()
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	offsets, walks0, walks1, production, err := loftmesh.ValidateLoftRecords(pl.profile0, pl.profile1, pl.plane0, pl.plane1, pl.alignment, pl.recordArea, work0, work1)
	require.NoError(t, err)
	if target <= 0 {
		target = production
	}
	pairs, sectionDelta, sectionMatchedDelta, stationRound, err := loftmesh.PairRecords(pl.profile0, pl.profile1, offsets, walks0, walks1, target, work0, work1)
	require.NoError(t, err)
	a, err := assembleLoft(t.Context(), pairs, pl.frame0, pl.frame1, pl.plane0, pl.xform, stationRound)
	require.NoError(t, err)
	return loftMassBuild{
		mass: buildLoftMass(pl, a, pairs, sectionDelta, sectionMatchedDelta), a: a, pairs: pairs,
		sectionDelta: sectionDelta, sectionMatchedDelta: sectionMatchedDelta,
	}
}

// twoRadiusOvalProfile is a convex loop of two straight sides, a radius-3
// semicircle and a radius-5 arc, every recorded point exact and both arcs of
// exactly equal end radii, so the arcs' chord departures differ from one
// another and an untrimmed one-chord build publishes delta == 0.
func twoRadiusOvalProfile() ProfileRecord {
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: pt(-4, -3), End: pt(4, -3), TStart: 0, TEnd: 1},
		ArcSeg{Center: pt(4, 0), Start: pt(4, -3), End: pt(4, 3), TStart: 0, TEnd: 1},
		LineSeg{Start: pt(4, 3), End: pt(-4, 3), TStart: 0, TEnd: 1},
		ArcSeg{Center: pt(0, 0), Start: pt(-4, 3), End: pt(-4, -3), TStart: 0, TEnd: 1},
	}}}
}

// quarterRingProfile is a radius-5 circle as four quarter arcs starting at the
// exact on-circle point (5·c, 5·s): (c, s) = (1, 0) gives the axis points, and
// (3/5, 4/5) the same circle turned by atan(4/3), its points still exact.
func quarterRingProfile(c, s float64) ProfileRecord {
	corners := []Point2{pt(5*c, 5*s), pt(-5*s, 5*c), pt(-5*c, -5*s), pt(5*s, -5*c)}
	segs := make([]CurveSegment, 4)
	for i := range corners {
		segs[i] = ArcSeg{Center: pt(0, 0), Start: corners[i], End: corners[(i+1)%4], TStart: 0, TEnd: 1}
	}
	return ProfileRecord{Outer: LoopRecord{Segments: segs}}
}

func loftPayloadOnPlanes(t *testing.T, p0, p1 ProfileRecord, pl0, pl1 PlaneRecord) loftPayload {
	t.Helper()
	return loftPayload{
		profile0: p0, profile1: p1, plane0: pl0, plane1: pl1,
		frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1),
		xform:      r3.Identity(),
		recordArea: loftRecordAreasOrZero(p0, p1),
	}
}

// testPlacement is a rigid motion whose rotation rounds every coordinate.
func testPlacement(t *testing.T) r3.Transform {
	t.Helper()
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(12, -5, 3))
	require.NoError(t, err)
	xform, err := rot.Then(shift)
	require.NoError(t, err)
	return xform
}

// heldRulingVolume is the exact rational Volume.Value holds: the held
// tetrahedron sum plus the exact twist correction.
func heldRulingVolume(m *loftMassAccumulator) *big.Rat {
	vol := new(big.Rat).Quo(m.Vol6, big.NewRat(6, 1))
	if m.Chorded.TwistVolumeCorrection != nil {
		vol.Add(vol, m.Chorded.TwistVolumeCorrection)
	}
	return vol
}

// requireEnclosesRat asserts [value − bound, value + bound] ⊇ [lo, hi], all
// read as exact rationals.
func requireEnclosesRat(t *testing.T, value, bound float64, lo, hi *big.Rat, msg string) {
	t.Helper()
	v, b := proofarith.FloatRat(value), proofarith.FloatRat(bound)
	require.NotNil(t, v)
	require.NotNil(t, b)
	low := new(big.Rat).Sub(v, b)
	high := new(big.Rat).Add(v, b)
	require.LessOrEqualf(t, low.Cmp(lo), 0, "%s: published low end %s above the true low end %s", msg, low.FloatString(20), lo.FloatString(20))
	require.GreaterOrEqualf(t, high.Cmp(hi), 0, "%s: published high end %s below the true high end %s", msg, high.FloatString(20), hi.FloatString(20))
}

// piBracket returns k·π enclosed as [k·π⁻, k·π⁺], π⁻ = math.Pi (which lies
// below π) and π⁺ the next float up.
func piBracket(k *big.Rat) (*big.Rat, *big.Rat) {
	lo := new(big.Rat).Mul(k, proofarith.FloatRat(math.Pi))
	hi := new(big.Rat).Mul(k, proofarith.FloatRat(math.Nextafter(math.Pi, math.Inf(1))))
	return lo, hi
}

// TestLoftVolumeResidualIsPerCellWallLegPlusSkirt pins Volume.Bound's
// composition on an oval prism whose two arcs depart by different amounts:
// the rounding of the exact ruled volume plus one allowance composing §2's
// vertex sweep where delta > 0, the per-cell wall leg and the skirt, the last
// two recomputed here cell by cell, the skirt's perimeter over every held
// seam cell. The skirt is exactly 0 at delta == 0 (one
// chord per arc, every station pinned at an exact end) and positive on the
// placed copy, and the per-cell wall leg is strictly below the build-wide
// matchedDelta × wallAreaUpper form.
//
// Shown to fail: dropping SkirtLeg from MassAccumulator.Volume turns the
// placed row red (the published bound loses the skirt's bits), and charging
// productUpper(matchedDelta, WallAreaUpper) in place of WallLeg turns every
// row red on the bound comparison and the strict per-cell check.
func TestLoftVolumeResidualIsPerCellWallLegPlusSkirt(t *testing.T) {
	t.Parallel()
	oval := twoRadiusOvalProfile()
	unplaced := loftPayloadOnPlanes(t, oval, oval, planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 10)))
	placed := unplaced
	placed.xform = testPlacement(t)

	for _, tc := range []struct {
		name      string
		pl        loftPayload
		target    float64
		zeroDelta bool
	}{
		{"unplaced, one chord per arc", unplaced, 100, true},
		{"unplaced, production target", unplaced, 0, false},
		{"placed copy, production target", placed, 0, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			b := loftMassAtTarget(t, tc.pl, tc.target)
			m, a := b.mass, b.a
			require.Positive(t, b.sectionDelta, "both arcs are chorded")
			if tc.zeroDelta {
				require.Zero(t, a.delta, "every station is a pinned exact end")
			} else {
				require.Positive(t, a.delta)
			}
			if tc.target == 0 {
				body := evalLoftFixture(t, tc.pl)
				require.Equal(t, body.volume, m.volume(a.verts, a.tris), "the helper reads the build evalLoft publishes")
			}

			// The per-cell wall leg and the every-cell seam perimeter, in the
			// order ComputeLoftChordedAllow walks the cells.
			var wallLeg, perimeter float64
			for i, p := range b.pairs {
				n := len(p.V)
				for j := range n {
					jn := (j + 1) % n
					vLo, vHi := a.verts[a.vIdx[i][j]], a.verts[a.vIdx[i][jn]]
					wLo, wHi := a.verts[a.wIdx[i][j]], a.verts[a.wIdx[i][jn]]
					perimeter = proofbound.AbsSumUpper(perimeter,
						math.Max(p.ArcUpperV[j], proofbound.CellSpanUpper(vLo, vHi)),
						math.Max(p.ArcUpperW[j], proofbound.CellSpanUpper(wLo, wHi)),
					)
					if p.Faceted[j] && p.MatchedDelta[j] <= 0 {
						continue
					}
					cellMatched := loftmesh.ChordCellDeltaUpper(p.MatchedDelta[j], a.delta)
					cellWall := proofbound.CellChordCurveAreaUpper(vLo, vHi, wLo, wHi, p.ArcUpperV[j], p.ArcUpperW[j], cellMatched)
					wallLeg = proofbound.AbsSumUpper(wallLeg, proofbound.ProductUpper(cellMatched, cellWall))
				}
			}
			skirt := 0.0
			if a.delta > 0 {
				skirt = proofbound.ProductUpper(proofbound.ProductUpper(b.matchedDelta(), a.delta), perimeter)
			}
			require.Equal(t, wallLeg, m.Chorded.WallLeg)
			require.Equal(t, skirt, m.Chorded.SkirtLeg)
			if tc.zeroDelta {
				require.Zero(t, m.Chorded.SkirtLeg, "the held seam lies in its cap plane")
			} else {
				require.Positive(t, m.Chorded.SkirtLeg, "a displaced seam leaves its cap plane")
			}
			require.Less(t, m.Chorded.WallLeg, proofbound.ProductUpper(b.matchedDelta(), m.Chorded.WallAreaUpper),
				"each cell is charged at its own departure, below the worst cell's")

			vol := heldRulingVolume(m)
			value, _ := vol.Float64()
			allow := 0.0
			if a.delta > 0 {
				allow = proofbound.SweptVolumeAllow(a.delta, proofbound.PerturbedAreaUpper(a.verts, a.tris, a.delta))
			}
			allow = proofbound.AbsSumUpper(allow, wallLeg, skirt)
			want := proofbound.AbsSumUpper(proofarith.RationalFloatError(vol, value), allow)
			got := m.volume(a.verts, a.tris)
			require.Equal(t, value, got.Value.Base())
			require.Equal(t, math.Float64bits(want), math.Float64bits(got.Bound.Base()),
				"Volume.Bound is rounding + sweptLeg + wallLeg + skirtLeg (want %g, got %g)", want, got.Bound.Base())
		})
	}
}

// TestLoftVolumeBoundEnclosesRefinedRing asserts the exact volume of a ruled
// ring loft lies inside [Value − Bound, Value + Bound] at station counts from
// 4 to 64 per quarter arc, untwisted and twisted. The twisted top is the
// same radius-5 circle turned by φ = atan(4/3): uniform-angle correspondence
// joins the bottom point at θ to the top point at θ + φ, every height's
// section is a circle of radius 5·|(1 − f) + f·e^{iφ}|, and the volume is
// π·25·10·(2 + cos φ)/3 = 650π/3; the untwisted ring is the cylinder 250π.
//
// Shown to fail: deleting WallLeg from MassAccumulator.Volume fails every
// row, twisted and untwisted.
func TestLoftVolumeBoundEnclosesRefinedRing(t *testing.T) {
	t.Parallel()
	bottom := quarterRingProfile(1, 0)
	for _, tc := range []struct {
		name string
		top  ProfileRecord
		k    *big.Rat
	}{
		{"untwisted", quarterRingProfile(1, 0), big.NewRat(250, 1)},
		{"twisted", quarterRingProfile(0.6, 0.8), big.NewRat(650, 3)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			pl := loftPayloadOnPlanes(t, bottom, tc.top, planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 10)))
			lo, hi := piBracket(tc.k)
			seen := map[int]struct{}{}
			for _, m := range []int{3, 4, 6, 8, 12, 16, 24, 32, 48, 64} {
				// A quarter arc chorded m times has sagitta 5·(1 − cos(π/(4m)));
				// the settled count lands at or just above m.
				target := 5 * (1 - math.Cos(math.Pi/float64(4*m))) * 1.001
				b := loftMassAtTarget(t, pl, target)
				perArc := b.stationsPerLoop() / 4
				seen[perArc] = struct{}{}
				got := b.mass.volume(b.a.verts, b.a.tris)
				requireEnclosesRat(t, got.Value.Base(), got.Bound.Base(), lo, hi, tc.name)
				t.Logf("%s m=%d: value=%.12g bound=%.4g", tc.name, perArc, got.Value.Base(), got.Bound.Base())
			}
			fewest, most := math.MaxInt, 0
			for n := range seen {
				fewest, most = min(fewest, n), max(most, n)
			}
			require.LessOrEqual(t, fewest, 4)
			require.GreaterOrEqual(t, most, 64)
		})
	}
}

// TestLoftAreaCapTubeIsPerCell pins §4's per-cell cap tube: CapAreaExcess is
// Σ over both caps and every charged cell of 2·cellMatched_k·arcLenUpper_k +
// π⁺·cellMatched_k², recomputed here cell by cell. On the untwisted ring every
// cell departs alike and the tube matches the build-wide
// proofbound.SectionDisplacementArea form to rounding; on the two-radius oval
// it is strictly below it.
//
// Shown to fail: charging SectionDisplacementArea(matchedDelta, walks,
// perimeter) per cap in place of the per-cell tube turns the oval row red.
func TestLoftAreaCapTubeIsPerCell(t *testing.T) {
	t.Parallel()
	ring := quarterRingProfile(1, 0)
	oval := twoRadiusOvalProfile()
	for _, tc := range []struct {
		name    string
		pl      loftPayload
		uniform bool
	}{
		{"uniform ring", loftPayloadOnPlanes(t, ring, ring, planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 10))), true},
		{"two-radius oval", loftPayloadOnPlanes(t, oval, oval, planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 10))), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			b := loftMassAtTarget(t, tc.pl, 0)
			piUp := math.Nextafter(math.Pi, math.Inf(1))
			var tube, perimV, perimW float64
			walks := 0
			for _, p := range b.pairs {
				for j := range p.V {
					if p.Faceted[j] && p.MatchedDelta[j] <= 0 {
						continue
					}
					walks++
					perimV = proofbound.AbsSumUpper(perimV, p.ArcUpperV[j])
					perimW = proofbound.AbsSumUpper(perimW, p.ArcUpperW[j])
					cellMatched := loftmesh.ChordCellDeltaUpper(p.MatchedDelta[j], b.a.delta)
					joint := proofbound.ProductUpper(piUp, proofbound.ProductUpper(cellMatched, cellMatched))
					twice := proofbound.ProductUpper(2, cellMatched)
					tube = proofbound.AbsSumUpper(tube,
						proofbound.ProductUpper(twice, p.ArcUpperV[j]), joint,
						proofbound.ProductUpper(twice, p.ArcUpperW[j]), joint)
				}
			}
			require.Equal(t, tube, b.mass.Chorded.CapAreaExcess)
			maxForm := proofbound.AbsSumUpper(
				proofbound.SectionDisplacementArea(b.matchedDelta(), walks, perimV),
				proofbound.SectionDisplacementArea(b.matchedDelta(), walks, perimW),
			)
			if tc.uniform {
				require.InEpsilon(t, maxForm, tube, 1e-9, "equal departures read the maximum form")
				return
			}
			require.Less(t, tube, maxForm, "each cell is charged at its own departure, below the worst cell's")
			t.Logf("per-cell tube %.6g against maximum form %.6g", tube, maxForm)
		})
	}
}

// TestLoftPlacedVolumeNeedsTheVertexSweep is the placed loft §2's step 1 and
// its faceted cells need sweptVolumeAllow for: a 10 × 10 slab 2^-10 thick
// whose one corner is a radius-2^-24 arc, placed by a rotation that rounds
// every coordinate. The build is chorded, so its wall leg and skirt are both
// present, but the arc is so small that both together, with the value's own
// rounding, sit far below the volume the placement's rounding moved.
//
// The exact volume is h·(100 − r² + π·r²/4), a rigid motion preserving it;
// the test asserts it lies inside Volume's bound and OUTSIDE the bound with
// the vertex sweep removed.
//
// Shown to fail: deleting the m.delta > 0 sweptVolumeAllow term from
// MassAccumulator.Volume turns the enclosure assertion red.
func TestLoftPlacedVolumeNeedsTheVertexSweep(t *testing.T) {
	t.Parallel()
	r := math.Ldexp(1, -24)
	h := math.Ldexp(1, -10)
	slab := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: pt(0, 0), End: pt(10, 0), TStart: 0, TEnd: 1},
		LineSeg{Start: pt(10, 0), End: pt(10, 10-r), TStart: 0, TEnd: 1},
		ArcSeg{Center: pt(10-r, 10-r), Start: pt(10, 10-r), End: pt(10-r, 10), TStart: 0, TEnd: 1},
		LineSeg{Start: pt(10-r, 10), End: pt(0, 10), TStart: 0, TEnd: 1},
		LineSeg{Start: pt(0, 10), End: pt(0, 0), TStart: 0, TEnd: 1},
	}}}
	pl := loftPayloadOnPlanes(t, slab, slab, planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, h)))
	pl.xform = testPlacement(t)

	b := loftMassAtTarget(t, pl, 0)
	m := b.mass
	require.Positive(t, b.sectionDelta, "the corner arc is chorded")
	require.Positive(t, b.a.delta, "the placement rounds the held vertices")
	body := evalLoftFixture(t, pl)
	got := body.volume
	require.Equal(t, got, m.volume(b.a.verts, b.a.tris))

	// V = h·(100 − r²·(1 − π/4)), enclosed through π's bracket.
	rr := new(big.Rat).Mul(proofarith.FloatRat(r), proofarith.FloatRat(r))
	hr := proofarith.FloatRat(h)
	quarterLo, quarterHi := piBracket(big.NewRat(1, 4))
	lo := new(big.Rat).Mul(hr, new(big.Rat).Sub(big.NewRat(100, 1), new(big.Rat).Mul(rr, new(big.Rat).Sub(big.NewRat(1, 1), quarterLo))))
	hi := new(big.Rat).Mul(hr, new(big.Rat).Sub(big.NewRat(100, 1), new(big.Rat).Mul(rr, new(big.Rat).Sub(big.NewRat(1, 1), quarterHi))))
	requireEnclosesRat(t, got.Value.Base(), got.Bound.Base(), lo, hi, "placed slab")

	// Without the sweep the bound is the rounding, the wall leg and the skirt.
	vol := heldRulingVolume(m)
	value, _ := vol.Float64()
	withoutSweep := proofbound.AbsSumUpper(proofarith.RationalFloatError(vol, value), m.Chorded.WallLeg, m.Chorded.SkirtLeg)
	v := proofarith.FloatRat(value)
	miss := new(big.Rat).Sub(lo, v)
	if over := new(big.Rat).Sub(v, hi); over.Cmp(miss) > 0 {
		miss = over
	}
	missF, _ := miss.Float64()
	require.Greaterf(t, missF, withoutSweep,
		"the placement moved the volume by %g, past the %g the chorded legs and rounding cover", missF, withoutSweep)
	t.Logf("placement moved the volume by %.3g; bound without the sweep %.3g, with it %.3g", missF, withoutSweep, got.Bound.Base())
}
