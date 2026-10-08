package apitest_test

import (
	"fmt"
	"math"
	"math/big"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers docs/modify-reach-design.md §8.3.1: a two-distance chamfer
// of complete prism cap loops. The reference face picks, per cap, whether the
// positional distance runs across the cap (dc) or down the side wall (ds).

// endCapFace references the receiver's end cap face.
func endCapFace(b *decad.Body) *decad.FaceQuery {
	return decad.Faces(decad.FaceCreatedBy(decad.CapEnd(b)))
}

// boxSideWalls references the four side walls of a filletBox-shaped receiver.
func boxSideWalls() *decad.FaceQuery {
	return decad.Faces(decad.NormalTo(r3.NewVec(1, 0, 0))).Or(decad.NormalTo(r3.NewVec(0, 1, 0)))
}

// asymChamfer chamfers sel on b with d along the reference faces and other
// along the faces beside them.
func asymChamfer(t *testing.T, b *decad.Body, sel decad.EdgeSelector, d, other float64, ref decad.FaceSelector) (*decad.Body, error) {
	t.Helper()
	return b.Chamfer(t.Context(), sel, units.Millimeters(d), decad.WithAsymmetricChamfer(ref, units.Millimeters(other)))
}

// boxCapBandVolume is the 100x60x20 plate's volume after one cap loop is set back
// dc across the cap and ds down the side: the straight slab plus the band, the
// eroded section's area integrated over the band. At axial fraction s the
// section is the rectangle eroded by s·dc, and the band is ds tall, so the band
// holds ds·∫₀¹ (L − 2s·dc)(W − 2s·dc) ds = ds·(LW − (L+W)·dc + 4/3·dc²).
func boxCapBandVolume(dc, ds float64) float64 {
	const L, W, h = 100.0, 60.0, filletBoxHeight
	return L*W*(h-ds) + ds*(L*W-(L+W)*dc+4.0/3.0*dc*dc)
}

// distinctZ returns the sorted distinct z coordinates of b's vertices.
func distinctZ(b *decad.Body) []float64 {
	seen := map[float64]struct{}{}
	for _, v := range b.Vertices() {
		seen[v.Position().Value.Z] = struct{}{}
	}
	out := make([]float64, 0, len(seen))
	for z := range seen {
		out = append(out, z)
	}
	sort.Float64s(out)
	return out
}

// capFaceCorners returns the vertices of b's end cap face, sorted.
func capFaceCorners(t *testing.T, b *decad.Body) []r3.Vec {
	t.Helper()
	faces, err := endCapFace(b).SelectFaces(b)
	require.NoError(t, err)
	require.Len(t, faces, 1)
	var out []r3.Vec
	for _, ce := range faces[0].Loops()[0].CoEdges() {
		out = append(out, ce.Start().Position().Value)
	}
	sort.Slice(out, func(i, j int) bool {
		if out[i].X != out[j].X {
			return out[i].X < out[j].X
		}
		return out[i].Y < out[j].Y
	})
	return out
}

func TestCapBlendAsymmetricBox(t *testing.T) {
	t.Parallel()
	const h = filletBoxHeight
	for _, tc := range []struct {
		name   string
		ref    func(*decad.Body) decad.FaceSelector
		d, oth float64
		dc, ds float64
	}{
		// The cap face takes d: dc = 3, ds = 6. Both volumes are float64s, so
		// the all-Plane band reports them Exact.
		{name: `cap referenced`, ref: func(b *decad.Body) decad.FaceSelector { return endCapFace(b) }, d: 3, oth: 6, dc: 3, ds: 6},
		// The side walls take d: the same two numbers swap.
		{name: `side referenced`, ref: func(*decad.Body) decad.FaceSelector { return boxSideWalls() }, d: 3, oth: 6, dc: 6, ds: 3},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, box := capBlendBox(t)
			out, err := asymChamfer(t, box, capLoopEdges(box), tc.d, tc.oth, tc.ref(box))
			require.NoError(t, err)
			requireManifold(t, out)
			decadtest.MeasuresVolume(t, out, units.CubicMillimeters(boxCapBandVolume(tc.dc, tc.ds)), decadtest.Exactly())
			require.Equal(t, []float64{0, h - tc.ds, h}, distinctZ(out), `the side level sits ds below the cap`)
			require.Equal(t, []r3.Vec{
				r3.NewVec(tc.dc, tc.dc, h), r3.NewVec(tc.dc, 60-tc.dc, h),
				r3.NewVec(100-tc.dc, tc.dc, h), r3.NewVec(100-tc.dc, 60-tc.dc, h),
			}, capFaceCorners(t, out), `the cap contour sits dc inside the loop`)
			for _, f := range capBlendPatchFaces(out) {
				require.Equal(t, decad.KindPlane, f.Surface().Kind())
			}
		})
	}

	// Swapping the reference face swaps the setbacks and nothing else: the cap
	// face taking 6 and the side walls taking 3 is the body the side walls
	// taking 6 and the cap face 3 would be, reading for reading.
	_, a := capBlendBox(t)
	capRef, err := asymChamfer(t, a, capLoopEdges(a), 6, 3, endCapFace(a))
	require.NoError(t, err)
	_, b := capBlendBox(t)
	sideRef, err := asymChamfer(t, b, capLoopEdges(b), 3, 6, boxSideWalls())
	require.NoError(t, err)
	requireSameMassReadings(t, capRef, sideRef)
	decadtest.MeasuresVolume(t, capRef, units.CubicMillimeters(boxCapBandVolume(6, 3)), decadtest.Exactly())
}

// requireSameMassReadings requires two bodies to publish bit-identical volume,
// area, centroid and bounds readings.
func requireSameMassReadings(t *testing.T, a, b *decad.Body) {
	t.Helper()
	va, err := a.Volume()
	require.NoError(t, err)
	vb, err := b.Volume()
	require.NoError(t, err)
	require.Equal(t, va, vb, `volume`)
	aa, err := a.Area()
	require.NoError(t, err)
	ab, err := b.Area()
	require.NoError(t, err)
	require.Equal(t, aa, ab, `area`)
	ca, err := a.Centroid()
	require.NoError(t, err)
	cb, err := b.Centroid()
	require.NoError(t, err)
	require.Equal(t, ca, cb, `centroid`)
	ba, err := a.Bounds()
	require.NoError(t, err)
	bb, err := b.Bounds()
	require.NoError(t, err)
	require.Equal(t, ba, bb, `bounds`)
}

func TestCapBlendAsymmetricCircularRim(t *testing.T) {
	t.Parallel()
	const R, H = 30.0, 20.0
	for _, tc := range []struct {
		name   string
		capRef bool
		dc, ds float64
	}{
		{name: `cap referenced`, capRef: true, dc: 5, ds: 2},
		{name: `side referenced`, capRef: false, dc: 2, ds: 5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			disk := circleProfile(t, R, H)
			ref := decad.Faces(decad.Cylindrical())
			if tc.capRef {
				ref = endCapFace(disk)
			}
			// d = 5 runs along the reference face, other = 2 beside it.
			out, err := asymChamfer(t, disk, capLoopEdges(disk), 5, 2, ref)
			require.NoError(t, err)
			requireManifold(t, out)
			// The band is the frustum between radius R at the side level and
			// R − dc at the cap, ds tall.
			R1 := R - tc.dc
			want := math.Pi*R*R*(H-tc.ds) + math.Pi*tc.ds/3*(R*R+R*R1+R1*R1)
			decadtest.MeasuresVolume(t, out, units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(1e-12)))

			patches := capBlendPatchFaces(out)
			require.Len(t, patches, 1)
			cone, ok := patches[0].Surface().(decad.Cone)
			require.True(t, ok, `the band over a circular wall is a Cone`)
			half, err := cone.HalfAngle.In(units.Radian)
			require.NoError(t, err)
			require.InDelta(t, math.Atan2(tc.dc, tc.ds), half, 1e-15, `the taper is atan(dc/ds)`)
		})
	}
}

// TestCapBlendAsymmetricHoleRim chamfers a circular HOLE's end-cap rim: the
// countersink widens the hole from radius rho at the side level to rho + dc at
// the cap, removing π·ds·(rho·dc + dc²/3).
func TestCapBlendAsymmetricHoleRim(t *testing.T) {
	t.Parallel()
	const rho, H = 10.0, 10.0
	for _, tc := range []struct {
		name   string
		capRef bool
		dc, ds float64
	}{
		{name: `cap referenced`, capRef: true, dc: 4, ds: 2},
		{name: `side referenced`, capRef: false, dc: 2, ds: 4},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, plate := plateWithDiskHole(t, 50, 50, rho)
			sel := decad.Edges(decad.CreatedBy(decad.CapEnd(plate)), decad.Circular())
			ref := decad.Faces(decad.Cylindrical())
			if tc.capRef {
				ref = endCapFace(plate)
			}
			out, err := asymChamfer(t, plate, sel, 4, 2, ref)
			require.NoError(t, err)
			requireManifold(t, out)
			want := 100*100*H - math.Pi*rho*rho*H - math.Pi*tc.ds*(rho*tc.dc+tc.dc*tc.dc/3)
			decadtest.MeasuresVolume(t, out, units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(1e-12)))
		})
	}
}

// TestCapBlendAsymmetricCapsPickApart chamfers two loops on two caps, each
// cap picking its own assignment: the hole's start-cap rim references its own
// wall, so it takes dc = 3 and ds = 2, while the outer loop's end-cap rim
// references the end cap face, so it takes dc = 2 and ds = 3.
func TestCapBlendAsymmetricCapsPickApart(t *testing.T) {
	t.Parallel()
	const rho, H = 10.0, 10.0
	_, plate := plateWithDiskHole(t, 50, 50, rho)
	sel := decad.Edges(decad.CreatedBy(decad.CapStart(plate)), decad.Circular()).
		Or(decad.CreatedBy(decad.CapEnd(plate)), decad.LongerThan(units.Millimeters(99)))
	ref := decad.Faces(decad.Cylindrical()).Or(decad.FaceCreatedBy(decad.CapEnd(plate)))
	out, err := asymChamfer(t, plate, sel, 2, 3, ref)
	require.NoError(t, err)
	requireManifold(t, out)

	// The countersink removes π·ds·(rho·dc + dc²/3) with dc = 3, ds = 2; the
	// outer band removes ds·((L+W)·dc − 4/3·dc²) with dc = 2, ds = 3.
	hole := math.Pi * 2 * (rho*3 + 3.0*3/3)
	outer := 3 * ((100+100)*2 - 4.0/3*2*2)
	want := 100*100*H - math.Pi*rho*rho*H - hole - outer
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(1e-12)))
	require.Equal(t, []float64{0, 2, H - 3, H}, distinctZ(out), `each cap's side level sits its own ds in`)
}

// TestCapBlendAsymmetricEqualDistancesAreTheEqualChamfer checks the equal case
// stays the body the plain chamfer builds, bit for bit, through either
// reference: other == d gives dc == ds == d on every patch kind — Plane walls,
// a whole-turn Cone, a mitered circular wall and a reflex corner's apex.
func TestCapBlendAsymmetricEqualDistancesAreTheEqualChamfer(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		build func(*testing.T) *decad.Body
		side  decad.FaceSelector
		d     float64
	}{
		{name: `box`, build: func(t *testing.T) *decad.Body { _, b := capBlendBox(t); return b }, side: boxSideWalls(), d: 5},
		{name: `disk`, build: func(t *testing.T) *decad.Body { return circleProfile(t, 30, 20) }, side: decad.Faces(decad.Cylindrical()), d: 5},
		{name: `quarter disk`, build: func(t *testing.T) *decad.Body { return quarterDiskBody(t, 10, 20) }, side: decad.Faces(decad.NormalTo(r3.NewVec(1, 0, 0))).Or(decad.NormalTo(r3.NewVec(0, 1, 0))).Or(decad.Cylindrical()), d: 2},
		{name: `reflex L`, build: reflexLBody, side: decad.Faces(decad.NormalTo(r3.NewVec(1, 0, 0))).Or(decad.NormalTo(r3.NewVec(0, 1, 0))), d: 3},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			plainBody := tc.build(t)
			plain, err := plainBody.Chamfer(t.Context(), capLoopEdges(plainBody), units.Millimeters(tc.d))
			require.NoError(t, err)
			for _, ref := range []struct {
				name string
				sel  func(*decad.Body) decad.FaceSelector
			}{
				{`cap`, func(b *decad.Body) decad.FaceSelector { return endCapFace(b) }},
				{`side`, func(*decad.Body) decad.FaceSelector { return tc.side }},
			} {
				b := tc.build(t)
				out, err := asymChamfer(t, b, capLoopEdges(b), tc.d, tc.d, ref.sel(b))
				require.NoError(t, err, ref.name)
				requireSameMassReadings(t, plain, out)
				require.Len(t, out.Faces(), len(plain.Faces()), ref.name)
			}
		})
	}
}

func TestCapBlendAsymmetricOverrunRefused(t *testing.T) {
	t.Parallel()
	const R, H = 4.0, 20.0
	for _, tc := range []struct {
		name     string
		capRef   bool
		d, other float64
		want     error
	}{
		// dc = 4 empties the radius-4 contour: SX6 at any ds.
		{name: `across the cap, cap referenced`, capRef: true, d: 4, other: 1, want: decad.ErrDegenerate},
		{name: `across the cap, side referenced`, capRef: false, d: 1, other: 4, want: decad.ErrDegenerate},
		// ds = 20 reaches the far end of the 20 mm sweep: SX7 at any dc.
		{name: `down the side, cap referenced`, capRef: true, d: 1, other: 20, want: decad.ErrUnsupported},
		{name: `down the side, side referenced`, capRef: false, d: 20, other: 1, want: decad.ErrUnsupported},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			disk := circleProfile(t, R, H)
			ref := decad.Faces(decad.Cylindrical())
			if tc.capRef {
				ref = endCapFace(disk)
			}
			_, err := asymChamfer(t, disk, capLoopEdges(disk), tc.d, tc.other, ref)
			require.ErrorIs(t, err, tc.want)
			other := decad.ErrUnsupported
			if tc.want == decad.ErrUnsupported {
				other = decad.ErrDegenerate
			}
			require.NotErrorIs(t, err, other, `one refusal answers to one sentinel`)
			require.Equal(t, []*decad.Body{disk}, disk.Document().Bodies())
		})
	}

	// Both caps of one loop: SX7 sums the two caps' ds, never dc.
	both := func(t *testing.T, capRef bool, d, other float64) error {
		t.Helper()
		_, box := capBlendBox(t)
		ref := boxSideWalls()
		if capRef {
			ref = decad.Faces(decad.FaceCreatedBy(decad.CapStart(box))).Or(decad.FaceCreatedBy(decad.CapEnd(box)))
		}
		_, err := asymChamfer(t, box, bothCapLoops(), d, other, ref)
		return err
	}
	require.ErrorIs(t, both(t, false, 10, 2), decad.ErrUnsupported, `ds = 10 on both caps meets at the midpoint`)
	require.NoError(t, both(t, true, 10, 2), `dc = 10 with ds = 2 on both caps leaves 16 mm between the bands`)
	require.NoError(t, both(t, false, 8, 2), `ds = 8 on both caps leaves 4 mm between the bands`)
}

// TestCapBlendAsymmetricUnrepresentableSetbackRefused checks SX13 reads the
// setback of its own axis: a dc below the radius's float64 spacing refuses the
// radial half, a ds below the sweep level's spacing refuses the axial half, and
// the same tiny number on the other axis builds.
func TestCapBlendAsymmetricUnrepresentableSetbackRefused(t *testing.T) {
	t.Parallel()
	const tiny = 1e-9
	t.Run(`radial`, func(t *testing.T) {
		t.Parallel()
		const R, H = 1e12, 10.0
		require.Equal(t, R, R-tiny, `the premise: tiny is below the radius's own float64 spacing`)
		disk := circleProfile(t, R, H)
		_, err := asymChamfer(t, disk, capLoopEdges(disk), tiny, 1, endCapFace(disk))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.NotErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, []*decad.Body{disk}, disk.Document().Bodies())
		// The tiny setback down the side moves a 10 mm level, so it builds.
		out, err := asymChamfer(t, disk, capLoopEdges(disk), tiny, 1, decad.Faces(decad.Cylindrical()))
		require.NoError(t, err)
		requireManifold(t, out)
	})
	t.Run(`axial`, func(t *testing.T) {
		t.Parallel()
		const H = 1e12
		require.Equal(t, H, H-tiny, `the premise: tiny is below the sweep level's own float64 spacing`)
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(0, 0, 1, 1)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		doc := decad.New()
		tower, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(H), Dir: decad.Along})
		require.NoError(t, err)
		sides := decad.Faces(decad.NormalTo(r3.NewVec(1, 0, 0))).Or(decad.NormalTo(r3.NewVec(0, 1, 0)))
		_, err = asymChamfer(t, tower, capLoopEdges(tower), tiny, 1e-3, sides)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.NotErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, []*decad.Body{tower}, doc.Bodies())
		// The tiny setback across the 1 mm square builds with a ds the level names.
		out, err := asymChamfer(t, tower, capLoopEdges(tower), 1e-3, tiny, sides)
		require.NoError(t, err)
		requireManifold(t, out)
	})
}

// TestCapBlendAsymmetricSideLevelCarriesOtherDistanceRounding pins which
// distance's unit-conversion rounding the side level charges. On the start cap
// the side level is 0 + ds, a sum that never rounds, so a side-level vertex's
// bound is ds's own conversion rounding alone: the other distance's where the
// cap face is referenced, and the positional distance's where the side walls
// are. A quarter inch stated in inches rescales to millimetres with a
// rounding; a millimetre distance rescales by one and carries none.
//
// Shown to fail: building the cap-referenced pick's dsDelta from the
// positional distance's rounding instead of the other distance's
// (asymmetricChamfer.capSetbacks) publishes the inch side level Exact.
func TestCapBlendAsymmetricSideLevelCarriesOtherDistanceRounding(t *testing.T) {
	t.Parallel()
	inch := units.Inches(0.01)
	held, err := inch.In(units.Millimeter)
	require.NoError(t, err)
	exact := new(big.Rat).Quo(new(big.Rat).Mul(new(big.Rat).SetFloat64(inch.Mag()), new(big.Rat).SetFloat64(inch.Unit().Factor())),
		new(big.Rat).SetFloat64(units.Millimeter.Factor()))
	rounding, _ := new(big.Rat).Abs(new(big.Rat).Sub(exact, new(big.Rat).SetFloat64(held))).Float64()
	require.Positive(t, rounding, `the premise: the inch-to-millimetre conversion rounds`)

	for _, tc := range []struct {
		name   string
		capRef bool
		exact  bool
	}{
		{name: `cap referenced: ds is the inch distance`, capRef: true},
		{name: `side referenced: ds is the millimetre distance`, capRef: false, exact: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, box := capBlendBox(t)
			ref := decad.Faces(decad.FaceCreatedBy(decad.CapStart(box)))
			if !tc.capRef {
				ref = boxSideWalls()
			}
			out, err := box.Chamfer(t.Context(), capLoopEdgesOn(box, false), units.Millimeters(0.5), decad.WithAsymmetricChamfer(ref, inch))
			require.NoError(t, err)
			ds := held
			if !tc.capRef {
				ds = 0.5
			}
			side := 0
			for _, v := range out.Vertices() {
				p := v.Position()
				if p.Value.Z != ds {
					continue
				}
				side++
				bound, err := p.Bound.In(units.Millimeter)
				require.NoError(t, err)
				if tc.exact {
					require.Equal(t, decad.Exact, p.Exactness)
					continue
				}
				require.GreaterOrEqual(t, bound, rounding, `the side level charges the other distance's conversion`)
				require.LessOrEqual(t, bound, 2*rounding)
			}
			require.Equal(t, 4, side, `the rectangle has four side-level vertices`)
		})
	}
}

// asymQuarterDiskLocusLength is quarterDiskBody's denoted miter locus length
// for a chamfer set back dc across the cap and ds down the side: the corner
// foot at axial fraction s sits at (sqrt(r² − 2·r·dc·s), s·dc) in the plane
// and ds·(1 − s) below the cap. A polyline through samples of the curve
// converges to its length from below.
func asymQuarterDiskLocusLength(r, dc, ds float64) float64 {
	const n = 1 << 16
	point := func(s float64) (float64, float64, float64) {
		return math.Sqrt(r*r - 2*r*dc*s), s * dc, s * ds
	}
	total := 0.0
	pu, pv, pz := point(0)
	for k := 1; k <= n; k++ {
		u, v, z := point(float64(k) / n)
		total += math.Sqrt((u-pu)*(u-pu) + (v-pv)*(v-pv) + (z-pz)*(z-pz))
		pu, pv, pz = u, v, z
	}
	return total
}

// TestCapBlendAsymmetricMiterRulingEnclosesItsLocus checks the miter ruling
// next to a circular wall encloses its denoted locus when the locus runs dc in
// the plane and ds along the sweep.
//
// Shown to fail: passing dc as the locus's axial span (capSlantEdge's call to
// capMiterLocusUpper) leaves the ds > dc rows' bound below the locus.
func TestCapBlendAsymmetricMiterRulingEnclosesItsLocus(t *testing.T) {
	t.Parallel()
	const r, h = 10.0, 20.0
	for _, tc := range []struct{ dc, ds float64 }{{1, 4}, {2, 6}, {3, 0.5}, {0.5, 3}} {
		t.Run(fmt.Sprintf("dc=%g,ds=%g", tc.dc, tc.ds), func(t *testing.T) {
			t.Parallel()
			body := quarterDiskBody(t, r, h)
			out, err := asymChamfer(t, body, capLoopEdges(body), tc.dc, tc.ds, endCapFace(body))
			require.NoError(t, err)
			edges := capBlendConePatchSlantEdges(out)
			require.Len(t, edges, 2, `the mitered Cone patch's two rulings`)
			locus := asymQuarterDiskLocusLength(r, tc.dc, tc.ds)
			for _, e := range edges {
				m, err := e.Length()
				require.NoError(t, err)
				chord, bound := m.Value.Mag(), m.Bound.Mag()
				require.Less(t, chord, locus, `the chord understates the conic it stands for`)
				require.GreaterOrEqual(t, chord+bound, locus, `the bound encloses the locus`)
				require.LessOrEqual(t, bound, chord)
			}
		})
	}
}

func TestCapBlendAsymmetricPlacedKeepsSetbacksAndRoles(t *testing.T) {
	t.Parallel()
	_, box := capBlendBox(t)
	out, err := asymChamfer(t, box, capLoopEdges(box), 3, 6, endCapFace(box))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(2, 4, 8))
	require.NoError(t, err)
	moved, err := out.Placed(t.Context(), shift)
	require.NoError(t, err)
	requireManifold(t, moved)
	// A translation by integers keeps every coordinate a float64.
	decadtest.MeasuresVolume(t, moved, units.CubicMillimeters(boxCapBandVolume(3, 6)), decadtest.Exactly())
	require.Equal(t, []float64{8, 8 + filletBoxHeight - 6, 8 + filletBoxHeight}, distinctZ(moved))
	roles := func(b *decad.Body) []string {
		var out []string
		for _, f := range capBlendPatchFaces(b) {
			out = append(out, f.Origins()[0].Role)
		}
		sort.Strings(out)
		return out
	}
	require.Equal(t, []string{`chamferCap(end,0,0)`, `chamferCap(end,0,1)`, `chamferCap(end,0,2)`, `chamferCap(end,0,3)`}, roles(moved))
	require.Equal(t, roles(out), roles(moved))
}

func TestTessellateCapBlendAsymmetric(t *testing.T) {
	t.Parallel()
	t.Run(`box`, func(t *testing.T) {
		t.Parallel()
		_, box := capBlendBox(t)
		out, err := asymChamfer(t, box, capLoopEdges(box), 3, 6, endCapFace(box))
		require.NoError(t, err)
		mesh, err := out.Tessellate(t.Context(), units.Millimeters(1))
		require.NoError(t, err)
		requireWatertight(t, mesh)
		require.Equal(t, 0.0, mesh.Bound().Mag(), `an exact all-Plane band chords nothing`)
		require.InDelta(t, boxCapBandVolume(3, 6), meshVolume(mesh), 1e-9)
	})
	t.Run(`disk`, func(t *testing.T) {
		t.Parallel()
		const r, h, dc, ds, tol = 10.0, 20.0, 1.0, 4.0, 0.25
		disk := circleProfile(t, r, h)
		out, err := asymChamfer(t, disk, capLoopEdges(disk), dc, ds, endCapFace(disk))
		require.NoError(t, err)
		mesh, err := out.Tessellate(t.Context(), units.Millimeters(tol))
		require.NoError(t, err)
		requireWatertight(t, mesh)
		require.True(t, mesh.VolumeVerified())
		bound := mesh.Bound().Mag()
		// Falsifier: points of the true frustum farther from the mesh than
		// Bound disprove it.
		for i := range 41 {
			s := float64(i) / 40
			rr := r - dc*s
			z := h - ds + ds*s
			for j := range 97 {
				th := 2 * math.Pi * float64(j) / 97
				p := r3.Vec{X: rr * math.Cos(th), Y: rr * math.Sin(th), Z: z}
				require.LessOrEqual(t, distanceToMesh(mesh, p), bound, `frustum sample %v`, p)
			}
		}
	})
	t.Run(`reflex L`, func(t *testing.T) {
		t.Parallel()
		body := reflexLBody(t)
		out, err := asymChamfer(t, body, capLoopEdges(body), 2, 5, endCapFace(body))
		require.NoError(t, err)
		mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.25))
		require.NoError(t, err)
		requireWatertight(t, mesh)
	})
}

// legendreNodes returns the n-point Gauss-Legendre nodes and weights on [0, 1].
func legendreNodes(n int) ([]float64, []float64) {
	legendre := func(z float64) (float64, float64) {
		p1, p2 := 1.0, 0.0
		for j := 1; j <= n; j++ {
			p1, p2 = ((2*float64(j)-1)*z*p1-(float64(j)-1)*p2)/float64(j), p1
		}
		return p1, float64(n) * (z*p1 - p2) / (z*z - 1)
	}
	x := make([]float64, n)
	w := make([]float64, n)
	for i := range n {
		z := math.Cos(math.Pi * (float64(i) + 0.75) / (float64(n) + 0.5))
		for range 100 {
			p, dp := legendre(z)
			next := z - p/dp
			if math.Abs(next-z) < 1e-16 {
				z = next
				break
			}
			z = next
		}
		_, dp := legendre(z)
		x[i] = (1 - z) / 2
		w[i] = 1 / ((1 - z*z) * dp * dp)
	}
	return x, w
}

// ruledQuarterDiskPatchArea integrates the area of quarterDiskBody's mitered
// Cone patch as the build rules it: straight rulings from the side-level arc
// (radius r over [0, π/2], at z = 0) to the cap-level arc (radius r − dc over
// the window its two miter feet trim, at z = ds), both angles linear in one
// parameter. Tensor Gauss-Legendre over the exact partial derivatives; the
// surface is smooth, so n points converge to float64 roundoff well before
// n = 64.
func ruledQuarterDiskPatchArea(r, dc, ds float64, n int) float64 {
	rc := r - dc
	a := math.Atan2(dc, math.Sqrt(r*r-2*r*dc))
	th0, th1 := 0.0, math.Pi/2
	c0, c1 := a, math.Pi/2-a
	xs, ws := legendreNodes(n)
	total := 0.0
	for i, u := range xs {
		ts, tc := th0+u*(th1-th0), c0+u*(c1-c0)
		side := r3.NewVec(r*math.Cos(ts), r*math.Sin(ts), 0)
		capP := r3.NewVec(rc*math.Cos(tc), rc*math.Sin(tc), ds)
		dSide := r3.NewVec(-r*math.Sin(ts), r*math.Cos(ts), 0).Scale(th1 - th0)
		dCap := r3.NewVec(-rc*math.Sin(tc), rc*math.Cos(tc), 0).Scale(c1 - c0)
		pv := capP.Sub(side)
		for j, v := range xs {
			pu := dSide.Scale(1 - v).Add(dCap.Scale(v))
			total += ws[i] * ws[j] * pu.Cross(pv).Len()
		}
	}
	return total
}

// TestCapBlendAsymmetricMiteredConeAreaEncloses checks a mitered Cone patch's
// published area bound covers the ruled patch it stands for at two distances,
// over setback ratios dc/ds from 1/1000 to 1000, the cap window closing from
// the side window's π/2 to 0.11 rad at dc = 4. The bound's skew term is
// proven (internal/capband/area.go), and every row leaves the whole published
// bound between 1.29 and 1.84 times the residual.
//
// Shown to fail: with coneSkewAreaAllow returning zero, every row's bound
// falls below its residual.
func TestCapBlendAsymmetricMiteredConeAreaEncloses(t *testing.T) {
	t.Parallel()
	const r, h = 10.0, 20.0
	for _, tc := range []struct{ dc, ds float64 }{
		{0.01, 10}, {0.1, 10}, {1, 8}, {0.5, 3}, {2, 2}, {3, 1}, {4, 0.5}, {4, 0.04}, {4, 0.004},
	} {
		t.Run(fmt.Sprintf("dc=%g,ds=%g", tc.dc, tc.ds), func(t *testing.T) {
			t.Parallel()
			body := quarterDiskBody(t, r, h)
			out, err := asymChamfer(t, body, capLoopEdges(body), tc.dc, tc.ds, endCapFace(body))
			require.NoError(t, err)
			var cone *decad.Face
			for _, f := range capBlendPatchFaces(out) {
				if f.Surface().Kind() == decad.KindCone {
					require.Nil(t, cone, `one Cone patch`)
					cone = f
				}
			}
			require.NotNil(t, cone)
			area, err := cone.Area()
			require.NoError(t, err)
			want := ruledQuarterDiskPatchArea(r, tc.dc, tc.ds, 64)
			require.InDelta(t, want, ruledQuarterDiskPatchArea(r, tc.dc, tc.ds, 96), 1e-12*want, `the quadrature has converged`)
			require.LessOrEqual(t, math.Abs(area.Value.Mag()-want), area.Bound.Mag(),
				`the published bound %v covers the ruled patch's %v mm^2`, area.Bound.Mag(), want)
		})
	}
}

// inchConversion is v converted to millimetres both ways: the float the
// evaluator holds and the exact rational the stated quantity denotes.
func inchConversion(t *testing.T, v units.Value) (float64, *big.Rat) {
	t.Helper()
	held, err := v.In(units.Millimeter)
	require.NoError(t, err)
	exact := new(big.Rat).Quo(new(big.Rat).Mul(new(big.Rat).SetFloat64(v.Mag()), new(big.Rat).SetFloat64(v.Unit().Factor())),
		new(big.Rat).SetFloat64(units.Millimeter.Factor()))
	require.NotEqual(t, 0, exact.Cmp(new(big.Rat).SetFloat64(held)), `the premise: the conversion to millimetres rounds`)
	return held, exact
}

// ratWithin reports whether |value − want| <= bound, compared exactly.
func ratWithin(value float64, want *big.Rat, bound float64) bool {
	gap := new(big.Rat).Abs(new(big.Rat).Sub(new(big.Rat).SetFloat64(value), want))
	return gap.Cmp(new(big.Rat).SetFloat64(bound)) <= 0
}

// TestCapBlendCapContourCarriesInPlaneDistanceRounding checks the cap contour
// charges the unit conversion of the distance across the cap. The 100x60
// plate's start cap is chamfered 0.1 in across the cap, which is 2.54 mm only
// to within the conversion's rounding (3.7e-17 mm), so every cap-level corner
// sits off the corner the stated distance denotes by that rounding on top of
// the offset solve's own. The volume leg guards the two-distance body's volume
// against the same denoted offset.
//
// Shown to fail: with the conversion left out of the cap contour's
// displacement (capContourDelta reading the point d rather than its span), the
// corners (97.46, 57.46) and (2.54, 57.46) published a bound of 6.280e-15 mm
// and sat 6.312e-15 mm from the denoted corner, in both rows.
func TestCapBlendCapContourCarriesInPlaneDistanceRounding(t *testing.T) {
	t.Parallel()
	inch := units.Inches(0.1)
	dc, exactDC := inchConversion(t, inch)
	const L, W, h = 100.0, 60.0, filletBoxHeight
	for _, tc := range []struct {
		name  string
		equal bool
	}{
		{name: `two distances: dc in inches, ds in millimetres`},
		{name: `equal: d in inches`, equal: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, box := capBlendBox(t)
			var opts []decad.ChamferOption
			if !tc.equal {
				opts = append(opts, decad.WithAsymmetricChamfer(decad.Faces(decad.FaceCreatedBy(decad.CapStart(box))), units.Millimeters(2)))
			}
			out, err := box.Chamfer(t.Context(), capLoopEdgesOn(box, false), inch, opts...)
			require.NoError(t, err)
			requireManifold(t, out)

			corner := func(u, v *big.Rat) [2]*big.Rat { return [2]*big.Rat{u, v} }
			sub := func(a float64, b *big.Rat) *big.Rat { return new(big.Rat).Sub(new(big.Rat).SetFloat64(a), b) }
			denoted := []([2]*big.Rat){
				corner(exactDC, exactDC), corner(exactDC, sub(W, exactDC)),
				corner(sub(L, exactDC), exactDC), corner(sub(L, exactDC), sub(W, exactDC)),
			}
			checked := 0
			for _, v := range out.Vertices() {
				p := v.Position()
				if p.Value.Z != 0 || p.Value.X == 0 || p.Value.X == L || p.Value.Y == 0 || p.Value.Y == W {
					continue
				}
				want := denoted[0]
				best := -1.0
				for _, c := range denoted {
					du, _ := sub(p.Value.X, c[0]).Float64()
					dv, _ := sub(p.Value.Y, c[1]).Float64()
					if d := math.Hypot(du, dv); best < 0 || d < best {
						best, want = d, c
					}
				}
				du, dv := sub(p.Value.X, want[0]), sub(p.Value.Y, want[1])
				gap2 := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
				require.Positive(t, gap2.Sign(), `the premise: the held corner is not the denoted one`)
				bound, err := p.Bound.In(units.Millimeter)
				require.NoError(t, err)
				rb := new(big.Rat).SetFloat64(bound)
				require.LessOrEqual(t, gap2.Cmp(new(big.Rat).Mul(rb, rb)), 0,
					`cap-level corner %v publishes bound %v but sits off the denoted corner`, p.Value, bound)
				checked++
			}
			require.Equal(t, 4, checked, `the cap contour has four corners`)

			if tc.equal {
				return
			}
			// The denoted volume, boxCapBandVolume over the exact dc.
			const ds = 2.0
			lw := new(big.Rat).SetFloat64(L * W)
			want := new(big.Rat).Mul(lw, new(big.Rat).SetFloat64(h-ds))
			band := new(big.Rat).Sub(lw, new(big.Rat).Mul(new(big.Rat).SetFloat64(L+W), exactDC))
			band.Add(band, new(big.Rat).Mul(big.NewRat(4, 3), new(big.Rat).Mul(exactDC, exactDC)))
			want.Add(want, new(big.Rat).Mul(new(big.Rat).SetFloat64(ds), band))
			vol, err := out.Volume()
			require.NoError(t, err)
			bound, err := vol.Bound.In(units.CubicMillimeter)
			require.NoError(t, err)
			require.True(t, ratWithin(vol.Value.Mag(), want, bound),
				`volume %v ± %v must enclose the volume the stated dc = %v mm denotes`, vol.Value.Mag(), bound, dc)
		})
	}
}

// TestCapBlendCapCircleCarriesInPlaneDistanceRounding checks a cornerless
// circle's cap contour charges the same conversion. A radius-10 disk chamfered
// 0.1 in holds its cap circle at 10 − 2.54 = 7.46 mm, a subtraction that rounds
// to nothing, so the offset solve's own displacement is zero and the cap
// circle's seam vertex sits off the denoted one by the conversion alone.
//
// Shown to fail: with the conversion left out (capWholeCircleDelta reading
// the point d), the seam vertex published an Exact position 3.7e-17 mm off the
// denoted seam.
func TestCapBlendCapCircleCarriesInPlaneDistanceRounding(t *testing.T) {
	t.Parallel()
	const r, h = 10.0, 8.0
	inch := units.Inches(0.1)
	dc, exactDC := inchConversion(t, inch)
	body := circleProfile(t, r, h)
	out, err := body.Chamfer(t.Context(), capLoopEdges(body), inch)
	require.NoError(t, err)
	requireManifold(t, out)

	denoted := new(big.Rat).Sub(new(big.Rat).SetFloat64(r), exactDC)
	seams := 0
	for _, v := range out.Vertices() {
		p := v.Position()
		if p.Value.Z != h {
			continue
		}
		require.Equal(t, r-dc, p.Value.X, `the cap circle's seam sits at the held offset radius`)
		gap := new(big.Rat).Abs(new(big.Rat).Sub(new(big.Rat).SetFloat64(p.Value.X), denoted))
		require.Positive(t, gap.Sign(), `the premise: the held seam is not the denoted one`)
		bound, err := p.Bound.In(units.Millimeter)
		require.NoError(t, err)
		require.True(t, ratWithin(p.Value.X, denoted, bound),
			`the cap seam publishes bound %v (%v) but sits off the denoted seam`, bound, p.Exactness)
		seams++
	}
	require.Equal(t, 1, seams, `the cap circle has one seam vertex`)
}
