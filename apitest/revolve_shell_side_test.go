package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers docs/modify-reach-design.md §9.3.2's side opening and
// docs/shell-opening-design.md §8's rim: a revolve shell that removes a
// connected run of generated side faces offsets the kept chain, closes it at
// each opening end with the removed walk's own carrier cut by the offset
// (Table RO), and sweeps that wall region. Fixtures are revolve_shell_test.go's:
// a meridian point (u, v) is z = u, ρ = v, and Pappus turns the wall region's
// ∫ρ dA into the volume.
//
// Legs shown to fail (each changed or deleted, the fixture watched go red,
// then restored): halving the right-angle rim's foot sends every right-angle
// volume red; replacing OpeningJoin's cut with the kept walk's own offset foot
// sends every slanted, acute, reflex and arc rim fixture red; dropping the
// span test leaves the short tread to the offset's generic S11a drop and lets
// the short cone build a wall past the cone; dropping the end corner's dead
// zone together with the re-crossing test leaves the filleted ridge's smooth
// ends to the §5 audit's contact refusal (either alone still refuses SO1
// through the other); reversing the rim arc's sense sends both bead
// volumes red; and publishing a zero section displacement for the side wall
// leaves the 5/3 miter, the √5 acute cut and both √24 bead cuts at seam
// vertices with a zero bound. The one-axis-end ordering gate is not exhibited: on these
// meridians the S11a drop gate fires first.

// sideFace names the one generated side face of b whose surface pick
// accepts, by that face's own role.
func sideFace(t *testing.T, b *decad.Body, pick func(decad.Surface) bool) *decad.FaceQuery {
	t.Helper()
	return decad.Faces(decad.FaceCreatedBy(sideRole(t, b, pick)))
}

// sideRole is the role of the one face of b whose surface pick accepts.
func sideRole(t *testing.T, b *decad.Body, pick func(decad.Surface) bool) decad.FeatureRef {
	t.Helper()
	var hit *decad.Face
	for _, f := range b.Faces() {
		if !pick(f.Surface()) {
			continue
		}
		require.Nil(t, hit, `exactly one face matches`)
		hit = f
	}
	require.NotNil(t, hit)
	return hit.Origins()[0]
}

// coneAndTop names the slanted meridian's cone and top disc.
func coneAndTop(t *testing.T, b *decad.Body) *decad.FaceQuery {
	t.Helper()
	return sideFace(t, b, coneSurface).Or(decad.FaceCreatedBy(sideRole(t, b, planeAtU(6))))
}

// cylinderOfRadius10 picks a cylindrical face of radius 10.
func cylinderOfRadius10() func(decad.Surface) bool {
	const r = 10.0
	return func(s decad.Surface) bool {
		c, ok := s.(decad.Cylinder)
		if !ok {
			return false
		}
		got, err := c.Radius.In(units.Millimeter)
		return err == nil && got == r
	}
}

// planeAtU picks a planar face whose frame origin lies at x = u.
func planeAtU(u float64) func(decad.Surface) bool {
	return func(s decad.Surface) bool {
		p, ok := s.(decad.Plane)
		return ok && math.Abs(p.Frame.N().X) > 0.5 && p.Frame.Origin().X == u
	}
}

func TestRevolveShellSideOpening(t *testing.T) {
	t.Parallel()
	t.Run("full ring without its outer skin", func(t *testing.T) {
		t.Parallel()
		// The ring z ∈ [0, 20], ρ ∈ [5, 10] loses its ρ = 10 skin. Inward
		// 1 mm, the wall is the meridian less the core z ∈ [1, 19],
		// ρ ∈ [6, 10]: ∫ρ dA = 750 − 576 = 174, a channel open outward.
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, decad.FullRevolution{})
		shelled, err := ring.Shell(t.Context(), sideFace(t, ring, cylinderOfRadius10()), units.Millimeters(1))
		require.NoError(t, err)
		require.Equal(t, []*decad.Body{shelled}, doc.Bodies())
		requireManifold(t, shelled)
		require.Len(t, shelled.Lumps(), 1)
		require.Len(t, shelled.Shells(), 1, `the opening leaves no void`)
		decadtest.MeasuresVolume(t, shelled, fullTurnVolume(174))
		// The receiver's ρ = 5 skin, the cavity's ρ = 6 wall, and the two
		// rims of the opening, each a band on the removed skin's radius.
		require.ElementsMatch(t, []float64{5, 6, 10, 10}, cylinderRadii(t, shelled))
		requireMeshWatertightAt(t, shelled, 0.05)
	})
	t.Run("full ring outward", func(t *testing.T) {
		t.Parallel()
		// Outward 1 mm: two end strips ρ ∈ [5, 10] (37.5 each), the ρ ∈ [4, 5]
		// strip under the core (90), and two quarter disks of radius 1 at the
		// corners (5π/4 − 1/3 each).
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, decad.FullRevolution{})
		shelled, err := ring.Shell(t.Context(), sideFace(t, ring, cylinderOfRadius10()), units.Millimeters(1),
			decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		requireManifold(t, shelled)
		decadtest.MeasuresVolume(t, shelled, fullTurnVolume(165-2.0/3+5*math.Pi/2))
	})
	t.Run("solid cylinder without one end", func(t *testing.T) {
		t.Parallel()
		// The solid cylinder z ∈ [0, 20], ρ ≤ 10 loses its z = 20 end: a cup
		// whose kept chain runs from the opening down to the axis, so its
		// offset ends on the axis at z = 2. π·(10²·20 − 8²·18).
		doc := decad.New()
		cyl := revolveMeridian(t, doc, cylinderMeridian, decad.FullRevolution{})
		shelled, err := cyl.Shell(t.Context(), sideFace(t, cyl, planeAtU(20)), units.Millimeters(2))
		require.NoError(t, err)
		requireManifold(t, shelled)
		require.Len(t, shelled.Shells(), 1)
		decadtest.MeasuresVolume(t, shelled, fullTurnVolume(1000-576))
		require.ElementsMatch(t, []float64{8, 10}, cylinderRadii(t, shelled), `no wall grows along the axis`)
		requireMeshWatertightAt(t, shelled, 0.05)
	})
	t.Run("slanted kept walk at a right-angle rim", func(t *testing.T) {
		t.Parallel()
		// The meridian (0, 0), (20, 0), (24, 8), (16, 12), (0, 12) loses the
		// cone (20, 0)–(24, 8). The kept walk (24, 8)–(16, 12) leaves it at a
		// right angle, so the rim is the removed cone's own line, but the kept
		// walk is slanted: its offset foot at the opening is
		// (24 − 1/√5, 8 − 2/√5), which the float build rounds. The wall's
		// record carries a nonzero section displacement
		// (docs/modify-reach-design.md §9.3.1), so the foot's swept vertex
		// encloses the irrational point. Leg shown to fail before this
		// fixture was accepted: publishing a zero displacement leaves the foot
		// vertex Exact at a float that is not the foot.
		doc := decad.New()
		body := revolveMeridian(t, doc, [][2]float64{{0, 0}, {20, 0}, {24, 8}, {16, 12}, {0, 12}}, decad.FullRevolution{})
		apexAt20 := func(s decad.Surface) bool {
			c, ok := s.(decad.Cone)
			return ok && math.Abs(c.Origin.X-20) < 1e-9
		}
		shelled, err := body.Shell(t.Context(), sideFace(t, body, apexAt20), units.Millimeters(1))
		require.NoError(t, err)
		requireManifold(t, shelled)
		const prec = 400
		ref := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
		inv5 := new(big.Float).SetPrec(prec).Quo(ref(1), new(big.Float).SetPrec(prec).Sqrt(ref(5)))
		footU := new(big.Float).SetPrec(prec).Sub(ref(24), inv5)
		footV := new(big.Float).SetPrec(prec).Sub(ref(8), new(big.Float).SetPrec(prec).Mul(ref(2), inv5))
		fu, _ := footU.Float64()
		fv, _ := footV.Float64()
		var foot *decad.Vertex
		for _, v := range shelled.Vertices() {
			p := v.Position().Value
			if math.Abs(p.X-fu) < 1e-6 && math.Abs(math.Hypot(p.Y, p.Z)-fv) < 1e-6 {
				foot = v
			}
		}
		require.NotNil(t, foot, `the opening's offset foot sweeps a seam vertex`)
		pos := foot.Position()
		require.Positive(t, pos.Bound.Base())
		// The seam sits at φ = 0, so the foot's world point is (u, v, 0).
		du := new(big.Float).SetPrec(prec).Sub(ref(pos.Value.X), footU)
		dv := new(big.Float).SetPrec(prec).Sub(ref(pos.Value.Y), footV)
		dz := ref(pos.Value.Z)
		sq := new(big.Float).SetPrec(prec).Mul(du, du)
		sq.Add(sq, new(big.Float).SetPrec(prec).Mul(dv, dv))
		sq.Add(sq, new(big.Float).SetPrec(prec).Mul(dz, dz))
		b := ref(pos.Bound.Base())
		require.LessOrEqual(t, sq.Cmp(new(big.Float).SetPrec(prec).Mul(b, b)), 0,
			`the foot vertex %v ± %v encloses the irrational foot`, pos.Value, pos.Bound)
	})
	t.Run("solid cylinder outward", func(t *testing.T) {
		t.Parallel()
		// Outward 2 mm: the skin strip ρ ∈ [10, 12] (440), the end slab
		// z ∈ [−2, 0] (100) and the quarter disk of radius 2 at the corner
		// (10π + 8/3).
		doc := decad.New()
		cyl := revolveMeridian(t, doc, cylinderMeridian, decad.FullRevolution{})
		shelled, err := cyl.Shell(t.Context(), sideFace(t, cyl, planeAtU(20)), units.Millimeters(2),
			decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		requireManifold(t, shelled)
		decadtest.MeasuresVolume(t, shelled, fullTurnVolume(540+8.0/3+10*math.Pi))
	})
	t.Run("half ring with both caps and its outer skin", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, halfTurn)
		var outer *decad.Face
		for _, f := range ring.Faces() {
			if cylinderOfRadius10()(f.Surface()) {
				outer = f
			}
		}
		require.NotNil(t, outer)
		sel := decad.Faces(decad.NormalTo(r3.NewVec(0, 0, 1))).Or(decad.FaceCreatedBy(outer.Origins()[0]))
		shelled, err := ring.Shell(t.Context(), sel, units.Millimeters(1))
		require.NoError(t, err)
		requireManifold(t, shelled)
		decadtest.MeasuresVolume(t, shelled, halfTurnVolume(174))
		// The two rim bands at the angular openings are the wall region.
		for _, ref := range []decad.FeatureRef{decad.CapStart(shelled), decad.CapEnd(shelled)} {
			rims, err := decad.Faces(decad.FaceCreatedBy(ref)).Exactly(1).SelectFaces(shelled)
			require.NoError(t, err)
			area, err := rims[0].Area()
			require.NoError(t, err)
			decadtest.Measures(t, "rim band area", area, units.SquareMillimeters(28))
		}
	})
	t.Run("placed", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, decad.FullRevolution{})
		shelled, err := ring.Shell(t.Context(), sideFace(t, ring, cylinderOfRadius10()), units.Millimeters(1))
		require.NoError(t, err)
		rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
		require.NoError(t, err)
		placed, err := shelled.Placed(t.Context(), rotation)
		require.NoError(t, err)
		require.Len(t, placed.Faces(), len(shelled.Faces()))
		decadtest.MeasuresVolume(t, placed, fullTurnVolume(174))
	})
}

func TestRevolveShellSideOpeningRefusals(t *testing.T) {
	t.Parallel()
	t.Run("two separate runs", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, decad.FullRevolution{})
		_, err := ring.Shell(t.Context(), decad.Faces(decad.Planar()), units.Millimeters(1))
		requireShellRefused(t, doc, ring, err, decad.ErrUnsupported, `one connected run`)
	})
	t.Run("every side face", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, decad.FullRevolution{})
		_, err := ring.Shell(t.Context(), decad.Faces(), units.Millimeters(1))
		requireShellRefused(t, doc, ring, err, decad.ErrUnsupported, `not all of them`)
	})
	t.Run("a removed walk shorter than the wall", func(t *testing.T) {
		t.Parallel()
		// A step whose 2 mm tread at ρ = 10 is the opening: the 3 mm rim at
		// the tread's outer end would run past the tread (SO2).
		doc := decad.New()
		step := revolveMeridian(t, doc, [][2]float64{{0, 5}, {20, 5}, {20, 10}, {18, 10}, {18, 11}, {0, 11}}, decad.FullRevolution{})
		_, err := step.Shell(t.Context(), sideFace(t, step, cylinderOfRadius10()), units.Millimeters(3))
		requireShellRefused(t, doc, step, err, decad.ErrUnsupported, `past the far end of the removed walk`)
	})
	t.Run("a run between two kept walks on the axis", func(t *testing.T) {
		t.Parallel()
		// The solid cylinder's skin lies between its two end disks, so the
		// two disks would each keep a wall of their own.
		doc := decad.New()
		cyl := revolveMeridian(t, doc, cylinderMeridian, decad.FullRevolution{})
		_, err := cyl.Shell(t.Context(), sideFace(t, cyl, cylinderOfRadius10()), units.Millimeters(1))
		requireShellRefused(t, doc, cyl, err, decad.ErrUnsupported, `two wall regions`)
	})
}

// slantedMeridian is docs/shell-opening-design.md §9's revolve fixture in
// (z, ρ): a cylinder ρ ≤ 8 over z ∈ [0, 2] under a frustum narrowing to ρ = 5
// at z = 6, closed by the top disc and the axis. Its ∫ρ dA is 150.
var slantedMeridian = [][2]float64{{0, 0}, {6, 0}, {6, 5}, {2, 8}, {0, 8}}

// coneSurface picks a conical face.
func coneSurface(s decad.Surface) bool { _, ok := s.(decad.Cone); return ok }

// torusSurface picks a toroidal face.
func torusSurface(s decad.Surface) bool { _, ok := s.(decad.Torus); return ok }

// beadSketch is a half disk off the axis: the chord ρ = 6 from z = 0 to
// z = 10 and the arc of radius 5 about (5, 6) over it.
func beadSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 6)
	end := s.CreatePoint(10, 6)
	c := s.CreatePoint(5, 6)
	for _, p := range []*sketch.Point{o, end, c} {
		s.Fix(p)
	}
	s.CreateLine(o, end)
	s.CreateArc(c, end, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// cutPrec is the binary precision the closed-form cuts are evaluated at.
const cutPrec = 400

// exactCut is a closed-form meridian coordinate at cutPrec.
func exactCut(x float64) *big.Float { return new(big.Float).SetPrec(cutPrec).SetFloat64(x) }

// requireCutEnclosed finds the seam vertex the wall's cut at meridian point
// (z, ρ) sweeps, and requires its position to carry a nonzero bound that
// encloses the closed-form point: the wall's section displacement charges the
// float cut (docs/shell-opening-design.md §8). The seam sits at φ = 0, so the
// cut's world point is (z, ρ, 0).
func requireCutEnclosed(t *testing.T, b *decad.Body, z, rho *big.Float) {
	t.Helper()
	zf, _ := z.Float64()
	rf, _ := rho.Float64()
	var hit *decad.Vertex
	for _, v := range b.Vertices() {
		p := v.Position().Value
		if math.Abs(p.X-zf) < 1e-6 && math.Abs(math.Hypot(p.Y, p.Z)-rf) < 1e-6 {
			hit = v
		}
	}
	require.NotNil(t, hit, `the cut sweeps a seam vertex`)
	pos := hit.Position()
	require.Positive(t, pos.Bound.Base(), `the float cut is charged`)
	du := new(big.Float).SetPrec(cutPrec).Sub(exactCut(pos.Value.X), z)
	dv := new(big.Float).SetPrec(cutPrec).Sub(exactCut(pos.Value.Y), rho)
	dz := exactCut(pos.Value.Z)
	sq := new(big.Float).SetPrec(cutPrec).Mul(du, du)
	sq.Add(sq, new(big.Float).SetPrec(cutPrec).Mul(dv, dv))
	sq.Add(sq, new(big.Float).SetPrec(cutPrec).Mul(dz, dz))
	bound := exactCut(pos.Bound.Base())
	require.LessOrEqual(t, sq.Cmp(new(big.Float).SetPrec(cutPrec).Mul(bound, bound)), 0,
		`the cut vertex %v ± %v encloses the closed-form cut`, pos.Value, pos.Bound)
}

func TestRevolveShellSlantedRim(t *testing.T) {
	t.Parallel()
	t.Run("obtuse end on a removed cone", func(t *testing.T) {
		t.Parallel()
		// Inward 1.5 mm with the cone and the top disc removed: the kept
		// cylinder's offset ρ = 6.5 meets the cone's own line at (z, ρ) =
		// (4, 6.5), so the rim is the cone frustum from ρ = 8 down to 6.5.
		// The cavity is the cylinder ρ ≤ 6.5 over z ∈ [1.5, 4] under the
		// frustum to (6, 5): ∫ρ dA = 52.8125 + 33.25, and the wall's is
		// 150 − 86.0625 = 1023/16.
		doc := decad.New()
		body := revolveMeridian(t, doc, slantedMeridian, decad.FullRevolution{})
		shelled, err := body.Shell(t.Context(), coneAndTop(t, body), units.Millimeters(1.5))
		require.NoError(t, err)
		requireManifold(t, shelled)
		require.Len(t, shelled.Shells(), 1, `the opening leaves no void`)
		decadtest.MeasuresVolume(t, shelled, fullTurnVolume(1023.0/16))
		var rim []decad.Cone
		for _, f := range shelled.Faces() {
			if c, ok := f.Surface().(decad.Cone); ok {
				rim = append(rim, c)
			}
		}
		require.Len(t, rim, 1, `the rim is the one conical face, on the removed cone's own carrier`)
		// The removed cone's half angle: its slant rises 4 in z per 3 in ρ.
		half, err := rim[0].HalfAngle.In(units.Radian)
		require.NoError(t, err)
		require.InDelta(t, math.Atan2(3, 4), half, 1e-12)
		requireMeshWatertightAt(t, shelled, 0.05)
	})
	t.Run("reflex end outward", func(t *testing.T) {
		t.Parallel()
		// Outward 0.75 mm the band lies off the material, so the cone's own
		// line is walked back past (2, 8) to the offset ρ = 8.75 at z = 1.
		// The wall: the slab under the bottom disc (24), the quarter disk at
		// the rim (8·0.140625π + 0.140625), the strip ρ ∈ [8, 8.75] over
		// z ∈ [0, 1] (6.28125), and the triangle up to the corner (3.09375).
		doc := decad.New()
		body := revolveMeridian(t, doc, slantedMeridian, decad.FullRevolution{})
		shelled, err := body.Shell(t.Context(), coneAndTop(t, body), units.Millimeters(0.75),
			decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		requireManifold(t, shelled)
		decadtest.MeasuresVolume(t, shelled, fullTurnVolume(33.515625+1.125*math.Pi))
	})
	t.Run("obtuse end on a removed disc", func(t *testing.T) {
		t.Parallel()
		// Inward 1 mm with the top disc alone removed: the kept cone's offset
		// meets z = 6 at ρ = 3.75. The cavity (z, ρ) = (1, 0), (1, 7),
		// (5/3, 7), (6, 3.75), (6, 0) is a cylinder under a frustum, ∫ρ dA
		// 49/3 + 290.265625/4.5, so the wall's is 19919/288.
		doc := decad.New()
		body := revolveMeridian(t, doc, slantedMeridian, decad.FullRevolution{})
		shelled, err := body.Shell(t.Context(), sideFace(t, body, planeAtU(6)), units.Millimeters(1))
		require.NoError(t, err)
		requireManifold(t, shelled)
		decadtest.MeasuresVolume(t, shelled, fullTurnVolume(19919.0/288))
		// The cylinder's offset ρ = 7 meets the cone's at z = 5/3, which no
		// float holds.
		requireCutEnclosed(t, shelled, new(big.Float).SetPrec(cutPrec).Quo(exactCut(5), exactCut(3)), exactCut(7))
	})
	t.Run("acute end on a removed base", func(t *testing.T) {
		t.Parallel()
		// The cone without its base disk: the slant's 1 mm offset meets the
		// base plane z = 0 at ρ = (20 − √5)/2, so the cavity is the cone of
		// height L = 20 − √5 under that radius, ∫ρ dA = L³/24.
		doc := decad.New()
		cone := revolveMeridian(t, doc, coneMeridian, decad.FullRevolution{})
		shelled, err := cone.Shell(t.Context(), sideFace(t, cone, planeAtU(0)), units.Millimeters(1))
		require.NoError(t, err)
		requireManifold(t, shelled)
		l := 20 - math.Sqrt(5)
		decadtest.MeasuresVolume(t, shelled, fullTurnVolume((8000-l*l*l)/24))
		root5 := new(big.Float).SetPrec(cutPrec).Sqrt(exactCut(5))
		rho := new(big.Float).SetPrec(cutPrec).Sub(exactCut(20), root5)
		requireCutEnclosed(t, shelled, exactCut(0), rho.Quo(rho, exactCut(2)))
	})
	// The bead's chord meets its arc at a right angle at both ends, and the
	// arc's circle crosses the chord's offset ρ = 6 ± 1 at z = 5 ± √24. The
	// wall is the disk's strip between the chord and the offset:
	// ∫ (6 + y)·2√(25 − y²) dy over y ∈ [0, 1] inward, over [−1, 0] outward,
	// where the arc is walked back past the chord's ends.
	strip := 6 * (math.Sqrt(24) + 25*math.Asin(0.2))
	lift := 2 * (125 - 24*math.Sqrt(24)) / 3
	for _, tc := range []struct {
		name string
		opts []decad.ShellOption
		want float64
		rho  float64
	}{
		{"arc rim inward", nil, strip + lift, 7},
		{"arc rim outward", []decad.ShellOption{decad.WithShellSense(decad.Outward)}, strip - lift, 5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			s, p := beadSketch(t)
			bead, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
			require.NoError(t, err)
			shelled, err := bead.Shell(t.Context(), sideFace(t, bead, torusSurface), units.Millimeters(1), tc.opts...)
			require.NoError(t, err)
			requireManifold(t, shelled)
			decadtest.MeasuresVolume(t, shelled, fullTurnVolume(tc.want))
			z := new(big.Float).SetPrec(cutPrec).Sqrt(exactCut(24))
			requireCutEnclosed(t, shelled, z.Add(z, exactCut(5)), exactCut(tc.rho))
			tori := 0
			for _, f := range shelled.Faces() {
				if torusSurface(f.Surface()) {
					tori++
				}
			}
			require.Equal(t, 2, tori, `each rim is a torus band on the removed arc's own circle`)
		})
	}
}

func TestRevolveShellSlantedRimRefusals(t *testing.T) {
	t.Parallel()
	t.Run("a removed cone shorter than its cut", func(t *testing.T) {
		t.Parallel()
		// Inward 3.5 mm the cut lies 3.5/0.6 along the 5 mm cone (SO2).
		doc := decad.New()
		body := revolveMeridian(t, doc, slantedMeridian, decad.FullRevolution{})
		_, err := body.Shell(t.Context(), coneAndTop(t, body), units.Millimeters(3.5))
		requireShellRefused(t, doc, body, err, decad.ErrUnsupported, `past the far end of the removed walk`)
	})
	t.Run("a rim landing on the corner arc", func(t *testing.T) {
		t.Parallel()
		// Outward 1.5 mm the cone's line walked back from (2, 8) reaches
		// ρ = 9.5 at z = 0, where the corner arc ends: the trimmed offset of
		// the cylinder has no length left (S11a).
		doc := decad.New()
		body := revolveMeridian(t, doc, slantedMeridian, decad.FullRevolution{})
		_, err := body.Shell(t.Context(), coneAndTop(t, body), units.Millimeters(1.5), decad.WithShellSense(decad.Outward))
		requireShellRefused(t, doc, body, err, decad.ErrUnsupported, `drops a section feature`)
	})
	t.Run("a removed walk tangent to the kept one", func(t *testing.T) {
		t.Parallel()
		// A ring whose outer end is a ridge of two slants, every rim
		// filleted 0.5 mm, then the fillet on the ridge removed: both ends of
		// the kept chain are smooth (SO1). The slants' float tangents leave
		// each corner a rounding off straight, so only the corner's dead
		// zone refuses it.
		doc := decad.New()
		ring := revolveMeridian(t, doc, [][2]float64{{0, 5}, {20, 5}, {23, 9}, {19, 13}, {0, 13}}, decad.FullRevolution{})
		filleted, err := ring.Fillet(t.Context(), decad.Edges(decad.Circular(), decad.Convex()), units.Millimeters(0.5))
		require.NoError(t, err)
		shoulder := func(s decad.Surface) bool {
			tor, ok := s.(decad.Torus)
			return ok && tor.Center.X > 21.5
		}
		_, err = filleted.Shell(t.Context(), sideFace(t, filleted, shoulder), units.Millimeters(0.25))
		requireShellRefused(t, doc, filleted, err, decad.ErrUnsupported, `smooth or cusped`)
	})
}
