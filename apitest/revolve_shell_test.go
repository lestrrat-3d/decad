package apitest_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers docs/modify-reach-design.md §9.3 (Table RX row RX2, Table
// BX row BX7): Shell on a partial revolve with both angular caps removed. Every
// fixture is drawn on the XY plane and revolved half a turn about the sketch's
// u axis, so a meridian point (u, v) is the axis coordinate z = u and the
// radius ρ = v, and both angular caps lie in the XY plane — the one selector
// Faces(NormalTo(z)) names exactly the two of them. Pappus turns each wall
// region's ∫ρ dA into the volume: V = π·∫ρ dA over a half turn.
//
// Legs shown to fail before these fixtures were accepted: offsetting the
// recorded meridian with its axis walk, instead of the effective one, grows a
// wall along the axis and sends the cylinder, cone and placement volumes red
// (the cylinder's wall reads 520π rather than 488π); reading S10 off the
// recorded half-section's own inradius refuses the cylinder's 7 mm wall;
// replacing the apex miter with the slant's own offset foot sends both inward
// cones red, and dropping the apex arc for a miter sends both outward cones
// red; dropping the axis-gate remap lets the outward wall that reaches across
// the axis leave as ErrDegenerate instead of ErrUnsupported; and dropping the
// axis-order gate leaves the flange to the §5 audit's generic crossing
// message, which the refusal row reads by text.

// halfTurn is the sweep every fixture here takes.
var halfTurn = decad.AngleExtent{A: units.Degrees(180), Dir: decad.Along}

// bothAngularCaps names the two angular caps of a half-turn revolve about the
// u axis of the XY plane: both lie in that plane.
func bothAngularCaps() *decad.FaceQuery {
	return decad.Faces(decad.NormalTo(r3.NewVec(0, 0, 1))).Exactly(2)
}

// halfTurnVolume is Pappus over a half turn.
func halfTurnVolume(q float64) units.Value { return units.CubicMillimeters(math.Pi * q) }

// cylinderRadii lists every cylindrical face's radius in millimetres.
func cylinderRadii(t *testing.T, b *decad.Body) []float64 {
	t.Helper()
	var out []float64
	for _, f := range b.Faces() {
		c, ok := f.Surface().(decad.Cylinder)
		if !ok {
			continue
		}
		r, err := c.Radius.In(units.Millimeter)
		require.NoError(t, err)
		out = append(out, r)
	}
	return out
}

// requireShellRefused asserts a refused shell left the receiver live and the
// document unchanged.
func requireShellRefused(t *testing.T, doc *decad.Document, body *decad.Body, err error, sentinel error, msg string) {
	t.Helper()
	require.ErrorIs(t, err, sentinel)
	require.Contains(t, err.Error(), msg)
	require.Equal(t, []*decad.Body{body}, doc.Bodies(), `a refused shell leaves the receiver live and the document unchanged`)
}

// ringMeridian is a rectangle strictly off the axis: z ∈ [0, 20], ρ ∈ [5, 10].
var ringMeridian = [][2]float64{{0, 5}, {20, 5}, {20, 10}, {0, 10}}

// cylinderMeridian is a solid cylinder's half-section, its bottom edge on the
// axis: z ∈ [0, 20], ρ ∈ [0, 10].
var cylinderMeridian = [][2]float64{{0, 0}, {20, 0}, {20, 10}, {0, 10}}

// coneMeridian is a cone's half-section: the base disk at z = 0 of radius 10,
// the apex on the axis at z = 20.
var coneMeridian = [][2]float64{{0, 0}, {20, 0}, {0, 10}}

func TestRevolveShellOffAxisRing(t *testing.T) {
	t.Parallel()
	t.Run("inward", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, halfTurn)
		shelled, err := ring.Shell(t.Context(), bothAngularCaps(), units.Millimeters(1))
		require.NoError(t, err)
		require.Equal(t, []*decad.Body{shelled}, doc.Bodies(), `the shell retires its receiver`)
		require.True(t, shelled.IsSolid())
		requireManifold(t, shelled)
		// The wall is the meridian less its 1 mm erosion z ∈ [1, 19],
		// ρ ∈ [6, 9]: ∫ρ dA = 75/2·20 − 45/2·18 = 345.
		decadtest.MeasuresVolume(t, shelled, halfTurnVolume(345))
		// Half-turn walls: the outer skin's cylinders π·5·20 + π·10·20 and end
		// annuli 2·(π/2)(100 − 25), the cavity's π·6·18 + π·9·18 and
		// 2·(π/2)(81 − 36), and the two rim bands of 46 each.
		decadtest.MeasuresArea(t, shelled, units.SquareMillimeters(690*math.Pi+92))
		require.ElementsMatch(t, []float64{5, 6, 9, 10}, cylinderRadii(t, shelled))
		// The two rim bands are the result's own angular caps, each the wall
		// region's area 100 − 54 = 46.
		for _, ref := range []decad.FeatureRef{decad.CapStart(shelled), decad.CapEnd(shelled)} {
			rims, err := decad.Faces(decad.FaceCreatedBy(ref)).Exactly(1).SelectFaces(shelled)
			require.NoError(t, err)
			area, err := rims[0].Area()
			require.NoError(t, err)
			decadtest.Measures(t, "rim band area", area, units.SquareMillimeters(46))
		}
	})
	t.Run("outward", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, halfTurn)
		shelled, err := ring.Shell(t.Context(), bothAngularCaps(), units.Millimeters(1), decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		requireManifold(t, shelled)
		// The 1 mm dilation is the core z ∈ [0, 20], ρ ∈ [4, 11] (∫ρ dA =
		// 1050), two end strips ρ ∈ [5, 10] (37.5 each), and four quarter disks
		// of radius 1 at the corners, whose centroids sit 4/(3π) off radii 5
		// and 10 in opposite senses (7.5π together). The wall subtracts the
		// meridian's own 750.
		decadtest.MeasuresVolume(t, shelled, halfTurnVolume(375+7.5*math.Pi))
	})
}

func TestRevolveShellSolidCylinder(t *testing.T) {
	t.Parallel()
	t.Run("inward", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		cyl := revolveMeridian(t, doc, cylinderMeridian, halfTurn)
		shelled, err := cyl.Shell(t.Context(), bothAngularCaps(), units.Millimeters(2))
		require.NoError(t, err)
		requireManifold(t, shelled)
		// The axis walk grows no wall: the cavity is z ∈ [2, 18], ρ ∈ [0, 8],
		// and the wall's ∫ρ dA = 100/2·20 − 64/2·16 = 488. Offsetting the
		// recorded half-section instead would leave a cavity ρ ∈ [2, 8] and a
		// 2 mm tube around the axis, 520.
		decadtest.MeasuresVolume(t, shelled, halfTurnVolume(488))
		require.ElementsMatch(t, []float64{8, 10}, cylinderRadii(t, shelled),
			`the outer wall and the cavity wall are the only cylinders; no tube surrounds the axis`)
		// Two cylinders, the four disks and annuli normal to the axis, and the
		// two rim bands.
		require.Len(t, shelled.Faces(), 8)
	})
	t.Run("inward past the half-section's own inradius", func(t *testing.T) {
		t.Parallel()
		// The recorded half-section's inradius is 5, the solid cylinder's
		// cavity limit min(R, H/2) = 10: a 7 mm wall leaves the cavity
		// z ∈ [7, 13], ρ ∈ [0, 3], ∫ρ dA = 27.
		doc := decad.New()
		cyl := revolveMeridian(t, doc, cylinderMeridian, halfTurn)
		shelled, err := cyl.Shell(t.Context(), bothAngularCaps(), units.Millimeters(7))
		require.NoError(t, err)
		decadtest.MeasuresVolume(t, shelled, halfTurnVolume(1000-27))
	})
	t.Run("outward", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		cyl := revolveMeridian(t, doc, cylinderMeridian, halfTurn)
		shelled, err := cyl.Shell(t.Context(), bothAngularCaps(), units.Millimeters(2), decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		requireManifold(t, shelled)
		// The dilation is z ∈ [0, 20], ρ ∈ [0, 12] (1440), two end strips
		// ρ ∈ [0, 10] 2 mm long (100 each), and two quarter disks of radius 2
		// at the rims (10π + 8/3 each). The axis ends meet the axis at a right
		// angle, so the dilation closes there along the axis, not around it.
		decadtest.MeasuresVolume(t, shelled, halfTurnVolume(1440+200+20*math.Pi+16.0/3-1000))
		require.ElementsMatch(t, []float64{10, 12}, cylinderRadii(t, shelled))
	})
}

func TestRevolveShellConeApex(t *testing.T) {
	t.Parallel()
	// The apex is a corner of the mirror union. Inward it is a miter: the
	// cavity's apex is the slant's offset line met with the axis, and its base
	// the 1 mm offset of the base disk — a right triangle with legs
	// L = 19 − √5 on the axis and L/2 across it. A right triangle with one leg
	// b on the axis and height h has ∫ρ dA = b·h²/6, so the wall's is
	// (20³ − L³)/24. Outward it is an arc about the apex from the slant's
	// outward normal down to the axis. The wall is then the 1 mm strips along
	// the slant (50√5 + 10) and the base (50), the rim's sector between their
	// outward normals (5(π − atan 2) + (1 + 1/√5)/3) and the apex's half
	// sector ((1 − 1/√5)/3).
	l := 19 - math.Sqrt(5)
	inward := (8000 - l*l*l) / 24
	outward := 50*math.Sqrt(5) + 60 + 5*(math.Pi-math.Atan(2)) + 2.0/3
	// The leading cone has its apex where the kept chain leaves the axis, the
	// trailing one (its mirror image in z = 10) where the chain arrives, so
	// each of the two end joins is read in both senses.
	leading := coneMeridian
	trailing := [][2]float64{{0, 0}, {20, 0}, {20, 10}}
	for _, tc := range []struct {
		name     string
		meridian [][2]float64
		opts     []decad.ShellOption
		want     float64
	}{
		{"leading inward", leading, nil, inward},
		{"trailing inward", trailing, nil, inward},
		{"leading outward", leading, []decad.ShellOption{decad.WithShellSense(decad.Outward)}, outward},
		{"trailing outward", trailing, []decad.ShellOption{decad.WithShellSense(decad.Outward)}, outward},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			cone := revolveMeridian(t, doc, tc.meridian, halfTurn)
			shelled, err := cone.Shell(t.Context(), bothAngularCaps(), units.Millimeters(1), tc.opts...)
			require.NoError(t, err)
			requireManifold(t, shelled)
			decadtest.MeasuresVolume(t, shelled, halfTurnVolume(tc.want))
		})
	}
}

func TestRevolveShellPlaced(t *testing.T) {
	t.Parallel()
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(40))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(40, -15, 7))
	require.NoError(t, err)
	motion, err := rot.Then(shift)
	require.NoError(t, err)
	doc := decad.New()
	cyl := revolveMeridian(t, doc, cylinderMeridian, halfTurn)
	placed, err := cyl.Placed(t.Context(), motion)
	require.NoError(t, err)
	// Both caps turned with the body, so their shared normal is the image of z.
	caps := decad.Faces(decad.NormalTo(motion.ApplyDir(r3.NewVec(0, 0, 1)))).Exactly(2)
	shelled, err := placed.Shell(t.Context(), caps, units.Millimeters(2))
	require.NoError(t, err)
	requireManifold(t, shelled)
	decadtest.MeasuresVolume(t, shelled, halfTurnVolume(488))

	// Roles index the result's own wall region and survive a further
	// placement: both rim bands resolve by role before and after, and every
	// face carries exactly one role of the shell step.
	roles := func(b *decad.Body) []string {
		var out []string
		for _, f := range b.Faces() {
			origins := f.Origins()
			require.Len(t, origins, 1)
			out = append(out, origins[0].Role)
		}
		return out
	}
	up, err := r3.Translation(r3.NewVec(0, 0, 50))
	require.NoError(t, err)
	moved, err := shelled.Placed(t.Context(), up)
	require.NoError(t, err)
	for _, b := range []*decad.Body{shelled, moved} {
		for _, ref := range []decad.FeatureRef{decad.CapStart(b), decad.CapEnd(b)} {
			_, err := decad.Faces(decad.FaceCreatedBy(ref)).Exactly(1).SelectFaces(b)
			require.NoError(t, err)
		}
		sides := 0
		for _, r := range roles(b) {
			if strings.HasPrefix(r, "side(0,") {
				sides++
			}
		}
		require.Equal(t, 6, sides, `the C-shaped wall region's six off-axis walks each sweep one side face`)
	}
	require.ElementsMatch(t, roles(shelled), roles(moved))
	decadtest.MeasuresVolume(t, moved, halfTurnVolume(488))
}

func TestRevolveShellRefusals(t *testing.T) {
	t.Parallel()
	t.Run("full turn opens a side face", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, decad.FullRevolution{})
		_, err := ring.Shell(t.Context(), decad.Faces(decad.Planar()), units.Millimeters(1))
		requireShellRefused(t, doc, ring, err, decad.ErrUnsupported, `§9.2`)
	})
	t.Run("kept angular cap", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, halfTurn)
		_, err := ring.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapStart(ring))), units.Millimeters(1))
		requireShellRefused(t, doc, ring, err, decad.ErrUnsupported, `SX8`)
	})
	t.Run("both caps and a side face", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, halfTurn)
		_, err := ring.Shell(t.Context(), decad.Faces(decad.Planar()).Exactly(4), units.Millimeters(1))
		requireShellRefused(t, doc, ring, err, decad.ErrUnsupported, `§9.2`)
	})
	t.Run("holed meridian", func(t *testing.T) {
		t.Parallel()
		s, p := holedSketch(t)
		doc := decad.New()
		body, err := doc.Revolve(s, p, uAxis, halfTurn)
		require.NoError(t, err)
		_, err = body.Shell(t.Context(), bothAngularCaps(), units.Millimeters(1))
		requireShellRefused(t, doc, body, err, decad.ErrUnsupported, `SX8`)
	})
	t.Run("two axis walks", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		spool := revolveMeridian(t, doc, [][2]float64{{0, 0}, {5, 0}, {5, 5}, {15, 5}, {15, 0}, {20, 0}, {20, 10}, {0, 10}}, halfTurn)
		_, err := spool.Shell(t.Context(), bothAngularCaps(), units.Millimeters(1))
		requireShellRefused(t, doc, spool, err, decad.ErrUnsupported, `more than one walk`)
	})
	t.Run("outward wall reaching across the axis", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, [][2]float64{{0, 1}, {20, 1}, {20, 5}, {0, 5}}, halfTurn)
		_, err := ring.Shell(t.Context(), bothAngularCaps(), units.Millimeters(2), decad.WithShellSense(decad.Outward))
		requireShellRefused(t, doc, ring, err, decad.ErrUnsupported, `mirror image`)
	})
	t.Run("inward offset crossing its mirror image on the axis", func(t *testing.T) {
		t.Parallel()
		// A 2 mm disk on the axis carrying a ring ρ ∈ [5, 10]: the ring keeps a
		// 1.5 mm wall's cavity, but the disk's two faces, eroded 1.5 mm each,
		// pass each other on the axis, so the cavity would split.
		doc := decad.New()
		flange := revolveMeridian(t, doc, [][2]float64{{0, 0}, {2, 0}, {2, 5}, {20, 5}, {20, 10}, {0, 10}}, halfTurn)
		_, err := flange.Shell(t.Context(), bothAngularCaps(), units.Millimeters(1.5))
		requireShellRefused(t, doc, flange, err, decad.ErrUnsupported, `in order`)
	})
	t.Run("inward wall at the cavity limit", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		cyl := revolveMeridian(t, doc, cylinderMeridian, halfTurn)
		_, err := cyl.Shell(t.Context(), bothAngularCaps(), units.Millimeters(10))
		requireShellRefused(t, doc, cyl, err, decad.ErrDegenerate, `inradius`)
	})
}

func TestRevolveShellTessellates(t *testing.T) {
	t.Parallel()
	// Table DX row DX3: the result is an ordinary revolvePayload, so the
	// revolve tessellator meshes it watertight, the two on-axis walks of the
	// cylinder's wall region included, and its occupied volume agrees with
	// the analytic one.
	for _, tc := range []struct {
		name     string
		meridian [][2]float64
	}{
		{"ring", ringMeridian},
		{"cylinder", cylinderMeridian},
		{"cone", coneMeridian},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			shelled, err := revolveMeridian(t, decad.New(), tc.meridian, halfTurn).
				Shell(t.Context(), bothAngularCaps(), units.Millimeters(1))
			require.NoError(t, err)
			mesh, err := shelled.Tessellate(t.Context(), units.Millimeters(0.05))
			require.NoError(t, err)
			requireWatertight(t, mesh)
			vol, err := shelled.Volume()
			require.NoError(t, err)
			require.InEpsilon(t, vol.Value.Mag(), meshVolume(mesh), 0.01)
		})
	}
}

// This half covers the closed full-turn shell of docs/modify-reach-design.md
// §9.3 (Table BX row BX6): Shell(nil, t, WithNoOpenings()) on a full revolve.
// The wall region is the same one a partial turn sweeps, swept a whole turn,
// so V = 2π·∫ρ dA over it. An off-axis meridian's wall region is the meridian
// with its offset as a hole loop, which the full turn sweeps into a void
// shell. A meridian with an on-axis walk gives a wall region of one loop that
// meets the axis twice; the full turn sweeps its two off-axis runs into two
// closed surfaces, the outer skin and the cavity wall.
//
// Legs shown to fail before these fixtures were accepted: building every run
// of a loop into one shell leaves the hollow cylinder, sphere and shaft with
// one non-void shell, and the direct revolve of a cavity meridian likewise;
// taking the loop's first run as the outer shell, rather than the run whose
// axis ends bracket the others, marks the spool drawn from its bite inside
// out; and stepping the sphere's G1 end join along the arc's float normal
// leaves the cavity's pole 1.2e-16 below the axis, so the inward hollow
// sphere refuses as SX8.

// fullTurnVolume is Pappus over a full turn.
func fullTurnVolume(q float64) units.Value { return units.CubicMillimeters(2 * math.Pi * q) }

// sphereBody revolves semicircleSketch's half disk, radius 5 about (5, 0), a
// full turn.
func sphereBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, p := semicircleSketch(t)
	body, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// shellArea sums the areas of a shell's faces in square millimetres.
func shellArea(t *testing.T, sh *decad.Shell) float64 {
	t.Helper()
	total := 0.0
	for _, f := range sh.Faces() {
		a, err := f.Area()
		require.NoError(t, err)
		mm2, err := a.Value.In(units.SquareMillimeter)
		require.NoError(t, err)
		total += mm2
	}
	return total
}

// requireOuterAndVoid asserts the body is one lump of exactly two shells, the
// second of them the one void shell, and returns them outer first.
func requireOuterAndVoid(t *testing.T, b *decad.Body) (*decad.Shell, *decad.Shell) {
	t.Helper()
	lumps := b.Lumps()
	require.Len(t, lumps, 1, `a closed shell is one piece of material`)
	shells := lumps[0].Shells()
	require.Len(t, shells, 2, `an outer skin and a cavity wall`)
	require.False(t, shells[0].IsVoid(), `the outer skin bounds no cavity`)
	require.True(t, shells[1].IsVoid(), `the cavity wall is the void shell`)
	for _, sh := range shells {
		require.False(t, sh.IsOpen())
	}
	return shells[0], shells[1]
}

func TestRevolveShellClosed(t *testing.T) {
	t.Parallel()
	outward := []decad.ShellOption{decad.WithNoOpenings(), decad.WithShellSense(decad.Outward)}
	inward := []decad.ShellOption{decad.WithNoOpenings()}
	ring := func(t *testing.T, doc *decad.Document) *decad.Body {
		return revolveMeridian(t, doc, ringMeridian, decad.FullRevolution{})
	}
	cylinder := func(t *testing.T, doc *decad.Document) *decad.Body {
		return revolveMeridian(t, doc, cylinderMeridian, decad.FullRevolution{})
	}
	torus := func(t *testing.T, doc *decad.Document) *decad.Body { return torusBody(t, doc, 10, 3) }
	// Every case takes a 1 mm wall. wall is the wall region's ∫ρ dA. skin is
	// the area of the surface the call keeps where it was: the receiver's
	// outer skin inward, which becomes the cavity wall outward. other is the
	// area of the surface the offset makes.
	for _, tc := range []struct {
		name  string
		body  func(*testing.T, *decad.Document) *decad.Body
		opts  []decad.ShellOption
		wall  float64
		skin  float64
		other float64
		// surveyed is false where the wall survey leaves the receiver itself
		// undecided: a sphere meridian's arc meets the axis.
		surveyed bool
	}{
		// A torus of tube radius 3 at 10 from the axis: ∫ρ dA = 10·π r², the
		// area 4π²·10·r.
		{"torus inward", torus, inward, 10 * math.Pi * (9 - 4), 4 * math.Pi * math.Pi * 30, 4 * math.Pi * math.Pi * 20, true},
		{"torus outward", torus, outward, 10 * math.Pi * (16 - 9), 4 * math.Pi * math.Pi * 30, 4 * math.Pi * math.Pi * 40, true},
		// ringMeridian: the cavity z ∈ [1, 19], ρ ∈ [6, 9] inward; outward
		// the dilation of TestRevolveShellOffAxisRing's half turn.
		{"ring inward", ring, inward, 345, 2*math.Pi*(5*20+10*20) + 2*math.Pi*75, 2*math.Pi*(6*18+9*18) + 2*math.Pi*45,
			true},
		{"ring outward", ring, outward, 375 + 7.5*math.Pi, 2*math.Pi*(5*20+10*20) + 2*math.Pi*75,
			// Two cylinders of radii 4 and 11 over z ∈ [0, 20], two annuli
			// ρ ∈ [5, 10] at z = −1 and 21, and four quarter tori of tube radius
			// 1 swept around radii 5 and 10, two each: 2π·(π/2·ρ ∓ 1) each.
			2*math.Pi*(4*20+11*20) + 2*math.Pi*75 + 2*2*math.Pi*(math.Pi/2*5-1) + 2*2*math.Pi*(math.Pi/2*10+1),
			true},
		// cylinderMeridian, a flat-ended solid cylinder R = 10, H = 20: the
		// cavity z ∈ [1, 19], ρ ∈ [0, 9], so ∫ρ dA = 1000 − 81/2·18.
		{"cylinder inward", cylinder, inward, 1000 - 729, 2*math.Pi*10*20 + 2*math.Pi*100, 2*math.Pi*9*18 + 2*math.Pi*81, true},
		// Outward: z ∈ [0, 20], ρ ∈ [0, 11] (1210), the end disks moved 1 mm
		// out (50 each), and two quarter disks of radius 1 at the rims
		// (π/4·10 + 1/3 each). The skin gains the cylinder radius 11, two end
		// disks of radius 10 and two quarter tori around radius 10.
		{"cylinder outward", cylinder, outward, 1210 + 100 + 5*math.Pi + 2.0/3 - 1000, 2*math.Pi*10*20 + 2*math.Pi*100,
			2*math.Pi*11*20 + 2*math.Pi*100 + 2*2*math.Pi*(math.Pi/2*10+1), true},
		// A half disk of radius r has ∫ρ dA = 2r³/3 and sweeps a sphere of
		// area 4πr².
		{"sphere inward", sphereBody, inward, 2.0 * (125 - 64) / 3, 100 * math.Pi, 64 * math.Pi, false},
		{"sphere outward", sphereBody, outward, 2.0 * (216 - 125) / 3, 100 * math.Pi, 144 * math.Pi, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			receiver := tc.body(t, doc)
			shelled, err := receiver.Shell(t.Context(), nil, units.Millimeters(1), tc.opts...)
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{shelled}, doc.Bodies(), `the shell retires its receiver`)
			require.True(t, shelled.IsSolid())
			requireManifold(t, shelled)
			decadtest.MeasuresVolume(t, shelled, fullTurnVolume(tc.wall))

			outer, cavity := requireOuterAndVoid(t, shelled)
			outerArea, cavityArea := tc.skin, tc.other
			if len(tc.opts) > 1 { // outward: the receiver's skin is the cavity
				outerArea, cavityArea = tc.other, tc.skin
			}
			require.InEpsilon(t, outerArea, shellArea(t, outer), 1e-9)
			require.InEpsilon(t, cavityArea, shellArea(t, cavity), 1e-9)

			// DX3: the revolve tessellator meshes both shells watertight, and
			// the mesh's occupied volume, the cavity subtracted, agrees with
			// the analytic one.
			mesh, err := shelled.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			requireWatertight(t, mesh)
			vol, err := shelled.Volume()
			require.NoError(t, err)
			require.InEpsilon(t, vol.Value.Mag(), meshVolume(mesh), 0.03)

			// DX9: the wall survey reads the shell thickness where it reads
			// the receiver at all.
			report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(0.5)))
			require.NoError(t, err)
			br := report.Bodies[0]
			if !tc.surveyed {
				require.Equal(t, decad.ScalarUndecided, br.Wall.Outcome)
				require.Equal(t, decad.Suspect, report.Status)
				return
			}
			require.Equal(t, decad.Sound, report.Status)
			require.Equal(t, decad.ScalarMeasured, br.Wall.Outcome)
			require.NotNil(t, br.Wall.Minimum)
			decadtest.Measures(t, "wall", br.Wall.Minimum.Measurement, units.Millimeters(1))
		})
	}
}

func TestRevolveShellClosedShaft(t *testing.T) {
	t.Parallel()
	// shaftMeridian's 1 mm cavity is the mirror union's erosion cut back to
	// the axis: z ∈ [1, 9], ρ ∈ [0, 9] beside z ∈ [9, 19], ρ ∈ [0, 4]
	// (∫ρ dA = 81/2·8 + 16/2·10 = 404), joined at the shoulder's concave
	// corner (10, 5) by an arc of radius 1 about it. The arc adds the square
	// z ∈ [9, 10], ρ ∈ [4, 5] (4.5) less the quarter disk about the corner,
	// whose centroid sits 4/(3π) below ρ = 5 (5π/4 − 1/3).
	doc := decad.New()
	shaft := revolveMeridian(t, doc, shaftMeridian, decad.FullRevolution{})
	shelled, err := shaft.Shell(t.Context(), nil, units.Millimeters(1), decad.WithNoOpenings())
	require.NoError(t, err)
	requireManifold(t, shelled)
	decadtest.MeasuresVolume(t, shelled, fullTurnVolume(shaftQ-404-4.5-1.0/3+5*math.Pi/4))
	_, cavity := requireOuterAndVoid(t, shelled)
	var radii []float64
	for _, f := range cavity.Faces() {
		if c, ok := f.Surface().(decad.Cylinder); ok {
			r, err := c.Radius.In(units.Millimeter)
			require.NoError(t, err)
			radii = append(radii, r)
		}
	}
	require.ElementsMatch(t, []float64{4, 9}, radii, `the cavity's two journals`)
}

func TestRevolveCavityMeridianSplitsShells(t *testing.T) {
	t.Parallel()
	// The spool meridian meets the axis along two walks: the rectangle
	// z ∈ [0, 20], ρ ∈ [0, 10] less the bite z ∈ [5, 15], ρ ∈ [0, 5]. A full
	// turn is a solid cylinder holding a closed cylindrical cavity, so its
	// boundary is two shells, the bite's the void one. The outer shell is the
	// run whose axis ends bracket the other's, wherever the loop's walk
	// starts: drawn from the outer corner and from the bite alike.
	for _, tc := range []struct {
		name string
		pts  [][2]float64
	}{
		{"outer first", [][2]float64{{0, 0}, {5, 0}, {5, 5}, {15, 5}, {15, 0}, {20, 0}, {20, 10}, {0, 10}}},
		{"bite first", [][2]float64{{5, 5}, {15, 5}, {15, 0}, {20, 0}, {20, 10}, {0, 10}, {0, 0}, {5, 0}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			spool := revolveMeridian(t, doc, tc.pts, decad.FullRevolution{})
			decadtest.MeasuresVolume(t, spool, fullTurnVolume(1000-125))
			outer, cavity := requireOuterAndVoid(t, spool)
			require.InEpsilon(t, 2*math.Pi*10*20+2*math.Pi*100, shellArea(t, outer), 1e-9)
			require.InEpsilon(t, 2*math.Pi*5*10+2*math.Pi*25, shellArea(t, cavity), 1e-9)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, 1, report.Bodies[0].Topology.Voids)
		})
	}
}

func TestRevolveShellClosedRefusals(t *testing.T) {
	t.Parallel()
	noOpenings := decad.WithNoOpenings()
	t.Run("partial turn", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, halfTurn)
		_, err := ring.Shell(t.Context(), nil, units.Millimeters(1), noOpenings)
		requireShellRefused(t, doc, ring, err, decad.ErrUnsupported, `SX8`)
	})
	t.Run("holed meridian", func(t *testing.T) {
		t.Parallel()
		s, p := holedSketch(t)
		doc := decad.New()
		body, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		_, err = body.Shell(t.Context(), nil, units.Millimeters(1), noOpenings)
		requireShellRefused(t, doc, body, err, decad.ErrUnsupported, `SX8`)
	})
	t.Run("two axis walks", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		spool := revolveMeridian(t, doc, [][2]float64{{0, 0}, {5, 0}, {5, 5}, {15, 5}, {15, 0}, {20, 0}, {20, 10}, {0, 10}}, decad.FullRevolution{})
		_, err := spool.Shell(t.Context(), nil, units.Millimeters(1), noOpenings)
		requireShellRefused(t, doc, spool, err, decad.ErrUnsupported, `more than one walk`)
	})
	t.Run("outward wall reaching across the axis", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, [][2]float64{{0, 1}, {20, 1}, {20, 5}, {0, 5}}, decad.FullRevolution{})
		_, err := ring.Shell(t.Context(), nil, units.Millimeters(2), noOpenings, decad.WithShellSense(decad.Outward))
		requireShellRefused(t, doc, ring, err, decad.ErrUnsupported, `mirror image`)
	})
	t.Run("inward wall at the cavity limit", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		cyl := revolveMeridian(t, doc, cylinderMeridian, decad.FullRevolution{})
		_, err := cyl.Shell(t.Context(), nil, units.Millimeters(10), noOpenings)
		requireShellRefused(t, doc, cyl, err, decad.ErrDegenerate, `inradius`)
	})
	t.Run("selector beside the option", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ring := revolveMeridian(t, doc, ringMeridian, decad.FullRevolution{})
		_, err := ring.Shell(t.Context(), decad.Faces(decad.Planar()), units.Millimeters(1), noOpenings)
		requireShellRefused(t, doc, ring, err, decad.ErrDegenerate, `SX1`)
	})
}
