package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// A cap-loop chamfer's reflex corner (a hole's convex corner, here a D bore's)
// composes with later booleans: agent-docs/tessellation-reach-design.md §7,
// "Reflex connector", gives the apex fan a vertex bound and a chord term.

const (
	reflexR, reflexRb, reflexFb, reflexRh, reflexH, reflexD = 18.1, 4.1, 3.6, 1.3, 5.0, 0.5
)

// reflexDPlate is a circle R18.1 plate with a D bore (an arc of radius 4.1 the
// long way and a flat at y = 3.6, as models/onewayclutch sketches it) and three
// r1.3 holes at 8.4 mm on 30, 150 and 270 degrees, 5 mm thick.
func reflexDPlate(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	s.AddConstraint(sketch.NewRadius(s.CreateCircle(o, reflexR), reflexR))
	hw := math.Sqrt(reflexRb*reflexRb - reflexFb*reflexFb)
	q1, q2 := s.CreatePoint(-hw, reflexFb), s.CreatePoint(hw, reflexFb)
	bore := s.CreateArc(o, q1, q2)
	flat := s.CreateLine(q2, q1)
	s.AddConstraint(sketch.NewRadius(bore, reflexRb), sketch.NewHorizontal(flat),
		sketch.NewDistancePointLine(o, flat, reflexFb))
	for _, a := range []float64{30, 150, 270} {
		c := s.CreatePoint(8.4*math.Cos(a*math.Pi/180), 8.4*math.Sin(a*math.Pi/180))
		s.Fix(c)
		s.AddConstraint(sketch.NewRadius(s.CreateCircle(c, reflexRh), reflexRh))
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var pr *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 4 {
			pr = p
		}
	}
	require.NotNil(t, pr)
	body, err := decad.Extrude(t.Context(), doc, s, pr, decad.Distance{D: units.Millimeters(reflexH), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// reflexDPlateChamfered chamfers every start-cap loop of a fresh D plate.
func reflexDPlateChamfered(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	plate := reflexDPlate(t, doc)
	ch, err := decad.Chamfer(t.Context(), plate,
		decad.Edges(decad.CreatedBy(decad.CapStart(plate))), units.Millimeters(reflexD))
	require.NoError(t, err)
	return ch
}

// reflexDPlateVolume is the chamfered plate's closed form: the prism, less the
// outer ring's chamfer, the D loop's dilation band (Steiner, the D being
// convex: P d^2/2 + pi d^3/3) and each hole's band.
func reflexDPlateVolume() float64 {
	half := math.Sqrt(reflexRb*reflexRb - reflexFb*reflexFb)
	theta := math.Acos(reflexFb / reflexRb)
	areaD := reflexRb*reflexRb*(math.Pi-theta) + reflexFb*half
	perimD := 2*reflexRb*(math.Pi-theta) + 2*half
	prism := (math.Pi*reflexR*reflexR - areaD - 3*math.Pi*reflexRh*reflexRh) * reflexH
	ring := math.Pi * reflexD * reflexD * (reflexR - reflexD/3)
	dband := perimD*reflexD*reflexD/2 + math.Pi*reflexD*reflexD*reflexD/3
	holes := 3 * (math.Pi*reflexRh*reflexD*reflexD + math.Pi*reflexD*reflexD*reflexD/3)
	return prism - ring - dband - holes
}

// TestCapBlendReflexDBoreCut cuts boxes from a D-bore plate whose every
// start-cap loop is chamfered, which puts a reflex corner (the D's flat meeting
// its arc) in the chamfer band. Each cut's volume must match the closed form
// within its own published bound.
func TestCapBlendReflexDBoreCut(t *testing.T) {
	t.Parallel()
	v0 := reflexDPlateVolume()

	t.Run("sequential cuts clear of the band", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		ch := reflexDPlateChamfered(t, doc)
		vol, err := ch.Volume()
		require.NoError(t, err)
		require.InDelta(t, v0, volumeMM(t, vol), 1e-6, `the chamfer itself is exact`)

		cut, err := decad.Cut(t.Context(), ch, boxBodyAtZ(t, doc, 11, -2, 15, 2, -1, 8))
		require.NoError(t, err)
		cv, err := cut.Volume()
		require.NoError(t, err)
		bound := boundMM3(t, cv)
		require.Greater(t, bound, 0.0)
		require.Less(t, bound, 0.6)
		require.Equal(t, decad.Approximate, cv.Exactness)
		require.LessOrEqual(t, math.Abs(volumeMM(t, cv)-(v0-80)), bound)
		requireBodyWatertight(t, cut)

		// The second box spans z in [3, 6], so only its 2 mm inside the 5 mm plate cuts: 4 x 4 x 2 = 32 mm3.
		cut2, err := decad.Cut(t.Context(), cut, boxBodyAtZ(t, doc, -15, -2, -11, 2, 3, 3))
		require.NoError(t, err)
		cv2, err := cut2.Volume()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(volumeMM(t, cv2)-(v0-80-32)), boundMM3(t, cv2))
		m, err := cut2.Tessellate(t.Context(), units.Millimeters(0.01))
		require.NoError(t, err)
		requireWatertight(t, m)
	})

	t.Run("box across the D's corner band", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		cutBody, err := decad.Cut(t.Context(), reflexDPlateChamfered(t, doc),
			boxBodyAtZ(t, doc, -3, 2.5, -1, 5, -1, 3))
		require.NoError(t, err)
		doc2 := decad.New()
		inter, err := decad.Intersect(t.Context(), reflexDPlateChamfered(t, doc2),
			boxBodyAtZ(t, doc2, -3, 2.5, -1, 5, -1, 3))
		require.NoError(t, err)
		cv, err := cutBody.Volume()
		require.NoError(t, err)
		iv, err := inter.Volume()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(volumeMM(t, cv)+volumeMM(t, iv)-v0), boundMM3(t, cv)+boundMM3(t, iv))
		requireBodyWatertight(t, cutBody)
		requireBodyWatertight(t, inter)
	})
}

// TestCapBlendReflexLBodyBoolean cuts the 3 mm chamfered L section across its
// reflex corner, where the line-line connector lies, and clear of the band.
func TestCapBlendReflexLBodyBoolean(t *testing.T) {
	t.Parallel()
	mk := func() *decad.Body {
		body := reflexLBody(t)
		ch, err := decad.Chamfer(t.Context(), body, capLoopEdges(body), units.Millimeters(3))
		require.NoError(t, err)
		return ch
	}
	exact := reflexLChamferVolume(3)

	t.Run("across the corner", func(t *testing.T) {
		t.Parallel()
		a, b := mk(), mk()
		cut, err := decad.Cut(t.Context(), a, boxBodyAtZ(t, a.Document(), 15, 15, 25, 25, -1, 30))
		require.NoError(t, err)
		in, err := decad.Intersect(t.Context(), b, boxBodyAtZ(t, b.Document(), 15, 15, 25, 25, -1, 30))
		require.NoError(t, err)
		cv, err := cut.Volume()
		require.NoError(t, err)
		iv, err := in.Volume()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(volumeMM(t, cv)+volumeMM(t, iv)-exact), boundMM3(t, cv)+boundMM3(t, iv))
		requireBodyWatertight(t, cut)
		requireBodyWatertight(t, in)
	})

	t.Run("clear of the band", func(t *testing.T) {
		t.Parallel()
		a := mk()
		// 10 x 10 x 16 mm inside the middle of one arm, away from every band.
		cut, err := decad.Cut(t.Context(), a, boxBodyAtZ(t, a.Document(), 5, 5, 15, 15, 2, 16))
		require.NoError(t, err)
		cv, err := cut.Volume()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(volumeMM(t, cv)-(exact-1600)), boundMM3(t, cv))
		requireBodyWatertight(t, cut)
	})
}
