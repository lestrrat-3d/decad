package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// opposedArcBody is one pocket tool of an opposed-arc scene.
type opposedArcBody func(*testing.T, *decad.Document) *decad.Body

func opposedArcBox(x0, y0, x1, y1, z0, h float64) opposedArcBody {
	return func(t *testing.T, doc *decad.Document) *decad.Body { return boxBodyAtZ(t, doc, x0, y0, x1, y1, z0, h) }
}

func opposedArcRound(x, r, z0, h float64) opposedArcBody {
	return func(t *testing.T, doc *decad.Document) *decad.Body { return circleBodyAtZ(t, doc, x, r, z0, h) }
}

func opposedArcPlate(t *testing.T, doc *decad.Document) *decad.Body {
	return boxBody(t, doc, -10, -10, 10, 10, 10)
}

// opposedArcPockets cuts the plate with the two pockets in the given order
// and returns the second cut's result. The first cut must leave a stacked
// payload, so the second tool ends at its interface.
func opposedArcPockets(t *testing.T, doc *decad.Document, plate, top, bottom opposedArcBody, topFirst bool) (*decad.Body, error) {
	t.Helper()
	first, second := top, bottom
	if !topFirst {
		first, second = bottom, top
	}
	part, err := decad.Cut(t.Context(), plate(t, doc), first(t, doc))
	require.NoError(t, err)
	require.Equal(t, "decad.stackedPrismPayload", decadtest.PayloadClass(part))
	return decad.Cut(t.Context(), part, second(t, doc))
}

// TestStackedOpposedArcLoops cuts pockets from opposite faces of a
// 20 x 20 x 10 plate that meet at z = 4 and overlap there in plan, where a
// plan loop holds circles or arcs. The pockets share no volume, so each
// closed form is the plate's 4000 mm^3 less both pockets' own volumes:
//
//   - round-across-rect: a 6 x 6 top pocket 6 deep and an r = 3 bottom
//     pocket 4 deep about (1, 0), which crosses the rectangle's edge x = 0 at
//     y = ±sqrt(8): 4000 - 216 - 36*pi;
//   - round-in-rect: the same rectangle over an r = 1 bottom pocket inside
//     it, which leaves a floor with a round hole: 4000 - 216 - 4*pi;
//   - rect-in-round: an r = 3 top pocket over a 2 x 2 bottom pocket inside
//     it: 4000 - 54*pi - 16;
//   - concentric: r = 3 over r = 1, a counterbore drilled from both faces:
//     4000 - 54*pi - 4*pi;
//   - corner-on-round: an r = 5 top pocket about the origin over a 4 x 12
//     bottom pocket whose corner (3, 4) lies on the circle from inside it, and
//     which crosses the circle at x = -1 and x = 3 below: 4000 - 150*pi - 192;
//   - slot-across-rect: a slot (24 + 4*pi mm^2) 6 deep over a 6 x 2 bottom
//     pocket crossing its right arc at x = 3 + sqrt(3): 4000 - 144 - 24*pi - 48;
//   - round-across-rect/x1024: the first scene moved 1024 mm along x, where
//     the line-circle vertices' rounding grows with the coordinates.
//
// Shown red: restoring the axis-line gate (stackedrecord's opposedLoop
// refusing circles and arcs) sends every case to the mesh path, which
// refuses the coplanar contact.
func TestStackedOpposedArcLoops(t *testing.T) {
	t.Parallel()
	slot := func(t *testing.T, doc *decad.Document) *decad.Body {
		return arcPocketBody(t, doc, 4, units.Millimeters(6), decad.Along, drawCrossSlot)
	}
	for _, tc := range []struct {
		name               string
		plate, top, bottom opposedArcBody
		// k and p state the closed form k + p*pi as arcVolume reads it.
		k, p     string
		maxBound float64
	}{
		{name: "round-across-rect", plate: opposedArcPlate, top: opposedArcBox(-6, -3, 0, 3, 4, 6),
			bottom: opposedArcRound(1, 3, 0, 4), k: "3784", p: "-36", maxBound: 1e-10},
		{name: "round-in-rect", plate: opposedArcPlate, top: opposedArcBox(-6, -3, 0, 3, 4, 6),
			bottom: opposedArcRound(-3, 1, 0, 4), k: "3784", p: "-4", maxBound: 1e-10},
		{name: "rect-in-round", plate: opposedArcPlate, top: opposedArcRound(-2, 3, 4, 6),
			bottom: opposedArcBox(-3, -1, -1, 1, 0, 4), k: "3984", p: "-54", maxBound: 1e-10},
		{name: "concentric", plate: opposedArcPlate, top: opposedArcRound(-2, 3, 4, 6),
			bottom: opposedArcRound(-2, 1, 0, 4), k: "4000", p: "-58", maxBound: 1e-10},
		{name: "corner-on-round", plate: opposedArcPlate, top: opposedArcRound(0, 5, 4, 6),
			bottom: opposedArcBox(-1, -8, 3, 4, 0, 4), k: "3808", p: "-150", maxBound: 1e-10},
		{name: "slot-across-rect", plate: opposedArcPlate, top: slot,
			bottom: opposedArcBox(2, -1, 8, 1, 0, 4), k: "3808", p: "-24", maxBound: 1e-10},
		{name: "round-across-rect/x1024",
			plate: func(t *testing.T, doc *decad.Document) *decad.Body {
				return boxBody(t, doc, 1014, -10, 1034, 10, 10)
			}, top: opposedArcBox(1018, -3, 1024, 3, 4, 6),
			bottom: opposedArcRound(1025, 3, 0, 4), k: "3784", p: "-36", maxBound: 1e-8},
	} {
		for _, topFirst := range []bool{true, false} {
			order := map[bool]string{true: "top-first", false: "bottom-first"}[topFirst]
			t.Run(tc.name+"/"+order, func(t *testing.T) {
				t.Parallel()
				doc := decad.New()
				part, err := opposedArcPockets(t, doc, tc.plate, tc.top, tc.bottom, topFirst)
				require.NoError(t, err)
				require.Equal(t, "decad.brepPayload", decadtest.PayloadClass(part))
				requireEveryEdgeOnTwoFaces(t, part)
				var cylinders int
				for _, f := range part.Faces() {
					switch s := f.Surface().(type) {
					case decad.Plane:
					case decad.Cylinder:
						cylinders++
					default:
						require.Failf(t, "unexpected surface", "%T", s)
					}
				}
				require.Positive(t, cylinders)

				volume, err := part.Volume()
				require.NoError(t, err)
				require.Equal(t, decad.Approximate, volume.Exactness)
				bound := boundMM3(t, volume)
				require.Positive(t, bound)
				require.Less(t, bound, tc.maxBound)
				gap := new(big.Float).SetPrec(volumeRefPrec).SetFloat64(volumeMM(t, volume))
				gap.Sub(gap, arcVolume(t, tc.k, tc.p, "0")).Abs(gap)
				require.LessOrEqual(t, gap.Cmp(new(big.Float).SetFloat64(bound)), 0,
					"volume %v misses the closed form by %s, bound %v", volumeMM(t, volume), gap.Text('g', 6), bound)

				mesh, err := part.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
				require.NoError(t, err)
				require.True(t, mesh.VolumeVerified())
				requireWatertight(t, mesh)
				report, err := doc.Verify(t.Context())
				require.NoError(t, err)
				require.Equal(t, decad.Sound, report.Status)
			})
		}
	}
}

// TestStackedOpposedArcLoopsRefuse keeps opposed pockets with circles or
// arcs that this route does not state off it. Under ExactOnly the mesh path
// refuses their coplanar contact.
//
//   - l-tangent: an r = 3 bottom pocket about the origin enters an L-shaped
//     top pocket across its edge x = -2 and touches its inner edge y = -3 at
//     (0, -3) from outside, where the floor and the ceiling would meet at one
//     point. The arrangement does not cut the tangency, so no scene vertex
//     shows it.
//   - tangent-inside: an r = 2 bottom pocket about (-1, 0) crosses a top
//     rectangle's edge x = 0 and touches its edges y = ±2 from inside.
//   - crossing-circles: two r = 2 pockets whose circles cross, which no exact
//     vertex record states.
//   - oblique: a top pocket with an oblique edge crossing a bottom rectangle.
//
// Shown red: dropping the CurvedCrossingsCut refusal from stackedunion's
// opposedCells builds l-tangent as a brep. The other cases also miss later
// in the build (the face audit for tangent-inside, the engine's two-circle
// junction for crossing-circles) or before it (the oblique-line gate).
func TestStackedOpposedArcLoopsRefuse(t *testing.T) {
	t.Parallel()
	lPocket := func(t *testing.T, doc *decad.Document) *decad.Body {
		return arcPocketBody(t, doc, 4, units.Millimeters(6), decad.Along, drawPolygon(
			[2]float64{-6, -7}, [2]float64{5, -7}, [2]float64{5, -3}, [2]float64{-2, -3},
			[2]float64{-2, 7}, [2]float64{-6, 7}))
	}
	oblique := func(t *testing.T, doc *decad.Document) *decad.Body {
		return arcPocketBody(t, doc, 4, units.Millimeters(6), decad.Along, drawPolygon(
			[2]float64{-6, -3}, [2]float64{0, -3}, [2]float64{-2, 3}, [2]float64{-6, 3}))
	}
	for _, tc := range []struct {
		name        string
		top, bottom opposedArcBody
	}{
		{name: "l-tangent", top: lPocket, bottom: opposedArcRound(0, 3, 0, 4)},
		{name: "tangent-inside", top: opposedArcBox(-6, -2, 0, 2, 4, 6), bottom: opposedArcRound(-1, 2, 0, 4)},
		{name: "crossing-circles", top: opposedArcRound(-2, 2, 4, 6), bottom: opposedArcRound(0, 2, 0, 4)},
		{name: "oblique", top: oblique, bottom: opposedArcBox(-3, -1, 2, 1, 0, 4)},
		{name: "slot-tangent-edge", top: opposedArcBox(-1, -1, 8, 2, 4, 6), bottom: func(t *testing.T, doc *decad.Document) *decad.Body {
			return arcPocketBody(t, doc, 0, units.Millimeters(4), decad.Along, drawCrossSlot)
		}},
	} {
		for _, topFirst := range []bool{true, false} {
			order := map[bool]string{true: "top-first", false: "bottom-first"}[topFirst]
			t.Run(tc.name+"/"+order, func(t *testing.T) {
				t.Parallel()
				doc := decad.New(decad.ExactOnly())
				_, err := opposedArcPockets(t, doc, opposedArcPlate, tc.top, tc.bottom, topFirst)
				require.ErrorIs(t, err, decad.ErrUnsupported)
				var be *decad.BooleanError
				require.ErrorAs(t, err, &be)
				require.Equal(t, decad.BooleanUnsupportedContact, be.Code)
			})
		}
	}
}
