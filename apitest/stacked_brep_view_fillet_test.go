package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestStackedBrepViewFillet fillets an edge that the brep receiver routes
// refuse, through the retained edge blend on the body's faceted view
// (agent-docs/tangent-notch-design.md T3): the plate's top edge at x = -10 or
// x = 10, on an A7 result (a pocket across a round hole) and on an A1 union (a
// round boss standing across the plate's outline). The circle crossings give
// both records a section displacement, which the receivers refuse. The edge is
// straight, 20 long, between two retained planes and ending at convex corners,
// so a fillet of radius 0.5 removes 20·(1 − π/4)·0.25, and its face is a
// cylinder of radius 0.5 along Y.
//
// Shown red: without the view fallback in brepmodify's Fillet both refuse with
// the receivers' section-displacement reading; without the Stack admission in
// stackedViewBlend the union does.
func TestStackedBrepViewFillet(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	for _, tc := range []struct {
		name  string
		build func(*testing.T, *decad.Document) *decad.Body
		x     float64
	}{
		{name: "a7-pocket-across-hole", x: 10, build: func(t *testing.T, doc *decad.Document) *decad.Body {
			part, err := decad.Cut(t.Context(), boxBody(t, doc, -10, -10, 10, 10, 10), circleBodyAtZ(t, doc, 0, 3, 0, 10))
			require.NoError(t, err)
			part, err = decad.Cut(t.Context(), part, boxBodyAtZ(t, doc, -2, -1.5, 6, 1.5, 6, 4))
			require.NoError(t, err)
			return part
		}},
		{name: "a1-boss-across-outline", x: -10, build: func(t *testing.T, doc *decad.Document) *decad.Body {
			part, err := decad.Union(t.Context(), boxBody(t, doc, -10, -10, 10, 10, 10), circleBodyAtZ(t, doc, 10, 3, 7, 6))
			require.NoError(t, err)
			return part
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			part := tc.build(t, decad.New())
			require.Equal(t, "decad.brepPayload", decadtest.PayloadClass(part))
			before, err := part.Volume()
			require.NoError(t, err)

			edge := decad.Edges(decad.EndpointAt(r3.Vec{X: tc.x, Y: -10, Z: 10}),
				decad.EndpointAt(r3.Vec{X: tc.x, Y: 10, Z: 10})).Exactly(1)
			out, err := decad.Fillet(t.Context(), part, edge, mm(0.5))
			require.NoError(t, err)
			after, err := out.Volume()
			require.NoError(t, err)
			removed := volumeMM(t, before) - volumeMM(t, after)
			require.InDelta(t, 20*(1-math.Pi/4)*0.25, removed, boundMM3(t, before)+boundMM3(t, after))

			var rounds int
			for _, f := range out.Faces() {
				c, ok := f.Surface().(decad.Cylinder)
				if !ok {
					continue
				}
				r, err := c.Radius.In(units.Millimeter)
				require.NoError(t, err)
				if math.Abs(r-0.5) > 1e-12 {
					continue
				}
				require.InDelta(t, 1, math.Abs(c.Axis.Y), 1e-12)
				rounds++
			}
			require.Equal(t, 1, rounds, "one cylinder of radius 0.5 along Y")
			_, err = out.Tessellate(t.Context(), mm(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
		})
	}
}

// TestStackedBrepViewFilletReachesRetainedBlend checks which selections the
// receiver routes refuse reach the retained edge blend on the faceted view. The
// A9 tangent fork notch's floor arc reaches it and stops at the blend's
// tangent-junction rule (retained-modify-reach SM2) until boss-root-overflow P6
// builds its tangent ends. The holed A1 union of the spoked disc and its Ø14
// boss (TestUnionHoledSharedSpanBoss) merged two near-equal circles into one
// carrier, which may lie the span's width from the wall it denotes; the view
// keeps no tag on its curved faces, so its root arcs keep the receiver's
// refusal until boss-root-overflow P5 states that charge on the tag.
//
// Shown red: without the view fallback the fork refuses with route L's SL1;
// without the curved-face untag the boss reaches the blend on a tag that
// states no charge.
func TestStackedBrepViewFilletReachesRetainedBlend(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	t.Run("boss14-charged-carrier", func(t *testing.T) {
		t.Parallel()
		_, disc := spokedHubDisc(t)
		out, err := bossOnCap(t, disc, 14)
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), out, decad.Edges(decad.Reflex(), decad.Circular()), mm(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "brep-modify SB1")
	})
	t.Run("fork", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		fork := forkBody(t, doc, 0, mm(6.5), 0, leverForkSketch(false, forkOptions{}))
		notch := forkBody(t, doc, 2.5, mm(5), 0, leverForkSketch(true, forkOptions{}))
		part, err := decad.Cut(t.Context(), fork, notch)
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), part, decad.Edges(decad.Reflex(), decad.Circular()).Exactly(1), mm(0.6))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "whose continuation is not selected")
	})
}
