package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// restingBossCase is a plate with round bosses of radius 6 and height 8
// standing on its top face at z = 10. The last boss rests with a bound, so the
// Union that adds it takes the shifted retry and moves it by s.
type restingBossCase struct {
	name string
	// make builds the joined body; at is the unmoved centre of the boss the
	// shifted Union moved, at the plate's top.
	make func(t *testing.T, doc *decad.Document) *decad.Body
	at   r3.Vec
	// bosses is how many bosses the body has.
	bosses int
}

func restingBossCases() []restingBossCase {
	return []restingBossCase{
		{
			name: "a Transform'd boss on a plate",
			make: func(t *testing.T, doc *decad.Document) *decad.Body {
				t.Helper()
				plate := boxBody(t, doc, 0, 0, 100, 60, 10)
				boss := translated(t, circlePrismAtZ(t, doc, 20, 30, 6, 10, 8), 30, 0, 0)
				joined, err := decad.Union(t.Context(), plate, boss)
				require.NoError(t, err)
				return joined
			},
			at:     r3.NewVec(50, 30, 10),
			bosses: 1,
		},
		{
			name: "a Transform'd boss on a plate and boss",
			make: func(t *testing.T, doc *decad.Document) *decad.Body {
				t.Helper()
				plate := boxBody(t, doc, 0, 0, 100, 60, 10)
				first, err := decad.Union(t.Context(), plate, circlePrismAtZ(t, doc, 20, 30, 6, 10, 8))
				require.NoError(t, err)
				second := translated(t, circlePrismAtZ(t, doc, 20, 30, 6, 10, 8), 30, 0, 0)
				joined, err := decad.Union(t.Context(), first, second)
				require.NoError(t, err)
				return joined
			},
			at:     r3.NewVec(50, 30, 10),
			bosses: 2,
		},
	}
}

// movedBossShift returns the shift s the retry gave the boss whose unmoved
// wall axis passes through at: the joined body's radius-6 Cylinder wall whose
// origin lies within the gap tolerance of at, less at. It asserts the wall is
// the boss's own cylinder translated by s.
func movedBossShift(t *testing.T, joined *decad.Body, at r3.Vec) r3.Vec {
	t.Helper()
	var found []r3.Vec
	for _, f := range joined.Faces() {
		cyl, ok := f.Surface().(decad.Cylinder)
		if !ok || cyl.Origin.Sub(at).Len() > 0.01 {
			continue
		}
		require.Equal(t, 6.0, cyl.Radius.Base(), "the translated wall keeps the boss's radius")
		require.Equal(t, r3.NewVec(0, 0, 1), cyl.Axis, "a translation keeps the axis")
		found = append(found, cyl.Origin.Sub(at))
	}
	require.Len(t, found, 1, "the moved boss's wall reads as one Cylinder")
	s := found[0]
	require.Positive(t, s.Len(), "the retry moved the boss")
	require.Negative(t, s.Z, "a resting boss sinks into the plate")
	return s
}

// TestBooleanGapShiftKeepsTranslatedTags unions a plate with a boss that rests
// on its top with a bound, so the Union takes the shifted retry. The moved
// boss's faces keep their tags translated by the shift: the wall reads as a
// radius-6 Cylinder about the boss's axis moved by s, its top as a Plane at
// z = 18 + s.z, and the root and rim as Circle3 edges of radius 6 about the
// moved axis.
func TestBooleanGapShiftKeepsTranslatedTags(t *testing.T) {
	t.Parallel()
	for _, tc := range restingBossCases() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			joined := tc.make(t, doc)
			require.Len(t, joined.Lumps(), 1)
			diagnostics := joined.OperationDiagnostics()
			require.NotEmpty(t, diagnostics)
			require.Equal(t, decad.OpDiagGapShiftApplied, diagnostics[len(diagnostics)-1].Code)

			s := movedBossShift(t, joined, tc.at)
			axis := tc.at.Add(r3.NewVec(s.X, s.Y, 0))
			top := 18 + s.Z

			var tops int
			for _, f := range joined.Faces() {
				plane, ok := f.Surface().(decad.Plane)
				if !ok || math.Abs(plane.Frame.Origin().Z-top) > 1e-12 {
					continue
				}
				require.InDelta(t, 1, math.Abs(plane.Frame.N().Z), 1e-15, "the moved top stays level")
				tops++
			}
			require.Equal(t, 1, tops, "the moved boss's top reads as a Plane at 18 + s.z")

			var roots, rims int
			for _, e := range joined.Edges() {
				c, ok := e.Curve().(decad.Circle3)
				if !ok || math.Hypot(c.Center.X-axis.X, c.Center.Y-axis.Y) > 1e-12 {
					continue
				}
				require.Equal(t, 6.0, c.Radius.Base())
				switch {
				case math.Abs(c.Center.Z-10) <= 1e-12:
					roots++
					require.False(t, e.IsConvex(), "the root is a concave edge")
				case math.Abs(c.Center.Z-top) <= 1e-12:
					rims++
					require.True(t, e.IsConvex(), "the rim is a convex edge")
				}
			}
			require.Equal(t, 1, roots, "the moved boss's root is a Circle3 on the plate's top")
			require.Equal(t, 1, rims, "the moved boss's rim is a Circle3 at 18 + s.z")
		})
	}
}

// TestBooleanGapShiftFilletsRestingBoss fillets every boss root and rim of a
// plate whose last boss joined through the shifted retry. The moved boss's
// translated tags admit the retained rim blend, so each edge gets a Torus.
//
// A radius-r fillet of a circular edge of radius R adds or removes the ring
// swept by the corner region of area A = (1 − π/4)·r², whose centroid lies
// d = r·(10 − 3π)/(3·(4 − π)) from the edge towards the corner. The root adds
// 2π·(R + d)·A and the rim removes 2π·(R − d)·A, so each boss adds 4π·A·d to
// 100·60·10 + π·6²·8. The moved boss sinks by |s.z|, so the solid the result
// states is smaller by π·6²·|s.z|; the bound covers both values.
func TestBooleanGapShiftFilletsRestingBoss(t *testing.T) {
	t.Parallel()
	const r = 1.0
	area := (1 - math.Pi/4) * r * r
	d := r * (10 - 3*math.Pi) / (3 * (4 - math.Pi))
	perBoss := 36*math.Pi*8 + 4*math.Pi*area*d
	for _, tc := range restingBossCases() {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			joined := tc.make(t, doc)

			edges := 2 * tc.bosses
			filleted, err := decad.Fillet(t.Context(), joined, decad.Edges(decad.Circular()).Exactly(edges), units.Millimeters(r))
			require.NoError(t, err)
			var tori int
			for _, f := range filleted.Faces() {
				if torus, ok := f.Surface().(decad.Torus); ok {
					require.Equal(t, r, torus.Minor.Base())
					tori++
				}
			}
			require.Equal(t, edges, tori, "each root and rim is blended by a Torus")

			s := movedBossShift(t, joined, tc.at)
			hand := 60000 + float64(tc.bosses)*perBoss
			v := requireVolumeWithin(t, filleted, hand)
			shifted := hand - 36*math.Pi*math.Abs(s.Z)
			require.LessOrEqual(t, math.Abs(volumeMM(t, v)-shifted), v.Bound.Base(),
				"the bound covers the solid with the moved boss, %v", shifted)

			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
		})
	}
}
