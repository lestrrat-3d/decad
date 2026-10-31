package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// requireVolumeWithin asserts a result's volume reads Approximate and that the
// bound it states covers the distance to the true volume.
func requireVolumeWithin(t *testing.T, b *decad.Body, truth float64) decad.Measurement {
	t.Helper()
	v, err := b.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, v.Exactness)
	require.LessOrEqual(t, math.Abs(volumeMM(t, v)-truth), v.Bound.Base(),
		"volume %v is farther from %v than its bound %v", volumeMM(t, v), truth, v.Bound)
	return v
}

// gapBoxes returns a 10×10×10 box and a second box placed at (dx, dy, dz) of the
// given size, in a document made with opts.
func gapBoxes(t *testing.T, dx, dy, dz, sx, sy, sz float64, opts ...decad.DocumentOption) (*decad.Body, *decad.Body) {
	t.Helper()
	doc := decad.New(opts...)
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := translated(t, boxBody(t, doc, 0, 0, sx, sy, sz), dx, dy, dz)
	return a, b
}

// TestBooleanGapTolerance covers the mesh-path contacts a gap tolerance opens:
// operands whose faces lie in one plane. Each case is decided the way a tolerant
// kernel decides it, and the volume bound covers the true volume.
func TestBooleanGapTolerance(t *testing.T) {
	t.Parallel()

	touching := func(opts ...decad.DocumentOption) (*decad.Body, *decad.Body) {
		// B sits on A's top face, shifted 0.5 mm in x so the analytic prism path
		// does not take the pair.
		return gapBoxes(t, 0.5, 0, 10, 10, 10, 10, opts...)
	}
	flush := func(opts ...decad.DocumentOption) (*decad.Body, *decad.Body) {
		// B is a 6×6×5 block inside A whose top face is flush with A's.
		return gapBoxes(t, 2, 2, 5, 6, 6, 5, opts...)
	}

	t.Run("a Union of boxes touching face on face is one body", func(t *testing.T) {
		t.Parallel()
		a, b := touching()
		joined, err := decad.Union(t.Context(), a, b)
		require.NoError(t, err)
		require.Len(t, joined.Lumps(), 1)
		requireVolumeWithin(t, joined, 2000)
		diagnostics := joined.OperationDiagnostics()
		require.Len(t, diagnostics, 1)
		diagnostic := diagnostics[0]
		require.Equal(t, decad.OpDiagGapShiftApplied, diagnostic.Code)
		require.Equal(t, decad.OperationDiagnosticWarning, diagnostic.Severity)
		require.Equal(t, decad.OperationUnion, diagnostic.Operation)
		require.Equal(t, "others", diagnostic.Subject.Name)
		require.NotNil(t, diagnostic.Subject.Ordinal)
		require.Zero(t, *diagnostic.Subject.Ordinal)
		require.NotNil(t, diagnostic.Change)
		require.Positive(t, diagnostic.Change.DisplacementBound.Base())
		require.LessOrEqual(t, diagnostic.Change.DisplacementBound.Base(), diagnostic.Change.AllowedReach.Base())
		diagnostic.Change.DisplacementBound = units.Millimeters(99)
		*diagnostic.Subject.Ordinal = 99
		again := joined.OperationDiagnostics()
		require.NotEqual(t, 99.0, again[0].Change.DisplacementBound.Base())
		require.Zero(t, *again[0].Subject.Ordinal)
		duplicate, err := decad.Copy(t.Context(), joined)
		require.NoError(t, err)
		require.Empty(t, duplicate.OperationDiagnostics(), "a copy reports only adjustments it applies itself")
	})

	t.Run("a Cut of a box touching the target leaves the target", func(t *testing.T) {
		t.Parallel()
		a, b := touching()
		cut, err := decad.Cut(t.Context(), a, b)
		require.NoError(t, err)
		requireVolumeWithin(t, cut, 1000)
	})

	t.Run("an Intersect of boxes that only touch is empty", func(t *testing.T) {
		t.Parallel()
		// Offsetting both footprint axes gives the mesh retry a face contact
		// it can open outward without leaving another pair of side faces flush.
		a, b := gapBoxes(t, 0.5, 2, 10, 10, 10, 10)
		doc := a.Document()
		before := doc.OperationHistory().Operations()
		common, err := decad.Intersect(t.Context(), a, b)
		require.Nil(t, common)
		wrapped := fmt.Errorf("checking the common volume: %w", err)
		require.ErrorIs(t, wrapped, decad.ErrBooleanFailed)
		var boolean *decad.BooleanError
		require.ErrorAs(t, wrapped, &boolean)
		require.Equal(t, decad.BooleanEmpty, boolean.Code)
		var diagnosticError decad.OperationDiagnosticError
		require.ErrorAs(t, wrapped, &diagnosticError)
		diagnostics := diagnosticError.OperationDiagnostics()
		require.Len(t, diagnostics, 2)
		require.Equal(t, decad.OpDiagResultEmpty, diagnostics[0].Code)
		require.Equal(t, decad.OperationDiagnosticFailure, diagnostics[0].Severity)
		require.Equal(t, decad.OperationIntersect, diagnostics[0].Operation)
		require.Equal(t, decad.OperationStageResult, diagnostics[0].Stage)
		shift := diagnostics[1]
		require.Equal(t, decad.OpDiagGapShiftApplied, shift.Code)
		require.Equal(t, decad.OperationDiagnosticWarning, shift.Severity)
		require.Equal(t, decad.OperationIntersect, shift.Operation)
		require.Equal(t, decad.OperationStageResult, shift.Stage)
		require.Equal(t, "b", shift.Subject.Name)
		require.Nil(t, shift.Subject.Ordinal)
		require.NotNil(t, shift.Change)
		require.Equal(t, decad.OperationChangeGapShift, shift.Change.Mechanism)
		require.Positive(t, shift.Change.DisplacementBound.Base())
		require.LessOrEqual(t, shift.Change.DisplacementBound.Base(), shift.Change.AllowedReach.Base())
		require.Equal(t, 0.01, shift.Change.AllowedReach.Base())
		shift.Change.DisplacementBound = units.Millimeters(99)
		require.NotEqual(t, 99.0, diagnosticError.OperationDiagnostics()[1].Change.DisplacementBound.Base())

		live := doc.Bodies()
		require.Len(t, live, 2)
		require.Same(t, a, live[0])
		require.Same(t, b, live[1])
		after := doc.OperationHistory().Operations()
		require.Len(t, after, len(before))
		for i, operation := range before {
			require.Same(t, operation, after[i])
		}
		for _, body := range []*decad.Body{a, b} {
			volume, err := body.Volume()
			require.NoError(t, err)
			require.Equal(t, 1000.0, volume.Value.Base())
			require.Empty(t, body.OperationDiagnostics())
		}
		next, err := decad.Copy(t.Context(), a)
		require.NoError(t, err)
		controlA, _ := gapBoxes(t, 0.5, 2, 10, 10, 10, 10)
		controlNext, err := decad.Copy(t.Context(), controlA)
		require.NoError(t, err)
		require.Equal(t, controlNext.Origin(), next.Origin())
	})

	t.Run("a Cut with a tool flush with the wall opens the pocket", func(t *testing.T) {
		t.Parallel()
		a, b := flush()
		cut, err := decad.Cut(t.Context(), a, b)
		require.NoError(t, err)
		require.Len(t, cut.Lumps(), 1)
		requireVolumeWithin(t, cut, 1000-180)
	})

	t.Run("an Intersect with a block flush with the wall is the block", func(t *testing.T) {
		t.Parallel()
		a, b := flush()
		common, err := decad.Intersect(t.Context(), a, b)
		require.NoError(t, err)
		requireVolumeWithin(t, common, 180)
	})

	t.Run("a Union with a block flush with the wall is the box", func(t *testing.T) {
		t.Parallel()
		a, b := flush()
		joined, err := decad.Union(t.Context(), a, b)
		require.NoError(t, err)
		requireVolumeWithin(t, joined, 1000)
	})

	t.Run("a smaller tolerance gives a tighter bound", func(t *testing.T) {
		t.Parallel()
		a, b := touching()
		wide, err := decad.Union(t.Context(), a, b)
		require.NoError(t, err)
		wideVol := requireVolumeWithin(t, wide, 2000)

		a, b = touching(decad.WithGapTolerance(units.Millimeters(0.0005)))
		narrow, err := decad.Union(t.Context(), a, b)
		require.NoError(t, err)
		narrowVol := requireVolumeWithin(t, narrow, 2000)
		require.Less(t, narrowVol.Bound.Base(), wideVol.Bound.Base())
	})

	t.Run("ExactOnly refuses the coplanar contact", func(t *testing.T) {
		t.Parallel()
		a, b := touching(decad.ExactOnly())
		_, err := decad.Union(t.Context(), a, b)
		requireCoplanarFaceRefusal(t, err)

		a, b = touching(decad.WithGapTolerance(units.Millimeters(0)))
		_, err = decad.Cut(t.Context(), a, b)
		requireCoplanarFaceRefusal(t, err)
	})

	t.Run("a tool flush with both walls of a slot is grown", func(t *testing.T) {
		t.Parallel()
		// No translation pokes the tool through both walls, so the retry grows it about its centre.
		doc := decad.New()
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		slot := translated(t, boxBody(t, doc, 0, 0, 4, 10, 4), 3, 0, 6)
		out, err := decad.Cut(t.Context(), a, slot)
		require.NoError(t, err)
		volume, err := out.Volume()
		require.NoError(t, err)
		require.InDelta(t, 1000-160, volume.Value.Base(), volume.Bound.Base()+1e-9)
		require.Less(t, volume.Bound.Base(), 1.0)
	})

	t.Run("ExactOnly leaves a tool flush with both walls refused", func(t *testing.T) {
		t.Parallel()
		doc := decad.New(decad.ExactOnly())
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		slot := translated(t, boxBody(t, doc, 0, 0, 4, 10, 4), 3, 0, 6)
		_, err := decad.Cut(t.Context(), a, slot)
		requireCoplanarFaceRefusal(t, err)
		require.Len(t, doc.Bodies(), 2, "a refusal leaves the operands live")
	})
}

// TestBooleanGapToleranceStaysExactForCurvedContact pins what the tolerance does
// not reach: a tangent curved contact is decided by where chords fall, and no
// shift of the second operand changes that, so the refusal stands.
func TestBooleanGapToleranceStaysExactForCurvedContact(t *testing.T) {
	t.Parallel()
	doc := decad.New(decad.ExactOnly())
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	cyl := diskBody(t, doc, 30, 10, 10)
	_, err := decad.Union(t.Context(), plate, cyl)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// TestBooleanGapToleranceRestingBossJoinsPlate unions a plate with a round boss
// that a Transform placed on the plate's top face. The placed boss carries a
// bound, so its facets meet the plate's top within the chord tolerance and the
// retry moves it. The boss must sink into the plate, so the result is one solid
// whose volume is the plate's plus the boss's: 100·60·10 + π·6²·8 = 60000 + 288π.
func TestBooleanGapToleranceRestingBossJoinsPlate(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 100, 60, 10)
	boss := translated(t, circlePrismAtZ(t, doc, 20, 30, 6, 10, 8), 30, 0, 0)
	joined, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	require.True(t, joined.IsSolid())
	require.Len(t, joined.Lumps(), 1, "the boss rests on the plate, so the union is one piece")
	require.Len(t, joined.Shells(), 1)
	requireVolumeWithin(t, joined, 60000+288*math.Pi)
	box, err := joined.Bounds()
	require.NoError(t, err)
	require.Less(t, box.Max.Z, 18.0, "the boss sank into the plate, so its top lies below z = 18")
	require.GreaterOrEqual(t, box.Max.Z+box.Bound.Base(), 18.0, "the box's bound covers the unmoved top")

	diagnostics := joined.OperationDiagnostics()
	require.Len(t, diagnostics, 1)
	require.Equal(t, decad.OpDiagGapShiftApplied, diagnostics[0].Code)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

// TestBooleanGapToleranceEdgeAndPointTouches covers bodies that meet only along
// an edge or at a point. Each is decided the way a tolerant kernel decides it,
// the volume bound covers the true volume, and ExactOnly keeps the refusal.
func TestBooleanGapToleranceEdgeAndPointTouches(t *testing.T) {
	t.Parallel()

	type pair func(doc *decad.Document) (*decad.Body, *decad.Body)
	cases := []struct {
		name string
		make pair
		// union, cut and intersect are the true volumes, and empty marks an
		// Intersect with nothing in common.
		union, cut, intersect float64
		empty                 bool
	}{
		{
			// Two cubes share one corner vertex.
			name: "a shared corner",
			make: func(doc *decad.Document) (*decad.Body, *decad.Body) {
				return boxBody(t, doc, 0, 0, 10, 10, 10), translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 10, 10, 10)
			},
			union: 2000, cut: 1000, empty: true,
		},
		{
			// An inner wedge shares the outer wedge's knife-edge line and lies
			// inside it.
			name: "a knife edge inside a wider wedge",
			make: func(doc *decad.Document) (*decad.Body, *decad.Body) {
				w := sketch.NewWorld()
				a := wedgePrism(t, doc, w, w.XY(), [3][2]float64{{0, 0}, {12, -4}, {12, 6}}, 5)
				b := translated(t, wedgePrism(t, doc, w, w.XY(), [3][2]float64{{0, 0}, {8, -1}, {8, 2}}, 3), 0, 0, 1)
				return a, b
			},
			union: 600, cut: 528, intersect: 72,
		},
		{
			// Two knife edges cross at right angles and meet at one point.
			name: "two edges crossing at a point",
			make: func(doc *decad.Document) (*decad.Body, *decad.Body) {
				w := sketch.NewWorld()
				a := wedgePrism(t, doc, w, w.XY(), [3][2]float64{{0, 0}, {12, -4}, {12, 6}}, 5)
				b := wedgePrism(t, doc, w, w.XZ(), [3][2]float64{{0, 0}, {-12, -6}, {-12, 5}}, 5)
				return a, b
			},
			union: 1260, cut: 600, empty: true,
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			a, b := tc.make(doc)
			joined, err := decad.Union(t.Context(), a, b)
			require.NoError(t, err)
			require.Len(t, joined.Lumps(), 1)
			requireVolumeWithin(t, joined, tc.union)

			doc = decad.New()
			a, b = tc.make(doc)
			cut, err := decad.Cut(t.Context(), a, b)
			require.NoError(t, err)
			requireVolumeWithin(t, cut, tc.cut)

			doc = decad.New()
			a, b = tc.make(doc)
			common, err := decad.Intersect(t.Context(), a, b)
			if tc.empty {
				require.ErrorIs(t, err, decad.ErrBooleanFailed)
				return
			}
			require.NoError(t, err)
			requireVolumeWithin(t, common, tc.intersect)
		})

		t.Run(tc.name+" under ExactOnly", func(t *testing.T) {
			t.Parallel()
			doc := decad.New(decad.ExactOnly())
			a, b := tc.make(doc)
			_, err := decad.Union(t.Context(), a, b)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			var be *decad.BooleanError
			require.ErrorAs(t, err, &be)
			require.Equal(t, decad.BooleanUnsupportedContact, be.Code)
			require.Len(t, doc.Bodies(), 2)
		})
	}
}
