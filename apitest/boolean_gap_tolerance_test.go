package apitest_test

import (
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
		a, b := touching()
		_, err := decad.Intersect(t.Context(), a, b)
		require.ErrorIs(t, err, decad.ErrBooleanFailed)
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

	t.Run("a refused retry leaves the operands live", func(t *testing.T) {
		t.Parallel()
		// A tool flush with both walls of a slot has no shift that pokes through
		// both, so the refusal stands.
		doc := decad.New()
		a := boxBody(t, doc, 0, 0, 10, 10, 10)
		slot := translated(t, boxBody(t, doc, 0, 0, 4, 10, 4), 3, 0, 6)
		_, err := decad.Cut(t.Context(), a, slot)
		requireCoplanarFaceRefusal(t, err)
		require.Len(t, doc.Bodies(), 2)
	})
}

// TestBooleanGapToleranceStaysExactForCurvedContact pins what the tolerance does
// not reach: a tangent curved contact is decided by where chords fall, and no
// shift of the second operand changes that, so the refusal stands.
func TestBooleanGapToleranceStaysExactForCurvedContact(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	cyl := diskBody(t, doc, 30, 10, 10)
	_, err := decad.Union(t.Context(), plate, cyl)
	require.ErrorIs(t, err, decad.ErrUnsupported)
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
