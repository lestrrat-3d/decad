package decad

// CI: isolated-tests

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestUnionSharedCapObliqueLoftAndCone(t *testing.T) {
	t.Parallel()
	for _, exact := range []bool{false, true} {
		doc := New()
		if exact {
			doc = New(ExactOnly())
		}
		lower := sharedCapFrustum(t, doc, sharedCapObliqueBase(), 2.0, 0.9922)
		cone := sharedCapCone(t, doc, [2]float64{}, 0.9922, 0.409, 4.15)
		joined, err := Union(lower, cone).Do(t.Context())
		require.NoError(t, err, "exact only: %v", exact)
		requireDisjointUnion(t, lower, cone, joined)
		if !exact {
			continue
		}
		// The Union retired both operands; the Intersect and the Cut read fresh copies.
		lower = sharedCapFrustum(t, doc, sharedCapObliqueBase(), 2.0, 0.9922)
		cone = sharedCapCone(t, doc, [2]float64{}, 0.9922, 0.409, 4.15)
		lv, err := lower.Volume()
		require.NoError(t, err)
		_, err = Intersect(lower, cone).Do(t.Context())
		var be *BooleanError
		require.ErrorAs(t, err, &be)
		require.Equal(t, BooleanEmpty, be.Code)
		lower = sharedCapFrustum(t, doc, sharedCapObliqueBase(), 2.0, 0.9922)
		cone = sharedCapCone(t, doc, [2]float64{}, 0.9922, 0.409, 4.15)
		cut, err := Cut(lower, cone).Do(t.Context())
		require.NoError(t, err)
		cv, err := cut.Volume()
		require.NoError(t, err)
		require.InDelta(t, lv.Value.Base(), cv.Value.Base(), lv.Bound.Base()+cv.Bound.Base())
	}
}

func TestUnionSharedCapCylinderAndCone(t *testing.T) {
	t.Parallel()
	doc := New(ExactOnly())
	lower := sharedCapFrustum(t, doc, [2]float64{}, 1.0, 1.0)
	cone := sharedCapCone(t, doc, [2]float64{}, 1.0, 0.4, 3.0)
	joined, err := Union(lower, cone).Do(t.Context())
	require.NoError(t, err)
	requireDisjointUnion(t, lower, cone, joined)
}

func TestUnionSharedCapPartialOverlap(t *testing.T) {
	t.Parallel()
	doc := New(ExactOnly())
	lower := sharedCapFrustum(t, doc, [2]float64{}, 1.5, 1.0)
	cone := sharedCapCone(t, doc, [2]float64{1.0, 0}, 1.0, 0.4, 3.0)
	joined, err := Union(lower, cone).Do(t.Context())
	require.NoError(t, err)
	requireDisjointUnion(t, lower, cone, joined)
}

func TestUnionSharedCapTangentCapsRefused(t *testing.T) {
	t.Parallel()
	doc := New(ExactOnly())
	lower := sharedCapFrustum(t, doc, [2]float64{}, 1.5, 1.0)
	cone := sharedCapCone(t, doc, [2]float64{2.0, 0}, 1.0, 0.4, 3.0)
	_, err := Union(lower, cone).Do(t.Context())
	var be *BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, BooleanUnsupportedContact, be.Code)
}

func TestUnionSharedCapBossOnPlate(t *testing.T) {
	t.Parallel()
	for _, exact := range []bool{false, true} {
		doc := New()
		if exact {
			doc = New(ExactOnly())
		}
		plate := internalOffsetBox(t, doc, -5, -4, 5, 4, 0, Distance{D: units.Millimeters(sharedCapHeight), Dir: Along})
		boss := sharedCapBoss(t, doc, [2]float64{1.5, -0.5}, 1.2, 2.0)
		joined, err := Union(plate, boss).Do(t.Context())
		require.NoError(t, err, "exact only: %v", exact)
		requireDisjointUnion(t, plate, boss, joined)
	}
}

func TestRevolveCapSidesNeedTheCapAloneOnItsPlane(t *testing.T) {
	t.Parallel()
	doc := New()
	cone := sharedCapCone(t, doc, [2]float64{}, 1.0, 0.4, 3.0)
	sides := meshbool.RevolveCapSides(topobuild.Faces(&cone.n))
	require.Len(t, sides, 2, "the cone meets its lowest and its highest level only in its caps")
	// The meridian returns to the lowest level at radius 1, so the revolve meets that plane in a circle beside its cap:
	// only the top keeps a side.
	notched := sharedCapRevolve(t, doc, [2]float64{}, [][2]float64{{0, 0}, {0, 0.5}, {1, 0.75}, {0, 1}, {2, 1}, {2, 0}})
	sides = meshbool.RevolveCapSides(topobuild.Faces(&notched.n))
	require.Len(t, sides, 1)
	require.Equal(t, 1, sides[0].N[2].Sign(), "the remaining side faces up")
}
