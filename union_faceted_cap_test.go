package decad

// CI: isolated-tests

import (
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestUnionFlushCapCutPrismAndLoft(t *testing.T) {
	t.Parallel()
	for _, exact := range []bool{false, true} {
		for _, bossFirst := range []bool{false, true} {
			doc := New()
			if exact {
				doc = New(ExactOnly())
			}
			lower := facetedCapCutPrism(t, doc)
			boss := facetedCapBoss(t, doc)
			var joined *Body
			var err error
			if bossFirst {
				joined, err = Union(boss, lower).Do(t.Context())
			} else {
				joined, err = Union(lower, boss).Do(t.Context())
			}
			require.NoError(t, err, "exact only %v, boss first %v", exact, bossFirst)
			requireStackedUnion(t, facetedCapCutPrism(t, doc), facetedCapBoss(t, doc), joined, 1)
		}
	}
}

func TestUnionFlushCapFarPostAcrossPlane(t *testing.T) {
	t.Parallel()
	doc := New(ExactOnly())
	lower, err := Union(facetedCapCutPrism(t, doc), facetedCapPost(t, doc, 5, 0)).Do(t.Context())
	require.NoError(t, err)
	joined, err := Union(lower, facetedCapBoss(t, doc)).Do(t.Context())
	require.NoError(t, err)
	lower, err = Union(facetedCapCutPrism(t, doc), facetedCapPost(t, doc, 5, 0)).Do(t.Context())
	require.NoError(t, err)
	requireStackedUnion(t, lower, facetedCapBoss(t, doc), joined, 2)
}

func TestUnionFlushCapNearPostRefused(t *testing.T) {
	t.Parallel()
	doc := New(ExactOnly())
	// The post stands inside the boss's box, clear of the boss, and crosses the cap plane: the lower body does not
	// lie below the plane near the boss.
	lower, err := Union(facetedCapCutPrism(t, doc), facetedCapPost(t, doc, 0.7, -3.3)).Do(t.Context())
	require.NoError(t, err)
	_, err = Union(lower, facetedCapBoss(t, doc)).Do(t.Context())
	var be *BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, BooleanUnsupportedContact, be.Code)
}

func TestUnionFlushCapTangentBossRefused(t *testing.T) {
	t.Parallel()
	doc := New(ExactOnly())
	// The boss's floor touches the lower body's top at one point of the axis circle: no disc of the plane lies inside
	// both faces.
	w := sketch.NewWorld()
	s, p := sharedCapCircle(t, w, 0, [2]float64{facetedCapJoinR + 0.5, 0}, 0.5)
	boss, err := Extrude(doc, s, p, Distance{D: units.Millimeters(1), Dir: Along}).Do(t.Context())
	require.NoError(t, err)
	_, err = Union(facetedCapCutPrism(t, doc), boss).Do(t.Context())
	var be *BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, BooleanUnsupportedContact, be.Code)
}

func TestUnionFlushCapCarriedByDrive(t *testing.T) {
	t.Parallel()
	// The drive's walls are held to its own Union's chord tolerance, which this small pair's is finer than.
	doc := New(WithChainTolerance(units.Millimeters(0.01)))
	joined, err := Union(facetedCapPrism(t, doc, -2.4, 0), facetedCapDrive(t, doc)).Do(t.Context())
	require.NoError(t, err)
	requireStackedUnion(t, facetedCapPrism(t, doc, -2.4, 0), facetedCapDrive(t, doc), joined, 1)
}

func TestUnionFlushCapCutPrismAndDrive(t *testing.T) {
	t.Parallel()
	doc := New(WithChainTolerance(units.Millimeters(0.01)))
	joined, err := Union(facetedCapCutPrism(t, doc), facetedCapDrive(t, doc)).Do(t.Context())
	require.NoError(t, err)
	requireStackedUnion(t, facetedCapCutPrism(t, doc), facetedCapDrive(t, doc), joined, 1)
}
