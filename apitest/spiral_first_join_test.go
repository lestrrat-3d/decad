package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Reference values of the joined solid, in mm^3 and faces. The face count is
// the blank's six revolved faces plus the tooth faces that survive the join.
const (
	spiralJoinedVolume = 3395.7478
	spiralJoinedFaces  = 34
)

// TestSpiralFirstToothUnionIsCertified drives Union(blank, cone-trimmed
// LoftFaces spiral tooth) through the spiral first-join certificate and the
// held-boundary validation. Either refusing makes Union fail with
// ErrUnsupported, so a joined solid proves both accepted the pair.
func TestSpiralFirstToothUnionIsCertified(t *testing.T) {
	doc := decad.New()
	blank, tooth := spiralBlankAndTooth(t, doc)
	require.Len(t, doc.Bodies(), 2)

	blankVolume, err := blank.Volume()
	require.NoError(t, err)
	toothVolume, err := tooth.Volume()
	require.NoError(t, err)
	blankV, toothV := blankVolume.Value.Base(), toothVolume.Value.Base()
	require.Positive(t, blankV)
	require.Positive(t, toothV)

	joined, err := decad.Union(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.True(t, joined.IsSolid())

	// The tooth root sits mostly inside the blank, so the union holds the whole
	// blank plus the tooth part outside it: a gain smaller than the tooth.
	joinedVolume, err := joined.Volume()
	require.NoError(t, err)
	joinedV := joinedVolume.Value.Base()
	slack := joinedVolume.Bound.Base() + blankVolume.Bound.Base() + toothVolume.Bound.Base()
	require.Greater(t, joinedV, blankV, "the tooth adds material outside the blank")
	require.Less(t, joinedV-blankV, toothV+slack, "the gain cannot exceed the tooth")
	// The mesh volume agrees across FMA and non-FMA builds to 1e-9 relative.
	// The tolerance of 5e-4 (about 1.7 mm^3) is a tenth of the 17.5 mm^3 tooth,
	// so a tooth lost from the union still fails, and it holds no library Bound.
	require.InEpsilon(t, spiralJoinedVolume, joinedV, 5e-4)
	require.Len(t, joined.Faces(), spiralJoinedFaces)

	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// TestSpiralTangentFirstToothUnionIsCertified repeats the legacy assertions on a
// tooth whose heel cap is exactly tangent to the blank's heel cone, built from
// ConeFromFace and LoftFacesToConeTangent. No hand-written record is involved:
// the real producers feed the real consumers.
func TestSpiralTangentFirstToothUnionIsCertified(t *testing.T) {
	doc := decad.New()
	blank, tooth, curved := spiralTangentBlankAndTooth(t, doc)

	// The planar end cap of the loft did not survive the heel Intersect.
	for _, face := range tooth.Faces() {
		require.NotContains(t, face.Origins(), decad.CapEnd(curved),
			"the loft's planar end cap is trimmed away")
	}

	blankVolume, err := blank.Volume()
	require.NoError(t, err)
	toothVolume, err := tooth.Volume()
	require.NoError(t, err)
	blankV, toothV := blankVolume.Value.Base(), toothVolume.Value.Base()
	require.Positive(t, blankV)
	require.Positive(t, toothV)

	joined, err := decad.Union(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, joined.IsSolid())

	joinedVolume, err := joined.Volume()
	require.NoError(t, err)
	joinedV := joinedVolume.Value.Base()
	slack := joinedVolume.Bound.Base() + blankVolume.Bound.Base() + toothVolume.Bound.Base()
	require.Greater(t, joinedV, blankV, "the tooth adds material outside the blank")
	require.Less(t, joinedV-blankV, toothV+slack, "the gain cannot exceed the tooth")
	require.InEpsilon(t, spiralJoinedVolume, joinedV, 5e-4)
	require.Len(t, joined.Faces(), spiralJoinedFaces)

	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
