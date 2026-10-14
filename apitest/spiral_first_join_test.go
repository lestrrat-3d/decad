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
	spiralJoinedVolume = 3395.748056677706
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
	require.InEpsilon(t, spiralJoinedVolume, joinedV, 1e-4)
	require.Len(t, joined.Faces(), spiralJoinedFaces)

	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
