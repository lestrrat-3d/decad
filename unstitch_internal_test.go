package decad

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestUnstitchNoPayloadIsDegenerate is Table R row R12's other half: a live
// body this evaluator did not build (no featurePayload at all). decad's
// public seam admits no way to construct one — every real body a caller can
// reach already carries the payload of whatever built it — so this fixture
// is hand-built directly, the same treatment stitch_internal_test.go gives
// its own seam-unreachable fixtures.
func TestUnstitchNoPayloadIsDegenerate(t *testing.T) {
	t.Parallel()
	d := &Document{}
	b := &Body{doc: d, kind: BodySheet}
	d.bodies = append(d.bodies, b)

	_, err := b.Unstitch(t.Context())
	require.ErrorIs(t, err, ErrDegenerate)
	require.Len(t, d.Bodies(), 1)
	require.Same(t, b, d.Bodies()[0])
}

// unstitchOneChamferBand builds capblend_internal_test.go's own far-placed
// chamfered rectangle (a nonzero-normalBound flat band patch) and returns
// that source face beside the SAME face's own copy Unstitch mints for it,
// both still live: results is the whole Unstitch return, and idx is
// results' own index for the band, so results[idx].Faces()[0] is the copy.
func unstitchOneChamferBand(t *testing.T) (src *Face, results []*Body, idx int) {
	t.Helper()
	motion := placedFarMotion(t)
	band := chamferedRectFlatPatches(t, motion)[0]
	require.Positive(t, band.face.normalBound,
		"a far-placed flat patch has a real departure from the plane it publishes")

	body := band.face.body
	wantFaces := body.Faces()
	idx = slices.Index(wantFaces, band.face)
	require.GreaterOrEqual(t, idx, 0, "the band face must be one of its own body's faces")

	results, err := body.Unstitch(t.Context())
	require.NoError(t, err)
	return band.face, results, idx
}

// TestUnstitchChamferBandNormalAtBoundCoversSource is
// docs/surface-design.md's T80: an unstitched chamfer band patch's own
// NormalAt bound must still cover the source face's own normalBound — the
// public-facing half of copyFaceUnderContext carrying that field forward
// rather than dropping it to zero.
func TestUnstitchChamferBandNormalAtBoundCoversSource(t *testing.T) {
	t.Parallel()
	src, results, idx := unstitchOneChamferBand(t)
	newFace := results[idx].Faces()[0]

	pt := newFace.loops[0].coedges[0].edge.start.position
	reading, err := newFace.NormalAt(pt)
	require.NoError(t, err)
	bound, err := reading.Bound.In(units.One)
	require.NoError(t, err)

	require.GreaterOrEqual(t, bound, src.normalBound,
		"the unstitched face's own published bound must still cover the source's normalBound")
}

// TestUnstitchPlacedCopyOfNonzeroNormalBoundFaceRefuses is
// docs/surface-design.md's T82: normalBound is dimensionless and a
// placement's own rounding is a length, so composing them is unsound;
// copyFaceUnderContext refuses a placed copy of a face whose normalBound is
// nonzero rather than invent the missing term.
func TestUnstitchPlacedCopyOfNonzeroNormalBoundFaceRefuses(t *testing.T) {
	t.Parallel()
	_, results, idx := unstitchOneChamferBand(t)
	sheet := results[idx]
	require.Positive(t, sheet.Faces()[0].normalBound)

	further, err := r3.Translation(r3.NewVec(1, 1, 1))
	require.NoError(t, err)
	_, err = sheet.Placed(t.Context(), further)
	require.ErrorIs(t, err, ErrUnsupported)
}
