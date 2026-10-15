package decad

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestUnstitchNoPayloadIsDegenerate is Table R row R12's other half: a live
// body this evaluator did not build (no topo.Payload at all). decad's
// public seam admits no way to construct one — every real body a caller can
// reach already carries the payload of whatever built it — so this fixture
// is hand-built directly, the same treatment stitch_internal_test.go gives
// its own seam-unreachable fixtures.
func TestUnstitchNoPayloadIsDegenerate(t *testing.T) {
	t.Parallel()
	d := bornDocument(&Document{})
	b := bornBody(&Body{n: topo.Body{Kind: topo.BodySheet, Doc: documentNode(d)}})
	d.n.LiveSet = append(d.n.LiveSet, bodyNode(b))

	_, err := Unstitch(t.Context(), b)
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
	require.Positive(t, band.face.n.NormalBound,
		"a far-placed flat patch has a real departure from the plane it publishes")

	body := bodyHandle(band.face.n.Body)
	wantFaces := body.Faces()
	idx = slices.Index(wantFaces, band.face)
	require.GreaterOrEqual(t, idx, 0, "the band face must be one of its own body's faces")

	results, err := Unstitch(t.Context(), body)
	require.NoError(t, err)
	return band.face, results, idx
}

// TestUnstitchChamferBandNormalAtBoundCoversSource is
// docs/surface-design.md's T80: an unstitched chamfer band patch's own
// NormalAt bound must still cover the source face's own normalBound — the
// public-facing half of stitchrecord's copyFaceUnder carrying that field forward
// rather than dropping it to zero.
func TestUnstitchChamferBandNormalAtBoundCoversSource(t *testing.T) {
	t.Parallel()
	src, results, idx := unstitchOneChamferBand(t)
	newFace := results[idx].Faces()[0]

	pt := newFace.n.Loops[0].CoEdges[0].Edge.Start.Position
	reading, err := newFace.NormalAt(pt)
	require.NoError(t, err)
	bound, err := reading.Bound.In(units.One)
	require.NoError(t, err)

	require.GreaterOrEqual(t, bound, src.n.NormalBound,
		"the unstitched face's own published bound must still cover the source's normalBound")
}

// TestUnstitchPlacementOfNonzeroNormalBoundFaceRefuses is
// docs/surface-design.md's T82: normalBound is dimensionless and a
// placement's own rounding is a length, so composing them is unsound;
// stitchrecord's copyFaceUnder refuses a placed copy of a face whose normalBound is
// nonzero rather than invent the missing term.
func TestUnstitchPlacementOfNonzeroNormalBoundFaceRefuses(t *testing.T) {
	t.Parallel()
	_, results, idx := unstitchOneChamferBand(t)
	sheet := results[idx]
	require.Positive(t, sheet.Faces()[0].n.NormalBound)

	further, err := r3.Translation(r3.NewVec(1, 1, 1))
	require.NoError(t, err)
	_, err = Transform(t.Context(), sheet, further)
	require.ErrorIs(t, err, ErrUnsupported)
}
