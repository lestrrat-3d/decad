package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This notched polygon has two reflex corners. It exercises the loft-only
// cap-band proof that contains their whole fan in a cap-height slab.
func notchedHoledLoft(t *testing.T) *Body {
	t.Helper()
	corners := []Point2{
		pt(-5, -5), pt(5, -5), pt(5, 5), pt(1, 5),
		pt(1, 3), pt(-1, 3), pt(-1, 5), pt(-5, 5),
	}
	segs := make([]curveSegment, len(corners))
	for i := range corners {
		segs[i] = lineSeg{Start: corners[i], End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1}
	}
	segs[0] = fitSplineSeg{Fit: point2ToRecordSlice([]Point2{
		pt(-5, -5), pt(-2, -5.1), pt(2, -5.1), pt(5, -5),
	}), TStart: 0, TEnd: 1}
	profile := profileRecord{
		Outer: loopRecord{Segments: segs},
		Holes: []loopRecord{{Segments: []curveSegment{
			circleSeg{Center: pt(0, 0), Radius: units.Millimeters(1), CCW: false, TStart: 1, TEnd: 0},
		}}},
	}
	p0, p1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 5))
	pl := loftPayload{
		profile0: profile, profile1: profile,
		recordArea: loftRecordAreasOrZero(profile, profile),
		plane0:     p0, plane1: p1,
		frame0: mustFrame(t, p0), frame1: mustFrame(t, p1),
		xform: r3.Identity(),
	}
	body, err := evalLoft(t.Context(), New(), producerID(1), pl,
		proofbound.NewWorkBudget(t.Context()), freeform.NewFreeformWork(), freeform.NewFreeformWork())
	require.NoError(t, err)
	return body
}

func TestLoftCapBandMotionRequiresReflexVertexIndices(t *testing.T) {
	outer := capBlendLoopMesh{
		capPts:      []Point2{pt(1, 1)},
		capLoV:      []int{2},
		capHiV:      []int{4},
		arcCount:    []int{1},
		capArcStart: []int{0},
	}
	vertices := make([]r3.Vec, 6)
	motion := []float64{1, 1, math.Inf(1), 1, math.Inf(1), 1}
	maxMotion, err := loftCapBandFiniteMotion(outer, vertices, motion)
	require.NoError(t, err)
	require.Equal(t, 1.0, maxMotion)
	require.Equal(t, 0.0, motion[2])
	require.Equal(t, 0.0, motion[4])
	motion = []float64{1, math.Inf(1), 1, 1, math.Inf(1), 1}
	_, err = loftCapBandFiniteMotion(outer, vertices, motion)
	require.ErrorIs(t, err, ErrUnsupported, "an equal count at different vertices cannot prove the cap slab")
}

func TestLoftCapBandReflexVolumeProofAndSharpBore(t *testing.T) {
	body := notchedHoledLoft(t)
	source := body.payload.(loftPayload)
	selector := Edges(OuterLoopOf(CapStart(body))).Or(OuterLoopOf(CapEnd(body)))
	edges, err := selector.SelectEdges(body)
	require.NoError(t, err)
	out, recognized, err := tryLoftCapChamfer(t.Context(), body, source, edges, 0.1, 0, nil)
	require.NoError(t, err)
	require.True(t, recognized)
	require.True(t, out.IsSolid())
	band := out.payload.(capBlendPayload)
	require.IsType(t, fitSplineSeg{}, band.loftSource.profile.Outer.Segments[0])
	for _, segment := range band.profile.Outer.Segments {
		require.IsType(t, lineSeg{}, segment, "the band offsets the certified held polygon")
	}
	volume, err := out.Volume()
	require.NoError(t, err)
	require.GreaterOrEqual(t, volume.Bound.Base(), source.proof.VolSymDiff)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Greater(t, mesh.volSymDiff, source.proof.VolSymDiff)
	require.NoError(t, requireVolumeProvingPayload(t.Context(), out, 0))
	for _, cap := range []FeatureRef{CapStart(out), CapEnd(out)} {
		faces, err := Faces(FaceCreatedBy(cap)).Exactly(1).SelectFaces(out)
		require.NoError(t, err)
		require.Len(t, faces[0].Loops(), 2)
		require.Len(t, faces[0].Loops()[1].Edges(), 1, "the bore rim is an unchamfered circle")
	}
	shift, err := r3.Translation(r3.NewVec(12, -7, 3))
	require.NoError(t, err)
	out.doc.commit(out)
	placed, err := out.PlacedCopy(t.Context(), shift)
	require.NoError(t, err)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())
}
