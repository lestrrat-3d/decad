package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This notched polygon has two reflex corners. It exercises the loft-only
// cap-band proof that contains their whole fan in a cap-height slab.
func notchedLoft(t *testing.T, withBore bool) *Body {
	t.Helper()
	corners := []planePoint{
		pt(-5, -5), pt(5, -5), pt(5, 5), pt(1, 5),
		pt(1, 3), pt(-1, 3), pt(-1, 5), pt(-5, 5),
	}
	segs := make([]curveSegment, len(corners))
	for i := range corners {
		segs[i] = lineSeg{Start: corners[i], End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1}
	}
	segs[0] = fitSplineSeg{Fit: point2ToRecordSlice([]planePoint{
		pt(-5, -5), pt(-2, -5.1), pt(2, -5.1), pt(5, -5),
	}), TStart: 0, TEnd: 1}
	profile := profileRecord{Outer: loopRecord{Segments: segs}}
	if withBore {
		profile.Holes = []loopRecord{{Segments: []curveSegment{
			circleSeg{Center: pt(0, 0), Radius: units.Millimeters(1), CCW: false, TStart: 1, TEnd: 0},
		}}}
	}
	p0, p1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 5))
	pl := loftPayload{
		Profile0: profile, Profile1: profile,
		RecordArea: loftRecordAreasOrZero(profile, profile),
		Plane0:     p0, Plane1: p1,
		Frame0: mustFrame(t, p0), Frame1: mustFrame(t, p1),
		Xform: r3.Identity(),
	}
	body, err := bodyResult(loftrecord.Build(t.Context(), envOf(documentNode(New()), producerID(1)), pl, proofbound.NewWorkBudget(t.Context()), freeform.NewFreeformWork(), freeform.NewFreeformWork()))
	require.NoError(t, err)
	return body
}

func TestLoftCapBandMotionRequiresReflexVertexIndices(t *testing.T) {
	outer := capblend.LoopMesh{
		CapPts:      []planePoint{pt(1, 1)},
		CapLoV:      []int{2},
		CapHiV:      []int{4},
		ArcCount:    []int{1},
		CapArcStart: []int{0},
	}
	vertices := make([]r3.Vec, 6)
	motion := []float64{1, 1, math.Inf(1), 1, math.Inf(1), 1}
	maxMotion, err := tessellation.LoftCapBandFiniteMotion(
		outer.ArcCount, outer.CapArcStart, outer.CapLoV, outer.CapHiV,
		len(outer.CapPts), vertices, motion)
	require.NoError(t, err)
	require.Equal(t, 1.0, maxMotion)
	require.Equal(t, 0.0, motion[2])
	require.Equal(t, 0.0, motion[4])
	motion = []float64{1, math.Inf(1), 1, 1, math.Inf(1), 1}
	_, err = tessellation.LoftCapBandFiniteMotion(
		outer.ArcCount, outer.CapArcStart, outer.CapLoV, outer.CapHiV,
		len(outer.CapPts), vertices, motion)
	require.ErrorIs(t, err, ErrUnsupported, "an equal count at different vertices cannot prove the cap slab")
}

func TestLoftCapBandReflexVolumeProofAndSharpBore(t *testing.T) {
	body := notchedLoft(t, true)
	source := body.n.Payload.(loftPayload)
	selector := Edges(OuterLoopOf(CapStart(body))).Or(OuterLoopOf(CapEnd(body)))
	edges, err := selector.SelectEdges(body)
	require.NoError(t, err)
	outNode, recognized, err := brepmodify.LoftCapChamfer(t.Context(), envOf(body.n.Doc, body.n.Doc.NextProducer), &body.n,
		source, edgeNodes(edges), 0.1, 0, nil)
	require.NoError(t, err)
	require.True(t, recognized)
	out := bodyHandle(outNode)
	require.True(t, out.IsSolid())
	band := out.n.Payload.(capBlendPayload)
	require.IsType(t, fitSplineSeg{}, band.LoftSource.Profile.Outer.Segments[0])
	for _, segment := range band.Profile.Outer.Segments {
		require.IsType(t, lineSeg{}, segment, "the band offsets the certified held polygon")
	}
	volume, err := out.Volume()
	require.NoError(t, err)
	require.GreaterOrEqual(t, volume.Bound.Base(), source.Proof.VolSymDiff)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Greater(t, mesh.n.VolSymDiff, source.Proof.VolSymDiff)
	require.NoError(t, booloperand.RequireVolumeProof(t.Context(), &out.n, 0))
	for _, cap := range []provenance.Ref{refOf(CapStart(out)), refOf(CapEnd(out))} {
		faces, err := Faces(FaceCreatedBy(featureRefOf(cap))).Exactly(1).SelectFaces(out)
		require.NoError(t, err)
		require.Len(t, faces[0].Loops(), 2)
		require.Len(t, faces[0].Loops()[1].Edges(), 1, "the bore rim is an unchamfered circle")
	}
	shift, err := r3.Translation(r3.NewVec(12, -7, 3))
	require.NoError(t, err)
	documentHandle(out.n.Doc).commit(out)
	placed, err := Copy(t.Context(), out, WithCopyPlacement(shift))
	require.NoError(t, err)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())
}

func TestLoftCapBandNoBoreChamfersBothOuterLoops(t *testing.T) {
	body := notchedLoft(t, false)
	documentHandle(body.n.Doc).commit(body)
	before, err := body.Volume()
	require.NoError(t, err)
	selector := Edges(OuterLoopOf(CapStart(body))).Or(OuterLoopOf(CapEnd(body)))
	out, err := Chamfer(t.Context(), body, selector, units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, out.IsSolid())
	require.Len(t, out.Shells(), 1)
	band := out.n.Payload.(capBlendPayload)
	require.True(t, band.StartLoops[0])
	require.True(t, band.EndLoops[0])
	require.Empty(t, band.Profile.Holes)
	require.IsType(t, fitSplineSeg{}, band.LoftSource.Profile.Outer.Segments[0])
	for _, cap := range [][2]provenance.Ref{{refOf(CapStart(body)), refOf(CapStart(out))}, {refOf(CapEnd(body)), refOf(CapEnd(out))}} {
		original, err := Faces(FaceCreatedBy(featureRefOf(cap[0]))).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		faces, err := Faces(FaceCreatedBy(featureRefOf(cap[1]))).Exactly(1).SelectFaces(out)
		require.NoError(t, err)
		require.Len(t, faces[0].Loops(), 1)
		originalArea, err := original[0].Area()
		require.NoError(t, err)
		chamferedArea, err := faces[0].Area()
		require.NoError(t, err)
		require.Less(t, chamferedArea.Value.Base()+chamferedArea.Bound.Base(),
			originalArea.Value.Base()-originalArea.Bound.Base())
	}
	after, err := out.Volume()
	require.NoError(t, err)
	require.Less(t, after.Value.Base()+after.Bound.Base(), before.Value.Base()-before.Bound.Base())
	require.GreaterOrEqual(t, after.Bound.Base(), body.n.Payload.(loftPayload).Proof.VolSymDiff)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Greater(t, mesh.n.VolSymDiff, body.n.Payload.(loftPayload).Proof.VolSymDiff)
	require.NoError(t, booloperand.RequireVolumeProof(t.Context(), &out.n, 0))
}
