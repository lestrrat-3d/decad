package massdispatch_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massdispatch"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prooflineage"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessdispatch"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type quantityCutMesher struct{}

func (quantityCutMesher) TessellateOperand(ctx context.Context, body *topo.Body, chord float64) (*topo.Mesh, bool, error) {
	mesh, err := tessdispatch.Body(ctx, body, chord, tessdispatch.All, tessdispatch.Options{Alloc: topotest.Alloc{}})
	if err != nil {
		return nil, false, err
	}
	if err := topobuild.RequireShellWinding(ctx, mesh, body.Lumps); err != nil {
		return nil, false, err
	}
	// Match the public adapter's publication after all verification succeeds.
	mesh.BoundaryOK = true
	return mesh, false, nil
}

func quantityCutNodes(t *testing.T) (*topo.Body, *topo.Body, *topo.Body) {
	t.Helper()
	ctx := t.Context()
	doc := &topo.Document{}
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	points := []sectionrecord.Point2{{}, {U: 20}, {U: 20, V: 20}, {V: 20}}
	segments := make([]sectionrecord.CurveSegment, len(points))
	for i, point := range points {
		segments[i] = sectionrecord.LineSeg{Start: point, End: points[(i+1)%len(points)], TEnd: 1}
	}
	build := func(ref provenance.Producer, p prismrecord.Payload) *topo.Body {
		body, err := prismrecord.Build(ctx, topo.Env{Doc: doc, Ref: ref, Alloc: topotest.Alloc{}}, p,
			freeform.NewFreeformWork())
		require.NoError(t, err)
		return body
	}
	target := build(1, prismrecord.Payload{Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}},
		Frame: frame, Xform: r3.Identity(), Z1: 20})
	rotation, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(20))
	require.NoError(t, err)
	axis := rotation.ApplyDir(r3.Vec{Z: 1})
	translation, err := r3.Translation(r3.Vec{X: 12, Y: 10, Z: 10}.Sub(axis.Scale(2)))
	require.NoError(t, err)
	placement, err := rotation.Then(translation)
	require.NoError(t, err)
	tool := build(2, prismrecord.Payload{Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TEnd: 1},
	}}}, Frame: frame, Xform: placement, Z1: 4})
	evaluation, err := booleaneval.Evaluate(ctx, meshbool.OpCut, target, tool, quantityCutMesher{}, 1, nil, nil)
	require.NoError(t, err)
	volume := measurement.Measurement{Value: units.CubicMillimeters(evaluation.Volume),
		Bound: units.CubicMillimeters(evaluation.VolumeBound), Exactness: measurement.ExactnessOf(evaluation.VolumeBound)}
	result, err := facetedrecord.BuildWithProof(ctx, topo.Env{Doc: doc, Ref: 3, Alloc: topotest.Alloc{}},
		evaluation.Payload, evaluation.Audit, volume, evaluation.VolumeRat)
	require.NoError(t, err)
	return result, target, tool
}

func TestCaptureContainedCutQuantitiesFromRealMesh(t *testing.T) {
	t.Parallel()
	result, target, tool := quantityCutNodes(t)
	ordinaryBound := result.Volume.Bound.Base()
	require.NoError(t, massdispatch.CaptureContainedCutQuantities(t.Context(), result, target, tool))
	lineage := result.VolumeLineage
	require.True(t, lineage != nil && lineage.Centroid != nil && lineage.Moments != nil)
	require.Equal(t, 1, lineage.TargetProducer)
	require.Equal(t, 2, lineage.ToolProducer)
	require.Less(t, result.Volume.Bound.Base(), ordinaryBound)
	require.InDelta(t, 8000-4*math.Pi, result.Volume.Value.Base(), result.Volume.Bound.Base()+1e-9)
	wantX := (80000 - 48*math.Pi) / (8000 - 4*math.Pi)
	require.InDelta(t, wantX, result.Centroid.Value.X, result.Centroid.Bound.Base()+1e-9)
	require.Nil(t, target.VolumeLineage)
	require.Nil(t, tool.VolumeLineage)

	t.Run("inconsistent reading keeps the completed result unchanged", func(t *testing.T) {
		broken, target, tool := quantityCutNodes(t)
		broken.VolumeLineage = nil
		broken.Volume = measurement.Measurement{Value: units.CubicMillimeters(0), Bound: units.CubicMillimeters(0), Exactness: measurement.Exact}
		err := massdispatch.CaptureContainedCutQuantities(t.Context(), broken, target, tool)
		var evidence *massdispatch.ContainedCutEvidenceError
		require.ErrorAs(t, err, &evidence)
		require.True(t, evidence.Unwrap() == prooflineage.ErrDisjointIntervals)
		require.ErrorIs(t, err, prooflineage.ErrDisjointIntervals)
		require.Nil(t, broken.VolumeLineage)
		require.Zero(t, broken.Volume.Value.Base())
	})
	t.Run("cancellation passes through without an evidence wrapper", func(t *testing.T) {
		unchanged, target, tool := quantityCutNodes(t)
		unchanged.VolumeLineage = nil
		originalVolume := unchanged.Volume.Value.Base()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		err := massdispatch.CaptureContainedCutQuantities(ctx, unchanged, target, tool)
		require.ErrorIs(t, err, context.Canceled)
		_, evidence := errors.AsType[*massdispatch.ContainedCutEvidenceError](err)
		require.False(t, evidence)
		require.Nil(t, unchanged.VolumeLineage)
		require.Equal(t, originalVolume, unchanged.Volume.Value.Base())
	})
}
