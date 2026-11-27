package apitest_test

// CI: isolated-tests

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftCutContinuedByMatchingExtrusion(t *testing.T) {
	check := func(t *testing.T, options ...decad.DocumentOption) {
		t.Helper()
		doc, body := loftPocketBody(t, options...)
		taper := loftPocketTaper(t, doc, 0.5)
		body, err := decad.Cut(body, taper).Do(t.Context())
		require.NoError(t, err)
		s, p := loftPocketProfile(t, -0.75, 0.5)
		continuation, err := decad.Extrude(doc, s, p, decad.Distance{D: units.Millimeters(15), Dir: decad.Along}).Do(t.Context())
		require.NoError(t, err)
		historyCount := len(doc.OperationHistory().Operations())
		bodyCount := len(doc.Bodies())
		previous := body
		body, err = decad.Cut(body, continuation).Do(t.Context())
		require.NoError(t, err)
		requireNoShift(t, body)
		operations := doc.OperationHistory().Operations()
		require.Len(t, operations, historyCount+1)
		operation := operations[len(operations)-1]
		require.Equal(t, decad.OperationCut, operation.Kind())
		inputs := operation.Inputs()
		require.Len(t, inputs, 2)
		require.Same(t, previous, inputs[0].Body)
		require.Same(t, continuation, inputs[1].Body)
		require.Equal(t, "target", inputs[0].Role)
		require.Equal(t, "tool", inputs[1].Role)
		require.Equal(t, decad.InputConsumed, inputs[0].Use)
		require.Equal(t, decad.InputConsumed, inputs[1].Use)
		require.Len(t, doc.Bodies(), bodyCount-1)
		plane := outlinePlane(t, r3.Vec{Z: 7}, r3.Vec{Z: 1})
		outline, err := body.SectionOutline(t.Context(), plane, units.Millimeters(0.1))
		require.NoError(t, err)
		// The point is inside the straight tool, just after the shared section.
		point := r3.Vec{X: -0.7488, Y: -2.5*math.Sqrt(3)/2 - 0.01, Z: 7}
		require.False(t, sectionContainsPoint(outline, point), "the held section retains material inside the tool at %v", point)
	}
	for _, tc := range []struct {
		name string
		opts []decad.DocumentOption
	}{{name: "default"}, {name: "exact", opts: []decad.DocumentOption{decad.ExactOnly()}}} {
		t.Run(tc.name, func(t *testing.T) { check(t, tc.opts...) })
	}
}

func TestLoftContinuationMatchingFacesKeepsRefusal(t *testing.T) {
	doc, body := loftPocketBody(t, decad.ExactOnly())
	taper := loftPocketTaper(t, doc, 0.5)
	body, err := decad.Cut(body, taper).Do(t.Context())
	require.NoError(t, err)
	s, p := loftPocketProfile(t, -0.75, 0.5)
	continuation, err := decad.Extrude(doc, s, p, decad.Distance{D: units.Millimeters(15), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	_, err = decad.Cut(body, continuation).MatchingFaces().Do(t.Context())
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestLoftUnionMatchingExtrusionWithoutShift(t *testing.T) {
	doc := decad.New(decad.ExactOnly())
	s0, p0 := loftPocketProfile(t, -1.25, 0)
	s1, p1 := loftPocketProfile(t, -0.75, 0.5)
	loft, err := decad.Loft(doc, s0, p0, s1, p1).Do(t.Context())
	require.NoError(t, err)
	extrusion, err := decad.Extrude(doc, s1, p1, decad.Distance{D: units.Millimeters(15), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	loftVolume, err := loft.Volume()
	require.NoError(t, err)
	extrusionVolume, err := extrusion.Volume()
	require.NoError(t, err)
	joined, err := decad.Union(loft, extrusion).Do(t.Context())
	require.NoError(t, err)
	joinedVolume, err := joined.Volume()
	require.NoError(t, err)
	require.InDelta(t, loftVolume.Value.Base()+extrusionVolume.Value.Base(), joinedVolume.Value.Base(),
		loftVolume.Bound.Base()+extrusionVolume.Bound.Base()+joinedVolume.Bound.Base())
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestMatchingLoftAndExtrusionCutWithoutShift(t *testing.T) {
	doc, body := loftPocketBody(t, decad.ExactOnly())
	s0, p0 := loftPocketProfile(t, -1.25, 0)
	s1, p1 := loftPocketProfile(t, -0.75, 0.5)
	loft, err := decad.Loft(doc, s0, p0, s1, p1).Do(t.Context())
	require.NoError(t, err)
	extrusion, err := decad.Extrude(doc, s1, p1, decad.Distance{D: units.Millimeters(15), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	joined, err := decad.Union(loft, extrusion).Do(t.Context())
	require.NoError(t, err)
	body, err = decad.Cut(body, joined).Do(t.Context())
	require.NoError(t, err)
	requireNoShift(t, body)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	plane := outlinePlane(t, r3.Vec{Z: 7}, r3.Vec{Z: 1})
	outline, err := body.SectionOutline(t.Context(), plane, units.Millimeters(0.1))
	require.NoError(t, err)
	point := r3.Vec{X: -0.7488, Y: -2.5*math.Sqrt(3)/2 - 0.01, Z: 7}
	require.False(t, sectionContainsPoint(outline, point), "the held section retains material inside the tool at %v", point)
}
