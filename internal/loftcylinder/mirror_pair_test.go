package loftcylinder_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestMirrorBlank(t *testing.T) {
	xy, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	blank := func(edit func(*prismrecord.Payload)) prismrecord.Payload {
		pp := prismrecord.Payload{Frame: xy, Z0: 0, Z1: 2, Xform: r3.Identity()}
		pp.Profile.Outer.Segments = []sectionrecord.CurveSegment{
			sectionrecord.CircleSeg{Radius: units.Millimeters(4), CCW: true},
		}
		if edit != nil {
			edit(&pp)
		}
		return pp
	}
	require.True(t, loftcylinder.MirrorBlank(blank(nil)))
	for name, edit := range map[string]func(*prismrecord.Payload){
		"surface result": func(pp *prismrecord.Payload) { pp.SurfaceResult = true },
		"placed": func(pp *prismrecord.Payload) {
			pp.Xform = r3.Transform{}
		},
		"cap delta":     func(pp *prismrecord.Payload) { pp.Z1Delta = 1e-9 },
		"section delta": func(pp *prismrecord.Payload) { pp.SectionDelta = 1e-9 },
		"empty height":  func(pp *prismrecord.Payload) { pp.Z1 = pp.Z0 },
		"off axis": func(pp *prismrecord.Payload) {
			pp.Profile.Outer.Segments[0] = sectionrecord.CircleSeg{
				Center: sectionrecord.Point2{U: 1}, Radius: units.Millimeters(4), CCW: true,
			}
		},
		"clockwise": func(pp *prismrecord.Payload) {
			pp.Profile.Outer.Segments[0] = sectionrecord.CircleSeg{Radius: units.Millimeters(4)}
		},
		"infinite radius": func(pp *prismrecord.Payload) {
			pp.Profile.Outer.Segments[0] = sectionrecord.CircleSeg{Radius: units.Millimeters(math.Inf(1)), CCW: true}
		},
	} {
		require.False(t, loftcylinder.MirrorBlank(blank(edit)), name)
	}
}

func TestMirrorPairMotion(t *testing.T) {
	source := &facetedrecord.LoftMirrorJoinRecord{}
	other := &facetedrecord.LoftMirrorJoinRecord{}
	identity := r3.Identity()
	motion, ok := loftcylinder.MirrorPairMotion(facetedrecord.Payload{
		LoftMirrorJoin: source, Xform: identity,
	}, source)
	require.True(t, ok)
	require.Equal(t, identity, motion)
	_, ok = loftcylinder.MirrorPairMotion(facetedrecord.Payload{
		LoftMirrorJoin: other, Xform: identity,
	}, source)
	require.False(t, ok, "the pair joins another source")

	rotated, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(math.Pi/4))
	require.NoError(t, err)
	motion, ok = loftcylinder.MirrorPairMotion(facetedrecord.Payload{
		LoftMirrorCopy: &facetedrecord.LoftMirrorPlacementRecord{Source: source, Motion: rotated},
		Xform:          rotated,
	}, source)
	require.True(t, ok)
	require.Equal(t, rotated, motion)
	_, ok = loftcylinder.MirrorPairMotion(facetedrecord.Payload{
		LoftMirrorCopy: &facetedrecord.LoftMirrorPlacementRecord{Source: other, Motion: rotated},
		Xform:          rotated,
	}, source)
	require.False(t, ok, "the copy places another source")
	_, ok = loftcylinder.MirrorPairMotion(facetedrecord.Payload{
		LoftMirrorCopy: &facetedrecord.LoftMirrorPlacementRecord{Source: source, Motion: rotated},
		Xform:          identity,
	}, source)
	require.False(t, ok, "the held mesh lost the recorded motion")
	shifted, err := r3.Translation(r3.Vec{Z: 1})
	require.NoError(t, err)
	_, ok = loftcylinder.MirrorPairMotion(facetedrecord.Payload{
		LoftMirrorCopy: &facetedrecord.LoftMirrorPlacementRecord{Source: source, Motion: shifted},
		Xform:          shifted,
	}, source)
	require.False(t, ok, "the motion is not axial")
}
