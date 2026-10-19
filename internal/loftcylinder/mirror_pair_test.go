package loftcylinder_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
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

// TestMirrorChamferUpperShifts pins each source tooth's upper shift to its own
// authenticated placement. The pairs and the sorted source teeth come in
// different orders, so a missing, reordered or reversed shift list fails.
func TestMirrorChamferUpperShifts(t *testing.T) {
	middle, err := r3.NewFrame(r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	identity := r3.Identity()
	near, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(10*2*math.Pi/17))
	require.NoError(t, err)
	require.Equal(t, math.Nextafter(1, 0), near.Basis().EZ.Z, "the rotation must be near-axial")
	axial, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(math.Pi/4))
	require.NoError(t, err)
	require.Equal(t, 1.0, axial.Basis().EZ.Z, "the rotation must be axial")

	identityRecord := &loftrecord.SourceIdentity{}
	join := &facetedrecord.LoftMirrorJoinRecord{Lower: &topo.Body{Payload: loftrecord.Payload{
		SourceIdentity: identityRecord, Frame1: middle, Xform: identity, AuthenticatedReconstruction: true,
	}}}
	area := measurement.Measurement{Value: units.SquareMillimeters(40), Bound: units.SquareMillimeters(1)}
	pair := func(motion r3.Transform) *topo.Body {
		fp := facetedrecord.Payload{ContactAudited: true, Xform: motion,
			LoftMirrorCopy: &facetedrecord.LoftMirrorPlacementRecord{Source: join, Motion: motion}}
		if motion == identity {
			fp = facetedrecord.Payload{ContactAudited: true, Xform: identity, LoftMirrorJoin: join}
		}
		return &topo.Body{Payload: fp, Area: area}
	}
	tooth := func(motion r3.Transform) *topo.Body {
		return &topo.Body{Payload: loftrecord.Payload{
			SourceIdentity: identityRecord, Xform: motion, AuthenticatedReconstruction: true,
		}}
	}
	shift, swept, ok := loftcylinder.MirrorUpperBridge(pair(near), near, 1)
	require.True(t, ok)
	// The Join accumulates each pair's charge with an upward sum.
	swept = proofbound.AbsSumUpper(0, swept)
	record := func(motions ...r3.Transform) *facetedrecord.LoftMirrorCylinderBoreRecord {
		pairs := make([]*topo.Body, len(motions))
		for i, motion := range motions {
			pairs[i] = pair(motion)
		}
		return &facetedrecord.LoftMirrorCylinderBoreRecord{Join: &facetedrecord.LoftMirrorCylinderJoinRecord{
			Source: join, Pairs: pairs, BridgeBound: shift, BridgeVolume: swept,
		}}
	}
	source := loftcylinder.ChamferSource{
		OriginalTeeth: []*topo.Body{tooth(near), tooth(axial), tooth(identity)},
	}

	shifts, err := loftcylinder.MirrorChamferUpperShifts(record(identity, axial, near), source)
	require.NoError(t, err)
	require.Len(t, shifts, 3)
	// 2m(1-EZ.Z) with m = 1 and EZ.Z one float step below one.
	want := big.NewRat(1, 1<<52)
	for i, expected := range []*big.Rat{want, new(big.Rat), new(big.Rat)} {
		require.Zero(t, expected.Cmp(shifts[i]), "tooth %d has shift %s, want %s", i, shifts[i], expected)
	}

	shifts, err = loftcylinder.MirrorChamferUpperShifts(record(identity, axial),
		loftcylinder.ChamferSource{OriginalTeeth: []*topo.Body{tooth(axial), tooth(identity)}})
	require.NoError(t, err)
	require.Nil(t, shifts, "axial teeth need no reflected check")

	for name, bad := range map[string]func(*facetedrecord.LoftMirrorCylinderBoreRecord){
		"bridge bound": func(r *facetedrecord.LoftMirrorCylinderBoreRecord) {
			r.Join.BridgeBound = math.Nextafter(shift, 0)
		},
		"bridge volume": func(r *facetedrecord.LoftMirrorCylinderBoreRecord) {
			r.Join.BridgeVolume = math.Nextafter(swept, 0)
		},
		"duplicate placement": func(r *facetedrecord.LoftMirrorCylinderBoreRecord) {
			r.Join.Pairs[1] = pair(near)
		},
	} {
		r := record(identity, axial, near)
		bad(r)
		_, err := loftcylinder.MirrorChamferUpperShifts(r, source)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
	}
	_, err = loftcylinder.MirrorChamferUpperShifts(record(identity, axial, near),
		loftcylinder.ChamferSource{OriginalTeeth: []*topo.Body{tooth(near), tooth(axial), tooth(near)}})
	require.ErrorIs(t, err, decaderr.ErrUnsupported, "a source tooth has no matching placement")
}
