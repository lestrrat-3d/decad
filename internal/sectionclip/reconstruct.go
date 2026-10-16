package sectionclip

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// SmoothRectangularSectionRecord reads the exact rectangle of an original
// smooth rectangular loft at the plane's world-XY level.
func SmoothRectangularSectionRecord(fp facetedPayload, plane r3.Frame) (profileRecord, bool) {
	// Placement drops the certificate. The xform and normal checks are
	// reject-only backstops for future producers and placement paths.
	if fp.SmoothRectSection == nil || fp.Xform != r3.Identity() ||
		plane.U() != r3.NewVec(1, 0, 0) || plane.V() != r3.NewVec(0, 1, 0) ||
		plane.N() != r3.NewVec(0, 0, 1) {
		return profileRecord{}, false
	}
	origin := fp.SmoothRectSection.Origin()
	if plane.Origin().X != origin.X || plane.Origin().Y != origin.Y {
		return profileRecord{}, false
	}
	w, h, ok := fp.SmoothRectSection.At(plane.Origin().Z)
	if !ok {
		return profileRecord{}, false
	}
	a, b := planePoint{U: -w, V: -h}, planePoint{U: w, V: -h}
	c, d := planePoint{U: w, V: h}, planePoint{U: -w, V: h}
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{
		lineSeg{Start: a, End: b, TStart: 0, TEnd: 1},
		lineSeg{Start: b, End: c, TStart: 0, TEnd: 1},
		lineSeg{Start: c, End: d, TStart: 0, TEnd: 1},
		lineSeg{Start: d, End: a, TStart: 0, TEnd: 1},
	}}}, true
}

// SmoothHomotheticSectionRecord reads the exact positive-scaled whole-line
// profile of an original three-section smooth loft at the plane's world-XY level.
func SmoothHomotheticSectionRecord(fp facetedPayload, plane r3.Frame) (profileRecord, bool) {
	if fp.SmoothHomotheticSection == nil || fp.Xform != r3.Identity() ||
		plane.U() != r3.NewVec(1, 0, 0) || plane.V() != r3.NewVec(0, 1, 0) ||
		plane.N() != r3.NewVec(0, 0, 1) {
		return profileRecord{}, false
	}
	origin := fp.SmoothHomotheticSection.Origin()
	if plane.Origin().X != origin.X || plane.Origin().Y != origin.Y {
		return profileRecord{}, false
	}
	return fp.SmoothHomotheticSection.At(plane.Origin().Z)
}

// SectionBrepStack reads only the certified slab profiles of an A1 Union.
// The same exact frame and strict interior-level check used for a prism
// applies to each slab separately. An interface level names two different
// sections, so neither slab admits it. The proxy is the stacked-union
// operand's prism proxy and slabs are its slabs.
func SectionBrepStack(ctx context.Context, proxy prismPayload, slabs []stackedrecord.Slab, plane r3.Frame) (
	*sketch.Sketch, []*sketch.Profile, error,
) {
	if proxy.SectionDelta != 0 {
		return nil, nil, fmt.Errorf(`%w: BRep section has a displaced source profile`, decaderr.ErrUnsupported)
	}
	for _, slab := range slabs {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		candidate := proxy
		candidate.Z0, candidate.Z1 = slab.Z0, slab.Z1
		if !candidate.SectionPlaneInside(plane) {
			continue
		}
		if slab.Z0Delta != 0 || slab.Z1Delta != 0 || len(slab.Regions) != 1 {
			return nil, nil, fmt.Errorf(`%w: BRep section needs one exact slab region`, decaderr.ErrUnsupported)
		}
		return ReconstructExactSection(ctx, slab.Regions[0], plane)
	}
	return nil, nil, fmt.Errorf(`%w: section plane lies outside BRep slab interiors`, decaderr.ErrUnsupported)
}

// ReconstructExactSection charges the profile record and rebuilds it as a
// Sketch profile on the plane frame.
func ReconstructExactSection(ctx context.Context, profile profileRecord, plane r3.Frame) (
	*sketch.Sketch, []*sketch.Profile, error,
) {
	work := freeform.NewFreeformWork()
	if _, err := momentinput.ChargeReconstruction(profile, work); err != nil {
		return nil, nil, err
	}
	s, result, err := momentinput.ReconstructProfileOnFrame(ctx, profile, plane)
	if err != nil {
		return nil, nil, err
	}
	return s, []*sketch.Profile{result}, nil
}
