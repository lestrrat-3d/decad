package loftrecord

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// directSections records two bounded analytic loops at original construction.
// Line and arc segment values contain no pointers into the caller's records.
type directSections struct {
	body      *topo.Body
	frames    [2]r3.Frame
	planes    [2]sectionrecord.PlaneRecord
	segments  [2][]momentinput.CurveSegment
	alignment []int
}

func originalLineArcSections(p Payload) bool {
	if !p.AuthenticatedReconstruction || p.SurfaceResult || p.ThroughBoreCut || p.Xform != r3.Identity() || p.ConstructionDelta != 0 || p.JoinDelta != 0 || p.JoinPlaneGap != 0 || len(p.Alignment) > 1 {
		return false
	}
	for _, profile := range [2]momentinput.Profile{p.Profile0, p.Profile1} {
		if len(profile.Holes) != 0 || len(profile.Outer.Segments) == 0 {
			return false
		}
		for _, s := range profile.Outer.Segments {
			switch s.(type) {
			case momentinput.LineSeg, momentinput.ArcSeg:
			default:
				return false
			}
		}
	}
	return true
}
func directSectionClass(p Payload) bool {
	return len(p.Profile0.Outer.Segments) <= 64 && len(p.Profile1.Outer.Segments) <= 64 && originalLineArcSections(p)
}

func sealDirectSections(b *topo.Body, p Payload) *directSections {
	if !directSectionClass(p) {
		return nil
	}
	return &directSections{
		body:      b,
		frames:    [2]r3.Frame{p.Frame0, p.Frame1},
		planes:    [2]sectionrecord.PlaneRecord{p.Plane0, p.Plane1},
		alignment: slices.Clone(p.Alignment),
		segments:  [2][]momentinput.CurveSegment{slices.Clone(p.Profile0.Outer.Segments), slices.Clone(p.Profile1.Outer.Segments)},
	}
}

// MatchesDirectSections compares the original section carriers, alignment and
// placement used to bound a stopped bevel against their construction records.
func (p Payload) MatchesDirectSections(b *topo.Body) bool {
	if p.SourceIdentity == nil || p.SourceIdentity.direct == nil || !directSectionClass(p) {
		return false
	}
	s := p.SourceIdentity.direct
	return s.body == b && s.frames == [2]r3.Frame{p.Frame0, p.Frame1} &&
		s.planes == [2]sectionrecord.PlaneRecord{p.Plane0, p.Plane1} && slices.Equal(s.alignment, p.Alignment) &&
		slices.Equal(s.segments[0], p.Profile0.Outer.Segments) && slices.Equal(s.segments[1], p.Profile1.Outer.Segments)
}
