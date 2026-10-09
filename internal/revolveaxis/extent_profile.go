package revolveaxis

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ExtentProfile shares analytic walks and their coordinate envelope across
// the three directional reads of one revolve's bounds.
type ExtentProfile struct {
	Walks      *momentinput.ProfileWalks
	CoordUpper float64
}

// AnalyticProfile leaves free-form and unknown segment kinds on the original
// per-read path, including their work charges and refusal order.
func AnalyticProfile(ctx context.Context, profile momentinput.Profile) (bool, error) {
	for _, loop := range append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...) {
		for _, segment := range loop.Segments {
			if err := ctx.Err(); err != nil {
				return false, err
			}
			switch segment.(type) {
			case sectionrecord.LineSeg, sectionrecord.ArcSeg, sectionrecord.CircleSeg:
			default:
				return false, nil
			}
		}
	}
	return true, ctx.Err()
}

// ResolveAnalyticExtentProfile resolves each analytic walk once and polls
// cancellation before each segment. Analytic walks charge no free-form work.
func ResolveAnalyticExtentProfile(
	ctx context.Context, profile momentinput.Profile, work *freeform.FreeformWork,
) (*ExtentProfile, error) {
	walks := &momentinput.ProfileWalks{
		Profile: profile,
		Outer:   make([]survey2d.SegmentWalk, len(profile.Outer.Segments)),
		Holes:   make([][]survey2d.SegmentWalk, len(profile.Holes)),
	}
	coordUpper := 0.0
	resolve := func(segments []sectionrecord.CurveSegment, result []survey2d.SegmentWalk) error {
		for i, segment := range segments {
			if err := ctx.Err(); err != nil {
				return err
			}
			walk, err := boundarywalk.WalkOf(segment, work)
			if err != nil {
				return err
			}
			if err := boundarywalk.RequireAnalyticWalk(walk, "a placed cap frame"); err != nil {
				return err
			}
			result[i] = walk
			coordUpper = math.Max(coordUpper, walk.CoordUpper)
		}
		return nil
	}
	if err := resolve(profile.Outer.Segments, walks.Outer); err != nil {
		return nil, err
	}
	for i, hole := range profile.Holes {
		walks.Holes[i] = make([]survey2d.SegmentWalk, len(hole.Segments))
		if err := resolve(hole.Segments, walks.Holes[i]); err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return &ExtentProfile{Walks: walks, CoordUpper: coordUpper}, nil
}
