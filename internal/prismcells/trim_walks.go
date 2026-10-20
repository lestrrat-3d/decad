package prismcells

import (
	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// TrimProfileWalks resolves profile's Outer-then-Holes walks for a trimmed
// ribbon's own Bounds reading (agent-docs/surface-intersection-design.md §7),
// charging TrimCutChargeUV into exactly the endpoint bound whose OWN
// recorded parameter is not a natural bound (0 or 1, per field — never per
// segment, since a segment cut on only one side keeps its natural end exact)
// — the coordinate the arrangement computed for that end, never one the
// record states verbatim. An endpoint whose own parameter IS natural gets no
// charge, whether or not the segment's OTHER end was cut, which is what lets
// an extreme won by an untouched vertex stay exactly as tight as the
// untrimmed sheet's own (T170) while an extreme won by a genuine cut point
// carries the bound its own construction owes (agent-docs/surface-design.md's
// end-cut fixture). The decision reads only the two recorded floats — never a
// coordinate comparison, never whether a value "looks like" it moved.
//
// This is never called for a plain chain Extrude ribbon: chainrecord.Build
// gates it on pp.sectionDelta != 0, which no construction but this design's
// Trim ever sets, so an ordinary ribbon keeps resolving through walkOf with
// no augmentation.
func TrimProfileWalks(profile momentinput.Profile, work *freeform.FreeformWork) (*momentinput.ProfileWalks, error) {
	before, beforeRecon := boundarywalk.WorkSpent(work)
	outer, holes, err := TrimBoundsWalks(profile.Outer, profile.Holes, func(seg sectionrecord.CurveSegment) (survey2d.SegmentWalk, error) {
		return boundarywalk.WalkOf(seg, work)
	})
	if err != nil {
		return nil, err
	}
	after, afterRecon := boundarywalk.WorkSpent(work)
	return &momentinput.ProfileWalks{
		Profile:             profile,
		Outer:               outer,
		Holes:               holes,
		Spent:               after - before,
		ReconstructionSpent: afterRecon - beforeRecon,
		Metered:             true,
	}, nil
}
