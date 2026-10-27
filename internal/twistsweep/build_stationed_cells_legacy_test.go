package twistsweep

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// legacyBuildStationedCells is the one-goroutine cell loop that
// buildStationedCells replaces, kept as the reference its differential test
// compares against: every cell in station order against one shared pair of
// work counters and one budget.
func legacyBuildStationedCells(ctx context.Context, env topo.Env, profile profileRecord, xform r3.Transform,
	recordArea float64, poly TwistPolygon, stations []StraightTwistStation, cells int,
	optional bool) ([]compositesweep.SpanPart, []loftrecord.Payload, error) {
	var err error
	parts := make([]compositesweep.SpanPart, cells)
	held := make([]loftrecord.Payload, cells)
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	budget := proofbound.NewWorkBudget(ctx)
	for i := range parts {
		part, buildErr := loftrecord.Build(ctx, env, loftrecord.Payload{
			Profile0: profile, Profile1: profile,
			Plane0: stations[i].Plane, Plane1: stations[i+1].Plane,
			Frame0: stations[i].Frame, Frame1: stations[i+1].Frame,
			Xform: xform, RecordArea: [2]float64{recordArea, recordArea},
			RetainTwistAreaCells: poly.HasArc,
		}, budget, work0, work1)
		if buildErr != nil {
			return nil, nil, fmt.Errorf("twisted sweep cell %d: %w", i, buildErr)
		}
		var heldOK bool
		held[i], heldOK = part.Payload.(loftrecord.Payload)
		if !heldOK {
			return nil, nil, fmt.Errorf(`%w: twisted sweep cell lost its Loft payload`, decaderr.ErrUnsupported)
		}
		if optional && i == 0 && !stationedTwistFirstCellFits(held[i], cells) {
			return nil, nil, fmt.Errorf(`%w: refined twist exceeds the fixed facet-pair work ceiling`, decaderr.ErrUnsupported)
		}
		if _, err := mergeTwistWalls(ctx, env, part, held[i], len(poly.Points)); err != nil {
			return nil, nil, fmt.Errorf("twisted sweep cell %d: %w", i, err)
		}
		parts[i], err = compositesweep.SpanPartOf(part)
		if err != nil {
			return nil, nil, fmt.Errorf("twisted sweep cell %d: %w", i, err)
		}
	}
	return parts, held, nil
}
