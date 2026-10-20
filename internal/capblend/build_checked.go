package capblend

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// BuildChecked runs the existence and constructed-geometry gates (SX6, SX7,
// SX12, and SX13's axial half) and, once every gate passes, builds the body. It
// is the shared entry the cap-loop Chamfer calls once a clean selection is
// classified. SX13's radial half is decided per circular wall as the band is
// constructed, in capband.BandRadius. start and end are the two
// caps' own setbacks (§8.3.1); a cap with no selected loop reads neither.
func BuildChecked(ctx context.Context, env topo.Env, pp prismrecord.Payload,
	start, end Setback, startLoops, endLoops map[int]bool, source *LoftSource) (*topo.Body, error) {
	height := pp.Z1 - pp.Z0
	loops := append([]sectionrecord.LoopRecord{pp.Profile.Outer}, pp.Profile.Holes...)
	cbp := Payload{
		Profile:    pp.Profile,
		Frame:      pp.Frame,
		Z0:         pp.Z0,
		Z1:         pp.Z1,
		Z0Delta:    pp.Z0Delta,
		Z1Delta:    pp.Z1Delta,
		Xform:      pp.Xform,
		Start:      start,
		End:        end,
		StartLoops: startLoops,
		EndLoops:   endLoops,
		LoftSource: source,
	}

	// SX6 + SX7/SX12: build the "mixed" profile — every selected loop offset
	// its own dc into the material (the cap contour at whichever cap it is
	// selected on; when a loop is selected on BOTH caps the two contours are
	// equal — resolveCapSetbacks gives both caps of such a loop one dc — so
	// one offset serves both),
	// every unselected loop left unchanged — and run the existing exact
	// offset + §5 audit machinery on it. SX6 is the offset's own drop
	// refusal, re-sentinelled here (capband.ChamferDropError); the audit's own
	// refusals keep the sentinel their §4 stage-6 row decided
	// (capband.ChamferAuditError) — SX7/SX12 for a crossing or contact, the base
	// S8/S9 ErrDegenerate for broken nesting. SX12 audits the exact offset
	// FAMILY, not the ruled patch the body builds: at axial fraction s, the
	// denoted miter locus is the parallel section offset by s*dc
	// (docs/modify-reach-design.md §8.3) — a fact about that family alone,
	// independent of which surface later reports the body's volume — and the
	// offset distance to any fixed feature is monotone non-increasing as the
	// offset grows from 0, so a crossing anywhere in the family occurs no
	// later than it occurs at the full offset dc — proving the family
	// disjoint at s=1 certifies every s in [0, 1].
	budget := proofbound.NewWorkBudget(ctx)
	mixed, err := MixedOffsetProfile(budget, cbp)
	if err != nil {
		return nil, capband.ChamferDropError(err)
	}

	// SX7 (band-meeting): a loop chamfered on both caps needs both bands to
	// fit without meeting; a loop chamfered on one cap needs its own band to
	// fit within the sweep. A band reaches its own cap's ds along the sweep
	// (§8.3.1), whatever its dc.
	//
	// It runs AFTER SX6, which docs/modify-reach-design.md puts in stage 5 while
	// this row is stage 6 ("SX6 precedes SX7"). The two rows answer the same §4
	// existence test in opposite directions, and stage order is how §4 keeps one
	// input from having two sentinels: a setback that empties the cap contour
	// names a body that does not exist at any sweep height, so it is SX6's
	// ErrDegenerate whether the sweep is short enough for the bands to meet or
	// not. Deciding the band reach first would let the sweep height alone pick
	// which sentinel that one nonexistent body reports.
	for li := range loops {
		reach := 0.0
		if startLoops[li] {
			reach += start.Ds
		}
		if endLoops[li] {
			reach += end.Ds
		}
		if reach >= height {
			return nil, fmt.Errorf(`%w: the chamfer band(s) on loop %d reach or pass the opposite end of the sweep; a merging kernel is not available`, decaderr.ErrUnsupported, li)
		}
	}

	if err := offset2d.AuditOffsetSectionBudget(budget, pp.Profile, mixed); err != nil {
		return nil, capband.ChamferAuditError(err)
	}
	if err := AuditSetbackSpan(budget, pp.Profile, cbp); err != nil {
		return nil, err
	}
	if err := RequireLevelsSeparate(cbp); err != nil {
		return nil, err
	}
	if err := RequireCornerLoci(budget, cbp); err != nil {
		return nil, err
	}
	return Build(ctx, env, cbp)
}
