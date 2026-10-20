package draftrecord

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Receiver is the straight prism a Draft tapers: the body's payload must be a
// prism that is not already a draft and whose section is exact
// (agent-docs/draft-design.md SD23).
func Receiver(b *topo.Body) (prismrecord.Payload, error) {
	if _, ok := b.Payload.(Payload); ok {
		return prismrecord.Payload{}, fmt.Errorf(`%w: this evaluator drafts a straight prism once; the receiver is already a draft body (draft SD23)`, decaderr.ErrUnsupported)
	}
	pp, ok := b.Payload.(prismrecord.Payload)
	if !ok {
		return prismrecord.Payload{}, fmt.Errorf(`%w: this evaluator drafts a straight prism only (an extrude, its filleted or chamfered result, a tube, or any of these placed) (draft SD23)`, decaderr.ErrUnsupported)
	}
	if err := pp.RequireExactSection("drafts"); err != nil {
		return prismrecord.Payload{}, err
	}
	return pp, nil
}

func hasRole(f *topo.Face, producer provenance.Producer, role string) bool {
	return slices.Contains(f.Origins, provenance.Ref{Producer: producer, Role: role})
}

// NeutralCap reports whether the neutral face is the receiver's capStart (true)
// or capEnd (false); any other face is refused (draft SD20).
func NeutralCap(f *topo.Face, b *topo.Body) (bool, error) {
	if f.Body != b {
		return false, fmt.Errorf(`%w: the neutral face must be a cap of the receiver, and it belongs to another body (draft SD20)`, decaderr.ErrUnsupported)
	}
	switch {
	case hasRole(f, b.Origin.Producer, provenance.RoleCapStart):
		return true, nil
	case hasRole(f, b.Origin.Producer, provenance.RoleCapEnd):
		return false, nil
	}
	return false, fmt.Errorf(`%w: the neutral face must be the receiver's capStart or capEnd; a wall or any other face is staged (draft SD20)`, decaderr.ErrUnsupported)
}

// KeptWalls returns the receiver walls the selection leaves untilted
// (draft SD21, SD22). selected must be faces of b.
func KeptWalls(b *topo.Body, producer provenance.Producer, selected []*topo.Face) (map[offset2d.DraftWall]struct{}, error) {
	for _, f := range selected {
		if hasRole(f, producer, provenance.RoleCapStart) || hasRole(f, producer, provenance.RoleCapEnd) {
			return nil, fmt.Errorf(`%w: the selection contains a cap of the receiver, which has no trace on the neutral plane to tilt about (draft SD22)`, decaderr.ErrDegenerate)
		}
	}
	for _, f := range selected {
		if !selectorquery.IsWallOf(f, producer) {
			return nil, fmt.Errorf(`%w: the selection contains a face that is no wall of the receiver (it carries no side(i, j) role of the receiver's own), so it names no wall to tilt (draft SD21)`, decaderr.ErrUnsupported)
		}
	}
	kept := map[offset2d.DraftWall]struct{}{}
	for _, l := range b.Lumps {
		for _, s := range l.Shells {
			for _, f := range s.Faces {
				if !selectorquery.IsWallOf(f, producer) || slices.Contains(selected, f) {
					continue
				}
				for _, o := range f.Origins {
					if o.Producer != producer {
						continue
					}
					var li, j int
					if n, err := fmt.Sscanf(o.Role, "side(%d,%d)", &li, &j); err != nil || n != 2 || o.Role != fmt.Sprintf("side(%d,%d)", li, j) {
						continue
					}
					kept[offset2d.DraftWall{Loop: li, Seg: j}] = struct{}{}
				}
			}
		}
	}
	return kept, nil
}

// FromPrism builds the draft of the straight prism pp about its near or far
// cap, tilting every wall but kept. The result is not yet committed.
func FromPrism(ctx context.Context, env topo.Env, pp prismrecord.Payload, nearStart bool, alpha, alphaDelta float64,
	kept map[offset2d.DraftWall]struct{}) (*topo.Body, error) {
	return Build(ctx, env, Payload{
		Profile:    pp.Profile,
		Frame:      pp.Frame,
		Z0:         pp.Z0,
		Z1:         pp.Z1,
		Z0Delta:    pp.Z0Delta,
		Z1Delta:    pp.Z1Delta,
		NearStart:  nearStart,
		Taper:      alpha,
		TaperDelta: alphaDelta,
		Xform:      pp.Xform,
		Kept:       kept,
	})
}

// Extrude builds the tapered extrude whose base is the draft payload base; a
// section that straddles the sketch plane builds as a two-sided draft.
func Extrude(ctx context.Context, env topo.Env, base Payload) (*topo.Body, error) {
	if base.Z0 < 0 && base.Z1 > 0 {
		negative, positive := base, base
		negative.Z1, negative.Z1Delta, negative.NearStart = 0, 0, false
		positive.Z0, positive.Z0Delta, positive.NearStart = 0, 0, true
		return BuildTwoSided(ctx, env, TwoSidedPayload{Negative: negative, Positive: positive})
	}
	return Build(ctx, env, base)
}
