package brepmodify

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// LoftCapChamfer is the matching axial loft's complete outer-cap loop
// route. Its cap contour follows the loft's certified held station polygon.
func LoftCapChamfer(ctx context.Context, env topo.Env, b *topo.Body, pl loftrecord.Payload, edges []*topo.Edge,
	d, dDelta float64, asym *Asymmetric) (*topo.Body, bool, error) {
	if asym != nil || pl.SurfaceResult || pl.Xform != r3.Identity() ||
		!sectionrecord.IdenticalRecord(pl.Profile0, pl.Profile1) ||
		len(pl.Profile0.Holes) > 1 ||
		pl.Frame0.U() != pl.Frame1.U() || pl.Frame0.V() != pl.Frame1.V() {
		return nil, false, nil
	}
	if len(pl.Profile0.Holes) == 1 {
		if len(pl.Profile0.Holes[0].Segments) != 1 {
			return nil, false, nil
		}
		if _, ok := pl.Profile0.Holes[0].Segments[0].(sectionrecord.CircleSeg); !ok {
			return nil, false, nil
		}
	}
	for _, offset := range pl.Alignment {
		if offset != 0 {
			return nil, false, nil
		}
	}
	end := pl.Frame0.ToLocal(pl.Frame1.Origin())
	if end.X != 0 || end.Y != 0 || end.Z <= 2*d {
		return nil, false, nil
	}
	var caps [2]*topo.Face
	for _, face := range topobuild.Faces(b) {
		for _, origin := range face.Origins {
			if origin == capRef(b, provenance.RoleCapStart) {
				caps[0] = face
			}
			if origin == capRef(b, provenance.RoleCapEnd) {
				caps[1] = face
			}
		}
	}
	if caps[0] == nil || caps[1] == nil {
		return nil, false, nil
	}
	if !loftcontact.MatchingHeldCapOuters(pl.Frame0, caps[0], caps[1]) {
		return nil, false, nil
	}
	selected := make(map[*topo.Edge]struct{}, len(edges))
	for _, edge := range edges {
		selected[edge] = struct{}{}
	}
	for _, endCap := range caps {
		if len(endCap.Loops) == 0 || !endCap.Loops[0].Outer {
			return nil, false, nil
		}
		for _, ce := range endCap.Loops[0].CoEdges {
			edge := ce.Edge
			if _, ok := selected[edge]; !ok {
				return nil, false, nil
			}
			delete(selected, edge)
		}
	}
	if len(selected) != 0 {
		return nil, false, nil
	}
	outer := make([]sectionrecord.CurveSegment, 0, len(caps[1].Loops[0].CoEdges))
	for _, ce := range caps[1].Loops[0].CoEdges {
		start := pl.Frame0.ToLocal(topobuild.CoEdgeStart(ce).Position)
		stop := pl.Frame0.ToLocal(topobuild.CoEdgeEnd(ce).Position)
		outer = append(outer, sectionrecord.LineSeg{
			Start:  sectionrecord.Point2{U: start.X, V: start.Y},
			End:    sectionrecord.Point2{U: stop.X, V: stop.Y},
			TStart: 0, TEnd: 1,
		})
	}
	profile := momentinput.Profile{
		Outer: sectionrecord.LoopRecord{Segments: outer},
		Holes: pl.Profile0.Holes,
	}
	pp := prismrecord.Payload{Profile: profile, Frame: pl.Frame0, Z0: 0, Z1: end.Z, Xform: r3.Identity()}
	setback := capblend.Setback{Dc: d, DcDelta: dDelta, Ds: d, DsDelta: dDelta}
	source := &capblend.LoftSource{Profile: pl.Profile0, Proof: pl.Proof}
	result, err := capblend.BuildChecked(ctx, env, pp, setback, setback,
		map[int]bool{0: true}, map[int]bool{0: true}, source)
	if err != nil {
		return nil, true, fmt.Errorf("loft cap chamfer: %w", err)
	}
	return result, true, nil
}

func capRef(b *topo.Body, role string) provenance.Ref {
	return provenance.Ref{Producer: b.Origin.Producer, Role: role}
}
