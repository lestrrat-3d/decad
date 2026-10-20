package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// tryLoftCapChamfer is the matching axial loft's complete outer-cap loop
// route. Its cap contour follows the loft's certified held station polygon.
func tryLoftCapChamfer(ctx context.Context, b *Body, pl loftPayload, edges []*Edge,
	d, dDelta float64, asym *asymmetricChamfer) (*Body, bool, error) {
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
		if _, ok := pl.Profile0.Holes[0].Segments[0].(circleSeg); !ok {
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
	var caps [2]*Face
	for _, face := range b.Faces() {
		for _, origin := range face.n.Origins {
			if origin == refOf(CapStart(b)) {
				caps[0] = face
			}
			if origin == refOf(CapEnd(b)) {
				caps[1] = face
			}
		}
	}
	if caps[0] == nil || caps[1] == nil {
		return nil, false, nil
	}
	if !loftcontact.MatchingHeldCapOuters(pl.Frame0, faceNode(caps[0]), faceNode(caps[1])) {
		return nil, false, nil
	}
	selected := make(map[*topo.Edge]struct{}, len(edges))
	for _, edge := range edges {
		selected[edgeNode(edge)] = struct{}{}
	}
	for _, cap := range caps {
		if len(cap.n.Loops) == 0 || !cap.n.Loops[0].Outer {
			return nil, false, nil
		}
		for _, ce := range cap.n.Loops[0].CoEdges {
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
	outer := make([]curveSegment, 0, len(caps[1].n.Loops[0].CoEdges))
	for _, ce := range caps[1].n.Loops[0].CoEdges {
		start := pl.Frame0.ToLocal(topobuild.CoEdgeStart(ce).Position)
		stop := pl.Frame0.ToLocal(topobuild.CoEdgeEnd(ce).Position)
		outer = append(outer, lineSeg{
			Start:  sectionrecord.Point2{U: start.X, V: start.Y},
			End:    sectionrecord.Point2{U: stop.X, V: stop.Y},
			TStart: 0, TEnd: 1,
		})
	}
	profile := profileRecord{
		Outer: loopRecord{Segments: outer},
		Holes: pl.Profile0.Holes,
	}
	pp := prismrecord.Payload{Profile: profile, Frame: pl.Frame0, Z0: 0, Z1: end.Z, Xform: r3.Identity()}
	setback := capSetback{Dc: d, DcDelta: dDelta, Ds: d, DsDelta: dDelta}
	source := &loftCapBandSource{Profile: pl.Profile0, Proof: pl.Proof}
	result, err := capblend.BuildChecked(ctx, envOf(b.n.Doc, b.n.Doc.NextProducer), pp, setback, setback,
		map[int]bool{0: true}, map[int]bool{0: true}, source)
	if err != nil {
		return nil, true, fmt.Errorf("loft cap chamfer: %w", err)
	}
	return bodyHandle(result), true, nil
}
