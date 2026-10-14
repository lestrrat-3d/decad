package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// buildLoftAnalyticBoreTopology replaces only the certified hole's triangle
// faces. The loft assembly and its mass and tessellation proofs stay intact:
// each former hole triangle role maps to the one cylindrical source face.
func buildLoftAnalyticBoreTopology(ctx context.Context, body *Body, ref producerID, pl loftPayload,
	a loftmesh.Assembly, start, end *Face, walls []*Face) ([]*Face, error) {
	if len(pl.profile0.Holes) != 1 || len(pl.profile0.Holes[0].Segments) != 1 ||
		len(a.VIdx) != 2 || len(start.loops) != 2 || len(end.loops) != 2 || len(walls) != a.Walls {
		return nil, fmt.Errorf("%w: a through-bore loft lost its certified hole", ErrUnsupported)
	}
	circle, ok := pl.profile0.Holes[0].Segments[0].(circleSeg)
	if !ok || circle.CCW || circle.TStart != 1 || circle.TEnd != 0 {
		return nil, fmt.Errorf("%w: a through-bore loft has no whole circle", ErrUnsupported)
	}
	level := pl.frame0.ToLocal(pl.frame1.Origin())
	if level.X != 0 || level.Y != 0 || level.Z <= 0 ||
		pl.frame0.ToWorld(r3.NewVec(0, 0, level.Z)) != pl.frame1.Origin() ||
		pl.frame0.ToWorld(r3.NewVec(circle.Center.U, circle.Center.V, level.Z)) !=
			pl.frame1.ToWorldUV(circle.Center.U, circle.Center.V) {
		return nil, fmt.Errorf("%w: a through-bore loft has no common circle axis", ErrUnsupported)
	}
	pp := prismPayload{Payload: prismrecord.Payload{
		Frame: pl.frame0, Z0: 0, Z1: level.Z, Xform: pl.xform,
		Z0Delta: a.Delta, Z1Delta: a.Delta,
	}}
	faces, bottom, top, _, err := buildLoopSidesAs(ctx, body, ref, pp, 1, true,
		pl.profile0.Holes[0], freeform.NewFreeformWork(), nil, levelToken{}, levelToken{}, false)
	if err != nil {
		return nil, err
	}
	if len(faces) != 1 || len(bottom) != 1 || len(top) != 1 {
		return nil, fmt.Errorf("%w: a through-bore circle did not build one wall", ErrUnsupported)
	}
	wall := faces[0]
	wall.n.Origins = nil
	// The prism builder states these circle loops in its local sweep order.
	// Carry the Loft assembly's whole-shell reversal onto the cylinder too;
	// the cap loops below already receive the same reversal.
	for _, loop := range wall.loops {
		loop.coedges = loftLoopCoedges(loop.coedges, a.Reversed)
	}
	// buildLoopSidesAs links its side face immediately, while evalLoft links
	// every live face after this rewrite. Clear those two private links once.
	bottom[0].edge.faces = nil
	top[0].edge.faces = nil
	filtered := make([]*Face, 0, len(walls))
	for k, face := range walls {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if a.Cell[k][0] == 1 {
			wall.n.Origins = append(wall.n.Origins, face.n.Origins...)
			continue
		}
		filtered = append(filtered, face)
	}
	if len(wall.n.Origins) < 2 {
		return nil, fmt.Errorf("%w: a through-bore has no source wall cells", ErrUnsupported)
	}
	filtered = append(filtered, wall)
	start.loops[1] = &Loop{n: start.loops[1].n,
		coedges: loftLoopCoedges([]coedge{{edge: bottom[0].edge, forward: false}}, a.Reversed)}
	end.loops[1] = &Loop{n: end.loops[1].n,
		coedges: loftLoopCoedges([]coedge{{edge: top[0].edge, forward: true}}, a.Reversed)}
	// The cap now has a circular hole. Its outer polygon is the original
	// assembly's held contour; only the hole area changes from its station
	// polygon to the exact circle's bounded integral.
	holeArea, err := momentinput.LoopEnclosedArea(ctx, pl.profile0.Holes[0])
	if err != nil {
		return nil, err
	}
	for cap, face := range []*Face{start, end} {
		pts, loops := a.Pts0, a.LoopIdx0
		if cap == 1 {
			pts, loops = a.Pts1, a.LoopIdx1
		}
		outer := loftmesh.CapPolygonAreaRat(pts, [][]int{loops[0]})
		value, _ := outer.Float64()
		if proofbound.IsNonFinite(value) {
			return nil, fmt.Errorf("%w: a through-bore cap area is not finite", ErrUnsupported)
		}
		area := proofbound.BoundedSub(
			proofbound.MeasuredScalar(value, proofarith.RationalFloatError(outer, value)), holeArea)
		face.n.Area = area.Value
		face.n.AreaBound = proofbound.AbsSumUpper(area.Bound, face.n.AreaBound)
	}
	return filtered, nil
}
