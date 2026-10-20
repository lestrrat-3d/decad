package brepmodify

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// PrismUnionFillet rounds one untouched outer vertical edge of a faceted Union
// of two rectangular prisms when the other prism is certified clear of the
// full removed radius band (docs/modify-reach-design.md §10.1).
func PrismUnionFillet(ctx context.Context, env topo.Env, hooks Hooks, b *topo.Body, edges []*topo.Edge,
	rec *facetedrecord.PrismUnionFilletRecord, radius, radiusDelta float64) (*topo.Body, bool, error) {
	if len(edges) != 1 || radiusDelta != 0 || !(radius > 0) || proofbound.IsNonFinite(radius) {
		return nil, false, nil
	}
	for _, pair := range [][2]*topo.Body{{rec.First, rec.Second}, {rec.Second, rec.First}} {
		pp, ok := pair[0].Payload.(prismrecord.Payload)
		if !ok {
			continue
		}
		corner, band, sourceEdge, ok := prismUnionFilletCorner(pair[0], pp, edges[0], radius)
		if !ok {
			continue
		}
		if !prismUnionFilletSourceFaces(edges[0], sourceEdge) ||
			!prismUnionFilletBandSeparate(pair[1], band) {
			continue
		}
		out, err := filletPrismUnionSource(ctx, env, pp, sourceEdge, corner, radius)
		if err != nil {
			return nil, true, err
		}
		// The bounded band R removed from the exact rectangular source is
		// strictly outside the other source's true box. Thus R∩B is empty,
		// and (A\R)∪B equals (A∪B)\R. The ordinary Boolean rebuild supplies
		// its own occupied-volume and boundary certificates.
		joined, err := hooks.Boolean(ctx, meshbool.OpUnion, env, out, pair[1])
		if err != nil {
			return nil, true, err
		}
		return joined, true, nil
	}
	return nil, false, nil
}

// The source must be an exact, unplaced +XY rectangle. Its selected edge and
// both endpoints must be the exact world-space edge of one rectangle corner.
// The returned outward box contains the whole removed radius band, including
// every point of its circular arc, with outward-rounded dyadic limits.
func prismUnionFilletCorner(source *topo.Body, pp prismrecord.Payload, result *topo.Edge, radius float64) (
	sectionrecord.Point2, compositesweep.AuditBox, *topo.Edge, bool,
) {
	if source == nil || result == nil || result.Start == nil || result.End == nil ||
		pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 || pp.SurfaceResult ||
		pp.Xform != r3.Identity() || pp.Frame.U() != (r3.Vec{X: 1}) ||
		pp.Frame.V() != (r3.Vec{Y: 1}) || pp.Frame.N() != (r3.Vec{Z: 1}) {
		return sectionrecord.Point2{}, compositesweep.AuditBox{}, nil, false
	}
	rect, ok := momentinput.AxisRectangle(pp.Profile)
	if !ok || pp.Z0 == pp.Z1 || !momentinput.PrismUnionFilletRadiusFits(rect, radius) {
		return sectionrecord.Point2{}, compositesweep.AuditBox{}, nil, false
	}
	chainBound, ok := boundaryChainBound(result)
	if !ok || chainBound != 0 || result.Start.Bound.Mag() != 0 || result.End.Bound.Mag() != 0 ||
		len(result.Faces) != 2 {
		return sectionrecord.Point2{}, compositesweep.AuditBox{}, nil, false
	}
	corners := [4][2]float64{{rect.U0, rect.V0}, {rect.U1, rect.V0}, {rect.U1, rect.V1}, {rect.U0, rect.V1}}
	for _, corner := range corners {
		p0, p1 := pp.Point(corner[0], corner[1], pp.Z0), pp.Point(corner[0], corner[1], pp.Z1)
		if !prismUnionFilletSameEnds(result, p0, p1) {
			continue
		}
		var sourceEdge *topo.Edge
		for _, edge := range topobuild.Edges(source) {
			if _, ok := edge.Curve.(surfacegeom.Line3); ok && edge.Start != nil && edge.End != nil &&
				edge.Start.Bound.Mag() == 0 && edge.End.Bound.Mag() == 0 &&
				prismUnionFilletSameEnds(edge, p0, p1) {
				if sourceEdge != nil {
					return sectionrecord.Point2{}, compositesweep.AuditBox{}, nil, false
				}
				sourceEdge = edge
			}
		}
		if sourceEdge == nil {
			return sectionrecord.Point2{}, compositesweep.AuditBox{}, nil, false
		}
		xOther := rect.U0
		if corner[0] == rect.U0 {
			xOther = rect.U1
		}
		yOther := rect.V0
		if corner[1] == rect.V0 {
			yOther = rect.V1
		}
		band, ok := compositesweep.PrismUnionFilletBand(pp.Frame.Origin(), corner, xOther, yOther, pp.Z0, pp.Z1, radius)
		return sectionrecord.Point2{U: corner[0], V: corner[1]}, band, sourceEdge, ok
	}
	return sectionrecord.Point2{}, compositesweep.AuditBox{}, nil, false
}

func prismUnionFilletSameEnds(edge *topo.Edge, p, q r3.Vec) bool {
	return (edge.Start.Position == p && edge.End.Position == q) ||
		(edge.Start.Position == q && edge.End.Position == p)
}

func prismUnionFilletSourceFaces(result, source *topo.Edge) bool {
	if len(source.Faces) != 2 || len(result.Faces) != 2 {
		return false
	}
	for _, face := range source.Faces {
		if !isPlanar(face) || len(face.Origins) != 1 || face.Origins[0].Role == provenance.RoleCapStart ||
			face.Origins[0].Role == provenance.RoleCapEnd {
			return false
		}
	}
	for _, face := range result.Faces {
		if len(face.Origins) != 1 || !isPlanar(face) {
			return false
		}
	}
	a, b := source.Faces[0].Origins[0], source.Faces[1].Origins[0]
	x, y := result.Faces[0].Origins[0], result.Faces[1].Origins[0]
	return a != b && ((a == x && b == y) || (a == y && b == x))
}

func prismUnionFilletBandSeparate(tool *topo.Body, band compositesweep.AuditBox) bool {
	box := tool.Bounds
	return !math.IsNaN(box.Bound.Mag()) && !math.IsInf(box.Bound.Mag(), 0) &&
		compositesweep.BoxesStrictlySeparated(band, compositesweep.AuditBox{
			Min: box.Min, Max: box.Max, Bound: box.Bound.Mag(),
		})
}

func filletPrismUnionSource(ctx context.Context, env topo.Env, pp prismrecord.Payload,
	selected *topo.Edge, corner sectionrecord.Point2, radius float64) (*topo.Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, pp.Profile)
	if err != nil {
		return nil, err
	}
	if len(loops) != 1 || len(loops[0].Walks) != 4 {
		return nil, fmt.Errorf("%w: the source rectangle has no four-corner walk", decaderr.ErrUnsupported)
	}
	li, ci, found, err := modifyselect.MatchCorner(budget, pp, loops, selected)
	if err != nil {
		return nil, err
	}
	if !found || li != 0 || loops[0].Walks[ci].StartU != corner.U ||
		loops[0].Walks[ci].StartV != corner.V {
		return nil, fmt.Errorf("%w: the source edge does not match the authenticated corner", decaderr.ErrUnsupported)
	}
	blend, err := offset2d.Fillet(loops[0].Walks, ci, radius, sectionaudit.Tolerance)
	if err != nil {
		return nil, err
	}
	if !offset2d.PrismUnionFilletQuarterArc(pp.Profile, corner, radius, blend) {
		return nil, fmt.Errorf("%w: the rectangular corner has no exact inward quarter-circle", decaderr.ErrUnsupported)
	}
	blends := []map[int]*offset2d.Blend{{ci: blend}}
	profile, arcs, err := offset2d.RewriteProfileBudget(budget, pp.Profile, loops, blends)
	if err != nil {
		return nil, err
	}
	if err := offset2d.AuditRewriteBudget(budget, pp.Profile, profile, loops, blends); err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return nil, err
		}
		return nil, fmt.Errorf("%w: the rectangular source fillet failed its section audit: %v", decaderr.ErrUnsupported, err)
	}
	return prismrecord.Build(ctx, env, prismrecord.Payload{
		Profile: profile, Frame: pp.Frame, Z0: pp.Z0, Z1: pp.Z1,
		Xform: pp.Xform, BlendSegs: arcs, BlendKind: string(breprecord.BandFillet),
	}, freeform.NewFreeformWork())
}

// boundaryChainBound returns the chord-chain bound of a Boolean rim: a
// FacetedCurve's own, or the one a retained curve replaced. ok is false for
// any other edge.
func boundaryChainBound(e *topo.Edge) (float64, bool) {
	if curve, faceted := e.Curve.(surfacegeom.FacetedCurve); faceted {
		return curve.Bound.Mag(), true
	}
	if e.RetainedCurve {
		return e.ChainBound, true
	}
	return 0, false
}

// chamferPrismUnion removes one triangular corner prism from an authenticated
// source of a faceted Union. The other source stays strictly outside the
// corner box, so the ordinary Boolean rebuild certifies the modified Union.
// PrismUnionChamfer removes one triangular corner prism from an authenticated
// source of a faceted Union. The other source stays strictly outside the
// corner box, so the ordinary Boolean rebuild certifies the modified Union.
func PrismUnionChamfer(ctx context.Context, env topo.Env, hooks Hooks, b *topo.Body, edges []*topo.Edge,
	rec *facetedrecord.PrismUnionFilletRecord, setback, setbackDelta float64) (*topo.Body, bool, error) {
	if len(edges) != 1 || setbackDelta != 0 || !(setback > 0) || proofbound.IsNonFinite(setback) {
		return nil, false, nil
	}
	for _, pair := range [][2]*topo.Body{{rec.First, rec.Second}, {rec.Second, rec.First}} {
		pp, ok := pair[0].Payload.(prismrecord.Payload)
		if !ok {
			continue
		}
		corner, band, sourceEdge, ok := prismUnionFilletCorner(pair[0], pp, edges[0], setback)
		if !ok || !prismUnionFilletSourceFaces(edges[0], sourceEdge) {
			continue
		}
		if !prismUnionFilletBandSeparate(pair[1], band) {
			return nil, true, fmt.Errorf("%w: another Union source occupies the selected chamfer corner", decaderr.ErrUnsupported)
		}
		out, err := chamferPrismUnionSource(ctx, env, pp, sourceEdge, corner, setback)
		if err != nil {
			return nil, true, err
		}
		// The removed triangular prism lies inside the certified corner box,
		// which is strictly separate from the unchanged source's true body.
		joined, err := hooks.Boolean(ctx, meshbool.OpUnion, env, out, pair[1])
		if err != nil {
			return nil, true, err
		}
		return joined, true, nil
	}
	return nil, false, nil
}

func chamferPrismUnionSource(ctx context.Context, env topo.Env, pp prismrecord.Payload,
	selected *topo.Edge, corner sectionrecord.Point2, setback float64) (*topo.Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, pp.Profile)
	if err != nil {
		return nil, err
	}
	if len(loops) != 1 || len(loops[0].Walks) != 4 {
		return nil, fmt.Errorf("%w: the source rectangle has no four-corner walk", decaderr.ErrUnsupported)
	}
	li, ci, found, err := modifyselect.MatchCorner(budget, pp, loops, selected)
	if err != nil {
		return nil, err
	}
	if !found || li != 0 || loops[0].Walks[ci].StartU != corner.U ||
		loops[0].Walks[ci].StartV != corner.V {
		return nil, fmt.Errorf("%w: the source edge does not match the authenticated corner", decaderr.ErrUnsupported)
	}
	blend, err := offset2d.Chamfer(loops[0].Walks, ci, setback, setback, sectionaudit.Tolerance)
	if err != nil {
		return nil, err
	}
	if !offset2d.PrismUnionChamferChord(pp.Profile, corner, setback, blend) {
		return nil, fmt.Errorf("%w: the rectangular corner has no exact inward chamfer chord", decaderr.ErrUnsupported)
	}
	blends := []map[int]*offset2d.Blend{{ci: blend}}
	profile, connectors, err := offset2d.RewriteProfileBudget(budget, pp.Profile, loops, blends)
	if err != nil {
		return nil, err
	}
	if err := offset2d.AuditRewriteBudget(budget, pp.Profile, profile, loops, blends); err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return nil, err
		}
		return nil, fmt.Errorf("%w: the rectangular source chamfer failed its section audit: %v", decaderr.ErrUnsupported, err)
	}
	return prismrecord.Build(ctx, env, prismrecord.Payload{
		Profile: profile, Frame: pp.Frame, Z0: pp.Z0, Z1: pp.Z1,
		Xform: pp.Xform, BlendSegs: connectors, BlendKind: string(breprecord.BandChamfer),
	}, freeform.NewFreeformWork())
}

// isPlanar reports whether the face lies on a plane, held or denoted.
func isPlanar(f *topo.Face) bool { return f.Surface.Kind() == surfacegeom.KindPlane || f.HeldPlanar }
