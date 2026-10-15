package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// prismUnionFilletRecord keeps the two immutable sources of a mesh Union.
// Only an original Union gets this record; placement drops it. The sources
// may be retired, so every use authenticates the selected result edge and
// works through booleanBody without committing an intermediate body.
type prismUnionFilletRecord struct{ First, Second *topo.Body }

func recognizePrismUnionFillet(a, b *Body) *prismUnionFilletRecord {
	if _, ok := a.n.Payload.(prismPayload); !ok {
		return nil
	}
	if _, ok := b.n.Payload.(prismPayload); !ok {
		return nil
	}
	return &prismUnionFilletRecord{First: bodyNode(a), Second: bodyNode(b)}
}

func (b *Body) filletPrismUnion(ctx context.Context, edges []*Edge,
	rec *prismUnionFilletRecord, radius, radiusDelta float64) (*Body, bool, error) {
	if len(edges) != 1 || radiusDelta != 0 || !(radius > 0) || proofbound.IsNonFinite(radius) {
		return nil, false, nil
	}
	for _, pair := range [][2]*Body{{bodyHandle(rec.First), bodyHandle(rec.Second)}, {bodyHandle(rec.Second), bodyHandle(rec.First)}} {
		pp, ok := pair[0].n.Payload.(prismPayload)
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
		out, err := filletPrismUnionSource(ctx, documentHandle(b.n.Doc), pp, sourceEdge, corner, radius)
		if err != nil {
			return nil, true, err
		}
		// The bounded band R removed from the exact rectangular source is
		// strictly outside the other source's true box. Thus R∩B is empty,
		// and (A\R)∪B equals (A∪B)\R. The ordinary Boolean rebuild supplies
		// its own occupied-volume and boundary certificates.
		joined, err := booleanBody(ctx, meshbool.OpUnion, out, pair[1], b.n.Doc.NextProducer)
		if err != nil {
			return nil, true, err
		}
		body, err := commitModifyResult(ctx, b, joined)
		return body, true, err
	}
	return nil, false, nil
}

// The source must be an exact, unplaced +XY rectangle. Its selected edge and
// both endpoints must be the exact world-space edge of one rectangle corner.
// The returned outward box contains the whole removed radius band, including
// every point of its circular arc, with outward-rounded dyadic limits.
func prismUnionFilletCorner(source *Body, pp prismPayload, result *Edge, radius float64) (
	sectionrecord.Point2, compositesweep.AuditBox, *Edge, bool,
) {
	if source == nil || result == nil || result.n.Start == nil || result.n.End == nil ||
		pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 || pp.SurfaceResult ||
		pp.Xform != r3.Identity() || pp.Frame.U() != (r3.Vec{X: 1}) ||
		pp.Frame.V() != (r3.Vec{Y: 1}) || pp.Frame.N() != (r3.Vec{Z: 1}) {
		return sectionrecord.Point2{}, compositesweep.AuditBox{}, nil, false
	}
	rect, ok := momentinput.AxisRectangle(pp.Profile)
	if !ok || pp.Z0 == pp.Z1 || !prismUnionFilletRadiusFits(rect, radius) {
		return sectionrecord.Point2{}, compositesweep.AuditBox{}, nil, false
	}
	curve, ok := result.n.Curve.(surfacegeom.FacetedCurve)
	if !ok || curve.Bound.Mag() != 0 || result.n.Start.Bound.Mag() != 0 || result.n.End.Bound.Mag() != 0 ||
		len(result.n.Faces) != 2 {
		return sectionrecord.Point2{}, compositesweep.AuditBox{}, nil, false
	}
	corners := [4][2]float64{{rect.U0, rect.V0}, {rect.U1, rect.V0}, {rect.U1, rect.V1}, {rect.U0, rect.V1}}
	for _, corner := range corners {
		p0, p1 := pp.Point(corner[0], corner[1], pp.Z0), pp.Point(corner[0], corner[1], pp.Z1)
		if !prismUnionFilletSameEnds(result, p0, p1) {
			continue
		}
		var sourceEdge *Edge
		for _, edge := range source.Edges() {
			if _, ok := edge.n.Curve.(surfacegeom.Line3); ok && edge.n.Start != nil && edge.n.End != nil &&
				edge.n.Start.Bound.Mag() == 0 && edge.n.End.Bound.Mag() == 0 &&
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
		band, ok := prismUnionFilletBand(pp.Frame.Origin(), corner, xOther, yOther, pp.Z0, pp.Z1, radius)
		return sectionrecord.Point2{U: corner[0], V: corner[1]}, band, sourceEdge, ok
	}
	return sectionrecord.Point2{}, compositesweep.AuditBox{}, nil, false
}

func prismUnionFilletSameEnds(edge *Edge, p, q r3.Vec) bool {
	return (edge.n.Start.Position == p && edge.n.End.Position == q) ||
		(edge.n.Start.Position == q && edge.n.End.Position == p)
}

func prismUnionFilletRadiusFits(rect momentinput.AxisRect, radius float64) bool {
	r := proofarith.FloatRat(radius)
	twice := new(big.Rat).Mul(r, big.NewRat(2, 1))
	width := new(big.Rat).Sub(proofarith.FloatRat(rect.U1), proofarith.FloatRat(rect.U0))
	height := new(big.Rat).Sub(proofarith.FloatRat(rect.V1), proofarith.FloatRat(rect.V0))
	return twice.Cmp(width) < 0 && twice.Cmp(height) < 0
}

func prismUnionFilletBand(origin r3.Vec, corner [2]float64, xOther, yOther, z0, z1, r float64) (
	compositesweep.AuditBox, bool,
) {
	axis := func(o, c, other float64) (float64, float64) {
		at := new(big.Rat).Add(proofarith.FloatRat(o), proofarith.FloatRat(c))
		inner := new(big.Rat).Set(at)
		if c < other {
			inner.Add(inner, proofarith.FloatRat(r))
		} else {
			inner.Sub(inner, proofarith.FloatRat(r))
		}
		if at.Cmp(inner) < 0 {
			return proofbound.RatFloatDown(at), proofbound.RatFloatUp(inner)
		}
		return proofbound.RatFloatDown(inner), proofbound.RatFloatUp(at)
	}
	x0, x1 := axis(origin.X, corner[0], xOther)
	y0, y1 := axis(origin.Y, corner[1], yOther)
	zA := new(big.Rat).Add(proofarith.FloatRat(origin.Z), proofarith.FloatRat(z0))
	zB := new(big.Rat).Add(proofarith.FloatRat(origin.Z), proofarith.FloatRat(z1))
	if zA.Cmp(zB) > 0 {
		zA, zB = zB, zA
	}
	zLo, zHi := proofbound.RatFloatDown(zA), proofbound.RatFloatUp(zB)
	if !proofbound.FiniteVec(r3.Vec{X: x0, Y: y0, Z: zLo}) ||
		!proofbound.FiniteVec(r3.Vec{X: x1, Y: y1, Z: zHi}) {
		return compositesweep.AuditBox{}, false
	}
	return compositesweep.AuditBox{Min: r3.Vec{X: x0, Y: y0, Z: zLo},
		Max: r3.Vec{X: x1, Y: y1, Z: zHi}}, true
}

func prismUnionFilletSourceFaces(result, source *Edge) bool {
	if len(source.n.Faces) != 2 || len(result.n.Faces) != 2 {
		return false
	}
	for _, face := range source.n.Faces {
		if !faceHandle(face).isPlanar() || len(face.Origins) != 1 || face.Origins[0].Role == provenance.RoleCapStart ||
			face.Origins[0].Role == provenance.RoleCapEnd {
			return false
		}
	}
	for _, face := range result.n.Faces {
		if len(face.Origins) != 1 || !faceHandle(face).isPlanar() {
			return false
		}
	}
	a, b := source.n.Faces[0].Origins[0], source.n.Faces[1].Origins[0]
	x, y := result.n.Faces[0].Origins[0], result.n.Faces[1].Origins[0]
	return a != b && ((a == x && b == y) || (a == y && b == x))
}

func prismUnionFilletBandSeparate(tool *Body, band compositesweep.AuditBox) bool {
	box, err := tool.Bounds()
	return err == nil && !math.IsNaN(box.Bound.Mag()) && !math.IsInf(box.Bound.Mag(), 0) &&
		compositesweep.BoxesStrictlySeparated(band, compositesweep.AuditBox{
			Min: box.Min, Max: box.Max, Bound: box.Bound.Mag(),
		})
}

func filletPrismUnionSource(ctx context.Context, d *Document, pp prismPayload,
	selected *Edge, corner sectionrecord.Point2, radius float64) (*Body, error) {
	budget := proofbound.NewWorkBudget(ctx)
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, pp.Profile)
	if err != nil {
		return nil, err
	}
	if len(loops) != 1 || len(loops[0].Walks) != 4 {
		return nil, fmt.Errorf("%w: the source rectangle has no four-corner walk", ErrUnsupported)
	}
	li, ci, found, err := matchCornerBudget(budget, pp, loops, selected)
	if err != nil {
		return nil, err
	}
	if !found || li != 0 || loops[0].Walks[ci].StartU != corner.U ||
		loops[0].Walks[ci].StartV != corner.V {
		return nil, fmt.Errorf("%w: the source edge does not match the authenticated corner", ErrUnsupported)
	}
	blend, err := computeFillet(loops[0], ci, radius)
	if err != nil {
		return nil, err
	}
	if !prismUnionFilletQuarterArc(pp, corner, radius, blend) {
		return nil, fmt.Errorf("%w: the rectangular corner has no exact inward quarter-circle", ErrUnsupported)
	}
	blends := []map[int]*cornerBlend{{ci: blend}}
	profile, arcs, err := rewriteProfileBudget(budget, pp.Profile, loops, blends)
	if err != nil {
		return nil, err
	}
	if err := offset2d.AuditRewriteBudget(budget, pp.Profile, profile, loops, blends); err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return nil, err
		}
		return nil, fmt.Errorf("%w: the rectangular source fillet failed its section audit: %v", ErrUnsupported, err)
	}
	return bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), d.nextProducerID()), prismrecord.Payload{
		Profile: profile, Frame: pp.Frame, Z0: pp.Z0, Z1: pp.Z1,
		Xform: pp.Xform, BlendSegs: arcs, BlendKind: string(breprecord.BandFillet),
	}, freeform.NewFreeformWork()))
}

// The rectangle's two source walls are perpendicular, and its material is
// inside their corner. Confirm that the rewrite's own arc is precisely the
// inward quadrant, with exactly representable centre and tangent feet.
func prismUnionFilletQuarterArc(pp prismPayload, corner sectionrecord.Point2, radius float64, blend *cornerBlend) bool {
	arc, ok := blend.Connector.(arcSeg)
	if !ok || arc.TStart != 0 || arc.TEnd != 1 || arc.Start != blend.FA || arc.End != blend.FB ||
		blend.CutbackA != radius || blend.CutbackB != radius {
		return false
	}
	rect, ok := momentinput.AxisRectangle(pp.Profile)
	if !ok {
		return false
	}
	xDir, yDir := 1, 1
	if corner.U == rect.U1 {
		xDir = -1
	}
	if corner.V == rect.V1 {
		yDir = -1
	}
	xInner, xOK := prismUnionFilletExactShift(corner.U, radius, xDir)
	yInner, yOK := prismUnionFilletExactShift(corner.V, radius, yDir)
	if !xOK || !yOK || arc.Center != (sectionrecord.Point2{U: xInner, V: yInner}) {
		return false
	}
	xFoot := sectionrecord.Point2{U: xInner, V: corner.V}
	yFoot := sectionrecord.Point2{U: corner.U, V: yInner}
	if xDir == yDir {
		return blend.FA == yFoot && blend.FB == xFoot
	}
	return blend.FA == xFoot && blend.FB == yFoot
}

func prismUnionFilletExactShift(c, radius float64, direction int) (float64, bool) {
	sum := new(big.Rat).Set(proofarith.FloatRat(c))
	if direction > 0 {
		sum.Add(sum, proofarith.FloatRat(radius))
	} else {
		sum.Sub(sum, proofarith.FloatRat(radius))
	}
	held, _ := sum.Float64()
	return held, !proofbound.IsNonFinite(held) && proofarith.FloatRat(held).Cmp(sum) == 0
}
