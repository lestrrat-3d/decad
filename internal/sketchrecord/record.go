package sketchrecord

import (
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/geom"
	"github.com/lestrrat-3d/units"
)

type (
	Point2           = sectionrecord.Point2
	LoopRecord       = sectionrecord.LoopRecord
	CurveSegment     = sectionrecord.CurveSegment
	LineSeg          = sectionrecord.LineSeg
	CircleSeg        = sectionrecord.CircleSeg
	ArcSeg           = sectionrecord.ArcSeg
	EllipseSeg       = sectionrecord.EllipseSeg
	EllipticalArcSeg = sectionrecord.EllipticalArcSeg
	ConicSeg         = sectionrecord.ConicSeg
	SplineSeg        = sectionrecord.SplineSeg
	ClosedSplineSeg  = sectionrecord.ClosedSplineSeg
	FitSplineSeg     = sectionrecord.FitSplineSeg
	NURBSSeg         = sectionrecord.NURBSSeg
)

var ErrUnrecordableProfile = decaderr.ErrUnrecordableProfile

// The root package uses these entry points for public recording and private
// boolean and surface operations.
func RecordEdge(edge sketch.BoundaryEdge) (CurveSegment, error) { return recordEdge(edge) }
func EdgeJoin(edge sketch.BoundaryEdge, segment CurveSegment) (LoopJoin, error) {
	return edgeJoin(edge, segment)
}
func FalsifyLoopJoins(name string, joins []LoopJoin) error { return falsifyLoopJoins(name, joins) }
func FalsifyChainJoins(joins []LoopJoin) error             { return falsifyChainJoins(joins) }

// recordChainSegments converts one authenticated chain's walk, edge by edge,
// in walk order, then disproves a walk whose recorded segments do not join at
// an INTERIOR junction (falsifyChainJoins) — the chain's own counterpart of
// recordLoop, which additionally checks the junction that closes a loop. A
// chain's two free ends state no junction to check at all.
func recordChainSegments(edges []sketch.BoundaryEdge) ([]CurveSegment, error) {
	segs := make([]CurveSegment, 0, len(edges))
	joins := make([]LoopJoin, 0, len(edges))
	for i, e := range edges {
		seg, err := recordEdge(e)
		if err != nil {
			return nil, fmt.Errorf("chain edge %d: %w", i, err)
		}
		join, err := edgeJoin(e, seg)
		if err != nil {
			return nil, fmt.Errorf("chain edge %d: %w", i, err)
		}
		segs = append(segs, seg)
		joins = append(joins, join)
	}
	if err := falsifyChainJoins(joins); err != nil {
		return nil, err
	}
	return segs, nil
}

// recordLoop converts one named boundary loop, edge by edge, in walk order,
// then disproves a loop whose recorded segments do not join (falsifyLoopJoins).
func recordLoop(name string, edges []sketch.BoundaryEdge) (LoopRecord, error) {
	segs := make([]CurveSegment, 0, len(edges))
	joins := make([]LoopJoin, 0, len(edges))
	for i, e := range edges {
		seg, err := recordEdge(e)
		if err != nil {
			return LoopRecord{}, fmt.Errorf("%s edge %d: %w", name, i, err)
		}
		join, err := edgeJoin(e, seg)
		if err != nil {
			return LoopRecord{}, fmt.Errorf("%s edge %d: %w", name, i, err)
		}
		segs = append(segs, seg)
		joins = append(joins, join)
	}
	if err := falsifyLoopJoins(name, joins); err != nil {
		return LoopRecord{}, err
	}
	return LoopRecord{Segments: segs}, nil
}

// recordEdge converts one boundary edge into its entity's own variant.
//
// Admission (docs/sketch-seam-design.md §1): a whole edge records from the
// entity's own defining data — TExact is never consulted, because there is no
// trim to certify (the whole *EllipticalArc edge's contingent flag is a fact
// about its pinned ends, not topology distrust). A Partial fragment records
// exactly when sketch certifies its cut — TExact — and the certified range is
// then checked by the reject-only falsifier. The entity kind never decides
// admission; it only selects the variant.
func recordEdge(e sketch.BoundaryEdge) (CurveSegment, error) {
	if e.Partial {
		if !e.TExact {
			return nil, fmt.Errorf(`%w: a %T fragment has an uncertified trim (TExact = false); an uncertified range is never recorded as an exact trim`, ErrUnrecordableProfile, e.Entity)
		}
		if err := falsifyRange(e); err != nil {
			return nil, err
		}
	}

	// Reversed is baked into the segment as the order of its range — TStart
	// and TEnd swapped, so TStart > TEnd says the walk runs against the
	// curve's natural sense — and a closed kind's CCW flips with it. The
	// entity's fields are never reordered.
	t0, t1 := e.TStart, e.TEnd
	ccw := true
	if e.Reversed {
		t0, t1 = t1, t0
		ccw = false
	}

	switch ent := e.Entity.(type) {
	case *sketch.Line:
		g := ent.Geometry()
		return LineSeg{Start: point2(g.Start), End: point2(g.End), TStart: t0, TEnd: t1}, nil
	case *sketch.Circle:
		g := ent.Geometry()
		return CircleSeg{Center: point2(g.Center), Radius: units.Millimeters(g.Radius), CCW: ccw, TStart: t0, TEnd: t1}, nil
	case *sketch.Arc:
		g := ent.Geometry()
		return ArcSeg{Center: point2(g.Center), Start: point2(g.Start), End: point2(g.End), TStart: t0, TEnd: t1}, nil
	case *sketch.Ellipse:
		g := ent.Geometry()
		return EllipseSeg{
			Center: point2(g.Center),
			Rx:     units.Millimeters(g.Rx), Ry: units.Millimeters(g.Ry), Rotation: units.Radians(g.Rotation),
			CCW: ccw, TStart: t0, TEnd: t1,
		}, nil
	case *sketch.EllipticalArc:
		g := ent.Geometry()
		return EllipticalArcSeg{
			Center: point2(g.Center), Start: point2(g.Start), End: point2(g.End),
			Rx: units.Millimeters(g.Rx), Ry: units.Millimeters(g.Ry), Rotation: units.Radians(g.Rotation),
			TStart: t0, TEnd: t1,
		}, nil
	case *sketch.Conic:
		g := ent.Geometry()
		return ConicSeg{Start: point2(g.Start), Apex: point2(g.Apex), End: point2(g.End), Rho: g.Rho, TStart: t0, TEnd: t1}, nil
	case *sketch.Spline:
		g := ent.Geometry()
		return SplineSeg{Control: points2(g.Control), TStart: t0, TEnd: t1}, nil
	case *sketch.ClosedSpline:
		g := ent.Geometry()
		return ClosedSplineSeg{Control: points2(g.Control), CCW: ccw, TStart: t0, TEnd: t1}, nil
	case *sketch.FitSpline:
		g := ent.Geometry()
		return FitSplineSeg{Fit: points2(g.Fit), TStart: t0, TEnd: t1}, nil
	case *sketch.NURBS:
		g := ent.Geometry()
		return NURBSSeg{
			Degree:  g.Degree,
			Control: points2(g.Control),
			Knots:   slices.Clone(g.Knots),
			Weights: slices.Clone(g.Weights),
			TStart:  t0, TEnd: t1,
		}, nil
	default:
		return nil, fmt.Errorf(`%w: no CurveSegment variant records a %T; a new entity kind upstream needs a new variant before decad accepts a profile that uses it`, ErrUnrecordableProfile, e.Entity)
	}
}

// point2 converts a plane-local geom point to its record form.
func point2(p *geom.Point) Point2 { return Point2{U: p.X, V: p.Y} }

// points2 converts a slice of plane-local geom points to record form.
func points2(ps []*geom.Point) []Point2 {
	out := make([]Point2, len(ps))
	for i, p := range ps {
		out[i] = point2(p)
	}
	return out
}
