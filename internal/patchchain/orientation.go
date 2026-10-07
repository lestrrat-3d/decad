package patchchain

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// OrientedEdge is an edge whose new patch face takes the given walk direction.
type OrientedEdge[E comparable, V comparable] struct {
	Edge       E
	Start, End V
	Forward    bool
}

// Orient walks the chosen edge senses into one connected cyclic order. Each
// selected edge's sense must already oppose its adjacent face's use. The
// chosen end of each edge must meet exactly one next start, and the walk must
// visit every edge before closing; disagreement is Table R row R18.
func Orient[E comparable, V comparable](edges []OrientedEdge[E, V], row string) ([]OrientedEdge[E, V], error) {
	nextFrom := map[V]*OrientedEdge[E, V]{}
	ends := map[E][2]V{}
	for i := range edges {
		oe := &edges[i]
		ends[oe.Edge] = [2]V{oe.Start, oe.End}
		start := oe.End
		if oe.Forward {
			start = oe.Start
		}
		if _, dup := nextFrom[start]; dup {
			return nil, fmt.Errorf(`%w: the chain's adjacent faces do not agree on its orientation (%s)`, decaderr.ErrDegenerate, row)
		}
		nextFrom[start] = oe
	}

	order := make([]OrientedEdge[E, V], 0, len(edges))
	seen := map[E]bool{}
	cur, curForward := edges[0].Edge, edges[0].Forward
	for range edges {
		if seen[cur] {
			return nil, fmt.Errorf(`%w: the chain's adjacent faces do not agree on its orientation (%s)`, decaderr.ErrDegenerate, row)
		}
		seen[cur] = true
		pair := ends[cur]
		order = append(order, OrientedEdge[E, V]{Edge: cur, Start: pair[0], End: pair[1], Forward: curForward})
		end := pair[0]
		if curForward {
			end = pair[1]
		}
		nxt, ok := nextFrom[end]
		if !ok {
			return nil, fmt.Errorf(`%w: the chain's adjacent faces do not agree on its orientation (%s)`, decaderr.ErrDegenerate, row)
		}
		cur, curForward = nxt.Edge, nxt.Forward
	}
	if cur != edges[0].Edge || len(seen) != len(edges) {
		return nil, fmt.Errorf(`%w: the chain's adjacent faces do not agree on its orientation (%s)`, decaderr.ErrDegenerate, row)
	}
	return order, nil
}

// CurveKind identifies the curve record a patch boundary can emit.
type CurveKind uint8

const (
	Line CurveKind = iota
	Circle
	Arc
	Unsupported
)

// GeometryEdge is one ordered edge of the rebuilt patch boundary.
type GeometryEdge struct {
	Start, End   r3.Vec
	Center, Axis r3.Vec
	Radius       units.Value
	Kind         CurveKind
	CurveName    string
	Forward      bool
}

// OrientedNormal reads a curved carrier axis first, then uses the straight
// chain's Newell sum. Reversing a curved edge negates its effective axis.
// For a straight closed chain, Σ Pi × Pi+1 in walk order is twice its signed
// area vector, independent of the origin; reversing the walk negates it.
// No candidate frame sign is corrected from a computed area, since that
// would conceal a wrong edge-sense decision. An exact-arm patch publishes
// this normal; a level-arm patch uses it only to choose the level normal's
// sign (docs/surface-design.md §5.2). Float operations retain their order.
func OrientedNormal(edges []GeometryEdge) (r3.Vec, error) {
	for _, edge := range edges {
		if edge.Kind != Circle && edge.Kind != Arc {
			continue
		}
		axis := edge.Axis
		if !edge.Forward {
			axis = axis.Scale(-1)
		}
		return axis, nil
	}

	var sum r3.Vec
	for _, edge := range edges {
		a, b := edge.Start, edge.End
		if !edge.Forward {
			a, b = b, a
		}
		sum = sum.Add(a.Cross(b))
	}
	if _, ok := sum.Normalize(); !ok {
		return r3.Vec{}, fmt.Errorf(`%w: a Body.Patch chain's orientation is degenerate`, decaderr.ErrDegenerate)
	}
	return sum, nil
}

// LevelNormal uses the fitted normal only to choose the recorded normal's sign.
// The published direction is the token's frame-derived normal or its exact
// negation, never the tilt or magnitude fitted to bounded vertices.
func LevelNormal(fitted, tokenNormal r3.Vec) (r3.Vec, error) {
	sign := fitted.Dot(tokenNormal)
	if sign == 0 {
		return r3.Vec{}, fmt.Errorf(`%w: a Body.Patch chain's orientation is degenerate against its own level`, decaderr.ErrDegenerate)
	}
	if sign < 0 {
		return tokenNormal.Scale(-1), nil
	}
	return tokenNormal, nil
}

// PlaneFrameFromNormal builds the plane frame from the chain's chosen normal.
// It projects a nonparallel reference into the plane with r3 operations,
// then delegates orthonormal frame construction to r3.NewFrame.
func PlaneFrameFromNormal(origin, normal r3.Vec) (r3.Frame, error) {
	n, ok := normal.Normalize()
	if !ok {
		return r3.Frame{}, fmt.Errorf(`%w: a Body.Patch chain's plane normal is degenerate`, decaderr.ErrDegenerate)
	}
	ref := r3.NewVec(1, 0, 0)
	if math.Abs(n.Dot(ref)) > 0.9 {
		ref = r3.NewVec(0, 1, 0)
	}
	u, ok := ref.Sub(n.Scale(ref.Dot(n))).Normalize()
	if !ok {
		return r3.Frame{}, fmt.Errorf(`%w: a Body.Patch chain's plane normal is degenerate`, decaderr.ErrDegenerate)
	}
	v := n.Cross(u)
	return r3.NewFrame(origin, u, v)
}

// SegmentsInFrame records each ordered edge in the chosen plane frame. When
// a curved edge's effective axis opposes the frame normal, the segment swaps
// Start and End together with TStart and TEnd. Reversing only the range would
// record the same arc backwards; swapping both describes the complementary
// arc while preserving the chain's walked endpoints (surface §5.2).
func SegmentsInFrame(frame r3.Frame, edges []GeometryEdge) ([]sectionrecord.CurveSegment, error) {
	segs := make([]sectionrecord.CurveSegment, len(edges))
	for i, edge := range edges {
		start, end := edge.Start, edge.End
		if !edge.Forward {
			start, end = end, start
		}
		p2Start := point2(frame, start)
		p2End := point2(frame, end)
		switch edge.Kind {
		case Line:
			segs[i] = sectionrecord.LineSeg{Start: p2Start, End: p2End, TStart: 0, TEnd: 1}
		case Circle:
			axis := edge.Axis
			if !edge.Forward {
				axis = axis.Scale(-1)
			}
			center := point2(frame, edge.Center)
			if axis.Dot(frame.N()) > 0 {
				segs[i] = sectionrecord.CircleSeg{Center: center, Radius: edge.Radius, CCW: true, TStart: 0, TEnd: 1}
			} else {
				segs[i] = sectionrecord.CircleSeg{Center: center, Radius: edge.Radius, CCW: false, TStart: 1, TEnd: 0}
			}
		case Arc:
			axis := edge.Axis
			if !edge.Forward {
				axis = axis.Scale(-1)
			}
			center := point2(frame, edge.Center)
			if axis.Dot(frame.N()) > 0 {
				segs[i] = sectionrecord.ArcSeg{Center: center, Start: p2Start, End: p2End, TStart: 0, TEnd: 1}
			} else {
				segs[i] = sectionrecord.ArcSeg{Center: center, Start: p2End, End: p2Start, TStart: 1, TEnd: 0}
			}
		default:
			return nil, fmt.Errorf(`%w: Body.Patch cannot represent curve kind %s`, decaderr.ErrUnsupported, edge.CurveName)
		}
	}
	return segs, nil
}

func point2(frame r3.Frame, p r3.Vec) sectionrecord.Point2 {
	local := frame.ToLocal(p)
	return sectionrecord.Point2{U: local.X, V: local.Y}
}
