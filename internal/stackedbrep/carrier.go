package stackedbrep

import (
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

type (
	Point2       = sectionrecord.Point2
	CurveSegment = sectionrecord.CurveSegment
	LineSeg      = sectionrecord.LineSeg
	CircleSeg    = sectionrecord.CircleSeg
	ArcSeg       = sectionrecord.ArcSeg
	LoopRecord   = sectionrecord.LoopRecord
)

const (
	PlaneCarrier  = ubPlane
	LineCarrier   = ubLine
	CircleCarrier = ubCircle
)

// ubKind names what a segment's carrier is.
const (
	ubPlane  = iota // an axis-aligned line: the plane axis == level, axis 0 for U and 1 for V
	ubLine          // an oblique line, keyed by its two recorded endpoints
	ubCircle        // a circle or arc, keyed by its centre and radius
)

// Carrier identifies the surface a segment lies on, from the segment's
// recorded data alone: a line by its recorded endpoints (an axis-aligned
// one by its plane), a circle or arc by its centre and radius.
type Carrier struct {
	Kind  int
	Axis  int
	Level float64
	A, B  Point2
	R     float64
}

// ubEnd is one walked end of a segment and how far it can sit from the point
// it denotes: the walk's own rounding, plus the cut displacement when the
// parameter there is one the arrangement computed. seg is the segment it
// ends, whose carrier a junction's crossing offset reads (crossingOffset),
// and cut marks a parameter the arrangement computed.
type ubEnd struct {
	p     Point2
	allow float64
	seg   CurveSegment
	cut   bool
}

// crossingOffset is the proven distance from a junction's canonical point p
// to the exact crossing of the carriers of the two segments meeting there
// (brepgeom.CrossingOffsetUpper), where either side is a cut: the arrangement
// placed p through a cut parameter that, far from the plane origin, is off
// the crossing by the coordinates' own rounding rather than within the cut
// allowance. A recorded corner, both sides at their natural ends, is the
// record's own point and is charged nothing. +Inf where the carriers state no
// crossing.
func crossingOffset(ei, sj ubEnd, p Point2) float64 {
	if !ei.cut && !sj.cut {
		return 0
	}
	return brepgeom.CrossingOffsetUpper(ei.seg, sj.seg, p)
}

// Unit is one maximal run of a loop's consecutive segments on one carrier,
// walked in one sense, between its two canonical vertices.
type Unit struct {
	Carrier  Carrier
	CCW      bool
	From, To Point2
}

// Loop is a loop restated as units; closed marks one whole circle.
type Loop struct {
	Units  []Unit
	Closed bool
}

// Face is one horizontal planar face before its record is written: a
// loop at a level, facing up (outward) or down.
type Face struct {
	Loop    Loop
	Level   int
	Outward bool
}

// ubVertex is one canonical vertex of the section: the carriers it lies on
// and its proven displacement from the point it denotes.
type ubVertex struct {
	carriers []Carrier
	delta    float64
}

// ubKey keys a crossing of two carriers; side tells the two crossings of a
// line with a circle apart.
type ubKey struct {
	c1, c2 Carrier
	side   int
}

type ubEntry struct {
	p     Point2
	delta float64
}

// Engine restates stacked-union loops and shares canonical vertices across them.
type Engine struct {
	LevelAt   map[float64]int
	Events    map[Point2]map[int]struct{}
	Faces     []Face
	SlabLoops [][]Loop
	allow     float64
	table     map[ubKey]ubEntry
	verts     map[Point2]*ubVertex
	junctions map[Point2]map[int][2]Carrier
}

// NewEngine creates one record builder for the supplied levels.
func NewEngine(levels []float64) *Engine {
	b := &Engine{
		LevelAt:   map[float64]int{},
		Events:    map[Point2]map[int]struct{}{},
		table:     map[ubKey]ubEntry{},
		verts:     map[Point2]*ubVertex{},
		junctions: map[Point2]map[int][2]Carrier{},
	}
	for i, level := range levels {
		b.LevelAt[level] = i
	}
	return b
}

// Allow is the largest canonical-vertex displacement recorded by the builder.
func (b *Engine) Allow() float64 { return b.allow }

// carrierOf reads a segment's carrier from its record and walk.
func ubCarrierOf(seg CurveSegment, w survey2d.SegmentWalk) (Carrier, error) {
	switch s := seg.(type) {
	case LineSeg:
		switch {
		case s.Start.U == s.End.U && s.Start.V == s.End.V:
			return Carrier{}, brepgeom.ErrStackedWallMiss
		case s.Start.U == s.End.U:
			return Carrier{Kind: ubPlane, Axis: 0, Level: s.Start.U + 0}, nil
		case s.Start.V == s.End.V:
			return Carrier{Kind: ubPlane, Axis: 1, Level: s.Start.V + 0}, nil
		}
		a, c := s.Start, s.End
		if c.U < a.U || (c.U == a.U && c.V < a.V) {
			a, c = c, a
		}
		return Carrier{Kind: ubLine, A: a, B: c}, nil
	case CircleSeg, ArcSeg:
		if !w.IsCircular() {
			return Carrier{}, brepgeom.ErrStackedWallMiss
		}
		return Carrier{Kind: ubCircle, A: Point2{U: w.CU + 0, V: w.CV + 0}, R: w.Radius}, nil
	}
	return Carrier{}, brepgeom.ErrStackedWallMiss
}

// ubEnds reads a segment's two walked ends with their allowances.
func ubEnds(seg CurveSegment, w survey2d.SegmentWalk) (ubEnd, ubEnd, error) {
	t0, t1, err := ubRange(seg)
	if err != nil {
		return ubEnd{}, ubEnd{}, err
	}
	cut := 0.0
	if (t0 != 0 && t0 != 1) || (t1 != 0 && t1 != 1) {
		speed, err := prismcells.CarrierSpeedUpper(seg)
		if err != nil {
			return ubEnd{}, ubEnd{}, err
		}
		cut = proofbound.CutDisplacementAllow(speed)
	}
	start := ubEnd{p: Point2{U: w.StartU + 0, V: w.StartV + 0}, allow: proofbound.WalkEndBoundAllow(w.StartBound), seg: seg}
	end := ubEnd{p: Point2{U: w.EndU + 0, V: w.EndV + 0}, allow: proofbound.WalkEndBoundAllow(w.EndBound), seg: seg}
	if t0 != 0 && t0 != 1 {
		start.allow = proofbound.AbsSumUpper(start.allow, cut)
		start.cut = true
	}
	if t1 != 0 && t1 != 1 {
		end.allow = proofbound.AbsSumUpper(end.allow, cut)
		end.cut = true
	}
	return start, end, nil
}

func ubRange(seg CurveSegment) (float64, float64, error) {
	switch s := seg.(type) {
	case LineSeg:
		return s.TStart, s.TEnd, nil
	case CircleSeg:
		return s.TStart, s.TEnd, nil
	case ArcSeg:
		return s.TStart, s.TEnd, nil
	}
	return 0, 0, brepgeom.ErrStackedWallMiss
}

// ubLess orders carriers so a pair keys the table one way.
func ubLess(a, b Carrier) bool {
	keys := func(c Carrier) [8]float64 {
		return [8]float64{float64(c.Kind), float64(c.Axis), c.Level, c.A.U, c.A.V, c.B.U, c.B.V, c.R}
	}
	ka, kb := keys(a), keys(b)
	for i := range ka {
		if ka[i] != kb[i] {
			return ka[i] < kb[i]
		}
	}
	return false
}

func ubPair(a, b Carrier) [2]Carrier {
	if ubLess(b, a) {
		return [2]Carrier{b, a}
	}
	return [2]Carrier{a, b}
}
