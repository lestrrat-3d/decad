package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/holeplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// bossShape is the non-round section of a boss in plain millimetres and radians.
type bossShape struct {
	// kind is one of the bossRect, bossSlot and bossPoly sections.
	kind int
	// width and height are a rectangle's size, and a slot's width and length;
	// flats and sides a polygon's; angle the turn about the boss axis.
	width, height, flats, angle float64
	sides                       int
}

const (
	bossRect = iota
	bossSlot
	bossPoly
)

// readBossShape reads a HoleShape as the section of a boss.
func readBossShape(shape HoleShape) (*bossShape, error) {
	length := func(v units.Value, what string) (float64, error) {
		m, err := sectionrecord.MagnitudeIn(v, units.Length, units.Millimeter, what)
		if err == nil && m == 0 {
			err = fmt.Errorf(`%w: %s must be positive`, ErrDegenerate, what)
		}
		return m, err
	}
	turn := func(v units.Value, what string) (float64, error) {
		return sectionrecord.MagnitudeIn(v, units.Angle, units.Radian, what)
	}
	switch s := shape.(type) {
	case RectangularHole:
		w, err := length(s.Width, "the rectangular boss width")
		if err != nil {
			return nil, err
		}
		h, err := length(s.Height, "the rectangular boss height")
		if err != nil {
			return nil, err
		}
		a, err := turn(s.Angle, "the rectangular boss angle")
		if err != nil {
			return nil, err
		}
		return &bossShape{kind: bossRect, width: w, height: h, angle: a}, nil
	case SlotHole:
		l, err := length(s.Length, "the slot boss length")
		if err != nil {
			return nil, err
		}
		w, err := length(s.Width, "the slot boss width")
		if err != nil {
			return nil, err
		}
		a, err := turn(s.Angle, "the slot boss angle")
		if err != nil {
			return nil, err
		}
		if l <= w {
			return nil, fmt.Errorf(`%w: the slot boss length %g mm must exceed its width %g mm`, ErrDegenerate, l, w)
		}
		return &bossShape{kind: bossSlot, width: w, height: l, angle: a}, nil
	case PolygonHole:
		f, err := length(s.AcrossFlats, "the polygonal boss width across flats")
		if err != nil {
			return nil, err
		}
		a, err := turn(s.Angle, "the polygonal boss angle")
		if err != nil {
			return nil, err
		}
		if s.Sides < 3 || s.Sides > holeplan.MaxPolygonSides {
			return nil, fmt.Errorf(`%w: a polygonal boss has from 3 to %d sides, got %d`, ErrDegenerate, holeplan.MaxPolygonSides, s.Sides)
		}
		return &bossShape{kind: bossPoly, flats: f, sides: s.Sides, angle: a}, nil
	default:
		return nil, fmt.Errorf(`%w: unknown boss shape %T`, ErrDegenerate, shape)
	}
}

// across is the section's smallest width, which a pilot must fit inside.
func (b *bossShape) across() float64 {
	switch b.kind {
	case bossRect:
		return math.Min(b.width, b.height)
	case bossSlot:
		return b.width
	default:
		return b.flats
	}
}

// sketchSection draws the section about (u, v) in s, each side moved out by grow
// millimetres (inward when negative), as a closed loop of fixed points.
func (b *bossShape) sketchSection(s *sketch.Sketch, u, v, grow float64) {
	sin, cos := math.Sincos(b.angle)
	at := func(along, across float64) *sketch.Point {
		pt := s.CreatePoint(u+along*cos-across*sin, v+along*sin+across*cos)
		s.Fix(pt)
		return pt
	}
	polygon := func(corners [][2]float64) {
		points := make([]*sketch.Point, len(corners))
		for i, c := range corners {
			points[i] = at(c[0], c[1])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
	}
	switch b.kind {
	case bossRect:
		hw, hh := b.width/2+grow, b.height/2+grow
		polygon([][2]float64{{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}})
	case bossPoly:
		radius := (b.flats/2 + grow) / math.Cos(math.Pi/float64(b.sides))
		corners := make([][2]float64, b.sides)
		for i := range corners {
			sn, cs := math.Sincos(math.Pi/float64(b.sides) + 2*math.Pi*float64(i)/float64(b.sides))
			corners[i] = [2]float64{radius * cs, radius * sn}
		}
		polygon(corners)
	default:
		half, radius := (b.height-b.width)/2, b.width/2+grow
		start, end := at(-half, 0), at(half, 0)
		p0, p1, p2, p3 := at(-half, -radius), at(half, -radius), at(half, radius), at(-half, radius)
		s.CreateLine(p0, p1)
		s.CreateArc(end, p1, p2)
		s.CreateLine(p2, p3)
		s.CreateArc(start, p3, p0)
	}
}
