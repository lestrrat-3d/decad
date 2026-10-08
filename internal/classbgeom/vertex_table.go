package classbgeom

import (
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// CrossingVertex is one canonical point and the carriers it lies on.
type CrossingVertex[C comparable] struct {
	Point       [3]float64
	CarrierList []C
	Delta       float64
}

func (v *CrossingVertex[C]) Carriers() []C { return v.CarrierList }

// CarrierGeometry describes a plane or cylinder in the reference axes.
type CarrierGeometry struct {
	Cylinder bool
	Axis     int
	Level    float64
	Center   [3]float64
	Segment  sectionrecord.CurveSegment
}

type crossingKey[C comparable] struct {
	cylinder   C
	planeAxis  int
	planeLevel float64
	side       int
}

type crossingEntry struct {
	value [3]float64
	delta float64
}

// VertexTable gives every scene the same float for a cylinder-plane crossing.
type VertexTable[C comparable] struct {
	geometry func(C) CarrierGeometry
	miss     error
	table    map[crossingKey[C]]crossingEntry
	Vertices map[[3]float64]*CrossingVertex[C]
}

// NewVertexTable allocates the table for one crossing-reach attempt.
func NewVertexTable[C comparable](geometry func(C) CarrierGeometry, miss error) *VertexTable[C] {
	return &VertexTable[C]{
		geometry: geometry, miss: miss,
		table:    map[crossingKey[C]]crossingEntry{},
		Vertices: map[[3]float64]*CrossingVertex[C]{},
	}
}

// SeedRecordedCorners records every existing circle-line corner with zero
// displacement before any scene computes a new crossing of those carriers.
func (t *VertexTable[C]) SeedRecordedCorners(prisms [2]CrossingPrism,
	frames [2]CrossingFrame, carrier func(int, int) C) error {
	for op, prism := range prisms {
		segs := prism.Segments
		for i := range segs {
			j := (i + 1) % len(segs)
			ci, cj := carrier(op, i), carrier(op, j)
			gi, gj := t.geometry(ci), t.geometry(cj)
			if gi.Cylinder == gj.Cylinder {
				continue
			}
			w, err := boundarywalk.WalkOf(segs[i], nil)
			if err != nil {
				return err
			}
			cylinder, plane := ci, gj
			if gj.Cylinder {
				cylinder, plane = cj, gi
			}
			frame := frames[op]
			x := ToX([3]float64{w.EndU, w.EndV, 0}, frame.Axis, frame.Sign)
			key, ok := t.key2(cylinder, plane, x)
			if !ok {
				return t.miss
			}
			t.table[key] = crossingEntry{value: x}
		}
	}
	return nil
}

func (t *VertexTable[C]) key2(cylinder C, plane CarrierGeometry, x [3]float64) (crossingKey[C], bool) {
	g := t.geometry(cylinder)
	free := 3 - g.Axis - plane.Axis
	if free < 0 || free > 2 || plane.Axis == g.Axis {
		return crossingKey[C]{}, false
	}
	diff := x[free] - g.Center[free]
	if diff == 0 {
		return crossingKey[C]{}, false
	}
	side := 1
	if diff < 0 {
		side = -1
	}
	return crossingKey[C]{cylinder: cylinder, planeAxis: plane.Axis, planeLevel: plane.Level, side: side}, true
}

// CanonicalPoint pins a scene junction to three exact plane levels or to the
// first float recorded for its cylinder-plane crossing key.
func (t *VertexTable[C]) CanonicalPoint(f, c1, c2 C, x [3]float64, delta float64) (CrossingVertex[C], error) {
	carriers := []C{f, c1, c2}
	var planes []CarrierGeometry
	var cylinder *C
	for i, c := range carriers {
		g := t.geometry(c)
		if g.Cylinder {
			if cylinder != nil && *cylinder != c {
				return CrossingVertex[C]{}, t.miss
			}
			cylinder = &carriers[i]
			continue
		}
		planes = append(planes, g)
	}
	var out [3]float64
	switch {
	case cylinder == nil:
		seen := [3]bool{}
		for _, plane := range planes {
			if seen[plane.Axis] {
				return CrossingVertex[C]{}, t.miss
			}
			seen[plane.Axis] = true
			out[plane.Axis] = plane.Level
		}
		return CrossingVertex[C]{Point: out, CarrierList: carriers}, nil
	case len(planes) == 2:
		g := t.geometry(*cylinder)
		axial, along := planes[0], planes[1]
		if along.Axis == g.Axis {
			axial, along = along, axial
		}
		if axial.Axis != g.Axis || along.Axis == g.Axis {
			return CrossingVertex[C]{}, t.miss
		}
		free := 3 - g.Axis - along.Axis
		offset := CrossingOffsetUpper(g.Segment, x[free], g.Center[free], along.Level, g.Center[along.Axis])
		if proofbound.IsNonFinite(offset) {
			return CrossingVertex[C]{}, t.miss
		}
		delta = max(delta, offset)
		if math.Abs(x[free]-g.Center[free]) <= 2*delta {
			return CrossingVertex[C]{}, t.miss
		}
		key, ok := t.key2(*cylinder, along, x)
		if !ok {
			return CrossingVertex[C]{}, t.miss
		}
		entry, ok := t.table[key]
		if !ok {
			entry = crossingEntry{value: x, delta: delta}
			entry.value[along.Axis] = along.Level
			t.table[key] = entry
		}
		out = entry.value
		out[g.Axis] = axial.Level
		return CrossingVertex[C]{Point: out, CarrierList: carriers, Delta: entry.delta}, nil
	default:
		return CrossingVertex[C]{}, t.miss
	}
}

// Record registers a canonical vertex with every carrier it was found on.
func (t *VertexTable[C]) Record(v CrossingVertex[C]) {
	existing, ok := t.Vertices[v.Point]
	if !ok {
		c := v
		c.CarrierList = slices.Clone(v.CarrierList)
		t.Vertices[v.Point] = &c
		return
	}
	existing.Delta = max(existing.Delta, v.Delta)
	for _, c := range v.CarrierList {
		if !slices.Contains(existing.CarrierList, c) {
			existing.CarrierList = append(existing.CarrierList, c)
		}
	}
}
