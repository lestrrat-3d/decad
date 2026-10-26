// Package stepbrep reads the solids of a STEP model into a triangle mesh
// (agent-docs/step-import-design.md). It reads the faces of MANIFOLD_SOLID_BREP
// and BREP_WITH_VOIDS whose surfaces are planes, cylinders or cones and whose edges are
// lines or circles, and refuses the rest.
package stepbrep

import (
	"errors"
	"fmt"
	"math"
	"strings"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/polytri"
	"github.com/lestrrat-3d/decad/internal/stepread"
	"github.com/lestrrat-3d/r3"
)

// ErrMalformed marks a model whose entities do not form the B-rep they claim.
var ErrMalformed = errors.New("stepbrep: malformed B-rep")

// ErrTooLarge marks a model that triangulates to more than the limit.
var ErrTooLarge = errors.New("stepbrep: too many triangles")

// Mesh is the triangulated solids, millimetres, vertices welded by coordinate.
type Mesh struct {
	Vertices  []r3.Vec
	Triangles [][3]int
}

type reader struct {
	m        *stepread.Model
	scale    float64
	limit    int
	vertices []r3.Vec
	index    map[[3]float64]int
	tris     [][3]int
	// samples caches each EDGE_CURVE's points from its start to its end, so two
	// faces that share an edge place the same points on it.
	samples map[int][]r3.Vec
}

// Read triangulates every solid of the model. limit is the most triangles it
// accepts.
func Read(m *stepread.Model, limit int) (Mesh, error) {
	r := &reader{m: m, limit: limit, index: map[[3]float64]int{}, samples: map[int][]r3.Vec{}}
	scale, err := lengthScale(m)
	if err != nil {
		return Mesh{}, err
	}
	r.scale = scale
	if name := assemblyEntity(m); name != "" {
		return Mesh{}, fmt.Errorf("%w: the file places its shapes as an assembly (%s), and placements are not read, so the parts would sit at their own origins",
			decaderr.ErrUnsupported, name)
	}
	solids := 0
	for _, id := range sortedIDs(m) {
		e := m.Entities[id]
		switch e.Name {
		case "MANIFOLD_SOLID_BREP":
			if len(e.Params) < 2 {
				return Mesh{}, fmt.Errorf("%w: #%d has no shell", ErrMalformed, id)
			}
			if err := r.shell(e.Params[1], false); err != nil {
				return Mesh{}, err
			}
			solids++
		case "BREP_WITH_VOIDS":
			if len(e.Params) < 3 {
				return Mesh{}, fmt.Errorf("%w: #%d has no shells", ErrMalformed, id)
			}
			if err := r.shell(e.Params[1], false); err != nil {
				return Mesh{}, err
			}
			voids, _ := e.Params[2].(stepread.List)
			for _, v := range voids {
				if err := r.shell(v, false); err != nil {
					return Mesh{}, err
				}
			}
			solids++
		}
	}
	if solids == 0 {
		return Mesh{}, fmt.Errorf("%w: the model holds no MANIFOLD_SOLID_BREP or BREP_WITH_VOIDS", ErrMalformed)
	}
	return Mesh{Vertices: r.vertices, Triangles: r.tris}, nil
}

// assemblyEntity names the first entity that makes the file an assembly: a
// NEXT_ASSEMBLY_USAGE_OCCURRENCE, or a representation relationship with a
// transformation, a member of a complex instance as well as a simple one. It is
// empty when the file has none. The reader places no shape, so it refuses these
// rather than overlay parts that the file puts apart.
func assemblyEntity(m *stepread.Model) string {
	for _, id := range sortedIDs(m) {
		e := m.Entities[id]
		names := []string{e.Name}
		for _, p := range e.Parts {
			names = append(names, p.Name)
		}
		for _, n := range names {
			switch n {
			case "NEXT_ASSEMBLY_USAGE_OCCURRENCE", "ITEM_DEFINED_TRANSFORMATION", "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION":
				return n
			}
		}
	}
	return ""
}

func sortedIDs(m *stepread.Model) []int {
	ids := make([]int, 0, len(m.Entities))
	for id := range m.Entities {
		ids = append(ids, id)
	}
	// Insertion sort keeps the dependency-free, deterministic order small models need.
	for i := 1; i < len(ids); i++ {
		for j := i; j > 0 && ids[j-1] > ids[j]; j-- {
			ids[j-1], ids[j] = ids[j], ids[j-1]
		}
	}
	return ids
}

// lengthScale is the length of the model's unit in millimetres. The unit is the
// length unit the model's GLOBAL_UNIT_ASSIGNED_CONTEXT lists; a model with no such
// context uses its conversion-based INCH or FOOT if it names one, else its SI
// length unit, and millimetres when it names none.
func lengthScale(m *stepread.Model) (float64, error) {
	ids := sortedIDs(m)
	var global []int
	for _, id := range ids {
		for _, p := range m.Entities[id].Parts {
			if p.Name == "GLOBAL_UNIT_ASSIGNED_CONTEXT" && len(p.Params) >= 1 {
				if units, ok := p.Params[0].(stepread.List); ok {
					for _, u := range units {
						if ref, ok := u.(stepread.Ref); ok {
							global = append(global, int(ref))
						}
					}
				}
			}
		}
	}
	candidates := global
	if len(candidates) == 0 {
		candidates = ids
	}
	var si, conversion *float64
	for _, id := range candidates {
		e, ok := m.Entities[id]
		if !ok {
			continue
		}
		scale, kind, ok, err := lengthUnitScale(e)
		if err != nil {
			return 0, err
		}
		if !ok {
			continue
		}
		if kind == "conversion" && conversion == nil {
			conversion = &scale
		}
		if kind == "si" && si == nil {
			si = &scale
		}
	}
	switch {
	case len(global) > 0 && conversion != nil:
		return *conversion, nil
	case len(global) > 0 && si != nil:
		return *si, nil
	case conversion != nil:
		return *conversion, nil
	case si != nil:
		return *si, nil
	}
	return 1, nil
}

// lengthUnitScale reads a length unit instance: its millimetre scale, whether it
// is an "si" or a "conversion" unit, and whether it is a length unit at all.
func lengthUnitScale(e stepread.Entity) (scale float64, kind string, ok bool, err error) {
	prefixes := map[stepread.Enum]float64{
		"MICRO": 1e-3, "MILLI": 1, "CENTI": 10, "DECI": 100, "KILO": 1e6, "HECTO": 1e5, "DECA": 1e4,
	}
	isLength := e.Name == "LENGTH_UNIT"
	for _, p := range e.Parts {
		if p.Name == "LENGTH_UNIT" {
			isLength = true
		}
	}
	if !isLength {
		return 0, "", false, nil
	}
	for _, p := range e.Parts {
		switch p.Name {
		case "SI_UNIT":
			if len(p.Params) >= 2 {
				if prefix, isEnum := p.Params[0].(stepread.Enum); isEnum {
					s, known := prefixes[prefix]
					if !known {
						return 0, "", false, fmt.Errorf("%w: unknown length prefix %q", ErrMalformed, prefix)
					}
					return s, "si", true, nil
				}
				return 1000, "si", true, nil
			}
		case "CONVERSION_BASED_UNIT":
			if len(p.Params) >= 1 {
				if name, isString := p.Params[0].(stepread.String); isString {
					switch strings.ToUpper(string(name)) {
					case "INCH":
						return 25.4, "conversion", true, nil
					case "FOOT":
						return 304.8, "conversion", true, nil
					}
				}
			}
		}
	}
	return 0, "", false, nil
}

// shell reads the closed shell a reference names, or an oriented closed shell
// that names one.
func (r *reader) shell(v stepread.Value, flipped bool) error {
	id, ok := v.(stepread.Ref)
	if !ok {
		return fmt.Errorf("%w: a shell is not a reference", ErrMalformed)
	}
	e, ok := r.m.Entities[int(id)]
	if !ok {
		return fmt.Errorf("%w: shell #%d is not defined", ErrMalformed, id)
	}
	switch e.Name {
	case "ORIENTED_CLOSED_SHELL":
		if len(e.Params) < 4 {
			return fmt.Errorf("%w: #%d is not an oriented closed shell", ErrMalformed, id)
		}
		orientation, _ := e.Params[3].(stepread.Enum)
		return r.shell(e.Params[2], flipped != (orientation == "F"))
	case "CLOSED_SHELL", "OPEN_SHELL":
		if len(e.Params) < 2 {
			return fmt.Errorf("%w: #%d has no faces", ErrMalformed, id)
		}
		faces, _ := e.Params[1].(stepread.List)
		for _, f := range faces {
			if err := r.face(f, flipped); err != nil {
				return err
			}
		}
		return nil
	default:
		return fmt.Errorf("%w: #%d is a %s, not a shell", ErrMalformed, id, e.Name)
	}
}

func (r *reader) entity(v stepread.Value, name string) (stepread.Entity, error) {
	ref, ok := v.(stepread.Ref)
	if !ok {
		return stepread.Entity{}, fmt.Errorf("%w: a %s is not a reference", ErrMalformed, name)
	}
	e, ok := r.m.Entities[int(ref)]
	if !ok {
		return stepread.Entity{}, fmt.Errorf("%w: %s #%d is not defined", ErrMalformed, name, ref)
	}
	return e, nil
}

// point is a position in millimetres.
func (r *reader) point(v stepread.Value) (r3.Vec, error) {
	e, err := r.entity(v, "point")
	if err != nil {
		return r3.Vec{}, err
	}
	if e.Name != "CARTESIAN_POINT" || len(e.Params) < 2 {
		return r3.Vec{}, fmt.Errorf("%w: #%d is not a CARTESIAN_POINT", ErrMalformed, e.ID)
	}
	return r.coords(e.Params[1], e.ID, r.scale)
}

func (r *reader) coords(v stepread.Value, id int, scale float64) (r3.Vec, error) {
	list, ok := v.(stepread.List)
	if !ok || len(list) != 3 {
		return r3.Vec{}, fmt.Errorf("%w: #%d does not hold three coordinates", ErrMalformed, id)
	}
	var c [3]float64
	for i, x := range list {
		n, ok := x.(stepread.Number)
		if !ok || math.IsNaN(float64(n)) || math.IsInf(float64(n), 0) {
			return r3.Vec{}, fmt.Errorf("%w: #%d has a coordinate that is not a finite number", ErrMalformed, id)
		}
		c[i] = float64(n) * scale
	}
	return r3.Vec{X: c[0], Y: c[1], Z: c[2]}, nil
}

// planeNormal is the axis of the PLANE a face lies on, as a unit vector.
func (r *reader) planeNormal(v stepread.Value) (r3.Vec, error) {
	e, err := r.entity(v, "surface")
	if err != nil {
		return r3.Vec{}, err
	}
	if e.Name != "PLANE" {
		return r3.Vec{}, fmt.Errorf("%w: a face lies on a %s, and only planes are read", decaderr.ErrUnsupported, nameOf(e))
	}
	if len(e.Params) < 2 {
		return r3.Vec{}, fmt.Errorf("%w: #%d has no placement", ErrMalformed, e.ID)
	}
	placement, err := r.entity(e.Params[1], "placement")
	if err != nil {
		return r3.Vec{}, err
	}
	if placement.Name != "AXIS2_PLACEMENT_3D" || len(placement.Params) < 3 {
		return r3.Vec{}, fmt.Errorf("%w: #%d is not an AXIS2_PLACEMENT_3D", ErrMalformed, placement.ID)
	}
	axis := r3.Vec{Z: 1}
	if _, none := placement.Params[2].(stepread.Null); !none {
		dir, err := r.entity(placement.Params[2], "direction")
		if err != nil {
			return r3.Vec{}, err
		}
		if dir.Name != "DIRECTION" || len(dir.Params) < 2 {
			return r3.Vec{}, fmt.Errorf("%w: #%d is not a DIRECTION", ErrMalformed, dir.ID)
		}
		if axis, err = r.coords(dir.Params[1], dir.ID, 1); err != nil {
			return r3.Vec{}, err
		}
	}
	n, ok := axis.Normalize()
	if !ok {
		return r3.Vec{}, fmt.Errorf("%w: a plane has a zero axis", ErrMalformed)
	}
	return n, nil
}

func nameOf(e stepread.Entity) string {
	if e.Name != "" {
		return e.Name
	}
	return "complex entity"
}

// ring reads a bound's vertex positions in loop order.
func (r *reader) ring(boundValue stepread.Value) (ring []r3.Vec, outer bool, err error) {
	bound, err := r.entity(boundValue, "bound")
	if err != nil {
		return nil, false, err
	}
	if (bound.Name != "FACE_OUTER_BOUND" && bound.Name != "FACE_BOUND") || len(bound.Params) < 3 {
		return nil, false, fmt.Errorf("%w: #%d is not a face bound", ErrMalformed, bound.ID)
	}
	outer = bound.Name == "FACE_OUTER_BOUND"
	loop, err := r.entity(bound.Params[1], "loop")
	if err != nil {
		return nil, false, err
	}
	switch loop.Name {
	case "EDGE_LOOP":
		edges, _ := loop.Params[1].(stepread.List)
		for _, ev := range edges {
			points, err := r.orientedEdgePoints(ev)
			if err != nil {
				return nil, false, err
			}
			ring = append(ring, points...)
		}
	case "POLY_LOOP":
		points, _ := loop.Params[1].(stepread.List)
		for _, pv := range points {
			p, err := r.point(pv)
			if err != nil {
				return nil, false, err
			}
			ring = append(ring, p)
		}
	default:
		return nil, false, fmt.Errorf("%w: #%d is a %s, not a loop", ErrMalformed, loop.ID, loop.Name)
	}
	if orientation, _ := bound.Params[2].(stepread.Enum); orientation == "F" {
		for i, j := 0, len(ring)-1; i < j; i, j = i+1, j-1 {
			ring[i], ring[j] = ring[j], ring[i]
		}
	}
	return ring, outer, nil
}

// orientedEdgePoints returns the points of the oriented edge in the direction it
// is walked, from where it begins up to but not including where it ends: the
// next edge of the loop begins there. A line gives its start, and an arc its
// start and the samples between.
func (r *reader) orientedEdgePoints(v stepread.Value) ([]r3.Vec, error) {
	oe, err := r.entity(v, "oriented edge")
	if err != nil {
		return nil, err
	}
	if oe.Name != "ORIENTED_EDGE" || len(oe.Params) < 5 {
		return nil, fmt.Errorf("%w: #%d is not an ORIENTED_EDGE", ErrMalformed, oe.ID)
	}
	edge, err := r.entity(oe.Params[3], "edge")
	if err != nil {
		return nil, err
	}
	samples, err := r.edgeSamples(edge)
	if err != nil {
		return nil, err
	}
	out := append([]r3.Vec(nil), samples[:len(samples)-1]...)
	if orientation, _ := oe.Params[4].(stepread.Enum); orientation == "F" {
		out = append([]r3.Vec(nil), samples[1:]...)
		for i, j := 0, len(out)-1; i < j; i, j = i+1, j-1 {
			out[i], out[j] = out[j], out[i]
		}
	}
	return out, nil
}

// face triangulates one ADVANCED_FACE (or FACE_SURFACE) and adds it, flipped
// when the shell it belongs to is.
func (r *reader) face(v stepread.Value, flipped bool) error {
	f, err := r.entity(v, "face")
	if err != nil {
		return err
	}
	if (f.Name != "ADVANCED_FACE" && f.Name != "FACE_SURFACE") || len(f.Params) < 4 {
		return fmt.Errorf("%w: #%d is not an ADVANCED_FACE", ErrMalformed, f.ID)
	}
	surface, err := r.entity(f.Params[2], "surface")
	if err != nil {
		return err
	}
	if surface.Name == "CYLINDRICAL_SURFACE" || surface.Name == "CONICAL_SURFACE" {
		return r.cylinderFace(f, surface, flipped)
	}
	if surface.Name == "SPHERICAL_SURFACE" {
		return r.sphereFace(f, surface, flipped)
	}
	normal, err := r.planeNormal(f.Params[2])
	if err != nil {
		return err
	}
	if sense, _ := f.Params[3].(stepread.Enum); sense == "F" {
		normal = normal.Scale(-1)
	}
	bounds, _ := f.Params[1].(stepread.List)
	if len(bounds) == 0 {
		return fmt.Errorf("%w: face #%d has no bound", ErrMalformed, f.ID)
	}
	var outerRing []r3.Vec
	var holeRings [][]r3.Vec
	for i, bv := range bounds {
		ring, outer, err := r.ring(bv)
		if err != nil {
			return err
		}
		if len(ring) < 3 {
			return fmt.Errorf("%w: a bound of face #%d has %d vertices", ErrMalformed, f.ID, len(ring))
		}
		if outer || (i == 0 && outerRing == nil && !hasOuter(r, bounds)) {
			if outerRing != nil {
				return fmt.Errorf("%w: face #%d has two outer bounds", ErrMalformed, f.ID)
			}
			outerRing = ring
		} else {
			holeRings = append(holeRings, ring)
		}
	}
	if outerRing == nil {
		return fmt.Errorf("%w: face #%d has no outer bound", ErrMalformed, f.ID)
	}
	u, ok := perpendicular(normal)
	if !ok {
		return fmt.Errorf("%w: a face has a zero normal", ErrMalformed)
	}
	vAxis := normal.Cross(u)
	project := func(ring []r3.Vec, wantCounterClockwise bool) []polytri.Point {
		out := make([]polytri.Point, len(ring))
		for i, p := range ring {
			out[i] = polytri.Point{p.Dot(u), p.Dot(vAxis)}
		}
		area := 0.0
		for i := range out {
			j := (i + 1) % len(out)
			area += out[i][0]*out[j][1] - out[j][0]*out[i][1]
		}
		if (area > 0) != wantCounterClockwise {
			for a, b := 0, len(ring)-1; a < b; a, b = a+1, b-1 {
				ring[a], ring[b] = ring[b], ring[a]
				out[a], out[b] = out[b], out[a]
			}
		}
		return out
	}
	outer2 := project(outerRing, true)
	holes2 := make([][]polytri.Point, len(holeRings))
	for i := range holeRings {
		holes2[i] = project(holeRings[i], false)
	}
	_, tris, err := polytri.Triangulate(outer2, holes2)
	if err != nil {
		return fmt.Errorf("%w: face #%d: %s", ErrMalformed, f.ID, err)
	}
	positions := append([]r3.Vec(nil), outerRing...)
	for _, h := range holeRings {
		positions = append(positions, h...)
	}
	for _, t := range tris {
		if len(r.tris) >= r.limit {
			return fmt.Errorf("%w: more than %d triangles", ErrTooLarge, r.limit)
		}
		a, b, c := r.weld(positions[t[0]]), r.weld(positions[t[1]]), r.weld(positions[t[2]])
		if flipped {
			b, c = c, b
		}
		r.tris = append(r.tris, [3]int{a, b, c})
	}
	return nil
}

// hasOuter reports whether some bound of the face is a FACE_OUTER_BOUND.
func hasOuter(r *reader, bounds stepread.List) bool {
	for _, bv := range bounds {
		if ref, ok := bv.(stepread.Ref); ok && r.m.Entities[int(ref)].Name == "FACE_OUTER_BOUND" {
			return true
		}
	}
	return false
}

func perpendicular(n r3.Vec) (r3.Vec, bool) {
	seed := r3.Vec{X: 1}
	if math.Abs(n.X) > 0.9 {
		seed = r3.Vec{Y: 1}
	}
	return n.Cross(seed).Normalize()
}

func (r *reader) weld(p r3.Vec) int {
	key := [3]float64{p.X + 0, p.Y + 0, p.Z + 0}
	if i, ok := r.index[key]; ok {
		return i
	}
	i := len(r.vertices)
	r.index[key] = i
	r.vertices = append(r.vertices, r3.Vec{X: key[0], Y: key[1], Z: key[2]})
	return i
}
