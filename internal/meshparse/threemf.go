package meshparse

import (
	"archive/zip"
	"bytes"
	"context"
	"encoding/xml"
	"errors"
	"fmt"
	"io"
	"math"
	"strconv"
	"strings"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// threeMFUnits is the length of each 3MF unit in millimetres.
var threeMFUnits = map[string]float64{
	"micron": 1e-3, "millimeter": 1, "centimeter": 10, "meter": 1000, "inch": 25.4, "foot": 304.8,
}

// maxComponentDepth bounds how deep components may nest.
const maxComponentDepth = 16

// threeMFObject is one <object> of the model: a mesh, or components that place
// other objects.
type threeMFObject struct {
	typ        string
	vertices   []r3.Vec
	triangles  [][3]int
	components []threeMFComponent
	hasMesh    bool
}

type threeMFComponent struct {
	object    string
	transform r3.Transform
}

type threeMFItem struct {
	object    string
	transform r3.Transform
}

// ThreeMF reads a 3MF package into a welded mesh. Every build item that is not
// a support is read: its mesh, or the meshes of its components, moved by the
// transforms on the item and its components and scaled from the model's unit to
// millimetres. Vertices with identical coordinates are welded, as [STL] does.
//
// A transform must be rigid: a rotation, a translation, or a reflection. One
// that scales or shears is [decaderr.ErrUnsupported]. Materials, colours,
// textures, slices, beam lattices and other extensions are ignored. ctx and r
// MUST NOT be nil. A package that is not a well-formed 3MF model is
// [ErrMalformed] and one past a limit is [ErrTooLarge].
func ThreeMF(ctx context.Context, r io.Reader, l Limits) (Mesh, error) {
	data, err := readAll(r, l)
	if err != nil {
		return Mesh{}, err
	}
	zr, err := zip.NewReader(bytes.NewReader(data), int64(len(data)))
	if err != nil {
		return Mesh{}, fmt.Errorf("3MF: %w: not a zip package: %s", ErrMalformed, err)
	}
	model, err := readModelPart(zr, l)
	if err != nil {
		return Mesh{}, err
	}
	scale, objects, items, err := parseModel(ctx, model, l)
	if err != nil {
		return Mesh{}, err
	}
	if len(items) == 0 {
		return Mesh{}, fmt.Errorf("3MF: %w: the build holds no item", ErrMalformed)
	}
	w := newWelder()
	var triangles [][3]int
	for _, item := range items {
		if err := ctx.Err(); err != nil {
			return Mesh{}, err
		}
		if err := emit(ctx, objects, item.object, item.transform, scale, 0, w, &triangles, l); err != nil {
			return Mesh{}, err
		}
	}
	if len(triangles) == 0 {
		return Mesh{}, fmt.Errorf("3MF: %w: the build holds no triangle", ErrMalformed)
	}
	return w.mesh(triangles), nil
}

// emit adds the triangles of the object id, moved by the transform to its
// parent's frame. parent maps the object's frame to the item's.
func emit(ctx context.Context, objects map[string]*threeMFObject, id string, parent r3.Transform, scale float64,
	depth int, w *welder, triangles *[][3]int, l Limits) error {
	if depth > maxComponentDepth {
		return fmt.Errorf("3MF: %w: components nest deeper than %d", ErrMalformed, maxComponentDepth)
	}
	obj, ok := objects[id]
	if !ok {
		return fmt.Errorf("3MF: %w: object %q is not defined", ErrMalformed, id)
	}
	if obj.typ == "support" || obj.typ == "solidsupport" {
		return nil
	}
	if obj.hasMesh {
		index := make([]int, len(obj.vertices))
		for i, v := range obj.vertices {
			p := parent.Apply(v)
			index[i] = w.add(r3.Vec{X: p.X * scale, Y: p.Y * scale, Z: p.Z * scale})
		}
		for _, t := range obj.triangles {
			if len(*triangles) >= l.Triangles {
				return fmt.Errorf("3MF: %w: more than %d triangles", ErrTooLarge, l.Triangles)
			}
			tri := [3]int{index[t[0]], index[t[1]], index[t[2]]}
			if parent.IsReflection() {
				tri[1], tri[2] = tri[2], tri[1]
			}
			*triangles = append(*triangles, tri)
		}
	}
	for _, c := range obj.components {
		if err := ctx.Err(); err != nil {
			return err
		}
		next, err := c.transform.Then(parent)
		if err != nil {
			return fmt.Errorf("3MF: %w: a component transform is not finite", ErrMalformed)
		}
		if err := emit(ctx, objects, c.object, next, scale, depth+1, w, triangles, l); err != nil {
			return err
		}
	}
	return nil
}

// readModelPart opens the package's model part: the target of the 3D model
// relationship in _rels/.rels, or 3D/3dmodel.model when there is none.
func readModelPart(zr *zip.Reader, l Limits) ([]byte, error) {
	files := map[string]*zip.File{}
	for _, f := range zr.File {
		files[strings.TrimPrefix(f.Name, "/")] = f
	}
	name := "3D/3dmodel.model"
	if rels, ok := files["_rels/.rels"]; ok {
		raw, err := readPart(rels, l)
		if err != nil {
			return nil, err
		}
		var parsed struct {
			Relationships []struct {
				Type   string `xml:"Type,attr"`
				Target string `xml:"Target,attr"`
			} `xml:"Relationship"`
		}
		if err := xml.Unmarshal(raw, &parsed); err != nil {
			return nil, fmt.Errorf("3MF: %w: _rels/.rels: %s", ErrMalformed, err)
		}
		for _, rel := range parsed.Relationships {
			if strings.HasSuffix(rel.Type, "/3dmodel") {
				name = strings.TrimPrefix(rel.Target, "/")
			}
		}
	}
	part, ok := files[name]
	if !ok {
		return nil, fmt.Errorf("3MF: %w: the package has no model part %q", ErrMalformed, name)
	}
	return readPart(part, l)
}

// readPart reads one zip entry, refusing a part that expands past the byte limit.
func readPart(f *zip.File, l Limits) ([]byte, error) {
	rc, err := f.Open()
	if err != nil {
		return nil, fmt.Errorf("3MF: %w: %s: %s", ErrMalformed, f.Name, err)
	}
	defer func() { _ = rc.Close() }()
	data, err := io.ReadAll(io.LimitReader(rc, l.Bytes+1))
	if err != nil {
		return nil, fmt.Errorf("3MF: %w: %s: %s", ErrMalformed, f.Name, err)
	}
	if int64(len(data)) > l.Bytes {
		return nil, fmt.Errorf("3MF: %w: part %s expands past %d bytes", ErrTooLarge, f.Name, l.Bytes)
	}
	return data, nil
}

// parseModel reads the model part's unit scale, objects and build items.
func parseModel(ctx context.Context, data []byte, l Limits) (float64, map[string]*threeMFObject, []threeMFItem, error) {
	dec := xml.NewDecoder(bytes.NewReader(data))
	scale := 1.0
	objects := map[string]*threeMFObject{}
	var items []threeMFItem
	var current *threeMFObject
	inMesh, triangleCount := false, 0
	attr := func(se xml.StartElement, name string) string {
		for _, a := range se.Attr {
			if a.Name.Local == name {
				return a.Value
			}
		}
		return ""
	}
	number := func(se xml.StartElement, name string) (float64, error) {
		f, err := strconv.ParseFloat(attr(se, name), 64)
		if err != nil || math.IsNaN(f) || math.IsInf(f, 0) {
			return 0, fmt.Errorf("3MF: %w: %s=%q is not a finite number", ErrMalformed, name, attr(se, name))
		}
		return f, nil
	}
	for step := 0; ; step++ {
		if step%4096 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, nil, nil, err
			}
		}
		tok, err := dec.Token()
		if errors.Is(err, io.EOF) {
			break
		}
		if err != nil {
			return 0, nil, nil, fmt.Errorf("3MF: %w: the model is not well-formed XML: %s", ErrMalformed, err)
		}
		switch t := tok.(type) {
		case xml.StartElement:
			switch t.Name.Local {
			case "model":
				if u := attr(t, "unit"); u != "" {
					s, ok := threeMFUnits[u]
					if !ok {
						return 0, nil, nil, fmt.Errorf("3MF: %w: unknown unit %q", ErrMalformed, u)
					}
					scale = s
				}
			case "object":
				id := attr(t, "id")
				if id == "" {
					return 0, nil, nil, fmt.Errorf("3MF: %w: an object has no id", ErrMalformed)
				}
				current = &threeMFObject{typ: attr(t, "type")}
				objects[id] = current
			case "mesh":
				if current == nil {
					return 0, nil, nil, fmt.Errorf("3MF: %w: a mesh outside an object", ErrMalformed)
				}
				inMesh, current.hasMesh = true, true
			case "vertex":
				if current == nil || !inMesh {
					return 0, nil, nil, fmt.Errorf("3MF: %w: a vertex outside a mesh", ErrMalformed)
				}
				var c [3]float64
				for k, n := range []string{"x", "y", "z"} {
					if c[k], err = number(t, n); err != nil {
						return 0, nil, nil, err
					}
				}
				current.vertices = append(current.vertices, r3.Vec{X: c[0], Y: c[1], Z: c[2]})
			case "triangle":
				if current == nil || !inMesh {
					return 0, nil, nil, fmt.Errorf("3MF: %w: a triangle outside a mesh", ErrMalformed)
				}
				triangleCount++
				if triangleCount > l.Triangles {
					return 0, nil, nil, fmt.Errorf("3MF: %w: more than %d triangles", ErrTooLarge, l.Triangles)
				}
				var tri [3]int
				for k, n := range []string{"v1", "v2", "v3"} {
					i, err := strconv.Atoi(attr(t, n))
					if err != nil || i < 0 || i >= len(current.vertices) {
						return 0, nil, nil, fmt.Errorf("3MF: %w: triangle index %s=%q names no vertex", ErrMalformed, n, attr(t, n))
					}
					tri[k] = i
				}
				current.triangles = append(current.triangles, tri)
			case "component":
				if current == nil {
					return 0, nil, nil, fmt.Errorf("3MF: %w: a component outside an object", ErrMalformed)
				}
				tr, err := parseTransform(attr(t, "transform"))
				if err != nil {
					return 0, nil, nil, err
				}
				current.components = append(current.components, threeMFComponent{object: attr(t, "objectid"), transform: tr})
			case "item":
				tr, err := parseTransform(attr(t, "transform"))
				if err != nil {
					return 0, nil, nil, err
				}
				items = append(items, threeMFItem{object: attr(t, "objectid"), transform: tr})
			}
		case xml.EndElement:
			switch t.Name.Local {
			case "object":
				current = nil
			case "mesh":
				inMesh = false
			}
		}
	}
	return scale, objects, items, nil
}

// parseTransform reads a 3MF transform attribute: twelve numbers, the rows of a
// 3 x 3 matrix then the translation, for a point as a row vector. An empty
// attribute is the identity. A transform that is not rigid is ErrUnsupported.
func parseTransform(s string) (r3.Transform, error) {
	if strings.TrimSpace(s) == "" {
		return r3.Identity(), nil
	}
	fields := strings.Fields(s)
	if len(fields) != 12 {
		return r3.Transform{}, fmt.Errorf("3MF: %w: a transform holds 12 numbers, got %d", ErrMalformed, len(fields))
	}
	var m [12]float64
	for i, f := range fields {
		v, err := strconv.ParseFloat(f, 64)
		if err != nil || math.IsNaN(v) || math.IsInf(v, 0) {
			return r3.Transform{}, fmt.Errorf("3MF: %w: %q is not a finite number", ErrMalformed, f)
		}
		m[i] = v
	}
	t, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{X: m[0], Y: m[1], Z: m[2]},
		EY: r3.Vec{X: m[3], Y: m[4], Z: m[5]},
		EZ: r3.Vec{X: m[6], Y: m[7], Z: m[8]},
	}, r3.Vec{X: m[9], Y: m[10], Z: m[11]})
	if err != nil {
		return r3.Transform{}, fmt.Errorf("3MF: %w: a transform that scales or shears is not read: %s", decaderr.ErrUnsupported, err)
	}
	return t, nil
}
